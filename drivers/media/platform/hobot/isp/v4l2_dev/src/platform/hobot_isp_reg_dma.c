/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include <linux/kernel.h> /* //printk() */
#include <asm/uaccess.h>
#include <linux/gfp.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <asm/types.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/time.h>
#include "system_dma.h"
#include "hobot_isp_reg_dma.h"

#if FW_USE_HOBOT_DMA

#define HOBOT_DMA_RESULT_CHK     0
#define HOBOT_DMA_DEBUG          1
#if HOBOT_DMA_DEBUG
    #define HOBOT_DMA_LOG(...)   printk(__VA_ARGS__)
#else
    #define HOBOT_DMA_LOG(...)
#endif

#define ISP_AREA1       0x1AAF8
#define ISP_AREA2       0x32AB8
#define TEST_SIZE       64

#define DUMP_DMA(name, addr, size)          \
        for(i=0;i<(size>>2);i+=4)  {       \
            printk(#name " 0x%08x:  %08x %08x %08x %08x\n", addr+(i*4),              \
            dma_isp_read_32(addr+(i*4)), dma_isp_read_32(addr+(i*4)+4), dma_isp_read_32(addr+(i*4)+8), dma_isp_read_32(addr+(i*4)+0xc));    }

#define DUMP_ISP(name, addr, size)          \
        for(i=0;i<(size>>2);i+=4)  {       \
            printk(#name " 0x%08x:  %08x %08x %08x %08x\n", addr+(i*4),              \
            system_hw_read_32(addr+(i*4)), system_hw_read_32(addr+(i*4)+4), system_hw_read_32(addr+(i*4)+8), system_hw_read_32(addr+(i*4)+0xc));    }

#define DUMP_SRAM(name, addr, size)          \
        for(i=0;i<(size>>2);i+=4)  {       \
            printk(#name " 0x%08x:  %08x %08x %08x %08x\n", addr+(i*4),              \
            system_dmac_sram_read_32(addr+(i*4)), system_dmac_sram_read_32(addr+(i*4)+4), system_dmac_sram_read_32(addr+(i*4)+8), system_dmac_sram_read_32(addr+(i*4)+0xc));    }

void dump_dma(void)
{
    uint32_t i;
    DUMP_DMA(DMA, 0, 0x70);
}

void cmp_result( hobot_dma_t *hobot_dma      )
{
    uint32_t test_pass=1;   // 0: to sram; 1: to isp
    uint32_t i,j;
    for(i=0;i<hobot_dma->nents_total;i++) {
        if(hobot_dma->hobot_dma_cmds[i].direction == HOBOT_DMA_DIR_READ_ISP) {
            for(j=0;j<hobot_dma->hobot_dma_cmds[i].size;j+=4) {
                if(system_hw_read_32(hobot_dma->hobot_dma_cmds[i].isp_sram_addr-0xB3000000+j) !=
                    system_dmac_sram_read_32(hobot_dma->hobot_dma_cmds[i].dma_sram_addr-HOBOT_DMA_SRAM_PA+j))  {
                    printk("Test fail (write SRAM) : [%08x] %08x != [%08x] %08x\n",
                        (hobot_dma->hobot_dma_cmds[i].isp_sram_addr+j),
                        system_hw_read_32(hobot_dma->hobot_dma_cmds[i].isp_sram_addr-0xB3000000+j),
                        (hobot_dma->hobot_dma_cmds[i].dma_sram_addr+j),
                        system_dmac_sram_read_32(hobot_dma->hobot_dma_cmds[i].dma_sram_addr-HOBOT_DMA_SRAM_PA+j));
                        test_pass=0;
                }
            }
        }
        else {    // for write ISP
            if((hobot_dma->hobot_dma_cmds[i].isp_sram_addr < (ISP_AREA1+0xB3000000)) &&
                ((ISP_AREA1+0xB3000000) < hobot_dma->hobot_dma_cmds[i].isp_sram_addr+hobot_dma->hobot_dma_cmds[i].size))
            {
                for(j=0;j<TEST_SIZE;j+=4) {
                    if(system_hw_read_32(ISP_AREA1+j) !=
                        system_dmac_sram_read_32(ISP_AREA1+j))  {
                        printk("DMA error (write ISP - ping) : [%08x] %08x != [%08x] %08x\n",
                            (ISP_AREA1+0xB3000000+j),
                            system_hw_read_32(ISP_AREA1+j),
                            (ISP_AREA1+HOBOT_DMA_SRAM_PA+j),
                            system_dmac_sram_read_32(ISP_AREA1+j));
                        test_pass=0;
                    }
                }
            }
            else if((hobot_dma->hobot_dma_cmds[i].isp_sram_addr < (ISP_AREA2+0xB3000000)) &&
                ((ISP_AREA2+0xB3000000) < hobot_dma->hobot_dma_cmds[i].isp_sram_addr+hobot_dma->hobot_dma_cmds[i].size))
            {
                for(j=0;j<TEST_SIZE;j+=4) {
                    if(system_hw_read_32(ISP_AREA2+j) !=
                        system_dmac_sram_read_32(ISP_AREA1+j))  {
                        printk("DMA error (write ISP - pong) : [%08x] %08x != [%08x] %08x\n",
                            (ISP_AREA2+0xB3000000+j),
                            system_hw_read_32(ISP_AREA2+j),
                            (ISP_AREA1+HOBOT_DMA_SRAM_PA+j),
                            system_dmac_sram_read_32(ISP_AREA1+j));
                        test_pass=0;
                    }
                }
            }
        }
    }
    if(test_pass == 0)
        dump_dma();
}

//Hobot DMA internal functions
irqreturn_t hobot_dma_interrupt(int irq, void *data)
{
    hobot_dma_t *hobot_dma = (hobot_dma_t*) data;
    uint32_t i;
    static uint32_t count = 0;

    // check int status
    if(dma_isp_dma_int_read()) {
        HOBOT_DMA_LOG("%s: receive dma done int (cnt=%d)\n", __FUNCTION__, count);
        count++;
        dma_isp_reg_clr_int_write(1);           // clear dma int
        dma_isp_reg_clr_int_write(0);           // clear dma int
    }
    dma_isp_reg_start_dma_write(0);         // disable dma
    dma_isp_reg_dma_sram_ch_en_write(0);    // cpu can control sram now
#if HOBOT_DMA_RESULT_CHK
    cmp_result(hobot_dma);
#endif

    // process each callback
    for(i=0; i<hobot_dma->call_back_num; i++) {
        if (NULL != hobot_dma->call_back_obj[i].cb) {
            hobot_dma->call_back_obj[i].cb(hobot_dma->call_back_obj[i].cb_data);
            HOBOT_DMA_LOG("%s : cb (%d/%d) done\n", __FUNCTION__,i,hobot_dma->call_back_num);
        }
        else
            printk("%s ERROR: cb is NULL\n", __FUNCTION__);
    }
    hobot_dma->call_back_num = 0;
    hobot_dma->nents_total = 0;
    hobot_dma->is_busy = 0;
    return IRQ_HANDLED;
}

void hobot_dma_init(hobot_dma_t *hobot_dma)
{
    struct device_node *np;
    uint32_t ret;
    unsigned long flags;

    // 1. check lock init
    if(hobot_dma->dma_ctrl_lock==NULL) {
        if(system_spinlock_init(&hobot_dma->dma_ctrl_lock)<0) {
            printk(KERN_ERR "ERROR: %s init dma ctrl lock failed\n", __FUNCTION__);
            return;
        }
    }

    // 2. check init status (if already inited, just exit)
    flags = system_spinlock_lock( hobot_dma->dma_ctrl_lock );
    if(hobot_dma->init_cnt>0) {
        printk("%s: already inited (init_cnt=%d)\n", __FUNCTION__,hobot_dma->init_cnt);
        system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
        return;
    }
    hobot_dma->is_busy = 0;
    // clean callback record
    hobot_dma->call_back_num = hobot_dma->nents_total = 0;
    memset(hobot_dma->call_back_obj, 0, sizeof(hobot_dma_callback_t)*HOBOT_DMA_MAX_CALLBACK);
    memset(hobot_dma->hobot_dma_cmds, 0, sizeof(hobot_dma_cmd_t)*HOBOT_DMA_MAX_CMD);
    hobot_dma->init_cnt++;
    system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );

    // 3. if first time init, mapping interrupt here
    np = of_find_compatible_node(NULL, NULL, "hobot,x2a-isp");
    if (!np) {
        printk(KERN_ERR "ERROR: %s entry not find\n", __FUNCTION__);
    }
    hobot_dma->irq_in_dts = irq_of_parse_and_map(np, HOBOT_DMA_IRQ_INDEX);  /* PTP2 IRQ */
    printk("%s irq index=%d, request_irq: num=%d,handler=%p,flag=%x,name=%s,data=%p\n", __FUNCTION__,
                    HOBOT_DMA_IRQ_INDEX, hobot_dma->irq_in_dts, hobot_dma_interrupt, IRQF_TRIGGER_HIGH,
                    HOBOT_DMA_IRQ_NAME, hobot_dma);
    ret = request_irq(hobot_dma->irq_in_dts, hobot_dma_interrupt, IRQF_TRIGGER_HIGH, HOBOT_DMA_IRQ_NAME, hobot_dma);
    if(ret)
        printk(KERN_ERR "ERROR: %s request_irq() failed: %d\n", __FUNCTION__, ret);

    // disable irq after request_irq, if not, it may cause "Unbalanced enable for IRQ" warning log
    disable_irq(hobot_dma->irq_in_dts);

    // register is mapping in system_hw_io.c : init_hw_io()
}

void hobot_dma_deinit(hobot_dma_t *hobot_dma)
{
    const char *devname;
    unsigned long flags;
    // 1. check lock init
    if(hobot_dma->dma_ctrl_lock==NULL) {
        printk(KERN_ERR "ERROR: %s dma_ctrl_lock = NULL\n", __FUNCTION__);
        return;
    }

    // 2. check init status (if already inited, just exit)
    flags = system_spinlock_lock( hobot_dma->dma_ctrl_lock );
    if(hobot_dma->init_cnt) {
        hobot_dma->init_cnt--;
        if(hobot_dma->init_cnt > 0) {
            printk("%s: wait for all user uninit (init_cnt=%d)\n", __FUNCTION__,hobot_dma->init_cnt);
            system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
            return;
        }
    }
    else {
        printk(KERN_ERR "%s: not init dma yet (init_cnt=%d)\n", __FUNCTION__,hobot_dma->init_cnt);
        system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
        return;
    }

    // 3. start to process deinit dma
    if(hobot_dma->enable_irq_cnt>0) {
        printk(KERN_ERR "ERROR: %s enable_irq_cnt = %d\n", __FUNCTION__,hobot_dma->enable_irq_cnt);
    }
    system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
    devname = free_irq(hobot_dma->irq_in_dts, hobot_dma);
    HOBOT_DMA_LOG("%s free irq %d, devname %p\n", __FUNCTION__, hobot_dma->irq_in_dts, devname);
}

void hobot_dma_enable_irq(hobot_dma_t *hobot_dma)
{
    unsigned long flags;
    // 1. check lock init
    if(hobot_dma->dma_ctrl_lock==NULL) {
        printk(KERN_ERR "ERROR: %s dma_ctrl_lock = NULL\n", __FUNCTION__);
        return;
    }

    // 2. enable irq
    flags = system_spinlock_lock( hobot_dma->dma_ctrl_lock );
    if(hobot_dma->init_cnt) {
        if(hobot_dma->enable_irq_cnt == 0) {
            enable_irq(hobot_dma->irq_in_dts);
            HOBOT_DMA_LOG("%s : enable dma irq (%d)\n", __FUNCTION__, hobot_dma->irq_in_dts);
        }
        else {
            HOBOT_DMA_LOG("%s : enable_irq_cnt (%d) != 0, just exit\n", __FUNCTION__, hobot_dma->enable_irq_cnt);
        }
        hobot_dma->enable_irq_cnt++;
    }
    else {
        printk(KERN_ERR "ERROR: %s hobot_dma not init (init_cnt = %d)\n",
                __FUNCTION__,hobot_dma->init_cnt);
    }
    system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
}

void hobot_dma_disable_irq(hobot_dma_t *hobot_dma)
{
    unsigned long flags;
    // 1. check lock init
    if(hobot_dma->dma_ctrl_lock==NULL) {
        printk(KERN_ERR "ERROR: %s dma_ctrl_lock = NULL\n", __FUNCTION__);
        return;
    }

    // 2. disable irq
    flags = system_spinlock_lock( hobot_dma->dma_ctrl_lock );
    if((hobot_dma->init_cnt) && (hobot_dma->enable_irq_cnt)){
        hobot_dma->enable_irq_cnt--;
        if(hobot_dma->enable_irq_cnt == 0) {
            disable_irq_nosync(hobot_dma->irq_in_dts);
            printk("%s : disable dma irq\n", __FUNCTION__);
        }
    }
    else {
        printk(KERN_ERR "ERROR: %s not allow to disable irq (init_cnt = %d, enable_irq_cnt = %d)\n",
                __FUNCTION__,hobot_dma->init_cnt,hobot_dma->enable_irq_cnt);
    }
    system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
}

static void hobot_dma_start(        hobot_dma_cmd_t *hobot_dma_cmds,
                                    uint32_t fw_ctx_id,
                                    uint32_t nents)
{
    uint32_t dma_cmd_que_setting = 0, i;
    static uint32_t count =0;
    // 1. enable dma channel (CPU can not access this SRAM bolck now)
    if(fw_ctx_id > 3) {
        printk(KERN_ERR "ERROR: %s fw_ctx_id(%d) is more than 3\n", __FUNCTION__,fw_ctx_id);
        fw_ctx_id = 0;
    }
    HOBOT_DMA_LOG("%s : fw_ctx_id=%d, nents=%d, \n", __FUNCTION__,fw_ctx_id,nents);
    dma_isp_reg_dma_sram_ch_sel_write(fw_ctx_id);   // select sram block
    dma_isp_reg_mask_int_write(DMA_INT_ENABLE);     // enable dma int

    // 2. set command queue for ch0~ch3
    for ( i = 0; i < nents; i++ ) {
        HOBOT_DMA_LOG("%s : [%d] - size=%d, sram=0x%x, isp=0x%x, %s\n",
            __FUNCTION__, i, hobot_dma_cmds[i].size,
            hobot_dma_cmds[i].dma_sram_addr,hobot_dma_cmds[i].isp_sram_addr,
            (hobot_dma_cmds[i].direction == HOBOT_DMA_DIR_WRITE_ISP)?"write ISP":"write sram");
        if(i==0) {
            dma_cmd_que_setting |= DMA_CH0;
            if(hobot_dma_cmds[i].direction == HOBOT_DMA_DIR_WRITE_ISP) {
                dma_cmd_que_setting |= DMA_CH0<<8;  // set ch0 direction (sram<-isp = 0; sram->isp = 1;)
            }
            dma_isp_reg_dma_tran_size_write(hobot_dma_cmds[i].size>>2);
            dma_isp_reg_dma_recv_size_write(hobot_dma_cmds[i].size>>2);
            dma_isp_reg_load_sram_addr_write((hobot_dma_cmds[i].dma_sram_addr-HOBOT_DMA_SRAM_PA)>>2);
            dma_isp_reg_load_ahb_addr_write(hobot_dma_cmds[i].isp_sram_addr);
        }
        else if(i==1) {
            dma_cmd_que_setting |= DMA_CH1;
            if(hobot_dma_cmds[i].direction == HOBOT_DMA_DIR_WRITE_ISP) {
                dma_cmd_que_setting |= DMA_CH1<<8;  // set ch0 direction (sram<-isp = 0; sram->isp = 1;)
            }
            dma_isp_reg_dma_tran_size_write_ch1(hobot_dma_cmds[i].size>>2);
            dma_isp_reg_dma_recv_size_write_ch1(hobot_dma_cmds[i].size>>2);
            dma_isp_reg_load_sram_addr_write_ch1((hobot_dma_cmds[i].dma_sram_addr-HOBOT_DMA_SRAM_PA)>>2);
            dma_isp_reg_load_ahb_addr_write_ch1(hobot_dma_cmds[i].isp_sram_addr);
        }
        else if(i==2) {
            dma_cmd_que_setting |= DMA_CH2;
            if(hobot_dma_cmds[i].direction == HOBOT_DMA_DIR_WRITE_ISP) {
                dma_cmd_que_setting |= DMA_CH2<<8;  // set ch0 direction (sram<-isp = 0; sram->isp = 1;)
            }
            dma_isp_reg_dma_tran_size_write_ch2(hobot_dma_cmds[i].size>>2);
            dma_isp_reg_dma_recv_size_write_ch2(hobot_dma_cmds[i].size>>2);
            dma_isp_reg_load_sram_addr_write_ch2((hobot_dma_cmds[i].dma_sram_addr-HOBOT_DMA_SRAM_PA)>>2);
            dma_isp_reg_load_ahb_addr_write_ch2(hobot_dma_cmds[i].isp_sram_addr);
        }
        else if(i==3) {
            dma_cmd_que_setting |= DMA_CH3;
            if(hobot_dma_cmds[i].direction == HOBOT_DMA_DIR_WRITE_ISP) {
                dma_cmd_que_setting |= DMA_CH3<<8;  // set ch0 direction (sram<-isp = 0; sram->isp = 1;)
            }
            dma_isp_reg_dma_tran_size_write_ch3(hobot_dma_cmds[i].size>>2);
            dma_isp_reg_dma_recv_size_write_ch3(hobot_dma_cmds[i].size>>2);
            dma_isp_reg_load_sram_addr_write_ch3((hobot_dma_cmds[i].dma_sram_addr-HOBOT_DMA_SRAM_PA)>>2);
            dma_isp_reg_load_ahb_addr_write_ch3(hobot_dma_cmds[i].isp_sram_addr);
        }
    }
    dma_isp_write_32(DMA_CMD_QUEUE_CTRL, dma_cmd_que_setting);
    dma_isp_reg_dma_sram_ch_en_write(1);            // lock sram for dma control

    // 3. start dma
    dma_isp_reg_start_dma_write(1);
    HOBOT_DMA_LOG("%s : start dma (cnt=%d)\n", __FUNCTION__,count);
    count++;
}

void hobot_dma_submit_cmd(hobot_dma_t *hobot_dma,
                                    uint32_t fw_ctx_id,
                                    struct scatterlist *isp_sram_sg,
                                    struct scatterlist *dma_sram_sg,
                                    unsigned int nents,
                                    uint32_t direction,
                                    dma_completion_callback cb,
                                    void *cb_data,
                                    uint8_t last_cmd)
{
    int i;
    unsigned long flags;
    // 1. check lock init
    if(hobot_dma->dma_ctrl_lock==NULL) {
        printk(KERN_ERR "ERROR: %s dma_ctrl_lock = NULL\n", __FUNCTION__);
        return;
    }

    flags = system_spinlock_lock( hobot_dma->dma_ctrl_lock );
    uint32_t nents_total = hobot_dma->nents_total;
    if(hobot_dma->is_busy) {
        printk(KERN_ERR "ERROR: %s dma is still busy now\n", __FUNCTION__);
        system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
        return;
    }
    if((hobot_dma->nents_total+nents) > HOBOT_DMA_MAX_CMD) {
        printk(KERN_ERR "ERROR: %s can not process too many memory blocks (nents_total=%d,cur_nents=%d)\n",
            __FUNCTION__, hobot_dma->nents_total, nents);
        system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
        return;
    }
    hobot_dma->call_back_obj[hobot_dma->call_back_num].cb = cb;
    hobot_dma->call_back_obj[hobot_dma->call_back_num].cb_data = cb_data;
    hobot_dma->call_back_num++;

    for ( i = 0; i < nents; i++ ) {
        hobot_dma->hobot_dma_cmds[i+nents_total].isp_sram_addr = sg_dma_address( isp_sram_sg );
        hobot_dma->hobot_dma_cmds[i+nents_total].dma_sram_addr = sg_dma_address( dma_sram_sg );
        hobot_dma->hobot_dma_cmds[i+nents_total].size          = sg_dma_len( isp_sram_sg );
        hobot_dma->hobot_dma_cmds[i+nents_total].direction = direction;
        isp_sram_sg = sg_next( isp_sram_sg );
        dma_sram_sg = sg_next( dma_sram_sg );
    }
    hobot_dma->nents_total += nents;

#if (HOBOT_DMA_SRAM_PA == HOBOT_DMA_SRAM_DEBUG_DRAM_MODE)
    static uint8_t tmp[HOBOT_DMA_SRAM_SIZE];
    struct scatterlist *src_sg = (direction == HOBOT_DMA_DIR_WRITE_ISP) ? dma_sram_sg : isp_sram_sg;
    struct scatterlist *dst_sg = (direction == HOBOT_DMA_DIR_WRITE_ISP) ? isp_sram_sg : dma_sram_sg;
    sg_copy_to_buffer(src_sg, nents, tmp, HOBOT_DMA_SRAM_SIZE);
    sg_copy_from_buffer(dst_sg, nents, tmp, HOBOT_DMA_SRAM_SIZE);

    // to process callback
    hobot_dma_interrupt(hobot_dma->irq_in_dts, hobot_dma);
#else
    // Trigger dma start in last command
    if(last_cmd) {
        hobot_dma->is_busy = 1;
        hobot_dma_start(hobot_dma->hobot_dma_cmds, fw_ctx_id, hobot_dma->nents_total);
    }
#endif
    system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
}


#endif  /* FW_USE_HOBOT_DMA */


