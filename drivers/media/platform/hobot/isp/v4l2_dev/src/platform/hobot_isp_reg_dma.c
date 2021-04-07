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

#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__
#include <linux/kernel.h> /* //printk() */
#include <asm/uaccess.h>
#include <linux/gfp.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <asm/types.h>
#include <asm/io.h>
#include <linux/time.h>
#include <linux/list.h>
#include "acamera_fw.h"
#include "system_dma.h"
#include "system_sw_io.h"
#include "acamera_isp_config.h"
#include "acamera_isp_core_nomem_settings.h"
#include "acamera_decompander0_mem_config.h"
#include "hobot_isp_reg_dma.h"
#include "vio_config.h"

#if FW_USE_HOBOT_DMA

#define HOBOT_DMA_RESULT_CHK     0
#define HOBOT_DMA_LOG(...)   pr_debug(__VA_ARGS__)

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

int list_count(struct list_head *list)
{
	int k = 0;
	struct list_head *this, *next;

	if (list_empty(list))
		return 0;

	list_for_each_safe(this, next, list)
		k++;

	return k;
}

extern int isp_stream_onoff_check(void);
extern acamera_firmware_t *acamera_get_firmware_ptr(void);
void isp_idma_start_transfer(hobot_dma_t *hobot_dma)
{
    uint8_t ppf = 0;
    int ret = 0;
    bool meet_condition = false;
	unsigned long flags;
	idma_descriptor_t *desc = NULL;
    acamera_firmware_t *fw = acamera_get_firmware_ptr();

	pr_debug("start\n");
	spin_lock_irqsave( hobot_dma->dma_ctrl_lock, flags );
	if (list_empty(&hobot_dma->pending_list)) {
		pr_debug("pending list empty.\n");
		spin_unlock_irqrestore(hobot_dma->dma_ctrl_lock, flags);
		return;
	}

    pr_debug("pending list count %d\n", list_count(&hobot_dma->pending_list));

	desc = list_first_entry(&hobot_dma->pending_list, idma_descriptor_t, node);
	if (desc) {
        /*
          prevent a case like:
          oom-killer be triggered, CPU is irqoff and prempt is also disabled.
          tasklet will be pending, after a while, tasklet schedule again, it may write to isp,
          at the same time isp is processing, so we need drop this kind of transfer.
          an invalid transfer is: ping in using now, transfer is going to write to ping.
        */
        if (fw->sif_isp_offline) {
            meet_condition = (fw->sw_frame_counter > 1 && isp_stream_onoff_check() > 0);
        } else {
            meet_condition = (isp_stream_onoff_check() > 0);
        }
        if (meet_condition && desc->direction == HOBOT_DMA_DIR_WRITE_ISP) {
            hobot_dma->hobot_dma_cmds[0].isp_sram_addr = sg_dma_address(desc->isp_sram_sg);
            if (hobot_dma->hobot_dma_cmds[0].isp_sram_addr == 0xb3000000 + ACAMERA_DECOMPANDER0_MEM_BASE_ADDR)
                ppf = ISP_CONFIG_PING;
            else
                ppf = ISP_CONFIG_PONG;

            pr_debug("isp addr is %x\n", hobot_dma->hobot_dma_cmds[0].isp_sram_addr);

            if (ppf == acamera_isp_isp_global_ping_pong_config_select_read(0)) {
                pr_err("ppf %d, invalid xfer, drop it.\n", ppf);
                if (desc->callback.cb)
                    desc->callback.cb(desc->callback.cb_data);
                list_move_tail(&desc->node, &hobot_dma->free_list);
                pr_debug("after drop, free list count %d\n", list_count(&hobot_dma->free_list));
                goto out;
            }
        }

		ret = hobot_dma_submit_cmd(hobot_dma, desc, 1);
        if (ret < 0) {  //submit failed, give back to pending list
            pr_err("xfer submit failed, back to pending queue.\n");
            list_add_tail(&desc->node, &hobot_dma->pending_list);
        }
    }

out:
	spin_unlock_irqrestore(hobot_dma->dma_ctrl_lock, flags);
	pr_debug("end\n");
}

static void isp_idma_tasklet(unsigned long data)
{
	unsigned long flags;
	idma_descriptor_t *desc, *next;
	hobot_dma_t *hobot_dma = (hobot_dma_t*) data;

	pr_debug("start\n");

	spin_lock_irqsave( hobot_dma->dma_ctrl_lock, flags );
	// process each callback

	if (!list_empty(&hobot_dma->done_list)) {
		list_for_each_entry_safe(desc, next, &hobot_dma->done_list, node) {
            pr_debug("dir %d, node add to free list\n", desc->direction);
            list_move_tail(&desc->node, &hobot_dma->free_list);
            pr_debug("free list count %d\n", list_count(&hobot_dma->free_list));
            if (desc->callback.cb) {
			    desc->callback.cb(desc->callback.cb_data);
            } else {
                pr_err("idma cb is null. desc %p, cb1 %p, cb2 %p, ctx id %d %p, dir %d %p, nents %d %p.\n",
                    desc, &desc->callback, desc->callback.cb,
                    desc->ctx_id, &desc->ctx_id, desc->direction, &desc->direction,
                    desc->isp_sram_nents, &desc->isp_sram_nents);
            }
		}
	}
	spin_unlock_irqrestore(hobot_dma->dma_ctrl_lock, flags);

	isp_idma_start_transfer(hobot_dma);
	pr_debug("end\n");
}

//Hobot DMA internal functions
irqreturn_t hobot_dma_interrupt(int irq, void *data)
{
    hobot_dma_t *hobot_dma = (hobot_dma_t*) data;
    static uint32_t count = 0;
    unsigned long flags;

    dma_isp_reg_start_dma_write(0);         // disable dma
    dma_isp_reg_mask_int_write(DMA_INT_DISABLE);  //mask dma irq
    dma_isp_reg_dma_sram_ch_en_write(0);    // cpu can control sram now

    spin_lock_irqsave( hobot_dma->dma_ctrl_lock, flags );
    // check int status
    if(dma_isp_dma_int_read()) {
        HOBOT_DMA_LOG("%s: receive dma done int (cnt=%d)\n", __FUNCTION__, count);
        count++;
        dma_isp_reg_clr_int_write(1);           // clear dma int
        dma_isp_reg_clr_int_write(0);           // clear dma int

	pr_debug("active list count %d\n", list_count(&hobot_dma->active_list));

	if (!list_empty(&hobot_dma->active_list))
		list_splice_tail_init(&hobot_dma->active_list, &hobot_dma->done_list);
    } else {
        // dma is not done, clear active_list
        pr_err("idma is not done.\n");
        if (!list_empty(&hobot_dma->active_list)) {
            list_splice_tail_init(&hobot_dma->active_list, &hobot_dma->free_list);
        }
    }

    hobot_dma->nents_total = 0;
    hobot_dma->is_busy = 0;
    spin_unlock_irqrestore(hobot_dma->dma_ctrl_lock, flags);

#if HOBOT_DMA_RESULT_CHK
    cmp_result(hobot_dma);
#endif
    pr_debug("end.\n");

    tasklet_schedule(&hobot_dma->tasklet);

    return IRQ_HANDLED;
}

static int idma_reg_dump = 1;
module_param(idma_reg_dump, int, S_IRUGO|S_IWUSR);
void hobot_idma_try_restore(void *data)
{
    hobot_dma_t *hobot_dma = (hobot_dma_t*) data;
    unsigned long flags;

    pr_debug("+\n");

    if (idma_reg_dump)
        dump_dma();

    dma_isp_reg_start_dma_write(0);         // disable dma
    dma_isp_reg_mask_int_write(DMA_INT_DISABLE);  //mask dma irq
    dma_isp_reg_dma_sram_ch_en_write(0);    // cpu can control sram now

    spin_lock_irqsave( hobot_dma->dma_ctrl_lock, flags );
    // check int status
    if(dma_isp_dma_int_read()) {
        dma_isp_reg_clr_int_write(1);           // clear dma int
        dma_isp_reg_clr_int_write(0);           // clear dma int
    }

    hobot_dma->nents_total = 0;
    hobot_dma->is_busy = 0;

    spin_unlock_irqrestore(hobot_dma->dma_ctrl_lock, flags);

    pr_debug("-\n");
}

void hobot_dma_init(hobot_dma_t *hobot_dma)
{
    int i = 0;
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
        system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );
        return;
    }
    hobot_dma->is_busy = 0;
    // clean callback record
    memset(hobot_dma->hobot_dma_cmds, 0, sizeof(hobot_dma_cmd_t)*HOBOT_DMA_MAX_CMD);
    hobot_dma->init_cnt++;
    system_spinlock_unlock( hobot_dma->dma_ctrl_lock, flags );

    INIT_LIST_HEAD(&hobot_dma->free_list);
    INIT_LIST_HEAD(&hobot_dma->pending_list);
    INIT_LIST_HEAD(&hobot_dma->done_list);
    INIT_LIST_HEAD(&hobot_dma->active_list);

    tasklet_init(&hobot_dma->tasklet, isp_idma_tasklet, (unsigned long)hobot_dma);

    for (i = 0; i < 4; i++) {
        idma_descriptor_t *desc;
        desc = system_sw_alloc(sizeof(idma_descriptor_t));
        if (!desc) {
            pr_err("alloc idma desc mem failed, size %lu\n",
					sizeof(idma_descriptor_t));
            return;
        }
        list_add_tail(&desc->node, &hobot_dma->free_list);
    }

    // 3. if first time init, mapping interrupt here
    np = of_find_compatible_node(NULL, NULL, "hobot,x3-isp");
    if (!np) {
        printk(KERN_ERR "ERROR: %s entry not find\n", __FUNCTION__);
    }
    hobot_dma->irq_in_dts = irq_of_parse_and_map(np, HOBOT_DMA_IRQ_INDEX);  /* PTP2 IRQ */
    ret = request_irq(hobot_dma->irq_in_dts, hobot_dma_interrupt, IRQF_TRIGGER_HIGH, HOBOT_DMA_IRQ_NAME, hobot_dma);
    if(ret)
        printk(KERN_ERR "ERROR: %s request_irq() failed: %d\n", __FUNCTION__, ret);

    irq_set_affinity_hint(hobot_dma->irq_in_dts, get_cpu_mask(VIO_IRQ_CPU_IDX));
    dma_isp_reg_mask_int_write(DMA_INT_DISABLE);  //mask dma irq

    // disable irq after request_irq, if not, it may cause "Unbalanced enable for IRQ" warning log
    //disable_irq(hobot_dma->irq_in_dts);

    // register is mapping in system_hw_io.c : init_hw_io()
}

void hobot_dma_deinit(hobot_dma_t *hobot_dma)
{
    int i;
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

    for (i = 0; i < 4; i++) {
        idma_descriptor_t *desc, *next;
        if (!list_empty(&hobot_dma->free_list)) {
            list_for_each_entry_safe(desc, next, &hobot_dma->free_list, node) {
                list_del(&desc->node);
                kfree(desc);
            }
        }
    }

    tasklet_kill(&hobot_dma->tasklet);

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
        HOBOT_DMA_LOG("%s : [%d] - size=%x, sram=0x%x, isp=0x%x, %s\n",
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

int hobot_dma_submit_cmd(hobot_dma_t *hobot_dma, idma_descriptor_t *desc, int last_cmd)
{
    int i;
    struct scatterlist *isp_sram_sg, *dma_sram_sg;
    int nents = desc->isp_sram_nents;
    int fw_ctx_id;
    int direction;

    pr_debug("start\n");
    // 1. check lock init
    if(hobot_dma->dma_ctrl_lock==NULL) {
        printk(KERN_ERR "ERROR: %s dma_ctrl_lock = NULL\n", __FUNCTION__);
        return -1;
    }

    uint32_t nents_total = hobot_dma->nents_total;
    if(hobot_dma->is_busy) {
        printk(KERN_ERR "ERROR: %s, ctx_id %d dma is still busy now\n", __FUNCTION__, desc->ctx_id);
        return -1;
    }
    if((hobot_dma->nents_total+nents) > HOBOT_DMA_MAX_CMD) {
        printk(KERN_ERR "ERROR: %s can not process too many memory blocks (nents_total=%d,cur_nents=%d)\n",
            __FUNCTION__, hobot_dma->nents_total, nents);
        return -1;
    }

    fw_ctx_id = desc->ctx_id;	
    direction = desc->direction;
    isp_sram_sg = desc->isp_sram_sg;
    dma_sram_sg = desc->dma_sram_sg;

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
    if(last_cmd) {
	hobot_dma->is_busy = 1;
        hobot_dma_start(hobot_dma->hobot_dma_cmds, fw_ctx_id, hobot_dma->nents_total);
	list_move_tail(&desc->node, &hobot_dma->active_list);
    }
#endif

    return 0;
}


#endif  /* FW_USE_HOBOT_DMA */


