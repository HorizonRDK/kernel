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

#include "acamera_logger.h"
#include <linux/gfp.h>
#include <linux/slab.h>
#include "acamera_firmware_config.h"
#include "acamera_fw.h"
#include "acamera_command_api.h"
#include <asm/io.h>
#include "hobot_isp_reg_dma_regset.h"

#define	BIT_FIELD_MASK(m, n)	((~0U >> (BITS_PER_LONG - (n - m + 1))) << m)

// for register read/write
struct regs_t {
        uint32_t addr;
        uint8_t m;
        uint8_t n;
        uint32_t v;
};

extern void *acamera_get_api_ctx_ptr(void);

#if HOBOT_REGISTER_MONITOR
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

#define HRM_RC_HW_BASE   0x11100000
#define HRM_RC_SW_BASE   0x22200000
#define HRM_RC_DMA_START 0x33310000
#define HRM_RC_DMA_END   0x33320000

#define MAX_SW_ISP_BASE 4
typedef struct _sw_isp_base_t {
    uint8_t* base;
    uint32_t size;
}sw_isp_base_t;

#define HOBOT_RM_MAX 1000
typedef struct _hobot_reg_monitor_pair_t {
    uint32_t offset;
    uint32_t value;
    uint32_t mask;
    uint32_t is_write;
    uint32_t trigger_times;
}hobot_reg_monitor_pair_t;
typedef struct _hobot_reg_monitor_t {
    hobot_reg_monitor_pair_t reg[HOBOT_RM_MAX];
    uint32_t                 cnt;
    uint32_t                 record_en;
    struct file              *file;
    uint32_t                 file_id_cnt;
}hobot_reg_monitor_t;

static uint32_t g_count=0;
static sw_isp_base_t g_sw_isp_base[MAX_SW_ISP_BASE];
static hobot_reg_monitor_t g_hobot_rm_data;

struct file *hobot_file_open(const char *path, int flags, int rights)
{
    struct file *filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if (IS_ERR(filp)) {
        err = PTR_ERR(filp);
        return NULL;
    }
    return filp;
}

void hobot_file_quick_open(void)
{
    char filename[100];
    sprintf(filename,"/tmp/isp%8.8x.reg",g_hobot_rm_data.file_id_cnt);
    printk("create file %s=====================================================",filename);
    g_hobot_rm_data.file = hobot_file_open(filename, O_CREAT | O_WRONLY | O_APPEND, 0);
    if(g_hobot_rm_data.file == NULL)
        printk("%s ERROR: file /tmp/isp.reg open fail", __FUNCTION__);
    else
        printk("%s file /tmp/isp.reg open successful", __FUNCTION__);
    g_hobot_rm_data.file_id_cnt++;

}

void hobot_file_close(struct file *file)
{
    filp_close(file, NULL);
}

int hobot_file_write(struct file *file, unsigned char *data, unsigned int size)
{
    loff_t pos;
    int ret;

    pos = file->f_pos;
    //printk("%s %ld",__FUNCTION__,pos);
    ret = kernel_write(file, data, size, &pos);
    file->f_pos = pos;

    if(pos%(50*1024)==0)
    {
        hobot_file_close(file);
        hobot_file_quick_open();
    }

    return ret;
}

int hobot_file_sync(struct file *file)
{
    vfs_fsync(file, 0);
    return 0;
}

void hobot_rm_dump_table( void )
{
    int i;
    printk("HOBOT RM (%d): ", g_hobot_rm_data.cnt);
    for(i = 0; i < g_hobot_rm_data.cnt; i++) {
        printk(KERN_CONT "%x, ", g_hobot_rm_data.reg[i].offset);
    }
}

void hobot_rm_reset( void )
{
    memset(&g_hobot_rm_data, 0, sizeof(hobot_reg_monitor_t));
}

void hobot_rm_enable_recoder(uint32_t enable)
{
    g_hobot_rm_data.record_en = enable;
}


void hobot_rm_init( void )
{
    hobot_rm_reset();
    hobot_file_quick_open();

    hobot_rm_enable_recoder(1);
}

void hobot_rm_add(uint32_t offset)
{
    g_hobot_rm_data.reg[g_hobot_rm_data.cnt].offset = offset;
    g_hobot_rm_data.cnt++;
}

void hobot_rm_check_n_record(uint32_t offset, uint32_t value)
{
    int i;
    int cnt = HOBOT_RM_MAX;
    int offset_for_cmp = offset & 0x000fffff;

#if HOBOT_REGISTER_MONITOR_FILEWRITE
   {
        //printk(KERN_CONT "[%x]=%x,",offset,value);
//      g_hobot_rm_data.file = hobot_file_open("/tmp/isp.reg", O_APPEND | O_CREAT | O_WRONLY, 0);
        if((g_hobot_rm_data.file)&&(in_interrupt()==0)){
            hobot_file_write(g_hobot_rm_data.file, (char*)&offset, sizeof(uint32_t));
            hobot_file_write(g_hobot_rm_data.file, (char*)&value, sizeof(uint32_t));
            hobot_file_sync(g_hobot_rm_data.file);
//          hobot_file_close(g_hobot_rm_data.file);
        }
    }
#endif

    if(cnt>g_hobot_rm_data.cnt) cnt = g_hobot_rm_data.cnt;

    for(i = 0; i < cnt; i++) {
        if(offset_for_cmp == g_hobot_rm_data.reg[i].offset) {
            printk("HRM [%x]=%x\n", offset, value);
#if HOBOT_REGISTER_MONITOR_DUMP_STACK
            dump_stack();
#endif
        }
    }
}
#else
void hobot_rm_check_n_record(uint32_t offset, uint32_t value)
{
}

#endif//HOBOT_REGISTER_MONITOR

int32_t init_sw_io( void )
{
    int32_t result = 0;
    //
    return result;
}

int32_t close_sw_io( void )
{
    int32_t result = 0;
    return result;
}

void *system_sw_alloc( uint32_t size )
{
    void* va;

    va = kzalloc( size, GFP_KERNEL | GFP_DMA | GFP_ATOMIC );
#if HOBOT_REGISTER_MONITOR
    hobot_rm_init();
    //hobot_rm_add(0x80);
//	    hobot_rm_add(0x18FE4);
//	    hobot_rm_add(0x18FE8);
//	    hobot_rm_add(0x18FF0);
//	    hobot_rm_add(0x18FFC);
//	    hobot_rm_add(0x19004);
    hobot_rm_dump_table();
#endif//HOBOT_REGISTER_MONITOR

#if HOBOT_REGISTER_MONITOR
    g_sw_isp_base[g_count].base = va;
    g_sw_isp_base[g_count].size = size;
    g_count++;
#endif//HOBOT_REGISTER_MONITOR
    return va;
}

void system_sw_free( void *ptr )
{
    kfree( ptr );
#if HOBOT_REGISTER_MONITOR
        pr_err("%s HOBOT_REGISTER_MONITOR not support memory free\n", __FUNCTION__);
        g_count = 0;
#endif//HOBOT_REGISTER_MONITOR
}

#if FW_USE_HOBOT_DMA

uint8_t *g_hobot_dma_va = NULL;

void *system_sw_alloc_dma_sram( uint32_t size , uint32_t context_id, uint32_t *phy_addr)
{
    void *va;
    if(g_hobot_dma_va == NULL) {
#if (HOBOT_DMA_SRAM_PA == HOBOT_DMA_SRAM_DEBUG_DRAM_MODE)   // use DDR to simulate sram area
        g_hobot_dma_va = ioremap(HOBOT_DMA_SRAM_PA, HOBOT_DMA_SRAM_SIZE);   // map with cache
#else
        g_hobot_dma_va = ioremap_nocache(HOBOT_DMA_SRAM_PA, HOBOT_DMA_SRAM_SIZE);   // map without cache
#endif
        if(g_hobot_dma_va == NULL) {
            pr_err("%s ERROR: Hobot DMA map addr %x, size %d fail, __va()=%p\n",
                __FUNCTION__, HOBOT_DMA_SRAM_PA, HOBOT_DMA_SRAM_SIZE, __va(HOBOT_DMA_SRAM_PA));
            g_hobot_dma_va = __va(HOBOT_DMA_SRAM_PA);
            return NULL;
        } else {
            pr_debug("%s SUCCESS: Hobot DMA map addr %x, size %d ok\n", __FUNCTION__, HOBOT_DMA_SRAM_PA, HOBOT_DMA_SRAM_SIZE);
        }
    }
    if(size > HOBOT_DMA_SRAM_ONE_ZONE) {
        pr_err("%s ERROR: allocate size (%d) more than sram unit size (128 KBytes)\n", __FUNCTION__, size);
        return NULL;
    }

    va = (void*) (g_hobot_dma_va + (context_id*HOBOT_DMA_SRAM_ONE_ZONE));
    *phy_addr = HOBOT_DMA_SRAM_PA+ (context_id*HOBOT_DMA_SRAM_ONE_ZONE);
    pr_debug("%s : pa=0x%x, context_id=%d, offset=%d\n", __FUNCTION__, *phy_addr, context_id, (context_id*HOBOT_DMA_SRAM_ONE_ZONE));

#if HOBOT_REGISTER_MONITOR
    g_sw_isp_base[context_id].base = va;
    g_sw_isp_base[context_id].size = size;
    g_count++;
#endif//HOBOT_REGISTER_MONITOR
    return va;
}

void system_sw_free_dma_sram( void *ptr ,uint32_t context_id)
{
    pr_debug("%s WARNING: not support memory free dma sram (context_id=%d)\n", __FUNCTION__, context_id);

#if HOBOT_REGISTER_MONITOR
    pr_debug("%s WARNING: HOBOT_REGISTER_MONITOR not support memory free\n", __FUNCTION__, context_id);
    g_count = 0;
#endif//HOBOT_REGISTER_MONITOR

}
#endif

#if FW_USE_HOBOT_DMA
void system_sram_access_assert(uintptr_t addr)
{
	uint32_t ctx_id = dma_isp_reg_dma_sram_ch_sel_read();
	uintptr_t start = (uintptr_t)g_hobot_dma_va + ctx_id * HOBOT_DMA_SRAM_ONE_ZONE;
	uintptr_t end = start + HOBOT_DMA_SRAM_ONE_ZONE;

	if (start <= addr && addr <= end && dma_isp_reg_dma_sram_ch_en_read()) { 
		pr_debug("ctx id %d, start %p, end %p, addr %p\n",
			ctx_id, (uint32_t *)start, (uint32_t *)end, (uint32_t *)addr);
		pr_err("access sw_reg when dma enable\n");
	}
}
#else
void system_sram_access_assert(uintptr_t addr)
{}
#endif

uint32_t system_sw_read_32( uintptr_t addr )
{
    system_sram_access_assert(addr);
    uint32_t result = 0;
    if ( (void *)addr != NULL ) {
        volatile uint32_t *p_addr = (volatile uint32_t *)( addr );
        result = *p_addr;
    } else {
        LOG( LOG_ERR, "Failed to read memory from address 0x%x. Base pointer is null ", addr );
    }
    return result;
}

uint16_t system_sw_read_16( uintptr_t addr )
{
    system_sram_access_assert(addr);
    uint16_t result = 0;
    if ( (void *)addr != NULL ) {
        volatile uint16_t *p_addr = (volatile uint16_t *)( addr );
        result = *p_addr;
    } else {
        LOG( LOG_ERR, "Failed to read memory from address 0x%x. Base pointer is null ", addr );
    }
    return result;
}

uint8_t system_sw_read_8( uintptr_t addr )
{
    system_sram_access_assert(addr);
    uint8_t result = 0;
    if ( (void *)addr != NULL ) {
        volatile uint8_t *p_addr = (volatile uint8_t *)( addr );
        result = *p_addr;
    } else {
        LOG( LOG_ERR, "Failed to read memory from address 0x%x. Base pointer is null ", addr );
    }
    return result;
}


void system_sw_write_32( uintptr_t addr, uint32_t data )
{
#if HOBOT_REGISTER_MONITOR
    hobot_rm_check_n_record((uint32_t)(((uint8_t*)addr) - g_sw_isp_base[0].base + HRM_RC_SW_BASE) ,data);
#endif
    system_sram_access_assert(addr);

    if ( (void *)addr != NULL ) {
        volatile uint32_t *p_addr = (volatile uint32_t *)( addr );
        *p_addr = data;
    } else {
        LOG( LOG_ERR, "Failed to write %d to memory 0x%x. Base pointer is null ", data, addr );
    }
}

void system_sw_write_16( uintptr_t addr, uint16_t data )
{
#if HOBOT_REGISTER_MONITOR
        hobot_rm_check_n_record((uint32_t)(((uint8_t*)addr) - g_sw_isp_base[0].base + HRM_RC_SW_BASE) ,data);
#endif
    system_sram_access_assert(addr);

    if ( (void *)addr != NULL ) {
        volatile uint16_t *p_addr = (volatile uint16_t *)( addr );
        *p_addr = data;
    } else {
        LOG( LOG_ERR, "Failed to write %d to memory 0x%x. Base pointer is null ", data, addr );
    }
}

void system_sw_write_8( uintptr_t addr, uint8_t data )
{
#if HOBOT_REGISTER_MONITOR
        hobot_rm_check_n_record((uint32_t)(((uint8_t*)addr) - g_sw_isp_base[0].base + HRM_RC_SW_BASE) ,data);
#endif
    system_sram_access_assert(addr);

    if ( (void *)addr != NULL ) {
        volatile uint8_t *p_addr = (volatile uint8_t *)( addr );
        *p_addr = data;
    } else {
        LOG( LOG_ERR, "Failed to write %d to memory 0x%x. Base pointer is null ", data, addr );
    }
}

void system_reg_rw(struct regs_t *rg, uint8_t dir)
{
	acamera_context_ptr_t context_ptr = (acamera_context_ptr_t)acamera_get_api_ctx_ptr();

    if (context_ptr->initialized == 0) {
        pr_err("context %d is not initialized.\n", context_ptr->context_id);
        return;
    }

    uintptr_t sw_addr = context_ptr->settings.isp_base + rg->addr;
    uint32_t data = system_sw_read_32(sw_addr);
	uint32_t mask = BIT_FIELD_MASK(rg->m, rg->n);

	if (dir == COMMAND_SET) {
		data = (data & ~mask) | (rg->v << rg->m);
		system_sw_write_32(sw_addr, data);
	} else if (dir == COMMAND_GET) {
		data &= mask;
		rg->v = data >> rg->m;
	}
}
