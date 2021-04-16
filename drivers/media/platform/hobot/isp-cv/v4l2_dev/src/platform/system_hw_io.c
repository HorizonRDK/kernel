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

#include "acamera_logger.h"
#include "system_spinlock.h"
#include <asm/io.h>
#include "hobot_isp_reg_dma_regset.h"

static void *p_hw_base = NULL;
static void *p_hw_sysctrl_base = NULL;
static void *p_hw_ips_base = NULL;
static void *p_hw_dmac_isp_base = NULL;
static void *p_hw_dmac_sram_base = NULL;
static sys_spinlock reg_lock;

#define X2A_SYS_CTRL_BASE   0xA1000000
#define X2A_SYS_CTRL_SIZE   0x1000
#define X2A_IPS_BASE        0xA4000000
#define X2A_IPS_SIZE        0x1000
// for DMA ISP register access
#define X2A_DMAC_ISP_BASE   0xA100D000
#define X2A_DMAC_ISP_SIZE   0x1000
#define X2A_DMAC_SRAM_BASE   0x80400000
#define X2A_DMAC_SRAM_SIZE   0x80000
///////////////

int32_t init_hw_io( resource_size_t addr , resource_size_t size )
{
    p_hw_base = ioremap_nocache( addr, size );
    p_hw_sysctrl_base = ioremap_nocache( X2A_SYS_CTRL_BASE, X2A_SYS_CTRL_SIZE );
    p_hw_ips_base = ioremap_nocache( X2A_IPS_BASE, X2A_IPS_SIZE );
    p_hw_dmac_isp_base = ioremap_nocache( X2A_DMAC_ISP_BASE, X2A_DMAC_ISP_SIZE );
    p_hw_dmac_sram_base = ioremap_nocache( X2A_DMAC_SRAM_BASE, X2A_DMAC_SRAM_SIZE );
    system_spinlock_init( &reg_lock );
    if ( p_hw_base == NULL ) {
        LOG( LOG_CRIT, "failed to map isp memory" );
        return -1;
    }
    if ( p_hw_sysctrl_base == NULL ) {
        LOG( LOG_CRIT, "failed to map sys_ctrl memory" );
        return -1;
    }
    if ( p_hw_ips_base == NULL ) {
        LOG( LOG_CRIT, "failed to map ips memory" );
        return -1;
    }
    if ( p_hw_dmac_isp_base == NULL ) {
        LOG( LOG_CRIT, "failed to map dmac_isp memory" );
        return -1;
    }
    if ( p_hw_dmac_sram_base == NULL ) {
        LOG( LOG_CRIT, "failed to map dmac_isp memory" );
        return -1;
    }

    return 0;
}

int32_t close_hw_io( void )
{
    int32_t result = 0;
    LOG( LOG_DEBUG, "IO functionality has been closed" );
    iounmap( p_hw_base );
    iounmap( p_hw_sysctrl_base );
    iounmap( p_hw_ips_base );
    iounmap( p_hw_dmac_isp_base );
    iounmap( p_hw_dmac_sram_base );
    system_spinlock_destroy( reg_lock );
    return result;
}

#if FW_USE_HOBOT_DMA
#define REG_ACCESS_WARNING        \
    if(dma_isp_reg_dma_sram_ch_en_read()) {  \
        printk("%s: warning : access isp_reg when dma enable\n", __FUNCTION__);    \
    }
#else
#define REG_ACCESS_WARNING
#endif

uint32_t system_hw_read_32( uintptr_t addr )
{
    REG_ACCESS_WARNING;
    uint32_t result = 0;
    if ( p_hw_base != NULL ) {
        unsigned long flags;
        flags = system_spinlock_lock( reg_lock );
        result = ioread32( p_hw_base + addr );
        system_spinlock_unlock( reg_lock, flags );
    } else {
        LOG( LOG_ERR, "Failed to read memory from address %d. Base pointer is null ", addr );
    }
    return result;
}

uint32_t system_ctrl_hw_read_32( uintptr_t addr )
{
    uint32_t result = 0;
    if ( p_hw_sysctrl_base != NULL ) {
        unsigned long flags;
        flags = system_spinlock_lock( reg_lock );
        result = ioread32( p_hw_sysctrl_base + addr );
        system_spinlock_unlock( reg_lock, flags );
    } else {
        LOG( LOG_ERR, "Failed to read memory from system ctrl address %d. Base pointer is null ", addr );
    }
    return result;
}

uint32_t system_ips_hw_read_32( uintptr_t addr )
{
    uint32_t result = 0;
    if ( p_hw_ips_base != NULL ) {
        unsigned long flags;
        flags = system_spinlock_lock( reg_lock );
        result = ioread32( p_hw_ips_base + addr );
        system_spinlock_unlock( reg_lock, flags );
    } else {
        LOG( LOG_ERR, "Failed to read memory from ips address %d. Base pointer is null ", addr );
    }
    return result;
}

uint32_t system_dmac_isp_hw_read_32( uintptr_t addr )
{
    uint32_t result = 0;
    if ( p_hw_dmac_isp_base != NULL ) {
        result = ioread32( p_hw_dmac_isp_base + addr );
    } else {
        LOG( LOG_CRIT, "Failed to read memory from dmac isp address %d. Base pointer is null ", addr );
    }
    return result;
}

uint32_t system_dmac_sram_read_32( uintptr_t addr )
{
    uint32_t result = 0;
    if ( p_hw_dmac_sram_base != NULL ) {
        result = ioread32( p_hw_dmac_sram_base + addr );
    } else {
        LOG( LOG_CRIT, "Failed to read memory from dmac sram address %p. Base pointer is null ", addr );
    }
    return result;
}


uint16_t system_hw_read_16( uintptr_t addr )
{
    REG_ACCESS_WARNING;
    uint16_t result = 0;
    if ( p_hw_base != NULL ) {
        unsigned long flags;
        flags = system_spinlock_lock( reg_lock );
        result = ioread16( p_hw_base + addr );
        system_spinlock_unlock( reg_lock, flags );
    } else {
        LOG( LOG_ERR, "Failed to read memory from address %d. Base pointer is null ", addr );
    }
    return result;
}

uint8_t system_hw_read_8( uintptr_t addr )
{
    REG_ACCESS_WARNING;
    uint8_t result = 0;
    if ( p_hw_base != NULL ) {
        unsigned long flags;
        flags = system_spinlock_lock( reg_lock );
        result = ioread8( p_hw_base + addr );
        system_spinlock_unlock( reg_lock, flags );
    } else {
        LOG( LOG_ERR, "Failed to read memory from address %d. Base pointer is null ", addr );
    }
    return result;
}

uint32_t check_offset = 16;


#if HOBOT_REGISTER_MONITOR
#define HRM_RC_HW_BASE   0x11100000
void hobot_rm_check_n_record(uint32_t offset, uint32_t value);
#endif

void system_hw_write_32( uintptr_t addr, uint32_t data )
{
    REG_ACCESS_WARNING;
#if HOBOT_REGISTER_MONITOR
    hobot_rm_check_n_record((uint32_t)(uint64_t)(((uint8_t*)addr)+HRM_RC_HW_BASE) ,data);
#endif

    if ( p_hw_base != NULL ) {
        void *ptr = (void *)( p_hw_base + addr );
        unsigned long flags;
        flags = system_spinlock_lock( reg_lock );
        iowrite32( data, ptr );
        system_spinlock_unlock( reg_lock, flags );
    } else {
        LOG( LOG_ERR, "Failed to write value %d to memory with offset %d. Base pointer is null ", data, addr );
    }
}

void system_ctrl_hw_write_32( uintptr_t addr, uint32_t data )
{
#if HOBOT_REGISTER_MONITOR
    hobot_rm_check_n_record((uint32_t)(uint64_t)(((uint8_t*)addr)+HRM_RC_HW_BASE) ,data);
#endif
    if ( p_hw_sysctrl_base != NULL ) {
        void *ptr = (void *)( p_hw_sysctrl_base + addr );
        unsigned long flags;
        flags = system_spinlock_lock( reg_lock );
        iowrite32( data, ptr );
        system_spinlock_unlock( reg_lock, flags );
    } else {
        LOG( LOG_ERR, "Failed to write value %d to memory with system ctrl offset %d. Base pointer is null ", data, addr );
    }
}

void system_dmac_isp_hw_write_32( uintptr_t addr, uint32_t data )
{
    if ( p_hw_dmac_isp_base != NULL ) {
        void *ptr = (void *)( p_hw_dmac_isp_base + addr );
        iowrite32( data, ptr );
    } else {
        LOG( LOG_CRIT, "Failed to write value %d to memory with dmac isp offset %d. Base pointer is null ", data, addr );
    }
}

void system_hw_write_16( uintptr_t addr, uint16_t data )
{
    REG_ACCESS_WARNING;
#if HOBOT_REGISTER_MONITOR
    hobot_rm_check_n_record((uint32_t)(uint64_t)(((uint8_t*)addr)+HRM_RC_HW_BASE) ,data);
#endif

    if ( p_hw_base != NULL ) {
        void *ptr = (void *)( p_hw_base + addr );
        unsigned long flags;
        flags = system_spinlock_lock( reg_lock );
        iowrite16( data, ptr );
        system_spinlock_unlock( reg_lock, flags );
    } else {
        LOG( LOG_ERR, "Failed to write value %d to memory with offset %d. Base pointer is null ", data, addr );
    }
}

void system_hw_write_8( uintptr_t addr, uint8_t data )
{
    REG_ACCESS_WARNING;
#if HOBOT_REGISTER_MONITOR
    hobot_rm_check_n_record((uint32_t)(uint64_t)(((uint8_t*)addr)+HRM_RC_HW_BASE) ,data);
#endif

    if ( p_hw_base != NULL ) {
        void *ptr = (void *)( p_hw_base + addr );
        unsigned long flags;
        flags = system_spinlock_lock( reg_lock );
        iowrite8( data, ptr );
        system_spinlock_unlock( reg_lock, flags );
    } else {
        LOG( LOG_ERR, "Failed to write value %d to memory with offset %d. Base pointer is null ", data, addr );
    }
}
