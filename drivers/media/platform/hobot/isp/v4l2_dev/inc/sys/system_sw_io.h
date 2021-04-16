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

#ifndef __system_sw_io_H__
#define __system_sw_io_H__

#include "acamera_types.h"
#include "acamera_firmware_config.h"

void *system_sw_alloc( uint32_t size );
void system_sw_free( void *ptr );
void *system_sw_get_dma_addr(int chn_idx);
void *system_sw_alloc_dma_sram( uint32_t size , uint32_t context_id, uint32_t *phy_addr);
void system_sw_free_dma_sram( void *ptr ,uint32_t context_id);

extern int swreg_access_debug;
extern void system_sram_access_assert(uintptr_t addr);


/**
 *   Read 32 bit word from isp memory
 *
 *   This function returns a 32 bits word from ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to read 32 bits word.
 *                 Correct values from 0 to ACAMERA_sw_MAX_ADDR
 *
 *   @return 32 bits memory value
 */
static inline uint32_t system_sw_read_32( uintptr_t addr )
{
#if HOBOT_REGISTER_MONITOR
    if (swreg_access_debug)
        system_sram_access_assert(addr);
#endif
    uint32_t result = 0;
    if ( (void *)addr != NULL ) {
        volatile uint32_t *p_addr = (volatile uint32_t *)( addr );
        result = *p_addr;
    } else {
        LOG( LOG_ERR, "Failed to read memory from address 0x%x. Base pointer is null ", addr );
    }
    return result;
}

/**
 *   Read 16 bit word from isp memory
 *
 *   This function returns a 16 bits word from ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to read 16 bits word.
 *                 Correct values from 0 to ACAMERA_sw_MAX_ADDR
 *
 *   @return 16 bits memory value
 */
static inline uint16_t system_sw_read_16( uintptr_t addr )
{
#if HOBOT_REGISTER_MONITOR
    if (swreg_access_debug)
        system_sram_access_assert(addr);
#endif
    uint16_t result = 0;
    if ( (void *)addr != NULL ) {
        volatile uint16_t *p_addr = (volatile uint16_t *)( addr );
        result = *p_addr;
    } else {
        LOG( LOG_ERR, "Failed to read memory from address 0x%x. Base pointer is null ", addr );
    }
    return result;
}

/**
 *   Read 8 bit word from isp memory
 *
 *   This function returns a 8 bits word from ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to read 8 bits word.
 *                 Correct values from 0 to ACAMERA_sw_MAX_ADDR
 *
 *   @return 8 bits memory value
 */
static inline uint8_t system_sw_read_8( uintptr_t addr )
{
#if HOBOT_REGISTER_MONITOR
    if (swreg_access_debug)
        system_sram_access_assert(addr);
#endif
    uint8_t result = 0;
    if ( (void *)addr != NULL ) {
        volatile uint8_t *p_addr = (volatile uint8_t *)( addr );
        result = *p_addr;
    } else {
        LOG( LOG_ERR, "Failed to read memory from address 0x%x. Base pointer is null ", addr );
    }
    return result;
}

/**
 *   Write 32 bits word to isp memory
 *
 *   This function writes a 32 bits word to ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to write data.
 *                 Correct values from 0 to ACAMERA_sw_MAX_ADDR.
 *   @param data - data to be written
 */
static inline void system_sw_write_32( uintptr_t addr, uint32_t data )
{
#if HOBOT_REGISTER_MONITOR
    hobot_rm_check_n_record((uint32_t)(((uint8_t*)addr) - g_sw_isp_base[0].base + HRM_RC_SW_BASE) ,data);
    if (swreg_access_debug)
        system_sram_access_assert(addr);
#endif
    if ( (void *)addr != NULL ) {
        volatile uint32_t *p_addr = (volatile uint32_t *)( addr );
        *p_addr = data;
    } else {
        LOG( LOG_ERR, "Failed to write %d to memory 0x%x. Base pointer is null ", data, addr );
    }
}

/**
 *   Write 16 bits word to isp memory
 *
 *   This function writes a 16 bits word to ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to write data.
 *                 Correct values from 0 to ACAMERA_sw_MAX_ADDR.
 *   @param data - data to be written
 */
static inline void system_sw_write_16( uintptr_t addr, uint16_t data )
{
#if HOBOT_REGISTER_MONITOR
        hobot_rm_check_n_record((uint32_t)(((uint8_t*)addr) - g_sw_isp_base[0].base + HRM_RC_SW_BASE) ,data);
    if (swreg_access_debug)
        system_sram_access_assert(addr);
#endif
    if ( (void *)addr != NULL ) {
        volatile uint16_t *p_addr = (volatile uint16_t *)( addr );
        *p_addr = data;
    } else {
        LOG( LOG_ERR, "Failed to write %d to memory 0x%x. Base pointer is null ", data, addr );
    }
}

/**
 *   Write 8 bits word to isp memory
 *
 *   This function writes a 8 bits word to ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to write data.
 *                 Correct values from 0 to ACAMERA_sw_MAX_ADDR.
 *   @param data - data to be written
 */

static inline void system_sw_write_8( uintptr_t addr, uint8_t data )
{
#if HOBOT_REGISTER_MONITOR
        hobot_rm_check_n_record((uint32_t)(((uint8_t*)addr) - g_sw_isp_base[0].base + HRM_RC_SW_BASE) ,data);
    if (swreg_access_debug)
        system_sram_access_assert(addr);
#endif
    if ( (void *)addr != NULL ) {
        volatile uint8_t *p_addr = (volatile uint8_t *)( addr );
        *p_addr = data;
    } else {
        LOG( LOG_ERR, "Failed to write %d to memory 0x%x. Base pointer is null ", data, addr );
    }
}


//if writer (value & mask)
void hobot_rm_add(uint32_t offset, uint32_t value, uint32_t mask, uint32_t is_write, uint32_t trigger_times);
void hobot_rm_init( void );



#endif /* __system_sw_io_H__ */
