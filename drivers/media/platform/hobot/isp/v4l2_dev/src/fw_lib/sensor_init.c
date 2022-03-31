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

#include "acamera_types.h"
#include "sensor_init.h"
#include "system_timer.h"
#include "acamera_logger.h"
#include "system_stdlib.h"
#include "acamera_firmware_config.h"

void acamera_load_array_sequence( acamera_sbus_ptr_t p_sbus, uintptr_t isp_offset, uint32_t size, const acam_reg_t **sequence, int group )
{
    const acam_reg_t *seq = sequence[group];
    uint32_t end_seq = 0;

    while ( end_seq == 0 ) {
        uint32_t cmd = seq->address;
        uint32_t val = seq->value;
        if ( seq->len ) //overide size if it is valid
            size = seq->len;
        switch ( cmd ) {
        case 0xFFFF: //wait command
            //time is given in milliseconds. convert to microseconds interval
            val *= 1000;
            system_timer_usleep( val );
            break;
        case 0x0000: //sequence end flag
            if ( seq->len == 0 && seq->value == 0 ) {
                end_seq = 1;
                break;
            }
        default:
            if ( size == 4 ) {
                if ( seq->mask ) {
                    uint32_t sdata = acamera_sbus_read_u32( p_sbus, seq->address + isp_offset );
                    uint32_t wdata = seq->value;
                    uint32_t mask = seq->mask;
                    val = ( sdata & ~mask ) | ( wdata & mask );
                }
                acamera_sbus_write_u32( p_sbus, seq->address + isp_offset, val );
                LOG( LOG_DEBUG, "A32: 0x%x : 0x%x", seq->address + isp_offset, val );
            } else if ( size == 2 ) {
                if ( seq->mask ) {
                    uint16_t sdata = acamera_sbus_read_u16( p_sbus, seq->address + isp_offset );
                    uint16_t wdata = (uint16_t)seq->value;
                    uint16_t mask = (uint16_t)seq->mask;
                    val = ( uint16_t )( ( sdata & ~mask ) | ( wdata & mask ) );
                }
                acamera_sbus_write_u16( p_sbus, seq->address + isp_offset, (uint16_t)val );
                LOG( LOG_DEBUG, "A16: 0x%x : 0x%x", seq->address + isp_offset, (uint16_t)val );
            } else if ( size == 1 ) {
                if ( seq->mask ) {
                    uint8_t sdata = acamera_sbus_read_u8( p_sbus, seq->address + isp_offset );
                    uint8_t wdata = (uint8_t)seq->value;
                    uint8_t mask = (uint8_t)seq->mask;
                    val = ( uint8_t )( ( sdata & ~mask ) | ( wdata & mask ) );
                }
                acamera_sbus_write_u8( p_sbus, seq->address + isp_offset, (uint8_t)val );
                LOG( LOG_DEBUG, "A16: 0x%x : 0x%x", seq->address + isp_offset, (uint8_t)val );
            } else {
                LOG( LOG_ERR, "Invalid size %d", size );
            }
            break;
        }
        seq++;
    }
}