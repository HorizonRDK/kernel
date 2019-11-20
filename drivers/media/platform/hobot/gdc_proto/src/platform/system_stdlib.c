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

#include "system_stdlib.h"
#include "linux/string.h"

#include <asm/io.h>


void * system_ddr_mem_init() {
    void *p_base=0;
#if HAS_FPGA_WRAPPER
#define JUNO_LOGIC_TILE_DDR_OFFSET ( 0x64400000 )
#define JUNO_LOGIC_TILE_DDR_SIZE ( 0x06FFFFFF )
    p_base = ioremap(JUNO_LOGIC_TILE_DDR_OFFSET, JUNO_LOGIC_TILE_DDR_SIZE);
#endif
    return p_base ;
}

int32_t system_memcpy( void* dst, const void* src, uint32_t size ) {
    int32_t result = 0 ;
    memcpy( dst, src, size ) ;
    return result ;
}


int32_t system_memset( void* ptr, uint8_t value, uint32_t size ) {
    int32_t result = 0 ;
    memset( ptr, value, size ) ;
    return result ;
}
