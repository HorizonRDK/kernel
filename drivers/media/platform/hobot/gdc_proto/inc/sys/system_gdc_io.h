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

#ifndef __SYSTEM_GDC_IO_H__
#define __SYSTEM_GDC_IO_H__

#include "system_stdlib.h"


/**
 *   Read 32 bit word from gdc memory
 *
 *   This function returns a 32 bits word from GDC memory with a given offset.
 *
 *   @param addr - the offset in GDC memory to read 32 bits word.
 *
 *   @return 32 bits memory value
 */
uint32_t system_gdc_read_32( uint32_t addr );


/**
 *   Write 32 bits word to gdc memory
 *
 *   This function writes a 32 bits word to GDC memory with a given offset.
 *
 *   @param addr - the offset in GDC memory to write data.
 *   @param data - data to be written
 */
void system_gdc_write_32( uint32_t addr, uint32_t data );
uint32_t system_dwe_read_32( uint32_t addr );
void system_dwe_write_32( uint32_t addr, uint32_t data );

#endif /* __SYSTEM_GDC_IO_H__ */
