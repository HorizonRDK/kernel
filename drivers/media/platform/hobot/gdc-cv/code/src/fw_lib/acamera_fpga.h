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

#ifndef __ACAMERA_FPGA_H__
#define __ACAMERA_FPGA_H__

//configure the output gdc configuration address/size and buffer address/size; and resolution
#include "system_stdlib.h"

/**
 *   FPGA initialization with resolution and input planar addresses
 *
 *   @param  active_width - frame reader output width resolution
 *   @param  active_height - frame reader output width resolution
 *   @param  total_input - number of addresses as input
 *   @param  in_addr - array of address
 *   @param  in_lineoffset - array of line offsets
 *
 *   @return 0 - success
 *           -1 - fail.
 */
int acamera_fpga_init( uint32_t active_width, uint32_t active_height,uint32_t total_input,uint32_t * in_addr,uint32_t * in_lineoffset );
/**
 *   Update frame reader yuv address and the line offsets
 *
 *   @param  total_input - number of addresses as input
 *   @param  out_addr - array of address to update fpga
 *   @param  out_lineoffset - array of line offsets to update fpga
 *
 */

void acamera_fpga_update_frame_reader(  uint32_t total_input, uint32_t * out_addr, uint32_t * out_lineoffset );
/**
 *   Get the recent frame writer settings
 *
 *   @param  total_input - number of addresses as input
 *   @param  in_addr - array of address is saved here
 *   @param  in_lineoffset - array of line offsets is saved here
 *
 *   @return 0 - success
 *           -1 - fail.
 *
 */
int acamera_fpga_get_frame_writer( uint32_t total_input,uint32_t * in_addr,uint32_t * in_lineoffset );

#endif
