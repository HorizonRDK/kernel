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

#ifndef __ACAMERA_GDC_API_H__
#define __ACAMERA_GDC_API_H__

#include "system_stdlib.h"

#define ACAMERA_GDC_MAX_INPUT 3

// each configuration addresses and size
typedef struct gdc_config {
    uint32_t config_addr;   //gdc config address
    uint32_t config_size;   //gdc config size in 32bit
    uint32_t input_width;  //gdc input width resolution
    uint32_t input_height; //gdc input height resolution
    uint32_t input_stride;  //gdc input stride (pixel)
    uint32_t output_width;  //gdc output width resolution
    uint32_t output_height; //gdc output height resolution
    uint32_t output_stride;  //gdc output stride (pixel)
    uint8_t  div_width;     //use in dividing UV dimensions; actually a shift right
    uint8_t  div_height;    //use in dividing UV dimensions; actually a shift right
    uint32_t total_planes;
    uint8_t sequential_mode; //sequential processing
} gdc_config_t;

// overall gdc settings and state
typedef struct gdc_settings {
    uint32_t base_gdc;        //writing/reading to gdc base address, currently not read by api
    gdc_config_t gdc_config; //array of gdc configuration and sizes

    uint32_t Out_buffer_addr[ACAMERA_GDC_MAX_INPUT];     //start memory to write gdc output framse
    int is_waiting_gdc;       //set when expecting an interrupt from gdc

    uint32_t total_planes;
    uint8_t seq_planes_pos; //sequential plance current index
} gdc_settings_t;

/**
 *   Configure the output gdc configuration address/size and buffer address/size; resolution; and sequential mode.
 *
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 *   @return 0 - success
 *           -1 - fail.
 */
int acamera_gdc_init( gdc_settings_t *gdc_settings );
/**
 *   This function stops the gdc block
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 */
void acamera_gdc_stop( gdc_settings_t *gdc_settings );

/**
 *   This function starts the gdc block
 *
 *   Writing 0->1 transition is necessary for trigger
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 */
void acamera_gdc_start( gdc_settings_t *gdc_settings );

/**
 *   This function points gdc to its input resolution and yuv address and offsets
 *
 *   Shown inputs to GDC are Y and UV plane address and offsets
 *
 *   @param  gdc_settings - overall gdc settings and state
 *   @param  num_input -  number of input addresses in the array to be processed by gdc
 *   @param  input_addr - input addresses in the array to be processed by gdc
 *
 *   @return 0 - success
 *           -1 - no interrupt from GDC.
 */

int acamera_gdc_process( gdc_settings_t *gdc_settings, uint32_t num_input, uint32_t * input_addr);
/**
 *   This function gets the GDC ouput frame addresses and offsets and updates the frame buffer via callback if it is available
 *
 *   Shown ouputs to GDC are Y and UV plane address and offsets
 *
 *   @param  gdc_settings - overall gdc settings and state
 *   @param  num_input -  number of input addresses in the array to be processed by gdc
 *
 *   @return 0 - success
 *           -1 - unexpected interrupt from GDC.
 */
//	int acamera_gdc_get_frame( gdc_settings_t *gdc_settings, uint32_t num_input );
int acamera_gdc_get_status(gdc_settings_t *gdc_settings);

#endif
