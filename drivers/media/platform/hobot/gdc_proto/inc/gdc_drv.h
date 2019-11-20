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

#ifndef __GDC_DRV_H__
#define __GDC_DRV_H__

#define GDC_MAX_INPUT_PLANE 3
// each configuration addresses and size
typedef struct gdc_in_config {
    uint32_t config_addr_phy;   //gdc config address
    uint32_t config_size;       //gdc config size in 32bit
} gdc_in_config_t;

typedef struct gdc_in {
    uint32_t input_addresses_phy[GDC_MAX_INPUT_PLANE];
    uint32_t input_size[GDC_MAX_INPUT_PLANE];
    uint32_t input_width;       //gdc input width resolution
    uint32_t input_height;      //gdc input height resolution
    uint32_t input_stride;      //gdc input stride (pixel)
} gdc_in_t;

typedef struct gdc_out {
    uint32_t buffer_out_addr_phy[GDC_MAX_INPUT_PLANE];   //start memory to write gdc output frame
    uint32_t buffer_out_size[GDC_MAX_INPUT_PLANE];       //size of memory output frames to determine if it is enough and can do multiple write points
    uint32_t output_width;          //gdc output width resolution
    uint32_t output_height;         //gdc output height resolution
    uint32_t output_stride;         //gdc output stride (pixel)
} gdc_out_t;

typedef struct gdc_image_info {
    uint32_t sensor_id;             //sensor id
    uint32_t frame_id;              //frame id
    uint64_t time_stamp;            //time stamp
}gdc_image_info_t;

// overall gdc settings and state
typedef struct gdc_process_settings {
    gdc_image_info_t image_info;
    gdc_in_config_t gdc_config;        //array of gdc configuration and sizes
    gdc_in_t gdc_in;                //array of gdc input buffer and sizes
    gdc_out_t gdc_out;              //array of gdc output buffer and sizes
    uint32_t total_planes;

    // yuv420 semiplanar : div_width = 0; div_height = 1
    // y_plane           : div_width = 0; div_height = 0
    // yuv420            : div_width = 1; div_height = 1
    // rgb_444           : div_width = 0; div_height = 0
    uint8_t  div_width;             //use in dividing UV dimensions; actually a shift right
    uint8_t  div_height;            //use in dividing UV dimensions; actually a shift right
} gdc_process_settings_t;


enum gdc_status_result{
    gdc_done=0,
    gdc_busy,
    gdc_error,
    gdc_config_error,
    gdc_abort,
    gdc_unaliged,
    gdc_incompatible_config,
    gdc_axi_rd_err,
    gdc_axi_wt_err,
    gdc_unknown_err,
};

// gdc status
typedef struct gdc_status {
    gdc_process_settings_t gdc_process_setting;
    uint32_t status_result;         // gdc_status_result
} gdc_status_t;


//////////////
// gdc ioctl
#define GDC_IOCTL_MAGIC  'G'
#define GDC_IOCTL_PROCESS                           _IO(GDC_IOCTL_MAGIC, 0)
#define GDC_IOCTL_GET_STATUS                        _IO(GDC_IOCTL_MAGIC, 1)
#define GDC_IOCTL_FORCE_STOP                        _IO(GDC_IOCTL_MAGIC, 2)
#endif
