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

//needed for gdc/gdc configuration
#include "acamera_gdc_config.h"

//data types and prototypes
#include "acamera_gdc_api.h"

//system_memcpy
#include "system_stdlib.h"
#include "system_log.h"
#include "gdc_drv.h"


/**
 *   Configure the output gdc configuration address/size and buffer address/size; and resolution.
 *
 *   More than one gdc settings can be accessed by index to a gdc_config_t.
 *
 *   @return 0 - success
 *           -1 - fail.
 */
int acamera_gdc_init( gdc_settings_t *gdc_settings )
{
    gdc_settings->is_waiting_gdc = 0;
    if ( gdc_settings->gdc_config.output_width == 0 || gdc_settings->gdc_config.output_height == 0 ) {
        LOG( LOG_ERR, "Wrong GDC output resolution.\n" );
        return -1;
    }
    //stop gdc
    acamera_gdc_gdc_start_flag_write( gdc_settings->base_gdc, 0 );
    //set the configuration address and size to the gdc block
    acamera_gdc_gdc_config_addr_write( gdc_settings->base_gdc, gdc_settings->gdc_config.config_addr );
    LOG(LOG_INFO,"config_addr:%lx\n", gdc_settings->gdc_config.config_addr);
    acamera_gdc_gdc_config_size_write( gdc_settings->base_gdc, gdc_settings->gdc_config.config_size );
    LOG(LOG_INFO,"config_size:%lx\n", gdc_settings->gdc_config.config_size);

    //set the gdc in and output resolution
    acamera_gdc_gdc_datain_width_write( gdc_settings->base_gdc, gdc_settings->gdc_config.input_width );
    acamera_gdc_gdc_datain_height_write( gdc_settings->base_gdc, gdc_settings->gdc_config.input_height );
    acamera_gdc_gdc_dataout_width_write( gdc_settings->base_gdc, gdc_settings->gdc_config.output_width );
    acamera_gdc_gdc_dataout_height_write( gdc_settings->base_gdc, gdc_settings->gdc_config.output_height );
    LOG(LOG_INFO,"gdc in w:%d, h:%d\n", gdc_settings->gdc_config.input_width, gdc_settings->gdc_config.input_height);
    LOG(LOG_INFO,"gdc out w:%d, h:%d\n", gdc_settings->gdc_config.output_width, gdc_settings->gdc_config.output_height);

    return 0;
}

/**
 *   This function stops the gdc block
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 */
void acamera_gdc_stop( gdc_settings_t *gdc_settings )
{
    gdc_settings->is_waiting_gdc = 0;
    acamera_gdc_gdc_start_flag_write( gdc_settings->base_gdc, 0 );
}

/**
 *   This function starts the gdc block
 *
 *   Writing 0->1 transition is necessary for trigger
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 */
void acamera_gdc_start( gdc_settings_t *gdc_settings )
{
    acamera_gdc_gdc_start_flag_write( gdc_settings->base_gdc, 0 ); //do a stop for sync
    acamera_gdc_gdc_start_flag_write( gdc_settings->base_gdc, 1 );
    gdc_settings->is_waiting_gdc = 1;
}


/**
 *   This function points gdc to its input resolution and yuv address and offsets
 *
 *   Shown inputs to GDC are Y and UV plane address and offsets
 *
 *
 *   @return 0 - success
 *           -1 - no interrupt from GDC.
 */

int acamera_gdc_process( gdc_settings_t *gdc_settings, uint32_t num_input, uint32_t * input_addr)
{
    if ( !gdc_settings->is_waiting_gdc ) {
        LOG(LOG_DEBUG,"starting GDC process, plain_num:%d.\n", num_input);
        if(num_input==0 || num_input>ACAMERA_GDC_MAX_INPUT){
            LOG(LOG_CRIT,"GDC number of input invalid %d.\n",num_input);
            return -1;
        }

        if(num_input<gdc_settings->gdc_config.total_planes){
            LOG(LOG_CRIT,"GDC number of input less than planes %d.\n",num_input);
            return -1;
        }

        uint32_t lineoffset,height;//, size;
        //process input addresses
        if(num_input>=1) {
            lineoffset = gdc_settings->gdc_config.input_stride;
            height=gdc_settings->gdc_config.input_height;
            if(gdc_settings->gdc_config.sequential_mode==1) {
                acamera_gdc_gdc_data1in_addr_write( gdc_settings->base_gdc,input_addr[gdc_settings->seq_planes_pos]);
                if(gdc_settings->seq_planes_pos>0) { //UV planes
                    lineoffset = gdc_settings->gdc_config.output_width >> gdc_settings->gdc_config.div_width;
                    height = gdc_settings->gdc_config.output_height >> gdc_settings->gdc_config.div_height;
                }
            } else {
                acamera_gdc_gdc_data1in_addr_write( gdc_settings->base_gdc,input_addr[0]);
                LOG(LOG_INFO,"input1 addr:%lx\n", input_addr[0]);
            }
            acamera_gdc_gdc_data1in_line_offset_write( gdc_settings->base_gdc, lineoffset );
            LOG(LOG_INFO,"data1in lineoffset:%d, hight:%d\n", lineoffset, height);
        }
        if(num_input>=2 && gdc_settings->gdc_config.sequential_mode==0) { //only processed if not in toggle mode
            LOG(LOG_INFO,"plain_num:%d case\n", num_input);
            lineoffset = gdc_settings->gdc_config.input_stride >> gdc_settings->gdc_config.div_width;
            height = gdc_settings->gdc_config.input_height >> gdc_settings->gdc_config.div_height;

            acamera_gdc_gdc_data2in_addr_write( gdc_settings->base_gdc, input_addr[1]);
            acamera_gdc_gdc_data2in_line_offset_write( gdc_settings->base_gdc,lineoffset );
            LOG(LOG_INFO,"input2 addr:%lx\n", input_addr[1]);
            LOG(LOG_INFO,"data2in lineoffset:%d, hight:%d\n", lineoffset, height);
        }
        if(num_input>=3 && gdc_settings->gdc_config.sequential_mode==0) { //only processed if not in toggle mode
            lineoffset = gdc_settings->gdc_config.input_stride >> gdc_settings->gdc_config.div_width;
            height = gdc_settings->gdc_config.input_height >> gdc_settings->gdc_config.div_height;
            acamera_gdc_gdc_data3in_addr_write( gdc_settings->base_gdc, input_addr[2]);
            acamera_gdc_gdc_data3in_line_offset_write( gdc_settings->base_gdc, lineoffset );
        }

        //outputs
        if(num_input>=1) {
            lineoffset = gdc_settings->gdc_config.output_stride;
            height=gdc_settings->gdc_config.output_height;
            LOG(LOG_INFO,"out1: plain_num:%d case, lineoffset:%d, hight:%d\n", num_input, lineoffset, height);
            if(gdc_settings->gdc_config.sequential_mode==1 && gdc_settings->seq_planes_pos>0){ //UV planes
                lineoffset = gdc_settings->gdc_config.output_stride >> gdc_settings->gdc_config.div_width;
                height = gdc_settings->gdc_config.output_height >> gdc_settings->gdc_config.div_height;
            }
            acamera_gdc_gdc_data1out_addr_write( gdc_settings->base_gdc, gdc_settings->Out_buffer_addr[0]);
            acamera_gdc_gdc_data1out_line_offset_write( gdc_settings->base_gdc, lineoffset );
            LOG(LOG_INFO,"out1: data1out_addr_write:%lx\n",gdc_settings->Out_buffer_addr[0]);
        }

        if(num_input>=2 && gdc_settings->gdc_config.sequential_mode==0){
            lineoffset = gdc_settings->gdc_config.output_stride >> gdc_settings->gdc_config.div_width;
            height = gdc_settings->gdc_config.output_height >> gdc_settings->gdc_config.div_height;
            LOG(LOG_INFO,"out2: plain_num:%d case, lineoffset:%d, hight:%d\n", num_input, lineoffset, height);
            acamera_gdc_gdc_data2out_addr_write( gdc_settings->base_gdc, gdc_settings->Out_buffer_addr[1]);
            acamera_gdc_gdc_data2out_line_offset_write( gdc_settings->base_gdc, lineoffset );
            LOG(LOG_INFO,"out2: data2out_addr_write:%lx\n",gdc_settings->Out_buffer_addr[1]);
        }

        if(num_input>=3 && gdc_settings->gdc_config.sequential_mode==0){
            lineoffset = gdc_settings->gdc_config.output_width >> gdc_settings->gdc_config.div_width;
            height = gdc_settings->gdc_config.output_height >> gdc_settings->gdc_config.div_height;
            acamera_gdc_gdc_data3out_addr_write( gdc_settings->base_gdc, gdc_settings->Out_buffer_addr[1]);
            acamera_gdc_gdc_data3out_line_offset_write( gdc_settings->base_gdc, lineoffset );
            LOG(LOG_INFO,"out2: data2out_addr_write:%lx\n",gdc_settings->Out_buffer_addr[1]);
        }

        if(gdc_settings->gdc_config.sequential_mode==1){ //update the planes position
            if(++(gdc_settings->seq_planes_pos)>=gdc_settings->gdc_config.total_planes)
                gdc_settings->seq_planes_pos=0;
        }

        acamera_gdc_start(gdc_settings);

        return 0;
    } else {
        acamera_gdc_gdc_start_flag_write( gdc_settings->base_gdc, 0 );
        LOG( LOG_CRIT, "No interrupt from GDC block. Still waiting...\n" );
        acamera_gdc_gdc_start_flag_write( gdc_settings->base_gdc, 1 );
        return -1;
    }
}

int acamera_gdc_get_status(gdc_settings_t *gdc_settings)
{
    uint32_t gdc_status = acamera_gdc_gdc_status_read(gdc_settings->base_gdc);
    uint32_t status_result = gdc_done;
    if(gdc_status) {
        if(gdc_status & ACAMERA_GDC_GDC_BUSY_MASK)
            status_result = gdc_busy;
        else if(gdc_status & ACAMERA_GDC_GDC_ERROR_MASK) {
            status_result = gdc_error;
            if(gdc_status & ACAMERA_GDC_GDC_CONFIGURATION_ERROR_MASK)
                status_result = gdc_config_error;
            else if(gdc_status & ACAMERA_GDC_GDC_USER_ABORT_MASK)
                status_result = gdc_abort;
            else if(gdc_status & ACAMERA_GDC_GDC_AXI_READER_ERROR_MASK)
                status_result = gdc_axi_rd_err;
            else if(gdc_status & ACAMERA_GDC_GDC_AXI_WRITER_ERROR_MASK)
                status_result = gdc_axi_wt_err;
            else if(gdc_status & ACAMERA_GDC_GDC_UNALIGNED_ACCESS_MASK)
                status_result = gdc_unaliged;
            else if(gdc_status & ACAMERA_GDC_GDC_INCOMPATIBLE_CONFIGURATION_MASK)
                status_result = gdc_incompatible_config;
        } else {
            status_result = gdc_unknown_err;
        }
    }
    LOG(LOG_DEBUG,"receive gdc interrupt (gdc_status=0x%x, status_result=%d)",
        gdc_status,status_result);
    return status_result;
}
