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
#include "acamera_driver_config.h"
#include "system_interrupts.h"
#include "system_control.h"
#include "system_stdlib.h"
#include "system_log.h"

#include "acamera_gdc_config.h"
#include "gdc_drv.h"

//gdc api functions
#include "acamera_gdc_api.h"

#if HAS_FPGA_WRAPPER
//fpga related functions
#include "acamera_fpga.h"
#endif


//gdc configuration sequences
#include "gdc_config_seq_semiplanar_yuv420.h"
#include "gdc_config_seq_plane_y.h"
#include "gdc_config_seq_planar_yuv420.h"
#include "gdc_config_seq_planar_rgb444.h"

static gdc_settings_t _gdc_settings;
static gdc_status_t _gdc_status;

static void interrupt_handler( void *param, uint32_t mask )
{
    //write 1 to clear gdc interrupt
    x2a_dwe_int_gdc_status_write(1);

    // update frame status
    gdc_settings_t *gdc_settings = (gdc_settings_t *)param;
    _gdc_status.status_result = acamera_gdc_get_status(gdc_settings);
    LOG(LOG_INFO, "IE&E %s-- %d,status is 0x%x ", __func__,  __LINE__, _gdc_status.status_result);
    if(_gdc_status.status_result != gdc_busy)
        gdc_settings->is_waiting_gdc = 0;
}

// The basic example of usage gdc is given below.
int gdc_fw_init( void )
{
    memset(&_gdc_status,0,sizeof(gdc_status_t));
    memset(&_gdc_settings,0,sizeof(gdc_settings_t));

    LOG(LOG_INFO, "IE&E %s--%d is running ", __func__,  __LINE__);
    bsp_init();
    acamera_gdc_stop( &_gdc_settings );
    system_interrupt_set_handler( interrupt_handler, &_gdc_settings );

    //enable the interrupts
    system_interrupts_enable();
    return 0;
}

int gdc_process(gdc_process_settings_t *gdc_process_setting)
{

    if((_gdc_status.status_result == gdc_busy)||(_gdc_settings.is_waiting_gdc)) {
        LOG(LOG_ERR, "GDC is still busy (sensor_id=%d, frame_id=%d, status=%d, is_waiting_gdc=%d)",
            _gdc_status.gdc_process_setting.image_info.sensor_id,
            _gdc_status.gdc_process_setting.image_info.frame_id,
            _gdc_status.status_result, _gdc_settings.is_waiting_gdc);
        return -1;
    }

    // 1. configure gdc config, buffer address and resolution
    _gdc_settings.base_gdc = 0;              // not use right now
    _gdc_settings.Out_buffer_addr[0] = gdc_process_setting->gdc_out.buffer_out_addr_phy[0];
    _gdc_settings.Out_buffer_addr[1] = gdc_process_setting->gdc_out.buffer_out_addr_phy[1];
    _gdc_settings.Out_buffer_addr[2] = gdc_process_setting->gdc_out.buffer_out_addr_phy[2];
    //_gdc_settings.buffer_size = gdc_process_setting->gdc_out.buffer_out_size;
    _gdc_settings.seq_planes_pos = 0;

    // 2. set the gdc config
    _gdc_settings.gdc_config.config_addr = gdc_process_setting->gdc_config.config_addr_phy;
    _gdc_settings.gdc_config.config_size = gdc_process_setting->gdc_config.config_size/4;        //size of configuration in 4bytes
    _gdc_settings.gdc_config.input_width = gdc_process_setting->gdc_in.input_width;
    _gdc_settings.gdc_config.input_height = gdc_process_setting->gdc_in.input_height;
    _gdc_settings.gdc_config.input_stride = gdc_process_setting->gdc_in.input_stride;
    _gdc_settings.gdc_config.output_width = gdc_process_setting->gdc_out.output_width;
    _gdc_settings.gdc_config.output_height = gdc_process_setting->gdc_out.output_height;
    _gdc_settings.gdc_config.output_stride = gdc_process_setting->gdc_out.output_stride;
    _gdc_settings.gdc_config.total_planes = gdc_process_setting->total_planes;
    _gdc_settings.gdc_config.sequential_mode= 0; //gdc_process_setting->sequential_mode;
    _gdc_settings.gdc_config.div_width = gdc_process_setting->div_width;
    _gdc_settings.gdc_config.div_height = gdc_process_setting->div_height;

    // 3. initialise the gdc by the first configuration
    if ( acamera_gdc_init( &_gdc_settings ) != 0 ) {
        LOG( LOG_ERR, "Failed to initialise GDC block" );
        return -1;
    }
    LOG( LOG_INFO, "Done gdc config..\n" );

    // 4. start gdc process
    if(acamera_gdc_process( &_gdc_settings,
                         gdc_process_setting->total_planes,
                         gdc_process_setting->gdc_in.input_addresses_phy) !=0) {
        LOG( LOG_ERR, "Failed to process GDC" );
        return -1;
    }
    LOG( LOG_INFO, "Done gdc process..\n" );
    _gdc_status.gdc_process_setting = *gdc_process_setting;
    return 0;
}

void gdc_get_status(gdc_status_t *status_update)
{
    uint32_t gdc_status_reg = acamera_gdc_gdc_status_read(0);
    LOG(LOG_INFO, "status reg = 0x%x", gdc_status_reg);
    status_update->status_result = _gdc_status.status_result;
}

void gdc_force_stop(void)
{
    acamera_gdc_stop(&_gdc_settings);
}

int gdc_fw_exit(void)
{
    LOG(LOG_INFO, "IE&E  -- %s--%d is runing", __func__, __LINE__);
    acamera_gdc_stop( &_gdc_settings );
    //system_interrupts_disable();
    bsp_destroy();
    return 0;
}
