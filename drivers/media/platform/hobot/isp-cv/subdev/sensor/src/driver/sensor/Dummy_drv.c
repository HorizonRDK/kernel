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

//-------------------------------------------------------------------------------------
//STRUCTURE:
//  VARIABLE SECTION:
//        CONTROLS - dependence from preprocessor
//        DATA     - modulation
//        RESET     - reset function
//        MIPI     - mipi settings
//        FLASH     - flash support
//  CONSTANT SECTION
//        DRIVER
//-------------------------------------------------------------------------------------

#include "acamera_types.h"
#include "acamera_logger.h"
#include "sensor_init.h"
#include "acamera_command_api.h"
#include "acamera_sbus_api.h"
#include "acamera_command_api.h"
#include "acamera_sensor_api.h"
#include "system_timer.h"
#include "acamera_firmware_config.h"
#include "acamera_math.h"

typedef struct _sensor_context_t {
    uint8_t address; // Sensor address for direct write (not used currently)
    acamera_sbus_t sbus;
    sensor_param_t param;
//	    sensor_mode_t supported_modes[ISP_MAX_SENSOR_MODES];
} sensor_context_t;

static sensor_context_t s_ctx[FIRMWARE_CONTEXT_NUMBER];
static int ctx_counter = 0;

static sensor_mode_t dummy_drv_supported_modes[] = {
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 3840,
        .resolution.height = 2160,
        .bits = 12,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 3840,
        .resolution.height = 2160,
        .bits = 10,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 3840,
        .resolution.height = 2160,
        .bits = 8,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 10,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 8,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1280,
        .resolution.height = 720,
        .bits = 12,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1280,
        .resolution.height = 720,
        .bits = 10,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1280,
        .resolution.height = 720,
        .bits = 8,
        .exposures = 1,
    },
    {
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 10,
        .exposures = 2,
    },
    {
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 3,
    },
    {//11, need modify json file::"sensor_mode": 11, and acamera_firmware_config.h::SENSOR_DEFAULT_PRESET_MODE 11
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1936,
        .resolution.height = 1096,
        .bits = 12,
        .exposures = 1,
    },
    {//12 DDR DOL2
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1952,
        .resolution.height = 1097,
        .bits = 8,
        .exposures = 2,
    },
    {//13 DDR DOL2
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1952,
        .resolution.height = 1097,
        .bits = 8,
        .exposures = 3,
    },
    {//14 DDR DOL2 (DDR->SIF->ISP)
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1952,
        .resolution.height = 1097,
        .bits = 12,
        .exposures = 2,
    },
    {//15 DDR DOL2
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1952,
        .resolution.height = 1097,
        .bits = 12,
        .exposures = 3,
    },
    {//16 IMX290
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1308,
        .resolution.height = 729,
        .bits = 8,
        .exposures = 1,
    },
    {//17 DDR DOL2 (IMX290->MIPI->SIF->ISP)
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1948,
        .resolution.height = 1060,
        .bits = 12,
        .exposures = 2,
    },
    {//18 DDR DOL3 (IMX290->MIPI->SIF->ISP)
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1948,
        .resolution.height = 1060,
        .bits = 12,
        .exposures = 3,
    },
    {//19 DVP 1920_1080_raw20.bin
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 20,
        .exposures = 1,
    },
    {//20 IMX290->SIF->ISP
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1952,
        .resolution.height = 1097,
        .bits = 10,
        .exposures = 1,
    },
    {//21 IMX290->SIF->ISP
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1952,
        .resolution.height = 1097,
        .bits = 12,
        .exposures = 1,
    },
    {//22 OV8865->SIF->ISP
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2160,
        .bits = 10,
        .exposures = 1,
    },
    {//23 DDR->SIF->ISP
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2160,
        .bits = 12,
        .exposures = 1,
    },
    {//24 DDR->SIF->ISP
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2160,
        .bits = 8,
        .exposures = 1,
    },
    {//25 DVP 1920_1080_raw16.bin
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 16,
        .exposures = 1,
    },
    {//26 DVP 1920_1080_raw12.bin
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 1,
    },
    {//27 DDR->SIF->ISP
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2448,
        .bits = 12,
        .exposures = 1,
    },
    {//28 DDR->SIF->ISP
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2448,
        .bits = 10,
        .exposures = 1,
    },
    {//29 DDR->SIF->ISP
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2448,
        .bits = 8,
        .exposures = 1,
    },
    {//30 IMX290->SIF->ISP
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1952,
        .resolution.height = 1097,
        .bits = 8,
        .exposures = 1,
    },
    {//31 DDR DOL2 (IMX290->MIPI->SIF->ISP)
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1948,
        .resolution.height = 1060,
        .bits = 8,
        .exposures = 2,
    },
    {//32 DDR DOL3 (IMX290->MIPI->SIF->ISP)
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1948,
        .resolution.height = 1060,
        .bits = 8,
        .exposures = 3,
    },
    {//33 DDR DOL2 (IMX290->MIPI->SIF->ISP)
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1948,
        .resolution.height = 1060,
        .bits = 10,
        .exposures = 2,
    },
    {//34 DDR DOL3 (IMX290->MIPI->SIF->ISP)
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1948,
        .resolution.height = 1060,
        .bits = 10,
        .exposures = 3,
    },

};


//*************************************************************************************
//--------------------DATA-------------------------------------------------------------
//--------------------RESET------------------------------------------------------------
static void sensor_hw_reset_enable( void )
{
}

static void sensor_hw_reset_disable( void )
{
}

//--------------------FLASH------------------------------------------------------------

static int32_t sensor_alloc_analog_gain( void *ctx, int32_t gain )
{
    return 0;
}

static int32_t sensor_alloc_digital_gain( void *ctx, int32_t gain )
{
    return 0;
}

static void sensor_alloc_integration_time( void *ctx, uint16_t *int_time, uint16_t *int_time_M, uint16_t *int_time_L )
{
}

static void sensor_update( void *ctx )
{
}

static void sensor_set_mode( void *ctx, uint8_t mode )
{
    sensor_context_t *p_ctx = ctx;
    sensor_param_t *param = &p_ctx->param;
    if(mode>=array_size(dummy_drv_supported_modes))
        mode = 0;
    param->active.width = dummy_drv_supported_modes[mode].resolution.width;
    param->active.height = dummy_drv_supported_modes[mode].resolution.height;
    param->total.width = dummy_drv_supported_modes[mode].resolution.width;
    param->total.height = dummy_drv_supported_modes[mode].resolution.height;
    param->pixels_per_line = param->total.width;
    param->integration_time_min = 1;
    param->integration_time_max = 1000;
    param->integration_time_long_max = 1000; // (((uint32_t)(dummy_drv_supported_modes[mode].resolution.height)) << 2)-256;
    param->integration_time_limit = 1000;
    param->mode = mode;
    param->lines_per_second = 0;
    param->sensor_exp_number = param->modes_table[mode].exposures;
    printk("ISP set mode to %d, %dx%d raw%d, dol%d", mode, dummy_drv_supported_modes[mode].resolution.width,
        dummy_drv_supported_modes[mode].resolution.height, dummy_drv_supported_modes[mode].bits, dummy_drv_supported_modes[mode].exposures);
}

static uint16_t sensor_get_id( void *ctx )
{
    return 0xFFFF;
}

static const sensor_param_t *sensor_get_parameters( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    return (const sensor_param_t *)&p_ctx->param;
}

static void sensor_disable_isp( void *ctx )
{
}

static uint32_t read_register( void *ctx, uint32_t address )
{
    return 0;
}

static void write_register( void *ctx, uint32_t address, uint32_t data )
{
}

static void stop_streaming( void *ctx )
{
}

static void start_streaming( void *ctx )
{
}

void sensor_deinit_dummy( void *ctx )
{
}
//--------------------Initialization------------------------------------------------------------
void sensor_init_dummy( void **ctx, sensor_control_t *ctrl )
{

    if ( ctx_counter < FIRMWARE_CONTEXT_NUMBER ) {
        sensor_context_t *p_ctx = &s_ctx[ctx_counter];
        ctx_counter++;

        p_ctx->param.sensor_exp_number = 1;
        p_ctx->param.again_log2_max = 0;
        p_ctx->param.dgain_log2_max = 0;
        p_ctx->param.integration_time_apply_delay = 2;
        p_ctx->param.isp_exposure_channel_delay = 0;

        p_ctx->param.modes_table = dummy_drv_supported_modes;
        p_ctx->param.modes_num = array_size( dummy_drv_supported_modes );
        p_ctx->param.sensor_ctx = &s_ctx;
        sensor_set_mode(p_ctx, 0);          // init to mode 0

        *ctx = p_ctx;

        ctrl->alloc_analog_gain = sensor_alloc_analog_gain;
        ctrl->alloc_digital_gain = sensor_alloc_digital_gain;
        ctrl->alloc_integration_time = sensor_alloc_integration_time;
        ctrl->sensor_update = sensor_update;
        ctrl->set_mode = sensor_set_mode;
        ctrl->get_id = sensor_get_id;
        ctrl->get_parameters = sensor_get_parameters;
        ctrl->disable_sensor_isp = sensor_disable_isp;
        ctrl->read_sensor_register = read_register;
        ctrl->write_sensor_register = write_register;
        ctrl->start_streaming = start_streaming;
        ctrl->stop_streaming = stop_streaming;

        // Reset sensor during initialization
        sensor_hw_reset_enable();
        system_timer_usleep( 1000 ); // reset at least 1 ms
        sensor_hw_reset_disable();
        system_timer_usleep( 1000 );

    } else {
        LOG( LOG_ERR, "Attempt to initialize more sensor instances than was configured. Sensor initialization failed." );
        *ctx = NULL;
    }
}

//*************************************************************************************
