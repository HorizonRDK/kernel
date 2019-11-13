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
#include "sensor_i2c.h"


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#endif


typedef struct _sensor_context_t {
    uint8_t address; // Sensor address for direct write (not used currently)
    uint8_t channel;
    acamera_sbus_t sbus;
    sensor_param_t param;
    //sensor_mode_t supported_modes[ISP_MAX_SENSOR_MODES];//TODO
} sensor_context_t;

static sensor_context_t s_ctx[FIRMWARE_CONTEXT_NUMBER];
static uint32_t ctx_counter = 0;

static sensor_mode_t dummy_drv_supported_modes[] = {
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 4096,
        .resolution.height = 3000,
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
    {//3
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 1,
    },
    {//4
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 10,
        .exposures = 1,
    },
    {//5
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 8,
        .exposures = 1,
    },
    {//6
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1280,
        .resolution.height = 720,
        .bits = 12,
        .exposures = 1,
    },
    {//7
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1280,
        .resolution.height = 720,
        .bits = 10,
        .exposures = 1,
    },
    {//8
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1280,
        .resolution.height = 720,
        .bits = 8,
        .exposures = 1,
    },
    {//9
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 10,
        .exposures = 2,
    },
    {//10
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
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 2,
    },
    {//18 DDR DOL3 (IMX290->MIPI->SIF->ISP)
        .wdr_mode = WDR_MODE_FS_LIN,
        .fps = 10 * 256,
        .resolution.width = 1948,
        .resolution.height = 1097,
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
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1952,
        .resolution.height = 1097,
        .bits = 10,
        .exposures = 1,
    },
    {//21 IMX290->SIF->ISP
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 25 * 256,//10 *256
        .resolution.width = 1952,
        .resolution.height = 1097,
        .bits = 12,
        .exposures = 1,
    },
    {//22 OV8865->SIF->ISP
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2160,
        .bits = 10,
        .exposures = 1,
    },
    {//23 DDR->SIF->ISP
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2160,
        .bits = 12,
        .exposures = 1,
    },
    {//24 DDR->SIF->ISP
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2160,
        .bits = 8,
        .exposures = 1,
    },
    {//25 DVP 1920_1080_raw16.bin
        //.wdr_mode = WDR_MODE_LINEAR,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 16,
        .exposures = 1,
    },
    {//26 DVP 1920_1080_raw12.bin
        //.wdr_mode = WDR_MODE_LINEAR,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 1,
    },
    {//27 DDR->SIF->ISP
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2448,
        .bits = 12,
        .exposures = 1,
    },
    {//28 DDR->SIF->ISP
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2448,
        .bits = 10,
        .exposures = 1,
    },
    {//29 DDR->SIF->ISP
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_LINEAR,
        .fps = 10 * 256,
        .resolution.width = 3264,
        .resolution.height = 2448,
        .bits = 8,
        .exposures = 1,
    },
    {//30 
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_NATIVE,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 1,
    },
    {//31 
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_NATIVE,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 2,
    },
    {//32 
        //.wdr_mode = WDR_MODE_FS_LIN,
	.wdr_mode = WDR_MODE_NATIVE,
        .fps = 10 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 3,
    },

};

//*************************************************************************************
//--------------------DATA-------------------------------------------------------------
struct sensor_operations *sensor_ops[FIRMWARE_CONTEXT_NUMBER];
struct sensor_priv sensor_data[FIRMWARE_CONTEXT_NUMBER];

//--------------------RESET------------------------------------------------------------
static void sensor_hw_reset_enable( void )
{
//-- TODO
        LOG( LOG_DEBUG, "IE&E %s", __func__);
//	if (sensor_ops[p_ctx->channel])
//		sensor_ops[p_ctx->channel]->sensor_hw_reset_enable();
}

static void sensor_hw_reset_disable( void )
{
//-- TODO
        LOG( LOG_DEBUG, "IE&E %s", __func__);
//	if (sensor_ops[p_ctx->channel])
//		sensor_ops[p_ctx->channel]->sensor_hw_reset_disable();
}

//--------------------FLASH------------------------------------------------------------

static int32_t sensor_alloc_analog_gain( void *ctx, int32_t gain )
{
//-- TODO
	sensor_context_t *p_ctx = ctx;
	int32_t analog_gain = 0;
        LOG( LOG_INFO, "IE&E %s, gain %d", __func__, analog_gain);

	gain = gain >> (LOG2_GAIN_SHIFT - 5);		
	if (sensor_ops[p_ctx->channel])
		analog_gain = sensor_ops[p_ctx->channel]->sensor_alloc_analog_gain(p_ctx->channel, gain);
	else
		LOG( LOG_ERR, " sensor_ops is null !");

	sensor_data[p_ctx->channel].analog_gain = analog_gain;

	gain = gain << (LOG2_GAIN_SHIFT - 5);		
	return gain;
}

static int32_t sensor_alloc_digital_gain( void *ctx, int32_t gain )
{
//-- TODO
	int32_t digital_gain = 0;
	sensor_context_t *p_ctx = ctx;

	LOG( LOG_INFO, "IE&E %s, gain %d ", __func__, gain);

	gain = gain >> (LOG2_GAIN_SHIFT - 5);
	if (sensor_ops[p_ctx->channel])
		digital_gain = sensor_ops[p_ctx->channel]->sensor_alloc_digital_gain(p_ctx->channel, gain);
	else
		LOG( LOG_ERR, " sensor_ops is null !");

	sensor_data[p_ctx->channel].digital_gain = digital_gain;

	gain = gain << (LOG2_GAIN_SHIFT - 5);		
	return gain;
}

static void sensor_alloc_integration_time( void *ctx, uint16_t *int_time, uint16_t *int_time_M, uint16_t *int_time_L )
{
//-- TODO
	sensor_context_t *p_ctx = ctx;

	LOG(LOG_INFO, "init s_t %d, M_t %d, L_t %d", *int_time, *int_time_M, *int_time_L);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->sensor_alloc_integration_time(p_ctx->channel, int_time, int_time_M, int_time_L);
	else
		LOG( LOG_ERR, " sensor_ops is null !");
	sensor_data[p_ctx->channel].int_time = *int_time;
	sensor_data[p_ctx->channel].int_time_M = *int_time_M;
	sensor_data[p_ctx->channel].int_time_L = *int_time_L;

	LOG(LOG_INFO, "end s_t %d, M_t %d, L_t %d", *int_time, *int_time_M, *int_time_L);
}

static void sensor_update( void *ctx )
{
//-- TODO 
	sensor_context_t *p_ctx = ctx;
        
//	 int32_t analog_gain;
//       int32_t digital_gain;
//        uint16_t int_time;
//        uint16_t int_time_M;
//        uint16_t int_time_L; 
#if 1
	LOG(LOG_INFO, "analog_gain %d, digital_gain %d, int_time %d ", sensor_data[p_ctx->channel].analog_gain, sensor_data[p_ctx->channel].digital_gain, sensor_data[p_ctx->channel].int_time);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->sensor_update(p_ctx->channel, sensor_data[p_ctx->channel]);
	else
		LOG( LOG_ERR, " sensor_ops is null !");
#endif
}

static void sensor_set_mode( void *ctx, uint8_t mode )
{
    sensor_context_t *p_ctx = ctx;
    sensor_param_t *param = &p_ctx->param;
    if(mode>=array_size(dummy_drv_supported_modes))
        mode = 0;
//TODO	

    p_ctx->param.sensor_exp_number = 1;
    p_ctx->param.again_log2_max = 2088960;//0 255*2^13 = 255*8196
    p_ctx->param.dgain_log2_max = 2088960;//0
    p_ctx->param.integration_time_apply_delay = 2;
    p_ctx->param.isp_exposure_channel_delay = 0;
   
    param->active.width = dummy_drv_supported_modes[mode].resolution.width;
    param->active.height = dummy_drv_supported_modes[mode].resolution.height;
    param->total.width = dummy_drv_supported_modes[mode].resolution.width;
    param->total.height = dummy_drv_supported_modes[mode].resolution.height;
    param->pixels_per_line = param->total.width;
    param->integration_time_min = 1;
    //param->integration_time_max = 11;//3685;//TODO
    param->integration_time_long_max = 3685 * 3; // (((uint32_t)(dummy_drv_supported_modes[mode].resolution.height)) << 2)-256;
    //param->integration_time_limit = 11;//TODO
    param->mode = mode;
    param->lines_per_second = 5993;//9212;//TODO
    param->sensor_exp_number = param->modes_table[mode].exposures;
    //sensor - init
#if 0
    if (sensor_ops[p_ctx->channel]) {
	if (param->modes_table[mode].wdr_mode == WDR_MODE_LINEAR) {
		//normal
    		sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, 0);
	} else if (param->modes_table[mode].wdr_mode == WDR_MODE_FS_LIN) {
		if (param->sensor_exp_number == 2)//dol2
    			sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, 1);
		else if (param->sensor_exp_number == 3)//dol3
    			sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, 2);
	} else if (param->modes_table[mode].wdr_mode == WDR_MODE_NATIVE) {
		//pwl
    		sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, 4);
	}
    }
#endif
    printk("sensor set mode to %d, %dx%d raw%d, dol%d", mode, dummy_drv_supported_modes[mode].resolution.width,
        dummy_drv_supported_modes[mode].resolution.height, dummy_drv_supported_modes[mode].bits, dummy_drv_supported_modes[mode].exposures);
}

static void sensor_set_type( void *ctx, uint8_t sensor_type, uint8_t sensor_i2c_channel )
{
    sensor_context_t *p_ctx = ctx;
    sensor_param_t *param = &p_ctx->param;

    LOG( LOG_INFO, "[%s--%d]", __func__, __LINE__ );
    if (param->sensor_type != sensor_type || param->sensor_i2c_channel != sensor_i2c_channel) {
    	param->sensor_type = sensor_type;
    	param->sensor_i2c_channel = sensor_i2c_channel;
    	sensor_ops[p_ctx->channel] = sensor_chn_open(p_ctx->channel, 1, sensor_type);
    	printk("sensor set type is %d, i2c chn is %d, dol%d ", sensor_type, sensor_i2c_channel, param->sensor_exp_number );
	if (sensor_ops[p_ctx->channel]) {
		if (param->modes_table[param->mode].wdr_mode == WDR_MODE_LINEAR) {
			//normal
    			param->lines_per_second = 10074;//
    			sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, 0);
		} else if (param->modes_table[param->mode].wdr_mode == WDR_MODE_FS_LIN) {
			if (param->sensor_exp_number == 2) {
				param->integration_time_max = 11;//TODO
				param->integration_time_limit = 11;//TODO
    				param->lines_per_second = 7183;//9212;
    				sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, 1);
			} else if (param->sensor_exp_number == 3) {
    				param->lines_per_second = 5993;//9212;
    				sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, 2);
			}
		} else if (param->modes_table[param->mode].wdr_mode == WDR_MODE_NATIVE) {
			//pwl
    			sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, 4);
			}
    		}
    }
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
//-- TODO
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->sensor_disable_isp(p_ctx->channel);
}

static uint32_t read_register( void *ctx, uint32_t address )
{
//-- TODO
	uint32_t data = 0;
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		data = sensor_ops[p_ctx->channel]->read_register(p_ctx->channel, address);
	return data;
}

static void write_register( void *ctx, uint32_t address, uint32_t data )
{
//-- TODO
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->write_register(p_ctx->channel, address, data);
}

static void stop_streaming( void *ctx )
{
//-- TODO	
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->stop_streaming(p_ctx->channel);
}

static void start_streaming( void *ctx )
{
//-- TODO
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->start_streaming(p_ctx->channel);
}

void sensor_deinit_dummy(uint32_t ctx_id, void *ctx )
{
}
//--------------------Initialization------------------------------------------------------------
void sensor_init_dummy(uint32_t ctx_id, void **ctx, sensor_control_t *ctrl )
{

	ctx_counter = ctx_id;
    if ( ctx_counter < FIRMWARE_CONTEXT_NUMBER ) {
        sensor_context_t *p_ctx = &s_ctx[ctx_counter];
        //ctx_counter++;
	sensor_ops[p_ctx->channel] = NULL;

	
	p_ctx->channel = ctx_id;
        p_ctx->param.sensor_exp_number = 1;
        p_ctx->param.again_log2_max = 2088960;//0 255*2^13 = 255*8196
        p_ctx->param.dgain_log2_max = 2088960;//0
        p_ctx->param.integration_time_apply_delay = 2;
        p_ctx->param.isp_exposure_channel_delay = 0;

        p_ctx->param.modes_table = dummy_drv_supported_modes;
        p_ctx->param.modes_num = array_size( dummy_drv_supported_modes );
        p_ctx->param.sensor_ctx = &s_ctx;
        sensor_set_mode(p_ctx, 0);          // init to mode 0

//TODO 
	p_ctx->param.sensor_type = SENSOR_IMX290;

        *ctx = p_ctx;

        ctrl->alloc_analog_gain = sensor_alloc_analog_gain;
        ctrl->alloc_digital_gain = sensor_alloc_digital_gain;
        ctrl->alloc_integration_time = sensor_alloc_integration_time;
        ctrl->sensor_update = sensor_update;
        ctrl->set_mode = sensor_set_mode;
	ctrl->set_sensor_type = sensor_set_type; 
        ctrl->get_id = sensor_get_id;
        ctrl->get_parameters = sensor_get_parameters;
        ctrl->disable_sensor_isp = sensor_disable_isp;
        ctrl->read_sensor_register = read_register;
        ctrl->write_sensor_register = write_register;
        ctrl->start_streaming = start_streaming;
        ctrl->stop_streaming = stop_streaming;
//TODO

	//update sensor 
#if 1
	LOG( LOG_INFO, "[%s--%d]", __func__, __LINE__ );
	sensor_ops[p_ctx->channel] = sensor_chn_open(ctx_counter,SENSOR_I2C, p_ctx->param.sensor_type);
//	if (sensor_ops[p_ctx->channel])
//		sensor_ops[p_ctx->channel]->sensor_init(ctx_counter, 0);
#endif
        // Reset sensor during initialization
        sensor_hw_reset_enable();//TODO
        system_timer_usleep( 1000 ); // reset at least 1 ms
        sensor_hw_reset_disable();
        system_timer_usleep( 1000 );

    } else {
        LOG( LOG_ERR, "Attempt to initialize more sensor instances than was configured. Sensor initialization failed." );
        *ctx = NULL;
    }
}

//*************************************************************************************
