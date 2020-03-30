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

#include "acamera_command_api.h"
#include "acamera_sensor_api.h"
#include "acamera_logger.h"
#include "acamera_firmware_settings.h"
#include "acamera_calibration.h"
#include "linux/kernel.h"

#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_SOC_IQ
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_IQ
#endif


extern uint32_t get_calibrations_static_linear_dummy( ACameraCalibrations *c );
extern uint32_t get_calibrations_static_fs_lin_dummy( ACameraCalibrations *c );
extern uint32_t get_calibrations_dynamic_linear_dummy( ACameraCalibrations *c );
extern uint32_t get_calibrations_dynamic_fs_lin_dummy( ACameraCalibrations *c );

extern int register_calib( ACameraCalibrations *c, uint8_t port );
extern int unregister_calib( ACameraCalibrations *c, uint8_t port );

uint32_t get_calibrations_dummy( uint32_t ctx_id, void *sensor_arg, ACameraCalibrations *c, uint32_t sensor_type )
{

    uint8_t ret = 0;
    int iret = 0;

    if ( !sensor_arg ) {
        LOG( LOG_ERR, "calibration sensor_arg is NULL" );
        return ret;
    }

    int32_t preset = ( (sensor_mode_t *)sensor_arg )->wdr_mode;

    //logic which calibration to apply
    switch ( preset ) {
    case WDR_MODE_LINEAR:
        LOG( LOG_DEBUG, "calibration switching to WDR_MODE_LINEAR %d ", (int)preset );
	
	iret = unregister_calib( c, (uint8_t)(ctx_id & 0xff) );	
	iret = register_calib( c, (uint8_t)(ctx_id & 0xff) );
	if (iret < 0) {
		LOG(LOG_DEBUG, "get default calib of port %d", ctx_id);
        	ret += ( get_calibrations_dynamic_linear_dummy( c ) + get_calibrations_static_linear_dummy( c ) );
	}
	break;
    case WDR_MODE_NATIVE:
        LOG( LOG_DEBUG, "calibration switching to WDR_MODE_NATIVE %d ", (int)preset );
	iret = unregister_calib( c, (uint8_t)(ctx_id & 0xff) );	
	iret = register_calib( c, (uint8_t)(ctx_id & 0xff) );
	if (iret < 0) {
		LOG(LOG_DEBUG, "get default calib of port %d", ctx_id);
        	ret += ( get_calibrations_dynamic_linear_dummy( c ) + get_calibrations_static_linear_dummy( c ) );
	}
        break;
    case WDR_MODE_FS_LIN:
        LOG( LOG_DEBUG, "calibration switching to WDR mode on mode %d ", (int)preset );
	iret = unregister_calib( c, (uint8_t)(ctx_id & 0xff) );	
	iret = register_calib( c, (uint8_t)(ctx_id & 0xff) );
	if (iret < 0) {
		LOG(LOG_DEBUG, "get default calib of port %d", ctx_id);
        	ret += ( get_calibrations_dynamic_fs_lin_dummy( c ) + get_calibrations_static_fs_lin_dummy( c ) );
	}
        break;
    default:
        LOG( LOG_DEBUG, "calibration defaults to WDR_MODE_LINEAR %d ", (int)preset );
	iret = unregister_calib( c, (uint8_t)(ctx_id & 0xff) );	
	//iret = register_calib( c, (uint8_t)(ctx_id & 0xff) );
        ret += ( get_calibrations_dynamic_linear_dummy( c ) + get_calibrations_static_linear_dummy( c ) );
        break;
    }

    return ret;
}
