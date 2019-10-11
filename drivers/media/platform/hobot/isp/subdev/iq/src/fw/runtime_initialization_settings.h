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

#include "acamera_types.h"
#include "acamera_firmware_config.h"



extern void sensor_init_dummy(uint32_t ctx_id, void** ctx, sensor_control_t*) ;
extern void sensor_deinit_dummy(uint32_t ctx_id, void *ctx ) ;
extern uint32_t get_calibrations_dummy( uint32_t ctx_id, void * sensor_arg,ACameraCalibrations *, uint32_t sensor_type) ;


#if FIRMWARE_CONTEXT_NUMBER == 1
        #define SENSOR_INIT_SUBDEV_FUNCTIONS {sensor_init_dummy,}
        #define SENSOR_DEINIT_SUBDEV_FUNCTIONS {sensor_deinit_dummy,}
        #define CALIBRATION_SUBDEV_FUNCTIONS {get_calibrations_dummy,}
#elif FIRMWARE_CONTEXT_NUMBER == 2
        #define SENSOR_INIT_SUBDEV_FUNCTIONS {sensor_init_dummy, sensor_init_dummy,}
        #define SENSOR_DEINIT_SUBDEV_FUNCTIONS {sensor_deinit_dummy, sensor_deinit_dummy,}
        #define CALIBRATION_SUBDEV_FUNCTIONS {get_calibrations_dummy, get_calibrations_dummy,}
#elif FIRMWARE_CONTEXT_NUMBER == 3
        #define SENSOR_INIT_SUBDEV_FUNCTIONS {sensor_init_dummy, sensor_init_dummy,\
                                                 sensor_init_dummy, }
        #define SENSOR_DEINIT_SUBDEV_FUNCTIONS {sensor_deinit_dummy, sensor_deinit_dummy, \
                                                 sensor_deinit_dummy, }
        #define CALIBRATION_SUBDEV_FUNCTIONS {get_calibrations_dummy, get_calibrations_dummy, \
                                                get_calibrations_dummy,}
#elif FIRMWARE_CONTEXT_NUMBER == 4
        #define SENSOR_INIT_SUBDEV_FUNCTIONS {sensor_init_dummy, sensor_init_dummy,\
                                                 sensor_init_dummy, sensor_init_dummy,}
        #define SENSOR_DEINIT_SUBDEV_FUNCTIONS {sensor_deinit_dummy, sensor_deinit_dummy, \
                                                 sensor_deinit_dummy, sensor_deinit_dummy,}
        #define CALIBRATION_SUBDEV_FUNCTIONS {get_calibrations_dummy, get_calibrations_dummy, \
                                                get_calibrations_dummy, get_calibrations_dummy,}
#elif FIRMWARE_CONTEXT_NUMBER == 5
        #define SENSOR_INIT_SUBDEV_FUNCTIONS {sensor_init_dummy, sensor_init_dummy,\
                                                 sensor_init_dummy, sensor_init_dummy,\
                                                 sensor_init_dummy, }
        #define SENSOR_DEINIT_SUBDEV_FUNCTIONS {sensor_deinit_dummy, sensor_deinit_dummy, \
                                                 sensor_deinit_dummy, sensor_deinit_dummy,\
                                                 sensor_deinit_dummy, }
        #define CALIBRATION_SUBDEV_FUNCTIONS {get_calibrations_dummy, get_calibrations_dummy, \
                                                get_calibrations_dummy, get_calibrations_dummy,\
                                                get_calibrations_dummy, }
#elif FIRMWARE_CONTEXT_NUMBER == 6
        #define SENSOR_INIT_SUBDEV_FUNCTIONS {sensor_init_dummy, sensor_init_dummy,\
                                                 sensor_init_dummy, sensor_init_dummy,\
                                                 sensor_init_dummy, sensor_init_dummy,}
        #define SENSOR_DEINIT_SUBDEV_FUNCTIONS {sensor_deinit_dummy, sensor_deinit_dummy, \
                                                 sensor_deinit_dummy, sensor_deinit_dummy,\
                                                 sensor_deinit_dummy, sensor_deinit_dummy,}
        #define CALIBRATION_SUBDEV_FUNCTIONS {get_calibrations_dummy, get_calibrations_dummy, \
                                                get_calibrations_dummy, get_calibrations_dummy,\
                                                get_calibrations_dummy, get_calibrations_dummy,}
#endif
