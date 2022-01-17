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

#include "acamera_firmware_config.h"
#include "acamera_lens_api.h"
#include "acamera_types.h"

/*    - Test the driver in this file                            */
/*                                                              */
#if ISP_SENSOR_DRIVER_V4L2
#include "v4l2_vcm.h"
#endif

#include "acamera_logger.h"

int32_t lens_init( void **ctx, lens_control_t *ctrl )
{

    uint32_t lens_bus = 0;
#if ISP_SENSOR_DRIVER_V4L2
    if ( lens_v4l2_subdev_test( lens_bus ) ) {
        lens_v4l2_subdev_init( ctx, ctrl, lens_bus );
        LOG( LOG_NOTICE, "Lens VCM driver is V4L2 subdev" );
        return 0;
    }
#endif
    LOG( LOG_WARNING, "NO VALID SENSOR DRIVER FOUND bus:0x%x", lens_bus );
    return -1;
}

void lens_deinit( void *ctx )
{
#if ISP_SENSOR_DRIVER_V4L2
    lens_v4l2_subdev_deinit( ctx );
#endif
}