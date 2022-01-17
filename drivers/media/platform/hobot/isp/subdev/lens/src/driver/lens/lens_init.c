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
#include "acamera_lens_api.h"

/*    - Test the driver in this file                            */
/*                                                              */

#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#endif

#define ISP_SENSOR_DRIVER_COMMON 1

#if ISP_SENSOR_DRIVER_COMMON
#include "null_vcm.h"
#endif

#include "acamera_logger.h"

int32_t sub_lens_init(void **ctx, lens_control_t *ctrl, uint32_t port)
{

    uint32_t lens_bus = 0;
#if ISP_SENSOR_DRIVER_COMMON
    // Null should always be tested last
    if ( lens_null_test( lens_bus ) ) {
        if (unlikely(port >= FIRMWARE_CONTEXT_NUMBER)) {
            LOG(LOG_ERR, "Failed to %s, port:%d must less than %d\n",
                __func__, port, FIRMWARE_CONTEXT_NUMBER);
            return -1;
        } else {
            lens_null_init(ctx, ctrl, port);
        }
        LOG( LOG_NOTICE, "Lens VCM driver is COMMON" );
        return 0;
    }
#endif

    LOG( LOG_WARNING, "NO VALID SENSOR DRIVER FOUND bus:0x%x", lens_bus );
    return -1;
}

void sub_lens_deinit(void *ctx)
{

}
