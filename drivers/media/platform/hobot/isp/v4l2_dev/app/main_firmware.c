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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <linux/init.h>

#include <linux/err.h>
#include <linux/errno.h>
#include "acamera.h"
#include "acamera_firmware_api.h"
#include "acamera_firmware_config.h"
#include "acamera_command_api.h"
#include "acamera_control_config.h"
#include "system_control.h"
#if ISP_HAS_CONNECTION_DEBUG
#include "acamera_cmd_interface.h"
#endif

#if ISP_HAS_STREAM_CONNECTION
#include "acamera_connection.h"
#endif

#include "application_command_api.h"

/* Entire main_firmware is not used when V4L2 interface is enabled.
 * All relevent functions and codes here can be found in fw-interface.c
 */
// the settings for each firmware context were pre-generated and
// saved in the header file. They are given as a reference and should be changed
// according to the customer needs.
#include "runtime_initialization_settings.h"

#if !V4L2_INTERFACE_BUILD //all these callbacks are managed by v4l2


// The driver can provide the full set of metadata parameters
// The callback_meta function should be set in initalization settings to support it.
void callback_meta( uint32_t ctx_num, const void *fw_metadata )
{
}


// The ISP pipeline can have several outputs such as Full Resolution, DownScaler1, DownScaler2 etc
// It is possible to set the firmware up for returning metadata on each output frame from
// the specific channel. This callbacks must be set in acamera_settings structure and passed to the firmware in
// acamera_init api function
// The pointer to the context can be used to differentiate contexts

#endif

#if ISP_HAS_STREAM_CONNECTION && CONNECTION_IN_THREAD

static struct task_struct *isp_fw_connections_thread = NULL;

extern int acamera_connection_valid(void);
static int connection_thread( void *foo )
{
    LOG( LOG_INFO, "connection_thread start" );

    acamera_connection_init();

    while ( acamera_connection_valid() && !kthread_should_stop() ) {
        acamera_connection_process();
    }

    acamera_connection_destroy();

    LOG( LOG_INFO, "connection_thread stop" );

    return 0;
}
#endif

// this is a main application IRQ handler to drive firmware
// The main purpose is to redirect irq events to the
// appropriate firmware context.
// There are several type of firmware IRQ which may happen.
// The other events are platform specific. The firmware can support several external irq events or
// does not support any. It totally depends on the system configuration and firmware compile time settings.
// Please see the ACamera Porting Guide for details.
static void interrupt_handler( void *data, uint32_t mask )
{
    acamera_interrupt_handler();
}


int isp_fw_init( uint32_t hw_isp_addr )
{
    int result = 0;
    uint32_t i;

    LOG( LOG_INFO, "fw_init start" );

    for ( i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
        settings[i].hw_isp_addr = hw_isp_addr;
    }

    // The firmware supports multicontext.
    // It means that the customer can use the same firmware for controlling
    // several instances of different sensors/isp. To initialise a context
    // the structure acamera_settings must be filled properly.
    // the total number of initialized context must not exceed FIRMWARE_CONTEXT_NUMBER
    // all contexts are numerated from 0 till ctx_number - 1
    result = acamera_init( settings, FIRMWARE_CONTEXT_NUMBER );

    bsp_init();

    if ( result == 0 ) {
        // uint32_t rc = 0;
        // uint32_t ctx_num;
        // uint32_t prev_ctx_num = 0;

        // application_command( TGENERAL, ACTIVE_CONTEXT, 0, COMMAND_GET, &prev_ctx_num );

        // set the interrupt handler. The last parameter may be used
        // to specify the context. The system must call this interrupt_handler
        // function whenever the ISP interrupt happens.
        // This interrupt handling procedure is only advisable and is used in ACamera demo application.
        // It can be changed by a customer discretion.
        system_interrupt_set_handler( interrupt_handler, NULL );

        // start streaming for sensors
        /*
        for ( ctx_num = 0; ctx_num < FIRMWARE_CONTEXT_NUMBER; ctx_num++ ) {
            application_command( TGENERAL, ACTIVE_CONTEXT, ctx_num, COMMAND_SET, &rc );
            application_command( TSENSOR, SENSOR_STREAMING, ON, COMMAND_SET, &rc );
        }

        application_command( TGENERAL, ACTIVE_CONTEXT, prev_ctx_num, COMMAND_SET, &rc );
        */
    } else {
        LOG( LOG_INFO, "Failed to start firmware processing thread. " );
    }

    LOG( LOG_INFO, "isp_fw_init result %d", result );
    if ( result == 0 ) {

#if ISP_HAS_STREAM_CONNECTION && CONNECTION_IN_THREAD
        LOG( LOG_INFO, "start connection thread %d", result );
        isp_fw_connections_thread = kthread_run( connection_thread, NULL, "isp_connection" );
#endif

        LOG( LOG_INFO, "start fw thread %d", result );
    }

    return 0;
}

void isp_fw_exit( void )
{
#if ISP_HAS_STREAM_CONNECTION && CONNECTION_IN_THREAD
    if ( isp_fw_connections_thread )
        kthread_stop( isp_fw_connections_thread );
#endif

    // this api function will free
    // all resources allocated by the firmware
    acamera_terminate();

    bsp_destroy();
}
