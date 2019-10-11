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

#include "acamera_lens_api.h"
#include "system_sensor.h"
#include "acamera_logger.h"
#include "acamera_sbus_api.h"
#include <string.h>


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#endif


typedef struct _lens_context_t {
    acamera_sbus_t p_bus;

    uint16_t pos;
    uint16_t prev_pos;
    uint16_t move_pos;

    uint32_t time;

    lens_param_t param;
} lens_context_t;

static lens_context_t lens_ctx[FIRMWARE_CONTEXT_NUMBER];


uint8_t lens_null_test( uint32_t lens_bus )
{
    return 1;//TODO
}

static void vcm_null_drv_move( void *ctx, uint16_t position )
{
    LOG( LOG_WARNING, "Null VCM driver use attempted: no lens movement can be performed with the null driver" );
    //LOG( LOG_CRIT, "IE&E %s, position %d ", __func__, position );
}

static uint8_t vcm_null_drv_is_moving( void *ctx )
{
    LOG( LOG_WARNING, "Null VCM driver use attempted: no lens movement can be performed with the null driver" );
    //LOG( LOG_CRIT, "IE&E %s  ", __func__ );
    return 0;
}

static void vcm_null_write_register( void *ctx, uint32_t address, uint32_t data )
{
}

static uint32_t vcm_null_read_register( void *ctx, uint32_t address )
{
    return 0;
}

static const lens_param_t *lens_get_parameters( void *ctx )
{
    lens_context_t *p_ctx = ctx;
    return (const lens_param_t *)&p_ctx->param;
}

void lens_null_deinit( void *ctx )
{
}

void lens_null_init( void **ctx, lens_control_t *ctrl, uint32_t lens_bus )
{

    //LOG( LOG_CRIT, "IE&E %s -----  ", __func__ );
    lens_context_t *p_ctx = &lens_ctx[0];
    *ctx = p_ctx;

    ctrl->is_moving = vcm_null_drv_is_moving;
    ctrl->move = vcm_null_drv_move;
    ctrl->write_lens_register = vcm_null_write_register;
    ctrl->read_lens_register = vcm_null_read_register;
    ctrl->get_parameters = lens_get_parameters;

    p_ctx->prev_pos = 0;

    memset( &p_ctx->param, 0, sizeof( lens_param_t ) );
    p_ctx->param.min_step = 1 << 6;
    p_ctx->param.lens_type = LENS_VCM_DRIVER_NULL;
}
