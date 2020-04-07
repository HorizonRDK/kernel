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

#if !defined( __LDC_FSM_H__ )
#define __LDC_FSM_H__


typedef struct _LDC_fsm_t LDC_fsm_t;
typedef struct _LDC_fsm_t *LDC_fsm_ptr_t;
typedef const struct _LDC_fsm_t *LDC_fsm_const_ptr_t;

#include "acamera_isp_config.h"
#include "acamera_lens_api.h"
#include "acamera_isp_core_nomem_settings.h"

struct _LDC_fsm_t {
	fsm_common_t cmn;

	acamera_fsm_mgr_t *p_fsm_mgr;
	uint32_t line_buf;
	uint32_t x_param_a;
	uint32_t x_param_b;
	uint32_t y_param_a;
	uint32_t y_param_b;
	uint32_t radius_x_offset;
	uint32_t radius_y_offset;
	uint32_t center_x_offset;
	uint32_t center_y_offset;
	uint32_t woi_y_length;
	uint32_t woi_y_start;
	uint32_t woi_x_length;
	uint32_t woi_x_start;
	uint32_t param_update;
	uint32_t ldc_bypass;

	void *lens_ctx;
	lens_control_t lens_ctrl;
};

void LDC_fsm_clear( LDC_fsm_ptr_t p_fsm );
void LDC_fsm_init( void *fsm, fsm_init_param_t *init_param );
int LDC_fsm_set_param( void *fsm, uint32_t param_id, void *input, uint32_t input_size );
int LDC_fsm_get_param( void *fsm, uint32_t param_id, void *input, uint32_t input_size, void *output, uint32_t output_size );

#endif /* __AF_FSM_H__ */
