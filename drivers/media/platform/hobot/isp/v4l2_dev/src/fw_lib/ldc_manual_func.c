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

#include "acamera_fw.h"
#include "ldc_manual_func.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

/* Use static memory here to make it cross-platform */
static LDC_fsm_t ldc_fsm_ctxs[FIRMWARE_CONTEXT_NUMBER];

 /*
  *  ldc param:
  *  uint32_t ldc_enable;
  *  ldc_path_sel_u path_sel;
  *  uint32_t y_start_addr;
  *  uint32_t c_start_addr;
  *  ldc_picsize_u picsize;
  *  uint32_t line_buf;
  *  ldc_algoparam_u x_param;
  *  ldc_algoparam_u y_param;
  *  ldc_off_shift_u off_shift;
  *  ldc_woi_u woi_x;
  *  ldc_woi_u woi_y;
  */

extern int set_ldc_param(uint32_t port, uint32_t *ptr, uint32_t size);
extern int get_ldc_param(uint32_t port, uint32_t *ptr, uint32_t size);

int update_ldc_param(void *fsm)
{
	int ret = 0;
	LDC_fsm_t *p_fsm = (LDC_fsm_t *)fsm;
	uint32_t ctx_id = (uint32_t)p_fsm->cmn.ctx_id;

	uint32_t data[11];

	ret = get_ldc_param(ctx_id, data, sizeof(data));
	if (!ret) {
		data[0] = p_fsm->ldc_bypass;
		data[5] = p_fsm->line_buf;
		data[6] = p_fsm->x_param_a << 16 | p_fsm->x_param_b;
		data[7] = p_fsm->y_param_a << 16 | p_fsm->y_param_b;
		data[8] = (p_fsm->radius_x_offset << 24 | p_fsm->radius_y_offset << 16
			 | p_fsm->center_x_offset << 8 | p_fsm->center_y_offset);
		//data[9] = p_fsm->woi_x_length << 16 | p_fsm->woi_x_start;
		//data[10] = p_fsm->woi_y_length << 16 | p_fsm->woi_y_start;
		ret = set_ldc_param(ctx_id, data, sizeof(data));
	}
	return ret;
}

int ldc_fsm_set_param(void *fsm, uint32_t param_id, void *input, uint32_t input_size)
{
	int rc = 0;
	LDC_fsm_t *p_fsm = (LDC_fsm_t *)fsm;

	if (p_fsm == NULL)
		return -1;

	switch (param_id) {
	case FSM_PARAM_SET_LDC_LINE_BUF:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->line_buf = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_X_PARAM_A:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->x_param_a = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_X_PARAM_B:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->x_param_b = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_Y_PARAM_A:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->y_param_a = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_Y_PARAM_B:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->y_param_b = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_RADIUS_XOFFSET:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->radius_x_offset = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_RADIUS_YOFFSET:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->radius_y_offset = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_CENTER_XOFFSET:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->center_x_offset = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_CENTER_YOFFSET:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->center_y_offset = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_WOI_YLENGTH:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->woi_y_length = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_WOI_YSTART:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->woi_y_start = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_WOI_XLENGTH:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->woi_x_length = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_WOI_XSTART:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->woi_x_start  = *(uint32_t *)input;

		break;

	case FSM_PARAM_SET_LDC_PARAM_UPDATE:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		rc = update_ldc_param(p_fsm);

		break;

	case FSM_PARAM_SET_LDC_BYPASS:
		if (!input || input_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		p_fsm->ldc_bypass = *(uint32_t *)input;

		break;

	default:
		rc = -1;
		break;
	}

	return rc;
}

int ldc_fsm_get_param(void *fsm, uint32_t param_id, void *input, uint32_t input_size, void *output, uint32_t output_size)
{
	int rc = 0;
	LDC_fsm_t *p_fsm = (LDC_fsm_t *)fsm;

	if (p_fsm == NULL)
		return -1;

	switch (param_id) {
	case FSM_PARAM_GET_LDC_LINE_BUF:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->line_buf;

		break;

	case FSM_PARAM_GET_LDC_X_PARAM_A:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->x_param_a;

		break;

	case FSM_PARAM_GET_LDC_X_PARAM_B:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->x_param_b;

		break;

	case FSM_PARAM_GET_LDC_Y_PARAM_A:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->y_param_a;

		break;


	case FSM_PARAM_GET_LDC_Y_PARAM_B:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->y_param_b;

		break;

	case FSM_PARAM_GET_LDC_RADIUS_XOFFSET:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->radius_x_offset;

		break;

	case FSM_PARAM_GET_LDC_RADIUS_YOFFSET:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->radius_y_offset;

		break;

	case FSM_PARAM_GET_LDC_CENTER_XOFFSET:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->center_x_offset;

		break;

	case FSM_PARAM_GET_LDC_WOI_YLENGTH:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->woi_y_length;

		break;

	case FSM_PARAM_GET_LDC_WOI_YSTART:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->woi_y_start;

		break;

	case FSM_PARAM_GET_LDC_WOI_XLENGTH:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->woi_x_length;

		break;

	case FSM_PARAM_GET_LDC_WOI_XSTART:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->woi_x_start;

		break;

	case FSM_PARAM_GET_LDC_PARAM_UPDATE:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->param_update;

		break;

	case FSM_PARAM_GET_LDC_BYPASS:
		if (!output || output_size != sizeof(uint32_t)) {
			LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
			rc = -1;
			break;
		}
		*(uint32_t *)output = p_fsm->ldc_bypass;

		break;

	default:
		rc = -1;
		break;
	}

	return rc;
}

void ldc_fsm_init(void *fsm, fsm_init_param_t *init_param)
{
	LDC_fsm_t *p_fsm = (LDC_fsm_t *)fsm;

	p_fsm->cmn.p_fsm_mgr = init_param->p_fsm_mgr;
	p_fsm->cmn.isp_base = init_param->isp_base;
	p_fsm->p_fsm_mgr = init_param->p_fsm_mgr;

	p_fsm->line_buf = 99;
	p_fsm->x_param_a = 0;
	p_fsm->x_param_b = 0;
	p_fsm->y_param_a = 0;
	p_fsm->y_param_b = 0;
	p_fsm->radius_x_offset = 0;
	p_fsm->radius_y_offset = 0;
	p_fsm->center_x_offset = 0;
	p_fsm->center_y_offset = 0;
	p_fsm->woi_y_length = 1079;
	p_fsm->woi_y_start = 0;
	p_fsm->woi_x_length = 1919;
	p_fsm->woi_x_start = 0;
	p_fsm->ldc_bypass = 0;
	p_fsm->param_update = 0;

	return;
}

void ldc_fsm_deinit(void *fsm)
{
	return;
}

fsm_common_t *LDC_get_fsm_common(uint8_t ctx_id)
{
	LDC_fsm_t *p_fsm_ctx = NULL;

	if (ctx_id >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_CRIT, "Invalid ctx_id: %d, greater than max: %d.", ctx_id, FIRMWARE_CONTEXT_NUMBER - 1);
		return NULL;
	}

	p_fsm_ctx = &ldc_fsm_ctxs[ctx_id];

	p_fsm_ctx->cmn.ctx_id = ctx_id;
	p_fsm_ctx->cmn.p_fsm = (void *)p_fsm_ctx;

	p_fsm_ctx->cmn.ops.init = ldc_fsm_init;
	p_fsm_ctx->cmn.ops.deinit = ldc_fsm_deinit;
	p_fsm_ctx->cmn.ops.run = NULL;
	p_fsm_ctx->cmn.ops.get_param = ldc_fsm_get_param;
	p_fsm_ctx->cmn.ops.set_param = ldc_fsm_set_param;
	p_fsm_ctx->cmn.ops.proc_event = (FUN_PTR_PROC_EVENT)NULL;
	p_fsm_ctx->cmn.ops.proc_interrupt = (FUN_PTR_PROC_INT)NULL;

	return &(p_fsm_ctx->cmn);
}

