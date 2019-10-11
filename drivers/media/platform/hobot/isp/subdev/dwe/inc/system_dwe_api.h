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

#ifndef __ACAMERA_DWE_API_H__
#define __ACAMERA_DWE_API_H__

#include "dis_base_io.h"
#include "ldc_base_io.h"
#include "dwe_dev.h"
#include "acamera_dwe_config.h"

typedef struct _dwe_param_s {
	dis_param_s dis_param;
	uint32_t ldc_enable;
	ldc_param_s ldc_param;
	pg_param_s pg_param;
} dwe_param_t;


typedef struct _dwe_context_t {
    void *ptr_mem;
    uint32_t ldc_irqstatus;
    uint32_t dis_irqsattus;
    dis_setting_u dis_setting;
    ldc_setting_u ldc_setting;
    int ctx_sitchwin; // control set param
    int prev_port;
    int curr_port;
    int next_port;
    int ldc_update;
    int dis_update;
    uint8_t ldc_cur;
    uint8_t dis_cur;
} dwe_context_t;

int ldc_swparam_set(uint8_t port, uint8_t *enable, ldc_param_s *pldc);
int ldc_swparam_get(uint8_t port, uint8_t *enable, ldc_param_s *pldc);
int dis_swparam_set(uint8_t port, dis_param_s *pdis);
int dis_swparam_get(uint8_t port, dis_param_s *pdis);
int pattgen_param_set(uint8_t port, pg_param_s *ppg);
int pattgen_param_get(uint8_t port, pg_param_s *ppg);
int start_pg_pulse(uint8_t port, ldc_param_s *ppg);
int dwe_init_api(dwe_context_t *ctx, struct dwe_dev_s *pdev, dwe_param_t **pparam);
int dwe_reset_api(dwe_context_t *ctx);
void dwe_deinit_api(dwe_context_t *ctx);
int ldc_hwparam_set(dwe_context_t *ctx, uint8_t port, uint8_t setting);
int dis_hwparam_set(dwe_context_t *ctx, uint8_t port, uint8_t setting);
int ldc_hwpath_set(dwe_context_t *ctx, uint8_t port);
int dis_hwpath_set(dwe_context_t *ctx, uint8_t port);


#endif /* __ACAMERA_DWE_API_H__ */
