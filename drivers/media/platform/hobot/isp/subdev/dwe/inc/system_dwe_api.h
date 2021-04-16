/*
 *
 *    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#ifndef __ACAMERA_DWE_API_H__
#define __ACAMERA_DWE_API_H__

#include "dis_base_io.h"
#include "ldc_base_io.h"
#include "dwe_dev.h"
#include "acamera_dwe_config.h"
#include "buffer_v4l2_stream.h"

typedef struct _dwe_param_s {
	dis_param_s dis_param;
	uint32_t ldc_enable;
	ldc_param_s ldc_param;
	pg_param_s pg_param;
} dwe_param_t;


typedef struct _dwe_context_t {
	//dis reserver memory
	// for ctx dump
	struct ion_client *client;
	struct ion_handle *handle;
	void *ptr_mem;
	phys_addr_t phy_mem;
	size_t mem_size;
	// reserved mem end
	uint32_t ldc_irqstatus;
	uint32_t dis_irqsattus;
	dis_setting_u dis_setting;
	ldc_setting_u ldc_setting;
        uint8_t ldc_running;
	uint32_t ldc_dev_num;
	int ldc_curr_port;
	int ldc_next_port;
	int ldc_update;
	uint32_t ldc_cur_num;
	uint8_t dis_running;
	uint32_t dis_dev_num;
	int dis_curr_port;
	int dis_next_port;
	int dis_update;
	uint32_t dis_cur_num;
	uint32_t online_enable;
	uint32_t online_port;
	dframe_t dframes[FIRMWARE_CONTEXT_NUMBER];
} dwe_context_t;

int ldc_swparam_set(uint32_t port, ldc_param_s *pldc, uint32_t ctrl_enable);
int ldc_swparam_get(uint32_t port, ldc_param_s *pldc);
int dis_swparam_set(uint32_t port, dis_param_s *pdis, uint32_t ctrl_enable);
int dis_swparam_get(uint32_t port, dis_param_s *pdis);
int pattgen_param_set(uint32_t port, pg_param_s *ppg);
int pattgen_param_get(uint32_t port, pg_param_s *ppg);
int start_pg_pulse(uint32_t port);
int pg_mode_enable(uint32_t input);
int dwe_init_api(dwe_context_t *ctx, struct dwe_dev_s *pdev, dwe_param_t **pparam);
int dwe_reset_api(dwe_context_t *ctx);
void dwe_deinit_api(dwe_context_t *ctx);
int ldc_hwparam_set(dwe_context_t *ctx, uint32_t port);
int dis_hwparam_set(dwe_context_t *ctx, uint32_t port);
int ldc_hwpath_set(dwe_context_t *ctx, uint32_t port);
int dis_hwpath_set(dwe_context_t *ctx, uint32_t port);
void ldc_printk_info(void);
void dwe_printk_info(void);
uint32_t get_dis_status(uint32_t port);

void dwe_sw_init(void);
void dwe_sw_deinit(void);

#endif /* __ACAMERA_DWE_API_H__ */
