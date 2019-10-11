/*  
 *
 *   Copyright (C) 2018 Horizon Inc.
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

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/dmaengine.h>
#include <linux/compiler.h>
#include <asm-generic/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/mman.h>

#include <linux/device.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include "acamera_logger.h"

#include "system_dwe_api.h"
#include "dwe_dev.h"


/* global variable define */
static dwe_param_t dwe_param[FIRMWARE_CONTEXT_NUMBER];
static struct dwe_dev_s *dev_ptr;

//used by gdc model
//model 0: gdc_0  1: gdc_1
//enable 0: disabel 1: enable
void init_gdc0_status(uint32_t model, uint32_t *enable)
{
	if (dev_ptr != NULL)
        	get_gdc_status(dev_ptr->dis_dev->io_vaddr, enable);
}
EXPORT_SYMBOL(init_gdc0_status);

void init_gdc0_mask(uint32_t model, uint32_t *enable)
{
	if (dev_ptr != NULL)
        	set_gdc_mask(dev_ptr->dis_dev->io_vaddr, enable);
}
EXPORT_SYMBOL(init_gdc0_mask);

void init_gdc1_status(uint32_t model, uint32_t *enable)
{
	if (dev_ptr != NULL)
        	get_gdc_status(dev_ptr->dis_dev->io_vaddr, enable);
}
EXPORT_SYMBOL(init_gdc1_status);

void init_gdc1_mask(uint32_t model, uint32_t *enable)
{
	if (dev_ptr != NULL)
        	set_gdc_mask(dev_ptr->dis_dev->io_vaddr, enable);
}
EXPORT_SYMBOL(init_gdc1_mask);

//driver <--> user  by chardev
int ldc_swparam_set(uint8_t port, uint8_t *enable, ldc_param_s *pldc)
{
	int ret = 0;
	
	if ( port > FIRMWARE_CONTEXT_NUMBER) {
		ret = -1;
	} else {	
		dwe_param[port].ldc_enable = *enable;
		if (pldc)	
			memcpy(&dwe_param[port].ldc_param, pldc, sizeof(ldc_param_s));
		else
			ret = -1;	
	}
	
	if (ret == 0) {	
		if ((FIRMWARE_CONTEXT_NUMBER < HADRWARE_CONTEXT_MAX) && (dev_ptr != NULL)) {
			set_chn_ldc_param(dev_ptr->ldc_dev->io_vaddr, &dwe_param[port].ldc_param, port);
		}		
	}
	
	return ret;
} 

int ldc_swparam_get(uint8_t port, uint8_t *enable, ldc_param_s *pldc)
{
	int ret = 0;

	if ( port > FIRMWARE_CONTEXT_NUMBER) {
		ret = -1;
	} else {
		*enable = dwe_param[port].ldc_enable;
		if (pldc)	
			memcpy(&dwe_param[port].ldc_param, pldc, sizeof(ldc_param_s));
		else
			ret = -1;	
	}
	
	return ret;
} 

int dis_swparam_set(uint8_t port, dis_param_s *pdis)
{
	int ret = 0;

	if ( port > FIRMWARE_CONTEXT_NUMBER) {
		ret = -1;
	} else {
		if (pdis)	
			memcpy(&dwe_param[port].dis_param, pdis, sizeof(dis_param_s));
		else
			ret = -1;	
	}
	
	if (ret == 0) {	
		if ((FIRMWARE_CONTEXT_NUMBER < HADRWARE_CONTEXT_MAX) && (dev_ptr != NULL)) {
			set_chn_dis_param(dev_ptr->ldc_dev->io_vaddr, &dwe_param[port].dis_param, port);
		}	
	}
	return ret;
} 

int dis_swparam_get(uint8_t port, dis_param_s *pdis)
{
	int ret = 0;

	if ( port > FIRMWARE_CONTEXT_NUMBER) {
		ret = -1;
	} else {
		if (pdis)	
			memcpy(pdis, &dwe_param[port].dis_param, sizeof(dis_param_s));
		else
			ret = -1;	
	}
	
	return ret;
}

int pattgen_param_set(uint8_t port, pg_param_s *ppg)
{
	int ret  = 0;

	if ( port > FIRMWARE_CONTEXT_NUMBER) {
		ret = -1;
	} else {
		if (ppg)	
			memcpy(ppg, &dwe_param[port].pg_param, sizeof(pg_param_s));
		else
			ret = -1;	
	}
	
	return ret;
} 

int pattgen_param_get(uint8_t port, pg_param_s *ppg)
{
	int ret = 0;

	if ( port > FIRMWARE_CONTEXT_NUMBER) {
		ret = -1;
	} else {
		if (ppg)	
			memcpy(ppg, &dwe_param[port].pg_param, sizeof(pg_param_s));
		else
			ret = -1;	
	}
	
	return ret;
}

int start_pg_pulse(uint8_t port, ldc_param_s *ppg)
{
	int ret = 0;
	return ret;
}

//user by subdev
int dwe_init_api(dwe_context_t *ctx, struct dwe_dev_s *pdev, dwe_param_t **pparam)
{
	int ret = 0;
	uint32_t tmp = 0;
	void *ptr = NULL;

	if (pdev != NULL)
		dev_ptr = pdev;
	
	*pparam = dwe_param;

	ptr = kzalloc( DIS_STAT_SIZE, GFP_KERNEL);
        if (ptr == NULL) {
                ret = -1;
        }

       	ctx->ptr_mem = ptr; 

	ctx->prev_port = -1;
	ctx->curr_port = -1;
	ctx->next_port = -1;
	
	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER; tmp++) {
		//the data is temp, ldc is bypass
		dwe_param[tmp].ldc_param.path_sel.path_g = 0x2000;
		dwe_param[tmp].ldc_param.y_start_addr = 0;
		dwe_param[tmp].ldc_param.c_start_addr = 0xa0000;
		dwe_param[tmp].ldc_param.picsize.size_g = 0x2cf04ff;
		dwe_param[tmp].ldc_param.line_buf = 0x63;
		dwe_param[tmp].ldc_param.x_param.param_g = 0x11003f0;
		dwe_param[tmp].ldc_param.y_param.param_g = 0xf003f0;
		dwe_param[tmp].ldc_param.off_shift.off_g = 0x02fe;
		dwe_param[tmp].ldc_param.woi_x.woi_g = 0x4ff;
		dwe_param[tmp].ldc_param.woi_y.woi_g = 0x2cf;
		dwe_param[tmp].ldc_enable = 0;
		dwe_param[tmp].dis_param.picsize.psize_g = 0x2cf04ff;
		dwe_param[tmp].dis_param.setting.set_g = 0x0;
		dwe_param[tmp].dis_param.dis_h_ratio = 0x10000;
		dwe_param[tmp].dis_param.dis_v_ratio = 0x10000;
		dwe_param[tmp].dis_param.crop_x.crop_g = 0x4ff0000;
		dwe_param[tmp].dis_param.crop_y.crop_g = 0x2cf0000;
		dwe_param[tmp].pg_param.size.psize_g = 0x2cf04ff;
	}
	
	return ret;
}

int dwe_reset_api(dwe_context_t *ctx)
{
	int ret = 0;
	uint32_t tmp = 0;

	if (ctx == NULL) {
		return -1;
	}
	
	ctx->prev_port = -1;
	ctx->curr_port = -1;
	ctx->next_port = -1;
	
	dev_ptr = NULL; 
	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER; tmp++) {
		//the data is temp, ldc is bypass
		dwe_param[tmp].ldc_enable = 0;
		dwe_param[tmp].dis_param.setting.set_g = 0x0;
	}
	
	return ret;
}

void dwe_deinit_api(dwe_context_t *ctx)
{
	dev_ptr = NULL;
	kfree(ctx->ptr_mem);
}

/* 
 * ldc & dis 
 * if  port == 1
 *     using setting[0];
 * if  1 < port < 5
 *     using setting[0---port-1]
 * if  port >= 5
 *     using setting[0,1],
 *   
 */
int ldc_hwparam_set(dwe_context_t *ctx, uint8_t port, uint8_t setting)
{
	int ret = 0;
	uint32_t tmp_cur = 0;
	
	if ((ctx == NULL) || (port > FIRMWARE_CONTEXT_NUMBER)) {
		return -1;
	}
	
	ctx->next_port = port;
	get_dwe_cur_index(dev_ptr->ldc_dev->io_vaddr, &tmp_cur);
	ctx->ldc_cur = tmp_cur;	
	if ((FIRMWARE_CONTEXT_NUMBER > HADRWARE_CONTEXT_MAX) && (dwe_param[port].ldc_enable)) {
		set_chn_ldc_param(dev_ptr->ldc_dev->io_vaddr, &dwe_param[port].ldc_param, ((tmp_cur & 0x01)  ^ 1));
	}
	
	
	return ret;
}

#if 0
int ldc_hwparam_get(dwe_context_t *ctx, uint8_t port, uint8_t setting)
{
}
#endif 

int dis_hwparam_set(dwe_context_t *ctx, uint8_t port, uint8_t setting)
{
	int ret = 0;
	uint32_t tmp_cur = 0;
	
	if ((ctx == NULL) || (port > FIRMWARE_CONTEXT_NUMBER)) {
		return -1;
	}

	if ( port > FIRMWARE_CONTEXT_NUMBER) {
		ret = -1;
	} else {
		get_dwe_cur_index(dev_ptr->dis_dev->io_vaddr, &tmp_cur);
		ctx->ldc_cur = tmp_cur;	
		if (FIRMWARE_CONTEXT_NUMBER > HADRWARE_CONTEXT_MAX) {
			set_chn_dis_param(dev_ptr->dis_dev->io_vaddr, &dwe_param[port].dis_param, ((tmp_cur & 0x01)  ^ 1));
		}
	}
	
	return ret;
}

#if 0
int dis_hwparam_get(dwe_context_t *ctx, uint8_t port, uint8_t setting)
{
}
#endif


/*
 *  if ldc == bypass
 *     set lypas_ldc == 1
 *  else
 *     if FIRMWARE_CONTEXT_NUMBER > 4
 *        setting[0-1]
 *     else
 *        setting[0-3]
 */
int ldc_hwpath_set(dwe_context_t *ctx, uint8_t port)
{
	int ret = 0;
	uint32_t tmp_cur = 0;
	uint32_t set_tmp = ctx->ldc_setting.set_g;

	if ((ctx == NULL) || (port > FIRMWARE_CONTEXT_NUMBER)) {
		return -1;
	}
	
	if ( dwe_param[port].ldc_enable == 0 ) {
		set_ldc_bypass(dev_ptr->ldc_dev->io_vaddr, &dwe_param[port].ldc_enable);
	} else {
		if (FIRMWARE_CONTEXT_NUMBER == 1) {
		} else if ( FIRMWARE_CONTEXT_NUMBER > HADRWARE_CONTEXT_MAX) {
			set_tmp |= 0xFF0000;	
			set_tmp &= (tmp_cur << (16 + tmp_cur * 2));
			set_ldc_setting(dev_ptr->ldc_dev->io_vaddr, &set_tmp);
		} else { 
			set_tmp |= 0xFF0000;	
			set_tmp &= (tmp_cur << (16 + port * 2));
			set_ldc_setting(dev_ptr->ldc_dev->io_vaddr, &set_tmp);
		}
	}

	return ret;	
}

/*
 *     setting image_size
 * 
 *     if FIRMWARE_CONTEXT_NUMBER > 4
 *        setting[0-1]
 *     else
 *        setting[0-3]
 */
int dis_hwpath_set(dwe_context_t *ctx, uint8_t port)
{
	int ret = 0;
	uint32_t tmp_cur = 0;

	uint32_t set_tmp = ctx->dis_setting.set_g;
	uint32_t size_tmp = dwe_param[port].dis_param.picsize.psize_g;

	if ((ctx == NULL) || (port > FIRMWARE_CONTEXT_NUMBER)) {
		return -1;
	}

	if (FIRMWARE_CONTEXT_NUMBER == 1) {
	} else if ( FIRMWARE_CONTEXT_NUMBER > HADRWARE_CONTEXT_MAX) {
		set_dwe_image_size(dev_ptr->dis_dev->io_vaddr, &size_tmp);
		set_tmp |= 0xFF0000;	
		set_tmp &= (tmp_cur << (16 + tmp_cur * 2));
		set_dwe_setting(dev_ptr->dis_dev->io_vaddr, &set_tmp);
		//TODO buffer	
	} else { 
		set_dwe_image_size(dev_ptr->dis_dev->io_vaddr, &size_tmp);
		set_tmp |= 0xFF0000;	
		set_tmp &= (tmp_cur << (16 + port * 2));
		set_dwe_setting(dev_ptr->dis_dev->io_vaddr, &set_tmp);
		//TODO buffer
	}
	
	return ret;	
}
