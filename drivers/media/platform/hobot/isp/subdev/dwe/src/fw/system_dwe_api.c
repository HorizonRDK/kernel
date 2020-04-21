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

#define pr_fmt(fmt) "[ldc_drv]: %s: " fmt, __func__

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
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/mman.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include "acamera_logger.h"

#include "system_dwe_api.h"
#include "dwe_dev.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

/* global variable define */
static dwe_param_t dwe_param[FIRMWARE_CONTEXT_NUMBER];
static struct dwe_dev_s *dev_ptr;
struct mutex mc_mutex;

typedef void(*rst_func)(void);

static rst_func gdc_call_back = NULL;
void gdc_call_install(rst_func p)
{
	gdc_call_back = p;
}
EXPORT_SYMBOL_GPL(gdc_call_install);

void gdc_rst_func(void)
{
	if (gdc_call_back) {
		gdc_call_back();
	}
}

//used by gdc model
//model 0: gdc_0  1: gdc_1
//enable 0: disabel 1: enable
void read_gdc_status(uint32_t model, uint32_t *enable)
{
	if (dev_ptr != NULL) {
		if (model == 0) {
			if (dev_ptr->dis_dev != NULL)
				get_gdc_status(dev_ptr->dis_dev->io_vaddr, enable);
		} else {
			if (dev_ptr->dis1_dev != NULL)
				get_gdc_status(dev_ptr->dis1_dev->io_vaddr, enable);
		}
	}
}
EXPORT_SYMBOL_GPL(read_gdc_status);

void write_gdc_status(uint32_t model, uint32_t *enable)
{
	if (dev_ptr != NULL) {
		if (model == 0) {
			if (dev_ptr->dis_dev != NULL)
				set_gdc_status(dev_ptr->dis_dev->io_vaddr, enable);
		} else {
			if (dev_ptr->dis1_dev != NULL)
				set_gdc_status(dev_ptr->dis1_dev->io_vaddr, enable);
		}
	}
}
EXPORT_SYMBOL_GPL(write_gdc_status);

void write_gdc_mask(uint32_t model, uint32_t *enable)
{
	if (dev_ptr != NULL) {
		if (model == 0) {
			if (dev_ptr->dis_dev != NULL)
				set_gdc_mask(dev_ptr->dis_dev->io_vaddr, enable);
		} else {
			if (dev_ptr->dis1_dev != NULL)
				set_gdc_mask(dev_ptr->dis1_dev->io_vaddr, enable);
		}
	}
}
EXPORT_SYMBOL_GPL(write_gdc_mask);

void read_gdc_mask(uint32_t model, uint32_t *enable)
{
	if (dev_ptr != NULL) {
		if (model == 0) {
			if (dev_ptr->dis_dev != NULL)
				get_gdc_mask(dev_ptr->dis_dev->io_vaddr, enable);
		} else {
			if (dev_ptr->dis1_dev != NULL)
				get_gdc_mask(dev_ptr->dis1_dev->io_vaddr, enable);
		}
	}
}
EXPORT_SYMBOL_GPL(read_gdc_mask);

void printk_ldcparam(ldc_param_s *pldc)
{
	LOG(LOG_INFO, "ldc_enable %d, path_sel %d, y_addr %d, c_addr %d, pic %d, line_buf %d, x_param %d, y_param %d, off shift %d, woi_x %d ,woi_y %d", pldc->ldc_enable, pldc->path_sel.path_g, pldc->y_start_addr, pldc->c_start_addr, pldc->picsize.size_g, pldc->line_buf, pldc->x_param.param_g, pldc->y_param.param_g, pldc->off_shift.off_g, pldc->woi_x.woi_g, pldc->woi_y.woi_g);
}

void printk_disparam(dis_param_s *pdis)
{
	LOG(LOG_INFO, "pic %d, path %d, h_rat %d, v_rat %d, crop_x %d, crop_y %d.", pdis->picsize.psize_g, pdis->path.path_g, pdis->dis_h_ratio, pdis->dis_v_ratio, pdis->crop_x.crop_g, pdis->crop_y.crop_g);
}

void printk_pgparam(pg_param_s *ppg)
{
	LOG(LOG_INFO, "pic %d, blank %d.", ppg->size.psize_g, ppg->blank.blank_g);
}

int set_ldc_param(uint32_t port, uint32_t *ptr, uint32_t size)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		ret = -1;
	} else {
		if (ptr) {
			memcpy(&dwe_param[port].ldc_param, ptr, size);
		} else {
			LOG(LOG_ERR, "---pldc is err!---");
			ret = -1;
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(set_ldc_param);

int get_ldc_param(uint32_t port, uint32_t *ptr, uint32_t size)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		ret = -1;
	} else {
		if (ptr) {
			memcpy(ptr, &dwe_param[port].ldc_param, size);
		} else {
			LOG(LOG_ERR, "---pldc is err!---");
			ret = -1;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(get_ldc_param);

extern int dis_set_ioctl(uint32_t port, uint32_t online);
extern int ldc_set_ioctl(uint32_t port, uint32_t online);

//driver <--> user  by chardev
int ldc_swparam_set(uint32_t port, ldc_param_s *pldc)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		ret = -1;
	} else {
		if (pldc) {
			memcpy(&dwe_param[port].ldc_param, pldc, sizeof(ldc_param_s));
		} else {
			LOG(LOG_ERR, "---pldc is null!---");
			ret = -1;
		}
	}

	printk_ldcparam(&dwe_param[port].ldc_param);

	if (dwe_param[port].ldc_param.ldc_enable == 0) {
		dwe_param[port].ldc_param.x_param.param_g = 0;
		dwe_param[port].ldc_param.y_param.param_g = 0;
		dwe_param[port].ldc_param.off_shift.off_g = 0;
	}
#if 0
	if (ret == 0) {
		if ((port < HADRWARE_CONTEXT_MAX) && (dev_ptr != NULL)) {
			printk(KERN_INFO "%s -- %d .\n", __func__, __LINE__);
			set_chn_ldc_param(dev_ptr->ldc_dev->io_vaddr, &dwe_param[port].ldc_param, 0);
		}
	}
	uint32_t temp = 1;
	set_ldc_bypass(dev_ptr->ldc_dev->io_vaddr, &temp);
#endif
	ldc_set_ioctl(port, 1);

	LOG(LOG_DEBUG, "port is %d", port);

	return ret;
}

int ldc_swparam_get(uint32_t port, ldc_param_s *pldc)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		ret = -1;
	} else {
		if (pldc) {
			memcpy(pldc, &dwe_param[port].ldc_param, sizeof(ldc_param_s));
		} else {
			LOG(LOG_ERR, "---pldc is err!---");
			ret = -1;
		}
	}

	printk_ldcparam(&dwe_param[port].ldc_param);
	LOG(LOG_DEBUG, "port is %d", port);
	return ret;
}

int dis_swparam_set(uint32_t port, dis_param_s *pdis)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		ret = -1;
	} else {
		if (pdis) {
			memcpy(&dwe_param[port].dis_param, pdis, sizeof(dis_param_s));
		} else {
			LOG(LOG_ERR, "---pdis is null!---");
			ret = -1;
		}
	}

	printk_disparam(&dwe_param[port].dis_param);

	LOG(LOG_DEBUG, "port is %d", port);
#if 0
	//temp
	if (ret == 0) {
		if ((port < HADRWARE_CONTEXT_MAX) && (dev_ptr != NULL)) {
			printk(KERN_INFO "%s -- %d .\n", __func__, __LINE__);
			set_chn_dis_param(dev_ptr->dis_dev->io_vaddr,
				&dwe_param[port].dis_param, 0);
		}
	}
#endif
	dis_set_ioctl(port, 1);
	return ret;
}

int dis_swparam_get(uint32_t port, dis_param_s *pdis)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		ret = -1;
	} else {
		if (pdis) {
			memcpy(pdis, &dwe_param[port].dis_param, sizeof(dis_param_s));
		} else {
			LOG(LOG_ERR, "---pdis is null!---");
			ret = -1;
		}
	}

	LOG(LOG_DEBUG, "port is %d", port);
	printk_disparam(&dwe_param[port].dis_param);
	return ret;
}

int pattgen_param_set(uint32_t port, pg_param_s *ppg)
{
	int ret  = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		ret = -1;
	} else {
		if (ppg) {
			memcpy(&dwe_param[port].pg_param, ppg, sizeof(pg_param_s));
		} else {
			LOG(LOG_ERR, "---ppg is err!---");
			ret = -1;
		}
	}

	printk_pgparam(&dwe_param[port].pg_param);
	LOG(LOG_DEBUG, "port is %d", port);

	return ret;
}

int pattgen_param_get(uint32_t port, pg_param_s *ppg)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		ret = -1;
	} else {
		if (ppg) {
			memcpy(ppg, &dwe_param[port].pg_param, sizeof(pg_param_s));
		} else {
			LOG(LOG_ERR, "---ppg is err!---");
			ret = -1;
		}
	}

	printk_pgparam(&dwe_param[port].pg_param);
	LOG(LOG_DEBUG, "port is %d", port);
	return ret;
}

int start_pg_pulse(uint32_t port)
{
	int ret = 0;
	uint32_t tmp = 1;

	mutex_lock(&mc_mutex);
	set_dwe_pg_size(dev_ptr->dis_dev->io_vaddr,
		&dwe_param[port].pg_param.size.psize_g);
	set_dwe_pg_blanking(dev_ptr->dis_dev->io_vaddr,
		&dwe_param[port].pg_param.blank.blank_g);
	set_dwe_pg_start(dev_ptr->dis_dev->io_vaddr, &tmp);
	mutex_unlock(&mc_mutex);

	LOG(LOG_DEBUG, "--pg_pulse--");

	return ret;
}

int pg_mode_enable(uint32_t input)
{
	int ret = 0;
	dis_checktype_u tmp;

	get_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp.type_g);

	//tmp.type_b.rg_pg_enable = (input & 0xff);
	//tmp.type_b.rg_pg_mode = (input & 0xff00) >> 8;
	tmp.type_g = input;

	set_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp.type_g);
	LOG(LOG_DEBUG, "--pg_mode 0x%x--", tmp.type_g);

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

	ptr = kzalloc(DIS_STAT_SIZE, GFP_KERNEL);
	if (ptr == NULL) {
		LOG(LOG_ERR, "---kzalloc is failed!---");
		ret = -1;
	}

	ctx->ptr_mem = ptr;
	ctx->phy_mem = (uint32_t)__virt_to_phys(ptr);

#if 0
	for (tmp = 0; tmp < HADRWARE_CONTEXT_MAX; tmp++) {
		set_chn_dis_addr(dev_ptr->dis_dev->io_vaddr, &ctx->phy_mem, tmp);
	}

	dwe_sw_init();
#endif

	ctx->ldc_running = 0;
	ctx->ldc_dev_num = 0;
	ctx->ldc_curr_port = 0;
	ctx->ldc_next_port = -1;
	ctx->ldc_update = 0;
	ctx->ldc_cur_num = 0;
	ctx->dis_running = 0;
	ctx->dis_dev_num = 0;
	ctx->dis_curr_port = 0;
	ctx->dis_next_port = -1;
	ctx->dis_update = 0;
	ctx->dis_cur_num = 0;
	ctx->online_enable = 1;
	ctx->online_port = 0;

	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER; tmp++) {
		//the data is temp, ldc is bypass
		dwe_param[tmp].ldc_param.ldc_enable = 0;
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
		dwe_param[tmp].dis_param.picsize.psize_g = 0x2cf04ff;
		dwe_param[tmp].dis_param.path.path_g = 0x0;
		dwe_param[tmp].dis_param.dis_h_ratio = 0x10000;
		dwe_param[tmp].dis_param.dis_v_ratio = 0x10000;
		dwe_param[tmp].dis_param.crop_x.crop_g = 0x4ff0000;
		dwe_param[tmp].dis_param.crop_y.crop_g = 0x2cf0000;
		dwe_param[tmp].pg_param.size.psize_g = 0x2cf04ff;
	}

	mutex_init(&mc_mutex);
	return ret;
}

int dwe_reset_api(dwe_context_t *ctx)
{
	int ret = 0;
	uint32_t tmp = 0;

	if (ctx == NULL) {
		return -1;
	}

	ctx->ldc_running = 0;
	ctx->ldc_dev_num = 0;
	ctx->ldc_curr_port = 0;
	ctx->ldc_next_port = -1;
	ctx->ldc_update = 0;
	ctx->ldc_cur_num = 0;
	ctx->dis_running = 0;
	ctx->dis_dev_num = 0;
	ctx->dis_curr_port = 0;
	ctx->dis_next_port = -1;
	ctx->dis_update = 0;
	ctx->dis_cur_num = 0;
	ctx->online_enable = 1;
	ctx->online_port = 0;
	
	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER; tmp++) {
		//the data is temp, ldc is bypass
		dwe_param[tmp].ldc_param.ldc_enable = 0;
		dwe_param[tmp].dis_param.path.path_g = 0x0;
	}

	LOG(LOG_DEBUG, "---[%s-%d]---", __func__, __LINE__);

	return ret;
}

void dwe_sw_init(void)
{
	uint32_t tmp = 0;

	//init ldc
	tmp = 0x03;
	set_ldc_soft_reset(dev_ptr->ldc_dev->io_vaddr, &tmp);

	tmp = 0x80fc0000;
	set_ldc_setting(dev_ptr->ldc_dev->io_vaddr, &tmp);

	tmp = 0xff;
	set_ldc_int_status(dev_ptr->ldc_dev->io_vaddr, &tmp);

	tmp = 0xff;
	set_ldc_int_mask(dev_ptr->ldc_dev->io_vaddr, &tmp);

	tmp = 0x02;
	set_ldc_soft_reset(dev_ptr->ldc_dev->io_vaddr, &tmp);

	//init dis
	get_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp);
	tmp |= 0x4;
	set_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp);

	tmp = 0x80fc0000;
	set_dwe_setting(dev_ptr->dis_dev->io_vaddr, &tmp);

	tmp = 0xf;
	set_dwe_int_status(dev_ptr->dis_dev->io_vaddr, &tmp);

	tmp = 0x1;
	set_dwe_int_mask(dev_ptr->dis_dev->io_vaddr, &tmp);

	get_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp);
	tmp &= 0xfb;
	set_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp);
}

extern void reset_dwe_ctx(void);

void dwe_sw_deinit(void)
{
	uint32_t tmp = 0;
	uint32_t tmp_dis = 0;
	uint32_t count = 0;

	while (count < 5) {
		get_ldc_int_status(dev_ptr->ldc_dev->io_vaddr, &tmp);
		get_dwe_int_status(dev_ptr->dis_dev->io_vaddr, &tmp_dis);
		if ((tmp == 0) && (tmp_dis == 0)) {
			break;
		} else {
			mdelay(10);
		}
		count++;
	}

	//init ldc
	tmp = 0x03;
	set_ldc_soft_reset(dev_ptr->ldc_dev->io_vaddr, &tmp);

	tmp = 0x0;
	set_ldc_int_mask(dev_ptr->ldc_dev->io_vaddr, &tmp);

	tmp = 0xff;
	set_ldc_int_status(dev_ptr->ldc_dev->io_vaddr, &tmp);

	tmp = 0x80fc0000;
	set_ldc_setting(dev_ptr->ldc_dev->io_vaddr, &tmp);

	tmp = 0x02;
	set_ldc_soft_reset(dev_ptr->ldc_dev->io_vaddr, &tmp);

	//init dis
	get_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp);
	tmp |= 0x4;
	set_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp);

	tmp = 0x0;
	set_dwe_int_mask(dev_ptr->dis_dev->io_vaddr, &tmp);

	tmp = 0xf;
	set_dwe_int_status(dev_ptr->dis_dev->io_vaddr, &tmp);

	tmp = 0x80fc0000;
	set_dwe_setting(dev_ptr->dis_dev->io_vaddr, &tmp);

	get_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp);
	tmp &= 0xfb;
	set_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &tmp);

	gdc_rst_func();
	reset_dwe_ctx();
}

void dwe_deinit_api(dwe_context_t *ctx)
{
	dwe_sw_deinit();

	dev_ptr = NULL;
	kfree(ctx->ptr_mem);
	LOG(LOG_DEBUG, "dwe_deinit_api is success");
}

//debug info
void ldc_printk_info(void)
{
	uint32_t tmp = 0;
	uint32_t addr = 0;
	uint32_t count = 0;

	addr = 0x4;
	ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	addr = 0x8;
	ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	addr = 0x40;
	ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	addr = 0x44;
	ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	//channl0
	for(count = 0; count < 2; count++) {
		addr = 0x100 + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x104 + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x108 + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x140 + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x144 + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x148 + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x14c + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x150 + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x180 + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x184 + count * 0x100;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	}
	addr = 0x708;
	ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	for(count = 0; count < 10; count++) {
		addr = 0x800 + count * 4;
		ldc_debug_info(dev_ptr->ldc_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	}
}

//debug info
void dwe_printk_info(void)
{
	uint32_t tmp = 0;
	uint32_t addr = 0;
	uint32_t count = 0;

	addr = 0x0;
	dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	addr = 0x4;
	dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	addr = 0x8;
	dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	addr = 0x10;
	dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	addr = 0x14;
	dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	addr = 0x40;
	dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	addr = 0x44;
	dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
	LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	//channel
	for(count = 0; count < 2; count++) {
		addr = 0x100 + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x104 + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x108 + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x140 + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x144 + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x148 + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x14c + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x150 + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x180 + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x184 + count * 0x100;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	}

	//info
	for(count = 0; count < 9; count++) {
		addr = 0x700 + count * 4;
		dwe_debug_info(dev_ptr->dis_dev->io_vaddr, addr, &tmp);
		LOG(LOG_DEBUG, "[dump] addr 0x%x, data 0x%x", addr, tmp);
	}
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
int ldc_hwparam_set(dwe_context_t *ctx, uint32_t port)
{
	int ret = 0;
	uint32_t tmp_cur = 0;
	
	if ((ctx == NULL) || (port >= FIRMWARE_CONTEXT_NUMBER)) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		return -EINVAL;
	}

	
	get_ldc_setting(dev_ptr->ldc_dev->io_vaddr, &tmp_cur);
	tmp_cur = (tmp_cur & 0x30000) >> 16;
	if (tmp_cur == 0) {
		ctx->ldc_dev_num = 1;
	} else {
		ctx->ldc_dev_num = 0;
	}

	set_chn_ldc_param(dev_ptr->ldc_dev->io_vaddr,
		&dwe_param[port].ldc_param, ctx->ldc_dev_num);

	LOG(LOG_DEBUG, "ldc_hwparam_set success!, ldc_dev_num %d", ctx->ldc_dev_num);
	return ret;
}

int dis_hwparam_set(dwe_context_t *ctx, uint32_t port)
{
	int ret = 0;
	uint32_t tmp_cur = 0;
	uint32_t tmp_addr = 0;
	
	if ((ctx == NULL) || (port >= FIRMWARE_CONTEXT_NUMBER)) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		return -EINVAL;
	}

	get_dwe_setting(dev_ptr->dis_dev->io_vaddr, &tmp_cur);
	tmp_cur = (tmp_cur & 0x30000) >> 16;
	if (tmp_cur == 0) {
		ctx->dis_dev_num = 1;
	} else {
		ctx->dis_dev_num = 0;
	}

	set_chn_dis_param(dev_ptr->dis_dev->io_vaddr,
		&dwe_param[port].dis_param, ctx->dis_dev_num);
	if (dwe_param[port].dis_param.path.path_b.rg_dis_enable == 1) {
		ret = dwe_stream_get_frame(port, &ctx->dframes[port]);
		if (ret < 0) {
			ctx->dframes[port].address = ctx->phy_mem;
			ctx->dframes[port].virt_addr = ctx->ptr_mem;
			LOG(LOG_ERR, "port %d get buffer failed!\n", port);
		} else {
			tmp_addr = ctx->dframes[port].address;
			LOG(LOG_INFO, "port %d, addr is %d !\n", port, ctx->dframes[port].address);
		}
	} else {
		ctx->dframes[port].address = ctx->phy_mem;
		ctx->dframes[port].virt_addr = ctx->ptr_mem;
		tmp_cur = dwe_param[port].dis_param.path.path_g | 1;
		//set_chn_dis_setting(dev_ptr->dis_dev->io_vaddr, &tmp_cur, ctx->dis_dev_num);
	}

	set_chn_dis_addr(dev_ptr->dis_dev->io_vaddr,
		&ctx->dframes[port].address, ctx->dis_dev_num);

	LOG(LOG_DEBUG, "dis_hwparam_set success!, dis_dev_num %d", ctx->dis_dev_num);
	
	return ret;
}

/*
 *  if ldc == bypass
 *     set lypas_ldc == 1
 *  else
 *     if FIRMWARE_CONTEXT_NUMBER > 4
 *        setting[0-1]
 *     else
 *        setting[0-3]
 */
int ldc_hwpath_set(dwe_context_t *ctx, uint32_t port)
{
	int ret = 0;
	//uint32_t tmp_cur = 0;
	uint32_t tmp_num = 1;
	uint32_t set_tmp = 0;

	if ((ctx == NULL) || (port >= FIRMWARE_CONTEXT_NUMBER)) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		return -EINVAL;
	}

	//get_ldc_cur_index(dev_ptr->ldc_dev->io_vaddr, &tmp_cur);
	//ctx->ldc_cur_num = tmp_cur;

	set_tmp = 0x03;
	set_ldc_soft_reset(dev_ptr->ldc_dev->io_vaddr, &set_tmp);

	get_ldc_setting(dev_ptr->ldc_dev->io_vaddr, &set_tmp);
	set_tmp &= 0x80fc0000;
	set_tmp |= ctx->ldc_dev_num << 16;
	//set_tmp |= (tmp_cur << (16 + ctx->ldc_dev_num * 2));
	set_ldc_setting(dev_ptr->ldc_dev->io_vaddr, &set_tmp);
	LOG(LOG_DEBUG, "num %d,set_tmp %x", ctx->ldc_dev_num, set_tmp);
	if (dwe_param[port].ldc_param.ldc_enable == 1) {
		LOG(LOG_DEBUG, "port %d is enable", port);
		tmp_num = 0;
	} else {
		LOG(LOG_DEBUG, "port %d is disable", port);
		tmp_num = 1;
	}
	set_ldc_bypass(dev_ptr->ldc_dev->io_vaddr, &tmp_num);

	set_tmp = 0x02;
	set_ldc_soft_reset(dev_ptr->ldc_dev->io_vaddr, &set_tmp);

	LOG(LOG_DEBUG, "ldc_hwpath_set success!");
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
int dis_hwpath_set(dwe_context_t *ctx, uint32_t port)
{
	int ret = 0;
	//uint32_t tmp_cur = 0;
	uint32_t set_tmp = 0;
	uint32_t size_tmp = dwe_param[port].dis_param.picsize.psize_g;

	if ((ctx == NULL) || (port >= FIRMWARE_CONTEXT_NUMBER)) {
		LOG(LOG_ERR, "---port %d param is error!---", port);
		return -EINVAL;
	}

	//get_dwe_cur_index(dev_ptr->dis_dev->io_vaddr, &tmp_cur);
	//ctx->dis_cur_num = tmp_cur;

	get_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &set_tmp);
	set_tmp |= 0x4;
	set_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &set_tmp);

	set_dwe_image_size(dev_ptr->dis_dev->io_vaddr, &size_tmp);

	get_dwe_setting(dev_ptr->dis_dev->io_vaddr, &set_tmp);
	set_tmp &= 0x80fc0000;
	set_tmp |= ctx->dis_dev_num << 16;
	//set_tmp |= (tmp_cur << (16 + ctx->dis_dev_num * 2));
	set_dwe_setting(dev_ptr->dis_dev->io_vaddr, &set_tmp);
	LOG(LOG_DEBUG, "num %d,set_tmp %x", ctx->dis_dev_num, set_tmp);

	get_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &set_tmp);
	set_tmp &= 0xfb;
	set_dwe_checktype(dev_ptr->dis_dev->io_vaddr, &set_tmp);

	LOG(LOG_DEBUG, "dis_hwpath_set success!");
	return ret;
}
