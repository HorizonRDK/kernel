/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <asm-generic/io.h>

#include "isp_dev_regs.h"
#include "x2_isp.h"
#include "isp_base.h"
#include "camera_13855.h"

struct isp_init_s CAMERA_13855;
unsigned char __iomem *isp_regbase;

#ifdef CONFIG_PM_SLEEP
#define ISP_REGS_LEN	406
uint32_t g_isp_regs[ISP_REGS_LEN];

void x2_isp_regs_store(void)
{
	int i;

	if (isp_regbase == NULL) {
		pr_info("%s:%s, isp_regbase == NULL\n", __FILE__, __func__);
		return;
	}

	for (i = 0; i < ISP_REGS_LEN; i++)
		g_isp_regs[i] = get_isp_reg(0x4 * i);
}

void x2_isp_regs_restore(void)
{
	int i;
	uint32_t data;

	if (isp_regbase == NULL) {
		pr_info("%s:%s, isp_regbase == NULL\n", __FILE__, __func__);
		return;
	}

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	for (i = 1; i < ISP_REGS_LEN - 1; i++)
		writel(g_isp_regs[i], isp_regbase + 0x4 * i);

	SetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);
}
#endif

int8_t clr_isp_regbase(void)
{
	isp_regbase = NULL;
	return 0;
}

int8_t set_isp_regbase(unsigned char __iomem *base)
{
	isp_regbase = base;
	return 0;
}

int8_t set_isp_wbg(struct ae_input_s *info)
{

	uint32_t data;

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	writel(info->isp_wbg_rg_gain, isp_regbase + HMP_WBG_W_0);
	writel(info->isp_wbg_bg_gain, isp_regbase + HMP_WBG_W_3);

	SetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	return 0;
}

uint32_t get_isp_reg(uint32_t regaddr)
{
	uint32_t d;

	d = readl(isp_regbase + regaddr);

	return d;
}

uint32_t write_isp_reg(struct con_reg_s *info)
{
	void __iomem *addr = NULL;

	addr = isp_regbase + info->isp_reg_addr;
	writel(info->isp_reg_data, addr);

	return 0;
}

int8_t set_isp_reg(struct con_reg_s *info)
{
	void __iomem *addr = NULL;
	uint32_t data;

	addr = isp_regbase + info->isp_reg_addr;

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	writel(info->isp_reg_data, addr);

	SetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);
	return 0;
}

int8_t set_isp_regs(uint32_t *info, uint32_t datalen)
{
	uint32_t data;
	uint32_t temp_i = 0;

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	for (temp_i = 0; temp_i < datalen; (temp_i = temp_i + 2))
		writel(*(info + temp_i + 1), isp_regbase + *(info + temp_i));

	SetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);
	return 0;
}

int8_t get_isp_regs(uint32_t *info, uint32_t datalen)
{
	uint32_t temp_i = 0;

	for (temp_i = 0; temp_i < datalen; (temp_i = temp_i + 2))
		*(info + temp_i + 1) = readl(isp_regbase + *(info + temp_i));

	return 0;
}

int8_t isp_write_regs(uint32_t addr, uint32_t datalen, uint32_t *info)
{
	uint32_t data;
	uint32_t temp_i = 0;

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	for (temp_i = 0; temp_i < datalen; temp_i++)
		writel(*(info + temp_i), (isp_regbase + addr + 4 * temp_i));
	//memcpy_toio((isp_regbase + addr), info, datalen);

	SetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);
	return 0;
}

int8_t isp_read_regs(uint32_t addr, uint32_t datalen, uint32_t *info)
{
	uint32_t temp_i = 0;

	for (temp_i = 0; temp_i < datalen; temp_i++)
		*(info + temp_i) = readl(isp_regbase + addr + 4 * temp_i);
//      memcpy_fromio(info, (isp_regbase + addr), datalen);
	return 0;
}

int8_t set_isp_stf_addr(void)
{
	uint32_t data;

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	writel(SET_TILE_ADDR, isp_regbase + STF_SFE_TILE_ADDR);
	writel(SET_GRID_ADDR, isp_regbase + STF_SFE_GRID_ADDR);
	writel(SET_RSUM_ADDR, isp_regbase + STF_SFE_RSUM_ADDR);
	writel(SET_HIST_ADDR, isp_regbase + STF_SFE_HIST_ADDR);
	writel(SET_CDR_ADDR0, isp_regbase + HMP_CDR_ADDR0);

	SetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	return 0;
}

int8_t set_isp_start(void)
{
	uint32_t data;

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	data = readl(isp_regbase + ISP_CONFIG);
	SetBit(BIT_ISP_EN, (data));
	writel(data, isp_regbase + ISP_CONFIG);

	return 0;
}

int8_t set_isp_stop(void)
{
	uint32_t data;

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	data = readl(isp_regbase + ISP_CONFIG);
	ResetBit(BIT_ISP_EN, (data));
	writel(data, isp_regbase + ISP_CONFIG);

	return 0;
}

int8_t isp_config_enable(void)
{
	uint32_t data;

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	SetBit(BIT_ISP_CONFIG_DONE_EN, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	return 0;
}

int8_t isp_config_disable(void)
{
	uint32_t data;

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE_EN, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	return 0;
}

int8_t isp_cfg_init(void)
{
	int i = 0;
	uint32_t data = 0;
	uint32_t *info = (void *)(&CAMERA_13855);

	data = readl(isp_regbase + ISP_CONFIG_DONE);
	ResetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	writel(0x6b3f7f2, isp_regbase + 0x000);
	writel(0x0f000870, isp_regbase + 0x04);

	memcpy_toio((void *)(isp_regbase + 8), (void *)&CAMERA_13855,
		    sizeof(struct isp_init_s) - 0x24c);
	memcpy_toio((void *)(isp_regbase + 0x400),
		    (void *)(((char *)(&CAMERA_13855)) + 0x318), 0x258);

	SetBit(BIT_ISP_CONFIG_DONE, (data));
	writel(data, isp_regbase + ISP_CONFIG_DONE);

	return 0;
}
