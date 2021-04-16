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

#ifndef __X2_ISP_BASE_H__
#define __X2_ISP_BASE_H__

#include <linux/types.h>
#include <asm/compiler.h>

#define SET_TILE_ADDR       0x000000
#define SET_GRID_ADDR       0x100000
#define SET_RSUM_ADDR       0x200000
#define SET_HIST_ADDR       0x300000
#define SET_CDR_ADDR0       0x400000
#define CDR_SAVING          0x500000

#define CDR_SIZE            0x100000

struct ae_input_s {
	uint32_t isp_wbg_rg_gain;
	uint32_t isp_wbg_bg_gain;
};

struct con_reg_s {
	uint32_t isp_reg_addr;
	uint32_t isp_reg_data;
};

#define SetBit(bit, val) (val = (uint32_t)(val | (0x1 << bit)))
#define ResetBit(bit, val) (val = (uint32_t)(val & ~(0x1 << bit)))

/**
 * isp_get_dev
 * @void: NULL
 *
 * Return: isp_dev_t type, ref isp_dev_s struction.
 */
int8_t set_isp_regbase(unsigned char __iomem *base);
int8_t set_isp_wbg(struct ae_input_s *info);
uint32_t get_isp_reg(uint32_t regaddr);
int8_t set_isp_reg(struct con_reg_s *info);
int8_t clr_isp_regbase(void);
int8_t set_isp_stf_addr(void);
int8_t set_isp_start(void);
int8_t set_isp_stop(void);
int8_t isp_config_enable(void);
int8_t isp_config_disable(void);
int8_t isp_cfg_init(void);
int8_t get_isp_regs(uint32_t *info, uint32_t datalen);
int8_t set_isp_regs(uint32_t *info, uint32_t datalen);
int8_t isp_write_regs(uint32_t addr, uint32_t datalen, uint32_t *info);
int8_t isp_read_regs(uint32_t addr, uint32_t datalen, uint32_t *info);
uint32_t write_isp_reg(struct con_reg_s *info);
#ifdef CONFIG_PM
void x2_isp_regs_store(void);
void x2_isp_regs_restore(void);
#endif

#endif /* __X2_ISP_DEV_H__ */
