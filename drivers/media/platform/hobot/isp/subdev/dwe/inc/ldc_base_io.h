/*
 * ldc_base_io.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LDC_BASE_IO_H__
#define __LDC_BASE_IO_H__

#include <linux/types.h>
#include <asm/compiler.h>
#include "system_hw_io.h"

/* reg address info */
#define LDC_SOFT_RESET              (0x004)
#define LDC_BYPASS                  (0x008)
#define LDC_SW_UPDATE               (0x040)
#define LDC_SET_SETTING             (0x044)

#define LDC_UNIT_0                  (0x000)
#define LDC_UNIT_1                  (0x100)
#define LDC_UNIT_2                  (0x200)
#define LDC_UNIT_3                  (0x300)

#define LDC_PATH_SEL_x              (0x100)
#define LDC_Y_START_ADDR_x          (0x104)
#define LDC_C_START_ADDR_x          (0x108)
#define LDC_IMAGE_SIZE_x            (0x140)
#define LDC_LINE_BUF_x              (0x144)
#define LDC_X_PARAMETER_x           (0x148)
#define LDC_Y_PARAMETER_x           (0x14C)
#define LDC_OFFSET_AND_SHIFT_x      (0x150)
#define LDC_IN_WOI_X_x              (0x180)
#define LDC_IN_WOT_Y_x              (0x184)

#define LDC_PATH_SEL_0              (0x100)
#define LDC_Y_START_ADDR_0          (0x104)
#define LDC_C_START_ADDR_0          (0x108)
#define LDC_IMAGE_SIZE_0            (0x140)
#define LDC_LINE_BUF_0              (0x144)
#define LDC_X_PARAMETER_0           (0x148)
#define LDC_Y_PARAMETER_0           (0x14C)
#define LDC_OFFSET_AND_SHIFT_0      (0x150)
#define LDC_IN_WOI_X_0              (0x180)
#define LDC_IN_WOT_Y_0              (0x184)

#define LDC_PATH_SEL_1              (0x200)
#define LDC_Y_START_ADDR_1          (0x204)
#define LDC_C_START_ADDR_1          (0x208)
#define LDC_IMAGE_SIZE_1            (0x240)
#define LDC_LINE_BUF_1              (0x244)
#define LDC_X_PARAMETER_1           (0x248)
#define LDC_Y_PARAMETER_1           (0x24C)
#define LDC_OFFSET_AND_SHIFT_1      (0x240)
#define LDC_IN_WOI_X_1              (0x280)
#define LDC_IN_WOT_Y_1              (0x284)

#define LDC_PATH_SEL_2              (0x300)
#define LDC_Y_START_ADDR_2          (0x304)
#define LDC_C_START_ADDR_2          (0x308)
#define LDC_IMAGE_SIZE_2            (0x340)
#define LDC_LINE_BUF_2              (0x344)
#define LDC_X_PARAMETER_2           (0x348)
#define LDC_Y_PARAMETER_2           (0x34C)
#define LDC_OFFSET_AND_SHIFT_2      (0x350)
#define LDC_IN_WOI_X_2              (0x380)
#define LDC_IN_WOT_Y_2              (0x384)

#define LDC_PATH_SEL_3              (0x400)
#define LDC_Y_START_ADDR_3          (0x404)
#define LDC_C_START_ADDR_3          (0x408)
#define LDC_IMAGE_SIZE_3            (0x440)
#define LDC_LINE_BUF_3              (0x444)
#define LDC_X_PARAMETER_3           (0x448)
#define LDC_Y_PARAMETER_3           (0x44C)
#define LDC_OFFSET_AND_SHIFT_3      (0x450)
#define LDC_IN_WOI_X_3              (0x480)
#define LDC_IN_WOT_Y_3              (0x484)

#define LDC_INT_STATUS              (0x700)
#define LDC_INT_MASK                (0x704)
#define LDC_CUR_SET_INDEX           (0x708)

//read only
#define ISP_Y_IN_CNT                (0x800)
#define ISP_UV_IN_CNT               (0x804)
#define COOR_Y_IN_CNT               (0x808)
#define COOR_UV_IN_CNT              (0x80C)
#define LDC_Y_OUT_CNT               (0x810)
#define LDC_LIN_BUF_CNT_Y0          (0x814)
#define LDC_LIN_BUF_CNT_Y1          (0x818)
#define LDC_LIN_BUF_CNT_UV0         (0x81C)
#define LDC_LIN_BUF_CNT_UV1         (0x820)

//----------------------------------------------------------------//
//register operation
//----------------------------------------------------------------//
static __inline void ldc_write_32reg(const char __iomem *regbase,
	uint32_t addr, uint32_t *buffer)
{
	sys_write_32reg(regbase, addr, buffer);
}

static __inline void ldc_read_32reg(const char __iomem *regbase,
	uint32_t addr, uint32_t *buffer)
{
	sys_read_32reg(regbase, addr, buffer);
}

static __inline void ldc_write_buffer(const char __iomem *regbase,
	uint32_t addr, uint32_t *buffer, uint32_t len)
{
	sys_write_buffer(regbase, addr, buffer, len);
}

static __inline void ldc_read_buffer(const char __iomem *regbase,
	uint32_t addr, uint32_t *buffer, uint32_t len)
{
	sys_read_buffer(regbase, addr, buffer, len);
}

//----------------------------------------------------------------------------------------- //
//change setting
//----------------------------------------------------------------------------------------- //
typedef struct _ldc_setting_s {
	uint32_t rg_max_index: 2;
	uint32_t reserverved1: 14;
	uint32_t rg_redirect_0: 2;
	uint32_t rg_redirect_1: 2;
	uint32_t rg_redirect_2: 2;
	uint32_t rg_redirect_3: 2;
	uint32_t reserverved0: 7;
	uint32_t rg_update_select: 1;
} ldc_setting_s; 

typedef union _ldc_setting_u {
	uint32_t set_g;
	ldc_setting_s set_b;
} ldc_setting_u;

static __inline void set_ldc_setting(const char __iomem *regbase,
	uint32_t *pset)
{
	ldc_write_32reg(regbase, LDC_SET_SETTING, pset);
}

static __inline void get_ldc_setting(const char __iomem *regbase,
	uint32_t *pset)
{
	ldc_read_32reg(regbase, LDC_SET_SETTING, pset);
}

//---------------------------------------------------------------//
//model setting
//---------------------------------------------------------------//
typedef struct _ldc_path_sel_s {
	uint32_t rg_y_only: 1;
	uint32_t rg_uv_mode: 1;
	uint32_t rg_uv_interpo: 1;
	uint32_t reserved1: 5;
	uint32_t rg_h_blank_cyc: 8;
	uint32_t reserved0: 16;
} ldc_path_sel_s;

typedef union _ldc_path_sel_u {
	uint32_t path_g;
	ldc_path_sel_s path_b;
} ldc_path_sel_u;

typedef struct _ldc_picsize_s {
	uint16_t pic_w;
	uint16_t pic_h;
} ldc_picsize_s;

typedef union _ldc_picsize_u {
	uint32_t size_g;
	ldc_picsize_s size_b;
} ldc_picsize_u;

typedef struct _ldc_algoparam_s {
	uint16_t rg_algo_param_b;
	uint16_t rg_algo_param_a;
} ldc_algoparam_s;

typedef union _ldc_algoparam_u {
	uint32_t param_g;
	ldc_algoparam_s param_b;
} ldc_algoparam_u;

typedef struct _ldc_off_shift_s {
	uint32_t rg_center_xoff: 8;
	uint32_t rg_center_yoff: 8;
	uint32_t reserved0: 16;
} ldc_off_shift_s;

typedef union _ldc_off_shift_u {
	uint32_t off_g;
	ldc_off_shift_s off_b;
} ldc_off_shift_u;

typedef struct _ldc_woi_s {
	uint32_t rg_start: 12;
	uint32_t reserved1: 4;
	uint32_t rg_length: 12;
	uint32_t reserved0: 4;
} ldc_woi_s;

typedef union _ldc_woi_u {
	uint32_t woi_g;
	ldc_woi_s woi_b;
} ldc_woi_u;

typedef struct _ldc_param_s {
	uint32_t ldc_enable;
	ldc_path_sel_u path_sel;
	uint32_t y_start_addr;
	uint32_t c_start_addr;
	ldc_picsize_u picsize;
	uint32_t line_buf;
	ldc_algoparam_u x_param;
	ldc_algoparam_u y_param;
	ldc_off_shift_u off_shift;
	ldc_woi_u woi_x;
	ldc_woi_u woi_y;
} ldc_param_s;

static __inline void set_chn_ldc_param(const char __iomem *regbase,
	ldc_param_s *param, uint32_t chn)
{
	uint32_t model_sw = chn * 0x100;

	if (chn < 4) {
		ldc_write_buffer(regbase, (model_sw + LDC_PATH_SEL_0),
			(uint32_t *)param + 1, 3);
		ldc_write_buffer(regbase, (model_sw + LDC_IMAGE_SIZE_0),
			(uint32_t *)param + 4, 5);
		ldc_write_buffer(regbase, (model_sw + LDC_IN_WOI_X_0),
			(uint32_t *)param + 9, 3);
	}
}

//--------------------------------------------------------------------//
//intterrupt setting
//--------------------------------------------------------------------//
typedef struct _ldc_irqmask_s {
	uint32_t input_frame_done_en: 1;
	uint32_t output_frame_done_en: 1;
	uint32_t frame_overwrite_en: 1;
	uint32_t line_overwrite_en: 1;
	uint32_t isp_in_overwrite_en: 1;
	uint32_t overflow_en: 1;
	uint32_t line_buf_woi_errror_en: 1;
	uint32_t reserved0: 25;
} ldc_irqmask_s;

typedef union _ldc_irqmask_u {
	uint32_t mask_g;
	ldc_irqmask_s mask_b;
} ldc_irqmask_u;

typedef struct _ldc_irqstatus_s {
	uint32_t input_frame_done: 1;
	uint32_t output_frame_done: 1;
	uint32_t frame_overwrite: 1;
	uint32_t line_overwrite: 1;
	uint32_t isp_in_overwrite: 1;
	uint32_t overflow: 1;
	uint32_t frame_start: 1;
	uint32_t line_buf_woi_error: 1;
	uint32_t reserved0: 25;
} ldc_irqstatus_s;

typedef union _ldc_irqstatus_u {
	uint32_t status_g;
	ldc_irqstatus_s status_b;
} ldc_irqstatus_u;

static __inline void set_ldc_int_mask(const char __iomem *regbase,
	uint32_t *int_mask)
{
	ldc_write_32reg(regbase, LDC_INT_MASK, int_mask);
}

static __inline void get_ldc_int_status(const char __iomem *regbase,
	uint32_t *int_status)
{
	ldc_read_32reg(regbase, LDC_INT_STATUS, int_status);
}

static __inline void set_ldc_int_status(const char __iomem *regbase,
	uint32_t *int_status)
{
	ldc_write_32reg(regbase, LDC_INT_STATUS, int_status);
}
//--------------------------------------------------------------------//
//ldc bypass
//--------------------------------------------------------------------//
static __inline void set_ldc_bypass(const char __iomem *regbase,
	uint32_t *bypass)
{
	ldc_write_32reg(regbase, LDC_BYPASS, bypass);
}

//--------------------------------------------------------------------//
//ldc_soft setting
//--------------------------------------------------------------------//
static __inline void set_ldc_soft_reset(const char __iomem *regbase,
	uint32_t *soft_reset_sw)
{
	ldc_write_32reg(regbase, LDC_SOFT_RESET, soft_reset_sw);
}

static __inline void set_ldc_sw_update(const char __iomem *regbase,
	uint32_t *sw_update_sw)
{
	ldc_write_32reg(regbase, LDC_SW_UPDATE, sw_update_sw);
}

static __inline void get_ldc_cur_index(const char __iomem *regbase,
	uint32_t *ldc_cur_index)
{
	ldc_read_32reg(regbase, LDC_CUR_SET_INDEX, ldc_cur_index);
}

static __inline void ldc_debug_info(const char __iomem *regbase, uint32_t addr,
	uint32_t *info)
{
	ldc_read_32reg(regbase, addr, info);
}
#endif /* __LDC_BASE_IO_H__ */
