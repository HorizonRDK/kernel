/*
 * dwe_base_io.h
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

#ifndef __DWE_BASE_IO_H__
#define __DWE_BASE_IO_H__

#include <linux/types.h>
#include <asm/compiler.h>
#include "system_hw_io.h"

/* reg address info */
#define DWE_CHECK_SUM_TYPE          (0x000)
#define DWE_IMAGE_SIZE              (0x004)

#define DWE_PG_START                (0x008)
#define DWE_PG_BLANKING             (0x010)
#define DWE_PG_SIZE                 (0x014)

#define DWE_SW_UPDATE               (0x040)
#define DWE_SET_SETTING             (0x044)

#define DWE_UNIT_0                  (0x000)
#define DWE_UNIT_1                  (0x100)
#define DWE_UNIT_2                  (0x200)
#define DWE_UNIT_3                  (0x300)

#define DWE_DIS_SETTING_0           (0x100)
#define DWE_DIS_H_RATIO_0           (0x104)
#define DWE_DIS_V_RATIO_0           (0x108)
#define DWE_DIS_X_CROP_0            (0x10C)
#define DWE_DIS_Y_CROP_0            (0x110)
#define DWE_DIS_BUF_ST_ADDR_0       (0x114)

#define DWE_DIS_SETTING_1           (0x200)
#define DWE_DIS_H_RATIO_1           (0x204)
#define DWE_DIS_V_RATIO_1           (0x208)
#define DWE_DIS_X_CROP_1            (0x20C)
#define DWE_DIS_Y_CROP_1            (0x210)
#define DWE_DIS_BUF_ST_ADDR_1       (0x214)

#define DWE_DIS_SETTING_2           (0x300)
#define DWE_DIS_H_RATIO_2           (0x304)
#define DWE_DIS_V_RATIO_2           (0x308)
#define DWE_DIS_X_CROP_2            (0x30C)
#define DWE_DIS_Y_CROP_2            (0x310)
#define DWE_DIS_BUF_ST_ADDR_2       (0x314)

#define DWE_DIS_SETTING_3           (0x400)
#define DWE_DIS_H_RATIO_3           (0x404)
#define DWE_DIS_V_RATIO_3           (0x408)
#define DWE_DIS_X_CROP_3            (0x40C)
#define DWE_DIS_Y_CROP_3            (0x410)
#define DWE_DIS_BUF_ST_ADDR_3       (0x414)

#define DWE_INT_STATUS              (0x700)
#define DWE_INT_MASK                (0x704)
#define DWE_CUR_SET_INDEX           (0x708)
#define DWE_Y_CHECK_SUM             (0x70C)
#define DWE_C_CHECK_SUM             (0x710)
#define DWE_FRAME_INDEX_0           (0x714)
#define DWE_INT_GDC_STATUS          (0x718)
#define DWE_INT_GDC_MASK            (0x71C)
#define FRAME_INDEX_1               (0x720)

//-------------------------------------------------------------------------------------------- //
//register operation
//-------------------------------------------------------------------------------------------- //
static __inline void dwe_write_32reg(const char __iomem *regbase, uint32_t addr, uint32_t *buffer)
{
	sys_write_32reg(regbase, addr, buffer);
}

static __inline void dwe_read_32reg(const char __iomem *regbase, uint32_t addr, uint32_t *buffer)
{
	sys_read_32reg(regbase, addr, buffer);
}

static __inline void dwe_write_buffer(const char __iomem *regbase, uint32_t addr, uint32_t *buffer, uint32_t len)
{
	sys_write_buffer(regbase, addr, buffer, len);
}

static __inline void dwe_read_buffer(const char __iomem *regbase, uint32_t addr, uint32_t *buffer, uint32_t len)
{
	sys_read_buffer(regbase, addr, buffer, len);
}

//----------------------------------------------------------------------------------------- //
//change image_size  IMAGE+PG_SIZE
//----------------------------------------------------------------------------------------- //
typedef struct _dis_picsize_s {
        uint16_t pic_h;
        uint16_t pic_w;
} dis_picsize_s;

typedef union _dis_picsieze_u {
        uint32_t psize_g;
        dis_picsize_s psize_b;
} dis_picsize_u;

static __inline void set_dwe_image_size(const char __iomem *regbase, uint32_t *picsize)
{
        dwe_write_32reg(regbase, DWE_IMAGE_SIZE, picsize);
}

static __inline void get_dwe_image_size(const char __iomem *regbase, uint32_t *picsize)
{
        dwe_read_32reg(regbase, DWE_IMAGE_SIZE, picsize);
}

static __inline void set_dwe_pg_size(const char __iomem *regbase, uint32_t *picsize)
{
        dwe_write_32reg(regbase, DWE_IMAGE_SIZE, picsize);
}

static __inline void get_dwe_pg_size(const char __iomem *regbase, uint32_t *picsize)
{
        dwe_read_32reg(regbase, DWE_IMAGE_SIZE, picsize);
}

//----------------------------------------------------------------------------------------- //
//pg_blanking setting
//----------------------------------------------------------------------------------------- //
typedef struct _pg_blanking_s {
        uint16_t rg_pg_v_blanking;
        uint16_t rg_pg_h_blanking;
} pg_blanking_s;

typedef union _pg_blanking_u {
        uint32_t blank_g;
        pg_blanking_s blank_b;
} pg_blanking_u;

typedef struct _pg_param_s {
        dis_picsize_u size;
        pg_blanking_u blank;
} pg_param_s; 

static __inline void set_dwe_pg_blanking(const char __iomem *regbase, uint32_t *pg_blank)
{
        dwe_write_32reg(regbase, DWE_PG_SIZE, pg_blank);
}

//----------------------------------------------------------------------------------------- //
//change setting
//----------------------------------------------------------------------------------------- //
typedef struct _dis_setting_s {
        uint32_t rg_update_select: 1;
        uint32_t reserverved0: 7;
        uint32_t rg_redirect_3: 2;
        uint32_t rg_redirect_2: 2;
        uint32_t rg_redirect_1: 2;
        uint32_t rg_redirect_0: 2;
        uint32_t reserverved1: 14;
        uint32_t rg_max_index: 2;
} dis_setting_s;

typedef union _dis_setting_u {
        uint32_t set_g;
        dis_setting_s set_b;
} dis_setting_u;

static __inline void set_dwe_setting(const char __iomem *regbase, uint32_t *pset)
{
        dwe_write_32reg(regbase, DWE_SET_SETTING, pset);
}

//----------------------------------------------------------------------------------------- //
//model setting
//----------------------------------------------------------------------------------------- //
typedef struct _dis_path_s {
	uint32_t reserved0: 30;
	uint32_t rg_dis_path_sel: 1;
	uint32_t rg_dis_enable: 1;
} dis_path_s;

typedef struct _dis_crop_s {
        uint16_t rg_dis_end;
        uint16_t rg_dis_start;
} dis_crop_s;

typedef union _dis_crop_u {
	uint32_t crop_g;
	dis_crop_s crop_b;
} dis_crop_u;

typedef struct _dis_param_s {
	dis_picsize_u picsize;	
	dis_setting_u setting;
	uint32_t dis_h_ratio;
	uint32_t dis_v_ratio;
	dis_crop_u crop_x;
	dis_crop_u crop_y;
	//uint32_t dis_buf_st_addr;
} dis_param_s;

static __inline void set_chn_dis_param(const char __iomem *regbase, dis_param_s *ptr, uint32_t model_sw)
{
	if (model_sw < 4 ) {
		uint32_t model_sw = model_sw * 0x100;
		dwe_write_buffer((regbase + model_sw), DWE_DIS_SETTING_0, (uint32_t *)(&ptr->setting), 5);
	}
}

//----------------------------------------------------------------------------------------- //
//intterrupt setting
//----------------------------------------------------------------------------------------- //
typedef struct _dis_irqstatus_s {
	uint32_t reserved0: 28;
	uint32_t int_pg_done: 1;
	uint32_t int_dis_h_ratio_err: 1;
	uint32_t int_dis_v_ratio_err: 1;
	uint32_t int_frame_done: 1;
} dis_irqstatus_s;

typedef union _dis_irqstatus_u {
	uint32_t status_g;
	dis_irqstatus_s status_b;
} dis_irqstatus_u;

typedef struct _dis_irqmask_s {
	uint32_t reserved0: 28;
	uint32_t int_pg_done_en: 1;
	uint32_t int_dis_h_ratio_err_en: 1;
	uint32_t int_dis_v_ratio_err_en: 1;
	uint32_t int_frame_done_en: 1;
} dis_irqmask_s;

typedef union _dis_irqmask_u {
	uint32_t mask_g;
	dis_irqmask_s mask_b;
} dis_irqmask_u;

static __inline void set_dwe_int_mask(const char __iomem *regbase, uint32_t *mask)
{
	dwe_write_32reg(regbase, DWE_INT_MASK, mask);
}

static __inline void get_dwe_int_status(const char __iomem *regbase, uint32_t *status)
{
	dwe_read_32reg(regbase, DWE_INT_STATUS, status);
}

//----------------------------------------------------------------------------------------- //
//dwe_soft setting
//----------------------------------------------------------------------------------------- //
static __inline void set_dwe_pg_start(const char __iomem *regbase, uint32_t *pg_start_sw)
{
	dwe_write_32reg(regbase, DWE_PG_START, pg_start_sw);
}

static __inline void set_dwe_sw_update(const char __iomem *regbase, uint32_t *sw_update_sw)
{
	dwe_write_32reg(regbase, DWE_SW_UPDATE, sw_update_sw);
}

static __inline void get_dwe_cur_index(const char __iomem *regbase, uint32_t *dwe_cur_index)
{
        dwe_read_32reg(regbase, DWE_CUR_SET_INDEX, dwe_cur_index);
}

static __inline void get_dwe_frame_index(const char __iomem *regbase, uint32_t *dwe_frame_index)
{
        dwe_read_32reg(regbase, DWE_FRAME_INDEX_0, dwe_frame_index);
}
//----------------------------------------------------------------------------------------- //
//check sum
//----------------------------------------------------------------------------------------- //
enum check_sw_s{
        y_check_sum = 0x0,
        c_check_sum = 0x4
};

static __inline void get_dwe_check_sum(const char __iomem *regbase, uint32_t *dwe_check, enum check_sw_s check_sw)
{
        dwe_read_32reg((regbase + check_sw), DWE_CUR_SET_INDEX, dwe_check);
}

//----------------------------------------------------------------------------------------- //
//gdc_status
//----------------------------------------------------------------------------------------- //
static __inline void set_gdc_mask(const char __iomem *regbase, uint32_t *gdc_mask)
{
        dwe_write_32reg(regbase, DWE_INT_GDC_MASK, gdc_mask);
}

static __inline void get_gdc_status(const char __iomem *regbase, uint32_t *gdc_status)
{
        dwe_read_32reg(regbase, DWE_INT_GDC_STATUS, gdc_status);
}

#endif /* __DWE_BASE_IO_H__ */
