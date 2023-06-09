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

#ifndef __HOBOT_IPS_HW_REG_H__
#define __HOBOT_IPS_HW_REG_H__

#include "vio_hw_common_api.h"

enum ips_reg {
	IPS_GENERAL_REG,
	IPS_INT_ENABLE,
	IPS_INT_STATUS,
	IPS_CLK_CTRL,
	IPS_SRAM_RTSEL,
	IPS_SRAM_WRSEL,
	IPS_SRAM_MUX,
	IPS_DEBUG_FLAG,
	IPS_DUMMY_REG,
	MOT_DET_ROT_LT,
	MOT_DET_ROT_WH,
	MOT_DET_PARA1,
	MOT_DET_PARA2,
	MOT_DET_IMG,
	AXI_BUS_CTRL,
	AXI_BUS_STATUS,
	AXI_BUS_CLR,
	ISP0_FRAME_ID,
	ISP0_FR_DMA_MON,
	ISP0_TEMPER_MSB_MON,
	ISP0_TEMPER_LSB_MON,
	ISP0_INT_EN,
	ISP0_VCKE_CTRL,
	ISP0_VCKE_TH0,
	ISP0_VCKE_TH1,
	IPS_BW_MON_EN,
	IPS_BW_CNT_CH0,
	IPS_BW_CNT_CH1,
	IPS_BW_CNT_CH2,
	IPS_BW_CNT_CH3,
	IPS_BW_CNT_CH4,
	IPS_BW_CNT_CH5,
	IPS_BW_CNT_CH6,
	IPS_BW_CNT_CH7,
	IPS_BW_CNT_CH8,
	IPS_BW_CNT_CH9,
	IPS_BW_CNT_CH10,
	IPS_BW_CNT_CH11,
	IPS_BW_CNT_CH12,
	IPS_BW_CNT_CH13,
	IPS_BW_CNT_CH14,
	IPS_BW_CNT_CH15,
	NUM_OF_IPS_REG,
};

static struct vio_reg_def ips_regs[NUM_OF_IPS_REG]={
	{"IPS_GENERAL_REG",      0x0000, RW},
	{"IPS_INT_ENABLE",       0x0004, RW},
	{"IPS_INT_STATUS",       0x0008, W1C},
	{"IPS_CLK_CTRL",         0x000C, RW},
	{"IPS_SRAM_RTSEL",       0x0010, RW},
	{"IPS_SRAM_WRSEL",       0x0014, RW},
	{"IPS_SRAM_MUX",         0x0018, RW},
	{"IPS_DEBUG_FLAG",       0x001C, RO},
	{"IPS_DUMMY_REG",        0x0020, RW},
	{"MOT_DET_ROT_LT",       0x0024, RW},
	{"MOT_DET_ROT_WH",       0x0028, RW},
	{"MOT_DET_PARA1",        0x002C, RW},
	{"MOT_DET_PARA2",        0x0030, RW},
	{"MOT_DET_IMG",          0x0034, RW},
	{"AXI_BUS_CTRL",         0x0038, RW},
	{"AXI_BUS_STATUS",       0x003C, RO},
	{"AXI_BUS_CLR",          0x0040, RW},
	{"ISP0_FRAME_ID",        0x0044, RO},
	{"ISP0_FR_DMA_MON",      0x0048, RO},
	{"ISP0_TEMPER_MSB_MON",  0x004C, RO},
	{"ISP0_TEMPER_LSB_MON",  0x0050, RO},
	{"ISP0_INT_EN",          0x0054, RW},
	{"ISP0_VCKE_CTRL",		 0X0058, RW},
	{"ISP0_VCKE_TH0",        0x005C, RW},
	{"ISP0_VCKE_TH1",        0x0060, RW},
	{"IPS_BW_MON_EN",        0x0100, RW},
	{"IPS_BW_CNT_CH0",       0x0104, RO},
	{"IPS_BW_CNT_CH1",       0x0108, RO},
	{"IPS_BW_CNT_CH2",       0x010C, RO},
	{"IPS_BW_CNT_CH3",       0x0110, RO},
	{"IPS_BW_CNT_CH4",       0x0114, RO},
	{"IPS_BW_CNT_CH5",       0x0118, RO},
	{"IPS_BW_CNT_CH6",       0x011C, RO},
	{"IPS_BW_CNT_CH7",       0x0120, RO},
	{"IPS_BW_CNT_CH8",       0x0124, RO},
	{"IPS_BW_CNT_CH9",       0x0128, RO},
	{"IPS_BW_CNT_CH10",      0x012C, RO},
	{"IPS_BW_CNT_CH11",      0x0130, RO},
	{"IPS_BW_CNT_CH12",      0x0134, RO},
	{"IPS_BW_CNT_CH13",      0x0138, RO},
	{"IPS_BW_CNT_CH14",      0x013C, RO},
	{"IPS_BW_CNT_CH15",      0x0140, RO},
};

enum ips_field{
	IPS_F_GEN_1,
	IPS_F_GEN_0,
	IPS_F_RSTN_SIF,
	IPS_F_RSTN_DWE1,
	IPS_F_RSTN_DWE0,
	IPS_F_RSTN_ISP0,
	IPS_F_RSTN_IRAM,
	IPS_F_RSTN_IPU0,
	IPS_F_RSTN_IPU_PYM,
	IPS_F_RSTN_PYM,
	IPS_F_MD_INT_EN,
	IPS_F_IRAM_INT_EN,
	IPS_F_AXI1_INT_EN,
	IPS_F_AXI0_INT_EN,
	IPS_F_IRAM_CG_EN,
	IPS_F_MD_CG_EN,
	IPS_F_PYM_ISP_CG_EN,
	IPS_F_PYM_DDR_CG_EN,
	IPS_F_PYM_US_CG_EN,
	IPS_F_PYM_UV_CG_EN,
	IPS_F_IPU1_CG_EN,
	IPS_F_IPU0_CG_EN,
	IPS_F_LDC1_CG_EN,
	IPS_F_LDC0_CG_EN,
	IPS_F_T2L1_CG_EN,
	IPS_F_T2L0_CG_EN,
	IPS_F_GDC1_CG_EN,
	IPS_F_GDC0_CG_EN,
	IPS_F_DWE1_CG_EN,
	IPS_F_DWE0_CG_EN,
	IPS_F_ISP1_CG_EN,
	IPS_F_ISP0_CG_EN,
	IPS_F_SIF_CG_EN,
	IPS_F_KP_011,
	IPS_F_WCT_01,
	IPS_F_RCT_01,
	IPS_F_MTSEL_00,
	IPS_F_RTSEL_11,
	IPS_F_RTSEL_10,
	IPS_F_RTSEL_01,
	IPS_F_RTSEL_00,
	IPS_F_WTSEL_11,
	IPS_F_WTSEL_10,
	IPS_F_WTSEL_01,
	IPS_F_WTSEL_00,
	IPS_F_AXI1_STATUS,
	IPS_F_AXI0_STATUS,
	IPS_F_DUMMY_REG1,
	IPS_F_DUMMY_REG0,
	IPS_F_MOT_DET_ROI_TOP,
	IPS_F_MOT_DET_ROI_LEFT,
	IPS_F_MOT_DET_ROI_HEIGHT,
	IPS_F_MOT_DET_ROI_WIDTH,
	IPS_F_MOT_DET_DIFF_THRESH,
	IPS_F_MOT_DET_THRESH,
	IPS_F_MOT_DET_STEP,
	IPS_F_MOT_DET_IMG_FMT,
	IPS_F_MOT_DET_PIX_WIDTH,
	IPS_F_MOT_DET_DATA_SEL,
	IPS_F_MOT_DET_EN,
	IPS_F_MOT_DET_REFRESH,
	IPS_F_MOT_DET_DEC_PREC,
	IPS_F_MOT_DET_WGT_DECAY,
	IPS_F_MOT_DET_IMG_WIDTH,
	IPS_F_MOT_DET_IMG_HEIGHT,
	IPS_F_SLV_SWITCH,
	IPS_F_FORCE_IDLE,
	IPS_F_IDLE,
	IPS_F_SWITCH_ERR,
	IPS_F_ISP_UV_LINETICK,
	IPS_F_ISP_UV_FIFO_USED,
	IPS_F_ISP_Y_LINETICK,
	IPS_F_ISP_Y_FIFO_USED,
	IPS_F_ISP0_TEMPER_MSB_RFIFO,
	IPS_F_ISP0_TEMPER_MSB_WFIFO,
	IPS_F_ISP0_TEMPER_LSB_RFIFO,
	IPS_F_ISP0_TEMPER_LSB_WFIFO,
	IPS_F_ISP0_UV_DMA_TH,
	IPS_F_ISP0_Y_DMA_TH,
	IPS_F_ISP0_TEMPER_MSB_DMA_TH,
	IPS_F_ISP0_TEMPER_LSB_DMA_TH,
	IPS_F_BW_WCNT_EN,
	IPS_F_BW_RCNT_EN,
	NUM_OF_IPS_FIELD,
};

static struct vio_field_def ips_fields[NUM_OF_IPS_FIELD] = {
	{IPS_GENERAL_REG,	IPS_F_GEN_1,         16, 16, 0},
	{IPS_GENERAL_REG,	IPS_F_GEN_0,          8,  8, 0},
	{IPS_GENERAL_REG,	IPS_F_RSTN_SIF,       7,  1, 1},
	{IPS_GENERAL_REG,	IPS_F_RSTN_DWE1,      6,  1, 1},
	{IPS_GENERAL_REG,	IPS_F_RSTN_DWE0,      5,  1, 1},
	{IPS_GENERAL_REG,	IPS_F_RSTN_ISP0,      4,  1, 1},
	{IPS_GENERAL_REG,	IPS_F_RSTN_IRAM,      3,  1, 1},
	{IPS_GENERAL_REG,	IPS_F_RSTN_IPU0,      2,  1, 1},
	{IPS_GENERAL_REG,	IPS_F_RSTN_IPU_PYM,   1,  1, 1},
	{IPS_GENERAL_REG,	IPS_F_RSTN_PYM,       0,  1, 1},
	{IPS_INT_ENABLE,	IPS_F_MD_INT_EN,      3,  1, 0},
	{IPS_INT_ENABLE,	IPS_F_IRAM_INT_EN,    2,  1, 1},
	{IPS_INT_ENABLE,	IPS_F_AXI1_INT_EN,    1,  1, 1},
	{IPS_INT_ENABLE,	IPS_F_AXI0_INT_EN,    1,  1, 1},
	{IPS_CLK_CTRL,	IPS_F_IRAM_CG_EN,        18,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_MD_CG_EN,          17,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_PYM_ISP_CG_EN,     16,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_PYM_DDR_CG_EN,     15,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_PYM_US_CG_EN,      14,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_PYM_UV_CG_EN,      13,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_IPU1_CG_EN,        12,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_IPU0_CG_EN,        11,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_LDC1_CG_EN,        10,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_LDC0_CG_EN,         9,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_T2L1_CG_EN,         8,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_T2L0_CG_EN,         7,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_GDC1_CG_EN,         6,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_GDC0_CG_EN,         5,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_DWE1_CG_EN,         4,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_DWE0_CG_EN,         3,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_ISP1_CG_EN,         2,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_ISP0_CG_EN,         1,  1, 0},
	{IPS_CLK_CTRL,	IPS_F_SIF_CG_EN,          0,  1, 0},
	{IPS_SRAM_RTSEL,	IPS_F_KP_011,        16,  3, 3},
	{IPS_SRAM_RTSEL,	IPS_F_WCT_01,		 12,  2, 1},
	{IPS_SRAM_RTSEL,	IPS_F_RCT_01,        10,  2, 1},
	{IPS_SRAM_RTSEL,	IPS_F_MTSEL_00,       8,  2, 0},
	{IPS_SRAM_RTSEL,	IPS_F_RTSEL_11,       6,  2, 3},
	{IPS_SRAM_RTSEL,	IPS_F_RTSEL_10,       4,  2, 2},
	{IPS_SRAM_RTSEL,	IPS_F_RTSEL_01,       2,  2, 1},
	{IPS_SRAM_RTSEL,	IPS_F_RTSEL_00,       0,  2, 0},
	{IPS_SRAM_WRSEL,	IPS_F_WTSEL_11,       6,  2, 3},
	{IPS_SRAM_WRSEL,	IPS_F_WTSEL_10,       4,  2, 2},
	{IPS_SRAM_WRSEL,	IPS_F_WTSEL_01,       2,  2, 1},
	{IPS_SRAM_WRSEL,	IPS_F_WTSEL_00,       0,  2, 0},
	{IPS_DEBUG_FLAG,	IPS_F_AXI1_STATUS,    1,  1, 0},
	{IPS_DEBUG_FLAG,	IPS_F_AXI0_STATUS,    0,  1, 0},
	{IPS_DUMMY_REG,		IPS_F_DUMMY_REG1,    16, 16, 0},
	{IPS_DUMMY_REG,		IPS_F_DUMMY_REG0,     0, 16, 0},
	{MOT_DET_ROT_LT,	IPS_F_MOT_DET_ROI_TOP,      16, 13, 0},
	{MOT_DET_ROT_LT,	IPS_F_MOT_DET_ROI_LEFT,      0, 13, 0},
	{MOT_DET_ROT_WH,	IPS_F_MOT_DET_ROI_HEIGHT,   16, 13, 0},
	{MOT_DET_ROT_WH,	IPS_F_MOT_DET_ROI_WIDTH,     0, 13, 0},
	{MOT_DET_PARA1,		IPS_F_MOT_DET_DIFF_THRESH,  16, 12, 0},
	{MOT_DET_PARA1,		IPS_F_MOT_DET_THRESH,        8,  8, 0},
	{MOT_DET_PARA1,		IPS_F_MOT_DET_STEP,          0,  8, 0},
	{MOT_DET_PARA2,		IPS_F_MOT_DET_IMG_FMT,      16,  4, 0},
	{MOT_DET_PARA2,		IPS_F_MOT_DET_PIX_WIDTH,    13,  3, 0},
	{MOT_DET_PARA2,		IPS_F_MOT_DET_DATA_SEL,     12,  1, 0},
	{MOT_DET_PARA2,		IPS_F_MOT_DET_EN,           11,  1, 0},
	{MOT_DET_PARA2,		IPS_F_MOT_DET_REFRESH,      10,  1, 0},
	{MOT_DET_PARA2,		IPS_F_MOT_DET_DEC_PREC,      8,  2, 0},
	{MOT_DET_PARA2,		IPS_F_MOT_DET_WGT_DECAY,     0,  8, 0},
	{MOT_DET_IMG,	IPS_F_MOT_DET_IMG_WIDTH,        16, 13, 0},
	{MOT_DET_IMG,	IPS_F_MOT_DET_IMG_HEIGHT,        0, 13, 0},
	{AXI_BUS_CTRL,	IPS_F_SLV_SWITCH,               16, 16, 0},
	{AXI_BUS_CTRL,	IPS_F_FORCE_IDLE,                0, 16, 0},
	{AXI_BUS_STATUS,	IPS_F_IDLE,                 16, 16, 0},
	{AXI_BUS_STATUS,	IPS_F_SWITCH_ERR,            0, 16, 0},
	{ISP0_FR_DMA_MON,	IPS_F_ISP_UV_LINETICK,      25,  1, 0},
	{ISP0_FR_DMA_MON,	IPS_F_ISP_UV_FIFO_USED,     16,  9, 0},
	{ISP0_FR_DMA_MON,	IPS_F_ISP_Y_LINETICK,       10,  1, 0},
	{ISP0_FR_DMA_MON,	IPS_F_ISP_Y_FIFO_USED,       0, 10, 0},
	{ISP0_TEMPER_MSB_MON,	IPS_F_ISP0_TEMPER_MSB_RFIFO,   16, 10, 0},
	{ISP0_TEMPER_MSB_MON,	IPS_F_ISP0_TEMPER_MSB_WFIFO,    0, 10, 0},
	{ISP0_TEMPER_LSB_MON,	IPS_F_ISP0_TEMPER_LSB_RFIFO,   16, 10, 0},
	{ISP0_TEMPER_LSB_MON,	IPS_F_ISP0_TEMPER_LSB_WFIFO,    0, 10, 0},
	{ISP0_VCKE_TH0,		IPS_F_ISP0_UV_DMA_TH,          10, 10, 0x1ff},
	{ISP0_VCKE_TH0,		IPS_F_ISP0_Y_DMA_TH,            0, 10, 0x1ff},
	{ISP0_VCKE_TH1,		IPS_F_ISP0_TEMPER_MSB_DMA_TH,  10, 10, 0x1ff},
	{ISP0_VCKE_TH1,		IPS_F_ISP0_TEMPER_LSB_DMA_TH,   0, 10, 0x1ff},
	{IPS_BW_MON_EN,		IPS_F_BW_WCNT_EN,     16, 16, 0xdf16},
	{IPS_BW_MON_EN,		IPS_F_BW_RCNT_EN,      0, 16, 0xdf16},
};
#endif
