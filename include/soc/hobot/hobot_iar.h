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

#ifndef __HOBOT_IAR_H__
#define __HOBOT_IAR_H__

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/ion.h>
#include "../../../drivers/media/platform/hobot/common_api/vio_framemgr.h"
#include "../../../drivers/media/platform/hobot/common_api/vio_config.h"
#include "../../../drivers/media/platform/hobot/common_api/vio_group_api.h"

#define MAX_FRAME_BUF_SIZE	(1920*1080*4)
#define VIDEO_FRAME_BUF_SIZE    (800*480*2)

#define REG_IAR_OVERLAY_OPT 0x0
#define REG_IAR_ALPHA_VALUE 0x4
#define REG_IAR_KEY_COLOR_RD4	0x8
#define REG_IAR_KEY_COLOR_RD3	0xC
#define REG_IAR_KEY_COLOR_RD2	0x10
#define REG_IAR_KEY_COLOR_RD1	0x14
#define REG_IAR_CROPPED_WINDOW_RD4	0x18
#define REG_IAR_CROPPED_WINDOW_RD3	0x1C
#define REG_IAR_CROPPED_WINDOW_RD2	0x20
#define REG_IAR_CROPPED_WINDOW_RD1	0x24
#define REG_IAR_IMAGE_WIDTH_FBUF_RD4	0x38
#define REG_IAR_IMAGE_WIDTH_FBUF_RD3	0x3C
#define REG_IAR_IMAGE_WIDTH_FBUF_RD2	0x40
#define REG_IAR_IMAGE_WIDTH_FBUF_RD1	0x44
#define REG_IAR_FORMAT_ORGANIZATION 0x48
#define REG_IAR_DISPLAY_POSTION_RD4 0x4C
#define REG_IAR_DISPLAY_POSTION_RD3 0x50
#define REG_IAR_DISPLAY_POSTION_RD2 0x54
#define REG_IAR_DISPLAY_POSTION_RD1 0x58
#define REG_IAR_HWC_CFG 0x5C
#define REG_IAR_HWC_SIZE	0x60
#define REG_IAR_HWC_POS 0x64
#define REG_IAR_BG_COLOR	0x68
#define REG_IAR_SRC_SIZE_UP 0x6C
#define REG_IAR_TGT_SIZE_UP 0x70
#define REG_IAR_STEP_UP 0x74
#define REG_IAR_UP_IMAGE_POSTION	0x78
#define REG_IAR_PP_CON_1	0x7C
#define REG_IAR_PP_CON_2	0x80
#define REG_IAR_THRESHOLD_RD4_3 0x84
#define REG_IAR_THRESHOLD_RD2_1 0x88
#define REG_IAR_CAPTURE_CON 0x8C
#define REG_IAR_BURST_LEN	0x94
#define REG_IAR_UPDATE	0x98
#define REG_IAR_FBUF_ADDR_RD4	0x100
#define REG_IAR_FBUF_ADDR_RD3	0x104
#define REG_IAR_FBUF_ADDR_RD2_Y 0x108
#define REG_IAR_FBUF_ADDR_RD2_U 0x10C
#define REG_IAR_FBUF_ADDR_RD2_V 0x110
#define REG_IAR_FBUF_ADDR_RD1_Y 0x114
#define REG_IAR_FBUF_ADDR_RD1_U 0x118
#define REG_IAR_FBUF_ADDR_RD1_V 0x11C
#define REG_IAR_SHADOW_FBUF_ADDR_RD4	0x120
#define REG_IAR_SHADOW_FBUF_ADDR_RD3	0x124
#define REG_IAR_SHADOW_FBUF_ADDR_RD2_Y	0x128
#define REG_IAR_SHADOW_FBUF_ADDR_RD2_U	0x12C
#define REG_IAR_SHADOW_FBUF_ADDR_RD2_V	0x130
#define REG_IAR_SHADOW_FBUF_ADDR_RD1_Y	0x134
#define REG_IAR_SHADOW_FBUF_ADDR_RD1_U	0x138
#define REG_IAR_SHADOW_FBUF_ADDR_RD1_V	0x13C
#define REG_IAR_CURRENT_CBUF_ADDR_WR_Y	0x140
#define REG_IAR_CURRENT_CBUF_ADDR_WR_UV 0x144
#define REG_IAR_PANEL_SIZE	0x200
#define REG_IAR_REFRESH_CFG 0x204
#define REG_IAR_PARAMETER_HTIM_FIELD1	0x208
#define REG_IAR_PARAMETER_VTIM_FIELD1	0x20C
#define REG_IAR_PARAMETER_VTIM_FIELD2	0x214
#define REG_IAR_PARAMETER_VFP_CNT_FIELD12	0x218
#define REG_IAR_GAMMA_X1_X4_R	0x21C
#define REG_IAR_GAMMA_X5_X8_R	0x220
#define REG_IAR_GAMMA_X9_X12_R	0x224
#define REG_IAR_GAMMA_X13_X15_R 0x228
#define REG_IAR_GAMMA_X1_X4_G	0x22C
#define REG_IAR_GAMMA_X5_X8_G	0x230
#define REG_IAR_GAMMA_X9_X12_G	0x234
#define REG_IAR_GAMMA_X13_X15_G 0x238
#define REG_IAR_GAMMA_X1_X4_B	0x23C
#define REG_IAR_GAMMA_X5_X8_B	0x240
#define REG_IAR_GAMMA_X9_X12_B	0x244
#define REG_IAR_GAMMA_X13_X15_B 0x248
#define REG_IAR_GAMMA_Y1_Y3_R	0x24C
#define REG_IAR_GAMMA_Y4_Y7_R	0x250
#define REG_IAR_GAMMA_Y8_Y11_R	0x254
#define REG_IAR_GAMMA_Y12_Y15_R 0x258
#define REG_IAR_GAMMA_Y1_Y3_G	0x25C
#define REG_IAR_GAMMA_Y4_Y7_G	0x260
#define REG_IAR_GAMMA_Y8_Y11_G	0x264
#define REG_IAR_GAMMA_Y12_Y15_G 0x268
#define REG_IAR_GAMMA_Y1_Y3_B	0x26C
#define REG_IAR_GAMMA_Y4_Y7_B	0x270
#define REG_IAR_GAMMA_Y8_Y11_B	0x274
#define REG_IAR_GAMMA_Y12_Y15_B 0x278
#define REG_IAR_GAMMA_Y16_RGB	0x27C
#define REG_IAR_AUTO_DBI_REFRESH_CNT	0x280
#define REG_IAR_HWC_SRAM_WR 0x300
#define REG_IAR_PALETTE 0x304
#define REG_IAR_DE_SRCPNDREG	0x308
#define REG_IAR_DE_INTMASK	0x30C
#define REG_IAR_DE_SETMASK	0x310
#define REG_IAR_DE_UNMASK	0x314
#define REG_IAR_DE_REFRESH_EN	0x318
#define REG_IAR_DE_CONTROL_WO	0x31C
#define REG_IAR_DE_STATUS	0x320
#define REG_IAR_DE_REVISION 0x330
#define REG_IAR_DE_MAXOSNUM_RD	0x334
#define REG_IAR_DE_MAXOSNUM_WR	0x338
#define REG_IAR_DE_SW_RST	0x33C
#define REG_IAR_DE_OUTPUT_SEL	0x340
#define REG_IAR_DE_AR_CLASS 0x400
#define REG_IAR_DE_AR_CLASS_WEIGHT	0x404

#define REG_DISP_LCDIF_CFG 0x800
#define REG_DISP_LCDIF_PADC_RESET_N 0x804

enum {
	IAR_CHANNEL_1 = 0,
	IAR_CHANNEL_2 = 1,
	IAR_CHANNEL_3 = 2,
	IAR_CHANNEL_4 = 3,
	IAR_CHANNEL_MAX = 4,
};
enum {
	IAR_PRI_1 = 0,
	IAR_PRI_2 = 1,
	IAR_PRI_3 = 2,
	IAR_PRI_4 = 3,
	IAR_PRI_MAX = 4,
};

enum iar_Reg_cfg_e {
	IAR_ALPHA_SELECT_PRI4,
	IAR_ALPHA_SELECT_PRI3,
	IAR_ALPHA_SELECT_PRI2,
	IAR_ALPHA_SELECT_PRI1,
	IAR_EN_RD_CHANNEL4,
	IAR_EN_RD_CHANNEL3,
	IAR_EN_RD_CHANNEL2,
	IAR_EN_RD_CHANNEL1,
	IAR_LAYER_PRIORITY_4,
	IAR_LAYER_PRIORITY_3,
	IAR_LAYER_PRIORITY_2,
	IAR_LAYER_PRIORITY_1,
	IAR_EN_OVERLAY_PRI4,
	IAR_EN_OVERLAY_PRI3,
	IAR_EN_OVERLAY_PRI2,
	IAR_EN_OVERLAY_PRI1,
	IAR_OV_MODE_PRI4,
	IAR_OV_MODE_PRI3,
	IAR_OV_MODE_PRI2,
	IAR_OV_MODE_PRI1,
	IAR_EN_ALPHA_PRI4,
	IAR_EN_ALPHA_PRI3,
	IAR_EN_ALPHA_PRI2,
	IAR_EN_ALPHA_PRI1,

	IAR_ALPHA_RD4,
	IAR_ALPHA_RD3,
	IAR_ALPHA_RD2,
	IAR_ALPHA_RD1,

	IAR_WINDOW_HEIGTH,
	IAR_WINDOW_WIDTH,

	IAR_WINDOW_START_Y,
	IAR_WINDOW_START_X,

	IAR_BT601_709_SEL,
	IAR_RGB565_CONVERT_SEL,
	IAR_IMAGE_FORMAT_ORG_RD4,
	IAR_IMAGE_FORMAT_ORG_RD3,
	IAR_IMAGE_FORMAT_ORG_RD2,
	IAR_IMAGE_FORMAT_ORG_RD1,

	IAR_LAYER_TOP_Y,
	IAR_LAYER_LEFT_X,

	IAR_HWC_COLOR,
	IAR_HWC_COLOR_EN,
	IAR_HWC_EN,

	IAR_HWC_HEIGHT,
	IAR_HWC_WIDTH,

	IAR_HWC_TOP_Y,
	IAR_HWC_LEFT_X,

	IAR_BG_COLOR,

	IAR_SRC_HEIGTH,
	IAR_SRC_WIDTH,

	IAR_TGT_HEIGTH,
	IAR_TGT_WIDTH,

	IAR_STEP_Y,
	IAR_STEP_X,

	IAR_UP_IMAGE_TOP_Y,
	IAR_UP_IMAGE_LEFT_X,

	IAR_CONTRAST,
	IAR_THETA_SIGN,
	IAR_UP_SCALING_EN,
	IAR_ALGORITHM_SELECT,
	IAR_BRIGHT_EN,
	IAR_CON_EN,
	IAR_SAT_EN,
	IAR_HUE_EN,
	IAR_GAMMA_ENABLE,
	IAR_DITHERING_EN,
	IAR_DITHERING_FLAG,

	IAR_OFF_BRIGHT,
	IAR_OFF_CONTRAST,
	IAR_SATURATION,
	IAR_THETA_ABS,

	IAR_THRESHOLD_RD4,
	IAR_THRESHOLD_RD3,

	IAR_THRESHOLD_RD2,
	IAR_THRESHOLD_RD1,

	IAR_CAPTURE_INTERLACE,
	IAR_CAPTURE_MODE,
	IAR_SOURCE_SEL,
	IAR_OUTPUT_FORMAT,

	IAR_SLICE_LINES,

	IAR_BURST_LEN_WR,
	IAR_BURST_LEN_RD,

	IAR_UPDATE,

	IAR_PANEL_HEIGHT,
	IAR_PANEL_WIDTH,

	IAR_ITU_R_656_EN,
	IAR_UV_SEQUENCE,
	IAR_P3_P2_P1_P0,
	IAR_YCBCR_OUTPUT,
	IAR_PIXEL_RATE,
	IAR_ODD_POLARITY,
	IAR_DEN_POLARITY,
	IAR_VSYNC_POLARITY,
	IAR_HSYNC_POLARITY,
	IAR_INTERLACE_SEL,
	IAR_PANEL_COLOR_TYPE,
	IAR_DBI_REFRESH_MODE,

	IAR_DPI_HBP_FIELD,
	IAR_DPI_HFP_FIELD,
	IAR_DPI_HSW_FIELD,

	IAR_DPI_VBP_FIELD,
	IAR_DPI_VFP_FIELD,
	IAR_DPI_VSW_FIELD,

	IAR_DPI_HBP_FIELD2,
	IAR_DPI_HFP_FIELD2,
	IAR_DPI_HSW_FIELD2,

	IAR_DPI_VBP_FIELD2,
	IAR_DPI_VFP_FIELD2,
	IAR_DPI_VSW_FIELD2,

	IAR_PARAMETER_VFP_CNT,

	IAR_GAMMA_XY_D_R,
	IAR_GAMMA_XY_C_R,
	IAR_GAMMA_XY_B_R,
	IAR_GAMMA_XY_A_R,

	IAR_GAMMA_Y16_R,
	IAR_GAMMA_Y16_G,
	IAR_GAMMA_Y16_B,

	IAR_HWC_SRAM_ADDR,
	IAR_HWC_SRAM_D,

	IAR_PALETTE_INDEX,
	IAR_PALETTE_DATA,

	IAR_INT_WR_FIFO_FULL,
	IAR_INT_CBUF_SLICE_START,
	IAR_INT_CBUF_SLICE_END,
	IAR_INT_CBUF_FRAME_END,
	IAR_INT_FBUF_FRAME_END,
	IAR_INT_FBUF_SWITCH_RD4,
	IAR_INT_FBUF_SWITCH_RD3,
	IAR_INT_FBUF_SWITCH_RD2,
	IAR_INT_FBUF_SWITCH_RD1,
	IAR_INT_FBUF_START_RD4,
	IAR_INT_FBUF_START_RD3,
	IAR_INT_FBUF_START_RD2,
	IAR_INT_FBUF_START_RD1,
	IAR_INT_FBUF_START,
	IAR_INT_BUFFER_EMPTY_RD4,
	IAR_INT_BUFFER_EMPTY_RD3,
	IAR_INT_BUFFER_EMPTY_RD2,
	IAR_INT_BUFFER_EMPTY_RD1,
	IAR_INT_THRESHOLD_RD4,
	IAR_INT_THRESHOLD_RD3,
	IAR_INT_THRESHOLD_RD2,
	IAR_INT_THRESHOLD_RD1,
	IAR_INT_FBUF_END_RD4,
	IAR_INT_FBUF_END_RD3,
	IAR_INT_FBUF_END_RD2,
	IAR_INT_FBUF_END_RD1,
	IAR_INT_VSYNC,

	IAR_AUTO_DBI_REFRESH_EN,
	IAR_DPI_TV_START,

	IAR_FIELD_ODD_CLEAR,
	IAR_DBI_START,
	IAR_CAPTURE_EN,

	IAR_DMA_STATE_RD4,
	IAR_DMA_STATE_RD3,
	IAR_DMA_STATE_RD2,
	IAR_DMA_STATE_RD1,
	IAR_DMA_STATE_WR,
	IAR_DPI_STATE,
	IAR_DBI_STATE,
	IAR_CAPTURE_STATUS,
	IAR_REFRESH_STATUS,

	IAR_CUR_BUF,
	IAR_CUR_BUFGRP,
	IAR_STALL_OCCUR,
	IAR_ERROR_OCCUR,

	IAR_MAXOSNUM_DMA0_RD1,
	IAR_MAXOSNUM_DMA1_RD1,
	IAR_MAXOSNUM_DMA2_RD1,
	IAR_MAXOSNUM_DMA0_RD2,
	IAR_MAXOSNUM_DMA1_RD2,
	IAR_MAXOSNUM_DMA2_RD2,
	IAR_MAXOSNUM_RD3,
	IAR_MAXOSNUM_RD4,

	IAR_MAXOSNUM_DMA0_WR,
	IAR_MAXOSNUM_DMA1_WR,

	IAR_WR_RST,
	IAR_RD_RST,

	IAR_IAR_OUTPUT_EN,
	IAR_RGB_OUTPUT_EN,
	IAR_BT1120_OUTPUT_EN,
	IAR_MIPI_OUTPUT_EN,

	IAR_AR_CLASS3_WEIGHT,
	IAR_AR_CLASS2_WEIGHT,
};

enum iar_wb_state {
	IAR_WB_INIT,
	IAR_WB_REBUFS,
	IAR_WB_STOP,
	IAR_WB_START,
};

typedef struct _frame_buf_t {
	void __iomem *vaddr;
	phys_addr_t 	paddr;
} frame_buf_t;
typedef struct _buf_addr_t {
	union {
		struct {
			uint32_t Yaddr;
			uint32_t Uaddr;
			uint32_t Vaddr;
		};
		uint32_t addr;
	};
} buf_addr_t;

typedef struct _pingpong_buf_t {
	frame_buf_t framebuf[2];
	buf_addr_t pixel_addr[2];
} pingpong_buf_t;

typedef struct _channel_base_cfg_t {
	uint32_t channel;
	uint32_t enable;
	uint32_t pri;
	uint32_t width;
	uint32_t height;
	uint32_t buf_width;
	uint32_t buf_height;
	uint32_t xposition;
	uint32_t yposition;
	uint32_t format;
	//buf_addr_t  bufaddr;
	uint32_t alpha;
	uint32_t keycolor;
	uint32_t alpha_sel;
	uint32_t ov_mode;
	uint32_t alpha_en;
	uint32_t crop_width;
	uint32_t crop_height;
} channel_base_cfg_t;

struct gamma_reg_bits_s {
	unsigned int part_a:8;
	unsigned int part_b:8;
	unsigned int part_c:8;
	unsigned int part_d:8;
};

typedef union _gamma_para_t {
	unsigned int value;
	struct gamma_reg_bits_s bit;
} gama_para_t;

typedef struct _gamma_cfg_t {
	gama_para_t gamma_xr[4];
	gama_para_t gamma_xg[4];
	gama_para_t gamma_xb[4];
	gama_para_t gamma_yr[4];
	gama_para_t gamma_yg[4];
	gama_para_t gamma_yb[4];
	gama_para_t gamma_y16rgb;
} gamma_cfg_t;

typedef struct _ppcon1_cfg_t {
	uint32_t dithering_flag;
	uint32_t dithering_en;
	uint32_t gamma_en;
	uint32_t hue_en;
	uint32_t sat_en;
	uint32_t con_en;
	uint32_t bright_en;
	uint32_t theta_sign;
	uint32_t contrast;
} ppcon1_cfg_t;

typedef struct _ppcon2_cfg_t {
	uint32_t theta_abs; //ppcon2
	uint32_t saturation;
	uint32_t off_contrast;
	uint32_t off_bright;
	float gamma_value;
} ppcon2_cfg_t;

typedef struct _refresh_cfg_t {
	uint32_t dbi_refresh_mode;  //refresh mode
	uint32_t panel_corlor_type;
	uint32_t interlace_sel;
	uint32_t odd_polarity;
	uint32_t pixel_rate;
	uint32_t ycbcr_out;
	uint32_t uv_sequence;
	uint32_t itu_r656_en;

	uint32_t auto_dbi_refresh_cnt;
	uint32_t auto_dbi_refresh_en;
} refresh_cfg_t;

typedef struct _output_cfg_t {
	uint32_t bgcolor;
	uint32_t out_sel;
	uint32_t width;
	uint32_t height;
	uint32_t big_endian;
	uint32_t display_addr_type;
	uint32_t display_cam_no;
	uint32_t display_addr_type_layer1;
        uint32_t display_cam_no_layer1;
	ppcon1_cfg_t ppcon1;
	ppcon2_cfg_t ppcon2;
	refresh_cfg_t refresh_cfg;
	uint32_t panel_type;
	uint32_t rotate;
	uint32_t user_control_disp;
	uint32_t user_control_disp_layer1;
} output_cfg_t;

typedef struct _upscaling_cfg_t {
	uint32_t enable;
	uint32_t src_width;
	uint32_t src_height;
	uint32_t tgt_width;
	uint32_t tgt_height;
	uint32_t step_x;
	uint32_t step_y;
	uint32_t pos_x;
	uint32_t pos_y;
} upscaling_cfg_t;

struct iar_dev_s {
	struct platform_device *pdev;
	struct ion_client *iar_iclient;
	struct ion_handle *iar_ihandle;
	void __iomem *regaddr;
	void __iomem *mipi_dsi_regaddr;
	void __iomem *sysctrl;
	struct reset_control *rst;
	int irq;
	spinlock_t spinlock;
	spinlock_t *lock;
	frame_buf_t frambuf[IAR_CHANNEL_MAX];
	pingpong_buf_t pingpong_buf[IAR_CHANNEL_MAX];
	unsigned int channel_format[IAR_CHANNEL_MAX];
	unsigned int buf_w_h[IAR_CHANNEL_MAX][2];
	int cur_framebuf_id[IAR_CHANNEL_MAX];
	struct task_struct *iar_task;
	wait_queue_head_t wq_head;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_bt1120;
	struct pinctrl_state *pins_bt656;
	struct pinctrl_state *pins_mipi_dsi;
	struct pinctrl_state *pins_rgb;
	struct pinctrl_state *pins_rgb_gpio;
	struct pinctrl_state *pins_voltage;
	struct clk *iar_pixel_clk;
	struct clk *iar_ipi_clk;
	struct clk *sif_mclk;
	struct vio_framemgr framemgr_layer[IAR_CHANNEL_MAX];
	wait_queue_head_t output_done_wq[IAR_CHANNEL_MAX];
	unsigned long output_state[IAR_CHANNEL_MAX];

	struct vio_framemgr framemgr;
	unsigned long state;
	unsigned long capture_state;
	wait_queue_head_t done_wq;
	int wb_sel;
	int wb_format;

	struct dentry *debug_file_iar;
};
extern struct iar_dev_s *g_iar_dev;

struct disp_timing {
	uint32_t hbp;
	uint32_t hfp;
	uint32_t hs;
	uint32_t vbp;
	uint32_t vfp;
	uint32_t vs;
	uint32_t vfp_cnt;
};

struct display_video_vaddr {
	void *channel0_y_addr;
	void *channel0_c_addr;
	void *channel1_y_addr;
	void *channel1_c_addr;
};

struct display_vio_channel_pipe {
	uint8_t disp_layer_no;
	uint8_t vio_pipeline;
	uint8_t vio_channel;
};

enum {
	FORMAT_YUV422_UYVY = 0,
	FORMAT_YUV422_VYUY = 1,
	FORMAT_YUV422_YVYU = 2,
	FORMAT_YUV422_YUYV = 3,
	FORMAT_YUV422SP_UV = 4,
	FORMAT_YUV422SP_VU = 5,
	FORMAT_YUV420SP_UV = 6,
	FORMAT_YUV420SP_VU = 7,
	FORMAT_YUV422P_UV = 8,
	FORMAT_YUV422P_VU = 9,
	FORMAT_YUV420P_UV = 10,
	FORMAT_YUV420P_VU = 11,
};
enum {
	FORMAT_8BPP = 0,
	FORMAT_RGB565 = 1,
	FORMAT_RGB888 = 2,
	FORMAT_RGB888P = 3,
	FORMAT_ARGB8888 = 4,
	FORMAT_RGBA8888 = 5,
};
enum {
	OUTPUT_MIPI_DSI = 0,
	OUTPUT_BT1120 = 1,
	OUTPUT_RGB = 2,
	OUTPUT_BT656 = 3,
	OUTPUT_IPI = 4,//SIF
};
enum DISPLAY_TYPE {
	LCD_7_TYPE = 0,//800*480 rgb
	HDMI_TYPE = 1,//hdmi
	MIPI_720P = 2,//mipi-dsi 720*1280
	MIPI_720P_TOUCH = 3,//mipi-dsi 720*1280 touch
	MIPI_1080P = 4,//1080*1920
	SIF_IPI = 5,
	BT656_TYPE = 6,//704x576
	UNUSED = 7,
};
#ifdef CONFIG_HOBOT_XJ3
enum XJ3_DISPLAY_TYPE {
	DISPLAY_CHANNEL0 = 1,
	DISPLAY_CHANNEL1 = 2,
	DISPLAY_CHANNEL2 = 3,
	DISPLAY_CHANNEL3 = 4,
	DISPLAY_CHANNEL4 = 5,
	DISPLAY_CHANNEL5 = 6,
	DS0 = 7,
	DS1 = 8,
	DS2 = 9,
	DS3 = 10,
	DS4 = 11,
	DS5 = 12,
	DS6 = 13,
	DS7 = 14,
	DS8 = 15,
	DS9 = 16,
	DS10 = 17,
	DS11 = 18,
	DS12 = 19,
	DS13 = 20,
	DS14 = 21,
	DS15 = 22,
	DS16 = 23,
	DS17 = 24,
	DS18 = 25,
	DS19 = 26,
	DS20 = 27,
	DS21 = 28,
	DS22 = 29,
	DS23 = 30,
	US0 = 31,
	US1 = 32,
	US2 = 33,
	US3 = 34,
	US4 = 35,
	US5 = 36,
	GDC0 = 37,
	GDC1 = 38,
};
enum DISPLAY_PIPELINE {
	PIPELINE0 = 0,
	PIPELINE1 = 1,
	PIPELINE2 = 2,
	PIPELINE3 = 3,
};
#else
enum DISPLAY_ADDR_TYPE {
	BASE, //0
	CROP, //1
	SCALE, //2
	DS0, //3
	DS1, //4
	DS2, //5
	DS3, //6
	DS4, //7
	DS5, //8
	DS6, //9
	DS7, //10
	DS8, //11
	DS9, //12
	DS10, //13
	DS11, //14
	DS12, //15
	DS13, //16
	DS14, //17
	DS15, //18
	DS16, //19
	DS17, //20
	DS18, //21
	DS19, //22
	DS20, //23
	DS21, //24
	DS22, //25
	DS23, //26
	US0, //27
	US1, //28
	US2, //29
	US3, //30
	US4, //31
	US5, //32
	DS_2_0, //33
	DS_2_1, //34
	DS_2_2, //35
	DS_2_3, //36
	DS_2_4, //37
	DS_2_5, //38
	DS_2_6, //39
	DS_2_7, //40
	DS_2_8, //41
	DS_2_9, //42
	DS_2_10, //43
	DS_2_11, //44
	DS_2_12, //45
	DS_2_13, //46
	DS_2_14, //47
	DS_2_15, //48
	DS_2_16, //49
	DS_2_17, //50
	DS_2_18, //51
	DS_2_19, //52
	DS_2_20, //53
	DS_2_21, //54
	DS_2_22, //55
	DS_2_23, //56
	US_2_0, //57
	US_2_1, //58
	US_2_2, //59
	US_2_3, //60
	US_2_4, //61
	US_2_5, //62

};
#endif

enum PIXEL_CLK {
	PIXEL_CLK_69,
	PIXEL_CLK_32,
	PIXEL_CLK_162,
};

typedef int (*hdmi_set_config_callback)(unsigned short vmode,
                unsigned short VideoFormat, unsigned short Afs);
//extern hdmi_set_config_callback config_hdmi;
void hdmi_register_config_callback(hdmi_set_config_callback func);

extern struct disp_timing video_1920x1080;
extern struct disp_timing video_800x480;
extern struct disp_timing video_720x1280;
extern struct disp_timing video_1080x1920;
extern struct disp_timing video_720x1280_touch;
extern struct disp_timing video_704x576;
//extern uint32_t hb_disp_base_board_id;
extern uint8_t iar_display_addr_type;
extern uint8_t iar_display_cam_no;
#ifdef CONFIG_HOBOT_XJ3
#define X3_GPIO_BASE    (0xA6003000)
#define X3_GPIO0_VALUE_REG      (0xC)
#define X3_PWM0_PINMUX  (0xA6004010)
extern uint8_t iar_display_addr_type_video1;
extern uint8_t iar_display_cam_no_video1;
#endif


extern int display_type;
extern struct ion_device *hb_ion_dev;
extern uint8_t disp_copy_done;
extern uint8_t disp_user_config_done;
extern unsigned int iar_debug_level;
extern unsigned int fb_num;
extern const char *base_board_name;
extern void iar_register_get_callback(iar_get_type_callback func);
extern void iar_register_set_callback(iar_set_addr_callback func);
#define IAR_DEBUG_PRINT(format, args...)	\
	do {									\
		if(iar_debug_level)					\
			printk("IAR debug: " format, ## args);		\
	} while(0)
int ips_set_iar_clk32(unsigned int clk_index);
int32_t iar_set_panel_timing(struct fb_info *fb, int display_type);
frame_buf_t* iar_get_framebuf_addr(uint32_t channel);
int32_t iar_set_bufaddr(uint32_t channel, buf_addr_t *addr);
int32_t iar_update(void);
buf_addr_t iar_addr_convert(phys_addr_t paddr);
int32_t iar_channel_base_cfg(channel_base_cfg_t *cfg);
int32_t iar_upscaling_cfg(upscaling_cfg_t *cfg);
int32_t iar_gamma_cfg(gamma_cfg_t *cfg);
int32_t iar_output_cfg(output_cfg_t *cfg);
int32_t iar_switch_buf(uint32_t channel);
int32_t iar_start(int update);
int32_t iar_stop(void);
int32_t iar_open(void);
int32_t iar_close(void);
int32_t iar_pre_init(void);
void hobot_iar_dump(void);
frame_buf_t* hobot_iar_get_framebuf_addr(int channel);
//int32_t iar_set_video_buffer(uint32_t yaddr, uint32_t caddr, int index);
int32_t iar_set_video_buffer(uint32_t slot_id);
//int32_t ipu_set_display_addr(uint32_t yaddr, uint32_t caddr);
int32_t ipu_set_display_addr(uint32_t disp_layer,
		uint32_t yaddr, uint32_t caddr);
int32_t iar_write_framebuf_dma(uint32_t channel, phys_addr_t srcaddr,
		uint32_t size);
void *ipu_get_iar_framebuf_addr(uint32_t channel, unsigned int index);
int8_t iar_get_ipu_display_addr_single(uint32_t display_addr[][2]);
int8_t iar_get_ipu_display_addr_dual(uint32_t display_addr[][2]);
int8_t iar_get_ipu_display_addr_ddrmode(uint32_t display_addr[][2]);
int8_t iar_checkout_display_camera(uint8_t camera_no);
int user_set_fb(void);
int set_video_display_channel(uint8_t channel_no);
int set_video_display_ddr_layer(uint8_t ddr_layer_no);
int32_t disp_set_timing(unsigned int resolution);
int enable_iar_irq(void);
int disable_iar_irq(void);
int iar_rotate_video_buffer(phys_addr_t yaddr,
		phys_addr_t uaddr, phys_addr_t vaddr);
int8_t disp_set_pixel_clk(uint64_t pixel_clk);
int disp_set_ppbuf_addr(uint8_t layer_no, void *yaddr, void *caddr);
int disp_set_panel_timing(struct disp_timing *timing);
int disp_pinmux_bt1120(void);
int disp_pinmux_bt656(void);
int disp_pinmux_mipi_dsi(void);
int disp_pinmux_rgb(void);
int panel_hardware_reset(void);
void panel_enter_standby(void);
void panel_exit_standby(void);
int mipi_dsi_panel_init(uint8_t panel_no);
int set_mipi_display(uint8_t panel_no);
int32_t iar_layer_disable(int32_t layer_no);
int32_t iar_layer_enable(int32_t layer_no);
int set_screen_backlight(unsigned int backlight_level);
int screen_backlight_init(void);
int screen_backlight_deinit(void);
int get_iar_module_rst_pin(void);
int enable_sif_mclk(void);
int disable_sif_mclk(void);
int iar_pixel_clk_enable(void);
int iar_pixel_clk_disable(void);
//int iar_is_enabled(void);
int iar_wb_dqbuf(struct frame_info *frameinfo);
int iar_wb_qbuf(struct frame_info *frameinfo);
int iar_wb_reqbufs(u32 buffers);
int iar_wb_start(void);
int iar_wb_done(void);
int iar_wb_stream_on(void);
int iar_wb_stream_off(void);
void iar_wb_setcfg(int value);
int iar_wb_getcfg(void);

int iar_output_dqbuf(int layer_no, struct frame_info *frameinfo);
int iar_output_qbuf(int layer_no, struct frame_info *frameinfo);
int iar_output_buf_init(int layer_no, struct frame_info *frameinfo);
int iar_output_reqbufs(int layer_no, unsigned int buffers);
int iar_output_stream_on(int layer_no);
int iar_output_stream_off(int layer_no);

// Supported rotation.
enum RotationMode {
	kRotate0 = 0,		// No rotation.
	kRotate90 = 90,		// Rotate 90 degrees clockwise.
	kRotate180 = 180,	// Rotate 180 degrees.
	kRotate270 = 270,	// Rotate 270 degrees clockwise.
	// Deprecated.
	kRotateNone = 0,
	kRotateClockwise = 90,
	kRotateCounterClockwise = 270,
};

int NV12ToI420Rotate(const uint8_t *src_y,
		int src_stride_y,
		const uint8_t *src_uv,
		int src_stride_uv,
		uint8_t *dst_y,
		int dst_stride_y,
		uint8_t *dst_u,
		int dst_stride_u,
		uint8_t *dst_v,
		int dst_stride_v,
		int width,
		int height,
		enum RotationMode mode);

void MergeUVPlane(const uint8_t *src_u,
		int src_stride_u,
		const uint8_t *src_v,
		int src_stride_v,
		uint8_t *dst_uv,
		int dst_stride_uv,
		int width,
		int height);

#endif //__HOBOT_IAR_H__
