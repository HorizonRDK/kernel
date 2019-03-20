/***************************************************************************
 *						COPYRIGHT NOTICE
 *			   Copyright 2018 Horizon Robotics, Inc.
 *					   All rights reserved.
 ***************************************************************************/

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/reset.h>
#include <linux/fb.h>
#include "x2/x2_ips.h"
#include "x2_iar.h"

unsigned int iar_debug_level = 0;
module_param(iar_debug_level, uint, 0644);

#define IAR_ENABLE 1
#define IAR_DISABLE 0

const unsigned int g_iarReg_cfg_table[][3] = {
	/*reg mask	reg offset*/
	{0x1, 0x1f},    /*ALPHA_SELECT_PRI4*/
	{0x1, 0x1e},    /*ALPHA_SELECT_PRI3*/
	{0x1, 0x1d},    /*ALPHA_SELECT_PRI2*/
	{0x1, 0x1c},    /*ALPHA_SELECT_PRI1*/
	{0x1, 0x1b},    /*EN_RD_CHANNEL4*/
	{0x1, 0x1a},    /*EN_RD_CHANNEL3*/
	{0x1, 0x19},    /*EN_RD_CHANNEL2*/
	{0x1, 0x18},    /*EN_RD_CHANNEL1*/
	{0x3, 0x16},    /*LAYER_PRIORITY_4*/
	{0x3, 0x14},    /*LAYER_PRIORITY_3*/
	{0x3, 0x12},    /*LAYER_PRIORITY_2*/
	{0x3, 0x10},    /*LAYER_PRIORITY_1*/
	{0x1, 0xf},     /*EN_OVERLAY_PRI4*/
	{0x1, 0xe},     /*EN_OVERLAY_PRI3*/
	{0x1, 0xd},     /*EN_OVERLAY_PRI2*/
	{0x1, 0xc},     /*EN_OVERLAY_PRI1*/
	{0x3, 0xa},     /*OV_MODE_PRI4*/
	{0x3, 0x8},     /*OV_MODE_PRI3*/
	{0x3, 0x6},     /*OV_MODE_PRI2*/
	{0x3, 0x4},     /*OV_MODE_PRI1*/
	{0x1, 0x3},     /*EN_ALPHA_PRI4*/
	{0x1, 0x2},     /*EN_ALPHA_PRI3*/
	{0x1, 0x1},     /*EN_ALPHA_PRI2*/
	{0x1, 0x0},     /*EN_ALPHA_PRI1*/
	/*X2_IAR_ALPHA_VALUE*/
	{0xff, 0x18},   /*ALPHA_RD4*/
	{0xff, 0x10},   /*ALPHA_RD3*/
	{0xff, 0x8},    /*ALPHA_RD2*/
	{0xff, 0x0},    /*ALPHA_RD1*/
	/*X2_IAR_CROPPED_WINDOW_RD_x*/
	{0x7ff, 0x10},  /*WINDOW_HEIGTH*/
	{0x7ff, 0x0},   /*WINDOW_WIDTH*/
	/*X2_IAR_WINDOW_POSITION_FBUF_RD_x*/
	{0xfff, 0x10},  /*WINDOW_START_Y*/
	{0xfff, 0x0},   /*WINDOW_START_X*/
	/*X2_IAR_FORMAT_ORGANIZATION*/
	{0x1, 0xf},     /*BT601_709_SEL*/
	{0x1, 0xe},     /*RGB565_CONVERT_SEL*/
	{0x7, 0xb},     /*IMAGE_FORMAT_ORG_RD4*/
	{0x7, 0x8},     /*IMAGE_FORMAT_ORG_RD3*/
	{0xf, 0x4},     /*IMAGE_FORMAT_ORG_RD2*/
	{0xf, 0x0},     /*IMAGE_FORMAT_ORG_RD1*/
	/*X2_IAR_DISPLAY_POSTION_RD_x*/
	{0x7ff, 0x10},  /*LAYER_TOP_Y*/
	{0x7ff, 0x0},   /*LAYER_LEFT_X*/
	/*X2_IAR_HWC_CFG*/
	{0xffffff, 0x2},    /*HWC_COLOR*/
	{0x1, 0x1},     /*HWC_COLOR_EN*/
	{0x1, 0x0},     /*HWC_EN*/
	/*X2_IAR_HWC_SIZE*/
	{0x7ff, 0x10},  /*HWC_HEIGHT*/
	{0x7ff, 0x0},   /*HWC_WIDTH*/
	/*X2_IAR_HWC_POS*/
	{0x7ff, 0x10},  /*HWC_TOP_Y*/
	{0x7ff, 0x0},   /*HWC_LEFT_X*/
	/*X2_IAR_BG_COLOR*/
	{0xffffff, 0x0},    /*BG_COLOR*/
	/*X2_IAR_SRC_SIZE_UP*/
	{0x7ff, 0x10},  /*SRC_HEIGTH*/
	{0x7ff, 0x0},   /*SRC_WIDTH*/
	/*X2_IAR_TGT_SIZE_UP*/
	{0x7ff, 0x10},  /*TGT_HEIGTH*/
	{0x7ff, 0x0},   /*TGT_WIDTH*/
	/*X2_IAR_STEP_UP*/
	{0xfff, 0x10},  /*STEP_Y*/
	{0xfff, 0x0},   /*STEP_X*/
	/*X2_IAR_UP_IMAGE_POSTION*/
	{0x7ff, 0x10},  /*UP_IMAGE_TOP_Y*/
	{0x7ff, 0x0},   /*UP_IMAGE_LEFT_X*/
	/*X2_IAR_PP_CON_1*/
	{0x3f, 0xa},    /*CONTRAST*/
	{0x1, 0x9},     /*THETA_SIGN*/
	{0x1, 0x8},     /*UP_SCALING_EN*/
	{0x1, 0x7},     /*ALGORITHM_SELECT*/
	{0x1, 0x6},     /*BRIGHT_EN*/
	{0x1, 0x5},     /*CON_EN*/
	{0x1, 0x4},     /*SAT_EN*/
	{0x1, 0x3},     /*HUE_EN*/
	{0x1, 0x2},     /*GAMMA_ENABLE*/
	{0x1, 0x1},     /*DITHERING_EN*/
	{0x1, 0x0},     /*DITHERING_FLAG*/
	/*X2_IAR_PP_CON_2*/
	{0xff, 0x18},   /*OFF_BRIGHT*/
	{0xff, 0x10},   /*OFF_CONTRAST*/
	{0xff, 0x8},    /*SATURATION*/
	{0xff, 0x0},    /*THETA_ABS*/
	/*X2_IAR_THRESHOLD_RD4_3*/
	{0x7ff, 0x10},  /*THRESHOLD_RD4*/
	{0x7ff, 0x0},   /*THRESHOLD_RD3*/
	/*X2_IAR_THRESHOLD_RD2_1*/
	{0x7ff, 0x10},  /*THRESHOLD_RD2*/
	{0x7ff, 0x0},   /*THRESHOLD_RD1*/
	/*X2_IAR_CAPTURE_CON*/
	{0x1, 0x6},     /*CAPTURE_INTERLACE*/
	{0x1, 0x5},     /*CAPTURE_MODE*/
	{0x3, 0x3},     /*SOURCE_SEL*/
	{0x7, 0x0},     /*OUTPUT_FORMAT*/
	/*X2_IAR_CAPTURE_SLICE_LINES*/
	{0xfff, 0x0},   /*SLICE_LINES*/
	/*X2_IAR_BURST_LEN*/
	{0xf, 0x4},     /*BURST_LEN_WR*/
	{0xf, 0x0},     /*BURST_LEN_RD*/
	/*X2_IAR_UPDATE*/
	{0x1, 0x0},     /*UPDATE*/
	/*X2_IAR_PANEL_SIZE*/
	{0x7ff, 0x10},  /*PANEL_HEIGHT*/
	{0x7ff, 0x0},   /*PANEL_WIDTH*/
	/*X2_IAR_REFRESH_CFG*/
	{0x1, 0x11},    /*ITU_R_656_EN*/
	{0x1, 0x10},    /*UV_SEQUENCE*/
	{0xf, 0xc},     /*P3_P2_P1_P0*/
	{0x1, 0xb},     /*YCBCR_OUTPUT*/
	{0x3, 0x9},     /*PIXEL_RATE*/
	{0x1, 0x8},     /*ODD_POLARITY*/
	{0x1, 0x7},     /*DEN_POLARITY*/
	{0x1, 0x6},     /*VSYNC_POLARITY*/
	{0x1, 0x5},     /*HSYNC_POLARITY*/
	{0x1, 0x4},     /*INTERLACE_SEL*/
	{0x3, 0x2},     /*PANEL_COLOR_TYPE*/
	{0x1, 0x1},     /*DBI_REFRESH_MODE*/
	/*X2_IAR_PARAMETER_HTIM_FIELD1*/
	{0x3ff, 0x14},  /*DPI_HBP_FIELD*/
	{0x3ff, 0xa},   /*DPI_HFP_FIELD*/
	{0x3ff, 0x0},   /*DPI_HSW_FIELD*/
	/*X2_IAR_PARAMETER_VTIM_FIELD1*/
	{0x3ff, 0x14},  /*DPI_VBP_FIELD*/
	{0x3ff, 0xa},   /*DPI_VFP_FIELD*/
	{0x3ff, 0x0},   /*DPI_VSW_FIELD*/
	/*X2_IAR_PARAMETER_HTIM_FIELD2*/
	{0x3ff, 0x14},  /*DPI_HBP_FIELD2*/
	{0x3ff, 0xa},   /*DPI_HFP_FIELD2*/
	{0x3ff, 0x0},   /*DPI_HSW_FIELD2*/
	/*X2_IAR_PARAMETER_VTIM_FIELD2*/
	{0x3ff, 0x14},  /*DPI_VBP_FIELD2*/
	{0x3ff, 0xa},   /*DPI_VFP_FIELD2*/
	{0x3ff, 0x0},   /*DPI_VSW_FIELD2*/
	/*X2_IAR_PARAMETER_VFP_CNT_FIELD12*/
	{0xffff, 0x0},  /*PARAMETER_VFP_CNT*/
	/*X2_IAR_GAMMA_X1_X4_R*/
	{0xff, 0x18},   /*GAMMA_XY_D_R*/
	{0xff, 0x10},   /*GAMMA_XY_C_R*/
	{0xff, 0x8},    /*GAMMA_XY_B_R*/
	{0xff, 0x0},    /*GAMMA_XY_A_R*/
	/*X2_IAR_GAMMA_Y16_RGB*/
	{0xff, 0x10},   /*GAMMA_Y16_R*/
	{0xff, 0x8},    /*GAMMA_Y16_G*/
	{0xff, 0x0},    /*GAMMA_Y16_B*/
	/*X2_IAR_HWC_SRAM_WR*/
	{0x1ff, 0x10},  /*HWC_SRAM_ADDR*/
	{0xffff, 0x0},  /*HWC_SRAM_D*/
	/*X2_IAR_PALETTE*/
	{0xff, 0x18},   /*PALETTE_INDEX*/
	{0xffffff, 0x0},    /*PALETTE_DATA*/
	/*X2_IAR_DE_SRCPNDREG*/
	{0x1, 0x1a},    /*INT_WR_FIFO_FULL*/
	{0x1, 0x19},    /*INT_CBUF_SLICE_START*/
	{0x1, 0x18},    /*INT_CBUF_SLICE_END*/
	{0x1, 0x17},    /*INT_CBUF_FRAME_END*/
	{0x1, 0x16},    /*INT_FBUF_FRAME_END*/
	{0x1, 0x15},    /*INT_FBUF_SWITCH_RD4*/
	{0x1, 0x14},    /*INT_FBUF_SWITCH_RD3*/
	{0x1, 0x13},    /*INT_FBUF_SWITCH_RD2*/
	{0x1, 0x12},    /*INT_FBUF_SWITCH_RD1*/
	{0x1, 0x11},    /*INT_FBUF_START_RD4*/
	{0x1, 0x10},    /*INT_FBUF_START_RD3*/
	{0x1, 0xf},     /*INT_FBUF_START_RD2*/
	{0x1, 0xe},     /*INT_FBUF_START_RD1*/
	{0x1, 0xd},     /*INT_FBUF_START*/
	{0x1, 0xc},     /*INT_BUFFER_EMPTY_RD4*/
	{0x1, 0xb},     /*INT_BUFFER_EMPTY_RD3*/
	{0x1, 0xa},     /*INT_BUFFER_EMPTY_RD2*/
	{0x1, 0x9},     /*INT_BUFFER_EMPTY_RD1*/
	{0x1, 0x8},     /*INT_THRESHOLD_RD4*/
	{0x1, 0x7},     /*INT_THRESHOLD_RD3*/
	{0x1, 0x6},     /*INT_THRESHOLD_RD2*/
	{0x1, 0x5},     /*INT_THRESHOLD_RD1*/
	{0x1, 0x4},     /*INT_FBUF_END_RD4*/
	{0x1, 0x3},     /*INT_FBUF_END_RD3*/
	{0x1, 0x2},     /*INT_FBUF_END_RD2*/
	{0x1, 0x1},     /*INT_FBUF_END_RD1*/
	{0x1, 0x0},     /*INT_VSYNC*/
	/*X2_IAR_DE_REFRESH_EN*/
	{0x1, 0x1},     /*AUTO_DBI_REFRESH_EN*/
	{0x1, 0x0},     /*DPI_TV_START*/
	/*X2_IAR_DE_CONTROL_WO*/
	{0x1, 0x2},     /*FIELD_ODD_CLEAR*/
	{0x1, 0x1},     /*DBI_START*/
	{0x1, 0x0},     /*CAPTURE_EN*/
	/*X2_IAR_DE_STATUS*/
	{0xf, 0x1c},    /*DMA_STATE_RD4*/
	{0xf, 0x18},    /*DMA_STATE_RD3*/
	{0xf, 0x14},    /*DMA_STATE_RD2*/
	{0xf, 0x10},    /*DMA_STATE_RD1*/
	{0xf, 0xc},     /*DMA_STATE_WR*/
	{0x7, 0x9},     /*DPI_STATE*/
	{0xf, 0x5},     /*DBI_STATE*/
	{0x1, 0x1},     /*CAPTURE_STATUS*/
	{0x1, 0x0},     /*REFRESH_STATUS*/
	/*X2_IAR_AXI_DEBUG_STATUS1*/
	{0xffff, 0x3},  /*CUR_BUF*/
	{0x1, 0x2},     /*CUR_BUFGRP*/
	{0x1, 0x1},     /*STALL_OCCUR*/
	{0x1, 0x0},     /*ERROR_OCCUR*/
	/*X2_IAR_DE_MAXOSNUM_RD*/
	{0x7, 0x1c},    /*MAXOSNUM_DMA0_RD1*/
	{0x7, 0x18},    /*MAXOSNUM_DMA1_RD1*/
	{0x7, 0x14},    /*MAXOSNUM_DMA2_RD1*/
	{0x7, 0x10},    /*MAXOSNUM_DMA0_RD2*/
	{0x7, 0xc},     /*MAXOSNUM_DMA1_RD2*/
	{0x7, 0x8},     /*MAXOSNUM_DMA2_RD2*/
	{0x7, 0x4},     /*MAXOSNUM_RD3*/
	{0x7, 0x0},     /*MAXOSNUM_RD4*/
	/*X2_IAR_DE_MAXOSNUM_WR*/
	{0x7, 0x4}, /*MAXOSNUM_DMA0_WR*/
	{0x7, 0x0}, /*MAXOSNUM_DMA1_WR*/
	/*X2_IAR_DE_SW_RST*/
	{0x1, 0x1},     /*WR_RST*/
	{0x1, 0x0},         /*RD_RST*/
	/*X2_IAR_DE_OUTPUT_SEL*/
	{0x1, 0x3},     /*IAR_OUTPUT_EN*/
	{0x1, 0x2},     /*RGB_OUTPUT_EN*/
	{0x1, 0x1},     /*BT1120_OUTPUT_EN*/
	{0x1, 0x0},     /*MIPI_OUTPUT_EN*/
	/*X2_IAR_DE_AR_CLASS_WEIGHT*/
	{0xffff, 0x10}, /*AR_CLASS3_WEIGHT*/
	{0xffff, 0x0},   /*AR_CLASS2_WEIGHT*/
};

typedef enum _iar_table_e {
	TABLE_MASK = 0,
	TABLE_OFFSET,
	TABLE_MAX,
} iar_table_t;

#define FBUF_SIZE_ADDR_OFFSET(X)  (REG_IAR_CROPPED_WINDOW_RD1-((X)*0x4))
#define FBUF_WIDTH_ADDR_OFFSET(X)  (REG_IAR_IMAGE_WIDTH_FBUF_RD1-((X)*0x4))
#define WIN_POS_ADDR_OFFSET(X)	(REG_IAR_DISPLAY_POSTION_RD1-((X)*0x4))
#define KEY_COLOR_ADDR_OFFSET(X)  (REG_IAR_KEY_COLOR_RD1-((X)*0x4))

#define VALUE_SET(value,mask,offset,regvalue)	((((value)&(mask))<<(offset)) | ((regvalue)&~((mask)<<(offset))))
#define VALUE_GET(mask,offset,regvalue) (((regvalue)>>(offset)) & (mask))

#define IAR_REG_SET_FILED(key, value, regvalue) VALUE_SET(value, g_iarReg_cfg_table[key][TABLE_MASK], g_iarReg_cfg_table[key][TABLE_OFFSET], regvalue)
#define IAR_REG_GET_FILED(key, regvalue) VALUE_GET(g_iarReg_cfg_table[key][TABLE_MASK], g_iarReg_cfg_table[key][TABLE_OFFSET], regvalue)

struct iar_dev_s {
	struct platform_device *pdev;
	void __iomem *regaddr;
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
};
struct iar_dev_s *g_iar_dev;

void x2_iar_dump(void)
{
	void __iomem *regaddr = g_iar_dev->regaddr;
	for (; (regaddr - g_iar_dev->regaddr) <= 0x404; regaddr += 0x4) {
		int regval = readl(regaddr);
		printk("iar reg:[0x%p]: 0x%x \n", regaddr, regval);
	}
}

buf_addr_t iar_addr_convert(phys_addr_t paddr)
{
	buf_addr_t addr;
	addr.addr = paddr;
	return addr;
}

int32_t iar_config_pixeladdr(void)
{
	int i = 0;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	for (i = 0; i < IAR_CHANNEL_MAX; i++) {
		if (i == IAR_CHANNEL_1 || i == IAR_CHANNEL_2) {
			switch (g_iar_dev->channel_format[i]) {
			case FORMAT_YUV420P_VU:
				{
					unsigned uoffset, voffset;
					voffset = g_iar_dev->buf_w_h[i][0] * g_iar_dev->buf_w_h[i][1];
					uoffset = g_iar_dev->buf_w_h[i][0] * g_iar_dev->buf_w_h[i][1] * 5 / 4;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Vaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr + voffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Vaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr + voffset;
				}
				break;
			case FORMAT_YUV420P_UV:
				{
					unsigned uoffset, voffset;
					uoffset = g_iar_dev->buf_w_h[i][0] * g_iar_dev->buf_w_h[i][1];
					voffset = g_iar_dev->buf_w_h[i][0] * g_iar_dev->buf_w_h[i][1] * 5 / 4;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Vaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr + voffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Vaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr + voffset;
				}
				break;
			case FORMAT_YUV420SP_UV:
			case FORMAT_YUV420SP_VU:
			case FORMAT_YUV422SP_UV:
			case FORMAT_YUV422SP_VU:
				{
					unsigned uoffset, voffset;
					uoffset = voffset = g_iar_dev->buf_w_h[i][0] * g_iar_dev->buf_w_h[i][1];
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Vaddr = 0;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Vaddr = 0;
				}
				break;
			case FORMAT_YUV422P_UV:
				{
					unsigned uoffset, voffset;
					uoffset = g_iar_dev->buf_w_h[i][0] * g_iar_dev->buf_w_h[i][1];
					voffset = g_iar_dev->buf_w_h[i][0] * g_iar_dev->buf_w_h[i][1] * 3 / 2;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Vaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr + voffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Vaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr + voffset;
				}
				break;
			case FORMAT_YUV422P_VU:
				{
					unsigned uoffset, voffset;
					voffset = g_iar_dev->buf_w_h[i][0] * g_iar_dev->buf_w_h[i][1];
					uoffset = g_iar_dev->buf_w_h[i][0] * g_iar_dev->buf_w_h[i][1] * 3 / 2;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Vaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr + voffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Uaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr + uoffset;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Vaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr + voffset;
				}
				break;
			case FORMAT_YUV422_UYVY:
			case FORMAT_YUV422_VYUY:
			case FORMAT_YUV422_YVYU:
			case FORMAT_YUV422_YUYV:
				{
					g_iar_dev->pingpong_buf[i].pixel_addr[0].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr;
					g_iar_dev->pingpong_buf[i].pixel_addr[1].Yaddr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr;
				}
				break;
			default:
				printk("not supported format\n");
				break;
			}
		} else {
			g_iar_dev->pingpong_buf[i].pixel_addr[0].addr = g_iar_dev->pingpong_buf[i].framebuf[0].paddr;
			g_iar_dev->pingpong_buf[i].pixel_addr[1].addr = g_iar_dev->pingpong_buf[i].framebuf[1].paddr;
		}
	}
	return 0;
}

int32_t iar_set_hvsync_timing(int outmode)
{
	uint32_t value;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}

	value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_HTIM_FIELD1);
	value = IAR_REG_SET_FILED(IAR_DPI_HBP_FIELD2, 0x3e8, value);
	value = IAR_REG_SET_FILED(IAR_DPI_HFP_FIELD2, 0x3e8, value);
	value = IAR_REG_SET_FILED(IAR_DPI_HSW_FIELD2, 0x5, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_HTIM_FIELD1);

	value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD1);
	value = IAR_REG_SET_FILED(IAR_DPI_VBP_FIELD, 0x4, value);
	if (outmode == OUTPUT_BT1120)
		value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD, 0x6, value);
	else
		value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD, 0x1, value);
	value = IAR_REG_SET_FILED(IAR_DPI_VSW_FIELD, 0x1, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD1);

	writel(0xa, g_iar_dev->regaddr + REG_IAR_PARAMETER_VFP_CNT_FIELD12);

	return 0;
}

int32_t iar_set_panel_timing(struct fb_info *fb)
{
	uint32_t value;

	if (g_iar_dev == NULL) {
		pr_err("IAR dev not inited!");
		return -1;
	}
	//bt->rgb888, 5(7)inch pannel
	value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_HTIM_FIELD1);
	value = IAR_REG_SET_FILED(IAR_DPI_HBP_FIELD,
			fb->var.right_margin, value);
	value = IAR_REG_SET_FILED(IAR_DPI_HFP_FIELD,
			fb->var.left_margin, value);
	value = IAR_REG_SET_FILED(IAR_DPI_HSW_FIELD,
			fb->var.hsync_len, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_HTIM_FIELD1);

//	value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_HTIM_FIELD2);
//	value = IAR_REG_SET_FILED(IAR_DPI_HBP_FIELD2,
//	fb->var.right_margin, value);
//	value = IAR_REG_SET_FILED(IAR_DPI_HFP_FIELD2,
//	fb->var.left_margin, value);
//	value = IAR_REG_SET_FILED(IAR_DPI_HSW_FIELD2,
//	fb->var.hsync_len, value);
//	writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_HTIM_FIELD2);


	value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD1);
	value = IAR_REG_SET_FILED(IAR_DPI_VBP_FIELD, fb->var.lower_margin,
			value);
//	if (outmode == OUTPUT_BT1120)
		value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD,
				fb->var.upper_margin, value);
//	else
//		value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD, 0x1, value);
	value = IAR_REG_SET_FILED(IAR_DPI_VSW_FIELD, fb->var.vsync_len, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD1);

	value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD2);
	value = IAR_REG_SET_FILED(IAR_DPI_VBP_FIELD2,
			fb->var.lower_margin, value);
//	if (outmode == OUTPUT_BT1120)
		value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD2,
				fb->var.upper_margin, value);
//	else
//		value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD2, 0x1, value);
	value = IAR_REG_SET_FILED(IAR_DPI_VSW_FIELD2, fb->var.vsync_len, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD2);

	writel(0xa, g_iar_dev->regaddr + REG_IAR_PARAMETER_VFP_CNT_FIELD12);
	//???????

	return 0;
}
EXPORT_SYMBOL_GPL(iar_set_panel_timing);

int32_t iar_channel_base_cfg(channel_base_cfg_t *cfg)
{
	uint32_t value, channelid, pri, target_filed;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	channelid = cfg->channel;
	pri = cfg->pri;

	value = IAR_REG_SET_FILED(IAR_WINDOW_WIDTH, cfg->width, 0); //set width
	value = IAR_REG_SET_FILED(IAR_WINDOW_HEIGTH, cfg->height, value);
	writel(value, g_iar_dev->regaddr + FBUF_SIZE_ADDR_OFFSET(channelid));

	writel(cfg->buf_width, g_iar_dev->regaddr + FBUF_WIDTH_ADDR_OFFSET(channelid));

	value = IAR_REG_SET_FILED(IAR_WINDOW_START_X, cfg->xposition, 0); //set display position
	value = IAR_REG_SET_FILED(IAR_WINDOW_START_Y, cfg->yposition, value);
	writel(value, g_iar_dev->regaddr + WIN_POS_ADDR_OFFSET(channelid));

	target_filed = IAR_IMAGE_FORMAT_ORG_RD1 - channelid; //set format
	value = readl(g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
	value = IAR_REG_SET_FILED(target_filed, cfg->format, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
	g_iar_dev->channel_format[channelid] = cfg->format;

	value = readl(g_iar_dev->regaddr + REG_IAR_ALPHA_VALUE);
	target_filed = IAR_ALPHA_RD1 - channelid; //set alpha
	value = IAR_REG_SET_FILED(target_filed, cfg->alpha, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_ALPHA_VALUE);

	writel(cfg->keycolor, g_iar_dev->regaddr + KEY_COLOR_ADDR_OFFSET(channelid)); //set keycolor

	value = readl(g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
	target_filed = IAR_LAYER_PRIORITY_1 - channelid; //set layer pri
	value = IAR_REG_SET_FILED(target_filed, cfg->pri, value);

	value = IAR_REG_SET_FILED(IAR_EN_OVERLAY_PRI1, 0x1, value);
	value = IAR_REG_SET_FILED(IAR_EN_OVERLAY_PRI2, 0x1, value);
	value = IAR_REG_SET_FILED(IAR_EN_OVERLAY_PRI3, 0x1, value);
	value = IAR_REG_SET_FILED(IAR_EN_OVERLAY_PRI4, 0x1, value);

	target_filed = IAR_EN_RD_CHANNEL1 - channelid; //enable this channel
	value = IAR_REG_SET_FILED(target_filed, cfg->enable, value);
	target_filed = IAR_ALPHA_SELECT_PRI1 - pri; //set alpha sel
	value = IAR_REG_SET_FILED(target_filed, cfg->alpha_sel, value);
	target_filed = IAR_OV_MODE_PRI1 - pri; //set overlay mode
	value = IAR_REG_SET_FILED(target_filed, cfg->ov_mode, value);
	target_filed = IAR_EN_ALPHA_PRI1 - pri; //set alpha en
	value = IAR_REG_SET_FILED(target_filed, cfg->alpha_en, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);

	g_iar_dev->buf_w_h[channelid][0] = cfg->buf_width;
	g_iar_dev->buf_w_h[channelid][1] = cfg->buf_height;
	iar_config_pixeladdr();

	return 0;
}
EXPORT_SYMBOL_GPL(iar_channel_base_cfg);

int32_t iar_upscaling_cfg(upscaling_cfg_t *cfg)
{
	uint32_t value;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}

	value = IAR_REG_SET_FILED(IAR_SRC_HEIGTH, cfg->src_height, 0);
	value = IAR_REG_SET_FILED(IAR_SRC_WIDTH, cfg->src_width, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_SRC_SIZE_UP);

	value = IAR_REG_SET_FILED(IAR_TGT_HEIGTH, cfg->tgt_height, 0);
	value = IAR_REG_SET_FILED(IAR_TGT_WIDTH, cfg->tgt_width, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_TGT_SIZE_UP);

	value = IAR_REG_SET_FILED(IAR_STEP_Y, cfg->step_y, 0);
	value = IAR_REG_SET_FILED(IAR_STEP_X, cfg->step_x, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_STEP_UP);

	value = IAR_REG_SET_FILED(IAR_UP_IMAGE_LEFT_X, cfg->pos_x, 0);
	value = IAR_REG_SET_FILED(IAR_UP_IMAGE_TOP_Y, cfg->pos_y, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_UP_IMAGE_POSTION);

	value = readl(g_iar_dev->regaddr + REG_IAR_PP_CON_1);
	value = IAR_REG_SET_FILED(IAR_UP_SCALING_EN, cfg->enable, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PP_CON_1);

	return 0;
}
EXPORT_SYMBOL_GPL(iar_upscaling_cfg);

int32_t iar_gamma_cfg(gamma_cfg_t *cfg)
{
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	writel(cfg->gamma_xr[0].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X1_X4_R);
	writel(cfg->gamma_xr[1].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X5_X8_R);
	writel(cfg->gamma_xr[2].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X9_X12_R);
	writel(cfg->gamma_xr[3].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X13_X15_R);
	writel(cfg->gamma_xg[0].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X1_X4_G);
	writel(cfg->gamma_xg[1].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X5_X8_G);
	writel(cfg->gamma_xg[2].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X9_X12_G);
	writel(cfg->gamma_xg[3].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X13_X15_G);
	writel(cfg->gamma_xb[0].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X1_X4_B);
	writel(cfg->gamma_xb[1].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X5_X8_B);
	writel(cfg->gamma_xb[2].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X9_X12_B);
	writel(cfg->gamma_xb[3].value, g_iar_dev->regaddr + REG_IAR_GAMMA_X13_X15_B);

	writel(cfg->gamma_xr[0].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y1_Y3_R);
	writel(cfg->gamma_xr[1].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y4_Y7_R);
	writel(cfg->gamma_xr[2].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y8_Y11_R);
	writel(cfg->gamma_xr[3].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y12_Y15_R);
	writel(cfg->gamma_xg[0].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y1_Y3_G);
	writel(cfg->gamma_xg[1].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y4_Y7_G);
	writel(cfg->gamma_xg[2].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y8_Y11_G);
	writel(cfg->gamma_xg[3].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y12_Y15_G);
	writel(cfg->gamma_xb[0].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y1_Y3_B);
	writel(cfg->gamma_xb[1].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y4_Y7_B);
	writel(cfg->gamma_xb[2].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y8_Y11_B);
	writel(cfg->gamma_xb[3].value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y12_Y15_B);

	writel(cfg->gamma_y16rgb.value, g_iar_dev->regaddr + REG_IAR_GAMMA_Y16_RGB);
	return 0;
}
EXPORT_SYMBOL_GPL(iar_gamma_cfg);

int32_t iar_output_cfg(output_cfg_t *cfg)
{
	uint32_t value;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	if (cfg->out_sel == OUTPUT_BT1120) {
		ips_pinmux_bt();
		ips_set_btout_clksrc(IAR_CLK);
	}
	iar_set_hvsync_timing(cfg->out_sel);

	writel(cfg->bgcolor, g_iar_dev->regaddr + REG_IAR_BG_COLOR);
	writel((0x1 << cfg->out_sel), g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);
	value = IAR_REG_SET_FILED(IAR_PANEL_WIDTH, cfg->width, 0);
	value = IAR_REG_SET_FILED(IAR_PANEL_HEIGHT, cfg->height, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PANEL_SIZE);

	value = IAR_REG_SET_FILED(IAR_CONTRAST, cfg->ppcon1.contrast, 0);
	value = IAR_REG_SET_FILED(IAR_THETA_SIGN, cfg->ppcon1.theta_sign, value);
	value = IAR_REG_SET_FILED(IAR_BRIGHT_EN, cfg->ppcon1.bright_en, value);
	value = IAR_REG_SET_FILED(IAR_CON_EN, cfg->ppcon1.con_en, value);
	value = IAR_REG_SET_FILED(IAR_SAT_EN, cfg->ppcon1.sat_en, value);
	value = IAR_REG_SET_FILED(IAR_HUE_EN, cfg->ppcon1.hue_en, value);
	value = IAR_REG_SET_FILED(IAR_GAMMA_ENABLE, cfg->ppcon1.gamma_en, value);
	value = IAR_REG_SET_FILED(IAR_DITHERING_EN, cfg->ppcon1.dithering_en, value);
	value = IAR_REG_SET_FILED(IAR_DITHERING_FLAG, cfg->ppcon1.dithering_flag, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PP_CON_1);

	value = IAR_REG_SET_FILED(IAR_OFF_BRIGHT, cfg->ppcon2.off_bright, 0);
	value = IAR_REG_SET_FILED(IAR_OFF_CONTRAST, cfg->ppcon2.off_contrast, value);
	value = IAR_REG_SET_FILED(IAR_SATURATION, cfg->ppcon2.saturation, value);
	value = IAR_REG_SET_FILED(IAR_THETA_ABS, cfg->ppcon2.theta_abs, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PP_CON_2);


	value = IAR_REG_SET_FILED(IAR_DBI_REFRESH_MODE, cfg->refresh_cfg.dbi_refresh_mode, 0);
	value = IAR_REG_SET_FILED(IAR_PANEL_COLOR_TYPE, cfg->refresh_cfg.panel_corlor_type, value);
	value = IAR_REG_SET_FILED(IAR_INTERLACE_SEL, cfg->refresh_cfg.interlace_sel, value);
	value = IAR_REG_SET_FILED(IAR_ODD_POLARITY, cfg->refresh_cfg.odd_polarity, value);
	value = IAR_REG_SET_FILED(IAR_PIXEL_RATE, cfg->refresh_cfg.pixel_rate, value);
	value = IAR_REG_SET_FILED(IAR_YCBCR_OUTPUT, cfg->refresh_cfg.ycbcr_out, value);
	value = IAR_REG_SET_FILED(IAR_UV_SEQUENCE, cfg->refresh_cfg.uv_sequence, value);
	value = IAR_REG_SET_FILED(IAR_ITU_R_656_EN, cfg->refresh_cfg.itu_r656_en, value);
	value = IAR_REG_SET_FILED(IAR_PIXEL_RATE, 0, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);

	return 0;
}
EXPORT_SYMBOL_GPL(iar_output_cfg);

int32_t iar_idma_init(void)
{
	uint32_t value;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	//maxosum
	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_DMA0_RD1, 0x7, 0);
	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_DMA1_RD1, 0x6, value);
	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_DMA2_RD1, 0x5, value);
	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_DMA0_RD2, 0x6, value);
	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_DMA1_RD2, 0x4, value);
	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_DMA2_RD2, 0x2, value);
	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_RD3, 0x5, value);
	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_RD4, 0x7, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_DE_MAXOSNUM_RD);

	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_DMA0_WR, 0x0, 0);
	value = IAR_REG_SET_FILED(IAR_MAXOSNUM_DMA1_WR, 0x6, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_DE_MAXOSNUM_WR);

	//set burst length
	value = IAR_REG_SET_FILED(IAR_BURST_LEN_WR, 0xf, 0);
	value = IAR_REG_SET_FILED(IAR_BURST_LEN_RD, 0xf, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_BURST_LEN);

	return 0;
}

frame_buf_t* iar_get_framebuf_addr(uint32_t channel)
{
	int index;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return NULL;
	}
	index = g_iar_dev->cur_framebuf_id[channel];
	return &g_iar_dev->pingpong_buf[channel].framebuf[index];
}
EXPORT_SYMBOL_GPL(iar_get_framebuf_addr);

int32_t iar_set_bufaddr(uint32_t channel, buf_addr_t *addr)
{
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	IAR_DEBUG_PRINT("channel:%d addr:0x%x", channel, addr->Yaddr);
	switch (channel) {
	case IAR_CHANNEL_1:
		{
			writel(addr->Yaddr, g_iar_dev->regaddr + REG_IAR_FBUF_ADDR_RD1_Y);
			writel(addr->Uaddr, g_iar_dev->regaddr + REG_IAR_FBUF_ADDR_RD1_U);
			writel(addr->Vaddr, g_iar_dev->regaddr + REG_IAR_FBUF_ADDR_RD1_V);
		}
		break;
	case IAR_CHANNEL_2:
		{
			writel(addr->Yaddr, g_iar_dev->regaddr + REG_IAR_FBUF_ADDR_RD2_Y);
			writel(addr->Uaddr, g_iar_dev->regaddr + REG_IAR_FBUF_ADDR_RD2_U);
			writel(addr->Vaddr, g_iar_dev->regaddr + REG_IAR_FBUF_ADDR_RD2_V);
		}
		break;
	case IAR_CHANNEL_3:
		{
			writel(addr->addr, g_iar_dev->regaddr + REG_IAR_FBUF_ADDR_RD3);
		}
		break;
	case IAR_CHANNEL_4:
		{
			writel(addr->addr, g_iar_dev->regaddr + REG_IAR_FBUF_ADDR_RD4);
		}
		break;
	default:
		printk(KERN_ERR "err channel set bufaddr\n");
	}
	return 0;
}
EXPORT_SYMBOL_GPL(iar_set_bufaddr);

int32_t iar_switch_buf(uint32_t channel)
{
	uint32_t index;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	index = g_iar_dev->cur_framebuf_id[channel];
	iar_set_bufaddr(channel, &g_iar_dev->pingpong_buf[channel].pixel_addr[index]);
	g_iar_dev->cur_framebuf_id[channel] = !index;
	return 0;
}
EXPORT_SYMBOL_GPL(iar_switch_buf);

int32_t iar_open(void)
{
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	//value = readl(g_iar_dev->sysctrl + 0x144);
	//value |= (0x1<<2);
	//writel(value, g_iar_dev->sysctrl + 0x144);
	//iar_pre_init();
	return 0;
}
EXPORT_SYMBOL_GPL(iar_open);

int32_t iar_close(void)
{
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	iar_stop();
	//value = readl(g_iar_dev->sysctrl + 0x148);
	//value |= (0x1<<2);
	//writel(value, g_iar_dev->sysctrl + 0x148);

	return 0;
}
EXPORT_SYMBOL_GPL(iar_close);

#if 1
unsigned int frequency_iar = 0;
module_param(frequency_iar, uint, S_IRUGO | S_IWUSR);

static void iar_timer(unsigned long dontcare);

static DEFINE_TIMER(iartimer, iar_timer, 0, 0);
static void iar_timer(unsigned long dontcare)
{
	printk("fq:%d\n", frequency_iar);
	frequency_iar = 0;

	mod_timer(&iartimer, jiffies + msecs_to_jiffies(MSEC_PER_SEC));
}
#endif

int32_t iar_start(int update)
{
	uint32_t value;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}

	value = readl(g_iar_dev->regaddr + REG_IAR_DE_REFRESH_EN);
	value = IAR_REG_SET_FILED(IAR_DPI_TV_START, 0x1, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_DE_REFRESH_EN);
	//mod_timer(&iartimer,jiffies + msecs_to_jiffies( MSEC_PER_SEC));
	writel(0x1, g_iar_dev->regaddr + REG_IAR_UPDATE);

	return 0;
}
EXPORT_SYMBOL_GPL(iar_start);

int32_t iar_stop(void)
{
	uint32_t value;
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	value = readl(g_iar_dev->regaddr + REG_IAR_DE_REFRESH_EN);
	value = IAR_REG_SET_FILED(IAR_DPI_TV_START, 0x0, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_DE_REFRESH_EN);

	writel(0x1, g_iar_dev->regaddr + REG_IAR_UPDATE);
	//del_timer(&iartimer);
	return 0;
}
EXPORT_SYMBOL_GPL(iar_stop);

int32_t iar_update(void)
{
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	writel(0x1, g_iar_dev->regaddr + REG_IAR_UPDATE);

	return 0;
}
EXPORT_SYMBOL_GPL(iar_update);

int32_t iar_pre_init(void)
{
	buf_addr_t * bufaddr_channe1,*bufaddr_channe3;

	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}

	iar_idma_init();
	bufaddr_channe1 = &g_iar_dev->pingpong_buf[IAR_CHANNEL_1].pixel_addr[0];
	bufaddr_channe1->Yaddr = g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr;
	iar_set_bufaddr(IAR_CHANNEL_1, bufaddr_channe1);

	bufaddr_channe3 = &g_iar_dev->pingpong_buf[IAR_CHANNEL_3].pixel_addr[0];
	bufaddr_channe3->addr = g_iar_dev->pingpong_buf[IAR_CHANNEL_3].framebuf[0].paddr;
	iar_set_bufaddr(IAR_CHANNEL_3, bufaddr_channe3);

	writel(0x7ffffff, g_iar_dev->regaddr + REG_IAR_DE_SETMASK);

	return 0;
}

frame_buf_t* x2_iar_get_framebuf_addr(int channel)
{
	if (NULL == g_iar_dev || channel < 0 || channel > IAR_CHANNEL_4) {
		printk(KERN_ERR "IAR dev not inited!");
		return NULL;
	}
	return &g_iar_dev->frambuf[channel];
}

static irqreturn_t x2_iar_irq(int this_irq, void *data)
{
	//struct iar_dev_s *iar = data;
	int regval = 0;
	disable_irq_nosync(this_irq);
	//TODO
	regval = readl(g_iar_dev->regaddr + REG_IAR_DE_SRCPNDREG);
	if (regval & BIT(22)) {
		writel(0xffffffff, g_iar_dev->regaddr + REG_IAR_DE_SRCPNDREG);
		//frequency_iar++;
		//printk("FD\n");
	}
	IAR_DEBUG_PRINT("IAR int:0x%x", regval);
	enable_irq(this_irq);
	return IRQ_HANDLED;
}

static int x2_iar_probe(struct platform_device *pdev)
{
	struct resource *res, *irq;
	int ret;
	struct device_node *np;
	struct resource r;
	g_iar_dev = devm_kzalloc(&pdev->dev, sizeof(struct iar_dev_s), GFP_KERNEL);
	if (!g_iar_dev) {
		dev_err(&pdev->dev, "Unable to alloc IAR DEV\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, g_iar_dev);
	spin_lock_init(&g_iar_dev->spinlock);
	g_iar_dev->lock = &g_iar_dev->spinlock;
	g_iar_dev->pdev = pdev;
	memset(&g_iar_dev->cur_framebuf_id, 0, IAR_CHANNEL_MAX * sizeof(int));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	g_iar_dev->regaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(g_iar_dev->regaddr))
		return PTR_ERR(g_iar_dev->regaddr);
	g_iar_dev->rst = devm_reset_control_get(&pdev->dev, "iar");
	if (IS_ERR(g_iar_dev->rst)) {
		dev_err(&pdev->dev, "missing controller reset\n");
		return PTR_ERR(g_iar_dev->rst);
	}
	reset_control_assert(g_iar_dev->rst);
	udelay(2);
	reset_control_deassert(g_iar_dev->rst);

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		return -ENODEV;
	}
	g_iar_dev->irq = irq->start;

	ret = request_threaded_irq(g_iar_dev->irq, x2_iar_irq, NULL, IRQF_TRIGGER_HIGH,
							   dev_name(&pdev->dev), g_iar_dev);
	disable_irq(g_iar_dev->irq);

	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&g_iar_dev->pdev->dev, "No %s specified\n", "memory-region");
		return -1;
	}
	ret = of_address_to_resource(np, 0, &r);
	if (ret) {
		dev_err(&pdev->dev, "No memory address assigned to the region\n");
		return -1;
	}

	//channel 2&4 disabled
	g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = r.start;
	g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = memremap(r.start, MAX_FRAME_BUF_SIZE * 2, MEMREMAP_WB);
	g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = r.start + MAX_FRAME_BUF_SIZE;
	g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr = r.start + MAX_FRAME_BUF_SIZE * 2;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr = ioremap_nocache(r.start + MAX_FRAME_BUF_SIZE * 2, resource_size(&r) - MAX_FRAME_BUF_SIZE * 2);
	//g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr = memremap(r.start + MAX_FRAME_BUF_SIZE*2, resource_size(&r) - MAX_FRAME_BUF_SIZE*2, MEMREMAP_WT);

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr
			= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr
			= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_3].framebuf[0].paddr
			= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_3].framebuf[0].vaddr
			= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_3].framebuf[1].paddr
			= g_iar_dev->pingpong_buf[IAR_CHANNEL_3].framebuf[0].paddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_3].framebuf[1].vaddr
			= g_iar_dev->pingpong_buf[IAR_CHANNEL_3].framebuf[0].vaddr + MAX_FRAME_BUF_SIZE;
	IAR_DEBUG_PRINT("iar 0x%p  0x%p %lld\n", g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr, g_iar_dev->pingpong_buf[IAR_CHANNEL_3].framebuf[1].vaddr, resource_size(&r) - MAX_FRAME_BUF_SIZE * 2);

	iar_pre_init();
	iar_close();
	return ret;
}

static int x2_iar_remove(struct platform_device *pdev)
{
	struct iar_dev_s *iar;

	iar = dev_get_drvdata(&pdev->dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id x2_iar_of_match[] = {
	{.compatible = "hobot,x2-iar"},
	{},
};
MODULE_DEVICE_TABLE(of, x2_iar_of_match);
#endif


static struct platform_driver x2_iar_driver = {
	.probe = x2_iar_probe,
	.remove = x2_iar_remove,
	.driver = {
		.name = "x2-iar",
		.of_match_table = of_match_ptr(x2_iar_of_match),
		//.pm = &x2_iar_pm,
	},
};

module_platform_driver(x2_iar_driver);
MODULE_LICENSE("GPL");

