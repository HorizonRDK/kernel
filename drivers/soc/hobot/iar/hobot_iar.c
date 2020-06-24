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
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/pwm.h>
#include <linux/uaccess.h>
#include <soc/hobot/hobot_ips_x2.h>
#include <soc/hobot/hobot_iar.h>
#include "linux/ion.h"
#include "../../../media/platform/hobot/common_api/vio_framemgr.h"

#define USE_ION_MEM
//#ifdef CONFIG_X3
#ifdef CONFIG_HOBOT_XJ3
#define IAR_MEM_SIZE 0x4000000	//64MB
#else
#define IAR_MEM_SIZE 0x2000000        //32MB
#endif

#define PWM_PERIOD_DEFAULT 1000
#define PWM_DUTY_DEFAULT 10
unsigned int iar_debug_level = 0;
EXPORT_SYMBOL(iar_debug_level);
module_param(iar_debug_level, uint, 0644);

#define IAR_ENABLE 1
#define IAR_DISABLE 0

#define DISPLAY_TYPE_TOTAL_SINGLE 33
#define DISPLAY_TYPE_TOTAL_MULTI 63

int panel_reset_pin = 28;
uint32_t iar_display_ipu_addr_single[DISPLAY_TYPE_TOTAL_SINGLE][2];
uint32_t iar_display_ipu_addr_dual[DISPLAY_TYPE_TOTAL_MULTI][2];
uint32_t iar_display_ipu_addr_ddrmode[33][2];
uint32_t iar_display_yaddr_offset;
uint32_t iar_display_caddr_offset;

#ifdef CONFIG_HOBOT_XJ2
uint8_t iar_display_addr_type = DS5;
uint8_t iar_display_cam_no;
#else
uint32_t hb_disp_base_board_id;
uint8_t iar_display_addr_type = DISPLAY_CHANNEL1;
uint8_t iar_display_cam_no = PIPELINE0;
uint8_t iar_display_addr_type_video1 = 0;
uint8_t iar_display_cam_no_video1 = PIPELINE0;
EXPORT_SYMBOL(hb_disp_base_board_id);
EXPORT_SYMBOL(iar_display_addr_type);
EXPORT_SYMBOL(iar_display_cam_no);
EXPORT_SYMBOL(iar_display_addr_type_video1);
EXPORT_SYMBOL(iar_display_cam_no_video1);
#endif

uint32_t iar_display_ipu_slot_size = 0x1000000;
uint8_t ch1_en;
uint8_t disp_user_config_done;
uint32_t ipu_display_slot_id;
uint32_t g_disp_yaddr;
uint32_t g_disp_caddr;
uint32_t g_disp_yaddr_video1;
uint32_t g_disp_caddr_video1;

uint8_t config_rotate;
uint8_t ipu_process_done;
uint8_t disp_user_update;
uint8_t disp_user_update_video1;
//uint8_t pingpong_config = 0;
uint8_t frame_count;
uint8_t rst_request_flag;
uint8_t disp_copy_done = 0;
static int disp_clk_already_enable = 0;
static int sif_mclk_is_open = 0;
static int sif_mclk_iar_open = 0;
static int stop_flag = 0;
phys_addr_t logo_paddr;
void *logo_vaddr = NULL;

struct disp_timing video_1920x1080 = {
	148, 88, 44, 36, 4, 5
};
struct disp_timing video_800x480 = {
	80, 120, 48, 32, 43, 2
};
struct disp_timing video_720x1280 = {
	36, 84, 24, 11, 13, 2
};
//timing for haps
//struct disp_timing video_1080x1920 = {
//	32, 300, 5, 4, 100, 5
//};
struct disp_timing video_1080x1920 = {
	100, 400, 5, 4, 400, 5
};
struct disp_timing video_720x1280_touch = {
	100, 100, 10, 20, 10, 10
};
EXPORT_SYMBOL(disp_user_config_done);
EXPORT_SYMBOL(disp_copy_done);
EXPORT_SYMBOL(video_1920x1080);
EXPORT_SYMBOL(video_800x480);
EXPORT_SYMBOL(video_720x1280);
EXPORT_SYMBOL(video_1080x1920);
EXPORT_SYMBOL(video_720x1280_touch);

uint32_t g_iar_regs[93];

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

struct iar_dev_s *g_iar_dev;
struct pwm_device *screen_backlight_pwm;
//int display_type = LCD_7_TYPE;
int display_type = UNUSED;
EXPORT_SYMBOL(display_type);

static void iar_regs_store(void)
{
	void __iomem *regaddr;
	int i;

	if (g_iar_dev == NULL) {
		pr_info("%s:%s, g_iar_dev=NULL\n", __FILE__, __func__);
		return;
	}

	/* offset: 0x0 ~ 0x94, 0x98 writeonly. */
	for (i = 0; i < 37; i++) {
		regaddr = g_iar_dev->regaddr + 0x4 * i;
		g_iar_regs[i] = readl(regaddr);
	}

	/* offset: 0x100 ~ 0x144 */
	for (i = 38; i < 56; i++) {
		regaddr = g_iar_dev->regaddr + 0x100 + (i - 38) * 0x4;
		g_iar_regs[i] = readl(regaddr);
	}

	/* offset: 0x200 ~ 0x280 */
	for (i = 56; i < 88; i++) {
		regaddr = g_iar_dev->regaddr + 0x200 + (i - 56) * 0x4;
		g_iar_regs[i] = readl(regaddr);
	}

	/* offset: 0x318 */
	regaddr = g_iar_dev->regaddr + 0x318;
	g_iar_regs[88] = readl(regaddr);

	/* offset: 0x334 */
	regaddr = g_iar_dev->regaddr + 0x334;
	g_iar_regs[89] = readl(regaddr);

	/* offset: 0x338 */
	regaddr = g_iar_dev->regaddr + 0x338;
	g_iar_regs[90] = readl(regaddr);

	/* offset: 0x340 */
	regaddr = g_iar_dev->regaddr + 0x340;
	g_iar_regs[91] = readl(regaddr);

#ifdef CONFIG_HOBOT_XJ3
	/* offset: 0x800 */
        regaddr = g_iar_dev->regaddr + 0x800;
        g_iar_regs[92] = readl(regaddr);
#endif
}

static void iar_regs_restore(void)
{
	void __iomem *regaddr;
	int i;

	if (g_iar_dev == NULL) {
		pr_info("%s:%s, g_iar_dev=NULL\n", __FILE__, __func__);
		return;
	}

	/* offset: 0x0 ~ 0x94, 0x98 writeonly. */
	for (i = 0; i < 37; i++) {
		regaddr = g_iar_dev->regaddr + 0x4 * i;
		writel(g_iar_regs[i], regaddr);
	}

	/* offset: 0x100 ~ 0x144 */
	for (i = 38; i < 56; i++) {
		regaddr = g_iar_dev->regaddr + 0x100 + (i - 38) * 0x4;
		writel(g_iar_regs[i], regaddr);
	}

	/* offset: 0x200 ~ 0x280 */
	for (i = 56; i < 88; i++) {
		regaddr = g_iar_dev->regaddr + 0x200 + (i - 56) * 0x4;
		writel(g_iar_regs[i], regaddr);
	}

	/* offset: 0x310 */
	regaddr = g_iar_dev->regaddr + 0x310;
	writel(0x7ffffff, regaddr);

	/* offset: 0x334 */
	regaddr = g_iar_dev->regaddr + 0x334;
	writel(g_iar_regs[89], regaddr);

	/* offset: 0x338 */
	regaddr = g_iar_dev->regaddr + 0x338;
	writel(g_iar_regs[90], regaddr);

	/* offset: 0x340 */
	regaddr = g_iar_dev->regaddr + 0x340;
	writel(g_iar_regs[91], regaddr);
	//writel((0x1 << g_out_sel), regaddr);

	/* offset: 0x318 */
	regaddr = g_iar_dev->regaddr + 0x318;
	writel(g_iar_regs[88], regaddr);

#ifdef CONFIG_HOBOT_XJ3
	/* offset: 0x800 */
	regaddr = g_iar_dev->regaddr + 0x800;
	writel(g_iar_regs[92], regaddr);
#endif
	/* offset: 0x98*/
	regaddr = g_iar_dev->regaddr + 0x98;
	writel(0x1, regaddr);
}

void hobot_iar_dump(void)
{
	void __iomem *regaddr = g_iar_dev->regaddr;
	for (; (regaddr - g_iar_dev->regaddr) <= 0x404; regaddr += 0x4) {
		int regval = readl(regaddr);
		printk("iar reg:[0x%p]: 0x%x \n", regaddr, regval);
	}
}
EXPORT_SYMBOL_GPL(hobot_iar_dump);

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

int disp_set_panel_timing(struct disp_timing *timing)
{
	uint32_t value;

	if (timing == NULL)
		return -1;
	pr_debug("disp set panel timing!!!!\n");
	value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_HTIM_FIELD1);
	value = IAR_REG_SET_FILED(IAR_DPI_HBP_FIELD, timing->hbp, value);
	value = IAR_REG_SET_FILED(IAR_DPI_HFP_FIELD, timing->hfp, value);
	value = IAR_REG_SET_FILED(IAR_DPI_HSW_FIELD, timing->hs, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_HTIM_FIELD1);

	value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD1);
	value = IAR_REG_SET_FILED(IAR_DPI_VBP_FIELD, timing->vbp, value);
	value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD, timing->vfp, value);
	value = IAR_REG_SET_FILED(IAR_DPI_VSW_FIELD, timing->vs, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD1);

	value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD2);
	value = IAR_REG_SET_FILED(IAR_DPI_VBP_FIELD2, timing->vbp, value);
	value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD2, timing->vfp, value);
	value = IAR_REG_SET_FILED(IAR_DPI_VSW_FIELD2, timing->vs, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD2);

	writel(0xa, g_iar_dev->regaddr + REG_IAR_PARAMETER_VFP_CNT_FIELD12);
	return 0;
}
EXPORT_SYMBOL_GPL(disp_set_panel_timing);

int32_t iar_channel_base_cfg(channel_base_cfg_t *cfg)
{
	uint32_t value, channelid, pri, target_filed;
	uint32_t reg_overlay_opt_value = 0;

	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}

	channelid = cfg->channel;
	if (cfg->enable > 0)
		cfg->enable = 1;
	if (cfg->pri > 3) {
		pr_err("iar_drvier: error channel priority, exit!!\n");
		return -1;
	}
        if (cfg->width > 1920 || cfg->buf_width > 1920 ||
			cfg->xposition > 1920 || cfg->crop_width > 1920) {
		pr_err("iar_driver: channel width exceed the max limit, exit!!\n");
		return -1;
	}
	if (cfg->height > 1920 || cfg->buf_height > 1920 ||
			cfg->yposition > 1920 || cfg->crop_height > 1920) {
		pr_err("iar_driver: channel height exceed the max limit, exit!!\n");
		return -1;
	}
	if (channelid < 2) {
		if (cfg->format > 11) {
			pr_err("iar_driver: error channel format, exit!!\n");
			return -1;
		}
	} else {
		if (cfg->format > 5) {
			pr_err("iar_driver: error channel format, exit!!\n");
			return -1;
		}
	}
	if (cfg->alpha > 255) {
		pr_err("iar_driver: error channel alpha value, exit!!\n");
		return -1;
	}
	if (cfg->alpha_sel > 1) {
		cfg->alpha_sel = 1;
	}
	if (cfg->alpha_en > 1) {
		cfg->alpha_en = 1;
	}
	if (cfg->ov_mode > 3) {
		pr_err("iar_driver: error channel ov mode, exit!!\n");
		return -1;
	}
	pri = cfg->pri;
	reg_overlay_opt_value = readl(g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
	reg_overlay_opt_value =
		reg_overlay_opt_value & (0xffffffff & ~(1 << (channelid + 24)));
	reg_overlay_opt_value =
		reg_overlay_opt_value | (cfg->enable << (channelid + 24));
	pr_info("channel id is %d, enable is %d, reg value is 0x%x.\n",
			channelid, cfg->enable, reg_overlay_opt_value);

	writel(reg_overlay_opt_value, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);

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

//	target_filed = IAR_EN_RD_CHANNEL1 - channelid; //enable this channel
//	value = IAR_REG_SET_FILED(target_filed, cfg->enable, value);
	target_filed = IAR_ALPHA_SELECT_PRI1 - pri; //set alpha sel
	value = IAR_REG_SET_FILED(target_filed, cfg->alpha_sel, value);
	target_filed = IAR_OV_MODE_PRI1 - pri; //set overlay mode
	value = IAR_REG_SET_FILED(target_filed, cfg->ov_mode, value);
	target_filed = IAR_EN_ALPHA_PRI1 - pri; //set alpha en
	value = IAR_REG_SET_FILED(target_filed, cfg->alpha_en, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);

	g_iar_dev->buf_w_h[channelid][0] = cfg->buf_width;
	g_iar_dev->buf_w_h[channelid][1] = cfg->buf_height;
	//writel(cfg->buf_height << 16 | cfg->buf_width,
	//	g_iar_dev->regaddr + REG_IAR_CROPPED_WINDOW_RD1 - channelid*4);
	writel(cfg->crop_height << 16 | cfg->crop_width,
                g_iar_dev->regaddr + REG_IAR_CROPPED_WINDOW_RD1 - channelid*4);
//	iar_config_pixeladdr();

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
	if (cfg->src_height > 1920 || cfg->tgt_height > 1920 ||
			cfg->src_width > 1920 || cfg->tgt_width > 1920) {
		pr_err("iar_driver: channel width/height exceed limit, exit!!\n");
		return -1;
	}
	if (cfg->src_height == 0 || cfg->tgt_height == 0 ||
			cfg->src_width == 0 || cfg->tgt_width == 0) {
		pr_err("iar_driver: channel width/height is zero, exit!!\n");
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

static int iar_enable_sif_mclk(void)
{
	int ret = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (g_iar_dev->sif_mclk == NULL) {
		pr_err("%s: sif_mclk is null!!\n", __func__);
		return -1;
	}
	if (sif_mclk_is_open == 0 && sif_mclk_iar_open == 0) {
		ret = clk_prepare_enable(g_iar_dev->sif_mclk);
		if (ret != 0) {
			pr_err("%s: failed to prepare sif_mclk!!\n", __func__);
			return ret;
		}
		sif_mclk_iar_open = 1;
	}
	return 0;
}

static int iar_disable_sif_mclk(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (g_iar_dev->sif_mclk == NULL) {
		pr_err("%s: sif_mclk is null!!\n", __func__);
		return -1;
	}
	if (sif_mclk_iar_open == 1) {
		clk_disable_unprepare(g_iar_dev->sif_mclk);
		sif_mclk_iar_open = 0;
	}
	return 0;
}

int8_t disp_set_pixel_clk(uint64_t pixel_clk)
{
	int32_t ret = 0;
	uint64_t pixel_rate;

#ifdef CONFIG_HOBOT_XJ2
	if (pixel_clk < 102000000)
		ips_set_iar_clk32(1);
	else
		ips_set_iar_clk32(0);
#endif
	//clk_disable_unprepare(g_iar_dev->iar_pixel_clk);

	pixel_rate = clk_round_rate(g_iar_dev->iar_pixel_clk, pixel_clk);
	ret = clk_set_rate(g_iar_dev->iar_pixel_clk, pixel_rate);
	if (ret) {
		pr_err("%s: err checkout iar pixel clock rate!!\n", __func__);
		return -1;
	}
#if 0
	ret = clk_prepare_enable(g_iar_dev->iar_pixel_clk);
	if (ret) {
		pr_err("%s: err enable iar pixel clock!!\n", __func__);
		return -1;
	}
#endif
	pixel_rate = clk_get_rate(g_iar_dev->iar_pixel_clk);
	pr_info("%s: iar pixel rate is %lld\n", __func__, pixel_rate);
	return 0;
}
EXPORT_SYMBOL_GPL(disp_set_pixel_clk);

static int disp_clk_disable(void)
{

	if (g_iar_dev->iar_pixel_clk == NULL)
		return -1;
	if (disp_clk_already_enable == 1) {
		clk_disable_unprepare(g_iar_dev->iar_pixel_clk);
		disp_clk_already_enable = 0;
	}
	return 0;
}

static int disp_clk_enable(void)
{
	uint64_t pixel_clock;
	int ret = 0;

	if (g_iar_dev->iar_pixel_clk == NULL)
		return -1;
	if (disp_clk_already_enable == 0) {
		ret = clk_prepare_enable(g_iar_dev->iar_pixel_clk);
		if (ret) {
			pr_err("%s: err enable iar pixel clock!!\n", __func__);
			return -1;
		}
		disp_clk_already_enable = 1;
	}
	if (display_type == LCD_7_TYPE || display_type == MIPI_1080P)
		pixel_clock = 32000000;
	else if (display_type == MIPI_720P_TOUCH)
		pixel_clock = 54000000;
	else if (display_type == SIF_IPI)
		pixel_clock = 54400000;
	else
		pixel_clock = 163000000;
	pixel_clock = clk_round_rate(g_iar_dev->iar_pixel_clk, pixel_clock);
	ret = clk_set_rate(g_iar_dev->iar_pixel_clk, pixel_clock);
	if (ret) {
		pr_err("%s: err checkout iar pixel clock rate!!\n", __func__);
		return -1;
	}
	pixel_clock = clk_get_rate(g_iar_dev->iar_pixel_clk);
	pr_debug("%s: iar pixel rate is %lld\n", __func__, pixel_clock);
	return 0;
}

int iar_pixel_clk_enable(void)
{
	int ret = 0;

	if (g_iar_dev->iar_pixel_clk == NULL)
		return -1;
	ret = clk_prepare_enable(g_iar_dev->iar_pixel_clk);
	if (ret) {
		pr_err("%s: err enable iar pixel clock!!\n", __func__);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(iar_pixel_clk_enable);

int iar_pixel_clk_disable(void)
{
	if (g_iar_dev->iar_pixel_clk == NULL)
		return -1;
	clk_disable_unprepare(g_iar_dev->iar_pixel_clk);
	return 0;
}
EXPORT_SYMBOL_GPL(iar_pixel_clk_disable);


static int ipi_clk_disable(void)
{
	if (g_iar_dev->iar_ipi_clk == NULL)
		return -1;
	clk_disable_unprepare(g_iar_dev->iar_ipi_clk);
	return 0;
}

static int ipi_clk_enable(void)
{
	if (g_iar_dev->iar_ipi_clk == NULL)
		return -1;
	return clk_prepare_enable(g_iar_dev->iar_ipi_clk);
}

static int screen_backlight_init(void)
{
	int ret = 0;

	pr_debug("initialize lcd backligbt!!!\n");
	screen_backlight_pwm = pwm_request(0, "lcd-pwm");
	if (IS_ERR(screen_backlight_pwm)) {
		pr_err("\nNo pwm device 0!!!!\n");
		return -ENODEV;
	}
	pr_debug("pwm request 0 is okay!!!\n");
	/**
	 * pwm_config(struct pwm_device *pwm, int duty_ns,
	 * int period_ns) - change a PWM device configuration
	 * @pwm: PWM device
	 * @duty_ns: "on" time (in nanoseconds)
	 * @period_ns: duration (in nanoseconds) of one cycle
	 *
	 * Returns: 0 on success or a negative error code on failure.
	 */
	ret = pwm_config(screen_backlight_pwm, PWM_DUTY_DEFAULT,
				PWM_PERIOD_DEFAULT);
	// 50Mhz,20ns period, on = 20ns
	if (ret) {
		pr_err("\nError config pwm!!!!\n");
		return ret;
	}
	pr_debug("pwm config is okay!!!\n");
/*
 *	ret = pwm_set_polarity(lcd_backlight_pwm,
 *			PWM_POLARITY_NORMAL);
 *	if (ret) {
 *		pr_err("\nError set pwm polarity!!!!\n");
 *		return ret;
 *	}
 *	pr_debug("pwm set polarity is okay!!!\n");
 */
	ret = pwm_enable(screen_backlight_pwm);
	if (ret) {
		pr_err("\nError enable pwm!!!!\n");
		return ret;
	}
	pr_debug("pwm enable is okay!!!\n");

	return 0;
}

int screen_backlight_change(unsigned int duty)
{
	int ret = 0;

	if (screen_backlight_pwm == NULL) {
		pr_err("pwm is not init!!\n");
		return -1;
	}

	pwm_disable(screen_backlight_pwm);

	ret = pwm_config(screen_backlight_pwm, duty, PWM_PERIOD_DEFAULT);

	if (ret) {
		pr_err("\nError config pwm!!!!\n");
		return ret;
	}

	ret = pwm_enable(screen_backlight_pwm);
	if (ret) {
		pr_err("\nError enable pwm!!!!\n");
		return ret;
	}
	pr_info("set screen backlight sucess!!\n");
	return 0;
}

int set_screen_backlight(unsigned int backlight_level)
{
	int retval = 0;
	unsigned int duty = 0;

	if (display_type == LCD_7_TYPE || display_type == MIPI_720P_TOUCH ||
			display_type == MIPI_1080P) {
		if (backlight_level == 0) {
			duty = PWM_PERIOD_DEFAULT - 1;
		} else if (backlight_level <= 10 && backlight_level > 0) {
			duty = PWM_PERIOD_DEFAULT -
				PWM_PERIOD_DEFAULT / 10 * backlight_level;
		} else {
			pr_err("error backlight value, exit!!\n");
			return -1;
		}

		retval = screen_backlight_change(duty);
		if (retval) {
			pr_err("error set lcd backlight!!\n");
			return retval;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(set_screen_backlight);

int disp_pinmux_bt1120(void)
{
	int ret = 0;
	void __iomem *pinctl_reg_addr;
        uint32_t reg_val = 0;

	if (!g_iar_dev->pins_bt1120)
		return -ENODEV;
	ret = pinctrl_select_state(g_iar_dev->pinctrl,
			g_iar_dev->pins_bt1120);
	pinctl_reg_addr = ioremap_nocache(0xa6004000 + 0x138, 4);
	reg_val = readl(pinctl_reg_addr);
	reg_val = (reg_val & 0xfffffffc) | 0x0000003c;
	writel(reg_val, pinctl_reg_addr);
	return ret;
}

int disp_pinmux_bt656(void)
{
	if (!g_iar_dev->pins_bt656)
		return -ENODEV;
	return pinctrl_select_state(g_iar_dev->pinctrl,
			g_iar_dev->pins_bt656);
}

int disp_pinmux_mipi_dsi(void)
{
	if (!g_iar_dev->pins_mipi_dsi)
		return -ENODEV;
	return pinctrl_select_state(g_iar_dev->pinctrl,
			g_iar_dev->pins_mipi_dsi);
}

int disp_pinmux_rgb(void)
{
	if (!g_iar_dev->pins_rgb)
		return -ENODEV;
	return pinctrl_select_state(g_iar_dev->pinctrl,
			g_iar_dev->pins_rgb);
}

int disp_pinmux_rgb_gpio(void)
{
	if (!g_iar_dev->pins_rgb_gpio)
		return -ENODEV;
	return pinctrl_select_state(g_iar_dev->pinctrl,
			g_iar_dev->pins_rgb_gpio);
}

int32_t iar_output_cfg(output_cfg_t *cfg)
{
	void __iomem *hitm1_reg_addr;
        uint32_t reg_val = 0;
	uint32_t value;
	int ret;

	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	if (cfg->out_sel > 4) {
		pr_err("%s: error output mode, exit!!\n", __func__);
		return -1;
	}
	if (cfg->width > 1920 || cfg->height > 1920) {
		pr_err("%s: panel width/height exceed limit, exit!!\n", __func__);
		return -1;
	}
#ifdef CONFIG_HOBOT_XJ3
	if (cfg->display_addr_type > 38 || cfg->display_addr_type_layer1 > 38) {
		pr_err("%s: error display vio addr type, exit!!\n", __func__);
		return -1;
	}
	if (cfg->display_cam_no > 3 || cfg->display_cam_no_layer1 > 3) {
		pr_err("%s: error display vio pipeline no, exit!!\n", __func__);
		return -1;
	}
#endif
	if (cfg->big_endian > 1)
		cfg->big_endian = 1;
	if (cfg->rotate > 1)
		cfg->rotate = 1;
	if (cfg->user_control_disp > 1)
		cfg->user_control_disp = 1;
	if (cfg->user_control_disp_layer1 > 1)
		cfg->user_control_disp_layer1 = 1;
	if (cfg->ppcon1.dithering_flag > 1)
		cfg->ppcon1.dithering_flag = 1; //0:rgb666;1:rgb565
	if (cfg->ppcon1.dithering_en > 1)
		cfg->ppcon1.dithering_en = 1;
	if (cfg->ppcon1.gamma_en > 1)
		cfg->ppcon1.gamma_en = 1;
	if (cfg->ppcon1.hue_en > 1)
		cfg->ppcon1.hue_en = 1;
	if (cfg->ppcon1.sat_en > 1)
		cfg->ppcon1.sat_en = 1;
	if (cfg->ppcon1.con_en > 1)
		cfg->ppcon1.con_en = 1;
	if (cfg->ppcon1.bright_en > 1)
		cfg->ppcon1.bright_en = 1;
	if (cfg->ppcon1.theta_sign > 1)
		cfg->ppcon1.theta_sign = 1; //negative
	if (cfg->ppcon1.contrast > 63) {
		pr_err("%s: ppcon1 contrast value exceed 63, exit!!\n", __func__);
		return -1;
	}
	if (cfg->ppcon2.theta_abs > 255) {
		pr_err("%s: theta abs exceed 255, exit!!\n", __func__);
		return -1;
	}
	if (cfg->ppcon2.saturation > 255) {
		pr_err("%s: saturation exceed 255, exit!!\n", __func__);
		return -1;
	}
	if (cfg->ppcon2.off_contrast > 255) {
		pr_err("%s: off contrast exceed 255, exit!!\n", __func__);
		return -1;
	}
	//if ((cfg->ppcon2.off_bright >= 0 && cfg->ppcon2.off_bright <= 0x7f) ||
	//		(cfg->ppcon2.off_bright >= 0xffffff80 &&
	//		cfg->ppcon2.off_bright <= 0xffffffff)) {
	//	pr_err("%s: off bright value error, exit!!\n", __func__);
	//	return -1;
	//}
	if (cfg->ppcon2.off_bright > 0x7f && cfg->ppcon2.off_bright < 0xffffff80) {
		pr_err("%s: off bright value error, exit!!\n", __func__);
		return -1;
	}
	writel(cfg->bgcolor, g_iar_dev->regaddr + REG_IAR_BG_COLOR);

	if (cfg->out_sel == OUTPUT_BT1120) {
		//output config
#ifdef CONFIG_HOBOT_XJ2
		ips_pinmux_bt();
#else
		display_type = HDMI_TYPE;
		ret = disp_pinmux_bt1120();
		if (ret)
			return -1;
		display_type = HDMI_TYPE;
		disp_set_panel_timing(&video_1920x1080);
#endif
#ifdef CONFIG_HOBOT_XJ2
		ips_set_btout_clksrc(IAR_CLK, true);//clk invert
#endif
		writel(0xa, g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);
		//color config
		value = readl(g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		value = IAR_REG_SET_FILED(IAR_PANEL_COLOR_TYPE, 2, value);
		//yuv444
		value = IAR_REG_SET_FILED(IAR_YCBCR_OUTPUT, 1, value);
		//convert ycbcr
		writel(value, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		value = readl(g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
		value = IAR_REG_SET_FILED(IAR_BT601_709_SEL, 1, value);
		writel(value, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
	} else if (cfg->out_sel == OUTPUT_MIPI_DSI) {
#ifdef CONFIG_HOBOT_XJ3
		//output config
		writel(0x8, g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);//0x340
		writel(0x13, g_iar_dev->regaddr + REG_DISP_LCDIF_CFG);//0x800
		writel(0x3, g_iar_dev->regaddr + REG_DISP_LCDIF_PADC_RESET_N);//0x804
		//color config
		writel(0x0, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);//0x204
		display_type = MIPI_720P_TOUCH;
#else
		pr_err("%s: error output mode!!!\n", __func__);
#endif
	} else if (cfg->out_sel == OUTPUT_RGB) {
		ret = disp_pinmux_rgb();
		if (ret) {
			pr_debug("err config RGB output pinmux!\n");
			return ret;
		}
		display_type = LCD_7_TYPE;
		disp_set_panel_timing(&video_800x480);
		hitm1_reg_addr = ioremap_nocache(0xA6004000 + 0x138, 4);
		reg_val = readl(hitm1_reg_addr);
		reg_val = (reg_val & 0xffffffc3) | 0x3c;
		writel(reg_val, hitm1_reg_addr);

		writel(0xc, g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);
		writel(0x0, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		writel(0x00000000, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		//rgb panel
	} else if (cfg->out_sel == OUTPUT_BT656) {
#ifdef CONFIG_HOBOT_XJ3
		ret = disp_pinmux_bt656();
		if (ret)
			return -1;
		writel(0x8, g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);
		// TODO(bt656_output_mode_config)
#else
		pr_err("%s: error output mode!!!\n", __func__);
#endif
	} else if (cfg->out_sel == OUTPUT_IPI) {
		writel(0x9, g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);
		//IPI(SIF)
		value = readl(g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		value = IAR_REG_SET_FILED(IAR_PANEL_COLOR_TYPE, 2, value);
		//yuv444
	} else {
		pr_err("%s: error output mode!!!\n", __func__);
		return -1;
	}

	value = IAR_REG_SET_FILED(IAR_PANEL_WIDTH, cfg->width, 0);
	value = IAR_REG_SET_FILED(IAR_PANEL_HEIGHT, cfg->height, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PANEL_SIZE);

	//writel(0x00000008, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
	config_rotate = cfg->rotate;
        disp_user_update = cfg->user_control_disp;
	disp_user_update_video1 = cfg->user_control_disp_layer1;
	iar_display_cam_no = cfg->display_cam_no;
	iar_display_addr_type = cfg->display_addr_type;
	iar_display_cam_no_video1 = cfg->display_cam_no_layer1;
	iar_display_addr_type_video1 = cfg->display_addr_type_layer1;

#ifdef CONFIG_HOBOT_XJ2
	if (iar_display_cam_no == 0) {
		iar_get_ipu_display_addr_single(iar_display_ipu_addr_single);
		iar_display_yaddr_offset = iar_display_ipu_addr_single[0][0] +
			iar_display_ipu_addr_single[cfg->display_addr_type][0];
		iar_display_caddr_offset = iar_display_ipu_addr_single[0][0] +
			iar_display_ipu_addr_single[cfg->display_addr_type][1];
		iar_display_ipu_slot_size = iar_display_ipu_addr_single[0][1];
	} else if (iar_display_cam_no == 1) {
		iar_get_ipu_display_addr_dual(iar_display_ipu_addr_dual);
		iar_display_yaddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[cfg->display_addr_type][0];
		iar_display_caddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[cfg->display_addr_type][1];
		iar_display_ipu_slot_size = iar_display_ipu_addr_dual[0][1];

	} else if (iar_display_cam_no == 2) {
		iar_get_ipu_display_addr_ddrmode(iar_display_ipu_addr_ddrmode);
		iar_display_yaddr_offset = iar_display_ipu_addr_ddrmode[0][0] +
			iar_display_ipu_addr_ddrmode[cfg->display_addr_type][0];
		iar_display_caddr_offset = iar_display_ipu_addr_ddrmode[0][0] +
			iar_display_ipu_addr_ddrmode[cfg->display_addr_type][1];
		iar_display_ipu_slot_size = iar_display_ipu_addr_ddrmode[0][1];

	}

	if (cfg->panel_type == 2) {
		if (display_type == HDMI_TYPE) {
			pr_err("%s: wrong json file or convert hw!\n",
					__func__);
		}
		display_type = MIPI_720P;
	} else if (cfg->panel_type == 1) {
		if (display_type == HDMI_TYPE) {
			pr_err("%s: wrong json file or convert hw!\n",
					__func__);
		}
		display_type = LCD_7_TYPE;
	} else if (cfg->panel_type == 0) {
		if (display_type == LCD_7_TYPE) {
			pr_err("%s: wrong json file or convert hw!\n",
					__func__);
		}
	}

	value = readl(g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
	value = IAR_REG_SET_FILED(IAR_PANEL_COLOR_TYPE, 2, value);
	value = IAR_REG_SET_FILED(IAR_YCBCR_OUTPUT, 1, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
#endif
	value = readl(g_iar_dev->regaddr + REG_IAR_PP_CON_1);
	value = IAR_REG_SET_FILED(IAR_CONTRAST, cfg->ppcon1.contrast, value);
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

#ifdef CONFIG_HOBOT_XJ3
	value = readl(g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
	if (cfg->big_endian == 0x1) {
		value = 0x00010000 | value;
	} else if (cfg->big_endian == 0x0) {
		value = 0xfffeffff & value;
	}
	writel(value, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
#endif
#if 0
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
#endif

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

void *ipu_get_iar_framebuf_addr(uint32_t channel, unsigned int index)
{
	if (g_iar_dev == NULL) {
		pr_err("IAR dev not inited!");
		return NULL;
	}
	return (void *)(g_iar_dev->pingpong_buf[channel].framebuf[index].vaddr);
}
EXPORT_SYMBOL_GPL(ipu_get_iar_framebuf_addr);


int32_t iar_set_bufaddr(uint32_t channel, buf_addr_t *addr)
{
	if (NULL == g_iar_dev) {
		pr_err("IAR dev not inited!");
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
			pr_debug("REG_IAR_FBUF_ADDR_RD3 = 0x%x\n", addr->addr);
		}
		break;
	case IAR_CHANNEL_4:
		{
			writel(addr->addr, g_iar_dev->regaddr + REG_IAR_FBUF_ADDR_RD4);
		}
		break;
	default:
		pr_debug("err channel set bufaddr\n");
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
//	g_iar_dev->cur_framebuf_id[channel] = !index;

	return 0;
}
EXPORT_SYMBOL_GPL(iar_switch_buf);

int32_t iar_set_video_buffer(uint32_t slot_id)
{
	if (disp_user_config_done == 1 && disp_user_update == 0) {
		ipu_display_slot_id = slot_id;
		ipu_process_done = 1;
		wake_up_interruptible(&g_iar_dev->wq_head);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(iar_set_video_buffer);

int32_t ipu_set_display_addr(uint32_t disp_layer,
		uint32_t yaddr, uint32_t caddr)
{
	pr_debug("iar: ipu set iar!!!!!!!!\n");
	pr_debug("layer is %d, yaddr is 0x%x, caddr is 0x%x\n",
			disp_layer, yaddr, caddr);
	disp_user_config_done = 1;//for debug
	//disp_user_update = 0;//for debug
	if (disp_user_config_done == 1 && disp_user_update == 0) {
		if (disp_layer == 0) {
			g_disp_yaddr = yaddr;
			g_disp_caddr = caddr;
		} else if (disp_layer == 1) {
			g_disp_yaddr_video1 = yaddr;
			g_disp_caddr_video1 = caddr;
		}
                ipu_process_done = 1;
		pr_debug("wake up iar!!!\n");
                wake_up_interruptible(&g_iar_dev->wq_head);
        }
        return 0;
}

EXPORT_SYMBOL_GPL(ipu_set_display_addr);

u32 ipu_get_iar_display_type(u8 pipeline[], u8 channel[])
{
	if (disp_user_update == 1) {
		return -1;
	} else {
		pipeline[0] = iar_display_cam_no;
		channel[0] = iar_display_addr_type;
		pipeline[1] = iar_display_cam_no_video1;
		channel[1] = iar_display_addr_type_video1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(ipu_get_iar_display_type);

int8_t iar_checkout_display_camera(uint8_t camera_no)
{
#ifdef CONFIG_HOBOT_XJ2
	iar_get_ipu_display_addr_dual(iar_display_ipu_addr_dual);
#endif

	iar_display_ipu_slot_size = iar_display_ipu_addr_dual[0][1];
	if (camera_no == 0) {
		pr_debug("iar: checkout camera 0!\n");
		iar_display_yaddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[iar_display_addr_type][0];
		iar_display_caddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[iar_display_addr_type][1];
	} else if (camera_no == 1) {
		pr_debug("iar: checkout camera 1!\n");
		iar_display_yaddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[30+iar_display_addr_type][0];
		iar_display_caddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[30+iar_display_addr_type][1];
		}
		return 0;

}
EXPORT_SYMBOL_GPL(iar_checkout_display_camera);

int set_video_display_channel(uint8_t channel_no)
{
	int ret = 0;

	if (iar_display_cam_no == 0) {
		pr_err("current configuration is single camera, can't set channel!!!\n");
		return -1;
	} else if (channel_no > 1) {
		pr_err("wrong camera channel number, exit!!\n");
		return -1;
	}

	ret = iar_checkout_display_camera(channel_no);

	return ret;
}
EXPORT_SYMBOL_GPL(set_video_display_channel);

int set_video_display_ddr_layer(uint8_t ddr_layer_no)
{
	int ret = 0;

	if (ddr_layer_no >= DISPLAY_TYPE_TOTAL_SINGLE) {
		pr_err("wrong display ddr layer number, exit!!!\n");
		return -1;
	}

	if (iar_display_cam_no == 0) {
		iar_display_yaddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[ddr_layer_no][0];
		iar_display_caddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[ddr_layer_no][1];

	} else {
		iar_display_yaddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[ddr_layer_no][0];
		iar_display_caddr_offset = iar_display_ipu_addr_dual[0][0] +
			iar_display_ipu_addr_dual[ddr_layer_no][1];

	}

	return ret;
}
EXPORT_SYMBOL_GPL(set_video_display_ddr_layer);

static int iar_thread(void *data)
{
	buf_addr_t display_addr;
	buf_addr_t display_addr_video1;

	while (!kthread_should_stop()) {
		if (kthread_should_stop())
			break;
		wait_event_interruptible(g_iar_dev->wq_head,
				ipu_process_done || stop_flag);
		ipu_process_done = 0;
#ifdef CONFIG_HOBOT_XJ3
		display_addr.Yaddr = g_disp_yaddr;
		display_addr.Uaddr = g_disp_caddr;
		display_addr.Vaddr = 0;
		display_addr_video1.Yaddr = g_disp_yaddr_video1;
		display_addr_video1.Uaddr = g_disp_caddr_video1;
		display_addr_video1.Vaddr = 0;
#else
		display_addr.Yaddr =
			ipu_display_slot_id * iar_display_ipu_slot_size
			+ iar_display_yaddr_offset;
		display_addr.Uaddr =
			ipu_display_slot_id * iar_display_ipu_slot_size
			+ iar_display_caddr_offset;

		display_addr.Vaddr = 0;
		//pr_debug("iar_display_yaddr offset is 0x%x.\n",
		//iar_display_yaddr_offset);
		//pr_debug("iar_display_caddr offset is 0x%x.\n",
		//iar_display_caddr_offset);
		//pr_debug("iar: iar_display_yaddr is 0x%x.\n",
		//display_addr.Yaddr);
		//pr_debug("iar: iar_display_caddr is 0x%x.\n",
		//display_addr.Uaddr);
#endif
		if (config_rotate) {
			iar_rotate_video_buffer(display_addr.Yaddr,
				display_addr.Uaddr, display_addr.Vaddr);
		} else {
			pr_debug("iar: iar display refresh!!!!!\n");
			pr_debug("iar display video 0 yaddr is 0x%x, caddr is 0x%x\n",
					display_addr.Yaddr, display_addr.Uaddr);
			pr_debug("iar display video 1 yaddr is 0x%x, caddr is 0x%x\n",
					display_addr_video1.Yaddr, display_addr_video1.Uaddr);
			iar_set_bufaddr(0, &display_addr);
			iar_set_bufaddr(1, &display_addr_video1);
			iar_update();
		}
	}
	return 0;
}


int32_t iar_open(void)
{
	int ret = 0;

	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}

	enable_sif_mclk();
	iar_pixel_clk_enable();
	enable_iar_irq();
	enable_irq(g_iar_dev->irq);

	init_waitqueue_head(&g_iar_dev->done_wq);
	init_waitqueue_head(&g_iar_dev->output_done_wq[0]);
	init_waitqueue_head(&g_iar_dev->output_done_wq[1]);

	g_iar_dev->state = BIT(IAR_WB_INIT);

	if (g_iar_dev->iar_task == NULL) {
		g_iar_dev->iar_task = kthread_run(iar_thread,
				(void *)g_iar_dev, "iar_thread");
		if (IS_ERR(g_iar_dev->iar_task)) {
			g_iar_dev->iar_task = NULL;
			dev_err(&g_iar_dev->pdev->dev, "iar thread create fail\n");
			ret = PTR_ERR(g_iar_dev->iar_task);
		}
		stop_flag = 0;
	} else {
		pr_err("ipu iar thread already run!!\n");
	}

	return ret;
}
EXPORT_SYMBOL_GPL(iar_open);

int32_t iar_layer_disable(int32_t layer_no)
{
	uint32_t reg_overlay_opt_value = 0;

	if (layer_no < 0 || layer_no > 3)
		return -1;
	reg_overlay_opt_value = readl(g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
	reg_overlay_opt_value = reg_overlay_opt_value &
		(0xffffffff & ~(1 << (layer_no + 24)));
	reg_overlay_opt_value =
		reg_overlay_opt_value | (0 << (layer_no + 24));
	writel(reg_overlay_opt_value, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
	iar_update();
	return 0;
}
EXPORT_SYMBOL_GPL(iar_layer_disable);

int32_t iar_layer_enable(int32_t layer_no)
{
	uint32_t reg_overlay_opt_value = 0;

	if (layer_no < 0 || layer_no > 3)
		return -1;
	reg_overlay_opt_value = readl(g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
	reg_overlay_opt_value = reg_overlay_opt_value &
		(0xffffffff & ~(1 << (layer_no + 24)));
	reg_overlay_opt_value = reg_overlay_opt_value |
		(1 << (layer_no + 24));
	writel(reg_overlay_opt_value, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
	iar_update();
	return 0;
}
EXPORT_SYMBOL_GPL(iar_layer_enable);

int32_t iar_close(void)
{
	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	disp_user_config_done = 0;
	iar_stop();

	disable_irq(g_iar_dev->irq);
	frame_manager_close(&g_iar_dev->framemgr);
	frame_manager_close(&g_iar_dev->framemgr_layer[0]);
	frame_manager_close(&g_iar_dev->framemgr_layer[1]);

	if (g_iar_dev->iar_task == NULL) {
		pr_err("iar vio thread already stop!!\n");
	} else {
		stop_flag = 1;
		kthread_stop(g_iar_dev->iar_task);
		g_iar_dev->iar_task = NULL;
	}
	disable_sif_mclk();
	iar_pixel_clk_disable();

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
	int ret = 0;

	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	iar_enable_sif_mclk();
	ret = disp_clk_enable();
	if (ret)
		return -1;
	value = readl(g_iar_dev->regaddr + REG_IAR_DE_REFRESH_EN);
	value = IAR_REG_SET_FILED(IAR_DPI_TV_START, 0x1, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_DE_REFRESH_EN);
	//mod_timer(&iartimer,jiffies + msecs_to_jiffies( MSEC_PER_SEC));
	writel(0x1, g_iar_dev->regaddr + REG_IAR_UPDATE);
	if (display_type == MIPI_720P_TOUCH) {
		panel_exit_standby();
	} else if (display_type == LCD_7_TYPE) {
		ret = disp_pinmux_rgb();
		if (ret)
			pr_err("error pinmux rgb func!!\n");
	}

	return 0;
}
EXPORT_SYMBOL_GPL(iar_start);

int32_t iar_stop(void)
{
	uint32_t value;
	int ret = 0;

	if (NULL == g_iar_dev) {
		printk(KERN_ERR "IAR dev not inited!");
		return -1;
	}
	if (display_type == MIPI_720P_TOUCH)
		panel_enter_standby();
	value = readl(g_iar_dev->regaddr + REG_IAR_DE_REFRESH_EN);
	value = IAR_REG_SET_FILED(IAR_DPI_TV_START, 0x0, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_DE_REFRESH_EN);

	writel(0x1, g_iar_dev->regaddr + REG_IAR_UPDATE);
	if (display_type == SIF_IPI) {
		ipi_clk_disable();
	} else if (display_type == LCD_7_TYPE) {
		ret = disp_pinmux_rgb_gpio();
		if (ret)
			pr_err("erroc pinmux rgb pins to gpio func!!\n");
	}
	ret = disp_clk_disable();
	iar_disable_sif_mclk();
	//del_timer(&iartimer);
	return ret;
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
	bufaddr_channe1->Yaddr =
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr;
	bufaddr_channe1->Uaddr =
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr +
		1280*720;
	iar_set_bufaddr(IAR_CHANNEL_1, bufaddr_channe1);

	bufaddr_channe3 = &g_iar_dev->pingpong_buf[IAR_CHANNEL_3].pixel_addr[0];
	bufaddr_channe3->addr =
		g_iar_dev->pingpong_buf[IAR_CHANNEL_3].framebuf[0].paddr;
	iar_set_bufaddr(IAR_CHANNEL_3, bufaddr_channe3);

	writel(0x7ffffff, g_iar_dev->regaddr + REG_IAR_DE_SETMASK);

	return 0;
}

frame_buf_t* hobot_iar_get_framebuf_addr(int channel)
{
	if (NULL == g_iar_dev || channel < 0 || channel > IAR_CHANNEL_4) {
		printk(KERN_ERR "IAR dev not inited!");
		return NULL;
	}
	return &g_iar_dev->frambuf[channel];
}
EXPORT_SYMBOL_GPL(hobot_iar_get_framebuf_addr);
/////////////iar wb

int iar_wb_stream_on(void)
{
	//
	if (!(g_iar_dev->state & (BIT(IAR_WB_STOP) | BIT(IAR_WB_REBUFS)
			| BIT(IAR_WB_INIT)))) {
		pr_err("invalid STREAM ON is requested(%lX)", g_iar_dev->state);
		return -EINVAL;
	}

	g_iar_dev->capture_state = 1;
	g_iar_dev->state = BIT(IAR_WB_START);

	return 0;
}

int iar_wb_stream_off(void)
{
	if (!(g_iar_dev->state & BIT(IAR_WB_START))) {
		pr_err("invalid STREAMOFF is requested(%lX)", g_iar_dev->state);
		return -EINVAL;
	}

	g_iar_dev->capture_state = 0;
	frame_manager_flush(&g_iar_dev->framemgr);
	frame_manager_close(&g_iar_dev->framemgr);

	g_iar_dev->state = BIT(IAR_WB_STOP);

	return 0;
}

int iar_wb_reqbufs(u32 buffers)
{
	int ret = 0;
	struct vio_framemgr *framemgr;

	if (!(g_iar_dev->state & (BIT(IAR_WB_STOP) | BIT(IAR_WB_INIT)))) {
		pr_err("invalid REQBUFS is requested(%lX)",
			g_iar_dev->state);
		return -EINVAL;
	}
	// if (!(g_iar_dev->state & ( BIT(VIO_VIDEO_REBUFS) ))) {
	// 	pr_err("invalid REQBUFS.\n");
	// 	return -EINVAL;
	// }

	framemgr = &g_iar_dev->framemgr;
	ret = frame_manager_open(framemgr, buffers);
	if (ret) {
		pr_err("frame manage open failed, ret(%d)", ret);
		return ret;
	}

	//	for (i = 0; i < buffers; i++) {
	//		framemgr->frames[i].data = pym_ctx->group;
	//	}

	// g_iar_dev->capture_state = 0;
	g_iar_dev->state = BIT(IAR_WB_REBUFS);

	return ret;
}

int iar_wb_qbuf(struct frame_info *frameinfo)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	//struct vio_group *group;
	unsigned long flags;
	int index;

	index = frameinfo->bufferindex;
	framemgr = &g_iar_dev->framemgr;
	BUG_ON(index >= framemgr->num_frames);

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = &framemgr->frames[index];
	if (frame->state == FS_FREE) {
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_REQUEST);
	} else {
		pr_err("frame(%d) is invalid state(%d)\n", index,
			frame->state);
		ret = -EINVAL;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	// group = pym_ctx->group;
	// if(group->leader == true)
	// 	vio_group_start_trigger(group->gtask, frame);

	return ret;
}

int iar_wb_dqbuf(struct frame_info *frameinfo)
{
	int ret = 0;
	struct list_head *done_list;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;

	framemgr = &g_iar_dev->framemgr;

	done_list = &framemgr->queued_list[FS_COMPLETE];
	wait_event_interruptible(g_iar_dev->done_wq, !list_empty(done_list));

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_COMPLETE);
	if (frame) {
		memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_FREE);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return ret;
}

void iar_wb_setcfg(int value)
{
	g_iar_dev->wb_sel = (value>>8) & 0xff;
	if (g_iar_dev->wb_sel < 0) {
		g_iar_dev->wb_sel = 0;
	} else if (g_iar_dev->wb_sel > 2) {
		g_iar_dev->wb_sel = 0;
	}

	g_iar_dev->wb_format = value & 0xff;
	if (g_iar_dev->wb_format < 0) {
		g_iar_dev->wb_format = 4;
	} else if (g_iar_dev->wb_format > 6) {
		g_iar_dev->wb_format = 4;
	}

	// printk("cfg: %x, sel: %d, format: %d, wbcon: %x\n", value,
	// 	g_iar_dev->wb_sel, g_iar_dev->wb_format,
	// 	((g_iar_dev->wb_sel&0x3) << 3) + (g_iar_dev->wb_format & 0x7));
}

int iar_wb_getcfg()
{
	int value = g_iar_dev->wb_format;
	value += (g_iar_dev->wb_sel << 8);
	return value;
}

EXPORT_SYMBOL_GPL(iar_wb_reqbufs);
EXPORT_SYMBOL_GPL(iar_wb_qbuf);
EXPORT_SYMBOL_GPL(iar_wb_dqbuf);
EXPORT_SYMBOL_GPL(iar_wb_stream_off);
EXPORT_SYMBOL_GPL(iar_wb_stream_on);
EXPORT_SYMBOL_GPL(iar_wb_setcfg);
EXPORT_SYMBOL_GPL(iar_wb_getcfg);

int iar_wb_capture_start(void)
{
	//
	int ret = 0;
	//
	// REG_IAR_CURRENT_CBUF_ADDR_WR_Y  0x140
	// REG_IAR_CURRENT_CBUF_ADDR_WR_UV 0x144
	// REG_IAR_CAPTURE_CON             0x8c
	//   #[4:3] SOURCE_SEL 0:Overlay 1:UP-Scale, 2: DE_PUT(*)
    //   #[2:0] OUTPUT_FMT: 100b:NV12 110b:Unpacked RGB888
	// REG_IAR_UPDATE                  0x98
	// REG_IAR_DE_CONTROL_WO           0x31c
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	int regval = 0;
	int wbcon = ((g_iar_dev->wb_sel&0x3) << 3) + (g_iar_dev->wb_format&0x7);
	size_t frame_size_y = 0;

	framemgr = &g_iar_dev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);

	if (frame) {
		if ((g_iar_dev->wb_format & 0x7) < 0x4) {
			frame_size_y = frame->frameinfo.width * frame->frameinfo.height;
			ret = ion_check_in_heap_carveout(frame->frameinfo.addr[0],
					frame_size_y);
			if (ret) {
				pr_err("%s: wb add is not in ion pool!!\n", __func__);
				return -1;
			}
			if (!frame->frameinfo.addr[1]) {
				frame->frameinfo.addr[1] = frame->frameinfo.addr[0] +
					frame->frameinfo.width * frame->frameinfo.height;
				ret = ion_check_in_heap_carveout(frame->frameinfo.addr[1],
						frame_size_y);
				if (ret) {
					pr_err("%s: wb add is not in ion pool!!\n", __func__);
					return -1;
				}
			} else {
				ret = ion_check_in_heap_carveout(frame->frameinfo.addr[1],
						frame_size_y);
				if (ret) {
					pr_err("%s: wb add is not in ion pool!!\n", __func__);
					return -1;
				}
			}
		} else if ((g_iar_dev->wb_format & 0x7) < 0x6) {
			frame_size_y = frame->frameinfo.width * frame->frameinfo.height;
			ret = ion_check_in_heap_carveout(frame->frameinfo.addr[0],
					frame_size_y);
			if (ret) {
				pr_err("%s: wb add is not in ion pool!!\n", __func__);
				return -1;
			}
			if (!frame->frameinfo.addr[1]) {
				frame->frameinfo.addr[1] = frame->frameinfo.addr[0] +
					frame->frameinfo.width * frame->frameinfo.height;
				ret = ion_check_in_heap_carveout(frame->frameinfo.addr[1],
						frame_size_y >> 1);
				if (ret) {
					pr_err("%s: wb add is not in ion pool!!\n", __func__);
					return -1;
				}
			} else {
				ret = ion_check_in_heap_carveout(frame->frameinfo.addr[1],
						frame_size_y >> 1);
				if (ret) {
					pr_err("%s: wb add is not in ion pool!!\n", __func__);
					return -1;
				}
			}
		} else if ((g_iar_dev->wb_format & 0x7) == 0x6) {
			frame_size_y = frame->frameinfo.width * frame->frameinfo.height * 4;
			ret = ion_check_in_heap_carveout(frame->frameinfo.addr[0],
					frame_size_y);
			if (ret) {
				pr_err("%s: wb add is not in ion pool!!\n", __func__);
				return -1;
			}
			frame->frameinfo.addr[1] = frame->frameinfo.addr[0];
		}
		// printk("iar_wb_capture from request: %d\n",
		// frame->frameinfo.bufferindex);

		writel(frame->frameinfo.addr[0],
			g_iar_dev->regaddr + REG_IAR_CURRENT_CBUF_ADDR_WR_Y);
		writel(frame->frameinfo.addr[1],
			g_iar_dev->regaddr + REG_IAR_CURRENT_CBUF_ADDR_WR_UV);
		// printk("Y: %x, UV: %x\n", frame->frameinfo.addr[0],
		// frame->frameinfo.addr[1]);
		writel(wbcon, g_iar_dev->regaddr + REG_IAR_CAPTURE_CON);
		regval = readl(g_iar_dev->regaddr + REG_IAR_UPDATE);
		regval |= 0x1;
		writel(regval, g_iar_dev->regaddr + REG_IAR_UPDATE);
		// printk("REG_IAR_UPDATE: %x\n", regval);
		regval = readl(g_iar_dev->regaddr + REG_IAR_DE_CONTROL_WO);
		regval |= 0x1;
		// printk("REG_IAR_DE_CONTROL_WO: %x\n", regval);
		writel(regval, g_iar_dev->regaddr + REG_IAR_DE_CONTROL_WO);

		trans_frame(framemgr, frame, FS_PROCESS);
	} else {
		#if 0
		frame = peek_frame(framemgr, FS_COMPLETE);
		if(frame) {
			// printk("iar_wb_capture from complete: %x\n", frame);

			writel(frame->frameinfo.addr[0],
				g_iar_dev->regaddr + REG_IAR_CURRENT_CBUF_ADDR_WR_Y);
			if(frame->frameinfo.addr[1] == NULL) {
				frame->frameinfo.addr[1] = frame->frameinfo.addr[0] +
					frame->frameinfo.width * frame->frameinfo.height;
			}
			writel(frame->frameinfo.addr[1],
				g_iar_dev->regaddr + REG_IAR_CURRENT_CBUF_ADDR_WR_UV);
			// printk("Y: %x, UV: %x\n", frame->frameinfo.addr[0],
			// frame->frameinfo.addr[1]);
			writel(wbcon, g_iar_dev->regaddr + REG_IAR_CAPTURE_CON);
			regval = readl(g_iar_dev->regaddr + REG_IAR_UPDATE);
			regval |= 0x1;
			writel(regval, g_iar_dev->regaddr + REG_IAR_UPDATE);
			// printk("REG_IAR_UPDATE: %x\n", regval);
			regval = readl(g_iar_dev->regaddr + REG_IAR_DE_CONTROL_WO);
			regval |= 0x1;
			// printk("REG_IAR_DE_CONTROL_WO: %x\n", regval);
			writel(regval, g_iar_dev->regaddr + REG_IAR_DE_CONTROL_WO);

			trans_frame(framemgr, frame, FS_PROCESS);
		}
		#else
			ret = -1;
		#endif
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return ret;
}

int iar_wb_capture_done(void)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	//struct vio_group *group;
	unsigned long flags;

	// group = pym_ctx->group;
	framemgr = &g_iar_dev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		// if(group->get_timestamps){
		// 	frame->frameinfo.frame_id = group->frameid.frame_id;
		// 	frame->frameinfo.timestamps =
		// 	    group->frameid.timestamps;
		// }
		// printk("iar_wb_done: %d\n", frame->frameinfo.bufferindex);
		do_gettimeofday(&frame->frameinfo.tv);

		// pym_set_iar_output(pym_ctx, frame);
		// pym_ctx->event = VIO_FRAME_DONE;
		trans_frame(framemgr, frame, FS_COMPLETE);
	} else {
		// pym_ctx->event = VIO_FRAME_NDONE;
		// pr_err("%s PROCESS queue has no member;\n", __func__);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	wake_up(&g_iar_dev->done_wq);
	return 0;
}

int iar_output_stream_on(layer_no)
{
	//
	// if (!(g_iar_dev->state & (BIT(IAR_WB_STOP) | BIT(IAR_WB_REBUFS)
	// 		| BIT(IAR_WB_INIT)))) {
	// 	pr_err("invalid STREAM ON is requested(%lX)", g_iar_dev->state);
	// 	return -EINVAL;
	// }

	g_iar_dev->output_state[layer_no] = 1;
	// g_iar_dev->state = BIT(IAR_WB_START);

	return 0;
}
EXPORT_SYMBOL_GPL(iar_output_stream_on);

int iar_output_stream_off(layer_no)
{
	// if (!(g_iar_dev->state & BIT(IAR_WB_START))) {
	// 	pr_err("invalid STREAMOFF is requested(%lX)", g_iar_dev->state);
	// 	return -EINVAL;
	// }

	if (g_iar_dev->output_state[layer_no] == 1) {
		frame_manager_flush(&g_iar_dev->framemgr_layer[layer_no]);
		frame_manager_close(&g_iar_dev->framemgr_layer[layer_no]);
		g_iar_dev->output_state[layer_no] = 0;
	}
	//frame_manager_flush(&g_iar_dev->framemgr_layer[1]);
	// frame_manager_close(&g_iar_dev->framemgr);

	// g_iar_dev->state = BIT(IAR_WB_STOP);

	return 0;
}
EXPORT_SYMBOL_GPL(iar_output_stream_off);

int iar_output_dqbuf(int layer_no, struct frame_info *frameinfo)
{
	int ret = 0;
	struct list_head *done_list;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;

	framemgr = &g_iar_dev->framemgr_layer[layer_no];

	done_list = &framemgr->queued_list[FS_COMPLETE];
	wait_event_interruptible(
		g_iar_dev->output_done_wq[layer_no], !list_empty(done_list));

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_COMPLETE);
	if (frame) {
		memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_FREE);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return ret;
}

int iar_output_qbuf(int layer_no, struct frame_info *frameinfo)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	int index;

	index = frameinfo->bufferindex;
	framemgr = &g_iar_dev->framemgr_layer[layer_no];
	BUG_ON(index >= framemgr->num_frames);

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = &framemgr->frames[index];
	if (frame->state == FS_FREE) {
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_REQUEST);
	} else {
		pr_err("frame(%d) is invalid state(%d)\n", index,
			frame->state);
		ret = -EINVAL;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return ret;
}

int iar_output_buf_init(int layer_no, struct frame_info *frameinfo)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	int index;

	index = frameinfo->bufferindex;
	framemgr = &g_iar_dev->framemgr_layer[layer_no];
	BUG_ON(index >= framemgr->num_frames);

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = &framemgr->frames[index];
	if (frame->state == FS_FREE) {
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_COMPLETE);
	} else {
		pr_err("frame(%d) is invalid state(%d)\n", index,
			frame->state);
		ret = -EINVAL;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return ret;
}

int iar_output_reqbufs(int layer_no, u32 buffers)
{
	int ret = 0;
	struct vio_framemgr *framemgr;

	framemgr = &g_iar_dev->framemgr_layer[layer_no];
	ret = frame_manager_open(framemgr, buffers);
	if (ret) {
		pr_err("frame manage open failed, ret(%d)", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(iar_output_buf_init);
EXPORT_SYMBOL_GPL(iar_output_dqbuf);
EXPORT_SYMBOL_GPL(iar_output_qbuf);
EXPORT_SYMBOL_GPL(iar_output_reqbufs);

int iar_output_done(int layer_no)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	int ret = -1;

	// group = pym_ctx->group;
	framemgr = &g_iar_dev->framemgr_layer[layer_no];
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		// printk("iar_output_done: %d\n", frame->frameinfo.bufferindex);
		do_gettimeofday(&frame->frameinfo.tv);
		trans_frame(framemgr, frame, FS_COMPLETE);
		ret = 0;
	} else {
		// pym_ctx->event = VIO_FRAME_NDONE;
		// pr_err("%s PROCESS queue has no member;\n", __func__);
		ret = -1;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	if (ret == 0) {
		wake_up(&g_iar_dev->output_done_wq[layer_no]);
		// printk("wake up\n");
	}
	return ret;
}

int iar_output_start(int layer_no)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	// int regval = 0;
	buf_addr_t display_addr;

	framemgr = &g_iar_dev->framemgr_layer[layer_no];
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);

	if (frame) {
		// printk("iar_wb_capture from request: %d\n",
		// frame->frameinfo.bufferindex);
		int size = frame->frameinfo.width * frame->frameinfo.height;
		display_addr.Yaddr = frame->frameinfo.addr[0];
		display_addr.Uaddr = frame->frameinfo.addr[0] + size;
		display_addr.Vaddr = 0;
		iar_set_bufaddr(layer_no, &display_addr);
		iar_update();
		trans_frame(framemgr, frame, FS_PROCESS);
		// printk("output start: %dx%d, %x",
		// frame->frameinfo.width, frame->frameinfo.height, display_addr.Yaddr);
	} else {
		// printk
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return ret;
}

static irqreturn_t hobot_iar_irq(int this_irq, void *data)
{
	//struct iar_dev_s *iar = data;
	int regval = 0;
	disable_irq_nosync(this_irq);
	//TODO
	regval = readl(g_iar_dev->regaddr + REG_IAR_DE_SRCPNDREG);
	//printk("hobot_iar_irq: %x\n", regval);

	if (regval & BIT(0)) {
		writel(BIT(0), g_iar_dev->regaddr + REG_IAR_DE_SRCPNDREG);
		//frequency_iar++;
		//printk("isr 22\n");
		if (g_iar_dev->output_state[0] == 1)
			iar_output_start(0);
		if (g_iar_dev->output_state[1] == 1)
			iar_output_start(1);
	}

	if (regval & BIT(22)) {
		writel(BIT(22), g_iar_dev->regaddr + REG_IAR_DE_SRCPNDREG);
		//frequency_iar++;
		//printk("isr 22\n");
		if(g_iar_dev->capture_state == 1) {
			//printk("wb_start\n");
			int ret = iar_wb_capture_start();
			if (ret == 0) {
				g_iar_dev->capture_state = 2;
			}
		}
		if (g_iar_dev->output_state[0] == 1) {
			iar_output_done(0);
		}
		if (g_iar_dev->output_state[1] == 1) {
			iar_output_done(1);
		}
	}

	if (regval & BIT(23)) {
		//printk("isr 23");
		writel(BIT(23), g_iar_dev->regaddr + REG_IAR_DE_SRCPNDREG);
		// cbuf done
		iar_wb_capture_done();
		g_iar_dev->capture_state = 1;
	}

	IAR_DEBUG_PRINT("IAR int:0x%x", regval);
	enable_irq(this_irq);
	return IRQ_HANDLED;
}

int disable_iar_irq(void)
{
	int regval = 0;

	regval = 0xffffffff;
	writel(regval, g_iar_dev->regaddr + REG_IAR_DE_SETMASK);

	IAR_DEBUG_PRINT("hobot_iar driver: mask all interrupts!\n");
	return 0;
}

int enable_iar_irq(void)
{
	int regval = 0;
	//FBUF_END: bit22, CBUF_END: bit23
	regval = 0x00c00000;
	writel(regval, g_iar_dev->regaddr + REG_IAR_DE_UNMASK);
	IAR_DEBUG_PRINT("hobot_iar driver: unmask FBUF end interrupt!\n");

	return 0;
}

int disp_set_ppbuf_addr(uint8_t layer_no, void *yaddr, void *caddr)
{
	buf_addr_t display_addr;
	uint8_t video_index;
	uint32_t y_size;
	void __iomem *video_to_display_vaddr;
	int ret = 0;

	if (layer_no > 1)
		return -1;
	if (yaddr == NULL)
		return 0;
	y_size =
	g_iar_dev->buf_w_h[layer_no][0] * g_iar_dev->buf_w_h[layer_no][1];
	video_index = g_iar_dev->cur_framebuf_id[layer_no];
	video_to_display_vaddr =
		g_iar_dev->pingpong_buf[layer_no].framebuf[!video_index].vaddr;
	ret = copy_from_user(video_to_display_vaddr, yaddr, y_size);
	if (ret) {
		pr_err("%s: error copy y imge from user!\n", __func__);
		return ret;
	}
        ret = copy_from_user(video_to_display_vaddr + y_size,
			caddr, y_size >> 1);
	if (ret) {
		pr_err("%s: error copy uv imge from user!\n", __func__);
		return ret;
	}

	disp_copy_done = 1;

	display_addr.Yaddr =
	g_iar_dev->pingpong_buf[layer_no].framebuf[!video_index].paddr;
	display_addr.Uaddr =
	g_iar_dev->pingpong_buf[layer_no].framebuf[!video_index].paddr + y_size;
	display_addr.Vaddr = 0;

	iar_set_bufaddr(layer_no, &display_addr);
	//switch video pingpong buffer
	g_iar_dev->cur_framebuf_id[layer_no] = !video_index;
	return 0;
}
EXPORT_SYMBOL_GPL(disp_set_ppbuf_addr);

int iar_rotate_video_buffer(phys_addr_t yaddr,
		phys_addr_t uaddr, phys_addr_t vaddr)
{
	int video_index;
	buf_addr_t display_addr;
	void __iomem *video_display_vaddr;

	video_index = g_iar_dev->cur_framebuf_id[IAR_CHANNEL_1];
	video_display_vaddr =
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[!video_index].vaddr;
	do {
		struct timeval tv_s;
		struct timeval tv_e;
		int time_cost = 0;
		uint8_t *src_addr = (void *)(phys_to_virt(yaddr));
		uint8_t *tmp_addr =
			(uint8_t *)g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr;

		do_gettimeofday(&tv_s);
		preempt_disable();

		NV12ToI420Rotate(src_addr, 1280,
				src_addr + 1280*720, 1280,
				video_display_vaddr, 720,
				tmp_addr, 720/2,
				tmp_addr + 1280*720, 720/2,
				1280, 720, kRotate90);

		MergeUVPlane(tmp_addr, 720/2,
				tmp_addr + 1280*720, 720/2,
				video_display_vaddr + 1280*720, 720,
				720/2, 1280/2);
		preempt_enable();

		do_gettimeofday(&tv_e);
		time_cost = (tv_e.tv_sec*1000 + tv_e.tv_usec/1000) -
			(tv_s.tv_sec*1000 + tv_s.tv_usec/1000);
		pr_debug("time cost %dms\n", time_cost);
	} while (0);

	//display video
	display_addr.Yaddr =
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[!video_index].paddr;
	display_addr.Uaddr =
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[!video_index].paddr
	+ 720*1280;
	display_addr.Vaddr = 0;

	iar_set_bufaddr(IAR_CHANNEL_1, &display_addr);
	//switch video pingpong buffer
	g_iar_dev->cur_framebuf_id[IAR_CHANNEL_1] = !video_index;

	iar_update();

	return 0;
}

/*
#define IAR_DRAW_WIDTH	(1920)
#define IAR_DRAW_HEIGHT	(1080)
#define IAR_DRAW_PBYTE	(4)
static void hobot_iar_draw_dot(char *frame, int x, int y, int color)
{
	int pbyte = IAR_DRAW_PBYTE;

	if (x >= IAR_DRAW_WIDTH || y >= IAR_DRAW_HEIGHT)
		return;

	frame += ((y * IAR_DRAW_WIDTH) + x) * pbyte;
	while (pbyte) {
		pbyte--;
		frame[pbyte] = (color >> (pbyte * 8)) & 0xFF;
	}
}

static void hobot_iar_draw_hline(char *frame, int x0, int x1, int y, int color)
{
	int xi, xa;

	xi = (x0 < x1) ? x0 : x1;
	xa = (x0 > x1) ? x0 : x1;
	while (xi <= xa) {
		hobot_iar_draw_dot(frame, xi, y, color);
		xi++;
	}
}

static void hobot_iar_draw_vline(char *frame, int x, int y0, int y1, int color)
{
	int yi, ya;

	yi = (y0 < y1) ? y0 : y1;
	ya = (y0 > y1) ? y0 : y1;
	while (yi <= ya) {
		hobot_iar_draw_dot(frame, x, yi, color);
		yi++;
	}
}

static void hobot_iar_draw_rect(char *frame, int x0, int y0, int x1, int y1,
							int color, int fill)
{
	int xi, xa, yi, ya;

	xi = (x0 < x1) ? x0 : x1;
	xa = (x0 > x1) ? x0 : x1;
	yi = (y0 < y1) ? y0 : y1;
	ya = (y0 > y1) ? y0 : y1;
	if (fill) {
		while (yi <= ya) {
			hobot_iar_draw_hline(frame, xi, xa, yi, color);
			yi++;
		}
	} else {
		hobot_iar_draw_hline(frame, xi, xa, yi, color);
		hobot_iar_draw_hline(frame, xi, xa, ya, color);
		hobot_iar_draw_vline(frame, xi, yi, ya, color);
		hobot_iar_draw_vline(frame, xa, yi, ya, color);
	}
}
*/
int panel_hardware_reset(void)
{
	gpio_direction_output(panel_reset_pin, 1);
	pr_debug("panel reset pin output high!\n");
	msleep(100);
	gpio_direction_output(panel_reset_pin, 0);
	pr_debug("panel reset pin output low!\n");
	msleep(120);
	gpio_direction_output(panel_reset_pin, 1);
	pr_debug("panel reset pin output high!\n");
	msleep(120);
	pr_info("panel reset success!\n");

	return 0;
}
EXPORT_SYMBOL_GPL(panel_hardware_reset);

int get_iar_module_rst_pin(void)
{
	if (rst_request_flag == 0)
		return -1;
	return panel_reset_pin;
}
EXPORT_SYMBOL_GPL(get_iar_module_rst_pin);


int enable_sif_mclk(void)
{
	int ret = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (g_iar_dev->sif_mclk == NULL) {
		pr_err("%s: sif_mclk is null!!\n", __func__);
		return -1;
	}
	ret = clk_prepare_enable(g_iar_dev->sif_mclk);
	if (ret != 0) {
		pr_err("%s: failed to prepare sif_mclk!!\n", __func__);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(enable_sif_mclk);

int disable_sif_mclk(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (g_iar_dev->sif_mclk == NULL) {
		pr_err("%s: sif_mclk is null!!\n", __func__);
		return -1;
	}
	clk_disable_unprepare(g_iar_dev->sif_mclk);
	return 0;
}
EXPORT_SYMBOL_GPL(disable_sif_mclk);


static int hobot_iar_probe(struct platform_device *pdev)
{
	struct resource *res, *irq, *res_mipi;
	phys_addr_t mem_paddr;
	size_t mem_size;
	int ret = 0;
	struct device_node *np;
	struct resource r;
	char *temp1;
	int tempi = 0;
	void *vaddr;
	uint64_t pixel_rate;
	char *type;
	void __iomem *hitm1_reg_addr;
        uint32_t reg_val = 0;
	uint32_t i = 0;
	buf_addr_t channel_buf_addr_3;
	buf_addr_t channel_buf_addr_4;

	pr_info("iar probe begin!!!\n");

	g_iar_dev = devm_kzalloc(&pdev->dev, sizeof(struct iar_dev_s), GFP_KERNEL);
	if (!g_iar_dev) {
		dev_err(&pdev->dev, "Unable to alloc IAR DEV\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, g_iar_dev);
	spin_lock_init(&g_iar_dev->spinlock);
	g_iar_dev->lock = &g_iar_dev->spinlock;

	spin_lock_init(&g_iar_dev->framemgr.slock);
	for (i = 0; i < IAR_CHANNEL_MAX; i++)
		spin_lock_init(&g_iar_dev->framemgr_layer[i].slock);

	g_iar_dev->pdev = pdev;
	memset(&g_iar_dev->cur_framebuf_id, 0, IAR_CHANNEL_MAX * sizeof(int));

	g_iar_dev->sif_mclk = devm_clk_get(&pdev->dev, "sif_mclk");
        if (IS_ERR(g_iar_dev->sif_mclk)) {
                dev_err(&pdev->dev, "failed to get sif_mclk\n");
                return PTR_ERR(g_iar_dev->sif_mclk);
        }
	sif_mclk_is_open = __clk_is_enabled(g_iar_dev->sif_mclk);
	iar_enable_sif_mclk();
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	g_iar_dev->regaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(g_iar_dev->regaddr))
		return PTR_ERR(g_iar_dev->regaddr);
#ifdef CONFIG_HOBOT_XJ3
	res_mipi = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	g_iar_dev->mipi_dsi_regaddr =
		devm_ioremap_resource(&pdev->dev, res_mipi);
	if (IS_ERR(g_iar_dev->mipi_dsi_regaddr))
		return PTR_ERR(g_iar_dev->mipi_dsi_regaddr);
	hitm1_reg_addr = ioremap_nocache(X3_GPIO_BASE + X3_GPIO0_VALUE_REG, 4);
	reg_val = readl(hitm1_reg_addr);
	reg_val = (((reg_val >> 14) & 0x1) << 1) | ((reg_val >> 12) & 0x1);
	hb_disp_base_board_id = reg_val + 1;
	pr_debug("iar: base board id is 0x%x\n", hb_disp_base_board_id);
	if (hb_disp_base_board_id == 0x1) {     //x3_dvb
		ret = screen_backlight_init();
		hitm1_reg_addr = ioremap_nocache(X3_PWM0_PINMUX, 4);
		writel(0x2, hitm1_reg_addr);
		if (ret)
			pr_err("%s: error init pwm0!!\n", __func__);
		pr_info("iar: pwm0 init success!!\n");
		ret = of_property_read_u32(pdev->dev.of_node,
			"disp_panel_reset_pin", &panel_reset_pin);
		if (ret) {
			dev_err(&pdev->dev, "Filed to get panel_reset_pin\n");
		} else {
			ret = gpio_request(panel_reset_pin, "disp_panel_reset_pin");
			if (ret) {
				pr_err("%s() Err get trigger pin ret= %d\n",
					__func__, ret);
				//return -ENODEV;
			} else {
				rst_request_flag = 1;
			}
		}
		pr_debug("gpio request succeed!!!!\n");
	}
#endif
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
		goto err1;
	}
	g_iar_dev->irq = irq->start;
	pr_debug("g_iar_dev->irq is %lld\n", irq->start);

	//return 0;
	g_iar_dev->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(g_iar_dev->pinctrl)) {
		dev_warn(&pdev->dev, "pinctrl get none\n");
		g_iar_dev->pinctrl = NULL;
		g_iar_dev->pins_bt1120 = NULL;
		g_iar_dev->pins_bt656 = NULL;
		g_iar_dev->pins_mipi_dsi = NULL;
		g_iar_dev->pins_rgb = NULL;
	} else {
		g_iar_dev->pins_bt1120 =
			pinctrl_lookup_state(g_iar_dev->pinctrl, "bt_func");
		if (IS_ERR(g_iar_dev->pins_bt1120)) {
			dev_info(&pdev->dev, "bt1120_func get error %ld\n",
					PTR_ERR(g_iar_dev->pins_bt1120));
			g_iar_dev->pins_bt1120 = NULL;
		}
		g_iar_dev->pins_bt656 = pinctrl_lookup_state(g_iar_dev->pinctrl,
					"bt656_func");
		if (IS_ERR(g_iar_dev->pins_bt656)) {
			dev_info(&pdev->dev, "bt656_func get error %ld\n",
					PTR_ERR(g_iar_dev->pins_bt656));
			g_iar_dev->pins_bt656 = NULL;
		}
		g_iar_dev->pins_mipi_dsi =
		pinctrl_lookup_state(g_iar_dev->pinctrl, "mipi_dsi_func");
		if (IS_ERR(g_iar_dev->pins_mipi_dsi)) {
			dev_info(&pdev->dev, "mipi_dsi_func get error %ld\n",
					PTR_ERR(g_iar_dev->pins_mipi_dsi));
			g_iar_dev->pins_mipi_dsi = NULL;
		}
		g_iar_dev->pins_rgb = pinctrl_lookup_state(g_iar_dev->pinctrl,
					"rgb_func");
		if (IS_ERR(g_iar_dev->pins_rgb)) {
			dev_info(&pdev->dev, "rgb_func get error %ld\n",
					PTR_ERR(g_iar_dev->pins_rgb));
			g_iar_dev->pins_rgb = NULL;
		}
		g_iar_dev->pins_rgb_gpio = pinctrl_lookup_state(g_iar_dev->pinctrl,
					"rgb_gpio_func");
		if (IS_ERR(g_iar_dev->pins_rgb_gpio)) {
			dev_warn(&pdev->dev, "rgb_gpio_func get error %ld\n",
					PTR_ERR(g_iar_dev->pins_rgb_gpio));
			g_iar_dev->pins_rgb_gpio = NULL;
		}
	}

	g_iar_dev->iar_ipi_clk = devm_clk_get(&pdev->dev, "iar_ipi_clk");
	if (IS_ERR(g_iar_dev->iar_ipi_clk)) {
		dev_warn(&pdev->dev, "failed to get iar_ipi_clk\n");
		g_iar_dev->iar_ipi_clk = NULL;
	}

	g_iar_dev->iar_pixel_clk = devm_clk_get(&pdev->dev, "iar_pix_clk");
	if (IS_ERR(g_iar_dev->iar_pixel_clk)) {
		dev_err(&pdev->dev, "failed to get iar_pix_clk\n");
		return PTR_ERR(g_iar_dev->iar_pixel_clk);
	}
	clk_prepare_enable(g_iar_dev->iar_pixel_clk);

	pixel_rate = clk_get_rate(g_iar_dev->iar_pixel_clk);
	pr_debug("%s: iar pixel rate is %lld\n", __func__, pixel_rate);

	init_waitqueue_head(&g_iar_dev->wq_head);
	ret = request_threaded_irq(g_iar_dev->irq, hobot_iar_irq, NULL, IRQF_TRIGGER_HIGH,
							   dev_name(&pdev->dev), g_iar_dev);
	disable_irq(g_iar_dev->irq);
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&g_iar_dev->pdev->dev, "No %s specified\n", "memory-region");
#ifndef USE_ION_MEM
		goto err1;
#endif
	} else {
		ret = of_address_to_resource(np, 0, &r);
		if (ret) {
			dev_err(&pdev->dev, "No memory address assigned to the region\n");
			goto err1;
		}

		pr_debug("iar reserved memory size is %lld\n", resource_size(&r));
		if (resource_size(&r) < MAX_FRAME_BUF_SIZE) {
			pr_debug("iar logo memory size is not large enough!(<1buffer)\n");
			//return -1;
		} else {
			logo_paddr = r.start;
			logo_vaddr = ioremap_nocache(r.start, MAX_FRAME_BUF_SIZE);
		}
	}
#ifdef USE_ION_MEM
	if (!hb_ion_dev) {
		dev_err(&pdev->dev, "NO ION device found!!");
		goto err1;
	}

	g_iar_dev->iar_iclient = ion_client_create(hb_ion_dev, "iar");
	if (IS_ERR(g_iar_dev->iar_iclient)) {
		dev_err(&pdev->dev, "Create iar ion client failed!!");
		ret = -ENOMEM;
		goto err1;
	}

#ifdef CONFIG_HOBOT_XJ2
	g_iar_dev->iar_ihandle = ion_alloc(g_iar_dev->iar_iclient,
		IAR_MEM_SIZE - MAX_FRAME_BUF_SIZE, 0x20,
		ION_HEAP_CARVEOUT_MASK, 0);
#else
	if (logo_vaddr == NULL) {
		g_iar_dev->iar_ihandle = ion_alloc(g_iar_dev->iar_iclient,
			IAR_MEM_SIZE, 0x20,
			ION_HEAP_CARVEOUT_MASK, 0);
		pr_debug("iar request 64M ion memory region!\n");
	} else {
		g_iar_dev->iar_ihandle = ion_alloc(g_iar_dev->iar_iclient,
			IAR_MEM_SIZE - MAX_FRAME_BUF_SIZE, 0x20,
			ION_HEAP_CARVEOUT_MASK, 0);
		pr_debug("iar request 56M ion memory region!\n");
	}
#endif
	if (!g_iar_dev->iar_ihandle || IS_ERR(g_iar_dev->iar_ihandle)) {
		dev_err(&pdev->dev, "Create iar ion client failed!!");
		goto err1;
	}

	ret = ion_phys(g_iar_dev->iar_iclient, g_iar_dev->iar_ihandle->id,
			&mem_paddr, &mem_size);

	if (ret) {
		dev_err(&pdev->dev, "Get buffer paddr failed!!");
		ion_free(g_iar_dev->iar_iclient, g_iar_dev->iar_ihandle);
		goto err2;
	}
	vaddr = ion_map_kernel(g_iar_dev->iar_iclient,
			g_iar_dev->iar_ihandle);

	if (IS_ERR(vaddr)) {
		dev_err(&pdev->dev, "Get buffer paddr failed!!");
		ion_free(g_iar_dev->iar_iclient, g_iar_dev->iar_ihandle);
		goto err2;
	}
#else
	#ifdef CONFIG_HOBOT_XJ3
	if (resource_size(&r) < MAX_FRAME_BUF_SIZE*8) {
                pr_err("iar memory size is not large enough!(<3buffer)\n");
                goto err1;
        }
        mem_paddr = r.start;
        //for reserved 32M(4*8) memory,
        //three buffers for video, one buffer for graphic(fb)

        //channel 2&4 disabled
        vaddr = ioremap_nocache(r.start, MAX_FRAME_BUF_SIZE * 8);
	#else
	if (resource_size(&r) < MAX_FRAME_BUF_SIZE*4) {
		pr_err("iar memory size is not large enough!(<3buffer)\n");
		goto err1;
	}
	mem_paddr = r.start;

	//for reserved 32M(4*8) memory,
	//three buffers for video, one buffer for graphic(fb)

	//channel 2&4 disabled
	vaddr = ioremap_nocache(r.start, MAX_FRAME_BUF_SIZE * 4);
	#endif
#endif

#ifdef USE_ION_MEM
#ifdef CONFIG_HOBOT_XJ3
	g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = mem_paddr;
	g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = vaddr;
	g_iar_dev->frambuf[IAR_CHANNEL_4].paddr =
		mem_paddr + MAX_FRAME_BUF_SIZE;
	g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr =
		vaddr + MAX_FRAME_BUF_SIZE;
	if (logo_vaddr == NULL) {
		g_iar_dev->frambuf[IAR_CHANNEL_1].paddr =
			mem_paddr + MAX_FRAME_BUF_SIZE * 7;
		g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr =
			vaddr + MAX_FRAME_BUF_SIZE * 7;
	} else {
		g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = logo_paddr;
		g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = logo_vaddr;
	}
	g_iar_dev->frambuf[IAR_CHANNEL_2].paddr =
		mem_paddr + MAX_FRAME_BUF_SIZE * 2;
	g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr =
		vaddr + MAX_FRAME_BUF_SIZE * 2;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr =
				mem_paddr + MAX_FRAME_BUF_SIZE * 3;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr =
				vaddr + MAX_FRAME_BUF_SIZE * 3;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr
			= mem_paddr + MAX_FRAME_BUF_SIZE * 4;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr
			= vaddr + MAX_FRAME_BUF_SIZE * 4;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr =
				mem_paddr + MAX_FRAME_BUF_SIZE * 5;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr =
				vaddr + MAX_FRAME_BUF_SIZE * 5;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].paddr
				= mem_paddr + MAX_FRAME_BUF_SIZE * 6;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].vaddr
				= vaddr + MAX_FRAME_BUF_SIZE * 6;

	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_3].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_4].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_1].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_2].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_2].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].vaddr);

#else
	g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = logo_paddr;
	g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = logo_vaddr;
	g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = mem_paddr;
	g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = vaddr;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr =
				mem_paddr + MAX_FRAME_BUF_SIZE;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr =
				vaddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr
			= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr
			= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr + MAX_FRAME_BUF_SIZE;

	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_1].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_3].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);

	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr);
#endif
#else
#ifdef CONFIG_HOBOT_XJ3
	g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = mem_paddr;
	g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = vaddr;
	g_iar_dev->frambuf[IAR_CHANNEL_4].paddr =
		mem_paddr + MAX_FRAME_BUF_SIZE;
	g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr =
		vaddr + MAX_FRAME_BUF_SIZE;
	g_iar_dev->frambuf[IAR_CHANNEL_1].paddr =
		mem_paddr + MAX_FRAME_BUF_SIZE * 2;
	g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr =
		vaddr + MAX_FRAME_BUF_SIZE * 2;
	g_iar_dev->frambuf[IAR_CHANNEL_2].paddr =
		mem_paddr + MAX_FRAME_BUF_SIZE * 3;
	g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr =
		vaddr + MAX_FRAME_BUF_SIZE * 3;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr =
		mem_paddr + MAX_FRAME_BUF_SIZE * 4;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr =
		vaddr + MAX_FRAME_BUF_SIZE * 4;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr =
		mem_paddr + MAX_FRAME_BUF_SIZE * 5;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr =
		vaddr + MAX_FRAME_BUF_SIZE * 5;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr =
		mem_paddr + MAX_FRAME_BUF_SIZE * 6;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr =
		vaddr + MAX_FRAME_BUF_SIZE * 6;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].paddr =
		mem_paddr + MAX_FRAME_BUF_SIZE * 7;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].vaddr =
		vaddr + MAX_FRAME_BUF_SIZE * 7;

	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_3].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_4].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_1].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_2].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_2].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr);

	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].vaddr);
#else
	g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = mem_paddr;
	g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = vaddr;
	g_iar_dev->frambuf[IAR_CHANNEL_3].paddr =
				mem_paddr + MAX_FRAME_BUF_SIZE;
	g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = vaddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr =
				mem_paddr + MAX_FRAME_BUF_SIZE * 2;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr =
				vaddr + MAX_FRAME_BUF_SIZE * 2;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr
		= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr +
			MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr
		= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr +
			MAX_FRAME_BUF_SIZE;

	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_1].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = 0x%llx\n",
				g_iar_dev->frambuf[IAR_CHANNEL_3].paddr);
	pr_debug("g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = 0x%p\n",
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);

	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr = 0x%llx\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr);
	pr_debug("g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr = 0x%p\n",
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr);
#endif
#endif
	ret = fb_get_options("hobot", &type);
	pr_debug("%s: fb get options display type is %s\n", __func__, type);
	if (type != NULL) {
#ifdef CONFIG_HOBOT_XJ3
		if (strncmp(type, "1080p", 5) == 0) {
			display_type = MIPI_1080P;
			pr_info("%s: panel type is MIPI_1080P\n", __func__);
		} else if (strncmp(type, "720p", 4) == 0) {
			display_type = MIPI_720P_TOUCH;
			pr_info("%s: panel type is MIPI_720P_TOUCH\n", __func__);
		} else if (strncmp(type, "lcd", 3) == 0) {
			display_type = LCD_7_TYPE;
			pr_info("%s: panel type is LCD_7_TYPE\n", __func__);
		} else if (strncmp(type, "hdmi", 4) == 0) {
			display_type = HDMI_TYPE;
			pr_info("%s: panel type is HDMI_TYPE\n", __func__);
		} else if (strncmp(type, "ipi", 3) == 0) {
			pr_info("%s: panel type is SIF IPI\n", __func__);
			display_type = SIF_IPI;
		} else {
			pr_err("wrong panel type!!!\n");
		}
#else
		if (display_type == LCD_7_TYPE) {
			if (strncmp(type, "mipi", 4) == 0)
				display_type = MIPI_720P;
		}
#endif
	}
	if (display_type == LCD_7_TYPE) {
		iar_display_cam_no = PIPELINE0;
		iar_display_addr_type = DISPLAY_CHANNEL1;
		temp1 = g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr;
		tempi = 0;
		for (i = 0; i < 800 * 4 * 40; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0xff;//B
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 800 * 4 * 40; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0xff;//G
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 800 * 4 * 40; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0xff;//R
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 800 * 4 * 40; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;//A
			}
			temp1++;
		}
		for (i = 0; i < 800 * 4 * 320; i++) {
			*temp1 = 0x0;
			temp1++;
		}
	} else if (display_type == MIPI_720P) {
		pr_debug("%s: display_type is mipi-720p-dsi panel!\n", __func__);
		ret = disp_set_pixel_clk(68000000);
		if (ret)
			return ret;
	} else if (display_type == MIPI_720P_TOUCH) {
		pr_info("%s: display_type is mipi-720p-dsi panel!\n", __func__);
		ret = disp_set_pixel_clk(54000000);
		if (ret)
			return ret;
		iar_display_cam_no = PIPELINE0;
                iar_display_addr_type = GDC0;
		temp1 = g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr;
		tempi = 0;
		for (i = 0; i < 720 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0xff;//B
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 720 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0xff;//g
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 720 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0xff;//r
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 720 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;//A
			}
			temp1++;
		}
		for (i = 0; i < 720 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0xff;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0xff;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0xff;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;//
			}
			temp1++;
		}
		for (i = 0; i < 720 * 4 * 780; i++) {
			*temp1 = 0x0;
			temp1++;
		}
		pr_debug("set mipi 720p touch done!\n");
	} else if (display_type == HDMI_TYPE || display_type == SIF_IPI) {
		if (display_type == SIF_IPI) {
			pr_debug("%s: display_type is SIF_IPI panel!\n", __func__);
			ret = ipi_clk_enable();
			if (ret) {
				pr_err("ipi_clk_enable failed %d\n", ret);
				return ret;
			}
			ret = disp_set_pixel_clk(54400000);
		} else {
			pr_debug("%s: display_type is HDMI panel!\n", __func__);
			ret = disp_set_pixel_clk(163000000);
		}
		iar_display_cam_no = PIPELINE0;
                iar_display_addr_type = DISPLAY_CHANNEL1;
		if (ret)
			return ret;
		temp1 = g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr;
                tempi = 0;
		for (i = 0; i < 1920 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0xff;//B
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 1920 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0xff;//g
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 1920 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0xff;//r
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 1920 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;//A
			}
			temp1++;
		}
		for (i = 0; i < 1920 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0xff;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0xff;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0xff;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;//
			}
			temp1++;
		}
		for (i = 0; i < 1920 * 4 * 580; i++) {
			*temp1 = 0x0;
			temp1++;
		}
		pr_debug("set HDMI done!\n");
#if 0
		temp1 = g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr;
		#define IAR_DRAW_X(c, p)	(IAR_DRAW_WIDTH * c / p)
		#define IAR_DRAW_XL(c, p)	((IAR_DRAW_WIDTH * c / p) - 1)
		#define IAR_DRAW_Y(c, p)	(IAR_DRAW_HEIGHT * c / p)
		#define IAR_DRAW_YL(c, p)	((IAR_DRAW_HEIGHT * c / p) - 1)
		// color: [0:3]-BGRA -> 0x[A][R][G][B]
		// white all.
		deta = 0xFFFFFFFF;
		hobot_iar_draw_rect(temp1, IAR_DRAW_X(0, 1), IAR_DRAW_Y(0, 1),
			IAR_DRAW_XL(1, 1), IAR_DRAW_YL(1, 1), deta, 1);
		// blue rect +1/10.
		deta = 0xFF0000FF;
		hobot_iar_draw_rect(temp1, IAR_DRAW_X(1, 10), IAR_DRAW_Y(1, 10),
			IAR_DRAW_XL(9, 10), IAR_DRAW_YL(9, 10), deta, 1);
		// green rect +1/10.
		deta = 0xFF00FF00;
		hobot_iar_draw_rect(temp1, IAR_DRAW_X(2, 10), IAR_DRAW_Y(2, 10),
			IAR_DRAW_XL(8, 10), IAR_DRAW_YL(8, 10), deta, 1);
		// red rect +1/10.
		deta = 0xFFFF0000;
		hobot_iar_draw_rect(temp1, IAR_DRAW_X(3, 10), IAR_DRAW_Y(3, 10),
			IAR_DRAW_XL(7, 10), IAR_DRAW_YL(7, 10), deta, 1);
		// black rect +1/10.
		deta = 0xFF000000;
		hobot_iar_draw_rect(temp1, IAR_DRAW_X(4, 10), IAR_DRAW_Y(4, 10),
			IAR_DRAW_XL(6, 10), IAR_DRAW_YL(6, 10), deta, 1);
#endif
	} else if (display_type == MIPI_1080P) {
		pr_debug("iar_driver: disp set mipi 1080p!\n");
		pr_debug("iar_driver: output 1080*1920 color bar bgr!\n");
		disp_set_pixel_clk(32000000);
		iar_display_cam_no = PIPELINE0;
		iar_display_addr_type = GDC1;
		// actual output 27.2Mhz(need 27Mhz)
		temp1 = g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr;
		tempi = 0;
		for (i = 0; i < 1080 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0xff;//B
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 1080 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0xff;//b
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 1080 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0xff;//g
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;
			}
			temp1++;
		}
		for (i = 0; i < 1080 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0x00;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;//A
			}
			temp1++;
		}
		for (i = 0; i < 1080 * 4 * 100; i++) {
			if (((i + 4) % 4) == 0) {
				*temp1 = 0xff;
			} else if (((i + 4) % 4) == 1) {
				*temp1 = 0xff;
			} else if (((i + 4) % 4) == 2) {
				*temp1 = 0xff;
			} else if (((i + 4) % 4) == 3) {
				*temp1 = 0xff;//
			}
			temp1++;
		}
		for (i = 0; i < 1080 * 4 * 1420; i++) {
			*temp1 = 0x0;
			temp1++;
		}
		pr_debug("set mipi 1080p done!\n");
	}
	channel_buf_addr_3.addr = g_iar_dev->frambuf[IAR_CHANNEL_3].paddr;
	channel_buf_addr_4.addr = g_iar_dev->frambuf[IAR_CHANNEL_4].paddr;
	iar_switch_buf(0);
	iar_set_bufaddr(IAR_CHANNEL_3, &channel_buf_addr_3);
	iar_set_bufaddr(IAR_CHANNEL_4, &channel_buf_addr_4);
	iar_update();
	clk_disable_unprepare(g_iar_dev->iar_pixel_clk);
	iar_disable_sif_mclk();
	pr_info("iar probe end success!!!\n");
	return 0;
err1:
	devm_kfree(&pdev->dev, g_iar_dev);
	return -1;
err2:
#ifdef USE_ION_MEM
	ion_client_destroy(g_iar_dev->iar_iclient);
#endif
	return ret;
}

static int hobot_iar_remove(struct platform_device *pdev)
{
	struct iar_dev_s *iar;

	iar = dev_get_drvdata(&pdev->dev);

	return 0;
}

/*int iar_is_enabled(void)
 *{
 *	//IAR NOT enabled cases as follow:
 *	// 1.disabled in dts not probe
 *
 *	if (g_iar_dev == NULL)
 *		return 0;
 *	// 2.disabled dynamic
 *
 *	//enabled case
 *	return 1;
 *}
 *EXPORT_SYMBOL_GPL(iar_is_enabled);
 */

#ifdef CONFIG_OF
static const struct of_device_id hobot_iar_of_match[] = {
	{.compatible = "hobot,hobot-iar"},
	{},
};
MODULE_DEVICE_TABLE(of, hobot_iar_of_match);
#endif

//#ifdef CONFIG_PM
int hobot_iar_suspend(struct device *dev)
{
	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

	iar_regs_store();

	return 0;
}

int hobot_iar_resume(struct device *dev)
{
	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);

	iar_regs_restore();

	return 0;
}
//#endif

static const struct dev_pm_ops hobot_iar_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(hobot_iar_suspend,
			hobot_iar_resume)
};

static struct platform_driver hobot_iar_driver = {
	.probe = hobot_iar_probe,
	.remove = hobot_iar_remove,
	.driver = {
		.name = "hobot-iar",
		.of_match_table = of_match_ptr(hobot_iar_of_match),
		.pm = &hobot_iar_pm,
	},
};
/*
static int __init hobot_iar_init(void)
{
        int ret;

    ret = platform_driver_register(&hobot_iar_driver);
    if (ret)
        pr_err("register hobot_ips_driver error\n");

        return ret;
}

fs_initcall(hobot_iar_init);
*/
module_platform_driver(hobot_iar_driver);
MODULE_LICENSE("GPL");
