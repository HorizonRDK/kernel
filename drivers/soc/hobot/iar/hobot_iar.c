/***************************************************************************
 *						COPYRIGHT NOTICE
 *			   Copyright 2018 Horizon Robotics, Inc.
 *					   All rights reserved.
 ***************************************************************************/

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/string.h>
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
#include <asm/unaligned.h>
#include <soc/hobot/hobot_ips_x2.h>
#include <soc/hobot/hobot_iar.h>
#include "linux/ion.h"
#include "../../../media/platform/hobot/common_api/vio_framemgr.h"
#include "./bmp_layout.h"

#define USE_ION_MEM
#ifdef CONFIG_HOBOT_XJ3
#define IAR_MEM_SIZE 0x4000000	//64MB
#define MAX_YUV_BUF_SIZE 0x2F7600 //3MB,1920*1080*1.5
#else
#define IAR_MEM_SIZE 0x2000000        //32MB
#endif

#define FORMAT_ORGANIZATION_VAL 0x9c36
#define REFRESH_CFG_VAL 0x808
unsigned int iar_debug_level = 0;
EXPORT_SYMBOL(iar_debug_level);
module_param(iar_debug_level, uint, 0644);

unsigned int video_layer_num = 2;
unsigned int display_out_width = 1920;
unsigned int display_out_height = 1080;
unsigned int fb_num = 1;
unsigned int logo = 0;
EXPORT_SYMBOL(fb_num);
EXPORT_SYMBOL(logo);
module_param(video_layer_num, uint, 0644);
module_param(display_out_width, uint, 0644);
module_param(display_out_height, uint, 0644);
module_param(fb_num, uint, 0644);
module_param(logo, uint, 0644);
static unsigned int frame_input_fps = 30;
static unsigned int frame_output_fps = 30;
module_param(frame_input_fps, uint, 0644);
module_param(frame_output_fps, uint, 0644);
static int frame_output = 0;
#define IAR_ENABLE 1
#define IAR_DISABLE 0
#define IARIONTYPE 13
#define DISPLAY_TYPE_TOTAL_SINGLE 33
#define DISPLAY_TYPE_TOTAL_MULTI 63

#define EMBEDIMG(_index, _path)                                   \
	__asm__(".section \".rodata\", \"a\"\n\t"       \
		"\nembedded_image_" #_index "_data:\n\t"              \
		".incbin \"" _path "\"\n\t"                           \
		"\nembedded_image_" #_index "_end:\n\t"               \
		".equ embedded_image_" #_index "_len, "               \
		"(embedded_image_" #_index "_end - "           \
		"  embedded_image_" #_index "_data)\n\t"       \
		".previous\n\t");

extern const char embedded_image_0_data[];

int panel_reset_pin = -1;
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
uint32_t iar_display_addr_type = DISPLAY_CHANNEL1;
uint32_t iar_display_cam_no = PIPELINE0;
uint32_t iar_display_addr_type_video1 = 0;
uint32_t iar_display_cam_no_video1 = PIPELINE0;
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

bool iar_video_not_pause = true;
EXPORT_SYMBOL_GPL(iar_video_not_pause);

uint32_t config_rotate;
uint8_t ipu_process_done;
uint32_t disp_user_update;
uint32_t disp_user_update_video1;
uint8_t frame_count;
uint8_t disp_copy_done = 0;
static int disp_clk_already_enable = 0;
static int ipi_clk_already_enable = 0;
static int sif_mclk_is_open = 0;
static int sif_mclk_iar_open = 0;
static int stop_flag = 0;
phys_addr_t logo_paddr;
void *logo_vaddr = NULL;
static int pwm0_request_status = 0;
static int pwm_period = 1000;
static int pwm_duty = 10;
static int bt656_output_pin_group = 0;// default output through low 8bit
static int bt1120_clk_invert = 0;// default bt clk not invert

struct disp_timing video_1920x1080 = {
	148, 88, 44, 36, 4, 5, 10
};

struct disp_timing video_1280x720 = {
	220, 150, 40, 20, 10, 5, 0
};

struct disp_timing video_800x480 = {
	80, 120, 48, 32, 43, 2, 10
};

struct disp_timing video_720x1280 = {
	36, 84, 24, 11, 13, 2, 10
};

struct disp_timing video_1080x1920 = {
	100, 400, 5, 4, 400, 5, 10
};
struct disp_timing video_720x1280_touch = {
	100, 100, 10, 20, 10, 10, 10
};
struct disp_timing video_704x576 = {
	288, 0, 0, 22, 2, 0, 0
};
struct disp_timing video_720x480 = {
	276, 0, 0, 19, 3, 0, 0
};

uint32_t pixel_clk_video_1920x1080 = 163000000;
uint32_t pixel_clk_video_1280x720 = 74250000;
uint32_t pixel_clk_video_800x480 = 32000000;
uint32_t pixel_clk_video_720x1280 = 68000000;
uint32_t pixel_clk_video_1080x1920 = 32000000;
uint32_t pixel_clk_video_720x1280_touch = 54400000;
uint32_t pixel_clk_video_704x576 = 27000000;
uint32_t pixel_clk_video_720x480 = 27000000;
EXPORT_SYMBOL(disp_user_config_done);
EXPORT_SYMBOL(disp_copy_done);
EXPORT_SYMBOL(video_1920x1080);
EXPORT_SYMBOL(video_1280x720);
EXPORT_SYMBOL(video_800x480);
EXPORT_SYMBOL(video_720x1280);
EXPORT_SYMBOL(video_1080x1920);
EXPORT_SYMBOL(video_720x1280_touch);
EXPORT_SYMBOL(video_704x576);
EXPORT_SYMBOL(video_720x480);

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
module_param(display_type, int, 0644);

static void iar_regs_store(void)
{
	void __iomem *regaddr;
	int i;

	if (g_iar_dev == NULL) {
		pr_info("%s:%s, g_iar_dev=NULL\n", __FILE__, __func__);
		return;
	}

	if (enable_sif_mclk() != 0)
		return;
	if (iar_pixel_clk_enable() != 0)
		return;
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
	disable_sif_mclk();
	iar_pixel_clk_disable();
}

static void iar_regs_restore(void)
{
	void __iomem *regaddr;
	int i;

	if (g_iar_dev == NULL) {
		pr_info("%s:%s, g_iar_dev=NULL\n", __FILE__, __func__);
		return;
	}

	if (enable_sif_mclk() != 0)
		return;
	if (iar_pixel_clk_enable() != 0)
		return;
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
	disable_sif_mclk();
	iar_pixel_clk_disable();
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

/**
 * Set iar module pixel clock phase invert
 * only used in XJ2 platform
 */
static void set_iar_pixel_clk_inv(void)
{
	void __iomem *regaddr;
	uint32_t reg_val = 0;

	regaddr = ioremap_nocache(0xA1000000 + 0x310, 4);
	reg_val = readl(regaddr);
	reg_val = reg_val | 0x1000;
	writel(reg_val, regaddr);
	iounmap(regaddr);
}

buf_addr_t iar_addr_convert(phys_addr_t paddr)
{
	buf_addr_t addr;
	addr.addr = (uint32_t)paddr;
	return addr;
}

/**
 * Set output interlace mode
 * only used for bt1120 output mode
 */
int disp_set_interlace_mode(void)
{
	uint32_t value;
	uint32_t height;

	if (NULL == g_iar_dev) {
		pr_err(KERN_ERR "IAR dev not inited!");
		return -1;
	}

	value = readl(g_iar_dev->regaddr + REG_IAR_PANEL_SIZE);
	if ((value & 0x7ff) == 0 || (value & 0x7ff0000) == 0)
		pr_err("%s: user not config output!!!\n", __func__);
	height = (value & 0x7ff0000) >> 17;//>>16 /2
	value = IAR_REG_SET_FILED(IAR_PANEL_HEIGHT, height, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PANEL_SIZE);

	writel(0, g_iar_dev->regaddr + REG_IAR_PARAMETER_VFP_CNT_FIELD12);
	value = readl(g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
	value = IAR_REG_SET_FILED(IAR_YCBCR_OUTPUT, 1, value);
	value = IAR_REG_SET_FILED(IAR_INTERLACE_SEL, 1, value);
	value = IAR_REG_SET_FILED(IAR_PANEL_COLOR_TYPE, 2, value);//yuv444
	writel(value, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);

	value = readl(g_iar_dev->regaddr + REG_IAR_DE_CONTROL_WO);
	value = IAR_REG_SET_FILED(IAR_FIELD_ODD_CLEAR, 1, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_DE_CONTROL_WO);

	return 0;
}
EXPORT_SYMBOL_GPL(disp_set_interlace_mode);

int disp_set_display_crop(struct position_cfg_t *crop_cfg)
{
	if (NULL == g_iar_dev) {
		pr_err(KERN_ERR "IAR dev not inited!");
		return -1;
	}

	writel(crop_cfg->y << 16 | crop_cfg->x,
		g_iar_dev->regaddr + REG_IAR_CROPPED_WINDOW_RD1
		- crop_cfg->layer * 4);
	return 0;
}
EXPORT_SYMBOL_GPL(disp_set_display_crop);

int disp_set_display_position(struct position_cfg_t *position_cfg)
{
	if (NULL == g_iar_dev) {
		pr_err(KERN_ERR "IAR dev not inited!");
		return -1;
	}

	writel(position_cfg->y << 16 | position_cfg->x,
		g_iar_dev->regaddr + REG_IAR_DISPLAY_POSTION_RD1
		- position_cfg->layer * 4);
	return 0;
}
EXPORT_SYMBOL_GPL(disp_set_display_position);

/**
 * Set display module timing paraters
 */
int disp_set_panel_timing(struct disp_timing *timing)
{
	uint32_t value;

	if (NULL == g_iar_dev) {
		pr_err(KERN_ERR "IAR dev not inited!");
		return -1;
	}

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

	if (display_type == HDMI_TYPE) {
		value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD2);
		value = IAR_REG_SET_FILED(IAR_DPI_VBP_FIELD2, timing->vbp, value);
		value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD2, timing->vfp + 1, value);
		value = IAR_REG_SET_FILED(IAR_DPI_VSW_FIELD2, timing->vs, value);
		writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD2);
	} else {
		value = readl(g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD2);
		value = IAR_REG_SET_FILED(IAR_DPI_VBP_FIELD2,
				(timing->vbp) + 1, value);// for bt656 spec
		value = IAR_REG_SET_FILED(IAR_DPI_VFP_FIELD2, timing->vfp, value);
		value = IAR_REG_SET_FILED(IAR_DPI_VSW_FIELD2, timing->vs, value);
		writel(value, g_iar_dev->regaddr + REG_IAR_PARAMETER_VTIM_FIELD2);
	}
	writel(timing->vfp_cnt,
		g_iar_dev->regaddr + REG_IAR_PARAMETER_VFP_CNT_FIELD12);
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
	if ((cfg->width * cfg->height) > (display_out_width * display_out_height)) {
		pr_err("%s:video layer size exceed user config when insmod driver!\n",
				__func__);
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
	target_filed = IAR_LAYER_PRIORITY_1 - cfg->pri;
	value = IAR_REG_SET_FILED(target_filed, channelid, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);

	value = IAR_REG_SET_FILED(IAR_EN_OVERLAY_PRI1, 0x1, value);
	value = IAR_REG_SET_FILED(IAR_EN_OVERLAY_PRI2, 0x1, value);
	value = IAR_REG_SET_FILED(IAR_EN_OVERLAY_PRI3, 0x1, value);
	value = IAR_REG_SET_FILED(IAR_EN_OVERLAY_PRI4, 0x1, value);

	target_filed = IAR_ALPHA_SELECT_PRI1 - pri; //set alpha sel
	value = IAR_REG_SET_FILED(target_filed, cfg->alpha_sel, value);
	target_filed = IAR_OV_MODE_PRI1 - pri; //set overlay mode
	value = IAR_REG_SET_FILED(target_filed, cfg->ov_mode, value);
	target_filed = IAR_EN_ALPHA_PRI1 - pri; //set alpha en
	value = IAR_REG_SET_FILED(target_filed, cfg->alpha_en, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);

	g_iar_dev->buf_w_h[channelid][0] = cfg->buf_width;
	g_iar_dev->buf_w_h[channelid][1] = cfg->buf_height;
	writel(cfg->crop_height << 16 | cfg->crop_width,
                g_iar_dev->regaddr + REG_IAR_CROPPED_WINDOW_RD1 - channelid*4);

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

	if ((cfg->src_width * cfg->src_height) >
			(display_out_width * display_out_height)) {
		pr_err("%s:src video size exceed user config when insmod driver!\n",
				__func__);
		return -1;
	}
	if ((cfg->tgt_width * cfg->tgt_height) >
			(display_out_width * display_out_height)) {
		pr_err("%s:tgt video size exceed user config when insmod driver!\n",
				__func__);
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
	void __iomem *iar_clk_reg_addr;
	uint32_t reg_val = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}

	if (g_iar_dev->iar_pixel_clk == NULL)
		return -1;
#ifdef CONFIG_HOBOT_XJ2
	if (pixel_clk < 102000000)
		ips_set_iar_clk32(1);
	else
		ips_set_iar_clk32(0);
#endif
	if (pixel_clk == 148350000) {
		iar_clk_reg_addr = ioremap_nocache(0xA1000000 + 0x240, 4);
                reg_val = readl(iar_clk_reg_addr);
		reg_val = reg_val & 0x7fffffff;
		reg_val = (reg_val & 0xff00ffff) | 0xa0000;
		writel(reg_val, iar_clk_reg_addr);
		iounmap(iar_clk_reg_addr);
	} else if (pixel_clk == 74175000) {
		iar_clk_reg_addr = ioremap_nocache(0xA1000000 + 0x240, 4);
                reg_val = readl(iar_clk_reg_addr);
		reg_val = reg_val & 0x7fffffff;
		reg_val = (reg_val & 0xff00ffff) | 0x002a0000;
		writel(reg_val, iar_clk_reg_addr);
		iounmap(iar_clk_reg_addr);
	} else {
		pixel_rate = clk_round_rate(g_iar_dev->iar_pixel_clk, pixel_clk);
		ret = clk_set_rate(g_iar_dev->iar_pixel_clk, pixel_rate);
		if (ret) {
			pr_err("%s: err checkout iar pixel clock rate!!\n", __func__);
			return -1;
		}
	}

	pixel_rate = clk_get_rate(g_iar_dev->iar_pixel_clk);
	pr_debug("%s: iar pixel rate is %lld\n", __func__, pixel_rate);
	return 0;
}
EXPORT_SYMBOL_GPL(disp_set_pixel_clk);

static int disp_clk_disable(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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
	if (display_type == LCD_7_TYPE)
		pixel_clock = pixel_clk_video_800x480;
	else if (display_type == MIPI_1080P)
		pixel_clock = pixel_clk_video_1080x1920;
	else if (display_type == MIPI_720P_TOUCH)
		pixel_clock = pixel_clk_video_720x1280_touch;
	else if (display_type == SIF_IPI)
		pixel_clock = 54400000;
	else if (display_type == BT656_TYPE)
		pixel_clock = pixel_clk_video_704x576;
	else
		pixel_clock = 163000000;
	if (display_type != HDMI_TYPE) {
		pixel_clock = clk_round_rate(g_iar_dev->iar_pixel_clk, pixel_clock);
		ret = clk_set_rate(g_iar_dev->iar_pixel_clk, pixel_clock);
		if (ret) {
			pr_err("%s: err checkout iar pixel clock rate!!\n", __func__);
			return -1;
		}
		if (display_type == BT656_TYPE)
			set_iar_pixel_clk_inv();
		pixel_clock = clk_get_rate(g_iar_dev->iar_pixel_clk);
		pr_err("%s: iar pixel rate is %lld\n", __func__, pixel_clock);
	}
	if (bt1120_clk_invert != 0) {
		pr_err("%s: set bt1120 clock phase invert!!\n", __func__);
		set_iar_pixel_clk_inv();
	}
	return 0;
}

int iar_pixel_clk_enable(void)
{
	int ret = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (g_iar_dev->iar_pixel_clk == NULL)
		return -1;
	clk_disable_unprepare(g_iar_dev->iar_pixel_clk);
	return 0;
}
EXPORT_SYMBOL_GPL(iar_pixel_clk_disable);


static int ipi_clk_disable(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (g_iar_dev->iar_ipi_clk == NULL)
		return -1;
	if (ipi_clk_already_enable == 1) {
		clk_disable_unprepare(g_iar_dev->iar_ipi_clk);
		ipi_clk_already_enable = 0;
	}
	return 0;
}

static int ipi_clk_enable(void)
{
	int ret = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (g_iar_dev->iar_ipi_clk == NULL)
		return -1;
	if (ipi_clk_already_enable == 0) {
		ret = clk_prepare_enable(g_iar_dev->iar_ipi_clk);
		if (ret) {
			pr_err("%s: err enable iar ipi clock!!\n", __func__);
			return -1;
		}
		ipi_clk_already_enable = 1;
	}
	return 0;
}

int screen_backlight_init(void)
{
	int ret = 0;

	pr_debug("initialize lcd backligbt!!!\n");
	if (pwm0_request_status == 1)
		return 0;

	screen_backlight_pwm = pwm_get(&g_iar_dev->pdev->dev, "backlight");
	if (IS_ERR(screen_backlight_pwm)) {
		pr_err("\niar:No pwm device!!!!\n");
		return -ENODEV;
	}
	pr_debug("pwm get is okay!!!\n");
	/**
	 * pwm_config(struct pwm_device *pwm, int duty_ns,
	 * int period_ns) - change a PWM device configuration
	 * @pwm: PWM device
	 * @duty_ns: "on" time (in nanoseconds)
	 * @period_ns: duration (in nanoseconds) of one cycle
	 *
	 * Returns: 0 on success or a negative error code on failure.
	 */
	ret = pwm_config(screen_backlight_pwm, pwm_duty,
				pwm_period);
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
	pwm0_request_status = 1;
	pr_debug("pwm enable is okay!!!\n");

	return 0;
}
EXPORT_SYMBOL_GPL(screen_backlight_init);

int screen_backlight_deinit(void)
{
	if (pwm0_request_status == 0)
		return 0;
	if (screen_backlight_pwm == NULL) {
		pr_err("pwm is not init!!\n");
		return -1;
	}
	pwm_put(screen_backlight_pwm);
	screen_backlight_pwm = NULL;
	pwm0_request_status = 0;
	return 0;
}
EXPORT_SYMBOL_GPL(screen_backlight_deinit);

int screen_backlight_change(unsigned int duty)
{
	int ret = 0;

	if (screen_backlight_pwm == NULL) {
		pr_err("pwm is not init!!\n");
		return -1;
	}

	pwm_disable(screen_backlight_pwm);

	ret = pwm_config(screen_backlight_pwm, duty, pwm_period);

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
			duty = pwm_period - 1;
		} else if (backlight_level <= 10 && backlight_level > 0) {
			duty = pwm_period -
				pwm_period / 10 * backlight_level;
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
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (!g_iar_dev->pins_bt1120)
		return -ENODEV;
	return pinctrl_select_state(g_iar_dev->pinctrl,
			g_iar_dev->pins_bt1120);
}

int disp_pinmux_bt656(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
        }
	if (!g_iar_dev->pins_bt656)
		return -ENODEV;
	return pinctrl_select_state(g_iar_dev->pinctrl,
			g_iar_dev->pins_bt656);
}

int disp_pinmux_mipi_dsi(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (!g_iar_dev->pins_mipi_dsi)
		return -ENODEV;
	return pinctrl_select_state(g_iar_dev->pinctrl,
			g_iar_dev->pins_mipi_dsi);
}

int disp_pinmux_rgb(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
        }
	if (!g_iar_dev->pins_rgb)
		return -ENODEV;
	return pinctrl_select_state(g_iar_dev->pinctrl,
			g_iar_dev->pins_rgb);
}

int disp_pinmux_rgb_gpio(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
        }
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
	if ((cfg->width * cfg->height) > (display_out_width * display_out_height)) {
		pr_err("%s:video out size exceed user config when insmod driver!\n",
				__func__);
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
		disp_set_panel_timing(&video_720x1280_touch);
		if (bt656_output_pin_group == 0)
			writel(0x8, g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);//0x340
		else
			writel(0x0, g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);//0x340
		writel(0x13, g_iar_dev->regaddr + REG_DISP_LCDIF_CFG);//0x800
		writel(0x3, g_iar_dev->regaddr + REG_DISP_LCDIF_PADC_RESET_N);//0x804
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
	} else if (cfg->out_sel == OUTPUT_BT656) {
#ifdef CONFIG_HOBOT_XJ3
		ret = disp_pinmux_bt1120();
		if (ret)
			return -1;
		display_type = BT656_TYPE;
		//disp_set_panel_timing(&video_704x576);
		if (cfg->height == 576) {
			disp_set_panel_timing(&video_704x576);
		} else if (cfg->height == 480) {
			disp_set_panel_timing(&video_720x480);
		}
		writel(0x18, g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);
		value = readl(g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		value = IAR_REG_SET_FILED(IAR_PANEL_COLOR_TYPE, 1, value);
		value = IAR_REG_SET_FILED(IAR_INTERLACE_SEL, 1, value);
		value = IAR_REG_SET_FILED(IAR_PIXEL_RATE, 1, value);
		value = IAR_REG_SET_FILED(IAR_YCBCR_OUTPUT, 1, value);
		value = IAR_REG_SET_FILED(IAR_ITU_R_656_EN, 1, value);

		writel(value, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		value = readl(g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
		value = IAR_REG_SET_FILED(IAR_BT601_709_SEL, 1, value);
		writel(value, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
		writel(0x4, g_iar_dev->regaddr + REG_IAR_DE_CONTROL_WO); //0x31c should be 4
#else
		pr_err("%s: error output mode!!!\n", __func__);
#endif
	} else if (cfg->out_sel == OUTPUT_IPI) {
		display_type = SIF_IPI;
		disp_set_panel_timing(&video_1920x1080);
		writel(0x9, g_iar_dev->regaddr + REG_IAR_DE_OUTPUT_SEL);
		//IPI(SIF)
		value = readl(g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		value = IAR_REG_SET_FILED(IAR_PANEL_COLOR_TYPE, 2, value);
		//yuv444
		value = IAR_REG_SET_FILED(IAR_YCBCR_OUTPUT, 1, value);
		//convert ycbcr
		writel(value, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		value = readl(g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
		value = IAR_REG_SET_FILED(IAR_BT601_709_SEL, 1, value);
		writel(value, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
	} else {
		pr_err("%s: error output mode!!!\n", __func__);
		return -1;
	}
	if (cfg->out_sel == OUTPUT_BT656) {
		value = IAR_REG_SET_FILED(IAR_PANEL_WIDTH, cfg->width, 0);
		value = IAR_REG_SET_FILED(IAR_PANEL_HEIGHT, (cfg->height) >> 1, value);

	} else {
		value = IAR_REG_SET_FILED(IAR_PANEL_WIDTH, cfg->width, 0);
		value = IAR_REG_SET_FILED(IAR_PANEL_HEIGHT, cfg->height, value);
	}
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
		display_type = MIPI_720P_TOUCH;
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

	if (cfg->out_sel != OUTPUT_BT656) {
		value = readl(g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
		value = IAR_REG_SET_FILED(IAR_PANEL_COLOR_TYPE, 2, value);
		value = IAR_REG_SET_FILED(IAR_YCBCR_OUTPUT, 1, value);
		writel(value, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
	}
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

/**
 * get actual display address currently
 */
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

/**
 * vio module get display address according to channel and current index
 * only used in XJ2 platform
 */
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

/**
 * Switch ping-pong buffer
 */
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

/**
 * Only used for XJ2 platform
 */
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

/**
 * VIO module set graphic address to be displayed
 * @disp_layer: display layer to be used
 * @yaddr: the y address of the graphic
 * @caddr: the uv address of the graphic
 */
int32_t ipu_set_display_addr(uint32_t disp_layer,
		uint32_t yaddr, uint32_t caddr)
{
	pr_debug("iar: ipu set iar!!!!!!!!\n");
	pr_debug("layer is %d, yaddr is 0x%x, caddr is 0x%x\n",
			disp_layer, yaddr, caddr);
	disp_user_config_done = 1;//for debug
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

/**
 * VIO module get pipeline number and vio output channel
 * user wanted to be displayed
 */
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

/**
 * Checkout display camera number
 * only used for XJ2 platform
 */
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

/**
 * Set output channel of VIO module user want to be displayed
 * only used for XJ2 platform
 */
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

/**
 * The thread that IAR display the output of VIO module
 */
static int iar_thread(void *data)
{
	buf_addr_t display_addr;
	buf_addr_t display_addr_video1;

	if (g_iar_dev == NULL) {
                pr_err("%s: iar not init!!\n", __func__);
                return -1;
        }
	while (!kthread_should_stop()) {
		if (kthread_should_stop())
			break;
		wait_event_interruptible(g_iar_dev->wq_head,
				(ipu_process_done & iar_video_not_pause) || stop_flag);
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

	if (enable_sif_mclk() != 0)
		return -1;
	if (iar_pixel_clk_enable() != 0)
		return -1;
	enable_iar_irq();
	enable_irq(g_iar_dev->irq);

	init_waitqueue_head(&g_iar_dev->done_wq);
	init_waitqueue_head(&g_iar_dev->output_done_wq[0]);
	init_waitqueue_head(&g_iar_dev->output_done_wq[1]);

	g_iar_dev->state = BIT(IAR_WB_INIT);

	if (hb_disp_base_board_id == 0x1)
		screen_backlight_init();
	if (g_iar_dev->iar_task == NULL) {
		g_iar_dev->iar_task = kthread_run(iar_thread,
				(void *)g_iar_dev, "iar_thread");
		if (IS_ERR(g_iar_dev->iar_task)) {
			g_iar_dev->iar_task = NULL;
			dev_err(&g_iar_dev->pdev->dev, "iar thread create fail\n");
			ret = PTR_ERR_OR_ZERO(g_iar_dev->iar_task);
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
	if (hb_disp_base_board_id == 0x1)
		screen_backlight_deinit();
	if (disable_sif_mclk() != 0)
		return -1;
	if (iar_pixel_clk_disable() != 0)
		return -1;
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
	if (iar_enable_sif_mclk() != 0)
		return -1;
	if (disp_clk_enable() != 0)
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
	} else if (display_type == SIF_IPI) {
		if (ipi_clk_enable() != 0)
			return -1;
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

	if (disp_clk_disable() != 0)
		return -1;
	if (iar_disable_sif_mclk() != 0)
		return -1;
	//del_timer(&iartimer);
	return ret;
}
EXPORT_SYMBOL_GPL(iar_stop);

/**
 * Bring the configuartion of iar registers into effect
 */
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

frame_buf_t* hobot_iar_get_framebuf_addr(int channel)
{
	if (NULL == g_iar_dev || channel < 0 || channel > IAR_CHANNEL_4) {
		printk(KERN_ERR "IAR dev not inited!");
		return NULL;
	}
	if (g_iar_dev->frambuf[channel].vaddr == NULL) {
		pr_err("%s: channel %d not alloc memory!!\n", __func__, channel);
		return NULL;
	}
	return &g_iar_dev->frambuf[channel];
}
EXPORT_SYMBOL_GPL(hobot_iar_get_framebuf_addr);

int hobot_iar_get_layer_size(unsigned int *width, unsigned int *height) {
	if (width == NULL || height == NULL)
		return -1;
	*width = display_out_width;
	*height = display_out_height;
	return 0;
}
EXPORT_SYMBOL_GPL(hobot_iar_get_layer_size);


int iar_wb_stream_on(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (!(g_iar_dev->state & (BIT(IAR_WB_STOP) | BIT(IAR_WB_INIT)))) {
		pr_err("invalid REQBUFS is requested(%lX)",
			g_iar_dev->state);
		return -EINVAL;
	}

	framemgr = &g_iar_dev->framemgr;
	ret = frame_manager_open(framemgr, buffers);
	if (ret) {
		pr_err("frame manage open failed, ret(%d)", ret);
		return ret;
	}

	g_iar_dev->state = BIT(IAR_WB_REBUFS);

	return ret;
}

int iar_wb_qbuf(struct frame_info *frameinfo)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
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

	return ret;
}

int iar_wb_dqbuf(struct frame_info *frameinfo)
{
	int ret = 0;
	struct list_head *done_list;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return;
	}
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
}

int iar_wb_getcfg()
{
	int value = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	value = g_iar_dev->wb_format;
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
	int ret = 0;
	/**
	 * REG_IAR_CURRENT_CBUF_ADDR_WR_Y  0x140
	 * REG_IAR_CURRENT_CBUF_ADDR_WR_UV 0x144
	 * REG_IAR_CAPTURE_CON             0x8c
	 * #[4:3] SOURCE_SEL 0:Overlay 1:UP-Scale, 2: DE_PUT(*)
	 * #[2:0] OUTPUT_FMT: 100b:NV12 110b:Unpacked RGB888
	 * REG_IAR_UPDATE                  0x98
	 * REG_IAR_DE_CONTROL_WO           0x31c
	 */
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	int regval = 0;
	int wbcon = ((g_iar_dev->wb_sel&0x3) << 3) + (g_iar_dev->wb_format&0x7);
	size_t frame_size_y = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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
		writel(wbcon, g_iar_dev->regaddr + REG_IAR_CAPTURE_CON);
		regval = readl(g_iar_dev->regaddr + REG_IAR_UPDATE);
		regval |= 0x1;
		writel(regval, g_iar_dev->regaddr + REG_IAR_UPDATE);
		regval = readl(g_iar_dev->regaddr + REG_IAR_DE_CONTROL_WO);
		regval |= 0x1;
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
	unsigned long flags;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	framemgr = &g_iar_dev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		do_gettimeofday(&frame->frameinfo.tv);
		trans_frame(framemgr, frame, FS_COMPLETE);
	} else {
		// pr_err("%s PROCESS queue has no member;\n", __func__);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	wake_up(&g_iar_dev->done_wq);
	return 0;
}

int iar_output_stream_on(layer_no)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
        }
	g_iar_dev->output_state[layer_no] = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(iar_output_stream_on);

int iar_output_stream_off(layer_no)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (g_iar_dev->output_state[layer_no] == 1) {
		frame_manager_flush(&g_iar_dev->framemgr_layer[layer_no]);
		frame_manager_close(&g_iar_dev->framemgr_layer[layer_no]);
		g_iar_dev->output_state[layer_no] = 0;
	}

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

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	index = frameinfo->bufferindex;
	framemgr = &g_iar_dev->framemgr_layer[layer_no];
	BUG_ON(index >= framemgr->num_frames);

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = &framemgr->frames[index];
	if (frame->state == FS_FREE) {
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		if ((frame_output_fps < 30) || (frame_output_fps > 60) ||
			(frame_input_fps < 25) || (frame_input_fps > 60)) {
			frame_output_fps = 30;
			frame_input_fps  = 30;
		}
		frame_output += frame_output_fps;
		if (frame_output > 2 * frame_input_fps) {
			frame->frameinfo.dynamic_flag = 3;
			frame_output -= (int)(3 * frame_input_fps);
		} else if (frame_output > frame_input_fps) {
			frame->frameinfo.dynamic_flag = 2;
			frame_output -= (int)(2 * frame_input_fps);
		} else {
			frame->frameinfo.dynamic_flag = 1;
			frame_output -= (int)(frame_input_fps);
		}

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

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	framemgr = &g_iar_dev->framemgr_layer[layer_no];
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		// printk("iar_output_done: %d\n", frame->frameinfo.bufferindex);
		do_gettimeofday(&frame->frameinfo.tv);
		trans_frame(framemgr, frame, FS_COMPLETE);
		ret = 0;
	} else {
		// pr_err("%s PROCESS queue has no member;\n", __func__);
		ret = -1;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	if (ret == 0) {
		wake_up(&g_iar_dev->output_done_wq[layer_no]);
	}
	return ret;
}

int iar_output_start(int layer_no)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	buf_addr_t display_addr;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	framemgr = &g_iar_dev->framemgr_layer[layer_no];
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);

	if (frame) {
		int size = frame->frameinfo.width * frame->frameinfo.height;
		display_addr.Yaddr = frame->frameinfo.addr[0];
		if (frame->frameinfo.addr[1] != 0)
			display_addr.Uaddr = frame->frameinfo.addr[1];
		else
			display_addr.Uaddr = frame->frameinfo.addr[0] + size;
		display_addr.Vaddr = 0;
		iar_set_bufaddr(layer_no, &display_addr);
		iar_update();
		frame->frameinfo.dynamic_flag--;
		if (frame->frameinfo.dynamic_flag == 0) {
			trans_frame(framemgr, frame, FS_PROCESS);
		}
	} else {
		// printk
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return ret;
}

/**
 * IAR module interrupt handler
 * interrupt status register meaning:
 * @bit0: DE starts refreshing a new frame
 * @bit22: DE finished refreshing current frame
 * @bit23: Capture path finishes capturing current frame
 */
static irqreturn_t hobot_iar_irq(int this_irq, void *data)
{
	int regval = 0;
	disable_irq_nosync(this_irq);
	//TODO
	regval = readl(g_iar_dev->regaddr + REG_IAR_DE_SRCPNDREG);

	if (regval & BIT(0)) {
		writel(BIT(0), g_iar_dev->regaddr + REG_IAR_DE_SRCPNDREG);
		if (g_iar_dev->output_state[0] == 1)
			iar_output_start(0);
		if (g_iar_dev->output_state[1] == 1)
			iar_output_start(1);
	}

	if (regval & BIT(22)) {
		writel(BIT(22), g_iar_dev->regaddr + REG_IAR_DE_SRCPNDREG);
		if(g_iar_dev->capture_state == 1) {
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

/**
 * Display one frame using ping-pong buffer machanism
 * @layer_no: display layer of IAR module
 * @yaddr: y address of the frame to be displayed
 * @caddr: uv address of the frame to be displayed
 */
int disp_set_ppbuf_addr(uint8_t layer_no, void *yaddr, void *caddr)
{
	buf_addr_t display_addr;
	uint32_t video_index;
	uint32_t y_size;
	void __iomem *video_to_display_vaddr;
	int ret = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (layer_no > 1)
		return -1;
	if (yaddr == NULL)
		return 0;
	if (g_iar_dev->pingpong_buf[layer_no].framebuf[0].vaddr == NULL ||
			g_iar_dev->pingpong_buf[layer_no].framebuf[1].vaddr == NULL) {
		pr_err("%s: video layer %d is not alloc memory,exit!!\n", __func__, layer_no);
		return -1;
	}
	y_size =
	g_iar_dev->buf_w_h[layer_no][0] * g_iar_dev->buf_w_h[layer_no][1];
	video_index = g_iar_dev->cur_framebuf_id[layer_no];
	video_to_display_vaddr =
		g_iar_dev->pingpong_buf[layer_no].framebuf[!video_index].vaddr;
	ret = (int)copy_from_user(video_to_display_vaddr, yaddr, y_size);
	if (ret) {
		pr_err("%s: error copy y imge from user!\n", __func__);
		return ret;
	}
        ret = (int)copy_from_user(video_to_display_vaddr + y_size,
			caddr, y_size >> 1);
	if (ret) {
		pr_err("%s: error copy uv imge from user!\n", __func__);
		return ret;
	}

	disp_copy_done = 1;

	display_addr.Yaddr =
	(uint32_t)g_iar_dev->pingpong_buf[layer_no].framebuf[!video_index].paddr;
	display_addr.Uaddr =
	(uint32_t)g_iar_dev->pingpong_buf[layer_no].framebuf[!video_index].paddr
	+ y_size;
	display_addr.Vaddr = 0;

	iar_set_bufaddr(layer_no, &display_addr);
	//switch video pingpong buffer
	g_iar_dev->cur_framebuf_id[layer_no] = !video_index;
	return 0;
}
EXPORT_SYMBOL_GPL(disp_set_ppbuf_addr);

/**
 * Rotate one frame
 * only used for XJ2 platform
 */
int iar_rotate_video_buffer(phys_addr_t yaddr,
		phys_addr_t uaddr, phys_addr_t vaddr)
{
	int video_index;
	buf_addr_t display_addr;
	void __iomem *video_display_vaddr;
#ifdef CONFIG_HOBOT_XJ3
	pr_err("%s: xj3 platform not support software rotate,exit!!\n", __func__);
	return -1;
#endif
	video_index = g_iar_dev->cur_framebuf_id[IAR_CHANNEL_1];
	video_display_vaddr =
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[!video_index].vaddr;
	do {
		struct timeval tv_s;
		struct timeval tv_e;
		long int time_cost = 0;
		uint8_t *src_addr = (void *)(phys_to_virt(yaddr));
		uint8_t *tmp_addr =
			(uint8_t *)g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr;

		do_gettimeofday(&tv_s);

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

		do_gettimeofday(&tv_e);
		time_cost = (tv_e.tv_sec*1000 + tv_e.tv_usec/1000) -
			(tv_s.tv_sec*1000 + tv_s.tv_usec/1000);
		pr_debug("time cost %ldms\n", time_cost);
	} while (0);

	//display video
	display_addr.Yaddr =
	(uint32_t)g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[!video_index].paddr;
	display_addr.Uaddr =
	(uint32_t)g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[!video_index].paddr
	+ 720*1280;
	display_addr.Vaddr = 0;

	iar_set_bufaddr(IAR_CHANNEL_1, &display_addr);
	//switch video pingpong buffer
	g_iar_dev->cur_framebuf_id[IAR_CHANNEL_1] = !video_index;

	iar_update();

	return 0;
}

int panel_hardware_reset(void)
{
	int ret = 0;

	ret = gpio_request(panel_reset_pin, "disp_panel_reset_pin");
	if (ret) {
		pr_err("%s: error request panel reset pin!!\n", __func__);
		return -1;
	}
	if (panel_reset_pin >= 0) {
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
	}
	gpio_free(panel_reset_pin);
	return 0;
}
EXPORT_SYMBOL_GPL(panel_hardware_reset);

int get_iar_module_rst_pin(void)
{
	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
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

static int stride_copy_bmp(int width, int height, const unsigned char *src,
		int x0, int y0, int stride0, int height0,
		unsigned char *dst, int x1, int y1, int stride1)
{
	int i, j;
	struct bmp_image *bmp = (struct bmp_image *)src;
	const unsigned char *bmap;
	int widthi, heighti;
	int stb0;
	unsigned colours, bmp_bpix;
	const unsigned char *p0, *hp0;
	unsigned char *p1, *hp1;
	unsigned char ct;

	int hdr_size;

	if (!bmp || !(bmp->header.signature[0] == 'B' &&
				bmp->header.signature[1] == 'M')) {
		pr_err("Error: no valid bmp image for logo");
		return -EINVAL;
	}

	widthi = get_unaligned_le32(&bmp->header.width);
	heighti = get_unaligned_le32(&bmp->header.height);

	if (width <= 0)
		width = widthi;
	if (height <= 0)
		height = heighti;
	if (stride0 <= 0)
		stride0 = width;
	if (height0 <= 0)
		height0 = height;

	stb0 = stride0 * 3;
	if (stb0 & 0x03) stb0 = (stride0 * 3 + 3) & ~0x03;

	bmp_bpix = get_unaligned_le16(&bmp->header.bit_count);
	hdr_size = get_unaligned_le16(&bmp->header.size);

	colours = 1 << bmp_bpix;

	bmap = src + get_unaligned_le32(&bmp->header.data_offset);

	p0 = bmap;
	p1 = dst;

	switch (bmp_bpix) {
	case 24:
		hp0 = p0 + y0 * stb0;
		hp1 = p1 + (y1 + height - 1) * stride1 * 4;
		for (i = 0; i < height; ++i) {
			p0 = hp0 + x0 * 3;
			p1 = hp1 + x1 * 4;
			for (j = 0; j < width; j++) {
				ct = 0xff;
				ct &= *p0;
				*(p1++) = *(p0++);
				ct &= *p0;
				*(p1++) = *(p0++);
				ct &= *p0;
				*(p1++) = *(p0++);
				*(p1++) = (ct >= 0xe0 ? 0 : 0xff);
			}
			hp0 += stb0;
			hp1 -= stride1 * 4;
		}
		break;
	case 32:
		hp0 = p0 + y0 * stride0 * 4;
		hp1 = p1 + (y1 + height - 1) * stride1 * 4;
		for (i = 0; i < height; ++i) {
			p0 = hp0 + x0 * 4;
			p1 = hp1 + x1 * 4;
			for (j = 0; j < width; j++) {
				*(p1++) = *(p0++);
				*(p1++) = *(p0++);
				*(p1++) = *(p0++);
				*(p1++) = *(p0++);
			}
			hp0 += stride0 * 4;
			hp1 -= stride1 * 4;
		}
		break;
	default:
		pr_err("logo only support 24/32 bits bmp\n");
		break;
	}
	return 0;
}
#if 0
static void stride_copy_420(int width, int height, const char *src,
		int x0, int y0, int stride0, int height0,
		char *dst, int x1, int y1, int stride1, int height1)
{
	const char *p0;
	char *p1;
	int i;

	// copy y
	p0 = src + (y0 * stride0) + x0;
	p1 = dst + (y1 * stride1) + x1;
	for (i = 0; i < height; i++) {
		memcpy(p1, p0, width);
		p0 += stride0;
		p1 += stride1;
	}
	// copy uv
	p0 = src + stride0 * height0;
	p1 = dst + stride1 * height1;
	x0 >>= 1;
	x1 >>= 1;
	stride0 >>= 1;
	stride1 >>= 1;
	width >>= 1;
	p0 += (y0 * stride0) + x0;
	p1 += (y1 * stride1) + x1;
	for (i=0; i < height; i++) {
		memcpy(p1, p0, width);
		p0 += stride0;
		p1 += stride1;
	}
}
#endif
static int display_color_bar(unsigned int width, unsigned height,
		char *draw_start_vaddr)
{
	char *vaddr;
	int i = 0;
	int color_bar_height = 0;
	int left_height = 0;

	if (draw_start_vaddr == NULL) {
		pr_err("%s: draw address is invalid(null), exit\n", __func__);
		return -1;
	}
	if (width > 1920 || height > 1920) {
		pr_err("%s: invalid input parameters,exit\n", __func__);
		return -1;
	}
	if ((width * height) > (display_out_width * display_out_height)) {
		pr_err("%s:video layer size exceed user config when insmod driver!\n",
				__func__);
		return -1;
	}
	vaddr = draw_start_vaddr;
	color_bar_height = (height >> 1) / 5;
	left_height = height - color_bar_height * 5;

	for (i = 0; i < width * 4 * color_bar_height; i++) {
		if (((i + 4) % 4) == 0) {
			*vaddr = 0xff;//B
		} else if (((i + 4) % 4) == 1) {
			*vaddr = 0x00;
		} else if (((i + 4) % 4) == 2) {
			*vaddr = 0x00;
		} else if (((i + 4) % 4) == 3) {
			*vaddr = 0xff;
		}
		vaddr++;
	}
	for (i = 0; i < width * 4 * color_bar_height; i++) {
		if (((i + 4) % 4) == 0) {
			*vaddr = 0x00;
		} else if (((i + 4) % 4) == 1) {
			*vaddr = 0xff;//G
		} else if (((i + 4) % 4) == 2) {
			*vaddr = 0x00;
		} else if (((i + 4) % 4) == 3) {
			*vaddr = 0xff;
		}
		vaddr++;
	}
	for (i = 0; i < width * 4 * color_bar_height; i++) {
		if (((i + 4) % 4) == 0) {
			*vaddr = 0x00;
		} else if (((i + 4) % 4) == 1) {
			*vaddr = 0x00;
		} else if (((i + 4) % 4) == 2) {
			*vaddr = 0xff;//R
		} else if (((i + 4) % 4) == 3) {
			*vaddr = 0xff;
		}
		vaddr++;
	}
	for (i = 0; i < width * 4 * color_bar_height; i++) {
		if (((i + 4) % 4) == 0) {
			*vaddr = 0x00;
		} else if (((i + 4) % 4) == 1) {
			*vaddr = 0x00;
		} else if (((i + 4) % 4) == 2) {
			*vaddr = 0x00;
		} else if (((i + 4) % 4) == 3) {
			*vaddr = 0xff;//A
		}
		vaddr++;
	}
	for (i = 0; i < width * 4 * color_bar_height; i++) {
		if (((i + 4) % 4) == 0) {
			*vaddr = 0xff;
		} else if (((i + 4) % 4) == 1) {
			*vaddr = 0xff;
		} else if (((i + 4) % 4) == 2) {
			*vaddr = 0xff;
		} else if (((i + 4) % 4) == 3) {
			*vaddr = 0xff;//A
		}
		vaddr++;
	}
	for (i = 0; i < width * 4 * left_height; i++) {
		*vaddr = 0x0;
		vaddr++;
	}
	return 0;
}

static int iar_debug_show(struct seq_file *s, void *unused)
{
        int iar_open = 0;
	uint32_t reg_value = 0;
	int channel_enable_status[4] = {0};
	int channel_priority[4] = {0};
	int channel_resolution[4][2];
	int channel_crop_resolution[4][2];
	int channel_display_position[4][2];
	int i = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init!!\n", __func__);
		return -1;
	}
	if (__clk_is_enabled(g_iar_dev->sif_mclk) &&
			__clk_is_enabled(g_iar_dev->iar_pixel_clk)) {
		iar_open = 1;
		seq_printf(s, "Display module status: display module is runing\n");
	} else {
		iar_open = 0;
		seq_printf(s, "Display module status: display module is stop\n");
	}
	if (enable_sif_mclk() != 0)
		return -1;
	if (iar_pixel_clk_enable() != 0)
		return -1;
	reg_value = readl(g_iar_dev->regaddr + 0x0);
	for (i = 0; i < 4; i++) {
		if ((reg_value >> (24 + i))  & 0x1) {
			channel_enable_status[i] = 1;
			channel_resolution[i][0] =
				readl(g_iar_dev->regaddr + 0x44 - i * 4);//width
			channel_resolution[i][1] = g_iar_dev->buf_w_h[i][1];
			channel_crop_resolution[i][0] =
				readl(g_iar_dev->regaddr + 0x24 - i * 4) & 0x7ff;//width
			channel_crop_resolution[i][1] =
				(readl(g_iar_dev->regaddr + 0x24 - i * 4) >> 16) & 0x7ff;
			channel_display_position[i][0] =
				readl(g_iar_dev->regaddr + 0x58 - i * 4) & 0x7ff;//x pos
			channel_display_position[i][1] =
				(readl(g_iar_dev->regaddr + 0x58 - i * 4) >> 16) & 0x7ff;
		}
		channel_priority[i] = (reg_value >> (16 + i * 2)) & 0x3;
	}
	seq_printf(s, "layer info:\n");
	for (i = 0; i < 4; i++) {
		if (channel_enable_status[i] == 1) {
			seq_printf(s,
				"           layer %d is enabled\n", i);
			seq_printf(s,
			"           layer %d resolution is width: %d, height: %d\n",
				i, channel_resolution[i][0], channel_resolution[i][1]);
			seq_printf(s,
			"           layer %d crop width is %d, height: %d\n", i,
				channel_crop_resolution[i][0], channel_crop_resolution[i][1]);
			seq_printf(s,
			"           layer %d display x position is %d, y position is %d\n", i,
				channel_display_position[i][0], channel_display_position[i][1]);
		}
	}
	seq_printf(s, "Priority: the highest priority layer is %d\n",
			channel_priority[0]);
	seq_printf(s, "          the second highest priority layer is %d\n",
			channel_priority[1]);
	seq_printf(s, "          the third highest priority layer is %d\n",
			channel_priority[2]);
	seq_printf(s, "          the lowest priority layer is %d\n",
			channel_priority[3]);
	reg_value = readl(g_iar_dev->regaddr + 0x340);
	if (reg_value & 0x1)
		seq_printf(s, "Output: output mode is IPI\n");
	else if (reg_value & 0x2)
		seq_printf(s, "Output: output mode is BT1120\n");
	else if (reg_value & 0x4)
		seq_printf(s, "Output: output mode is RGB\n");
	else if ((readl(g_iar_dev->regaddr + 0x800) & 0x13) == 0x13)
		seq_printf(s, "Output: output mode is MIPI-DSI\n");
	reg_value = readl(g_iar_dev->regaddr + 0x200);
	seq_printf(s, "        output resolution width is %d, height is %d\n",
			reg_value & 0x7ff, (reg_value >> 16) & 0x7ff);

	if (disable_sif_mclk() != 0)
		return -1;

	if (iar_pixel_clk_disable() != 0)
		return -1;

        return 0;
}

static int iar_debug_open(struct inode *inode, struct file *file)
{
        return single_open(file, iar_debug_show, inode->i_private);
}

static const struct file_operations iar_debug_fops = {
        .open = iar_debug_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

/**
 * Config IAR module accroding to target display panel
 * this interface mainly used for fb(frame-buffer) driver
 */
int user_config_display(enum DISPLAY_TYPE d_type)
{
	buf_addr_t graphic_display_paddr;
	buf_addr_t graphic1_display_paddr;
	channel_base_cfg_t channel_base_cfg[2] = {{0}, {0}};
	output_cfg_t output_cfg = {0};

	graphic_display_paddr.addr = (uint32_t)g_iar_dev->frambuf[2].paddr;
	graphic1_display_paddr.addr = (uint32_t)g_iar_dev->frambuf[3].paddr;

	enable_sif_mclk();
	iar_pixel_clk_enable();
	switch (d_type) {
                case HDMI_TYPE:
#ifdef CONFIG_HOBOT_XJ2
			disp_set_panel_timing(&video_1920x1080);
#else
			disp_set_panel_timing(&video_1920x1080);
			channel_base_cfg[0].enable = 1;
			channel_base_cfg[1].enable = 1;
			channel_base_cfg[0].channel = IAR_CHANNEL_1;
			channel_base_cfg[0].pri = 2;
			channel_base_cfg[0].width = 1920;
			channel_base_cfg[0].height = 1080;
			channel_base_cfg[0].buf_width = 1920;
			channel_base_cfg[0].buf_height = 1080;
			channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
			channel_base_cfg[0].alpha_sel = 0;
			channel_base_cfg[0].ov_mode = 0;
			channel_base_cfg[0].alpha_en = 1;
			channel_base_cfg[0].alpha = 255;
			channel_base_cfg[0].crop_width = 1920;
			channel_base_cfg[0].crop_height = 1080;
			channel_base_cfg[1].channel = IAR_CHANNEL_3;
			channel_base_cfg[1].pri = 0;
			channel_base_cfg[1].width = 1920;
			channel_base_cfg[1].height = 1080;
			channel_base_cfg[1].buf_width = 1920;
			channel_base_cfg[1].buf_height = 1080;
			channel_base_cfg[1].format = 4;//ARGB8888
			channel_base_cfg[1].alpha_sel = 0;
			channel_base_cfg[1].ov_mode = 0;
			channel_base_cfg[1].alpha_en = 1;
			channel_base_cfg[1].alpha = 128;
			channel_base_cfg[1].crop_width = 1920;
			channel_base_cfg[1].crop_height = 1080;

			output_cfg.out_sel = 1;
			output_cfg.width = 1920;
			output_cfg.height = 1080;
			output_cfg.bgcolor = 16744328;//white.

			iar_channel_base_cfg(&channel_base_cfg[0]);
			iar_channel_base_cfg(&channel_base_cfg[1]);
			iar_output_cfg(&output_cfg);

			writel(0x0472300f, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
			//panel color type is yuv444, YCbCr conversion needed
			writel(8, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
			//select BT709 color domain
			writel(FORMAT_ORGANIZATION_VAL,
					g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
			iar_switch_buf(0);
			iar_set_bufaddr(IAR_CHANNEL_3, &graphic_display_paddr);
			iar_set_bufaddr(IAR_CHANNEL_4, &graphic1_display_paddr);
			iar_update();
#endif
                        break;
                case LCD_7_TYPE:
			disp_set_panel_timing(&video_800x480);
			channel_base_cfg[0].enable = 1;
			channel_base_cfg[1].enable = 1;
			channel_base_cfg[0].channel = IAR_CHANNEL_1;
			channel_base_cfg[0].pri = 2;
			channel_base_cfg[0].width = 800;
			channel_base_cfg[0].height = 480;
			channel_base_cfg[0].buf_width = 800;
			channel_base_cfg[0].buf_height = 480;
			channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
			channel_base_cfg[0].alpha_sel = 0;
			channel_base_cfg[0].ov_mode = 0;
			channel_base_cfg[0].alpha_en = 1;
			channel_base_cfg[0].alpha = 255;
			channel_base_cfg[0].crop_width = 800;
			channel_base_cfg[0].crop_height = 480;
			channel_base_cfg[1].channel = IAR_CHANNEL_3;
			channel_base_cfg[1].pri = 0;
			channel_base_cfg[1].width = 800;
			channel_base_cfg[1].height = 480;
			channel_base_cfg[1].buf_width = 800;
			channel_base_cfg[1].buf_height = 480;
			channel_base_cfg[1].format = 4;//ARGB8888
			channel_base_cfg[1].alpha_sel = 0;
			channel_base_cfg[1].ov_mode = 0;
			channel_base_cfg[1].alpha_en = 1;
			channel_base_cfg[1].alpha = 128;
			channel_base_cfg[1].crop_width = 800;
			channel_base_cfg[1].crop_height = 480;

#ifdef CONFIG_HOBOT_XJ2
			output_cfg.out_sel = 1;
#else
			output_cfg.out_sel = 2;
#endif
			output_cfg.width = 800;
			output_cfg.height = 480;
			output_cfg.bgcolor = 16744328;//white.

			iar_channel_base_cfg(&channel_base_cfg[0]);
			iar_channel_base_cfg(&channel_base_cfg[1]);
			iar_output_cfg(&output_cfg);
#ifdef CONFIG_HOBOT_XJ2
			writel(0x041bf00f, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
			//panel color type is yuv444, YCbCr conversion needed
			writel(REFRESH_CFG_VAL, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
			//select BT709 color domain
			writel(FORMAT_ORGANIZATION_VAL,
					g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
#ifdef CONFIG_LOGO
#ifndef CONFIG_LOGO_FROM_KERNEL
			if (logo_vaddr != NULL)
			if (g_iar_dev->frambuf[2].vaddr != NULL)
					memcpy(g_iar_dev->frambuf[2].vaddr, logo_vaddr, 800*480*4);
#endif
#endif
#else
			writel(0x0572300f, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
			//panel color type is yuv444, YCbCr conversion needed
			writel(0, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
			//select BT709 color domain
			writel(FORMAT_ORGANIZATION_VAL,
					g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
#endif
			iar_switch_buf(0);
			iar_set_bufaddr(IAR_CHANNEL_3, &graphic_display_paddr);
			iar_set_bufaddr(IAR_CHANNEL_4, &graphic1_display_paddr);
			iar_update();
                        break;
		case MIPI_720P:
			disp_set_panel_timing(&video_720x1280);
			channel_base_cfg[0].enable = 1;
			channel_base_cfg[1].enable = 1;
			channel_base_cfg[0].channel = IAR_CHANNEL_1;
			channel_base_cfg[0].pri = 2;
			channel_base_cfg[0].width = 720;
			channel_base_cfg[0].height = 1280;
			channel_base_cfg[0].buf_width = 720;
			channel_base_cfg[0].buf_height = 1280;
			channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
			channel_base_cfg[0].alpha_sel = 0;
			channel_base_cfg[0].ov_mode = 0;
			channel_base_cfg[0].alpha_en = 1;
			channel_base_cfg[0].alpha = 255;
			channel_base_cfg[0].crop_width = 720;
			channel_base_cfg[0].crop_height = 1280;
			channel_base_cfg[1].channel = IAR_CHANNEL_3;
			channel_base_cfg[1].pri = 0;
			channel_base_cfg[1].width = 720;
			channel_base_cfg[1].height = 1280;
			channel_base_cfg[1].buf_width = 720;
			channel_base_cfg[1].buf_height = 1280;
			channel_base_cfg[1].format = 4;//ARGB8888
			channel_base_cfg[1].alpha_sel = 0;
			channel_base_cfg[1].ov_mode = 0;
			channel_base_cfg[1].alpha_en = 1;
			channel_base_cfg[1].alpha = 128;
			channel_base_cfg[1].crop_width = 720;
			channel_base_cfg[1].crop_height = 1280;

			output_cfg.out_sel = 1;
			output_cfg.width = 720;
			output_cfg.height = 1280;
			output_cfg.bgcolor = 16744328;//white.

			iar_channel_base_cfg(&channel_base_cfg[0]);
			iar_channel_base_cfg(&channel_base_cfg[1]);
			iar_output_cfg(&output_cfg);

			writel(0x041bf00f, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
			//panel color type is yuv444, YCbCr conversion needed
			writel(REFRESH_CFG_VAL, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
			//select BT709 color domain
			writel(FORMAT_ORGANIZATION_VAL,
					g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);

			iar_switch_buf(0);
			iar_set_bufaddr(2, &graphic_display_paddr);
			iar_update();
			break;
		case MIPI_1080P:
			disp_set_panel_timing(&video_1080x1920);
			channel_base_cfg[0].enable = 1;
			channel_base_cfg[1].enable = 1;
			channel_base_cfg[0].channel = IAR_CHANNEL_1;
			channel_base_cfg[0].pri = 2;
			channel_base_cfg[0].width = 1080;
			channel_base_cfg[0].height = 1920;
			channel_base_cfg[0].buf_width = 1080;
			channel_base_cfg[0].buf_height = 1920;
			channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
			channel_base_cfg[0].alpha_sel = 0;
			channel_base_cfg[0].ov_mode = 0;
			channel_base_cfg[0].alpha_en = 1;
			channel_base_cfg[0].alpha = 255;
			channel_base_cfg[0].crop_width = 1080;
			channel_base_cfg[0].crop_height = 1920;
			channel_base_cfg[1].channel = IAR_CHANNEL_3;
			channel_base_cfg[1].pri = 0;
			channel_base_cfg[1].width = 1080;
			channel_base_cfg[1].height = 1920;
			channel_base_cfg[1].buf_width = 1080;
			channel_base_cfg[1].buf_height = 1920;
			channel_base_cfg[1].format = 4;//ARGB8888
			channel_base_cfg[1].alpha_sel = 0;
			channel_base_cfg[1].ov_mode = 0;
			channel_base_cfg[1].alpha_en = 1;
			channel_base_cfg[1].alpha = 128;
			channel_base_cfg[1].crop_width = 1080;
			channel_base_cfg[1].crop_height = 1920;

			output_cfg.out_sel = 0;//mipi-dsi
			output_cfg.width = 1080;
			output_cfg.height = 1920;
			output_cfg.bgcolor = 16744328;//white.

			iar_channel_base_cfg(&channel_base_cfg[0]);
			iar_channel_base_cfg(&channel_base_cfg[1]);
			iar_output_cfg(&output_cfg);

			writel(0x0572300f, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
			writel(0x406, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);

			iar_switch_buf(0);
			iar_set_bufaddr(IAR_CHANNEL_3, &graphic_display_paddr);
			iar_update();
			break;
		case MIPI_720P_TOUCH:
			disp_set_panel_timing(&video_720x1280_touch);
			channel_base_cfg[0].enable = 1;
			channel_base_cfg[1].enable = 1;
			channel_base_cfg[0].channel = IAR_CHANNEL_1;
			channel_base_cfg[0].pri = 2;
			channel_base_cfg[0].width = 720;
			channel_base_cfg[0].height = 1280;
			channel_base_cfg[0].buf_width = 720;
			channel_base_cfg[0].buf_height = 1280;
			channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
			channel_base_cfg[0].alpha_sel = 0;
			channel_base_cfg[0].ov_mode = 0;
			channel_base_cfg[0].alpha_en = 1;
			channel_base_cfg[0].alpha = 255;
			channel_base_cfg[0].crop_width = 720;
			channel_base_cfg[0].crop_height = 1280;
			channel_base_cfg[1].channel = IAR_CHANNEL_3;
			channel_base_cfg[1].pri = 0;
			channel_base_cfg[1].width = 720;
			channel_base_cfg[1].height = 1280;
			channel_base_cfg[1].buf_width = 720;
			channel_base_cfg[1].buf_height = 1280;
			channel_base_cfg[1].format = 4;//ARGB8888
			channel_base_cfg[1].alpha_sel = 0;
			channel_base_cfg[1].ov_mode = 0;
			channel_base_cfg[1].alpha_en = 1;
			channel_base_cfg[1].alpha = 128;
			channel_base_cfg[1].crop_width = 720;
			channel_base_cfg[1].crop_height = 1280;

			output_cfg.out_sel = 0;//mipi-dsi
			output_cfg.width = 720;
			output_cfg.height = 1280;
			output_cfg.bgcolor = 16744328;//white.

			iar_channel_base_cfg(&channel_base_cfg[0]);
			iar_channel_base_cfg(&channel_base_cfg[1]);
			iar_output_cfg(&output_cfg);

			writel(0x0572300f, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
			writel(0x406, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
			iar_switch_buf(0);
			iar_set_bufaddr(IAR_CHANNEL_3, &graphic_display_paddr);
			iar_update();
			break;
		case MIPI_720P_H:
			disp_set_panel_timing(&video_1280x720);
			channel_base_cfg[0].enable = 1;
			channel_base_cfg[1].enable = 1;
			channel_base_cfg[0].channel = IAR_CHANNEL_1;
			channel_base_cfg[0].pri = 2;
			channel_base_cfg[0].width = 1280;
			channel_base_cfg[0].height = 720;
			channel_base_cfg[0].buf_width = 1280;
			channel_base_cfg[0].buf_height = 720;
			channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
			channel_base_cfg[0].alpha_sel = 0;
			channel_base_cfg[0].ov_mode = 0;
			channel_base_cfg[0].alpha_en = 1;
			channel_base_cfg[0].alpha = 255;
			channel_base_cfg[0].crop_width = 1280;
			channel_base_cfg[0].crop_height = 720;
			channel_base_cfg[1].channel = IAR_CHANNEL_3;
			channel_base_cfg[1].pri = 0;
			channel_base_cfg[1].width = 1280;
			channel_base_cfg[1].height = 720;
			channel_base_cfg[1].buf_width = 1280;
			channel_base_cfg[1].buf_height = 720;
			channel_base_cfg[1].format = 4;//ARGB8888
			channel_base_cfg[1].alpha_sel = 0;
			channel_base_cfg[1].ov_mode = 0;
			channel_base_cfg[1].alpha_en = 1;
			channel_base_cfg[1].alpha = 128;
			channel_base_cfg[1].crop_width = 1280;
			channel_base_cfg[1].crop_height = 720;

			output_cfg.out_sel = 0;//mipi-dsi
			output_cfg.width = 1280;
			output_cfg.height = 720;
			output_cfg.bgcolor = 16744328;//white.

			iar_channel_base_cfg(&channel_base_cfg[0]);
			iar_channel_base_cfg(&channel_base_cfg[1]);
			iar_output_cfg(&output_cfg);

			writel(0x0572300f, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
			writel(0x406, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
			iar_switch_buf(0);
			iar_set_bufaddr(IAR_CHANNEL_3, &graphic_display_paddr);
			iar_update();
			break;
		case BT656_TYPE:
			disp_set_panel_timing(&video_704x576);
			channel_base_cfg[0].enable = 1;
			channel_base_cfg[1].enable = 1;
			channel_base_cfg[0].channel = IAR_CHANNEL_1;
			channel_base_cfg[0].pri = 2;
			channel_base_cfg[0].width = 704;
			channel_base_cfg[0].height = 576;
			channel_base_cfg[0].buf_width = 704;
			channel_base_cfg[0].buf_height = 576;
			channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
			channel_base_cfg[0].alpha_sel = 0;
			channel_base_cfg[0].ov_mode = 0;
			channel_base_cfg[0].alpha_en = 1;
			channel_base_cfg[0].alpha = 255;
			channel_base_cfg[0].crop_width = 704;
			channel_base_cfg[0].crop_height = 576;
			channel_base_cfg[1].channel = IAR_CHANNEL_3;
			channel_base_cfg[1].pri = 0;
			channel_base_cfg[1].width = 704;
			channel_base_cfg[1].height = 576;
			channel_base_cfg[1].buf_width = 704;
			channel_base_cfg[1].buf_height = 576;
			channel_base_cfg[1].format = 4;//ARGB8888
			channel_base_cfg[1].alpha_sel = 0;
			channel_base_cfg[1].ov_mode = 0;
			channel_base_cfg[1].alpha_en = 1;
			channel_base_cfg[1].alpha = 128;
			channel_base_cfg[1].crop_width = 704;
			channel_base_cfg[1].crop_height = 576;

			output_cfg.out_sel = 3;//bt656
			output_cfg.width = 704;
			output_cfg.height = 576;
			output_cfg.bgcolor = 16744328;//white.

			iar_channel_base_cfg(&channel_base_cfg[0]);
			iar_channel_base_cfg(&channel_base_cfg[1]);
			iar_output_cfg(&output_cfg);

			writel(0x0572300f, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
			writel(0x406, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);

			iar_switch_buf(0);
			iar_set_bufaddr(IAR_CHANNEL_3, &graphic_display_paddr);
			iar_update();
			break;
		case SIF_IPI:
			disp_set_panel_timing(&video_1920x1080);
			channel_base_cfg[0].enable = 1;
			channel_base_cfg[1].enable = 1;
			channel_base_cfg[0].channel = IAR_CHANNEL_1;
			channel_base_cfg[0].pri = 2;
			channel_base_cfg[0].width = 1920;
			channel_base_cfg[0].height = 1080;
			channel_base_cfg[0].buf_width = 1920;
			channel_base_cfg[0].buf_height = 1080;
			channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
			channel_base_cfg[0].alpha_sel = 0;
			channel_base_cfg[0].ov_mode = 0;
			channel_base_cfg[0].alpha_en = 1;
			channel_base_cfg[0].alpha = 255;
			channel_base_cfg[0].crop_width = 1920;
			channel_base_cfg[0].crop_height = 1080;
			channel_base_cfg[1].channel = IAR_CHANNEL_3;
			channel_base_cfg[1].pri = 0;
			channel_base_cfg[1].width = 1920;
			channel_base_cfg[1].height = 1080;
			channel_base_cfg[1].buf_width = 1920;
			channel_base_cfg[1].buf_height = 1080;
			channel_base_cfg[1].format = 4;//ARGB8888
			channel_base_cfg[1].alpha_sel = 0;
			channel_base_cfg[1].ov_mode = 0;
			channel_base_cfg[1].alpha_en = 1;
			channel_base_cfg[1].alpha = 128;
			channel_base_cfg[1].crop_width = 1920;
			channel_base_cfg[1].crop_height = 1080;

			output_cfg.out_sel = 4;
			output_cfg.width = 1920;
			output_cfg.height = 1080;
			output_cfg.bgcolor = 16744328;//white.

			iar_channel_base_cfg(&channel_base_cfg[0]);
			iar_channel_base_cfg(&channel_base_cfg[1]);
			iar_output_cfg(&output_cfg);

			writel(0x0572300f, g_iar_dev->regaddr + REG_IAR_OVERLAY_OPT);
			//panel color type is yuv444, YCbCr conversion needed
			writel(8, g_iar_dev->regaddr + REG_IAR_REFRESH_CFG);
			//select BT709 color domain
			writel(0x406, g_iar_dev->regaddr + REG_IAR_FORMAT_ORGANIZATION);
			iar_switch_buf(0);
			iar_set_bufaddr(IAR_CHANNEL_3, &graphic_display_paddr);
			iar_set_bufaddr(IAR_CHANNEL_4, &graphic1_display_paddr);
			iar_update();
			break;
		default:
			break;
	}
	iar_pixel_clk_disable();
	disable_sif_mclk();
	return 0;
}
EXPORT_SYMBOL_GPL(user_config_display);

static int hobot_xj3_iar_memory_alloc(phys_addr_t paddr, void *vaddr,
		unsigned int video_num, unsigned int fb_num,
		unsigned int size_nv12, unsigned int size_rgba)
{
#ifdef USE_ION_MEM
	if (fb_num == 0 && video_num == 0) {
		if(logo_paddr != 0) {
			pr_err("%s: fb number should not be 0 , if display logo!\n", __func__);
			return -1;
		}
		g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = 0;
		g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = NULL;
		g_iar_dev->frambuf[IAR_CHANNEL_2].paddr = 0;
		g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr = NULL;
		g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = 0;
		g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = NULL;
		g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = 0;
		g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = NULL;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr = 0;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr = NULL;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr = 0;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr = NULL;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr = 0;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr = NULL;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].paddr = 0;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].vaddr = NULL;
	} else if (fb_num > 0 && video_num == 0) {
		if (logo_paddr == 0) {
			g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = paddr;
			g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = vaddr;
			g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = paddr + (fb_num - 1) * size_rgba;
			g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = vaddr + (fb_num - 1) * size_rgba;
		} else {
			if (fb_num == 1) {
				g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = logo_paddr;
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = logo_vaddr;
				g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = logo_paddr;
				g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = logo_vaddr;
			} else {
				g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = logo_paddr;
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = logo_vaddr;
				g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = paddr;
				g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = vaddr;
			}
		}
	} else if (fb_num == 0 && video_num > 0) {
		if(logo_paddr != 0) {
                        pr_err("%s: fb number should not be 0!\n",
					__func__);
                        return -1;
                }
		g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = 0;
		g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = NULL;
		g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = 0;
		g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = NULL;
		g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = paddr;
		g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = vaddr;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr = paddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr = vaddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr =
			paddr + size_nv12 * 2;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr =
			vaddr + size_nv12 * 2;
		g_iar_dev->frambuf[IAR_CHANNEL_2].paddr =
			paddr + (video_num -1) * 3 * size_nv12;
		g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr =
			vaddr + (video_num -1) * 3 * size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr =
			g_iar_dev->frambuf[IAR_CHANNEL_2].paddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr =
			g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].paddr =
			g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].vaddr =
			g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr + size_nv12;
	} else if (fb_num > 0 && video_num > 0) {
		if (logo_paddr == 0) {
			g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = paddr;
			g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = vaddr;
			g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = paddr + (fb_num - 1) * size_rgba;
			g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = vaddr + (fb_num - 1) * size_rgba;
			g_iar_dev->frambuf[IAR_CHANNEL_1].paddr =
                        g_iar_dev->frambuf[IAR_CHANNEL_4].paddr + size_rgba;
			g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr =
			g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr + size_rgba;
		} else {
			if (fb_num == 1) {
				g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = logo_paddr;
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = logo_vaddr;
				g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = logo_paddr;
				g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = logo_vaddr;
				g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = paddr;
				g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = vaddr;
			} else {
				g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = logo_paddr;
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = logo_vaddr;
				g_iar_dev->frambuf[IAR_CHANNEL_4].paddr = paddr;
				g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr = vaddr;
				g_iar_dev->frambuf[IAR_CHANNEL_1].paddr =
					g_iar_dev->frambuf[IAR_CHANNEL_4].paddr + size_rgba;
				g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr =
					g_iar_dev->frambuf[IAR_CHANNEL_4].vaddr + size_rgba;
			}
		}
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr =
			g_iar_dev->frambuf[IAR_CHANNEL_1].paddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr =
			g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr =
			g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr =
			g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr + size_nv12;
		g_iar_dev->frambuf[IAR_CHANNEL_2].paddr =
			g_iar_dev->frambuf[IAR_CHANNEL_1].paddr + (video_num - 1) * 3 * size_nv12;
		g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr =
			g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr + (video_num - 1) * 3 * size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr =
			g_iar_dev->frambuf[IAR_CHANNEL_2].paddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr =
			g_iar_dev->frambuf[IAR_CHANNEL_2].vaddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].paddr =
			g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].paddr + size_nv12;
		g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[1].vaddr =
			g_iar_dev->pingpong_buf[IAR_CHANNEL_2].framebuf[0].vaddr + size_nv12;
	}
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
	return 0;
#else
	return -1;
#endif
}

#ifndef CONFIG_HOBOT_XJ3
static int hobot_xj2_iar_memory_alloc(phys_addr_t logo_paddr, void *logo_vaddr,
		phys_addr_t mem_paddr, void *vaddr)
{
#ifdef USE_ION_MEM
	g_iar_dev->frambuf[IAR_CHANNEL_1].paddr = logo_paddr;
	g_iar_dev->frambuf[IAR_CHANNEL_1].vaddr = logo_vaddr;
	g_iar_dev->frambuf[IAR_CHANNEL_3].paddr = mem_paddr;
	g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr = vaddr;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr =
				mem_paddr + MAX_FRAME_BUF_SIZE;
	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr =
				vaddr + MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].paddr
		= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].paddr
		+ MAX_FRAME_BUF_SIZE;

	g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[1].vaddr
		= g_iar_dev->pingpong_buf[IAR_CHANNEL_1].framebuf[0].vaddr
		+ MAX_FRAME_BUF_SIZE;
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
#endif
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
	return 0;
}
#endif
static int hobot_iar_probe(struct platform_device *pdev)
{
	struct resource *res, *irq, *res_mipi;
	phys_addr_t mem_paddr;
	size_t mem_size;
	int ret = 0;
	struct device_node *np;
	struct resource r;
	void *vaddr;
	uint64_t pixel_rate;
	char *type;
	uint32_t i = 0;
	buf_addr_t channel_buf_addr_3;
	buf_addr_t channel_buf_addr_4;
	struct disp_timing default_timing = {80, 120, 48, 32, 43, 2};
	size_t iar_request_ion_size = 0;
	char* fb_num_str = NULL;
	char* temp = NULL;
	uint32_t timing[7] = {0};
	uint32_t need_startup_img = 0;
	uint32_t size_wh = 0;
	uint32_t size_nv12 = 0;
	uint32_t size_rgba = 0;

	pr_info("iar probe begin!!!\n");
	pr_debug("module input para: video layer number is %d\n", video_layer_num);
	pr_debug("                   video layer0 width is %d\n", display_out_width);
	pr_debug("                   video layer0 height is %d\n", display_out_height);
	pr_debug("                   framebuffer number is %d\n", fb_num);

	if (video_layer_num == 0 && fb_num == 0) {
		display_out_width = 1920;
		display_out_height = 1080;
	}
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

	g_iar_dev->debug_file_iar = debugfs_create_file("iar", 0664, NULL,
			g_iar_dev, &iar_debug_fops);
	if (!g_iar_dev->debug_file_iar)
		pr_err("Failed to create client debugfs at %s\n", "iar");

	g_iar_dev->sif_mclk = devm_clk_get(&pdev->dev, "sif_mclk");
        if (IS_ERR_OR_NULL(g_iar_dev->sif_mclk)) {
                dev_err(&pdev->dev, "failed to get sif_mclk\n");
                goto err1;
        }
	sif_mclk_is_open = __clk_is_enabled(g_iar_dev->sif_mclk);
	if (iar_enable_sif_mclk() != 0) {
		pr_err("Error enable sif mclk!!\n");
		goto err1;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	g_iar_dev->regaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(g_iar_dev->regaddr)) {
		pr_err("Error remap resource of iar register!\n");
		goto err1;
	}
#ifdef CONFIG_HOBOT_XJ3
	res_mipi = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	g_iar_dev->mipi_dsi_regaddr =
		devm_ioremap_resource(&pdev->dev, res_mipi);
	if (IS_ERR(g_iar_dev->mipi_dsi_regaddr)) {
		pr_err("Error remap resource of mipi dsi register!\n");
		goto err1;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
                        "default_display_type", &display_type);

	hb_disp_base_board_id = (uint32_t)simple_strtoul(base_board_name, NULL, 16);

	ret = of_property_read_u32(pdev->dev.of_node,
			"disp_panel_reset_pin", &panel_reset_pin);
	if (ret) {
		dev_err(&pdev->dev, "Filed to get panel_reset_pin\n");
		goto err1;
	}

#ifndef KBUILD_SRC
#define KBUILD_SRC "."
#endif
	EMBEDIMG(0, KBUILD_SRC "/drivers/soc/hobot/iar/bootlogo.bmp");
#endif
	g_iar_dev->rst = devm_reset_control_get(&pdev->dev, "iar");
	if (IS_ERR(g_iar_dev->rst)) {
		dev_err(&pdev->dev, "missing controller reset\n");
		goto err1;
	} else {
		if (logo == 0) {
			reset_control_assert(g_iar_dev->rst);
			udelay(2);
			reset_control_deassert(g_iar_dev->rst);
		}
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		goto err1;
	}
	g_iar_dev->irq = (int)irq->start;
	pr_debug("g_iar_dev->irq is %lld\n", irq->start);

	g_iar_dev->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(g_iar_dev->pinctrl)) {
		dev_warn(&pdev->dev, "pinctrl get none\n");
		g_iar_dev->pinctrl = NULL;
		g_iar_dev->pins_bt1120 = NULL;
		g_iar_dev->pins_bt656 = NULL;
		g_iar_dev->pins_mipi_dsi = NULL;
		g_iar_dev->pins_rgb = NULL;
	} else {
		g_iar_dev->pins_voltage = pinctrl_lookup_state(g_iar_dev->pinctrl,
					"bt1120_voltage_func");
		if (IS_ERR(g_iar_dev->pins_voltage)) {
			dev_info(&pdev->dev, "bt1120_voltage_func get error %ld\n",
					PTR_ERR(g_iar_dev->pins_voltage));
			g_iar_dev->pins_voltage = NULL;
		}

		if (g_iar_dev->pins_voltage) {
			ret = pinctrl_select_state(g_iar_dev->pinctrl, g_iar_dev->pins_voltage);
			if (ret) {
				dev_info(&pdev->dev, "bt1120_voltage_func set error %d\n", ret);
			}
		}
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
			dev_info(&pdev->dev, "rgb_gpio_func get error %ld\n",
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
		goto err1;
	}
	clk_prepare_enable(g_iar_dev->iar_pixel_clk);

	pixel_rate = clk_get_rate(g_iar_dev->iar_pixel_clk);
	pr_debug("%s: iar pixel rate is %lld\n", __func__, pixel_rate);

	init_waitqueue_head(&g_iar_dev->wq_head);
	ret = request_threaded_irq(g_iar_dev->irq, hobot_iar_irq, NULL, IRQF_TRIGGER_HIGH,
							   dev_name(&pdev->dev), g_iar_dev);
	disable_irq(g_iar_dev->irq);

	ret = of_property_read_u32(pdev->dev.of_node, "bt656_pin_group",
			&bt656_output_pin_group);
	if (ret) {
		pr_err("error get bt656 output pin group config, use low 8bit default!!\n");
		bt656_output_pin_group = 0;// default use low 8 bits
	}

	ret = of_property_read_u32(pdev->dev.of_node, "btclk_invert",
			&bt1120_clk_invert);
	if (ret) {
		pr_err("error get bt1120 clk invert config, use normal as default!!\n");
		bt1120_clk_invert = 0;// default not invert
	}

	ret = of_property_read_u32(pdev->dev.of_node, "startup-img",
			&need_startup_img);
	if (ret) {
		pr_err("error get startup img config!!\n");
		need_startup_img = 0;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"timing_1920x1080", timing, 7);
	if (ret == 0) {
		pixel_clk_video_1920x1080 = timing[0];
		video_1920x1080.hbp = timing[1];
		video_1920x1080.hfp = timing[2];
		video_1920x1080.hs = timing[3];
		video_1920x1080.vbp = timing[4];
		video_1920x1080.vfp = timing[5];
		video_1920x1080.vs = timing[6];
	} else {
		pixel_clk_video_1920x1080 = 163000000;
		pr_err("can't find timing for 1920*1080, use default!!\n");
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"timing_1280x720", timing, 7);
	if (ret == 0) {
		pixel_clk_video_1280x720 = timing[0];
		video_1280x720.hbp = timing[1];
		video_1280x720.hfp = timing[2];
		video_1280x720.hs = timing[3];
		video_1280x720.vbp = timing[4];
		video_1280x720.vfp = timing[5];
		video_1280x720.vs = timing[6];
	} else {
		pixel_clk_video_1280x720 = 74250000;
		pr_err("can't find timing for 1280*720, use default!!\n");
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"timing_800x480", timing, 7);
	if (ret == 0) {
		pixel_clk_video_800x480 = timing[0];
		video_800x480.hbp = timing[1];
		video_800x480.hfp = timing[2];
		video_800x480.hs = timing[3];
		video_800x480.vbp = timing[4];
		video_800x480.vfp = timing[5];
		video_800x480.vs = timing[6];
	} else {
		pixel_clk_video_800x480 = 32000000;
		pr_err("can't find timing for 800*480, use default!!\n");
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"timing_1080x1920", timing, 7);
	if (ret == 0) {
		pixel_clk_video_1080x1920 = timing[0];
		video_1080x1920.hbp = timing[1];
		video_1080x1920.hfp = timing[2];
		video_1080x1920.hs = timing[3];
		video_1080x1920.vbp = timing[4];
		video_1080x1920.vfp = timing[5];
		video_1080x1920.vs = timing[6];
	} else {
		pixel_clk_video_1080x1920 = 32000000;
		pr_err("can't find timing for 1080*1920, use default!!\n");
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"timing_720x1280_touch", timing, 7);
	if (ret == 0) {
		pixel_clk_video_720x1280_touch = timing[0];
		video_720x1280_touch.hbp = timing[1];
		video_720x1280_touch.hfp = timing[2];
		video_720x1280_touch.hs = timing[3];
		video_720x1280_touch.vbp = timing[4];
		video_720x1280_touch.vfp = timing[5];
		video_720x1280_touch.vs = timing[6];
	} else {
		pixel_clk_video_720x1280_touch = 54000000;
		pr_err("can't find timing for 720*1280 touch, use default!!\n");
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"timing_704x576", timing, 7);
	if (ret == 0) {
		pixel_clk_video_704x576 = timing[0];
		video_704x576.hbp = timing[1];
		video_704x576.hfp = timing[2];
		video_704x576.hs = timing[3];
		video_704x576.vbp = timing[4];
		video_704x576.vfp = timing[5];
		video_704x576.vs = timing[6];
	} else {
		pixel_clk_video_704x576 = 27000000;
		pr_err("can't find timing for 704*576 touch, use default!!\n");
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"timing_720x480", timing, 7);
	if (ret == 0) {
		pixel_clk_video_720x480 = timing[0];
		video_720x480.hbp = timing[1];
		video_720x480.hfp = timing[2];
		video_720x480.hs = timing[3];
		video_720x480.vbp = timing[4];
		video_720x480.vfp = timing[5];
		video_720x480.vs = timing[6];
	} else {
		pixel_clk_video_720x480 = 27000000;
		pr_err("can't find timing for 720*480 touch, use default!!\n");
	}

	ret = fb_get_options("hobot", &type);
	pr_debug("%s: fb get options display type is %s\n", __func__, type);
	if (type != NULL) {
#ifdef CONFIG_HOBOT_XJ3
		fb_num_str = strstr(type, "fb_num=");
		if (fb_num_str != NULL) {
			temp = fb_num_str + 7;
			ret = kstrtouint(temp, 0, &fb_num);
			if (ret) {
				pr_err("error fb num type, use default value!!\n");
				fb_num = 1;
			} else {
				if (fb_num > 2) {
					pr_err("error fb num, use default value!!\n");
					fb_num = 1;
				}
			}
		}
		pr_debug("fb_num_str is %s, fb_num is %d\n", fb_num_str, fb_num);
		if (strstr(type, "1080p") != NULL) {
			display_type = MIPI_1080P;
			pr_info("%s: panel type is MIPI_1080P\n", __func__);
		} else if (strstr(type, "dsi1280x720") != NULL) {
			display_type = MIPI_720P_H;
			pr_info("%s: panel type is MIPI 1280x720\n", __func__);
		} else if (strstr(type, "720p") != NULL) {
			display_type = MIPI_720P_TOUCH;
			pr_info("%s: panel type is MIPI_720P_TOUCH\n", __func__);
		} else if (strstr(type, "dsi720x1280") != NULL) {	//for SDB panel
			display_type = MIPI_720P_TOUCH;
			pr_info("%s: panel type is MIPI_720P_TOUCH\n", __func__);
		} else if (strstr(type, "lcd") != NULL) {
			display_type = LCD_7_TYPE;
			pr_info("%s: panel type is LCD_7_TYPE\n", __func__);
		} else if (strstr(type, "hdmi") != NULL) {
			display_type = HDMI_TYPE;
			pr_info("%s: panel type is HDMI_TYPE\n", __func__);
		} else if (strstr(type, "ipi") != NULL) {
			pr_info("%s: panel type is SIF IPI\n", __func__);
			display_type = SIF_IPI;
		} else if (strstr(type, "bt656") != NULL) {
			pr_info("%s: panel type is BT656\n", __func__);
			display_type = BT656_TYPE;
		}
#else
		if (display_type == LCD_7_TYPE) {
			if (strncmp(type, "mipi", 4) == 0)
				display_type = MIPI_720P;
		}
#endif
	}

	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
#ifndef USE_ION_MEM
		dev_err(&g_iar_dev->pdev->dev, "No %s specified\n", "memory-region");
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
		} else {
			logo_paddr = r.start;
			logo_vaddr = ioremap_nocache(r.start, MAX_FRAME_BUF_SIZE);
		}
	}
#ifdef USE_ION_MEM
	if (fb_num != 0 || video_layer_num != 0) {
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
	}

#ifdef CONFIG_HOBOT_XJ2
	g_iar_dev->iar_ihandle = ion_alloc(g_iar_dev->iar_iclient,
		IAR_MEM_SIZE - MAX_FRAME_BUF_SIZE, 0x20,
		ION_HEAP_CARVEOUT_MASK, 0);
#else
	if (fb_num != 0 || video_layer_num != 0) {
		size_wh = display_out_height * display_out_width;
		size_nv12 = size_wh * 3 / 2;
		size_rgba = size_wh * 4;
		if (logo_vaddr == NULL) {
			iar_request_ion_size = video_layer_num * size_nv12 * 3 + fb_num * size_rgba;
			g_iar_dev->iar_ihandle = ion_alloc(g_iar_dev->iar_iclient,
				iar_request_ion_size, 0x20,
				ION_HEAP_CARVEOUT_MASK, (IARIONTYPE << 16)|0);
		} else {
			if (fb_num > 0) {
				iar_request_ion_size = video_layer_num * size_nv12 * 3
					+ (fb_num - 1) * size_rgba;
			} else {
				iar_request_ion_size = video_layer_num * size_nv12 * 3;
			}
			g_iar_dev->iar_ihandle = ion_alloc(g_iar_dev->iar_iclient,
				iar_request_ion_size, 0x20,
				ION_HEAP_CARVEOUT_MASK, (IARIONTYPE << 16)|0);
		}
		pr_debug("iar request ion size is %ld\n", iar_request_ion_size);
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
	}
#else
	#ifdef CONFIG_HOBOT_XJ3
	if (resource_size(&r) < 0x1a00000) {
                pr_err("iar memory size is not large enough!(<3buffer)\n");
                goto err1;
        }
        mem_paddr = r.start;
        //for reserved 32M(4*8) memory,
        //three buffers for video, one buffer for graphic(fb)

        //channel 2&4 disabled
        vaddr = ioremap_nocache(r.start, resource_size(&r));
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

#ifdef CONFIG_HOBOT_XJ3
	hobot_xj3_iar_memory_alloc(mem_paddr, vaddr, video_layer_num,
			fb_num, size_nv12, size_rgba);
#else
	hobot_xj2_iar_memory_alloc(logo_paddr, logo_vaddr, mem_paddr, vaddr);
#endif

	if (display_type == LCD_7_TYPE) {
		iar_display_cam_no = PIPELINE0;
		iar_display_addr_type = DISPLAY_CHANNEL1;
		display_color_bar(800, 480, g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);
	} else if (display_type == MIPI_720P) {
		pr_debug("%s: display_type is mipi-720p-dsi panel!\n", __func__);
		ret = disp_set_pixel_clk(pixel_clk_video_720x1280);
		if (ret)
			return ret;
	} else if (display_type == MIPI_720P_TOUCH) {
		pr_info("%s: display_type is mipi-720p-dsi panel!\n", __func__);
		ret = disp_set_pixel_clk(pixel_clk_video_720x1280_touch);
		if (ret)
			return ret;
		iar_display_cam_no = PIPELINE0;
                iar_display_addr_type = GDC0;
		if (need_startup_img) {
			stride_copy_bmp(0, 0, embedded_image_0_data, 0, 0, 0, 0,
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr, 24, 24, 720);
		} else {
			display_color_bar(720, 1280, g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);
		}
		pr_debug("set mipi 720p touch done!\n");
	} else if (display_type == MIPI_720P_H) {
		pr_info("%s: display_type is mipi-1280x720-dsi panel!\n", __func__);
		ret = disp_set_pixel_clk(pixel_clk_video_1280x720);
		if (ret)
			return ret;
		iar_display_cam_no = PIPELINE0;
                iar_display_addr_type = DISPLAY_CHANNEL1;
		if (need_startup_img) {
			stride_copy_bmp(0, 0, embedded_image_0_data, 0, 0, 0, 0,
				g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr, 24, 24, 1280);
		} else {
			display_color_bar(1280, 720, g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);
		}
		pr_debug("set mipi 1280x720 display done!\n");
	} else if (display_type == BT656_TYPE) {
		pr_info("%s: display_type is BT656 panel!\n", __func__);
		ret = disp_set_pixel_clk(pixel_clk_video_704x576);
		if (ret)
			return ret;
		iar_display_cam_no = PIPELINE0;
		iar_display_addr_type = DISPLAY_CHANNEL1;
		display_color_bar(704, 576, g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);
		pr_debug("set bt656 panel done!\n");
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
			ret = disp_set_pixel_clk(pixel_clk_video_1920x1080);
		}
		if (ret)
			return ret;
		iar_display_cam_no = PIPELINE0;
		iar_display_addr_type = DISPLAY_CHANNEL1;
		if (need_startup_img) {
			stride_copy_bmp(0, 0, embedded_image_0_data, 0, 0, 0, 0,
					g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr, 24, 24, 1920);
		} else {
			display_color_bar(1920, 1080, g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);
		}
		pr_debug("set HDMI done!\n");
	} else if (display_type == MIPI_1080P) {
		pr_debug("iar_driver: disp set mipi 1080p!\n");
		pr_debug("iar_driver: output 1080*1920 color bar bgr!\n");
		disp_set_pixel_clk(pixel_clk_video_1080x1920);
		iar_display_cam_no = PIPELINE0;
		iar_display_addr_type = GDC1;
		// actual output 27.2Mhz(need 27Mhz)
		display_color_bar(1080, 1920, g_iar_dev->frambuf[IAR_CHANNEL_3].vaddr);
		pr_debug("set mipi 1080p done!\n");
	}
	if (logo == 0)
		disp_set_panel_timing(&default_timing);

	channel_buf_addr_3.addr = (uint32_t)g_iar_dev->frambuf[IAR_CHANNEL_3].paddr;
	channel_buf_addr_4.addr = (uint32_t)g_iar_dev->frambuf[IAR_CHANNEL_4].paddr;
	iar_switch_buf(0);
	iar_set_bufaddr(IAR_CHANNEL_3, &channel_buf_addr_3);
	iar_set_bufaddr(IAR_CHANNEL_4, &channel_buf_addr_4);
	iar_register_get_callback((int (*)(u8 *, u8 *))(ipu_get_iar_display_type));
	iar_register_set_callback(ipu_set_display_addr);
	iar_update();

	if (logo == 0) {
		clk_disable_unprepare(g_iar_dev->iar_pixel_clk);
		iar_disable_sif_mclk();
	}
	pr_info("iar probe end success!!!\n");
	return 0;
err1:
	devm_kfree(&pdev->dev, g_iar_dev);
	g_iar_dev = NULL;
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
	debugfs_remove_recursive(iar->debug_file_iar);
#ifdef USE_ION_MEM
	ion_client_destroy(g_iar_dev->iar_iclient);
#endif
	free_irq(g_iar_dev->irq, g_iar_dev);
	devm_kfree(&pdev->dev, g_iar_dev);
	dev_set_drvdata(&pdev->dev, NULL);
	g_iar_dev = NULL;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hobot_iar_of_match[] = {
	{.compatible = "hobot,hobot-iar"},
	{},
};
MODULE_DEVICE_TABLE(of, hobot_iar_of_match);
#endif

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

static int __init x3_iar_init(void)
{
    int ret = platform_driver_register(&hobot_iar_driver);
    if (ret)
        vio_err("platform_driver_register failed: %d\n", ret);

    return ret;
}

late_initcall(x3_iar_init);

static void __exit x3_iar_exit(void)
{
    platform_driver_unregister(&hobot_iar_driver);
}

module_exit(x3_iar_exit);
MODULE_LICENSE("GPL");
