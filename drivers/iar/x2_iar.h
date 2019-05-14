#ifndef __X2_IAR_H__
#define __X2_IAR_H__

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/fb.h>
#include <linux/slab.h>

#define MAX_FRAME_BUF_SIZE	(1920*1080*4)

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
	ppcon1_cfg_t ppcon1;
	ppcon2_cfg_t ppcon2;
	refresh_cfg_t refresh_cfg;
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
	OUTPUT_MIPI = 0,
	OUTPUT_BT1120 = 1,
	OUTPUT_RGB888 = 2,
};
enum DISPLAY_TYPE {
	LCD_7_TYPE,
	HDMI_TYPE,
};

extern int display_type;
extern unsigned int iar_debug_level;
#define IAR_DEBUG_PRINT(format, args...)	\
	do {									\
		if(iar_debug_level)					\
			printk("IAR debug: " format, ## args);		\
	} while(0)
int ips_set_iar_clk32(void);
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
void x2_iar_dump(void);
frame_buf_t* x2_iar_get_framebuf_addr(int channel);

#endif //__X2_IAR_H__
