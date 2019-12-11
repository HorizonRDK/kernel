/*
 * Copyright rui.guo@horizon.ai
 */
#ifndef __X2_LT9211_H__
#define __X2_LT9211_H__

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include "../../iar/x2_iar.h"

#define LT9211_SLAVE_ADDR 0x2D

#define MAX_FRAME_BUF_SIZE  (1920*1080*4)

#define ENABLE 0x1
#define DISABLE 0x0

#define CONFIG_LT9211_DEBUG (DISABLE)

#if (CONFIG_LT9211_DEBUG == ENABLE)
#define LT9211_DEBUG(format, args...)	\
	pr_info("LT9211 debug: " format, ## args)
#else
#define LT9211_DEBUG(format, args...)
#endif

extern int lt9211_reset_pin;
extern int lcd_reset_pin;
extern int display_type;
extern struct x2_lt9211_s *g_x2_lt9211;
//extern struct ips_dev_s *g_ipsdev;
enum MIPI_LANE_NUM {
	MIPI_1LANE = 0x10,
	MIPI_2LANE = 0x20,
	MIPI_4LANE = 0x00
};
#define MIPI_LaneNum MIPI_4LANE

enum MIPI_VIDEO_MODE {
	MIPI_BurstMode,
	MIPI_NonBurst_SyncPulse_Mode,
	MIPI_NonBurst_SyncEvent_Mode
};
#define MIPI_VideoMode MIPI_BurstMode

struct x2_lt9211_s {
	struct i2c_client *client;
	const char *name;
	int major;
	int minor;
	struct cdev cdev;
	dev_t dev_num;
	struct class *x2_lt9211_classes;
	struct mutex lt9211_mutex;
};

struct video_timing {
	u16 hfp;
	u16 hs;
	u16 hbp;
	u16 hact;
	u16 htotal;
	u16 vfp;
	u16 vs;
	u16 vbp;
	u16 vact;
	u16 vtotal;
	u32 pclk_khz;
};

//typedef enum VideoFormat
//{
//    video_640x480_60Hz_vic = 1,
//    video_1280x720_60Hz_vic,
//    video_1366x768_60Hz_vic,
//    video_1280x1024_60Hz_vic,
//    video_1920x1080_60Hz_vic,
//    video_1920x1200_60Hz_vic,
//    video_none
//};
enum VideoFormat {
	video_384x292_60Hz_vic,
	video_640x480_60Hz_vic,
	video_1280x720_60Hz_vic,
	video_1366x768_60Hz_vic,
	video_1280x1024_60Hz_vic,
	video_1920x1080_60Hz_vic,
	video_1920x1200_60Hz_vic,
	video_1920x1080_25Hz_vic,
	video_720x1280_60Hz_vic,
	video_none
};
#define _Mipi_PortA_
//#define _Mipi_PortB_


enum LT9211_OUTPUTMODE_ENUM {
	OUTPUT_RGB888 = 0,
	OUTPUT_BT656_8BIT = 1,
	OUTPUT_BT1120_16BIT = 2,
	OUTPUT_LVDS_2_PORT = 3,
	OUTPUT_LVDS_1_PORT = 4,
	OUTPUT_YCbCr444 = 5,
	OUTPUT_YCbCr422_16BIT
};
#define LT9211_OutPutModde  OUTPUT_RGB888

//typedef enum VIDEO_INPUTMODE_ENUM {
//	Input_RGB888,
//	Input_YCbCr444,
//	Input_YCbCr422_16BIT,
//} _Video_Input_Mode;
enum VIDEO_INPUTMODE_ENUM {
	Input_RGB888,
	Input_YCbCr444,
	Input_YCbCr422_16BIT,
};
#define Video_Input_Mode  Input_YCbCr422_16BIT

//int ips_set_iar_clk(void);
int set_lt9211_config(struct fb_info *fb);
int lt9211_dsi_lcd_init(void);
int x2_lt9211_write_byte(struct i2c_client *client, uint32_t addr,
		uint8_t reg, uint8_t val);
int x2_lt9211_read_byte(struct i2c_client *client, uint32_t addr,
		uint8_t reg, uint8_t *val);
int x2_write_lt9211(uint8_t reg_addr, uint8_t reg_val);
int x2_read_lt9211(uint8_t reg_addr, uint8_t *reg_val);
int lt9211_reset(void);
int lt9211_reset_first(void);
int lt9211_config(unsigned int convert_type);
int lt9211_chip_id(void);
int lt9211_system_int_to_mipi(void);
int lt9211_ttl_rx_phy_to_mipi(void);
int lt9211_csc_to_mipi(void);
int lt9211_video_check_to_mipi(void);
int lt9211_set_timing_para_to_mipi(void);
int lt9211_mipi_tx_phy(void);
int lt9211_mipi_tx_pll(void);
int lt9211_set_tx_timing(void);
int lt9211_mipi_tx_digital(void);
int lt9211_system_int_to_RGB(void);
int lt9211_ttl_rx_phy_to_RGB(void);
int lt9211_BT_video_check_to_RGB(void);
int lt9211_set_timing_para_to_RGB(void);
int lt9211_tx_digital_RGB(void);
int lt9211_RGB_tx_phy(void);
int lt9211_RGB_tx_pll(void);
int lt9211_rx_csc(void);
int lt9211_mipi_clock_debug(void);
void lt9211_mipi_patten(struct video_timing *video_format);
int lt9211_set_mipi_tx_timing(struct video_timing *video_format);

int init_panel(unsigned int pannel_type);
int LCD_reset(void);
int generic_short_write_1P(uint8_t data0, uint8_t data1);
int dcs_pkt_write(uint8_t dcs_di, uint8_t len, uint8_t *ptr);
int set_lcd_backlight(unsigned int backlight_level);
int set_lt9211_output_ctrl(unsigned int on_off);


#endif
