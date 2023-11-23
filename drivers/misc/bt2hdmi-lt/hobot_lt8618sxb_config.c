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
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include "hobot_lt8618sxb.h"

// LT8618SX RGB 输入，HDMI输出，寄存器设置
//#define   _LT8618SX_ADR        0x76
#define _LT8618SX_ADR 0x3B	// IIC Address，If CI2CA pins are low(0~400mV)
#define LT8618SXB_SLAVE_ADDR 0x3B
#define _Ver_U3_		// LT8618SXB IC version
#define BYPASS_INIT 
//#define _LOG_
//#define _Phase_Debug_
//#define DEBUG_I
//#define _Ver_U2_ // LT8618SXB IC version

/*--------------------------------
  CI2CA            I2C address
  -----------------------
  (0~400mV)----------0x72(default)
  (400mV~800mV)------0x7a
  (800mV~1200mV)-----0x90
  (1200mV~1600mV)----0x92

  (2000mV~2400mV)----0x94
  (2400mV~2800mV)----0x7e
  (2800mV~3300mV)----0x76
  -----------------------------------*/

//extern hobot_write_lt8618sxb(u8 RegAddr, u8 data);     // IIC write operation
//extern u8 hobot_read_lt8618sxb(u8 RegAddr);            // IIC read operation
edid_raw_t edid_raw_data;
extern struct x2_lt8618sxb_s *g_x2_lt8618sxb;
extern struct gpio_desc *lt8618sxb_reset_gpio;
void Resolution_change(hobot_hdmi_sync_t* timing);
void Debug_DispNum(u8 msg)
{
	pr_debug("0x%x\n", msg);
}
hobot_hdmi_sync_t sys_fs_edid;
static int hobot_lt8618sxb_write_byte(struct i2c_client *client,
		uint32_t addr, uint8_t reg, uint8_t val)
{
	struct i2c_msg msg;
	uint8_t buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);	//发送数据
#ifdef DEBUG_I
	printk("reg buffer: 0x%x\n", buf[0]);
	printk("Write adr: 0x%x;reg: 0x%x;val: 0x%x\n", addr, reg, val);
#endif
	if (ret >= 0)
		return 0;

	pr_err("lt8618sxb i2c write error addr:0x%x reg:0x%x ret %d !\n",
			addr, reg, ret);

	return ret;
}

static int hobot_write_lt8618sxb(uint8_t reg_addr, uint8_t reg_val)
{
	if (g_x2_lt8618sxb == NULL) return -1;
	return hobot_lt8618sxb_write_byte(g_x2_lt8618sxb->client,
			LT8618SXB_SLAVE_ADDR, reg_addr,
			reg_val);
}

static int hobot_lt8618sxb_read_byte(struct i2c_client *client, uint32_t addr,
		uint8_t reg, uint8_t * val)
{
	struct i2c_msg msg[2];
	uint8_t buf[1];
	int ret;

	buf[0] = reg & 0xFF;

	msg[0].addr = addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
#ifdef DEBUG_I
	printk("reg buffer: 0x%x\n", buf[0]);
	printk("Read adr: 0x%x;reg: 0x%x;val: 0x%x\n", addr, reg, *val);
#endif
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}
	pr_err("lt8618sxb i2c read error addr: 0x%x reg: 0x%x ret %d !!!\n",
			addr, reg, ret);
	return ret;
}

static uint8_t hobot_read_lt8618sxb(uint8_t reg_addr)
{
	uint8_t reg_val = 0;
	int ret = 0;

	if (g_x2_lt8618sxb == NULL) return -1;

	ret = hobot_lt8618sxb_read_byte(g_x2_lt8618sxb->client,
			LT8618SXB_SLAVE_ADDR, reg_addr,
			&reg_val);
	if (ret) {
		pr_err("hobot read lt8618sxb error!\n");
	}

	return reg_val;
}

//--------------------------------------------------------//

// 1、Reset LT8618SX

// 2、LT8618SX Initial setting:

//--------------------------------------------------------//

// #define _LT8618_HDCP_ //If HDMI encrypted output is not required,
// mask this macro definition

//--------------------------------------------------------//
// Use the synchronization signal inside the bt1120 signal
#define _Embedded_sync_
// need to provide external h sync, V sync and de signals to lt8618sxb
// #define _External_sync_

//--------------------------------------------------------//
// 如果 BT1120 是单沿采样（SDR）的话，输入的CLK就是pixel CLK；
// 如果是双沿采样（DDR）的话，输入的CLK就是pixel CLK的一半。
bool Use_DDRCLK;		// 1: DDR mode; 0: SDR (normal) mode

//--------------------------------------------------------//
//BT1120 Mapping(embedded sync): Y and C can be swapped
/******************************************************************************
  Output        16-bit        16-bit         20-bit        24-bit
  Port        Mapping1    Mapping2    Mapping        Mapping
  -------|------------|---------|---------|---------|-----------|---------|-------
  D0             Y[0]        X            X             Y[0]
  D1             Y[1]        X            X             Y[1]
  D2             Y[2]        X            Y[0]         Y[2]
  D3             Y[3]        X            Y[1]         Y[3]
  D4             Y[4]        X            Y[2]         Y[4]
  D5             Y[5]        X            Y[3]         Y[5]
  D6             Y[6]        X            Y[4]         Y[6]
  D7             Y[7]        X            Y[5]         Y[7]
  D8             C[0]        Y[0]         Y[6]         Y[8]
  D9             C[1]        Y[1]         Y[7]         Y[9]
  D10            C[2]        Y[2]         Y[8]        Y[10]
  D11         C[3]        Y[3]         Y[9]        Y[11]
  D12         C[4]        Y[4]         X            C[0]
  D13         C[5]        Y[5]         X            C[1]
  D14         C[6]        Y[6]         C[0]        C[2]
  D15         C[7]        Y[7]         C[1]        C[3]
  D16         X            C[0]        C[2]        C[4]
  D17         X            C[1]        C[3]        C[5]
  D18         X            C[2]        C[4]        C[6]
  D19         X            C[3]        C[5]        C[7]
  D20         X            C[4]        C[6]        C[8]
  D21         X            C[5]        C[7]        C[9]
  D22         X            C[6]        C[8]        C[10]
  D23         X            C[7]        C[9]        C[11]

  HS            X            X            X            X            X
  VS            X            X            X            X            X
  DE            X            X            X            X            X
 *****************************************************************************/

//YC422 Mapping(Separate  sync): // _External_sync_;
/*****************************************************************************
  Output        16-bit        16-bit         20-bit        24-bit
  Port        Mapping1    Mapping2    Mapping        Mapping
  -------|------------|---------|---------|---------|-----------|---------|-----
  D0          Y[0]        X            X             Y[0]
  D1             Y[1]        X            X             Y[1]
  D2             Y[2]        X            Y[0]         Y[2]
  D3             Y[3]        X            Y[1]         Y[3]
  D4             Y[4]        X            X             C[0]
  D5             Y[5]        X            X             C[1]
  D6             Y[6]        X            C[0]         C[2]
  D7             Y[7]        X            C[1]         C[3]
  D8             C[0]        Y[0]         Y[2]         Y[4]
  D9             C[1]        Y[1]         Y[3]         Y[5]
  D10            C[2]        Y[2]         Y[4]        Y[6]
  D11         C[3]        Y[3]         Y[5]        Y[7]
  D12         C[4]        Y[4]         Y[6]        Y[8]
  D13         C[5]        Y[5]         Y[7]        Y[9]
  D14         C[6]        Y[6]         Y[8]        Y[10]
  D15         C[7]        Y[7]         Y[9]        Y[11]
  D16         X            C[0]        C[2]        C[4]
  D17         X            C[1]        C[3]        C[5]
  D18         X            C[2]        C[4]        C[6]
  D19         X            C[3]        C[5]        C[7]
  D20         X            C[4]        C[6]        C[8]
  D21         X            C[5]        C[7]        C[9]
  D22         X            C[6]        C[8]        C[10]
  D23         X            C[7]        C[9]        C[11]

  HS            Hsync        Hsync        Hsync        Hsync        Hsync
  VS            Vsync        Vsync        Vsync        Vsync        Vsync
  DE            DE            DE            DE            DE            DE
 ******************************************************************************/

#define _16bit_			// 16bit YC
//#define _20bit_  // 20bit YC
//#define _24bit_ // 24bit YC

#ifdef _16bit_
// BT1120 16bit
// BT1120 input from D0 to D15 of LT8618SXB pins. // D0 ~ D7 Y ; D8 ~ D15 C
#define _D0_D15_In_ 0x30
// BT1120 input from D8 to D23 of LT8618SXB pins. // D8 ~ D15 Y ; D16 ~ D23 C
#define _D8_D23_In_ 0x70
// BT1120 input from D0 to D15 of LT8618SXB pins. // D0 ~ D7 C ; D8 ~ D15 Y
#define _D0_D15_In_2_ 0x00
// BT1120 input from D8 to D23 of LT8618SXB pins. // D8 ~ D15 C ; D16 ~ D23 Y
#define _D8_D23_In_2_ 0x60

#define _YC_Channel_ _D8_D23_In_2_

#define _Reg0x8246_ 0x00
// bit1/bit0 = 0:Input data color depth is 8 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x00
#else
//==========================================//
#ifdef _Embedded_sync_

// 20bit
#ifdef _20bit_
// BT1120 input from D2 ~ D11 Y & D14 ~ D23 C of LT8618SXB pins.
#define _D2_D23_In_ 0x0b
// BT1120 input from D2 ~ D11 C & D14 ~ D23 Y of LT8618SXB pins.
#define _D2_D23_In_2_ 0x00

#define _YC_Channel_ 0x00
#define _Reg0x8246_ _D2_D23_In_2_	// input setting
// bit1 = 0;bit0 = 1: Input data color depth is 10 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x01

#else
// 24bit
// BT1120 input from D0 ~ D11 Y & D12 ~ D23 C of LT8618SXB pins.
#define _D0_D23_In_ 0x0b
// BT1120 input from D0 ~ D11 C & D12 ~ D23 Y of LT8618SXB pins.
#define _D0_D23_In_2_ 0x00

#define _YC_Channel_ 0x00
#define _Reg0x8246_ _D0_D23_In_2_	// input setting
// bit1 = 1;bit0 = 0: Input data color depth is 12 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x02

#endif
//==========================================//

#else //YC422 Mapping(Separate  sync): // _External_sync_;

#define _YC_swap_en_ 0x70
#define _YC_swap_dis_ 0x60

#define _YC_Channel_ _YC_swap_dis_	// input setting

#define _Reg0x8246_ 0X00

#ifdef _20bit_
// bit1 = 0;bit0 = 1: Input data color depth is 10 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x01
#else // 24bit
// bit1 = 1;bit0 = 0: Input data color depth is 12 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x02
#endif

#endif
#endif

//--------------------------------------------------------//
#define _Read_TV_EDID_
#ifdef _Read_TV_EDID_
u8 Sink_EDID[256];
u8 Sink_EDID2[256];
#endif

u8 I2CADR;

//--------------------------------------------------------//

u8 VIC_Num;			// vic ,0x10: 1080P ;  0x04 : 720P ;

//--------------------------------------------------------//

bool flag_Ver_u3 = 0;

//--------------------------------------------------------//

enum {
	_32KHz = 0,
	_44d1KHz,
	_48KHz,

	_88d2KHz,
	_96KHz,
	_176Khz,
	_196KHz
};

u16 IIS_N[] = {
	4096,			// 32K
	6272,			// 44.1K
	6144,			// 48K
	12544,			// 88.2K
	12288,			// 96K
	25088,			// 176K
	24576			// 196K
};

u16 Sample_Freq[] = {
	0x30,			// 32K
	0x00,			// 44.1K
	0x20,			// 48K
	0x80,			// 88.2K
	0xa0,			// 96K
	0xc0,			// 176K
	0xe0			//  196K
};

//************************************/

#ifdef _External_sync_
u16 hfp, hs_width, hbp, h_act, h_tal, v_act, v_tal, vfp, vs_width, vbp;
bool hs_pol, vs_pol;
#else
u16 h_act, h_tal, v_act;
#endif

u32 CLK_Cnt;

//--------------------------------------------------------//

u8 CLK_bound;
//#define CLK_bound _Bound_60_90M
// SDR:720P 60/50Hz; DDR:1080P 60/50Hz. -- 74.25M

//#ifdef _Ver_U3_
static u8 LT8618SXB_PLL_u3[3][3] = {
	{0x00, 0x9e, 0xaa},	// < 50MHz
	{0x00, 0x9e, 0x99},	// 50 ~ 100M
	{0x00, 0x9e, 0x88},	// > 100M
};

//#else // u2 version
static u8 LT8618SXB_PLL_u2[3][3] = {
	{0x00, 0x94, 0xaa},	// < 50MHz
	{0x01, 0x94, 0x99},	// 50 ~ 100M
	{0x03, 0x94, 0x88},	// > 100M
};

//#endif
//#endif

enum {
	_Less_than_50M = 0x00,
	//    _Bound_25_50M,
	_Bound_50_100M,
	_Greater_than_100M
};

//--------------------------------------------------------//

u8 Resolution_Num;

#define _16_9_ 0x2A		// 16:9
#define _4_3_ 0x19		// 4:3

static int Format_Timing[][14] = {
	// H_FP / H_sync / H_BP / H_act / H_total / V_FP / V_sync / V_BP /
	// V_act / V_total / Vic / Pic_Ratio / Clk_bound(SDR) / Clk_bound(DDR)
	{16, 62, 60, 720, 858, 9, 6, 30, 480, 525, 2,
		_4_3_, _Less_than_50M, _Less_than_50M},	// 480P 60Hz 27MHz 4:3
	// { 16,    62,  60,  720,    858,  9, 6, 30, 480,  525,    3,
	// _16_9_, _Less_than_50M,     _Less_than_50M\},    // 480P 60Hz 27MHz 16:9
	{12, 64, 68, 720, 864, 5, 5, 39, 576, 625, 17,
		_4_3_, _Less_than_50M, _Less_than_50M},	// 576P 50Hz 27MHz 4:3
	// { 12,    64,  68,  720,    864,  5, 5, 39, 576,  625,    18,
	// _16_9_, _Less_than_50M,     _Less_than_50M},    // 576P 50Hz 27MHz 16:9
	{110, 40, 220, 1280, 1650, 5, 5, 20, 720, 750, 4,
		_16_9_, _Bound_50_100M, _Less_than_50M},	// 720P 60Hz 74.25MHz
	{440, 40, 220, 1280, 1980, 5, 5, 20, 720, 750, 19,
		_16_9_, _Bound_50_100M, _Less_than_50M},	// 720P 50Hz 74.25MHz
	{1760, 40, 220, 1280, 3300, 5, 5, 20, 720, 750, 62,
		_16_9_, _Bound_50_100M, _Less_than_50M},	// 720P 30Hz 74.25MHz
	{2420, 40, 220, 1280, 3960, 5, 5, 20, 720, 750, 61,
		_16_9_, _Bound_50_100M, _Less_than_50M},	// 720P 25Hz 74.25MHz
	{88, 44, 148, 1920, 2200, 4, 5, 36, 1080, 1125, 16,
		_16_9_, _Greater_than_100M, _Bound_50_100M},	// 1080P  60Hz 148.5MHz
	{528, 44, 148, 1920, 2640, 4, 5, 36, 1080, 1125, 31,
		_16_9_, _Greater_than_100M, _Bound_50_100M},	// 1080P  50Hz 148.5MHz
	{88, 44, 148, 1920, 2200, 4, 5, 36, 1080, 1125, 34,
		_16_9_, _Bound_50_100M, _Less_than_50M},	// 1080P  30Hz 74.25MHz
	{528, 44, 148, 1920, 2640, 4, 5, 36, 1080, 1125, 33,
		_16_9_, _Bound_50_100M, _Less_than_50M},	// 1080P  25Hz 74.25MHz
	// { 638,    44,     148, 1920, 2750, 4, 5,     36, 1080, 1125, 33,
	// _16_9_, _Bound_50_100M,     _Less_than_50M},  // 1080P  24Hz 74.25MHz
	{88, 44, 148, 1920, 2200, 2, 5, 15, 540, 562, 5, _16_9_,
		_Bound_50_100M, _Less_than_50M},	// 1080i  60Hz 74.25MHz
	{528, 44, 148, 1920, 2640, 2, 5, 25, 540, 562, 20, _16_9_,
		_Bound_50_100M, _Less_than_50M},	// 1080i  50Hz 74.25MHz
	{176, 88, 296, 3840, 4400, 8, 10, 72, 2160, 2250, 95, _16_9_,
		_Greater_than_100M, _Greater_than_100M},	// 4K 30Hz    297MHz
	{40, 128, 88, 800, 1056, 1, 4, 23, 600, 628, 0, _4_3_,
		_Less_than_50M, _Less_than_50M},	// VESA 800x600 40MHz
	{24, 136, 160, 1024, 1344, 3, 6, 29, 768, 806, 0, _4_3_,
		_Bound_50_100M, _Less_than_50M},	// VESA 1024X768 65MHz
	{5, 13, 270, 1024, 1312, 2, 3, 17, 600, 622, 0, _4_3_,
		_Less_than_50M, _Less_than_50M},	// VESA 1024X600 65MHz	
	{44, 88, 124, 800, 1056, 3, 6, 46, 480, 535, 0, _4_3_,
		_Less_than_50M, _Less_than_50M},	// VESA 800X480 65MHz	
	{70, 143, 213, 1366, 1792, 3, 3, 24, 768, 798, 0, _16_9_,
		_Bound_50_100M, _Bound_50_100M},	// VESA 800X480 65MHz	
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0},	// default	
	/*
	   {},
	   {},
	   {},
	   {},
	// */
};

enum {
	H_fp = 0,
	H_sync,
	H_bp,
	H_act,
	H_tol,

	V_fp,
	V_sync,
	V_bp,
	V_act,
	V_tol,

	Vic,

	Pic_Ratio,		// Image proportion

	Clk_bound_SDR,		// SDR
	Clk_bound_DDR		// DDR
};



enum {
	_480P60_ = 0,
	_576P50_,

	_720P60_,
	_720P50_,
	_720P30_,
	_720P25_,

	_1080P60_,
	_1080P50_,
	_1080P30_,
	_1080P25_,

	_1080i60_,
	_1080i50_,

	_4K30_,

	_800x600P60_,
	_1024x768P60_,
	_1024x600_,
	_800x480_,
	_1366x768_,
};

//--------------------------------------------------------//

#ifdef _LT8618_HDCP_
/***********************************************************/

void LT8618SXB_HDCP_Init(void)
{
	hobot_write_lt8618sxb(0xff, 0x85);
	hobot_write_lt8618sxb(0x03, 0xc2);
	hobot_write_lt8618sxb(0x07, 0x1f);

	hobot_write_lt8618sxb(0xff, 0x80);
	hobot_write_lt8618sxb(0x13, 0xfe);
	hobot_write_lt8618sxb(0x13, 0xff);

	hobot_write_lt8618sxb(0x14, 0x00);
	hobot_write_lt8618sxb(0x14, 0xff);

	hobot_write_lt8618sxb(0xff, 0x85);
	hobot_write_lt8618sxb(0x07, 0x1f);
	hobot_write_lt8618sxb(0x13, 0xfe);
	hobot_write_lt8618sxb(0x17, 0x0f);
	hobot_write_lt8618sxb(0x15, 0x05);
}

/***********************************************************/

void LT8618SXB_HDCP_Enable(void)
{
	hobot_write_lt8618sxb(0xff, 0x80);
	hobot_write_lt8618sxb(0x14, 0x00);
	hobot_write_lt8618sxb(0x14, 0xff);

	hobot_write_lt8618sxb(0xff, 0x85);
	hobot_write_lt8618sxb(0x15, 0x01);	//disable HDCP
	hobot_write_lt8618sxb(0x15, 0x71);	//enable HDCP
	hobot_write_lt8618sxb(0x15, 0x65);	//enable HDCP
}

/***********************************************************/

void LT8618SXB_HDCP_Disable(void)
{
	hobot_write_lt8618sxb(0xff, 0x85);
	hobot_write_lt8618sxb(0x15, 0x45);	//enable HDCP
}
#endif

/***********************************************************

 ***********************************************************/

int LT8618SXB_Read_EDID(hobot_hdmi_sync_t * sync)
{
#ifdef _Read_TV_EDID_
	//printk("LT8618SXB_Read_EDID in\n");
	u8 i, j, x;
	u8 extended_flag = 0x00;
	u32 max_retry = 20;
	u32 retry = 0;
	if (g_x2_lt8618sxb == NULL) return -1;
	sync->vic = 0;
	sync->pic_ratio = _4_3_;
	edid_raw_data.block_num = -1;
	hobot_write_lt8618sxb(0xff, 0x85);
	//hobot_write_lt8618sxb(0x02,0x0a); //I2C 100K
	hobot_write_lt8618sxb(0x03, 0xc9);
	hobot_write_lt8618sxb(0x04, 0xa0);	//0xA0 is EDID device address
	hobot_write_lt8618sxb(0x05, 0x00);	//0x00 is EDID offset address
	hobot_write_lt8618sxb(0x06, 0x20);	//length for read
	hobot_write_lt8618sxb(0x14, 0x7f);

	for (i = 0; i < 8; i++) { // block 0 & 1
		hobot_write_lt8618sxb(0x05, i * 32);	//0x00 is EDID offset address
		hobot_write_lt8618sxb(0x07, 0x36);
		hobot_write_lt8618sxb(0x07, 0x34);
		hobot_write_lt8618sxb(0x07, 0x37);
		msleep(5);	// wait 5ms for reading edid data.
		if (hobot_read_lt8618sxb(0x40) & 0x02) {
			//DDC No Ack or Abitration lost

				for( retry = 0; retry < max_retry; retry++ ){
					if(!(hobot_read_lt8618sxb(0x40) & 0x50)){
						break;
					}else
						continue;
				}
				if(retry==max_retry)
				{
					pr_err("retry:%d\n",retry);
					pr_err("\r\nread edid failed: no ack");
					goto end;
				}
				//printk("LT8618SXB_Read_EDID 11 \r\n");
				for (j = 0; j < 32; j++) {
					Sink_EDID[i * 32 + j] =
						hobot_read_lt8618sxb(0x83);
					//printk("Sink_EDID[%d * 32 + %d] =  %u\n", i,j,Sink_EDID[i * 32 + j]);
					if(i * 32 + j == 0x3a){
						if(sync != NULL){
							sync->hact = (Sink_EDID[0x3a] & 0xf0) * 16 + Sink_EDID[0x38];
							sync->htotal = (Sink_EDID[0x3a] & 0xf0) * 16 + Sink_EDID[0x38] + ((Sink_EDID[0x3a] & 0x0f ) * 0x100 + Sink_EDID[0x39]);
						}
						//printk("EDID_Timing[hact] = %d\n",(Sink_EDID[0x3a] & 0xf0) * 16 + Sink_EDID[0x38]);
						//printk("EDID_Timing[htotal] = %d\n",(Sink_EDID[0x3a] & 0xf0) * 16 + Sink_EDID[0x38] + ((Sink_EDID[0x3a] & 0x0f ) * 0x100 + Sink_EDID[0x39]));
					}
					if(i * 32 + j == 0x3d){
						if(sync != NULL){
							sync->vact = (Sink_EDID[0x3d] & 0xf0) * 16 + Sink_EDID[0x3b];
							sync->vtotal = (Sink_EDID[0x3d] & 0xf0) * 16 + Sink_EDID[0x3b] + ((Sink_EDID[0x3d] & 0x03)*0x100 + Sink_EDID[0x3c] );
						}
						//printk("EDID_Timing[vact] = %d\n",(Sink_EDID[0x3d] & 0xf0) * 16 + Sink_EDID[0x3b]);
						//printk("EDID_Timing[vtotal] = %d\n",(Sink_EDID[0x3d] & 0xf0) * 16 + Sink_EDID[0x3b] + ((Sink_EDID[0x3d] & 0x03)*0x100 + Sink_EDID[0x3c] ));
					}
					if(i * 32 + j == 0x37){
						if(sync != NULL){
							sync->clk = Sink_EDID[0x37] * 0x100 + Sink_EDID[0x36];
						}
						//printk("EDID_Timing[pclk_10khz] = %d\n",Sink_EDID[0x37] * 0x100 + Sink_EDID[0x36]);
					}
					if(i * 32 + j == 0x41){
						if(sync != NULL){
							sync->hfp = (Sink_EDID[0x41] & 0xC0 ) * 4 + Sink_EDID[0x3e];
							sync->hs = (Sink_EDID[0x41] & 0x30 ) * 16 + Sink_EDID[0x3f];
							sync->vfp = (Sink_EDID[0x41] & 0x0c ) * 4 + (Sink_EDID[0x40] & 0xf0) / 16;
							sync->vs = (Sink_EDID[0x41] & 0x03 ) * 16 + (Sink_EDID[0x40] & 0x0f);
							sync->hbp = ((Sink_EDID[0x3a] & 0x0f ) * 0x100 + Sink_EDID[0x39]) - ((Sink_EDID[0x41] & 0x30 )* 16 + Sink_EDID[0x3f])- ((Sink_EDID[0x41] & 0x0c)*4 + Sink_EDID[0x3e]);
							sync->vbp = ((Sink_EDID[0x3d] & 0x03 ) * 0x100 + Sink_EDID[0x3c]) - ((Sink_EDID[0x41] & 0x03)* 16 + (Sink_EDID[0x40] & 0x0f)) -((Sink_EDID[0x41] & 0x0c)*4 +(Sink_EDID[0x40] & 0xf0) / 16);
						}
						//printk("EDID_Timing[hfp] = %d\n",(Sink_EDID[0x41] & 0xC0 ) * 4 + Sink_EDID[0x3e]);
						//printk("EDID_Timing[hs] = %d\n",(Sink_EDID[0x41] & 0x30 ) * 16 + Sink_EDID[0x3f]);
						//printk("EDID_Timing[vfp] = %d\n",(Sink_EDID[0x41] & 0x0c ) * 4 + (Sink_EDID[0x40] & 0xf0) / 16);
						//printk("EDID_Timing[vs] = %d\n",(Sink_EDID[0x41] & 0x03 ) * 16 + (Sink_EDID[0x40] & 0x0f));
						//printk("EDID_Timing[hbp] = %d\n",((Sink_EDID[0x3a] & 0x0f ) * 0x100 + Sink_EDID[0x39]) - ((Sink_EDID[0x41] & 0x30 )* 16 + Sink_EDID[0x3f])- ((Sink_EDID[0x41] & 0x0c)*4 + Sink_EDID[0x3e]));
						//printk("EDID_Timing[vbp] = %d\n",((Sink_EDID[0x3d] & 0x03 ) * 0x100 + Sink_EDID[0x3c]) - ((Sink_EDID[0x41] & 0x03)* 16 + (Sink_EDID[0x40] & 0x0f)) -((Sink_EDID[0x41] & 0x0c)*4 +(Sink_EDID[0x40] & 0xf0) / 16));
					}
					//if(i * 32 + j == 0x47){
					//	printk("EDID_H_polarity = %d,EDID_V_polarity = %d\n",Sink_EDID[0x47]& 0x20,Sink_EDID[0x47]& 0x40);
					//}
					//    edid_data = hobot_read_lt8618sxb(0x83);
					if ((i == 3) && (j == 30)) {
						//    extended_flag = edid_data & 0x03;
						extended_flag =
							Sink_EDID[i * 32 +
							j] & 0x03;
						pr_debug("EDID:extended_flag:%d\n",extended_flag);
					}
					//    printf("%02bx,", edid_data);
				}
				if (i == 3) {
					if (extended_flag < 1) { //no block 1, stop reading edid.
						hobot_write_lt8618sxb(0x03, 0xc2);
						hobot_write_lt8618sxb(0x07, 0x1f);
						pr_debug("EDID:Only 1 block\n");
						//printk("LT8618SXB_Read_EDID 11 out\n");
						return 0;
					}
				}
		} else {
			pr_debug("\r\nread edid failed: accs not done");
			goto end;
		}
	}
	//printk("LT8618SXB_Read_EDID 111111\n");
	for(x = 0; x < 17; x++){
		//printk("LT8618SXB_Read_EDID x = %d\n",x);
		if(sync != NULL){
			//printk("Format_Timing[x][0] = %d,sync->hfp = %d,Format_Timing[x][1]=%d,sync->hs =%d,Format_Timing[x][2]=%d,sync->hbp=%d,Format_Timing[x][3]=%d,sync->hact=%d,Format_Timing[x][4]=%d,sync->htotal,Format_Timing[x][5]=%d,sync->vfp=%d,Format_Timing[x][6]=%d,sync->vs=%d,Format_Timing[x][7]=%d,sync->vbp=%d,Format_Timing[x][8]=%d,sync->vact=%d,Format_Timing[x][9]=%d,sync->vtotal=%d",
			//Format_Timing[x][0],sync->hfp,Format_Timing[x][1],sync->hs,Format_Timing[x][2],sync->hbp,Format_Timing[x][3],sync->hact,Format_Timing[x][4],sync->htotal,Format_Timing[x][5],sync->vfp,Format_Timing[x][6],sync->vs,Format_Timing[x][7],sync->vbp,Format_Timing[x][8],sync->vact,Format_Timing[x][9],sync->vtotal);
			if(Format_Timing[x][0] == sync->hfp && Format_Timing[x][1] == sync->hs && Format_Timing[x][2] == sync->hbp && Format_Timing[x][3] == sync->hact &&
				Format_Timing[x][4] == sync->htotal && Format_Timing[x][5] == sync->vfp && Format_Timing[x][6] == sync->vs && Format_Timing[x][7] == sync->vbp &&
				Format_Timing[x][8] == sync->vact && Format_Timing[x][9] == sync->vtotal ){
					sync->vic = Format_Timing[x][10];
					sync->pic_ratio = Format_Timing[x][11];
				}
		}
	}
	if (extended_flag < 2) { //no block 2, stop reading edid.
		hobot_write_lt8618sxb(0x03, 0xc2);
		hobot_write_lt8618sxb(0x07, 0x1f);
		pr_debug("EDID:Only 2 block\n");
		memcpy(&edid_raw_data.edid_data,Sink_EDID,256*sizeof(u8));
		edid_raw_data.block_num = 1;
		return 0;
	}

	for (i = 0; i < 8; i++) {
		hobot_write_lt8618sxb(0x05, i * 32);
		hobot_write_lt8618sxb(0x07, 0x76);
		hobot_write_lt8618sxb(0x07, 0x74);
		hobot_write_lt8618sxb(0x07, 0x77);
		msleep(5);	// wait 5ms for reading edid data.
		if (hobot_read_lt8618sxb(0x40) & 0x02) {
			if (hobot_read_lt8618sxb(0x40) & 0x50) {
				pr_err("\r\nread edid failed: no ack");
				goto end;
			} else {
				for (j = 0; j < 32; j++) {
					Sink_EDID2[i * 32 + j] =
						hobot_read_lt8618sxb(0x83);
				}
				if (i == 3) {
					if (extended_flag < 3) { //no block 1, stop reading edid.
						break;
					}
				}
			}
		} else {
			pr_debug("\r\nread edid failed: accs not done");
			goto end;
		}
	}
	edid_raw_data.block_num = 2;
	memcpy(&edid_raw_data.edid_data2,Sink_EDID2,256*sizeof(u8));
	//printf("\r\nread edid succeeded, checksum = ",Sink_EDID[255]);
end:
	hobot_write_lt8618sxb(0x03, 0xc2);
	hobot_write_lt8618sxb(0x07, 0x1f);
	return -1;
#endif
	
	//#endif
}

//------------------------------------------------------------

int LT8618SXB_Chip_ID(void)
{

	int ret, count = 0;
	uint8_t id0 = 0, id1 = 0, id2 = 0;

	pr_debug("*****LT8618SXB_Chip_ID***** enter\n");
	ret = hobot_write_lt8618sxb(0xFF, 0x80);
	if (ret) {
		printk("hobot write lt8618sxb err!\n");
		return ret;
	}
	ret = hobot_write_lt8618sxb(0xee, 0x01);
	if (ret) {
		printk("hobot write lt8618sxb err!\n");
		return ret;
	}

	while (1) {
		id0 = hobot_read_lt8618sxb(0x00);
		id1 = hobot_read_lt8618sxb(0x01);
		id2 = hobot_read_lt8618sxb(0x02);
		if (id0 == 0x17 && id1 == 0x02 && (id2 & 0xfc) == 0xe0) {
			break;
		}
		count++;
		if (count > 5) {
			return -1;
		}
		msleep(100);
	}

#ifdef _LOG_
	printk("\r\nRead LT8618SXB ID ");
	printk("\r\n ");
	printk("\r\nLT8618SXB Chip ID = 0x%x", hobot_read_lt8618sxb(0x00));	// 0x17
	printk(" ", hobot_read_lt8618sxb(0x01));	// 0x02
	printk(" ", hobot_read_lt8618sxb(0x02));	// 0xE1 / 0xE2
	printk("\r\n ");
#endif

	/**************************************
	  _Ver_U2 ID: 0x17 0x02 0xE1
	  _Ver_U3 ID: 0x17 0x02 0xE2
	 **************************************/
	if (id2 == 0xe2) {
		flag_Ver_u3 = 1;	// u3
	} else if (id2 == 0xe1) {
		flag_Ver_u3 = 0;	// u2
	}
	return 0;
}

// The threshold value of high HPD detected by lt8618sxb is 1.2V
u8 LT8618SXB_HPD_status(void)
{
	//printk("*****LT8618SXB_HPD_status*****\n");
	u8 HPD_Status = 0;

	hobot_write_lt8618sxb(0xff, 0x82);	// Register bank
	if ((hobot_read_lt8618sxb(0x5e) & 0x05) == 0x05) {
		HPD_Status = 1;	// HPD is High
	}

	return HPD_Status;
}

void LT8618SXB_RST_PD_Init(void)
{
	//printk("*****LT8618SXB_RST_PD_Init*****\n");
	hobot_write_lt8618sxb(0xff, 0x80);
	hobot_write_lt8618sxb(0x11, 0x00);	//reset MIPI Rx logic.
	hobot_write_lt8618sxb(0x13, 0xf1);
	hobot_write_lt8618sxb(0x13, 0xf9);	// Reset TTL video process
}

void LT8618SXB_HDMI_TX_En(bool enable)
{
	//printk("*****LT8618SXB_HDMI_TX_En*****\n");

	hobot_write_lt8618sxb(0xff, 0x81);
	if (enable) {
		hobot_write_lt8618sxb(0x30, 0xea);
	} else {
		hobot_write_lt8618sxb(0x30, 0x00);
	}
}

//------------------------------------------------------------

//_Embedded_sync_ mode，lt8618sxb can only detect h total and H / V active.
//_External_sync_ mode, lt8618sxb can detects a detailed timing value.
void LT8618SXB_Video_check(void)
{
	pr_debug("*****LT8618SXB_Video_check*****\n");
	hobot_write_lt8618sxb(0xff, 0x82);	//video

#ifdef _Embedded_sync_
	h_tal = hobot_read_lt8618sxb(0x8f);
	h_tal = (h_tal << 8) + hobot_read_lt8618sxb(0x90);
	pr_debug("h_total = 0x%x\n",h_tal);

	v_act = hobot_read_lt8618sxb(0x8b);
	v_act = (v_act << 8) + hobot_read_lt8618sxb(0x8c);
	pr_debug("v_active = 0x%x\n",v_act);
	Debug_DispNum(v_act);

	h_act = hobot_read_lt8618sxb(0x8d);
	h_act = (h_act << 8) + hobot_read_lt8618sxb(0x8e);
	pr_debug("h_active = 0x%x\n",h_act);
	Debug_DispNum(h_act);

	CLK_Cnt = (hobot_read_lt8618sxb(0x1d) & 0x0f) *
		0x10000 + hobot_read_lt8618sxb(0x1e) *
		0x100 + hobot_read_lt8618sxb(0x1f);
	//Pixel_CLK =    CLK_Cnt * 1000;

	pr_debug("Read LT8618SXB Video Check:\n");
	pr_debug("h_total = 0x%x\n",h_tal);
	Debug_DispNum(h_tal);

	pr_debug("h_active = 0x%x\n",h_act);
	pr_debug("v_active = 0x%x\n",v_act);

#else
	u8 temp;

	vs_pol = 0;
	hs_pol = 0;

	// _External_sync_
	temp = hobot_read_lt8618sxb(0x70);	//hs vs polarity
	if (temp & 0x02) {
		vs_pol = 1;
	}
	if (temp & 0x01) {
		hs_pol = 1;
	}

	vs_width = hobot_read_lt8618sxb(0x71);

	hs_width = hobot_read_lt8618sxb(0x72);
	hs_width = (hs_width << 8) + hobot_read_lt8618sxb(0x73);

	vbp = hobot_read_lt8618sxb(0x74);
	vfp = hobot_read_lt8618sxb(0x75);

	hbp = hobot_read_lt8618sxb(0x76);
	hbp = (hbp << 8) + hobot_read_lt8618sxb(0x77);

	hfp = hobot_read_lt8618sxb(0x78);
	hfp = (hfp << 8) + hobot_read_lt8618sxb(0x79);

	v_tal = hobot_read_lt8618sxb(0x7a);
	v_tal = (v_tal << 8) + hobot_read_lt8618sxb(0x7b);

	h_tal = hobot_read_lt8618sxb(0x7c);
	h_tal = (h_tal << 8) + hobot_read_lt8618sxb(0x7d);

	v_act = hobot_read_lt8618sxb(0x7e);
	v_act = (v_act << 8) + hobot_read_lt8618sxb(0x7f);

	h_act = hobot_read_lt8618sxb(0x80);
	h_act = (h_act << 8) + hobot_read_lt8618sxb(0x81);

	CLK_Cnt =
		(hobot_read_lt8618sxb(0x1d) & 0x0f) * \\0x10000 +
		hobot_read_lt8618sxb(0x1e) * \\0x100 + hobot_read_lt8618sxb(0x1f);
	// pixel CLK =    CLK_Cnt * 1000

	pr_debug("\r\nRead LT8618SXB Video Check:");

	pr_debug("\r\n ");
	pr_debug("\r\nh_fp, hs_wid, h_bp, h_act, h_tol = ");
	Debug_DispNum(hfp);
	pr_debug(", ");
	Debug_DispNum(hs_width);
	pr_debug(", ");
	Debug_DispNum(hbp);
	pr_debug(", ");
	Debug_DispNum(h_act);
	pr_debug(", ");
	Debug_DispNum(h_tal);

	pr_debug("\r\n ");
	pr_debug("\r\nv_fp, vs_wid, v_bp, v_act, v_tol = ");
	Debug_DispNum(vfp);
	pr_debug(", ");
	Debug_DispNum(vs_width);
	pr_debug(", ");
	Debug_DispNum(vbp);
	pr_debug(", ");
	Debug_DispNum(v_act);
	pr_debug(", ");
	Debug_DispNum(v_tal);
#endif
}

/***********************************************************

 ***********************************************************/
void LT8618SXB_Reset(void)
{
	//printk("*****LT8618SXB_Reset*****\n");
	if (lt8618sxb_reset_gpio) {
		gpiod_set_value_cansleep(lt8618sxb_reset_gpio, 0);
		msleep(100);
		gpiod_set_value_cansleep(lt8618sxb_reset_gpio, 1);
		msleep(100);
	}
}

/***********************************************************

 ***********************************************************/
void LT8618SXB_TTL_Input_Analog(void)
{
	//printk("*****LT8618SXB_TTL_Input_Analog*****\n");
	// TTL mode
	hobot_write_lt8618sxb(0xff, 0x81);	// register bank
	hobot_write_lt8618sxb(0x02, 0x66);
	hobot_write_lt8618sxb(0x0a, 0x06);
	hobot_write_lt8618sxb(0x15, 0x06);

	hobot_write_lt8618sxb(0x4e, 0xa8);

	hobot_write_lt8618sxb(0xff, 0x82);
	hobot_write_lt8618sxb(0x1b, 0x75);
	hobot_write_lt8618sxb(0x1c, 0x30);	// 30000
}

/***********************************************************

 ***********************************************************/
void LT8618SXB_TTL_Input_Digtal(void)
{
	//printk("*****LT8618SXB_TTL_Input_Digtal*****\n");
	hobot_write_lt8618sxb(0xff, 0x80);
#ifdef _Embedded_sync_
	// Internal generate sync/de control logic clock enable
	hobot_write_lt8618sxb(0x0A, 0xF0);
#else // _External_sync_
	hobot_write_lt8618sxb(0x0A, 0xC0);
#endif

	// TTL_Input_Digtal
	hobot_write_lt8618sxb(0xff, 0x82);	// register bank
	hobot_write_lt8618sxb(0x45, _YC_Channel_);	// YC channel swap
	hobot_write_lt8618sxb(0x46, _Reg0x8246_);
	hobot_write_lt8618sxb(0x50, 0x00);
	//*
	if (Use_DDRCLK) {
		hobot_write_lt8618sxb(0x4f, 0x80);	// 0x80: dclk
	} else {
		hobot_write_lt8618sxb(0x4f, 0x40);	// 0x40: txpll_clk
	}
	//*/
	//    hobot_write_lt8618sxb(0x4f, 0x40);

#ifdef _Embedded_sync_
	hobot_write_lt8618sxb(0x51, 0x42);	// Select BT rx decode det_vs/hs/de
	// Embedded sync mode input enable.
	hobot_write_lt8618sxb(0x48, 0x08 + _Reg0x8248_D1_D0_);
#else // _External_sync_
	// Select TTL process module input video data
	hobot_write_lt8618sxb(0x51, 0x00);
	hobot_write_lt8618sxb(0x48, 0x00 + _Reg0x8248_D1_D0_);
#endif
}

/***********************************************************

 ***********************************************************/
void LT8618SXB_PLL_setting(void)
{
	//printk("*****LT8618SXB_PLL_setting*****\n");
	u8 read_val;
	u8 j;
	u8 cali_done;
	u8 cali_val;
	u8 lock;

	if (g_x2_lt8618sxb == NULL) return -1;

	CLK_bound =
		(u8) Format_Timing[Resolution_Num][Clk_bound_SDR +
		(u8) (Use_DDRCLK)];

	hobot_write_lt8618sxb(0xff, 0x81);
	hobot_write_lt8618sxb(0x23, 0x40);
	if (flag_Ver_u3) {
		hobot_write_lt8618sxb(0x24, 0x62);	//icp set
	} else {
		hobot_write_lt8618sxb(0x24, 0x64);	//icp set
	}
	hobot_write_lt8618sxb(0x26, 0x55);

	hobot_write_lt8618sxb(0x29, 0x04);	//for U3 for U3 SDR/DDR fixed phase

	if (flag_Ver_u3) {
		hobot_write_lt8618sxb(0x25, LT8618SXB_PLL_u3[CLK_bound][0]);
		hobot_write_lt8618sxb(0x2c, LT8618SXB_PLL_u3[CLK_bound][1]);
		hobot_write_lt8618sxb(0x2d, LT8618SXB_PLL_u3[CLK_bound][2]);
	} else {
		hobot_write_lt8618sxb(0x25, LT8618SXB_PLL_u2[CLK_bound][0]);
		hobot_write_lt8618sxb(0x2c, LT8618SXB_PLL_u2[CLK_bound][1]);
		hobot_write_lt8618sxb(0x2d, LT8618SXB_PLL_u2[CLK_bound][2]);
	}

	if (Use_DDRCLK) {
		if (flag_Ver_u3) {
			hobot_write_lt8618sxb(0x4d, 0x05);
			hobot_write_lt8618sxb(0x27, 0x60);	//0x60 //ddr 0x66
			hobot_write_lt8618sxb(0x28, 0x88);
		} else {
			read_val = hobot_read_lt8618sxb(0x2c) & 0x7f;
			read_val = read_val * 2 | 0x80;
			hobot_write_lt8618sxb(0x2c, read_val);

			hobot_write_lt8618sxb(0x4d, 0x04);
			hobot_write_lt8618sxb(0x27, 0x60);	//0x60
			hobot_write_lt8618sxb(0x28, 0x88);
		}

#ifdef _LOG_
		printk("\r\n PLL DDR");
#endif
	} else {
		if (flag_Ver_u3) {
			hobot_write_lt8618sxb(0x4d, 0x0d);
			hobot_write_lt8618sxb(0x27, 0x60);	//0x06
			hobot_write_lt8618sxb(0x28, 0x88);	// 0x88
		} else {
			hobot_write_lt8618sxb(0x4d, 0x00);
			hobot_write_lt8618sxb(0x27, 0x60);	//0x06
			hobot_write_lt8618sxb(0x28, 0x00);	// 0x88
		}

#ifdef _LOG_
		printk("\r\n PLL SDR");
#endif
	}

	hobot_write_lt8618sxb(0xff, 0x81);

	read_val = hobot_read_lt8618sxb(0x2b);
	hobot_write_lt8618sxb(0x2b, read_val & 0xfd);	// sw_en_txpll_cal_en

	read_val = hobot_read_lt8618sxb(0x2e);
	hobot_write_lt8618sxb(0x2e, read_val & 0xfe);	//sw_en_txpll_iband_set

	hobot_write_lt8618sxb(0xff, 0x82);
	hobot_write_lt8618sxb(0xde, 0x00);
	hobot_write_lt8618sxb(0xde, 0xc0);

	hobot_write_lt8618sxb(0xff, 0x80);
	hobot_write_lt8618sxb(0x16, 0xf1);
	hobot_write_lt8618sxb(0x18, 0xdc);	//txpll _sw_rst_n
	hobot_write_lt8618sxb(0x18, 0xfc);
	hobot_write_lt8618sxb(0x16, 0xf3);

	hobot_write_lt8618sxb(0xff, 0x81);

	//#ifdef _Ver_U3_
	if (flag_Ver_u3) {
		if (Use_DDRCLK) {
			hobot_write_lt8618sxb(0x2a, 0x10);
			hobot_write_lt8618sxb(0x2a, 0x30);
		} else {
			hobot_write_lt8618sxb(0x2a, 0x00);
			hobot_write_lt8618sxb(0x2a, 0x20);
		}
	}
	//#endif

	for (j = 0; j < 0x05; j++) {
		msleep(10);
		hobot_write_lt8618sxb(0xff, 0x80);
		hobot_write_lt8618sxb(0x16, 0xe3);	/* pll lock logic reset */
		hobot_write_lt8618sxb(0x16, 0xf3);

		hobot_write_lt8618sxb(0xff, 0x82);
		lock = 0x80 & hobot_read_lt8618sxb(0x15);
		cali_val = hobot_read_lt8618sxb(0xea);
		cali_done = 0x80 & hobot_read_lt8618sxb(0xeb);

		if (lock && cali_done && (cali_val != 0xff)
				&& (cali_val >= 0x20)) {
#ifdef _LOG_
			printk("TXPLL Lock");
			hobot_write_lt8618sxb(0xff, 0x82);
			printk("0x82ea=%x\n", hobot_read_lt8618sxb(0xea));
			printk("0x82eb=%x\n", hobot_read_lt8618sxb(0xeb));
			printk("0x82ec=%x\n", hobot_read_lt8618sxb(0xec));
			printk("0x82ed=%x\n", hobot_read_lt8618sxb(0xed));
			printk("0x82ee=%x\n", hobot_read_lt8618sxb(0xee));
			printk("0x82ef=%x\n", hobot_read_lt8618sxb(0xef));

#endif
		} else {
			hobot_write_lt8618sxb(0xff, 0x80);
			hobot_write_lt8618sxb(0x16, 0xf1);
			hobot_write_lt8618sxb(0x18, 0xdc);	//txpll _sw_rst_n
			hobot_write_lt8618sxb(0x18, 0xfc);
			hobot_write_lt8618sxb(0x16, 0xf3);
#ifdef _LOG_
			printk("TXPLL Reset\n");
#endif
		}
	}
}

/***********************************************************

 ***********************************************************/
void LT8618SXB_Audio_setting(void)
{
	//printk("*****LT8618SXB_Audio_setting*****\n");
	//----------------IIS-----------------------
	// IIS Input
	hobot_write_lt8618sxb(0xff, 0x82);	// register bank
	// bit7 = 0 : DVI output; bit7 = 1: HDMI output
	hobot_write_lt8618sxb(0xd6, 0x0e);
	hobot_write_lt8618sxb(0xd7, 0x04);	// sync polarity

	hobot_write_lt8618sxb(0xff, 0x84);	// register bank
	hobot_write_lt8618sxb(0x06, 0x08);
	hobot_write_lt8618sxb(0x07, 0x10);	// SD0 channel selected

	hobot_write_lt8618sxb(0x09, 0x00);	// 0x00 :Left justified; default
	// 0x02 :Right justified;

	//-----------------SPDIF---------------------

	/*    SPDIF Input
	      hobot_write_lt8618sxb(0xff, 0x82);// register bank
	      hobot_write_lt8618sxb(0xd6, 0x8e);
	      hobot_write_lt8618sxb(0xd7, 0x00);    //sync polarity

	      hobot_write_lt8618sxb(0xff, 0x84);// register bank
	      hobot_write_lt8618sxb(0x06, 0x0c);
	      hobot_write_lt8618sxb(0x07, 0x10);
	// */

	//-------------------------------------------

	hobot_write_lt8618sxb(0x0f, 0x0b + Sample_Freq[_48KHz]);

	// 根据音频数据长度的不同，设置不同的寄存器值。
	hobot_write_lt8618sxb(0x34, 0xd4);	//CTS_N / 2; 32bit
	//    hobot_write_lt8618sxb(0x34, 0xd5);    //CTS_N / 4; 16bit

	hobot_write_lt8618sxb(0x35, (u8) (IIS_N[_48KHz] / 0x10000));
	hobot_write_lt8618sxb(0x36, (u8) ((IIS_N[_48KHz] & 0x00FFFF) / 0x100));
	hobot_write_lt8618sxb(0x37, (u8) (IIS_N[_48KHz] & 0x0000FF));

	hobot_write_lt8618sxb(0x3c, 0x21);	// Null packet enable
}

/***********************************************************

 ***********************************************************/

// LT8618SXB only supports three color space convert: YUV422, yuv444 and rgb888.
// Color space convert of YUV420 is not supported.
void LT8618SXB_CSC_setting(void)
{
	//printk("*****LT8618SXB_CSC_setting*****\n");
	// color space config
	hobot_write_lt8618sxb(0xff, 0x82);	// register bank
	//    hobot_write_lt8618sxb(0xb9, 0x08);// YCbCr444 to RGB
	hobot_write_lt8618sxb(0xb9, 0x18);	// YCbCr422 to RGB

	//    hobot_write_lt8618sxb(0xb9, 0x80);// RGB to YCbCr444
	//    hobot_write_lt8618sxb(0xb9, 0xa0);// RGB to YCbCr422

	//    hobot_write_lt8618sxb(0xb9, 0x10);// YCbCr422 to YCbCr444
	//    hobot_write_lt8618sxb(0xb9, 0x20);// YCbCr444 to YCbCr422

	//    hobot_write_lt8618sxb(0xb9, 0x00); // No csc
}

/***********************************************************

 ***********************************************************/
void LT8618SXB_AVI_setting(void)
{
	//    printk("*****LT8618SXB_AVI_setting*****\n");
	//AVI
	u8 AVI_PB0 = 0x00;
	u8 AVI_PB1 = 0x00;
	u8 AVI_PB2 = 0x00;

	/*************************************************************************
	  The 0x43 register is checksums,
	  changing the value of the 0x45 or 0x47 register,
	  and the value of the 0x43 register is also changed.
	  0x43, 0x44, 0x45, and 0x47 are the sum of the four register values is 0x6F.
	 *************************************************************************/

	//    VIC_Num = 0x04; // 720P 60; Corresponding to the resolution to be output
	//    VIC_Num = 0x10;    // 1080P 60
	//    VIC_Num = 0x1F;    // 1080P 50
	//    VIC_Num = 0x5F;    // 4K30;

	//================================================================//

	// Please refer to function: void LT8618SXB_CSC_setting(void)

	/****************************************************
	  Because the color space of BT1120 is YUV422,
	  if lt8618sxb does not do color space convert (no CSC),
	  the color space of output HDMI is YUV422.
	 *****************************************************/

//	VIC_Num = 0x10;

	// if No csc
	// AVI_PB1 = 0x30;  // PB1,color space: YUV444 0x50;YUV422 0x30; RGB 0x10

	// if YCbCr422 to RGB CSC
	AVI_PB1 = 0x10;		// PB1,color space: YUV444 0x50;YUV422 0x30; RGB 0x10

	// PB2; picture aspect rate: 0x19:4:3 ;     0x2A : 16:9
	AVI_PB2 = Format_Timing[Resolution_Num][Pic_Ratio];

	AVI_PB0 = ((AVI_PB1 + AVI_PB2 + VIC_Num) <= 0x6f) ?
		(0x6f - AVI_PB1 - AVI_PB2 - VIC_Num) :
		(0x16f - AVI_PB1 - AVI_PB2 - VIC_Num);
	// register bank
	hobot_write_lt8618sxb(0xff, 0x84);
	// PB0,avi packet checksum
	hobot_write_lt8618sxb(0x43, AVI_PB0);
	// PB1,color space: YUV444 0x50;YUV422 0x30; RGB 0x10
	hobot_write_lt8618sxb(0x44, AVI_PB1);
	// PB2;picture aspect rate: 0x19:4:3 ; 0x2A : 16:9
	hobot_write_lt8618sxb(0x45, AVI_PB2);
	// PB4;vic ,0x10: 1080P ;  0x04 : 720P
	hobot_write_lt8618sxb(0x47, VIC_Num);

	//    hobot_write_lt8618sxb(0xff,0x84); \
	//    8618SXB hdcp1.4 加密的话，\
	//    要保证hfp + 8410[5:0](rg_island_tr_res) 的个数 \
	//   （video de的下降沿到 最近的一个aude 的间隔），\
	//    大于58；hdcp1.4 spec中有要求
	//    hobot_write_lt8618sxb(0x10, 0x30);             //data iland
	//    hobot_write_lt8618sxb(0x12, 0x64);             //act_h_blank

	//VS_IF, 4k 30hz need send VS_IF packet. Please refer to hdmi1.4 spec 8.2.3
	if (VIC_Num == 95) {
		//       hobot_write_lt8618sxb(0xff,0x84);
		hobot_write_lt8618sxb(0x3d, 0x2a);	//UD1 infoframe enable

		hobot_write_lt8618sxb(0x74, 0x81);
		hobot_write_lt8618sxb(0x75, 0x01);
		hobot_write_lt8618sxb(0x76, 0x05);
		hobot_write_lt8618sxb(0x77, 0x49);
		hobot_write_lt8618sxb(0x78, 0x03);
		hobot_write_lt8618sxb(0x79, 0x0c);
		hobot_write_lt8618sxb(0x7a, 0x00);
		hobot_write_lt8618sxb(0x7b, 0x20);
		hobot_write_lt8618sxb(0x7c, 0x01);
	} else {
		//       hobot_write_lt8618sxb(0xff,0x84);
		hobot_write_lt8618sxb(0x3d, 0x0a);	//UD1 infoframe disable
	}
}

/***********************************************************

 ***********************************************************/
void LT8618SXB_TX_Phy(void)
{
	//    printk("*****LT8618SXB_TX_Phy*****\n");

	// HDMI_TX_Phy
	hobot_write_lt8618sxb(0xff, 0x81);	// register bank
	hobot_write_lt8618sxb(0x30, 0xea);
#if 1				// DC mode
	hobot_write_lt8618sxb(0x31, 0x44);	// HDMDITX_dc_det_en 0x44
	hobot_write_lt8618sxb(0x32, 0x4a);
	hobot_write_lt8618sxb(0x33, 0x0b);	// 0x0b
#else // AC mode
	hobot_write_lt8618sxb(0x31, 0x73);	// HDMDITX_dc_det_en 0x44
	hobot_write_lt8618sxb(0x32, 0xea);
	hobot_write_lt8618sxb(0x33, 0x4a);	// 0x0b
#endif

	hobot_write_lt8618sxb(0x34, 0x00);
	hobot_write_lt8618sxb(0x35, 0x00);
	hobot_write_lt8618sxb(0x36, 0x00);
	hobot_write_lt8618sxb(0x37, 0x44);
	hobot_write_lt8618sxb(0x3f, 0x0f);

	hobot_write_lt8618sxb(0x40, 0xb0);	//0xa0 -- CLK tap0 swing
	hobot_write_lt8618sxb(0x41, 0x68);	//0xa0 -- D0 tap0 swing
	hobot_write_lt8618sxb(0x42, 0x68);	//0xa0 -- D1 tap0 swing
	hobot_write_lt8618sxb(0x43, 0x68);	//0xa0 -- D2 tap0 swing

	hobot_write_lt8618sxb(0x44, 0x0a);
}

/***********************************************************

 ***********************************************************/

u8 LT8618SX_Phase_1(void)
{
	u8 temp = 0;
	u8 read_val = 0;

	u8 OK_CNT = 0x00;
	u8 OK_CNT_1 = 0x00;
	u8 OK_CNT_2 = 0x00;
	u8 OK_CNT_3 = 0x00;
	u8 Jump_CNT = 0x00;
	u8 Jump_Num = 0x00;
	u8 Jump_Num_1 = 0x00;
	u8 Jump_Num_2 = 0x00;
	u8 Jump_Num_3 = 0x00;
	bool temp0_ok = 0;
	bool temp9_ok = 0;
	bool b_OK = 0;

	hobot_write_lt8618sxb(0xff, 0x80);	// register bank
	hobot_write_lt8618sxb(0x13, 0xf1);
	msleep(5);
	hobot_write_lt8618sxb(0x13, 0xf9);	// Reset TTL video process
	msleep(10);
	hobot_write_lt8618sxb(0xff, 0x81);

	for (temp = 0; temp < 0x0a; temp++) {
		hobot_write_lt8618sxb(0x27, (0x60 + temp));
		//#ifdef _DDR_
		if (Use_DDRCLK) {
			hobot_write_lt8618sxb(0x4d, 0x05);
			msleep(5);
			hobot_write_lt8618sxb(0x4d, 0x0d);
			//    msleep(50);
		} else {
			hobot_write_lt8618sxb(0x4d, 0x01);
			msleep(5);
			hobot_write_lt8618sxb(0x4d, 0x09);
		}
		//#endif
		msleep(10);
		read_val = hobot_read_lt8618sxb(0x50) & 0x01;

#ifdef _Phase_Debug_
		printk("\r\ntemp=");
		Debug_DispNum(temp);
		printk("\r\nread_val=");
		Debug_DispNum(read_val);
#endif

		if (read_val == 0) {
			OK_CNT++;

			if (b_OK == 0) {
				b_OK = 1;
				Jump_CNT++;

				if (Jump_CNT == 1) {
					Jump_Num_1 = temp;
				} else if (Jump_CNT == 3) {
					Jump_Num_2 = temp;
				} else if (Jump_CNT == 5) {
					Jump_Num_3 = temp;
				}
			}

			if (Jump_CNT == 1) {
				OK_CNT_1++;
			} else if (Jump_CNT == 3) {
				OK_CNT_2++;
			} else if (Jump_CNT == 5) {
				OK_CNT_3++;
			}

			if (temp == 0) {
				temp0_ok = 1;
			}
			if (temp == 9) {
				Jump_CNT++;
				temp9_ok = 1;
			}
		} else {
			if (b_OK) {
				b_OK = 0;
				Jump_CNT++;
			}
		}
	}

	if ((Jump_CNT == 0) || (Jump_CNT > 6)) {
#ifdef _Phase_Debug_
		printk("\r\ncali phase fail");
#endif
		return 0;
	}

	if ((temp9_ok == 1) && (temp0_ok == 1)) {
		if (Jump_CNT == 6) {
			OK_CNT_3 = OK_CNT_3 + OK_CNT_1;
			OK_CNT_1 = 0;
		} else if (Jump_CNT == 4) {
			OK_CNT_2 = OK_CNT_2 + OK_CNT_1;
			OK_CNT_1 = 0;
		}
	}
	if (Jump_CNT >= 2) {
		if (OK_CNT_1 >= OK_CNT_2) {
			if (OK_CNT_1 >= OK_CNT_3) {
				OK_CNT = OK_CNT_1;
				Jump_Num = Jump_Num_1;
			} else {
				OK_CNT = OK_CNT_3;
				Jump_Num = Jump_Num_3;
			}
		} else {
			if (OK_CNT_2 >= OK_CNT_3) {
				OK_CNT = OK_CNT_2;
				Jump_Num = Jump_Num_2;
			} else {
				OK_CNT = OK_CNT_3;
				Jump_Num = Jump_Num_3;
			}
		}
	}

	hobot_write_lt8618sxb(0xff, 0x81);

	if ((Jump_CNT == 2) || (Jump_CNT == 4) || (Jump_CNT == 6)) {
		hobot_write_lt8618sxb(0x27,
				(0x60 +
				 (Jump_Num + (OK_CNT / 2)) % 0x0a));
	} else if (OK_CNT >= 0x09) {
		hobot_write_lt8618sxb(0x27, 0x65);
	}
#ifdef _Phase_Debug_
	//    printk("\r\nRead LT8618SXB ID ");
	//    printk("\r\n ");
	printk("\r\nReg0x8127 = : 0x%x", hobot_read_lt8618sxb(0x27));
	//    printk(" ", HDMI_ReadI2C_Byte(0x01));
	//    printk(" ", HDMI_ReadI2C_Byte(0x02));
	//    printk("\r\n ");
#endif

	return 1;
}

/***********************************************************

 ***********************************************************/
#ifdef _Embedded_sync_
void LT8618SXB_BT_Timing_setting(void)
{
	//printk("*****LT8618SXB_BT_Timing_setting*****\n");

	// BT1120 内同步信号只有DE, 如果没有外部sync 和DE, \
	// Video check 只能检测到H total 和 H/V active.
	// BT1120 的Timing 值设置就不能通过Video check读取timing状态 \
	// 寄存器的值来设置，需要根据前端BT1120分辨率来设置 \
	// The synchronization signal in BT1120 is only DE.
	// Without external sync and DE, Video check can only \
	// detect H total and H/V active.
	// BT1120 Timing value settings can not be set by reading \
	// the value of the timing status register \
	// through Video check, which needs to be set according to \
	// the front BT1120 resolution.

	// LT8618SX_BT1120_Set
	// Pls refer to array : Format_Timing[][14]
	hobot_write_lt8618sxb(0xff, 0x82);
	hobot_write_lt8618sxb(0x20,
			(u8) (Format_Timing[Resolution_Num][H_act] /
				256));
	hobot_write_lt8618sxb(0x21,
			(u8) (Format_Timing[Resolution_Num][H_act] %
				256));
	hobot_write_lt8618sxb(0x22,
			(u8) (Format_Timing[Resolution_Num][H_fp] / 256));
	hobot_write_lt8618sxb(0x23,
			(u8) (Format_Timing[Resolution_Num][H_fp] % 256));
	hobot_write_lt8618sxb(0x24,
			(u8) (Format_Timing[Resolution_Num][H_sync] /
				256));
	hobot_write_lt8618sxb(0x25,
			(u8) (Format_Timing[Resolution_Num][H_sync] %
				256));
	hobot_write_lt8618sxb(0x26, 0x00);
	hobot_write_lt8618sxb(0x27, 0x00);
	hobot_write_lt8618sxb(0x36,
			(u8) (Format_Timing[Resolution_Num][V_act] /
				256));
	hobot_write_lt8618sxb(0x37,
			(u8) (Format_Timing[Resolution_Num][V_act] %
				256));
	hobot_write_lt8618sxb(0x38,
			(u8) (Format_Timing[Resolution_Num][V_fp] / 256));
	hobot_write_lt8618sxb(0x39,
			(u8) (Format_Timing[Resolution_Num][V_fp] % 256));
	hobot_write_lt8618sxb(0x3a,
			(u8) (Format_Timing[Resolution_Num][V_bp] / 256));
	hobot_write_lt8618sxb(0x3b,
			(u8) (Format_Timing[Resolution_Num][V_bp] % 256));
	hobot_write_lt8618sxb(0x3c,
			(u8) (Format_Timing[Resolution_Num][V_sync] /
				256));
	hobot_write_lt8618sxb(0x3d,
			(u8) (Format_Timing[Resolution_Num][V_sync] %
				256));
}

#if 1
u8 LT8618SX_Phase(void)
{
	//printk("*****LT8618SX_Phase*****\n");
	u8 temp = 0;
	u8 read_value = 0;
	u8 b_ok = 0;
	u8 Temp_f = 0;

	for (temp = 0; temp < 0x0a; temp++) {
		hobot_write_lt8618sxb(0xff, 0x81);
		hobot_write_lt8618sxb(0x27, (0x60 + temp));

		if (Use_DDRCLK) {
			hobot_write_lt8618sxb(0x4d, 0x05);
			msleep(5);
			hobot_write_lt8618sxb(0x4d, 0x0d);
		} else {
			hobot_write_lt8618sxb(0x4d, 0x01);
			msleep(5);
			hobot_write_lt8618sxb(0x4d, 0x09);
		}

		msleep(50);

		read_value = hobot_read_lt8618sxb(0x50);

#ifdef _Phase_Debug_
		printk("\r\ntemp=");
		Debug_DispNum(temp);
		printk("\r\nread_value=");
		Debug_DispNum(read_value);
#endif
		if (read_value == 0x00) {
			if (b_ok == 0) {
				Temp_f = temp;
			}
			b_ok = 1;
		} else {
			b_ok = 0;
		}
	}
#ifdef _Phase_Debug_
	printk("\r\nTemp_f=");
	Debug_DispNum(Temp_f);
#endif
	hobot_write_lt8618sxb(0xff, 0x81);
	hobot_write_lt8618sxb(0x27, (0x60 + Temp_f));

	return Temp_f;
}

/***********************************************************

 ***********************************************************/
bool LT8618SXB_Phase_config(void)
{
	//printk("*****LT8618SXB_Phase_config*****\n");
	u8 Temp = 0x00;
	u8 Temp_f = 0x00;
	u8 OK_CNT = 0x00;
	u8 OK_CNT_1 = 0x00;
	u8 OK_CNT_2 = 0x00;
	u8 OK_CNT_3 = 0x00;
	u8 Jump_CNT = 0x00;
	u8 Jump_Num = 0x00;
	u8 Jump_Num_1 = 0x00;
	u8 Jump_Num_2 = 0x00;
	u8 Jump_Num_3 = 0x00;
	bool temp0_ok = 0;
	bool temp9_ok = 0;
	bool b_OK = 0;
	u16 V_ACT = 0x0000;
	u16 H_ACT = 0x0000;
	u16 H_TOTAL = 0x0000;
	//    u16        V_TOTAL       = 0x0000;
	//    u8        H_double   = 1;

	Temp_f = LT8618SX_Phase();	//it's setted before video check

	while (Temp <= 0x09) {
		hobot_write_lt8618sxb(0xff, 0x81);
		hobot_write_lt8618sxb(0x27, (0x60 + Temp));
		hobot_write_lt8618sxb(0xff, 0x80);
		hobot_write_lt8618sxb(0x13, 0xf1);	//ttl video process reset///20191121
		hobot_write_lt8618sxb(0x12, 0xfb);	//video check reset//20191121
		msleep(5);	//add 20191121
		hobot_write_lt8618sxb(0x12, 0xff);	//20191121
		hobot_write_lt8618sxb(0x13, 0xf9);	//20191121

		msleep(80);	//

		hobot_write_lt8618sxb(0xff, 0x81);
		hobot_write_lt8618sxb(0x51, 0x42);

		hobot_write_lt8618sxb(0xff, 0x82);
		H_TOTAL = hobot_read_lt8618sxb(0x8f);
		H_TOTAL = (H_TOTAL << 8) + hobot_read_lt8618sxb(0x90);
		V_ACT = hobot_read_lt8618sxb(0x8b);
		V_ACT = (V_ACT << 8) + hobot_read_lt8618sxb(0x8c);
		H_ACT = hobot_read_lt8618sxb(0x8d);
		H_ACT = (H_ACT << 8) + hobot_read_lt8618sxb(0x8e) - 0x04;	//note

		//                    hobot_write_lt8618sxb(0xff, 0x80);
		//                    hobot_write_lt8618sxb(0x09, 0xfe);

#ifdef _Phase_Debug_
		printk("\r\n h_total=");
		Debug_DispNum(H_TOTAL);
		printk("\r\n v_act=");
		Debug_DispNum(V_ACT);
		printk("\r\n h_act=");
		Debug_DispNum(H_ACT);
#endif
		if ((V_ACT > (Format_Timing[Resolution_Num][V_act] - 5))
				&& (V_ACT < (Format_Timing[Resolution_Num][V_act] + 5))
				&& (H_ACT > (Format_Timing[Resolution_Num][H_act] - 5))
				&& (H_ACT < (Format_Timing[Resolution_Num][H_act] + 5))
				&& (H_TOTAL > (Format_Timing[Resolution_Num][H_tol] - 5))
				&& (H_TOTAL < (Format_Timing[Resolution_Num][H_tol] + 5))) {
			OK_CNT++;

			if (b_OK == 0) {
				b_OK = 1;
				Jump_CNT++;

				if (Jump_CNT == 1) {
					Jump_Num_1 = Temp;
				} else if (Jump_CNT == 3) {
					Jump_Num_2 = Temp;
				} else if (Jump_CNT == 5) {
					Jump_Num_3 = Temp;
				}
			}

			if (Jump_CNT == 1) {
				OK_CNT_1++;
			} else if (Jump_CNT == 3) {
				OK_CNT_2++;
			} else if (Jump_CNT == 5) {
				OK_CNT_3++;
			}

			if (Temp == 0) {
				temp0_ok = 1;
			}
			if (Temp == 9) {
				Jump_CNT++;
				temp9_ok = 1;
			}
#ifdef _Phase_Debug_
			printk("\r\n this phase is ok,temp=");
			Debug_DispNum(Temp);
			printk("\r\n Jump_CNT=");
			Debug_DispNum(Jump_CNT);
#endif
		} else {
			if (b_OK) {
				b_OK = 0;
				Jump_CNT++;
			}
#ifdef _Phase_Debug_
			printk("\r\n this phase is fail,temp=");
			Debug_DispNum(Temp);
			printk("\r\n Jump_CNT=");
			Debug_DispNum(Jump_CNT);
#endif
		}

		Temp++;
	}

#ifdef _Phase_Debug_
	printk("\r\n OK_CNT_1=");
	Debug_DispNum(OK_CNT_1);
	printk("\r\n OK_CNT_2=");
	Debug_DispNum(OK_CNT_2);
	printk("\r\n OK_CNT_3=");
	Debug_DispNum(OK_CNT_3);
#endif

	if ((Jump_CNT == 0) || (Jump_CNT > 6)) {
#ifdef _Phase_Debug_
		printk("\r\ncali phase fail");
#endif
		return 0;
	}

	if ((temp9_ok == 1) && (temp0_ok == 1)) {
		if (Jump_CNT == 6) {
			OK_CNT_3 = OK_CNT_3 + OK_CNT_1;
			OK_CNT_1 = 0;
		} else if (Jump_CNT == 4) {
			OK_CNT_2 = OK_CNT_2 + OK_CNT_1;
			OK_CNT_1 = 0;
		}
	}

	if (Jump_CNT >= 2) {
		if (OK_CNT_1 >= OK_CNT_2) {
			if (OK_CNT_1 >= OK_CNT_3) {
				OK_CNT = OK_CNT_1;
				Jump_Num = Jump_Num_1;
			} else {
				OK_CNT = OK_CNT_3;
				Jump_Num = Jump_Num_3;
			}
		} else {
			if (OK_CNT_2 >= OK_CNT_3) {
				OK_CNT = OK_CNT_2;
				Jump_Num = Jump_Num_2;
			} else {
				OK_CNT = OK_CNT_3;
				Jump_Num = Jump_Num_3;
			}
		}
	}
	hobot_write_lt8618sxb(0xff, 0x81);

	if ((Jump_CNT == 2) || (Jump_CNT == 4) || (Jump_CNT == 6)) {
		hobot_write_lt8618sxb(0x27,
				(0x60 +
				 (Jump_Num + (OK_CNT / 2)) % 0x0a));
	}

	if (OK_CNT == 0x0a) {
		hobot_write_lt8618sxb(0x27, (0x60 + (Temp_f + 5) % 0x0a));
	}
#ifdef _Phase_Debug_
	printk("cail phase is 0x%x", hobot_read_lt8618sxb(0x27));
#endif

	return 1;
}
#endif
#endif

int hdmi_get_edid(void *param){
	hobot_hdmi_sync_t sync;
	int ret = LT8618SXB_Read_EDID(&sync);
	if(!ret){
		hobot_hdmi_sync_t *fb_sync = (hobot_hdmi_sync_t *)param;
		if(fb_sync != NULL){
			fb_sync->hfp = sync.hfp;
			fb_sync->hs = sync.hs;
			fb_sync->hbp = sync.hbp;
			fb_sync->hact = sync.hact;
			fb_sync->htotal = sync.htotal;
			fb_sync->vfp = sync.vfp;
			fb_sync->vs = sync.vs;
			fb_sync->vbp = sync.vbp;
			fb_sync->vact = sync.vact;
			fb_sync->vtotal = sync.vtotal;
			fb_sync->clk = sync.clk;
		}
	}
	return ret;
}
EXPORT_SYMBOL(hdmi_get_edid);
void hdmi_set_resolution(hobot_hdmi_sync_t* user_timing){
	Resolution_change(user_timing);
}
EXPORT_SYMBOL(hdmi_set_resolution);
/***********************************************************

 ***********************************************************/
void LT8618SX_Initial(void)
{
	//hdmi_register_get_edid_callback(hdmi_get_edid);
	//hdmi_register_set_resolution_callback(hdmi_set_resolution);
	//printk("*****LT8618SX_Initial*****\n");
	Use_DDRCLK = 0;		// 1: DDR mode; 0: SDR (normal) mode
	hobot_hdmi_sync_t temp_timing;
	// Parameters required by LT8618SXB_BT_Timing_setting(void)
	Resolution_Num = _1080P60_;
	CLK_bound =
		Format_Timing[Resolution_Num][Clk_bound_SDR + (u8) (Use_DDRCLK)];
	VIC_Num = Format_Timing[Resolution_Num][Vic];
	//    VIC_Num = 0x10; // 1080P 60
	//    VIC_Num = 0x1F; // 1080P 50
	//    VIC_Num = 0x5F; // 4K30;

	//  With different resolutions, Vic has different values,
	//  Refer to the following list

	/*************************************
	  Resolution            VIC_Num (VIC: Video Format Identification Code)
	  --------------------------------------
	  640x480                1
	  720x480P 60Hz        2
	  720x480i 60Hz        6

	  720x576P 50Hz        17
	  720x576i 50Hz        21

	  1280x720P 24Hz        60
	  1280x720P 25Hz        61
	  1280x720P 30Hz        62
	  1280x720P 50Hz        19
	  1280x720P 60Hz        4

	  1920x1080P 24Hz        32
	  1920x1080P 25Hz        33
	  1920x1080P 30Hz        34

	  1920x1080i 50Hz        20
	  1920x1080i 60Hz        5

	  1920x1080P 50Hz        31
	  1920x1080P 60Hz        16

	  3840x2160 30Hz        95 // 4K30

	  Other resolution     0(default) // Such as 800x600 / 1024x768 / 1366x768 / 1280x1024.....
	 **************************************/
	//--------------------------------------------------------//
	I2CADR = _LT8618SX_ADR;	// 设置IIC地址
	LT8618SXB_Chip_ID();	// for debug
#ifndef BYPASS_INIT
	LT8618SXB_Reset();

	//********************************************************//
	// Before initializing lt8168sxb, you need to enable IIC of lt8618sxb
	hobot_write_lt8618sxb(0xff, 0x80);	// register bank
	hobot_write_lt8618sxb(0xee, 0x01);	// enable IIC
	//********************************************************//


	LT8618SXB_RST_PD_Init();

	// TTL mode
	LT8618SXB_TTL_Input_Analog();
	LT8618SXB_TTL_Input_Digtal();

	// Wait for the signal to be stable
	// and decide whether the delay is necessary according to the actual situation
	msleep(1000);		// 等待信号稳定,根据实际情况决定是否需要延时
	LT8618SXB_Video_check();	// For debug

	//------------------------PLL----------------------------------//
	LT8618SXB_PLL_setting();

	//-------------------------------------------
	LT8618SXB_Audio_setting();

	//-------------------------------------------
	LT8618SXB_CSC_setting();

	//-------------------------------------------
#ifdef _LT8618_HDCP_
	LT8618SXB_HDCP_Init();
#endif

	//-------------------------------------------
	LT8618SXB_AVI_setting();

	// This operation is not necessary. Read TV EDID if necessary.


	//-------------------------------------------
#ifdef _LT8618_HDCP_
	LT8618SXB_HDCP_Enable();
#endif

#ifdef _Embedded_sync_
	//-------------------------------------------
	LT8618SXB_BT_Timing_setting();

	if (flag_Ver_u3) {
		LT8618SX_Phase_1();
	} else {
		LT8618SXB_Phase_config();
	}
#else
	LT8618SX_Phase_1();
#endif

	//-------------------------------------------

	//    LT8618SXB_RST_PD_Init();

	//-------------------------------------------

	// HDMI_TX_Phy
	LT8618SXB_TX_Phy();
#endif
	LT8618SXB_Read_EDID(&temp_timing);	// Read TV  EDID
}



// When the lt8618sxb works, the resolution of the bt1120 signal changes.
// The following settings need to be configured.
void Resolution_change(hobot_hdmi_sync_t* user_timing)
{
	//printk("Change\n");
	int ret = 0;
	int aspect_ratio;
	int pixel_clk_mhz = 0;
	int i = 0;
	hobot_hdmi_sync_t edid_timing;

	if(user_timing != NULL && user_timing->auto_detect == 0){
		pr_info("none detect\n");
		memcpy(&edid_timing, user_timing, sizeof(hobot_hdmi_sync_t));
	}else{
		pr_info("auto detect\n");
		ret = LT8618SXB_Read_EDID(&edid_timing);
		if(ret){
			pr_err("Get EDID fail\n");
			goto err0;
		}	
	}
	pr_info("%s hfp:%d,hs:%d,hbp:%d,hact:%d,htotal:%d,vfp:%d,vs:%d,vbp:%d,vact:%d,vtotal:%d,vic:%d,clk:%d\n",
	__func__,
	edid_timing.hfp,
	edid_timing.hs,
	edid_timing.hbp,
	edid_timing.hact,
	edid_timing.htotal,
	edid_timing.vfp,
	edid_timing.vs,
	edid_timing.vbp,
	edid_timing.vact,
	edid_timing.vtotal,
	edid_timing.vic,
	edid_timing.clk
	);

	Format_Timing[18][0] =  edid_timing.hfp;
	Format_Timing[18][1] =  edid_timing.hs;
	Format_Timing[18][2] =  edid_timing.hbp;
	Format_Timing[18][3] =  edid_timing.hact;
	Format_Timing[18][4] =  edid_timing.htotal;
	Format_Timing[18][5] =  edid_timing.vfp;
	Format_Timing[18][6] =  edid_timing.vs;
	Format_Timing[18][7] =  edid_timing.vbp;
	Format_Timing[18][8] =  edid_timing.vact;
	Format_Timing[18][9] =  edid_timing.vtotal;
	Format_Timing[18][10] =  edid_timing.vic;
	aspect_ratio = (edid_timing.hact * 1000) / (edid_timing.vact) ;
	pr_debug("asp ratio:%d\n",aspect_ratio);
	if (aspect_ratio > 1750 && aspect_ratio < 1780) {
        pr_debug("ratio: 16:9\n");
		Format_Timing[18][11] =  _16_9_;
    } else if (aspect_ratio > 1320 && aspect_ratio < 1340 ) {
        pr_debug("ratio: 4:3\n");	
		Format_Timing[18][11] =  _4_3_;
    } else {
        pr_info("Unsupport ratio,using 4:3\n");
		Format_Timing[18][11] =  _4_3_;
    }
	pixel_clk_mhz = edid_timing.clk / 1000000;
	pr_err("pixel clk:%d Mhz\n",pixel_clk_mhz);
	if (pixel_clk_mhz > 100)		
	{
		Format_Timing[18][12] = _Greater_than_100M;
		Format_Timing[18][13] = pixel_clk_mhz / 2 >= 100?_Greater_than_100M:_Bound_50_100M;
		pr_debug("Format_Timing[18][12]:%d,Format_Timing[18][13]:%d\n",Format_Timing[18][12],Format_Timing[18][13]);
	}
	else if(pixel_clk_mhz <=100 && pixel_clk_mhz >= 50)
	{
		Format_Timing[18][12] = _Bound_50_100M;
		Format_Timing[18][13] = pixel_clk_mhz / 2 >= 50?_Bound_50_100M:_Less_than_50M;
		pr_debug("Format_Timing[18][12]:%d,Format_Timing[18][13]:%d\n",Format_Timing[18][12],Format_Timing[18][13]);

	}
	else if(pixel_clk_mhz < 50)
	{
		Format_Timing[18][12] = _Less_than_50M;
		Format_Timing[18][13] = _Less_than_50M;
	}
	for(i = 0;i<14;i++){
		pr_debug("timing[18][%d]:%d\n",i,Format_Timing[18][i]);
	}

	Resolution_Num = 18;

	CLK_bound =
		Format_Timing[Resolution_Num][Clk_bound_SDR + (u8) (Use_DDRCLK)];

	VIC_Num = Format_Timing[Resolution_Num][Vic];

#ifndef BYPASS_INIT
	I2CADR = _LT8618SX_ADR;	// 设置IIC地址
	LT8618SXB_Reset();

	//********************************************************//
	// Before initializing lt8168sxb, you need to enable IIC of lt8618sxb
	hobot_write_lt8618sxb(0xff, 0x80);	// register bank
	hobot_write_lt8618sxb(0xee, 0x01);	// enable IIC
	//********************************************************//
#endif
	LT8618SXB_RST_PD_Init();

	// TTL mode
	LT8618SXB_TTL_Input_Analog();
	LT8618SXB_TTL_Input_Digtal();
#ifndef BYPASS_INIT
	// Wait for the signal to be stable
	// and decide whether the delay is necessary according to the actual situation
	msleep(1000);		// 等待信号稳定,根据实际情况决定是否需要延时
	LT8618SXB_Video_check();	// For debug
#endif
	//------------------------PLL----------------------------------//
	LT8618SXB_PLL_setting();
#ifndef BYPASS_INIT
	//-------------------------------------------
	LT8618SXB_Audio_setting();

	//-------------------------------------------


	//-------------------------------------------
#ifdef _LT8618_HDCP_
	LT8618SXB_HDCP_Init();
#endif

#endif
	LT8618SXB_CSC_setting();
	//-------------------------------------------
	LT8618SXB_AVI_setting();

	// This operation is not necessary. Read TV EDID if necessary.
	//LT8618SXB_Read_EDID(NULL);	// Read TV  EDID
#ifndef BYPASS_INIT
	//-------------------------------------------
#ifdef _LT8618_HDCP_
	LT8618SXB_HDCP_Enable();
#endif
#endif
#ifdef _Embedded_sync_
	//-------------------------------------------
	LT8618SXB_BT_Timing_setting();

	if (flag_Ver_u3) {
		LT8618SX_Phase_1();
	} else {
		LT8618SXB_Phase_config();
	}
#else
	LT8618SX_Phase_1();
#endif

	//-------------------------------------------

	//    LT8618SXB_RST_PD_Init();

	//-------------------------------------------
	LT8618SXB_TX_Phy();
	return;

err0:
	pr_err("%s fail\n",__func__);

}

/************************************** The End Of File **************************************/
