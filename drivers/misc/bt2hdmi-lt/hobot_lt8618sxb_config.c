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


/*--------------------------------
   CI2CA            I2C address
   -----------------------
   (0~400mV)----------0x72(default)
   (400mV~800mV)------0x7a
   (800mV~1200mV)-----0x90
   (1200mV~1600mV)	----0x92

   (2000mV~2400mV)----0x94
   (2400mV~2800mV)----0x7e
   (2800mV~3300mV)----0x76
   -----------------------------------*/

/***************************************************


   要注意的是IIC地址0x72(0x76)的最低位bit0是读写标志位，如果是Linux系统的IIC，最高位是读写标志位，IIC地址需要右移一位，IIC地址变成0x39(0x3B).

   IIC 速率不要超过100K

   如果LT8618SXB 的IIC 地址不是0x72;需要用主控的GPIO 去复位LT8618SXB，先拉低100ms，再拉高，delay100ms，再初始化LT8618SXB寄存器。

   The lowest bit 0 of the IIC address 0x72 (0x76) of LT8618SXB is the read-write flag bit.
   In the case of Linux IIC, the highest bit is the read-write flag bit, The IIC address needs to be moved one bit to the right, and the IIC address becomes 0x39 (0x3B).

   IIC rate should not exceed 100K.

   If the IIC address of LT8618SXB is not 0x72, you need to reset LT8618SXB with the master GPIO, pull down 100ms, then pull up, delay 100ms, and initialize the LT8618SXB register.

 ****************************************************/

static int dvi_mode = 1;

// 1、Reset LT8618SX
// 2、LT8618SX Initial setting:


#define _D0_D15_In 0x30 // BT1120 input from D0 to D15 of LT8618SXB pins. // D0 ~ D7 Y ; D8 ~ D15 C
#define _D8_D23_In 0x70 // BT1120 input from D8 to D23 of LT8618SXB pins. // D8 ~ D15 Y ; D16 ~ D23 C

#define _D0_D15_In_2 0x00 // BT1120 input from D0 to D15 of LT8618SXB pins. // D0 ~ D7 C ; D8 ~ D15 Y
#define _D8_D23_In_2 0x60 // BT1120 input from D8 to D23 of LT8618SXB pins. // D8 ~ D15 C ; D16 ~ D23 Y

// #define _LT8618_HDCP_ // 如果需要HDMI 加密输出，请打开这个宏定义

static uint8_t Use_DDRCLK; // 1: DDR mode; 0: SDR (normal) mode

static uint8_t HDMI_VIC; // vic ,0x10: 1080P ;  0x04 : 720P ; Refer to the following list

static uint8_t HDMI_CS; //color spece ,  RGB:0x10    ycbcr422:0x30   ycbcr44:0x50  //2020.04.22

static uint8_t HDMI_PA; //picture aspect,16:9=0x2a   4:3=0x19                      //2020.04.22

#ifdef _Read_TV_EDID_

uint8_t Sink_EDID[256];

#endif

/*************************************
   Resolution			HDMI_VIC
   --------------------------------------
   640x480				1
   720x480P 60Hz		2
   720x480i 60Hz		6

   720x576P 50Hz		17
   720x576i 50Hz		21

   1280x720P 24Hz		60
   1280x720P 25Hz		61
   1280x720P 30Hz		62
   1280x720P 50Hz		19
   1280x720P 60Hz		4

   1920x1080P 24Hz		32
   1920x1080P 25Hz		33
   1920x1080P 30Hz		34

   1920x1080i 50Hz		20
   1920x1080i 60Hz		5

   1920x1080P 50Hz		31
   1920x1080P 60Hz		16

   Other resolution	0(default)

 **************************************/
enum {
    _32KHz = 0,
    _44d1KHz,
    _48KHz,

    _88d2KHz,
    _96KHz,
    _176Khz,
    _196KHz
};

static uint16_t IIS_N[] = {
    4096, // 32K
    6272, // 44.1K
    6144, // 48K
    12544, // 88.2K
    12288, // 96K
    25088, // 176K
    24576 // 196K
};

static uint16_t Sample_Freq[] = {
    0x30, // 32K
    0x00, // 44.1K
    0x20, // 48K
    0x80, // 88.2K
    0xa0, // 96K
    0xc0, // 176K
    0xe0 //  196K
};

//************************************/

static uint8_t Resolution_Num;

//BT1120 内同步信号只有DE, 如果没有外部sync 和DE,Video check 只能检测到H/V total 和 H/V active.

static uint8_t LT8618SXB_BT1120_SDR_PLL_setting[12][3] = {
    //{ 0x00, 0xa8, 0xbb },   // < 50MHz
    { 0x00, 0xa8, 0x99 }, // < 50MHz
    { 0x00, 0x94, 0xaa }, // 50 ~ 59M
    { 0x01, 0xa8, 0xaa }, // 60 ~ 89M
    { 0x02, 0xbc, 0xaa }, // 90 ~ 99M

    { 0x02, 0x9e, 0x99 }, // 100 ~ 119M
    { 0x03, 0xa8, 0x99 }, // 120 - 149M
    { 0x04, 0xb2, 0x99 }, // 150 - 179M
    { 0x05, 0xbc, 0x99 }, // 180 - 199M

    { 0x05, 0x9e, 0x88 }, // 200 - 209M
    { 0x06, 0xa3, 0x88 }, // 210 - 239M
    { 0x07, 0xa8, 0x88 }, // 240 - 269M
    { 0x08, 0xad, 0x88 }, // >= 270 M
};

static uint8_t LT8618SXB_BT1120_DDR_PLL_setting[6][3] = {
    // 根据实际D CLK的大小选择，比如
    // BT1120 720P 60Hz DDR, D_CLK is 37MHz, select _Less_than_50M
    // BT1120 1080P 60Hz DDR, D_CLK is 74.25MHz, select _Bound_60_90M.
    // BT1120 4K30Hz DDR, D_CLK is 148.5MHz, select _Bound_120_150M

    //	{0x00,0xd0,0xbb},// < 25MHz
    { 0x00, 0xa8, 0xaa }, // 25 ~ 50M
    { 0x00, 0x94, 0x99 }, // 50 ~ 59M
    { 0x01, 0xa8, 0x99 }, // 60 ~ 89M

    { 0x02, 0xbc, 0x99 }, // 90 ~ 99M
    { 0x02, 0x9e, 0x88 }, // 100 ~ 119M
    { 0x03, 0xa8, 0x88 }, // 120 - 149M
    //	{0x04,0xb2,0x88},// 150 - 179M

    //	{0x05,0xbc,0x88},// 180 - 209M
    //	{0x06,0xc6,0x88},// 210 - 239M
    //	{0x07,0xd0,0x88},// 240 - 269M
    //	{0x08,0xda,0x88},// >= 270 M
};

enum {
    _Less_than_50M = 0x00, // SDR:480P/ 576P -- 27M ; DDR: 480P/ 576P -- 13.5M; DDR: 720P 60/50Hz -- 37.125M
    _Bound_50_60M,
    _Bound_60_90M, // SDR:720P 60/50Hz; DDR:1080P 60/50Hz. -- 74.25M
    _Bound_90_100M,

    _Bound_100_120M,
    _Bound_120_150M, // SDR:1080P 60/50Hz; DDR:4K30Hz. -- 148.5M
    _Bound_150_180M,
    _Bound_180_200M,

    _Bound_200_210M,
    _Bound_210_240M,
    _Bound_240_270M,
    _Greater_than_270M // SDR:4K30Hz
};

static uint8_t CLK_Num;

//==============================================================//

static int Resolution_Timing[][11] = {
    // hfp,	hs,		hbp,		hact,		htotal,	vfp,	vs,	vbp,	vact,		vtotal,	pixel_CLK/10000
    //-------------------------------------------------------------------------------------//
    // 0	- 720P 60Hz 74.25MHz
    { 110, 40, 220, 1280, 1650, 5, 5, 20, 720, 750, 7425 }, //	720P 60Hz 74.25MHz

    // 1 - 1024x768 60Hz	65MHz
    { 24, 136, 160, 1024, 1344, 3, 6, 29, 768, 806, 6500 }, //	1024x768 60Hz	65MHz

    // 2 - 1280x1024 60Hz	108MHz
    { 48, 112, 248, 1280, 1688, 1, 3, 38, 1024, 1066, 10800 }, //	1280x1024 60Hz	108MHz

    // 3 - 1600x1200 60Hz	162MHz
    { 64, 192, 304, 1600, 2160, 1, 3, 46, 1200, 1250, 16200 }, //	1600x1200 60Hz	162MHz

    // 4 - 1080P30  30Hz	74.25MHz
    { 88, 44, 148, 1920, 2200, 4, 5, 36, 1080, 1125, 7425 }, //	1080P30	 30Hz	74.25MHz

    // 5 - 1920x1200 60Hz
    { 48, 32, 80, 1920, 2080, 3, 6, 26, 1200, 1235, 15400 }, //	1920x1200 CVT-RB  	154MHz
    //	{ 136,	200,		336,		1920,	2592,	3,	6,	36, 	1200,	1245,	19325},	//	1920x1200 CVT  	193.25MHz

    // 6 - 1080P60  60Hz	148.5MHz
    { 88, 44, 148, 1920, 2156, 4, 5, 36, 1080, 1125, 14850 }, //	1080P60	 60Hz	148.5MHz

    // 7 - 2560x1600 60Hz
    { 48, 32, 80, 2560, 2720, 3, 6, 37, 1600, 1646, 26850 }, //	2560x1600  CVT-RB	268.5MHz
    //	{ 192,	280,		472,		2560,	3504,	3,	6,	49,	1600,	1658,	34850},	//	2560x1600 CVT	348.5MHz

    // 8 - 3840x2160 30Hz   297MHz
    { 176, 88, 296, 3840, 4400, 8, 10, 72, 2160, 2250, 29700 },

    // 9 - 3840x2160 60Hz 	 XXX 	--	40K60 resolution is not supported
    //	{ },

    // 10 - 720x576 50Hz	27MHz
    { 12, 64, 68, 720, 864, 5, 5, 39, 576, 625, 2700 }, // 576P50	 50Hz	27MHz

    // 11 - 720x480 60Hz	27MHz
    { 16, 62, 60, 720, 858, 9, 6, 30, 480, 525, 2700 }, // 480P60  60Hz	27MHz

    // 12 - 1440x900 60Hz
    { 48, 32, 80, 1440, 1600, 3, 6, 17, 900, 926, 8875 }, // 1440x900  CVT-RB	88.75MHz
    //	{ 80,	152,		232,		1440,	1904,	3,	6,	25,	900,		934,		10650},	// 1440x900  CVT		106.5MHz
};

static uint8_t VIC_Num[] = {
    0x04, // 720P60
    0x00, // 1024x768 60Hz
    0x00, // 1280x1024 60Hz
    0x00, // 1600x1200 60Hz

    0x22, // 1080P30
    0x00, // 1920x1200 60Hz
    0x10, // 1080P60
    0x00, // 2560x1600 60Hz

    0x00, // 3840x2160 30Hz
    //	0x00, // XXX    3840x2160 60Hz
    0x0e, // 576P50
    0x02, // 480P60
    0x00 // 1440x900 60Hz
};

static uint8_t SDR_PLL_Setting[] = {
    _Bound_60_90M, // 720P60  74.25M
    _Bound_60_90M, // 1024x768 60Hz  65M
    _Bound_100_120M, // 1280x1024 60Hz 108M
    _Bound_150_180M, // 1600x1200 60Hz  162M

    _Bound_60_90M, // 1080P30  74.25M

    _Bound_150_180M, // 1920x1200 60Hz  CVT-RB 154M
    //	_Bound_180_200M, // 1920x1200 60Hz  CVT 193M

    _Bound_120_150M, // 1080P60	148.5M

    _Bound_240_270M, // 2560x1600 CVT_RB  268.5M
    //	_Greater_than_270M, // 2560x1600 CVT  348.5M

    _Greater_than_270M, // 3840x2160 30Hz	  297M
    //	0x00, // 3840x2160 60Hz

    _Less_than_50M, // 576P50	27M
    _Less_than_50M, // 480P60	27M

    _Bound_60_90M // 1440x900 CVT_RB  88.75M
    //	_Bound_100_120M // 1440x900 CVT		106.5M

};

static uint8_t DDR_PLL_Setting[] = {
    _Less_than_50M, // 720P60  74.25M
    _Less_than_50M, // 1024x768 60Hz  65M
    _Bound_50_60M, // 1280x1024 60Hz 108M
    _Bound_60_90M, // 1600x1200 60Hz	162M

    _Less_than_50M, // 1080P30  74.25M

    _Bound_60_90M, // 1920x1200 60Hz	CVT-RB 154M
    //	_Bound_90_100M, // 1920x1200 60Hz	CVT 193M

    _Bound_60_90M, // 1080P60 148.5M

    _Bound_120_150M, // 2560x1600 CVT_RB  268.5M
    //	_Bound_150_180M, // 2560x1600 CVT  348.5M

    _Bound_150_180M, // 3840x2160 30Hz	  297M
    //	0x00, // 3840x2160 60Hz

    _Less_than_50M, // 576P50	27M
    _Less_than_50M, // 480P60	27M

    _Less_than_50M // 1440x900 CVT_RB  88.75M
    //	_Bound_50_60M // 1440x900 CVT 	106.5M

};

enum {
    H_FrontPorch = 0,
    H_SyncWid,
    H_BackPorch,

    H_Active,
    H_Total,

    V_FrontPorch,
    V_SyncWid,
    V_BackPorch,

    V_Active,
    V_Total,

    Pixel_CLK
};

enum {
    _720P60_ = 0,
    _1024x768P60_,
    _1280x1024P60_,
    _1600x1200P60_,

    _1080P30_,
    _1920x1200P60_,
    _1080P60_,
    _2560x1600P60_,

    _3840x2160P30_,
    //	_3840x2160P60_,	// XXX   Chip does not support
    _720x576P50_,
    _720x480P60_,
    _1440x900P60_
};


#define LT8618SXB_SLAVE_ADDR        (0x3B)
extern struct x2_lt8618sxb_s* g_x2_lt8618sxb;
extern int lt8618sxb_reset_pin;

static int hobot_lt8618sxb_write_byte(struct i2c_client* client, uint32_t addr,
    uint8_t reg, uint8_t val)
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

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret >= 0)
        return 0;

    pr_err("lt8618sxb i2c write error addr:0%x reg:0x%x ret %d !\n",
        addr, reg, ret);

    return ret;
}

static int hobot_write_lt8618sxb(uint8_t reg_addr, uint8_t reg_val)
{
    return hobot_lt8618sxb_write_byte(g_x2_lt8618sxb->client, LT8618SXB_SLAVE_ADDR, reg_addr, reg_val);
}

static int hobot_lt8618sxb_read_byte(struct i2c_client* client, uint32_t addr,
    uint8_t reg, uint8_t* val)
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

    ret = hobot_lt8618sxb_read_byte(g_x2_lt8618sxb->client, LT8618SXB_SLAVE_ADDR, reg_addr, &reg_val);
    if (ret) {
        pr_err("hobot read lt8618sxb error!\n");
    }

    return reg_val;
}


int LT8618SX_Chip_ID(void)
{
    int ret = 0;

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

    printk("************* Read LT8618SXB ID ***************\n");
    printk("LT8618SXB Chip ID = 0x%x, 0x%x, 0x%x\n", hobot_read_lt8618sxb(0x00),
        hobot_read_lt8618sxb(0x01), hobot_read_lt8618sxb(0x02));

    return 0;
}


#ifdef _LT8618_HDCP_
/***********************************************************/

static void LT8618SX_HDCP_Init(void) //luodexing
{
    hobot_write_lt8618sxb(0xff, 0x85);
    hobot_write_lt8618sxb(0x07, 0x1f);
    hobot_write_lt8618sxb(0x13, 0xfe); // [7]=force_hpd, [6]=force_rsen, [5]=vsync_pol, [4]=hsync_pol,
        // [3]=hdmi_mode, [2]=no_accs_when_rdy, [1]=skip_wt_hdmi
    hobot_write_lt8618sxb(0x17, 0x0f); // [7]=ri_short_read, [3]=sync_pol_mode, [2]=srm_chk_done,
        // [1]=bksv_srm_pass, [0]=ksv_list_vld
    hobot_write_lt8618sxb(0x15, 0x05);
    //hobot_write_lt8618sxb(0x15,0x65);// [7]=key_ddc_st_sel, [6]=tx_hdcp_en,[5]=tx_auth_en, [4]=tx_re_auth
}

/***********************************************************/

static void LT8618SX_HDCP_Enable(void) //luodexing
{
    hobot_write_lt8618sxb(0xff, 0x80);
    hobot_write_lt8618sxb(0x14, 0x7f);
    hobot_write_lt8618sxb(0x14, 0xff);
    hobot_write_lt8618sxb(0xff, 0x85);
    hobot_write_lt8618sxb(0x15, 0x01); //disable HDCP
    hobot_write_lt8618sxb(0x15, 0x71); //enable HDCP
    hobot_write_lt8618sxb(0x15, 0x65); //enable HDCP
}

/***********************************************************/

static void LT8618SX_HDCP_Disable(void) //luodexing
{
    hobot_write_lt8618sxb(0xff, 0x85);
    hobot_write_lt8618sxb(0x15, 0x45); //enable HDCP
}

#endif

/***********************************************************

***********************************************************/

static void LT8918SX_Read_EDID(void)
{
#ifdef _Read_TV_EDID_

    uint8_t i, j;

    hobot_write_lt8618sxb(0xff, 0x85);
    hobot_write_lt8618sxb(0x02, 0x0a); //I2C 100K
    hobot_write_lt8618sxb(0x03, 0xc9);
    hobot_write_lt8618sxb(0x04, 0xa0); //0xA0 is EDID device address
    hobot_write_lt8618sxb(0x05, 0x00); //0x00 is EDID offset address
    hobot_write_lt8618sxb(0x06, 0x20); //length for read
    hobot_write_lt8618sxb(0x14, 0x7f);
    for (i = 0; i < 8; i++) {
        hobot_write_lt8618sxb(0x05, i * 32); //0x00 is EDID offset address
        hobot_write_lt8618sxb(0x07, 0x36);
        hobot_write_lt8618sxb(0x07, 0x31);
        hobot_write_lt8618sxb(0x07, 0x37);
        msleep(20);

        if (hobot_read_lt8618sxb(0x40) & 0x02) //KEY_DDC_ACCS_DONE=1
        {
            if (hobot_read_lt8618sxb(0x40) & 0x50) //DDC No Ack or Abitration lost
            {
                printk("LT8918 Read EDID Failed ...\n");
                return;
            } else {
		int k;
                for (j = 0; j < 32; j++) {
                    Sink_EDID[i * 32 + j] = hobot_read_lt8618sxb(0x83);
			for (k=0; k<32; k++)
                    printk("0x%02X ", Sink_EDID[i*32 + j + k]);
			printk("\n");
                }
            }
        }
    }
#endif
}

//------------------------------------------------------------

//BT1120 内同步信号只有DE, 如果没有外部sync 和DE,只能检测到H total 和 H/V active.
static void LT8618SX_Video_check(void)
{
    uint16_t h_tal, h_act, v_act;
    uint32_t CLK_Cnt;

    hobot_write_lt8618sxb(0xff, 0x82); //video

    h_tal = hobot_read_lt8618sxb(0x8f);
    h_tal = (h_tal << 8) + hobot_read_lt8618sxb(0x90);
    //	Debug_Printf("\r\nh_total = ");
    //    Debug_DispNum(h_tal);

    v_act = hobot_read_lt8618sxb(0x8b);
    v_act = (v_act << 8) + hobot_read_lt8618sxb(0x8c);
    //	Debug_Printf("\r\nv_active = ");
    //    Debug_DispNum(v_act);

    h_act = hobot_read_lt8618sxb(0x8d);
    h_act = (h_act << 8) + hobot_read_lt8618sxb(0x8e);
    //	Debug_Printf("\r\nh_active = ");
    //    Debug_DispNum(h_act);

    CLK_Cnt = (hobot_read_lt8618sxb(0x1d) & 0x0f) * 0x10000 + hobot_read_lt8618sxb(0x1e) * 0x100 + hobot_read_lt8618sxb(0x1f);
    // Pixel CLK =	CLK_Cnt * 1000
}

/***********************************************************/

static int LT8618SX_Reset(void)
{
	if (lt8618sxb_reset_pin >= 0) {
		gpio_direction_output(lt8618sxb_reset_pin, 0);
		msleep(100);
		gpio_direction_output(lt8618sxb_reset_pin, 1);
		msleep(100);
	}
	return 0;
}

/***********************************************************/

void LT8618SX_Initial(void)
{
    uint8_t Temp = 0x00;

    uint8_t OK_CNT = 0x00;
    uint8_t Jump_CNT = 0x00;
    uint8_t Jump_Num = 0x00;
    bool b_OK = 0;
    uint16_t v_act = 0x0000;
    uint16_t h_act = 0x0000;
    uint16_t h_total = 0x0000;

    //-----------------------------//

    Resolution_Num = _1080P60_; // 1080P60 BT1120 Input

    Use_DDRCLK = 0; // 1: DDR mode; 0: SDR (normal) mode

    HDMI_VIC = VIC_Num[Resolution_Num];

    if (Use_DDRCLK) {
        CLK_Num = DDR_PLL_Setting[Resolution_Num];
    } else {
        CLK_Num = SDR_PLL_Setting[Resolution_Num];
    }

    //-----------------------------//

    LT8618SX_Reset();

    hobot_write_lt8618sxb(0xff, 0x80); // register bank
    hobot_write_lt8618sxb(0xee, 0x01);

    //	hobot_write_lt8618sxb( 0xff, 0x80 );// register bank
    hobot_write_lt8618sxb(0x11, 0x00); //reset MIPI Rx logic.

    hobot_write_lt8618sxb(0x13, 0xf1);
    hobot_write_lt8618sxb(0x13, 0xf9); // Reset TTL video process

    // TTL mode
    hobot_write_lt8618sxb(0xff, 0x81); // register bank
    hobot_write_lt8618sxb(0x02, 0x66);
    hobot_write_lt8618sxb(0x0a, 0x06);
    hobot_write_lt8618sxb(0x15, 0x06);

    hobot_write_lt8618sxb(0x4e, 0xa8);

    hobot_write_lt8618sxb(0xff, 0x82);
    hobot_write_lt8618sxb(0x1b, 0x77);
    hobot_write_lt8618sxb(0x1c, 0xEC); // ring

    hobot_write_lt8618sxb(0xff, 0x80);
    hobot_write_lt8618sxb(0x0A, 0xF0);

    // TTL_Input_Digtal
    hobot_write_lt8618sxb(0xff, 0x82); // register bank
    hobot_write_lt8618sxb(0x45, _D8_D23_In_2); //RGB channel swap

    if (Use_DDRCLK) {
        hobot_write_lt8618sxb(0x4f, 0x80); // 0x80: dclk
    } else {
        hobot_write_lt8618sxb(0x4f, 0x40); // 0x40: txpll_clk
    }

    hobot_write_lt8618sxb(0x50, 0x00);

    msleep(100);

    hobot_write_lt8618sxb(0x51, 0x42); // Select BT rx decode det_vs/hs/de
    hobot_write_lt8618sxb(0x48, 0x08); // Embedded sync mode input enable.

    //----------------------------------------------------------//

    //Wait for the signal to be stable and decide whether the delay is necessary according to the actual situation
    msleep(100); // 等待信号稳定,根据实际情况决定是否需要延时

    LT8618SX_Video_check(); // For debug

    //------------------------PLL----------------------------------//
    // PLL
    hobot_write_lt8618sxb(0xff, 0x81);
    hobot_write_lt8618sxb(0x23, 0x40);
    hobot_write_lt8618sxb(0x24, 0x64); //icp set

    hobot_write_lt8618sxb(0x2e, 0x01); // 0x01
    hobot_write_lt8618sxb(0x2f, 0x10); // 0x00
    hobot_write_lt8618sxb(0x26, 0x55);

    if (Use_DDRCLK) {
        hobot_write_lt8618sxb(0x4d, 0x05);

        hobot_write_lt8618sxb(0x25, LT8618SXB_BT1120_DDR_PLL_setting[CLK_Num][0x00]); ////0x05 //pre-divider3 ddr 02
        hobot_write_lt8618sxb(0x2c, LT8618SXB_BT1120_DDR_PLL_setting[CLK_Num][0x01]); // 9e
        hobot_write_lt8618sxb(0x2d, LT8618SXB_BT1120_DDR_PLL_setting[CLK_Num][0x02]); // 88

        hobot_write_lt8618sxb(0x27, 0x66); //0x60 //ddr 0x66
        hobot_write_lt8618sxb(0x28, 0x88); // 0x88
    } else {
        hobot_write_lt8618sxb(0x4d, 0x00);

        hobot_write_lt8618sxb(0x25, LT8618SXB_BT1120_SDR_PLL_setting[CLK_Num][0x00]); ////0x05 //pre-divider3 ddr 02
        hobot_write_lt8618sxb(0x2c, LT8618SXB_BT1120_SDR_PLL_setting[CLK_Num][0x01]); // 9e
        hobot_write_lt8618sxb(0x2d, LT8618SXB_BT1120_SDR_PLL_setting[CLK_Num][0x02]); // 88

        hobot_write_lt8618sxb(0x27, 0x06); //0x60 //ddr 0x66
        hobot_write_lt8618sxb(0x28, 0x00); // 0x88
        //hobot_write_lt8618sxb(0x27, 0x60); //0x60 //ddr 0x66
        //hobot_write_lt8618sxb(0x28, 0x00); // 0x88
    }

    // as long as changing the resolution or changing the input clock,	You need to configure the following registers.
    hobot_write_lt8618sxb(0xff, 0x82);
    hobot_write_lt8618sxb(0xde, 0x00);
    hobot_write_lt8618sxb(0xde, 0xc0);

    hobot_write_lt8618sxb(0xff, 0x80);
    hobot_write_lt8618sxb(0x16, 0xf1);
    hobot_write_lt8618sxb(0x16, 0xf3);

    //-------------------------------------------

    // IIS Input
    hobot_write_lt8618sxb(0xff, 0x82); // register bank
    hobot_write_lt8618sxb(0xd6, dvi_mode ? 0x0e : 0x8e); // bit7 = 0 : DVI output; bit7 = 1: HDMI output
    hobot_write_lt8618sxb(0xd7, 0x04); //sync polarity

    hobot_write_lt8618sxb(0xff, 0x84); // register bank
    hobot_write_lt8618sxb(0x06, 0x08);
    hobot_write_lt8618sxb(0x07, 0x10);

    hobot_write_lt8618sxb(0x0f, 0x0b + Sample_Freq[_48KHz]);

    hobot_write_lt8618sxb(0x34, 0xd4); //CTS_N / 2; 32bit
    //	hobot_write_lt8618sxb( 0x34, 0xd5 );	//CTS_N / 4; 16bit

    hobot_write_lt8618sxb(0x35, (uint8_t)(IIS_N[_48KHz] / 0x10000));
    hobot_write_lt8618sxb(0x36, (uint8_t)((IIS_N[_48KHz] & 0x00FFFF) / 0x100));
    hobot_write_lt8618sxb(0x37, (uint8_t)(IIS_N[_48KHz] & 0x0000FF));

    hobot_write_lt8618sxb(0x3c, 0x21); // Null packet enable

    //-------------------------------------------

    /*		// SPDIF Input
   hobot_write_lt8618sxb( 0xff, 0x82 );// register bank
   hobot_write_lt8618sxb( 0xd6, 0x8e );
   hobot_write_lt8618sxb( 0xd7, 0x04 );	//sync polarity

   hobot_write_lt8618sxb( 0xff, 0x84 );// register bank
   hobot_write_lt8618sxb( 0x06, 0x0c );
   hobot_write_lt8618sxb( 0x07, 0x10 );

   hobot_write_lt8618sxb( 0x0f, 0x0b + Sample_Freq[_48KHz]);

   hobot_write_lt8618sxb( 0x34, 0xd4 );//CTS_N
   //	hobot_write_lt8618sxb( 0x34, 0xd5 );	//CTS_N / 4; 16bit

   hobot_write_lt8618sxb( 0x35, (uint8_t)(IIS_N[_48KHz]/0x10000) );
   hobot_write_lt8618sxb( 0x36, (uint8_t)((IIS_N[_48KHz]&0x00FFFF)/0x100) );
   hobot_write_lt8618sxb( 0x37, (uint8_t)(IIS_N[_48KHz]&0x0000FF) );

   hobot_write_lt8618sxb( 0x3c, 0x21 );	// Null packet enable

 */

    //-------------------------------------------

    // color space config
    hobot_write_lt8618sxb(0xff, 0x82); // register bank
    //	hobot_write_lt8618sxb( 0xb9, 0x08 );// YCbCr444 to RGB
    //	hobot_write_lt8618sxb( 0xb9, 0x18 );// YCbCr422 to RGB
    //      HDMI_CS=0x10;

    //	hobot_write_lt8618sxb( 0xb9, 0x20 );// YCbCr444 to YCbCr422
    //	hobot_write_lt8618sxb( 0xb9, 0xa0 );// RGB to YCbCr422
    //      HDMI_CS=0x30;

    //	hobot_write_lt8618sxb( 0xb9, 0x80 );// RGB to YCbCr444
    //	hobot_write_lt8618sxb( 0xb9, 0x10 );// YCbCr422 to YCbCr444
    //      HDMI_CS=0x50;

    hobot_write_lt8618sxb(0xb9, dvi_mode ? 0x18 : 0x00); // No csc//BT1120_16BIT; hdmi mode 0 dvi mode 0x18
    HDMI_CS = 0x30;

    //-------------------------------------------

#ifdef _LT8618_HDCP_
    LT8618SX_HDCP_Init();
#endif

    //-------------------------------------------

    //AVI

    /********************************************************************************
	   The 0x43 register is checksums,
	   changing the value of the 0x45 or 0x47 register,
	   and the value of the 0x43 register is also changed.
	   0x43, 0x44, 0x45, and 0x47 are the sum of the four register values is 0x6F.
	 *********************************************************************************/

    hobot_write_lt8618sxb(0xff, 0x84);
    hobot_write_lt8618sxb(0x43, 0x6f - (HDMI_VIC + HDMI_CS + HDMI_PA)); // avi packet checksum ,avi_pb0
    hobot_write_lt8618sxb(0x44, HDMI_CS); // color space: ycbcr422 0x30; RGB 0x10
    hobot_write_lt8618sxb(0x45, HDMI_PA); // 0x19:4:3 ; 0x2A : 16:9
    hobot_write_lt8618sxb(0x47, 0x00 + HDMI_VIC); //

    //-------------------------------------------------------------------------------------

    // HDMI_TX_Phy
    hobot_write_lt8618sxb(0xff, 0x81); // register bank
    hobot_write_lt8618sxb(0x30, 0xea);
    hobot_write_lt8618sxb(0x31, 0x44);
    hobot_write_lt8618sxb(0x32, 0x4a);
    hobot_write_lt8618sxb(0x33, 0x0b);
    hobot_write_lt8618sxb(0x34, 0x00);
    hobot_write_lt8618sxb(0x35, 0x00);
    hobot_write_lt8618sxb(0x36, 0x00);
    hobot_write_lt8618sxb(0x37, 0x44);
    hobot_write_lt8618sxb(0x3f, 0x0f);

    hobot_write_lt8618sxb(0x40, 0xb0); //0xa0 -- CLK tap0 swing
    hobot_write_lt8618sxb(0x41, 0x68); //0xa0 -- D0 tap0 swing
    hobot_write_lt8618sxb(0x42, 0x68); //0xa0 -- D1 tap0 swing
    hobot_write_lt8618sxb(0x43, 0x68); //0xa0 -- D2 tap0 swing

    hobot_write_lt8618sxb(0x44, 0x0a);

    //-------------------------------------------

    //-------------------------------------------
    LT8918SX_Read_EDID(); // Read TV  EDID
    //-------------------------------------------

#ifdef _LT8618_HDCP_
    LT8618SX_HDCP_Enable();
#endif

    //-------------------------------------------

    // BT1120 内同步信号只有DE, 如果没有外部sync 和DE,Video check 只能检测到H/V total 和 H/V active.
    // BT1120 的Timing 值设置就不能通过Video check读取timing状态寄存器的值来设置，需要根据前端BT1120分辨率来设置。
    // The synchronization signal in BT1120 is only DE. Without external sync and DE, Video check can only detect H/V total and H/V active.
    // BT1120 Timing value settings can not be set by reading the value of the timing status register through Video check, which needs to be set according to the front BT1120 resolution.

    //LT8618SX_BT1120_Set

    hobot_write_lt8618sxb(0xff, 0x82);
    hobot_write_lt8618sxb(0x20, (uint8_t)(Resolution_Timing[Resolution_Num][H_Active] / 256));
    hobot_write_lt8618sxb(0x21, (uint8_t)(Resolution_Timing[Resolution_Num][H_Active] % 256));
    hobot_write_lt8618sxb(0x22, (uint8_t)(Resolution_Timing[Resolution_Num][H_FrontPorch] / 256));
    hobot_write_lt8618sxb(0x23, (uint8_t)(Resolution_Timing[Resolution_Num][H_FrontPorch] % 256));
    hobot_write_lt8618sxb(0x24, (uint8_t)(Resolution_Timing[Resolution_Num][H_SyncWid] / 256));
    hobot_write_lt8618sxb(0x25, (uint8_t)(Resolution_Timing[Resolution_Num][H_SyncWid] % 256));
    hobot_write_lt8618sxb(0x26, 0x00);
    hobot_write_lt8618sxb(0x27, 0x00);
    hobot_write_lt8618sxb(0x36, (uint8_t)(Resolution_Timing[Resolution_Num][V_Active] / 256));
    hobot_write_lt8618sxb(0x37, (uint8_t)(Resolution_Timing[Resolution_Num][V_Active] % 256));
    hobot_write_lt8618sxb(0x38, (uint8_t)(Resolution_Timing[Resolution_Num][V_FrontPorch] / 256));
    hobot_write_lt8618sxb(0x39, (uint8_t)(Resolution_Timing[Resolution_Num][V_FrontPorch] % 256));
    hobot_write_lt8618sxb(0x3a, (uint8_t)(Resolution_Timing[Resolution_Num][V_BackPorch] / 256));
    hobot_write_lt8618sxb(0x3b, (uint8_t)(Resolution_Timing[Resolution_Num][V_BackPorch] % 256));
    hobot_write_lt8618sxb(0x3c, (uint8_t)(Resolution_Timing[Resolution_Num][V_SyncWid] / 256));
    hobot_write_lt8618sxb(0x3d, (uint8_t)(Resolution_Timing[Resolution_Num][V_SyncWid] % 256));

    //-------------------------------------------

    //hobot_write_lt8618sxb( 0xff, 0x80 );   // register bank//20191121
    //hobot_write_lt8618sxb( 0x13, 0xf1 );//20191121
    //msleep( 5 );//20191121
    //hobot_write_lt8618sxb( 0x13, 0xf9 );   // Reset TTL video process//20191121

    // phase config
    {
        while (Temp <= 0x09) {
            //	msleep( 100 );
            hobot_write_lt8618sxb(0xff, 0x81);
            hobot_write_lt8618sxb(0x27, (0x60 + Temp));
            hobot_write_lt8618sxb(0x4d, 0x0d);

            hobot_write_lt8618sxb(0xff, 0x80);
            hobot_write_lt8618sxb(0x13, 0xf1); //ttl video process reset///20191121
            hobot_write_lt8618sxb(0x12, 0xfb); //video check reset//20191121
            //msleep( 50 ); // > 30ms//20191121
            msleep(1); //add 20191121
            hobot_write_lt8618sxb(0x12, 0xff); //20191121
            hobot_write_lt8618sxb(0x13, 0xf9); //20191121
            msleep(120); //大于两帧20191121

            hobot_write_lt8618sxb(0xff, 0x82);

            //	v_act  = hobot_read_lt8618sxb( 0x8b )*0x100 + hobot_read_lt8618sxb( 0x8c );

            h_total = hobot_read_lt8618sxb(0x8f);
            h_total = (h_total << 8) + hobot_read_lt8618sxb(0x90);

            v_act = hobot_read_lt8618sxb(0x8b);
            v_act = (v_act << 8) + hobot_read_lt8618sxb(0x8c);

            h_act = hobot_read_lt8618sxb(0x8d);
            h_act = (h_act << 8) + hobot_read_lt8618sxb(0x8e);

            //printk("phase   =%d\n", Temp); //20191121
            //printk("h_total =%d\n", h_total); //20191121
            //printk("v_act   =%d\n", v_act); //20191121
            //printk("h_act   =%d\n", h_act); //20191121

            if ((v_act > (Resolution_Timing[Resolution_Num][V_Active] - 5)) && (v_act < (Resolution_Timing[Resolution_Num][V_Active] + 5))
                && (h_act > (Resolution_Timing[Resolution_Num][H_Active] - 5)) && (h_act < (Resolution_Timing[Resolution_Num][H_Active] + 5))
                && (h_total > (Resolution_Timing[Resolution_Num][H_Total] - 5)) && (h_total < (Resolution_Timing[Resolution_Num][H_Total] + 5))) {
                OK_CNT++;

                if (b_OK == 0) {
                    b_OK = 1;
                    Jump_CNT++;

                    if ((Jump_CNT == 1) || (Jump_CNT == 3)) {
                        Jump_Num = Temp;
                    }
                }
            } else {
                if (b_OK) {
                    b_OK = 0;
                    Jump_CNT++;
                }
            }

            Temp++;
        }

        hobot_write_lt8618sxb(0xff, 0x81);

        if ((Jump_CNT == 1) || (Jump_CNT == 2) || (Jump_CNT == 4)) {
            hobot_write_lt8618sxb(0x27, (0x60 + Jump_Num + (OK_CNT / 2)));
        } else if (Jump_CNT == 3) {
            hobot_write_lt8618sxb(0x27, (0x60 + (Jump_Num + (OK_CNT / 2)) % 0x0a));
        }
        printk("lt8618 0x8127=%x\n", hobot_read_lt8618sxb(0x27)); //20191121
    }
}

/************************************** The End Of File **************************************/

