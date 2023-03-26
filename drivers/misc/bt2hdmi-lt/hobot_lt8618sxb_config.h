/************************************************************
 * Copyright (C), 2009-2011, Donjin Tech. Co., Ltd.
 * FileName:		// 文件名
 * Author:			// 作者
 * Date:			// 日期
 * Description:		// 模块描述
 * Version:			// 版本信息
 * Function List:	// 主要函数及其功能
 *     1. -------
 * History:			// 历史修改记录
 *     <author>  <time>   <version >   <desc>
 *     David    96/10/12     1.0     build this moudle
 ***********************************************************/

/************************************************************
*	ProjectName:	   LT8619C
*	FileName:	       lt8618EXB.h
*	BuildData:	     2017-06-23
*	Version：        V1.0.0
*	Company:	       Lontium
************************************************************/

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

#ifndef DRIVERS_MISC_BT2HDMI_LT_HOBOT_LT8618SXB_CONFIG_H_
#define DRIVERS_MISC_BT2HDMI_LT_HOBOT_LT8618SXB_CONFIG_H_

#define _LT8618_
//#define _LT8618EXB_
#define _LT8618SXB_

//#define _LT8619C_

//#define _EDID_

#ifdef _LT8618SXB_
#define _Read_TV_EDID_
#endif

#define _YUV_			// _YUV_
//#define _RGB888_ // _LVDS_
//#define _LVDS_

#ifdef _YUV_

#define _BT1120_		// _BT1120_  /  _BT656_

#ifdef _BT656_
//#define _8bit_ // 8bit YC
#define _10bit_			// 10bit YC
//#define _12bit_ // 12bit YC
#endif

#ifdef _BT1120_
#define _16bit_			// 16bit YC
//#define _20bit_  // 20bit YC
//#define _24bit_ // 24bit YC
#endif

//*****************************/

// LT8618SXB BT656/BT1120 输入使用能内同步
//#define _i_input_ // for  test
#define _Embedded_sync_

//-----------------------------/

// LT8618SXB BT656/BT1120 输入使用外部sync、DE；这样的话就不需要设置TIMING参数。
//#define _YUV_DE_En_

//*****************************/
#endif

//#define _DDR_
#define _SDR_

//#define _LT8618_HDCP_

//#define _LT8619_GPIO_En_
//#define _LT8619_GPIO13_15_En_

#define _uart_debug_

#define _Phase_Debug_

//--------------------------------------//
//#define _RGB_Input_
//#define _BT1120_Input_
//#define _BT656_Input_
//--------------------------------------//

//--------------------------------------//

#define LT8618_ADR 0x76

#define LT8618EX_ADR 0x76
#define LT8618EX_ADR_last 0x7e

//--------------------------------------//

#define _RGB_Input_ 0x02
#define _YUV_Input_ 0x03

#define _BT656_Input_ 0x34
#define _BT1120_Input_ 0x33
#define _YCbCr444_Input_ 0x35
#define _YCbCr422_16bit_Input_ 0x36

//--------------------------------------//

#define _YUV_Low_16bit_YC_Swap_Input 0x12	// D0 ~ D15
#define _YUV_High_16bit_YC_Swap_Input 0x10	// D8 ~ D23

#define _YUV_Low_16bit_YC_No_Swap_Input 0x14	// D0 ~ D15
#define _YUV_High_16bit_YC_No_Swap_Input 0x11	// D8 ~ D23

//--------------------------------------//

// 设置IIS 音频输入，IIS和SPDIF只能二选一
#define _IIS_Input_ 0x11

// 设置SPDIF 输出，IIS和SPDIF只能二选一
#define _SPDIF_Input_ 0x39
//--------------------------------------//

// for LT8618SXB
#define _SXInPut_BGR_ 0x70	// B0~B7[D0:D7];G0~G7[D8:D15];R0~R7[D16:D23];
#define _SXInPut_GBR_ 0x60	// G0~G7[D0:D7];B0~B7[D8:D15];R0~R7[D16:D23];

#define _SXInPut_BRG_ 0x50	// B0~B7[D0:D7];R0~R7[D8:D15];G0~G7[D16:D23];
#define _SXInPut_RBG_ 0x40	// R0~R7[D0:D7];B0~B7[D8:D15];G0~G7[D16:D23];

#define _SXInPut_GRB_ 0x30	// G0~G7[D0:D7];R0~R7[D8:D15];B0~B7[D16:D23];
#define _SXInPut_RGB_ 0x20	// R0~R7[D0:D7];G0~G7[D8:D15];B0~B7[D16:D23];

//--------------------------------------//

//#define _D0_D7_In     0x40    // BT656 input from D0 to D7 of LT8618SXB pins.
//#define _D8_D15_In    0x00    // BT656 input from D8 to D15 of LT8618SXB pins.
//#define _D16_D23_In 0x60    // BT656 input from D16 to D23 of LT8618SXB pins.

#ifdef _BT656_
#ifdef _8bit_
// BT656 8bit
#define _D0_D7_In_ 0x40		// BT656 input from D0 to D7 of LT8618SXB pins.
#define _D8_D15_In_ 0x00	// BT656 input from D8 to D15 of LT8618SXB pins.
#define _D16_D23_In_ 0x60	// BT656 input from D16 to D23 of LT8618SXB pins.

#define _YC_Channel_ _D8_D15_In_

// bit1/bit0 = 0:Input data color depth is 8 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x00
#else

// BT656 10bit/12bit

// BT656 10bit input from D2,D3, D8 to D15 of LT8618SXB. 
// BT656 12bit input from D0 to D3, D8 to D15 of LT8618SXB.
#define _D8_D15_In_ 0x00
#define _YC_Channel_ _D8_D15_In_

#ifdef _10bit_
// bit1 = 0;bit0 = 1: Input data color depth is 10 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x01
#else // 12bit
// bit1 = 1;bit0 = 0: Input data color depth is 12 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x02
#endif

#endif
#endif
//--------------------------------------//

#ifdef _BT1120_
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

#define _YC_Channel_ _D8_D23_In_

// bit1/bit0 = 0:Input data color depth is 8 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x00
#else
// BT1120 20bit/24bit
// BT1120 10bit input from D2 ~ D3(Y0~Y1),D6 ~ D7(C0~C1), 
// D8 ~ D15(Y2~Y9),D16 ~ D23(C2~C9) of LT8618SXB. 
// BT1120 12bit input from D0 ~ D3(Y0~Y3),D4 ~ D7(C0~C3), 
// D8 ~ D15(Y4~Y11),D16 ~ D23(C4~C11) of LT8618SXB.
#define _YC_Channel_ 0x70


#ifdef _20bit_
// bit1 = 0;bit0 = 1: Input data color depth is 10 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x01
#else // 24bit
// bit1 = 1;bit0 = 0: Input data color depth is 12 bit enable for BT
#define _Reg0x8248_D1_D0_ 0x02
#endif

#endif
#endif
//--------------------------------------//

//--------------------------------------//

#define Output_RGB888 0x00
#define Output_YCbCr444 0x01
#define Output_YCbCr422 0x02

//--------------------------------------//

#define Check_Ycbcr444 0x04
#define Check_RGB888 0x44
#define Out_Ycbcr444 0x50
#define out_RGB888 0x10
#define Color_Ycbcr444 0x02
#define Color_RGB888 0x00
#define Color_Ycbcr422 0x12

//--------------------------------------//
enum { _480P60_ = 0,
	_576P50_,
	_720P60_,
	_720P50_,
	//  _720P30_,
	//  _720P25_,
	_1080P60_,
	_1080P50_,
	_1080P30_,
	//  _1080P25_,

	//  _1080i60_,
	//  _1080i50_,
	_4K30_,			// When setting bt656 input, 4k30 resolution is not supported.

	//  _800x600P60_,
	//  _1024x768P60_,
	Resolution_Num
};

#define _No_input_ 0xff
/*
#define  VESA_720x480p_60_27MHz 0
#define  VESA_720x576p_50_27MHz ( VESA_720x480p_60_27MHz + 1 )

//#define  VESA_720x480i_60_27MHz ( VESA_720x576p_50_27MHz + 1 )
//#define  VESA_720x576i_50_27MHz ( VESA_720x480i_60_27MHz + 1 )

//#define   VESA_720x480p_60_54MHz		(VESA_720x576i_50_27MHz + 1)
//#define   VESA_720x576p_50_54MHz		(VESA_720x480p_60_54MHz + 1)

//#define   VESA_1024x768_60		(VESA_720x576p_50_54MHz + 1)

#define  VESA_1280x720p_60	( VESA_720x576p_50_27MHz + 1 )
#define  VESA_1280x720p_50	( VESA_1280x720p_60 + 1 )
#define  VESA_1280x720p_30	( VESA_1280x720p_50 + 1 )
#define  VESA_1280x720p_25	( VESA_1280x720p_30 + 1 )
//#define   VESA_1280x720p_24		(VESA_1280x720p_25 + 1)

//#define   VESA_1366x768_60NB		(VESA_1280x720p_24 + 1)

#define  VESA_1920x1080p_60 ( VESA_1280x720p_25 + 1 )
#define  VESA_1920x1080p_50 ( VESA_1920x1080p_60 + 1 )
//#define   VESA_1920x1080p_30		(VESA_1920x1080p_50 + 1)
//#define   VESA_1920x1080p_25		(VESA_1920x1080p_30 + 1)
//#define   VESA_1920x1080p_24		(VESA_1920x1080p_25 + 1)

#define  VESA_1920x1080i_60 ( VESA_1920x1080p_50 + 1 )
#define  VESA_1920x1080i_50 ( VESA_1920x1080i_60 + 1 )

//#define   VESA_640x480p_60_25MHz		(VESA_1920x1080i_50 + 1)
#define  VESA_3840x2160p_30Hz ( VESA_1920x1080i_50 + 1 )

#define  Resolution_Num2 ( VESA_3840x2160p_30Hz + 1 )
//*/
//--------------------------------------//
#define _VIC_PC 0x00

#define _VIC_480P60 2		// 3
#define _VIC_576P50 17		// 18

#define _VIC_480i60 8		// 9
#define _VIC_576i50 23		// 24

#define _VIC_720P60 4		// 69
#define _VIC_720P50 19		// 68
#define _VIC_720P30 62		// 67
#define _VIC_720P25 61		// 66

#define _VIC_1080P60 16		// 76
#define _VIC_1080P50 31		// 75

#define _VIC_1080P30 34		// 74
#define _VIC_1080P25 33		// 73
#define _VIC_1080P24 32		// 72

#define _VIC_1080i60 5
#define _VIC_1080i50 20

#define _VIC_4K30 95		// 105

//=================================//

#ifdef _LT8618SXB_
#define _LT8618SXB_D0_D7_Bit_Swap_En 0x01	// D0~D7 High/Low bit swap enable
#define _LT8618SXB_D0_D7_Bit_Swap_Dis 0x00

#define _LT8618SXB_D8_D15_Bit_Swap_En 0x02	// D8~D15 High/Low bit swap enable
#define _LT8618SXB_D8_D15_Bit_Swap_Dis 0x00

#define _LT8618SXB_D16_D23_Bit_Swap_En 0x04	// D16~D23 High/Low bit swap enable
#define _LT8618SXB_D16_D23_Bit_Swap_Dis 0x00
#endif

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

void LT8618SXB_Reset(void);

#endif // DRIVERS_MISC_BT2HDMI_LT_HOBOT_LT8618SXB_CONFIG_H_
/**************************** The End Of File *******************************/
