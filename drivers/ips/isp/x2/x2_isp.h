/*
 * Copyright (C) 2018/04/20 Horizon Robotics Co., Ltd.
 *
 * x2_isp.h
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef   __X2_ISP__
#define   __X2_ISP__

/*************************************************************
 * base register
*************************************************************/

#define X2_ISP_REG_ISP_CONFIG			(0x200)
#define X2_ISP_REG_isp_video_in_size	(0x204)
#define X2_ISP_REG_DPP_BL_B_0			(0x208)
#define X2_ISP_REG_DPP_BL_B_1			(0x20C)
#define X2_ISP_REG_DPP_BL_B_2			(0x210)
#define X2_ISP_REG_DPP_BL_B_3			(0x214)
#define X2_ISP_REG_DPP_DS_S				(0x218)
#define X2_ISP_REG_DPP_AMP_W_0			(0x21C)
#define X2_ISP_REG_DPP_AMP_W_1			(0x220)
#define X2_ISP_REG_DPP_AMP_W_2			(0x224)
#define X2_ISP_REG_DPP_AMP_W_3			(0x228)
#define X2_ISP_REG_DPP_AMP_B_0			(0x22C)
#define X2_ISP_REG_DPP_AMP_B_1			(0x230)
#define X2_ISP_REG_DPP_AMP_B_2			(0x234)
#define X2_ISP_REG_DPP_AMP_B_3			(0x238)
#define X2_ISP_REG_DPP_LMT_L_0			(0x23C)
#define X2_ISP_REG_DPP_LMT_L_1			(0x240)
#define X2_ISP_REG_DPP_LMT_L_2			(0x244)
#define X2_ISP_REG_DPP_LMT_L_3			(0x248)
#define X2_ISP_REG_DPP_LMT_H_0			(0x24C)
#define X2_ISP_REG_DPP_LMT_H_1			(0x250)
#define X2_ISP_REG_DPP_LMT_H_2			(0x254)
#define X2_ISP_REG_DPP_LMT_H_3			(0x258)
#define X2_ISP_REG_HMP_DEG_W			(0x25C)
#define X2_ISP_REG_HMP_DEG_B			(0x260)
#define X2_ISP_REG_HMP_WBG_W_0			(0x264)
#define X2_ISP_REG_HMP_WBG_W_1			(0x268)
#define X2_ISP_REG_HMP_WBG_W_2			(0x26C)
#define X2_ISP_REG_HMP_WBG_W_3			(0x270)
#define X2_ISP_REG_HMP_WBG_B_0			(0x274)
#define X2_ISP_REG_HMP_WBG_B_1			(0x278)
#define X2_ISP_REG_HMP_WBG_B_2			(0x27C)
#define X2_ISP_REG_HMP_WBG_B_3			(0x280)
#define X2_ISP_REG_HMP_CB1_W_0			(0x284)
#define X2_ISP_REG_HMP_CB1_W_1			(0x288)
#define X2_ISP_REG_HMP_CB1_W_2			(0x28C)
#define X2_ISP_REG_HMP_CB1_W_3			(0x290)
#define X2_ISP_REG_HMP_CB1_B_0			(0x294)
#define X2_ISP_REG_HMP_CB1_B_1			(0x298)
#define X2_ISP_REG_HMP_CB1_B_2			(0x29C)
#define X2_ISP_REG_HMP_CB1_B_3			(0x2A0)
#define X2_ISP_REG_HMP_GMA_PRE_W_0		(0x2A4)
#define X2_ISP_REG_HMP_GMA_PRE_W_1		(0x2A8)
#define X2_ISP_REG_HMP_GMA_PRE_W_2		(0x2AC)
#define X2_ISP_REG_HMP_GMA_PRE_W_3		(0x2B0)
#define X2_ISP_REG_HMP_GMA_PRE_B_0		(0x2B4)
#define X2_ISP_REG_HMP_GMA_PRE_B_1		(0x2B8)
#define X2_ISP_REG_HMP_GMA_PRE_B_2		(0x2BC)
#define X2_ISP_REG_HMP_GMA_PRE_B_3		(0x2C0)
#define X2_ISP_REG_HMP_GMA_LUT_SEL		(0x2C4)
#define X2_ISP_REG_HMP_GMA_LUT1_W_0		(0x2C8)
#define X2_ISP_REG_HMP_GMA_LUT1_W_1		(0x2CC)
#define X2_ISP_REG_HMP_GMA_LUT1_W_2		(0x2D0)
#define X2_ISP_REG_HMP_GMA_LUT1_W_3		(0x2D4)
#define X2_ISP_REG_HMP_GMA_LUT1_W_4		(0x2D8)
#define X2_ISP_REG_HMP_GMA_LUT1_W_5		(0x2DC)
#define X2_ISP_REG_HMP_GMA_LUT1_W_6		(0x2E0)
#define X2_ISP_REG_HMP_GMA_LUT1_W_7		(0x2E4)
#define X2_ISP_REG_HMP_GMA_LUT1_W_8		(0x2E8)
#define X2_ISP_REG_HMP_GMA_LUT1_W_9		(0x2EC)
#define X2_ISP_REG_HMP_GMA_LUT1_W_10	(0x2F0)
#define X2_ISP_REG_HMP_GMA_LUT1_W_11	(0x2F4)
#define X2_ISP_REG_HMP_GMA_LUT1_W_12	(0x2F8)
#define X2_ISP_REG_HMP_GMA_LUT1_W_13	(0x2FC)
#define X2_ISP_REG_HMP_GMA_LUT1_W_14	(0x300)
#define X2_ISP_REG_HMP_GMA_LUT1_W_15	(0x304)
#define X2_ISP_REG_HMP_GMA_LUT1_W_16	(0x308)
#define X2_ISP_REG_HMP_GMA_LUT1_W_17	(0x30C)
#define X2_ISP_REG_HMP_GMA_LUT1_W_18	(0x310)
#define X2_ISP_REG_HMP_GMA_LUT1_W_19	(0x314)
#define X2_ISP_REG_HMP_GMA_LUT1_W_20	(0x318)
#define X2_ISP_REG_HMP_GMA_LUT1_W_21	(0x31C)
#define X2_ISP_REG_HMP_GMA_LUT1_W_22	(0x320)
#define X2_ISP_REG_HMP_GMA_LUT1_W_23	(0x324)
#define X2_ISP_REG_HMP_GMA_LUT1_W_24	(0x328)
#define X2_ISP_REG_HMP_GMA_LUT1_W_25	(0x32C)
#define X2_ISP_REG_HMP_GMA_LUT1_W_26	(0x330)
#define X2_ISP_REG_HMP_GMA_LUT1_W_27	(0x334)
#define X2_ISP_REG_HMP_GMA_LUT1_W_28	(0x338)
#define X2_ISP_REG_HMP_GMA_LUT1_W_29	(0x33C)
#define X2_ISP_REG_HMP_GMA_LUT1_W_30	(0x340)
#define X2_ISP_REG_HMP_GMA_LUT2_W_0		(0x344)
#define X2_ISP_REG_HMP_GMA_LUT2_W_1		(0x348)
#define X2_ISP_REG_HMP_GMA_LUT2_W_2		(0x34C)
#define X2_ISP_REG_HMP_GMA_LUT2_W_3		(0x350)
#define X2_ISP_REG_HMP_GMA_LUT2_W_4		(0x354)
#define X2_ISP_REG_HMP_GMA_LUT2_W_5		(0x358)
#define X2_ISP_REG_HMP_GMA_LUT2_W_6		(0x35C)
#define X2_ISP_REG_HMP_GMA_LUT2_W_7		(0x360)
#define X2_ISP_REG_HMP_GMA_LUT2_W_8		(0x364)
#define X2_ISP_REG_HMP_GMA_LUT2_W_9		(0x368)
#define X2_ISP_REG_HMP_GMA_LUT2_W_10	(0x36C)
#define X2_ISP_REG_HMP_GMA_LUT2_W_11	(0x370)
#define X2_ISP_REG_HMP_GMA_LUT2_W_12	(0x374)
#define X2_ISP_REG_HMP_GMA_LUT2_W_13	(0x378)
#define X2_ISP_REG_HMP_GMA_LUT2_W_14	(0x37C)
#define X2_ISP_REG_HMP_GMA_LUT2_W_15	(0x380)
#define X2_ISP_REG_HMP_GMA_LUT2_W_16	(0x384)
#define X2_ISP_REG_HMP_GMA_LUT2_W_17	(0x388)
#define X2_ISP_REG_HMP_GMA_LUT2_W_18	(0x38C)
#define X2_ISP_REG_HMP_GMA_LUT2_W_19	(0x390)
#define X2_ISP_REG_HMP_GMA_LUT2_W_20	(0x394)
#define X2_ISP_REG_HMP_GMA_LUT2_W_21	(0x398)
#define X2_ISP_REG_HMP_GMA_LUT2_W_22	(0x39C)
#define X2_ISP_REG_HMP_GMA_LUT2_W_23	(0x3A0)
#define X2_ISP_REG_HMP_GMA_LUT2_W_24	(0x3A4)
#define X2_ISP_REG_HMP_GMA_LUT2_W_25	(0x3A8)
#define X2_ISP_REG_HMP_GMA_LUT2_W_26	(0x3AC)
#define X2_ISP_REG_HMP_GMA_LUT2_W_27	(0x3B0)
#define X2_ISP_REG_HMP_GMA_LUT2_W_28	(0x3B4)
#define X2_ISP_REG_HMP_GMA_LUT2_W_29	(0x3B8)
#define X2_ISP_REG_HMP_GMA_LUT2_W_30	(0x3BC)
#define X2_ISP_REG_HMP_GMA_LUT3_W_0		(0x3C0)
#define X2_ISP_REG_HMP_GMA_LUT3_W_1		(0x3C4)
#define X2_ISP_REG_HMP_GMA_LUT3_W_2		(0x3C8)
#define X2_ISP_REG_HMP_GMA_LUT3_W_3		(0x3CC)
#define X2_ISP_REG_HMP_GMA_LUT3_W_4		(0x3D0)
#define X2_ISP_REG_HMP_GMA_LUT3_W_5		(0x3D4)
#define X2_ISP_REG_HMP_GMA_LUT3_W_6		(0x3D8)
#define X2_ISP_REG_HMP_GMA_LUT3_W_7		(0x3DC)
#define X2_ISP_REG_HMP_GMA_LUT3_W_8		(0x3E0)
#define X2_ISP_REG_HMP_GMA_LUT3_W_9		(0x3E4)
#define X2_ISP_REG_HMP_GMA_LUT3_W_10	(0x3E8)
#define X2_ISP_REG_HMP_GMA_LUT3_W_11	(0x3EC)
#define X2_ISP_REG_HMP_GMA_LUT3_W_12	(0x3F0)
#define X2_ISP_REG_HMP_GMA_LUT3_W_13	(0x3F4)
#define X2_ISP_REG_HMP_GMA_LUT3_W_14	(0x3F8)
#define X2_ISP_REG_HMP_GMA_LUT3_W_15	(0x3FC)
#define X2_ISP_REG_HMP_GMA_LUT3_W_16	(0x400)
#define X2_ISP_REG_HMP_GMA_LUT3_W_17	(0x404)
#define X2_ISP_REG_HMP_GMA_LUT3_W_18	(0x408)
#define X2_ISP_REG_HMP_GMA_LUT3_W_19	(0x40C)
#define X2_ISP_REG_HMP_GMA_LUT3_W_20	(0x410)
#define X2_ISP_REG_HMP_GMA_LUT3_W_21	(0x414)
#define X2_ISP_REG_HMP_GMA_LUT3_W_22	(0x418)
#define X2_ISP_REG_HMP_GMA_LUT3_W_23	(0x41C)
#define X2_ISP_REG_HMP_GMA_LUT3_W_24	(0x420)
#define X2_ISP_REG_HMP_GMA_LUT3_W_25	(0x424)
#define X2_ISP_REG_HMP_GMA_LUT3_W_26	(0x428)
#define X2_ISP_REG_HMP_GMA_LUT3_W_27	(0x42C)
#define X2_ISP_REG_HMP_GMA_LUT3_W_28	(0x430)
#define X2_ISP_REG_HMP_GMA_LUT3_W_29	(0x434)
#define X2_ISP_REG_HMP_GMA_LUT3_W_30	(0x438)
#define X2_ISP_REG_HMP_GMA_POST_W_0		(0x43C)
#define X2_ISP_REG_HMP_GMA_POST_W_1		(0x440)
#define X2_ISP_REG_HMP_GMA_POST_W_2		(0x444)
#define X2_ISP_REG_HMP_GMA_POST_W_3		(0x448)
#define X2_ISP_REG_HMP_GMA_POST_B_0		(0x44C)
#define X2_ISP_REG_HMP_GMA_POST_B_1		(0x450)
#define X2_ISP_REG_HMP_GMA_POST_B_2		(0x454)
#define X2_ISP_REG_HMP_GMA_POST_B_3		(0x458)
#define X2_ISP_REG_HMP_CCM_W_00			(0x45C)
#define X2_ISP_REG_HMP_CCM_W_01			(0x460)
#define X2_ISP_REG_HMP_CCM_W_02			(0x464)
#define X2_ISP_REG_HMP_CCM_W_10			(0x468)
#define X2_ISP_REG_HMP_CCM_W_11			(0x46C)
#define X2_ISP_REG_HMP_CCM_W_12			(0x470)
#define X2_ISP_REG_HMP_CCM_W_20			(0x474)
#define X2_ISP_REG_HMP_CCM_W_21			(0x478)
#define X2_ISP_REG_HMP_CCM_W_22			(0x47C)
#define X2_ISP_REG_HMP_CCM_B_0			(0x480)
#define X2_ISP_REG_HMP_CCM_B_1			(0x484)
#define X2_ISP_REG_HMP_CCM_B_2			(0x488)
#define X2_ISP_REG_HMP_CB2_W_0			(0x48C)
#define X2_ISP_REG_HMP_CB2_W_1			(0x490)
#define X2_ISP_REG_HMP_CB2_W_2			(0x494)
#define X2_ISP_REG_HMP_CB2_B_0			(0x498)
#define X2_ISP_REG_HMP_CB2_B_1			(0x49C)
#define X2_ISP_REG_HMP_CB2_B_2			(0x4A0)
#define X2_ISP_REG_HMP_YCC1_W_00		(0x4A4)
#define X2_ISP_REG_HMP_YCC1_W_01		(0x4A8)
#define X2_ISP_REG_HMP_YCC1_W_02		(0x4AC)
#define X2_ISP_REG_HMP_YCC1_W_10		(0x4B0)
#define X2_ISP_REG_HMP_YCC1_W_11		(0x4B4)
#define X2_ISP_REG_HMP_YCC1_W_12		(0x4B8)
#define X2_ISP_REG_HMP_YCC1_W_20		(0x4BC)
#define X2_ISP_REG_HMP_YCC1_W_21		(0x4C0)
#define X2_ISP_REG_HMP_YCC1_W_22		(0x4C4)
#define X2_ISP_REG_HMP_YCC1_B_0			(0x4C8)
#define X2_ISP_REG_HMP_YCC1_B_1			(0x4CC)
#define X2_ISP_REG_HMP_YCC1_B_2			(0x4D0)
#define X2_ISP_REG_HMP_CDR_OFST_H		(0x4D4)
#define X2_ISP_REG_HMP_CDR_OFST_V		(0x4D8)
#define X2_ISP_REG_HMP_CDR_SIZE_H		(0x4DC)
#define X2_ISP_REG_HMP_CDR_SIZE_V		(0x4E0)
#define X2_ISP_REG_HMP_CDR_CONT_H		(0x4E4)
#define X2_ISP_REG_HMP_CDR_CONT_V		(0x4E8)
#define X2_ISP_REG_HMP_CDR_BINS			(0x4EC)
#define X2_ISP_REG_HMP_CDR_BINS_S0		(0x4F0)
#define X2_ISP_REG_HMP_CDR_BINS_S1		(0x4F4)
#define X2_ISP_REG_HMP_CDR_CLIP_S		(0x4F8)
#define X2_ISP_REG_HMP_CDR_CLIP_L		(0x4FC)
#define X2_ISP_REG_HMP_CDR_CLIP_H		(0x500)
#define X2_ISP_REG_HMP_CDR_ALPHA_0		(0x504)
#define X2_ISP_REG_HMP_CDR_ALPHA_1		(0x508)
#define X2_ISP_REG_HMP_CDR_ALPHA_2		(0x50C)
#define X2_ISP_REG_HMP_CDR_NORM_S		(0x510)
#define X2_ISP_REG_HMP_CDR_NORM_DIV		(0x514)
#define X2_ISP_REG_HMP_CDR_Y_C_ON		(0x518)
#define X2_ISP_REG_HMP_CDR_BYPASS_LINES	(0x51C)
#define X2_ISP_REG_HMP_YCC2_W_00		(0x600)
#define X2_ISP_REG_HMP_YCC2_W_01		(0x604)
#define X2_ISP_REG_HMP_YCC2_W_02		(0x608)
#define X2_ISP_REG_HMP_YCC2_W_10		(0x60C)
#define X2_ISP_REG_HMP_YCC2_W_11		(0x610)
#define X2_ISP_REG_HMP_YCC2_W_12		(0x614)
#define X2_ISP_REG_HMP_YCC2_W_20		(0x618)
#define X2_ISP_REG_HMP_YCC2_W_21		(0x61C)
#define X2_ISP_REG_HMP_YCC2_W_22		(0x620)
#define X2_ISP_REG_HMP_YCC2_B_0			(0x624)
#define X2_ISP_REG_HMP_YCC2_B_1			(0x628)
#define X2_ISP_REG_HMP_YCC2_B_2			(0x62C)
#define X2_ISP_REG_STF_SADP_LPF_W_0		(0x630)
#define X2_ISP_REG_STF_SADP_LPF_W_1		(0x634)
#define X2_ISP_REG_STF_SADP_LPF_W_2		(0x638)
#define X2_ISP_REG_STF_SADP_LPF_W_3		(0x63C)
#define X2_ISP_REG_STF_SADP_LPF_W_4		(0x640)
#define X2_ISP_REG_STF_SADP_ROI_OFST_H	(0x644)
#define X2_ISP_REG_STF_SADP_ROI_OFST_V	(0x648)
#define X2_ISP_REG_STF_SADP_ROI_STDE_H	(0x64C)
#define X2_ISP_REG_STF_SADP_ROI_STDE_V	(0x650)
#define X2_ISP_REG_STF_SADP_ROI_SCNT_H	(0x654)
#define X2_ISP_REG_STF_SADP_ROI_SCNT_V	(0x658)
#define X2_ISP_REG_STF_IPP_SCB_W_0		(0x65C)
#define X2_ISP_REG_STF_IPP_SCB_W_1		(0x660)
#define X2_ISP_REG_STF_IPP_SCB_W_2		(0x664)
#define X2_ISP_REG_STF_IPP_SCB_B_0		(0x668)
#define X2_ISP_REG_STF_IPP_SCB_B_1		(0x66C)
#define X2_ISP_REG_STF_IPP_SCB_B_2		(0x670)
#define X2_ISP_REG_STF_SGMA_PRE_W_0		(0x674)
#define X2_ISP_REG_STF_SGMA_PRE_W_1		(0x678)
#define X2_ISP_REG_STF_SGMA_PRE_W_2		(0x67C)
#define X2_ISP_REG_STF_SGMA_PRE_B_0		(0x680)
#define X2_ISP_REG_STF_SGMA_PRE_B_1		(0x684)
#define X2_ISP_REG_STF_SGMA_PRE_B_2		(0x688)
#define X2_ISP_REG_STF_SGMA_LUT_W_0		(0x68C)
#define X2_ISP_REG_STF_SGMA_LUT_W_1		(0x690)
#define X2_ISP_REG_STF_SGMA_LUT_W_2		(0x694)
#define X2_ISP_REG_STF_SGMA_LUT_W_3		(0x698)
#define X2_ISP_REG_STF_SGMA_LUT_W_4		(0x69C)
#define X2_ISP_REG_STF_SGMA_LUT_W_5		(0x6A0)
#define X2_ISP_REG_STF_SGMA_LUT_W_6		(0x6A4)
#define X2_ISP_REG_STF_SGMA_LUT_W_7		(0x6A8)
#define X2_ISP_REG_STF_SGMA_LUT_W_8		(0x6AC)
#define X2_ISP_REG_STF_SGMA_LUT_W_9		(0x6B0)
#define X2_ISP_REG_STF_SGMA_LUT_W_10	(0x6B4)
#define X2_ISP_REG_STF_SGMA_LUT_W_11	(0x6B8)
#define X2_ISP_REG_STF_SGMA_LUT_W_12	(0x6BC)
#define X2_ISP_REG_STF_SGMA_LUT_W_13	(0x6C0)
#define X2_ISP_REG_STF_SGMA_LUT_W_14	(0x6C4)
#define X2_ISP_REG_STF_SGMA_LUT_W_15	(0x6C8)
#define X2_ISP_REG_STF_SGMA_LUT_W_16	(0x6CC)
#define X2_ISP_REG_STF_SGMA_LUT_W_17	(0x6D0)
#define X2_ISP_REG_STF_SGMA_LUT_W_18	(0x6D4)
#define X2_ISP_REG_STF_SGMA_LUT_W_19	(0x6D8)
#define X2_ISP_REG_STF_SGMA_LUT_W_20	(0x6DC)
#define X2_ISP_REG_STF_SGMA_LUT_W_21	(0x6E0)
#define X2_ISP_REG_STF_SGMA_LUT_W_22	(0x6E4)
#define X2_ISP_REG_STF_SGMA_LUT_W_23	(0x6E8)
#define X2_ISP_REG_STF_SGMA_LUT_W_24	(0x6EC)
#define X2_ISP_REG_STF_SGMA_LUT_W_25	(0x6F0)
#define X2_ISP_REG_STF_SGMA_LUT_W_26	(0x6F4)
#define X2_ISP_REG_STF_SGMA_LUT_W_27	(0x6F8)
#define X2_ISP_REG_STF_SGMA_LUT_W_28	(0x6FC)
#define X2_ISP_REG_STF_SGMA_LUT_W_29	(0x700)
#define X2_ISP_REG_STF_SGMA_LUT_W_30	(0x704)
#define X2_ISP_REG_STF_SGMA_POST_W_0	(0x708)
#define X2_ISP_REG_STF_SGMA_POST_W_1	(0x70C)
#define X2_ISP_REG_STF_SGMA_POST_W_2	(0x710)
#define X2_ISP_REG_STF_SGMA_POST_B_0	(0x714)
#define X2_ISP_REG_STF_SGMA_POST_B_1	(0x718)
#define X2_ISP_REG_STF_SGMA_POST_B_2	(0x71C)
#define X2_ISP_REG_STF_IPP_SCCM_W_00	(0x720)
#define X2_ISP_REG_STF_IPP_SCCM_W_01	(0x724)
#define X2_ISP_REG_STF_IPP_SCCM_W_02	(0x728)
#define X2_ISP_REG_STF_IPP_SCCM_W_10	(0x72C)
#define X2_ISP_REG_STF_IPP_SCCM_W_11	(0x730)
#define X2_ISP_REG_STF_IPP_SCCM_W_12	(0x734)
#define X2_ISP_REG_STF_IPP_SCCM_W_20	(0x738)
#define X2_ISP_REG_STF_IPP_SCCM_W_21	(0x73C)
#define X2_ISP_REG_STF_IPP_SCCM_W_22	(0x740)
#define X2_ISP_REG_STF_IPP_SCCM_B_0		(0x744)
#define X2_ISP_REG_STF_IPP_SCCM_B_1		(0x748)
#define X2_ISP_REG_STF_IPP_SCCM_B_2		(0x74C)
#define X2_ISP_REG_STF_IPP_SYCC_W_0		(0x750)
#define X2_ISP_REG_STF_IPP_SYCC_W_1		(0x754)
#define X2_ISP_REG_STF_IPP_SYCC_W_2		(0x758)
#define X2_ISP_REG_STF_IPP_SYCC_B		(0x75C)
#define X2_ISP_REG_STF_SFE_GRID_OFST_H	(0x760)
#define X2_ISP_REG_STF_SFE_GRID_OFST_V	(0x764)
#define X2_ISP_REG_STF_SFE_GRID_SIZE_H	(0x768)
#define X2_ISP_REG_STF_SFE_GRID_SIZE_V	(0x76C)
#define X2_ISP_REG_STF_SFE_GRID_CONT_H	(0x770)
#define X2_ISP_REG_STF_SFE_GRID_CONT_V	(0x774)
#define X2_ISP_REG_STF_SFE_GRID_THR_H	(0x778)
#define X2_ISP_REG_STF_SFE_GRID_THR_L	(0x77C)
#define X2_ISP_REG_STF_SFE_GRID_S		(0x780)
#define X2_ISP_REG_STF_SFE_GRID_L		(0x784)
#define X2_ISP_REG_STF_SFE_GRID_H		(0x788)
#define X2_ISP_REG_STF_SFE_TILE_OFST_H	(0x78C)
#define X2_ISP_REG_STF_SFE_TILE_OFST_V	(0x790)
#define X2_ISP_REG_STF_SFE_TILE_SIZE_H	(0x794)
#define X2_ISP_REG_STF_SFE_TILE_SIZE_V	(0x798)
#define X2_ISP_REG_STF_SFE_TILE_CONT_H	(0x79C)
#define X2_ISP_REG_STF_SFE_TILE_CONT_V	(0x7A0)
#define X2_ISP_REG_STF_SFE_TILE_BINS	(0x7A4)
#define X2_ISP_REG_STF_SFE_TILE_S		(0x7A8)
#define X2_ISP_REG_STF_SFE_TILE_L		(0x7AC)
#define X2_ISP_REG_STF_SFE_TILE_H		(0x7B0)
#define X2_ISP_REG_STF_SFE_HIST_OFST_H	(0x7B4)
#define X2_ISP_REG_STF_SFE_HIST_OFST_V	(0x7B8)
#define X2_ISP_REG_STF_SFE_HIST_SIZE_H	(0x7BC)
#define X2_ISP_REG_STF_SFE_HIST_SIZE_V	(0x7C0)
#define X2_ISP_REG_STF_SFE_HIST_BINS	(0x7C4)
#define X2_ISP_REG_STF_SFE_HIST_S		(0x7C8)
#define X2_ISP_REG_STF_SFE_HIST_L		(0x7CC)
#define X2_ISP_REG_STF_SFE_HIST_H		(0x7D0)
#define X2_ISP_REG_STF_SFE_RSUM_OFST_H	(0x7D4)
#define X2_ISP_REG_STF_SFE_RSUM_OFST_V	(0x7D8)
#define X2_ISP_REG_STF_SFE_RSUM_SIZE_H	(0x7DC)
#define X2_ISP_REG_STF_SFE_RSUM_CONT_V	(0x7E0)
#define X2_ISP_REG_STF_SFE_RSUM_S		(0x7E4)
#define X2_ISP_REG_STF_SFE_RSUM_L		(0x7E8)
#define X2_ISP_REG_STF_SFE_RSUM_H		(0x7EC)
#define X2_ISP_REG_STF_SFE_TILE_ADDR	(0x7F0)
#define X2_ISP_REG_STF_SFE_GRID_ADDR	(0x7F4)
#define X2_ISP_REG_STF_SFE_RSUM_ADDR	(0x7F8)
#define X2_ISP_REG_STF_SFE_HIST_ADDR	(0x7FC)
#define X2_ISP_REG_HMP_CDR_ADDR0		(0x800)
#define X2_ISP_REG_HMP_CDR_ADDR1		(0x804)
#define X2_ISP_REG_SFE_MUX				(0x808)
#define X2_ISP_REG_IPP_CLIP_H			(0x80C)
#define X2_ISP_REG_IPP_CLIP_L			(0x810)
#define X2_ISP_REG_IPP_CLIP_S			(0x814)
#define X2_ISP_REG_STF_IPP_SYCC1_W_0	(0x818)
#define X2_ISP_REG_STF_IPP_SYCC1_W_1	(0x81C)
#define X2_ISP_REG_STF_IPP_SYCC1_W_2	(0x820)
#define X2_ISP_REG_STF_IPP_SYCC1_B		(0x824)
#define X2_ISP_REG_HMP_GMA_LUT_CLIP_L	(0x828)
#define X2_ISP_REG_HMP_GMA_LUT_CLIP_H	(0x82C)
#define X2_ISP_REG_HMP_CB1_CLIP_L		(0x830)
#define X2_ISP_REG_HMP_CB1_CLIP_H		(0x834)
#define X2_ISP_REG_HMP_CCM_CLIP			(0x838)
#define X2_ISP_REG_HMP_CB2_CLIP			(0x83C)
#define X2_ISP_REG_HMP_YCC1_CLIP		(0x840)
#define X2_ISP_REG_STF_IPP_SGMA_CLIP	(0x844)
#define X2_ISP_REG_STF_IPP_SCCM_CLIP	(0x848)
#define X2_ISP_REG_STF_IPP_SYCC0_CLIP	(0x84C)
#define X2_ISP_REG_STF_IPP_SYCC1_CLIP	(0x850)
#define X2_ISP_REG_ISP_CONFIG_DONE		(0x854)


/*************************************************************
 * register bit
*************************************************************/

/*    X2_ISP_ISP_CONFIG    */
#define   X2_ISP_ISP_EN(n)                (((n) & 0x1) << 0x1f)
#define   X2_ISP_ISP_EN_MASK                (0x1 << 0x1f)
#define   X2_ISP_ISP_EN_SHIT(n)                (((n) & 0x1) >> 0x1f)
#define   X2_ISP_HMP_GMA_LUT_BYPASS(n)                (((n) & 0x1) << 0x1a)
#define   X2_ISP_HMP_GMA_LUT_BYPASS_MASK                (0x1 << 0x1a)
#define   X2_ISP_HMP_GMA_LUT_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x1a)
#define   X2_ISP_STF_SGMA_LUT_BYPASS(n)                (((n) & 0x1) << 0x19)
#define   X2_ISP_STF_SGMA_LUT_BYPASS_MASK                (0x1 << 0x19)
#define   X2_ISP_STF_SGMA_LUT_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x19)
#define   X2_ISP_STF_IPP_SYCC1_BYPASS(n)                (((n) & 0x1) << 0x18)
#define   X2_ISP_STF_IPP_SYCC1_BYPASS_MASK                (0x1 << 0x18)
#define   X2_ISP_STF_IPP_SYCC1_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x18)
#define   X2_ISP_HMP_CDR_BYPASS(n)                (((n) & 0x1) << 0x17)
#define   X2_ISP_HMP_CDR_BYPASS_MASK                (0x1 << 0x17)
#define   X2_ISP_HMP_CDR_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x17)
#define   X2_ISP_STF_IPP_SYCC_BYPASS(n)                (((n) & 0x1) << 0x16)
#define   X2_ISP_STF_IPP_SYCC_BYPASS_MASK                (0x1 << 0x16)
#define   X2_ISP_STF_IPP_SYCC_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x16)
#define   X2_ISP_STF_IPP_SCCM_BYPASS(n)                (((n) & 0x1) << 0x15)
#define   X2_ISP_STF_IPP_SCCM_BYPASS_MASK                (0x1 << 0x15)
#define   X2_ISP_STF_IPP_SCCM_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x15)
#define   X2_ISP_STF_IPP_SGMA_POST_BYPASS(n)                (((n) & 0x1) << 0x14)
#define   X2_ISP_STF_IPP_SGMA_POST_BYPASS_MASK                (0x1 << 0x14)
#define   X2_ISP_STF_IPP_SGMA_POST_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x14)
#define   X2_ISP_STF_IPP_SGMA_PRE_BYPASS(n)                (((n) & 0x1) << 0x13)
#define   X2_ISP_STF_IPP_SGMA_PRE_BYPASS_MASK                (0x1 << 0x13)
#define   X2_ISP_STF_IPP_SGMA_PRE_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x13)
#define   X2_ISP_STF_IPP_SGMA_BYPASS(n)                (((n) & 0x1) << 0x12)
#define   X2_ISP_STF_IPP_SGMA_BYPASS_MASK                (0x1 << 0x12)
#define   X2_ISP_STF_IPP_SGMA_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x12)
#define   X2_ISP_STF_IPP_SCB_BYPASS(n)                (((n) & 0x1) << 0x11)
#define   X2_ISP_STF_IPP_SCB_BYPASS_MASK                (0x1 << 0x11)
#define   X2_ISP_STF_IPP_SCB_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x11)
#define   X2_ISP_STF_SADP_LPF_BYPASS(n)                (((n) & 0x1) << 0x10)
#define   X2_ISP_STF_SADP_LPF_BYPASS_MASK                (0x1 << 0x10)
#define   X2_ISP_STF_SADP_LPF_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x10)
#define   X2_ISP_HMP_GMA_POST_BYPASS(n)                (((n) & 0x1) << 0xf)
#define   X2_ISP_HMP_GMA_POST_BYPASS_MASK                (0x1 << 0xf)
#define   X2_ISP_HMP_GMA_POST_BYPASS_SHIT(n)                (((n) & 0x1) >> 0xf)
#define   X2_ISP_HMP_GMA_PRE_BYPASS(n)                (((n) & 0x1) << 0xe)
#define   X2_ISP_HMP_GMA_PRE_BYPASS_MASK                (0x1 << 0xe)
#define   X2_ISP_HMP_GMA_PRE_BYPASS_SHIT(n)                (((n) & 0x1) >> 0xe)
#define   X2_ISP_HMP_GMA_BYPASS(n)                (((n) & 0x1) << 0xd)
#define   X2_ISP_HMP_GMA_BYPASS_MASK                (0x1 << 0xd)
#define   X2_ISP_HMP_GMA_BYPASS_SHIT(n)                (((n) & 0x1) >> 0xd)
#define   X2_ISP_HMP_YCC2_BYPASS(n)                (((n) & 0x1) << 0xc)
#define   X2_ISP_HMP_YCC2_BYPASS_MASK                (0x1 << 0xc)
#define   X2_ISP_HMP_YCC2_BYPASS_SHIT(n)                (((n) & 0x1) >> 0xc)
#define   X2_ISP_HMP_YCC1_BYPASS(n)                (((n) & 0x1) << 0xb)
#define   X2_ISP_HMP_YCC1_BYPASS_MASK                (0x1 << 0xb)
#define   X2_ISP_HMP_YCC1_BYPASS_SHIT(n)                (((n) & 0x1) >> 0xb)
#define   X2_ISP_HMP_CB2_BYPASS(n)                (((n) & 0x1) << 0xa)
#define   X2_ISP_HMP_CB2_BYPASS_MASK                (0x1 << 0xa)
#define   X2_ISP_HMP_CB2_BYPASS_SHIT(n)                (((n) & 0x1) >> 0xa)
#define   X2_ISP_HMP_CCM_BYPASS(n)                (((n) & 0x1) << 0x9)
#define   X2_ISP_HMP_CCM_BYPASS_MASK                (0x1 << 0x9)
#define   X2_ISP_HMP_CCM_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x9)
#define   X2_ISP_HMP_CB1_BYPASS(n)                (((n) & 0x1) << 0x8)
#define   X2_ISP_HMP_CB1_BYPASS_MASK                (0x1 << 0x8)
#define   X2_ISP_HMP_CB1_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x8)
#define   X2_ISP_HMP_WBG_BYPASS(n)                (((n) & 0x1) << 0x7)
#define   X2_ISP_HMP_WBG_BYPASS_MASK                (0x1 << 0x7)
#define   X2_ISP_HMP_WBG_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x7)
#define   X2_ISP_HMP_DEG_BYPASS(n)                (((n) & 0x1) << 0x6)
#define   X2_ISP_HMP_DEG_BYPASS_MASK                (0x1 << 0x6)
#define   X2_ISP_HMP_DEG_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x6)
#define   X2_ISP_DPP_AMP_BYPASS(n)                (((n) & 0x1) << 0x5)
#define   X2_ISP_DPP_AMP_BYPASS_MASK                (0x1 << 0x5)
#define   X2_ISP_DPP_AMP_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x5)
#define   X2_ISP_DPP_BYPASS(n)                (((n) & 0x1) << 0x4)
#define   X2_ISP_DPP_BYPASS_MASK                (0x1 << 0x4)
#define   X2_ISP_DPP_BYPASS_SHIT(n)                (((n) & 0x1) >> 0x4)
#define   X2_ISP_IMG_FORMAT(n)                (((n) & 0xf) << 0x0)
#define   X2_ISP_IMG_FORMAT_MASK                (0xf << 0x0)
#define   X2_ISP_IMG_FORMAT_SHIT(n)                (((n) & 0xf) >> 0x0)

/*    X2_ISP_isp_video_in_size    */
#define   X2_ISP_ISP_VIDEO_IN_WIDTH(n)                (((n) & 0x1fff) << 0x10)
#define   X2_ISP_ISP_VIDEO_IN_WIDTH_MASK                (0x1fff << 0x10)
#define   X2_ISP_ISP_VIDEO_IN_WIDTH_SHIT(n)                (((n) & 0x1fff) >> 0x10)
#define   X2_ISP_ISP_VIDEO_IN_HEIGHT(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_ISP_VIDEO_IN_HEIGHT_MASK                (0x1fff << 0x0)
#define   X2_ISP_ISP_VIDEO_IN_HEIGHT_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_DPP_BL_B_0    */
#define   X2_ISP_DPP_BL_B(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_BL_B_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_BL_B_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_BL_B_1    */
#define   X2_ISP_DPP_BL_B_1(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_BL_B_1_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_BL_B_1_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_BL_B_2    */
#define   X2_ISP_DPP_BL_B_2(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_BL_B_2_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_BL_B_2_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_BL_B_3    */
#define   X2_ISP_DPP_BL_B_3(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_BL_B_3_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_BL_B_3_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_DS_S    */
#define   X2_ISP_DPP_DS_S(n)                (((n) & 0xf) << 0x14)
#define   X2_ISP_DPP_DS_S_MASK                (0xf << 0x14)
#define   X2_ISP_DPP_DS_S_SHIT(n)                (((n) & 0xf) >> 0x14)
#define   X2_ISP_DPP_DS_S_6(n)                (((n) & 0xf) << 0x10)
#define   X2_ISP_DPP_DS_S_6_MASK                (0xf << 0x10)
#define   X2_ISP_DPP_DS_S_6_SHIT(n)                (((n) & 0xf) >> 0x10)
#define   X2_ISP_DPP_DS_S_5(n)                (((n) & 0xf) << 0xc)
#define   X2_ISP_DPP_DS_S_5_MASK                (0xf << 0xc)
#define   X2_ISP_DPP_DS_S_5_SHIT(n)                (((n) & 0xf) >> 0xc)
#define   X2_ISP_DPP_DS_S_4(n)                (((n) & 0xf) << 0x8)
#define   X2_ISP_DPP_DS_S_4_MASK                (0xf << 0x8)
#define   X2_ISP_DPP_DS_S_4_SHIT(n)                (((n) & 0xf) >> 0x8)
#define   X2_ISP_DPP_DS_S_3(n)                (((n) & 0x3) << 0x6)
#define   X2_ISP_DPP_DS_S_3_MASK                (0x3 << 0x6)
#define   X2_ISP_DPP_DS_S_3_SHIT(n)                (((n) & 0x3) >> 0x6)
#define   X2_ISP_DPP_DS_S_2(n)                (((n) & 0x3) << 0x4)
#define   X2_ISP_DPP_DS_S_2_MASK                (0x3 << 0x4)
#define   X2_ISP_DPP_DS_S_2_SHIT(n)                (((n) & 0x3) >> 0x4)
#define   X2_ISP_DPP_DS_S_1(n)                (((n) & 0x3) << 0x2)
#define   X2_ISP_DPP_DS_S_1_MASK                (0x3 << 0x2)
#define   X2_ISP_DPP_DS_S_1_SHIT(n)                (((n) & 0x3) >> 0x2)
#define   X2_ISP_DPP_DS_S_0(n)                (((n) & 0x3) << 0x0)
#define   X2_ISP_DPP_DS_S_0_MASK                (0x3 << 0x0)
#define   X2_ISP_DPP_DS_S_0_SHIT(n)                (((n) & 0x3) >> 0x0)

/*    X2_ISP_DPP_AMP_W_0    */
#define   X2_ISP_DPP_AMP_W(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_DPP_AMP_W_MASK                (0xfff << 0x0)
#define   X2_ISP_DPP_AMP_W_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_DPP_AMP_W_1    */
#define   X2_ISP_DPP_AMP_W_1(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_DPP_AMP_W_1_MASK                (0xfff << 0x0)
#define   X2_ISP_DPP_AMP_W_1_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_DPP_AMP_W_2    */
#define   X2_ISP_DPP_AMP_W_2(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_DPP_AMP_W_2_MASK                (0xfff << 0x0)
#define   X2_ISP_DPP_AMP_W_2_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_DPP_AMP_W_3    */
#define   X2_ISP_DPP_AMP_W_3(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_DPP_AMP_W_3_MASK                (0xfff << 0x0)
#define   X2_ISP_DPP_AMP_W_3_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_DPP_AMP_B_0    */
#define   X2_ISP_DPP_AMP_B(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_DPP_AMP_B_MASK                (0x1fffff << 0x0)
#define   X2_ISP_DPP_AMP_B_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_DPP_AMP_B_1    */
#define   X2_ISP_DPP_AMP_B_1(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_DPP_AMP_B_1_MASK                (0x1fffff << 0x0)
#define   X2_ISP_DPP_AMP_B_1_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_DPP_AMP_B_2    */
#define   X2_ISP_DPP_AMP_B_2(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_DPP_AMP_B_2_MASK                (0x1fffff << 0x0)
#define   X2_ISP_DPP_AMP_B_2_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_DPP_AMP_B_3    */
#define   X2_ISP_DPP_AMP_B_3(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_DPP_AMP_B_3_MASK                (0x1fffff << 0x0)
#define   X2_ISP_DPP_AMP_B_3_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_DPP_LMT_L_0    */
#define   X2_ISP_DPP_LMT_L(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_LMT_L_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_LMT_L_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_LMT_L_1    */
#define   X2_ISP_DPP_LMT_L_1(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_LMT_L_1_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_LMT_L_1_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_LMT_L_2    */
#define   X2_ISP_DPP_LMT_L_2(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_LMT_L_2_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_LMT_L_2_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_LMT_L_3    */
#define   X2_ISP_DPP_LMT_L_3(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_LMT_L_3_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_LMT_L_3_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_LMT_H_0    */
#define   X2_ISP_DPP_LMT_H(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_LMT_H_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_LMT_H_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_LMT_H_1    */
#define   X2_ISP_DPP_LMT_H_1(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_LMT_H_1_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_LMT_H_1_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_LMT_H_2    */
#define   X2_ISP_DPP_LMT_H_2(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_LMT_H_2_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_LMT_H_2_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_DPP_LMT_H_3    */
#define   X2_ISP_DPP_LMT_H_3(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_DPP_LMT_H_3_MASK                (0xfffff << 0x0)
#define   X2_ISP_DPP_LMT_H_3_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_HMP_DEG_W    */
#define   X2_ISP_HMP_DEG_W(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_HMP_DEG_W_MASK                (0x1ffff << 0x0)
#define   X2_ISP_HMP_DEG_W_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_HMP_DEG_B    */
#define   X2_ISP_HMP_DEG_B(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_DEG_B_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_DEG_B_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_WBG_W_0    */
#define   X2_ISP_HMP_WBG_W(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_HMP_WBG_W_MASK                (0x1ffff << 0x0)
#define   X2_ISP_HMP_WBG_W_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_HMP_WBG_W_1    */
#define   X2_ISP_HMP_WBG_W_1(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_HMP_WBG_W_1_MASK                (0x1ffff << 0x0)
#define   X2_ISP_HMP_WBG_W_1_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_HMP_WBG_W_2    */
#define   X2_ISP_HMP_WBG_W_2(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_HMP_WBG_W_2_MASK                (0x1ffff << 0x0)
#define   X2_ISP_HMP_WBG_W_2_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_HMP_WBG_W_3    */
#define   X2_ISP_HMP_WBG_W_3(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_HMP_WBG_W_3_MASK                (0x1ffff << 0x0)
#define   X2_ISP_HMP_WBG_W_3_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_HMP_WBG_B_0    */
#define   X2_ISP_HMP_WBG_B(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_WBG_B_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_WBG_B_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_WBG_B_1    */
#define   X2_ISP_HMP_WBG_B_1(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_WBG_B_1_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_WBG_B_1_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_WBG_B_2    */
#define   X2_ISP_HMP_WBG_B_2(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_WBG_B_2_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_WBG_B_2_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_WBG_B_3    */
#define   X2_ISP_HMP_WBG_B_3(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_WBG_B_3_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_WBG_B_3_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_CB1_W_0    */
#define   X2_ISP_HMP_CB1_W(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_HMP_CB1_W_MASK                (0x1ffff << 0x0)
#define   X2_ISP_HMP_CB1_W_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_HMP_CB1_W_1    */
#define   X2_ISP_HMP_CB1_W_1(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_HMP_CB1_W_1_MASK                (0x1ffff << 0x0)
#define   X2_ISP_HMP_CB1_W_1_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_HMP_CB1_W_2    */
#define   X2_ISP_HMP_CB1_W_2(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_HMP_CB1_W_2_MASK                (0x1ffff << 0x0)
#define   X2_ISP_HMP_CB1_W_2_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_HMP_CB1_W_3    */
#define   X2_ISP_HMP_CB1_W_3(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_HMP_CB1_W_3_MASK                (0x1ffff << 0x0)
#define   X2_ISP_HMP_CB1_W_3_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_HMP_CB1_B_0    */
#define   X2_ISP_HMP_CB1_B(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_CB1_B_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_CB1_B_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_CB1_B_1    */
#define   X2_ISP_HMP_CB1_B_1(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_CB1_B_1_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_CB1_B_1_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_CB1_B_2    */
#define   X2_ISP_HMP_CB1_B_2(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_CB1_B_2_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_CB1_B_2_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_CB1_B_3    */
#define   X2_ISP_HMP_CB1_B_3(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_CB1_B_3_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_CB1_B_3_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_GMA_PRE_W_0    */
#define   X2_ISP_HMP_GMA_PRE_W(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_PRE_W_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_PRE_W_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_PRE_W_1    */
#define   X2_ISP_HMP_GMA_PRE_W_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_PRE_W_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_PRE_W_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_PRE_W_2    */
#define   X2_ISP_HMP_GMA_PRE_W_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_PRE_W_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_PRE_W_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_PRE_W_3    */
#define   X2_ISP_HMP_GMA_PRE_W_3(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_PRE_W_3_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_PRE_W_3_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_PRE_B_0    */
#define   X2_ISP_HMP_GMA_PRE_B(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_GMA_PRE_B_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_GMA_PRE_B_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_GMA_PRE_B_1    */
#define   X2_ISP_HMP_GMA_PRE_B_1(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_GMA_PRE_B_1_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_GMA_PRE_B_1_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_GMA_PRE_B_2    */
#define   X2_ISP_HMP_GMA_PRE_B_2(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_GMA_PRE_B_2_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_GMA_PRE_B_2_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_GMA_PRE_B_3    */
#define   X2_ISP_HMP_GMA_PRE_B_3(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_HMP_GMA_PRE_B_3_MASK                (0x1fffff << 0x0)
#define   X2_ISP_HMP_GMA_PRE_B_3_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT_SEL    */
#define   X2_ISP_HMP_GMA_R3_LUT_SEL(n)                (((n) & 0x3) << 0x6)
#define   X2_ISP_HMP_GMA_R3_LUT_SEL_MASK                (0x3 << 0x6)
#define   X2_ISP_HMP_GMA_R3_LUT_SEL_SHIT(n)                (((n) & 0x3) >> 0x6)
#define   X2_ISP_HMP_GMA_G2_LUT_SEL(n)                (((n) & 0x3) << 0x4)
#define   X2_ISP_HMP_GMA_G2_LUT_SEL_MASK                (0x3 << 0x4)
#define   X2_ISP_HMP_GMA_G2_LUT_SEL_SHIT(n)                (((n) & 0x3) >> 0x4)
#define   X2_ISP_HMP_GMA_G1_LUT_SEL(n)                (((n) & 0x3) << 0x2)
#define   X2_ISP_HMP_GMA_G1_LUT_SEL_MASK                (0x3 << 0x2)
#define   X2_ISP_HMP_GMA_G1_LUT_SEL_SHIT(n)                (((n) & 0x3) >> 0x2)
#define   X2_ISP_HMP_GMA_B0_LUT_SEL(n)                (((n) & 0x3) << 0x0)
#define   X2_ISP_HMP_GMA_B0_LUT_SEL_MASK                (0x3 << 0x0)
#define   X2_ISP_HMP_GMA_B0_LUT_SEL_SHIT(n)                (((n) & 0x3) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_0    */
#define   X2_ISP_HMP_GMA_LUT1_W(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_1    */
#define   X2_ISP_HMP_GMA_LUT1_W_1(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_1_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_1_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_2    */
#define   X2_ISP_HMP_GMA_LUT1_W_2(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_2_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_2_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_3    */
#define   X2_ISP_HMP_GMA_LUT1_W_3(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_3_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_3_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_4    */
#define   X2_ISP_HMP_GMA_LUT1_W_4(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_4_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_4_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_5    */
#define   X2_ISP_HMP_GMA_LUT1_W_5(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_5_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_5_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_6    */
#define   X2_ISP_HMP_GMA_LUT1_W_6(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_6_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_6_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_7    */
#define   X2_ISP_HMP_GMA_LUT1_W_7(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_7_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_7_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_8    */
#define   X2_ISP_HMP_GMA_LUT1_W_8(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_8_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_8_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_9    */
#define   X2_ISP_HMP_GMA_LUT1_W_9(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_9_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_9_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_10    */
#define   X2_ISP_HMP_GMA_LUT1_W_10(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_10_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_10_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_11    */
#define   X2_ISP_HMP_GMA_LUT1_W_11(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_11_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_11_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_12    */
#define   X2_ISP_HMP_GMA_LUT1_W_12(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_12_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_12_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_13    */
#define   X2_ISP_HMP_GMA_LUT1_W_13(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_13_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_13_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_14    */
#define   X2_ISP_HMP_GMA_LUT1_W_14(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_14_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_14_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_15    */
#define   X2_ISP_HMP_GMA_LUT1_W_15(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_15_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_15_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_16    */
#define   X2_ISP_HMP_GMA_LUT1_W_16(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_16_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_16_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_17    */
#define   X2_ISP_HMP_GMA_LUT1_W_17(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_17_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_17_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_18    */
#define   X2_ISP_HMP_GMA_LUT1_W_18(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_18_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_18_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_19    */
#define   X2_ISP_HMP_GMA_LUT1_W_19(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_19_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_19_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_20    */
#define   X2_ISP_HMP_GMA_LUT1_W_20(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_20_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_20_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_21    */
#define   X2_ISP_HMP_GMA_LUT1_W_21(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_21_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_21_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_22    */
#define   X2_ISP_HMP_GMA_LUT1_W_22(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_22_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_22_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_23    */
#define   X2_ISP_HMP_GMA_LUT1_W_23(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_23_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_23_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_24    */
#define   X2_ISP_HMP_GMA_LUT1_W_24(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_24_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_24_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_25    */
#define   X2_ISP_HMP_GMA_LUT1_W_25(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_25_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_25_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_26    */
#define   X2_ISP_HMP_GMA_LUT1_W_26(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_26_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_26_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_27    */
#define   X2_ISP_HMP_GMA_LUT1_W_27(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_27_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_27_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_28    */
#define   X2_ISP_HMP_GMA_LUT1_W_28(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_28_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_28_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_29    */
#define   X2_ISP_HMP_GMA_LUT1_W_29(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_29_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_29_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT1_W_30    */
#define   X2_ISP_HMP_GMA_LUT1_W_30(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_30_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT1_W_30_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_0    */
#define   X2_ISP_HMP_GMA_LUT2_W(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_1    */
#define   X2_ISP_HMP_GMA_LUT2_W_1(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_1_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_1_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_2    */
#define   X2_ISP_HMP_GMA_LUT2_W_2(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_2_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_2_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_3    */
#define   X2_ISP_HMP_GMA_LUT2_W_3(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_3_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_3_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_4    */
#define   X2_ISP_HMP_GMA_LUT2_W_4(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_4_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_4_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_5    */
#define   X2_ISP_HMP_GMA_LUT2_W_5(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_5_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_5_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_6    */
#define   X2_ISP_HMP_GMA_LUT2_W_6(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_6_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_6_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_7    */
#define   X2_ISP_HMP_GMA_LUT2_W_7(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_7_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_7_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_8    */
#define   X2_ISP_HMP_GMA_LUT2_W_8(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_8_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_8_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_9    */
#define   X2_ISP_HMP_GMA_LUT2_W_9(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_9_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_9_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_10    */
#define   X2_ISP_HMP_GMA_LUT2_W_10(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_10_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_10_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_11    */
#define   X2_ISP_HMP_GMA_LUT2_W_11(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_11_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_11_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_12    */
#define   X2_ISP_HMP_GMA_LUT2_W_12(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_12_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_12_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_13    */
#define   X2_ISP_HMP_GMA_LUT2_W_13(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_13_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_13_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_14    */
#define   X2_ISP_HMP_GMA_LUT2_W_14(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_14_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_14_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_15    */
#define   X2_ISP_HMP_GMA_LUT2_W_15(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_15_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_15_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_16    */
#define   X2_ISP_HMP_GMA_LUT2_W_16(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_16_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_16_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_17    */
#define   X2_ISP_HMP_GMA_LUT2_W_17(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_17_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_17_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_18    */
#define   X2_ISP_HMP_GMA_LUT2_W_18(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_18_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_18_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_19    */
#define   X2_ISP_HMP_GMA_LUT2_W_19(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_19_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_19_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_20    */
#define   X2_ISP_HMP_GMA_LUT2_W_20(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_20_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_20_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_21    */
#define   X2_ISP_HMP_GMA_LUT2_W_21(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_21_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_21_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_22    */
#define   X2_ISP_HMP_GMA_LUT2_W_22(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_22_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_22_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_23    */
#define   X2_ISP_HMP_GMA_LUT2_W_23(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_23_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_23_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_24    */
#define   X2_ISP_HMP_GMA_LUT2_W_24(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_24_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_24_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_25    */
#define   X2_ISP_HMP_GMA_LUT2_W_25(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_25_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_25_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_26    */
#define   X2_ISP_HMP_GMA_LUT2_W_26(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_26_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_26_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_27    */
#define   X2_ISP_HMP_GMA_LUT2_W_27(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_27_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_27_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_28    */
#define   X2_ISP_HMP_GMA_LUT2_W_28(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_28_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_28_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_29    */
#define   X2_ISP_HMP_GMA_LUT2_W_29(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_29_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_29_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT2_W_30    */
#define   X2_ISP_HMP_GMA_LUT2_W_30(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_30_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT2_W_30_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_0    */
#define   X2_ISP_HMP_GMA_LUT3_W(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_1    */
#define   X2_ISP_HMP_GMA_LUT3_W_1(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_1_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_1_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_2    */
#define   X2_ISP_HMP_GMA_LUT3_W_2(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_2_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_2_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_3    */
#define   X2_ISP_HMP_GMA_LUT3_W_3(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_3_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_3_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_4    */
#define   X2_ISP_HMP_GMA_LUT3_W_4(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_4_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_4_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_5    */
#define   X2_ISP_HMP_GMA_LUT3_W_5(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_5_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_5_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_6    */
#define   X2_ISP_HMP_GMA_LUT3_W_6(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_6_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_6_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_7    */
#define   X2_ISP_HMP_GMA_LUT3_W_7(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_7_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_7_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_8    */
#define   X2_ISP_HMP_GMA_LUT3_W_8(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_8_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_8_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_9    */
#define   X2_ISP_HMP_GMA_LUT3_W_9(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_9_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_9_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_10    */
#define   X2_ISP_HMP_GMA_LUT3_W_10(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_10_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_10_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_11    */
#define   X2_ISP_HMP_GMA_LUT3_W_11(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_11_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_11_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_12    */
#define   X2_ISP_HMP_GMA_LUT3_W_12(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_12_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_12_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_13    */
#define   X2_ISP_HMP_GMA_LUT3_W_13(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_13_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_13_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_14    */
#define   X2_ISP_HMP_GMA_LUT3_W_14(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_14_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_14_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_15    */
#define   X2_ISP_HMP_GMA_LUT3_W_15(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_15_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_15_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_16    */
#define   X2_ISP_HMP_GMA_LUT3_W_16(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_16_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_16_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_17    */
#define   X2_ISP_HMP_GMA_LUT3_W_17(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_17_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_17_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_18    */
#define   X2_ISP_HMP_GMA_LUT3_W_18(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_18_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_18_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_19    */
#define   X2_ISP_HMP_GMA_LUT3_W_19(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_19_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_19_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_20    */
#define   X2_ISP_HMP_GMA_LUT3_W_20(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_20_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_20_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_21    */
#define   X2_ISP_HMP_GMA_LUT3_W_21(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_21_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_21_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_22    */
#define   X2_ISP_HMP_GMA_LUT3_W_22(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_22_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_22_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_23    */
#define   X2_ISP_HMP_GMA_LUT3_W_23(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_23_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_23_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_24    */
#define   X2_ISP_HMP_GMA_LUT3_W_24(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_24_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_24_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_25    */
#define   X2_ISP_HMP_GMA_LUT3_W_25(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_25_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_25_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_26    */
#define   X2_ISP_HMP_GMA_LUT3_W_26(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_26_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_26_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_27    */
#define   X2_ISP_HMP_GMA_LUT3_W_27(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_27_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_27_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_28    */
#define   X2_ISP_HMP_GMA_LUT3_W_28(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_28_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_28_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_29    */
#define   X2_ISP_HMP_GMA_LUT3_W_29(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_29_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_29_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT3_W_30    */
#define   X2_ISP_HMP_GMA_LUT3_W_30(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_30_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_GMA_LUT3_W_30_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_GMA_POST_W_0    */
#define   X2_ISP_HMP_GMA_POST_W(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_POST_W_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_POST_W_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_POST_W_1    */
#define   X2_ISP_HMP_GMA_POST_W_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_POST_W_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_POST_W_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_POST_W_2    */
#define   X2_ISP_HMP_GMA_POST_W_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_POST_W_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_POST_W_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_POST_W_3    */
#define   X2_ISP_HMP_GMA_POST_W_3(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_POST_W_3_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_POST_W_3_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_POST_B_0    */
#define   X2_ISP_HMP_GMA_POST_B(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_POST_B_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_POST_B_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_POST_B_1    */
#define   X2_ISP_HMP_GMA_POST_B_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_POST_B_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_POST_B_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_POST_B_2    */
#define   X2_ISP_HMP_GMA_POST_B_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_POST_B_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_POST_B_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_POST_B_3    */
#define   X2_ISP_HMP_GMA_POST_B_3(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_GMA_POST_B_3_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_GMA_POST_B_3_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_CCM_W_00    */
#define   X2_ISP_HMP_CCM_W(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_CCM_W_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_CCM_W_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_CCM_W_01    */
#define   X2_ISP_HMP_CCM_W_01(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_CCM_W_01_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_CCM_W_01_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_CCM_W_02    */
#define   X2_ISP_HMP_CCM_W_02(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_CCM_W_02_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_CCM_W_02_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_CCM_W_10    */
#define   X2_ISP_HMP_CCM_W_10(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_CCM_W_10_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_CCM_W_10_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_CCM_W_11    */
#define   X2_ISP_HMP_CCM_W_11(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_CCM_W_11_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_CCM_W_11_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_CCM_W_12    */
#define   X2_ISP_HMP_CCM_W_12(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_CCM_W_12_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_CCM_W_12_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_CCM_W_20    */
#define   X2_ISP_HMP_CCM_W_20(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_CCM_W_20_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_CCM_W_20_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_CCM_W_21    */
#define   X2_ISP_HMP_CCM_W_21(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_CCM_W_21_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_CCM_W_21_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_CCM_W_22    */
#define   X2_ISP_HMP_CCM_W_22(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_CCM_W_22_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_CCM_W_22_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_CCM_B_0    */
#define   X2_ISP_HMP_CCM_B(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_CCM_B_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_CCM_B_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_CCM_B_1    */
#define   X2_ISP_HMP_CCM_B_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_CCM_B_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_CCM_B_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_CCM_B_2    */
#define   X2_ISP_HMP_CCM_B_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_CCM_B_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_CCM_B_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_CB2_W_0    */
#define   X2_ISP_HMP_CB2_W(n)                (((n) & 0x7fff) << 0x0)
#define   X2_ISP_HMP_CB2_W_MASK                (0x7fff << 0x0)
#define   X2_ISP_HMP_CB2_W_SHIT(n)                (((n) & 0x7fff) >> 0x0)

/*    X2_ISP_HMP_CB2_W_1    */
#define   X2_ISP_HMP_CB2_W_1(n)                (((n) & 0x7fff) << 0x0)
#define   X2_ISP_HMP_CB2_W_1_MASK                (0x7fff << 0x0)
#define   X2_ISP_HMP_CB2_W_1_SHIT(n)                (((n) & 0x7fff) >> 0x0)

/*    X2_ISP_HMP_CB2_W_2    */
#define   X2_ISP_HMP_CB2_W_2(n)                (((n) & 0x7fff) << 0x0)
#define   X2_ISP_HMP_CB2_W_2_MASK                (0x7fff << 0x0)
#define   X2_ISP_HMP_CB2_W_2_SHIT(n)                (((n) & 0x7fff) >> 0x0)

/*    X2_ISP_HMP_CB2_B_0    */
#define   X2_ISP_HMP_CB2_B(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_CB2_B_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_CB2_B_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_CB2_B_1    */
#define   X2_ISP_HMP_CB2_B_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_CB2_B_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_CB2_B_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_CB2_B_2    */
#define   X2_ISP_HMP_CB2_B_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_CB2_B_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_CB2_B_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_YCC1_W_00    */
#define   X2_ISP_HMP_YCC1_W(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC1_W_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC1_W_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC1_W_01    */
#define   X2_ISP_HMP_YCC1_W_01(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC1_W_01_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC1_W_01_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC1_W_02    */
#define   X2_ISP_HMP_YCC1_W_02(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC1_W_02_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC1_W_02_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC1_W_10    */
#define   X2_ISP_HMP_YCC1_W_10(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC1_W_10_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC1_W_10_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC1_W_11    */
#define   X2_ISP_HMP_YCC1_W_11(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC1_W_11_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC1_W_11_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC1_W_12    */
#define   X2_ISP_HMP_YCC1_W_12(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC1_W_12_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC1_W_12_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC1_W_20    */
#define   X2_ISP_HMP_YCC1_W_20(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC1_W_20_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC1_W_20_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC1_W_21    */
#define   X2_ISP_HMP_YCC1_W_21(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC1_W_21_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC1_W_21_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC1_W_22    */
#define   X2_ISP_HMP_YCC1_W_22(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC1_W_22_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC1_W_22_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC1_B_0    */
#define   X2_ISP_HMP_YCC1_B(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_YCC1_B_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_YCC1_B_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_YCC1_B_1    */
#define   X2_ISP_HMP_YCC1_B_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_YCC1_B_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_YCC1_B_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_YCC1_B_2    */
#define   X2_ISP_HMP_YCC1_B_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_YCC1_B_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_YCC1_B_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_CDR_OFST_H    */
#define   X2_ISP_HMP_CDR_OFST_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_CDR_OFST_H_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_CDR_OFST_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_CDR_OFST_V    */
#define   X2_ISP_HMP_CDR_OFST_V(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_CDR_OFST_V_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_CDR_OFST_V_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_CDR_SIZE_H    */
#define   X2_ISP_HMP_CDR_SIZE_H(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_CDR_SIZE_H_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_CDR_SIZE_H_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_CDR_SIZE_V    */
#define   X2_ISP_HMP_CDR_SIZE_V(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_HMP_CDR_SIZE_V_MASK                (0x1fff << 0x0)
#define   X2_ISP_HMP_CDR_SIZE_V_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_CDR_CONT_H    */
#define   X2_ISP_HMP_CDR_CONT_H(n)                (((n) & 0xff) << 0x0)
#define   X2_ISP_HMP_CDR_CONT_H_MASK                (0xff << 0x0)
#define   X2_ISP_HMP_CDR_CONT_H_SHIT(n)                (((n) & 0xff) >> 0x0)

/*    X2_ISP_HMP_CDR_CONT_V    */
#define   X2_ISP_HMP_CDR_CONT_V(n)                (((n) & 0xff) << 0x0)
#define   X2_ISP_HMP_CDR_CONT_V_MASK                (0xff << 0x0)
#define   X2_ISP_HMP_CDR_CONT_V_SHIT(n)                (((n) & 0xff) >> 0x0)

/*    X2_ISP_HMP_CDR_BINS    */
#define   X2_ISP_HMP_CDR_BINS(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_HMP_CDR_BINS_MASK                (0x7ff << 0x0)
#define   X2_ISP_HMP_CDR_BINS_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_HMP_CDR_BINS_S0    */
#define   X2_ISP_HMP_CDR_BINS_S(n)                (((n) & 0x1f) << 0x0)
#define   X2_ISP_HMP_CDR_BINS_S_MASK                (0x1f << 0x0)
#define   X2_ISP_HMP_CDR_BINS_S_SHIT(n)                (((n) & 0x1f) >> 0x0)

/*    X2_ISP_HMP_CDR_BINS_S1    */
#define   X2_ISP_HMP_CDR_BINS_S1(n)                (((n) & 0x1f) << 0x0)
#define   X2_ISP_HMP_CDR_BINS_S1_MASK                (0x1f << 0x0)
#define   X2_ISP_HMP_CDR_BINS_S1_SHIT(n)                (((n) & 0x1f) >> 0x0)

/*    X2_ISP_HMP_CDR_CLIP_S    */
#define   X2_ISP_HMP_CDR_CLIP_S(n)                (((n) & 0x3f) << 0x0)
#define   X2_ISP_HMP_CDR_CLIP_S_MASK                (0x3f << 0x0)
#define   X2_ISP_HMP_CDR_CLIP_S_SHIT(n)                (((n) & 0x3f) >> 0x0)

/*    X2_ISP_HMP_CDR_CLIP_L    */
#define   X2_ISP_HMP_CDR_CLIP_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_CDR_CLIP_L_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_CDR_CLIP_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_CDR_CLIP_H    */
#define   X2_ISP_HMP_CDR_CLIP_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_CDR_CLIP_H_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_CDR_CLIP_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_CDR_ALPHA_0    */
#define   X2_ISP_HMP_CDR_ALPHA(n)                (((n) & 0x1ff) << 0x0)
#define   X2_ISP_HMP_CDR_ALPHA_MASK                (0x1ff << 0x0)
#define   X2_ISP_HMP_CDR_ALPHA_SHIT(n)                (((n) & 0x1ff) >> 0x0)

/*    X2_ISP_HMP_CDR_ALPHA_1    */
#define   X2_ISP_HMP_CDR_ALPHA_1(n)                (((n) & 0x1ff) << 0x0)
#define   X2_ISP_HMP_CDR_ALPHA_1_MASK                (0x1ff << 0x0)
#define   X2_ISP_HMP_CDR_ALPHA_1_SHIT(n)                (((n) & 0x1ff) >> 0x0)

/*    X2_ISP_HMP_CDR_ALPHA_2    */
#define   X2_ISP_HMP_CDR_ALPHA_2(n)                (((n) & 0x1ff) << 0x0)
#define   X2_ISP_HMP_CDR_ALPHA_2_MASK                (0x1ff << 0x0)
#define   X2_ISP_HMP_CDR_ALPHA_2_SHIT(n)                (((n) & 0x1ff) >> 0x0)

/*    X2_ISP_HMP_CDR_NORM_S    */
#define   X2_ISP_HMP_CDR_NORM_S(n)                (((n) & 0x3f) << 0x0)
#define   X2_ISP_HMP_CDR_NORM_S_MASK                (0x3f << 0x0)
#define   X2_ISP_HMP_CDR_NORM_S_SHIT(n)                (((n) & 0x3f) >> 0x0)

/*    X2_ISP_HMP_CDR_NORM_DIV    */
#define   X2_ISP_HMP_CDR_NORM_DIV(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_CDR_NORM_DIV_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_CDR_NORM_DIV_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_CDR_Y_C_ON    */
#define   X2_ISP_HMP_CDR_C_ON(n)                (((n) & 0x1) << 0x1)
#define   X2_ISP_HMP_CDR_C_ON_MASK                (0x1 << 0x1)
#define   X2_ISP_HMP_CDR_C_ON_SHIT(n)                (((n) & 0x1) >> 0x1)
#define   X2_ISP_HMP_CDR_Y_ON(n)                (((n) & 0x1) << 0x0)
#define   X2_ISP_HMP_CDR_Y_ON_MASK                (0x1 << 0x0)
#define   X2_ISP_HMP_CDR_Y_ON_SHIT(n)                (((n) & 0x1) >> 0x0)

/*    X2_ISP_HMP_CDR_BYPASS_LINES    */
#define   X2_ISP_HMP_CDR_BYPASS_LINES(n)                (((n) & 0xf) << 0x0)
#define   X2_ISP_HMP_CDR_BYPASS_LINES_MASK                (0xf << 0x0)
#define   X2_ISP_HMP_CDR_BYPASS_LINES_SHIT(n)                (((n) & 0xf) >> 0x0)

/*    X2_ISP_HMP_YCC2_W_00    */
#define   X2_ISP_HMP_YCC2_W(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC2_W_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC2_W_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC2_W_01    */
#define   X2_ISP_HMP_YCC2_W_01(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC2_W_01_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC2_W_01_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC2_W_02    */
#define   X2_ISP_HMP_YCC2_W_02(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC2_W_02_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC2_W_02_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC2_W_10    */
#define   X2_ISP_HMP_YCC2_W_10(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC2_W_10_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC2_W_10_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC2_W_11    */
#define   X2_ISP_HMP_YCC2_W_11(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC2_W_11_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC2_W_11_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC2_W_12    */
#define   X2_ISP_HMP_YCC2_W_12(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC2_W_12_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC2_W_12_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC2_W_20    */
#define   X2_ISP_HMP_YCC2_W_20(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC2_W_20_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC2_W_20_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC2_W_21    */
#define   X2_ISP_HMP_YCC2_W_21(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC2_W_21_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC2_W_21_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC2_W_22    */
#define   X2_ISP_HMP_YCC2_W_22(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_HMP_YCC2_W_22_MASK                (0xffff << 0x0)
#define   X2_ISP_HMP_YCC2_W_22_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_HMP_YCC2_B_0    */
#define   X2_ISP_HMP_YCC2_B(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_HMP_YCC2_B_MASK                (0x7ff << 0x0)
#define   X2_ISP_HMP_YCC2_B_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_HMP_YCC2_B_1    */
#define   X2_ISP_HMP_YCC2_B_1(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_HMP_YCC2_B_1_MASK                (0x7ff << 0x0)
#define   X2_ISP_HMP_YCC2_B_1_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_HMP_YCC2_B_2    */
#define   X2_ISP_HMP_YCC2_B_2(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_HMP_YCC2_B_2_MASK                (0x7ff << 0x0)
#define   X2_ISP_HMP_YCC2_B_2_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_STF_SADP_LPF_W_0    */
#define   X2_ISP_STF_SADP_LPF_W(n)                (((n) & 0x3ff) << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_MASK                (0x3ff << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_SHIT(n)                (((n) & 0x3ff) >> 0x0)

/*    X2_ISP_STF_SADP_LPF_W_1    */
#define   X2_ISP_STF_SADP_LPF_W_1(n)                (((n) & 0x3ff) << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_1_MASK                (0x3ff << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_1_SHIT(n)                (((n) & 0x3ff) >> 0x0)

/*    X2_ISP_STF_SADP_LPF_W_2    */
#define   X2_ISP_STF_SADP_LPF_W_2(n)                (((n) & 0x3ff) << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_2_MASK                (0x3ff << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_2_SHIT(n)                (((n) & 0x3ff) >> 0x0)

/*    X2_ISP_STF_SADP_LPF_W_3    */
#define   X2_ISP_STF_SADP_LPF_W_3(n)                (((n) & 0x3ff) << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_3_MASK                (0x3ff << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_3_SHIT(n)                (((n) & 0x3ff) >> 0x0)

/*    X2_ISP_STF_SADP_LPF_W_4    */
#define   X2_ISP_STF_SADP_LPF_W_4(n)                (((n) & 0x3ff) << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_4_MASK                (0x3ff << 0x0)
#define   X2_ISP_STF_SADP_LPF_W_4_SHIT(n)                (((n) & 0x3ff) >> 0x0)

/*    X2_ISP_STF_SADP_ROI_OFST_H    */
#define   X2_ISP_STF_SADP_ROI_OFST_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SADP_ROI_OFST_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SADP_ROI_OFST_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SADP_ROI_OFST_V    */
#define   X2_ISP_STF_SADP_ROI_OFST_V(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SADP_ROI_OFST_V_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SADP_ROI_OFST_V_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SADP_ROI_STDE_H    */
#define   X2_ISP_STF_SADP_ROI_STDE_H(n)                (((n) & 0xff) << 0x0)
#define   X2_ISP_STF_SADP_ROI_STDE_H_MASK                (0xff << 0x0)
#define   X2_ISP_STF_SADP_ROI_STDE_H_SHIT(n)                (((n) & 0xff) >> 0x0)

/*    X2_ISP_STF_SADP_ROI_STDE_V    */
#define   X2_ISP_STF_SADP_ROI_STDE_V(n)                (((n) & 0xff) << 0x0)
#define   X2_ISP_STF_SADP_ROI_STDE_V_MASK                (0xff << 0x0)
#define   X2_ISP_STF_SADP_ROI_STDE_V_SHIT(n)                (((n) & 0xff) >> 0x0)

/*    X2_ISP_STF_SADP_ROI_SCNT_H    */
#define   X2_ISP_STF_SADP_ROI_SCNT_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SADP_ROI_SCNT_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SADP_ROI_SCNT_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SADP_ROI_SCNT_V    */
#define   X2_ISP_STF_SADP_ROI_SCNT_V(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SADP_ROI_SCNT_V_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SADP_ROI_SCNT_V_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_IPP_SCB_W_0    */
#define   X2_ISP_STF_IPP_SCB_W(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_STF_IPP_SCB_W_MASK                (0x1ffff << 0x0)
#define   X2_ISP_STF_IPP_SCB_W_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCB_W_1    */
#define   X2_ISP_STF_IPP_SCB_W_1(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_STF_IPP_SCB_W_1_MASK                (0x1ffff << 0x0)
#define   X2_ISP_STF_IPP_SCB_W_1_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCB_W_2    */
#define   X2_ISP_STF_IPP_SCB_W_2(n)                (((n) & 0x1ffff) << 0x0)
#define   X2_ISP_STF_IPP_SCB_W_2_MASK                (0x1ffff << 0x0)
#define   X2_ISP_STF_IPP_SCB_W_2_SHIT(n)                (((n) & 0x1ffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCB_B_0    */
#define   X2_ISP_STF_IPP_SCB_B(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_STF_IPP_SCB_B_MASK                (0x1fffff << 0x0)
#define   X2_ISP_STF_IPP_SCB_B_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCB_B_1    */
#define   X2_ISP_STF_IPP_SCB_B_1(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_STF_IPP_SCB_B_1_MASK                (0x1fffff << 0x0)
#define   X2_ISP_STF_IPP_SCB_B_1_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCB_B_2    */
#define   X2_ISP_STF_IPP_SCB_B_2(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_STF_IPP_SCB_B_2_MASK                (0x1fffff << 0x0)
#define   X2_ISP_STF_IPP_SCB_B_2_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_STF_SGMA_PRE_W_0    */
#define   X2_ISP_STF_SGMA_PRE_W(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_SGMA_PRE_W_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_SGMA_PRE_W_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_SGMA_PRE_W_1    */
#define   X2_ISP_STF_SGMA_PRE_W_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_SGMA_PRE_W_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_SGMA_PRE_W_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_SGMA_PRE_W_2    */
#define   X2_ISP_STF_SGMA_PRE_W_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_SGMA_PRE_W_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_SGMA_PRE_W_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_SGMA_PRE_B_0    */
#define   X2_ISP_STF_SGMA_PRE_B(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_STF_SGMA_PRE_B_MASK                (0x1fffff << 0x0)
#define   X2_ISP_STF_SGMA_PRE_B_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_STF_SGMA_PRE_B_1    */
#define   X2_ISP_STF_SGMA_PRE_B_1(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_STF_SGMA_PRE_B_1_MASK                (0x1fffff << 0x0)
#define   X2_ISP_STF_SGMA_PRE_B_1_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_STF_SGMA_PRE_B_2    */
#define   X2_ISP_STF_SGMA_PRE_B_2(n)                (((n) & 0x1fffff) << 0x0)
#define   X2_ISP_STF_SGMA_PRE_B_2_MASK                (0x1fffff << 0x0)
#define   X2_ISP_STF_SGMA_PRE_B_2_SHIT(n)                (((n) & 0x1fffff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_0    */
#define   X2_ISP_STF_SGMA_LUT_W(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_1    */
#define   X2_ISP_STF_SGMA_LUT_W_1(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_1_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_1_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_2    */
#define   X2_ISP_STF_SGMA_LUT_W_2(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_2_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_2_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_3    */
#define   X2_ISP_STF_SGMA_LUT_W_3(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_3_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_3_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_4    */
#define   X2_ISP_STF_SGMA_LUT_W_4(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_4_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_4_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_5    */
#define   X2_ISP_STF_SGMA_LUT_W_5(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_5_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_5_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_6    */
#define   X2_ISP_STF_SGMA_LUT_W_6(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_6_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_6_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_7    */
#define   X2_ISP_STF_SGMA_LUT_W_7(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_7_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_7_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_8    */
#define   X2_ISP_STF_SGMA_LUT_W_8(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_8_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_8_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_9    */
#define   X2_ISP_STF_SGMA_LUT_W_9(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_9_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_9_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_10    */
#define   X2_ISP_STF_SGMA_LUT_W_10(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_10_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_10_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_11    */
#define   X2_ISP_STF_SGMA_LUT_W_11(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_11_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_11_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_12    */
#define   X2_ISP_STF_SGMA_LUT_W_12(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_12_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_12_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_13    */
#define   X2_ISP_STF_SGMA_LUT_W_13(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_13_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_13_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_14    */
#define   X2_ISP_STF_SGMA_LUT_W_14(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_14_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_14_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_15    */
#define   X2_ISP_STF_SGMA_LUT_W_15(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_15_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_15_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_16    */
#define   X2_ISP_STF_SGMA_LUT_W_16(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_16_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_16_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_17    */
#define   X2_ISP_STF_SGMA_LUT_W_17(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_17_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_17_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_18    */
#define   X2_ISP_STF_SGMA_LUT_W_18(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_18_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_18_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_19    */
#define   X2_ISP_STF_SGMA_LUT_W_19(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_19_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_19_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_20    */
#define   X2_ISP_STF_SGMA_LUT_W_20(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_20_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_20_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_21    */
#define   X2_ISP_STF_SGMA_LUT_W_21(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_21_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_21_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_22    */
#define   X2_ISP_STF_SGMA_LUT_W_22(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_22_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_22_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_23    */
#define   X2_ISP_STF_SGMA_LUT_W_23(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_23_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_23_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_24    */
#define   X2_ISP_STF_SGMA_LUT_W_24(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_24_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_24_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_25    */
#define   X2_ISP_STF_SGMA_LUT_W_25(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_25_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_25_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_26    */
#define   X2_ISP_STF_SGMA_LUT_W_26(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_26_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_26_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_27    */
#define   X2_ISP_STF_SGMA_LUT_W_27(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_27_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_27_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_28    */
#define   X2_ISP_STF_SGMA_LUT_W_28(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_28_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_28_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_29    */
#define   X2_ISP_STF_SGMA_LUT_W_29(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_29_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_29_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_LUT_W_30    */
#define   X2_ISP_STF_SGMA_LUT_W_30(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_30_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SGMA_LUT_W_30_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SGMA_POST_W_0    */
#define   X2_ISP_STF_SGMA_POST_W(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_SGMA_POST_W_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_SGMA_POST_W_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_SGMA_POST_W_1    */
#define   X2_ISP_STF_SGMA_POST_W_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_SGMA_POST_W_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_SGMA_POST_W_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_SGMA_POST_W_2    */
#define   X2_ISP_STF_SGMA_POST_W_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_SGMA_POST_W_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_SGMA_POST_W_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_SGMA_POST_B_0    */
#define   X2_ISP_STF_SGMA_POST_B(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_SGMA_POST_B_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_SGMA_POST_B_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_SGMA_POST_B_1    */
#define   X2_ISP_STF_SGMA_POST_B_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_SGMA_POST_B_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_SGMA_POST_B_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_SGMA_POST_B_2    */
#define   X2_ISP_STF_SGMA_POST_B_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_SGMA_POST_B_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_SGMA_POST_B_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_W_00    */
#define   X2_ISP_STF_IPP_SCCM_W(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_W_01    */
#define   X2_ISP_STF_IPP_SCCM_W_01(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_01_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_01_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_W_02    */
#define   X2_ISP_STF_IPP_SCCM_W_02(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_02_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_02_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_W_10    */
#define   X2_ISP_STF_IPP_SCCM_W_10(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_10_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_10_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_W_11    */
#define   X2_ISP_STF_IPP_SCCM_W_11(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_11_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_11_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_W_12    */
#define   X2_ISP_STF_IPP_SCCM_W_12(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_12_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_12_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_W_20    */
#define   X2_ISP_STF_IPP_SCCM_W_20(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_20_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_20_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_W_21    */
#define   X2_ISP_STF_IPP_SCCM_W_21(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_21_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_21_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_W_22    */
#define   X2_ISP_STF_IPP_SCCM_W_22(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_22_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_W_22_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_B_0    */
#define   X2_ISP_STF_IPP_SCCM_B(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_B_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_B_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_B_1    */
#define   X2_ISP_STF_IPP_SCCM_B_1(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_B_1_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_B_1_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_B_2    */
#define   X2_ISP_STF_IPP_SCCM_B_2(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_B_2_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_B_2_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC_W_0    */
#define   X2_ISP_STF_IPP_SYCC_W(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC_W_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SYCC_W_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC_W_1    */
#define   X2_ISP_STF_IPP_SYCC_W_1(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC_W_1_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SYCC_W_1_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC_W_2    */
#define   X2_ISP_STF_IPP_SYCC_W_2(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC_W_2_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SYCC_W_2_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC_B    */
#define   X2_ISP_STF_IPP_SYCC_B(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC_B_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_IPP_SYCC_B_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_OFST_H    */
#define   X2_ISP_STF_SFE_GRID_OFST_H(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_OFST_H_MASK                (0x7ff << 0x0)
#define   X2_ISP_STF_SFE_GRID_OFST_H_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_OFST_V    */
#define   X2_ISP_STF_SFE_GRID_OFST_V(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_OFST_V_MASK                (0x7ff << 0x0)
#define   X2_ISP_STF_SFE_GRID_OFST_V_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_SIZE_H    */
#define   X2_ISP_STF_SFE_GRID_SIZE_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_SIZE_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_GRID_SIZE_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_SIZE_V    */
#define   X2_ISP_STF_SFE_GRID_SIZE_V(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_SIZE_V_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_GRID_SIZE_V_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_CONT_H    */
#define   X2_ISP_STF_SFE_GRID_CONT_H(n)                (((n) & 0xff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_CONT_H_MASK                (0xff << 0x0)
#define   X2_ISP_STF_SFE_GRID_CONT_H_SHIT(n)                (((n) & 0xff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_CONT_V    */
#define   X2_ISP_STF_SFE_GRID_CONT_V(n)                (((n) & 0xff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_CONT_V_MASK                (0xff << 0x0)
#define   X2_ISP_STF_SFE_GRID_CONT_V_SHIT(n)                (((n) & 0xff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_THR_H    */
#define   X2_ISP_STF_SFE_GRID_THR_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_THR_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_GRID_THR_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_THR_L    */
#define   X2_ISP_STF_SFE_GRID_THR_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_THR_L_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_GRID_THR_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_S    */
#define   X2_ISP_STF_SFE_GRID_S(n)                (((n) & 0x1f) << 0x0)
#define   X2_ISP_STF_SFE_GRID_S_MASK                (0x1f << 0x0)
#define   X2_ISP_STF_SFE_GRID_S_SHIT(n)                (((n) & 0x1f) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_L    */
#define   X2_ISP_STF_SFE_GRID_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_L_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_GRID_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_H    */
#define   X2_ISP_STF_SFE_GRID_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_GRID_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_OFST_H    */
#define   X2_ISP_STF_SFE_TILE_OFST_H(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_OFST_H_MASK                (0x7ff << 0x0)
#define   X2_ISP_STF_SFE_TILE_OFST_H_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_OFST_V    */
#define   X2_ISP_STF_SFE_TILE_OFST_V(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_OFST_V_MASK                (0x7ff << 0x0)
#define   X2_ISP_STF_SFE_TILE_OFST_V_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_SIZE_H    */
#define   X2_ISP_STF_SFE_TILE_SIZE_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_SIZE_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_TILE_SIZE_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_SIZE_V    */
#define   X2_ISP_STF_SFE_TILE_SIZE_V(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_SIZE_V_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_TILE_SIZE_V_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_CONT_H    */
#define   X2_ISP_STF_SFE_TILE_CONT_H(n)                (((n) & 0xff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_CONT_H_MASK                (0xff << 0x0)
#define   X2_ISP_STF_SFE_TILE_CONT_H_SHIT(n)                (((n) & 0xff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_CONT_V    */
#define   X2_ISP_STF_SFE_TILE_CONT_V(n)                (((n) & 0xff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_CONT_V_MASK                (0xff << 0x0)
#define   X2_ISP_STF_SFE_TILE_CONT_V_SHIT(n)                (((n) & 0xff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_BINS    */
#define   X2_ISP_STF_SFE_TILE_BINS(n)                (((n) & 0x3ff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_BINS_MASK                (0x3ff << 0x0)
#define   X2_ISP_STF_SFE_TILE_BINS_SHIT(n)                (((n) & 0x3ff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_S    */
#define   X2_ISP_STF_SFE_TILE_S(n)                (((n) & 0x1f) << 0x0)
#define   X2_ISP_STF_SFE_TILE_S_MASK                (0x1f << 0x0)
#define   X2_ISP_STF_SFE_TILE_S_SHIT(n)                (((n) & 0x1f) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_L    */
#define   X2_ISP_STF_SFE_TILE_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_L_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_TILE_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_H    */
#define   X2_ISP_STF_SFE_TILE_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_TILE_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_HIST_OFST_H    */
#define   X2_ISP_STF_SFE_HIST_OFST_H(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_STF_SFE_HIST_OFST_H_MASK                (0x7ff << 0x0)
#define   X2_ISP_STF_SFE_HIST_OFST_H_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_STF_SFE_HIST_OFST_V    */
#define   X2_ISP_STF_SFE_HIST_OFST_V(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_STF_SFE_HIST_OFST_V_MASK                (0x7ff << 0x0)
#define   X2_ISP_STF_SFE_HIST_OFST_V_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_STF_SFE_HIST_SIZE_H    */
#define   X2_ISP_STF_SFE_HIST_SIZE_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_HIST_SIZE_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_HIST_SIZE_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_HIST_SIZE_V    */
#define   X2_ISP_STF_SFE_HIST_SIZE_V(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_HIST_SIZE_V_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_HIST_SIZE_V_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_HIST_BINS    */
#define   X2_ISP_STF_SFE_HIST_BINS(n)                (((n) & 0x3ff) << 0x0)
#define   X2_ISP_STF_SFE_HIST_BINS_MASK                (0x3ff << 0x0)
#define   X2_ISP_STF_SFE_HIST_BINS_SHIT(n)                (((n) & 0x3ff) >> 0x0)

/*    X2_ISP_STF_SFE_HIST_S    */
#define   X2_ISP_STF_SFE_HIST_S(n)                (((n) & 0x1f) << 0x0)
#define   X2_ISP_STF_SFE_HIST_S_MASK                (0x1f << 0x0)
#define   X2_ISP_STF_SFE_HIST_S_SHIT(n)                (((n) & 0x1f) >> 0x0)

/*    X2_ISP_STF_SFE_HIST_L    */
#define   X2_ISP_STF_SFE_HIST_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_HIST_L_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_HIST_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_HIST_H    */
#define   X2_ISP_STF_SFE_HIST_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_HIST_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_HIST_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_RSUM_OFST_H    */
#define   X2_ISP_STF_SFE_RSUM_OFST_H(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_STF_SFE_RSUM_OFST_H_MASK                (0x7ff << 0x0)
#define   X2_ISP_STF_SFE_RSUM_OFST_H_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_STF_SFE_RSUM_OFST_V    */
#define   X2_ISP_STF_SFE_RSUM_OFST_V(n)                (((n) & 0x7ff) << 0x0)
#define   X2_ISP_STF_SFE_RSUM_OFST_V_MASK                (0x7ff << 0x0)
#define   X2_ISP_STF_SFE_RSUM_OFST_V_SHIT(n)                (((n) & 0x7ff) >> 0x0)

/*    X2_ISP_STF_SFE_RSUM_SIZE_H    */
#define   X2_ISP_STF_SFE_RSUM_SIZE_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_RSUM_SIZE_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_RSUM_SIZE_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_RSUM_CONT_V    */
#define   X2_ISP_STF_SFE_RSUM_CONT_V(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_RSUM_CONT_V_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_RSUM_CONT_V_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_RSUM_S    */
#define   X2_ISP_STF_SFE_RSUM_S(n)                (((n) & 0x1f) << 0x0)
#define   X2_ISP_STF_SFE_RSUM_S_MASK                (0x1f << 0x0)
#define   X2_ISP_STF_SFE_RSUM_S_SHIT(n)                (((n) & 0x1f) >> 0x0)

/*    X2_ISP_STF_SFE_RSUM_L    */
#define   X2_ISP_STF_SFE_RSUM_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_RSUM_L_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_RSUM_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_RSUM_H    */
#define   X2_ISP_STF_SFE_RSUM_H(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_SFE_RSUM_H_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_SFE_RSUM_H_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_SFE_TILE_ADDR    */
#define   X2_ISP_STF_SFE_TILE_ADDR(n)                (((n) & 0xffffffff) << 0x0)
#define   X2_ISP_STF_SFE_TILE_ADDR_MASK                (0xffffffff << 0x0)
#define   X2_ISP_STF_SFE_TILE_ADDR_SHIT(n)                (((n) & 0xffffffff) >> 0x0)

/*    X2_ISP_STF_SFE_GRID_ADDR    */
#define   X2_ISP_STF_SFE_GRID_ADDR(n)                (((n) & 0xffffffff) << 0x0)
#define   X2_ISP_STF_SFE_GRID_ADDR_MASK                (0xffffffff << 0x0)
#define   X2_ISP_STF_SFE_GRID_ADDR_SHIT(n)                (((n) & 0xffffffff) >> 0x0)

/*    X2_ISP_STF_SFE_RSUM_ADDR    */
#define   X2_ISP_STF_SFE_HIST_ADDR(n)                (((n) & 0xffffffff) << 0x0)
#define   X2_ISP_STF_SFE_HIST_ADDR_MASK                (0xffffffff << 0x0)
#define   X2_ISP_STF_SFE_HIST_ADDR_SHIT(n)                (((n) & 0xffffffff) >> 0x0)

/*    X2_ISP_STF_SFE_HIST_ADDR    */
#define   X2_ISP_STF_SFE_RSUM_ADDR(n)                (((n) & 0xffffffff) << 0x0)
#define   X2_ISP_STF_SFE_RSUM_ADDR_MASK                (0xffffffff << 0x0)
#define   X2_ISP_STF_SFE_RSUM_ADDR_SHIT(n)                (((n) & 0xffffffff) >> 0x0)

/*    X2_ISP_HMP_CDR_ADDR0    */
#define   X2_ISP_HMP_CDR_ADDR(n)                (((n) & 0xffffffff) << 0x0)
#define   X2_ISP_HMP_CDR_ADDR_MASK                (0xffffffff << 0x0)
#define   X2_ISP_HMP_CDR_ADDR_SHIT(n)                (((n) & 0xffffffff) >> 0x0)

/*    X2_ISP_HMP_CDR_ADDR1    */
#define   X2_ISP_HMP_CDR_ADDR1(n)                (((n) & 0xffffffff) << 0x0)
#define   X2_ISP_HMP_CDR_ADDR1_MASK                (0xffffffff << 0x0)
#define   X2_ISP_HMP_CDR_ADDR1_SHIT(n)                (((n) & 0xffffffff) >> 0x0)

/*    X2_ISP_SFE_MUX    */
#define   X2_ISP_TILE_SEL(n)                (((n) & 0x1) << 0x3)
#define   X2_ISP_TILE_SEL_MASK                (0x1 << 0x3)
#define   X2_ISP_TILE_SEL_SHIT(n)                (((n) & 0x1) >> 0x3)
#define   X2_ISP_GRID_SEL(n)                (((n) & 0x1) << 0x2)
#define   X2_ISP_GRID_SEL_MASK                (0x1 << 0x2)
#define   X2_ISP_GRID_SEL_SHIT(n)                (((n) & 0x1) >> 0x2)
#define   X2_ISP_HIST_SEL(n)                (((n) & 0x1) << 0x1)
#define   X2_ISP_HIST_SEL_MASK                (0x1 << 0x1)
#define   X2_ISP_HIST_SEL_SHIT(n)                (((n) & 0x1) >> 0x1)
#define   X2_ISP_RSUM_SEL(n)                (((n) & 0x1) << 0x0)
#define   X2_ISP_RSUM_SEL_MASK                (0x1 << 0x0)
#define   X2_ISP_RSUM_SEL_SHIT(n)                (((n) & 0x1) >> 0x0)

/*    X2_ISP_IPP_CLIP_H    */
#define   X2_ISP_IPP_CLIP_H(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_IPP_CLIP_H_MASK                (0xfffff << 0x0)
#define   X2_ISP_IPP_CLIP_H_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_IPP_CLIP_L    */
#define   X2_ISP_IPP_CLIP_L(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_IPP_CLIP_L_MASK                (0xfffff << 0x0)
#define   X2_ISP_IPP_CLIP_L_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_IPP_CLIP_S    */
#define   X2_ISP_IPP_CLIP_S(n)                (((n) & 0x1f) << 0x0)
#define   X2_ISP_IPP_CLIP_S_MASK                (0x1f << 0x0)
#define   X2_ISP_IPP_CLIP_S_SHIT(n)                (((n) & 0x1f) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC1_W_0    */
#define   X2_ISP_STF_IPP_SYCC1_W(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_W_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_W_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC1_W_1    */
#define   X2_ISP_STF_IPP_SYCC1_W_1(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_W_1_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_W_1_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC1_W_2    */
#define   X2_ISP_STF_IPP_SYCC1_W_2(n)                (((n) & 0xffff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_W_2_MASK                (0xffff << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_W_2_SHIT(n)                (((n) & 0xffff) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC1_B    */
#define   X2_ISP_STF_IPP_SYCC1_B(n)                (((n) & 0x1fff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_B_MASK                (0x1fff << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_B_SHIT(n)                (((n) & 0x1fff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT_CLIP_L    */
#define   X2_ISP_HMP_GMA_LUT_CLIP_L(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT_CLIP_L_MASK                (0xfffff << 0x0)
#define   X2_ISP_HMP_GMA_LUT_CLIP_L_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_HMP_GMA_LUT_CLIP_H    */
#define   X2_ISP_HMP_GMA_LUT_CLIP_H(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_HMP_GMA_LUT_CLIP_H_MASK                (0xfffff << 0x0)
#define   X2_ISP_HMP_GMA_LUT_CLIP_H_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_HMP_CB1_CLIP_L    */
#define   X2_ISP_HMP_CB1_CLIP_L(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_HMP_CB1_CLIP_L_MASK                (0xfffff << 0x0)
#define   X2_ISP_HMP_CB1_CLIP_L_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_HMP_CB1_CLIP_H    */
#define   X2_ISP_HMP_CB1_CLIP_H(n)                (((n) & 0xfffff) << 0x0)
#define   X2_ISP_HMP_CB1_CLIP_H_MASK                (0xfffff << 0x0)
#define   X2_ISP_HMP_CB1_CLIP_H_SHIT(n)                (((n) & 0xfffff) >> 0x0)

/*    X2_ISP_HMP_CCM_CLIP    */
#define   X2_ISP_HMP_CCM_CLIP_H(n)                (((n) & 0xfff) << 0x10)
#define   X2_ISP_HMP_CCM_CLIP_H_MASK                (0xfff << 0x10)
#define   X2_ISP_HMP_CCM_CLIP_H_SHIT(n)                (((n) & 0xfff) >> 0x10)
#define   X2_ISP_HMP_CCM_CLIP_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_CCM_CLIP_L_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_CCM_CLIP_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_CB2_CLIP    */
#define   X2_ISP_HMP_CB2_CLIP_H(n)                (((n) & 0xfff) << 0x10)
#define   X2_ISP_HMP_CB2_CLIP_H_MASK                (0xfff << 0x10)
#define   X2_ISP_HMP_CB2_CLIP_H_SHIT(n)                (((n) & 0xfff) >> 0x10)
#define   X2_ISP_HMP_CB2_CLIP_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_CB2_CLIP_L_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_CB2_CLIP_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_HMP_YCC1_CLIP    */
#define   X2_ISP_HMP_YCC1_CLIP_H(n)                (((n) & 0xfff) << 0x10)
#define   X2_ISP_HMP_YCC1_CLIP_H_MASK                (0xfff << 0x10)
#define   X2_ISP_HMP_YCC1_CLIP_H_SHIT(n)                (((n) & 0xfff) >> 0x10)
#define   X2_ISP_HMP_YCC1_CLIP_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_HMP_YCC1_CLIP_L_MASK                (0xfff << 0x0)
#define   X2_ISP_HMP_YCC1_CLIP_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_IPP_SGMA_CLIP    */
#define   X2_ISP_STF_IPP_SGMA_CLIP_H(n)                (((n) & 0xfff) << 0x10)
#define   X2_ISP_STF_IPP_SGMA_CLIP_H_MASK                (0xfff << 0x10)
#define   X2_ISP_STF_IPP_SGMA_CLIP_H_SHIT(n)                (((n) & 0xfff) >> 0x10)
#define   X2_ISP_STF_IPP_SGMA_CLIP_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_IPP_SGMA_CLIP_L_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_IPP_SGMA_CLIP_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_IPP_SCCM_CLIP    */
#define   X2_ISP_STF_IPP_SCCM_CLIP_H(n)                (((n) & 0xfff) << 0x10)
#define   X2_ISP_STF_IPP_SCCM_CLIP_H_MASK                (0xfff << 0x10)
#define   X2_ISP_STF_IPP_SCCM_CLIP_H_SHIT(n)                (((n) & 0xfff) >> 0x10)
#define   X2_ISP_STF_IPP_SCCM_CLIP_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_IPP_SCCM_CLIP_L_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_IPP_SCCM_CLIP_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC0_CLIP    */
#define   X2_ISP_STF_IPP_SYCC0_CLIP_H(n)                (((n) & 0xfff) << 0x10)
#define   X2_ISP_STF_IPP_SYCC0_CLIP_H_MASK                (0xfff << 0x10)
#define   X2_ISP_STF_IPP_SYCC0_CLIP_H_SHIT(n)                (((n) & 0xfff) >> 0x10)
#define   X2_ISP_STF_IPP_SYCC0_CLIP_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC0_CLIP_L_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_IPP_SYCC0_CLIP_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_STF_IPP_SYCC1_CLIP    */
#define   X2_ISP_STF_IPP_SYCC1_CLIP_H(n)                (((n) & 0xfff) << 0x10)
#define   X2_ISP_STF_IPP_SYCC1_CLIP_H_MASK                (0xfff << 0x10)
#define   X2_ISP_STF_IPP_SYCC1_CLIP_H_SHIT(n)                (((n) & 0xfff) >> 0x10)
#define   X2_ISP_STF_IPP_SYCC1_CLIP_L(n)                (((n) & 0xfff) << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_CLIP_L_MASK                (0xfff << 0x0)
#define   X2_ISP_STF_IPP_SYCC1_CLIP_L_SHIT(n)                (((n) & 0xfff) >> 0x0)

/*    X2_ISP_ISP_CONFIG_DONE    */
#define   X2_ISP_ISP_CONFIG_DONE_EN(n)                (((n) & 0x1) << 0x1)
#define   X2_ISP_ISP_CONFIG_DONE_EN_MASK                (0x1 << 0x1)
#define   X2_ISP_ISP_CONFIG_DONE_EN_SHIT(n)                (((n) & 0x1) >> 0x1)
#define   X2_ISP_ISP_CONFIG_DONE(n)                (((n) & 0x1) << 0x0)
#define   X2_ISP_ISP_CONFIG_DONE_MASK                (0x1 << 0x0)
#define   X2_ISP_ISP_CONFIG_DONE_SHIT(n)                (((n) & 0x1) >> 0x0)

#endif   /*__X2_ISP__*/
