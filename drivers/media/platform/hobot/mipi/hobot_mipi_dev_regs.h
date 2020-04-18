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

#ifndef __HOBOT_MIPI_DEV_REGS_H__
#define __HOBOT_MIPI_DEV_REGS_H__

/*************************************************
*  MIPI device register offset
**************************************************/
#define REG_MIPI_DEV_VERSION                (0x00)
#define REG_MIPI_DEV_CSI2_RESETN            (0x04)
#define REG_MIPI_DEV_INT_ST_MAIN            (0x20)
#define REG_MIPI_DEV_INT_ST_VPG             (0x24)
#define REG_MIPI_DEV_INT_ST_IDI             (0x28)
#define REG_MIPI_DEV_INT_ST_IPI             (0x2C)
#define REG_MIPI_DEV_INT_ST_PHY             (0x30)
#define REG_MIPI_DEV_INT_MASK_N_VPG         (0x40)
#define REG_MIPI_DEV_INT_FORCE_VPG          (0x44)
#define REG_MIPI_DEV_INT_MASK_N_IDI         (0x48)
#define REG_MIPI_DEV_INT_FORCE_IDI          (0x4C)
#define REG_MIPI_DEV_INT_MASK_N_IPI         (0x50)
#define REG_MIPI_DEV_INT_FORCE_IPI          (0x54)
#define REG_MIPI_DEV_INT_MASK_N_PHY         (0x58)
#define REG_MIPI_DEV_INT_FORCE_PHY          (0x5C)
#define REG_MIPI_DEV_VPG_CTRL               (0x80)
#define REG_MIPI_DEV_VPG_STATUS             (0x84)
#define REG_MIPI_DEV_VPG_MODE_CFG           (0x88)
#define REG_MIPI_DEV_VPG_PKT_CFG            (0x8C)
#define REG_MIPI_DEV_VPG_PKT_SIZE           (0x90)
#define REG_MIPI_DEV_VPG_HSA_TIME           (0x94)
#define REG_MIPI_DEV_VPG_HBP_TIME           (0x98)
#define REG_MIPI_DEV_VPG_HLINE_TIME         (0x9C)
#define REG_MIPI_DEV_VPG_VSA_LINES          (0xA0)
#define REG_MIPI_DEV_VPG_VBP_LINES          (0xA4)
#define REG_MIPI_DEV_VPG_VFP_LINES          (0xA8)
#define REG_MIPI_DEV_VPG_ACT_LINES          (0xAC)
#define REG_MIPI_DEV_VPG_MAX_FRAME_NUM      (0xB0)
#define REG_MIPI_DEV_VPG_START_LINE_NUM     (0xB4)
#define REG_MIPI_DEV_VPG_STEP_LINE_NUM      (0xB8)
#define REG_MIPI_DEV_VPG_BK_LINES           (0xBC)
#define REG_MIPI_DEV_PHY_RSTZ               (0xE0)
#define REG_MIPI_DEV_PHY_IF_CFG             (0xE4)
#define REG_MIPI_DEV_LPCLK_CTRL             (0xE8)
#define REG_MIPI_DEV_PHY_ULPS_CTRL          (0xEC)
#define REG_MIPI_DEV_CLKMGR_CFG             (0xF0)
#define REG_MIPI_DEV_PHY_TX_TRIGGERS        (0xF4)
#define REG_MIPI_DEV_PHY_CAL                (0xF8)
#define REG_MIPI_DEV_TO_CNT_CFG             (0xFC)
#define REG_MIPI_DEV_PHY_STATUS             (0x110)
#define REG_MIPI_DEV_PHY0_TST_CTRL0         (0x114)
#define REG_MIPI_DEV_PHY0_TST_CTRL1         (0x118)
#define REG_MIPI_DEV_PHY1_TST_CTRL0         (0x11C)
#define REG_MIPI_DEV_PHY1_TST_CTRL1         (0x120)
#define REG_MIPI_DEV_IPI_PKT_CFG            (0x140)
#define REG_MIPI_DEV_IPI_PIXELS             (0x144)
#define REG_MIPI_DEV_IPI_MAX_FRAME_NUM      (0x148)
#define REG_MIPI_DEV_IPI_START_LINE_NUM     (0x14C)
#define REG_MIPI_DEV_IPI_STEP_LINE_NUM      (0x150)
#define REG_MIPI_DEV_IPI_LINES              (0x154)
#define REG_MIPI_DEV_IPI_DATA_SEND_START    (0x158)

#endif //__HOBOT_MIPI_DEV_REGS_H__
