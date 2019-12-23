/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file     mipi_host_control_regs.h
 * @brief    MIPI HOST controler regs addr
 * @author   tarryzhang (tianyu.zhang@hobot.cc)
 * @date     2017/7/6
 * @version  V1.0
 * @par      Horizon Robotics
 */
#ifndef __HOBOT_MIPI_HOST_REGS_H__
#define __HOBOT_MIPI_HOST_REGS_H__

/*************************************************
*  MIPI register offset
**************************************************/
#define REG_MIPI_HOST_VERSION                  (0x00)
#define REG_MIPI_HOST_N_LANES                  (0x04)
#define REG_MIPI_HOST_CSI2_RESETN              (0x08)
#define REG_MIPI_HOST_INT_ST_MAIN              (0x0c)
#define REG_MIPI_HOST_DATA_IDS_1               (0x10)
#define REG_MIPI_HOST_DATA_IDS_2               (0x14)
#define REG_MIPI_HOST_PHY_SHUTDOWNZ            (0x40)
#define REG_MIPI_HOST_DPHY_RSTZ                (0x44)
#define REG_MIPI_HOST_PHY_RX                   (0x48)
#define REG_MIPI_HOST_PHY_STOPSTATE            (0x4c)
#define REG_MIPI_HOST_PHY_TEST_CTRL0           (0x50)
#define REG_MIPI_HOST_PHY_TEST_CTRL1           (0x54)
#define REG_MIPI_HOST_PHY2_TEST_CTRL0          (0x58)
#define REG_MIPI_HOST_PHY2_TEST_CTRL1          (0x5c)
#define REG_MIPI_HOST_IPI_MODE                 (0x80)
#define REG_MIPI_HOST_IPI_VCID                 (0x84)
#define REG_MIPI_HOST_IPI_DATA_TYPE            (0x88)
#define REG_MIPI_HOST_IPI_MEM_FLUSH            (0x8c)
#define REG_MIPI_HOST_IPI_HSA_TIME             (0x90)
#define REG_MIPI_HOST_IPI_HBP_TIME             (0x94)
#define REG_MIPI_HOST_IPI_HSD_TIME             (0x98)
#define REG_MIPI_HOST_IPI_HLINE_TIME           (0x9c)
#define REG_MIPI_HOST_IPI_ADV_FEATURES         (0xac)
#define REG_MIPI_HOST_IPI_VSA_LINES            (0xb0)
#define REG_MIPI_HOST_IPI_VBP_LINES            (0xb4)
#define REG_MIPI_HOST_IPI_VFP_LINES            (0xb8)
#define REG_MIPI_HOST_IPI_VACTIVE_LINES        (0xbc)
#define REG_MIPI_HOST_PHY_CAL                  (0xcc)
#define REG_MIPI_HOST_INT_ST_PHY_FATAL         (0xe0)
#define REG_MIPI_HOST_INT_MSK_PHY_FATAL        (0xe4)
#define REG_MIPI_HOST_INT_FORCE_PHY_FATAL      (0xe8)
#define REG_MIPI_HOST_INT_ST_PKT_FATAL         (0xf0)
#define REG_MIPI_HOST_INT_MSK_PKT_FATAL        (0xf4)
#define REG_MIPI_HOST_INT_FORCE_PKT_FATAL      (0xf8)
#define REG_MIPI_HOST_INT_ST_FRAME_FATAL       (0x100)
#define REG_MIPI_HOST_INT_MSK_FRAME_FATAL      (0x104)
#define REG_MIPI_HOST_INT_FORCE_FRAME_FATAL    (0x108)
#define REG_MIPI_HOST_INT_ST_PHY               (0x110)
#define REG_MIPI_HOST_INT_MSK_PHY              (0x114)
#define REG_MIPI_HOST_INT_FORCE_PHY            (0x118)
#define REG_MIPI_HOST_INT_ST_PKT               (0x120)
#define REG_MIPI_HOST_INT_MSK_PKT              (0x124)
#define REG_MIPI_HOST_INT_FORCE_PKT            (0x128)
#define REG_MIPI_HOST_INT_ST_LINE              (0x130)
#define REG_MIPI_HOST_INT_MSK_LINE             (0x134)
#define REG_MIPI_HOST_INT_FORCE_LINE           (0x138)
#define REG_MIPI_HOST_INT_ST_IPI               (0x140)
#define REG_MIPI_HOST_INT_MSK_IPI              (0x144)
#define REG_MIPI_HOST_INT_FORCE_IPI            (0x148)
#define REG_MIPI_HOST_INT_ST_IPI2              (0x150)
#define REG_MIPI_HOST_INT_MSK_IPI2             (0x154)
#define REG_MIPI_HOST_INT_FORCE_IPI2           (0x158)
#define REG_MIPI_HOST_IPI2_MODE                (0x200)
#define REG_MIPI_HOST_IPI2_VCID                (0x204)
#define REG_MIPI_HOST_IPI2_DATA_TYPE           (0x208)
#define REG_MIPI_HOST_IPI2_MEM_FLUSH           (0x20c)
#define REG_MIPI_HOST_IPI2_HSA_TIME            (0x210)
#define REG_MIPI_HOST_IPI2_HBP_TIME            (0x214)
#define REG_MIPI_HOST_IPI2_HSD_TIME            (0x218)
#define REG_MIPI_HOST_IPI2_ADV_FEATURES        (0x21c)

#endif //__HOBOT_MIPI_HOST_REGS_H__
