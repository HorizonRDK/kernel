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

#ifndef __HOBOT_MIPI_HOST_REGS_H__
#define __HOBOT_MIPI_HOST_REGS_H__

/*************************************************
*  MIPI host register offset
**************************************************/
#define REG_MIPI_HOST_VERSION                  (0x00)
#define REG_MIPI_HOST_N_LANES                  (0x04)
#define REG_MIPI_HOST_CSI2_RESETN              (0x08)
#define REG_MIPI_HOST_INT_ST_MAIN              (0x0c)
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
#define REG_MIPI_HOST_IPI_SOFTRSTN             (0xa0)
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
#define REG_MIPI_HOST_INT_ST_IPI3              (0x160)
#define REG_MIPI_HOST_INT_MSK_IPI3             (0x164)
#define REG_MIPI_HOST_INT_FORCE_IPI3           (0x168)
#define REG_MIPI_HOST_INT_ST_IPI4              (0x170)
#define REG_MIPI_HOST_INT_MSK_IPI4             (0x174)
#define REG_MIPI_HOST_INT_FORCE_IPI4           (0x178)
#define REG_MIPI_HOST_IPI2_MODE                (0x200)
#define REG_MIPI_HOST_IPI2_VCID                (0x204)
#define REG_MIPI_HOST_IPI2_DATA_TYPE           (0x208)
#define REG_MIPI_HOST_IPI2_MEM_FLUSH           (0x20c)
#define REG_MIPI_HOST_IPI2_HSA_TIME            (0x210)
#define REG_MIPI_HOST_IPI2_HBP_TIME            (0x214)
#define REG_MIPI_HOST_IPI2_HSD_TIME            (0x218)
#define REG_MIPI_HOST_IPI2_ADV_FEATURES        (0x21c)
#define REG_MIPI_HOST_IPI3_MODE                (0x220)
#define REG_MIPI_HOST_IPI3_VCID                (0x224)
#define REG_MIPI_HOST_IPI3_DATA_TYPE           (0x228)
#define REG_MIPI_HOST_IPI3_MEM_FLUSH           (0x22c)
#define REG_MIPI_HOST_IPI3_HSA_TIME            (0x230)
#define REG_MIPI_HOST_IPI3_HBP_TIME            (0x234)
#define REG_MIPI_HOST_IPI3_HSD_TIME            (0x238)
#define REG_MIPI_HOST_IPI3_ADV_FEATURES        (0x23c)
#define REG_MIPI_HOST_IPI4_MODE                (0x240)
#define REG_MIPI_HOST_IPI4_VCID                (0x244)
#define REG_MIPI_HOST_IPI4_DATA_TYPE           (0x248)
#define REG_MIPI_HOST_IPI4_MEM_FLUSH           (0x24c)
#define REG_MIPI_HOST_IPI4_HSA_TIME            (0x250)
#define REG_MIPI_HOST_IPI4_HBP_TIME            (0x254)
#define REG_MIPI_HOST_IPI4_HSD_TIME            (0x258)
#define REG_MIPI_HOST_IPI4_ADV_FEATURES        (0x25c)
#define REG_MIPI_HOST_INT_ST_BNDRY_FRAME_FATAL (0x280)
#define REG_MIPI_HOST_INT_MSK_BNDRY_FRAME_FATAL (0x284)
#define REG_MIPI_HOST_INT_FORCE_BNDRY_FRAME_FATAL (0x288)
#define REG_MIPI_HOST_INT_ST_SEQ_FRAME_FATAL   (0x290)
#define REG_MIPI_HOST_INT_MSK_SEQ_FRAME_FATAL  (0x294)
#define REG_MIPI_HOST_INT_FORCE_SEQ_FRAME_FATAL (0x298)
#define REG_MIPI_HOST_INT_ST_CRC_FRAME_FATAL   (0x2A0)
#define REG_MIPI_HOST_INT_MSK_CRC_FRAME_FATAL  (0x2A4)
#define REG_MIPI_HOST_INT_FORCE_CRC_FRAME_FATAL (0x2A8)
#define REG_MIPI_HOST_INT_ST_PLD_CRC_FATAL     (0x2B0)
#define REG_MIPI_HOST_INT_MSK_PLD_CRC_FATAL    (0x2B4)
#define REG_MIPI_HOST_INT_FORCE_PLD_CRC_FATAL  (0x2B8)
#define REG_MIPI_HOST_INT_ST_DATA_ID           (0x2C0)
#define REG_MIPI_HOST_INT_MSK_DATA_ID          (0x2C4)
#define REG_MIPI_HOST_INT_FORCE_DATA_ID        (0x2C8)
#define REG_MIPI_HOST_INT_ST_ECC_CORRECT       (0x2D0)
#define REG_MIPI_HOST_INT_MSK_ECC_CORRECT      (0x2D4)
#define REG_MIPI_HOST_INT_FORCE_ECC_CORRECT    (0x2D8)

/* IP VERSION */
#ifdef CONFIG_ARCH_HOBOT
#define MIPI_HOST_IS_1P4(b)	       ((mipi_getreg((b) + REG_MIPI_HOST_VERSION)) \
									> 0x31343000)
#else
#define MIPI_HOST_IS_1P4(b)	       ((mipi_getreg((b) + REG_MIPI_HOST_VERSION)) \
									> 0x00003100)
#endif

#endif //__HOBOT_MIPI_HOST_REGS_H__
