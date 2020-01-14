/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2020 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/

#ifndef __HOBOT_MIPI_DPHY_REGS_H__
#define __HOBOT_MIPI_DPHY_REGS_H__

/*************************************************
*  MIPI dphy register offset
**************************************************/
/* ips sw regs for mipi of x2 */
#define REG_X2IPS_MIPI_DEV_PLL_CTRL1        (0xE0)
#define REG_X2IPS_MIPI_DEV_PLL_CTRL2        (0xE4)
#define REG_X2IPS_MIPI_HOST_PLL_CTRL1       (0xE8)
#define REG_X2IPS_MIPI_HOST_PLL_CTRL2       (0xEC)
#define X2IPS_MIPI_PLL_SEL_CLR_OFFS         (20)
#define X2IPS_MIPI_PLL_SEL_CLR_MASK         (0x3)

#define REG_X2IPS_MIPI_CTRL                 (0xF0)
#define X2IPS_MIPI_BPASS_GEN_DLY_OFFS       (4)
#define X2IPS_MIPI_BPASS_GEN_DLY_MASK       (0x3f)
#define X2IPS_MIPI_BPASS_GEN_HSYNC_OFFS     (1)
#define X2IPS_MIPI_BPASS_GEN_HSYNC_MASK     (0x1)
#define X2IPS_MIPI_DEV_SHADOW_CLR_OFFS      (0)
#define X2IPS_MIPI_DEV_SHADOW_CLR_MASK      (0x1)

#define REG_X2IPS_MIPI_FREQRANGE            (0xF4)
#define X2IPS_MIPI_DEV_CFGCLK_FRANGE_OFFS   (24)
#define X2IPS_MIPI_DEV_CFGCLK_FRANGE_MASK   (0xff)
#define X2IPS_MIPI_DEV_HS_FRANGE_OFFS       (16)
#define X2IPS_MIPI_DEV_HS_FRANGE_MASK       (0x7f)
#define X2IPS_MIPI_HOST_CFGCLK_FRANGE_OFFS  (8)
#define X2IPS_MIPI_HOST_CFGCLK_FRANGE_MASK  (0xff)
#define X2IPS_MIPI_HOST_HS_FRANGE_OFFS      (0)
#define X2IPS_MIPI_HOST_HS_FRANGE_MASK      (0x7f)

/* vio sw regs for mipi of x3 */
#define REG_X3VIO_MIPI_DEV_PLL_CTRL1        (0x80)
#define REG_X3VIO_MIPI_DEV_PLL_CTRL2        (0x84)

#define REG_X3VIO_MIPI_DEV_CTRL             (0x88)
#define X3VIO_MIPI_BPASS_GEN_DLY_OFFS       (4)
#define X3VIO_MIPI_BPASS_GEN_DLY_MASK       (0x3f)
#define X3VIO_MIPI_BPASS_GEN_HSYNC_OFFS     (1)
#define X3VIO_MIPI_BPASS_GEN_HSYNC_MASK     (0x1)
#define X3VIO_MIPI_DEV_SHADOW_CLR_OFFS      (0)
#define X3VIO_MIPI_DEV_SHADOW_CLR_MASK      (0x1)

#define REG_X3VIO_MIPI_DEV_FREQRANGE        (0x8C)
#define X3VIO_MIPI_DEV_CFGCLK_FRANGE_OFFS   (8)
#define X3VIO_MIPI_DEV_CFGCLK_FRANGE_MASK   (0xff)
#define X3VIO_MIPI_DEV_HS_FRANGE_OFFS       (0)
#define X3VIO_MIPI_DEV_HS_FRANGE_MASK       (0x7f)

#define REG_X3VIO_MIPI_RX0_PLL_CTRL1        (0xA0)
#define REG_X3VIO_MIPI_RX0_PLL_CTRL2        (0xA4)
#define REG_X3VIO_MIPI_RX1_PLL_CTRL1        (0xA8)
#define REG_X3VIO_MIPI_RX1_PLL_CTRL2        (0xAC)
#define REG_X3VIO_MIPI_RX2_PLL_CTRL1        (0xB0)
#define REG_X3VIO_MIPI_RX2_PLL_CTRL2        (0xB4)
#define REG_X3VIO_MIPI_RX3_PLL_CTRL1        (0xB8)
#define REG_X3VIO_MIPI_RX3_PLL_CTRL2        (0xBC)
#define X3VIO_MIPI_PLL_SEL_CLR_OFFS         (20)
#define X3VIO_MIPI_PLL_SEL_CLR_MASK         (0x3)

#define REG_X3VIO_MIPI_RX0                  (0xC0)
#define REG_X3VIO_MIPI_FREQRANGE_RX0        (0xC4)
#define REG_X3VIO_MIPI_RX1                  (0xC8)
#define REG_X3VIO_MIPI_FREQRANGE_RX1        (0xCC)
#define REG_X3VIO_MIPI_RX2                  (0xD0)
#define REG_X3VIO_MIPI_FREQRANGE_RX2        (0xD4)
#define REG_X3VIO_MIPI_RX3                  (0xD8)
#define REG_X3VIO_MIPI_FREQRANGE_RX3        (0xDC)
#define X3VIO_MIPI_RXN_CFGCLK_FRANGE_OFFS   (8)
#define X3VIO_MIPI_RXN_CFGCLK_FRANGE_MASK   (0xff)
#define X3VIO_MIPI_RXN_HS_FRANGE_OFFS       (0)
#define X3VIO_MIPI_RXN_HS_FRANGE_MASK       (0x7f)

#define REG_X3VIO_MIPI_TX_DPHY_CTRL         (0xE0)
#define X3VIO_MIPI_TX_DPHY_SEL_OFFS         (0)
#define X3VIO_MIPI_TX_DPHY_SEL_MASK         (0x1)

#define REG_X3VIO_MIPI_RX_DPHY_CTRL         (0xE4)
#define X3VIO_MIPI_RX_DPHY_MODE02_OFFS      (0)
#define X3VIO_MIPI_RX_DPHY_MODE02_MASK      (0x1)
#define X3VIO_MIPI_RX_DPHY_MODE13_OFFS      (1)
#define X3VIO_MIPI_RX_DPHY_MODE13_MASK      (0x1)

/* reg macro */
#define DP_ROFFS(r, n)    (r##_MIPI_##n##_OFFS)
#define DP_RMASK(r, n)    (r##_MIPI_##n##_MASK)
#define DP_VMASK(r, n)    (DP_RMASK(r, n) << DP_ROFFS(r, n))
#define DP_REG2V(r, n, v) (((v) >> DP_ROFFS(r,n)) & DP_RMASK(r,n))
#define DP_V2REG(r, n, v) (((v) << DP_ROFFS(r,n)) & DP_RMASK(r,n))

#endif //__HOBOT_MIPI_DPHY_REGS_H__
