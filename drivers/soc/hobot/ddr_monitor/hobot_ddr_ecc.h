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

#ifndef __HOBOT_DDR_ECC_H__
#define __HOBOT_DDR_ECC_H__

#include "./hobot_ddr_monitor.h"

/* Define DDRC registers for ECC interrupt */
#define uMCTL2_ECCCFG0         0x0070
#define uMCTL2_ECCSTAT         0x0078
#define ECC_STAT_CORR_ERR      BIT8
#define ECC_STAT_UNCORR_ERR    BIT16

#define uMCTL2_ECCERRCNT       0x0080

#define uMCTL2_ECCCTL          0x007c
#define ECC_CORR_ERR_INTR_EN   BIT8
#define ECC_UNCORR_ERR_INTR_EN BIT9
#define ECC_UNCORR_ERR_CNT_CLR BIT3
#define ECC_CORR_ERR_CNT_CLR   BIT2
#define ECC_UNCORR_ERR_CLR     BIT1
#define ECC_CORR_ERR_CLR       BIT0


extern int hobot_ddr_ecc_init(struct platform_device *pdev);
extern int hobot_ddr_ecc_deinit(struct platform_device *pdev);

#endif //__HOBOT_DDR_ECC_H__
