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

#ifndef __HOBOT_MIPI_DPHY_H__
#define __HOBOT_MIPI_DPHY_H__

typedef struct _mipi_dphy_tx_param_s {
	/* type must be: uint32_t */
	uint32_t txout_param_valid;
	uint32_t txout_freq_autolarge_enbale;
	uint32_t txout_freq_gain_precent;
	uint32_t txout_freq_force;
} mipi_dphy_tx_param_t;

#define MIPI_DPHY_TX_PARAM_NAMES \
	"txout_param_valid", \
	"txout_freq_autolarge_enbale", \
	"txout_freq_gain_precent", \
	"txout_freq_force"

typedef struct _mipi_phy_sub_s {
	void __iomem  *iomem;
	struct device *dev;
	void          *param;
} mipi_phy_sub_t;

int32_t mipi_dphy_register(int type, int port, mipi_phy_sub_t *sub);
int32_t mipi_dphy_unregister(int type, int port);

int32_t mipi_host_dphy_initialize(uint16_t mipiclk, uint16_t lane, uint16_t settle, void __iomem *iomem);
void    mipi_host_dphy_reset(void __iomem *iomem);

int32_t mipi_dev_dphy_initialize(void __iomem *iomem, uint16_t mipiclk, uint16_t lane, uint16_t settle);
void    mipi_dev_dphy_reset(void __iomem *iomem);

struct class * mipi_dphy_class(void);

#endif /*__HOBOT_MIPI_DPHY_H__*/
