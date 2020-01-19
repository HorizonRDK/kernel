/*
 * Copyright (c) 2019, Horizon Robotics, Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _DW_MMC_HOBOT_H_
#define _DW_MMC_HOBOT_H_

struct dw_mci_hobot_priv_data {
	void __iomem *sysctrl_reg;
	void __iomem *padcctrl_reg;
	u32 clock_frequency;
	int default_sample_phase;
	u32 uhs_180v_gpio;
	u32 uhs_180v_logic;
	u32 powerup_gpio;
	u32 powerup_logic;
	u32 ctrl_id;
	u8 current_drv_phase;
	u8 current_sample_phase;
	u8 current_phase_cnt;
};

int x2_mmc_disable_clk(struct dw_mci_hobot_priv_data *priv);
int x2_mmc_enable_clk(struct dw_mci_hobot_priv_data *priv);
void x2_mmc_set_power(struct dw_mci_hobot_priv_data *priv, bool val);

#endif
