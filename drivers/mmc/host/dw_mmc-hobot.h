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

#ifndef _DW_MMC_HOBOT_H_
#define _DW_MMC_HOBOT_H_

#include <linux/pinctrl/consumer.h>

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
	struct pinctrl *pin_ctrl;
	struct pinctrl_state *pin_state_1_8v;
	struct pinctrl_state *pin_state_3_3v;
	int mmc_fixed_voltage;
};

int hb_mmc_disable_clk(struct dw_mci_hobot_priv_data *priv);
int hb_mmc_enable_clk(struct dw_mci_hobot_priv_data *priv);
void hb_mmc_set_power(struct dw_mci_hobot_priv_data *priv, bool val);

#endif