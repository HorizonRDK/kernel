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
	void __iomem *clk_ctrl_reg;
	void __iomem *padcctrl_reg;
	bool not_mmc;
	u32 clock_frequency;
	int default_sample_phase;
	u32 uhs_180v_gpio;
	u32 uhs_180v_logic;
	u32 powerup_gpio;
	u32 powerup_logic;
	u32 ctrl_id;
	int current_drv_phase;
	int current_sample_phase;
	int current_phase_cnt;
	struct pinctrl *pin_ctrl;
	struct pinctrl_state *pin_state_1_8v;
	struct pinctrl_state *pin_state_3_3v;
	int mmc_fixed_voltage;
/*using U-boot stage tuning result*/
#ifdef CONFIG_MMC_TUNE_FROM_UBOOT
	bool is_get_tune;
	u32 uboot_tune_phase;
#endif /*CONFIG_MMC_TUNE_FROM_UBOOT*/
};

#ifdef CONFIG_MMC_TUNE_FROM_UBOOT
#define MMC_UBOOT_GET_TUNE_OK	0
#define MMC_UBOOT_GET_TUNE_ERR	1
#define IS_TUNE_PHASE_ERR(x) \
	(((x) > 0xf) ? MMC_UBOOT_GET_TUNE_ERR : MMC_UBOOT_GET_TUNE_OK)
#endif /*CONFIG_MMC_TUNE_FROM_UBOOT*/

int hb_mmc_disable_clk(struct dw_mci *host);
int hb_mmc_enable_clk(struct dw_mci *host);
void hb_mmc_set_power(struct dw_mci_hobot_priv_data *priv, bool val);

#endif
