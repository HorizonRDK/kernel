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

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"
#include "dw_mmc-hobot.h"

#define DWMMC_MMC_ID			(1)
#define DWMMC_SD1_ID			(2)
#define DWMMC_SD2_ID			(3)
#ifdef CONFIG_HOBOT_XJ3
/* Maximum mmc device is 3, starting from 1 used to determine register offset*/
#define MMC_MAX_ID				(3)
#else
#define MMC_MAX_ID				(1)
#endif
#define HOBOT_DW_MCI_FREQ_MAX		(200000000)
#define HOBOT_SYSCTRL_REG		(0xA1000000)
#define HOBOT_SD0_PHASE_REG		(0x320)
#define HOBOT_SD1_PHASE_REG		(0x330)
#define HOBOT_SD2_PHASE_REG		(0x340)

#define HOBOT_CLKEN_CLR			(0x158)
#define HOBOT_SD0_CLKEN_CLR_SHIFT	(15)
#define HOBOT_SD1_CLKEN_CLR_SHIFT	(16)
#define HOBOT_SD2_CLKEN_CLR_SHIFT	(25)

#define HOBOT_CLKEN_SET			(0x154)
#define HOBOT_SD0_CLKEN_SET_SHIFT	(15)
#define HOBOT_SD1_CLKEN_SET_SHIFT	(16)
#define HOBOT_SD2_CLKEN_SET_SHIFT	(25)

#define HOBOT_CLKOFF_STA		(0x258)
#define HOBOT_SD0_CLKOFF_STA_SHIFT	(1<<1)
#define HOBOT_SD1_CLKOFF_STA_SHIFT	(1<<2)
#define HOBOT_SD2_CLKOFF_STA_SHIFT	(1<<3)

/*#define HOBOT_MMC_DEGREE_MASK			(0xF)*/
#define HOBOT_MMC_SAMPLE_DEGREE_SHIFT	(12)
#define HOBOT_MMC_DRV_DEGREE_SHIFT	(8)
#define SDCARD_RD_THRESHOLD		(512)
#define NUM_PHASES			(16)
#define TUNING_ITERATION_TO_PHASE(i)	(i)

#define HOBOT_PDACCTRL_REG		(0xA6003000)
#define HOBOT_SD0_PDACCTRL0_SHIFT	(0x80)
#define HOBOT_SD0_PDACCTRL1_SHIFT	(0x84)
#define HOBOT_SD0_PDACCTRL2_SHIFT	(0x88)

#define SD0_PADC_VAL_CLR0		(0xF8F8F8E8)
#define SD0_PADC_VAL_CLR1		(0xF8F8F8F8)
#define SD0_PADC_VAL_CLR2		(0xF8F8F8F8)
#define SD0_PADC_VAL			(0x02020202)
//hobot
#define HOBOT_SD1_PDACCTRL0_SHIFT	(0x90)
#define HOBOT_SD1_PDACCTRL1_SHIFT	(0x94)
#define SD1_PADC_VAL_CLR0		(0xF8F8F8E8)
#define SD1_PADC_VAL_CLR1		(0xF8F8F8F8)
#define SD1_PADC_VAL			(0x04040404)

#define VER		"HOBOT-mmc_V20.200402"

static int hb_mmc_set_sd_padcctrl(struct dw_mci_hobot_priv_data *priv)
{
#ifdef CONFIG_HOBOT_XJ2
	u32 reg_value;
	if (priv->ctrl_id == DWMMC_MMC_ID) {
		reg_value =
		    readl(priv->padcctrl_reg + HOBOT_SD0_PDACCTRL0_SHIFT);
		reg_value &= SD0_PADC_VAL_CLR0;
		reg_value |= SD0_PADC_VAL;
		writel(reg_value,
		       priv->padcctrl_reg + HOBOT_SD0_PDACCTRL0_SHIFT);

		reg_value =
		    readl(priv->padcctrl_reg + HOBOT_SD0_PDACCTRL1_SHIFT);
		reg_value &= SD0_PADC_VAL_CLR1;
		reg_value |= SD0_PADC_VAL;
		writel(reg_value,
		       priv->padcctrl_reg + HOBOT_SD0_PDACCTRL1_SHIFT);

		reg_value =
		    readl(priv->padcctrl_reg + HOBOT_SD0_PDACCTRL2_SHIFT);
		reg_value &= SD0_PADC_VAL_CLR2;
		reg_value |= SD0_PADC_VAL;
		writel(reg_value,
		       priv->padcctrl_reg + HOBOT_SD0_PDACCTRL2_SHIFT);
	} else {
		reg_value =
		    readl(priv->padcctrl_reg + HOBOT_SD1_PDACCTRL0_SHIFT);
		reg_value &= SD1_PADC_VAL_CLR0;
		reg_value |= SD1_PADC_VAL;
		writel(reg_value,
		       priv->padcctrl_reg + HOBOT_SD1_PDACCTRL0_SHIFT);

		reg_value =
		    readl(priv->padcctrl_reg + HOBOT_SD1_PDACCTRL1_SHIFT);
		reg_value &= SD1_PADC_VAL_CLR1;
		reg_value |= SD1_PADC_VAL;
		writel(reg_value,
		       priv->padcctrl_reg + HOBOT_SD1_PDACCTRL1_SHIFT);
	}
#endif
	return 0;
}

int hb_mmc_disable_clk(struct dw_mci *host)
{
#ifdef CONFIG_HOBOT_XJ2
	struct dw_mci_hobot_priv_data *priv = host->priv;
	unsigned long timeout = jiffies + msecs_to_jiffies(10);
	u32 clken_clr_shift = 0, clkoff_sta_shift = 0;
	u32 reg_value;
	int retry = 0;

	if (priv->ctrl_id == DWMMC_MMC_ID) {
		clken_clr_shift = HOBOT_SD0_CLKEN_CLR_SHIFT;
		clkoff_sta_shift = HOBOT_SD0_CLKOFF_STA_SHIFT;
	} else if (priv->ctrl_id == DWMMC_SD1_ID) {
		clken_clr_shift = HOBOT_SD1_CLKEN_CLR_SHIFT;
		clkoff_sta_shift = HOBOT_SD1_CLKOFF_STA_SHIFT;
	}

	while (retry++ < 5) {
		reg_value = 1 << clken_clr_shift;
		writel(reg_value, priv->sysctrl_reg + HOBOT_CLKEN_CLR);

		while (time_before(jiffies, timeout)) {
			reg_value = readl(priv->sysctrl_reg + HOBOT_CLKOFF_STA);
			if (reg_value & clkoff_sta_shift) {
				usleep_range(1, 2);
				return 0;
			}
			usleep_range(1, 2);
		}

		timeout = jiffies + msecs_to_jiffies(10);
		pr_debug("Warn disable mmc clk failed %d times\n", retry);
	}

	pr_err("Err disable mmc clk, ctrl_id:%d!\n", priv->ctrl_id);

	return -1;
#else
	clk_disable(host->ciu_clk);
	return 0;
#endif
}

int hb_mmc_enable_clk(struct dw_mci *host)
{
#ifdef CONFIG_HOBOT_XJ2
	struct dw_mci_hobot_priv_data *priv = host->priv;
	unsigned long timeout = jiffies + msecs_to_jiffies(10);
	u32 clken_set_shift = 0, clkoff_sta_shift = 0;
	u32 reg_value;
	int retry = 0;

	if (priv->ctrl_id == DWMMC_MMC_ID) {
		clken_set_shift = HOBOT_SD0_CLKEN_SET_SHIFT;
		clkoff_sta_shift = HOBOT_SD0_CLKOFF_STA_SHIFT;
	} else if (priv->ctrl_id == DWMMC_SD1_ID) {
		clken_set_shift = HOBOT_SD1_CLKEN_SET_SHIFT;
		clkoff_sta_shift = HOBOT_SD1_CLKOFF_STA_SHIFT;
	}

	while (retry++ < 5) {
		reg_value = 1 << clken_set_shift;
		writel(reg_value, priv->sysctrl_reg + HOBOT_CLKEN_SET);

		while (time_before(jiffies, timeout)) {
			reg_value = readl(priv->sysctrl_reg + HOBOT_CLKOFF_STA);
			if (!(reg_value & clkoff_sta_shift)) {
				if (priv->ctrl_id == DWMMC_MMC_ID)
					usleep_range(10, 20);
				else
					usleep_range(20000, 25000);
				return 0;
			}
			usleep_range(1, 2);
		}

		timeout = jiffies + msecs_to_jiffies(10);
		pr_debug("Warn enable mmc clk failed %d times\n", retry);
	}

	pr_err("Err enable mmc clk, ctrl_id:%d!\n", priv->ctrl_id);

	return -1;
#else
	clk_enable(host->ciu_clk);
	return 0;
#endif
}

void hb_mmc_set_power(struct dw_mci_hobot_priv_data *priv, bool val)
{
	bool logic_val = val;

	if (priv->powerup_logic) {
		if (val)
			logic_val = 0;
		else
			logic_val = 1;
	}
	if (priv->not_mmc) {
		if (priv->powerup_gpio) {
			gpio_direction_output(priv->powerup_gpio, logic_val);
			usleep_range(1000, 2000);
		}
	}
}

static int hb_mmc_set_sample_phase(struct dw_mci *host,
				   int degrees)
{
	struct dw_mci_hobot_priv_data *priv = host->priv;
	u32 reg_value;
	if (priv->clk_ctrl_reg == NULL) {
		dev_info_once(host->dev,
				"Clock ctrl register not found, ignoring clk operation.\n");
		return 0;
	}

	priv->current_sample_phase = degrees;
	pr_debug("Sample_phase set: %d\n", degrees);

	disable_irq(host->irq);
	tasklet_disable(&host->tasklet);
	hb_mmc_disable_clk(host);

	reg_value = readl(priv->clk_ctrl_reg);
	reg_value &= 0xFFFF0FFF;
	reg_value |= degrees << HOBOT_MMC_SAMPLE_DEGREE_SHIFT;
	writel(reg_value, priv->clk_ctrl_reg);

	hb_mmc_enable_clk(host);
	/* usleep is required for SDcard/SDIO initialization */
	if (priv->not_mmc)
		usleep_range(20000, 25000);
	tasklet_enable(&host->tasklet);
	enable_irq(host->irq);
	return 0;
}

static int hb_mmc_set_drv_phase(struct dw_mci *host,
				int degrees)
{
	struct dw_mci_hobot_priv_data *priv = host->priv;
	u32 reg_value;
	if (priv->clk_ctrl_reg == NULL) {
		dev_info_once(host->dev,
				"Clock ctrl register not found, ignoring clk operation.\n");
		return 0;
	}
	priv->current_drv_phase = degrees;

	disable_irq(host->irq);
	tasklet_disable(&host->tasklet);
	hb_mmc_disable_clk(host);

	reg_value = readl(priv->clk_ctrl_reg);
	reg_value &= 0xFFFFF0FF;
	reg_value |= degrees << HOBOT_MMC_DRV_DEGREE_SHIFT;;
	writel(reg_value, priv->clk_ctrl_reg);

	hb_mmc_enable_clk(host);
	/* usleep is required for SDcard/SDIO initialization */
	if (priv->not_mmc)
		usleep_range(20000, 25000);
	tasklet_enable(&host->tasklet);
	enable_irq(host->irq);
	return 0;
}

static int dw_mci_hb_sample_tuning(struct dw_mci_slot *slot, u32 opcode)
{
	struct dw_mci *host = slot->host;
	struct dw_mci_hobot_priv_data *priv = host->priv;
	struct mmc_host *mmc = slot->mmc;
	int ret = 0;
	int i;
	bool v, prev_v = 0, first_v;
	struct range_t {
		int start;
		int end;	/* inclusive */
	};
	struct range_t *ranges;
	unsigned int range_count = 0;
	int longest_range_len = -1;
	int longest_range = -1;
	int middle_phase;

	ranges = kmalloc_array(NUM_PHASES / 2 + 1, sizeof(*ranges), GFP_KERNEL);
	if (!ranges)
		return -ENOMEM;

	/* Try each phase and extract good ranges */
	for (i = 0; i < NUM_PHASES; i++) {
		hb_mmc_set_sample_phase(host, TUNING_ITERATION_TO_PHASE(i));

		v = !mmc_send_tuning(mmc, opcode, NULL);

		if (i == 0)
			first_v = v;

		if ((!prev_v) && v) {
			range_count++;
			ranges[range_count - 1].start = i;
		}

		if (v) {
			ranges[range_count - 1].end = i;
		} else if (i < NUM_PHASES - 2) {
			/*
			 * No need to check too close to an invalid
			 * one since testing bad phases is slow. Skip
			 * the adjacent phase but always test the last phase.
			 */
			i++;
		}

		prev_v = v;
	}

	if (range_count == 0) {
		dev_dbg(host->dev, "All sample phases bad!");
		ret = -EIO;
		goto free;
	}

	/* wrap around case, merge the end points */
	if ((range_count > 1) && first_v && v) {
		ranges[0].start = ranges[range_count - 1].start;
		range_count--;
	}

	if (ranges[0].start == 0 && ranges[0].end == NUM_PHASES - 1) {
		hb_mmc_set_sample_phase(host, priv->default_sample_phase);
		dev_dbg(host->dev,
			"All sample phases work, using default phase %d.",
			priv->default_sample_phase);
		goto free;
	}

	/* Find the longest range */
	for (i = 0; i < range_count; i++) {
		int len = (ranges[i].end - ranges[i].start + 1);

		if (len < 0)
			len += NUM_PHASES;

		if (longest_range_len < len) {
			longest_range_len = len;
			longest_range = i;
		}

		dev_dbg(host->dev,
			"current_drv_phase=%d,Good sample phase range %d-%d (%d len)\n",
			priv->current_drv_phase,
			TUNING_ITERATION_TO_PHASE(ranges[i].start),
			TUNING_ITERATION_TO_PHASE(ranges[i].end), len);
	}

	dev_dbg(host->dev,
		"current_drv_phase=%d, Best sample phase range %d-%d (%d len)\n",
		priv->current_drv_phase,
		TUNING_ITERATION_TO_PHASE(ranges[longest_range].start),
		TUNING_ITERATION_TO_PHASE(ranges[longest_range].end),
		longest_range_len);

	middle_phase = ranges[longest_range].start + longest_range_len / 2;
	middle_phase %= NUM_PHASES;
	dev_dbg(host->dev,
		"current_drv_phase=%d,Successfully tuned sample phase to %d\n",
		priv->current_drv_phase,
		TUNING_ITERATION_TO_PHASE(middle_phase));

	hb_mmc_set_sample_phase(host, TUNING_ITERATION_TO_PHASE(middle_phase));

free:
	kfree(ranges);
	return ret;
}

#ifdef CONFIG_MMC_TUNE_FROM_UBOOT
/* check U-boot tuning data is less than or equal to 0xf */
static int __inline__ check_uboot_tune_para(struct dw_mci *host)
{
	struct dw_mci_hobot_priv_data *priv = host->priv;

	if( (priv->is_get_tune == MMC_UBOOT_GET_TUNE_ERR)
		|| IS_TUNE_PHASE_ERR(priv->uboot_tune_phase))
		return MMC_UBOOT_GET_TUNE_ERR;

	dev_err(host->dev, "get mmc tune data from uboot:%d.\n",
			priv->uboot_tune_phase);
	return MMC_UBOOT_GET_TUNE_OK;
}

static int dw_mci_hb_try_tuning_from_uboot(struct dw_mci_slot *slot, u32 opcode)
{
	int ret;
	unsigned int middle_phase;
	struct dw_mci *host = slot->host;
	struct dw_mci_hobot_priv_data *priv = host->priv;
	struct mmc_host *mmc = slot->mmc;

	ret = check_uboot_tune_para(host);
	if (ret == MMC_UBOOT_GET_TUNE_ERR) {
		dev_err(host->dev,
				"check uboot tune data error.\n");
		/*get uboot tuning data error, run normal tuning*/
		return dw_mci_hb_sample_tuning(slot, opcode);
	}
	middle_phase = priv->uboot_tune_phase;
	/*using uboot stage tuning data execute tuning once*/
	hb_mmc_set_sample_phase(host, TUNING_ITERATION_TO_PHASE(middle_phase));
	ret = mmc_send_tuning(mmc, opcode, NULL);
	if (ret) {
		dev_err(host->dev, "using uboot tune data fail, middle_phase= %d\n",
			middle_phase);
		return dw_mci_hb_sample_tuning(slot, opcode); /*normal tuning step*/
	}
	dev_info(host->dev,
			 "use sample tuning phase from uboot: %d\n", middle_phase);

	return ret;
}
#endif /*CONFIG_MMC_TUNE_FROM_UBOOT*/

static int dw_mci_hb_execute_tuning(struct dw_mci_slot *slot, u32 opcode)
{
	int ret = -EIO;

#ifdef CONFIG_MMC_TUNE_FROM_UBOOT
	ret = dw_mci_hb_try_tuning_from_uboot(slot, opcode);
#else
	ret = dw_mci_hb_sample_tuning(slot, opcode);
#endif

	return ret;
}

static void dw_mci_hb_set_ios(struct dw_mci *host, struct mmc_ios *ios)
{
	struct dw_mci_hobot_priv_data *priv = host->priv;
	int ret;
	unsigned int cclkin;
	unsigned long bus_hz;
	int phase;

#ifdef CONFIG_HOBOT_FPGA_X3
	return;
#endif
	if (ios->clock == 0)
		return;
	/*
	 * cclkin: source clock of mmc controller
	 * bus_hz: card interface clock generated by CLKGEN
	 * bus_hz = cclkin / HOBOT_CLKGEN_DIV
	 * ios->clock = (div == 0) ? bus_hz : (bus_hz / (2 * div))
	 *
	 * Note: div can only be 0 or 1
	 *       if DDR50 8bit mode(only emmc work in 8bit mode),
	 *       div must be set 1
	 */
	if (ios->bus_width == MMC_BUS_WIDTH_8 &&
	    ios->timing == MMC_TIMING_MMC_DDR52)
		cclkin = 2 * ios->clock;
	else
		cclkin = ios->clock;

	ret = clk_set_rate(host->ciu_clk, cclkin);
	if (ret)
		dev_warn(host->dev, "failed to set rate %uHz\n", ios->clock);

	bus_hz = clk_get_rate(host->ciu_clk);
	if (bus_hz != host->bus_hz) {
		host->bus_hz = bus_hz;
		/* force dw_mci_setup_bus() */
		host->current_speed = 0;
	}

	/* Make sure we use phases which we can enumerate with */
	hb_mmc_set_sample_phase(host, priv->default_sample_phase);

	/*
	 * Set the drive phase offset based on speed mode to achieve hold times.
	 *
	 * NOTE: this is _not_ a value that is dynamically tuned and is also
	 * _not_ a value that will vary from board to board.  It is a value
	 * that could vary between different SoC models if they had massively
	 * different output clock delays inside their dw_mmc IP block (delay_o),
	 * but since it's OK to overshoot a little we don't need to do complex
	 * calculations and can pick values that will just work for everyone.
	 *
	 * When picking values we'll stick with picking 0/90/180/270 since
	 * those can be made very accurately on all known Rockchip SoCs.
	 *
	 * Note that these values match values from the DesignWare Databook
	 * tables for the most part except for SDR12 and "ID mode".  For those
	 * two modes the databook calculations assume a clock in of 50MHz.  As
	 * seen above, we always use a clock in rate that is exactly the
	 * card's input clock (times RK3288_CLKGEN_DIV, but that gets divided
	 * back out before the controller sees it).
	 *
	 * From measurement of a single device, it appears that delay_o is
	 * about .5 ns.  Since we try to leave a bit of margin, it's expected
	 * that numbers here will be fine even with much larger delay_o
	 * (the 1.4 ns assumed by the DesignWare Databook would result in the
	 * same results, for instance).
	 */

	/*
	 * In almost all cases a 90 degree phase offset will provide
	 * sufficient hold times across all valid input clock rates
	 * assuming delay_o is not absurd for a given SoC.  We'll use
	 * that as a default.
	 */
	phase = 4;

	switch (ios->timing) {
	case MMC_TIMING_MMC_DDR52:
		/*
		 * Since clock in rate with MMC_DDR52 is doubled when
		 * bus width is 8 we need to double the phase offset
		 * to get the same timings.
		 */
		if (ios->bus_width == MMC_BUS_WIDTH_8)
			phase = 8;
		break;
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_MMC_HS200:
		/*
		 * In the case of 150 MHz clock (typical max for
		 * HobotRobtics SoCs), 90 degree offset will add a delay
		 * of 1.67 ns.  That will meet min hold time of .8 ns
		 * as long as clock output delay is < .87 ns.  On
		 * SoCs measured this seems to be OK, but it doesn't
		 * hurt to give margin here, so we use 180.
		 */
		phase = 4;
		break;
	}

	hb_mmc_set_drv_phase(host, phase);
	pr_debug("%s ctrl_id=%d sample_phase=%d phase=%d"
		" ios->timing=%d ios->signal_voltage=%d ios->clock=%d"
		" cclkin=%d bus_hz=%lu host->bus_hz=%d\n",
		__func__, priv->ctrl_id, priv->default_sample_phase,
		phase, ios->timing, ios->signal_voltage, ios->clock,
		cclkin, bus_hz, host->bus_hz);
	return;
}

static int dw_mci_hb_prepare_hs400_tuning(struct dw_mci *host,
					  struct mmc_ios *ios)
{
	return 0;
}

static int dw_mci_hb_parse_dt(struct dw_mci *host)
{
	struct device_node *np = host->dev->of_node;
	struct dw_mci_hobot_priv_data *priv;
	unsigned int of_val = 0;
#ifdef CONFIG_MMC_TUNE_FROM_UBOOT
	const char *of_tune_middle;
#endif /*CONFIG_MMC_TUNE_FROM_UBOOT*/


	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ctrl_id = of_alias_get_id(host->dev->of_node, "mmc");
	if (priv->ctrl_id > MMC_MAX_ID)
		return -EINVAL;
	priv->not_mmc = false;
	priv->not_mmc |= device_property_read_bool(host->dev, "no-mmc");

	if (of_property_read_u32(np, "hobot,default-sample-phase",
				 &priv->default_sample_phase))
		priv->default_sample_phase = 0;

	priv->sysctrl_reg = ioremap(HOBOT_SYSCTRL_REG, 0x400);
	if (IS_ERR(priv->sysctrl_reg))
		return PTR_ERR(priv->sysctrl_reg);

	priv->padcctrl_reg = ioremap(HOBOT_PDACCTRL_REG, 0x200);
	if (IS_ERR(priv->padcctrl_reg))
		return PTR_ERR(priv->padcctrl_reg);

	priv->uhs_180v_gpio = 0;
	priv->uhs_180v_logic = 0;
	priv->powerup_gpio = 0;
	priv->powerup_logic = 0;
	priv->mmc_fixed_voltage = 0;

	of_val = 0;
	if (!device_property_read_u32(host->dev, "clk_ctrl_off", &of_val)) {
		if (of_val)
			priv->clk_ctrl_reg = priv->sysctrl_reg + of_val;
		else
			priv->clk_ctrl_reg = NULL;
	}
	of_val = 0;
	if (!device_property_read_u32(host->dev, "powerup-logic", &of_val)) {
		if (of_val)
			priv->powerup_logic = 1;
		else
			priv->powerup_logic = 0;
	}
	of_val = 0;
	if (!device_property_read_u32(host->dev, "powerup-gpio", &of_val)) {
		priv->powerup_gpio = of_val;
		if(gpio_request(priv->powerup_gpio, "sd-power"))
			dev_err(host->dev, "request sd-power gpio failed!\n");

		if (priv->not_mmc) {
			hb_mmc_set_power(priv, 0);
			hb_mmc_set_power(priv, 1);
		}
	}

	of_val = 0;
	if (!device_property_read_u32(host->dev, "uhs-180v-logic", &of_val)) {
		if (of_val)
			priv->uhs_180v_logic = 1;
		else
			priv->uhs_180v_logic = 0;
	}
	of_val = 0;
	if (!device_property_read_u32(host->dev, "uhs-180v-gpio", &of_val)) {
		priv->uhs_180v_gpio = of_val;
		if (gpio_request(priv->uhs_180v_gpio, NULL))
			dev_err(host->dev, "request uhs-180v-gpio failed!\n");
	}

	if (!device_property_read_u32
	    (host->dev, "mmc-fixed-voltage", &priv->mmc_fixed_voltage)) {
		dev_info(host->dev, "mmc set to fixed voltage %d\n", priv->mmc_fixed_voltage);
	}

    priv->pin_ctrl = devm_pinctrl_get(host->dev);
    if (IS_ERR(priv->pin_ctrl)) {
        dev_err(host->dev, "pin_ctrl not found!\n");
		return -ENODEV;
    }

    priv->pin_state_1_8v = pinctrl_lookup_state(priv->pin_ctrl, "voltage_1_8v");
    if (IS_ERR(priv->pin_state_1_8v)) {
        priv->pin_state_1_8v = NULL;
    }

	priv->pin_state_3_3v = pinctrl_lookup_state(priv->pin_ctrl, "voltage_3_3v");
    if (IS_ERR(priv->pin_state_3_3v)) {
        priv->pin_state_3_3v = NULL;
    }

#ifdef CONFIG_MMC_TUNE_FROM_UBOOT
	/*The device tree is analyzed to obtain the tuning phase passed in the U-Boot*/
	if(device_property_read_string(host->dev,
			"uboot-tuning-middle-phase", &of_tune_middle)) {
		dev_info(host->dev, "cannot get uboot tune data.\n");
		priv->is_get_tune = MMC_UBOOT_GET_TUNE_ERR;
	} else {
		priv->is_get_tune = MMC_UBOOT_GET_TUNE_OK;
		priv->uboot_tune_phase = simple_strtoul(of_tune_middle, NULL, 16);
	}
#endif /* CONFIG_MMC_TUNE_FROM_UBOOT */

	host->priv = priv;

	return 0;
}

static int dw_mci_hobot_init(struct dw_mci *host)
{
	struct dw_mci_hobot_priv_data *priv = host->priv;

	mci_writel(host, CDTHRCTL, SDMMC_SET_THLD(SDCARD_RD_THRESHOLD,
						  SDMMC_CARD_RD_THR_EN));
	hb_mmc_set_sd_padcctrl(priv);

	return 0;
}

static int dw_mci_set_sel18(struct dw_mci *host, bool val)
{
	struct dw_mci_hobot_priv_data *priv;
	int ret = 0;
	bool logic_val = val;

	priv = host->priv;

	if (priv->uhs_180v_logic) {
		if (val)
			logic_val = 0;
		else
			logic_val = 1;
	}

	dev_dbg(host->dev, "ctrl_id=%d uhs_180v_gpio=%d val=%d lval=%d\n",
			priv->ctrl_id, priv->uhs_180v_gpio, val, logic_val);

	if (priv->uhs_180v_gpio) {
		if (val) {
			ret = gpio_direction_output(priv->uhs_180v_gpio, logic_val);
			if (priv->pin_ctrl && priv->pin_state_1_8v) {
				dev_dbg(host->dev, "set pinctrl-voltage to 1.8v");
				ret |= pinctrl_select_state(priv->pin_ctrl, priv->pin_state_1_8v);
			}
		} else {
			if (priv->pin_ctrl && priv->pin_state_3_3v) {
				dev_dbg(host->dev, "set pinctrl-voltage to 3.3v");
				ret = pinctrl_select_state(priv->pin_ctrl, priv->pin_state_3_3v);
			}
			ret |= gpio_direction_output(priv->uhs_180v_gpio, logic_val);
		}

		usleep_range(5000, 10000);
	} else if (priv->mmc_fixed_voltage != 0) {
		if (val) {
			if (priv->pin_ctrl && priv->pin_state_1_8v) {
				dev_dbg(host->dev, "set pinctrl-voltage to 1.8v");
				ret |= pinctrl_select_state(priv->pin_ctrl, priv->pin_state_1_8v);
			}
		} else {
			if (priv->pin_ctrl && priv->pin_state_3_3v) {
				dev_dbg(host->dev, "set pinctrl-voltage to 3.3v");
				ret = pinctrl_select_state(priv->pin_ctrl, priv->pin_state_3_3v);
			}
		}

		usleep_range(5000, 10000);
	}

	if (ret) {
		dev_err(host->dev, "sel18 %u error\n", val);
		return ret;
	}

	return 0;
}

static int dw_mci_hb_switch_voltage(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	struct dw_mci_hobot_priv_data *priv = host->priv;
	u32 v18 = SDMMC_UHS_18V << slot->id;
	int ret = 0;
	u32 uhs;

	if (!priv)
		return 0;

	if (priv->not_mmc) {
		if (priv->mmc_fixed_voltage == 3300 ||
			ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
			ios->signal_voltage = MMC_SIGNAL_VOLTAGE_330;
			ret = dw_mci_set_sel18(host, 0);
		} else if (priv->mmc_fixed_voltage == 1800 ||
				   ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
			ios->signal_voltage = MMC_SIGNAL_VOLTAGE_180;
			ret = dw_mci_set_sel18(host, 1);
		} else {
			dev_dbg(host->dev, "voltage not supported\n");
			return -EINVAL;
		}
	} else {
		ios->signal_voltage = MMC_SIGNAL_VOLTAGE_180;
	}

	/*
	 * Program the voltage.  Note that some instances of dw_mmc may use
	 * the UHS_REG for this.  For other instances (like exynos) the UHS_REG
	 * does no harm but you need to set the regulator directly.  Try both.
	 */
	uhs = mci_readl(host, UHS_REG);
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		uhs &= ~v18;
	else
		uhs |= v18;

	if (!IS_ERR(mmc->supply.vqmmc)) {
		ret = mmc_regulator_set_vqmmc(mmc, ios);

		if (ret) {
			dev_dbg(&mmc->class_dev,
				"Regulator set error %d - %s V\n",
				ret, uhs & v18 ? "1.8" : "3.3");
			return ret;
		}
	}

	mci_writel(host, UHS_REG, uhs);
	mci_writel(host, UHS_REG_EXT, 0);

	return ret;
}

/* Common capabilities of X2 SoC */
static unsigned long dw_mci_hb_dwmmc_caps[4] = {
	MMC_CAP_CMD23,
	MMC_CAP_CMD23,
	MMC_CAP_CMD23,
	MMC_CAP_CMD23,
};

static const struct dw_mci_drv_data hb_drv_data = {
	.caps = dw_mci_hb_dwmmc_caps,
	.num_caps = ARRAY_SIZE(dw_mci_hb_dwmmc_caps),
	.init = dw_mci_hobot_init,
	.set_ios = dw_mci_hb_set_ios,
	.parse_dt = dw_mci_hb_parse_dt,
	.execute_tuning = dw_mci_hb_execute_tuning,
	.prepare_hs400_tuning = dw_mci_hb_prepare_hs400_tuning,
	.switch_voltage = dw_mci_hb_switch_voltage,
};

static const struct of_device_id dw_mci_hobot_match[] = {
	{.compatible = "hobot,x2-dw-mshc", .data = &hb_drv_data},
	{.compatible = "hobot,hb-dw-mmc", .data = &hb_drv_data},
	{},
};

MODULE_DEVICE_TABLE(of, dw_mci_hobot_match);

static int dw_mci_hobot_probe(struct platform_device *pdev)
{
	const struct dw_mci_drv_data *drv_data;
	const struct of_device_id *match;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	dev_info(&pdev->dev, "ver: %s\n", VER);

	match = of_match_node(dw_mci_hobot_match, pdev->dev.of_node);
	if (match == NULL) {
		dev_err(&pdev->dev, "of_match_node returned NULL!\n");
		return -ENODEV;
	}

	drv_data = match->data;

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);

	ret = dw_mci_pltfm_register(pdev, drv_data);
	if (ret) {
		pm_runtime_disable(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
		pm_runtime_put_noidle(&pdev->dev);
		return ret;
	}

	pm_runtime_put_autosuspend(&pdev->dev);

	return 0;
}

static int dw_mci_hobot_remove(struct platform_device *pdev)
{
	struct dw_mci *host;
	struct dw_mci_hobot_priv_data *priv;

	dev_info(&pdev->dev, "remove\n");

	host = platform_get_drvdata(pdev);
	priv = host->priv;

	if (priv->padcctrl_reg)
		iounmap(priv->padcctrl_reg);
	if (priv->sysctrl_reg)
		iounmap(priv->sysctrl_reg);
	if (priv->uhs_180v_gpio)
		gpio_free(priv->uhs_180v_gpio);

	if (priv->powerup_gpio) {
		hb_mmc_set_power(priv, 0);
		gpio_free(priv->powerup_gpio);
	}

	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	return dw_mci_pltfm_remove(pdev);
}

static const struct dev_pm_ops dw_mci_hobot_pmops = {
#ifdef CONFIG_HOBOT_XJ2
	SET_SYSTEM_SLEEP_PM_OPS(dw_mci_system_suspend,
				dw_mci_system_resume)
#else
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
#endif
	SET_RUNTIME_PM_OPS(dw_mci_runtime_suspend,
			   dw_mci_runtime_resume,
			   NULL)
};

static struct platform_driver dw_mci_hobot_pltfm_driver = {
	.probe = dw_mci_hobot_probe,
	.remove = dw_mci_hobot_remove,
	.driver = {
		   .name = "dwmmc_hobot",
		   .of_match_table = dw_mci_hobot_match,
		   .pm = &dw_mci_hobot_pmops,
		   },
};

module_platform_driver(dw_mci_hobot_pltfm_driver);

MODULE_AUTHOR("Horizon Robotics, Ltd");
MODULE_DESCRIPTION("Horizon Robotics Specific DW-MSHC Driver Extension");
MODULE_ALIAS("platform: dwmmc_hobot");
MODULE_LICENSE("GPL v2");
