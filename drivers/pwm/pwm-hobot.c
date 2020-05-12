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

#define pr_fmt(fmt) "hobot-pwm: " fmt

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>

/* the offset of pwm registers */
#define PWM_EN             0x00
#define PWM_TIME_SLICE     0x04
#define PWM_FREQ           0x08
#define PWM_FREQ1          0x0C
#define PWM_RATIO          0x14
#define PWM_SRCPND         0x1C
#define PWM_INTMASK        0x20
#define PWM_SETMASK        0x24
#define PWM_UNMASK         0x28

#define PWM_INT_EN         (1U<<3)
#define PWM_NPWM           3         /* number of channels per pwm chip(controller) */
#define PWM_CLK            192000000
#define PWM_NAME           "hobot-pwm"

struct hobot_pwm_chip {
	struct pwm_chip chip;
	struct clk *mclk;
	int irq;
	char name[8];
	void __iomem *base;
};

#define PWM_ENABLE	BIT(31)
#define PWM_PIN_LEVEL	BIT(30)

#define to_hobot_pwm_chip(_chip) \
	container_of(_chip, struct hobot_pwm_chip, chip)

/* IO accessors */
static inline u32 hobot_pwm_rd(struct hobot_pwm_chip *hobot_chip, u32 reg)
{
	return ioread32(hobot_chip->base + reg);
}

static inline void hobot_pwm_wr(struct hobot_pwm_chip *hobot_chip, u32 reg, u32 value)
{
	iowrite32(value, hobot_chip->base + reg);
}

static irqreturn_t hobot_pwm_irq_handler(int irq, void *data)
{
	u32 status = 0;
	struct hobot_pwm_chip *hbpwm = (struct hobot_pwm_chip *)data;

	status = hobot_pwm_rd(hbpwm, PWM_SRCPND);
	hobot_pwm_wr(hbpwm, PWM_SRCPND, status);
	//dev_info(hbpwm->chip.dev, "pwm_irq_handler\n");

	return IRQ_HANDLED;
}

static int hobot_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm, int duty_ns, int period_ns)
{
	u32 val, reg, offset;
	int pwm_freq, pwm_ratio;
	struct hobot_pwm_chip *hbpwm = to_hobot_pwm_chip(chip);

	/* config pwm freq */
	pwm_freq = div64_u64((uint64_t)PWM_CLK * (uint64_t)period_ns,
			(unsigned long long)NSEC_PER_SEC);
	if(0xFFF < pwm_freq) {
		return -ERANGE;
	}
	reg = pwm->hwpwm == 2 ? PWM_FREQ1 : PWM_FREQ;
	offset = (pwm->hwpwm % 2) * 16;
	val = hobot_pwm_rd(hbpwm, reg);
	val &= ~(0xFFF<<offset);
	val |= pwm_freq<<offset;
	hobot_pwm_wr(hbpwm, reg, val);

	/* config pwm duty */
	pwm_ratio = div64_u64((unsigned long long)duty_ns * 256, period_ns);
	val = hobot_pwm_rd(hbpwm, PWM_RATIO);
	offset = pwm->hwpwm * 8;
	val &= ~(0xFF<<offset);
	val |= pwm_ratio << offset;
	hobot_pwm_wr(hbpwm, PWM_RATIO, val);

	return 0;
}

static int hobot_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	int ret;
	struct hobot_pwm_chip *hbpwm = to_hobot_pwm_chip(chip);

	val = hobot_pwm_rd(hbpwm, PWM_EN);
	val |= (1<<pwm->hwpwm);
	val |= PWM_INT_EN;
	hobot_pwm_wr(hbpwm, PWM_EN, val);

	return 0;
}

static void hobot_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	struct hobot_pwm_chip *hbpwm = to_hobot_pwm_chip(chip);

	val = hobot_pwm_rd(hbpwm, PWM_EN);
	val &= (~(1<<pwm->hwpwm));
	if (!(val & 0x7))
		val &= (~PWM_INT_EN);
	hobot_pwm_wr(hbpwm, PWM_EN, val);

	return;
}

static int hobot_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	int ret;
	struct hobot_pwm_chip *hbpwm = to_hobot_pwm_chip(chip);

	if (hbpwm->mclk == NULL)
		return -1;
	if (!__clk_is_enabled(hbpwm->mclk)) {
		ret = clk_prepare_enable(hbpwm->mclk);
		if (ret) {
			pr_err("failed to enable pwm clock\n");
			return ret;
		}
	}
	return 0;
}

static void hobot_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct hobot_pwm_chip *hbpwm = to_hobot_pwm_chip(chip);

	if (hbpwm->mclk == NULL)
		return;
	clk_disable_unprepare(hbpwm->mclk);
}

static const struct pwm_ops hobot_pwm_ops = {
	.config  = hobot_pwm_config,
	.enable  = hobot_pwm_enable,
	.disable = hobot_pwm_disable,
	.request = hobot_pwm_request,
	.free = hobot_pwm_free,
	.owner   = THIS_MODULE,
};

static int hobot_pwm_probe(struct platform_device *pdev)
{
	int ret, id;
	struct hobot_pwm_chip *hbpwm;
	struct resource *res;
	struct device_node *node = pdev->dev.of_node;

	hbpwm = devm_kzalloc(&pdev->dev, sizeof(struct hobot_pwm_chip), GFP_KERNEL);
	if (!hbpwm)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hbpwm->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hbpwm->base))
		return PTR_ERR(hbpwm->base);

	hbpwm->mclk = devm_clk_get(&pdev->dev, "pwm_mclk");
	if (IS_ERR(hbpwm->mclk) || (hbpwm->mclk == NULL)) {
		hbpwm->mclk = NULL;
		dev_err(&pdev->dev, "Can't get pwm mclk\n");
	}

	/* Look for a serialN alias */
	id = of_alias_get_id(node, "pwm");
	if (id < 0) {
		dev_err(&pdev->dev, "Get id of pwm:%d is err!\n", id);
		id = 0;
	}
	sprintf(hbpwm->name, "%s%d", PWM_NAME, id);

	hbpwm->irq = irq_of_parse_and_map(node, 0);
	if (0 == hbpwm->irq) {
		dev_err(&pdev->dev, "IRQ map failed!\n");
		return -EINVAL;
	} else {
		ret = request_irq(hbpwm->irq, hobot_pwm_irq_handler, IRQF_SHARED, hbpwm->name, hbpwm);
		if (ret) {
			dev_err(&pdev->dev, "unable to request IRQ %d\n", hbpwm->irq);
			return ret;
		} else {
			hobot_pwm_wr(hbpwm, PWM_SETMASK, 0);
			hobot_pwm_wr(hbpwm, PWM_UNMASK, 1);
		}
	}

	hbpwm->chip.dev  = &pdev->dev;
	hbpwm->chip.ops  = &hobot_pwm_ops;
	hbpwm->chip.npwm = PWM_NPWM;
	hbpwm->chip.base = -1;

	ret = pwmchip_add(&hbpwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip, error %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, hbpwm);
	pr_info("%s registered\n", hbpwm->name);

	return 0;
}

static int hobot_pwm_remove(struct platform_device *pdev)
{
	struct hobot_pwm_chip *hbpwm = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < hbpwm->chip.npwm; i++)
		pwm_disable(&hbpwm->chip.pwms[i]);

	return pwmchip_remove(&hbpwm->chip);
}

static const struct of_device_id hobot_pwm_dt_ids[] = {
	{ .compatible = "hobot,hobot-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hobot_pwm_dt_ids);

static struct platform_driver hobot_pwm_driver = {
	.driver = {
		.name = "hobot-pwm",
		.of_match_table = hobot_pwm_dt_ids,
	},
	.probe = hobot_pwm_probe,
	.remove = hobot_pwm_remove,
};
module_platform_driver(hobot_pwm_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("HOBOT PWM driver");
MODULE_LICENSE("GPL v2");
