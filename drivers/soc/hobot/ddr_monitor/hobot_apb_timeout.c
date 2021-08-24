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

#define pr_fmt(fmt) KBUILD_MODNAME ":" fmt

#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#define APB_TIMEOUT_CNT_ENABLE 0x00
#define APB_TIMEOUT_CNT_TGT    0x04
#define APB_TIMEOUT_PERR       0x08
#define APB_TIMEOUT_ERRADDR    0x0C
#define APB_TIMEOUT_SRCPND     0x10

struct apb_timeout_dev {
	void __iomem *reg_base;
	int err_mode; /* 0:irq, 1: data abort */
	int timeout;
	struct clk *pclk;
	u32 clk_hz;
	u32 cnt;
} apb_tmt;

static int timeout_ms = 20; /* default timeout in milliseconds */
static int timeout_ms_set(const char *val, const struct kernel_param *kp)
{
	int ret;
	u32 tgt_cnt;

	ret = param_set_int(val, kp);
	if (ret < 0)
		return ret;

	if (timeout_ms == 0)
		timeout_ms = 10;

	if (apb_tmt.reg_base == NULL)
		return ret;

	if ((timeout_ms / 1000)  > 0xffffffff / apb_tmt.clk_hz)
		timeout_ms = (0xffffffff / apb_tmt.clk_hz) * 1000;

	tgt_cnt = timeout_ms * (apb_tmt.clk_hz / 1000);

	writel(tgt_cnt, apb_tmt.reg_base + APB_TIMEOUT_CNT_TGT);
	apb_tmt.cnt = tgt_cnt;
	pr_debug("set apb timeout tgt_cnt:%u, timeout_ms:%d, clk_hz=%u\n",
		tgt_cnt, timeout_ms, apb_tmt.clk_hz);

	return ret;
}

static const struct kernel_param_ops timeout_ms_param_ops = {
	.set = timeout_ms_set,
	.get = param_get_int,
};

module_param_cb(timeout_ms, &timeout_ms_param_ops, &timeout_ms, 0644);

static int err_mode = 1; /* default error mode, 1: data abort, 0: no data abort */
static int err_mode_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret < 0)
		return ret;

	if (err_mode != 0)
		err_mode = 1;

	if (apb_tmt.reg_base == NULL)
		return ret;

	writel(err_mode, apb_tmt.reg_base + APB_TIMEOUT_PERR);

	apb_tmt.err_mode = err_mode;

	return ret;
}

static const struct kernel_param_ops err_mode_param_ops = {
	.set = err_mode_set,
	.get = param_get_int,
};

module_param_cb(err_mode, &err_mode_param_ops, &err_mode, 0644);

static int cnt_enable = 1; /* default error mode, 1: data abort, 0: no data abort */
static int cnt_enable_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret < 0)
		return ret;

	if (cnt_enable != 0)
		cnt_enable = 1;

	if (apb_tmt.reg_base == NULL)
		return ret;

	writel(cnt_enable, apb_tmt.reg_base + APB_TIMEOUT_CNT_ENABLE);

	return ret;
}

static const struct kernel_param_ops cnt_enable_param_ops = {
	.set = cnt_enable_set,
	.get = param_get_int,
};

module_param_cb(cnt_enable, &cnt_enable_param_ops, &cnt_enable, 0644);

static irqreturn_t apb_timeout_isr(int irq, void *data)
{
	struct apb_timeout_dev *apbtmt = (struct apb_timeout_dev *)data;

	writel(0x1, apbtmt->reg_base + APB_TIMEOUT_SRCPND);

	pr_err("!!!!!!!!!! APB bus timeout detected !!!!!!!!!");
	pr_err("PERRMODE: %s\n",
		readl(apbtmt->reg_base + APB_TIMEOUT_PERR) ? "irq" : "data abort");
	pr_err("PERRADDR: 0x%08x\n", readl(apbtmt->reg_base + APB_TIMEOUT_ERRADDR));

	pr_err("0xA7000000: 0x%08x APB_TIMEOUT_CNT_ENABLE  \n", readl(apb_tmt.reg_base + APB_TIMEOUT_CNT_ENABLE));
	pr_err("0xA7000004: 0x%08x APB_TIMEOUT_CNT_TGT     \n", readl(apb_tmt.reg_base + APB_TIMEOUT_CNT_TGT));
	pr_err("0xA7000008: 0x%08x APB_TIMEOUT_PERR        \n", readl(apb_tmt.reg_base + APB_TIMEOUT_PERR));
	pr_err("0xA700000C: 0x%08x APB_TIMEOUT_ERRADDR     \n", readl(apb_tmt.reg_base + APB_TIMEOUT_ERRADDR));
	pr_err("0xA7000010: 0x%08x APB_TIMEOUT_SRCPND      \n", readl(apb_tmt.reg_base + APB_TIMEOUT_SRCPND));

	return IRQ_HANDLED;
}

static int apb_timeout_probe(struct platform_device *pdev)
{
	struct resource *pres;
	int irq, ret;

	/* support MPU start */
	pres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pres) {
		pr_err("DDR controller base is not detected.\n");
		return -ENODEV;
	}

	apb_tmt.reg_base = devm_ioremap_resource(&pdev->dev, pres);
	if (IS_ERR(apb_tmt.reg_base)) {
		pr_err("apb timeout base address map failed, %ld\n",
			PTR_ERR(apb_tmt.reg_base));
		return PTR_ERR(apb_tmt.reg_base);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_info("apb timeout irq is not in dts\n");
		return -ENODEV;
	}

	ret = request_irq(irq, apb_timeout_isr, IRQF_TRIGGER_HIGH,
				  "apb_timeout", &apb_tmt);
	if (ret) {
		dev_err(&pdev->dev, "Could not request apb timeout irq %d\n",
				irq);
		return -ENODEV;
	}

	apb_tmt.pclk = devm_clk_get(&pdev->dev, "sys_pclk");

	if (IS_ERR(apb_tmt.pclk)) {
		pr_err("failed to get sys_pclk");
		return -ENODEV;
	}
	apb_tmt.clk_hz = clk_get_rate(apb_tmt.pclk);

	apb_tmt.cnt = timeout_ms * (apb_tmt.clk_hz / 1000);

	writel(apb_tmt.cnt, apb_tmt.reg_base + APB_TIMEOUT_CNT_TGT);

	writel(err_mode, apb_tmt.reg_base + APB_TIMEOUT_PERR);
	writel(cnt_enable, apb_tmt.reg_base + APB_TIMEOUT_CNT_ENABLE);

	pr_info("set apb timeout tgt_cnt:%u, timeout_ms:%d, clk_hz=%u\n",
		apb_tmt.cnt, timeout_ms, apb_tmt.clk_hz);

	return 0;
}

static int apb_timeout_remove(struct platform_device *pdev)
{
	return  0;
}

static const struct of_device_id apb_timeout_match[] = {
	{.compatible = "hobot,apb_timeout"},
	{}
};

MODULE_DEVICE_TABLE(of, apb_timeout_match);

#ifdef CONFIG_PM
static int apb_timeout_suspend(struct device *dev)
{
        return 0;
}

static int apb_timeout_resume(struct device *dev)
{
	writel(apb_tmt.cnt, apb_tmt.reg_base + APB_TIMEOUT_CNT_TGT);
	writel(err_mode, apb_tmt.reg_base + APB_TIMEOUT_PERR);
	writel(cnt_enable, apb_tmt.reg_base + APB_TIMEOUT_CNT_ENABLE);

	return 0;
}

static const struct dev_pm_ops apbtimeout_pm_ops = {
        SET_SYSTEM_SLEEP_PM_OPS(apb_timeout_suspend,
                                apb_timeout_resume)
};
#endif

static struct platform_driver apb_timeout_driver = {
	.probe	= apb_timeout_probe,
	.remove = apb_timeout_remove,
	.driver = {
		.name	= "apb_timeout",
		.of_match_table = apb_timeout_match,
#ifdef CONFIG_PM
		.pm	= &apbtimeout_pm_ops,
#endif
	},
};

module_platform_driver(apb_timeout_driver);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("apb timeout detection module");
MODULE_LICENSE("GPL");
