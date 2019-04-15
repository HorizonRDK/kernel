/*
 * Hisilicon SoC reset code
 *
 * Copyright (c) 2014 Hisilicon Ltd.
 * Copyright (c) 2014 Linaro Ltd.
 *
 * Author: Haojian Zhuang <haojian.zhuang@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <asm/proc-fns.h>

#define X2_REBOOT_OPT    0x00001001
static void __iomem *base;
static u32 reboot_offset;


static int x2_restart_handler(struct notifier_block *this,
				unsigned long mode, void *cmd)
{
	writel_relaxed(X2_REBOOT_OPT, base + reboot_offset);

	return NOTIFY_DONE;
}

static struct notifier_block x2_restart_nb = {
	.notifier_call = x2_restart_handler,
	.priority = 128,
};

static int x2_reboot_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int err;

	base = of_iomap(np, 0);
	if (!base) {
		WARN(1, "failed to map base address");
		return -ENODEV;
	}

	if (of_property_read_u32(np, "reboot-offset", &reboot_offset) < 0) {
		pr_err("failed to find reboot-offset property\n");
		iounmap(base);
		return -EINVAL;
	}

	err = register_restart_handler(&x2_restart_nb);
	if (err) {
		dev_err(&pdev->dev, "cannot register restart handler (err=%d)\n",
			err);
		iounmap(base);
	}

	return err;
}

static const struct of_device_id x2_reboot_of_match[] = {
	{ .compatible = "hobot,x2-power" },
	{}
};

static struct platform_driver x2_reboot_driver = {
	.probe = x2_reboot_probe,
	.driver = {
		.name = "x2-reboot",
		.of_match_table = x2_reboot_of_match,
	},
};
module_platform_driver(x2_reboot_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("X2 Reboot driver");
MODULE_LICENSE("GPL v2");
