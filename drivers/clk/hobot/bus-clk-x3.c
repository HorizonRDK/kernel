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

#include <linux/clk.h>
#include <asm/io.h>
#include <linux/notifier.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <soc/hobot/hobot_bus.h>
#include <linux/device.h>
#include "common.h"

#define TO_BUS(ptr, member) container_of(ptr, struct hobot_bus, member)
#define MHZ(x) ((x) * 1000000UL)

#define	SYSPLL_LOW_FREQ		240
#define	SYSPLL_NORMAL_FREQ	1500
#define	PERIPLL_LOW_FREQ	192
#define	PERIPLL_NORMAL_FREQ	1536

#define SYSCLK_SEL_BIT          (1 << 24)
#define PERICLK_SEL_BIT         (1 << 20)

#define SYSPLL_FREQ_CTRL                     (0x010)
#define SYSPLL_PD_CTRL                       (0x014)
#define SYSPLL_STATUS                        (0x018)

#define PERIPLL_FREQ_CTRL            	(0x050)
#define PERIPLL_PD_CTRL                      (0x054)
#define PERIPLL_STATUS                       (0x058)

#define PERISYS_DIV_SEL		(0x250)
#define SD0_CCLK_DIV		(0x320)
#define SD1_CCLK_DIV			(0x330)
#define SD2_CCLKDIV			(0x340)



#define UART_MCLK_DIV_SEL(x)            (((x) & 0xF) << 4)
#define SPI_MCLK_DIV_SEL(x)            (((x) & 0xF) << 8)
#define SD_MCLK_DIV_SEL(x)              (((x) & 0xF) << 4)

#define SYS_AP_ACLK_DIV_SEL(x)		(((x) & 0x7) << 16)
#define SYS_PCLK_DIV_SEL(x)		(((x) & 0x7) << 12)
#define SYS_NOC_ACLK_DIV_SEL(x)		(((x) & 0x7) << 8)

#define CPUBUS_DIV_SEL		(0x204)
#define PLLCLK_SEL		(0x300)
#define PD_BIT                          (1 << 0)
#define DSMPD_BIT                       (1 << 4)
#define FOUTPOST_DIV_BIT        (1 << 8)
#define FOUTVCO_BIT                     (1 << 12)
#define BYPASS_BIT                      (1 << 16)
#define LOCK_BIT                (1 << 0)

#define FBDIV_BITS(x)           ((x & 0xFFF) << 0)
#define REFDIV_BITS(x)          ((x & 0x3F) << 12)
#define POSTDIV1_BITS(x)        ((x & 0x7) << 20)
#define POSTDIV2_BITS(x)        ((x & 0x7) << 24)

extern void __iomem *clk_reg_base;
struct hobot_bus {
	struct device *dev;
	struct resource *res;
	void __iomem *sysctl;
	int old_state;
	int state;
	struct mutex lock;

	struct notifier_block   freq_policy;
};


static BLOCKING_NOTIFIER_HEAD(hb_bus_notifier_list);
static BLOCKING_NOTIFIER_HEAD(hb_usb_notifier_list);
static ATOMIC_NOTIFIER_HEAD(hb_console_notifier_list);

int hb_bus_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hb_bus_notifier_list, nb);
}
EXPORT_SYMBOL(hb_bus_register_client);

int hb_bus_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hb_bus_notifier_list, nb);
}
EXPORT_SYMBOL(hb_bus_unregister_client);

int hb_bus_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&hb_bus_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(hb_bus_notifier_call_chain);

int hb_usb_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hb_usb_notifier_list, nb);
}
EXPORT_SYMBOL(hb_usb_register_client);

int hb_usb_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hb_usb_notifier_list, nb);
}
EXPORT_SYMBOL(hb_usb_unregister_client);

int hb_usb_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&hb_usb_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(hb_usb_notifier_call_chain);

int hb_console_register_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&hb_console_notifier_list, nb);
}
EXPORT_SYMBOL(hb_console_register_client);

int hb_console_unregister_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&hb_console_notifier_list, nb);
}
EXPORT_SYMBOL(hb_console_unregister_client);

int hb_console_notifier_call_chain(unsigned long val, void *v)
{
	return atomic_notifier_call_chain(&hb_console_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(hb_console_notifier_call_chain);

void switch_peri_pll(void __iomem *base, ulong pll_val)
{
	unsigned int value;
	int try_num = 5;

	pr_debug("%s: base :%p, enter, pll_val:%lu"
			"clk_reg_base:%p\n", __func__,
				base, pll_val, clk_reg_base);
	value = readl(base + PLLCLK_SEL) & (~PERICLK_SEL_BIT);
	writel(value, base + PLLCLK_SEL);

	writel(PD_BIT | DSMPD_BIT | FOUTPOST_DIV_BIT | FOUTVCO_BIT,
			base + PERIPLL_PD_CTRL);

	switch (pll_val) {
		case MHZ(PERIPLL_LOW_FREQ):
			value = FBDIV_BITS(8) | REFDIV_BITS(1) |
				POSTDIV1_BITS(1) | POSTDIV2_BITS(1);
			break;
		case MHZ(PERIPLL_NORMAL_FREQ):
		default:
			value = FBDIV_BITS(64) | REFDIV_BITS(1) |
				POSTDIV1_BITS(1) | POSTDIV2_BITS(1);
			break;
	}

	writel(value, base + PERIPLL_FREQ_CTRL);
	value = readl(base + PERIPLL_FREQ_CTRL);

	value = readl(base + PERIPLL_PD_CTRL);
	value &= ~(PD_BIT | FOUTPOST_DIV_BIT);
	writel(value, base + PERIPLL_PD_CTRL);

	while (!(value = readl(base + PERIPLL_STATUS) & LOCK_BIT)) {
		if (try_num <= 0) {
			break;
		}
		udelay(100);
		try_num--;
	}

	value = readl(base + PLLCLK_SEL);
	value |= PERICLK_SEL_BIT;
	writel(value, base + PLLCLK_SEL);

	return;
}

void switch_sys_pll(void __iomem *base, ulong pll_val)
{
	unsigned int value;
	int try_num = 5;

	pr_debug("%s: base :%p, enter, pll_val:%lu \n", __func__, base, pll_val);
	value = readl(base + PLLCLK_SEL) & (~SYSCLK_SEL_BIT);
	writel(value, base + PLLCLK_SEL);

	writel(PD_BIT | DSMPD_BIT | FOUTPOST_DIV_BIT | FOUTVCO_BIT,
			base + SYSPLL_PD_CTRL);

	switch (pll_val) {
		case MHZ(SYSPLL_LOW_FREQ):
			value = FBDIV_BITS(10) | REFDIV_BITS(1) |
				POSTDIV1_BITS(1) | POSTDIV2_BITS(1);
			break;
		case MHZ(SYSPLL_NORMAL_FREQ):
		default:
			value = FBDIV_BITS(125) | REFDIV_BITS(1) |
				POSTDIV1_BITS(2) | POSTDIV2_BITS(1);
			break;
	}

	writel(value, base + SYSPLL_FREQ_CTRL);
	value = readl(base + SYSPLL_FREQ_CTRL);

	value = readl(base + SYSPLL_PD_CTRL);
	value &= ~(PD_BIT | FOUTPOST_DIV_BIT);
	writel(value, base + SYSPLL_PD_CTRL);

	while (!(value = readl(base + SYSPLL_STATUS) & LOCK_BIT)) {
		if (try_num <= 0) {
			break;
		}
		udelay(100);
		try_num--;
	}

	value = readl(base + PLLCLK_SEL);
	value |= SYSCLK_SEL_BIT;
	writel(value, base + PLLCLK_SEL);

	return;
}

static void switch_div(void __iomem *base, ulong pll_val)
{
	unsigned int value;

	if (pll_val == MHZ(PERIPLL_LOW_FREQ)) {
		value = readl(base + PERISYS_DIV_SEL);
		value &= ~UART_MCLK_DIV_SEL(0xf);
		value |=  UART_MCLK_DIV_SEL(0);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + PERISYS_DIV_SEL);

		value = readl(base + PERISYS_DIV_SEL);
		value &= ~SPI_MCLK_DIV_SEL(0xf);
		value |= SPI_MCLK_DIV_SEL(0);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + PERISYS_DIV_SEL);

		value = readl(base + CPUBUS_DIV_SEL);
		value &= ~SYS_AP_ACLK_DIV_SEL(0x7);
		value |= SYS_AP_ACLK_DIV_SEL(7);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + CPUBUS_DIV_SEL);

		value = readl(base + CPUBUS_DIV_SEL);
		value &= ~SYS_PCLK_DIV_SEL(0x7);
		value |= SYS_PCLK_DIV_SEL(3);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + CPUBUS_DIV_SEL);

		value = readl(base + CPUBUS_DIV_SEL);
		value &= ~SYS_NOC_ACLK_DIV_SEL(0x7);
		value |= SYS_NOC_ACLK_DIV_SEL(7);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + CPUBUS_DIV_SEL);

#if 0
		value = readl(base + SD0_CCLK_DIV);
		value &= ~SD_MCLK_DIV_SEL(0xf);
		value |= SD_MCLK_DIV_SEL(0);
		writel(value, base + SD0_CCLK_DIV);
#endif
	} else if (pll_val == MHZ(PERIPLL_NORMAL_FREQ)) {
		value = readl(base + PERISYS_DIV_SEL);
		value &= ~UART_MCLK_DIV_SEL(0xf);
		value |= UART_MCLK_DIV_SEL(7);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + PERISYS_DIV_SEL);

		value = readl(base + PERISYS_DIV_SEL);
		value &= ~SPI_MCLK_DIV_SEL(0xf);
		value |= SPI_MCLK_DIV_SEL(7);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + PERISYS_DIV_SEL);

		value = readl(base + CPUBUS_DIV_SEL);
		value &= ~SYS_AP_ACLK_DIV_SEL(0x7);
		value |= SYS_AP_ACLK_DIV_SEL(2);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + CPUBUS_DIV_SEL);

		value = readl(base + CPUBUS_DIV_SEL);
		value &= ~SYS_PCLK_DIV_SEL(0x7);
		value |= SYS_PCLK_DIV_SEL(4);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + CPUBUS_DIV_SEL);

		value = readl(base + CPUBUS_DIV_SEL);
		value &= ~SYS_NOC_ACLK_DIV_SEL(0x7);
		value |= SYS_NOC_ACLK_DIV_SEL(1);
		pr_debug("%s: %d,value:0x%x\n", __func__, __LINE__, value);
		writel(value, base + CPUBUS_DIV_SEL);

#if 0
		value = readl(base + SD0_CCLK_DIV);
		value &= ~SD_MCLK_DIV_SEL(0xf);
		value |= SD_MCLK_DIV_SEL(0x7);
		writel(value, base + SD0_CCLK_DIV);
#endif
	}
	udelay(10);
}
static int enter_powersave_mode(struct hobot_bus *bus)
{
	pr_debug("%s: enter\n", __func__);
	switch_sys_pll(bus->sysctl, MHZ(SYSPLL_LOW_FREQ));
	switch_peri_pll(bus->sysctl, MHZ(PERIPLL_LOW_FREQ));
	switch_div(bus->sysctl, MHZ(PERIPLL_LOW_FREQ));

	return 0;
}

static int exit_powersave_mode(struct hobot_bus *bus)
{
	pr_debug("%s: enter\n", __func__);
	switch_div(bus->sysctl, MHZ(PERIPLL_NORMAL_FREQ));
	switch_sys_pll(bus->sysctl, MHZ(SYSPLL_NORMAL_FREQ));
	switch_peri_pll(bus->sysctl, MHZ(PERIPLL_NORMAL_FREQ));

	return 0;
}

static int bus_change_state(struct hobot_bus *bus, int state)
{
	int ret = 0;

	if (bus->state == state) {
		pr_debug("already in state:%d\n", state);
		return ret;
	}

	if (state == POWERSAVE_STATE) {
		ret = enter_powersave_mode(bus);
	} else if (state == OTHER_STATE) {
		ret = exit_powersave_mode(bus);
	}

	bus->old_state = bus->state;
	bus->state = state;
	return ret;
}

static int cpu_governor_callback(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct hobot_bus *bus = TO_BUS(nb, freq_policy);
	struct cpufreq_policy *policy = data;

	switch (val) {
		case CPUFREQ_ADJUST:
			pr_debug("policy->governor: %s adjust\n", policy->governor->name);
			break;
		case CPUFREQ_NOTIFY:
			if (!strcmp(policy->governor->name, "powersave")
					&& (bus->state != POWERSAVE_STATE)) {
				pr_debug("policy->governor: %s notify\n", policy->governor->name);
				hb_usb_notifier_call_chain(POWERSAVE_STATE, NULL);
				hb_bus_notifier_call_chain(HB_BUS_SIGNAL_START, (void*)&bus->old_state);
				hb_console_notifier_call_chain(HB_BUS_SIGNAL_START, NULL);

				bus_change_state(bus, POWERSAVE_STATE);

				hb_console_notifier_call_chain(HB_BUS_SIGNAL_END, NULL);
				hb_bus_notifier_call_chain(HB_BUS_SIGNAL_END, (void*)&bus->old_state);
			} else if (strcmp(policy->governor->name, "powersave")
					&& (bus->state == POWERSAVE_STATE)){
				pr_debug("policy->governor: %s notify\n", policy->governor->name);
				hb_bus_notifier_call_chain(HB_BUS_SIGNAL_START, (void*)&bus->old_state);
				hb_console_notifier_call_chain(HB_BUS_SIGNAL_START, NULL);

				bus_change_state(bus, OTHER_STATE);

				hb_console_notifier_call_chain(HB_BUS_SIGNAL_END, NULL);
				hb_bus_notifier_call_chain(HB_BUS_SIGNAL_END, (void*)&bus->old_state);
				hb_usb_notifier_call_chain(OTHER_STATE, NULL);
			}
			break;
	}
	return 0;
}

static int hobot_bus_parse_of(struct platform_device *pdev,
		struct hobot_bus *bus)
{
	int ret = 0;
	struct resource *res;
	u32 reg_info[2];

	ret = of_property_read_u32_array(pdev->dev.of_node, "reg-info", reg_info,
			ARRAY_SIZE(reg_info));
	if (ret) {
		dev_err(bus->dev, "get reg_info failed");
		goto err;
	}

	res = devm_kmalloc(&pdev->dev, sizeof(struct resource), GFP_KERNEL);
	if (!res) {
		dev_err(bus->dev, "malloc resource failed");
		goto err;
	}

	bus->res = res;
	bus->res->start = reg_info[0];
	bus->res->end =  reg_info[0] + reg_info[1] - 1;
	bus->res->flags = IORESOURCE_MEM;
#if 0
	bus->sysctl = ioremap(bus->res->start, reg_info[1]);
	if (IS_ERR(bus->sysctl)) {
		dev_err(bus->dev, "ioremap platform resource failed");
		ret = PTR_ERR(bus->sysctl);
		goto err;
	}
#endif
	bus->sysctl = clk_reg_base;
err:
	return ret;
}

static int hobot_bus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hobot_bus *bus;
	int ret;

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

	mutex_init(&bus->lock);
	bus->old_state = POWERSAVE_STATE;
	bus->state = OTHER_STATE;

	bus->dev = &pdev->dev;
	platform_set_drvdata(pdev, bus);

	ret = hobot_bus_parse_of(pdev, bus);
	if (ret < 0)
		return ret;

	bus->freq_policy.notifier_call = cpu_governor_callback;

	ret = cpufreq_register_notifier(&bus->freq_policy,
			CPUFREQ_POLICY_NOTIFIER);

	pr_info("hobot-bus: new bus device registered:ret:%d\n", ret);

	return ret;
}

static const struct of_device_id hobot_bus_of_match[] = {
	{ .compatible = "hobot,hobot-bus", },
};

MODULE_DEVICE_TABLE(of, hobot_bus_of_match);

static struct platform_driver hobot_bus_platdrv = {
	.probe		= hobot_bus_probe,
	.driver = {
		.name	= "hobot-bus",
		.of_match_table = of_match_ptr(hobot_bus_of_match),
	},
};
module_platform_driver(hobot_bus_platdrv);

MODULE_DESCRIPTION("Generic Hobot Bus frequency driver");
MODULE_LICENSE("GPL v2");

