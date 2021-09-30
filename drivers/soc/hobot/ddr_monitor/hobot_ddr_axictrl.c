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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <asm-generic/io.h>

#include "./hobot_ddr_axictrl.h"

#define AXI_BUS_BASE 0xA4000038


struct ddr_axictrl {
	void __iomem *axibus_reg;
	struct mutex mlock;
	struct clk *sif_mclk;
	int sif_mclk_is_open;
} axictrl;

static ssize_t open_sif_mclk(void)
{
	if (axictrl.sif_mclk == NULL)
		return -1;

	axictrl.sif_mclk_is_open =
		__clk_is_enabled(axictrl.sif_mclk);
	if (axictrl.sif_mclk_is_open == 0) {
		clk_prepare_enable(axictrl.sif_mclk);
	}

	return 0;
}

static ssize_t close_sif_mclk(void)
{
	if (axictrl.sif_mclk == NULL)
		return -1;

	if (axictrl.sif_mclk_is_open == 0) {
		clk_disable_unprepare(axictrl.sif_mclk);
	}

	return 0;
}

u32 read_axibus_reg(void)
{
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();

	val = readl(axictrl.axibus_reg);

	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return val;
}

static ssize_t sifw_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 17;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();

	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t sifw_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 17;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "sif: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "sif: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

static ssize_t isp0m0_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 18;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t isp0m0_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 18;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "isp_0_m0: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "isp_0_m0: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

// isp_0_m1
static ssize_t isp0m1_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 19;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t isp0m1_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 19;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "isp_0_m1: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "isp_0_m1: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

// isp_0_m2
static ssize_t isp0m2_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	unsigned int val = 0;
	int shift = 20;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);
	return count;
}

static ssize_t isp0m2_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 20;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "isp_0_m2: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "isp_0_m2: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

// gdc_0
static ssize_t gdc0_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 24;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);

	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t gdc0_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 24;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "gdc_0: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "gdc_0: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

// t21
static ssize_t t21_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 25;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t t21_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 25;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "t21: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "t21: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

// gdc_1
static ssize_t gdc1_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 26;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);

	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t gdc1_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 26;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "gdc_1: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "gdc_1: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

// sifr
static ssize_t sifr_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 27;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t sifr_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 27;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "sifr: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "sifr: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

// ipu
static ssize_t ipu0_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 28;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t ipu0_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 28;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "ipu0: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "ipu0: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

// pym
static ssize_t pym_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 30;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t pym_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 30;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "pym: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "pym: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

// iar
static ssize_t iar_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 31;
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);

	open_sif_mclk();
	sscanf(buf, "%x", &val);
	if (val != 1 && val != 0) {
		pr_err("set value %d error,you should set 0~1\n", val);
		mutex_unlock(&axictrl.mlock);
		return count;
	}
	tmp = readl(axictrl.axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (val << shift);

	writel(tmp, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t iar_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 31;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += snprintf(buf + len, 64, "iar: vio1\n");
	} else {
		len += snprintf(buf + len, 64, "iar: vio0\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

static ssize_t all_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int val = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	sscanf(buf, "%x", &val);

	writel(val, axictrl.axibus_reg);
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return count;
}

static ssize_t all_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;

	mutex_lock(&axictrl.mlock);
	open_sif_mclk();
	tmp = readl(axictrl.axibus_reg);
	len = snprintf(buf, 64, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1<<17)) {    // sif bif17
		len += snprintf(buf + len, 64, "sifw: vio1\n");
	}
	if (tmp & (1<<18)) {    // bit18
		len += snprintf(buf + len, 64, "isp_0_m0: vio1\n");
	}
	if (tmp & (1<<19)) {    //  bit19
		len += snprintf(buf + len, 64, "isp_0_m1: vio1\n");
	}
	if (tmp & (1<<20)) {   // bit20
		len += snprintf(buf + len, 64, "isp_0_m2: vio1\n");
	}
	// if (tmp & (1<<21)) {
	// 	len += snprintf(buf + len, 64, "isp_1_m0: vio1\n");
	// }
	// if (tmp & (1<<22)) {
	// 	len += snprintf(buf + len, 64, "isp_1_m1: vio1\n");
	// }
	// if (tmp & (1<<23)) {
	// 	len += snprintf(buf + len, 64, "isp_1_m2: vio1\n");
	// }
	if (tmp & (1<<24)) {
		len += snprintf(buf + len, 64, "gdc0: vio1\n");
	}
	if (tmp & (1<<25)) {
		len += snprintf(buf + len, 64, "t21_0: vio1\n");
	}
	if (tmp & (1<<26)) {
		len += snprintf(buf + len, 64, "gdc_1: vio1\n");
	}
	if (tmp & (1<<27)) {
		len += snprintf(buf + len, 64, "sifr: vio1\n");
	}
	if (tmp & (1<<28)) {
		len += snprintf(buf + len, 64, "ipu_0: vio1\n");
	}
	// if (tmp & (1<<29)) {
	// 	len += snprintf(buf + len, 64, "ipu_1: vio1\n");
	// }
	if (tmp & (1<<30)) {
		len += snprintf(buf + len, 64, "pym: vio1\n");
	}
	if (tmp & (1<<31)) {
		len += snprintf(buf + len, 64, "iar: vio1\n");
	}
	close_sif_mclk();
	mutex_unlock(&axictrl.mlock);

	return len;
}

static struct driver_attribute sifw_axibus_ctrl    = __ATTR(sifw, 0664,
						       sifw_axibus_ctrl_show,
						       sifw_axibus_ctrl_store);
static struct driver_attribute isp0m0_axibus_ctrl    = __ATTR(isp0m0, 0664,
						       isp0m0_axibus_ctrl_show,
						       isp0m0_axibus_ctrl_store);
static struct driver_attribute isp0m1_axibus_ctrl    = __ATTR(isp0m1, 0664,
						       isp0m1_axibus_ctrl_show,
						       isp0m1_axibus_ctrl_store);
static struct driver_attribute isp0m2_axibus_ctrl    = __ATTR(isp0m2, 0664,
						       isp0m2_axibus_ctrl_show,
						       isp0m2_axibus_ctrl_store);

static struct driver_attribute gdc0_axibus_ctrl    = __ATTR(gdc0, 0664,
						       gdc0_axibus_ctrl_show,
						       gdc0_axibus_ctrl_store);
static struct driver_attribute t21_axibus_ctrl    = __ATTR(t21, 0664,
						       t21_axibus_ctrl_show,
						       t21_axibus_ctrl_store);
static struct driver_attribute gdc1_axibus_ctrl    = __ATTR(gdc1, 0664,
						       gdc1_axibus_ctrl_show,
						       gdc1_axibus_ctrl_store);
static struct driver_attribute ipu0_axibus_ctrl    = __ATTR(ipu0, 0664,
						       ipu0_axibus_ctrl_show,
						       ipu0_axibus_ctrl_store);
static struct driver_attribute sifr_axibus_ctrl    = __ATTR(sifr, 0664,
						       sifr_axibus_ctrl_show,
						       sifr_axibus_ctrl_store);
static struct driver_attribute pym_axibus_ctrl    = __ATTR(pym, 0664,
						       pym_axibus_ctrl_show,
						       pym_axibus_ctrl_store);
static struct driver_attribute iar_axibus_ctrl    = __ATTR(iar, 0664,
						       iar_axibus_ctrl_show,
						       iar_axibus_ctrl_store);

static struct driver_attribute all_axibus_ctrl    = __ATTR(all, 0664,
						       all_axibus_ctrl_show,
						       all_axibus_ctrl_store);

static struct attribute *axibus_ctrl_attrs[] = {
	&iar_axibus_ctrl.attr,
	&pym_axibus_ctrl.attr,
	&sifr_axibus_ctrl.attr,
	&ipu0_axibus_ctrl.attr,
	&gdc1_axibus_ctrl.attr,
	&t21_axibus_ctrl.attr,
	&gdc0_axibus_ctrl.attr,
	&isp0m2_axibus_ctrl.attr,
	&isp0m1_axibus_ctrl.attr,
	&isp0m0_axibus_ctrl.attr,
	&sifw_axibus_ctrl.attr,
	&all_axibus_ctrl.attr,
	NULL,
};
struct attribute_group axibus_group = {
	.name = "axibus_ctrl",
	.attrs = axibus_ctrl_attrs,
};


int hobot_ddr_axictrl_init(struct platform_device *pdev)
{
	mutex_init(&axictrl.mlock);
	axictrl.sif_mclk = devm_clk_get(&pdev->dev, "sif_mclk");
	if (IS_ERR(axictrl.sif_mclk)) {
		pr_err("failed to get sif_mclk");
		return -ENODEV;
	}

	axictrl.axibus_reg = devm_ioremap_nocache(&pdev->dev, AXI_BUS_BASE, 4);
	/* set ipu0 to vio1 by default */
	writel(0x10000000, axictrl.axibus_reg);

	return 0;
}

int hobot_ddr_axictrl_deinit(struct platform_device *pdev)
{
	return 0;
}
