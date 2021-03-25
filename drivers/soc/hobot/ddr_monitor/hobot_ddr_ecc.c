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

#include <linux/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <asm-generic/io.h>
#include <linux/moduleparam.h>

#include "./hobot_ddr_ecc.h"

unsigned int g_ecc_stat_rst = 0; // ecc statistical data reset
module_param(g_ecc_stat_rst, uint, 0644);

struct ddr_ecc_s {
	int irq;
	int enabled;
	void __iomem *ddrc_base;
	u64 corr_cnt;        // 1bit corrected error hit count
	u64 uncorr_cnt;      // 2bit uncorrected error hit count
	u64 uncorr_corr_cnt; // 3bit uncertain error hit count
	struct notifier_block panic_blk;
} ddr_ecc;


static irqreturn_t ddr_ecc_isr(int this_irq, void *data)
{
	void __iomem *ddrc_base = ddr_ecc.ddrc_base;
	void __iomem *ecc_ctrl = ddrc_base + uMCTL2_ECCCTL;
	u32 ecc_stat;
	u32 ecc_cnt;

	ecc_stat = readl(ddrc_base + uMCTL2_ECCSTAT);
	ecc_cnt  = readl(ddrc_base + uMCTL2_ECCERRCNT);

	if ((ecc_stat & ECC_STAT_CORR_ERR) &&
	     (ecc_stat & ECC_STAT_UNCORR_ERR)) {
		ddr_ecc.uncorr_corr_cnt += (ecc_cnt & 0xFFFF) + (ecc_cnt >> 16);
		pr_debug("3+ bits ecc error hits %llu\n", ddr_ecc.uncorr_corr_cnt);
	} else if (ecc_stat & ECC_STAT_CORR_ERR) {
		ddr_ecc.corr_cnt += ecc_cnt & 0xFFFF;
		pr_debug("1 bit ecc error hits %llu\n", ddr_ecc.corr_cnt);
	} else if (ecc_stat & ECC_STAT_UNCORR_ERR) {
		ddr_ecc.uncorr_cnt += ecc_cnt >> 16;
		pr_debug("2 bits ecc error hits %llu\n", ddr_ecc.uncorr_cnt);
	}

	writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_CNT_CLR, ecc_ctrl);
	writel(readl(ecc_ctrl) | ECC_CORR_ERR_CNT_CLR, ecc_ctrl);

	writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_CLR, ecc_ctrl);
	writel(readl(ecc_ctrl) | ECC_CORR_ERR_CLR, ecc_ctrl);


	return IRQ_HANDLED;
}

static int ddr_ecc_enable_irq(void *ddrc_base, int enable)
{
	u32 *ecc_ctrl;

	if (IS_ERR_OR_NULL(ddrc_base)) {
		pr_err("invalid ddrc base address\n");
		return -EINVAL;
	}

	ecc_ctrl = ddrc_base + uMCTL2_ECCCTL;
	if (enable) {
		writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_CLR, ecc_ctrl);
		writel(readl(ecc_ctrl) | ECC_CORR_ERR_CLR, ecc_ctrl);

		writel(readl(ecc_ctrl) | ECC_CORR_ERR_INTR_EN, ecc_ctrl);
		writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_INTR_EN, ecc_ctrl);
	} else {
		writel(readl(ecc_ctrl) & ~ECC_CORR_ERR_INTR_EN, ecc_ctrl);
		writel(readl(ecc_ctrl) & ~ECC_UNCORR_ERR_INTR_EN, ecc_ctrl);

		writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_CLR, ecc_ctrl);
		writel(readl(ecc_ctrl) | ECC_CORR_ERR_CLR, ecc_ctrl);
	}

	return 0;
}

static ssize_t ddr_ecc_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t len = 0;

	if (ddr_ecc.enabled == 0) {
		len += snprintf(buf+len, 64, "ddr ecc is not enabled\n");
	} else {
		if (g_ecc_stat_rst) {
			ddr_ecc.corr_cnt = 0;
			ddr_ecc.uncorr_cnt = 0;
			ddr_ecc.uncorr_corr_cnt = 0;
			g_ecc_stat_rst = 0;
		}
		len += snprintf(buf+len, 64, "ddr ecc is enabled\n");

		len += snprintf(buf+len, 64, "1 bit corrected error hits %llu times\n",
				ddr_ecc.corr_cnt);

		len += snprintf(buf+len, 64, "2 bits uncorrected error hits %llu times\n",
				ddr_ecc.uncorr_cnt);

		len += snprintf(buf+len, 64, "3+ bits uncertain error hits %llu times\n",
				ddr_ecc.uncorr_corr_cnt);
	}

	return len;
}

static DEVICE_ATTR_RO(ddr_ecc_stat);

static int ddr_ecc_panic_handler(struct notifier_block *this,
        unsigned long event, void *ptr)
{
	if (ddr_ecc.enabled != 0) {
		pr_err("ddr ecc is enabled.\n");

		pr_err("1 bit corrected error hits %llu times\n",
		               ddr_ecc.corr_cnt);
		pr_err("2 bits uncorrected error hits %llu times\n",
		               ddr_ecc.uncorr_cnt);
		pr_err("3+ bits uncertain error hits %llu times\n",
		               ddr_ecc.uncorr_corr_cnt);
	}

	return NOTIFY_DONE;
}


int hobot_ddr_ecc_init(struct platform_device *pdev)
{
	struct resource *pres;
	int ret = 0;

	/* support ECC start */
	pres = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pres) {
		pr_err("DDR controller base is not detected.\n");
		return -ENODEV;
	}

	ddr_ecc.ddrc_base = devm_ioremap_resource(&pdev->dev, pres);
	if (IS_ERR(ddr_ecc.ddrc_base)) {
		pr_err("%s DDRC base address map failed\n", __func__);
		return PTR_ERR(ddr_ecc.ddrc_base);
	}

	if (readl(ddr_ecc.ddrc_base + uMCTL2_ECCCFG0) & 0x7) {
		ddr_ecc.enabled = 1;
		ddr_ecc.irq = platform_get_irq(pdev, 1);
		if (ddr_ecc.irq < 0) {
			pr_info("ddr ecc irq is not in dts\n");
			return -ENODEV;
		}

		ret = request_irq(ddr_ecc.irq, ddr_ecc_isr, IRQF_TRIGGER_HIGH,
				  "ddr_ecc", &ddr_ecc);
		if (ret) {
			pr_err("Could not request ddr IRQ:%d\n", ddr_ecc.irq);
			return -ENODEV;
		}

		ddr_ecc.panic_blk.notifier_call = ddr_ecc_panic_handler;
		atomic_notifier_chain_register(&panic_notifier_list,
		                   &ddr_ecc.panic_blk);

		ddr_ecc_enable_irq(ddr_ecc.ddrc_base, 1);
	}

	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_ddr_ecc_stat.attr)) {
		pr_err("ddr_ecc_stat attr create failed\n");
		return -ENOMEM;
	}

	return ret;
}

int hobot_ddr_ecc_deinit(struct platform_device *pdev)
{
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_ddr_ecc_stat.attr);
	return 0;
}
