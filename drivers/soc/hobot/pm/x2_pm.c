/*
 *
 *   Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */
#include <linux/platform_device.h>
#include <linux/genalloc.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/compiler.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <asm-generic/cacheflush.h>
#include <asm/suspend.h>
#include <asm/cpu_ops.h>
#include <linux/delay.h>
#include "x2_pm.h"

struct x2_suspend_data {
	void __iomem *pmu;
	void __iomem *ddrc;
	void __iomem *ddrp;
	void __iomem *sysctl;
	void __iomem *padc;
};

struct x2_suspend {
	struct device *dev;
	unsigned int wakeup_src_mask;
	unsigned int sleep_period;
	int wakeup_count;
	struct x2_suspend_data data;
};

/* global variable define */
static struct x2_suspend *x2_suspend;

static void (*x2_suspend_sram_fn)(struct x2_suspend_data *);
extern void x2_resume(void);
extern void x2_suspend_in_sram(struct x2_suspend_data *suspend_data);
extern u32 x2_suspend_in_sram_sz;

static void x2_core1_resume(void)
{
	cpu_ops[1]->cpu_prepare(1);
	x2_suspend->wakeup_count++;
}

struct syscore_ops x2_core1_syscore_ops = {
	.resume         = x2_core1_resume,
};

static ssize_t wakeup_cnt_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return snprintf(buf, sizeof(x2_suspend->wakeup_count),
			"%d\n", x2_suspend->wakeup_count);
}

static ssize_t sleep_period_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t len)
{
	int ret;
	unsigned int input;

	ret = kstrtou32(buf, 10, &input);
	if (ret)
		return ret;

	if (input != 0) {
		x2_suspend->wakeup_src_mask =
			~(WAKEUP_SRC_PADC_EXT | WAKEUP_SRC_RTC) & 0xff;
		x2_suspend->sleep_period = input * CLK_HZ & ~SLEEP_PERIOD;
	} else {
		x2_suspend->wakeup_src_mask = ~WAKEUP_SRC_PADC_EXT & 0xff;
		x2_suspend->sleep_period = SLEEP_PERIOD;
	}

	return len;
}

static DEVICE_ATTR_WO(sleep_period);
static DEVICE_ATTR_RO(wakeup_cnt);

static int x2_suspend_valid_state(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_MEM:
		return 1;
	default:
		return 0;
	}
}

static int x2_suspend_begin(suspend_state_t state)
{
	return 0;
}

extern void *__asm_flush_dcache_all(void);

static int x2_suspend_finish(unsigned long val)
{
	__asm_flush_dcache_all();
	x2_suspend_sram_fn(&x2_suspend->data);

	return 0;
}

static int x2_suspend_enter(suspend_state_t state)
{
	void *padc, *pmu;
	unsigned int val;

	padc = x2_suspend->data.padc;
	pmu = x2_suspend->data.pmu;

	val = readl(padc + WAKEUP_PIN_REG);
	val &= ~WAKEUP_PIN_CFG;
	writel(val, padc + WAKEUP_PIN_REG);

	writel(x2_suspend->wakeup_src_mask, pmu + X2_PMU_W_SRC_MASK);
	writel(x2_suspend->sleep_period, pmu + X2_PMU_SLEEP_PERIOD);
	writel(0, pmu + X2_PMU_OUTPUT_CTRL);
	writel(SLEEP_TRIG, pmu + X2_PMU_SLEEP_CMD);

	val = virt_to_phys((void *)cpu_resume);
	writel(val, pmu + X2_PMU_SW_REG_03);

	memcpy(x2_suspend_sram_fn, x2_suspend_in_sram, x2_suspend_in_sram_sz);
	flush_icache_range((unsigned long)x2_suspend_sram_fn,
		(unsigned long)x2_suspend_sram_fn + x2_suspend_in_sram_sz);
	isb();
	return cpu_suspend(0, x2_suspend_finish);
}

static void x2_suspend_end(void)
{

}

static const struct platform_suspend_ops x2_suspend_ops = {
	.valid  = x2_suspend_valid_state,
	.begin  = x2_suspend_begin,
	.enter  = x2_suspend_enter,
	.end    = x2_suspend_end,
};

static void *sram_init(struct platform_device *pdev)
{
	struct resource *res;
	size_t size;
	struct page **pages;
	phys_addr_t start, page_start;
	unsigned int page_count, i;
	pgprot_t prot;
	void *vaddr;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "found no memory resource\n");
		return NULL;
	}

	size = resource_size(res);
	start = res->start;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);
	prot = __pgprot((PROT_NORMAL_WT & ~PTE_PXN) | PTE_CONT);
	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		dev_err(x2_suspend->dev,
			"Failed to allocate array for %u pages\n", page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}

	vaddr = vm_map_ram(pages, page_count, -1, prot);
	if (vaddr == NULL)
		dev_err(x2_suspend->dev, "vm_map_ram failed\n");

	x2_suspend_sram_fn = vaddr;
	kfree(pages);

	return vaddr;
}

static int x2_suspend_probe(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	struct x2_suspend_data *data = NULL;

	dev_info(&pdev->dev, "[%s] is start !\n", __func__);

	x2_suspend = devm_kmalloc(&pdev->dev,
			sizeof(struct x2_suspend), GFP_KERNEL);
	if (!x2_suspend)
		goto err_out;

	x2_suspend->dev = &pdev->dev;
	data = &x2_suspend->data;

	data->pmu = ioremap(X2_PMU_BASE_PA, X2_PMU_ADDR_SIZE);
	if (!data->pmu) {
		pr_err("%s:%s, PMU ioremap error\n", __FILE__, __func__);
		ret = -EIO;
		goto err_out1;
	}

	data->padc = ioremap(X2_PADC_BASE_PA, X2_PADC_ADDR_LEN);
	if (!data->padc) {
		pr_err("%s:%s, PADC ioremap error\n", __FILE__, __func__);
		ret = -EIO;
		goto err_out1;
	}

	data->ddrc = ioremap(DDRC_BASE_PA, DDRC_ADDR_SIZE);
	if (!data->ddrc) {
		pr_err("%s:%s, DDRC ioremap error\n", __FILE__, __func__);
		ret = -EIO;
		goto err_out1;
	}

	data->ddrp = ioremap(DDRP_BASE_PA, DDRP_ADDR_SIZE);
	if (!data->ddrp) {
		pr_err("%s:%s, DDRP ioremap error\n", __FILE__, __func__);
		ret = -EIO;
		goto err_out1;
	}

	data->sysctl = ioremap(SYSCTRL_BASE_PA, SYSCTRL_ADDR_SIZE);
	if (!data->sysctl) {
		pr_err("%s:%s, SYSCTRL ioremap error\n", __FILE__, __func__);
		ret = -EIO;
		goto err_out1;
	}

	sram_init(pdev);

	if (x2_suspend_sram_fn)
		suspend_set_ops(&x2_suspend_ops);
	else
		pr_info("PM not supported, due to no SRAM allocated\n");

	x2_suspend->wakeup_src_mask = ~WAKEUP_SRC_PADC_EXT & 0xff;
	x2_suspend->sleep_period = SLEEP_PERIOD;

	platform_set_drvdata(pdev, x2_suspend);

	device_create_file(&pdev->dev, &dev_attr_sleep_period);
	device_create_file(&pdev->dev, &dev_attr_wakeup_cnt);

	register_syscore_ops(&x2_core1_syscore_ops);

	dev_info(&pdev->dev, "[%s] is end !\n", __func__);
	return 0;

err_out1:
	devm_kfree(&pdev->dev, x2_suspend);
err_out:
	return ret;
}

static int x2_suspend_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "[%s] is remove!\n", __func__);
	devm_kfree(&pdev->dev, x2_suspend);
	x2_suspend = NULL;
	return 0;
}

static const struct of_device_id x2_suspend_match[] = {
	{.compatible = "hobot,x2-suspend"},
	{}
};

MODULE_DEVICE_TABLE(of, x2_suspend_match);

static struct platform_driver x2_suspend_driver = {
	.probe = x2_suspend_probe,
	.remove = x2_suspend_remove,
	.driver = {
		   .name = "x2-suspend",
		   .of_match_table = x2_suspend_match,
		   },
};

module_platform_driver(x2_suspend_driver);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("X2 suspend module");
MODULE_LICENSE("GPL");
