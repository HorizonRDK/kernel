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

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/reset.h>
#include <linux/pinctrl/consumer.h>
#include <linux/suspend.h>

#include "ips_hw_api.h"
#include "hobot_dev_ips.h"
#include "vio_config.h"

#define MODULE_NAME "X3 IPS"
#define REGISTER_CLK(name) {name, NULL}

ulong sif_mclk_freq = 0;
module_param(sif_mclk_freq, ulong, 0644);

EXPORT_SYMBOL(sif_mclk_freq);

struct x3_ips_dev *g_ips_dev;

struct vio_clk vio_clk_list[] = {
	REGISTER_CLK("sif_mclk"),
	REGISTER_CLK("mipi_rx0_ipi"),
	REGISTER_CLK("mipi_rx1_ipi"),
	REGISTER_CLK("mipi_rx2_ipi"),
	REGISTER_CLK("mipi_rx3_ipi"),
	REGISTER_CLK("mipi_tx_ipi"),
	REGISTER_CLK("mipi_cfg_host"),
	REGISTER_CLK("pym_mclk"),
	REGISTER_CLK("mipi_dev_ref"),
	REGISTER_CLK("mipi_host_ref"),
	REGISTER_CLK("sensor0_mclk"),
	REGISTER_CLK("sensor1_mclk"),
	REGISTER_CLK("sensor2_mclk"),
	REGISTER_CLK("sensor3_mclk"),
};

void ips_set_module_reset(unsigned long module)
{
	unsigned long flags;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	ips_module_reset(g_ips_dev->base_reg, module);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);
}
EXPORT_SYMBOL_GPL(ips_set_module_reset);

int ips_set_clk_ctrl(unsigned long module, bool enable)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	mutex_lock(&g_ips_dev->shared_mux);
	if (enable)
		vio_clk_enable("sif_mclk");
	ret = ips_clk_ctrl(g_ips_dev->base_reg, module, enable);
	if (!enable)
		vio_clk_disable("sif_mclk");
	mutex_unlock(&g_ips_dev->shared_mux);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_set_clk_ctrl);

int ips_set_bus_ctrl(unsigned int cfg)
{
	int ret = 0;
	unsigned long flags;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	ips_set_axi_bus_ctrl(g_ips_dev->base_reg, cfg);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_set_bus_ctrl);

int ips_set_md_cfg(sif_output_md_t *cfg)
{
	int ret = 0;
	unsigned long flags;
	struct roi_rect rect;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	rect.roi_height = cfg->roi_height;
	rect.roi_width = cfg->roi_width;
	rect.roi_x = cfg->roi_left;
	rect.roi_y = cfg->roi_top;
	ips_mot_set_roi(g_ips_dev->base_reg, &rect);
	ips_mot_set_diff_thd(g_ips_dev->base_reg, cfg->grid_tolerance);
	ips_mot_set_thresh(g_ips_dev->base_reg, cfg->threshold);
	ips_mot_set_step(g_ips_dev->base_reg, cfg->grid_step);
	ips_mot_data_sel(g_ips_dev->base_reg, 0x1);
	ips_mot_set_prec(g_ips_dev->base_reg, cfg->precision, cfg->weight_decay);
	ips_mot_enable(g_ips_dev->base_reg, cfg->enable);
	ips_set_sram_mux(g_ips_dev->base_reg, 1);
	ips_enable_intr(g_ips_dev->base_reg, MOD_INTR, true);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_set_md_cfg);

int ips_disable_md(void)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	ips_mot_enable(g_ips_dev->base_reg, 0);
	ips_set_sram_mux(g_ips_dev->base_reg, 0);
	ips_enable_intr(g_ips_dev->base_reg, MOD_INTR, false);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_disable_md);

int ips_set_md_refresh(bool enable)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	ips_mot_set_refresh(g_ips_dev->base_reg, enable);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_set_md_refresh);

int ips_set_md_resolution(u32 width, u32 height)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	ips_mot_set_resolution(g_ips_dev->base_reg, width, height);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_set_md_resolution);

int ips_get_md_event(void)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	wait_event_interruptible(g_ips_dev->done_wq, g_ips_dev->event == 1);
	g_ips_dev->event = 0;
	return ret;
}
EXPORT_SYMBOL_GPL(ips_get_md_event);


int ips_set_md_fmt(u32 fmt)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	ips_mot_set_fmt(g_ips_dev->base_reg, fmt, 0x5);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_set_md_fmt);

int ips_get_bus_ctrl(void)
{
	int ret = 0;
	unsigned long flags;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	ret = ips_get_axi_bus_ctrl(g_ips_dev->base_reg);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_get_bus_ctrl);

void ips_set_iram_size(u32 iram_size)
{
	BUG_ON(!g_ips_dev);
	g_ips_dev->iram_used_size = iram_size;
	vio_dbg("%s: 0x%x\n", __func__, iram_size);
}
EXPORT_SYMBOL_GPL(ips_set_iram_size);

int ips_get_free_iram_range(u32 *address)
{
	int length = 0;
	*address = g_ips_dev->iram_used_size;
	length = IRAM_MAX_RANG - g_ips_dev->iram_used_size;

	return length;
}
EXPORT_SYMBOL_GPL(ips_get_free_iram_range);

int ips_get_bus_status(void)
{
	int ret = 0;
	unsigned long flags;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	ret = ipu_get_axi_statue(g_ips_dev->base_reg);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_get_bus_status);

int ips_get_isp_frameid(void)
{
	int ret = 0;
	unsigned long flags;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	ret = ips_get_isp_frame_id(g_ips_dev->base_reg);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_get_isp_frameid);

void ips_set_isp_interrupt(bool enable)
{
	unsigned long flags;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	ips_enable_isp0_intr(g_ips_dev->base_reg, enable);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);
}
EXPORT_SYMBOL_GPL(ips_set_isp_interrupt);

void ips_set_isp_vcke_ctrl(bool enable)
{
	unsigned long flags;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	isp_vcke_ctrl(g_ips_dev->base_reg, enable);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);
}
EXPORT_SYMBOL_GPL(ips_set_isp_vcke_ctrl);

void ips_set_isp_vcke_th0(u32 cfg)
{
	unsigned long flags;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	isp_vcke_th0(g_ips_dev->base_reg, cfg);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);
}
EXPORT_SYMBOL_GPL(ips_set_isp_vcke_th0);

void ips_set_isp_vcke_th1(u32 cfg)
{
	unsigned long flags;
	BUG_ON(!g_ips_dev);

	spin_lock_irqsave(&g_ips_dev->shared_slock, flags);
	isp_vcke_th1(g_ips_dev->base_reg, cfg);
	spin_unlock_irqrestore(&g_ips_dev->shared_slock, flags);
}
EXPORT_SYMBOL_GPL(ips_set_isp_vcke_th1);

int vio_clk_enable(const char *name)
{
	int ret = 0;
	size_t index;
	struct clk *clk = NULL;

	for (index = 0; index < ARRAY_SIZE(vio_clk_list); index++) {
		if (!strcmp(name, vio_clk_list[index].name))
			clk = vio_clk_list[index].clk;
	}

	if (IS_ERR_OR_NULL(clk)) {
		vio_err("[@][ERR] %s: clk_target_list is NULL : %s\n", __func__, name);
		return -EINVAL;
	}

#ifdef DBG_DUMPCMU
	vio_info("[@][ENABLE] %s : (enable_count : %d)\n",
			name, __clk_get_enable_count(clk));
#endif
	ret = clk_prepare_enable(clk);
	if (ret) {
		vio_err("[@][ERR] %s: clk_prepare_enable is fail(%s)\n", __func__, name);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(vio_clk_enable);

int vio_clk_disable(const char *name)
{
	size_t index;
	struct clk *clk = NULL;

	for (index = 0; index < ARRAY_SIZE(vio_clk_list); index++) {
		if (!strcmp(name, vio_clk_list[index].name))
			clk = vio_clk_list[index].clk;
	}

	if (IS_ERR_OR_NULL(clk)) {
		vio_err("[@][ERR] %s: clk_target_list is NULL : %s\n", __func__, name);
		return -EINVAL;
	}

	clk_disable_unprepare(clk);

#ifdef DBG_DUMPCMU
	vio_info("[@][DISABLE] %s : (enable_count : %d)\n",
			name, __clk_get_enable_count(clk));
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(vio_clk_disable);

int vio_set_clk_rate(const char *name, ulong frequency)
{
	int ret = 0;
	size_t index;
	ulong round_rate = 0;
	struct clk *clk = NULL;

	for (index = 0; index < ARRAY_SIZE(vio_clk_list); index++) {
		if (!strcmp(name, vio_clk_list[index].name))
			clk = vio_clk_list[index].clk;
	}

	if (IS_ERR_OR_NULL(clk)) {
		vio_err("[@][ERR] %s: clk_target_list is NULL : %s\n", __func__, name);
		return -EINVAL;
	}

	round_rate = clk_round_rate(clk, frequency);
	ret = clk_set_rate(clk, round_rate);
	if (ret) {
		vio_err("[@][ERR] %s: clk_set_rate is fail(%s)\n", __func__, name);
		return ret;
	}

	vio_dbg("%s : frequence %ld\n", __func__, round_rate);

	return ret;
}
EXPORT_SYMBOL_GPL(vio_set_clk_rate);

ulong vio_get_clk_rate(const char *name)
{
	ulong frequency;
	size_t index;
	struct clk *clk = NULL;

	for (index = 0; index < ARRAY_SIZE(vio_clk_list); index++) {
		if (!strcmp(name, vio_clk_list[index].name))
			clk = vio_clk_list[index].clk;
	}

	if (IS_ERR_OR_NULL(clk)) {
		vio_err("[@][ERR] %s: clk_target_list is NULL : %s\n", __func__, name);
		return -EINVAL;
	}

	frequency = clk_get_rate(clk);

	return frequency;
}
EXPORT_SYMBOL_GPL(vio_get_clk_rate);

int vio_get_clk(struct device *dev)
{
	struct clk *clk;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(vio_clk_list); i++) {
		clk = devm_clk_get(dev, vio_clk_list[i].name);
		if (IS_ERR_OR_NULL(clk)) {
			vio_err("[@][ERR] %s: could not lookup clock : %s\n",
				__func__, vio_clk_list[i].name);
			return -EINVAL;
		}
		vio_clk_list[i].clk = clk;
		//vio_clk_enable(vio_clk_list[i].name);
		vio_info("%s clock frequence is %ld\n",
			vio_clk_list[i].name, vio_get_clk_rate(vio_clk_list[i].name));
	}

	return 0;
}

static int x3_ips_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_ips_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_ips_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_ips_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x3_ips_pm_ops = {
	.suspend		= x3_ips_suspend,
	.resume			= x3_ips_resume,
	.runtime_suspend	= x3_ips_runtime_suspend,
	.runtime_resume		= x3_ips_runtime_resume,
};

static irqreturn_t ips_isr(int this_irq, void *data)
{
	struct x3_ips_dev *ips;
	u32 status = 0;

	ips = data;
	ips_get_intr_status(ips->base_reg, MOD_INTR, &status, true);

	ips->event = 1;
	wake_up(&ips->done_wq);
	vio_info("MD interrupt\n");

	return IRQ_HANDLED;
}

static ssize_t ips_reg_dump(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_ips_dev *ips;

	ips = dev_get_drvdata(dev);

	ips_hw_dump(ips->base_reg);

	return 0;
}

static DEVICE_ATTR(regdump, 0444, ips_reg_dump, NULL);

static int x3_ips_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x3_ips_dev *ips;
	struct resource *mem_res;
	struct device *dev = NULL;

	ips = kzalloc(sizeof(struct x3_ips_dev), GFP_KERNEL);
	if (!ips) {
		vio_err("ips is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		vio_err("Failed to get io memory region(%p)", mem_res);
		ret = -EBUSY;
		goto err_get_resource;
	}

	ips->regs_start = mem_res->start;
	ips->regs_end = mem_res->end;
	ips->base_reg =  devm_ioremap_nocache(&pdev->dev, mem_res->start, resource_size(mem_res));
	if (!ips->base_reg) {
		vio_err("Failed to remap io region(%p)", ips->base_reg);
		ret = -ENOMEM;
		goto err_get_resource;
	}

	/* Get IRQ SPI number */
	ips->irq = platform_get_irq(pdev, 0);
	if (ips->irq < 0) {
		vio_err("Failed to get ips_irq(%d)", ips->irq);
		ret = -EBUSY;
		goto err_get_irq;
	}

	ret = request_threaded_irq(ips->irq, ips_isr, NULL, IRQF_TRIGGER_HIGH, "ips", ips);
	if (ret) {
		vio_err("request_irq(IRQ_SIF %d) is fail(%d)", ips->irq, ret);
		goto err_get_irq;
	}

	dev = &pdev->dev;
	ret = device_create_file(dev, &dev_attr_regdump);
	if (ret < 0) {
		vio_err("create regdump failed (%d)\n", ret);
		goto p_err;
	}

	ret = vio_get_clk(&pdev->dev);
	/* mask all interrupt source */
	ips_set_intr_mask(ips->base_reg, 0);

	platform_set_drvdata(pdev, ips);

	g_ips_dev = ips;
	spin_lock_init(&ips->shared_slock);
	mutex_init(&ips->shared_mux);
	init_waitqueue_head(&ips->done_wq);
	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

	return 0;

err_get_irq:
	iounmap(ips->base_reg);

err_get_resource:
	kfree(ips);
p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;

}

static int x3_ips_remove(struct platform_device *pdev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x3_ips_match[] = {
	{
		.compatible = "hobot,x3-ips",
	},
	{},
};
MODULE_DEVICE_TABLE(of, x3_ips_match);

static struct platform_driver x3_ips_driver = {
	.probe		= x3_ips_probe,
	.remove 	= x3_ips_remove,
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &x3_ips_pm_ops,
		.of_match_table = x3_ips_match,
	}
};

#else
static struct platform_device_id x3_ips_driver_ids[] = {
	{
		.name		= MODULE_NAME,
		.driver_data	= 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, x3_ips_driver_ids);

static struct platform_driver x3_ips_driver = {
	.probe		= x3_ips_probe,
	.remove		= __devexit_p(x3_ips_remove),
	.id_table	= x3_ips_driver_ids,
	.driver	  = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &x3_ips_pm_ops,
	}
};
#endif

static int __init x3_ips_init(void)
{
	int ret = platform_driver_register(&x3_ips_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

late_initcall(x3_ips_init);

static void __exit x3_ips_exit(void)
{
	platform_driver_unregister(&x3_ips_driver);
}
module_exit(x3_ips_exit);

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X3 IPS driver");
MODULE_LICENSE("GPL v2");
