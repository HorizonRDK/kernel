/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

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

#define MODULE_NAME "X2A IPS"
#define REGISTER_CLK(name) {name, NULL}

struct x2a_ips_dev *g_ips_dev;

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
};

void ips_set_module_reset(unsigned long module)
{
	BUG_ON(!g_ips_dev);

	ips_module_reset(g_ips_dev->base_reg, module);
}
EXPORT_SYMBOL_GPL(ips_set_module_reset);

int ips_set_clk_ctrl(unsigned long module, bool enable)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	ret = ips_clk_ctrl(g_ips_dev->base_reg, module, enable);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_set_clk_ctrl);

int ips_set_bus_ctrl(unsigned int cfg)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	ips_set_axi_bus_ctrl(g_ips_dev->base_reg, cfg);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_set_bus_ctrl);

int ips_get_bus_ctrl(void)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	ret = ips_get_axi_bus_ctrl(g_ips_dev->base_reg);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_get_bus_ctrl);

int ips_get_bus_status(void)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	ret = ipu_get_axi_statue(g_ips_dev->base_reg);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_get_bus_status);

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
	struct clk *clk = NULL;

	for (index = 0; index < ARRAY_SIZE(vio_clk_list); index++) {
		if (!strcmp(name, vio_clk_list[index].name))
			clk = vio_clk_list[index].clk;
	}

	if (IS_ERR_OR_NULL(clk)) {
		vio_err("[@][ERR] %s: clk_target_list is NULL : %s\n", __func__, name);
		return -EINVAL;
	}

	ret = clk_set_rate(clk, frequency);
	if (ret) {
		vio_err("[@][ERR] %s: clk_set_rate is fail(%s)\n", __func__, name);
		return ret;
	}

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

static int x2a_ips_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_ips_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_ips_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_ips_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x2a_ips_pm_ops = {
	.suspend		= x2a_ips_suspend,
	.resume			= x2a_ips_resume,
	.runtime_suspend	= x2a_ips_runtime_suspend,
	.runtime_resume		= x2a_ips_runtime_resume,
};

static irqreturn_t ips_isr(int this_irq, void *data)
{
	return IRQ_HANDLED;
}

static int x2a_ips_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x2a_ips_dev *ips;
	struct resource *mem_res;

	ips = kzalloc(sizeof(struct x2a_ips_dev), GFP_KERNEL);
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
#if 0
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
#endif
	ret = vio_get_clk(&pdev->dev);

	g_ips_dev = ips;

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

static int x2a_ips_remove(struct platform_device *pdev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x2a_ips_match[] = {
	{
		.compatible = "hobot,x2a-ips",
	},
	{},
};
MODULE_DEVICE_TABLE(of, x2a_ips_match);

static struct platform_driver x2a_ips_driver = {
	.probe		= x2a_ips_probe,
	.remove 	= x2a_ips_remove,
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &x2a_ips_pm_ops,
		.of_match_table = x2a_ips_match,
	}
};

#else
static struct platform_device_id x2a_ips_driver_ids[] = {
	{
		.name		= MODULE_NAME,
		.driver_data	= 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, x2a_ips_driver_ids);

static struct platform_driver x2a_ips_driver = {
	.probe		= x2a_ips_probe,
	.remove		= __devexit_p(x2a_ips_remove),
	.id_table	= x2a_ips_driver_ids,
	.driver	  = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &x2a_ips_pm_ops,
	}
};
#endif

static int __init x2a_ips_init(void)
{
	int ret = platform_driver_register(&x2a_ips_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

late_initcall(x2a_ips_init);

static void __exit x2a_ips_exit(void)
{
	platform_driver_unregister(&x2a_ips_driver);
}
module_exit(x2a_ips_exit);

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X2A IPS driver");
MODULE_LICENSE("GPL");
