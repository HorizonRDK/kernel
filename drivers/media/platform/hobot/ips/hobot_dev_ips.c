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

struct x2a_ips_dev *g_ips_dev;

int ips_set_clk_ctrl(unsigned long module, bool enable)
{
	int ret = 0;
	BUG_ON(!g_ips_dev);

	ret = ips_clk_ctrl(g_ips_dev->base_reg, module, enable);

	return ret;
}
EXPORT_SYMBOL_GPL(ips_set_clk_ctrl);

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
	return 0;
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
