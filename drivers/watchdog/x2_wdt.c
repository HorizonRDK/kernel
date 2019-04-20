/*
 * X2 watchdog controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/delay.h>
#include <x2/x2_timer.h>

#define X2_WDT_NAME				"x2_wdt"

/* timeout value (in seconds) */
#define X2_WDT_DEFAULT_TIMEOUT	10
#define X2_WDT_MIN_TIMEOUT		1
#define X2_WDT_MAX_TIMEOUT		178

static int wdt_timeout;
static int nowayout = WATCHDOG_NOWAYOUT;

module_param(wdt_timeout, int, 0644);
MODULE_PARM_DESC(wdt_timeout,
		 "Watchdog time in seconds. (default="
		 __MODULE_STRING(X2_WDT_DEFAULT_TIMEOUT) ")");

module_param(nowayout, int, 0644);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

#define x2_wdt_rd(dev, reg) 	  ioread32((dev)->regs_base + (reg))
#define x2_wdt_wr(dev, reg, val)  iowrite32((val), (dev)->regs_base + (reg))

struct x2_wdt {
	void __iomem *regs_base;
	int irq;
	struct clk *clock;
	spinlock_t io_lock;
	struct watchdog_device x2_wdd;
};

static void x2_wdt_init_hw(struct x2_wdt *x2wdt)
{
	u32 val;

	/* enable wtd mode */
	val = x2_wdt_rd(x2wdt, X2_TIMER_TMRMODE_REG);
	val &= 0xFFFFF0FF;
	val |= (X2_TIMER_WDT_MODE << X2_TIMER_T2MODE_OFFSET);
	x2_wdt_wr(x2wdt, X2_TIMER_TMRMODE_REG, val);

	return;
}

static int x2_wdt_stop(struct watchdog_device *wdd)
{
	u32 val;
	struct x2_wdt *x2wdt = watchdog_get_drvdata(wdd);

	spin_lock(&x2wdt->io_lock);

	val = x2_wdt_rd(x2wdt, X2_TIMER_TMREN_REG);
	val |= X2_TIMER_T2STOP;
	x2_wdt_wr(x2wdt, X2_TIMER_TMRSTOP_REG, val);

	/* disable wdt interrupt */
	if (x2wdt->irq)
		x2_wdt_wr(x2wdt, X2_TIMER_TMR_SETMASK_REG, X2_TIMER_WDT_INTMASK);

	spin_unlock(&x2wdt->io_lock);

	return 0;
}

static int x2_wdt_reload(struct watchdog_device *wdd)
{
	struct x2_wdt *x2wdt = watchdog_get_drvdata(wdd);

	spin_lock(&x2wdt->io_lock);
	x2_wdt_wr(x2wdt, X2_TIMER_WDCLR_REG, X2_TIMER_WDT_RESET);
	spin_unlock(&x2wdt->io_lock);

	return 0;
}

static int x2_wdt_start(struct watchdog_device *wdd)
{
	u32 count, val;
	struct x2_wdt *x2wdt = watchdog_get_drvdata(wdd);
	unsigned long freq = clk_get_rate(x2wdt->clock);

	spin_lock(&x2wdt->io_lock);

	/* Fill the count reg */
	count = wdd->timeout * freq;
	x2_wdt_wr(x2wdt, X2_TIMER_WDTGT_REG, count);
	x2_wdt_wr(x2wdt, X2_TIMER_WDWAIT_REG, count);

	/* enable wdt interrupt */
	if (x2wdt->irq) {
		val = ~(x2_wdt_rd(x2wdt, X2_TIMER_TMR_INTMASK_REG));
		val |= X2_TIMER_WDT_INTMASK;
		x2_wdt_wr(x2wdt, X2_TIMER_TMR_UNMASK_REG, val);
	}

	/* Start wdt timer */
	val = x2_wdt_rd(x2wdt, X2_TIMER_TMREN_REG);
	val |= X2_TIMER_T2START;
	x2_wdt_wr(x2wdt, X2_TIMER_TMRSTART_REG, val);

	/* reset previos value */
	x2_wdt_wr(x2wdt, X2_TIMER_WDCLR_REG, X2_TIMER_WDT_RESET);

	spin_unlock(&x2wdt->io_lock);

	return 0;
}

static int x2_wdt_settimeout(struct watchdog_device *wdd, unsigned int new_time)
{
	if (wdd->timeout > X2_WDT_MAX_TIMEOUT)
		wdd->timeout = X2_WDT_MAX_TIMEOUT;
	else if (wdd->timeout < X2_WDT_MIN_TIMEOUT)
		wdd->timeout = X2_WDT_MIN_TIMEOUT;
	else
		wdd->timeout = new_time;

	return x2_wdt_start(wdd);
}

static irqreturn_t x2_wdt_irq_handler(int irq, void *data)
{
	u32 val;
	struct x2_wdt *x2wdt = (struct x2_wdt *)data;

	/* clear the irq */
	val = x2_wdt_rd(x2wdt, X2_TIMER_TMR_SRCPND_REG);
	val &= X2_TIMER_WDT_INTMASK;
	x2_wdt_wr(x2wdt, X2_TIMER_TMR_SRCPND_REG, val);

	/* Feed the dog */
	x2_wdt_reload(&x2wdt->x2_wdd);

	return IRQ_HANDLED;
}

static const struct watchdog_info x2_wdt_info = {
	.identity = "x2_wdt_watchdog",
	.options  = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static int x2_wdt_restart(struct watchdog_device *wdd, unsigned long action,
						  void *data)
{
	u32 count, val;
	struct x2_wdt *x2wdt = watchdog_get_drvdata(wdd);
	unsigned long freq = clk_get_rate(x2wdt->clock);

	spin_lock(&x2wdt->io_lock);

	/* Fill the count reg: 100ms */
	count = freq / 10;
	x2_wdt_wr(x2wdt, X2_TIMER_WDTGT_REG, count);
	x2_wdt_wr(x2wdt, X2_TIMER_WDWAIT_REG, count);

	/* disable wdt interrupt */
	if (x2wdt->irq)
		x2_wdt_wr(x2wdt, X2_TIMER_TMR_SETMASK_REG,
				  X2_TIMER_WDT_INTMASK);

	/* Start wdt timer */
	val = x2_wdt_rd(x2wdt, X2_TIMER_TMREN_REG);
	val |= X2_TIMER_T2START;
	x2_wdt_wr(x2wdt, X2_TIMER_TMRSTART_REG, val);

	/* reset previos value */
	x2_wdt_wr(x2wdt, X2_TIMER_WDCLR_REG, X2_TIMER_WDT_RESET);

	spin_unlock(&x2wdt->io_lock);

	/* wait for reset to assert... */
	mdelay(500);

	return NOTIFY_DONE;
}

static const struct watchdog_ops x2_wdt_ops = {
	.owner = THIS_MODULE,
	.start = x2_wdt_start,
	.stop  = x2_wdt_stop,
	.ping  = x2_wdt_reload,
	.set_timeout = x2_wdt_settimeout,
	.restart = x2_wdt_restart,
};

/**
 * x2_wdt_probe - Probe call for the device.
 *
 * @pdev: handle to the platform device structure.
 * Return: 0 on success, negative error otherwise.
 *
 * It does all the memory allocation and registration for the device.
 */
static int x2_wdt_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	struct x2_wdt *x2wdt;
	struct watchdog_device *x2_wdd;

	x2wdt = devm_kzalloc(&pdev->dev, sizeof(*x2wdt), GFP_KERNEL);
	if (!x2wdt)
		return -ENOMEM;

	x2_wdd = &x2wdt->x2_wdd;
	x2_wdd->info = &x2_wdt_info;
	x2_wdd->ops = &x2_wdt_ops;
	x2_wdd->timeout = X2_WDT_DEFAULT_TIMEOUT;
	x2_wdd->min_timeout = X2_WDT_MIN_TIMEOUT;
	x2_wdd->max_timeout = X2_WDT_MAX_TIMEOUT;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	x2wdt->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(x2wdt->regs_base)) {

		pr_info("enter %s_%d!\n", __func__, __LINE__);
		ret = PTR_ERR(x2wdt->regs_base);
		goto err;
	}

	x2_wdt_init_hw(x2wdt);

	/* Register the interrupt */
	x2wdt->irq = platform_get_irq(pdev, 0);

	if (x2wdt->irq >= 0) {
		ret = devm_request_irq(&pdev->dev, x2wdt->irq, x2_wdt_irq_handler, 0,
								pdev->name, x2wdt);
		if (ret) {
			dev_err(&pdev->dev, "can't register interrupt handler err=%d\n", ret);
			goto err;
		}
	}

	x2wdt->clock = devm_clk_get(&pdev->dev, "watchdog_mclk");
	if (IS_ERR(x2wdt->clock)) {
		dev_err(&pdev->dev, "failed to find watchdog clock source\n");
		ret = PTR_ERR(x2wdt->clock);
		goto err;
	}

	ret = clk_prepare_enable(x2wdt->clock);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err;
	}

	/* Initialize the members of x2_wdt structure */
	x2_wdd->parent = &pdev->dev;

	ret = watchdog_init_timeout(x2_wdd, wdt_timeout, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "unable to set timeout value.\n");
		goto err_clk;
	}

	watchdog_set_nowayout(x2_wdd, nowayout);
	watchdog_set_restart_priority(x2_wdd, 192);
	watchdog_stop_on_reboot(x2_wdd);
	watchdog_set_drvdata(x2_wdd, x2wdt);

	spin_lock_init(&x2wdt->io_lock);

	ret = watchdog_register_device(x2_wdd);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register wdt device.\n");
		goto err_clk;
	}
	platform_set_drvdata(pdev, x2wdt);

	dev_info(&pdev->dev, "X2 Watchdog Timer at %p with timeout %ds%s\n",
			x2wdt->regs_base, x2_wdd->timeout, nowayout?",nowayout":"");

	return 0;

 err_clk:
	clk_disable_unprepare(x2wdt->clock);

 err:
	return ret;
}

/**
 * x2_wdt_remove - Probe call for the device.
 *
 * @pdev: handle to the platform device structure.
 * Return: 0 on success, otherwise negative error.
 *
 * Unregister the device after releasing the resources.
 */
static int x2_wdt_remove(struct platform_device *pdev)
{
	struct x2_wdt *x2wdt = platform_get_drvdata(pdev);

	x2_wdt_stop(&x2wdt->x2_wdd);
	watchdog_unregister_device(&x2wdt->x2_wdd);
	clk_disable_unprepare(x2wdt->clock);

	return 0;
}

/**
 * x2_wdt_shutdown - Stop the device.
 *
 * @pdev: handle to the platform structure.
 *
 */
static void x2_wdt_shutdown(struct platform_device *pdev)
{
	struct x2_wdt *x2wdt = platform_get_drvdata(pdev);

	x2_wdt_stop(&x2wdt->x2_wdd);
}

static const struct of_device_id x2_wdt_of_match[] = {
	{ .compatible = "hobot,x2-wdt", },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, x2_wdt_of_match);

/* Driver Structure */
static struct platform_driver x2_wdt_driver = {
	.probe		= x2_wdt_probe,
	.remove 	= x2_wdt_remove,
	.shutdown	= x2_wdt_shutdown,
	.driver 	= {
		.name	= X2_WDT_NAME,
		.of_match_table = x2_wdt_of_match,
	},
};

module_platform_driver(x2_wdt_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("Watchdog driver for X2 WDT");
MODULE_LICENSE("GPL v2");
