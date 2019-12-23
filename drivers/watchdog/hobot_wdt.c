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
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/debug.h>
#include <uapi/linux/sched/types.h>
#include <soc/hobot/hobot_timer.h>

#define X2_WDT_NAME				"x2_wdt"

/* timeout value (in seconds) */
#define X2_WDT_DEFAULT_TIMEOUT	10
#define X2_WDT_MIN_TIMEOUT		1
#define X2_WDT_MAX_TIMEOUT		178

static int wdt_timeout;
static int nowayout = WATCHDOG_NOWAYOUT;
static u64 timer_rate;
static int trigger_bark;
static int disabled;
static int panic_on_bark;
static int on_panic;
static spinlock_t on_panic_lock;

module_param(wdt_timeout, int, 0644);
MODULE_PARM_DESC(wdt_timeout,
		 "Watchdog time in seconds. (default="
		 __MODULE_STRING(X2_WDT_DEFAULT_TIMEOUT) ")");

module_param(nowayout, int, 0644);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

module_param(trigger_bark, int, 0644);
MODULE_PARM_DESC(trigger_bark,
		 "Trigger watchdog for test (default="
		 __MODULE_STRING(WATCHDOG_TRIGGER_BARK) ")");

module_param(disabled, int, 0644);
MODULE_PARM_DESC(disabled,
		 "disable watchdog for debugging purpose (default="
		 __MODULE_STRING(WATCHDOG_DISABLE) ")");

module_param(panic_on_bark, int, 0644);
MODULE_PARM_DESC(panic_on_bark,
		 "trigger panic in IPI on other CPU (default="
		 __MODULE_STRING(WATCHDOG_PANIC) ")");

#define x2_wdt_rd(dev, reg) 	  ioread32((dev)->regs_base + (reg))
#define x2_wdt_wr(dev, reg, val)  iowrite32((val), (dev)->regs_base + (reg))

struct x2_wdt {
	void __iomem *regs_base;
	struct clk *clock;
	spinlock_t io_lock;
	struct watchdog_device x2_wdd;

	bool enabled;
	u32 pet_time;
	u32 bark_time;
	u32 bite_time;
	u32 bark_irq;
	u32 bite_irq;
	u64 last_pet;

	struct notifier_block panic_blk;
	u32 panic_wait_time;

	struct task_struct *watchdog_thread;
	struct timer_list pet_timer;
	wait_queue_head_t  pet_complete;
	bool timer_expired;
	u64 timer_expired_time;
	u64 thread_start_time;
	bool barking;
};

static void x2_wdt_init_hw(struct x2_wdt *x2wdt)
{
	u32 val;

	/* set timer2 to watchdog mode */
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

	/* reset previos value */
	x2_wdt_wr(x2wdt, X2_TIMER_WDCLR_REG, X2_TIMER_WDT_RESET);

	val = x2_wdt_rd(x2wdt, X2_TIMER_TMREN_REG);
	val |= X2_TIMER_T2STOP;
	x2_wdt_wr(x2wdt, X2_TIMER_TMRSTOP_REG, val);

	/* disable wdt interrupt */
	if (x2wdt->bark_irq)
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

static int x2_wdt_ping(struct watchdog_device *wdd)
{
	return x2_wdt_reload(wdd);
}

static int x2_wdt_start(struct watchdog_device *wdd)
{
	struct x2_wdt *x2wdt = watchdog_get_drvdata(wdd);
	u32 bark_count;
	u32 bite_count;
	u32 val;

	spin_lock(&x2wdt->io_lock);

	/* reset previos value */
	x2_wdt_wr(x2wdt, X2_TIMER_WDCLR_REG, X2_TIMER_WDT_RESET);

	/* Fill the count reg */
	bark_count = x2wdt->bark_time * timer_rate;
	bite_count = x2wdt->bite_time * timer_rate;
	x2_wdt_wr(x2wdt, X2_TIMER_WDTGT_REG, bark_count);
	x2_wdt_wr(x2wdt, X2_TIMER_WDWAIT_REG, bite_count);

	/* enable wdt interrupt */
	if (x2wdt->bark_irq) {
		val = ~(x2_wdt_rd(x2wdt, X2_TIMER_TMR_INTMASK_REG));
		val |= X2_TIMER_WDT_INTMASK;
		x2_wdt_wr(x2wdt, X2_TIMER_TMR_UNMASK_REG, val);
	}

	/* Start wdt timer */
	val = x2_wdt_rd(x2wdt, X2_TIMER_TMREN_REG);
	val |= X2_TIMER_T2START;
	x2_wdt_wr(x2wdt, X2_TIMER_TMRSTART_REG, val);

	/* Unmask bark irq */
	val = ~(x2_wdt_rd(x2wdt, X2_TIMER_TMR_INTMASK_REG));
	val |= X2_TIMER_WDT_INTMASK;
	x2_wdt_wr(x2wdt, X2_TIMER_TMR_UNMASK_REG, val);

	spin_unlock(&x2wdt->io_lock);

	return 0;
}

static int x2_wdt_settimeout(struct watchdog_device *wdd, unsigned int new_time)
{
	struct x2_wdt *x2wdt = watchdog_get_drvdata(wdd);

	if (wdd->timeout > X2_WDT_MAX_TIMEOUT)
		wdd->timeout = X2_WDT_MAX_TIMEOUT;
	else if (wdd->timeout < X2_WDT_MIN_TIMEOUT)
		wdd->timeout = X2_WDT_MIN_TIMEOUT;
	else
		wdd->timeout = new_time;

	x2wdt->bark_time = wdd->timeout;
	x2wdt->bite_time =  x2wdt->bark_time + 5;

	return x2_wdt_start(wdd);
}

void dump_cpu_state(void *data)
{
	pr_err("dump stack on on cpu:%d\n", get_cpu());
	dump_stack();

	spin_lock(&on_panic_lock);
	if (panic_on_bark && on_panic == 0) {
		on_panic = 1;
		spin_unlock(&on_panic_lock);
		panic("Panic on watchdog bark ...");
	}
}

void check_other_cpus(void)
{
	int cpu;
	struct cpumask  mask;

	cpu = get_cpu();
	memcpy(&mask, cpu_online_mask, sizeof(struct cpumask));
	cpumask_clear_cpu(cpu, &mask);
	for_each_cpu(cpu, &mask) {
		smp_call_function_single(cpu, dump_cpu_state,
			NULL, 1);
	}
}

static irqreturn_t x2_wdt_bark_irq_handler(int irq, void *data)
{
	u32 val;
	struct x2_wdt *x2wdt = (struct x2_wdt *)data;

	x2wdt->barking = true;

	/* clear the irq */
	val = x2_wdt_rd(x2wdt, X2_TIMER_TMR_SRCPND_REG);
	val &= X2_TIMER_WDT_INTMASK;
	x2_wdt_wr(x2wdt, X2_TIMER_TMR_SRCPND_REG, val);

	pr_err("x2_wdt: bark at %lld on cpu:%d\n", sched_clock(), get_cpu());
	dump_stack();
	pr_err("x2_wdt: check other cpus\n");

	/* flush cache for dump */
	printk_safe_flush_on_panic();

	/* may stuck here if other cpu can't response IPI */
	check_other_cpus();

	pr_err("x2_wdt: waiting for panic on other CPU or watchdog bite ...\n");

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

	spin_lock(&x2wdt->io_lock);

	/* Fill the count reg: 100ms */
	count = timer_rate / 10;
	x2_wdt_wr(x2wdt, X2_TIMER_WDTGT_REG, count);
	x2_wdt_wr(x2wdt, X2_TIMER_WDWAIT_REG, count);

	/* disable wdt interrupt */
	if (x2wdt->bark_irq)
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
	.ping  = x2_wdt_ping,
	.set_timeout = x2_wdt_settimeout,
	.restart = x2_wdt_restart,
};

static int panic_wdog_handler(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct x2_wdt *x2wdt = container_of(this, struct x2_wdt, panic_blk);
	u32 val;
	u32 count;

	if (panic_timeout != 0) {
		/* Fill the count reg */
		count = (panic_timeout + x2wdt->panic_wait_time) *
		(u32)timer_rate;
		x2_wdt_wr(x2wdt, X2_TIMER_WDTGT_REG, count);
		x2_wdt_wr(x2wdt, X2_TIMER_WDWAIT_REG, count);

		/* reset previos value */
		x2_wdt_wr(x2wdt, X2_TIMER_WDCLR_REG, X2_TIMER_WDT_RESET);
	} else {
		/* no need wdt here */
		val = x2_wdt_rd(x2wdt, X2_TIMER_TMREN_REG);
		val |= X2_TIMER_T2STOP;
		x2_wdt_wr(x2wdt, X2_TIMER_TMRSTOP_REG, val);

		/* make sure watchdog is stopped before proceeding */
		mb();
	}
	return NOTIFY_DONE;
}

static void pet_watchdog(struct x2_wdt *x2wdt)
{
	if (x2wdt->barking) {
		pr_err("bark irq trigger, still responsing irq\n");
		dump_stack();
	}

	x2_wdt_reload(&x2wdt->x2_wdd);
}

static void pet_timer_wakeup(unsigned long data)
{
	struct x2_wdt *x2wdt = (struct x2_wdt *)data;

	if (disabled) {
		pr_info("watchdog is disabled, stop watchdog.");
		x2wdt->enabled = 0;
		x2_wdt_stop(&x2wdt->x2_wdd);
	}

	if (trigger_bark) {
		pr_info("trigger bark for test.\n");
		return;
	}

	x2wdt->timer_expired = true;
	x2wdt->timer_expired_time = sched_clock();
	wake_up(&x2wdt->pet_complete);
}

static __ref int watchdog_kthread(void *arg)
{
	struct x2_wdt *x2wdt = (struct x2_wdt *)arg;
	unsigned long expired_time = 0;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO-1};
	int ret;

	sched_setscheduler(current, SCHED_FIFO, &param);
	while (!kthread_should_stop()) {
		do {
			ret = wait_event_interruptible(x2wdt->pet_complete,
						x2wdt->timer_expired);
		} while (ret != 0);

		x2wdt->thread_start_time = sched_clock();

		x2wdt->timer_expired = false;

		if (x2wdt->enabled) {
			expired_time = msecs_to_jiffies(x2wdt->pet_time * 1000);
			pet_watchdog(x2wdt);
		}
		/* Check again before scheduling
		 * Could have been changed on other cpu
		 */
		mod_timer(&x2wdt->pet_timer, jiffies + expired_time);
	}

	return 0;
}

static int x2_wdog_dt_to_pdata(struct platform_device *pdev,
					struct x2_wdt *x2wdt)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	x2wdt->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(x2wdt->regs_base)) {
		ret = PTR_ERR(x2wdt->regs_base);
		return ret;
	}

	x2wdt->bark_irq = platform_get_irq(pdev, 0);
	if (x2wdt->bark_irq < 0) {
		dev_err(&pdev->dev, "failed to get bark_irq\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(node, "bark-time", &x2wdt->bark_time);
	if (ret) {
		dev_err(&pdev->dev, "reading bark time failed\n");
		return -ENXIO;
	}
	ret = of_property_read_u32(node, "bite-time", &x2wdt->bite_time);
	if (ret) {
		dev_err(&pdev->dev, "reading bite time failed\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(node, "pet-time", &x2wdt->pet_time);
	if (ret) {
		dev_err(&pdev->dev, "reading pet time failed\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(node, "panic-wait-time",
	&x2wdt->panic_wait_time);
	if (ret) {
		dev_err(&pdev->dev, "reading panic wait time failed\n");
		return -ENXIO;
	}

	dev_info(&pdev->dev, "watchdog setting [%ds %ds %ds]\n",
		x2wdt->bark_time, x2wdt->bite_time, x2wdt->pet_time);

	return 0;
}
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
	int ret;
	struct x2_wdt *x2wdt;
	struct watchdog_device *x2_wdd;
	u64 expired_time;

	x2wdt = devm_kzalloc(&pdev->dev, sizeof(*x2wdt), GFP_KERNEL);
	if (!x2wdt)
		return -ENOMEM;

	ret = x2_wdog_dt_to_pdata(pdev, x2wdt);
	if (ret)
		goto err;
	x2_wdd = &x2wdt->x2_wdd;
	x2_wdd->info = &x2_wdt_info;
	x2_wdd->ops = &x2_wdt_ops;
	x2_wdd->timeout = x2wdt->bark_time;
	x2_wdd->min_timeout = X2_WDT_MIN_TIMEOUT;
	x2_wdd->max_timeout = X2_WDT_MAX_TIMEOUT;

	x2_wdt_init_hw(x2wdt);

	if (x2wdt->bark_irq >= 0) {
		ret = devm_request_irq(&pdev->dev, x2wdt->bark_irq,
		x2_wdt_bark_irq_handler, 0,
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

	timer_rate = clk_get_rate(x2wdt->clock);
	if (!timer_rate) {
		pr_err("failed to get watchdog clock rate\n");
		return -1;
	}

	/* Initialize the members of x2_wdt structure */
	x2_wdd->parent = &pdev->dev;

	wdt_timeout = x2wdt->bark_time;
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

	/* attach x2_watchdog on CPU0 */
	x2wdt->watchdog_thread = kthread_create_on_cpu(watchdog_kthread,
		x2wdt, 0, "x2_watchdog");
	if (IS_ERR(x2wdt->watchdog_thread)) {
		ret = PTR_ERR(x2wdt->watchdog_thread);
		goto err;
	}

	expired_time = msecs_to_jiffies(x2wdt->pet_time * 1000);

	x2wdt->panic_blk.notifier_call = panic_wdog_handler;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &x2wdt->panic_blk);

	init_waitqueue_head(&x2wdt->pet_complete);
	x2wdt->timer_expired = false;

	wake_up_process(x2wdt->watchdog_thread);
	init_timer(&x2wdt->pet_timer);
	x2wdt->pet_timer.data = (unsigned long)x2wdt;
	x2wdt->pet_timer.function = pet_timer_wakeup;
	x2wdt->pet_timer.expires = jiffies + expired_time;
	add_timer(&x2wdt->pet_timer);

	x2_wdt_start(&x2wdt->x2_wdd);
	x2wdt->last_pet = sched_clock();
	x2wdt->enabled = true;

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

#ifdef CONFIG_PM
int x2_wdt_suspend(struct device *dev)
{
	struct x2_wdt *x2wdt = dev_get_drvdata(dev);

	if (x2wdt->enabled)
		x2_wdt_stop(&x2wdt->x2_wdd);

	x2wdt->enabled = false;
	return 0;
}

int x2_wdt_resume(struct device *dev)
{
	struct x2_wdt *x2wdt = dev_get_drvdata(dev);

	if (!x2wdt->enabled)
		x2_wdt_start(&x2wdt->x2_wdd);

	x2wdt->enabled = true;
	return 0;
}
#endif

static const struct dev_pm_ops x2_wdt_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(x2_wdt_suspend,
			x2_wdt_resume)
};

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
		.pm = &x2_wdt_dev_pm_ops,
	},
};

module_platform_driver(x2_wdt_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("Watchdog driver for X2 WDT");
MODULE_LICENSE("GPL v2");
