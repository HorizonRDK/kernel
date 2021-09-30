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

#ifdef CONFIG_PREEMPT_RT_FULL
#include <linux/swait.h>
#endif

#define HOBOT_WDT_NAME				"hobot_wdt"

/* timeout value (in seconds) */
#define HOBOT_WDT_DEFAULT_TIMEOUT	10
#define HOBOT_WDT_MIN_TIMEOUT		1
#define HOBOT_WDT_MAX_TIMEOUT		178
#define HOBOT_WDT_PET_BARK_INTERVAL 3

static int wdt_timeout;
static int wdt_timeleft;
static int nowayout = WATCHDOG_NOWAYOUT;
static u64 timer_rate;
static int panic_on_bark;
static int on_panic;
static int do_ipi_ping = 1;

#ifdef CONFIG_HOBOT_WATCHDOG_ENABLE
static int kthread_disabled = 0;
#else
static int kthread_disabled = 1;
#endif

module_param(nowayout, int, 0644);
module_param(panic_on_bark, int, 0644);
module_param(do_ipi_ping, int, 0644);

#define hobot_wdt_rd(dev, reg)       ioread32((dev)->regs_base + (reg))
#define hobot_wdt_wr(dev, reg, val)  iowrite32((val), (dev)->regs_base + (reg))

struct hobot_wdt {
	void __iomem *regs_base;
	struct clk *clock;
	spinlock_t io_lock;
	struct watchdog_device hobot_wdd;

	bool enabled;
	u32 pet_time;
	u32 bark_time;
	u32 bite_time;
	u32 bite_interval;
	s32 bark_irq;
	u32 bite_irq;
	u64 last_pet;

	struct notifier_block panic_blk;

	struct task_struct *watchdog_thread;
	struct hrtimer pet_timer;
#ifdef CONFIG_PREEMPT_RT_FULL
	struct swait_queue_head  pet_wait;
#else
	wait_queue_head_t  pet_wait;
#endif
	bool timer_expired;
	u64 thread_wakeup_time;
	bool barking;

	cpumask_t alive_mask;
	unsigned long long ping_start[NR_CPUS];
	unsigned long long ping_end[NR_CPUS];
} *g_hbwdt;

static int hobot_wdt_start(struct watchdog_device *wdd);
static int hobot_wdt_stop(struct watchdog_device *wdd);
static int hobot_wdt_settimeout(struct watchdog_device *wdd,
		unsigned int new_time);

static int wdt_timeout_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret < 0)
		return ret;

	ret = hobot_wdt_settimeout(&g_hbwdt->hobot_wdd, wdt_timeout);

	wdt_timeout = g_hbwdt->bite_time;

	return ret;
}

static const struct kernel_param_ops wdt_timeout_param_ops = {
	.set = wdt_timeout_set,
	.get = param_get_int,
};

module_param_cb(wdt_timeout, &wdt_timeout_param_ops, &wdt_timeout, 0644);

static int wdt_kthread_disable_set(const char *val,
		const struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret < 0)
		return ret;

	if (kthread_disabled == 0 && !g_hbwdt->enabled) {
		hobot_wdt_start(&g_hbwdt->hobot_wdd);
	} else if (kthread_disabled == 1 && g_hbwdt->enabled) {
		hobot_wdt_stop(&g_hbwdt->hobot_wdd);
	}

	return 0;
}

static const struct kernel_param_ops wdt_disable_param_ops = {
	.set = wdt_kthread_disable_set,
	.get = param_get_int,
};

module_param_cb(kthread_disable, &wdt_disable_param_ops,
		&kthread_disabled, 0644);

int wdt_timeleft_get(char *buffer, const struct kernel_param *kp)
{
	u32 timeleft;
	int ret;

	timeleft = hobot_wdt_rd(g_hbwdt, HOBOT_TIMER_WD1D_REG) / timer_rate;
	timeleft = g_hbwdt->bark_time - timeleft + g_hbwdt->bite_interval;

	ret = snprintf(buffer, 64, "timeleft:%ds.\n", timeleft);

	return ret;
}

static const struct kernel_param_ops wdt_timeleft_param_ops = {
	.get = wdt_timeleft_get,
};

module_param_cb(wdt_timeleft, &wdt_timeleft_param_ops, &wdt_timeleft, 0444);


static void hobot_wdt_init_hw(struct hobot_wdt *hbwdt)
{
	u32 val;

	/* set timer2 to watchdog mode */
	val = hobot_wdt_rd(hbwdt, HOBOT_TIMER_TMRMODE_REG);
	val &= 0xFFFFF0FF;
	val |= (HOBOT_TIMER_WDT_MODE << HOBOT_TIMER_T2MODE_OFFSET);
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMRMODE_REG, val);

	/* reset previous value */
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_WDCLR_REG, HOBOT_TIMER_WDT_RESET);

	val = hobot_wdt_rd(hbwdt, HOBOT_TIMER_TMREN_REG);
	val |= HOBOT_TIMER_T2STOP;
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMRSTOP_REG, val);

	/* disable bark irq */
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMR_SETMASK_REG, HOBOT_TIMER_WDT_INTMASK);

	return;
}

static int hobot_wdt_stop(struct watchdog_device *wdd)
{
	u32 val;
	struct hobot_wdt *hbwdt = watchdog_get_drvdata(wdd);

	if (!hbwdt->enabled) {
		pr_debug("watchdog is already stopped.");
		return 0;
	}

	pr_debug("\n");
	if (!kthread_disabled)
		hrtimer_cancel(&hbwdt->pet_timer);

	spin_lock(&hbwdt->io_lock);

	hbwdt->enabled = false;

	/* reset previous value */
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_WDCLR_REG, HOBOT_TIMER_WDT_RESET);

	val = hobot_wdt_rd(hbwdt, HOBOT_TIMER_TMREN_REG);
	val |= HOBOT_TIMER_T2STOP;
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMRSTOP_REG, val);

	/* disable wdt interrupt */
	if (hbwdt->bark_irq)
		hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMR_SETMASK_REG, HOBOT_TIMER_WDT_INTMASK);

	spin_unlock(&hbwdt->io_lock);

	return 0;
}

static int hobot_wdt_reload(struct watchdog_device *wdd)
{
	struct hobot_wdt *hbwdt = watchdog_get_drvdata(wdd);

	spin_lock(&hbwdt->io_lock);
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_WDCLR_REG, HOBOT_TIMER_WDT_RESET);
	spin_unlock(&hbwdt->io_lock);

	return 0;
}

static int hobot_wdt_ping(struct watchdog_device *wdd)
{
	return hobot_wdt_reload(wdd);
}

static int hobot_wdt_start(struct watchdog_device *wdd)
{
	struct hobot_wdt *hbwdt = watchdog_get_drvdata(wdd);
	u32 bark_count;
	u32 bite_count;
	u32 val;

	pr_debug("\n");

	if(hbwdt->enabled) {
		pr_debug("watchdog is already started\n");
		return 0;
	}

	spin_lock(&hbwdt->io_lock);

	/* reset previos value */
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_WDCLR_REG, HOBOT_TIMER_WDT_RESET);

	/* Fill the count reg */
	bark_count = hbwdt->bark_time * timer_rate;
	bite_count = (hbwdt->bite_time - hbwdt->bark_time) * timer_rate;
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_WDTGT_REG, bark_count);
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_WDWAIT_REG, bite_count);

	/* enable wdt interrupt */
	if (hbwdt->bark_irq) {
		val = ~(hobot_wdt_rd(hbwdt, HOBOT_TIMER_TMR_INTMASK_REG));
		val |= HOBOT_TIMER_WDT_INTMASK;
		hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMR_UNMASK_REG, val);
	}

	/* Start wdt timer */
	val = hobot_wdt_rd(hbwdt, HOBOT_TIMER_TMREN_REG);
	val |= HOBOT_TIMER_T2START;
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMRSTART_REG, val);

	/* Unmask bark irq */
	val = ~(hobot_wdt_rd(hbwdt, HOBOT_TIMER_TMR_INTMASK_REG));
	val |= HOBOT_TIMER_WDT_INTMASK;
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMR_UNMASK_REG, val);

	hbwdt->enabled = true;
	spin_unlock(&hbwdt->io_lock);

	if (!kthread_disabled && !hrtimer_active(&hbwdt->pet_timer)) {
		hobot_wdt_reload(&hbwdt->hobot_wdd);
		hrtimer_start(&hbwdt->pet_timer,
			ms_to_ktime((u64)hbwdt->pet_time * 1000), HRTIMER_MODE_REL);
	}

	return 0;
}

static int hobot_wdt_settimeout(struct watchdog_device *wdd,
		unsigned int new_time)
{
	struct hobot_wdt *hbwdt = watchdog_get_drvdata(wdd);

	if (new_time > HOBOT_WDT_MAX_TIMEOUT) {
		pr_info("timeout should shorter than maxtime: %d\n",
			HOBOT_WDT_MAX_TIMEOUT);
		wdd->timeout = HOBOT_WDT_MAX_TIMEOUT;
	} else if (new_time <= hbwdt->pet_time + hbwdt->bite_interval) {
		pr_info("timeout should longer than pettime+interval: %d\n",
				hbwdt->pet_time + hbwdt->bite_interval);
		wdd->timeout = hbwdt->pet_time + hbwdt->bite_interval + 1;
	} else {
		wdd->timeout = new_time;
	}

	hbwdt->bite_time = wdd->timeout;
	hbwdt->bark_time = hbwdt->bite_time - hbwdt->bite_interval;
	wdt_timeout = wdd->timeout;

	if (!hbwdt->enabled)
		return 0;

	hobot_wdt_stop(wdd);
	pr_info("New wdt timeout time:%d\n", hbwdt->bite_time);

	return hobot_wdt_start(wdd);
}
static unsigned int hbot_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct hobot_wdt *hbwdt = watchdog_get_drvdata(wdd);
	u32 timeleft;

	timeleft = hobot_wdt_rd(hbwdt, HOBOT_TIMER_WD1D_REG) / timer_rate;
	timeleft = g_hbwdt->bark_time - timeleft + g_hbwdt->bite_interval;

	return timeleft;
}

void dump_cpu_state(void *data)
{
	pr_err("dump stack on on cpu:%d\n", smp_processor_id());
	dump_stack();

	if (panic_on_bark && on_panic == 0) {
		on_panic = 1;
		panic("Panic on watchdog bark ...");
	}
	pr_err("dump stack on on cpu:%d finished\n\n", smp_processor_id());
}

void check_other_cpus(void)
{
	int cpu;
	struct cpumask  mask;

	cpu = smp_processor_id();
	memcpy(&mask, cpu_online_mask, sizeof(struct cpumask));
	cpumask_clear_cpu(cpu, &mask);
	for_each_cpu(cpu, &mask) {
		/* nowait to skip the deadloop cpu */
		smp_call_function_single(cpu, dump_cpu_state,
			NULL, 1);
	}
}

static void dump_alive_mask(struct hobot_wdt *hbwdt)
{
    char buf[32];

    scnprintf(buf, sizeof(buf), "%*pb1",
		cpumask_pr_args(&hbwdt->alive_mask));
    pr_err("cpu alive mask since last pet %s\n\n", buf);
}

static irqreturn_t hobot_wdt_bark_irq_handler(int irq, void *data)
{
	u32 val;
	struct hobot_wdt *hbwdt = (struct hobot_wdt *)data;

	hbwdt->barking = true;

	/* clear the irq */
	val = hobot_wdt_rd(hbwdt, HOBOT_TIMER_TMR_SRCPND_REG);
	val &= HOBOT_TIMER_WDT_INTMASK;
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMR_SRCPND_REG, val);

	pr_err("hobot_wdt: bark at %lld on cpu:%d\n", sched_clock(), get_cpu());
	dump_stack();
	pr_err("hobot_wdt: check other cpus\n\n");

	/* flush cache for dump */
	printk_safe_flush_on_panic();

	if (do_ipi_ping)
		dump_alive_mask(hbwdt);

	/* may stuck here if other cpu can't response IPI */
	check_other_cpus();

	pr_err("hobot_wdt: waiting for panic on other CPU or watchdog bite ...\n");

	return IRQ_HANDLED;
}

static const struct watchdog_info hobot_wdt_info = {
	.identity = "hobot_watchdog",
	.options  = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static int hobot_wdt_restart(struct watchdog_device *wdd, unsigned long action,
						  void *data)
{
	u32 count, val;
	struct hobot_wdt *hbwdt = watchdog_get_drvdata(wdd);

	spin_lock(&hbwdt->io_lock);

	/* Fill the count reg: 100ms */
	count = timer_rate / 10;
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_WDTGT_REG, count);
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_WDWAIT_REG, count);

	/* disable wdt interrupt */
	if (hbwdt->bark_irq)
		hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMR_SETMASK_REG,
				  HOBOT_TIMER_WDT_INTMASK);

	/* Start wdt timer */
	val = hobot_wdt_rd(hbwdt, HOBOT_TIMER_TMREN_REG);
	val |= HOBOT_TIMER_T2START;
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_TMRSTART_REG, val);

	/* reset previos value */
	hobot_wdt_wr(hbwdt, HOBOT_TIMER_WDCLR_REG, HOBOT_TIMER_WDT_RESET);

	spin_unlock(&hbwdt->io_lock);

	/* wait for reset to assert... */
	mdelay(500);
	return NOTIFY_DONE;
}

static const struct watchdog_ops hobot_wdt_ops = {
	.owner = THIS_MODULE,
	.start = hobot_wdt_start,
	.stop  = hobot_wdt_stop,
	.ping  = hobot_wdt_ping,
	.set_timeout = hobot_wdt_settimeout,
	.get_timeleft = hbot_wdt_get_timeleft,
	.restart = hobot_wdt_restart,
};

static int hobot_wdt_panic_handler(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct hobot_wdt *hbwdt = container_of(this, struct hobot_wdt, panic_blk);

	if (!hbwdt->enabled)
		return 0;

	hobot_wdt_reload(&hbwdt->hobot_wdd);

	return NOTIFY_DONE;
}

static void pet_watchdog(struct hobot_wdt *hbwdt)
{
	if (hbwdt->barking)
		pr_info("bark irq triggered, timer is still running\n");

	hobot_wdt_reload(&hbwdt->hobot_wdd);
}

/*
 * This timer handler wakes up thread, then thread pet the watchdog
 */
static enum hrtimer_restart pet_timer_wakeup(struct hrtimer *hrt)
{
	struct hobot_wdt *hbwdt =
		container_of(hrt, struct hobot_wdt, pet_timer);

	pr_debug("\n");

	hbwdt->timer_expired = true;
	hbwdt->last_pet = sched_clock();
#ifdef CONFIG_PREEMPT_RT_FULL
	swake_up(&hbwdt->pet_wait);
#else
	wake_up(&hbwdt->pet_wait);
#endif

	return HRTIMER_NORESTART;
}

static void ipi_report_alive(void *data)
{
	struct hobot_wdt *hbwdt = (struct hobot_wdt *)data;
	int cpu = smp_processor_id();

	cpumask_set_cpu(cpu, &hbwdt->alive_mask);
	hbwdt->ping_end[cpu] = sched_clock();
	smp_mb();
	pr_debug("alive on cpu:%d\n", cpu);
}

static void hobot_wdt_ipi_ping(struct hobot_wdt *hbwdt)
{
	int cpu;

	cpumask_clear(&hbwdt->alive_mask);
	for_each_cpu(cpu, cpu_online_mask) {
		hbwdt->ping_start[cpu] = sched_clock();
		/* wait until ipi is reponsed */
		smp_call_function_single(cpu, ipi_report_alive, hbwdt, 1);
	}
}

static __ref int watchdog_kthread(void *arg)
{
	struct hobot_wdt *hbwdt = (struct hobot_wdt *)arg;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO-1};
	int ret;

	sched_setscheduler(current, SCHED_FIFO, &param);

	while (!kthread_should_stop()) {
		do {
#ifdef CONFIG_PREEMPT_RT_FULL
			ret = swait_event_interruptible(hbwdt->pet_wait,
						hbwdt->timer_expired);
#else
			ret = wait_event_interruptible(hbwdt->pet_wait,
						hbwdt->timer_expired);
#endif
		} while (ret != 0);

		hbwdt->thread_wakeup_time = sched_clock();
		pr_debug("thread_wakeup_time: %llu\n", hbwdt->thread_wakeup_time);

		if (do_ipi_ping)
			hobot_wdt_ipi_ping(hbwdt);

		hbwdt->timer_expired = false;

		if (hbwdt->enabled) {
			pet_watchdog(hbwdt);
			hrtimer_start(&hbwdt->pet_timer,
				ms_to_ktime((u64)hbwdt->pet_time * 1000), HRTIMER_MODE_REL);
		}
	}

	return 0;
}

static int hobot_wdog_dt_to_pdata(struct platform_device *pdev,
					struct hobot_wdt *hbwdt)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	hbwdt->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hbwdt->regs_base)) {
		ret = PTR_ERR(hbwdt->regs_base);
		return ret;
	}

	hbwdt->bark_irq = platform_get_irq(pdev, 0);
	if (hbwdt->bark_irq < 0) {
		dev_err(&pdev->dev, "failed to get bark_irq\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(node, "bark-time", &hbwdt->bark_time);
	if (ret) {
		dev_err(&pdev->dev, "reading bark time failed\n");
		return -ENXIO;
	}
	ret = of_property_read_u32(node, "bite-time", &hbwdt->bite_time);
	if (ret) {
		dev_err(&pdev->dev, "reading bite time failed\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(node, "pet-time", &hbwdt->pet_time);
	if (ret) {
		dev_err(&pdev->dev, "reading pet time failed\n");
		return -ENXIO;
	}

	/*
	 * if bite_time is changed with new wdt_timeout, bark_time is adjusted
	 * with this interval accordingly
	 */
	hbwdt->bite_interval = hbwdt->bite_time - hbwdt->bark_time;

	dev_info(&pdev->dev, "watchdog setting [pet:%ds bark:%ds bite:%ds]\n",
		hbwdt->pet_time, hbwdt->bark_time, hbwdt->bite_time);

	return 0;
}
/**
 * hobot_wdt_probe - Probe call for the device.
 *
 * @pdev: handle to the platform device structure.
 * Return: 0 on success, negative error otherwise.
 *
 * It does all the memory allocation and registration for the device.
 */
static int hobot_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct hobot_wdt *hbwdt;
	struct watchdog_device *hobot_wdd;
	u64 expired_time;

	hbwdt = devm_kzalloc(&pdev->dev, sizeof(*hbwdt), GFP_KERNEL);
	if (!hbwdt)
		return -ENOMEM;

	g_hbwdt = hbwdt;

	ret = hobot_wdog_dt_to_pdata(pdev, hbwdt);
	if (ret)
		goto err;

	hobot_wdd = &hbwdt->hobot_wdd;
	hobot_wdd->info = &hobot_wdt_info;
	hobot_wdd->ops = &hobot_wdt_ops;
	hobot_wdd->timeout = hbwdt->bite_time;
	hobot_wdd->min_timeout = HOBOT_WDT_MIN_TIMEOUT;
	hobot_wdd->max_timeout = HOBOT_WDT_MAX_TIMEOUT;

	hobot_wdt_init_hw(hbwdt);

	if (hbwdt->bark_irq >= 0) {
		ret = devm_request_irq(&pdev->dev, hbwdt->bark_irq,
		hobot_wdt_bark_irq_handler, 0,
								pdev->name, hbwdt);
		if (ret) {
			dev_err(&pdev->dev, "can't register interrupt handler err=%d\n", ret);
			goto err;
		}
	}

	hbwdt->clock = devm_clk_get(&pdev->dev, "watchdog_mclk");
	if (IS_ERR(hbwdt->clock)) {
		dev_err(&pdev->dev, "failed to find watchdog clock source\n");
		ret = PTR_ERR(hbwdt->clock);
		goto err;
	}

	ret = clk_prepare_enable(hbwdt->clock);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err;
	}

	timer_rate = clk_get_rate(hbwdt->clock);
	if (!timer_rate) {
		pr_err("failed to get watchdog clock rate\n");
		return -1;
	}

	/* Initialize the members of hobot_wdt structure */
	hobot_wdd->parent = &pdev->dev;

	wdt_timeout = hbwdt->bite_time;
	ret = watchdog_init_timeout(hobot_wdd, wdt_timeout, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "unable to set timeout value.\n");
		goto err_clk;
	}

	watchdog_set_nowayout(hobot_wdd, nowayout);
	watchdog_set_restart_priority(hobot_wdd, 192);
	watchdog_stop_on_reboot(hobot_wdd);
	watchdog_set_drvdata(hobot_wdd, hbwdt);

	spin_lock_init(&hbwdt->io_lock);

	ret = watchdog_register_device(hobot_wdd);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register wdt device.\n");
		goto err_clk;
	}

	platform_set_drvdata(pdev, hbwdt);

	/* attach hobot_watchdog on CPU0 */
	hbwdt->watchdog_thread = kthread_create_on_cpu(watchdog_kthread,
		hbwdt, 0, "hw_watchdog");
	if (IS_ERR(hbwdt->watchdog_thread)) {
		ret = PTR_ERR(hbwdt->watchdog_thread);
		goto err;
	}

	expired_time = msecs_to_jiffies(hbwdt->pet_time * 1000);

	hbwdt->panic_blk.notifier_call = hobot_wdt_panic_handler;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &hbwdt->panic_blk);

#ifdef CONFIG_PREEMPT_RT_FULL
	init_swait_queue_head(&hbwdt->pet_wait);
#else
	init_waitqueue_head(&hbwdt->pet_wait);
#endif
	hbwdt->timer_expired = false;


	wake_up_process(hbwdt->watchdog_thread);

	hrtimer_init(&hbwdt->pet_timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL_PINNED_HARD);
	hbwdt->pet_timer.function = pet_timer_wakeup;
	hrtimer_cancel(&hbwdt->pet_timer);

	if (!kthread_disabled) {
		hobot_wdt_start(&hbwdt->hobot_wdd);
		hbwdt->last_pet = sched_clock();
	}

	dev_info(&pdev->dev, "Hobot Watchdog Timer at %p with timeout %ds%s\n",
			hbwdt->regs_base, hobot_wdd->timeout, nowayout?",nowayout":"");

	return 0;

 err_clk:
	clk_disable_unprepare(hbwdt->clock);

 err:
	return ret;
}

/**
 * hobot_wdt_remove - Probe call for the device.
 *
 * @pdev: handle to the platform device structure.
 * Return: 0 on success, otherwise negative error.
 *
 * Unregister the device after releasing the resources.
 */
static int hobot_wdt_remove(struct platform_device *pdev)
{
	struct hobot_wdt *hbwdt = platform_get_drvdata(pdev);

	hobot_wdt_stop(&hbwdt->hobot_wdd);
	watchdog_unregister_device(&hbwdt->hobot_wdd);
	clk_disable_unprepare(hbwdt->clock);

	return 0;
}

/**
 * hobot_wdt_shutdown - Stop the device.
 *
 * @pdev: handle to the platform structure.
 *
 */
static void hobot_wdt_shutdown(struct platform_device *pdev)
{
	struct hobot_wdt *hbwdt = platform_get_drvdata(pdev);

	hobot_wdt_stop(&hbwdt->hobot_wdd);
}

#ifdef CONFIG_PM
int hobot_wdt_suspend(struct device *dev)
{
	struct hobot_wdt *hbwdt = dev_get_drvdata(dev);

	if (kthread_disabled)
		return 0;

	if (hbwdt->enabled)
		hobot_wdt_stop(&hbwdt->hobot_wdd);

	hbwdt->enabled = false;
	return 0;
}

int hobot_wdt_resume(struct device *dev)
{
	struct hobot_wdt *hbwdt = dev_get_drvdata(dev);

	if (kthread_disabled)
		return 0;

	if (!hbwdt->enabled)
		hobot_wdt_start(&hbwdt->hobot_wdd);

	hbwdt->enabled = true;
	return 0;
}
#endif

static const struct dev_pm_ops hobot_wdt_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hobot_wdt_suspend,
			hobot_wdt_resume)
};

static const struct of_device_id hobot_wdt_of_match[] = {
	{ .compatible = "hobot,hobot-wdt", },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, hobot_wdt_of_match);

/* Driver Structure */
static struct platform_driver hobot_wdt_driver = {
	.probe		= hobot_wdt_probe,
	.remove 	= hobot_wdt_remove,
	.shutdown	= hobot_wdt_shutdown,
	.driver 	= {
		.name	= HOBOT_WDT_NAME,
		.of_match_table = hobot_wdt_of_match,
		.pm = &hobot_wdt_dev_pm_ops,
	},
};

module_platform_driver(hobot_wdt_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("Hobot Watchdog driver");
MODULE_LICENSE("GPL v2");
