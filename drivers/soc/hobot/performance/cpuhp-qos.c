/*
 * cpufreq-qos.c --- Description
 *
 * Copyright (C) 2020, schspa, all rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ":QOS: " fmt
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/moduleparam.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <trace/events/power.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/pm_qos.h>

/* To handle cpufreq min/max request */
struct cpuhp_status {
	unsigned int min;
	unsigned int max;
	spinlock_t lock;
	struct task_struct *hp_thread;
	unsigned int online_cpus; /* current onlined cpu */
};

static struct cpuhp_status cpuhp_stat;

static void wake_up_cpuhp_thread(struct cpuhp_status *stat);

static int need_adjuest(struct cpuhp_status *state)
{
	if (state->online_cpus < state->min || state->online_cpus > state->max)
		return 1;

	return 0;
}

static int cpufreq_hp_offline(unsigned int online_cpu)
{
	unsigned long flags;

	spin_lock_irqsave(&cpuhp_stat.lock, flags);
	cpuhp_stat.online_cpus--;
	if (need_adjuest(&cpuhp_stat)) {
		wake_up_cpuhp_thread(&cpuhp_stat);
	}
	spin_unlock_irqrestore(&cpuhp_stat.lock, flags);

	return 0;
}

static int cpufreq_hp_online(unsigned int online_cpu)
{
	unsigned long flags;

	spin_lock_irqsave(&cpuhp_stat.lock, flags);
	cpuhp_stat.online_cpus++;
	if (need_adjuest(&cpuhp_stat)) {
		wake_up_cpuhp_thread(&cpuhp_stat);
	}
	spin_unlock_irqrestore(&cpuhp_stat.lock, flags);

	return 0;
}

static int core_ctl_online_core(unsigned int cpu)
{
	int ret;
	struct device *dev;

	dev = get_cpu_device(cpu);
	if (!dev) {
		pr_err("%s: failed to get cpu%d device\n", __func__, cpu);
		ret = -ENODEV;
	} else {
		ret = device_online(dev);
	}
	return ret;
}

static int core_ctl_offline_core(unsigned int cpu)
{
	int ret;
	struct device *dev;

	dev = get_cpu_device(cpu);
	if (!dev) {
		pr_err("%s: failed to get cpu%d device\n", __func__, cpu);
		ret = -ENODEV;
	} else {
		ret = device_offline(dev);
	}
	return ret;
}

static void __ref do_core_ctl(struct cpuhp_status *stats)
{
	unsigned long flags;
	unsigned int min, max, online;

	lock_device_hotplug();

	spin_lock_irqsave(&cpuhp_stat.lock, flags);
	min = cpuhp_stat.min;
	max = cpuhp_stat.max;
	online = cpuhp_stat.online_cpus;
	spin_unlock_irqrestore(&cpuhp_stat.lock, flags);

	if (online > max) {
		core_ctl_offline_core(online - 1);
	} else if (online < min) {
		core_ctl_online_core(online);
	}
	unlock_device_hotplug();
}

static int __ref try_core_ctl(void *data)
{
	struct cpuhp_status *stats = data;
	unsigned long flags;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&stats->lock, flags);
		/* keep running until oneline cpu number is in min ~ max */
		if (!need_adjuest(stats)) {
			spin_unlock_irqrestore(&stats->lock, flags);
			schedule();
			if (kthread_should_stop())
				break;
			spin_lock_irqsave(&stats->lock, flags);
		}
		set_current_state(TASK_RUNNING);
		spin_unlock_irqrestore(&stats->lock, flags);

		do_core_ctl(stats);
	}

	return 0;
}

static void wake_up_cpuhp_thread(struct cpuhp_status *stat)
{
	wake_up_process(stat->hp_thread);
}

static int cpuhp_pm_min_qos_callback(struct notifier_block *nb, unsigned long val,
				 void *v)
{
	unsigned long flags;

	spin_lock_irqsave(&cpuhp_stat.lock, flags);
	cpuhp_stat.min = val;
	spin_unlock_irqrestore(&cpuhp_stat.lock, flags);
	wake_up_cpuhp_thread(&cpuhp_stat);

	return 0;
}

static int cpuhp_pm_max_qos_callback(struct notifier_block *nb, unsigned long val,
				void *v)
{
	unsigned long flags;
	spin_lock_irqsave(&cpuhp_stat.lock, flags);
	cpuhp_stat.max = val;
	spin_unlock_irqrestore(&cpuhp_stat.lock, flags);
	wake_up_cpuhp_thread(&cpuhp_stat);

	return 0;
}

static struct notifier_block cpuhp_min_qos_notifier = {
	.notifier_call = cpuhp_pm_min_qos_callback,
	.priority = INT_MIN,
};

static struct notifier_block cpuhp_max_qos_notifier = {
	.notifier_call = cpuhp_pm_max_qos_callback,
	.priority = INT_MAX,
};

static int __init hb_cpuhp_qos_init(void)
{
	int ret;

	cpuhp_stat.min = 0;
	cpuhp_stat.max = NR_CPUS;
	spin_lock_init(&cpuhp_stat.lock);

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "hb_perf/qos:online",
				cpufreq_hp_online, cpufreq_hp_offline);
	if (ret < 0) {
		pr_warn("failed to register cpuhp online callbacks\n");
		return ret;
	}

	cpuhp_stat.hp_thread =
		kthread_run(try_core_ctl, &cpuhp_stat, "core_ctl");
	if (IS_ERR(cpuhp_stat.hp_thread)) {
		return PTR_ERR(cpuhp_stat.hp_thread);
	}

	pm_qos_add_notifier(PM_QOS_CPU_ONLINE_MIN, &cpuhp_min_qos_notifier);
	pm_qos_add_notifier(PM_QOS_CPU_ONLINE_MAX, &cpuhp_max_qos_notifier);

	return 0;
}
late_initcall(hb_cpuhp_qos_init);
