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
struct cpu_status {
	unsigned int min;
	unsigned int max;
};
static DEFINE_PER_CPU(struct cpu_status, cpu_stats);

static int perf_adjust_notify(struct notifier_block *nb, unsigned long val,
			      void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int cpu = policy->cpu;
	struct cpu_status *cpu_st = &per_cpu(cpu_stats, cpu);
	unsigned int min = cpu_st->min, max = cpu_st->max;

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	pr_debug("CPU%u policy before: %u:%u kHz\n", cpu, policy->min,
		 policy->max);
	pr_debug("CPU%u seting min:max %u:%u kHz\n", cpu, min, max);

	cpufreq_verify_within_limits(policy, min, max);

	pr_debug("CPU%u policy after: %u:%u kHz\n", cpu, policy->min,
		 policy->max);

	return NOTIFY_OK;
}

static struct notifier_block perf_cpufreq_nb = {
	.notifier_call = perf_adjust_notify,
};

static int cpufreq_pm_qos_callback(struct notifier_block *nb, unsigned long val,
				   void *v)
{
	struct cpufreq_policy policy;
	int pm_qos_class = *((int *)v);
	int cpu;
	int is_min;
	struct cpu_status *stats;

	is_min = pm_qos_class >= PM_QOS_CPU_FREQ_MAX ? 0 : 1;
	cpu = is_min ? pm_qos_class - PM_QOS_CPU_FREQ_MIN :
		       pm_qos_class - PM_QOS_CPU_FREQ_MAX;

	stats = &per_cpu(cpu_stats, cpu);
	if (is_min)
		stats->min = val;
	else
		stats->max = val;

	get_online_cpus();

	if (cpufreq_get_policy(&policy, cpu)) {
		goto out;
	}

	if (is_min) {
		if (cpu_online(cpu) && (policy.min != stats->min)) {
			cpufreq_update_policy(cpu);
		}
	} else {
		if (cpu_online(cpu) && (policy.max != stats->max)) {
			cpufreq_update_policy(cpu);
		}
	}

out:
	put_online_cpus();
	return 0;
}

static struct notifier_block cpufreq_min_qos_notifier = {
	.notifier_call = cpufreq_pm_qos_callback,
	.priority = INT_MIN,
};

static int __init hb_performance_init(void)
{
	unsigned int cpu;

	for_each_present_cpu (cpu)
		per_cpu(cpu_stats, cpu).max = UINT_MAX;

	pm_qos_add_notifier(PM_QOS_CPU_FREQ_MIN, &cpufreq_min_qos_notifier);
	cpufreq_register_notifier(&perf_cpufreq_nb, CPUFREQ_POLICY_NOTIFIER);

	return 0;
}
late_initcall(hb_performance_init);
