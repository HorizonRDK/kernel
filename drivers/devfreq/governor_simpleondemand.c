/*
 *  linux/drivers/devfreq/governor_simpleondemand.c
 *
 *  Copyright (C) 2011 Samsung Electronics
 *	MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/devfreq.h>
#include <linux/math64.h>
#include <linux/delay.h>
#include "governor.h"

/* Default constants for DevFreq-Simple-Ondemand (DFSO) */
#define DFSO_UPTHRESHOLD	(90)
#define DFSO_DOWNDIFFERENCTIAL	(5)

#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
static struct devfreq_simple_ondemand_data *so_data;
/*
 * Keep devfreq_simple_ondemand_data on static variable so_data since
 * the devfreq->data could be changed when switching governor.
 * This function should be called before adding simple_ondemand governor
 */
void devfreq_simple_ondemand_func_set_data(
		struct devfreq_simple_ondemand_data *data)
{
	so_data = data;
}

extern int devfreq_simple_ondemand_event_enable_disable(
		struct devfreq_simple_ondemand_data *so_data, int enable);
#endif

static int devfreq_simple_ondemand_func(struct devfreq *df,
					unsigned long *freq)
{
	int err;
	struct devfreq_dev_status *stat;
	unsigned long long a, b;
	unsigned int dfso_upthreshold = DFSO_UPTHRESHOLD;
	unsigned int dfso_downdifferential = DFSO_DOWNDIFFERENCTIAL;
	struct devfreq_simple_ondemand_data *data = df->data;
	unsigned long max = (df->max_freq) ? df->max_freq : UINT_MAX;

	err = devfreq_update_stats(df);
	if (err)
		return err;

	stat = &df->last_status;

	if (data) {
		if (data->upthreshold)
			dfso_upthreshold = data->upthreshold;
		if (data->downdifferential)
			dfso_downdifferential = data->downdifferential;
	}

	if (dfso_upthreshold > 100 ||
	    dfso_upthreshold < dfso_downdifferential)
		return -EINVAL;

	/* Assume MAX if it is going to be divided by zero */
	if (stat->total_time == 0) {
		*freq = max;
		return 0;
	}

	/* Prevent overflow */
	if (stat->busy_time >= (1 << 24) || stat->total_time >= (1 << 24)) {
		stat->busy_time >>= 7;
		stat->total_time >>= 7;
	}

	/* Set MAX if it's busy enough */
	if (stat->busy_time * 100 >
	    stat->total_time * dfso_upthreshold) {
		*freq = max;
		return 0;
	}

	/* Set MAX if we do not know the initial frequency */
	if (stat->current_frequency == 0) {
		*freq = max;
		return 0;
	}

	/* Keep the current frequency */
	if (stat->busy_time * 100 >
	    stat->total_time * (dfso_upthreshold - dfso_downdifferential)) {
		*freq = stat->current_frequency;
		return 0;
	}

	/* Set the desired frequency based on the load */
	a = stat->busy_time;
	a *= stat->current_frequency;
	b = div_u64(a, stat->total_time);
	b *= 100;
	b = div_u64(b, (dfso_upthreshold - dfso_downdifferential / 2));
	*freq = (unsigned long) b;

	if (df->min_freq && *freq < df->min_freq)
		*freq = df->min_freq;
	if (df->max_freq && *freq > df->max_freq)
		*freq = df->max_freq;

	return 0;
}

static int devfreq_simple_ondemand_handler(struct devfreq *devfreq,
				unsigned int event, void *data)
{
	switch (event) {
	case DEVFREQ_GOV_START:
#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
		/* recover simple_ondemand_data since it could be lost when
		 * switching from other governor.
		 */
		if (so_data != NULL) {
			devfreq->data = so_data;
			devfreq_simple_ondemand_event_enable_disable(so_data, 1);
			msleep(200);
		}
#endif
		devfreq_monitor_start(devfreq);

		break;

	case DEVFREQ_GOV_STOP:
		devfreq_monitor_stop(devfreq);

#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
		if (so_data != NULL)
			devfreq_simple_ondemand_event_enable_disable(so_data, 0);
#endif
		break;

	case DEVFREQ_GOV_INTERVAL:
		devfreq_interval_update(devfreq, (unsigned int *)data);
		break;

	case DEVFREQ_GOV_SUSPEND:
		devfreq_monitor_suspend(devfreq);
		break;

	case DEVFREQ_GOV_RESUME:
		devfreq_monitor_resume(devfreq);
		break;

	default:
		break;
	}

	return 0;
}

static struct devfreq_governor devfreq_simple_ondemand = {
	.name = "simple_ondemand",
	.get_target_freq = devfreq_simple_ondemand_func,
	.event_handler = devfreq_simple_ondemand_handler,
};

static int __init devfreq_simple_ondemand_init(void)
{
	return devfreq_add_governor(&devfreq_simple_ondemand);
}
subsys_initcall(devfreq_simple_ondemand_init);

static void __exit devfreq_simple_ondemand_exit(void)
{
	int ret;

	ret = devfreq_remove_governor(&devfreq_simple_ondemand);
	if (ret)
		pr_err("%s: failed remove governor %d\n", __func__, ret);

	return;
}
module_exit(devfreq_simple_ondemand_exit);
MODULE_LICENSE("GPL");
