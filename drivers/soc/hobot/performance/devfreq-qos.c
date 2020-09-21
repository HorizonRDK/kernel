/*
 * devfreq-qos.c --- Description
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
#include <linux/devfreq.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <trace/events/power.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/pm_qos.h>
#include <linux/of_device.h>
#include <linux/devfreq.h>

#include <governor.h>

struct hb_devfreq_qos_data {
	struct pm_qos_constraints cons;
	struct blocking_notifier_head notifiers;
	struct notifier_block notifier;
	struct devfreq *df;
	unsigned long limit, orig_limit, max_freq;
	bool type_min;
};

static int devfreq_pm_qos_callback(struct notifier_block *nb, unsigned long val,
				void *v)
{
	struct hb_devfreq_qos_data *qos_data =
		container_of(nb, struct hb_devfreq_qos_data, notifier);
	unsigned long freq;

	freq = qos_data->max_freq * val / PM_QOS_DEVFREQ_MAX_VALUE;

	devfreq_recommended_opp(qos_data->df->dev.parent, &freq,
				qos_data->cons.type == PM_QOS_MIN ?
				0 : DEVFREQ_FLAG_LEAST_UPPER_BOUND);
	qos_data->limit = freq;
	if (qos_data->type_min) {
		qos_data->df->min_freq = freq;
	} else {
		qos_data->df->max_freq = freq;
	}
	devfreq_update_stats(qos_data->df);

	mutex_lock(&qos_data->df->lock);
	(void) update_devfreq(qos_data->df);
	mutex_unlock(&qos_data->df->lock);

	return 0;
}

static int hb_devfreq_qos_probe(struct platform_device *pdev)
{
	struct hb_devfreq_qos_data *qos_data;
	int classid, ret;
	struct blocking_notifier_head ;

	qos_data = devm_kzalloc(&pdev->dev, sizeof(*qos_data), GFP_KERNEL);
	if (!qos_data)
		return -ENOMEM;
	qos_data->df = devfreq_get_devfreq_by_phandle(&pdev->dev, 0);
	if (IS_ERR(qos_data->df)) {
		ret = -EPROBE_DEFER;
		goto free_data;
	}

	qos_data->type_min = of_property_read_bool(pdev->dev.of_node,
						"devfreq-qos,type-min");
	if (qos_data->type_min) {
		qos_data->cons.default_value = PM_QOS_DEVFREQ_MIN_DEFAULT_VALUE;
		qos_data->cons.no_constraint_value =
			PM_QOS_DEVFREQ_MIN_DEFAULT_VALUE;
		qos_data->cons.target_value = PM_QOS_DEVFREQ_MIN_DEFAULT_VALUE;
		qos_data->cons.type = PM_QOS_MIN;
		qos_data->orig_limit = qos_data->df->min_freq;
	} else {
		qos_data->cons.default_value = PM_QOS_DEVFREQ_MAX_DEFAULT_VALUE;
		qos_data->cons.no_constraint_value =
			PM_QOS_DEVFREQ_MAX_DEFAULT_VALUE;
		qos_data->cons.target_value = PM_QOS_DEVFREQ_MAX_DEFAULT_VALUE;
		qos_data->cons.type = PM_QOS_MAX;
		qos_data->orig_limit = qos_data->df->max_freq;
	}
	plist_head_init(&qos_data->cons.list);
	qos_data->cons.notifiers = &qos_data->notifiers;
	qos_data->max_freq = qos_data->df->max_freq;

	qos_data->limit = qos_data->df->max_freq * qos_data->cons.default_value
		/ PM_QOS_DEVFREQ_MAX_DEFAULT_VALUE;

	pr_debug("%s: limit: %lu\n", dev_name(&pdev->dev), qos_data->limit);

	classid = pm_qos_class_register(dev_name(&pdev->dev), &qos_data->cons);
	if (classid < 0) {
		goto free_data;
	}

	qos_data->notifier.notifier_call = devfreq_pm_qos_callback;
	pm_qos_add_notifier(classid, &qos_data->notifier);

	return 0;

free_data:
	return ret;
}

static int hb_devfreq_qos_remove(struct platform_device *pdev)
{
	/* TODO: support remove devfreq_device */
	return 0;
}

static const struct of_device_id hb_devfreq_qos_dt_ids[] = {
	{ .compatible = "hobot,devfreq_qos", },
	{ }
};
MODULE_DEVICE_TABLE(of, hb_devfreq_qos_dt_ids);

static struct platform_driver hb_devfreq_qos_driver = {
	.probe		= hb_devfreq_qos_probe,
	.remove		= hb_devfreq_qos_remove,
	.driver		= {
		.name	= "hb_devfreq_qos",
		.of_match_table = hb_devfreq_qos_dt_ids,
	},
};
module_platform_driver(hb_devfreq_qos_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("pm_qos driver for devfreq");
MODULE_AUTHOR("Zhaohui.shi <zhaohui.shi@horizon.ai>");
