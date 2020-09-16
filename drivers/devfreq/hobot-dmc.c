/*
 * Copyright (c) 2020, Horizon Co., Ltd.
 * Author: Zhaohui Shi <zhaohui.shi@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/devfreq-event.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/rwsem.h>
#include <linux/suspend.h>

#include "governor.h"

struct hobot_dmcfreq {
	struct device *dev;
	struct devfreq *devfreq;
	struct devfreq_simple_ondemand_data ondemand_data;
	struct clk *dmc_clk;
	struct devfreq_event_dev *edev;
	struct mutex lock;

	unsigned long rate, target_rate;
};

static int hobot_dmcfreq_target(struct device *dev, unsigned long *freq,
				 u32 flags)
{
	struct hobot_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long target_rate;
	int err;

	target_rate = *freq;
	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	target_rate = *freq;
	dev_pm_opp_put(opp);

	if (dmcfreq->rate == target_rate)
		return 0;

	mutex_lock(&dmcfreq->lock);

	err = clk_set_rate(dmcfreq->dmc_clk, target_rate);
	if (err) {
		dev_err(dev, "Cannot to set frequency %lu (%d)\n",
			target_rate, err);
		goto out;
	}

	/*
	 * Check the dpll rate,
	 * There only two result we will get,
	 * 1. Ddr frequency scaling fail, we still get the old rate.
	 * 2. Ddr frequency scaling sucessful, we get the rate we set.
	 */
	dmcfreq->rate = clk_get_rate(dmcfreq->dmc_clk);

	/* If get the incorrect rate, set voltage to old value. */
	if (dmcfreq->rate != target_rate) {
		dev_err(dev, "Get wrong ddr frequency, Request frequency %lu,\
			Current frequency %lu\n", target_rate, dmcfreq->rate);
		goto out;
	}

out:
	mutex_unlock(&dmcfreq->lock);
	return err;
}

static int hobot_dmcfreq_get_dev_status(struct device *dev,
					struct devfreq_dev_status *stat)
{
	struct hobot_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	struct devfreq_event_data edata;
	int ret = 0;

	ret = devfreq_event_get_event(dmcfreq->edev, &edata);
	if (ret < 0)
		return ret;

	stat->current_frequency = dmcfreq->rate;
	stat->busy_time = edata.load_count;
	stat->total_time = edata.total_count;

	return ret;
}

static int hobot_dmcfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct hobot_dmcfreq *dmcfreq = dev_get_drvdata(dev);

	*freq = dmcfreq->rate;

	return 0;
}

static struct devfreq_dev_profile hobot_devfreq_dmc_profile = {
	.polling_ms	= 200,
	.target		= hobot_dmcfreq_target,
	.get_dev_status	= hobot_dmcfreq_get_dev_status,
	.get_cur_freq	= hobot_dmcfreq_get_cur_freq,
};

static __maybe_unused int hobot_dmcfreq_suspend(struct device *dev)
{
	struct hobot_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	int ret = 0;

	ret = devfreq_event_disable_edev(dmcfreq->edev);
	if (ret < 0) {
		dev_err(dev, "failed to disable the devfreq-event devices\n");
		return ret;
	}

	ret = devfreq_suspend_device(dmcfreq->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to suspend the devfreq devices\n");
		return ret;
	}

	return 0;
}

static __maybe_unused int hobot_dmcfreq_resume(struct device *dev)
{
	struct hobot_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	int ret = 0;

	ret = devfreq_event_enable_edev(dmcfreq->edev);
	if (ret < 0) {
		dev_err(dev, "failed to enable the devfreq-event devices\n");
		return ret;
	}

	ret = devfreq_resume_device(dmcfreq->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to resume the devfreq devices\n");
		return ret;
	}
	return ret;
}

static SIMPLE_DEV_PM_OPS(hobot_dmcfreq_pm, hobot_dmcfreq_suspend,
			hobot_dmcfreq_resume);

static int hobot_dmcfreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hobot_dmcfreq *ctx;
	int ret;
	struct dev_pm_opp *opp;
	int i, max_opps;
	unsigned long rate;

	ctx = devm_kzalloc(dev, sizeof(struct hobot_dmcfreq), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);

	ctx->dmc_clk = devm_clk_get(dev, "dmc_clk");
	if (IS_ERR(ctx->dmc_clk)) {
		dev_err(dev, "Cannot get the clk dmc_clk\n");
		return PTR_ERR(ctx->dmc_clk);
	};

	ctx->edev = devfreq_event_get_edev_by_phandle(dev, 0);
	if (IS_ERR(ctx->edev))
		return -EPROBE_DEFER;

	ret = devfreq_event_enable_edev(ctx->edev);
	if (ret < 0) {
		dev_err(dev, "failed to enable devfreq-event devices\n");
		return ret;
	}

	/*
	 * We add a devfreq driver to our parent since it has a device tree node
	 * with operating points.
	 */
	if (dev_pm_opp_of_add_table(dev)) {
		dev_err(dev, "Invalid operating-points in device tree.\n");
		return -EINVAL;
	}

	ctx->rate = clk_get_rate(ctx->dmc_clk);

	opp = devfreq_recommended_opp(dev, &ctx->rate, 0);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	ctx->rate = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);

	ret = of_property_read_u32(dev->of_node, "upthreshold",
				&ctx->ondemand_data.upthreshold);
	if (ret)
		pr_info("%s: failed to get upthreshold in dts, use default\n",
				__func__);

	ret = of_property_read_u32(dev->of_node, "downdifferential",
				&ctx->ondemand_data.downdifferential);
	if (ret)
		pr_info("%s: failed to get  downdifferential in dts, use default\n",
				__func__);

	hobot_devfreq_dmc_profile.initial_freq = ctx->rate;

	ctx->devfreq = devm_devfreq_add_device(dev,
					   &hobot_devfreq_dmc_profile,
					   "simple_ondemand",
					   &ctx->ondemand_data);
	if (IS_ERR(ctx->devfreq))
		return PTR_ERR(ctx->devfreq);

	ctx->devfreq->min_freq = ULONG_MAX;
	ctx->devfreq->max_freq = 0;

	max_opps = dev_pm_opp_get_opp_count(dev);
	if (max_opps <= 0)
		return max_opps ? max_opps : -ENODATA;

	for (i = 0, rate = 0; i < max_opps; i++, rate++) {
		opp = dev_pm_opp_find_freq_ceil(dev, &rate);
		if (IS_ERR(opp)) {
			ret = PTR_ERR(opp);
			return ret;
		}
		if (ctx->devfreq->min_freq > rate)
			ctx->devfreq->min_freq = rate;
		if (ctx->devfreq->max_freq < rate)
			ctx->devfreq->max_freq = rate;

		dev_pm_opp_put(opp);
	}

	devm_devfreq_register_opp_notifier(dev, ctx->devfreq);

	ctx->dev = dev;
	platform_set_drvdata(pdev, ctx);

	return 0;
}

static const struct of_device_id hobotdmc_devfreq_of_match[] = {
    {.compatible = "hobot,j5-dmc"},
    {.compatible = "hobot,xj3-dmc"},
    {},
};
MODULE_DEVICE_TABLE(of, hobotdmc_devfreq_of_match);

static struct platform_driver hobot_dmcfreq_driver = {
	.probe	= hobot_dmcfreq_probe,
	.driver = {
		.name	= "hobot-dmc-freq",
		.pm	= &hobot_dmcfreq_pm,
		.of_match_table = hobotdmc_devfreq_of_match,
	},
};
module_platform_driver(hobot_dmcfreq_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Zhaohui Shi <zhaohui.shi@horizon.ai>");
MODULE_DESCRIPTION("Horizon dmcfreq driver with devfreq framework");
