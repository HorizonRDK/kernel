/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * Zhang Guoying <guoying.zhang@horizon.ai>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#include <asm/delay.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>
#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
#include <linux/devfreq_cooling.h>
#endif
#include "bpu_ctrl.h"

int32_t bpu_core_pend_on(struct bpu_core *core)
{
	if (core == NULL) {
		pr_err("Pend on invalid core!\n");
		return -EINVAL;
	}

	atomic_set(&core->pend_flag, 1);

	return 0;
}

int32_t bpu_core_pend_off(struct bpu_core *core)
{
	if (core == NULL) {
		pr_err("Pend off invalid core!\n");
		return -EINVAL;
	}

	atomic_set(&core->pend_flag, 0);
	bpu_prio_trig_out(core->prio_sched);

	return 0;
}

/*
 * check if bpu core is pending by pend func, wait
 * timeout(jiffes) for release pending.
 */
int32_t bpu_core_is_pending(struct bpu_core *core)
{
	if (core == NULL) {
		pr_err("Check invalid core!\n");
		return -EINVAL;
	}

	return atomic_read(&core->pend_flag);
}
/*
 * use to wait bpu leisure (task in hwfifo done,
 * and pending new task to hwfifo, if can wait to
 * leisure, timeout is jiffes)
 */
int32_t bpu_core_pend_to_leisure(struct bpu_core *core, int32_t timeout)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("Pend to leisure invalid core!\n");
		return -EINVAL;
	}

	ret = bpu_core_pend_on(core);
	if (ret) {
		pr_err("Pend on to leisure fail!\n");
		return -EINVAL;
	}

	while (core->running_task_num > 0) {
		if (timeout > 0) {
			if(!wait_for_completion_timeout(
						&core->no_task_comp, timeout))
				core->running_task_num--;
		} else
			wait_for_completion(&core->no_task_comp);
	}

	return ret;
}

int32_t bpu_core_enable(struct bpu_core *core)
{
	int32_t err, ret = 0;

	if (core == NULL) {
		pr_err("Enable invalid core!\n");
		return -EINVAL;
	}

	if (core->hw_enabled)
		return 0;

	ret = bpu_core_pend_to_leisure(core, HZ);
	if (ret) {
		dev_err(core->dev, "Pend for Disable core failed!\n");
		return ret;
	}

	if (core->hw_ops->enable)
		ret = core->hw_ops->enable(core);
	else {
		if (core->regulator) {
			ret = regulator_enable(core->regulator);
			if (ret != 0) {
				dev_err(core->dev,
						"bpu core[%d] enable error\n", core->index);
			}
		}

		if (core->aclk) {
			if (!__clk_is_enabled(core->aclk)) {
				ret = clk_prepare_enable(core->aclk);
				if (ret)
					dev_err(core->dev,
							"bpu core[%d] enable aclk error\n", core->index);
			}
		}

		if (core->mclk) {
			if (!__clk_is_enabled(core->mclk)) {
				ret = clk_prepare_enable(core->mclk);
				if (ret)
					dev_err(core->dev,
							"bpu core[%d] enable mclk error\n", core->index);
			}
		}
	}

	enable_irq(core->irq);
	core->hw_enabled = 1;

	err = bpu_core_pend_off(core);
	if (ret) {
		dev_err(core->dev, "Pend off for Disable core failed!\n");
		return err;
	}

	return ret;
}
EXPORT_SYMBOL(bpu_core_enable);

int32_t bpu_core_disable(struct bpu_core *core)
{
	int32_t err, ret = 0;

	if (core == NULL) {
		pr_err("Disable invalid core!\n");
		return -EINVAL;
	}
	if (core->hw_enabled == 0)
		return 0;

	core->hw_enabled = 0;
	/*
	 * need wait running fc task done to prevent
	 * BPU or system problem
	 */
	ret = bpu_core_pend_to_leisure(core, HZ);
	if (ret) {
		dev_err(core->dev, "Pend for Disable core failed!\n");
		return ret;
	}
	disable_irq(core->irq);

	if (core->hw_ops->disable)
		ret = core->hw_ops->disable(core);
	else {
		if (core->aclk) {
			if (__clk_is_enabled(core->aclk))
				clk_disable_unprepare(core->aclk);
		}

		if (core->mclk) {
			if (!__clk_is_enabled(core->mclk))
				clk_disable_unprepare(core->mclk);
		}

		if (core->regulator) {
			ret = regulator_disable(core->regulator);
			if (ret != 0)
				dev_err(core->dev,
						"bpu core[%d] disable error\n", core->index);
		}
	}

	err = bpu_core_pend_off(core);
	if (ret) {
		dev_err(core->dev, "Pend off for Disable core failed!\n");
		return err;
	}

	return ret;
}
EXPORT_SYMBOL(bpu_core_disable);

int32_t bpu_core_reset(struct bpu_core *core)
{
	int32_t ret;
	int32_t i;

	if (core == NULL) {
		pr_err("Reset invalid core!\n");
		return -EINVAL;
	}

	for (i = 0; i < BPU_PRIO_NUM; i++)
		kfifo_reset(&core->run_fc_fifo[i]);
	kfifo_reset(&core->done_fc_fifo);

	if (core->hw_ops->reset)
		return core->hw_ops->reset(core);

	bpu_core_disable(core);
	bpu_core_enable(core);

	if (core->rst) {
		ret = reset_control_assert(core->rst);
		if (ret < 0) {
			dev_err(core->dev, "bpu core reset assert failed\n");
			return ret;
		}
		udelay(1);
		ret = reset_control_deassert(core->rst);
		if (ret < 0) {
			dev_err(core->dev, "bpu core reset deassert failed\n");
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL(bpu_core_reset);

int32_t bpu_core_process_recover(struct bpu_core *core)
{
	struct bpu_fc tmp_bpu_fc;
	struct kfifo recovery_kfifo[BPU_PRIO_NUM];
	unsigned long flags;
	int32_t ret, i;

	if (core == NULL) {
		dev_err(core->dev, "TO recovery no bpu core\n");
		return -ENODEV;
	}

	dev_err(core->dev, "TO recovery bpu core%d\n", core->index);
	/* copy run_fc_fifo for recover */
	for (i = BPU_PRIO_NUM - 1; i >= 0; i--) {
		memcpy(&recovery_kfifo[i],
				&core->run_fc_fifo[i], sizeof(recovery_kfifo[i]));
		while (kfifo_len(&recovery_kfifo[i])) {
			ret = kfifo_get(&recovery_kfifo[i], &tmp_bpu_fc);
			if (ret < 1) {
				dev_err(core->dev,
						"Get recovery bpu fc failed in BPU Core%d\n",
						core->index);
				return -EINVAL;
			}

			if (core->hw_ops->write_fc) {
				spin_lock_irqsave(&core->spin_lock, flags);
				ret = core->hw_ops->write_fc(core, &tmp_bpu_fc, 0);
				if (ret < 0) {
					spin_unlock_irqrestore(&core->spin_lock, flags);
					dev_err(core->dev, "TO recovery bpu core%d\n",
							core->index);
					return ret;
				}
				spin_unlock_irqrestore(&core->spin_lock, flags);
			}
		}
	}
	
	return 0;
}
EXPORT_SYMBOL(bpu_core_process_recover);

int32_t bpu_core_set_volt(struct bpu_core *core, int32_t volt)
{
	int32_t err, ret = 0;

	if (core == NULL) {
		pr_err("Invalid core set volt!\n");
		return -EINVAL;
	}

	if (volt == 0) {
		ret = bpu_core_disable(core);
		if (ret) {
			pr_err("Diable bpu core%d set volt!\n", core->index);
			return ret;
		}
	}

	if (core->hw_ops->set_volt) {
		ret = bpu_core_pend_to_leisure(core, HZ);
		if (ret) {
			dev_err(core->dev, "Pend for core volt change failed!\n");
			return ret;
		}

		ret = core->hw_ops->set_volt(core, volt);
		if (ret)
			dev_err(core->dev, "BPU Core set volt %d failed!\n", volt);

		err = bpu_core_pend_off(core);
		if (err) {
			dev_err(core->dev, "Pend off from core volt change failed!\n");
			return err;
		}
	}

	return ret;
}
EXPORT_SYMBOL(bpu_core_set_volt);

int32_t bpu_core_set_clk(struct bpu_core *core, uint64_t rate)
{
	int32_t err, ret = 0;

	if (core == NULL) {
		pr_err("Invalid core set volt!\n");
		return -EINVAL;
	}

	if (core->hw_ops->set_clk) {
		ret = bpu_core_pend_to_leisure(core, HZ);
		if (ret) {
			dev_err(core->dev, "Pend for core clk change failed!\n");
			return ret;
		}

		ret = core->hw_ops->set_clk(core, rate);
		if (ret)
			dev_err(core->dev, "BPU Core set clk to %lld failed!\n", rate);

		err = bpu_core_pend_off(core);
		if (err) {
			dev_err(core->dev, "Pend off from core clk change failed!\n");
			return err;
		}
	}

	return ret;
}
EXPORT_SYMBOL(bpu_core_set_clk);

#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
static int bpu_core_set_freq(struct device *dev,
		unsigned long *freq, u32 flags)
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	int64_t rate, target_volt, target_rate;
	int32_t err = 0;

	/* FIXME: when change freq, bpu should not set fc */

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp)) {
		err = PTR_ERR(opp);
		goto opp_err;
	}
	rate = dev_pm_opp_get_freq(opp);
	target_volt = dev_pm_opp_get_voltage(opp);

	target_rate = clk_round_rate(core->mclk, rate);
	if (target_rate <= 0)
		target_rate = rate;

	if (core->dvfs->rate == target_rate) {
		if (core->dvfs->volt != target_volt) {
			err = bpu_core_set_volt(core, target_volt);
			if (err) {
				dev_err(dev, "Cannot set voltage %llu uV\n",
						target_volt);
				goto out;
			}
			core->dvfs->volt = target_volt;
			goto out;
		}
	} else {
		/*
		 * To higher rate: need set volt first
		 * To lower rate: need set rate first
		 */
		if (core->dvfs->rate < target_rate) {
			err = bpu_core_set_volt(core, target_volt);
			if (err) {
				dev_err(dev, "Cannot set voltage %llu uV\n",
						target_volt);
				goto out;
			}
		}

		err = bpu_core_set_clk(core, target_rate);
		if (err) {
			dev_err(dev, "Cannot set frequency %llu (%d)\n",
					target_rate, err);
			bpu_core_set_volt(core, core->dvfs->rate);
			goto out;
		}

		if (core->dvfs->rate > target_rate) {
			err = bpu_core_set_volt(core, target_volt);
			if (err) {
				dev_err(dev, "Cannot set vol %llu uV\n", target_volt);
				goto out;
			}
		}

		core->dvfs->rate = target_rate;
		core->dvfs->volt = target_volt;
	}
out:
	dev_pm_opp_put(opp);
opp_err:
	return err;
}

static int bpu_core_get_freq(struct device *dev,
		unsigned long *freq)
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);

	if ((core->dvfs != NULL) && (core != NULL))
		*freq = core->dvfs->rate;

	return 0;
}

int32_t bpu_core_dvfs_register(struct bpu_core *core, const char *name)
{
	const char *gov_name;

	if (core == NULL) {
		pr_err("NO BPU Core!!\n");
		return -EINVAL;
	}

	core->dvfs = devm_kzalloc(core->dev,
			sizeof(struct bpu_core_dvfs), GFP_KERNEL);
	if (core->dvfs == NULL) {
		dev_err(core->dev, "Can't alloc BPU dvfs.\n");
		return -ENOMEM;
	}

	if (dev_pm_opp_of_add_table(core->dev)) {
		dev_err(core->dev, "Invalid operating-points in devicetree.\n");
		return -EINVAL;
	}

	core->dvfs->profile.polling_ms	= 0;
	core->dvfs->profile.target = bpu_core_set_freq;
	core->dvfs->profile.get_cur_freq = bpu_core_get_freq;
	core->dvfs->profile.initial_freq = clk_get_rate(core->mclk);

	core->dvfs->rate = core->dvfs->profile.initial_freq;
	core->dvfs->volt = regulator_get_voltage(core->regulator);

	if (of_property_read_string(core->dev->of_node, "governor", &gov_name)) {
		if (name)
			gov_name = name;
		else
			gov_name = "performance";
	}

	core->dvfs->devfreq = devm_devfreq_add_device(core->dev,
			&core->dvfs->profile, gov_name, NULL);
	if (IS_ERR(core->dvfs->devfreq)) {
		dev_err(core->dev, "Can't add dvfs to BPU core.\n");
		return PTR_ERR(core->dvfs->devfreq);
	}

	core->dvfs->devfreq->min_freq = core->dvfs->profile.freq_table[0];
	core->dvfs->devfreq->max_freq =
		core->dvfs->profile.freq_table[core->dvfs->profile.max_state
		? core->dvfs->profile.max_state - 1 : 0];
	core->dvfs->level_num = core->dvfs->profile.max_state;

	devm_devfreq_register_opp_notifier(core->dev, core->dvfs->devfreq);

	core->dvfs->cooling = of_devfreq_cooling_register(core->dev->of_node,
			core->dvfs->devfreq);

	core->power_level = 1;

	return 0;
}
EXPORT_SYMBOL(bpu_core_dvfs_register);

void bpu_core_dvfs_unregister(struct bpu_core *core)
{
	if (core == NULL)
		return;

	devfreq_cooling_unregister(core->dvfs->cooling);
	devm_devfreq_remove_device(core->dev, core->dvfs->devfreq);
	devm_devfreq_unregister_opp_notifier(core->dev, core->dvfs->devfreq);
	dev_pm_opp_of_remove_table(core->dev);

	if (core->dvfs)
		devm_kfree(core->dev, core->dvfs);
}
EXPORT_SYMBOL(bpu_core_dvfs_unregister);

int32_t bpu_core_set_freq_level(struct bpu_core *core, int32_t level)
{
	uint64_t wanted;
	int32_t ret = 0;
	int32_t i;

	if (core == NULL) {
		pr_err("Invalid core set freq level!\n");
		return -EINVAL;
	}

	if (level > 0) {
		/* if level > 0, make the governor to default gover*/
		if (core->dvfs->cooling == NULL) {
			for (i = 0; i < core->dvfs->profile.max_state; i++)
				dev_pm_opp_enable(core->dev,
						core->dvfs->profile.freq_table[i]);
			core->dvfs->cooling =
				of_devfreq_cooling_register(core->dev->of_node,
						core->dvfs->devfreq);
		}
		core->power_level = 1;
	} else {
		/* if level <= 0, user freq set*/
		if (core->dvfs->cooling != NULL) {
			devfreq_cooling_unregister(core->dvfs->cooling);
			core->dvfs->cooling = NULL;
		}

		if ((level <= (-1 * core->dvfs->level_num)) && (level != 0)) {
			dev_err(core->dev,
					"Set BPU core%d freq level(%d) lower then lowest(%d)\n",
					core->index, level, -1 * core->dvfs->level_num + 1);
			level = -1 * core->dvfs->level_num + 1;
		}

		for (i = 0; i < core->dvfs->profile.max_state; i++) {
			dev_pm_opp_disable(core->dev,
					core->dvfs->profile.freq_table[i]);
		}

		wanted = core->dvfs->profile.freq_table[core->dvfs->profile.max_state
			? core->dvfs->profile.max_state - 1 + level : 0];

		dev_pm_opp_enable(core->dev, wanted);

		mutex_lock(&core->dvfs->devfreq->lock);
		if (wanted < core->dvfs->rate)
			ret = bpu_core_set_freq(core->dev, (unsigned long *)&wanted, 0);
		else
			ret = bpu_core_set_freq(core->dev,
					(unsigned long *)&wanted, DEVFREQ_FLAG_LEAST_UPPER_BOUND);
		if (ret) {
			mutex_unlock(&core->dvfs->devfreq->lock);
			dev_err(core->dev,
					"Set BPU core%d set freq level(%d) failed\n",
					core->index, level);
			return ret;
		}
		mutex_unlock(&core->dvfs->devfreq->lock);
		core->power_level = level;
	}

	return ret;

}
EXPORT_SYMBOL(bpu_core_set_freq_level);
#endif

int32_t bpu_core_set_limit(struct bpu_core *core, int32_t limit)
{
	if (core == NULL) {
		pr_err("Invalid core set freq level!\n");
		return -EINVAL;
	}

	if ((limit < 0) || (limit > FC_MAX_DEPTH))
		limit = 0;

	core->fc_buf_limit = limit;

	return 0;
}
EXPORT_SYMBOL(bpu_core_set_limit);
