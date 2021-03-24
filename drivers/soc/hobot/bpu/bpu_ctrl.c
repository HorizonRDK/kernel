/*
 * Copyright (C) 2019 Horizon Robotics
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

static uint8_t bpu_core_type(struct bpu_core *core)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("BPY Core TYPE on invalid core!\n");/*PRQA S ALL*/
		return 0;
	}

	if (core->hw_ops->status != NULL) {
		ret = core->hw_ops->status(core, TYPE_STATE);
		if (ret < 0) {
			dev_err(core->dev, "Get Invalid Core Type!\n");
				return 0;
		}
		return (uint8_t)ret;
	} else {
		return CORE_TYPE_ANY;
	}
}

static int32_t bpu_core_pend_on(struct bpu_core *core)
{
	if (core == NULL) {
		pr_err("Pend on invalid core!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	atomic_set(&core->pend_flag, 1);/*PRQA S ALL*/

	return 0;
}

static int32_t bpu_core_pend_off(struct bpu_core *core)
{
	if (core == NULL) {
		pr_err("Pend off invalid core!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	atomic_set(&core->pend_flag, 0);/*PRQA S ALL*/
	bpu_prio_trig_out(core->prio_sched);

	return 0;
}

/*
 * check if bpu core is pending by pend func, wait
 * timeout(jiffes) for release pending.
 */
int32_t bpu_core_is_pending(const struct bpu_core *core)
{
	if (core == NULL) {
		pr_err("Check invalid core!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	return atomic_read(&core->pend_flag);/*PRQA S ALL*/
}
/*
 * use to wait bpu leisure (task in hwfifo done,
 * and pending new task to hwfifo, if can wait to
 * leisure, timeout is jiffes)
 */
static int32_t bpu_core_pend_to_leisure(struct bpu_core *core, int32_t timeout)
{
	int32_t core_work_state = 1;
	int32_t ret;

	if (core == NULL) {
		pr_err("Pend to leisure invalid core!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	ret = bpu_core_pend_on(core);
	if (ret != 0) {
		pr_err("Pend on to leisure fail!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	while (core->running_task_num > 0) {
		if (core->hw_ops->status != NULL) {
			(void)core->hw_ops->status(core, UPDATE_STATE);
			core_work_state = core->hw_ops->status(core, WORK_STATE);
			if (core_work_state == 0) {
				/* if core do not work, just break wait */
				break;
			}
		}
		if (timeout > 0) {
			if(wait_for_completion_timeout(
					&core->no_task_comp, (uint32_t)timeout) == 0u) {
				if (core->hw_ops->status != NULL) {
					(void)core->hw_ops->status(core, UPDATE_STATE);
					ret = core->hw_ops->status(core, WORK_STATE);
					if (ret == core_work_state) {
						/* if states between wait are same, break to not wait*/
						ret = 0;
						break;
					}
				}
				core->running_task_num--;
			}
		} else {
			wait_for_completion(&core->no_task_comp);
		}
	}

	return ret;
}
int32_t bpu_core_clk_on(const struct bpu_core *core)
{
	int32_t ret = 0;

	if (core == NULL) {
		pr_err("To Clock on Invalid BPU Core\n");/*PRQA S ALL*/
		return ret;
	}

	if (core->aclk != NULL) {
		if (!__clk_is_enabled(core->aclk)) {
			ret = clk_prepare_enable(core->aclk);
			if (ret != 0) {
				dev_err(core->dev,
						"bpu core[%d] aclk enable failed\n",
						core->index);
			}
		}
	}

	if (core->mclk != NULL) {
		if (!__clk_is_enabled(core->mclk)) {
			ret = clk_prepare_enable(core->mclk);
			if (ret != 0) {
				dev_err(core->dev,
						"bpu core[%d] mclk enable failed\n",
						core->index);
			}
		}
	}

	return ret;
}
EXPORT_SYMBOL(bpu_core_clk_on);/*PRQA S ALL*/

int32_t bpu_core_clk_off(const struct bpu_core *core)
{
	int32_t ret = 0;

	if (core == NULL) {
		return ret;
	}

	if (core->aclk != NULL) {
		if (__clk_is_enabled(core->aclk)) {
			clk_disable_unprepare(core->aclk);
		}
	}

	if (core->mclk != NULL) {
		if (__clk_is_enabled(core->mclk)) {
			clk_disable_unprepare(core->mclk);
		}
	}

	return ret;
}
EXPORT_SYMBOL(bpu_core_clk_off);/*PRQA S ALL*/

int32_t bpu_core_power_on(const struct bpu_core *core)
{
	int32_t ret = 0;

	if (core == NULL) {
		pr_err("To power on Invalid BPU Core\n");/*PRQA S ALL*/
		return ret;
	}

	if (core->regulator != NULL) {
		ret = regulator_enable(core->regulator);
		if (ret != 0) {
			dev_err(core->dev,
					"bpu core[%d] enable error\n", core->index);
		}
	}

	return ret;
}
EXPORT_SYMBOL(bpu_core_power_on);/*PRQA S ALL*/

int32_t bpu_core_power_off(const struct bpu_core *core)
{
	int32_t ret = 0;

	if (core == NULL) {
		return ret;
	}

	if (core->regulator != NULL) {
		ret = regulator_disable(core->regulator);
		if (ret != 0) {
			dev_err(core->dev,
					"bpu core[%d] regulator disable failed\n",
					core->index);
		}
	}

	return ret;
}
EXPORT_SYMBOL(bpu_core_power_off);/*PRQA S ALL*/

int32_t bpu_core_enable(struct bpu_core *core)
{
	int32_t err, ret;

	if (core == NULL) {
		pr_err("Enable invalid core!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	if (core->hw_enabled > 0u) {
		return 0;
	}

#ifdef CONFIG_X3_BPU
	pm_qos_add_request(&core->pm_qos_req, PM_QOS_DEVFREQ, 10000);
#endif

	if (core->hw_ops->enable != NULL) {
		ret = core->hw_ops->enable(core);
	} else {
		ret = bpu_core_power_on(core);
		ret += bpu_core_clk_on(core);
	}

	enable_irq(core->irq);
	core->hw_enabled = 1;

	err = bpu_core_pend_off(core);
	if (ret != 0) {
		dev_err(core->dev, "Pend off for Disable core failed!\n");
		return err;
	}

	bpu_prio_plug_in(core->prio_sched);

	return ret;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_enable);
// PRQA S ALL --

static int32_t bpu_core_plug_out(struct bpu_core *core)
{
	int32_t tmp_work_state = 1;
	int32_t ret = 0;

	if (core == NULL) {
		pr_err("Disable invalid core!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	if (core->hotplug > 0u) {
		/*
		 * if need support hotplug on/off, before real
		 * disable, need wait the prio task fifos empty
		 */
		do {
			ret = bpu_prio_wait_empty(core->prio_sched, HZ);
			if (core->hw_ops->status != NULL) {
				tmp_work_state = core->hw_ops->status(core, WORK_STATE);
			}
		/* if core not work, can exit waiting */
		} while ((ret != 0) && (tmp_work_state > 0));
	}

	return ret;
}

int32_t bpu_core_disable(struct bpu_core *core)
{
	int32_t err, ret;

	if (core == NULL) {
		pr_err("Disable invalid core!\n");/*PRQA S ALL*/
		return -EINVAL;
	}
	if (core->hw_enabled == 0u) {
		return 0;
	}

	bpu_prio_plug_out(core->prio_sched);
	ret = bpu_core_plug_out(core);
	if (ret != 0) {
		bpu_prio_plug_in(core->prio_sched);
		dev_err(core->dev, "Try to pre Plugout core failed!\n");
		return ret;
	}

	core->hw_enabled = 0;
	/*
	 * need wait running fc task done to prevent
	 * BPU or system problem
	 */
	ret = bpu_core_pend_to_leisure(core, HZ);
	if (ret != 0) {
		dev_err(core->dev, "Pend for Disable core failed!\n");
		return ret;
	}
	disable_irq(core->irq);

	if (core->hw_ops->disable != NULL) {
		ret = core->hw_ops->disable(core);
	} else {
		ret = bpu_core_clk_off(core);
		ret += bpu_core_power_off(core);
	}

	err = bpu_core_pend_off(core);
	if (err != 0) {
		dev_err(core->dev, "Pend off for Disable core failed!\n");
		return err;
	}
#ifdef CONFIG_X3_BPU
	pm_qos_remove_request(&core->pm_qos_req);
#endif

	return ret;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_disable);
// PRQA S ALL --

int32_t bpu_core_reset(struct bpu_core *core)
{
	struct bpu_fc tmp_bpu_fc;
	int32_t ret;
	uint32_t i;

	if (core == NULL) {
		pr_err("Reset invalid core!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	for (i = 0; i < BPU_PRIO_NUM; i++) {
		while (!kfifo_is_empty(&core->run_fc_fifo[i])) {
			ret = kfifo_get(&core->run_fc_fifo[i], &tmp_bpu_fc);/*PRQA S ALL*/
			if (ret < 1) {
				continue;
			}
			bpu_fc_clear(&tmp_bpu_fc);
		}
		kfifo_reset(&core->run_fc_fifo[i]);/*PRQA S ALL*/
	}
	kfifo_reset(&core->done_fc_fifo);/*PRQA S ALL*/

	if (core->hw_ops->reset != NULL) {
		return core->hw_ops->reset(core);
	}

	ret = bpu_core_disable(core);
	if (ret != 0) {
		dev_err(core->dev, "BPU core %d disable failed\n", core->index);
	}
	ret = bpu_core_enable(core);
	if (ret != 0) {
		dev_err(core->dev, "BPU core %d enable failed\n", core->index);/*PRQA S ALL*/
	}

	if (core->rst != NULL) {
		ret = reset_control_assert(core->rst);
		if (ret < 0) {
			dev_err(core->dev, "bpu core reset assert failed\n");
			return ret;
		}
		udelay(1);/*PRQA S ALL*/
		ret = reset_control_deassert(core->rst);
		if (ret < 0) {
			dev_err(core->dev, "bpu core reset deassert failed\n");
			return ret;
		}
	}

	return 0;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_reset);
// PRQA S ALL --

int32_t bpu_core_process_recover(struct bpu_core *core)
{
	struct bpu_fc tmp_bpu_fc;
	DECLARE_KFIFO_PTR(recovery_kfifo[BPU_PRIO_NUM], struct bpu_fc);
	unsigned long flags;/*PRQA S ALL*/
	int32_t ret;
	int32_t i;

	if (core == NULL) {
		pr_err("TO recovery no bpu core\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	dev_err(core->dev, "TO recovery bpu core%d\n", core->index);
	/* copy run_fc_fifo for recover */
	for (i = (int32_t)BPU_PRIO_NUM - 1; i >= 0; i--) {
		(void)memcpy(&recovery_kfifo[i], &core->run_fc_fifo[i],/*PRQA S ALL*/
				sizeof(recovery_kfifo[i]));
		while (kfifo_len(&recovery_kfifo[i]) > 0) {/*PRQA S ALL*/
			ret = kfifo_get(&recovery_kfifo[i], &tmp_bpu_fc);/*PRQA S ALL*/
			if (ret < 1) {
				dev_err(core->dev,
						"Get recovery bpu fc failed in BPU Core%d\n",
						core->index);
				return -EINVAL;
			}

			spin_lock_irqsave(&core->spin_lock, flags);/*PRQA S ALL*/
			if ((core->hw_ops->write_fc != NULL) && (tmp_bpu_fc.fc_data != NULL)) {
				ret = core->hw_ops->write_fc(core, &tmp_bpu_fc, 0);
				if (ret < 0) {
					spin_unlock_irqrestore(&core->spin_lock, flags);
					dev_err(core->dev, "TO recovery bpu core%d\n",
							core->index);
					return ret;
				}
			}
			spin_unlock_irqrestore(&core->spin_lock, flags);
		}
	}

	return 0;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_process_recover);
// PRQA S ALL --

static int32_t bpu_core_raw_set_volt(const struct bpu_core *core, int32_t volt)
{
	int32_t ret = 0;

	if (core == NULL) {
		pr_err("Set invalid bpu core voltage!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (volt <= 0) {
		pr_err("Set invalid value bpu core voltage!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	if (core->regulator != NULL) {
		ret = regulator_set_voltage(core->regulator,
					volt, INT_MAX);
		if (ret != 0) {
			dev_err(core->dev, "Cannot set voltage %u uV\n", volt);
			return ret;
		}
	}

	return ret;
}

static int32_t bpu_core_set_volt(struct bpu_core *core, int32_t volt)
{
	int32_t err, ret = 0;

	if (core == NULL) {
		pr_err("Invalid core set volt!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	if (volt == 0) {
		ret = bpu_core_disable(core);
		if (ret != 0) {
			dev_err(core->dev, "Diable bpu core%d set volt!\n", core->index);
			return ret;
		}
	}

	ret = bpu_core_pend_to_leisure(core, HZ);
	if (ret != 0) {
		dev_err(core->dev, "Pend for core volt change failed!\n");
		return ret;
	}

	if (core->hw_ops->set_volt != NULL) {
		ret = core->hw_ops->set_volt(core, volt);
	} else {
		ret = bpu_core_raw_set_volt(core, volt);
	}
	if (ret != 0) {
		dev_err(core->dev, "BPU Core set volt %d failed!\n", volt);
	}

	err = bpu_core_pend_off(core);
	if (err != 0) {
		dev_err(core->dev, "Pend off from core volt change failed!\n");
		return err;
	}

	return ret;
}

static int32_t bpu_core_set_clk(struct bpu_core *core, uint64_t rate)
{
	int32_t err, ret = 0;

	if (core == NULL) {
		pr_err("Invalid core set volt!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	ret = bpu_core_pend_to_leisure(core, HZ);
	if (ret != 0) {
		dev_err(core->dev, "Pend for core clk change failed!\n");
		return ret;
	}

	/*
	 * set clock must use special timing and step
	 * or else some unknown exceptions will appear
	 */
	if (core->hw_ops->set_clk != NULL) {
		ret = core->hw_ops->set_clk(core, rate);
	} else {
		ret = -ENODEV;
	}
	if (ret != 0) {
		dev_err(core->dev, "BPU Core set clk to %lld failed!\n", rate);
	}

	err = bpu_core_pend_off(core);
	if (err != 0) {
		dev_err(core->dev, "Pend off from core clk change failed!\n");
		return err;
	}

	return ret;
}

static uint64_t bpu_core_get_clk(struct bpu_core *core)
{
	uint64_t rate;

	if (core == NULL) {
		pr_err("Invalid core get clk!\n");/*PRQA S ALL*/
		return 0;
	}

	rate = clk_get_rate(core->mclk);

	return rate;
}

int32_t bpu_core_ext_ctrl(struct bpu_core *core, ctrl_cmd_t cmd, uint64_t *data)
{
	if ((core == NULL) || (data == NULL)) {
		pr_err("Invalid core/data!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	if (cmd == CORE_TYPE) {
		*data = bpu_core_type(core);
		return 0;
	}

	if (cmd == SET_CLK) {
		return bpu_core_set_clk(core, *data);
	}

	if (cmd == GET_CLK) {
		*data = bpu_core_get_clk(core);
		return 0;
	}

	pr_err("Invalid core cmd!\n");/*PRQA S ALL*/

	return -EINVAL;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_ext_ctrl);
// PRQA S ALL --

#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
static int bpu_core_set_freq(struct device *dev, unsigned long *freq, u32 flags)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	struct dev_pm_opp *opp;
	uint64_t rate, target_rate;
	uint64_t target_volt;
	int32_t err = 0;
	int32_t ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp)) {/*PRQA S ALL*/
		err = PTR_ERR(opp);/*PRQA S ALL*/
		return err;
	}
	rate = dev_pm_opp_get_freq(opp);
	target_volt = dev_pm_opp_get_voltage(opp);

	target_rate = (uint64_t)clk_round_rate(core->mclk, rate);
	if (target_rate <= 0u) {
		target_rate = rate;
	}

	if (core->dvfs->rate == target_rate) {
		if (core->dvfs->volt != target_volt) {
			err = bpu_core_set_volt(core, (int32_t)target_volt);
			if (err != 0) {
				dev_err(dev, "Cannot set voltage %llu uV\n",
						target_volt);
				dev_pm_opp_put(opp);
				return err;
			}
			core->dvfs->volt = target_volt;
			dev_pm_opp_put(opp);
			return err;
		}
	} else {
		/*
		 * To higher rate: need set volt first
		 * To lower rate: need set rate first
		 */
		if (core->dvfs->rate < target_rate) {
			err = bpu_core_set_volt(core, (int32_t)target_volt);
			if (err != 0) {
				dev_err(dev, "Cannot set voltage %llu uV\n",
						target_volt);
				dev_pm_opp_put(opp);
				return err;
			}
		}

		err = bpu_core_set_clk(core, target_rate);
		if (err != 0) {
			dev_err(dev, "Cannot set frequency %llu (%d)\n",
					target_rate, err);
			ret = bpu_core_set_volt(core, (int32_t)core->dvfs->rate);
			if (ret != 0) {
				dev_err(dev, "Recovery to old volt failed\n");
			}
			dev_pm_opp_put(opp);
			return err;
		}

		if (core->dvfs->rate > target_rate) {
			err = bpu_core_set_volt(core, (int32_t)target_volt);
			if (err != 0) {
				dev_err(dev, "Cannot set vol %llu uV\n", target_volt);
				dev_pm_opp_put(opp);
				return err;
			}
		}

		core->dvfs->rate = target_rate;
		core->dvfs->volt = target_volt;
	}

	return err;
}

static int bpu_core_get_freq(struct device *dev, unsigned long *freq)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/

	if (core != NULL) {
		if (core->dvfs != NULL) {
			core->dvfs->rate = bpu_core_get_clk(core);
			*freq = core->dvfs->rate;
		}
	}

	return 0;
}

int32_t bpu_core_dvfs_register(struct bpu_core *core, const char *name)
{
	struct dev_pm_opp *opp;
	const char *gov_name;
	uint32_t tmp_state;
	int32_t ret;

	if (core == NULL) {
		pr_err("NO BPU Core!!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	core->dvfs = (struct bpu_core_dvfs *)devm_kzalloc(core->dev,/*PRQA S ALL*/
			sizeof(struct bpu_core_dvfs), GFP_KERNEL);
	if (core->dvfs == NULL) {
		dev_err(core->dev, "Can't alloc BPU dvfs.\n");
		return -ENOMEM;
	}

	if (dev_pm_opp_of_add_table(core->dev) != 0) {
		devm_kfree(core->dev, (void *)core->dvfs);/*PRQA S ALL*/
		core->dvfs = NULL;
		dev_err(core->dev, "Invalid operating-points in devicetree.\n");
		return -EINVAL;
	}

	core->dvfs->profile.polling_ms	= 0;
	core->dvfs->profile.target = bpu_core_set_freq;
	core->dvfs->profile.get_cur_freq = bpu_core_get_freq;
	if (core->mclk != NULL) {
		core->dvfs->profile.initial_freq = clk_get_rate(core->mclk);
	} else {
		devm_kfree(core->dev, (void *)core->dvfs);/*PRQA S ALL*/
		core->dvfs = NULL;
		dev_err(core->dev, "No adjustable clock for dvfs.\n");
		return -EINVAL;
	}

	core->dvfs->rate = core->dvfs->profile.initial_freq;
	if (core->regulator != NULL) {
		core->dvfs->volt = (uint32_t)regulator_get_voltage(core->regulator);
	} else {
		dev_err(core->dev, "No adjustable regulator for dvfs, use fake value.\n");
		opp = devfreq_recommended_opp(core->dev, &core->dvfs->profile.initial_freq,
				DEVFREQ_FLAG_LEAST_UPPER_BOUND);
		if (IS_ERR(opp)) {/*PRQA S ALL*/
			devm_kfree(core->dev, (void *)core->dvfs);/*PRQA S ALL*/
			core->dvfs = NULL;
			dev_err(core->dev, "Can't find opp\n");
			return PTR_ERR(opp);/*PRQA S ALL*/
		}
		core->dvfs->volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);
	}

	if (of_property_read_string(core->dev->of_node, "governor", &gov_name) != 0) {
		if (name != NULL) {
			gov_name = name;
		} else {
			gov_name = "performance";
		}
	}

	core->dvfs->devfreq = devm_devfreq_add_device(core->dev,
			&core->dvfs->profile, gov_name, NULL);
	if (IS_ERR(core->dvfs->devfreq)) {/*PRQA S ALL*/
		core->dvfs->devfreq = NULL;
		dev_err(core->dev, "Can't add dvfs to BPU core.\n");
		return PTR_ERR(core->dvfs->devfreq);/*PRQA S ALL*/
	}

	if (core->dvfs->profile.max_state > 0u) {
		tmp_state = core->dvfs->profile.max_state - 1u;
	} else {
		tmp_state = 0;
	}
	core->dvfs->devfreq->min_freq = core->dvfs->profile.freq_table[0];
	core->dvfs->devfreq->max_freq =
		core->dvfs->profile.freq_table[tmp_state];
	core->dvfs->level_num = core->dvfs->profile.max_state;

	ret = devm_devfreq_register_opp_notifier(core->dev, core->dvfs->devfreq);

	core->dvfs->cooling = of_devfreq_cooling_register(core->dev->of_node,
			core->dvfs->devfreq);

	core->power_level = 1;

	return ret;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_dvfs_register);
// PRQA S ALL --

void bpu_core_dvfs_unregister(struct bpu_core *core)
{
	if (core == NULL) {
		return;
	}

	if (core->dvfs == NULL) {
		return;
	}

	if ((core->dvfs->cooling != NULL) && (!IS_ERR(core->dvfs->cooling))) {
		devfreq_cooling_unregister(core->dvfs->cooling);
	}

	if ((core->dvfs->devfreq != NULL) && (!IS_ERR(core->dvfs->devfreq))) {
		devm_devfreq_unregister_opp_notifier(core->dev, core->dvfs->devfreq);
		devm_devfreq_remove_device(core->dev, core->dvfs->devfreq);
	}
	dev_pm_opp_of_remove_table(core->dev);

	devm_kfree(core->dev, (void *)core->dvfs);/*PRQA S ALL*/
	core->dvfs = NULL;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_dvfs_unregister);
// PRQA S ALL --
int32_t bpu_core_set_freq_level(struct bpu_core *core, int32_t level)
{
	uint64_t wanted;
	int32_t tmp_state, tmp_level;
	int32_t ret = 0;
	uint32_t i;

	if (core == NULL) {
		pr_err("Invalid core set freq level!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	if (core->dvfs == NULL) {
		pr_err("No Adjustable clock to support set freq level!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	tmp_level = level;
	if (level > 0) {
		/* if level > 0, make the governor to default gover*/
		if (core->dvfs->cooling == NULL) {
			for (i = 0; i < core->dvfs->profile.max_state; i++) {
				ret = dev_pm_opp_enable(core->dev,
						core->dvfs->profile.freq_table[i]);
			}
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

		if ((level <= (-1 * (int32_t)core->dvfs->level_num)) && (level != 0)) {
			dev_err(core->dev,
					"Set BPU core%d freq level(%d) lower then lowest(%d)\n",
					core->index, level, (-1 * (int32_t)core->dvfs->level_num) + 1);
			tmp_level = (-1 * (int32_t)core->dvfs->level_num) + 1;
		}

		for (i = 0; i < core->dvfs->profile.max_state; i++) {
			ret = dev_pm_opp_disable(core->dev,
					core->dvfs->profile.freq_table[i]);
			if (ret != 0) {
				dev_err(core->dev, "Set BPU core pm opp disable failed\n");
			}
		}

		if (((int32_t)core->dvfs->profile.max_state + tmp_level) > 0) {
			tmp_state = ((int32_t)core->dvfs->profile.max_state + tmp_level) - 1;
		} else {
			tmp_state = 0;
		}
		wanted = core->dvfs->profile.freq_table[tmp_state];

		ret = dev_pm_opp_enable(core->dev, wanted);
		if (ret != 0) {
			dev_err(core->dev, "Set BPU core pm opp enable failed\n");
		}

		mutex_lock(&core->dvfs->devfreq->lock);
		if (wanted < core->dvfs->rate) {
			ret = bpu_core_set_freq(core->dev, (unsigned long *)&wanted, 0);/*PRQA S ALL*/
		} else {
			ret = bpu_core_set_freq(core->dev,
					(unsigned long *)&wanted,/*PRQA S ALL*/
					DEVFREQ_FLAG_LEAST_UPPER_BOUND);
		}
		if (ret != 0) {
			mutex_unlock(&core->dvfs->devfreq->lock);
			dev_err(core->dev,
					"Set BPU core%d set freq level(%d) failed\n",
					core->index, tmp_level);
			return ret;
		}
		mutex_unlock(&core->dvfs->devfreq->lock);
		core->power_level = tmp_level;
	}

	return ret;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_set_freq_level);
// PRQA S ALL --
#endif

int32_t bpu_core_set_limit(struct bpu_core *core, int32_t limit)
{
	int32_t tmp_limit;

	if (core == NULL) {
		pr_err("Invalid core set freq level!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	if ((limit < 0) || (limit > (int32_t)FC_MAX_DEPTH)) {
		tmp_limit = 0;
	} else {
		tmp_limit = limit;
	}

	core->fc_buf_limit = tmp_limit;

	return 0;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_set_limit);
// PRQA S ALL --

uint64_t bpu_clk_get_rate(struct clk *clk)
{
	return clk_get_rate(clk);
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_clk_get_rate);
// PRQA S ALL --

int32_t bpu_clk_set_rate(struct clk *clk, uint64_t rate)
{
	return clk_set_rate(clk, rate);
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_clk_set_rate);
// PRQA S ALL --
int32_t bpu_reset_ctrl(struct reset_control *rstc, uint16_t val)
{
	if (val > 0u) {
		return reset_control_assert(rstc);
	}

	return reset_control_deassert(rstc);
}
EXPORT_SYMBOL(bpu_reset_ctrl);
