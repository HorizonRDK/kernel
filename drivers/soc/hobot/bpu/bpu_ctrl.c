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
#include "bpu.h"
#include "bpu_core.h"

int bpu_core_enable(struct bpu_core *core)
{
	int ret;

	if (!core) {
		pr_err("Enable invalid core!\n");
		return -EINVAL;
	}

	if (core->hw_ops->enable)
		return core->hw_ops->enable(core);

	ret = regulator_enable(core->regulator);
	if (ret != 0) {
		dev_err(core->dev, "bpu core[%d] enable error\n", core->index);
		return ret;
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

	return ret;
}
EXPORT_SYMBOL(bpu_core_enable);

int bpu_core_disable(struct bpu_core *core)
{
	int ret;

	if (!core) {
		pr_err("Disable invalid core!\n");
		return -EINVAL;
	}

	if (core->hw_ops->disable)
		return core->hw_ops->disable(core);

	if (core->aclk) {
		if (__clk_is_enabled(core->aclk))
			clk_disable_unprepare(core->aclk);
	}

	if (core->mclk) {
		if (!__clk_is_enabled(core->mclk))
			clk_disable_unprepare(core->mclk);
	}

	ret = regulator_disable(core->regulator);
	if (ret != 0)
		dev_err(core->dev, "bpu core[%d] disable error\n", core->index);

	return ret;
}
EXPORT_SYMBOL(bpu_core_disable);

int bpu_core_reset(struct bpu_core *core)
{
	int ret;

	if (!core) {
		pr_err("Reset invalid core!\n");
		return -EINVAL;
	}

	kfifo_reset(&core->run_fc_fifo);
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

int bpu_core_set_volt(struct bpu_core *core, int clk)
{
	if (!core) {
		pr_err("Invalid core set volt!\n");
		return -EINVAL;
	}

	if (core->hw_ops->set_volt)
		return core->hw_ops->set_volt(core, clk);

	return 0;
}
EXPORT_SYMBOL(bpu_core_set_volt);
