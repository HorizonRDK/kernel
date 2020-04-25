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

#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>
#include <asm-generic/delay.h>

#include "common.h"

#define to_cnnclk(_hw) container_of(_hw, struct cnnclk, hw)

struct cnnclk {
	struct clk_hw hw;
	struct clk *cnnpll;
	struct clk *syspllc;
	struct clk *cnn_src;
	struct clk *cnnpll_mux;
	struct clk *cnn_osc;
	unsigned int flags;
	void __iomem *clk_base;
	spinlock_t lock;
};

static unsigned long cnnclk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct cnnclk *cnn_clk;
	unsigned long rate;

	cnn_clk = to_cnnclk(hw);
	rate = clk_get_rate(cnn_clk->cnnpll);

	return rate;
}

static long cnnclk_round_rate(struct clk_hw *hw,
				unsigned long rate, unsigned long *prate)
{
	/* return directly, use exact rate for cpu/cnn freq */;
	return rate;
}

struct cnn_pll_table {
	unsigned long cnn_freq;
	unsigned long pll_freq;
};

#define CNN_FREQ_NUM 4
struct cnn_pll_table cpll_table[CNN_FREQ_NUM] = {
	{1000000000, 1000000000},
	{1200000000, 1200000000},
	{800000000,  800000000},
	{400000000,  400000000},
};

static int __set_cnnpll_clk(struct clk_hw *hw, unsigned long cnn_freq)
{
	struct cnnclk *cnnclk;
	unsigned long pll_rate, old_pll_rate;
	int found = 0;
	int i, ret;

	cnnclk = to_cnnclk(hw);

	for (i = 0; i < CNN_FREQ_NUM; i++) {
		if (cnn_freq == cpll_table[i].cnn_freq) {
			pll_rate = cpll_table[i].pll_freq;
			found = 1;
			break;
		}
	}

	if (!found) {
		pr_err("Not found rate:%lu in table\n", cnn_freq);
		return -1;
	}

	old_pll_rate = clk_get_rate(cnnclk->cnnpll);
	if (pll_rate != old_pll_rate) {
		/* enable and switch to syspllc */
		clk_prepare(cnnclk->syspllc);
		clk_enable(cnnclk->syspllc);

		/* switch to syspllc */
		ret = clk_set_parent(cnnclk->cnn_src, cnnclk->syspllc);
		if (ret)
			return ret;

		/* change PLL */
		clk_prepare(cnnclk->cnnpll);
		clk_enable(cnnclk->cnnpll);
		clk_disable(cnnclk->cnnpll);

		ret = clk_set_rate(cnnclk->cnnpll, pll_rate);
		if (ret) {
			clk_set_parent(cnnclk->cnnpll_mux, cnnclk->cnn_src);
			return ret;
		}
		clk_prepare(cnnclk->cnnpll);
		clk_enable(cnnclk->cnnpll);

		/* switch back to cnnpll */
		ret = clk_set_parent(cnnclk->cnn_src, cnnclk->cnnpll);
		if (ret)
			return ret;

		clk_disable(cnnclk->syspllc);
	}

	return 0;
}

static int cnnclk_set_rate(struct clk_hw *hw,
		unsigned long rate, unsigned long parent_rate)
{
	struct cnnclk *cnn_clk;
	int ret;
	static int recursive_flag;

	/*
	 * clk_set_rate may re-enter this function since cnnclk is children of
	 * cnnpll_mux return if recursive call is detected to prevent deadloop
	 */
	if (recursive_flag == 1) {
		recursive_flag = 0;
		return 0;
	}

	cnn_clk = to_cnnclk(hw);
	recursive_flag = 1;
	ret = __set_cnnpll_clk(hw, rate);
	recursive_flag = 0;

	return ret;
}

const struct clk_ops cnnclk_ops = {
	.recalc_rate = cnnclk_recalc_rate,
	.round_rate = cnnclk_round_rate,
	.set_rate = cnnclk_set_rate,
};

static struct clk *cnnclk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct cnnclk *cnn_clk;
	struct clk *clk;
	unsigned int ret;

	cnn_clk = kzalloc(sizeof(*cnn_clk), GFP_KERNEL);
	if (!cnn_clk)
		return NULL;

	init.name = name;
	init.ops = ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	cnn_clk->hw.init = &init;

	cnn_clk->cnnpll = clk_get(NULL, "cnnpll_clk");
	cnn_clk->syspllc = clk_get(NULL, "armpll2_clk");
	cnn_clk->cnn_src = clk_get(NULL, "cnn_src_clk");
	cnn_clk->cnnpll_mux = clk_get(NULL, "cnn_pllmux_clk");
	cnn_clk->cnn_osc = clk_get(NULL, "osc_clk");

	spin_lock_init(&cnn_clk->lock);
	clk = clk_register(dev, &cnn_clk->hw);
	if (IS_ERR(clk)) {
		pr_err("Failed to register clock for %s!\n", name);
		kfree(cnn_clk);
		return NULL;
	}

	ret = clk_hw_register_clkdev(&cnn_clk->hw, name, NULL);
	if (ret) {
		pr_err("Failed to register lookup %s!\n", name);
		clk_unregister(clk);
		kfree(cnn_clk);
		return NULL;
	}

	return clk;
}

static void __init _of_hobot_cnnclk_setup(struct device_node *node,
				const struct clk_ops *ops)
{
	struct clk *clk;
	const char *parent_name;
	unsigned int flags = 0;

	if (of_clk_get_parent_count(node) != 1) {
		pr_err("%s must have 1 parent\n", node->name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	clk = cnnclk_register(NULL, node->name, parent_name, flags, ops);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

static void __init of_hobot_cnnclk_setup(struct device_node *node)
{
	_of_hobot_cnnclk_setup(node, &cnnclk_ops);
}
CLK_OF_DECLARE(hobot_x3_cnnclk, "hobot,cnn-clk-x3", of_hobot_cnnclk_setup);
