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
#include <linux/delay.h>

#include "common.h"

#define PLL_CTRL_FBDIV_BIT 0
#define PLL_CTRL_REFDIV_BIT 12
#define PLL_CTRL_POSTDIV1_BIT 20
#define PLL_CTRL_POSTDIV2_BIT 24

#define PLL_CTRL_FBDIV_FIELD 0xFFF
#define PLL_CTRL_REFDIV_FIELD 0x3F
#define PLL_CTRL_POSTDIV1_FIELD 0x7
#define PLL_CTRL_POSTDIV2_FIELD 0x7

#define PLL_PD_CTRL_PD_BIT 0
#define PLL_PD_CTRL_DSMPD_BIT 4
#define PLL_PD_CTRL_FOUTPOSTDIVPD_BIT 8
#define PLL_PD_CTRL_FOUTVCOPD_BIT 12
#define PLL_PD_CTRL_BYPASS_BIT 16

#define PLL_STATUS_LOCK_BIT 0

#define PLL_BYPASS_MODE 0x1
#define PLL_LAINT_MODE 0x0

#define PLL_LAINT_REFDIV_MIN 1
#define PLL_LAINT_REFDIV_MAX 63
#define PLL_LAINT_FBDIV_MIN 16
#define PLL_LAINT_FBDIV_MAX 640
#define PLL_LAINT_POSTDIV1_MIN 1
#define PLL_LAINT_POSTDIV1_MAX 7
#define PLL_LAINT_POSTDIV2_MIN 1
#define PLL_LAINT_POSTDIV2_MAX 7

#define PLL_LAINT_MIN_FREQ 800000000
#define PLL_LAINT_MAX_FREQ 3200000000

struct clk_laintpll_reg {
	void __iomem *freq_reg;
	void __iomem *pd_reg;
	void __iomem *status;
};

struct laintpll_bestdiv {
	unsigned long freq;
	unsigned int refdiv;
	unsigned int fbdiv;
	unsigned int postdiv1;
	unsigned int postdiv2;
};

/*
 * Only support 1.6GHz, 2GHz and 2.4G for ARMPLL,
 * add more if there are new pll freq use case for other plls
 */
static struct laintpll_bestdiv pll_bestdiv_table[] = {
	{240000000,  1, 10, 1, 1},
	{400000000,  1, 100, 6, 1},
	{800000000,  1, 100, 3, 1},
	{950000000,  6, 475, 2, 1},
	{1000000000, 1, 125, 3, 1},
	{1200000000, 1, 100, 2, 1},
	{1500000000, 1, 125, 2, 1},
	{1600000000, 1, 200, 3, 1},
#if 0
	{2000000000, 1, 250, 3, 1},
	{2400000000, 1, 100, 1, 1},
#endif
};

#define to_clk_laintpll(_hw) container_of(_hw, struct clk_laintpll, hw)

struct clk_laintpll {
	struct clk_hw hw;
	struct clk_laintpll_reg reg;
	unsigned int flags;
	unsigned int mode;
	unsigned int refdiv;
	unsigned int fbdiv;
	unsigned int postdiv1;
	unsigned int postdiv2;
	unsigned int round_rate;
	spinlock_t lock;
};

static int clk_lainpll_wait_lock(struct clk_laintpll *pll)
{
	int count = 500;
	u32 val = readl_relaxed(pll->reg.pd_reg) & BIT(PLL_PD_CTRL_PD_BIT);

	/* No need to wait for lock when pll is not powered up */
	if (val)
		return 0;

	/* Wait for PLL to lock */
	do {
		if (readl_relaxed(pll->reg.status) & BIT(PLL_STATUS_LOCK_BIT))
			break;
		udelay(10);
	} while (--count > 0);

	val = readl_relaxed(pll->reg.status) & BIT(PLL_STATUS_LOCK_BIT);

	return  val ? 0 : -ETIMEDOUT;
}


static unsigned long laintpll_recalc_rate(struct clk_hw *hw,
			unsigned long parent_rate)
{
	struct clk_laintpll *clk;
	unsigned int fbdiv, refdiv, postdiv1, postdiv2;
	unsigned int bypass;
	unsigned int val;
	unsigned long rate;

	clk = to_clk_laintpll(hw);

	val = readl(clk->reg.pd_reg);
	bypass = (val & (1 << PLL_PD_CTRL_BYPASS_BIT)) >> PLL_PD_CTRL_BYPASS_BIT;
	if(bypass){
		pr_info("%s: laintpll bypass\n", __func__);
		return parent_rate;
	}

	val = readl(clk->reg.freq_reg);
	fbdiv = (val & (PLL_CTRL_FBDIV_FIELD << PLL_CTRL_FBDIV_BIT))
			>> PLL_CTRL_FBDIV_BIT;
	refdiv = (val & (PLL_CTRL_REFDIV_FIELD << PLL_CTRL_REFDIV_BIT))
			>> PLL_CTRL_REFDIV_BIT;
	postdiv1 = (val & (PLL_CTRL_POSTDIV1_FIELD << PLL_CTRL_POSTDIV1_BIT))
			>> PLL_CTRL_POSTDIV1_BIT;
	postdiv2 = (val & (PLL_CTRL_POSTDIV2_FIELD << PLL_CTRL_POSTDIV2_BIT))
			>> PLL_CTRL_POSTDIV2_BIT;

	rate = (parent_rate / refdiv) * fbdiv / postdiv1 /postdiv2;
	return rate;
}

/*
 * lookup table for the preset pll frequency division
 * return 0 if found, return -1 if not found
 */
static int laintpll_lookup_table(struct clk_laintpll *clk,
			unsigned long rate)
{
	struct laintpll_bestdiv *bestdiv;
	int i;

	for (i = 0; i < ARRAY_SIZE(pll_bestdiv_table); i++) {
		if (rate == pll_bestdiv_table[i].freq) {
			bestdiv = &pll_bestdiv_table[i];
			clk->refdiv = bestdiv->refdiv;
			clk->fbdiv  = bestdiv->fbdiv;
			clk->postdiv1 = bestdiv->postdiv1;
			clk->postdiv2 = bestdiv->postdiv2;

			return 0;
		}
	}

	return -EINVAL;
}

static long laintpll_round_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long *parent_rate)
{
	struct clk_laintpll *clk;
	unsigned int refdiv, fbdiv, postdiv1, postdiv2;
	unsigned int refdiv_pre = 0, fbdiv_pre = 0, postdiv1_pre = 0, postdiv2_pre = 0;
	unsigned int refdiv_post, fbdiv_post, postdiv1_post, postdiv2_post;
	unsigned int found = 0;
	unsigned long prate;
	unsigned long rate_pre, rate_post, rate_cur;

	clk = to_clk_laintpll(hw);

	prate = *parent_rate;
	if((rate < prate) | (rate > PLL_LAINT_MAX_FREQ)){
		pr_warn("%s: request frequency %ldHz not supported\n", __func__, rate);
		return -EINVAL;
	}

	if(rate == prate){
		pr_info("%s: request parent rate\n", __func__);
		clk->mode = PLL_BYPASS_MODE;
		return prate;
	}

	if (!laintpll_lookup_table(clk, rate)) {
		pr_debug("%s: find rate:%lu in quick table!\n", __func__, rate);
		return rate;
	}
	pr_info("new rate:%lu, add it into pll freq table!\n", rate);

	clk->mode = PLL_LAINT_MODE;
	/*
	 * loop here for general rounding the freq combination value here,
	 * for more efficient and power saving, lookup table can be used with
	 * limited combination, but usecase need to be confirmed.
	 */
	rate_post = PLL_LAINT_MAX_FREQ;
	rate_pre = prate;
	for (refdiv = PLL_LAINT_REFDIV_MIN; refdiv <= PLL_LAINT_REFDIV_MAX;
			refdiv++){
		for (fbdiv = PLL_LAINT_FBDIV_MAX; (fbdiv >= PLL_LAINT_FBDIV_MIN)
			&& (prate / refdiv * fbdiv <= rate_post)
			&& (prate / refdiv * fbdiv
			>= PLL_LAINT_MIN_FREQ); fbdiv--){
			for (postdiv1 = PLL_LAINT_POSTDIV1_MAX;
				postdiv1 >= PLL_LAINT_POSTDIV1_MIN;
							postdiv1--) {
				for (postdiv2 = PLL_LAINT_POSTDIV2_MIN;
					(postdiv2 <= postdiv1); postdiv2++) {
					rate_cur = (prate / refdiv) * fbdiv
							/ postdiv1 / postdiv2;
					if ((rate_cur > rate)
						&& (rate_cur < rate_post)) {
						rate_post = rate_cur;
						refdiv_post = refdiv;
						fbdiv_post = fbdiv;
						postdiv2_post = postdiv2;
						postdiv1_post = postdiv1;
						continue;
					}
					if (rate_cur == rate) {
						found = 1;
						break;
					} else if ((rate_cur < rate)
						&& (rate_cur > rate_pre)) {
						rate_pre = rate_cur;
						refdiv_pre = refdiv;
						fbdiv_pre = fbdiv;
						postdiv2_pre = postdiv2;
						postdiv1_pre = postdiv1;
						break;
					}
				}
				if (found == 1)
					break;
			}
			if (found == 1)
				break;
		}
		if (found == 1)
			break;
	}

	if (found) {
		clk->refdiv = refdiv;
		clk->fbdiv = fbdiv;
		clk->postdiv1 = postdiv1;
		clk->postdiv2 = postdiv2;
		pr_debug("%s: found exact match %lu\n", __func__, rate);
		return rate;
	} else {
		clk->refdiv = refdiv_pre;
		clk->fbdiv = fbdiv_pre;
		clk->postdiv1 = postdiv1_pre;
		clk->postdiv2 = postdiv2_pre;
		pr_debug("%s: found closest %lu\n", __func__, rate_pre);
		return rate_pre;
	}
}

static int laintpll_set_rate(struct clk_hw *hw,
		unsigned long rate, unsigned long parent_rate)
{
	struct clk_laintpll *clk;
	unsigned long new_rate;
	unsigned int val;
	int ret;

	clk = to_clk_laintpll(hw);

	switch (clk->mode){
		case PLL_BYPASS_MODE:
			new_rate = parent_rate;
			if(new_rate != rate){
				pr_warn("%s: failed with invalid round rate %lu, expecting %lu\n",
						__func__, rate, new_rate);
				return -EINVAL;
			}
			val = readl(clk->reg.pd_reg);
			val |= (1 << PLL_PD_CTRL_BYPASS_BIT);
			writel(val, clk->reg.pd_reg);
			break;
		case PLL_LAINT_MODE:
			new_rate = (parent_rate / clk->refdiv) * clk->fbdiv
						/ clk->postdiv1 / clk->postdiv2;
			if(new_rate != rate){
				pr_warn("%s: failed with invalid round rate %lu, expecting %lu\n",
						__func__, rate, new_rate);
				return -EINVAL;
			}
			val = readl(clk->reg.pd_reg);
			val &= ~(1 << PLL_PD_CTRL_BYPASS_BIT);
			writel(val, clk->reg.pd_reg);

			val = (clk->refdiv & PLL_CTRL_REFDIV_FIELD)
					<< PLL_CTRL_REFDIV_BIT;
			val |= ((clk->fbdiv & PLL_CTRL_FBDIV_FIELD)
					<< PLL_CTRL_FBDIV_BIT);
			val |= ((clk->postdiv1 & PLL_CTRL_POSTDIV1_FIELD)
					<< PLL_CTRL_POSTDIV1_BIT);
			val |= ((clk->postdiv2 & PLL_CTRL_POSTDIV2_FIELD)
					<< PLL_CTRL_POSTDIV2_BIT);
			writel(val, clk->reg.freq_reg);
			break;
		default:
			pr_warn("%s: invalid clk mode\n", __func__);
			return -EINVAL;
	}

	ret = clk_lainpll_wait_lock(clk);

	return ret;
}

int laintpll_clk_enable(struct clk_hw *hw)
{
	struct clk_laintpll *clk;
	unsigned int pd;
	unsigned long flags;
	unsigned int val;

	clk = to_clk_laintpll(hw);

	spin_lock_irqsave(&clk->lock, flags);

	val = readl(clk->reg.pd_reg);
	pd = (val & (1 << PLL_PD_CTRL_PD_BIT)) >> PLL_PD_CTRL_PD_BIT;
	if (pd) {
		val &= (~(1 << PLL_PD_CTRL_PD_BIT)) &
				(~(1 << PLL_PD_CTRL_FOUTPOSTDIVPD_BIT));
		writel(val, clk->reg.pd_reg);
		clk_lainpll_wait_lock(clk);
	}

	spin_unlock_irqrestore(&clk->lock, flags);

	return 0;
}

static void laintpll_clk_disable(struct clk_hw *hw)
{
	struct clk_laintpll *clk;
	unsigned long flags;
	unsigned int pd;
	unsigned int val;

	clk = to_clk_laintpll(hw);

	spin_lock_irqsave(&clk->lock, flags);
	val = readl(clk->reg.pd_reg);
	pd = (val & (1 << PLL_PD_CTRL_PD_BIT)) >> PLL_PD_CTRL_PD_BIT;
	if (!pd) {
		val |= 1 << PLL_PD_CTRL_PD_BIT;
		val |= 1 << PLL_PD_CTRL_FOUTPOSTDIVPD_BIT;
		writel(val, clk->reg.pd_reg);
	}
	spin_unlock_irqrestore(&clk->lock, flags);

	return;
}

const struct clk_ops laintpll_clk_ops = {
	.enable = &laintpll_clk_enable,
	.disable = &laintpll_clk_disable,
	.recalc_rate = &laintpll_recalc_rate,
	.round_rate = &laintpll_round_rate,
	.set_rate = &laintpll_set_rate,
};

static struct clk *laintpll_clk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		struct clk_laintpll_reg *reg, unsigned int clk_pll_flags,
		const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct clk_laintpll *clk_hw;
	struct clk *clk;
	unsigned int ret;

	clk_hw = kzalloc(sizeof(*clk_hw), GFP_KERNEL);
	if(!clk_hw){
		pr_err("%s: failed to malloc pll clk for %s!\n", __func__, name);
		return NULL;
	}

	init.name = name;
	init.ops = ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	clk_hw->hw.init = &init;
	memcpy(&clk_hw->reg, reg, sizeof(*reg));
	clk_hw->flags = clk_pll_flags;

	spin_lock_init(&clk_hw->lock);
	clk = clk_register(dev, &clk_hw->hw);
	if(IS_ERR(clk)){
		pr_err("%s: failed to register clock for %s!\n", __func__, name);
		kfree(clk_hw);
		return NULL;
	}

	ret = clk_hw_register_clkdev(&clk_hw->hw, name, NULL);
	if(ret){
		pr_err("%s: failed to register lookup %s!\n", __func__, name);
		clk_unregister(clk);
		kfree(clk_hw);
		return NULL;
	}

	return clk;
}

static void __init _of_hobot_laintpll_clk_setup(struct device_node *node,
			const struct clk_ops *ops)
{
	struct clk *clk;
	struct clk_laintpll_reg reg;
	const char *parent_name;
	unsigned int flags = 0;
	unsigned int clk_laintpll_flags = 0;
	unsigned int data[3];
	void __iomem *reg_base;
	int ret;

	if(of_clk_get_parent_count(node) != 1){
		pr_err("%s: %s must have 1 parent\n", __func__, node->name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	reg_base = clk_get_register_base(node);
	if(!reg_base){
		pr_err("%s: %s failed to get the reg base!\n", __func__, node->name);
		return;
	}

	ret = of_property_read_u32_array(node, "offset", data, 3);
	if(ret){
		pr_err("%s: %s missing offset property", __func__, node->name);
		return;
	}
	reg.freq_reg = reg_base + data[0];
	reg.pd_reg = reg_base + data[1];
	reg.status = reg_base + data[2];

	clk = laintpll_clk_register(NULL, node->name, parent_name, flags, &reg,
			clk_laintpll_flags, ops);

	if(!IS_ERR(clk)){
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	}

	//pr_info("%s: %s pll clock set up.\n", __func__, node->name);
}

static void __init of_hobot_laintpll_clk_setup(struct device_node *node)
{
	_of_hobot_laintpll_clk_setup(node, &laintpll_clk_ops);
}
CLK_OF_DECLARE(hobot_laintpll_clk, "hobot,laintpll-clk",
		of_hobot_laintpll_clk_setup);
