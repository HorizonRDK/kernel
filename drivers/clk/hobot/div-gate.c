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

#include "div-comm.h"

#define to_clk_div_gate(_hw) container_of(_hw, struct clk_div_gate, hw)

struct clk_div_gate {
	struct clk_hw hw;
	struct clk_div_reg div_reg;
	struct clk_gate_reg gate_reg;
	unsigned int flags;
	raw_spinlock_t *lock;
};

int div_gate_clk_enable(struct clk_hw *hw)
{
	struct clk_div_gate *clk = to_clk_div_gate(hw);
	unsigned int val, status, reg_val;
	unsigned long flags = 0;

	if (clk->lock)
		raw_spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	val = readl(clk->gate_reg.clken_sta_reg);
	status = (val & (1 << clk->gate_reg.clken_sta_bit))
	>> clk->gate_reg.clken_sta_bit;

	if(clk->gate_reg.disable_bit == 32) {
		if (!status) {
		val |= 1 << clk->gate_reg.enable_bit;
		writel(val, clk->gate_reg.enable_reg);
		}
	} else {
		if (!status) {
		reg_val = 1 << clk->gate_reg.enable_bit;
		writel(reg_val, clk->gate_reg.enable_reg);
		}
	}

	if (clk->lock)
		raw_spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return 0;
}

void div_gate_clk_disable(struct clk_hw *hw)
{
	struct clk_div_gate *clk = to_clk_div_gate(hw);
	unsigned int val, status, reg_val;
	unsigned long flags = 0;


	clk = to_clk_div_gate(hw);

	if (clk->lock)
		raw_spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	val = readl(clk->gate_reg.clken_sta_reg);
	status = (val & (1 << clk->gate_reg.clken_sta_bit))
	>> clk->gate_reg.clken_sta_bit;
	/* determining is ips reg? */
	if(clk->gate_reg.disable_bit == 32) {
		if (status) {
			val &= ~(1 << clk->gate_reg.enable_bit);
			writel(val, clk->gate_reg.enable_reg);
		}
	} else {
		if (status) {
			reg_val = 1 << clk->gate_reg.disable_bit;
			writel(reg_val, clk->gate_reg.disable_reg);
		}
	}

	if (clk->lock)
		raw_spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return;
}

int div_gate_clk_is_enabled(struct clk_hw *hw)
{
	struct clk_div_gate *clk;
	unsigned int val, status;

	clk = to_clk_div_gate(hw);

	val = readl(clk->gate_reg.clken_sta_reg);
	status = (val & (1 << clk->gate_reg.clken_sta_bit))
	>> clk->gate_reg.clken_sta_bit;

	return status;
}

unsigned long div_gate_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_div_gate *clk = to_clk_div_gate(hw);
	unsigned int val;

	val = clk_readl(clk->div_reg.divider_reg) >> (clk->div_reg.div_bits);
	val &= div_mask(clk->div_reg.div_field);

	return divider_recalc_rate(hw, parent_rate, val, NULL,
					clk->flags, clk->div_reg.div_field);
}

static int div_gate_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_div_gate *clk = to_clk_div_gate(hw);
	int value, reg_val, reg0_val, reg1_val;
	unsigned long flags = 0;
	u32 val;

	if (clk->lock)
		raw_spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	/* disable clk */
	if(clk->gate_reg.disable_bit == 32) {
		reg_val = readl(clk->gate_reg.clken_sta_reg);
		reg_val |= 0 << clk->gate_reg.enable_bit;
		writel(reg_val, clk->gate_reg.enable_reg);
	} else {
		reg0_val = 1 << clk->gate_reg.disable_bit;
		writel(reg0_val, clk->gate_reg.disable_reg);
	}

	value = divider_get_val(rate, parent_rate, NULL,
				(u8)(clk->div_reg.div_field), clk->flags);
	if (value < 0) {
		if (clk->lock) {
		raw_spin_unlock_irqrestore(clk->lock, flags);
		} else {
		__release(clk->lock);
		}
		return value;
	}

	if (clk->flags & CLK_DIVIDER_HIWORD_MASK) {
		val = div_mask(clk->div_reg.div_field) << (clk->div_reg.div_bits + 16);
	} else {
		val = clk_readl(clk->div_reg.divider_reg);
		val &= ~(div_mask(clk->div_reg.div_field) << clk->div_reg.div_bits);
	}
	val |= (u32)value << clk->div_reg.div_bits;
	clk_writel(val, clk->div_reg.divider_reg);

	/* enable clk */
	if(clk->gate_reg.disable_bit == 32) {
		reg_val = readl(clk->gate_reg.clken_sta_reg);
		reg_val |= 1 << clk->gate_reg.enable_bit;
		writel(reg_val, clk->gate_reg.enable_reg);
	} else {
		reg1_val = 1 << clk->gate_reg.enable_bit;
		writel(reg1_val, clk->gate_reg.enable_reg);
	}

	if (clk->lock)
		raw_spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return 0;
}

static long div_gate_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct clk_div_gate *clk = to_clk_div_gate(hw);
	int bestdiv;

	/* if read only, just return current value */
	if (clk->flags & CLK_DIVIDER_READ_ONLY) {
		bestdiv = clk_readl(clk->div_reg.divider_reg) >> (clk->div_reg.div_bits);
		bestdiv &= div_mask(clk->div_reg.div_field);
		bestdiv = _get_div(NULL, bestdiv, clk->flags,
			(u8)(clk->div_reg.div_field));
		if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
			struct clk_hw *parent = clk_hw_get_parent(hw);
			*prate = clk_hw_round_rate(parent, rate * bestdiv);
		}
		return DIV_ROUND_UP_ULL((u64)*prate, bestdiv);
	}
	return divider_round_rate(hw, rate, prate, NULL,
					(u8)(clk->div_reg.div_field), clk->flags);
}

const struct clk_ops div_gate_clk_ops = {
	.enable = &div_gate_clk_enable,
	.disable = &div_gate_clk_disable,
	.is_enabled = &div_gate_clk_is_enabled,

	.recalc_rate = &div_gate_clk_recalc_rate,
	.round_rate = &div_gate_clk_round_rate,
	.set_rate = &div_gate_clk_set_rate,
};

static struct clk *div_gate_clk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags, struct clk_div_reg *div_reg,
		struct clk_gate_reg *gate_reg, unsigned int clk_gate_flags,
		unsigned int clk_divider_flags, raw_spinlock_t *lock,
		const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct clk_div_gate *clk_hw;
	struct clk *clk;
	int ret;

	clk_hw = kzalloc(sizeof(*clk_hw), GFP_KERNEL);
	if (!clk_hw) {
		pr_err("%s: failed to malloc memory for %s!\n", __func__, name);
		return NULL;
	}

	clk_hw->hw.init = &init;
	init.name = name;
	init.ops = ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags | CLK_IS_BASIC;

	memcpy(&clk_hw->div_reg, div_reg, sizeof(*div_reg));
	memcpy(&clk_hw->gate_reg, gate_reg, sizeof(*gate_reg));
	clk_hw->flags = clk_divider_flags;

	clk_hw->lock = lock;
	clk = clk_register(dev, &clk_hw->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register clock for %s!\n", __func__, name);
		kfree(clk_hw);
		return NULL;
	}

	ret = clk_hw_register_clkdev(&clk_hw->hw, name, NULL);
	if (ret) {
		pr_err("%s: failed to register lookup %s!\n", __func__, name);
		clk_unregister(clk);
		kfree(clk_hw);
		return NULL;
	}
	return clk;
}

static void __init _of_hobot_div_gate_clk_setup(struct device_node *node,
		const struct clk_ops *ops)
{
	struct clk *clk;
	struct clk_gate_reg gate_reg;
	struct clk_div_reg div_reg;
	const char *parent_name;
	unsigned int flags = 0;
	unsigned int val;
	unsigned int clk_gate_flags = 0;
	unsigned int clk_divider_flags = 0;
	unsigned int data[5] = {0};
	static void __iomem *reg_base;
	static void __iomem *ipsreg_base;
	raw_spinlock_t *lock;
	int ret;

	lock = kzalloc(sizeof(*lock), GFP_KERNEL);
	if(!lock)
		return;
	raw_spin_lock_init(lock);

	if(of_clk_get_parent_count(node) != 1) {
		pr_err("%s: %s must have 1 parent\n", __func__, node->name);
		goto err;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	reg_base = clk_get_register_base(node);
	if (!reg_base) {
		pr_err("%s: %s failed to get the reg base!\n", __func__, node->name);
		goto err;
	}

	ret = of_property_read_u32_array(node, "bits", data, 4);
	if (ret) {
		pr_err("%s:%s no bits property", __func__, node->name);
		goto err;
	}
	div_reg.div_bits = data[0];
	gate_reg.clken_sta_bit = data[1];
	gate_reg.enable_bit = data[2];
	gate_reg.disable_bit = data[3];

	if (gate_reg.disable_bit != 32) {
		ret = of_property_read_u32_array(node, "offset", data, 4);
		if (ret) {
			pr_err("%s: %s no offset property", __func__, node->name);
			goto err;
		}
		div_reg.divider_reg = reg_base + data[0];
		gate_reg.clken_sta_reg = reg_base + data[1];
		gate_reg.enable_reg = reg_base + data[2];
		gate_reg.disable_reg = reg_base + data[3];
	} else {
		ipsreg_base = clk_get_ipsregister_base(node);
		if (!ipsreg_base) {
			pr_err("%s: %s failed to get the reg base!\n", __func__, node->name);
			goto err;
		}

		ret = of_property_read_u32_array(node, "offset", data, 4);
		if (ret) {
			pr_err("%s:%s no offset property", __func__, node->name);
			goto err;
		}
		div_reg.divider_reg = reg_base + data[0];
		gate_reg.clken_sta_reg = ipsreg_base + data[1];
		gate_reg.enable_reg = ipsreg_base + data[2];
		gate_reg.disable_reg = ipsreg_base + data[3];
	}

	ret = of_property_read_u32_array(node, "field", data, 4);
	if (ret) {
		pr_err("%s:%s no field property", __func__, node->name);
		goto err;
	}
	div_reg.div_field = data[0];
	gate_reg.clken_sta_field = data[1];
	gate_reg.enable_field = data[2];
	gate_reg.disable_field = data[3];

	ret = of_property_read_u32(node, "clk-flags", &val);
	if (!ret)
		flags |= val;

	ret = of_property_read_u32(node, "clk-gate-flags", &val);
	if (!ret)
		clk_gate_flags |= val;

	ret = of_property_read_u32(node, "clk-divider-flags", &val);
	if (!ret)
		clk_divider_flags |= val;

	clk = div_gate_clk_register(NULL, node->name, parent_name, flags, &div_reg,
		&gate_reg, clk_gate_flags, clk_divider_flags, lock, ops);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	// pr_info("%s: %s div_gate clock set up.\n", __func__, node->name);

	return;

err:
	kfree(lock);
	return;
}

static void __init of_hobot_div_gate_clk_setup(struct device_node *node)
{
	_of_hobot_div_gate_clk_setup(node, &div_gate_clk_ops);
}
CLK_OF_DECLARE(hobot_div_gate_clk, "hobot,div-gate-clk",
				of_hobot_div_gate_clk_setup);
