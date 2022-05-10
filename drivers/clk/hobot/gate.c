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
#include <linux/delay.h>
#include <linux/clk-provider.h>

#include "common.h"

struct clk_gate_reg {
	void __iomem *clken_sta_reg;
	unsigned int clken_sta_bit;
	unsigned int clken_sta_field;
	void __iomem *clkoff_sta_reg;
	unsigned int clkoff_sta_bit;
	unsigned int clkoff_sta_field;
	void __iomem *enable_reg;
	unsigned int enable_bit;
	unsigned int enable_field;
	void __iomem *disable_reg;
	unsigned int disable_bit;
	unsigned int disable_field;
};

#define to_hobot_clk_gate(_hw) container_of(_hw, struct hobot_clk_gate, hw)

struct hobot_clk_gate {
	struct clk_hw hw;
	struct clk_gate_reg reg;
	unsigned int flags;
	raw_spinlock_t *lock;
};

int gate_clk_enable(struct clk_hw *hw)
{
	struct hobot_clk_gate *clk;
	unsigned int val, val1, status0, status1, reg_val;
	unsigned long flags = 0;

	clk = to_hobot_clk_gate(hw);

	if (clk->lock)
		raw_spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	udelay(1);

	val = readl(clk->reg.clken_sta_reg);
	status0 = (val & (1 << clk->reg.clken_sta_bit))
	>> clk->reg.clken_sta_bit;
	/* determining if clkoff_sta_reg exists */
	if (clk->reg.clkoff_sta_bit != 32) {
		val1 = readl(clk->reg.clkoff_sta_reg);
		status1 = (val1 & (1 << clk->reg.clkoff_sta_bit))
		>> clk->reg.clkoff_sta_bit;

		if (status0 == 0 || status1 == 1) {
			reg_val = 1 << clk->reg.enable_bit;
			writel(reg_val, clk->reg.enable_reg);
		}
	} else {
		if(clk->reg.disable_bit == 32) {
			if (!status0) {
				val |= 1 << clk->reg.enable_bit;
				writel(val, clk->reg.enable_reg);
			}
		} else {
			if (!status0) {
				reg_val = 1 << clk->reg.enable_bit;
				writel(reg_val, clk->reg.enable_reg);
			}
		}
	}

	if (clk->lock)
		raw_spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return 0;
}

void gate_clk_disable(struct clk_hw *hw)
{
	struct hobot_clk_gate *clk;
	unsigned int val, val1, status0, status1, reg_val;
	unsigned long flags = 0;

	clk = to_hobot_clk_gate(hw);

	if (clk->lock)
		raw_spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	val = readl(clk->reg.clken_sta_reg);
	status0 = (val & (1 << clk->reg.clken_sta_bit))
	>> clk->reg.clken_sta_bit;
	/* determining if clkoff_sta_reg exists */
	if (clk->reg.clkoff_sta_bit != 32) {
		val1 = readl(clk->reg.clkoff_sta_reg);
		status1 = (val1 & (1 << clk->reg.clkoff_sta_bit))
		>> clk->reg.clkoff_sta_bit;

		if (status0 == 1 && status1 == 0) {
			reg_val = 1 << clk->reg.disable_bit;
			writel(reg_val, clk->reg.disable_reg);
		}
	} else {
		if(clk->reg.disable_bit == 32) {
			if (status0) {
				val &= ~(1 << clk->reg.enable_bit);
				writel(val, clk->reg.enable_reg);
			}
		} else {
			if (status0) {
				reg_val = 1 << clk->reg.disable_bit;
				writel(reg_val, clk->reg.disable_reg);
			}
		}
	}

	udelay(1);

	if (clk->lock)
		raw_spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return;
}

int gate_clk_is_enabled(struct clk_hw *hw)
{
	struct hobot_clk_gate *clk;
	unsigned int val, status;

	clk = to_hobot_clk_gate(hw);

	val = readl(clk->reg.clken_sta_reg);
	status = (val & (1 << clk->reg.clken_sta_bit))
	>> clk->reg.clken_sta_bit;

	return status;
}

const struct clk_ops gate_clk_ops = {
	.enable = &gate_clk_enable,
	.disable = &gate_clk_disable,
	.is_enabled = &gate_clk_is_enabled,
};

static struct clk *gate_clk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags, struct clk_gate_reg *reg,
		unsigned int clk_gate_flags, raw_spinlock_t *lock, const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct hobot_clk_gate *clk_hw;
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

	memcpy(&clk_hw->reg, reg, sizeof(*reg));
	clk_hw->flags = clk_gate_flags;

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

static void __init _of_hobot_gate_clk_setup(struct device_node *node,
		const struct clk_ops *ops)
{
	struct clk *clk;
	struct clk_gate_reg reg;
	const char *parent_name;
	unsigned int flags = CLK_OPS_PARENT_ENABLE;
	unsigned int val;
	unsigned int clk_gate_flags = 0;
	unsigned int data[4] = {0};
	static void __iomem *reg_base;
	static void __iomem *ipsreg_base;
	raw_spinlock_t *lock;
	int ret;

	lock = kzalloc(sizeof(*lock), GFP_KERNEL);
	if(!lock)
		return;
	raw_spin_lock_init(lock);

	if (of_clk_get_parent_count(node) != 1) {
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
	reg.clken_sta_bit = data[0];
	reg.enable_bit = data[1];
	reg.disable_bit = data[2];
	reg.clkoff_sta_bit = data[3];

	if (reg.disable_bit != 32) {
		ret = of_property_read_u32_array(node, "offset", data, 4);
		if (ret) {
			pr_err("%s: %s missing offset property", __func__, node->name);
			goto err;
		}
		reg.clken_sta_reg = reg_base + data[0];
		reg.enable_reg = reg_base + data[1];
		reg.disable_reg = reg_base + data[2];
		reg.clkoff_sta_reg = reg_base + data[3];
	} else {
		ipsreg_base = clk_get_ipsregister_base(node);
		if (!ipsreg_base) {
			pr_err("%s: %s failed to get the reg base!\n", __func__, node->name);
			goto err;
		}

		ret = of_property_read_u32_array(node, "offset", data, 4);
		if (ret) {
			pr_err("%s: %s missing offset property", __func__, node->name);
			goto err;
		}
		reg.clken_sta_reg = ipsreg_base + data[0];
		reg.enable_reg = ipsreg_base + data[1];
		reg.disable_reg = ipsreg_base + data[2];
		reg.clkoff_sta_reg = ipsreg_base + data[3];
	}

	if (reg.clkoff_sta_reg != reg_base &&
		reg.clkoff_sta_reg != ipsreg_base) {
		ret = of_property_read_u32_array(node, "field", data, 4);
		if (ret) {
			pr_err("%s:%s no field property", __func__, node->name);
			goto err;
		}
		reg.clken_sta_field = data[0];
		reg.enable_field = data[1];
		reg.disable_field = data[2];
		reg.clkoff_sta_field = data[3];
	} else {
		ret = of_property_read_u32_array(node, "field", data, 3);
		if (ret) {
			pr_err("%s:%s no field property", __func__, node->name);
			goto err;
		}
		reg.clken_sta_field = data[0];
		reg.enable_field = data[1];
		reg.disable_field = data[2];
	}

	ret = of_property_read_u32(node, "clk-flags", &val);
	if (!ret)
		flags |= val;

	clk = gate_clk_register(NULL, node->name, parent_name,
		flags, &reg, clk_gate_flags, lock, ops);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	/* pr_info("%s: %s gate clock set up.\n", __func__, node->name); */
	return;
err:
	kfree(lock);
	return;
}

static void __init of_hobot_gate_clk_setup(struct device_node *node)
{
	_of_hobot_gate_clk_setup(node, &gate_clk_ops);
}
CLK_OF_DECLARE(hobot_gate_clk, "hobot,gate-clk", of_hobot_gate_clk_setup);
