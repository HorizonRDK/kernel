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

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/clkdev.h>

#include "common.h"

static void __init _of_hobot_divider_clk_setup(struct device_node *node)
{
	struct clk_hw *clk_hw;
	const char* parent_name;
	void __iomem *reg_base;
	void __iomem *reg;
	unsigned int val;
	unsigned int offset;
	unsigned int field;
	unsigned int flags = 0;
	unsigned int clk_divider_flags = 0;
	spinlock_t *lock;
	int ret;

	if(of_clk_get_parent_count(node) != 1){
		pr_err("%s: %s must have 1 parent!\n", __func__, node->name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	reg_base = clk_get_register_base(node);

	ret = of_property_read_u32(node, "offset", &val);
	if(ret){
		pr_err("%s: %s missing offset property!\n", __func__, node->name);
		return;
	}
	reg = reg_base + val;

	ret = of_property_read_u32(node, "bits", &val);
	if(ret){
		pr_err("%s: %s missing bits property!\n", __func__, node->name);
		return;
	}
	offset = val;

	ret = of_property_read_u32(node, "field", &val);
	if (ret) {
		pr_err("%s: %s missing field property!\n", __func__, node->name);
		return;
	}
	field = val;

	ret = of_property_read_u32(node, "clk-flags", &val);
	if (!ret)
		flags |= val;

	ret = of_property_read_u32(node, "clk-divider-flags", &val);
	if (!ret)
		clk_divider_flags |= val;

	lock = kzalloc(sizeof(spinlock_t), GFP_KERNEL);
	if(!lock){
		pr_err("%s: %s failed to malloc spinlock!\n", __func__, node->name);
		return;
	}
	spin_lock_init(lock);

	clk_hw = clk_hw_register_divider(NULL, node->name, parent_name, flags, reg,
			(u8)offset, (u8)field, (u8)clk_divider_flags, lock);
	if(IS_ERR(clk_hw)){
		kfree(lock);
		pr_err("%s: %s failed to register the clock!\n", __func__, node->name);
		return;
	}

	ret = clk_hw_register_clkdev(clk_hw, node->name, NULL);
	if(ret){
		pr_err("%s: failed to register lookup %s!\n", __func__, node->name);
		kfree(lock);
		clk_hw_unregister_divider(clk_hw);
		return;
	}

	of_clk_add_provider(node, of_clk_src_simple_get, clk_hw->clk);
	//pr_info("%s: %s divider clock set up.\n", __func__, node->name);
	return;
}

static void __init of_hobot_divider_clk_setup(struct device_node *node)
{
	_of_hobot_divider_clk_setup(node);
}
CLK_OF_DECLARE(hobot_div_clk, "hobot,div-clk", of_hobot_divider_clk_setup);
