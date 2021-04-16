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
#include <linux/clkdev.h>

static void __init _of_hobot_fixed_clk_setup(struct device_node *node)
{
	struct clk_hw *clk_hw;
	unsigned int fixed_rate;
	unsigned int flags = 0;
	int ret;

	ret = of_property_read_u32(node, "clock-freq", &fixed_rate);
	if(ret){
		pr_err("%s: %s missing clk-freq property!\n", __func__, node->name);
		return;
	}

	clk_hw = clk_hw_register_fixed_rate(NULL, node->name, NULL, flags, fixed_rate);
	if(IS_ERR(clk_hw)){
		pr_err("%s: %s failed to register the clock!\n", __func__, node->name);
		return;
	}

	ret = clk_hw_register_clkdev(clk_hw, node->name, NULL);
	if(ret){
		pr_err("%s: failed to register lookup %s!\n", __func__, node->name);
		clk_hw_unregister_fixed_rate(clk_hw);
		return;
	}

	of_clk_add_provider(node, of_clk_src_simple_get, clk_hw->clk);
	//pr_info("%s: %s fixed clock set up.\n", __func__, node->name);
	return;
}

static void __init of_hobot_fixed_clk_setup(struct device_node *node)
{
	_of_hobot_fixed_clk_setup(node);
}
CLK_OF_DECLARE(hobot_fixed_clk, "hobot,fixed-clk", of_hobot_fixed_clk_setup);
