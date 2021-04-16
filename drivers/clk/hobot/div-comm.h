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

#ifndef DRIVERS_CLK_HOBOT_DIV_COMM_H_
#define DRIVERS_CLK_HOBOT_DIV_COMM_H_

#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/kernel.h>

#include "common.h"

#define div_mask(width)	((1 << (width)) - 1)

struct clk_div_reg{
	void __iomem *divider_reg;
	unsigned int div_bits;
	unsigned int div_field;
};

struct clk_gate_reg{
	void __iomem *clken_sta_reg;
	unsigned int clken_sta_bit;
	unsigned int clken_sta_field;
	void __iomem *enable_reg;
	unsigned int enable_bit;
	unsigned int enable_field;
	void __iomem *disable_reg;
	unsigned int disable_bit;
	unsigned int disable_field;
};

extern unsigned int _get_div(const struct clk_div_table *table,
				unsigned int val, unsigned long flags, u8 width);

#endif // DRIVERS_CLK_HOBOT_DIV_COMM_H_

