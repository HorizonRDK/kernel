/*
 *  /drivers/clk/hobot/div_comm.h
 *
 *  This head file can be  used for div-gate.c and endiv_gate.c.
 *  In HOBOT clock driver, div-gate.c and endiv_gate.c have common interfaces to use.
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

