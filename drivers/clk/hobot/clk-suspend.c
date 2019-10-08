/*
 * Horizon Robotics-specific suspend support
 *
 *  Copyright (C) 2019 Horizon Robotics Inc.
 *  All rights reserved.
 *  Author: leye.wang<leye.wang@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define pr_fmt(fmt) "CLK suspend: " fmt

#include <linux/syscore_ops.h>
#include <linux/suspend.h>

#include "common.h"

#define PLLCLK_SEL			0x300
#define VIOPLL_PD_CTRL		0x44
#define CNNPLL_PD_CTRL		0x24
#define VIOSYS_CLK_DIV_SEL1	0x240
#define VIOSYS_CLK_DIV_SEL2	0x244

static u32 pllclk_sel = 0x111110;
static u32 viopll_pd_ctrl;
static u32 viosys_clk_div_sel1;
static u32 viosys_clk_div_sel2;
static u32 cnnpll_pd_ctrl;

extern void __iomem *clk_reg_base;

static u32 reg_read(char *string, int offset)
{
	u32 value;

	value = readl(clk_reg_base + offset);
	pr_info("Reg[%s], \toffset: 0x%03x, \tvalue: 0x%x\n", string, offset, value);

	return value;
}

static void dump_sysctrl_regs(void)
{
	pr_info("\n###########ARMPLL##########\n");
	reg_read("ARMPLL1_FREQ_CTRL", 0x0);
	reg_read("ARMPLL1_PD_CTRL", 0x04);
	reg_read("ARMPLL1_STATUS", 0x08);
	reg_read("ARMPLL2_FREQ_CTRL", 0x10);
	reg_read("ARMPLL2_PD_CTRL", 0x14);
	reg_read("ARMPLL2_STATUS", 0x18);
	reg_read("CPUSYS_CLKEN", 0x100);
	reg_read("CPUSYS_CLK_DIV_SEL", 0x200);
	reg_read("CPUBUS_CLK_DIV_SEL", 0x204);
	reg_read("CPUSYS_CLKOFF_STA", 0x208);
	reg_read("PLLCLK_SEL", 0x300);
	reg_read("CPUSYS_SW_RSTEN", 0x400);

	pr_info("\n###########CNNPLL##########\n");
	reg_read("CNNPLL_FREQ_CTRL", 0x20);
	reg_read("CNNPLL_PD_CTRL", 0x24);
	reg_read("CNNPLL_STATUS", 0x28);
	reg_read("CNNSYS_CLKEN", 0x120);
	reg_read("CNNSYS_CLK_DIV_SEL", 0x220);
	reg_read("CNNSYS_CLKOFF_STA", 0x228);
	reg_read("CNNSYS_SW_RSTEN", 0x420);

	pr_info("\n###########DDRPLL##########\n");
	reg_read("DDRPLL_FREQ_CTRL", 0x30);
	reg_read("DDRPLL_PD_CTRL", 0x34);
	reg_read("DDRPLL_STATUS", 0x38);
	reg_read("DDRPLL_FRAC", 0x3C);
	reg_read("DDRSYS_CLKEN", 0x130);
	reg_read("DDRSYS_CLK_DIV_SEL", 0x230);
	reg_read("DDRSYS_CLKOFF_STA", 0x238);
	reg_read("DDRSYS_SW_RSTEN", 0x430);

	pr_info("\n###########VIOPLL##########\n");
	reg_read("VIOPLL_FREQ_CTRL", 0x40);
	reg_read("VIOPLL_PD_CTRL", 0x44);
	reg_read("VIOPLL_STATUS", 0x48);
	reg_read("VIOSYS_CLKEN", 0x140);
	reg_read("VIOSYS_CLK_DIV_SEL1", 0x240);
	reg_read("VIOSYS_CLK_DIV_SEL2", 0x244);
	reg_read("VIOSYS_CLKOFF_STA", 0x248);
	reg_read("VIOSYS_CLK_CTRL", 0x310);
	reg_read("VIOSYS_SW_RSTEN", 0x440);

	pr_info("\n###########PERIPLL##########\n");
	reg_read("PERIPLL_FREQ_CTRL", 0x50);
	reg_read("PERIPLL_PD_CTRL", 0x54);
	reg_read("PERIPLL_STATUS", 0x58);
	reg_read("PERISYS_CLKEN", 0x150);
	reg_read("PERISYS_CLK_DIV_SEL", 0x250);
	reg_read("PERISYS_CLKOFF_STA", 0x258);
	reg_read("SD0_CCLK_CTRL", 0x320);
	reg_read("SD1_CCLK_CTRL", 0x330);
	reg_read("ETH0_CLK_CTRL", 0x340);
	reg_read("I2S0_CLK_CTRL", 0x350);
	reg_read("I2S1_CLK_CTRL", 0x360);
	reg_read("PLLCLK_TEST_CTRL", 0x370);
	reg_read("PERISYS_SW_RSTEN", 0x450);

}

#ifdef CONFIG_PM_SLEEP
static int x2_clk_suspend(void)
{
	pr_info("%s:%s, syscore enter suspend...\n", __FILE__, __func__);

	if (clk_reg_base == NULL) {
		pr_info("%s:%s, clk_reg_base == NULL\n", __FILE__, __func__);
		return 0;
	}

//	dump_sysctrl_regs();

	pllclk_sel = readl(clk_reg_base + PLLCLK_SEL);
	viopll_pd_ctrl = readl(clk_reg_base + VIOPLL_PD_CTRL);
	cnnpll_pd_ctrl = readl(clk_reg_base + CNNPLL_PD_CTRL);
	viosys_clk_div_sel1 = readl(clk_reg_base + VIOSYS_CLK_DIV_SEL1);
	viosys_clk_div_sel2 = readl(clk_reg_base + VIOSYS_CLK_DIV_SEL2);

	return 0;
}

static void x2_clk_resume(void)
{
	pr_info("%s:%s, syscore enter resume...\n", __FILE__, __func__);

	if (clk_reg_base == NULL) {
		pr_info("%s:%s, clk_reg_base == NULL\n", __FILE__, __func__);
		return;
	}

	writel(pllclk_sel, clk_reg_base + PLLCLK_SEL);
	writel(viopll_pd_ctrl, clk_reg_base + VIOPLL_PD_CTRL);
	writel(cnnpll_pd_ctrl, clk_reg_base + CNNPLL_PD_CTRL);
	writel(viosys_clk_div_sel1, clk_reg_base + VIOSYS_CLK_DIV_SEL1);
	writel(viosys_clk_div_sel2, clk_reg_base + VIOSYS_CLK_DIV_SEL2);

//	dump_sysctrl_regs();
}

static struct syscore_ops x2_clk_syscore_ops = {
	.suspend = x2_clk_suspend,
	.resume = x2_clk_resume,
};
#endif

static int __init x2_clk_suspend_init(void)
{
	pr_info("%s:%s, register syscore operations.\n", __FILE__, __func__);
#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&x2_clk_syscore_ops);
#endif

	return 0;
}
arch_initcall(x2_clk_suspend_init);
