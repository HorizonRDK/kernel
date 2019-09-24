#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clkdev.h>
#include "common.h"

#define div_mask(width) ((1 << (width)) - 1)

static void __init _of_x2_fixed_factor_clk_setup(struct device_node *node)
{
	struct clk_hw *clk_hw;
	const char* parent_name;
	unsigned int div, mult;
	unsigned int flags = 0;
	unsigned int offset;
	unsigned int field;
	void __iomem *reg_base;
	void __iomem *reg;
	unsigned int val;
	int ret;

	if(of_clk_get_parent_count(node) != 1){
		pr_err("%s: %s must have 1 parent!\n", __func__, node->name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);
	reg_base = clk_get_register_base(node);

	ret = of_property_read_u32(node, "offset", &val);
	if (!ret)
		reg = reg_base + val;
	else
		reg = NULL;

	ret = of_property_read_u32(node, "bits", &val);
	if (!ret)
		offset = val;

	ret = of_property_read_u32(node, "field", &val);
	if (!ret)
		field = val;

	ret = of_property_read_u32(node, "clk-div", &div);
	if (ret || div < 1) {
		pr_err("%s: %s missing clk-div property or div is invalid!\n",
								node->name);
		return;
	}

	ret = of_property_read_u32(node, "clk-mult", &mult);
	if (ret) {
		pr_err("%s: %s missing clk-mult property!\n", __func__, node->name);
		return;
	}

	if (reg != NULL) {
		val = clk_readl(reg);
		val &= ~(div_mask(field) << offset);
		val |= (div - 1) << offset;
		clk_writel(val, reg);
	}

	ret = of_property_read_u32(node, "clk-flags", &val);
	if (!ret)
		flags |= val;

	clk_hw = clk_hw_register_fixed_factor(NULL, node->name, parent_name, flags, mult, div);
	if (IS_ERR(clk_hw)) {
		pr_err("%s: %s failed to register the clock!\n", __func__, node->name);
		return;
	}

	ret = clk_hw_register_clkdev(clk_hw, node->name, NULL);
	if (ret) {
		pr_err("%s: failed to register lookup %s!\n", __func__, node->name);
		clk_hw_unregister_fixed_factor(clk_hw);
		return;
	}

	of_clk_add_provider(node, of_clk_src_simple_get, clk_hw->clk);
	//pr_info("%s: %s fixed factor clock set up.\n", __func__, node->name);
	return;
}

static void __init of_x2_fixed_factor_clk_setup(struct device_node *node)
{
	_of_x2_fixed_factor_clk_setup(node);
}
CLK_OF_DECLARE(x2_fixed_factor_clk, "x2,fixed-factor-clk", of_x2_fixed_factor_clk_setup);
