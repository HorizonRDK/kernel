#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clkdev.h>

static void __init _of_x2_fixed_clk_setup(struct device_node *node)
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

static void __init of_x2_fixed_clk_setup(struct device_node *node)
{
	_of_x2_fixed_clk_setup(node);
}
CLK_OF_DECLARE(x2_fixed_clk, "x2,fixed-clk", of_x2_fixed_clk_setup);
