#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/clkdev.h>

#include "common.h"

static void __init _of_x2_mux_clk_setup(struct device_node *node)
{
	struct clk_hw *clk_hw;
	const char **parent_names;
	unsigned int num_parents;
	void __iomem *reg_base;
	void __iomem *reg;
	unsigned int val;
	unsigned int offset;
	unsigned int field;
	unsigned int flags = 0;
	unsigned int clk_mux_flags = 0;
	spinlock_t *lock;
	int ret;

	num_parents = of_clk_get_parent_count(node);
	if(num_parents < 2){
		pr_err("%s: %s %d must have at least 2 parent!\n", __func__, node->name, num_parents);
		return;
	}

	parent_names = kzalloc((sizeof(char*) * num_parents), GFP_KERNEL);
	if(!parent_names){
		pr_err("%s: %s failed to malloc memory for parent!\n", __func__, node->name);
		return;
	}

	of_clk_parent_fill(node, parent_names, num_parents);

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
	if(ret){
		pr_err("%s: %s missing field property!\n", __func__, node->name);
		return;
	}
	field = val;

	lock = kzalloc(sizeof(spinlock_t), GFP_KERNEL);
	if(!lock){
		pr_err("%s: %s failed to malloc spinlock!\n", __func__, node->name);
		return;
	}
	spin_lock_init(lock);

	clk_hw = clk_hw_register_mux(NULL, node->name, parent_names, num_parents, flags, reg, offset, field, clk_mux_flags, lock);
	if(IS_ERR(clk_hw)){
		kfree(parent_names);
		kfree(lock);
		pr_err("%s: %s failed to register the clock!\n", __func__, node->name);
		return;
	}

	ret = clk_hw_register_clkdev(clk_hw, node->name, NULL);
	if(ret){
		pr_err("%s: failed to register lookup %s!\n", __func__, node->name);
		kfree(parent_names);
		kfree(lock);
		clk_hw_unregister_divider(clk_hw);
		return;
	}

	of_clk_add_provider(node, of_clk_src_simple_get, clk_hw->clk);
	//pr_info("%s: %s mux clock set up.\n", __func__, node->name);
	return;
}

static void __init of_x2_mux_clk_setup(struct device_node *node)
{
	_of_x2_mux_clk_setup(node);
}
CLK_OF_DECLARE(x2_mux_clk, "x2,mux-clk", of_x2_mux_clk_setup);
