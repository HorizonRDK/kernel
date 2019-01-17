#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "common.h"

#define to_clk_gpio_x2(_hw) container_of(_hw, struct clk_gpio_x2, hw)

struct clk_gpio_x2 {
	struct clk_hw hw;
	void __iomem *reg;
	unsigned int bit;
	unsigned int flags;
	spinlock_t lock;
};

int x2_gpio_clk_enable(struct clk_hw *hw)
{
	struct clk_gpio_x2 *clk;
	unsigned int val, status;

	clk = to_clk_gpio_x2(hw);

	__acquire(clk->lock);
	val = readl(clk->reg);
	status = (val & (1 << clk->bit)) >> clk->bit;

	if(status){
		val &= ~(1 << clk->bit);
		writel(val, clk->reg);
	}

	__release(clk->lock);
	return 0;
}

void x2_gpio_clk_disable(struct clk_hw *hw)
{

	struct clk_gpio_x2 *clk;
	unsigned int val, status;

	clk = to_clk_gpio_x2(hw);

	__acquire(clk->lock);
	val = readl(clk->reg);
	status = (val & (1 << clk->bit)) >> clk->bit;

	if(!status){
		val |= (1 << clk->bit);
		writel( val, clk->reg);
	}

	__release(clk->lock);
	return;
}

int x2_gpio_clk_is_enabled(struct clk_hw *hw)
{
	struct clk_gpio_x2 *clk;
	unsigned int val, status;

	clk = to_clk_gpio_x2(hw);

	val = readl(clk->reg);
	status = (val & (1 << clk->bit)) >> clk->bit;

	return status?0:1;
}

const struct clk_ops x2_gpio_clk_ops = {
	.enable = &x2_gpio_clk_enable,
	.disable = &x2_gpio_clk_disable,
	.is_enabled = &x2_gpio_clk_is_enabled,
};

static struct clk *x2_gpio_clk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags, void __iomem *reg, unsigned int bit,
		unsigned int clk_gpio_flags, const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct clk_gpio_x2 *clk_hw;
	struct clk *clk;
	int ret;

	clk_hw = kzalloc(sizeof(*clk_hw), GFP_KERNEL);
	if(!clk_hw){
		pr_err("%s: failed to malloc memory for %s!\n", __func__, name);
		return NULL;
	}

	clk_hw->hw.init = &init;
	init.name = name;
	init.ops = ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	clk_hw->reg = reg;
	clk_hw->bit = bit;
	clk_hw->flags = clk_gpio_flags;

	spin_lock_init(&clk_hw->lock);
	clk = clk_register(dev, &clk_hw->hw);
	if(IS_ERR(clk)){
		pr_err("%s: failed to register clock for %s!\n", __func__, name);
		kfree(clk_hw);
		return NULL;
	}

	ret = clk_hw_register_clkdev(&clk_hw->hw, name, NULL);
	if(ret){
		pr_err("%s: failed to register lookup %s!\n", __func__, name);
		clk_unregister(clk);
		kfree(clk_hw);
		return NULL;
	}

	return clk;
}

static void __init _of_x2_gpio_clk_setup(struct device_node *node, const struct clk_ops *ops)
{
	struct clk *clk;
	const char *parent_name;
	unsigned int flags = 0;
	unsigned int clk_gpio_flags = 0;
	unsigned int bit;
	unsigned int val;
	void __iomem *reg_base;
	void __iomem *reg;
	int ret;

	if(of_clk_get_parent_count(node) != 1){
		pr_err("%s: %s must have 1 parent\n", __func__, node->name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	reg_base = clk_get_register_base(node);
	if(!reg_base){
		pr_err("%s: %s failed to get the reg base!\n", __func__, node->name);
		return;
	}

	ret = of_property_read_u32(node, "offset", &val);
	if(ret){
		pr_err("%s: %s missing offset property", __func__, node->name);
		return;
	}
	reg = reg_base + val;

	ret = of_property_read_u32(node, "bits", &val);
	if(ret){
		pr_err("%s: %s missing bits property", __func__, node->name);
		return;
	}
	bit = val;

	clk = x2_gpio_clk_register(NULL, node->name, parent_name, flags, reg, bit, clk_gpio_flags, ops);

	if(!IS_ERR(clk)){
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	}

	//pr_info("%s: %s gpio clock set up.\n", __func__, node->name);
}

static void __init of_x2_gpio_clk_setup(struct device_node *node)
{
	_of_x2_gpio_clk_setup(node, &x2_gpio_clk_ops);
}
CLK_OF_DECLARE(x2_gpio_clk, "x2,gpio-clk", of_x2_gpio_clk_setup);
