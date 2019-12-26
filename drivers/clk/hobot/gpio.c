#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "common.h"

#define to_hobot_clk_gpio(_hw) container_of(_hw, struct hobot_clk_gpio, hw)

struct hobot_clk_gpio {
	struct clk_hw hw;
	void __iomem *reg;
	unsigned int bit;
	unsigned int flags;
	spinlock_t *lock;
};

int gpio_clk_enable(struct clk_hw *hw)
{
	struct hobot_clk_gpio *clk;
	unsigned int val, status;
	unsigned long flags = 0;

	clk = to_hobot_clk_gpio(hw);

	if (clk->lock)
		spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	val = readl(clk->reg);
	status = (val & (1 << clk->bit)) >> clk->bit;

	if(status){
		val &= ~(1 << clk->bit);
		writel(val, clk->reg);
	}

	if (clk->lock)
		spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return 0;
}

void gpio_clk_disable(struct clk_hw *hw)
{

	struct hobot_clk_gpio *clk;
	unsigned int val, status;
	unsigned long flags = 0;

	clk = to_hobot_clk_gpio(hw);

	if (clk->lock)
		spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	val = readl(clk->reg);
	status = (val & (1 << clk->bit)) >> clk->bit;

	if(!status){
		val |= (1 << clk->bit);
		writel( val, clk->reg);
	}

	if (clk->lock)
		spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return;
}

int gpio_clk_is_enabled(struct clk_hw *hw)
{
	struct hobot_clk_gpio *clk;
	unsigned int val, status;

	clk = to_hobot_clk_gpio(hw);

	val = readl(clk->reg);
	status = (val & (1 << clk->bit)) >> clk->bit;

	return status?0:1;
}

const struct clk_ops gpio_clk_ops = {
	.enable = &gpio_clk_enable,
	.disable = &gpio_clk_disable,
	.is_enabled = &gpio_clk_is_enabled,
};

static struct clk *gpio_clk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags, void __iomem *reg, unsigned int bit,
		unsigned int clk_gpio_flags, spinlock_t *lock, const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct hobot_clk_gpio *clk_hw;
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

	clk_hw->lock = lock;
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

static void __init _of_hobot_gpio_clk_setup(struct device_node *node,
	const struct clk_ops *ops)
{
	struct clk *clk;
	const char *parent_name;
	unsigned int flags = 0;
	unsigned int clk_gpio_flags = 0;
	unsigned int bit;
	unsigned int val;
	void __iomem *reg_base;
	void __iomem *reg;
	spinlock_t *lock;
	int ret;

	lock = kzalloc(sizeof(*lock), GFP_KERNEL);
	if(!lock)
		return;
	spin_lock_init(lock);

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

	ret = of_property_read_u32(node, "clk-flags", &val);
	if (!ret)
		flags |= val;

	clk = gpio_clk_register(NULL, node->name, parent_name, flags, reg,
		bit, clk_gpio_flags, lock, ops);

	if(!IS_ERR(clk)){
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	}
	//pr_info("%s: %s gpio clock set up.\n", __func__, node->name);
}

static void __init of_hobot_gpio_clk_setup(struct device_node *node)
{
	_of_hobot_gpio_clk_setup(node, &gpio_clk_ops);
}
CLK_OF_DECLARE(hobot_gpio_clk, "hobot,gpio-clk", of_hobot_gpio_clk_setup);
