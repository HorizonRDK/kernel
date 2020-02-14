#include "div-comm.h"

struct clk_engate_reg{
	struct clk_gate_reg gt_reg;
	void __iomem *clkoff_sta_reg;
	unsigned int clkoff_sta_bit;
	unsigned int clkoff_sta_field;
};

struct clk_invert_reg{
	void __iomem *inv_reg;
	unsigned int inv_bit;
	unsigned int inv_field;
};

#define to_clk_endiv(_hw) container_of(_hw, struct clk_endiv, hw)

struct clk_endiv {
	struct clk_hw hw;
	struct clk_div_reg div_reg;
	struct clk_engate_reg gate_reg;
	struct clk_invert_reg invert_reg;
	unsigned int flags;
	spinlock_t *lock;
};

int endiv_clk_enable(struct clk_hw *hw)
{
	struct clk_endiv *clk = to_clk_endiv(hw);
	unsigned int val, val1, status0, status1, reg_val;
	unsigned long flags = 0;

	if (clk->lock)
		spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	val = readl(clk->gate_reg.gt_reg.clken_sta_reg);
	status0 = (val & (1 << clk->gate_reg.gt_reg.clken_sta_bit))
	>> clk->gate_reg.gt_reg.clken_sta_bit;

	val1 = readl(clk->gate_reg.clkoff_sta_reg);
	status1 = (val1 & (1 << clk->gate_reg.clkoff_sta_bit))
	>> clk->gate_reg.clkoff_sta_bit;

	if (status0 == 0 || status1 == 1) {
		reg_val = 1 << clk->gate_reg.gt_reg.enable_bit;
		writel(reg_val, clk->gate_reg.gt_reg.enable_reg);
	}

	if (clk->lock)
		spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return 0;
}

void endiv_clk_disable(struct clk_hw *hw)
{
	struct clk_endiv *clk = to_clk_endiv(hw);
	unsigned int val, val1, status0, status1, reg_val;
	unsigned long flags = 0;

	if (clk->lock)
		spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	val = readl(clk->gate_reg.gt_reg.clken_sta_reg);
	status0 = (val & (1 << clk->gate_reg.gt_reg.clken_sta_bit))
	>> clk->gate_reg.gt_reg.clken_sta_bit;

	val1 = readl(clk->gate_reg.clkoff_sta_reg);
	status1 = (val1 & (1 << clk->gate_reg.clkoff_sta_bit))
	>> clk->gate_reg.clkoff_sta_bit;

	if (status0 == 1 && status1 == 0) {
		reg_val = 1 << clk->gate_reg.gt_reg.disable_bit;
		writel(reg_val, clk->gate_reg.gt_reg.disable_reg);
	}

	if (clk->lock)
		spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return;
}

int endiv_clk_is_enabled(struct clk_hw *hw)
{
	struct clk_endiv *clk;
	unsigned int val, status;

	clk = to_clk_endiv(hw);

	val = readl(clk->gate_reg.gt_reg.clken_sta_reg);
	status = (val & (1 << clk->gate_reg.gt_reg.clken_sta_bit))
	>> clk->gate_reg.gt_reg.clken_sta_bit;

	return status;
}

unsigned long endiv_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_endiv *clk = to_clk_endiv(hw);
	unsigned int val;

	val = clk_readl(clk->div_reg.divider_reg) >> (clk->div_reg.div_bits);
	val &= div_mask(clk->div_reg.div_field);

	return divider_recalc_rate(hw, parent_rate, val, NULL,
					clk->flags, clk->div_reg.div_field);
}

static int endiv_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_endiv *clk = to_clk_endiv(hw);
	int value, reg0_val, reg1_val;
	unsigned long flags = 0;
	u32 val;

	if (clk->lock)
		spin_lock_irqsave(clk->lock, flags);
	else
		__acquire(clk->lock);

	/* disable clk */
	reg0_val = 1 << clk->gate_reg.gt_reg.disable_bit;
	writel(reg0_val, clk->gate_reg.gt_reg.disable_reg);

	value = divider_get_val(rate, parent_rate, NULL,
				clk->div_reg.div_field, clk->flags);
	if (value < 0) {
		if (clk->lock) {
		spin_unlock_irqrestore(clk->lock, flags);
		} else {
		__release(clk->lock);
		}
		return value;
	}

	if (clk->flags & CLK_DIVIDER_HIWORD_MASK) {
		val = div_mask(clk->div_reg.div_field) << (clk->div_reg.div_bits + 16);
	} else {
		val = clk_readl(clk->div_reg.divider_reg);
		val &= ~(div_mask(clk->div_reg.div_field) << clk->div_reg.div_bits);
	}
	val |= (u32)value << clk->div_reg.div_bits;
	clk_writel(val, clk->div_reg.divider_reg);

	/* enable clk */
	reg1_val = 1 << clk->gate_reg.gt_reg.enable_bit;
	writel(reg1_val, clk->gate_reg.gt_reg.enable_reg);

	if (clk->lock)
		spin_unlock_irqrestore(clk->lock, flags);
	else
		__release(clk->lock);

	return 0;
}

static long endiv_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct clk_endiv *clk = to_clk_endiv(hw);
	int bestdiv;

	/* if read only, just return current value */
	if (clk->flags & CLK_DIVIDER_READ_ONLY) {
		bestdiv = clk_readl(clk->div_reg.divider_reg) >> (clk->div_reg.div_bits);
		bestdiv &= div_mask(clk->div_reg.div_field);
		bestdiv = _get_div(NULL, bestdiv, clk->flags,
			clk->div_reg.div_field);
		if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
			struct clk_hw *parent = clk_hw_get_parent(hw);
			*prate = clk_hw_round_rate(parent, rate * bestdiv);
		}
		return DIV_ROUND_UP_ULL((u64)*prate, bestdiv);
	}

	return divider_round_rate(hw, rate, prate, NULL,
					clk->div_reg.div_field, clk->flags);
}

static int endiv_clk_set_phase(struct clk_hw *hw, int degrees)
{
	struct clk_endiv *clk_hw = to_clk_endiv(hw);
	unsigned int value;

	value = readl(clk_hw->invert_reg.inv_reg);
	value |= 1 << clk_hw->invert_reg.inv_bit;
	clk_writel(value, clk_hw->invert_reg.inv_reg);

	return 0;

//	return clk_set_phase(clk, degrees);
}

const struct clk_ops endiv_clk_ops = {
	.enable = &endiv_clk_enable,
	.disable = &endiv_clk_disable,
	.is_enabled = &endiv_clk_is_enabled,

	.recalc_rate = &endiv_clk_recalc_rate,
	.round_rate = &endiv_clk_round_rate,
	.set_rate = &endiv_clk_set_rate,

	.set_phase = &endiv_clk_set_phase,
};

static struct clk *endiv_clk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags, struct clk_div_reg *div_reg,
		struct clk_engate_reg *gate_reg, struct clk_invert_reg *invert_reg,
		unsigned int clk_gate_flags, unsigned int clk_divider_flags,
		spinlock_t *lock, const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct clk_endiv *clk_hw;
	struct clk *clk;
	int ret;

	clk_hw = kzalloc(sizeof(*clk_hw), GFP_KERNEL);
	if (!clk_hw) {
		pr_err("%s: failed to malloc memory for %s!\n", __func__, name);
		return NULL;
	}

	clk_hw->hw.init = &init;
	init.name = name;
	init.ops = ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags | CLK_IS_BASIC;

	memcpy(&clk_hw->div_reg, div_reg, sizeof(*div_reg));
	memcpy(&clk_hw->gate_reg, gate_reg, sizeof(*gate_reg));
	memcpy(&clk_hw->invert_reg, invert_reg, sizeof(*invert_reg));
	clk_hw->flags = clk_divider_flags;

	clk_hw->lock = lock;
	clk = clk_register(dev, &clk_hw->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register clock for %s!\n", __func__, name);
		kfree(clk_hw);
		return NULL;
	}

	ret = clk_hw_register_clkdev(&clk_hw->hw, name, NULL);
	if (ret) {
		pr_err("%s: failed to register lookup %s!\n", __func__, name);
		clk_unregister(clk);
		kfree(clk_hw);
		return NULL;
	}
	return clk;
}

static void __init _of_hobot_endiv_clk_setup(struct device_node *node,
		const struct clk_ops *ops)
{
	struct clk_engate_reg gate_reg;
	struct clk_div_reg div_reg;
	struct clk_invert_reg invert_reg;
	const char *parent_name;
	unsigned int flags = 0;
	unsigned int val;
	unsigned int clk_gate_flags = 0;
	unsigned int clk_divider_flags = 0;
	unsigned int data[6] = {0};
	static void __iomem *reg_base;
	spinlock_t *lock = NULL;
	struct clk *clk;
	int ret;

	lock = kzalloc(sizeof(*lock), GFP_KERNEL);
	if(!lock)
		return;
	spin_lock_init(lock);

	if(of_clk_get_parent_count(node) != 1) {
		pr_err("%s: %s must have 1 parent\n", __func__, node->name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);


	reg_base = clk_get_register_base(node);
	if (!reg_base) {
		pr_err("%s: %s failed to get the reg base!\n", __func__, node->name);
		return;
	}

	ret = of_property_read_u32_array(node, "bits", data, 6);
	if (ret) {
		pr_err("%s:%s no bits property", __func__, node->name);
		return;
	}
	div_reg.div_bits = data[0];
	gate_reg.gt_reg.clken_sta_bit = data[1];
	gate_reg.gt_reg.enable_bit = data[2];
	gate_reg.gt_reg.disable_bit = data[3];
	gate_reg.clkoff_sta_bit = data[4];
	invert_reg.inv_bit = data[5];

	ret = of_property_read_u32_array(node, "offset", data, 6);
	if (ret) {
		pr_err("%s: %s no offset property", __func__, node->name);
		return;
	}
	div_reg.divider_reg = reg_base + data[0];
	gate_reg.gt_reg.clken_sta_reg = reg_base + data[1];
	gate_reg.gt_reg.enable_reg = reg_base + data[2];
	gate_reg.gt_reg.disable_reg = reg_base + data[3];
	gate_reg.clkoff_sta_reg = reg_base + data[4];
	invert_reg.inv_reg = reg_base + data[5];

	if (invert_reg.inv_bit != 32) {
		ret = of_property_read_u32_array(node, "field", data, 6);
		if (ret) {
			pr_err("%s:%s no field property", __func__, node->name);
			return;
		}
		div_reg.div_field = data[0];
		gate_reg.gt_reg.clken_sta_field = data[1];
		gate_reg.gt_reg.enable_field = data[2];
		gate_reg.gt_reg.disable_field = data[3];
		gate_reg.clkoff_sta_field = data[4];
		invert_reg.inv_field = data[5];
	} else {
		ret = of_property_read_u32_array(node, "field", data, 5);
		if (ret) {
			pr_err("%s:%s no field property", __func__, node->name);
			return;
		}
		div_reg.div_field = data[0];
		gate_reg.gt_reg.clken_sta_field = data[1];
		gate_reg.gt_reg.enable_field = data[2];
		gate_reg.gt_reg.disable_field = data[3];
		gate_reg.clkoff_sta_field = data[4];
	}

	ret = of_property_read_u32(node, "clk-flags", &val);
	if (!ret)
		flags |= val;

	ret = of_property_read_u32(node, "clk-gate-flags", &val);
	if (!ret)
		clk_gate_flags |= val;

	ret = of_property_read_u32(node, "clk-divider-flags", &val);
	if (!ret)
		clk_divider_flags |= val;

	clk = endiv_clk_register(NULL, node->name, parent_name, flags,
		&div_reg, &gate_reg, &invert_reg, clk_gate_flags,
		clk_divider_flags, lock, ops);
	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	pr_debug("%s: %s endiv clock set up.\n", __func__, node->name);
}

static void __init of_hobot_endiv_clk_setup(struct device_node *node)
{
	_of_hobot_endiv_clk_setup(node, &endiv_clk_ops);
}
CLK_OF_DECLARE(hobot_endiv_clk, "hobot,endiv-clk", of_hobot_endiv_clk_setup);
