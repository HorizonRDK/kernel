#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>

#include "common.h"

#define PLL_FREQ_CTRL_FBDIV_BIT 0
#define PLL_FREQ_CTRL_REFDIV_BIT 12
#define PLL_FREQ_CTRL_POSTDIV1_BIT 20
#define PLL_FREQ_CTRL_POSTDIV2_BIT 24

#define PLL_FREQ_CTRL_FBDIV_FIELD 0xFFF
#define PLL_FREQ_CTRL_REFDIV_FIELD 0x3F
#define PLL_FREQ_CTRL_POSTDIV1_FIELD 0x7
#define PLL_FREQ_CTRL_POSTDIV2_FIELD 0x7

#define PLL_PD_CTRL_PD_BIT 0
#define PLL_PD_CTRL_DSMPD_BIT 4
#define PLL_PD_CTRL_FOUTPOSTDIVPD_BIT 8
#define PLL_PD_CTRL_FOUTVCOPD_BIT 12
#define PLL_PD_CTRL_BYPASS_BIT 16

#define PLL_BYPASS_MODE 0x1
#define PLL_LAINT_MODE 0x0

#define PLL_LAINT_REFDIV_MIN 1
#define PLL_LAINT_REFDIV_MAX 63
#define PLL_LAINT_FBDIV_MIN 16
#define PLL_LAINT_FBDIV_MAX 320
#define PLL_LAINT_POSTDIV1_MIN 1
#define PLL_LAINT_POSTDIV1_MAX 7
#define PLL_LAINT_POSTDIV2_MIN 1
#define PLL_LAINT_POSTDIV2_MAX 7

#define PLL_LAINT_MAX_FREQ 2500000000

struct clk_laintpll_reg {
	void __iomem *freq_reg;
	void __iomem *pd_reg;
};

#define to_clk_laintpll(_hw) container_of(_hw, struct clk_laintpll, hw)

struct clk_laintpll {
	struct clk_hw hw;
	struct clk_laintpll_reg reg;
	unsigned int flags;
	unsigned int mode;
	unsigned int refdiv;
	unsigned int fbdiv;
	unsigned int postdiv1;
	unsigned int postdiv2;
	unsigned int round_rate;
	spinlock_t lock;
};

static unsigned long laintpll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_laintpll *clk;
	unsigned int fbdiv, refdiv, postdiv1, postdiv2;
	unsigned int bypass;
	unsigned int val;
	unsigned long rate;

	clk = to_clk_laintpll(hw);

	val = readl(clk->reg.pd_reg);
	bypass = (val & (1 << PLL_PD_CTRL_BYPASS_BIT)) >> PLL_PD_CTRL_BYPASS_BIT;
	if(bypass){
		pr_info("%s: laintpll bypass\n", __func__);
		return parent_rate;
	}

	val = readl(clk->reg.freq_reg);
	fbdiv = (val & (PLL_FREQ_CTRL_FBDIV_FIELD << PLL_FREQ_CTRL_FBDIV_BIT)) >> PLL_FREQ_CTRL_FBDIV_BIT;
	refdiv = (val & (PLL_FREQ_CTRL_REFDIV_FIELD << PLL_FREQ_CTRL_REFDIV_BIT)) >> PLL_FREQ_CTRL_REFDIV_BIT;
	postdiv1 = (val & (PLL_FREQ_CTRL_POSTDIV1_FIELD << PLL_FREQ_CTRL_POSTDIV1_BIT)) >> PLL_FREQ_CTRL_POSTDIV1_BIT;
	postdiv2 = (val & (PLL_FREQ_CTRL_POSTDIV2_FIELD << PLL_FREQ_CTRL_POSTDIV2_BIT)) >> PLL_FREQ_CTRL_POSTDIV2_BIT;

	rate = (parent_rate / refdiv) * fbdiv / postdiv1 /postdiv2;
	return rate;
}

static long laintpll_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *parent_rate)
{
	struct clk_laintpll *clk;
	unsigned int refdiv, fbdiv, postdiv1, postdiv2;
	unsigned int refdiv_pre, fbdiv_pre, postdiv1_pre, postdiv2_pre;
	unsigned int refdiv_post, fbdiv_post, postdiv1_post, postdiv2_post;
	unsigned int found = 0;
	unsigned long prate;
	unsigned long rate_pre, rate_post, rate_cur;

	clk = to_clk_laintpll(hw);

	prate = *parent_rate;
	if((rate < prate) | (rate > PLL_LAINT_MAX_FREQ)){
		pr_warn("%s: request frequency %ldHz not supported\n", __func__, rate);
		return -EINVAL;
	}

	if(rate == prate){
		pr_info("%s: request parent rate\n", __func__);
		clk->mode = PLL_BYPASS_MODE;
		return prate;
	}

	clk->mode = PLL_LAINT_MODE;
	/* loop here for general rounding the freq combination value here, for more efficient and power
	 * saving, lookup table can be used with limited combination, but usecase need to be confirmed
	 */
	rate_post = PLL_LAINT_MAX_FREQ;
	rate_pre = prate;
	for(refdiv = PLL_LAINT_REFDIV_MIN; refdiv <= PLL_LAINT_REFDIV_MAX; refdiv++){
		for(fbdiv = PLL_LAINT_FBDIV_MAX; fbdiv >= PLL_LAINT_FBDIV_MIN; fbdiv--){
			for(postdiv1 = PLL_LAINT_POSTDIV1_MAX; postdiv1 >= PLL_LAINT_POSTDIV1_MIN; postdiv1--){
				for(postdiv2 = PLL_LAINT_POSTDIV2_MIN; postdiv2 <= PLL_LAINT_POSTDIV2_MAX; postdiv2++){
					rate_cur = (prate / refdiv) * fbdiv / postdiv1 / postdiv2;
					if((rate_cur > rate) && (rate_cur < rate_post)){
						rate_post = rate_cur;
						refdiv_post = refdiv;
						fbdiv_post = fbdiv;
						postdiv2_post = postdiv2;
						postdiv1_post = postdiv1;
						continue;
					}
					if(rate_cur == rate){
						found = 1;
						break;
					}else if((rate_cur < rate) && (rate_cur > rate_pre)){
						rate_pre = rate_cur;
						refdiv_pre = refdiv;
						fbdiv_pre = fbdiv;
						postdiv2_pre = postdiv2;
						postdiv1_pre = postdiv1;
						break;
					}
				}
				if(1 == found)
					break;
			}
			if(1 == found)
				break;
		}
		if(1 == found)
			break;
	}

	if(found){
		clk->refdiv = refdiv;
		clk->fbdiv = fbdiv;
		clk->postdiv1 = postdiv1;
		clk->postdiv2 = postdiv2;
		pr_debug("%s: found exact match %lu\n", __func__, rate);
		return rate;
	}else{
		clk->refdiv = refdiv_pre;
		clk->fbdiv = fbdiv_pre;
		clk->postdiv1 = postdiv1_pre;
		clk->postdiv2 = postdiv2_pre;
		pr_debug("%s: found closest %lu\n", __func__, rate_pre);
		return rate_pre;
	}
}

static int laintpll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct clk_laintpll *clk;
	unsigned long new_rate;
	unsigned int val;

	clk = to_clk_laintpll(hw);
	switch (clk->mode){
		case PLL_BYPASS_MODE:
			new_rate = parent_rate;
			if(new_rate != rate){
				pr_warn("%s: failed with invalid round rate %lu, expecting %lu\n", __func__, rate, new_rate);
				return -EINVAL;
			}
			val = readl(clk->reg.pd_reg);
			val |= (1 << PLL_PD_CTRL_BYPASS_BIT);
			writel(val, clk->reg.pd_reg);
			break;
		case PLL_LAINT_MODE:
			new_rate = (parent_rate / clk->refdiv) * clk->fbdiv / clk->postdiv1 / clk->postdiv2;
			if(new_rate != rate){
				pr_warn("%s: failed with invalid round rate %lu, expecting %lu\n", __func__, rate, new_rate);
				return -EINVAL;
			}
			val = readl(clk->reg.pd_reg);
			val &= ~(1 << PLL_PD_CTRL_BYPASS_BIT);
			writel(val, clk->reg.pd_reg);

			val = (clk->refdiv & PLL_FREQ_CTRL_REFDIV_FIELD) << PLL_FREQ_CTRL_REFDIV_BIT;
			val |= ((clk->fbdiv & PLL_FREQ_CTRL_FBDIV_FIELD) << PLL_FREQ_CTRL_FBDIV_BIT);
			val |= ((clk->postdiv1 & PLL_FREQ_CTRL_POSTDIV1_FIELD) << PLL_FREQ_CTRL_POSTDIV1_BIT);
			val |= ((clk->postdiv2 & PLL_FREQ_CTRL_POSTDIV2_FIELD) << PLL_FREQ_CTRL_POSTDIV2_BIT);
			writel(val, clk->reg.freq_reg);
			break;
		default:
			pr_warn("%s: invalid clk mode\n", __func__);
			return -EINVAL;
	}
	return 0;
}

int laintpll_clk_enable(struct clk_hw *hw)
{
#if 0
	struct clk_laintpll *clk;
	unsigned int pd, foutpostdivpd;
	unsigned int val;
	unsigned long flags;

	clk = to_clk_laintpll(hw);

	val = readl(clk->reg.pd_reg);

	foutpostdivpd = (val & (1 << PLL_PD_CTRL_FOUTPOSTDIVPD_BIT)) >> PLL_PD_CTRL_FOUTPOSTDIVPD_BIT;
	pd = (val & (1 << PLL_PD_CTRL_PD_BIT)) >> PLL_PD_CTRL_PD_BIT;
	if ((pd == 0) | (foutpostdivpd == 0)){
		spin_lock_irqsave(&clk->lock, flags);
		val &= ~(1 << PLL_PD_CTRL_PD_BIT);
		writel(val, clk->reg.pd_reg);
		val = readl(clk->reg.pd_reg);
		val |= (1 << PLL_PD_CTRL_FOUTPOSTDIVPD_BIT);
		writel(val, clk->reg.pd_reg);
		val = readl(clk->reg.pd_reg);
		val |= (1 << PLL_PD_CTRL_PD_BIT);
		writel(val, clk->reg.pd_reg);
		spin_unlock_irqrestore(&clk->lock, flags);
	}
#endif
	return 0;
}

static void laintpll_clk_disable(struct clk_hw *hw)
{
#if 0
	struct clk_laintpll *clk;
	unsigned int pd;
	unsigned int val;

	clk = to_clk_laintpll(hw);

	val = readl(clk->reg.pd_reg);
	pd = (val & (1 << PLL_PD_CTRL_PD_BIT)) >> PLL_PD_CTRL_PD_BIT;
	if(pd){
		val &= ~(1 << PLL_PD_CTRL_PD_BIT);
		writel(val, clk->reg.pd_reg);
	}
#endif
	return;
}

const struct clk_ops laintpll_clk_ops = {
	.enable = &laintpll_clk_enable,
	.disable = &laintpll_clk_disable,
	.recalc_rate = &laintpll_recalc_rate,
	.round_rate = &laintpll_round_rate,
	.set_rate = &laintpll_set_rate,
};

static struct clk *laintpll_clk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags, struct clk_laintpll_reg *reg,
		unsigned int clk_pll_flags, const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct clk_laintpll *clk_hw;
	struct clk *clk;
	unsigned int ret;

	clk_hw = kzalloc(sizeof(*clk_hw), GFP_KERNEL);
	if(!clk_hw){
		pr_err("%s: failed to malloc pll clk for %s!\n", __func__, name);
		return NULL;
	}

	init.name = name;
	init.ops = ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	clk_hw->hw.init = &init;
	memcpy(&clk_hw->reg, reg, sizeof(*reg));
	clk_hw->flags = clk_pll_flags;

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

static void __init _of_x2_laintpll_clk_setup(struct device_node *node, const struct clk_ops *ops)
{
	struct clk *clk;
	struct clk_laintpll_reg reg;
	const char *parent_name;
	unsigned int flags = 0;
	unsigned int clk_laintpll_flags = 0;
	unsigned int data[2];
	void __iomem *reg_base;
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

	ret = of_property_read_u32_array(node, "offset", data, 2);
	if(ret){
		pr_err("%s: %s missing offset property", __func__, node->name);
		return;
	}
	reg.freq_reg = reg_base + data[0];
	reg.pd_reg = reg_base + data[1];

	clk = laintpll_clk_register(NULL, node->name, parent_name, flags, &reg, clk_laintpll_flags, ops);

	if(!IS_ERR(clk)){
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	}

	//pr_info("%s: %s pll clock set up.\n", __func__, node->name);
}

static void __init of_x2_laintpll_clk_setup(struct device_node *node)
{
	_of_x2_laintpll_clk_setup(node, &laintpll_clk_ops);
}
CLK_OF_DECLARE(x2_laintpll_clk, "x2,laintpll-clk", of_x2_laintpll_clk_setup);
