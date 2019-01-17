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
#define PLL_PD_CTRL_DACPD_BIT 5
#define PLL_PD_CTRL_FOUTPOSTDIVPD_BIT 8
#define PLL_PD_CTRL_FOUTVCOPD_BIT 12
#define PLL_PD_CTRL_FOUT4OHASEPD_BIT 13
#define PLL_PD_CTRL_BYPASS_BIT 16

#define PLL_FRAC_BIT 0
#define PLL_FRAC_FIELD 0xFFFFFF

#define PLL_BYPASS_MODE 0x1
#define PLL_FRAC_MODE 0x0

#define PLL_FRAC_REFDIV_MIN 1
#define PLL_FRAC_REFDIV_MAX 63
#define PLL_FRAC_FBDIV_MIN 20
#define PLL_FRAC_FBDIV_MAX 320
#define PLL_FRAC_FBDIV_INT_MIN 16
#define PLL_FRAC_FBDIV_INT_MAX 3200
#define PLL_FRAC_POSTDIV1_MIN 1
#define PLL_FRAC_POSTDIV1_MAX 7
#define PLL_FRAC_POSTDIV2_MIN 1
#define PLL_FRAC_POSTDIV2_MAX 7

#define PLL_FRAC_MAX_FREQ 3200000000

struct clk_fracpll_reg {
	void __iomem *freq_reg;
	void __iomem *pd_reg;
	void __iomem *frac_reg;
};

#define to_clk_fracpll(_hw) container_of(_hw, struct clk_fracpll, hw)

struct clk_fracpll {
	struct clk_hw hw;
	struct clk_fracpll_reg reg;
	unsigned int flags;
	unsigned int mode;
	unsigned int refdiv;
	unsigned int fbdiv;
	unsigned int postdiv1;
	unsigned int postdiv2;
	unsigned int frac;
	spinlock_t lock;
};

int fracpll_clk_enable(struct clk_hw *hw)
{
	struct clk_fracpll *clk;
	unsigned int pd, foutpostdivpd;
	unsigned int val;
	unsigned long flags;

	clk = to_clk_fracpll(hw);

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

	return 0;
}

static void fracpll_clk_disable(struct clk_hw *hw)
{
	struct clk_fracpll *clk;
	unsigned int pd;
	unsigned int val;

	clk = to_clk_fracpll(hw);

	val = readl(clk->reg.pd_reg);
	pd = (val & (1 << PLL_PD_CTRL_PD_BIT)) >> PLL_PD_CTRL_PD_BIT;
	if(pd){
		val &= ~(1 << PLL_PD_CTRL_PD_BIT);
		writel(val, clk->reg.pd_reg);
	}
	return;
}

static unsigned long fracpll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_fracpll *clk;
	unsigned int fbdiv, refdiv, postdiv1, postdiv2;
	unsigned int dsmpd, bypass;
	unsigned int val, frac;
	unsigned long rate;

	clk = to_clk_fracpll(hw);

	val = readl(clk->reg.pd_reg);
	bypass = (val & (1 << PLL_PD_CTRL_BYPASS_BIT)) >> PLL_PD_CTRL_BYPASS_BIT;
	if(bypass){
		pr_info("%s: fracpll bypass\n", __func__);
		return parent_rate;
	}

	dsmpd = (val & (1 << PLL_PD_CTRL_DSMPD_BIT)) >> PLL_PD_CTRL_DSMPD_BIT;

	val = readl(clk->reg.freq_reg);
	fbdiv = (val & (PLL_FREQ_CTRL_FBDIV_FIELD << PLL_FREQ_CTRL_FBDIV_BIT)) >> PLL_FREQ_CTRL_FBDIV_BIT;
	refdiv = (val & (PLL_FREQ_CTRL_REFDIV_FIELD << PLL_FREQ_CTRL_REFDIV_BIT)) >> PLL_FREQ_CTRL_REFDIV_BIT;
	postdiv1 = (val & (PLL_FREQ_CTRL_POSTDIV1_FIELD << PLL_FREQ_CTRL_POSTDIV1_BIT)) >> PLL_FREQ_CTRL_POSTDIV1_BIT;
	postdiv2 = (val & (PLL_FREQ_CTRL_POSTDIV2_FIELD << PLL_FREQ_CTRL_POSTDIV2_BIT)) >> PLL_FREQ_CTRL_POSTDIV2_BIT;

	if(dsmpd){
		rate = (parent_rate / refdiv) * fbdiv / postdiv1 /postdiv2;
	}else{
		val = readl(clk->reg.frac_reg);
		frac = (val & (PLL_FRAC_FIELD) << PLL_FRAC_BIT) >> PLL_FRAC_BIT;
		rate = (parent_rate / refdiv) * (fbdiv + frac / 0x1000000) / postdiv1 / postdiv2;
	}
	return rate;
}

static long fracpll_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *parent_rate)
{
	struct clk_fracpll *clk;
	unsigned int refdiv, fbdiv, postdiv1, postdiv2;
	unsigned int refdiv_pre, fbdiv_pre, postdiv1_pre, postdiv2_pre;
	unsigned int refdiv_post, fbdiv_post, postdiv1_post, postdiv2_post;
	unsigned int found = 0;
	unsigned long prate;
	unsigned long rate_pre, rate_post, rate_cur;
	unsigned int val, dsmpd;

	clk = to_clk_fracpll(hw);

	prate = *parent_rate;
	if((rate < prate) | (rate > PLL_FRAC_MAX_FREQ)){
		pr_warn("%s: request frequency %ldHz not supported\n", __func__, rate);
		return -EINVAL;
	}

	if(rate == prate){
		pr_info("%s: request parent rate\n", __func__);
		clk->mode = PLL_BYPASS_MODE;
		return prate;
	}

	clk->mode = PLL_FRAC_MODE;

	val = readl(clk->reg.pd_reg);
	dsmpd = (val & (1 << PLL_PD_CTRL_DSMPD_BIT)) >> PLL_PD_CTRL_DSMPD_BIT;
	if (!dsmpd){
		pr_warn("%s: fractional mode not support, should always be integer mode!\n", __func__);
		return -EINVAL;
	}

	/* loop here for general rounding the freq combination value here, for more efficient and power
	 * saving, lookup table can be used with limited combination, but usecase need to be confirmed
	 */
	rate_post = PLL_FRAC_MAX_FREQ;
	rate_pre = prate;
	for(refdiv = PLL_FRAC_REFDIV_MIN; refdiv <= PLL_FRAC_REFDIV_MAX; refdiv++){
		for(fbdiv = PLL_FRAC_FBDIV_INT_MAX; fbdiv >= PLL_FRAC_FBDIV_INT_MIN; fbdiv--){
			for(postdiv1 = PLL_FRAC_POSTDIV1_MAX; postdiv1 >= PLL_FRAC_POSTDIV1_MIN; postdiv1--){
				for(postdiv2 = PLL_FRAC_POSTDIV2_MIN; postdiv2 <= PLL_FRAC_POSTDIV2_MAX; postdiv2++){
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

static int fracpll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct clk_fracpll *clk;
	unsigned long new_rate;
	unsigned int val, dsmpd;

	clk = to_clk_fracpll(hw);
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
		case PLL_FRAC_MODE:
			val = readl(clk->reg.pd_reg);
			dsmpd = (val & (1 << PLL_PD_CTRL_DSMPD_BIT)) >> PLL_PD_CTRL_DSMPD_BIT;
			if(!dsmpd){
				pr_warn("%s: fractional mode not supported, should always be integer mode!\n", __func__);
				return -EINVAL;
			}

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

const struct clk_ops fracpll_clk_ops = {
	.enable = &fracpll_clk_enable,
	.disable = &fracpll_clk_disable,
	.recalc_rate = &fracpll_recalc_rate,
	.round_rate = &fracpll_round_rate,
	.set_rate = &fracpll_set_rate,
};

static struct clk *fracpll_clk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags, struct clk_fracpll_reg *reg,
		unsigned int clk_pll_flags, const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct clk_fracpll *clk_hw;
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

static void __init _of_x2_fracpll_clk_setup(struct device_node *node, const struct clk_ops *ops)
{
	struct clk *clk;
	struct clk_fracpll_reg reg;
	const char *parent_name;
	unsigned int flags = 0;
	unsigned int clk_fracpll_flags = 0;
	unsigned int data[3];
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

	ret = of_property_read_u32_array(node, "offset", data, 3);
	if(ret){
		pr_err("%s: %s missing offset property", __func__, node->name);
		return;
	}
	reg.freq_reg = reg_base + data[0];
	reg.pd_reg = reg_base + data[1];
	reg.frac_reg = reg_base + data[2];

	clk = fracpll_clk_register(NULL, node->name, parent_name, flags, &reg, clk_fracpll_flags, ops);

	if(!IS_ERR(clk)){
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	}

	//pr_info("%s: %s fracpll clock set up.\n", __func__, node->name);
}

static void __init of_x2_fracpll_clk_setup(struct device_node *node)
{
	_of_x2_fracpll_clk_setup(node, &fracpll_clk_ops);
}
CLK_OF_DECLARE(x2_fracpll_clk, "x2,fracpll-clk", of_x2_fracpll_clk_setup);
