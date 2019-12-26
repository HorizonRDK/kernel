#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>
#include <asm-generic/delay.h>

#include "common.h"

#define to_armcpuclk(_hw) container_of(_hw, struct armcpuclk, hw)

struct armcpuclk {
	struct clk_hw hw;
	struct clk *armpll1;
	struct clk *armpll2;
	struct clk *armpll_mux;
	struct clk *cpu_div;
	struct clk *cpu_mux;
	struct clk *cpu_osc;
	unsigned int flags;
	void __iomem *clk_base;
	spinlock_t lock;
};

static unsigned long armcpuclk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct armcpuclk *cpu_clk;
	unsigned long rate;

	cpu_clk = to_armcpuclk(hw);
	rate = clk_get_rate(cpu_clk->cpu_div);

	return rate;
}


static long armcpuclk_round_rate(struct clk_hw *hw,
				unsigned long rate, unsigned long *prate)
{
	/* return directly, use exact rate for cpu freq */;
	return rate;
}

struct cpu_armpll_table {
	unsigned long cpu_freq;
	unsigned long pll_freq;
};

#define CPU_FREQ_NUM 5
struct cpu_armpll_table armpll_table[CPU_FREQ_NUM] = {
	{250000000,  2000000000},
	{500000000,  2000000000},
	{800000000,  1600000000},
	{1000000000, 2000000000},
	{1200000000, 2400000000},
};

static int __set_armpll_clk(struct clk_hw *hw, unsigned long cpu_freq)
{
	struct armcpuclk *armcpuclk;
	unsigned long pll_rate, old_pll_rate;
	int found = 0;
	int i, ret;

	armcpuclk = to_armcpuclk(hw);

	for (i = 0; i < CPU_FREQ_NUM; i++) {
		if (cpu_freq == armpll_table[i].cpu_freq) {
			pll_rate = armpll_table[i].pll_freq;
			found = 1;
			break;
		}
	}

	if (!found) {
		pr_err("Not found rate:%lu in table\n", cpu_freq);
		return -1;
	}

	old_pll_rate = clk_get_rate(armcpuclk->armpll1);
	if (pll_rate != old_pll_rate) {

		/* enable and switch to pll2 */
		clk_set_rate(armcpuclk->armpll2, old_pll_rate);
		clk_prepare(armcpuclk->armpll2);
		clk_enable(armcpuclk->armpll2);

		/* switch to armpll2 */
		ret = clk_set_parent(armcpuclk->armpll_mux, armcpuclk->armpll2);
		if (ret)
			return ret;

		/* change PLL */
		clk_prepare(armcpuclk->armpll1);
		clk_enable(armcpuclk->armpll1);
		clk_disable(armcpuclk->armpll1);

		ret = clk_set_rate(armcpuclk->armpll1, pll_rate);
		if (ret) {
			clk_set_parent(armcpuclk->cpu_mux, armcpuclk->cpu_div);
			return ret;
		}
		clk_prepare(armcpuclk->armpll1);
		clk_enable(armcpuclk->armpll1);

		/* switch back to armpll1 */
		ret = clk_set_parent(armcpuclk->armpll_mux, armcpuclk->armpll1);
		if (ret)
			return ret;

		clk_disable(armcpuclk->armpll2);
	}

	return 0;
}

static int armcpuclk_set_rate(struct clk_hw *hw,
		unsigned long rate, unsigned long parent_rate)
{
	struct armcpuclk *cpu_clk;
	int ret;
	static int recursive_flag;

	/*
	 * clk_set_rate may re-enter this function since armcpuclk is children of
	 * cpu_div return if recursive call is detected to prevent deadloop
	 */
	if (recursive_flag == 1) {
		recursive_flag = 0;
		return 0;
	}

	cpu_clk = to_armcpuclk(hw);
	recursive_flag = 1;
	ret = __set_armpll_clk(hw, rate);
	clk_set_rate(cpu_clk->cpu_div, rate);
	recursive_flag = 0;

	return ret;
}

const struct clk_ops armcpuclk_ops = {
	.recalc_rate = armcpuclk_recalc_rate,
	.round_rate = armcpuclk_round_rate,
	.set_rate = armcpuclk_set_rate,
};

static struct clk *armcpuclk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct armcpuclk *cpu_clk;
	struct clk *clk;
	unsigned int ret;

	cpu_clk = kzalloc(sizeof(*cpu_clk), GFP_KERNEL);
	if (!cpu_clk)
		return NULL;

	init.name = name;
	init.ops = ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	cpu_clk->hw.init = &init;

	cpu_clk->armpll1 = clk_get(NULL, "armpll1_clk");
	cpu_clk->armpll2 = clk_get(NULL, "armpll2_clk");
	cpu_clk->armpll_mux = clk_get(NULL, "arm_pllmux_clk");
	cpu_clk->cpu_div = clk_get(NULL, "cpu_divclk");
	cpu_clk->cpu_mux = clk_get(NULL, "cpu_mux_clk");
	cpu_clk->cpu_osc = clk_get(NULL, "osc_clk");

	spin_lock_init(&cpu_clk->lock);
	clk = clk_register(dev, &cpu_clk->hw);
	if (IS_ERR(clk)) {
		pr_err("Failed to register clock for %s!\n", name);
		kfree(cpu_clk);
		return NULL;
	}

	ret = clk_hw_register_clkdev(&cpu_clk->hw, name, NULL);
	if (ret) {
		pr_err("Failed to register lookup %s!\n", name);
		clk_unregister(clk);
		kfree(cpu_clk);
		return NULL;
	}

	return clk;
}

static void __init _of_hobot_armcpuclk_setup(struct device_node *node,
				const struct clk_ops *ops)
{
	struct clk *clk;
	const char *parent_name;
	unsigned int flags = 0;

	if (of_clk_get_parent_count(node) != 1) {
		pr_err("%s must have 1 parent\n", node->name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	clk = armcpuclk_register(NULL, node->name, parent_name, flags, ops);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

}

static void __init of_hobot_armcpuclk_setup(struct device_node *node)
{
	_of_hobot_armcpuclk_setup(node, &armcpuclk_ops);
}
CLK_OF_DECLARE(hobot_x2_armcpuclk, "hobot,cpu-clk-x2",
		of_hobot_armcpuclk_setup);
