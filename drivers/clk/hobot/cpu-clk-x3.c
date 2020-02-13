#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>
#include <asm-generic/delay.h>

#include "common.h"

#define to_cpuclk(_hw) container_of(_hw, struct cpuclk, hw)

struct cpuclk {
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

static unsigned long cpuclk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct cpuclk *cpu_clk;
	unsigned long rate;

	cpu_clk = to_cpuclk(hw);
	rate = clk_get_rate(cpu_clk->cpu_div);

	return rate;
}

static long cpuclk_round_rate(struct clk_hw *hw,
				unsigned long rate, unsigned long *prate)
{
	/* return directly, use exact rate for cpu freq */;
	return rate;
}

struct cpu_pll_table {
	unsigned long cpu_freq;
	unsigned long pll_freq;
};

#define CPU_FREQ_NUM 6
struct cpu_pll_table pll_table[CPU_FREQ_NUM] = {
	{250000000,  2000000000},
	{500000000,  1500000000},
	{800000000,  1600000000},
	{1000000000, 2000000000},
	{1200000000, 2400000000},
	{1500000000, 1500000000},
};

static int __set_armpll_clk(struct clk_hw *hw, unsigned long cpu_freq)
{
	struct cpuclk *cpuclk;
	unsigned long pll_rate, old_pll_rate;
	int found = 0;
	int i, ret;

	cpuclk = to_cpuclk(hw);

	for (i = 0; i < CPU_FREQ_NUM; i++) {
		if (cpu_freq == pll_table[i].cpu_freq) {
			pll_rate = pll_table[i].pll_freq;
			found = 1;
			break;
		}
	}

	if (!found) {
		pr_err("Not found rate:%lu in table\n", cpu_freq);
		return -1;
	}

	old_pll_rate = clk_get_rate(cpuclk->armpll1);
	if (pll_rate != old_pll_rate) {

		/* enable and switch to pll2 */
		clk_prepare(cpuclk->armpll2);
		clk_enable(cpuclk->armpll2);

		/* switch to armpll2 */
		ret = clk_set_parent(cpuclk->armpll_mux, cpuclk->armpll2);
		if (ret)
			return ret;

		/* change PLL */
		clk_prepare(cpuclk->armpll1);
		clk_enable(cpuclk->armpll1);
		clk_disable(cpuclk->armpll1);

		ret = clk_set_rate(cpuclk->armpll1, pll_rate);
		if (ret) {
			clk_set_parent(cpuclk->cpu_mux, cpuclk->cpu_div);
			return ret;
		}
		clk_prepare(cpuclk->armpll1);
		clk_enable(cpuclk->armpll1);

		/* switch back to armpll1 */
		ret = clk_set_parent(cpuclk->armpll_mux, cpuclk->armpll1);
		if (ret)
			return ret;

		clk_disable(cpuclk->armpll2);
	}

	return 0;
}

static int cpuclk_set_rate(struct clk_hw *hw,
		unsigned long rate, unsigned long parent_rate)
{
	struct cpuclk *cpu_clk;
	int ret;
	static int recursive_flag;

	/*
	 * clk_set_rate may re-enter this function since cpuclk is children of
	 * cpu_div return if recursive call is detected to prevent deadloop
	 */
	if (recursive_flag == 1) {
		recursive_flag = 0;
		return 0;
	}

	cpu_clk = to_cpuclk(hw);
	recursive_flag = 1;
	ret = __set_armpll_clk(hw, rate);
	clk_set_rate(cpu_clk->cpu_div, rate);
	recursive_flag = 0;

	return ret;
}

const struct clk_ops cpuclk_ops = {
	.recalc_rate = cpuclk_recalc_rate,
	.round_rate = cpuclk_round_rate,
	.set_rate = cpuclk_set_rate,
};

static struct clk *cpuclk_register(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct cpuclk *cpu_clk;
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

static void __init _of_hobot_cpuclk_setup(struct device_node *node,
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

	clk = cpuclk_register(NULL, node->name, parent_name, flags, ops);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

}

static void __init of_hobot_cpuclk_setup(struct device_node *node)
{
	_of_hobot_cpuclk_setup(node, &cpuclk_ops);
}
CLK_OF_DECLARE(hobot_x3_cpuclk, "hobot,cpu-clk-x3", of_hobot_cpuclk_setup);
