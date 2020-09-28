/*
 * dmc-clk.c --- DDR controller clock.
 *
 * Copyright (C) 2020, Schspa, all rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>
#include <linux/arm-smccc.h>
#include <linux/moduleparam.h>
#include <linux/smp.h>
#include <asm-generic/delay.h>
#include <linux/delay.h>

#include "common.h"

#define to_dmc_clk(_hw) container_of(_hw, struct dmc_clk, hw)

/*
 * [FIXME] cur_ddr_rate is used to store current ddr freq before cr5
 * function finished
 */
static unsigned long cur_ddr_rate;
#define DDR_METHOD_LEN 32
static char ddr_method[DDR_METHOD_LEN] = "test";

typedef void(hobot_invoke_fn)(unsigned long, unsigned long, unsigned long,
			unsigned long, unsigned long, unsigned long,
			unsigned long, unsigned long,
			struct arm_smccc_res *);

struct dmc_clk {
	struct clk_hw hw;
	unsigned int flags;
	void __iomem *clk_base;
	spinlock_t lock;
	unsigned long rate;
	hobot_invoke_fn *invoke_fn;
	uint32_t fid;
	uint32_t get_channels_cmd;
	uint32_t set_cmd;
	uint32_t get_cmd;
	uint32_t channel;
	struct clk *ddr_mclk;
	atomic_t vote_cnt;
};

struct dmc_clk *g_ddrclk;

/**
 * hobot_smccc_smc() - Method to call firmware via SMC.
 * @a0: Argument passed to Secure EL3.
 * @a1: Argument passed to Secure EL3.
 * @a2: Argument passed to Secure EL3.
 * @a3: Argument passed to Secure EL3.
 * @a4: Argument passed to Secure EL3.
 * @a5: Argument passed to Secure EL3.
 * @a6: Argument passed to Secure EL3.
 * @a7: Argument passed to Secure EL3.
 * @res: return code stored in.
 *
 * This function call arm_smccc_smc directly.
 *
 * Return: return value stored in res argument.
 */
static void hobot_smccc_smc(unsigned long a0, unsigned long a1,
			unsigned long a2, unsigned long a3,
			unsigned long a4, unsigned long a5,
			unsigned long a6, unsigned long a7,
			struct arm_smccc_res *res)
{
	arm_smccc_smc(a0, a1, a2, a3, a4, a5, a6, a7, res);
}

/**
 * hobot_smccc_hvc() - Method to call firmware via HVC.
 * @a0: Arguments passed to firmware.
 * @a1: Arguments passed to firmware.
 * @a2: Arguments passed to firmware.
 * @a3: Arguments passed to firmware.
 * @a4: Arguments passed to firmware.
 * @a5: Arguments passed to firmware.
 * @a6: Arguments passed to firmware.
 * @a7: Arguments passed to firmware.
 * @res: return code stored in.
 *
 * This function call arm_smccc_hvc directly.
 *
 * Return: return value stored in res argument.
 */
static void hobot_smccc_hvc(unsigned long a0, unsigned long a1,
			unsigned long a2, unsigned long a3,
			unsigned long a4, unsigned long a5,
			unsigned long a6, unsigned long a7,
			struct arm_smccc_res *res)
{
	arm_smccc_hvc(a0, a1, a2, a3, a4, a5, a6, a7, res);
}

static void hobot_ddrdfc_test(unsigned long a0, unsigned long a1,
			unsigned long a2, unsigned long rate,
			unsigned long a4, unsigned long a5,
			unsigned long a6, unsigned long a7,
			struct arm_smccc_res *res)
{

	int i;
	pr_debug("%s rate:%ld\n", __func__, rate);
	for (i = 0; i < 1000; i++)
		udelay(100);

	// just for debugging
	res->a0 = 0;
}

typedef void (*CR5_SEND_IPI_HANDLE_T)(void);
CR5_SEND_IPI_HANDLE_T send_ipi_cr5;
void register_cr5_send_ipi_func(CR5_SEND_IPI_HANDLE_T func)
{
        send_ipi_cr5 = func;
}
EXPORT_SYMBOL(register_cr5_send_ipi_func);

#define CR5_DDR_SERVICE_START_FLAG 0xA5
typedef void *(*CR5_GET_BASE_HANDLE_T)(void);
CR5_GET_BASE_HANDLE_T cr5_get_msginfo_base;
void register_cr5_get_msginfo_base(CR5_GET_BASE_HANDLE_T func)
{
        cr5_get_msginfo_base = func;
}
EXPORT_SYMBOL(register_cr5_get_msginfo_base);

static void hobot_smccc_cr5(unsigned long a0, unsigned long a1,
                        unsigned long a2, unsigned long rate,
                        unsigned long a4, unsigned long a5,
                        unsigned long a6, unsigned long a7,
                        struct arm_smccc_res *res)
{
	char index = 0;
	void *sram_vaddr = NULL;
	volatile char *p_index = NULL;
	volatile char *p_status = NULL;
	volatile char *p_err = NULL;
	volatile char *p_serive_started = NULL;

	pr_debug("%s rate:%ld\n", __func__, rate);

	if (cr5_get_msginfo_base == NULL) {
		pr_debug("hobot_smccc_cr5, CR5 not work, keep rate:%ld\n", rate);

		/* keep devfreq working before cr5 is ready */
		cur_ddr_rate = rate;
		res->a0 = 0;

		return;
	}

	sram_vaddr = cr5_get_msginfo_base();

	if (sram_vaddr == NULL) {
		res->a0 = -ENODEV;
		pr_debug("hobot_smccc_cr5, CR5 msg vaddr is not valid\n");
		return;
	}

	p_index = sram_vaddr;
	p_status = sram_vaddr + 1;
	p_err = sram_vaddr + 2;
	p_serive_started = sram_vaddr + 3;

	if (rate == 667000000)
		index = 0;
	else if (rate == 2666000000)
		index = 1;
	else if (rate == 3200000000)
		index = 2;
	else if (rate == 0) {
		/* most case reclac rate is 0
		 * we dont case, return OK
		 */
		res->a0 = 0;
		return;
	} else {
		res->a0 = -EINVAL;
		return;
	}

	*p_index = index;
	*p_status = 0;
	*p_serive_started = CR5_DDR_SERVICE_START_FLAG;
#if 0
	udelay(10);
	send_ipi_cr5();
#endif
	*p_serive_started = CR5_DDR_SERVICE_START_FLAG;
	while(!*p_status) {
	}
	res->a0 = *p_err;
}



static hobot_invoke_fn *get_invoke_func(struct device_node *np)
{

	const char *method;

	if (of_property_read_string(np, "method", &method)) {
		pr_info("no \"method\" property, use test method.\n");
		/* use test with no method in dts */
		method = ddr_method;
	}

	strncpy(ddr_method, method, sizeof(ddr_method));

	if (!strcmp("hvc", method))
		return hobot_smccc_hvc;
	else if (!strcmp("smc", method))
		return hobot_smccc_smc;
	else if (!strcmp("test", method))
		return hobot_ddrdfc_test;
	else if (!strcmp("cr5", method))
		return hobot_smccc_cr5;

	pr_warn("invalid \"method\" property: %s\n", method);
	strncpy(ddr_method, "N/A", sizeof(ddr_method));
	return ERR_PTR(-EINVAL);
}

int dmc_clk_set_method(char *method)
{
	if (g_ddrclk == NULL || method == NULL) {
		pr_err("invalid params: gddrclk:%p, method:%p\n",
			g_ddrclk, method);
		return -EINVAL;
	}

	spin_lock(&g_ddrclk->lock);
	if (!strcmp("hvc", method)) {
		g_ddrclk->invoke_fn = hobot_smccc_hvc;
	} else if (!strcmp("smc", method)) {
		g_ddrclk->invoke_fn = hobot_smccc_smc;
	} else if (!strcmp("test", method)) {
		g_ddrclk->invoke_fn = hobot_ddrdfc_test;
	} else if (!strcmp("cr5", method)) {
		g_ddrclk->invoke_fn = hobot_smccc_cr5;
	} else {
		pr_err("method: %s is not supported\n", method);
		spin_unlock(&g_ddrclk->lock);
		return -EINVAL;
	}

	strncpy(ddr_method, method, sizeof(ddr_method));
	spin_unlock(&g_ddrclk->lock);


	return 0;
}

char *dmc_clk_get_method(void)
{
	pr_debug("cur method:%s: func: %pf\n",
		ddr_method, g_ddrclk->invoke_fn);
	return ddr_method;
}


static void spin_on_cpu(void *info)
{
	pr_debug("%s %d enter\n", __func__, smp_processor_id());

	/* tell the clock changing core this core is parked */
	atomic_dec(&g_ddrclk->vote_cnt);
	smp_mb();

	/* this core will spin on the lock before ddr freq
	 * changing finished
	 */
	spin_lock(&g_ddrclk->lock);


	spin_unlock(&g_ddrclk->lock);
	pr_debug("%s %d exit\n", __func__, smp_processor_id());
}

static void park_other_cpus(void)
{
    int cpu;
	struct cpumask mask;

    cpu = get_cpu();

    memcpy(&mask, cpu_online_mask, sizeof(struct cpumask));
    cpumask_clear_cpu(cpu, &mask);

    for_each_cpu(cpu, &mask)
		atomic_inc(&g_ddrclk->vote_cnt);

	smp_mb();

	on_each_cpu_mask(&mask, spin_on_cpu, NULL, 0);

	put_cpu();
}


static int dmc_set_rate(struct clk_hw *hw,
			unsigned long rate, unsigned long parent_rate)
{
	struct dmc_clk *dclk = container_of(hw, struct dmc_clk, hw);
	struct arm_smccc_res res;
	unsigned long flags;
	int timeout = 1000000;

	spin_lock(&dclk->lock);

	atomic_set(&dclk->vote_cnt, 0);
	park_other_cpus();
	local_irq_save(flags);

	while(atomic_read(&dclk->vote_cnt) > 0 && timeout > 0) {
		udelay(100);
		timeout -= 100;
	}

	if (timeout <= 0) {
		pr_err("timeout waiting for ipi voting\n");
		local_irq_restore(flags);
		spin_unlock(&dclk->lock);
		return -EBUSY;
	}

#ifndef CONFIG_HOBOT_XJ3
	dclk->invoke_fn(dclk->fid, dclk->set_cmd, dclk->channel, rate, 0, 0, 0, 0, &res);
#else
	dclk->invoke_fn(0, 0, 0, rate, 0, 0, 0, 0, &res);
#endif
	if (res.a0 != 0) {
		pr_err("%s: ddr channel:%u set rate to %lu failed with status :%ld\n",
			__func__, dclk->channel, rate, res.a0);
		spin_unlock(&dclk->lock);
		return -EINVAL;
	}
	dclk->rate = rate;
	cur_ddr_rate = rate;

	local_irq_restore(flags);
	spin_unlock(&dclk->lock);

	return 0;
}

unsigned long hobot_dmc_cur_rate(void)
{
	return cur_ddr_rate;
}
EXPORT_SYMBOL(hobot_dmc_cur_rate);

static unsigned long dmc_recalc_rate(struct clk_hw *hw,
				unsigned long parent_rate)
{
	struct dmc_clk *dclk = container_of(hw, struct dmc_clk, hw);
#ifndef CONFIG_HOBOT_XJ3
	struct arm_smccc_res res;

	dclk->invoke_fn(dclk->fid, dclk->get_cmd, dclk->channel, 0, 0, 0, 0, 0, &res);
	dclk->invoke_fn(0, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != 0) {
		pr_err("%s: ddr channel:%u get rate failed with status :%ld\n",
			__func__, dclk->channel, res.a0);
	}

	dclk->rate = res.a1;
#endif

	/* FIXME: find a better way to get current ddr freq */
	pr_debug("parent_rate: %ld,  dclk->rate:%ld, cur_ddr_rate:%ld\n",
		parent_rate, dclk->rate, cur_ddr_rate);

	return dclk->rate;
}

static long dmc_round_rate(struct clk_hw *hw,
			unsigned long rate,
			unsigned long *prate)
{
	/* TODO: round rate with frequency table. */
	pr_debug("dmc_round_rate: rate:%ld\n", rate);
	return rate;
}

const struct clk_ops dmc_clk_ops = {
	.recalc_rate = dmc_recalc_rate,
	.set_rate = dmc_set_rate,
	.round_rate = dmc_round_rate,
};

static struct clk *dmc_clk_register(struct device *dev, struct device_node *node,
				unsigned long flags,
				const struct clk_ops *ops)
{
	struct clk_init_data init = {NULL};
	struct dmc_clk *ddrclk;
	struct clk *clk;
	int ret;
	hobot_invoke_fn *invoke_fn;

	ddrclk = kzalloc(sizeof(*ddrclk), GFP_KERNEL);
	if (!ddrclk)
		return NULL;

	g_ddrclk = ddrclk;

#ifdef CONFIG_HOBOT_XJ3
	invoke_fn = get_invoke_func(node);
#else
	uint32_t channel = 0;
	struct device_node *tee_node;
	tee_node = of_parse_phandle(node, "tee-dram-handle", 0);
	invoke_fn = get_invoke_func(tee_node);
#endif

	if (IS_ERR(invoke_fn)) {
		ret = PTR_ERR(invoke_fn);
		goto err_free;
	}
	ddrclk->invoke_fn = invoke_fn;

#ifndef CONFIG_HOBOT_XJ3
	struct arm_smccc_res res;
#define GET_OFNODE_U32(node, name, dst)					\
	do {								\
		ret = of_property_read_u32(node, name, dst);		\
		if (ret) {						\
			pr_err("%s: failed to get " name "\n", __func__); \
			ret = -EINVAL;					\
			goto err_free;					\
		}							\
	} while (0)

	GET_OFNODE_U32(tee_node, "fid", &ddrclk->fid);
	GET_OFNODE_U32(tee_node, "get_channels_cmd", &ddrclk->get_channels_cmd);
	GET_OFNODE_U32(tee_node, "set_cmd", &ddrclk->set_cmd);
	GET_OFNODE_U32(tee_node, "get_cmd", &ddrclk->get_cmd);

	of_node_put(tee_node);

	ret = of_property_read_u32(node, "channel-id", &channel);
	if (ret) {
		pr_err("%s: can't find hank-id property\n", __func__);
		goto err_free;
	}

	ddrclk->invoke_fn(ddrclk->fid, ddrclk->get_channels_cmd, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0 != 0) {
		pr_err("failed to get total channels from tee world.");
		goto err_free;
	}

	if (res.a1 <= channel) {
		pr_err("ddr channel %u exceed %lu\n", channel, res.a1);
		goto err_free;
	}

#endif

	init.name = node->full_name;
	init.ops = ops;
	init.flags = flags;

	ddrclk->hw.init = &init;

	spin_lock_init(&ddrclk->lock);
	clk = clk_register(dev, &ddrclk->hw);
	if (IS_ERR(clk)) {
		pr_err("Failed to register clock for %s!\n", node->full_name);
		kfree(ddrclk);
		return NULL;
	}

	return clk;

err_free:
	kfree(ddrclk);
	g_ddrclk = NULL;
	return NULL;
}

static void __init _of_hobot_dmc_clk_setup(struct device_node *node,
					const struct clk_ops *ops)
{
	struct clk *clk;
	unsigned int flags = 0;

	clk = dmc_clk_register(NULL, node, flags, ops);

	if (!IS_ERR_OR_NULL(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

static void __init of_hobot_dmc_clk_setup(struct device_node *node)
{
	_of_hobot_dmc_clk_setup(node, &dmc_clk_ops);
}
CLK_OF_DECLARE(hobot_dmc_clk, "hobot,hobot-dmc-clk", of_hobot_dmc_clk_setup);
