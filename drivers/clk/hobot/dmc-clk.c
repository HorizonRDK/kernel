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
#include <linux/device.h>
#include <linux/devfreq.h>
#include <soc/hobot/hobot_bus.h>

#include "common.h"

#define DDR_METHOD_LEN 32
#define DMC_FID        0x82000008
#define DRAM_SET_RATE        0x10
#define DRAM_GET_RATE        0x11
#define DDR_INDEX_P0   0
#define DDR_INDEX_P1   1
#define DDR_INDEX_P2    2
#define DDR_FREQ_3200   3200000000
#define DDR_FREQ_2666   2666000000
#define DDR_FREQ_1333   1333000000
#define DDR_FREQ_667    667000000
#define DDR_FREQ_333    333000000

#define to_dmc_clk(_hw) container_of(_hw, struct dmc_clk, hw)

/*
 * [FIXME] cur_ddr_rate is used to store current ddr freq before cr5
 * function finished
 */
static unsigned long cur_ddr_rate;
static char ddr_method[DDR_METHOD_LEN] = "test";
static DEFINE_PER_CPU(call_single_data_t, dfs_csd);

typedef void(hobot_invoke_fn)(unsigned long, unsigned long, unsigned long,
			unsigned long, unsigned long, unsigned long,
			unsigned long, unsigned long,
			struct arm_smccc_res *);

struct dmc_clk {
	struct clk_hw hw;
	unsigned int flags;
	void __iomem *clk_base;
	spinlock_t lock;
	raw_spinlock_t raw_lock;
	unsigned long rate;
	hobot_invoke_fn *invoke_fn;
	uint32_t fid;
	uint32_t get_channels_cmd;
	uint32_t set_cmd;
	uint32_t get_cmd;
	uint32_t channel;
	struct clk *ddr_mclk;
	atomic_t vote_cnt;
	struct mutex mlock;
};

struct dmc_clk *g_ddrclk;

void dmc_lock(void)
{
	mutex_lock(&g_ddrclk->mlock);
}
EXPORT_SYMBOL(dmc_lock);

void dmc_unlock(void)
{
	mutex_unlock(&g_ddrclk->mlock);
}
EXPORT_SYMBOL(dmc_unlock);

static LIST_HEAD(dfs_list);
static LIST_HEAD(dfs_lock_list);
static DEFINE_MUTEX(dfs_list_mtx);

static inline struct hobot_dpm *to_hobot_dpm(struct list_head *node)
{
	return container_of(node, struct hobot_dpm, entry);
}

static int hobot_dfs_notifier(struct list_head *from_list,
		struct list_head *to_list, unsigned long val, int state)
{
	dpm_fn_t callback = NULL;
	struct hobot_dpm *dpm = NULL;
	int error = 0;

	while (!list_empty(from_list)) {
		dpm = to_hobot_dpm(from_list->next);

		get_device(dpm->dev);

		callback = dpm->dpm_call;
		if (callback) {
			error = callback(dpm, val, state);
			if (error) {
				pr_debug("Device %s return failed\n", dev_name(dpm->dev));
				break;
			}
		}

		if (!list_empty(&dpm->entry))
			list_move(&dpm->entry, to_list);

		put_device(dpm->dev);
	}

	return error;
}

void hobot_dfs_register(struct hobot_dpm *n, struct device *dev)
{
	struct hobot_dpm *dpm;

	mutex_lock(&dfs_list_mtx);

	list_for_each_entry(dpm, &dfs_list, entry) {
		if (n->priority <= dpm->priority) {
			list_add_tail(&n->entry, &dpm->entry);
			break;
		}
	}

	if (&dpm->entry == &dfs_list)
		list_add_tail(&n->entry, &dfs_list);

	n->dev = dev;

	mutex_unlock(&dfs_list_mtx);
}
EXPORT_SYMBOL(hobot_dfs_register);

void hobot_dfs_unregister(struct hobot_dpm *n)
{
        mutex_lock(&dfs_list_mtx);
        list_del(&n->entry);
        mutex_unlock(&dfs_list_mtx);
}
EXPORT_SYMBOL(hobot_dfs_unregister);

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
	for (i = 0; i < 6; i++)
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

static CR5_SEND_IPI_HANDLE_T cr5_start;
void register_cr5_start_func(CR5_SEND_IPI_HANDLE_T func)
{
	cr5_start = func;
}
EXPORT_SYMBOL(register_cr5_start_func);

static CR5_SEND_IPI_HANDLE_T cr5_stop;
void register_cr5_stop_func(CR5_SEND_IPI_HANDLE_T func)
{
	cr5_stop = func;
}
EXPORT_SYMBOL(register_cr5_stop_func);

typedef void *(*CR5_GET_BASE_HANDLE_T)(void);
static CR5_GET_BASE_HANDLE_T cr5_get_msginfo_base;
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
	volatile char *p_reduce_bus_clk = NULL;

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
	p_status = sram_vaddr;
	p_index = sram_vaddr + 2;
	p_err = sram_vaddr + 1;
	p_serive_started = sram_vaddr + 3;
	p_reduce_bus_clk = sram_vaddr + 4;
	if (a1 == DRAM_SET_RATE) {
		if (rate == DDR_FREQ_333) {
			index = 2;
		} else if (rate == DDR_FREQ_1333) {
			index = 1;
		} else if (rate == DDR_FREQ_2666) {
			index = 0;
		} else if (rate == 0) {
			/* most case reclac rate is 0
			 * we dont case, return OK
			 */
			res->a0 = 0;
			return;
		} else {
			res->a0 = -EINVAL;
			return;
		}
	} else {
		index = -1;
	}

	*p_index = index;
	*p_status = 0;
	*p_serive_started = (unsigned char)a1;
	*p_reduce_bus_clk = (unsigned char)a4;
#if 0
	udelay(10);
	send_ipi_cr5();
#endif
	cr5_start();
	while(!*p_status) {
	}
	cr5_stop();
	res->a0 = *p_err;
	res->a1 = (unsigned char)*p_index;
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
	unsigned long flags;

	pr_debug("%s %d enter\n", __func__, smp_processor_id());

	/* tell the clock changing core this core is parked */
	atomic_dec(&g_ddrclk->vote_cnt);
	smp_mb();

	/* this core will spin on the lock before ddr freq
	 * changing finished
	 */
	raw_spin_lock_irqsave(&g_ddrclk->raw_lock, flags);


	raw_spin_unlock_irqrestore(&g_ddrclk->raw_lock, flags);
	pr_debug("%s %d exit\n", __func__, smp_processor_id());
}

extern int smp_call_function_single_irq_disabled(int cpu,
               smp_call_func_t func, void *info, int wait);

static void park_other_cpus(void)
{
    int cpu;
	struct cpumask mask;
	call_single_data_t *csd;
	int ret = 0;

    cpu = get_cpu();

    memcpy(&mask, cpu_online_mask, sizeof(struct cpumask));
    cpumask_clear_cpu(cpu, &mask);

    for_each_cpu(cpu, &mask)
		atomic_inc(&g_ddrclk->vote_cnt);

	smp_mb();

	for_each_cpu(cpu, &mask) {
		csd = &per_cpu(dfs_csd, cpu);
		csd->func = spin_on_cpu;
        ret = smp_call_function_single_async(cpu, csd);
		if(ret < 0)
			pr_err("single_async failed %d\n", ret);
	}
	put_cpu();
}

#define POWERSAVE	"powersave"
extern ssize_t hobot_dmc_governor(char *buf);

static int dmc_set_rate(struct clk_hw *hw,
			unsigned long rate, unsigned long parent_rate)
{
	struct dmc_clk *dclk = container_of(hw, struct dmc_clk, hw);
	struct arm_smccc_res res;
	char buf[DEVFREQ_NAME_LEN] = {0};
	uint32_t is_powersave = 0;
	unsigned long flags;
	int timeout = 2000;
	int ret = 0;

	raw_spin_lock_irqsave(&dclk->raw_lock, flags);

	atomic_set(&dclk->vote_cnt, 0);
	park_other_cpus();

	if(!mutex_trylock(&dclk->mlock)) {
		ret = -EBUSY;
		goto err2;
	}

	while(atomic_read(&dclk->vote_cnt) > 0 && timeout > 0) {
		udelay(100);
		timeout -= 100;
	}

	if (timeout <= 0) {
		pr_err("timeout waiting for ipi voting\n");
		ret = -EBUSY;
		goto err1;
	}

	if (hobot_dmc_governor(buf) >= 0)
		if (!strncmp(buf, POWERSAVE, strlen(POWERSAVE)))
			is_powersave = 1;

	if (rate == 100)
		rate = 333000000;

	printk("%s,%d: is_powersave:%d\n", __func__, __LINE__, is_powersave);

	ret = hobot_dfs_notifier(&dfs_list,
			&dfs_lock_list, HB_BUS_SIGNAL_START, 0);
	if (0 == ret) {
#ifndef CONFIG_HOBOT_XJ3
		dclk->invoke_fn(dclk->fid, dclk->set_cmd,
				dclk->channel, rate, 0, 0, 0, 0, &res);
#else
		dclk->invoke_fn(DMC_FID, DRAM_SET_RATE, 0, rate, is_powersave, 0, 0, 0, &res);
#endif
		if (res.a0 != 0) {
			pr_err("%s: ddr channel:%u set rate to %lu failed with status :%ld\n",
				__func__, dclk->channel, rate, res.a0);
			ret = -EINVAL;
			goto err0;
		}

		dclk->rate = rate;
		//cur_ddr_rate = rate;
	}
err0:
	hobot_dfs_notifier(&dfs_lock_list, &dfs_list, HB_BUS_SIGNAL_END, 0);
err1:
	mutex_unlock(&dclk->mlock);
err2:
	spin_unlock_irqrestore(&dclk->lock, flags);

	return ret;
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
	struct arm_smccc_res res;
#ifndef CONFIG_HOBOT_XJ3

	dclk->invoke_fn(dclk->fid, dclk->get_cmd, dclk->channel, 0, 0, 0, 0, 0, &res);
	dclk->invoke_fn(0, 0, 0, 0, 0, 0, 0, 0, &res);
#else
	dclk->invoke_fn(DMC_FID, DRAM_GET_RATE, 0, 0, 0, 0, 0, 0, &res);
#endif
	if (res.a0 != 0) {
		pr_err("%s: ddr channel:%u get rate failed with status :%ld\n",
			__func__, dclk->channel, res.a0);
	}

	if (DDR_INDEX_P0 == res.a1)
		dclk->rate = DDR_FREQ_2666;
	else if (DDR_INDEX_P1 == res.a1)
		dclk->rate = DDR_FREQ_1333;
	else if (DDR_INDEX_P2 == res.a1)
		dclk->rate = DDR_FREQ_333;

	printk("parent_rate: %ld,  dclk->rate:%ld, pre_ddr_rate:%ld, res.a0:%d\n",
		parent_rate, dclk->rate, cur_ddr_rate, res.a1);

	cur_ddr_rate = dclk->rate;
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
	ddrclk->rate = DDR_FREQ_2666;
	spin_lock_init(&ddrclk->lock);
	raw_spin_lock_init(&ddrclk->raw_lock);
	mutex_init(&ddrclk->mlock);
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
