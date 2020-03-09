/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * Zhang Guoying <guoying.zhang@horizon.ai>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#include <linux/slab.h>
#include <linux/sched.h>
#include "bpu.h"
#include "bpu_core.h"

extern struct bpu *g_bpu;

void bpu_sched_seed_update(void)
{
	struct bpu_core *tmp_core;
	struct list_head *pos, *pos_n;
	int run_fc_num = 0;
	int i;

	list_for_each_safe(pos, pos_n, &g_bpu->core_list) {
		tmp_core = (struct bpu_core *)pos;
		if (tmp_core) {
			for (i = 0; i < BPU_PRIO_NUM; i++)
				run_fc_num += kfifo_len(&tmp_core->run_fc_fifo[i]);
		}
	}

	if (run_fc_num) {
		if (g_bpu->sched_seed < HZ)
			g_bpu->sched_seed++;
		if (g_bpu->sched_seed > HZ)
			g_bpu->sched_seed = HZ;
	} else {
		g_bpu->sched_seed = 2;
		mod_timer(g_bpu->sched_timer, jiffies + g_bpu->sched_seed);
	}
}
EXPORT_SYMBOL(bpu_sched_seed_update);

static void bpu_sched_worker(unsigned long arg)
{
	struct bpu *bpu = (struct bpu *)arg;
	int ret;

	if (!bpu) {
		pr_err("No bpu to sched!\n");
		return;
	}

	bpu->stat_reset_count++;
	if (bpu->stat_reset_count >= HZ / bpu->sched_seed) {
		ret = bpu_stat_reset(bpu);
		if (ret)
			pr_err("Bpu stat reset failed!\n");

		/*TODO: judge wether bpu core dead, if dead, reset */
		bpu->stat_reset_count = 0;
	}

	bpu->sched_timer->expires = jiffies + bpu->sched_seed;
	add_timer(bpu->sched_timer);

	if (bpu->sched_seed < HZ)
		bpu->sched_seed++;
	if (bpu->sched_seed > HZ)
		bpu->sched_seed = HZ;
}

int bpu_sched_start(struct bpu *bpu)
{
	if (!bpu)
		return -EINVAL;

	bpu->sched_timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (!bpu->sched_timer) {
		pr_err("bpu sched timer create failed\n");
		return -ENOMEM;
	}

	init_timer(bpu->sched_timer);
	bpu->sched_timer->function = &bpu_sched_worker;
	bpu->sched_timer->data = (unsigned long)bpu;
	bpu->sched_seed = HZ;
	bpu->stat_reset_count = 0;

	bpu->sched_timer->expires = jiffies + bpu->sched_seed;
	add_timer(bpu->sched_timer);

	return 0;
}

int bpu_sched_stop(struct bpu *bpu)
{
	if (!bpu)
		return -EINVAL;

	if (!bpu->sched_timer) {
		pr_err("Try to stop uninited bpu sched\n");
		return -EINVAL;
	}

	del_timer_sync(bpu->sched_timer);
	kfree(bpu->sched_timer);
	bpu->sched_timer = NULL;
	return 0;
}

MODULE_DESCRIPTION("BPU and Cores sched for process");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
