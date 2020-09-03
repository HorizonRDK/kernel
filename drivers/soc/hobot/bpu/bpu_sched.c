/*
 * Copyright (C) 2019 Horizon Robotics
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
#include "bpu_ctrl.h"

#define DEFAULT_SCHED_SEED (2u)

void bpu_sched_seed_update(void)
{
	struct bpu_core *tmp_core;
	struct list_head *pos, *pos_n;
	int32_t run_fc_num = 0;
	uint32_t i;

	list_for_each_safe(pos, pos_n, &g_bpu->core_list) { /*PRQA S ALL*/
		tmp_core = (struct bpu_core *)pos;/*PRQA S ALL*/
		if (tmp_core != NULL) {
			for (i = 0; i < BPU_PRIO_NUM; i++) {
				run_fc_num += kfifo_len(&tmp_core->run_fc_fifo[i]);/*PRQA S ALL*/
			}
		}
	}

	if (run_fc_num > 0) {
		if (g_bpu->sched_seed < (uint32_t)HZ) {
			g_bpu->sched_seed++;
		}
		if (g_bpu->sched_seed > (uint32_t)HZ) {
			g_bpu->sched_seed = HZ;
		}
	} else {
		g_bpu->sched_seed = DEFAULT_SCHED_SEED;
	}
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_sched_seed_update);
// PRQA S ALL --

static void bpu_sched_check_to_core(struct bpu *bpu)
{
	struct bpu_core *tmp_core;
	struct list_head *pos, *pos_n;
	int32_t ret;

	if (bpu == NULL) {
		return;
	}

	list_for_each_safe(pos, pos_n, &bpu->core_list) {/*PRQA S ALL*/
		tmp_core = (struct bpu_core *)pos;/*PRQA S ALL*/
		if (tmp_core != NULL) {
			if ((tmp_core->hw_ops->status == NULL) || (tmp_core->hw_enabled == 0)) {
				continue;
			}

			(void)tmp_core->hw_ops->status(tmp_core, UPDATE_STATE);
			if (tmp_core->hw_ops->status(tmp_core, WORK_STATE) > 0) {
				continue;
			}

			ret = bpu_core_reset(tmp_core);
			if (ret != 0) {
				pr_err("Bpu core%d reset failed when check not work!\n",
						tmp_core->index);/*PRQA S ALL*/
			}

			ret = bpu_core_process_recover(tmp_core);
			if (ret != 0) {
				pr_err("BPU core%d recover failed\n", tmp_core->index);
			}
		}
	}
}

static void bpu_sched_worker(unsigned long arg) /*PRQA S ALL*/
{
	struct bpu *bpu = (struct bpu *)arg;  /*PRQA S ALL*/
	uint32_t tmp_reset_count;
	int32_t ret;

	if (bpu == NULL) {
		pr_err("No bpu to sched!\n");/*PRQA S ALL*/
		return;
	}

	bpu->stat_reset_count++;
	tmp_reset_count = (uint32_t)HZ / bpu->sched_seed;
	if (bpu->stat_reset_count >= tmp_reset_count) {
		ret = bpu_stat_reset(bpu);
		if (ret != 0) {
			pr_err("Bpu stat reset failed!\n");/*PRQA S ALL*/
		}

		/* judge wether bpu core dead, if dead, reset and recovery */
		bpu_sched_check_to_core(bpu);

		bpu->stat_reset_count = 0;
	}

	bpu->sched_timer->expires = jiffies + bpu->sched_seed;
	add_timer(bpu->sched_timer);

	if (bpu->sched_seed < (uint32_t)HZ) {
		bpu->sched_seed++;
	}
	if (bpu->sched_seed > (uint32_t)HZ) {
		bpu->sched_seed = HZ;
	}
}

int32_t bpu_sched_start(struct bpu *bpu)
{
	if (bpu == NULL) {
		return -EINVAL;
	}

	bpu->sched_timer = (struct timer_list *)kmalloc(sizeof(struct timer_list), GFP_KERNEL);/*PRQA S ALL*/
	if (bpu->sched_timer == NULL) {
		pr_err("bpu sched timer create failed\n");/*PRQA S ALL*/
		return -ENOMEM;
	}

	init_timer(bpu->sched_timer);/*PRQA S ALL*/
	bpu->sched_timer->function = &bpu_sched_worker;
	bpu->sched_timer->data = (unsigned long)bpu;/*PRQA S ALL*/
	bpu->sched_seed = HZ;
	bpu->stat_reset_count = 0;

	bpu->sched_timer->expires = jiffies + DEFAULT_SCHED_SEED;
	add_timer(bpu->sched_timer);

	return 0;
}

int32_t bpu_sched_stop(struct bpu *bpu)
{
	int32_t ret;

	if (bpu == NULL) {
		return -EINVAL;
	}

	if (bpu->sched_timer == NULL) {
		pr_err("Try to stop uninited bpu sched\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	ret = del_timer_sync(bpu->sched_timer);
	if (ret == 0) {
		pr_debug("del no sched timer\n");/*PRQA S ALL*/
	}
	kfree((void *)bpu->sched_timer);/*PRQA S ALL*/
	bpu->sched_timer = NULL;
	return 0;
}

// PRQA S ALL ++
MODULE_DESCRIPTION("BPU and Cores sched for process");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
// PRQA S ALL --
