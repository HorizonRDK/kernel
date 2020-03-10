/*
 * Copyright (C) 2020 Horizon Robotics
 *
 * Zhang Guoying <guoying.zhang@horizon.ai>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include "bpu_prio.h"
#include "bpu_ctrl.h"

/*
 * Provide bpu priority method, some soc can support
 * limited hardware priority, the other not support 
 * hardware priority.
 * bpu prio try to support software priority.
 */

static void bpu_prio_tasklet(unsigned long data)
{
	struct bpu_prio *prio = (struct bpu_prio *)data;
	struct bpu_core *core;
	struct bpu_prio_node *tmp_prio_node;
	int32_t running_fc_num = 0;
	int32_t ret, i;

	if (prio == NULL)
		return;

	if (prio->inited == 0) {
		return;
	}

	core = prio->bind_core;

	if (core == NULL) {
		pr_err("BPU Prio has not bind bpu core\n");
		return;
	}

	if (core->fc_buf_limit > 0) {
		for(i = 0; i < BPU_PRIO_NUM; i++)
			running_fc_num += kfifo_len(&core->run_fc_fifo[i]);

		if (running_fc_num >= core->fc_buf_limit)
			return;
	}

	for (i = prio->level_num - 1; i >= 0; i--) {
		tmp_prio_node = &prio->prios[i];
		if (tmp_prio_node->left_slice_num > 0) {
			ret = bpu_write_fc_to_core(core,
					&tmp_prio_node->residue_bpu_fc,
					tmp_prio_node->residue_bpu_fc.info.slice_num
					- tmp_prio_node->left_slice_num);
			if (ret >= 0)
				tmp_prio_node->left_slice_num -= ret;

		} else {
			if (kfifo_len(&tmp_prio_node->buf_fc_fifo) == 0)
				continue;
			ret = kfifo_get(&tmp_prio_node->buf_fc_fifo,
					&tmp_prio_node->residue_bpu_fc);
			if (ret <= 0)
				continue;
			ret = bpu_write_fc_to_core(core,
					&tmp_prio_node->residue_bpu_fc, 0);
			if (ret < 0)
				ret = 0;

			tmp_prio_node->left_slice_num =
				tmp_prio_node->residue_bpu_fc.info.slice_num - ret;
			if (tmp_prio_node->left_slice_num < 0)
				tmp_prio_node->left_slice_num = 0;
		}
	}
}

struct bpu_prio *bpu_prio_init(struct bpu_core *core, uint32_t levels)
{
	struct bpu_prio *prio;
	int32_t i, j, ret;

	prio = kzalloc(sizeof(struct bpu_prio), GFP_KERNEL);
	if (prio == NULL) {
		pr_err("can't create bpu prio for mem failed\n");
		return NULL;
	}

	prio->bind_core = core;
	prio->level_num = levels;

	prio->prios = kzalloc(sizeof(struct bpu_prio_node) *
			prio->level_num, GFP_KERNEL);
	if (!prio->prios) {
		kfree(prio);
		pr_err("can't create bpu prio container for mem failed\n");
		return NULL;
	}

	mutex_init(&prio->mutex_lock);

	for (i = 0; i < prio->level_num; i++) {
		ret = kfifo_alloc(&prio->prios[i].buf_fc_fifo,
				FC_MAX_DEPTH, GFP_KERNEL);
		if (ret) {
			for (j = 0; j < i; j++)
				kfifo_free(&prio->prios[i].buf_fc_fifo);

			kfree(prio->prios);
			kfree(prio);
			pr_err("can't create bpu prio for mem failed\n");
			return NULL;
		}
		prio->prios[i].level = i;
		prio->prios[i].left_slice_num = 0;
	}

	tasklet_init(&prio->tasklet, bpu_prio_tasklet, (unsigned long)prio);
	prio->inited = 1;

	return prio;
}
EXPORT_SYMBOL(bpu_prio_init);

void bpu_prio_exit(struct bpu_prio *prio)
{
	int32_t i;

	if(prio == NULL)
		return;

	mutex_lock(&prio->mutex_lock);
	tasklet_kill(&prio->tasklet);

	for (i = 0; i < prio->level_num; i++)
		kfifo_free(&prio->prios[i].buf_fc_fifo);

	prio->inited = 0;
	kfree(prio->prios);
	prio->prios = NULL;
	mutex_unlock(&prio->mutex_lock);
	kfree(prio);
}
EXPORT_SYMBOL(bpu_prio_exit);

int32_t bpu_prio_in(struct bpu_prio *prio, struct bpu_fc *bpu_fc)
{
	uint32_t level;
	int32_t ret;

	if (prio == NULL)
		return -EINVAL;

	level = bpu_fc->info.priority;

	if (level > prio->level_num)
		level = prio->level_num - 1;

	mutex_lock(&prio->mutex_lock);
	if (prio->inited == 0) {
		mutex_unlock(&prio->mutex_lock);
		return -EINVAL;
	}

	if (kfifo_is_full(&prio->prios[level].buf_fc_fifo)) {
		mutex_unlock(&prio->mutex_lock);
		return -EBUSY;
	}

	ret = kfifo_in(&prio->prios[level].buf_fc_fifo, bpu_fc, 1);
	if (ret < 1) {
		mutex_unlock(&prio->mutex_lock);
		return -EBUSY;
	}

	ret = bpu_prio_trig_out(prio);
	mutex_unlock(&prio->mutex_lock);

	return ret;
}
EXPORT_SYMBOL(bpu_prio_in);

int32_t bpu_prio_trig_out(struct bpu_prio *prio)
{
	if (prio == NULL)
		return -EINVAL;

	if (prio->inited == 0) {
		return -EINVAL;
	}

	tasklet_schedule(&prio->tasklet);

	return 0;
}
EXPORT_SYMBOL(bpu_prio_trig_out);
