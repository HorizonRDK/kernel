/*
 * Copyright (C) 2020 Horizon Robotics
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

static void bpu_prio_tasklet(unsigned long data)/*PRQA S ALL*/
{
	struct bpu_prio *prio = (struct bpu_prio *)data;/*PRQA S ALL*/
	struct bpu_core *core;
	struct bpu_prio_node *tmp_prio_node;
	int32_t running_fc_num = 0;
	int32_t ret, i;

	if (prio == NULL) {
		return;
	}

	if (prio->inited == 0u) {
		return;
	}

	core = prio->bind_core;

	if (core == NULL) {
		pr_err("BPU Prio has not bind bpu core\n");/*PRQA S ALL*/
		return;
	}

	if (core->fc_buf_limit > 0) {
		for(i = 0; i < (int32_t)BPU_PRIO_NUM; i++) {
			running_fc_num += kfifo_len(&core->run_fc_fifo[i]);/*PRQA S ALL*/
		}

		if (running_fc_num >= core->fc_buf_limit) {
			return;
		}
	}

	for (i = (int32_t)prio->level_num - 1; i >= 0; i--) {
		tmp_prio_node = &prio->prios[i];
		if (tmp_prio_node->left_slice_num > 0) {
			ret = bpu_write_fc_to_core(core,
					&tmp_prio_node->residue_bpu_fc,
					tmp_prio_node->residue_bpu_fc.info.slice_num
					- (uint32_t)tmp_prio_node->left_slice_num);
			if (ret >= 0) {
				tmp_prio_node->left_slice_num -= ret;
			}
		} else {
			if (kfifo_len(&tmp_prio_node->buf_fc_fifo) == 0) {/*PRQA S ALL*/
				if (i == 0) {
					complete(&prio->no_task_comp);
				}
				continue;
			}
			ret = kfifo_get(&tmp_prio_node->buf_fc_fifo,/*PRQA S ALL*/
					&tmp_prio_node->residue_bpu_fc);
			if (ret <= 0) {
				continue;
			}
			ret = bpu_write_fc_to_core(core,
					&tmp_prio_node->residue_bpu_fc, 0);
			if (ret < 0) {
				ret = 0;
			}

			tmp_prio_node->left_slice_num =
				(int32_t)tmp_prio_node->residue_bpu_fc.info.slice_num - ret;
			if (tmp_prio_node->left_slice_num < 0) {
				tmp_prio_node->left_slice_num = 0;
			}
		}
		if (tmp_prio_node->left_slice_num == 0) {
			tmp_prio_node->buffered_time -= tmp_prio_node->residue_bpu_fc.info.process_time;
		}
		break;
	}
}

struct bpu_prio *bpu_prio_init(struct bpu_core *core, uint32_t levels)
{
	struct bpu_prio *prio;
	uint32_t i, j;

	prio = (struct bpu_prio *)kzalloc(sizeof(struct bpu_prio), GFP_KERNEL);/*PRQA S ALL*/
	if (prio == NULL) {
		pr_err("can't create bpu prio for mem failed\n");/*PRQA S ALL*/
		return NULL;
	}

	prio->bind_core = core;
	prio->level_num = levels;

	prio->prios = (struct bpu_prio_node *)kzalloc(sizeof(struct bpu_prio_node) * prio->level_num, GFP_KERNEL);/*PRQA S ALL*/
	if (prio->prios == NULL) {
		kfree((void *)prio);/*PRQA S ALL*/
		pr_err("can't create bpu prio container for mem failed\n");/*PRQA S ALL*/
		return NULL;
	}

	mutex_init(&prio->mutex_lock);/*PRQA S ALL*/
	init_completion(&prio->no_task_comp);/*PRQA S ALL*/

	for (i = 0u; i < prio->level_num; i++) {
		prio->prios[i].bpu_fc_fifo_buf =
			vmalloc(roundup_pow_of_two(FC_MAX_DEPTH / (i + 1u))
					* sizeof(struct bpu_fc));
		if (prio->prios[i].bpu_fc_fifo_buf == NULL) {
			for (j = 0; j < i; j++) {
				vfree(prio->prios[i].bpu_fc_fifo_buf);/*PRQA S ALL*/
			}

			kfree((void *)prio->prios);/*PRQA S ALL*/
			kfree((void *)prio);/*PRQA S ALL*/
			pr_err("can't create bpu prio for mem failed\n");/*PRQA S ALL*/
			return NULL;
		}

		kfifo_init(&prio->prios[i].buf_fc_fifo,
				prio->prios[i].bpu_fc_fifo_buf,
				roundup_pow_of_two(FC_MAX_DEPTH / (i + 1u))
				* sizeof(struct bpu_fc));

		prio->prios[i].level = (uint32_t)i;
		prio->prios[i].left_slice_num = 0;
	}

	tasklet_init(&prio->tasklet, bpu_prio_tasklet, (unsigned long)prio);/*PRQA S ALL*/
	prio->plug_in = 1;
	prio->inited = 1;

	return prio;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_prio_init);
// PRQA S ALL --

void bpu_prio_exit(struct bpu_prio *prio)
{
	struct bpu_fc tmp_bpu_fc;
	uint32_t i;
	int32_t ret;

	if(prio == NULL) {
		return;
	}

	mutex_lock(&prio->mutex_lock);
	tasklet_kill(&prio->tasklet);

	for (i = 0u; i < prio->level_num; i++) {
		while (!kfifo_is_empty(&prio->prios[i].buf_fc_fifo)) {
			ret = kfifo_get(&prio->prios[i].buf_fc_fifo, &tmp_bpu_fc);/*PRQA S ALL*/
			if (ret < 1) {
				continue;
			}
			bpu_fc_clear(&tmp_bpu_fc);
		}
		vfree(prio->prios[i].bpu_fc_fifo_buf);/*PRQA S ALL*/
	}

	prio->plug_in = 0;
	prio->inited = 0;
	kfree((void *)prio->prios); /*PRQA S ALL*/
	prio->prios = NULL;
	mutex_unlock(&prio->mutex_lock);
	kfree((void *)prio);/*PRQA S ALL*/
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_prio_exit);
// PRQA S ALL --

int32_t bpu_prio_in(struct bpu_prio *prio, const struct bpu_fc *bpu_fc)
{
	uint32_t level;
	int32_t ret;

	if (prio == NULL) {
		return -EINVAL;
	}

	level = bpu_fc->info.priority;

	if (level > prio->level_num) {
		level = prio->level_num - 1u;
	}

	mutex_lock(&prio->mutex_lock);
	if (prio->inited == 0u) {
		mutex_unlock(&prio->mutex_lock);
		return -EINVAL;
	}

	if (kfifo_is_full(&prio->prios[level].buf_fc_fifo)) {/*PRQA S ALL*/
		mutex_unlock(&prio->mutex_lock);
		return -EBUSY;
	}

	ret = kfifo_in(&prio->prios[level].buf_fc_fifo, bpu_fc, 1);/*PRQA S ALL*/
	if (ret < 1) {
		mutex_unlock(&prio->mutex_lock);
		return -EBUSY;
	}

	prio->prios[level].buffered_time += bpu_fc->info.process_time;
	bpu_prio_trig_out(prio);
	mutex_unlock(&prio->mutex_lock);

	return ret;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_prio_in);
// PRQA S ALL --

void bpu_prio_trig_out(struct bpu_prio *prio)
{
	if (prio == NULL) {
		return;
	}

	if (prio->inited == 0u) {
		return;
	}

	tasklet_schedule(&prio->tasklet);
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_prio_trig_out);
// PRQA S ALL --

static uint32_t bpu_prio_left_task_num(struct bpu_prio *prio)
{
	uint32_t left_num = 0u;
	uint32_t i;

	if (prio == NULL) {
		return 0;
	}

	if (prio->inited == 0u) {
		return 0;
	}

	for (i = 0u; i < prio->level_num; i++) {
		left_num = kfifo_len(&prio->prios[i].buf_fc_fifo);/*PRQA S ALL*/
	}

	return left_num;
}

bool bpu_prio_is_plug_in(struct bpu_prio *prio)
{
	if (prio == NULL) {
		return false;
	}

	if (prio->inited == 0u) {
		return false;
	}

	if (prio->plug_in > 0u) {
		return true;
	}

	return false;
}

void bpu_prio_plug_in(struct bpu_prio *prio)
{
	if (prio == NULL) {
		return;
	}

	if (prio->inited == 0u) {
		return;
	}

	prio->plug_in = 1u;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_prio_plug_in);
// PRQA S ALL --

void bpu_prio_plug_out(struct bpu_prio *prio)
{
	if (prio == NULL) {
		return;
	}

	if (prio->inited == 0u) {
		return;
	}

	prio->plug_in = 0u;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_prio_plug_out);
// PRQA S ALL --

int32_t bpu_prio_wait_empty(struct bpu_prio *prio, int32_t jiffes)
{
	uint32_t left_num;

	if (prio == NULL) {
		return 0;
	}

	if (prio->inited == 0u) {
		return 0;
	}

	left_num = bpu_prio_left_task_num(prio);
	if (left_num > 0u) {
		if(wait_for_completion_timeout(&prio->no_task_comp, jiffes) == 0u) {
			/* timeout, may need wait again*/
			return -EAGAIN;
		}
	}

	return 0;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_prio_wait_empty);
// PRQA S ALL --
