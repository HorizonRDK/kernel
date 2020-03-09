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
#include "bpu.h"
#include "bpu_core.h"

#define P_RESET_COEF	16

extern struct bpu *g_bpu;

/* bpu time statistics mainly trigger by fc event*/
int bpu_core_update(struct bpu_core *core, struct bpu_fc *fc)
{
	struct bpu_fc_group *tmp_fc_group;
	struct bpu_user *tmp_user;
	uint64_t tmp_time;
	unsigned long flags;

	if (!core || !fc)
		return -EINVAL;

	spin_lock_irqsave(&core->spin_lock, flags);
	do_gettimeofday(&fc->end_point);

	if (TIME_VAL(&fc->start_point) < TIME_VAL(&core->last_done_point))
		fc->start_point = core->last_done_point;

	fc->info.process_time = TIME_INTERVAL(&fc->start_point, &fc->end_point);

	tmp_fc_group = bpu_get_fc_group(fc);
	tmp_user = bpu_get_user(fc);
	if (unlikely(TIME_VAL(&fc->start_point) < TIME_VAL(&core->p_start_point))) {
		tmp_time = TIME_INTERVAL(&core->p_start_point, &fc->end_point);
		core->p_run_time += tmp_time;
		if (tmp_fc_group)
			tmp_fc_group->p_run_time += tmp_time;
		if (tmp_user)
			tmp_user->p_run_time += tmp_time;
	} else {
		core->p_run_time += fc->info.process_time;
		if (tmp_fc_group)
			tmp_fc_group->p_run_time += fc->info.process_time;
		if (tmp_user)
			tmp_user->p_run_time += fc->info.process_time;
	}

	core->last_done_point = fc->end_point;

	core->k_point = fc->end_point;

	spin_unlock_irqrestore(&core->spin_lock, flags);

	return 0;
}
EXPORT_SYMBOL(bpu_core_update);

static int bpu_core_stat_reset(struct bpu_core *core)
{
	struct timeval tmp_start_point;
	struct list_head *pos, *pos_n;
	struct bpu_user *tmp_user;
	uint64_t tmp_time;
	unsigned long flags;
	unsigned long user_flags;

	if (!core)
		return -EINVAL;

	spin_lock_irqsave(&core->spin_lock, flags);

	do_gettimeofday(&tmp_start_point);
	core->p_run_time /= P_RESET_COEF;

	/* reset the core user p_run_time */
	list_for_each_safe(pos, pos_n, &core->user_list) {
		tmp_user = list_entry(pos, struct bpu_user, node);
		if (tmp_user) {
			spin_lock_irqsave(&tmp_user->spin_lock, user_flags);
			tmp_user->p_run_time /= P_RESET_COEF;
			spin_unlock_irqrestore(&tmp_user->spin_lock, user_flags);
		}
	}

	/* use last reset interval coefficient */
	if (core->p_start_point.tv_sec != 0) {
		tmp_time = TIME_VAL(&tmp_start_point)
			- TIME_INTERVAL(&core->p_start_point,
					&tmp_start_point) / P_RESET_COEF;

		core->p_start_point.tv_sec = tmp_time / 1000000;
		core->p_start_point.tv_usec = tmp_time % 1000000;
	} else
		core->p_start_point = tmp_start_point;

	spin_unlock_irqrestore(&core->spin_lock, flags);

	return 0;
}

int bpu_stat_reset(struct bpu *bpu)
{
	struct list_head *pos, *pos_n;
	struct bpu_core *tmp_core;
	struct bpu_fc_group *tmp_fc_group;
	struct bpu_user *tmp_user;
	unsigned long group_flags;
	unsigned long user_flags;
	int ret;

	/* reset the bpu core */
	list_for_each_safe(pos, pos_n, &bpu->core_list) {
		tmp_core = list_entry(pos, struct bpu_core, node);
		if (tmp_core) {
			ret = bpu_core_stat_reset(tmp_core);
			if (ret) {
				mutex_unlock(&bpu->mutex_lock);
				pr_err("bpu core[%d] stat reset failed\n", tmp_core->index);
				return ret;
			}
		}
	}

	/* reset the bpu group p_run_time */
	list_for_each_safe(pos, pos_n, &bpu->group_list) {
		tmp_fc_group = list_entry(pos, struct bpu_fc_group, node);
		if (tmp_fc_group) {
			spin_lock_irqsave(&tmp_fc_group->spin_lock, group_flags);
			tmp_fc_group->p_run_time /= P_RESET_COEF;
			spin_unlock_irqrestore(&tmp_fc_group->spin_lock, group_flags);
		}
	}

	/* reset the bpu user p_run_time */
	list_for_each_safe(pos, pos_n, &bpu->user_list) {
		tmp_user = list_entry(pos, struct bpu_user, node);
		if (tmp_user) {
			spin_lock_irqsave(&tmp_user->spin_lock, user_flags);
			tmp_user->p_run_time /= P_RESET_COEF;
			spin_unlock_irqrestore(&tmp_user->spin_lock, user_flags);
		}
	}

	return 0;
}

int bpu_core_ratio(struct bpu_core *core)
{
	struct timeval tmp_point;
	uint64_t pass_time;
	unsigned long flags;
	int ratio;

	spin_lock_irqsave(&core->spin_lock, flags);
	do_gettimeofday(&tmp_point);

	pass_time = TIME_INTERVAL(&core->p_start_point, &tmp_point);
	if (core->p_run_time > pass_time)
		core->p_run_time = pass_time;

	ratio = core->p_run_time * 100 / pass_time;
	spin_unlock_irqrestore(&core->spin_lock, flags);

	core->ratio = ratio;

	return ratio;
}
EXPORT_SYMBOL(bpu_core_ratio);

int bpu_fc_group_ratio(struct bpu_fc_group *group)
{
	struct list_head *pos, *pos_n;
	struct timeval tmp_point;
	struct bpu_core *tmp_core;
	uint64_t pass_time = 0;
	unsigned long flags;
	int ratio = 0;

	if (!group)
		return 0;

	spin_lock_irqsave(&group->spin_lock, flags);
	do_gettimeofday(&tmp_point);

	list_for_each_safe(pos, pos_n, &g_bpu->core_list) {
		tmp_core = list_entry(pos, struct bpu_core, node);
		if (tmp_core)
			pass_time += TIME_INTERVAL(&tmp_core->p_start_point, &tmp_point);
	}

	if (pass_time != 0) {
		if (group->p_run_time > pass_time)
			group->p_run_time = pass_time;

		ratio = group->p_run_time * 100 / pass_time;
	}
	spin_unlock_irqrestore(&group->spin_lock, flags);

	return ratio;
}

int bpu_user_ratio(struct bpu_user *user)
{
	struct list_head *pos, *pos_n;
	struct timeval tmp_point;
	struct bpu_core *tmp_core;
	uint64_t pass_time = 0;
	unsigned long flags;
	int ratio = 0;

	if (!user)
		return 0;

	spin_lock_irqsave(&user->spin_lock, flags);
	do_gettimeofday(&tmp_point);

	list_for_each_safe(pos, pos_n, &g_bpu->core_list) {
		tmp_core = list_entry(pos, struct bpu_core, node);
		if (tmp_core)
			pass_time += TIME_INTERVAL(&tmp_core->p_start_point, &tmp_point);
	}

	if (pass_time != 0) {
		if (user->p_run_time > pass_time)
			user->p_run_time = pass_time;

		ratio = user->p_run_time * 100 / pass_time;
	}
	spin_unlock_irqrestore(&user->spin_lock, flags);

	return ratio;
}

int bpu_ratio(struct bpu *bpu)
{
	struct list_head *pos, *pos_n;
	struct bpu_core *tmp_core;
	int ratio = 0;
	int core_num = 0;

	list_for_each_safe(pos, pos_n, &bpu->core_list) {
		tmp_core = list_entry(pos, struct bpu_core, node);
		if (tmp_core) {
			ratio += bpu_core_ratio(tmp_core);
			core_num++;
		}
	}

	if (!core_num)
		return 0;

	bpu->ratio = ratio /core_num;

	return bpu->ratio;
}

MODULE_DESCRIPTION("BPU and Cores statistics related realize");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
