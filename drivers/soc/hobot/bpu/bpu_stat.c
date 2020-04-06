/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#include "bpu.h"
#include "bpu_core.h"

#define P_RESET_COEF	(16u)

static uint64_t time_val(const struct timeval *p)
{
	if (p == NULL) {
		return 0u;
	}

	return (((uint64_t)p->tv_sec * (uint64_t)SECTOUS) + (uint64_t)p->tv_usec);
}

static uint64_t time_interval(const struct timeval *p_old, const struct timeval *p_new)
{
	return time_val(p_new) - time_val(p_old);
}

/* bpu time statistics mainly trigger by fc event*/
void bpu_core_update(struct bpu_core *core, struct bpu_fc *fc)
{
	struct bpu_fc_group *tmp_fc_group;
	struct bpu_user *tmp_user;
	struct timeval tmp_start_point;
	uint64_t tmp_time;
	unsigned long flags;/*PRQA S ALL*/

	if ((core == NULL) || (fc == NULL)) {
		return;
	}

	spin_lock_irqsave(&core->spin_lock, flags);/*PRQA S ALL*/
	do_gettimeofday(&fc->end_point);

	fc->info.process_time = time_interval(&fc->start_point, &fc->end_point);

	tmp_fc_group = bpu_get_fc_group(fc);
	tmp_user = bpu_get_user(fc);

	/* update for start point for interval calc */
	if (time_val(&fc->start_point) < time_val(&core->last_done_point)) {
		tmp_start_point = core->last_done_point;
	} else {
		tmp_start_point = fc->start_point;
	}

	if (time_val(&tmp_start_point) >= time_val(&core->p_start_point)) {
		tmp_time = time_interval(&tmp_start_point, &fc->end_point);
	} else {
		tmp_time = time_interval(&core->p_start_point, &fc->end_point);
	}

	core->p_run_time += tmp_time;
	if (tmp_fc_group != NULL) {
		tmp_fc_group->p_run_time += tmp_time;
	}
	if (tmp_user != NULL) {
		tmp_user->p_run_time += tmp_time;
	}

	core->last_done_point = fc->end_point;

	spin_unlock_irqrestore(&core->spin_lock, flags);
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_update);
// PRQA S ALL --

static int32_t bpu_core_stat_reset(struct bpu_core *core)
{
	struct timeval tmp_start_point;
	struct list_head *pos, *pos_n;
	struct bpu_user *tmp_user;
	uint64_t tmp_time;
	unsigned long flags;/*PRQA S ALL*/
	unsigned long user_flags;/*PRQA S ALL*/

	if (core == NULL) {
		return -EINVAL;
	}

	spin_lock_irqsave(&core->spin_lock, flags);/*PRQA S ALL*/

	do_gettimeofday(&tmp_start_point);
	core->p_run_time /= P_RESET_COEF;

	/* reset the core user p_run_time */
	list_for_each_safe(pos, pos_n, &core->user_list) {/*PRQA S ALL*/
		tmp_user = (struct bpu_user *)list_entry(pos, struct bpu_user, node);/*PRQA S ALL*/
		if (tmp_user != NULL) {
			spin_lock_irqsave(&tmp_user->spin_lock, user_flags);/*PRQA S ALL*/
			tmp_user->p_run_time /= P_RESET_COEF;
			spin_unlock_irqrestore(&tmp_user->spin_lock, user_flags);
		}
	}

	/* use last reset interval coefficient */
	if (core->p_start_point.tv_sec != 0) {
		tmp_time = time_val(&tmp_start_point)
			- (time_interval(&core->p_start_point,
					&tmp_start_point) / P_RESET_COEF);

		core->p_start_point.tv_sec = tmp_time / SECTOUS;/*PRQA S ALL*/
		core->p_start_point.tv_usec = tmp_time % SECTOUS;/*PRQA S ALL*/
	} else {
		core->p_start_point = tmp_start_point;
	}

	spin_unlock_irqrestore(&core->spin_lock, flags);

	return 0;
}

int32_t bpu_stat_reset(struct bpu *bpu)
{
	struct list_head *pos, *pos_n;
	struct bpu_core *tmp_core;
	struct bpu_fc_group *tmp_fc_group;
	struct bpu_user *tmp_user;
	unsigned long group_flags;/*PRQA S ALL*/
	unsigned long user_flags;/*PRQA S ALL*/
	int32_t ret;

	/* reset the bpu core */
	list_for_each_safe(pos, pos_n, &bpu->core_list) {/*PRQA S ALL*/
		tmp_core = (struct bpu_core *)list_entry(pos, struct bpu_core, node);/*PRQA S ALL*/
		if (tmp_core != NULL) {
			ret = bpu_core_stat_reset(tmp_core);
			if (ret != 0) {
				mutex_unlock(&bpu->mutex_lock);
				pr_err("bpu core[%d] stat reset failed\n", tmp_core->index);/*PRQA S ALL*/
				return ret;
			}
		}
	}

	/* reset the bpu group p_run_time */
	list_for_each_safe(pos, pos_n, &bpu->group_list) {/*PRQA S ALL*/
		tmp_fc_group = (struct bpu_fc_group *)list_entry(pos, struct bpu_fc_group, node);/*PRQA S ALL*/
		if (tmp_fc_group != NULL) {
			spin_lock_irqsave(&tmp_fc_group->spin_lock, group_flags);/*PRQA S ALL*/
			tmp_fc_group->p_run_time /= P_RESET_COEF;
			spin_unlock_irqrestore(&tmp_fc_group->spin_lock, group_flags);
		}
	}

	/* reset the bpu user p_run_time */
	list_for_each_safe(pos, pos_n, &bpu->user_list) {/*PRQA S ALL*/
		tmp_user = (struct bpu_user *)list_entry(pos, struct bpu_user, node);/*PRQA S ALL*/
		if (tmp_user != NULL) {
			spin_lock_irqsave(&tmp_user->spin_lock, user_flags);/*PRQA S ALL*/
			tmp_user->p_run_time /= P_RESET_COEF;
			spin_unlock_irqrestore(&tmp_user->spin_lock, user_flags);
		}
	}

	return 0;
}

uint32_t bpu_core_ratio(struct bpu_core *core)
{
	struct timeval tmp_point;
	uint64_t pass_time;
	unsigned long flags;/*PRQA S ALL*/
	uint64_t ratio;

	spin_lock_irqsave(&core->spin_lock, flags);/*PRQA S ALL*/
	do_gettimeofday(&tmp_point);

	pass_time = time_interval(&core->p_start_point, &tmp_point);
	if (core->p_run_time > pass_time) {
		core->p_run_time = pass_time;
	}

	ratio = core->p_run_time * PERSENT / pass_time;
	spin_unlock_irqrestore(&core->spin_lock, flags);

	core->ratio = (uint32_t)ratio;

	return (uint32_t)ratio;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_ratio);
// PRQA S ALL --

uint32_t bpu_fc_group_ratio(struct bpu_fc_group *group)
{
	struct list_head *pos, *pos_n;
	struct timeval tmp_point;
	struct bpu_core *tmp_core;
	uint64_t pass_time = 0;
	unsigned long flags;/*PRQA S ALL*/
	uint64_t ratio = 0;

	if (group == NULL) {
		return 0;
	}

	spin_lock_irqsave(&group->spin_lock, flags);/*PRQA S ALL*/
	do_gettimeofday(&tmp_point);

	list_for_each_safe(pos, pos_n, &g_bpu->core_list) {/*PRQA S ALL*/
		tmp_core = (struct bpu_core *)list_entry(pos, struct bpu_core, node);/*PRQA S ALL*/
		if (bpu_core_is_online(tmp_core)) {
			pass_time += time_interval(&tmp_core->p_start_point, &tmp_point);
		}
	}

	if (pass_time != 0u) {
		if (group->p_run_time > pass_time) {
			group->p_run_time = pass_time;
		}

		ratio = group->p_run_time * PERSENT / pass_time;
	}
	spin_unlock_irqrestore(&group->spin_lock, flags);

	return (uint32_t)ratio;
}

uint32_t bpu_user_ratio(struct bpu_user *user)
{
	struct list_head *pos, *pos_n;
	struct timeval tmp_point;
	struct bpu_core *tmp_core;
	uint64_t pass_time = 0;
	unsigned long flags;/*PRQA S ALL*/
	uint64_t ratio = 0u;

	if (user == NULL) {
		return 0;
	}

	spin_lock_irqsave(&user->spin_lock, flags);/*PRQA S ALL*/
	do_gettimeofday(&tmp_point);

	list_for_each_safe(pos, pos_n, &g_bpu->core_list) {/*PRQA S ALL*/
		tmp_core = (struct bpu_core *)list_entry(pos, struct bpu_core, node);/*PRQA S ALL*/
		if (bpu_core_is_online(tmp_core)) {
			pass_time += time_interval(&tmp_core->p_start_point, &tmp_point);
		}
	}

	if (pass_time != 0u) {
		if (user->p_run_time > pass_time) {
			user->p_run_time = pass_time;
		}

		ratio = user->p_run_time * PERSENT / pass_time;
	}
	spin_unlock_irqrestore(&user->spin_lock, flags);

	return (uint32_t)ratio;
}

uint32_t bpu_ratio(struct bpu *bpu)
{
	struct list_head *pos, *pos_n;
	struct bpu_core *tmp_core;
	uint32_t ratio = 0u;
	uint32_t core_num = 0u;

	list_for_each_safe(pos, pos_n, &bpu->core_list) {/*PRQA S ALL*/
		tmp_core = (struct bpu_core *)list_entry(pos, struct bpu_core, node);/*PRQA S ALL*/
		if (bpu_core_is_online(tmp_core)) {
			ratio += bpu_core_ratio(tmp_core);
			core_num++;
		}
	}
	if (core_num == 0u) {
		return 0;
	}

	bpu->ratio = ratio / core_num;

	return bpu->ratio;
}

// PRQA S ALL ++
MODULE_DESCRIPTION("BPU and Cores statistics related realize");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
// PRQA S ALL --
