/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#ifndef __BPU_CTRL_H__
#define __BPU_CTRL_H__
#include "bpu_core.h"

int32_t bpu_core_is_pending(const struct bpu_core *core);
int32_t bpu_core_clk_on(const struct bpu_core *core);
int32_t bpu_core_clk_off(const struct bpu_core *core);
int32_t bpu_core_power_on(const struct bpu_core *core);
int32_t bpu_core_power_off(const struct bpu_core *core);
int32_t bpu_core_enable(struct bpu_core *core);
int32_t bpu_core_disable(struct bpu_core *core);
int32_t bpu_core_reset(struct bpu_core *core);
int32_t bpu_core_process_recover(struct bpu_core *core);
int32_t bpu_core_set_limit(struct bpu_core *core, int32_t limit);

#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
int32_t bpu_core_dvfs_register(struct bpu_core *core, const char *name);
void bpu_core_dvfs_unregister(struct bpu_core *core);
int32_t bpu_core_set_freq_level(struct bpu_core *core, int32_t level);
#else
static inline int32_t bpu_core_dvfs_register(struct bpu_core *core,
		const char *name)
{
	return 0;
}

static inline void bpu_core_dvfs_unregister(struct bpu_core *core)
{
	return;
}

static inline int32_t bpu_core_set_freq_level(struct bpu_core *core,
		int32_t level)
{
	pr_err_ratelimited("Not support change freq level\n");
	return 0;
}
#endif
int32_t bpu_core_bus_dvfs_register(struct bpu_core *core);
void bpu_core_bus_dvfs_unregister(struct bpu_core *core);

#endif
