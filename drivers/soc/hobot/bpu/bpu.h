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
#ifndef __BPU_H__
#define __BPU_H__

#include <linux/time.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <uapi/hobot/bpu.h>

#define PERSENT		(100u)
#define SECTOMS		(1000)
#define SECTOUS		(1000000)

/* must be a power of 2 */
#define BPU_CORE_RECORE_NUM		64

struct bpu_core;
struct bpu_hw_fc;

struct bpu_fc {
	struct user_bpu_fc info;

	/* communicate id with bpu hw */
	uint32_t hw_id;

	/* real fc data*/
	struct bpu_hw_fc *fc_data;

	/* group id */
	uint32_t *g_id;

	/* identifier for diff file user */
	void **user;

	/* index in hw core buffer */
	uint32_t index;

	/*
	 * start: time set to hw buffer
	 * end: time done by bpu core
	 */
	struct timeval start_point;
	struct timeval end_point;
};

struct bpu_fc_group {
	struct list_head node;
	uint32_t id;
	int32_t proportion;

	/* when group is busy, firt to buffer fc fifo */
	DECLARE_KFIFO_PTR(buffer_fc_fifo, struct bpu_fc);
	/* to protect in fifo*/
	spinlock_t spin_lock;

	/* run time in statistical period */
	uint64_t p_run_time;
};

struct bpu_user {
	struct list_head node;
	uint32_t id;
	uint16_t is_alive;
	int32_t running_task_num;

	wait_queue_head_t poll_wait;
	/* to protect in fifo*/
	spinlock_t spin_lock;

	/* to protect out fifo*/
	struct mutex mutex_lock;
	struct completion no_task_comp;

	/* which fifo will report to user */
	DECLARE_KFIFO_PTR(done_fcs, struct user_bpu_fc);

	/* run time in statistical period */
	uint64_t p_run_time;

	/*
	 * use the pointer's point to user for judging
	 * whether valid of user after file released
	 */
	void **p_file_private;

	void *host;
};

struct bpu {
	struct device *dev;
	struct miscdevice miscdev;
	atomic_t open_counter;

	struct timer_list *sched_timer;

	struct mutex mutex_lock;
	spinlock_t spin_lock;

	wait_queue_head_t poll_wait;

	/* list to store bpu cores*/
	struct list_head core_list;

	/* list to store user */
	struct list_head user_list;
	/* list to store user */
	struct list_head group_list;
	int32_t busy_thres;

	/* use the value to adjust sched time */
	uint32_t sched_seed;
	uint32_t stat_reset_count;

	struct bus_type *bus;

	uint32_t ratio;
};

extern struct bpu *g_bpu;

int32_t bpu_write_with_user(const struct bpu_core *core,
			struct bpu_user *user,
			const char __user *buf, size_t len);
int32_t bpu_read_with_user(struct bpu_core *core,
			struct bpu_user *user,
			const char __user *buf, size_t len);

static inline struct bpu_user *bpu_get_user(struct bpu_fc *fc)
{
	if (fc == NULL)
		return NULL;

	if (fc->user == NULL)
		return NULL;

	return (struct bpu_user *)(*(fc->user));
}

static inline struct bpu_fc_group *bpu_get_fc_group(struct bpu_fc *fc)
{
	if (fc == NULL)
		return NULL;

	if (fc->g_id == NULL)
		return NULL;

	return (struct bpu_fc_group *)container_of(fc->g_id, struct bpu_fc_group, id);
}

static inline uint32_t bpu_group_id(uint32_t raw_id)
{
	return raw_id & 0xFFFFu;
}

static inline uint32_t bpu_group_user(uint32_t raw_id)
{
	return raw_id >> 16u;
}

/* register core apis */
int32_t bpu_core_register(struct bpu_core *core);
void bpu_core_unregister(struct bpu_core *core);
int32_t bpu_write_fc_to_core(struct bpu_core *core,
		struct bpu_fc *bpu_fc, uint32_t offpos);

/* statusis apis */
uint32_t bpu_ratio(struct bpu *bpu);
uint32_t bpu_fc_group_ratio(struct bpu_fc_group *group);
uint32_t bpu_user_ratio(struct bpu_user *user);

/* sched apis*/
int32_t bpu_sched_start(struct bpu *bpu);
int32_t bpu_sched_stop(struct bpu *bpu);
void bpu_core_update(struct bpu_core *core, struct bpu_fc *fc);
uint32_t bpu_core_ratio(struct bpu_core *core);
void bpu_sched_seed_update(void);

/* ctrl apis */
int32_t bpu_stat_reset(struct bpu *bpu);

/* sys apis */
int32_t bpu_sys_system_init(struct bpu *bpu);
int32_t bpu_core_create_sys(struct bpu_core *core);
void bpu_core_discard_sys(const struct bpu_core *core);

#endif
