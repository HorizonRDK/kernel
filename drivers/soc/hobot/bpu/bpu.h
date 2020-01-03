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

#define TIME_VAL(ptimeval) ((ptimeval)->tv_sec * 1000000 \
				+ (ptimeval)->tv_usec)

#define TIME_INTERVAL(ptimeval_old, ptimeval_new)	\
				(TIME_VAL(ptimeval_new) - TIME_VAL(ptimeval_old))

#define GROUP_USER(id) (id >> 16)
#define GROUP_ID(id) (id & 0xFFFF)

/* must be a power of 2 */
#define BPU_CORE_RECORE_NUM		64

struct bpu_core;

union bpu_ioctl_arg {
	struct bpu_group group;
	__u32 ratio;
	__u16 cap;
	__u16 reset;
};

struct bpu_fc {
	struct user_bpu_fc info;

	/* communicate id with bpu hw */
	uint32_t hw_id;

	/* real fc data*/
	void *fc_data;

	/* group id */
	unsigned int *g_id;

	/* identifier for diff file user */
	void **user;

	/* index in hw core buffer */
	unsigned int index;

	/*
	 * start: time set to hw buffer
	 * end: time done by bpu core
	 */
	struct timeval start_point;
	struct timeval end_point;
};

struct bpu_fc_group {
	struct list_head node;
	unsigned int id;
	int proportion;

	/* when group is busy, firt to buffer fc fifo */
	DECLARE_KFIFO_PTR(buffer_fc_fifo, struct bpu_fc);
	/* to protect in fifo*/
	spinlock_t spin_lock;

	/* run time in statistical period */
	uint64_t p_run_time;
};

struct bpu_user {
	struct list_head node;
	unsigned int id;

	wait_queue_head_t poll_wait;
	/* to protect in fifo*/
	spinlock_t spin_lock;

	/* to protect out fifo*/
	struct mutex mutex_lock;

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

	wait_queue_head_t poll_wait;

	/* list to store bpu cores*/
	struct list_head core_list;

	/* list to store user */
	struct list_head user_list;
	/* list to store user */
	struct list_head group_list;
	int busy_thres;

	/* use the value to adjust sched time */
	int sched_seed;
	int stat_reset_count;

	struct bus_type *bus;

	int ratio;
};

/* create bpu_fc from user fc info*/
int bpu_fc_create_from_user(struct bpu_fc *fc,
		struct user_bpu_fc *user_fc, const void *data);
/* mainly clear fc data in bpu_fc*/
void bpu_fc_clear(struct bpu_fc *fc);

int bpu_write_with_user(struct bpu_core *core,
			struct bpu_user *user,
			const char __user *buf, size_t len);
int bpu_read_with_user(struct bpu_core *core,
			struct bpu_user *user,
			char __user *buf, size_t len);

int bpu_fc_bind_user(struct bpu_fc *fc, struct bpu_user *user);
int bpu_fc_bind_group(struct bpu_fc *fc, uint32_t group_id);

static inline struct bpu_user *bpu_get_user(struct bpu_fc *fc)
{
	if (!fc)
		return NULL;

	if (!fc->user)
		return NULL;

	return (struct bpu_user *)(*(fc->user));
}

static inline struct bpu_fc_group *bpu_get_fc_group(struct bpu_fc *fc)
{
	if (!fc)
		return NULL;

	if (!fc->g_id)
		return NULL;

	return container_of(fc->g_id, struct bpu_fc_group, id);
}

/* register core apis */
int bpu_core_register(struct bpu_core *core);
void bpu_core_unregister(struct bpu_core *core);
int bpu_write_fc_to_core(struct bpu_core *core, struct bpu_fc *bpu_fc);

/* fc group apis */
struct bpu_fc_group *bpu_create_group(uint32_t group_id);
void bpu_delete_group(uint32_t group_id);

/* statusis apis */
int bpu_ratio(struct bpu *bpu);
int bpu_fc_group_ratio(struct bpu_fc_group *group);
int bpu_user_ratio(struct bpu_user *user);

/* sched apis*/
int bpu_sched_start(struct bpu *bpu);
int bpu_sched_stop(struct bpu *bpu);
struct bpu_core *bpu_sched_suit_core(struct bpu *bpu, uint64_t core_mask);
int bpu_core_update(struct bpu_core *core, struct bpu_fc *fc);
void bpu_sched_seed_update(void);
int bpu_ratio(struct bpu *bpu);
int bpu_core_ratio(struct bpu_core *core);

/* ctrl apis */
int bpu_stat_reset(struct bpu *bpu);
int bpu_core_reset(struct bpu_core *core);
void flush_fc_area(void *start, size_t size);

/* sys apis */
int bpu_sys_system_init(struct bpu *bpu);
int bpu_core_create_sys(struct bpu_core *core);
int bpu_core_discard_sys(struct bpu_core *core);

#endif
