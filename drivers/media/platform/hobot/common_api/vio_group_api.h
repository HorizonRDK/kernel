/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __VIO_GROUP_API_H__
#define __VIO_GROUP_API_H__

#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/wait.h>

#include "vio_framemgr.h"

#define MAX_SUB_DEVICE  8
#define MAX_SHADOW_NUM 4

#define GROUP_ID_SIF_OUT	0
#define GROUP_ID_SIF_IN		1
#define GROUP_ID_IPU		2
#define GROUP_ID_PYM		3
#define GROUP_ID_NUMBER		4

#define X3_IAR_INTERFACE
#define SET_CPU_AFFINITY

enum vio_group_task_state {
	VIO_GTASK_START,
	VIO_GTASK_REQUEST_STOP,
	VIO_GTASK_SHOT,
	VIO_GTASK_SHOT_STOP,
};

enum vio_group_state {
	VIO_GROUP_OPEN,
	VIO_GROUP_INIT,
	VIO_GROUP_START,
	VIO_GROUP_FORCE_STOP,
	VIO_GROUP_OTF_INPUT,
	VIO_GROUP_OTF_OUTPUT,
	VIO_GROUP_DMA_INPUT,
	VIO_GROUP_DMA_OUTPUT,
	VIO_GROUP_LEADER,
};

struct vio_group_task{
	struct task_struct		*task;
	struct kthread_worker	worker;
	unsigned long				state;
	atomic_t			refcount;
	struct semaphore    hw_resource;
	u32 id;
};

struct vio_group{
	spinlock_t 			slock;
	void *sub_ctx[MAX_SUB_DEVICE];
	struct frame_id frameid;
	unsigned long state;
	u32 id;
	u32 instance;
	u32 output_flag;
	bool get_timestamps;
	bool leader;
	struct vio_group		*next;
	struct vio_group		*prev;
	struct vio_group		*head;
	struct vio_chain		*chain;
	struct vio_group_task *gtask;
	void (*frame_work)(struct vio_group *group);
};

struct vio_video_ctx{
	wait_queue_head_t		done_wq;
	struct vio_framemgr 	framemgr;
	struct vio_group		*group;
	unsigned long			state;

	u32 id;
	u32 event;
	bool leader;
};

struct vio_chain{
	struct vio_group group[GROUP_ID_NUMBER];
	unsigned long state;
};

int vio_group_task_start(struct vio_group_task *group_task);
int vio_group_task_stop(struct vio_group_task *group_task);
void vio_group_start_trigger(struct vio_group_task *group_task, struct vio_frame *frame);
struct vio_group *vio_get_chain_group(int instance, u32 group_id);
int vio_bind_chain_groups(struct vio_group *src_group, struct vio_group *dts_group);
int vio_init_chain(int instance);
void vio_bind_group_done(int instance);
void vio_get_frame_id(struct vio_group *group);
int vio_group_init_mp(u32 group_id);

#ifdef X3_IAR_INTERFACE
extern u32 ipu_get_iar_display_type(void);
extern int32_t ipu_set_display_addr(u32 yaddr, u32 caddr);
#endif

#endif
