#ifndef __VIO_GROUP_API_H__
#define __VIO_GROUP_API_H__

#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/wait.h>

#include "vio_framemgr.h"

enum vio_group_task_state {
	VIO_GTASK_START,
	VIO_GTASK_REQUEST_STOP,
	VIO_GTASK_SHOT,
	VIO_GTASK_SHOT_STOP,
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
	struct vio_video_ctx *sub_ctx[5];
	unsigned long state;
	u32 instance;
	struct vio_group		*next;
	struct vio_group		*prev;
	struct vio_group		*head;
};

struct vio_video_ctx{
	wait_queue_head_t		done_wq;
	struct vio_framemgr 	framemgr;
	struct vio_group		*group;
	struct semaphore 		smp_resource;

	u32 id;
	unsigned long			state;
	bool leader;
};

struct vio_chain{
	struct vio_group sif_out;
	struct vio_group sif_in;
	struct vio_group ipu;
	struct vio_group pym;
};

int vio_group_task_start(struct vio_group_task *group_task);
int vio_group_task_stop(struct vio_group_task *group_task);
void vio_group_start_trigger(struct vio_group_task *group_task, struct vio_frame *frame);

#endif
