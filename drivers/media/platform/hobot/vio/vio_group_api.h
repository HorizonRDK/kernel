#ifndef __VIO_GROUP_API_H__
#define __VIO_GROUP_API_H__

#include <linux/kthread.h>
#include <linux/semaphore.h>
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

int vio_group_task_start(struct vio_group_task *group_task);
int vio_group_task_stop(struct vio_group_task *group_task);
void vio_group_start_trigger(struct vio_group_task *group_task, struct vio_frame *frame);

#endif
