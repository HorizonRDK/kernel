/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include "vio_group_api.h"

static struct vio_chain chain[VIO_MAX_STREAM];
int vio_group_task_start(struct vio_group_task *group_task)
{
	int ret = 0;
	char name[30];
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	BUG_ON(!group_task);

	if (test_bit(VIO_GTASK_START, &group_task->state))
		goto p_work;

	kthread_init_worker(&group_task->worker);
	snprintf(name, sizeof(name), "vio_gw%d", group_task->id);
	group_task->task = kthread_run(kthread_worker_fn, &group_task->worker, name);
	if (IS_ERR(group_task->task)) {
		vio_err("failed to create buffer task, err(%ld)\n",PTR_ERR(group_task->task));
		ret = PTR_ERR(group_task->task);
		goto p_err;
	}

	ret = sched_setscheduler_nocheck(group_task->task, SCHED_FIFO, &param);
	if (ret) {
		vio_err("sched_setscheduler_nocheck is fail(%d)", ret);
		goto p_err;
	}

	sema_init(&group_task->hw_resource, 1);

	set_bit(VIO_GTASK_START, &group_task->state);
p_work:
	atomic_inc(&group_task->refcount);	
p_err:
	return ret;
}

int vio_group_task_stop(struct vio_group_task *group_task)
{
	int ret = 0;
	int refcount;

	BUG_ON(!group_task);
	if (!test_bit(VIO_GTASK_START, &group_task->state)) {
		vio_err("gtask(%d) is not started", group_task->id);
		ret = -EINVAL;
		goto p_err;
	}

	if (IS_ERR_OR_NULL(group_task->task)) {
		vio_err("task of group_task(%d) is invalid(%p)", group_task->id, group_task->task);
		ret = -EINVAL;
		goto p_err;
	}

	refcount = atomic_dec_return(&group_task->refcount);
	if (refcount > 0)
		goto p_err;

	set_bit(VIO_GTASK_REQUEST_STOP, &group_task->state);

	if(test_bit(VIO_GTASK_SHOT, &group_task->state)){
		set_bit(VIO_GTASK_SHOT_STOP, &group_task->state);
		up(&group_task->hw_resource);
	}

	kthread_stop(group_task->task);

	clear_bit(VIO_GTASK_SHOT_STOP, &group_task->state);
	clear_bit(VIO_GTASK_REQUEST_STOP, &group_task->state);
	clear_bit(VIO_GTASK_START, &group_task->state);

p_err:
	return ret;

}

void vio_group_start_trigger(struct vio_group_task *group_task, struct vio_frame *frame)
{
	kthread_queue_work(&group_task->worker, &frame->work);
}

struct vio_group *vio_get_chain_group(int instance, u32 group_id)
{
	struct vio_group *group = NULL;
	struct vio_chain *ischain;

	vio_init_chain(instance);

	if(group_id > GROUP_ID_NUMBER){
		vio_err("[%s]wrong group id (%d)\n", __func__, group_id);
	}

	ischain = &chain[instance];
	group = &ischain->group[group_id];

	return group;
}

void vio_group_init(struct vio_group *group)
{
	BUG_ON(!group);

	group->state = 0;
	group->leader = 0;
	group->gtask = NULL;
	group->frame_work = NULL;
	group->next = NULL;
	group->prev = group;
}

int vio_init_chain(int instance)
{
	struct vio_chain *ischain;
	struct vio_group *group;
	int i = 0;

	ischain = &chain[instance];

	for(i = 0; i < GROUP_ID_NUMBER; i++){
		group = &ischain->group[i];
		if(!test_bit(VIO_GROUP_INIT, &group->state)){
			vio_group_init(group);
			group->chain = ischain;
			group->instance= instance;
			set_bit(VIO_GROUP_INIT, &group->state);
		}
	}

	return 0;
}

int vio_bind_chain_groups(struct vio_group *src_group, struct vio_group *dts_group)
{
	BUG_ON(!src_group);
	BUG_ON(!dts_group);

	src_group->next = dts_group;
	dts_group->prev = src_group;

	return 0;
}

void vio_bind_group_done(int instance)
{
	int i = 0;
	struct vio_chain *ischain;
	struct vio_group *group;

	ischain = &chain[instance];
	for(i = 0; i < GROUP_ID_NUMBER; i++){
		group = &ischain->group[i];
		if(test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)){
			group->get_timestamps = true;
			break;
		}
	}

	for(i = 0; i < GROUP_ID_NUMBER; i++){
		group = &ischain->group[i];
		if(test_bit(VIO_GROUP_DMA_INPUT, &group->state)){
			group->leader = true;
		} else if (test_bit(VIO_GROUP_OTF_INPUT, &group->state)) {
			vio_bind_chain_groups(&ischain->group[i - 1], group);
		}
	}

	for(i = 0; i < GROUP_ID_NUMBER; i++){
		group = &ischain->group[i];
		if(group->leader){
			break;
		}else if(i >= GROUP_ID_IPU){
			if(test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)){
				group->leader = true;
				break;
			}
		}
	}
}

void vio_get_frame_id(struct vio_group *group)
{
	struct vio_chain *ischain;
	struct vio_group *sif_group;

	ischain = group->chain;
	sif_group = &ischain->group[GROUP_ID_SIF_OUT];

	memcpy(&group->frameid, &sif_group->frameid, sizeof(struct frame_id));

}
