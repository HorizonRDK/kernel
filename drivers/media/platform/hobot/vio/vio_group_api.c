#include "vio_group_api.h"

int vio_group_task_start(struct vio_group_task *group_task)
{
	int ret = 0;
	char name[30];

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

