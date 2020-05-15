/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/delay.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include "vio_group_api.h"

static struct vio_core iscore;

isp_callback sif_isp_ctx_sync;
EXPORT_SYMBOL(sif_isp_ctx_sync);

void isp_register_callback(isp_callback func)
{
	sif_isp_ctx_sync = func;
}
EXPORT_SYMBOL(isp_register_callback);

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
#ifdef SET_CPU_AFFINITY
	ret = set_cpus_allowed_ptr(group_task->task, cpumask_of(1));
#endif

	sema_init(&group_task->hw_resource, 1);

	set_bit(VIO_GTASK_START, &group_task->state);
p_work:
	atomic_inc(&group_task->refcount);	
p_err:
	return ret;
}
EXPORT_SYMBOL(vio_group_task_start);

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
EXPORT_SYMBOL(vio_group_task_stop);

void vio_group_start_trigger(struct vio_group *group, struct vio_frame *frame)
{
	struct vio_group_task *group_task;

	group_task = group->gtask;
	atomic_inc(&group->rcount);
	kthread_queue_work(&group_task->worker, &frame->work);
}
EXPORT_SYMBOL(vio_group_start_trigger);

int vio_group_init_mp(u32 group_id)
{
	struct vio_group *group = NULL;
	u8 i;

	if (group_id > GROUP_ID_NUMBER) {
		vio_err("[%s]wrong group_id (%d)\n", __func__, group_id);
		return -EINVAL;
	}

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		group = &iscore.chain[i].group[group_id];
		spin_lock_init(&group->slock);
	}
	return 0;
}
EXPORT_SYMBOL(vio_group_init_mp);

struct vio_group *vio_get_chain_group(int instance, u32 group_id)
{
	struct vio_group *group = NULL;
	struct vio_chain *ischain;

	if (instance < 0 || instance >= VIO_MAX_STREAM) {
		vio_err("can't support instance %d\n", instance);
		return NULL;
	}

	vio_init_chain(instance);

	if (group_id > GROUP_ID_NUMBER) {
		vio_err("[%s]wrong group id (%d)\n", __func__, group_id);
	}

	ischain = &iscore.chain[instance];
	group = &ischain->group[group_id];

	return group;
}
EXPORT_SYMBOL(vio_get_chain_group);

void vio_group_init(struct vio_group *group)
{
	int i = 0;

	BUG_ON(!group);

	group->state = 0;
	group->leader = 0;
	group->gtask = NULL;
	group->frame_work = NULL;
	group->next = NULL;
	group->prev = group;
	group->head = group;
	group->frameid.frame_id = 0;
	atomic_set(&group->rcount, 0);
	for(i = 0; i < MAX_SUB_DEVICE; i++)
		group->sub_ctx[i] = NULL;
}
EXPORT_SYMBOL(vio_group_init);

int vio_init_chain(int instance)
{
	struct vio_chain *ischain;
	struct vio_group *group;
	int i = 0;

	ischain = &iscore.chain[instance];

	for (i = 0; i < GROUP_ID_NUMBER; i++) {
		group = &ischain->group[i];
		spin_lock(&group->slock);
		if (!test_bit(VIO_GROUP_INIT, &group->state)) {
			vio_group_init(group);
			group->chain = ischain;
			group->instance= instance;
			group->id = i;
			set_bit(VIO_GROUP_INIT, &group->state);
		}
		spin_unlock(&group->slock);
	}

	return 0;
}
EXPORT_SYMBOL(vio_init_chain);

int vio_bind_chain_groups(struct vio_group *src_group, struct vio_group *dts_group)
{
	struct vio_group *group;

	BUG_ON(!src_group);
	BUG_ON(!dts_group);

	src_group->next = dts_group;
	dts_group->prev = src_group;

	group = src_group;
	while (group != group->prev) {
		group = group->prev;
	}
	dts_group->head = group;

	if (src_group->id == GROUP_ID_IPU && src_group->get_timestamps)
		dts_group->get_timestamps = true;

	return 0;
}
EXPORT_SYMBOL(vio_bind_chain_groups);

void vio_bind_group_done(int instance)
{
	int i = 0;
	char stream[64];
	int offset = 0;
	struct vio_chain *ischain;
	struct vio_group *group;
	struct vio_group *leader_group;

	ischain = &iscore.chain[instance];
	for (i = 0; i < GROUP_ID_NUMBER; i++) {
		group = &ischain->group[i];
		if (test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)) {
			group->get_timestamps = true;
			break;
		}
	}

	for (i = 0; i < GROUP_ID_NUMBER; i++) {
		group = &ischain->group[i];
		if (test_bit(VIO_GROUP_DMA_INPUT, &group->state)) {
			group->leader = true;
			snprintf(&stream[offset], sizeof(stream) - offset,
					"=>G%d", group->id);
			offset = strlen(stream);
		} else if (test_bit(VIO_GROUP_OTF_INPUT, &group->state)) {
			vio_bind_chain_groups(&ischain->group[i - 1], group);
			snprintf(&stream[offset], sizeof(stream) - offset,
					"->G%d", group->id);
			offset = strlen(stream);
		}
	}

	for (i = 0; i < GROUP_ID_NUMBER; i++) {
		group = &ischain->group[i];
		if (group->leader) {
			break;
		} else if (i >= GROUP_ID_IPU) {
			if (test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)) {
				group->leader = true;
				group->head = group;
				leader_group = group;
				while (group->next) {
					group = group->next;
					group->head = leader_group;
				}
				break;
			}
		}
	}

	vio_info("Stream%d path: G0%s\n", group->instance, stream);
}
EXPORT_SYMBOL(vio_bind_group_done);

void vio_get_frame_id(struct vio_group *group)
{
	struct vio_chain *ischain;
	struct vio_group *sif_group;

	ischain = group->chain;
	sif_group = &ischain->group[GROUP_ID_SIF_OUT];

	memcpy(&group->frameid, &sif_group->frameid, sizeof(struct frame_id));

}
EXPORT_SYMBOL(vio_get_frame_id);

void vio_reset_module(u32 module)
{
	u32 cfg = 0;
	u32 cnt = 20;
	u32 value = 0;
	u32 bit = 0;
	u32 reset = 0;

	if (module == GROUP_ID_IPU) {
		bit = IPU0_IDLE;
		reset = IPU0_RST;
	} else if (module == GROUP_ID_PYM) {
		bit = PYM_IDLE;
		reset = PYM_RST;
	}

	cfg = ips_get_bus_ctrl() | bit;
	ips_set_bus_ctrl(cfg);

	while(1) {
		value = ips_get_bus_status();
		if (value & bit << 16)
			break;

		msleep(5);
		cnt--;
		if (cnt == 0) {
			vio_info("%s timeout\n", __func__);
			break;
		}
	}

	ips_set_module_reset(reset);
	if (module == GROUP_ID_PYM)
		ips_set_module_reset(IPU_PYM_RST);

	cfg = ips_get_bus_ctrl() & ~bit;
	ips_set_bus_ctrl(cfg);
}
EXPORT_SYMBOL(vio_reset_module);

void vio_group_done(struct vio_group *group)
{
	struct vio_group_task *group_task;
	struct vio_group *group_leader;

	group_leader = group->head;
	group_task = group_leader->gtask;

	if (group->next == NULL)
		group_leader->sema_flag |= 1 << 0;

	if (group->head == group)
		group_leader->sema_flag |= 1 << 1;

	if(group_leader->sema_flag == 0x3) {
		up(&group_task->hw_resource);
		group_leader->sema_flag = 0;
		vio_dbg("[S%d][G%d]up hw_resource G%d\n", group->instance, group->id,
		group_leader->id);
	}
}
EXPORT_SYMBOL(vio_group_done);

void vio_dwe_clk_enable(void)
{
	struct vio_core *core;
	bool enable = true;

	core = &iscore;
	if (atomic_read(&core->rsccount) == 0) {
		if (sif_mclk_freq)
			vio_set_clk_rate("sif_mclk", sif_mclk_freq);
		ips_set_clk_ctrl(GDC0_CLOCK_GATE, enable);
		ips_set_clk_ctrl(GDC1_CLOCK_GATE, enable);
		ips_set_clk_ctrl(DWE0_CLOCK_GATE, enable);
		vio_info("%s done", __func__);
	}
	atomic_inc(&core->rsccount);
}
EXPORT_SYMBOL(vio_dwe_clk_enable);

void vio_dwe_clk_disable(void)
{
	struct vio_core *core;
	bool enable = false;

	core = &iscore;
	if (atomic_dec_return(&core->rsccount) == 0) {
		ips_set_clk_ctrl(GDC0_CLOCK_GATE, enable);
		ips_set_clk_ctrl(GDC1_CLOCK_GATE, enable);
		ips_set_clk_ctrl(DWE0_CLOCK_GATE, enable);
		vio_info("%s done", __func__);
	}
}
EXPORT_SYMBOL(vio_dwe_clk_disable);
