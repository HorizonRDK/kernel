/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>

#include <linux/bug.h>

#include "vio_framemgr.h"
#include "vio_group_api.h"

int frame_fcount(struct vio_frame *frame, void *data)
{
	return frame->fcount - (u32)(ulong)data;
}

int put_frame(struct vio_framemgr *this, struct vio_frame *frame,
			enum vio_frame_state state)
{
	if (state == FS_INVALID)
		return -EINVAL;
	if (!frame) {
		vio_err("invalid frame");
		return -EFAULT;
	}
	frame->state = state;
	list_add_tail(&frame->list, &this->queued_list[state]);
	this->queued_count[state]++;
#ifdef TRACE_FRAME
	print_frame_queue(this, state);
#endif
	return 0;
}

struct vio_frame *get_frame(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	struct vio_frame *frame;
	if (state == FS_INVALID)
		return NULL;
	if (!this->queued_count[state])
		return NULL;
	frame = list_first_entry(&this->queued_list[state], struct vio_frame, list);
	list_del(&frame->list);
	this->queued_count[state]--;
	frame->state = FS_INVALID;
	return frame;
}

int trans_frame(struct vio_framemgr *this, struct vio_frame *frame,
			enum vio_frame_state state)
{
	if (!frame) {
		vio_err("invalid frame");
		return -EFAULT;
	}
	if ((frame->state == FS_INVALID) || (state == FS_INVALID))
		return -EINVAL;
	if (!this->queued_count[frame->state]) {
		vio_err("%s frame queue is empty (0x%08x)", frame_state_name[frame->state],
							this->id);
		return -EINVAL;
	}
	list_del(&frame->list);
	this->queued_count[frame->state]--;
	//if (state == FS_PROCESS && (!(this->id & FRAMEMGR_ID_HW)))
	//	frame->bak_flag = frame->out_flag;
	return put_frame(this, frame, state);
}

struct vio_frame *peek_frame(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	if (state == FS_INVALID)
		return NULL;
	if (!this->queued_count[state])
		return NULL;
	return list_first_entry(&this->queued_list[state], struct vio_frame, list);
}

struct vio_frame *peek_frame_tail(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	if (state == FS_INVALID)
		return NULL;
	if (!this->queued_count[state])
		return NULL;
	return list_last_entry(&this->queued_list[state], struct vio_frame, list);
}

struct vio_frame *find_frame(struct vio_framemgr *this,
			     enum vio_frame_state state,
			     int (*fn)(struct vio_frame *, void *), void *data)
{
	struct vio_frame *frame;
	if (state == FS_INVALID)
		return NULL;
	if (!this->queued_count[state]) {
		vio_err("%s frame queue is empty (0x%08x)", frame_state_name[state],
							this->id);
		return NULL;
	}
	list_for_each_entry(frame, &this->queued_list[state], list) {
		if (!fn(frame, data))
			return frame;
	}
	return NULL;
}

void print_frame_queue(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	struct vio_frame *frame, *temp;
	if (!(TRACE_ID & this->id))
			return;
	vio_info("[FRM] %s(0x%08x, %d) :", frame_state_name[state],
					this->id, this->queued_count[state]);
	list_for_each_entry_safe(frame, temp, &this->queued_list[state], list)
		vio_cont("%d[%d]->", frame->index, frame->fcount);

	vio_cont("X\n");
}

void print_frame_info_queue(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	unsigned long long when[MAX_FRAME_INFO];
	unsigned long usec[MAX_FRAME_INFO];
	struct vio_frame *frame, *temp;
	if (!(TRACE_ID & this->id))
			return;
	vio_info("[FRM_INFO] %s(0x%08x, %d) :", hw_frame_state_name[state],
					this->id, this->queued_count[state]);
	list_for_each_entry_safe(frame, temp, &this->queued_list[state], list) {
		when[INFO_FRAME_START]    = frame->frame_info[INFO_FRAME_START].when;
		when[INFO_CONFIG_LOCK]    = frame->frame_info[INFO_CONFIG_LOCK].when;
		when[INFO_FRAME_END_PROC] = frame->frame_info[INFO_FRAME_END_PROC].when;
		usec[INFO_FRAME_START]    = do_div(when[INFO_FRAME_START], NSEC_PER_SEC);
		usec[INFO_CONFIG_LOCK]    = do_div(when[INFO_CONFIG_LOCK], NSEC_PER_SEC);
		usec[INFO_FRAME_END_PROC] = do_div(when[INFO_FRAME_END_PROC], NSEC_PER_SEC);
		vio_cont("%d[%d]([%5lu.%06lu],[%5lu.%06lu],[%5lu.%06lu])->",
			frame->index, frame->fcount,
			(unsigned long)when[INFO_FRAME_START],    usec[INFO_FRAME_START] / NSEC_PER_USEC,
			(unsigned long)when[INFO_CONFIG_LOCK],    usec[INFO_CONFIG_LOCK] / NSEC_PER_USEC,
			(unsigned long)when[INFO_FRAME_END_PROC], usec[INFO_FRAME_END_PROC] / NSEC_PER_USEC);
	}
	vio_cont("X\n");
}

void frame_work_function(struct kthread_work *work)
{
	struct vio_frame *frame;
	struct vio_group *group;
	struct vio_group *leader;
	struct vio_group_task *gtask;
	bool try_rdown = false;
	int ret = 0;

	frame = container_of(work, struct vio_frame, work);
	leader = (struct vio_group *) frame->data;
	gtask = leader->gtask;

	set_bit(VIO_GTASK_SHOT, &gtask->state);

	if (unlikely(test_bit(VIO_GTASK_REQUEST_STOP, &gtask->state))) {
		vio_err(" cancel by gstop0");
		goto p_err_ignore;
	}

	ret = down_interruptible(&gtask->hw_resource);
	if (ret) {
		vio_err(" down fail(%d)", ret);
		goto p_err_ignore;
	}

	try_rdown = true;

	if (unlikely(test_bit(VIO_GTASK_SHOT_STOP, &gtask->state))) {
		vio_err(" cancel by gstop1");
		goto p_err_ignore;
	}

	group = leader;
	while(group->next){
		group = group->next;
		vio_dbg("%s #1\n", __func__);
		if(group->frame_work)
			group->frame_work(group);
	}
	vio_dbg("%s #2\n", __func__);

	leader->frame_work(leader);
	clear_bit(VIO_GTASK_SHOT, &gtask->state);

	return;

p_err_ignore:
	if (try_rdown)
		up(&gtask->hw_resource);

	clear_bit(VIO_GTASK_SHOT, &gtask->state);

	return;
}

int frame_manager_open(struct vio_framemgr *this, u32 buffers)
{
	u32 i;
	unsigned long flag;
	spin_lock_init(&this->slock);
	/*
	 * We already have frames allocated, so we should free them first.
	 * reqbufs(n) could be called multiple times from userspace after
	 * each video node was opened.
	 */
	if (this->frames)
		kfree(this->frames);
	this->frames = kzalloc(sizeof(struct vio_frame) * buffers, GFP_KERNEL);
	if (!this->frames) {
		vio_err("failed to allocate frames");
		return -ENOMEM;
	}
	spin_lock_irqsave(&this->slock, flag);
	this->num_frames = buffers;
	for (i = 0; i < NR_FRAME_STATE; i++) {
		this->queued_count[i] = 0;
		INIT_LIST_HEAD(&this->queued_list[i]);
	}
	for (i = 0; i < buffers; ++i) {
		this->frames[i].index = i;
		put_frame(this, &this->frames[i], FS_FREE);
		kthread_init_work(&this->frames[i].work, frame_work_function);
	}

	spin_unlock_irqrestore(&this->slock, flag);
	return 0;
}

int frame_manager_close(struct vio_framemgr *this)
{
	u32 i;
	unsigned long flag;
	spin_lock_irqsave(&this->slock, flag);
	if (this->frames) {
		kfree(this->frames);
		this->frames = NULL;
	}
	this->num_frames = 0;
	for (i = 0; i < NR_FRAME_STATE; i++) {
		this->queued_count[i] = 0;
		INIT_LIST_HEAD(&this->queued_list[i]);
	}
	spin_unlock_irqrestore(&this->slock, flag);
	return 0;
}

int frame_manager_flush(struct vio_framemgr *this)
{
	unsigned long flag;
	struct vio_frame *frame, *temp;
	enum vio_frame_state i;
	spin_lock_irqsave(&this->slock, flag);
	for (i = FS_REQUEST; i < FS_INVALID; i++) {
		list_for_each_entry_safe(frame, temp, &this->queued_list[i], list)
			trans_frame(this, frame, FS_FREE);
	}
	spin_unlock_irqrestore(&this->slock, flag);
	BUG_ON(this->queued_count[FS_FREE] != this->num_frames);
	return 0;
}

void frame_manager_print_queues(struct vio_framemgr *this)
{
	int i;
	for (i = 0; i < NR_FRAME_STATE; i++)
		print_frame_queue(this, (enum vio_frame_state)i);
}

void frame_manager_print_info_queues(struct vio_framemgr *this)
{
	int i;
	for (i = 0; i < NR_FRAME_STATE; i++)
		print_frame_info_queue(this, (enum vio_frame_state)i);
}
