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
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <linux/poll.h>

#include "hobot_dev_sif.h"
#include "hobot_dev_ips.h"
#include "sif_hw_api.h"

#define MODULE_NAME "X3 SIF"

static int mismatch_limit = 1;
module_param(mismatch_limit, int, 0644);

extern struct class *vps_class;
typedef int (*isp_callback)(int);
isp_callback sif_isp_ctx_sync;
int sif_video_streamoff(struct sif_video_ctx *sif_ctx);

void isp_register_callback(isp_callback func)
{
	sif_isp_ctx_sync = func;
}
EXPORT_SYMBOL(isp_register_callback);

static int x3_sif_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_sif_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_sif_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_sif_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x3_sif_pm_ops = {
	.suspend = x3_sif_suspend,
	.resume = x3_sif_resume,
	.runtime_suspend = x3_sif_runtime_suspend,
	.runtime_resume = x3_sif_runtime_resume,
};

void sif_config_rdma_cfg(struct x3_sif_dev *sif, u8 index,
			struct frame_info *frameinfo)
{
	sif_set_rdma_enable(sif->base_reg, index, true);
	sif_set_rdma_buf_addr(sif->base_reg, index, frameinfo->addr[index]);
}

void sif_read_frame_work(struct vio_group *group)
{
	unsigned long flags;
	u32 instance = 0;
	struct sif_video_ctx *ctx;
	struct x3_sif_dev *sif;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct frame_info *frameinfo;

	instance = group->instance;
	ctx = group->sub_ctx[0];
	sif = ctx->sif_dev;

	atomic_set(&sif->instance, instance);

	if (sif_isp_ctx_sync != NULL) {
		(*sif_isp_ctx_sync)(instance);
	}

	framemgr = &ctx->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame) {
		frameinfo = &frame->frameinfo;
		sif_config_rdma_cfg(sif, 0, frameinfo);

		if (ctx->dol_num > 1) {
			sif_config_rdma_cfg(sif, 1, frameinfo);
		}

		if (ctx->dol_num > 2) {
			sif_config_rdma_cfg(sif, 2, frameinfo);
		}
		sif_set_rdma_trigger(sif->base_reg, 1);
		trans_frame(framemgr, frame, FS_PROCESS);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
}


void sif_write_frame_work(struct vio_group *group)
{
	u32 instance = 0;
	u32 buf_index = 0;
	u32 mux_index = 0;
	u32 yuv_format = 0;
	unsigned long flags;
	struct sif_video_ctx *ctx;
	struct x3_sif_dev *sif;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;

	instance = group->instance;
	ctx = group->sub_ctx[0];
	sif = ctx->sif_dev;
	mux_index = ctx->mux_index;

	atomic_set(&sif->instance, instance);

	framemgr = &ctx->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame) {
		buf_index = ctx->bufcount % 4;
		yuv_format = (frame->frameinfo.format == HW_FORMAT_YUV422);
		sif_set_wdma_buf_addr(sif->base_reg, mux_index, buf_index,
				      frame->frameinfo.addr[0]);
		sif_transfer_ddr_owner(sif->base_reg, mux_index, buf_index);

		if (yuv_format || ctx->dol_num > 1) {
			sif_set_wdma_buf_addr(sif->base_reg, mux_index + 1,
					      buf_index, frame->frameinfo.addr[1]);
			sif_transfer_ddr_owner(sif->base_reg, mux_index + 1,
					       buf_index);
		}

		if(ctx->dol_num > 2){
			sif_set_wdma_buf_addr(sif->base_reg, mux_index + 2,
					      buf_index, frame->frameinfo.addr[2]);
			sif_transfer_ddr_owner(sif->base_reg, mux_index + 2,
					       buf_index);
		}
		trans_frame(framemgr, frame, FS_PROCESS);
		ctx->bufcount++;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
}


static int x3_sif_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	int minor = 0;
	struct sif_video_ctx *sif_ctx;
	struct x3_sif_dev *sif;

	minor = MINOR(inode->i_rdev);

	sif = container_of(inode->i_cdev, struct x3_sif_dev, cdev);
	sif_ctx = kzalloc(sizeof(struct sif_video_ctx), GFP_KERNEL);
	if (sif_ctx == NULL) {
		vio_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	init_waitqueue_head(&sif_ctx->done_wq);
	sif_ctx->id = minor;
	sif_ctx->sif_dev = sif;
	file->private_data = sif_ctx;
	sif_ctx->state = BIT(VIO_VIDEO_OPEN);

	if (atomic_read(&sif->open_cnt) == 0) {
		vio_clk_enable("sif_mclk");
	}

	atomic_inc(&sif->open_cnt);
	vio_info("SIF open node %d\n", minor);
p_err:
	return ret;
}

static ssize_t x3_sif_write(struct file *file, const char __user * buf,
			     size_t count, loff_t * ppos)
{
	return 0;
}

static ssize_t x3_sif_read(struct file *file, char __user * buf, size_t size,
			    loff_t * ppos)
{
	return 0;
}

static u32 x3_sif_poll(struct file *file, struct poll_table_struct *wait)
{
	int ret = 0;
	struct sif_video_ctx *sif_ctx;

	sif_ctx = file->private_data;

	poll_wait(file, &sif_ctx->done_wq, wait);

	if(sif_ctx->event == VIO_FRAME_DONE)
		ret = POLLIN;
	else if(sif_ctx->event == VIO_FRAME_NDONE)
		ret = POLLERR;

	sif_ctx->event = 0;

	return ret;
}

static int x3_sif_close(struct inode *inode, struct file *file)
{
	struct sif_video_ctx *sif_ctx;
	struct vio_group *group;
	struct x3_sif_dev *sif;

	sif_ctx = file->private_data;
	group = sif_ctx->group;
	sif = sif_ctx->sif_dev;

	if (group)
		clear_bit(VIO_GROUP_INIT, &group->state);

	if (group->gtask)
		vio_group_task_stop(group->gtask);

	frame_manager_close(&sif_ctx->framemgr);

	if (atomic_dec_return(&sif->open_cnt) == 0) {
		if (test_bit(SIF_HW_RUN, &sif->state)) {
			set_bit(SIF_HW_FORCE_STOP, &sif->state);
			sif_video_streamoff(sif_ctx);
			clear_bit(SIF_HW_FORCE_STOP, &sif->state);
			atomic_set(&sif->rsccount, 0);
			vio_info("sif force stream off\n");
		}
		//it should disable after ipu stream off because it maybe contain ipu/sif clk
		//vio_clk_disable("sif_mclk");
	}
	sif_ctx->state = BIT(VIO_VIDEO_CLOSE);

	kfree(sif_ctx);

	vio_info("SIF close node %d\n", sif_ctx->id);
	return 0;
}

int sif_mux_init(struct sif_video_ctx *sif_ctx, sif_cfg_t *sif_config)
{
	int ret = 0;
	u32 cfg = 0;
	u32 mux_index = 0;
	u32 ddr_enable = 0;
	u32 dol_exp_num = 0;
	struct x3_sif_dev *sif;
	struct vio_group_task *gtask;
	struct vio_group *group;

	sif = sif_ctx->sif_dev;
	group = sif_ctx->group;

	mux_index = sif_config->input.mipi.func.set_mux_out_index;
	sif_ctx->mux_index = mux_index;

	if (sif_config->input.mipi.data.format == HW_FORMAT_YUV422) {
		if (mux_index % 2 == 0) {
			set_bit(mux_index + 1, &sif->state);
			vio_info("sif input format is yuv, and current mux = %d\n",
			     mux_index);
		} else
			vio_err("sif input format is yuv, but mux is wrong = %d\n",
			     mux_index);
	}

	dol_exp_num = sif_config->output.isp.dol_exp_num;
	sif_ctx->dol_num = dol_exp_num;
	if(dol_exp_num == 2){
		set_bit(SIF_DOL2_MODE + mux_index + 1, &sif->state);
		vio_info("DOL2 mode, current mux = %d\n", mux_index);
	}else if(dol_exp_num == 3){
		set_bit(SIF_DOL2_MODE + mux_index + 1, &sif->state);
		set_bit(SIF_DOL2_MODE + mux_index + 2, &sif->state);
		vio_info("DOL3 mode, current mux = %d\n", mux_index);
	}

	sif_ctx->rx_num = sif_config->input.mipi.mipi_rx_index;
	sif_ctx->initial_frameid = true;
	sif->sif_mux[mux_index] = group;

	ddr_enable =  sif_config->output.ddr.enable;
	if(ddr_enable == 0) {
		set_bit(VIO_GROUP_OTF_OUTPUT, &group->state);
		cfg = ips_get_bus_ctrl() | 0xd21e << 16;
	} else {
		set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);
		cfg = ips_get_bus_ctrl() | 0xc002 << 16;

		gtask = &sif->sifout_task[mux_index];
		gtask->id = group->id;
		group->gtask = gtask;
		group->frame_work = sif_write_frame_work;
		vio_group_task_start(gtask);
		sema_init(&gtask->hw_resource, 4);
	}

	ips_set_bus_ctrl(cfg);
	sif_hw_config(sif->base_reg, sif_config);

	sif_ctx->bufcount = sif_get_current_bufindex(sif->base_reg, mux_index);
	sif->mismatch_cnt = 0;

	vio_info("[S%d][V%d] %s mux_index %d\n", group->instance,
		sif_ctx->id, __func__, mux_index);
	return ret;
}

int sif_video_init(struct sif_video_ctx *sif_ctx, unsigned long arg)
{
	int ret = 0;
	struct x3_sif_dev *sif;
	struct vio_group *group;
	sif_cfg_t sif_config;

	group = sif_ctx->group;
	if (!(sif_ctx->state & (BIT(VIO_VIDEO_S_INPUT) | BIT(VIO_VIDEO_REBUFS)))) {
		vio_err("[%d][%s][V%02d] invalid INIT is requested(%lX)",
			group->instance, __func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	ret = copy_from_user((char *) &sif_config, (u32 __user *) arg,
			   sizeof(sif_cfg_t));
	if (ret)
		return -EFAULT;

	sif = sif_ctx->sif_dev;

	if (sif_ctx->id == 0) {
		ret = sif_mux_init(sif_ctx, &sif_config);
	} else if (sif_ctx->id == 1) {
		sif_ctx->dol_num = sif_config.output.isp.dol_exp_num;
		sif_set_isp_performance(sif->base_reg, 2);
		set_bit(SIF_DMA_IN_ENABLE, &sif->state);
		set_bit(VIO_GROUP_DMA_INPUT, &group->state);
		set_bit(VIO_GROUP_OTF_OUTPUT, &group->state);
		vio_group_task_start(group->gtask);
	}

	sif_ctx->state = BIT(VIO_VIDEO_INIT);

	vio_info("[S%d][V%d] %s done\n", group->instance,
		sif_ctx->id, __func__);

	return ret;
}

int sif_bind_chain_group(struct sif_video_ctx *sif_ctx, int instance)
{
	int ret = 0;
	int group_id = 0;
	struct vio_group *group;
	struct x3_sif_dev *sif;
	struct vio_chain *chain;

	if (!(sif_ctx->state & BIT(VIO_VIDEO_OPEN))) {
		vio_err("[%s]invalid BIND is requested(%lX)",
			__func__, sif_ctx->state);
		return -EINVAL;
	}

	if (instance < 0 || instance >= VIO_MAX_STREAM) {
		vio_err("wrong instance id(%d)\n", instance);
		return -EFAULT;
	}

	sif = sif_ctx->sif_dev;

	if(sif_ctx->id == 0)
		group_id = GROUP_ID_SIF_OUT;
	else
		group_id = GROUP_ID_SIF_IN;

	group = vio_get_chain_group(instance, group_id);
	if (!group)
		return -EFAULT;

	group->sub_ctx[0] = sif_ctx;
	sif_ctx->group = group;

	chain = group->chain;

	if(sif_ctx->id == 1){
		sif->sif_input[instance] = group;
		group->frame_work = sif_read_frame_work;
		group->gtask = &sif->sifin_task;
	}
	sif_ctx->state = BIT(VIO_VIDEO_S_INPUT);

	vio_info("[S%d][V%d] %s done\n", group->instance,
		sif_ctx->id, __func__);

	return ret;
}

int sif_video_streamon(struct sif_video_ctx *sif_ctx)
{
	unsigned long flag;
	struct x3_sif_dev *sif_dev;

	if (!(sif_ctx->state & (BIT(VIO_VIDEO_STOP)
			| BIT(VIO_VIDEO_REBUFS)
			| BIT(VIO_VIDEO_INIT)))) {
		vio_err("[%s][V%02d] invalid STREAM ON is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	sif_dev = sif_ctx->sif_dev;

	if (atomic_read(&sif_dev->rsccount) > 0)
		goto p_inc;

	spin_lock_irqsave(&sif_dev->shared_slock, flag);
	sif_dev->error_count = 0;
	sif_hw_enable(sif_dev->base_reg);
	set_bit(SIF_HW_RUN, &sif_dev->state);
	spin_unlock_irqrestore(&sif_dev->shared_slock, flag);

p_inc:
	atomic_inc(&sif_dev->rsccount);
	sif_ctx->state = BIT(VIO_VIDEO_START);

	vio_info("[S%d][V%d] %s\n", sif_ctx->group->instance,
		sif_ctx->id, __func__);

	return 0;
}

int sif_video_streamoff(struct sif_video_ctx *sif_ctx)
{
	struct x3_sif_dev *sif_dev;
	struct vio_framemgr *framemgr;
	unsigned long flag;

	if (!(sif_ctx->state & BIT(VIO_VIDEO_START))) {
		vio_err("[%s][V%02d] invalid STREAM OFF is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	sif_dev = sif_ctx->sif_dev;
	framemgr = &sif_ctx->framemgr;

	if (atomic_dec_return(&sif_dev->rsccount) > 0
		&& !test_bit(SIF_HW_FORCE_STOP, &sif_dev->state))
		goto p_dec;

	msleep(100);

	spin_lock_irqsave(&sif_dev->shared_slock, flag);

	sif_hw_disable(sif_dev->base_reg);
	clear_bit(SIF_HW_RUN, &sif_dev->state);

	if (sif_ctx->id == 1)
		clear_bit(SIF_DMA_IN_ENABLE, &sif_dev->state);
	spin_unlock_irqrestore(&sif_dev->shared_slock, flag);
p_dec:

	if (sif_ctx->id == 0)
		clear_bit(sif_ctx->mux_index + 1, &sif_dev->state);

	if (framemgr->frames != NULL)
		frame_manager_flush(framemgr);

	sif_ctx->state = BIT(VIO_VIDEO_STOP);

	vio_info("[S%d][V%d] %s\n", sif_ctx->group->instance,
		sif_ctx->id, __func__);

	return 0;
}

int sif_video_s_stream(struct sif_video_ctx *sif_ctx, bool enable)
{
	if (enable)
		sif_video_streamon(sif_ctx);
	else
		sif_video_streamoff(sif_ctx);

	return 0;
}

int sif_video_reqbufs(struct sif_video_ctx *sif_ctx, u32 buffers)
{
	int ret = 0;
	int i = 0;
	struct vio_framemgr *framemgr;
	struct vio_group *group;

	if (!(sif_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS) |
		      BIT(VIO_VIDEO_INIT) | BIT(VIO_VIDEO_S_INPUT)))) {
		vio_err("[%s][V%02d] invalid REQBUFS is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	vio_info("%s: buffers = %d\n", __func__, buffers);
	framemgr = &sif_ctx->framemgr;
	ret = frame_manager_open(framemgr, buffers);
	if (ret) {
		vio_err("frame manage open failed, ret(%d)\n", ret);
		return ret;
	}

	group = sif_ctx->group;
	for (i = 0; i < buffers; i++) {
		framemgr->frames[i].data = group;
	}

	sif_ctx->state = BIT(VIO_VIDEO_REBUFS);

	vio_info("[S%d][V%d] %s reqbuf num %d\n", group->instance,
		sif_ctx->id, __func__, buffers);

	return ret;
}

int sif_video_qbuf(struct sif_video_ctx *sif_ctx,
			struct frame_info *frameinfo)
{
	int ret = 0;
	int index = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	struct vio_group *group;

	index = frameinfo->bufferindex;
	framemgr = &sif_ctx->framemgr;
	BUG_ON(index >= framemgr->num_frames);

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = &framemgr->frames[index];
	if (frame->state == FS_FREE) {
		/*TODO: get the phy address from ion handle @kaikai */
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_REQUEST);
	} else {
		vio_err("frame(%d) is invalid state(%d)\n", index,
			frame->state);
		ret = -EINVAL;
	}

	framemgr_x_barrier_irqr(framemgr, 0, flags);

	group = sif_ctx->group;
	vio_group_start_trigger(group, frame);

	vio_dbg("[S%d][V%d] %s index %d\n", sif_ctx->group->instance,
		sif_ctx->id, __func__, frameinfo->bufferindex);

	return ret;

}

int sif_video_dqbuf(struct sif_video_ctx *sif_ctx,
			struct frame_info *frameinfo)
{
	int ret = 0;
	struct list_head *done_list;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;

	framemgr = &sif_ctx->framemgr;

	done_list = &framemgr->queued_list[FS_COMPLETE];
	wait_event_interruptible(sif_ctx->done_wq, !list_empty(done_list));

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_COMPLETE);
	if (frame) {
		memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_FREE);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	vio_dbg("[S%d][V%d] %s index %d\n", sif_ctx->group->instance,
		sif_ctx->id, __func__, frameinfo->bufferindex);

	return ret;
}

static long x3_sif_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int ret = 0;
	int buffers = 0;
	int enable = 0;
	int instance = 0;
	struct sif_video_ctx *sif_ctx;
	struct frame_info frameinfo;
	struct vio_group *group;

	sif_ctx = file->private_data;
	BUG_ON(!sif_ctx);
	group = sif_ctx->group;

	if (_IOC_TYPE(cmd) != SIF_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case SIF_IOC_INIT:
		ret = sif_video_init(sif_ctx, arg);
		break;
	case SIF_IOC_STREAM:
		ret = get_user(enable, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		ret = sif_video_s_stream(sif_ctx, ! !enable);
		break;
	case SIF_IOC_DQBUF:
		sif_video_dqbuf(sif_ctx, &frameinfo);
		ret = copy_to_user((void __user *) arg,
				(char *) &frameinfo, sizeof(struct frame_info));
		if (ret)
			return -EFAULT;
		break;
	case SIF_IOC_QBUF:
		ret = copy_from_user((char *) &frameinfo,
				(u32 __user *) arg, sizeof(struct frame_info));
		if (ret)
			return -EFAULT;
		sif_video_qbuf(sif_ctx, &frameinfo);
		break;
	case SIF_IOC_REQBUFS:
		ret = get_user(buffers, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		sif_video_reqbufs(sif_ctx, buffers);
		break;
	case SIF_IOC_BIND_GROUP:
		ret = get_user(instance, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		sif_bind_chain_group(sif_ctx, instance);
		break;
	case SIF_IOC_END_OF_STREAM:
		ret = get_user(instance, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		vio_bind_group_done(instance);
		break;
	default:
		vio_err("wrong ioctl command\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

static struct file_operations x3_sif_fops = {
	.owner = THIS_MODULE,
	.open = x3_sif_open,
	.write = x3_sif_write,
	.read = x3_sif_read,
	.release = x3_sif_close,
	.poll = x3_sif_poll,
	.unlocked_ioctl = x3_sif_ioctl,
	.compat_ioctl = x3_sif_ioctl,
};

void sif_get_timestamps(struct vio_group *group, struct frame_id *info){
	struct sif_video_ctx *sif_ctx;

	sif_ctx = group->sub_ctx[0];

	info = &sif_ctx->info;
}


void sif_frame_done(struct sif_video_ctx *sif_ctx)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	unsigned long flags;

	group = sif_ctx->group;
	framemgr = &sif_ctx->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		if(group->get_timestamps){
			frame->frameinfo.frame_id = group->frameid.frame_id;
			frame->frameinfo.timestamps =
			    group->frameid.timestamps;
		}

		do_gettimeofday(&frame->frameinfo.tv);

		trans_frame(framemgr, frame, FS_COMPLETE);
		sif_ctx->event = VIO_FRAME_DONE;
	} else {
		sif_ctx->event = VIO_FRAME_NDONE;
		vio_err("%s PROCESS queue has no member;\n", __func__);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	wake_up(&sif_ctx->done_wq);
}

static irqreturn_t sif_isr(int irq, void *data)
{
	int ret = 0;
	u32 mux_index = 0;
	u32 instance;
	u32 status = 0;
	u32 intr_en = 0;
	struct sif_irq_src irq_src;
	struct x3_sif_dev *sif;
	struct sif_video_ctx *sif_ctx;
	struct vio_group *group;
	struct vio_group_task *gtask;

	sif = (struct x3_sif_dev *) data;
	memset(&irq_src, 0x0, sizeof(struct sif_irq_src));
	ret = sif_get_irq_src(sif->base_reg, &irq_src, true);
	intr_en = sif_get_frame_intr(sif->base_reg);
	status = intr_en & irq_src.sif_frm_int;

	vio_info("%s: sif_frm_int = 0x%x,sif_out_int =0x%x, status  = 0x%x\n",
		__func__, irq_src.sif_frm_int, irq_src.sif_out_int, status);

	if (status) {
		for (mux_index = 0; mux_index <= 7; mux_index++) {
			if (test_bit(mux_index, &sif->state)
			    && (mux_index % 2 == 1))
				continue;

			if (test_bit(mux_index + SIF_DOL2_MODE, &sif->state))
				continue;

			if (status & 1 <<
			    (mux_index + INTR_SIF_MUX0_FRAME_DONE)) {
			    group = sif->sif_mux[mux_index];
				sif_ctx = group->sub_ctx[0];
				sif_frame_done(sif_ctx);
			}
			//Frame start processing
			if ((status & 1 << mux_index)) {
				group = sif->sif_mux[mux_index];
				sif_ctx = group->sub_ctx[0];
				if(sif_ctx->initial_frameid){
					sif_enable_init_frameid(sif->base_reg, sif_ctx->rx_num, 0);
					sif_ctx->initial_frameid = false;
				}
				sif_get_frameid_timestamps(sif->base_reg,
							   mux_index, &group->frameid);

				if (test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)) {
					gtask = group->gtask;
					if (unlikely(list_empty(&gtask->hw_resource.wait_list))) {
						vio_err("[mux %d]GP%d(res %d, rcnt %d)\n", mux_index,
							gtask->id,
							gtask->hw_resource.count,
							atomic_read(&group->rcount));
					} else {
						up(&gtask->hw_resource);
					}
				}
			}
		}
	}

	if (test_bit(SIF_DMA_IN_ENABLE, &sif->state)
	    && (irq_src.sif_out_int & 1 << SIF_ISP_OUT_FE)) {
		instance = atomic_read(&sif->instance);
		group = sif->sif_input[instance];
		gtask = group->gtask;

		sif_ctx = group->sub_ctx[0];
		sif_frame_done(sif_ctx);

		up(&gtask->hw_resource);
	}

	if (irq_src.sif_frm_int & 1 << INTR_SIF_IN_SIZE_MISMATCH) {
		if (mismatch_limit < 0 || sif->mismatch_cnt < mismatch_limit) {
			vio_err("input size mismatch(0x%x)\n", irq_src.sif_err_status);
			sif_print_rx_status(sif->base_reg, irq_src.sif_err_status);
		}
		sif->mismatch_cnt++;
	}

	if (irq_src.sif_frm_int & 1 << INTR_SIF_IN_OVERFLOW) {
		sif->error_count++;
		vio_err("input buffer overflow(0x%x)\n", irq_src.sif_in_buf_overflow);
		sif_print_buffer_status(sif->base_reg);
	}

	if (irq_src.sif_frm_int & 1 << INTR_SIF_OUT_BUF_ERROR) {
		sif->error_count++;
		vio_err("Out buffer error\n");
	}

	if (sif->error_count >= SIF_ERR_COUNT) {
		sif_hw_dump(sif->base_reg);
		sif->error_count = 0;
	}

	return IRQ_HANDLED;
}

int x3_sif_device_node_init(struct x3_sif_dev *sif)
{
	int ret = 0;
	struct device *dev = NULL;

	ret = alloc_chrdev_region(&sif->devno, 0, MAX_DEVICE, "x3_sif");
	if (ret < 0) {
		vio_err("Error %d while alloc chrdev sif", ret);
		goto err_req_cdev;
	}

	cdev_init(&sif->cdev, &x3_sif_fops);
	sif->cdev.owner = THIS_MODULE;
	ret = cdev_add(&sif->cdev, sif->devno, MAX_DEVICE);
	if (ret) {
		vio_err("Error %d while adding x2 sif cdev", ret);
		goto err;
	}

	if (vps_class)
		sif->class = vps_class;
	else
		sif->class = class_create(THIS_MODULE, X3_SIF_NAME);

	dev = device_create(sif->class, NULL, MKDEV(MAJOR(sif->devno), 0),
				NULL, "sif_capture");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		vio_err("sif device create fail\n");
		goto err;
	}

	dev = device_create(sif->class, NULL, MKDEV(MAJOR(sif->devno), 1),
	    		NULL, "sif_ddr_input");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		vio_err("sif device create fail\n");
		goto err;
	}

	return ret;
err:
	class_destroy(sif->class);
err_req_cdev:
	unregister_chrdev_region(sif->devno, MAX_DEVICE);
	return ret;
}

static ssize_t sif_reg_dump(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_sif_dev *sif;

	sif = dev_get_drvdata(dev);

	sif_hw_dump(sif->base_reg);

	return 0;
}

static DEVICE_ATTR(regdump, 0444, sif_reg_dump, NULL);

static int x3_sif_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x3_sif_dev *sif;
	struct resource *mem_res;
	struct device *dev = NULL;

	sif = kzalloc(sizeof(struct x3_sif_dev), GFP_KERNEL);
	if (!sif) {
		vio_err("sif is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		vio_err("Failed to get io memory region(%p)", mem_res);
		ret = -EBUSY;
		goto err_get_resource;
	}

	sif->regs_start = mem_res->start;
	sif->regs_end = mem_res->end;
	sif->base_reg = devm_ioremap_nocache(&pdev->dev,
			mem_res->start, resource_size(mem_res));
	if (!sif->base_reg) {
		vio_err("Failed to remap io region(%p)", sif->base_reg);
		ret = -ENOMEM;
		goto err_get_resource;
	}

	/* Get IRQ SPI number */
	sif->irq = platform_get_irq(pdev, 0);
	if (sif->irq < 0) {
		vio_err("Failed to get csi_irq(%d)", sif->irq);
		ret = -EBUSY;
		goto err_get_irq;
	}

	ret = request_threaded_irq(sif->irq, sif_isr, NULL,
			IRQF_TRIGGER_HIGH, "sif", sif);
	if (ret) {
		vio_err("request_irq(IRQ_SIF %d) is fail(%d)", sif->irq, ret);
		goto err_get_irq;
	}

	sema_init(&sif->sifin_task.hw_resource, 1);
	atomic_set(&sif->rsccount, 0);
	atomic_set(&sif->open_cnt, 0);

	spin_lock_init(&sif->shared_slock);

	x3_sif_device_node_init(sif);

	dev = &pdev->dev;
	ret = device_create_file(dev, &dev_attr_regdump);
	if(ret < 0) {
		vio_err("create regdump failed (%d)\n",ret);
		goto p_err;
	}

	platform_set_drvdata(pdev, sif);

	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

	return 0;

err_get_irq:
	iounmap(sif->base_reg);

err_get_resource:
	kfree(sif);
p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;

}

static int x3_sif_remove(struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	struct x3_sif_dev *sif;

	BUG_ON(!pdev);

	sif = platform_get_drvdata(pdev);

	free_irq(sif->irq, sif);
	for(i = 0; i < MAX_DEVICE; i++)
		device_destroy(sif->class, MKDEV(MAJOR(sif->devno), i));

	class_destroy(sif->class);
	cdev_del(&sif->cdev);
	unregister_chrdev_region(sif->devno, MAX_DEVICE);
	kfree(sif);

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x3_sif_match[] = {
	{
	 .compatible = "hobot,x3-sif",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, x3_sif_match);

static struct platform_driver x3_sif_driver = {
	.probe = x3_sif_probe,
	.remove = x3_sif_remove,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_sif_pm_ops,
		   .of_match_table = x3_sif_match,
		   }
};

#else
static struct platform_device_id x3_sif_driver_ids[] = {
	{
	 .name = MODULE_NAME,
	 .driver_data = 0,
	 },
	{},
};

MODULE_DEVICE_TABLE(platform, x3_sif_driver_ids);

static struct platform_driver x3_sif_driver = {
	.probe = x3_sif_probe,
	.remove = __devexit_p(x3_sif_remove),
	.id_table = x3_sif_driver_ids,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_sif_pm_ops,
		   }
};
#endif

static int __init x3_sif_init(void)
{
	int ret = platform_driver_register(&x3_sif_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

late_initcall(x3_sif_init);

static void __exit x3_sif_exit(void)
{
	platform_driver_unregister(&x3_sif_driver);
}

module_exit(x3_sif_exit);

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X3 SIF driver");
MODULE_LICENSE("GPL");
