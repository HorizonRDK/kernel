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

void sif_config_rdma_cfg(struct sif_subdev *subdev, u8 index,
			struct frame_info *frameinfo)
{
	u32 height = 0;
	u32 width = 0;
	u32 format = 0;
	u32 pixel_length = 0;
	u32 stride = 0;
	struct x3_sif_dev *sif;
	struct vio_group *group;

	sif = subdev->sif_dev;
	group = subdev->group;
	height = subdev->ddrin_fmt.height;
	width = subdev->ddrin_fmt.width;
	format  = subdev->ddrin_fmt.format;
	pixel_length = subdev->ddrin_fmt.pix_length;
	sif_config_rdma_fmt(sif->base_reg, pixel_length, width, height);
	stride = sif_get_stride(pixel_length, width);
	sif_set_rdma_buf_stride(sif->base_reg, index, stride);

	sif_set_rdma_enable(sif->base_reg, index, true);
	sif_set_rdma_buf_addr(sif->base_reg, index, frameinfo->addr[index]);
	vio_info("[S%d]ddr in width = %d, height = %d, stride = %d\n",
			group->instance, width, height, stride);
}

void sif_read_frame_work(struct vio_group *group)
{
	unsigned long flags;
	u32 instance = 0;
	struct x3_sif_dev *sif;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct frame_info *frameinfo;
	struct sif_subdev * subdev;

	instance = group->instance;

	if (sif_isp_ctx_sync != NULL) {
		(*sif_isp_ctx_sync)(instance);
	}

	subdev = group->sub_ctx[0];
	if (unlikely(!subdev)) {
		vio_err("%s error subdev null,instance %d", __func__, instance);
		return;
	}

	sif = subdev->sif_dev;

	atomic_set(&sif->instance, instance);

	framemgr = &subdev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame) {
		frameinfo = &frame->frameinfo;
		sif_config_rdma_cfg(subdev, 0, frameinfo);

		if (subdev->dol_num > 1) {
			sif_config_rdma_cfg(subdev, 1, frameinfo);
		}

		if (subdev->dol_num > 2) {
			sif_config_rdma_cfg(subdev, 2, frameinfo);
		}
		sif_set_rdma_trigger(sif->base_reg, 1);
		trans_frame(framemgr, frame, FS_PROCESS);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	vio_dbg("[S%d]%s:done", group->instance, __func__);
}


void sif_write_frame_work(struct vio_group *group)
{
	u32 instance = 0;
	u32 buf_index = 0;
	u32 mux_index = 0;
	u32 yuv_format = 0;
	unsigned long flags;
	struct x3_sif_dev *sif;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct sif_subdev *subdev;

	instance = group->instance;
	subdev = group->sub_ctx[0];
	if (unlikely(!subdev)) {
		vio_err("%s subdev null,instance %d", __func__, instance);
		return;
	}

	sif = subdev->sif_dev;
	mux_index = subdev->mux_index;

	framemgr = &subdev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame) {
		buf_index = subdev->bufcount % 4;
		yuv_format = (frame->frameinfo.format == HW_FORMAT_YUV422);
		sif_set_wdma_buf_addr(sif->base_reg, mux_index, buf_index,
				      frame->frameinfo.addr[0]);
		sif_transfer_ddr_owner(sif->base_reg, mux_index, buf_index);

		if (yuv_format || subdev->dol_num > 1) {
			sif_set_wdma_buf_addr(sif->base_reg, mux_index + 1,
					      buf_index, frame->frameinfo.addr[1]);
			sif_transfer_ddr_owner(sif->base_reg, mux_index + 1,
					       buf_index);
		}

		if (subdev->dol_num > 2) {
			sif_set_wdma_buf_addr(sif->base_reg, mux_index + 2,
					      buf_index, frame->frameinfo.addr[2]);
			sif_transfer_ddr_owner(sif->base_reg, mux_index + 2,
					       buf_index);
		}
		trans_frame(framemgr, frame, FS_PROCESS);
		subdev->bufcount++;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	vio_dbg("[S%d]%s:done", group->instance, __func__);
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
	struct sif_subdev *subdev;

	sif_ctx = file->private_data;
	sif = sif_ctx->sif_dev;
	subdev = sif_ctx->subdev;
	group = subdev->group;

	if ((group) && atomic_dec_return(&subdev->refcount) == 0) {
		subdev->state = 0;
		clear_bit(VIO_GROUP_INIT, &group->state);
		clear_bit(subdev->mux_index + 1, &sif->state);
		if (subdev->dol_num > 1)
			clear_bit(SIF_DOL2_MODE + subdev->mux_index + 1, &sif->state);
		if (subdev->dol_num > 2)
			clear_bit(SIF_DOL2_MODE + subdev->mux_index + 2, &sif->state);

		if (group->gtask)
			vio_group_task_stop(group->gtask);

		frame_manager_close(sif_ctx->framemgr);
	}

	if (atomic_dec_return(&sif->open_cnt) == 0) {
		clear_bit(SIF_DMA_IN_ENABLE, &sif->state);
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

	spin_lock(&subdev->slock);
	clear_bit(sif_ctx->ctx_index, &subdev->val_ctx_mask);
	kfree(sif_ctx);
	spin_unlock(&subdev->slock);

	vio_info("SIF close node %d\n", sif_ctx->id);
	return 0;
}

int sif_get_stride(u32 pixel_length, u32 width)
{
	u32 stride = 0;

	switch (pixel_length) {
	case PIXEL_LENGTH_8BIT:
		stride = width;
		break;
	case PIXEL_LENGTH_10BIT:
		stride = width * 5 / 4;
		break;
	case PIXEL_LENGTH_12BIT:
		stride = width * 3 / 2;
		break;
	case PIXEL_LENGTH_16BIT:
		stride = width * 2;
		break;
	case PIXEL_LENGTH_20BIT:
		stride = width * 5 / 2;
		break;
	default:
		vio_err("wrong pixel length is %d\n", pixel_length);
		break;
	}

	return stride;
}

int sif_mux_init(struct sif_subdev *subdev, sif_cfg_t *sif_config)
{
	int ret = 0;
	u32 cfg = 0;
	u32 mux_index = 0;
	u32 ddr_enable = 0;
	u32 dol_exp_num = 0;
	struct x3_sif_dev *sif;
	struct vio_group_task *gtask;
	struct vio_group *group;

	sif = subdev->sif_dev;
	group = subdev->group;

	mux_index = sif_config->input.mipi.func.set_mux_out_index;
	subdev->mux_index = mux_index;

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
	subdev->dol_num = dol_exp_num;
	if(dol_exp_num == 2){
		set_bit(SIF_DOL2_MODE + mux_index + 1, &sif->state);
		vio_info("DOL2 mode, current mux = %d\n", mux_index);
	}else if(dol_exp_num == 3){
		set_bit(SIF_DOL2_MODE + mux_index + 1, &sif->state);
		set_bit(SIF_DOL2_MODE + mux_index + 2, &sif->state);
		vio_info("DOL3 mode, current mux = %d\n", mux_index);
	}

	subdev->rx_num = sif_config->input.mipi.mipi_rx_index;
	subdev->initial_frameid = true;
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

	subdev->bufcount = sif_get_current_bufindex(sif->base_reg, mux_index);
	sif->mismatch_cnt = 0;

	vio_info("[S%d] %s mux_index %d\n", group->instance,
			__func__, mux_index);

	return ret;
}

int sif_video_init(struct sif_video_ctx *sif_ctx, unsigned long arg)
{
	int ret = 0;
	struct x3_sif_dev *sif;
	struct vio_group *group;
	struct sif_subdev *subdev;
	sif_cfg_t sif_config;

	group = sif_ctx->group;
	if (!(sif_ctx->state & (BIT(VIO_VIDEO_S_INPUT) | BIT(VIO_VIDEO_REBUFS)))) {
		vio_err("[%d][%s][V%02d] invalid INIT is requested(%lX)",
			group->instance, __func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	subdev = sif_ctx->subdev;
	if (test_bit(SIF_SUBDEV_INIT, &subdev->state)) {
		vio_info("subdev already init, current refcount(%d)\n",
				atomic_read(&subdev->refcount));
		return ret;
	}

	ret = copy_from_user((char *) &sif_config, (u32 __user *) arg,
			   sizeof(sif_cfg_t));
	if (ret)
		return -EFAULT;

	sif = subdev->sif_dev;

	if (sif_ctx->id == 0) {
		ret = sif_mux_init(subdev, &sif_config);
	} else if (sif_ctx->id == 1) {
		subdev->dol_num = sif_config.output.isp.dol_exp_num;
		memcpy(&subdev->ddrin_fmt, &sif_config.input.ddr.data,
			sizeof(sif_data_desc_t));
		sif_set_isp_performance(sif->base_reg, 10);
		set_bit(SIF_DMA_IN_ENABLE, &sif->state);
		set_bit(VIO_GROUP_DMA_INPUT, &group->state);
		set_bit(VIO_GROUP_OTF_OUTPUT, &group->state);
	}

	sif_ctx->state = BIT(VIO_VIDEO_INIT);
	set_bit(SIF_SUBDEV_INIT, &subdev->state);

	vio_info("[S%d][V%d] %s done\n", group->instance,
		sif_ctx->id, __func__);

	return ret;
}

int sif_bind_chain_group(struct sif_video_ctx *sif_ctx, int instance)
{
	int ret = 0;
	int group_id = 0;
	int i = 0;
	struct vio_group *group;
	struct x3_sif_dev *sif;
	struct sif_subdev *subdev;

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

	if (sif_ctx->id == 0) {
		subdev = &sif->sif_mux_subdev[instance];
		group_id = GROUP_ID_SIF_OUT;
		subdev->id = sif_ctx->id;
	} else {
		subdev = &sif->sif_in_subdev[instance];
		group_id = GROUP_ID_SIF_IN;
		subdev->id = sif_ctx->id;
	}
	group = vio_get_chain_group(instance, group_id);
	if (!group)
		return -EFAULT;

	group->sub_ctx[0] = subdev;
	sif_ctx->group = group;
	sif_ctx->subdev = subdev;
	sif_ctx->framemgr = &subdev->framemgr;
	subdev->sif_dev = sif;
	subdev->group = group;

	spin_lock(&subdev->slock);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if(!test_bit(i, &subdev->val_ctx_mask)) {
			subdev->ctx[i] = sif_ctx;
			sif_ctx->ctx_index = i;
			set_bit(i, &subdev->val_ctx_mask);
			break;
		}
	}
	spin_unlock(&subdev->slock);
	if (i == VIO_MAX_SUB_PROCESS) {
		vio_err("alreay open too much for one pipeline\n");
		return -EFAULT;
	}
	atomic_inc(&subdev->refcount);

	if(sif_ctx->id == 1){
		sif->sif_input[instance] = group;
		group->frame_work = sif_read_frame_work;
		group->gtask = &sif->sifin_task;
		vio_group_task_start(group->gtask);
	}

	sif_ctx->state = BIT(VIO_VIDEO_S_INPUT);

	vio_info("[S%d][V%d] %s done, ctx_index(%d) refcount(%d)\n",
		group->instance, sif_ctx->id, __func__,
		i, atomic_read(&subdev->refcount));

	return ret;
}

int sif_video_streamon(struct sif_video_ctx *sif_ctx)
{
	int ret = 0;
	unsigned long flag;
	struct x3_sif_dev *sif_dev;
	struct sif_subdev *subdev;

	if (!(sif_ctx->state & (BIT(VIO_VIDEO_STOP)
			| BIT(VIO_VIDEO_REBUFS)
			| BIT(VIO_VIDEO_INIT)))) {
		vio_err("[%s][V%02d] invalid STREAM ON is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	subdev = sif_ctx->subdev;
	if (test_bit(SIF_SUBDEV_STREAM_ON, &subdev->state))
		return ret;

	sif_dev = sif_ctx->sif_dev;

	if (atomic_read(&sif_dev->rsccount) > 0)
		goto p_inc;

	msleep(500);
	spin_lock_irqsave(&sif_dev->shared_slock, flag);
	sif_dev->error_count = 0;
	sif_hw_enable(sif_dev->base_reg);
	set_bit(SIF_HW_RUN, &sif_dev->state);
	spin_unlock_irqrestore(&sif_dev->shared_slock, flag);

p_inc:
	atomic_inc(&sif_dev->rsccount);
	sif_ctx->state = BIT(VIO_VIDEO_START);
	set_bit(SIF_SUBDEV_STREAM_ON, &subdev->state);

	vio_info("[S%d][V%d]%s\n", sif_ctx->group->instance,
		sif_ctx->id, __func__);

	return ret;
}

int sif_video_streamoff(struct sif_video_ctx *sif_ctx)
{
	int ret = 0;
	struct x3_sif_dev *sif_dev;
	struct vio_framemgr *framemgr;
	struct sif_subdev *subdev;
	unsigned long flag;

	if (!(sif_ctx->state & BIT(VIO_VIDEO_START))) {
		vio_err("[%s][V%02d] invalid STREAM OFF is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	subdev = sif_ctx->subdev;
	if (test_bit(SIF_SUBDEV_STREAM_OFF, &subdev->state))
		return ret;

	sif_dev = sif_ctx->sif_dev;
	framemgr = sif_ctx->framemgr;

	if (atomic_dec_return(&sif_dev->rsccount) > 0
		&& !test_bit(SIF_HW_FORCE_STOP, &sif_dev->state))
		goto p_dec;

	msleep(100);

	spin_lock_irqsave(&sif_dev->shared_slock, flag);

	sif_hw_disable(sif_dev->base_reg);
	clear_bit(SIF_HW_RUN, &sif_dev->state);

	spin_unlock_irqrestore(&sif_dev->shared_slock, flag);
p_dec:

	if (framemgr->frames != NULL)
		frame_manager_flush(framemgr);

	sif_ctx->state = BIT(VIO_VIDEO_STOP);
	set_bit(SIF_SUBDEV_STREAM_OFF, &subdev->state);

	vio_info("[S%d][V%d]%s\n", sif_ctx->group->instance,
		sif_ctx->id, __func__);

	return ret;
}

int sif_video_s_stream(struct sif_video_ctx *sif_ctx, bool enable)
{
	int ret = 0;

	if (enable)
		ret = sif_video_streamon(sif_ctx);
	else
		ret = sif_video_streamoff(sif_ctx);

	return ret;
}

int sif_video_reqbufs(struct sif_video_ctx *sif_ctx, u32 buffers)
{
	int ret = 0;
	int i = 0;
	struct vio_framemgr *framemgr;
	struct vio_group *group;
	struct sif_subdev *subdev;

	if (!(sif_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS) |
		      BIT(VIO_VIDEO_INIT) | BIT(VIO_VIDEO_S_INPUT)))) {
		vio_err("[%s][V%02d] invalid REQBUFS is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	subdev = sif_ctx->subdev;
	if (test_bit(SIF_SUBDEV_REQBUF, &subdev->state))
		return ret;

	framemgr = sif_ctx->framemgr;
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

	set_bit(SIF_SUBDEV_REQBUF, &subdev->state);

	vio_info("[S%d][V%d]%s reqbuf num %d\n", group->instance,
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
	framemgr = sif_ctx->framemgr;
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

	vio_dbg("[S%d][V%d]%s index %d\n", sif_ctx->group->instance,
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

	framemgr = sif_ctx->framemgr;

	done_list = &framemgr->queued_list[FS_COMPLETE];
	wait_event_interruptible(sif_ctx->done_wq, !list_empty(done_list));

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_COMPLETE);
	if (frame) {
		memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_FREE);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	vio_dbg("[S%d][V%d]%s index %d\n", sif_ctx->group->instance,
		sif_ctx->id, __func__, frameinfo->bufferindex);

	return ret;
}

int sif_enable_bypass(struct sif_video_ctx *sif_ctx, u32 cfg)
{
	int i = 0;
	struct x3_sif_dev *sif;

	sif = sif_ctx->sif_dev;
	for (i = 0; i < 4; i++) {
		sif_hw_enable_bypass(sif->base_reg, i, cfg & (1 << i));
	}

	return 0;
}

static long x3_sif_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int ret = 0;
	int buffers = 0;
	int enable = 0;
	int instance = 0;
	u32 cfg = 0;
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
	case SIF_IOC_BYPASS:
		ret = get_user(cfg, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		ret = sif_enable_bypass(sif_ctx, cfg);
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
	struct sif_subdev *subdev;

	subdev = group->sub_ctx[0];

	info = &subdev->info;
}


void sif_frame_done(struct sif_subdev *subdev)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	struct sif_video_ctx *sif_ctx;
	unsigned long flags;
	u32 event = 0;
	int i = 0;

	BUG_ON(!subdev);

	group = subdev->group;
	framemgr = &subdev->framemgr;
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
		event = VIO_FRAME_DONE;
	} else {
		event = VIO_FRAME_NDONE;
		vio_err("[S%d][V%d]SIF PROCESS queue has no member;\n",
			group->instance, subdev->id);
		vio_err("[S%d][V%d][FRM](%d %d %d %d %d)\n",
			group->instance,
			subdev->id,
			framemgr->queued_count[0],
			framemgr->queued_count[1],
			framemgr->queued_count[2],
			framemgr->queued_count[3],
			framemgr->queued_count[4]);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	spin_lock(&subdev->slock);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if (test_bit(i, &subdev->val_ctx_mask)) {
			sif_ctx = subdev->ctx[i];
			sif_ctx->event = event;
			wake_up(&sif_ctx->done_wq);
		}
	}
	spin_unlock(&subdev->slock);
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
	struct vio_group *group;
	struct vio_group_task *gtask;
	struct sif_subdev *subdev;

	sif = (struct x3_sif_dev *) data;
	memset(&irq_src, 0x0, sizeof(struct sif_irq_src));
	ret = sif_get_irq_src(sif->base_reg, &irq_src, true);
	intr_en = sif_get_frame_intr(sif->base_reg);
	status = intr_en & irq_src.sif_frm_int;

	vio_dbg("%s: sif_frm_int = 0x%x,sif_out_int =0x%x, status  = 0x%x\n",
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
				subdev = group->sub_ctx[0];
				sif_frame_done(subdev);
			}
			//Frame start processing
			if ((status & 1 << mux_index)) {
				group = sif->sif_mux[mux_index];
				subdev = group->sub_ctx[0];
				if (subdev->initial_frameid) {
					sif_enable_init_frameid(sif->base_reg, subdev->rx_num, 0);
					subdev->initial_frameid = false;
				}
				sif_get_frameid_timestamps(sif->base_reg,
							   mux_index, &group->frameid);

				if (test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)) {
					gtask = group->gtask;
					if (unlikely(list_empty(&gtask->hw_resource.wait_list))) {
						vio_err("[S%d]GP%d(res %d, rcnt %d)\n",
							group->instance,
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

		vio_group_done(group);

		subdev = group->sub_ctx[0];
		sif_frame_done(subdev);
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

int sif_subdev_init(struct sif_subdev *subdev)
{
	int ret = 0;

	spin_lock_init(&subdev->slock);
	atomic_set(&subdev->refcount, 0);

	return ret;
}

int x3_sif_subdev_init(struct x3_sif_dev *sif)
{
	int i = 0;
	int ret = 0;
	struct sif_subdev *subdev;

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		subdev = &sif->sif_in_subdev[i];
		ret = sif_subdev_init(subdev);

		subdev = &sif->sif_mux_subdev[i];
		ret = sif_subdev_init(subdev);
	}

	return ret;
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

/*
 * multi-process node
 */
static int x3_vio_mp_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct mp_ctx *mp_ctx;
	struct x3_vio_mp_dev *vio_mp_dev;

	vio_mp_dev = container_of(inode->i_cdev, struct x3_vio_mp_dev, cdev);
	mp_ctx = kzalloc(sizeof(struct mp_ctx), GFP_KERNEL);
	if (mp_ctx == NULL) {
		vio_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}
	mp_ctx->mp_dev = vio_mp_dev;
	file->private_data = mp_ctx;

	vio_info("vio mp dev open.\n");
p_err:
	return ret;
}

static int x3_vio_mp_close(struct inode *inode, struct file *file)
{
	struct mp_ctx *mp_ctx;
	struct x3_vio_mp_dev *vio_mp_dev;

	mp_ctx = file->private_data;
	vio_mp_dev = mp_ctx->mp_dev;
	if(mp_ctx->refcount)
		atomic_dec(mp_ctx->refcount);
	vio_info("vio mp close instance %d\n", mp_ctx->instance);
	kfree(mp_ctx);

	return 0;
}


int vio_mp_bind_chain_group(struct mp_ctx *mp_ctx, int instance)
{
	int ret = 0;
	struct x3_vio_mp_dev *vio_mp_dev;

	if (instance < 0 || instance >= VIO_MAX_STREAM) {
		vio_err("wrong instance id(%d)\n", instance);
		return -EFAULT;
	}

	vio_mp_dev = mp_ctx->mp_dev;
	mp_ctx->refcount = &vio_mp_dev->refcount[instance];
	mp_ctx->instance = instance;
	atomic_inc(mp_ctx->refcount);

	vio_info("%s mp %d bind done, refcount(%d)\n",
		__func__, instance, atomic_read(mp_ctx->refcount));

	return ret;
}

static long x3_vio_mp_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int ret = 0;
	int instance = 0;
	struct mp_ctx *mp_ctx;

	mp_ctx = file->private_data;
	BUG_ON(!mp_ctx);

	if (_IOC_TYPE(cmd) != VIO_MP_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case VIO_MP_IOC_BIND_GROUP:
		ret = get_user(instance, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		ret = vio_mp_bind_chain_group(mp_ctx, instance);
		if (ret)
			return -EFAULT;
		break;
	case VIO_MP_IOC_GET_REFCOUNT:
		ret = put_user(atomic_read(mp_ctx->refcount),
			(u32 __user *) arg);
		if (ret)
			return -EFAULT;
		break;
	default:
		vio_err("wrong ioctl command\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}


static struct file_operations x3_vio_mp_fops = {
	.owner = THIS_MODULE,
	.open = x3_vio_mp_open,
	.release = x3_vio_mp_close,
	.unlocked_ioctl = x3_vio_mp_ioctl,
	.compat_ioctl = x3_vio_mp_ioctl,
};

int x3_vio_mp_device_node_init(struct x3_vio_mp_dev *vio_mp)
{
	int ret = 0;
	struct device *dev = NULL;

	ret = alloc_chrdev_region(&vio_mp->devno, 0, 1, "x3_vio_mp");
	if (ret < 0) {
		vio_err("Error %d while alloc chrdev vio_mp", ret);
		goto err_req_cdev;
	}

	cdev_init(&vio_mp->cdev, &x3_vio_mp_fops);
	vio_mp->cdev.owner = THIS_MODULE;
	ret = cdev_add(&vio_mp->cdev, vio_mp->devno, MAX_DEVICE_VIO_MP);
	if (ret) {
		vio_err("Error %d while adding x3 vio mp cdev", ret);
		goto err;
	}

	if (vps_class)
		vio_mp->class = vps_class;
	else
		vio_mp->class = class_create(THIS_MODULE, X3_VIO_MP_NAME);

	dev = device_create(vio_mp->class, NULL, MKDEV(MAJOR(vio_mp->devno), 0),
				NULL, "vio_mp");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		vio_err("vio mp device create fail\n");
		goto err;
	}

	return ret;
err:
	class_destroy(vio_mp->class);
err_req_cdev:
	unregister_chrdev_region(vio_mp->devno, MAX_DEVICE_VIO_MP);
	return ret;
}

struct x3_vio_mp_dev *g_vio_mp_dev = NULL;
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

	ret = vio_group_init_mp(GROUP_ID_SIF_OUT);
	if (ret < 0) {
		vio_err("init chain group(sif_out) multi-process error.");
		goto err_init_group_mp;
	}
	ret = vio_group_init_mp(GROUP_ID_SIF_IN);
	if (ret < 0) {
		vio_err("init chain group(sif_in) multi-process error.");
		goto err_init_group_mp;
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
	/*4 ddr in channel can not be 0 together*/
	sif_enable_dma(sif->base_reg, 0x10000);
	ret = x3_sif_subdev_init(sif);

#if 1
	/* vio mp dev node init*/
	g_vio_mp_dev = kzalloc(sizeof(struct x3_vio_mp_dev), GFP_KERNEL);
	if (!g_vio_mp_dev) {
		vio_err("vio mp dev is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	x3_vio_mp_device_node_init(g_vio_mp_dev);
#endif

	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

	return 0;

err_init_group_mp:
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

	for(i = 0; i < MAX_DEVICE_VIO_MP; i++)
		device_destroy(g_vio_mp_dev->class,
			MKDEV(MAJOR(g_vio_mp_dev->devno), i));

	class_destroy(g_vio_mp_dev->class);
	cdev_del(&g_vio_mp_dev->cdev);
	unregister_chrdev_region(g_vio_mp_dev->devno, MAX_DEVICE);
	kfree(g_vio_mp_dev);

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
