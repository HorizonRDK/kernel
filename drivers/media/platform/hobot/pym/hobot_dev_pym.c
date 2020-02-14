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
#include <linux/poll.h>

#include "hobot_dev_pym.h"
#include "pym_hw_api.h"

#define MODULE_NAME "X3 PYM"

extern struct class *vps_class;
static int x3_pym_open(struct inode *inode, struct file *file)
{
	struct pym_video_ctx *pym_ctx;
	struct x3_pym_dev *pym;
	int ret = 0;
	int minor;

	minor = MINOR(inode->i_rdev);

	pym = container_of(inode->i_cdev, struct x3_pym_dev, cdev);
	pym_ctx = kzalloc(sizeof(struct pym_video_ctx), GFP_KERNEL);
	if (pym_ctx == NULL) {
		vio_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	init_waitqueue_head(&pym_ctx->done_wq);

	pym_ctx->pym_dev = pym;
	file->private_data = pym_ctx;
	pym_ctx->state = BIT(VIO_VIDEO_OPEN);

	atomic_inc(&pym->open_cnt);

p_err:
	return ret;
}

static ssize_t x3_pym_write(struct file *file, const char __user *buf,
			     size_t count, loff_t * ppos)
{
	return 0;
}

static ssize_t x3_pym_read(struct file *file, char __user *buf,
				size_t size, loff_t * ppos)
{
	return 0;

}

static u32 x3_pym_poll(struct file *file, struct poll_table_struct *wait)
{
	int ret = 0;
	struct pym_video_ctx *pym_ctx;

	pym_ctx = file->private_data;

	poll_wait(file, &pym_ctx->done_wq, wait);
	if (pym_ctx->event == VIO_FRAME_DONE)
		ret = POLLIN;
	else if (pym_ctx->event == VIO_FRAME_NDONE)
		ret = POLLERR;

	pym_ctx->event = 0;

	return ret;
}

static int x3_pym_close(struct inode *inode, struct file *file)
{
	struct pym_video_ctx *pym_ctx;
	struct vio_group *group;
	struct x3_pym_dev *pym;

	pym_ctx = file->private_data;
	group = pym_ctx->group;
	pym = pym_ctx->pym_dev;

	if (group)
		clear_bit(VIO_GROUP_INIT, &group->state);

	if (group->gtask)
		vio_group_task_stop(group->gtask);

	frame_manager_close(&pym_ctx->framemgr);

	pym_ctx->state = BIT(VIO_VIDEO_CLOSE);

	if (atomic_dec_return(&pym->open_cnt) == 0) {
		clear_bit(PYM_OTF_INPUT, &pym->state);
		clear_bit(PYM_DMA_INPUT, &pym->state);
	}

	kfree(pym_ctx);

	vio_info("PYM close node %d\n", group->instance);
	return 0;
}

void pym_set_buffers(struct x3_pym_dev *pym, struct vio_frame *frame)
{
	int i = 0;
	u32 y_addr, uv_addr;

	for (i = 0; i < MAX_PYM_DS_COUNT; i++) {
		y_addr = frame->frameinfo.spec.ds_y_addr[i];
		uv_addr = frame->frameinfo.spec.ds_uv_addr[i];
		pym_wdma_ds_set_addr(pym->base_reg, i, y_addr, uv_addr);
	}

	for (i = 0; i < MAX_PYM_US_COUNT; i++) {
		y_addr = frame->frameinfo.spec.us_y_addr[i];
		uv_addr = frame->frameinfo.spec.us_uv_addr[i];
		pym_wdma_us_set_addr(pym->base_reg, i, y_addr, uv_addr);
	}
}

static void pym_frame_work(struct vio_group *group)
{
	struct pym_video_ctx *ctx;
	struct x3_pym_dev *pym;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	u32 instance = 0;
	u8 shadow_index = 0;

	instance = group->instance;
	ctx = group->sub_ctx[0];
	pym = ctx->pym_dev;

	vio_info("%s start\n", __func__);

	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;

	atomic_set(&pym->instance, instance);

	framemgr = &ctx->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame) {
		pym_set_common_rdy(pym->base_reg, 0);
		pym_set_shd_select(pym->base_reg, shadow_index);

		pym_set_buffers(pym, frame);

		if (test_bit(PYM_DMA_INPUT, &pym->state)) {
			pym_rdma_set_addr(pym->base_reg,
					  frame->frameinfo.addr[0],
					  frame->frameinfo.addr[1]);
		}
		pym_set_common_rdy(pym->base_reg, 1);

		if (test_bit(PYM_DMA_INPUT, &pym->state))
			pym_set_rdma_start(pym->base_reg);

		trans_frame(framemgr, frame, FS_PROCESS);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	vio_info("%s done\n", __func__);

	return;
}
void pym_hw_enable(struct x3_pym_dev *pym, bool enable)
{
	pym_set_common_rdy(pym->base_reg, 0);

	if(enable)
		pym_set_intr_mask(pym->base_reg, 0);
	else
		pym_set_intr_mask(pym->base_reg, 0xf);

	pym_enable_module(pym->base_reg, enable);
	if (test_bit(PYM_DMA_INPUT, &pym->state))
		pym_enable_ddr_clk(pym->base_reg, enable);

	if (test_bit(PYM_OTF_INPUT, &pym->state))
		pym_enable_otf_clk(pym->base_reg, enable);

	pym_set_common_rdy(pym->base_reg, 1);

}
void pym_update_param(struct pym_video_ctx *pym_ctx)
{
	u16 src_width, src_height, roi_width;
	u32 shadow_index = 0;
	int i = 0;
	u32 base_layer_nums = 0;
	u32 ds_bapass_uv = 0;
	pym_cfg_t *pym_config;
	struct vio_group *group;
	struct x3_pym_dev *pym;
	struct roi_rect rect;

	group = pym_ctx->group;
	pym = pym_ctx->pym_dev;
	pym_config = &pym_ctx->pym_cfg;
	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;

	pym_set_shd_rdy(pym->base_reg, shadow_index, 0);

	//config src size
	src_width = pym_config->img_width;
	src_height = pym_config->img_height;
	pym_config_src_size(pym->base_reg, shadow_index, src_width, src_height);

	// config ds roi and factor
	for (i = 0; i < MAX_PYM_DS_COUNT; i++) {
		rect.roi_x = pym_config->stds_box[i].roi_x;
		rect.roi_y = pym_config->stds_box[i].roi_y;
		rect.roi_width = pym_config->stds_box[i].tgt_width;
		rect.roi_height = pym_config->stds_box[i].tgt_height;
		pym_ds_config_factor(pym->base_reg, shadow_index, i,
				     pym_config->stds_box[i].factor);
		pym_ds_config_roi(pym->base_reg, shadow_index, i, &rect);
		roi_width = pym_config->stds_box[i].roi_width;
		pym_ds_set_src_width(pym->base_reg, shadow_index, i, roi_width);
	}

	if (pym_config->ds_uv_bypass > 0) {
		for(i = MAX_PYM_DS_COUNT; i >= 1; --i)
			if (i % 4 != 0) {
				ds_bapass_uv  <<= 1;
				if (pym_config->ds_uv_bypass >> i & 0x1) {
					ds_bapass_uv |= 1;
				}
			}
	}

	pym_ds_uv_bypass(pym->base_reg, shadow_index, ds_bapass_uv);
	vio_info("%s: %d\n", __func__, ds_bapass_uv);

	if (pym_config->ds_layer_en > 4) {
		base_layer_nums = (pym_config->ds_layer_en - 1) / 4;
		pym_ds_enabe_base_layer(pym->base_reg, shadow_index, base_layer_nums);
	}
	//config us roi and factor
	for (i = 0; i < MAX_PYM_US_COUNT; i++) {
		rect.roi_x = pym_config->stus_box[i].roi_x;
		rect.roi_y = pym_config->stus_box[i].roi_y;
		rect.roi_width = pym_config->stus_box[i].tgt_width;
		rect.roi_height = pym_config->stus_box[i].tgt_height;
		pym_us_config_factor(pym->base_reg, shadow_index, i,
				     pym_config->stus_box[i].factor);
		pym_us_config_roi(pym->base_reg, shadow_index, i, &rect);
		roi_width = pym_config->stus_box[i].roi_width;
		pym_us_set_src_width(pym->base_reg, shadow_index, i, roi_width);
	}

	pym_us_uv_bypass(pym->base_reg, shadow_index, pym_config->us_uv_bypass);
	pym_us_enabe_layer(pym->base_reg, shadow_index, pym_config->us_layer_en);
	pym_set_shd_rdy(pym->base_reg, shadow_index, 1);

	//config common register
	pym_set_common_rdy(pym->base_reg, 0);

	pym_select_input_path(pym->base_reg, pym_config->img_scr);

	pym_set_common_rdy(pym->base_reg, 1);

}

int pym_bind_chain_group(struct pym_video_ctx *pym_ctx, int instance)
{
	int ret = 0;
	struct vio_group *group;
	struct x3_pym_dev *pym;
	struct vio_chain *chain;

	if (!(pym_ctx->state & BIT(VIO_VIDEO_OPEN))) {
		vio_err("[%s]invalid BIND is requested(%lX)",
			__func__, pym_ctx->state);
		return -EINVAL;
	}

	if (instance < 0 || instance >= VIO_MAX_STREAM) {
		vio_err("wrong instance id(%d)\n", instance);
		return -EFAULT;
	}

	pym = pym_ctx->pym_dev;

	group = vio_get_chain_group(instance, GROUP_ID_PYM);
	if (!group)
		return -EFAULT;

	group->sub_ctx[0] = pym_ctx;
	pym->group[instance] = group;
	pym_ctx->group = group;

	group->frame_work = pym_frame_work;
	group->gtask = &pym->gtask;
	group->gtask->id = group->id;

	chain = group->chain;

	vio_info("[%s]instance = %d\n", __func__, instance);
	pym_ctx->state = BIT(VIO_VIDEO_S_INPUT);

	return ret;
}

int pym_video_init(struct pym_video_ctx *pym_ctx, unsigned long arg)
{
	int ret = 0;
	pym_cfg_t *pym_config;
	struct vio_group *group;
	struct x3_pym_dev *pym_dev;

	group = pym_ctx->group;
	pym_dev = pym_ctx->pym_dev;
	pym_config = &pym_ctx->pym_cfg;

	if (!(pym_ctx->state & (BIT(VIO_VIDEO_S_INPUT)
							| BIT(VIO_VIDEO_REBUFS)))) {
		vio_err("[V%02d] invalid INIT is requested(%lX)",
			group->instance, pym_ctx->state);
		return -EINVAL;
	}

	ret = copy_from_user((char *)pym_config,
				(u32 __user *) arg, sizeof(pym_cfg_t));
	if (ret)
		return -EFAULT;

	if (pym_config->img_scr == 1) {
		if (test_bit(PYM_DMA_INPUT, &pym_dev->state)){
		 	vio_err("PYM DMA input already,can't set otf input\n");
			return -EINVAL;
		}else{
			set_bit(PYM_OTF_INPUT, &pym_dev->state);
			set_bit(VIO_GROUP_OTF_INPUT, &group->state);
		}
	}else{
		if (test_bit(PYM_OTF_INPUT, &pym_dev->state)) {
			vio_err("PYM otf input already,can't set dma input\n");
			return -EINVAL;
		} else {
			set_bit(PYM_DMA_INPUT, &pym_dev->state);
			set_bit(VIO_GROUP_DMA_INPUT, &group->state);
		}
	}
		
	pym_update_param(pym_ctx);

	set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);

	vio_group_task_start(group->gtask);

	pym_ctx->state = BIT(VIO_VIDEO_INIT);

	return ret;
}

int pym_video_streamon(struct pym_video_ctx *pym_ctx)
{
	struct x3_pym_dev *pym_dev;
	unsigned long flags;

	pym_dev = pym_ctx->pym_dev;

	if (!(pym_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS)
			| BIT(VIO_VIDEO_INIT)))) {
		vio_err("[V%02d] invalid STREAM ON is requested(%lX)",
			pym_ctx->group->instance, pym_ctx->state);
		return -EINVAL;
	}

	if (atomic_read(&pym_dev->rsccount) > 0)
		goto p_inc;

	spin_lock_irqsave(&pym_dev->shared_slock, flags);

	pym_hw_enable(pym_dev, true);

	spin_unlock_irqrestore(&pym_dev->shared_slock, flags);

p_inc:
	atomic_inc(&pym_dev->rsccount);
	pym_ctx->state = BIT(VIO_VIDEO_START);

	return 0;
}

int pym_video_streamoff(struct pym_video_ctx *pym_ctx)
{
	struct x3_pym_dev *pym_dev;
	unsigned long flag;

	if (!(pym_ctx->state & BIT(VIO_VIDEO_START))) {
		vio_err("[V%02d] invalid STREAMOFF is requested(%lX)",
			pym_ctx->group->instance, pym_ctx->state);
		return -EINVAL;
	}
	pym_dev = pym_ctx->pym_dev;

	if (atomic_dec_return(&pym_dev->rsccount) > 0)
		goto p_dec;

	spin_lock_irqsave(&pym_dev->shared_slock, flag);
	pym_hw_enable(pym_dev, false);

	spin_unlock_irqrestore(&pym_dev->shared_slock, flag);
p_dec:

	frame_manager_flush(&pym_ctx->framemgr);

	pym_ctx->state = BIT(VIO_VIDEO_STOP);

	return 0;
}

int pym_video_s_stream(struct pym_video_ctx *pym_ctx, bool enable)
{
	int ret = 0;

	if (enable)
		ret = pym_video_streamon(pym_ctx);
	else
		ret = pym_video_streamoff(pym_ctx);

	return ret;
}

int pym_video_reqbufs(struct pym_video_ctx *pym_ctx, u32 buffers)
{
	int ret = 0;
	int i = 0;
	struct vio_framemgr *framemgr;

	if (!(pym_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS) |
		      BIT(VIO_VIDEO_INIT)| BIT(VIO_VIDEO_S_INPUT)))) {
		vio_err("[V%02d] invalid REQBUFS is requested(%lX)",
			pym_ctx->group->instance, pym_ctx->state);
		return -EINVAL;
	}

	framemgr = &pym_ctx->framemgr;
	ret = frame_manager_open(framemgr, buffers);
	if (ret) {
		vio_err("frame manage open failed, ret(%d)", ret);
		return ret;
	}

	for (i = 0; i < buffers; i++) {
		framemgr->frames[i].data = pym_ctx->group;
	}

	pym_ctx->state = BIT(VIO_VIDEO_REBUFS);

	return ret;
}

int pym_video_qbuf(struct pym_video_ctx *pym_ctx, struct frame_info *frameinfo)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	unsigned long flags;
	int index;

	index = frameinfo->bufferindex;
	framemgr = &pym_ctx->framemgr;
	BUG_ON(index >= framemgr->num_frames);

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = &framemgr->frames[index];
	if (frame->state == FS_FREE) {
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_REQUEST);
	} else {
		vio_err("frame(%d) is invalid state(%d)\n", index,
			frame->state);
		ret = -EINVAL;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	group = pym_ctx->group;
	if(group->leader == true)
		vio_group_start_trigger(group->gtask, frame);

	return ret;

}

int pym_video_dqbuf(struct pym_video_ctx *pym_ctx, struct frame_info *frameinfo)
{
	int ret = 0;
	struct list_head *done_list;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;

	framemgr = &pym_ctx->framemgr;

	done_list = &framemgr->queued_list[FS_COMPLETE];
	wait_event_interruptible(pym_ctx->done_wq, !list_empty(done_list));

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_COMPLETE);
	if (frame) {
		memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_FREE);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	return ret;
}

static long x3_pym_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int ret = 0;
	int buffers = 0;
	int enable = 0;
	int instance = 0;
	struct pym_video_ctx *pym_ctx;
	struct frame_info frameinfo;
	struct vio_group *group;

	pym_ctx = file->private_data;
	BUG_ON(!pym_ctx);
	group = pym_ctx->group;

	if (_IOC_TYPE(cmd) != PYM_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case PYM_IOC_INIT:
		ret = pym_video_init(pym_ctx, arg);
		break;
	case PYM_IOC_STREAM:
		ret = get_user(enable, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		vio_info("V[%d]=====PYM_IOC_STREAM==enable %d===========\n",
			 group->instance, enable);
		pym_video_s_stream(pym_ctx, ! !enable);
		break;
	case PYM_IOC_DQBUF:
		pym_video_dqbuf(pym_ctx, &frameinfo);
		ret = copy_to_user((void __user *) arg, (char *) &frameinfo,
				 sizeof(struct frame_info));
		vio_info("V[%d]=====PYM_IOC_DQBUF==ret %d===========\n",
			 group->instance, ret);
		if (ret)
			return -EFAULT;
		break;
	case PYM_IOC_QBUF:
		ret = copy_from_user((char *) &frameinfo, (u32 __user *) arg,
				   sizeof(struct frame_info));
		vio_info("V[%d]=====PYM_IOC_QBUF==ret %d===========\n",
			 group->instance, ret);
		if (ret)
			return -EFAULT;
		pym_video_qbuf(pym_ctx, &frameinfo);
		break;
	case PYM_IOC_REQBUFS:
		ret = get_user(buffers, (u32 __user *) arg);
		vio_info("V[%d]=====PYM_IOC_REQBUFS==ret %d===========\n",
			 group->instance, ret);
		if (ret)
			return -EFAULT;
		pym_video_reqbufs(pym_ctx, buffers);
		break;
	case PYM_IOC_END_OF_STREAM:
		ret = get_user(instance, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		vio_bind_group_done(instance);
		break;
	case PYM_IOC_BIND_GROUP:
		ret = get_user(instance, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		ret = pym_bind_chain_group(pym_ctx, instance);
		break;
	default:
		vio_err("wrong ioctl command\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

void pym_set_iar_output(struct pym_video_ctx *pym_ctx, struct vio_frame *frame)
{
#ifdef X3_IAR_INTERFACE
	struct special_buffer *spec;
	u32 display_layer = 0;

	display_layer = ipu_get_iar_display_type();
	spec = &frame->frameinfo.spec;
	if (display_layer >= 31)
		ipu_set_display_addr(spec->us_y_addr[display_layer - 31],
			spec->us_uv_addr[display_layer - 31]);
	else if (display_layer >= 7)
		ipu_set_display_addr(spec->ds_y_addr[display_layer - 7],
			spec->ds_uv_addr[display_layer - 7]);
#endif
}

void pym_frame_done(struct pym_video_ctx *pym_ctx)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	unsigned long flags;

	group = pym_ctx->group;
	framemgr = &pym_ctx->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		if(group->get_timestamps){
			frame->frameinfo.frame_id = group->frameid.frame_id;
			frame->frameinfo.timestamps =
			    group->frameid.timestamps;
		}

		do_gettimeofday(&frame->frameinfo.tv);

		pym_set_iar_output(pym_ctx, frame);
		pym_ctx->event = VIO_FRAME_DONE;
		trans_frame(framemgr, frame, FS_COMPLETE);
	} else {
		pym_ctx->event = VIO_FRAME_NDONE;
		vio_err("%s PROCESS queue has no member;\n", __func__);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	wake_up(&pym_ctx->done_wq);
}

static irqreturn_t pym_isr(int irq, void *data)
{
	u32 status;
	u32 instance;
	struct x3_pym_dev *pym;
	struct vio_group *group;
	struct vio_group_task *gtask;

	pym = data;
	gtask = &pym->gtask;
	instance = atomic_read(&pym->instance);
	group = pym->group[instance];
	pym_get_intr_status(pym->base_reg, &status, true);
	vio_info("%s:status = 0x%x\n", __func__, status);

	if (status & (1 << INTR_PYM_DS_FRAME_DROP))
		vio_err("DS drop frame\n");

	if (status & (1 << INTR_PYM_US_FRAME_DROP))
		vio_err("US drop frame\n");

	if (status & (1 << INTR_PYM_FRAME_DONE)) {
		if (test_bit(PYM_DMA_INPUT, &pym->state)) {
			up(&gtask->hw_resource);
		}
		pym_frame_done(group->sub_ctx[GROUP_ID_SRC]);
	}

	if (status & (1 << INTR_PYM_FRAME_START)) {
		if (test_bit(PYM_OTF_INPUT, &pym->state)) {
			up(&gtask->hw_resource);
		}
		if (group && group->get_timestamps)
			vio_get_frame_id(group);
	}

	return IRQ_HANDLED;
}

static struct file_operations x3_pym_fops = {
	.owner = THIS_MODULE,
	.open = x3_pym_open,
	.write = x3_pym_write,
	.read = x3_pym_read,
	.poll = x3_pym_poll,
	.release = x3_pym_close,
	.unlocked_ioctl = x3_pym_ioctl,
	.compat_ioctl = x3_pym_ioctl,
};

static int x3_pym_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_pym_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_pym_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_pym_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x3_pym_pm_ops = {
	.suspend = x3_pym_suspend,
	.resume = x3_pym_resume,
	.runtime_suspend = x3_pym_runtime_suspend,
	.runtime_resume = x3_pym_runtime_resume,
};

int x3_pym_device_node_init(struct x3_pym_dev *pym)
{
	int ret = 0;
	dev_t devno;
	struct device *dev = NULL;

	ret = alloc_chrdev_region(&devno, 0, MAX_DEVICE, "x3_pym");
	if (ret < 0) {
		vio_err("Error %d while alloc chrdev pym", ret);
		goto err_req_cdev;
	}

	cdev_init(&pym->cdev, &x3_pym_fops);
	pym->cdev.owner = THIS_MODULE;
	ret = cdev_add(&pym->cdev, devno, MAX_DEVICE);
	if (ret) {
		vio_err("Error %d while adding x2 pym cdev", ret);
		goto err;
	}

	if (vps_class)
		pym->class = vps_class;
	else
		pym->class = class_create(THIS_MODULE, X3_PYM_NAME);

	dev = device_create(pym->class, NULL, MKDEV(MAJOR(devno), 0),
					NULL, "pym");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		vio_err("pym device create fail\n");
		goto err;
	}

	return ret;
err:
	class_destroy(pym->class);
err_req_cdev:
	unregister_chrdev_region(devno, MAX_DEVICE);
	return ret;
}

static ssize_t pym_reg_dump(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_pym_dev *pym;

	pym = dev_get_drvdata(dev);

	pym_hw_dump(pym->base_reg);

	return 0;
}

static DEVICE_ATTR(regdump, 0444, pym_reg_dump, NULL);

static int x3_pym_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x3_pym_dev *pym;
	struct resource *mem_res;
	struct device *dev = NULL;

	pym = kzalloc(sizeof(struct x3_pym_dev), GFP_KERNEL);
	if (!pym) {
		vio_err("pym is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		vio_err("Failed to get io memory region(%p)", mem_res);
		ret = -EBUSY;
		goto err_get_resource;
	}

	pym->regs_start = mem_res->start;
	pym->regs_end = mem_res->end;
	pym->base_reg = devm_ioremap_nocache(&pdev->dev,
					mem_res->start,
					resource_size(mem_res));
	if (!pym->base_reg) {
		vio_err("Failed to remap io region(%p)", pym->base_reg);
		ret = -ENOMEM;
		goto err_get_resource;
	}

	/* Get IRQ SPI number */
	pym->irq = platform_get_irq(pdev, 0);
	if (pym->irq < 0) {
		vio_err("Failed to get csi_irq(%d)", pym->irq);
		ret = -EBUSY;
		goto err_get_irq;
	}

	ret = request_irq(pym->irq, pym_isr, IRQF_TRIGGER_HIGH, "pym", pym);
	if (ret) {
		vio_err("request_irq(IRQ_PYM %d) is fail(%d)", pym->irq, ret);
		goto err_get_irq;
	}

	x3_pym_device_node_init(pym);

	dev = &pdev->dev;
	ret = device_create_file(dev, &dev_attr_regdump);
	if(ret < 0) {
		vio_err("create regdump failed (%d)\n",ret);
		goto p_err;
	}
	platform_set_drvdata(pdev, pym);

	spin_lock_init(&pym->shared_slock);
	sema_init(&pym->gtask.hw_resource, 1);
	atomic_set(&pym->gtask.refcount, 0);
	atomic_set(&pym->rsccount, 0);
	atomic_set(&pym->open_cnt, 0);

	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

	return 0;

err_get_irq:
	iounmap(pym->base_reg);

err_get_resource:
	kfree(pym);
p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;

}

static int x3_pym_remove(struct platform_device *pdev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x3_pym_match[] = {
	{
	 .compatible = "hobot,x3-pym",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, x3_pym_match);

static struct platform_driver x3_pym_driver = {
	.probe = x3_pym_probe,
	.remove = x3_pym_remove,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_pym_pm_ops,
		   .of_match_table = x3_pym_match,
		   }
};

#else
static struct platform_device_id x3_pym_driver_ids[] = {
	{
	 .name = MODULE_NAME,
	 .driver_data = 0,
	 },
	{},
};

MODULE_DEVICE_TABLE(platform, x3_pym_driver_ids);

static struct platform_driver x3_pym_driver = {
	.probe = x3_pym_probe,
	.remove = __devexit_p(x3_pym_remove),
	.id_table = x3_pym_driver_ids,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_pym_pm_ops,
		   }
};
#endif

static int __init x3_pym_init(void)
{
	int ret = platform_driver_register(&x3_pym_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

late_initcall(x3_pym_init);

static void __exit x3_pym_exit(void)
{
	platform_driver_unregister(&x3_pym_driver);
}

module_exit(x3_pym_exit);

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X3 PYM driver");
MODULE_LICENSE("GPL");
