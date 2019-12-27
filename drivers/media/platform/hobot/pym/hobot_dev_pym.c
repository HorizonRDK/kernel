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

#define MODULE_NAME "X2A PYM"

#if CONFIG_QEMU_TEST
static struct timer_list tm[4];
static int g_test_bit = 0;
static int g_int_test = -1;
static int timer_init(struct x2a_pym_dev *pym, int index);
#endif

static int x2a_pym_open(struct inode *inode, struct file *file)
{
	struct pym_video_ctx *pym_ctx;
	struct x2a_pym_dev *pym;
	int ret = 0;
	int minor;

	minor = MINOR(inode->i_rdev);

	pym = container_of(inode->i_cdev, struct x2a_pym_dev, cdev);
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

p_err:
	return ret;
}

static ssize_t x2a_pym_write(struct file *file, const char __user *buf,
			     size_t count, loff_t * ppos)
{
	return 0;
}

static ssize_t x2a_pym_read(struct file *file, char __user *buf, size_t size,
			    loff_t * ppos)
{
	return 0;

}

static u32 x2a_pym_poll(struct file *file, struct poll_table_struct *wait)
{
	int ret = 0;
	struct pym_video_ctx *pym_ctx;

	pym_ctx = file->private_data;

	poll_wait(file, &pym_ctx->done_wq, wait);
	if(pym_ctx->event == VIO_FRAME_DONE)
		ret = POLLIN;
	else if(pym_ctx->event == VIO_FRAME_NDONE)
		ret = POLLERR;

	pym_ctx->event = 0;

	return ret;
}

static int x2a_pym_close(struct inode *inode, struct file *file)
{

	struct pym_video_ctx *pym_ctx;
	struct vio_group *group;
	struct x2a_pym_dev *pym;

	pym_ctx = file->private_data;
	group = pym_ctx->group;
	pym = pym_ctx->pym_dev;

	clear_bit(VIO_GROUP_INIT, &group->state);

	vio_group_task_stop(&pym->gtask);

	pym_ctx->state = BIT(VIO_VIDEO_CLOSE);

	frame_manager_close(&pym_ctx->framemgr);

	kfree(pym_ctx);

	vio_info("PYM close node %d\n", group->instance);
	return 0;
}

void pym_set_buffers(struct x2a_pym_dev *pym, struct vio_frame *frame)
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
	struct x2a_pym_dev *pym;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	u32 instance = 0;
	u8 shadow_index = 0;

	instance = group->instance;
	ctx = group->sub_ctx[0];
	pym = ctx->pym_dev;
	shadow_index = instance % MAX_SHADOW_NUM;

	pym_set_shd_rdy(pym->base_reg, shadow_index, 0);
	pym_set_shd_select(pym->base_reg, shadow_index);

	atomic_set(&pym->instance, instance);

	framemgr = &ctx->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame) {
		pym_set_buffers(pym, frame);

		if (test_bit(PYM_DMA_INPUT, &pym->state)) {
			pym_rdma_set_addr(pym->base_reg,
					  frame->frameinfo.addr[0],
					  frame->frameinfo.addr[1]);
			pym_set_rdma_start(pym->base_reg);
		}

		trans_frame(framemgr, frame, FS_PROCESS);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	//pym_hw_dump(pym->base_reg);
	pym_set_shd_rdy(pym->base_reg, shadow_index, 1);
	//msleep(10000);

	return;
}

void pym_update_param(struct pym_video_ctx *pym_ctx, pym_cfg_t *pym_config)
{
	u16 src_width, src_height, roi_width;
	u32 shadow_index;
	int i = 0;
	u32 base_layer_nums = 0;
	struct vio_group *group;
	struct x2a_pym_dev *pym;
	struct roi_rect rect;

	group = pym_ctx->group;
	shadow_index = group->instance % MAX_SHADOW_NUM;
	pym = pym_ctx->pym_dev;

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

	base_layer_nums = pym_config->ds_layer_en / 4;
	pym_ds_enabe_base_layer(pym->base_reg, shadow_index, base_layer_nums);

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
	pym_set_shd_rdy(pym->base_reg, shadow_index, 1);

	//config common register
	pym_set_common_rdy(pym->base_reg, 0);

	pym_set_intr_mask(pym->base_reg, 0);

	pym_select_input_path(pym->base_reg, pym_config->img_scr);

	pym_set_common_rdy(pym->base_reg, 1);

}

int pym_bind_chain_group(struct pym_video_ctx *pym_ctx, int instance)
{
	int ret = 0;
	struct vio_group *group;
	struct x2a_pym_dev *pym;
	struct vio_chain *chain;

	if (!(pym_ctx->state & BIT(VIO_VIDEO_OPEN))) {
		vio_err("[%s]invalid BIND is requested(%lX)",
			__func__, pym_ctx->state);
		return -EINVAL;
	}

	if(instance < 0 && instance >= VIO_MAX_STREAM){
		vio_err("wrong instance id(%d)\n", instance);
		return -EFAULT;
	}

	pym = pym_ctx->pym_dev;

	group = vio_get_chain_group(instance, GROUP_ID_PYM);
	group->sub_ctx[0] = pym_ctx;
	pym->group[instance] = group;
	pym_ctx->group = group;

	group->frame_work = pym_frame_work;
	group->gtask = &pym->gtask;

	chain = group->chain;

	vio_info("[%s]instance = %d\n", __func__, instance);
	pym_ctx->state = BIT(VIO_VIDEO_S_INPUT);

	return ret;
}

int pym_video_init(struct pym_video_ctx *pym_ctx, unsigned long arg)
{
	pym_cfg_t pym_config;
	struct vio_group *group;
	struct x2a_pym_dev *pym_dev;
	int ret = 0;

	group = pym_ctx->group;
	pym_dev = pym_ctx->pym_dev;

	if (!(pym_ctx->state & (BIT(VIO_VIDEO_S_INPUT) | BIT(VIO_VIDEO_REBUFS)))) {
		vio_err("[V%02d] invalid INIT is requested(%lX)",
			group->instance, pym_ctx->state);
		return -EINVAL;
	}

	ret = copy_from_user((char *) &pym_config,
				(u32 __user *) arg, sizeof(pym_cfg_t));
	if (ret)
		return -EFAULT;

	if(pym_config.img_scr == 1){
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
		
	pym_update_param(pym_ctx, &pym_config);

	vio_group_task_start(&pym_dev->gtask);	//TODO:move to open function;

	pym_ctx->state = BIT(VIO_VIDEO_INIT);

	return ret;
}

int pym_video_streamon(struct pym_video_ctx *pym_ctx)
{
	struct x2a_pym_dev *pym_dev;
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

	pym_set_common_rdy(pym_dev->base_reg, 0);

	pym_enable_module(pym_dev->base_reg, true);

	pym_set_common_rdy(pym_dev->base_reg, 1);

#if CONFIG_QEMU_TEST
	timer_init(pym_dev, 0);
#endif

	spin_unlock_irqrestore(&pym_dev->shared_slock, flags);

p_inc:
#if CONFIG_QEMU_TEST
	g_test_bit |= 1 << INTR_PYM_FRAME_START | 1 << INTR_PYM_FRAME_DONE;
#endif

	atomic_inc(&pym_dev->rsccount);
	pym_ctx->state = BIT(VIO_VIDEO_START);

	return 0;
}

int pym_video_streamoff(struct pym_video_ctx *pym_ctx)
{
	struct x2a_pym_dev *pym_dev;
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

	clear_bit(PYM_OTF_INPUT, &pym_dev->state);

	pym_set_common_rdy(pym_dev->base_reg, 0);

	pym_enable_module(pym_dev->base_reg, false);

	pym_set_common_rdy(pym_dev->base_reg, 1);

#if CONFIG_QEMU_TEST
	del_timer_sync(&tm[0]);
	g_test_bit = 0;
#endif

	clear_bit(PYM_OTF_INPUT, &pym_dev->state);
	clear_bit(PYM_DMA_INPUT, &pym_dev->state);

	spin_unlock_irqrestore(&pym_dev->shared_slock, flag);
	vio_info("%s timer del\n", __func__);
p_dec:

	frame_manager_flush(&pym_ctx->framemgr);

	pym_ctx->state = BIT(VIO_VIDEO_STOP);

	return 0;
}

int pym_video_s_stream(struct pym_video_ctx *pym_ctx, bool enable)
{
	if (enable)
		pym_video_streamon(pym_ctx);
	else
		pym_video_streamoff(pym_ctx);

	return 0;
}

int pym_video_reqbufs(struct pym_video_ctx *pym_ctx, u32 buffers)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	int i = 0;

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
		vio_group_start_trigger(&pym_ctx->pym_dev->gtask, frame);

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

void pym_group_set_state(struct vio_group *group, unsigned long state)
{
	set_bit(state, &group->state);
}

static long x2a_pym_ioctl(struct file *file, unsigned int cmd,
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
	if(display_layer >= 31)
		ipu_set_display_addr(spec->ds_y_addr[display_layer - 31],
			spec->ds_uv_addr[display_layer - 31]);
	else if(display_layer >= 7)
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
			frame->frameinfo.timestamp_l =
			    group->frameid.timestamp_l;
			frame->frameinfo.timestamp_m =
			    group->frameid.timestamp_m;
		}

		pym_set_iar_output(pym_ctx, frame);
		pym_ctx->event = VIO_FRAME_DONE;
		trans_frame(framemgr, frame, FS_COMPLETE);
	}else {
		pym_ctx->event = VIO_FRAME_NDONE;
		vio_err("%s PROCESS queue has no member;\n", __func__);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	wake_up(&pym_ctx->done_wq);
}

static irqreturn_t pym_isr(int irq, void *data)
{
	u32 status;
	struct x2a_pym_dev *pym;
	u32 instance;
	struct vio_group *group;
	struct vio_group_task *gtask;

	pym = data;
	gtask = &pym->gtask;
	instance = atomic_read(&pym->instance);
	group = pym->group[instance];
	pym_get_intr_status(pym->base_reg, &status, true);
	vio_info("%s:status = 0x%x\n", __func__, status);

	if (status & (1 << INTR_PYM_FRAME_DONE)) {
		if (test_bit(PYM_DMA_INPUT, &pym->state)) {
			up(&gtask->hw_resource);
		}
		pym_frame_done(group->sub_ctx[GROUP_ID_SRC]);
	}

	if (status & (1 << INTR_PYM_FRAME_START)) {
		if (test_bit(PYM_OTF_INPUT, &pym->state))
			up(&gtask->hw_resource);

		if (group && group->get_timestamps)
			vio_get_frame_id(group);
	}

	if (status & (1 << INTR_PYM_DS_FRAME_DROP))
		vio_err("DS drop frame\n");

	if (status & (1 << INTR_PYM_US_FRAME_DROP))
		vio_err("US drop frame\n");

	return 0;
}

static struct file_operations x2a_pym_fops = {
	.owner = THIS_MODULE,
	.open = x2a_pym_open,
	.write = x2a_pym_write,
	.read = x2a_pym_read,
	.poll = x2a_pym_poll,
	.release = x2a_pym_close,
	.unlocked_ioctl = x2a_pym_ioctl,
	.compat_ioctl = x2a_pym_ioctl,
};

static int x2a_pym_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_pym_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_pym_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_pym_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x2a_pym_pm_ops = {
	.suspend = x2a_pym_suspend,
	.resume = x2a_pym_resume,
	.runtime_suspend = x2a_pym_runtime_suspend,
	.runtime_resume = x2a_pym_runtime_resume,
};

int x2a_pym_device_node_init(struct x2a_pym_dev *pym)
{
	int ret = 0;
	dev_t devno;
	struct device *dev = NULL;

	ret = alloc_chrdev_region(&devno, 0, MAX_DEVICE, "x2a_pym");
	if (ret < 0) {
		vio_err("Error %d while alloc chrdev pym", ret);
		goto err_req_cdev;
	}

	cdev_init(&pym->cdev, &x2a_pym_fops);
	pym->cdev.owner = THIS_MODULE;
	ret = cdev_add(&pym->cdev, devno, MAX_DEVICE);
	if (ret) {
		vio_err("Error %d while adding x2 pym cdev", ret);
		goto err;
	}

	pym->class = class_create(THIS_MODULE, X2A_PYM_NAME);

	dev =
	    device_create(pym->class, NULL, MKDEV(MAJOR(devno), 0), NULL,
			  "pym");
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

static ssize_t pym_reg_dump(struct device *dev,struct device_attribute *attr, char* buf)
{
	struct x2a_pym_dev *pym;

	pym = dev_get_drvdata(dev);

	pym_hw_dump(pym->base_reg);

	return 0;
}

static DEVICE_ATTR(regdump, 0444, pym_reg_dump, NULL);

static int x2a_pym_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x2a_pym_dev *pym;
	struct resource *mem_res;
	struct device *dev = NULL;

	pym = kzalloc(sizeof(struct x2a_pym_dev), GFP_KERNEL);
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
	pym->base_reg =
	    devm_ioremap_nocache(&pdev->dev, mem_res->start,
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

	x2a_pym_device_node_init(pym);

	dev = &pdev->dev;
	ret = device_create_file(dev, &dev_attr_regdump);
	if(ret < 0) {
		vio_err("create regdump failed (%d)\n",ret);
		goto p_err;
	}
	platform_set_drvdata(pdev, pym);

	atomic_set(&pym->gtask.refcount, 0);
	spin_lock_init(&pym->shared_slock);
	sema_init(&pym->gtask.hw_resource, 1);

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

static int x2a_pym_remove(struct platform_device *pdev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x2a_pym_match[] = {
	{
	 .compatible = "hobot,x2a-pym",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, x2a_pym_match);

static struct platform_driver x2a_pym_driver = {
	.probe = x2a_pym_probe,
	.remove = x2a_pym_remove,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x2a_pym_pm_ops,
		   .of_match_table = x2a_pym_match,
		   }
};

#else
static struct platform_device_id x2a_pym_driver_ids[] = {
	{
	 .name = MODULE_NAME,
	 .driver_data = 0,
	 },
	{},
};

MODULE_DEVICE_TABLE(platform, x2a_pym_driver_ids);

static struct platform_driver x2a_pym_driver = {
	.probe = x2a_pym_probe,
	.remove = __devexit_p(x2a_pym_remove),
	.id_table = x2a_pym_driver_ids,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x2a_pym_pm_ops,
		   }
};
#endif

#if CONFIG_QEMU_TEST
static void pym_timer(unsigned long data)
{
	u32 status;
	struct x2a_pym_dev *pym;
	u32 instance;
	struct vio_group *group;
	struct vio_group_task *gtask;

	pym = (struct x2a_pym_dev *) data;
	gtask = &pym->gtask;
	instance = atomic_read(&pym->instance);
	group = pym->group[instance];
	pym_get_intr_status(pym->base_reg, &status, true);
	status = g_test_bit;
	mod_timer(&tm[0], jiffies + HZ / 25);

	vio_info("%s:%d\n", __func__, status);
	if (status & (1 << INTR_PYM_FRAME_DONE)) {
		if (!test_bit(PYM_OTF_INPUT, &pym->state)) {
			up(&gtask->hw_resource);
		}
		pym_frame_done(group->sub_ctx[GROUP_ID_SRC]);
	}

	if (status & (1 << INTR_PYM_FRAME_START)) {
		if (test_bit(PYM_OTF_INPUT, &pym->state))
			up(&gtask->hw_resource);
	}
}

static int timer_init(struct x2a_pym_dev *pym, int index)
{
	init_timer(&tm[index]);
	tm[index].expires = jiffies + HZ / 10;
	tm[index].function = pym_timer;
	tm[index].data = (unsigned long) pym;
	add_timer(&tm[index]);
}

static int __init x2a_pym_init(void)
{
	int ret = 0;
	struct x2a_pym_dev *pym;

	pym = kzalloc(sizeof(struct x2a_pym_dev), GFP_KERNEL);
	if (!pym) {
		vio_err("pym is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	pym->base_reg = kzalloc(0x1000, GFP_KERNEL);

	x2a_pym_device_node_init(pym);
	atomic_set(&pym->gtask.refcount, 0);
	spin_lock_init(&pym->shared_slock);
	sema_init(&pym->gtask.hw_resource, 1);

	vio_info("[FRT:D] %s(%d)\n", __func__, ret);
	atomic_set(&pym->rsccount, 0);

	return 0;

p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;
}

#else

static int __init x2a_pym_init(void)
{
	int ret = platform_driver_register(&x2a_pym_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}
#endif

late_initcall(x2a_pym_init);

static void __exit x2a_pym_exit(void)
{
	platform_driver_unregister(&x2a_pym_driver);
}

module_exit(x2a_pym_exit);

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X2A PYM driver");
MODULE_LICENSE("GPL");
