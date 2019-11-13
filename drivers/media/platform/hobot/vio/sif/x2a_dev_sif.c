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

#include "x2a_dev_sif.h"
#include "sif_hw_api.h"


#define MODULE_NAME "X2A SIF"

struct timer_list tm[4];
int g_test_bit = 0;
int g_test_input = 0;
int g_test = -1;

static int timer_init(struct x2a_sif_dev *sif, int index);

static int x2a_sif_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_sif_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_sif_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_sif_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x2a_sif_pm_ops = {
	.suspend		= x2a_sif_suspend,
	.resume			= x2a_sif_resume,
	.runtime_suspend	= x2a_sif_runtime_suspend,
	.runtime_resume		= x2a_sif_runtime_resume,
};

struct sif_group *sif_get_mux_group(struct x2a_sif_dev *sif, u32 mux_index)
{
	struct sif_group *group;

	if (mux_index >= 8){
		vio_err("wrong mux_index(%d)\n", mux_index);
		goto p_err;
	}

	group = &sif->sif_mux[mux_index];
	if (!test_bit(SIF_GROUP_OPEN, &group->state)) {
		set_bit(SIF_GROUP_OPEN, &group->state);
		return group;
	}

p_err:
	return NULL;
}

struct sif_group *sif_get_input_group(struct x2a_sif_dev *sif)
{
	u32 stream;
	struct sif_group *group;
	for (stream = 0; stream < VIO_MAX_STREAM; ++stream) {
		group = &sif->sif_input[stream];
		if (!test_bit(SIF_GROUP_OPEN, &group->state)) {
			group->instance = stream;
			set_bit(SIF_GROUP_OPEN, &group->state);
			return group;
		}
	}
	return NULL;
}

static void sif_read_work_function(struct kthread_work *work)
{
	struct sif_video_ctx *ctx;
	struct x2a_sif_dev *sif;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct frame_info *frameinfo;
	struct sif_group *group;
	struct vio_group_task *gtask;
	unsigned long flags;
	u32 instance = 0;
	bool try_rdown = false;
	int ret = 0;

	frame = container_of(work, struct vio_frame, work);
	group = (struct sif_group *)frame->data;
	instance = group->instance;
	ctx = group->sub_ctx;
	sif = ctx->sif_dev;
	gtask = &sif->sifin_task;

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

	atomic_set(&sif->instance, instance);

	framemgr = &ctx->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0 ,flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if(frame){
		frameinfo = &frame->frameinfo;
		sif_config_rdma_fmt(sif->base_reg, frameinfo->format, 
			frameinfo->width, frameinfo->height);
		sif_set_rdma_buf_addr(sif->base_reg, 0, frame->frameinfo.addr[0]);
		sif_set_rdma_trigger(sif->base_reg, 1);
		trans_frame(framemgr, frame, FS_PROCESS);
	}
	framemgr_x_barrier_irqr(framemgr, 0 ,flags);

	clear_bit(VIO_GTASK_SHOT, &gtask->state);

	return;

p_err_ignore:
	if(try_rdown)
		up(&gtask->hw_resource);

	clear_bit(VIO_GTASK_SHOT, &gtask->state);
	return;
}

static void sif_write_work_function(struct kthread_work *work)
{
	struct sif_video_ctx *ctx;
	struct x2a_sif_dev *sif;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct sif_group *group;
	struct vio_group_task *gtask;
	unsigned long flags;
	u32 buf_index, mux_index;
	u32 yuv_format = 0;
	bool try_rdown = false;
	int ret =0;

	frame = container_of(work, struct vio_frame, work);
	group = (struct sif_group *)frame->data;
	ctx = group->sub_ctx;
	sif = ctx->sif_dev;
	mux_index = ctx->mux_index;
	gtask = &sif->sifout_task[mux_index];
	
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

	framemgr = &ctx->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0 ,flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if(frame){
		buf_index = ctx->bufcount % 4;
		yuv_format = (frame->frameinfo.format == SIF_YUV_INPUT); 

		sif_set_wdma_buf_addr(sif->base_reg, mux_index, buf_index, frame->frameinfo.addr[0]);
		sif_transfer_ddr_owner(sif->base_reg, mux_index, buf_index);
		if(yuv_format){
			sif_set_wdma_buf_addr(sif->base_reg, mux_index + 1, buf_index, frame->frameinfo.addr[1]);
			sif_transfer_ddr_owner(sif->base_reg, mux_index + 1, buf_index);
		}

		trans_frame(framemgr, frame, FS_PROCESS);
		ctx->bufcount++;
	}
	framemgr_x_barrier_irqr(framemgr, 0 ,flags);

	clear_bit(VIO_GTASK_SHOT, &gtask->state);

	return;

p_err_ignore:
	if(try_rdown)
		up(&gtask->hw_resource);

	clear_bit(VIO_GTASK_SHOT, &gtask->state);
	return;
}

static int x2a_sif_open(struct inode *inode, struct file *file)
{
	struct sif_video_ctx *sif_ctx;
	struct x2a_sif_dev *sif;
	struct sif_group *group;
	int ret = 0;
	int minor;

	minor = MINOR(inode->i_rdev);

	sif = container_of(inode->i_cdev, struct x2a_sif_dev, cdev);
	sif_ctx = kzalloc(sizeof(struct sif_video_ctx), GFP_KERNEL);
	if (sif_ctx == NULL) {
		vio_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	if(minor == 1){
		group = sif_get_input_group(sif);
		if (group == NULL){
			vio_err("can get free group\n");
			goto p_err;
		}

		group->id = minor;
		group->sub_ctx = sif_ctx;
		sif_ctx->group = group;
	}

	init_waitqueue_head(&sif_ctx->done_wq);
	sif_ctx->id = minor;
	sif_ctx->sif_dev = sif;
	file->private_data = sif_ctx;
	sif_ctx->state = BIT(VIO_VIDEO_OPEN);
	vio_info("SIF open node %d\n",minor);
p_err:
	return 0;
}
static ssize_t x2a_sif_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t x2a_sif_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	return 0;
}

static int x2a_sif_close(struct inode *inode, struct file *file)
{
	struct sif_video_ctx *sif_ctx;
	struct sif_group *group;
	struct x2a_sif_dev *sif;

	sif_ctx = file->private_data;
	group = sif_ctx->group;
	sif = sif_ctx->sif_dev;

	clear_bit(SIF_GROUP_OPEN, &group->state);

	if(sif_ctx->id == 0){
		vio_group_task_stop(&sif->sifout_task[sif_ctx->mux_index]);
	}else if(sif_ctx->id == 1){
		vio_group_task_stop(&sif->sifin_task);
	}


	frame_manager_close(&sif_ctx->framemgr);

	sif_ctx->state = BIT(VIO_VIDEO_CLOSE);

	kfree(sif_ctx);

	vio_info("SIF close node %d\n", sif_ctx->id);
	return 0;
}

int sif_send_isp_signal(void)
{
	return 0;
}

int sif_mux_init(struct sif_video_ctx *sif_ctx, unsigned long arg)
{
	int ret = 0;
	u32 mux_index = 0;
	u32 isp_flyby = 0, ipu_flyby = 0;
	sif_cfg_t sif_config;
	struct sif_group *group;
	struct x2a_sif_dev *sif;
	struct vio_group_task *gtask;

	ret = copy_from_user((char *)&sif_config, (u32 __user *)arg, sizeof(sif_cfg_t));
	if(ret)
		return -EFAULT;

	sif = sif_ctx->sif_dev;

	mux_index = sif_config.input.mipi.func.set_mux_out_index;
	sif_ctx->mux_index = mux_index;
	group = sif_get_mux_group(sif, mux_index);
	if (group == NULL){
		vio_err("can get free group\n");
		return -EFAULT;
	}

	isp_flyby = sif_config.output.isp.enable;
	ipu_flyby = sif_config.output.ipu.enable_flyby;

	if(isp_flyby == 1 || ipu_flyby == 1)
		set_bit(SIF_OTF_OUTPUT, &sif->state);

	if(sif_config.input.mipi.data.format == SIF_YUV_INPUT){
		if(mux_index % 2 == 0){
			set_bit(mux_index + 1, &sif->state);
			vio_info("sif input format is yuv, and current mux = %d\n", mux_index);
		}else
			vio_err("sif input format is yuv, but mux is wrong = %d\n", mux_index);
	}

	group->id = sif_ctx->id;
	group->sub_ctx = sif_ctx;
	sif_ctx->group = group;
	gtask = &sif->sifout_task[mux_index];

	sema_init(&gtask->hw_resource, 4);
	vio_group_task_start(gtask);

	sif_hw_config(sif->base_reg, &sif_config);

	if(test_bit(SIF_OTF_OUTPUT, &sif->state))
		sif_disable_wdma(sif->base_reg);

	sif_ctx->bufcount = 0;

	vio_info("%s,mux index = %d\n", __func__, mux_index);
	return 0;
}
int sif_video_init(struct sif_video_ctx *sif_ctx, unsigned long arg)
{

	struct x2a_sif_dev *sif;
	int ret = 0;

	if(!(sif_ctx->state & (BIT(VIO_VIDEO_OPEN) | BIT(VIO_VIDEO_REBUFS)))){
		vio_err("[%s][V%02d] invalid INIT is requested(%lX)", __func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	sif = sif_ctx->sif_dev;

	if(sif_ctx->id == 0){
		sif_mux_init(sif_ctx, arg);
	}else if(sif_ctx->id == 1){
		vio_group_task_start(&sif->sifin_task);
	}

	vio_info("V[%d]%s ret(%d)\n", sif_ctx->id, __func__, ret);

	sif_ctx->state = BIT(VIO_VIDEO_INIT);

	return ret;
}

int sif_video_streamon(struct sif_video_ctx *sif_ctx)
{
	struct x2a_sif_dev *sif_dev;
	sif_dev = sif_ctx->sif_dev;
	unsigned long flag;

	if(!(sif_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS) | BIT(VIO_VIDEO_INIT)))){
		vio_err("[%s][V%02d] invalid STREAM ON is requested(%lX)", __func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	if(atomic_read(&sif_dev->rsccount) > 0)
		goto p_inc;

	//sif_hw_dump(sif_dev->base_reg);

	spin_lock_irqsave(&sif_dev->shared_slock, flag);
	sif_hw_enable(sif_dev->base_reg);
	spin_unlock_irqrestore(&sif_dev->shared_slock, flag);	

p_inc:
	atomic_inc(&sif_dev->rsccount);

#if CONFIG_QEMU_TEST
	if(sif_ctx->id == 0)
		g_test_bit |= 1 << sif_ctx->mux_index | 1 << (sif_ctx->mux_index + 8);
	else
		g_test_input = 1;
	timer_init(sif_dev, 0);
#endif
	sif_ctx->state = BIT(VIO_VIDEO_START);

	vio_dbg("%s g_test_bit = %d\n", __func__, g_test_bit);
	return 0;
}


int sif_video_streamoff(struct sif_video_ctx *sif_ctx)
{
	struct x2a_sif_dev *sif_dev;
	struct vio_framemgr *framemgr;
	unsigned long flag;

	if(!(sif_ctx->state & BIT(VIO_VIDEO_START))){
		vio_err("[%s][V%02d] invalid STREAM OFF is requested(%lX)", __func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	sif_dev = sif_ctx->sif_dev;
	framemgr = &sif_ctx->framemgr;

	if(atomic_dec_return(&sif_dev->rsccount) > 0)
		goto p_dec;

#if CONFIG_QEMU_TEST
	del_timer_sync(&tm[0]);
#endif

	spin_lock_irqsave(&sif_dev->shared_slock, flag);

	sif_hw_disable(sif_dev->base_reg);
	sif_hw_dump(sif_dev->base_reg);
	spin_unlock_irqrestore(&sif_dev->shared_slock, flag);

p_dec:

	clear_bit(sif_ctx->mux_index + 1, &sif_dev->state);

	frame_manager_flush(framemgr);
	sif_ctx->state = BIT(VIO_VIDEO_STOP);

	vio_dbg("%s\n", __func__);
	g_test_input = 0;
	g_test_bit = 0;
	g_test = -1;
	return 0;
}

int sif_video_s_stream(struct sif_video_ctx *sif_ctx, bool enable)
{
	if(enable)
		sif_video_streamon(sif_ctx);
	else
		sif_video_streamoff(sif_ctx);

	return 0;
}

int sif_video_reqbufs(struct sif_video_ctx *sif_ctx, u32 buffers)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct sif_group *group;
	int i;

	if(!(sif_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS) | BIT(VIO_VIDEO_INIT)))){
		vio_err("[%s][V%02d] invalid REQBUFS is requested(%lX)", __func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	vio_info("%s: buffers = %d\n", __func__, buffers);
	framemgr = &sif_ctx->framemgr;
	ret = frame_manager_open(framemgr, buffers);
	if(ret){
		vio_err("frame manage open failed, ret(%d)\n",ret);
		return ret;
	}

	group = sif_ctx->group;
	for(i = 0; i < buffers; i++){
		if(group->id == 1)
			kthread_init_work(&framemgr->frames[i].work, sif_read_work_function);
		else
			kthread_init_work(&framemgr->frames[i].work, sif_write_work_function);
		framemgr->frames[i].data = sif_ctx->group;
	}

	sif_ctx->state = BIT(VIO_VIDEO_REBUFS);

	return ret;
}

int sif_video_qbuf(struct sif_video_ctx *sif_ctx, struct frame_info *frameinfo)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	struct vio_group_task	*sif_task;
	int index = 0;

	index = frameinfo->bufferindex;
	framemgr = &sif_ctx->framemgr;
	BUG_ON(index >= framemgr->num_frames);

	framemgr_e_barrier_irqs(framemgr, 0 ,flags);
	frame = &framemgr->frames[index];
	if(frame->state == FS_FREE){
		/*TODO: get the phy address from ion handle*/
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_REQUEST);
	}else{
		vio_err("frame(%d) is invalid state(%d)\n", index, frame->state);
		ret = -EINVAL;
	}

	framemgr_x_barrier_irqr(framemgr, 0 ,flags);
	if(sif_ctx->group->id == 0)
		sif_task = &sif_ctx->sif_dev->sifout_task[sif_ctx->mux_index];
	else
		sif_task = &sif_ctx->sif_dev->sifin_task;

	vio_group_start_trigger(sif_task, frame);

	return ret;

}

int sif_video_dqbuf(struct sif_video_ctx *sif_ctx, struct frame_info *frameinfo)
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
	if(frame){
		memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_FREE);
	}
	framemgr_x_barrier_irqr(framemgr, 0 ,flags);

	return ret;
}

static long x2a_sif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int buffers = 0;
	int enable = 0;
	struct sif_video_ctx *sif_ctx;
	struct frame_info frameinfo;

	sif_ctx = file->private_data;
	BUG_ON(!sif_ctx);

	if (_IOC_TYPE(cmd) != SIF_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
		case SIF_IOC_INIT:
			ret = sif_video_init(sif_ctx, arg);
			break;
		case SIF_IOC_STREAM:
			ret  = get_user(enable,(u32 __user *) arg);
			if (ret)
				return -EFAULT;
			vio_info("V[%d]=====SIF_IOC_STREAM==enable %d===========\n",sif_ctx->id, enable);
			sif_video_s_stream(sif_ctx,!!enable);
			break;
		case SIF_IOC_DQBUF:
			vio_info("V[%d]=====SIF_IOC_DQBUF===============",sif_ctx->id);
			sif_video_dqbuf(sif_ctx, &frameinfo);
			ret = copy_to_user((void __user *)arg, (char *)&frameinfo, sizeof(struct frame_info));
			if(ret)
				return -EFAULT;			
			break;
		case SIF_IOC_QBUF:
			ret =copy_from_user((char *)&frameinfo,(u32 __user *) arg, sizeof(struct frame_info));
			if(ret)
				return -EFAULT;
			vio_info("V[%d]=====SIF_IOC_QBUF==ret %d===========\n",sif_ctx->id, ret);
			sif_video_qbuf(sif_ctx, &frameinfo);
			break;
		case SIF_IOC_REQBUFS:
			ret  = get_user(buffers,(u32 __user *) arg);
			vio_info("V[%d]=====SIF_IOC_REQBUFS==ret %d===========\n",sif_ctx->id, ret);
			if (ret)
				return -EFAULT;
			sif_video_reqbufs(sif_ctx, buffers);
			break;
		default:
			vio_err("wrong ioctl command\n");
			ret = -EFAULT;
			break;
	}

	return ret;
}

static struct file_operations x2a_sif_fops = {
	.owner  = THIS_MODULE,
	.open   = x2a_sif_open,
	.write  = x2a_sif_write,
	.read = x2a_sif_read,
	.release  = x2a_sif_close,
	.unlocked_ioctl = x2a_sif_ioctl,
	.compat_ioctl = x2a_sif_ioctl,
};

void sif_frame_done(struct sif_video_ctx *sif_ctx)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;

	framemgr = &sif_ctx->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0 ,flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if(frame){
		if(sif_ctx->info.update){
			frame->frameinfo.frame_id = sif_ctx->info.frame_id;
			frame->frameinfo.timestamp_l = sif_ctx->info.timestamp_l;
			frame->frameinfo.timestamp_m = sif_ctx->info.timestamp_m;
		}
		trans_frame(framemgr, frame, FS_COMPLETE);
	}
	framemgr_x_barrier_irqr(framemgr, 0 ,flags);

	wake_up(&sif_ctx->done_wq);
}

static irqreturn_t sif_isr(int irq, void *data)
{
	struct sif_irq_src irq_src;
	struct x2a_sif_dev *sif;
	struct sif_video_ctx *sif_ctx;
	struct vio_group_task *gtask;
	u32 mux_index = 0;
	u32 instance;
	int ret = 0;
	u32 status = 0;
	u32 intr_en = 0;

	sif = (struct x2a_sif_dev *)data;
	memset(&irq_src, 0x0, sizeof(struct sif_irq_src));
	ret = sif_get_irq_src(sif->base_reg, &irq_src, true);
	intr_en = sif_get_frame_intr(sif->base_reg);
	status = intr_en & irq_src.sif_frm_int;

	vio_dbg("%s: sif_frm_int = 0x%x,sif_out_int =0x%x, status  = 0x%x\n", __func__, irq_src.sif_frm_int, irq_src.sif_out_int, status);
	if(status){
		for(mux_index = 0; mux_index <= 7; mux_index ++){
			if(test_bit(mux_index, &sif->state) && (mux_index % 2 == 1))
				continue;

			//Frame start processing
			sif_ctx = sif->sif_mux[mux_index].sub_ctx;
			if((status & 1 << mux_index)){
				sif_get_frameid_timestamps(sif->base_reg, mux_index, &sif_ctx->info);
				gtask = &sif->sifout_task[mux_index];
				up(&gtask->hw_resource);
			}

			if(status & 1 << (mux_index + INTR_SIF_MUX0_FRAME_DONE)){
				sif_ctx->info.update = 1;
				sif_frame_done(sif_ctx);
			}
		}
	}

	if(test_bit(SIF_DMA_IN_ENABLE, &sif->state) && (irq_src.sif_out_int & 1 << SIF_ISP_OUT_FE)){
		gtask = &sif->sifin_task;
		instance = atomic_read(&sif->instance);
		sif_ctx = sif->sif_input[instance].sub_ctx;
		sif_frame_done(sif_ctx);

		up(&gtask->hw_resource);
	}

	if(irq_src.sif_err_status)
		vio_err("input size mismatch\n");

	if(irq_src.sif_in_buf_overflow)
		vio_err("input buffer overflow\n");
}

int x2a_sif_device_node_init(struct x2a_sif_dev *sif)
{
	int ret = 0;
	dev_t devno;
	struct device *dev = NULL;

	ret = alloc_chrdev_region(&devno, 0, MAX_DEVICE, "x2a_sif");
	if (ret < 0) {
		vio_err("Error %d while alloc chrdev sif", ret);
		goto err_req_cdev;
	}

	cdev_init(&sif->cdev, &x2a_sif_fops);
	sif->cdev.owner = THIS_MODULE;
	ret = cdev_add(&sif->cdev, devno, MAX_DEVICE);
	if (ret) {
		vio_err("Error %d while adding x2 sif cdev", ret);
		goto err;
	}

	sif->class = class_create(THIS_MODULE, X2A_SIF_NAME);

	dev = device_create(sif->class, NULL, MKDEV(MAJOR(devno), 0), NULL, "sif_w0");
	if (IS_ERR(dev)) {
		ret  = -EINVAL;
		vio_err("sif device create fail\n");
		goto err;
	}

	dev = device_create(sif->class, NULL, MKDEV(MAJOR(devno), 1), NULL, "sif_r0");
	if (IS_ERR(dev)) {
		ret  = -EINVAL;
		vio_err("sif device create fail\n");
		goto err;
	}

	return ret;
err:
	class_destroy(sif->class);
err_req_cdev:
	unregister_chrdev_region(devno, MAX_DEVICE);	
	return ret;
}

static int x2a_sif_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x2a_sif_dev *sif;
	struct resource *mem_res;

	sif = kzalloc(sizeof(struct x2a_sif_dev), GFP_KERNEL);
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
	sif->base_reg =  devm_ioremap_nocache(&pdev->dev, mem_res->start, resource_size(mem_res));
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
	vio_info("sif->irq = %d\n",sif->irq);

	ret = request_threaded_irq(sif->irq, sif_isr, NULL, IRQF_TRIGGER_HIGH, "sif", sif);
	if (ret) {
		vio_err("request_irq(IRQ_SIF %d) is fail(%d)", sif->irq, ret);
		goto err_get_irq;
	}

	sema_init(&sif->sifin_task.hw_resource, 1);

	atomic_set(&sif->rsccount, 0);

	spin_lock_init(&sif->shared_slock);

	x2a_sif_device_node_init(sif);

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

static int x2a_sif_remove(struct platform_device *pdev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x2a_sif_match[] = {
	{
		.compatible = "hobot,x2a-sif",
	},
	{},
};
MODULE_DEVICE_TABLE(of, x2a_sif_match);

static struct platform_driver x2a_sif_driver = {
	.probe		= x2a_sif_probe,
	.remove 	= x2a_sif_remove,
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &x2a_sif_pm_ops,
		.of_match_table = x2a_sif_match,
	}
};

#else
static struct platform_device_id x2a_sif_driver_ids[] = {
	{
		.name		= MODULE_NAME,
		.driver_data	= 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, x2a_sif_driver_ids);

static struct platform_driver x2a_sif_driver = {
	.probe		= x2a_sif_probe,
	.remove		= __devexit_p(x2a_sif_remove),
	.id_table	= x2a_sif_driver_ids,
	.driver	  = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &x2a_sif_pm_ops,
	}
};
#endif

#if CONFIG_QEMU_TEST
static void sif_timer(unsigned long data)
{
	struct sif_irq_src irq_src;
	struct x2a_sif_dev *sif;
	struct sif_video_ctx *sif_ctx;
	struct vio_group_task *gtask;
	u32 mux_index = 0;
	u32 instance;
	int ret = 0;

	mod_timer(&tm[0],jiffies+HZ/25);

	sif = (struct x2a_sif_dev *)data;
	memset(&irq_src, 0x0, sizeof(struct sif_irq_src));
	ret = sif_get_irq_src(sif->base_reg, &irq_src, true);

	irq_src.sif_frm_int = g_test_bit;

	if(irq_src.sif_frm_int)
		for(mux_index = 0; mux_index <= 7; mux_index ++){
			//Frame start processing
			if(sif_get_wdma_enable(sif->base_reg, mux_index)){
				sif_ctx = sif->sif_mux[mux_index].sub_ctx;
				if((irq_src.sif_frm_int & 1 << mux_index)){
					gtask = &sif->sifout_task[mux_index];
					up(&gtask->hw_resource);
					vio_info("current FS interrupt mux = %d\n", mux_index);
				}
				
				mdelay(10);
				if(irq_src.sif_frm_int & 1 << (mux_index + 8)){
					sif_frame_done(sif_ctx);
					vio_info("current FE interrupt mux = %d\n", mux_index);
				}
			}
		}

	if(g_test_input)
		irq_src.sif_out_int = 1 << SIF_ISP_OUT_FE;

	if((irq_src.sif_out_int & 1 << SIF_ISP_OUT_FE)){
		gtask = &sif->sifin_task;
		instance = atomic_read(&sif->instance);
		sif_ctx = sif->sif_input[instance].sub_ctx;
		sif_frame_done(sif_ctx);

		up(&gtask->hw_resource);
	}
}

static int timer_init(struct x2a_sif_dev *sif, int index){
	if(g_test == index)
		return 0;
	g_test = index;
	init_timer(&tm[index]);
	tm[index].expires = jiffies + HZ/10;
	tm[index].function = sif_timer;
	tm[index].data = (unsigned long)sif;
	add_timer(&tm[index]);
}

static int __init x2a_sif_init(void)
{
	int ret = 0;
	struct x2a_sif_dev *sif;

	sif = kzalloc(sizeof(struct x2a_sif_dev), GFP_KERNEL);
	if (!sif) {
		vio_err("sif is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	sif->base_reg = kzalloc(0x1000, GFP_KERNEL);
	x2a_sif_device_node_init(sif);
	atomic_set(&sif->sifin_task.refcount, 0);
	sema_init(&sif->sifin_task.hw_resource, 1);
	spin_lock_init(&sif->shared_slock);

	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

	return 0;

p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;
}

#else

static int __init x2a_sif_init(void)
{
	int ret = platform_driver_register(&x2a_sif_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}
#endif

late_initcall(x2a_sif_init);

static void __exit x2a_sif_exit(void)
{
	platform_driver_unregister(&x2a_sif_driver);
}
module_exit(x2a_sif_exit);

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X2A SIF driver");
MODULE_LICENSE("GPL");
