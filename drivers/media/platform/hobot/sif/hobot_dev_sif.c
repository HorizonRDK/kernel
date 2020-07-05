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
#include <soc/hobot/diag.h>

#include "hobot_dev_sif.h"
#include "sif_hw_api.h"

#define MODULE_NAME "X3 SIF"

char sif_node_name[MAX_DEVICE][8] = {"capture", "ddrin"};
static int mismatch_limit = 1;
module_param(mismatch_limit, int, 0644);
int testpattern_fps = 30;
module_param(testpattern_fps, int, 0644);
static bool debug_log_print = 0;
module_param(debug_log_print, bool, 0644);

int sif_video_streamoff(struct sif_video_ctx *sif_ctx);
extern isp_callback sif_isp_ctx_sync;
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
	mux_index = subdev->ddr_mux_index;

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
		if (sif_mclk_freq)
			vio_set_clk_rate("sif_mclk", sif_mclk_freq);
		ips_set_clk_ctrl(SIF_CLOCK_GATE, true);
		/*4 ddr in channel can not be 0 together*/
		sif_enable_dma(sif->base_reg, 0x10000);
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
	struct vio_framemgr *framemgr;
	unsigned long flags;
	struct list_head *done_list;
	struct x3_sif_dev *sif;

	sif_ctx = file->private_data;
	framemgr = sif_ctx->framemgr;
	sif = sif_ctx->sif_dev;

	poll_wait(file, &sif_ctx->done_wq, wait);
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	done_list = &framemgr->queued_list[FS_COMPLETE];
	if (!list_empty(done_list)) {
		sif->statistic.pollin_comp[sif_ctx->group->instance]\
			[sif_ctx->id]++;
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		return POLLIN;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	if(sif_ctx->event == VIO_FRAME_DONE) {
		sif->statistic.pollin_fe[sif_ctx->group->instance]\
			[sif_ctx->id]++;
		ret = POLLIN;
	} else if (sif_ctx->event == VIO_FRAME_NDONE) {
		sif->statistic.pollerr[sif_ctx->group->instance]\
			[sif_ctx->id]++;
		ret = POLLERR;
	}

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
	if (sif_ctx->state & BIT(VIO_VIDEO_OPEN)) {
		vio_info("[Sx][V%d] %s: only open.\n", sif_ctx->id, __func__);
		atomic_dec(&sif->open_cnt);
		kfree(sif_ctx);
		return 0;
	}

	if ((subdev) && atomic_dec_return(&subdev->refcount) == 0) {
		subdev->state = 0;
		if (sif_ctx->id == 0) {
			clear_bit(subdev->mux_index, &sif->mux_mask);
			clear_bit(subdev->ddr_mux_index, &sif->mux_mask);
			clear_bit(subdev->ddr_mux_index, &sif->state);
			if (subdev->dol_num > 1)
				clear_bit(SIF_DOL2_MODE + subdev->mux_index + 1, &sif->state);
			if (subdev->dol_num > 2)
				clear_bit(SIF_DOL2_MODE + subdev->mux_index + 2, &sif->state);

			if (subdev->mux_nums > 1) {
				clear_bit(subdev->mux_index + 1, &sif->mux_mask);
				clear_bit(subdev->ddr_mux_index + 1, &sif->mux_mask);
			}
			if (subdev->mux_nums > 2) {
				clear_bit(subdev->mux_index + 2, &sif->mux_mask);
				clear_bit(subdev->ddr_mux_index + 2, &sif->mux_mask);
			}

			if (subdev->fmt.width > LINE_BUFFER_SIZE) {
				clear_bit(subdev->mux_index + 2, &sif->mux_mask);
				clear_bit(subdev->ddr_mux_index + 2, &sif->mux_mask);
			}
		}

		group = subdev->group;
		if (group) {
			clear_bit(VIO_GROUP_INIT, &group->state);
			if (group->gtask) {
				vio_group_task_stop(group->gtask);
			}
		}

		frame_manager_close(sif_ctx->framemgr);
	}

	if (atomic_dec_return(&sif->open_cnt) == 0) {
		clear_bit(SIF_DMA_IN_ENABLE, &sif->state);
		clear_bit(SIF_OTF_OUTPUT, &sif->state);
		if (test_bit(SIF_HW_RUN, &sif->state)) {
			set_bit(SIF_HW_FORCE_STOP, &sif->state);
			sif_video_streamoff(sif_ctx);
			clear_bit(SIF_HW_FORCE_STOP, &sif->state);
			atomic_set(&sif->rsccount, 0);
			vio_info("sif force stream off\n");
		}
		//it should disable after ipu stream off because it maybe contain ipu/sif clk
		//vio_clk_disable("sif_mclk");
		ips_set_clk_ctrl(SIF_CLOCK_GATE, false);
	}
	sif_ctx->state = BIT(VIO_VIDEO_CLOSE);

	if (subdev) {
		spin_lock(&subdev->slock);
		clear_bit(sif_ctx->ctx_index, &subdev->val_ctx_mask);
		spin_unlock(&subdev->slock);
	}

	vio_info("SIF close node %d\n", sif_ctx->id);
	kfree(sif_ctx);

	return 0;
}

int sif_get_stride(u32 pixel_length, u32 width)
{
	u32 stride = 0;

	switch (pixel_length) {
	case PIXEL_LENGTH_8BIT:
		stride = ALIGN(width, 16);
		break;
	case PIXEL_LENGTH_10BIT:
		stride = ALIGN(width * 5 / 4, 16);
		break;
	case PIXEL_LENGTH_12BIT:
		stride = ALIGN(width * 3 / 2, 16);
		break;
	case PIXEL_LENGTH_16BIT:
		stride = ALIGN(width * 2, 16);
		break;
	case PIXEL_LENGTH_20BIT:
		stride = ALIGN(width * 5 / 2, 16);
		break;
	default:
		vio_err("wrong pixel length is %d\n", pixel_length);
		break;
	}

	return stride;
}

int get_free_mux(struct x3_sif_dev *sif, u32 index, int format, u32 dol_num,
		u32 width, u32 *mux_numbers)
{
	int ret = 0;
	int i = 0;
	int step = 1;
	int mux_nums = 1;
	int mux_for_4k[] = {0, 1, 4, 5};

	mux_nums = dol_num;
	if (format == HW_FORMAT_YUV422) {
		step = 2;
		mux_nums = 2;
	}

	*mux_numbers = mux_nums;

	mutex_lock(&sif->shared_mutex);
	if (width > LINE_BUFFER_SIZE && step == 1) {
		for (i = 0; i < 4; i++) {
			if (!test_bit(mux_for_4k[i], &sif->mux_mask)) {
				if (test_bit(mux_for_4k[i] + 2, &sif->mux_mask))
					continue;

				set_bit(mux_for_4k[i], &sif->mux_mask);
				set_bit(mux_for_4k[i] + 2, &sif->mux_mask);
				ret = mux_for_4k[i];
				break;
			}
		}
		if (i >= 4) {
			vio_err("can't get free mux for 4k\n");
			ret = -EINVAL;
		}
	} else {
		for (i = index; i < SIF_MUX_MAX; i += step) {
			if (!test_bit(i, &sif->mux_mask)) {
				if (mux_nums > 1 && test_bit(i + 1, &sif->mux_mask))
					continue;
				if (mux_nums > 2 && test_bit(i + 2, &sif->mux_mask))
					continue;

				set_bit(i, &sif->mux_mask);
				if (mux_nums > 1)
					set_bit(i + 1, &sif->mux_mask);
				if (mux_nums > 2)
					set_bit(i + 2, &sif->mux_mask);
				ret = i;
				break;
			}
		}
	}
	mutex_unlock(&sif->shared_mutex);

	if (i >= SIF_MUX_MAX) {
		vio_err("can't get free mux\n");
		ret = -EINVAL;
	}

	return ret;
}
int sif_mux_init(struct sif_subdev *subdev, sif_cfg_t *sif_config)
{
	int ret = 0;
	u32 cfg = 0;
	u32 md_enable = 0;
	int mux_index = 0;
	int ddr_mux_index = 0;
	u32 mux_nums = 0;
	u32 ddr_enable = 0;
	u32 dol_exp_num = 0;
	int format = 0;
	int isp_flyby = 0;
	u32 width = 0;
	struct x3_sif_dev *sif;
	struct vio_group_task *gtask;
	struct vio_group *group;

	sif = subdev->sif_dev;
	group = subdev->group;

	width = sif_config->input.mipi.data.width;
	format = sif_config->input.mipi.data.format;
	dol_exp_num = sif_config->output.isp.dol_exp_num;
	isp_flyby = sif_config->output.isp.func.enable_flyby;
	ddr_enable =  sif_config->output.ddr.enable;
	md_enable = sif_config->output.md.enable;

	if (isp_flyby && !test_bit(SIF_OTF_OUTPUT, &sif->state))
		set_bit(SIF_OTF_OUTPUT, &sif->state);

	/* mux initial*/
	if (!isp_flyby && test_bit(SIF_OTF_OUTPUT, &sif->state)) {
		mux_index = get_free_mux(sif, 4, format, dol_exp_num, width, &mux_nums);
		if (mux_index < 0)
			return mux_index;
		sif_config->output.isp.func.enable_flyby = 1;
	} else {
		mux_index = get_free_mux(sif, 0, format, dol_exp_num, width, &mux_nums);
		if (mux_index < 0)
			return mux_index;
	}

	subdev->mux_nums = mux_nums;
	sif_config->input.mipi.func.set_mux_out_index = mux_index;
	subdev->mux_index = mux_index;
	ddr_mux_index = mux_index;

	if (isp_flyby && ddr_enable) {
		vio_info("ddr output enable in online mode\n");
		ddr_mux_index = get_free_mux(sif, 4, format, dol_exp_num,
				width, &mux_nums);
		if (ddr_mux_index < 0)
			return ddr_mux_index;
		sif->sif_mux[ddr_mux_index] = group;
	}
	sif_config->output.ddr.mux_index = ddr_mux_index;
	subdev->ddr_mux_index = ddr_mux_index;

	if (format == HW_FORMAT_YUV422) {
		if (mux_index % 2 == 0) {
			vio_info("sif input format is yuv, and current mux = %d\n",
			     mux_index);
		} else
			vio_err("sif input format is yuv, but mux is wrong = %d\n",
			     mux_index);
	}

	/* DOL initial*/
	subdev->dol_num = dol_exp_num;
	if(dol_exp_num == 2){
		set_bit(SIF_DOL2_MODE + mux_index + 1, &sif->state);
		if (ddr_mux_index != mux_index)
			set_bit(SIF_DOL2_MODE + ddr_mux_index + 1, &sif->state);
		vio_info("DOL2 mode, current mux = %d\n", mux_index);
	}else if(dol_exp_num == 3){
		set_bit(SIF_DOL2_MODE + mux_index + 1, &sif->state);
		set_bit(SIF_DOL2_MODE + mux_index + 2, &sif->state);
		if (ddr_mux_index != mux_index) {
			set_bit(SIF_DOL2_MODE + ddr_mux_index + 1, &sif->state);
			set_bit(SIF_DOL2_MODE + ddr_mux_index + 2, &sif->state);
		}
		vio_info("DOL3 mode, current mux = %d\n", mux_index);
	}

	/* MD initial*/
	if (md_enable)
		subdev->md_refresh_count = 0;

	subdev->rx_index = sif_config->input.mipi.mipi_rx_index;
	subdev->vc_index = sif_config->input.mipi.vc_index[0];
	subdev->ipi_channels = sif_config->input.mipi.channels;
	if (subdev->rx_index < 2)
		subdev->ipi_index = subdev->rx_index * 4 + subdev->vc_index;
	else
		subdev->ipi_index = 8 + (subdev->rx_index - 2) * 2 + subdev->vc_index;

	subdev->initial_frameid = true;
	sif->sif_mux[mux_index] = group;

	if(ddr_enable == 0) {
		set_bit(VIO_GROUP_OTF_OUTPUT, &group->state);
		cfg = ips_get_bus_ctrl() | 0xd21e << 16;
	} else {
		set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);
		cfg = ips_get_bus_ctrl() | 0x9002 << 16;

		gtask = &sif->sifout_task[mux_index];
		gtask->id = group->id;
		group->gtask = gtask;
		group->frame_work = sif_write_frame_work;
		vio_group_task_start(gtask);
		sema_init(&gtask->hw_resource, 4);
		set_bit(ddr_mux_index, &sif->state);
	}

	// ips_set_bus_ctrl(cfg);
	sif_hw_config(sif->base_reg, sif_config);

	subdev->bufcount = sif_get_current_bufindex(sif->base_reg, ddr_mux_index);
	sif->mismatch_cnt = 0;

	memcpy(&subdev->fmt, &sif_config->input.mipi.data,
		sizeof(sif_data_desc_t));

	vio_info("[S%d] %s mux_index %d ddr_mux_index = %d\n", group->instance,
			__func__, mux_index, ddr_mux_index);

	return ret;
}

int sif_video_init(struct sif_video_ctx *sif_ctx, unsigned long arg)
{
	int ret = 0;
	struct x3_sif_dev *sif;
	struct vio_group *group;
	struct sif_subdev *subdev;
	sif_cfg_t *sif_config;

	if (!(sif_ctx->state & (BIT(VIO_VIDEO_S_INPUT) | BIT(VIO_VIDEO_REBUFS)))) {
		vio_err("[%s][V%02d] invalid INIT is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	sif_ctx->state = BIT(VIO_VIDEO_INIT);

	group = sif_ctx->group;
	subdev = sif_ctx->subdev;
	sif_config = &subdev->sif_cfg;
	if (test_bit(SIF_SUBDEV_INIT, &subdev->state)) {
		vio_info("subdev already init, current refcount(%d)\n",
				atomic_read(&subdev->refcount));
		return ret;
	}

	ret = copy_from_user((char *) sif_config, (u32 __user *) arg,
			   sizeof(sif_cfg_t));
	if (ret)
		return -EFAULT;

	sif = subdev->sif_dev;

	if (sif_ctx->id == 0) {
		ret = sif_mux_init(subdev, sif_config);
	} else if (sif_ctx->id == 1) {
		subdev->dol_num = sif_config->output.isp.dol_exp_num;
		memcpy(&subdev->ddrin_fmt, &sif_config->input.ddr.data,
			sizeof(sif_data_desc_t));

		sif_set_isp_performance(sif->base_reg, sif->hblank);

		set_bit(SIF_DMA_IN_ENABLE, &sif->state);
		set_bit(VIO_GROUP_DMA_INPUT, &group->state);
		set_bit(VIO_GROUP_OTF_OUTPUT, &group->state);
	}

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
	if (atomic_read(&subdev->refcount) >= 1) {
		vio_err("%s more than one pipeline bind\n", __func__);
		return -EFAULT;
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
	sif->statistic.enable[instance] = 1;

	vio_info("[S%d][V%d] %s done, ctx_index(%d) refcount(%d)\n",
		group->instance, sif_ctx->id, __func__,
		i, atomic_read(&subdev->refcount));

	return ret;
}

int sif_video_streamon(struct sif_video_ctx *sif_ctx)
{
	int ret = 0;
	struct x3_sif_dev *sif_dev;
	struct sif_subdev *subdev;

	if (!(sif_ctx->state & (BIT(VIO_VIDEO_STOP)
			| BIT(VIO_VIDEO_REBUFS)
			| BIT(VIO_VIDEO_INIT)))) {
		vio_err("[%s][V%02d] invalid STREAM ON is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}
	sif_ctx->state = BIT(VIO_VIDEO_START);

	subdev = sif_ctx->subdev;
	if (test_bit(SIF_SUBDEV_STREAM_ON, &subdev->state)) {
		vio_info("subdev already stream on, current refcount(%d)\n",
				atomic_read(&subdev->refcount));
		return ret;
	}
	sif_dev = sif_ctx->sif_dev;
	sif_start_pattern_gen(sif_dev->base_reg, 0);

	if (atomic_read(&sif_dev->rsccount) > 0)
		goto p_inc;

	msleep(500);
	mutex_lock(&sif_dev->shared_mutex);
	sif_dev->error_count = 0;
	sif_hw_post_config(sif_dev->base_reg, &subdev->sif_cfg);
	sif_hw_enable(sif_dev->base_reg);
	set_bit(SIF_HW_RUN, &sif_dev->state);
	mutex_unlock(&sif_dev->shared_mutex);

p_inc:
	atomic_inc(&sif_dev->rsccount);
	set_bit(SIF_SUBDEV_STREAM_ON, &subdev->state);

	vio_info("[S%d][V%d]%s\n", sif_ctx->group->instance,
		sif_ctx->id, __func__);

	return ret;
}

int sif_video_streamoff(struct sif_video_ctx *sif_ctx)
{
	int ret = 0;
	int i = 0;
	struct x3_sif_dev *sif_dev;
	struct vio_framemgr *framemgr;
	struct sif_subdev *subdev;

	if (!(sif_ctx->state & BIT(VIO_VIDEO_START))) {
		vio_err("[%s][V%02d] invalid STREAM OFF is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}
	sif_ctx->state = BIT(VIO_VIDEO_STOP);

	subdev = sif_ctx->subdev;
	if (test_bit(SIF_SUBDEV_STREAM_OFF, &subdev->state)) {
		vio_info("subdev already stream off, current refcount(%d)\n",
				atomic_read(&subdev->refcount));
		return ret;
	}
	sif_dev = sif_ctx->sif_dev;
	framemgr = sif_ctx->framemgr;

	if (sif_ctx->id == 0) {
		sif_enable_frame_intr(sif_dev->base_reg, subdev->mux_index, false);
		sif_enable_frame_intr(sif_dev->base_reg, subdev->ddr_mux_index, false);
		for (i = 0; i < subdev->ipi_channels; i++)
			sif_disable_ipi(sif_dev->base_reg, subdev->ipi_index + i);
	}

	if (atomic_dec_return(&sif_dev->rsccount) > 0
		&& !test_bit(SIF_HW_FORCE_STOP, &sif_dev->state))
		goto p_dec;

	msleep(100);

	mutex_lock(&sif_dev->shared_mutex);

	sif_hw_disable(sif_dev->base_reg);
	clear_bit(SIF_HW_RUN, &sif_dev->state);

	mutex_unlock(&sif_dev->shared_mutex);
p_dec:

	if (framemgr->frames != NULL)
		frame_manager_flush(framemgr);

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
	sif_ctx->state = BIT(VIO_VIDEO_REBUFS);

	subdev = sif_ctx->subdev;
	if (test_bit(SIF_SUBDEV_REQBUF, &subdev->state)) {
		vio_info("subdev already reqbufs, current refcount(%d)\n",
				atomic_read(&subdev->refcount));
		return ret;
	}

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
	struct x3_sif_dev *sif;

	index = frameinfo->bufferindex;
	framemgr = sif_ctx->framemgr;
	sif = sif_ctx->sif_dev;
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

	if (sif_ctx->ctx_index == 0)
		sif->statistic.q_normal\
			[sif_ctx->group->instance][sif_ctx->id]++;
	vio_dbg("[S%d][V%d]%s index %d\n", sif_ctx->group->instance,
		sif_ctx->id, __func__, frameinfo->bufferindex);

	return ret;

}

int sif_video_dqbuf(struct sif_video_ctx *sif_ctx,
			struct frame_info *frameinfo)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	struct x3_sif_dev *sif;

	framemgr = sif_ctx->framemgr;
	sif = sif_ctx->sif_dev;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_COMPLETE);
	if (frame) {
		memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_FREE);
		sif_ctx->event = 0;
		if (sif_ctx->ctx_index == 0)
			sif->statistic.dq_normal\
				[sif_ctx->group->instance][sif_ctx->id]++;
	} else {
		ret = -EFAULT;
		sif_ctx->event = 0;
		vio_err("[S%d][V%d] %s (p%d) complete empty.",
			sif_ctx->group->instance, sif_ctx->id, __func__,
			sif_ctx->ctx_index);
		if (sif_ctx->ctx_index == 0)
			sif->statistic.dq_err\
				[sif_ctx->group->instance][sif_ctx->id]++;
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		return ret;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	vio_dbg("[S%d][V%d]%s index %d\n", sif_ctx->group->instance,
		sif_ctx->id, __func__, frameinfo->bufferindex);

	return ret;
}

int sif_enable_bypass(struct sif_video_ctx *sif_ctx, unsigned long arg)
{
	int ret = 0;
	struct x3_sif_dev *sif;
	sif_input_bypass_t cfg;

	ret = copy_from_user((char *) &cfg, (u32 __user *) arg,
			   sizeof(sif_input_bypass_t));
	if (ret) {
		vio_err("%s copy_from_user error(%d)\n", __func__, ret);
		return -EFAULT;
	}

	sif = sif_ctx->sif_dev;
	sif_set_bypass_cfg(sif->base_reg, &cfg);

	return 0;
}

int sif_set_mot_cfg(struct sif_video_ctx *sif_ctx, unsigned long arg)
{
	int ret = 0;
	sif_output_md_t md;
	struct x3_sif_dev *sif;

	if (!(sif_ctx->state & (BIT(VIO_VIDEO_S_INPUT) | BIT(VIO_VIDEO_REBUFS) |
				BIT(VIO_VIDEO_INIT)))) {
		vio_err("[%s][V%02d] invalid MD is requested(%lX)",
				__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}

	ret = copy_from_user((char *) &md, (u32 __user *) arg,
			   sizeof(sif_output_md_t));
	if (ret) {
		vio_err("%s copy_from_user error(%d)\n", __func__, ret);
		return -EFAULT;
	}
	sif = sif_ctx->sif_dev;

	sif_set_md_output(sif->base_reg, &md);

	vio_info("%s: done\n", __func__);
	return ret;
}

int sif_set_pattern_cfg(struct sif_video_ctx *sif_ctx, unsigned long arg)
{
	int ret = 0;
	struct sif_pattern_cfg cfg;
	struct x3_sif_dev *sif;
	struct sif_subdev *subdev;

	ret = copy_from_user((char *) &cfg, (u32 __user *) arg,
			   sizeof(struct sif_pattern_cfg));
	if (ret) {
		vio_err("%s copy_from_user error(%d)\n", __func__, ret);
		return -EFAULT;
	}

	if (cfg.instance > VIO_MAX_STREAM) {
		vio_err("%s : wrong instance %d\n", __func__, cfg.instance);
		return -EFAULT;
	}

	sif = sif_ctx->sif_dev;
	subdev = &sif->sif_mux_subdev[cfg.instance];

	sif_set_pattern_gen(sif->base_reg, subdev->vc_index + 1, &subdev->fmt,
				cfg.framerate);

	vio_info("[%d]%s: frame_rate %d\n", cfg.instance, __func__, cfg.framerate);
	return ret;
}

void sif_video_user_stats(struct sif_video_ctx *sif_ctx,
	struct user_statistic *stats)
{
	struct x3_sif_dev *sif;
	struct vio_group *group;
	struct user_statistic *stats_in_drv;
	u32 instance, dev_id;

	sif = sif_ctx->sif_dev;
	group = sif_ctx->group;
	if (!sif || !group) {
		vio_err("%s init err", __func__);
	}

	if (sif_ctx->ctx_index == 0) {
		instance = group->instance;
		dev_id = sif_ctx->id;
		stats_in_drv = &sif->statistic.user_stats[instance][dev_id];
		stats_in_drv->cnt[USER_STATS_NORM_FRM] =
			stats->cnt[USER_STATS_NORM_FRM];
		stats_in_drv->cnt[USER_STATS_SEL_TMOUT] =
			stats->cnt[USER_STATS_SEL_TMOUT];
		stats_in_drv->cnt[USER_STATS_DROP] =
			stats->cnt[USER_STATS_DROP];
	}
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
	struct user_statistic stats;

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
		ret = sif_video_dqbuf(sif_ctx, &frameinfo);
		if (ret)
			return -EFAULT;
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
		ret = sif_bind_chain_group(sif_ctx, instance);
		if (ret)
			return -EFAULT;
		break;
	case SIF_IOC_END_OF_STREAM:
		ret = get_user(instance, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		vio_bind_group_done(instance);
		break;
	case SIF_IOC_BYPASS:
		ret = sif_enable_bypass(sif_ctx, arg);
		break;
	case SIF_IOC_MD_EVENT:
		ret = ips_get_md_event();
		break;
	case SIF_IOC_MD_CFG:
		ret = sif_set_mot_cfg(sif_ctx, arg);
		break;
	case SIF_IOC_PATTERN_CFG:
		ret = sif_set_pattern_cfg(sif_ctx, arg);
		break;
	case SIF_IOC_USER_STATS:
		ret = copy_from_user((char *) &stats, (u32 __user *) arg,
				   sizeof(struct user_statistic));
		if (ret)
			return -EFAULT;
		sif_video_user_stats(sif_ctx, &stats);
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
	struct x3_sif_dev *sif;
	unsigned long flags;
	u32 event = 0;
	int i = 0;

	BUG_ON(!subdev);

	group = subdev->group;
	sif = subdev->sif_dev;
	framemgr = &subdev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		if(group->get_timestamps){
			frame->frameinfo.frame_id = group->frameid.frame_id;
			frame->frameinfo.timestamps =
			    group->frameid.timestamps;
		}

		vio_set_stat_info(group->instance, SIF_CAP_FE + group->id * 2,
				group->frameid.frame_id);

		do_gettimeofday(&frame->frameinfo.tv);

		trans_frame(framemgr, frame, FS_COMPLETE);
		event = VIO_FRAME_DONE;
		sif->statistic.fe_normal[group->instance][subdev->id]++;
	} else {
		sif->statistic.fe_lack_buf[group->instance][subdev->id]++;
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
	vio_dbg("%s: mux_index = %d\n", __func__, subdev->ddr_mux_index);
}

static void sif_diag_report(uint8_t errsta, unsigned int status)
{
	unsigned int sta;

	sta = status;
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_VIO,
				EventIdVioSifErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&sta,
				sizeof(unsigned int));
	} else {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_VIO,
				EventIdVioSifErr,
				DiagEventStaSuccess);
	}
}

static irqreturn_t sif_isr(int irq, void *data)
{
	int ret = 0;
	u32 mux_index = 0;
	u32 instance = 0;
	u32 status = 0;
	u32 intr_en = 0;
	u32 err_occured = 0;
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
			if (test_bit(mux_index + SIF_DOL2_MODE, &sif->state))
				continue;

			if (status & 1 << (mux_index + INTR_SIF_MUX0_FRAME_DONE)) {
				group = sif->sif_mux[mux_index];
				subdev = group->sub_ctx[0];
				sif_frame_done(subdev);
			}
			//Frame start processing
			if ((status & 1 << mux_index)) {
				group = sif->sif_mux[mux_index];
				subdev = group->sub_ctx[0];
				sif->statistic.fs[group->instance]++;
				sif->statistic.grp_tsk_left[group->instance]
					= atomic_read(&group->rcount);
				if (subdev->initial_frameid) {
					sif_enable_init_frameid(sif->base_reg, subdev->rx_index, 0);
					subdev->initial_frameid = false;
				}
				sif_get_frameid_timestamps(sif->base_reg, mux_index,
						subdev->ipi_index, &group->frameid, subdev->dol_num);

				if (debug_log_print)
					vio_print_stat_info(group->instance);

				vio_set_stat_info(group->instance, SIF_CAP_FS,
						group->frameid.frame_id);

				if (subdev->md_refresh_count == 1) {
					ips_set_md_refresh(0);
				}
				subdev->md_refresh_count++;
				if (test_bit(mux_index, &sif->state)) {
					gtask = group->gtask;
					if (unlikely(list_empty(&gtask->hw_resource.wait_list))) {
						vio_err("[S%d]GP%d(res %d, rcnt %d)\n",
							group->instance,
							gtask->id,
							gtask->hw_resource.count,
							atomic_read(&group->rcount));
						sif->statistic.fs_lack_task[group->instance]++;
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

	if (test_bit(SIF_DMA_IN_ENABLE, &sif->state) &&
		(irq_src.sif_out_int & 1 << SIF_ISP_OUT_FS)) {
			instance = atomic_read(&sif->instance);
			group = sif->sif_input[instance];
			vio_set_stat_info(group->instance, SIF_IN_FS,
					group->frameid.frame_id);
	}

	if (irq_src.sif_frm_int & 1 << INTR_SIF_IN_SIZE_MISMATCH) {
		if (mismatch_limit < 0 || sif->mismatch_cnt < mismatch_limit) {
			vio_err("input size mismatch(0x%x)\n", irq_src.sif_err_status);
			sif_print_rx_status(sif->base_reg, irq_src.sif_err_status);
		}
		sif->mismatch_cnt++;
		err_occured = 1;
		instance = atomic_read(&sif->instance);
		sif->statistic.hard_mismatch[instance]++;
	}

	if (irq_src.sif_frm_int & 1 << INTR_SIF_IN_OVERFLOW) {
		sif->error_count++;
		vio_err("input buffer overflow(0x%x)\n", irq_src.sif_in_buf_overflow);
		sif_print_buffer_status(sif->base_reg);
		err_occured = 1;
		instance = atomic_read(&sif->instance);
		sif->statistic.hard_overflow[instance]++;
	}

	if (irq_src.sif_frm_int & 1 << INTR_SIF_OUT_BUF_ERROR) {
		sif->error_count++;
		vio_err("Out buffer error\n");
		err_occured = 1;
		instance = atomic_read(&sif->instance);
		sif->statistic.hard_buf_err[instance]++;
	}

	if (sif->error_count >= SIF_ERR_COUNT) {
		sif_hw_dump(sif->base_reg);
		sif->error_count = 0;
	}
	sif_diag_report(err_occured, irq_src.sif_frm_int);
	return IRQ_HANDLED;
}

int sif_subdev_init(struct sif_subdev *subdev)
{
	int ret = 0;

	spin_lock_init(&subdev->slock);
	spin_lock_init(&subdev->framemgr.slock);
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

static ssize_t sif_hblank_read(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_sif_dev *sif;

	sif = dev_get_drvdata(dev);

	return snprintf(buf, 64, "%d\n", sif->hblank);
}

static ssize_t sif_hblank_store(struct device *dev,
				       struct device_attribute *devAttr,
				       const char *buf, size_t size)
{
	struct x3_sif_dev *sif;

	sif = dev_get_drvdata(dev);
	sif->hblank = simple_strtoul(buf, NULL, 0);

	vio_info("%s : hblank = %d\n", __func__, sif->hblank);

	return size;
}
static DEVICE_ATTR(hblank, 0660, sif_hblank_read, sif_hblank_store);

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

static ssize_t sif_stat_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_sif_dev *sif;
	u32 offset = 0;
	int instance = 0;
	int i = 0;
	ssize_t len = 0;
	struct user_statistic *stats;

	sif = dev_get_drvdata(dev);

	for(instance = 0; instance < VIO_MAX_STREAM; instance++) {
		if (!sif->statistic.enable[instance])
			continue;
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"*******S%d info:******\n",
			instance);
		offset += len;
		for (i = 0; i < 2; i++) {
			stats = &sif->statistic.user_stats[instance][i];
			len = snprintf(&buf[offset], PAGE_SIZE - offset,
				"ch%d(%s) USER: normal %d, sel tout %d, "
				"drop %d, dq fail %d, sel err%d\n",
				i, sif_node_name[i],
				stats->cnt[USER_STATS_NORM_FRM],
				stats->cnt[USER_STATS_SEL_TMOUT],
				stats->cnt[USER_STATS_DROP],
				stats->cnt[USER_STATS_DQ_FAIL],
				stats->cnt[USER_STATS_SEL_ERR]);
			offset += len;

			len = snprintf(&buf[offset],  PAGE_SIZE - offset,
				"ch%d(%s) DRV: fe_normal %d, fe_lack_buf %d, "
				"pollin_comp %d, pollin_isr %d, pollerr %d, "
				"dq_normal %d, dq_err %d, "
				"q_normal %d\n",
				i, sif_node_name[i],
				sif->statistic.fe_normal[instance][i],
				sif->statistic.fe_lack_buf[instance][i],
				sif->statistic.pollin_comp[instance][i],
				sif->statistic.pollin_fe[instance][i],
				sif->statistic.pollerr[instance][i],
				sif->statistic.dq_normal[instance][i],
				sif->statistic.dq_err[instance][i],
				sif->statistic.q_normal[instance][i]);
			offset += len;
		}
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"DRV: fs %d, grp_tsk_left %d, fs_lack_task %d, "
			"mismatch %d, overflow %d, buf err %d\n",
			sif->statistic.fs[instance],
			sif->statistic.grp_tsk_left[instance],
			sif->statistic.fs_lack_task[instance],
			sif->statistic.hard_mismatch[instance],
			sif->statistic.hard_overflow[instance],
			sif->statistic.hard_buf_err[instance]);
		offset += len;
	}
	return offset;
}

static ssize_t sif_stat_store(struct device *dev,
					struct device_attribute *attr,
					const char *page, size_t len)
{
	struct x3_sif_dev *sif;

	sif = dev_get_drvdata(dev);
	if (sif)
		memset(&sif->statistic, 0, sizeof(sif->statistic));

	return len;
}
static DEVICE_ATTR(err_status, S_IRUGO|S_IWUSR, sif_stat_show, sif_stat_store);

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

	mutex_init(&sif->shared_mutex);

	x3_sif_device_node_init(sif);

	dev = &pdev->dev;
	ret = device_create_file(dev, &dev_attr_regdump);
	if(ret < 0) {
		vio_err("create regdump failed (%d)\n", ret);
		goto p_err;
	}

	ret = device_create_file(dev, &dev_attr_hblank);
	if(ret < 0) {
		vio_err("create hblank failed (%d)\n", ret);
		goto p_err;
	}

	ret = device_create_file(dev, &dev_attr_err_status);
	if (ret < 0) {
		vio_err("create err_status failed (%d)\n", ret);
		goto p_err;
	}

	platform_set_drvdata(pdev, sif);

	ret = x3_sif_subdev_init(sif);

	sif->hblank = 10;

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
	if (diag_register(ModuleDiag_VIO, EventIdVioSifErr,
					4, 400, 8000, NULL) < 0)
		vio_err("sif diag register fail\n");

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

	device_remove_file(&pdev->dev, &dev_attr_regdump);
	device_remove_file(&pdev->dev, &dev_attr_hblank);
	device_remove_file(&pdev->dev, &dev_attr_err_status);

	free_irq(sif->irq, sif);
	for(i = 0; i < MAX_DEVICE; i++)
		device_destroy(sif->class, MKDEV(MAJOR(sif->devno), i));

	if (!vps_class)
		class_destroy(sif->class);
	cdev_del(&sif->cdev);
	unregister_chrdev_region(sif->devno, MAX_DEVICE);
	kfree(sif);

	for(i = 0; i < MAX_DEVICE_VIO_MP; i++)
		device_destroy(g_vio_mp_dev->class,
			MKDEV(MAJOR(g_vio_mp_dev->devno), i));

	if (!vps_class)
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
MODULE_LICENSE("GPL v2");
