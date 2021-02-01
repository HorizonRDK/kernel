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
#include <linux/pm_qos.h>
#include "hobot_dev_sif.h"
#include "sif_hw_api.h"

#define MODULE_NAME "X3 SIF"
extern struct vio_frame_id  sif_frame_info[VIO_MAX_STREAM];

char sif_node_name[MAX_DEVICE][8] = {"capture", "ddrin"};
static int mismatch_limit = 1;
module_param(mismatch_limit, int, 0644);
int testpattern_fps = 30;
module_param(testpattern_fps, int, 0644);
static bool debug_log_print = 0;
module_param(debug_log_print, bool, 0644);
static int g_print_instance = 0;
int sif_video_streamoff(struct sif_video_ctx *sif_ctx);
static struct pm_qos_request sif_pm_qos_req;
static int g_sif_fps[VIO_MAX_STREAM] = {0, };
static int g_sif_idx[VIO_MAX_STREAM] = {0, };
static int g_sif_fps_lasttime[VIO_MAX_STREAM] = {0, };

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

int sif_check_phyaddr(struct frame_info *frameinfo, u8 dol_num)
{
	int ret = 0;
	bool yuv_format = 0;

	yuv_format = (frameinfo->format == HW_FORMAT_YUV422);
	ret = ion_check_in_heap_carveout(frameinfo->addr[0], 0);
	if (ret < 0) {
		vio_err("phyaddr[0] 0x%x is beyond ion address region\n",
				frameinfo->addr[0]);
	}

	if (dol_num > 1 || yuv_format) {
		ret = ion_check_in_heap_carveout(frameinfo->addr[1], 0);
		if (ret < 0) {
			vio_err("phyaddr[1] 0x%x is beyond ion address region\n",
					frameinfo->addr[1]);
		}
	}

	if (dol_num > 2) {
		ret = ion_check_in_heap_carveout(frameinfo->addr[2], 0);
		if (ret < 0) {
			vio_err("phyaddr[2] 0x%x is beyond ion address region\n",
					frameinfo->addr[2]);
		}
	}
	return ret;
}

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

	vio_dbg("[S%d]ddr in width = %d, height = %d, stride = %d\n",
			group->instance, width, height, stride);
}


void sif_read_frame_work(struct vio_group *group)
{
	unsigned long flags;
	u32 instance = 0, ldc_rst_flag;
	int ret = 0;
	struct x3_sif_dev *sif;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct frame_info *frameinfo;
	struct sif_subdev * subdev;

	instance = group->instance;

	if (sif_isp_ctx_sync != NULL) {
		ret = (*sif_isp_ctx_sync)(instance);
	}

	subdev = group->sub_ctx[0];
	if (unlikely(!subdev)) {
		vio_err("%s error subdev null,instance %d", __func__, instance);
		return;
	}

	sif = subdev->sif_dev;
	vio_ldc_access_mutex_lock();
	vio_get_ldc_rst_flag(&ldc_rst_flag);
	atomic_set(&sif->instance, instance);

	framemgr = &subdev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame) {
		if(ldc_rst_flag == 1) {
			vio_dbg("[S%d]%s:ret %d ldc_rst_flag %d", group->instance,
						__func__, ret, ldc_rst_flag);
			trans_frame(framemgr, frame, FS_COMPLETE);
		} else {
			if (ret == 0) {
				frameinfo = &frame->frameinfo;
				group->frameid.frame_id = frameinfo->frame_id;
				group->frameid.timestamps = frameinfo->timestamps;
				group->frameid.tv = frameinfo->tv;
				sif_frame_info[instance].frame_id = frameinfo->frame_id;
				sif_frame_info[instance].timestamps = frameinfo->timestamps;
				sif_frame_info[instance].tv = frameinfo->tv;
				vio_dbg("sif_read_frame_work frame_id %d", frameinfo->frame_id);
				sif_config_rdma_cfg(subdev, 0, frameinfo);
				if (subdev->dol_num > 1) {
					sif_config_rdma_cfg(subdev, 1, frameinfo);
				}

				if (subdev->dol_num > 2) {
					sif_config_rdma_cfg(subdev, 2, frameinfo);
				}
				sif_set_rdma_trigger(sif->base_reg, 1);
			}
			trans_frame(framemgr, frame, FS_PROCESS);
		}
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	vio_dbg("[S%d]%s:done", group->instance, __func__);
	vio_ldc_access_mutex_unlock();
}

void sif_write_frame_work(struct vio_group *group)
{
	u32 instance = 0, i;
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
		if (subdev->fps_ctrl.skip_frame) {
			frame->hw_idx = buf_index;
		} else {
			sif_transfer_ddr_owner(sif->base_reg, mux_index, buf_index);
		}

		if (yuv_format || subdev->dol_num > 1) {
			sif_set_wdma_buf_addr(sif->base_reg, mux_index + 1,
					      buf_index, frame->frameinfo.addr[1]);
			if (subdev->fps_ctrl.skip_frame) {
				frame->hw_idx = buf_index;
			} else {
				sif_transfer_ddr_owner(sif->base_reg, mux_index+1, buf_index);
			}
		}
		if (subdev->splice_info.splice_enable) {
			for(i = 0; i < subdev->splice_info.pipe_num; i++) {
			sif_set_wdma_buf_addr(sif->base_reg, mux_index + 1 + i,
									  buf_index, frame->frameinfo.addr[i + 1]);
			sif_transfer_ddr_owner(sif->base_reg, mux_index + 1 + i,
						   buf_index);
			}
		}
		if (subdev->dol_num > 2) {
			sif_set_wdma_buf_addr(sif->base_reg, mux_index + 2,
					      buf_index, frame->frameinfo.addr[2]);
			if (subdev->fps_ctrl.skip_frame) {
				frame->hw_idx = buf_index;
			} else {
				sif_transfer_ddr_owner(sif->base_reg, mux_index+2, buf_index);
			}
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

	ret = mutex_lock_interruptible(&sif->shared_mutex);
	if (ret) {
		vio_err("open sif mutex lock failed:%d", ret);
		goto p_err;
	}
	if (atomic_inc_return(&sif->open_cnt) == 1) {
		if (sif_mclk_freq)
			vio_set_clk_rate("sif_mclk", sif_mclk_freq);
		ips_set_clk_ctrl(SIF_CLOCK_GATE, true);
		pm_qos_add_request(&sif_pm_qos_req, PM_QOS_DEVFREQ, 10000);
		/*4 ddr in channel can not be 0 together*/
		sif_enable_dma(sif->base_reg, 0x10000);
	}
	mutex_unlock(&sif->shared_mutex);
p_err:
	vio_dbg("%s open_cnt :%d", __func__, atomic_read(&sif->open_cnt));
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
void sif_fps_ctrl_deinit(struct sif_subdev *subdev);
static int x3_sif_close(struct inode *inode, struct file *file)
{
	int i;
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
	if (!(sif_ctx->state & BIT(VIO_VIDEO_STOP))) {
		sif_video_streamoff(sif_ctx);
	}

	mutex_lock(&sif->shared_mutex);
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
			if (subdev->mux_nums > 3) {
				clear_bit(subdev->mux_index + 3, &sif->mux_mask);
				clear_bit(subdev->ddr_mux_index + 3, &sif->mux_mask);
			}
			if (subdev->mux_nums == SIF_MUX_MAX) {  /* 8M  yuv*/
				for (i = 0; i < subdev->mux_nums; i++) {
					clear_bit(subdev->mux_index + i, &sif->mux_mask);
					clear_bit(subdev->ddr_mux_index + i, &sif->mux_mask);
				}
			}

			if (subdev->fmt.width >= LINE_BUFFER_SIZE) {
				clear_bit(subdev->mux_index + 2, &sif->mux_mask);
				clear_bit(subdev->ddr_mux_index + 2, &sif->mux_mask);
				if(subdev->dol_num > 2) { /*8M DOL3*/
					clear_bit(subdev->mux_index + 4, &sif->mux_mask);
					clear_bit(subdev->mux_index + 6, &sif->mux_mask);
					clear_bit(subdev->ddr_mux_index + 4, &sif->mux_mask);
					clear_bit(subdev->ddr_mux_index + 6, &sif->mux_mask);
				}
			}

			if(subdev->splice_info.splice_enable) {
				for (i = 0; i < subdev->splice_info.pipe_num; i++) {
				 clear_bit(SIF_SPLICE_OP + subdev->mux_index + i + 1, &sif->state);
			}
		  }
			sif_fps_ctrl_deinit(subdev);
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
		sif_hw_disable_ex(sif->base_reg);
		//it should disable after ipu stream off because it maybe contain ipu/sif clk
		//vio_clk_disable("sif_mclk");
		ips_set_module_reset(SIF_RST);
		ips_set_clk_ctrl(SIF_CLOCK_GATE, false);
		pm_qos_remove_request(&sif_pm_qos_req);
		vio_info("[S%d][V%d]%s SIF last process close \n",
				sif_ctx->group->instance, sif_ctx->id, __func__);
	}
	mutex_unlock(&sif->shared_mutex);
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
		u32 width, u32 *mux_numbers, u32 splice_enable, u32 pipe_num)
{
	int ret = 0;
	int i = 0;
	int step = 1;
	int mux_nums = 1;
	int mux_for_4k[] = {0, 1, 4, 5};
	int mux_for_2length[] = {0, 4};
	int start_index = 0;

	mux_nums = dol_num;
	if (format == HW_FORMAT_YUV422) {
		step = 2;
		mux_nums = 2;
	}
	if(width >= LINE_BUFFER_SIZE && dol_num == 2) {
		step = 4;
		mux_nums = 4;
	}
	if(width >= LINE_BUFFER_SIZE && dol_num == 3) {
		step = 4;
		mux_nums = 6;
	}
	if(splice_enable) {
		if(pipe_num == 1)
			mux_nums = 2;
		else if (pipe_num == 2)
			mux_nums = 3;
		else if (pipe_num == 3)
			mux_nums = 4;
	}
	if (format == HW_FORMAT_YUV422 && width >= LINE_BUFFER_SIZE) {
		if(width == 3840) {  /*8M yuv*/
			step = 8;
			mux_nums = 8;
		} else {
			step = 4;
			mux_nums = 4;
		}
	}
	*mux_numbers = mux_nums;

	if (width >= LINE_BUFFER_SIZE && step == 1 && format != SIF_FORMAT_YUV_RAW8) {
		if (index == 4)
			start_index = 2;

		for (i = start_index; i < 4; i++) {
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

		vio_info("mux %d sif->mux_mask 0x%lx \n", ret, sif->mux_mask);
	} else if (width >= LINE_BUFFER_SIZE && step == 4) {
		/*4M yuv or 8M DOL2 need 2fifo merge*/
		for (i = start_index; i < 2; i++) {
			if (!test_bit(mux_for_2length[i], &sif->mux_mask)) {
				if (test_bit(mux_for_2length[i] + 1, &sif->mux_mask))
					continue;
				if (test_bit(mux_for_2length[i] + 2, &sif->mux_mask))
					continue;
				if (test_bit(mux_for_2length[i] + 3, &sif->mux_mask))
					continue;
				if (i == 0 && dol_num== 3 &&
					test_bit(mux_for_2length[i] + 4, &sif->mux_mask)
					&& test_bit(mux_for_2length[i] + 6, &sif->mux_mask))
					continue;

				set_bit(mux_for_2length[i], &sif->mux_mask);
				set_bit(mux_for_2length[i] + 1, &sif->mux_mask);
				set_bit(mux_for_2length[i] + 2, &sif->mux_mask);
				set_bit(mux_for_2length[i] + 3, &sif->mux_mask);
				if (i == 0 && dol_num== 3) {
					set_bit(mux_for_2length[i] + 4, &sif->mux_mask);
					set_bit(mux_for_2length[i] + 6, &sif->mux_mask);
				}
				ret = mux_for_2length[i];
				break;
			}
		}
		vio_info("ret %d sif->mux_mask 0x%lx\n", ret, sif->mux_mask);
	} else if (step == 8) {
		/* 8M  yuv need 8 mux, y need fifo0~3, uv need fifo4~7 */
		for (i = start_index; i < SIF_MUX_MAX; i++) {
			if (!test_bit(i, &sif->mux_mask)) {
				if (test_bit(i + 1, &sif->mux_mask))
					continue;
				if (test_bit(i + 2, &sif->mux_mask))
					continue;
				if (test_bit(i + 3, &sif->mux_mask))
					continue;

				set_bit(i, &sif->mux_mask);
				set_bit(i + 1, &sif->mux_mask);
				set_bit(i + 2, &sif->mux_mask);
				set_bit(i + 3, &sif->mux_mask);
			}
		}
		ret = start_index;
		vio_info("ret %d sif->mux_mask 0x%lx \n", ret, sif->mux_mask);
	} else {
		for (i = index; i < SIF_MUX_MAX; i += step) {
			if (!test_bit(i, &sif->mux_mask)) {
				if (mux_nums > 1 && test_bit(i + 1, &sif->mux_mask))
					continue;
				if (mux_nums > 2 && test_bit(i + 2, &sif->mux_mask))
					continue;
				if (mux_nums > 3 && test_bit(i + 3, &sif->mux_mask))
					continue;

				set_bit(i, &sif->mux_mask);
				if (mux_nums > 1)
					set_bit(i + 1, &sif->mux_mask);
				if (mux_nums > 2)
					set_bit(i + 2, &sif->mux_mask);
				if (mux_nums > 3)
					set_bit(i + 3, &sif->mux_mask);
				ret = i;
				break;
			}
		}
	}

	if (i > SIF_MUX_MAX) {
		vio_err("can't get free mux\n");
		ret = -EINVAL;
	}

	return ret;
}
int sif_mux_init(struct sif_subdev *subdev, sif_cfg_t *sif_config)
{
	int ret = 0;
	u32 cfg = 0;
	int mux_index = 0;
	int ddr_mux_index = 0;
	u32 mux_nums = 0;
	u32 ddr_enable = 0;
	u32 dol_exp_num = 0;
	int format = 0;
	int isp_flyby = 0;
	u32 width = 0, i;
	struct x3_sif_dev *sif;
	struct vio_group_task *gtask;
	struct vio_group *group;
	u32 splice_enable, pipe_num;

	sif = subdev->sif_dev;
	group = subdev->group;

	width = sif_config->input.mipi.data.width;
	format = sif_config->input.mipi.data.format;
	dol_exp_num = sif_config->output.isp.dol_exp_num;
	isp_flyby = sif_config->output.isp.func.enable_flyby;
	ddr_enable =  sif_config->output.ddr.enable;
	splice_enable = sif_config->input.splice.splice_enable;
	pipe_num = sif_config->input.splice.pipe_num;
	subdev->splice_info.splice_enable = splice_enable;
	subdev->splice_info.splice_mode = sif_config->input.splice.splice_mode;
	subdev->splice_info.pipe_num = sif_config->input.splice.pipe_num;

	if (isp_flyby && !test_bit(SIF_OTF_OUTPUT, &sif->state))
		set_bit(SIF_OTF_OUTPUT, &sif->state);

	/* mux initial*/
	if (!isp_flyby && test_bit(SIF_OTF_OUTPUT, &sif->state)) {
		mux_index = get_free_mux(sif, 4, format, dol_exp_num, width, &mux_nums,
				splice_enable, pipe_num);
		if (mux_index < 0)
			return mux_index;
		sif_config->output.isp.func.enable_flyby = 1;
	} else {
		mux_index = get_free_mux(sif, 0, format, dol_exp_num, width, &mux_nums,
				splice_enable, pipe_num);
		if (mux_index < 0)
			return mux_index;
	}

	vio_dbg("S[%d] mux_index %d mux_nums %d", group->instance,
			mux_index, mux_nums);
	subdev->mux_nums = mux_nums;
	sif_config->input.mipi.func.set_mux_out_index = mux_index;
	subdev->mux_index = mux_index;
	ddr_mux_index = mux_index;
	subdev->format = format;
	if (format == HW_FORMAT_YUV422)
		subdev->mux_index1 = mux_index + 1;
	if (dol_exp_num > 1)
		subdev->mux_index1 = mux_index + 1;
	if(dol_exp_num > 2) {
		subdev->mux_index2 = mux_index + 2;
	}
	if (isp_flyby && ddr_enable) {
		vio_info("ddr output enable in online mode\n");
		ddr_mux_index = get_free_mux(sif, 4, format, dol_exp_num,
				width, &mux_nums, splice_enable, pipe_num);
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
	// if (md_enable)
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
	if(splice_enable) {
		for (i = 0; i < pipe_num; i++) {
			subdev->splice_info.mux_index[i] = mux_index + 1 + i;
			subdev->splice_info.splice_rx_index[i] =
				sif_config->input.splice.mipi_rx_index[i];
			subdev->splice_info.splice_vc_index[i] =
				sif_config->input.splice.vc_index[i];
		if (subdev->splice_info.splice_rx_index[i] < 2)
			subdev->splice_info.splice_ipi_index[i] =
				subdev->splice_info.splice_rx_index[i] * 4 +
					subdev->splice_info.splice_vc_index[i];
		else
			subdev->splice_info.splice_ipi_index[i] = 8 +
			(subdev->splice_info.splice_rx_index[i] - 2) * 2 +
					subdev->splice_info.splice_vc_index[i];

		sif->sif_mux[mux_index + 1 + i] = group;
		set_bit(SIF_SPLICE_OP + mux_index + 1 + i, &sif->state);

		vio_info("splice_mode %d pipe_num %d \n",
				subdev->splice_info.splice_mode, pipe_num);
		vio_info("[S%d] %s ipi_index %d splice_ipi_index = %d splice_vc_index %d\n",
			group->instance, __func__, subdev->ipi_index,
			subdev->splice_info.splice_ipi_index[i],
			subdev->splice_info.splice_vc_index[i]);
	   }
	}

	if(ddr_enable == 0) {
		set_bit(VIO_GROUP_OTF_OUTPUT, &group->state);
		cfg = ips_get_bus_ctrl() | 0xd21e << 16;
	} else {
		if (isp_flyby)
			set_bit(VIO_GROUP_OTF_OUTPUT, &group->state);
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
/* sif->ddr scenario, Initialize sif skip frame param */
void sif_fps_ctrl_init(struct sif_subdev *subdev, sif_cfg_t *sif_config)
{
	fps_ctrl_t *fps_ctrl = &subdev->fps_ctrl;

	fps_ctrl->skip_frame = sif_config->output.ddr.fps_cfg.hw_skip_frame;
	fps_ctrl->in_fps = sif_config->output.ddr.fps_cfg.in_fps;
	fps_ctrl->out_fps = sif_config->output.ddr.fps_cfg.out_fps;
	fps_ctrl->curr_cnt = 0;

	/* need fps ctrl */
	if (fps_ctrl->skip_frame) {
		/* must lost first frame */
		atomic_set(&fps_ctrl->lost_this_frame, 1);
	} else {
		atomic_set(&fps_ctrl->lost_this_frame, 0);
	}
	vio_dbg("skip_frame=%d,in_fps:%d, out_fps=%d\n",
			fps_ctrl->skip_frame, fps_ctrl->in_fps, fps_ctrl->out_fps);
	return;
}
void sif_fps_ctrl_deinit(struct sif_subdev *subdev)
{
	fps_ctrl_t *fps_ctrl = &subdev->fps_ctrl;
	fps_ctrl->skip_frame = 0;
	fps_ctrl->in_fps = 0;
	fps_ctrl->out_fps = 0;
	fps_ctrl->curr_cnt = 0;
	atomic_set(&fps_ctrl->lost_next_frame, 0);
	atomic_set(&fps_ctrl->lost_this_frame, 0);

	return;
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

	mutex_lock(&sif->shared_mutex);
	if (sif_ctx->id == 0) {
		ret = sif_mux_init(subdev, sif_config);
		sif_fps_ctrl_init(subdev, sif_config);
	} else if (sif_ctx->id == 1) {
		subdev->dol_num = sif_config->output.isp.dol_exp_num;
		memcpy(&subdev->ddrin_fmt, &sif_config->input.ddr.data,
			sizeof(sif_data_desc_t));

		sif_set_isp_performance(sif->base_reg, sif->hblank);

		set_bit(SIF_DMA_IN_ENABLE, &sif->state);
		set_bit(VIO_GROUP_DMA_INPUT, &group->state);
		set_bit(VIO_GROUP_OTF_OUTPUT, &group->state);
	}
	mutex_unlock(&sif->shared_mutex);
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
	unsigned long flags;

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
		vio_err("%s instance %d more than one pipeline bind\n",
			__func__, instance);
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

	spin_lock_irqsave(&subdev->slock, flags);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if(!test_bit(i, &subdev->val_ctx_mask)) {
			subdev->ctx[i] = sif_ctx;
			sif_ctx->ctx_index = i;
			set_bit(i, &subdev->val_ctx_mask);
			break;
		}
	}
	spin_unlock_irqrestore(&subdev->slock, flags);
	if (i == VIO_MAX_SUB_PROCESS) {
		vio_err("alreay open too much for one pipeline\n");
		return -EFAULT;
	}
	atomic_inc(&subdev->refcount);

	if(sif_ctx->id == 1){
		sif->sif_input[instance] = group;
		group->frame_work = sif_read_frame_work;
		group->gtask = &sif->sifin_task;
		group->gtask->id = group->id;
		ret = mutex_lock_interruptible(&sif->shared_mutex);
		if (ret) {
			vio_err("bind sif mutex lock failed:%d", ret);
			goto p_err;
		}
		vio_group_task_start(group->gtask);
		mutex_unlock(&sif->shared_mutex);
	}

	sif_ctx->state = BIT(VIO_VIDEO_S_INPUT);
	sif->statistic.enable[instance] = 1;

	vio_info("[S%d][V%d] %s done, ctx_index(%d) refcount(%d)\n",
		group->instance, sif_ctx->id, __func__,
		i, atomic_read(&subdev->refcount));
p_err:
	return ret;
}

int sif_video_streamon(struct sif_video_ctx *sif_ctx)
{
	int ret = 0;
	struct x3_sif_dev *sif_dev;
	struct sif_subdev *subdev;
	u32 isp_enable, ipi_mode;
	sif_input_mipi_t *p_mipi = NULL;

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

	p_mipi = &subdev->sif_cfg.input.mipi;
	ipi_mode = p_mipi->ipi_mode;

	isp_enable = subdev->sif_cfg.output.isp.enable;
	mutex_lock(&sif_dev->shared_mutex);
	if (sif_ctx->id == 0) {
		vio_info("[S%d][V%d]%s mux_index %d \n",
			sif_ctx->group->instance, sif_ctx->id, __func__,
			subdev->mux_index);
		sif_enable_frame_intr(sif_dev->base_reg, subdev->mux_index, true);
		if(subdev->ddr_mux_index != subdev->mux_index) {
			sif_enable_frame_intr(sif_dev->base_reg, subdev->ddr_mux_index, true);
		}
		if(subdev->dol_num > 1 || ipi_mode > 1) {
			sif_enable_frame_intr(sif_dev->base_reg, subdev->mux_index + 1, true);
		}
		if(subdev->dol_num > 2 || ipi_mode > 2) {
			sif_enable_frame_intr(sif_dev->base_reg, subdev->mux_index + 2, true);
		}
	}
	if (isp_enable && atomic_read(&sif_dev->isp_init_cnt) == 0) {
		sif_raw_isp_output_config(sif_dev->base_reg, &subdev->sif_cfg.output);
	}
	if (atomic_read(&sif_dev->rsccount) > 0) {
		goto p_inc;
	}
	sif_dev->error_count = 0;
	sif_hw_post_config(sif_dev->base_reg, &subdev->sif_cfg);
	sif_hw_enable(sif_dev->base_reg);
	set_bit(SIF_HW_RUN, &sif_dev->state);

p_inc:
	if(isp_enable) {
		atomic_inc(&sif_dev->isp_init_cnt);
	}
	atomic_inc(&sif_dev->rsccount);
	mutex_unlock(&sif_dev->shared_mutex);
	set_bit(SIF_SUBDEV_STREAM_ON, &subdev->state);

	vio_clear_stat_info(sif_ctx->group->instance);
	vio_info("[S%d][V%d]%s isp_init_cnt %d rsccount %d \n",
		sif_ctx->group->instance, sif_ctx->id, __func__,
		atomic_read(&sif_dev->isp_init_cnt),
		atomic_read(&sif_dev->rsccount));
	return ret;
}

int sif_video_streamoff(struct sif_video_ctx *sif_ctx)
{
	int ret = 0;
	int i = 0;
	struct x3_sif_dev *sif_dev;
	struct vio_framemgr *framemgr;
	struct sif_subdev *subdev;
	u32 isp_enable;
	u32 splice_enable, pipe_num;

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

	mutex_lock(&sif_dev->shared_mutex);
	if (sif_ctx->id == 0) {
		sif_enable_frame_intr(sif_dev->base_reg, subdev->mux_index, false);
		sif_enable_frame_intr(sif_dev->base_reg, subdev->ddr_mux_index, false);
		for (i = 0; i < subdev->ipi_channels; i++)
			sif_disable_ipi(sif_dev->base_reg, subdev->ipi_index + i);
		splice_enable = subdev->splice_info.splice_enable;
		pipe_num = subdev->splice_info.pipe_num;
		if(splice_enable) {
			for (i = 0; i < pipe_num; i++) {
				sif_enable_frame_intr(sif_dev->base_reg, subdev->mux_index + 1 + i, false);
				sif_disable_ipi(sif_dev->base_reg, subdev->splice_info.splice_ipi_index[i]);
			}
		}
	}
	isp_enable = subdev->sif_cfg.output.isp.enable;
	if (isp_enable && (atomic_dec_return(&sif_dev->isp_init_cnt) == 0)) {
		sif_disable_isp_out_config(sif_dev->base_reg);
		atomic_set(&sif_dev->isp_init_cnt, 0);
	}
	if (atomic_dec_return(&sif_dev->rsccount) > 0) {
		mutex_unlock(&sif_dev->shared_mutex);
		goto p_dec;
	}
	msleep(100);
	sif_hw_disable(sif_dev->base_reg);
	clear_bit(SIF_HW_RUN, &sif_dev->state);
	atomic_set(&sif_dev->rsccount, 0);
	vio_info("[S%d][V%d]%s SIF last process stream off \n",
		sif_ctx->group->instance,
		sif_ctx->id, __func__);
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
	int i = 0, instance;
	struct vio_framemgr *framemgr;
	struct vio_group *group;
	struct x3_sif_dev *sif;
	struct sif_subdev *subdev;

	if (!(sif_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS) |
		      BIT(VIO_VIDEO_INIT) | BIT(VIO_VIDEO_S_INPUT)))) {
		vio_err("[%s][V%02d] invalid REQBUFS is requested(%lX)",
			__func__, sif_ctx->id, sif_ctx->state);
		return -EINVAL;
	}
	sif_ctx->state = BIT(VIO_VIDEO_REBUFS);
	sif = sif_ctx->sif_dev;
	subdev = sif_ctx->subdev;
	instance = sif_ctx->group->instance;
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
		if (sif_ctx->id == 0) {
			framemgr->frames[i].mp_work = &sif->sifout_work[instance][i].work;
			frame_work_init(&sif->sifout_work[instance][i].work);
			sif->sifout_work[instance][i].group = group;
		} else {
			frame_work_init(&sif->sifin_work[instance][i].work);
			framemgr->frames[i].mp_work = &sif->sifin_work[instance][i].work;
			sif->sifin_work[instance][i].group = group;
		}
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
	struct sif_subdev *subdev;

	index = frameinfo->bufferindex;
	framemgr = sif_ctx->framemgr;
	sif = sif_ctx->sif_dev;
	BUG_ON(index >= framemgr->num_frames);

	subdev = sif_ctx->subdev;

	if (subdev->dol_num) {
		ret = sif_check_phyaddr(frameinfo, subdev->dol_num);
		if (ret)
			return -EINVAL;
	}

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
	vio_group_start_trigger_mp(group, frame);

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
	struct sif_subdev *subdev;

	framemgr = sif_ctx->framemgr;
	sif = sif_ctx->sif_dev;
	subdev = sif_ctx->subdev;

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
int sif_set_mot_start(struct sif_video_ctx *sif_ctx)
{
	int ret = 0;
	struct x3_sif_dev *sif;

	sif = sif_ctx->sif_dev;

	ips_set_md_enable();
	sif_set_md_enable(sif->base_reg);
	vio_info("%s: done\n", __func__);
	return ret;
}

int sif_set_mot_stop(struct sif_video_ctx *sif_ctx)
{
	int ret = 0;
	struct x3_sif_dev *sif;

	sif = sif_ctx->sif_dev;

	ips_set_md_disable();
	sif_set_md_disable(sif->base_reg);
	vio_info("%s: done\n", __func__);
	return ret;
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
	struct x3_sif_dev *sif_dev;
	struct vio_bind_info_dev *bind_info_dev;
	struct frame_info frameinfo;
	struct vio_group *group;
	struct user_statistic stats;
	struct hb_bind_info_s (*bind_info)[HB_ID_MAX];

	sif_ctx = file->private_data;
	BUG_ON(!sif_ctx);
	group = sif_ctx->group;
	sif_dev = sif_ctx->sif_dev;
	BUG_ON(!sif_dev);
	bind_info_dev = sif_dev->vio_bind_info_dev;
	BUG_ON(!bind_info_dev);

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
			return ret;
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

		bind_info = (bind_info_dev->bind_info);
		memset(bind_info[instance], 0,
			sizeof(struct hb_bind_info_s) * HB_ID_MAX);

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
	case SIF_IOC_MD_ENABLE:
		ret = sif_set_mot_start(sif_ctx);
		break;
	case SIF_IOC_MD_DISENABLE:
		ret = sif_set_mot_stop(sif_ctx);
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

int x3_sif_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;
	phys_addr_t offset = (phys_addr_t)vma->vm_pgoff << PAGE_SHIFT;

	/* It's illegal to wrap around the end of the physical address space. */
	if (offset + (phys_addr_t)size - 1 < offset)
		return -EINVAL;

	// if (!valid_mmap_phys_addr_range(vma->vm_pgoff, size))
	// 	return -EINVAL;

	// if (!private_mapping_ok(vma))
	// 	return -ENOSYS;

	// if (!range_is_allowed(vma->vm_pgoff, size))
	// 	return -EPERM;

	/* Remap-pfn-range will mark the range VM_IO */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    vma->vm_pgoff,
			    size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
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
	.mmap = x3_sif_mmap,
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
	struct sif_video_ctx *sif_ctx = NULL;
	struct x3_sif_dev *sif;
	unsigned long flags;
	u32 event = 0;
	int i = 0;

	BUG_ON(!subdev);

	group = subdev->group;
	sif = subdev->sif_dev;

	framemgr = &subdev->framemgr;
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if (test_bit(i, &subdev->val_ctx_mask)) {
			sif_ctx = subdev->ctx[i];
		}
	}
	if(subdev->splice_info.splice_enable && sif_ctx->id == 0) {
		if (subdev->splice_info.splice_done && subdev->splice_info.frame_done) {
			framemgr_e_barrier_irqs(framemgr, 0, flags);
			frame = peek_frame(framemgr, FS_PROCESS);
			if (frame) {
				if (group->id == 0) {
					struct timeval tmp_tv;
					do_gettimeofday(&tmp_tv);
					g_sif_idx[group->instance]++;
					if (tmp_tv.tv_sec > g_sif_fps_lasttime[group->instance]) {
						g_sif_fps[group->instance] = g_sif_idx[group->instance];
						g_sif_fps_lasttime[group->instance] = tmp_tv.tv_sec;
						g_sif_idx[group->instance] = 0;
					}
				}
				frame->frameinfo.frame_id = group->frameid.frame_id;
				frame->frameinfo.timestamps = group->frameid.timestamps;
				frame->frameinfo.tv = group->frameid.tv;
				vio_set_stat_info(group->instance, SIF_CAP_FE + group->id * 2,
						group->frameid.frame_id);

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
			spin_lock_irqsave(&subdev->slock, flags);
			for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
				if (test_bit(i, &subdev->val_ctx_mask)) {
					sif_ctx = subdev->ctx[i];
					sif_ctx->event = event;
					wake_up(&sif_ctx->done_wq);
				}
			}
			spin_unlock_irqrestore(&subdev->slock, flags);
			subdev->splice_info.splice_done = 0;
			subdev->splice_info.frame_done = 0;
		}
	} else {
			framemgr_e_barrier_irqs(framemgr, 0, flags);
			frame = peek_frame(framemgr, FS_PROCESS);
			if (frame) {
				if (group->id == 0) {
					struct timeval tmp_tv;
					do_gettimeofday(&tmp_tv);
					g_sif_idx[group->instance]++;
					if (tmp_tv.tv_sec > g_sif_fps_lasttime[group->instance]) {
						g_sif_fps[group->instance] = g_sif_idx[group->instance];
						g_sif_fps_lasttime[group->instance] = tmp_tv.tv_sec;
						g_sif_idx[group->instance] = 0;
					}
				}
				frame->frameinfo.frame_id = group->frameid.frame_id;
				frame->frameinfo.timestamps = group->frameid.timestamps;
				frame->frameinfo.tv = group->frameid.tv;
				vio_set_stat_info(group->instance, SIF_CAP_FE + group->id * 2,
						group->frameid.frame_id);

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
			spin_lock_irqsave(&subdev->slock, flags);
			for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
				if (test_bit(i, &subdev->val_ctx_mask)) {
					sif_ctx = subdev->ctx[i];
					sif_ctx->event = event;
					wake_up(&sif_ctx->done_wq);
				}
			}
			spin_unlock_irqrestore(&subdev->slock, flags);
	}
	vio_dbg("%s: mux_index = %d\n", __func__, subdev->ddr_mux_index);
}

void sif_frame_ndone(struct sif_subdev *subdev)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	struct sif_video_ctx *sif_ctx;
	struct x3_sif_dev *sif;
	unsigned long flags;
	int i = 0;

	BUG_ON(!subdev);

	group = subdev->group;
	sif = subdev->sif_dev;
	framemgr = &subdev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
        vio_dbg("ndone bidx%d fid%d, proc->req.",
               frame->frameinfo.bufferindex,
               frame->frameinfo.frame_id);
		trans_frame(framemgr, frame, FS_REQUEST);
		vio_group_start_trigger_mp(group, frame);
	} else {
		vio_err("[S%d][V%d] NDONE [FRM](%d %d %d %d %d)\n",
			group->instance,
			subdev->id,
			framemgr->queued_count[FS_FREE],
			framemgr->queued_count[FS_REQUEST],
			framemgr->queued_count[FS_PROCESS],
			framemgr->queued_count[FS_COMPLETE],
			framemgr->queued_count[FS_USED]);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	spin_lock_irqsave(&subdev->slock, flags);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if (test_bit(i, &subdev->val_ctx_mask)) {
			sif_ctx = subdev->ctx[i];
			sif_ctx->event = VIO_FRAME_DONE;
			wake_up(&sif_ctx->done_wq);
		}
	}
	spin_unlock_irqrestore(&subdev->slock, flags);
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

struct sif_subdev *sif_find_overflow_subdev(struct x3_sif_dev *sif, u32 mux)
{
	u32 i = 0;

	for(i = 0; i < VIO_MAX_STREAM; i++) {
		if (sif->sif_mux_subdev[i].mux_index == mux) {
			vio_dbg("%s mux %d\n", __func__, mux);
			return &sif->sif_mux_subdev[i];
		}
		if (sif->sif_mux_subdev[i].format == HW_FORMAT_YUV422) {
			if(sif->sif_mux_subdev[i].mux_index1 == mux)  {
				vio_dbg("%s yuv mux %d\n", __func__, mux);
				if (sif->sif_mux_subdev[i].overflow == 1)
					return NULL;
				else
					return &sif->sif_mux_subdev[i];
			}
		} else if (sif->sif_mux_subdev[i].dol_num > 1) {
			if(sif->sif_mux_subdev[i].mux_index1 == mux)  {
				vio_dbg("%s dol_num %d mux %d\n", __func__,
					sif->sif_mux_subdev[i].dol_num, mux);
				if (sif->sif_mux_subdev[i].overflow == 1)
					return NULL;
				else
					return &sif->sif_mux_subdev[i];
			}
		} else if (sif->sif_mux_subdev[i].dol_num > 2) {
			if (sif->sif_mux_subdev[i].mux_index2 == mux) {
				vio_dbg("%s dol_num %d mux %d\n", __func__,
					sif->sif_mux_subdev[i].dol_num, mux);
				if (sif->sif_mux_subdev[i].overflow == 1)
					return NULL;
				else
					return &sif->sif_mux_subdev[i];
			}
		}
	}

	return NULL;
}
static void subdev_balance_lost_next_frame(struct sif_subdev *subdev)
{
	int lost_flag;
	fps_ctrl_t *fps_ctrl = &subdev->fps_ctrl;

	if ((fps_ctrl->in_fps) && (fps_ctrl->out_fps)) {
		lost_flag = atomic_read(&fps_ctrl->lost_next_frame);
		atomic_set(&fps_ctrl->lost_this_frame, lost_flag);
		fps_ctrl->curr_cnt += fps_ctrl->out_fps;
		if (fps_ctrl->curr_cnt < fps_ctrl->in_fps) {
			atomic_set(&fps_ctrl->lost_next_frame, 1);
		} else {
			fps_ctrl->curr_cnt -= fps_ctrl->in_fps;
			atomic_set(&fps_ctrl->lost_next_frame, 0);
		}
		vio_dbg("[S%d][V%d]%s lost_this = 0x%x, lost_next = 0x%x,",
				subdev->group->instance, subdev->id, __func__,
				atomic_read(&fps_ctrl->lost_this_frame),
				atomic_read(&fps_ctrl->lost_next_frame));
	}

	return;
}
static void subdev_set_frm_owner(struct sif_subdev *subdev,
									struct x3_sif_dev *sif, u32 mux_index)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	u32 yuv_format = 0;

	framemgr = &subdev->framemgr;
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		sif_transfer_ddr_owner(sif->base_reg, mux_index, frame->hw_idx);
		yuv_format = (frame->frameinfo.format == HW_FORMAT_YUV422);
		if (yuv_format || subdev->dol_num > 1) {
			sif_transfer_ddr_owner(sif->base_reg, mux_index+1, frame->hw_idx);
		}
		if (subdev->dol_num > 2) {
			sif_transfer_ddr_owner(sif->base_reg, mux_index+2, frame->hw_idx);
		}
	}
	return;
}

static irqreturn_t sif_isr(int irq, void *data)
{
	int ret = 0, i;
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
	u32 mux = 0;
	u8 temp;

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
				if(subdev->splice_info.splice_enable) {
					if (test_bit(mux_index + SIF_SPLICE_OP, &sif->state)) {
						subdev->splice_info.splice_done = 1;
					} else {
						subdev->splice_info.frame_done = 1;
					}
				}
				sif_frame_done(subdev);
				/* fps_ctrl:FDone:if not lost next frame,transfer owner to HW */
				if ((subdev->fps_ctrl.skip_frame)&&
						(!atomic_read(&subdev->fps_ctrl.lost_this_frame))) {
					subdev_set_frm_owner(subdev, sif, mux_index);
				}
			}
			//Frame start processing
			if ((status & 1 << mux_index)) {
				group = sif->sif_mux[mux_index];
				if(!group || !group->sub_ctx[0]) {
					return IRQ_HANDLED;
				}
				subdev = group->sub_ctx[0];
				sif->statistic.fs[group->instance]++;
				sif->statistic.grp_tsk_left[group->instance]
					= atomic_read(&group->rcount);

				if (subdev->initial_frameid) {
					if (test_bit(mux_index + SIF_SPLICE_OP, &sif->state)) {
						for(i = 0; i < subdev->splice_info.pipe_num; i++) {
							sif_enable_init_frameid(sif->base_reg,
								subdev->splice_info.splice_rx_index[i], 0);
						}
					} else {
						sif_enable_init_frameid(sif->base_reg, subdev->rx_index, 0);
					}
					subdev->initial_frameid = false;
				}
				if (test_bit(mux_index + SIF_SPLICE_OP, &sif->state)) {
					for(i = 0; i < subdev->splice_info.pipe_num; i++) {
						 sif_get_frameid_timestamps(sif->base_reg,
							subdev->splice_info.mux_index[i],
						 subdev->splice_info.splice_ipi_index[i],
						 &group->frameid, subdev->dol_num,
						 group->instance, &subdev->sif_cfg.output,
						 &subdev->sif_cfg.input);
					}
				} else {
					sif_get_frameid_timestamps(sif->base_reg, mux_index, subdev->ipi_index,
					&group->frameid, subdev->dol_num, group->instance, &subdev->sif_cfg.output,
					&subdev->sif_cfg.input);
				}
				if (test_bit(mux_index + SIF_SPLICE_OP, &sif->state)) {
					continue;
				}
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
					if (unlikely(list_empty(&gtask->hw_resource.wait_list)) &&
						 gtask->hw_resource.count >= 4) {
						vio_err("[S%d]GP%d(res %d, rcnt %d)\n",
							group->instance,
							gtask->id,
							gtask->hw_resource.count,
							atomic_read(&group->rcount));
						sif->statistic.fs_lack_task[group->instance]++;
					} else {
						if ((subdev->fps_ctrl.skip_frame)
							&&(atomic_read(&subdev->fps_ctrl.lost_this_frame))) {
							vio_dbg("fps_ctrl:FS:lost_this_frame\n");
						} else {
							up(&gtask->hw_resource);
						}
					}
				}
				/* fps_ctrl:FS:if not lost next frame,transfer owner to HW */
				if (subdev->fps_ctrl.skip_frame) {
					if (!atomic_read(&subdev->fps_ctrl.lost_next_frame)) {
						subdev_set_frm_owner(subdev, sif, mux_index);
					}
					subdev_balance_lost_next_frame(subdev);
				}
			}
		}
	}
	if (test_bit(SIF_DMA_IN_ENABLE, &sif->state)
	    && (irq_src.sif_out_int & 1 << SIF_ISP_OUT_FE)) {
		instance = atomic_read(&sif->instance);
		if(!sif->sif_input[instance]) {
			return IRQ_HANDLED;
		}
		group = sif->sif_input[instance];
		gtask = group->gtask;

		vio_group_done(group);

		subdev = group->sub_ctx[0];
		sif_frame_done(subdev);
	}
	if (test_bit(SIF_DMA_IN_ENABLE, &sif->state) &&
		(irq_src.sif_out_int & 1 << SIF_ISP_OUT_FS)) {
			instance = atomic_read(&sif->instance);
			if(!sif->sif_input[instance]) {
				return IRQ_HANDLED;
			}
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

		temp = irq_src.sif_in_buf_overflow;
		while(temp > 0) {
			subdev = NULL;
			if (temp & 0x01) {
				subdev = sif_find_overflow_subdev(sif, mux);
			}
			mux += 1;
			temp >>= 1;
			if (subdev) {
				subdev->overflow = 1;
				sif_frame_ndone(subdev);
			}
		}
		for(i = 0; i < VIO_MAX_STREAM; i++) {
			sif->sif_mux_subdev[i].overflow = 0;
		}
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

	if (atomic_read(&sif->open_cnt) != 0) {
		sif_hw_dump(sif->base_reg);
	} else {
		vio_info("SIF has not been initialized");
	}
	return 0;
}

static DEVICE_ATTR(regdump, 0444, sif_reg_dump, NULL);

#define SIF_STA_SHOW(n, fmt, ...) \
						s += sprintf(s, "%-15s: " fmt "\n", #n, \
		## __VA_ARGS__)

static ssize_t sif_cfg_info(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_sif_dev *sif;
	char *s = buf;
	sif_cfg_t *sif_cfg = NULL;
	int i;
	struct sif_subdev *subdev = NULL;

	sif = dev_get_drvdata(dev);
	for(i = 0; i < VIO_MAX_STREAM; i++) {
		if (test_bit(SIF_SUBDEV_INIT, &sif->sif_mux_subdev[i].state)) {
			subdev = &sif->sif_mux_subdev[i];
		    sif_cfg = &sif->sif_mux_subdev[i].sif_cfg;
		} else if (test_bit(SIF_SUBDEV_INIT, &sif->sif_in_subdev[i].state)) {
			subdev = &sif->sif_in_subdev[i];
		    sif_cfg = &sif->sif_in_subdev[i].sif_cfg;
		} else {
			s += sprintf(s, "pipe %d not inited\n", i);
			subdev = NULL;
			sif_cfg = NULL;
		}

		if (subdev && sif_cfg) {
		   SIF_STA_SHOW(pipeid, "%d", i);
		   SIF_STA_SHOW(mipi_rx_index, "%d   %s",
		   subdev->rx_index, "mipi_index");
		   SIF_STA_SHOW(vc_index, "%d   %s", subdev->vc_index, "vc of mipi");
		   SIF_STA_SHOW(input_width, "%d", sif_cfg->input.mipi.data.width);
		   SIF_STA_SHOW(input_height, "%d", sif_cfg->input.mipi.data.height);
		   SIF_STA_SHOW(format, "%d   %s",
			sif_cfg->input.mipi.data.format, "raw:0 yuv:8");
		   SIF_STA_SHOW(ipi_channels, "%d", sif_cfg->input.mipi.channels);
		   SIF_STA_SHOW(isp_enable, "%d   %s",
			sif_cfg->output.isp.enable, "1: isp enable,  0: isp disable");
		   SIF_STA_SHOW(ddrin_enable, "%d   %s",
			sif_cfg->input.ddr.enable, "0: online->isp  1: offline->isp");
		   SIF_STA_SHOW(out_ddr_enable, "%d   %s",
			sif_cfg->output.ddr.enable, "1: sif->offline");
		   SIF_STA_SHOW(isp_flyby, "%d   %s",
			sif_cfg->output.isp.func.enable_flyby, "1: sif->online->isp");
		   SIF_STA_SHOW(buffer_num, "%d   %s",
			sif_cfg->output.ddr.buffer_num, "capture buff nums");
		   SIF_STA_SHOW(ipu_flyby, "%d   %s",
			sif_cfg->output.ipu.enable_flyby, "1: sif->online->ipu");
		   s += sprintf(s, "\n\n");
	   }
	}
	return (s - buf);
}
static DEVICE_ATTR(cfg_info, 0444, sif_cfg_info, NULL);

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

/*
 * vio bind info dev node
 */
static int vio_bind_info_dev_open(struct inode *inode, struct file *file)
{
	struct vio_bind_info_dev *bind_dev;

	bind_dev = container_of(inode->i_cdev, struct vio_bind_info_dev, cdev);
	file->private_data = bind_dev;

	vio_info("vio bind info dev open.\n");

	return 0;
}

static int vio_bind_info_dev_close(struct inode *inode, struct file *file)
{
	vio_info("vio bind info dev close.\n");

	return 0;
}

static long vio_bind_info_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int ret = 0;
	struct vio_bind_info_dev *bind_dev;
	struct hb_bind_info_update_s bind_info_update;
	SYS_MOD_S *src_mod, *dst_mod;

	bind_dev = file->private_data;
	BUG_ON(!bind_dev);
	if (_IOC_TYPE(cmd) != VIO_BIND_INFO_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case VIO_BIND_INFO_UPDATE:
		ret = copy_from_user((char *) &bind_info_update,
				(u32 __user *) arg,
				sizeof(struct hb_bind_info_update_s));
		if (ret)
			return -EFAULT;
		src_mod = &bind_info_update.src_mod;
		dst_mod = &bind_info_update.dst_mod;
		vio_dbg("src mod id%d devid%d chnid%d\n", src_mod->enModId,
				src_mod->s32DevId, src_mod->s32ChnId);
		vio_dbg("dst mod id%d devid%d chnid%d\n", dst_mod->enModId,
				dst_mod->s32DevId, dst_mod->s32ChnId);

		bind_dev->bind_info[src_mod->s32DevId][src_mod->enModId].\
			out[src_mod->s32ChnId].next_mod = dst_mod->enModId;
		bind_dev->bind_info[src_mod->s32DevId][src_mod->enModId].\
			out[src_mod->s32ChnId].next_dev_id = dst_mod->s32DevId;
		bind_dev->bind_info[src_mod->s32DevId][src_mod->enModId].\
			out[src_mod->s32ChnId].next_chn_id = dst_mod->s32ChnId;

		bind_dev->bind_info[dst_mod->s32DevId][dst_mod->enModId].\
			in[dst_mod->s32ChnId].prev_mod = src_mod->enModId;
		bind_dev->bind_info[dst_mod->s32DevId][dst_mod->enModId].\
			in[dst_mod->s32ChnId].prev_dev_id = src_mod->s32DevId;
		bind_dev->bind_info[dst_mod->s32DevId][dst_mod->enModId].\
			in[dst_mod->s32ChnId].prev_chn_id = src_mod->s32ChnId;

		break;
	default:
		vio_err("wrong ioctl command\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

static struct file_operations vio_bind_info_fops = {
	.owner = THIS_MODULE,
	.open = vio_bind_info_dev_open,
	.release = vio_bind_info_dev_close,
	.unlocked_ioctl = vio_bind_info_ioctl,
	.compat_ioctl = vio_bind_info_ioctl,
};

int vio_bind_info_dev_node_init(struct vio_bind_info_dev *bind_dev)
{
	int ret = 0;
	struct device *dev = NULL;

	ret = alloc_chrdev_region(&bind_dev->devno, 0, 1, "vio_bind_info_dev");
	if (ret < 0) {
		vio_err("Error %d while alloc chrdev vio_bind_info", ret);
		goto err_req_cdev;
	}

	cdev_init(&bind_dev->cdev, &vio_bind_info_fops);
	bind_dev->cdev.owner = THIS_MODULE;
	ret = cdev_add(&bind_dev->cdev, bind_dev->devno, 1);
	if (ret) {
		vio_err("Error %d while adding vio bind info dev", ret);
		goto err;
	}

	if (vps_class)
		bind_dev->class = vps_class;
	else
		bind_dev->class = class_create(THIS_MODULE,
				X3_VIO_BIND_INFO_DEV_NAME);

	dev = device_create(bind_dev->class, NULL,
			MKDEV(MAJOR(bind_dev->devno), 0),
			NULL, "vio_bind_info");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		vio_err("vio bind info dev create fail\n");
		goto err;
	}

	return ret;
err:
	class_destroy(bind_dev->class);
err_req_cdev:
	unregister_chrdev_region(bind_dev->devno, 1);
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
	if (sif) {
		memset(&sif->statistic, 0, sizeof(sif->statistic));
	}
	return len;
}
static DEVICE_ATTR(err_status, S_IRUGO|S_IWUSR, sif_stat_show, sif_stat_store);

static ssize_t vio_delay_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	u32 offset = 0;
	offset = vio_print_delay(g_print_instance, buf, PAGE_SIZE);
	return offset;
}

static ssize_t vio_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	g_print_instance = simple_strtoul(buf, NULL, 0);
	if (g_print_instance >= VIO_MAX_STREAM)
		g_print_instance = VIO_MAX_STREAM - 1;
	else if (g_print_instance < 0)
		g_print_instance = 0;
	return len;
}

static DEVICE_ATTR(vio_delay, S_IRUGO|S_IWUSR, vio_delay_show, vio_delay_store);

static ssize_t sif_fps_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	u32 offset = 0;
	int i, len;

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		if (g_sif_fps[i] == 0) continue;
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
				"sif pipe %d: output fps %d\n", i, g_sif_fps[i]);
		offset += len;
		g_sif_fps[i] = 0;
	}
	return offset;
}

static DEVICE_ATTR(fps, S_IRUGO, sif_fps_show, NULL);

char VIO_MOD_NAME[][5] = {
	"SYS",
	"VIN",
	"VOT",
	"VPS",
	"RGN",
	"AIN",
	"AOT",
	"VENC",
	"VDEC",
	"AENC",
	"ADEC"
};

static ssize_t vio_bind_info_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_sif_dev *sif;
	struct vio_bind_info_dev *bind_dev;
	/*struct hb_bind_info_s **bind_info;*/
	struct hb_bind_info_s (*bind_info)[HB_ID_MAX];
	struct hb_bind_info_s *head_mod;
	struct hb_bind_info_s *mod;
	struct hb_bind_info_s *next_mod;
	uint8_t pipe_id = 0;
	uint8_t mod_id = 0;
	uint8_t i, j, k, out_chn_num, link_flag;
	int next_chn, prev_chn;
	ssize_t len = 0;
	uint16_t offset = 0, x_offset = 0;

	sif = dev_get_drvdata(dev);
	BUG_ON(!sif);
	bind_dev = sif->vio_bind_info_dev;
	BUG_ON(!bind_dev);
	bind_info = (bind_dev->bind_info);

	if (bind_info == NULL) {
		vio_err("null bind_info error\n");
		return 0;
	}
	for (pipe_id = 0; pipe_id < MAX_VIO_DEV; pipe_id++) {
		for (mod_id = 0; mod_id < HB_ID_MAX; mod_id++) {
			mod = &bind_info[pipe_id][mod_id];
			if (mod == NULL) {
				vio_err("NULL mod error\n");
				return 0;
			}
			mod->this_mod = mod_id;
			mod->this_dev_id = pipe_id;
			mod->had_show = 0;
		}
	}
	for (pipe_id = 0; pipe_id < MAX_VIO_DEV; pipe_id++) {
		link_flag = 0;
		x_offset = 0;
		for (mod_id = 0; mod_id < HB_ID_MAX; mod_id++) {
			mod = &bind_info[pipe_id][mod_id];
			head_mod = mod;
			/*
			 *find a linked mod
			 */
			for (i = 0; i < MAX_INPUT_CHANNEL; i++) {
				if (mod->in[i].prev_mod == 0)
					continue;
				mod = &bind_info[mod->in[i].prev_dev_id]\
					[mod->in[i].prev_mod];
				link_flag = 1;
				break;
			}
			if (mod == head_mod)
				continue;
			vio_dbg("find a linked module %s\n",
					VIO_MOD_NAME[mod->this_mod]);
			/*
			 *use the linked mod find the head mod
			 */
			for (j = 0; j < HB_ID_MAX; j++) {
				for (i = 0; i < MAX_INPUT_CHANNEL; i++) {
					if (mod->in[i].prev_mod == 0)
						continue;
					mod = &bind_info[mod->in[i].prev_dev_id]\
						[mod->in[i].prev_mod];
				}
			}
			head_mod = mod;
			vio_dbg("find the head module %s\n",
					VIO_MOD_NAME[mod->this_mod]);
			break;
		}
		if (link_flag == 0)
			continue;
		if (head_mod->had_show != 0)
			continue;
		/*
		 *print the bind info
		 */
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"\n[%s%d]", VIO_MOD_NAME[head_mod->this_mod],
			head_mod->this_dev_id);
		offset += len;
		head_mod->had_show = 1;

		while (1) {
			next_chn = -1;
			out_chn_num = 0;
			for (i = 0; i < MAX_OUTPUT_CHANNEL; i++) {
				if (mod->out[i].next_mod == 0)
					continue;
				next_chn = i;
				out_chn_num++;
				if (out_chn_num > 1) {
					len = snprintf(&buf[offset],
						PAGE_SIZE - offset,
						"\n");
					offset += len;
					for (k = 0; k < x_offset / 2 + 5; k++)
						offset += snprintf(&buf[offset],
							PAGE_SIZE - offset, " ");
					offset += snprintf(&buf[offset],
						PAGE_SIZE - offset, "|");
				}
				len = snprintf(&buf[offset],
					PAGE_SIZE - offset,
					"-#%d <==",
					next_chn);
				offset += len;
				if (out_chn_num <= 1)
					x_offset += len;
				next_mod = &bind_info[mod->out[next_chn].\
					next_dev_id][mod->out[next_chn].next_mod];
				for (j = 0; j < MAX_INPUT_CHANNEL; j++) {
					if (next_mod->in[j].prev_mod != 0) {
						prev_chn = j;
						len = snprintf(&buf[offset],
							PAGE_SIZE - offset,
							"==> #%d",
							prev_chn);
						offset += len;
						if (out_chn_num <= 1)
							x_offset += len;
					}
				}
				len = snprintf(&buf[offset], PAGE_SIZE - offset,
					"-[%s%d]", VIO_MOD_NAME[next_mod->this_mod],
					next_mod->this_dev_id);
				offset += len;
				if (out_chn_num <= 1)
					x_offset += len;
			}
			if (next_chn == -1)
				break;
			mod = &bind_info[mod->out[next_chn].next_dev_id]\
				[mod->out[next_chn].next_mod];
		}
	}
	offset += snprintf(&buf[offset], PAGE_SIZE - offset, "\n");

	return offset;
}

static ssize_t vio_bind_info_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	int pipe_num;
	struct x3_sif_dev *sif;
	struct vio_bind_info_dev *bind_dev;
	struct hb_bind_info_s (*bind_info)[HB_ID_MAX];

	sif = dev_get_drvdata(dev);
	bind_dev = sif->vio_bind_info_dev;
	bind_info = (bind_dev->bind_info);

	pipe_num = simple_strtoul(buf, NULL, 0);
	if (pipe_num < MAX_VIO_DEV)
		memset(bind_info[pipe_num], 0,
			sizeof(struct hb_bind_info_s) * HB_ID_MAX);
	if (pipe_num == MAX_VIO_DEV)
		memset(&bind_info[0][0], 0,
			sizeof(struct hb_bind_info_s) *\
			HB_ID_MAX * MAX_VIO_DEV);

	return len;
}
static DEVICE_ATTR(bind_info, S_IRUGO|S_IWUSR,
		vio_bind_info_show, vio_bind_info_store);

struct x3_vio_mp_dev *g_vio_mp_dev = NULL;
struct vio_bind_info_dev *g_vio_bind_info_dev = NULL;
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
	atomic_set(&sif->isp_init_cnt, 0);
	atomic_set(&sif->open_cnt, 0);
	vio_init_ldc_access_mutex();
	vio_rst_mutex_init();
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
	ret = device_create_file(dev, &dev_attr_cfg_info);
	if (ret < 0) {
		vio_err("create dev_attr_cfg_info failed (%d)\n", ret);
		goto p_err;
	}
	ret = device_create_file(dev, &dev_attr_vio_delay);
	if (ret < 0) {
		vio_err("create dev_attr_vio_delay failed (%d)\n", ret);
		goto p_err;
	}
	ret = device_create_file(dev, &dev_attr_bind_info);
	if (ret < 0) {
		vio_err("create dev_attr_bind_info failed (%d)\n", ret);
		goto p_err;
	}
	ret = device_create_file(dev, &dev_attr_fps);
	if (ret < 0) {
		vio_err("create fps failed (%d)\n", ret);
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

	g_vio_bind_info_dev = kzalloc(sizeof(struct vio_bind_info_dev),
			GFP_KERNEL);
	if (g_vio_bind_info_dev == NULL) {
		vio_err("bind info dev is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}
	vio_bind_info_dev_node_init(g_vio_bind_info_dev);
	sif->vio_bind_info_dev = g_vio_bind_info_dev;

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
	device_remove_file(&pdev->dev, &dev_attr_cfg_info);
	device_remove_file(&pdev->dev, &dev_attr_vio_delay);
	device_remove_file(&pdev->dev, &dev_attr_bind_info);
	device_remove_file(&pdev->dev, &dev_attr_fps);

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

	device_destroy(g_vio_bind_info_dev->class,
			MKDEV(MAJOR(g_vio_bind_info_dev->devno), 0));
	if (!vps_class)
		class_destroy(g_vio_bind_info_dev->class);
	cdev_del(&g_vio_bind_info_dev->cdev);
	unregister_chrdev_region(g_vio_bind_info_dev->devno, 1);
	kfree(g_vio_bind_info_dev);

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
