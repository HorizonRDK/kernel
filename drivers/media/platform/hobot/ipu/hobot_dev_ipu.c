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
#include <soc/hobot/diag.h>

#include "hobot_dev_ipu.h"
#include "ipu_hw_api.h"

#define MODULE_NAME "X3 IPU"

static u32 color[MAX_OSD_COLOR_NUM] = {
	0xff8080, 0x008080, 0x968080, 0x8DC01B, 0x952B15, 0xE10094, 0x545BA7,
	0x9E26C4, 0x34AAB5, 0xD47A9E, 0x4C54FF, 0xD06057, 0x0FC574, 0x3A5E56,
	0x2968C5
};

char ipu_node_name[MAX_DEVICE][8] =
	{"src", "us", "ds0", "ds1", "ds2", "ds3", "ds4", "none"};

void ipu_hw_set_cfg(struct ipu_subdev *subdev);
void ipu_update_hw_param(struct ipu_subdev *subdev);
int ipu_video_streamoff(struct ipu_video_ctx *ipu_ctx);
enum buffer_owner ipu_index_owner(struct ipu_video_ctx *ipu_ctx, u32 index);

static int x3_ipu_open(struct inode *inode, struct file *file)
{
	struct ipu_video_ctx *ipu_ctx;
	struct x3_ipu_dev *ipu;
	int ret = 0;
	int minor;

	minor = MINOR(inode->i_rdev);

	ret = 0;
	ipu = container_of(inode->i_cdev, struct x3_ipu_dev, cdev);
	ipu_ctx = kzalloc(sizeof(struct ipu_video_ctx), GFP_KERNEL);
	if (ipu_ctx == NULL) {
		vio_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	init_waitqueue_head(&ipu_ctx->done_wq);

	ipu_ctx->id = minor;
	ipu_ctx->ipu_dev = ipu;
	file->private_data = ipu_ctx;
	ipu_ctx->state = BIT(VIO_VIDEO_OPEN);

	if (atomic_read(&ipu->open_cnt) == 0) {
		atomic_set(&ipu->backup_fcount, 0);
		atomic_set(&ipu->sensor_fcount, 0);
		if (sif_mclk_freq)
			vio_set_clk_rate("sif_mclk", sif_mclk_freq);
		ips_set_clk_ctrl(IPU0_CLOCK_GATE, true);
	}

	atomic_inc(&ipu->open_cnt);
p_err:
	return ret;
}

static int x3_ipu_close(struct inode *inode, struct file *file)
{
	struct ipu_video_ctx *ipu_ctx;
	struct x3_ipu_dev *ipu;
	struct vio_group *group;
	struct ipu_subdev *subdev;
	int instance = 0;
	u32 index;
	u32 cnt;
	int ret = 0;
	u32 ctx_index;
	u32 id;

	ipu_ctx = file->private_data;
	ipu = ipu_ctx->ipu_dev;

	if (ipu_ctx->state & BIT(VIO_VIDEO_OPEN)) {
		vio_info("[Sx][V%d] %s: only open.\n", ipu_ctx->id, __func__);
		atomic_dec(&ipu->open_cnt);
		kfree(ipu_ctx);
		return 0;
	}

	// if pipeline's all subdevs are closed, pipeline is disabled
	ipu->statistic.enable_subdev[ipu_ctx->belong_pipe] &= ~(BIT(ipu_ctx->id));
	if (ipu->statistic.enable_subdev[ipu_ctx->belong_pipe] == 0) {
		ipu->statistic.enable[ipu_ctx->belong_pipe] = 0;
		vio_dbg("pipeline%d all subdev closed", ipu_ctx->belong_pipe);
	}

	index = ipu_ctx->frm_fst_ind;
	cnt = ipu_ctx->frm_num;
	frame_manager_flush_mp(ipu_ctx->framemgr, index, cnt, ipu_ctx->ctx_index);

	group = ipu_ctx->group;
	subdev = ipu_ctx->subdev;
	if ((group) &&(atomic_dec_return(&subdev->refcount) == 0)) {
		subdev->state = 0;
		if (group->gtask && subdev->leader)
			vio_group_task_stop(group->gtask);
		subdev->leader = false;
		subdev->skip_flag = false;
		group->output_flag = 0;
		instance = group->instance;
		ipu->reuse_shadow0_count &= ~instance;
		if (atomic_dec_return(&group->node_refcount) == 0) {
			clear_bit(VIO_GROUP_LEADER, &group->state);
			clear_bit(VIO_GROUP_INIT, &group->state);
		}
	}
	frame_manager_close_mp(ipu_ctx->framemgr, index, cnt, ipu_ctx->ctx_index);
	if (atomic_dec_return(&ipu->open_cnt) == 0) {
		clear_bit(IPU_OTF_INPUT, &ipu->state);
		clear_bit(IPU_DMA_INPUT, &ipu->state);
		clear_bit(IPU_DS2_DMA_OUTPUT, &ipu->state);
		clear_bit(IPU_HW_CONFIG, &ipu->state);
		clear_bit(IPU_REUSE_SHADOW0, &ipu->state);

		if (test_bit(IPU_HW_RUN, &ipu->state)) {
			set_bit(IPU_HW_FORCE_STOP, &ipu->state);
			ipu_video_streamoff(ipu_ctx);
			clear_bit(IPU_HW_FORCE_STOP, &ipu->state);
			atomic_set(&ipu->rsccount, 0);
			vio_info("ipu force stream off\n");
		}
		ips_set_clk_ctrl(IPU0_CLOCK_GATE, false);
		ipu->frame_drop_count = 0;
		ipu->reuse_shadow0_count = 0;
		sema_init(&ipu->gtask.hw_resource, 1);
		atomic_set(&ipu->gtask.refcount, 0);
	}

	ipu_ctx->state = BIT(VIO_VIDEO_CLOSE);

	clear_bit(ipu_ctx->ctx_index, &subdev->val_ctx_mask);
	ctx_index = ipu_ctx->ctx_index;
	id = ipu_ctx->id;
	subdev->ctx[ctx_index] = NULL;
	kfree(ipu_ctx);

	vio_info("[S%d]IPU close node V%d proc %d\n", group->instance,
		id, ctx_index);

	return ret;
}

static ssize_t x3_ipu_write(struct file *file, const char __user * buf,
				size_t count, loff_t * ppos)
{
	return 0;
}

static ssize_t x3_ipu_read(struct file *file, char __user * buf, size_t size,
				loff_t * ppos)
{
	return 0;
}

static u32 x3_ipu_poll(struct file *file, struct poll_table_struct *wait)
{
	int ret = 0;
	struct ipu_video_ctx *ipu_ctx;
	struct vio_framemgr *framemgr;
	unsigned long flags;
	struct list_head *done_list;
	struct x3_ipu_dev *ipu;

	ipu_ctx = file->private_data;
	framemgr = ipu_ctx->framemgr;
	ipu = ipu_ctx->ipu_dev;

	poll_wait(file, &ipu_ctx->done_wq, wait);
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	done_list = &framemgr->queued_list[FS_COMPLETE];
	if (!list_empty(done_list)) {
		ipu->statistic.pollin_comp[ipu_ctx->group->instance]\
			[ipu_ctx->id]++;
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		return POLLIN;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	if(ipu_ctx->event == VIO_FRAME_DONE) {
		ipu->statistic.pollin_fe[ipu_ctx->group->instance]\
			[ipu_ctx->id]++;
		ret = POLLIN;
	} else if (ipu_ctx->event == VIO_FRAME_NDONE) {
		ipu->statistic.pollerr[ipu_ctx->group->instance]\
			[ipu_ctx->id]++;
		ret =  POLLERR;
	}

	return ret;
}
int ipu_check_phyaddr(struct vio_frame *frame)
{
	int ret = 0;

	ret = ion_check_in_heap_carveout(frame->frameinfo.addr[0], 0);
	if (ret < 0) {
		vio_err("phyaddr[0] 0x%x is beyond ion address region\n",
				frame->frameinfo.addr[0]);
	}

	ret = ion_check_in_heap_carveout(frame->frameinfo.addr[1], 0);
	if (ret < 0) {
		vio_err("phyaddr[1] 0x%x is beyond ion address region\n",
				frame->frameinfo.addr[1]);
	}

	return ret;
}

void ipu_frame_work(struct vio_group *group)
{
	struct ipu_subdev *subdev;
	struct x3_ipu_dev *ipu;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	int i = 0;
	u32 instance = 0;
	u32 rdy = 0;
	u8 shadow_index = 0;

	instance = group->instance;
	subdev = group->sub_ctx[0];
	if (!subdev) {
		vio_err("%s group%d sub mp 0 err.\n", __func__, instance);
		return;
	}
	ipu = subdev->ipu_dev;
	if (instance < MAX_SHADOW_NUM)
		shadow_index = instance;
	vio_dbg("[S%d]%s start\n", instance, __func__);

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy & ~(1 << 4);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	ipu_set_shd_select(ipu->base_reg, shadow_index);

	atomic_set(&ipu->instance, instance);
	for (i = MAX_DEVICE - 1; i >= 0; i--) {
		subdev = group->sub_ctx[i];
		if (!subdev)
			continue;
		framemgr = &subdev->framemgr;
		framemgr_e_barrier_irqs(framemgr, 0, flags);
		frame = peek_frame(framemgr, FS_REQUEST);
		if (frame) {
			if (i != GROUP_ID_SRC)
				ipu_check_phyaddr(frame);

			switch (i) {
			case GROUP_ID_SRC:
				ipu_set_rdma_addr(ipu->base_reg,
					frame->frameinfo.addr[0],
					frame->frameinfo.addr[1]);
				if (test_bit(IPU_REUSE_SHADOW0, &ipu->state))
					ipu_update_hw_param(subdev);
				break;
			case GROUP_ID_US:
				ipu_set_us_wdma_addr(ipu->base_reg,
							 frame->frameinfo.addr[0],
							 frame->frameinfo.addr[1]);
				break;
			case GROUP_ID_DS0:
			case GROUP_ID_DS1:
			case GROUP_ID_DS2:
			case GROUP_ID_DS3:
			case GROUP_ID_DS4:
				ipu_set_ds_wdma_addr(ipu->base_reg, i - 2,
							 frame->frameinfo.addr[0],
							 frame->frameinfo.addr[1]);
				break;
			default:
				break;
			}

			ipu_hw_set_cfg(subdev);
			trans_frame(framemgr, frame, FS_PROCESS);
		}
		framemgr_x_barrier_irqr(framemgr, 0, flags);
	}
	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy | (1 << 4);
	ipu_set_shd_rdy(ipu->base_reg, rdy);
	atomic_inc(&ipu->backup_fcount);

	if (test_bit(IPU_DMA_INPUT, &ipu->state))
		ipu_set_rdma_start(ipu->base_reg);

	if (!test_bit(IPU_HW_CONFIG, &ipu->state))
		set_bit(IPU_HW_CONFIG, &ipu->state);
	vio_dbg("[S%d]%s done; rdy = %d\n", instance, __func__, rdy);
}


void ipu_set_group_leader(struct vio_group *group, enum group_id id)
{
	struct ipu_subdev *subdev;

	subdev = group->sub_ctx[id];
	if (id >= GROUP_ID_MAX || id < GROUP_ID_SRC || !subdev) {
		vio_err("%s wrong id %d or subdev null", __func__, id);
		return;
	}

	if (!test_bit(VIO_GROUP_LEADER, &group->state)) {
		set_bit(VIO_GROUP_LEADER, &group->state);
		subdev->leader = true;
		vio_info("[S%d][V%d] %s\n", group->instance, id, __func__);
	}
}

void ipu_clear_group_leader(struct vio_group *group)
{
	struct ipu_subdev *subdev;
	int i;

	for (i = 0; i < GROUP_ID_MAX; i++) {
		subdev = group->sub_ctx[i];
		if (!subdev)
			continue;
		subdev->leader = false;
	}

	clear_bit(VIO_GROUP_LEADER, &group->state);
}

int ipu_update_scale_info(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	ipu_ds_info_t *sc_cfg;
	struct ipu_subdev *subdev;

	subdev = ipu_ctx->subdev;
	sc_cfg = &subdev->info_cfg.sc_info;
	ret = copy_from_user((char *) sc_cfg, (u32 __user *) arg,
			   sizeof(ipu_ds_info_t));
	if (ret)
		return -EFAULT;

	subdev->info_cfg.info_update = 1;
	vio_dbg("[S%d][V%d] %s\n", ipu_ctx->group->instance, ipu_ctx->id,
		 __func__);

	return ret;
}

int ipu_update_osd_color_map(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	u32 id = 0;
	osd_color_map_t *color_map;
	struct ipu_subdev *subdev;

	subdev = ipu_ctx->subdev;
	id = ipu_ctx->id;
	if (id < GROUP_ID_US || id > GROUP_ID_DS1) {
		vio_err("%s wrong ctx id %d\n", __func__, id);
		return -EFAULT;
	}
	color_map = &subdev->osd_cfg.color_map;
	ret = copy_from_user((char *) color_map, (u32 __user *) arg,
			   sizeof(osd_color_map_t));
	if (ret)
		return -EFAULT;

	vio_dbg("[S%d][V%d] %s\n", ipu_ctx->group->instance, ipu_ctx->id,
		 __func__);

	return ret;
}

int ipu_update_osd_addr(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	u32 id = 0;
	u32 *osd_buf;
	struct ipu_subdev *subdev;

	subdev = ipu_ctx->subdev;
	id = ipu_ctx->id;
	if (id < GROUP_ID_US || id > GROUP_ID_DS1) {
		vio_err("%s wrong ctx id %d\n", __func__, id);
		return -EFAULT;
	}
	osd_buf = subdev->osd_cfg.osd_buf;
	ret = copy_from_user((char *) osd_buf, (u32 __user *) arg,
			   MAX_OSD_LAYER * sizeof(u32));
	if (ret)
		return -EFAULT;

	subdev->osd_cfg.osd_buf_update = 1;
	vio_dbg("[S%d][V%d] %s\n", subdev->group->instance, ipu_ctx->id,
		 __func__);

	return ret;
}

int ipu_update_osd_roi(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	u32 id = 0;
	osd_box_t *osd_box;
	struct ipu_subdev *subdev;

	subdev = ipu_ctx->subdev;
	id = ipu_ctx->id;
	if (id < GROUP_ID_US || id > GROUP_ID_DS1) {
		vio_err("%s wrong ctx id %d\n", __func__, id);
		return -EFAULT;
	}
	osd_box = subdev->osd_cfg.osd_box;
	ret = copy_from_user((char *) osd_box, (u32 __user *) arg,
			  MAX_OSD_NUM * sizeof(osd_box_t));
	if (ret)
		return -EFAULT;

	subdev->osd_cfg.osd_box_update = 1;

	vio_dbg("[S%d][V%d] %s\n", subdev->group->instance, ipu_ctx->id,
		 __func__);

	return ret;
}

void ipu_hw_set_osd_cfg(struct ipu_subdev *subdev, u32 shadow_index)
{
	u32 osd_index = 0;
	u32 id = 0;
	u32 i = 0;
	u8 osd_enable = 0;
	u8 osd_overlay = 0;
	u32 sta_enable = 0;
	u16 start_x, start_y, width, height;
	u32 __iomem *base_reg;
	osd_box_t *osd_box;
	osd_sta_box_t *sta_box;
	u32 *osd_buf;
	osd_color_map_t *color_map;
	struct ipu_osd_cfg *osd_cfg;

	id = subdev->id;
	if (id < GROUP_ID_US || id > GROUP_ID_DS1) {
		return;
	}

	if(id == GROUP_ID_US)
		osd_index = 2;
	else
		osd_index = id - GROUP_ID_DS0;

	base_reg = subdev->ipu_dev->base_reg;

	osd_cfg = &subdev->osd_cfg;
	osd_box = osd_cfg->osd_box;
	osd_buf = osd_cfg->osd_buf;

	if (osd_cfg->osd_box_update) {
		for (i = 0; i < MAX_OSD_LAYER; i++) {
			osd_enable |= osd_box[i].osd_en << i;
			osd_overlay |= osd_box[i].osd_en ? osd_box[i].overlay_mode << i : 0;
			start_x = osd_box[i].start_x;
			start_y = osd_box[i].start_y;
			width = osd_box[i].width;
			height = osd_box[i].height;
			ipu_set_osd_roi(base_reg, shadow_index, osd_index, i, start_x,
					start_y, width, height);
		}
		ipu_set_osd_overlay_mode(base_reg, shadow_index, osd_index,
				osd_overlay);
		ipu_set_osd_enable(base_reg, shadow_index, osd_index, osd_enable);
		vio_dbg("OSD[%d]osd enable = 0x%x, overlay = 0x%x", osd_index,
				osd_enable, osd_overlay);
	}
	osd_cfg->osd_box_update = 0;

	if(osd_cfg->osd_buf_update) {
		for (i = 0; i < MAX_OSD_LAYER; i++) {
			ipu_set_osd_addr(base_reg, shadow_index, osd_index,
					i, osd_buf[i]);
		}
	}
	osd_cfg->osd_buf_update = 0;

	color_map = &subdev->osd_cfg.color_map;
	if(color_map->color_map_update){
		for (i = 0; i < MAX_OSD_COLOR_NUM; i++) {
			ipu_set_osd_color(base_reg, i, color_map->color_map[i]);
		}
	}
	color_map->color_map_update = 0;

	sta_box = osd_cfg->osd_sta;
	if (osd_cfg->osd_sta_update) {
		for (i = 0; i < MAX_STA_NUM; i++) {
			sta_enable |= sta_box[i].sta_en << i;
			start_x = sta_box[i].start_x;
			start_y = sta_box[i].start_y;
			width = sta_box[i].width;
			height = sta_box[i].height;
			ipu_set_osd_sta_roi(base_reg, shadow_index, osd_index, i,
							start_x, start_y, width, height);
		}
		ipu_set_osd_sta_enable(base_reg, shadow_index, osd_index, sta_enable);
		vio_dbg("OSD[%d]sta enable = 0x%x", osd_index, sta_enable);
	}
	osd_cfg->osd_sta_update = 0;

	if(osd_cfg->osd_sta_level_update){
		for (i = 0; i < MAX_OSD_STA_LEVEL_NUM; i++)
			ipu_set_osd_sta_level(base_reg, shadow_index, i,
					osd_cfg->osd_sta_level[i]);
	}
	osd_cfg->osd_sta_level_update = 0;
}

void ipu_set_roi_enable(struct ipu_subdev *subdev, u32 shadow_index,
			bool roi_en)
{
	u32 id = 0;
	u8 ds_ch = 0;
	u32 __iomem *base_reg;

	id = subdev->id;
	base_reg = subdev->ipu_dev->base_reg;

	if (id == GROUP_ID_US) {
		ipu_set_us_roi_enable(base_reg, shadow_index, roi_en);
	} else if (id >= GROUP_ID_DS0) {
		ds_ch = id - GROUP_ID_DS0;
		ipu_set_ds_roi_enable(base_reg, shadow_index, ds_ch, roi_en);
	}
}

void ipu_set_sc_enable(struct ipu_subdev *subdev, u32 shadow_index,
			bool sc_en)
{
	u32 id = 0;
	u8 ds_ch = 0;
	u32 __iomem *base_reg;

	id = subdev->id;
	base_reg = subdev->ipu_dev->base_reg;

	if (id == GROUP_ID_US) {
		ipu_set_us_enable(base_reg, shadow_index, sc_en);;
	} else if (id >= GROUP_ID_DS0) {
		ds_ch = id - GROUP_ID_DS0;
		ipu_set_ds_enable(base_reg, shadow_index, ds_ch, sc_en);
	}
}

void ipu_hw_set_roi_cfg(struct ipu_subdev *subdev, u32 shadow_index,
			ipu_roi_box_t *roi)
{
	u32 id = 0;
	u8 ds_ch = 0;
	u32 __iomem *base_reg;

	id = subdev->id;
	base_reg = subdev->ipu_dev->base_reg;

	if (id == GROUP_ID_US) {
		ipu_set_us_roi_rect(base_reg, shadow_index, roi->start_x,
					roi->start_y, roi->width, roi->height);
	} else {
		ds_ch = id - GROUP_ID_DS0;
		ipu_set_ds_roi_rect(base_reg, shadow_index, ds_ch,
					roi->start_x, roi->start_y,
					roi->width, roi->height);
	}
}

void ipu_hw_set_scale_cfg(struct ipu_subdev *subdev, u32 shadow_index,
			ipu_scale_info_t *sc_info)
{
	u32 id = 0;
	u8 ds_ch = 0;
	u32 __iomem *base_reg;

	id = subdev->id;
	base_reg = subdev->ipu_dev->base_reg;

	if (id == GROUP_ID_US) {
		ipu_set_us_target(base_reg, shadow_index,
					sc_info->step_x, sc_info->step_y,
					sc_info->tgt_width, sc_info->tgt_height);
	} else {
		ds_ch = id - GROUP_ID_DS0;
		ipu_set_ds_step(base_reg, shadow_index, ds_ch,
					sc_info->step_x, sc_info->step_y,
					sc_info->pre_scale_x, sc_info->pre_scale_y);
		ipu_set_ds_target_size(base_reg, shadow_index, ds_ch,
					sc_info->tgt_width, sc_info->tgt_height);
	}

	vio_dbg("[V%d]%s: stepx = %d, stepy = %d, tgt_width = %d, tgt_height = %d\n",
			id, __func__, sc_info->step_x, sc_info->step_y,
			sc_info->tgt_width, sc_info->tgt_height);
}

void ipu_hw_set_wdma_stride(struct ipu_subdev *subdev, u32 shadow_index,
			u32 stride_y, u32 stride_uv)
{
	u32 id = 0;
	u8 ds_ch = 0;
	u32 __iomem *base_reg;

	id = subdev->id;
	base_reg = subdev->ipu_dev->base_reg;

	if (id == GROUP_ID_US) {
		ipu_set_us_wdma_stride(base_reg, shadow_index,
					stride_y, stride_uv);
	} else {
		ds_ch = id - GROUP_ID_DS0;
		ipu_set_ds_wdma_stride(base_reg, shadow_index, ds_ch,
				   stride_y, stride_uv);
	}
}

void ipu_hw_set_info_cfg(struct ipu_subdev *subdev, u32 shadow_index)
{
	u32 id = 0;
	struct ipu_info_cfg *info_cfg;
	ipu_ds_info_t *sc_info;
	ipu_roi_box_t *roi;
	ipu_scale_info_t *scale_info;

	id = subdev->id;
	if (id < GROUP_ID_US || id > GROUP_ID_DS4) {
		return;
	}

	info_cfg = &subdev->info_cfg;
	sc_info = &info_cfg->sc_info;
	roi = &sc_info->ds_roi_info;
	scale_info = &sc_info->ds_sc_info;

	if (info_cfg->info_update) {
		if (sc_info->ds_roi_en)
			ipu_hw_set_roi_cfg(subdev, shadow_index, roi);
		ipu_set_roi_enable(subdev, shadow_index, sc_info->ds_roi_en);

		if (sc_info->ds_sc_en) {
			ipu_hw_set_scale_cfg(subdev, shadow_index, scale_info);
			ipu_hw_set_wdma_stride(subdev, shadow_index,
					sc_info->ds_stride_y, sc_info->ds_stride_uv);
		}
		ipu_set_sc_enable(subdev, shadow_index, sc_info->ds_sc_en);
		info_cfg->info_update = 0;
	}
}

void ipu_hw_set_cfg(struct ipu_subdev *subdev)
{
	u32 rdy = 0;
	u32 __iomem *base_reg;
	u32 shadow_index = 0;
	struct vio_group *group;

	group = subdev->group;
	if (!group) {
		vio_err("[%s] group null", __func__);
		return;
	}
	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;

	base_reg = subdev->ipu_dev->base_reg;

	rdy = ipu_get_shd_rdy(base_reg);
	rdy = rdy & ~(1 << shadow_index);
	ipu_set_shd_rdy(base_reg, rdy);

	ipu_hw_set_osd_cfg(subdev, shadow_index);
	ipu_hw_set_info_cfg(subdev, shadow_index);

	rdy = ipu_get_shd_rdy(base_reg);
	rdy = rdy | (1 << shadow_index);
	ipu_set_shd_rdy(base_reg, rdy);
}

int ipu_update_osd_sta_roi(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	osd_sta_box_t *osd_sta;
	struct ipu_subdev *subdev;

	subdev = ipu_ctx->subdev;
	osd_sta = subdev->osd_cfg.osd_sta;
	ret = copy_from_user((char *) osd_sta, (u32 __user *) arg,
			   MAX_STA_NUM * sizeof(osd_sta_box_t));
	if (ret)
		return -EFAULT;

	subdev->osd_cfg.osd_sta_update = 1;

	vio_dbg("[S%d][V%d] %s\n", ipu_ctx->group->instance, ipu_ctx->id,
		 __func__);

	return ret;
}

int ipu_update_osd_sta_level(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	u8 *osd_sta_level;
	struct ipu_subdev *subdev;

	subdev = ipu_ctx->subdev;
	osd_sta_level = subdev->osd_cfg.osd_sta_level;
	ret = copy_from_user((char *) osd_sta_level, (u32 __user *) arg,
			   MAX_OSD_STA_LEVEL_NUM * sizeof(u8));
	if (ret)
		return -EFAULT;

	subdev->osd_cfg.osd_sta_level_update = 1;

	vio_dbg("[S%d][V%d] %s\n", ipu_ctx->group->instance, ipu_ctx->id,
		 __func__);

	return ret;
}

int ipu_get_osd_bin(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	u32 i = 0;
	u32 osd_index = 0;
	u32 id = 0;
	u16 sta_bin[MAX_STA_NUM][MAX_STA_BIN_NUM];
	u32 __iomem *base_reg;
	struct ipu_subdev *subdev;

	subdev = ipu_ctx->subdev;
	id = ipu_ctx->id;
	if(id < GROUP_ID_US || id > GROUP_ID_DS1)
		return -EFAULT;

	if(id == GROUP_ID_US)
		osd_index = 2;
	else
		osd_index = id - GROUP_ID_DS0;


	base_reg = subdev->ipu_dev->base_reg;
	for (i = 0; i < MAX_STA_NUM; i++) {
		ipu_get_osd_sta_bin(base_reg, osd_index, i, sta_bin[i]);
	}

	ret = copy_to_user((void __user *) arg, (char *) sta_bin[0],
			 sizeof(u16) * MAX_STA_NUM * MAX_STA_BIN_NUM);
	if (ret)
		return -EFAULT;

	vio_dbg("[S%d][V%d] %s\n", ipu_ctx->group->instance, ipu_ctx->id,
		 __func__);

	return ret;
}

int ipu_channel_wdma_enable(struct ipu_subdev *subdev, bool enable)
{
	int ret = 0;
	u32 rdy = 0;
	u32 id = 0;
	u32 shadow_index = 0;
	struct vio_group *group;
	struct x3_ipu_dev *ipu;
	ipu_ds_info_t *info;

	id = subdev->id;
	group = subdev->group;
	ipu = subdev->ipu_dev;
	if (!group || !ipu) {
		vio_err("[%s] group/ipu null", __func__);
		return -EFAULT;
	}

	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;

	info = &subdev->scale_cfg;

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy & ~(1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	if (enable) {
		ipu_set_roi_enable(subdev, shadow_index, info->ds_roi_en);
		ipu_set_sc_enable(subdev, shadow_index, info->ds_sc_en);
	} else {
		if (shadow_index == 0) {
			if (((ipu->reuse_shadow0_count & 0x11) == 0x11) ||
				((ipu->reuse_shadow0_count & 0x30) == 0x30) ||
				((ipu->reuse_shadow0_count & 0x21) == 0x21)) {
				vio_info("can't disable shadow0 by group%d,reuse(0x%x)\n",
					group->instance, ipu->reuse_shadow0_count);
				return ret;
			}
		}
		ipu_set_roi_enable(subdev, shadow_index, enable);
		ipu_set_sc_enable(subdev, shadow_index, enable);
	}

	if (id == GROUP_ID_SRC) {
		if (enable) {
			ipu_set_ds_roi_enable(ipu->base_reg, shadow_index, 2,
				info->ds_roi_en);
			ipu_set_ds_enable(ipu->base_reg, shadow_index, 2, info->ds_sc_en);
		} else {
			ipu_set_ds_roi_enable(ipu->base_reg, shadow_index, 2, false);
			ipu_set_ds_enable(ipu->base_reg, shadow_index, 2, false);
		}
	}
	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy | (1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	return ret;
}

int ipu_update_ds_ch_param(struct ipu_subdev *subdev, u8 ds_ch,
			   ipu_ds_info_t *ds_config)
{
	int ret = 0;
	u32 rdy = 0;
	u32 shadow_index = 0;
	u16 dst_width = 0, dst_height = 0;
	u16 dst_stepx = 0, dst_stepy = 0, dst_prex = 0, dst_prey = 0;
	u16 ds_stride_y = 0, ds_stride_uv = 0;
	u16 roi_x = 0, roi_y = 0, roi_width = 0, roi_height = 0;
	struct vio_group *group;
	struct x3_ipu_dev *ipu;

	group = subdev->group;
	ipu = subdev->ipu_dev;
	if (!group || !ipu) {
		vio_err("[%s] group/ipu null", __func__);
		return -EFAULT;
	}

	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy & ~(1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	dst_width = ds_config->ds_sc_info.tgt_width;
	dst_height = ds_config->ds_sc_info.tgt_height;
	ipu_set_ds_target_size(ipu->base_reg, shadow_index, ds_ch, dst_width,
			       dst_height);

	ds_stride_y = ds_config->ds_stride_y;
	ds_stride_uv = ds_config->ds_stride_uv;
	ipu_set_ds_wdma_stride(ipu->base_reg, shadow_index, ds_ch,
				   ds_stride_y, ds_stride_uv);

	dst_stepx = ds_config->ds_sc_info.step_x;
	dst_stepy = ds_config->ds_sc_info.step_y;
	dst_prex = ds_config->ds_sc_info.pre_scale_x;
	dst_prey = ds_config->ds_sc_info.pre_scale_y;
	ipu_set_ds_step(ipu->base_reg, shadow_index, ds_ch, dst_stepx,
				   dst_stepy, dst_prex, dst_prey);

	roi_x = ds_config->ds_roi_info.start_x;
	roi_y = ds_config->ds_roi_info.start_y;
	roi_width = ds_config->ds_roi_info.width;
	roi_height = ds_config->ds_roi_info.height;
	ipu_set_ds_roi_rect(ipu->base_reg, shadow_index, ds_ch, roi_x,
			       roi_y, roi_width, roi_height);

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy | (1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	//ipu_set_intr_mask(ipu->base_reg, 0);
	vio_dbg("[%d][ds%d]roi_x = %d, roi_y = %d, roi_width = %d, roi_height = %d\n",
		 shadow_index, ds_ch, roi_x, roi_y, roi_width, roi_height);
	vio_dbg("[%d][ds%d]stride_y = %d, stride_uv = %d\n", shadow_index, ds_ch,
		 ds_stride_y, ds_stride_uv);
	return ret;
}

int ipu_update_ds_param(struct ipu_subdev *subdev, ipu_ds_info_t *ds_config)
{
	int ret = 0;
	u8 ds_ch = 0;
	struct vio_group *group;
	struct x3_ipu_dev *ipu;

	ds_ch = subdev->id - GROUP_ID_DS0;
	ret = ipu_update_ds_ch_param(subdev, ds_ch, ds_config);

	ipu = subdev->ipu_dev;
	group = subdev->group;
	if (!group || !ipu) {
		vio_err("[%s] group/ipu null", __func__);
		return -EFAULT;
	}
	if (ds_config->ds_roi_en || ds_config->ds_sc_en) {
		ipu_set_group_leader(group, subdev->id);
		group->output_flag++;
		set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);
		if (subdev->id == GROUP_ID_DS2) {
			set_bit(IPU_DS2_DMA_OUTPUT, &ipu->state);
		}
	}
	return ret;
}

int ipu_update_us_param(struct ipu_subdev *subdev, ipu_us_info_t *us_config)
{
	u16 dst_width = 0, dst_height = 0;
	u16 dst_stepx = 0, dst_stepy = 0;
	u16 roi_x = 0, roi_y = 0, roi_width = 0, roi_height = 0;
	int ret = 0;
	u32 rdy = 0;
	u32 shadow_index = 0;
	struct vio_group *group;
	struct x3_ipu_dev *ipu;

	group = subdev->group;
	ipu = subdev->ipu_dev;

	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy & ~(1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	dst_width = us_config->us_sc_info.tgt_width;
	dst_height = us_config->us_sc_info.tgt_height;
	dst_stepx = us_config->us_sc_info.step_x;
	dst_stepy = us_config->us_sc_info.step_y;
	ipu_set_us_target(ipu->base_reg, shadow_index, dst_stepx, dst_stepy,
			  dst_width, dst_height);
	ipu_set_us_wdma_stride(ipu->base_reg, shadow_index,
			       us_config->us_stride_y,
			       us_config->us_stride_uv);

	roi_x = us_config->us_roi_info.start_x;
	roi_y = us_config->us_roi_info.start_y;
	roi_width = us_config->us_roi_info.width;
	roi_height = us_config->us_roi_info.height;
	ipu_set_us_roi_rect(ipu->base_reg, shadow_index, roi_x, roi_y,
			       roi_width, roi_height);

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy | (1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);
	vio_dbg("roi_x = %d, roi_y = %d, roi_width = %d, roi_height = %d\n",
		 roi_x, roi_y, roi_width, roi_height);
	vio_dbg("step_x = %d, step_y = %d, tgt_width = %d, tgt_height = %d\n",
		 dst_stepx, dst_stepy, dst_width, dst_height);
	return ret;
}

int ipu_set_path_attr(struct ipu_subdev *subdev, ipu_cfg_t *ipu_cfg)
{
	struct vio_group *group;
	struct x3_ipu_dev *ipu;
	ipu_src_ctrl_t *ipu_ctrl;

	group = subdev->group;
	ipu = subdev->ipu_dev;
	ipu_ctrl = &ipu_cfg->ctrl_info;

	if (ipu_cfg->ctrl_info.source_sel != IPU_FROM_DDR_YUV420) {
		if (test_bit(IPU_DMA_INPUT, &ipu->state)){
			vio_err("IPU DMA input already,can't set otf input\n");
			return -EINVAL;
		}else{
			set_bit(IPU_OTF_INPUT, &ipu->state);
			set_bit(VIO_GROUP_OTF_INPUT, &group->state);
		}
	} else {
		if (test_bit(VIO_GROUP_LEADER, &group->state)) {
			ipu_clear_group_leader(group);
			subdev->skip_flag = true;
		}
		ipu_set_group_leader(group, GROUP_ID_SRC);
		if (test_bit(IPU_OTF_INPUT, &ipu->state)) {
			vio_err("IPU otf input already,can't set dma input\n");
			return -EINVAL;
		} else {
			set_bit(IPU_DMA_INPUT, &ipu->state);
			set_bit(VIO_GROUP_DMA_INPUT, &group->state);
		}
	}

	return 0;
}

int ipu_update_common_param(struct ipu_subdev *subdev,
			ipu_cfg_t *ipu_cfg)
{
	int i = 0;
	int ret = 0;
	u8 Id_en = 0;
	u32 rdy = 0;
	u32 shadow_index = 0;
	struct vio_group *group;
	struct x3_ipu_dev *ipu;
	ipu_src_ctrl_t *ipu_ctrl;
	u16 src_width, src_height, src_stride_uv, src_stride_y;

	group = subdev->group;
	ipu = subdev->ipu_dev;
	if (!group || !ipu) {
		vio_err("[%s] group/ipu null", __func__);
		return -EFAULT;
	}
	ipu_ctrl = &ipu_cfg->ctrl_info;

	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy & ~(1 << shadow_index) & ~(1 << 4);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	// SRC select
	ipu_src_select(ipu->base_reg, ipu_ctrl->source_sel);

	// enable frameid
	Id_en = ((ipu_ctrl->us_frame_id_en)
			| (ipu_ctrl->ds_frame_id_en[0] << 1)
			| (ipu_ctrl->ds_frame_id_en[1] << 2)
			| (ipu_ctrl->ds_frame_id_en[2] << 3)
			| (ipu_ctrl->ds_frame_id_en[3] << 4)
			| (ipu_ctrl->ds_frame_id_en[4] << 5));
	ipu_set_frameid_enable(ipu->base_reg, Id_en);

	///  Source W, H
	src_width = ipu_ctrl->src_width;
	src_height = ipu_ctrl->src_height;
	ipu_set_input_img_size(ipu->base_reg, shadow_index, src_width,
			       src_height);

	if (ipu_ctrl->ds2_to_pym_en == 1) {
		ipu_set_ds2_wdma_enable(ipu->base_reg, shadow_index, 1);
	} else {
		ipu_set_ds2_wdma_enable(ipu->base_reg, shadow_index, 0);
	}

	ipu_update_ds_ch_param(subdev, 2, &ipu_cfg->ds_info[2]);

	///RD Buffer stride
	src_stride_uv = ipu_ctrl->src_stride_uv;
	src_stride_y = ipu_ctrl->src_stride_y;
	ipu_set_rdma_stride(ipu->base_reg, shadow_index, src_stride_y,
			    src_stride_uv);

	/// Set ODS color
	for (i = 0; i < MAX_OSD_COLOR_NUM; i++) {
		ipu_set_osd_color(ipu->base_reg, i, color[i]);
	}

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy | (1 << shadow_index) | (1 << 4);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	memcpy(&subdev->scale_cfg, &ipu_cfg->ds_info[2], sizeof(ipu_ds_info_t));

	return ret;
}

void ipu_update_hw_param(struct ipu_subdev *subdev)
{
	int i = 0;
	ipu_cfg_t *ipu_cfg;
	ipu_us_info_t *us_info;
	ipu_ds_info_t *ds_info;
	struct vio_group *group;

	ipu_cfg = &subdev->ipu_cfg;
	group = subdev->group;

	for (i = 0; i < MAX_DEVICE; i++) {
		subdev = group->sub_ctx[i];
		if (subdev) {
			switch (i) {
			case GROUP_ID_SRC:
				ipu_update_common_param(subdev, ipu_cfg);
				break;
			case GROUP_ID_US:
				us_info = &ipu_cfg->us_info;
				ipu_update_us_param(subdev, us_info);
				break;
			case GROUP_ID_DS0:
			case GROUP_ID_DS1:
			case GROUP_ID_DS2:
			case GROUP_ID_DS3:
			case GROUP_ID_DS4:
				ds_info = &ipu_cfg->ds_info[i-2];
				ipu_update_ds_param(subdev, ds_info);
				break;
			default:
				break;
			}
		}
	}
}

int ipu_video_init(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	ipu_ds_info_t *scale_config;
	ipu_cfg_t *ipu_cfg;
	struct x3_ipu_dev *ipu;
	struct vio_group *group;
	struct ipu_subdev *subdev;

	ipu = ipu_ctx->ipu_dev;
	group = ipu_ctx->group;

	if (!(ipu_ctx->state & (BIT(VIO_VIDEO_S_INPUT) | BIT(VIO_VIDEO_REBUFS)))) {
		vio_err("[%s] invalid INIT is requested(%lX)", __func__, ipu_ctx->state);
		return -EINVAL;
	}

	subdev = ipu_ctx->subdev;
	if (test_and_set_bit(IPU_SUBDEV_INIT, &subdev->state)) {
		vio_info("subdev already init, current refcount(%d)\n",
				atomic_read(&subdev->refcount));
		ipu_ctx->state = BIT(VIO_VIDEO_INIT);
		return ret;
	}

	if (ipu_ctx->id == GROUP_ID_SRC) {
		ipu_cfg = &subdev->ipu_cfg;
		ret = copy_from_user((char *)ipu_cfg, (u32 __user *) arg,
				   sizeof(ipu_cfg_t));
		if (ret)
			goto err;
		ret = ipu_update_common_param(subdev, ipu_cfg);
		if (ret)
			goto err;
		ret = ipu_set_path_attr(subdev, ipu_cfg);
		if (ret)
			goto err;
	} else if (ipu_ctx->id == GROUP_ID_US) {
		scale_config = &subdev->scale_cfg;
		ret = copy_from_user((char *) scale_config, (u32 __user *) arg,
				   sizeof(ipu_us_info_t));
		if (ret)
			goto err;

		ret = ipu_update_us_param(subdev, (ipu_us_info_t*) scale_config);
		if (ret)
			goto err;

		if (scale_config->ds_roi_en || scale_config->ds_sc_en) {
			group->output_flag++;
			ipu_set_group_leader(group, subdev->id);
			set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);
		}
	} else if (ipu_ctx->id >= GROUP_ID_DS0) {
		scale_config = &subdev->scale_cfg;
		ret = copy_from_user((char *) scale_config, (u32 __user *) arg,
				   sizeof(ipu_ds_info_t));
		if (ret)
			goto err;
		ret = ipu_update_ds_param(subdev, scale_config);
		if (ret)
			goto err;
	}

	if(!test_bit(IPU_REUSE_SHADOW0, &ipu->state)
		&& group->instance >= MAX_SHADOW_NUM)
		set_bit(IPU_REUSE_SHADOW0, &ipu->state);

	if (group->instance >= MAX_SHADOW_NUM || group->instance == 0) {
		ipu->reuse_shadow0_count |= 1 << group->instance;
		vio_info("reuse_shadow0_count = %d\n", ipu->reuse_shadow0_count);
	}

	if (subdev->leader && !subdev->skip_flag)
		vio_group_task_start(group->gtask);
	atomic_inc(&group->node_refcount);

	ipu_ctx->state = BIT(VIO_VIDEO_INIT);

	vio_info("[S%d][V%d]%s done\n", group->instance, ipu_ctx->id, __func__);

	return ret;
err:
	clear_bit(IPU_SUBDEV_INIT, &subdev->state);
	return ret;
}

int ipu_bind_chain_group(struct ipu_video_ctx *ipu_ctx, int instance)
{
	int ret = 0;
	int id = 0;
	struct vio_group *group;
	struct x3_ipu_dev *ipu;
	struct ipu_subdev *subdev;
	int i;

	if (!(ipu_ctx->state & BIT(VIO_VIDEO_OPEN))) {
		vio_err("[%s]invalid BIND is requested(%lX)\n",
			__func__, ipu_ctx->state);
		return -EINVAL;
	}

	if (instance < 0 || instance >= VIO_MAX_STREAM) {
		vio_err("wrong instance id(%d)\n", instance);
		return -EFAULT;
	}

	ipu = ipu_ctx->ipu_dev;

	group = vio_get_chain_group(instance, GROUP_ID_IPU);
	if (!group)
		return -EFAULT;

	id = ipu_ctx->id;
	subdev = &ipu->subdev[instance][id];
	ipu->group[instance] = group;

	group->sub_ctx[id] = subdev;
	ipu_ctx->group = group;
	ipu_ctx->subdev = subdev;
	ipu_ctx->framemgr = &subdev->framemgr;
	subdev->ipu_dev = ipu;
	subdev->group = group;
	subdev->id = id;

	spin_lock(&subdev->slock);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if(!test_bit(i, &subdev->val_ctx_mask)) {
			subdev->ctx[i] = ipu_ctx;
			ipu_ctx->ctx_index = i;
			set_bit(i, &subdev->val_ctx_mask);
			break;
		}
	}
	spin_unlock(&subdev->slock);
	if (i == VIO_MAX_SUB_PROCESS) {
		vio_err("alreay open too much for one pipeline\n");
		return -EFAULT;
	}
	if (atomic_inc_return(&subdev->refcount) == 1)
		frame_manager_init_mp(ipu_ctx->framemgr);

	group->frame_work = ipu_frame_work;
	group->gtask = &ipu->gtask;
	group->gtask->id = group->id;
	ipu->statistic.enable[instance] = 1;
	ipu->statistic.enable_subdev[instance] |= BIT(ipu_ctx->id);
	ipu_ctx->belong_pipe = instance;

	vio_info("[S%d][V%d] %s done\n", instance, id, __func__);
	ipu_ctx->state = BIT(VIO_VIDEO_S_INPUT);

	return ret;
}

int ipu_video_streamon(struct ipu_video_ctx *ipu_ctx)
{
	u32 cnt = 20;
	struct x3_ipu_dev *ipu;
	struct vio_group *group;
	struct ipu_subdev *subdev;

	ipu = ipu_ctx->ipu_dev;
	group = ipu_ctx->group;

	if (!(ipu_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS)
			| BIT(VIO_VIDEO_INIT)))) {
		vio_err("[%s][V%02d] invalid STREAM_ON is requested(%lX)\n",
			__func__, group->instance, ipu_ctx->state);
		return -EINVAL;
	}

	subdev = ipu_ctx->subdev;
	ipu_channel_wdma_enable(subdev, true);

	if (atomic_read(&ipu->rsccount) > 0)
		goto p_inc;

	if (group->leader && test_bit(IPU_OTF_INPUT, &ipu->state)) {
		while(1) {
			if (test_bit(IPU_HW_CONFIG, &ipu->state))
				break;

			msleep(5);
			cnt--;
			if (cnt == 0) {
				vio_info("%s timeout\n", __func__);
				break;
			}
		}
	}
	//ipu_set_ddr_fifo_thred(ipu->base_reg, 0, 0);
	//ipu_set_ddr_fifo_thred(ipu->base_reg, 1, 0);

	set_bit(IPU_HW_RUN, &ipu->state);
p_inc:
	atomic_inc(&ipu->rsccount);

	ipu_ctx->state = BIT(VIO_VIDEO_START);

	vio_info("[S%d][V%d] %s\n", group->instance, ipu_ctx->id,
		 __func__);

	return 0;
}

int ipu_video_streamoff(struct ipu_video_ctx *ipu_ctx)
{
	struct x3_ipu_dev *ipu_dev;
	struct vio_group *group;
	struct ipu_subdev *subdev;

	if (!(ipu_ctx->state & BIT(VIO_VIDEO_START))) {
		vio_err("[%s][V%02d] invalid STREAM_OFF is requested(%lX)\n",
			__func__, ipu_ctx->group->instance, ipu_ctx->state);
		return -EINVAL;
	}

	ipu_dev = ipu_ctx->ipu_dev;
	group = ipu_ctx->group;
	subdev = ipu_ctx->subdev;

	/* last process of this sub_mp */
	if (atomic_read(&subdev->refcount) == 1)
		ipu_channel_wdma_enable(subdev, false);

	if (atomic_dec_return(&ipu_dev->rsccount) > 0
		&& !test_bit(IPU_HW_FORCE_STOP, &ipu_dev->state))
		goto p_dec;

	vio_reset_module(group->id);

	clear_bit(IPU_HW_RUN, &ipu_dev->state);
p_dec:
	if (ipu_ctx->framemgr->frames_mp[ipu_ctx->frm_fst_ind] != NULL) {
		/* wait for frame to be transfered to USED or FREE */
		if (atomic_read(&subdev->refcount) > 1)
			frame_manager_flush_mp_prepare(ipu_ctx->framemgr,
				ipu_ctx->frm_fst_ind, ipu_ctx->frm_num,
				ipu_ctx->ctx_index);
	}

	ipu_ctx->state = BIT(VIO_VIDEO_STOP);

	vio_info("[S%d][V%d]%s:proc %d\n", group->instance, ipu_ctx->id,
		__func__, ipu_ctx->ctx_index);

	return 0;
}

int ipu_video_s_stream(struct ipu_video_ctx *ipu_ctx, bool enable)
{
	if (enable)
		ipu_video_streamon(ipu_ctx);
	else
		ipu_video_streamoff(ipu_ctx);

	return 0;
}

int ipu_video_reqbufs(struct ipu_video_ctx *ipu_ctx, u32 buffers)
{
	int ret = 0;
	int i = 0;
	u32 first_index;
	u32 instance = 0;
	struct vio_framemgr *framemgr;
	struct ipu_subdev *subdev;
	struct x3_ipu_dev *ipu_dev;

	if (!(ipu_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS) |
		      BIT(VIO_VIDEO_INIT) | BIT(VIO_VIDEO_S_INPUT)))) {
		vio_err("[%s][V%02d] invalid REQBUFS is requested(%lX)\n",
			__func__, ipu_ctx->group->instance, ipu_ctx->state);
		return -EINVAL;
	}

	subdev = ipu_ctx->subdev;
	ipu_dev = ipu_ctx->ipu_dev;
	framemgr = ipu_ctx->framemgr;
	ret = frame_manager_open_mp(framemgr, buffers, &first_index);
	if (ret) {
		vio_err("frame manage open failed, ret(%d)", ret);
		return ret;
	}
	ipu_ctx->frm_fst_ind = first_index;
	ipu_ctx->frm_num = buffers;
	instance = ipu_ctx->group->instance;
	for (i = first_index; i < (first_index + buffers); i++) {
		framemgr->frames_mp[i]->data = ipu_ctx->group;
		framemgr->frames_mp[i]->mp_work = &ipu_dev->vwork[instance][i].work;
		ipu_dev->vwork[instance][i].group = ipu_ctx->group;
	}

	ipu_ctx->state = BIT(VIO_VIDEO_REBUFS);
	set_bit(IPU_SUBDEV_REQBUF, &subdev->state);
	vio_info("[S%d][V%d]%s buffer number %d first index %d\n",
		ipu_ctx->group->instance, ipu_ctx->id, __func__, buffers, first_index);

	return ret;
}

int ipu_video_getindex(struct ipu_video_ctx *ipu_ctx)
{
	u32 buf_index;

	if (!ipu_ctx->group) {
		vio_err("[%s] group null", __func__);
		return -EFAULT;
	}
	if (!(ipu_ctx->state & BIT(VIO_VIDEO_REBUFS))) {
		vio_err("[%s][V%02d]state error when get buf index(%lX)\n",
			__func__, ipu_ctx->group->instance, ipu_ctx->state);
		return -EINVAL;
	}

	buf_index = ipu_ctx->frm_fst_ind;
	if (buf_index >= VIO_MP_MAX_FRAMES) {
		vio_err("[%s][V%02d]index too large(%d).\n",
			__func__, ipu_ctx->group->instance, buf_index);
		return -EFAULT;
	}

	return buf_index;
}

int ipu_video_qbuf(struct ipu_video_ctx *ipu_ctx, struct frame_info *frameinfo)
{
	int ret = 0;
	int index = 0;
	unsigned long flags;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	struct ipu_subdev *subdev;
	struct x3_ipu_dev *ipu;

	index = frameinfo->bufferindex;
	framemgr = ipu_ctx->framemgr;
	subdev = ipu_ctx->subdev;
	group = ipu_ctx->group;
	ipu = ipu_ctx->ipu_dev;
	if (index >= framemgr->max_index) {
		vio_err("[S%d] %s index err(%d-%d).\n", group->instance,
		__func__, index, framemgr->max_index);
		return -EINVAL;
	}
	if (framemgr->index_state[index] == FRAME_IND_FREE) {
		vio_err("[S%d] %s index%d state err.\n", group->instance,
			__func__, index);
		return -EINVAL;
	}

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = framemgr->frames_mp[index];
	if (frame == NULL) {
		vio_err("[S%d] %s frame null, index %d.\n", group->instance,
			__func__, index);
		ret = -EFAULT;
		goto err;
	}
	ipu_ctx->frm_num_usr--;
	if (frame->state == FS_FREE) {
		framemgr->dispatch_mask[index] &= ~(1 << ipu_ctx->ctx_index);
		if (framemgr->index_state[index] == FRAME_IND_STREAMOFF) {
			vio_dbg("[S%d][V%d] q fail:FREE proc%d bidx%d is streaming off",
				group->instance, ipu_ctx->id,
				ipu_ctx->ctx_index, index);
			ret = 0;
			goto err;
		}
		vio_dbg("[S%d][V%d] q:FREE->REQ,proc%d bidx%d",
			group->instance, ipu_ctx->id,
			ipu_ctx->ctx_index, index);
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_REQUEST);
	} else if (frame->state == FS_USED) {
		framemgr->dispatch_mask[index] &= ~(1 << ipu_ctx->ctx_index);
		if (framemgr->dispatch_mask[index] == 0) {
			if (framemgr->index_state[index] == FRAME_IND_STREAMOFF) {
				vio_dbg("[S%d][V%d] q fail:USED proc%d bidx%d is streaming off",
					group->instance, ipu_ctx->id,
					ipu_ctx->ctx_index, index);
				ret = 0;
				goto err;
			}
			vio_dbg("[S%d][V%d] q:USED->REQ,proc%d bidx%d",
				group->instance, ipu_ctx->id,
				ipu_ctx->ctx_index, index);
			memcpy(&frame->frameinfo, frameinfo,
				sizeof(struct frame_info));
			trans_frame(framemgr, frame, FS_REQUEST);
		} else {
			if ((framemgr->queued_count[FS_REQUEST] <= 2)
			&& (ipu_index_owner(ipu_ctx, index)
				== VIO_BUFFER_THIS)) {
				vio_info("[S%d][V%d] q:force proc%d bidx%d to req,mask %x",
					group->instance, ipu_ctx->id,
					ipu_ctx->ctx_index, index,
					framemgr->dispatch_mask[index]);
				framemgr->dispatch_mask[index] = 0;
				memcpy(&frame->frameinfo, frameinfo,
					sizeof(struct frame_info));
				trans_frame(framemgr, frame, FS_REQUEST);
			} else {
				vio_dbg("[S%d][V%d] q:disp mask%d,proc%d bidx%d",
					group->instance, ipu_ctx->id,
					framemgr->dispatch_mask[index],
					ipu_ctx->ctx_index, index);
				ret = 0;
				goto err;
			}
		}
	} else {
		vio_err("[S%d][V%d] q:proc%d frame(%d) is invalid state(%d)\n",
			group->instance, ipu_ctx->id, ipu_ctx->ctx_index,
			index, frame->state);
		ret = -EINVAL;
		goto err;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	if (subdev->leader == true && group->leader) {
		vio_group_start_trigger_mp(group, frame);
	}

	if (ipu_ctx->ctx_index == 0)
		ipu->statistic.q_normal\
			[ipu_ctx->group->instance][ipu_ctx->id]++;
	vio_dbg("[S%d][V%d] %s index %d\n", group->instance, ipu_ctx->id,
		__func__, frameinfo->bufferindex);
	return ret;
err:
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	return ret;
}

int ipu_video_dqbuf(struct ipu_video_ctx *ipu_ctx, struct frame_info *frameinfo)
{
	int ret = 0;
	unsigned long flags;
	struct list_head *done_list;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct ipu_subdev *subdev;
	u32 bufindex;
	u32 ctx_index;
	struct x3_ipu_dev *ipu;

	framemgr = ipu_ctx->framemgr;
	subdev = ipu_ctx->subdev;
	ctx_index = ipu_ctx->ctx_index;
	ipu = ipu_ctx->ipu_dev;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	#if 0
	if (ipu_ctx->frm_num_usr > (int)ipu_ctx->frm_num) {
		ret = -EFAULT;
		ipu_ctx->event = 0;
		vio_err("[S%d][V%d] %s (p%d) dq too much frame(%d-%d).",
			ipu_ctx->group->instance, ipu_ctx->id, __func__,
			ipu_ctx->ctx_index, ipu_ctx->frm_num_usr,
			ipu_ctx->frm_num);
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		return ret;
	}
	#endif
	framemgr->ctx_mask |= (1 << ipu_ctx->ctx_index);
	done_list = &framemgr->queued_list[FS_COMPLETE];
	if (!list_empty(done_list)) {
		frame = peek_frame(framemgr, FS_COMPLETE);
		if (frame) {
			memcpy(frameinfo, &frame->frameinfo,
				sizeof(struct frame_info));
			trans_frame(framemgr, frame, FS_USED);
			bufindex = frame->frameinfo.bufferindex;
			framemgr->dispatch_mask[bufindex] = framemgr->ctx_mask;
			/* copy frame_info to subdev*/
			if (atomic_read(&subdev->refcount) > 1) {
				memcpy(&subdev->frameinfo, &frame->frameinfo,
					sizeof(struct frame_info));
			}

			ipu_ctx->event = 0;
			ipu_ctx->frm_num_usr++;
			vio_dbg("[S%d][V%d] %s (p%d b%d f%d) from FS_COMPLETE.",
				ipu_ctx->group->instance, ipu_ctx->id, __func__,
				ipu_ctx->ctx_index,
				frame->frameinfo.bufferindex,
				frame->frameinfo.frame_id);
		}
		if (ipu_ctx->ctx_index == 0)
			ipu->statistic.dq_normal\
				[ipu_ctx->group->instance][ipu_ctx->id]++;
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		return ret;
	} else {
		if (atomic_read(&subdev->refcount) == 1) {
			ret = -EFAULT;
			ipu_ctx->event = 0;
			vio_err("[S%d][V%d] %s (p%d) complete empty.",
				ipu_ctx->group->instance, ipu_ctx->id, __func__,
				ipu_ctx->ctx_index);
			if (ipu_ctx->ctx_index == 0)
				ipu->statistic.dq_err\
					[ipu_ctx->group->instance][ipu_ctx->id]++;
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			return ret;
		}
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	/* copy frame_info from subdev */
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	if (ipu_ctx->event == VIO_FRAME_DONE) {
		bufindex = subdev->frameinfo.bufferindex;
		memcpy(frameinfo, &subdev->frameinfo, sizeof(struct frame_info));
		ipu_ctx->frm_num_usr++;
	} else {
		ret = -EFAULT;
		vio_err("[S%d] %s proc%d no frame, event %d.\n",
			ipu_ctx->group->instance, __func__, ctx_index,
			ipu_ctx->event);
		ipu_ctx->event = 0;
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		goto DONE;
	}
	ipu_ctx->event = 0;
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	vio_dbg("[S%d][V%d] %s (p%d b%d f%d) form Subdev\n",
		ipu_ctx->group->instance,
		ipu_ctx->id, __func__, ipu_ctx->ctx_index,
		frameinfo->bufferindex, frameinfo->frame_id);
DONE:
	return ret;
}

void ipu_video_user_stats(struct ipu_video_ctx *ipu_ctx,
	struct user_statistic *stats)
{
	struct x3_ipu_dev *ipu;
	struct vio_group *group;
	struct user_statistic *stats_in_drv;
	u32 instance, dev_id;

	ipu = ipu_ctx->ipu_dev;
	group = ipu_ctx->group;
	if (!ipu || !group) {
		vio_err("%s init err", __func__);
	}

	if (ipu_ctx->ctx_index == 0) {
		instance = group->instance;
		dev_id = ipu_ctx->id;
		stats_in_drv = &ipu->statistic.user_stats[instance][dev_id];
		stats_in_drv->cnt[USER_STATS_NORM_FRM] =
			stats->cnt[USER_STATS_NORM_FRM];
		stats_in_drv->cnt[USER_STATS_SEL_TMOUT] =
			stats->cnt[USER_STATS_SEL_TMOUT];
		stats_in_drv->cnt[USER_STATS_DROP] =
			stats->cnt[USER_STATS_DROP];
	}
}

static long x3_ipu_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int ret = 0;
	int buffers = 0;
	int enable = 0;
	int instance = 0;
	struct ipu_video_ctx *ipu_ctx;
	struct frame_info frameinfo;
	struct vio_group *group;
	u32 buf_index;
	struct user_statistic stats;

	ipu_ctx = file->private_data;
	BUG_ON(!ipu_ctx);
	group = ipu_ctx->group;

	if (_IOC_TYPE(cmd) != IPU_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case IPU_IOC_INIT:
		ret = ipu_video_init(ipu_ctx, arg);
		break;
	case IPU_IOC_STREAM:
		ret = get_user(enable, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		ipu_video_s_stream(ipu_ctx, ! !enable);
		break;
	case IPU_IOC_DQBUF:
		ret = ipu_video_dqbuf(ipu_ctx, &frameinfo);
		if (ret)
			return -EFAULT;
		ret = copy_to_user((void __user *) arg, (char *) &frameinfo,
				 sizeof(struct frame_info));
		if (ret)
			return -EFAULT;
		break;
	case IPU_IOC_QBUF:
		ret = copy_from_user((char *) &frameinfo, (u32 __user *) arg,
				   sizeof(struct frame_info));
		if (ret)
			return -EFAULT;
		ipu_video_qbuf(ipu_ctx, &frameinfo);
		break;
	case IPU_IOC_REQBUFS:
		ret = get_user(buffers, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		ipu_video_reqbufs(ipu_ctx, buffers);
		break;
	case IPU_IOC_GET_INDEX:
		buf_index = ipu_video_getindex(ipu_ctx);
		if (buf_index < 0)
			return -EFAULT;
		ret = put_user(buf_index, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		break;
	case IPU_IOC_OSD_ROI:
		ret = ipu_update_osd_roi(ipu_ctx, arg);
		break;
	case IPU_IOC_OSD_ADDR:
		ret = ipu_update_osd_addr(ipu_ctx, arg);
		break;
	case IPU_IOC_OSD_STA:
		ret = ipu_update_osd_sta_roi(ipu_ctx, arg);
		break;
	case IPU_IOC_OSD_STA_LEVEL:
		ret = ipu_update_osd_sta_level(ipu_ctx, arg);
		break;
	case IPU_IOC_OSD_STA_BIN:
		ret = ipu_get_osd_bin(ipu_ctx, arg);
		break;
	case IPU_IOC_OSD_COLOR_MAP:
		ret = ipu_update_osd_color_map(ipu_ctx, arg);
		break;
	case IPU_IOC_SCALE_INFO:
		ret = ipu_update_scale_info(ipu_ctx, arg);
		break;
	case IPU_IOC_BIND_GROUP:
		ret = get_user(instance, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		ret = ipu_bind_chain_group(ipu_ctx, instance);
		break;
	case IPU_IOC_EOS:
		ret = get_user(instance, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		vio_bind_group_done(instance);
		break;
	case IPU_IOC_USER_STATS:
		ret = copy_from_user((char *) &stats, (u32 __user *) arg,
				   sizeof(struct user_statistic));
		if (ret)
			return -EFAULT;
		ipu_video_user_stats(ipu_ctx, &stats);
		break;
	default:
		vio_err("wrong ioctl command\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

int x3_ipu_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret = 0;
	struct ipu_video_ctx *ipu_ctx;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	int buffer_index;
	u32 paddr;
	u32 addr[2];
	u32 size;
	unsigned long flags;

	ipu_ctx = file->private_data;
	if (!ipu_ctx) {
		vio_err("%s ipu_ctx is null.", __func__);
		ret = -EFAULT;
		goto err;
	}

	framemgr = ipu_ctx->framemgr;
	buffer_index = vma->vm_pgoff >> 1;
	ret = ipu_index_owner(ipu_ctx, buffer_index);
	if( ret != VIO_BUFFER_OTHER ) {
		vio_err("[S%d][V%d] %s proc %d error,index %d not other's",
			ipu_ctx->group->instance, ipu_ctx->id, __func__,
			ipu_ctx->ctx_index, buffer_index);
		ret = -EFAULT;
		goto err;
	}

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = framemgr->frames_mp[buffer_index];
	if (frame) {
		addr[0] = frame->frameinfo.addr[0];
		addr[1] = frame->frameinfo.addr[1];
		size = frame->frameinfo.width * frame->frameinfo.height;
	} else {
		vio_err("[S%d][V%d] %s proc %d error,frame null",
			ipu_ctx->group->instance, ipu_ctx->id, __func__,
			ipu_ctx->ctx_index);
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		ret = -EFAULT;
		goto err;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	if ((vma->vm_pgoff&0x01) == 0) {
		paddr = addr[0];	// y
	} else {
		paddr = addr[1];	// uv
		size = (size >> 1);
	}

	vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP;
	if (remap_pfn_range(vma, vma->vm_start, paddr >> PAGE_SHIFT,
		vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		vio_err("[S%d][V%d] %s proc %d error,remap.",
			ipu_ctx->group->instance, ipu_ctx->id, __func__,
			ipu_ctx->ctx_index);
		ret = -EAGAIN;
		goto err;
	}

	vio_dbg("[S%d][V%d] %s map success, proc %d index %d size %d paddr %x.",
		ipu_ctx->group->instance, ipu_ctx->id, __func__,
		ipu_ctx->ctx_index, buffer_index, size, paddr);
	return 0;
err:
	return ret;
}

enum buffer_owner ipu_index_owner(struct ipu_video_ctx *ipu_ctx,
	u32 index)
{
	if (index >= VIO_MP_MAX_FRAMES)
		return VIO_BUFFER_OWN_INVALID;
	else if ((index >= ipu_ctx->frm_fst_ind)
		&&(index < (ipu_ctx->frm_fst_ind + ipu_ctx->frm_num)))
		return VIO_BUFFER_THIS;
	else
		return VIO_BUFFER_OTHER;
}

void ipu_set_iar_output(struct ipu_subdev *subdev, struct vio_frame *frame)
{
#ifdef X3_IAR_INTERFACE
	int ret = 0;
	int i = 0;
	u8 dis_instance[2] = {0, 0};
	u8 display_layer[2] ={0, 0};
	struct vio_group *group;

	group = subdev->group;
	ret = ipu_get_iar_display_type(dis_instance, display_layer);
	if (!ret) {
		for (i = 0; i < 2; i++) {
			if(group->instance == dis_instance[i] &&
					subdev->id == display_layer[i]) {
				ipu_set_display_addr(i, frame->frameinfo.addr[0],
						frame->frameinfo.addr[1]);
			}
			vio_dbg("[D%d]IPU display_layer = %d, dis_instance = %d", i,
				display_layer[i], dis_instance[i]);
		}
	}

#endif
}

void ipu_frame_done(struct ipu_subdev *subdev)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	struct ipu_video_ctx *ipu_ctx;
	struct x3_ipu_dev *ipu;
	unsigned long flags;
	u32 i;
	u32 event = 0;

	group = subdev->group;
	ipu = subdev->ipu_dev;
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
		vio_set_stat_info(group->instance, IPU_FS + subdev->id,
				group->frameid.frame_id);

		vio_dbg("done bidx%d fid%d ",
			frame->frameinfo.bufferindex,
			frame->frameinfo.frame_id);
		ipu_set_iar_output(subdev, frame);
		event = VIO_FRAME_DONE;
		trans_frame(framemgr, frame, FS_COMPLETE);
		ipu->statistic.fe_normal[group->instance][subdev->id]++;
	} else {
		ipu->statistic.fe_lack_buf[group->instance][subdev->id]++;
		event = VIO_FRAME_NDONE;
		vio_err("[S%d][V%d]IPU PROCESS queue has no member;\n",
				group->instance, subdev->id);
		vio_err("[S%d][V%d][FRM](%d %d %d %d %d)\n",
			group->instance,
			subdev->id,
			framemgr->queued_count[FS_FREE],
			framemgr->queued_count[FS_REQUEST],
			framemgr->queued_count[FS_PROCESS],
			framemgr->queued_count[FS_COMPLETE],
			framemgr->queued_count[FS_USED]);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	spin_lock(&subdev->slock);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if (test_bit(i, &subdev->val_ctx_mask)) {
			ipu_ctx = subdev->ctx[i];
			ipu_ctx->event = event;
			wake_up(&ipu_ctx->done_wq);
		}
	}
	spin_unlock(&subdev->slock);
}

void ipu_frame_ndone(struct ipu_subdev *subdev)
{
	struct ipu_video_ctx *ipu_ctx = NULL;
	int i = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	unsigned long flags;

	spin_lock(&subdev->slock);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if (test_bit(i, &subdev->val_ctx_mask))
			ipu_ctx = subdev->ctx[i];
	}
	if(!ipu_ctx) {
		spin_unlock(&subdev->slock);
		vio_err("%s:%d subdev.ctx[0] is null .\n", __func__, __LINE__);
		return;
	}
	spin_unlock(&subdev->slock);

	group = subdev->group;
	framemgr = &subdev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		vio_dbg("ndone bidx%d fid%d, proc->req.",
			frame->frameinfo.bufferindex,
			frame->frameinfo.frame_id);
		trans_frame(framemgr, frame, FS_REQUEST);
		if (subdev->leader == true && group->leader)
			vio_group_start_trigger_mp(group, frame);
	} else {
		vio_err("[S%d][V%d]ndone IPU PROCESS queue has no member;\n",
				group->instance, subdev->id);
		vio_err("[S%d][V%d][FRM](%d %d %d %d %d)\n",
			group->instance,
			subdev->id,
			framemgr->queued_count[FS_FREE],
			framemgr->queued_count[FS_REQUEST],
			framemgr->queued_count[FS_PROCESS],
			framemgr->queued_count[FS_COMPLETE],
			framemgr->queued_count[FS_USED]);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	spin_lock(&subdev->slock);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if (test_bit(i, &subdev->val_ctx_mask)) {
			ipu_ctx = subdev->ctx[i];
			ipu_ctx->event = VIO_FRAME_DONE;
			wake_up(&ipu_ctx->done_wq);
		}
	}
	spin_unlock(&subdev->slock);
}

static void ipu_diag_report(uint8_t errsta, unsigned int status)
{
	unsigned int sta;

	sta = status;
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_VIO,
				EventIdVioIpuErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&sta,
				sizeof(unsigned int));
	} else {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_VIO,
				EventIdVioIpuErr,
				DiagEventStaSuccess);
	}
}

static irqreturn_t ipu_isr(int irq, void *data)
{
	u32 status = 0;
	bool drop_flag = 0;
	u32 instance = 0;
	u32 size_err = 0;
	u32 err_status = 0;
	u32 err_occured = 0;
	struct x3_ipu_dev *ipu;
	struct vio_group *group;
	struct vio_group_task *gtask;
	struct ipu_subdev	*subdev;

	ipu = data;
	gtask = &ipu->gtask;
	instance = atomic_read(&ipu->instance);
	group = ipu->group[instance];

	ipu_get_intr_status(ipu->base_reg, &status, true);
	size_err = ipu_get_size_err(ipu->base_reg);
	err_status = ipu_get_err_status(ipu->base_reg);
	vio_dbg("%s status = 0x%x\n", __func__, status);

	if (size_err || err_status) {
		ipu_clear_size_err(ipu->base_reg, 1);
		ipu_clear_size_err(ipu->base_reg, 0);
		vio_warn("IPU size error detection(0x%x)\n", size_err);
		vio_warn("IPU size error status(0x%x)\n", err_status);
		err_occured = 1;
	}

	if (status & (1 << INTR_IPU_US_FRAME_DROP)) {
		vio_err("[S%d]US Frame drop\n", instance);
		subdev = group->sub_ctx[GROUP_ID_US];
		if (subdev)
			ipu_frame_ndone(subdev);

		err_occured = 2;
		ipu->frame_drop_count++;
		ipu->statistic.hard_frame_drop[instance][GROUP_ID_US]++;
		drop_flag = true;
	}

	if (status & (1 << INTR_IPU_DS0_FRAME_DROP)) {
		vio_err("[S%d]DS0 Frame drop\n", instance);
		subdev = group->sub_ctx[GROUP_ID_DS0];
		if (subdev)
			ipu_frame_ndone(subdev);

		err_occured = 2;
		ipu->frame_drop_count++;
		ipu->statistic.hard_frame_drop[instance][GROUP_ID_DS0]++;
		drop_flag = true;
	}

	if (status & (1 << INTR_IPU_DS1_FRAME_DROP)) {
		vio_err("[S%d]DS1 Frame drop\n", instance);
		subdev = group->sub_ctx[GROUP_ID_DS1];
		if (subdev)
			ipu_frame_ndone(subdev);

		err_occured = 2;
		ipu->frame_drop_count++;
		ipu->statistic.hard_frame_drop[instance][GROUP_ID_DS1]++;
		drop_flag = true;
	}

	if (status & (1 << INTR_IPU_DS2_FRAME_DROP)) {
		vio_err("[S%d]DS2 Frame drop\n", instance);
		subdev = group->sub_ctx[GROUP_ID_DS2];
		if (subdev)
			ipu_frame_ndone(subdev);

		err_occured = 2;
		ipu->frame_drop_count++;
		ipu->statistic.hard_frame_drop[instance][GROUP_ID_DS2]++;
		drop_flag = true;
	}

	if (status & (1 << INTR_IPU_DS3_FRAME_DROP)) {
		vio_err("[S%d]DS3 Frame drop\n", instance);
		subdev = group->sub_ctx[GROUP_ID_DS3];
		if (subdev)
			ipu_frame_ndone(subdev);

		err_occured = 2;
		ipu->frame_drop_count++;
		ipu->statistic.hard_frame_drop[instance][GROUP_ID_DS3]++;
		drop_flag = true;
	}

	if (status & (1 << INTR_IPU_DS4_FRAME_DROP)) {
		vio_err("[S%d]DS4 Frame drop\n", instance);
		subdev = group->sub_ctx[GROUP_ID_DS4];
		if (subdev)
			ipu_frame_ndone(subdev);

		err_occured = 2;
		ipu->frame_drop_count++;
		ipu->statistic.hard_frame_drop[instance][GROUP_ID_DS4]++;
		drop_flag = true;
	}

	if (status & (1 << INTR_IPU_FRAME_DONE)) {
		if (!group->leader)
			vio_group_done(group);

		if (test_bit(IPU_DMA_INPUT, &ipu->state)) {
			vio_group_done(group);
			subdev = group->sub_ctx[GROUP_ID_SRC];
			if (subdev)
				ipu_frame_done(subdev);
		}
	}

	if (status & (1 << INTR_IPU_US_FRAME_DONE)) {
		subdev = group->sub_ctx[GROUP_ID_US];
		if (subdev)
			ipu_frame_done(subdev);
	}

	if (status & (1 << INTR_IPU_DS0_FRAME_DONE)) {
		subdev = group->sub_ctx[GROUP_ID_DS0];
		if (subdev)
			ipu_frame_done(subdev);
	}

	if (status & (1 << INTR_IPU_DS1_FRAME_DONE)) {
		subdev = group->sub_ctx[GROUP_ID_DS1];
		if (subdev)
			ipu_frame_done(subdev);
	}

	if (status & (1 << INTR_IPU_DS2_FRAME_DONE)
	    && test_bit(IPU_DS2_DMA_OUTPUT, &ipu->state)) {
		subdev = group->sub_ctx[GROUP_ID_DS2];
		if (subdev)
			ipu_frame_done(subdev);
	}

	if (status & (1 << INTR_IPU_DS3_FRAME_DONE)) {
		subdev = group->sub_ctx[GROUP_ID_DS3];
		if (subdev)
			ipu_frame_done(subdev);
	}

	if (status & (1 << INTR_IPU_DS4_FRAME_DONE)) {
		subdev = group->sub_ctx[GROUP_ID_DS4];
			ipu_frame_done(subdev);
	}

	if (status & (1 << INTR_IPU_FRAME_START)) {
		ipu->statistic.fs[instance]++;
		ipu->statistic.tal_fs++;
		ipu->statistic.grp_tsk_left[instance]
			= atomic_read(&group->rcount);
		ipu->statistic.tal_frm_work = atomic_read(&ipu->backup_fcount);
		atomic_inc(&ipu->sensor_fcount);
		if (test_bit(IPU_OTF_INPUT, &ipu->state)
				&& group->leader) {
			if (unlikely(list_empty(&gtask->hw_resource.wait_list))) {
				vio_err("[S%d]GP%d(res %d, rcnt %d, bcnt %d, scnt %d)\n",
					group->instance,
					gtask->id,
					gtask->hw_resource.count,
					atomic_read(&group->rcount),
					atomic_read(&ipu->backup_fcount),
					atomic_read(&ipu->sensor_fcount));
				ipu->statistic.fs_lack_task[instance]++;
			} else {
				up(&gtask->hw_resource);
			}
		}

		group->frameid.frame_id++;
		if (group && group->get_timestamps) {
			vio_get_frame_id(group);
			vio_dbg("[S%d]IPU frame count = %d\n",
					instance, group->frameid.frame_id);
		}
		vio_set_stat_info(group->instance, IPU_FS, group->frameid.frame_id);
	}

	if (ipu->frame_drop_count > 20) {
		//ipu_hw_dump(ipu->base_reg);
		vio_err("[S%d]too many Frame drop\n", instance);
		ipu->frame_drop_count = 0;
	}

	if (drop_flag) {
		if (group->output_flag == 1)
			vio_group_done(group);
	}

	if (err_occured == 1)
		ipu_diag_report(err_occured, err_status);
	else
		ipu_diag_report(err_occured, status);
	return IRQ_HANDLED;
}

static struct file_operations x3_ipu_fops = {
	.owner = THIS_MODULE,
	.open = x3_ipu_open,
	.write = x3_ipu_write,
	.read = x3_ipu_read,
	.poll = x3_ipu_poll,
	.release = x3_ipu_close,
	.unlocked_ioctl = x3_ipu_ioctl,
	.compat_ioctl = x3_ipu_ioctl,
	.mmap = x3_ipu_mmap,
};

static int x3_ipu_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_ipu_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_ipu_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_ipu_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x3_ipu_pm_ops = {
	.suspend = x3_ipu_suspend,
	.resume = x3_ipu_resume,
	.runtime_suspend = x3_ipu_runtime_suspend,
	.runtime_resume = x3_ipu_runtime_resume,
};

int ipu_subdev_init(struct ipu_subdev *subdev)
{
	int ret = 0;

	spin_lock_init(&subdev->slock);
	atomic_set(&subdev->refcount, 0);

	return ret;
}

int x3_ipu_subdev_init(struct x3_ipu_dev *ipu)
{
	int i = 0, j = 0;
	int ret = 0;
	struct ipu_subdev *subdev;

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		for (j = 0; j < MAX_DEVICE; j++) {
			subdev = &ipu->subdev[i][j];
			ret = ipu_subdev_init(subdev);
		}
	}

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		for (j = 0; j < VIO_MP_MAX_FRAMES; j++) {
			frame_work_init(&ipu->vwork[i][j].work);
		}
	}

	return ret;
}

int x3_ipu_device_node_init(struct x3_ipu_dev *ipu)
{
	int ret = 0;
	int i = 0;
	char name[32];
	struct device *dev = NULL;

	ret = alloc_chrdev_region(&ipu->devno, 0, MAX_DEVICE, "x3_ipu");
	if (ret < 0) {
		vio_err("Error %d while alloc chrdev ipu", ret);
		goto err_req_cdev;
	}

	cdev_init(&ipu->cdev, &x3_ipu_fops);
	ipu->cdev.owner = THIS_MODULE;
	ret = cdev_add(&ipu->cdev, ipu->devno, MAX_DEVICE);
	if (ret) {
		vio_err("Error %d while adding x2 ipu cdev", ret);
		goto err;
	}

	if (vps_class)
		ipu->class = vps_class;
	else
		ipu->class = class_create(THIS_MODULE, X3_IPU_NAME);

	dev = device_create(ipu->class, NULL, MKDEV(MAJOR(ipu->devno), 0), NULL,
			  "ipu_s0");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		vio_err("ipu device create fail\n");
		goto err;
	}

	dev = device_create(ipu->class, NULL, MKDEV(MAJOR(ipu->devno), 1), NULL,
			  "ipu_us");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		vio_err("ipu device create fail\n");
		goto err;
	}
	for (i = 0; i < 6; i++) {
		snprintf(name, 32, "ipu_ds%d", i);
		dev = device_create(ipu->class, NULL, MKDEV(MAJOR(ipu->devno), i + 2),
			      NULL, name);
		if (IS_ERR(dev)) {
			ret = -EINVAL;
			vio_err("ipu device create fail\n");
			goto err;
		}
	}

	return ret;
err:
	class_destroy(ipu->class);
err_req_cdev:
	unregister_chrdev_region(ipu->devno, MAX_DEVICE);
	return ret;
}

static ssize_t ipu_reg_dump(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_ipu_dev *ipu;

	ipu = dev_get_drvdata(dev);

	if (atomic_read(&ipu->rsccount) == 0)
		ips_set_clk_ctrl(IPU0_CLOCK_GATE, true);

	ipu_hw_dump(ipu->base_reg);

	if (atomic_read(&ipu->rsccount) == 0)
		ips_set_clk_ctrl(IPU0_CLOCK_GATE, false);

	return 0;
}

static DEVICE_ATTR(regdump, 0444, ipu_reg_dump, NULL);

static ssize_t ipu_stat_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_ipu_dev *ipu;
	u32 offset = 0;
	int instance, instance_start;
	int i = 0;
	ssize_t len = 0;
	struct user_statistic *stats;

	ipu = dev_get_drvdata(dev);
	if (!memcmp(attr->attr.name, "err_status01", sizeof("err_status01")))
		instance_start = 0;
	else if (!memcmp(attr->attr.name, "err_status23", sizeof("err_status23")))
		instance_start = 2;
	else if (!memcmp(attr->attr.name, "err_status45", sizeof("err_status45")))
		instance_start = 4;
	else if (!memcmp(attr->attr.name, "err_status67", sizeof("err_status67")))
		instance_start = 6;
	else
		instance_start = 0;

	for(instance = instance_start; instance < instance_start + 2; instance++) {
		if (!ipu->statistic.enable[instance])
			continue;
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"*******S%d info:******\n",
			instance);
		offset += len;
		for (i = 0; i < 7; i++) {
			stats = &ipu->statistic.user_stats[instance][i];
			len = snprintf(&buf[offset], PAGE_SIZE - offset,
				"ch%d(%s) USER: normal %d, sel tout %d, drop %d, "
				"dq fail %d, sel err%d\n",
				i, ipu_node_name[i],
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
				"q_normal %d "
				"hard_frame_drop %d\n",
				i, ipu_node_name[i],
				ipu->statistic.fe_normal[instance][i],
				ipu->statistic.fe_lack_buf[instance][i],
				ipu->statistic.pollin_comp[instance][i],
				ipu->statistic.pollin_fe[instance][i],
				ipu->statistic.pollerr[instance][i],
				ipu->statistic.dq_normal[instance][i],
				ipu->statistic.dq_err[instance][i],
				ipu->statistic.q_normal[instance][i],
				ipu->statistic.hard_frame_drop[instance][i]);
			offset += len;
		}
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"DRV: fs %d, grp_tsk_left %d, fs_lack_task %d\n",
			ipu->statistic.fs[instance],
			ipu->statistic.grp_tsk_left[instance],
			ipu->statistic.fs_lack_task[instance]);
		offset += len;
	}

	len = snprintf(&buf[offset], PAGE_SIZE - offset,
		"DRV: tatal_fs %d, tatal_frm_work %d\n",
		ipu->statistic.tal_fs,
		ipu->statistic.tal_frm_work);
	offset += len;

	return offset;
}
static ssize_t ipu_stat_store(struct device *dev,
					struct device_attribute *attr,
					const char *page, size_t len)
{
	struct x3_ipu_dev *ipu;
	int instance, instance_start;

	ipu = dev_get_drvdata(dev);
	if (!memcmp(attr->attr.name, "err_status01", sizeof("err_status01")))
		instance_start = 0;
	else if (!memcmp(attr->attr.name, "err_status23", sizeof("err_status23")))
		instance_start = 2;
	else if (!memcmp(attr->attr.name, "err_status45", sizeof("err_status45")))
		instance_start = 4;
	else if (!memcmp(attr->attr.name, "err_status67", sizeof("err_status67")))
		instance_start = 6;
	else
		instance_start = 0;

	for(instance = instance_start; instance < instance_start + 2; instance++) {
		ipu->statistic.enable[instance] = 0;
		ipu->statistic.fs_lack_task[instance] = 0;
		ipu->statistic.fs[instance] = 0;
		ipu->statistic.grp_tsk_left[instance] = 0;
		memset(ipu->statistic.fe_normal[instance], 0,
			sizeof(ipu->statistic.fe_normal[instance]));
		memset(ipu->statistic.fe_lack_buf[instance], 0,
			sizeof(ipu->statistic.fe_lack_buf[instance]));
		memset(ipu->statistic.hard_frame_drop[instance], 0,
			sizeof(ipu->statistic.hard_frame_drop[instance]));
		memset(ipu->statistic.dq_normal[instance], 0,
			sizeof(ipu->statistic.dq_normal[instance]));
		memset(ipu->statistic.dq_err[instance], 0,
			sizeof(ipu->statistic.dq_err[instance]));
		memset(ipu->statistic.pollin_fe[instance], 0,
			sizeof(ipu->statistic.pollin_fe[instance]));
		memset(ipu->statistic.pollin_comp[instance], 0,
			sizeof(ipu->statistic.pollin_comp[instance]));
		memset(ipu->statistic.pollerr[instance], 0,
			sizeof(ipu->statistic.pollerr[instance]));
		memset(ipu->statistic.q_normal[instance], 0,
			sizeof(ipu->statistic.q_normal[instance]));
		memset(ipu->statistic.user_stats[instance], 0,
			sizeof(ipu->statistic.user_stats[instance]));
		if (instance == 0) {
			ipu->statistic.tal_fs = 0;
			ipu->statistic.tal_frm_work = 0;
		}
	}

	return len;
}
static DEVICE_ATTR(err_status01, S_IRUGO|S_IWUSR, ipu_stat_show,
	ipu_stat_store);
static DEVICE_ATTR(err_status23, S_IRUGO|S_IWUSR, ipu_stat_show,
	ipu_stat_store);
static DEVICE_ATTR(err_status45, S_IRUGO|S_IWUSR, ipu_stat_show,
	ipu_stat_store);
static DEVICE_ATTR(err_status67, S_IRUGO|S_IWUSR, ipu_stat_show,
	ipu_stat_store);

static ssize_t enabled_pipeline_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	struct x3_ipu_dev *ipu;
	u32 offset = 0, len = 0;
	int enabled_pipe_num = 0;
	int i = 0;
	ipu = dev_get_drvdata(dev);

	len = snprintf(buf, PAGE_SIZE - offset, "enable pipe index:");
	offset += len;
	for (i = 0; i < VIO_MAX_STREAM; i++) {
		if (ipu->statistic.enable[i]) {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "%d,", 1);
			enabled_pipe_num++;
		} else {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "%d,", 0);
		}
		offset += len;
	}

	len = snprintf(buf+offset, PAGE_SIZE - offset, "\n%d pipeline(s) enabled\n", enabled_pipe_num);
	offset += len;
	return offset;
}

static DEVICE_ATTR_RO(enabled_pipeline);

static ssize_t get_pipeline_info(int pipeid, struct device *dev,
					struct device_attribute *attr, char* buf)
{
	struct x3_ipu_dev *ipu;
	int i = 0;
	u32 offset = 0, len = 0;
	ipu_ds_info_t *ds_config;
	ipu_us_info_t *us_config;
	ipu = dev_get_drvdata(dev);

	if (ipu->statistic.enable[0] == 0) {
		len = snprintf(buf+offset, PAGE_SIZE - offset, "pipeline %d is disabled\n", pipeid);
		offset += len;
	} else {
		len = snprintf(buf+offset, PAGE_SIZE - offset, "pipeline %d ipu config:\n", pipeid);
		offset += len;

		ipu_input_type_e input_type = ipu->subdev[pipeid][0].ipu_cfg.ctrl_info.source_sel;
		if (input_type == 0) {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "input mode: %d, %s\n", input_type, "sif online to ipu");
		} else if (input_type == 1) {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "input mode: %d, %s\n", input_type, "isp online to ipu");
		} else if (input_type == 3) {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "input mode: %d, %s\n", input_type, "ddr to ipu");
		} else {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "input mode: %d, %s\n", input_type, "wrong");
		}
		offset += len;
		len = snprintf(buf+offset, PAGE_SIZE - offset, "channel config:\n");
		offset += len;
		// us config
		us_config = (ipu_us_info_t *)&ipu->subdev[pipeid][1].scale_cfg;
		len = snprintf(buf+offset, PAGE_SIZE - offset, "\tus channel: roi_en %d, wxh:%dx%d, upscale_en:%d, wxh:%dx%d\n",
				us_config->us_roi_en, us_config->us_roi_info.width, us_config->us_roi_info.height,
				us_config->us_sc_en, us_config->us_sc_info.tgt_width, us_config->us_sc_info.tgt_height
			);
		offset += len;

		for (i = 0; i < 5; i++) {
			ds_config = (ipu_ds_info_t *)&ipu->subdev[pipeid][2+i].scale_cfg;
			len = snprintf(buf+offset, PAGE_SIZE - offset, "\tds%d channel: roi_en %d, wxh:%dx%d, downscale_en:%d, wxh:%dx%d\n",
					i, ds_config->ds_roi_en, ds_config->ds_roi_info.width, ds_config->ds_roi_info.height,
					ds_config->ds_sc_en, ds_config->ds_sc_info.tgt_width, ds_config->ds_sc_info.tgt_height
				);
			offset += len;
		}
	}
	return offset;

}

static ssize_t pipeline0_info_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	return get_pipeline_info(0, dev, attr, buf);
}

static ssize_t pipeline1_info_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	return get_pipeline_info(1, dev, attr, buf);
}

static ssize_t pipeline2_info_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	return get_pipeline_info(2, dev, attr, buf);
}

static ssize_t pipeline3_info_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	return get_pipeline_info(3, dev, attr, buf);
}

static ssize_t pipeline4_info_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	return get_pipeline_info(4, dev, attr, buf);
}

static ssize_t pipeline5_info_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	return get_pipeline_info(5, dev, attr, buf);
}

static ssize_t pipeline6_info_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	return get_pipeline_info(6, dev, attr, buf);
}

static ssize_t pipeline7_info_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	return get_pipeline_info(7, dev, attr, buf);
}

static DEVICE_ATTR_RO(pipeline0_info);
static DEVICE_ATTR_RO(pipeline1_info);
static DEVICE_ATTR_RO(pipeline2_info);
static DEVICE_ATTR_RO(pipeline3_info);
static DEVICE_ATTR_RO(pipeline4_info);
static DEVICE_ATTR_RO(pipeline5_info);
static DEVICE_ATTR_RO(pipeline6_info);
static DEVICE_ATTR_RO(pipeline7_info);


static struct attribute *ipu_info_attrs[] = {
	&dev_attr_enabled_pipeline.attr,
	&dev_attr_pipeline0_info.attr,
	&dev_attr_pipeline1_info.attr,
	&dev_attr_pipeline2_info.attr,
	&dev_attr_pipeline3_info.attr,
	&dev_attr_pipeline4_info.attr,
	&dev_attr_pipeline5_info.attr,
	&dev_attr_pipeline6_info.attr,
	&dev_attr_pipeline7_info.attr,
	NULL
};

static struct attribute_group ipu_info_group = {
	.attrs	= ipu_info_attrs,
	.name	= "info",
};

static int x3_ipu_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x3_ipu_dev *ipu;
	struct device *dev = NULL;
	struct resource *mem_res;

	vio_err("sizeof vio_frame:%d", sizeof(struct vio_frame));

	ipu = kzalloc(sizeof(struct x3_ipu_dev), GFP_KERNEL);
	if (!ipu) {
		vio_err("ipu is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		vio_err("Failed to get io memory region(%p)", mem_res);
		ret = -EBUSY;
		goto err_get_resource;
	}

	ipu->regs_start = mem_res->start;
	ipu->regs_end = mem_res->end;
	ipu->base_reg = devm_ioremap_nocache(&pdev->dev,
			mem_res->start, resource_size(mem_res));
	if (!ipu->base_reg) {
		vio_err("Failed to remap io region(%p)", ipu->base_reg);
		ret = -ENOMEM;
		goto err_get_resource;
	}

	/* Get IRQ SPI number */
	ipu->irq = platform_get_irq(pdev, 0);
	if (ipu->irq < 0) {
		vio_err("Failed to get ipu->irq(%d)", ipu->irq);
		ret = -EBUSY;
		goto err_get_irq;
	}

	ret = request_irq(ipu->irq, ipu_isr, IRQF_TRIGGER_HIGH, "ipu", ipu);
	if (ret) {
		vio_err("request_irq(IRQ_IPU %d) is fail(%d)", ipu->irq, ret);
		goto err_get_irq;
	}

	x3_ipu_device_node_init(ipu);

	dev = &pdev->dev;
	ret = device_create_file(dev, &dev_attr_regdump);
	if (ret < 0) {
		vio_err("create regdump failed (%d)\n", ret);
		goto p_err;
	}

	ret = device_create_file(dev, &dev_attr_err_status01);
	if (ret < 0) {
		vio_err("create err_status failed (%d)\n", ret);
		goto p_err;
	}
	ret = device_create_file(dev, &dev_attr_err_status23);
	if (ret < 0) {
		vio_err("create err_status failed (%d)\n", ret);
		goto p_err;
	}
	ret = device_create_file(dev, &dev_attr_err_status45);
	if (ret < 0) {
		vio_err("create err_status failed (%d)\n", ret);
		goto p_err;
	}
	ret = device_create_file(dev, &dev_attr_err_status67);
	if (ret < 0) {
		vio_err("create err_status failed (%d)\n", ret);
		goto p_err;
	}

	// create sysfs node for ipu info
	ret = sysfs_create_group(&dev->kobj, &ipu_info_group);
	if (ret) {
		vio_err("create ipu info group fail");
		goto p_err;
	}

	platform_set_drvdata(pdev, ipu);

	sema_init(&ipu->gtask.hw_resource, 1);
	atomic_set(&ipu->gtask.refcount, 0);
	atomic_set(&ipu->rsccount, 0);
	atomic_set(&ipu->open_cnt, 0);

	x3_ipu_subdev_init(ipu);
	vio_group_init_mp(GROUP_ID_IPU);
	if (diag_register(ModuleDiag_VIO, EventIdVioIpuErr,
					4, 380, 8000, NULL) < 0)
		pr_err("ipu dual diag register fail\n");
	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

	return 0;

err_get_irq:
	iounmap(ipu->base_reg);

err_get_resource:
	kfree(ipu);
p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;

}

static int x3_ipu_remove(struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	struct x3_ipu_dev *ipu;

	BUG_ON(!pdev);

	ipu = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_regdump);
	device_remove_file(&pdev->dev, &dev_attr_err_status01);
	device_remove_file(&pdev->dev, &dev_attr_err_status23);
	device_remove_file(&pdev->dev, &dev_attr_err_status45);
	device_remove_file(&pdev->dev, &dev_attr_err_status67);
	sysfs_remove_group(&pdev->dev.kobj, &ipu_info_group);
	

	free_irq(ipu->irq, ipu);

	for(i = 0; i < MAX_DEVICE; i++)
		device_destroy(ipu->class, MKDEV(MAJOR(ipu->devno), i));

	if (!vps_class)
		class_destroy(ipu->class);
	cdev_del(&ipu->cdev);
	unregister_chrdev_region(ipu->devno, MAX_DEVICE);
	kfree(ipu);

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x3_ipu_match[] = {
	{
	 .compatible = "hobot,x3-ipu",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, x3_ipu_match);

static struct platform_driver x3_ipu_driver = {
	.probe = x3_ipu_probe,
	.remove = x3_ipu_remove,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_ipu_pm_ops,
		   .of_match_table = x3_ipu_match,
		   }
};

#else
static struct platform_device_id x3_ipu_driver_ids[] = {
	{
	 .name = MODULE_NAME,
	 .driver_data = 0,
	 },
	{},
};

MODULE_DEVICE_TABLE(platform, x3_ipu_driver_ids);

static struct platform_driver x3_ipu_driver = {
	.probe = x3_ipu_probe,
	.remove = x3_ipu_remove,
	.id_table = x3_ipu_driver_ids,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_ipu_pm_ops,
		   }
};
#endif

static int __init x3_ipu_init(void)
{
	int ret = platform_driver_register(&x3_ipu_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

late_initcall(x3_ipu_init);

static void __exit x3_ipu_exit(void)
{
	platform_driver_unregister(&x3_ipu_driver);
}

module_exit(x3_ipu_exit);
MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X3 IPU driver");
MODULE_LICENSE("GPL v2");
