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

#include "hobot_dev_ipu.h"
#include "hobot_dev_ips.h"

#include "ipu_hw_api.h"

#define MODULE_NAME "X2A IPU"

#if CONFIG_QEMU_TEST
static struct timer_list tm[4];
static int g_test_bit = 0;
static int g_int_test = -1;

static int timer_init(struct x2a_ipu_dev *ipu, int index);
#endif

void ipu_hw_set_osd_cfg(struct ipu_video_ctx *ipu_ctx);
extern struct class *vps_class;

static u32 color[MAX_OSD_COLOR_NUM] = {
	0x601010, 0x606010, 0x60B010, 0x60F010, 0x60F060, 0x60F0B0, 0x60F0F0,
    0x6010F0, 0x6060F0, 0x60B0F0, 0x601060, 0x6010B0, 0x80FFFF, 0xFFFFFF,
    0x000000
};

static void ipu_isr_bh(struct work_struct *data);
int ipu_put_client(struct ipu_sub_mp *sub_mp,
		struct ipu_video_ctx *ipu_sub_ctx);
struct ipu_video_ctx *ipu_get_client(struct ipu_sub_mp *sub_mp);

static int x2a_ipu_open(struct inode *inode, struct file *file)
{
	struct ipu_video_ctx *ipu_ctx;
	struct x2a_ipu_dev *ipu;
	int ret = 0;
	int minor;

	minor = MINOR(inode->i_rdev);

	ipu = container_of(inode->i_cdev, struct x2a_ipu_dev, cdev);
	ipu_ctx = kzalloc(sizeof(struct ipu_video_ctx), GFP_KERNEL);
	if (ipu_ctx == NULL) {
		vio_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	ipu_ctx->id = minor;

	init_waitqueue_head(&ipu_ctx->done_wq);

	ipu_ctx->ipu_dev = ipu;
	file->private_data = ipu_ctx;

	ipu_ctx->state = BIT(VIO_VIDEO_OPEN);

	atomic_inc(&ipu->open_cnt);
p_err:
	return 0;
}

static int x2a_ipu_close(struct inode *inode, struct file *file)
{
	struct ipu_video_ctx *ipu_ctx;
	struct x2a_ipu_dev *ipu;
	struct vio_group *group;
	u32 index;
	u32 cnt;

	ipu_ctx = file->private_data;
	group = ipu_ctx->group;
	ipu = ipu_ctx->ipu_dev;

	if (group && ipu_ctx->leader)
		clear_bit(VIO_GROUP_LEADER, &group->state);

	if (group)
		clear_bit(VIO_GROUP_INIT, &group->state);

	if (group->gtask)
		vio_group_task_stop(group->gtask);

	index = ipu_ctx->frm_fst_ind;
	cnt = ipu_ctx->frm_num;
	frame_manager_close_mp(ipu_ctx->framemgr, index, cnt);

	ipu_ctx->state = BIT(VIO_VIDEO_CLOSE);

	if (atomic_dec_return(&ipu->open_cnt) == 0) {
		clear_bit(IPU_OTF_INPUT, &ipu->state);
		clear_bit(IPU_DMA_INPUT, &ipu->state);
		clear_bit(IPU_DS2_DMA_OUTPUT, &ipu->state);
	}

	kfree(ipu_ctx);

	return 0;
}

static ssize_t x2a_ipu_write(struct file *file, const char __user * buf,
				size_t count, loff_t * ppos)
{
	return 0;
}

static ssize_t x2a_ipu_read(struct file *file, char __user * buf, size_t size,
				loff_t * ppos)
{
	return 0;
}

static u32 x2a_ipu_poll(struct file *file, struct poll_table_struct *wait)
{
	int ret = 0;
	struct ipu_video_ctx *ipu_ctx;
	unsigned long flags;
	struct list_head *done_list;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct ipu_sub_mp *sub_mp;

	ipu_ctx = file->private_data;
	ipu_ctx->ispoll = 1;
	framemgr = ipu_ctx->framemgr;
	sub_mp = ipu_ctx->sub_mp;

	poll_wait(file, &ipu_ctx->done_wq, wait);
	if(ipu_ctx->event == VIO_FRAME_DONE)
		return POLLIN;
	else if(ipu_ctx->event == VIO_FRAME_NDONE)
		return POLLERR;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	done_list = &framemgr->queued_list[FS_COMPLETE];
	if (!list_empty(done_list)) {
		frame = peek_frame(framemgr, FS_COMPLETE);
		if (frame) {
			ret = POLLIN;
			vio_dbg("poll complete fid%d,bufinx%d ",
				frame->frameinfo.frame_id,
				frame->frameinfo.bufferindex);
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			return	ret;
		}
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	/* add to client list */
	spin_lock(&sub_mp->slock);
	if (!ipu_ctx->in_list) {
		ret = ipu_put_client(sub_mp, ipu_ctx);
		if (ret < 0) {
			spin_unlock(&sub_mp->slock);
			vio_err("ipu put client error.");
			return ret;
		}
	}
	spin_unlock(&sub_mp->slock);
	return ret;
}

void ipu_frame_work(struct vio_group *group)
{
	struct ipu_video_ctx *back_ctx;
	struct ipu_sub_mp *sub_mp;
	struct x2a_ipu_dev *ipu;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	int i = 0;
	u32 instance = 0;
	u32 rdy = 0;
	u8 shadow_index = 0;

	instance = group->instance;
	sub_mp = group->sub_ctx[0];
	if (!sub_mp) {
		vio_err("%s group%d sub mp 0 err.\n",
			__func__, instance);
		return;
	}
	ipu = sub_mp->ipu_dev;
	shadow_index = instance % MAX_SHADOW_NUM;
	vio_info("%s start\n", __func__);

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy & ~(1 << 4);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	ipu_set_shd_select(ipu->base_reg, shadow_index);

	atomic_set(&ipu->instance, instance);
	for (i = MAX_DEVICE - 1; i >= 0; i--) {
		sub_mp = group->sub_ctx[i];
		if (!sub_mp)
			continue;
		back_ctx = sub_mp->dev[0];
		if (back_ctx != NULL) {
			framemgr = back_ctx->framemgr;
			framemgr_e_barrier_irqs(framemgr, 0, flags);
			frame = peek_frame(framemgr, FS_REQUEST);
			if (frame) {
				switch (i) {
				case GROUP_ID_SRC:
					ipu_set_rdma_addr(ipu->base_reg,
							  frame->frameinfo.addr[0],
							  frame->frameinfo.addr[1]);
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

				ipu_hw_set_osd_cfg(back_ctx);
				trans_frame(framemgr, frame, FS_PROCESS);
			}
			framemgr_x_barrier_irqr(framemgr, 0, flags);
		}
	}
	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy | (1 << 4);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	if (test_bit(IPU_DMA_INPUT, &ipu->state))
		ipu_set_rdma_start(ipu->base_reg);
	vio_info("%s done\n", __func__);
}

void ipu_set_group_leader(struct vio_group *group, enum group_id id,
					u32 proc_id)
{
	struct ipu_video_ctx *ipu_ctx, *ctx_mp;
	struct ipu_sub_mp *sub_mp;
	u32 i;

	if (id >= GROUP_ID_MAX || id < GROUP_ID_SRC)
		vio_err("%s wrong id", __func__);
	else if (proc_id >= VIO_MAX_SUB_PROCESS)
		vio_err("%s wrong process id", __func__);
	else {
		sub_mp = group->sub_ctx[id];
		if (!sub_mp) {
			vio_err("%s wrong sub_mp", __func__);
			return;
		}
		if (!test_bit(VIO_GROUP_LEADER, &group->state)) {
			set_bit(VIO_GROUP_LEADER, &group->state);

			ipu_ctx = sub_mp->dev[proc_id];
			if (!ipu_ctx) {
				vio_err("%s wrong ipu_ctx", __func__);
				return;
			}
			ipu_ctx->leader = true;
			vio_info("%s:%d\n", __func__, id);
		} else {
			for (i = 0; i < sub_mp->proc_count; i++) {
				ctx_mp = sub_mp->dev[i];
				if (ctx_mp->leader == true) {
					ipu_ctx->leader = true;
					vio_info("%s:%d\n", __func__, id);
					break;
				}
			}
		}
	}
}

void ipu_clear_group_leader(struct vio_group *group)
{
	int i = 0;
	int j = 0;
	struct ipu_video_ctx *ipu_ctx;
	struct ipu_sub_mp *sub_mp;

	for (i = 0; i < GROUP_ID_MAX; i++) {
		sub_mp = group->sub_ctx[i];
		if (!sub_mp)
			continue;
		for (j = 0; j < sub_mp->proc_count; j++ ) {
			ipu_ctx = sub_mp->dev[j];
			if (ipu_ctx)
				ipu_ctx->leader = false;
		}
	}

	clear_bit(VIO_GROUP_LEADER, &group->state);
}

int ipu_update_osd_addr(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	u32 id = 0;
	u32 *osd_buf;

	id = ipu_ctx->id;
	if (id < GROUP_ID_US || id > GROUP_ID_DS1) {
		vio_err("%s wrong ctx id %d\n", __func__, id);
		return -EFAULT;
	}
	osd_buf = ipu_ctx->osd_cfg.osd_buf;
	ret = copy_from_user((char *) osd_buf, (u32 __user *) arg,
			   MAX_OSD_LAYER * sizeof(u32));
	if (ret)
		return -EFAULT;

	ipu_ctx->osd_cfg.osd_buf_update = 1;

	return ret;
}

int ipu_update_osd_roi(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	u32 id = 0;
	osd_box_t *osd_box;

	id = ipu_ctx->id;
	if (id < GROUP_ID_US || id > GROUP_ID_DS1) {
		vio_err("%s wrong ctx id %d\n", __func__, id);
		return -EFAULT;
	}
	osd_box = ipu_ctx->osd_cfg.osd_box;
	ret = copy_from_user((char *) osd_box, (u32 __user *) arg,
			  MAX_OSD_NUM * sizeof(osd_box_t));
	if (ret)
		return -EFAULT;

	ipu_ctx->osd_cfg.osd_box_update = 1;

	return ret;
}

void ipu_hw_set_osd_cfg(struct ipu_video_ctx *ipu_ctx)
{
	u32 shadow_index = 0;
	u32 osd_index = 0;
	u32 id = 0;
	u32 i = 0;
	u8 sta_en = 0;
	u16 start_x, start_y, width, height;
	u32 __iomem *base_reg;
	osd_box_t *osd_box;
	osd_sta_box_t *sta_box;
	u32 *osd_buf;
	osd_color_map_t *color_map;
	struct ipu_osd_cfg *osd_cfg;

	id = ipu_ctx->id;
	if (id < GROUP_ID_US || id > GROUP_ID_DS1) {
		return;
	}

	if(id == GROUP_ID_US)
		osd_index = 2;
	else
		osd_index = id - GROUP_ID_DS0;

	shadow_index = ipu_ctx->group->instance % MAX_SHADOW_NUM;
	base_reg = ipu_ctx->ipu_dev->base_reg;

	osd_cfg = &ipu_ctx->osd_cfg;
	osd_box = osd_cfg->osd_box;
	osd_buf = osd_cfg->osd_buf;
	for (i = 0; i < MAX_OSD_LAYER; i++) {
		if(osd_cfg->osd_box_update){
			if(osd_box[i].osd_en){
				start_x = osd_box[i].start_x;
				start_y = osd_box[i].start_y;
				width = osd_box[i].width;
				height = osd_box[i].height;
				ipu_set_osd_roi(base_reg, shadow_index, osd_index, i, start_x,
						start_y, width, height);
				ipu_set_osd_overlay_mode(base_reg, shadow_index, osd_index, i,
						osd_box->overlay_mode);

				ipu_set_osd_enable(base_reg, shadow_index, osd_index, i, true);

				osd_cfg->osd_box_update = 0;
			}else
				ipu_set_osd_enable(base_reg, shadow_index, osd_index, i, false);
			}
			if(osd_cfg->osd_buf_update){
				ipu_set_osd_addr(base_reg, shadow_index, osd_index,
						i, osd_buf[i]);
				osd_cfg->osd_buf_update = 0;
		}

	}

	color_map = &ipu_ctx->osd_cfg.color_map;
	if(color_map->color_map_update){
		for (i = 0; i < MAX_OSD_COLOR_NUM; i++) {
			ipu_set_osd_color(base_reg, i, color_map->color_map[i]);
		}
		color_map->color_map_update = 0;
	}

	sta_box = osd_cfg->osd_sta;
	for (i = 0; i < MAX_STA_NUM; i++) {
		sta_en = sta_box[i].sta_en;
		if (sta_en && osd_cfg->osd_sta_update) {
			start_x = sta_box[i].start_x;
			start_y = sta_box[i].start_y;
			width = sta_box[i].width;
			height = sta_box[i].height;
			ipu_set_osd_sta_enable(base_reg, shadow_index, osd_index, i,
							sta_en);
			ipu_set_osd_sta_roi(base_reg, shadow_index, osd_index, i,
							start_x, start_y, width, height);
			osd_cfg->osd_sta_update = 0;
		}
	}

	if(osd_cfg->osd_sta_level_update){
		ipu_set_osd_sta_level(base_reg, shadow_index, i,
			      osd_cfg->osd_sta_level[osd_index]);
		osd_cfg->osd_sta_level_update = 0;
	}

}

int ipu_update_osd_sta_roi(struct ipu_video_ctx *ipu_ctx, unsigned long arg)	//osd_sta_box_t (*sta_box)[MAX_STA_NUM], u8 *osd_sta_level)
{
	int ret = 0;
	osd_sta_box_t *osd_sta;

	osd_sta = ipu_ctx->osd_cfg.osd_sta;
	ret = copy_from_user((char *) osd_sta, (u32 __user *) arg,
			   MAX_STA_NUM * sizeof(osd_sta_box_t));
	if (ret)
		return -EFAULT;

	ipu_ctx->osd_cfg.osd_sta_update = 1;

	return ret;
}

int ipu_update_osd_sta_level(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	u8 *osd_sta_level;

	osd_sta_level = ipu_ctx->osd_cfg.osd_sta_level;
	ret = copy_from_user((char *) osd_sta_level, (u32 __user *) arg,
			   MAX_OSD_STA_LEVEL_NUM * sizeof(u8));
	if (ret)
		return -EFAULT;

	ipu_ctx->osd_cfg.osd_sta_level_update = 1;

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

	id = ipu_ctx->id;
	if(id < GROUP_ID_US || id > GROUP_ID_DS1)
		return -EFAULT;

	if(id == GROUP_ID_US)
		osd_index = 2;
	else
		osd_index = id - GROUP_ID_DS0;


	base_reg = ipu_ctx->ipu_dev->base_reg;
	for (i = 0; i < MAX_STA_NUM; i++) {
		ipu_get_osd_sta_bin(base_reg, osd_index, i, sta_bin[i]);
	}

	ret = copy_to_user((void __user *) arg, (char *) sta_bin[0],
			 sizeof(u16) * MAX_STA_NUM * MAX_STA_BIN_NUM);
	if (ret)
		return -EFAULT;
	return ret;
}

int ipu_channel_wdma_disable(struct ipu_video_ctx *ipu_ctx)
{
	int ret = 0;
	u32 rdy = 0;
	u32 id = 0;
	u32 shadow_index = 0;
	u8 ds_ch = 0;
	struct vio_group *group;
	struct x2a_ipu_dev *ipu;

	id = ipu_ctx->id;
	group = ipu_ctx->group;
	ipu = ipu_ctx->ipu_dev;
	shadow_index = group->instance % MAX_SHADOW_NUM;
	ds_ch = id - GROUP_ID_DS0;

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy & ~(1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	if (id == GROUP_ID_US) {
		ipu_set_us_enable(ipu->base_reg, shadow_index, false);
		ipu_set_us_roi_enable(ipu->base_reg, shadow_index, false);
	} else {
		ipu_set_ds_enable(ipu->base_reg, shadow_index, ds_ch, false);
		ipu_set_ds_roi_enable(ipu->base_reg, shadow_index, ds_ch, false);
	}

	return ret;
}

int ipu_update_ds_ch_param(struct ipu_video_ctx *ipu_ctx, u8 ds_ch,
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
	struct x2a_ipu_dev *ipu;

	group = ipu_ctx->group;
	ipu = ipu_ctx->ipu_dev;
	shadow_index = group->instance % MAX_SHADOW_NUM;

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
	ipu_set_ds_enable(ipu->base_reg, shadow_index, ds_ch, ds_config->ds_sc_en);


	roi_x = ds_config->ds_roi_info.start_x;
	roi_y = ds_config->ds_roi_info.start_y;
	roi_width = ds_config->ds_roi_info.width;
	roi_height = ds_config->ds_roi_info.height;
	ipu_set_ds_roi_rect(ipu->base_reg, shadow_index, ds_ch, roi_x,
			       roi_y, roi_width, roi_height);
	ipu_set_ds_roi_enable(ipu->base_reg, shadow_index, ds_ch,
				   ds_config->ds_roi_en);

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy | (1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	//ipu_set_intr_mask(ipu->base_reg, 0);
	vio_info("[%d][ds%d]roi_x = %d, roi_y = %d, roi_width = %d, roi_height = %d\n",
		 shadow_index, ds_ch, roi_x, roi_y, roi_width, roi_height);
	return ret;
}

int ipu_update_ds_param(struct ipu_video_ctx *ipu_ctx,
			ipu_ds_info_t *ds_config)
{
	int ret = 0;
	u8 ds_ch = 0;
	struct vio_group *group;
	struct x2a_ipu_dev *ipu;

	ds_ch = ipu_ctx->id - GROUP_ID_DS0;
	ret = ipu_update_ds_ch_param(ipu_ctx, ds_ch, ds_config);

	ipu = ipu_ctx->ipu_dev;
	group = ipu_ctx->group;
	if (ds_config->ds_roi_en || ds_config->ds_sc_en) {
		ipu_set_group_leader(group, ipu_ctx->id, ipu_ctx->proc_id);
		set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);
		if (ipu_ctx->id == GROUP_ID_DS2) {
			set_bit(IPU_DS2_DMA_OUTPUT, &ipu->state);
		}
	}
	return ret;
}

int ipu_update_ds_param_mp(struct ipu_video_ctx *ipu_ctx,
		ipu_ds_info_t * ds_config)
{
	struct vio_group *group;
	u8 ds_ch = 0;
	struct x2a_ipu_dev *ipu;

	ipu = ipu_ctx->ipu_dev;
	group = ipu_ctx->group;
	ds_ch = ipu_ctx->id - GROUP_ID_DS0;
	if (ds_config->ds_roi_en || ds_config->ds_sc_en) {
		ipu_set_group_leader(group, ipu_ctx->id, ipu_ctx->proc_id);
		set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);
		if (ipu_ctx->id == GROUP_ID_DS2) {
			set_bit(IPU_DS2_DMA_OUTPUT, &ipu->state);
		}
	}
	return 0;
}

int ipu_update_us_param(struct ipu_video_ctx *ipu_ctx,
			ipu_us_info_t *us_config)
{
	u16 dst_width = 0, dst_height = 0;
	u16 dst_stepx = 0, dst_stepy = 0;
	u16 roi_x = 0, roi_y = 0, roi_width = 0, roi_height = 0;
	int ret = 0;
	u32 rdy = 0;
	u32 shadow_index = 0;
	struct vio_group *group;
	struct x2a_ipu_dev *ipu;

	group = ipu_ctx->group;
	shadow_index = group->instance % MAX_SHADOW_NUM;
	ipu = ipu_ctx->ipu_dev;

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy & ~(1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	dst_width = us_config->us_sc_info.tgt_width;
	dst_height = us_config->us_sc_info.tgt_height;
	ipu_set_us_target(ipu->base_reg, shadow_index, dst_stepx, dst_stepy,
			  dst_width, dst_height);

	dst_stepx = us_config->us_sc_info.step_x;
	dst_stepy = us_config->us_sc_info.step_y;
	ipu_set_us_wdma_stride(ipu->base_reg, shadow_index,
			       us_config->us_stride_y,
			       us_config->us_stride_uv);
	ipu_set_us_enable(ipu->base_reg, shadow_index, us_config->us_sc_en);

	roi_x = us_config->us_roi_info.start_x;
	roi_y = us_config->us_roi_info.start_y;
	roi_width = us_config->us_roi_info.width;
	roi_height = us_config->us_roi_info.height;
	ipu_set_us_roi_rect(ipu->base_reg, shadow_index, roi_x, roi_y,
			       roi_width, roi_height);
	ipu_set_us_roi_enable(ipu->base_reg, shadow_index, us_config->us_roi_en);

	if (us_config->us_roi_en || us_config->us_sc_en) {
		ipu_set_group_leader(group, ipu_ctx->id, ipu_ctx->proc_id);
		set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);
	}

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy | (1 << shadow_index);
	ipu_set_shd_rdy(ipu->base_reg, rdy);
	vio_info("roi_x = %d, roi_y = %d, roi_width = %d, roi_height = %d\n",
		 roi_x, roi_y, roi_width, roi_height);
	vio_info("step_x = %d, step_y = %d, tgt_width = %d, tgt_height = %d\n",
		 dst_stepx, dst_stepy, dst_width, dst_height);
	return ret;
}

int ipu_update_us_param_mp(struct ipu_video_ctx *ipu_ctx,
		ipu_us_info_t * us_config)
{
	struct vio_group *group;

	group = ipu_ctx->group;
	if (us_config->us_roi_en || us_config->us_sc_en) {
		ipu_set_group_leader(group, ipu_ctx->id, ipu_ctx->proc_id);
		set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);
	}

	return 0;
}

int ipu_update_common_param(struct ipu_video_ctx *ipu_ctx,
			ipu_cfg_t *ipu_cfg)
{
	int i = 0;
	int ret = 0;
	u8 Id_en = 0;
	u32 rdy = 0;
	u32 shadow_index = 0;
	struct vio_group *group;
	struct x2a_ipu_dev *ipu;
	ipu_src_ctrl_t *ipu_ctrl;
	u16 src_width, src_height, src_stride_uv, src_stride_y;

	group = ipu_ctx->group;
	ipu = ipu_ctx->ipu_dev;
	ipu_ctrl = &ipu_cfg->ctrl_info;
	shadow_index = group->instance % MAX_SHADOW_NUM;

	rdy = ipu_get_shd_rdy(ipu->base_reg);
	rdy = rdy & ~(1 << shadow_index) & ~(1 << 4);
	ipu_set_shd_rdy(ipu->base_reg, rdy);

	// SRC select
	ipu_src_select(ipu->base_reg, ipu_ctrl->source_sel);

	if (ipu_ctrl->source_sel != IPU_FROM_DDR_YUV420) {
		if (test_bit(IPU_DMA_INPUT, &ipu->state)){
			vio_err("IPU DMA input already,can't set otf input\n");
			return -EINVAL;
		}else{
			set_bit(IPU_OTF_INPUT, &ipu->state);
			set_bit(VIO_GROUP_OTF_INPUT, &group->state);
		}
	} else {
		if (test_bit(IPU_OTF_INPUT, &ipu->state)) {
			vio_err("IPU otf input already,can't set dma input\n");
			return -EINVAL;
		} else {
			set_bit(IPU_DMA_INPUT, &ipu->state);
			set_bit(VIO_GROUP_DMA_INPUT, &group->state);
		}
	}

	if (ipu_ctrl->source_sel == IPU_FROM_DDR_YUV420) {
		ipu_clear_group_leader(group);
		ipu_set_group_leader(group, GROUP_ID_SRC, ipu_ctx->proc_id);
	}
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

	ipu_update_ds_ch_param(ipu_ctx, 2, &ipu_cfg->ds_info[2]);

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

	return ret;
}

int ipu_update_common_param_mp(struct ipu_video_ctx *ipu_ctx,
			ipu_cfg_t *ipu_cfg)
{
	ipu_src_ctrl_t *ipu_ctrl;
	struct vio_group *group;

	group = ipu_ctx->group;
	ipu_ctrl = &ipu_cfg->ctrl_info;
	if (ipu_ctrl->source_sel == IPU_FROM_DDR_YUV420) {
		clear_bit(VIO_GROUP_LEADER, &group->state);
		ipu_set_group_leader(group, GROUP_ID_SRC, ipu_ctx->proc_id);
	}
	return 0;
}

int ipu_video_init(struct ipu_video_ctx *ipu_ctx, unsigned long arg)
{
	int ret = 0;
	u32 cfg = 0;
	ipu_us_info_t us_config;
	ipu_ds_info_t ds_config;
	ipu_cfg_t ipu_cfg;
	struct x2a_ipu_dev *ipu;
	struct vio_group *group;
	struct ipu_sub_mp *sub_mp;

	sub_mp = ipu_ctx->sub_mp;
	if (!sub_mp) {
		vio_err("[%s] sub_mp null", __func__);
		return -EFAULT;
	}

	ret = down_interruptible(&sub_mp->hw_init_sem);
	if (ret < 0) {
		vio_err("[%s] down_interruptible", __func__);
		return -EFAULT;
	}

	ipu = ipu_ctx->ipu_dev;

	if (!(ipu_ctx->state & (BIT(VIO_VIDEO_S_INPUT) | BIT(VIO_VIDEO_REBUFS)))) {
		vio_err("[%s] invalid INIT is requested(%lX)", __func__, ipu_ctx->state);
		return -EINVAL;
	}

	ips_set_clk_ctrl(IPU0_CLOCK_GATE, true);

	cfg = ips_get_bus_ctrl() | 0xd21e << 16;
	ips_set_bus_ctrl(cfg);

	if (ipu_ctx->id == GROUP_ID_SRC) {
		ret = copy_from_user((char *) &ipu_cfg, (u32 __user *) arg,
				   sizeof(ipu_cfg_t));
		if (ret)
			goto done;
		if (!test_bit(IPU_SUB_MP_USER_INIT, &sub_mp->state))
			ret = ipu_update_common_param(ipu_ctx, &ipu_cfg);
		else
			ret = ipu_update_common_param_mp(ipu_ctx, &ipu_cfg);
	} else if (ipu_ctx->id == GROUP_ID_US) {
		ret = copy_from_user((char *) &us_config, (u32 __user *) arg,
				   sizeof(ipu_us_info_t));
		if (ret)
			goto done;
		if (!test_bit(IPU_SUB_MP_USER_INIT, &sub_mp->state))
			ret = ipu_update_us_param(ipu_ctx, &us_config);
		else
			ret = ipu_update_us_param_mp(ipu_ctx, &us_config);
	} else if (ipu_ctx->id >= GROUP_ID_DS0) {
		ret = copy_from_user((char *) &ds_config, (u32 __user *) arg,
				   sizeof(ipu_ds_info_t));
		if (ret)
			goto done;
		if (!test_bit(IPU_SUB_MP_USER_INIT, &sub_mp->state))
			ret = ipu_update_ds_param(ipu_ctx, &ds_config);
		else
			ret = ipu_update_ds_param_mp(ipu_ctx, &ds_config);
	}

	group = ipu_ctx->group;
	vio_group_task_start(group->gtask);

	ipu_ctx->state = BIT(VIO_VIDEO_INIT);
	set_bit(IPU_SUB_MP_USER_INIT, &sub_mp->state);

done:
	up(&sub_mp->hw_init_sem);
	return ret;
}

int ipu_bind_chain_group(struct ipu_video_ctx *ipu_ctx, int instance)
{
	int ret = 0;
	int id = 0;
	struct vio_group *group;
	struct x2a_ipu_dev *ipu;
	struct vio_chain *chain;
	struct ipu_sub_mp *sub_mp_alloc, *sub_mp;
	u32 proc_id;
	u8 isfree;

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

	isfree = 0;
	sub_mp_alloc = NULL;
	spin_lock(&group->slock);
	if (!group->sub_ctx[id]) {
		spin_unlock(&group->slock);
		sub_mp_alloc = kzalloc(sizeof(struct ipu_sub_mp), GFP_KERNEL);
		if (!sub_mp_alloc) {
			vio_err("%s:%d alloc sub_mp err.", __func__, __LINE__);
			return -ENOMEM;
		}
		spin_lock(&group->slock);
	}
	if (!group->sub_ctx[id]) {
		group->sub_ctx[id] = sub_mp_alloc;
		sub_mp = sub_mp_alloc;
	} else {
		sub_mp = group->sub_ctx[id];
		if (sub_mp_alloc)
			isfree = 1;
	}
	if (!test_bit(IPU_SUB_MP_INIT, &sub_mp->state)) {
		spin_lock_init(&sub_mp->slock);
		INIT_LIST_HEAD(&sub_mp->client_list);
		set_bit(IPU_SUB_MP_INIT, &sub_mp->state);
	}

	proc_id = sub_mp->proc_count;
	if (proc_id >= VIO_MAX_SUB_PROCESS) {
		spin_unlock(&group->slock);
		vio_err("IPU bind too many files on ins%d minor%d",
			instance, id);
		return -EMFILE;
	}
	sub_mp->dev[proc_id] = ipu_ctx;
	sub_mp->proc_count++;
	sub_mp->group = group;
	sub_mp->ipu_dev = ipu;
	sema_init(&sub_mp->hw_init_sem, 1);
	ipu_ctx->proc_id = proc_id;
	ipu_ctx->sub_mp = sub_mp;
	ipu_ctx->framemgr = &sub_mp->framemgr;
	if (!group->sub_ctx[id])
		group->sub_ctx[id] = sub_mp;
	spin_unlock(&group->slock);

	if (isfree)
		kfree(sub_mp_alloc);

	ipu->group[instance] = group;
	ipu_ctx->group = group;

	group->frame_work = ipu_frame_work;
	group->gtask = &ipu->gtask;
	group->gtask->id = group->id;

	chain = group->chain;

	vio_info("[%s][%d]instance = %d\n", __func__, id, instance);
	ipu_ctx->state = BIT(VIO_VIDEO_S_INPUT);

	return ret;
}

int ipu_video_streamon(struct ipu_video_ctx *ipu_ctx)
{
	struct x2a_ipu_dev *ipu_dev;
	struct vio_group *group;

	ipu_dev = ipu_ctx->ipu_dev;
	group = ipu_ctx->group;

	if (!(ipu_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS)
			| BIT(VIO_VIDEO_INIT)))) {
		vio_err("[%s][V%02d] invalid STREAM_ON is requested(%lX)\n",
			__func__, group->instance, ipu_ctx->state);
		return -EINVAL;
	}

	if (atomic_read(&ipu_dev->rsccount) > 0)
		goto p_inc;

#if CONFIG_QEMU_TEST
	timer_init(ipu_dev, 0);
#endif
	msleep(10);
p_inc:
#if CONFIG_QEMU_TEST
	if (ipu_ctx->id == GROUP_ID_SRC)
		g_test_bit |=
		    1 << INTR_IPU_FRAME_START | 1 << INTR_IPU_FRAME_DONE;
	else
		g_test_bit |= 1 << (ipu_ctx->id + 1);
#endif
	atomic_inc(&ipu_dev->rsccount);

	ipu_ctx->state = BIT(VIO_VIDEO_START);

	vio_info("[S%d][ID %d]%s\n", group->instance, ipu_ctx->id,
		 __func__);

	return 0;
}

int ipu_video_streamoff(struct ipu_video_ctx *ipu_ctx)
{
	u32 value = 0;
	u32 cfg = 0;
	u32 cnt = 20;
	struct x2a_ipu_dev *ipu_dev;
	struct vio_group *group;

	ipu_dev = ipu_ctx->ipu_dev;
	group = ipu_ctx->group;

	if (!(ipu_ctx->state & BIT(VIO_VIDEO_START))) {
		vio_err("[%s][V%02d] invalid STREAM_OFF is requested(%lX)\n",
			__func__, ipu_ctx->group->instance, ipu_ctx->state);
		return -EINVAL;
	}

	ipu_channel_wdma_disable(ipu_ctx);

	if (atomic_dec_return(&ipu_dev->rsccount) > 0)
		goto p_dec;

#if CONFIG_QEMU_TEST
	del_timer_sync(&tm[0]);
	if (ipu_ctx->id == GROUP_ID_SRC)
		g_test_bit &= ~(1 << INTR_IPU_FRAME_START)
		& ~(1 << INTR_IPU_FRAME_DONE);
	else
		g_test_bit &= ~(1 << (ipu_ctx->id + 1));

	vio_info("%s timer del\n", __func__);
#else

	cfg = ips_get_bus_ctrl() | 1 << 12;
	ips_set_bus_ctrl(cfg);

	while(1) {
		value = ips_get_bus_status();
		if(value & 1 << 28)
			break;

		msleep(10);
		cnt--;
		if(cnt == 0) {
			vio_info("%s timeout\n", __func__);
			break;
		}
	}

	cfg = ips_get_bus_ctrl() & ~(1 << 12);
	ips_set_bus_ctrl(cfg);

	ips_set_module_reset(IPU0_RST);

	ips_set_clk_ctrl(IPU0_CLOCK_GATE, false);
#endif

p_dec:
	if (ipu_ctx->framemgr->frames_mp != NULL)
		frame_manager_flush_mp(ipu_ctx->framemgr, ipu_ctx->frm_fst_ind,
			ipu_ctx->frm_num);

	ipu_ctx->state = BIT(VIO_VIDEO_STOP);

	vio_info("[S%d][ID %d]%s\n", group->instance, ipu_ctx->id,
		 __func__);

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
	struct vio_framemgr *framemgr;
	u32 buf_index;

	if (!(ipu_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS) |
		      BIT(VIO_VIDEO_INIT) | BIT(VIO_VIDEO_S_INPUT)))) {
		vio_err("[%s][V%02d] invalid REQBUFS is requested(%lX)\n",
			__func__, ipu_ctx->group->instance, ipu_ctx->state);
		return -EINVAL;
	}

	framemgr = ipu_ctx->framemgr;
	ret = frame_manager_open_mp(framemgr, buffers, &buf_index);
	if (ret) {
		vio_err("frame manage open failed, ret(%d)", ret);
		return ret;
	}
	ipu_ctx->frm_fst_ind = buf_index;
	ipu_ctx->frm_num = buffers;
	vio_info("ipu_ctx->frm_fst_ind %d", ipu_ctx->frm_fst_ind);
	for (i = buf_index; i < (buf_index + buffers); i++) {
		framemgr->frames_mp[i]->data = ipu_ctx->group;
	}

	ipu_ctx->state = BIT(VIO_VIDEO_REBUFS);

	return ret;
}

int ipu_video_getindex(struct ipu_video_ctx *ipu_ctx)
{
	u32 buf_index;

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
	u8 dispatch_cnt;

	index = frameinfo->bufferindex;
	framemgr = ipu_ctx->framemgr;
	BUG_ON(index >= framemgr->num_frames);

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = framemgr->frames_mp[index];
	if (frame->state == FS_FREE) {
		vio_dbg("q:FREE->REQ,proc%d bidx%d", ipu_ctx->proc_id, index);
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		trans_frame(framemgr, frame, FS_REQUEST);
	} else if (frame->state == FS_USED) {
		dispatch_cnt = framemgr->dispatch_cnt[index] - 1;
		if (dispatch_cnt >= VIO_MAX_SUB_PROCESS) {
			vio_err("frame(%d) dispatch count(%d) err\n", index,
				dispatch_cnt);
			ret = -EINVAL;
			goto err;
		} else if (dispatch_cnt == 0) {
			vio_dbg("q:USED->REQ,proc%d bidx%d", ipu_ctx->proc_id,
				index);
			framemgr->dispatch_cnt[index] = dispatch_cnt;
			memcpy(&frame->frameinfo, frameinfo,
				sizeof(struct frame_info));
			trans_frame(framemgr, frame, FS_REQUEST);
		} else {
			vio_dbg("q:disp cnt%d,proc%d bidx%d", dispatch_cnt,
				ipu_ctx->proc_id, index);
			framemgr->dispatch_cnt[index] = dispatch_cnt;
			ret = 0;
			goto err;
		}
	} else {
		vio_err("frame(%d) is invalid state(%d)\n", index,
			frame->state);
		ret = -EINVAL;
		goto err;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	group = ipu_ctx->group;
	if (ipu_ctx->leader == true && group->leader) {
		vio_group_start_trigger(group->gtask, frame);
	}
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
	struct ipu_sub_mp *sub_mp;

	framemgr = ipu_ctx->framemgr;
	sub_mp = ipu_ctx->sub_mp;
	done_list = &framemgr->queued_list[FS_COMPLETE];
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	if (!list_empty(done_list)) {
		frame = peek_frame(framemgr, FS_COMPLETE);
		if (frame) {
			memcpy(frameinfo, &frame->frameinfo,
				sizeof(struct frame_info));
			trans_frame(framemgr, frame, FS_FREE);
			vio_dbg("ipu minor:%d dqbuf(f%d b%d) from FS_COMPLETE.",
				ipu_ctx->id, frame->frameinfo.frame_id,
				frame->frameinfo.bufferindex);
		}
		framemgr_x_barrier_irqr(framemgr, 0, flags);

		return ret;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	/* add to client list */
	if (!ipu_ctx->ispoll) {
		spin_lock(&sub_mp->slock);
		ret = ipu_put_client(sub_mp, ipu_ctx);
		if (ret < 0) {
			spin_unlock(&sub_mp->slock);
			vio_err("ipu put client error.");
			return ret;
		} else {
			vio_dbg("dq(%d):put client ok.", ipu_ctx->id);
		}
		spin_unlock(&sub_mp->slock);
		wait_event_interruptible(ipu_ctx->done_wq,
				ipu_ctx->event == VIO_FRAME_DONE);
	}

	/* copy frameinfo */
	if (ipu_ctx->event == VIO_FRAME_DONE) {
		spin_lock(&sub_mp->slock);
		memcpy(frameinfo, &ipu_ctx->frameinfo,
			sizeof(struct frame_info));
		spin_unlock(&sub_mp->slock);
		vio_dbg("dq proc%d bidx%d fid%d", ipu_ctx->proc_id,
			frameinfo->bufferindex, frameinfo->frame_id);
		ipu_ctx->event = 0;
	} else {
		ret = -EFAULT;
		ipu_ctx->event = 0;
	}

	return ret;
}

static long x2a_ipu_ioctl(struct file *file, unsigned int cmd,
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
		vio_dbg("V[%d][%d]=====IPU_IOC_STREAM==enable %d===========\n",
			 group->instance, ipu_ctx->id, enable);
		ipu_video_s_stream(ipu_ctx, ! !enable);
		break;
	case IPU_IOC_DQBUF:
		ipu_video_dqbuf(ipu_ctx, &frameinfo);
		ret = copy_to_user((void __user *) arg, (char *) &frameinfo,
				 sizeof(struct frame_info));
		vio_dbg("V[%d][%d]=====IPU_IOC_DQBUF(p%d b%d f%d)==ret %d====\n",
			 group->instance, ipu_ctx->id, ipu_ctx->proc_id,
			 frameinfo.bufferindex, frameinfo.frame_id, ret);
		if (ret)
			return -EFAULT;
		break;
	case IPU_IOC_QBUF:
		ret = copy_from_user((char *) &frameinfo, (u32 __user *) arg,
				   sizeof(struct frame_info));
		vio_dbg("V[%d][%d]=====IPU_IOC_QBUF(p%d b%d)==ret %d===========\n",
			 group->instance, ipu_ctx->id, ipu_ctx->proc_id,
			 frameinfo.bufferindex, ret);
		if (ret)
			return -EFAULT;
		ipu_video_qbuf(ipu_ctx, &frameinfo);
		break;
	case IPU_IOC_REQBUFS:
		vio_dbg("V[%d][%d]=====IPU_IOC_REQBUFS==ret %d===========\n",
			 group->instance, ipu_ctx->id, ret);
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
		vio_dbg("V[%d][%d]=====IPU_IOC_GET_INDEX==%d=============\n",
			 group->instance, ipu_ctx->id, buf_index);
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
	default:
		vio_err("wrong ioctl command\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

void ipu_set_iar_output(struct ipu_video_ctx *ipu_ctx, struct vio_frame *frame)
{
#ifdef X3_IAR_INTERFACE
	u32 display_layer = 0;

	display_layer = ipu_get_iar_display_type();
	if(ipu_ctx->id == display_layer)
		ipu_set_display_addr(frame->frameinfo.addr[0], frame->frameinfo.addr[1]);
#endif
}

void ipu_frame_done(struct ipu_sub_mp *sub_mp)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	struct ipu_video_ctx *ipu_ctx;
	unsigned long flags;

	if(!sub_mp) {
		vio_err("%s:%d sub_mp not init.\n", __func__, __LINE__);
		return;
	}
	group = sub_mp->group;
	framemgr = &sub_mp->framemgr;
	ipu_ctx = sub_mp->dev[0];
	if(!ipu_ctx) {
		vio_err("%s:%d sub_mp.dev[0] is null .\n", __func__, __LINE__);
		return;
	}
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		if(group->get_timestamps){
			frame->frameinfo.frame_id = group->frameid.frame_id;
			frame->frameinfo.timestamps =
			    group->frameid.timestamps;
		}

		do_gettimeofday(&frame->frameinfo.tv);

		vio_dbg("done bidx%d fid%d ",
			frame->frameinfo.bufferindex,
			frame->frameinfo.frame_id);
		ipu_set_iar_output(ipu_ctx, frame);
		trans_frame(framemgr, frame, FS_COMPLETE);
	} else {
		vio_err("PROCESS queue has no member;\n");
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
}

int ipu_put_client(struct ipu_sub_mp *sub_mp,
		struct ipu_video_ctx *ipu_sub_ctx)
{
	if (!ipu_sub_ctx) {
		vio_err("[%s]invalid client device.", __func__);
		return -EFAULT;
	}
	if (sub_mp->client_count > VIO_MAX_SUB_PROCESS) {
		vio_err("[%s]client count %d err.", __func__,
			sub_mp->client_count);
		return -EFAULT;
	}
	list_add_tail(&ipu_sub_ctx->list, &sub_mp->client_list);
	sub_mp->client_count++;
	ipu_sub_ctx->in_list = 1;

	return 0;
}

struct ipu_video_ctx *ipu_get_client(struct ipu_sub_mp *sub_mp)

{
	struct ipu_video_ctx *ipu_sub_ctx;

	if (!sub_mp->client_count)
		return NULL;
	ipu_sub_ctx = list_first_entry(&sub_mp->client_list,
			struct ipu_video_ctx, list);
	list_del(&ipu_sub_ctx->list);
	sub_mp->client_count--;
	ipu_sub_ctx->in_list = 0;
	return ipu_sub_ctx;
}

void ipu_dispatch_frm(struct ipu_work *ipu_work)
{
	unsigned long flags;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	u32 i;
	struct ipu_video_ctx *ipu_ctx;
	struct frame_info *frame_info;
	u32 client_count;
	u32 bufindex;
	struct ipu_sub_mp *sub_mp;
	u32 id;
	u32 instance = 0;
	struct vio_group *group;
	struct x2a_ipu_dev *ipu;

	ipu = ipu_work->ipu;
	instance = atomic_read(&ipu_work->instance);
	group = ipu->group[instance];
	for (id = 0; id < MAX_SUB_DEVICE; id++) {
		if (!(ipu_work->work_sta & (1 << id)))
			continue;
		sub_mp = group->sub_ctx[id];
		framemgr = &sub_mp->framemgr;
		spin_lock(&sub_mp->slock);
		client_count = sub_mp->client_count;
		if (!client_count) {
			vio_dbg("disp:warn,client count %d", client_count);
		} else {
			framemgr_e_barrier_irqs(framemgr, 0, flags);
			frame = peek_frame(framemgr, FS_COMPLETE);
			if (frame)
				trans_frame(framemgr, frame, FS_USED);
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			if (!frame)
				vio_dbg("no frame in FS_COMPLETE when dispatching");
			else
				bufindex = frame->frameinfo.bufferindex;
		}
		for (i = 0; i < client_count; i++) {
			ipu_ctx = ipu_get_client(sub_mp);
			if (!ipu_ctx) {
				spin_unlock(&sub_mp->slock);
				vio_dbg("ipu:client not enough when dispatching.");
				break;
			}
			if (frame) {
				frame_info = &ipu_ctx->frameinfo;
				memcpy(frame_info, &frame->frameinfo,
					sizeof(struct frame_info));
				vio_dbg("disp%d ,bidx%d fid%d", i,
					frame_info->bufferindex,
					frame_info->frame_id);
				framemgr_e_barrier_irqs(framemgr, 0, flags);
				framemgr->dispatch_cnt[bufindex]++;
				framemgr_x_barrier_irqr(framemgr, 0, flags);
				ipu_ctx->event = VIO_FRAME_DONE;

			} else {
				ipu_ctx->event = VIO_FRAME_NDONE;
			}
			wake_up(&ipu_ctx->done_wq);
		}
		spin_unlock(&sub_mp->slock);
	}
}

static void ipu_isr_bh(struct work_struct *data)
{
	struct ipu_work *ipu_work;

	ipu_work = container_of(data, struct ipu_work, work);
	ipu_dispatch_frm(ipu_work);
}

void ipu_work_init(struct x2a_ipu_dev *ipu)
{
	u32 i;

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		INIT_WORK(&ipu->work[i].work, ipu_isr_bh);
		ipu->work[i].work_sta = 0;
		atomic_set(&ipu->work[i].instance, i);
		ipu->work[i].ipu = ipu;
	}
}
static irqreturn_t ipu_isr(int irq, void *data)
{
	u32 status = 0;
	u32 instance = 0;
	u32 size_err = 0;
	struct x2a_ipu_dev *ipu;
	struct vio_group *group;
	struct vio_group_task *gtask;
	struct ipu_work	*ipu_work;
	struct ipu_sub_mp	*sub_mp;

	ipu = data;
	gtask = &ipu->gtask;
	instance = atomic_read(&ipu->instance);
	group = ipu->group[instance];
	ipu_get_intr_status(ipu->base_reg, &status, true);
	size_err = ipu_get_size_err(ipu->base_reg);
	vio_dbg("%s status = 0x%x\n", __func__, status);

	if (size_err) {
		ipu_clear_size_err(ipu->base_reg, 1);
		ipu_clear_size_err(ipu->base_reg, 0);
		vio_err("IPU size error detection(0x%x)\n", size_err);
	}

	if (status & (1 << INTR_IPU_US_FRAME_DROP)) {
		vio_err("US Frame drop\n");
	}

	if (status & (1 << INTR_IPU_DS0_FRAME_DROP)) {
		vio_err("DS0 Frame drop\n");
	}

	if (status & (1 << INTR_IPU_DS1_FRAME_DROP)) {
		vio_err("DS1 Frame drop\n");
	}

	if (status & (1 << INTR_IPU_DS2_FRAME_DROP)) {
		vio_err("DS2 Frame drop\n");
	}

	if (status & (1 << INTR_IPU_DS3_FRAME_DROP)) {
		vio_err("DS3 Frame drop\n");
	}

	if (status & (1 << INTR_IPU_DS4_FRAME_DROP)) {
		vio_err("DS4 Frame drop\n");
	}

	ipu_work = &ipu->work[instance];
	ipu_work->work_sta = 0;
	if (status & (1 << INTR_IPU_FRAME_DONE)) {
		if (test_bit(IPU_DMA_INPUT, &ipu->state)) {
			up(&gtask->hw_resource);
			sub_mp = group->sub_ctx[GROUP_ID_SRC];
			if (sub_mp) {
				ipu_frame_done(sub_mp);
				ipu_work->work_sta |= (1 << GROUP_ID_SRC);
			}
		}
	}

	if (status & (1 << INTR_IPU_US_FRAME_DONE)) {
		sub_mp = group->sub_ctx[GROUP_ID_US];
		if (sub_mp) {
			ipu_frame_done(sub_mp);
			ipu_work->work_sta |= (1 << GROUP_ID_US);
		}
	}

	if (status & (1 << INTR_IPU_DS0_FRAME_DONE)) {
		sub_mp = group->sub_ctx[GROUP_ID_DS0];
		if (sub_mp) {
			ipu_frame_done(sub_mp);
			ipu_work->work_sta |= (1 << GROUP_ID_DS0);
		}
	}

	if (status & (1 << INTR_IPU_DS1_FRAME_DONE)) {
		sub_mp = group->sub_ctx[GROUP_ID_DS1];
		if (sub_mp) {
			ipu_frame_done(sub_mp);
			ipu_work->work_sta |= (1 << GROUP_ID_DS1);
		}
	}

	if (status & (1 << INTR_IPU_DS2_FRAME_DONE)
	    && test_bit(IPU_DS2_DMA_OUTPUT, &ipu->state)) {
		sub_mp = group->sub_ctx[GROUP_ID_DS2];
		if (sub_mp) {
			ipu_frame_done(sub_mp);
			ipu_work->work_sta |= (1 << GROUP_ID_DS2);
		}
	}

	if (status & (1 << INTR_IPU_DS3_FRAME_DONE)) {
		sub_mp = group->sub_ctx[GROUP_ID_DS3];
		if (sub_mp) {
			ipu_frame_done(sub_mp);
			ipu_work->work_sta |= (1 << GROUP_ID_DS3);
		}
	}

	if (status & (1 << INTR_IPU_DS4_FRAME_DONE)) {
		sub_mp = group->sub_ctx[GROUP_ID_DS4];
		if (sub_mp) {
			ipu_frame_done(sub_mp);
			ipu_work->work_sta |= (1 << GROUP_ID_DS4);
		}
	}
	if (ipu_work->work_sta) {
		queue_work(system_highpri_wq, &ipu_work->work);
	}

	if (status & (1 << INTR_IPU_FRAME_START)) {
		if (test_bit(IPU_OTF_INPUT, &ipu->state)) {
			up(&gtask->hw_resource);
		}
		if (group && group->get_timestamps) {
			vio_get_frame_id(group);
			vio_dbg("IPU frame count = %d\n", group->frameid.frame_id);
		}
	}

	return IRQ_HANDLED;
}

static struct file_operations x2a_ipu_fops = {
	.owner = THIS_MODULE,
	.open = x2a_ipu_open,
	.write = x2a_ipu_write,
	.read = x2a_ipu_read,
	.poll = x2a_ipu_poll,
	.release = x2a_ipu_close,
	.unlocked_ioctl = x2a_ipu_ioctl,
	.compat_ioctl = x2a_ipu_ioctl,
};

static int x2a_ipu_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_ipu_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_ipu_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x2a_ipu_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x2a_ipu_pm_ops = {
	.suspend = x2a_ipu_suspend,
	.resume = x2a_ipu_resume,
	.runtime_suspend = x2a_ipu_runtime_suspend,
	.runtime_resume = x2a_ipu_runtime_resume,
};

int x2a_ipu_device_node_init(struct x2a_ipu_dev *ipu)
{
	int ret = 0;
	dev_t devno;
	struct device *dev = NULL;
	int i = 0;
	char name[32];

	ret = alloc_chrdev_region(&devno, 0, MAX_DEVICE, "x2a_ipu");
	if (ret < 0) {
		vio_err("Error %d while alloc chrdev ipu", ret);
		goto err_req_cdev;
	}

	cdev_init(&ipu->cdev, &x2a_ipu_fops);
	ipu->cdev.owner = THIS_MODULE;
	ret = cdev_add(&ipu->cdev, devno, MAX_DEVICE);
	if (ret) {
		vio_err("Error %d while adding x2 ipu cdev", ret);
		goto err;
	}

	if (vps_class)
		ipu->class = vps_class;
	else
		ipu->class = class_create(THIS_MODULE, X2A_IPU_NAME);

	dev = device_create(ipu->class, NULL, MKDEV(MAJOR(devno), 0), NULL,
			  "ipu_s0");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		vio_err("ipu device create fail\n");
		goto err;
	}

	dev = device_create(ipu->class, NULL, MKDEV(MAJOR(devno), 1), NULL,
			  "ipu_us");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		vio_err("ipu device create fail\n");
		goto err;
	}
	for (i = 0; i < 6; i++) {
		snprintf(name, 32, "ipu_ds%d", i);
		dev = device_create(ipu->class, NULL, MKDEV(MAJOR(devno), i + 2),
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
	unregister_chrdev_region(devno, MAX_DEVICE);
	return ret;
}

static ssize_t ipu_reg_dump(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x2a_ipu_dev *ipu;

	ipu = dev_get_drvdata(dev);

	if (atomic_read(&ipu->rsccount) == 0)
		ips_set_clk_ctrl(IPU0_CLOCK_GATE, true);

	ipu_hw_dump(ipu->base_reg);

	if (atomic_read(&ipu->rsccount) == 0)
		ips_set_clk_ctrl(IPU0_CLOCK_GATE, false);

	return 0;
}

static DEVICE_ATTR(regdump, 0444, ipu_reg_dump, NULL);

static int x2a_ipu_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x2a_ipu_dev *ipu;
	struct device *dev = NULL;
	struct resource *mem_res;

	ipu = kzalloc(sizeof(struct x2a_ipu_dev), GFP_KERNEL);
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

#if CONFIG_QEMU_TEST
	ipu->base_reg = kzalloc(0x1000, GFP_KERNEL);
#endif
	/* Get IRQ SPI number */
	ipu->irq = platform_get_irq(pdev, 0);
	if (ipu->irq < 0) {
		vio_err("Failed to get ipu->irq(%d)", ipu->irq);
		ret = -EBUSY;
		goto err_get_irq;
	}

	ret = vio_group_init_mp(GROUP_ID_IPU);
	if (ret < 0) {
		vio_err("init chain group(IPU) multi-process error.");
		goto err_init_group_mp;
	}
	ipu_work_init(ipu);

	ret = request_irq(ipu->irq, ipu_isr, IRQF_TRIGGER_HIGH, "ipu", ipu);
	if (ret) {
		vio_err("request_irq(IRQ_IPU %d) is fail(%d)", ipu->irq, ret);
		goto err_get_irq;
	}

	x2a_ipu_device_node_init(ipu);

	dev = &pdev->dev;
	ret = device_create_file(dev, &dev_attr_regdump);
	if(ret < 0) {
		vio_err("create regdump failed (%d)\n",ret);
		goto p_err;
	}
	platform_set_drvdata(pdev, ipu);

	sema_init(&ipu->gtask.hw_resource, 1);
	atomic_set(&ipu->gtask.refcount, 0);
	atomic_set(&ipu->rsccount, 0);

	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

	return 0;

err_init_group_mp:
err_get_irq:
	iounmap(ipu->base_reg);

err_get_resource:
	kfree(ipu);
p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;

}

static int x2a_ipu_remove(struct platform_device *pdev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x2a_ipu_match[] = {
	{
	 .compatible = "hobot,x2a-ipu",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, x2a_ipu_match);

static struct platform_driver x2a_ipu_driver = {
	.probe = x2a_ipu_probe,
	.remove = x2a_ipu_remove,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x2a_ipu_pm_ops,
		   .of_match_table = x2a_ipu_match,
		   }
};

#else
static struct platform_device_id x2a_ipu_driver_ids[] = {
	{
	 .name = MODULE_NAME,
	 .driver_data = 0,
	 },
	{},
};

MODULE_DEVICE_TABLE(platform, x2a_ipu_driver_ids);

static struct platform_driver x2a_ipu_driver = {
	.probe = x2a_ipu_probe,
	.remove = x2a_ipu_remove,
	.id_table = x2a_ipu_driver_ids,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x2a_ipu_pm_ops,
		   }
};
#endif

#if CONFIG_QEMU_TEST
static void ipu_timer(unsigned long data)
{
	u32 status;
	u32 instance;
	struct x2a_ipu_dev *ipu;
	struct vio_group *group;
	struct vio_group_task *gtask;

	ipu = (struct x2a_ipu_dev *) data;
	gtask = &ipu->gtask;
	instance = atomic_read(&ipu->instance);
	group = ipu->group[instance];
	ipu_get_intr_status(ipu->base_reg, &status, true);

	status = g_test_bit;
	vio_info("%s status = %x\n", __func__, status);
	mod_timer(&tm[0], jiffies + HZ / 25);
	if (status & (1 << INTR_IPU_FRAME_DONE)) {
		if (test_bit(IPU_DMA_INPUT, &ipu->state)) {
			up(&gtask->hw_resource);
			ipu_frame_done(group->sub_ctx[GROUP_ID_SRC]);
			vio_info("%s ipu_frame_done\n", __func__);
		}
	}

	if (status & (1 << INTR_IPU_US_FRAME_DONE)) {
		ipu_frame_done(group->sub_ctx[GROUP_ID_US]);
	}

	if (status & (1 << INTR_IPU_DS0_FRAME_DONE)) {
		ipu_frame_done(group->sub_ctx[GROUP_ID_DS0]);
	}

	if (status & (1 << INTR_IPU_DS1_FRAME_DONE)) {
		ipu_frame_done(group->sub_ctx[GROUP_ID_DS1]);
	}

	if (status & (1 << INTR_IPU_DS2_FRAME_DONE)) {
		ipu_frame_done(group->sub_ctx[GROUP_ID_DS2]);
	}

	if (status & (1 << INTR_IPU_DS3_FRAME_DONE)) {
		ipu_frame_done(group->sub_ctx[GROUP_ID_DS3]);
	}

	if (status & (1 << INTR_IPU_DS4_FRAME_DONE)) {
		ipu_frame_done(group->sub_ctx[GROUP_ID_DS4]);
	}

	if (status & (1 << INTR_IPU_FRAME_START)) {
		if (test_bit(IPU_OTF_INPUT, &ipu->state))
			up(&gtask->hw_resource);
	}

}

static int timer_init(struct x2a_ipu_dev *ipu, int index)
{
	init_timer(&tm[index]);
	tm[index].expires = jiffies + HZ / 10;
	tm[index].function = ipu_timer;
	tm[index].data = (unsigned long) ipu;
	add_timer(&tm[index]);
}

static int __init x2a_ipu_init(void)
{
	int ret = 0;
	struct x2a_ipu_dev *ipu;

	ipu = kzalloc(sizeof(struct x2a_ipu_dev), GFP_KERNEL);
	if (!ipu) {
		vio_err("ipu is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	ipu->base_reg = kzalloc(0x1000, GFP_KERNEL);

	x2a_ipu_device_node_init(ipu);
	atomic_set(&ipu->gtask.refcount, 0);
	spin_lock_init(&ipu->shared_slock);
	sema_init(&ipu->gtask.hw_resource, 1);

	vio_info("[FRT:D] %s(%d)\n", __func__, ret);
	atomic_set(&ipu->rsccount, 0);

	return 0;

p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;
}

#else

static int __init x2a_ipu_init(void)
{
	int ret = platform_driver_register(&x2a_ipu_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}
#endif


late_initcall(x2a_ipu_init);

static void __exit x2a_ipu_exit(void)
{
	platform_driver_unregister(&x2a_ipu_driver);
}

module_exit(x2a_ipu_exit);

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X2A IPU driver");
MODULE_LICENSE("GPL");
