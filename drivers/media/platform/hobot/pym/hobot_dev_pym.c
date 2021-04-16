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
#include <linux/ion.h>
#include <linux/pm_qos.h>

#include "hobot_dev_pym.h"
#include "pym_hw_api.h"

#define MODULE_NAME "X3 PYM"

static int g_pym_fps[VIO_MAX_STREAM] = {0, };
static int g_pym_idx[VIO_MAX_STREAM] = {0, };
static int g_pym_fps_lasttime[VIO_MAX_STREAM] = {0, };

void pym_update_param(struct pym_subdev *subdev);
void pym_update_param_ch(struct pym_subdev *subdev);
int pym_video_streamoff(struct pym_video_ctx *pym_ctx);

extern struct ion_device *hb_ion_dev;
static struct pm_qos_request pym_pm_qos_req;
static struct mutex pym_mutex;

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

	ret = mutex_lock_interruptible(&pym_mutex);
	if (ret) {
		vio_err("open ipu mutex lock failed:%d", ret);
		goto p_err;
	}
	if (atomic_read(&pym->open_cnt) == 0) {
		pm_qos_add_request(&pym_pm_qos_req, PM_QOS_DEVFREQ, 10000);
		msleep(100);
		atomic_set(&pym->backup_fcount, 0);
		atomic_set(&pym->sensor_fcount, 0);
		atomic_set(&pym->enable_cnt, 0);
		vio_clk_enable("sif_mclk");
		vio_clk_enable("pym_mclk");
	}

	atomic_inc(&pym->open_cnt);
	mutex_unlock(&pym_mutex);
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
	struct vio_framemgr *framemgr;
	unsigned long flags;
	struct list_head *done_list;
	struct x3_pym_dev *pym;

	pym_ctx = file->private_data;
	framemgr = pym_ctx->framemgr;
	pym = pym_ctx->pym_dev;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	pym_ctx->subdev->poll_mask |= (1 << pym_ctx->ctx_index);
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	poll_wait(file, &pym_ctx->done_wq, wait);
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	done_list = &framemgr->queued_list[FS_COMPLETE];
	if (!list_empty(done_list)) {
		pym->statistic.pollin_comp[pym_ctx->group->instance]++;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	if (pym_ctx->event == VIO_FRAME_DONE) {
		pym->statistic.pollin_fe[pym_ctx->group->instance]++;
		ret = POLLIN;
	} else if (pym_ctx->event == VIO_FRAME_NDONE) {
		pym->statistic.pollerr[pym_ctx->group->instance]++;
		ret = POLLERR;
	}

	return ret;
}

static int x3_pym_try_release_process_ion(struct pym_video_ctx *pym_ctx,
	struct mp_vio_frame *frame_array_addr[])
{
	struct mp_vio_frame *frame, *frame_start;
	int i = 0, k = 0, frm_num = 0;
	struct x3_pym_dev *pym = pym_ctx->pym_dev;

	if (!frame_array_addr) {
		return 0;
	}

	frame_start = (struct mp_vio_frame *)frame_array_addr[0];
	frm_num = frame_start->ion_bufffer_num;

	// process frames free, allocated in reqbufs
	for (i = 0; i < frm_num ; i++) {
		frame = (struct mp_vio_frame *)frame_array_addr[i];
		if (!frame)
			continue;
		for (k = 0; k < frame->plane_count; k++) {
			ion_free(pym->ion_client , frame->ion_handle[k]);
		}
		frame_array_addr[i] = NULL;
	}

	vfree(frame_start);

	return 0;
}

static int x3_pym_release_subdev_all_ion(struct pym_subdev *subdev)
{
	struct mp_vio_frame *frame, *frame_start, *frame_array_addr[VIO_MP_MAX_FRAMES];
	int i, j, k, first_index = 0, frm_num = 0;
	struct x3_pym_dev *pym = subdev->pym_dev;
	struct vio_framemgr *framemgr = &subdev->framemgr;
	unsigned long flags;

	vio_dbg("%s: ctx_mask %lx", __func__, subdev->val_ctx_mask);

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	memcpy(frame_array_addr, framemgr->frames_mp,
		sizeof(struct mp_vio_frame *)*VIO_MP_MAX_FRAMES);
	for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
		frame = (struct mp_vio_frame *)framemgr->frames_mp[i];
		if (!frame)
			continue;
		list_del(&frame->common_frame.list);
		framemgr->queued_count[frame->common_frame.state]--;
		framemgr->frames_mp[i] = NULL;
		framemgr->index_state[i] = FRAME_IND_FREE;
		frame->common_frame.dispatch_mask = 0x00;
		frame->common_frame.poll_mask = 0x00;
	}

	framemgr->max_index = 0;
	framemgr->num_frames = 0;
	framemgr->ctx_mask = 0;
	framemgr->state = FRAMEMGR_CREAT;
	subdev->frameinfo.bufferindex = -1;
	subdev->poll_mask = 0x00;
	for (i = 0; i < FS_INVALID; i++) {
		vio_dbg("framemgr %d queue num:%d", i, framemgr->queued_count[i]);
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	// process frames free, allocated in reqbufs
	for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
		frame_start = (struct mp_vio_frame *)frame_array_addr[i];
		if (!frame_start)
			continue;
		first_index = frame_start->first_indx;
		frm_num = frame_start->ion_bufffer_num;

		// free ion
		for (j = first_index; j < first_index + frm_num ; j++) {
			frame = (struct mp_vio_frame *)frame_array_addr[j];
			if (!frame)
				continue;
			for (k = 0; k < frame->plane_count; k++) {
				ion_free(pym->ion_client , frame->ion_handle[k]);
			}
			frame_array_addr[j] = NULL;
		}

		vfree(frame_start);
	}

	vio_dbg("%s ion free done", __func__);

	return 0;
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

static int x3_pym_close(struct inode *inode, struct file *file)
{
	struct pym_video_ctx *pym_ctx;
	struct vio_group *group;
	struct x3_pym_dev *pym;
	struct pym_subdev *subdev;
	u32 index = 0;
	u32 count = 0;
	int instance = 0;
	unsigned long flags;

	pym_ctx = file->private_data;
	pym = pym_ctx->pym_dev;
	if (pym_ctx->state & BIT(VIO_VIDEO_OPEN)) {
		vio_info("[Sx][V%d] %s: only open.\n", pym_ctx->id, __func__);
		atomic_dec(&pym->open_cnt);
		kfree(pym_ctx);
		return 0;
	}

	vio_dbg("pym pipeline %d close", pym_ctx->belong_pipe);

	if (!(pym_ctx->state & BIT(VIO_VIDEO_STOP))) {
		pym_video_streamoff(pym_ctx);
	}

	index = pym_ctx->frm_fst_ind;
	count = pym_ctx->frm_num;
	group = pym_ctx->group;
	subdev = pym_ctx->subdev;

	// in this function, we only need to release current process captured frame
	//vio_dbg("from pym index %d, cnt %d, subdev id:%d", index, count, subdev->id);
	//frame_manager_flush_mp(pym_ctx->framemgr, index, count, pym_ctx->ctx_index);
	mutex_lock(&pym_mutex);
	vio_dbg("pym in close,begin clean");
	if ((group) && atomic_dec_return(&subdev->refcount) == 0) {
		vio_dbg("pym subdev %d", subdev->id);
		subdev->state = 0;
		if (group->gtask)
			vio_group_task_stop(group->gtask);
		instance = group->instance;
		if (atomic_dec_return(&group->node_refcount) == 0)
			clear_bit(VIO_GROUP_INIT, &group->state);

		// this is the subdev's last process close, it's ok to free all ion buffer
		x3_pym_release_subdev_all_ion(subdev);
		pym->statistic.enable[group->instance] = 0;
		memset(&pym->subdev[group->instance].pym_cfg, 0, sizeof(pym_cfg_t));
	}

	if (atomic_dec_return(&pym->open_cnt) == 0) {
		clear_bit(PYM_OTF_INPUT, &pym->state);
		clear_bit(PYM_DMA_INPUT, &pym->state);
		clear_bit(PYM_REUSE_SHADOW0, &pym->state);

		if (test_bit(PYM_HW_RUN, &pym->state)) {
			set_bit(PYM_HW_FORCE_STOP, &pym->state);
			pym_video_streamoff(pym_ctx);
			clear_bit(PYM_HW_FORCE_STOP, &pym->state);
			atomic_set(&pym->rsccount, 0);
			vio_info("pym force stream off\n");
		}
		if (group->group_scenario == VIO_GROUP_SIF_OFF_ISP_ON_IPU_ON_PYM ||
				group->group_scenario == VIO_GROUP_SIF_OFF_ISP_ON_IPU_OFF_PYM) {
			if (group != NULL)
				vio_group_done(group);
			else
				vio_info("pym group already NULL\n");
		}
		pym_hw_enable(pym, false);
		vio_reset_module(GROUP_ID_PYM);
		vio_clk_disable("pym_mclk");
		vio_clk_disable("sif_mclk");
		sema_init(&pym->gtask.hw_resource, 1);
		atomic_set(&pym->gtask.refcount, 0);
		pm_qos_remove_request(&pym_pm_qos_req);
	}
	mutex_unlock(&pym_mutex);

	pym_ctx->state = BIT(VIO_VIDEO_CLOSE);

	spin_lock_irqsave(&subdev->slock, flags);
	clear_bit(pym_ctx->ctx_index, &subdev->val_ctx_mask);
	subdev->ctx[pym_ctx->ctx_index] = NULL;
	spin_unlock_irqrestore(&subdev->slock, flags);
	kfree(pym_ctx);

	vio_info("[S%d] PYM close node\n", group->instance);
	return 0;
}

int pym_check_phyaddr(u32 addr)
{
	int ret = 0;

	ret = ion_check_in_heap_carveout(addr, 0);
	if (ret < 0) {
		vio_err("pym phyaddr 0x%x is beyond ion address region\n",
				addr);
	}

	return ret;
}

void pym_set_buffers(struct pym_subdev *subdev, struct vio_frame *frame)
{
	int i = 0;
	u32 shadow_index = 0;
	u32 base_layer_nums = 0;
	u32 y_addr, uv_addr;
	pym_cfg_t *pym_config;
	struct vio_group *group;
	struct x3_pym_dev *pym;

	group = subdev->group;
	pym = subdev->pym_dev;
	pym_config = &subdev->pym_cfg;
	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;

	for (i = 0; i < MAX_PYM_DS_COUNT; i++) {
		y_addr = frame->frameinfo.spec.ds_y_addr[i];
		uv_addr = frame->frameinfo.spec.ds_uv_addr[i];
		pym_wdma_ds_set_addr(pym->base_reg, i, y_addr, uv_addr);
		pym_ds_config_factor(pym->base_reg, shadow_index, i,
				pym_config->stds_box[i].factor);
	}

	if (pym_config->ds_layer_en > 3 && pym_config->ds_layer_en < 24) {
		base_layer_nums = pym_config->ds_layer_en / 4;
		pym_ds_enabe_base_layer(pym->base_reg, shadow_index, base_layer_nums);
	}

	for (i = 0; i < MAX_PYM_US_COUNT; i++) {
		y_addr = frame->frameinfo.spec.us_y_addr[i];
		uv_addr = frame->frameinfo.spec.us_uv_addr[i];
		pym_wdma_us_set_addr(pym->base_reg, i, y_addr, uv_addr);
	}
	pym_us_enabe_layer(pym->base_reg, shadow_index, pym_config->us_layer_en);

	subdev->disable_flag = false;
	vio_dbg("S%d: %s shadow %d\n", group->instance, __func__, shadow_index);
}

static void pym_disable_layer(struct pym_subdev *subdev)
{
	int i = 0;
	u32 shadow_index = 0;
	pym_cfg_t *pym_config;
	struct vio_group *group;
	struct x3_pym_dev *pym;

	group = subdev->group;
	pym = subdev->pym_dev;
	pym_config = &subdev->pym_cfg;
	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;

	for (i = 0; i < MAX_PYM_DS_COUNT; i++) {
		pym_ds_config_factor(pym->base_reg, shadow_index, i, 0);
	}

	pym_ds_enabe_base_layer(pym->base_reg, shadow_index, 0);
	pym_us_enabe_layer(pym->base_reg, shadow_index, 0);
	subdev->disable_flag = true;

	vio_dbg("S%d: %s shadow %d\n", group->instance, __func__, shadow_index);
}
static void pym_frame_work(struct vio_group *group)
{
	struct x3_pym_dev *pym;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct pym_subdev * subdev;
	unsigned long flags;
	u32 instance = 0;
	u8 shadow_index = 0;

	subdev = group->sub_ctx[0];
	if (unlikely(!subdev)) {
		vio_err("%s error subdev null,instance %d", __func__, instance);
		return;
	}

	instance = group->instance;
	subdev = group->sub_ctx[0];
	pym = subdev->pym_dev;

	if (instance < MAX_SHADOW_NUM)
		shadow_index = instance;

	atomic_inc(&pym->backup_fcount);

	atomic_set(&pym->instance, instance);

	framemgr = &subdev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);

	frame = peek_frame(framemgr, FS_PROCESS);
	/*
	 *only all online scene pro queue may have buf;
	 *non all online scene if pro queue have member,
	 *frame work may already called, so skip req->pro
	 */
	if (frame && !vio_check_all_online_state(group))
		goto end_req_to_pro;

	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame) {
		pym_set_shd_rdy(pym->base_reg, shadow_index, 0);
		if (test_bit(PYM_REUSE_SHADOW0, &pym->state) &&
				shadow_index == 0)
			pym_update_param(subdev);

		if (subdev->update_all) {
			frame->frameinfo.dynamic_flag = subdev->update_all;
			subdev->update_all = 0;
			pym_update_param(subdev);
		}
		if (subdev->update_ch == 1) {
			subdev->update_ch = 0;
			pym_update_param_ch(subdev);
		}

		pym_set_common_rdy(pym->base_reg, 0);
		pym_set_shd_select(pym->base_reg, shadow_index);

		pym_set_buffers(subdev, frame);

		if (test_bit(PYM_DMA_INPUT, &pym->state)) {
			pym_rdma_set_addr(pym->base_reg,
					  frame->frameinfo.addr[0],
					  frame->frameinfo.addr[1]);
		}

		pym_set_common_rdy(pym->base_reg, 1);
		pym_set_shd_rdy(pym->base_reg, shadow_index, 1);
		atomic_inc(&pym->enable_cnt);

		if (test_bit(PYM_DMA_INPUT, &pym->state))
			pym_set_rdma_start(pym->base_reg);

		trans_frame(framemgr, frame, FS_PROCESS);
	}
end_req_to_pro:
	vio_dbg("[S%d] work (%d %d %d %d %d)",
			group->instance,
			framemgr->queued_count[FS_FREE],
			framemgr->queued_count[FS_REQUEST],
			framemgr->queued_count[FS_PROCESS],
			framemgr->queued_count[FS_COMPLETE],
			framemgr->queued_count[FS_USED]);
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	if (!test_bit(PYM_HW_CONFIG, &pym->state))
		set_bit(PYM_HW_CONFIG, &pym->state);

	return;
}

void pym_update_param(struct pym_subdev *subdev)
{
	u16 src_width, src_height, roi_width;
	u32 shadow_index = 0;
	int i = 0;
	u32 ds_bapass_uv = 0;
	pym_cfg_t *pym_config;
	struct vio_group *group;
	struct x3_pym_dev *pym;
	struct roi_rect rect;

	group = subdev->group;
	pym = subdev->pym_dev;
	pym_config = &subdev->pym_cfg;
	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;
	else
		set_bit(PYM_REUSE_SHADOW0, &pym->state);

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

	//config common register
	pym_set_common_rdy(pym->base_reg, 0);

	pym_set_frame_id(pym->base_reg, 0);
	pym_select_input_path(pym->base_reg, pym_config->img_scr);

	pym_set_common_rdy(pym->base_reg, 1);
}

void pym_update_param_ch(struct pym_subdev *subdev)
{
	u16 roi_width;
	u32 shadow_index = 0;
	pym_cfg_t *pym_config;
	struct vio_group *group;
	struct x3_pym_dev *pym;
	struct roi_rect rect;
	uint8_t ch;
	pym_scale_ch_t *pym_scale_ch;

	group = subdev->group;
	pym = subdev->pym_dev;
	pym_config = &subdev->pym_cfg;
	pym_scale_ch = &subdev->pym_cfg_ch;
	if (group->instance < MAX_SHADOW_NUM)
		shadow_index = group->instance;
	else
		set_bit(PYM_REUSE_SHADOW0, &pym->state);

	if (pym_scale_ch->type == PYM_CH_DS) {
		ch = pym_scale_ch->ch;
		if (ch >= MAX_PYM_DS_COUNT) {
			vio_err("%s ds channel num err %d", __func__, ch);
			pym_set_shd_rdy(pym->base_reg, shadow_index, 1);
			return;
		}
		memcpy(&pym_config->stds_box[ch], &pym_scale_ch->ch_scale,
			sizeof(pym_scale_box_t));
		rect.roi_x = pym_scale_ch->ch_scale.roi_x;
		rect.roi_y = pym_scale_ch->ch_scale.roi_y;
		rect.roi_width = pym_scale_ch->ch_scale.tgt_width;
		rect.roi_height = pym_scale_ch->ch_scale.tgt_height;
		pym_ds_config_factor(pym->base_reg, shadow_index, ch,
				     pym_scale_ch->ch_scale.factor);
		pym_ds_config_roi(pym->base_reg, shadow_index, ch, &rect);
		roi_width = pym_scale_ch->ch_scale.roi_width;
		pym_ds_set_src_width(pym->base_reg, shadow_index, ch, roi_width);
	} else if (pym_scale_ch->type == PYM_CH_US) {
		ch = pym_scale_ch->ch;
		if (ch > MAX_PYM_US_COUNT) {
			vio_err("%s us channel num err %d", __func__, ch);
			pym_set_shd_rdy(pym->base_reg, shadow_index, 1);
			return;
		}
		memcpy(&pym_config->stus_box[ch], &pym_scale_ch->ch_scale,
			sizeof(pym_scale_box_t));
		rect.roi_x = pym_scale_ch->ch_scale.roi_x;
		rect.roi_y = pym_scale_ch->ch_scale.roi_y;
		rect.roi_width = pym_scale_ch->ch_scale.tgt_width;
		rect.roi_height = pym_scale_ch->ch_scale.tgt_height;
		pym_us_config_factor(pym->base_reg, shadow_index, ch,
				     pym_scale_ch->ch_scale.factor);
		pym_us_config_roi(pym->base_reg, shadow_index, ch, &rect);
		roi_width = pym_scale_ch->ch_scale.roi_width;
		pym_us_set_src_width(pym->base_reg, shadow_index, ch, roi_width);
	} else {
		vio_err("%s channel type err.", __func__);
		pym_set_shd_rdy(pym->base_reg, shadow_index, 1);
		return;
	}
}

int pym_bind_chain_group(struct pym_video_ctx *pym_ctx, int instance,
		int *mp_share)
{
	int ret = 0;
	int i = 0;
	struct vio_group *group;
	struct x3_pym_dev *pym;
	struct pym_subdev *subdev;
	unsigned long flags;

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

	subdev = &pym->subdev[instance];
	pym->group[instance] = group;

	group->sub_ctx[GROUP_ID_SRC] = subdev;
	pym_ctx->group = group;
	pym_ctx->subdev = subdev;
	pym_ctx->framemgr = &subdev->framemgr;
	subdev->pym_dev = pym;
	subdev->group = group;

	spin_lock_irqsave(&subdev->slock, flags);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if(!test_bit(i, &subdev->val_ctx_mask)) {
			subdev->ctx[i] = pym_ctx;
			pym_ctx->ctx_index = i;
			set_bit(i, &subdev->val_ctx_mask);
			break;
		}
	}
	spin_unlock_irqrestore(&subdev->slock, flags);
	if (i == VIO_MAX_SUB_PROCESS) {
		vio_err("alreay open too much for one pipeline\n");
		return -EFAULT;
	}
	if (atomic_inc_return(&subdev->refcount) == 1)
		frame_manager_init_mp(pym_ctx->framemgr);
	else
		*mp_share = 0xff;

	group->frame_work = pym_frame_work;
	group->gtask = &pym->gtask;
	group->gtask->id = group->id;
	pym->statistic.enable[instance] = 1;
	pym->statistic.err_stat_enable[instance] = 1;
	pym_ctx->belong_pipe = instance;

	vio_info("[S%d][V%d] %s done, ctx_index(%d) refcount(%d)\n",
		group->instance, pym_ctx->id, __func__,
		i, atomic_read(&subdev->refcount));
	pym_ctx->state = BIT(VIO_VIDEO_S_INPUT);

	return ret;
}

int pym_video_init(struct pym_video_ctx *pym_ctx, unsigned long arg)
{
	int ret = 0;
	pym_cfg_t *pym_config;
	struct vio_group *group;
	struct x3_pym_dev *pym_dev;
	struct pym_subdev *subdev;

	group = pym_ctx->group;
	pym_dev = pym_ctx->pym_dev;
	subdev = pym_ctx->subdev;
	pym_config = &subdev->pym_cfg;

	if (!(pym_ctx->state & (BIT(VIO_VIDEO_S_INPUT)
							| BIT(VIO_VIDEO_REBUFS)))) {
		vio_err("[V%02d] invalid INIT is requested(%lX)",
			group->instance, pym_ctx->state);
		return -EINVAL;
	}

	subdev = pym_ctx->subdev;
	if (test_and_set_bit(PYM_SUBDEV_INIT, &subdev->state)) {
		vio_info("subdev already init, current refcount(%d)\n",
				atomic_read(&subdev->refcount));
		return ret;
	}

	ret = copy_from_user((char *)pym_config,
				(u32 __user *) arg, sizeof(pym_cfg_t));
	if (ret)
		goto err;

	if (pym_config->img_scr == 1) {
		if (test_bit(PYM_DMA_INPUT, &pym_dev->state)){
		 	vio_err("PYM DMA input already,can't set otf input\n");
			ret = -EINVAL;
			goto err;
		}else{
			set_bit(PYM_OTF_INPUT, &pym_dev->state);
			set_bit(VIO_GROUP_OTF_INPUT, &group->state);
		}
	}else{
		if (test_bit(PYM_OTF_INPUT, &pym_dev->state)) {
			vio_err("PYM otf input already,can't set dma input\n");
			ret = -EINVAL;
			goto err;
		} else {
			set_bit(PYM_DMA_INPUT, &pym_dev->state);
			set_bit(VIO_GROUP_DMA_INPUT, &group->state);
		}
	}

	pym_update_param(subdev);

	set_bit(VIO_GROUP_DMA_OUTPUT, &group->state);
	ret = mutex_lock_interruptible(&pym_mutex);
	if (ret) {
		vio_err("ipu init mutex lock failed:%d", ret);
		goto err;
	}
	vio_group_task_start(group->gtask);
	mutex_unlock(&pym_mutex);
	atomic_inc(&group->node_refcount);

	pym_ctx->state = BIT(VIO_VIDEO_INIT);

	return ret;

err:
	clear_bit(PYM_SUBDEV_INIT, &subdev->state);
	return ret;
}

int pym_video_streamon(struct pym_video_ctx *pym_ctx)
{
	int ret = 0;
	u32 cnt = 20;
	unsigned long flags;
	struct x3_pym_dev *pym_dev;
	struct vio_group *group;

	pym_dev = pym_ctx->pym_dev;
	group = pym_ctx->group;

	if (!(pym_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS)
			| BIT(VIO_VIDEO_INIT)))) {
		vio_err("[V%02d] invalid STREAM ON is requested(%lX)",
			pym_ctx->group->instance, pym_ctx->state);
		return -EINVAL;
	}

	if (atomic_read(&pym_dev->rsccount) > 0)
		goto p_inc;

	if (group->leader && test_bit(PYM_OTF_INPUT, &pym_dev->state)) {
		while(1) {
			if (test_bit(PYM_HW_CONFIG, &pym_dev->state))
				break;

			msleep(5);
			cnt--;
			if (cnt == 0) {
				vio_info("%s timeout\n", __func__);
				break;
			}
		}
	}

	spin_lock_irqsave(&pym_dev->shared_slock, flags);

	pym_hw_enable(pym_dev, true);
	set_bit(PYM_HW_RUN, &pym_dev->state);

	spin_unlock_irqrestore(&pym_dev->shared_slock, flags);

p_inc:
	atomic_inc(&pym_dev->rsccount);
	pym_ctx->state = BIT(VIO_VIDEO_START);

	vio_info("[S%d]%s\n", group->instance, __func__);

	return ret;
}


static int pym_flush_mp_prepare(struct pym_video_ctx *pym_ctx)
{
	struct vio_framemgr *this;
	struct vio_frame *frame;
	struct vio_group *group;
	unsigned long flags;
	int i;
	int index_start;
	int buffers;
	int proc_id;

	this = pym_ctx->framemgr;
	group = pym_ctx->group;

	index_start = pym_ctx->frm_fst_ind;
	buffers = pym_ctx->frm_num;
	proc_id = pym_ctx->ctx_index;

	if ((index_start + buffers) > VIO_MP_MAX_FRAMES) {
		vio_err("invalid index when flush frame manager.");
		return -EFAULT;
	}
	if (buffers == 0) {
		vio_err("%s buffer number is ", __func__);
		return -EFAULT;
	}

	framemgr_e_barrier_irqs(this, 0, flags);
	for (i = index_start; i < (buffers + index_start); i++) {
		this->index_state[i] = FRAME_IND_STREAMOFF;
	}
	this->ctx_mask &= ~(1 << proc_id);
	/* clear the frame mask bit of this ctx*/
	for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
		if ((this->index_state[i] != FRAME_IND_USING)
			&& (this->index_state[i] != FRAME_IND_STREAMOFF))
			continue;
		frame = this->frames_mp[i];
		if (frame) {
			frame->dispatch_mask &= ~(1 << proc_id);
			frame->poll_mask &= ~(1 << proc_id);
			/*
			 * caution: this is for multiprocess share scenario
			 * if frame is not been used by any process (dispatch_mask)
			 * and is not been polled by any process (poll_mask)
			 * and is not streamoff
			 * and this frame is in FS_USED, it's ok to request
			 *
			 * if frame is not in FS_USED, it means this frame's frameinfo is never
			 * filled in qbuf ioctl, it's frameinfo content is invalid
			 */
			if (frame->dispatch_mask == 0x0000 && frame->poll_mask == 0x00
					&& (this->index_state[i] != FRAME_IND_FREE)
					&& (frame->state == FS_USED)) {
				frame->dispatch_mask |= 0xFF00;
				trans_frame(this, frame, FS_REQUEST);
				if(group->leader == true)
					vio_group_start_trigger_mp(group, frame);
			}
		}
	}
	framemgr_x_barrier_irqr(this, 0, flags);

	vio_dbg("%s proc %d:", __func__, proc_id);
	for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
		if ((this->index_state[i] != FRAME_IND_USING)
			&& (this->index_state[i] != FRAME_IND_STREAMOFF))
			continue;
		//vio_dbg("frm%d mask 0x%x", i, this->dispatch_mask[i]);
	}

	return 0;
}

int pym_video_streamoff(struct pym_video_ctx *pym_ctx)
{
	int ret = 0;
	unsigned long flag;
	struct x3_pym_dev *pym_dev;
	struct pym_subdev *subdev;
	struct vio_group *group;

	if (!(pym_ctx->state & BIT(VIO_VIDEO_START))) {
		vio_err("[V%02d] invalid STREAMOFF is requested(%lX)",
			pym_ctx->group->instance, pym_ctx->state);
		return -EINVAL;
	}

	subdev = pym_ctx->subdev;
	pym_dev = pym_ctx->pym_dev;
	group = pym_ctx->group;

	if (atomic_dec_return(&pym_dev->rsccount) > 0
		&& !test_bit(PYM_HW_FORCE_STOP, &pym_dev->state))
		goto p_dec;

	spin_lock_irqsave(&pym_dev->shared_slock, flag);
	clear_bit(PYM_HW_RUN, &pym_dev->state);
	spin_unlock_irqrestore(&pym_dev->shared_slock, flag);

p_dec:
	if (pym_ctx->framemgr->frames_mp[pym_ctx->frm_fst_ind] != NULL) {
		if (atomic_read(&subdev->refcount) > 1) {
			pym_flush_mp_prepare(pym_ctx);
		}
	}
	pym_ctx->state = BIT(VIO_VIDEO_STOP);

	vio_info("[S%d]%s\n", group->instance, __func__);

	return ret;
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
	u32 first_index = 0;
	u32 instance = 0;
	struct vio_framemgr *framemgr;
	struct pym_subdev *subdev;
	struct x3_pym_dev *pym_dev;

	if (!(pym_ctx->state & (BIT(VIO_VIDEO_STOP) | BIT(VIO_VIDEO_REBUFS) |
		      BIT(VIO_VIDEO_INIT)| BIT(VIO_VIDEO_S_INPUT)))) {
		vio_err("[V%02d] invalid REQBUFS is requested(%lX)",
			pym_ctx->group->instance, pym_ctx->state);
		return -EINVAL;
	}

	subdev = pym_ctx->subdev;
	pym_dev = pym_ctx->pym_dev;
	framemgr = pym_ctx->framemgr;
	ret = frame_manager_open_mp(framemgr, buffers, &first_index);
	if (ret) {
		vio_err("frame manage open mp failed, ret(%d)", ret);
		return ret;
	}
	pym_ctx->frm_fst_ind = first_index;
	pym_ctx->frm_num = buffers;
	instance = pym_ctx->group->instance;
	for (i = first_index; i < (buffers + first_index) ; i++) {
		framemgr->frames_mp[i]->data = pym_ctx->group;
		frame_work_init(&pym_dev->vwork[instance][i].work);
		framemgr->frames_mp[i]->mp_work = &pym_dev->vwork[instance][i].work;
		pym_dev->vwork[instance][i].group = pym_ctx->group;
	}

	pym_ctx->state = BIT(VIO_VIDEO_REBUFS);
	set_bit(PYM_SUBDEV_REQBUF, &subdev->state);
	vio_info("[S%d]%s reqbuf number %d first index %d\n",
		pym_ctx->group->instance, __func__, buffers, first_index);

	return ret;
}

enum buffer_owner pym_index_owner(struct pym_video_ctx *pym_ctx,
	u32 index)
{
	if (index >= VIO_MP_MAX_FRAMES)
		return VIO_BUFFER_OWN_INVALID;
	else if ((index >= pym_ctx->frm_fst_ind)
		&&(index < (pym_ctx->frm_fst_ind + pym_ctx->frm_num)))
		return VIO_BUFFER_THIS;
	else
		return VIO_BUFFER_OTHER;
}

int pym_video_qbuf(struct pym_video_ctx *pym_ctx, struct frame_info *frameinfo)
{
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	unsigned long flags;
	int index;
	struct x3_pym_dev *pym;
	struct mp_vio_frame  *frame_array_addr[VIO_MP_MAX_FRAMES];
	int i = 0, first_index, frame_num;
	u16 mask = 0x00;
	int tmp_num = 0;
	struct mp_vio_frame *mp_frame;
	struct vio_frame *release_frame;

	index = frameinfo->bufferindex;
	framemgr = pym_ctx->framemgr;
	group = pym_ctx->group;
	pym = pym_ctx->pym_dev;
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

	ret = pym_check_phyaddr(frameinfo->spec.ds_uv_addr[0]);
	if (ret)
		return -EINVAL;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = framemgr->frames_mp[index];
	if (frame == NULL) {
		vio_err("[S%d] %s frame null, index %d.\n", group->instance,
			__func__, index);
		ret = -EFAULT;
		goto err;
	}

	pym_ctx->frm_num_usr--;
	if (frame->state == FS_FREE) {
		frame->dispatch_mask &= ~(1 << pym_ctx->ctx_index);
		if (framemgr->index_state[index] == FRAME_IND_STREAMOFF) {
			vio_dbg("[S%d] q fail:FREE proc%d bidx%d is streaming off",
				group->instance, pym_ctx->ctx_index, index);
			ret = 0;
			goto try_releas_ion;
		}
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		frame->dispatch_mask |= 0xFF00;
		trans_frame(framemgr, frame, FS_REQUEST);
		vio_dbg("[S%d] q:FREE->REQ,proc%d bidx%d,(%d %d %d %d %d)",
			group->instance, pym_ctx->ctx_index, index,
			framemgr->queued_count[FS_FREE],
			framemgr->queued_count[FS_REQUEST],
			framemgr->queued_count[FS_PROCESS],
			framemgr->queued_count[FS_COMPLETE],
			framemgr->queued_count[FS_USED]);
		vio_dbg("pym free q index%d,y:0x%x,uv:0x%x", frame->frameinfo.bufferindex,
					frame->frameinfo.spec.ds_y_addr[0],
					frame->frameinfo.spec.ds_uv_addr[0]);
	} else if (frame->state == FS_USED) {
		frame->dispatch_mask &= ~(1 << pym_ctx->ctx_index);

		// tran frame to request if no process need
		if (frame->dispatch_mask == 0 && frame->poll_mask == 0x00) {
			if (framemgr->index_state[index] == FRAME_IND_STREAMOFF) {
				// this frame is already stream off
				// check if all it's process frames are ready to free
				vio_dbg("[S%d] q fail:USED proc%d bidx%d is streaming off",
					group->instance, pym_ctx->ctx_index, index);
				ret = 0;
				goto try_releas_ion;
			}
			memcpy(&frame->frameinfo, frameinfo,
				sizeof(struct frame_info));
			frame->dispatch_mask |= 0xFF00;
			trans_frame(framemgr, frame, FS_REQUEST);
			vio_dbg("[S%d] q:USED->REQ,proc%d bidx%d,(%d %d %d %d %d)",
				group->instance, pym_ctx->ctx_index, index,
				framemgr->queued_count[FS_FREE],
				framemgr->queued_count[FS_REQUEST],
				framemgr->queued_count[FS_PROCESS],
				framemgr->queued_count[FS_COMPLETE],
				framemgr->queued_count[FS_USED]);
			vio_dbg("pym used q index%d,y:0x%x,uv:0x%x", frame->frameinfo.bufferindex,
					frame->frameinfo.spec.ds_y_addr[0],
					frame->frameinfo.spec.ds_uv_addr[0]);
		} else {
			vio_dbg("[S%d] q:disp mask%d,proc%d bidx%d,(%d %d %d %d %d)",
					group->instance,
					frame->dispatch_mask,
					pym_ctx->ctx_index, index,
					framemgr->queued_count[FS_FREE],
					framemgr->queued_count[FS_REQUEST],
					framemgr->queued_count[FS_PROCESS],
					framemgr->queued_count[FS_COMPLETE],
					framemgr->queued_count[FS_USED]);
			ret = 0;
			goto err;
		}
	} else {
		vio_err("[S%d] frame(%d) is invalid state(%d)\n",
			group->instance, index, frame->state);
		ret = -EINVAL;
		goto err;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	if(group->leader == true)
		vio_group_start_trigger_mp(group, frame);

	if (pym_ctx->ctx_index == 0)
		pym->statistic.q_normal[pym_ctx->group->instance]++;
	vio_dbg("[S%d] %s index %d\n", group->instance,
		__func__, frameinfo->bufferindex);

	return ret;
try_releas_ion:
	// check if process's all frames are ok to free
	mp_frame = (struct mp_vio_frame *)frame;
	first_index = mp_frame->first_indx;
	frame_num = mp_frame->ion_bufffer_num;
	for (i = first_index; i < first_index+frame_num; i++) {
		release_frame = framemgr->frames_mp[i];
		if (release_frame) {
			mask |= release_frame->dispatch_mask;
			vio_dbg("pym index %d, mask:%x", i, release_frame->dispatch_mask);
		}
	}
	if (mask == 0x0000) {
		memcpy(frame_array_addr, &framemgr->frames_mp[first_index],
				sizeof(struct mp_vio_frame *)*frame_num);
		for (i = first_index; i < first_index+frame_num; i++) {
			mp_frame = (struct mp_vio_frame *)framemgr->frames_mp[i];
			if (!mp_frame)
				continue;
			list_del(&mp_frame->common_frame.list);
			framemgr->queued_count[mp_frame->common_frame.state]--;
			framemgr->frames_mp[i] = NULL;
			framemgr->index_state[i] = FRAME_IND_FREE;
			mp_frame->common_frame.dispatch_mask = 0;
		}

		framemgr->num_frames -= frame_num;

		if (framemgr->max_index == (first_index + frame_num)) {
			tmp_num = 0;
			for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
				if ((framemgr->index_state[i] == FRAME_IND_USING)
					|| (framemgr->index_state[i] == FRAME_IND_STREAMOFF))
					tmp_num++;
				if (tmp_num == framemgr->num_frames)
					break;
			}
			framemgr->max_index = i + 1;
		}
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	if (mask == 0x0000) {
		vio_info("pym release ion for proc%d", pym_ctx->ctx_index);
		x3_pym_try_release_process_ion(pym_ctx, frame_array_addr);
	}
	return ret;

err:
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	return ret;
}

int pym_video_dqbuf(struct pym_video_ctx *pym_ctx, struct frame_info *frameinfo)
{
	int ret = 0;
	struct list_head *done_list;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	unsigned long flags;
	u32 bufindex;
	u32 ctx_index;
	struct pym_subdev *subdev;
	struct vio_group *group;
	struct x3_pym_dev *pym;
	int cache_bufindex;
	struct vio_frame *cache_frame;

	framemgr = pym_ctx->framemgr;
	ctx_index = pym_ctx->ctx_index;
	subdev = pym_ctx->subdev;
	pym = pym_ctx->pym_dev;
	group = pym_ctx->group;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	#if 0
	if (pym_ctx->frm_num_usr > (int)pym_ctx->frm_num) {
		ret = -EFAULT;
		pym_ctx->event = 0;
		vio_err("[S%d] %s (p%d) dq too much frame(%d-%d).",
			pym_ctx->group->instance, __func__,
			pym_ctx->ctx_index, pym_ctx->frm_num_usr,
			pym_ctx->frm_num);
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		return ret;
	}
	#endif
	framemgr->ctx_mask |= (1 << pym_ctx->ctx_index);
	done_list = &framemgr->queued_list[FS_COMPLETE];
	if (!list_empty(done_list)) {
		frame = peek_frame(framemgr, FS_COMPLETE);
		if (frame) {
			// in this point, new frame is OK, we need:
			// if cached frame can be released to request;
			// if not, cached frame will be release in qbuf;
			// then cacned frame is update to the latest
			cache_bufindex = subdev->frameinfo.bufferindex;
			if (cache_bufindex >=0 && cache_bufindex < 128) {
				cache_frame = framemgr->frames_mp[cache_bufindex];
				if (cache_frame) {
					cache_frame->poll_mask = 0x00;
					if (cache_frame->dispatch_mask == 0x0000
							&& (framemgr->index_state[cache_bufindex] == FRAME_IND_USING)
							&& (cache_frame->frameinfo.addr[0] != 0)) {
						subdev->frameinfo.bufferindex = -1;
						cache_frame->dispatch_mask = 0xFF00;
						trans_frame(framemgr, cache_frame, FS_REQUEST);
						if(group->leader == true)
							vio_group_start_trigger_mp(group, frame);
						vio_dbg("pym dq trans to request%d", cache_bufindex);
					} else if (cache_frame->dispatch_mask == 0x0000
							&& (framemgr->index_state[cache_bufindex] == FRAME_IND_STREAMOFF)) {
						subdev->frameinfo.bufferindex = -1;
						vio_dbg("pym cache frame already been streamoff:%d", cache_bufindex);
					}
				}
			}
			memcpy(frameinfo, &frame->frameinfo,
				sizeof(struct frame_info));
			trans_frame(framemgr, frame, FS_USED);
			bufindex = frame->frameinfo.bufferindex;
			frame->dispatch_mask = 0x0000;
			frame->dispatch_mask |= (1 << pym_ctx->ctx_index);
			frame->poll_mask &= ~(1 << pym_ctx->ctx_index);
			/* copy frame_info to subdev*/
			if (atomic_read(&subdev->refcount) > 1) {
				memcpy(&subdev->frameinfo, &frame->frameinfo,
					sizeof(struct frame_info));
			}
			if (list_empty(done_list))
				pym_ctx->event = 0;
			pym_ctx->frm_num_usr++;
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			vio_dbg("[S%d] %s proc%d index%d frame%d from COMP,(%d %d %d %d %d)\n",
				pym_ctx->group->instance, __func__, ctx_index,
				frameinfo->bufferindex, frameinfo->frame_id,
				framemgr->queued_count[FS_FREE],
				framemgr->queued_count[FS_REQUEST],
				framemgr->queued_count[FS_PROCESS],
				framemgr->queued_count[FS_COMPLETE],
				framemgr->queued_count[FS_USED]);
			if (pym_ctx->ctx_index == 0)
				pym->statistic.dq_normal[pym_ctx->group->instance]++;
			return ret;
		}
	} else {
		if (atomic_read(&subdev->refcount) == 1) {
			ret = -EFAULT;
			pym_ctx->event = 0;
			vio_err("[S%d][V%d] %s (p%d) complete empty.",
				pym_ctx->group->instance, pym_ctx->id, __func__,
				pym_ctx->ctx_index);
			if (pym_ctx->ctx_index == 0)
				pym->statistic.dq_err[pym_ctx->group->instance]++;
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			return ret;
		}
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	/* copy frame_info from subdev */
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	if (pym_ctx->event == VIO_FRAME_DONE) {
		// if cached frame is exist, if so, this time get the cached frame
		cache_bufindex = subdev->frameinfo.bufferindex;
		if (cache_bufindex >= 0 && cache_bufindex < 128) {
			frame = framemgr->frames_mp[cache_bufindex];
			if (frame && (frame->state == FS_USED)) {
				memcpy(frameinfo, &subdev->frameinfo, sizeof(struct frame_info));
				frame->dispatch_mask |= (1 << pym_ctx->ctx_index);
				frame->poll_mask &= ~(1 << pym_ctx->ctx_index);
				pym_ctx->frm_num_usr++;
			} else {
				ret = -EAGAIN;
				vio_dbg("[S%d] %s proc%d frame been qback by others.\n",
					pym_ctx->group->instance, __func__, ctx_index);
				pym_ctx->event = 0;
				framemgr_x_barrier_irqr(framemgr, 0, flags);
				goto DONE;
			}
		} else {
			ret = -EAGAIN;
			vio_dbg("[S%d] %s proc%d cached index wrong %d.\n",
				pym_ctx->group->instance, __func__,
				ctx_index, cache_bufindex);
			pym_ctx->event = 0;
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			goto DONE;
		}
	} else {
		ret = -EFAULT;
		vio_err("[S%d] %s proc%d no frame, event %d.\n",
			pym_ctx->group->instance, __func__, ctx_index,
			pym_ctx->event);
		pym_ctx->event = 0;
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		goto DONE;
	}
	pym_ctx->event = 0;
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	vio_dbg("[S%d] %s proc%d index%d frame%d from subdev.\n",
		pym_ctx->group->instance, __func__, ctx_index,
		frameinfo->bufferindex, frameinfo->frame_id);
DONE:
	return ret;
}

int pym_video_getindex(struct pym_video_ctx *pym_ctx)
{
	u32 buf_index;

	if (!pym_ctx->group) {
		vio_err("[%s] group null", __func__);
		return -EFAULT;
	}
	if (!(pym_ctx->state & BIT(VIO_VIDEO_REBUFS))) {
		vio_err("[%s][V%02d]state error when get buf index(%lX)\n",
			__func__, pym_ctx->group->instance, pym_ctx->state);
		return -EINVAL;
	}

	buf_index = pym_ctx->frm_fst_ind;
	if (buf_index >= VIO_MP_MAX_FRAMES) {
		vio_err("[%s][V%02d]index too large(%d).\n",
			__func__, pym_ctx->group->instance, buf_index);
		return -EFAULT;
	}

	return buf_index;
}

void pym_video_user_stats(struct pym_video_ctx *pym_ctx,
	struct user_statistic *stats)
{
	struct x3_pym_dev *pym;
	struct vio_group *group;
	struct user_statistic *stats_in_drv;
	u32 instance;

	pym = pym_ctx->pym_dev;
	group = pym_ctx->group;
	if (!pym || !group) {
		vio_err("%s init err", __func__);
	}

	if (pym_ctx->ctx_index == 0) {
		instance = group->instance;
		stats_in_drv = &pym->statistic.user_stats[instance];
		stats_in_drv->cnt[USER_STATS_NORM_FRM] =
			stats->cnt[USER_STATS_NORM_FRM];
		stats_in_drv->cnt[USER_STATS_SEL_TMOUT] =
			stats->cnt[USER_STATS_SEL_TMOUT];
		stats_in_drv->cnt[USER_STATS_DROP] =
			stats->cnt[USER_STATS_DROP];
	}
}

int pym_update_scale_info(struct pym_video_ctx *pym_ctx, unsigned long arg)
{
	int ret = 0;
	unsigned long flags;
	struct vio_group *group;
	struct pym_subdev *subdev;
	pym_cfg_t *pym_config;
	pym_cfg_t pym_new_config;
	struct vio_framemgr *framemgr;

	group = pym_ctx->group;
	subdev = pym_ctx->subdev;
	framemgr = &subdev->framemgr;
	if (!group || !subdev || !framemgr) {
		vio_err("%s update_scale_info err", __func__);
		return -EINVAL;
	}

	pym_config = &subdev->pym_cfg;
	ret = copy_from_user((char *)&pym_new_config,
				(u32 __user *) arg, sizeof(pym_cfg_t));
	if (ret == 0) {
		framemgr_e_barrier_irqs(framemgr, 0, flags);
		subdev->update_all = pym_new_config.cfg_index;
		memcpy((char *)pym_config, &pym_new_config, sizeof(pym_cfg_t));
		framemgr_x_barrier_irqr(framemgr, 0, flags);
	}

	vio_dbg("[S%d]%s: request updata all cfg.\n",
		group->instance, __func__);
	return ret;
}

int pym_update_ch_scale_info(struct pym_video_ctx *pym_ctx, unsigned long arg)
{
	int ret = 0;
	struct vio_group *group;
	struct pym_subdev *subdev;
	pym_scale_ch_t *pym_config_ch;

	group = pym_ctx->group;
	subdev = pym_ctx->subdev;
	if (!group || !subdev) {
		vio_err("%s init err", __func__);
	}

	pym_config_ch = &subdev->pym_cfg_ch;
	ret = copy_from_user((char *)pym_config_ch,
				(u32 __user *) arg, sizeof(pym_scale_ch_t));
	if (ret)
		return -EFAULT;
	subdev->update_ch = 1;

	vio_dbg("[S%d]%s: ds%d f%d x-y(%d-%d) w-h(%d-%d) tgt(%d-%d).\n",
		group->instance, __func__,
		pym_config_ch->ch,
		pym_config_ch->ch_scale.factor,
		pym_config_ch->ch_scale.roi_x,
		pym_config_ch->ch_scale.roi_y,
		pym_config_ch->ch_scale.roi_width,
		pym_config_ch->ch_scale.roi_height,
		pym_config_ch->ch_scale.tgt_width,
		pym_config_ch->ch_scale.tgt_height);
	return ret;
}

static void print_pym_ion_addr(struct pym_video_ctx *pym_ctx,
		struct kernel_ion *ion_buffer)
{
	int i = 0, j = 0, k = 0;
	struct vio_framemgr *framemgr;
	struct mp_vio_frame *frame;

	framemgr = pym_ctx->framemgr;
	for (i = pym_ctx->frm_fst_ind;
		i < (pym_ctx->frm_fst_ind+ion_buffer->buf_num); i++) {
       frame = (struct mp_vio_frame *)framemgr->frames_mp[i];
	   k = i - pym_ctx->frm_fst_ind;
       // every plane will have a phy addr
       for (j = 0; j < ion_buffer->one[k].planecount; j++) {
		vio_dbg("frame index %d,plane %d,frame addr[0x%llx],buffer addr[0x%llx]",
			i, j, frame->addr[j], ion_buffer->one[k].paddr[j]);
       }
	}
}

static void print_user_buffer_cfg(struct pym_video_ctx *pym_ctx,
		struct kernel_ion *ion_buffer)
{
	int i = 0, j = 0;
	for (i = 0; i < ion_buffer->buf_num; i++) {
		for (j = 0; j < ion_buffer->one[i].planecount; j++) {
			vio_dbg("buffer %d, plane %d, size:%zu",
				i, j, ion_buffer->one[i].planeSize[j]);
		}
	}
}

int pym_alloc_ion_bufffer(struct pym_video_ctx *pym_ctx,
		struct kernel_ion *ion_buffer)
{
	int i = 0, j = 0, k = 0, m = 0;
	int ret = 0;
	struct vio_framemgr *framemgr;
	struct mp_vio_frame *frame;
	struct x3_pym_dev *pym;
	unsigned long flags;
	unsigned int ion_flag;

	ion_flag = ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC;
	ion_flag |= (pym_ctx->id + 7) << 16;

	if (!(pym_ctx->state & (BIT(VIO_VIDEO_S_INPUT) | BIT(VIO_VIDEO_REBUFS)))) {
	       vio_err("[%s] invalid PYM_IOC_KERNEL_ION is requested(%lX)",
		   			__func__, pym_ctx->state);
	       return -EINVAL;
	}

	if ((pym_ctx->frm_num != ion_buffer->buf_num) || (pym_ctx->frm_num <= 0)) {
	       vio_err("PYM_IOC_KERNEL_ION failed, buffer num not match");
	       return -EINVAL;
	}

	framemgr = pym_ctx->framemgr;
	pym = pym_ctx->pym_dev;

	print_user_buffer_cfg(pym_ctx, ion_buffer);

	// for all frame, allocated Y/UV ion buffer
	for (i = pym_ctx->frm_fst_ind;
		i < (pym_ctx->frm_fst_ind+pym_ctx->frm_num); i++) {
       frame = (struct mp_vio_frame *)framemgr->frames_mp[i];
	   k = i - pym_ctx->frm_fst_ind; // for ion_buffer index

       // every plane will have a phy addr
       for (j = 0; j < ion_buffer->one[k].planecount; j++) {
           frame->ion_handle[j] = ion_alloc(pym->ion_client,
			ion_buffer->one[k].planeSize[j], PAGE_SIZE,
			ION_HEAP_CARVEOUT_MASK, ion_flag);
           if (IS_ERR(frame->ion_handle[j])) {
                   vio_err("pym ion alloc failed failed");

                   // clean up current frame ion
                   for (m = j-1; m >= 0; m--)
                           ion_free(pym->ion_client, frame->ion_handle[m]);
                   goto ion_cleanup;
           }
           ret = ion_phys(pym->ion_client, frame->ion_handle[j]->id,
                           &ion_buffer->one[k].paddr[j],
                           &ion_buffer->one[k].planeSize[j]);
           if (ret) {
                   vio_err("pym ion get phys addr failed");
                   // clean up current frame ion
                   for (; j >= 0; j--)
                           ion_free(pym->ion_client, frame->ion_handle[j]);
                   goto ion_cleanup;
           }

           // set vio_frame addr the same as allocted ion phys
		   frame->addr[j] = ion_buffer->one[k].paddr[j];
       }
	}

	// every frame rembers which process it belongs to
	// and the firs_indx with buffer num,plane count
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	for (i = pym_ctx->frm_fst_ind;
			i < (pym_ctx->frm_fst_ind+pym_ctx->frm_num); i++) {
		frame = (struct mp_vio_frame *)framemgr->frames_mp[i];
		frame->first_indx = pym_ctx->frm_fst_ind;
		frame->ion_bufffer_num = pym_ctx->frm_num;
		frame->plane_count = ion_buffer->one[0].planecount;
		frame->proc_id = pym_ctx->ctx_index;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	set_bit(VIO_VIDEO_ION_ALLOC, &pym_ctx->state);
	print_pym_ion_addr(pym_ctx, ion_buffer);
	return 0;

ion_cleanup:
	// cleanup all the other frame data
	for (i = i-1; i >= (int)pym_ctx->frm_fst_ind; i--) {
	       frame = (struct mp_vio_frame *)framemgr->frames_mp[i];
		   k = i - pym_ctx->frm_fst_ind;
	       for (j = 0; j < ion_buffer->one[k].planecount; j++) {
	               ion_free(pym->ion_client, frame->ion_handle[j]);
	       }
	}
	return -ENOMEM;
}

static long x3_pym_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int ret = 0;
	int buffers = 0;
	int enable = 0;
	int instance = 0;
	int mp_share = 0;
	struct pym_video_ctx *pym_ctx;
	struct frame_info frameinfo;
	struct vio_group *group;
	int buf_index;
	struct user_statistic stats;
	struct kernel_ion *pym_ion = NULL;

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
		ret = pym_video_s_stream(pym_ctx, !!enable);
		break;
	case PYM_IOC_DQBUF:
		ret = pym_video_dqbuf(pym_ctx, &frameinfo);
		if (ret)
			return ret;
		ret = copy_to_user((void __user *) arg, (char *) &frameinfo,
				 sizeof(struct frame_info));
		if (ret)
			return -EFAULT;
		break;
	case PYM_IOC_QBUF:
		ret = copy_from_user((char *) &frameinfo, (u32 __user *) arg,
				   sizeof(struct frame_info));
		if (ret)
			return -EFAULT;
		ret = pym_video_qbuf(pym_ctx, &frameinfo);
		break;
	case PYM_IOC_REQBUFS:
		ret = get_user(buffers, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
		ret = pym_video_reqbufs(pym_ctx, buffers);
		break;
	case PYM_IOC_GET_INDEX:
		buf_index = pym_video_getindex(pym_ctx);
		if (buf_index < 0)
			return -EFAULT;
		ret = put_user(buf_index, (u32 __user *) arg);
		if (ret)
			return -EFAULT;
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
		ret = pym_bind_chain_group(pym_ctx, instance, &mp_share);
		if (ret)
			return -EFAULT;
		ret = put_user(mp_share, (u32 __user *)arg);
		if (ret)
			return -EFAULT;
		break;
	case PYM_IOC_USER_STATS:
		ret = copy_from_user((char *) &stats, (u32 __user *) arg,
				   sizeof(struct user_statistic));
		if (ret)
			return -EFAULT;
		pym_video_user_stats(pym_ctx, &stats);
		break;
	case PYM_IOC_SCALE_INFO:
		ret = pym_update_scale_info(pym_ctx, arg);
		break;
	case PYM_IOC_SCALE_INFO_CH:
		ret = pym_update_ch_scale_info(pym_ctx, arg);
		break;
	case PYM_IOC_KERNEL_ION:
		pym_ion = (struct kernel_ion *)vmalloc(sizeof(struct kernel_ion));
		if (!pym_ion) {
			vio_err("alloc pym ion struct faild");
			return -ENOMEM;
		}
		ret = copy_from_user(pym_ion, (u32 __user *) arg, sizeof(struct kernel_ion));
		if (ret) {
			vfree(pym_ion);
			return -EFAULT;
		}
		ret = pym_alloc_ion_bufffer(pym_ctx, pym_ion);
		if (ret) {
			vfree(pym_ion);
			vio_err("alloc ion buffer failed");
			return -EFAULT;
		}
		ret = copy_to_user((u32 __user *) arg, pym_ion, sizeof(struct kernel_ion));
		if (ret) {
			vfree(pym_ion);
			vio_err("copy to user failed");
			return -EFAULT;
		}
		vfree(pym_ion);
		break;
	default:
		vio_err("wrong ioctl command\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

int x3_pym_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret = 0;
	struct pym_video_ctx *pym_ctx;
	struct vio_framemgr *framemgr;
	struct mp_vio_frame *frame;
	int buffer_index;
	u32 paddr = 0;
	unsigned long flags;

	pym_ctx = file->private_data;
	if (!pym_ctx) {
		vio_err("%s pym_ctx is null.", __func__);
		ret = -EFAULT;
		goto err;
	}

	framemgr = pym_ctx->framemgr;
	buffer_index = vma->vm_pgoff;
	/*ret = pym_index_owner(pym_ctx, buffer_index);
	if( ret != VIO_BUFFER_OTHER ) {
		vio_err("[S%d][V%d] %s proc %d error,index %d not other's",
			pym_ctx->group->instance, pym_ctx->id, __func__,
			pym_ctx->ctx_index, buffer_index);
		ret = -EFAULT;
		goto err;
	}*/

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = (struct mp_vio_frame *)framemgr->frames_mp[buffer_index];
	if (frame) {
		paddr = frame->addr[0];
	} else {
		vio_err("[S%d][V%d] %s proc %d error,frame null",
			pym_ctx->group->instance, pym_ctx->id, __func__,
			pym_ctx->ctx_index);
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		ret = -EFAULT;
		goto err;
	}
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	if (paddr == 0) {
		vio_err("[S%d][V%d] %s proc %d error,paddr %x.",
			pym_ctx->group->instance, pym_ctx->id, __func__,
			pym_ctx->ctx_index, paddr);
		ret = -EAGAIN;
		goto err;
	}

	vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP;
	if (remap_pfn_range(vma, vma->vm_start, paddr >> PAGE_SHIFT,
		vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		vio_err("[S%d][V%d] %s proc %d error,remap.",
			pym_ctx->group->instance, pym_ctx->id, __func__,
			pym_ctx->ctx_index);
		ret = -EAGAIN;
		goto err;
	}

	vio_dbg("[S%d][V%d] %s map success, proc %d index %d paddr %x.",
		pym_ctx->group->instance, pym_ctx->id, __func__,
		pym_ctx->ctx_index, buffer_index, paddr);
	return 0;
err:
	return ret;
}

void pym_set_iar_output(struct pym_subdev *subdev, struct vio_frame *frame)
{
#ifdef X3_IAR_INTERFACE
	struct special_buffer *spec;
	struct vio_group *group;
	u8 dis_instance[2] = {0, 0};
	u8 display_layer[2] ={0, 0};
	int ret = 0;
	int i = 0;

	if (iar_get_type == NULL || iar_set_addr == NULL)
		return;
	ret = iar_get_type(dis_instance, display_layer);
	spec = &frame->frameinfo.spec;
	group = subdev->group;
	if (!ret) {
		for (i = 0; i < 2; i++) {
			if (group->instance == dis_instance[i] && display_layer[i] < 37) {
				if (display_layer[i] >= 31)
					iar_set_addr(i, spec->us_y_addr[display_layer[i] - 31],
						spec->us_uv_addr[display_layer[i] - 31]);
				else if (display_layer[i] >= 7)
					iar_set_addr(i, spec->ds_y_addr[display_layer[i] - 7],
						spec->ds_uv_addr[display_layer[i] - 7]);
			}
			vio_dbg("[D%d]PYM display_layer = %d, dis_instance = %d", i,
				display_layer[i], dis_instance[0]);
		}
	}

#endif
}

void pym_frame_done(struct pym_subdev *subdev)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	struct pym_video_ctx *pym_ctx;
	struct x3_pym_dev *pym;
	unsigned long flags;
	u32 event = 0;
	int i = 0;
	int cache_bufindex;
	struct vio_frame *cache_frame;
	struct vio_frame_id frmid;
	unsigned long poll_mask = 0;
	struct timeval tmp_tv;

	group = subdev->group;
	pym = subdev->pym_dev;
	framemgr = &subdev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
		if(subdev->pym_cfg.img_scr) {
			vio_get_ipu_frame_info(group->instance, &frmid);
			vio_dbg("[S%d] pym online frame_id %d",
				group->instance, frmid.frame_id);
			frame->frameinfo.frame_id = frmid.frame_id;
			frame->frameinfo.timestamps = frmid.timestamps;
			frame->frameinfo.tv = frmid.tv;
		}
		vio_dbg("[S%d]pym done fid %d timestamps %llu",
			group->instance,
			frame->frameinfo.frame_id,
			frame->frameinfo.timestamps);
		vio_set_stat_info(group->instance, PYM_FE, group->frameid.frame_id);
		do_gettimeofday(&tmp_tv);
		g_pym_idx[group->instance]++;
		if (tmp_tv.tv_sec > g_pym_fps_lasttime[group->instance]) {
			g_pym_fps[group->instance] = g_pym_idx[group->instance];
			g_pym_fps_lasttime[group->instance] = tmp_tv.tv_sec;
			g_pym_idx[group->instance] = 0;
		}

		pym_set_iar_output(subdev, frame);
		event = VIO_FRAME_DONE;
		trans_frame(framemgr, frame, FS_COMPLETE);
		frame->poll_mask = subdev->poll_mask;
		poll_mask = subdev->poll_mask;
		subdev->poll_mask = 0x00;
		pym->statistic.fe_normal[group->instance]++;

		// check if cache frame is release by others process
		// if so, tran cache frame to request for hardware use
		cache_bufindex = subdev->frameinfo.bufferindex;
		if (cache_bufindex >=0 && cache_bufindex < 128) {
			cache_frame = framemgr->frames_mp[cache_bufindex];
			if (cache_frame) {
				cache_frame->poll_mask = 0x00;
				if (cache_frame->dispatch_mask == 0x0000
						&& (framemgr->index_state[cache_bufindex] == FRAME_IND_USING)
						&& (cache_frame->frameinfo.addr[0] != 0)) {
					subdev->frameinfo.bufferindex = -1;
					cache_frame->dispatch_mask = 0xFF00;
					trans_frame(framemgr, cache_frame, FS_REQUEST);
					if(group->leader == true)
						vio_group_start_trigger_mp(group, frame);
				}
			}
		}
	} else {
		pym->statistic.fe_lack_buf[group->instance]++;
		event = VIO_FRAME_NDONE;
		vio_err("[S%d]PYM PROCESS queue has no member;\n", group->instance);
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

	/*
	 *pym us framedrop isr and done isr will be appear at the same frame
	 *pym_frame_ndone will called after framedrop
	 *check process frame to skip wakeup twice
	 */
	if (frame) {
		spin_lock_irqsave(&subdev->slock, flags);
		/*
		 * bit 1 represent main process
		 * always set event and wakeup main process
		 */
		poll_mask |= 0x01;
		vio_dbg("pym done subdev ctx mask:%lu", subdev->val_ctx_mask);
		for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
			/*
			 * wake up sub process when already poll
			 */
			if (test_bit(i, &subdev->val_ctx_mask) &&
					test_bit(i, &poll_mask)) {
				pym_ctx = subdev->ctx[i];
				if (pym_ctx) {
					pym_ctx->event = event;
					wake_up(&pym_ctx->done_wq);
				}
			}
		}
		spin_unlock_irqrestore(&subdev->slock, flags);
	}
}

void pym_frame_ndone(struct pym_subdev *subdev)
{
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_group *group;
	struct pym_video_ctx *pym_ctx;
	unsigned long flags;
	int i = 0;

	group = subdev->group;
	framemgr = &subdev->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame) {
        vio_dbg("ndone bidx%d fid%d, proc->req.",
               frame->frameinfo.bufferindex,
               frame->frameinfo.frame_id);
		trans_frame(framemgr, frame, FS_REQUEST);
		if(group->leader == true)
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
	vio_dbg("pym ndone subdev ctx mask:%lu", subdev->val_ctx_mask);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if (test_bit(i, &subdev->val_ctx_mask)) {
			pym_ctx = subdev->ctx[i];
			if (pym_ctx) {
				pym_ctx->event = VIO_FRAME_DONE;
				wake_up(&pym_ctx->done_wq);
			}
		}
	}
	spin_unlock_irqrestore(&subdev->slock, flags);
}

static irqreturn_t pym_isr(int irq, void *data)
{
	u32 status;
	u32 instance;
	bool drop_flag = 0;
	struct x3_pym_dev *pym;
	struct vio_group *group;
	struct vio_group_task *gtask;
	struct pym_subdev *subdev;

	pym = data;
	gtask = &pym->gtask;
	instance = atomic_read(&pym->instance);
	group = pym->group[instance];
	subdev = group->sub_ctx[GROUP_ID_SRC];
	pym_get_intr_status(pym->base_reg, &status, true);
	vio_dbg("%s:status = 0x%x\n", __func__, status);

	if (status & (1 << INTR_PYM_DS_FRAME_DROP)) {
		vio_err("[%d]DS drop frame\n", instance);
		drop_flag = true;
		if (subdev->disable_flag)
			pym->statistic.soft_frame_drop_ds[instance]++;
		else
			pym->statistic.hard_frame_drop_ds[instance]++;
	}

	if (status & (1 << INTR_PYM_US_FRAME_DROP)) {
		vio_err("[%d]US drop frame\n", instance);
		drop_flag = true;
		if (subdev->disable_flag)
			pym->statistic.soft_frame_drop_us[instance]++;
		else
			pym->statistic.hard_frame_drop_us[instance]++;
	}

	if (status & (1 << INTR_PYM_FRAME_DONE)) {
		if (!group->leader)
			vio_group_done(group);

		if (test_bit(PYM_DMA_INPUT, &pym->state)) {
			vio_group_done(group);
		}
		pym_frame_done(subdev);
	}

	if (status & (1 << INTR_PYM_FRAME_START)) {
		pym->statistic.fs[instance]++;
		pym->statistic.tal_fs++;
		pym->statistic.grp_tsk_left[instance]
			= atomic_read(&group->rcount);
		pym->statistic.tal_frm_work = atomic_read(&pym->backup_fcount);
		atomic_inc(&pym->sensor_fcount);

		if(atomic_read(&pym->enable_cnt)) {
			atomic_dec(&pym->enable_cnt);
			pym_disable_layer(subdev);
		}

		if (test_bit(PYM_OTF_INPUT, &pym->state)
				&& group->leader) {
			if (unlikely(list_empty(&gtask->hw_resource.wait_list)) &&
					gtask->hw_resource.count >= 1) {
				vio_err("[S%d]GP%d(res %d, rcnt %d, bcnt %d, scnt %d)\n",
					group->instance,
					gtask->id,
					gtask->hw_resource.count,
					atomic_read(&group->rcount),
					atomic_read(&pym->backup_fcount),
					atomic_read(&pym->sensor_fcount));
				pym->statistic.fs_lack_task[instance]++;
			} else {
				up(&gtask->hw_resource);
			}
		}
		vio_set_stat_info(group->instance, PYM_FS, group->frameid.frame_id);
	}

	if (drop_flag) {
		if (!group->leader || test_bit(PYM_DMA_INPUT, &pym->state))
			vio_group_done(group);
		pym_frame_ndone(subdev);
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
	.mmap = x3_pym_mmap,
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

int pym_subdev_init(struct pym_subdev *subdev)
{
	int ret = 0;

	spin_lock_init(&subdev->slock);
	atomic_set(&subdev->refcount, 0);
	subdev->frameinfo.bufferindex = -1;
	subdev->poll_mask = 0x00;

	return ret;
}

int x3_pym_subdev_init(struct x3_pym_dev *pym)
{
	int i = 0, j = 0;
	int ret = 0;
	struct pym_subdev *subdev;

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		subdev = &pym->subdev[i];
		ret = pym_subdev_init(subdev);
	}

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		for (j = 0; j < VIO_MP_MAX_FRAMES; j++) {
			frame_work_init(&pym->vwork[i][j].work);
		}
	}

	return ret;
}

int x3_pym_device_node_init(struct x3_pym_dev *pym)
{
	int ret = 0;
	struct device *dev = NULL;

	ret = alloc_chrdev_region(&pym->devno, 0, MAX_DEVICE, "x3_pym");
	if (ret < 0) {
		vio_err("Error %d while alloc chrdev pym", ret);
		goto err_req_cdev;
	}

	cdev_init(&pym->cdev, &x3_pym_fops);
	pym->cdev.owner = THIS_MODULE;
	ret = cdev_add(&pym->cdev, pym->devno, MAX_DEVICE);
	if (ret) {
		vio_err("Error %d while adding x2 pym cdev", ret);
		goto err;
	}

	if (vps_class)
		pym->class = vps_class;
	else
		pym->class = class_create(THIS_MODULE, X3_PYM_NAME);

	dev = device_create(pym->class, NULL, MKDEV(MAJOR(pym->devno), 0),
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
	unregister_chrdev_region(pym->devno, MAX_DEVICE);
	return ret;
}

static ssize_t pym_reg_dump(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_pym_dev *pym;

	pym = dev_get_drvdata(dev);

	vio_clk_enable("pym_mclk");
	pym_hw_dump(pym->base_reg);
	vio_clk_disable("pym_mclk");

	return 0;
}

static DEVICE_ATTR(regdump, 0444, pym_reg_dump, NULL);


static ssize_t pym_stat_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct x3_pym_dev *pym;
	u32 offset = 0;
	int instance = 0;
	ssize_t len = 0;
	int output = 0;
	struct user_statistic *stats;

	pym = dev_get_drvdata(dev);
	for(instance = 0; instance < VIO_MAX_STREAM; instance++) {
		if (!pym->statistic.err_stat_enable[instance])
			continue;
		output = 1;
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"*******S%d info:******\n",
			instance);
		offset += len;

		stats = &pym->statistic.user_stats[instance];
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"USER: normal %d, sel tout %d, "
			"drop %d, dq fail %d, sel err%d\n",
			stats->cnt[USER_STATS_NORM_FRM],
			stats->cnt[USER_STATS_SEL_TMOUT],
			stats->cnt[USER_STATS_DROP],
			stats->cnt[USER_STATS_DQ_FAIL],
			stats->cnt[USER_STATS_SEL_ERR]);
		offset += len;

		len = snprintf(&buf[offset],  PAGE_SIZE - offset,
			"DRV: fe_normal %d, fe_lack_buf %d, "
			"pollin_comp %d, pollin_isr %d, pollerr %d, "
			"dq_normal %d, dq_err %d, "
			"q_normal %d, "
			"hard_frame_drop_us %d, hard_frame_drop_ds %d\n"
			"soft_frame_drop_us %d, soft_frame_drop_ds %d\n",
			pym->statistic.fe_normal[instance],
			pym->statistic.fe_lack_buf[instance],
			pym->statistic.pollin_comp[instance],
			pym->statistic.pollin_fe[instance],
			pym->statistic.pollerr[instance],
			pym->statistic.dq_normal[instance],
			pym->statistic.dq_err[instance],
			pym->statistic.q_normal[instance],
			pym->statistic.hard_frame_drop_us[instance],
			pym->statistic.hard_frame_drop_ds[instance],
			pym->statistic.soft_frame_drop_us[instance],
			pym->statistic.soft_frame_drop_ds[instance]);
		offset += len;

		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"DRV: fs %d, grp_tsk_left %d, fs_lack_task %d\n",
			pym->statistic.fs[instance],
			pym->statistic.grp_tsk_left[instance],
			pym->statistic.fs_lack_task[instance]);
		offset += len;
	}

	if (output == 1) {
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"DRV: tatal_fs %d, tatal_frm_work %d\n",
			pym->statistic.tal_fs,
			pym->statistic.tal_frm_work);
		offset += len;
	}

	return offset;
}

static ssize_t pym_stat_store(struct device *dev,
					struct device_attribute *attr,
					const char *page, size_t len)
{
	struct x3_pym_dev *pym;

	pym = dev_get_drvdata(dev);
	if (pym) {
		//memset(&pym->statistic, 0, sizeof(pym->statistic));
		memset(&pym->statistic.fe_normal, 0, sizeof(pym->statistic.fe_normal));
		memset(&pym->statistic.fe_normal, 0, sizeof(pym->statistic.fe_normal));
		memset(&pym->statistic.fe_lack_buf, 0, sizeof(pym->statistic.fe_lack_buf));
		memset(&pym->statistic.hard_frame_drop_us, 0, sizeof(pym->statistic.hard_frame_drop_us));
		memset(&pym->statistic.hard_frame_drop_ds, 0, sizeof(pym->statistic.hard_frame_drop_ds));
		memset(&pym->statistic.soft_frame_drop_us, 0, sizeof(pym->statistic.soft_frame_drop_us));
		memset(&pym->statistic.soft_frame_drop_ds, 0, sizeof(pym->statistic.soft_frame_drop_ds));
		memset(&pym->statistic.fs_lack_task, 0, sizeof(pym->statistic.fs_lack_task));
		memset(&pym->statistic.dq_normal, 0, sizeof(pym->statistic.dq_normal));
		memset(&pym->statistic.dq_err, 0, sizeof(pym->statistic.dq_err));
		memset(&pym->statistic.pollin_fe, 0, sizeof(pym->statistic.pollin_fe));
		memset(&pym->statistic.pollin_comp, 0, sizeof(pym->statistic.pollin_comp));
		memset(&pym->statistic.pollerr, 0, sizeof(pym->statistic.pollerr));
		memset(&pym->statistic.q_normal, 0, sizeof(pym->statistic.q_normal));
		memset(&pym->statistic.fs, 0, sizeof(pym->statistic.fs));
		memset(&pym->statistic.grp_tsk_left, 0, sizeof(pym->statistic.grp_tsk_left));
		memset(&pym->statistic.tal_fs, 0, sizeof(pym->statistic.tal_fs));
		memset(&pym->statistic.tal_frm_work, 0, sizeof(pym->statistic.tal_frm_work));
		memset(&pym->statistic.user_stats, 0, sizeof(pym->statistic.user_stats));
		memset(&pym->statistic.err_stat_enable, 0, sizeof(pym->statistic.err_stat_enable));
	}
	return len;
}
static DEVICE_ATTR(err_status, S_IRUGO|S_IWUSR, pym_stat_show, pym_stat_store);

static ssize_t pym_fps_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	u32 offset = 0;
	int i, len;

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		if (g_pym_fps[i] == 0) continue;
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
				"pym pipe %d: output fps %d\n", i, g_pym_fps[i]);
		offset += len;
		g_pym_fps[i] = 0;
	}
	return offset;
}
static DEVICE_ATTR(fps, S_IRUGO, pym_fps_show, NULL);

static ssize_t enabled_pipeline_show(struct device *dev,
					struct device_attribute *attr, char* buf)
{
	struct x3_pym_dev *pym;
	u32 offset = 0, len = 0;
	int enabled_pipe_num = 0;
	int i = 0;
	pym = dev_get_drvdata(dev);

	len = snprintf(buf, PAGE_SIZE - offset, "enable pipe index:");
	offset += len;
	for (i = 0; i < VIO_MAX_STREAM; i++) {
		if (pym->statistic.enable[i]) {
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
	struct x3_pym_dev *pym;
	pym_cfg_t *pym_config;
	struct vio_framemgr *framemgr;
	unsigned long flags;
	int input_type;
	int i = 0;
	u32 offset = 0, len = 0;
	pym = dev_get_drvdata(dev);

	if (pym->statistic.enable[pipeid] == 0) {
		len = snprintf(buf+offset, PAGE_SIZE - offset, "pipeline %d is disabled\n", pipeid);
		offset += len;
		goto done_return;
	}

	if (pym->statistic.enable[0] == 0) {
		len = snprintf(buf+offset, PAGE_SIZE - offset, "pipeline %d is disabled\n", pipeid);
		offset += len;
	} else {
		framemgr = &pym->subdev[i].framemgr;
		len = snprintf(buf+offset, PAGE_SIZE - offset, "pipeline %d queue:\n", pipeid);
		offset += len;
		framemgr_e_barrier_irqs(framemgr, 0, flags);
		len = snprintf(buf+offset, PAGE_SIZE - offset,
				"pipeline %d queue(free:%d request:%d process:%d complete:%d used:%d)\n",
				i,
				framemgr->queued_count[FS_FREE],
				framemgr->queued_count[FS_REQUEST],
				framemgr->queued_count[FS_PROCESS],
				framemgr->queued_count[FS_COMPLETE],
				framemgr->queued_count[FS_USED]);
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		offset += len;

		len = snprintf(buf+offset, PAGE_SIZE - offset, "pipeline %d pym config:\n", pipeid);
		offset += len;

		input_type = pym->subdev[pipeid].pym_cfg.img_scr;
		if (input_type == 0) {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "input mode: %d, %s\n", input_type, "ddr to pym");
		} else if (input_type == 1) {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "input mode: %d, %s\n", input_type, "ipu online to pym");
		} else {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "input mode: %d, %s\n", input_type, "wrong");
		}
		offset += len;

		len = snprintf(buf+offset, PAGE_SIZE - offset, "channel config:\n");
		offset += len;

		pym_config = &pym->subdev[pipeid].pym_cfg;
		for (i = 0; i < MAX_PYM_DS_COUNT; i++) {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "\tds%d \tfactor:%d, \ttgt wxh:%dx%d, \tstartXY:%d-%d\n",
				i, pym_config->stds_box[i].factor, pym_config->stds_box[i].tgt_width, pym_config->stds_box[i].tgt_height,
				pym_config->stds_box[i].roi_x, pym_config->stds_box[i].roi_y);
			offset += len;
		}

		for (i = 0; i < MAX_PYM_US_COUNT; i++) {
			len = snprintf(buf+offset, PAGE_SIZE - offset, "\tus%d \tfactor:%d, \ttgt wxh:%dx%d, \tstartXY:%d-%d\n",
				i, pym_config->stus_box[i].factor, pym_config->stus_box[i].tgt_width, pym_config->stus_box[i].tgt_height,
				pym_config->stus_box[i].roi_x, pym_config->stus_box[i].roi_y);
			offset += len;
		}
	}

done_return:
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


static struct attribute *pym_info_attrs[] = {
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

static struct attribute_group pym_info_group = {
	.attrs	= pym_info_attrs,
	.name	= "info",
};

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

	irq_set_affinity_hint(pym->irq, get_cpu_mask(VIO_IRQ_CPU_IDX));

	pym->ion_client = ion_client_create(hb_ion_dev, "pym_driver_ion");
	if (IS_ERR(pym->ion_client)) {
			vio_err("pym ion client create failed.");
			goto err_get_irq;
	}

	x3_pym_device_node_init(pym);

	dev = &pdev->dev;
	ret = device_create_file(dev, &dev_attr_regdump);
	if (ret < 0) {
		vio_err("create regdump failed (%d)\n", ret);
		goto p_err;
	}

	ret = device_create_file(dev, &dev_attr_err_status);
	if (ret < 0) {
		vio_err("create err_status failed (%d)\n", ret);
		goto p_err;
	}

	ret = device_create_file(dev, &dev_attr_fps);
	if (ret < 0) {
		vio_err("create fps failed (%d)\n", ret);
		goto p_err;
	}

	platform_set_drvdata(pdev, pym);

	// create sysfs node for pym info
	ret = sysfs_create_group(&dev->kobj, &pym_info_group);
	if (ret) {
		vio_err("create pym info group fail");
		goto p_err;
	}

	spin_lock_init(&pym->shared_slock);
	sema_init(&pym->gtask.hw_resource, 1);
	atomic_set(&pym->gtask.refcount, 0);
	atomic_set(&pym->rsccount, 0);
	atomic_set(&pym->open_cnt, 0);
	mutex_init(&pym_mutex);

	x3_pym_subdev_init(pym);
	vio_group_init_mp(GROUP_ID_PYM);

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
	struct x3_pym_dev *pym;

	BUG_ON(!pdev);

	pym = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_regdump);
	device_remove_file(&pdev->dev, &dev_attr_err_status);
	device_remove_file(&pdev->dev, &dev_attr_fps);
	sysfs_remove_group(&pdev->dev.kobj, &pym_info_group);
	ion_client_destroy(pym->ion_client);

	free_irq(pym->irq, pym);

	device_destroy(pym->class, pym->devno);
	if (!vps_class)
		class_destroy(pym->class);
	cdev_del(&pym->cdev);
	unregister_chrdev_region(pym->devno, MAX_DEVICE);
	kfree(pym);

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
MODULE_LICENSE("GPL v2");
