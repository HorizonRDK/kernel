/*
 *    driver, v4l2 stream interface
 *
 *    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <asm/div64.h>
#include <linux/sched.h>

#include <linux/videodev2.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include "acamera_logger.h"
#include "acamera_dwe_config.h"
#include "buffer_v4l2_stream.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

static dwe_v4l2_stream_t *dwe_buf[FIRMWARE_CONTEXT_NUMBER];

int dwe_stream_get_frame(uint32_t ctx_id, dframe_t *dframes)
{
	int rc = 0;
	dwe_v4l2_stream_t *pstream = NULL;
	dwe_v4l2_buffer_t *pbuf = NULL;
	struct v4l2_format *v4l2_fmt;
	struct vb2_buffer *vb;

	LOG(LOG_DEBUG, " ctx_id = %d!\n", ctx_id);
	if ((ctx_id >= FIRMWARE_CONTEXT_NUMBER) || (dwe_buf[ctx_id] == NULL)
		|| (dframes == NULL)) {
		pr_debug("dwe_buf[%d] is err !\n", ctx_id); /*PRQA S 3238,0685 ++*/
		return -1;
	}

	pstream = dwe_buf[ctx_id];
	v4l2_fmt = &pstream->cur_v4l2_fmt;
	/* try to get an active buffer from vb2 queue  */
	spin_lock(&pstream->slock);
	if (!list_empty(&pstream->stream_buffer_list)) {
		pbuf = list_entry(pstream->stream_buffer_list.next, /*PRQA S 2880,0497,2810,2992 ++*/
			dwe_v4l2_buffer_t, list);
		list_del(&pbuf->list);
		list_add_tail(&pbuf->list, &pstream->stream_buffer_list_busy);
	}
	spin_unlock(&pstream->slock);

	if (!pbuf) {
		pr_debug("pbuf is null !\n"); /*PRQA S 3238,0685 ++*/
		return -1;
	}

	vb = &pbuf->vb;

	dframes->type = vb->type;  // frame type.
	dframes->virt_addr = vb2_plane_vaddr(vb, 0);
#if (BUFFER_DMA)
	pr_debug("virt_addr%p, addr memory %p !\n",
		dframes->virt_addr, vb2_plane_cookie(vb, 0));
	dframes->address = *((uint32_t *)vb2_plane_cookie(vb, 0));
#else
	dframes->address = (uint32_t)__virt_to_phys(dframes->virt_addr);
#endif
	pr_debug(" type %d -- addr memory %d !\n", vb->type, dframes->address);
	return rc;
}
EXPORT_SYMBOL(dwe_stream_get_frame);

int dwe_stream_put_frame(uint32_t ctx_id, dframe_t *dframes)
{
	dwe_v4l2_stream_t *pstream = NULL;
	dwe_v4l2_buffer_t *pbuf = NULL;
	struct vb2_buffer *vb;

	if ((ctx_id >= FIRMWARE_CONTEXT_NUMBER) || (dwe_buf[ctx_id] == NULL)
		|| (dframes == NULL)) {
		pr_debug("dwe_buf[%d] is err !\n", ctx_id);
		return -1;
	}

	LOG(LOG_DEBUG, "ctx_id = %d!\n", ctx_id);

	pstream = dwe_buf[ctx_id];

	/* try to get an active buffer from vb2 queue  */
	spin_lock(&pstream->slock);
	if (!list_empty(&pstream->stream_buffer_list_busy)) {
		pbuf = list_entry(pstream->stream_buffer_list_busy.next,
			dwe_v4l2_buffer_t, list);
		list_del(&pbuf->list);
	}
	spin_unlock(&pstream->slock);

	if (!pbuf) {
		pr_debug("pbuf is null !\n");
		return -1;
	}

	vb = &pbuf->vb;
	vb->timestamp = ktime_get_ns();
	//*(uint64_t *)dframes->virt_addr = vb->timestamp;

	pr_debug(" timestamp %lld!\n", vb->timestamp);
	/* Put buffer back to vb2 queue */
	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

	return 0;
}
EXPORT_SYMBOL(dwe_stream_put_frame);

int dwe_v4l2_stream_init(dwe_v4l2_stream_t **ppstream, int ctx_id)
{
	dwe_v4l2_stream_t *new_stream = NULL;

	LOG(LOG_DEBUG, "ctx_id = %d!\n", ctx_id);
	/* allocate dwe_v4l2_stream_t */
	new_stream = kzalloc(sizeof(dwe_v4l2_stream_t), GFP_KERNEL);
	if (new_stream == NULL) {
		return -ENOMEM;
	}

	//new_stream->cur_v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	new_stream->cur_v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	new_stream->cur_v4l2_fmt.fmt.pix.width = 1920;
	new_stream->cur_v4l2_fmt.fmt.pix.height = 1080;
	new_stream->cur_v4l2_fmt.fmt.pix.sizeimage = (1920 + 1080) * 4;

	/* init control fields */
	new_stream->ctx_id = ctx_id;
	new_stream->stream_started = 0;
	new_stream->last_frame_id = 0xFFFFFFFF;

	/* init list */
	INIT_LIST_HEAD(&new_stream->stream_buffer_list);
	INIT_LIST_HEAD(&new_stream->stream_buffer_list_busy);

	/* init locks */
	spin_lock_init(&new_stream->slock);

	/* return stream private ptr to caller */
	*ppstream = new_stream;

	dwe_buf[ctx_id] = new_stream;
	return 0;
}

void dwe_v4l2_stream_deinit(dwe_v4l2_stream_t *pstream)
{
	if (!pstream) {
		LOG(LOG_ERR, "Null stream passed");
		return;
	}

	/* do stream-off first if it's on */
	dwe_v4l2_stream_off(pstream);
	dwe_buf[pstream->ctx_id] = NULL;
	/* release fw_info */
	if (pstream) { /*PRQA S 2991 ++*/
		kzfree(pstream);
		pstream = NULL;
	}
}

static void dwe_v4l2_stream_buffer_list_release(dwe_v4l2_stream_t *pstream,
				struct list_head *stream_buffer_list)
{
	dwe_v4l2_buffer_t *buf;
	struct vb2_buffer *vb;

	while (!list_empty(stream_buffer_list)) {
		buf = list_entry(stream_buffer_list->next,
			dwe_v4l2_buffer_t, list);
		list_del(&buf->list);

		vb = &buf->vb;

		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);

		LOG(LOG_INFO, "vid_cap buffer %d done", pstream->ctx_id);
	}
}

int dwe_v4l2_stream_on(dwe_v4l2_stream_t *pstream)
{
	if (!pstream) {
		LOG(LOG_ERR, "Null stream passed");
		return -EINVAL;
	}

	/* Resets frame counters */
	pstream->fw_frame_seq_count = 0;

	/* control fields update */
	pstream->stream_started = 1;

	return 0;
}

void dwe_v4l2_stream_off(dwe_v4l2_stream_t *pstream)
{
	if (!pstream) {
		LOG(LOG_ERR, "Null stream passed");
		return;
	}

	// control fields update
	pstream->stream_started = 0;

	/* Release all active buffers */
	spin_lock(&pstream->slock);
	dwe_v4l2_stream_buffer_list_release(pstream,
		&pstream->stream_buffer_list);
	dwe_v4l2_stream_buffer_list_release(pstream,
		&pstream->stream_buffer_list_busy);
	spin_unlock(&pstream->slock);
}
