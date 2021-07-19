/*
 *    driver, vb2_buffer interface
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

#include <linux/dma-mapping.h>
#include <linux/videodev2.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>
#include "acamera_logger.h"
#include "acamera_dwe_config.h"
#include "buffer_v4l2_stream.h"
#include "buffer_vb2.h"

#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

/* ----------------------------------------------------------------
 * VB2 operations
 */
static int dwe_vb2_queue_setup(struct vb2_queue *vq,
			unsigned int *nbuffers, unsigned int *nplanes,
			unsigned int sizes[], struct device *alloc_devs[])
{
	dwe_v4l2_stream_t *pstream = vb2_get_drv_priv(vq);
	struct v4l2_format vfmt;
	//unsigned int size;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	LOG(LOG_INFO, "vq: %p, *nplanes: %u.", vq, *nplanes);
	LOG(LOG_INFO, "vq->num_buffers: %u vq->type:%u.", vq->num_buffers, vq->type);

	memcpy(&vfmt, &pstream->cur_v4l2_fmt, sizeof(struct v4l2_format));

	if (vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		*nplanes = 1;
		sizes[0] = vfmt.fmt.pix.sizeimage;
		LOG(LOG_INFO, "nplanes: %u, size: %u", *nplanes, sizes[0]);
	} else if (vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		LOG(LOG_INFO, "nplanes: %u", *nplanes);
	} else {
		LOG(LOG_ERR, "Wrong type: %u", vfmt.type);
		return -EINVAL;
	}

	/*
	 * videobuf2-vmalloc allocator is context-less so no need to set
	 * alloc_ctxs array.
	 */

	LOG(LOG_INFO, "nbuffers: %u, nplanes: %u", *nbuffers, *nplanes);

	return 0;
}

static int dwe_vb2_buf_prepare(struct vb2_buffer *vb)
{
	unsigned long size;
	//dwe_v4l2_stream_t *pstream = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_format vfmt;
	int i;

	// get current format
	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	memset(&vfmt, 0, sizeof(struct v4l2_format));
	if (vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		size = vfmt.fmt.pix.sizeimage;

		if (vb2_plane_size(vb, 0) < size) {
			LOG(LOG_ERR, "buffer too small (%lu < %lu)", vb2_plane_size(vb, 0), size);
			return -EINVAL;
		}

		vb2_set_plane_payload(vb, 0, size);
		LOG(LOG_INFO, "single plane payload set %d", size);
	} else if (vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		for (i = 0; i < vfmt.fmt.pix_mp.num_planes; i++) {
			size = vfmt.fmt.pix_mp.plane_fmt[i].sizeimage;
			if (vb2_plane_size(vb, i) < size) {
				LOG(LOG_ERR, "i:%d buffer too small (%lu < %lu)",
					i, vb2_plane_size(vb, 0), size);
				return -EINVAL;
			}
			vb2_set_plane_payload(vb, i, size);
			LOG(LOG_INFO, "i:%d payload set %d", i, size);
		}
	}

	return 0;
}

static void dwe_vb2_buf_queue(struct vb2_buffer *vb)
{
	if (vb == NULL) {
		LOG(LOG_ERR, "vb ptr is null");
		return;
	}
	dwe_v4l2_stream_t *pstream = vb2_get_drv_priv(vb->vb2_queue);

	dwe_v4l2_buffer_t *buf = container_of(vb, dwe_v4l2_buffer_t, vb); /*PRQA S 0497,2992,2810,2880 ++*/
	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	spin_lock(&pstream->slock);
	list_add_tail(&buf->list, &pstream->stream_buffer_list);
	spin_unlock(&pstream->slock);
}

static const struct vb2_ops dwe_vb2_ops = {
	.queue_setup = dwe_vb2_queue_setup, // called from VIDIOC_REQBUFS
	.buf_prepare = dwe_vb2_buf_prepare,
	.buf_queue = dwe_vb2_buf_queue, //qbuf
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};


/* ----------------------------------------------------------------
 * VB2 external interface for dwe-v4l2
 */
int dwe_vb2_queue_init(struct vb2_queue *q, struct mutex *mlock,
			dwe_v4l2_stream_t *pstream, struct device *dev)
{
	memset(q, 0, sizeof(struct vb2_queue));

	/* start creating the vb2 queues */
	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	//all stream multiplanar
	q->type = pstream->cur_v4l2_fmt.type;

	q->io_modes = VB2_MMAP | VB2_READ;
	q->drv_priv = pstream;
	q->buf_struct_size = sizeof(dwe_v4l2_buffer_t);

	q->ops = &dwe_vb2_ops;
#if (BUFFER_DMA)
	q->mem_ops = &vb2_dma_contig_memops;
	LOG(LOG_DEBUG, "use vb2_dma_contig_memops !\n");
#else
	q->mem_ops = &vb2_vmalloc_memops;
	LOG(LOG_DEBUG, "use vb2_vmalloc_memops !\n");
#endif
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 1;
	mutex_init(mlock);
	q->lock = mlock;
	//ops
	q->dev = dev;

	return vb2_queue_init(q);
}

void dwe_vb2_queue_release(struct vb2_queue *q)
{
	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	vb2_queue_release(q);
}
