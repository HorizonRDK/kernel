#include <media/videobuf2-dma-contig.h>
#include "avb.h"

struct avb_v4l2_buffer {
	struct vb2_v4l2_buffer buf;
	struct list_head queue;
	struct avb_v4l2_private *avb_dev;
};




#define to_avb_buffer(vb)	container_of(vb, struct avb_v4l2_buffer, buf)

static void avb_v4l2_return_all_buffers(struct avb_v4l2_private *avb_dev,
		enum vb2_buffer_state state)
{
	struct avb_v4l2_buffer *buf, *nbuf;
	unsigned long irq_flags;

	spin_lock_irqsave(&avb_dev->queued_lock, irq_flags);
	list_for_each_entry_safe(buf, nbuf, &avb_dev->queued_bufs, queue) {
		vb2_buffer_done(&buf->buf.vb2_buf, state);
		list_del(&buf->queue);
	}
	spin_unlock_irqrestore(&avb_dev->queued_lock, irq_flags);
}




static int avb_v4l2_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
		unsigned int *nplanes, unsigned int sizes[],
		struct device *alloc_devs[])
{
	struct avb_v4l2_private *avb_dev = vb2_get_drv_priv(vq);

	*nbuffers = 2;
	*nplanes = 1;
	sizes[0] = avb_dev->format.sizeimage;
	return 0;
}




static int avb_v4l2_buf_prepare(struct vb2_buffer *vb)
{
	struct avb_v4l2_private *avb_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct avb_v4l2_buffer *avb_buf = to_avb_buffer(vbuf);
	unsigned long size = avb_dev->format.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(&avb_dev->pdev->dev, "buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	avb_buf->avb_dev = avb_dev;
	vb2_set_plane_payload(vb, 0, size);
	return 0;
}


static void avb_v4l2_buf_queue(struct vb2_buffer *vb)
{
	struct avb_v4l2_private *avb_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct avb_v4l2_buffer *avb_buf = to_avb_buffer(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&avb_dev->queued_lock, flags);
	list_add_tail(&avb_buf->queue, &avb_dev->queued_bufs);
	spin_unlock_irqrestore(&avb_dev->queued_lock, flags);
}


static int avb_v4l2_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct avb_v4l2_private *avb_dev = vb2_get_drv_priv(vq);
	int ret;

	ret = tsn_set_buffer_size(avb_dev->link, avb_dev->format.sizeimage);
	if (ret) {
		dev_err(&avb_dev->pdev->dev, "failed to set TSN buffer size\n");
		avb_v4l2_return_all_buffers(avb_dev, VB2_BUF_STATE_ERROR);
		return ret;
	}

	avb_dev->sequence = 0;
	tsn_lb_enable(avb_dev->link);
	return 0;
}



static void avb_v4l2_stop_streaming(struct vb2_queue *vq)
{
	struct avb_v4l2_private *avb_dev = vb2_get_drv_priv(vq);

	tsn_lb_disable(avb_dev->link);
	avb_v4l2_return_all_buffers(avb_dev, VB2_BUF_STATE_ERROR);
}



static struct vb2_ops avb_v4l2_queue_ops = {
	.queue_setup = avb_v4l2_queue_setup,
	.buf_prepare = avb_v4l2_buf_prepare,
	.buf_queue = avb_v4l2_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = avb_v4l2_start_streaming,
	.stop_streaming = avb_v4l2_stop_streaming,
};


int avb_v4l2_buffer_done(struct avb_v4l2_private *avb_dev)
{
	size_t ret, bytes = avb_dev->format.sizeimage;
	struct avb_v4l2_buffer *avb_buf;
	unsigned long flags;
	void *dest;

	spin_lock_irqsave(&avb_dev->queued_lock, flags);
	if (list_empty(&avb_dev->queued_bufs)) {
		spin_unlock_irqrestore(&avb_dev->queued_lock, flags);
		return 0;
	}

	avb_buf = list_first_entry(&avb_dev->queued_bufs,
			struct avb_v4l2_buffer, queue);
	if (!avb_buf) {
		spin_unlock_irqrestore(&avb_dev->queued_lock, flags);
		return 0;
	}

	list_del(&avb_buf->queue);
	spin_unlock_irqrestore(&avb_dev->queued_lock, flags);

	/* FIXME: Better way ? */
	dest = vb2_plane_vaddr(&avb_buf->buf.vb2_buf, 0);
	ret = tsn_buffer_read(avb_dev->link, dest, bytes);
	if (ret != bytes) {
		dev_err(&avb_dev->pdev->dev, "failed to capture data from TSN\n");
		vb2_buffer_done(&avb_buf->buf.vb2_buf, VB2_BUF_STATE_ERROR);
		avb_v4l2_return_all_buffers(avb_dev, VB2_BUF_STATE_ERROR);
		tsn_lb_disable(avb_dev->link);
		return -EIO;
	}

	avb_buf->buf.sequence = avb_dev->sequence++;
	avb_buf->buf.vb2_buf.timestamp = ktime_get_ns();
	vb2_buffer_done(&avb_buf->buf.vb2_buf, VB2_BUF_STATE_DONE);
	return 0;
}


int avb_v4l2_buffer_init(struct avb_v4l2_private *avb_dev)
{
	struct vb2_queue *queue = &avb_dev->vb2_queue;

	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	queue->lock = &avb_dev->lock;
	queue->drv_priv = avb_dev;
	queue->buf_struct_size = sizeof(struct avb_v4l2_buffer);
	queue->ops = &avb_v4l2_queue_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
		V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	queue->min_buffers_needed = 1;
	queue->dev = &avb_dev->pdev->dev;

	/* Default link state is disabled */
	tsn_lb_disable(avb_dev->link);

	return vb2_queue_init(queue);
}

void avb_v4l2_buffer_exit(struct avb_v4l2_private *avb_dev)
{
}
