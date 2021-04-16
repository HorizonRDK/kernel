#ifndef __MEDIA_AVB_H__
#define __MEDIA_AVB_H__

#include <linux/platform_device.h>
#include <linux/tsn.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#define AVB_V4L2_DEFAULT_PIX_FMT		V4L2_PIX_FMT_BGR32

struct avb_v4l2_video_format {
	const char *name;
	const char *description;
	u32 fourcc;
	u8 depth;
};



struct avb_v4l2_private {
	struct platform_device *pdev;
	struct v4l2_device v4l2_dev;
	struct vb2_queue vb2_queue;
	struct list_head queued_bufs;
	spinlock_t queued_lock;
	unsigned int sequence;
	struct video_device video;
	struct v4l2_pix_format format;
	const struct avb_v4l2_video_format *fmtinfo;
	struct tsn_link *link;
	struct mutex lock;
};

int avb_v4l2_video_init(struct avb_v4l2_private *avb_dev);
void avb_v4l2_video_exit(struct avb_v4l2_private *avb_dev);
int avb_v4l2_buffer_done(struct avb_v4l2_private *avb_dev);
int avb_v4l2_buffer_init(struct avb_v4l2_private *avb_dev);
void avb_v4l2_buffer_exit(struct avb_v4l2_private *avb_dev);

#endif /* __MEDIA_AVB_H__ */
