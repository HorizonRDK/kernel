#include <linux/avb.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include "avb.h"

#define video_to_avb(v)		container_of(v, struct avb_v4l2_private, video)



static const struct avb_v4l2_video_format avb_v4l2_formats[] = {
	{
		.name = "BGR32",
		.description = "BGR 32 (32 bpp)",
		.fourcc = V4L2_PIX_FMT_BGR32,
		.depth = 32,
	}
};



static const struct avb_v4l2_video_format
		*avb_v4l2_get_format_by_fourcc(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(avb_v4l2_formats); i++) {
		const struct avb_v4l2_video_format *format = &avb_v4l2_formats[i];

		if (format->fourcc == fourcc)
			return format;
	}

	return ERR_PTR(-EINVAL);
}


static int avb_v4l2_vidioc_querycap(struct file *file, void *fh,
		struct v4l2_capability *cap)
{
	struct avb_v4l2_private *avb_dev = video_drvdata(file);

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_DEVICE_CAPS |
		V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
		V4L2_CAP_READWRITE;

	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, "TSN V4L2 SHIM", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
			dev_name(&avb_dev->pdev->dev));
	return 0;
}


static int avb_v4l2_vidioc_enum_fmt(struct file *file, void *fh,
		struct v4l2_fmtdesc *fmt)
{
	const struct avb_v4l2_video_format *fmtinfo;

	if (fmt->index >= ARRAY_SIZE(avb_v4l2_formats))
		return -EINVAL;

	fmtinfo = &avb_v4l2_formats[fmt->index];
	fmt->pixelformat = fmtinfo->fourcc;
	strlcpy(fmt->description, fmtinfo->description, sizeof(fmt->description));
	return 0;
}




static int avb_v4l2_vidioc_g_fmt(struct file *file, void *fh,
		struct v4l2_format *format)
{
	struct avb_v4l2_private *avb_dev = video_drvdata(file);

	format->fmt.pix = avb_dev->format;
	return 0;
}

static int avb_v4l2_fill_fmt(struct avb_v4l2_private *avb_dev,
		struct v4l2_pix_format *pix,
		const struct avb_v4l2_video_format **fmtinfo)
{
	const struct avb_v4l2_video_format *info;

	info = avb_v4l2_get_format_by_fourcc(pix->pixelformat);
	if (IS_ERR(info))
		info = avb_v4l2_get_format_by_fourcc(AVB_V4L2_DEFAULT_PIX_FMT);

	pix->pixelformat = info->fourcc;
	pix->width = AVB_VIDEO_XRES;
	pix->height = AVB_VIDEO_YRES;
	pix->bytesperline = (pix->width * info->depth) / 8;
	pix->sizeimage = pix->bytesperline * pix->height;

	if (fmtinfo)
		*fmtinfo = info;
	return 0;
}

static int avb_v4l2_vidioc_s_fmt(struct file *file, void *fh,
		struct v4l2_format *format)
{
	struct avb_v4l2_private *avb_dev = video_drvdata(file);
	const struct avb_v4l2_video_format *fmtinfo;
	int ret;

	if (vb2_is_busy(&avb_dev->vb2_queue))
		return -EBUSY;

	ret = avb_v4l2_fill_fmt(avb_dev, &format->fmt.pix, &fmtinfo);
	if (ret)
		return ret;

	avb_dev->format = format->fmt.pix;
	avb_dev->fmtinfo = fmtinfo;
	return 0;
}



static int avb_v4l2_vidioc_try_fmt(struct file *file, void *fh,
		struct v4l2_format *format)
{
	struct avb_v4l2_private *avb_dev = video_drvdata(file);

	return avb_v4l2_fill_fmt(avb_dev, &format->fmt.pix, NULL);
}


static int avb_v4l2_vidioc_enum_input(struct file *file, void *fh,
		struct v4l2_input *in)
{
	if (in->index != 0)
		return -EINVAL;

	/* FIXME */
	sprintf(in->name, "VIRTUAL-A");
	in->type = V4L2_INPUT_TYPE_CAMERA;
	in->capabilities = 0;
	in->status = 0;
	return 0;
}



static int avb_v4l2_vidioc_s_input(struct file *file, void *priv, unsigned i)
{
	if (i != 0)
		return -EINVAL;
	return 0;
}

static int avb_v4l2_vidioc_g_input(struct file *file, void *priv, unsigned *i)
{
	*i = 0;
	return 0;
}

static int avb_v4l2_vidioc_g_parm(struct file *file, void *fh,
		struct v4l2_streamparm *parm)
{
	parm->parm.capture.timeperframe.numerator = 1;
	parm->parm.capture.timeperframe.denominator = 60;
	return 0;
}


static const struct v4l2_ioctl_ops avb_v4l2_video_ioctl_ops = {
	.vidioc_querycap = avb_v4l2_vidioc_querycap,
	/* format handling */
	.vidioc_enum_fmt_vid_cap = avb_v4l2_vidioc_enum_fmt,
	.vidioc_g_fmt_vid_cap = avb_v4l2_vidioc_g_fmt,
	.vidioc_s_fmt_vid_cap = avb_v4l2_vidioc_s_fmt,
	.vidioc_try_fmt_vid_cap = avb_v4l2_vidioc_try_fmt,
	/* buffer control */
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	/* streaming control */
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	/* input handling */
	.vidioc_enum_input = avb_v4l2_vidioc_enum_input,
	.vidioc_s_input = avb_v4l2_vidioc_s_input,
	.vidioc_g_input = avb_v4l2_vidioc_g_input,
	/* stream type-dependent parameter ioctls */
	.vidioc_g_parm = avb_v4l2_vidioc_g_parm,
	/* debug */
	/*.vidioc_log_status = avb_v4l2_log_status,*/
};

static const struct v4l2_file_operations avb_v4l2_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};




int avb_v4l2_video_init(struct avb_v4l2_private *avb_dev)
{
	struct video_device *video = &avb_dev->video;
	const struct avb_v4l2_video_format *fmtinfo;
	struct device *dev = &avb_dev->pdev->dev;
	int ret;

	mutex_init(&avb_dev->lock);
	INIT_LIST_HEAD(&avb_dev->queued_bufs);
	spin_lock_init(&avb_dev->queued_lock);

	video->fops = &avb_v4l2_fops;
	video->v4l2_dev = &avb_dev->v4l2_dev;
	video->queue = &avb_dev->vb2_queue;
	snprintf(video->name, sizeof(video->name), "AVB input");
	video->vfl_type = VFL_TYPE_GRABBER;
	video->vfl_dir = VFL_DIR_RX;
	video->release = video_device_release_empty;
	video->ioctl_ops = &avb_v4l2_video_ioctl_ops;
	video->lock = &avb_dev->lock;

	video_set_drvdata(video, avb_dev);

	/* Buffer control */
	ret = avb_v4l2_buffer_init(avb_dev);
	if (ret) {
		dev_err(dev, "failed to initialize buffer control\n");
		return ret;
	}

	/* Fill initial settings */
	avb_dev->format.pixelformat = AVB_V4L2_DEFAULT_PIX_FMT;
	avb_v4l2_fill_fmt(avb_dev, &avb_dev->format, &fmtinfo);
	avb_dev->fmtinfo = fmtinfo;

	ret = video_register_device(video, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(dev, "failed to register video device\n");
		goto err_buffer;
	}

	return 0;

err_buffer:
	avb_v4l2_buffer_exit(avb_dev);
	return ret;
}

void avb_v4l2_video_exit(struct avb_v4l2_private *avb_dev)
{
	if (video_is_registered(&avb_dev->video))
		video_unregister_device(&avb_dev->video);
	avb_v4l2_buffer_exit(avb_dev);
	mutex_destroy(&avb_dev->lock);
}
