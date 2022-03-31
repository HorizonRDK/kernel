/*
 *    driver, char device interface
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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/spinlock_types.h>
#include <linux/wait.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/atomic.h>

#include "dwe_char.h"
#include "acamera_logger.h"
#include "system_dwe_api.h"
#include "vio_group_api.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

extern struct class *vps_class;
dwe_charmod_s *dwe_mod[FIRMWARE_CONTEXT_NUMBER];
static uint32_t pipe_count = 0;
static uint32_t pipe_key = 0;
struct mutex g_lock;

extern void vio_dwe_clk_enable(void);
extern void vio_dwe_clk_disable(void);

static int dwe_fop_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	uint32_t tmp = 0;
	dwe_charmod_s *dwe_cdev = NULL;
	//int minor = iminor(pinode);
	int minor = imajor(pinode);

	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER; tmp++) {
		if (dwe_mod[tmp]->dev_minor_id == minor) {
			dwe_cdev = dwe_mod[tmp];
			LOG(LOG_DEBUG, " tmp %d is open !\n", tmp);
			break;
		}
	}

	if (dwe_cdev == NULL) {
		LOG(LOG_ERR, " minor is error !\n");
		return -EINVAL;
	}

	if (atomic_cmpxchg(&dwe_cdev->user_num, 0, 1) != 0) {
		LOG(LOG_ERR, " more than one pthred use !\n");
		return -EBUSY;
	}

	pfile->private_data = dwe_cdev;

	// protect pipe_count & reset_ctrl
	mutex_lock(&g_lock);
	if (pipe_count == 0) {
		vio_ldc_access_mutex_lock();
		vio_set_ldc_rst_flag(0);
		vio_ldc_access_mutex_unlock();
		vio_dwe_clk_enable();
		dwe_sw_init();
	}
	pipe_count++;
	mutex_unlock(&g_lock);

	//init stream
	ret = dwe_v4l2_stream_init(&dwe_mod[tmp]->pstream, tmp);

	//init vb2-buffer
//mis
#if 0
	ret = dwe_vb2_queue_init(&dwe_mod[tmp]->vb2_q,
		&dwe_mod[tmp]->mlock, dwe_mod[tmp]->pstream,
		dwe_mod[tmp]->dwe_chardev.this_device);
#endif
//platform
#if 1
	ret = dwe_vb2_queue_init(&dwe_mod[tmp]->vb2_q,
		&dwe_mod[tmp]->mlock, dwe_mod[tmp]->pstream,
		&dwe_mod[tmp]->g_dwe_dev->dev);
#endif

	if (ret < 0) {
		goto vb2_failed;
	}

	LOG(LOG_INFO, "open is success !\n");

	return 0;
vb2_failed:
	dwe_v4l2_stream_deinit(dwe_mod[tmp]->pstream);
	//dwe_vb2_queue_release(&dwe_mod[tmp]->vb2_q);
	return ret;
}

static int dwe_fop_release(struct inode *pinode, struct file *pfile)
{
	dwe_charmod_s *dwe_cdev = pfile->private_data;
	dwe_v4l2_stream_t *pstream = dwe_cdev->pstream;

	if (dwe_cdev == NULL) {
		return -ENXIO;
	}

	if (atomic_cmpxchg(&dwe_cdev->user_num, 1, 0) != 1) {
		return -EPERM;
	}

	/* deinit stream */
	if (pstream) {
		dwe_v4l2_stream_deinit(pstream);
		pstream = NULL;
	}

	/* release vb2 queue */
	if (dwe_cdev->vb2_q.lock)
		mutex_lock(dwe_cdev->vb2_q.lock);

	dwe_vb2_queue_release(&dwe_cdev->vb2_q);

	if (dwe_cdev->vb2_q.lock)
		mutex_unlock(dwe_cdev->vb2_q.lock);

	pfile->private_data = NULL;
	// protect pipe_count & reset_ctrl
	mutex_lock(&g_lock);
	pipe_count--;
	if (pipe_count == 0) {
		dwe_sw_deinit();
		vio_dwe_clk_disable();
	}
	mutex_unlock(&g_lock);

	LOG(LOG_DEBUG, "close is success!\n");
	return 0;
}

static ssize_t dwe_fop_read(struct file *pfile, char *puser_buf,
	size_t len, loff_t *poff)
{
	ssize_t ret = 0;

	dwe_charmod_s *dwe_cdev = pfile->private_data;

	ret = vb2_read(&dwe_cdev->vb2_q, puser_buf, len, poff,
		(int)(pfile->f_flags & O_NONBLOCK));

	return ret;
}

static ssize_t dwe_fop_write(struct file *pfile, const char *puser_buf,
				size_t len, loff_t *poff)
{
	int ret = 0;
	dwe_charmod_s *dwe_cdev = pfile->private_data;
	dframe_t tmp;

	ret = dwe_stream_get_frame(dwe_cdev->port, &tmp);
	if (ret == 0) {
		LOG(LOG_DEBUG, "dwe_strea_get_frame is success !\n");
		ret = dwe_stream_put_frame(dwe_cdev->port, &tmp);
		if (ret == 0) {
			LOG(LOG_DEBUG, "dwe_strea_put_frame is success !\n");
		}
	}

	return 0;
}

static unsigned int dwe_fop_poll(struct file *pfile,
				struct poll_table_struct *wait)
{
	dwe_charmod_s *dwe_cdev = pfile->private_data;
	int ret = 0;


	if (dwe_cdev->vb2_q.lock && mutex_lock_interruptible(dwe_cdev->vb2_q.lock))
		return POLLERR;//return POLLERR;

//	ret = vb2_poll(&dwe_cdev->vb2_q, pfile, wait);
	ret = vb2_core_poll(&dwe_cdev->vb2_q, pfile, wait);

	if (dwe_cdev->vb2_q.lock)
		mutex_unlock(dwe_cdev->vb2_q.lock);

	LOG(LOG_INFO, "poll ret= %d !\n", ret);
	return ret;
}

static int dwe_fop_mmap(struct file *pfile, struct vm_area_struct *vma)
{
	dwe_charmod_s *dwe_cdev = pfile->private_data;
	int ret = 0;

	ret = vb2_mmap(&dwe_cdev->vb2_q, vma);
	return ret;
}

/* Per-stream control operations */
static inline bool dwe_v4l2_is_q_busy(struct vb2_queue *queue)
{
	//return queue->owner ;
	return 0;
}

static int dwe_v4l2_streamon(void *priv, enum v4l2_buf_type i)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;

	if (dwe_v4l2_is_q_busy(&sp->vb2_q)) /*PRQA S 2992 ++*/
		return -EBUSY; /*PRQA S 2880 ++*/

	rc = vb2_streamon(&sp->vb2_q, i);
	if (rc != 0) {
		LOG(LOG_ERR, "fail to vb2_streamon. (rc=%d)", rc);
		return rc;
	}

	/* Start hardware */

	return rc;
}

#if 0
static int dwe_v4l2_streamoff(void *priv, enum v4l2_buf_type i)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;

	if (dwe_v4l2_is_q_busy(&sp->vb2_q)) /*PRQA S 2992 ++*/
		return -EBUSY;
	/* vb streamoff */
	rc = vb2_streamoff(&sp->vb2_q, i);

	return rc;
}
#endif

/* vb2 customization for multi-stream support */
static int dwe_v4l2_reqbufs(void *priv, struct v4l2_requestbuffers *p)
{
	int ret = 0;
	dwe_charmod_s *sp = priv;

	if (dwe_v4l2_is_q_busy(&sp->vb2_q)) /*PRQA S 2992 ++*/
		return -EBUSY; /*PRQA S 2880 ++*/

	LOG(LOG_INFO, "count %d, type %d, memory %d",
		p->count, p->type, p->memory);

	ret = vb2_reqbufs(&sp->vb2_q, p);
	if (ret < 0) {
		LOG(LOG_INFO, "ret = %d !\n", ret);
		return ret;
	}
	dwe_v4l2_streamon(priv, p->type);
	return ret;
}

#if 0
static int dwe_v4l2_expbuf(void *priv, struct v4l2_exportbuffer *p)
{
	int ret = 0;
	dwe_charmod_s *sp = priv;

	if (dwe_v4l2_is_q_busy(&sp->vb2_q)) /*PRQA S 2992 ++*/
		return -EBUSY; /*PRQA S 2880 ++*/

	ret = vb2_expbuf(&sp->vb2_q, p);

	LOG(LOG_DEBUG, "expbuf type:%d index:%d plane:%d rc: %d",
		p->type, p->index, p->plane, ret);
	return ret;
}
#endif

static int dwe_v4l2_querybuf(void *priv, struct v4l2_buffer *p)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;

	rc = vb2_querybuf(&sp->vb2_q, p);
	LOG(LOG_DEBUG, "querybuf p->type:%d p->index:%d p->length %d, rc %d",
		p->type, p->index, p->length, rc);
	return rc;
}

static int dwe_v4l2_qbuf(void *priv, struct v4l2_buffer *p)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;

	LOG(LOG_INFO, "qbuf p->type:%d p->index:%d \n", p->type, p->index);
	LOG(LOG_DEBUG, "(ownermatch=%d)", dwe_v4l2_is_q_busy(&sp->vb2_q)); /*PRQA S 2992 ++*/
	if (dwe_v4l2_is_q_busy(&sp->vb2_q)) /*PRQA S 2992 ++*/
		return -EBUSY; /*PRQA S 2880 ++*/

	rc = vb2_qbuf(&sp->vb2_q, p);
	if (rc == 0) {
		LOG(LOG_INFO, "qbuf memory:%d,state %d \n", p->memory,
			sp->vb2_q.bufs[p->index]->state);
		LOG(LOG_INFO, "type:%d num_buffers:%d\n", sp->vb2_q.type,
			sp->vb2_q.num_buffers);
	}

	LOG(LOG_INFO, "qbuf p->type:%d p->index:%d, rc %d\n", p->type, p->index, rc);
	return rc;
}

static int dwe_v4l2_dqbuf(void *priv, struct v4l2_buffer *p)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;

	LOG(LOG_DEBUG, "(ownermatch=%d)", dwe_v4l2_is_q_busy(&sp->vb2_q)); /*PRQA S 2992 ++*/
	if (dwe_v4l2_is_q_busy(&sp->vb2_q)) /*PRQA S 2992 ++*/
		return -EBUSY; /*PRQA S 2880 ++*/

	rc = vb2_dqbuf(&sp->vb2_q, p, O_NONBLOCK);
	LOG(LOG_INFO, "rc = %d !\n", rc);
	return rc;
}

static int dwe_v4l2_format(void *priv, struct v4l2_format *p)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;
	dwe_v4l2_stream_t *pstream = sp->pstream;

	memcpy(&pstream->cur_v4l2_fmt, p, sizeof(struct v4l2_format));

	LOG(LOG_INFO, "rc = %d !\n", rc);
	return rc;
}

/*PRQA S 0591 ++*/
static long dwe_fop_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	long ret = 0;

	dwe_charmod_s *dwe_cdev = pfile->private_data;
	dis_param_s tmp_dis;
	ldc_param_s tmp_ldc;
	pg_param_s  tmp_pg;
	uint32_t tmp_enable = 0;
	/* used for dis stats */
	struct v4l2_buffer tmp_b;
	struct v4l2_requestbuffers tmp_p;
	struct v4l2_format tmp_f;

	switch (cmd) {
	case DWEC_SET_DIS_PARAM: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&tmp_dis, (void __user *)arg,
			sizeof(dis_param_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
	        if (pipe_count > 1) {
		        ret = dis_swparam_set(dwe_cdev->port, &tmp_dis, 0);
                } else {
		        ret = dis_swparam_set(dwe_cdev->port, &tmp_dis, 1);
                }
	}
		break;
	case DWEC_GET_DIS_PARAM: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		ret = dis_swparam_get(dwe_cdev->port, &tmp_dis);
		if (ret == 0) {
			if (copy_to_user((void __user *)arg, (void *)&tmp_dis,
				sizeof(dis_param_s))) {
				LOG(LOG_ERR, "copy is err !\n");
				return -EINVAL;
			}
		}
	}
		break;
	case DWEC_SET_LDC_PARAM: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&tmp_ldc, (void __user *)arg,
			sizeof(ldc_param_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
	        if (pipe_count > 1) {
		        ret = ldc_swparam_set(dwe_cdev->port, &tmp_ldc, 0);
                } else {
		        ret = ldc_swparam_set(dwe_cdev->port, &tmp_ldc, 1);
                }
	}
		break;
	case DWEC_GET_LDC_PARAM: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		ret = ldc_swparam_get(dwe_cdev->port, &tmp_ldc);
		if (ret == 0) {
			if (copy_to_user((void __user *)arg, (void *)&tmp_ldc,
				sizeof(ldc_param_s))) {
				LOG(LOG_ERR, "copy is err !\n");
				return -EINVAL;
			}
		}
	}
		break;
	case DWEC_SET_PG_PARAM: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&tmp_pg, (void __user *)arg,
			sizeof(pg_param_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = pattgen_param_set(dwe_cdev->port, &tmp_pg);
	}
		break;
	case DWEC_GET_PG_PARAM: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		ret = pattgen_param_get(dwe_cdev->port, &tmp_pg);
		if (ret == 0) {
			if (copy_to_user((void __user *)arg, (void *)&tmp_pg,
				sizeof(pg_param_s))) {
				LOG(LOG_ERR, "copy is err !\n");
				return -EINVAL;
			}
		}
	}
		break;
	case DWEC_START_PG: {
		ret = start_pg_pulse(dwe_cdev->port);
	}
		break;
	case DWEC_PG_ENABEL: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&tmp_enable, (void __user *)arg,
			sizeof(uint32_t))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = pg_mode_enable(tmp_enable);
	}
		break;
	case DWEC_REQBUFS: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&tmp_p, (void __user *)arg,
			sizeof(struct v4l2_requestbuffers))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = dwe_v4l2_reqbufs((void *)(dwe_cdev), &tmp_p);
		if (ret == 0) {
			if (copy_to_user((void __user *)arg, (void *)&tmp_p,
				sizeof(struct v4l2_requestbuffers))) {
				LOG(LOG_ERR, "copy is err !\n");
				return -EINVAL;
			}
		}
	}
		break;
	case DWEC_QUERYBUF: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&tmp_b, (void __user *)arg,
			sizeof(struct v4l2_buffer))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = dwe_v4l2_querybuf((void *)(dwe_cdev), &tmp_b);
		if (ret == 0) {
			if (copy_to_user((void __user *)arg, (void *)&tmp_b,
				sizeof(struct v4l2_buffer))) {
				LOG(LOG_ERR, "copy is err !\n");
				return -EINVAL;
			}
		}
	}
		break;
	case DWEC_DQBUF: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&tmp_b, (void __user *)arg,
			sizeof(struct v4l2_buffer))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = dwe_v4l2_dqbuf((void *)(dwe_cdev), &tmp_b);
		if (ret == 0) {
			if (copy_to_user((void __user *)arg, (void *)&tmp_b,
				sizeof(struct v4l2_buffer))) {
				LOG(LOG_ERR, "copy is err !\n");
	 			return -EINVAL;
			}
		}
	}
		break;
	case DWEC_QBUF: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -EINVAL;
		}
		if (copy_from_user((void *)&tmp_b, (void __user *)arg,
			sizeof(struct v4l2_buffer))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = dwe_v4l2_qbuf((void *)(dwe_cdev), &tmp_b);
		if (ret == 0) {
			if (copy_to_user((void __user *)arg, (void *)&tmp_b,
				sizeof(struct v4l2_buffer))) {
				LOG(LOG_ERR, "copy is err !\n");
	 			return -EINVAL;
			}
		}
	}
		break;
	case DWEC_FORMAT: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -EINVAL;
		}
		if (copy_from_user((void *)&tmp_f, (void __user *)arg,
			sizeof(struct v4l2_format))) {
			LOG(LOG_ERR, "copy is err !\n");
	 		return -EINVAL;
		}
		ret = dwe_v4l2_format((void *)(dwe_cdev), &tmp_f);
	}
		break;
	default: {
		LOG(LOG_ERR, "---cmd is err---\n");
		ret = -1;
	}
		break;
	}

	return ret;
}


const struct file_operations dwe_fops = {
	.owner = THIS_MODULE,
	.open = dwe_fop_open,
	.read = dwe_fop_read,
	.write = dwe_fop_write,
	.release = dwe_fop_release,
	.unlocked_ioctl = dwe_fop_ioctl,
	.compat_ioctl = dwe_fop_ioctl,
	.mmap = dwe_fop_mmap,
	.poll = dwe_fop_poll,
};

static int dwe_probe(struct platform_device *dev)
{
        int ret = 0;
	    int port = 0;

	if (strstr(dev->name, "dwe_sbuf0") != 0) {
		port = 0;
	} else if (strstr(dev->name, "dwe_sbuf1") != 0) {
		port = 1;
	} else if (strstr(dev->name, "dwe_sbuf2") != 0) {
		port = 2;
	} else if (strstr(dev->name, "dwe_sbuf3") != 0) {
		port = 3;
	} else if (strstr(dev->name, "dwe_sbuf4") != 0) {
		port = 4;
	} else if (strstr(dev->name, "dwe_sbuf5") != 0) {
		port = 5;
	} else if (strstr(dev->name, "dwe_sbuf6") != 0) {
		port = 6;
	} else if (strstr(dev->name, "dwe_sbuf7") != 0) {
		port = 7;
	}

	ret = register_chrdev(0, dev->name, &dwe_fops);
	if (ret < 0) {
		return ret;
	}

	dwe_mod[port]->dev_minor_id = ret;
	device_create(vps_class, NULL, MKDEV(dwe_mod[port]->dev_minor_id, 0),
		NULL, "dwe_sbuf%d", port);
	ret = of_dma_configure(&dev->dev, dev->dev.of_node);
    return 0;
}

static int dwe_remove(struct platform_device *dev)
{
        return 0;
}

static struct platform_driver dwe_device_driver[FIRMWARE_CONTEXT_NUMBER] = {
#if FIRMWARE_CONTEXT_NUMBER > 0
	{
		.probe          = dwe_probe,
		.remove         = dwe_remove,
		.driver         = {
			.name   = "dwe_sbuf0",
		},
	},
#endif
#if FIRMWARE_CONTEXT_NUMBER > 1
	{
		.probe          = dwe_probe,
		.remove         = dwe_remove,
		.driver         = {
			.name   = "dwe_sbuf1",
		},
	},
#endif
#if FIRMWARE_CONTEXT_NUMBER > 2
	{
		.probe          = dwe_probe,
		.remove         = dwe_remove,
		.driver         = {
			.name   = "dwe_sbuf3",
		},
	},
#endif
#if FIRMWARE_CONTEXT_NUMBER > 3
	{
		.probe          = dwe_probe,
		.remove         = dwe_remove,
		.driver         = {
			.name   = "dwe_sbuf4",
		},
	},
#endif
#if FIRMWARE_CONTEXT_NUMBER > 4
	{
		.probe          = dwe_probe,
		.remove         = dwe_remove,
		.driver         = {
			.name   = "dwe_sbuf5",
		},
	},
#endif
#if FIRMWARE_CONTEXT_NUMBER > 5
	{
		.probe          = dwe_probe,
		.remove         = dwe_remove,
		.driver         = {
			.name   = "dwe_sbuf6",
		},
	},
#endif
#if FIRMWARE_CONTEXT_NUMBER > 6
	{
		.probe          = dwe_probe,
		.remove         = dwe_remove,
		.driver         = {
			.name   = "dwe_sbuf7",
		},
	},
#endif
#if FIRMWARE_CONTEXT_NUMBER > 7
	{
		.probe          = dwe_probe,
		.remove         = dwe_remove,
		.driver         = {
			.name   = "dwe_sbuf8",
		},
	},
#endif
};

int __init dwe_dev_init(uint32_t port)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		return -ENXIO;
	}

	dwe_mod[port] = kzalloc(sizeof(dwe_charmod_s), GFP_KERNEL);
	if (dwe_mod[port] == NULL) {
		LOG(LOG_ERR, " kzalloc is failed !\n");
		return -ENOMEM;
	}

	snprintf(dwe_mod[port]->name, CHARDEVNAME_LEN, "dwe_sbuf%d", port);

//misc_device
#if 0
	dwe_mod[port]->dwe_chardev.name = dwe_mod[port]->name;
	dwe_mod[port]->dwe_chardev.minor = MISC_DYNAMIC_MINOR;
	dwe_mod[port]->dwe_chardev.fops = &dwe_fops;

	ret = misc_register(&dwe_mod[port]->dwe_chardev);
	if (ret) {
		LOG(LOG_ERR, " register failed, err %d !\n", ret);
		goto register_err;
	}
	dwe_mod[port]->dev_minor_id = dwe_mod[port]->dwe_chardev.minor;
	dwe_mod[port]->dwe_chardev.this_device->bus = &platform_bus_type;
	ret = of_dma_configure(dwe_mod[port]->dwe_chardev.this_device,
		dwe_mod[port]->dwe_chardev.this_device->of_node);
	dwe_mod[port]->port = port;
#endif
//platform
#if 1
	dwe_mod[port]->g_dwe_dev = platform_device_alloc(dwe_mod[port]->name, -1);
	if (!dwe_mod[port]->g_dwe_dev) {
		ret = -ENOMEM;
		goto register_err;
	}

	ret = platform_device_add(dwe_mod[port]->g_dwe_dev);
	if (ret < 0) {
		platform_device_put(dwe_mod[port]->g_dwe_dev);
		goto register_err;
	}

	dwe_device_driver[port].driver.name = dwe_mod[port]->name;
	ret = platform_driver_register(&dwe_device_driver[port]);
	if (ret < 0) {
		platform_device_unregister(dwe_mod[port]->g_dwe_dev);
		goto register_err;
	}

	dwe_mod[port]->port = port;
	atomic_set(&dwe_mod[port]->user_num, 0);
	if (pipe_key == 0) {
		mutex_init(&g_lock);
	}
	pipe_key++;
#endif
	LOG(LOG_INFO, "%s register success !\n", dwe_mod[port]->name);
	return ret;

register_err:
	kzfree(dwe_mod[port]);
	dwe_mod[port] = NULL;
	return ret;
}
EXPORT_SYMBOL(dwe_dev_init);

void __exit dwe_dev_exit(int port)
{
	if ((port < FIRMWARE_CONTEXT_NUMBER) && (dwe_mod[port] != NULL)) {
//misc
#if 0
		misc_deregister(&dwe_mod[port]->dwe_chardev);
#endif
//platform
#if 1
		platform_driver_unregister(&dwe_device_driver[port]);
		platform_device_unregister(dwe_mod[port]->g_dwe_dev);
#endif
		if (pipe_key == 1) {
			mutex_destroy(&g_lock);
		}
		pipe_key--;
		kzfree(dwe_mod[port]);
		dwe_mod[port] = NULL;
	}
}
EXPORT_SYMBOL(dwe_dev_exit);
