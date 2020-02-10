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
#include "dwe_char.h"
#include "acamera_logger.h"
#include "system_dwe_api.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

dwe_charmod_s *dwe_mod[FIRMWARE_CONTEXT_NUMBER];

static int dwe_fop_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	uint32_t tmp = 0;
	dwe_charmod_s *dwe_cdev = NULL;
	int minor = iminor(pinode);

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER; tmp++) {
		if (dwe_mod[tmp]->dev_minor_id == minor) {
			dwe_cdev = dwe_mod[tmp];
			printk(KERN_INFO " tmp %d is open !\n", tmp);
			break;
		}
	}

	if (dwe_cdev == NULL) {
		printk(KERN_INFO " minor is error !\n");
		return -EINVAL;
	}

	if (dwe_cdev->user_num > 0) {
		printk(KERN_INFO " more than one pthred use !\n");
		return -ENXIO;
	}

	spin_lock(&dwe_cdev->slock);
	if (dwe_cdev->user_num == 0) {
		dwe_sw_init();
	}
	dwe_cdev->user_num++;
	pfile->private_data = dwe_cdev;
	spin_unlock(&dwe_cdev->slock);

	//init stream
	ret = dwe_v4l2_stream_init(&dwe_mod[tmp]->pstream, tmp);

	//init vb2-buffer
	ret = dwe_vb2_queue_init(&dwe_mod[tmp]->vb2_q,
		&dwe_mod[tmp]->mlock, dwe_mod[tmp]->pstream,
		dwe_mod[tmp]->dwe_chardev.this_device);
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

extern void dwe0_reset_control(void);

static int dwe_fop_release(struct inode *pinode, struct file *pfile)
{
	dwe_charmod_s *dwe_cdev = pfile->private_data;
	dwe_v4l2_stream_t *pstream = dwe_cdev->pstream;

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

	spin_lock(&dwe_cdev->slock);
	dwe_cdev->user_num--;
	if (dwe_cdev->user_num == 0) {
		dwe_sw_deinit();
		dwe0_reset_control();
	}
	spin_unlock(&dwe_cdev->slock);
	pfile->private_data = NULL;

	LOG(LOG_DEBUG, "---[%s-%d]--- close is success!\n",
		__func__, __LINE__);
	return 0;
}

static ssize_t dwe_fop_read(struct file *pfile, char *puser_buf,
	size_t len, loff_t *poff)
{
	int ret = 0;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	dwe_charmod_s *dwe_cdev = pfile->private_data;

	ret = vb2_read(&dwe_cdev->vb2_q, puser_buf, len, poff,
		pfile->f_flags & O_NONBLOCK);

	return ret;
}

static ssize_t dwe_fop_write(struct file *pfile, const char *puser_buf,
				size_t len, loff_t *poff)
{
	int ret = 0;
	dwe_charmod_s *dwe_cdev = pfile->private_data;
	dframe_t tmp;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	ret = dwe_stream_get_frame(dwe_cdev->port, &tmp);
	if (ret == 0) {
		printk(KERN_INFO "dwe_strea_get_frame is success !\n");
		ret = dwe_stream_put_frame(dwe_cdev->port, &tmp);
		if (ret == 0) {
			printk(KERN_INFO "dwe_strea_put_frame is success !\n");
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

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
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

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	if (dwe_v4l2_is_q_busy(&sp->vb2_q))
		return -EBUSY;

	rc = vb2_streamon(&sp->vb2_q, i);
	if (rc != 0) {
		LOG(LOG_ERR, "fail to vb2_streamon. (rc=%d)", rc);
		return rc;
	}

	/* Start hardware */

	return rc;
}

static int dwe_v4l2_streamoff(void *priv, enum v4l2_buf_type i)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	if (dwe_v4l2_is_q_busy(&sp->vb2_q))
		return -EBUSY;
	/* vb streamoff */
	rc = vb2_streamoff(&sp->vb2_q, i);

	return rc;
}

/* vb2 customization for multi-stream support */
static int dwe_v4l2_reqbufs(void *priv, struct v4l2_requestbuffers *p)
{
	int ret = 0;
	dwe_charmod_s *sp = priv;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	if (dwe_v4l2_is_q_busy(&sp->vb2_q))
		return -EBUSY;

	LOG(LOG_INFO, "count %d, type %d, memory %d",
		p->count, p->type, p->memory);

	ret = vb2_reqbufs(&sp->vb2_q, p);
	if (ret < 0) {
		LOG(LOG_INFO, "%s -- %d, ret = %d !\n",
			__func__, __LINE__, ret);
		return ret;
	}
	dwe_v4l2_streamon(priv, p->type);
	return ret;
}

static int dwe_v4l2_expbuf(void *priv, struct v4l2_exportbuffer *p)
{
	int ret = 0;
	dwe_charmod_s *sp = priv;

	if (dwe_v4l2_is_q_busy(&sp->vb2_q))
		return -EBUSY;

	ret = vb2_expbuf(&sp->vb2_q, p);

	LOG(LOG_DEBUG, "expbuf type:%d index:%d plane:%d rc: %d",
		p->type, p->index, p->plane, ret);
	return ret;
}

static int dwe_v4l2_querybuf(void *priv, struct v4l2_buffer *p)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;

	rc = vb2_querybuf(&sp->vb2_q, p);
	LOG(LOG_DEBUG, "querybuf p->type:%d p->index:%d , rc %d",
		p->type, p->index, rc);
	LOG(LOG_INFO, "index %d, type %d, length %d !\n",
		p->index, p->type, p->length);
	return rc;
}

static int dwe_v4l2_qbuf(void *priv, struct v4l2_buffer *p)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;

	LOG(LOG_INFO, "qbuf p->type:%d p->index:%d \n", p->type, p->index);
	LOG(LOG_DEBUG, "(ownermatch=%d)", dwe_v4l2_is_q_busy(&sp->vb2_q));
	if (dwe_v4l2_is_q_busy(&sp->vb2_q))
		return -EBUSY;

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

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	LOG(LOG_DEBUG, "(ownermatch=%d)", dwe_v4l2_is_q_busy(&sp->vb2_q));
	if (dwe_v4l2_is_q_busy(&sp->vb2_q))
		return -EBUSY;

	rc = vb2_dqbuf(&sp->vb2_q, p, O_NONBLOCK);
	LOG(LOG_INFO, "rc = %d !\n", rc);
	return rc;
}

static int dwe_v4l2_format(void *priv, struct v4l2_format *p)
{
	int rc = 0;
	dwe_charmod_s *sp = priv;
	dwe_v4l2_stream_t *pstream = sp->pstream;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	memcpy(&pstream->cur_v4l2_fmt, p, sizeof(struct v4l2_format));

	LOG(LOG_INFO, "rc = %d !\n", rc);
	return rc;
}

//temp
extern int dis_set_ioctl(uint32_t port, uint32_t online);
extern int ldc_set_ioctl(uint32_t port, uint32_t online);
static uint32_t tmp_port_save = 0;
//

static long dwe_fop_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	long ret = 0;

	dwe_charmod_s *dwe_cdev = pfile->private_data;
	dis_param_s tmp_dis;
	ldc_param_s tmp_ldc;
	pg_param_s  tmp_pg;
	uint32_t tmp_enable = 0;
	struct v4l2_buffer tmp_b;
	struct v4l2_requestbuffers tmp_p;
	struct v4l2_format tmp_f;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

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
		ret = dis_swparam_set(dwe_cdev->port, &tmp_dis);
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
		ret = ldc_swparam_set(dwe_cdev->port, &tmp_ldc);
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
		//tmp_port_save = tmp_port_save ^ 1;
		//ret = dis_set_ioctl(tmp_port_save, 0);
		//ret = ldc_set_ioctl(tmp_port_save, 0);
		msleep(1000);
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

int __init dwe_dev_init(uint32_t port)
{
	int ret = 0;
	int rc = 0;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		return -ENXIO;
	}

	dwe_mod[port] = kzalloc(sizeof(dwe_charmod_s), GFP_KERNEL);
	if (dwe_mod[port] == NULL) {
		printk(KERN_INFO "%s --%d kzalloc !\n", __func__, __LINE__);
		return -ENOMEM;
	}

	snprintf(dwe_mod[port]->name, CHARDEVNAME_LEN, "dwe_sbuf%d", port);
	
//misc_device
	dwe_mod[port]->dwe_chardev.name = dwe_mod[port]->name;
	dwe_mod[port]->dwe_chardev.minor = MISC_DYNAMIC_MINOR;
	dwe_mod[port]->dwe_chardev.fops = &dwe_fops;

	ret = misc_register(&dwe_mod[port]->dwe_chardev);
	if (ret) {
		printk(KERN_INFO "%s --%d, register failed, err %d !\n", __func__, __LINE__, ret);
		goto register_err;
	}
	dwe_mod[port]->dev_minor_id = dwe_mod[port]->dwe_chardev.minor;
	dwe_mod[port]->dwe_chardev.this_device->bus = &platform_bus_type;
	rc = of_dma_configure(dwe_mod[port]->dwe_chardev.this_device,
		dwe_mod[port]->dwe_chardev.this_device->of_node);
	dwe_mod[port]->port = port;
	spin_lock_init(&(dwe_mod[port]->slock));
	LOG(LOG_INFO, "%s register success !\n", dwe_mod[port]->name);
	return ret;
register_err:
	kzfree(dwe_mod[port]);
	return ret;
}
EXPORT_SYMBOL(dwe_dev_init);

void __exit dwe_dev_exit(int port)
{
	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	if ((port < FIRMWARE_CONTEXT_NUMBER) && (dwe_mod[port] != NULL)) {
		misc_deregister(&dwe_mod[port]->dwe_chardev);
		kzfree(dwe_mod[port]);
		dwe_mod[port] = NULL;
	}
}
EXPORT_SYMBOL(dwe_dev_exit);

static void tmpdevs_exit(void)
{
	uint32_t tmp = 0;

	for (tmp = 0; tmp < 4; tmp++) {
		dwe_dev_exit(tmp);
	}
}

static int tmpdevs_init(void)
{
	int ret = 0;
	uint32_t tmp = 0;

	for (tmp = 0; tmp < 4; tmp++) {
		ret = dwe_dev_init(tmp);
		if (ret < 0) {
			printk(KERN_INFO "dwe_dev_init %d is failed\n", tmp);
			goto devinit_err;
		}
	}

	return ret;
devinit_err:
	tmpdevs_exit();
	return ret;
}

//module_init(tmpdevs_init);
//module_exit(tmpdevs_exit);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("dwe_char dev of x2a");
MODULE_LICENSE("GPL v2");
