/*   Copyright (C) 2018 Horizon Inc.
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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/spinlock_types.h>
#include <linux/printk.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>

#include "inc/camera_dev.h"
#include "inc/camera_ctrl.h"
#include "inc/camera_subdev.h"

camera_ctrlmod_s *camera_ctrl;
sensor_ctrl_info_t sensor_info[8];

static DECLARE_WAIT_QUEUE_HEAD(sensor_update);
static uint32_t update_flag[8];

void set_sensor_aexp_info(uint32_t port, void *ptr)
{
	if ((ptr) && (port < 8)) {
		sensor_priv_t *data = (sensor_priv_t *)ptr;
		sensor_info[port].gain_num = data->gain_num;
		memcpy(sensor_info[port].gain_buf, data->gain_buf, sizeof(sensor_info[port].gain_buf));
		sensor_info[port].dgain_num = data->dgain_num;
		memcpy(sensor_info[port].dgain_buf, data->dgain_buf, sizeof(sensor_info[port].dgain_buf));
		sensor_info[port].en_dgain = data->en_dgain;
		sensor_info[port].line_num = data->line_num;
		memcpy(sensor_info[port].line_buf, data->line_buf, sizeof(sensor_info[port].line_buf));
		sensor_info[port].rgain = data->rgain;
		sensor_info[port].bgain = data->bgain;
		sensor_info[port].grgain = data->grgain;
		sensor_info[port].gbgain = data->gbgain;
		sensor_info[port].mode = data->mode;
		sensor_info[port].port = port;
	}
}

void set_sensor_af_pos_info(uint32_t port, uint32_t af_pos)
{
	if (port < 8) {
		sensor_info[port].af_pos = af_pos;
	}
}
EXPORT_SYMBOL_GPL(set_sensor_af_pos_info);

void set_sensor_zoom_pos_info(uint32_t port, uint32_t zoom_pos)
{
	if (port < 8) {
		sensor_info[port].zoom_pos = zoom_pos;
	}
}
EXPORT_SYMBOL_GPL(set_sensor_zoom_pos_info);

int sensor_ctrl_completion_timeout(uint32_t port, uint32_t timeout)
{
	uint32_t td = 0;
	int ret = 0;

	td = wait_event_timeout(sensor_update, update_flag[port], msecs_to_jiffies(timeout));
        update_flag[port] = 0;
	if (!td) {
		pr_debug("ctx[%d] is time_out\n", port);
		ret = -1;
	}
	return ret;
}

void sensor_ctrl_wakeup(uint32_t port)
{
	update_flag[port] = 1;
	wake_up(&sensor_update);
}

static int camera_ctrl_fop_open(struct inode *pinode, struct file *pfile)
{
	pr_info("camera_fop_open begin %d \n", __LINE__);
	mutex_lock(&camera_ctrl->m_mutex);
	camera_ctrl->user_num++;
	if (camera_ctrl->user_num == 1) {
		pfile->private_data = camera_ctrl;
	}
	mutex_unlock(&camera_ctrl->m_mutex);
	pr_info("camera_fop_open success %d\n", __LINE__);
	return 0;
}

static int camera_ctrl_fop_release(struct inode *pinode, struct file *pfile)
{
	camera_ctrlmod_s *camera_cdev = pfile->private_data;

	mutex_lock(&camera_cdev->m_mutex);
	camera_cdev->user_num--;
	if (camera_cdev->user_num == 0) {
		pfile->private_data = NULL;
	}
	mutex_unlock(&camera_cdev->m_mutex);
	pr_info("camera_fop_release success %d\n", __LINE__);
	return 0;
}
static long camera_ctrl_fop_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	uint32_t port = 0;

	switch(cmd) {
		case SENSOR_CTRL_INFO_SYNC:
			if (copy_from_user((void *)&port, (void __user *)arg, sizeof(uint32_t))) {
				pr_err("copy is err !\n");
				ret = -EINVAL;
			} else {
				ret = sensor_ctrl_completion_timeout(port, 200);
				if (!ret) {
					if (copy_to_user((void __user *)arg, (void *)(&sensor_info[port]),
								sizeof(sensor_ctrl_info_t))) {
						pr_err("copy is err !\n");
						ret = -EINVAL;
					}
				}
			}
			break;
		default: {
			pr_err("ioctl cmd is err \n");
			ret = -1;
		}
			break;
	}

	return ret;
}

const struct file_operations camera_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = camera_ctrl_fop_open,
	.release = camera_ctrl_fop_release,
	.unlocked_ioctl = camera_ctrl_fop_ioctl,
	.compat_ioctl = camera_ctrl_fop_ioctl,
};

int __init camera_ctrl_init(void)
{
	int ret = 0;

	memset(update_flag, 0 , sizeof(update_flag));
	camera_ctrl = kzalloc(sizeof(camera_ctrlmod_s), GFP_KERNEL);
	if (camera_ctrl == NULL) {
		pr_err("%s --%d kzalloc !\n", __func__, __LINE__);
		return -ENOMEM;
	}
	snprintf(camera_ctrl->name, CHAR_DEVNAME_LEN, "sensor_ctrl");
	camera_ctrl->camera_chardev.name = camera_ctrl->name;
	camera_ctrl->camera_chardev.minor = MISC_DYNAMIC_MINOR;
	camera_ctrl->camera_chardev.fops = &camera_ctrl_fops;

	ret = misc_register(&camera_ctrl->camera_chardev);
	if (ret) {
		pr_err("%s --%d, register failed, err %d !\n",
					__func__, __LINE__, ret);
		goto register_err;
	}
	camera_ctrl->dev_minor_id = camera_ctrl->camera_chardev.minor;
	mutex_init(&(camera_ctrl->m_mutex));
	pr_info(" %s register success !\n", camera_ctrl->name);
	return ret;

register_err:
	kzfree(camera_ctrl);
	return ret;
}

void __exit camera_ctrl_exit(void)
{
	misc_deregister(&camera_ctrl->camera_chardev);
	kzfree(camera_ctrl);
	camera_ctrl = NULL;
	pr_info("camera_dev_exit success %d\n", __LINE__);
}

void camera_ctrldev_exit(void)
{
	camera_ctrl_exit();
}

int camera_ctrldev_init(void)
{
	int ret = 0;

	ret = camera_ctrl_init();
	if (ret < 0) {
		pr_info("camera_ctrl_init is failed\n");
	}

	return ret;
}

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("camera_ctrl dev of x3");
MODULE_LICENSE("GPL");

