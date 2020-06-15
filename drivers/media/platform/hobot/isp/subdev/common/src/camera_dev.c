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
#include "inc/camera_subdev.h"
#include "inc/camera_i2c.h"
#include "inc/camera_sys_api.h"
#include "inc/camera_spi.h"

camera_charmod_s *camera_mod[CAMERA_TOTAL_NUMBER];

static int camera_fop_open(struct inode *pinode, struct file *pfile)
{
	uint32_t tmp = 0;
	camera_charmod_s *camera_cdev = NULL;
	int minor = iminor(pinode);

	pr_info("camera_fop_open begin %d \n", __LINE__);
	for (tmp = 0; tmp < CAMERA_TOTAL_NUMBER; tmp++) {
		if(camera_mod[tmp] &&
				camera_mod[tmp]->dev_minor_id == minor ) {
				camera_cdev = camera_mod[tmp];
				pr_info(" tmp %d is open !\n", tmp);
				break;
			}
	}
	spin_lock(&camera_cdev->slock);
	if (camera_cdev->user_num > 0) {
		pr_info("more than one pthred use !\n");
	} else {
		camera_cdev->mst_file = pfile;
		camera_cdev->start_num = 0;
		mutex_init(&camera_cdev->user_mutex);
		pr_info("user_mutex init !\n");
	}
	camera_cdev->user_num++;
	spin_unlock(&camera_cdev->slock);
	pfile->private_data = camera_cdev;
	pr_info("camera_fop_open success %d\n", __LINE__);
	return 0;
}

static int camera_fop_release(struct inode *pinode, struct file *pfile)
{
	camera_charmod_s *camera_cdev = pfile->private_data;

	spin_lock(&camera_cdev->slock);
	camera_cdev->user_num--;
	camera_cdev->mst_file = NULL;
	if (camera_cdev->user_num <= 0) {
		camera_i2c_release(camera_cdev->port);
	}
	spin_unlock(&camera_cdev->slock);
	pfile->private_data = NULL;
	pr_info("camera_fop_release success %d\n", __LINE__);
	return 0;
}
static long camera_fop_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	camera_charmod_s *camera_cdev = pfile->private_data;
	sensor_turning_data_t turning_data;
	int mst_flg = 0;

	//pr_info("---[%s-%d]---\n", __func__, __LINE__);

	spin_lock(&camera_cdev->slock);
	if (camera_cdev->mst_file == NULL) {
		camera_cdev->mst_file = pfile;
		mst_flg = 1;
	} else if (camera_cdev->mst_file != pfile) {
		mst_flg = 0;
	} else {
		mst_flg = 1;
	}
	spin_unlock(&camera_cdev->slock);

	switch(cmd) {
		case SENSOR_TURNING_PARAM: {
			if (arg == 0) {
				pr_err("arg is null !\n");
				return -EINVAL;
			}
			if (mst_flg == 0) {
				pr_info("this file is not master,cmd tunning!\n");
				return 0;
			}
			if (copy_from_user((void *)&turning_data, (void __user *)arg,
				sizeof(sensor_turning_data_t))) {
				pr_err("copy is err !\n");
				return -EINVAL;
			}
			if(turning_data.bus_type == I2C_BUS) { // i2c
				camera_i2c_open(camera_cdev->port, turning_data.bus_num,
						turning_data.sensor_name, turning_data.sensor_addr);
			} else if (turning_data.bus_type == SPI_BUS) { // spi
				camera_spi_open(camera_cdev->port, turning_data.bus_num, turning_data.cs,
						turning_data.spi_mode, turning_data.spi_speed, turning_data.sensor_name);
			}
			ret = camera_sys_turining_set(camera_cdev->port, &turning_data);
		}
			break;
		case SENSOR_OPEN_CNT:
			if (copy_to_user((void __user *)arg,
				(void *)&camera_cdev->user_num,
				sizeof(uint32_t))) {
				pr_err("ioctl copy to user is error! %d\n", __LINE__);
				return -EINVAL;
			}
			break;
		case SENSOR_SET_START_CNT:
			if (copy_from_user((void *)&camera_cdev->start_num,
				(void __user *)arg, sizeof(int))) {
				pr_err("ioctl set user start count err !\n");
				spin_unlock(&camera_cdev->slock);
				return -EINVAL;
			}
			if (camera_cdev->start_num == 1)
				pr_info("ioctl sensor start %d\n", __LINE__);
			if (camera_cdev->start_num == 0)
				pr_info("ioctl sensor stop %d\n", __LINE__);
			break;
		case SENSOR_GET_START_CNT:
			if (copy_to_user((void __user *)arg,
				(void *)&camera_cdev->start_num,
				sizeof(int))) {
				pr_err("ioctl get user start count err !\n");
				return -EINVAL;
			}
			break;
		case SENSOR_USER_LOCK:
			if (mutex_lock_interruptible(&camera_cdev->user_mutex)) {
				pr_err("ioctl sensor user lock error!\n");
				return -EINVAL;
			}
			break;
		case SENSOR_USER_UNLOCK:
			mutex_unlock(&camera_cdev->user_mutex);
			break;
		case SENSOR_AE_SHARE:
			if (copy_from_user((void *)&camera_cdev->ae_share_flag,
				(void __user *)arg, sizeof(uint32_t))) {
				pr_err("ioctl ae share flag set err !\n");
				return -EINVAL;
			}
			pr_err("ae_share %d \n", camera_cdev->ae_share_flag);
			break;
		default: {
			pr_err("ioctl cmd is err \n");
			ret = -1;
		}
			break;
	}

	return ret;
}

const struct file_operations camera_fops = {
	.owner = THIS_MODULE,
	.open = camera_fop_open,
	.release = camera_fop_release,
	.unlocked_ioctl = camera_fop_ioctl,
	.compat_ioctl = camera_fop_ioctl,
};

int __init camera_dev_init(uint32_t port)
{
	int ret = 0;

	if (port > CAMERA_TOTAL_NUMBER) {
		return -ENXIO;
	}
	camera_mod[port] = kzalloc(sizeof(camera_charmod_s), GFP_KERNEL);
	if (camera_mod[port] == NULL) {
		pr_err("%s --%d kzalloc !\n", __func__, __LINE__);
		return -ENOMEM;
	}
	snprintf(camera_mod[port]->name, CHAR_DEVNAME_LEN, "port_%d", port);
	camera_mod[port]->camera_chardev.name = camera_mod[port]->name;
	camera_mod[port]->camera_chardev.minor = MISC_DYNAMIC_MINOR;
	camera_mod[port]->camera_chardev.fops = &camera_fops;

	ret = misc_register(&camera_mod[port]->camera_chardev);
	if (ret) {
		pr_err("%s --%d, register failed, err %d !\n",
					__func__, __LINE__, ret);
		goto register_err;
	}
	camera_mod[port]->dev_minor_id = camera_mod[port]->camera_chardev.minor;
	camera_mod[port]->port = port;
	spin_lock_init(&(camera_mod[port]->slock));
	pr_info("port %d %s register success !\n", port, camera_mod[port]->name);
	return ret;

register_err:
	kzfree(camera_mod[port]);
	return ret;
}

void __exit camera_dev_exit(int port)
{
	if ((port < CAMERA_TOTAL_NUMBER) && (camera_mod[port] != NULL)) {
		misc_deregister(&camera_mod[port]->camera_chardev);
		kzfree(camera_mod[port]);
		camera_mod[port] = NULL;
	}
	pr_info("camera_dev_exit success %d\n", __LINE__);
}

void camera_cdev_exit(void)
{
	uint32_t tmp = 0;

	for (tmp = 0; tmp < CAMERA_TOTAL_NUMBER; tmp++) {
		camera_dev_exit(tmp);
	}
}

int camera_cdev_init(void)
{
	int ret = 0;
	uint32_t tmp = 0;

	for (tmp = 0; tmp < CAMERA_TOTAL_NUMBER; tmp++) {
		ret = camera_dev_init(tmp);
		if (ret < 0) {
			pr_info("camera_dev_init %d is failed\n", tmp);
			goto devinit_err;
		}
	}

	return ret;
devinit_err:
	camera_cdev_exit();
	return ret;
}

// module_init(camera_cdev_init);
// module_exit(camera_cdev_exit);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("camera_char dev of x3");
MODULE_LICENSE("GPL");

