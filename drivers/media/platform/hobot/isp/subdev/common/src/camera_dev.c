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
#include <linux/list.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/spinlock_types.h>
#include <linux/wait.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>

#include "inc/camera_dev.h"
#include "inc/camera_subdev.h"
#include "inc/acamera_logger.h"
#include "inc/camera_i2c.h"
#include "inc/camera_sys_api.h"
#include "inc/camera_spi.h"

camera_charmod_s *camera_mod[CAMERA_TOTAL_NUMBER];

static int camera_fop_open(struct inode *pinode, struct file *pfile)
{
	uint32_t tmp = 0;
	camera_charmod_s *camera_cdev = NULL;
	int minor = iminor(pinode);

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	for (tmp = 0; tmp < CAMERA_TOTAL_NUMBER; tmp++) {
		if(camera_mod[tmp] &&
				camera_mod[tmp]->dev_minor_id == minor ) {
				camera_cdev = camera_mod[tmp];
				printk(KERN_INFO " tmp %d is open !\n", tmp);
				break;
			}
	}
	spin_lock(&camera_cdev->slock);
	if (camera_cdev->user_num > 0) {
		spin_unlock(&camera_cdev->slock);
		printk(KERN_INFO " more than one pthred use !\n");
		return -ENXIO;
	}
	camera_cdev->user_num++;
	spin_unlock(&camera_cdev->slock);
	pfile->private_data = camera_cdev;

	printk(KERN_INFO "dma_ops1  is %p !\n",
		get_dma_ops(camera_mod[tmp]->camera_chardev.this_device));
	LOG(LOG_INFO, "open is success !\n");

	return 0;
}

static int camera_fop_release(struct inode *pinode, struct file *pfile)
{
	camera_charmod_s *camera_cdev = pfile->private_data;

	spin_lock(&camera_cdev->slock);
	camera_cdev->user_num--;
	spin_unlock(&camera_cdev->slock);
	pfile->private_data = NULL;

	LOG(LOG_DEBUG, "---[%s-%d]--- close is success!\n",
		__func__, __LINE__);
	return 0;
}
static long camera_fop_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	camera_charmod_s *camera_cdev = pfile->private_data;
	sensor_turning_data_t turning_data;
	struct i2c_client *client = camera_cdev->client;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	switch (cmd) {
		case SENSOR_TURNING_PARAM: {
			if (arg == 0) {
				LOG(LOG_ERR, "arg is null !\n");
				return -EINVAL;
			}
			if (copy_from_user((void *)&turning_data, (void __user *)arg,
				sizeof(sensor_turning_data_t))) {
				LOG(LOG_ERR, "copy is err !\n");
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
		case X2A_VIDIOC_I2CREAD: {
			x2a_camera_i2c_t  reg = {0, };
			uint8_t data = 0;
			if (!arg) {
				printk(KERN_INFO "x2a camera i2c read error, reg should not be NULL\n");
				return -EINVAL;
			}
			ret = copy_from_user((void *)&reg, (void __user *)arg,
								sizeof(x2a_camera_i2c_t));
			if (ret) {
				printk(KERN_INFO "x2a camera i2c read error \n");
				return -EINVAL;
			}
			if (reg.reg_size == 1) {
				if (camera_user_i2c_read_byte(client, reg.i2c_addr,
							(uint8_t)reg.reg, (uint8_t *)&data)) {
					printk(KERN_INFO "i2c read error\n");
					return -EINVAL;
				}
			} else {
				if (camera_user_i2c_read(client, reg.i2c_addr, reg.reg, &data)) {
					printk(KERN_INFO "i2c read error\n");
					return -EINVAL;
				}
			}
			reg.data = data;
			if (copy_to_user((void __user *)arg, (void *)&reg,
						sizeof(x2a_camera_i2c_t))) {
				printk(KERN_INFO "x2a camera i2c read error, copy data failed \n");
				return -EINVAL;
			}
		}
			break;
	case X2A_VIDIOC_I2CWRITE: {
			x2a_camera_i2c_t  reg = {0, };
			if (!arg) {
				printk(KERN_INFO "x2a camera i2c write error, reg should not be NULL\n");
				return -EINVAL;
			}
			ret = copy_from_user((void *)&reg, (void __user *)arg,
						sizeof(x2a_camera_i2c_t));
			if (ret) {
				printk(KERN_INFO "x2a camera i2c write error\n");
				return -EINVAL;
			}
			if (reg.reg_size == 1) {
				if (camera_user_i2c_write_byte(client, reg.i2c_addr, (uint8_t)reg.reg,
												(uint8_t)reg.data)) {
					printk(KERN_INFO "i2c write error\n");
					return -EINVAL;
				}
			} else {
				if (camera_user_i2c_write(client, reg.i2c_addr, (uint16_t)reg.reg,
											(uint8_t)reg.data)) {
					printk(KERN_INFO "i2c write error\n");
					return -EINVAL;
				}
			}
			// printk(KERN_INFO "x2a camera i2c write 0x%x 0x%x: 0x%x\n",
			// reg.i2c_addr, reg.reg, reg.data);
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
	int rc = 0;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	if (port > CAMERA_TOTAL_NUMBER) {
		return -ENXIO;
	}

	camera_mod[port] = kzalloc(sizeof(camera_charmod_s), GFP_KERNEL);
	if (camera_mod[port] == NULL) {
		printk(KERN_INFO "%s --%d kzalloc !\n", __func__, __LINE__);
		return -ENOMEM;
	}
	snprintf(camera_mod[port]->name, CHAR_DEVNAME_LEN, "camera_%d", port);
	// misc_device
	camera_mod[port]->camera_chardev.name = camera_mod[port]->name;
	camera_mod[port]->camera_chardev.minor = MISC_DYNAMIC_MINOR;
	camera_mod[port]->camera_chardev.fops = &camera_fops;

	ret = misc_register(&camera_mod[port]->camera_chardev);
	if (ret) {
		printk(KERN_INFO "%s --%d, register failed, err %d !\n",
					__func__, __LINE__, ret);
		goto register_err;
	}
	printk(KERN_INFO "%s --%d port %d camera_mod[port]->name %s", __func__, __LINE__, port, camera_mod[port]->name);
	camera_mod[port]->dev_minor_id = camera_mod[port]->camera_chardev.minor;
	camera_mod[port]->port = port;
	spin_lock_init(&(camera_mod[port]->slock));
	LOG(LOG_INFO, "%s register success !\n", camera_mod[port]->name);
	return ret;

register_err:
	kzfree(camera_mod[port]);
	return ret;
}

void __exit camera_dev_exit(int port)
{
	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);
	if ((port < CAMERA_TOTAL_NUMBER) && (camera_mod[port] != NULL)) {
		misc_deregister(&camera_mod[port]->camera_chardev);
		kzfree(camera_mod[port]);
		camera_mod[port] = NULL;
	}
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
			printk(KERN_INFO "camera_dev_init %d is failed\n", tmp);
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
MODULE_DESCRIPTION("camera_char dev of x2a");
MODULE_LICENSE("GPL");

