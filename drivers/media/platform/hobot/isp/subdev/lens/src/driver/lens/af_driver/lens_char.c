/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
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
 *
 *    @file      lens_char.c
 *    @brief     provide char dev of lens to configure param
 *    @details 
 *    @mainpage 
 *    @author    IE&E
 *    @email     yongyong.duan@horizon.ai
 *    @version   v-1.0.0
 *    @date      2020-1-17
 *    @license
 *    @copyright
 *********************************************************************/

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
#include <linux/delay.h>
#include "lens_char.h"
#include "acamera_logger.h"
#include "lens_driver.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#endif

struct lens_charmod_s {
        char name[CHARDEVNAME_LEN];
        spinlock_t slock;
        int dev_minor_id;
        struct miscdevice lens_chardev;
};

struct lens_charmod_s *lens_mod;

static int lens_fop_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	struct lens_charmod_s *lens_cdev = NULL;

	//spin_lock(&lens_cdev->slock);
	pfile->private_data = lens_cdev;
	//spin_unlock(&lens_cdev->slock);

	LOG(LOG_INFO, "open is success!");

	return ret;
}

static int lens_fop_release(struct inode *pinode, struct file *pfile)
{
	/* deinit stream */
	pfile->private_data = NULL;

	LOG(LOG_DEBUG, "close is success!");
	return 0;
}

static long lens_fop_ioctl(struct file *pfile, unsigned int cmd,
	unsigned long arg)
{
	long ret = 0;

	//struct lens_charmod_s *lens_cdev = pfile->private_data;

	struct chardev_port_param lens_param;
	struct motor_pos_set pos_param;
	uint32_t port = 0;

	switch (cmd) {
	case LENS_AF_INIT: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&lens_param, (void __user *)arg,
			sizeof(struct chardev_port_param))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = set_af_param(lens_param.port, &lens_param);
		if (ret >= 0) {
			ret = set_af_init(lens_param.port);
		}
	}
	break;
	case LENS_ZOOM_INIT: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&lens_param, (void __user *)arg,
			sizeof(struct chardev_port_param))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = set_zoom_param(lens_param.port, &lens_param);
		if (ret >= 0) {
			ret = set_zoom_init(lens_param.port);
		}
	}
	break;
	case LENS_AF_RESET: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&port, (void __user *)arg,
			sizeof(uint32_t))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = set_af_init((uint16_t)port);
	}
	break;
	case LENS_ZOOM_RESET: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&port, (void __user *)arg,
			sizeof(uint32_t))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = set_zoom_init((uint16_t)port);
	}
	break;
	case LENS_AF_DEINIT: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&port, (void __user *)arg,
			sizeof(uint32_t))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = free_af_param((uint16_t)port);
	}
	break;
	case LENS_ZOOM_DEINIT: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&port, (void __user *)arg,
			sizeof(uint32_t))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = free_zoom_param((uint16_t)port);
	}
	break;
	case LENS_SET_I2C_PARAM: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&lens_param, (void __user *)arg,
			sizeof(struct motor_i2c_param))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
	}
	break;
	case LENS_SET_AF_POS: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&pos_param, (void __user *)arg,
			sizeof(struct motor_pos_set))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = set_af_pos(pos_param.port, pos_param.pos);
	}
	break;
	case LENS_SET_ZOOM_POS: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&pos_param, (void __user *)arg,
			sizeof(struct motor_pos_set))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		ret = set_zoom_pos(pos_param.port, pos_param.pos);
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


const struct file_operations lens_fops = {
	.owner = THIS_MODULE,
	.open = lens_fop_open,
	.release = lens_fop_release,
	.unlocked_ioctl = lens_fop_ioctl,
	.compat_ioctl = lens_fop_ioctl,
};

int __init lens_dev_init(void)
{
	int ret = 0;

	lens_mod = kzalloc(sizeof(struct lens_charmod_s), GFP_KERNEL);
	if (lens_mod == NULL) {
		LOG(LOG_ERR, "char dev kzalloc failed !\n");
		return -ENOMEM;
	}

	snprintf(lens_mod->name, CHARDEVNAME_LEN, LENS_CHARDEV_NAME);
//misc_device
	lens_mod->lens_chardev.name = lens_mod->name;
	lens_mod->lens_chardev.minor = MISC_DYNAMIC_MINOR;
	lens_mod->lens_chardev.fops = &lens_fops;

	ret = misc_register(&lens_mod->lens_chardev);
	if (ret) {
		LOG(LOG_ERR, "register failed, err %d !\n", ret);
		goto register_err;
	}
	lens_mod->dev_minor_id = lens_mod->lens_chardev.minor;
	spin_lock_init(&lens_mod->slock);

	return ret;
register_err:
	kzfree(lens_mod);
	return ret;
}
EXPORT_SYMBOL(lens_dev_init);

void __exit lens_dev_exit(void)
{
	if (lens_mod != NULL) {
		misc_deregister(&lens_mod->lens_chardev);
		kzfree(lens_mod);
		lens_mod = NULL;
	}
}
EXPORT_SYMBOL(lens_dev_exit);

MODULE_AUTHOR("yongyong.duan@horizon.ai");
MODULE_DESCRIPTION("lens char dev of x3");
MODULE_LICENSE("GPL v2");
