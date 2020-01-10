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

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

struct lens_charmod_s {
        const char name[CHARDEVNAME_LEN];
        spinlock_t slock;
        int dev_minor_id;
        struct miscdevice lens_chardev;
};

struct lens_charmod_s *lens_mod;

static int lens_fop_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	struct lens_charmod_s *lens_cdev = NULL;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	spin_lock(&lens_cdev->slock);
	pfile->private_data = lens_cdev;
	spin_unlock(&lens_cdev->slock);

	LOG(LOG_INFO, "open is success !\n");

	return ret;
}

static int lens_fop_release(struct inode *pinode, struct file *pfile)
{
	/* deinit stream */
	pfile->private_data = NULL;

	LOG(LOG_DEBUG, "---[%s-%d]--- close is success!\n",
		__func__, __LINE__);
	return 0;
}

static long lens_fop_ioctl(struct file *pfile, unsigned int cmd,
	unsigned long arg)
{
	long ret = 0;

	//struct lens_charmod_s *lens_cdev = pfile->private_data;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	switch (cmd) {
#if 0
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
	}
		break;
#endif
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

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	lens_mod = kzalloc(sizeof(struct lens_charmod_s), GFP_KERNEL);
	if (lens_mod == NULL) {
		LOG(LOG_ERR, "char dev kzalloc failed !\n");
		return -ENOMEM;
	}

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
	spin_lock_init(&(lens_mod->slock));

	return ret;
register_err:
	kzfree(lens_mod);
	return ret;
}
EXPORT_SYMBOL(lens_dev_init);

void __exit lens_dev_exit(void)
{
	LOG(LOG_DEBUG, "---lens dev exit---");

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
