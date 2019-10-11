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

#include <linux/device.h>
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


#include "system_dwe_api.h"
#include "dwe_char.h"

dwe_charmod_s *dwe_mod[FIRMWARE_CONTEXT_NUMBER];

static int dwe_fop_open(struct inode *pinode, struct file *pfile)
{
	uint32_t tmp = 0;
	dwe_charmod_s *dwe_cdev = NULL;
	int minor = iminor( pinode );
	
	
	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER; tmp++) {
		if ( dwe_mod[tmp]->dev_minor_id == minor ) {
			dwe_cdev = dwe_mod[tmp];
			break;
		}
	}

	if (dwe_cdev == NULL ) {
		return -EINVAL;
	}

	if (dwe_cdev->user_num > 0) {
		return -ENXIO;	
	}

	spin_lock(&dwe_cdev->slock);	
	dwe_cdev->user_num++;
	pfile->private_data = dwe_cdev;
	spin_unlock(&dwe_cdev->slock);
	
	return 0;
}

static int dwe_fop_release(struct inode *pinode, struct file *pfile)
{
	dwe_charmod_s *dwe_cdev = pfile->private_data;
	
	spin_lock(&dwe_cdev->slock);	
	dwe_cdev->user_num--;
	spin_unlock(&dwe_cdev->slock);
	pfile->private_data = NULL;

	return 0;
}

static ssize_t dwe_fop_read(struct file *pfile, char *puser_buf,
	size_t len, loff_t *poff)
{
	int ret = 0;

	dwe_charmod_s *dwe_cdev = pfile->private_data;
	
	return ret;
}

static ssize_t dwe_fop_write(struct file *pfile, const char *puser_buf,
			     size_t len, loff_t *poff)
{
	dwe_charmod_s *dwe_cdev = pfile->private_data;

	return 0;
}

long dwe_fop_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	dwe_charmod_s *dwe_cdev = pfile->private_data;

	switch (cmd) {
	case DWEC_SET_DIS_PARAM:
		break;
	case DWEC_SET_LDC_PARAM:
		break;
	case DWEC_GET_DIS_PARAM:
		break;
	case DWEC_GET_LDC_PARAM:
		break;
	default:
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
};

int __init dwe_dev_init(uint32_t port)
{
	int ret = 0;

	if (port > FIRMWARE_CONTEXT_NUMBER) {
		return -ENXIO;
	}

	dwe_mod[port] = kzalloc(sizeof(dwe_charmod_s), GFP_KERNEL);
	if (dwe_mod[port] == NULL) {
		printk(KERN_INFO "%s --%d kzalloc !\n", __func__, __LINE__);
		return -ENOMEM;
	}

	snprintf( dwe_mod[port]->name, CHARDEVNAME_LEN, "dwe_sbuf%d", port );
	
	dwe_mod[port]->dwe_chardev.name = dwe_mod[port]->name;
	dwe_mod[port]->dwe_chardev.minor = MISC_DYNAMIC_MINOR;
	dwe_mod[port]->dwe_chardev.fops = &dwe_fops;

	ret = misc_register( &dwe_mod[port]->dwe_chardev );
	if ( ret ) {
		printk(KERN_INFO "%s --%d, register failed, err %d !\n", __func__, __LINE__, ret);
		goto register_err;
	}

	dwe_mod[port]->dev_minor_id = dwe_mod[port]->dwe_chardev.minor;
	spin_lock_init( &( dwe_mod[port]->slock ) );

	printk(KERN_INFO "%s register success !\n", dwe_mod[port]->name);
	return ret;

register_err:
	kzfree(dwe_mod[port]);

	return ret;
}
EXPORT_SYMBOL(dwe_dev_init);

void __exit dwe_dev_exit(int port)
{
	if ((port < FIRMWARE_CONTEXT_NUMBER) && (dwe_mod[port] != NULL)) {
		misc_deregister( &dwe_mod[port]->dwe_chardev );
		kzfree(dwe_mod[port]);
		dwe_mod[port] = NULL;
	}
}
EXPORT_SYMBOL(dwe_dev_exit);


MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("dwe_char dev of x2a");
MODULE_LICENSE("GPL");
