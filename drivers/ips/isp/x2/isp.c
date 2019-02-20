/*
	isp.c - driver, char device interface

	Copyright (C) 2018 Horizon Inc.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
*/

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include "isp.h"
#include "isp_dev.h"

/* global variable define */
struct isp_mod_s *pIspMod;

struct cdev isp_cdev;

static struct fasync_struct *pisp_async;

typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define ISP_READ	_IOWR('p', 0, reg_t)
#define ISP_WRITE	_IOW('p', 1, reg_t)

/* function */
static irqreturn_t isp_interrupt(int irq, void *dev_id)
{
	kill_fasync(&pisp_async, SIGIO, POLL_IN);
	return IRQ_HANDLED;
}

static int isp_mod_open(struct inode *pinode, struct file *pfile)
{
	printk(KERN_INFO "isp_mod_open()!\n");
	#if 0
	if (num >= ISP_NR_DEVS)
		return -ENODEV;

	pispdev = isp_get_dev();
	if (!pispdev) {
		return -ENOMEM;
	}

	pdev = &pIspMod[num];

	pfile->private_data = pdev;

	ret = request_irq(pispdev->irq, isp_interrupt, 0, ISP_NAME, NULL);
	if (ret)
		return -ENODEV;
	#endif
	return 0;
}

static int isp_mod_release(struct inode *pinode, struct file *pfile)
{
	printk(KERN_INFO "isp_mod_release()!\n");
	#if 0
	pispdev = isp_get_dev();
	if (!pispdev) {
		return -ENOMEM;
	}

	free_irq(pispdev->irq, NULL);
	#endif
	return 0;
}

static ssize_t isp_mod_read(struct file *pfile, char *puser_buf, size_t len, loff_t *poff)
{
	printk(KERN_INFO "isp_mod_read()!\n");
	return 0;
}

static ssize_t isp_mod_write(struct file *pfile, const char *puser_buf, size_t len, loff_t *poff)
{
	printk(KERN_INFO "isp_mod_write()!\n");
	return 0;
}

static long isp_mod_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	isp_dev_t *pIspDev = isp_get_dev();
	void __iomem *iomem = pIspDev->regbase;
	reg_t		   reg;

	if ( NULL == iomem ) {
		printk(KERN_ERR "x2 isp no iomem\n");
		return -EINVAL;
	}
	switch (cmd) {
	case ISP_READ:
		if (!arg) {
			printk(KERN_ERR "x2 isp reg read error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR "x2 isp reg read error, copy data from user failed\n");
			return -EINVAL;
		}
		reg.value = readl(iomem + reg.offset);
		if ( copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg)) ) {
			printk(KERN_ERR "x2 isp reg read error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case ISP_WRITE:
		if ( !arg ) {
			printk(KERN_ERR "x2 isp reg write error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR "x2 isp reg write error, copy data from user failed\n");
			return -EINVAL;
		}
		//printk("addr:0x%x, value:0x%x",iomem + reg.offset, reg.value);
		writel(reg.value, iomem + reg.offset );
		break;
	default:

		break;
	}
	return 0;
}

static int isp_mod_mmap(struct file *pfile, struct vm_area_struct *pvma)
{
	isp_dev_t *pispdev = isp_get_dev();

	printk(KERN_INFO "isp_mod_mmap()!\n");

	if (!pispdev) {
		return -ENOMEM;
	}

	pvma->vm_flags |= VM_IO;
	pvma->vm_flags |= VM_LOCKED;

	if (remap_pfn_range(pvma, pvma->vm_start, pispdev->mapbase >> PAGE_SHIFT, pvma->vm_end - pvma->vm_start, pvma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
}

static int isp_mod_fasync(int fd, struct file *pfile, int on)
{
	printk(KERN_INFO "isp_mod_fasync()\n");
	return fasync_helper(fd, pfile, on, &pisp_async);
}

struct file_operations isp_mod_fops = {
	.owner			= THIS_MODULE,
	.open			= isp_mod_open,
	.read			= isp_mod_read,
	.write			= isp_mod_write,
	.release		= isp_mod_release,
	.unlocked_ioctl = isp_mod_ioctl,
	.mmap			= isp_mod_mmap,
	.fasync 		= isp_mod_fasync,
};

static int __init isp_dev_init(void)
{
	int ret = 0;
	int error;

	printk(KERN_INFO "isp_dev_init()\n");
	pIspMod = kmalloc(ISP_NR_DEVS * sizeof(struct isp_mod_s), GFP_KERNEL);
	if (!pIspMod) {
		ret = -ENOMEM;
		goto fail_malloc;
	}
	memset(pIspMod, 0, sizeof(struct isp_mod_s));
#if 0
	for (i = 0; i < ISP_NR_DEVS; i++) {
		pIspMod[i].size    = ISP_DEV_SIZE;
		pIspMod[i].pData   = kmalloc(ISP_DEV_SIZE, GFP_KERNEL);
		if (!pIspMod[i].pData) {
			kfree(pIspMod);
			ret = -ENOMEM;
			goto fail_malloc;
		}
		memset(pIspMod[i].pData, 0, sizeof(ISP_DEV_SIZE));
	}
#endif
	pIspMod->isp_classes = class_create(THIS_MODULE, "x2_isp");
	if (IS_ERR(pIspMod->isp_classes))
		return PTR_ERR(pIspMod->isp_classes);

	error = alloc_chrdev_region(&pIspMod->dev_num, 0, 1, "x2_isp");

	if (!error) {
		pIspMod->major = MAJOR(pIspMod->dev_num);
		pIspMod->minor = MINOR(pIspMod->dev_num);
	}

	if (ret < 0)
		return ret;

	cdev_init(&isp_cdev, &isp_mod_fops);
	isp_cdev.owner	= THIS_MODULE;

	cdev_add(&isp_cdev, pIspMod->dev_num, ISP_NR_DEVS);

	device_create(pIspMod->isp_classes, NULL, pIspMod->dev_num, NULL, "x2_isp");
	if (ret)
		return ret;

	return 0;

fail_malloc:
	unregister_chrdev_region(pIspMod->dev_num, 1);
	return ret;
}

static void __exit isp_dev_exit(void)
{
	printk(KERN_INFO "isp_dev_exit()\n");

	cdev_del(&isp_cdev);
#if 0
	for (i = 0; i < ISP_NR_DEVS; i++) {
		if (pIspMod[i].pData)
			kfree(pIspMod[i].pData);
	}
#endif
	kfree(pIspMod);

	unregister_chrdev_region(pIspMod->dev_num, 1);
}

module_init(isp_dev_init);
module_exit(isp_dev_exit);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("Image Signal Process for X2 of chip");
MODULE_LICENSE("GPL");
