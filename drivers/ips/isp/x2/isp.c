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
static int isp_major = ISP_MAJOR;
module_param(isp_major, int, S_IRUGO);

struct isp_mod_s *pIspMod;

struct cdev isp_cdev;

static struct fasync_struct *pisp_async;

/* function */
static irqreturn_t isp_interrupt(int irq, void *dev_id)
{
	kill_fasync(&pisp_async, SIGIO, POLL_IN);
	return IRQ_HANDLED;
}

static int isp_mod_open(struct inode *pinode, struct file *pfile)
{
	int                 ret;
	struct isp_mod_s    *pdev;
	isp_dev_t           *pispdev;

	printk(KERN_INFO "isp_mod_open()!\n");

	int num = MINOR(pinode->i_rdev);

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

	return 0;
}

static int isp_mod_release(struct indoe *pinode, struct file *pfile)
{
	isp_dev_t           *pispdev;

	printk(KERN_INFO "isp_mod_release()!\n");

	pispdev = isp_get_dev();
	if (!pispdev) {
		return -ENOMEM;
	}

	free_irq(pispdev->irq, NULL);
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

static int isp_mod_ioctl(struct inode *pinode, struct file *pfile, unsigned int cmd, unsigned long arg)
{
	printk(KERN_INFO "isp_mod_ioctrl()!\n");
	return 0;
}

static int isp_mod_mmap(struct file *pfile, struct vm_area_struct *pvma)
{
	struct isp_mod_s *pmod = pfile->private_data;
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
	.owner          = THIS_MODULE,
	.open           = isp_mod_open,
	.read           = isp_mod_read,
	.write          = isp_mod_write,
	.release        = isp_mod_release,
	.unlocked_ioctl = isp_mod_ioctl,
	.compat_ioctl   = isp_mod_ioctl,
	.mmap           = isp_mod_mmap,
	.fasync         = isp_mod_fasync,
};

static int __init isp_dev_init(void)
{
	int ret, i;

	printk(KERN_INFO "isp_dev_init()\n");

	dev_t devno = MKDEV(isp_major, 0);

	if (isp_major) {
		ret = register_chrdev_region(devno, 1, "x2_isp");
	} else {
		ret = alloc_chrdev_region(&devno, 0, 1, "x2_isp");
		isp_major = MAJOR(devno);
	}

	if (ret < 0)
		return ret;

	cdev_init(&isp_cdev, &isp_mod_fops);
	isp_cdev.owner  = THIS_MODULE;

	cdev_add(&isp_cdev, devno, ISP_NR_DEVS);

	pIspMod = kmalloc(ISP_NR_DEVS * sizeof(struct isp_mod_s), GFP_KERNEL);
	if (!pIspMod) {
		ret = -ENOMEM;
		goto fail_malloc;
	}
	memset(pIspMod, 0, sizeof(struct isp_mod_s));

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

	return 0;

fail_malloc:
	unregister_chrdev_region(devno, 1);
	return ret;
}

static void __exit isp_dev_exit(void)
{
	int i;

	printk(KERN_INFO "isp_dev_exit()\n");

	cdev_del(&isp_cdev);

	for (i = 0; i < ISP_NR_DEVS; i++) {
		if (pIspMod[i].pData)
			kfree(pIspMod[i].pData);
	}

	kfree(pIspMod);

	unregister_chrdev_region(MKDEV(isp_major, 0), 1);
}

module_init(isp_dev_init);
module_exit(isp_dev_exit);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("Image Signal Process for X2 of chip");
MODULE_LICENSE("GPL");
