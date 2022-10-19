/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *  Author: leye.wang<leye.wang@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <asm/ioctl.h>

#define HMIPC_IOCTL_MAGIC	'N'
#define HMIPC_SET_COUNT	(_IOW(HMIPC_IOCTL_MAGIC, 0, int))

// #define POLL_DEBUG

#undef PDEBUG_HMIPC
#ifdef POLL_DEBUG
#define PDEBUG_HMIPC(fmt, ...)		printk(KERN_ERR "[poll]:"fmt, ##__VA_ARGS__)
#else
#define PDEBUG_HMIPC(fmt, ...)
#endif

#define HMIPC_DEV_NAME		"hmipc"

struct hmipc_channel {
	struct cdev cdev;
	struct class *class;
	struct device *device;

	wait_queue_head_t wq_head;

	struct mutex hmipc_mutex;
	int32_t count;

	dev_t devno;
};

struct hmipc_channel *g_hmipc_channel = NULL;

static int hmipc_open(struct inode *inode, struct file *filp)
{
	struct hmipc_channel *channel_ctx;

	channel_ctx = kzalloc(sizeof(struct hmipc_channel), GFP_KERNEL);
	if (channel_ctx == NULL) {
		pr_err("struct hmipc_channel allocate fail\n");
		return -ENOMEM;
	}
	mutex_init(&channel_ctx->hmipc_mutex);
	init_waitqueue_head(&channel_ctx->wq_head);
	filp->private_data = channel_ctx;
	// filp->private_data = g_hmipc_channel;
	PDEBUG_HMIPC("%s:%d\n", __func__, __LINE__);

	return 0;
}

static int hmipc_close(struct inode *inode, struct file *filp)
{
	struct hmipc_channel *channel = filp->private_data;
	kfree(channel);
	return 0;
}

static unsigned int hmipc_poll(struct file *filp,
		struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct hmipc_channel *channel = filp->private_data;

	mutex_lock(&channel->hmipc_mutex);

	poll_wait(filp, &channel->wq_head, wait);

	PDEBUG_HMIPC("[%s]channel->count = %d\n", __func__, channel->count);

	if (channel->count > 0) {
		mask |= POLLIN | POLLRDNORM;

		/* decrease each time */
		// channel->count--;
	}
	mutex_unlock(&channel->hmipc_mutex);

	return mask;
}

static long hmipc_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct hmipc_channel *channel = filp->private_data;
	int cnt;

	switch (cmd) {
		case HMIPC_SET_COUNT: /*PRQA S ALL*/
			mutex_lock(&channel->hmipc_mutex);
			if (copy_from_user(&cnt, (void __user *)arg, _IOC_SIZE(cmd))) {
				mutex_unlock(&channel->hmipc_mutex);
				pr_err("copy_from_user fail:%d\n", __LINE__);
				return -EFAULT;
			}
			PDEBUG_HMIPC("set value = %d\n", cnt);

			if (channel->count == 0) {
				wake_up_interruptible(&channel->wq_head);
				PDEBUG_HMIPC("call wake_up_interruptible");
			}

			/* update count */
			channel->count += cnt;
			if (cnt == 0)
				channel->count = 0;
			if (channel->count < 0)
				channel->count = 0;

			PDEBUG_HMIPC("[%s]channel->count = %d\n", __func__, channel->count);

			mutex_unlock(&channel->hmipc_mutex);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static struct file_operations hmipc_fops = {
	.owner = THIS_MODULE,
	.open = hmipc_open,
	.release = hmipc_close,
	.poll = hmipc_poll,
	.unlocked_ioctl = hmipc_ioctl,
	.compat_ioctl = hmipc_ioctl,
};

static int __init hmipc_init(void)
{
	int ret;

	if (g_hmipc_channel == NULL) {
		g_hmipc_channel = (struct hmipc_channel *)kzalloc(
				sizeof(struct hmipc_channel), GFP_KERNEL);
		if (g_hmipc_channel == NULL) {
			pr_err("struct hmipc_channel allocate fail\n");
			return -1;
		}
	}

	/* allocate device number */
	ret = alloc_chrdev_region(&g_hmipc_channel->devno, 0, 1, HMIPC_DEV_NAME);
	if (ret < 0) {
		pr_err("alloc_chrdev_region fail:%d\n", ret);
		goto alloc_chrdev_err;
	}

	/* set char-device */
	cdev_init(&g_hmipc_channel->cdev, &hmipc_fops);
	g_hmipc_channel->cdev.owner = THIS_MODULE;
	ret = cdev_add(&g_hmipc_channel->cdev, g_hmipc_channel->devno, 1);
	if (ret < 0) {
		pr_err("cdev_add fail:%d\n", ret);
		goto cdev_add_err;
	}

	/* create device */
	g_hmipc_channel->class = class_create(THIS_MODULE, HMIPC_DEV_NAME); /*PRQA S 3237*/
	if (IS_ERR(g_hmipc_channel->class)) {
		pr_err("class_create fail\n");
		goto class_create_err;
	}
	g_hmipc_channel->device = device_create(g_hmipc_channel->class, NULL,
			g_hmipc_channel->devno, NULL, HMIPC_DEV_NAME);
	if (IS_ERR(g_hmipc_channel->device)) {
		pr_err("device_create fail\n");
		goto device_create_err;
	}

	// mutex_init(&g_hmipc_channel->hmipc_mutex);
	// init_waitqueue_head(&g_hmipc_channel->wq_head);

	PDEBUG_HMIPC("%s:%d\n", __func__, __LINE__);

	return 0;

device_create_err:
	class_destroy(g_hmipc_channel->class);
class_create_err:
	cdev_del(&g_hmipc_channel->cdev);
cdev_add_err:
	unregister_chrdev_region(g_hmipc_channel->devno, 1);
alloc_chrdev_err:
	kfree(g_hmipc_channel);
	g_hmipc_channel = NULL;
	return -1;
}

static void __exit hmipc_exit(void)
{
	cdev_del(&g_hmipc_channel->cdev);
	device_destroy(g_hmipc_channel->class, g_hmipc_channel->devno);
	unregister_chrdev_region(g_hmipc_channel->devno, 1);
	class_destroy(g_hmipc_channel->class);

	kfree(g_hmipc_channel);
	g_hmipc_channel = NULL;
}

module_init(hmipc_init); /*PRQA S 0605*/
module_exit(hmipc_exit); /*PRQA S 0605*/

MODULE_DESCRIPTION("hmipc driver for asynchronous notification");
MODULE_AUTHOR("leye.wang@horizon.ai");
MODULE_LICENSE("GPL");
