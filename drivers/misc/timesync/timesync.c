/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/unistd.h>

#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>

#include <linux/hrtimer.h>
#include <linux/cdev.h>

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("zhen.zhong");
MODULE_DESCRIPTION("timesync module");

#define timesync_IOC_MAGIC  (0x6A)

#define timesync_TIMESTAMP_TRIG     _IO(timesync_IOC_MAGIC, 8)
#define timesync_GET_TIMESTAMP      _IO(timesync_IOC_MAGIC, 9)

#define j22mcu_j2_send_irq_gpio         (118)
#define mcu2j2_j2_recv_irq_gpio		(18)
DEFINE_SPINLOCK(timesync_spinlock);
unsigned long spinlock_flags;
struct timesync_data {
	int gpio_timestamp;
	int gpio_timestamp_irq;
	int eint_timestamp_irq;
};
struct timesync_data timesyncdata;

struct kfifo timestamp_kfifo;

static int timesync_major;
static struct cdev timesync_cdev;
static struct class  *timesync_class;
static struct device *timesync_dev;


struct hrtimer hrtimer_10ms_timer;

ktime_t kt;
struct timeval tv_global;

static enum hrtimer_restart  hrtimer_10ms_timer_poll(struct hrtimer *timer)
{
	struct timesync_data *timesyndata_temp = &timesyncdata;

	gpio_direction_output(timesyndata_temp->gpio_timestamp, 1);

	return HRTIMER_NORESTART;
}


static irqreturn_t x2_eint_timestamp_irq(int irq, void *dev_id)
{
	struct timeval *tv = &tv_global;

	struct timesync_data *timesyndata_temp = &timesyncdata;


	spin_lock_irqsave(&timesync_spinlock, spinlock_flags);
	do_gettimeofday(tv);
	gpio_direction_output(timesyndata_temp->gpio_timestamp, 0);
	kt = ms_to_ktime(10);

	hrtimer_start(&hrtimer_10ms_timer, kt, HRTIMER_MODE_REL);
	hrtimer_10ms_timer.function = hrtimer_10ms_timer_poll;

	spin_unlock_irqrestore(&timesync_spinlock, spinlock_flags);
	return IRQ_HANDLED;
}


static int timesync_open(struct inode *inode, struct file *filp)

{
	return 0;
}

#if 1
static int timesync_release(struct inode *inode, struct file *filp)
{
	return 0;
}
#endif

static long timesync_ioctrl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct timeval tv_temp;
	int	retval = 0;
	int copied = 0;
	struct timesync_data *timesyndata_temp = &timesyncdata;

	switch (cmd) {
	case timesync_GET_TIMESTAMP:
		spin_lock_irqsave(&timesync_spinlock, spinlock_flags);
		tv_temp = tv_global;
		spin_unlock_irqrestore(&timesync_spinlock, spinlock_flags);
		copied = copy_to_user((void __user *)arg,
			&tv_temp, sizeof(struct timeval));
		if (copied)
			retval = -1;
	break;
	case timesync_TIMESTAMP_TRIG:
		gpio_direction_output(timesyndata_temp->gpio_timestamp, 0);
		msleep(20);
		gpio_direction_output(timesyndata_temp->gpio_timestamp, 1);
	break;
	default:
	break;
	}
	return retval;
}

static const struct file_operations timesync_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.unlocked_ioctl = timesync_ioctrl,
	.open =		timesync_open,
	.release =	timesync_release,
};
static int timesync_probe(struct platform_device *pdev)
{
	int	ret = 0;
	dev_t	devno;
	int	status = -ENXIO;
	unsigned long flags =  IRQF_TRIGGER_FALLING;
	struct cdev  *p_cdev = &timesync_cdev;
	struct timesync_data *timesyndata_temp = &timesyncdata;

	timesync_major = 0;
	pr_err("timesync probe begin!\n");
	ret = of_property_read_u32(pdev->dev.of_node,
	"timesync_MCU2J2_irq_pin", &timesyndata_temp->gpio_timestamp_irq);
	if (ret) {
		pr_err("get timesync_MCU2J2_irq_pin error\n");
		goto get_timesync_MCU2J2_irq_pin_error;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
	"timesync_J22MCU_irq_pin", &timesyndata_temp->gpio_timestamp);
	if (ret) {
		pr_err("get timesync_J22MCU_irq_pin error\n");
		goto get_timesync_J22MCU_irq_pin_error;
	}

	ret = alloc_chrdev_region(&devno, 0, 1, "x2_timesync");
	if (ret < 0) {
		pr_err("Error %d while alloc chrdev timesync\n", ret);
		goto alloc_chrdev_error;
	}
	timesync_major = MAJOR(devno);
	cdev_init(p_cdev, &timesync_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		pr_err("Error %d while adding x2 timesync cdev", ret);
		goto cdev_add_error;
	}
	timesync_class = class_create(THIS_MODULE, "x2_timesync");
	if (IS_ERR(timesync_class)) {
		pr_err("[%s:%d] class_create error\n",
			__func__, __LINE__);
		ret = PTR_ERR(timesync_class);
		goto class_create_error;
	}
	timesync_dev = device_create(timesync_class, NULL,
		MKDEV(timesync_major, 0), NULL, "x2_timesync");


	status = gpio_request(
				timesyndata_temp->gpio_timestamp,
				"timesync-timestamp");

	if (status) {
		pr_err("request gpio_timestamp fail\n");
		goto err_request_timestamp_irq_gpio;
	}
	gpio_direction_output(timesyndata_temp->gpio_timestamp, 1);

	status = gpio_request(
		timesyndata_temp->gpio_timestamp_irq,
		"timesync-timestamp-irq");

	if (status) {
		pr_err("request gpio_timestamp_irq fail\n");
		goto err_request_timestamp_irq_gpio;
	}
	timesyndata_temp->eint_timestamp_irq =
			gpio_to_irq(timesyndata_temp->gpio_timestamp_irq);

	irq_set_irq_type(
			timesyndata_temp->eint_timestamp_irq,
			IRQ_TYPE_EDGE_FALLING);

	status = request_threaded_irq(
		timesyndata_temp->eint_timestamp_irq, x2_eint_timestamp_irq,
		NULL, flags | IRQF_ONESHOT, "x2_timesync",
		(void *)timesyndata_temp);

	if (status) {
		pr_err("request timestamp interrupt fail %d\n", status);
		goto err_request_timestamp_interrupt;
	}
	hrtimer_init(&hrtimer_10ms_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pr_err("timesync probe success!\n");
	return 0;

err_request_timestamp_irq_gpio:
	gpio_free(timesyndata_temp->gpio_timestamp);
err_request_timestamp_interrupt:
	gpio_free(timesyndata_temp->gpio_timestamp_irq);
class_create_error:
	cdev_del(&timesync_cdev);
cdev_add_error:
	unregister_chrdev_region(MKDEV(timesync_major, 0), 1);
alloc_chrdev_error:

get_timesync_J22MCU_irq_pin_error:

get_timesync_MCU2J2_irq_pin_error:

	pr_err("init failed!\n");
	return -1;
}
static int timesync_remove(struct platform_device *pdev)
{
	struct timesync_data *timesyncdev_temp = &timesyncdata;

	gpio_free(timesyncdev_temp->gpio_timestamp);

	free_irq(timesyncdev_temp->eint_timestamp_irq,
			(void *)timesyncdev_temp);

	gpio_free(timesyncdev_temp->gpio_timestamp_irq);
	kfifo_free(&timestamp_kfifo);
	device_destroy(timesync_class, MKDEV(timesync_major, 0));
	class_destroy(timesync_class);
	cdev_del(&timesync_cdev);
	unregister_chrdev_region(MKDEV(timesync_major, 0), 1);
	return 0;

}


static const struct of_device_id timesync_of_match[] = {
	{.compatible = "timesync",},
	{},
};
	static struct platform_driver timesync_driver = {
		.probe	  = timesync_probe,
		.remove   = timesync_remove,
		.driver   = {
				.owner	= THIS_MODULE,
				.name	= "timesync",
				.of_match_table = timesync_of_match,
		},
	};


static int __init timesync_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&timesync_driver);
	if (ret)
		pr_err("register timesync_driver error\n");
	return ret;
}

static void __exit timesync_exit(void)
{
	platform_driver_unregister(&timesync_driver);
}

module_init(timesync_init);
module_exit(timesync_exit);
