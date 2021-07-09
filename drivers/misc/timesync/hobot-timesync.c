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
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#define timesync_IOC_MAGIC  		(0x6A)
#define timesync_TIMESTAMP_TRIG    	_IO(timesync_IOC_MAGIC, 8)
#define timesync_GET_TIMESTAMP      	_IO(timesync_IOC_MAGIC, 9)

#define HOBOT_TIMESYNC			"hobot_timesync"

struct hb_timesync {
	struct device *dev;
	struct miscdevice miscdev;

	int irq_pin;
	int irq_num;
#ifdef CONFIG_TIMESYNC_DEBUG
	int debug_pin;
#endif

	struct timeval tv;
};

static irqreturn_t timesync_irq(int irq, void *dev)
{
	struct hb_timesync *timesync = dev;

	do_gettimeofday(&timesync->tv);

#ifdef CONFIG_TIMESYNC_DEBUG
	gpio_direction_output(timesync->debug_pin, 0);
	mdelay(20);
	gpio_direction_output(timesync->debug_pin, 1);
#endif

	return IRQ_HANDLED;
}

static int timesync_open(struct inode *inode, struct file *filp)
{
	struct hb_timesync *timesync = 
		container_of(filp->private_data, struct hb_timesync, miscdev);

	filp->private_data = timesync;

	return 0;
}

static int timesync_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static long timesync_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct hb_timesync *timesync = filp->private_data;
	struct timeval tmp_tv;

	switch (cmd) {
	case timesync_GET_TIMESTAMP:
		disable_irq(timesync->irq_num);
		memcpy(&tmp_tv, &timesync->tv, sizeof(timesync->tv));
		enable_irq(timesync->irq_num);

		if (copy_to_user((void __user *)arg, &tmp_tv, sizeof(tmp_tv))) {
			dev_err(timesync->dev, "copy timeval failed\n");
			ret = -EFAULT;
		}
		break;
#ifdef CONFIG_TIMESYNC_DEBUG
	case timesync_TIMESTAMP_TRIG:
		gpio_direction_output(timesync->debug_pin, 0);
		mdelay(20);
		gpio_direction_output(timesync->debug_pin, 1);
		break;
#endif
	default:
		dev_err(timesync->dev, "invalid cmd\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations timesync_fops = {
	.owner =	THIS_MODULE,
	.open =		timesync_open,
	.release =	timesync_release,
	.unlocked_ioctl = timesync_ioctl,
};

static int timesync_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct hb_timesync *timesync = NULL;	

	dev_info(&pdev->dev, "hobot timesync probe begin\n");

	timesync = devm_kzalloc(&pdev->dev, sizeof(*timesync), GFP_KERNEL);
	if (!timesync) {
		dev_err(&pdev->dev, "memory alloc failed\n");
		return -ENOMEM;
	}

	timesync->dev = &pdev->dev;

	ret = of_property_read_u32(pdev->dev.of_node,
			"timesync_irq_pin", &timesync->irq_pin);
	if (ret) {
		dev_err(&pdev->dev, "get irq pin failed\n");
		return ret;
	}

#ifdef CONFIG_TIMESYNC_DEBUG
	ret = of_property_read_u32(pdev->dev.of_node,
			"timesync_debug_pin", &timesync->debug_pin);
	if (ret) {
		dev_err(&pdev->dev, "get debug pin failed\n");
		return ret;
	}
#endif

	timesync->miscdev.minor = MISC_DYNAMIC_MINOR;
	timesync->miscdev.name = HOBOT_TIMESYNC;
	timesync->miscdev.fops = &timesync_fops;
	ret = misc_register(&timesync->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "device register failed\n");
		return ret;
	}

	ret = gpio_request(timesync->irq_pin, "irq-pin");
	if (ret) {
		dev_err(&pdev->dev, "gpio request failed\n");
		goto irq_pin_request_err;
	}
	timesync->irq_num = gpio_to_irq(timesync->irq_pin);
	dev_info(&pdev->dev, "irq_pin = %d, irq_num = %d\n",
			timesync->irq_pin, timesync->irq_num);
	ret = request_threaded_irq(timesync->irq_num, timesync_irq, NULL,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, HOBOT_TIMESYNC, timesync);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed\n");
		goto request_irq_err;
	}

#ifdef CONFIG_TIMESYNC_DEBUG
	ret = gpio_request(timesync->debug_pin, "debug-pin");
	if (ret) {
		dev_err(&pdev->dev, "gpio request failed\n");
		goto debug_pin_request_err;
	}
	gpio_direction_output(timesync->debug_pin, 1);
#endif

	platform_set_drvdata(pdev, timesync);

	dev_info(&pdev->dev, "hobot timesync probe success\n");

	return 0;

#ifdef CONFIG_TIMESYNC_DEBUG
debug_pin_request_err:
#endif
	free_irq(timesync->irq_num, timesync);
request_irq_err:
	gpio_free(timesync->irq_pin);
irq_pin_request_err:
	misc_deregister(&timesync->miscdev);
	return ret;
}

static int timesync_remove(struct platform_device *pdev)
{
	struct hb_timesync *timesync = platform_get_drvdata(pdev);

#ifdef CONFIG_TIMESYNC_DEBUG
	gpio_free(timesync->debug_pin);
#endif

	free_irq(timesync->irq_num, timesync);
	gpio_free(timesync->irq_pin);

	misc_deregister(&timesync->miscdev);

	return 0;
}

static const struct of_device_id timesync_of_match[] = {
	{.compatible = "hobot,timesync",},
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, timesync_of_match);

static struct platform_driver timesync_driver = {
	.probe	  = timesync_probe,
	.remove   = timesync_remove,
	.driver   = {
			.owner	= THIS_MODULE,
			.name	= "hobot_timesync",
			.of_match_table = timesync_of_match,
	},
};

module_platform_driver(timesync_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("hobot timesync module");
MODULE_LICENSE("GPL v2");
