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


#define HOBOT_CAMERA_LOCK_STATUS		"hobot_camera_check"
#define BUFLEN					16

struct hb_camera_lock_status {
	struct device *dev;
	struct miscdevice miscdev;

	int irq_pin;
	int irq_num;
};

struct hb_camera_lock_status camera_lock_status;
int unlock_count = 0;

/* irq handled
 * check and print camera lock status
 */
static irqreturn_t camera_lock_status_irq(int irq, void *dev)
{
	int gpio_value = 0;

	gpio_value = gpio_get_value(camera_lock_status.irq_pin);
	if (gpio_value == 1) {
		// if don't init or reset camera, this is error, otherwise it is normal.
		dev_warn(camera_lock_status.dev, "camera is lock\n");
	} else if (gpio_value == 0) {
		unlock_count += 1;
		// if don't init or reset camera, this is error, otherwise it is normal.
		dev_warn(camera_lock_status.dev, "camera unlock!!!\n");
	}
	return IRQ_HANDLED;
}

static int camera_lock_status_open(struct inode *inode, struct file *filp)
{
	struct hb_camera_lock_status *camera_lock_status =
		container_of(filp->private_data, struct hb_camera_lock_status, miscdev);

	filp->private_data = camera_lock_status;

	return 0;
}

static int camera_lock_status_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static ssize_t unlock_count_show(struct device *dev,
		struct device_attribute *attr, char* buf)
{
	int len = 0;

	len = snprintf(buf, BUFLEN, "%d\n", unlock_count);
	dev_info(dev, "camera unlock time is %d\n", unlock_count);
	unlock_count = 0;
	return len;
}
static DEVICE_ATTR(unlock_count, 0444, unlock_count_show, NULL);

static const struct file_operations camera_lock_status_fops = {
	.owner =	THIS_MODULE,
	.open =		camera_lock_status_open,
	.release =	camera_lock_status_release,
};

static int camera_lock_status_probe(struct platform_device *pdev)
{
	int ret = 0;

	dev_info(&pdev->dev, "hobot camera_lock_status probe begin\n");

	ret = of_property_read_u32(pdev->dev.of_node,
			"camera_check_irq_pin", &camera_lock_status.irq_pin);
	if (ret) {
		dev_err(&pdev->dev, "camera check devieces get irq pin failed!\n");
		goto err;
	}

	camera_lock_status.dev = &pdev-dev;
	camera_lock_status.miscdev.minor = MISC_DYNAMIC_MINOR;
	camera_lock_status.miscdev.name = HOBOT_CAMERA_LOCK_STATUS;
	camera_lock_status.miscdev.fops = &camera_lock_status_fops;
	ret = misc_register(&camera_lock_status.miscdev);
	if (ret) {
		dev_err(&pdev->dev, "camera lock status device register failed\n");
		goto err;
	}

	ret = gpio_request(camera_lock_status.irq_pin, "irq-pin");
	if (ret) {
		dev_err(&pdev->dev, "camera lock gpio request failed\n");
		goto irq_pin_request_err;
	}
	camera_lock_status.irq_num = gpio_to_irq(camera_lock_status.irq_pin);
	dev_info(&pdev->dev, "irq_pin = %d, irq_num = %d\n",
			camera_lock_status.irq_pin, camera_lock_status.irq_num);
	ret = request_irq(camera_lock_status.irq_num, camera_lock_status_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			HOBOT_CAMERA_LOCK_STATUS, &camera_lock_status);
	if (ret) {
		dev_err(&pdev->dev, "camera lock status request irq failed\n");
		goto request_irq_err;
	}
	ret = device_create_file(&pdev->dev, &dev_attr_unlock_count);
	if (ret < 0) {
		dev_err(&pdev->dev, "create fps failed (%d)\n", ret);
		goto create_dev_attr_err;
	}

	dev_info(&pdev->dev, "hobot camera_lock_status probe success\n");

	return 0;

create_dev_attr_err:
	free_irq(camera_lock_status.irq_num, &camera_lock_status);
request_irq_err:
	gpio_free(camera_lock_status.irq_pin);
irq_pin_request_err:
	misc_deregister(&camera_lock_status.miscdev);
err:
	return ret;
}

static int camera_lock_status_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_unlock_count);

	free_irq(camera_lock_status.irq_num, &camera_lock_status);

	gpio_free(camera_lock_status.irq_pin);

	misc_deregister(&camera_lock_status.miscdev);

	return 0;
}
static const struct of_device_id camera_check_of_match[] = {
	{.compatible = "hobot,camera_check", },
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, camera_check_of_match);

static struct platform_driver camera_check_driver = {
	.probe    = camera_lock_status_probe,
	.remove   = camera_lock_status_remove,
	.driver   = {
		.owner  = THIS_MODULE,
		.name   = "hobot_camera_check",
		.of_match_table = camera_check_of_match,
	},
};

module_platform_driver(camera_check_driver);

MODULE_AUTHOR("v-yameng.lu<v-yameng.lu@horizon.ai>");
MODULE_DESCRIPTION("hobot camera_lock_status module");
MODULE_LICENSE("GPL v2");

