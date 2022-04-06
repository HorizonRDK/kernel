/*
 * Horizon Robotics
 *
 * Copyright (C) 2021 Horizon Robotics Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/usb/gadget.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include "core.h"//NOLINT

#define HBUSB_GPIO_DETECT_MS	             100	/* ms */
#define HBUSB_UNPLUG_CHECK_COUNTS	     5          /* count */

enum hbusb_extcon_gpio_mode {
	HBUSB_EXTCON_GPIO_NULL = 0,
	HBUSB_EXTCON_GPIO_POLL,
	HBUSB_EXTCON_GPIO_INT,
};

struct find_data {
	u8 name[64];
};

struct vbus_usb_udc {
	struct usb_gadget_driver	*driver;
	struct usb_gadget		*gadget;
	struct device			dev;
	struct list_head		list;
	bool				vbus;
};

struct vbus_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;

	enum hbusb_extcon_gpio_mode gpio_mode;
	struct gpio_desc *vbus_gpiod;

	struct task_struct *vbus_poll_task;

	u32 unplug_cnt;
	u32 pluged;
};

static struct device *vbus_dev = NULL;

static int vbus_find_match(struct device *dev, void *data)
{
	struct find_data *d = data;

	if (strcmp(dev_name(dev), d->name) == 0) {
		vbus_dev = dev;
		return 1;
	}

	return 0;
}

static int vbus_find_device(void)
{
	struct find_data data = {0};
	u8 *usb_name = "b2000000.dwc3";

	memcpy(data.name, usb_name, strlen(usb_name));
	return bus_for_each_dev(&platform_bus_type, NULL, &data,
		       vbus_find_match);
}

static inline int usb_gadget_udc_start(struct vbus_usb_udc *udc)
{
	return udc->gadget->ops->udc_start(udc->gadget, udc->driver);
}

static inline void usb_gadget_udc_stop(struct vbus_usb_udc *udc)
{
	udc->gadget->ops->udc_stop(udc->gadget);
}

static int
vbus_soft_reset(struct vbus_extcon_info * info, struct device *dev)
{
	struct vbus_usb_udc	*udc;
	struct dwc3	*dwc;

	if (!dev)
		return -EOPNOTSUPP;

	dwc = dev_get_drvdata(dev);
	if (!dwc || !dwc->gadget->dev.driver)
		return -EOPNOTSUPP;

	udc = (struct vbus_usb_udc *)(dwc->gadget->udc);
	if (!udc || !udc->driver)
		return -EOPNOTSUPP;

	if (!udc->gadget || !udc->gadget->ops)
		return -EOPNOTSUPP;

	dev_info(info->dev, "vbus gpio detect unplug, now reset gadget\n");
	/* disconnect */
	usb_gadget_disconnect(udc->gadget);
	udc->driver->disconnect(udc->gadget);
	usb_gadget_udc_stop(udc);

	/* connect */
	usb_gadget_udc_start(udc);
	usb_gadget_connect(udc->gadget);

	return 0;
}

static int vbus_poll_kthread(void *arg)
{
	struct vbus_extcon_info *info = arg;
	int value;

	if (!info)
		return -1;

	info->unplug_cnt = 0;
	info->pluged = 0;

	while (!kthread_should_stop()) {
		value = gpiod_get_value(info->vbus_gpiod);
		if (!value) {
			info->unplug_cnt = 0;
			info->pluged = 1;
		} else {
			info->unplug_cnt++;
		}

		/* vbus from low->high and high keep 500ms will do reset */
		if (info->pluged && info->unplug_cnt > HBUSB_UNPLUG_CHECK_COUNTS) {
			info->pluged = 0;
			info->unplug_cnt = 0;
			vbus_soft_reset(info, vbus_dev);
		}

		msleep(HBUSB_GPIO_DETECT_MS);
	}

	return 0;
}

static int vbus_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct vbus_extcon_info *info;
	int ret;
	u32 gpio_mode = 0;

	if (!np)
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;

	if (of_property_read_u32(np, "gpio-mode", &gpio_mode)) {
		dev_info(dev, "failed get gpio-mode, default mode 0. nothing\n");
		gpio_mode = 0;
	}

	if (gpio_mode > HBUSB_EXTCON_GPIO_INT) {
		dev_info(dev, "not support gpio-mode %d, default mode 0.\n",
				gpio_mode);
		gpio_mode = 0;
	}
	info->gpio_mode = gpio_mode;

	info->vbus_gpiod = devm_gpiod_get_optional(&pdev->dev, "vbus",
			GPIOD_IN);
	if (info->gpio_mode && !info->vbus_gpiod) {
		dev_err(dev, "failed to get vbus gpios\n");
		return -ENODEV;
	}

	if (info->gpio_mode && IS_ERR(info->vbus_gpiod)) {
		dev_err(dev, "vbus_gpiod ptr err\n");
		return PTR_ERR(info->vbus_gpiod);
	}

	if (info->gpio_mode == HBUSB_EXTCON_GPIO_POLL) {
		ret = vbus_find_device();
		if (ret <= 0) {
			dev_err(dev, "not found device\n");
			return -ENODEV;
		}

		info->vbus_poll_task = kthread_run(vbus_poll_kthread, info,
				"vbus_poll_kthread");
		if (IS_ERR(info->vbus_poll_task)) {
			dev_err(dev, "failed to create kthread\n");
			return -ENODEV;
		}
	}

	platform_set_drvdata(pdev, info);
	dev_info(dev, "vbus_extcon_probe success");

	return 0;
}

static int vbus_extcon_remove(struct platform_device *pdev)
{
	struct vbus_extcon_info *info = platform_get_drvdata(pdev);

	if (info->vbus_poll_task) {
		kthread_stop(info->vbus_poll_task);
	}

	return 0;
}

static const struct of_device_id vbus_extcon_dt_match[] = {
	{ .compatible = "linux,extcon-vbus-gpio", },
	{ /* sentinel */ }
};

static const struct platform_device_id vbus_extcon_platform_ids[] = {
	{ .name = "extcon-vbus-gpio", },
	{ /* sentinel */ }
};

static struct platform_driver vbus_extcon_driver = {
	.probe		= vbus_extcon_probe,
	.remove		= vbus_extcon_remove,
	.driver		= {
		.name	= "extcon-vbus-gpio",
		/* .pm	= &usb_extcon_pm_ops, */
		.of_match_table = vbus_extcon_dt_match,
	},
	.id_table = vbus_extcon_platform_ids,
};

static int __init vbus_module_init(void)
{
	return platform_driver_register(&vbus_extcon_driver);
}

static void __exit vbus_module_exit(void)
{
	platform_driver_unregister(&vbus_extcon_driver);
}

module_init(vbus_module_init);
module_exit(vbus_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("horizon.ai");

