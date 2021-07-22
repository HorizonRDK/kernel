/**
 * drivers/extcon/extcon-usb-gpio.c - USB GPIO extcon driver
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com
 * Author: Roger Quadros <rogerq@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>

/* module parameters for id/vbus gpio mode : 0 - irq, 1 - polling */
static unsigned int id_poll_mode = 0;
module_param(id_poll_mode, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(id_poll_mode, "id gpio mode: 0 - irq, 1 - polling");

static unsigned int vbus_poll_mode = 0;
module_param(vbus_poll_mode, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(vbus_poll_mode, "vbus gpio mode: 0 - irq, 1 - polling");

#define USB_GPIO_DEBOUNCE_MS	500	/* ms */

struct usb_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;

	struct gpio_desc *id_gpiod;
	struct gpio_desc *vbus_gpiod;
	/* irq for id/vbus irq mode */
	int id_irq;
	int vbus_irq;

	/* task for id/vbus polling mode */
	struct task_struct *id_task;
	struct task_struct *vbus_task;
	int saved_id_state;
	int saved_vbus_state;

	struct gpio_desc *usb_host_reset;
	struct gpio_desc *usb_host_exreset;

	unsigned long debounce_jiffies;
	struct delayed_work wq_detcable;
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

/*
 * "USB" = VBUS and "USB-HOST" = !ID, so we have:
 * Both "USB" and "USB-HOST" can't be set as active at the
 * same time so if "USB-HOST" is active (i.e. ID is 0)  we keep "USB" inactive
 * even if VBUS is on.
 *
 *  State              |    ID   |   VBUS
 * ----------------------------------------
 *  [1] USB            |    H    |    H
 *  [2] none           |    H    |    L
 *  [3] USB-HOST       |    L    |    H
 *  [4] USB-HOST       |    L    |    L
 *
 * In case we have only one of these signals:
 * - VBUS only - we want to distinguish between [1] and [2], so ID is always 1.
 * - ID only - we want to distinguish between [1] and [4], so VBUS = ID.
*/
static void usb_extcon_detect_cable(struct work_struct *work)
{
	int id, vbus;
	struct usb_extcon_info *info = container_of(to_delayed_work(work),
						    struct usb_extcon_info,
						    wq_detcable);

	/* check ID and VBUS and update cable state */
	id = info->id_gpiod ?
		gpiod_get_value_cansleep(info->id_gpiod) : 1;
	vbus = info->vbus_gpiod ?
		gpiod_get_value_cansleep(info->vbus_gpiod) : id;

	/* at first we clean states which are no longer active */
	if (id) {
		if (info->usb_host_reset)
			gpiod_set_value_cansleep(info->usb_host_reset, 1);
		if (info->usb_host_exreset)
			gpiod_set_value_cansleep(info->usb_host_exreset, 1);
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, false);
	}

	if (!vbus)
		extcon_set_state_sync(info->edev, EXTCON_USB, false);

	if (!id) {
		if (info->usb_host_reset)
			gpiod_set_value_cansleep(info->usb_host_reset, 0);
		if (info->usb_host_exreset)
			gpiod_set_value_cansleep(info->usb_host_exreset, 0);
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, true);
	} else {
		if (vbus)
			extcon_set_state_sync(info->edev, EXTCON_USB, true);
	}
}

static irqreturn_t usb_irq_handler(int irq, void *dev_id)
{
	struct usb_extcon_info *info = dev_id;

	queue_delayed_work(system_power_efficient_wq, &info->wq_detcable,
			   info->debounce_jiffies);

	return IRQ_HANDLED;
}


static int id_poll_task(void *arg)
{
	struct usb_extcon_info *info = arg;
	int cur_id;

	if (!info && !info->id_gpiod)
		return -EINVAL;

	/* get initialized id pin state and save it to saved_id_state */
	info->saved_id_state = gpiod_get_value_cansleep(info->id_gpiod);

	while (!kthread_should_stop()) {
		cur_id = gpiod_get_value_cansleep(info->id_gpiod);

		/* if id state is changed, enqueue a work for usb role switch */
		if (cur_id != info->saved_id_state) {
			queue_delayed_work(system_power_efficient_wq, &info->wq_detcable,
					   info->debounce_jiffies);
			info->saved_id_state = cur_id;
		}

		msleep(100);	// wait 100ms
	}

	return 0;
}

static int vbus_poll_task(void *arg)
{
	// TODO:

	return 0;
}

static int usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct usb_extcon_info *info;
	int ret;

	if (!np)
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->id_gpiod = devm_gpiod_get_optional(&pdev->dev, "id", GPIOD_IN);
	info->vbus_gpiod = devm_gpiod_get_optional(&pdev->dev, "vbus",
						   GPIOD_IN);

	if (!info->id_gpiod && !info->vbus_gpiod) {
		dev_err(dev, "failed to get gpios\n");
		return -ENODEV;
	}

	if (IS_ERR(info->id_gpiod))
		return PTR_ERR(info->id_gpiod);

	if (IS_ERR(info->vbus_gpiod))
		return PTR_ERR(info->vbus_gpiod);

	info->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}

	info->usb_host_reset = devm_gpiod_get_optional(&pdev->dev, "host-rst", GPIOD_OUT_LOW);
	if (info->usb_host_reset) {
		gpiod_set_value(info->usb_host_reset, 0);
		udelay(5*1000);
		gpiod_set_value(info->usb_host_reset, 1);
		udelay(100);
		gpiod_set_value(info->usb_host_reset, 0);
	}

	info->usb_host_exreset = devm_gpiod_get_optional(&pdev->dev, "host-exrst", GPIOD_OUT_HIGH);
	if (info->usb_host_exreset) {
		gpiod_set_value(info->usb_host_exreset, 0);
	}

#if 0
	if (info->id_gpiod)
		ret = gpiod_set_debounce(info->id_gpiod,
					 USB_GPIO_DEBOUNCE_MS * 1000);
	if (!ret && info->vbus_gpiod)
		ret = gpiod_set_debounce(info->vbus_gpiod,
					 USB_GPIO_DEBOUNCE_MS * 1000);

	if (ret < 0)
#endif
		info->debounce_jiffies = msecs_to_jiffies(USB_GPIO_DEBOUNCE_MS);

	INIT_DELAYED_WORK(&info->wq_detcable, usb_extcon_detect_cable);

	/* init for saved id/vbus state */
	info->saved_id_state = -1;
	info->saved_vbus_state = -1;

	if (info->id_gpiod) {
		if (!id_poll_mode) {	// id gpio use irq mode
			info->id_irq = gpiod_to_irq(info->id_gpiod);
			if (info->id_irq < 0) {
				dev_err(dev, "failed to get ID IRQ\n");
				return info->id_irq;
			}

			ret = devm_request_threaded_irq(dev, info->id_irq, NULL,
							usb_irq_handler,
							IRQF_TRIGGER_RISING |
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							pdev->name, info);
			if (ret < 0) {
				dev_err(dev, "failed to request handler for ID IRQ\n");
				return ret;
			}
		} else {	// id gpio use polling mode
			info->id_task = kthread_create(id_poll_task, info, "id-poll");
			if (IS_ERR(info->id_task))
				return PTR_ERR(info->id_task);
		}
	}

	if (info->vbus_gpiod) {
		if (!vbus_poll_mode) {	// vbus gpio use irq mode
			info->vbus_irq = gpiod_to_irq(info->vbus_gpiod);
			if (info->vbus_irq < 0) {
				dev_err(dev, "failed to get VBUS IRQ\n");
				return info->vbus_irq;
			}

			ret = devm_request_threaded_irq(dev, info->vbus_irq, NULL,
							usb_irq_handler,
							IRQF_TRIGGER_RISING |
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							pdev->name, info);
			if (ret < 0) {
				dev_err(dev, "failed to request handler for VBUS IRQ\n");
				return ret;
			}
		} else {	// vbus gpio use polling mode
			info->vbus_task = kthread_create(vbus_poll_task, info, "vbus-poll");
			if (IS_ERR(info->vbus_task))
				return PTR_ERR(info->vbus_task);
		}
	}

	platform_set_drvdata(pdev, info);
	device_set_wakeup_capable(&pdev->dev, true);

	/* Perform initial detection */
	usb_extcon_detect_cable(&info->wq_detcable.work);

	/* wakeup id/vbus gpio polling task */
	if (info->id_gpiod && id_poll_mode && info->id_task)
		wake_up_process(info->id_task);

	if (info->vbus_gpiod && vbus_poll_mode && info->vbus_task)
		wake_up_process(info->vbus_task);

	return 0;
}

static int usb_extcon_remove(struct platform_device *pdev)
{
	struct usb_extcon_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&info->wq_detcable);
	device_init_wakeup(&pdev->dev, false);

	if (id_poll_mode && info->id_task)
		kthread_stop(info->id_task);

	if (vbus_poll_mode && info->vbus_task)
		kthread_stop(info->vbus_task);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int usb_extcon_suspend(struct device *dev)
{
	struct usb_extcon_info *info = dev_get_drvdata(dev);
	int ret = 0;

	if (device_may_wakeup(dev)) {
		if (info->id_gpiod && !id_poll_mode) {
			ret = enable_irq_wake(info->id_irq);
			if (ret)
				return ret;
		}
		if (info->vbus_gpiod &&!vbus_poll_mode) {
			ret = enable_irq_wake(info->vbus_irq);
			if (ret) {
				if (info->id_gpiod && !id_poll_mode)
					disable_irq_wake(info->id_irq);

				return ret;
			}
		}
	}

	/*
	 * We don't want to process any IRQs after this point
	 * as GPIOs used behind I2C subsystem might not be
	 * accessible until resume completes. So disable IRQ.
	 */
	if (info->id_gpiod && !id_poll_mode)
		disable_irq(info->id_irq);
	if (info->vbus_gpiod && !vbus_poll_mode)
		disable_irq(info->vbus_irq);

	if (!device_may_wakeup(dev))
		pinctrl_pm_select_sleep_state(dev);

	return ret;
}

static int usb_extcon_resume(struct device *dev)
{
	struct usb_extcon_info *info = dev_get_drvdata(dev);
	int ret = 0;

	if (!device_may_wakeup(dev))
		pinctrl_pm_select_default_state(dev);

	if (device_may_wakeup(dev)) {
		if (info->id_gpiod && !id_poll_mode) {
			ret = disable_irq_wake(info->id_irq);
			if (ret)
				return ret;
		}
		if (info->vbus_gpiod && !vbus_poll_mode) {
			ret = disable_irq_wake(info->vbus_irq);
			if (ret) {
				if (info->id_gpiod && !id_poll_mode)
					enable_irq_wake(info->id_irq);

				return ret;
			}
		}
	}

	if (info->id_gpiod && !id_poll_mode)
		enable_irq(info->id_irq);
	if (info->vbus_gpiod && vbus_poll_mode)
		enable_irq(info->vbus_irq);

	queue_delayed_work(system_power_efficient_wq,
			   &info->wq_detcable, 0);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(usb_extcon_pm_ops,
			 usb_extcon_suspend, usb_extcon_resume);

static const struct of_device_id usb_extcon_dt_match[] = {
	{ .compatible = "linux,extcon-usb-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usb_extcon_dt_match);

static const struct platform_device_id usb_extcon_platform_ids[] = {
	{ .name = "extcon-usb-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, usb_extcon_platform_ids);

static struct platform_driver usb_extcon_driver = {
	.probe		= usb_extcon_probe,
	.remove		= usb_extcon_remove,
	.driver		= {
		.name	= "extcon-usb-gpio",
		.pm	= &usb_extcon_pm_ops,
		.of_match_table = usb_extcon_dt_match,
	},
	.id_table = usb_extcon_platform_ids,
};

module_platform_driver(usb_extcon_driver);

MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
MODULE_DESCRIPTION("USB GPIO extcon driver");
MODULE_LICENSE("GPL v2");
