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
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/extcon.h>
#include <soc/hobot/hobot_bus.h>

#include "core.h"

struct dwc3_powersave {
	struct device		*dev;
	struct device		*parent;	/* parent dwc3 device */

	/* ddr freq notifier */
	struct notifier_block	ddrfreq_nb;
	/* notify usb suspend & resume event */
	struct notifier_block	powersave_nb;
	/* notify extcon usb gpio event */
	struct notifier_block	edev_nb;
	struct extcon_dev	*edev;
	struct mutex		notifier_lock;

	u32			current_powersave_status;
	u32			last_powersave_status;
};

/*
 * For ddr dynamic frequency selection & handling.
 * Register a ddr frequency change notifier.
 * When ddr frequency change, hb_bus module will notify HB_BUS_SIGNAL_START
 * and HB_BUS_SIGNAL_END signals. Between this 2 signals, we need to guarantee
 * usb controller don't access the ddr. Therefore, use the spinlock dwc->lock
 * to do synchronize.
 */
static int ddr_dfs_call(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct dwc3_powersave *dwc_pwr = container_of(self,
			struct dwc3_powersave, ddrfreq_nb);
	struct device	*dev = dwc_pwr->dev;
	struct device	*dwc3_device = dwc_pwr->parent;

	if (!dwc3_device)
		return NOTIFY_BAD;

	switch (action) {
	case HB_BUS_SIGNAL_START:
		/*
		 * Between START & END, there is sleep function, so couldn't use
		 * below spin_lock.
		 */
		dev_dbg(dev, "%s: bus signal start.\n", __func__);
		break;

	case HB_BUS_SIGNAL_END:
		/* enter powersave state, suspend dwc3 & shutdown the power */
		dev_dbg(dev, "%s: bus signal end.\n", __func__);
		break;

	default:
		dev_err(dev, "%s: Not defined action (%lu)\n",
				__func__, action);
		break;
	}

	return NOTIFY_OK;
}

static int power_save_call(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct dwc3_powersave *dwc_pwr = container_of(self,
			struct dwc3_powersave, powersave_nb);
	struct device	*dev = dwc_pwr->dev;
	struct device	*dwc3_device = dwc_pwr->parent;

	if (!dwc3_device)
		return NOTIFY_BAD;

	switch (action) {
	case POWERSAVE_STATE:
		mutex_lock(&dwc_pwr->notifier_lock);
		dwc_pwr->last_powersave_status = 1;
		if (dwc_pwr->current_powersave_status) {
			dev_info(dev, "%s: dwc3 already in powersave status\n", __func__);
			mutex_unlock(&dwc_pwr->notifier_lock);
			break;
		}

		/* before enter powersave state, release dwc3 driver */
		dev_dbg(dev, "%s: powersave state.\n", __func__);
		dev_dbg(dev, "%s: manually detach dwc3 driver.\n", __func__);
		device_release_driver(dwc3_device);

		dwc_pwr->current_powersave_status = 1;
		mutex_unlock(&dwc_pwr->notifier_lock);

		break;
	case OTHER_STATE:
		mutex_lock(&dwc_pwr->notifier_lock);
		dwc_pwr->last_powersave_status = 0;
		if (!dwc_pwr->current_powersave_status) {
			dev_info(dev, "%s: dwc3 already in normal status\n",
					__func__);
			mutex_unlock(&dwc_pwr->notifier_lock);
			break;
		}

		/* before leave powersave state, attach device to dwc3 driver */
		dev_dbg(dev, "%s: otherstate state.\n", __func__);
		dev_dbg(dev, "%s: manually attach dwc3 driver.\n", __func__);
		if (device_attach(dwc3_device) < 0)
			dev_err(dwc3_device, "%s: attach device to driver failed\n", __func__);

		dwc_pwr->current_powersave_status = 0;
		mutex_unlock(&dwc_pwr->notifier_lock);

		break;
	default:
		dev_err(dwc3_device, "%s: Not defined action (%lu)\n",
				__func__, action);
		break;
	}

	return NOTIFY_OK;
}

static int extcon_usb_notifier(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct dwc3_powersave *dwc_pwr = container_of(self,
			struct dwc3_powersave, edev_nb);
	struct device	*dev = dwc_pwr->dev;
	struct device	*dwc3_device = dwc_pwr->parent;

	dev_dbg(dev, "%s: action(%lu), last_powerstatus(%d)\n",
			__func__, action, dwc_pwr->last_powersave_status);

	if (action) {
		/* extcon otg usb device plug in */
		mutex_lock(&dwc_pwr->notifier_lock);
		if (!dwc_pwr->current_powersave_status) {
			dev_info(dev, "%s: current not powersave status. needn't attach dwc3 driver",
					__func__);
			mutex_unlock(&dwc_pwr->notifier_lock);
			return NOTIFY_DONE;
		}

		/* leave powersave state, attach device to dwc3 driver */
		dev_dbg(dev, "%s: manually attach dwc3 driver.\n", __func__);
		if (device_attach(dwc3_device) < 0)
			dev_err(dwc3_device, "%s: attach device to driver failed\n", __func__);

		dwc_pwr->current_powersave_status = 0;
		mutex_unlock(&dwc_pwr->notifier_lock);
	} else {
		/* extcon otg usb device plug out, restore previous
		 * powersave status if current not in powersave status */
		mutex_lock(&dwc_pwr->notifier_lock);
		if (dwc_pwr->last_powersave_status &&
				!dwc_pwr->current_powersave_status) {
			dev_info(dev, "%s: previous in powersave status, "
					"need to restore it\n", __func__);
			device_release_driver(dwc3_device);
			dwc_pwr->current_powersave_status = 1;
		}
		mutex_unlock(&dwc_pwr->notifier_lock);
	}

	return NOTIFY_DONE;
}

static const struct of_device_id of_dwc3_powersave_match[] = {
	{ .compatible = "hobot,dwc3-powersave", },
};

static int dwc3_powersave_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct dwc3_powersave	*dwc_pwr;
	struct platform_device	*dwc3_pdev = NULL;
	struct device_node	*dwc3_node;	/* parent, dwc3 device node */
	int	ret;

	dwc_pwr = devm_kzalloc(dev, sizeof(*dwc_pwr), GFP_KERNEL);
	if (!dwc_pwr)
		return -ENOMEM;

	/* get dwc3 device */
	dwc3_node = of_parse_phandle(dev->of_node, "dwc3", 0);
	if (!dwc3_node) {
		dev_err(dev, "%s of_parse_phandle dwc3 failed\n", __func__);
		return -EFAULT;
	}

	dwc3_pdev = of_find_device_by_node(dwc3_node);
	if (!dwc3_pdev) {
		dev_err(dev, "%s of_find_device_by_node failed\n", __func__);
		ret = -EFAULT;
		goto find_device_fail;
	}

	dwc_pwr->dev = dev;
	dwc_pwr->parent = &dwc3_pdev->dev;

	mutex_init(&dwc_pwr->notifier_lock);

	if (of_property_read_bool(dwc3_node, "extcon"))
		dwc_pwr->edev = extcon_get_edev_by_phandle(&dwc3_pdev->dev, 0);

	if (IS_ERR(dwc_pwr->edev)) {
		ret = PTR_ERR(dwc_pwr->edev);
		goto edev_err;
	}

	/* register extcon usb gpio notifier */
	dwc_pwr->edev_nb.notifier_call = extcon_usb_notifier;
	ret = extcon_register_notifier(dwc_pwr->edev, EXTCON_USB_HOST,
				       &dwc_pwr->edev_nb);
	if (ret < 0) {
		dev_err(dev, "couldn't register extcon usb notifier\n");
		goto edev_err;
	}

	/* register ddr dfs notifier */
	dwc_pwr->ddrfreq_nb.notifier_call = ddr_dfs_call;
	ret = hb_bus_register_client(&dwc_pwr->ddrfreq_nb);
	if (ret)
		goto notifier_err1;

	/* register powersave notifier */
	dwc_pwr->powersave_nb.notifier_call = power_save_call;
	ret = hb_usb_register_client(&dwc_pwr->powersave_nb);
	if (ret)
		goto notifier_err2;

	platform_set_drvdata(pdev, dwc_pwr);

	return 0;

notifier_err2:
	/* unregister ddr dfs notifier */
	hb_bus_unregister_client(&dwc_pwr->ddrfreq_nb);
notifier_err1:
	/* unregister extcon usb gpio notifier */
	hb_bus_unregister_client(&dwc_pwr->edev_nb);
edev_err:
find_device_fail:
	of_node_put(dwc3_node);

	return ret;
}

static int dwc3_powersave_remove(struct platform_device *pdev)
{
	struct dwc3_powersave	*dwc_pwr = platform_get_drvdata(pdev);

	/* unregister power save notifier */
	hb_usb_unregister_client(&dwc_pwr->powersave_nb);

	/* unregister ddr dfs notifier */
	hb_bus_unregister_client(&dwc_pwr->ddrfreq_nb);

	/* unregister extcon usb gpio notifier */
	extcon_unregister_notifier(dwc_pwr->edev, EXTCON_USB_HOST,
				   &dwc_pwr->edev_nb);

	return 0;
}

static struct platform_driver dwc3_powersave_driver = {
	.probe		= dwc3_powersave_probe,
	.remove		= dwc3_powersave_remove,
	.driver		= {
		.name	= "dwc3-powersave",
		.of_match_table	= of_match_ptr(of_dwc3_powersave_match),
	},
};

module_platform_driver(dwc3_powersave_driver);

MODULE_ALIAS("platform:dwc3-powersave");
MODULE_AUTHOR("Jianghe Xu<jianghe.xu@horizon.ai>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("dwc3 powersave driver");
