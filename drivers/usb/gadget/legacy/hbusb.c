/* Copyright (C) 2020 - 2021  Horizon */
// SPDX-License-Identifier: GPL-2.0
/*Bulk channel transfer gadget driver*/
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/of_device.h>
#include "hobot/hbusb/u_hbusb.h"

#define DRIVER_DESC		"Hobot HBUSB bulk channel"
#define DRIVER_VERSION		"V1.0"

#define DRIVER_VENDOR_NUM			0x1011
#define DRIVER_PRODUCT_BASE_NUM		0x2021

USB_GADGET_COMPOSITE_OPTIONS();

static int devno_base = 0;

/*-----------device/string desc------------------*/
static struct usb_device_descriptor device_desc = {
		.bLength =		sizeof( struct usb_device_descriptor),
		.bDescriptorType =	USB_DT_DEVICE,

		/* .bcdUSB = DYNAMIC */
		.bDeviceClass =		USB_CLASS_COMM,
		.bDeviceSubClass =	0,
		.bDeviceProtocol =	0,

		.idVendor =			cpu_to_le16(DRIVER_VENDOR_NUM),
		.idProduct = 		cpu_to_le16(DRIVER_PRODUCT_BASE_NUM),
		.bNumConfigurations =	1,
};

static const char serial[] = "0123456789.0123456789.0123456789";

static struct usb_string strings_dev[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = "Horizon",
	[USB_GADGET_PRODUCT_IDX].s = "HBUSB",
	[USB_GADGET_SERIAL_IDX].s = serial,
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
		.language	= 0x0409,	/* en-us */
		.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_configuration hbusb_config = {
		.label = "hbusb",
		.bConfigurationValue	= 1,
		/* .iConfiguration = DYNAMIC */
		.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};
/*-------------------------------------------------------------------------*/

static struct usb_function_instance *fi_hbusb;
static struct usb_function *f_hbusb;

static int hbusb_do_config(struct usb_configuration *c)
{
	int status;

	f_hbusb = usb_get_function(fi_hbusb);
	if (IS_ERR(f_hbusb))
		return PTR_ERR(f_hbusb);

	status = usb_add_function(c, f_hbusb);
	if (status < 0)
		usb_put_function(f_hbusb);

	return status;
}

static int hbusb_bind(struct usb_composite_dev *cdev)
{
	struct usb_gadget	*gadget = cdev->gadget;
	struct f_hbusb_opts	*hbusb_opts = NULL;
	int					status;
	struct usb_string	*usb_string;


	fi_hbusb = usb_get_function_instance("hbusb");
	if (IS_ERR(fi_hbusb))
		return PTR_ERR(fi_hbusb);

	hbusb_opts = container_of(fi_hbusb, struct f_hbusb_opts, func_inst);
	hbusb_set_gadget(hbusb_opts->dev, cdev->gadget);
	status = hbusb_register_cdev(hbusb_opts->dev);
	if (status)
		goto fail0;
	hbusb_opts->bound = true;

	usb_string = stringtab_dev.strings;
	status = usb_string_ids_tab(cdev, usb_string);
	if (status < 0)
		goto fail0;
	device_desc.iManufacturer = usb_string[USB_GADGET_MANUFACTURER_IDX].id;
	device_desc.iProduct = usb_string[USB_GADGET_PRODUCT_IDX].id;
	device_desc.idProduct = cpu_to_le16(
			DRIVER_PRODUCT_BASE_NUM + devno_base);

	status = usb_add_config(cdev, &hbusb_config, hbusb_do_config);
	if (status < 0)
		goto fail0;

	usb_composite_overwrite_options(cdev, &coverwrite);
	dev_info(&gadget->dev, "%s, version: " DRIVER_VERSION "\n",
			DRIVER_DESC);

	return 0;

fail0:
	usb_put_function_instance(fi_hbusb);
	return status;
}

static int hbusb_unbind(struct usb_composite_dev *cdev)
{
	usb_put_function(f_hbusb);
	usb_put_function_instance(fi_hbusb);

	return 0;
}

static struct usb_composite_driver hbusb_driver = {
		.name		= "hbusb",
		.dev		= &device_desc,
		.strings	= dev_strings,
		.max_speed	= USB_SPEED_SUPER,
		.bind		= hbusb_bind,
		.unbind		= hbusb_unbind,
};

static int __init init(void)
{
	return usb_composite_probe(&hbusb_driver);
}

static void __exit cleanup(void)
{
	usb_composite_unregister(&hbusb_driver);
}

module_init(init);
module_exit(cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("horizon.ai");


