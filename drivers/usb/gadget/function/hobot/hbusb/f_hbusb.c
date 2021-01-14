/* Copyright (C) 2020 - 2021  Horizon */
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018-2019
 *
 * Bulk channel transfer
 */
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/types.h>

#include <linux/atomic.h>

#include "u_hbusb.h"//NOLINT

#define HBUSB_EPIN_MAX_NUM 16
#define HBUSB_EPOUT_MAX_NUM 16

static uint request_epin_num = 0;
static uint request_epout_num = 0;

struct f_hbusb {
	struct hbusb			port;
	u8					data_id;
};

static inline struct f_hbusb *func_to_hbusb(struct usb_function *f)
{
	return container_of(f, struct f_hbusb, port.func);
}

/*-------------------------------------------------------------------------*/
static struct usb_interface_descriptor hbusb_data_intf = {
	.bLength =		sizeof(hbusb_data_intf),
	.bDescriptorType =	USB_DT_INTERFACE,

	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =		0,
	.bInterfaceClass =		USB_CLASS_COMM,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	/* .iInterface = DYNAMIC */
};

static struct usb_endpoint_descriptor ss_in_desc[HBUSB_EPIN_MAX_NUM] = {
	{
		.bLength =		USB_DT_ENDPOINT_SIZE,
		.bDescriptorType =	USB_DT_ENDPOINT,

		.bEndpointAddress =	USB_DIR_IN,
		.bmAttributes =		USB_ENDPOINT_XFER_BULK,
		.wMaxPacketSize =	cpu_to_le16(1024),
	},
};

static struct usb_endpoint_descriptor ss_out_desc[HBUSB_EPOUT_MAX_NUM] = {
	{
		.bLength =		USB_DT_ENDPOINT_SIZE,
		.bDescriptorType =	USB_DT_ENDPOINT,

		.bEndpointAddress =	USB_DIR_OUT,
		.bmAttributes =		USB_ENDPOINT_XFER_BULK,
		.wMaxPacketSize =	cpu_to_le16(1024),
	},
};

static struct usb_ss_ep_comp_descriptor ss_bulk_comp_desc = {
	.bLength =		sizeof(ss_bulk_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,
	/* the following 2 values can be tweaked if necessary */
	.bMaxBurst =		10,  /*actual test 2 is enough*/
	.bmAttributes =	0,
};

static struct usb_descriptor_header *
		hbusb_ss_function[2*HBUSB_EPOUT_MAX_NUM + 1] = {
	(struct usb_descriptor_header *) &hbusb_data_intf,
	NULL,
};

/* string descriptors: */
static struct usb_string hbusb_string_defs[] = {
	[0].s = "hobot usb transfer",
	{  } /* end of list */
};

static struct usb_gadget_strings hbusb_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		hbusb_string_defs,
};

static struct usb_gadget_strings *hbusb_strings[] = {
	&hbusb_string_table,
	NULL,
};
/*-------------------------------------------------------------------------*/


void hbusb_set_epnum_param(unsigned int epin_num,
									unsigned int epout_num)
{
	request_epin_num = epin_num;
	request_epout_num = epout_num;
}
EXPORT_SYMBOL_GPL(hbusb_set_epnum_param);


/*-------------------------------------------------------------------------*/
static void hbusb_free_inst(struct usb_function_instance *f)
{
	struct f_hbusb_opts *opts;

	opts = container_of(f, struct f_hbusb_opts, func_inst);
	if (opts->bound)
		hbusb_cleanup(opts->dev);
	else
		hbusb_release_default(opts->dev);

	opts->dev = NULL;
	kfree(opts);
}

static struct usb_function_instance *hbusb_alloc_inst(void)
{
	struct f_hbusb_opts *opts;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = hbusb_free_inst;
	opts->dev = hbusb_setup_default(request_epin_num, request_epout_num);
	if (IS_ERR(opts->dev)) {
		opts->dev = NULL;
		kfree(opts);
		return ERR_PTR(-EINVAL);
	}

	return &opts->func_inst;
}
/*-------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------*/
static int func_hbusb_bind(struct usb_configuration *c, struct usb_function *f)
{
	int i = 0, j = 0;
	struct usb_composite_dev *cdev = c->cdev;
	struct f_hbusb		*hbusb = func_to_hbusb(f);
	struct usb_string	*us;
	int					status;
	struct usb_ep		*ep;
	struct usb_ep		*endpoint;
	struct f_hbusb_opts       *opts;
	unsigned int		gadget_epin_num = 0, gadget_epout_num = 0;

	opts = container_of(f->fi, struct f_hbusb_opts, func_inst);

	/*
	 * in drivers/usb/gadget/configfs.c:configfs_composite_bind()
	 * configurations are bound in sequence with list_for_each_entry,
	 * in each configuration its functions are bound in sequence
	 * with list_for_each_entry, so we assume no race condition
	 * with regard to opts->bound access
	 * note Now bct not support CONFIGFS, we reserve this for future
	 */
	if (!opts->bound) {
		hbusb_set_gadget(opts->dev, cdev->gadget);
		status = hbusb_register_cdev(opts->dev);
		if (status)
			goto fail;
		opts->bound = true;
	}

	us = usb_gstrings_attach(cdev, hbusb_strings,
						ARRAY_SIZE(hbusb_string_defs));
	if (IS_ERR(us)) {
		status = PTR_ERR(us);
		goto fail;
	}
	hbusb_data_intf.iInterface = us[0].id;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	hbusb->data_id = status;
	hbusb_data_intf.bInterfaceNumber = status;

	status = -ENODEV;
	/*check epin_num and epout_num valid */
	if ((request_epin_num == 0) && (request_epout_num == 0)) {
		goto fail;
	} else {
		list_for_each_entry(endpoint, &cdev->gadget->ep_list, ep_list) {
			if (endpoint->caps.dir_in) {
				gadget_epin_num++;
			} else {
				gadget_epout_num++;
			}
		}
		INFO(cdev, "gadget ep[in:%d, out:%d] request ep[in:%d out:%d]\n",
				gadget_epin_num, gadget_epout_num,
				request_epin_num, request_epout_num);
		if ((request_epin_num > gadget_epin_num)
			|| (request_epout_num > gadget_epin_num)) {
			ERROR(cdev,
		"gadget request num is wrong, request[in:%d, out:%d], gadget [in:%d, out%d]",
			request_epin_num, request_epout_num, gadget_epin_num, gadget_epout_num);
			goto fail;
		}
	}

	/* allocate instance-specific endpoints */
	for (i = 0; i < request_epin_num; i++) {
		ss_in_desc[i].bLength = USB_DT_ENDPOINT_SIZE;
		ss_in_desc[i].bDescriptorType = USB_DT_ENDPOINT;
		ss_in_desc[i].bEndpointAddress = USB_DIR_IN | (i + 1);
		ss_in_desc[i].bmAttributes = USB_ENDPOINT_XFER_BULK;
		ss_in_desc[i].wMaxPacketSize = cpu_to_le16(1024);
		ep = usb_ep_autoconfig_ss(cdev->gadget,
					&ss_in_desc[i], &ss_bulk_comp_desc);
		if (!ep)
			goto fail;
		hbusb->port.bulkin[i] = ep;
		DBG(cdev, "hbusb: %s speed IN/%s\n",
			gadget_is_superspeed(c->cdev->gadget) ? "super" :
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			hbusb->port.bulkin[i]->name);
	}

	for (i = 0; i < request_epout_num; i++) {
		ss_out_desc[i].bLength = USB_DT_ENDPOINT_SIZE;
		ss_out_desc[i].bDescriptorType = USB_DT_ENDPOINT;
		ss_out_desc[i].bEndpointAddress = USB_DIR_OUT | (i + 1);
		ss_out_desc[i].bmAttributes = USB_ENDPOINT_XFER_BULK;
		ss_out_desc[i].wMaxPacketSize = cpu_to_le16(1024);
		ep = usb_ep_autoconfig_ss(cdev->gadget,
					&ss_out_desc[i], &ss_bulk_comp_desc);
		if (!ep)
			goto fail;
		hbusb->port.bulkout[i] = ep;
		DBG(cdev, "hbusb: %s speed OUT/%s\n",
			gadget_is_superspeed(c->cdev->gadget) ? "super" :
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			hbusb->port.bulkout[i]->name);
	}

	status = -ENOMEM;
	/*first config hbusb_data_intf interface class and endpoint num*/
	hbusb_data_intf.bNumEndpoints = request_epin_num + request_epout_num;
	for (i = 0, j = 1; i < request_epin_num; i++, j += 2) {
		hbusb_ss_function[j] = (struct usb_descriptor_header *)&ss_in_desc[i];
		hbusb_ss_function[j + 1] = (struct usb_descriptor_header *)&ss_bulk_comp_desc;
	}

	for (i = 0; i < request_epout_num; i++,  j += 2) {
		hbusb_ss_function[j] = (struct usb_descriptor_header *)&ss_out_desc[i];
		hbusb_ss_function[j + 1] = (struct usb_descriptor_header *)&ss_bulk_comp_desc;
	}
	hbusb_ss_function[j] = NULL;

	status = usb_assign_descriptors(f, NULL, NULL, hbusb_ss_function, NULL);
	if (status)
		goto fail;

	return 0;

fail:
	return status;
}

static void func_hbusb_unbind(struct usb_configuration *c,
							struct usb_function *f)
{
	usb_free_all_descriptors(f);
}

static int func_hbusb_set_alt(struct usb_function *f,
					unsigned int intf, unsigned int alt)
{
	int					i;
	int                 status;
	struct f_hbusb		*hbusb = func_to_hbusb(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	if (intf == hbusb->data_id) {
		if (hbusb->port.bulkin[0]->enabled
			||hbusb->port.bulkout[0]->enabled) {
			DBG(cdev, "reset hbusb\n");
			hbusb_disconnect(&hbusb->port);
		}

		for (i = 0; i < request_epin_num; i++) {
			if (!hbusb->port.bulkin[i]->desc) {
				if (config_ep_by_speed(cdev->gadget, f, hbusb->port.bulkin[i])) {
					hbusb->port.bulkin[i]->desc = NULL;
					return -EINVAL;
				}
			}
		}

		for (i = 0; i < request_epout_num; i++) {
			if (!hbusb->port.bulkout[i]->desc) {
				if (config_ep_by_speed(cdev->gadget, f, hbusb->port.bulkout[i])) {
					hbusb->port.bulkout[i]->desc = NULL;
					return -EINVAL;
				}
			}
		}

		DBG(cdev, "hbusb RX/TX early activation ...\n");
		status = hbusb_connect(&hbusb->port);
		if (status)
			return status;
	}

	return 0;
}

static void func_hbusb_disable(struct usb_function *f)
{
	struct f_hbusb		*hbusb = func_to_hbusb(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	DBG(cdev, "hbusb deactivated\n");
	hbusb_disconnect(&hbusb->port);
}

static void func_hbusb_free(struct usb_function *f)
{
	struct f_hbusb *hbusb;
	struct f_hbusb_opts *opts;

	hbusb = func_to_hbusb(f);
	opts = container_of(f->fi, struct f_hbusb_opts, func_inst);
	kfree(hbusb);
	mutex_lock(&opts->lock);
	opts->refcnt--;
	mutex_unlock(&opts->lock);
}
/*-------------------------------------------------------------------------*/


static struct usb_function *hbusb_alloc(struct usb_function_instance *fi)
{
	struct f_hbusb		*hbusb;
	struct f_hbusb_opts	*opts;

	/* allocate and initialize one new instance */
	hbusb = kzalloc(sizeof(*hbusb), GFP_KERNEL);
	if (!hbusb)
		return ERR_PTR(-ENOMEM);

	opts = container_of(fi, struct f_hbusb_opts, func_inst);
	mutex_lock(&opts->lock);
	opts->refcnt++;
	hbusb->port.ioport = dev_get_drvdata(opts->dev);
	mutex_unlock(&opts->lock);

	hbusb->port.func.name = "hbusb";
	/* descriptors are per-instance copies */
	hbusb->port.func.bind = func_hbusb_bind;
	hbusb->port.func.unbind = func_hbusb_unbind;
	hbusb->port.func.set_alt = func_hbusb_set_alt;
	hbusb->port.func.disable = func_hbusb_disable;
	//hbusb->port.func.setup = func_hbusb_setup;
	hbusb->port.func.free_func = func_hbusb_free;

	return &hbusb->port.func;
}

DECLARE_USB_FUNCTION_INIT(hbusb, hbusb_alloc_inst, hbusb_alloc);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("horizon.ai");
