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

#include <asm/io.h>
#include <linux/notifier.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <soc/hobot/hobot_bus.h>
#include <linux/device.h>

static BLOCKING_NOTIFIER_HEAD(hb_bus_notifier_list);
static BLOCKING_NOTIFIER_HEAD(hb_usb_notifier_list);
static ATOMIC_NOTIFIER_HEAD(hb_console_notifier_list);

int hb_bus_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hb_bus_notifier_list, nb);
}
EXPORT_SYMBOL(hb_bus_register_client);

int hb_bus_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hb_bus_notifier_list, nb);
}
EXPORT_SYMBOL(hb_bus_unregister_client);

int hb_bus_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&hb_bus_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(hb_bus_notifier_call_chain);

int hb_usb_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hb_usb_notifier_list, nb);
}
EXPORT_SYMBOL(hb_usb_register_client);

int hb_usb_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hb_usb_notifier_list, nb);
}
EXPORT_SYMBOL(hb_usb_unregister_client);

int hb_usb_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&hb_usb_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(hb_usb_notifier_call_chain);

int hb_console_register_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&hb_console_notifier_list, nb);
}
EXPORT_SYMBOL(hb_console_register_client);

int hb_console_unregister_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&hb_console_notifier_list, nb);
}
EXPORT_SYMBOL(hb_console_unregister_client);

int hb_console_notifier_call_chain(unsigned long val, void *v)
{
	return atomic_notifier_call_chain(&hb_console_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(hb_console_notifier_call_chain);

