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

#ifndef __HOBOT_BUS_H__
#define __HOBOT_BUS_H__

#define HB_BUS_SIGNAL_NONE          0
#define HB_BUS_SIGNAL_START   1
#define HB_BUS_SIGNAL_END      2

#define UNKOWN_STATE    0
#define POWERSAVE_STATE 1
#define OTHER_STATE     2

struct hobot_dpm;
typedef int (*dpm_fn_t)(struct hobot_dpm *dpm,
		unsigned long val, int state);

struct hobot_dpm {
        struct list_head entry;
        struct device *dev;
        dpm_fn_t dpm_call;
        int priority;
};

void hobot_dpm_register(struct hobot_dpm *n, struct device *dev);
void hobot_dpm_unregister(struct hobot_dpm *n);

int hb_bus_register_client(struct notifier_block *nb);
int hb_bus_unregister_client(struct notifier_block *nb);

int hb_usb_register_client(struct notifier_block *nb);
int hb_usb_unregister_client(struct notifier_block *nb);

int hb_atomic_register_client(struct notifier_block *nb);
int hb_atomic_unregister_client(struct notifier_block *nb);
#endif
