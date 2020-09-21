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

int hb_bus_register_client(struct notifier_block *nb);
int hb_bus_unregister_client(struct notifier_block *nb);

int hb_usb_register_client(struct notifier_block *nb);
int hb_usb_unregister_client(struct notifier_block *nb);

int hb_console_register_client(struct notifier_block *nb);
int hb_console_unregister_client(struct notifier_block *nb);
#endif
