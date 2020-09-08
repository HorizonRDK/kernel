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

#ifndef DRIVERS_PINCTRL_HOBOT_HB_PINCTRL_CDEV_H_
#define DRIVERS_PINCTRL_HOBOT_HB_PINCTRL_CDEV_H_

#include <linux/kernel.h>

#define PINCTRL_SET  0x100
#define PINCTRL_GET  0x101

#define PINCTRL_SET_SECHMITT (0x0001)
#define PINCTRL_SET_PULL     (0x0002)
#define PINCTRL_SET_DRIVE    (0x0004)
#define PINCTRL_SET_FUNC     (0x0008)
#define PINCTRL_SET_ALL      (PINCTRL_SET_SECHMITT | PINCTRL_SET_PULL | \
							  PINCTRL_SET_DRIVE | PINCTRL_SET_FUNC)

#define PINCTRL_IOCTRL_MAX_PINS (128)

enum {
    IOCTL_PIN_FUNC_0 = 0,
    IOCTL_PIN_FUNC_1,
    IOCTL_PIN_FUNC_2,
	IOCTL_PIN_FUNC_3,
	IOCTL_PIN_FUNC_MAX,
};

enum {
    IOCTL_PIN_PULL_DIS = 0,
    IOCTL_PIN_PULL_UP,
    IOCTL_PIN_PULL_DOWN,
	IOCTL_PIN_PULL_MAX,
};

enum {
    IOCTL_PIN_SECHMITT_DIS = 0,
    IOCTL_PIN_SECHMITT_EN,
	IOCTL_PIN_SECHMITT_MAX,
};

enum {
	DRIVE_SET_ROUND = 0,
	DRIVE_SET_CEIL,
	DRIVE_SET_FLOOR,
};

struct hb_pinctrl_config {
    unsigned char pin_func;
    unsigned char pin_pull;
    unsigned char pin_drive;
    unsigned char pin_sechmitt;
    unsigned int  pin_num;
	unsigned int  set_mask;
};

struct hb_pinctrl_ioctrl_data {
    struct hb_pinctrl_config __user *config;
    unsigned int npins;
};

#endif //DRIVERS_PINCTRL_HOBOT_HB_PINCTRL_CDEV_H_
