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

#ifndef __HOBOT_DDR_AXICTRL_H__
#define __HOBOT_DDR_AXICTRL_H__

extern struct attribute_group axibus_group;

extern int hobot_ddr_axictrl_init(struct platform_device *pdev);
extern int hobot_ddr_axictrl_deinit(struct platform_device *pdev);

#endif //__HOBOT_DDR_AXICTRL_H__
