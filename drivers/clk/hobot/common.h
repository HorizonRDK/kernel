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

#ifndef __CLK_H__
#define __CLK_H__

extern void __iomem * clk_get_register_base(struct device_node *np);
extern void __iomem * clk_get_ipsregister_base(struct device_node *np);

#endif
