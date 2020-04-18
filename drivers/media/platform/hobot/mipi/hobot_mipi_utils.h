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

#ifndef __HOBOT_MIPI_UTILS_H__
#define __HOBOT_MIPI_UTILS_H__

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>

#define CONFIG_MIPI_DEBUG

/* should has: struct device *dev; */
#ifdef CONFIG_MIPI_DEBUG
#define mipiinfo(format, ...)   dev_info(dev, format "\n" , ##__VA_ARGS__)
#define mipierr(format, ...)    dev_err(dev, format "\n" , ##__VA_ARGS__)
#else
#define mipiinfo(format, ...)
#define mipierr(format, ...)    dev_err(dev, format "\n" , ##__VA_ARGS__)
#endif

/* should has: param include dbg_value */
			// dev_info_ratelimited(dev, format "\n", ##__VA_ARGS__);
#define mipidbg(format, ...)	\
	do {						\
		if (param->dbg_value > 0)	\
			dev_info(dev, format "\n", ##__VA_ARGS__);	\
	} while (0)

#define mipi_getreg(a)          readl(a)
#define mipi_putreg(a,v)\
	do {\
		writel(v,a); \
	} while(0)

#endif //__HOBOT_MIPI_UTILS_H__
