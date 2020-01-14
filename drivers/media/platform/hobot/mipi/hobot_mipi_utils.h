/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2020 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/

/**
 * @file     hobot_mipi_utils.h
 * @brief    Mipi utils head file
 * @author   tarryzhang (tianyu.zhang@hobot.cc)
 * @date     2017/7/6
 * @version  V1.0
 * @par      Horizon Robotics
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
#define mipidbg(format, ...)	\
	do {						\
		if (param->dbg_value > 0)	\
			dev_dbg_ratelimited(dev, format "\n", ##__VA_ARGS__);	\
	} while (0)

#define mipi_getreg(a)          readl(a)
#define mipi_putreg(a,v)\
	do {\
		writel(v,a); \
	} while(0)

#endif //__HOBOT_MIPI_UTILS_H__
