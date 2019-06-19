/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file     sif_utils.h
 * @brief    SIF Device head file
 * @author   tarryzhang (tianyu.zhang@hobot.cc)
 * @date     2017/7/6
 * @version  V1.0
 * @par      Horizon Robotics
 */
#ifndef __X2_MIPI_UTILS_H__
#define __X2_MIPI_UTILS_H__

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>

#define CONFIG_MIPI_DEBUG

#ifdef CONFIG_MIPI_DEBUG
#define mipiinfo(format, ...)   printk(KERN_INFO format "\n" , ##__VA_ARGS__)
#define mipierr(format, ...)    printk(KERN_ERR format "\n" , ##__VA_ARGS__)
#else
#define mipiinfo(format, ...)
#define mipierr(format, ...)    printk(KERN_ERR format "\n" , ##__VA_ARGS__)
#endif

/* for mipi debug */
extern unsigned int dbg_value;
#define mipidbg(format, ...)	\
	do {						\
		if ((dbg_value >= 1))	\
			printk(KERN_INFO format "\n", ##__VA_ARGS__);	\
	} while (0)
/* for mipi debug */

#define mipi_getreg(a)          readl(a)
#define mipi_putreg(a,v)\
	do {\
		writel(v,a); \
		/*mipiinfo("[mipi reg]: write %p: 0x%x", a, v);*/ \
	} while(0)
#endif //__X2_SIF_UTILS_H__
