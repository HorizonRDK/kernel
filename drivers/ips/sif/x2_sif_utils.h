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
#ifndef __X2_SIF_UTILS_H__
#define __X2_SIF_UTILS_H__

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>

#define CONFIG_SIF_DEBUG

#ifdef CONFIG_SIF_DEBUG
#define sifinfo(format, ...)   printk(KERN_INFO format "\n" , ##__VA_ARGS__)
#define siferr(format, ...)    printk(KERN_ERR format "\n" , ##__VA_ARGS__)
#else
#define sifinfo(format, ...)
#define siferr(format, ...)    printk(KERN_ERR format "\n" , ##__VA_ARGS__)
#endif

#define sif_getreg(a)          readl(a)
#define sif_putreg(a,v)        writel(v,a)

#endif //__X2_SIF_UTILS_H__
