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

#define SIF_LOG_INFO (1)
extern unsigned int sif_debug_level;
#define sifinfo(format, ...) \
	do {\
		if((sif_debug_level >= SIF_LOG_INFO)) \
			printk(KERN_INFO "[sif][info]: "format "\n", ##__VA_ARGS__); \
	} while(0)
#define siferr(format, ...) printk(KERN_ERR "[sif][error]: "format "\n", ##__VA_ARGS__)

#endif //__X2_SIF_UTILS_H__
