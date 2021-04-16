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

#ifndef __HOBOT_MIPI_DEV_H__
#define __HOBOT_MIPI_DEV_H__

#include <linux/types.h>

#define MIPIDEV_CHANNEL_NUM (4)
#define MIPIDEV_CHANNEL_0   (0)
#define MIPIDEV_CHANNEL_1   (1)
#define MIPIDEV_CHANNEL_2   (2)
#define MIPIDEV_CHANNEL_3   (3)

typedef struct _mipi_dev_cfg_t {
	uint16_t lane;
	uint16_t datatype;
	uint16_t fps;
	uint16_t mclk;
	uint16_t mipiclk;
	uint16_t width;
	uint16_t height;
	uint16_t linelenth;
	uint16_t framelenth;
	uint16_t settle;
	uint16_t vpg;
	uint16_t ipi_lines;
	uint16_t channel_num;
	uint16_t channel_sel[MIPIDEV_CHANNEL_NUM];
} mipi_dev_cfg_t;

typedef struct _mipi_dev_ipi_info_t {
	uint16_t index;
	uint16_t fatal;
	uint16_t mode;
	uint16_t vc;
	uint16_t datatype;
	uint16_t maxfnum;
	uint32_t pixels;
	uint32_t lines;
} mipi_dev_ipi_info_t;

#define MIPIDEVIOC_MAGIC 'v'
#define MIPIDEVIOC_INIT             _IOW(MIPIDEVIOC_MAGIC, 0, mipi_dev_cfg_t)
#define MIPIDEVIOC_DEINIT           _IO(MIPIDEVIOC_MAGIC,  1)
#define MIPIDEVIOC_START            _IO(MIPIDEVIOC_MAGIC,  2)
#define MIPIDEVIOC_STOP             _IO(MIPIDEVIOC_MAGIC,  3)
#define MIPIDEVIOC_IPI_GET_INFO     _IOR(MIPIDEVIOC_MAGIC, 4, mipi_dev_ipi_info_t)
#define MIPIDEVIOC_IPI_SET_INFO     _IOW(MIPIDEVIOC_MAGIC, 5, mipi_dev_ipi_info_t)

#endif /*__HOBOT_MIPI_DEV_H__*/
