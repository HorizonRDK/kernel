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

#ifndef __HOBOT_MIPI_HOST_H__
#define __HOBOT_MIPI_HOST_H__

#include <linux/types.h>

#define MIPIHOST_CHANNEL_NUM (4)
#define MIPIHOST_CHANNEL_0   (0)
#define MIPIHOST_CHANNEL_1   (1)
#define MIPIHOST_CHANNEL_2   (2)
#define MIPIHOST_CHANNEL_3   (3)

typedef struct _mipi_host_cfg_t {
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
	uint16_t hsaTime;
	uint16_t hbpTime;
	uint16_t hsdTime;
	uint16_t channel_num;
	uint16_t channel_sel[MIPIHOST_CHANNEL_NUM];
} mipi_host_cfg_t;

typedef struct _mipi_host_ipi_reset_t {
	uint16_t mask;
	uint16_t enable;
} mipi_host_ipi_reset_t;

typedef struct _mipi_host_ipi_info_t {
	uint16_t index;
	uint16_t fatal;
	uint16_t mode;
	uint16_t vc;
	uint16_t datatype;
	uint16_t hsa;
	uint16_t hbp;
	uint16_t hsd;
	uint32_t adv;
} mipi_host_ipi_info_t;

#define MIPIHOSTIOC_MAGIC 'v'
#define MIPIHOSTIOC_INIT             _IOW(MIPIHOSTIOC_MAGIC, 0, mipi_host_cfg_t)
#define MIPIHOSTIOC_DEINIT           _IO(MIPIHOSTIOC_MAGIC,  1)
#define MIPIHOSTIOC_START            _IO(MIPIHOSTIOC_MAGIC,  2)
#define MIPIHOSTIOC_STOP             _IO(MIPIHOSTIOC_MAGIC,  3)
#define MIPIHOSTIOC_SNRCLK_SET_EN    _IOW(MIPIHOSTIOC_MAGIC, 4, uint32_t)
#define MIPIHOSTIOC_SNRCLK_SET_FREQ  _IOW(MIPIHOSTIOC_MAGIC, 5, uint32_t)
#define MIPIHOSTIOC_PRE_INIT_REQUEST _IOW(MIPIHOSTIOC_MAGIC, 6, uint32_t)
#define MIPIHOSTIOC_PRE_START_REQUEST _IOW(MIPIHOSTIOC_MAGIC, 7, uint32_t)
#define MIPIHOSTIOC_PRE_INIT_RESULT  _IOW(MIPIHOSTIOC_MAGIC, 8, uint32_t)
#define MIPIHOSTIOC_PRE_START_RESULT _IOW(MIPIHOSTIOC_MAGIC, 9, uint32_t)
#define MIPIHOSTIOC_IPI_RESET        _IOW(MIPIHOSTIOC_MAGIC, 10, mipi_host_ipi_reset_t)
#define MIPIHOSTIOC_IPI_GET_INFO     _IOR(MIPIHOSTIOC_MAGIC, 11, mipi_host_ipi_info_t)
#define MIPIHOSTIOC_IPI_SET_INFO     _IOW(MIPIHOSTIOC_MAGIC, 12, mipi_host_ipi_info_t)

#endif /*__HOBOT_MIPI_HOST_H__*/
