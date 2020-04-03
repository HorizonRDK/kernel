/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2020 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/

/**
 * @file     hobot mipi_host.h
 * @brief    MIPI HOST Common define
 * @author   tarryzhang (tianyu.zhang@hobot.cc)
 * @date     2017/7/6
 * @version  V1.0
 * @par      Horizon Robotics
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

#define MIPIHOSTIOC_MAGIC 'v'
#define MIPIHOSTIOC_INIT             _IOW(MIPIHOSTIOC_MAGIC, 0, mipi_host_cfg_t)
#define MIPIHOSTIOC_DEINIT           _IO(MIPIHOSTIOC_MAGIC,  1)
#define MIPIHOSTIOC_START            _IO(MIPIHOSTIOC_MAGIC,  2)
#define MIPIHOSTIOC_STOP             _IO(MIPIHOSTIOC_MAGIC,  3)
#define MIPIHOSTIOC_SNRCLK_SET_EN    _IOW(MIPIHOSTIOC_MAGIC, 4, uint32_t)
#define MIPIHOSTIOC_SNRCLK_SET_FREQ  _IOW(MIPIHOSTIOC_MAGIC, 5, uint32_t)
#define MIPIHOSTIOC_GET_INIT_CNT     _IOR(MIPIHOSTIOC_MAGIC, 6, uint32_t)
#define MIPIHOSTIOC_GET_START_CNT    _IOR(MIPIHOSTIOC_MAGIC, 7, uint32_t)
#define MIPIHOSTIOC_PUT_CNT          _IO(MIPIHOSTIOC_MAGIC, 8)

#endif /*__HOBOT_MIPI_HOST_H__*/
