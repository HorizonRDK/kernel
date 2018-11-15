/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file     mipi_host.h
 * @brief    MIPI HOST Common define
 * @author   tarryzhang (tianyu.zhang@hobot.cc)
 * @date     2017/7/6
 * @version  V1.0
 * @par      Horizon Robotics
 */
#ifndef __X2_MIPI_HOST_H__
#define __X2_MIPI_HOST_H__

#include <linux/types.h>

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
} mipi_host_cfg_t;

#define MIPIHOSTIOC_MAGIC 'v'
#define MIPIHOSTIOC_INIT             _IOW(MIPIHOSTIOC_MAGIC, 0, mipi_host_cfg_t)
#define MIPIHOSTIOC_DEINIT           _IO(MIPIHOSTIOC_MAGIC,  1)
#define MIPIHOSTIOC_START            _IO(MIPIHOSTIOC_MAGIC,  2)
#define MIPIHOSTIOC_STOP             _IO(MIPIHOSTIOC_MAGIC,  3)

#endif /*__X2_MIPI_HOST_H__*/
