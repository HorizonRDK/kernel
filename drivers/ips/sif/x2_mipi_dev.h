/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file     mipi_dev.h
 * @brief    MIPI DEV Common define
 * @author   tarryzhang (tianyu.zhang@hobot.cc)
 * @date     2017/7/6
 * @version  V1.0
 * @par      Horizon Robotics
 */
#ifndef __X2_MIPI_DEV_H__
#define __X2_MIPI_DEV_H__

#include <linux/types.h>

#define MIPI_DEV_CSI2_RAISE         (0x01)
#define MIPI_DEV_CSI2_RESETN        (0x00)

typedef struct _mipi_dev_control_t {
	uint16_t lane;
	uint16_t datatype;
	uint16_t vpg;
	uint16_t mclk;
	uint16_t fps;
	uint16_t mipiclk;
	uint16_t width;
	uint16_t height;
	uint16_t linelenth;
	uint16_t framelenth;
	uint16_t settle;
} mipi_dev_control_t;

int32_t mipi_dev_start(void);
int32_t mipi_dev_stop(void);
int32_t mipi_dev_init(mipi_dev_control_t * control);
int32_t mipi_dev_deinit(void);

#endif /*__X2_MIPI_DEV_H__*/
