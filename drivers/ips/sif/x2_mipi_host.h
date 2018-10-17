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

#define MIPI_HOST_CSI2_RAISE       (0x01)
#define MIPI_HOST_CSI2_RESETN      (0x00)
#define MIPI_HOST_BITWIDTH_48      (0)
#define MIPI_HOST_BITWIDTH_16      (1)

typedef struct _mipi_host_control_t {
	uint16_t lane;
	uint16_t pixlen;
	uint16_t datatype;
	uint16_t bitWidth;
	uint16_t fps;
	uint16_t mclk;
	uint16_t mipiclk;
	uint16_t pixclk;
	uint16_t width;
	uint16_t height;
	uint16_t linelenth;
	uint16_t framelenth;
	uint16_t settle;
	uint16_t hsaTime;
	uint16_t hbpTime;
	uint16_t hsdTime;
} mipi_host_control_t;

int32_t mipi_host_start(void);
int32_t mipi_host_stop(void);
int32_t mipi_host_init(mipi_host_control_t * control);
void mipi_host_deinit(void);

#endif /*__X2_MIPI_HOST_H__*/
