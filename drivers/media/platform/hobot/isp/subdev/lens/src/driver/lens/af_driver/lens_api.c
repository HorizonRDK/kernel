/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    @file      lens_api.c
 *    @brief     provide api for isp driver
 *    @details 
 *    @mainpage 
 *    @author    IE&E
 *    @email     yongyong.duan@horizon.ai
 *    @version   v-1.0.0
 *    @date      2020-1-17
 *    @license
 *    @copyright
 *********************************************************************/

#include "acamera_logger.h"
#include "lens_api.h"
#include "lens_driver.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#endif

int lens_api_af_init(uint16_t chn)
{
	return 0;
}

int lens_api_af_move(uint16_t chn, uint32_t pos)
{
	int ret = 0;

	ret = lens_driver_move(chn, LENS_AF_PARAM_ID, pos);

	return ret;
}

void lens_api_af_stop(uint16_t chn)
{
	lens_driver_stop(chn, LENS_AF_PARAM_ID);
}

uint8_t lens_api_af_get_status(uint16_t chn)
{
	return lens_driver_get_status(chn, LENS_AF_PARAM_ID);
}

uint32_t lens_api_af_get_pos(uint16_t chn)
{
	return lens_driver_get_pos(chn, LENS_AF_PARAM_ID);
}

int lens_api_af_get_param(uint16_t chn)
{
	return 0;
}

void lens_api_af_write_reg(uint16_t chn, uint32_t addr, uint32_t data)
{
	lens_driver_write_reg(chn, LENS_AF_PARAM_ID, addr, data);
}

void lens_api_af_read_reg(uint16_t chn, uint32_t addr, uint32_t *data)
{
	*data = lens_driver_read_reg(chn, LENS_AF_PARAM_ID, addr);
}

int lens_api_zoom_init(uint16_t chn)
{
	return 0;
}

int lens_api_zoom_move(uint16_t chn, uint32_t pos)
{
	int ret = 0;

	ret = lens_driver_move(chn, LENS_ZOOM_PARAM_ID, pos);

	return ret;
}

void lens_api_zoom_stop(uint16_t chn)
{
	lens_driver_stop(chn, LENS_ZOOM_PARAM_ID);
}

uint8_t lens_api_zoom_get_status(uint16_t chn)
{
	return lens_driver_get_status(chn, LENS_ZOOM_PARAM_ID);
}

uint32_t lens_api_zoom_get_pos(uint16_t chn)
{
	return lens_driver_get_pos(chn, LENS_ZOOM_PARAM_ID);
}

int lens_api_zoom_get_param(uint16_t chn)
{
	return 0;
}

void lens_api_zoom_write_reg(uint16_t chn, uint32_t addr, uint32_t data)
{
	lens_driver_write_reg(chn, LENS_ZOOM_PARAM_ID, addr, data);
}

void lens_api_zoom_read_reg(uint16_t chn, uint32_t addr, uint32_t *data)
{
	*data = lens_driver_read_reg(chn, LENS_ZOOM_PARAM_ID, addr);
}
