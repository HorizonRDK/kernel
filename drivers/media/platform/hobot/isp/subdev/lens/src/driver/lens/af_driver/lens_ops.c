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
 *    @file      lens_ops.c
 *    @brief     provide ops func
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
#include "lens_ops.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#endif

void motor_calculate_param(void *ctx, uint32_t *param)
{
	struct motor_param_s *dev_param = (struct motor_param_s *)(ctx);

	if ((dev_param == NULL) || (param == NULL)) {
		return;
	}

	*param = 10;
}

