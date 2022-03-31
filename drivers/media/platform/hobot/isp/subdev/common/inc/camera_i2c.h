/*    Copyright (C) 2018 Horizon Inc.
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
 */

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_I2C_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_I2C_H_

#define CAM_I2C_RETRY_MAX	3

int camera_i2c_release(uint32_t port);
int camera_i2c_open(uint32_t port, uint32_t i2c_bus,
			char *sensor_name, uint32_t sensor_addr);
int camera_i2c_read(uint32_t port, uint32_t reg_addr,
			uint32_t bit_width, char *buf, uint32_t count);
int camrea_i2c_adapter_read(uint32_t port, uint16_t slave_addr,
		uint32_t reg_addr, int bit_width,
		uint16_t *value, int count);
int camera_i2c_write(uint32_t port, uint32_t reg_addr,
			uint32_t bit_width, const char *buf, uint32_t count);

#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_I2C_H_

