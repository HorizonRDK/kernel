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

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SPI_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SPI_H_


int camera_spi_open(uint32_t port, uint32_t spi_bus,
	uint32_t cs, uint32_t mode, uint32_t speed, char *sensor_name);
int camera_spi_write(uint32_t port, uint32_t chip_id, uint32_t reg_width,
		uint32_t reg_addr, void *buf, uint32_t length);
int camera_spi_read(uint32_t port, uint32_t chip_id, uint32_t reg_width,
		uint32_t reg_addr, void *buf, uint32_t length);

#endif //DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SPI_H_
