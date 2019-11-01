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

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_DEV_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_DEV_H_

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>

#include "inc/camera_subdev.h"

#define CHAR_DEVNAME_LEN  20
#define I2C_BUS  1
#define SPI_BUS  2

#define CAMERA_IOC_MAGIC    'h'
#define SENSOR_TURNING_PARAM  _IOW(CAMERA_IOC_MAGIC, 0, sensor_turning_data_t)
#define X2A_VIDIOC_I2CREAD       _IOW(CAMERA_IOC_MAGIC, 1, x2a_camera_i2c_t)
#define X2A_VIDIOC_I2CWRITE       _IOW(CAMERA_IOC_MAGIC, 2, x2a_camera_i2c_t)

typedef struct _camera_charmod_s {
	char name[CHAR_DEVNAME_LEN];
	uint32_t devflag;
	spinlock_t slock;
	uint32_t user_num;
	int dev_minor_id;
	struct miscdevice camera_chardev;
	uint32_t port;
	struct i2c_client *client;
    struct i2c_board_info board_info;
	struct spi_device *spidev;
	struct spi_board_info spi_board;
    sensor_turning_data_t camera_param;
} camera_charmod_s;

extern camera_charmod_s *camera_mod[CAMERA_TOTAL_NUMBER];

typedef struct _x2a_camera_i2c_t {
	uint32_t i2c_addr;
	uint32_t reg_size;
	uint32_t reg;
	uint32_t data;
} x2a_camera_i2c_t;

int camera_cdev_init(void);
void camera_cdev_exit(void);

#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_DEV_H_
