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

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_CTRL_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_CTRL_H_

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>

#include "inc/camera_subdev.h"

#define CTRL_DEVNAME_LEN  20

typedef struct sensor_ctrl_info_s {
	uint32_t port;
	uint32_t gain_num;
	uint32_t gain_buf[4];
	uint32_t dgain_num;
	uint32_t dgain_buf[4];
	uint32_t en_dgain;
	uint32_t line_num;
	uint32_t line_buf[4];
	uint32_t rgain;
	uint32_t bgain;
	uint32_t grgain;
	uint32_t gbgain;
	uint32_t af_pos;
	uint32_t zoom_pos;
	uint32_t mode;
} sensor_ctrl_info_t;

#define CAMERA_CTRL_IOC_MAGIC    'x'
#define SENSOR_CTRL_INFO_SYNC	_IOWR(CAMERA_CTRL_IOC_MAGIC, 20, sensor_ctrl_info_t)

typedef struct _camera_ctrlmod_s {
        char name[CTRL_DEVNAME_LEN];
        uint32_t user_num;
        int dev_minor_id;
        struct miscdevice camera_chardev;
        struct mutex m_mutex;
        // wait queue
        uint32_t port_flag[8];
} camera_ctrlmod_s;

extern camera_ctrlmod_s *camera_ctrl;

void set_sensor_aexp_info(uint32_t port, void *ptr);
int camera_ctrldev_init(void);
void camera_ctrldev_exit(void);
void sensor_ctrl_wakeup(uint32_t port);

#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_DEV_H_
