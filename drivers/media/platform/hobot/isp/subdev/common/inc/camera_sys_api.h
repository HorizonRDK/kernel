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

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SYS_API_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SYS_API_H_


enum control_mode_e {
        NORMAL_M = 0x00,
        DOL2_M = 0x01,
        DOL3_M = 0x02,
        PWL = 0x03
};

int camera_sys_priv_set(uint32_t port, sensor_priv_t *priv_param);
int camera_sys_get_param(uint32_t port, sensor_data_t *sensor_data);
int camera_sys_turining_set(uint32_t port, sensor_turning_data_t *turning_pram);
int	camera_sys_alloc_again(uint32_t port, uint32_t *a_gain);
int camera_sys_alloc_dgain(uint32_t port, uint32_t *a_gain);
int camera_sys_alloc_intergration_time(uint32_t port,
		uint32_t *intergration_time);
int camera_sys_sensor_write(uint32_t port, uint32_t address, uint32_t w_data);
int camera_sys_sensor_read(uint32_t port, uint32_t address, uint32_t *r_data);
int camera_sys_stream_on(uint32_t port);
int camera_sys_stream_off(uint32_t port);


#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SYS_API_H_
