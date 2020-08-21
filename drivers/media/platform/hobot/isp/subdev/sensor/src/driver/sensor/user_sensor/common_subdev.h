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

#ifndef __COMMON_CAMERA_SUBDEV_H__
#define __COMMON_CAMERA_SUBDEV_H__

#define V4L2_CAMERA_NAME "camera"

typedef struct sensor_priv {
	uint32_t gain_num;
	uint32_t gain_buf[4];
	uint32_t dgain_num;
	uint32_t dgain_buf[4];
	uint32_t en_dgain;
	uint32_t line_num;
	uint32_t line_buf[4];
	uint32_t rgain;
	uint32_t bgain;
	uint8_t  mode;
} sensor_priv_t;

typedef struct sensor_data {
	uint32_t  turning_type;
	uint32_t  step_gain;
	uint32_t  again_prec;
	uint32_t  dgain_prec;
	uint32_t  conversion;
	uint32_t  VMAX;
	uint32_t  HMAX;
	uint32_t  FSC_DOL2;
	uint32_t  FSC_DOL3;
	uint32_t  RHS1;
	uint32_t  RHS2;
	uint32_t  lane;
	uint32_t  clk;
	uint32_t  fps;
	uint32_t  gain_max;
	uint32_t  pixels_per_line;
	uint32_t  analog_gain_max;
	uint32_t  digital_gain_max;
	uint32_t  exposure_time_max;
	uint32_t  exposure_time_min;
	uint32_t  exposure_time_long_max;
	uint32_t  active_width;
	uint32_t  active_height;
} sensor_data_t;

struct sensor_arg {
	 uint32_t port;
	 sensor_priv_t *sensor_priv;
	 sensor_data_t *sensor_data;
	 uint32_t *a_gain;
	 uint32_t *d_gain;
	 uint32_t  address;
	 uint32_t  w_data;
	 uint32_t  *r_data;
	 uint32_t *integration_time;
};

enum SENSOR_MODE {
	SENSOR_LINEAR = 1,
	SENSOR_DOL2,
	SENSOR_DOL3,
	SENSOR_DOL4,
	SENSOR_PWL,
};

enum camera_IOCTL {
	SENSOR_UPDATE = 0,
	SENSOR_WRITE,
	SENSOR_READ,
	SENSOR_GET_PARAM,
	SENSOR_STREAM_ON,
	SENSOR_STREAM_OFF,
	SENSOR_ALLOC_ANALOG_GAIN,
	SENSOR_ALLOC_DIGITAL_GAIN,
	SENSOR_ALLOC_INTEGRATION_TIME,
	SENSOR_AWB_UPDATE
};

#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SUBDEV_H_

