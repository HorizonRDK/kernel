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

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SUBDEV_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SUBDEV_H_

#define CAMERA_TOTAL_NUMBER  4
#define V4L2_CAMERA_NAME "camera"

typedef struct sensor_priv {
	  uint32_t gain_num;
	  uint32_t *gain_buf;
	  uint32_t en_dgain;
	  uint32_t line_num;
	  uint32_t *line_buf;
	  uint8_t  mode;
}sensor_priv_t;

struct sensor_arg {
	 uint32_t port;
	 sensor_priv_t *sensor_priv;
};

typedef struct dol3_s {
	uint32_t s_gain;
	uint32_t s_gain_length;
	uint32_t m_gain;
	uint32_t m_gain_length;
	uint32_t l_gain;
	uint32_t l_gain_length;
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t l_line;
	uint32_t l_line_length;
} dol3_t;

typedef struct dol2_s {
	uint32_t s_gain;
    uint32_t s_gain_length;
	uint32_t m_gain;
	uint32_t m_gain_length;
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
}dol2_t;

typedef struct normal_s {
	uint32_t s_gain;
	uint32_t s_gain_length;
	uint32_t s_line;
	uint32_t s_line_length;
}normal_t;

typedef struct sensor_data {
	uint32_t  gain_type;
	uint32_t  VMAX;
	uint32_t  HMAX;
	uint32_t  FSC_DOL2;
	uint32_t  FSC_DOL3;
	uint32_t  RHS1;
	uint32_t  RHS2;
	uint32_t  lane;
	uint32_t  clk;
	uint32_t  frame;
	uint32_t  gain_max;
	uint32_t  lines_per_second;
	uint32_t  analog_gain_max;
	uint32_t  digital_gain_max;
	uint32_t  exposure_time_max;
	uint32_t  exposure_time_min;
	uint32_t  exposure_time_long_max;
	uint32_t  active_width;
	uint32_t  active_height;
}sensor_data_t;

typedef struct sensor_turning_data {
	uint32_t  port;
	char     *sensor_name;
	uint32_t  sensor_addr;
    uint32_t  bus_num;
	uint32_t  bus_type;
	uint32_t  reg_length;
	uint32_t  mode;
	uint32_t  cs;
	uint32_t  spi_mode;
	uint32_t  spi_speed;
	normal_t normal;
	dol2_t   dol2;
	dol3_t   dol3;
	sensor_data_t sensor_data;
}sensor_turning_data_t;

struct sensor_basic_arg {
	 uint32_t port;
	 sensor_turning_data_t *sensor_turning;
};

enum camera_IOCTL {
    SENSOR_UPDATE = 0,
    SENSOR_GET_PARAM
};

#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SUBDEV_H_

