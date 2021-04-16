/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __HOBOT_SENSOR_DEV_H__
#define __HOBOT_SENSOR_DEV_H__

#include <linux/types.h>
#include <linux/videodev2.h>

#define SENSOR_NAME_LENTH_MAX    (32)
typedef struct _mipi_param_t {
	uint32_t    lane_num;
	uint32_t    fps;
	uint32_t    mclk;
	uint32_t    mipiclk;
	uint32_t    linelenth;
	uint32_t    framelenth;
	uint32_t    settle;
} mipi_param_t;

typedef enum _sensor_order_e {
	SENSOR_ORDER_YUYV,
	SENSOR_ORDER_YVYU,
	SENSOR_ORDER_UYVU,
	SENSOR_ORDER_VYUY,
	SENSOR_ORDER_MAX,
} sensor_order_t;

typedef enum _sensor_format_t {
	SENSOR_FMT_YUV,
	SENSOR_FMT_RAW,
	SENSOR_FMT_RGB,
	SENSOR_FMT_MAX,
} sensor_format_t;

typedef enum _sensor_bus_t {
	SENSOR_BUS_DVP,
	SENSOR_BUS_BT1120,
	SENSOR_BUS_MIPI,
	SENSOR_BUS_MAX,
} sensor_bus_t;

typedef enum _sensor_pix_len_e {
	SENSOR_PIX_LEN_8,
	SENSOR_PIX_LEN_10,
	SENSOR_PIX_LEN_12,
	SENSOR_PIX_LEN_14,
	SENSOR_PIX_LEN_16,
	SENSOR_PIX_LEN_20,
	SENSOR_PIX_LEN_MAX,
} sensor_pix_len_t;

typedef enum _sensor_i2c_data_type_e {
	SENSOR_I2C_BYTE_DATA = 1,
	SENSOR_I2C_WORD_DATA,
	SENSOR_I2C_DWORD_DATA,
	SENSOR_I2C_DATA_TYPE_MAX,
} sensor_i2c_data_type_t;

typedef enum _sensor_power_type_e {
	SENSOR_POWER_TYPE_VA = 1,
	SENSOR_POWER_TYPE_VD,
	SENSOR_POWER_TYPE_VIO,
	SENSOR_POWER_TYPE_GPIO,
	SENSOR_POWER_TYPE_MAX,
} sensor_power_type_t;

typedef struct _sensor_reg_array_t {
	uint16_t reg_addr;
	uint16_t reg_data;
	uint32_t reg_size;
} sensor_reg_array_t;

typedef struct _sensor_reg_setting_t {
	sensor_reg_array_t      *reg_setting;
	uint16_t                 size;
	uint16_t                 delay;
} sensor_reg_setting_t;

typedef struct _sensor_power_array_t {
	sensor_power_type_t      type;
	uint16_t                 pin;
	uint8_t                  value;
	uint8_t                  delay;
} sensor_power_array_t;

typedef struct _sensor_power_setting_t {
	sensor_power_array_t    *power_list;
	uint16_t                 size;
} sensor_power_setting_t;

typedef struct _sensor_chip_setting_t {
	uint16_t                 i2c_bus;
	uint16_t                 slave_addr;
	sensor_i2c_data_type_t   reg_size;
	sensor_i2c_data_type_t   data_size;
	sensor_reg_array_t       chip_id;
} sensor_chip_setting_t;

typedef struct _sensor_settings_t {
	mipi_param_t             mipi_params;
	sensor_reg_setting_t     init_settings;
} sensor_settings_t;

typedef struct _sensor_size_t {
	uint16_t  width;
	uint16_t  height;
} sensor_size_t;

typedef struct _sensor_init_t {
	uint16_t                 lane;
	sensor_pix_len_t         pixlen;
	sensor_size_t            size;
	sensor_settings_t       *sensor_settings;
} sensor_init_t;

typedef struct _sensor_info_t {
	uint8_t                *name;
	sensor_bus_t            bus;
	sensor_format_t         format;
	sensor_chip_setting_t   chip;
	sensor_power_setting_t  power;
	sensor_reg_setting_t    start;
	sensor_reg_setting_t    stop;
	sensor_reg_setting_t    order;
	sensor_init_t           *init;
	uint32_t                size;
} sensor_info_t;

typedef struct _sensor_dev_t {
	uint8_t                 *name;
	sensor_chip_setting_t   *chip;
	sensor_power_setting_t  *power;
	sensor_reg_setting_t    *init;
	sensor_reg_setting_t    *start;
	sensor_reg_setting_t    *stop;
	sensor_reg_setting_t    *order;
	mipi_param_t            *mipi;
} sensor_dev_t;

typedef struct _sensor_cfg_t {
	uint32_t                 lane;
	uint32_t                 bus;
	uint32_t                 format;
	uint32_t                 pixlen;
	uint32_t                 width;
	uint32_t                 height;
} sensor_cfg_t;

typedef struct _x2_camera_i2c_t {
	uint32_t i2c_addr;
	uint32_t reg_size;
	uint32_t reg;
	uint32_t data;
} x2_camera_i2c_t;

#define X2VIDIOC_PROP             _IOW('V', BASE_VIDIOC_PRIVATE, sensor_cfg_t)
#define X2VIDIOC_CSIPARAM         _IOR('V', BASE_VIDIOC_PRIVATE+1, mipi_param_t)
#define X2VIDIOC_I2CREAD          _IOWR('V', BASE_VIDIOC_PRIVATE+2, x2_camera_i2c_t)
#define X2VIDIOC_I2CWRITE         _IOW('V', BASE_VIDIOC_PRIVATE+3, x2_camera_i2c_t)
#define X2VIDIOC_STREAM_ON        _IO('V', BASE_VIDIOC_PRIVATE+4)
#define X2VIDIOC_STREAM_OFF       _IO('V', BASE_VIDIOC_PRIVATE+5)
int sensor_dev_prop(sensor_dev_t *sensor_dev, sensor_cfg_t *cfg, uint8_t currindex);

#endif //__HOBOT_SENSOR_DEV_H__
