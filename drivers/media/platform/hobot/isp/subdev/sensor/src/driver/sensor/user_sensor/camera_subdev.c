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
 *********************************************************************/
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>

#include "acamera_logger.h"
#include "../sensor_i2c.h"
#include "acamera_firmware_config.h"
#include "camera_subdev.h"
#include "common_subdev.h"

sensor_priv_t sensor_ctl[FIRMWARE_CONTEXT_NUMBER];
sensor_data_t sensor_param[FIRMWARE_CONTEXT_NUMBER];
struct v4l2_subdev *common_subdev;

extern struct v4l2_subdev * get_sensor_subdev(const char* name);

static int common_control_init(void)
{
	int ret = 0;

	common_subdev = get_sensor_subdev(V4L2_CAMERA_NAME);
	if (common_subdev == NULL) {
		LOG(LOG_DEBUG, "get subdev failed!");
		ret = -1;
	}

	return ret;
}

#if 0
static void common_control_deinit(void)
{
	common_subdev = NULL;
}
#endif

static void common_alloc_analog_gain(uint8_t chn, int32_t *gain_ptr, uint32_t gain_num)
{
	int ret = 0;
	struct sensor_arg settings;

	if ((common_subdev != NULL) && (chn < FIRMWARE_CONTEXT_NUMBER) && (gain_ptr != NULL)) {
		settings.port = (uint32_t)chn;
		settings.a_gain = (uint32_t *)gain_ptr;
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_ALLOC_ANALOG_GAIN, &settings);

                switch (gain_num) {
                case 3:
			sensor_ctl[chn].gain_buf[2] = (uint32_t)gain_ptr[2];
                case 2:
			sensor_ctl[chn].gain_buf[1] = (uint32_t)gain_ptr[1];
                case 1:
			sensor_ctl[chn].gain_buf[0] = (uint32_t)gain_ptr[0];
			sensor_ctl[chn].gain_num = gain_num;
                        break;
                default:
			sensor_ctl[chn].gain_num = 0;
                }
#if 0
		switch (sensor_ctl[chn].mode) {
		case SENSOR_LINEAR:
		case SENSOR_PWL:
			sensor_ctl[chn].gain_buf[0] = analog_gain;
			sensor_ctl[chn].gain_num = 1;
			LOG(LOG_ERR, "liner gain is %d", analog_gain);
		break;
		case SENSOR_DOL2:
			sensor_ctl[chn].gain_buf[0] = analog_gain;
			//sensor_ctl[chn].gain_buf[1] = analog_gain;
			sensor_ctl[chn].gain_num = 1;
		break;
		case SENSOR_DOL3:
			sensor_ctl[chn].gain_buf[0] = analog_gain;
			//sensor_ctl[chn].gain_buf[1] = analog_gain;
			//sensor_ctl[chn].gain_buf[2] = analog_gain;
			sensor_ctl[chn].gain_num = 1;
		break;
		case SENSOR_DOL4:
			LOG(LOG_ERR, "common subdev pointer is NULL");
		break;
		default:
			LOG(LOG_ERR, "sensor mode is error");
		break;
		}
#endif
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}
}

static void common_alloc_digital_gain(uint8_t chn, int32_t *gain_ptr, uint32_t gain_num)
{
	int ret = 0;
	struct sensor_arg settings;

	if ((common_subdev != NULL) && (chn < FIRMWARE_CONTEXT_NUMBER) && (gain_ptr != NULL)) {
		settings.port = (uint32_t)chn;
		settings.d_gain = (uint32_t *)gain_ptr;
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_ALLOC_DIGITAL_GAIN, &settings);

                switch (gain_num) {
                case 3:
			sensor_ctl[chn].dgain_buf[2] = (uint32_t)gain_ptr[2];
                case 2:
			sensor_ctl[chn].dgain_buf[1] = (uint32_t)gain_ptr[1];
                case 1:
			sensor_ctl[chn].dgain_buf[0] = (uint32_t)gain_ptr[0];
			sensor_ctl[chn].dgain_num = gain_num;
                        break;
                default:
			sensor_ctl[chn].dgain_num = 0;
                }
#if 0
		switch (sensor_ctl[chn].mode) {
		case SENSOR_LINEAR:
		case SENSOR_PWL:
			sensor_ctl[chn].dgain_buf[0] = digital_gain;
			sensor_ctl[chn].dgain_num = 1;
		break;
		case SENSOR_DOL2:
			sensor_ctl[chn].dgain_buf[0] = digital_gain;
			//sensor_ctl[chn].gain_buf[1] = digital_gain;
			sensor_ctl[chn].dgain_num = 1;
		break;
		case SENSOR_DOL3:
			sensor_ctl[chn].dgain_buf[0] = digital_gain;
			//sensor_ctl[chn].gain_buf[1] = digital_gain;
			//sensor_ctl[chn].gain_buf[2] = digital_gain;
			sensor_ctl[chn].dgain_num = 1;
		break;
		case SENSOR_DOL4:
			LOG(LOG_ERR, "common subdev pointer is NULL");
		break;
		default:
			LOG(LOG_ERR, "sensor mode is error");
		break;
		}
#endif
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}
}

static void common_alloc_integration_time(uint8_t chn, uint16_t *int_time,
	uint16_t *int_time_M, uint16_t *int_time_L)
{
	int ret = 0;
	struct sensor_arg settings;

	uint32_t time_L = (uint32_t)(*int_time);
	uint32_t time_M = (uint32_t)(*int_time_M);
	uint32_t time_S = (uint32_t)(*int_time_L);

	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		settings.integration_time = &time_L;
		//settings.integration_time = &time_M;
		//settings.integration_time = &time_L;

		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_ALLOC_INTEGRATION_TIME, &settings);

		switch (sensor_ctl[chn].mode) {
		case SENSOR_LINEAR:
		case SENSOR_PWL:
			sensor_ctl[chn].line_buf[0] = time_L;
			sensor_ctl[chn].line_num = 1;
			pr_debug("linear time is %d", time_L);
		break;
		case SENSOR_DOL2:
			sensor_ctl[chn].line_buf[0] = time_L;
			sensor_ctl[chn].line_buf[1] = time_S;
			sensor_ctl[chn].line_num = 2;
		break;
		case SENSOR_DOL3:
			sensor_ctl[chn].line_buf[0] = time_L;
			sensor_ctl[chn].line_buf[1] = time_M;
			sensor_ctl[chn].line_buf[2] = time_S;
			sensor_ctl[chn].line_num = 3;
		break;
		case SENSOR_DOL4:
			LOG(LOG_ERR, "dol4 not support now.");
		break;
		default:
			LOG(LOG_ERR, "sensor mode %d is error", sensor_ctl[chn].mode);
		break;
		}
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}

	*int_time = (uint16_t)(time_L);
	*int_time_M = (uint16_t)(time_M);
	*int_time_L =(uint16_t)(time_S);
}

static void common_update(uint8_t chn, struct sensor_priv_old updata)
{
	int ret = 0;
	struct sensor_arg settings;

	if (chn >= FIRMWARE_CONTEXT_NUMBER) {
		pr_err("chn %d is invalid.\n", chn);
		return;
	}

	if (sensor_param[chn].lines_per_second == 0)
		return;

	if (common_subdev != NULL) {
		settings.port = chn;
		settings.sensor_priv = &sensor_ctl[chn];
		pr_debug("chn is %d\n", chn);
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_UPDATE, &settings);
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}
}

static void sensor_awb_update(uint8_t chn, uint32_t rgain, uint32_t grgain, uint32_t gbgain, uint32_t bgain)
{
	int ret = 0;
	struct sensor_arg settings;

	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		sensor_ctl[chn].rgain = rgain;
		sensor_ctl[chn].grgain = grgain;
		sensor_ctl[chn].gbgain = gbgain;
		sensor_ctl[chn].bgain = bgain;
		settings.sensor_priv = &sensor_ctl[chn];
		pr_debug("chn is %d", chn);
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_AWB_UPDATE, &settings);
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}
}

static void get_common_info(uint8_t chn)
{
	int ret = 0;
	struct sensor_arg settings;

	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		settings.sensor_data = &sensor_param[chn];

		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_GET_PARAM, &settings);
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}
}

static int common_init(uint8_t chn, uint8_t mode)
{
	int ret = 0;

	//todo sensor tyep info
	sensor_ctl[chn].mode = mode;
	get_common_info(chn);

	return ret;
}

static int common_hw_reset_enable(void)
{
	return 0;
}

static int common_hw_reset_disable(void)
{
	return 0;
}

static uint16_t common_get_id(uint8_t chn)
{
	return 0;
}

static void common_disable_isp(uint8_t chn)
{
	//
}

static uint32_t common_read_register(uint8_t chn, uint32_t address)
{
	int ret = 0;
	uint32_t value = 0;
	struct sensor_arg settings;

	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		settings.address = address;
		settings.r_data = &value;
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_READ, &settings);
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}

	return value;
}

static void common_write_register(uint8_t chn, uint32_t address, uint32_t data)
{
	int ret = 0;
	struct sensor_arg settings;

	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		settings.address = address;
		settings.w_data = data;
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_WRITE, &settings);
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}
}

static void common_stop_streaming(uint8_t chn)
{
	int ret = 0;
	struct sensor_arg settings;
	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_STREAM_OFF, &settings);
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}
}

static void common_start_streaming(uint8_t chn)
{
	int ret = 0;
	struct sensor_arg settings;
	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_STREAM_ON, &settings);
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}
}

void common_get_param(uint8_t chn, struct _setting_param_t *user_para)
{
	//get info
	get_common_info(chn);

	user_para->lines_per_second = sensor_param[chn].lines_per_second;
	user_para->analog_gain_max = sensor_param[chn].analog_gain_max;
	user_para->digital_gain_max = sensor_param[chn].digital_gain_max;
	user_para->exposure_time_max = sensor_param[chn].exposure_time_max;
	user_para->exposure_time_min = sensor_param[chn].exposure_time_min;
	user_para->exposure_time_long_max = sensor_param[chn].exposure_time_long_max;
	user_para->active_width = (uint16_t)(sensor_param[chn].active_width);
	user_para->active_height = (uint16_t)(sensor_param[chn].active_height);
	user_para->fps = sensor_param[chn].fps;
}

static struct sensor_operations common_ops = {
	.param_enable = 0,
	.sensor_hw_reset_enable = common_hw_reset_enable,
	.sensor_hw_reset_disable = common_hw_reset_disable,
	.sensor_alloc_analog_gain = common_alloc_analog_gain,
	.sensor_alloc_digital_gain = common_alloc_digital_gain,
	.sensor_alloc_integration_time = common_alloc_integration_time,
	.sensor_update = common_update,
	.sensor_get_id = common_get_id,
	.sensor_disable_isp = common_disable_isp,
	.read_register = common_read_register,
	.write_register = common_write_register,
	.stop_streaming = common_stop_streaming,
	.start_streaming = common_start_streaming,
	.sensor_init = common_init,
	.sesor_get_para = common_get_param,
	.sensor_awb_update = sensor_awb_update,
};

struct sensor_operations *common_ops_register(void)
{
	int ret = 0;
	struct sensor_operations *ops_ptr = NULL;

	ret = common_control_init();
	if (ret == 0) {
		ops_ptr = &common_ops;
	} else {
		ops_ptr = NULL;
	}

	return ops_ptr;
}
