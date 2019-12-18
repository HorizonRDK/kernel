#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>

#include "acamera_logger.h"
#include "../sensor_i2c.h"
#include "../sensor_math.h"
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

static void common_control_deinit(void)
{
	common_subdev = NULL;
}

static int32_t common_alloc_analog_gain(uint8_t chn, int32_t gain)
{
	int ret = 0;
	int analog_gain = gain;
	struct sensor_arg settings;

	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		settings.a_gain = &analog_gain;
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_ALLOC_DIGITAL_GAIN, &settings);

		switch (sensor_ctl[chn].mode) {
		case SENSOR_LINEAR:
		case SENSOR_PWL:
			sensor_ctl[chn].gain_buf[0] = analog_gain;
			sensor_ctl[chn].gain_num = 1;
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
	} else {
		LOG(LOG_ERR, "common subdev pointer is NULL");
	}

	return analog_gain;
}

static int32_t common_alloc_digital_gain(uint8_t chn, int32_t gain)
{
	int ret = 0;
	int digital_gain = gain;
	struct sensor_arg settings;

	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		settings.d_gain = &digital_gain;
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_ALLOC_DIGITAL_GAIN, &settings);
#if 0
		switch (sensor_ctl[chn].mode) {
		case SENSOR_LINEAR:
		case SENSOR_PWL:
			sensor_ctl[chn].gain_buf[0] = digital_gain;
			sensor_ctl[chn].gain_num = 1;
		break;
		case SENSOR_DOL2:
			sensor_ctl[chn].gain_buf[0] = digital_gain;
			//sensor_ctl[chn].gain_buf[1] = digital_gain;
			sensor_ctl[chn].gain_num = 1;
		break;
		case SENSOR_DOL3:
			sensor_ctl[chn].gain_buf[0] = digital_gain;
			//sensor_ctl[chn].gain_buf[1] = digital_gain;
			//sensor_ctl[chn].gain_buf[2] = digital_gain;
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

	return digital_gain;
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
			LOG(LOG_ERR, "common subdev pointer is NULL");
		break;
		default:
			LOG(LOG_ERR, "sensor mode is error");
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

	if (common_subdev != NULL && chn < FIRMWARE_CONTEXT_NUMBER) {
		settings.port = chn;
		settings.sensor_priv = &sensor_ctl[chn];
		// Initial local parameters
		ret = v4l2_subdev_call(common_subdev, core, ioctl,
			SENSOR_UPDATE, &settings);
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

static void common_switch_mode(uint8_t chn, uint32_t type)
{
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
	return 0;
}

static void common_write_register(uint8_t chn, uint32_t address, uint32_t data)
{
	//
}

static void common_stop_streaming(uint8_t chn)
{
	//
}

static void common_start_streaming(uint8_t chn)
{
	//0x3000, 0x00
}

void common_get_param(uint8_t chn, struct _setting_param_t *user_para)
{
	//get info
	get_common_info(chn);

	user_para->lines_per_second = sensor_param[chn].pixels_per_line;
	user_para->analog_gain_max = sensor_param[chn].analog_gain_max;
	user_para->digital_gain_max = sensor_param[chn].digital_gain_max;
	user_para->exposure_time_max = sensor_param[chn].exposure_time_max;
	user_para->exposure_time_min = sensor_param[chn].exposure_time_min;
	user_para->exposure_time_long_max = sensor_param[chn].exposure_time_long_max;
	user_para->active_width = sensor_param[chn].active_width;
	user_para->active_height = sensor_param[chn].active_height;
}

static struct sensor_operations common_ops = {
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
