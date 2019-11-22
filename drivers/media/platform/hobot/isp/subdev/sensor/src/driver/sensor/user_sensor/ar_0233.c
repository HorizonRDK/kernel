#include "acamera_logger.h"
#include "acamera_firmware_config.h"
#include "../sensor_i2c.h"
#include "../sensor_math.h"
#include "ar_0233.h"

ar0233_param_t ar0233_param[FIRMWARE_CONTEXT_NUMBER];

int set_ex_line_pwl_0233(uint8_t chn, uint32_t line)
{
	int ret = 0;
	char buf[2];
	buf[1] = (char)(line & 0xff);
	buf[0] = (char)((line >> 8) & 0xff);

	ret = sensor_i2c_write(chn, 0x3012, 16, buf, 2);
	return 0;
}

int set_ex_gain_pwl_0233(uint8_t chn, uint32_t gain)
{
	uint32_t gain_tmp;
	int ret = 0;
	char buf[2];

	gain = gain >> 1;
	if(gain < 128)
		gain = 128; //128==0x80

	if(gain > 0x7fe)
		gain = 0x7fe;

	buf[0] = (char)((gain >> 8) & 0xff);
	buf[1] = (char)(gain & 0xff);
	ret = sensor_i2c_write(chn, 0x305e, 16, buf, 2);

	return 0;
}

//gain_setting_0  lcg   //gain_setting_1 hcg
int set_ex_gain_control_0233(uint8_t chn, uint32_t exposure_setting,
	uint32_t gain_setting_0, uint16_t gain_setting_1)
{
	int ret = 0;
	char buf[2];

	buf[0] = 0x00;
	buf[1] = 0x01;
	ret = sensor_i2c_write(chn, 0x3022, 16, buf, 2);
	set_ex_line_pwl_0233(chn, exposure_setting);
	set_ex_gain_pwl_0233(chn, gain_setting_0);
	buf[0] = 0x00;
	buf[1] = 0x00;
	ret = sensor_i2c_write(chn, 0x3022, 16, buf, 2);
	return 0;
}

//TODO ar0233 analog_gain

static int ar0233_set_a_gain(uint8_t chn, uint32_t a_gain)
{
	int ret = 0;
	uint32_t temp = 0;
	char buf[2];

	if (a_gain <= 256) {// coarse 0
		a_gain = 256;
		buf[0] = 0x00;
		buf[1] = 0x11;
		ret = sensor_i2c_write(chn, 0x3366, 16, buf, 2);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = sensor_i2c_write(chn, 0x336a, 16, buf, 2);
	} else if (a_gain < 512) { //coarse 1 1x
		buf[0] = 0x00;
		buf[1] = 0x11;
		ret = sensor_i2c_write(chn, 0x3366, 16, buf, 2);
		temp = (uint32_t)((32*(a_gain -256))/a_gain);
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[1] |= ((temp & 0x0f) | ((temp << 4) & 0xf0));
		ret = sensor_i2c_write(chn, 0x336a, 16, buf, 2);
	} else if (a_gain < 1024) { //coarse 2 2x
		buf[0] = 0x00;
		buf[1] = 0x22;
		ret = sensor_i2c_write(chn, 0x3366, 16, buf, 2);
		temp = (uint32_t)(((16 * a_gain - 16 * 2 * 256)/a_gain) * 2);
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[1] |= ((temp & 0x0f) | ((temp << 4) & 0xf0));
		ret = sensor_i2c_write(chn, 0x336a, 16, buf, 2);
	} else if (a_gain < 2048) { //coarse 3 4x
		buf[0] = 0x00;
		buf[1] = 0x33;
		ret = sensor_i2c_write(chn, 0x3366, 16, buf, 2);
		temp = (uint32_t)(((16 * a_gain - 16 * 4 * 256)/a_gain) * 2);
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[1] |= ((temp & 0x0f) | ((temp << 4) & 0xf0));
		ret = sensor_i2c_write(chn, 0x336a, 16, buf, 2);
	} else if (a_gain < 1024) { //coarse 4 8x
		buf[0] = 0x00;
		buf[1] = 0x44;
		ret = sensor_i2c_write(chn, 0x3366, 16, buf, 2);
		temp = (uint32_t)(((16 * a_gain - 16 * 8 * 256)/a_gain) * 2);
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[1] |= ((temp & 0x0f) | ((temp << 4) & 0xf0));
		ret = sensor_i2c_write(chn, 0x336a, 16, buf, 2);
	} else if (a_gain < 2048) { //coarse 5 16x
		buf[0] = 0x00;
		buf[1] = 0x55;
		ret = sensor_i2c_write(chn, 0x3366, 16, buf, 2);
		temp = (uint32_t)(((16 * a_gain - 16 * 16 * 256)/a_gain) * 2);
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[1] |= ((temp & 0x0f) | ((temp << 4) & 0xf0));
		ret = sensor_i2c_write(chn, 0x336a, 16, buf, 2);
	} else if (a_gain < 16384) { //coarse 6 32x
		buf[0] = 0x00;
		buf[1] = 0x66;
		ret = sensor_i2c_write(chn, 0x3366, 16, buf, 2);
		temp = (uint32_t)(((8 * a_gain - 256 * 256)/a_gain) * 4);
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[1] |= ((temp & 0x0f) | ((temp << 4) & 0xf0));
		ret = sensor_i2c_write(chn, 0x336a, 16, buf, 2);
	} else if (a_gain < 21350) { //coarse 7  64x
		buf[0] = 0x00;
		buf[1] = 0x77;
		ret = sensor_i2c_write(chn, 0x3366, 16, buf, 2);
		temp = (uint32_t)(((4 * a_gain - 256 * 256)/a_gain) * 8);
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[1] |= ((temp & 0x0f) | ((temp << 4) & 0xf0));
		ret = sensor_i2c_write(chn, 0x336a, 16, buf, 2);
	} else {
		buf[0] = 0x00;
		buf[1] = 0x77;
		ret = sensor_i2c_write(chn, 0x3366, 16, buf, 2);
		buf[0] = 0x00;
		buf[1] = 0xff;
		ret = sensor_i2c_write(chn, 0x336a, 16, buf, 2);
	}
}

static int ar0233_init(uint8_t chn, uint8_t mode)
{
	int ret = 0;
	uint32_t tmp_c = 0;
	uint32_t tmp_size = 0;
	uint16_t tmp_addr;
	char tmp_data;

	LOG(LOG_INFO, "mode = %d", mode);

	switch (mode) {
	case 0://pwl
#if 0
		tmp_size = sizeof(imx390_pwl12_setting)/sizeof(uint16_t);
		while (tmp_c < tmp_size) {
			tmp_addr = imx390_pwl12_setting[tmp_c++];
			tmp_data = (char)(imx390_pwl12_setting[tmp_c++] & 0xff);
			sensor_i2c_write(tmp_addr, 16, &tmp_data, 1);
		}
#endif
		LOG(LOG_NOTICE, "0233 pwl init success ");
		ar0233_param[chn].lines_per_second = 15858;
		ar0233_param[chn].exposure_time_max = 4000;
		ar0233_param[chn].exposure_time_min = 32;
		ar0233_param[chn].exposure_time_long_max = 4000;
		ar0233_param[chn].gain_max = 128 * 8192;

		break;
	case 1://lcg
#if 0
		tmp_size = sizeof(imx390_lcg_setting)/sizeof(uint16_t);
		while (tmp_c < tmp_size) {
			tmp_addr = imx390_lcg_setting[tmp_c++];
			tmp_data = (char)(imx390_lcg_setting[tmp_c++] & 0xff);
			sensor_i2c_write(tmp_addr, 16, &tmp_data, 1);
		}
#endif
		LOG(LOG_NOTICE, "0233 lcg init success");
		break;
	case 2:
		LOG(LOG_NOTICE, "0233 raw12 hcg init success");
		break;
	default:
		LOG(LOG_NOTICE, "init failed");
		ret = -1;
		break;
	}

	return ret;
}

static int ar0233_hw_reset_enable(void)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
	return 0;
}

static int ar0233_hw_reset_disable(void)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
	return 0;
}

static int32_t ar0233_alloc_analog_gain(uint8_t chn, int32_t gain)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
	uint32_t analog_gain = 0;

	analog_gain = gain;
	return analog_gain;
}

static int32_t ar0233_alloc_digital_gain(uint8_t chn, int32_t gain)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
	return 0;
}

static void ar0233_alloc_integration_time(uint8_t chn, uint16_t *int_time,
	uint16_t *int_time_M, uint16_t *int_time_L)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
}

static void ar0233_update(uint8_t chn, struct sensor_priv updata)
{
	uint32_t data = 0;
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
/*
	struct sensor_priv {
		int32_t analog_gain;
		int32_t digital_gain;
		uint16_t int_time; 
		uint16_t int_time_M;
		uint16_t int_time_L; 
	};
*/
	data = sensor_date(updata.analog_gain);
	LOG(LOG_INFO, "int_time %d, analog_gain %d", updata.int_time, data);
	set_ex_gain_control_0233(chn, updata.int_time, data, 0);
	//set390_ex_gain_control(updata.int_time, updata.analog_gain, 385);
}

static uint16_t ar0233_get_id(uint8_t chn)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
	return 0;
}

static void ar0233_disable_isp(uint8_t chn)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
}

static uint32_t ar0233_read_register(uint8_t chn, uint32_t address)
{
	char buf[1];

	sensor_i2c_read(chn, (uint16_t)(address), 16, buf, 1);
	return (uint32_t)(buf[0]);
}

static void ar0233_write_register(uint8_t chn, uint32_t address, uint32_t data)
{
	char buf = (char)(data & 0xff);
	sensor_i2c_write(chn, (uint16_t)(address), 16, &buf, 1);
}

static void ar0233_start_streaming(uint8_t chn)
{
	//0x3000, 0x01
	char buf[2];
	LOG(LOG_INFO, "[%s -- %d ]", __func__, __LINE__);
	buf[0] = 0x00;
	buf[1] = 0x5c;
	sensor_i2c_write(chn, 0x301a, 16, buf, 2);
}

static void ar0233_stop_streaming(uint8_t chn)
{
	//0x3002, 0x00	// Master Mode
	//0x3000, 0x00	// Run
	char buf[2];
	LOG(LOG_INFO, "[%s -- %d ]", __func__, __LINE__);
	buf[0] = 0x00;
	buf[1] = 0x58;
	sensor_i2c_write(chn, 0x301a, 16, buf, 2);
}

void ar0233_get_para(uint8_t chn, struct _setting_param_t *user_para)
{
        user_para->lines_per_second = ar0233_param[chn].lines_per_second;
        user_para->analog_gain_max = ar0233_param[chn].gain_max;
        user_para->digital_gain_max = ar0233_param[chn].gain_max;
        user_para->exposure_time_max = ar0233_param[chn].exposure_time_max;
        user_para->exposure_time_min = ar0233_param[chn].exposure_time_min;
        user_para->exposure_time_long_max =
		ar0233_param[chn].exposure_time_long_max;
        user_para->active_width = ar0233_param[chn].active_width;
        user_para->active_height = ar0233_param[chn].active_height;
}


static struct sensor_operations ar0233_ops = {
	.sensor_hw_reset_enable = ar0233_hw_reset_enable,
	.sensor_hw_reset_disable = ar0233_hw_reset_disable,
	.sensor_alloc_analog_gain = ar0233_alloc_analog_gain,
	.sensor_alloc_digital_gain = ar0233_alloc_digital_gain,
	.sensor_alloc_integration_time = ar0233_alloc_integration_time,
	.sensor_update = ar0233_update,
	.sensor_get_id = ar0233_get_id,
	.sensor_disable_isp = ar0233_disable_isp,
	.read_register = ar0233_read_register,
	.write_register = ar0233_write_register,
	.stop_streaming = ar0233_stop_streaming,
	.start_streaming = ar0233_start_streaming,
	.sensor_init = ar0233_init,
	.sesor_get_para = ar0233_get_para,
};

struct sensor_operations *ar0233_ops_register(void)
{
	return &ar0233_ops;
}

