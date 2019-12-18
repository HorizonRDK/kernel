#include "acamera_logger.h"
#include "acamera_firmware_config.h"
#include "../sensor_i2c.h"
#include "../sensor_math.h"

#include "os8a10.h"
#include "./inc/os8a10_setting.h"

os8a10_param_t os8a10_param[FIRMWARE_CONTEXT_NUMBER];

//if check frame out, we could read 0x3491
static int set_os8a10_init(uint8_t chn)
{
	char init_d[3];
	int ret = 0;
//line
	sensor_i2c_read(chn, 0x380e, 16, init_d, 2);
	os8a10_param[chn].VMAX = init_d[0];
	os8a10_param[chn].VMAX = (os8a10_param[chn].VMAX << 8) | init_d[0];

	sensor_i2c_read(chn, 0x380c, 16, init_d, 2);
	os8a10_param[chn].HMAX = init_d[0];
	os8a10_param[chn].HMAX = (os8a10_param[chn].VMAX << 8) | init_d[0];

	return ret;
}

static int set_os8a10_normal_exposure(uint8_t chn, uint32_t input_exp)
{
	char exp_d[3];
	int ret = 0;

	//line must >= 8
	if (input_exp < 8) {
		input_exp = 8;
	}

	exp_d[0] =(char)((input_exp >> 8) & 0xff);
	exp_d[1] =(char)(input_exp & 0xff);
	ret = sensor_i2c_write(chn, 0x3501, 16, exp_d, 3);
	return ret;
}

static int set_os8a10_dol2_exposure(uint8_t chn,
	uint32_t input_exp1, uint32_t input_exp2)
{
	char exp_d[3];
	int ret = 0;

	//line must >= 8
	if (input_exp1 < 8) {
		input_exp1 = 8;
	}

	exp_d[0] =(char)((input_exp1 >> 8) & 0xff);
	exp_d[1] =(char)(input_exp1 & 0xff);
	ret = sensor_i2c_write(chn, 0x3501, 16, exp_d, 2);

	//line must >= 8
	if (input_exp2 < 8) {
		input_exp2 = 8;
	}

	exp_d[0] =(char)((input_exp2 >> 8) & 0xff);
	exp_d[1] =(char)(input_exp2 & 0xff);
	ret = sensor_i2c_write(chn, 0x3511, 16, exp_d, 2);

	return ret;
}

static int set_os8a10_normal_gain(uint8_t chn, uint32_t input_gain)
{// long gain
	char gain_d[2];
	uint32_t a_gain = 0;
	uint32_t d_gain = 0;
	int ret = 0;

	//gain must >=1
	if (input_gain < 256)
		input_gain = 256;

	if (input_gain <= 3968) { // a_gain  max 15.5
		a_gain = (input_gain >> 1);
		gain_d[0] = (char)((a_gain >> 8) && 0xff);
		gain_d[1] = (char)(a_gain && 0xff);
		ret = sensor_i2c_write(chn, 0x3508, 16, gain_d, 2);
	} else { //d_gain
		gain_d[0] = 0x07;
		gain_d[1] = 0xc0;
		ret = sensor_i2c_write(chn, 0x3508, 16, gain_d, 2);
		d_gain = (uint32_t)((input_gain << 3) / 31);
		gain_d[0] = (char)((d_gain >> 8) && 0xff);
		gain_d[1] = (char)(d_gain && 0xff);
		ret = sensor_i2c_write(chn, 0x350a, 16, gain_d, 2);
	}

	return ret;
}

static int set_os8a10_hdr_gain(uint8_t chn, uint32_t input_gain)
{//short gain
	char gain_d[2];
	uint32_t a_gain = 0;
	uint32_t d_gain = 0;
	int ret = 0;

	//gain must >=1
	if (input_gain < 256)
		input_gain = 256;

	if (input_gain <= 3968) { // a_gain  max 15.5
		a_gain = (input_gain >> 1);
		gain_d[0] = (char)((a_gain >> 8) && 0xff);
		gain_d[1] = (char)(a_gain && 0xff);
		ret = sensor_i2c_write(chn, 0x350c, 16, gain_d, 2);
	} else { //d_gain
		gain_d[0] = 0x07;
		gain_d[1] = 0xc0;
		ret = sensor_i2c_write(chn, 0x3508, 16, gain_d, 2);
		d_gain = (uint32_t)((input_gain << 3) / 31);
		gain_d[0] = (char)((d_gain >> 8) && 0xff);
		gain_d[1] = (char)(d_gain && 0xff);
		ret = sensor_i2c_write(chn, 0x350e, 16, gain_d, 2);
	}

	return ret;
}

static int set_os8a10_ex_gain_control(uint8_t chn, uint32_t expo_L,
	uint32_t expo_M, uint32_t expo_S, uint32_t gain)
{
	int ret = 0;
	uint32_t a_gain = 0;

	a_gain = sensor_date(gain);;
	//a_gain = sensor_log10(gain);//ux.8
	//a_gain =(uint32_t)(((a_gain * 200) / 3) >> 8);

	switch(os8a10_param[chn].os8a10_mode_save) {
	case OS8A10_NORMAL_M:
		if (expo_S > os8a10_param[chn].VMAX - 8) {
			expo_S = os8a10_param[chn].VMAX - 8;
		}
		set_os8a10_normal_gain(chn, a_gain);
		set_os8a10_normal_exposure(chn, expo_S);
		break;
	case OS8A10_DOL2_M:
		if ((expo_S + expo_L) > os8a10_param[chn].VMAX - 4) {
			expo_L = os8a10_param[chn].VMAX - 4 - expo_S;
		}
		set_os8a10_normal_gain(chn, a_gain);
		set_os8a10_hdr_gain(chn, a_gain);
		set_os8a10_dol2_exposure(chn, expo_S, expo_L);//
		break;
	default:
		LOG(LOG_ERR, "mode is err !");
		ret = -1;
		break;
	}

	return ret;
}

static int os8a10_init(uint8_t chn, uint8_t mode)
{
	int ret = 0;
	uint32_t tmp_c = 0;
	uint32_t tmp_size = 0;
	uint16_t tmp_addr;
	char tmp_data;

	switch(mode) {
	case 0:
		tmp_size = sizeof(os8a10_raw10_normal_setting)/sizeof(uint16_t);
		while(tmp_c < tmp_size) {
			tmp_addr = os8a10_raw10_normal_setting[tmp_c++];
			tmp_data =(char)(os8a10_raw10_normal_setting[tmp_c++] & 0xff);
			ret = sensor_i2c_write(chn, tmp_addr, 16, &tmp_data, 1);
			LOG(LOG_DEBUG, "tmp_addr %x, data %x", tmp_addr, tmp_data);
		}
		os8a10_param[chn].os8a10_mode_save = OS8A10_NORMAL_M;
		os8a10_param[chn].lines_per_second = 10074;
		os8a10_param[chn].exposure_time_max = 3880;
		os8a10_param[chn].exposure_time_min = 1;
		os8a10_param[chn].exposure_time_long_max = 10000;

		//gain control init
		tmp_data = 0x88;
		ret = sensor_i2c_write(chn, 0x3503, 16, &tmp_data, 1);
		LOG(LOG_CRIT, "os8a10 raw12 normal init success ", __func__, __LINE__);
		break;
	case 1:
		tmp_size = sizeof(os8a10_raw10_hdr_setting)/sizeof(uint16_t);
		while(tmp_c < tmp_size) {
			tmp_addr = os8a10_raw10_hdr_setting[tmp_c++];
			tmp_data =(char)(os8a10_raw10_hdr_setting[tmp_c++] & 0xff);
			ret = sensor_i2c_write(chn, tmp_addr, 16, &tmp_data, 1);
		}
		os8a10_param[chn].os8a10_mode_save = OS8A10_DOL2_M;
		os8a10_param[chn].lines_per_second = 10074;
		os8a10_param[chn].exposure_time_max = 3880;
		os8a10_param[chn].exposure_time_min = 1;
		os8a10_param[chn].exposure_time_long_max = 10000;
		LOG(LOG_CRIT, "os8a10 raw12 dol2 init success", __func__, __LINE__);
		break;
	default:
		LOG(LOG_CRIT, "init failed", __func__, __LINE__);
		ret = -1;
		break;
	}
	set_os8a10_init(chn);

	return ret;
}

static int os8a10_hw_reset_enable(void)
{
	return 0;
}

static int os8a10_hw_reset_disable(void)
{
	return 0;
}

static int32_t os8a10_alloc_analog_gain(uint8_t chn, int32_t gain)
{
	LOG(LOG_DEBUG, "gain %d ", gain);
	int32_t analog_gain = 0;

	analog_gain = gain;
	return analog_gain;
}

static int32_t os8a10_alloc_digital_gain(uint8_t chn, int32_t gain)
{
	int32_t digital_gain = 0;

	digital_gain = gain;
	return digital_gain;
}

static void os8a10_alloc_integration_time(uint8_t chn, uint16_t *int_time,
	uint16_t *int_time_M, uint16_t *int_time_L)
{
}

static void os8a10_update(uint8_t chn, struct sensor_priv_old updata)
{
	set_os8a10_ex_gain_control(chn, updata.int_time_L, updata.int_time_M,
		updata.int_time, updata.analog_gain);
}

static uint16_t os8a10_get_id(uint8_t chn)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
	return 0;
}

static void os8a10_disable_isp(uint8_t chn)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
}

static uint32_t os8a10_read_register(uint8_t chn, uint32_t address)
{
	char buf[1];

	sensor_i2c_read(chn, (uint16_t)(address), 16, buf, 1);
	return(uint32_t)(buf[0]);
}

static void os8a10_write_register(uint8_t chn, uint32_t address, uint32_t data)
{
	char buf =(char)(data & 0xff);
	sensor_i2c_write(chn, (uint16_t)(address), 16, &buf, 1);
}

static void os8a10_stop_streaming(uint8_t chn)
{
	//0x3000, 0x01
	char buf = 0x00;
	LOG(LOG_INFO, "[%s -- %d ]", __func__, __LINE__);
	sensor_i2c_write(chn, 0x0100, 16, &buf, 1);
}

static void os8a10_start_streaming(uint8_t chn)
{
	//0x3000, 0x00
	char buf = 0x01;
	LOG(LOG_INFO, "[%s -- %d ]", __func__, __LINE__);
	sensor_i2c_write(chn, 0x0100, 16, &buf, 1);
}

static void os8a10_get_para(uint8_t chn, struct _setting_param_t *user_para)
{
	user_para->lines_per_second = os8a10_param[chn].lines_per_second;
	user_para->analog_gain_max = os8a10_param[chn].gain_max;
	user_para->digital_gain_max = os8a10_param[chn].gain_max;
	user_para->exposure_time_max = os8a10_param[chn].exposure_time_max;
	user_para->exposure_time_min = os8a10_param[chn].exposure_time_min;
	user_para->exposure_time_long_max = os8a10_param[chn].exposure_time_long_max;
	user_para->active_width = os8a10_param[chn].active_width;
	user_para->active_height = os8a10_param[chn].active_height;
}

static struct sensor_operations os8a10_ops = {
	.sensor_hw_reset_enable = os8a10_hw_reset_enable,
	.sensor_hw_reset_disable = os8a10_hw_reset_disable,
	.sensor_alloc_analog_gain = os8a10_alloc_analog_gain,
	.sensor_alloc_digital_gain = os8a10_alloc_digital_gain,
	.sensor_alloc_integration_time = os8a10_alloc_integration_time,
	.sensor_update = os8a10_update,
	.sensor_get_id = os8a10_get_id,
	.sensor_disable_isp = os8a10_disable_isp,
	.read_register = os8a10_read_register,
	.write_register = os8a10_write_register,
	.stop_streaming = os8a10_stop_streaming,
	.start_streaming = os8a10_start_streaming,
	.sensor_init = os8a10_init,
	.sesor_get_para = os8a10_get_para,
};

struct sensor_operations *os8a10_ops_register(void)
{
	return &os8a10_ops;
}
