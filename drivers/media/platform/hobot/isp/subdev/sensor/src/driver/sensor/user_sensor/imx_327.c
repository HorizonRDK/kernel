#include "acamera_logger.h"
#include "acamera_firmware_config.h"
#include "../sensor_i2c.h"
#include "../sensor_math.h"

#include "imx_327.h"
#include "./inc/imx_327_setting.h"

imx327_param_t imx327_param[FIRMWARE_CONTEXT_NUMBER];

//if check frame out, we could read 0x3491

static int set_gain_mode(uint8_t chn, uint8_t g_mode)
{
	int ret = 0;
	char tmp_data = 0;

	if(g_mode == 0) { //normal mode
		tmp_data = 0x21;
		ret = sensor_i2c_read(chn, IMX327_FPGC, 16, &tmp_data, 1);
		tmp_data = 0xf0;
		ret = sensor_i2c_read(chn, IMX327_FPGC_1, 16, &tmp_data, 1);
		ret = sensor_i2c_read(chn, IMX327_FPGC_2, 16, &tmp_data, 1);

	} else {//each frame gain adjustment
		tmp_data = 0x61;
		ret = sensor_i2c_read(chn, IMX327_FPGC, 16, &tmp_data, 1);
		tmp_data = 0x64;
		ret = sensor_i2c_read(chn, IMX327_FPGC_1, 16, &tmp_data, 1);
		ret = sensor_i2c_read(chn, IMX327_FPGC_2, 16, &tmp_data, 1);
	}

	return 0;
}

static int set_imx327_init(uint8_t chn)
{
	char init_d[3];
	int ret = 0;
//line
	ret = sensor_i2c_read(chn, IMX327_VAMX, 16, init_d, 3);
	imx327_param[chn].VMAX = init_d[2];
	imx327_param[chn].VMAX =(imx327_param[chn].VMAX << 8) | init_d[1];
	imx327_param[chn].VMAX =(imx327_param[chn].VMAX << 8) | init_d[0];

	imx327_param[chn].FSC_DOL2 = imx327_param[chn].VMAX * 2;
	imx327_param[chn].FSC_DOL3 = imx327_param[chn].VMAX * 4;
	imx327_param[chn].gain_max = 210;
	imx327_param[chn].exposure_time_min = 1;
	imx327_param[chn].exposure_time_max = imx327_param[chn].VMAX;

	ret = sensor_i2c_read(chn, IMX327_HAMX, 16, init_d, 2);
	imx327_param[chn].HMAX = init_d[1];
	imx327_param[chn].HMAX =(imx327_param[chn].HMAX << 8) | init_d[0];

	ret = sensor_i2c_read(chn, IMX327_RHS1, 16, init_d, 3);
	imx327_param[chn].RHS1 = init_d[2];
	imx327_param[chn].RHS1 =(imx327_param[chn].RHS1 << 8) | init_d[1];
	imx327_param[chn].RHS1 =(imx327_param[chn].RHS1 << 8) | init_d[0];

	ret = sensor_i2c_read(chn, IMX327_RHS2, 16, init_d, 3);
	imx327_param[chn].RHS2 = init_d[2];
	imx327_param[chn].RHS2 =(imx327_param[chn].RHS2 << 8) | init_d[1];
	imx327_param[chn].RHS2 =(imx327_param[chn].RHS2 << 8) | init_d[0];
//lane
	ret = sensor_i2c_read(chn, IMX327_CSI_LANE_MODE, 16, init_d, 1);
	if(init_d[0] == 3)
		imx327_param[chn].lane = 4;
	else
		imx327_param[chn].lane = 2;
//clk
	ret = sensor_i2c_read(chn, IMX327_INCKSEL6, 16, init_d, 1);
	if(init_d[0] == 0x1A)
		imx327_param[chn].clk = 37125000;
	else
		imx327_param[chn].clk = 74250000;

	imx327_param[chn].frame =(imx327_param[chn].clk * imx327_param[chn].lane)
		/(imx327_param[chn].HMAX * imx327_param[chn].VMAX);
	imx327_param[chn].lines_per_second =(imx327_param[chn].clk *
		imx327_param[chn].lane) / imx327_param[chn].VMAX;
// width & heigth
	ret = sensor_i2c_read(chn, IMX327_X_SIZE, 16, init_d, 2);
	imx327_param[chn].active_width = init_d[1];
	imx327_param[chn].active_width =(imx327_param[chn].active_width << 8)
		| init_d[0];

	ret = sensor_i2c_read(chn, IMX327_Y_SIZE, 16, init_d, 2);
	imx327_param[chn].active_height = init_d[1];
	imx327_param[chn].active_height =(imx327_param[chn].active_height << 8)
		| init_d[0];

	return ret;
}

static int set_imx327_normal_exposure(uint8_t chn, uint32_t input_exp)
{
	char exp_d[3];
	int ret = 0;

	input_exp = imx327_param[chn].VMAX - 1 - input_exp;

	exp_d[2] =(char)((input_exp >> 16) & 0x03);
	exp_d[1] =(char)((input_exp >> 8) & 0xff);
	exp_d[0] =(char)(input_exp & 0xff);
	ret = sensor_i2c_write(chn, IMX327_SHS1, 16, exp_d, 3);
	return ret;
}

static int set_imx327_dol2_exposure(uint8_t chn, uint32_t input_exp1,
	uint32_t input_exp2)
{
	char exp_d[3];
	int ret = 0;

	input_exp2 = imx327_param[chn].VMAX - 1 - input_exp2;
	exp_d[2] =(char)((input_exp2 >> 16) & 0x03);
	exp_d[1] =(char)((input_exp2 >> 8) & 0xff);
	exp_d[0] =(char)(input_exp2 & 0xff);
	ret = sensor_i2c_write(chn, IMX327_SHS2, 16, exp_d, 3);

	input_exp1 = imx327_param[chn].RHS1 - 1 - input_exp1;
	exp_d[2] =(char)((input_exp1 >> 16) & 0x03);
	exp_d[1] =(char)((input_exp1 >> 8) & 0xff);
	exp_d[0] =(char)(input_exp1 & 0xff);
	ret = sensor_i2c_write(chn, IMX327_SHS1, 16, exp_d, 3);

	return ret;
}

static int set_imx327_dol3_exposure(uint8_t chn, uint32_t input_exp1,
	uint32_t input_exp2, uint32_t input_exp3)
{
	char exp_d[3];
	int ret = 0;

	input_exp3 = imx327_param[chn].VMAX - 1 - input_exp3;
	exp_d[2] =(char)((input_exp3 >> 16) & 0x03);
	exp_d[1] =(char)((input_exp3 >> 8) & 0xff);
	exp_d[0] =(char)(input_exp3 & 0xff);
	ret = sensor_i2c_write(chn, IMX327_SHS3, 16, exp_d, 3);

	input_exp1 = imx327_param[chn].RHS1 - 1 - input_exp1;
	exp_d[2] =(char)((input_exp1 >> 16) & 0x03);
	exp_d[1] =(char)((input_exp1 >> 8) & 0xff);
	exp_d[0] =(char)(input_exp1 & 0xff);
	ret = sensor_i2c_write(chn, IMX327_SHS1, 16, exp_d, 3);

	input_exp2 = imx327_param[chn].RHS2 - 1 - input_exp2;
	exp_d[2] =(char)((input_exp2 >> 16) & 0x03);
	exp_d[1] =(char)((input_exp2 >> 8) & 0xff);
	exp_d[0] =(char)(input_exp2 & 0xff);
	ret = sensor_i2c_write(chn, IMX327_SHS2, 16, exp_d, 3);

	return ret;
}

static int set_normal_gain(uint8_t chn, uint32_t input_gain)
{
	char gain_d[2];
	int ret = 0;

	gain_d[0] =(char)(input_gain & 0xff);
	ret = sensor_i2c_write(chn, IMX327_GAIN, 16, gain_d, 1);

	return ret;
}

static int set_dol2_gain(uint8_t chn, uint32_t gain1, uint32_t gain2)
{
	char gain_d[2];
	int ret = 0;

	gain_d[0] =(char)(gain1 & 0xff);
	ret = sensor_i2c_write(chn, IMX327_GAIN1, 16, gain_d, 1);

	gain_d[0] =(char)(gain2 & 0xff);
	ret = sensor_i2c_write(chn, IMX327_GAIN1, 16, gain_d, 1);

	return ret;
}

static int set_dol3_gain(uint8_t chn, uint32_t gain1, uint32_t gain2)
{
	char gain_d[2];
	int ret = 0;

	gain_d[0] =(char)(gain1 & 0xff);
	ret = sensor_i2c_write(chn, IMX327_GAIN1, 16, gain_d, 1);

	gain_d[0] =(char)(gain2 & 0xff);
	ret = sensor_i2c_write(chn, IMX327_GAIN1, 16, gain_d, 1);

	return ret;
}

static int set327_ex_gain_control(uint8_t chn, uint32_t expo_L,
	uint32_t expo_M, uint32_t expo_S, uint32_t gain)
{
	int ret = 0;
	uint32_t a_gain = 0;

	a_gain = sensor_log10(gain);//ux.8
	a_gain =(uint32_t)(((a_gain * 200) / 3) >> 8);

	switch(imx327_param[chn].imx327_mode_save) {
	case NORMAL_M:
		set_normal_gain(chn, a_gain);
		set_imx327_normal_exposure(chn, expo_S);
		break;
	case DOL2_M:
		set_normal_gain(chn, a_gain);
		set_imx327_dol2_exposure(chn, expo_S, expo_M);//
		break;
	case DOL3_M:
		set_normal_gain(chn, a_gain);
		set_imx327_dol3_exposure(chn, expo_S, expo_M, expo_L);//
		break;
	default:
		LOG(LOG_ERR, "mode is err !");
		ret = -1;
		break;
	}

	return ret;
}

static int imx327_init(uint8_t chn, uint8_t mode)
{
	int ret = 0;
	uint32_t tmp_c = 0;
	uint32_t tmp_size = 0;
	uint16_t tmp_addr;
	char tmp_data;

	switch(mode) {
	case 0:
		tmp_size = sizeof(imx327_raw12_normal_setting)/sizeof(uint16_t);
		while(tmp_c < tmp_size) {
			tmp_addr = imx327_raw12_normal_setting[tmp_c++];
			tmp_data =(char)(imx327_raw12_normal_setting[tmp_c++] & 0xff);
			ret = sensor_i2c_write(chn, tmp_addr, 16, &tmp_data, 1);
			LOG(LOG_DEBUG, "tmp_addr %x, data %x", tmp_addr, tmp_data);
		}
		imx327_param[chn].imx327_mode_save = NORMAL_M;
		LOG(LOG_CRIT, "imx327 raw12 normal init success ", __func__, __LINE__);
		break;
	case 1:
		tmp_size = sizeof(imx327_raw12_dol2_setting)/sizeof(uint16_t);
		while(tmp_c < tmp_size) {
			tmp_addr = imx327_raw12_dol2_setting[tmp_c++];
			tmp_data =(char)(imx327_raw12_dol2_setting[tmp_c++] & 0xff);
			ret = sensor_i2c_write(chn, tmp_addr, 16, &tmp_data, 1);
		}
		imx327_param[chn].imx327_mode_save = DOL2_M;
		LOG(LOG_CRIT, "imx327 raw12 dol2 init success", __func__, __LINE__);
		break;
	default:
		LOG(LOG_CRIT, "init failed", __func__, __LINE__);
		ret = -1;
		break;
	}
	set_imx327_init(chn);

	return ret;
}

static int imx327_hw_reset_enable(void)
{
	return 0;
}

static int imx327_hw_reset_disable(void)
{
	return 0;
}

static int32_t imx327_alloc_analog_gain(uint8_t chn, int32_t gain)
{
	LOG(LOG_DEBUG, "gain %d ", gain);
	int32_t analog_gain = 0;

	analog_gain = gain;
	return analog_gain;
}

static int32_t imx327_alloc_digital_gain(uint8_t chn, int32_t gain)
{
	int32_t digital_gain = 0;

	digital_gain = gain;
	return digital_gain;
}

static void imx327_alloc_integration_time(uint8_t chn, uint16_t *int_time,
	uint16_t *int_time_M, uint16_t *int_time_L)
{
}

static void imx327_update(uint8_t chn, struct sensor_priv updata)
{
	set327_ex_gain_control(chn, updata.int_time_L, updata.int_time_M,
		updata.int_time, updata.analog_gain);
}

static uint16_t imx327_get_id(uint8_t chn)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
	return 0;
}

static void imx327_disable_isp(uint8_t chn)
{
	LOG(LOG_DEBUG, "[%s -- %d ]", __func__, __LINE__);
}

static uint32_t imx327_read_register(uint8_t chn, uint32_t address)
{
	char buf[1];

	sensor_i2c_read(chn, (uint16_t)(address), 16, buf, 1);
	return(uint32_t)(buf[0]);
}

static void imx327_write_register(uint8_t chn, uint32_t address, uint32_t data)
{
	char buf =(char)(data & 0xff);
	sensor_i2c_write(chn, (uint16_t)(address), 16, &buf, 1);
}

static void imx327_stop_streaming(uint8_t chn)
{
	//0x3000, 0x01
	char buf = 0x01;
	LOG(LOG_INFO, "[%s -- %d ]", __func__, __LINE__);
	sensor_i2c_write(chn, 0x3000, 16, &buf, 1);
}

static void imx327_start_streaming(uint8_t chn)
{
	//0x3000, 0x00
	char buf = 0x00;
	LOG(LOG_INFO, "[%s -- %d ]", __func__, __LINE__);
	sensor_i2c_write(chn, 0x3002, 16, &buf, 1);
	sensor_i2c_write(chn, 0x3000, 16, &buf, 1);
}

void imx327_get_para(uint8_t chn, struct _setting_param_t user_para)
{
	user_para.lines_per_second = imx327_param[chn].lines_per_second;
	user_para.analog_gain_max = imx327_param[chn].gain_max;
	user_para.digital_gain_max = imx327_param[chn].gain_max;
	user_para.exposure_time_max = imx327_param[chn].VMAX - 2;
	user_para.exposure_time_min = 1;
	user_para.exposure_time_long_max = imx327_param[chn].VMAX - 2;
	user_para.active_width = imx327_param[chn].active_width;
	user_para.active_height = imx327_param[chn].active_height;
}

static struct sensor_operations imx327_ops = {
	.sensor_hw_reset_enable = imx327_hw_reset_enable,
	.sensor_hw_reset_disable = imx327_hw_reset_disable,
	.sensor_alloc_analog_gain = imx327_alloc_analog_gain,
	.sensor_alloc_digital_gain = imx327_alloc_digital_gain,
	.sensor_alloc_integration_time = imx327_alloc_integration_time,
	.sensor_update = imx327_update,
	.sensor_get_id = imx327_get_id,
	.sensor_disable_isp = imx327_disable_isp,
	.read_register = imx327_read_register,
	.write_register = imx327_write_register,
	.stop_streaming = imx327_stop_streaming,
	.start_streaming = imx327_start_streaming,
	.sensor_init = imx327_init,
	.sesor_get_para = imx327_get_para,
};

struct sensor_operations *imx327_ops_register(void)
{
	return &imx327_ops;
}
