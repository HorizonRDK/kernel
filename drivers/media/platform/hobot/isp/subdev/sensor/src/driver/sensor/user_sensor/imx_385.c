#include "acamera_logger.h"
#include "acamera_firmware_config.h"
#include "../sensor_i2c.h"
#include "../sensor_math.h"

#include "imx_385.h"
#include "./inc/imx_385_setting.h"

imx385_param_t imx385_param[FIRMWARE_CONTEXT_NUMBER];


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#endif


//TODO 
//if check frame out, we could read 0x3491


static int set_imx385_init(uint8_t chn)
{
	char init_d[3];
	int ret = 0;
//line
	ret = sensor_i2c_read(chn, IMX385_VAMX, 16, init_d, 3);
	imx385_param[chn].VMAX = init_d[2];
	imx385_param[chn].VMAX = (imx385_param[chn].VMAX << 8) | init_d[1];
	imx385_param[chn].VMAX = (imx385_param[chn].VMAX << 8) | init_d[0];

	imx385_param[chn].FSC_DOL2 = imx385_param[chn].VMAX * 2;
	imx385_param[chn].gain_max = 210;
	
	ret = sensor_i2c_read(chn, IMX385_HAMX, 16, init_d, 2);
	imx385_param[chn].HMAX = init_d[1];
	imx385_param[chn].HMAX = (imx385_param[chn].HMAX << 8) | init_d[0];
	
	ret = sensor_i2c_read(chn, IMX385_RHS1, 16, init_d, 3);	
	imx385_param[chn].RHS1 = init_d[2];
	imx385_param[chn].RHS1 = (imx385_param[chn].RHS1 << 8) | init_d[1];
	imx385_param[chn].RHS1 = (imx385_param[chn].RHS1 << 8) | init_d[0];
#if 0
//lane	
	ret = sensor_i2c_read(chn, IMX385_CSI_LANE_MODE, 16, init_d, 1);
	if (init_d[0] == 3)
		imx385_param[chn].lane = 4;
	else	
		imx385_param[chn].lane = 2;		
//clk
	ret = sensor_i2c_read(chn, IMX385_INCKFREQ1, 16, init_d, 1);
	if (init_d[0] == 0x40)
		imx385_param[chn].clk = 37125000;
	else	 
		imx385_param[chn].clk = 74250000;		

	imx385_param[chn].frame = (imx385_param[chn].clk * imx385_param[chn].lane) / (imx385_param[chn].HMAX * imx385_param[chn].VMAX);
	imx385_param[chn].lines_per_second = (imx385_param[chn].clk * imx385_param[chn].lane) / imx385_param[chn].VMAX;
#endif
// width & heigth
	imx385_param[chn].active_width = 1936;
	imx385_param[chn].active_width = 1097;
 
	return ret;
} 

static int set_imx385_normal_exposure(uint8_t chn, uint32_t input_exp)
{
	char exp_d[3];
	int ret = 0;
	
	input_exp = imx385_param[chn].VMAX - 1 - input_exp;
	
	exp_d[2] = (char)((input_exp >> 16) & 0x03);
	exp_d[1] = (char)((input_exp >> 8) & 0xff);
	exp_d[0] = (char)(input_exp & 0xff);
	ret = sensor_i2c_write(chn, IMX385_SHS1, 16, exp_d, 3);
	return ret;
}

static int set_imx385_dol2_exposure(uint8_t chn, uint32_t input_exp1, uint32_t input_exp2)
{
	char exp_d[3];
	int ret = 0;
	
	input_exp2 = imx385_param[chn].VMAX - 1 - input_exp2;
	exp_d[2] = (char)((input_exp2 >> 16) & 0x03);
	exp_d[1] = (char)((input_exp2 >> 8) & 0xff);
	exp_d[0] = (char)(input_exp2 & 0xff);
	ret = sensor_i2c_write(chn, IMX385_SHS2, 16, exp_d, 3);

	input_exp1 = imx385_param[chn].RHS1 - 1 - input_exp1;	
	exp_d[2] = (char)((input_exp1 >> 16) & 0x03);
	exp_d[1] = (char)((input_exp1 >> 8) & 0xff);
	exp_d[0] = (char)(input_exp1 & 0xff);
	ret = sensor_i2c_write(chn, IMX385_SHS1, 16, exp_d, 3);
	
	return ret;
}

static int set_normal_gain(uint8_t chn, uint32_t input_gain)
{
	char gain_d[2];
	int ret = 0;
	
	gain_d[0] = (char)(input_gain & 0xff);
	gain_d[1] = (char)((input_gain >> 8) & 0x03);
	ret = sensor_i2c_write(chn, IMX385_GAIN, 16, gain_d, 2);

	return ret;
}

static int set_dol2_gain(uint8_t chn, uint32_t gain1, uint32_t gain2)
{
	char gain_d[2];
	int ret = 0;
	
	gain_d[0] = (char)(gain1 & 0xff);
	ret = sensor_i2c_write(chn, IMX385_GAIN, 16, gain_d, 1);

	gain_d[0] = (char)(gain2 & 0xff);
	ret = sensor_i2c_write(chn, IMX385_GAIN, 16, gain_d, 1);
	
	return ret;
}

static int set385_ex_gain_control(uint8_t chn, uint32_t expo_L, uint32_t expo_M, uint32_t expo_S, uint32_t gain)
{
	int ret = 0;
	uint32_t a_gain = 0;

	a_gain = sensor_log10(gain);//ux.8
	a_gain = (uint32_t)((a_gain * 200) >> 8);

        switch(imx385_param[chn].imx385_mode_save) {
        case imx385_NORMAL_M:
		set_normal_gain(chn, a_gain);
		set_imx385_normal_exposure(chn, expo_S);
		break;
        case imx385_DOL2_M:
		set_normal_gain(chn, a_gain);
		set_imx385_dol2_exposure(chn, expo_S, expo_L);
		break;
	default:
		LOG( LOG_ERR, " mode is err !");
		ret = -1;
		break;
	}
	
	return ret;
}

static int imx385_init(uint8_t chn, uint8_t mode)
{
	int ret = 0;
	uint32_t tmp_c = 0;
	uint32_t tmp_size = 0;
	uint16_t tmp_addr;
	char tmp_data;

	LOG( LOG_INFO,"  mode = %d", mode);
	
	switch (mode) {
	case 0:
#if 0
		tmp_size = sizeof(imx385_raw12_normal_setting)/sizeof(uint16_t);
		while (tmp_c < tmp_size) {
			tmp_addr = imx385_raw12_normal_setting[tmp_c++];
			tmp_data = (char)(imx385_raw12_normal_setting[tmp_c++] & 0xff);
			ret = sensor_i2c_write(chn, tmp_addr, 16, &tmp_data, 1);
		}
#endif
		imx385_param[chn].imx385_mode_save = imx385_NORMAL_M;
		imx385_param[chn].lines_per_second = 10074;
		imx385_param[chn].exposure_time_max = imx385_param[chn].VMAX - 2;
		imx385_param[chn].exposure_time_min = 1;
		imx385_param[chn].exposure_time_long_max = imx385_param[chn].FSC_DOL2 - 2;

		LOG( LOG_CRIT," imx385 raw12 normal init success ");
		break;
	case 1:
#if 0
		tmp_size = sizeof(imx385_raw12_dol2_setting)/sizeof(uint16_t);
		while (tmp_c < tmp_size) {
			tmp_addr = imx385_raw12_dol2_setting[tmp_c++];
			tmp_data = (char)(imx385_raw12_dol2_setting[tmp_c++] & 0xff);
			ret = sensor_i2c_write(chn, tmp_addr, 16, &tmp_data, 1);
		}
#endif
		imx385_param[chn].imx385_mode_save = imx385_DOL2_M;
		imx385_param[chn].lines_per_second = 7183;
		imx385_param[chn].exposure_time_max = imx385_param[chn].RHS1 - 2;
		imx385_param[chn].exposure_time_min = 1;
		imx385_param[chn].exposure_time_long_max = imx385_param[chn].FSC_DOL2 - 2;
		LOG( LOG_CRIT," imx385 raw12 dol2 init success");
		break;
	default:
		LOG( LOG_CRIT," init failed");
		ret = -1;
		break;
	}
	//set_imx385_init(chn);
	
	return ret;
}

static int imx385_hw_reset_enable (void)
{
	return 0;
}

static int imx385_hw_reset_disable (void)
{
	return 0;
}

static int32_t imx385_alloc_analog_gain( uint8_t chn, int32_t gain )
{
	int32_t analog_gain = 0;
	
	analog_gain = gain;	
	return analog_gain;	
}

static int32_t imx385_alloc_digital_gain( uint8_t chn, int32_t gain )
{
	int32_t digital_gain = 0;
	
	digital_gain = gain;	
	return digital_gain;	
}

static void imx385_alloc_integration_time( uint8_t chn, uint16_t *int_time, uint16_t *int_time_M, uint16_t *int_time_L )
{
}

static void imx385_update(uint8_t chn, struct sensor_priv_old updata)
{	
	LOG( LOG_CRIT," int_time %d, int_time_M %d, int_time_L %d, analog_gain %d ", updata.int_time, updata.int_time_M, updata.int_time_L, updata.analog_gain);
	
	set385_ex_gain_control(chn, updata.int_time_L, updata.int_time_M, updata.int_time, updata.analog_gain);
}

static uint16_t imx385_get_id( uint8_t chn )
{
	return 0;	
}

static void imx385_disable_isp( uint8_t chn )
{
}

static uint32_t imx385_read_register( uint8_t chn, uint32_t address )
{
	char buf[1];
	
	sensor_i2c_read(chn, (uint16_t)(address), 16, buf, 1);
	return (uint32_t)(buf[0]);
}

static void imx385_write_register( uint8_t chn, uint32_t address, uint32_t data )
{
	char buf = (char)(data & 0xff);
	sensor_i2c_write(chn, (uint16_t)(address), 16, &buf, 1);
}

static void imx385_stop_streaming( uint8_t chn )
{
	//0x3000, 0x01	
	char buf = 0x01;
	sensor_i2c_write(chn, 0x3000, 16, &buf, 1);
}

static void imx385_start_streaming( uint8_t chn )
{
	//0x3000, 0x00	
	char buf = 0x00;
	sensor_i2c_write(chn, 0x3002, 16, &buf, 1);
	sensor_i2c_write(chn, 0x3000, 16, &buf, 1);
}

void imx385_get_para(uint8_t chn, struct _setting_param_t *user_para)
{
	user_para->lines_per_second = imx385_param[chn].lines_per_second;
	user_para->analog_gain_max = imx385_param[chn].gain_max;
	user_para->digital_gain_max = imx385_param[chn].gain_max;
	user_para->exposure_time_max = imx385_param[chn].exposure_time_max;
	user_para->exposure_time_min = imx385_param[chn].exposure_time_min;
	user_para->exposure_time_long_max = imx385_param[chn].exposure_time_long_max;
	user_para->active_width = imx385_param[chn].active_width;
	user_para->active_height = imx385_param[chn].active_height;
}

static struct sensor_operations imx385_ops = {
	.param_enable = 1,
	.sensor_hw_reset_enable = imx385_hw_reset_enable,
	.sensor_hw_reset_disable = imx385_hw_reset_disable,
	.sensor_alloc_analog_gain = imx385_alloc_analog_gain,
	.sensor_alloc_digital_gain = imx385_alloc_digital_gain,
	.sensor_alloc_integration_time = imx385_alloc_integration_time,
	.sensor_update = imx385_update,
	.sensor_get_id = imx385_get_id,
	.sensor_disable_isp = imx385_disable_isp,
	.read_register = imx385_read_register,
	.write_register = imx385_write_register,
	.stop_streaming = imx385_stop_streaming,
	.start_streaming = imx385_start_streaming,
	.sensor_init = imx385_init,
	.sesor_get_para = imx385_get_para,
};

struct sensor_operations *imx385_ops_register(void)
{
	return &imx385_ops;
}
