#include "x2/x2_sensor_dev.h"

//#define TEST_PATTERN 1
#define DS953_I2C_ADDR   (0x30>>1)
#define DS953_CHIP_INFO  { 0x37, 0x7A, 0 }

static sensor_power_array_t ds953_mipi_yuv_power[] = {
	{SENSOR_POWER_TYPE_GPIO, 11, 0, 60},
	{SENSOR_POWER_TYPE_GPIO, 11, 1, 6},
	{SENSOR_POWER_TYPE_GPIO, 12, 0, 60},
	{SENSOR_POWER_TYPE_GPIO, 12, 1, 6},
};

#define PATTERN_GEN
static sensor_reg_array_t ds953_mipi_yuv_720p_reg_array[] = {
	{0x03, 0x00},
	{0x06, 0x23},
	{0x07, 0x15},
	{0x17, 0x3E}, //Enable Sensor, Select GPIO1 to sense
	{0x18, 0xB2}, //Enable Sensor Gain Setting (Use Default)
	{0x1A, 0x62}, //Set Sensor Upper and Lower Limits (Use Default)
	{0x1D, 0x3F}, //Enable Sensor Alarms
	{0x1E, 0x7F}, //Enable Sending Alarms over BCC
#ifdef PATTERN_GEN
	{0xB0, 0x00}, //Indirect Pattern Gen Registers
	{0xB1, 0x01}, //PGEN_CTL
	{0xB2, 0x01}, //
	{0xB1, 0x02}, //PGEN_CFG
	{0xB2, 0x33}, //
	{0xB1, 0x03}, //PGEN_CSI_DI
	{0xB2, 0x1e}, //RGB888
	{0xB1, 0x04}, //PGEN_LINE_SIZE1
	{0xB2, 0x0A}, //
	{0xB1, 0x05}, //PGEN_LINE_SIZE0
	{0xB2, 0x00}, //
	{0xB1, 0x06}, //PGEN_BAR_SIZE1
	{0xB2, 0x01}, //
	{0xB1, 0x07}, //PGEN_BAR_SIZE0
	{0xB2, 0x40}, //
	{0xB1, 0x08}, //PGEN_ACT_LPF1
	{0xB2, 0x02}, //
	{0xB1, 0x09}, //PGEN_ACT_LPF0
	{0xB2, 0xD0}, //
	{0xB1, 0x0A}, //PGEN_TOT_LPF1
	{0xB2, 0x03}, //
	{0xB1, 0x0B}, //PGEN_TOT_LPF0
	{0xB2, 0x00}, //
	{0xB1, 0x0C}, //PGEN_LINE_PD1
	{0xB2, 0x0C}, //
	{0xB1, 0x0D}, //PGEN_LINE_PD0
	{0xB2, 0x67}, //
	{0xB1, 0x0E}, //PGEN_VBP
	{0xB2, 0x21}, //
	{0xB1, 0x0F}, //PGEN_VFP
	{0xB2, 0x0A}, //
#endif
};
static sensor_reg_array_t ds953_mipi_yuv_start_reg_array[] = {
	{0x03, 0x02},
};
static sensor_reg_array_t ds953_mipi_yuv_stop_reg_array[] = {
	{0x03, 0x00},
};

static sensor_settings_t ds953_mipi_yuv_720p_setting = {
	.init_settings = {
		.reg_setting = ds953_mipi_yuv_720p_reg_array,
		.size = sizeof(ds953_mipi_yuv_720p_reg_array) / sizeof(sensor_reg_array_t),
		.delay = 50,
	},
	.mipi_params = {
		.lane_num = 4,
		.mclk = 24,
		.mipiclk = 400,
		.linelenth = 1280,
		.framelenth = 720,
		.fps = 30,
		.settle = 48,
	},
};

static sensor_init_t ds953_mipi_yuv_init[] = {
	{4, SENSOR_PIX_LEN_8, {1280, 720},  &ds953_mipi_yuv_720p_setting},
};

sensor_info_t  ds953_mipi_yuv_info  = {
	.name = (uint8_t *)"ds953",
	.bus = SENSOR_BUS_MIPI,
	.format = SENSOR_FMT_YUV,
	.chip = {
		.i2c_bus = 1,
		.slave_addr = DS953_I2C_ADDR,
		.reg_size = SENSOR_I2C_BYTE_DATA,
		.data_size = SENSOR_I2C_BYTE_DATA,
		.chip_id = DS953_CHIP_INFO
	},
	.power = {
		.power_list = ds953_mipi_yuv_power,
		.size = sizeof(ds953_mipi_yuv_power) / sizeof(sensor_power_array_t),
	},
	.start = {
		.reg_setting = ds953_mipi_yuv_start_reg_array,
		.size = sizeof(ds953_mipi_yuv_start_reg_array) / sizeof(sensor_reg_array_t),
		.delay = 50,
	},
	.stop = {
		.reg_setting = ds953_mipi_yuv_stop_reg_array,
		.size = sizeof(ds953_mipi_yuv_stop_reg_array) / sizeof(sensor_reg_array_t),
		.delay = 50,
	},
	.init = ds953_mipi_yuv_init,
	.size = sizeof(ds953_mipi_yuv_init) / sizeof(sensor_init_t),
};
