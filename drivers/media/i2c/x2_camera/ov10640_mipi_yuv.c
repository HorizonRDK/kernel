#include "soc/hobot/x2_sensor_dev.h"

//#define TEST_PATTERN 1
#define OV10640_I2C_ADDR   (0x48>>1)
#define OV10640_CHIP_INFO  { 0, 0x149, 0 }

static sensor_power_array_t ov10640_mipi_yuv_power[] = {
	{SENSOR_POWER_TYPE_GPIO, 11, 0, 60},
	{SENSOR_POWER_TYPE_GPIO, 11, 1, 6},
	{SENSOR_POWER_TYPE_GPIO, 12, 0, 60},
	{SENSOR_POWER_TYPE_GPIO, 12, 1, 6},
};

static sensor_reg_array_t ov10640_mipi_yuv_720p_reg_array[] = {
	{0xfffd, 0x80},
	{0xfffe, 0x29},
	{0x2015, 0xa1},
	{0xd000, 0x04},
};
static sensor_reg_array_t ov10640_mipi_yuv_start_reg_array[] = {
	{0xfffd, 0x80},
	{0xfffe, 0x29},
	{0x2015, 0x80},  //80:continuous    81:gate clock
	{0xfffd, 0x80},
	{0xfffe, 0x19},
	{0x5000, 0x01},
	{0xfffe, 0x80},
	{0x00c0, 0xe2},
};
static sensor_reg_array_t ov10640_mipi_yuv_stop_reg_array[] = {
	{0xfffd, 0x80},
	{0xfffe, 0x29},
	{0x2015, 0xa1},
	{0xd000, 0x04},
};

static sensor_settings_t ov10640_mipi_yuv_720p_setting = {
	.init_settings = {
		.reg_setting = ov10640_mipi_yuv_720p_reg_array,
		.size = sizeof(ov10640_mipi_yuv_720p_reg_array) / sizeof(sensor_reg_array_t),
		.delay = 50,
	},
	.mipi_params = {
		.lane_num = 4,
		.mclk = 24,
		.mipiclk = 1536,
		.linelenth = 1460,
		.framelenth = 912,
		.fps = 30,
		.settle = 48,
	},
};

static sensor_init_t ov10640_mipi_yuv_init[] = {
	{4, SENSOR_PIX_LEN_8, {1280, 720},  &ov10640_mipi_yuv_720p_setting},
};

sensor_info_t  ov10640_mipi_yuv_info  = {
	.name = (uint8_t *)"ov10640",
	.bus = SENSOR_BUS_MIPI,
	.format = SENSOR_FMT_YUV,
	.chip = {
		.i2c_bus = 1,
		.slave_addr = OV10640_I2C_ADDR,
		.reg_size = SENSOR_I2C_WORD_DATA,
		.data_size = SENSOR_I2C_BYTE_DATA,
		.chip_id = OV10640_CHIP_INFO
	},
	.power = {
		.power_list = ov10640_mipi_yuv_power,
		.size = sizeof(ov10640_mipi_yuv_power) / sizeof(sensor_power_array_t),
	},
	.start = {
		.reg_setting = ov10640_mipi_yuv_start_reg_array,
		.size = sizeof(ov10640_mipi_yuv_start_reg_array) / sizeof(sensor_reg_array_t),
		.delay = 50,
	},
	.stop = {
		.reg_setting = ov10640_mipi_yuv_stop_reg_array,
		.size = sizeof(ov10640_mipi_yuv_stop_reg_array) / sizeof(sensor_reg_array_t),
		.delay = 50,
	},
	.init = ov10640_mipi_yuv_init,
	.size = sizeof(ov10640_mipi_yuv_init) / sizeof(sensor_init_t),
};
