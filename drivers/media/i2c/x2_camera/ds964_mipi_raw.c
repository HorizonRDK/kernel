#include "soc/hobot/x2_sensor_dev.h"

#define PATTERN_GEN
#define DS964_I2C_ADDR   (0x60>>1)
#define DS964_CHIP_INFO  { 0xF0, 0x5F, 0 }

static sensor_power_array_t ds964_mipi_raw_power[] = {
    { SENSOR_POWER_TYPE_GPIO, 11, 0, 60 },
    { SENSOR_POWER_TYPE_GPIO, 11, 1, 6 },
    { SENSOR_POWER_TYPE_GPIO, 12, 0, 60 },
    { SENSOR_POWER_TYPE_GPIO, 12, 1, 6 },
};

static sensor_reg_array_t ds964_mipi_raw14_720p_reg_array[] = {
	{ 0x32, 0x01 }, // CSI0 select
	{ 0x21, 0x80 }, // CSI0 select
	{ 0x1f, 0x03 }, // CSI0 800mbps
	{ 0x33, 0x0D },
    /* Pattern Generator, 1280 * 720 30fps*/
    { 0xB0, 0x00 }, // Indirect Pattern Gen Registors

	{ 0xB1, 0x01 },
	{ 0xB2, 0x01 },
	{ 0xB1, 0x02 },
	{ 0xB2, 0x33 },
	{ 0xB1, 0x03 },
	{ 0xB2, 0x2D },
	{ 0xB1, 0x04 },
	{ 0xB2, 0x08 },
	{ 0xB1, 0x05 },
	{ 0xB2, 0xC0 },
	{ 0xB1, 0x06 },
	{ 0xB2, 0x01 },
	{ 0xB1, 0x07 },
	{ 0xB2, 0x18 },
	{ 0xB1, 0x08 },
	{ 0xB2, 0x02 },
	{ 0xB1, 0x09 },
	{ 0xB2, 0xD0 },
	{ 0xB1, 0x0A },
	{ 0xB2, 0x04 },
	{ 0xB1, 0x0B },
	{ 0xB2, 0x40 },
	{ 0xB1, 0x0C },
	{ 0xB2, 0x0C },
	{ 0xB1, 0x0D },
	{ 0xB2, 0x67 },
	{ 0xB1, 0x0E },
	{ 0xB2, 0x21 },
	{ 0xB1, 0x0F },
	{ 0xB2, 0x0A },

    { 0xB0, 0x02 },
    { 0xB1, 0x40 },
    { 0xB2, 0x83 },
    { 0xB2, 0x8D },
    { 0xB2, 0x87 },
    { 0xB2, 0x87 },
    { 0xB2, 0x83 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB0, 0x02 },
    { 0xB1, 0x60 },
    { 0xB2, 0x83 },
    { 0xB2, 0x8D },
    { 0xB2, 0x87 },
    { 0xB2, 0x87 },
    { 0xB2, 0x83 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 }, 
};

static sensor_reg_array_t ds964_mipi_raw12_720p_reg_array[] = {
	{ 0x32, 0x01 }, // CSI0 select
	{ 0x21, 0x80 }, // CSI0 select
	{ 0x1f, 0x03 }, // CSI0 800mbps
	{ 0x33, 0x0D },
    /* Pattern Generator, 1280 * 720 30fps*/
    { 0xB0, 0x00 }, // Indirect Pattern Gen Registors

	{ 0xB1, 0x01 },
	{ 0xB2, 0x01 },
	{ 0xB1, 0x02 },
	{ 0xB2, 0x33 },
	{ 0xB1, 0x03 },
	{ 0xB2, 0x2C },
	{ 0xB1, 0x04 },
	{ 0xB2, 0x07 },
	{ 0xB1, 0x05 },
	{ 0xB2, 0x80 },
	{ 0xB1, 0x06 },
	{ 0xB2, 0x00 },
	{ 0xB1, 0x07 },
	{ 0xB2, 0xF0 },
	{ 0xB1, 0x08 },
	{ 0xB2, 0x02 },
	{ 0xB1, 0x09 },
	{ 0xB2, 0xD0 },
	{ 0xB1, 0x0A },
	{ 0xB2, 0x04 },
	{ 0xB1, 0x0B },
	{ 0xB2, 0x40 },
	{ 0xB1, 0x0C },
	{ 0xB2, 0x0C },
	{ 0xB1, 0x0D },
	{ 0xB2, 0x67 },
	{ 0xB1, 0x0E },
	{ 0xB2, 0x21 },
	{ 0xB1, 0x0F },
	{ 0xB2, 0x0A },

    { 0xB0, 0x02 },
    { 0xB1, 0x40 },
    { 0xB2, 0x83 },
    { 0xB2, 0x8D },
    { 0xB2, 0x87 },
    { 0xB2, 0x87 },
    { 0xB2, 0x83 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB0, 0x02 },
    { 0xB1, 0x60 },
    { 0xB2, 0x83 },
    { 0xB2, 0x8D },
    { 0xB2, 0x87 },
    { 0xB2, 0x87 },
    { 0xB2, 0x83 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 }, 
};

static sensor_reg_array_t ds964_mipi_raw8_720p_reg_array[] = {
	{ 0x32, 0x01 }, // CSI0 select
	{ 0x21, 0x80 }, // CSI0 select
	{ 0x1f, 0x03 }, // CSI0 800mbps
	{ 0x33, 0x0D },
    /* Pattern Generator, 1280 * 720 30fps*/
    { 0xB0, 0x00 }, // Indirect Pattern Gen Registors

	{ 0xB1, 0x01 },
	{ 0xB2, 0x01 },
	{ 0xB1, 0x02 },
	{ 0xB2, 0x33 },
	{ 0xB1, 0x03 },
	{ 0xB2, 0x2A },
	{ 0xB1, 0x04 },
	{ 0xB2, 0x05 },
	{ 0xB1, 0x05 },
	{ 0xB2, 0x00 },
	{ 0xB1, 0x06 },
	{ 0xB2, 0x00 },
	{ 0xB1, 0x07 },
	{ 0xB2, 0xA0 },
	{ 0xB1, 0x08 },
	{ 0xB2, 0x02 },
	{ 0xB1, 0x09 },
	{ 0xB2, 0xD0 },
	{ 0xB1, 0x0A },
	{ 0xB2, 0x04 },
	{ 0xB1, 0x0B },
	{ 0xB2, 0x40 },
	{ 0xB1, 0x0C },
	{ 0xB2, 0x0C },
	{ 0xB1, 0x0D },
	{ 0xB2, 0x67 },
	{ 0xB1, 0x0E },
	{ 0xB2, 0x21 },
	{ 0xB1, 0x0F },
	{ 0xB2, 0x0A },

    { 0xB0, 0x02 },
    { 0xB1, 0x40 },
    { 0xB2, 0x83 },
    { 0xB2, 0x8D },
    { 0xB2, 0x87 },
    { 0xB2, 0x87 },
    { 0xB2, 0x83 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB0, 0x02 },
    { 0xB1, 0x60 },
    { 0xB2, 0x83 },
    { 0xB2, 0x8D },
    { 0xB2, 0x87 },
    { 0xB2, 0x87 },
    { 0xB2, 0x83 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 }, 
};

static sensor_reg_array_t ds964_mipi_raw10_720p_reg_array[] = {
	{ 0x32, 0x01 }, // CSI0 select
	{ 0x21, 0x80 }, // CSI0 select
	{ 0x1f, 0x03 }, // CSI0 800mbps
	{ 0x33, 0x01 },
    /* Pattern Generator, 1280 * 720 30fps*/
    { 0xB0, 0x00 }, // Indirect Pattern Gen Registors

	{ 0xB1, 0x01 },
	{ 0xB2, 0x01 },
	{ 0xB1, 0x02 },
	{ 0xB2, 0x33 },
	{ 0xB1, 0x03 },
	{ 0xB2, 0x2B },
	{ 0xB1, 0x04 },
	{ 0xB2, 0x06 },
	{ 0xB1, 0x05 },
	{ 0xB2, 0x40 },
	{ 0xB1, 0x06 },
	{ 0xB2, 0x00 },
	{ 0xB1, 0x07 },
	{ 0xB2, 0xC8 },
	{ 0xB1, 0x08 },
	{ 0xB2, 0x02 },
	{ 0xB1, 0x09 },
	{ 0xB2, 0xD0 },
	{ 0xB1, 0x0A },
	{ 0xB2, 0x04 },
	{ 0xB1, 0x0B },
	{ 0xB2, 0x40 },
	{ 0xB1, 0x0C },
	{ 0xB2, 0x0C },
	{ 0xB1, 0x0D },
	{ 0xB2, 0x67 },
	{ 0xB1, 0x0E },
	{ 0xB2, 0x21 },
	{ 0xB1, 0x0F },
	{ 0xB2, 0x0A },

    { 0xB0, 0x02 },
    { 0xB1, 0x40 },
    { 0xB2, 0x83 },
    { 0xB2, 0x8D },
    { 0xB2, 0x87 },
    { 0xB2, 0x87 },
    { 0xB2, 0x83 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB0, 0x02 },
    { 0xB1, 0x60 },
    { 0xB2, 0x83 },
    { 0xB2, 0x8D },
    { 0xB2, 0x87 },
    { 0xB2, 0x87 },
    { 0xB2, 0x83 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 },
    { 0xB2, 0x86 },
    { 0xB2, 0x84 }, 
};

static sensor_reg_array_t ds964_mipi_raw_start_reg_array[] = {
	{ 0x33, 0x03 },
};
static sensor_reg_array_t ds964_mipi_raw_stop_reg_array[] = {
	{ 0x33, 0x00 },
};

static sensor_settings_t ds964_mipi_raw14_720p_setting = {
    .init_settings = {
        .reg_setting = ds964_mipi_raw14_720p_reg_array,
        .size = sizeof(ds964_mipi_raw14_720p_reg_array) / sizeof(sensor_reg_array_t),
        .delay = 50,
    },
    .mipi_params = {
        .lane_num = 4,
        .mclk = 24,
        .mipiclk = 400,
        .linelenth = 1440,
        .framelenth = 972,
        .fps = 30,
        .settle = 48,
    },
};

static sensor_settings_t ds964_mipi_raw12_720p_setting = {
    .init_settings = {
        .reg_setting = ds964_mipi_raw12_720p_reg_array,
        .size = sizeof(ds964_mipi_raw12_720p_reg_array) / sizeof(sensor_reg_array_t),
        .delay = 50,
    },
    .mipi_params = {
        .lane_num = 4,
        .mclk = 24,
        .mipiclk = 400,
        .linelenth = 1440,
        .framelenth = 972,
        .fps = 30,
        .settle = 48,
    },
};

static sensor_settings_t ds964_mipi_raw10_720p_setting = {
    .init_settings = {
        .reg_setting = ds964_mipi_raw10_720p_reg_array,
        .size = sizeof(ds964_mipi_raw10_720p_reg_array) / sizeof(sensor_reg_array_t),
        .delay = 50,
    },
    .mipi_params = {
        .lane_num = 4,
        .mclk = 24,
        .mipiclk = 400,
        .linelenth = 1440,
        .framelenth = 972,
        .fps = 30,
        .settle = 48,
    },
};

static sensor_settings_t ds964_mipi_raw8_720p_setting = {
    .init_settings = {
        .reg_setting = ds964_mipi_raw8_720p_reg_array,
        .size = sizeof(ds964_mipi_raw8_720p_reg_array) / sizeof(sensor_reg_array_t),
        .delay = 50,
    },
    .mipi_params = {
        .lane_num = 4,
        .mclk = 24,
        .mipiclk = 400,
        .linelenth = 1440,
        .framelenth = 972,
        .fps = 30,
        .settle = 48,
    },
};

static sensor_init_t ds964_mipi_raw_init[] = {
    { 4, SENSOR_PIX_LEN_8, { 1280, 720 },  &ds964_mipi_raw8_720p_setting },
    { 4, SENSOR_PIX_LEN_10, { 1280, 720 },  &ds964_mipi_raw10_720p_setting },
    { 4, SENSOR_PIX_LEN_12, { 1280, 720 },  &ds964_mipi_raw12_720p_setting },
    { 4, SENSOR_PIX_LEN_14, { 1280, 720 },  &ds964_mipi_raw14_720p_setting },
};

sensor_info_t  ds964_mipi_raw_info  = {
    .name = (uint8_t *)"ds964",
    .bus = SENSOR_BUS_MIPI,
    .format = SENSOR_FMT_RAW,
    .chip = {
        .i2c_bus = 1,
        .slave_addr = DS964_I2C_ADDR,
        .reg_size = SENSOR_I2C_BYTE_DATA,
        .data_size = SENSOR_I2C_BYTE_DATA,
        .chip_id = DS964_CHIP_INFO
    },
    .power = {
        .power_list = ds964_mipi_raw_power,
        .size = sizeof(ds964_mipi_raw_power) / sizeof(sensor_power_array_t),
    },
    .start = {
        .reg_setting = ds964_mipi_raw_start_reg_array,
        .size = sizeof(ds964_mipi_raw_start_reg_array) / sizeof(sensor_reg_array_t),
        .delay = 50,
    },
    .stop = {
        .reg_setting = ds964_mipi_raw_stop_reg_array,
        .size = sizeof(ds964_mipi_raw_stop_reg_array) / sizeof(sensor_reg_array_t),
        .delay = 50,
    },
    .init = ds964_mipi_raw_init,
    .size = sizeof(ds964_mipi_raw_init) / sizeof(sensor_init_t),
};
