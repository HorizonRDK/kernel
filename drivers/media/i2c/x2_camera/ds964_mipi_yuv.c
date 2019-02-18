#include "x2/x2_sensor_dev.h"

#define PATTERN_GEN
#define DS964_I2C_ADDR   (0x60>>1)
#define DS964_CHIP_INFO  { 0xF0, 0x5F, 0 }

static sensor_power_array_t ds964_mipi_yuv_power[] = {
    { SENSOR_POWER_TYPE_GPIO, 11, 0, 60 },
    { SENSOR_POWER_TYPE_GPIO, 11, 1, 6 },
    { SENSOR_POWER_TYPE_GPIO, 12, 0, 60 },
    { SENSOR_POWER_TYPE_GPIO, 12, 1, 6 },
};

static sensor_reg_array_t ds964_mipi_yuv_720p_reg_array[] = {
	{ 0x32, 0x01 }, // CSI0 select
	{ 0x21, 0x80 }, // CSI0 select
	{ 0x1f, 0x03 }, // CSI0 800mbps
	{ 0x33, 0x0D },
#ifdef PATTERN_GEN
    /* Pattern Generator, 1280 * 720 30fps*/
    { 0xB0, 0x00 }, // Indirect Pattern Gen Registors

	{ 0xB1, 0x01 },
	{ 0xB2, 0x01 },
	{ 0xB1, 0x02 },
	{ 0xB2, 0x33 },
	{ 0xB1, 0x03 },
	{ 0xB2, 0x1E },
	{ 0xB1, 0x04 },
	{ 0xB2, 0x0A },
	{ 0xB1, 0x05 },
	{ 0xB2, 0x00 },
	{ 0xB1, 0x06 },
	{ 0xB2, 0x01 },
	{ 0xB1, 0x07 },
	{ 0xB2, 0x40 },
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

#else
	/* board.devAddr = 0x7a */
/*	{ 0x10, 0x81 }, // To configure GPIO0 to bring out Lock for Port0,
	{ 0x11, 0x85 }, // To configure GPIO1 to bring out Lock for Port1,
	{ 0x12, 0x89 }, // To configure GPIO2 to bring out Lock for Port2,
	{ 0x13, 0x8D }, // To configure GPIO3 to bring out Lock for Port3,
	*/

	/* CSI0 output */
	{ 0x20, 0x00 }, // forwarding all RX to CSI0

	/* RX_PORT0 */
	{ 0x4c, 0x01 }, // RX_PORT0
	{ 0x58, 0x58 }, // enable pass throu
	{ 0x5c, 0xe8 }, // "SER_ALIAS_ID 0x5c value ", hex board.ReadReg 0x5c
	{ 0x5d, 0x60 }, // "SlaveID[0] 0x5d value ", hex board.ReadReg 0x5d
	{ 0x65, 0x70 }, // "SlaveAlias[0] 0x65 value ", hex board.ReadReg 0x65
	{ 0x7c, 0x81 }, // FV active low
	{ 0x70, 0x1E }, // VC0 and CSI0 datatype 0x1e yuv422_8b
	{ 0x6d, 0x7F }, // 913A 10-bit mode, FPD_MODE

	/* RX_PORT1 */
	{ 0x4c, 0x12 }, // RX_PORT1
	{ 0x58, 0x58 }, // enable pass throu
	{ 0x5c, 0x1a }, // "SER_ALIAS_ID 0x5c value ", hex board.ReadReg 0x5c
	{ 0x5d, 0x60 }, // "SlaveID[0] 0x5d value ", hex board.ReadReg 0x5d
	{ 0x65, 0x62 }, // "SlaveAlias[0] 0x65 value ", hex board.ReadReg 0x65
	{ 0x7c, 0x81 }, // FV active low
	{ 0x70, 0x5E }, // VC1 and CSI0 datatype 0x1e yuv422_8b
	{ 0x6d, 0x7F }, // 913A 10-bit mode, FPD_MODE

	/* RX_PORT2 */
	{ 0x4c, 0x24 }, // RX_PORT2
	{ 0x58, 0x58 }, // enable pass throu
	{ 0x5c, 0x1c }, // "SER_ALIAS_ID 0x5c value ", hex board.ReadReg 0x5c
	{ 0x5d, 0x60 }, // "SlaveID[0] 0x5d value ", hex board.ReadReg 0x5d
	{ 0x65, 0x66 }, // "SlaveAlias[0] 0x65 value ", hex board.ReadReg 0x65
	{ 0x7c, 0x81 }, // FV active low
	{ 0x70, 0x9E }, // VC2 and CSI0 datatype 0x1e yuv422_8b
	{ 0x6d, 0x7F }, // 913A 10-bit mode, FPD_MODE

	/* RX_PORT3 */
	{ 0x4c, 0x38 }, // RX_PORT3
	{ 0x58, 0x58 }, // enable pass throu
	{ 0x5c, 0x1e }, // "SER_ALIAS_ID 0x5c value ", hex board.ReadReg 0x5c
	{ 0x5d, 0x60 }, // "SlaveID[0] 0x5d value ", hex board.ReadReg 0x5d
	{ 0x65, 0x68 }, // "SlaveAlias[0] 0x65 value ", hex board.ReadReg 0x65
	{ 0x7c, 0x81 }, // // FV active low
	{ 0x70, 0xdE }, // VC3 and CSI0 datatype 0x1e yuv422_8b
	{ 0x6d, 0x7F }, // 913A 10-bit mode, FPD_MODE

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
#endif
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
static sensor_reg_array_t ds964_mipi_yuv_start_reg_array[] = {
	{ 0x33, 0x03 },
};
static sensor_reg_array_t ds964_mipi_yuv_stop_reg_array[] = {
	{ 0x33, 0x00 },
};

static sensor_settings_t ds964_mipi_yuv_720p_setting = {
    .init_settings = {
        .reg_setting = ds964_mipi_yuv_720p_reg_array,
        .size = sizeof(ds964_mipi_yuv_720p_reg_array) / sizeof(sensor_reg_array_t),
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

static sensor_init_t ds964_mipi_yuv_init[] = {
    { 4, SENSOR_PIX_LEN_8, { 1280, 720 },  &ds964_mipi_yuv_720p_setting },
};

sensor_info_t  ds964_mipi_yuv_info  = {
    .name = (uint8_t *)"ds964",
    .bus = SENSOR_BUS_MIPI,
    .format = SENSOR_FMT_YUV,
    .chip = {
        .i2c_bus = 1,
        .slave_addr = DS964_I2C_ADDR,
        .reg_size = SENSOR_I2C_BYTE_DATA,
        .data_size = SENSOR_I2C_BYTE_DATA,
        .chip_id = DS964_CHIP_INFO
    },
    .power = {
        .power_list = ds964_mipi_yuv_power,
        .size = sizeof(ds964_mipi_yuv_power) / sizeof(sensor_power_array_t),
    },
    .start = {
        .reg_setting = ds964_mipi_yuv_start_reg_array,
        .size = sizeof(ds964_mipi_yuv_start_reg_array) / sizeof(sensor_reg_array_t),
        .delay = 50,
    },
    .stop = {
        .reg_setting = ds964_mipi_yuv_stop_reg_array,
        .size = sizeof(ds964_mipi_yuv_stop_reg_array) / sizeof(sensor_reg_array_t),
        .delay = 50,
    },
    .init = ds964_mipi_yuv_init,
    .size = sizeof(ds964_mipi_yuv_init) / sizeof(sensor_init_t),
};
