#include "x2/x2_sensor_dev.h"

//#define TEST_PATTERN 1
#define OV10635_I2C_ADDR   (0x70>>1)
#define OV10635_CHIP_INFO  { 0x300A, 0xA635, 0 }

static sensor_power_array_t ov10635_dvp_yuv_power[] = {
	{SENSOR_POWER_TYPE_GPIO, 11, 0, 60},
	{SENSOR_POWER_TYPE_GPIO, 11, 1, 6},
	{SENSOR_POWER_TYPE_GPIO, 12, 0, 60},
	{SENSOR_POWER_TYPE_GPIO, 12, 1, 6},
};

static sensor_reg_array_t ov10635_dvp_yuv_720p_reg_array[] = {
	{0x0103, 0x01},
	{0x300c, 0x61},
	{0x301b, 0xff},
	{0x301c, 0xff},
	{0x301a, 0xff},
	{0x3011, 0x42},
	{0x6900, 0x0c},
	{0x6901, 0x11},
	{0x3503, 0x10},
	{0x3025, 0x03},
	/* system clock and pll clock  */
	{0x3003, 0x24}, //0x30, 0x14
	{0x3004, 0x22}, //0x22, 0x11
	{0x3005, 0x24}, //0x30, 0x20
	{0x3006, 0x31}, //0x22, 0x91
	{0x3600, 0x74},
	{0x3601, 0x2b},
	{0x3612, 0x00},
	{0x3611, 0x67},
	{0x3633, 0xca},
	{0x3602, 0x2f},
	{0x3603, 0x00},
	{0x3630, 0x28},
	{0x3631, 0x16},
	{0x3714, 0x10},
	{0x371d, 0x01},
	{0x4300, 0x38},
	{0x3007, 0x01},
	{0x3024, 0x01}, //0x00, 0x01
	{0x3020, 0x0b},
	{0x3702, 0x1a},
	{0x3703, 0x40},
	{0x3704, 0x2a},
	{0x3709, 0xa8},
	{0x3709, 0xa8},
	{0x370c, 0xc7},
	{0x370d, 0x80},
	{0x3712, 0x00},
	{0x3713, 0x20},
	{0x3715, 0x04},
	{0x381d, 0x40},
	{0x381c, 0x00},
	{0x3822, 0x50},
	{0x3824, 0x50},
	{0x3815, 0x8c},
	{0x3804, 0x05},
	{0x3805, 0x1f},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3806, 0x02},
	{0x3807, 0xfd},
	{0x3802, 0x00},
	{0x3803, 0x2c},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x02},
	{0x380b, 0xd0},
	{0x380c, 0x06},
	{0x380d, 0xf6},
	{0x6e42, 0x02},
	{0x6e43, 0xec},
	{0x380e, 0x02},
	{0x380f, 0xec},
	{0x3813, 0x02},
	{0x3811, 0x10},
	{0x381f, 0x0c},
	{0x3828, 0x03},
	{0x3829, 0x10},
	{0x382a, 0x10},
	{0x382b, 0x10},
	{0x3621, 0x64},
	{0x5005, 0x08},
	{0x56d5, 0x00},
	{0x56d6, 0x80},
	{0x56d7, 0x00},
	{0x56d8, 0x00},
	{0x56d9, 0x00},
	{0x56da, 0x80},
	{0x56db, 0x00},
	{0x56dc, 0x00},
	{0x56e8, 0x00},
	{0x56e9, 0x7f},
	{0x56ea, 0x00},
	{0x56eb, 0x7f},
	{0x5100, 0x00},
	{0x5101, 0x80},
	{0x5102, 0x00},
	{0x5103, 0x80},
	{0x5104, 0x00},
	{0x5105, 0x80},
	{0x5106, 0x00},
	{0x5107, 0x80},
	{0x5108, 0x00},
	{0x5109, 0x00},
	{0x510a, 0x00},
	{0x510b, 0x00},
	{0x510c, 0x00},
	{0x510d, 0x00},
	{0x510e, 0x00},
	{0x510f, 0x00},
	{0x5110, 0x00},
	{0x5111, 0x80},
	{0x5112, 0x00},
	{0x5113, 0x80},
	{0x5114, 0x00},
	{0x5115, 0x80},
	{0x5116, 0x00},
	{0x5117, 0x80},
	{0x5118, 0x00},
	{0x5119, 0x00},
	{0x511a, 0x00},
	{0x511b, 0x00},
	{0x511c, 0x00},
	{0x511d, 0x00},
	{0x511e, 0x00},
	{0x511f, 0x00},
	{0x56d0, 0x00},
	{0x5006, 0x24},
	{0x5608, 0x0e},
	{0x52d7, 0x06},
	{0x528d, 0x08},
	{0x5293, 0x12},
	{0x52d3, 0x12},
	{0x5288, 0x06},
	{0x5289, 0x20},
	{0x52c8, 0x06},
	{0x52c9, 0x20},
	{0x52cd, 0x04},
	{0x5381, 0x00},
	{0x5382, 0xff},
	{0x5589, 0x76},
	{0x558a, 0x47},
	{0x558b, 0xef},
	{0x558c, 0xc9},
	{0x558d, 0x49},
	{0x558e, 0x30},
	{0x558f, 0x67},
	{0x5590, 0x3f},
	{0x5591, 0xf0},
	{0x5592, 0x10},
	{0x55a2, 0x6d},
	{0x55a3, 0x55},
	{0x55a4, 0xc3},
	{0x55a5, 0xb5},
	{0x55a6, 0x43},
	{0x55a7, 0x38},
	{0x55a8, 0x5f},
	{0x55a9, 0x4b},
	{0x55aa, 0xf0},
	{0x55ab, 0x10},
	{0x5581, 0x52},
	{0x5300, 0x01},
	{0x5301, 0x00},
	{0x5302, 0x00},
	{0x5303, 0x0e},
	{0x5304, 0x00},
	{0x5305, 0x0e},
	{0x5306, 0x00},
	{0x5307, 0x36},
	{0x5308, 0x00},
	{0x5309, 0xd9},
	{0x530a, 0x00},
	{0x530b, 0x0f},
	{0x530c, 0x00},
	{0x530d, 0x2c},
	{0x530e, 0x00},
	{0x530f, 0x59},
	{0x5310, 0x00},
	{0x5311, 0x7b},
	{0x5312, 0x00},
	{0x5313, 0x22},
	{0x5314, 0x00},
	{0x5315, 0xd5},
	{0x5316, 0x00},
	{0x5317, 0x13},
	{0x5318, 0x00},
	{0x5319, 0x18},
	{0x531a, 0x00},
	{0x531b, 0x26},
	{0x531c, 0x00},
	{0x531d, 0xdc},
	{0x531e, 0x00},
	{0x531f, 0x02},
	{0x5320, 0x00},
	{0x5321, 0x24},
	{0x5322, 0x00},
	{0x5323, 0x56},
	{0x5324, 0x00},
	{0x5325, 0x85},
	{0x5326, 0x00},
	{0x5327, 0x20},
	{0x5609, 0x01},
	{0x560a, 0x40},
	{0x560b, 0x01},
	{0x560c, 0x40},
	{0x560d, 0x00},
	{0x560e, 0xfa},
	{0x560f, 0x00},
	{0x5610, 0xfa},
	{0x5611, 0x02},
	{0x5612, 0x80},
	{0x5613, 0x02},
	{0x5614, 0x80},
	{0x5615, 0x01},
	{0x5616, 0x2c},
	{0x5617, 0x01},
	{0x5618, 0x2c},
	{0x563b, 0x01},
	{0x563c, 0x01},
	{0x563d, 0x01},
	{0x563e, 0x01},
	{0x563f, 0x03},
	{0x5640, 0x03},
	{0x5641, 0x03},
	{0x5642, 0x05},
	{0x5643, 0x09},
	{0x5644, 0x05},
	{0x5645, 0x05},
	{0x5646, 0x05},
	{0x5647, 0x05},
	{0x5651, 0x00},
	{0x5652, 0x80},
	{0x521a, 0x01},
	{0x521b, 0x03},
	{0x521c, 0x06},
	{0x521d, 0x0a},
	{0x521e, 0x0e},
	{0x521f, 0x12},
	{0x5220, 0x16},
	{0x5223, 0x02},
	{0x5225, 0x04},
	{0x5227, 0x08},
	{0x5229, 0x0c},
	{0x522b, 0x12},
	{0x522d, 0x18},
	{0x522f, 0x1e},
	{0x5241, 0x04},
	{0x5242, 0x01},
	{0x5243, 0x03},
	{0x5244, 0x06},
	{0x5245, 0x0a},
	{0x5246, 0x0e},
	{0x5247, 0x12},
	{0x5248, 0x16},
	{0x524a, 0x03},
	{0x524c, 0x04},
	{0x524e, 0x08},
	{0x5250, 0x0c},
	{0x5252, 0x12},
	{0x5254, 0x18},
	{0x5256, 0x1e},
	{0x4606, 0x07},
	{0x4607, 0x71},
	{0x460a, 0x02},
	{0x460b, 0x70},
	{0x460c, 0x00},
	{0x4620, 0x0e},
	{0x4700, 0x04},
	{0x4701, 0x00},
	{0x4702, 0x01},
	{0x4004, 0x04},
	{0x4005, 0x18},
	{0x4001, 0x06},
	{0x4050, 0x22},
	{0x4051, 0x24},
	{0x4052, 0x02},
	{0x4057, 0x9c},
	{0x405a, 0x00},
	{0x4202, 0x02},
	{0x3023, 0x10},
	{0x0100, 0x01},
	{0x0100, 0x01},
	{0x6f10, 0x07},
	{0x6f11, 0x82},
	{0x6f12, 0x04},
	{0x6f13, 0x00},
	{0x6f14, 0x1f},
	{0x6f15, 0xdd},
	{0x6f16, 0x04},
	{0x6f17, 0x04},
	{0x6f18, 0x36},
	{0x6f19, 0x66},
	{0x6f1a, 0x04},
	{0x6f1b, 0x08},
	{0x6f1c, 0x0c},
	{0x6f1d, 0xe7},
	{0x6f1e, 0x04},
	{0x6f1f, 0x0c},
	{0xd000, 0x19},
	{0xd001, 0xa0},
	{0xd002, 0x00},
	{0xd003, 0x01},
	{0xd004, 0xa9},
	{0xd005, 0xad},
	{0xd006, 0x10},
	{0xd007, 0x40},
	{0xd008, 0x44},
	{0xd009, 0x00},
	{0xd00a, 0x68},
	{0xd00b, 0x00},
	{0xd00c, 0x15},
	{0xd00d, 0x00},
	{0xd00e, 0x00},
	{0xd00f, 0x00},
	{0xd010, 0x19},
	{0xd011, 0xa0},
	{0xd012, 0x00},
	{0xd013, 0x01},
	{0xd014, 0xa9},
	{0xd015, 0xad},
	{0xd016, 0x13},
	{0xd017, 0xd0},
	{0xd018, 0x44},
	{0xd019, 0x00},
	{0xd01a, 0x68},
	{0xd01b, 0x00},
	{0xd01c, 0x15},
	{0xd01d, 0x00},
	{0xd01e, 0x00},
	{0xd01f, 0x00},
	{0xd020, 0x19},
	{0xd021, 0xa0},
	{0xd022, 0x00},
	{0xd023, 0x01},
	{0xd024, 0xa9},
	{0xd025, 0xad},
	{0xd026, 0x14},
	{0xd027, 0xb8},
	{0xd028, 0x44},
	{0xd029, 0x00},
	{0xd02a, 0x68},
	{0xd02b, 0x00},
	{0xd02c, 0x15},
	{0xd02d, 0x00},
	{0xd02e, 0x00},
	{0xd02f, 0x00},
	{0xd030, 0x19},
	{0xd031, 0xa0},
	{0xd032, 0x00},
	{0xd033, 0x01},
	{0xd034, 0xa9},
	{0xd035, 0xad},
	{0xd036, 0x14},
	{0xd037, 0xdc},
	{0xd038, 0x44},
	{0xd039, 0x00},
	{0xd03a, 0x68},
	{0xd03b, 0x00},
	{0xd03c, 0x15},
	{0xd03d, 0x00},
	{0xd03e, 0x00},
	{0xd03f, 0x00},
	{0xd040, 0x9c},
	{0xd041, 0x21},
	{0xd042, 0xff},
	{0xd043, 0xe4},
	{0xd044, 0xd4},
	{0xd045, 0x01},
	{0xd046, 0x48},
	{0xd047, 0x00},
	{0xd048, 0xd4},
	{0xd049, 0x01},
	{0xd04a, 0x50},
	{0xd04b, 0x04},
	{0xd04c, 0xd4},
	{0xd04d, 0x01},
	{0xd04e, 0x60},
	{0xd04f, 0x08},
	{0xd050, 0xd4},
	{0xd051, 0x01},
	{0xd052, 0x70},
	{0xd053, 0x0c},
	{0xd054, 0xd4},
	{0xd055, 0x01},
	{0xd056, 0x80},
	{0xd057, 0x10},
	{0xd058, 0x19},
	{0xd059, 0xc0},
	{0xd05a, 0x00},
	{0xd05b, 0x01},
	{0xd05c, 0xa9},
	{0xd05d, 0xce},
	{0xd05e, 0x02},
	{0xd05f, 0xa4},
	{0xd060, 0x9c},
	{0xd061, 0xa0},
	{0xd062, 0x00},
	{0xd063, 0x00},
	{0xd064, 0x84},
	{0xd065, 0x6e},
	{0xd066, 0x00},
	{0xd067, 0x00},
	{0xd068, 0xd8},
	{0xd069, 0x03},
	{0xd06a, 0x28},
	{0xd06b, 0x76},
	{0xd06c, 0x1a},
	{0xd06d, 0x00},
	{0xd06e, 0x00},
	{0xd06f, 0x01},
	{0xd070, 0xaa},
	{0xd071, 0x10},
	{0xd072, 0x03},
	{0xd073, 0xf0},
	{0xd074, 0x18},
	{0xd075, 0x60},
	{0xd076, 0x00},
	{0xd077, 0x01},
	{0xd078, 0xa8},
	{0xd079, 0x63},
	{0xd07a, 0x07},
	{0xd07b, 0x80},
	{0xd07c, 0xe0},
	{0xd07d, 0xa0},
	{0xd07e, 0x00},
	{0xd07f, 0x04},
	{0xd080, 0x18},
	{0xd081, 0xc0},
	{0xd082, 0x00},
	{0xd083, 0x00},
	{0xd084, 0xa8},
	{0xd085, 0xc6},
	{0xd086, 0x00},
	{0xd087, 0x00},
	{0xd088, 0x8c},
	{0xd089, 0x63},
	{0xd08a, 0x00},
	{0xd08b, 0x00},
	{0xd08c, 0xd4},
	{0xd08d, 0x01},
	{0xd08e, 0x28},
	{0xd08f, 0x14},
	{0xd090, 0xd4},
	{0xd091, 0x01},
	{0xd092, 0x30},
	{0xd093, 0x18},
	{0xd094, 0x07},
	{0xd095, 0xff},
	{0xd096, 0xf8},
	{0xd097, 0xfd},
	{0xd098, 0x9c},
	{0xd099, 0x80},
	{0xd09a, 0x00},
	{0xd09b, 0x03},
	{0xd09c, 0xa5},
	{0xd09d, 0x6b},
	{0xd09e, 0x00},
	{0xd09f, 0xff},
	{0xd0a0, 0x18},
	{0xd0a1, 0xc0},
	{0xd0a2, 0x00},
	{0xd0a3, 0x01},
	{0xd0a4, 0xa8},
	{0xd0a5, 0xc6},
	{0xd0a6, 0x01},
	{0xd0a7, 0x02},
	{0xd0a8, 0xe1},
	{0xd0a9, 0x6b},
	{0xd0aa, 0x58},
	{0xd0ab, 0x00},
	{0xd0ac, 0x84},
	{0xd0ad, 0x8e},
	{0xd0ae, 0x00},
	{0xd0af, 0x00},
	{0xd0b0, 0xe1},
	{0xd0b1, 0x6b},
	{0xd0b2, 0x30},
	{0xd0b3, 0x00},
	{0xd0b4, 0x98},
	{0xd0b5, 0xb0},
	{0xd0b6, 0x00},
	{0xd0b7, 0x00},
	{0xd0b8, 0x8c},
	{0xd0b9, 0x64},
	{0xd0ba, 0x00},
	{0xd0bb, 0x6e},
	{0xd0bc, 0xe5},
	{0xd0bd, 0xa5},
	{0xd0be, 0x18},
	{0xd0bf, 0x00},
	{0xd0c0, 0x10},
	{0xd0c1, 0x00},
	{0xd0c2, 0x00},
	{0xd0c3, 0x06},
	{0xd0c4, 0x95},
	{0xd0c5, 0x8b},
	{0xd0c6, 0x00},
	{0xd0c7, 0x00},
	{0xd0c8, 0x94},
	{0xd0c9, 0xa4},
	{0xd0ca, 0x00},
	{0xd0cb, 0x70},
	{0xd0cc, 0xe5},
	{0xd0cd, 0x65},
	{0xd0ce, 0x60},
	{0xd0cf, 0x00},
	{0xd0d0, 0x0c},
	{0xd0d1, 0x00},
	{0xd0d2, 0x00},
	{0xd0d3, 0x62},
	{0xd0d4, 0x15},
	{0xd0d5, 0x00},
	{0xd0d6, 0x00},
	{0xd0d7, 0x00},
	{0xd0d8, 0x18},
	{0xd0d9, 0x60},
	{0xd0da, 0x80},
	{0xd0db, 0x06},
	{0xd0dc, 0xa8},
	{0xd0dd, 0x83},
	{0xd0de, 0x38},
	{0xd0df, 0x29},
	{0xd0e0, 0xa8},
	{0xd0e1, 0xe3},
	{0xd0e2, 0x40},
	{0xd0e3, 0x08},
	{0xd0e4, 0x8c},
	{0xd0e5, 0x84},
	{0xd0e6, 0x00},
	{0xd0e7, 0x00},
	{0xd0e8, 0xa8},
	{0xd0e9, 0xa3},
	{0xd0ea, 0x40},
	{0xd0eb, 0x09},
	{0xd0ec, 0xa8},
	{0xd0ed, 0xc3},
	{0xd0ee, 0x38},
	{0xd0ef, 0x2a},
	{0xd0f0, 0xd8},
	{0xd0f1, 0x07},
	{0xd0f2, 0x20},
	{0xd0f3, 0x00},
	{0xd0f4, 0x8c},
	{0xd0f5, 0x66},
	{0xd0f6, 0x00},
	{0xd0f7, 0x00},
	{0xd0f8, 0xd8},
	{0xd0f9, 0x05},
	{0xd0fa, 0x18},
	{0xd0fb, 0x00},
	{0xd0fc, 0x18},
	{0xd0fd, 0x60},
	{0xd0fe, 0x00},
	{0xd0ff, 0x01},
	{0xd100, 0x98},
	{0xd101, 0x90},
	{0xd102, 0x00},
	{0xd103, 0x00},
	{0xd104, 0x84},
	{0xd105, 0xae},
	{0xd106, 0x00},
	{0xd107, 0x00},
	{0xd108, 0xa8},
	{0xd109, 0x63},
	{0xd10a, 0x06},
	{0xd10b, 0x4c},
	{0xd10c, 0x9c},
	{0xd10d, 0xc0},
	{0xd10e, 0x00},
	{0xd10f, 0x00},
	{0xd110, 0xd8},
	{0xd111, 0x03},
	{0xd112, 0x30},
	{0xd113, 0x00},
	{0xd114, 0x8c},
	{0xd115, 0x65},
	{0xd116, 0x00},
	{0xd117, 0x6e},
	{0xd118, 0xe5},
	{0xd119, 0x84},
	{0xd11a, 0x18},
	{0xd11b, 0x00},
	{0xd11c, 0x10},
	{0xd11d, 0x00},
	{0xd11e, 0x00},
	{0xd11f, 0x07},
	{0xd120, 0x18},
	{0xd121, 0x80},
	{0xd122, 0x80},
	{0xd123, 0x06},
	{0xd124, 0x94},
	{0xd125, 0x65},
	{0xd126, 0x00},
	{0xd127, 0x70},
	{0xd128, 0xe5},
	{0xd129, 0x43},
	{0xd12a, 0x60},
	{0xd12b, 0x00},
	{0xd12c, 0x0c},
	{0xd12d, 0x00},
	{0xd12e, 0x00},
	{0xd12f, 0x3e},
	{0xd130, 0xa8},
	{0xd131, 0x64},
	{0xd132, 0x38},
	{0xd133, 0x24},
	{0xd134, 0x18},
	{0xd135, 0x80},
	{0xd136, 0x80},
	{0xd137, 0x06},
	{0xd138, 0xa8},
	{0xd139, 0x64},
	{0xd13a, 0x38},
	{0xd13b, 0x24},
	{0xd13c, 0x8c},
	{0xd13d, 0x63},
	{0xd13e, 0x00},
	{0xd13f, 0x00},
	{0xd140, 0xa4},
	{0xd141, 0x63},
	{0xd142, 0x00},
	{0xd143, 0x40},
	{0xd144, 0xbc},
	{0xd145, 0x23},
	{0xd146, 0x00},
	{0xd147, 0x00},
	{0xd148, 0x0c},
	{0xd149, 0x00},
	{0xd14a, 0x00},
	{0xd14b, 0x2a},
	{0xd14c, 0xa8},
	{0xd14d, 0x64},
	{0xd14e, 0x6e},
	{0xd14f, 0x44},
	{0xd150, 0x19},
	{0xd151, 0x00},
	{0xd152, 0x80},
	{0xd153, 0x06},
	{0xd154, 0xa8},
	{0xd155, 0xe8},
	{0xd156, 0x3d},
	{0xd157, 0x05},
	{0xd158, 0x8c},
	{0xd159, 0x67},
	{0xd15a, 0x00},
	{0xd15b, 0x00},
	{0xd15c, 0xb8},
	{0xd15d, 0x63},
	{0xd15e, 0x00},
	{0xd15f, 0x18},
	{0xd160, 0xb8},
	{0xd161, 0x63},
	{0xd162, 0x00},
	{0xd163, 0x98},
	{0xd164, 0xbc},
	{0xd165, 0x03},
	{0xd166, 0x00},
	{0xd167, 0x00},
	{0xd168, 0x10},
	{0xd169, 0x00},
	{0xd16a, 0x00},
	{0xd16b, 0x10},
	{0xd16c, 0xa9},
	{0xd16d, 0x48},
	{0xd16e, 0x67},
	{0xd16f, 0x02},
	{0xd170, 0xb8},
	{0xd171, 0xa3},
	{0xd172, 0x00},
	{0xd173, 0x19},
	{0xd174, 0x8c},
	{0xd175, 0x8a},
	{0xd176, 0x00},
	{0xd177, 0x00},
	{0xd178, 0xa9},
	{0xd179, 0x68},
	{0xd17a, 0x67},
	{0xd17b, 0x03},
	{0xd17c, 0xb8},
	{0xd17d, 0xc4},
	{0xd17e, 0x00},
	{0xd17f, 0x08},
	{0xd180, 0x8c},
	{0xd181, 0x6b},
	{0xd182, 0x00},
	{0xd183, 0x00},
	{0xd184, 0xb8},
	{0xd185, 0x85},
	{0xd186, 0x00},
	{0xd187, 0x98},
	{0xd188, 0xe0},
	{0xd189, 0x63},
	{0xd18a, 0x30},
	{0xd18b, 0x04},
	{0xd18c, 0xe0},
	{0xd18d, 0x64},
	{0xd18e, 0x18},
	{0xd18f, 0x00},
	{0xd190, 0xa4},
	{0xd191, 0x83},
	{0xd192, 0xff},
	{0xd193, 0xff},
	{0xd194, 0xb8},
	{0xd195, 0x64},
	{0xd196, 0x00},
	{0xd197, 0x48},
	{0xd198, 0xd8},
	{0xd199, 0x0a},
	{0xd19a, 0x18},
	{0xd19b, 0x00},
	{0xd19c, 0xd8},
	{0xd19d, 0x0b},
	{0xd19e, 0x20},
	{0xd19f, 0x00},
	{0xd1a0, 0x9c},
	{0xd1a1, 0x60},
	{0xd1a2, 0x00},
	{0xd1a3, 0x00},
	{0xd1a4, 0xd8},
	{0xd1a5, 0x07},
	{0xd1a6, 0x18},
	{0xd1a7, 0x00},
	{0xd1a8, 0xa8},
	{0xd1a9, 0x68},
	{0xd1aa, 0x38},
	{0xd1ab, 0x22},
	{0xd1ac, 0x9c},
	{0xd1ad, 0x80},
	{0xd1ae, 0x00},
	{0xd1af, 0x70},
	{0xd1b0, 0xa8},
	{0xd1b1, 0xe8},
	{0xd1b2, 0x38},
	{0xd1b3, 0x43},
	{0xd1b4, 0xd8},
	{0xd1b5, 0x03},
	{0xd1b6, 0x20},
	{0xd1b7, 0x00},
	{0xd1b8, 0x9c},
	{0xd1b9, 0xa0},
	{0xd1ba, 0x00},
	{0xd1bb, 0x00},
	{0xd1bc, 0xa8},
	{0xd1bd, 0xc8},
	{0xd1be, 0x38},
	{0xd1bf, 0x42},
	{0xd1c0, 0x8c},
	{0xd1c1, 0x66},
	{0xd1c2, 0x00},
	{0xd1c3, 0x00},
	{0xd1c4, 0x9c},
	{0xd1c5, 0xa5},
	{0xd1c6, 0x00},
	{0xd1c7, 0x01},
	{0xd1c8, 0xb8},
	{0xd1c9, 0x83},
	{0xd1ca, 0x00},
	{0xd1cb, 0x08},
	{0xd1cc, 0xa4},
	{0xd1cd, 0xa5},
	{0xd1ce, 0x00},
	{0xd1cf, 0xff},
	{0xd1d0, 0x8c},
	{0xd1d1, 0x67},
	{0xd1d2, 0x00},
	{0xd1d3, 0x00},
	{0xd1d4, 0xe0},
	{0xd1d5, 0x63},
	{0xd1d6, 0x20},
	{0xd1d7, 0x00},
	{0xd1d8, 0xa4},
	{0xd1d9, 0x63},
	{0xd1da, 0xff},
	{0xd1db, 0xff},
	{0xd1dc, 0xbc},
	{0xd1dd, 0x43},
	{0xd1de, 0x00},
	{0xd1df, 0x07},
	{0xd1e0, 0x0c},
	{0xd1e1, 0x00},
	{0xd1e2, 0x00},
	{0xd1e3, 0x5b},
	{0xd1e4, 0xbc},
	{0xd1e5, 0x05},
	{0xd1e6, 0x00},
	{0xd1e7, 0x02},
	{0xd1e8, 0x03},
	{0xd1e9, 0xff},
	{0xd1ea, 0xff},
	{0xd1eb, 0xf6},
	{0xd1ec, 0x9c},
	{0xd1ed, 0xa0},
	{0xd1ee, 0x00},
	{0xd1ef, 0x00},
	{0xd1f0, 0xa8},
	{0xd1f1, 0xa4},
	{0xd1f2, 0x55},
	{0xd1f3, 0x86},
	{0xd1f4, 0x8c},
	{0xd1f5, 0x63},
	{0xd1f6, 0x00},
	{0xd1f7, 0x00},
	{0xd1f8, 0xa8},
	{0xd1f9, 0xc4},
	{0xd1fa, 0x6e},
	{0xd1fb, 0x45},
	{0xd1fc, 0xa8},
	{0xd1fd, 0xe4},
	{0xd1fe, 0x55},
	{0xd1ff, 0x87},
	{0xd200, 0xd8},
	{0xd201, 0x05},
	{0xd202, 0x18},
	{0xd203, 0x00},
	{0xd204, 0x8c},
	{0xd205, 0x66},
	{0xd206, 0x00},
	{0xd207, 0x00},
	{0xd208, 0xa8},
	{0xd209, 0xa4},
	{0xd20a, 0x6e},
	{0xd20b, 0x46},
	{0xd20c, 0xd8},
	{0xd20d, 0x07},
	{0xd20e, 0x18},
	{0xd20f, 0x00},
	{0xd210, 0xa8},
	{0xd211, 0x84},
	{0xd212, 0x55},
	{0xd213, 0x88},
	{0xd214, 0x8c},
	{0xd215, 0x65},
	{0xd216, 0x00},
	{0xd217, 0x00},
	{0xd218, 0xd8},
	{0xd219, 0x04},
	{0xd21a, 0x18},
	{0xd21b, 0x00},
	{0xd21c, 0x03},
	{0xd21d, 0xff},
	{0xd21e, 0xff},
	{0xd21f, 0xce},
	{0xd220, 0x19},
	{0xd221, 0x00},
	{0xd222, 0x80},
	{0xd223, 0x06},
	{0xd224, 0x8c},
	{0xd225, 0x63},
	{0xd226, 0x00},
	{0xd227, 0x00},
	{0xd228, 0xa4},
	{0xd229, 0x63},
	{0xd22a, 0x00},
	{0xd22b, 0x40},
	{0xd22c, 0xbc},
	{0xd22d, 0x23},
	{0xd22e, 0x00},
	{0xd22f, 0x00},
	{0xd230, 0x13},
	{0xd231, 0xff},
	{0xd232, 0xff},
	{0xd233, 0xc8},
	{0xd234, 0x9d},
	{0xd235, 0x00},
	{0xd236, 0x00},
	{0xd237, 0x40},
	{0xd238, 0xa8},
	{0xd239, 0x64},
	{0xd23a, 0x55},
	{0xd23b, 0x86},
	{0xd23c, 0xa8},
	{0xd23d, 0xa4},
	{0xd23e, 0x55},
	{0xd23f, 0x87},
	{0xd240, 0xd8},
	{0xd241, 0x03},
	{0xd242, 0x40},
	{0xd243, 0x00},
	{0xd244, 0xa8},
	{0xd245, 0x64},
	{0xd246, 0x55},
	{0xd247, 0x88},
	{0xd248, 0xd8},
	{0xd249, 0x05},
	{0xd24a, 0x40},
	{0xd24b, 0x00},
	{0xd24c, 0xd8},
	{0xd24d, 0x03},
	{0xd24e, 0x40},
	{0xd24f, 0x00},
	{0xd250, 0x03},
	{0xd251, 0xff},
	{0xd252, 0xff},
	{0xd253, 0xc1},
	{0xd254, 0x19},
	{0xd255, 0x00},
	{0xd256, 0x80},
	{0xd257, 0x06},
	{0xd258, 0x94},
	{0xd259, 0x84},
	{0xd25a, 0x00},
	{0xd25b, 0x72},
	{0xd25c, 0xe5},
	{0xd25d, 0xa4},
	{0xd25e, 0x60},
	{0xd25f, 0x00},
	{0xd260, 0x0c},
	{0xd261, 0x00},
	{0xd262, 0x00},
	{0xd263, 0x3f},
	{0xd264, 0x9d},
	{0xd265, 0x60},
	{0xd266, 0x01},
	{0xd267, 0x00},
	{0xd268, 0x85},
	{0xd269, 0x4e},
	{0xd26a, 0x00},
	{0xd26b, 0x00},
	{0xd26c, 0x98},
	{0xd26d, 0x70},
	{0xd26e, 0x00},
	{0xd26f, 0x00},
	{0xd270, 0x8c},
	{0xd271, 0x8a},
	{0xd272, 0x00},
	{0xd273, 0x6f},
	{0xd274, 0xe5},
	{0xd275, 0x63},
	{0xd276, 0x20},
	{0xd277, 0x00},
	{0xd278, 0x10},
	{0xd279, 0x00},
	{0xd27a, 0x00},
	{0xd27b, 0x07},
	{0xd27c, 0x15},
	{0xd27d, 0x00},
	{0xd27e, 0x00},
	{0xd27f, 0x00},
	{0xd280, 0x8c},
	{0xd281, 0xaa},
	{0xd282, 0x00},
	{0xd283, 0x6e},
	{0xd284, 0xe0},
	{0xd285, 0x63},
	{0xd286, 0x28},
	{0xd287, 0x02},
	{0xd288, 0xe0},
	{0xd289, 0x84},
	{0xd28a, 0x28},
	{0xd28b, 0x02},
	{0xd28c, 0x07},
	{0xd28d, 0xff},
	{0xd28e, 0xf8},
	{0xd28f, 0x66},
	{0xd290, 0xe0},
	{0xd291, 0x63},
	{0xd292, 0x5b},
	{0xd293, 0x06},
	{0xd294, 0x8c},
	{0xd295, 0x6a},
	{0xd296, 0x00},
	{0xd297, 0x77},
	{0xd298, 0xe0},
	{0xd299, 0x63},
	{0xd29a, 0x5b},
	{0xd29b, 0x06},
	{0xd29c, 0xbd},
	{0xd29d, 0x63},
	{0xd29e, 0x00},
	{0xd29f, 0x00},
	{0xd2a0, 0x0c},
	{0xd2a1, 0x00},
	{0xd2a2, 0x00},
	{0xd2a3, 0x3c},
	{0xd2a4, 0x15},
	{0xd2a5, 0x00},
	{0xd2a6, 0x00},
	{0xd2a7, 0x00},
	{0xd2a8, 0x8c},
	{0xd2a9, 0x8a},
	{0xd2aa, 0x00},
	{0xd2ab, 0x78},
	{0xd2ac, 0xb8},
	{0xd2ad, 0x63},
	{0xd2ae, 0x00},
	{0xd2af, 0x88},
	{0xd2b0, 0xe1},
	{0xd2b1, 0x64},
	{0xd2b2, 0x5b},
	{0xd2b3, 0x06},
	{0xd2b4, 0xbd},
	{0xd2b5, 0x6b},
	{0xd2b6, 0x00},
	{0xd2b7, 0x00},
	{0xd2b8, 0x0c},
	{0xd2b9, 0x00},
	{0xd2ba, 0x00},
	{0xd2bb, 0x34},
	{0xd2bc, 0xd4},
	{0xd2bd, 0x01},
	{0xd2be, 0x18},
	{0xd2bf, 0x14},
	{0xd2c0, 0xb9},
	{0xd2c1, 0x6b},
	{0xd2c2, 0x00},
	{0xd2c3, 0x88},
	{0xd2c4, 0x85},
	{0xd2c5, 0x01},
	{0xd2c6, 0x00},
	{0xd2c7, 0x14},
	{0xd2c8, 0xbd},
	{0xd2c9, 0x68},
	{0xd2ca, 0x00},
	{0xd2cb, 0x00},
	{0xd2cc, 0x0c},
	{0xd2cd, 0x00},
	{0xd2ce, 0x00},
	{0xd2cf, 0x2c},
	{0xd2d0, 0xd4},
	{0xd2d1, 0x01},
	{0xd2d2, 0x58},
	{0xd2d3, 0x18},
	{0xd2d4, 0x84},
	{0xd2d5, 0x81},
	{0xd2d6, 0x00},
	{0xd2d7, 0x14},
	{0xd2d8, 0xbd},
	{0xd2d9, 0xa4},
	{0xd2da, 0x01},
	{0xd2db, 0x00},
	{0xd2dc, 0x10},
	{0xd2dd, 0x00},
	{0xd2de, 0x00},
	{0xd2df, 0x05},
	{0xd2e0, 0x84},
	{0xd2e1, 0xc1},
	{0xd2e2, 0x00},
	{0xd2e3, 0x18},
	{0xd2e4, 0x9c},
	{0xd2e5, 0xa0},
	{0xd2e6, 0x01},
	{0xd2e7, 0x00},
	{0xd2e8, 0xd4},
	{0xd2e9, 0x01},
	{0xd2ea, 0x28},
	{0xd2eb, 0x14},
	{0xd2ec, 0x84},
	{0xd2ed, 0xc1},
	{0xd2ee, 0x00},
	{0xd2ef, 0x18},
	{0xd2f0, 0xbd},
	{0xd2f1, 0x66},
	{0xd2f2, 0x00},
	{0xd2f3, 0x00},
	{0xd2f4, 0x0c},
	{0xd2f5, 0x00},
	{0xd2f6, 0x00},
	{0xd2f7, 0x20},
	{0xd2f8, 0x9d},
	{0xd2f9, 0x00},
	{0xd2fa, 0x00},
	{0xd2fb, 0x00},
	{0xd2fc, 0x84},
	{0xd2fd, 0x61},
	{0xd2fe, 0x00},
	{0xd2ff, 0x18},
	{0xd300, 0xbd},
	{0xd301, 0xa3},
	{0xd302, 0x01},
	{0xd303, 0x00},
	{0xd304, 0x10},
	{0xd305, 0x00},
	{0xd306, 0x00},
	{0xd307, 0x03},
	{0xd308, 0x9c},
	{0xd309, 0x80},
	{0xd30a, 0x01},
	{0xd30b, 0x00},
	{0xd30c, 0xd4},
	{0xd30d, 0x01},
	{0xd30e, 0x20},
	{0xd30f, 0x18},
	{0xd310, 0x18},
	{0xd311, 0x60},
	{0xd312, 0x80},
	{0xd313, 0x06},
	{0xd314, 0x85},
	{0xd315, 0x01},
	{0xd316, 0x00},
	{0xd317, 0x14},
	{0xd318, 0xa8},
	{0xd319, 0x83},
	{0xd31a, 0x38},
	{0xd31b, 0x29},
	{0xd31c, 0xa8},
	{0xd31d, 0xc3},
	{0xd31e, 0x40},
	{0xd31f, 0x08},
	{0xd320, 0x8c},
	{0xd321, 0x84},
	{0xd322, 0x00},
	{0xd323, 0x00},
	{0xd324, 0xa8},
	{0xd325, 0xa3},
	{0xd326, 0x38},
	{0xd327, 0x2a},
	{0xd328, 0xa8},
	{0xd329, 0xe3},
	{0xd32a, 0x40},
	{0xd32b, 0x09},
	{0xd32c, 0xe0},
	{0xd32d, 0x64},
	{0xd32e, 0x40},
	{0xd32f, 0x00},
	{0xd330, 0xd8},
	{0xd331, 0x06},
	{0xd332, 0x18},
	{0xd333, 0x00},
	{0xd334, 0x8c},
	{0xd335, 0x65},
	{0xd336, 0x00},
	{0xd337, 0x00},
	{0xd338, 0x84},
	{0xd339, 0x81},
	{0xd33a, 0x00},
	{0xd33b, 0x18},
	{0xd33c, 0xe3},
	{0xd33d, 0xe3},
	{0xd33e, 0x20},
	{0xd33f, 0x00},
	{0xd340, 0xd8},
	{0xd341, 0x07},
	{0xd342, 0xf8},
	{0xd343, 0x00},
	{0xd344, 0x03},
	{0xd345, 0xff},
	{0xd346, 0xff},
	{0xd347, 0x6f},
	{0xd348, 0x18},
	{0xd349, 0x60},
	{0xd34a, 0x00},
	{0xd34b, 0x01},
	{0xd34c, 0x0f},
	{0xd34d, 0xff},
	{0xd34e, 0xff},
	{0xd34f, 0x9d},
	{0xd350, 0x18},
	{0xd351, 0x60},
	{0xd352, 0x80},
	{0xd353, 0x06},
	{0xd354, 0x00},
	{0xd355, 0x00},
	{0xd356, 0x00},
	{0xd357, 0x11},
	{0xd358, 0xa8},
	{0xd359, 0x83},
	{0xd35a, 0x6e},
	{0xd35b, 0x43},
	{0xd35c, 0xe0},
	{0xd35d, 0x6c},
	{0xd35e, 0x28},
	{0xd35f, 0x02},
	{0xd360, 0xe0},
	{0xd361, 0x84},
	{0xd362, 0x28},
	{0xd363, 0x02},
	{0xd364, 0x07},
	{0xd365, 0xff},
	{0xd366, 0xf8},
	{0xd367, 0x30},
	{0xd368, 0xb8},
	{0xd369, 0x63},
	{0xd36a, 0x00},
	{0xd36b, 0x08},
	{0xd36c, 0x03},
	{0xd36d, 0xff},
	{0xd36e, 0xff},
	{0xd36f, 0xc0},
	{0xd370, 0x85},
	{0xd371, 0x4e},
	{0xd372, 0x00},
	{0xd373, 0x00},
	{0xd374, 0x03},
	{0xd375, 0xff},
	{0xd376, 0xff},
	{0xd377, 0xe7},
	{0xd378, 0xd4},
	{0xd379, 0x01},
	{0xd37a, 0x40},
	{0xd37b, 0x18},
	{0xd37c, 0x9c},
	{0xd37d, 0x60},
	{0xd37e, 0x00},
	{0xd37f, 0x00},
	{0xd380, 0x03},
	{0xd381, 0xff},
	{0xd382, 0xff},
	{0xd383, 0xdb},
	{0xd384, 0xd4},
	{0xd385, 0x01},
	{0xd386, 0x18},
	{0xd387, 0x14},
	{0xd388, 0x03},
	{0xd389, 0xff},
	{0xd38a, 0xff},
	{0xd38b, 0xce},
	{0xd38c, 0x9d},
	{0xd38d, 0x6b},
	{0xd38e, 0x00},
	{0xd38f, 0xff},
	{0xd390, 0x03},
	{0xd391, 0xff},
	{0xd392, 0xff},
	{0xd393, 0xc6},
	{0xd394, 0x9c},
	{0xd395, 0x63},
	{0xd396, 0x00},
	{0xd397, 0xff},
	{0xd398, 0xa8},
	{0xd399, 0xe3},
	{0xd39a, 0x38},
	{0xd39b, 0x0f},
	{0xd39c, 0x8c},
	{0xd39d, 0x84},
	{0xd39e, 0x00},
	{0xd39f, 0x00},
	{0xd3a0, 0xa8},
	{0xd3a1, 0xa3},
	{0xd3a2, 0x38},
	{0xd3a3, 0x0e},
	{0xd3a4, 0xa8},
	{0xd3a5, 0xc3},
	{0xd3a6, 0x6e},
	{0xd3a7, 0x42},
	{0xd3a8, 0xd8},
	{0xd3a9, 0x07},
	{0xd3aa, 0x20},
	{0xd3ab, 0x00},
	{0xd3ac, 0x8c},
	{0xd3ad, 0x66},
	{0xd3ae, 0x00},
	{0xd3af, 0x00},
	{0xd3b0, 0xd8},
	{0xd3b1, 0x05},
	{0xd3b2, 0x18},
	{0xd3b3, 0x00},
	{0xd3b4, 0x85},
	{0xd3b5, 0x21},
	{0xd3b6, 0x00},
	{0xd3b7, 0x00},
	{0xd3b8, 0x85},
	{0xd3b9, 0x41},
	{0xd3ba, 0x00},
	{0xd3bb, 0x04},
	{0xd3bc, 0x85},
	{0xd3bd, 0x81},
	{0xd3be, 0x00},
	{0xd3bf, 0x08},
	{0xd3c0, 0x85},
	{0xd3c1, 0xc1},
	{0xd3c2, 0x00},
	{0xd3c3, 0x0c},
	{0xd3c4, 0x86},
	{0xd3c5, 0x01},
	{0xd3c6, 0x00},
	{0xd3c7, 0x10},
	{0xd3c8, 0x44},
	{0xd3c9, 0x00},
	{0xd3ca, 0x48},
	{0xd3cb, 0x00},
	{0xd3cc, 0x9c},
	{0xd3cd, 0x21},
	{0xd3ce, 0x00},
	{0xd3cf, 0x1c},
	{0xd3d0, 0x9c},
	{0xd3d1, 0x21},
	{0xd3d2, 0xff},
	{0xd3d3, 0xfc},
	{0xd3d4, 0xd4},
	{0xd3d5, 0x01},
	{0xd3d6, 0x48},
	{0xd3d7, 0x00},
	{0xd3d8, 0x18},
	{0xd3d9, 0x60},
	{0xd3da, 0x00},
	{0xd3db, 0x01},
	{0xd3dc, 0xa8},
	{0xd3dd, 0x63},
	{0xd3de, 0x07},
	{0xd3df, 0x80},
	{0xd3e0, 0x8c},
	{0xd3e1, 0x63},
	{0xd3e2, 0x00},
	{0xd3e3, 0x68},
	{0xd3e4, 0xbc},
	{0xd3e5, 0x03},
	{0xd3e6, 0x00},
	{0xd3e7, 0x00},
	{0xd3e8, 0x10},
	{0xd3e9, 0x00},
	{0xd3ea, 0x00},
	{0xd3eb, 0x0c},
	{0xd3ec, 0x15},
	{0xd3ed, 0x00},
	{0xd3ee, 0x00},
	{0xd3ef, 0x00},
	{0xd3f0, 0x07},
	{0xd3f1, 0xff},
	{0xd3f2, 0xd9},
	{0xd3f3, 0x98},
	{0xd3f4, 0x15},
	{0xd3f5, 0x00},
	{0xd3f6, 0x00},
	{0xd3f7, 0x00},
	{0xd3f8, 0x18},
	{0xd3f9, 0x60},
	{0xd3fa, 0x80},
	{0xd3fb, 0x06},
	{0xd3fc, 0xa8},
	{0xd3fd, 0x63},
	{0xd3fe, 0xc4},
	{0xd3ff, 0xb8},
	{0xd400, 0x8c},
	{0xd401, 0x63},
	{0xd402, 0x00},
	{0xd403, 0x00},
	{0xd404, 0xbc},
	{0xd405, 0x23},
	{0xd406, 0x00},
	{0xd407, 0x01},
	{0xd408, 0x10},
	{0xd409, 0x00},
	{0xd40a, 0x00},
	{0xd40b, 0x25},
	{0xd40c, 0x9d},
	{0xd40d, 0x00},
	{0xd40e, 0x00},
	{0xd40f, 0x00},
	{0xd410, 0x00},
	{0xd411, 0x00},
	{0xd412, 0x00},
	{0xd413, 0x0b},
	{0xd414, 0xb8},
	{0xd415, 0xe8},
	{0xd416, 0x00},
	{0xd417, 0x02},
	{0xd418, 0x07},
	{0xd419, 0xff},
	{0xd41a, 0xd6},
	{0xd41b, 0x24},
	{0xd41c, 0x15},
	{0xd41d, 0x00},
	{0xd41e, 0x00},
	{0xd41f, 0x00},
	{0xd420, 0x18},
	{0xd421, 0x60},
	{0xd422, 0x80},
	{0xd423, 0x06},
	{0xd424, 0xa8},
	{0xd425, 0x63},
	{0xd426, 0xc4},
	{0xd427, 0xb8},
	{0xd428, 0x8c},
	{0xd429, 0x63},
	{0xd42a, 0x00},
	{0xd42b, 0x00},
	{0xd42c, 0xbc},
	{0xd42d, 0x23},
	{0xd42e, 0x00},
	{0xd42f, 0x01},
	{0xd430, 0x10},
	{0xd431, 0x00},
	{0xd432, 0x00},
	{0xd433, 0x1b},
	{0xd434, 0x9d},
	{0xd435, 0x00},
	{0xd436, 0x00},
	{0xd437, 0x00},
	{0xd438, 0xb8},
	{0xd439, 0xe8},
	{0xd43a, 0x00},
	{0xd43b, 0x02},
	{0xd43c, 0x9c},
	{0xd43d, 0xc0},
	{0xd43e, 0x00},
	{0xd43f, 0x00},
	{0xd440, 0x18},
	{0xd441, 0xa0},
	{0xd442, 0x80},
	{0xd443, 0x06},
	{0xd444, 0xe0},
	{0xd445, 0x67},
	{0xd446, 0x30},
	{0xd447, 0x00},
	{0xd448, 0xa8},
	{0xd449, 0xa5},
	{0xd44a, 0xce},
	{0xd44b, 0xb0},
	{0xd44c, 0x19},
	{0xd44d, 0x60},
	{0xd44e, 0x00},
	{0xd44f, 0x01},
	{0xd450, 0xa9},
	{0xd451, 0x6b},
	{0xd452, 0x06},
	{0xd453, 0x14},
	{0xd454, 0xe0},
	{0xd455, 0x83},
	{0xd456, 0x28},
	{0xd457, 0x00},
	{0xd458, 0x9c},
	{0xd459, 0xc6},
	{0xd45a, 0x00},
	{0xd45b, 0x01},
	{0xd45c, 0xe0},
	{0xd45d, 0x63},
	{0xd45e, 0x18},
	{0xd45f, 0x00},
	{0xd460, 0x8c},
	{0xd461, 0x84},
	{0xd462, 0x00},
	{0xd463, 0x00},
	{0xd464, 0xe0},
	{0xd465, 0xa3},
	{0xd466, 0x58},
	{0xd467, 0x00},
	{0xd468, 0xa4},
	{0xd469, 0xc6},
	{0xd46a, 0x00},
	{0xd46b, 0xff},
	{0xd46c, 0xb8},
	{0xd46d, 0x64},
	{0xd46e, 0x00},
	{0xd46f, 0x18},
	{0xd470, 0xbc},
	{0xd471, 0x46},
	{0xd472, 0x00},
	{0xd473, 0x03},
	{0xd474, 0x94},
	{0xd475, 0x85},
	{0xd476, 0x00},
	{0xd477, 0x00},
	{0xd478, 0xb8},
	{0xd479, 0x63},
	{0xd47a, 0x00},
	{0xd47b, 0x98},
	{0xd47c, 0xe0},
	{0xd47d, 0x64},
	{0xd47e, 0x18},
	{0xd47f, 0x00},
	{0xd480, 0x0f},
	{0xd481, 0xff},
	{0xd482, 0xff},
	{0xd483, 0xf0},
	{0xd484, 0xdc},
	{0xd485, 0x05},
	{0xd486, 0x18},
	{0xd487, 0x00},
	{0xd488, 0x9c},
	{0xd489, 0x68},
	{0xd48a, 0x00},
	{0xd48b, 0x01},
	{0xd48c, 0xa5},
	{0xd48d, 0x03},
	{0xd48e, 0x00},
	{0xd48f, 0xff},
	{0xd490, 0xbc},
	{0xd491, 0x48},
	{0xd492, 0x00},
	{0xd493, 0x01},
	{0xd494, 0x0f},
	{0xd495, 0xff},
	{0xd496, 0xff},
	{0xd497, 0xea},
	{0xd498, 0xb8},
	{0xd499, 0xe8},
	{0xd49a, 0x00},
	{0xd49b, 0x02},
	{0xd49c, 0x18},
	{0xd49d, 0x60},
	{0xd49e, 0x00},
	{0xd49f, 0x01},
	{0xd4a0, 0xa8},
	{0xd4a1, 0x63},
	{0xd4a2, 0x06},
	{0xd4a3, 0x14},
	{0xd4a4, 0x07},
	{0xd4a5, 0xff},
	{0xd4a6, 0xe4},
	{0xd4a7, 0x05},
	{0xd4a8, 0x9c},
	{0xd4a9, 0x83},
	{0xd4aa, 0x00},
	{0xd4ab, 0x10},
	{0xd4ac, 0x85},
	{0xd4ad, 0x21},
	{0xd4ae, 0x00},
	{0xd4af, 0x00},
	{0xd4b0, 0x44},
	{0xd4b1, 0x00},
	{0xd4b2, 0x48},
	{0xd4b3, 0x00},
	{0xd4b4, 0x9c},
	{0xd4b5, 0x21},
	{0xd4b6, 0x00},
	{0xd4b7, 0x04},
	{0xd4b8, 0x18},
	{0xd4b9, 0x60},
	{0xd4ba, 0x00},
	{0xd4bb, 0x01},
	{0xd4bc, 0x9c},
	{0xd4bd, 0x80},
	{0xd4be, 0xff},
	{0xd4bf, 0xff},
	{0xd4c0, 0xa8},
	{0xd4c1, 0x63},
	{0xd4c2, 0x09},
	{0xd4c3, 0xef},
	{0xd4c4, 0xd8},
	{0xd4c5, 0x03},
	{0xd4c6, 0x20},
	{0xd4c7, 0x00},
	{0xd4c8, 0x18},
	{0xd4c9, 0x60},
	{0xd4ca, 0x80},
	{0xd4cb, 0x06},
	{0xd4cc, 0xa8},
	{0xd4cd, 0x63},
	{0xd4ce, 0xc9},
	{0xd4cf, 0xef},
	{0xd4d0, 0xd8},
	{0xd4d1, 0x03},
	{0xd4d2, 0x20},
	{0xd4d3, 0x00},
	{0xd4d4, 0x44},
	{0xd4d5, 0x00},
	{0xd4d6, 0x48},
	{0xd4d7, 0x00},
	{0xd4d8, 0x15},
	{0xd4d9, 0x00},
	{0xd4da, 0x00},
	{0xd4db, 0x00},
	{0xd4dc, 0x18},
	{0xd4dd, 0x80},
	{0xd4de, 0x00},
	{0xd4df, 0x01},
	{0xd4e0, 0xa8},
	{0xd4e1, 0x84},
	{0xd4e2, 0x0a},
	{0xd4e3, 0x12},
	{0xd4e4, 0x8c},
	{0xd4e5, 0x64},
	{0xd4e6, 0x00},
	{0xd4e7, 0x00},
	{0xd4e8, 0xbc},
	{0xd4e9, 0x03},
	{0xd4ea, 0x00},
	{0xd4eb, 0x00},
	{0xd4ec, 0x13},
	{0xd4ed, 0xff},
	{0xd4ee, 0xff},
	{0xd4ef, 0xfe},
	{0xd4f0, 0x15},
	{0xd4f1, 0x00},
	{0xd4f2, 0x00},
	{0xd4f3, 0x00},
	{0xd4f4, 0x44},
	{0xd4f5, 0x00},
	{0xd4f6, 0x48},
	{0xd4f7, 0x00},
	{0xd4f8, 0x15},
	{0xd4f9, 0x00},
	{0xd4fa, 0x00},
	{0xd4fb, 0x00},
	{0xd4fc, 0x00},
	{0xd4fd, 0x00},
	{0xd4fe, 0x00},
	{0xd4ff, 0x00},
	{0xd500, 0x00},
	{0xd501, 0x00},
	{0xd502, 0x00},
	{0xd503, 0x00},
	{0x6f0e, 0x33},
	{0x6f0f, 0x33},
	{0x460e, 0x08},
	{0x460f, 0x01},
	{0x4610, 0x05},
	{0x4611, 0x01},
	{0x4612, 0x00},
	{0x4613, 0x01},
	{0x4605, 0x08},
	{0x4608, 0x00},
	{0x4609, 0x08},
	{0x6804, 0x00},
	{0x6805, 0x06},
	{0x6806, 0x00},
	{0x5120, 0x00},
	{0x3510, 0x00},
	{0x3504, 0x00},
	{0x6800, 0x00},
	{0x6f0d, 0x0f},
	{0x5000, 0xff},
	{0x5001, 0xbf},
	{0x5002, 0x7e},
	{0x5003, 0x0c},
	{0x503d, 0x00},
	{0xc450, 0x01},
	{0xc452, 0x04},
	{0xc453, 0x00},
	{0xc454, 0x00},
	{0xc455, 0x00},
	{0xc456, 0x00},
	{0xc457, 0x00},
	{0xc458, 0x00},
	{0xc459, 0x00},
	{0xc45b, 0x00},
	{0xc45c, 0x00},
	{0xc45d, 0x00},
	{0xc45e, 0x00},
	{0xc45f, 0x00},
	{0xc460, 0x00},
	{0xc461, 0x01},
	{0xc462, 0x01},
	{0xc464, 0x88},
	{0xc465, 0x00},
	{0xc466, 0x8a},
	{0xc467, 0x00},
	{0xc468, 0x86},
	{0xc469, 0x00},
	{0xc46a, 0x40},
	{0xc46b, 0x50},
	{0xc46c, 0x30},
	{0xc46d, 0x28},
	{0xc46e, 0x60},
	{0xc46f, 0x40},
	{0xc47c, 0x01},
	{0xc47d, 0x38},
	{0xc47e, 0x00},
	{0xc47f, 0x00},
	{0xc480, 0x00},
	{0xc481, 0xff},
	{0xc482, 0x00},
	{0xc483, 0x40},
	{0xc484, 0x00},
	{0xc485, 0x18},
	{0xc486, 0x00},
	{0xc487, 0x18},
	{0xc488, 0x2e},
	{0xc489, 0x40},
	{0xc48a, 0x2e},
	{0xc48b, 0x40},
	{0xc48c, 0x00},
	{0xc48d, 0x04},
	{0xc48e, 0x00},
	{0xc48f, 0x04},
	{0xc490, 0x07},
	{0xc492, 0x20},
	{0xc493, 0x08},
	{0xc498, 0x02},
	{0xc499, 0x00},
	{0xc49a, 0x02},
	{0xc49b, 0x00},
	{0xc49c, 0x02},
	{0xc49d, 0x00},
	{0xc49e, 0x02},
	{0xc49f, 0x60},
	{0xc4a0, 0x03},
	{0xc4a1, 0x00},
	{0xc4a2, 0x04},
	{0xc4a3, 0x00},
	{0xc4a4, 0x00},
	{0xc4a5, 0x10},
	{0xc4a6, 0x00},
	{0xc4a7, 0x40},
	{0xc4a8, 0x00},
	{0xc4a9, 0x80},
	{0xc4aa, 0x0d},
	{0xc4ab, 0x00},
	{0xc4ac, 0x0f},
	{0xc4ad, 0xc0},
	{0xc4b4, 0x01},
	{0xc4b5, 0x01},
	{0xc4b6, 0x00},
	{0xc4b7, 0x01},
	{0xc4b8, 0x00},
	{0xc4b9, 0x01},
	{0xc4ba, 0x01},
	{0xc4bb, 0x00},
	{0xc4bc, 0x01},
	{0xc4bd, 0x60},
	{0xc4be, 0x02},
	{0xc4bf, 0x33},
	{0xc4c8, 0x03},
	{0xc4c9, 0xd0},
	{0xc4ca, 0x0e},
	{0xc4cb, 0x00},
	{0xc4cc, 0x0e},
	{0xc4cd, 0x51},
	{0xc4ce, 0x0e},
	{0xc4cf, 0x51},
	{0xc4d0, 0x04},
	{0xc4d1, 0x80},
	{0xc4e0, 0x04},
	{0xc4e1, 0x02},
	{0xc4e2, 0x01},
	{0xc4e4, 0x10},
	{0xc4e5, 0x20},
	{0xc4e6, 0x30},
	{0xc4e7, 0x40},
	{0xc4e8, 0x50},
	{0xc4e9, 0x60},
	{0xc4ea, 0x70},
	{0xc4eb, 0x80},
	{0xc4ec, 0x90},
	{0xc4ed, 0xa0},
	{0xc4ee, 0xb0},
	{0xc4ef, 0xc0},
	{0xc4f0, 0xd0},
	{0xc4f1, 0xe0},
	{0xc4f2, 0xf0},
	{0xc4f3, 0x80},
	{0xc4f4, 0x00},
	{0xc4f5, 0x20},
	{0xc4f6, 0x02},
	{0xc4f7, 0x00},
	{0xc4f8, 0x04},
	{0xc4f9, 0x0b},
	{0xc4fa, 0x00},
	{0xc4fb, 0x00},
	{0xc4fc, 0x01},
	{0xc4fd, 0x00},
	{0xc4fe, 0x04},
	{0xc4ff, 0x02},
	{0xc500, 0x48},
	{0xc501, 0x74},
	{0xc502, 0x58},
	{0xc503, 0x80},
	{0xc504, 0x05},
	{0xc505, 0x80},
	{0xc506, 0x03},
	{0xc507, 0x80},
	{0xc508, 0x01},
	{0xc509, 0xc0},
	{0xc50a, 0x01},
	{0xc50b, 0xa0},
	{0xc50c, 0x01},
	{0xc50d, 0x2c},
	{0xc50e, 0x01},
	{0xc50f, 0x0a},
	{0xc510, 0x00},
	{0xc511, 0x00},
	{0xc512, 0xe5},
	{0xc513, 0x14},
	{0xc514, 0x04},
	{0xc515, 0x00},
	{0xc518, 0x03},
	{0xc519, 0x48},
	{0xc51a, 0x07},
	{0xc51b, 0x70},
	{0xc2e0, 0x00},
	{0xc2e1, 0x51},
	{0xc2e2, 0x00},
	{0xc2e3, 0xd6},
	{0xc2e4, 0x01},
	{0xc2e5, 0x5e},
	{0xc2e9, 0x01},
	{0xc2ea, 0x7a},
	{0xc2eb, 0x90},
	{0xc2ed, 0x00},
	{0xc2ee, 0x7a},
	{0xc2ef, 0x64},
	{0xc308, 0x00},
	{0xc309, 0x00},
	{0xc30a, 0x00},
	{0xc30c, 0x00},
	{0xc30d, 0x01},
	{0xc30e, 0x00},
	{0xc30f, 0x00},
	{0xc310, 0x01},
	{0xc311, 0x60},
	{0xc312, 0xff},
	{0xc313, 0x08},
	{0xc314, 0x01},
	{0xc315, 0x7f},
	{0xc316, 0xff},
	{0xc317, 0x0b},
	{0xc318, 0x00},
	{0xc319, 0x0c},
	{0xc31a, 0x00},
	{0xc31b, 0xe0},
	{0xc31c, 0x00},
	{0xc31d, 0x14},
	{0xc31e, 0x00},
	{0xc31f, 0x91},
	{0xc320, 0xff},
	{0xc321, 0x7a},
	{0xc322, 0xff},
	{0xc323, 0xf5},
	{0xc324, 0xff},
	{0xc325, 0xf3},
	{0xc326, 0x00},
	{0xc327, 0x40},
	{0xc328, 0xff},
	{0xc329, 0xce},
	{0xc32a, 0xff},
	{0xc32b, 0xe2},
	{0xc32c, 0xff},
	{0xc32d, 0xb6},
	{0xc32e, 0x00},
	{0xc32f, 0x67},
	{0xc330, 0xff},
	{0xc331, 0xf9},
	{0xc332, 0x00},
	{0xc333, 0xd9},
	{0xc334, 0x00},
	{0xc335, 0x2e},
	{0xc336, 0x00},
	{0xc337, 0xb1},
	{0xc338, 0xff},
	{0xc339, 0x64},
	{0xc33a, 0xff},
	{0xc33b, 0xeb},
	{0xc33c, 0xff},
	{0xc33d, 0xe8},
	{0xc33e, 0x00},
	{0xc33f, 0x48},
	{0xc340, 0xff},
	{0xc341, 0xd0},
	{0xc342, 0xff},
	{0xc343, 0xed},
	{0xc344, 0xff},
	{0xc345, 0xad},
	{0xc346, 0x00},
	{0xc347, 0x66},
	{0xc348, 0x01},
	{0xc349, 0x00},
	{0x6700, 0x04},
	{0x6701, 0x7b},
	{0x6702, 0xfd},
	{0x6703, 0xf9},
	{0x6704, 0x3d},
	{0x6705, 0x71},
	{0x6706, 0x78},
	{0x6708, 0x05},
	{0x6f06, 0x6f},
	{0x6f07, 0x00},
	{0x6f0a, 0x6f},
	{0x6f0b, 0x00},
	{0x6f00, 0x03},
	{0xc34c, 0x01},
	{0xc34d, 0x00},
	{0xc34e, 0x46},
	{0xc34f, 0x55},
	{0xc350, 0x00},
	{0xc351, 0x40},
	{0xc352, 0x00},
	{0xc353, 0xff},
	{0xc354, 0x04},
	{0xc355, 0x08},
	{0xc356, 0x01},
	{0xc357, 0xef},
	{0xc358, 0x30},
	{0xc359, 0x01},
	{0xc35a, 0x64},
	{0xc35b, 0x46},
	{0xc35c, 0x00},
	{0x3042, 0xf0},
	{0x301b, 0xf0},
	{0x301c, 0xf0},
	{0x301a, 0xf0},
	{0xceb0, 0x00},
	{0xceb1, 0x00},
	{0xceb2, 0x00},
	{0xceb3, 0x00},
	{0xceb4, 0x00},
	{0xceb5, 0x00},
	{0xceb6, 0x00},
	{0xceb7, 0x00},
	{0xc4bc, 0x01},
	{0xc4bd, 0x60},
	{0x5608, 0x0d},
	{0x5000, 0xff},
	{0x5120, 0x00},
	{0x5100, 0x01},
	{0x5101, 0x2d},
	{0x5102, 0x00},
	{0x5103, 0xaa},
	{0x5104, 0x00},
	{0x5105, 0xaa},
	{0x5106, 0x00},
	{0x5107, 0x80},
	{0x5110, 0x00},
	{0x5111, 0xff},
	{0x5112, 0x00},
	{0x5113, 0x97},
	{0x5114, 0x00},
	{0x5115, 0x97},
	{0x5116, 0x00},
	{0x5117, 0x80},
	{0x5cfc, 0x01},
	{0x5cfd, 0x2d},
	{0x5cfe, 0x00},
	{0x5cff, 0xaa},
	{0x5d00, 0x00},
	{0x5d01, 0xaa},
	{0x5d02, 0x00},
	{0x5d03, 0x80},
	{0x5d04, 0x00},
	{0x5d05, 0xff},
	{0x5d06, 0x00},
	{0x5d07, 0x97},
	{0x5d08, 0x00},
	{0x5d09, 0x97},
	{0x5d0a, 0x00},
	{0x5d0b, 0x80},
	{0xc2e6, 0x02},
	{0xc2e7, 0x00},
	{0xc2e8, 0x01},
	{0x5583, 0x10},
	{0x5596, 0x03},
	{0x5591, 0xfa},
	{0x5592, 0x02},
	{0x55aa, 0xfa},
	{0x55ab, 0x02},
	{0x5580, 0xff},
	{0x5581, 0x52},
	{0xc4cc, 0x10},
	{0xc4cd, 0x18},
	{0xc4ce, 0x10},
	{0xc4cf, 0x18},
	{0xc4b9, 0x01},
	{0xc2e9, 0x01},
	{0xc2ea, 0x7a},
	{0xc2eb, 0x90},
	{0xc4d2, 0x08},
	{0xc4b8, 0x01},
	{0xc4dc, 0x10},
	{0xc4dd, 0x20},
	{0xc4ba, 0x01},
	{0x5589, 0x72},
	{0x558a, 0x54},
	{0x5586, 0x12},
	{0x5587, 0x15},
	{0x5588, 0x15},
	{0x558b, 0x00},
	{0x558c, 0x00},
	{0x558d, 0x3f},
	{0x558e, 0x22},
	{0x55a2, 0x75},
	{0x55a3, 0x55},
	{0x559f, 0x12},
	{0x55a0, 0x14},
	{0x55a1, 0x15},
	{0x55a4, 0x00},
	{0x55a5, 0x00},
	{0x55a6, 0x43},
	{0x55a7, 0x21},
	{0xceb0, 0x00},
	{0xceb1, 0x00},
	{0xceb2, 0x00},
	{0xceb3, 0x00},
	{0xceb4, 0x00},
	{0xceb5, 0x00},
	{0xceb6, 0x00},
	{0xceb7, 0x00},
	{0x3832, 0x01},
	{0x3833, 0x1A},
	{0x3834, 0x03},
	{0x3835, 0x48},
	{0x302E, 0x01}, //fsin enable
	{0x301b, 0xf1},
	{0x3003, 0x28},
	{0x3004, 0x22},
	{0x3005, 0x24},
	{0x3006, 0x22},
	{0x3024, 0x00},
	{0x3621, 0x63},
	{0x3702, 0x1a},
	{0x3703, 0x3c},
	{0x3704, 0x2a},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x2c},
	{0x3804, 0x00},
	{0x3805, 0xff},
	{0x3806, 0x03},
	{0x3807, 0x01},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x02},
	{0x380b, 0xd0},
	{0x380c, 0x06},
	{0x380d, 0xf6},
	{0x6e42, 0x02},
	{0x6e43, 0xec},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x381c, 0x00},
	{0x381e, 0x00},
	{0x381f, 0x0c},
	{0x4001, 0x06},
	{0x4004, 0x04},
	{0x4050, 0x22},
	{0x4051, 0x24},
	{0x4605, 0x0c},
	{0x4606, 0x0c},
	{0x4607, 0x87},
	{0xc488, 0x2e},
	{0xc489, 0x40},
	{0xc48a, 0x2e},
	{0xc48b, 0x40},
	{0xc4cc, 0x10},
	{0xc4cd, 0x18},
	{0xc4ce, 0x10},
	{0xc4cf, 0x18},
	{0xc510, 0x00},
	{0xc511, 0x00},
	{0xc512, 0xe7},
	{0xc513, 0xe8},
	{0x5005, 0x08},
	{0x3007, 0x01},
	{0xc518, 0x02},
	{0xc519, 0xec},
	{0xc51a, 0x06},
	{0xc51b, 0xf6},
	{0x5608, 0x0e},
	{0x3815, 0x8c},
	{0x301b, 0xf0},
#ifdef TEST_PATTERN
	{0x503d, 0x80},
#endif
	{0x0100, 0x00},
};
static sensor_reg_array_t ov10635_dvp_yuv_start_reg_array[] = {
	{0x0100, 0x01},
};
static sensor_reg_array_t ov10635_dvp_yuv_stop_reg_array[] = {
	{0x0100, 0x00},
};

static sensor_settings_t ov10635_dvp_yuv_720p_setting = {
	.init_settings = {
		.reg_setting = ov10635_dvp_yuv_720p_reg_array,
		.size = sizeof(ov10635_dvp_yuv_720p_reg_array) / sizeof(sensor_reg_array_t),
		.delay = 50,
	},
};

static sensor_init_t ov10635_dvp_yuv_init[] = {
	{0, SENSOR_PIX_LEN_8, {1280, 720},  &ov10635_dvp_yuv_720p_setting},
};

sensor_info_t  ov10635_dvp_yuv_info  = {
	.name = (uint8_t *)"ov10635",
	.bus = SENSOR_BUS_DVP,
	.format = SENSOR_FMT_YUV,
	.chip = {
		.i2c_bus = 1,
		.slave_addr = OV10635_I2C_ADDR,
		.reg_size = SENSOR_I2C_WORD_DATA,
		.data_size = SENSOR_I2C_BYTE_DATA,
		.chip_id = OV10635_CHIP_INFO
	},
	.power = {
		.power_list = ov10635_dvp_yuv_power,
		.size = sizeof(ov10635_dvp_yuv_power) / sizeof(sensor_power_array_t),
	},
	.start = {
		.reg_setting = ov10635_dvp_yuv_start_reg_array,
		.size = sizeof(ov10635_dvp_yuv_start_reg_array) / sizeof(sensor_reg_array_t),
		.delay = 50,
	},
	.stop = {
		.reg_setting = ov10635_dvp_yuv_stop_reg_array,
		.size = sizeof(ov10635_dvp_yuv_stop_reg_array) / sizeof(sensor_reg_array_t),
		.delay = 50,
	},
	.init = ov10635_dvp_yuv_init,
	.size = sizeof(ov10635_dvp_yuv_init) / sizeof(sensor_init_t),
};
