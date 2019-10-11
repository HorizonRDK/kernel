#ifndef __IMX_290_SETTING_H__
#define __IMX_290_SETTING_H__


#ifdef __cplusplus
extern "C" {
#endif

static uint16_t imx290_raw12_normal_setting[] = {
	0x3000, 0x01,	// Standby
	0x3003, 0x01,	// SW_RESET
	0x3002, 0x00,	// Master Mode
	0x3005, 0x01,	// [0:0] 0:AD-10bit		1:AD-12bit
	0x3007, 0x00,	// [7:4] 0:FHD1080, 1:HD720, 4: Crop from FHD
	0x3009, 0x02,	// [1:0] FRSEL, [4:4] FDG_SEL
	0x300a, 0xf0,	// [7:0] BlkLevel
	0x300f, 0x00,	// Set to 0x00
	0x3010, 0x21,	// Set to 0x21
	0x3012, 0x64,	// Set to 0x64
	0x3016, 0x09,	// Set to 0x09
	0x3018,	0x65,	// Pair of 0x3019
	0x3019,	0x0e,	// VMAX[17:0], 0x0465 = 1125
	0x301c, 0xA0,	// Pair of 301d
	0x301d, 0x3c,	// HMAX[15:9], <<1080p>> 0x1130:30FPS, 0x14A0:25FPS
	0x3046, 0x01,	// ODBIT[0:0] 0:10bit, 1:12bit  [7:4] LVDS, MIPI don't care
	0x305c, 0x18,	// INCKSEL1 ?
	0x305d, 0x03,	// INCKSEL2 ?
	0x305e, 0x20,	// INCKSEL3 ?
	0x305f, 0x01,	// INCKSEL4 ?
	0x3070, 0x02,	// Set to 0x02
	0x3071, 0x11,	// Set to 0x11
	0x309b, 0x10,	// Set to 0x10
	0x309c, 0x22,	// Set to 0x22
	0x30a2, 0x02,	// Set to 0x02
	0x30a6,	0x20,	// Set to 0x20
	0x30a8, 0x20,	// Set to 0x20
	0x30aa, 0x20,	// Set to 0x20
	0x30ac, 0x20,	// Set to 0x20
	0x30b0, 0x43,	// Set to 0x43
	0x3119, 0x9E,	// Set to 0x9E
	0x311c, 0x1E,	// Set to 0x1E
	0x311e, 0x08,	// Set to 0x08
	0x3128, 0x05,	// Set to 0x05
	0x3129, 0x00,	// ADBIT1[7:0] 0x00:12bit, 0x1d:10bit
	0x313d, 0x83,	// Set to 0x83
	0x3150, 0x03,	// Set to 0x03
	0x315e, 0x1a,	// INCKSEL5 0x1a:37.125MHz, 0x1b:74.25MHz
	0x3164, 0x1a,	// INCKSEL6 0x1a:37.125MHz, 0x1b:74.25MHz
	0x317c, 0x00,	// ADBIT2[7:0] 0x00:12bit, 0x12:10bit
	0x317e, 0x00,	// Set to 0x00
	0x31ec, 0x0E,	// ADBIT3[7:0] 0x0E:12bit, 0x37:10bit
	0x32b8, 0x50,	// Set to 0x50
	0x32b9, 0x10,	// Set to 0x10
	0x32ba, 0x00,	// Set to 0x00
	0x32bb, 0x04,	// Set to 0x04
	0x32c8, 0x50,	// Set to 0x50
	0x32c9, 0x10,	// Set to 0x10
	0x32ca, 0x00,	// Set to 0x00
	0x32cb, 0x04,	// Set to 0x04
	0x332c, 0xd3,	// Set to 0xd3
	0x332d, 0x10,	// Set to 0x10
	0x332e, 0x0d,	// Set to 0x0d
	0x3358, 0x06,	// Set to 0x06
	0x3359, 0xe1,	// Set to 0xe1
	0x335a, 0x11,	// Set to 0x11
	0x3360, 0x1e,	// Set to 0x1e
	0x3361, 0x61,	// Set to 0x61
	0x3362, 0x10,	// Set to 0x10
	0x33b0, 0x50,	// Set to 0x50
	0x33b2, 0x1a,	// Set to 0x1a
	0x33b3, 0x04,	// Set to 0x04
	0x3405, 0x20,	// REPRTITIOM[5:4] Ref to Output Signal Interface Control ??
	0x3407, 0x03,	// PHY_LANE_NUM[1:0]
	0x3414, 0x0a,	// OPB_SIZE_V[5:0]
	0x3418, 0x38,	// Pair or 0x3419 
	0x3419, 0x04,	// Y_OUT_SIZE[12:0],  0x449 = 1097
	0x3441, 0x0c,	// Pair of 0x3442
	0x3442, 0x0c,	// CSI_DT_FMT[15:0] 0x0A0A:RAW10, 0x0C0C:RAW12
	0x3443, 0x03,	// CSI_LANE_MODE[1:0] 0:x, 1:2Lane, 3:x, 3:4Lane
	0x3444, 0x20,	// Pair of 0x3445
	0x3445, 0x25,	// EXTCK_FREQ[15:0] 0x2520:37.125MHz, 0x4A40:74.25MHz
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x1f,
	0x3449, 0x00,
	0x344a, 0x17,
	0x344b, 0x00,
	0x344c, 0x0f,
	0x344d, 0x00,
	0x344e, 0x17,
	0x344f, 0x00,
	0x3450, 0x47,
	0x3451, 0x00,
	0x3452, 0x0f,
	0x3453, 0x00,
	0x3454, 0x0f,
	0x3455, 0x00,
	0x3472, 0x80,	// Pair od 0x3473
	0x3473, 0x07,	// X_OUT_SIZE[12:0]	0x07a0 = 1952
	0x3480, 0x49,	// INCKSEL7[7:0] 0x49:37.125MHz, 0x92:74.25MHz
};

static uint16_t imx290_raw12_dol2_setting[] = {
	0x3000, 0x01,	// Standby
	0x3003, 0x01,	// SW_RESET
	0x3002, 0x00,	// Master Mode
	0x3005, 0x01,	// [0:0] 0:AD-10bit		1:AD-12bit
	0x3007, 0x00,	// [7:4] 0:FHD1080, 1:HD720, 4: Crop from FHD
	0x3009, 0x02,	// [1:0] FRSEL, [4:4] FDG_SEL
	0x300a, 0xf0,	// [7:0] BlkLevel
	0x300c, 0x11,	// WDMODE, [0:0] 0: Normal, 1: DOL Mode; [4:5] 0: x, 1: DOL-2 2: DOL-3
	0x300f, 0x00,	// Set to 0x00
	0x3010, 0x21,	// Set to 0x21
	0x3012, 0x64,	// Set to 0x64
	0x3016, 0x09,	// Set to 0x09
	0x3018,	0x65,	// Pair of 0x3019
	0x3019,	0x0E,	// VMAX[17:0], 0x0465 = 1125
	0x301c, 0xA0,	// Pair of 301d
	0x301d, 0x14,	// HMAX[15:9], <<1080p>> 0x1130:30FPS, 0x14A0:25FPS
	0x3020, 0x02,	// SHS1
	0x3021, 0x00,	// SHS1
	0x3022, 0x00,	// SHS1 = 0x000002
	0x3024, 0x49,	// SHS2
	0x3025, 0x08,	// SHS2
	0x3026, 0x00,	// SHS2 = 0x000849	(FAE Version 0x0007C9)
	0x3030, 0x0B,	// RHS1
	0x3031, 0x00,	// RHS1
	0x3032, 0x00,	// RHS1 = 0x00000B
	0x3045, 0x05,	// DOLSCDEN, ** PATTERN 1**
	0x3046, 0x01,	// ODBIT[0:0] 0:10bit, 1:12bit  [7:4] LVDS, MIPI don't care
	0x305c, 0x18,	// INCKSEL1 ?
	0x305d, 0x03,	// INCKSEL2 ?
	0x305e, 0x20,	// INCKSEL3 ?
	0x305f, 0x01,	// INCKSEL4 ?
	0x3070, 0x02,	// Set to 0x02
	0x3071, 0x11,	// Set to 0x11
	0x309b, 0x10,	// Set to 0x10
	0x309c, 0x22,	// Set to 0x22
	0x30a2, 0x02,	// Set to 0x02
	0x30a6,	0x20,	// Set to 0x20
	0x30a8, 0x20,	// Set to 0x20
	0x30aa, 0x20,	// Set to 0x20
	0x30ac, 0x20,	// Set to 0x20
	0x30b0, 0x43,	// Set to 0x43
	0x3119, 0x9E,	// Set to 0x9E
	0x311c, 0x1E,	// Set to 0x1E
	0x311e, 0x08,	// Set to 0x08
	0x3128, 0x05,	// Set to 0x05
	0x3129, 0x00,	// ADBIT1[7:0] 0x00:12bit, 0x1d:10bit
	0x313d, 0x83,	// Set to 0x83
	0x3150, 0x03,	// Set to 0x03
	0x315e, 0x1a,	// INCKSEL5 0x1a:37.125MHz, 0x1b:74.25MHz
	0x3164, 0x1a,	// INCKSEL6 0x1a:37.125MHz, 0x1b:74.25MHz
	0x317c, 0x00,	// ADBIT2[7:0] 0x00:12bit, 0x12:10bit
	0x317e, 0x00,	// Set to 0x00
	0x31ec, 0x0E,	// ADBIT3[7:0] 0x0E:12bit, 0x37:10bit
	0x32b8, 0x50,	// Set to 0x50
	0x32b9, 0x10,	// Set to 0x10
	0x32ba, 0x00,	// Set to 0x00
	0x32bb, 0x04,	// Set to 0x04
	0x32c8, 0x50,	// Set to 0x50
	0x32c9, 0x10,	// Set to 0x10
	0x32ca, 0x00,	// Set to 0x00
	0x32cb, 0x04,	// Set to 0x04
	0x332c, 0xd3,	// Set to 0xd3
	0x332d, 0x10,	// Set to 0x10
	0x332e, 0x0d,	// Set to 0x0d
	0x3358, 0x06,	// Set to 0x06
	0x3359, 0xe1,	// Set to 0xe1
	0x335a, 0x11,	// Set to 0x11
	0x3360, 0x1e,	// Set to 0x1e
	0x3361, 0x61,	// Set to 0x61
	0x3362, 0x10,	// Set to 0x10
	0x33b0, 0x50,	// Set to 0x50
	0x33b2, 0x1a,	// Set to 0x1a
	0x33b3, 0x04,	// Set to 0x04
	0x3405, 0x20,	// REPRTITIOM[5:4] Ref to Output Signal Interface Control ??
	0x3407, 0x03,	// PHY_LANE_NUM[1:0]
	0x3414, 0x0a,	// OPB_SIZE_V[5:0]
	0x3418, 0x9C,	// Pair or 0x3419 
	0x3419, 0x08,	// Y_OUT_SIZE[12:0],  0x89c = 2204
	0x3441, 0x0c,	// Pair of 0x3442
	0x3442, 0x0c,	// CSI_DT_FMT[15:0] 0x0A0A:RAW10, 0x0C0C:RAW12
	0x3443, 0x03,	// CSI_LANE_MODE[1:0] 0:x, 1:2Lane, 3:x, 3:4Lane
	0x3444, 0x20,	// Pair of 0x3445
	0x3445, 0x25,	// EXTCK_FREQ[15:0] 0x2520:37.125MHz, 0x4A40:74.25MHz
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x1f,
	0x3449, 0x00,
	0x344a, 0x17,
	0x344b, 0x00,
	0x344c, 0x0f,
	0x344d, 0x00,
	0x344e, 0x17,
	0x344f, 0x00,
	0x3450, 0x47,
	0x3451, 0x00,
	0x3452, 0x0f,
	0x3453, 0x00,
	0x3454, 0x0f,
	0x3455, 0x00,
	0x3472, 0xA0,	// Pair od 0x3473
	0x3473, 0x07,	// X_OUT_SIZE[12:0]	0x07A0 = 1952
	0x347b, 0x23,	// MIF_SYNC_TIME0 0x24: Default, 0x23: DOL Mode
	0x3480, 0x49,	// INCKSEL7[7:0] 0x49:37.125MHz, 0x92:74.25MHz      
};

static uint16_t imx290_raw12_dol3_setting[] = {
	0x3000, 0x01,	// Standby
	0x3003, 0x01,	// SW_RESET
	0x3002, 0x00,	// Master Mode
	0x3005, 0x01,	// [0:0] 0:AD-10bit		1:AD-12bit
	0x3007, 0x00,	// [7:4] 0:FHD1080, 1:HD720, 4: Crop from FHD
	0x3009, 0x02,	// [1:0] FRSEL, [4:4] FDG_SEL
	0x300a, 0xf0,	// [7:0] BlkLevel
	0x300c, 0x21,	// WDMODE, [0:0] 0: Normal, 1: DOL Mode; [4:5] 0: x, 1: DOL-2 2: DOL-3
	0x300f, 0x00,	// Set to 0x00
	0x3010, 0x21,	// Set to 0x21
	0x3012, 0x64,	// Set to 0x64
	0x3016, 0x09,	// Set to 0x09
	0x3018,	0x65,	// Pair of 0x3019
	0x3019,	0x30,	// VMAX[17:0], 0x0465 = 1125
	0x301c, 0xA0,	// Pair of 301d
	0x301d, 0x14,	// HMAX[15:9], <<1080p>> 0x1130:30FPS, 0x14A0:25FPS
	0x3020, 0x05,	// SHS1
	0x3021, 0x00,	// SHS1
	0x3022, 0x00,	// SHS1 = 0x000005
	0x3024, 0x0B,	// SHS2
	0x3025, 0x01,	// SHS2
	0x3026, 0x00,	// SHS2 = 0x00010B
	0x3028, 0x93,	// SHS3
	0x3029, 0x01,	// SHS3
	0x302a, 0x00,	// SHS3 = 0x000193
	0x3030, 0x06,	// RHS1
	0x3031, 0x01,	// RHS1
	0x3032, 0x00,	// RHS1 = 0x000106
	0x3034, 0x1c,	// RHS2
	0x3035, 0x01,	// RHS2
	0x3036, 0x00,	// RHS2 = 0x00011C
	0x3045, 0x05,	// DOLSCDEN, ** PATTERN 1**
	0x3046, 0x01,	// ODBIT[0:0] 0:10bit, 1:12bit  [7:4] LVDS, MIPI don't care
	0x305c, 0x18,	// INCKSEL1 ?
	0x305d, 0x03,	// INCKSEL2 ?
	0x305e, 0x20,	// INCKSEL3 ?
	0x305f, 0x01,	// INCKSEL4 ?
	0x3070, 0x02,	// Set to 0x02
	0x3071, 0x11,	// Set to 0x11
	0x309b, 0x10,	// Set to 0x10
	0x309c, 0x22,	// Set to 0x22
	0x30a2, 0x02,	// Set to 0x02
	0x30a6,	0x20,	// Set to 0x20
	0x30a8, 0x20,	// Set to 0x20
	0x30aa, 0x20,	// Set to 0x20
	0x30ac, 0x20,	// Set to 0x20
	0x30b0, 0x43,	// Set to 0x43
	0x3119, 0x9E,	// Set to 0x9E
	0x311c, 0x1E,	// Set to 0x1E
	0x311e, 0x08,	// Set to 0x08
	0x3128, 0x05,	// Set to 0x05
	0x3129, 0x00,	// ADBIT1[7:0] 0x00:12bit, 0x1d:10bit
	0x313d, 0x83,	// Set to 0x83
	0x3150, 0x03,	// Set to 0x03
	0x315e, 0x1a,	// INCKSEL5 0x1a:37.125MHz, 0x1b:74.25MHz
	0x3164, 0x1a,	// INCKSEL6 0x1a:37.125MHz, 0x1b:74.25MHz
	0x317c, 0x00,	// ADBIT2[7:0] 0x00:12bit, 0x12:10bit
	0x317e, 0x00,	// Set to 0x00
	0x31ec, 0x0E,	// ADBIT3[7:0] 0x0E:12bit, 0x37:10bit
	0x32b8, 0x50,	// Set to 0x50
	0x32b9, 0x10,	// Set to 0x10
	0x32ba, 0x00,	// Set to 0x00
	0x32bb, 0x04,	// Set to 0x04
	0x32c8, 0x50,	// Set to 0x50
	0x32c9, 0x10,	// Set to 0x10
	0x32ca, 0x00,	// Set to 0x00
	0x32cb, 0x04,	// Set to 0x04
	0x332c, 0xd3,	// Set to 0xd3
	0x332d, 0x10,	// Set to 0x10
	0x332e, 0x0d,	// Set to 0x0d
	0x3358, 0x06,	// Set to 0x06
	0x3359, 0xe1,	// Set to 0xe1
	0x335a, 0x11,	// Set to 0x11
	0x3360, 0x1e,	// Set to 0x1e
	0x3361, 0x61,	// Set to 0x61
	0x3362, 0x10,	// Set to 0x10
	0x33b0, 0x50,	// Set to 0x50
	0x33b2, 0x1a,	// Set to 0x1a
	0x33b3, 0x04,	// Set to 0x04
	0x3405, 0x20,	// REPRTITIOM[5:4] Ref to Output Signal Interface Control ??
	0x3407, 0x03,	// PHY_LANE_NUM[1:0]
	0x3414, 0x00,	// OPB_SIZE_V[5:0]
	0x3415, 0x00,
	0x3418, 0x55,	// Pair or 0x3419 
	0x3419, 0x11,	// Y_OUT_SIZE[12:0],  0x1155 = 4437
	0x3441, 0x0c,	// Pair of 0x3442
	0x3442, 0x0c,	// CSI_DT_FMT[15:0] 0x0A0A:RAW10, 0x0C0C:RAW12
	0x3443, 0x03,	// CSI_LANE_MODE[1:0] 0:x, 1:2Lane, 3:x, 3:4Lane
	0x3444, 0x20,	// Pair of 0x3445
	0x3445, 0x25,	// EXTCK_FREQ[15:0] 0x2520:37.125MHz, 0x4A40:74.25MHz
	0x3446, 0x47,
	0x3447, 0x00,
	0x3448, 0x1f,
	0x3449, 0x00,
	0x344a, 0x17,
	0x344b, 0x00,
	0x344c, 0x0f,
	0x344d, 0x00,
	0x344e, 0x17,
	0x344f, 0x00,
	0x3450, 0x47,
	0x3451, 0x00,
	0x3452, 0x0f,
	0x3453, 0x00,
	0x3454, 0x0f,
	0x3455, 0x00,
	0x3472, 0xA0,	// Pair od 0x3473
	0x3473, 0x07,	// X_OUT_SIZE[12:0]	0x07A0 = 1952
	0x347b, 0x23,	// MIF_SYNC_TIME0 0x24: Default, 0x23: DOL Mode
	0x3480, 0x49,	// INCKSEL7[7:0] 0x49:37.125MHz, 0x92:74.25MHz
};

#ifdef __cplusplus
}
#endif
#endif

