/**
 *  The confidential and proprietary information contained in this file may
 *   only be used by a person authorised under and to the extent permitted
 *  by a subsisting licensing agreement from ARM Limited or its affiliates.
 *
 *        (C) COPYRIGHT 2019 ARM Limited or its affiliates
 *                       ALL RIGHTS RESERVED
 *
 *       This entire notice must be reproduced on all copies of this file
 *   and copies of this file may only be made by a person if such person is
 *    permitted to do so under the terms of a subsisting license agreement
 *                  from ARM Limited or its affiliates.
 *
 */

#include "acamera_command_api.h"
#include "acamera_firmware_settings.h"

// created from 2019-08-07T11:22:43.539Z UTCdynamic-calibrations(linear isp)-imx290-20190807.json

// CALIBRATION_SCALER_H_FILTER (1x256 4 bytes)
static uint32_t _calibration_scaler_h_filter[]
 = {670499328,194343,704053760,194596,754385408,194593,787939840,194846,821494272,194844,871825920,129561,905380352,129559,938934784,129812,972489216,130065,989331968,130063,1006174720,130316,1023017472,130314,1056637184,65032,1056702720,65286,1073545472,65284,1090387968,2,1073741824,0,1073872896,254,1057292032,509,1040645888,508,1040776704,507,1007353089,762,990706945,761,974126081,760,957479937,759,924121857,759,890763777,759,857340417,759,807205122,759,773781762,759,740423682,759,690288642,759,637271301,16644902,670891013,16710180,687733765,16709923,704576261,16775457,704641798,16775200,704707078,63518,721549830,63261,721615110,128796,738457862,128794,755300358,194073,755431430,194071,755496966,259605,789116677,259604,789182213,325138,789313285,325136,789444101,325135,789444101,390670,789575172,390669,789640708,390923,789771780,390921,789902851,390920,756413955,456711,756545026,456710,756676098,456708,739964673,456963,723318529,456962,723384064,457217,706672640,457216,690026751,457727,690092543,392190,673446398,392445,656734974,392444,504036092,16386590,504101628,16386334,504167419,16386333,520944635,16386077,537787387,16320285,537852923,16320284,537853179,16320028,554695930,16319772,554761466,16319771,554827258,16319514,571604474,16319258,571670266,16319256,571735802,16319000,571736057,16319000,571867129,16318743,571867129,16318743,571867385,16318486,571933177,16383765,571933177,16383765,571998969,16383763,571998969,16449043,571999225,16449042,572130297,16448785,555353337,16448785,555419129,16448528,555484665,16448527,538707705,16514062,538707961,16513806,538773497,16513805,521996538,16513804,505219578,16513804,505285114,16579083,235603458,396046,235603714,396045,235603714,396045,235603714,396045,235669250,395789,235669250,395789,235669251,330253,235669251,330253,235669251,330253,235669507,329997,235669507,329997,235669507,329997,235669507,329997,235669508,264461,235669508,264461,235669764,264460,235669764,264460,235669764,264460,235735300,264204,235735300,264204,235735301,198668,235735301,198668,235735301,198668,235735301,198668,235735557,198412,235735557,198412,235735557,198412,235735558,132876,235735558,132876,235735814,132875,235735814,132875,235735814,132875};

// CALIBRATION_AWB_ZONE_WGHT_VER (1x32 2 bytes)
static uint16_t _calibration_awb_zone_wght_ver[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_AWB_BG_MAX_GAIN (3x2 2 bytes)
static uint16_t _calibration_awb_bg_max_gain[][2]
 =  {
  { 0, 100 },
  { 256, 100 },
  { 1792, 200 }
};

// CALIBRATION_IRIDIX_STRENGTH_MAXIMUM (1x1 1 bytes)
static uint8_t _calibration_iridix_strength_maximum[]
 = {255};

// CALIBRATION_SINTER_RADIAL_PARAMS (1x4 2 bytes)
static uint16_t _calibration_sinter_radial_params[]
 = {0,960,540,1770};

// CALIBRATION_SHARPEN_FR (7x2 2 bytes)
static uint16_t _calibration_sharpen_fr[][2]
 =  {
  { 0, 42 },
  { 256, 27 },
  { 512, 20 },
  { 768, 10 },
  { 1024, 8 },
  { 1280, 2 },
  { 1536, 1 }
};

// CALIBRATION_PF_RADIAL_PARAMS (1x3 2 bytes)
static uint16_t _calibration_pf_radial_params[]
 = {960,540,1770};

// CALIBRATION_SINTER_STRENGTH_MC_CONTRAST (1x2 2 bytes)
static uint16_t _calibration_sinter_strength_mc_contrast[]
 = {0,0};

// CALIBRATION_CMOS_EXPOSURE_PARTITION_LUTS (2x10 2 bytes)
static uint16_t _calibration_cmos_exposure_partition_luts[][10]
 =  {
  { 1, 1, 2, 2, 4, 4, 10, 32, 20, 128 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// CALIBRATION_SHARP_ALT_UD (8x2 2 bytes)
static uint16_t _calibration_sharp_alt_ud[][2]
 =  {
  { 0, 10 },
  { 256, 10 },
  { 512, 10 },
  { 768, 8 },
  { 1024, 8 },
  { 1280, 2 },
  { 1536, 0 },
  { 1792, 0 }
};

// CALIBRATION_MESH_SHADING_STRENGTH (1x2 2 bytes)
static uint16_t _calibration_mesh_shading_strength[]
 = {0,4096};

// CALIBRATION_AF_LMS (1x21 4 bytes)
static uint32_t _calibration_af_lms[]
 = {4480,4480,4480,7168,7168,7168,53248,53248,53248,58560,58560,58560,11,6,2,30,131072,131072,262144,65536,0};

// CALIBRATION_DP_SLOPE (7x2 2 bytes)
static uint16_t _calibration_dp_slope[][2]
 =  {
  { 0, 170 },
  { 256, 170 },
  { 512, 170 },
  { 768, 1800 },
  { 1024, 1911 },
  { 1280, 2200 },
  { 1536, 2400 }
};

// CALIBRATION_SHARPEN_DS1 (9x2 2 bytes)
static uint16_t _calibration_sharpen_ds1[][2]
 =  {
  { 0, 70 },
  { 256, 70 },
  { 512, 70 },
  { 768, 70 },
  { 1024, 70 },
  { 1280, 50 },
  { 1536, 40 },
  { 1792, 25 },
  { 2048, 10 }
};

// CALIBRATION_SINTER_INTCONFIG (7x2 2 bytes)
static uint16_t _calibration_sinter_intconfig[][2]
 =  {
  { 0, 10 },
  { 256, 10 },
  { 512, 8 },
  { 768, 8 },
  { 1024, 7 },
  { 1280, 5 },
  { 1536, 4 }
};

// CALIBRATION_CNR_UV_DELTA12_SLOPE (8x2 2 bytes)
static uint16_t _calibration_cnr_uv_delta12_slope[][2]
 =  {
  { 0, 1500 },
  { 256, 2000 },
  { 512, 2100 },
  { 768, 2100 },
  { 1024, 3500 },
  { 1280, 4100 },
  { 1536, 5900 },
  { 1792, 6100 }
};

// CALIBRATION_RGB2YUV_CONVERSION (1x12 2 bytes)
static uint16_t _calibration_rgb2yuv_conversion[]
 = {76,150,29,32811,32852,128,128,32875,32788,0,512,512};

// CALIBRATION_STITCHING_MS_MOV_MULT (3x2 2 bytes)
static uint16_t _calibration_stitching_ms_mov_mult[][2]
 =  {
  { 0, 128 },
  { 256, 128 },
  { 512, 100 }
};

// CALIBRATION_AWB_AVG_COEF (1x1 1 bytes)
static uint8_t _calibration_awb_avg_coef[]
 = {7};

// CALIBRATION_AE_CONTROL (1x9 4 bytes)
static uint32_t _calibration_ae_control[]
 = {30,236,0,0,0,100,99,1,2};

// CALIBRATION_AWB_ZONE_WGHT_HOR (1x32 2 bytes)
static uint16_t _calibration_awb_zone_wght_hor[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_CUSTOM_SETTINGS_CONTEXT (1x4 4 bytes)
static uint32_t _calibration_custom_settings_context[]
 = {0,0,0,0};

static uint32_t _calibration_temper_threshold[] = {100000, 2000000};

 static uint8_t _calibration_user_temper_noise_lut[] =
   {0,0,0,0,0,0,0,1,25,36,41,45,47,50,51,53,54,56,57,58,59,60,60,61,62,63,63,63,64,65,65,66,66,67,67,68,68,68,69,69,69,70,70,70,71,71,71,72,72,72,73,73,73,73,74,74,74,74,74,75,75,75,75,76,76,76,76,76,77,77,77,77,77,77,78,78,78,78,78,78,79,79,79,79,79,79,80,80,80,80,80,80,80,81,81,81,81,81,81,81,81,82,82,82,82,82,82,82,82,82,83,83,83,83,83,83,83,83,83,84,84,84,84,84,84,84,84,84};

static uint8_t _calibration_user_temper_noise_lut_1[] =
  {0,0,0,0,0,0,0,1,25,36,41,45,47,50,51,53,54,56,57,58,59,60,60,61,62,63,63,63,64,65,65,66,66,67,67,68,68,68,69,69,69,70,70,70,71,71,71,72,72,72,73,73,73,73,74,74,74,74,74,75,75,75,75,76,76,76,76,76,77,77,77,77,77,77,78,78,78,78,78,78,79,79,79,79,79,79,80,80,80,80,80,80,80,81,81,81,81,81,81,81,81,82,82,82,82,82,82,82,82,82,83,83,83,83,83,83,83,83,83,84,84,84,84,84,84,84,84,84};

static uint8_t _calibration_user_temper_noise_lut_2[] =
  {0,0,0,0,0,0,0,1,25,36,41,45,47,50,51,53,54,56,57,58,59,60,60,61,62,63,63,63,64,65,65,66,66,67,67,68,68,68,69,69,69,70,70,70,71,71,71,72,72,72,73,73,73,73,74,74,74,74,74,75,75,75,75,76,76,76,76,76,77,77,77,77,77,77,78,78,78,78,78,78,79,79,79,79,79,79,80,80,80,80,80,80,80,81,81,81,81,81,81,81,81,82,82,82,82,82,82,82,82,82,83,83,83,83,83,83,83,83,83,84,84,84,84,84,84,84,84,84};

 static uint8_t _calibration_user_sinter_lut[] =
  {0,0,0,0,0,0,0,17,45,51,55,58,61,63,65,66,67,68,70,71,72,72,73,73,75,75,75,76,77,77,78,78,79,79,80,80,81,81,81,82,82,82,83,83,83,84,84,84,84,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85};

// CALIBRATION_GAMMA_EV1
static uint16_t _calibration_gamma_ev1[] =
    /*sRGB highcontrast{0, 150, 261, 359, 452, 541, 623, 702, 781, 859, 937, 1014, 1087, 1158, 1224, 1288, 1348, 1407, 1464, 1519, 1572, 1625, 1676, 1727, 1775, 1823, 1869, 1913, 1956, 1999, 2041, 2082, 2123, 2162, 2201, 2238, 2276, 2312, 2348, 2383, 2417, 2451, 2485, 2516, 2549, 2580, 2611, 2641, 2671, 2701, 2730, 2759, 2787, 2816, 2843, 2871, 2897, 2923, 2950, 2975, 3000, 3025, 3048, 3071, 3095,3118, 3139, 3161, 3182, 3204, 3224, 3244, 3263, 3283, 3302, 3322, 3340, 3358, 3377, 3394, 3412, 3429, 3447, 3464, 3481, 3497, 3514, 3530, 3546, 3562, 3579, 3594, 3610, 3625, 3641, 3656, 3671, 3686, 3701, 3716, 3731, 3745, 3759, 3774, 3788, 3802, 3816, 3830, 3843, 3857, 3871, 3884, 3898, 3911, 3924, 3936, 3949, 3962, 3974, 3987, 4000, 4011, 4024, 4036, 4048, 4060, 4072, 4083, 4095}; */
    /*sRGB 65{0,192,318,419,511,596,675,749,820,887,950,1012,1070,1126,1180,1231,1282,1332,1380,1428,1475,1521,1568,1614,1660,1706,1751,1796,1842,1890,1938,1988,2037,2085,2133,2180,2228,2273,2319,2363,2406,2447,2489,2528,2566,2603,2638,2671,2703,2734,2762,2790,2818,2845,2871,2897,2921,2946,2970,2993,3016,3038,3060,3081,3103,3123,3143,3163,3183,3203,3222,3241,3259,3278,3296,3315,3333,3351,3369,3386,3403,3420,3438,3455,3472,3489,3506,3522,3539,3555,3572,3588,3604,3620,3635,3651,3666,3681,3696,3712,3726,3741,3755,3770,3784,3798,3813,3827,3840,3854,3868,3881,3895,3908,3921,3934,3947,3960,3972,3985,3998,4010,4023,4035,4048,4060,4071,4083,4095}; */
    {0, 250, 406, 527, 630, 723, 807, 889, 969, 1045, 1120, 1192, 1260, 1327, 1391, 1453, 1512, 1570, 1625, 1677, 1728, 1777, 1824, 1870, 1913, 1955, 1995, 2033, 2069, 2105, 2140, 2174, 2207, 2238, 2270, 2300, 2330, 2359, 2387, 2415, 2443, 2469, 2496, 2522, 2548, 2573, 2598, 2622, 2646, 2671, 2694, 2717, 2740, 2763, 2786, 2809, 2830, 2852, 2875, 2896, 2918, 2939, 2960, 2981, 3002, 3023, 3043, 3064, 3084, 3105, 3125, 3145, 3164, 3184, 3203, 3223, 3242, 3262, 3281, 3299, 3318, 3336, 3355, 3373, 3391, 3409, 3427, 3445, 3463, 3480, 3498, 3515, 3532, 3549, 3567, 3584, 3600, 3617, 3633, 3650, 3667,3683, 3699, 3715, 3731, 3748, 3764, 3780, 3795, 3811, 3827, 3842, 3858, 3873, 3888, 3904, 3919, 3934, 3948, 3964, 3979, 3993, 4008, 4023, 4038, 4052, 4066, 4081, 4095};


// CALIBRATION_GAMMA_EV2
static uint16_t _calibration_gamma_ev2[] =
    /*sRGB highcontrast{0, 150, 261, 359, 452, 541, 623, 702, 781, 859, 937, 1014, 1087, 1158, 1224, 1288, 1348, 1407, 1464, 1519, 1572, 1625, 1676, 1727, 1775, 1823, 1869, 1913, 1956, 1999, 2041, 2082, 2123, 2162, 2201, 2238, 2276, 2312, 2348, 2383, 2417, 2451, 2485, 2516, 2549, 2580, 2611, 2641, 2671, 2701, 2730, 2759, 2787, 2816, 2843, 2871, 2897, 2923, 2950, 2975, 3000, 3025, 3048, 3071, 3095,3118, 3139, 3161, 3182, 3204, 3224, 3244, 3263, 3283, 3302, 3322, 3340, 3358, 3377, 3394, 3412, 3429, 3447, 3464, 3481, 3497, 3514, 3530, 3546, 3562, 3579, 3594, 3610, 3625, 3641, 3656, 3671, 3686, 3701, 3716, 3731, 3745, 3759, 3774, 3788, 3802, 3816, 3830, 3843, 3857, 3871, 3884, 3898, 3911, 3924, 3936, 3949, 3962, 3974, 3987, 4000, 4011, 4024, 4036, 4048, 4060, 4072, 4083, 4095}; */
    /*sRGB 65{0,192,318,419,511,596,675,749,820,887,950,1012,1070,1126,1180,1231,1282,1332,1380,1428,1475,1521,1568,1614,1660,1706,1751,1796,1842,1890,1938,1988,2037,2085,2133,2180,2228,2273,2319,2363,2406,2447,2489,2528,2566,2603,2638,2671,2703,2734,2762,2790,2818,2845,2871,2897,2921,2946,2970,2993,3016,3038,3060,3081,3103,3123,3143,3163,3183,3203,3222,3241,3259,3278,3296,3315,3333,3351,3369,3386,3403,3420,3438,3455,3472,3489,3506,3522,3539,3555,3572,3588,3604,3620,3635,3651,3666,3681,3696,3712,3726,3741,3755,3770,3784,3798,3813,3827,3840,3854,3868,3881,3895,3908,3921,3934,3947,3960,3972,3985,3998,4010,4023,4035,4048,4060,4071,4083,4095}; */
    {0, 250, 406, 527, 630, 723, 807, 889, 969, 1045, 1120, 1192, 1260, 1327, 1391, 1453, 1512, 1570, 1625, 1677, 1728, 1777, 1824, 1870, 1913, 1955, 1995, 2033, 2069, 2105, 2140, 2174, 2207, 2238, 2270, 2300, 2330, 2359, 2387, 2415, 2443, 2469, 2496, 2522, 2548, 2573, 2598, 2622, 2646, 2671, 2694, 2717, 2740, 2763, 2786, 2809, 2830, 2852, 2875, 2896, 2918, 2939, 2960, 2981, 3002, 3023, 3043, 3064, 3084, 3105, 3125, 3145, 3164, 3184, 3203, 3223, 3242, 3262, 3281, 3299, 3318, 3336, 3355, 3373, 3391, 3409, 3427, 3445, 3463, 3480, 3498, 3515, 3532, 3549, 3567, 3584, 3600, 3617, 3633, 3650, 3667, 3683, 3699, 3715, 3731, 3748, 3764, 3780, 3795, 3811, 3827, 3842, 3858, 3873, 3888, 3904, 3919, 3934, 3948, 3964, 3979, 3993, 4008, 4023, 4038, 4052, 4066, 4081, 4095};

static int32_t _calibration_gamma_threshold[] = {100000, 2000000};

// CALIBRATION_IRIDIX_MIN_MAX_STR (1x1 2 bytes)
static uint16_t _calibration_iridix_min_max_str[]
 = {0};

// CALIBRATION_SHARP_ALT_D (8x2 2 bytes)
static uint16_t _calibration_sharp_alt_d[][2]
 =  {
  { 0, 16 },
  { 256, 16 },
  { 512, 10 },
  { 768, 10 },
  { 1024, 10 },
  { 1280, 8 },
  { 1536, 5 },
  { 1792, 0 }
};

// CALIBRATION_IRIDIX_AVG_COEF (1x1 1 bytes)
static uint8_t _calibration_iridix_avg_coef[]
 = {30};

// CALIBRATION_STITCHING_MS_NP (3x2 2 bytes)
static uint16_t _calibration_stitching_ms_np[][2]
 =  {
  { 0, 3680 },
  { 256, 3680 },
  { 512, 2680 }
};

// CALIBRATION_EVTOLUX_PROBABILITY_ENABLE (1x1 1 bytes)
static uint8_t _calibration_evtolux_probability_enable[]
 = {0};

// CALIBRATION_AE_CORRECTION (1x12 1 bytes)
static uint8_t _calibration_ae_correction[]
 = {128,128,128,128,128,108,98,88,78,78,78,75};

// CALIBRATION_AE_ZONE_WGHT_VER (1x32 2 bytes)
static uint16_t _calibration_ae_zone_wght_ver[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_SINTER_THRESH1 (7x2 2 bytes)
static uint16_t _calibration_sinter_thresh1[][2]
 =  {
  { 0, 0 },
  { 256, 0 },
  { 512, 2 },
  { 768, 3 },
  { 1024, 4 },
  { 1280, 4 },
  { 1536, 5 }
};

// CALIBRATION_AF_ZONE_WGHT_VER (1x17 2 bytes)
static uint16_t _calibration_af_zone_wght_ver[]
 = {0,0,16,16,16,16,16,16,16,16,16,16,16,16,16,0,0};

// CALIBRATION_DP_THRESHOLD (7x2 2 bytes)
static uint16_t _calibration_dp_threshold[][2]
 =  {
  { 0, 4095 },
  { 256, 312 },
  { 512, 302 },
  { 768, 110 },
  { 1024, 95 },
  { 1280, 85 },
  { 1536, 70 }
};

// CALIBRATION_SCALER_V_FILTER (1x256 4 bytes)
static uint32_t _calibration_scaler_v_filter[]
 = {4194304,0,37813760,0,54590721,255,88079361,255,121633537,254,155122177,254,205322497,254,238811393,253,288946177,509,322435073,508,372635649,507,422770689,507,456259585,506,506394625,506,556595201,505,606730241,505,640088321,505,690288897,504,740423937,504,773782017,504,823917057,504,840563457,504,890698497,504,924056577,504,940702977,504,990838016,505,1007484416,505,1040842240,506,1057488384,507,1057357568,508,1074003712,509,1073872896,254,623311869,392699,656734974,392444,673446398,392445,690092543,392190,690026751,457727,706672640,457216,723384064,457217,723318529,456962,739964673,456963,756676098,456708,756545026,456710,756413955,456711,789902851,390920,789771780,390921,789640708,390923,789575172,390669,789509637,390669,789444101,325135,789313285,325136,789182213,325138,789116677,259604,755496966,259605,755431430,194071,755300358,194073,738457862,128794,721615110,128796,721549830,63261,704707078,63518,704641798,16775200,704576261,16775457,687733765,16709923,670891013,16710180,505285370,16579082,505285114,16579083,505219578,16513804,521996538,16513804,538773497,16513805,538707961,16513806,538707705,16514062,555484665,16448527,555419129,16448528,555353337,16448785,572130297,16448785,571999225,16449042,571998969,16449043,571998969,16383763,571933177,16383765,571933177,16383765,571867385,16318486,571867129,16318743,571867129,16318743,571736057,16319000,571735802,16319000,571670266,16319256,571604474,16319258,554827258,16319514,554761466,16319771,554695930,16319772,537853179,16320028,537852923,16320284,537787387,16320285,520944635,16386077,504167419,16386333,504101628,16386334,235801350,132619,235735814,132875,235735814,132875,235735814,132875,235735558,132876,235735558,132876,235735557,198412,235735557,198412,235735557,198412,235735301,198668,235735301,198668,235735301,198668,235735301,198668,235735300,264204,235735300,264204,235669764,264460,235669764,264460,235669764,264460,235669508,264461,235669508,264461,235669507,329997,235669507,329997,235669507,329997,235669507,329997,235669251,330253,235669251,330253,235669251,330253,235669250,395789,235669250,395789,235603714,396045,235603714,396045,235603714,396045};

// AWB_COLOUR_PREFERENCE (1x4 2 bytes)
static uint16_t _awb_colour_preference[]
 = {7500,6000,4700,2800};

// CALIBRATION_EXPOSURE_RATIO_ADJUSTMENT (4x2 2 bytes)
static uint16_t _calibration_exposure_ratio_adjustment[][2]
 =  {
  { 256, 256 },
  { 4096, 256 },
  { 8192, 256 },
  { 16384, 256 }
};

// CALIBRATION_SINTER_SAD (7x2 2 bytes)
static uint16_t _calibration_sinter_sad[][2]
 =  {
  { 0, 8 },
  { 256, 8 },
  { 512, 5 },
  { 768, 5 },
  { 1024, 9 },
  { 1280, 11 },
  { 1536, 13 }
};

// CALIBRATION_IRIDIX_EV_LIM_NO_STR (1x2 4 bytes)
static uint32_t _calibration_iridix_ev_lim_no_str[]
 = {3750000,3574729};

// CALIBRATION_AE_CONTROL_HDR_TARGET (8x2 2 bytes)
static uint16_t _calibration_ae_control_hdr_target[][2]
 =  {
  { 0, 139 },
  { 256, 139 },
  { 512, 139 },
  { 768, 187 },
  { 1024, 236 },
  { 1280, 236 },
  { 1536, 236 },
  { 1792, 236 }
};

// CALIBRATION_DEMOSAIC_NP_OFFSET (7x2 2 bytes)
static uint16_t _calibration_demosaic_np_offset[][2]
 =  {
  { 0, 1 },
  { 256, 1 },
  { 512, 1 },
  { 768, 3 },
  { 1024, 18 },
  { 1280, 18 },
  { 1536, 15 }
};


// CALIBRATION_PF_RADIAL_LUT (1x33 1 bytes)
static uint8_t _calibration_pf_radial_lut[]
 = {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};

// CALIBRATION_TEMPER_STRENGTH (8x2 2 bytes)
static uint16_t _calibration_temper_strength[][2]
 =  {
  { 0, 120 },
  { 256, 120 },
  { 512, 120 },
  { 768, 120 },
  { 1024, 140 },
  { 1280, 145 },
  { 1536, 195 },
  { 1792, 195 }
};

// CALIBRATION_STATUS_INFO (1x5 4 bytes)
static uint32_t _calibration_status_info[]
 = {2200118,4172373,1795,7168,0};

// CALIBRATION_STITCHING_LM_NP (4x2 2 bytes)
static uint16_t _calibration_stitching_lm_np[][2]
 =  {
  { 0, 540 },
  { 768, 1458 },
  { 1024, 1458 },
  { 1280, 3000 }
};

// CALIBRATION_SHARP_ALT_DU (8x2 2 bytes)
static uint16_t _calibration_sharp_alt_du[][2]
 =  {
  { 0, 16 },
  { 256, 16 },
  { 512, 10 },
  { 768, 10 },
  { 1024, 8 },
  { 1280, 8 },
  { 1536, 5 },
  { 1792, 0 }
};

// CALIBRATION_IRIDIX_EV_LIM_FULL_STR (1x1 4 bytes)
static uint32_t _calibration_iridix_ev_lim_full_str[]
 = {2557570};

// CALIBRATION_SINTER_THRESH4 (7x2 2 bytes)
static uint16_t _calibration_sinter_thresh4[][2]
 =  {
  { 0, 0 },
  { 256, 0 },
  { 512, 0 },
  { 768, 5 },
  { 1024, 64 },
  { 1280, 64 },
  { 1536, 128 }
};

// CALIBRATION_CCM_ONE_GAIN_THRESHOLD (1x1 2 bytes)
static uint16_t _calibration_ccm_one_gain_threshold[]
 = {1408};

// CALIBRATION_IRIDIX8_STRENGTH_DK_ENH_CONTROL (1x15 4 bytes)
static uint32_t _calibration_iridix8_strength_dk_enh_control[]
 = {20,95,800,2000,8,20,7680,12800,0,40,40,7680,10240,20,0};

// CALIBRATION_FS_MC_OFF (1x1 2 bytes)
static uint16_t _calibration_fs_mc_off[]
 = {2048};

// CALIBRATION_SINTER_STRENGTH (8x2 2 bytes)
static uint16_t _calibration_sinter_strength[][2]
 =  {
  { 0, 35 },
  { 256, 43 },
  { 512, 53 },
  { 768, 65 },
  { 1024, 65 },
  { 1280, 70 },
  { 1536, 78 },
  { 1792, 82 }
};

// CALIBRATION_STITCHING_LM_MED_NOISE_INTENSITY (3x2 2 bytes)
static uint16_t _calibration_stitching_lm_med_noise_intensity[][2]
 =  {
  { 0, 32 },
  { 1536, 32 },
  { 2048, 4095 }
};

// CALIBRATION_SINTER_STRENGTH1 (8x2 2 bytes)
static uint16_t _calibration_sinter_strength1[][2]
 =  {
  { 0, 155 },
  { 256, 155 },
  { 512, 125 },
  { 768, 115 },
  { 1024, 115 },
  { 1280, 115 },
  { 1536, 85 },
  { 1792, 85 }
};

// CALIBRATION_SINTER_STRENGTH4 (8x2 2 bytes)
static uint16_t _calibration_sinter_strength4[][2]
 =  {
  { 0, 155 },
  { 256, 155 },
  { 512, 125 },
  { 768, 115 },
  { 1024, 115 },
  { 1280, 115 },
  { 1536, 85 },
  { 1792, 85 }
};

// CALIBRATION_CMOS_CONTROL (1x18 4 bytes)
static uint32_t _calibration_cmos_control[]
 = {0,50,0,0,0,0,0,0,128,0,256,255,184,128,0,133,0,4};

// CALIBRATION_SINTER_RADIAL_LUT (1x33 1 bytes)
static uint8_t _calibration_sinter_radial_lut[]
 = {0,0,0,0,0,0,1,3,4,6,7,9,10,12,13,15,16,18,19,21,22,24,24,24,24,24,24,24,24,24,24,24,24};

// CALIBRATION_AE_ZONE_WGHT_HOR (1x32 2 bytes)
static uint16_t _calibration_ae_zone_wght_hor[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_AWB_MIX_LIGHT_PARAMETERS (1x9 4 bytes)
static uint32_t _calibration_awb_mix_light_parameters[]
 = {1,500,3000,2000,330,5,260,252,0};

// CALIBRATION_AUTO_LEVEL_CONTROL (1x7 4 bytes)
static uint32_t _calibration_auto_level_control[]
 = {1,99,0,50,75,15,0};

// CALIBRATION_SATURATION_STRENGTH (9x2 2 bytes)
static uint16_t _calibration_saturation_strength[][2]
 =  {
  { 0, 128 },
  { 256, 128 },
  { 512, 128 },
  { 768, 118 },
  { 1024, 105 },
  { 1280, 90 },
  { 1535, 90 },
  { 1536, 128 },
  { 1792, 128 }
};

// CALIBRATION_STITCHING_LM_MOV_MULT (3x2 2 bytes)
static uint16_t _calibration_stitching_lm_mov_mult[][2]
 =  {
  { 0, 128 },
  { 384, 20 },
  { 1280, 8 }
};

// CALIBRATION_AE_EXPOSURE_CORRECTION (1x12 4 bytes)
static uint32_t _calibration_ae_exposure_correction[]
 = {6710,15739,15778,23282,56186,500325,632161,1190074,1406400,2382765,3295034,5491142};

// CALIBRATION_AF_ZONE_WGHT_HOR (1x17 2 bytes)
static uint16_t _calibration_af_zone_wght_hor[]
 = {0,0,16,16,16,16,16,16,16,16,16,16,16,16,16,0,0};

static uint32_t _calibration_zoom_lms[] = {
13, 0, 180, 361, 541, 722, 902, 1082, 1263, 1443, 1624, 1804, 1985, 2165, 2275
};

static uint32_t _calibration_zoom_af_lms[][21] = {
{166400/4, 166400/4, 166400/4, 167680/4, 167680/4, 167680/4,176000/4,176000/4,176000/4,177920/4,177920/4,177920/4,11,6,2,30,131072,131072,262144,65536, 0},
{136320/4, 136320/4, 136320/4, 137600/4, 137600/4, 137600/4,147200/4,147200/4,147200/4,149120/4,149120/4,149120/4,11,6,2,30,131072,131072,262144,65536, 0},
{110720/4, 110720/4, 110720/4, 112000/4, 112000/4, 112000/4,120320/4,120320/4,120320/4,122240/4,122240/4,122240/4,11,6,2,30,131072,131072,262144,65536, 0},
{88960/4, 88960/4, 88960/4, 90240/4, 90240/4, 90240/4, 99200/4, 99200/4, 99200/4,101120/4,101120/4,101120/4,11,6,2,30,131072,131072,262144,65536, 0},
{72320/4, 72320/4, 72320/4, 73600/4, 73600/4, 73600/4, 81920/4, 81920/4, 81920/4, 83840/4, 83840/4, 83840/4,11,6,2,30,131072,131072,262144,65536, 0},
{58240/4, 58240/4, 58240/4, 59520/4, 59520/4, 59520/4, 67840/4, 67840/4, 67840/4, 69760/4, 69760/4, 69760/4,11,6,2,30,131072,131072,262144,65536, 0},
{46080/4, 46080/4, 46080/4, 47360/4, 47360/4, 47360/4, 56960/4, 56960/4, 56960/4, 58880/4, 58880/4, 58880/4,11,6,2,30,131072,131072,262144,65536, 0},
{36480/4, 36480/4, 36480/4, 37760/4, 37760/4, 37760/4, 46720/4, 46720/4, 46720/4, 48640/4, 48640/4, 48640/4,11,6,2,30,131072,131072,262144,65536, 0},
{28800/4, 28800/4, 28800/4, 30080/4, 30080/4, 30080/4, 39040/4, 39040/4, 39040/4, 40960/4, 40960/4, 40960/4,11,6,2,30,131072,131072,262144,65536, 0},
{23040/4, 23040/4, 23040/4, 24320/4, 24320/4, 24320/4, 33280/4, 33280/4, 33280/4, 35200/4, 35200/4, 35200/4,11,6,2,30,131072,131072,262144,65536, 0},
{18560/4, 18560/4, 18560/4, 19840/4, 19840/4, 19840/4, 28800/4, 28800/4, 28800/4, 30720/4, 30720/4, 30720/4,11,6,2,30,131072,131072,262144,65536, 0},
{14720/4, 14720/4, 14720/4, 16000/4, 16000/4, 16000/4, 24960/4, 24960/4, 24960/4, 26880/4, 26880/4, 26880/4,11,6,2,30,131072,131072,262144,65536, 0},
{11520/4, 11520/4, 11520/4, 12800/4, 12800/4, 12800/4, 22400/4, 22400/4, 22400/4, 24320/4, 24320/4, 24320/4,11,6,2,30,131072,131072,262144,65536, 0}
};

static int16_t _calibration_bypass_control[] = {0, 0, 1*256, 5*256};

// CALIBRATION_iridix_bright_pr (7x2 2 bytes)
static uint16_t _calibration_iridix_bright_pr[][2]
 =  {
  { 0, 170 },
  { 256, 170 },
  { 512, 170 },
  { 768, 1800 },
  { 1024, 1911 },
  { 1280, 2200 },
  { 1536, 2400 }
};

// CALIBRATION_iridix_svariance (7x2 2 bytes)
static uint16_t _calibration_iridix_svariance[][2]
 =  {
  { 0, 170 },
  { 256, 170 },
  { 512, 170 },
  { 768, 1800 },
  { 1024, 1911 },
  { 1280, 2200 },
  { 1536, 2400 }
};


static LookupTable calibration_scaler_h_filter = { .ptr = _calibration_scaler_h_filter, .rows = 1, .cols = sizeof(_calibration_scaler_h_filter) / sizeof(_calibration_scaler_h_filter[0]), .width = sizeof(_calibration_scaler_h_filter[0] ) };
static LookupTable calibration_awb_zone_wght_ver = { .ptr = _calibration_awb_zone_wght_ver, .rows = 1, .cols = sizeof(_calibration_awb_zone_wght_ver) / sizeof(_calibration_awb_zone_wght_ver[0]), .width = sizeof(_calibration_awb_zone_wght_ver[0] ) };
static LookupTable calibration_awb_bg_max_gain = { .ptr = _calibration_awb_bg_max_gain, .cols = 2, .rows = sizeof(_calibration_awb_bg_max_gain) / sizeof(_calibration_awb_bg_max_gain[0]), .width = sizeof(_calibration_awb_bg_max_gain[0][0] ) };
static LookupTable calibration_iridix_strength_maximum = { .ptr = _calibration_iridix_strength_maximum, .rows = 1, .cols = sizeof(_calibration_iridix_strength_maximum) / sizeof(_calibration_iridix_strength_maximum[0]), .width = sizeof(_calibration_iridix_strength_maximum[0] ) };
static LookupTable calibration_sinter_radial_params = { .ptr = _calibration_sinter_radial_params, .rows = 1, .cols = sizeof(_calibration_sinter_radial_params) / sizeof(_calibration_sinter_radial_params[0]), .width = sizeof(_calibration_sinter_radial_params[0] ) };
static LookupTable calibration_sharpen_fr = { .ptr = _calibration_sharpen_fr, .cols = 2, .rows = sizeof(_calibration_sharpen_fr) / sizeof(_calibration_sharpen_fr[0]), .width = sizeof(_calibration_sharpen_fr[0][0] ) };
static LookupTable calibration_pf_radial_params = { .ptr = _calibration_pf_radial_params, .rows = 1, .cols = sizeof(_calibration_pf_radial_params) / sizeof(_calibration_pf_radial_params[0]), .width = sizeof(_calibration_pf_radial_params[0] ) };
static LookupTable calibration_sinter_strength_mc_contrast = { .ptr = _calibration_sinter_strength_mc_contrast, .rows = 1, .cols = sizeof(_calibration_sinter_strength_mc_contrast) / sizeof(_calibration_sinter_strength_mc_contrast[0]), .width = sizeof(_calibration_sinter_strength_mc_contrast[0] ) };
static LookupTable calibration_cmos_exposure_partition_luts = { .ptr = _calibration_cmos_exposure_partition_luts, .cols = 10, .rows = sizeof(_calibration_cmos_exposure_partition_luts) / sizeof(_calibration_cmos_exposure_partition_luts[0]), .width = sizeof(_calibration_cmos_exposure_partition_luts[0][0] ) };
static LookupTable calibration_sharp_alt_ud = { .ptr = _calibration_sharp_alt_ud, .cols = 2, .rows = sizeof(_calibration_sharp_alt_ud) / sizeof(_calibration_sharp_alt_ud[0]), .width = sizeof(_calibration_sharp_alt_ud[0][0] ) };
static LookupTable calibration_mesh_shading_strength = { .ptr = _calibration_mesh_shading_strength, .rows = 1, .cols = sizeof(_calibration_mesh_shading_strength) / sizeof(_calibration_mesh_shading_strength[0]), .width = sizeof(_calibration_mesh_shading_strength[0] ) };
static LookupTable calibration_af_lms = { .ptr = _calibration_af_lms, .rows = 1, .cols = sizeof(_calibration_af_lms) / sizeof(_calibration_af_lms[0]), .width = sizeof(_calibration_af_lms[0] ) };
static LookupTable calibration_dp_slope = { .ptr = _calibration_dp_slope, .cols = 2, .rows = sizeof(_calibration_dp_slope) / sizeof(_calibration_dp_slope[0]), .width = sizeof(_calibration_dp_slope[0][0] ) };
static LookupTable calibration_sharpen_ds1 = { .ptr = _calibration_sharpen_ds1, .cols = 2, .rows = sizeof(_calibration_sharpen_ds1) / sizeof(_calibration_sharpen_ds1[0]), .width = sizeof(_calibration_sharpen_ds1[0][0] ) };
static LookupTable calibration_sinter_intconfig = { .ptr = _calibration_sinter_intconfig, .cols = 2, .rows = sizeof(_calibration_sinter_intconfig) / sizeof(_calibration_sinter_intconfig[0]), .width = sizeof(_calibration_sinter_intconfig[0][0] ) };
static LookupTable calibration_cnr_uv_delta12_slope = { .ptr = _calibration_cnr_uv_delta12_slope, .cols = 2, .rows = sizeof(_calibration_cnr_uv_delta12_slope) / sizeof(_calibration_cnr_uv_delta12_slope[0]), .width = sizeof(_calibration_cnr_uv_delta12_slope[0][0] ) };
static LookupTable calibration_rgb2yuv_conversion = { .ptr = _calibration_rgb2yuv_conversion, .rows = 1, .cols = sizeof(_calibration_rgb2yuv_conversion) / sizeof(_calibration_rgb2yuv_conversion[0]), .width = sizeof(_calibration_rgb2yuv_conversion[0] ) };
static LookupTable calibration_stitching_ms_mov_mult = { .ptr = _calibration_stitching_ms_mov_mult, .cols = 2, .rows = sizeof(_calibration_stitching_ms_mov_mult) / sizeof(_calibration_stitching_ms_mov_mult[0]), .width = sizeof(_calibration_stitching_ms_mov_mult[0][0] ) };
static LookupTable calibration_awb_avg_coef = { .ptr = _calibration_awb_avg_coef, .rows = 1, .cols = sizeof(_calibration_awb_avg_coef) / sizeof(_calibration_awb_avg_coef[0]), .width = sizeof(_calibration_awb_avg_coef[0] ) };
static LookupTable calibration_ae_control = { .ptr = _calibration_ae_control, .rows = 1, .cols = sizeof(_calibration_ae_control) / sizeof(_calibration_ae_control[0]), .width = sizeof(_calibration_ae_control[0] ) };
static LookupTable calibration_awb_zone_wght_hor = { .ptr = _calibration_awb_zone_wght_hor, .rows = 1, .cols = sizeof(_calibration_awb_zone_wght_hor) / sizeof(_calibration_awb_zone_wght_hor[0]), .width = sizeof(_calibration_awb_zone_wght_hor[0] ) };
static LookupTable calibration_custom_settings_context = { .ptr = _calibration_custom_settings_context, .rows = 1, .cols = sizeof(_calibration_custom_settings_context) / sizeof(_calibration_custom_settings_context[0]), .width = sizeof(_calibration_custom_settings_context[0] ) };
static LookupTable calibration_iridix_min_max_str = { .ptr = _calibration_iridix_min_max_str, .rows = 1, .cols = sizeof(_calibration_iridix_min_max_str) / sizeof(_calibration_iridix_min_max_str[0]), .width = sizeof(_calibration_iridix_min_max_str[0] ) };
static LookupTable calibration_sharp_alt_d = { .ptr = _calibration_sharp_alt_d, .cols = 2, .rows = sizeof(_calibration_sharp_alt_d) / sizeof(_calibration_sharp_alt_d[0]), .width = sizeof(_calibration_sharp_alt_d[0][0] ) };
static LookupTable calibration_iridix_avg_coef = { .ptr = _calibration_iridix_avg_coef, .rows = 1, .cols = sizeof(_calibration_iridix_avg_coef) / sizeof(_calibration_iridix_avg_coef[0]), .width = sizeof(_calibration_iridix_avg_coef[0] ) };
static LookupTable calibration_stitching_ms_np = { .ptr = _calibration_stitching_ms_np, .cols = 2, .rows = sizeof(_calibration_stitching_ms_np) / sizeof(_calibration_stitching_ms_np[0]), .width = sizeof(_calibration_stitching_ms_np[0][0] ) };
static LookupTable calibration_evtolux_probability_enable = { .ptr = _calibration_evtolux_probability_enable, .rows = 1, .cols = sizeof(_calibration_evtolux_probability_enable) / sizeof(_calibration_evtolux_probability_enable[0]), .width = sizeof(_calibration_evtolux_probability_enable[0] ) };
static LookupTable calibration_ae_correction = { .ptr = _calibration_ae_correction, .rows = 1, .cols = sizeof(_calibration_ae_correction) / sizeof(_calibration_ae_correction[0]), .width = sizeof(_calibration_ae_correction[0] ) };
static LookupTable calibration_ae_zone_wght_ver = { .ptr = _calibration_ae_zone_wght_ver, .rows = 1, .cols = sizeof(_calibration_ae_zone_wght_ver) / sizeof(_calibration_ae_zone_wght_ver[0]), .width = sizeof(_calibration_ae_zone_wght_ver[0] ) };
static LookupTable calibration_sinter_thresh1 = { .ptr = _calibration_sinter_thresh1, .cols = 2, .rows = sizeof(_calibration_sinter_thresh1) / sizeof(_calibration_sinter_thresh1[0]), .width = sizeof(_calibration_sinter_thresh1[0][0] ) };
static LookupTable calibration_af_zone_wght_ver = { .ptr = _calibration_af_zone_wght_ver, .rows = 1, .cols = sizeof(_calibration_af_zone_wght_ver) / sizeof(_calibration_af_zone_wght_ver[0]), .width = sizeof(_calibration_af_zone_wght_ver[0] ) };
static LookupTable calibration_dp_threshold = { .ptr = _calibration_dp_threshold, .cols = 2, .rows = sizeof(_calibration_dp_threshold) / sizeof(_calibration_dp_threshold[0]), .width = sizeof(_calibration_dp_threshold[0][0] ) };
static LookupTable calibration_scaler_v_filter = { .ptr = _calibration_scaler_v_filter, .rows = 1, .cols = sizeof(_calibration_scaler_v_filter) / sizeof(_calibration_scaler_v_filter[0]), .width = sizeof(_calibration_scaler_v_filter[0] ) };
static LookupTable awb_colour_preference = { .ptr = _awb_colour_preference, .rows = 1, .cols = sizeof(_awb_colour_preference) / sizeof(_awb_colour_preference[0]), .width = sizeof(_awb_colour_preference[0] ) };
static LookupTable calibration_exposure_ratio_adjustment = { .ptr = _calibration_exposure_ratio_adjustment, .cols = 2, .rows = sizeof(_calibration_exposure_ratio_adjustment) / sizeof(_calibration_exposure_ratio_adjustment[0]), .width = sizeof(_calibration_exposure_ratio_adjustment[0][0] ) };
static LookupTable calibration_sinter_sad = { .ptr = _calibration_sinter_sad, .cols = 2, .rows = sizeof(_calibration_sinter_sad) / sizeof(_calibration_sinter_sad[0]), .width = sizeof(_calibration_sinter_sad[0][0] ) };
static LookupTable calibration_iridix_ev_lim_no_str = { .ptr = _calibration_iridix_ev_lim_no_str, .rows = 1, .cols = sizeof(_calibration_iridix_ev_lim_no_str) / sizeof(_calibration_iridix_ev_lim_no_str[0]), .width = sizeof(_calibration_iridix_ev_lim_no_str[0] ) };
static LookupTable calibration_ae_control_hdr_target = { .ptr = _calibration_ae_control_hdr_target, .cols = 2, .rows = sizeof(_calibration_ae_control_hdr_target) / sizeof(_calibration_ae_control_hdr_target[0]), .width = sizeof(_calibration_ae_control_hdr_target[0][0] ) };
static LookupTable calibration_demosaic_np_offset = { .ptr = _calibration_demosaic_np_offset, .cols = 2, .rows = sizeof(_calibration_demosaic_np_offset) / sizeof(_calibration_demosaic_np_offset[0]), .width = sizeof(_calibration_demosaic_np_offset[0][0] ) };
static LookupTable calibration_pf_radial_lut = { .ptr = _calibration_pf_radial_lut, .rows = 1, .cols = sizeof(_calibration_pf_radial_lut) / sizeof(_calibration_pf_radial_lut[0]), .width = sizeof(_calibration_pf_radial_lut[0] ) };
static LookupTable calibration_temper_strength = { .ptr = _calibration_temper_strength, .cols = 2, .rows = sizeof(_calibration_temper_strength) / sizeof(_calibration_temper_strength[0]), .width = sizeof(_calibration_temper_strength[0][0] ) };
static LookupTable calibration_status_info = { .ptr = _calibration_status_info, .rows = 1, .cols = sizeof(_calibration_status_info) / sizeof(_calibration_status_info[0]), .width = sizeof(_calibration_status_info[0] ) };
static LookupTable calibration_stitching_lm_np = { .ptr = _calibration_stitching_lm_np, .cols = 2, .rows = sizeof(_calibration_stitching_lm_np) / sizeof(_calibration_stitching_lm_np[0]), .width = sizeof(_calibration_stitching_lm_np[0][0] ) };
static LookupTable calibration_sharp_alt_du = { .ptr = _calibration_sharp_alt_du, .cols = 2, .rows = sizeof(_calibration_sharp_alt_du) / sizeof(_calibration_sharp_alt_du[0]), .width = sizeof(_calibration_sharp_alt_du[0][0] ) };
static LookupTable calibration_iridix_ev_lim_full_str = { .ptr = _calibration_iridix_ev_lim_full_str, .rows = 1, .cols = sizeof(_calibration_iridix_ev_lim_full_str) / sizeof(_calibration_iridix_ev_lim_full_str[0]), .width = sizeof(_calibration_iridix_ev_lim_full_str[0] ) };
static LookupTable calibration_sinter_thresh4 = { .ptr = _calibration_sinter_thresh4, .cols = 2, .rows = sizeof(_calibration_sinter_thresh4) / sizeof(_calibration_sinter_thresh4[0]), .width = sizeof(_calibration_sinter_thresh4[0][0] ) };
static LookupTable calibration_ccm_one_gain_threshold = { .ptr = _calibration_ccm_one_gain_threshold, .rows = 1, .cols = sizeof(_calibration_ccm_one_gain_threshold) / sizeof(_calibration_ccm_one_gain_threshold[0]), .width = sizeof(_calibration_ccm_one_gain_threshold[0] ) };
static LookupTable calibration_iridix8_strength_dk_enh_control = { .ptr = _calibration_iridix8_strength_dk_enh_control, .rows = 1, .cols = sizeof(_calibration_iridix8_strength_dk_enh_control) / sizeof(_calibration_iridix8_strength_dk_enh_control[0]), .width = sizeof(_calibration_iridix8_strength_dk_enh_control[0] ) };
static LookupTable calibration_fs_mc_off = { .ptr = _calibration_fs_mc_off, .rows = 1, .cols = sizeof(_calibration_fs_mc_off) / sizeof(_calibration_fs_mc_off[0]), .width = sizeof(_calibration_fs_mc_off[0] ) };
static LookupTable calibration_sinter_strength = { .ptr = _calibration_sinter_strength, .cols = 2, .rows = sizeof(_calibration_sinter_strength) / sizeof(_calibration_sinter_strength[0]), .width = sizeof(_calibration_sinter_strength[0][0] ) };
static LookupTable calibration_stitching_lm_med_noise_intensity = { .ptr = _calibration_stitching_lm_med_noise_intensity, .cols = 2, .rows = sizeof(_calibration_stitching_lm_med_noise_intensity) / sizeof(_calibration_stitching_lm_med_noise_intensity[0]), .width = sizeof(_calibration_stitching_lm_med_noise_intensity[0][0] ) };
static LookupTable calibration_sinter_strength1 = { .ptr = _calibration_sinter_strength1, .cols = 2, .rows = sizeof(_calibration_sinter_strength1) / sizeof(_calibration_sinter_strength1[0]), .width = sizeof(_calibration_sinter_strength1[0][0] ) };
static LookupTable calibration_cmos_control = { .ptr = _calibration_cmos_control, .rows = 1, .cols = sizeof(_calibration_cmos_control) / sizeof(_calibration_cmos_control[0]), .width = sizeof(_calibration_cmos_control[0] ) };
static LookupTable calibration_sinter_radial_lut = { .ptr = _calibration_sinter_radial_lut, .rows = 1, .cols = sizeof(_calibration_sinter_radial_lut) / sizeof(_calibration_sinter_radial_lut[0]), .width = sizeof(_calibration_sinter_radial_lut[0] ) };
static LookupTable calibration_ae_zone_wght_hor = { .ptr = _calibration_ae_zone_wght_hor, .rows = 1, .cols = sizeof(_calibration_ae_zone_wght_hor) / sizeof(_calibration_ae_zone_wght_hor[0]), .width = sizeof(_calibration_ae_zone_wght_hor[0] ) };
static LookupTable calibration_awb_mix_light_parameters = { .ptr = _calibration_awb_mix_light_parameters, .rows = 1, .cols = sizeof(_calibration_awb_mix_light_parameters) / sizeof(_calibration_awb_mix_light_parameters[0]), .width = sizeof(_calibration_awb_mix_light_parameters[0] ) };
static LookupTable calibration_auto_level_control = { .ptr = _calibration_auto_level_control, .rows = 1, .cols = sizeof(_calibration_auto_level_control) / sizeof(_calibration_auto_level_control[0]), .width = sizeof(_calibration_auto_level_control[0] ) };
static LookupTable calibration_saturation_strength = { .ptr = _calibration_saturation_strength, .cols = 2, .rows = sizeof(_calibration_saturation_strength) / sizeof(_calibration_saturation_strength[0]), .width = sizeof(_calibration_saturation_strength[0][0] ) };
static LookupTable calibration_stitching_lm_mov_mult = { .ptr = _calibration_stitching_lm_mov_mult, .cols = 2, .rows = sizeof(_calibration_stitching_lm_mov_mult) / sizeof(_calibration_stitching_lm_mov_mult[0]), .width = sizeof(_calibration_stitching_lm_mov_mult[0][0] ) };
static LookupTable calibration_ae_exposure_correction = { .ptr = _calibration_ae_exposure_correction, .rows = 1, .cols = sizeof(_calibration_ae_exposure_correction) / sizeof(_calibration_ae_exposure_correction[0]), .width = sizeof(_calibration_ae_exposure_correction[0] ) };
static LookupTable calibration_af_zone_wght_hor = { .ptr = _calibration_af_zone_wght_hor, .rows = 1, .cols = sizeof(_calibration_af_zone_wght_hor) / sizeof(_calibration_af_zone_wght_hor[0]), .width = sizeof(_calibration_af_zone_wght_hor[0] ) };
static LookupTable calibration_zoom_lms = {.ptr = _calibration_zoom_lms, .rows = 1, .cols = sizeof(_calibration_zoom_lms)/sizeof(_calibration_zoom_lms[0]), .width = sizeof( _calibration_zoom_lms[0])};
static LookupTable calibration_zoom_af_lms = {.ptr = _calibration_zoom_af_lms, .rows = sizeof( _calibration_zoom_af_lms ) / sizeof( _calibration_zoom_af_lms[0] ), .cols = 21, .width = sizeof( _calibration_zoom_af_lms[0][0] )};

static LookupTable calibration_gamma_ev1 = {.ptr = _calibration_gamma_ev1, .rows = 1, .cols = sizeof( _calibration_gamma_ev1 ) / sizeof( _calibration_gamma_ev1[0] ), .width = sizeof( _calibration_gamma_ev1[0] )};
static LookupTable calibration_gamma_ev2 = {.ptr = _calibration_gamma_ev2, .rows = 1, .cols = sizeof( _calibration_gamma_ev2 ) / sizeof( _calibration_gamma_ev2[0] ), .width = sizeof( _calibration_gamma_ev2[0] )};

static LookupTable calibration_gamma_threshold = {.ptr = _calibration_gamma_threshold, .rows = 1, .cols = sizeof( _calibration_gamma_threshold ) / sizeof( _calibration_gamma_threshold[0] ), .width = sizeof( _calibration_gamma_threshold[0] )};

static LookupTable calibration_bypass_control = {.ptr = _calibration_bypass_control, .rows = 1, .cols = sizeof(_calibration_bypass_control) / sizeof(_calibration_bypass_control[0]), .width = sizeof(_calibration_bypass_control[0])};
static LookupTable calibration_sinter_strength4 = {.ptr = _calibration_sinter_strength4, .cols = 2, .rows = sizeof(_calibration_sinter_strength4) / sizeof(_calibration_sinter_strength4[0]), .width = sizeof(_calibration_sinter_strength4[0][0] ) };

static LookupTable calibration_temper_threshold = {.ptr = _calibration_temper_threshold, .rows = 1, .cols = sizeof( _calibration_temper_threshold ) / sizeof( _calibration_temper_threshold[0] ), .width = sizeof( _calibration_temper_threshold[0] )};
static LookupTable calibration_user_temper_noise_lut = { .ptr = _calibration_user_temper_noise_lut, .rows = 1, .cols = sizeof(_calibration_user_temper_noise_lut) / sizeof(_calibration_user_temper_noise_lut[0]), .width = sizeof(_calibration_user_temper_noise_lut[0] ) };
static LookupTable calibration_user_temper_noise_lut_1 = { .ptr = _calibration_user_temper_noise_lut_1, .rows = 1, .cols = sizeof(_calibration_user_temper_noise_lut_1) / sizeof(_calibration_user_temper_noise_lut_1[0]), .width = sizeof(_calibration_user_temper_noise_lut_1[0] ) };
static LookupTable calibration_user_temper_noise_lut_2 = { .ptr = _calibration_user_temper_noise_lut_2, .rows = 1, .cols = sizeof(_calibration_user_temper_noise_lut_2) / sizeof(_calibration_user_temper_noise_lut_2[0]), .width = sizeof(_calibration_user_temper_noise_lut_2[0] ) };
static LookupTable calibration_user_sinter_lut = { .ptr = _calibration_user_sinter_lut, .rows = 1, .cols = sizeof(_calibration_user_sinter_lut) / sizeof(_calibration_user_sinter_lut[0]), .width = sizeof(_calibration_user_sinter_lut[0] ) };

uint32_t get_calibrations_dynamic_linear_dummy( ACameraCalibrations *c ) {
//uint32_t get_dynamic_calibrations( ApicalCalibrations * c ) {
    uint32_t result = 0;
    if (c != 0) {
        c->calibrations[CALIBRATION_SCALER_H_FILTER] = &calibration_scaler_h_filter;
        c->calibrations[CALIBRATION_AWB_ZONE_WGHT_VER] = &calibration_awb_zone_wght_ver;
        c->calibrations[CALIBRATION_AWB_BG_MAX_GAIN] = &calibration_awb_bg_max_gain;
        c->calibrations[CALIBRATION_IRIDIX_STRENGTH_MAXIMUM] = &calibration_iridix_strength_maximum;
        c->calibrations[CALIBRATION_SINTER_RADIAL_PARAMS] = &calibration_sinter_radial_params;
        c->calibrations[CALIBRATION_SHARPEN_FR] = &calibration_sharpen_fr;
        c->calibrations[CALIBRATION_PF_RADIAL_PARAMS] = &calibration_pf_radial_params;
        c->calibrations[CALIBRATION_SINTER_STRENGTH_MC_CONTRAST] = &calibration_sinter_strength_mc_contrast;
        c->calibrations[CALIBRATION_CMOS_EXPOSURE_PARTITION_LUTS] = &calibration_cmos_exposure_partition_luts;
        c->calibrations[CALIBRATION_SHARP_ALT_UD] = &calibration_sharp_alt_ud;
        c->calibrations[CALIBRATION_MESH_SHADING_STRENGTH] = &calibration_mesh_shading_strength;
        c->calibrations[CALIBRATION_AF_LMS] = &calibration_af_lms;
        c->calibrations[CALIBRATION_DP_SLOPE] = &calibration_dp_slope;
        c->calibrations[CALIBRATION_SHARPEN_DS1] = &calibration_sharpen_ds1;
        c->calibrations[CALIBRATION_SINTER_INTCONFIG] = &calibration_sinter_intconfig;
        c->calibrations[CALIBRATION_CNR_UV_DELTA12_SLOPE] = &calibration_cnr_uv_delta12_slope;
        c->calibrations[CALIBRATION_RGB2YUV_CONVERSION] = &calibration_rgb2yuv_conversion;
        c->calibrations[CALIBRATION_STITCHING_MS_MOV_MULT] = &calibration_stitching_ms_mov_mult;
        c->calibrations[CALIBRATION_AWB_AVG_COEF] = &calibration_awb_avg_coef;
        c->calibrations[CALIBRATION_AE_CONTROL] = &calibration_ae_control;
        c->calibrations[CALIBRATION_AWB_ZONE_WGHT_HOR] = &calibration_awb_zone_wght_hor;
        c->calibrations[CALIBRATION_CUSTOM_SETTINGS_CONTEXT] = &calibration_custom_settings_context;
        c->calibrations[CALIBRATION_IRIDIX_MIN_MAX_STR] = &calibration_iridix_min_max_str;
        c->calibrations[CALIBRATION_SHARP_ALT_D] = &calibration_sharp_alt_d;
        c->calibrations[CALIBRATION_IRIDIX_AVG_COEF] = &calibration_iridix_avg_coef;
        c->calibrations[CALIBRATION_STITCHING_MS_NP] = &calibration_stitching_ms_np;
        c->calibrations[CALIBRATION_EVTOLUX_PROBABILITY_ENABLE] = &calibration_evtolux_probability_enable;
        c->calibrations[CALIBRATION_AE_CORRECTION] = &calibration_ae_correction;
        c->calibrations[CALIBRATION_AE_ZONE_WGHT_VER] = &calibration_ae_zone_wght_ver;
        c->calibrations[CALIBRATION_SINTER_THRESH1] = &calibration_sinter_thresh1;
        c->calibrations[CALIBRATION_AF_ZONE_WGHT_VER] = &calibration_af_zone_wght_ver;
        c->calibrations[CALIBRATION_DP_THRESHOLD] = &calibration_dp_threshold;
        c->calibrations[CALIBRATION_SCALER_V_FILTER] = &calibration_scaler_v_filter;
        c->calibrations[AWB_COLOUR_PREFERENCE] = &awb_colour_preference;
        c->calibrations[CALIBRATION_EXPOSURE_RATIO_ADJUSTMENT] = &calibration_exposure_ratio_adjustment;
        c->calibrations[CALIBRATION_SINTER_SAD] = &calibration_sinter_sad;
        c->calibrations[CALIBRATION_IRIDIX_EV_LIM_NO_STR] = &calibration_iridix_ev_lim_no_str;
        c->calibrations[CALIBRATION_AE_CONTROL_HDR_TARGET] = &calibration_ae_control_hdr_target;
        c->calibrations[CALIBRATION_DEMOSAIC_NP_OFFSET] = &calibration_demosaic_np_offset;
        c->calibrations[CALIBRATION_PF_RADIAL_LUT] = &calibration_pf_radial_lut;
        c->calibrations[CALIBRATION_TEMPER_STRENGTH] = &calibration_temper_strength;
        c->calibrations[CALIBRATION_STATUS_INFO] = &calibration_status_info;
        c->calibrations[CALIBRATION_STITCHING_LM_NP] = &calibration_stitching_lm_np;
        c->calibrations[CALIBRATION_SHARP_ALT_DU] = &calibration_sharp_alt_du;
        c->calibrations[CALIBRATION_IRIDIX_EV_LIM_FULL_STR] = &calibration_iridix_ev_lim_full_str;
        c->calibrations[CALIBRATION_SINTER_THRESH4] = &calibration_sinter_thresh4;
        c->calibrations[CALIBRATION_CCM_ONE_GAIN_THRESHOLD] = &calibration_ccm_one_gain_threshold;
        c->calibrations[CALIBRATION_IRIDIX8_STRENGTH_DK_ENH_CONTROL] = &calibration_iridix8_strength_dk_enh_control;
        c->calibrations[CALIBRATION_FS_MC_OFF] = &calibration_fs_mc_off;
        c->calibrations[CALIBRATION_SINTER_STRENGTH] = &calibration_sinter_strength;
        c->calibrations[CALIBRATION_STITCHING_LM_MED_NOISE_INTENSITY] = &calibration_stitching_lm_med_noise_intensity;
        c->calibrations[CALIBRATION_SINTER_STRENGTH1] = &calibration_sinter_strength1;
        c->calibrations[CALIBRATION_CMOS_CONTROL] = &calibration_cmos_control;
        c->calibrations[CALIBRATION_SINTER_RADIAL_LUT] = &calibration_sinter_radial_lut;
        c->calibrations[CALIBRATION_AE_ZONE_WGHT_HOR] = &calibration_ae_zone_wght_hor;
        c->calibrations[CALIBRATION_AWB_MIX_LIGHT_PARAMETERS] = &calibration_awb_mix_light_parameters;
        c->calibrations[CALIBRATION_AUTO_LEVEL_CONTROL] = &calibration_auto_level_control;
        c->calibrations[CALIBRATION_SATURATION_STRENGTH] = &calibration_saturation_strength;
        c->calibrations[CALIBRATION_STITCHING_LM_MOV_MULT] = &calibration_stitching_lm_mov_mult;
        c->calibrations[CALIBRATION_AE_EXPOSURE_CORRECTION] = &calibration_ae_exposure_correction;
        c->calibrations[CALIBRATION_AF_ZONE_WGHT_HOR] = &calibration_af_zone_wght_hor;
	c->calibrations[CALIBRATION_ZOOM_LMS] = &calibration_zoom_lms;
	c->calibrations[CALIBRATION_ZOOM_AF_LMS] = &calibration_zoom_af_lms;
	c->calibrations[CALIBRATION_GAMMA_EV1] = &calibration_gamma_ev1;
	c->calibrations[CALIBRATION_GAMMA_EV2] = &calibration_gamma_ev2;
	c->calibrations[CALIBRATION_GAMMA_THRESHOLD] = &calibration_gamma_threshold;
	c->calibrations[CALIBRATION_BYPASS_CONTROL] = &calibration_bypass_control;
	c->calibrations[CALIBRATION_SINTER_STRENGTH4] = &calibration_sinter_strength4;
  c->calibrations[CALIBRATION_TEMPER_THRESHOLD] = &calibration_temper_threshold;
	c->calibrations[CALIBRATION_USER_TEMPER_NOISE_LUT] = &calibration_user_temper_noise_lut;
  c->calibrations[CALIBRATION_USER_TEMPER_NOISE_LUT_1] = &calibration_user_temper_noise_lut_1;
  c->calibrations[CALIBRATION_USER_TEMPER_NOISE_LUT_2] = &calibration_user_temper_noise_lut_2;
	c->calibrations[CALIBRATION_USER_SINTER_LUT] = &calibration_user_sinter_lut;
    } else {
        result = -1;
    }
    return result;
}
