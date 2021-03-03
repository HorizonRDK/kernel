//------------------------------------------------------------------------------
//  The confidential and proprietary information contained in this file may
//  only be used by a person authorised under and to the extent permitted
//  by a subsisting licensing agreement from ARM Limited or its affiliates.
//
//         (C) COPYRIGHT 2018-2019 ARM Limited or its affiliates.
//             ALL RIGHTS RESERVED
//
//  This entire notice must be reproduced on all copies of this file
//  and copies of this file may only be made by a person if such person is
//  permitted to do so under the terms of a subsisting license agreement
//  from ARM Limited or its affiliates.
//
//  Release Information : Cortex-A53_STL-r0p0-00eac0
//
//------------------------------------------------------------------------------
//===========================================================================================
//   About: About the File
//      A53_STL constants and global variables definition file
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      N.A.
//
//===========================================================================================//

#ifndef DRIVERS_SOC_HOBOT_CPU_CAL_SCHEDULER_INC_A53_STL_ARRAYS_H_
#define DRIVERS_SOC_HOBOT_CPU_CAL_SCHEDULER_INC_A53_STL_ARRAYS_H_

// Maximum number of SW tests performed by A53 STL at EL1
//#define A53_STL_EL1_TOT_MODULES_MAX     (45u)
#define A53_STL_EL1_TOT_MODULES_MAX     (52u)
// Maximum number of SW tests performed by A53 STL at EL2
#define A53_STL_EL2_TOT_MODULES_MAX     (45u)
// Maximum number of SW tests performed by A53 STL at EL3
#define A53_STL_EL3_TOT_MODULES_MAX     (11u)

// Definition of the identifier of the tests
#define A53_STL_TEST_EL1_CORE_ID_001    (0)
#define A53_STL_TEST_EL1_CORE_ID_002    (1)
#define A53_STL_TEST_EL1_CORE_ID_003    (2)
#define A53_STL_TEST_EL1_CORE_ID_004    (3)
#define A53_STL_TEST_EL1_CORE_ID_005    (4)
#define A53_STL_TEST_EL1_CORE_ID_006    (5)
#define A53_STL_TEST_EL1_CORE_ID_007    (6)
#define A53_STL_TEST_EL1_CORE_ID_008    (7)
#define A53_STL_TEST_EL1_CORE_ID_009    (8)
#define A53_STL_TEST_EL1_CORE_ID_010    (9)
#define A53_STL_TEST_EL1_CORE_ID_011    (10)
#define A53_STL_TEST_EL1_CORE_ID_012    (11)
#define A53_STL_TEST_EL1_CORE_ID_013    (12)
#define A53_STL_TEST_EL1_CORE_ID_014    (13)
#define A53_STL_TEST_EL1_CORE_ID_015    (14)
#define A53_STL_TEST_EL1_CORE_ID_016    (15)
#define A53_STL_TEST_EL1_CORE_ID_017    (16)
#define A53_STL_TEST_EL1_CORE_ID_018    (17)
#define A53_STL_TEST_EL1_CORE_ID_019    (18)
#define A53_STL_TEST_EL1_CORE_ID_020    (19)
#define A53_STL_TEST_EL1_CORE_ID_021    (20)
#define A53_STL_TEST_EL1_CORE_ID_022    (21)
#define A53_STL_TEST_EL1_CORE_ID_023    (22)
#define A53_STL_TEST_EL1_CORE_ID_024    (23)
#define A53_STL_TEST_EL1_CORE_ID_025    (24)
#define A53_STL_TEST_EL1_CORE_ID_026    (25)
#define A53_STL_TEST_EL1_CORE_ID_027    (26)
#define A53_STL_TEST_EL1_CORE_ID_028    (27)
#define A53_STL_TEST_EL1_CORE_ID_029    (28)
#define A53_STL_TEST_EL1_CORE_ID_030    (29)
#define A53_STL_TEST_EL1_CORE_ID_031    (30)
#define A53_STL_TEST_EL1_CORE_ID_032    (31)
#define A53_STL_TEST_EL1_CORE_ID_033    (32)
#define A53_STL_TEST_EL1_CORE_ID_034    (33)
#define A53_STL_TEST_EL1_CORE_ID_041    (40)
#define A53_STL_TEST_EL1_CORE_ID_042    (41)
#define A53_STL_TEST_EL1_CORE_ID_043    (42)
#define A53_STL_TEST_EL1_CORE_ID_044    (43)
#define A53_STL_TEST_EL1_CORE_ID_045    (44)
#define A53_STL_TEST_EL1_CORE_ID_046    (45)
#define A53_STL_TEST_EL1_CORE_ID_047    (46)
#define A53_STL_TEST_EL1_CORE_ID_048    (47)
#define A53_STL_TEST_EL1_CORE_ID_049    (48)
#define A53_STL_TEST_EL1_CORE_ID_050    (49)
#define A53_STL_TEST_EL1_CORE_ID_051    (50)
#define A53_STL_TEST_EL2_CORE_ID_001    (0)
#define A53_STL_TEST_EL2_CORE_ID_002    (1)
#define A53_STL_TEST_EL2_CORE_ID_003    (2)
#define A53_STL_TEST_EL2_CORE_ID_004    (3)
#define A53_STL_TEST_EL2_CORE_ID_005    (4)
#define A53_STL_TEST_EL2_CORE_ID_006    (5)
#define A53_STL_TEST_EL2_CORE_ID_007    (6)
#define A53_STL_TEST_EL2_CORE_ID_008    (7)
#define A53_STL_TEST_EL2_CORE_ID_009    (8)
#define A53_STL_TEST_EL2_CORE_ID_010    (9)
#define A53_STL_TEST_EL2_CORE_ID_011    (10)
#define A53_STL_TEST_EL2_CORE_ID_012    (11)
#define A53_STL_TEST_EL2_CORE_ID_013    (12)
#define A53_STL_TEST_EL2_CORE_ID_014    (13)
#define A53_STL_TEST_EL2_CORE_ID_015    (14)
#define A53_STL_TEST_EL2_CORE_ID_016    (15)
#define A53_STL_TEST_EL2_CORE_ID_017    (16)
#define A53_STL_TEST_EL2_CORE_ID_018    (17)
#define A53_STL_TEST_EL2_CORE_ID_019    (18)
#define A53_STL_TEST_EL2_CORE_ID_020    (19)
#define A53_STL_TEST_EL2_CORE_ID_021    (20)
#define A53_STL_TEST_EL2_CORE_ID_022    (21)
#define A53_STL_TEST_EL2_CORE_ID_023    (22)
#define A53_STL_TEST_EL2_CORE_ID_024    (23)
#define A53_STL_TEST_EL2_CORE_ID_025    (24)
#define A53_STL_TEST_EL2_CORE_ID_026    (25)
#define A53_STL_TEST_EL2_CORE_ID_027    (26)
#define A53_STL_TEST_EL2_CORE_ID_028    (27)
#define A53_STL_TEST_EL2_CORE_ID_029    (28)
#define A53_STL_TEST_EL2_CORE_ID_030    (29)
#define A53_STL_TEST_EL2_CORE_ID_031    (30)
#define A53_STL_TEST_EL2_CORE_ID_032    (31)
#define A53_STL_TEST_EL2_CORE_ID_033    (32)
#define A53_STL_TEST_EL2_CORE_ID_034    (33)
#define A53_STL_TEST_EL2_CORE_ID_041    (40)
#define A53_STL_TEST_EL2_CORE_ID_042    (41)
#define A53_STL_TEST_EL2_CORE_ID_043    (42)
#define A53_STL_TEST_EL2_CORE_ID_044    (43)
#define A53_STL_TEST_EL2_CORE_ID_045    (44)
#define A53_STL_TEST_EL2_CORE_ID_046    (45)
#define A53_STL_TEST_EL2_CORE_ID_047    (46)
#define A53_STL_TEST_EL2_CORE_ID_048    (47)
#define A53_STL_TEST_EL2_CORE_ID_049    (48)
#define A53_STL_TEST_EL2_CORE_ID_050    (49)
#define A53_STL_TEST_EL2_CORE_ID_051    (50)
#define A53_STL_TEST_EL3_CORE_ID_035    (34)
#define A53_STL_TEST_EL3_CORE_ID_036    (35)
#define A53_STL_TEST_EL3_CORE_ID_037    (36)
#define A53_STL_TEST_EL3_CORE_ID_038    (37)
#define A53_STL_TEST_EL3_CORE_ID_039    (38)
#define A53_STL_TEST_EL3_CORE_ID_040    (39)
#define A53_STL_TEST_EL3_CORE_ID_052    (51)
#define A53_STL_TEST_EL3_CORE_ID_053    (52)
#define A53_STL_TEST_EL3_CORE_ID_054    (53)
#define A53_STL_TEST_EL3_GIC_ID_001     (0)
#define A53_STL_TEST_EL3_GIC_ID_002     (1)

// Enabled HW parts definition

#define A53_STL_EL1_CORE_001_ENA
#define A53_STL_EL1_CORE_002_ENA
#define A53_STL_EL1_CORE_003_ENA
#define A53_STL_EL1_CORE_004_ENA
#define A53_STL_EL1_CORE_005_ENA
#define A53_STL_EL1_CORE_006_ENA
#define A53_STL_EL1_CORE_007_ENA
#define A53_STL_EL1_CORE_008_ENA
#define A53_STL_EL1_CORE_009_ENA
#define A53_STL_EL1_CORE_010_ENA
#define A53_STL_EL1_CORE_011_ENA
#define A53_STL_EL1_CORE_012_ENA
#define A53_STL_EL1_CORE_013_ENA
#define A53_STL_EL1_CORE_014_ENA
#define A53_STL_EL1_CORE_015_ENA
#define A53_STL_EL1_CORE_016_ENA
#define A53_STL_EL1_CORE_017_ENA
#define A53_STL_EL1_CORE_018_ENA
#define A53_STL_EL1_CORE_019_ENA
#define A53_STL_EL1_CORE_020_ENA
#define A53_STL_EL1_CORE_021_ENA
#define A53_STL_EL1_CORE_022_ENA
#define A53_STL_EL1_CORE_023_ENA
#define A53_STL_EL1_CORE_024_ENA
#define A53_STL_EL1_CORE_025_ENA
#define A53_STL_EL1_CORE_026_ENA
#define A53_STL_EL1_CORE_027_ENA
#define A53_STL_EL1_CORE_028_ENA
#define A53_STL_EL1_CORE_029_ENA
#define A53_STL_EL1_CORE_030_ENA
#define A53_STL_EL1_CORE_031_ENA
#define A53_STL_EL1_CORE_032_ENA
#define A53_STL_EL1_CORE_033_ENA
#define A53_STL_EL1_CORE_034_ENA
#define A53_STL_EL1_CORE_041_ENA
#define A53_STL_EL1_CORE_042_ENA
#define A53_STL_EL1_CORE_043_ENA
#define A53_STL_EL1_CORE_044_ENA
#define A53_STL_EL1_CORE_045_ENA
#define A53_STL_EL1_CORE_046_ENA
#define A53_STL_EL1_CORE_047_ENA
#define A53_STL_EL1_CORE_048_ENA
#define A53_STL_EL1_CORE_049_ENA
#define A53_STL_EL1_CORE_050_ENA
#define A53_STL_EL1_CORE_051_ENA
/*
#define A53_STL_EL2_CORE_001_ENA
#define A53_STL_EL2_CORE_002_ENA
#define A53_STL_EL2_CORE_003_ENA
#define A53_STL_EL2_CORE_004_ENA
#define A53_STL_EL2_CORE_005_ENA
#define A53_STL_EL2_CORE_006_ENA
#define A53_STL_EL2_CORE_007_ENA
#define A53_STL_EL2_CORE_008_ENA
#define A53_STL_EL2_CORE_009_ENA
#define A53_STL_EL2_CORE_010_ENA
#define A53_STL_EL2_CORE_011_ENA
#define A53_STL_EL2_CORE_012_ENA
#define A53_STL_EL2_CORE_013_ENA
#define A53_STL_EL2_CORE_014_ENA
#define A53_STL_EL2_CORE_015_ENA
#define A53_STL_EL2_CORE_016_ENA
#define A53_STL_EL2_CORE_017_ENA
#define A53_STL_EL2_CORE_018_ENA
#define A53_STL_EL2_CORE_019_ENA
#define A53_STL_EL2_CORE_020_ENA
#define A53_STL_EL2_CORE_021_ENA
#define A53_STL_EL2_CORE_022_ENA
#define A53_STL_EL2_CORE_023_ENA
#define A53_STL_EL2_CORE_024_ENA
#define A53_STL_EL2_CORE_025_ENA
#define A53_STL_EL2_CORE_026_ENA
#define A53_STL_EL2_CORE_027_ENA
#define A53_STL_EL2_CORE_028_ENA
#define A53_STL_EL2_CORE_029_ENA
#define A53_STL_EL2_CORE_030_ENA
#define A53_STL_EL2_CORE_031_ENA
#define A53_STL_EL2_CORE_032_ENA
#define A53_STL_EL2_CORE_033_ENA
#define A53_STL_EL2_CORE_034_ENA
#define A53_STL_EL2_CORE_041_ENA
#define A53_STL_EL2_CORE_042_ENA
#define A53_STL_EL2_CORE_043_ENA
#define A53_STL_EL2_CORE_044_ENA
#define A53_STL_EL2_CORE_045_ENA
#define A53_STL_EL2_CORE_046_ENA
#define A53_STL_EL2_CORE_047_ENA
#define A53_STL_EL2_CORE_048_ENA
#define A53_STL_EL2_CORE_049_ENA
#define A53_STL_EL2_CORE_050_ENA
#define A53_STL_EL2_CORE_051_ENA
#define A53_STL_EL3_CORE_035_ENA
#define A53_STL_EL3_CORE_036_ENA
#define A53_STL_EL3_CORE_037_ENA
#define A53_STL_EL3_CORE_038_ENA
#define A53_STL_EL3_CORE_039_ENA
#define A53_STL_EL3_CORE_040_ENA
#define A53_STL_EL3_CORE_052_ENA
#define A53_STL_EL3_CORE_053_ENA
#define A53_STL_EL3_CORE_054_ENA
#define A53_STL_EL3_GIC_001_ENA
#define A53_STL_EL3_GIC_002_ENA
*/
// Switch case constants definition
#define A53_STL_CASE_EL1_CORE_ID_001    (0)
#define A53_STL_CASE_EL1_CORE_ID_002    (1)
#define A53_STL_CASE_EL1_CORE_ID_003    (2)
#define A53_STL_CASE_EL1_CORE_ID_004    (3)
#define A53_STL_CASE_EL1_CORE_ID_005    (4)
#define A53_STL_CASE_EL1_CORE_ID_006    (5)
#define A53_STL_CASE_EL1_CORE_ID_007    (6)
#define A53_STL_CASE_EL1_CORE_ID_008    (7)
#define A53_STL_CASE_EL1_CORE_ID_009    (8)
#define A53_STL_CASE_EL1_CORE_ID_010    (9)
#define A53_STL_CASE_EL1_CORE_ID_011    (10)
#define A53_STL_CASE_EL1_CORE_ID_012    (11)
#define A53_STL_CASE_EL1_CORE_ID_013    (12)
#define A53_STL_CASE_EL1_CORE_ID_014    (13)
#define A53_STL_CASE_EL1_CORE_ID_015    (14)
#define A53_STL_CASE_EL1_CORE_ID_016    (15)
#define A53_STL_CASE_EL1_CORE_ID_017    (16)
#define A53_STL_CASE_EL1_CORE_ID_018    (17)
#define A53_STL_CASE_EL1_CORE_ID_019    (18)
#define A53_STL_CASE_EL1_CORE_ID_020    (19)
#define A53_STL_CASE_EL1_CORE_ID_021    (20)
#define A53_STL_CASE_EL1_CORE_ID_022    (21)
#define A53_STL_CASE_EL1_CORE_ID_023    (22)
#define A53_STL_CASE_EL1_CORE_ID_024    (23)
#define A53_STL_CASE_EL1_CORE_ID_025    (24)
#define A53_STL_CASE_EL1_CORE_ID_026    (25)
#define A53_STL_CASE_EL1_CORE_ID_027    (26)
#define A53_STL_CASE_EL1_CORE_ID_028    (27)
#define A53_STL_CASE_EL1_CORE_ID_029    (28)
#define A53_STL_CASE_EL1_CORE_ID_030    (29)
#define A53_STL_CASE_EL1_CORE_ID_031    (30)
#define A53_STL_CASE_EL1_CORE_ID_032    (31)
#define A53_STL_CASE_EL1_CORE_ID_033    (32)
#define A53_STL_CASE_EL1_CORE_ID_034    (33)
#define A53_STL_CASE_EL1_CORE_ID_041    (34)
#define A53_STL_CASE_EL1_CORE_ID_042    (35)
#define A53_STL_CASE_EL1_CORE_ID_043    (36)
#define A53_STL_CASE_EL1_CORE_ID_044    (37)
#define A53_STL_CASE_EL1_CORE_ID_045    (38)
#define A53_STL_CASE_EL1_CORE_ID_046    (39)
#define A53_STL_CASE_EL1_CORE_ID_047    (40)
#define A53_STL_CASE_EL1_CORE_ID_048    (41)
#define A53_STL_CASE_EL1_CORE_ID_049    (42)
#define A53_STL_CASE_EL1_CORE_ID_050    (43)
#define A53_STL_CASE_EL1_CORE_ID_051    (44)
#define A53_STL_CASE_EL2_CORE_ID_001    (0)
#define A53_STL_CASE_EL2_CORE_ID_002    (1)
#define A53_STL_CASE_EL2_CORE_ID_003    (2)
#define A53_STL_CASE_EL2_CORE_ID_004    (3)
#define A53_STL_CASE_EL2_CORE_ID_005    (4)
#define A53_STL_CASE_EL2_CORE_ID_006    (5)
#define A53_STL_CASE_EL2_CORE_ID_007    (6)
#define A53_STL_CASE_EL2_CORE_ID_008    (7)
#define A53_STL_CASE_EL2_CORE_ID_009    (8)
#define A53_STL_CASE_EL2_CORE_ID_010    (9)
#define A53_STL_CASE_EL2_CORE_ID_011    (10)
#define A53_STL_CASE_EL2_CORE_ID_012    (11)
#define A53_STL_CASE_EL2_CORE_ID_013    (12)
#define A53_STL_CASE_EL2_CORE_ID_014    (13)
#define A53_STL_CASE_EL2_CORE_ID_015    (14)
#define A53_STL_CASE_EL2_CORE_ID_016    (15)
#define A53_STL_CASE_EL2_CORE_ID_017    (16)
#define A53_STL_CASE_EL2_CORE_ID_018    (17)
#define A53_STL_CASE_EL2_CORE_ID_019    (18)
#define A53_STL_CASE_EL2_CORE_ID_020    (19)
#define A53_STL_CASE_EL2_CORE_ID_021    (20)
#define A53_STL_CASE_EL2_CORE_ID_022    (21)
#define A53_STL_CASE_EL2_CORE_ID_023    (22)
#define A53_STL_CASE_EL2_CORE_ID_024    (23)
#define A53_STL_CASE_EL2_CORE_ID_025    (24)
#define A53_STL_CASE_EL2_CORE_ID_026    (25)
#define A53_STL_CASE_EL2_CORE_ID_027    (26)
#define A53_STL_CASE_EL2_CORE_ID_028    (27)
#define A53_STL_CASE_EL2_CORE_ID_029    (28)
#define A53_STL_CASE_EL2_CORE_ID_030    (29)
#define A53_STL_CASE_EL2_CORE_ID_031    (30)
#define A53_STL_CASE_EL2_CORE_ID_032    (31)
#define A53_STL_CASE_EL2_CORE_ID_033    (32)
#define A53_STL_CASE_EL2_CORE_ID_034    (33)
#define A53_STL_CASE_EL2_CORE_ID_041    (34)
#define A53_STL_CASE_EL2_CORE_ID_042    (35)
#define A53_STL_CASE_EL2_CORE_ID_043    (36)
#define A53_STL_CASE_EL2_CORE_ID_044    (37)
#define A53_STL_CASE_EL2_CORE_ID_045    (38)
#define A53_STL_CASE_EL2_CORE_ID_046    (39)
#define A53_STL_CASE_EL2_CORE_ID_047    (40)
#define A53_STL_CASE_EL2_CORE_ID_048    (41)
#define A53_STL_CASE_EL2_CORE_ID_049    (42)
#define A53_STL_CASE_EL2_CORE_ID_050    (43)
#define A53_STL_CASE_EL2_CORE_ID_051    (44)
#define A53_STL_CASE_EL3_CORE_ID_035    (0)
#define A53_STL_CASE_EL3_CORE_ID_036    (1)
#define A53_STL_CASE_EL3_CORE_ID_037    (2)
#define A53_STL_CASE_EL3_CORE_ID_038    (3)
#define A53_STL_CASE_EL3_CORE_ID_039    (4)
#define A53_STL_CASE_EL3_CORE_ID_040    (5)
#define A53_STL_CASE_EL3_CORE_ID_052    (6)
#define A53_STL_CASE_EL3_CORE_ID_053    (7)
#define A53_STL_CASE_EL3_CORE_ID_054    (8)
#define A53_STL_CASE_EL3_GIC_ID_001     (9)
#define A53_STL_CASE_EL3_GIC_ID_002     (10)
typedef unsigned int uint32_t;

// Definition of the list of tests to execute
typedef struct {
    uint32_t mode;
    uint32_t idTest;
} a53_stl_cfg_t;

// Bitmap Array for executable tests at EL1 on RT execution mode
extern const a53_stl_cfg_t a53_stl_modules_EL1[A53_STL_EL1_TOT_MODULES_MAX];
// Bitmap Array for executable tests at EL2 on RT execution mode
extern const a53_stl_cfg_t a53_stl_modules_EL2[A53_STL_EL2_TOT_MODULES_MAX];
// Bitmap Array for executable tests at EL3 on RT execution mode
extern const a53_stl_cfg_t a53_stl_modules_EL3[A53_STL_EL3_TOT_MODULES_MAX];

#endif //DRIVERS_SOC_HOBOT_CPU_CAL_SCHEDULER_INC_A53_STL_ARRAYS_H_
