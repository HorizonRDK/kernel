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

#ifndef DRIVERS_SOC_HOBOT_CPU_CAL_SCHEDULER_INC_A53_STL_GLOBAL_DEFS_H_
#define DRIVERS_SOC_HOBOT_CPU_CAL_SCHEDULER_INC_A53_STL_GLOBAL_DEFS_H_


// A53 STL caches size define
// L1 Data cache size: 8, 16, 32 or 64 kb
#define A53_STL_L1_DATA_CACHE_SIZE            (32u)
// L1 Instruction cache size: 8, 16, 32 or 64 kb
#define A53_STL_L1_INSTRUCTION_CACHE_SIZE     (32u)
// L2 cache size: 128, 256, 512, 1024 or 2048 kb
#define A53_STL_L2_CACHE_SIZE                 (512u)

// A53 STL Force Fail Functionality: 0 inactive, 1 active
//#define A53_STL_FAIL_TEST (1u)


#endif //DRIVERS_SOC_HOBOT_CPU_CAL_SCHEDULER_INC_A53_STL_GLOBAL_DEFS_H_
