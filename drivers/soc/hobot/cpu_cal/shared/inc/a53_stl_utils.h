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
//      A53_STL utility functions
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

#ifndef DRIVERS_SOC_HOBOT_CPU_CAL_SHARED_INC_A53_STL_UTILS_H_
#define DRIVERS_SOC_HOBOT_CPU_CAL_SHARED_INC_A53_STL_UTILS_H_

// Zero value constant
#define A53_STL_ZERO_VALUE            (0x00000000)
#define A53_STL_ONE_VALUE             (0x00000001)

// Scheduler init values
#define A53_STL_FCTLR_INIT_0          (0x00000000)

// FCTLR bit access masks
#define A53_STL_FCTLR_STATUS_MSK      (0x0000000F)
#define A53_STL_FCTLR_STATUS_PING     (0x00000002)
#define A53_STL_FCTLR_STATUS_DONE     (0x00000003)
#define A53_STL_FCTLR_STATUS_FAIL     (0x00000004)
#define A53_STL_FCTLR_STATUS_PDONE    (0x00000005)
#define A53_STL_FCTLR_STATUS_MSK_R    (0xFFFFFFF0)
#define A53_STL_FCTLR_OFFSET          (0x00000000)

// FPIR bit access masks
#define A53_STL_FPIR_MTID_MSK         (0x3F000000)
#define A53_STL_FPIR_STID_MSK         (0x003F0000)
#define A53_STL_FPIR_MTID_CORE        (0x00000001)
#define A53_STL_FPIR_MTID_GIC         (0x00000002)
#define A53_STL_FPIR_MTID_SHF         (0x00000018)
#define A53_STL_FPIR_STID_SHF         (0x00000010)

// FFMIR bit access masks
#define A53_STL_FFMIR_FMID_DATA_COR   (0x00000004)
#define A53_STL_FFMIR_FMID_EXCEPTION  (0x00000002)
#define A53_STL_FFMIR_OFFSET          (0x00000008)
#define A53_STL_FFMIR_FMID_CLR_MSK    (0xFFFFFFF0)
#ifdef CCODE

// Definition of the null pointer
#define A53_STL_NULL                  (0)

// Function to get the CPU identifier
extern uint32_t a53_stl_getcpuid(void);

// Function to set data corruption error in the memory mapped registers FFMIR
extern void a53_stl_set_regs_error(void);

// Function to set PING status in the memory mapped registers FCTLR
extern void a53_stl_set_fctlr_ping(void);

// Function to set exception error in the memory mapped registers FFMIR
extern void a53_stl_set_ffmir_exception(void);

// Function fmov to d2-d17 from x2-x15
extern uint32_t fmov_d2d17_x2x15(void);

// Function fmov to v2-v17 from x2-x15
extern uint32_t fmov_vector_v2v17_x2x15(void);

// Function fmov to x18-x25 from v18-v25
extern uint32_t fmov_x18x25_v18v25(void);

// Function fmov to d0-d23 from x2-x16
extern uint32_t fmov_d0d23_x2x16(void);

// Function fmov to d2-d9 from x2-x9
extern uint32_t fmov_d2d9_x2x9(void);

// Function fmov to d2-d17 from x4-x17
extern uint32_t fmov_d2d17_x4x17(void);

// Function fmov to x18-x25 from d18-d25
extern uint32_t fmov_x18x25_d18d25(void);

// Function fmov to d1-d4 from x1-x2
extern uint32_t fmov_d1d4_x1x2(void);

// Function fmov to v1-v4 from x3-x4
extern uint32_t fmov_v1v4_x3x4(void);

// Function fmov to d1-d8 from x1-x4
extern uint32_t fmov_d1d8_x1x4(void);

// Function fmov to v1-v8 from x1-x4
extern uint32_t fmov_vector_v1v8_x1x4(void);

// Function fmov to d1-d8 from x1-x8
extern uint32_t fmov_d1d8_x1x8(void);

// Function fmov to v1-v8 from x1-x8
extern uint32_t fmov_v1v8_x1x8(void);

// Function fmov to v1-v4 from x1-x2
extern uint32_t fmov_v1v4_x1x2(void);

// Function fmov to v1-v4 from x1-x2
extern uint32_t fmov_d1d8_x1x2x3x4(void);

// Function to check results of x26,x27,x28and x29
extern uint32_t check_x26x29_x18x24(void);

// Function to initialize the value x1-x29 to x0
extern uint32_t mov_x1x29_x0(void);

// Function to check the value x1-x29 to x0
extern uint32_t check_x1x29_x0(void);

// Function to check the value x1-x29 to x0
extern uint32_t cmp_x1x29_x0(void);

// Function to check the value x26-x29 to x15-x4 and x18-x24
extern uint32_t check_x26x29_x15x8_x18x24(void);

// Function to move the value to x15-x8 from d18-d25
extern uint32_t fmov_x15x8_d18d25(void);

// Function to move the value to x1-x29 from v0-v28
extern uint32_t fmov_x1x29_v0v28(void);


#endif

#endif //DRIVERS_SOC_HOBOT_CPU_CAL_SHARED_INC_A53_STL_UTILS_H_
