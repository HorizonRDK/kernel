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
//      A53_STL APIs signatures
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


#ifndef DRIVERS_SOC_HOBOT_CPU_CAL_SCHEDULER_INC_A53_STL_H_
#define DRIVERS_SOC_HOBOT_CPU_CAL_SCHEDULER_INC_A53_STL_H_

#define CCODE
typedef unsigned int uint32_t;

// Definition of the EL
#define  A53_STL_EL0_EL1  (1)
#define  A53_STL_EL2      (2)
#define  A53_STL_EL3      (3)

// A53 STL execution modes
#define A53_STL_OOR_MODE       (1u)
#define A53_STL_RT_MODE        (2u)

// Result constants
#define A53_STL_RET_OK    (0x900Du)
#define A53_STL_ERROR     (0xBADu)

// A53 STL memory mapped registers structure type definition
typedef struct a53_stl_state {
    // Offset: 0x000 (R/W) Fault Control Register
    volatile uint32_t fctlr;
    // Offset: 0x004 (R/W) Fault Partition Identifier Register
    volatile uint32_t fpir;
    // Offset: 0x008 (R/W) Fault Failure Mode Identification Register
    volatile uint32_t ffmri;
} a53_stl_state_t;

// A53 STL structure type definition
typedef struct a53_stl_type {
    uint32_t start;
    uint32_t end;
    uint32_t mode;
    uint32_t el;
    a53_stl_state_t * stateRegsBase;
    uint32_t  (*cfg)(uint32_t mode, uint32_t * const base);
    uint32_t  (*set)(const uint32_t mode,
					const uint32_t start,
					const uint32_t end,
					const uint32_t el);
    uint32_t  (*stl)(void);
}a53_stl_t;

//====================================================================================================================================
//   Function: A53_STL
//      A53 STL scheduler
//
//   Parameters:
//      None
//
//   Returns:
//      Global pass/fail result of all executed MODs:
//			A53_STL_RET_OK (0x900D) Test execution success;
//			A53_STL_ERROR (0xBAD) Test execution failed
//====================================================================================================================================//
extern uint32_t A53_STL(void);

//====================================================================================================================================
//   Function: A53_STL_init
//      Init index of MOD execution
//
//   Parameters:
//      mode - Unused
//      base - Base address of the memory area used for
//				tracking STL-state information and reporting failure-modes
//
//   Returns:
//      Global pass/fail result of all executed MODs:
//			A53_STL_RET_OK (0x900D) Test execution success;
//			A53_STL_ERROR (0xBAD) Test execution failed
//====================================================================================================================================//
extern uint32_t A53_STL_init(uint32_t mode, uint32_t * const base);

//====================================================================================================================================
//   Function: A53_STL_setParam
//      Initialize the behavior of the A53 STL
//
//   Parameters:
//      mode   - Indicates the mode of execution:
//					Run-Time (RT) if it is set to 0 ;
//					Out Of Reset (OOR) if it is set to 1
//      start - Indicates the starting test ID
//      end   - Indicates the last test ID
//      el    - Indicates the EL
//
//   Returns:
//      Global pass/fail result of all executed MODs:
//			A53_STL_RET_OK (0x900D) Test execution success;
//			A53_STL_ERROR (0xBAD) Test execution failed
//====================================================================================================================================//
extern uint32_t A53_STL_setParam(uint32_t mode,
										uint32_t start,
										uint32_t end,
										uint32_t el);

#endif //DRIVERS_SOC_HOBOT_CPU_CAL_SCHEDULER_INC_A53_STL_H_

