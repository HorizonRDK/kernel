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
//      A53_STL function implementation
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


// System inclusions
//#include <stdint.h>
#include <linux/types.h>
#include <linux/kernel.h>

// STL library inclusions
#include "../inc/a53_stl.h"
#include "../inc/a53_stl_global_defs.h"
#include "../inc/a53_stl_arrays.h"
#include "../../shared/inc/a53_stl_utils.h"

// SW modules inclusion
//#include "../../stl_core/inc/a53_stl_core_p006_n001.h"
//#include "../../stl_core/inc/a53_stl_core_p007_n001.h"

#include "../../stl_core/inc/a53_stl_core_p001_n001.h"
#include "../../stl_core/inc/a53_stl_core_p002_n001.h"
#include "../../stl_core/inc/a53_stl_core_p003_n001.h"
#include "../../stl_core/inc/a53_stl_core_p004_n001.h"
#include "../../stl_core/inc/a53_stl_core_p005_n001.h"
#include "../../stl_core/inc/a53_stl_core_p006_n001.h"
#include "../../stl_core/inc/a53_stl_core_p007_n001.h"
#include "../../stl_core/inc/a53_stl_core_p008_n001.h"
#include "../../stl_core/inc/a53_stl_core_p009_n001.h"
#include "../../stl_core/inc/a53_stl_core_p010_n001.h"
#include "../../stl_core/inc/a53_stl_core_p011_n001.h"
#include "../../stl_core/inc/a53_stl_core_p012_n001.h"
#include "../../stl_core/inc/a53_stl_core_p013_n001.h"
#include "../../stl_core/inc/a53_stl_core_p014_n001.h"
#include "../../stl_core/inc/a53_stl_core_p015_n001.h"
#include "../../stl_core/inc/a53_stl_core_p016_n001.h"
#include "../../stl_core/inc/a53_stl_core_p017_n001.h"
#include "../../stl_core/inc/a53_stl_core_p018_n001.h"
#include "../../stl_core/inc/a53_stl_core_p019_n001.h"
#include "../../stl_core/inc/a53_stl_core_p020_n001.h"
#include "../../stl_core/inc/a53_stl_core_p021_n001.h"
#include "../../stl_core/inc/a53_stl_core_p022_n001.h"
#include "../../stl_core/inc/a53_stl_core_p023_n001.h"
#include "../../stl_core/inc/a53_stl_core_p024_n001.h"
#include "../../stl_core/inc/a53_stl_core_p025_n001.h"
#include "../../stl_core/inc/a53_stl_core_p026_n001.h"
#include "../../stl_core/inc/a53_stl_core_p027_n001.h"
#include "../../stl_core/inc/a53_stl_core_p028_n001.h"
#include "../../stl_core/inc/a53_stl_core_p029_n001.h"
#include "../../stl_core/inc/a53_stl_core_p030_n001.h"
#include "../../stl_core/inc/a53_stl_core_p031_n001.h"
#include "../../stl_core/inc/a53_stl_core_p032_n001.h"
#include "../../stl_core/inc/a53_stl_core_p033_n001.h"
#include "../../stl_core/inc/a53_stl_core_p034_n001.h"
#include "../../stl_core/inc/a53_stl_core_p035_n001.h"
#include "../../stl_core/inc/a53_stl_core_p036_n001.h"
#include "../../stl_core/inc/a53_stl_core_p037_n001.h"
#include "../../stl_core/inc/a53_stl_core_p038_n001.h"
#include "../../stl_core/inc/a53_stl_core_p039_n001.h"
#include "../../stl_core/inc/a53_stl_core_p040_n001.h"
#include "../../stl_core/inc/a53_stl_core_p041_n001.h"
#include "../../stl_core/inc/a53_stl_core_p042_n001.h"
#include "../../stl_core/inc/a53_stl_core_p043_n001.h"
#include "../../stl_core/inc/a53_stl_core_p044_n001.h"
#include "../../stl_core/inc/a53_stl_core_p045_n001.h"
#include "../../stl_core/inc/a53_stl_core_p046_n001.h"
#include "../../stl_core/inc/a53_stl_core_p047_n001.h"
#include "../../stl_core/inc/a53_stl_core_p048_n001.h"
#include "../../stl_core/inc/a53_stl_core_p049_n001.h"
#include "../../stl_core/inc/a53_stl_core_p050_n001.h"
#include "../../stl_core/inc/a53_stl_core_p051_n001.h"
#include "../../stl_core/inc/a53_stl_core_p052_n001.h"
#include "../../stl_core/inc/a53_stl_core_p053_n001.h"
#include "../../stl_core/inc/a53_stl_core_p054_n001.h"
#include "../../stl_gic/inc/a53_stl_gic_p001_n001.h"
#include "../../stl_gic/inc/a53_stl_gic_p002_n001.h"

// Definition of the number of CPUs in the cluster
#define CPUS_NUM             (4u)

// Definition of test input set and execution results
// Defensive error value
#define A53_STL_UKNOWN_ERR    (0xFFFFu)
// Test ended succesfully
#define A53_STL_TEST_OK       (0xAAAAu)
// Test ended with error
#define A53_STL_TEST_ERROR    (0xBBBBu)

// Error on the input parameters
#define A53_STL_INPUT_SET_ERR (0xDDDDu)
// Error in the input test configuration
#define A53_STL_INPUT_CFG_ERR (0xCCCCu)


// A53 STL scheduler for exception level EL1
static uint32_t a53_stl_el1(const a53_stl_t * stlInput);

// A53 STL scheduler for exception level EL2
static uint32_t a53_stl_el2(const a53_stl_t * stlInput);

// A53 STL scheduler for exception level EL3
static uint32_t a53_stl_el3(const a53_stl_t * stlInput);

// Function to set the status of the STL
static uint32_t a53_stl_set_status_fctlr(
											const a53_stl_t * stlInput,
											uint32_t status);

// Function to provide the status of the STL
static uint32_t a53_stl_get_status_fctlr(
											const a53_stl_t * stlInput,
											uint32_t * ptrStatus);

// Function to provide the Main Test ID of the STL
static uint32_t a53_stl_set_mtid_fpir(
											const a53_stl_t * stlInput,
											uint32_t mtid);

// Function to provide the Sub Test ID of the STL
static uint32_t a53_stl_set_stid_fpir(
											const a53_stl_t * stlInput,
											uint32_t stid);

// Function to set the Sub Test ID of the STL
// and to set the Main Test ID of the STL
static uint32_t a53_stl_set_reg_mem(
											const a53_stl_t * stlInput,
											uint32_t mtid, uint32_t stid);

// Function to run a SW part in EL1
static uint32_t RunEL1SwPart(const a53_stl_t * stlInput, uint32_t idTest);

// Function to run a SW part in EL2
static uint32_t RunEL2SwPart(const a53_stl_t * stlInput, uint32_t idTest);

// Function to run a SW part in EL3
static uint32_t RunEL3SwPart(const a53_stl_t * stlInput, uint32_t idTest);

// Set test information into the memory mapped registers for EL1
static uint32_t SetMemRegsTestIdEL1(
											const a53_stl_t * stlInput,
											uint32_t idTest);

// Set test information into the memory mapped registers for EL2
static uint32_t SetMemRegsTestIdEL2(
											const a53_stl_t * stlInput,
											uint32_t idTest);

// Set test information into the memory mapped registers for EL3
static uint32_t SetMemRegsTestIdEL3(
											const a53_stl_t * stlInput,
											uint32_t idTest);

// Cores instances
//static a53_stl_t stlInstance[CPUS_NUM];
static a53_stl_t stlInstance;

//====================================================================================================================================
//   Function: A53_STL
//      A53 scheduler
//
//   Parameters:
//      None
//
//   Returns: uint32_t
//      Global pass/fail result of all executed SW parts:
//			A53_STL_RET_OK (0x900D) Execution success;
//			A53_STL_ERROR (0xBAD) Execution failed
//====================================================================================================================================//
uint32_t A53_STL(void) {
    // Function return value
    uint32_t retVal;
    // Cpu identificator
    uint32_t cpuId;
    // Pointer to the CPU istance
    const a53_stl_t * ptrStlInstance;

    // Defensive initialization
    retVal = A53_STL_ERROR;
    cpuId  = CPUS_NUM;
    ptrStlInstance = A53_STL_NULL;

    // Get the CPU identifier
//    cpuId = a53_stl_getcpuid();
	cpuId = 0;
    // Check the CPU identifier
    if (cpuId < CPUS_NUM) {
//        ptrStlInstance = (const a53_stl_t *)&stlInstance[cpuId];
//		  switch( stlInstance[cpuId].el) {
		ptrStlInstance = (const a53_stl_t *)&stlInstance;
		switch( stlInstance.el ) {
            // Run for EL0-EL1
            case A53_STL_EL0_EL1:
                retVal = a53_stl_el1(ptrStlInstance);
                break;

            // Run for EL2
            case A53_STL_EL2:
                retVal = a53_stl_el2(ptrStlInstance);
                break;

            // Run for EL3
            case A53_STL_EL3:
                retVal = a53_stl_el3(ptrStlInstance);
                break;

            default:
                retVal = A53_STL_ERROR;
                break;
        }
    } else {
        retVal = A53_STL_ERROR;
    }
    return retVal;
}

//====================================================================================================================================
//   Function: A53_STL_init
//      Init index of MOD execution
//
//   Parameters:
//      mode - Unused
//      base - Base address of the memory area used for
//					tracking STL-state information and reporting failure-modes
//
//   Returns: uint32_t
//      Global pass/fail result of library initialization:
//			A53_STL_RET_OK (0x900D) Initialization success;
//			A53_STL_ERROR (0xBAD) Initialization failed
//====================================================================================================================================//
uint32_t A53_STL_init(uint32_t mode, uint32_t * const base) {
    // Function return value
    uint32_t retVal;
    // Cpu identificator
    uint32_t cpuId;

    // Defensive initialization
    retVal = A53_STL_ERROR;
    cpuId  = CPUS_NUM;

    // Get the CPU identifier
//	  cpuId = a53_stl_getcpuid();
	cpuId = 0;

    // Check the CPU identifier
    if (cpuId < CPUS_NUM) {
//        stlInstance[cpuId].stateRegsBase = (a53_stl_state_t *) base;
		stlInstance.stateRegsBase = (a53_stl_state_t *) base;
        retVal = A53_STL_RET_OK;
    } else {
        retVal = A53_STL_ERROR;
    }

    return retVal;
}

//====================================================================================================================================
//   Function: A53_STL_setParam
//      Initialize the behavior of the A53 STL
//
//   Parameters:
//      mode  - Indicates the mode of execution:
//				Run-Time (RT) if it is set to 0 ; Out Of Reset (OOR) if it is set to 1
//      start - Indicates the starting test ID
//      end   - Indicates the last test ID
//      el    - Indicates the EL
//
//   Returns: uint32_t
//      Global pass/fail result of parameters setting:
//			A53_STL_RET_OK (0x900D) Setting success;
//			A53_STL_ERROR (0xBAD) Setting failed
//====================================================================================================================================//
uint32_t A53_STL_setParam(
								uint32_t mode,
								uint32_t start,
								uint32_t end,
								uint32_t el) {
    // Function return value
    uint32_t retVal;
    // Cpu identificator
    uint32_t cpuId;

    // Defensive initialization
    retVal = A53_STL_ERROR;
    cpuId  = CPUS_NUM;

    // Get the CPU identifier
    cpuId = a53_stl_getcpuid();

    // Check the CPU identifier
    if ( ( cpuId < CPUS_NUM )
		&& ( (mode == A53_STL_OOR_MODE) || (mode == A53_STL_RT_MODE) ) ) {
        switch ( el ) {
            // Check parameter for EL0-EL1
            case A53_STL_EL0_EL1:
                if ((start < A53_STL_EL1_TOT_MODULES_MAX)
					&& (end < A53_STL_EL1_TOT_MODULES_MAX)) {
                    //Set the success of the parameters
                    retVal = A53_STL_RET_OK;
                }
                break;

            // Check parameter for EL2
            case A53_STL_EL2:
                if ((start < A53_STL_EL2_TOT_MODULES_MAX)
					&& (end < A53_STL_EL2_TOT_MODULES_MAX)) {
                    //Set the success of the parameters
                    retVal = A53_STL_RET_OK;
                }
                break;

            // Check parameter for EL3
            case A53_STL_EL3:
                if ((start < A53_STL_EL3_TOT_MODULES_MAX)
					&& (end < A53_STL_EL3_TOT_MODULES_MAX)) {
                    //Set the success of the parameters
                    retVal = A53_STL_RET_OK;
                }
                break;

            default:
                retVal = A53_STL_ERROR;
                break;
        }
    }
    if ( retVal == A53_STL_RET_OK ) {
        // Set the working parameters
/*
        stlInstance[cpuId].start = start;
        stlInstance[cpuId].end   = end;
        stlInstance[cpuId].mode  = mode;
        stlInstance[cpuId].el    = el;
*/
		stlInstance.start = start;
        stlInstance.end   = end;
        stlInstance.mode  = mode;
        stlInstance.el    = el;
    }

    return retVal;
}
//====================================================================================================================================
//   Function: a53_stl_el1
//      A53 STL scheduler for exception level EL1
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//
//   Returns: uint32_t
//      Global pass/fail result of all executed SW parts:
//			A53_STL_RET_OK (0x900D) SW parts execution success;
//			A53_STL_ERROR (0xBAD) SW parts execution failed
//====================================================================================================================================//
static uint32_t a53_stl_el1(const a53_stl_t * stlInput) {
    // Function return value
    uint32_t returnVal;
    uint32_t i;
    // Test return value
    uint32_t resultTest;
    // status of FCTRL register
    uint32_t fctlrStatus;

    // Initialization
    returnVal   = ~A53_STL_RET_OK;
    resultTest  = A53_STL_TEST_ERROR;
    fctlrStatus = (uint32_t)A53_STL_FCTLR_INIT_0;

    //Check the input parameters
    if ( ( stlInput->start < A53_STL_EL1_TOT_MODULES_MAX )
		&& (stlInput->end < A53_STL_EL1_TOT_MODULES_MAX)
		&& (stlInput->start <= stlInput->end)
		&& ( (stlInput->mode == A53_STL_RT_MODE)
			|| (stlInput->mode == A53_STL_OOR_MODE) ) ) {
        // Reset the status of the next running tests
        resultTest = A53_STL_TEST_OK;
    }

    // Run tests
    i = stlInput->start;
    while ( (i <= stlInput->end) && (resultTest == A53_STL_TEST_OK) ) {
        // Check the consistency of the current test
        if (((a53_stl_modules_EL1[i].mode
				& ~((uint32_t)(A53_STL_OOR_MODE | A53_STL_RT_MODE)))
				!= A53_STL_ZERO_VALUE)
			|| (a53_stl_modules_EL1[i].mode == A53_STL_ZERO_VALUE) ) {
            resultTest = A53_STL_INPUT_CFG_ERR;
        }
        // If the test is correctly configured
        // and check the requested execution mode
        if ( ( resultTest == A53_STL_TEST_OK )
			&& (a53_stl_modules_EL1[i].mode & stlInput->mode) != A53_STL_ZERO_VALUE ) {
            // Run the SW part for EL1
            resultTest = RunEL1SwPart(stlInput, a53_stl_modules_EL1[i].idTest);

            // Schedule current module ID
            if (resultTest == A53_STL_TEST_OK) {
                // Read the status of the current test in FCTLR
                resultTest = a53_stl_get_status_fctlr(stlInput, &fctlrStatus);
                // Check the test result
                if ( ( resultTest != A53_STL_TEST_OK)
					|| (fctlrStatus != A53_STL_FCTLR_STATUS_PDONE) ) {
                    // Set return value on scheduler exit
                    resultTest = A53_STL_TEST_ERROR;
                }
            }
// If fault injection is active (1), corrupt return_val
#ifdef A53_STL_FAIL_TEST

			if(test_count < 5) {
				test_count++;
				resultTest = A53_STL_TEST_ERROR;
			}

//            resultTest = A53_STL_TEST_ERROR;
#endif
        }
        i++;
    }
    // Check the return value of the test
    // to set the return value of the function
    if (resultTest == A53_STL_TEST_OK) {
        resultTest = a53_stl_set_status_fctlr(stlInput,
											(uint32_t)A53_STL_FCTLR_STATUS_DONE);
        if ( resultTest == A53_STL_TEST_OK ) {
            returnVal = A53_STL_RET_OK;
        } else {
            // Ignore the result of the a53_stl_set_status_fctlr
            // there is already an error condition
            resultTest = a53_stl_set_status_fctlr(stlInput,
            									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
            returnVal = A53_STL_ERROR;
        }
    } else {
        // Fault result corresponding to FCTLR Register
        // STATUS[3:0] 0x4 FAIL code
        a53_stl_set_status_fctlr(stlInput, (uint32_t)A53_STL_FCTLR_STATUS_FAIL);
        returnVal = A53_STL_ERROR;
    }

    return returnVal;
}

//====================================================================================================================================
//   Function: a53_stl_el2
//      A53 STL scheduler for exception level EL2
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//
//   Returns: uint32_t
//      Global pass/fail result of all executed SW parts:
//			A53_STL_RET_OK (0x900D) SW parts execution success;
//			A53_STL_ERROR (0xBAD) SW parts execution failed
//====================================================================================================================================//
static uint32_t a53_stl_el2(const a53_stl_t * stlInput) {
    // Function return value
    uint32_t returnVal;
    uint32_t i;
    // Test return value
    uint32_t resultTest;
    // status of FCTRL register
    uint32_t fctlrStatus;

    // Defensive initialization
    returnVal   = ~A53_STL_RET_OK;
    resultTest  = A53_STL_TEST_ERROR;
    fctlrStatus = (uint32_t)A53_STL_FCTLR_INIT_0;

    //Check the input parameters
    if ( ( stlInput->start < A53_STL_EL2_TOT_MODULES_MAX )
		&& (stlInput->end < A53_STL_EL2_TOT_MODULES_MAX )
		&& (stlInput->start  <= stlInput->end)
		&& ( (stlInput->mode == A53_STL_RT_MODE)
			|| (stlInput->mode == A53_STL_OOR_MODE) ) ) {
        // Reset the status of the next running tests
        resultTest = A53_STL_TEST_OK;
    }

    // Run tests
    i = stlInput->start;

    while ( (i <= stlInput->end) && (resultTest == A53_STL_TEST_OK) ) {
        // Check the consistency of the current test
        if ((( a53_stl_modules_EL2[i].mode
				& ~((uint32_t)(A53_STL_OOR_MODE | A53_STL_RT_MODE)) )
				!= A53_STL_ZERO_VALUE )
			|| (a53_stl_modules_EL2[i].mode == A53_STL_ZERO_VALUE) ) {
            resultTest = A53_STL_INPUT_CFG_ERR;
        }
        // If the test is correctly configured
        // and check the requested execution mode
        if ( ( resultTest == A53_STL_TEST_OK )
			&& (a53_stl_modules_EL2[i].mode & stlInput->mode)
				!= A53_STL_ZERO_VALUE )  {
            // Run the SW part for EL2
            resultTest = RunEL2SwPart(stlInput, a53_stl_modules_EL2[i].idTest);

            if (resultTest == A53_STL_TEST_OK) {
                // Read the status of the current test in FCTLR
                resultTest = a53_stl_get_status_fctlr(stlInput, &fctlrStatus);
                // Check the test result
                if ( ( resultTest != A53_STL_TEST_OK)
					|| (fctlrStatus != (uint32_t)A53_STL_FCTLR_STATUS_PDONE) ) {
                    // Set return value on scheduler exit
                    resultTest = A53_STL_TEST_ERROR;
                }
            }

// If fault injection is active (1), corrupt return_val
#ifdef A53_STL_FAIL_TEST
            resultTest = A53_STL_TEST_ERROR;
#endif
        }
        i++;
    }

    // Check the return value of the test
    // to set the return value of the function
    if (resultTest == A53_STL_TEST_OK) {
        resultTest = a53_stl_set_status_fctlr(stlInput,
											(uint32_t)A53_STL_FCTLR_STATUS_DONE);
        if ( resultTest == A53_STL_TEST_OK ) {
            returnVal = A53_STL_RET_OK;
        } else {
            // Ignore the result of the a53_stl_set_status_fctlr
            // there is already an error condition
            resultTest = a53_stl_set_status_fctlr(stlInput,
            									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
            returnVal = A53_STL_ERROR;
        }
    } else {
        // Ignore the result of the a53_stl_set_status_fctlr
        // there is already an error condition
        resultTest = a53_stl_set_status_fctlr(stlInput,
        										(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
        returnVal = A53_STL_ERROR;
    }

    return returnVal;
}
//====================================================================================================================================
//   Function: a53_stl_el3
//      A53 STL scheduler for exception level EL3
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//
//   Returns: uint32_t
//      Global pass/fail result of all executed MODs:
//			A53_STL_RET_OK (0x900D) Test execution success;
//			A53_STL_ERROR (0xBAD) Test execution failed
//
//====================================================================================================================================//
static uint32_t a53_stl_el3(const a53_stl_t * stlInput) {
    // Function return value
    uint32_t returnVal;
    uint32_t i;
    // Test return value
    uint32_t resultTest;
    // status of FCTRL register
    uint32_t fctlrStatus;

    // Defensive initialization
    returnVal   = ~A53_STL_RET_OK;
    resultTest  = A53_STL_TEST_ERROR;
    fctlrStatus = (uint32_t)A53_STL_FCTLR_INIT_0;

    //Check the input parameters
    if ( ( stlInput->start < A53_STL_EL3_TOT_MODULES_MAX )
		&& (stlInput->end < A53_STL_EL3_TOT_MODULES_MAX)
		&& (stlInput->start <= stlInput->end)
		&& ( (stlInput->mode == A53_STL_RT_MODE)
				|| (stlInput->mode == A53_STL_OOR_MODE) ) ) {
        // Reset the status of the next running tests
        resultTest = A53_STL_TEST_OK;
    }

    // Run tests
    i = stlInput->start;
    while ( (i <= stlInput->end) && (resultTest == A53_STL_TEST_OK) ) {
        // Check the consistency of the current test
        if ((( a53_stl_modules_EL3[i].mode
				& ~((uint32_t)(A53_STL_OOR_MODE | A53_STL_RT_MODE)) )
				!= A53_STL_ZERO_VALUE )
			|| (a53_stl_modules_EL3[i].mode == A53_STL_ZERO_VALUE) ) {
            resultTest = A53_STL_INPUT_CFG_ERR;
        }
        // If the test is correctly configured
        // and check the requested execution mode
        if ( ( resultTest == A53_STL_TEST_OK )
			&& (a53_stl_modules_EL3[i].mode & stlInput->mode) \
				!= A53_STL_ZERO_VALUE ) {
            // Run the SW part for EL3
            resultTest = RunEL3SwPart(stlInput, a53_stl_modules_EL3[i].idTest);

            if (resultTest == A53_STL_TEST_OK) {
                // Read the status of the current test in FCTLR
                resultTest = a53_stl_get_status_fctlr(stlInput, &fctlrStatus);
                // Check the test result
                if ( (resultTest != A53_STL_TEST_OK)
					|| (fctlrStatus != (uint32_t)A53_STL_FCTLR_STATUS_PDONE) ) {
                    // Set return value on scheduler exit
                    resultTest = A53_STL_TEST_ERROR;
                }
            }
// If fault injection is active (1), corrupt return_val
#ifdef A53_STL_FAIL_TEST
            resultTest = A53_STL_TEST_ERROR;
#endif
        }
        i++;
    }

    // Check the return value of the test
    // to set the return value of the function
    if (resultTest == A53_STL_TEST_OK) {
        resultTest = a53_stl_set_status_fctlr(stlInput,
											(uint32_t)A53_STL_FCTLR_STATUS_DONE);
        if ( resultTest == A53_STL_TEST_OK ) {
            returnVal = A53_STL_RET_OK;
        } else {
            // Ignore the result of the a53_stl_set_status_fctlr
            // there is already an error condition
            resultTest = a53_stl_set_status_fctlr(stlInput,
            									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
            returnVal = A53_STL_ERROR;
        }
    } else {
        // Ignore the result of the a53_stl_set_status_fctlr
        // there is already an error condition
        resultTest = a53_stl_set_status_fctlr(stlInput,
        									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
        returnVal = A53_STL_ERROR;
    }

    return returnVal;
}

//====================================================================================================================================
//   Function: a53_stl_set_status_fctlr
//      Function to set the status of the STL
//
//   Parameters:
//      stlInput  - Pointer to the STL instance
//      status    - Status of the STL
//
//   Returns: uint32_t
//      Global pass/fail result of all executed MODs:
//			A53_STL_TEST_OK Function execution success;
//			A53_STL_TEST_ERROR Function execution failed
//
//====================================================================================================================================//
static uint32_t a53_stl_set_status_fctlr(
													const a53_stl_t * stlInput,
													uint32_t status) {
    // Function return value
    uint32_t retVal;
    // Mask to access to the FCTLR STATUS bits
    uint32_t regMask;

    //Defensive initialization to uknown error
    retVal   = A53_STL_UKNOWN_ERR;
    regMask  = A53_STL_FCTLR_STATUS_MSK;

    if  ( stlInput != A53_STL_NULL ) {
        stlInput->stateRegsBase->fctlr &= ~regMask;
        stlInput->stateRegsBase->fctlr |= (status & regMask);
        retVal = A53_STL_TEST_OK;
    } else {
        retVal = A53_STL_TEST_ERROR;
    }

    return retVal;
}

//====================================================================================================================================
//   Function: a53_stl_get_status_fctlr
//      Function to provide the status of the STL
//
//   Parameters:
//      stlInput  - Pointer to the STL instance
//      ptrStatus - Pointer to the status of the STL
//
//   Returns: uint32_t
//      Global pass/fail result of all executed MODs:
//			A53_STL_TEST_OK Function execution success;
//			A53_STL_TEST_ERROR Function execution failed
//
//====================================================================================================================================//
static uint32_t a53_stl_get_status_fctlr(
													const a53_stl_t * stlInput,
													uint32_t * ptrStatus) {
    // Function return value
    uint32_t retVal;
    // Mask to access to the FCTLR STATUS bits
    uint32_t regMask;

    //Defensive initialization to uknown error
    retVal   = A53_STL_UKNOWN_ERR;
    regMask  = A53_STL_FCTLR_STATUS_MSK;

    if ( (ptrStatus != A53_STL_NULL) && (stlInput != A53_STL_NULL) ) {
        *ptrStatus = (stlInput->stateRegsBase->fctlr & regMask);
        retVal     = A53_STL_TEST_OK;
    } else {
        //Set function error
        retVal = A53_STL_TEST_ERROR;
    }

    return retVal;
}

//====================================================================================================================================
//   Function: a53_stl_set_mtid_fpir
//      Function to set the Main Test ID of the STL
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//      mtid     - Main Test ID
//
//   Returns: uint32_t
//      Global pass/fail result of all executed MODs:
//			A53_STL_TEST_OK Function execution success;
//			A53_STL_TEST_ERROR Function execution failed
//
//====================================================================================================================================//
static uint32_t a53_stl_set_mtid_fpir(
										const a53_stl_t * stlInput,
										uint32_t mtid) {
    // Function return value
    uint32_t retVal;
    // Mask to access to the FPIR MTID bits
    uint32_t regMask;
    // Number of bits to shift to access to the FPIR MTID field
    uint32_t regShift;

    //Defensive initialization to uknown error
    retVal   = A53_STL_UKNOWN_ERR;
    regMask  = A53_STL_FPIR_MTID_MSK;
    regShift = A53_STL_FPIR_MTID_SHF;

    if  ( stlInput != A53_STL_NULL ) {
        stlInput->stateRegsBase->fpir &= ~regMask;
        stlInput->stateRegsBase->fpir |= ((mtid << regShift) & regMask);
        retVal = A53_STL_TEST_OK;
    } else {
        //Set function error
        retVal = A53_STL_TEST_ERROR;
    }

    return retVal;
}

//====================================================================================================================================
//   Function: a53_stl_set_stid_fpir
//      Function to set the Sub Test ID of the STL
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//      stid     - Sub Test ID
//
//   Returns: uint32_t
//      Global pass/fail result of all executed MODs:
//			A53_STL_TEST_OK Function execution success;
//			A53_STL_TEST_ERROR Function execution failed
//
//====================================================================================================================================//
static uint32_t a53_stl_set_stid_fpir(
											const a53_stl_t * stlInput,
											uint32_t stid) {
    // Function return value
    uint32_t retVal;
    // Mask to access to the FPIR STID bits
    uint32_t regMask;
    // Number of bits to shift to access to the FPIR STID field
    uint32_t regShift;

    //Defensive initialization to uknown error
    retVal   = A53_STL_UKNOWN_ERR;
    regMask  = A53_STL_FPIR_STID_MSK;
    regShift = A53_STL_FPIR_STID_SHF;

    if  ( stlInput != A53_STL_NULL ) {
        stlInput->stateRegsBase->fpir &= ~regMask;
        stlInput->stateRegsBase->fpir |= ((stid << regShift) & regMask);
        retVal = A53_STL_TEST_OK;
    } else {
        //Set function error
        retVal = A53_STL_TEST_ERROR;
    }

    return retVal;
}

//====================================================================================================================================
//   Function: a53_stl_set_reg_mem
//      Function to set the Sub Test ID of the STL
//			and to set the Main Test ID of the STL
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//      stid     - Sub Test ID
//      mtid     - Main Test ID
//
//   Returns: uint32_t
//      Global pass/fail result of all executed MODs:
//			A53_STL_TEST_OK Function execution success;
//			A53_STL_TEST_ERROR Function execution failed
//
//====================================================================================================================================//
// Function to set the Sub Test ID of the STL
// and to set the Main Test ID of the STL
static uint32_t a53_stl_set_reg_mem(
											const a53_stl_t * stlInput,
											uint32_t mtid, uint32_t stid) {
    // Function return value
    uint32_t retVal;

    //Defensive initialization to uknown error
    retVal = A53_STL_UKNOWN_ERR;

    if  ( stlInput != A53_STL_NULL ) {
        retVal = a53_stl_set_mtid_fpir(stlInput, mtid);
//		printk("%s %s %d %x \n",__FILE__, __func__, __LINE__, retVal);
        if ( retVal == A53_STL_TEST_OK ) {
            retVal = a53_stl_set_stid_fpir(stlInput, stid);
//			printk("%s %s %d %x \n",__FILE__, __func__, __LINE__, retVal);
        } else {
            retVal = A53_STL_TEST_ERROR;
        }
    } else {
        //Set function error
//        printk("%s %s %d %x \n",__FILE__, __func__, __LINE__, retVal);
        retVal = A53_STL_TEST_ERROR;
    }
//	printk("%s %s %d %x \n",__FILE__, __func__, __LINE__, retVal);
    return retVal;
}

//====================================================================================================================================
//   Function: RunEL3SwPart
//      Function to run a Sw part specified
//			in the input parameters for EL3
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//      idTest   - Identifier of the test to run
//
//   Returns: uint32_t
//      Global pass/fail result of executed SW part:
//			A53_STL_TEST_OK Test execution success;
//			A53_STL_TEST_ERROR Test execution failed
//
//====================================================================================================================================//
static uint32_t RunEL3SwPart(const a53_stl_t * stlInput, uint32_t idTest) {
    // Function return value
    uint32_t resultTest;

    // Defensive initialization
    resultTest  = A53_STL_TEST_ERROR;

    // Set the identification of the test to run
    resultTest = SetMemRegsTestIdEL3(stlInput, idTest);

    if (resultTest == A53_STL_TEST_OK) {
        // Schedule current module ID
        switch (idTest) {
#ifdef A53_STL_EL3_CORE_035_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_035:
                a53_stl_core_p035_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_CORE_036_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_036:
                a53_stl_core_p036_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_CORE_037_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_037:
                a53_stl_core_p037_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_CORE_038_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_038:
                a53_stl_core_p038_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_CORE_039_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_039:
                a53_stl_core_p039_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_CORE_040_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_040:
                a53_stl_core_p040_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_CORE_052_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_052:
                a53_stl_core_p052_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_CORE_053_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_053:
                a53_stl_core_p053_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_CORE_054_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_054:
                a53_stl_core_p054_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_GIC_001_ENA
            case (uint8_t) A53_STL_CASE_EL3_GIC_ID_001:
                a53_stl_gic_p001_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL3_GIC_002_ENA
            case (uint8_t) A53_STL_CASE_EL3_GIC_ID_002:
                a53_stl_gic_p002_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
            default:
                // Fault result corresponding to FCTLR Register
                resultTest = a53_stl_set_status_fctlr(stlInput,
                									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
                // Force the function return value to the error
                // also if the function above return ok
                resultTest = A53_STL_TEST_ERROR;
                break;
        }
    }

    return resultTest;
}

//====================================================================================================================================
//   Function: RunEL2SwPart
//      Function to run a Sw part specified in the input parameters for EL2
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//      idTest   - Identifier of the test to run
//
//   Returns: uint32_t
//      Global pass/fail result of executed SW part:
//			A53_STL_TEST_OK Test execution success;
//			A53_STL_TEST_ERROR Test execution failed
//
//====================================================================================================================================//
static uint32_t RunEL2SwPart(const a53_stl_t * stlInput, uint32_t idTest) {
    // Function return value
    uint32_t resultTest;

    // Defensive initialization
    resultTest  = A53_STL_TEST_ERROR;

    // Set the identification of the test to run
    resultTest = SetMemRegsTestIdEL2(stlInput, idTest);

    if (resultTest == A53_STL_TEST_OK) {
        // Schedule current module ID
        switch (idTest) {
#ifdef A53_STL_EL2_CORE_001_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_001:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_001
                a53_stl_core_p001_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_002_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_002:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_002
                a53_stl_core_p002_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_003_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_003:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_003
                a53_stl_core_p003_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_004_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_004:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_004
                a53_stl_core_p004_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_005_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_005:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_005
                a53_stl_core_p005_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_006_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_006:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_006
                a53_stl_core_p006_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_007_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_007:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_007
                a53_stl_core_p007_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_008_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_008:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_008
                a53_stl_core_p008_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_009_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_009:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_009
                a53_stl_core_p009_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_010_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_010:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_010
                a53_stl_core_p010_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_011_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_011:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_011
                a53_stl_core_p011_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_012_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_012:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_012
                a53_stl_core_p012_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_013_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_013:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_013
                a53_stl_core_p013_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_014_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_014:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_014
                a53_stl_core_p014_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_015_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_015:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_015
                a53_stl_core_p015_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_016_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_016:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_016
                a53_stl_core_p016_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_017_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_017:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_017
                a53_stl_core_p017_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_018_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_018:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_018
                a53_stl_core_p018_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_019_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_019:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_019
                a53_stl_core_p019_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_020_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_020:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_020
                a53_stl_core_p020_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_021_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_021:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_021
                a53_stl_core_p021_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_022_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_022:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_022
                a53_stl_core_p022_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_023_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_023:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_023
                a53_stl_core_p023_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_024_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_024:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_024
                a53_stl_core_p024_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL2_CORE_025_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_025:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_025
                a53_stl_core_p025_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_026_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_026:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_026
                a53_stl_core_p026_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_027_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_027:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_027
                a53_stl_core_p027_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_028_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_028:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_028
                a53_stl_core_p028_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_029_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_029:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_029
                a53_stl_core_p029_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_030_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_030:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_030
                a53_stl_core_p030_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_031_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_031:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_031
                a53_stl_core_p031_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_032_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_032:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_032
                a53_stl_core_p032_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_033_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_033:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_033
                a53_stl_core_p033_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_034_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_034:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_034
                a53_stl_core_p034_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_041_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_041:
                a53_stl_core_p041_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_042_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_042:
                a53_stl_core_p042_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_043_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_043:
                a53_stl_core_p043_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_044_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_044:
                a53_stl_core_p044_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_045_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_045:
                a53_stl_core_p045_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_046_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_046:
                a53_stl_core_p046_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_047_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_047:
                a53_stl_core_p047_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_048_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_048:
                a53_stl_core_p048_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_049_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_049:
                a53_stl_core_p049_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_050_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_050:
                a53_stl_core_p050_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL2_CORE_051_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_051:
                a53_stl_core_p051_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
            default:
                // Fault result corresponding to FCTLR Register
                resultTest = a53_stl_set_status_fctlr(stlInput,
                									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
                // Force the function return value to the error
                // also if the function above return ok
                resultTest = A53_STL_TEST_ERROR;
                break;
        }
    }

    return resultTest;
}
//====================================================================================================================================
//   Function: RunEL1SwPart
//      Function to run a Sw part specified
//			in the input parameters for EL1
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//      idTest   - Identifier of the test to run
//
//   Returns: uint32_t
//      Global pass/fail result of executed SW part:
//			A53_STL_TEST_OK Test execution success;
//			A53_STL_TEST_ERROR Test execution failed
//
//====================================================================================================================================//
static uint32_t RunEL1SwPart(const a53_stl_t * stlInput, uint32_t idTest) {
    // Function return value
    uint32_t resultTest;

    // Defensive initialization
    resultTest  = A53_STL_TEST_ERROR;

    // Set the identification of the test to run
    resultTest = SetMemRegsTestIdEL1(stlInput, idTest);

    if (resultTest == A53_STL_TEST_OK) {
        // Schedule current module ID
        switch (idTest) {
#ifdef A53_STL_EL1_CORE_001_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_001:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_001
                a53_stl_core_p001_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_002_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_002:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_002
                a53_stl_core_p002_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_003_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_003:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_003
                a53_stl_core_p003_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_004_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_004:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_004
                a53_stl_core_p004_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_005_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_005:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_005
                a53_stl_core_p005_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_006_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_006:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_006
                a53_stl_core_p006_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_007_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_007:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_007
                a53_stl_core_p007_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_008_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_008:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_008
                a53_stl_core_p008_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_009_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_009:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_009
                a53_stl_core_p009_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_010_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_010:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_010
                a53_stl_core_p010_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_011_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_011:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_011
                a53_stl_core_p011_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_012_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_012:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_012
                a53_stl_core_p012_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_013_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_013:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_013
                a53_stl_core_p013_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_014_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_014:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_014
                a53_stl_core_p014_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_015_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_015:
                // Run SW part A53_STL_CASE_EL2_CORE_ID_015
                a53_stl_core_p015_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_016_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_016:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_016
                a53_stl_core_p016_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_017_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_017:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_017
                a53_stl_core_p017_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_018_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_018:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_018
                a53_stl_core_p018_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_019_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_019:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_019
                a53_stl_core_p019_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_020_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_020:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_020
                a53_stl_core_p020_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_021_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_021:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_021
                a53_stl_core_p021_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_022_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_022:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_022
                a53_stl_core_p022_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_023_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_023:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_023
                a53_stl_core_p023_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_024_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_024:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_024
                a53_stl_core_p024_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_025_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_025:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_025
                a53_stl_core_p025_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_026_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_026:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_026
                a53_stl_core_p026_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_027_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_027:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_027
                a53_stl_core_p027_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_028_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_028:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_028
                a53_stl_core_p028_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_029_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_029:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_029
                a53_stl_core_p029_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_030_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_030:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_030
                a53_stl_core_p030_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_031_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_031:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_031
                a53_stl_core_p031_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_032_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_032:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_032
                a53_stl_core_p032_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_033_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_033:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_033
                a53_stl_core_p033_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

#ifdef A53_STL_EL1_CORE_034_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_034:
                // Run SW part A53_STL_CASE_EL1_CORE_ID_034
                a53_stl_core_p034_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_041_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_041:
                a53_stl_core_p041_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_042_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_042:
                a53_stl_core_p042_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_043_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_043:
                a53_stl_core_p043_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_044_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_044:
                a53_stl_core_p044_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_045_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_045:
                a53_stl_core_p045_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_046_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_046:
                a53_stl_core_p046_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_047_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_047:
                a53_stl_core_p047_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_048_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_048:
                a53_stl_core_p048_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_049_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_049:
                a53_stl_core_p049_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_050_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_050:
                a53_stl_core_p050_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif
#ifdef A53_STL_EL1_CORE_051_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_051:
                a53_stl_core_p051_n001((uint32_t *) stlInput->stateRegsBase);
                break;
#endif

            default:
                // Fault result corresponding to FCTLR Register
                resultTest = a53_stl_set_status_fctlr(stlInput,
                									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
                // Force the function return value to the error
                // also if the function above return ok
                resultTest = A53_STL_TEST_ERROR;
                break;
        }
    }
    return resultTest;
}

//====================================================================================================================================
//   Function: SetMemRegsTestIdEL1
//      Function to set the information of the SW part
//		under test in EL1 in the memory mapped register
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//      idTest   - Identifier of the test
//
//   Returns: uint32_t
//      Global pass/fail result of set the information:
//			A53_STL_TEST_OK Setting execution success;
//			A53_STL_TEST_ERROR Setting execution failed
//
//====================================================================================================================================//
static uint32_t SetMemRegsTestIdEL1(
											const a53_stl_t * stlInput,
											uint32_t idTest) {
    // Function return value
    uint32_t resultTest;

    // Defensive initialization
    resultTest  = A53_STL_TEST_ERROR;

    if ( stlInput != A53_STL_NULL ) {
        // Schedule current module ID
        switch (idTest) {
#ifdef A53_STL_EL1_CORE_001_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_001:
            // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_001);
                break;
#endif
#ifdef A53_STL_EL1_CORE_002_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_002:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_002);
                break;
#endif

#ifdef A53_STL_EL1_CORE_003_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_003:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_003);
                break;
#endif

#ifdef A53_STL_EL1_CORE_004_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_004:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_004);
                break;
#endif

#ifdef A53_STL_EL1_CORE_005_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_005:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_005);
                break;
#endif

#ifdef A53_STL_EL1_CORE_006_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_006:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_006);
                break;
#endif

#ifdef A53_STL_EL1_CORE_007_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_007:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_007);
                break;
#endif

#ifdef A53_STL_EL1_CORE_008_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_008:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_008);
                break;
#endif

#ifdef A53_STL_EL1_CORE_009_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_009:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_009);
                break;
#endif

#ifdef A53_STL_EL1_CORE_010_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_010:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_010);
                break;
#endif

#ifdef A53_STL_EL1_CORE_011_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_011:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_011);
                break;
#endif

#ifdef A53_STL_EL1_CORE_012_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_012:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_012);
                break;
#endif

#ifdef A53_STL_EL1_CORE_013_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_013:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_013);
                break;
#endif

#ifdef A53_STL_EL1_CORE_014_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_014:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_014);
                break;
#endif

#ifdef A53_STL_EL1_CORE_015_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_015:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_015);
                break;
#endif

#ifdef A53_STL_EL1_CORE_016_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_016:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_016);
                break;
#endif

#ifdef A53_STL_EL1_CORE_017_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_017:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_017);
                break;
#endif

#ifdef A53_STL_EL1_CORE_018_ENA
			case (uint8_t) A53_STL_CASE_EL1_CORE_ID_018:
				// Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_018);
				break;
#endif
#ifdef A53_STL_EL1_CORE_019_ENA
			case (uint8_t) A53_STL_CASE_EL1_CORE_ID_019:
				// Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_019);
				break;
#endif
#ifdef A53_STL_EL1_CORE_020_ENA
			case (uint8_t) A53_STL_CASE_EL1_CORE_ID_020:
				// Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_020);
				break;
#endif
#ifdef A53_STL_EL1_CORE_021_ENA
			case (uint8_t) A53_STL_CASE_EL1_CORE_ID_021:
				// Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_021);
				break;
#endif
#ifdef A53_STL_EL1_CORE_022_ENA
			case (uint8_t) A53_STL_CASE_EL1_CORE_ID_022:
				// Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_022);
				break;
#endif
#ifdef A53_STL_EL1_CORE_023_ENA
			case (uint8_t) A53_STL_CASE_EL1_CORE_ID_023:
				// Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_023);
				break;
#endif

#ifdef A53_STL_EL1_CORE_024_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_024:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_024);
                break;
#endif

#ifdef A53_STL_EL1_CORE_025_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_025:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_025);
                break;
#endif

#ifdef A53_STL_EL1_CORE_026_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_026:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_026);
                break;
#endif

#ifdef A53_STL_EL1_CORE_027_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_027:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_027);
                break;
#endif

#ifdef A53_STL_EL1_CORE_028_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_028:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_028);
                break;
#endif

#ifdef A53_STL_EL1_CORE_029_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_029:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_029);
                break;
#endif

#ifdef A53_STL_EL1_CORE_030_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_030:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_030);
                break;
#endif

#ifdef A53_STL_EL1_CORE_031_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_031:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_031);
                break;
#endif
#ifdef A53_STL_EL1_CORE_032_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_032:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_032);
                break;
#endif

#ifdef A53_STL_EL1_CORE_033_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_033:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_033);
                break;
#endif
#ifdef A53_STL_EL1_CORE_034_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_034:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_034);
                break;
#endif
#ifdef A53_STL_EL1_CORE_041_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_041:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_041);
                break;
#endif
#ifdef A53_STL_EL1_CORE_042_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_042:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_042);
                break;
#endif
#ifdef A53_STL_EL1_CORE_043_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_043:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_043);
                break;
#endif
#ifdef A53_STL_EL1_CORE_044_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_044:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_044);
                break;
#endif
#ifdef A53_STL_EL1_CORE_045_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_045:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_045);
                break;
#endif
#ifdef A53_STL_EL1_CORE_046_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_046:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_046);
                break;
#endif
#ifdef A53_STL_EL1_CORE_047_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_047:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_047);
                break;
#endif
#ifdef A53_STL_EL1_CORE_048_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_048:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_048);
                break;
#endif
#ifdef A53_STL_EL1_CORE_049_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_049:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_049);
                break;
#endif
#ifdef A53_STL_EL1_CORE_050_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_050:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_050);
                break;
#endif
#ifdef A53_STL_EL1_CORE_051_ENA
            case (uint8_t) A53_STL_CASE_EL1_CORE_ID_051:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL1_CORE_ID_051);
                break;
#endif

            default:
                // Set invalid test identifier
                resultTest = A53_STL_TEST_ERROR;
                break;
        }
    } else {
        // Set invalid input parameter
        resultTest = A53_STL_TEST_ERROR;
    }

    if ( resultTest == A53_STL_TEST_ERROR ) {
        // Fault result corresponding to FCTLR Register
        resultTest = a53_stl_set_status_fctlr(stlInput,
        									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
        // Force the function return value to the error
        // also if the function above return ok
        resultTest = A53_STL_TEST_ERROR;
    }


    return resultTest;
}

//====================================================================================================================================
//   Function: SetMemRegsTestIdEL2
//      Function to set the information of the SW part
//			under test in EL2 in the memory mapped register
//
//   Parameters:
//      stlInput - Pointer to the STL instance
//      idTest   - Identifier of the test
//
//   Returns: uint32_t
//      Global pass/fail result of set the information:
//			A53_STL_TEST_OK Setting execution success;
//			A53_STL_TEST_ERROR Setting execution failed
//
//====================================================================================================================================//
static uint32_t SetMemRegsTestIdEL2(
											const a53_stl_t * stlInput,
											uint32_t idTest) {
    // Function return value
    uint32_t resultTest;

    // Defensive initialization
    resultTest  = A53_STL_TEST_ERROR;

    if ( stlInput != A53_STL_NULL ) {
        // Schedule current module ID
        switch (idTest) {
#ifdef A53_STL_EL2_CORE_001_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_001:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_001);
                break;
#endif
#ifdef A53_STL_EL2_CORE_002_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_002:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_002);
                break;
#endif

#ifdef A53_STL_EL2_CORE_003_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_003:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_003);
                break;
#endif

#ifdef A53_STL_EL2_CORE_004_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_004:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_004);
                break;
#endif

#ifdef A53_STL_EL2_CORE_005_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_005:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_005);
                break;
#endif

#ifdef A53_STL_EL2_CORE_006_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_006:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_006);
                break;
#endif

#ifdef A53_STL_EL2_CORE_007_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_007:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_007);
                break;
#endif

#ifdef A53_STL_EL2_CORE_008_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_008:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_008);
                break;
#endif

#ifdef A53_STL_EL2_CORE_009_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_009:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_009);
                break;
#endif

#ifdef A53_STL_EL2_CORE_010_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_010:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_010);
                break;
#endif

#ifdef A53_STL_EL2_CORE_011_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_011:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_011);
                break;
#endif

#ifdef A53_STL_EL2_CORE_012_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_012:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_012);
                break;
#endif

#ifdef A53_STL_EL2_CORE_013_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_013:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_013);
                break;
#endif

#ifdef A53_STL_EL2_CORE_014_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_014:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_014);
                break;
#endif

#ifdef A53_STL_EL2_CORE_015_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_015:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_015);
                break;
#endif

#ifdef A53_STL_EL2_CORE_016_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_016:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_016);
                break;
#endif

#ifdef A53_STL_EL2_CORE_017_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_017:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_017);
                break;
#endif
#ifdef A53_STL_EL2_CORE_018_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_018:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_018);
                break;
#endif
#ifdef A53_STL_EL2_CORE_019_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_019:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_019);
                break;
#endif
#ifdef A53_STL_EL2_CORE_020_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_020:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_020);
                break;
#endif
#ifdef A53_STL_EL2_CORE_021_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_021:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_021);
                break;
#endif
#ifdef A53_STL_EL2_CORE_022_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_022:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_022);
                break;
#endif
#ifdef A53_STL_EL2_CORE_023_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_023:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_023);
                break;
#endif

#ifdef A53_STL_EL2_CORE_024_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_024:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_024);
                break;
#endif

#ifdef A53_STL_EL2_CORE_025_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_025:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_025);
                break;
#endif

#ifdef A53_STL_EL2_CORE_026_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_026:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_026);
                break;
#endif

#ifdef A53_STL_EL2_CORE_027_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_027:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_027);
                break;
#endif

#ifdef A53_STL_EL2_CORE_028_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_028:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_028);
                break;
#endif

#ifdef A53_STL_EL2_CORE_029_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_029:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_029);
                break;
#endif

#ifdef A53_STL_EL2_CORE_030_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_030:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_030);
                break;
#endif

#ifdef A53_STL_EL2_CORE_031_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_031:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_031);
                break;
#endif
#ifdef A53_STL_EL2_CORE_032_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_032:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_032);
                break;
#endif

#ifdef A53_STL_EL2_CORE_033_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_033:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_033);
                break;
#endif
#ifdef A53_STL_EL2_CORE_034_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_034:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_034);
                break;
#endif
#ifdef A53_STL_EL2_CORE_041_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_041:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_041);
                break;
#endif
#ifdef A53_STL_EL2_CORE_042_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_042:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_042);
                break;
#endif
#ifdef A53_STL_EL2_CORE_043_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_043:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_043);
                break;
#endif
#ifdef A53_STL_EL2_CORE_044_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_044:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_044);
                break;
#endif
#ifdef A53_STL_EL2_CORE_045_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_045:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_045);
                break;
#endif
#ifdef A53_STL_EL2_CORE_046_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_046:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_046);
                break;
#endif
#ifdef A53_STL_EL2_CORE_047_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_047:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_047);
                break;
#endif
#ifdef A53_STL_EL2_CORE_048_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_048:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_048);
                break;
#endif
#ifdef A53_STL_EL2_CORE_049_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_049:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_049);
                break;
#endif
#ifdef A53_STL_EL2_CORE_050_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_050:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_050);
                break;
#endif
#ifdef A53_STL_EL2_CORE_051_ENA
            case (uint8_t) A53_STL_CASE_EL2_CORE_ID_051:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL2_CORE_ID_051);
                break;
#endif
            default:
                // Set invalid test identifier
                resultTest = A53_STL_TEST_ERROR;
                break;
        }
    } else {
        // Set invalid input parameter
        resultTest = A53_STL_TEST_ERROR;
    }

    if ( resultTest == A53_STL_TEST_ERROR ) {
        // Fault result corresponding to FCTLR Register
        resultTest = a53_stl_set_status_fctlr(stlInput,
        									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
        // Force the function return value to the error
        // also if the function above return ok
        resultTest = A53_STL_TEST_ERROR;
    }


    return resultTest;
}

//====================================================================================================================================
//   Function: SetMemRegsTestIdEL3
//      Function to set the information of the SW part
//		under test in EL3 in the memory mapped register
//   Parameters:
//      stlInput - Pointer to the STL instance
//      idTest   - Identifier of the test
//
//   Returns: uint32_t
//      Global pass/fail result of set the information:
//				A53_STL_TEST_OK Setting execution success;
//				A53_STL_TEST_ERROR Setting execution failed
//
//====================================================================================================================================//
static uint32_t SetMemRegsTestIdEL3(
											const a53_stl_t * stlInput,
											uint32_t idTest) {
    // Function return value
    uint32_t resultTest;

    // Defensive initialization
    resultTest  = A53_STL_TEST_ERROR;

    if ( stlInput != A53_STL_NULL ) {
        // Schedule current module ID
        switch (idTest) {
#ifdef A53_STL_EL3_CORE_035_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_035:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL3_CORE_ID_035);
                break;
#endif
#ifdef A53_STL_EL3_CORE_036_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_036:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL3_CORE_ID_036);
                break;
#endif
#ifdef A53_STL_EL3_CORE_037_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_037:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL3_CORE_ID_037);
                break;
#endif
#ifdef A53_STL_EL3_CORE_038_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_038:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL3_CORE_ID_038);
                break;
#endif
#ifdef A53_STL_EL3_CORE_039_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_039:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL3_CORE_ID_039);
                break;
#endif
#ifdef A53_STL_EL3_CORE_040_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_040:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL3_CORE_ID_040);
                break;
#endif
#ifdef A53_STL_EL3_CORE_052_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_052:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL3_CORE_ID_052);
                break;
#endif
#ifdef A53_STL_EL3_CORE_053_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_053:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL3_CORE_ID_053);
                break;
#endif
#ifdef A53_STL_EL3_CORE_054_ENA
            case (uint8_t) A53_STL_CASE_EL3_CORE_ID_054:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_CORE,
                								(uint32_t)A53_STL_TEST_EL3_CORE_ID_054);
                break;
#endif
#ifdef A53_STL_EL3_GIC_001_ENA
            case (uint8_t) A53_STL_CASE_EL3_GIC_ID_001:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_GIC,
                								(uint32_t)A53_STL_TEST_EL3_GIC_ID_001);
                break;
#endif
#ifdef A53_STL_EL3_GIC_002_ENA
            case (uint8_t) A53_STL_CASE_EL3_GIC_ID_002:
                // Set Main Test ID and Sub Test ID on FPIR register
                resultTest = a53_stl_set_reg_mem(stlInput,
                								(uint32_t)A53_STL_FPIR_MTID_GIC,
                								(uint32_t)A53_STL_TEST_EL3_GIC_ID_002);
                break;
#endif
            default:
                // Set invalid test identifier
                resultTest = A53_STL_TEST_ERROR;
                break;
        }
    } else {
        // Set invalid input parameter
        resultTest = A53_STL_TEST_ERROR;
    }

    if ( resultTest == A53_STL_TEST_ERROR ) {
        // Fault result corresponding to FCTLR Register
        resultTest = a53_stl_set_status_fctlr(
        									stlInput,
        									(uint32_t)A53_STL_FCTLR_STATUS_FAIL);
        // Force the function return value to the error
        // also if the function above return ok
        resultTest = A53_STL_TEST_ERROR;
    }

    return resultTest;
}




