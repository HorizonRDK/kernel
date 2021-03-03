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
// About: About the File
// Armv8-A AArch64 common helper functions
//------------------------------------------------------------------------------
//===========================================================================================

#ifndef DRIVERS_SOC_HOBOT_CPU_CAL_SHARED_ARMV8_INC_V8_AARCH64_H_
#define DRIVERS_SOC_HOBOT_CPU_CAL_SHARED_ARMV8_INC_V8_AARCH64_H_

/*
 * Parameters for data barriers
 */
#define OSHLD   1
#define OSHST   2
#define OSH     3
#define NSHLD   5
#define NSHST   6
#define NSH     7
#define ISHLD   9
#define ISHST  10
#define ISH    11
#define LD     13
#define ST     14
#define SY     15

/**********************************************************************/

/*
 * function prototypes
 */

/*
 * void InvalidateUDCaches(void)
 *   invalidates all Unified and Data Caches
 *
 * Inputs
 *   <none>
 *
 * Returns
 *   <nothing>
 *
 * Side Effects
 *   guarantees that all levels of cache will be invalidated before
 *   returning to caller
 */
void InvalidateUDCaches(void);

/*
 * unsigned long long EnableCachesEL1(void)
 *   enables I- and D- caches at EL1
 *
 * Inputs
 *   <none>
 *
 * Returns
 *   New value of SCTLR_EL1
 *
 * Side Effects
 *   context will be synchronised before returning to caller
 */
unsigned long long EnableCachesEL1(void);

/*
 * unsigned long long GetMIDR(void)
 *   returns the contents of MIDR_EL0
 *
 * Inputs
 *   <none>
 *
 * Returns
 *   MIDR_EL0
 */
unsigned long long GetMIDR(void);

/*
 * unsigned long long GetMPIDR(void)
 *   returns the contents of MPIDR_EL0
 *
 * Inputs
 *   <none>
 *
 * Returns
 *   MPIDR_EL0
 */
unsigned long long GetMPIDR(void);

/*
 * unsigned int GetCPUID(void)
 *   returns the Aff0 field of MPIDR_EL0
 *
 * Inputs
 *   <none>
 *
 * Returns
 *   MPIDR_EL0[7:0]
 */
unsigned int GetCPUID(void);

#endif //DRIVERS_SOC_HOBOT_CPU_CAL_SHARED_ARMV8_INC_V8_AARCH64_H_
