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
//      Context switch subroutine for tests
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


//===========================================================================================
//   Function: a53_stl_restore:
//      Context switch subroutine for tests
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        #include "../inc/a53_stl_constants.h"

        .align 4

        .section .section_a53_stl_restore,"ax",%progbits
        .global a53_stl_restore
        .type a53_stl_restore, %function

a53_stl_restore:
        // Context switch restore for GPR registers

        // Increment the sp of 8 bytes because of link register saving in a53_stl_preamble function
        add             sp, sp, A53_STL_STACK_8_OFFSET
#ifdef ALIGN_8

        ldr             x1, [sp, A53_STL_STACK_0_OFFSET]
        ldr             x2, [sp, A53_STL_STACK_8_OFFSET]
        ldr             x3, [sp, A53_STL_STACK_10_OFFSET]
        ldr             x4, [sp, A53_STL_STACK_18_OFFSET]
        ldr             x5, [sp, A53_STL_STACK_20_OFFSET]
        ldr             x6, [sp, A53_STL_STACK_28_OFFSET]
        ldr             x7, [sp, A53_STL_STACK_30_OFFSET]
        ldr             x8, [sp, A53_STL_STACK_38_OFFSET]
        ldr             x9, [sp, A53_STL_STACK_40_OFFSET]
        ldr             x10, [sp, A53_STL_STACK_48_OFFSET]
        ldr             x11, [sp, A53_STL_STACK_50_OFFSET]
        ldr             x12, [sp, A53_STL_STACK_58_OFFSET]
        ldr             x13, [sp, A53_STL_STACK_60_OFFSET]
        ldr             x14, [sp, A53_STL_STACK_68_OFFSET]
        ldr             x15, [sp, A53_STL_STACK_70_OFFSET]
        ldr             x16, [sp, A53_STL_STACK_78_OFFSET]
        ldr             x17, [sp, A53_STL_STACK_80_OFFSET]
        ldr             x18, [sp, A53_STL_STACK_88_OFFSET]
        ldr             x19, [sp, A53_STL_STACK_90_OFFSET]
        ldr             x20, [sp, A53_STL_STACK_98_OFFSET]
        ldr             x21, [sp, A53_STL_STACK_A0_OFFSET]
        ldr             x22, [sp, A53_STL_STACK_A8_OFFSET]
        ldr             x23, [sp, A53_STL_STACK_B0_OFFSET]
        ldr             x24, [sp, A53_STL_STACK_B8_OFFSET]
        ldr             x25, [sp, A53_STL_STACK_C0_OFFSET]
        ldr             x26, [sp, A53_STL_STACK_C8_OFFSET]
        ldr             x27, [sp, A53_STL_STACK_D0_OFFSET]
        ldr             x28, [sp, A53_STL_STACK_D8_OFFSET]
        ldr             x29, [sp, A53_STL_STACK_E0_OFFSET]

        // Restore x1 - FCTLR register address
        ldr             x1, [sp, A53_STL_STACK_E8_OFFSET]

        add             sp, sp, A53_STL_STACK_PTR_240_B
#else

		ldr 			x1, [sp, A53_STL_STACK_0_OFFSET]
		ldr 			x2, [sp, A53_STL_STACK_10_OFFSET]
		ldr 			x3, [sp, A53_STL_STACK_20_OFFSET]
		ldr 			x4, [sp, A53_STL_STACK_30_OFFSET]
		ldr 			x5, [sp, A53_STL_STACK_40_OFFSET]
		ldr 			x6, [sp, A53_STL_STACK_50_OFFSET]
		ldr 			x7, [sp, A53_STL_STACK_60_OFFSET]
		ldr 			x8, [sp, A53_STL_STACK_70_OFFSET]
		ldr 			x9, [sp, A53_STL_STACK_80_OFFSET]
		ldr 			x10, [sp, A53_STL_STACK_90_OFFSET]
		ldr 			x11, [sp, A53_STL_STACK_A0_OFFSET]
		ldr 			x12, [sp, A53_STL_STACK_B0_OFFSET]
		ldr 			x13, [sp, A53_STL_STACK_C0_OFFSET]
		ldr 			x14, [sp, A53_STL_STACK_D0_OFFSET]
		ldr 			x15, [sp, A53_STL_STACK_E0_OFFSET]
		ldr 			x16, [sp, A53_STL_STACK_F0_OFFSET]
		ldr 			x17, [sp, A53_STL_STACK_100_OFFSET]
		ldr 			x18, [sp, A53_STL_STACK_110_OFFSET]
		ldr 			x19, [sp, A53_STL_STACK_120_OFFSET]
		ldr 			x20, [sp, A53_STL_STACK_130_OFFSET]
		ldr 			x21, [sp, A53_STL_STACK_140_OFFSET]
		ldr 			x22, [sp, A53_STL_STACK_150_OFFSET]
		ldr 			x23, [sp, A53_STL_STACK_160_OFFSET]
		ldr 			x24, [sp, A53_STL_STACK_170_OFFSET]
		ldr 			x25, [sp, A53_STL_STACK_180_OFFSET]
		ldr 			x26, [sp, A53_STL_STACK_190_OFFSET]
		ldr 			x27, [sp, A53_STL_STACK_1A0_OFFSET]
		ldr 			x28, [sp, A53_STL_STACK_1B0_OFFSET]
		ldr 			x29, [sp, A53_STL_STACK_1C0_OFFSET]
		
		// Restore x1 - FCTLR register address
//		ldr 			x1, [sp, A53_STL_STACK_E8_OFFSET]
		ldr 			x1, [sp, A53_STL_STACK_1D0_OFFSET]
		
		add 			sp, sp, A53_STL_STACK_PTR_480_B
#endif
        // Context switch restore for FP registers

        ldr             q0, [sp, A53_STL_STACK_0_OFFSET]
        ldr             q1, [sp, A53_STL_STACK_10_OFFSET]
        ldr             q2, [sp, A53_STL_STACK_20_OFFSET]
        ldr             q3, [sp, A53_STL_STACK_30_OFFSET]
        ldr             q4, [sp, A53_STL_STACK_40_OFFSET]
        ldr             q5, [sp, A53_STL_STACK_50_OFFSET]
        ldr             q6, [sp, A53_STL_STACK_60_OFFSET]
        ldr             q7, [sp, A53_STL_STACK_70_OFFSET]
        ldr             q8, [sp, A53_STL_STACK_80_OFFSET]
        ldr             q9, [sp, A53_STL_STACK_90_OFFSET]
        ldr             q10, [sp, A53_STL_STACK_A0_OFFSET]
        ldr             q11, [sp, A53_STL_STACK_B0_OFFSET]
        ldr             q12, [sp, A53_STL_STACK_C0_OFFSET]
        ldr             q13, [sp, A53_STL_STACK_D0_OFFSET]
        ldr             q14, [sp, A53_STL_STACK_E0_OFFSET]
        ldr             q15, [sp, A53_STL_STACK_F0_OFFSET]
        ldr             q16, [sp, A53_STL_STACK_100_OFFSET]
        ldr             q17, [sp, A53_STL_STACK_110_OFFSET]
        ldr             q18, [sp, A53_STL_STACK_120_OFFSET]
        ldr             q19, [sp, A53_STL_STACK_130_OFFSET]
        ldr             q20, [sp, A53_STL_STACK_140_OFFSET]
        ldr             q21, [sp, A53_STL_STACK_150_OFFSET]
        ldr             q22, [sp, A53_STL_STACK_160_OFFSET]
        ldr             q23, [sp, A53_STL_STACK_170_OFFSET]
        ldr             q24, [sp, A53_STL_STACK_180_OFFSET]
        ldr             q25, [sp, A53_STL_STACK_190_OFFSET]
        ldr             q26, [sp, A53_STL_STACK_1A0_OFFSET]
        ldr             q27, [sp, A53_STL_STACK_1B0_OFFSET]
        ldr             q28, [sp, A53_STL_STACK_1C0_OFFSET]
        ldr             q29, [sp, A53_STL_STACK_1D0_OFFSET]
        ldr             q30, [sp, A53_STL_STACK_1E0_OFFSET]
        ldr             q31, [sp, A53_STL_STACK_1F0_OFFSET]

        add             sp, sp, A53_STL_STACK_PTR_512_B

        ret

        .end
