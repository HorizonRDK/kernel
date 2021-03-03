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
//   Function: a53_stl_save:
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

        .section .section_a53_stl_save,"ax",%progbits
        .global a53_stl_save
        .type a53_stl_save, %function

a53_stl_save:

        // Context switch save for FP registers:
        sub             sp, sp, A53_STL_STACK_PTR_512_B

        str             q0, [sp, A53_STL_STACK_0_OFFSET]
        str             q1, [sp, A53_STL_STACK_10_OFFSET]
        str             q2, [sp, A53_STL_STACK_20_OFFSET]
        str             q3, [sp, A53_STL_STACK_30_OFFSET]
        str             q4, [sp, A53_STL_STACK_40_OFFSET]
        str             q5, [sp, A53_STL_STACK_50_OFFSET]
        str             q6, [sp, A53_STL_STACK_60_OFFSET]
        str             q7, [sp, A53_STL_STACK_70_OFFSET]
        str             q8, [sp, A53_STL_STACK_80_OFFSET]
        str             q9, [sp, A53_STL_STACK_90_OFFSET]
        str             q10, [sp, A53_STL_STACK_A0_OFFSET]
        str             q11, [sp, A53_STL_STACK_B0_OFFSET]
        str             q12, [sp, A53_STL_STACK_C0_OFFSET]
        str             q13, [sp, A53_STL_STACK_D0_OFFSET]
        str             q14, [sp, A53_STL_STACK_E0_OFFSET]
        str             q15, [sp, A53_STL_STACK_F0_OFFSET]
        str             q16, [sp, A53_STL_STACK_100_OFFSET]
        str             q17, [sp, A53_STL_STACK_110_OFFSET]
        str             q18, [sp, A53_STL_STACK_120_OFFSET]
        str             q19, [sp, A53_STL_STACK_130_OFFSET]
        str             q20, [sp, A53_STL_STACK_140_OFFSET]
        str             q21, [sp, A53_STL_STACK_150_OFFSET]
        str             q22, [sp, A53_STL_STACK_160_OFFSET]
        str             q23, [sp, A53_STL_STACK_170_OFFSET]
        str             q24, [sp, A53_STL_STACK_180_OFFSET]
        str             q25, [sp, A53_STL_STACK_190_OFFSET]
        str             q26, [sp, A53_STL_STACK_1A0_OFFSET]
        str             q27, [sp, A53_STL_STACK_1B0_OFFSET]
        str             q28, [sp, A53_STL_STACK_1C0_OFFSET]
        str             q29, [sp, A53_STL_STACK_1D0_OFFSET]
        str             q30, [sp, A53_STL_STACK_1E0_OFFSET]
        str             q31, [sp, A53_STL_STACK_1F0_OFFSET]

        // Context switch save for GPR registers
#ifdef ALIGN_8
        sub             sp, sp, A53_STL_STACK_PTR_240_B
        str             x1, [sp, A53_STL_STACK_0_OFFSET]
        str             x2, [sp, A53_STL_STACK_8_OFFSET]
        str             x3, [sp, A53_STL_STACK_10_OFFSET]
        str             x4, [sp, A53_STL_STACK_18_OFFSET]
        str             x5, [sp, A53_STL_STACK_20_OFFSET]
        str             x6, [sp, A53_STL_STACK_28_OFFSET]
        str             x7, [sp, A53_STL_STACK_30_OFFSET]
        str             x8, [sp, A53_STL_STACK_38_OFFSET]
        str             x9, [sp, A53_STL_STACK_40_OFFSET]
        str             x10, [sp, A53_STL_STACK_48_OFFSET]
        str             x11, [sp, A53_STL_STACK_50_OFFSET]
        str             x12, [sp, A53_STL_STACK_58_OFFSET]
        str             x13, [sp, A53_STL_STACK_60_OFFSET]
        str             x14, [sp, A53_STL_STACK_68_OFFSET]
        str             x15, [sp, A53_STL_STACK_70_OFFSET]
        str             x16, [sp, A53_STL_STACK_78_OFFSET]
        str             x17, [sp, A53_STL_STACK_80_OFFSET]
        str             x18, [sp, A53_STL_STACK_88_OFFSET]
        str             x19, [sp, A53_STL_STACK_90_OFFSET]
        str             x20, [sp, A53_STL_STACK_98_OFFSET]
        str             x21, [sp, A53_STL_STACK_A0_OFFSET]
        str             x22, [sp, A53_STL_STACK_A8_OFFSET]
        str             x23, [sp, A53_STL_STACK_B0_OFFSET]
        str             x24, [sp, A53_STL_STACK_B8_OFFSET]
        str             x25, [sp, A53_STL_STACK_C0_OFFSET]
        str             x26, [sp, A53_STL_STACK_C8_OFFSET]
        str             x27, [sp, A53_STL_STACK_D0_OFFSET]
        str             x28, [sp, A53_STL_STACK_D8_OFFSET]
        str             x29, [sp, A53_STL_STACK_E0_OFFSET]

        // Saved passed paramenters into the stack, FCTLR register address is in x0
        str             x0, [sp, A53_STL_STACK_E8_OFFSET]
#else

		sub 			sp, sp, A53_STL_STACK_PTR_480_B
		str 			x1, [sp, A53_STL_STACK_0_OFFSET]
		str 			x2, [sp, A53_STL_STACK_10_OFFSET]
		str 			x3, [sp, A53_STL_STACK_20_OFFSET]
		str 			x4, [sp, A53_STL_STACK_30_OFFSET]
		str 			x5, [sp, A53_STL_STACK_40_OFFSET]
		str 			x6, [sp, A53_STL_STACK_50_OFFSET]
		str 			x7, [sp, A53_STL_STACK_60_OFFSET]
		str 			x8, [sp, A53_STL_STACK_70_OFFSET]
		str 			x9, [sp, A53_STL_STACK_80_OFFSET]
		str 			x10, [sp, A53_STL_STACK_90_OFFSET]
		str 			x11, [sp, A53_STL_STACK_A0_OFFSET]
		str 			x12, [sp, A53_STL_STACK_B0_OFFSET]
		str 			x13, [sp, A53_STL_STACK_C0_OFFSET]
		str 			x14, [sp, A53_STL_STACK_D0_OFFSET]
		str 			x15, [sp, A53_STL_STACK_E0_OFFSET]
		str 			x16, [sp, A53_STL_STACK_F0_OFFSET]
		str 			x17, [sp, A53_STL_STACK_100_OFFSET]
		str 			x18, [sp, A53_STL_STACK_110_OFFSET]
		str 			x19, [sp, A53_STL_STACK_120_OFFSET]
		str 			x20, [sp, A53_STL_STACK_130_OFFSET]
		str 			x21, [sp, A53_STL_STACK_140_OFFSET]
		str 			x22, [sp, A53_STL_STACK_150_OFFSET]
		str 			x23, [sp, A53_STL_STACK_160_OFFSET]
		str 			x24, [sp, A53_STL_STACK_170_OFFSET]
		str 			x25, [sp, A53_STL_STACK_180_OFFSET]
		str 			x26, [sp, A53_STL_STACK_190_OFFSET]
		str 			x27, [sp, A53_STL_STACK_1A0_OFFSET]
		str 			x28, [sp, A53_STL_STACK_1B0_OFFSET]
		str 			x29, [sp, A53_STL_STACK_1C0_OFFSET]
		// Saved passed paramenters into the stack, FCTLR register address is in x0
		str 			x0, [sp, A53_STL_STACK_1D0_OFFSET]
#endif
        ret

        .end
