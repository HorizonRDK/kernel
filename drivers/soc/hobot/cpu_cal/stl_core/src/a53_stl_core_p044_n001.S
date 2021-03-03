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
//      Testing of addition and subtraction instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_044
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p044_n001
//      Testing of addition and subtraction instructions
//
//   Parameters:
//      R0 - Result address
//
//   Returns:
//      N.A.
//===========================================================================================//


        #include "../../shared/inc/a53_stl_constants.h"
        #include "../../shared/inc/a53_stl_utils.h"

        .align 4

        .section .section_a53_stl_core_p044_n001,"ax",%progbits
        .global a53_stl_core_p044_n001
        .type a53_stl_core_p044_n001, %function

a53_stl_core_p044_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 26
//-----------------------------------------------------------

// Basic Module 26
// SUB <Xd>, <Xn>, <R><m>{, <extend> {#<amount>}}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM26_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM26_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM26_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM26_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM26_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM26_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM26_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM26_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM26_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM26_INPUT_VALUE_6
        ldr             x12, =A53_STL_BM26_INPUT_VALUE_5
        ldr             x13, =A53_STL_BM26_INPUT_VALUE_6
        ldr             x14, =A53_STL_BM26_INPUT_VALUE_7
        ldr             x15, =A53_STL_BM26_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM26_INPUT_VALUE_7
        ldr             x17, =A53_STL_BM26_INPUT_VALUE_8

        sub             x18, x2, w3, uxtb #A53_STL_BM26_IMM_VALUE_1
        sub             x19, x4, w5, uxtb #A53_STL_BM26_IMM_VALUE_1

        sub             x20, x6, w7, uxth #A53_STL_BM26_IMM_VALUE_2
        sub             x21, x8, w9, uxth #A53_STL_BM26_IMM_VALUE_2

        sub             x22, x10, w11, uxtw #A53_STL_BM26_IMM_VALUE_1
        sub             x23, x12, w13, uxtw #A53_STL_BM26_IMM_VALUE_1

        sub             x24, x14, x15, uxtx #A53_STL_BM26_IMM_VALUE_1
        sub             x25, x16, x17, uxtx #A53_STL_BM26_IMM_VALUE_1

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM26_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM26_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM26_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM26_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM26_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_26:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 26
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 27
//-----------------------------------------------------------

// Basic Module 27
// SUBS <Xd>, <Xn>, <R><m>{, <extend> {#<amount>}}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Test N = 0, Z = 0, C = 1, V = 1

        // Set operand registers

        ldr             x1, =A53_STL_BM27_INPUT_VALUE_3
        ldr             x2, =A53_STL_BM27_INPUT_VALUE_3
        ldr             x3, =A53_STL_BM27_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM27_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM27_INPUT_VALUE_1
        ldr             x6, =A53_STL_BM27_INPUT_VALUE_2
        ldr             x7, =A53_STL_BM27_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM27_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM27_INPUT_VALUE_1
        ldr             x10, =A53_STL_BM27_INPUT_VALUE_2

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x18, x1, x3, uxtx #A53_STL_BM27_IMM_VALUE_1
        sub             x19, x2, x4, uxtx #A53_STL_BM27_IMM_VALUE_1
        subs            x20, x5, x6, uxtx #A53_STL_BM27_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM27_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x21, x7, x8, uxtx #A53_STL_BM27_IMM_VALUE_1
        subs            x22, x9, x10, uxtx #A53_STL_BM27_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM27_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM27_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x26, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x22
        b.ne            error

        // Test N = 0, Z = 1, C = 1, V = 0

        // Set operand registers

        ldr             x1, =A53_STL_BM27_INPUT_VALUE_5
        ldr             x2, =A53_STL_BM27_INPUT_VALUE_5
        ldr             x3, =A53_STL_BM27_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM27_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM27_INPUT_VALUE_4
        ldr             x6, =A53_STL_BM27_INPUT_VALUE_4
        ldr             x7, =A53_STL_BM27_INPUT_VALUE_5
        ldr             x8, =A53_STL_BM27_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM27_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM27_INPUT_VALUE_4

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x18, x1, w3, uxth #A53_STL_BM27_IMM_VALUE_1
        sub             x19, x2, w4, uxth #A53_STL_BM27_IMM_VALUE_1
        subs            x20, x5, w6, uxth #A53_STL_BM27_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM27_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x21, x7, w8, uxth #A53_STL_BM27_IMM_VALUE_1
        subs            x22, x9, w10, uxth #A53_STL_BM27_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM27_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM27_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x26, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x22
        b.ne            error

        // Test N = 1, Z = 0, C = 0, V = 1 with CMP alias

        // Set operand registers

        ldr             x1, =A53_STL_BM27_INPUT_VALUE_6
        ldr             x2, =A53_STL_BM27_INPUT_VALUE_6
        ldr             x3, =A53_STL_BM27_INPUT_VALUE_7
        ldr             x4, =A53_STL_BM27_INPUT_VALUE_7
        ldr             x5, =A53_STL_BM27_INPUT_VALUE_6
        ldr             x6, =A53_STL_BM27_INPUT_VALUE_7
        ldr             x7, =A53_STL_BM27_INPUT_VALUE_6
        ldr             x8, =A53_STL_BM27_INPUT_VALUE_7
        ldr             x9, =A53_STL_BM27_INPUT_VALUE_6
        ldr             x10, =A53_STL_BM27_INPUT_VALUE_7

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x18, x1, x3, uxtx #A53_STL_BM27_IMM_VALUE_1
        sub             x19, x2, x4, uxtx #A53_STL_BM27_IMM_VALUE_1
        subs            xzr, x5, x6, uxtx #A53_STL_BM27_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM27_GOLDEN_FLAGS_3

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x21, x7, x8, uxtx #A53_STL_BM27_IMM_VALUE_1
        subs            xzr, x9, x10, uxtx #A53_STL_BM27_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM27_GOLDEN_FLAGS_3

check_correct_result_BM_27:
        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 27
//-----------------------------------------------------------

        // All Basic Modules pass without errors
        mov             x0, A53_STL_FCTLR_STATUS_PDONE

        b               call_postamble

error:
        // Set Data Corruption (0x4) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_regs_error

call_postamble:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p044_n001_end:

        ret

        .end
