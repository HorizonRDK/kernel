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
//      CORE_005
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p005_n001
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

        .section .section_a53_stl_core_p005_n001,"ax",%progbits
        .global a53_stl_core_p005_n001
        .type a53_stl_core_p005_n001, %function

a53_stl_core_p005_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 20
//-----------------------------------------------------------

// Basic Module 20
// ADD <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        mov             x2, A53_STL_BM20_INPUT_VALUE_1
        mov             x3, A53_STL_BM20_INPUT_VALUE_2
        mov             x4, A53_STL_BM20_INPUT_VALUE_1
        mov             x5, A53_STL_BM20_INPUT_VALUE_2
        mov             x6, A53_STL_BM20_INPUT_VALUE_3
        mov             x7, A53_STL_BM20_INPUT_VALUE_4
        mov             x8, A53_STL_BM20_INPUT_VALUE_3
        mov             x9, A53_STL_BM20_INPUT_VALUE_4
        mov             x10, A53_STL_BM20_INPUT_VALUE_5
        mov             x11, A53_STL_BM20_INPUT_VALUE_6
        mov             x12, A53_STL_BM20_INPUT_VALUE_5
        mov             x13, A53_STL_BM20_INPUT_VALUE_6
        mov             x14, A53_STL_BM20_INPUT_VALUE_7
        mov             x15, A53_STL_BM20_INPUT_VALUE_8
        mov             x16, A53_STL_BM20_INPUT_VALUE_7
        mov             x17, A53_STL_BM20_INPUT_VALUE_8

        add             x18, x2, x6
        add             x19, x4, x8

        add             x20, x3, x7
        add             x21, x5, x9

        add             x22, x10, x14
        add             x23, x12, x16

        add             x24, x11, x15
        add             x25, x13, x17

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM20_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM20_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM20_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM20_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM20_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_20:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 20
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 21
//-----------------------------------------------------------

// Basic Module 21
// ADDS <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Test N = 1, Z = 0, C = 0, V = 1

        // Set operand registers

        ldr             x1, =A53_STL_BM21_INPUT_VALUE_3
        ldr             x2, =A53_STL_BM21_INPUT_VALUE_3
        ldr             x3, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM21_INPUT_VALUE_1
        ldr             x6, =A53_STL_BM21_INPUT_VALUE_2
        ldr             x7, =A53_STL_BM21_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM21_INPUT_VALUE_1
        ldr             x10, =A53_STL_BM21_INPUT_VALUE_2

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x18, x1, x3, lsl #A53_STL_BM21_IMM_VALUE_1
        adds            x19, x2, x4, lsl #A53_STL_BM21_IMM_VALUE_2
        adds            x20, x5, x6, lsl #A53_STL_BM21_IMM_VALUE_3

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM21_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x21, x7, x8, lsl #A53_STL_BM21_IMM_VALUE_4
        adds            x22, x9, x10, lsl #A53_STL_BM21_IMM_VALUE_3
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM21_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM21_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x26, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x19
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x21
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM21_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x22
        b.ne            error

        // Test N = 0, Z = 0, C = 1, V = 0

        // Set operand registers

        ldr             x1, =A53_STL_BM21_INPUT_VALUE_7
        ldr             x2, =A53_STL_BM21_INPUT_VALUE_7
        ldr             x3, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM21_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM21_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM21_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM21_INPUT_VALUE_5
        ldr             x10, =A53_STL_BM21_INPUT_VALUE_6

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x18, x1, x3, lsl #A53_STL_BM21_IMM_VALUE_1
        adds            x19, x2, x4, lsl #A53_STL_BM21_IMM_VALUE_2
        adds            x20, x5, x6, lsl #A53_STL_BM21_IMM_VALUE_3

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM21_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x21, x7, x8, lsl #A53_STL_BM21_IMM_VALUE_4
        adds            x22, x9, x10, lsl #A53_STL_BM21_IMM_VALUE_3
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM21_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM21_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x26, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x19
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x21
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM21_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x22
        b.ne            error

        // Test N = 0, Z = 1, C = 0, V = 0 with CMN alias

        // Set operand registers

        ldr             x1, =A53_STL_BM21_INPUT_VALUE_8
        ldr             x2, =A53_STL_BM21_INPUT_VALUE_8
        ldr             x3, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x6, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x7, =A53_STL_BM21_INPUT_VALUE_8
        ldr             x8, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM21_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM21_INPUT_VALUE_4

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x18, x1, x3, asr #A53_STL_BM21_IMM_VALUE_1
        adds            x19, x2, x4, asr #A53_STL_BM21_IMM_VALUE_1
        adds            xzr, x5, x6

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM21_GOLDEN_FLAGS_3

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x21, x7, x8, lsl #A53_STL_BM21_IMM_VALUE_4
        adds            xzr, x9, x10
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM21_GOLDEN_FLAGS_3

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM21_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x26, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x19
        b.ne            error

check_correct_result_BM_21:
        // Compare local and golden signature
        cmp             x26, x21
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 21
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

a53_stl_core_p005_n001_end:

        ret

        .end
