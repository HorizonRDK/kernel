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
//      CORE_043
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p043_n001
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

        .section .section_a53_stl_core_p043_n001,"ax",%progbits
        .global a53_stl_core_p043_n001
        .type a53_stl_core_p043_n001, %function

a53_stl_core_p043_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 22
//-----------------------------------------------------------

// Basic Module 22
// SUB <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM22_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM22_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM22_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM22_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM22_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM22_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM22_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM22_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM22_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM22_INPUT_VALUE_6
        ldr             x12, =A53_STL_BM22_INPUT_VALUE_5
        ldr             x13, =A53_STL_BM22_INPUT_VALUE_6

        sub             x18, x2, x3
        sub             x19, x4, x5

        sub             x20, x6, x7
        sub             x21, x8, x9

        // NEG alias

        sub             x22, xzr, x10
        sub             x23, xzr, x12

        sub             x24, xzr, x11
        sub             x25, xzr, x13

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM22_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM22_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM22_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM22_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM22_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_22:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 22
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 23
//-----------------------------------------------------------

// Basic Module 23
// SUBS <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Test N = 0, Z = 0, C = 1, V = 1

        // Set operand registers

        ldr             x1, =A53_STL_BM23_INPUT_VALUE_3
        ldr             x2, =A53_STL_BM23_INPUT_VALUE_3
        ldr             x3, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM23_INPUT_VALUE_1
        ldr             x6, =A53_STL_BM23_INPUT_VALUE_2
        ldr             x7, =A53_STL_BM23_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM23_INPUT_VALUE_1
        ldr             x10, =A53_STL_BM23_INPUT_VALUE_2

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x18, x1, x3
        sub             x19, x2, x4
        subs            x20, x5, x6

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM23_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x21, x7, x8
        subs            x22, x9, x10
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM23_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM23_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x26, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x22
        b.ne            error

        // Test N = 0, Z = 1, C = 1, V = 0

        // Set operand registers

        ldr             x1, =A53_STL_BM23_INPUT_VALUE_7
        ldr             x2, =A53_STL_BM23_INPUT_VALUE_7
        ldr             x3, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM23_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM23_INPUT_VALUE_5
        ldr             x7, =A53_STL_BM23_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM23_INPUT_VALUE_5
        ldr             x10, =A53_STL_BM23_INPUT_VALUE_5

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x18, x1, x3
        sub             x19, x2, x4
        subs            x20, x5, x6

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM23_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x21, x7, x8
        subs            x22, x9, x10
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM23_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM23_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x26, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x22
        b.ne            error

        // Test N = 0, Z = 1, C = 1, V = 0 with CMP alias

        // Set operand registers

        ldr             x1, =A53_STL_BM23_INPUT_VALUE_8
        ldr             x2, =A53_STL_BM23_INPUT_VALUE_8
        ldr             x3, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x6, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x7, =A53_STL_BM23_INPUT_VALUE_8
        ldr             x8, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM23_INPUT_VALUE_4

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x18, x1, x3
        sub             x19, x2, x4
        subs            xzr, x5, x6, lsl #A53_STL_BM23_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM23_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x21, x7, x8
        subs            xzr, x9, x10, lsl #A53_STL_BM23_IMM_VALUE_2
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM23_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Test N = 1, Z = 0, C = 0, V = 0 with NEGS alias

        // Set operand registers

        ldr             x1, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x2, =A53_STL_BM23_INPUT_VALUE_4
        ldr             x3, =A53_STL_BM23_INPUT_VALUE_9
        ldr             x4, =A53_STL_BM23_INPUT_VALUE_9
        ldr             x5, =A53_STL_BM23_INPUT_VALUE_9
        ldr             x6, =A53_STL_BM23_INPUT_VALUE_9
        ldr             x7, =A53_STL_BM23_INPUT_VALUE_9
        ldr             x8, =A53_STL_BM23_INPUT_VALUE_9
        ldr             x9, =A53_STL_BM23_INPUT_VALUE_9
        ldr             x10, =A53_STL_BM23_INPUT_VALUE_9

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x18, x1, x3
        sub             x19, x2, x4
        subs            x20, xzr, x6, lsr #A53_STL_BM23_IMM_VALUE_3

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM23_GOLDEN_FLAGS_3

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        sub             x21, x7, x8
        subs            x22, xzr, x10, lsr #A53_STL_BM23_IMM_VALUE_3
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM23_GOLDEN_FLAGS_3

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM23_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x26, x20
        b.ne            error

check_correct_result_BM_23:
        // Compare local and golden signature
        cmp             x26, x22
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 23
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

a53_stl_core_p043_n001_end:

        ret

        .end
