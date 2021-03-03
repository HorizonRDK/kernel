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
//      Testing of Conditional select instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_012
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p012_n001
//      Testing of Conditional select instructions
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

        .section .section_a53_stl_core_p012_n001,"ax",%progbits
        .global a53_stl_core_p012_n001
        .type a53_stl_core_p012_n001, %function

a53_stl_core_p012_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 58
//-----------------------------------------------------------

// Basic Module 58
// CSEL <Xd>, <Xn>, <Xm>, <cond>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM58_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM58_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM58_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM58_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM58_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM58_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM58_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM58_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM58_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM58_INPUT_VALUE_6
        ldr             x12, =A53_STL_BM58_INPUT_VALUE_5
        ldr             x13, =A53_STL_BM58_INPUT_VALUE_6
        ldr             x14, =A53_STL_BM58_INPUT_VALUE_7
        ldr             x15, =A53_STL_BM58_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM58_INPUT_VALUE_7
        ldr             x17, =A53_STL_BM58_INPUT_VALUE_8

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 1st test: TRUE condition
        csel            x18, x2, x3, ne
        csel            x19, x4, x5, ne
        // 2nd test: FALSE condition
        csel            x20, x6, x7, eq
        csel            x21, x8, x9, eq
        // 3rd test: TRUE condition
        csel            x22, x10, x11, ne
        csel            x23, x12, x13, ne
        // 4th test: FALSE condition
        csel            x24, x14, x15, eq
        csel            x25, x16, x17, eq

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM58_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM58_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM58_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM58_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM58_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_58:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 58
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 59
//-----------------------------------------------------------

// Basic Module 59
// CSINC <Xd>, <Xn>, <Xm>, <cond>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Set operand register

        ldr             x0, =A53_STL_BM59_INPUT_VALUE_1
        ldr             x1, =A53_STL_BM59_INPUT_VALUE_2
        ldr             x2, =A53_STL_BM59_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM59_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM59_INPUT_VALUE_3
        ldr             x5, =A53_STL_BM59_INPUT_VALUE_4
        ldr             x6, =A53_STL_BM59_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM59_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM59_INPUT_VALUE_5
        ldr             x9, =A53_STL_BM59_INPUT_VALUE_5
        ldr             x10, =A53_STL_BM59_INPUT_VALUE_6
        ldr             x11, =A53_STL_BM59_INPUT_VALUE_6

        // Clear NZCV flags
        mov             x12, A53_STL_NZCV_CLR
        msr             nzcv, x12

        // 1st test: TRUE condition
        csinc           x12, x0, x1, ne
        csinc           x13, x2, x3, ne
        // 2nd test: FALSE condition
        csinc           x14, x4, x5, eq
        csinc           x15, x6, x7, eq
        // 3rd test: TRUE condition with CINC alias
        csinc           x16, x8, x8, eq
        csinc           x17, x9, x9, eq
        // 4th test: FALSE condition with CINC alias
        csinc           x18, x10, x10, ne
        csinc           x19, x11, x11, ne
        // 5th test: TRUE condition with CSET alias
        csinc           x20, xzr, xzr, eq
        csinc           x21, xzr, xzr, eq
        // 6th test: FALSE condition with CSET alias
        csinc           x22, xzr, xzr, ne
        csinc           x23, xzr, xzr, ne

        // Initialize x24 with golden signature
        ldr             x24, =A53_STL_BM59_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x24, x12
        b.ne            error

        // Compare local and golden signature
        cmp             x24, x13
        b.ne            error

        // initialize x25 with golden signature
        ldr             x25, =A53_STL_BM59_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x25, x14
        b.ne            error

        // Compare local and golden signature
        cmp             x25, x15
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM59_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x26, x16
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x17
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM59_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x27, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x19
        b.ne            error

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM59_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x28, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x28, x21
        b.ne            error

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM59_GOLDEN_VALUE_6

        // Compare local and golden signature
        cmp             x29, x22
        b.ne            error

check_correct_result_BM_59:
        // Compare local and golden signature
        cmp             x29, x23
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 59
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 60
//-----------------------------------------------------------

// Basic Module 60
// CSINV <Xd>, <Xn>, <Xm>, <cond>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x0, =A53_STL_BM60_INPUT_VALUE_1
        ldr             x1, =A53_STL_BM60_INPUT_VALUE_2
        ldr             x2, =A53_STL_BM60_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM60_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM60_INPUT_VALUE_3
        ldr             x5, =A53_STL_BM60_INPUT_VALUE_4
        ldr             x6, =A53_STL_BM60_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM60_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM60_INPUT_VALUE_5
        ldr             x9, =A53_STL_BM60_INPUT_VALUE_5
        ldr             x10, =A53_STL_BM60_INPUT_VALUE_6
        ldr             x11, =A53_STL_BM60_INPUT_VALUE_6

        // Clear NZCV flags
        mov             x12, A53_STL_NZCV_CLR
        msr             nzcv, x12

        // 1st test: TRUE condition
        csinv           x12, x0, x1, ne
        csinv           x13, x2, x3, ne
        // 2nd test: FALSE condition
        csinv           x14, x4, x5, eq
        csinv           x15, x6, x7, eq
        // 3rd test: TRUE condition with CINV alias
        csinv           x16, x8, x8, eq
        csinv           x17, x9, x9, eq
        // 4th test: FALSE condition with CINV alias
        csinv           x18, x10, x10, ne
        csinv           x19, x11, x11, ne
        // 5th test: TRUE condition with CSETM alias
        csinv           x20, xzr, xzr, eq
        csinv           x21, xzr, xzr, eq
        // 6th test: FALSE condition with CSETM alias
        csinv           x22, xzr, xzr, ne
        csinv           x23, xzr, xzr, ne

        // Initialize x24 with golden signature
        ldr             x24, =A53_STL_BM60_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x24, x12
        b.ne            error

        // Compare local and golden signature
        cmp             x24, x13
        b.ne            error

        // initialize x25 with golden signature
        ldr             x25, =A53_STL_BM60_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x25, x14
        b.ne            error

        // Compare local and golden signature
        cmp             x25, x15
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM60_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x26, x16
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x17
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x19
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM60_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x21
        b.ne            error

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM60_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x28, x22
        b.ne            error

check_correct_result_BM_60:
        // Compare local and golden signature
        cmp             x28, x23
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 60
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 61
//-----------------------------------------------------------

// Basic Module 61
// CSNEG <Xd>, <Xn>, <Xm>, <cond>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x0, =A53_STL_BM61_INPUT_VALUE_1
        ldr             x1, =A53_STL_BM61_INPUT_VALUE_2
        ldr             x2, =A53_STL_BM61_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM61_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM61_INPUT_VALUE_3
        ldr             x5, =A53_STL_BM61_INPUT_VALUE_4
        ldr             x6, =A53_STL_BM61_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM61_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM61_INPUT_VALUE_5
        ldr             x9, =A53_STL_BM61_INPUT_VALUE_5
        ldr             x10, =A53_STL_BM61_INPUT_VALUE_6
        ldr             x11, =A53_STL_BM61_INPUT_VALUE_6

        // Clear NZCV flags
        mov             x12, A53_STL_NZCV_CLR
        msr             nzcv, x12

        // 1st test: TRUE condition
        csneg           x12, x0, x1, ne
        csneg           x13, x2, x3, ne
        // 2nd test: FALSE condition
        csneg           x14, x4, x5, eq
        csneg           x15, x6, x7, eq
        // 3rd test: TRUE condition with CNEG alias
        csneg           x16, x8, x8, eq
        csneg           x17, x9, x9, eq
        // 4th test: FALSE condition with CNEG alias
        csneg           x18, x10, x10, ne
        csneg           x19, x11, x11, ne

        // Initialize x24 with golden signature
        ldr             x24, =A53_STL_BM61_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x24, x12
        b.ne            error

        // Compare local and golden signature
        cmp             x24, x13
        b.ne            error

        // initialize x25 with golden signature
        ldr             x25, =A53_STL_BM61_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x25, x14
        b.ne            error

        // Compare local and golden signature
        cmp             x25, x15
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM61_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x26, x16
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x17
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM61_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x27, x18
        b.ne            error

check_correct_result_BM_61:
        // Compare local and golden signature
        cmp             x27, x19
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 61
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

a53_stl_core_p012_n001_end:

        ret

        .end
