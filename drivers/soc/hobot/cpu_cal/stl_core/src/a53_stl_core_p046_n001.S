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
//      CORE_046
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p046_n001
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

        .section .section_a53_stl_core_p046_n001,"ax",%progbits
        .global a53_stl_core_p046_n001
        .type a53_stl_core_p046_n001, %function

a53_stl_core_p046_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping


//-----------------------------------------------------------
// START BASIC MODULE 62
//-----------------------------------------------------------

// Basic Module 62
// CCMN <Xn>, <Xm>, #<nzcv>, <cond>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x1, =A53_STL_BM62_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM62_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM62_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM62_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM62_INPUT_VALUE_3
        ldr             x6, =A53_STL_BM62_INPUT_VALUE_2
        ldr             x7, =A53_STL_BM62_INPUT_VALUE_1
        ldr             x8, =A53_STL_BM62_INPUT_VALUE_2
        ldr             x9, =A53_STL_BM62_INPUT_VALUE_3
        ldr             x10, =A53_STL_BM62_INPUT_VALUE_2
        ldr             x11, =A53_STL_BM62_INPUT_VALUE_4
        ldr             x12, =A53_STL_BM62_INPUT_VALUE_2
        ldr             x13, =A53_STL_BM62_INPUT_VALUE_4
        ldr             x14, =A53_STL_BM62_INPUT_VALUE_2
        ldr             x15, =A53_STL_BM62_INPUT_VALUE_5
        ldr             x16, =A53_STL_BM62_INPUT_VALUE_2
        ldr             x17, =A53_STL_BM62_INPUT_VALUE_4
        ldr             x18, =A53_STL_BM62_INPUT_VALUE_2
        ldr             x19, =A53_STL_BM62_INPUT_VALUE_5
        ldr             x20, =A53_STL_BM62_INPUT_VALUE_2

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 1st test: TRUE condition, NZCV flags = 0x8
        ccmn            x1, x2, #A53_STL_BM62_IMM_VALUE_1, ne
        ccmn            x3, x4, #A53_STL_BM62_IMM_VALUE_1, ne
        ccmn            x5, x6, #A53_STL_BM62_IMM_VALUE_2, ge

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM62_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 2nd test: TRUE condition, NZCV flags = 0x8
        ccmn            x7, x8, #A53_STL_BM62_IMM_VALUE_1, ne
        ccmn            x9, x10, #A53_STL_BM62_IMM_VALUE_2, ge
        // Necessary for dual issue pipeline stimulation strategy
        mov             x1, x3

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM62_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 3rd test: FALSE condition, NZCV flags = 0x7
        ccmn            x11, x12, #A53_STL_BM62_IMM_VALUE_1, ne
        ccmn            x13, x14, #A53_STL_BM62_IMM_VALUE_1, ne
        ccmn            x15, x16, #A53_STL_BM62_IMM_VALUE_3, eq

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM62_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 4th test: FALSE condition, NZCV flags = 0x7
        ccmn            x17, x18, #A53_STL_BM62_IMM_VALUE_1, ne
        ccmn            x19, x20, #A53_STL_BM62_IMM_VALUE_3, eq
        // Necessary for dual issue pipeline stimulation strategy
        mov             x11, x13

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM62_GOLDEN_FLAGS_2

check_correct_result_BM_62:
        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 62
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 63
//-----------------------------------------------------------

// Basic Module 63
// CCMP <Xn>, <Xm>, #<nzcv>, <cond>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x1, =A53_STL_BM63_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM63_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM63_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM63_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM63_INPUT_VALUE_3
        ldr             x6, =A53_STL_BM63_INPUT_VALUE_2
        ldr             x7, =A53_STL_BM63_INPUT_VALUE_1
        ldr             x8, =A53_STL_BM63_INPUT_VALUE_2
        ldr             x9, =A53_STL_BM63_INPUT_VALUE_3
        ldr             x10, =A53_STL_BM63_INPUT_VALUE_2
        ldr             x11, =A53_STL_BM63_INPUT_VALUE_4
        ldr             x12, =A53_STL_BM63_INPUT_VALUE_5
        ldr             x13, =A53_STL_BM63_INPUT_VALUE_4
        ldr             x14, =A53_STL_BM63_INPUT_VALUE_5
        ldr             x15, =A53_STL_BM63_INPUT_VALUE_6
        ldr             x16, =A53_STL_BM63_INPUT_VALUE_5
        ldr             x17, =A53_STL_BM63_INPUT_VALUE_4
        ldr             x18, =A53_STL_BM63_INPUT_VALUE_5
        ldr             x19, =A53_STL_BM63_INPUT_VALUE_6
        ldr             x20, =A53_STL_BM63_INPUT_VALUE_5

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 1st test: TRUE condition, NZCV flags = 0x8
        ccmp            x1, x2, #A53_STL_BM63_IMM_VALUE_1, eq
        ccmp            x3, x4, #A53_STL_BM63_IMM_VALUE_1, eq
        ccmp            x5, x6, #A53_STL_BM63_IMM_VALUE_2, ne

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM63_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 2nd test: TRUE condition, NZCV flags = 0x8
        ccmp            x7, x8, #A53_STL_BM63_IMM_VALUE_1, eq
        ccmp            x9, x10, #A53_STL_BM63_IMM_VALUE_2, ne
        // Necessary for dual issue pipeline stimulation strategy
        mov             x1, x3

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM63_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 3rd test: FALSE condition, NZCV flags = 0x7
        ccmp            x11, x12, #A53_STL_BM63_IMM_VALUE_1, eq
        ccmp            x13, x14, #A53_STL_BM63_IMM_VALUE_1, eq
        ccmp            x15, x16, #A53_STL_BM63_IMM_VALUE_3, eq

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM63_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 4th test: FALSE condition, NZCV flags = 0x7
        ccmp            x17, x18, #A53_STL_BM63_IMM_VALUE_1, eq
        ccmp            x19, x20, #A53_STL_BM63_IMM_VALUE_3, eq
        // Necessary for dual issue pipeline stimulation strategy
        mov             x11, x13

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM63_GOLDEN_FLAGS_2

check_correct_result_BM_63:
        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 63
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 64
//-----------------------------------------------------------

// Basic Module 64
// CCMP <Xn>, #<imm>, #<nzcv>, <cond>
        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x1, =A53_STL_BM64_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM64_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM64_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM64_INPUT_VALUE_1
        ldr             x9, =A53_STL_BM64_INPUT_VALUE_3
        ldr             x11, =A53_STL_BM64_INPUT_VALUE_4
        ldr             x13, =A53_STL_BM64_INPUT_VALUE_4
        ldr             x15, =A53_STL_BM64_INPUT_VALUE_6
        ldr             x17, =A53_STL_BM64_INPUT_VALUE_4
        ldr             x19, =A53_STL_BM64_INPUT_VALUE_6

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 1st test: TRUE condition, NZCV flags = 0x2
        ccmp            x1, #A53_STL_BM64_INPUT_VALUE_2, #A53_STL_BM64_IMM_VALUE_1, eq
        ccmp            x3, #A53_STL_BM64_INPUT_VALUE_2, #A53_STL_BM64_IMM_VALUE_1, eq
        ccmp            x5, #A53_STL_BM64_INPUT_VALUE_2, #A53_STL_BM64_IMM_VALUE_2, ne

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM64_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 2nd test: TRUE condition, NZCV flags = 0x2
        ccmp            x7, #A53_STL_BM64_INPUT_VALUE_2, #A53_STL_BM64_IMM_VALUE_1, eq
        ccmp            x9, #A53_STL_BM64_INPUT_VALUE_2, #A53_STL_BM64_IMM_VALUE_2, ne
        // Necessary for dual issue pipeline stimulation strategy
        mov             x1, x3

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM64_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 3rd test: FALSE condition, NZCV flags = 0xD
        ccmp            x11, #A53_STL_BM64_INPUT_VALUE_5, #A53_STL_BM64_IMM_VALUE_1, eq
        ccmp            x13, #A53_STL_BM64_INPUT_VALUE_5, #A53_STL_BM64_IMM_VALUE_1, eq
        ccmp            x15, #A53_STL_BM64_INPUT_VALUE_5, #A53_STL_BM64_IMM_VALUE_3, eq

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM64_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // 4th test: FALSE condition, NZCV flags = 0xD
        ccmp            x17, #A53_STL_BM64_INPUT_VALUE_5, #A53_STL_BM64_IMM_VALUE_1, eq
        ccmp            x19, #A53_STL_BM64_INPUT_VALUE_5, #A53_STL_BM64_IMM_VALUE_3, eq
        // Necessary for dual issue pipeline stimulation strategy
        mov             x11, x13

        // Check NZCV flags

        mrs             x21, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM64_GOLDEN_FLAGS_2

check_correct_result_BM_64:
        // Compare local and golden flags
        cmp             x0, x21
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 64
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

a53_stl_core_p046_n001_end:

        ret

        .end
