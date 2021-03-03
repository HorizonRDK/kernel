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
//      Testing of logical instructions as AND, ANDS, EOR and ORR instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_002
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p002_n001
//      Testing of logical instructions as AND, ANDS, EOR and ORR instructions
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

        .section .section_a53_stl_core_p002_n001,"ax",%progbits
        .global a53_stl_core_p002_n001
        .type a53_stl_core_p002_n001, %function

a53_stl_core_p002_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]
        // call preamble subroutine
        bl              a53_stl_preamble


        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 03
//-----------------------------------------------------------

// Basic Module 03
// AND <Xd|SP>, <Xn>, #<imm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand registers

        ldr             x2, =A53_STL_BM03_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM03_INPUT_VALUE_1
        ldr             x10, =A53_STL_BM03_INPUT_VALUE_2
        ldr             x12, =A53_STL_BM03_INPUT_VALUE_2

        and             x18, x2, A53_STL_BM03_IMM_VALUE_1
        and             x19, x4, A53_STL_BM03_IMM_VALUE_1

        and             x20, x2, A53_STL_BM03_IMM_VALUE_2
        and             x21, x4, A53_STL_BM03_IMM_VALUE_2

        and             x22, x10, A53_STL_BM03_IMM_VALUE_1
        and             x23, x12, A53_STL_BM03_IMM_VALUE_1

        and             x24, x10, A53_STL_BM03_IMM_VALUE_2
        and             x25, x12, A53_STL_BM03_IMM_VALUE_2

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM03_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM03_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM03_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM03_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM03_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_03:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 03
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 04
//-----------------------------------------------------------

// Basic Module 04
// ANDS <Xd|SP>, <Xn>, #<imm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand registers

        ldr             x1, =A53_STL_BM04_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM04_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM04_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM04_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM04_INPUT_VALUE_2
        ldr             x7, =A53_STL_BM04_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM04_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM04_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM04_INPUT_VALUE_3
        ldr             x11, =A53_STL_BM04_INPUT_VALUE_4

        // Test N = 1, Z = 0, C = 0, V = 0

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        ands            x18, x1, A53_STL_BM04_IMM_VALUE_1
        ands            x19, x2, A53_STL_BM04_IMM_VALUE_1
        ands            x20, x3, A53_STL_BM04_IMM_VALUE_2

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM04_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        ands            x21, x4, A53_STL_BM04_IMM_VALUE_1
        ands            x22, x5, A53_STL_BM04_IMM_VALUE_2
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM04_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM04_GOLDEN_VALUE_1

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
        ldr             x27, =A53_STL_BM04_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x22
        b.ne            error

        // Test N = 0, Z = 1, C = 0, V = 0

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        ands            x18, x7, A53_STL_BM04_IMM_VALUE_3
        ands            x19, x8, A53_STL_BM04_IMM_VALUE_3
        ands            x20, x9, A53_STL_BM04_IMM_VALUE_4

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM04_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        ands            x21, x10, A53_STL_BM04_IMM_VALUE_3
        ands            x22, x11, A53_STL_BM04_IMM_VALUE_4
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM04_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM04_GOLDEN_VALUE_3

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
        ldr             x27, =A53_STL_BM04_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

check_correct_result_BM_04:
        // Compare local and golden signature
        cmp             x27, x22
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 04
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 05
//-----------------------------------------------------------

// Basic Module 05
// EOR <Xd|SP>, <Xn>, #<imm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM05_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM05_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM05_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM05_INPUT_VALUE_2
        ldr             x10, =A53_STL_BM05_INPUT_VALUE_3
        ldr             x11, =A53_STL_BM05_INPUT_VALUE_4
        ldr             x12, =A53_STL_BM05_INPUT_VALUE_3
        ldr             x13, =A53_STL_BM05_INPUT_VALUE_4

        eor             x18, x2, A53_STL_BM05_IMM_VALUE_1
        eor             x19, x4, A53_STL_BM05_IMM_VALUE_1

        eor             x20, x3, A53_STL_BM05_IMM_VALUE_2
        eor             x21, x5, A53_STL_BM05_IMM_VALUE_2

        eor             x22, x10, A53_STL_BM05_IMM_VALUE_3
        eor             x23, x12, A53_STL_BM05_IMM_VALUE_3

        eor             x24, x11, A53_STL_BM05_IMM_VALUE_4
        eor             x25, x13, A53_STL_BM05_IMM_VALUE_4

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM05_GOLDEN_VALUE_1

        // initialize x27 with golden signature
        ldr             x27, =A53_STL_BM05_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM05_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM05_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM05_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_05:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 05
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 06
//-----------------------------------------------------------

// Basic Module 06
// ORR <Xd|SP>, <Xn>, #<imm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM06_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM06_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM06_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM06_INPUT_VALUE_2
        ldr             x10, =A53_STL_BM06_INPUT_VALUE_3
        ldr             x11, =A53_STL_BM06_INPUT_VALUE_4
        ldr             x12, =A53_STL_BM06_INPUT_VALUE_3
        ldr             x13, =A53_STL_BM06_INPUT_VALUE_4

        orr             x18, x2, A53_STL_BM06_IMM_VALUE_1
        orr             x19, x4, A53_STL_BM06_IMM_VALUE_1

        orr             x20, x3, A53_STL_BM06_IMM_VALUE_2
        orr             x21, x5, A53_STL_BM06_IMM_VALUE_2

        orr             x22, x10, A53_STL_BM06_IMM_VALUE_3
        orr             x23, x12, A53_STL_BM06_IMM_VALUE_3

        orr             x24, x11, A53_STL_BM06_IMM_VALUE_4
        orr             x25, x13, A53_STL_BM06_IMM_VALUE_4

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM06_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM06_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM06_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM06_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM06_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_06:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 06
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

a53_stl_core_p002_n001_end:

        ret

        .end
