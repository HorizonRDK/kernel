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
//      Testing of FMUL, FSQRT and FDIV instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_FPU
//
//   About: TEST_ID
//      CORE_047
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p047_n001
//      Testing of FMUL, FSQRT and FDIV instructions
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

        .section .section_a53_stl_core_p047_n001,"ax",%progbits
        .global a53_stl_core_p047_n001
        .type a53_stl_core_p047_n001, %function

a53_stl_core_p047_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 361
//-----------------------------------------------------------

// Basic Module 361
// FMUL (scalar) test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x0, =DATA_SET_05_FMUL
        // Add offset to LUT in order to point first test element (if offset is #0 start from LUT beginning)
        add             x0, x0, A53_STL_BM361_LUT_OFFSET
        ldr             x1, =DATA_SET_05_END
        ldr             x1, [x1]

loop_FMUL_s:

        // Load first couple of operands from LUT
        ldp             x4, x5, [x0], A53_STL_LUT_COUPLE_INCR
        // Load first couple of operands result from LUT
        ldr             x26, [x0], A53_STL_LUT_SINGLE_INCR
        // Load second couple of operands from LUT
        ldp             x8, x9, [x0], A53_STL_LUT_COUPLE_INCR
        // Load second couple of operands result from LUT
        ldr             x27, [x0], A53_STL_LUT_SINGLE_INCR
        // Load third couple of operands from LUT
        ldp             x12, x13, [x0], A53_STL_LUT_COUPLE_INCR
        // Load third couple of operands result from LUT
        ldr             x28, [x0], A53_STL_LUT_SINGLE_INCR
        // Load fourth couple of operands from LUT
        ldp             x16, x17, [x0], A53_STL_LUT_COUPLE_INCR
        // Load fourth couple of operands result from LUT
        ldr             x29, [x0], A53_STL_LUT_SINGLE_INCR

        // Initialize x2 with the first expected value to check the error in the check_x26x29_x15x8_x18x24
        mov             x2, x26

        bl              fmov_d2d17_x4x17

        // FMUL <Sd>, <Sn>, <Sm>

        fmul            s18, s2, s3
        fmul            s19, s4, s5

        fmul            s20, s6, s7
        fmul            s21, s8, s9

        fmul            s22, s10, s11
        fmul            s23, s12, s13

        fmul            s24, s14, s15
        fmul            s25, s16, s17

        bl              fmov_x18x25_d18d25

        // Check results
        bl              check_x26x29_x18x24
        cmp             x26, x2
        b.ne            error

check_correct_result_BM_361:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error

        // Decrement LUT counter to end by 12 elements
        sub             x1, x1, A53_STL_LUT_CNT_DECR_12

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x1, A53_STL_BM361_LUT_FINISH
        bne             loop_FMUL_s


//-----------------------------------------------------------
// END BASIC MODULE 361
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 159
//-----------------------------------------------------------

// Basic Module 159
// FSQRT (scalar, double precision) test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x2, =A53_STL_BM159_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM159_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM159_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM159_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM159_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM159_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM159_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM159_INPUT_VALUE_4

        bl              fmov_d2d9_x2x9

        // FSQRT <Dd>, <Dn>

        fsqrt           d18, d2
        fsqrt           d19, d3

        fsqrt           d20, d4
        fsqrt           d21, d5

        fsqrt           d22, d6
        fsqrt           d23, d7

        fsqrt           d24, d8
        fsqrt           d25, d9

        bl              fmov_x18x25_d18d25

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM159_GOLDEN_VALUE_1

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM159_GOLDEN_VALUE_2

        // initialize w28 with golden signature
        ldr             x28, =A53_STL_BM159_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM159_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM159_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_159:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 159
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

a53_stl_core_p047_n001_end:

        ret

        .end
