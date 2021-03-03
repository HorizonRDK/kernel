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
//      Testing of FMAX and FMIN instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_FPU
//
//   About: TEST_ID
//      CORE_024
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p024_n001
//      Testing of FMAX and FMIN instructions
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

        .section .section_a53_stl_core_p024_n001,"ax",%progbits
        .global a53_stl_core_p024_n001
        .type a53_stl_core_p024_n001, %function

a53_stl_core_p024_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 144
//-----------------------------------------------------------

// Basic Module
// FMAX (scalar, double precision) test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x2, =A53_STL_BM144_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM144_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM144_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM144_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM144_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM144_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM144_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM144_INPUT_VALUE_4

        bl              fmov_d2d9_x2x9

        // FMAX <Dd>, <Dn>, <Dm>

        fmax            d18, d2, d3
        fmax            d19, d4, d5

        fmax            d20, d6, d7
        fmax            d21, d8, d9

        // initialize x26 with golden signature
        ldr             x26, =A53_STL_BM144_GOLDEN_VALUE_1
        fmov            d26, x26

        // compare local and golden signature
        fcmp            d26, d18
        b.ne            error

        // compare local and golden signature
        fcmp            d26, d19
        b.ne            error

        // initialize x27 with golden signature
        ldr             x27, =A53_STL_BM144_GOLDEN_VALUE_2
        fmov            d27, x27

        // compare local and golden signature
        fcmp            d27, d20
        b.ne            error

check_correct_result_BM_144:
        // compare local and golden signature
        fcmp            d27, d21
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 144
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 359
//-----------------------------------------------------------

// Basic Module
// FMAX (scalar) test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x2, =A53_STL_BM359_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM359_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM359_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM359_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM359_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM359_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM359_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM359_INPUT_VALUE_4

        bl              fmov_d2d9_x2x9

        // FMAX <Sd>, <Sn>, <Sm>

        fmax            s18, s2, s3
        fmax            s19, s4, s5

        fmax            s20, s6, s7
        fmax            s21, s8, s9

        // initialize x26 with golden signature
        ldr             x26, =A53_STL_BM359_GOLDEN_VALUE_1
        fmov            d26, x26

        // compare local and golden signature
        fcmp            d26, d18
        b.ne            error

        // compare local and golden signature
        fcmp            d26, d19
        b.ne            error

        // initialize x27 with golden signature
        ldr             x27, =A53_STL_BM359_GOLDEN_VALUE_2
        fmov            d27, x27

        // compare local and golden signature
        fcmp            d27, d20
        b.ne            error

check_correct_result_BM_359:
        // compare local and golden signature
        fcmp            d27, d21
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 359
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 146
//-----------------------------------------------------------

// Basic Module
// FMIN (scalar, double precision) test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x2, =A53_STL_BM146_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM146_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM146_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM146_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM146_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM146_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM146_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM146_INPUT_VALUE_4

        bl              fmov_d2d9_x2x9

        // FMIN <Dd>, <Dn>, <Dm>

        fmin            d18, d2, d3
        fmin            d19, d4, d5

        fmin            d20, d6, d7
        fmin            d21, d8, d9

        // initialize x26 with golden signature
        ldr             x26, =A53_STL_BM146_GOLDEN_VALUE_1
        fmov            d26, x26

        // compare local and golden signature
        fcmp            d26, d18
        b.ne            error

        // compare local and golden signature
        fcmp            d26, d19
        b.ne            error

        // initialize x27 with golden signature
        ldr             x27, =A53_STL_BM146_GOLDEN_VALUE_2
        fmov            d27, x27

        // compare local and golden signature
        fcmp            d27, d20
        b.ne            error

check_correct_result_BM_146:
        // compare local and golden signature
        fcmp            d27, d21
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 146
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 360
//-----------------------------------------------------------

// Basic Module
// FMIN (scalar) test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x2, =A53_STL_BM360_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM360_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM360_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM360_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM360_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM360_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM360_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM360_INPUT_VALUE_4

        bl              fmov_d2d9_x2x9

        // FMIN <Sd>, <Sn>, <Sm>

        fmin            s18, s2, s3
        fmin            s19, s4, s5

        fmin            s20, s6, s7
        fmin            s21, s8, s9

        // initialize x26 with golden signature
        ldr             x26, =A53_STL_BM360_GOLDEN_VALUE_1
        fmov            d26, x26

        // compare local and golden signature
        fcmp            d26, d18
        b.ne            error

        // compare local and golden signature
        fcmp            d26, d19
        b.ne            error

        // initialize x27 with golden signature
        ldr             x27, =A53_STL_BM360_GOLDEN_VALUE_2
        fmov            d27, x27

        // compare local and golden signature
        fcmp            d27, d20
        b.ne            error

check_correct_result_BM_360:
        // compare local and golden signature
        fcmp            d27, d21
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 360
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

a53_stl_core_p024_n001_end:

        ret

        .end
