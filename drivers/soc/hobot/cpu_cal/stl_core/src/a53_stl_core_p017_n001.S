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
//      Testing of SIMD and FP instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_FPU
//
//   About: TEST_ID
//      CORE_017
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p017_n001
//      Testing of SIMD and FP instructions
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

        .section .section_a53_stl_core_p017_n001,"ax",%progbits
        .global a53_stl_core_p017_n001
        .type a53_stl_core_p017_n001, %function

a53_stl_core_p017_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 120
//-----------------------------------------------------------

// Basic Module 120
// FMADD <Sd>, <Sn>, <Sm>, <Sa>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x0, =DATA_SET_13_FMADD_S
        // Add offset to LUT in order to point first test element (if offset is #0 start from LUT beginning)
        add             x0, x0, A53_STL_BM120_LUT_OFFSET
        ldr             x1, =DATA_SET_13_END
        ldr             x1, [x1]

loop_FMADD_single:

        // Load first couple of operands from LUT
        ldp             w2, w3, [x0], A53_STL_LUT_SINGLE_INCR
        // Load first couple of operands accumulation value and result from LUT
        ldp             w4, w26, [x0], A53_STL_LUT_SINGLE_INCR
        // Load second couple of operands from LUT
        ldp             w6, w7, [x0], A53_STL_LUT_SINGLE_INCR
        // Load second couple of operands accumulation value and result from LUT
        ldp             w8, w27, [x0], A53_STL_LUT_SINGLE_INCR
        // Load third couple of operands from LUT
        ldp             w10, w11, [x0], A53_STL_LUT_SINGLE_INCR
        // Load third couple of operands accumulation value and result from LUT
        ldp             w12, w28, [x0], A53_STL_LUT_SINGLE_INCR
        // Load fourth couple of operands from LUT
        ldp             w14, w15, [x0], A53_STL_LUT_SINGLE_INCR
        // Load fourth couple of operands accumulation value and result from LUT
        ldp             w16, w29, [x0], A53_STL_LUT_SINGLE_INCR

        bl              fmov_d0d23_x2x16

        // FMADD <Sd>, <Sn>, <Sm>, <Sa>

        fmadd           s24, s0, s1, s2
        fmadd           s25, s3, s4, s5

        fmadd           s26, s6, s7, s8
        fmadd           s27, s9, s10, s11

        fmadd           s28, s12, s13, s14
        fmadd           s29, s15, s16, s17

        fmadd           s30, s18, s19, s20
        fmadd           s31, s21, s22, s23

        // Check results

        fmov            w18, s24
        fmov            w19, s25
        fmov            w20, s26
        fmov            w21, s27
        fmov            w22, s28
        fmov            w23, s29
        fmov            w24, s30
        fmov            w25, s31

        // compare local and golden signature
        cmp             w26, w18
        b.ne            error

        // compare local and golden signature
        cmp             w26, w19
        b.ne            error

        // compare local and golden signature
        cmp             w27, w20
        b.ne            error

        // compare local and golden signature
        cmp             w27, w21
        b.ne            error

        // compare local and golden signature
        cmp             w28, w22
        b.ne            error

        // compare local and golden signature
        cmp             w28, w23
        b.ne            error

        // compare local and golden signature
        cmp             w29, w24
        b.ne            error

check_correct_result_BM_120:
        // compare local and golden signature
        cmp             w29, w25
        b.ne            error

        // Decrement LUT counter to end by 16 elements
        sub             x1, x1, A53_STL_LUT_CNT_DECR_16

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x1, A53_STL_BM120_LUT_FINISH
        bne             loop_FMADD_single

//-----------------------------------------------------------
// END BASIC MODULE 120
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

a53_stl_core_p017_n001_end:

        ret

        .end
