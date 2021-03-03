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
//      CORE_021
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p021_n001
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

        .section .section_a53_stl_core_p021_n001,"ax",%progbits
        .global a53_stl_core_p021_n001
        .type a53_stl_core_p021_n001, %function

a53_stl_core_p021_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 121
//-----------------------------------------------------------

// Basic Module 121
// FMSUB <Dd>, <Dn>, <Dm>, <Da>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x0, =DATA_SET_14_FMSUB
        // Add offset to LUT in order to point first test element (if offset is #0 start from LUT beginning)
        add             x0, x0, A53_STL_BM121_LUT_OFFSET
        ldr             x1, =DATA_SET_14_END
        ldr             x1, [x1]

loop_FMSUB:

        // Load first couple of operands from LUT
        ldp             x2, x3, [x0], A53_STL_LUT_PAIR_INCR
        // Load first couple of operands accumulation value and result from LUT
        ldp             x4, x26, [x0], A53_STL_LUT_PAIR_INCR
        // Load second couple of operands from LUT
        ldp             x6, x7, [x0], A53_STL_LUT_PAIR_INCR
        // Load second couple of operands accumulation value and result from LUT
        ldp             x8, x27, [x0], A53_STL_LUT_PAIR_INCR
        // Load third couple of operands from LUT
        ldp             x10, x11, [x0], A53_STL_LUT_PAIR_INCR
        // Load third couple of operands accumulation value and result from LUT
        ldp             x12, x28, [x0], A53_STL_LUT_PAIR_INCR
        // Load fourth couple of operands from LUT
        ldp             x14, x15, [x0], A53_STL_LUT_PAIR_INCR
        // Load fourth couple of operands accumulation value and result from LUT
        ldp             x16, x29, [x0], A53_STL_LUT_PAIR_INCR

        // Initialize x5 with the first expected value to check the error in the check_x26x29_x18x24
        mov             x5, x26

        bl              fmov_d0d23_x2x16

        // FMSUB <Dd>, <Dn>, <Dm>, <Da>

        fmsub           d24, d0, d1, d2
        fmsub           d25, d3, d4, d5

        fmsub           d26, d6, d7, d8
        fmsub           d27, d9, d10, d11

        fmsub           d28, d12, d13, d14
        fmsub           d29, d15, d16, d17

        fmsub           d30, d18, d19, d20
        fmsub           d31, d21, d22, d23

        // Check results

        fmov            x18, d24
        fmov            x19, d25
        fmov            x20, d26
        fmov            x21, d27
        fmov            x22, d28
        fmov            x23, d29
        fmov            x24, d30
        fmov            x25, d31

        // Check results
        bl              check_x26x29_x18x24
        cmp             x26, x5
        b.ne            error

check_correct_result_BM_121:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error

        // Decrement LUT counter to end by 16 elements
        sub             x1, x1, A53_STL_LUT_CNT_DECR_16

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x1, A53_STL_BM121_LUT_FINISH
        bne             loop_FMSUB


//-----------------------------------------------------------
// END BASIC MODULE 121
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

a53_stl_core_p021_n001_end:

        ret

        .end
