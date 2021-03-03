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
//      CORE_025
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p025_n001
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

        .section .section_a53_stl_core_p025_n001,"ax",%progbits
        .global a53_stl_core_p025_n001
        .type a53_stl_core_p025_n001, %function

a53_stl_core_p025_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 153
//-----------------------------------------------------------

// Basic Module 153
// FMUL (scalar, double precision) test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x0, =DATA_SET_03_FMUL
        // Add offset to LUT in order to point first test element (if offset is #0 start from LUT beginning)
        add             x0, x0, A53_STL_BM153_LUT_OFFSET
        ldr             x1, =DATA_SET_03_END
        ldr             x1, [x1]

loop_FMUL:

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

        // Initialize x2 with the first expected value to check the error in the check_x26x29_x18x24
        mov             x2, x26

        bl              fmov_d2d17_x4x17

        // FMUL <Dd>, <Dn>, <Dm>

        fmul            d18, d2, d3
        fmul            d19, d4, d5

        fmul            d20, d6, d7
        fmul            d21, d8, d9

        fmul            d22, d10, d11
        fmul            d23, d12, d13

        fmul            d24, d14, d15
        fmul            d25, d16, d17

        bl              fmov_x18x25_d18d25

        // Check results
        bl              check_x26x29_x18x24
        cmp             x26, x2
        b.ne            error

check_correct_result_BM_153:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error

        // Decrement LUT counter to end by 12 elements
        sub             x1, x1, A53_STL_LUT_CNT_DECR_12

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x1, A53_STL_BM153_LUT_FINISH
        bne             loop_FMUL

//-----------------------------------------------------------
// END BASIC MODULE 153
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 184
//-----------------------------------------------------------

// Basic Module 184
// FMUL (vector) test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x0, =DATA_SET_03_FMUL
        // Add offset to LUT in order to point first test element (if offset is #0 start from LUT beginning)
        add             x0, x0, A53_STL_BM184_LUT_OFFSET
        ldr             x1, =DATA_SET_03_END
        ldr             x1, [x1]

loop_FMUL_v:

        // Load first couple of operands from LUT
        ldp             x2, x3, [x0], A53_STL_LUT_COUPLE_INCR
        // Load first couple of operands result from LUT
        ldr             x26, [x0], A53_STL_LUT_SINGLE_INCR
        // Load second couple of operands from LUT
        ldp             x6, x7, [x0], A53_STL_LUT_COUPLE_INCR
        // Load second couple of operands result from LUT
        ldr             x27, [x0], A53_STL_LUT_SINGLE_INCR
        // Load third couple of operands from LUT
        ldp             x10, x11, [x0], A53_STL_LUT_COUPLE_INCR
        // Load third couple of operands result from LUT
        ldr             x28, [x0], A53_STL_LUT_SINGLE_INCR
        // Load fourth couple of operands from LUT
        ldp             x14, x15, [x0], A53_STL_LUT_COUPLE_INCR
        // Load fourth couple of operands result from LUT
        ldr             x29, [x0], A53_STL_LUT_SINGLE_INCR

        // Initialize x4 with the first expected value to check the error in the check_x26x29_x18x24
        mov             x4, x26

        bl              fmov_v2v17_x2x15

        // FMUL <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        fmul            v18.2d, v2.2d, v3.2d
        fmul            v19.2d, v4.2d, v5.2d

        fmul            v20.2d, v6.2d, v7.2d
        fmul            v21.2d, v8.2d, v9.2d

        fmul            v22.2d, v10.2d, v11.2d
        fmul            v23.2d, v12.2d, v13.2d

        fmul            v24.2d, v14.2d, v15.2d
        fmul            v25.2d, v16.2d, v17.2d

        // Check results

        bl              fmov_x18x25_v18v25

        // Check results
        bl              check_x26x29_x18x24
        cmp             x26, x4
        b.ne            error

check_correct_result_BM_184:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error

        // Decrement LUT counter to end by 12 elements
        sub             x1, x1, A53_STL_LUT_CNT_DECR_12

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x1, A53_STL_BM184_LUT_FINISH
        bne             loop_FMUL_v


//-----------------------------------------------------------
// END BASIC MODULE 184
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

a53_stl_core_p025_n001_end:

        ret

        .end
