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
//      CORE_029
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p029_n001
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

        .section .section_a53_stl_core_p029_n001,"ax",%progbits
        .global a53_stl_core_p029_n001
        .type a53_stl_core_p029_n001, %function

a53_stl_core_p029_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 71
//-----------------------------------------------------------

// Basic Module 71
// FMADD (double precision) with Dual Issue

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x0, =DATA_SET_08_FMADD
        // Add offset to LUT in order to point first test element (if offset is #0 start from LUT beginning)
        add             x0, x0, A53_STL_BM71_LUT_OFFSET
        ldr             x1, =DATA_SET_08_END
        ldr             x1, [x1]

loop_FMADD_v:

        // Load first couple of operands from LUT
        ldp             x2, x3, [x0], A53_STL_LUT_COUPLE_INCR
        // Load first couple of operands accumulation value and result from LUT
        ldp             x4, x26, [x0], A53_STL_LUT_COUPLE_INCR
        // Load second couple of operands from LUT
        ldp             x6, x7, [x0], A53_STL_LUT_COUPLE_INCR
        // Load second couple of operands accumulation value and result from LUT
        ldp             x8, x27, [x0], A53_STL_LUT_COUPLE_INCR
        // Load third couple of operands from LUT
        ldp             x10, x11, [x0], A53_STL_LUT_COUPLE_INCR
        // Load third couple of operands accumulation value and result from LUT
        ldp             x12, x28, [x0], A53_STL_LUT_COUPLE_INCR
        // Load fourth couple of operands from LUT
        ldp             x14, x15, [x0], A53_STL_LUT_COUPLE_INCR
        // Load fourth couple of operands accumulation value and result from LUT
        ldp             x16, x29, [x0], A53_STL_LUT_COUPLE_INCR

        // Initialize x5 with the first expected value to check the error in the check_x26x29_x15x8_x18x24
        mov             x5, x26

        bl              fmov_d0d23_x2x16

        // FMADD <Dd>, <Dn>, <Dm>, <Da>

        fmadd           d24, d0, d1, d2
        fmadd           d25, d3, d4, d5

        fmadd           d26, d6, d7, d8
        fmadd           d27, d9, d10, d11

        fmadd           d28, d12, d13, d14
        fmadd           d29, d15, d16, d17

        fmadd           d30, d18, d19, d20
        fmadd           d31, d21, d22, d23

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

check_correct_result_BM_71:

        // compare local and golden signature
        cmp             x29, x25
        b.ne            error

        // Decrement LUT counter to end by 16 elements
        sub             x1, x1, A53_STL_LUT_CNT_DECR_16

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x1, A53_STL_BM71_LUT_FINISH
        bne             loop_FMADD_v


//-----------------------------------------------------------
// END BASIC MODULE 71
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 84
//-----------------------------------------------------------

// Basic Module 84
// PMUL <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM84_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM84_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM84_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM84_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute pmul instruction

        pmul            v17.16b, v1.16b, v5.16b
        pmul            v18.16b, v3.16b, v7.16b

        pmul            v19.16b, v2.16b, v6.16b
        pmul            v20.16b, v4.16b, v8.16b

        // Copy v17 in x11 and x15 for signature check
        fmov            x11, d17
        fmov            x15, v17.d[1]
        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d18
        fmov            x16, v18.d[1]

        // Copy v19 in x13 and x17 for signature check
        fmov            x13, d19
        fmov            x17, v19.d[1]

        // Copy v20 in x14 and x18 for signature check
        fmov            x14, d20
        fmov            x18, v20.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM84_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM84_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

check_correct_result_BM_84:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 84
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 85
//-----------------------------------------------------------

// Basic Module 85
// CMTST <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM85_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM85_INPUT_VALUE_2
        ldr             x2, =A53_STL_BM85_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM85_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute cmtst instruction

        cmtst           v17.16b, v1.16b, v5.16b
        cmtst           v18.16b, v3.16b, v7.16b

        cmtst           v19.16b, v2.16b, v6.16b
        cmtst           v20.16b, v4.16b, v8.16b

        // Copy v17 in x11 and x15 for signature check
        fmov            x11, d17
        fmov            x15, v17.d[1]
        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d18
        fmov            x16, v18.d[1]

        // Copy v19 in x13 adn x17 for signature check
        fmov            x13, d19
        fmov            x17, v19.d[1]

        // Copy v20 in x14 and x18 for signature check
        fmov            x14, d20
        fmov            x18, v20.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM85_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error


        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM85_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

check_correct_result_BM_85:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 85
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

a53_stl_core_p029_n001_end:

        ret

        .end
