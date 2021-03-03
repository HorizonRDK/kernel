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
//      Testing of GPR registers
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_FPU
//
//   About: TEST_ID
//      CORE_023
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p023_n001
//      Testing of GPR registers
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

        .section .section_a53_stl_core_p023_n001,"ax",%progbits
        .global a53_stl_core_p023_n001
        .type a53_stl_core_p023_n001, %function

a53_stl_core_p023_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping


//-----------------------------------------------------------
// START BASIC MODULE 139
//-----------------------------------------------------------

// Basic Module 139
// FPU and SIMD registers test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Get input test value in x0

        ldr             x0, =A53_STL_BM139_INPUT_VALUE_1

        // Set all Dx and Vx registers with test value
        bl              fmov_d0d31_x0

        bl              fmov_v0v31_x0

        // Get golden value in x0
        ldr             x0, =A53_STL_BM139_GOLDEN_VALUE_1

        // Read back values from Dx registers
        bl              fmov_x1x29_d0d28

        // Check results on Dx registers

        // compare register value with golden value
        bl              cmp_x1x29_x0
        ldr             x1, =A53_STL_BM139_GOLDEN_VALUE_1
        cmp             x1, x0
        b.ne            error

        // Read back values from Dx registers

        fmov            x1, d29
        fmov            x2, d30
        fmov            x3, d31

        // Check results on Dx registers

        // compare register value with golden value
        cmp             x1, x0
        b.ne            error

        // compare register value with golden value
        cmp             x2, x0
        b.ne            error

        // compare register value with golden value
        cmp             x3, x0
        b.ne            error

        // Read back values from Vx registers
        bl              fmov_x1x29_v0v28

        // Check results on Vx registers

        // compare register value with golden value
        bl              cmp_x1x29_x0
        ldr             x1, =A53_STL_BM139_GOLDEN_VALUE_1
        cmp             x1, x0
        b.ne            error

        // Read back values from Vx registers

        fmov            x1, v29.d[1]
        fmov            x2, v30.d[1]
        fmov            x3, v31.d[1]

        // Check results on Vx registers

        // compare register value with golden value
        cmp             x1, x0
        b.ne            error

        // compare register value with golden value
        cmp             x2, x0
        b.ne            error

        // compare register value with golden value
        cmp             x3, x0
        b.ne            error

        // Get input test value in x0

        ldr             x0, =A53_STL_BM139_INPUT_VALUE_2

        // Set all Dx and Vx registers with test value
        bl              fmov_d0d31_x0

        bl              fmov_v0v31_x0

        // Get golden value in x0
        ldr             x0, =A53_STL_BM139_GOLDEN_VALUE_2

        // Read back values from Dx registers
        bl              fmov_x1x29_d0d28

        // Check results on Dx registers
        // compare register value with golden value
        bl              cmp_x1x29_x0
        ldr             x1, =A53_STL_BM139_GOLDEN_VALUE_2
        cmp             x1, x0
        b.ne            error

        // Read back values from Dx registers

        fmov            x1, d29
        fmov            x2, d30
        fmov            x3, d31

        // Check results on Dx registers

        // compare register value with golden value
        cmp             x1, x0
        b.ne            error

        // compare register value with golden value
        cmp             x2, x0
        b.ne            error

        // compare register value with golden value
        cmp             x3, x0
        b.ne            error

        // Read back values from Vx registers
        bl              fmov_x1x29_v0v28

        // Check results on Vx registers

        // compare register value with golden value
        bl              cmp_x1x29_x0
        ldr             x1, =A53_STL_BM139_GOLDEN_VALUE_2
        cmp             x1, x0
        b.ne            error

        // Read back values from Vx registers

        fmov            x1, v29.d[1]
        fmov            x2, v30.d[1]
        fmov            x3, v31.d[1]

        // Check results on Vx registers

        // compare register value with golden value
        cmp             x1, x0
        b.ne            error

        // compare register value with golden value
        cmp             x2, x0
        b.ne            error

check_correct_result_BM_139:
        // compare register value with golden value
        cmp             x3, x0
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 139
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

a53_stl_core_p023_n001_end:

        ret

        .end
