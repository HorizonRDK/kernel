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
//      Testing of LDR/STR (SIMD and FP) instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_FPU, AI_MEM
//
//   About: TEST_ID
//      CORE_026
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p026_n001
//      Testing of LDR/STR (SIMD and FP) instructions
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

        .section .section_a53_stl_core_p026_n001,"ax",%progbits
        .global a53_stl_core_p026_n001
        .type a53_stl_core_p026_n001, %function

a53_stl_core_p026_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 70
//-----------------------------------------------------------

// Basic Module 70
// LDR <Qt>, [<Xn|SP>, (<Wm>|<Xm>){, <extend> {<amount>}}] and STR <Qt>, [<Xn|SP>, (<Wm>|<Xm>){, <extend> {<amount>}}]

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set input configuration values in the X registers
        ldr             x1, =A53_STL_BM70_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM70_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM70_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM70_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM70_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM70_INPUT_VALUE_6

        // Decrement SP of 80 bytes to store registers into the stack
        sub             sp, sp, A53_STL_BM70_STK_PTR_80_B

        // Save offset in x30
        mov             x30, A53_STL_BM70_STK_PTR_80_B

        // Low part of V0,Q0
        fmov            d0, x1
        // High part of V0,Q0
        fmov            v0.d[1], x1
        // Low part of V1,Q1
        fmov            d1, x2
        // High part of V1,Q1
        fmov            v1.d[1], x2
        // Low part of V2,Q2
        fmov            d2, x3
        // High part of V2,Q2
        fmov            v2.d[1], x3
        // Low part of V3,Q3
        fmov            d3, x4
        // High part of V3,Q3
        fmov            v3.d[1], x4
        // Low part of V4,Q4
        fmov            d4, x5
        // High part of V4,Q4
        fmov            v4.d[1], x5

        // store a value into the stack pointer
        str             q0, [sp]
        // store a value into the stack pointer
        str             q1, [sp, A53_STL_BM70_STK_PTR_OFF_10]
        // store a value into the stack pointer
        str             q2, [sp, A53_STL_BM70_STK_PTR_OFF_20]
        // store a value into the stack pointer
        str             q3, [sp, A53_STL_BM70_STK_PTR_OFF_30]
        // store a value into the stack pointer
        str             q4, [sp, A53_STL_BM70_STK_PTR_OFF_40]

        // load the value from the stack pointer
        ldr             q5, [sp]
        // load the value from the stack pointer
        ldr             q6, [sp, A53_STL_BM70_STK_PTR_OFF_10]
        // load the value from the stack pointer
        ldr             q7, [sp, A53_STL_BM70_STK_PTR_OFF_20]
        // load the value from the stack pointer
        ldr             q8, [sp, A53_STL_BM70_STK_PTR_OFF_30]
        // load the value from the stack pointer
        ldr             q9, [sp, A53_STL_BM70_STK_PTR_OFF_40]

        // Check q0 and q5
        cmeq            v10.2D, v0.2D, v5.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70

        // Check q1 and q6
        cmeq            v10.2D, v1.2D, v6.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70

        // Check q2 and q7
        cmeq            v10.2D, v2.2D, v7.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70

        // Check q3 and q8
        cmeq            v10.2D, v3.2D, v8.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70

        // Check q4 and q9
        cmeq            v10.2D, v4.2D, v9.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70

        // Test complementary bits field
        // Low part of V0,Q0
        fmov            d0, x6
        // High part of V0,Q0
        fmov            v0.d[1], x6
        // Low part of V1,Q1
        fmov            d1, x4
        // High part of V1,Q1
        fmov            v1.d[1], x4
        // Low part of V2,Q2
        fmov            d2, x5
        // High part of V2,Q2
        fmov            v2.d[1], x5
        // Low part of V3,Q3
        fmov            d3, x2
        // High part of V3,Q3
        fmov            v3.d[1], x2
        // Low part of V4,Q4
        fmov            d4, x3
        // High part of V4,Q4
        fmov            v4.d[1], x3

        // Store a value into the stack pointer
        str             q0, [sp]
        // Store a value into the stack pointer
        str             q1, [sp, A53_STL_BM70_STK_PTR_OFF_10]
        // Store a value into the stack pointer
        str             q2, [sp, A53_STL_BM70_STK_PTR_OFF_20]
        // Store a value into the stack pointer
        str             q3, [sp, A53_STL_BM70_STK_PTR_OFF_30]
        // Store a value into the stack pointer
        str             q4, [sp, A53_STL_BM70_STK_PTR_OFF_40]

        // Load the value from the stack pointer
        ldr             q5, [sp]
        // Load the value from the stack pointer
        ldr             q6, [sp, A53_STL_BM70_STK_PTR_OFF_10]
        // Load the value from the stack pointer
        ldr             q7, [sp, A53_STL_BM70_STK_PTR_OFF_20]
        // Load the value from the stack pointer
        ldr             q8, [sp, A53_STL_BM70_STK_PTR_OFF_30]
        // Load the value from the stack pointer
        ldr             q9, [sp, A53_STL_BM70_STK_PTR_OFF_40]

        // Check q0 and q5
        cmeq            v10.2D, v0.2D, v5.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70

        // Check q1 and q6
        cmeq            v10.2D, v1.2D, v6.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70

        // Check q2 and q7
        cmeq            v10.2D, v2.2D, v7.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70

        // Check q3 and q8
        cmeq            v10.2D, v3.2D, v8.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70

        // Check q4 and q9
        cmeq            v10.2D, v4.2D, v9.2D
        // Load compare result in q10
        fmov            x10, d10
        fmov            x11, v10.d[1]

check_correct_result_BM_70:
        // Check compare result
        cmp             x10, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        cmp             x11, #A53_STL_BM70_GOLDEN_VALUE_1
        b.ne            error_BM_70
        b               success_BM_70

error_BM_70:
        // Increment SP of 80 bytes
        add             sp, sp, A53_STL_BM70_STK_PTR_80_B
        b               error

success_BM_70:
        // Increment SP of 80 bytes
        add             sp, sp, A53_STL_BM70_STK_PTR_80_B

//-----------------------------------------------------------
// END BASIC MODULE 70
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

a53_stl_core_p026_n001_end:

        ret

        .end
