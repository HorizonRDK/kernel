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
//      Testing of LDR/STR instructions with MMU enabled at EL3
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_MEM
//
//   About: TEST_ID
//      CORE_041
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p041_n001
//      Testing of LDR/STR instructions
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

        .section .section_a53_stl_core_p041_n001,"ax",%progbits
        .global a53_stl_core_p041_n001
        .type a53_stl_core_p041_n001, %function

a53_stl_core_p041_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine
        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

        // Initialize x30 offset for stack pointer recovery in case of error
        mov             x30, A53_STL_STACK_OFFSET_0

//-----------------------------------------------------------
// START BASIC MODULE 69
//-----------------------------------------------------------

// Basic Module 69
// LDRB <Wt>, [<Xn|SP>, <Xm>{, LSL <amount>}] and STRB <Wt>, [<Xn|SP>, <Xm>{, LSL <amount>}]

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Decrement SP by 8*8byte
        sub             sp, sp, A53_STL_BM69_STK_PTR_64_B

        // Save offset in x30
        mov             x30, A53_STL_BM69_STK_PTR_64_B

        ldr             x0, =A53_STL_BM69_NUM_LDR

        ldr             x11, =A53_STL_BM69_INDEX_0
        ldr             x12, =A53_STL_BM69_INDEX_1
        ldr             x13, =A53_STL_BM69_INDEX_2
        ldr             x14, =A53_STL_BM69_INDEX_3
        ldr             x15, =A53_STL_BM69_INDEX_4
        ldr             x16, =A53_STL_BM69_INDEX_5
        ldr             x17, =A53_STL_BM69_INDEX_6
        ldr             x18, =A53_STL_BM69_INDEX_7

        ldr             w1, =A53_STL_BM69_INPUT_BYTE_1
        ldr             w2, =A53_STL_BM69_INPUT_BYTE_2
        ldr             w3, =A53_STL_BM69_INPUT_BYTE_3
        ldr             w4, =A53_STL_BM69_INPUT_BYTE_4
        ldr             w5, =A53_STL_BM69_INPUT_BYTE_5
        ldr             w6, =A53_STL_BM69_INPUT_BYTE_6
        ldr             w7, =A53_STL_BM69_INPUT_BYTE_7
        ldr             w8, =A53_STL_BM69_INPUT_BYTE_8

loop_store:
        // store a value into the stack
        strb            w1, [sp, x11]
        strb            w2, [sp, x12]
        strb            w3, [sp, x13]
        strb            w4, [sp, x14]
        strb            w5, [sp, x15]
        strb            w6, [sp, x16]
        strb            w7, [sp, x17]
        strb            w8, [sp, x18]

        add             x11, x11, A53_STL_BM69_INCR_8
        add             x12, x12, A53_STL_BM69_INCR_8
        add             x13, x13, A53_STL_BM69_INCR_8
        add             x14, x14, A53_STL_BM69_INCR_8
        add             x15, x15, A53_STL_BM69_INCR_8
        add             x16, x16, A53_STL_BM69_INCR_8
        add             x17, x17, A53_STL_BM69_INCR_8
        add             x18, x18, A53_STL_BM69_INCR_8

        // Decrement LUT counter to end by 12 elements
        sub             x0, x0, A53_STL_BM69_DECR_LUT_12

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x0, A53_STL_BM69_IMM_OFF_0
        bne             loop_store

        ldr             x0, =A53_STL_BM69_NUM_LDR

        ldr             x11, =A53_STL_BM69_INDEX_0
        ldr             x12, =A53_STL_BM69_INDEX_1
        ldr             x13, =A53_STL_BM69_INDEX_2
        ldr             x14, =A53_STL_BM69_INDEX_3
        ldr             x15, =A53_STL_BM69_INDEX_4
        ldr             x16, =A53_STL_BM69_INDEX_5
        ldr             x17, =A53_STL_BM69_INDEX_6
        ldr             x18, =A53_STL_BM69_INDEX_7

loop_load:

        ldrb            w21, [sp, x11]
        ldrb            w22, [sp, x12]
        ldrb            w23, [sp, x13]
        ldrb            w24, [sp, x14]
        ldrb            w25, [sp, x15]
        ldrb            w26, [sp, x16]
        ldrb            w27, [sp, x17]
        ldrb            w28, [sp, x18]

        add             x11, x11, A53_STL_BM69_INCR_8
        add             x12, x12, A53_STL_BM69_INCR_8
        add             x13, x13, A53_STL_BM69_INCR_8
        add             x14, x14, A53_STL_BM69_INCR_8
        add             x15, x15, A53_STL_BM69_INCR_8
        add             x16, x16, A53_STL_BM69_INCR_8
        add             x17, x17, A53_STL_BM69_INCR_8
        add             x18, x18, A53_STL_BM69_INCR_8

        // Compare local and golden signature
        cmp             w21, A53_STL_BM69_GOLDEN_VALUE_1
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w22, A53_STL_BM69_GOLDEN_VALUE_2
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w23, A53_STL_BM69_GOLDEN_VALUE_3
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w24, A53_STL_BM69_GOLDEN_VALUE_4
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w25, A53_STL_BM69_GOLDEN_VALUE_5
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w26, A53_STL_BM69_GOLDEN_VALUE_6
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w27, A53_STL_BM69_GOLDEN_VALUE_7
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w28, A53_STL_BM69_GOLDEN_VALUE_8
        b.ne            error_BM_69

        // Decrement LUT counter to end by 12 elements
        sub             x0, x0, A53_STL_BM69_DECR_LUT_12

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x0, A53_STL_BM69_IMM_OFF_0
        bne             loop_load

        ldr             x0, =A53_STL_BM69_NUM_LDR

        ldr             x11, =A53_STL_BM69_INDEX_0
        ldr             x12, =A53_STL_BM69_INDEX_1
        ldr             x13, =A53_STL_BM69_INDEX_2
        ldr             x14, =A53_STL_BM69_INDEX_3
        ldr             x15, =A53_STL_BM69_INDEX_4
        ldr             x16, =A53_STL_BM69_INDEX_5
        ldr             x17, =A53_STL_BM69_INDEX_6
        ldr             x18, =A53_STL_BM69_INDEX_7

        ldr             w1, =A53_STL_BM69_INPUT_BYTE_2
        ldr             w2, =A53_STL_BM69_INPUT_BYTE_1
        ldr             w3, =A53_STL_BM69_INPUT_BYTE_4
        ldr             w4, =A53_STL_BM69_INPUT_BYTE_3
        ldr             w5, =A53_STL_BM69_INPUT_BYTE_6
        ldr             w6, =A53_STL_BM69_INPUT_BYTE_5
        ldr             w7, =A53_STL_BM69_INPUT_BYTE_8
        ldr             w8, =A53_STL_BM69_INPUT_BYTE_7

loop_store_rev:
        // store a value into the stack
        strb            w1, [sp, x11]
        strb            w2, [sp, x12]
        strb            w3, [sp, x13]
        strb            w4, [sp, x14]
        strb            w5, [sp, x15]
        strb            w6, [sp, x16]
        strb            w7, [sp, x17]
        strb            w8, [sp, x18]

        add             x11, x11, A53_STL_BM69_INCR_8
        add             x12, x12, A53_STL_BM69_INCR_8
        add             x13, x13, A53_STL_BM69_INCR_8
        add             x14, x14, A53_STL_BM69_INCR_8
        add             x15, x15, A53_STL_BM69_INCR_8
        add             x16, x16, A53_STL_BM69_INCR_8
        add             x17, x17, A53_STL_BM69_INCR_8
        add             x18, x18, A53_STL_BM69_INCR_8

        // Decrement LUT counter to end by 12 elements
        sub             x0, x0, A53_STL_BM69_DECR_LUT_12

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x0, A53_STL_BM69_IMM_OFF_0
        bne             loop_store_rev

        ldr             x0, =A53_STL_BM69_NUM_LDR

        ldr             x11, =A53_STL_BM69_INDEX_0
        ldr             x12, =A53_STL_BM69_INDEX_1
        ldr             x13, =A53_STL_BM69_INDEX_2
        ldr             x14, =A53_STL_BM69_INDEX_3
        ldr             x15, =A53_STL_BM69_INDEX_4
        ldr             x16, =A53_STL_BM69_INDEX_5
        ldr             x17, =A53_STL_BM69_INDEX_6
        ldr             x18, =A53_STL_BM69_INDEX_7

loop_load_rev:

        ldrb            w21, [sp, x11]
        ldrb            w22, [sp, x12]
        ldrb            w23, [sp, x13]
        ldrb            w24, [sp, x14]
        ldrb            w25, [sp, x15]
        ldrb            w26, [sp, x16]
        ldrb            w27, [sp, x17]
        ldrb            w28, [sp, x18]

        add             x11, x11, A53_STL_BM69_INCR_8
        add             x12, x12, A53_STL_BM69_INCR_8
        add             x13, x13, A53_STL_BM69_INCR_8
        add             x14, x14, A53_STL_BM69_INCR_8
        add             x15, x15, A53_STL_BM69_INCR_8
        add             x16, x16, A53_STL_BM69_INCR_8
        add             x17, x17, A53_STL_BM69_INCR_8
        add             x18, x18, A53_STL_BM69_INCR_8

        // Compare local and golden signature
        cmp             w21, A53_STL_BM69_GOLDEN_VALUE_2
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w22, A53_STL_BM69_GOLDEN_VALUE_1
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w23, A53_STL_BM69_GOLDEN_VALUE_4
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w24, A53_STL_BM69_GOLDEN_VALUE_3
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w25, A53_STL_BM69_GOLDEN_VALUE_6
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w26, A53_STL_BM69_GOLDEN_VALUE_5
        b.ne            error_BM_69

        // Compare local and golden signature
        cmp             w27, A53_STL_BM69_GOLDEN_VALUE_8
        b.ne            error_BM_69

check_correct_result_BM_69:
        // Compare local and golden signature
        cmp             w28, A53_STL_BM69_GOLDEN_VALUE_7
        b.ne            error_BM_69

        // Decrement LUT counter to end by 12 elements
        sub             x0, x0, A53_STL_BM69_DECR_LUT_12

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x0, A53_STL_BM69_IMM_OFF_0
        bne             loop_load_rev
        b               success_BM_69

error_BM_69:
        // Increment SP
        add             sp, sp, A53_STL_BM69_STK_PTR_64_B
        b               error

success_BM_69:
        // Increment SP
        add             sp, sp, A53_STL_BM69_STK_PTR_64_B

//-----------------------------------------------------------
// END BASIC MODULE 69
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

a53_stl_core_p041_n001_end:

        ret

        .end
