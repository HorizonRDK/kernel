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
//      Testing of load and store instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_MEM
//
//   About: TEST_ID
//      CORE_013
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p013_n001
//      Testing of load and store instructions
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

        .section .section_a53_stl_core_p013_n001,"ax",%progbits
        .global a53_stl_core_p013_n001
        .type a53_stl_core_p013_n001, %function

a53_stl_BM239_test_label:
        .word 0x00000000
        .word 0x00000002

a53_stl_core_p013_n001:

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
// START BASIC MODULE 65
//-----------------------------------------------------------

// Basic Module 65
// LDR <Xt>, [<Xn|SP>], #<simm> and STR <Xt>, [<Xn|SP>], #<simm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Initialize  X1,X2,X3,x4,x5 registers with the test values
        mov             x1, A53_STL_BM65_INPUT_VALUE_1
        mov             x2, A53_STL_BM65_INPUT_VALUE_2
        mov             x3, A53_STL_BM65_INPUT_VALUE_3
        mov             x4, A53_STL_BM65_INPUT_VALUE_4
        mov             x5, A53_STL_BM65_INPUT_VALUE_5

        // Decrement SP of 40 bytes (5*8bytes) to store registers into the stack
        sub             sp, sp, A53_STL_BM65_STK_PTR_40_B

        // Save offset in x30
        mov             x30, A53_STL_BM65_STK_PTR_40_B

        // Store of the test values from registers to SP
        str             x1, [sp, A53_STL_BM65_STR_OFFSET_1]
        str             x2, [sp, A53_STL_BM65_STR_OFFSET_2]
        str             x3, [sp, A53_STL_BM65_STR_OFFSET_3]
        str             x4, [sp, A53_STL_BM65_STR_OFFSET_4]
        str             x5, [sp, A53_STL_BM65_STR_OFFSET_5]

        // Load the value from the stack offset 0x0
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_1]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_1

        // check if the loaded value is the expected value in SP offset 0x0
        cmp             x0, x28
        b.ne            error_BM_65

        // Load the value from the stack offset 0x8
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_2]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_2

        // check if the loaded value is the expected value in SP offset 0x8
        cmp             x0, x28
        b.ne            error_BM_65

        // Load the value from the stack offset 0x10
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_3]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_3

        // check if the loaded value is the expected value in SP offset 0x10
        cmp             x0, x28
        b.ne            error_BM_65

        // Load the value from the stack offset 0x18
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_4]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_4

        // check if the loaded value is the expected value in SP offset 0x18
        cmp             x0, x28
        b.ne            error_BM_65

        // Load the value from the stack offset 0x20
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_5]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_5

        // check if the loaded value is the expected value in SP offset 0x20
        cmp             x0, x28
        b.ne            error_BM_65

        // Testing complementary input values

        // Initialize  X1,X2,X3,x4,x5 registers with the test values
        mov             x1, A53_STL_BM65_INPUT_VALUE_2
        mov             x2, A53_STL_BM65_INPUT_VALUE_1
        mov             x3, A53_STL_BM65_INPUT_VALUE_6
        mov             x4, A53_STL_BM65_INPUT_VALUE_5
        mov             x5, A53_STL_BM65_INPUT_VALUE_4

        // Store of the test values from registers to SP
        str             x1, [sp, A53_STL_BM65_STR_OFFSET_1]
        str             x2, [sp, A53_STL_BM65_STR_OFFSET_2]
        str             x3, [sp, A53_STL_BM65_STR_OFFSET_3]
        str             x4, [sp, A53_STL_BM65_STR_OFFSET_4]
        str             x5, [sp, A53_STL_BM65_STR_OFFSET_5]

        // Load the value from the stack offset 0x0
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_1]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_2

        // check if the loaded value is the expected value in SP offset 0x0
        cmp             x0, x28
        b.ne            error_BM_65

        // Load the value from the stack offset 0x8
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_2]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_1

        // check if the loaded value is the expected value in SP offset 0x8
        cmp             x0, x28
        b.ne            error_BM_65

        // Load the value from the stack offset 0x10
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_3]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_6

        // check if the loaded value is the expected value in SP offset 0x10
        cmp             x0, x28
        b.ne            error_BM_65

        // Load the value from the stack offset 0x18
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_4]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_5

        // check if the loaded value is the expected value in SP offset 0x18
        cmp             x0, x28
        b.ne            error_BM_65

        // Load the value from the stack offset 0x20
        ldr             x0, [sp, A53_STL_BM65_STR_OFFSET_5]
        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM65_GOLDEN_VALUE_4

check_correct_result_BM_65:
        // check if the loaded value is the expected value in SP offset 0x20
        cmp             x0, x28
        b.ne            error_BM_65
        b               success_BM_65

error_BM_65:
        // Increment SP of 40 bytes (5*8bytes)
        add             sp, sp, A53_STL_BM65_STK_PTR_40_B
        b               error

success_BM_65:
        // Increment SP of 40 bytes (5*8bytes)
        add             sp, sp, A53_STL_BM65_STK_PTR_40_B

//-----------------------------------------------------------
// END BASIC MODULE 65
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 66
//-----------------------------------------------------------

// Basic Module 66
// LDR <Xt>, [<Xn|SP>, (<Wm>|<Xm>)] and STR <Xt>, [<Xn|SP>, (<Wm>|<Xm>)]

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Initialize  X1,X2,X3,x4 registers with the test values
        mov             x1, A53_STL_BM66_INPUT_VALUE_1
        mov             x2, A53_STL_BM66_INPUT_VALUE_2

        mov             x3, A53_STL_BM66_STR_OFFSET_1
        mov             x4, A53_STL_BM66_STR_OFFSET_2

        // Decrement SP of 56 bytes (7*8bytes) to store registers into the stack
        sub             sp, sp, A53_STL_BM66_STK_PTR_56_B

        // Save offset in x30
        mov             x30, A53_STL_BM66_STK_PTR_56_B

        // Store of the test values from registers to SP
        str             x1, [sp, x3]
        str             x2, [sp, x4]

        // Load the value from the stack offset 0x28
        ldr             x0, [sp, x3]
        ldr             x28, =A53_STL_BM66_GOLDEN_VALUE_1

        // check if the loaded value is the expected value in SP offset 0x28
        cmp             x0, x28
        b.ne            error_BM_66

        // Load the value from the stack offset 0x30
        ldr             x0, [sp, x4]
        ldr             x28, =A53_STL_BM66_GOLDEN_VALUE_2

check_correct_result_BM_66:
        // check if the loaded value is the expected value in SP offset 0x30
        cmp             x0, x28
        b.ne            error_BM_66
        b               success_BM_66

error_BM_66:
        // Increment SP of 56 bytes (7*8bytes)
        add             sp, sp, A53_STL_BM66_STK_PTR_56_B
        b               error

success_BM_66:
        // Increment SP of 56 bytes (7*8bytes)
        add             sp, sp, A53_STL_BM66_STK_PTR_56_B

//-----------------------------------------------------------
// END BASIC MODULE 66
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 293
//-----------------------------------------------------------

// Basic Module 293
// LDR <Xt>, <label>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Load the value from PC plus label
        ldr             x0, =a53_stl_BM239_test_label
        adr             x1, a53_stl_BM239_test_label

check_correct_result_BM_293:
        // check if the loaded value is the expected value in x1
        cmp             x0, x1
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 293
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 80
//-----------------------------------------------------------

// Basic Module 80
// LDRB <Wt>, [<Xn|SP>], #<simm> and STRB <Wt>, [<Xn|SP>], #<simm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Initialize w1 and w2 registers with the test values
        mov             w1, A53_STL_BM80_INPUT_VALUE_1
        mov             w2, A53_STL_BM80_INPUT_VALUE_2

        // Decrement SP of 72 bytes (9*8bytes) to store registers into the stack
        sub             sp, sp, A53_STL_BM80_STK_PTR_72_B

        // Save offset in x30
        mov             x30, A53_STL_BM80_STK_PTR_72_B

        // Store of the test values from registers to SP
        strb            w1, [sp, A53_STL_BM80_STR_OFFSET_1]
        strb            w2, [sp, A53_STL_BM80_STR_OFFSET_2]

        // Load the value from the stack offset 0x38
        ldrb            w0, [sp, A53_STL_BM80_STR_OFFSET_1]

        // Load in x28 the golden value
        mov             w28, A53_STL_BM80_GOLDEN_VALUE_1

        // check if the loaded value is the expected value in SP offset 0x38
        cmp             w0, w28
        b.ne            error_BM_80

        // Load the value from the stack offset 0x40
        ldrb            w0, [sp, A53_STL_BM80_STR_OFFSET_2]
        // Load in x28 the golden value
        mov             w28, A53_STL_BM80_GOLDEN_VALUE_2

check_correct_result_BM_80:
        // check if the loaded value is the expected value in SP offset 0x40
        cmp             w0, w28
        b.ne            error_BM_80
        b               success_BM_80

error_BM_80:
        // Increment SP of 72 bytes (9*8bytes)
        add             sp, sp, A53_STL_BM80_STK_PTR_72_B
        b               error

success_BM_80:
        // Increment SP of 72 bytes (9*8bytes)
        add             sp, sp, A53_STL_BM80_STK_PTR_72_B

//-----------------------------------------------------------
// END BASIC MODULE 80
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


a53_stl_core_p013_n001_end:

        ret

        .end
