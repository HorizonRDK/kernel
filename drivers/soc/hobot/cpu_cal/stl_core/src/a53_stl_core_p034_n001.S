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
//      Testing of multiplication instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: None
//
//   About: TEST_ID
//      CORE_034
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p034_n001
//      Testing of multiplication instructions
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

        .section .section_a53_stl_core_p034_n001,"ax",%progbits
        .global a53_stl_core_p034_n001
        .type a53_stl_core_p034_n001, %function

a53_stl_core_p034_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 227
//-----------------------------------------------------------

// Basic Module 227
// MUL <Xd>, <Xn>, <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM227_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM227_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM227_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM227_INPUT_VALUE_1
        ldr             x6, =A53_STL_BM227_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM227_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM227_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM227_INPUT_VALUE_3
        ldr             x10, =A53_STL_BM227_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM227_INPUT_VALUE_6
        ldr             x12, =A53_STL_BM227_INPUT_VALUE_6
        ldr             x13, =A53_STL_BM227_INPUT_VALUE_5
        ldr             x14, =A53_STL_BM227_INPUT_VALUE_7
        ldr             x15, =A53_STL_BM227_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM227_INPUT_VALUE_8
        ldr             x17, =A53_STL_BM227_INPUT_VALUE_7

        // 1st test: Result is partial representable
        mul             x18, x2, x3
        mul             x19, x4, x5
        // 2nd test: Result is fully representable
        mul             x20, x6, x7
        mul             x21, x8, x9
        // 3rd test: Result is 0xAAAAAAAAAAAAAAAA
        mul             x22, x10, x11
        mul             x23, x12, x13
        // 4th test: Result is 0x5555555555555555
        mul             x24, x14, x15
        mul             x25, x16, x17

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM227_GOLDEN_VALUE_1

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM227_GOLDEN_VALUE_2

        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM227_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM227_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM227_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error


check_correct_result_BM_227:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 227
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 301
//-----------------------------------------------------------

// Basic Module 301
// MADD <Xd>, <Xn>, <Xm>, <Xa>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        mov             x28, A53_STL_BM301_INPUT_VALUE_1
        mov             x29, A53_STL_BM301_INPUT_VALUE_2

        // Set operand register

        mov             x0, A53_STL_BM301_INPUT_VALUE_3
        mov             x1, A53_STL_BM301_INPUT_VALUE_4
        mov             x2, A53_STL_BM301_INPUT_VALUE_5
        mov             x3, A53_STL_BM301_INPUT_VALUE_6

        // 1st test with Xa = 0 and Xm = 1

        madd            x28, x28, x1, x0
        madd            x29, x29, x1, x0

        // 2nd test with Xa negative and Xm positive

        madd            x28, x28, x2, x3
        madd            x29, x29, x2, x3

        // 3rd test with Xa positive and Xm negative

        madd            x28, x28, x3, x2
        madd            x29, x29, x3, x2

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM301_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM301_GOLDEN_SIGN_2

check_correct_result_BM_301:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 301
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 302
//-----------------------------------------------------------

// Basic Module 302
// MSUB <Xd>, <Xn>, <Xm>, <Xa>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        mov             x28, A53_STL_BM302_INPUT_VALUE_1
        mov             x29, A53_STL_BM302_INPUT_VALUE_2

        // Set operand register

        mov             x4, A53_STL_BM302_INPUT_VALUE_3
        mov             x5, A53_STL_BM302_INPUT_VALUE_4
        mov             x6, A53_STL_BM302_INPUT_VALUE_5
        mov             x7, A53_STL_BM302_INPUT_VALUE_6
        mov             x8, A53_STL_BM302_INPUT_VALUE_7
        mov             x9, A53_STL_BM302_INPUT_VALUE_8

        // 1st test with Xa = 0 and Xm = 1

        msub            x28, x28, x5, x4
        msub            x29, x29, x5, x4

        // 2nd test with Xa negative and Xm positive

        msub            x28, x28, x6, x7
        msub            x29, x29, x6, x7

        // 3rd test with Xa positive and Xm negative

        msub            x28, x28, x7, x6
        msub            x29, x29, x7, x6

        // MNEG alias Xm positive

        msub            x28, x28, x8, xzr
        msub            x29, x29, x8, xzr

        // MNEG alias Xm negative

        msub            x28, x28, x9, xzr
        msub            x29, x29, x9, xzr

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM302_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM302_GOLDEN_SIGN_2

check_correct_result_BM_302:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 302
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 303
//-----------------------------------------------------------

// Basic Module 303
// SMADDL <Xd>, <Wn>, <Wm>, <Xa>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        ldr             x28, =A53_STL_BM303_INPUT_VALUE_1
        ldr             x29, =A53_STL_BM303_INPUT_VALUE_2

        // Set operand register

        mov             x10, A53_STL_BM303_INPUT_VALUE_7
        mov             x11, A53_STL_BM303_INPUT_VALUE_8
        ldr             x12, =A53_STL_BM303_INPUT_VALUE_3
        ldr             x13, =A53_STL_BM303_INPUT_VALUE_4
        ldr             x14, =A53_STL_BM303_INPUT_VALUE_5
        ldr             x15, =A53_STL_BM303_INPUT_VALUE_6

        // 1st test with Xa = 0 and Xm = 1

        smaddl          x28, w28, w11, x10
        smaddl          x29, w29, w11, x10

        // 2nd test with Xa positive and Wm negative

        smaddl          x28, w28, w12, x13
        smaddl          x29, w29, w12, x13

        // 3rd test with Xa negative and Wm positive

        smaddl          x28, w28, w13, x12
        smaddl          x29, w29, w13, x12

        // SMULL alias Wm negative

        smaddl          x28, w28, w14, xzr
        smaddl          x29, w29, w14, xzr

        // SMULL alias Wm positive

        smaddl          x28, w28, w15, xzr
        smaddl          x29, w29, w15, xzr

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM303_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM303_GOLDEN_SIGN_2

check_correct_result_BM_303:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 303
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 304
//-----------------------------------------------------------

// Basic Module 304
// SMSUBL <Xd>, <Wn>, <Wm>, <Xa>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        ldr             x28, =A53_STL_BM304_INPUT_VALUE_1
        ldr             x29, =A53_STL_BM304_INPUT_VALUE_2

        // Set operand register

        mov             x16, A53_STL_BM304_INPUT_VALUE_7
        mov             x17, A53_STL_BM304_INPUT_VALUE_8
        ldr             x18, =A53_STL_BM304_INPUT_VALUE_3
        ldr             x19, =A53_STL_BM304_INPUT_VALUE_4
        ldr             x20, =A53_STL_BM304_INPUT_VALUE_5
        ldr             x21, =A53_STL_BM304_INPUT_VALUE_6

        // 1st test with Xa = 0 and Xm = 1

        smsubl          x28, w28, w17, x16
        smsubl          x29, w29, w17, x16

        // 2nd test with Xa positive and Wm negative

        smsubl          x28, w28, w18, x19
        smsubl          x29, w29, w18, x19

        // 3rd test with Xa negative and Wm positive

        smsubl          x28, w28, w19, x18
        smsubl          x29, w29, w19, x18

        // SMNEGL alias Wm negative

        smsubl          x28, w28, w20, xzr
        smsubl          x29, w29, w20, xzr

        // SMNEGL alias Wm positive

        smsubl          x28, w28, w21, xzr
        smsubl          x29, w29, w21, xzr

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM304_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM304_GOLDEN_SIGN_2

check_correct_result_BM_304:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 304
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 305
//-----------------------------------------------------------

// Basic Module 305
// SMULH <Xd>, <Xn>, <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        ldr             x28, =A53_STL_BM305_INPUT_VALUE_1
        ldr             x29, =A53_STL_BM305_INPUT_VALUE_2

        // Set operand register

        ldr             x24, =A53_STL_BM305_INPUT_VALUE_3
        ldr             x25, =A53_STL_BM305_INPUT_VALUE_4

        // 1st test with Xm negative

        smulh           x28, x28, x24
        smulh           x29, x29, x24

        // 2nd test with Xm positive

        smulh           x28, x28, x25
        smulh           x29, x29, x25

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM305_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM305_GOLDEN_SIGN_2

check_correct_result_BM_305:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 305
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 306
//-----------------------------------------------------------

// Basic Module 306
// UMADDL <Xd>, <Wn>, <Wm>, <Xa>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        ldr             x28, =A53_STL_BM306_INPUT_VALUE_1
        ldr             x29, =A53_STL_BM306_INPUT_VALUE_2

        // Set operand register

        mov             x4, A53_STL_BM306_INPUT_VALUE_7
        mov             x5, A53_STL_BM306_INPUT_VALUE_8
        ldr             x6, =A53_STL_BM306_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM306_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM306_INPUT_VALUE_5
        ldr             x9, =A53_STL_BM306_INPUT_VALUE_6

        // 1st test with Xa = 0 and Xm = 1

        umaddl          x28, w28, w5, x4
        umaddl          x29, w29, w5, x4

        // 2nd test with Xa negative and Wm positive

        umaddl          x28, w28, w6, x7
        umaddl          x29, w29, w6, x7

        // 3rd test with Xa positive and Wm negative

        umaddl          x28, w28, w7, x6
        umaddl          x29, w29, w7, x6

        // UMULL alias Wm negative

        umaddl          x28, w28, w8, xzr
        umaddl          x29, w29, w8, xzr

        // UMULL alias Wm positive

        umaddl          x28, w28, w9, xzr
        umaddl          x29, w29, w9, xzr

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM306_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM306_GOLDEN_SIGN_2

check_correct_result_BM_306:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 306
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 307
//-----------------------------------------------------------

// Basic Module 307
// UMSUBL <Xd>, <Wn>, <Wm>, <Xa>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        ldr             x28, =A53_STL_BM307_INPUT_VALUE_1
        ldr             x29, =A53_STL_BM307_INPUT_VALUE_2

        // Set operand register

        mov             x10, A53_STL_BM307_INPUT_VALUE_7
        mov             x11, A53_STL_BM307_INPUT_VALUE_8
        ldr             x12, =A53_STL_BM307_INPUT_VALUE_3
        ldr             x13, =A53_STL_BM307_INPUT_VALUE_4
        ldr             x14, =A53_STL_BM307_INPUT_VALUE_5
        ldr             x15, =A53_STL_BM307_INPUT_VALUE_6

        // 1st test with Xa = 0 and Xm = 1

        umsubl          x28, w28, w11, x10
        umsubl          x29, w29, w11, x10

        // 2nd test with Xa negative and Wm positive

        umsubl          x28, w28, w12, x13
        umsubl          x29, w29, w12, x13

        // 3rd test with Xa positive and Wm negative

        umsubl          x28, w28, w13, x12
        umsubl          x29, w29, w13, x12

        // UMNEGL alias Wm positive

        umsubl          x28, w28, w14, xzr
        umsubl          x29, w29, w14, xzr

        // UMNEGL alias Wm negative

        umsubl          x28, w28, w15, xzr
        umsubl          x29, w29, w15, xzr

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM307_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM307_GOLDEN_SIGN_2

check_correct_result_BM_307:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 307
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 308
//-----------------------------------------------------------

// Basic Module 308
// UMULH <Xd>, <Xn>, <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        ldr             x28, =A53_STL_BM308_INPUT_VALUE_1
        ldr             x29, =A53_STL_BM308_INPUT_VALUE_2

        // Set operand register

        ldr             x24, =A53_STL_BM308_INPUT_VALUE_3
        ldr             x25, =A53_STL_BM308_INPUT_VALUE_4

        // 1st test with Xm negatve

        umulh           x28, x28, x24
        umulh           x29, x29, x24

        // 2nd test with Xm positive

        umulh           x28, x28, x25
        umulh           x29, x29, x25

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM308_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM308_GOLDEN_SIGN_2

check_correct_result_BM_308:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 308
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

a53_stl_core_p034_n001_end:

        ret

        .end
