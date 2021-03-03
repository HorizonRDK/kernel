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
//      Testing of ADR, ADRS, bitfield and extract instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_042
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p042_n001
//      Testing of ADR, ADRS, bitfield and extract instructions
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

        .section .section_a53_stl_core_p042_n001,"ax",%progbits
        .global a53_stl_core_p042_n001
        .type a53_stl_core_p042_n001, %function

a53_stl_core_p042_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping


//-----------------------------------------------------------
// START BASIC MODULE 17
//-----------------------------------------------------------

// Basic Module 17
// SBFM <Xd>, <Xn>, #<immr>, #<imms>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM17_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM17_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM17_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM17_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM17_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM17_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM17_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM17_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM17_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM17_INPUT_VALUE_5
        ldr             x12, =A53_STL_BM17_INPUT_VALUE_6
        ldr             x13, =A53_STL_BM17_INPUT_VALUE_6
        mov             x14, A53_STL_BM17_IMM_VALUE_1
        mov             x15, A53_STL_BM17_IMM_VALUE_1
        mov             x16, A53_STL_BM17_IMM_VALUE_1
        mov             x17, A53_STL_BM17_IMM_VALUE_1
        mov             x18, A53_STL_BM17_IMM_VALUE_1
        mov             x19, A53_STL_BM17_IMM_VALUE_1
        mov             x20, A53_STL_BM17_IMM_VALUE_1
        mov             x21, A53_STL_BM17_IMM_VALUE_1
        mov             x22, A53_STL_BM17_IMM_VALUE_1
        mov             x23, A53_STL_BM17_IMM_VALUE_1
        mov             x24, A53_STL_BM17_IMM_VALUE_1
        mov             x25, A53_STL_BM17_IMM_VALUE_1

        // 1st test: SBFX alias

        sbfm            x14, x2, A53_STL_BM17_IMM_VALUE_2, A53_STL_BM17_IMM_VALUE_4
        sbfm            x15, x3, A53_STL_BM17_IMM_VALUE_2, A53_STL_BM17_IMM_VALUE_4

        // 2nd test: SBFIZ alias

        sbfm            x16, x4, A53_STL_BM17_IMM_VALUE_3, A53_STL_BM17_IMM_VALUE_2
        sbfm            x17, x5, A53_STL_BM17_IMM_VALUE_3, A53_STL_BM17_IMM_VALUE_2

        // 3rd test: SXTB alias

        sbfm            x18, x6, A53_STL_BM17_IMM_VALUE_1, A53_STL_BM17_IMM_VALUE_4
        sbfm            x19, x7, A53_STL_BM17_IMM_VALUE_1, A53_STL_BM17_IMM_VALUE_4

        // 4th test: SXTH alias

        sbfm            x20, x8, A53_STL_BM17_IMM_VALUE_1, A53_STL_BM17_IMM_VALUE_5
        sbfm            x21, x9, A53_STL_BM17_IMM_VALUE_1, A53_STL_BM17_IMM_VALUE_5

        // 5th test: SXTW alias

        sbfm            x22, x10, A53_STL_BM17_IMM_VALUE_1, A53_STL_BM17_IMM_VALUE_6
        sbfm            x23, x11, A53_STL_BM17_IMM_VALUE_1, A53_STL_BM17_IMM_VALUE_6

        // 6th test: ASR (immediate) alias

        sbfm            x24, x12, A53_STL_BM17_IMM_VALUE_7, A53_STL_BM17_IMM_VALUE_8
        sbfm            x25, x13, A53_STL_BM17_IMM_VALUE_7, A53_STL_BM17_IMM_VALUE_8

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM17_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x26, x14
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x15
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM17_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x27, x16
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x17
        b.ne            error

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM17_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x28, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x28, x19
        b.ne            error

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM17_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x29, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x29, x21
        b.ne            error

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM17_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x28, x22
        b.ne            error

        // Compare local and golden signature
        cmp             x28, x23
        b.ne            error

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM17_GOLDEN_VALUE_6

        // Compare local and golden signature
        cmp             x29, x24
        b.ne            error

check_correct_result_BM_17:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 17
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 18
//-----------------------------------------------------------

// Basic Module 18
// UBFM <Xd>, <Xn>, #<immr>, #<imms>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Set operand register

        ldr             x2, =A53_STL_BM18_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM18_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM18_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM18_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM18_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM18_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM18_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM18_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM18_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM18_INPUT_VALUE_5
        ldr             x12, =A53_STL_BM18_INPUT_VALUE_6
        ldr             x13, =A53_STL_BM18_INPUT_VALUE_6
        mov             x14, A53_STL_BM18_IMM_VALUE_1
        mov             x15, A53_STL_BM18_IMM_VALUE_1
        mov             x16, A53_STL_BM18_IMM_VALUE_1
        mov             x17, A53_STL_BM18_IMM_VALUE_1
        mov             x18, A53_STL_BM18_IMM_VALUE_1
        mov             x19, A53_STL_BM18_IMM_VALUE_1
        mov             x20, A53_STL_BM18_IMM_VALUE_1
        mov             x21, A53_STL_BM18_IMM_VALUE_1
        mov             x22, A53_STL_BM18_IMM_VALUE_1
        mov             x23, A53_STL_BM18_IMM_VALUE_1
        mov             x24, A53_STL_BM18_IMM_VALUE_1
        mov             x25, A53_STL_BM18_IMM_VALUE_1

        // 1st test: UXTB alias

        ubfm            w14, w2, A53_STL_BM18_IMM_VALUE_1, A53_STL_BM18_IMM_VALUE_2
        ubfm            w15, w3, A53_STL_BM18_IMM_VALUE_1, A53_STL_BM18_IMM_VALUE_2

        // 2nd test: UXTH alias

        ubfm            w16, w4, A53_STL_BM18_IMM_VALUE_1, A53_STL_BM18_IMM_VALUE_3
        ubfm            w17, w5, A53_STL_BM18_IMM_VALUE_1, A53_STL_BM18_IMM_VALUE_3

        // 3rd test: LSL (immediate) alias

        ubfm            x18, x6, A53_STL_BM18_IMM_VALUE_4, A53_STL_BM18_IMM_VALUE_5
        ubfm            x19, x7, A53_STL_BM18_IMM_VALUE_4, A53_STL_BM18_IMM_VALUE_5

        // 4th test: LSR (immediate) alias

        ubfm            x20, x8, A53_STL_BM18_IMM_VALUE_6, A53_STL_BM18_IMM_VALUE_7
        ubfm            x21, x9, A53_STL_BM18_IMM_VALUE_6, A53_STL_BM18_IMM_VALUE_7

        // 5th test: UBFIZ alias

        ubfm            x22, x10, A53_STL_BM18_IMM_VALUE_8, A53_STL_BM18_IMM_VALUE_9
        ubfm            x23, x11, A53_STL_BM18_IMM_VALUE_8, A53_STL_BM18_IMM_VALUE_9

        // 6th test: UBFX alias

        ubfm            x24, x12, A53_STL_BM18_IMM_VALUE_1, A53_STL_BM18_IMM_VALUE_3
        ubfm            x25, x13, A53_STL_BM18_IMM_VALUE_1, A53_STL_BM18_IMM_VALUE_3

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM18_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x26, x14
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x15
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM18_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x27, x16
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x17
        b.ne            error

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM18_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x28, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x28, x19
        b.ne            error

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM18_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x29, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x29, x21
        b.ne            error

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM18_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x28, x22
        b.ne            error

        // Compare local and golden signature
        cmp             x28, x23
        b.ne            error

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM18_GOLDEN_VALUE_6

        // Compare local and golden signature
        cmp             x29, x24
        b.ne            error

check_correct_result_BM_18:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 18
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 19
//-----------------------------------------------------------

// Basic Module 19
// EXTR <Xd>, <Xn>, <Xm>, #<lsb>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM19_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM19_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM19_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM19_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM19_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM19_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM19_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM19_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM19_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM19_INPUT_VALUE_6
        ldr             x12, =A53_STL_BM19_INPUT_VALUE_5
        ldr             x13, =A53_STL_BM19_INPUT_VALUE_6
        ldr             x14, =A53_STL_BM19_INPUT_VALUE_7
        ldr             x15, =A53_STL_BM19_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM19_INPUT_VALUE_7
        ldr             x17, =A53_STL_BM19_INPUT_VALUE_8

        // 1st test

        extr            x18, x2, x3, A53_STL_BM19_IMM_VALUE_1
        extr            x19, x4, x5, A53_STL_BM19_IMM_VALUE_1

        // 2nd test

        extr            x20, x6, x7, A53_STL_BM19_IMM_VALUE_2
        extr            x21, x8, x9, A53_STL_BM19_IMM_VALUE_2

        // 3rd test

        extr            x22, x10, x11, A53_STL_BM19_IMM_VALUE_3
        extr            x23, x12, x13, A53_STL_BM19_IMM_VALUE_3

        // 4th test

        extr            x24, x14, x15, A53_STL_BM19_IMM_VALUE_4
        extr            x25, x16, x17, A53_STL_BM19_IMM_VALUE_4

        // Initialize w26 with golden signature
        ldr             x26, =A53_STL_BM19_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM19_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM19_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM19_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM19_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_19:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 19
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

a53_stl_core_p042_n001_end:

        ret

        .end
