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
//      CORE_004
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p004_n001
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

        .section .section_a53_stl_core_p004_n001,"ax",%progbits
        .global a53_stl_core_p004_n001
        .type a53_stl_core_p004_n001, %function

a53_stl_core_p004_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping


//-----------------------------------------------------------
// START BASIC MODULE 14
//-----------------------------------------------------------

// Basic Module 14
// ADRP <Xd>, <label>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
a53_stl_BM_14_tc1_label:
        bl              a53_stl_BM_14_tc2_label
a53_stl_BM_14_tc2_label:
        adrp            x0, a53_stl_BM_14_tc1_label
        adrp            x1, a53_stl_BM_14_tc1_label
a53_stl_BM_14_tc3_label:
        adrp            x2, a53_stl_BM_14_tc2_label
        adrp            x3, a53_stl_BM_14_tc2_label
a53_stl_BM_14_tc4_label:
        adrp            x4, a53_stl_BM_14_tc3_label
        adrp            x5, a53_stl_BM_14_tc3_label
a53_stl_BM_14_tc5_label:
        adrp            x6, a53_stl_BM_14_tc4_label
        adrp            x7, a53_stl_BM_14_tc4_label

        // Make results independent from code location
        mov             x8, x30
        and             x8, x8, A53_STL_BM14_MASK_VALUE
        sub             x10, x8, x0
        sub             x11, x8, x1

        and             x0, x0, A53_STL_BM14_MASK_VALUE
        sub             x12, x0, x2
        sub             x13, x0, x3

        and             x2, x2, A53_STL_BM14_MASK_VALUE
        sub             x14, x2, x4
        sub             x15, x2, x5

        and             x4, x4, A53_STL_BM14_MASK_VALUE
        sub             x16, x4, x6
        sub             x17, x4, x7

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM14_GOLDEN_VALUE

        // Compare local and golden signature
        cmp             x26, x10
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x11
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x12
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x13
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x14
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x15
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x16
        b.ne            error

check_correct_result_BM_14:
        // Compare local and golden signature
        cmp             x26, x17
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 14
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 15
//-----------------------------------------------------------

// Basic Module 15
// ADR <Xd>, <label>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

a53_stl_BM_15_tc1_label:
        bl              a53_stl_BM_15_tc2_label
a53_stl_BM_15_tc2_label:
        adr             x10, a53_stl_BM_15_tc1_label
        adr             x11, a53_stl_BM_15_tc1_label
a53_stl_BM_15_tc3_label:
        adr             x12, a53_stl_BM_15_tc2_label
        adr             x13, a53_stl_BM_15_tc2_label
a53_stl_BM_15_tc4_label:
        adr             x14, a53_stl_BM_15_tc3_label
        adr             x15, a53_stl_BM_15_tc3_label
a53_stl_BM_15_tc5_label:
        adr             x16, a53_stl_BM_15_tc4_label
        adr             x17, a53_stl_BM_15_tc4_label

        // Make results independent from code location
        mov             x18, x30
        sub             x20, x18, x10
        sub             x21, x18, x11

        sub             x22, x12, x10
        sub             x23, x13, x10

        sub             x24, x14, x12
        sub             x25, x15, x12

        sub             x26, x16, x14
        sub             x27, x17, x14

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM15_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x28, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x28, x21
        b.ne            error

        // Compare local and golden signature
        cmp             x28, x22
        b.ne            error

        // Compare local and golden signature
        cmp             x28, x23
        b.ne            error

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM15_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x29, x24
        b.ne            error

        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error

        // Compare local and golden signature
        cmp             x29, x26
        b.ne            error

check_correct_result_BM_15:
        // Compare local and golden signature
        cmp             x29, x27
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 15
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 16
//-----------------------------------------------------------

// Basic Module 16
// BFM <Xd>, <Xn>, #<immr>, #<imms>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM16_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM16_INPUT_VALUE_1
        mov             x18, A53_STL_BM16_IMM_VALUE_1
        mov             x19, A53_STL_BM16_IMM_VALUE_1
        ldr             x20, =A53_STL_BM16_INPUT_VALUE_2
        ldr             x21, =A53_STL_BM16_INPUT_VALUE_2
        ldr             x10, =A53_STL_BM16_INPUT_VALUE_3
        ldr             x12, =A53_STL_BM16_INPUT_VALUE_3
        ldr             x14, =A53_STL_BM16_INPUT_VALUE_4
        ldr             x16, =A53_STL_BM16_INPUT_VALUE_4
        mov             x22, A53_STL_BM16_IMM_VALUE_1
        mov             x23, A53_STL_BM16_IMM_VALUE_1
        mov             x24, A53_STL_BM16_IMM_VALUE_1
        mov             x25, A53_STL_BM16_IMM_VALUE_1

        // 1st test: BFXIL alias

        bfm             x18, x2, A53_STL_BM16_IMM_VALUE_1, A53_STL_BM16_IMM_VALUE_2
        bfm             x19, x4, A53_STL_BM16_IMM_VALUE_1, A53_STL_BM16_IMM_VALUE_2

        // 2nd test: BFC alias

        bfm             x20, xzr, A53_STL_BM16_IMM_VALUE_3, A53_STL_BM16_IMM_VALUE_2
        bfm             x21, xzr, A53_STL_BM16_IMM_VALUE_3, A53_STL_BM16_IMM_VALUE_2

        // 3rd test: BFI alias

        bfm             x22, x10, A53_STL_BM16_IMM_VALUE_4, A53_STL_BM16_IMM_VALUE_5
        bfm             x23, x12, A53_STL_BM16_IMM_VALUE_4, A53_STL_BM16_IMM_VALUE_5

        // 4th test: BFI alias

        bfm             x24, x14, A53_STL_BM16_IMM_VALUE_4, A53_STL_BM16_IMM_VALUE_5
        bfm             x25, x16, A53_STL_BM16_IMM_VALUE_4, A53_STL_BM16_IMM_VALUE_5

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM16_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM16_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM16_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM16_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM16_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_16:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 16
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

a53_stl_core_p004_n001_end:

        ret

        .end
