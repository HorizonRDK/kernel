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
//      Testing of CRC instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_010
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p010_n001
//      Testing of CRC instructions
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

        .section .section_a53_stl_core_p010_n001,"ax",%progbits
        .global a53_stl_core_p010_n001
        .type a53_stl_core_p010_n001, %function

a53_stl_core_p010_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 44
//-----------------------------------------------------------

// Basic Module 44
// CRC32B, CRC32H, CRC32W, CRC32X test

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        mov             w28, A53_STL_BM44_INPUT_VALUE_1
        mov             w29, A53_STL_BM44_INPUT_VALUE_2

        // Set operand register

        mov             w2, A53_STL_BM44_INPUT_VALUE_3
        mov             w3, A53_STL_BM44_INPUT_VALUE_4
        mov             w4, A53_STL_BM44_INPUT_VALUE_5
        mov             w5, A53_STL_BM44_INPUT_VALUE_6
        mov             w6, A53_STL_BM44_INPUT_VALUE_7
        mov             w7, A53_STL_BM44_INPUT_VALUE_8
        mov             x8, A53_STL_BM44_INPUT_VALUE_9
        mov             x9, A53_STL_BM44_INPUT_VALUE_10

        // CRC32B <Wd>, <Wn>, <Wm>

        crc32b          w28, w28, w2
        crc32b          w29, w29, w2

        crc32b          w28, w28, w3
        crc32b          w29, w29, w3


        // CRC32H <Wd>, <Wn>, <Wm>

        crc32h          w28, w28, w4
        crc32h          w29, w29, w4

        crc32h          w28, w28, w5
        crc32h          w29, w29, w5

        // CRC32W <Wd>, <Wn>, <Wm>

        crc32w          w28, w28, w6
        crc32w          w29, w29, w6

        crc32w          w28, w28, w7
        crc32w          w29, w29, w7

        // CRC32X <Wd>, <Wn>, <Xm>

        crc32x          w28, w28, x8
        crc32x          w29, w29, x8

        crc32x          w28, w28, x9
        crc32x          w29, w29, x9

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM44_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM44_GOLDEN_SIGN_2

check_correct_result_BM_44:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 44
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 45
//-----------------------------------------------------------

// Basic Module 45
// CRC32CB, CRC32CH, CRC32CW, CRC32CX tests

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation registers initialization

        mov             w28, A53_STL_BM45_INPUT_VALUE_1
        mov             w29, A53_STL_BM45_INPUT_VALUE_2

        // Set operand register
        mov             w10, A53_STL_BM45_INPUT_VALUE_3
        mov             w11, A53_STL_BM45_INPUT_VALUE_4
        mov             w12, A53_STL_BM45_INPUT_VALUE_5
        mov             w13, A53_STL_BM45_INPUT_VALUE_6
        mov             w14, A53_STL_BM45_INPUT_VALUE_7
        mov             w15, A53_STL_BM45_INPUT_VALUE_8
        mov             x16, A53_STL_BM45_INPUT_VALUE_9
        mov             x17, A53_STL_BM45_INPUT_VALUE_10


        // CRC32CB <Wd>, <Wn>, <Wm>

        crc32cb         w28, w28, w10
        crc32cb         w29, w29, w10

        crc32cb         w28, w28, w11
        crc32cb         w29, w29, w11


        // CRC32CH <Wd>, <Wn>, <Wm>

        crc32ch         w28, w28, w12
        crc32ch         w29, w29, w12

        crc32ch         w28, w28, w13
        crc32ch         w29, w29, w13

        // CRC32CW <Wd>, <Wn>, <Wm>

        crc32cw         w28, w28, w14
        crc32cw         w29, w29, w14

        crc32cw         w28, w28, w15
        crc32cw         w29, w29, w15

        // CRC32CX <Wd>, <Wn>, <Xm>

        crc32cx         w28, w28, x16
        crc32cx         w29, w29, x16

        crc32cx         w28, w28, x17
        crc32cx         w29, w29, x17

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM45_GOLDEN_SIGN_1

        // compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM45_GOLDEN_SIGN_2

check_correct_result_BM_45:
        // compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 45
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

a53_stl_core_p010_n001_end:

        ret

        .end
