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
//      Testing of shift instructions as ASRV, LSLV, LSRV and RORV
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_009
//
//===========================================================================================//
//===========================================================================================
//   Function: A53_STL_Core_p009_n001
//      Testing of shift instructions as ASRV, LSLV, LSRV and RORV
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

        .section .section_a53_stl_core_p009_n001,"ax",%progbits
        .global a53_stl_core_p009_n001
        .type a53_stl_core_p009_n001, %function

a53_stl_core_p009_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 40
//-----------------------------------------------------------

// Basic Module 40
// ASRV <Xd>, <Xn>, <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM40_INPUT_VALUE_1
        mov             x3, A53_STL_BM40_INPUT_VALUE_5
        ldr             x4, =A53_STL_BM40_INPUT_VALUE_1
        mov             x5, A53_STL_BM40_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM40_INPUT_VALUE_2
        mov             x7, A53_STL_BM40_INPUT_VALUE_6
        ldr             x8, =A53_STL_BM40_INPUT_VALUE_2
        mov             x9, A53_STL_BM40_INPUT_VALUE_6
        ldr             x10, =A53_STL_BM40_INPUT_VALUE_3
        mov             x11, A53_STL_BM40_INPUT_VALUE_7
        ldr             x12, =A53_STL_BM40_INPUT_VALUE_3
        mov             x13, A53_STL_BM40_INPUT_VALUE_7
        ldr             x14, =A53_STL_BM40_INPUT_VALUE_4
        mov             x15, A53_STL_BM40_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM40_INPUT_VALUE_4
        mov             x17, A53_STL_BM40_INPUT_VALUE_8

        // 1st test

        asrv            x18, x2, x3
        asrv            x19, x4, x5

        // 2nd test

        asrv            x20, x6, x7
        asrv            x21, x8, x9

        // 3rd test

        asrv            x22, x10, x11
        asrv            x23, x12, x13

        // 4th test

        asrv            x24, x14, x15
        asrv            x25, x16, x17

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM40_GOLDEN_VALUE_1

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM40_GOLDEN_VALUE_2

        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM40_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM40_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM40_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_40:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 40
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 41
//-----------------------------------------------------------

// Basic Module 41
// LSLV <Xd>, <Xn>, <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        mov             x2, A53_STL_BM41_INPUT_VALUE_1
        mov             x3, A53_STL_BM41_INPUT_VALUE_2
        mov             x4, A53_STL_BM41_INPUT_VALUE_1
        mov             x5, A53_STL_BM41_INPUT_VALUE_2
        mov             x6, A53_STL_BM41_INPUT_VALUE_3
        mov             x7, A53_STL_BM41_INPUT_VALUE_4
        mov             x8, A53_STL_BM41_INPUT_VALUE_3
        mov             x9, A53_STL_BM41_INPUT_VALUE_4
        mov             x10, A53_STL_BM41_INPUT_VALUE_5
        mov             x11, A53_STL_BM41_INPUT_VALUE_6
        mov             x12, A53_STL_BM41_INPUT_VALUE_5
        mov             x13, A53_STL_BM41_INPUT_VALUE_6
        mov             x14, A53_STL_BM41_INPUT_VALUE_7
        mov             x15, A53_STL_BM41_INPUT_VALUE_8
        mov             x16, A53_STL_BM41_INPUT_VALUE_7
        mov             x17, A53_STL_BM41_INPUT_VALUE_8

        // 1st test

        lslv            x18, x2, x3
        lslv            x19, x4, x5

        // 2nd test

        lslv            x20, x6, x7
        lslv            x21, x8, x9

        // 3rd test

        lslv            x22, x10, x11
        lslv            x23, x12, x13

        // 4th test

        lslv            x24, x14, x15
        lslv            x25, x16, x17

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM41_GOLDEN_VALUE_1
        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM41_GOLDEN_VALUE_2
        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM41_GOLDEN_VALUE_3
        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM41_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM41_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_41:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 41
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 42
//-----------------------------------------------------------

// Basic Module 42
// LSRV <Xd>, <Xn>, <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM42_INPUT_VALUE_1
        mov             x3, A53_STL_BM42_INPUT_VALUE_5
        ldr             x4, =A53_STL_BM42_INPUT_VALUE_1
        mov             x5, A53_STL_BM42_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM42_INPUT_VALUE_2
        mov             x7, A53_STL_BM42_INPUT_VALUE_6
        ldr             x8, =A53_STL_BM42_INPUT_VALUE_2
        mov             x9, A53_STL_BM42_INPUT_VALUE_6
        ldr             x10, =A53_STL_BM42_INPUT_VALUE_3
        mov             x11, A53_STL_BM42_INPUT_VALUE_7
        ldr             x12, =A53_STL_BM42_INPUT_VALUE_3
        mov             x13, A53_STL_BM42_INPUT_VALUE_7
        ldr             x14, =A53_STL_BM42_INPUT_VALUE_4
        mov             x15, A53_STL_BM42_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM42_INPUT_VALUE_4
        mov             x17, A53_STL_BM42_INPUT_VALUE_8

        // 1st test

        lsrv            x18, x2, x3
        lsrv            x19, x4, x5

        // 2nd test

        lsrv            x20, x6, x7
        lsrv            x21, x8, x9

        // 3rd test

        lsrv            x22, x10, x11
        lsrv            x23, x12, x13

        // 4th test

        lsrv            x24, x14, x15
        lsrv            x25, x16, x17

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM42_GOLDEN_VALUE_1
        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM42_GOLDEN_VALUE_2
        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM42_GOLDEN_VALUE_3
        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM42_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM42_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_42:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 42
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 43
//-----------------------------------------------------------

// Basic Module 43
// RORV <Xd>, <Xn>, <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM43_INPUT_VALUE_1
        mov             x3, A53_STL_BM43_INPUT_VALUE_5
        ldr             x4, =A53_STL_BM43_INPUT_VALUE_1
        mov             x5, A53_STL_BM43_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM43_INPUT_VALUE_2
        mov             x7, A53_STL_BM43_INPUT_VALUE_6
        ldr             x8, =A53_STL_BM43_INPUT_VALUE_2
        mov             x9, A53_STL_BM43_INPUT_VALUE_6
        ldr             x10, =A53_STL_BM43_INPUT_VALUE_3
        mov             x11, A53_STL_BM43_INPUT_VALUE_7
        ldr             x12, =A53_STL_BM43_INPUT_VALUE_3
        mov             x13, A53_STL_BM43_INPUT_VALUE_7
        ldr             x14, =A53_STL_BM43_INPUT_VALUE_4
        mov             x15, A53_STL_BM43_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM43_INPUT_VALUE_4
        mov             x17, A53_STL_BM43_INPUT_VALUE_8

        // 1st test

        rorv            x18, x2, x3
        rorv            x19, x4, x5

        // 2nd test

        rorv            x20, x6, x7
        rorv            x21, x8, x9

        // 3rd test

        rorv            x22, x10, x11
        rorv            x23, x12, x13

        // 4th test

        rorv            x24, x14, x15
        rorv            x25, x16, x17

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM43_GOLDEN_VALUE_1

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM43_GOLDEN_VALUE_2

        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM43_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM43_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM43_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_43:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 43
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

a53_stl_core_p009_n001_end:

        ret

        .end
