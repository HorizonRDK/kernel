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
//      Testing of Bit operation instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_011
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p011_n001
//      Testing of Bit operation instructions
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

        .section .section_a53_stl_core_p011_n001,"ax",%progbits
        .global a53_stl_core_p011_n001
        .type a53_stl_core_p011_n001, %function

a53_stl_core_p011_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 52
//-----------------------------------------------------------

// Basic Module 52
// CLS <Xd>, <Xn>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM52_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM52_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM52_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM52_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM52_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM52_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM52_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM52_INPUT_VALUE_4
        mov             x18, A53_STL_BM52_IMM_VALUE_1
        mov             x19, A53_STL_BM52_IMM_VALUE_1
        mov             x20, A53_STL_BM52_IMM_VALUE_1
        mov             x21, A53_STL_BM52_IMM_VALUE_1
        mov             x22, A53_STL_BM52_IMM_VALUE_1
        mov             x23, A53_STL_BM52_IMM_VALUE_1
        mov             x24, A53_STL_BM52_IMM_VALUE_1
        mov             x25, A53_STL_BM52_IMM_VALUE_1

        // 1st test

        cls             x18, x2
        cls             x19, x3

        // 2nd test

        cls             x20, x4
        cls             x21, x5

        // 3rd test

        cls             x22, x6
        cls             x23, x7

        // 4th test

        cls             x24, x8
        cls             x25, x9

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM52_GOLDEN_VALUE_1
        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM52_GOLDEN_VALUE_2
        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM52_GOLDEN_VALUE_3
        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM52_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM52_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_52:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 52
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 53
//-----------------------------------------------------------

// Basic Module 53
// CLZ <Xd>, <Xn>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM53_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM53_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM53_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM53_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM53_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM53_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM53_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM53_INPUT_VALUE_4
        mov             x18, A53_STL_BM53_IMM_VALUE_1
        mov             x19, A53_STL_BM53_IMM_VALUE_1
        mov             x20, A53_STL_BM53_IMM_VALUE_1
        mov             x21, A53_STL_BM53_IMM_VALUE_1
        mov             x22, A53_STL_BM53_IMM_VALUE_1
        mov             x23, A53_STL_BM53_IMM_VALUE_1
        mov             x24, A53_STL_BM53_IMM_VALUE_1
        mov             x25, A53_STL_BM53_IMM_VALUE_1

        // 1st test

        clz             x18, x2
        clz             x19, x3

        // 2nd test

        clz             x20, x4
        clz             x21, x5

        // 3rd test

        clz             x22, x6
        clz             x23, x7

        // 4th test

        clz             x24, x8
        clz             x25, x9

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM53_GOLDEN_VALUE_1
        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM53_GOLDEN_VALUE_2
        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM53_GOLDEN_VALUE_3
        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM53_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM53_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error


check_correct_result_BM_53:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 53
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 54
//-----------------------------------------------------------

// Basic Module 54
// RBIT <Xd>, <Xn>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Set operand register

        ldr             x2, =A53_STL_BM54_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM54_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM54_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM54_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM54_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM54_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM54_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM54_INPUT_VALUE_4
        mov             x18, A53_STL_BM54_IMM_VALUE_1
        mov             x19, A53_STL_BM54_IMM_VALUE_1
        mov             x20, A53_STL_BM54_IMM_VALUE_1
        mov             x21, A53_STL_BM54_IMM_VALUE_1
        mov             x22, A53_STL_BM54_IMM_VALUE_1
        mov             x23, A53_STL_BM54_IMM_VALUE_1
        mov             x24, A53_STL_BM54_IMM_VALUE_1
        mov             x25, A53_STL_BM54_IMM_VALUE_1

        // 1st test

        rbit            x18, x2
        rbit            x19, x3

        // 2nd test

        rbit            x20, x4
        rbit            x21, x5

        // 3rd test

        rbit            x22, x6
        rbit            x23, x7

        // 4th test

        rbit            x24, x8
        rbit            x25, x9

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM54_GOLDEN_VALUE_1
        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM54_GOLDEN_VALUE_2
        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM54_GOLDEN_VALUE_3
        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM54_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM54_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_54:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 54
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 55
//-----------------------------------------------------------

// Basic Module 55
// REV <Xd>, <Xn>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM55_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM55_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM55_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM55_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM55_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM55_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM55_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM55_INPUT_VALUE_4
        mov             x18, A53_STL_BM55_IMM_VALUE_1
        mov             x19, A53_STL_BM55_IMM_VALUE_1
        mov             x20, A53_STL_BM55_IMM_VALUE_1
        mov             x21, A53_STL_BM55_IMM_VALUE_1
        mov             x22, A53_STL_BM55_IMM_VALUE_1
        mov             x23, A53_STL_BM55_IMM_VALUE_1
        mov             x24, A53_STL_BM55_IMM_VALUE_1
        mov             x25, A53_STL_BM55_IMM_VALUE_1

        // 1st test

        rev             x18, x2
        rev             x19, x3

        // 2nd test

        rev             x20, x4
        rev             x21, x5

        // 3rd test: REV64 alias
/*
        rev64           x22, x6
        rev64           x23, x7

        // 4th test: REV64 alias

        rev64           x24, x8
        rev64           x25, x9
*/
		// 3rd test: REV64 alias
		
		rev			x22, x6
		rev			x23, x7
		
		// 4th test: REV64 alias
		
		rev			x24, x8
		rev			x25, x9

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM55_GOLDEN_VALUE_1
        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM55_GOLDEN_VALUE_2
        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM55_GOLDEN_VALUE_3
        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM55_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM55_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_55:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 55
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 56
//-----------------------------------------------------------

// Basic Module 56
// REV16 <Xd>, <Xn>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM56_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM56_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM56_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM56_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM56_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM56_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM56_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM56_INPUT_VALUE_4
        mov             x18, A53_STL_BM56_IMM_VALUE_1
        mov             x19, A53_STL_BM56_IMM_VALUE_1
        mov             x20, A53_STL_BM56_IMM_VALUE_1
        mov             x21, A53_STL_BM56_IMM_VALUE_1
        mov             x22, A53_STL_BM56_IMM_VALUE_1
        mov             x23, A53_STL_BM56_IMM_VALUE_1
        mov             x24, A53_STL_BM56_IMM_VALUE_1
        mov             x25, A53_STL_BM56_IMM_VALUE_1

        // 1st test

        rev16           x18, x2
        rev16           x19, x3

        // 2nd test

        rev16           x20, x4
        rev16           x21, x5

        // 3rd test

        rev16           x22, x6
        rev16           x23, x7

        // 4th test

        rev16           x24, x8
        rev16           x25, x9

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM56_GOLDEN_VALUE_1
        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM56_GOLDEN_VALUE_2
        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM56_GOLDEN_VALUE_3
        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM56_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM56_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_56:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 56
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 57
//-----------------------------------------------------------

// Basic Module 57
// REV32 <Xd>, <Xn>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM57_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM57_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM57_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM57_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM57_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM57_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM57_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM57_INPUT_VALUE_4
        mov             x18, A53_STL_BM57_IMM_VALUE_1
        mov             x19, A53_STL_BM57_IMM_VALUE_1
        mov             x20, A53_STL_BM57_IMM_VALUE_1
        mov             x21, A53_STL_BM57_IMM_VALUE_1
        mov             x22, A53_STL_BM57_IMM_VALUE_1
        mov             x23, A53_STL_BM57_IMM_VALUE_1
        mov             x24, A53_STL_BM57_IMM_VALUE_1
        mov             x25, A53_STL_BM57_IMM_VALUE_1

        // 1st test

        rev32           x18, x2
        rev32           x19, x3

        // 2nd test

        rev32           x20, x4
        rev32           x21, x5

        // 3rd test

        rev32           x22, x6
        rev32           x23, x7

        // 4th test

        rev32           x24, x8
        rev32           x25, x9

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM57_GOLDEN_VALUE_1
        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM57_GOLDEN_VALUE_2
        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM57_GOLDEN_VALUE_3
        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM57_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM57_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_57:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 57
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

a53_stl_core_p011_n001_end:

        ret

        .end
