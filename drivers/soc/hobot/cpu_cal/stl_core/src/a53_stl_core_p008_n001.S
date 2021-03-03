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
//      Testing of logical instructions as AND, ANDS, BIC, BICS,  EON, EOR, ORR and ORN
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_008
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p008_n001
//      Testing of logical instructions as AND, ANDS, BIC, BICS,  EON, EOR, ORR and ORN
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

        .section .section_a53_stl_core_p008_n001,"ax",%progbits
        .global a53_stl_core_p008_n001
        .type a53_stl_core_p008_n001, %function

a53_stl_core_p008_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 32
//-----------------------------------------------------------

// Basic Module 32
// AND <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        mov             x2, A53_STL_BM32_INPUT_VALUE_1
        mov             x3, A53_STL_BM32_INPUT_VALUE_2
        mov             x4, A53_STL_BM32_INPUT_VALUE_1
        mov             x5, A53_STL_BM32_INPUT_VALUE_2
        mov             x6, A53_STL_BM32_INPUT_VALUE_3
        mov             x7, A53_STL_BM32_INPUT_VALUE_4
        mov             x8, A53_STL_BM32_INPUT_VALUE_3
        mov             x9, A53_STL_BM32_INPUT_VALUE_4
        mov             x10, A53_STL_BM32_INPUT_VALUE_5
        mov             x11, A53_STL_BM32_INPUT_VALUE_6
        mov             x12, A53_STL_BM32_INPUT_VALUE_5
        mov             x13, A53_STL_BM32_INPUT_VALUE_6
        mov             x14, A53_STL_BM32_INPUT_VALUE_7
        mov             x15, A53_STL_BM32_INPUT_VALUE_8
        mov             x16, A53_STL_BM32_INPUT_VALUE_7
        mov             x17, A53_STL_BM32_INPUT_VALUE_8

        and             x18, x2, x6
        and             x19, x4, x8

        and             x20, x3, x7
        and             x21, x5, x9

        and             x22, x10, x14
        and             x23, x12, x16

        and             x24, x11, x15
        and             x25, x13, x17

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM32_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM32_GOLDEN_VALUE_2

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM32_GOLDEN_VALUE_3

        // Initialize x29 with golden signature
        ldr             x29, =A53_STL_BM32_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM32_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_32:
        // Compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 32
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 33
//-----------------------------------------------------------

// Basic Module 33
// ANDS <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Test N = 1, Z = 0, C = 0, V = 0

        // Set operand registers

        ldr             x1, =A53_STL_BM33_INPUT_VALUE_3
        ldr             x2, =A53_STL_BM33_INPUT_VALUE_3
        ldr             x3, =A53_STL_BM33_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM33_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM33_INPUT_VALUE_1
        ldr             x6, =A53_STL_BM33_INPUT_VALUE_2
        ldr             x7, =A53_STL_BM33_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM33_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM33_INPUT_VALUE_1
        ldr             x10, =A53_STL_BM33_INPUT_VALUE_2

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        ands            x18, x1, x3, lsl #A53_STL_BM33_IMM_VALUE_1
        ands            x19, x2, x4, lsl #A53_STL_BM33_IMM_VALUE_1
        ands            x20, x5, x6, lsl #A53_STL_BM33_IMM_VALUE_2

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM33_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        ands            x21, x7, x8, lsl #A53_STL_BM33_IMM_VALUE_1
        ands            x22, x9, x10, lsl #A53_STL_BM33_IMM_VALUE_2
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM33_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM33_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x26, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x19
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x21
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM33_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x22
        b.ne            error

        // Test N = 1, Z = 0, C = 0, V = 0 with TST alias

        // Set operand registers

        ldr             x1, =A53_STL_BM33_INPUT_VALUE_7
        ldr             x2, =A53_STL_BM33_INPUT_VALUE_7
        ldr             x3, =A53_STL_BM33_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM33_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM33_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM33_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM33_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM33_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM33_INPUT_VALUE_5
        ldr             x10, =A53_STL_BM33_INPUT_VALUE_6

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        ands            x18, x1, x3, lsl #A53_STL_BM33_IMM_VALUE_2
        ands            x19, x2, x4, lsl #A53_STL_BM33_IMM_VALUE_2
        ands            xzr, x5, x6, lsl #A53_STL_BM33_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM33_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        ands            x21, x7, x8, lsl #A53_STL_BM33_IMM_VALUE_2
        ands            xzr, x9, x10, lsl #A53_STL_BM33_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM33_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM33_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x26, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x19
        b.ne            error

check_correct_result_BM_33:
        // Compare local and golden signature
        cmp             x26, x21
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 33
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 34
//-----------------------------------------------------------

// Basic Module 34
// BIC <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM34_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM34_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM34_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM34_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM34_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM34_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM34_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM34_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM34_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM34_INPUT_VALUE_6
        ldr             x12, =A53_STL_BM34_INPUT_VALUE_5
        ldr             x13, =A53_STL_BM34_INPUT_VALUE_6
        ldr             x14, =A53_STL_BM34_INPUT_VALUE_7
        ldr             x15, =A53_STL_BM34_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM34_INPUT_VALUE_7
        ldr             x17, =A53_STL_BM34_INPUT_VALUE_8

        // 1st test

        bic             x18, x2, x3, asr #A53_STL_BM34_IMM_VALUE_1
        bic             x19, x4, x5, asr #A53_STL_BM34_IMM_VALUE_1

        // 2nd test

        bic             x20, x6, x7, lsr #A53_STL_BM34_IMM_VALUE_2
        bic             x21, x8, x9, lsr #A53_STL_BM34_IMM_VALUE_2

        // 3rd test

        bic             x22, x10, x11, lsl #A53_STL_BM34_IMM_VALUE_3
        bic             x23, x12, x13, lsl #A53_STL_BM34_IMM_VALUE_3

        // 4th test

        bic             x24, x14, x15, ror #A53_STL_BM34_IMM_VALUE_4
        bic             x25, x16, x17, ror #A53_STL_BM34_IMM_VALUE_4

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM34_GOLDEN_VALUE_1

        ldr             x27, =A53_STL_BM34_GOLDEN_VALUE_2

        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM34_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM34_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM34_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_34:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 34
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 35
//-----------------------------------------------------------

// Basic Module 35
// BICS <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Test N = 1, Z = 0, C = 0, V = 0

        // Set operand registers

        ldr             x1, =A53_STL_BM35_INPUT_VALUE_3
        ldr             x2, =A53_STL_BM35_INPUT_VALUE_3
        ldr             x3, =A53_STL_BM35_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM35_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM35_INPUT_VALUE_1
        ldr             x6, =A53_STL_BM35_INPUT_VALUE_2
        ldr             x7, =A53_STL_BM35_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM35_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM35_INPUT_VALUE_1
        ldr             x10, =A53_STL_BM35_INPUT_VALUE_2

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        bics            x18, x1, x3, lsl #A53_STL_BM35_IMM_VALUE_1
        bics            x19, x2, x4, lsl #A53_STL_BM35_IMM_VALUE_1
        bics            x20, x5, x6, lsl #A53_STL_BM35_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM35_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        bics            x21, x7, x8, lsl #A53_STL_BM35_IMM_VALUE_1
        bics            x22, x9, x10, lsl #A53_STL_BM35_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM35_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM35_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x26, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x19
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x21
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM35_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x22
        b.ne            error

        // Test N = 0, Z = 1, C = 0, V = 0

        // Set operand registers

        ldr             x1, =A53_STL_BM35_INPUT_VALUE_7
        ldr             x2, =A53_STL_BM35_INPUT_VALUE_7
        ldr             x3, =A53_STL_BM35_INPUT_VALUE_4
        ldr             x4, =A53_STL_BM35_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM35_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM35_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM35_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM35_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM35_INPUT_VALUE_5
        ldr             x10, =A53_STL_BM35_INPUT_VALUE_6

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        bics            x18, x1, x3, lsr #A53_STL_BM35_IMM_VALUE_1
        bics            x19, x2, x4, lsr #A53_STL_BM35_IMM_VALUE_1
        bics            x20, x5, x6, lsr #A53_STL_BM35_IMM_VALUE_2

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM35_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        bics            x21, x7, x8, lsr #A53_STL_BM35_IMM_VALUE_1
        bics            x22, x9, x10, lsr #A53_STL_BM35_IMM_VALUE_2
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM35_GOLDEN_FLAGS_2
        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM35_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x26, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x19
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x21
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM35_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

check_correct_result_BM_35:
        // Compare local and golden signature
        cmp             x27, x22
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 35
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

a53_stl_core_p008_n001_end:

        ret

        .end
