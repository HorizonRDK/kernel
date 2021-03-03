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
//      CORE_045
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p045_n001
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

        .section .section_a53_stl_core_p045_n001,"ax",%progbits
        .global a53_stl_core_p045_n001
        .type a53_stl_core_p045_n001, %function

a53_stl_core_p045_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 36
//-----------------------------------------------------------

// Basic Module 36
// EON <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM36_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM36_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM36_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM36_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM36_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM36_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM36_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM36_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM36_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM36_INPUT_VALUE_6
        ldr             x12, =A53_STL_BM36_INPUT_VALUE_5
        ldr             x13, =A53_STL_BM36_INPUT_VALUE_6
        ldr             x14, =A53_STL_BM36_INPUT_VALUE_7
        ldr             x15, =A53_STL_BM36_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM36_INPUT_VALUE_7
        ldr             x17, =A53_STL_BM36_INPUT_VALUE_8

        // 1st test

        eon             x18, x2, x3, asr #A53_STL_BM36_IMM_VALUE_1
        eon             x19, x4, x5, asr #A53_STL_BM36_IMM_VALUE_1

        // 2nd test

        eon             x20, x6, x7, lsr #A53_STL_BM36_IMM_VALUE_2
        eon             x21, x8, x9, lsr #A53_STL_BM36_IMM_VALUE_2

        // 3rd test

        eon             x22, x10, x11, lsl #A53_STL_BM36_IMM_VALUE_3
        eon             x23, x12, x13, lsl #A53_STL_BM36_IMM_VALUE_3

        // 4th test

        eon             x24, x14, x15, ror #A53_STL_BM36_IMM_VALUE_4
        eon             x25, x16, x17, ror #A53_STL_BM36_IMM_VALUE_4

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM36_GOLDEN_VALUE_1

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM36_GOLDEN_VALUE_2

        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM36_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM36_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM36_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_36:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 36
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 37
//-----------------------------------------------------------

// Basic Module 37
// EOR <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM37_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM37_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM37_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM37_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM37_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM37_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM37_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM37_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM37_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM37_INPUT_VALUE_6
        ldr             x12, =A53_STL_BM37_INPUT_VALUE_5
        ldr             x13, =A53_STL_BM37_INPUT_VALUE_6
        ldr             x14, =A53_STL_BM37_INPUT_VALUE_7
        ldr             x15, =A53_STL_BM37_INPUT_VALUE_8
        ldr             x16, =A53_STL_BM37_INPUT_VALUE_7
        ldr             x17, =A53_STL_BM37_INPUT_VALUE_8

        // 1st test

        eor             x18, x2, x3, asr #A53_STL_BM37_IMM_VALUE_1
        eor             x19, x4, x5, asr #A53_STL_BM37_IMM_VALUE_1

        // 2nd test

        eor             x20, x6, x7, lsr #A53_STL_BM37_IMM_VALUE_2
        eor             x21, x8, x9, lsr #A53_STL_BM37_IMM_VALUE_2

        // 3rd test

        eor             x22, x10, x11, lsl #A53_STL_BM37_IMM_VALUE_3
        eor             x23, x12, x13, lsl #A53_STL_BM37_IMM_VALUE_3

        // 4th test

        eor             x24, x14, x15, ror #A53_STL_BM37_IMM_VALUE_4
        eor             x25, x16, x17, ror #A53_STL_BM37_IMM_VALUE_4

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM37_GOLDEN_VALUE_1

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM37_GOLDEN_VALUE_2

        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM37_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM37_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM37_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_37:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 37
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 38
//-----------------------------------------------------------

// Basic Module 38
// ORR <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM38_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM38_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM38_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM38_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM38_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM38_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM38_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM38_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM38_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM38_INPUT_VALUE_5
        ldr             x12, =A53_STL_BM38_INPUT_VALUE_6
        ldr             x13, =A53_STL_BM38_INPUT_VALUE_6

        // 1st test

        orr             x18, x2, x3, lsl #A53_STL_BM38_IMM_VALUE_1
        orr             x19, x4, x5, lsl #A53_STL_BM38_IMM_VALUE_1

        // 2nd test

        orr             x20, x6, x7, lsr #A53_STL_BM38_IMM_VALUE_2
        orr             x21, x8, x9, lsr #A53_STL_BM38_IMM_VALUE_2

        // 3rd test: MOV alias

        orr             x22, xzr, x10
        orr             x23, xzr, x11

        // 4th test: MOV alias

        orr             x24, xzr, x12
        orr             x25, xzr, x13

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM38_GOLDEN_VALUE_1

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM38_GOLDEN_VALUE_2

        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM38_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM38_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM38_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_38:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 38
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 39
//-----------------------------------------------------------

// Basic Module 39
// ORN <Xd>, <Xn>, <Xm>{, <shift> #<amount>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x2, =A53_STL_BM39_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM39_INPUT_VALUE_2
        ldr             x4, =A53_STL_BM39_INPUT_VALUE_1
        ldr             x5, =A53_STL_BM39_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM39_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM39_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM39_INPUT_VALUE_3
        ldr             x9, =A53_STL_BM39_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM39_INPUT_VALUE_5
        ldr             x11, =A53_STL_BM39_INPUT_VALUE_5
        ldr             x12, =A53_STL_BM39_INPUT_VALUE_6
        ldr             x13, =A53_STL_BM39_INPUT_VALUE_6

        // 1st test

        orn             x18, x2, x3, lsl #A53_STL_BM39_IMM_VALUE_1
        orn             x19, x4, x5, lsl #A53_STL_BM39_IMM_VALUE_1

        // 2nd test

        orn             x20, x6, x7, lsr #A53_STL_BM39_IMM_VALUE_2
        orn             x21, x8, x9, lsr #A53_STL_BM39_IMM_VALUE_2

        // 3rd test: MVN alias

        orn             x22, xzr, x10
        orn             x23, xzr, x11

        // 4th test: MVN alias

        orn             x24, xzr, x12
        orn             x25, xzr, x13

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM39_GOLDEN_VALUE_1

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM39_GOLDEN_VALUE_2

        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM39_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM39_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM39_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_39:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 39
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

a53_stl_core_p045_n001_end:

        ret

        .end
