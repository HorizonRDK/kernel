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
//      Testing of addition and subtraction instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_003
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p003_n001
//      Testing of addition and subtraction instructions
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

        .section .section_a53_stl_core_p003_n001,"ax",%progbits
        .global a53_stl_core_p003_n001
        .type a53_stl_core_p003_n001, %function

a53_stl_core_p003_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]

        // call preamble subroutine

        bl              a53_stl_preamble
        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 07
//-----------------------------------------------------------

// Basic Module 07
// ADD <Xd|SP>, <Xn|SP>, #<imm>{, <shift>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Initialize operands registers
        mov             x28, A53_STL_BM07_INPUT_VALUE_1
        mov             x29, A53_STL_BM07_INPUT_VALUE_2

        add             x28, x28, A53_STL_BM07_IMM_VALUE_1
        add             x29, x29, A53_STL_BM07_IMM_VALUE_1

        add             x28, x28, A53_STL_BM07_IMM_VALUE_2
        add             x29, x29, A53_STL_BM07_IMM_VALUE_2

        add             x28, x28, A53_STL_BM07_IMM_VALUE_3
        add             x29, x29, A53_STL_BM07_IMM_VALUE_3

        add             x28, x28, A53_STL_BM07_IMM_VALUE_4
        add             x29, x29, A53_STL_BM07_IMM_VALUE_4

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM07_GOLDEN_SIGN_1
        // Compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM07_GOLDEN_SIGN_2

check_correct_result_BM_07:
        // Compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 07
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 08
//-----------------------------------------------------------

// Basic Module 08
// ADDS <Xd>, <Xn|SP>, #<imm>{, <shift>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand registers

        ldr             x1, =A53_STL_BM08_INPUT_VALUE_3
        ldr             x2, =A53_STL_BM08_INPUT_VALUE_3
        ldr             x3, =A53_STL_BM08_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM08_INPUT_VALUE_3
        ldr             x5, =A53_STL_BM08_INPUT_VALUE_1
        ldr             x6, =A53_STL_BM08_INPUT_VALUE_4
        ldr             x7, =A53_STL_BM08_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM08_INPUT_VALUE_2
        ldr             x9, =A53_STL_BM08_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM08_INPUT_VALUE_2

        // Test N = 1, Z = 0, C = 0, V = 1

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x18, x1, A53_STL_BM08_IMM_VALUE_2
        adds            x19, x2, A53_STL_BM08_IMM_VALUE_2
        adds            x20, x3, A53_STL_BM08_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM08_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x21, x4, A53_STL_BM08_IMM_VALUE_2
        adds            x22, x5, A53_STL_BM08_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM08_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM08_GOLDEN_VALUE_1

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM08_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x26, x18
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x19
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x21
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x27, x22
        b.ne            error

        // Test N = 0, Z = 1, C = 1, V = 0

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x18, x6, A53_STL_BM08_IMM_VALUE_2
        adds            x19, x7, A53_STL_BM08_IMM_VALUE_2
        adds            x20, x8, A53_STL_BM08_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM08_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        adds            x21, x9, A53_STL_BM08_IMM_VALUE_2
        adds            x22, x10, A53_STL_BM08_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM08_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM08_GOLDEN_VALUE_3

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
        ldr             x27, =A53_STL_BM08_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x27, x20
        b.ne            error

check_correct_result_BM_08:
        // Compare local and golden signature
        cmp             x27, x22
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 08
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 09
//-----------------------------------------------------------

// Basic Module 09
// SUB <Xd|SP>, <Xn|SP>, #<imm>{, <shift>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Initialize operands registers
        mov             x28, A53_STL_BM09_INPUT_VALUE_1
        mov             x29, A53_STL_BM09_INPUT_VALUE_2

        sub             x28, x28, A53_STL_BM09_IMM_VALUE_1
        sub             x29, x29, A53_STL_BM09_IMM_VALUE_1

        sub             x28, x28, A53_STL_BM09_IMM_VALUE_2
        sub             x29, x29, A53_STL_BM09_IMM_VALUE_2

        sub             x28, x28, A53_STL_BM09_IMM_VALUE_3
        sub             x29, x29, A53_STL_BM09_IMM_VALUE_3

        sub             x28, x28, A53_STL_BM09_IMM_VALUE_4
        sub             x29, x29, A53_STL_BM09_IMM_VALUE_4

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM09_GOLDEN_SIGN_1
        // Compare local and golden signature
        cmp             x26, x28
        b.ne            error

        // Initialize x27 with golden signature
        ldr             x27, =A53_STL_BM09_GOLDEN_SIGN_2

check_correct_result_BM_09:
        // Compare local and golden signature
        cmp             x27, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 09
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 10
//-----------------------------------------------------------

// Basic Module 10
// SUBS <Xd>, <Xn|SP>, #<imm>{, <shift>}

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand registers

        ldr             x1, =A53_STL_BM10_INPUT_VALUE_3
        ldr             x2, =A53_STL_BM10_INPUT_VALUE_3
        ldr             x3, =A53_STL_BM10_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM10_INPUT_VALUE_3
        ldr             x5, =A53_STL_BM10_INPUT_VALUE_1
        ldr             x6, =A53_STL_BM10_INPUT_VALUE_4
        ldr             x7, =A53_STL_BM10_INPUT_VALUE_4
        ldr             x8, =A53_STL_BM10_INPUT_VALUE_2
        ldr             x9, =A53_STL_BM10_INPUT_VALUE_4
        ldr             x10, =A53_STL_BM10_INPUT_VALUE_2
        ldr             x11, =A53_STL_BM10_INPUT_VALUE_5
        ldr             x12, =A53_STL_BM10_INPUT_VALUE_5
        ldr             x13, =A53_STL_BM10_INPUT_VALUE_6
        ldr             x14, =A53_STL_BM10_INPUT_VALUE_5
        ldr             x15, =A53_STL_BM10_INPUT_VALUE_6

        // Test N = 1, Z = 0, C = 1, V = 0

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // Necessary for dual issue pipeline stimulation strategy
        sub             x18, x1, A53_STL_BM10_IMM_VALUE_2
        // Necessary for dual issue pipeline stimulation strategy
        sub             x19, x2, A53_STL_BM10_IMM_VALUE_2
        subs            x20, x3, A53_STL_BM10_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM10_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // Necessary for dual issue pipeline stimulation strategy
        sub             x21, x4, A53_STL_BM10_IMM_VALUE_2
        subs            x22, x5, A53_STL_BM10_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM10_GOLDEN_FLAGS_1

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM10_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x26, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x22
        b.ne            error

        // Test N = 0, Z = 0, C = 1, V = 1

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // Necessary for dual issue pipeline stimulation strategy
        sub             x18, x6, A53_STL_BM10_IMM_VALUE_2
        // Necessary for dual issue pipeline stimulation strategy
        sub             x19, x7, A53_STL_BM10_IMM_VALUE_2
        subs            x20, x8, A53_STL_BM10_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM10_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // Necessary for dual issue pipeline stimulation strategy
        sub             x21, x9, A53_STL_BM10_IMM_VALUE_2
        subs            x22, x10, A53_STL_BM10_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM10_GOLDEN_FLAGS_2

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM10_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x26, x20
        b.ne            error

        // Compare local and golden signature
        cmp             x26, x22
        b.ne            error

        // Test N = 0, Z = 1, C = 1, V = 0

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // Necessary for dual issue pipeline stimulation strategy
        sub             x18, x11, A53_STL_BM10_IMM_VALUE_2
        // Necessary for dual issue pipeline stimulation strategy
        sub             x19, x12, A53_STL_BM10_IMM_VALUE_2
        subs            x20, x13, A53_STL_BM10_IMM_VALUE_1

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM10_GOLDEN_FLAGS_3

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Clear NZCV flags
        mov             x0, A53_STL_NZCV_CLR
        msr             nzcv, x0

        // Necessary for dual issue pipeline stimulation strategy
        sub             x21, x14, A53_STL_BM10_IMM_VALUE_2
        subs            x22, x15, A53_STL_BM10_IMM_VALUE_1
        // Necessary for dual issue pipeline stimulation strategy
        mov             x23, x21

        // Check NZCV flags

        mrs             x17, nzcv

        // Initialize x0 with golden flags value
        ldr             x0, =A53_STL_BM10_GOLDEN_FLAGS_3

        // Compare local and golden flags
        cmp             x0, x17
        b.ne            error

        // Initialize x26 with golden signature
        ldr             x26, =A53_STL_BM10_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x26, x20
        b.ne            error

check_correct_result_BM_10:
        // Compare local and golden signature
        cmp             x26, x22
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 10
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

a53_stl_core_p003_n001_end:

        ret

        .end
