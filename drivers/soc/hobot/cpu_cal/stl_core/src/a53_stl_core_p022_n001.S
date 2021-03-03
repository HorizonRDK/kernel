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
//      Testing of SIMD and FP instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_FPU
//
//   About: TEST_ID
//      CORE_022
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p022_n001
//      Testing of SIMD and FP instructions
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

        .section .section_a53_stl_core_p022_n001,"ax",%progbits
        .global a53_stl_core_p022_n001
        .type a53_stl_core_p022_n001, %function

a53_stl_core_p022_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping



//-----------------------------------------------------------
// START BASIC MODULE 86
//-----------------------------------------------------------

// Basic Module 86
// RSUBHN{2} <Vd>.<Tb>, <Vn>.<Ta>, <Vm>.<Ta>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM86_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM86_INPUT_VALUE_2
        ldr             x2, =A53_STL_BM86_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM86_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute rsubhn instruction

        rsubhn          v17.4h, v1.4s, v5.4s
        rsubhn          v18.4h, v3.4s, v7.4s

        rsubhn          v19.4h, v2.4s, v6.4s
        rsubhn          v20.4h, v4.4s, v8.4s

        // Copy v17 in x11 and x15 for signature check
        fmov            x11, d17
        fmov            x15, v17.d[1]

        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d18
        fmov            x16, v18.d[1]

        // Copy v19 in x13 and x17 for signature check
        fmov            x13, d19
        fmov            x17, v19.d[1]

        // Copy v20 in x14 and x18 for signature check
        fmov            x14, d20
        fmov            x18, v20.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM86_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM86_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM86_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM86_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

check_correct_result_BM_86:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 86
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 87
//-----------------------------------------------------------

// Basic Module 87
// URSQRTE <Vd>.<T>, <Vn>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x2 with the test values
        ldr             x1, =A53_STL_BM87_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM87_INPUT_VALUE_2

        bl              fmov_d1d4_x1x2

        bl              fmov_v1v4_x1x2

        // execute ursqrte instruction

        ursqrte         v17.4s, v1.4s
        ursqrte         v18.4s, v3.4s

        ursqrte         v19.4s, v2.4s
        ursqrte         v20.4s, v4.4s

        // Copy v17 in x11 and x15 for signature check
        fmov            x11, d17
        fmov            x15, v17.d[1]

        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d18
        fmov            x16, v18.d[1]

        // Copy v19 in x13 and x17 for signature check
        fmov            x13, d19
        fmov            x17, v19.d[1]

        // Copy v20 in x14 and x18 for signature check
        fmov            x14, d20
        fmov            x18, v20.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM87_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM87_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

check_correct_result_BM_87:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 87
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 88
//-----------------------------------------------------------

// Basic Module 88
// SADDLV <V><d>, <Vn>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x2 with the test values
        ldr             x1, =A53_STL_BM88_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM88_INPUT_VALUE_2

        bl              fmov_d1d4_x1x2

        bl              fmov_v1v4_x1x2

        // execute saddlv instruction

        saddlv          d17, v1.4s
        saddlv          d18, v3.4s

        saddlv          d19, v2.4s
        saddlv          d20, v4.4s

        // Copy v17 in x11 for signature check
        fmov            x11, d17

        // Copy v18 in x12 for signature check
        fmov            x12, d18

        // Copy v19 in x13 for signature check
        fmov            x13, d19

        // Copy v20 in x14 for signature check
        fmov            x14, d20

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM88_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM88_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

check_correct_result_BM_88:

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 88
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

a53_stl_core_p022_n001_end:

        ret

        .end
