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
//      CORE_028
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p028_n001
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

        .section .section_a53_stl_core_p028_n001,"ax",%progbits
        .global a53_stl_core_p028_n001
        .type a53_stl_core_p028_n001, %function

a53_stl_core_p028_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 13
//-----------------------------------------------------------

// Basic Module 13
// FCVTMS (scalar)

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register

        ldr             x2, =A53_STL_BM13_INPUT_VALUE_1
        ldr             x3, =A53_STL_BM13_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM13_INPUT_VALUE_2
        ldr             x5, =A53_STL_BM13_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM13_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM13_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM13_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM13_INPUT_VALUE_4

        bl              fmov_d2d9_x2x9

        // FCVTMS <Xd>, <Dn>

        fcvtms          x18, d2
        fcvtms          x19, d3

        fcvtms          x20, d4
        fcvtms          x21, d5

        fcvtms          x22, d6
        fcvtms          x23, d7

        fcvtms          x24, d8
        fcvtms          x25, d9

        // initialize w26 with golden signature
        ldr             x26, =A53_STL_BM13_GOLDEN_VALUE_1

        // initialize w27 with golden signature
        ldr             x27, =A53_STL_BM13_GOLDEN_VALUE_2

        // initialize w27 with golden signature
        ldr             x28, =A53_STL_BM13_GOLDEN_VALUE_3

        // initialize w27 with golden signature
        ldr             x29, =A53_STL_BM13_GOLDEN_VALUE_4

        // Check results
        bl              check_x26x29_x18x24
        // Initialize x27 with the first expected value to check the error in the check_x26x29_x18x24
        ldr             x27, =A53_STL_BM13_GOLDEN_VALUE_1
        cmp             x26, x27
        b.ne            error

check_correct_result_BM_13:
        // compare local and golden signature
        cmp             x29, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 13
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 30
//-----------------------------------------------------------

// Basic Module 30
// FMAXNMV (Single-precision and double-precision variant) test
// FMAXNMV <V><d>, <Vn>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Set operand register
        ldr             x1, =A53_STL_BM30_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM30_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM30_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM30_INPUT_VALUE_4

        // low part of V1,Q1
        fmov            d1, x1
        // high part of V1,Q1
        fmov            v1.d[1], x2
        // low part of V2,Q2
        fmov            d2, x1
        // high part of V2,Q2
        fmov            v2.d[1], x2
        // low part of V3,Q3
        fmov            d3, x3
        // high part of V3,Q3
        fmov            v3.d[1], x4
        // low part of V4,Q4
        fmov            d4, x3
        // high part of V4,Q4
        fmov            v4.d[1], x4

        // FMAXNMV <V><d>, <Vn>.<T>

        fmaxnmv         s5, v1.4s
        fmaxnmv         s6, v2.4s

        fmaxnmv         s7, v3.4s
        fmaxnmv         s8, v4.4s

        // initialize s20 with golden signature
        ldr             s20, =A53_STL_BM30_GOLDEN_VALUE_1

        // compare local and golden signature
        fcmp            s20, s5
        b.ne            error

        // compare local and golden signature
        fcmp            s20, s6
        b.ne            error

        // compare local and golden signature
        fcmp            s20, s7
        b.ne            error

check_correct_result_BM_30:
        // compare local and golden signature
        fcmp            s20, s8
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 30
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 31
//-----------------------------------------------------------

// Basic Module 31
// FCVTNS <Xd>, <Dn>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x2 with the test values
        ldr             x1, =A53_STL_BM31_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM31_INPUT_VALUE_2

        bl              fmov_d1d4_x1x2

        // Execute FCVTNS instruction
        fcvtns          x10, d1
        fcvtns          x11, d3

        fcvtns          x12, d2
        fcvtns          x13, d4

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM31_GOLDEN_VALUE_1

        // compare local and golden signature
        cmp             x10, x28
        b.ne            error

        // compare local and golden signature
        cmp             x11, x28
        b.ne            error


        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM31_GOLDEN_VALUE_2

        // compare local and golden signature
        cmp             x12, x28
        b.ne            error

check_correct_result_BM_31:
        // compare local and golden signature
        cmp             x13, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 31
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 46
//-----------------------------------------------------------

// Basic Module 46
// FRINTA <Dd>, <Dn>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x2 with the test values
        ldr             x1, =A53_STL_BM46_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM46_INPUT_VALUE_2

        bl              fmov_d1d4_x1x2

        // Execute FRINTA instruction
        frinta          d10, d1
        frinta          d11, d3

        frinta          d12, d2
        frinta          d13, d4

        // Move fp registers into gp registers
        fmov            x10, d10
        fmov            x11, d11
        fmov            x12, d12
        fmov            x13, d13

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM46_GOLDEN_VALUE_1

        // compare local and golden signature
        cmp             x10, x28
        b.ne            error

        // compare local and golden signature
        cmp             x11, x28
        b.ne            error


        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM46_GOLDEN_VALUE_2

        // compare local and golden signature
        cmp             x12, x28
        b.ne            error

check_correct_result_BM_46:
        // compare local and golden signature
        cmp             x13, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 46
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 47
//-----------------------------------------------------------

// Basic Module 47
// FRSQRTE <Vd>.<T>, <Vn>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        mov             x1, A53_STL_BM47_INPUT_VALUE_1
        mov             x2, A53_STL_BM47_INPUT_VALUE_2
        mov             x3, A53_STL_BM47_INPUT_VALUE_3
        mov             x4, A53_STL_BM47_INPUT_VALUE_4

        bl              fmov_d1d4_x1x2

        bl              fmov_v1v4_x3x4

        // Execute FRSQRTE instruction

        frsqrte         v17.2d, v1.2d
        frsqrte         v18.2d, v3.2d

        frsqrte         v19.2d, v2.2d
        frsqrte         v20.2d, v4.2d

        // Copy v17 in x11 and x12 for signature check
        fmov            x11, d17
        fmov            x12, v17.d[1]
        // Copy v18 in x13 and x14 for signature check
        fmov            x13, d18
        fmov            x14, v18.d[1]

        // Copy v19 in x15 and x16 for signature check
        fmov            x15, d19
        fmov            x16, v19.d[1]
        // Copy v20 in x17 and x18 for signature check
        fmov            x17, d20
        fmov            x18, v20.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM47_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x13, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM47_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x27 the golden value
        ldr             x28, =A53_STL_BM47_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x15, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

check_correct_result_BM_47:
        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 47
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 48
//-----------------------------------------------------------

// Basic Module 48
// FMINNMP (Single-precision and double-precision) test
// FMINNMP <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x1, =A53_STL_BM48_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM48_INPUT_VALUE_2

        fmov            d1, x1
        fmov            d2, x2
        fmov            d3, x1
        fmov            d4, x2
        fmov            v1.d[1], x2
        fmov            v2.d[1], x1
        fmov            v3.d[1], x2
        fmov            v4.d[1], x1
        fmov            d5, x2
        fmov            d6, x1
        fmov            d7, x2
        fmov            d8, x1
        fmov            v5.d[1], x1
        fmov            v6.d[1], x2
        fmov            v7.d[1], x1
        fmov            v8.d[1], x2

        // FMINNMP <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        fminnmp         v10.2D, v1.2D, v2.2D
        fminnmp         v11.2D, v3.2D, v4.2D

        fminnmp         v12.2D, v5.2D, v6.2D
        fminnmp         v13.2D, v7.2D, v8.2D

        // initialize x20 with golden signature
        ldr             x20, =A53_STL_BM48_GOLDEN_VALUE_1
        fmov            d20, x20

        // compare local and golden signature
        fmov            x21, v10.d[1]
        fmov            d21, x21
        fcmp            d10, d20
        b.ne            error
        fcmp            d21, d20
        b.ne            error

        fmov            x21, v11.d[1]
        fmov            d21, x21
        fcmp            d11, d20
        b.ne            error
        fcmp            d21, d20
        b.ne            error

        fmov            x21, v12.d[1]
        fmov            d21, x21
        fcmp            d12, d20
        b.ne            error
        fcmp            d21, d20
        b.ne            error

        fmov            x21, v13.d[1]
check_correct_result_BM_48:
        fmov            d21, x21
        fcmp            d13, d20
        b.ne            error
        fcmp            d21, d20
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 48
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 49
//-----------------------------------------------------------

// Basic Module 49
// SQRSHRUN <Vd>.<Tb>, <Vn>.<Ta>, #<shift>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM49_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM49_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM49_INPUT_VALUE_1
        ldr             x4, =A53_STL_BM49_INPUT_VALUE_2

        bl              fmov_d1d4_x1x2

        bl              fmov_v1v4_x3x4

        // Execute SQRSHRUN instruction

        sqshrun         v17.2s, v1.2d, A53_STL_BM49_INPUT_VALUE_3
        sqshrun         v18.2s, v3.2d, A53_STL_BM49_INPUT_VALUE_3

        sqshrun         v19.2s, v2.2d, A53_STL_BM49_INPUT_VALUE_4
        sqshrun         v20.2s, v4.2d, A53_STL_BM49_INPUT_VALUE_4

        // Copy v17 in x11 for signature check
        fmov            x11, d17
        // Copy v18 in x12 for signature check
        fmov            x12, d18

        // Copy v19 in x13 for signature check
        fmov            x13, d19
        // Copy v20 in x14 for signature check
        fmov            x14, d20

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM49_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM49_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

check_correct_result_BM_49:
        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 49
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 50
//-----------------------------------------------------------

// Basic Module 50
// FACGT (Vector single-precision and double-precision) test
// FACGT <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Set operand register

        ldr             x1, =A53_STL_BM50_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM50_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM50_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM50_INPUT_VALUE_4

        fmov            d1, x1
        fmov            d2, x3
        fmov            d3, x1
        fmov            d4, x3
        fmov            v1.d[1], x2
        fmov            v2.d[1], x4
        fmov            v3.d[1], x2
        fmov            v4.d[1], x4
        fmov            d5, x2
        fmov            d6, x4
        fmov            d7, x2
        fmov            d8, x4
        fmov            v5.d[1], x1
        fmov            v6.d[1], x3
        fmov            v7.d[1], x1
        fmov            v8.d[1], x3

        // FACGT <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        facgt           v10.2D, v1.2D, v2.2D
        facgt           v11.2D, v3.2D, v4.2D

        facgt           v12.2D, v5.2D, v6.2D
        facgt           v13.2D, v7.2D, v8.2D

        // initialize x20 and x21 with golden signatures
        ldr             x20, =A53_STL_BM50_GOLDEN_VALUE_1
        ldr             x21, =A53_STL_BM50_GOLDEN_VALUE_2

        // compare local and golden signatures
        // Check result in v10
        fmov            x22, v10.d[1]
        fmov            x23, d10
        cmp             x20, x22
        b.ne            error
        cmp             x21, x23
        b.ne            error

        // Check result in v11
        fmov            x22, v11.d[1]
        fmov            x23, d11
        cmp             x20, x22
        b.ne            error
        cmp             x21, x23
        b.ne            error

        // Check result in v12
        fmov            x22, v12.d[1]
        fmov            x23, d12
        cmp             x20, x23
        b.ne            error
        cmp             x21, x22
        b.ne            error

        // Check result in v13
        fmov            x22, v13.d[1]
        fmov            x23, d13
        cmp             x20, x23
        b.ne            error
check_correct_result_BM_50:
        cmp             x21, x22
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 50
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

a53_stl_core_p028_n001_end:

        ret

        .end
