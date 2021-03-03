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
//      CORE_032
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p032_n001
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

        .section .section_a53_stl_core_p032_n001,"ax",%progbits
        .global a53_stl_core_p032_n001
        .type a53_stl_core_p032_n001, %function

a53_stl_core_p032_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 102
//-----------------------------------------------------------

// Basic Module 102
// FCVTZS <Vd>.<T>, <Vn>.<T>, #<fbits>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM102_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM102_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM102_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM102_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM102_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM102_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM102_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM102_INPUT_VALUE_8

        bl              fmov_d1d8_x1x8

        bl              fmov_v1v8_x1x8

        // clean FPSR register
        msr             fpsr, xzr

        fcvtzs          v20.2d, v3.2d, A53_STL_BM102_INPUT_VALUE_2
        fcvtzs          v18.2d, v7.2d, A53_STL_BM102_INPUT_VALUE_6
        fcvtzs          v19.2d, v1.2d, A53_STL_BM102_INPUT_VALUE_8

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM102_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        fcvtzs          v21.2d, v7.2d, A53_STL_BM102_INPUT_VALUE_8
        fcvtzs          v22.2d, v5.2d, A53_STL_BM102_INPUT_VALUE_4
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM102_FPSR_VALUE_2

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // Copy v20 in x11 and x15 for signature check
        fmov            x11, d20
        fmov            x15, v20.d[1]
        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d18
        fmov            x16, v18.d[1]

        // Copy v19 in x13 and x17 for signature check
        fmov            x13, d19
        fmov            x17, v19.d[1]

        // Copy v21 in x14 and x18 for signature check
        fmov            x14, d21
        fmov            x18, v21.d[1]

        // Copy v22 in x20 and x21 for signature check
        fmov            x20, d22
        fmov            x21, v22.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM102_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM102_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM102_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM102_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM102_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

check_correct_result_BM_102:

        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 102
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 103
//-----------------------------------------------------------

// Basic Module 103
// SADALP <Vd>.<Ta>, <Vn>.<Tb>
        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM103_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM103_INPUT_VALUE_2

        bl              fmov_d1d4_x1x2
        fmov            d17, xzr
        fmov            d18, xzr
        fmov            d19, xzr
        fmov            d20, xzr

        bl              fmov_v1v4_x1x2
        fmov            v17.d[1], xzr
        fmov            v18.d[1], xzr
        fmov            v19.d[1], xzr
        fmov            v20.d[1], xzr

        // execute sadalp instruction

        sadalp          v17.2d, v1.4s
        sadalp          v18.2d, v3.4s

        sadalp          v19.2d, v2.4s
        sadalp          v20.2d, v4.4s

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
        ldr             x27, =A53_STL_BM103_GOLDEN_VALUE_1

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
        ldr             x28, =A53_STL_BM103_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

check_correct_result_BM_103:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 103
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 104
//-----------------------------------------------------------

// Basic Module 104
// SQRSHRN{2} <Vd>.<Tb>, <Vn>.<Ta>, #<shift>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM104_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM104_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM104_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM104_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM104_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM104_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM104_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM104_INPUT_VALUE_8

        bl              fmov_d1d8_x1x8
        fmov            d18, xzr
        fmov            d19, xzr
        fmov            d20, xzr
        fmov            d21, xzr
        fmov            d22, xzr

        bl              fmov_v1v8_x1x8
        fmov            v18.d[1], xzr
        fmov            v19.d[1], xzr
        fmov            v20.d[1], xzr
        fmov            v21.d[1], xzr
        fmov            v22.d[1], xzr

        // clean FPSR register
        msr             fpsr, xzr

        sqrshrn2        v20.4s, v3.2d, A53_STL_BM104_INPUT_VALUE_2
        sqrshrn2        v18.4s, v7.2d, A53_STL_BM104_INPUT_VALUE_6
        sqrshrn2        v19.4s, v1.2d, A53_STL_BM104_INPUT_VALUE_8

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM104_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        sqrshrn2        v21.4s, v7.2d, A53_STL_BM104_INPUT_VALUE_8
        sqrshrn2        v22.4s, v5.2d, A53_STL_BM104_INPUT_VALUE_4
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM104_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // Copy v20 in x11 and x15 for signature check
        fmov            x11, d20
        fmov            x15, v20.d[1]
        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d18
        fmov            x16, v18.d[1]

        // Copy v19 in x13 and x17 for signature check
        fmov            x13, d19
        fmov            x17, v19.d[1]

        // Copy v21 in x14 and x18 for signature check
        fmov            x14, d21
        fmov            x18, v21.d[1]

        // Copy v22 in x20 and x21 for signature check
        fmov            x20, d22
        fmov            x21, v22.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM104_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM104_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM104_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM104_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM104_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM104_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM104_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM104_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM104_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM104_GOLDEN_VALUE_6

check_correct_result_BM_104:

        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 104
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 105
//-----------------------------------------------------------

// Basic Module 105
// INS <Vd>.<Ts>[<index>], <R><n>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM105_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM105_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM105_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM105_INPUT_VALUE_4
        fmov            d17, xzr
        fmov            d18, xzr
        fmov            d19, xzr
        fmov            d20, xzr
        fmov            v17.d[1], xzr
        fmov            v18.d[1], xzr
        fmov            v19.d[1], xzr
        fmov            v20.d[1], xzr

        // execute ext instruction

        ins             v17.h[2], w1
        ins             v18.h[3], w2

        ins             v19.d[0], x3
        ins             v20.d[1], x4

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
        ldr             x27, =A53_STL_BM105_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM105_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM105_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM105_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM105_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM105_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM105_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM105_GOLDEN_VALUE_5


check_correct_result_BM_105:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 105
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 106
//-----------------------------------------------------------

// Basic Module 106
// BSL <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM106_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM106_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM106_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM106_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM106_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM106_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM106_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM106_INPUT_VALUE_8
        ldr             x9, =A53_STL_BM106_INPUT_VALUE_9
        ldr             x10, =A53_STL_BM106_INPUT_VALUE_10
        ldr             x11, =A53_STL_BM106_INPUT_VALUE_11
        ldr             x12, =A53_STL_BM106_INPUT_VALUE_12

        bl              fmov_d1d8_x1x8
        fmov            d17, x9
        fmov            d18, x10
        fmov            d19, x11
        fmov            d20, x12

        bl              fmov_v1v8_x1x8
        fmov            v17.d[1], x9
        fmov            v18.d[1], x10
        fmov            v19.d[1], x11
        fmov            v20.d[1], x12

        // execute ext instruction

        bsl             v17.16b, v1.16b, v2.16b
        bsl             v18.16b, v3.16b, v4.16b

        bsl             v19.16b, v5.16b, v6.16b
        bsl             v20.16b, v7.16b, v8.16b

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
        ldr             x27, =A53_STL_BM106_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM106_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM106_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM106_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

check_correct_result_BM_106:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 106
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 107
//-----------------------------------------------------------

// Basic Module 107
// SRHADD <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM107_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM107_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM107_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM107_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM107_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM107_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM107_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM107_INPUT_VALUE_8

        bl              fmov_d1d8_x1x8

        bl              fmov_v1v8_x1x8

        // execute srhadd instruction

        srhadd          v17.4s, v1.4s, v2.4s
        srhadd          v18.4s, v3.4s, v4.4s

        srhadd          v19.4s, v5.4s, v6.4s
        srhadd          v20.4s, v7.4s, v8.4s

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
        ldr             x27, =A53_STL_BM107_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM107_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM107_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM107_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

check_correct_result_BM_107:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 107
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 108
//-----------------------------------------------------------

// Basic Module 108
// SQRDMULH <Vd>.<T>, <Vn>.<T>, <Vm>.<Ts>[<index>]

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM108_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM108_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM108_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM108_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM108_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM108_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM108_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM108_INPUT_VALUE_8
        ldr             x9, =A53_STL_BM108_INPUT_VALUE_9
        ldr             x10, =A53_STL_BM108_INPUT_VALUE_10

        bl              fmov_d1d8_x1x8
        fmov            d9, x9
        fmov            d10, x10

        bl              fmov_v1v8_x1x8
        fmov            v9.d[1], x9
        fmov            v10.d[1], x10

        // clean FPSR register
        msr             fpsr, xzr

        sqrdmulh        v20.4s, v1.4s, v2.s[0]
        sqrdmulh        v18.4s, v3.4s, v4.s[1]
        sqrdmulh        v19.4s, v5.4s, v6.s[2]

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM108_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        sqrdmulh        v21.4s, v7.4s, v8.s[3]
        sqrdmulh        v22.4s, v9.4s, v10.s[0]
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM108_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // Copy v20 in x11 and x15 for signature check
        fmov            x11, d20
        fmov            x15, v20.d[1]
        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d18
        fmov            x16, v18.d[1]

        // Copy v19 in x13 and x17 for signature check
        fmov            x13, d19
        fmov            x17, v19.d[1]

        // Copy v21 in x14 and x18 for signature check
        fmov            x14, d21
        fmov            x18, v21.d[1]

        // Copy v22 in x20 and x21 for signature check
        fmov            x20, d22
        fmov            x21, v22.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM108_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM108_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM108_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM108_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM108_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

check_correct_result_BM_108:

        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 108
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

a53_stl_core_p032_n001_end:

        ret

        .end
