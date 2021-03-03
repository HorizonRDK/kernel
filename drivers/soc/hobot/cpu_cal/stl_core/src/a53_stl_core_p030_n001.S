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
//      CORE_030
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p030_n001
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

        .section .section_a53_stl_core_p030_n001,"ax",%progbits
        .global a53_stl_core_p030_n001
        .type a53_stl_core_p030_n001, %function

a53_stl_core_p030_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 89
//-----------------------------------------------------------

// Basic Module 89
// SMINP <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM89_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM89_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM89_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM89_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute sminp instruction

        sminp           v17.8h, v1.8h, v5.8h
        sminp           v18.8h, v3.8h, v7.8h

        sminp           v19.8h, v2.8h, v6.8h
        sminp           v20.8h, v4.8h, v8.8h

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
        ldr             x27, =A53_STL_BM89_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM89_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error


        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM89_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM89_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

check_correct_result_BM_89:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 89
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 90
//-----------------------------------------------------------

// Basic Module 90
// FMULX <Vd>.<T>, <Vn>.<T>, <Vm>.<T>
        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM90_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM90_INPUT_VALUE_4
        ldr             x3, =A53_STL_BM90_INPUT_VALUE_5
        ldr             x4, =A53_STL_BM90_INPUT_VALUE_6
        ldr             x5, =A53_STL_BM90_INPUT_VALUE_2
        ldr             x6, =A53_STL_BM90_INPUT_VALUE_3
        ldr             x7, =A53_STL_BM90_INPUT_VALUE_3
        ldr             x8, =A53_STL_BM90_INPUT_VALUE_4
        ldr             x9, =A53_STL_BM90_INPUT_VALUE_7
        ldr             x10, =A53_STL_BM90_INPUT_VALUE_7

        fmov            d1, x1
        fmov            d2, x2
        fmov            d3, x9
        fmov            d4, x10
        fmov            d5, x3
        fmov            d6, x4
        fmov            d7, x5
        fmov            d8, x6

        fmov            v1.d[1], x1
        fmov            v2.d[1], x2
        fmov            v3.d[1], x9
        fmov            v4.d[1], x10
        fmov            v5.d[1], x3
        fmov            v6.d[1], x4
        fmov            v7.d[1], x5
        fmov            v8.d[1], x6

        // FMULX <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // clean FPSR register
        msr             fpsr, xzr

        fmulx           v20.2d, v7.2d, v8.2d
        fmulx           v18.2d, v1.2d, v2.2d
        fmulx           v19.2d, v5.2d, v6.2d

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM90_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        fmulx           v21.2d, v1.2d, v5.2d

        fmulx           v22.2d, v3.2d, v4.2d
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM90_FPSR_VALUE_2

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
        ldr             x27, =A53_STL_BM90_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM90_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM90_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM90_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM90_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

check_correct_result_BM_90:
        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 90
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 91
//-----------------------------------------------------------

// Basic Module 91
// TRN1 <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM91_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM91_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM91_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM91_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute trn1 instruction

        trn1            v17.8h, v1.8h, v5.8h
        trn1            v18.8h, v3.8h, v7.8h

        trn1            v19.8h, v2.8h, v6.8h
        trn1            v20.8h, v4.8h, v8.8h

        // Copy v17 in x11 and x15 for signature check
        fmov            x11, d17
        fmov            x15, v17.d[1]
        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d18
        fmov            x16, v18.d[1]

        // Copy v19 in x13 adn x17 for signature check
        fmov            x13, d19
        fmov            x17, v19.d[1]

        // Copy v20 in x14 and x18 for signature check
        fmov            x14, d20
        fmov            x18, v20.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM91_GOLDEN_VALUE_1

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
        ldr             x28, =A53_STL_BM91_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

check_correct_result_BM_91:
        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 91
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 92
//-----------------------------------------------------------

// Basic Module 92
// UQRSHL <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM92_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM92_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM92_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM92_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM92_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM92_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM92_INPUT_VALUE_1
        ldr             x8, =A53_STL_BM92_INPUT_VALUE_2
        ldr             x9, =A53_STL_BM92_INPUT_VALUE_7
        ldr             x10, =A53_STL_BM92_INPUT_VALUE_8


        fmov            d1, x1
        fmov            d2, x2
        fmov            d3, x9
        fmov            d4, x10
        fmov            d5, x3
        fmov            d6, x4
        fmov            d7, x5
        fmov            d8, x6

        fmov            v1.d[1], x1
        fmov            v2.d[1], x2
        fmov            v3.d[1], x9
        fmov            v4.d[1], x10
        fmov            v5.d[1], x3
        fmov            v6.d[1], x4
        fmov            v7.d[1], x5
        fmov            v8.d[1], x6

        // clean FPSR register
        msr             fpsr, xzr

        uqrshl          v20.2d, v1.2d, v2.2d
        uqrshl          v18.2d, v5.2d, v6.2d
        uqrshl          v19.2d, v7.2d, v8.2d

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM92_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        uqrshl          v21.2d, v1.2d, v8.2d
        uqrshl          v22.2d, v3.2d, v4.2d
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM92_FPSR_VALUE_1

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
        ldr             x27, =A53_STL_BM92_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM92_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM92_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM92_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM92_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

check_correct_result_BM_92:
        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 92
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 93
//-----------------------------------------------------------

// Basic Module 93
// FDIV <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM93_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM93_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM93_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM93_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM93_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM93_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM93_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM93_INPUT_VALUE_8

        bl              fmov_d1d8_x1x8

        bl              fmov_v1v8_x1x8

        // FDIV <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // clean FPSR register
        msr             fpsr, xzr

        fdiv            v18.2d, v1.2d, v2.2d
        fdiv            v19.2d, v3.2d, v6.2d
        fdiv            v20.2d, v1.2d, v7.2d

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM93_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        fdiv            v21.2d, v1.2d, v5.2d
        fdiv            v22.2d, v8.2d, v3.2d
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM93_FPSR_VALUE_2

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // Copy v20 in x11 and x15 for signature check
        fmov            x11, d18
        fmov            x15, v18.d[1]
        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d19
        fmov            x16, v19.d[1]

        // Copy v19 in x13 and x17 for signature check
        fmov            x13, d20
        fmov            x17, v20.d[1]

        // Copy v21 in x14 and x18 for signature check
        fmov            x14, d21
        fmov            x18, v21.d[1]

        // Copy v22 in x20 and x21 for signature check
        fmov            x20, d22
        fmov            x21, v22.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM93_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM93_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM93_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM93_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM93_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        fdiv            v18.2d, v2.2d, v1.2d
        fdiv            v19.2d, v3.2d, v6.2d
        fdiv            v20.2d, v5.2d, v6.2d

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM93_FPSR_VALUE_3

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        fdiv            v21.2d, v1.2d, v5.2d
        fdiv            v22.2d, v4.2d, v8.2d
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM93_FPSR_VALUE_4

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // Copy v20 in x11 and x15 for signature check
        fmov            x11, d18
        fmov            x15, v18.d[1]
        // Copy v18 in x12 and x16 for signature check
        fmov            x12, d19
        fmov            x16, v19.d[1]

        // Copy v19 in x13 and x17 for signature check
        fmov            x13, d20
        fmov            x17, v20.d[1]

        // Copy v21 in x14 and x18 for signature check
        fmov            x14, d21
        fmov            x18, v21.d[1]

        // Copy v22 in x20 and x21 for signature check
        fmov            x20, d22
        fmov            x21, v22.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM93_GOLDEN_VALUE_6

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM93_GOLDEN_VALUE_7

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM93_GOLDEN_VALUE_8

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM93_GOLDEN_VALUE_9

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM93_GOLDEN_VALUE_10

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

check_correct_result_BM_93:
        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 93
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 94
//-----------------------------------------------------------

// Basic Module 94
// UQRSHRN{2} <Vd>.<Tb>, <Vn>.<Ta>, #<shift>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM94_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM94_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM94_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM94_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM94_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM94_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM94_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM94_INPUT_VALUE_8

        bl              fmov_d1d8_x1x8

        bl              fmov_v1v8_x1x8

        // clean FPSR register
        msr             fpsr, xzr

        uqrshrn         v20.2s, v3.2d, A53_STL_BM94_INPUT_VALUE_2
        uqrshrn         v18.2s, v7.2d, A53_STL_BM94_INPUT_VALUE_6
        uqrshrn         v19.2s, v1.2d, A53_STL_BM94_INPUT_VALUE_8

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM94_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        uqrshrn         v21.2s, v7.2d, A53_STL_BM94_INPUT_VALUE_8
        uqrshrn         v22.2s, v5.2d, A53_STL_BM94_INPUT_VALUE_4
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM94_FPSR_VALUE_1

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
        ldr             x27, =A53_STL_BM94_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM94_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM94_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error


        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM94_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM94_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM94_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM94_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM94_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM94_GOLDEN_VALUE_1

check_correct_result_BM_94:
        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 94
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

a53_stl_core_p030_n001_end:

        ret

        .end
