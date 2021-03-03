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
//      CORE_031
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p031_n001
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

        .section .section_a53_stl_core_p031_n001,"ax",%progbits
        .global a53_stl_core_p031_n001
        .type a53_stl_core_p031_n001, %function

a53_stl_core_p031_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 95
//-----------------------------------------------------------

// Basic Module 95
// TBX <Vd>.<Ta>, { <Vn>.16B, <Vn+1>.16B, <Vn+2>.16B, <Vn+3>.16B }, <Vm>.<Ta>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x6 with the test values
        ldr             x1, =A53_STL_BM95_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM95_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM95_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM95_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM95_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM95_INPUT_VALUE_6

        fmov            d1, x1
        fmov            d2, xzr
        fmov            d3, x3
        fmov            d4, x4
        fmov            d6, x6

        fmov            v1.d[1], x2
        fmov            v2.d[1], xzr
        fmov            v3.d[1], x3
        fmov            v4.d[1], x5
        fmov            v6.d[1], x6

        // execute tbx instruction

        add             v10.16b, v1.16b, v2.16b
        add             v11.16b, v10.16b, v3.16b
        add             v12.16b, v11.16b, v3.16b
        add             v13.16b, v12.16b, v3.16b
        add             v14.16b, v13.16b, v3.16b
        add             v15.16b, v14.16b, v3.16b
        add             v16.16b, v15.16b, v3.16b
        add             v17.16b, v16.16b, v3.16b
        add             v18.16b, v17.16b, v3.16b
        add             v19.16b, v18.16b, v3.16b
        add             v20.16b, v19.16b, v3.16b
        add             v21.16b, v20.16b, v3.16b
        add             v22.16b, v21.16b, v3.16b
        add             v23.16b, v22.16b, v3.16b
        add             v24.16b, v23.16b, v3.16b
        add             v7.16b, v4.16b, v6.16b
        add             v8.16b, v7.16b, v6.16b
        add             v9.16b, v8.16b, v6.16b

        tbx             v26.16b, {v10.16b, v11.16b, v12.16b, v13.16b}, v4.16b
        tbx             v27.16b, {v14.16b, v15.16b, v16.16b, v17.16b}, v7.16b

        tbx             v28.16b, {v18.16b, v19.16b, v20.16b, v21.16b}, v8.16b
        tbx             v29.16b, {v22.16b, v23.16b, v24.16b, v25.16b}, v9.16b

        // Copy v26 in x11 and x15 for signature check
        fmov            x11, d26
        fmov            x15, v26.d[1]
        // Copy v27 in x12 and x16 for signature check
        fmov            x12, d27
        fmov            x16, v27.d[1]

        // Copy v28 in x13 adn x17 for signature check
        fmov            x13, d28
        fmov            x17, v28.d[1]

        // Copy v29 in x14 and x18 for signature check
        fmov            x14, d29
        fmov            x18, v29.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM95_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM95_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM95_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM95_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM95_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM95_GOLDEN_VALUE_6

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM95_GOLDEN_VALUE_7

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM95_GOLDEN_VALUE_8

check_correct_result_BM_95:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 95
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 96
//-----------------------------------------------------------

// Basic Module 96
// SCVTF <Vd>.<T>, <Vn>.<T>, #<fbits>

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Save the value of fpsr in x10
        mrs             x10, fpsr

        // Set FPSR to 0x0 for rounding to nearest rounding mode
        msr             fpsr, xzr

        ldr             x1, =A53_STL_BM96_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM96_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM96_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM96_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM96_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM96_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM96_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM96_INPUT_VALUE_8

        bl              fmov_d1d8_x1x8

        bl              fmov_v1v8_x1x8

        // clean FPSR register
        msr             fpsr, xzr

        scvtf           v20.2d, v3.2d, A53_STL_BM96_INPUT_VALUE_2
        scvtf           v18.2d, v7.2d, A53_STL_BM96_INPUT_VALUE_6
        scvtf           v19.2d, v1.2d, A53_STL_BM96_INPUT_VALUE_8

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM96_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        scvtf           v21.2d, v7.2d, A53_STL_BM96_INPUT_VALUE_8
        scvtf           v22.2d, v5.2d, A53_STL_BM96_INPUT_VALUE_4
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM96_FPSR_VALUE_1

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
        ldr             x27, =A53_STL_BM96_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM96_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM96_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM96_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM96_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

check_correct_result_BM_96:
        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

        // Restore the value of fpsr from x10
        msr             fpsr, x10

//-----------------------------------------------------------
// END BASIC MODULE 96
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 97
//-----------------------------------------------------------

// Basic Module 97
// FRINTN <Vd>.<T>, <Vn>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM97_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM97_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM97_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM97_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM97_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM97_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM97_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM97_INPUT_VALUE_8

        bl              fmov_d1d8_x1x8

        bl              fmov_v1v8_x1x8

        // clean FPSR register
        msr             fpsr, xzr

        frintn          v20.2d, v3.2d
        frintn          v18.2d, v7.2d
        frintn          v19.2d, v1.2d

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM97_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        frintn          v21.2d, v7.2d
        frintn          v22.2d, v5.2d
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM97_FPSR_VALUE_1

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
        ldr             x27, =A53_STL_BM97_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM97_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM97_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM97_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM97_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

check_correct_result_BM_97:
        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 97
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 98
//-----------------------------------------------------------

// Basic Module 98
// FCSEL <Dd>, <Dn>, <Dm>, <cond>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of NZCV register in x10
        mrs             x10, nzcv
        // Set nzcv to 0x0
        msr             nzcv, xzr

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM98_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM98_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM98_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM98_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        fmov            d17, xzr
        fmov            d18, xzr
        fmov            d19, xzr
        fmov            d20, xzr

        // execute fcsel instruction

        fcsel           d17, d2, d1, gt
        fcsel           d18, d8, d7, eq

        fcsel           d19, d6, d5, ls
        fcsel           d20, d3, d4, vc

        // Copy v17 in x11 for signature check
        fmov            x11, d17
        // Copy v18 in x12 for signature check
        fmov            x12, d18

        // Copy v19 in x13 for signature check
        fmov            x13, d19

        // Copy v20 in x14 for signature check
        fmov            x14, d20

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM98_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM98_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM98_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM98_GOLDEN_VALUE_4

check_correct_result_BM_98:
        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Restore the value of NZCV from x10
        msr             nzcv, x10

//-----------------------------------------------------------
// END BASIC MODULE 98
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 99
//-----------------------------------------------------------

// Basic Module 99
// REV64 <Vd>.<T>, <Vn>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM99_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM99_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM99_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM99_INPUT_VALUE_4

        fmov            d1, x1
        fmov            d2, x3
        fmov            d3, x1
        fmov            d4, x3

        fmov            v1.d[1], x2
        fmov            v2.d[1], x4
        fmov            v3.d[1], x2
        fmov            v4.d[1], x4

        // execute rev64 instruction

        rev64           v17.4s, v1.4s
        rev64           v18.4s, v2.4s

        rev64           v19.4s, v3.4s
        rev64           v20.4s, v4.4s

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
        ldr             x27, =A53_STL_BM99_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM99_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM99_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM99_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM99_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM99_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM99_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM99_GOLDEN_VALUE_4

check_correct_result_BM_99:
        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 99
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 100
//-----------------------------------------------------------

// Basic Module 100
// XTN{2} <Vd>.<Tb>, <Vn>.<Ta>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM100_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM100_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM100_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM100_INPUT_VALUE_4

        fmov            d1, x1
        fmov            d2, x3
        fmov            d3, x1
        fmov            d4, x3

        fmov            v1.d[1], x2
        fmov            v2.d[1], x4
        fmov            v3.d[1], x2
        fmov            v4.d[1], x4

        // execute xtn instruction

        xtn             v17.2s, v1.2d
        xtn             v18.2s, v2.2d

        xtn             v19.2s, v3.2d
        xtn             v20.2s, v4.2d

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
        ldr             x27, =A53_STL_BM100_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM100_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM100_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM100_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM100_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM100_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM100_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM100_GOLDEN_VALUE_3

check_correct_result_BM_100:
        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 100
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 101
//-----------------------------------------------------------

// Basic Module 101
// EXT <Vd>.<T>, <Vn>.<T>, <Vm>.<T>, #<index>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM101_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM101_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM101_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM101_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM101_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM101_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM101_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM101_INPUT_VALUE_8

        fmov            d1, x1
        fmov            d2, x3
        fmov            d3, x1
        fmov            d4, x3
        fmov            d5, x5
        fmov            d6, x7
        fmov            d7, x5
        fmov            d8, x7

        fmov            v1.d[1], x2
        fmov            v2.d[1], x4
        fmov            v3.d[1], x2
        fmov            v4.d[1], x4
        fmov            v5.d[1], x6
        fmov            v6.d[1], x8
        fmov            v7.d[1], x6
        fmov            v8.d[1], x8

        // execute ext instruction

        ext             v17.16b, v1.16b, v2.16b, A53_STL_BM101_INPUT_VALUE_9
        ext             v18.16b, v3.16b, v4.16b, A53_STL_BM101_INPUT_VALUE_9

        ext             v19.16b, v5.16b, v6.16b, A53_STL_BM101_INPUT_VALUE_10
        ext             v20.16b, v7.16b, v8.16b, A53_STL_BM101_INPUT_VALUE_10

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
        ldr             x27, =A53_STL_BM101_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM101_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM101_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM101_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM101_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM101_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM101_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM101_GOLDEN_VALUE_4

check_correct_result_BM_101:
        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 101
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

a53_stl_core_p031_n001_end:

        ret

        .end
