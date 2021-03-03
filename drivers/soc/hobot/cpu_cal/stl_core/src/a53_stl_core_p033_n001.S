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
//      CORE_033
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p033_n001
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

        .section .section_a53_stl_core_p033_n001,"ax",%progbits
        .global a53_stl_core_p033_n001
        .type a53_stl_core_p033_n001, %function

a53_stl_core_p033_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 109
//-----------------------------------------------------------

// Basic Module 109
// UZP1 <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM109_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM109_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM109_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM109_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute uzp1 instruction

        uzp1            v17.16b, v1.16b, v5.16b
        uzp1            v18.16b, v3.16b, v7.16b

        uzp1            v19.16b, v2.16b, v6.16b
        uzp1            v20.16b, v4.16b, v8.16b

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
        ldr             x27, =A53_STL_BM109_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM109_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM109_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM109_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

check_correct_result_BM_109:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 109
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 110
//-----------------------------------------------------------

// Basic Module 110
// ZIP2 <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM110_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM110_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM110_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM110_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute zip2 instruction

        zip2            v17.4s, v1.4s, v5.4s
        zip2            v18.4s, v3.4s, v7.4s

        zip2            v19.4s, v2.4s, v6.4s
        zip2            v20.4s, v4.4s, v8.4s

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
        ldr             x27, =A53_STL_BM110_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM110_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM110_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM110_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

check_correct_result_BM_110:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 110
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 111
//-----------------------------------------------------------

// Basic Module 111
// DUP <Vd>.<T>, <Vn>.<Ts>[<index>]

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM111_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM111_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM111_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM111_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute dup instruction

        dup             v17.16b, v1.b[5]
        dup             v18.8h, v3.h[4]

        dup             v19.4s, v5.s[3]
        dup             v20.2d, v6.d[0]

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
        ldr             x27, =A53_STL_BM111_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM111_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM111_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM111_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

check_correct_result_BM_111:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 111
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 112
//-----------------------------------------------------------

// Basic Module 112
// EOR <Vd>.<T>, <Vn>.<T>, <Vm>.<T>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM112_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM112_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM112_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM112_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute eor instruction

        eor             v17.16b, v1.16b, v2.16b
        eor             v18.16b, v4.16b, v5.16b

        eor             v19.16b, v3.16b, v7.16b
        eor             v20.16b, v6.16b, v8.16b

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
        ldr             x27, =A53_STL_BM112_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM112_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM112_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM112_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

check_correct_result_BM_112:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 112
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 113
//-----------------------------------------------------------

// Basic Module 113
// SQDMLSL{2} <Vd>.<Ta>, <Vn>.<Tb>, <Vm>.<Ts>[<index>]

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM113_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM113_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM113_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM113_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM113_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM113_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM113_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM113_INPUT_VALUE_8
        ldr             x9, =A53_STL_BM113_INPUT_VALUE_9
        ldr             x10, =A53_STL_BM113_INPUT_VALUE_10

        bl              fmov_d1d8_x1x8
        fmov            d9, x9
        fmov            d10, x10
        fmov            d18, xzr
        fmov            d19, xzr
        fmov            d20, xzr
        fmov            d21, xzr
        fmov            d22, xzr

        bl              fmov_v1v8_x1x8
        fmov            v9.d[1], x9
        fmov            v10.d[1], x10
        fmov            v18.d[1], xzr
        fmov            v19.d[1], xzr
        fmov            v20.d[1], xzr
        fmov            v21.d[1], xzr
        fmov            v22.d[1], xzr

        // clean FPSR register
        msr             fpsr, xzr

        sqdmlsl         v20.2d, v1.2s, v2.s[0]
        sqdmlsl         v18.4s, v3.4h, v4.h[3]
        sqdmlsl         v19.2d, v5.2s, v6.s[1]

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM113_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        sqdmlsl         v21.2d, v7.2s, v8.s[2]
        sqdmlsl         v22.4s, v9.4h, v10.h[4]
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM113_FPSR_VALUE_1

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
        ldr             x27, =A53_STL_BM113_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM113_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM113_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM113_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM113_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

check_correct_result_BM_113:

        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 113
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 114
//-----------------------------------------------------------

// Basic Module 114
// AND <Vd>.<T>, <Vn>.<T>, <Vm>.<T>
        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x4 with the test values
        ldr             x1, =A53_STL_BM114_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM114_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM114_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM114_INPUT_VALUE_4

        bl              fmov_d1d8_x1x4

        bl              fmov_v1v8_x1x4

        // execute and instruction

        and             v17.8b, v1.8b, v2.8b
        and             v18.8b, v4.8b, v5.8b

        and             v19.8b, v3.8b, v7.8b
        and             v20.8b, v6.8b, v8.8b

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
        ldr             x27, =A53_STL_BM114_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM114_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM114_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM114_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM114_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM114_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM114_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM114_GOLDEN_VALUE_5

check_correct_result_BM_114:

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 114
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 115
//-----------------------------------------------------------

// Basic Module 115
// FADDP <Vd>.<T>, <Vn>.<T>, <Vm>.<T>
        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        ldr             x1, =A53_STL_BM115_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM115_INPUT_VALUE_2
        ldr             x3, =A53_STL_BM115_INPUT_VALUE_3
        ldr             x4, =A53_STL_BM115_INPUT_VALUE_4
        ldr             x5, =A53_STL_BM115_INPUT_VALUE_5
        ldr             x6, =A53_STL_BM115_INPUT_VALUE_6
        ldr             x7, =A53_STL_BM115_INPUT_VALUE_7
        ldr             x8, =A53_STL_BM115_INPUT_VALUE_8
        ldr             x9, =A53_STL_BM115_INPUT_VALUE_9
        ldr             x10, =A53_STL_BM115_INPUT_VALUE_10

        bl              fmov_d1d8_x1x8
        fmov            d9, x9
        fmov            d10, x10

        bl              fmov_v1v8_x1x8
        fmov            v9.d[1], x9
        fmov            v10.d[1], x10

        // clean FPSR register
        msr             fpsr, xzr

        faddp           v20.2d, v1.2d, v2.2d
        faddp           v18.2d, v9.2d, v10.2d
        faddp           v19.2d, v3.2d, v4.2d

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM115_FPSR_VALUE_1

        // Compare local and golden flags
        cmp             x0, x1
        b.ne            error

        // clean FPSR register
        msr             fpsr, xzr

        faddp           v21.2d, v7.2d, v8.2d
        faddp           v22.2d, v5.2d, v6.2d
        fmov            v23.d[1], x4

        // read the value of FPSR
        mrs             x0, fpsr
        ldr             x1, =A53_STL_BM115_FPSR_VALUE_2

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
        ldr             x27, =A53_STL_BM115_GOLDEN_VALUE_1

        // Compare local and golden signature
        cmp             x11, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM115_GOLDEN_VALUE_2

        // Compare local and golden signature
        cmp             x15, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM115_GOLDEN_VALUE_3

        // Compare local and golden signature
        cmp             x12, x27
        b.ne            error

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM115_GOLDEN_VALUE_4

        // Compare local and golden signature
        cmp             x16, x27
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM115_GOLDEN_VALUE_5

        // Compare local and golden signature
        cmp             x13, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM115_GOLDEN_VALUE_6

        // Compare local and golden signature
        cmp             x17, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM115_GOLDEN_VALUE_7

        // Compare local and golden signature
        cmp             x14, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM115_GOLDEN_VALUE_8

        // Compare local and golden signature
        cmp             x18, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM115_GOLDEN_VALUE_9

        // Compare local and golden signature
        cmp             x20, x28
        b.ne            error

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM115_GOLDEN_VALUE_10

check_correct_result_BM_115:

        // Compare local and golden signature
        cmp             x21, x28
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 115
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

a53_stl_core_p033_n001_end:

        ret

        .end
