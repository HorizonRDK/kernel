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
//      Testing of LDR/STR (SIMD and FP) instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_FPU, AI_MEM
//
//   About: TEST_ID
//      CORE_049
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p049_n001
//      Testing of LDR/STR (SIMD and FP) instructions
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

        .section .section_a53_stl_core_p049_n001,"ax",%progbits
        .global a53_stl_core_p049_n001
        .type a53_stl_core_p049_n001, %function

a53_stl_core_p049_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping


//-----------------------------------------------------------
// START BASIC MODULE 28
//-----------------------------------------------------------

// Basic Module 28
// ST4 { <Vt>.<T>, <Vt2>.<T>, <Vt3>.<T>, <Vt4>.<T> }, [<Xn|SP>], <Xm>
// LD4 { <Vt>.<T>, <Vt2>.<T>, <Vt3>.<T>, <Vt4>.<T> }, [<Xn|SP>], <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x8 with the test values
        mov             x1, A53_STL_BM28_INPUT_VALUE_1
        mov             x2, A53_STL_BM28_INPUT_VALUE_2
        mov             x3, A53_STL_BM28_INPUT_VALUE_3
        mov             x4, A53_STL_BM28_INPUT_VALUE_4
        mov             x5, A53_STL_BM28_INPUT_VALUE_5
        mov             x6, A53_STL_BM28_INPUT_VALUE_6
        mov             x7, A53_STL_BM28_INPUT_VALUE_7
        mov             x8, A53_STL_BM28_INPUT_VALUE_8

        bl              fmov_d1d8_x1x2x3x4

        fmov            d9, x5
        fmov            d10, x6
        fmov            d11, x7
        fmov            d12, x8
        fmov            d13, x5
        fmov            d14, x6
        fmov            d15, x7
        fmov            d16, x8

        fmov            v1.d[1], x1
        fmov            v2.d[1], x2
        fmov            v3.d[1], x3
        fmov            v4.d[1], x4
        fmov            v5.d[1], x1
        fmov            v6.d[1], x2
        fmov            v7.d[1], x3
        fmov            v8.d[1], x4

        fmov            v9.d[1], x5
        fmov            v10.d[1], x6
        fmov            v11.d[1], x7
        fmov            v12.d[1], x8
        fmov            v13.d[1], x5
        fmov            v14.d[1], x6
        fmov            v15.d[1], x7
        fmov            v16.d[1], x8

        // Decrease SP value to store registers from v1 to v8
        sub             sp, sp, A53_STL_BM28_STK_PTR_352_B

        // save value of SP to use it after store operations
        mov             x10, sp
        // init x0 with offset value
        mov             x0, A53_STL_BM28_INIT_OFFSET_40

        // Store values from registers to SP
        st4             {v1.2d, v2.2d, v3.2d, v4.2d}, [sp], x0
        st4             {v5.2d, v6.2d, v7.2d, v8.2d}, [sp], x0

        st4             {v9.2d, v10.2d, v11.2d, v12.2d}, [sp], x0
        st4             {v13.2d, v14.2d, v15.2d, v16.2d}, [sp]

        // restore value of SP
        mov             sp, x10

        // Load values from SP

        ld4             {v17.2d, v18.2d, v19.2d, v20.2d}, [sp], x0
        ld4             {v21.2d, v22.2d, v23.2d, v24.2d}, [sp], x0

        ld4             {v25.2d, v26.2d, v27.2d, v28.2d}, [sp], x0
        ld4             {v29.2d, v30.2d, v31.2d, v0.2d}, [sp]

        // restore value of SP
        mov             sp, x10

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
        // Copy v21 in x19 and x20 for signature check
        fmov            x19, d21
        fmov            x20, v21.d[1]
        // Copy v22 in x21 and x22 for signature check
        fmov            x21, d22
        fmov            x22, v22.d[1]
        // Copy v23 in x23 and x24 for signature check
        fmov            x23, d23
        fmov            x24, v23.d[1]
        // Copy v24 in x25 and x26 for signature check
        fmov            x25, d24
        fmov            x26, v24.d[1]

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM28_GOLDEN_VALUE_1

        // Check if the loaded value is the expected value in SP
        cmp             x11, x27
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x12, x27
        b.ne            error_BM_28

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM28_GOLDEN_VALUE_2

        // Check if the loaded value is the expected value in SP
        cmp             x13, x28
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x14, x28
        b.ne            error_BM_28

        // Load in x29 the golden value
        ldr             x29, =A53_STL_BM28_GOLDEN_VALUE_3

        // Check if the loaded value is the expected value in SP
        cmp             x15, x29
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x16, x29
        b.ne            error_BM_28

        // Load in x0 the golden value
        ldr             x0, =A53_STL_BM28_GOLDEN_VALUE_4

        // Check if the loaded value is the expected value in SP
        cmp             x17, x0
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x18, x0
        b.ne            error_BM_28


        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM28_GOLDEN_VALUE_1

        // Check if the loaded value is the expected value in SP
        cmp             x19, x27
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x20, x27
        b.ne            error_BM_28

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM28_GOLDEN_VALUE_2

        // Check if the loaded value is the expected value in SP
        cmp             x21, x28
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x22, x28
        b.ne            error_BM_28

        // Load in x29 the golden value
        ldr             x29, =A53_STL_BM28_GOLDEN_VALUE_3

        // Check if the loaded value is the expected value in SP
        cmp             x23, x29
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x24, x29
        b.ne            error_BM_28

        // Load in x0 the golden value
        ldr             x0, =A53_STL_BM28_GOLDEN_VALUE_4

        // Check if the loaded value is the expected value in SP
        cmp             x25, x0
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x26, x0
        b.ne            error_BM_28


        // Copy v25 in x11 and x12 for signature check
        fmov            x11, d25
        fmov            x12, v25.d[1]
        // Copy v26 in x13 and x14 for signature check
        fmov            x13, d26
        fmov            x14, v26.d[1]
        // Copy v27 in x15 and x16 for signature check
        fmov            x15, d27
        fmov            x16, v27.d[1]
        // Copy v28 in x17 and x18 for signature check
        fmov            x17, d28
        fmov            x18, v28.d[1]
        // Copy v29 in x19 and x20 for signature check
        fmov            x19, d29
        fmov            x20, v29.d[1]
        // Copy v30 in x21 and x22 for signature check
        fmov            x21, d30
        fmov            x22, v30.d[1]
        // Copy v31 in x23 and x24 for signature check
        fmov            x23, d31
        fmov            x24, v31.d[1]
        // Copy v0 in x25 and x26 for signature check
        fmov            x25, d0
        fmov            x26, v0.d[1]


        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM28_GOLDEN_VALUE_5

        // Check if the loaded value is the expected value in SP
        cmp             x11, x27
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x12, x27
        b.ne            error_BM_28

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM28_GOLDEN_VALUE_6

        // Check if the loaded value is the expected value in SP
        cmp             x13, x28
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x14, x28
        b.ne            error_BM_28

        // Load in x29 the golden value
        ldr             x29, =A53_STL_BM28_GOLDEN_VALUE_7

        // Check if the loaded value is the expected value in SP
        cmp             x15, x29
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x16, x29
        b.ne            error_BM_28

        // Load in x0 the golden value
        ldr             x0, =A53_STL_BM28_GOLDEN_VALUE_8

        // Check if the loaded value is the expected value in SP
        cmp             x17, x0
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x18, x0
        b.ne            error_BM_28


        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM28_GOLDEN_VALUE_5

        // Check if the loaded value is the expected value in SP
        cmp             x19, x27
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x20, x27
        b.ne            error_BM_28

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM28_GOLDEN_VALUE_6

        // Check if the loaded value is the expected value in SP
        cmp             x21, x28
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x22, x28
        b.ne            error_BM_28

        // Load in x29 the golden value
        ldr             x29, =A53_STL_BM28_GOLDEN_VALUE_7

        // Check if the loaded value is the expected value in SP
        cmp             x23, x29
        b.ne            error_BM_28

        // Check if the loaded value is the expected value in SP
        cmp             x24, x29
        b.ne            error_BM_28

        // Load in x0 the golden value
        ldr             x0, =A53_STL_BM28_GOLDEN_VALUE_8

        // Check if the loaded value is the expected value in SP
        cmp             x25, x0
        b.ne            error_BM_28

check_correct_result_BM_28:

        // Check if the loaded value is the expected value in SP
        cmp             x26, x0
        b.ne            error_BM_28
        b               success_BM_28

error_BM_28:
        // Restore the original value of SP
        add             sp, sp, A53_STL_BM28_STK_PTR_352_B
        b               error

success_BM_28:
        // Restore the original value of SP
        add             sp, sp, A53_STL_BM28_STK_PTR_352_B

//-----------------------------------------------------------
// END BASIC MODULE 28
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 29
//-----------------------------------------------------------

// Basic Module 29
// ST3 { <Vt>.<T>, <Vt2>.<T>, <Vt3>.<T> }, [<Xn|SP>], <Xm>
// LD3 { <Vt>.<T>, <Vt2>.<T>, <Vt3>.<T> }, [<Xn|SP>], <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // This test has been performed with the following values for Float control registers:
        // FPCR -> 0x00000000
        // FPSR -> 0x00000000

        // Initialize registers from x1 to x8 with the test values
        mov             x1, A53_STL_BM29_INPUT_VALUE_1
        mov             x2, A53_STL_BM29_INPUT_VALUE_2
        mov             x3, A53_STL_BM29_INPUT_VALUE_3
        mov             x4, A53_STL_BM29_INPUT_VALUE_4
        mov             x5, A53_STL_BM29_INPUT_VALUE_5
        mov             x6, A53_STL_BM29_INPUT_VALUE_6

        fmov            d1, x1
        fmov            d2, x2
        fmov            d3, x3
        fmov            d4, x1
        fmov            d5, x2
        fmov            d6, x3

        fmov            d7, x4
        fmov            d8, x5
        fmov            d9, x6
        fmov            d10, x4
        fmov            d11, x5
        fmov            d12, x6

        fmov            v1.d[1], x1
        fmov            v2.d[1], x2
        fmov            v3.d[1], x3
        fmov            v4.d[1], x1
        fmov            v5.d[1], x2
        fmov            v6.d[1], x3

        fmov            v7.d[1], x4
        fmov            v8.d[1], x5
        fmov            v9.d[1], x6
        fmov            v10.d[1], x4
        fmov            v11.d[1], x5
        fmov            v12.d[1], x6

        // Decrease SP value to store registers from v1 to v8
        sub             sp, sp, A53_STL_BM29_STK_PTR_288_B
        // save value of SP to use it after store operations
        mov             x10, sp
        // init x0 with offset value
        mov             x0, A53_STL_BM29_INIT_OFFSET_30

        // Store values from registers to SP
        st3             {v1.2d, v2.2d, v3.2d}, [sp], x0
        st3             {v4.2d, v5.2d, v6.2d}, [sp], x0

        st3             {v7.2d, v8.2d, v9.2d}, [sp], x0
        st3             {v10.2d, v11.2d, v12.2d}, [sp]

        // restore value of SP
        mov             sp, x10

        // Load values from SP

        ld3             {v17.2d, v18.2d, v19.2d}, [sp], x0
        ld3             {v20.2d, v21.2d, v22.2d}, [sp], x0

        ld3             {v23.2d, v24.2d, v25.2d}, [sp], x0
        ld3             {v29.2d, v30.2d, v31.2d}, [sp]

        // restore value of SP
        mov             sp, x10

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
        // Copy v21 in x19 and x20 for signature check
        fmov            x19, d21
        fmov            x20, v21.d[1]
        // Copy v22 in x21 and x22 for signature check
        fmov            x21, d22
        fmov            x22, v22.d[1]


        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM29_GOLDEN_VALUE_1

        // Check if the loaded value is the expected value in SP
        cmp             x11, x27
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x12, x27
        b.ne            error_BM_29

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM29_GOLDEN_VALUE_2

        // Check if the loaded value is the expected value in SP
        cmp             x13, x28
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x14, x28
        b.ne            error_BM_29

        // Load in x29 the golden value
        ldr             x29, =A53_STL_BM29_GOLDEN_VALUE_3

        // Check if the loaded value is the expected value in SP
        cmp             x15, x29
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x16, x29
        b.ne            error_BM_29

        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM29_GOLDEN_VALUE_1

        // Check if the loaded value is the expected value in SP
        cmp             x17, x27
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x18, x27
        b.ne            error_BM_29

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM29_GOLDEN_VALUE_2

        // Check if the loaded value is the expected value in SP
        cmp             x19, x28
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x20, x28
        b.ne            error_BM_29

        // Load in x29 the golden value
        ldr             x29, =A53_STL_BM29_GOLDEN_VALUE_3

        // Check if the loaded value is the expected value in SP
        cmp             x21, x29
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x22, x29
        b.ne            error_BM_29


        // Copy v23 in x11 and x12 for signature check
        fmov            x11, d23
        fmov            x12, v23.d[1]
        // Copy v24 in x13 and x14 for signature check
        fmov            x13, d24
        fmov            x14, v24.d[1]
        // Copy v25 in x15 and x16 for signature check
        fmov            x15, d25
        fmov            x16, v25.d[1]
        // Copy v29 in x19 and x20 for signature check
        fmov            x19, d29
        fmov            x20, v29.d[1]
        // Copy v30 in x21 and x22 for signature check
        fmov            x21, d30
        fmov            x22, v30.d[1]
        // Copy v31 in x23 and x24 for signature check
        fmov            x23, d31
        fmov            x24, v31.d[1]


        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM29_GOLDEN_VALUE_4

        // Check if the loaded value is the expected value in SP
        cmp             x11, x27
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x12, x27
        b.ne            error_BM_29

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM29_GOLDEN_VALUE_5

        // Check if the loaded value is the expected value in SP
        cmp             x13, x28
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x14, x28
        b.ne            error_BM_29

        // Load in x29 the golden value
        ldr             x29, =A53_STL_BM29_GOLDEN_VALUE_6

        // Check if the loaded value is the expected value in SP
        cmp             x15, x29
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x16, x29
        b.ne            error_BM_29


        // Load in x27 the golden value
        ldr             x27, =A53_STL_BM29_GOLDEN_VALUE_4

        // Check if the loaded value is the expected value in SP
        cmp             x19, x27
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x20, x27
        b.ne            error_BM_29

        // Load in x28 the golden value
        ldr             x28, =A53_STL_BM29_GOLDEN_VALUE_5

        // Check if the loaded value is the expected value in SP
        cmp             x21, x28
        b.ne            error_BM_29

        // Check if the loaded value is the expected value in SP
        cmp             x22, x28
        b.ne            error_BM_29

        // Load in x29 the golden value
        ldr             x29, =A53_STL_BM29_GOLDEN_VALUE_6

        // Check if the loaded value is the expected value in SP
        cmp             x23, x29
        b.ne            error_BM_29

check_correct_result_BM_29:

        // Check if the loaded value is the expected value in SP
        cmp             x24, x29
        b.ne            error_BM_29
        b               success_BM_29

error_BM_29:
        // Restore the original value of SP
        add             sp, sp, A53_STL_BM29_STK_PTR_288_B
        b               error

success_BM_29:
        // Restore the original value of SP
        add             sp, sp, A53_STL_BM29_STK_PTR_288_B

//-----------------------------------------------------------
// END BASIC MODULE 29
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

a53_stl_core_p049_n001_end:

        ret

        .end
