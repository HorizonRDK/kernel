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
//      Testing of control registers
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AD-FMSK
//
//   About: TEST_ID
//      CORE_038
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p038_n001
//      Testing of control registers
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

        .section .section_a53_stl_core_p038_n001,"ax",%progbits
        .global a53_stl_core_p038_n001
        .type a53_stl_core_p038_n001, %function

a53_stl_core_p038_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]

        // call preamble subroutine
        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

        // Save D,A,I,F bits in x30
        mrs             x30, daif

a53_stl_win_c_p038:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1

//-----------------------------------------------------------
// START BASIC MODULE 353
//-----------------------------------------------------------

// Basic Module 353
// Testing TTBR0_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TTBR0_EL1 in x2
        mrs             x2, ttbr0_el1

        // initialize x27 with 0xAAAAAAAAAAAAAAAA
        ldr             x27, =A53_STL_BM353_INPUT_VALUE_1

        // write TTBR0_EL1 register
        msr             ttbr0_el1, x27

        // read back the value of TTBR0_EL1
        mrs             x26, ttbr0_el1

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM353_GOLDEN_VALUE_1

        cmp             x28, x26
        b.ne            error_BM_353

        // initialize x25 with 0x5555555555555555
        ldr             x25, =A53_STL_BM353_INPUT_VALUE_2

        // write TTBR0_EL1 register
        msr             ttbr0_el1, x25

        // read back the value of TTBR0_EL1
        mrs             x24, ttbr0_el1

        // initialize x28 with golden signature (0x5555555555555551)
        ldr             x28, =A53_STL_BM353_GOLDEN_VALUE_2

check_correct_result_BM_353:
        cmp             x28, x24
        b.ne            error_BM_353
        b               success_BM_353

error_BM_353:
        // restore TTBR0_EL1 register from x2
        msr             ttbr0_el1, x2
        b               error

success_BM_353:
        // restore TTBR0_EL1 register from x2
        msr             ttbr0_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 353
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 354
//-----------------------------------------------------------

// Basic Module 354
// Testing TTBR0_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TTBR0_EL2 in x2
        mrs             x2, ttbr0_el2

        // initialize x24 with 0xAAAAAAAAAAAAAAAA
        ldr             x24, =A53_STL_BM354_INPUT_VALUE_1

        // write TTBR0_EL2 register
        msr             ttbr0_el2, x24

        // read back the value of TTBR0_EL2
        mrs             x23, ttbr0_el2

        // initialize x28 with golden signature (0x0000AAAAAAAAAAA0)
        ldr             x28, =A53_STL_BM354_GOLDEN_VALUE_1

        cmp             x28, x23
        b.ne            error_BM_354

        // initialize x22 with 0x5555555555555555
        ldr             x22, =A53_STL_BM354_INPUT_VALUE_2

        // write TTBR0_EL2 register
        msr             ttbr0_el2, x22

        // read back the value of TTBR0_EL2
        mrs             x21, ttbr0_el2

        // initialize x28 with golden signature (0x0000555555555550)
        ldr             x28, =A53_STL_BM354_GOLDEN_VALUE_2

check_correct_result_BM_354:
        cmp             x28, x21
        b.ne            error_BM_354
        b               success_BM_354

error_BM_354:
        // restore TTBR0_EL2 register from x2
        msr             ttbr0_el2, x2
        b               error

success_BM_354:
        // restore TTBR0_EL2 register from x2
        msr             ttbr0_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 354
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 355
//-----------------------------------------------------------

// Basic Module 355
// Testing TTBR1_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TTBR1_EL1 in x2
        mrs             x2, ttbr1_el1

        // initialize x20 with 0xAAAAAAAAAAAAAAAA
        ldr             x20, =A53_STL_BM355_INPUT_VALUE_1

        // write TTBR1_EL1 register
        msr             ttbr1_el1, x20

        // read back the value of TTBR1_EL1
        mrs             x19, ttbr1_el1

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM355_GOLDEN_VALUE_1

        cmp             x28, x19
        b.ne            error_BM_355

        // initialize x18 with 0x5555555555555555
        ldr             x18, =A53_STL_BM355_INPUT_VALUE_2

        // write TTBR1_EL1 register
        msr             ttbr1_el1, x18

        // read back the value of TTBR1_EL1
        mrs             x17, ttbr1_el1

        // initialize x28 with golden signature (0x5555555555555551)
        ldr             x28, =A53_STL_BM355_GOLDEN_VALUE_2

check_correct_result_BM_355:
        cmp             x28, x17
        b.ne            error_BM_355
        b               success_BM_355

error_BM_355:
        // restore TTBR1_EL1 register from x2
        msr             ttbr1_el1, x2
        b               error

success_BM_355:
        // restore TTBR1_EL1 register from x2
        msr             ttbr1_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 355
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 356
//-----------------------------------------------------------

// Basic Module 356
// Testing VTTBR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Save the value of VTTBR_EL2 in x2
        mrs             x2, vttbr_el2

        // initialize x16 with 0xAAAAAAAAAAAAAAAA
        ldr             x16, =A53_STL_BM356_INPUT_VALUE_1

        // write VTTBR_EL2 register
        msr             vttbr_el2, x16

        // read back the value of VTTBR_EL2
        mrs             x15, vttbr_el2

        // initialize x28 with golden signature (0x00AAAAAAAAAAAAA0)
        ldr             x28, =A53_STL_BM356_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_356

        // initialize x14 with 0x5555555555555555
        ldr             x14, =A53_STL_BM356_INPUT_VALUE_2

        // write VTTBR_EL2 register
        msr             vttbr_el2, x14

        // read back the value of VTTBR_EL2
        mrs             x13, vttbr_el2

        // initialize x28 with golden signature (0x0055555555555550)
        ldr             x28, =A53_STL_BM356_GOLDEN_VALUE_2

check_correct_result_BM_356:
        cmp             x28, x13
        b.ne            error_BM_356
        b               success_BM_356

error_BM_356:
        // restore VTTBR_EL2 register from x2
        msr             vttbr_el2, x2
        b               error

success_BM_356:
        // restore VTTBR_EL2 register from x2
        msr             vttbr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 356
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 357
//-----------------------------------------------------------

// Basic Module 357
// Testing TCR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TCR_EL1 in x2
        mrs             x2, tcr_el1

        // initialize x8 with 0xAAAAAAAAAAAAAAAA
        ldr             x8, =A53_STL_BM357_INPUT_VALUE_1

        // write TCR_EL1 register
        msr             tcr_el1, x8

        // read back the value of TCR_EL1
        mrs             x7, tcr_el1

        // initialize x28 with golden signature (0x00000022AAAA2AAA)
        ldr             x28, =A53_STL_BM357_GOLDEN_VALUE_1

        cmp             x28, x7
        b.ne            error_BM_357

        // initialize x10 with 0x5555555555555555
        ldr             x10, =A53_STL_BM357_INPUT_VALUE_2

        // write TCR_EL1 register
        msr             tcr_el1, x10

        // read back the value of TCR_EL1
        mrs             x6, tcr_el1

        // initialize x28 with golden signature (0x0000005555555515)
        ldr             x28, =A53_STL_BM357_GOLDEN_VALUE_2

check_correct_result_BM_357:
        cmp             x28, x6
        b.ne            error_BM_357
        b               success_BM_357

error_BM_357:
        // restore TCR_EL1 register from x2
        msr             tcr_el1, x2
        b               error

success_BM_357:
        // restore TCR_EL1 register from x2
        msr             tcr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 357
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 364
//-----------------------------------------------------------

// Basic Module 364
// Testing TCR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TCR_EL2 in x2
        mrs             x2, tcr_el2

        // initialize x8 with 0xAAAAAAAAAAAAAAAA
        ldr             x8, =A53_STL_BM364_INPUT_VALUE_1

        // write TCR_EL2 register
        msr             tcr_el2, x8

        // read back the value of TCR_EL2
        mrs             x7, tcr_el2

        // initialize x28 with golden signature (0x80822A2A)
        ldr             x28, =A53_STL_BM364_GOLDEN_VALUE_1

        cmp             x28, x7
        b.ne            error_BM_364

        // initialize x6 with 0x5555555555555555
        ldr             x6, =A53_STL_BM364_INPUT_VALUE_2

        // write TCR_EL2 register
        msr             tcr_el2, x6

        // read back the value of TCR_EL2
        mrs             x5, tcr_el2

        // initialize x28 with golden signature (0x80955515)
        ldr             x28, =A53_STL_BM364_GOLDEN_VALUE_2

check_correct_result_BM_364:
        cmp             x28, x5
        b.ne            error_BM_364
        b               success_BM_364

error_BM_364:
        // restore TCR_EL2 register from x2
        msr             tcr_el2, x2
        b               error

success_BM_364:
        // restore TCR_EL2 register from x2
        msr             tcr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 364
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 243
//-----------------------------------------------------------

// Basic Module 243
// Testing VTCR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of VTCR_EL2 in x2
        mrs             x2, vtcr_el2

        // initialize x8 with 0xAAAAAAAAAAAAAAAA
        ldr             x8, =A53_STL_BM243_INPUT_VALUE_1

        // write VTCR_EL2 register
        msr             vtcr_el2, x8

        // read back the value of VTCR_EL2
        mrs             x7, vtcr_el2

        // initialize x28 with golden signature (0x80022AAA)
        ldr             x28, =A53_STL_BM243_GOLDEN_VALUE_1

        cmp             x28, x7
        b.ne            error_BM_243

        // initialize x6 with 0x5555555555555555
        ldr             x6, =A53_STL_BM243_INPUT_VALUE_2

        // write VTCR_EL2 register
        msr             vtcr_el2, x6

        // read back the value of VTCR_EL2
        mrs             x5, vtcr_el2

        // initialize x28 with golden signature (0x80055555)
        ldr             x28, =A53_STL_BM243_GOLDEN_VALUE_2

check_correct_result_BM_243:
        cmp             x28, x5
        b.ne            error_BM_243
        b               success_BM_243

error_BM_243:
        // restore VTCR_EL2 register from x2
        msr             vtcr_el2, x2
        b               error

success_BM_243:
        // restore VTCR_EL2 register from x2
        msr             vtcr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 243
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 247
//-----------------------------------------------------------

// Basic Module 247
// Testing PAR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of PAR_EL1 in x5
        mrs             x5, par_el1

        // initialize x4 with 0xAAAAAAAAAAAAAAAA
        ldr             x4, =A53_STL_BM247_INPUT_VALUE_1

        // write PAR_EL1 register
        msr             par_el1, x4

        // read back the value of PAR_EL1
        mrs             x3, par_el1

        // initialize x28 with golden signature (0xAA0000AAAAAAAAAA)
        ldr             x28, =A53_STL_BM247_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_247

        // initialize x2 with 0x5555555555555555
        ldr             x2, =A53_STL_BM247_INPUT_VALUE_2

        // write PAR_EL1 register
        msr             par_el1, x2

        // read back the value of PAR_EL1
        mrs             x1, par_el1

        // initialize x28 with golden signature (0x5500005555555555)
        ldr             x28, =A53_STL_BM247_GOLDEN_VALUE_2

check_correct_result_BM_247:
        cmp             x28, x1
        b.ne            error_BM_247
        b               success_BM_247

error_BM_247:
        // restore PAR_EL1 register from x5
        msr             par_el1, x5
        b               error

success_BM_247:
        // restore PAR_EL1 register from x5
        msr             par_el1, x5

//-----------------------------------------------------------
// END BASIC MODULE 247
//-----------------------------------------------------------

        // All Basic Modules pass without errors
        mov             x0, A53_STL_FCTLR_STATUS_PDONE

        b               call_postamble

error:

        // Set Data Corruption (0x4) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_regs_error

call_postamble:

        // Restore D,A,I and F bits of PSTATE from x30
        msr             daif, x30

a53_stl_win_c_p038_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p038_n001_end:

        ret

        .end
