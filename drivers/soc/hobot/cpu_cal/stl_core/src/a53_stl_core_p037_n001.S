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
//      Local assumptions of use: AD-FMSK, AI_FPU
//
//   About: TEST_ID
//      CORE_037
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p037_n001
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

        .section .section_a53_stl_core_p037_n001,"ax",%progbits
        .global a53_stl_core_p037_n001
        .type a53_stl_core_p037_n001, %function

a53_stl_core_p037_n001:

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

a53_stl_win_c_p037:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1

//-----------------------------------------------------------
// START BASIC MODULE 332
//-----------------------------------------------------------

// Basic Module 332
// Testing MDCR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of MDCR_EL2 in x2
        mrs             x2, mdcr_el2

        // initialize x20 with 0xAAAAAAAAAAAAAAAA
        ldr             x20, =A53_STL_BM332_INPUT_VALUE_1

        // write MDCR_EL2 register
        msr             mdcr_el2, x20

        // read back the value of MDCR_EL2
        mrs             x21, mdcr_el2

        // initialize x28 with golden signature (0x00000AAA)
        ldr             x28, =A53_STL_BM332_GOLDEN_VALUE_1

        cmp             x28, x21
        b.ne            error_BM_332

        // initialize x22 with 0x5555555555555555
        ldr             x22, =A53_STL_BM332_INPUT_VALUE_2

        // write MDCR_EL2 register
        msr             mdcr_el2, x22

        // read back the value of MDCR_EL2
        mrs             x23, mdcr_el2

        // initialize x28 with golden signature (0x00000555)
        ldr             x28, =A53_STL_BM332_GOLDEN_VALUE_2

check_correct_result_BM_332:
        cmp             x28, x23
        b.ne            error_BM_332
        b               success_BM_332

error_BM_332:
        // restore MDCR_EL2 register from x2
        msr             mdcr_el2, x2
        b               error

success_BM_332:
        // restore MDCR_EL2 register from x2
        msr             mdcr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 332
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 333
//-----------------------------------------------------------

// Basic Module 333
// Testing MVFR0_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of MVFR0_EL1
        mrs             x10, mvfr0_el1

        // initialize x28 with golden signature (0x0000000010110222)
        ldr             x28, =A53_STL_BM333_GOLDEN_VALUE

check_correct_result_BM_333:
        cmp             x28, x10
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 333
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 334
//-----------------------------------------------------------

// Basic Module 334
// Testing MVFR1_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of MVFR1_EL1
        mrs             x11, mvfr1_el1

        // initialize x28 with golden signature (0x0000000012111111)
        ldr             x28, =A53_STL_BM334_GOLDEN_VALUE

check_correct_result_BM_334:
        cmp             x28, x11
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 334
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 335
//-----------------------------------------------------------

// Basic Module 335
// Testing TPIDR_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Save the value of TPIDR_EL0 in x2
        mrs             x2, tpidr_el0

        // initialize x5 with 0xAAAAAAAAAAAAAAAA
        ldr             x5, =A53_STL_BM335_INPUT_VALUE_1

        // write TPIDR_EL0 register
        msr             tpidr_el0, x5

        // read back the value of TPIDR_EL0
        mrs             x6, tpidr_el0

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM335_GOLDEN_VALUE_1

        cmp             x28, x6
        b.ne            error_BM_335

        // initialize x7 with 0x5555555555555555
        ldr             x7, =A53_STL_BM335_INPUT_VALUE_2

        // write TPIDR_EL0 register
        msr             tpidr_el0, x7

        // read back the value of TPIDR_EL0
        mrs             x8, tpidr_el0

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM335_GOLDEN_VALUE_2

check_correct_result_BM_335:
        cmp             x28, x8
        b.ne            error_BM_335
        b               success_BM_335

error_BM_335:
        // restore TPIDR_EL0 register from x2
        msr             tpidr_el0, x2
        b               error

success_BM_335:
        // restore TPIDR_EL0 register from x2
        msr             tpidr_el0, x2

//-----------------------------------------------------------
// END BASIC MODULE 335
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 336
//-----------------------------------------------------------

// Basic Module 336
// Testing TPIDR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TPIDR_EL1 in x2
        mrs             x2, tpidr_el1

        // initialize x7 with 0xAAAAAAAAAAAAAAAA
        ldr             x7, =A53_STL_BM336_INPUT_VALUE_1

        // write TPIDR_EL1 register
        msr             tpidr_el1, x7

        // read back the value of TPIDR_EL1
        mrs             x8, tpidr_el1

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM336_GOLDEN_VALUE_1

        cmp             x28, x8
        b.ne            error_BM_336

        // initialize x5 with 0x5555555555555555
        ldr             x5, =A53_STL_BM336_INPUT_VALUE_2

        // write TPIDR_EL1 register
        msr             tpidr_el1, x5

        // read back the value of TPIDR_EL1
        mrs             x6, tpidr_el1

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM336_GOLDEN_VALUE_2

check_correct_result_BM_336:
        cmp             x28, x6
        b.ne            error_BM_336
        b               success_BM_336

error_BM_336:
        // restore TPIDR_EL1 register from x2
        msr             tpidr_el1, x2
        b               error

success_BM_336:
        // restore TPIDR_EL1 register from x2
        msr             tpidr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 336
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 337
//-----------------------------------------------------------

// Basic Module 337
// Testing VMPIDR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of VMPIDR_EL2 in x2
        mrs             x2, vmpidr_el2

        // initialize x9 with 0xAAAAAAAAAAAAAAAA
        ldr             x9, =A53_STL_BM337_INPUT_VALUE_1

        // write VMPIDR_EL2 register
        msr             vmpidr_el2, x9

        // read back the value of VMPIDR_EL2
        mrs             x10, vmpidr_el2

        // initialize x28 with golden signature (0x000000AA80AAAAAA)
        ldr             x28, =A53_STL_BM337_GOLDEN_VALUE_1

        cmp             x28, x10
        b.ne            error_BM_337

        // initialize x11 with 0x5555555555555555
        ldr             x11, =A53_STL_BM337_INPUT_VALUE_2

        // write VMPIDR_EL2 register
        msr             vmpidr_el2, x11

        // read back the value of VMPIDR_EL2
        mrs             x12, vmpidr_el2

        // initialize x28 with golden signature (0x00000055C1555555)
        ldr             x28, =A53_STL_BM337_GOLDEN_VALUE_2

check_correct_result_BM_337:
        cmp             x28, x12
        b.ne            error_BM_337
        b               success_BM_337

error_BM_337:
        // restore VMPIDR_EL2 register from x2
        msr             vmpidr_el2, x2
        b               error

success_BM_337:
        // restore VMPIDR_EL2 register from x2
        msr             vmpidr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 337
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 338
//-----------------------------------------------------------

// Basic Module 338
// Testing FPSR Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of FPSR in x2
        mrs             x2, fpsr

        // initialize x10 with 0xAAAAAAAAAAAAAAAA
        ldr             x10, =A53_STL_BM338_INPUT_VALUE_1

        // write FPSR register
        msr             fpsr, x10

        // read back the value of FPSR
        mrs             x11, fpsr

        // initialize x28 with golden signature (0x00000000A800008A)
        ldr             x28, =A53_STL_BM338_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_338

        // initialize x12 with 0x5555555555555555
        ldr             x12, =A53_STL_BM338_INPUT_VALUE_2

        // write FPSR register
        msr             fpsr, x12

        // read back the value of FPSR
        mrs             x13, fpsr

        // initialize x28 with golden signature (0x0000000050000015)
        ldr             x28, =A53_STL_BM338_GOLDEN_VALUE_2

check_correct_result_BM_338:
        cmp             x28, x13
        b.ne            error_BM_338
        b               success_BM_338

error_BM_338:
        // restore FPSR register from x2
        msr             fpsr, x2
        b               error

success_BM_338:
        // restore FPSR register from x2
        msr             fpsr, x2

//-----------------------------------------------------------
// END BASIC MODULE 338
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 339
//-----------------------------------------------------------

// Basic Module 339
// Testing FPCR Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of FPCR in x2
        mrs             x2, fpcr

        // initialize x10 with 0xAAAAAAAAAAAAAAAA
        ldr             x10, =A53_STL_BM339_INPUT_VALUE_1

        // write FPCR register
        msr             fpcr, x10

        // read back the value of FPCR
        mrs             x11, fpcr

        // initialize x28 with golden signature (0x0000000002800000)
        ldr             x28, =A53_STL_BM339_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_339

        // initialize x12 with 0x5555555555555555
        ldr             x12, =A53_STL_BM339_INPUT_VALUE_2

        // write FPCR register
        msr             fpcr, x12

        // read back the value of FPCR
        mrs             x13, fpcr

        // initialize x28 with golden signature (0x0000000005400000)
        ldr             x28, =A53_STL_BM339_GOLDEN_VALUE_2

check_correct_result_BM_339:
        cmp             x28, x13
        b.ne            error_BM_339
        b               success_BM_339

error_BM_339:
        // restore FPCR register from x2
        msr             fpcr, x2
        b               error

success_BM_339:
        // restore FPCR register from x2
        msr             fpcr, x2

//-----------------------------------------------------------
// END BASIC MODULE 339
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 340
//-----------------------------------------------------------

// Basic Module 340
// Testing ESR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ESR_EL1 in x2
        mrs             x2, esr_el1

        // initialize x5 with 0xAAAAAAAAAAAAAAAA
        ldr             x5, =A53_STL_BM340_INPUT_VALUE_1

        // write ESR_EL1 register
        msr             esr_el1, x5

        // read back the value of ESR_EL1
        mrs             x18, esr_el1

        // initialize x28 with golden signature (0x00000000AAAAAAAA)
        ldr             x28, =A53_STL_BM340_GOLDEN_VALUE_1

        cmp             x28, x18
        b.ne            error_BM_340

        // initialize x7 with 0x5555555555555555
        ldr             x7, =A53_STL_BM340_INPUT_VALUE_2

        // write ESR_EL1 register
        msr             esr_el1, x7

        // read back the value of ESR_EL1
        mrs             x19, esr_el1

        // initialize x28 with golden signature (0x0000000055555555)
        ldr             x28, =A53_STL_BM340_GOLDEN_VALUE_2

check_correct_result_BM_340:
        cmp             x28, x19
        b.ne            error_BM_340
        b               success_BM_340

error_BM_340:
        // restore ESR_EL1 register from x2
        msr             esr_el1, x2
        b               error

success_BM_340:
        // restore ESR_EL1 register from x2
        msr             esr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 340
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 341
//-----------------------------------------------------------

// Basic Module 341
// Testing FAR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of FAR_EL1 in x2
        mrs             x2, far_el1

        // initialize x8 with 0xAAAAAAAAAAAAAAAA
        ldr             x8, =A53_STL_BM341_INPUT_VALUE_1

        // write FAR_EL1 register
        msr             far_el1, x8

        // read back the value of FAR_EL1
        mrs             x9, far_el1

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM341_GOLDEN_VALUE_1

        cmp             x28, x9
        b.ne            error_BM_341

        // initialize x6 with 0x5555555555555555
        ldr             x6, =A53_STL_BM341_INPUT_VALUE_2

        // write FAR_EL1 register
        msr             far_el1, x6

        // read back the value of FAR_EL1
        mrs             x7, far_el1

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM341_GOLDEN_VALUE_2

check_correct_result_BM_341:
        cmp             x28, x7
        b.ne            error_BM_341
        b               success_BM_341

error_BM_341:
        // restore FAR_EL1 register from x2
        msr             far_el1, x2
        b               error

success_BM_341:
        // restore FAR_EL1 register from x2
        msr             far_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 341
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 342
//-----------------------------------------------------------

// Basic Module 342
// Testing FPEXC32_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Save the value of FPEXC32_EL2 in x2
        mrs             x2, fpexc32_el2

        // initialize x17 with 0xAAAAAAAAAAAAAAAA
        ldr             x17, =A53_STL_BM342_INPUT_VALUE_1

        // write FPEXC32_EL2 register
        msr             fpexc32_el2, x17

        // read back the value of FPEXC32_EL2
        mrs             x20, fpexc32_el2

        // initialize x28 with golden signature (0x0000000000000700)
        ldr             x28, =A53_STL_BM342_GOLDEN_VALUE_1

        cmp             x28, x20
        b.ne            error_BM_342

        // initialize x19 with 0x5555555555555555
        ldr             x19, =A53_STL_BM342_INPUT_VALUE_2

        // write FPEXC32_EL2 register
        msr             fpexc32_el2, x19

        // read back the value of FPEXC32_EL2
        mrs             x18, fpexc32_el2

        // initialize x28 with golden signature (0x0000000040000700)
        ldr             x28, =A53_STL_BM342_GOLDEN_VALUE_2

check_correct_result_BM_342:
        cmp             x28, x18
        b.ne            error_BM_342
        b               success_BM_342

error_BM_342:
        // restore FPEXC32_EL2 register from x2
        msr             fpexc32_el2, x2
        b               error

success_BM_342:
        // restore FPEXC32_EL2 register from x2
        msr             fpexc32_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 342
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

a53_stl_win_c_p037_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p037_n001_end:

        ret

        .end
