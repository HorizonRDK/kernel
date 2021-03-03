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
//      CORE_052
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p052_n001
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

        .section .section_a53_stl_core_p052_n001,"ax",%progbits
        .global a53_stl_core_p052_n001
        .type a53_stl_core_p052_n001, %function

a53_stl_core_p052_n001:

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

a53_stl_win_c_p052:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1

//-----------------------------------------------------------
// START BASIC MODULE 343
//-----------------------------------------------------------

// Basic Module 343
// Testing CPACR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CPACR_EL1 in x2
        mrs             x2, cpacr_el1

        // initialize x16 with 0xAAAAAAAAAAAAAAAA
        ldr             x16, =A53_STL_BM343_INPUT_VALUE_1

        // write CPACR_EL1 register
        msr             cpacr_el1, x16

        // read back the value of CPACR_EL1
        mrs             x15, cpacr_el1

        // initialize x28 with golden signature (0x0000000080A00000)
        ldr             x28, =A53_STL_BM343_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_343

        // initialize x14 with 0x5555555555555555
        ldr             x14, =A53_STL_BM343_INPUT_VALUE_2

        // write CPACR_EL1 register
        msr             cpacr_el1, x14

        // read back the value of CPACR_EL1
        mrs             x13, cpacr_el1

        // initialize x28 with golden signature (0x0000000000500000)
        ldr             x28, =A53_STL_BM343_GOLDEN_VALUE_2

check_correct_result_BM_343:
        cmp             x28, x13
        b.ne            error_BM_343
        b               success_BM_343

error_BM_343:
        // restore CPACR_EL1 register from x2
        msr             cpacr_el1, x2
        b               error

success_BM_343:
        // restore CPACR_EL1 register from x2
        msr             cpacr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 343
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 344
//-----------------------------------------------------------

// Basic Module 344
// Testing CPTR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CPTR_EL2 in x2
        mrs             x2, cptr_el2

        // initialize x15 with 0xAAAAAAAAAAAAAAAA
        ldr             x15, =A53_STL_BM344_INPUT_VALUE_1

        // write CPTR_EL2 register
        msr             cptr_el2, x15

        // read back the value of CPTR_EL2
        mrs             x10, cptr_el2

        // initialize x28 with golden signature (0x000000008000BBFF)
        ldr             x28, =A53_STL_BM344_GOLDEN_VALUE_1

        cmp             x28, x10
        b.ne            error_BM_344

        // initialize x11 with 0x5555555555555555
        ldr             x11, =A53_STL_BM344_INPUT_VALUE_2

        // write CPTR_EL2 register
        msr             cptr_el2, x11

        // read back the value of CPTR_EL2
        mrs             x12, cptr_el2

        // initialize x28 with golden signature (0x00000000000037FF)
        ldr             x28, =A53_STL_BM344_GOLDEN_VALUE_2

check_correct_result_BM_344:
        cmp             x28, x12
        b.ne            error_BM_344
        b               success_BM_344

error_BM_344:
        // restore CPTR_EL2 register from x2
        msr             cptr_el2, x2
        b               error

success_BM_344:
        // restore CPTR_EL2 register from x2
        msr             cptr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 344
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 345
//-----------------------------------------------------------

// Basic Module 345
// Testing MIDR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of MIDR_EL1
        mrs             x12, midr_el1

        // initialize x28 with golden signature (0x410FD034)
        ldr             x28, =A53_STL_BM345_GOLDEN_VALUE

        // Mask bits for both x12 and x28
        ldr             x10, =A53_STL_BM345_MASK_VALUE
        orr             x28, x28, x10
        orr             x12, x12, x10

check_correct_result_BM_345:
        cmp             x28, x12
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 345
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 346
//-----------------------------------------------------------

// Basic Module 346
// Testing SCTLR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SCTLR_EL1 in x2
        mrs             x2, sctlr_el1

        // initialize x5 with 0xAAAAAAAAAAAAAAAA
        ldr             x5, =A53_STL_BM346_INPUT_VALUE_1

        // write SCTLR_EL1 register
        msr             sctlr_el1, x5

        // read back the value of SCTLR_EL1
        mrs             x4, sctlr_el1

        // initialize x28 with golden signature (0x0000000022C8AAAA)
        ldr             x28, =A53_STL_BM346_GOLDEN_VALUE_1

        cmp             x28, x4
        b.ne            error_BM_346

        // initialize x6 with 0x5555555555555555
        ldr             x6, =A53_STL_BM346_INPUT_VALUE_2

        // write SCTLR_EL1 register
        msr             sctlr_el1, x6

        // read back the value of SCTLR_EL1
        mrs             x7, sctlr_el1

        // initialize x28 with golden signature (0x0000000055D55915)
        ldr             x28, =A53_STL_BM346_GOLDEN_VALUE_2

check_correct_result_BM_346:
        cmp             x28, x7
        b.ne            error_BM_346
        b               success_BM_346

error_BM_346:
        // restore SCTLR_EL1 register from x2
        msr             sctlr_el1, x2
        b               error

success_BM_346:
        // restore SCTLR_EL1 register from x2
        msr             sctlr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 346
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 347
//-----------------------------------------------------------

// Basic Module 347
// Testing SCTLR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SCTLR_EL2 in x2
        mrs             x2, sctlr_el2

        // initialize x15 with 0xAAAAAAAAAAAAAAAA
        ldr             x15, =A53_STL_BM347_INPUT_VALUE_1

        // write SCTLR_EL2 register
        msr             sctlr_el2, x15

        // read back the value of SCTLR_EL2
        mrs             x14, sctlr_el2

        // initialize x28 with golden signature (0x000000000x32CD08BA)
        ldr             x28, =A53_STL_BM347_GOLDEN_VALUE_1

        cmp             x28, x14
        b.ne            error_BM_347

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM347_INPUT_VALUE_2

        // write SCTLR_EL2 register
        msr             sctlr_el2, x16

        // read back the value of SCTLR_EL2
        mrs             x17, sctlr_el2

        // initialize x28 with golden signature (0x0000000070C51915)
        ldr             x28, =A53_STL_BM347_GOLDEN_VALUE_2

check_correct_result_BM_347:
        cmp             x28, x17
        b.ne            error_BM_347
        b               success_BM_347

error_BM_347:
        // restore SCTLR_EL2 register from x2
        msr             sctlr_el2, x2
        b               error

success_BM_347:
        // restore SCTLR_EL2 register from x2
        msr             sctlr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 347
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 348
//-----------------------------------------------------------

// Basic Module 348
// Testing VBAR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of VBAR_EL2 in x2
        mrs             x2, vbar_el2

        // initialize x10 with 0xAAAAAAAAAAAAAAAA
        ldr             x10, =A53_STL_BM348_INPUT_VALUE_1

        // write VBAR_EL2 register
        msr             vbar_el2, x10

        // read back the value of VBAR_EL2
        mrs             x11, vbar_el2

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAA0)
        ldr             x28, =A53_STL_BM348_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_348

        // initialize x12 with 0x5555555555555555
        ldr             x12, =A53_STL_BM348_INPUT_VALUE_2

        // write VBAR_EL2 register
        msr             vbar_el2, x12

        // read back the value of VBAR_EL2
        mrs             x13, vbar_el2

        // initialize x28 with golden signature (0x5555555555555540)
        ldr             x28, =A53_STL_BM348_GOLDEN_VALUE_2

check_correct_result_BM_348:
        cmp             x28, x13
        b.ne            error_BM_348
        b               success_BM_348

error_BM_348:
        // restore VBAR_EL2 register from x2
        msr             vbar_el2, x2
        b               error

success_BM_348:
        // restore VBAR_EL2 register from x2
        msr             vbar_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 348
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 349
//-----------------------------------------------------------

// Basic Module 349
// Testing VBAR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of VBAR_EL1 in x2
        mrs             x2, vbar_el1

        // initialize x20 with 0xAAAAAAAAAAAAAAAA
        ldr             x20, =A53_STL_BM349_INPUT_VALUE_1

        // write VBAR_EL1 register
        msr             vbar_el1, x20

        // read back the value of VBAR_EL1
        mrs             x19, vbar_el1

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAA0)
        ldr             x28, =A53_STL_BM349_GOLDEN_VALUE_1

        cmp             x28, x19
        b.ne            error_BM_349

        // initialize x18 with 0x5555555555555555
        ldr             x18, =A53_STL_BM349_INPUT_VALUE_2

        // write VBAR_EL1 register
        msr             vbar_el1, x18

        // read back the value of VBAR_EL1
        mrs             x26, vbar_el1

        // initialize x28 with golden signature (0x5555555555555540)
        ldr             x28, =A53_STL_BM349_GOLDEN_VALUE_2

check_correct_result_BM_349:
        cmp             x28, x26
        b.ne            error_BM_349
        b               success_BM_349

error_BM_349:
        // restore VBAR_EL1 register from x2
        msr             vbar_el1, x2
        b               error

success_BM_349:
        // restore VBAR_EL1 register from x2
        msr             vbar_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 349
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 350
//-----------------------------------------------------------

// Basic Module 350
// Testing MVFR2_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of MVFR2_EL1
        mrs             x14, mvfr2_el1

        // initialize x28 with golden signature (0x0000000000000043)
        ldr             x28, =A53_STL_BM350_GOLDEN_VALUE

check_correct_result_BM_350:
        cmp             x28, x14
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 350
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 351
//-----------------------------------------------------------

// Basic Module 351
// Testing ID_AA64ISAR0_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64ISAR0_EL1
        mrs             x15, id_aa64isar0_el1

        // initialize x28 with golden signature (0x0000000000011120)
        ldr             x28, =A53_STL_BM351_GOLDEN_VALUE

        // Mask bits for both x11 and x28
        ldr             x10, =A53_STL_BM351_MASK_VALUE
        orr             x28, x28, x10
        orr             x15, x15, x10

check_correct_result_BM_351:
        cmp             x28, x15
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 351
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 352
//-----------------------------------------------------------

// Basic Module 352
// Testing ID_ISAR5_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_ISAR5_EL1
        mrs             x14, id_isar5_el1

        // initialize x28 with golden signature (0x00010001)
        ldr             x28, =A53_STL_BM352_GOLDEN_VALUE

        // Mask bits for both x14 and x28
        ldr             x10, =A53_STL_BM352_MASK_VALUE
        orr             x28, x28, x10
        orr             x14, x14, x10

check_correct_result_BM_352:
        cmp             x28, x14
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 352
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

a53_stl_win_c_p052_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p052_n001_end:

        ret

        .end
