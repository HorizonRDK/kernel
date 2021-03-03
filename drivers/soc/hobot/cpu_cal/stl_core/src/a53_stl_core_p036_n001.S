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
//      CORE_036
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p036_n001
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

        .section .section_a53_stl_core_p036_n001,"ax",%progbits
        .global a53_stl_core_p036_n001
        .type a53_stl_core_p036_n001, %function

a53_stl_core_p036_n001:

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

a53_stl_win_c_p036:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1

//-----------------------------------------------------------
// START BASIC MODULE 290
//-----------------------------------------------------------

// Basic Module 290
// Testing CSSELR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CSSELR_EL1 in x2
        mrs             x2, csselr_el1

        // initialize x8 with 0xAAAAAAAAAAAAAAAA
        ldr             x8, =A53_STL_BM290_INPUT_VALUE_1

        // write CSSELR_EL1 register
        msr             csselr_el1, x8

        // read back the value of CSSELR_EL1
        mrs             x9, csselr_el1

        // initialize x28 with golden signature (0x0000000000000002)
        ldr             x28, =A53_STL_BM290_GOLDEN_VALUE_1

        cmp             x28, x9
        b.ne            error_BM_290

        // initialize x7 with 0x5555555555555555
        ldr             x7, =A53_STL_BM290_INPUT_VALUE_2

        // write CSSELR_EL1 register
        msr             csselr_el1, x7

        // read back the value of CSSELR_EL1
        mrs             x6, csselr_el1

        // initialize x28 with golden signature (0x0000000000000001)
        ldr             x28, =A53_STL_BM290_GOLDEN_VALUE_2

check_correct_result_BM_290:
        cmp             x28, x6
        b.ne            error_BM_290
        b               success_BM_290

error_BM_290:
        // restore CSSELR_EL1 register from x2
        msr             csselr_el1, x2
        b               error

success_BM_290:
        // restore CSSELR_EL1 register from x2
        msr             csselr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 290
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 291
//-----------------------------------------------------------

// Basic Module 291
// Testing DACR32_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of DACR32_EL2 in x2
        mrs             x2, dacr32_el2

        // initialize x8 with 0xAAAAAAAAAAAAAAAA
        ldr             x8, =A53_STL_BM291_INPUT_VALUE_1

        // write DACR32_EL2 register
        msr             dacr32_el2, x8

        // read back the value of DACR32_EL2
        mrs             x9, dacr32_el2

        // initialize x28 with golden signature (0x00000000AAAAAAAA)
        ldr             x28, =A53_STL_BM291_GOLDEN_VALUE_1

        cmp             x28, x9
        b.ne            error_BM_291

        // initialize x7 with 0x5555555555555555
        ldr             x7, =A53_STL_BM291_INPUT_VALUE_2

        // write DACR32_EL2 register
        msr             dacr32_el2, x7

        // read back the value of DACR32_EL2
        mrs             x6, dacr32_el2

        // initialize x28 with golden signature (0x0000000055555555)
        ldr             x28, =A53_STL_BM291_GOLDEN_VALUE_2

check_correct_result_BM_291:
        cmp             x28, x6
        b.ne            error_BM_291
        b               success_BM_291

error_BM_291:
        // restore DACR32_EL2 register from x2
        msr             dacr32_el2, x2
        b               error

success_BM_291:
        // restore DACR32_EL2 register from x2
        msr             dacr32_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 291
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 292
//-----------------------------------------------------------

// Basic Module 292
// Testing HCR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of HCR_EL2 in x4
        mrs             x4, hcr_el2

        // initialize x6 with 0xAAAAAAAAAAAAAAAA
        ldr             x6, =A53_STL_BM292_INPUT_VALUE_1

        // write HCR_EL2 register
        msr             hcr_el2, x6

        // read back the value of HCR_EL2
        mrs             x5, hcr_el2

        // initialize x28 with golden signature (0x000000028AAAAAAA)
        ldr             x28, =A53_STL_BM292_GOLDEN_VALUE_1

        cmp             x28, x5
        b.ne            error_BM_292

        // initialize x3 with 0x5555555555555555
        ldr             x3, =A53_STL_BM292_INPUT_VALUE_2

        // write HCR_EL2 register
        msr             hcr_el2, x3

        // read back the value of HCR_EL2
        mrs             x2, hcr_el2

        // initialize x28 with golden signature (0x0000000155555557)
        ldr             x28, =A53_STL_BM292_GOLDEN_VALUE_2

check_correct_result_BM_292:
        cmp             x28, x2
        b.ne            error_BM_292
        b               success_BM_292

error_BM_292:
        // restore HCR_EL2 register from x4
        msr             hcr_el2, x4
        b               error

success_BM_292:
        // restore HCR_EL2 register from x4
        msr             hcr_el2, x4

//-----------------------------------------------------------
// END BASIC MODULE 292
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 309
//-----------------------------------------------------------

// Basic Module 309
// Testing HPFAR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of HPFAR_EL2 in x4
        mrs             x4, hpfar_el2

        // initialize x6 with 0xAAAAAAAAAAAAAAAA
        ldr             x6, =A53_STL_BM309_INPUT_VALUE_1

        // write HPFAR_EL2 register
        msr             hpfar_el2, x6

        // read back the value of HPFAR_EL2
        mrs             x5, hpfar_el2

        // initialize x28 with golden signature (0x00000000AAAAAAA0)
        ldr             x28, =A53_STL_BM309_GOLDEN_VALUE_1

        cmp             x28, x5
        b.ne            error_BM_309

        // initialize x3 with 0x5555555555555555
        ldr             x3, =A53_STL_BM309_INPUT_VALUE_2

        // write HPFAR_EL2 register
        msr             hpfar_el2, x3

        // read back the value of HPFAR_EL2
        mrs             x2, hpfar_el2

        // initialize x28 with golden signature (0x0000000055555550)
        ldr             x28, =A53_STL_BM309_GOLDEN_VALUE_2

check_correct_result_BM_309:
        cmp             x28, x2
        b.ne            error_BM_309
        b               success_BM_309

error_BM_309:
        // restore HPFAR_EL2 register from x4
        msr             hpfar_el2, x4
        b               error

success_BM_309:
        // restore HPFAR_EL2 register from x4
        msr             hpfar_el2, x4

//-----------------------------------------------------------
// END BASIC MODULE 309
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 310
//-----------------------------------------------------------

// Basic Module 310
// Testing HSTR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of HSTR_EL2 in x8
        mrs             x8, hstr_el2

        // initialize x4 with 0xAAAAAAAAAAAAAAAA
        ldr             x4, =A53_STL_BM310_INPUT_VALUE_1

        // write HSTR_EL2 register
        msr             hstr_el2, x4

        // read back the value of HSTR_EL2
        mrs             x5, hstr_el2

        // initialize x28 with golden signature (0x000000000000AAAA)
        ldr             x28, =A53_STL_BM310_GOLDEN_VALUE_1

        cmp             x28, x5
        b.ne            error

        // initialize x2 with 0x5555555555555555
        ldr             x2, =A53_STL_BM310_INPUT_VALUE_2

        // write HSTR_EL2 register
        msr             hstr_el2, x2

        // read back the value of HSTR_EL2
        mrs             x6, hstr_el2

        // initialize x28 with golden signature (0x0000000000001545)
        ldr             x28, =A53_STL_BM310_GOLDEN_VALUE_2

check_correct_result_BM_310:
        cmp             x28, x6
        b.ne            error_BM_310
        b               success_BM_310

error_BM_310:
        // restore HSTR_EL2 register from x8
        msr             hstr_el2, x8
        b               error

success_BM_310:
        // restore HSTR_EL2 register from x8
        msr             hstr_el2, x8

//-----------------------------------------------------------
// END BASIC MODULE 310
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 311
//-----------------------------------------------------------

// Basic Module 311
// Testing ID_AA64AFR0_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64AFR0_EL1
        mrs             x27, id_aa64afr0_el1

        // initialize x28 with golden signature (0x00000000000000000)
        ldr             x28, =A53_STL_BM311_GOLDEN_VALUE

check_correct_result_BM_311:
        cmp             x28, x27
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 311
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 312
//-----------------------------------------------------------

// Basic Module 312
// Testing ID_AA64AFR1_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64AFR1_EL1
        mrs             x27, id_aa64afr1_el1

        // initialize x28 with golden signature (0x00000000000000000)
        ldr             x28, =A53_STL_BM312_GOLDEN_VALUE

check_correct_result_BM_312:
        cmp             x28, x27
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 312
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 313
//-----------------------------------------------------------

// Basic Module 313
// Testing ID_AA64DFR0_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64DFR0_EL1
        mrs             x26, id_aa64dfr0_el1

        // initialize x28 with golden signature (0x0000000010305106)
        ldr             x28, =A53_STL_BM313_GOLDEN_VALUE

check_correct_result_BM_313:
        cmp             x28, x26
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 313
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 314
//-----------------------------------------------------------

// Basic Module 314
// Testing ID_AA64DFR1_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64DFR1_EL1
        mrs             x25, id_aa64dfr1_el1

        // initialize x28 with golden signature (0x0000000000000000)
        ldr             x28, =A53_STL_BM314_GOLDEN_VALUE

check_correct_result_BM_314:
        cmp             x28, x25
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 314
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 315
//-----------------------------------------------------------

// Basic Module 315
// Testing ID_AA64ISAR1_EL1 Register
        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64ISAR1_EL1
        mrs             x9, id_aa64isar1_el1

        // initialize x28 with golden signature (0x0000000000000000)
        ldr             x28, =A53_STL_BM315_GOLDEN_VALUE

check_correct_result_BM_315:
        cmp             x28, x9
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 315
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 316
//-----------------------------------------------------------

// Basic Module 316
// Testing ID_AA64MMFR0_EL1 Register
        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64MMFR0_EL1
        mrs             x24, id_aa64mmfr0_el1

        // initialize x28 with golden signature (0x0000000000001122)
        ldr             x28, =A53_STL_BM316_GOLDEN_VALUE

check_correct_result_BM_316:
        cmp             x28, x24
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 316
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 317
//-----------------------------------------------------------

// Basic Module 317
// Testing ID_AA64MMFR1_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64MMFR1_EL1
        mrs             x23, id_aa64mmfr1_el1

        // initialize x28 with golden signature (0x0000000000000000)
        ldr             x28, =A53_STL_BM317_GOLDEN_VALUE

check_correct_result_BM_317:
        cmp             x28, x23
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 317
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 318
//-----------------------------------------------------------

// Basic Module 318
// Testing ID_AA64PFR0_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64PFR0_EL1
        mrs             x20, id_aa64pfr0_el1

        // initialize x28 with golden signature (0x0000000001002222)
        ldr             x28, =A53_STL_BM318_GOLDEN_VALUE

check_correct_result_BM_318:
        cmp             x28, x20
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 318
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 319
//-----------------------------------------------------------

// Basic Module 319
// Testing ID_AA64PFR1_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_AA64PFR1_EL1
        mrs             x19, id_aa64pfr1_el1

        // initialize x28 with golden signature (0x0000000000000000)
        ldr             x28, =A53_STL_BM319_GOLDEN_VALUE

check_correct_result_BM_319:
        cmp             x28, x19
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 319
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 320
//-----------------------------------------------------------

// Basic Module 320
// Testing ID_DFR0_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_DFR0_EL1
        mrs             x18, id_dfr0_el1

        // initialize x28 with golden signature (0x0000000003010066)
        ldr             x28, =A53_STL_BM320_GOLDEN_VALUE

check_correct_result_BM_320:
        cmp             x28, x18
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 320
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 321
//-----------------------------------------------------------

// Basic Module 321
// Testing ID_ISAR0_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_ISAR0_EL1
        mrs             x18, id_isar0_el1

        // initialize x28 with golden signature (0x0000000002101110)
        ldr             x28, =A53_STL_BM321_GOLDEN_VALUE

check_correct_result_BM_321:
        cmp             x28, x18
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 321
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 322
//-----------------------------------------------------------

// Basic Module 322
// Testing ID_ISAR1_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_ISAR1_EL1
        mrs             x17, id_isar1_el1

        // initialize x28 with golden signature (0x0000000013112111)
        ldr             x28, =A53_STL_BM322_GOLDEN_VALUE

check_correct_result_BM_322:
        cmp             x28, x17
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 322
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 323
//-----------------------------------------------------------

// Basic Module 323
// Testing ID_ISAR2_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_ISAR2_EL1
        mrs             x16, id_isar2_el1

        // initialize x28 with golden signature (0x0000000021232042)
        ldr             x28, =A53_STL_BM323_GOLDEN_VALUE

check_correct_result_BM_323:
        cmp             x28, x16
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 323
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 324
//-----------------------------------------------------------

// Basic Module 324
// Testing ID_ISAR3_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_ISAR3_EL1
        mrs             x15, id_isar3_el1

        // initialize x28 with golden signature (0x0000000001112131)
        ldr             x28, =A53_STL_BM324_GOLDEN_VALUE

check_correct_result_BM_324:
        cmp             x28, x15
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 324
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 325
//-----------------------------------------------------------

// Basic Module 325
// Testing ID_ISAR4_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_ISAR4_EL1
        mrs             x14, id_isar4_el1

        // initialize x28 with golden signature (0x0000000000011142)
        ldr             x28, =A53_STL_BM325_GOLDEN_VALUE

check_correct_result_BM_325:
        cmp             x28, x14
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 325
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 326
//-----------------------------------------------------------

// Basic Module 326
// Testing ID_MMFR0_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_MMFR0_EL1
        mrs             x13, id_mmfr0_el1

        // initialize x28 with golden signature (0x0000000010201105)
        ldr             x28, =A53_STL_BM326_GOLDEN_VALUE

check_correct_result_BM_326:
        cmp             x28, x13
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 326
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 327
//-----------------------------------------------------------

// Basic Module 327
// Testing ID_MMFR1_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_MMFR1_EL1
        mrs             x12, id_mmfr1_el1

        // initialize x28 with golden signature (0x0000000040000000)
        ldr             x28, =A53_STL_BM327_GOLDEN_VALUE

check_correct_result_BM_327:
        cmp             x28, x12
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 327
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 328
//-----------------------------------------------------------

// Basic Module 328
// Testing ID_MMFR2_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_MMFR2_EL1
        mrs             x11, id_mmfr2_el1

        // initialize x28 with golden signature (0x0000000001260000)
        ldr             x28, =A53_STL_BM328_GOLDEN_VALUE

check_correct_result_BM_328:
        cmp             x28, x11
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 328
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 329
//-----------------------------------------------------------

// Basic Module 329
// Testing ID_MMFR3_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_MMFR3_EL1
        mrs             x10, id_mmfr3_el1

        // initialize x28 with golden signature (0x0000000002102211)
        ldr             x28, =A53_STL_BM329_GOLDEN_VALUE

check_correct_result_BM_329:
        cmp             x28, x10
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 329
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 330
//-----------------------------------------------------------

// Basic Module 330
// Testing ID_PFR0_EL1 Register
        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_PFR0_EL1
        mrs             x9, id_pfr0_el1

        // initialize x28 with golden signature (0x0000000000000131)
        ldr             x28, =A53_STL_BM330_GOLDEN_VALUE

check_correct_result_BM_330:
        cmp             x28, x9
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 330
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 331
//-----------------------------------------------------------

// Basic Module 331
// Testing ID_PFR1_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ID_PFR1_EL1
        mrs             x7, id_pfr1_el1

        // initialize x28 with golden signature (0x0000000010011011)
        ldr             x28, =A53_STL_BM331_GOLDEN_VALUE

check_correct_result_BM_331:
        cmp             x28, x7
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 331
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

a53_stl_win_c_p036_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p036_n001_end:

        ret

        .end
