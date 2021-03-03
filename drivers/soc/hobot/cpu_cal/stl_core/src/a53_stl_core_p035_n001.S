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
//      CORE_035
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p035_n001
//      Testing of control registers
//
//   Parameters:
//      R0 - Result address
//
//   Returns:
//      N.A.
//===========================================================================================//

        #include "../../scheduler/inc/a53_stl_global_defs.h"
        #include "../../shared/inc/a53_stl_constants.h"
        #include "../../shared/inc/a53_stl_utils.h"

        .align 4

        .section .section_a53_stl_core_p035_n001,"ax",%progbits
        .global a53_stl_core_p035_n001
        .type a53_stl_core_p035_n001, %function

a53_stl_core_p035_n001:

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

a53_stl_win_c_p035:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1

//-----------------------------------------------------------
// START BASIC MODULE 228
//-----------------------------------------------------------

// Basic Module 228
// Testing IFSR32_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of IFSR32_EL2 in x2
        mrs             x2, ifsr32_el2

        // initialize x8 with 0xAAAAAAAAAAAAAAAA
        ldr             x8, =A53_STL_BM228_INPUT_VALUE_1

        // write IFSR32_EL2 register
        msr             ifsr32_el2, x8

        // read back the value of IFSR32_EL2
        mrs             x9, ifsr32_el2

        // initialize x28 with golden signature (0x0000022A)
        ldr             x28, =A53_STL_BM228_GOLDEN_VALUE_1

        cmp             x28, x9
        b.ne            error_BM_228

        // initialize x7 with 0x5555555555555555
        ldr             x7, =A53_STL_BM228_INPUT_VALUE_2

        // write IFSR32_EL2 register
        msr             ifsr32_el2, x7

        // read back the value of IFSR32_EL2
        mrs             x6, ifsr32_el2

        // initialize x28 with golden signature (0x00001415)
        ldr             x28, =A53_STL_BM228_GOLDEN_VALUE_2

check_correct_result_BM_228:
        cmp             x28, x6
        b.ne            error_BM_228
        b               success_BM_228

error_BM_228:
        // restore IFSR32_EL2 register from x2
        msr             ifsr32_el2, x2
        b               error

success_BM_228:
        // restore IFSR32_EL2 register from x2
        msr             ifsr32_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 228
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 229
//-----------------------------------------------------------

// Basic Module 229
// Testing ACTLR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Save the value of ACTLR_EL2 in x2
        mrs             x2, actlr_el2

        // initialize x4 with 0xAAAAAAAAAAAAAAAA
        ldr             x4, =A53_STL_BM229_INPUT_VALUE_1

        // write ACTLR_EL2 register
        msr             actlr_el2, x4

        // read back the value of ACTLR_EL2
        mrs             x0, actlr_el2

        // initialize x28 with golden signature (0x00000022)
        ldr             x28, =A53_STL_BM229_GOLDEN_VALUE_1

        cmp             x28, x0
        b.ne            error_BM_229

        // initialize x4 with 0x5555555555555555
        ldr             x4, =A53_STL_BM229_INPUT_VALUE_2

        // write ACTLR_EL2 register
        msr             actlr_el2, x4

        // read back the value of ACTLR_EL2
        mrs             x0, actlr_el2

        // initialize x28 with golden signature (0x00000051)
        ldr             x28, =A53_STL_BM229_GOLDEN_VALUE_2

check_correct_result_BM_229:
        cmp             x28, x0
        b.ne            error_BM_229
        b               success_BM_229

error_BM_229:
        // restore ACTLR_EL2 register from x2
        msr             actlr_el2, x2
        b               error

success_BM_229:
        // restore ACTLR_EL2 register from x2
        msr             actlr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 229
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 230
//-----------------------------------------------------------

// Basic Module 230
// Testing NZCV Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of NZCV in x2
        mrs             x2, nzcv

        // initialize x4 with 0xAAAAAAAAAAAAAAAA
        ldr             x4, =A53_STL_BM230_INPUT_VALUE_1

        // write NZCV register
        msr             nzcv, x4

        // read back the value of NZCV
        mrs             x0, nzcv

        // initialize x28 with golden signature (0xA0000000)
        ldr             x28, =A53_STL_BM230_GOLDEN_VALUE_1

        cmp             x28, x0
        b.ne            error_BM_230

        // initialize x4 with 0x5555555555555555
        ldr             x4, =A53_STL_BM230_INPUT_VALUE_2

        // write NZCV register
        msr             nzcv, x4

        // read back the value of NZCV
        mrs             x0, nzcv

        // initialize x28 with golden signature (0x50000000)
        ldr             x28, =A53_STL_BM230_GOLDEN_VALUE_2

check_correct_result_BM_230:
        cmp             x28, x0
        b.ne            error_BM_230
        b               success_BM_230

error_BM_230:
        // restore NZCV register from x2
        msr             nzcv, x2
        b               error

success_BM_230:
        // restore NZCV register from x2
        msr             nzcv, x2

//-----------------------------------------------------------
// END BASIC MODULE 230
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 231
//-----------------------------------------------------------

// Basic Module 231
// Testing TPIDR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TPIDR_EL2 in x2
        mrs             x2, tpidr_el2

        // initialize x4 with 0xAAAAAAAAAAAAAAAA
        ldr             x4, =A53_STL_BM231_INPUT_VALUE_1

        // write TPIDR_EL2 register
        msr             tpidr_el2, x4

        // read back the value of TPIDR_EL2
        mrs             x0, tpidr_el2

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM231_GOLDEN_VALUE_1

        cmp             x28, x0
        b.ne            error_BM_231

        // initialize x4 with 0x5555555555555555
        ldr             x4, =A53_STL_BM231_INPUT_VALUE_2

        // write TPIDR_EL2 register
        msr             tpidr_el2, x4

        // read back the value of TPIDR_EL2
        mrs             x0, tpidr_el2

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM231_GOLDEN_VALUE_2

check_correct_result_BM_231:
        cmp             x28, x0
        b.ne            error_BM_231
        b               success_BM_231

error_BM_231:
        // restore TPIDR_EL2 register from x2
        msr             tpidr_el2, x2
        b               error

success_BM_231:
        // restore TPIDR_EL2 register from x2
        msr             tpidr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 231
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 232
//-----------------------------------------------------------

// Basic Module 232
// Testing VPIDR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of VPIDR_EL2 in x2
        mrs             x2, vpidr_el2

        // initialize x4 with 0xAAAAAAAAAAAAAAAA
        ldr             x4, =A53_STL_BM232_INPUT_VALUE_1

        // write VPIDR_EL2 register
        msr             vpidr_el2, x4

        // read back the value of VPIDR_EL2
        mrs             x0, vpidr_el2

        // initialize x28 with golden signature (0x00000000AAAAAAAA)
        ldr             x28, =A53_STL_BM232_GOLDEN_VALUE_1

        cmp             x28, x0
        b.ne            error_BM_232

        // initialize x4 with 0x5555555555555555
        ldr             x4, =A53_STL_BM232_INPUT_VALUE_2

        // write VPIDR_EL2 register
        msr             vpidr_el2, x4

        // read back the value of VPIDR_EL2
        mrs             x0, vpidr_el2

        // initialize x28 with golden signature (0x0000000055555555)
        ldr             x28, =A53_STL_BM232_GOLDEN_VALUE_2

check_correct_result_BM_232:
        cmp             x28, x0
        b.ne            error_BM_232
        b               success_BM_232

error_BM_232:
        // restore VPIDR_EL2 register from x2
        msr             vpidr_el2, x2
        b               error

success_BM_232:
        // restore VPIDR_EL2 register from x2
        msr             vpidr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 232
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 284
//-----------------------------------------------------------

// Basic Module 284
// Testing ESR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ESR_EL2 in x2
        mrs             x2, esr_el2

        // initialize x6 with 0xAAAAAAAAAAAAAAAA
        ldr             x6, =A53_STL_BM284_INPUT_VALUE_1

        // write ESR_EL2 register
        msr             esr_el2, x6

        // read back the value of ESR_EL2
        mrs             x9, esr_el2

        // initialize x28 with golden signature (0x00000000AAAAAAAA)
        ldr             x28, =A53_STL_BM284_GOLDEN_VALUE_1

        cmp             x28, x9
        b.ne            error_BM_284

        // initialize x15 with 0x5555555555555555
        ldr             x15, =A53_STL_BM284_INPUT_VALUE_2

        // write ESR_EL2 register
        msr             esr_el2, x15

        // read back the value of ESR_EL2
        mrs             x20, esr_el2

        // initialize x28 with golden signature (0x0000000055555555)
        ldr             x28, =A53_STL_BM232_GOLDEN_VALUE_2

check_correct_result_BM_284:
        cmp             x28, x20
        b.ne            error_BM_284
        b               success_BM_284

error_BM_284:
        // restore ESR_EL2 register from x2
        msr             esr_el2, x2
        b               error

success_BM_284:
        // restore ESR_EL2 register from x2
        msr             esr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 284
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 286
//-----------------------------------------------------------

// Basic Module 286
// Testing CLIDR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of CLIDR_EL1
        mrs             x27, clidr_el1

        // initialize x28 with golden signature (0x000000000A200023)
        ldr             x28, =A53_STL_BM286_GOLDEN_VALUE

check_correct_result_BM_286:
        cmp             x28, x27
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 286
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 289
//-----------------------------------------------------------

// Basic Module 289
// Testing FAR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of FAR_EL2 in x2
        mrs             x2, far_el2

        // initialize x10 with 0xAAAAAAAAAAAAAAAA
        ldr             x10, =A53_STL_BM289_INPUT_VALUE_1

        // write FAR_EL2 register
        msr             far_el2, x10

        // read back the value of FAR_EL2
        mrs             x11, far_el2

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM289_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_289

        // initialize x12 with 0x5555555555555555
        ldr             x12, =A53_STL_BM289_INPUT_VALUE_2

        // write FAR_EL2 register
        msr             far_el2, x12

        // read back the value of FAR_EL2
        mrs             x13, far_el2

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM289_GOLDEN_VALUE_2

check_correct_result_BM_289:
        cmp             x28, x13
        b.ne            error_BM_289
        b               success_BM_289

error_BM_289:
        // restore FAR_EL2 register from x2
        msr             far_el2, x2
        b               error

success_BM_289:
        // restore FAR_EL2 register from x2
        msr             far_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 289
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 161
//-----------------------------------------------------------

// Basic Module 161
// Testing DCZID_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of DCZID_EL0
        mrs             x14, dczid_el0

        // initialize x28 with golden signature (0x00000004)
        ldr             x28, =A53_STL_BM161_GOLDEN_VALUE_1

check_correct_result_BM_161:
        cmp             x28, x14
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 161
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 162
//-----------------------------------------------------------

// Basic Module 162
// Testing REVIDR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of REVIDR_EL1
        mrs             x14, revidr_el1

        // initialize x28 with golden signature (0x00000380)
        ldr             x28, =A53_STL_BM162_GOLDEN_VALUE_1

        // Mask bits for both x14 and x28
        orr             x28, x28, A53_STL_BM162_MASK_VALUE
        orr             x14, x14, A53_STL_BM162_MASK_VALUE

check_correct_result_BM_162:
        cmp             x28, x14
        b.ne            error


//-----------------------------------------------------------
// END BASIC MODULE 162
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 163
//-----------------------------------------------------------

// Basic Module 163
// Testing CSSELR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CSSELR_EL1 in x2
        mrs             x2, csselr_el1

        // initialize x10 with 0x00 to consider L1 data cache
        mov             x10, A53_STL_BM163_INPUT_VALUE_1

        // set CSSELR_EL1 to 0x0 to consider L1 data cache
        msr             csselr_el1, x10

        // read back the value of CCSIDR_EL1
        mrs             x11, ccsidr_el1

        // initialize x28 with golden signature (0x700FE01A)
        ldr             x28, =A53_STL_BM163_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_163

        // initialize x12 with 0x01 to consider L1 instruction cache
        mov             x12, A53_STL_BM163_INPUT_VALUE_2

        // set CSSELR_EL1 to 0x01 to consider L1 instruction cache
        msr             csselr_el1, x12

        // read back the value of CCSIDR_EL1
        mrs             x13, ccsidr_el1

        // initialize x28 with golden signature (0x201FE00A)
        ldr             x28, =A53_STL_BM163_GOLDEN_VALUE_2

        cmp             x28, x13
        b.ne            error_BM_163

        // initialize x14 with 0x2 to consider L2 cache
        mov             x14, A53_STL_BM163_INPUT_VALUE_3

        // set CSSELR_EL1 to 0x10 to consider L2 cache
        msr             csselr_el1, x14

        // read back the value of CCSIDR_EL1
        mrs             x15, ccsidr_el1

        // initialize x28 with golden signature (0x703FE07A)
        ldr             x28, =A53_STL_BM163_GOLDEN_VALUE_3

check_correct_result_BM_163:
        cmp             x28, x15
        b.ne            error_BM_163
        b               success_BM_163

error_BM_163:
        // restore CSSELR_EL1 register from x2
        msr             csselr_el1, x2
        b               error

success_BM_163:
        // restore CSSELR_EL1 register from x2
        msr             csselr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 163
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 275
//-----------------------------------------------------------

// Basic Module 275
// Testing CBAR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of CBAR_EL1
        mrs             x11, s3_1_c15_c3_0

        // initialize x28 with golden signature (0x13080000)
        ldr             x28, =A53_STL_BM275_GOLDEN_VALUE_1

        // Mask bits for both x11 and x28
        ldr             x10, =A53_STL_BM275_MASK_VALUE
        orr             x28, x28, x10
        orr             x11, x11, x10

check_correct_result_BM_275:
        cmp             x28, x11
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 275
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

a53_stl_win_c_p035_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p035_n001_end:

        ret

        .end
