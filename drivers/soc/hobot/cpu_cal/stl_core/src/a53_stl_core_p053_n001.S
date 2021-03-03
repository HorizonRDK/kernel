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
//      CORE_053
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p053_n001
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

        .section .section_a53_stl_core_p053_n001,"ax",%progbits
        .global a53_stl_core_p053_n001
        .type a53_stl_core_p053_n001, %function

a53_stl_core_p053_n001:

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

a53_stl_win_c_p053:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1


//-----------------------------------------------------------
// START BASIC MODULE 233
//-----------------------------------------------------------

// Basic Module 233
// Testing MAIR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of MAIR_EL1 in x2
        mrs             x2, mair_el1

        // initialize x5 with 0xAAAAAAAAAAAAAAAA
        ldr             x5, =A53_STL_BM233_INPUT_VALUE_1

        // write MAIR_EL1 register
        msr             mair_el1, x5

        // read back the value of MAIR_EL1
        mrs             x6, mair_el1

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM233_GOLDEN_VALUE_1

        cmp             x28, x6
        b.ne            error_BM_233

        // initialize x7 with 0x5555555555555555
        ldr             x7, =A53_STL_BM233_INPUT_VALUE_2

        // write MAIR_EL1 register
        msr             mair_el1, x7

        // read back the value of MAIR_EL1
        mrs             x8, mair_el1

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM233_GOLDEN_VALUE_2

check_correct_result_BM_233:
        cmp             x28, x8
        b.ne            error_BM_233
        b               success_BM_233

error_BM_233:
        // restore MAIR_EL1 register from x2
        msr             mair_el1, x2
        b               error

success_BM_233:
        // restore MAIR_EL1 register from x2
        msr             mair_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 233
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 248
//-----------------------------------------------------------

// Basic Module 248
// Testing MAIR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of MAIR_EL2 in x2
        mrs             x2, mair_el2

        // initialize x9 with 0xAAAAAAAAAAAAAAAA
        ldr             x9, =A53_STL_BM248_INPUT_VALUE_1

        // write MAIR_EL2 register
        msr             mair_el2, x9

        // read back the value of MAIR_EL2
        mrs             x10, mair_el2

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM248_GOLDEN_VALUE_1

        cmp             x28, x10
        b.ne            error_BM_248

        // initialize x11 with 0x5555555555555555
        ldr             x11, =A53_STL_BM248_INPUT_VALUE_2

        // write MAIR_EL2 register
        msr             mair_el2, x11

        // read back the value of MAIR_EL2
        mrs             x12, mair_el2

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM248_GOLDEN_VALUE_2

check_correct_result_BM_248:
        cmp             x28, x12
        b.ne            error_BM_248
        b               success_BM_248

error_BM_248:
        // restore MAIR_EL2 register from x2
        msr             mair_el2, x2
        b               error

success_BM_248:
        // restore MAIR_EL2 register from x2
        msr             mair_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 248
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 249
//-----------------------------------------------------------

// Basic Module 249
// Testing CONTEXTIDR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CONTEXTIDR_EL1 in x2
        mrs             x2, contextidr_el1

        // initialize x9 with 0xAAAAAAAAAAAAAAAA
        ldr             x9, =A53_STL_BM249_INPUT_VALUE_1

        // write CONTEXTIDR_EL1 register
        msr             contextidr_el1, x9

        // read back the value of CONTEXTIDR_EL1
        mrs             x10, contextidr_el1

        // initialize x28 with golden signature (0xAAAAAAAA)
        ldr             x28, =A53_STL_BM249_GOLDEN_VALUE_1

        cmp             x28, x10
        b.ne            error_BM_249

        // initialize x11 with 0x5555555555555555
        ldr             x11, =A53_STL_BM249_INPUT_VALUE_2

        // write CONTEXTIDR_EL1 register
        msr             contextidr_el1, x11

        // read back the value of CONTEXTIDR_EL1
        mrs             x12, contextidr_el1

        // initialize x28 with golden signature (0x55555555)
        ldr             x28, =A53_STL_BM249_GOLDEN_VALUE_2

check_correct_result_BM_249:
        cmp             x28, x12
        b.ne            error_BM_249
        b               success_BM_249

error_BM_249:
        // restore CONTEXTIDR_EL1 register from x2
        msr             contextidr_el1, x2
        b               error

success_BM_249:
        // restore CONTEXTIDR_EL1 register from x2
        msr             contextidr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 249
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 250
//-----------------------------------------------------------

// Basic Module 250
// Testing SPSR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SPSR_EL1 in x2
        mrs             x2, spsr_el1

        // initialize x13 with 0xAAAAAAAAAAAAAAAA
        ldr             x13, =A53_STL_BM250_INPUT_VALUE_1

        // write SPSR_EL1 register
        msr             spsr_el1, x13

        // read back the value of SPSR_EL1
        mrs             x14, spsr_el1

        // initialize x28 with golden signature (0xAA2AAAAA)
        ldr             x28, =A53_STL_BM250_GOLDEN_VALUE_1

        cmp             x28, x14
        b.ne            error_BM_250

        // initialize x15 with 0x5555555555555555
        ldr             x15, =A53_STL_BM250_INPUT_VALUE_2

        // write SPSR_EL1 register
        msr             spsr_el1, x15

        // read back the value of SPSR_EL1
        mrs             x16, spsr_el1

        // initialize x28 with golden signature (0x54155555)
        ldr             x28, =A53_STL_BM250_GOLDEN_VALUE_2

check_correct_result_BM_250:
        cmp             x28, x16
        b.ne            error_BM_250
        b               success_BM_250

error_BM_250:
        // restore SPSR_EL1 register from x2
        msr             spsr_el1, x2
        b               error

success_BM_250:
        // restore SPSR_EL1 register from x2
        msr             spsr_el1, x2


//-----------------------------------------------------------
// END BASIC MODULE 250
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 251
//-----------------------------------------------------------

// Basic Module 251
// Testing SPSR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SPSR_EL2 in x2
        mrs             x2, spsr_el2

        // initialize x17 with 0xAAAAAAAAAAAAAAAA
        ldr             x17, =A53_STL_BM251_INPUT_VALUE_1

        // write SPSR_EL2 register
        msr             spsr_el2, x17

        // read back the value of SPSR_EL2
        mrs             x18, spsr_el2

        // initialize x28 with golden signature (0xAA2AAAAA)
        ldr             x28, =A53_STL_BM251_GOLDEN_VALUE_1

        cmp             x28, x18
        b.ne            error_BM_251

        // initialize x19 with 0x5555555555555555
        ldr             x19, =A53_STL_BM251_INPUT_VALUE_2

        // write SPSR_EL2 register
        msr             spsr_el2, x19

        // read back the value of SPSR_EL2
        mrs             x20, spsr_el2

        // initialize x28 with golden signature (0x54155555)
        ldr             x28, =A53_STL_BM251_GOLDEN_VALUE_2

check_correct_result_BM_251:
        cmp             x28, x20
        b.ne            error_BM_251
        b               success_BM_251

error_BM_251:
        // restore SPSR_EL2 register from x2
        msr             spsr_el2, x2
        b               success_BM_251

success_BM_251:
        // restore SPSR_EL2 register from x2
        msr             spsr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 251
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 252
//-----------------------------------------------------------

// Basic Module 252
// Testing SPSR_ABT Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SPSR_ABT in x2
        mrs             x2, spsr_abt

        // initialize x25 with 0xAAAAAAAAAAAAAAAA
        ldr             x25, =A53_STL_BM252_INPUT_VALUE_1

        // write SPSR_ABT register
        msr             spsr_abt, x25

        // read back the value of SPSR_ABT
        mrs             x26, spsr_abt

        // initialize x28 with golden signature (0xAA0AAAAA)
        ldr             x28, =A53_STL_BM252_GOLDEN_VALUE_1

        cmp             x28, x26
        b.ne            error_BM_252

        // initialize x27 with 0x5555555555555555
        ldr             x27, =A53_STL_BM252_INPUT_VALUE_2

        // write SPSR_ABT register
        msr             spsr_abt, x27

        // read back the value of SPSR_ABT
        mrs             x29, spsr_abt

        // initialize x28 with golden signature (0x54155555)
        ldr             x28, =A53_STL_BM252_GOLDEN_VALUE_2

check_correct_result_BM_252:
        cmp             x28, x29
        b.ne            error_BM_252
        b               success_BM_252

error_BM_252:
        // restore SPSR_ABT register from x2
        msr             spsr_abt, x2
        b               error

success_BM_252:
        // restore SPSR_ABT register from x2
        msr             spsr_abt, x2

//-----------------------------------------------------------
// END BASIC MODULE 252
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 253
//-----------------------------------------------------------

// Basic Module 253
// Testing SPSR_FIQ Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SPSR_FIQ in x2
        mrs             x2, spsr_fiq

        // initialize x29 with 0xAAAAAAAAAAAAAAAA
        ldr             x29, =A53_STL_BM253_INPUT_VALUE_1

        // write SPSR_FIQ register
        msr             spsr_fiq, x29

        // read back the value of SPSR_FIQ
        mrs             x27, spsr_fiq

        // initialize x28 with golden signature (0xAA0AAAAA)
        ldr             x28, =A53_STL_BM253_GOLDEN_VALUE_1

        cmp             x28, x27
        b.ne            error_BM_253

        // initialize x26 with 0x5555555555555555
        ldr             x26, =A53_STL_BM253_INPUT_VALUE_2

        // write SPSR_FIQ register
        msr             spsr_fiq, x26

        // read back the value of SPSR_FIQ
        mrs             x25, spsr_fiq

        // initialize x28 with golden signature (0x54155555)
        ldr             x28, =A53_STL_BM253_GOLDEN_VALUE_2

check_correct_result_BM_253:
        cmp             x28, x25
        b.ne            error_BM_253
        b               success_BM_253

error_BM_253:
        // restore SPSR_FIQ register from x2
        msr             spsr_fiq, x2
        b               error

success_BM_253:
        // restore SPSR_FIQ register from x2
        msr             spsr_fiq, x2

//-----------------------------------------------------------
// END BASIC MODULE 253
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 254
//-----------------------------------------------------------

// Basic Module 254
// Testing SPSR_UND Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SPSR_UND in x2
        mrs             x2, spsr_und

        // initialize x24 with 0xAAAAAAAAAAAAAAAA
        ldr             x24, =A53_STL_BM254_INPUT_VALUE_1

        // write SPSR_UND register
        msr             spsr_und, x24

        // read back the value of SPSR_UND
        mrs             x23, spsr_und

        // initialize x28 with golden signature (0xAA0AAAAA)
        ldr             x28, =A53_STL_BM254_GOLDEN_VALUE_1

        cmp             x28, x23
        b.ne            error_BM_254

        // initialize x22 with 0x5555555555555555
        ldr             x22, =A53_STL_BM254_INPUT_VALUE_2

        // write SPSR_UND register
        msr             spsr_und, x22

        // read back the value of SPSR_UND
        mrs             x21, spsr_und

        // initialize x28 with golden signature (0x54155555)
        ldr             x28, =A53_STL_BM254_GOLDEN_VALUE_2

check_correct_result_BM_254:
        cmp             x28, x21
        b.ne            error_BM_254
        b               success_BM_254

error_BM_254:
        // restore SPSR_UND register from x2
        msr             spsr_und, x2
        b               error

success_BM_254:
        // restore SPSR_UND register from x2
        msr             spsr_und, x2

//-----------------------------------------------------------
// END BASIC MODULE 254
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 255
//-----------------------------------------------------------

// Basic Module 255
// Testing SPSR_IRQ Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SPSR_IRQ in x2
        mrs             x2, spsr_irq

        // initialize x20 with 0xAAAAAAAAAAAAAAAA
        ldr             x20, =A53_STL_BM255_INPUT_VALUE_1

        // write SPSR_IRQ register
        msr             spsr_irq, x20

        // read back the value of SPSR_IRQ
        mrs             x19, spsr_irq

        // initialize x28 with golden signature (0xAA0AAAAA)
        ldr             x28, =A53_STL_BM255_GOLDEN_VALUE_1

        cmp             x28, x19
        b.ne            error_BM_255

        // initialize x18 with 0x5555555555555555
        ldr             x18, =A53_STL_BM255_INPUT_VALUE_2

        // write SPSR_IRQ register
        msr             spsr_irq, x18

        // read back the value of SPSR_IRQ
        mrs             x17, spsr_irq

        // initialize x28 with golden signature (0x54155555)
        ldr             x28, =A53_STL_BM255_GOLDEN_VALUE_2

check_correct_result_BM_255:
        cmp             x28, x17
        b.ne            error_BM_255
        b               success_BM_255

error_BM_255:
        // restore SPSR_IRQ register from x2
        msr             spsr_irq, x2
        b               error

success_BM_255:
        // restore SPSR_IRQ register from x2
        msr             spsr_irq, x2

//-----------------------------------------------------------
// END BASIC MODULE 255
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 256
//-----------------------------------------------------------

// Basic Module 256
// Testing ELR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ELR_EL1 in x2
        mrs             x2, elr_el1

        // initialize x10 with 0xA5A5A5A5A5A5A5A5
        ldr             x10, =A53_STL_BM256_INPUT_VALUE_1

        // write ELR_EL1 register
        msr             elr_el1, x10

        // read back the value of ELR_EL1
        mrs             x11, elr_el1

        // initialize x28 with golden signature (0xA5A5A5A5A5A5A5A5)
        ldr             x28, =A53_STL_BM256_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_256

        // initialize x12 with 0x5A5A5A5A5A5A5A5A
        ldr             x12, =A53_STL_BM256_INPUT_VALUE_2

        // write ELR_EL1 register
        msr             elr_el1, x12

        // read back the value of ELR_EL1
        mrs             x13, elr_el1

        // initialize x28 with golden signature (0x5A5A5A5A5A5A5A5A)
        ldr             x28, =A53_STL_BM256_GOLDEN_VALUE_2

check_correct_result_BM_256:
        cmp             x28, x13
        b.ne            error_BM_256
        b               success_BM_256

error_BM_256:
        // restore ELR_EL1 register from x2
        msr             elr_el1, x2
        b               error

success_BM_256:
        // restore ELR_EL1 register from x2
        msr             elr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 256
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 257
//-----------------------------------------------------------

// Basic Module 257
// Testing ELR_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ELR_EL2 in x2
        mrs             x2, elr_el2

        // initialize x14 with 0x3C3C3C3C3C3C3C3C
        ldr             x14, =A53_STL_BM257_INPUT_VALUE_1

        // write ELR_EL2 register
        msr             elr_el2, x14

        // read back the value of ELR_EL2
        mrs             x15, elr_el2

        // initialize x28 with golden signature (0x3C3C3C3C3C3C3C3C)
        ldr             x28, =A53_STL_BM257_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_257

        // initialize x16 with 0xC3C3C3C3C3C3C3C3
        ldr             x16, =A53_STL_BM257_INPUT_VALUE_2

        // write ELR_EL2 register
        msr             elr_el2, x16

        // read back the value of ELR_EL2
        mrs             x17, elr_el2

        // initialize x28 with golden signature (0xC3C3C3C3C3C3C3C3)
        ldr             x28, =A53_STL_BM257_GOLDEN_VALUE_2

check_correct_result_BM_257:
        cmp             x28, x17
        b.ne            error_BM_257
        b               success_BM_257

error_BM_257:
        // restore ELR_EL2 register from x2
        msr             elr_el2, x2
        b               error

success_BM_257:
        // restore ELR_EL2 register from x2
        msr             elr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 257
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

a53_stl_win_c_p053_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p053_n001_end:

        ret

        .end
