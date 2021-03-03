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
//      CORE_040
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p040_n001
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

        .section .section_a53_stl_core_p040_n001,"ax",%progbits
        .global a53_stl_core_p040_n001
        .type a53_stl_core_p040_n001, %function

a53_stl_core_p040_n001:

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

a53_stl_win_c_p040:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1

//-----------------------------------------------------------
// START BASIC MODULE 234
//-----------------------------------------------------------

// Basic Module 234
// Testing MDCR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of MDCR_EL3 in x2
        mrs             x2, mdcr_el3

        // initialize x19 with 0xAAAAAAAAAAAAAAAA
        ldr             x19, =A53_STL_BM234_INPUT_VALUE_1

        // write MDCR_EL3 register
        msr             mdcr_el3, x19

        // read back the value of MDCR_EL3
        mrs             x18, mdcr_el3

        // initialize x28 with golden signature (0x0000000000228200)
        ldr             x28, =A53_STL_BM234_GOLDEN_VALUE_1

        cmp             x28, x18
        b.ne            error_BM_234

        // initialize x17 with 0x5555555555555555
        ldr             x17, =A53_STL_BM234_INPUT_VALUE_2

        // write MDCR_EL3 register
        msr             mdcr_el3, x17

        // read back the value of MDCR_EL3
        mrs             x16, mdcr_el3

        // initialize x28 with golden signature (0x0000000000114440)
        ldr             x28, =A53_STL_BM234_GOLDEN_VALUE_2

check_correct_result_BM_234:
        cmp             x28, x16
        b.ne            error_BM_234
        b               success_BM_234

error_BM_234:
        // restore MDCR_EL3 register from x2
        msr             mdcr_el3, x2
        b               error

success_BM_234:
        // restore MDCR_EL3 register from x2
        msr             mdcr_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 234
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 235
//-----------------------------------------------------------

// Basic Module 235
// Testing SCR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SCR_EL3 in x2
        mrs             x2, scr_el3

        // initialize x15 with 0xAAAAAAAAAAAAAAAA
        ldr             x15, =A53_STL_BM235_INPUT_VALUE_1

        // write SCR_EL3 register
        msr             scr_el3, x15

        // read back the value of SCR_EL3
        mrs             x16, scr_el3

        // initialize x28 with golden signature (0x0000000000002AAA)
        ldr             x28, =A53_STL_BM235_GOLDEN_VALUE_1

        cmp             x28, x16
        b.ne            error_BM_235

        // initialize x17 with 0x5555555555555555
        ldr             x17, =A53_STL_BM235_INPUT_VALUE_2

        // write SCR_EL3 register
        msr             scr_el3, x17

        // read back the value of SCR_EL3
        mrs             x18, scr_el3

        // initialize x28 with golden signature (0x0000000000001515)
        ldr             x28, =A53_STL_BM235_GOLDEN_VALUE_2

check_correct_result_BM_235:
        cmp             x28, x18
        b.ne            error_BM_235
        b               success_BM_235

error_BM_235:
        // restore SCR_EL3 register from x2
        msr             scr_el3, x2
        b               error

success_BM_235:
        // restore SCR_EL3 register from x2
        msr             scr_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 235
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 236
//-----------------------------------------------------------

// Basic Module 236
// Testing SDER32_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SDER32_EL3 in x2
        mrs             x2, sder32_el3

        // initialize x10 with 0xAAAAAAAAAAAAAAAA
        ldr             x10, =A53_STL_BM236_INPUT_VALUE_1

        // write SDER32_EL3 register
        msr             sder32_el3, x10

        // read back the value of SDER32_EL3
        mrs             x11, sder32_el3

        // initialize x28 with golden signature (0x0000000000000002)
        ldr             x28, =A53_STL_BM236_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_236

        // initialize x12 with 0x5555555555555555
        ldr             x12, =A53_STL_BM236_INPUT_VALUE_2

        // write SDER32_EL3 register
        msr             sder32_el3, x12

        // read back the value of SDER32_EL3
        mrs             x13, sder32_el3

        // initialize x28 with golden signature (0x0000000000000001)
        ldr             x28, =A53_STL_BM236_GOLDEN_VALUE_2

check_correct_result_BM_236:
        cmp             x28, x13
        b.ne            error_BM_236
        b               success_BM_236

error_BM_236:
        // restore SDER32_EL3 register from x2
        msr             sder32_el3, x2
        b               error

success_BM_236:
        // restore SDER32_EL3 register from x2
        msr             sder32_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 236
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 237
//-----------------------------------------------------------

// Basic Module 237
// Testing TPIDR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TPIDR_EL3 in x2
        mrs             x2, tpidr_el3

        // initialize x9 with 0xAAAAAAAAAAAAAAAA
        ldr             x9, =A53_STL_BM237_INPUT_VALUE_1

        // write TPIDR_EL3 register
        msr             tpidr_el3, x9

        // read back the value of TPIDR_EL3
        mrs             x10, tpidr_el3

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM237_GOLDEN_VALUE_1

        cmp             x28, x10
        b.ne            error_BM_237

        // initialize x11 with 0x5555555555555555
        ldr             x11, =A53_STL_BM237_INPUT_VALUE_2

        // write TPIDR_EL3 register
        msr             tpidr_el3, x11

        // read back the value of TPIDR_EL3
        mrs             x12, tpidr_el3

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM237_GOLDEN_VALUE_2

check_correct_result_BM_237:
        cmp             x28, x12
        b.ne            error_BM_237
        b               success_BM_237

error_BM_237:
        // restore TPIDR_EL3 register from x2
        msr             tpidr_el3, x2
        b               error

success_BM_237:
        // restore TPIDR_EL3 register from x2
        msr             tpidr_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 237
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 238
//-----------------------------------------------------------

// Basic Module 238
// Testing ESR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Save the value of ESR_EL3 in x2
        mrs             x2, esr_el3

        // initialize x6 with 0xAAAAAAAAAAAAAAAA
        ldr             x6, =A53_STL_BM238_INPUT_VALUE_1

        // write ESR_EL3 register
        msr             esr_el3, x6

        // read back the value of ESR_EL3
        mrs             x7, esr_el3

        // initialize x28 with golden signature (0xAAAAAAAA)
        ldr             x28, =A53_STL_BM238_GOLDEN_VALUE_1

        cmp             x28, x7
        b.ne            error_BM_238

        // initialize x8 with 0x5555555555555555
        ldr             x8, =A53_STL_BM238_INPUT_VALUE_2

        // write ESR_EL3 register
        msr             esr_el3, x8

        // read back the value of ESR_EL3
        mrs             x9, esr_el3

        // initialize x28 with golden signature (0x55555555)
        ldr             x28, =A53_STL_BM238_GOLDEN_VALUE_2

check_correct_result_BM_238:
        cmp             x28, x9
        b.ne            error_BM_238
        b               success_BM_238

error_BM_238:
        // restore ESR_EL3 register from x2
        msr             esr_el3, x2
        b               error

success_BM_238:
        // restore ESR_EL3 register from x2
        msr             esr_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 238
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 239
//-----------------------------------------------------------

// Basic Module 239
// Testing FAR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of FAR_EL3 in x2
        mrs             x2, far_el3

        // initialize x25 with 0xAAAAAAAAAAAAAAAA
        ldr             x25, =A53_STL_BM239_INPUT_VALUE_1

        // write FAR_EL3 register
        msr             far_el3, x25

        // read back the value of FAR_EL3
        mrs             x24, far_el3

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM239_GOLDEN_VALUE_1

        cmp             x28, x24
        b.ne            error_BM_239

        // initialize x26 with 0x5555555555555555
        ldr             x26, =A53_STL_BM239_INPUT_VALUE_2

        // write FAR_EL1 register
        msr             far_el3, x26

        // read back the value of FAR_EL3
        mrs             x27, far_el3

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM239_GOLDEN_VALUE_2

check_correct_result_BM_239:
        cmp             x28, x27
        b.ne            error_BM_239
        b               success_BM_239

error_BM_239:
        // restore FAR_EL3 register from x2
        msr             far_el3, x2
        b               error

success_BM_239:
        // restore FAR_EL3 register from x2
        msr             far_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 239
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 240
//-----------------------------------------------------------

// Basic Module 240
// Testing VBAR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of VBAR_EL3 in x2
        mrs             x2, vbar_el3

        // initialize x15 with 0xAAAAAAAAAAAAA800
        ldr             x10, =A53_STL_BM240_INPUT_VALUE_1

        // write VBAR_EL3 register
        msr             vbar_el3, x10

        // read back the value of VBAR_EL3
        mrs             x11, vbar_el3

        // mask read value
        ldr             x0, =A53_STL_BM240_MASK_VALUE
        and             x11, x11, x0

        // initialize x28 with golden signature (0xAAAAAAAAAAAAA800)
        ldr             x28, =A53_STL_BM240_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_240

        // initialize x12 with 0x5555555555555400
        ldr             x12, =A53_STL_BM240_INPUT_VALUE_2

        // write VBAR_EL3 register
        msr             vbar_el3, x12

        // read back the value of VBAR_EL3
        mrs             x13, vbar_el3

        // mask read value
        ldr             x0, =A53_STL_BM240_MASK_VALUE
        and             x13, x13, x0

        // initialize x28 with golden signature (0x5555555555555400)
        ldr             x28, =A53_STL_BM240_GOLDEN_VALUE_2

check_correct_result_BM_240:
        cmp             x28, x13
        b.ne            error_BM_240
        b               success_BM_240

error_BM_240:
        // restore VBAR_EL3 register from x2
        msr             vbar_el3, x2
        b               error

success_BM_240:
        // restore VBAR_EL3 register from x2
        msr             vbar_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 240
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 241
//-----------------------------------------------------------

// Basic Module 241
// Testing TTBR0_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TTBR0_EL3 in x2
        mrs             x2, ttbr0_el3

        // initialize x20 with 0xAAAAAAAAAAAAAAAA
        ldr             x20, =A53_STL_BM241_INPUT_VALUE_1

        // write TTBR0_EL3 register
        msr             ttbr0_el3, x20

        // read back the value of TTBR0_EL3
        mrs             x19, ttbr0_el3

        // initialize x28 with golden signature (0x00AAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM241_GOLDEN_VALUE_1

        cmp             x28, x19
        b.ne            error_BM_241

        // initialize x18 with 0x5555555555555555
        ldr             x18, =A53_STL_BM241_INPUT_VALUE_2

        // write TTBR0_EL3 register
        msr             ttbr0_el3, x18

        // read back the value of TTBR0_EL3
        mrs             x17, ttbr0_el3

        // initialize x28 with golden signature (0x0055555555555551)
        ldr             x28, =A53_STL_BM241_GOLDEN_VALUE_2

check_correct_result_BM_241:
        cmp             x28, x17
        b.ne            error_BM_241
        b               success_BM_241

error_BM_241:
        // restore TTBR0_EL3 register from x2
        msr             ttbr0_el3, x2
        b               error

success_BM_241:
        // restore TTBR0_EL3 register from x2
        msr             ttbr0_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 241
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 242
//-----------------------------------------------------------

// Basic Module 242
// Testing TCR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TCR_EL3 in x2
        mrs             x2, tcr_el3

        // initialize x12 with 0xAAAAAAAAAAAAAAAA
        ldr             x12, =A53_STL_BM242_INPUT_VALUE_1

        // write TCR_EL3 register
        msr             tcr_el3, x12

        // read back the value of TCR_EL3
        mrs             x11, tcr_el3

        // initialize x28 with golden signature (0x00000000AA822AAA)
        ldr             x28, =A53_STL_BM242_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_242

        // initialize x10 with 0x5555555555555555
        ldr             x10, =A53_STL_BM242_INPUT_VALUE_2

        // write TCR_EL3 register
        msr             tcr_el3, x10

        // read back the value of TCR_EL3
        mrs             x9, tcr_el3

        // initialize x28 with golden signature (0x0000000015555515)
        ldr             x28, =A53_STL_BM242_GOLDEN_VALUE_2

check_correct_result_BM_242:
        cmp             x28, x9
        b.ne            error_BM_242
        b               success_BM_242

error_BM_242:
        // restore TCR_EL3 register from x2
        msr             tcr_el3, x2
        b               error

success_BM_242:
        // restore TCR_EL3 register from x2
        msr             tcr_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 242
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

a53_stl_win_c_p040_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p040_n001_end:

        ret

        .end
