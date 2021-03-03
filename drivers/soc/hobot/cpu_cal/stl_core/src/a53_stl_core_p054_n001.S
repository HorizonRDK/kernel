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
//      Local assumptions of use: AD-FMSK, AI_SHR
//
//   About: TEST_ID
//      CORE_054
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p054_n001
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

        .section .section_a53_stl_core_p054_n001,"ax",%progbits
        .global a53_stl_core_p054_n001
        .type a53_stl_core_p054_n001, %function

a53_stl_core_p054_n001:

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

a53_stl_win_c_p054:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1

//-----------------------------------------------------------
// START BASIC MODULE 244
//-----------------------------------------------------------

// Basic Module 244
// Testing MAIR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of MAIR_EL3 in x5
        mrs             x5, mair_el3

        // initialize x1 with 0xAAAAAAAAAAAAAAAA
        ldr             x1, =A53_STL_BM244_INPUT_VALUE_1

        // write MAIR_EL3 register
        msr             mair_el3, x1

        // read back the value of MAIR_EL3
        mrs             x2, mair_el3

        // initialize x28 with golden signature (0xAAAAAAAAAAAAAAAA)
        ldr             x28, =A53_STL_BM244_GOLDEN_VALUE_1

        cmp             x28, x2
        b.ne            error_BM_244

        // initialize x3 with 0x5555555555555555
        ldr             x3, =A53_STL_BM244_INPUT_VALUE_2

        // write MAIR_EL3 register
        msr             mair_el3, x3

        // read back the value of MAIR_EL3
        mrs             x4, mair_el3

        // initialize x28 with golden signature (0x5555555555555555)
        ldr             x28, =A53_STL_BM244_GOLDEN_VALUE_2

check_correct_result_BM_244:
        cmp             x28, x4
        b.ne            error_BM_244
        b               success_BM_244

error_BM_244:
        // restore MAIR_EL3 register from x5
        msr             mair_el3, x5
        b               error

success_BM_244:
        // restore MAIR_EL3 register from x5
        msr             mair_el3, x5

//-----------------------------------------------------------
// END BASIC MODULE 244
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 245
//-----------------------------------------------------------

// Basic Module 245
// Testing SPSR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SPSR_EL3 in x2
        mrs             x2, spsr_el3

        // initialize x21 with 0xAAAAAAAAAAAAAAAA
        ldr             x21, =A53_STL_BM245_INPUT_VALUE_1

        // write SPSR_EL3 register
        msr             spsr_el3, x21

        // read back the value of SPSR_EL3
        mrs             x22, spsr_el3

        // initialize x28 with golden signature (0xAA2AAAAA)
        ldr             x28, =A53_STL_BM245_GOLDEN_VALUE_1

        cmp             x28, x22
        b.ne            error_BM_245

        // initialize x23 with 0x5555555555555555
        ldr             x23, =A53_STL_BM245_INPUT_VALUE_2

        // write SPSR_EL3 register
        msr             spsr_el3, x23

        // read back the value of SPSR_EL3
        mrs             x24, spsr_el3

        // initialize x28 with golden signature (0x54155555)
        ldr             x28, =A53_STL_BM245_GOLDEN_VALUE_2

check_correct_result_BM_245:
        cmp             x28, x24
        b.ne            error_BM_245
        b               success_BM_245

error_BM_245:
        // restore SPSR_EL3 register from x2
        msr             spsr_el3, x2
        b               error

success_BM_245:
        // restore SPSR_EL3 register from x2
        msr             spsr_el3, x2


//-----------------------------------------------------------
// END BASIC MODULE 245
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 246
//-----------------------------------------------------------

// Basic Module 246
// Testing ELR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ELR_EL3 in x2
        mrs             x2, elr_el3

        // initialize x20 with 0x4B4B4B4B4B4B4B4B
        ldr             x20, =A53_STL_BM246_INPUT_VALUE_1

        // write ELR_EL3 register
        msr             elr_el3, x20

        // read back the value of ELR_EL3
        mrs             x21, elr_el3

        // initialize x28 with golden signature (0x4B4B4B4B4B4B4B4B)
        ldr             x28, =A53_STL_BM246_GOLDEN_VALUE_1

        cmp             x28, x21
        b.ne            error_BM_246

        // initialize x22 with 0xB4B4B4B4B4B4B4B4
        ldr             x22, =A53_STL_BM246_INPUT_VALUE_2

        // write ELR_EL3 register
        msr             elr_el3, x22

        // read back the value of ELR_EL3
        mrs             x23, elr_el3

        // initialize x28 with golden signature (0xB4B4B4B4B4B4B4B4)
        ldr             x28, =A53_STL_BM246_GOLDEN_VALUE_2

check_correct_result_BM_246:
        cmp             x28, x23
        b.ne            error_BM_246
        b               success_BM_246

error_BM_246:
        // restore ELR_EL3 register from x2
        msr             elr_el3, x2
        b               error

success_BM_246:
        // restore ELR_EL3 register from x2
        msr             elr_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 246
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 285
//-----------------------------------------------------------

// Basic Module 285
// Testing ACTLR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ACTLR_EL3 in x2
        mrs             x2, actlr_el3

        // initialize x24 with 0xAAAAAAAAAAAAAAAA
        ldr             x24, =A53_STL_BM285_INPUT_VALUE_1

        // write ACTLR_EL3 register
        msr             actlr_el3, x24

        // read back the value of ACTLR_EL3
        mrs             x21, actlr_el3

        // initialize x28 with golden signature (0x0000000000000022)
        ldr             x28, =A53_STL_BM285_GOLDEN_VALUE_1

        cmp             x28, x21
        b.ne            error_BM_285

        // initialize x26 with 0x5555555555555555
        ldr             x26, =A53_STL_BM285_INPUT_VALUE_2

        // write ACTLR_EL3 register
        msr             actlr_el3, x26

        // read back the value of ACTLR_EL3
        mrs             x23, actlr_el3

        // initialize x28 with golden signature (0x0000000000000051)
        ldr             x28, =A53_STL_BM285_GOLDEN_VALUE_2

check_correct_result_BM_285:
        cmp             x28, x23
        b.ne            error_BM_285
        b               success_BM_285

error_BM_285:
        // restore ACTLR_EL3 register from x2
        msr             actlr_el3, x2
        b               error

success_BM_285:
        // restore ACTLR_EL3 register from x2
        msr             actlr_el3, x2


//-----------------------------------------------------------
// END BASIC MODULE 285
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 287
//-----------------------------------------------------------

// Basic Module 287
// Testing CPTR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CPTR_EL3 in x2
        mrs             x2, cptr_el3

        // initialize x22 with 0xAAAAAAAAAAAAAAAA
        ldr             x22, =A53_STL_BM287_INPUT_VALUE_1

        // write CPTR_EL3 register
        msr             cptr_el3, x22

        // read back the value of CPTR_EL3
        mrs             x18, cptr_el3

        // initialize x28 with golden signature (0x0000000080000000)
        ldr             x28, =A53_STL_BM287_GOLDEN_VALUE_1

        cmp             x28, x18
        b.ne            error_BM_287

        // initialize x17 with 0x5555555555555555
        ldr             x17, =A53_STL_BM287_INPUT_VALUE_2

        // write CPTR_EL3 register
        msr             cptr_el3, x17

        // read back the value of CPTR_EL3
        mrs             x13, cptr_el3

        // initialize x28 with golden signature (0x0000000000000400)
        ldr             x28, =A53_STL_BM287_GOLDEN_VALUE_2

check_correct_result_BM_287:
        cmp             x28, x13
        b.ne            error_BM_287
        b               success_BM_287

error_BM_287:
        // restore CPTR_EL3 register from x2
        msr             cptr_el3, x2
        b               error

success_BM_287:
        // restore CPTR_EL3 register from x2
        msr             cptr_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 287
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 288
//-----------------------------------------------------------

// Basic Module 288
// Testing CPUECTLR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CPUECTLR_EL1 in x2
        mrs             x2, s3_1_c15_c2_1

        // initialize x11 with 0xAAAAAAAAAAAAAAAA
        ldr             x11, =A53_STL_BM288_INPUT_VALUE_1

        // write CPUECTLR_EL1 register
        msr             s3_1_c15_c2_1, x11

        // read back the value of CPUECTLR_EL1
        mrs             x12, s3_1_c15_c2_1

        // initialize x28 with golden signature (0x000000000000002A)
        ldr             x28, =A53_STL_BM288_GOLDEN_VALUE_1

        cmp             x28, x12
        b.ne            error_BM_288

        // initialize x14 with 0x5555555555555555
        ldr             x14, =A53_STL_BM288_INPUT_VALUE_2

        // write CPUECTLR_EL1 register
        msr             s3_1_c15_c2_1, x14

        // read back the value of CPUECTLR_EL1
        mrs             x16, s3_1_c15_c2_1

        // initialize x28 with golden signature (0x0000000000000055)
        ldr             x28, =A53_STL_BM288_GOLDEN_VALUE_2

check_correct_result_BM_288:
        cmp             x28, x16
        b.ne            error_BM_288
        b               success_BM_288

error_BM_288:
        // restore CPTR_EL3 register from x2
        msr             s3_1_c15_c2_1, x2
        b               error

success_BM_288:
        // restore CPTR_EL3 register from x2
        msr             s3_1_c15_c2_1, x2

//-----------------------------------------------------------
// END BASIC MODULE 288
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 260
//-----------------------------------------------------------

// Basic Module 260
// Testing SP_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SP_EL2 in x2
        mrs             x2, sp_el2

        // initialize x13 with 0x6969696969696969
        ldr             x13, =A53_STL_BM260_INPUT_VALUE_1

        // write SP_EL2 register
        msr             sp_el2, x13

        // read back the value of SP_EL2
        mrs             x14, sp_el2

        // initialize x28 with golden signature (0x6969696969696969)
        ldr             x28, =A53_STL_BM260_GOLDEN_VALUE_1

        cmp             x28, x14
        b.ne            error_BM_260

        // initialize x15 with 0x9696969696969696
        ldr             x15, =A53_STL_BM260_INPUT_VALUE_2

        // write SP_EL2 register
        msr             sp_el2, x15

        // read back the value of SP_EL2
        mrs             x16, sp_el2

        // initialize x28 with golden signature (0x9696969696969696)
        ldr             x28, =A53_STL_BM260_GOLDEN_VALUE_2

check_correct_result_BM_260:
        cmp             x28, x16
        b.ne            error_BM_260
        b               success_BM_260

error_BM_260:
        // restore SP_EL2 register from x2
        msr             sp_el2, x2
        b               error

success_BM_260:
        // restore SP_EL2 register from x2
        msr             sp_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 260
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 262
//-----------------------------------------------------------

// Basic Module 262
// Testing CNTFRQ_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTFRQ_EL0 in x6
        mrs             x6, cntfrq_el0

        // initialize x2 with 0x9696969696969696
        ldr             x2, =A53_STL_BM262_INPUT_VALUE_1

        // write CNTFRQ_EL0 register
        msr             cntfrq_el0, x2

        // read back the value of CNTFRQ_EL0
        mrs             x3, cntfrq_el0

        // initialize x28 with golden signature (0x96969696)
        ldr             x28, =A53_STL_BM262_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_262

        // initialize x4 with 0x6969696969696969
        ldr             x4, =A53_STL_BM262_INPUT_VALUE_2

        // write CNTFRQ_EL0 register
        msr             cntfrq_el0, x4

        // read back the value of CNTFRQ_EL0
        mrs             x5, cntfrq_el0

        // initialize x28 with golden signature (0x69696969)
        ldr             x28, =A53_STL_BM262_GOLDEN_VALUE_2

check_correct_result_BM_262:
        cmp             x28, x5
        b.ne            error_BM_262
        b               success_BM_262

error_BM_262:
        // restore CNTFRQ_EL0 register from x6
        msr             cntfrq_el0, x6
        b               error

success_BM_262:
        // restore CNTFRQ_EL0 register from x6
        msr             cntfrq_el0, x6

//-----------------------------------------------------------
// END BASIC MODULE 262
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 274
//-----------------------------------------------------------

// Basic Module 274
// Testing L2ECTLR_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of L2ECTLR_EL1 in x2
        mrs             x2, s3_1_c11_c0_3

        // initialize x18 with 0xAAAAAAAAAAAAAAAA
        ldr             x18, =A53_STL_BM274_INPUT_VALUE_1

        // write L2ECTLR_EL1 register
        msr             s3_1_c11_c0_3, x18

        // read back the value of L2ECTLR_EL1
        mrs             x19, s3_1_c11_c0_3

        // initialize x28 with golden signature (0x00000002)
        ldr             x28, =A53_STL_BM274_GOLDEN_VALUE_1

        cmp             x28, x19
        b.ne            error_BM_274

        // initialize x20 with 0x5555555555555555
        ldr             x20, =A53_STL_BM274_INPUT_VALUE_2

        // write L2ECTLR_EL1 register
        msr             s3_1_c11_c0_3, x20

        // read back the value of L2ECTLR_EL1
        mrs             x21, s3_1_c11_c0_3

        // initialize x28 with golden signature (0x00000005)
        ldr             x28, =A53_STL_BM274_GOLDEN_VALUE_2

check_correct_result_BM_274:
        cmp             x28, x21
        b.ne            error_BM_274
        b               success_BM_274

error_BM_274:
        // restore L2ECTLR_EL1 register from x2
        msr             s3_1_c11_c0_3, x2
        b               error

success_BM_274:
        // restore L2ECTLR_EL1 register from x2
        msr             s3_1_c11_c0_3, x2

//-----------------------------------------------------------
// END BASIC MODULE 274
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

a53_stl_win_c_p054_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p054_n001_end:

        ret

        .end
