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
//      CORE_039
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p039_n001
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

        .section .section_a53_stl_core_p039_n001,"ax",%progbits
        .global a53_stl_core_p039_n001
        .type a53_stl_core_p039_n001, %function

a53_stl_core_p039_n001:

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

a53_stl_win_c_p039:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1

//-----------------------------------------------------------
// START BASIC MODULE 258
//-----------------------------------------------------------

// Basic Module 258
// Testing SP_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SP_EL0 in x2
        mrs             x2, sp_el0

        // initialize x11 with 0x2D2D2D2D2D2D2D2D
        ldr             x11, =A53_STL_BM258_INPUT_VALUE_1

        // write SP_EL0 register
        msr             sp_el0, x11

        // read back the value of SP_EL0
        mrs             x12, sp_el0

        // initialize x28 with golden signature (0x2D2D2D2D2D2D2D2D)
        ldr             x28, =A53_STL_BM258_GOLDEN_VALUE_1

        cmp             x28, x12
        b.ne            error_BM_258

        // initialize x13 with 0xD2D2D2D2D2D2D2D2
        ldr             x13, =A53_STL_BM258_INPUT_VALUE_2

        // write SP_EL0 register
        msr             sp_el0, x13

        // read back the value of SP_EL0
        mrs             x14, sp_el0

        // initialize x28 with golden signature (0xD2D2D2D2D2D2D2D2)
        ldr             x28, =A53_STL_BM258_GOLDEN_VALUE_2

check_correct_result_BM_258:
        cmp             x28, x14
        b.ne            error_BM_258
        b               success_BM_258

error_BM_258:
        // restore SP_EL0 register from x2
        msr             sp_el0, x2
        b               error

success_BM_258:
        // restore SP_EL0 register from x2
        msr             sp_el0, x2

//-----------------------------------------------------------
// END BASIC MODULE 258
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 259
//-----------------------------------------------------------

// Basic Module 259
// Testing SP_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SP_EL1 in x2
        mrs             x2, sp_el1

        // initialize x12 with 0xACACACACACACACAC
        ldr             x12, =A53_STL_BM259_INPUT_VALUE_1

        // write SP_EL1 register
        msr             sp_el1, x12

        // read back the value of SP_EL1
        mrs             x13, sp_el1

        // initialize x28 with golden signature (0xACACACACACACACAC)
        ldr             x28, =A53_STL_BM259_GOLDEN_VALUE_1

        cmp             x28, x13
        b.ne            error_BM_259

        // initialize x14 with 0x5353535353535353
        ldr             x14, =A53_STL_BM259_INPUT_VALUE_2

        // write SP_EL1 register
        msr             sp_el1, x14

        // read back the value of SP_EL1
        mrs             x15, sp_el1

        // initialize x28 with golden signature (0x5353535353535353)
        ldr             x28, =A53_STL_BM259_GOLDEN_VALUE_2

check_correct_result_BM_259:
        cmp             x28, x15
        b.ne            error_BM_259
        b               success_BM_259

error_BM_259:
        // restore SP_EL1 register from x2
        msr             sp_el1, x2
        b               error

success_BM_259:
        // restore SP_EL1 register from x2
        msr             sp_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 259
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 261
//-----------------------------------------------------------

// Basic Module 261
// Testing TPIDRRO_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of TPIDRRO_EL0 in x6
        mrs             x6, tpidrro_el0

        // initialize x2 with 0x9696969696969696
        ldr             x2, =A53_STL_BM261_INPUT_VALUE_1

        // write TPIDRRO_EL0 register
        msr             tpidrro_el0, x2

        // read back the value of TPIDRRO_EL0
        mrs             x3, tpidrro_el0

        // initialize x28 with golden signature (0x9696969696969696)
        ldr             x28, =A53_STL_BM261_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_261

        // initialize x4 with 0x6969696969696969
        ldr             x4, =A53_STL_BM261_INPUT_VALUE_2

        // write TPIDRRO_EL0 register
        msr             tpidrro_el0, x4

        // read back the value of TPIDRRO_EL0
        mrs             x5, tpidrro_el0

        // initialize x28 with golden signature (0x6969696969696969)
        ldr             x28, =A53_STL_BM261_GOLDEN_VALUE_2

check_correct_result_BM_261:
        cmp             x28, x5
        b.ne            error_BM_261
        b               success_BM_261

error_BM_261:
        // restore TPIDRRO_EL0 register from x6
        msr             tpidrro_el0, x6
        b               error

success_BM_261:
        // restore TPIDRRO_EL0 register from x6
        msr             tpidrro_el0, x6

//-----------------------------------------------------------
// END BASIC MODULE 261
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 263
//-----------------------------------------------------------

// Basic Module 263
// Testing CNTHCTL_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTHCTL_EL2 in x6
        mrs             x6, cnthctl_el2

        // initialize x2 with 0xCCCCCCCCCCCCCCCC
        ldr             x2, =A53_STL_BM263_INPUT_VALUE_1

        // write CNTHCTL_EL2 register
        msr             cnthctl_el2, x2

        // read back the value of CNTHCTL_EL2
        mrs             x3, cnthctl_el2

        // initialize x28 with golden signature (0x000000CC)
        ldr             x28, =A53_STL_BM263_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_263

        // initialize x4 with 0x3333333333333333
        ldr             x4, =A53_STL_BM263_INPUT_VALUE_2

        // write CNTHCTL_EL2 register
        msr             cnthctl_el2, x4

        // read back the value of CNTHCTL_EL2
        mrs             x5, cnthctl_el2

        // initialize x28 with golden signature (0x00000033)
        ldr             x28, =A53_STL_BM263_GOLDEN_VALUE_2

check_correct_result_BM_263:
        cmp             x28, x5
        b.ne            error_BM_263
        b               success_BM_263

error_BM_263:
        // restore CNTHCTL_EL2 register from x6
        msr             cnthctl_el2, x6
        b               error

success_BM_263:
        // restore CNTHCTL_EL2 register from x6
        msr             cnthctl_el2, x6

//-----------------------------------------------------------
// END BASIC MODULE 263
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 264
//-----------------------------------------------------------

// Basic Module 264
// Testing CNTHP_CTL_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTHP_CTL_EL2 in x6
        mrs             x6, cnthp_ctl_el2

        // initialize x2 with 0x8888888888888888
        ldr             x2, =A53_STL_BM264_INPUT_VALUE_1

        // write CNTHP_CTL_EL2 register
        msr             cnthp_ctl_el2, x2

        // read back the value of CNTHP_CTL_EL2
        mrs             x3, cnthp_ctl_el2

        // initialize x28 with golden signature (0x00000000)
        ldr             x28, =A53_STL_BM264_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_264

        // initialize x4 with 0x7777777777777776
        ldr             x4, =A53_STL_BM264_INPUT_VALUE_2

        // write CNTHP_CTL_EL2 register
        msr             cnthp_ctl_el2, x4

        // read back the value of CNTHP_CTL_EL2
        mrs             x5, cnthp_ctl_el2

        // initialize x28 with golden signature (0x00000002)
        ldr             x28, =A53_STL_BM264_GOLDEN_VALUE_2

check_correct_result_BM_264:
        cmp             x28, x5
        b.ne            error_BM_264
        b               success_BM_264

error_BM_264:
        // restore CNTHP_CTL_EL2 register from x6
        msr             cnthp_ctl_el2, x6
        b               error

success_BM_264:
        // restore CNTHP_CTL_EL2 register from x6
        msr             cnthp_ctl_el2, x6

//-----------------------------------------------------------
// END BASIC MODULE 264
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 265
//-----------------------------------------------------------

// Basic Module 265
// Testing CNTHP_CVAL_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTHP_CVAL_EL2 in x6
        mrs             x6, cnthp_cval_el2

        // initialize x2 with 0xDDDDDDDDDDDDDDDD
        ldr             x2, =A53_STL_BM265_INPUT_VALUE_1

        // write CNTHP_CVAL_EL2 register
        msr             cnthp_cval_el2, x2

        // read back the value of CNTHP_CVAL_EL2
        mrs             x3, cnthp_cval_el2

        // initialize x28 with golden signature (0xDDDDDDDDDDDDDDDD)
        ldr             x28, =A53_STL_BM265_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_265

        // initialize x4 with 0x2222222222222222
        ldr             x4, =A53_STL_BM265_INPUT_VALUE_2

        // write CNTHP_CVAL_EL2 register
        msr             cnthp_cval_el2, x4

        // read back the value of CNTHP_CVAL_EL2
        mrs             x5, cnthp_cval_el2

        // initialize x28 with golden signature (0x2222222222222222)
        ldr             x28, =A53_STL_BM265_GOLDEN_VALUE_2

check_correct_result_BM_265:
        cmp             x28, x5
        b.ne            error_BM_265
        b               success_BM_265

error_BM_265:
        // restore CNTHP_CVAL_EL2 register from x6
        msr             cnthp_cval_el2, x6
        b               error

success_BM_265:
        // restore CNTHP_CVAL_EL2 register from x6
        msr             cnthp_cval_el2, x6

//-----------------------------------------------------------
// END BASIC MODULE 265
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 269
//-----------------------------------------------------------

// Basic Module 269
// Testing CNTP_CVAL_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTP_CVAL_EL0 in x2
        mrs             x2, cntp_cval_el0

        // initialize x10 with 0xCCCCCCCCCCCCCCCC
        ldr             x10, =A53_STL_BM269_INPUT_VALUE_1

        // write CNTP_CVAL_EL0 register
        msr             cntp_cval_el0, x10

        // read back the value of CNTP_CVAL_EL0
        mrs             x11, cntp_cval_el0

        // initialize x28 with golden signature (0xCCCCCCCCCCCCCCCC)
        ldr             x28, =A53_STL_BM269_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_269

        // initialize x12 with 0x3333333333333333
        ldr             x12, =A53_STL_BM269_INPUT_VALUE_2

        // write CNTP_CVAL_EL0 register
        msr             cntp_cval_el0, x12

        // read back the value of CNTP_CVAL_EL0
        mrs             x13, cntp_cval_el0

        // initialize x28 with golden signature (0x3333333333333333)
        ldr             x28, =A53_STL_BM269_GOLDEN_VALUE_2

check_correct_result_BM_269:
        cmp             x28, x13
        b.ne            error_BM_269
        b               success_BM_269

error_BM_269:
        // restore CNTP_CVAL_EL0 register from x2
        msr             cntp_cval_el0, x2
        b               error

success_BM_269:
        // restore CNTP_CVAL_EL0 register from x2
        msr             cntp_cval_el0, x2


//-----------------------------------------------------------
// END BASIC MODULE 269
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 271
//-----------------------------------------------------------

// Basic Module 271
// Testing CNTKCTL_EL1 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception
        // Save the value of CNTKCTL_EL1 in x2
        mrs             x2, cntkctl_el1

        // initialize x14 with 0xEEEEEEEEEEEEEEEE
        ldr             x14, =A53_STL_BM271_INPUT_VALUE_1

        // write CNTKCTL_EL1 register
        msr             cntkctl_el1, x14

        // read back the value of CNTKCTL_EL1
        mrs             x15, cntkctl_el1

        // initialize x28 with golden signature (0x000002EE)
        ldr             x28, =A53_STL_BM271_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_271

        // initialize x16 with 0x1111111111111111
        ldr             x16, =A53_STL_BM271_INPUT_VALUE_2

        // write CNTKCTL_EL1 register
        msr             cntkctl_el1, x16

        // read back the value of CNTKCTL_EL1
        mrs             x17, cntkctl_el1

        // initialize x28 with golden signature (0x00000111)
        ldr             x28, =A53_STL_BM271_GOLDEN_VALUE_2

check_correct_result_BM_271:
        cmp             x28, x17
        b.ne            error_BM_271
        b               success_BM_271

error_BM_271:
        // restore CNTKCTL_EL1 register from x2
        msr             cntkctl_el1, x2
        b               error

success_BM_271:
        // restore CNTKCTL_EL1 register from x2
        msr             cntkctl_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 271
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 272
//-----------------------------------------------------------

// Basic Module 272
// Testing CNTV_CVAL_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTV_CVAL_EL0 in x2
        mrs             x2, cntv_cval_el0

        // initialize x14 with 0x2222222222222222
        ldr             x14, =A53_STL_BM272_INPUT_VALUE_1

        // write CNTV_CVAL_EL0 register
        msr             cntv_cval_el0, x14

        // read back the value of CNTV_CVAL_EL0
        mrs             x15, cntv_cval_el0

        // initialize x28 with golden signature (0x2222222222222222)
        ldr             x28, =A53_STL_BM272_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_272

        // initialize x16 with 0xDDDDDDDDDDDDDDDD
        ldr             x16, =A53_STL_BM272_INPUT_VALUE_2

        // write CNTV_CVAL_EL0 register
        msr             cntv_cval_el0, x16

        // read back the value of CNTV_CVAL_EL0
        mrs             x17, cntv_cval_el0

        // initialize x28 with golden signature (0xDDDDDDDDDDDDDDDD)
        ldr             x28, =A53_STL_BM272_GOLDEN_VALUE_2

check_correct_result_BM_272:
        cmp             x28, x17
        b.ne            error_BM_272
        b               success_BM_272

error_BM_272:
        // restore CNTV_CVAL_EL0 register from x2
        msr             cntv_cval_el0, x2
        b               error

success_BM_272:
        // restore CNTV_CVAL_EL0 register from x2
        msr             cntv_cval_el0, x2

//-----------------------------------------------------------
// END BASIC MODULE 272
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 273
//-----------------------------------------------------------

// Basic Module 273
// Testing CNTVOFF_EL2 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTVOFF_EL2 in x2
        mrs             x2, cntvoff_el2

        // initialize x14 with 0x4444444444444444
        ldr             x14, =A53_STL_BM273_INPUT_VALUE_1

        // write CNTVOFF_EL2 register
        msr             cntvoff_el2, x14

        // read back the value of CNTVOFF_EL2
        mrs             x15, cntvoff_el2

        // initialize x28 with golden signature (0x4444444444444444)
        ldr             x28, =A53_STL_BM273_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_273

        // initialize x16 with 0xBBBBBBBBBBBBBBBB
        ldr             x16, =A53_STL_BM273_INPUT_VALUE_2

        // write CNTVOFF_EL2 register
        msr             cntvoff_el2, x16

        // read back the value of CNTVOFF_EL2
        mrs             x17, cntvoff_el2

        // initialize x28 with golden signature (0xBBBBBBBBBBBBBBBB)
        ldr             x28, =A53_STL_BM273_GOLDEN_VALUE_2

check_correct_result_BM_273:
        cmp             x28, x17
        b.ne            error_BM_273
        b               success_BM_273

error_BM_273:
        // restore CNTVOFF_EL2 register from x2
        msr             cntvoff_el2, x2
        b               error

success_BM_273:
        // restore CNTVOFF_EL2 register from x2
        msr             cntvoff_el2, x2


//-----------------------------------------------------------
// END BASIC MODULE 273
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 270
//-----------------------------------------------------------

// Basic Module 270
// Testing SCTLR_EL3 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of SCTLR_EL3 in x2
        mrs             x2, sctlr_el3

        // initialize x10 with 0xAAAAAAAAAAAAAAAA
        ldr             x10, =A53_STL_BM270_INPUT_VALUE_1

        // write SCTLR_EL3 register
        msr             sctlr_el3, x10

        // read back the value of SCTLR_EL3
        mrs             x11, sctlr_el3
        isb

        // initialize x28 with golden signature (0x22C828BA)
        ldr             x28, =A53_STL_BM270_GOLDEN_VALUE_1

        cmp             x28, x11
        b.ne            error_BM_270

        // initialize x12 with 0x5555555555555554
        ldr             x12, =A53_STL_BM270_INPUT_VALUE_2

        // write SCTLR_EL3 register
        msr             sctlr_el3, x12

        // read back the value of SCTLR_EL3
        mrs             x13, sctlr_el3

        // initialize x28 with golden signature (0x50D51914)
        ldr             x28, =A53_STL_BM270_GOLDEN_VALUE_2

check_correct_result_BM_270:
        cmp             x28, x13
        b.ne            error_BM_270
        b               success_BM_270

error_BM_270:
        // restore SCTLR_EL3 register from x2
        msr             sctlr_el3, x2
        isb
        b               error

success_BM_270:
        // restore SCTLR_EL3 register from x2
        msr             sctlr_el3, x2
        isb

//-----------------------------------------------------------
// END BASIC MODULE 270
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 268
//-----------------------------------------------------------

// Basic Module 268
// Testing CTR_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of CTR_EL0
        mrs             x14, ctr_el0

        // initialize x28 with golden signature (0x84444004)
        ldr             x28, =A53_STL_BM268_GOLDEN_VALUE_1

        // Mask bit 13 for both x14 and x28
        orr             x28, x28, A53_STL_BM268_MASK_VALUE_1
        orr             x14, x14, A53_STL_BM268_MASK_VALUE_1

check_correct_result_BM_268:
        cmp             x28, x14
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 268
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 266
//-----------------------------------------------------------

// Basic Module 266
// Testing CNTV_CTL_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTV_CTL_EL0 in x6
        mrs             x6, cntv_ctl_el0

        // initialize x2 with 0x8888888888888888
        ldr             x2, =A53_STL_BM266_INPUT_VALUE_1

        // write CNTV_CTL_EL0 register
        msr             cntv_ctl_el0, x2

        // read back the value of CNTV_CTL_EL0
        mrs             x3, cntv_ctl_el0

        // initialize x28 with golden signature (0x00000000)
        ldr             x28, =A53_STL_BM266_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_266

        // initialize x4 with 0x7777777777777776
        ldr             x4, =A53_STL_BM266_INPUT_VALUE_2

        // write CNTV_CTL_EL0 register
        msr             cntv_ctl_el0, x4

        // read back the value of CNTV_CTL_EL0
        mrs             x5, cntv_ctl_el0

        // initialize x28 with golden signature (0x00000002)
        ldr             x28, =A53_STL_BM266_GOLDEN_VALUE_2

check_correct_result_BM_266:
        cmp             x28, x5
        b.ne            error_BM_266
        b               success_BM_266

error_BM_266:
        // restore CNTV_CTL_EL0 register from x6
        msr             cntv_ctl_el0, x6
        b               error

success_BM_266:
        // restore CNTV_CTL_EL0 register from x6
        msr             cntv_ctl_el0, x6

//-----------------------------------------------------------
// END BASIC MODULE 266
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 267
//-----------------------------------------------------------

// Basic Module 267
// Testing CNTP_CTL_EL0 Register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTP_CTL_EL0 in x6
        mrs             x6, cntp_ctl_el0

        // initialize x2 with 0x8888888888888888
        ldr             x2, =A53_STL_BM267_INPUT_VALUE_1

        // write CNTP_CTL_EL0 register
        msr             cntp_ctl_el0, x2

        // read back the value of CNTP_CTL_EL0
        mrs             x3, cntp_ctl_el0

        // initialize x28 with golden signature (0x00000000)
        ldr             x28, =A53_STL_BM267_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_267

        // initialize x4 with 0x7777777777777776
        ldr             x4, =A53_STL_BM267_INPUT_VALUE_2

        // write CNTP_CTL_EL0 register
        msr             cntp_ctl_el0, x4

        // read back the value of CNTP_CTL_EL0
        mrs             x5, cntp_ctl_el0

        // initialize x28 with golden signature (0x00000002)
        ldr             x28, =A53_STL_BM267_GOLDEN_VALUE_2

check_correct_result_BM_267:
        cmp             x28, x5
        b.ne            error_BM_267
        b               success_BM_267

error_BM_267:
        // restore CNTP_CTL_EL0 register from x6
        msr             cntp_ctl_el0, x6
        b               error

success_BM_267:
        // restore CNTP_CTL_EL0 register from x6
        msr             cntp_ctl_el0, x6

//-----------------------------------------------------------
// END BASIC MODULE 267
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 11
//-----------------------------------------------------------

// Basic Module 11
// CNTV_CTL_EL0

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTV_CTL_EL0 in x6
        mrs             x6, cntv_ctl_el0

        // initialize x2 with 0x8888888888888888
        ldr             x2, =A53_STL_BM11_INPUT_VALUE_1

        // write CNTV_CTL_EL0 register
        msr             cntv_ctl_el0, x2

        // read back the value of CNTV_CTL_EL0
        mrs             x3, cntv_ctl_el0

        // initialize x28 with golden signature (0x00000000)
        ldr             x28, =A53_STL_BM11_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_11

        // initialize x4 with 0x7777777777777776
        ldr             x4, =A53_STL_BM11_INPUT_VALUE_2

        // write CNTV_CTL_EL0 register
        msr             cntv_ctl_el0, x4

        // read back the value of CNTV_CTL_EL0
        mrs             x5, cntv_ctl_el0

        // initialize x28 with golden signature (0x00000002)
        ldr             x28, =A53_STL_BM11_GOLDEN_VALUE_2

check_correct_result_BM_11:
        cmp             x28, x5
        b.ne            error_BM_11
        b               success_BM_11

error_BM_11:
        // restore CNTV_CTL_EL0 register from x6
        msr             cntv_ctl_el0, x6
        b               error

success_BM_11:
        // restore CNTV_CTL_EL0 register from x6
        msr             cntv_ctl_el0, x6


//-----------------------------------------------------------
// END BASIC MODULE 11
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 12
//-----------------------------------------------------------

// Basic Module 12
// CNTP_CTL_EL0

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of CNTP_CTL_EL0 in x6
        mrs             x6, cntp_ctl_el0

        // initialize x2 with 0x8888888888888888
        ldr             x2, =A53_STL_BM12_INPUT_VALUE_1

        // write CNTP_CTL_EL0 register
        msr             cntp_ctl_el0, x2

        // read back the value of CNTP_CTL_EL0
        mrs             x3, cntp_ctl_el0

        // initialize x28 with golden signature (0x00000000)
        ldr             x28, =A53_STL_BM12_GOLDEN_VALUE_1

        cmp             x28, x3
        b.ne            error_BM_12

        // initialize x4 with 0x7777777777777776
        ldr             x4, =A53_STL_BM12_INPUT_VALUE_2

        // write CNTP_CTL_EL0 register
        msr             cntp_ctl_el0, x4

        // read back the value of CNTP_CTL_EL0
        mrs             x5, cntp_ctl_el0

        // initialize x28 with golden signature (0x00000002)
        ldr             x28, =A53_STL_BM12_GOLDEN_VALUE_2

check_correct_result_BM_12:
        cmp             x28, x5
        b.ne            error_BM_12
        b               success_BM_12

error_BM_12:
        // restore CNTP_CTL_EL0 register from x6
        msr             cntp_ctl_el0, x6
        b               error

success_BM_12:
        // restore CNTP_CTL_EL0 register from x6
        msr             cntp_ctl_el0, x6

//-----------------------------------------------------------
// END BASIC MODULE 12
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

a53_stl_win_c_p039_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_core_p039_n001_end:

        ret

        .end
