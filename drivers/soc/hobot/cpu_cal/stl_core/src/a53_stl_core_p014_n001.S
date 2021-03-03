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
//      Testing of Jump and Branch instructions (using control flow)
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_014
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p014_n001
//      Testing of Jump and Branch instructions (using control flow
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

        .section .section_a53_stl_core_p014_n001,"ax",%progbits
        .global a53_stl_core_p014_n001
        .type a53_stl_core_p014_n001, %function

a53_stl_core_p014_n001:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]


        // call preamble subroutine

        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 67
//-----------------------------------------------------------

// Basic Module 67
// B.cond

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation register initialization
        mov             x29, A53_STL_SIGN_SEED

        // Init Control Flow Register
        mov             x0, A53_STL_BM67_INIT_0_VALUE
        ldr             x1, =A53_STL_BM67_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM67_INPUT_VALUE_2
        // x8 is used to set nzcv flags to 0
        mov             x8, A53_STL_BM67_INIT_0_VALUE

// EQ
a53_stl_jumplabel_EQ:
        ldr             x0, =A53_STL_BM67_INPUT_VALUE_3
        // x9 used to set Flags
        mov             x9, A53_STL_BM67_JMP1_FLAGS
        // Z=1
        msr             nzcv, x9
        b.eq            a53_stl_jumplabel_MI
        eor             x0, x1, x0
// NE
a53_stl_jumplabel_NE:
        mvn             x0, x2, lsr #A53_STL_BM67_JMP2_LSR_3
        // Z=0
        msr             nzcv, x8
        b.ne            a53_stl_jumplabel_GT
        eor             x0, x2, x0
// CS or HS
a53_stl_jumplabel_CS:
        add             x0, x0, x1, lsl #A53_STL_BM67_JMP3_LSL_1
        mov             x9, A53_STL_BM67_JMP3_FLAGS
        // C=1
        msr             nzcv, x9
        b.cs            a53_stl_jumplabel_HI
        eor             x0, x1, x0
// CC or LO
a53_stl_jumplabel_CC:
        sub             x0, x0, x2, lsl #A53_STL_BM67_JMP4_LSL_2
        // C=0
        msr             nzcv, x8
        b.cc            a53_stl_jumplabel_GE
        eor             x0, x2, x0
// MI
a53_stl_jumplabel_MI:
        add             x0, x0, x1, lsr #A53_STL_BM67_JMP5_LSR_1
        // N=1
        mov             x9, A53_STL_BM67_JMP5_FLAGS
        msr             nzcv, x9
        b.mi            a53_stl_jumplabel_LT
        eor             x0, x1, x0
// PL
a53_stl_jumplabel_PL:
        add             x0, x0, x2, lsl #A53_STL_BM67_JMP6_LSL_4
        // N=0
        msr             nzcv, x8
        b.pl            a53_stl_jumplabel_VC
        eor             x0, x2, x0
// VS
a53_stl_jumplabel_VS:
        sub             x0, x0, x1, lsl #A53_STL_BM67_JMP7_LSL_1
        mov             x9, A53_STL_BM67_JMP7_FLAGS
        // V=1
        msr             nzcv, x9
        b.vs            a53_stl_jumplabel_NE
        eor             x0, x1, x0
// VC
a53_stl_jumplabel_VC:
        sub             x0, x0, x1, lsr #A53_STL_BM67_JMP8_LSR_3
        // V=0
        msr             nzcv, x8
        b.vc            a53_stl_jumplabel_CS
        eor             x0, x1, x0
// HI
a53_stl_jumplabel_HI:
        add             x0, x0, x2, asr #A53_STL_BM67_JMP9_ASR_1
        mov             x9, A53_STL_BM67_JMP9_FLAGS
        // C=1 Z=0
        msr             nzcv, x9
        b.hi            a53_stl_jumplabel_CC
        eor             x0, x2, x0
// LS
a53_stl_jumplabel_LS:
        add             x0, x0, x1, lsl #A53_STL_BM67_JMP10_LSL_4
        mov             x9, A53_STL_BM67_JMP10_FLAGS
        // C=0 Z=1
        msr             nzcv, x9
        b.ls            a53_stl_jumplabel_VS
        eor             x0, x1, x0
// GE
a53_stl_jumplabel_GE:
        sub             x0, x0, x2, lsr #A53_STL_BM67_JMP11_LSR_3
        mov             x9, A53_STL_BM67_JMP11_FLAGS
        // N=V=1
        msr             nzcv, x9
        b.ge            a53_stl_jumplabel_LE
        eor             x0, x2, x0
// LT
a53_stl_jumplabel_LT:
        add             x0, x0, x1, lsr #A53_STL_BM67_JMP12_LSR_4
        // N!=V N=1 V=0
        mov             x9, A53_STL_BM67_JMP12_FLAGS
        msr             nzcv, x9
        b.lt            a53_stl_jumplabel_PL
        eor             x0, x1, x0
// GT
a53_stl_jumplabel_GT:
        add             x0, x0, x2, lsl #A53_STL_BM67_JMP13_LSL_3
        // Z=0
        msr             nzcv, x8
        b.gt            a53_stl_jumplabel_BM67_sign
        eor             x0, x2, x0
// LE
a53_stl_jumplabel_LE:
        sub             x0, x0, x2, lsl #A53_STL_BM67_JMP14_LSL_2
        // Z=1 N!=V N=1 V=0
        mov             x9, A53_STL_BM67_JMP14_FLAGS
        msr             nzcv, x9
        b.le            a53_stl_jumplabel_LS
        eor             x0, x2, x0
// end test
a53_stl_jumplabel_BM67_sign:
        // rotate right to reduce masking effect
        ror             x29, x29, A53_STL_ROTATE_1
        // accumulate the result in BM signature
        eor             x29, x0, x29

        // initialize w28 with golden signature
        ldr             x28, =A53_STL_BM67_GOLDEN_SIGN

check_correct_result_BM_67:
        // compare local and golden signature
        cmp             x28, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 67
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 68
//-----------------------------------------------------------

// Basic Module 68
// Testing BLR, BR, CBZ, TBZ, CBNZ, TBNZ

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation register initialization
        mov             x29, A53_STL_SIGN_SEED

        // Init Control Flow Register
        mov             x0, A53_STL_BM68_INIT_0_VALUE
        ldr             x1, =A53_STL_BM68_INPUT_VALUE_1
        ldr             x2, =A53_STL_BM68_INPUT_VALUE_2

        // save current LR
        sub             sp, sp, A53_STL_BM68_STK_PTR_8_B
        str             x30, [sp, A53_STL_BM68_STACK_OFFSET]

// BLR
a53_stl_jumplabel_BLR:
        ldr             x0, =A53_STL_BM68_INPUT_VALUE_3
        ldr             x9, =a53_stl_jumplabel_CBZ
        blr             x9
        eor             x0, x1, x0
// BR
a53_stl_jumplabel_BR:
        mvn             x0, x2, lsr #A53_STL_BM68_JMP17_LSR_3
        ldr             x9, =a53_stl_jumplabel_CBNZ
        br              x9
        eor             x0, x2, x0
// CBZ
a53_stl_jumplabel_CBZ:
        add             x0, x0, x1, lsl #A53_STL_BM68_JMP18_LSL_1
        mov             x9, A53_STL_BM68_INIT_0_VALUE
        cbz             x9, a53_stl_jumplabel_TBNZ
        eor             x0, x1, x0
// TBZ
a53_stl_jumplabel_TBZ:
        sub             x0, x0, x2, lsl #A53_STL_BM68_JMP19_LSL_2
        mov             x9, A53_STL_BM68_INIT_0_VALUE
        tbz             x9, A53_STL_BM68_IMM_VALUE_0, a53_stl_jumplabel_BR
        eor             x0, x2, x0
// CBNZ
a53_stl_jumplabel_CBNZ:
        add             x0, x0, x1, lsr #A53_STL_BM68_JMP20_LSR_1
        mov             x9, A53_STL_BM68_INIT_1_VALUE
        cbnz            x9, a53_stl_jumplabel_BM68_sign
        eor             x0, x1, x0
// TBNZ
a53_stl_jumplabel_TBNZ:
        add             x0, x0, x2, lsl #A53_STL_BM68_JMP21_LSL_4
        mov             x9, A53_STL_BM68_INIT_1_VALUE
        tbnz            x9, A53_STL_BM68_IMM_VALUE_0, a53_stl_jumplabel_TBZ
        eor             x0, x2, x0

// end test
a53_stl_jumplabel_BM68_sign:
        // rotate right to reduce masking effect
        ror             x29, x29, A53_STL_ROTATE_1
        // accumulate the result in BM signature
        eor             x29, x0, x29
        // restore LR
        ldr             x30, [sp, A53_STL_BM68_STACK_OFFSET]

        // initialize w28 with golden signature
        ldr             x28, =A53_STL_BM68_GOLDEN_SIGN

check_correct_result_BM_68:
        // compare local and golden signature
        cmp             x28, x29
        b.ne            error_BM_68
        b               success_BM_68

error_BM_68:
        // Increment SP
        add             sp, sp, A53_STL_BM68_STK_PTR_8_B
        b               error

success_BM_68:
        // Increment SP
        add             sp, sp, A53_STL_BM68_STK_PTR_8_B
//-----------------------------------------------------------
// END BASIC MODULE 68
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

a53_stl_core_p014_n001_end:

        ret

        .end
