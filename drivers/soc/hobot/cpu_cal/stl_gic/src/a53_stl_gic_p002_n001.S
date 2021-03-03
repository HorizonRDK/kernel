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
//      Testing of GIC registers
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: AI_GIC, AD-FMSK
//
//   About: TEST_ID
//      GIC_002
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_gic_p002_n001
//      Testing of GIC registers
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

        .section .section_a53_stl_gic_p002_n001,"ax",%progbits
        .global a53_stl_gic_p002_n001
        .type a53_stl_gic_p002_n001, %function

a53_stl_gic_p002_n001:

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

a53_stl_win_g_p002:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1


//-----------------------------------------------------------
// START BASIC MODULE 78
//-----------------------------------------------------------

// Basic Module 78
// ICH_LR0_EL2 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICH_LR0_EL2 in x2
        mrs             x2, ich_lr0_el2

        // initialize x14 with 0xAAAAAAAAAAAAAAAA
        ldr             x14, =A53_STL_BM78_INPUT_VALUE_1

        // write ICH_LR0_EL2 register
        msr             ich_lr0_el2, x14

        // read back the value of ICH_LR0_EL2
        mrs             x15, ich_lr0_el2

        // initialize x28 with golden signature (0xA0A802AA0000AAAA)
        ldr             x28, =A53_STL_BM78_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_78

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM78_INPUT_VALUE_2

        // write ICH_LR0_EL2 register
        msr             ich_lr0_el2, x16

        // read back the value of ICH_LR0_EL2
        mrs             x17, ich_lr0_el2

        // initialize x28 with golden signature (0x5050015500005555)
        ldr             x28, =A53_STL_BM78_GOLDEN_VALUE_2

check_correct_result_BM_78:
        cmp             x28, x17
        b.ne            error_BM_78
        b               success_BM_78

error_BM_78:
        // restore ICH_LR0_EL2 register from x2
        msr             ich_lr0_el2, x2
        b               error

success_BM_78:
        // restore ICH_LR0_EL2 register from x2
        msr             ich_lr0_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 78
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 79
//-----------------------------------------------------------

// Basic Module 79
// ICH_LR1_EL2 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICH_LR1_EL2 in x2
        mrs             x2, ich_lr1_el2

        // initialize x14 with 0xAAAAAAAAAAAAAAAA
        ldr             x14, =A53_STL_BM79_INPUT_VALUE_1

        // write ICH_LR1_EL2 register
        msr             ich_lr1_el2, x14

        // read back the value of ICH_LR1_EL2
        mrs             x15, ich_lr1_el2

        // initialize x28 with golden signature (0xA0A802AA0000AAAA)
        ldr             x28, =A53_STL_BM79_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_79

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM79_INPUT_VALUE_2

        // write ICH_LR1_EL2 register
        msr             ich_lr1_el2, x16

        // read back the value of ICH_LR1_EL2
        mrs             x17, ich_lr1_el2

        // initialize x28 with golden signature (0x5050015500005555)
        ldr             x28, =A53_STL_BM79_GOLDEN_VALUE_2

check_correct_result_BM_79:
        cmp             x28, x17
        b.ne            error_BM_79
        b               success_BM_79

error_BM_79:
        // restore ICH_LR1_EL2 register from x2
        msr             ich_lr1_el2, x2
        b               error

success_BM_79:
        // restore ICH_LR1_EL2 register from x2
        msr             ich_lr1_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 79
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 81
//-----------------------------------------------------------

// Basic Module 81
// ICH_LR2_EL2 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICH_LR2_EL2 in x2
        mrs             x2, ich_lr2_el2

        // initialize x14 with 0xAAAAAAAAAAAAAAAA
        ldr             x14, =A53_STL_BM81_INPUT_VALUE_1

        // write ICH_LR2_EL2 register
        msr             ich_lr2_el2, x14

        // read back the value of ICH_LR2_EL2
        mrs             x15, ich_lr2_el2

        // initialize x28 with golden signature (0xA0A802AA0000AAAA)
        ldr             x28, =A53_STL_BM81_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_81

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM81_INPUT_VALUE_2

        // write ICH_LR2_EL2 register
        msr             ich_lr2_el2, x16

        // read back the value of ICH_LR2_EL2
        mrs             x17, ich_lr2_el2

        // initialize x28 with golden signature (0x5050015500005555)
        ldr             x28, =A53_STL_BM81_GOLDEN_VALUE_2

check_correct_result_BM_81:
        cmp             x28, x17
        b.ne            error_BM_81
        b               success_BM_81

error_BM_81:
        // restore ICH_LR2_EL2 register from x2
        msr             ich_lr2_el2, x2
        b               error

success_BM_81:
        // restore ICH_LR2_EL2 register from x2
        msr             ich_lr2_el2, x2


//-----------------------------------------------------------
// END BASIC MODULE 81
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 82
//-----------------------------------------------------------

// Basic Module 82
// ICH_LR3_EL2 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICH_LR3_EL2 in x2
        mrs             x2, ich_lr3_el2

        // initialize x14 with 0xAAAAAAAAAAAAAAAA
        ldr             x14, =A53_STL_BM82_INPUT_VALUE_1

        // write ICH_LR3_EL2 register
        msr             ich_lr3_el2, x14

        // read back the value of ICH_LR3_EL2
        mrs             x15, ich_lr3_el2

        // initialize x28 with golden signature (0xA0A802AA0000AAAA)
        ldr             x28, =A53_STL_BM82_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_82

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM82_INPUT_VALUE_2

        // write ICH_LR3_EL2 register
        msr             ich_lr3_el2, x16

        // read back the value of ICH_LR3_EL2
        mrs             x17, ich_lr3_el2

        // initialize x28 with golden signature (0x5050015500005555)
        ldr             x28, =A53_STL_BM82_GOLDEN_VALUE_2

check_correct_result_BM_82:
        cmp             x28, x17
        b.ne            error_BM_82
        b               success_BM_82

error_BM_82:
        // restore ICH_LR3_EL2 register from x2
        msr             ich_lr3_el2, x2
        b               error

success_BM_82:
        // restore ICH_LR3_EL2 register from x2
        msr             ich_lr3_el2, x2


//-----------------------------------------------------------
// END BASIC MODULE 82
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 83
//-----------------------------------------------------------

// Basic Module 83
// ICH_VTR_EL2 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // read back the value of ICH_VTR_EL2
        mrs             x15, ich_vtr_el2

        // initialize x28 with golden signature (0x0000000090000003)
        ldr             x28, =A53_STL_BM83_GOLDEN_VALUE_1

        // Mask bit 20 for both x15 and x28
        orr             x28, x28, A53_STL_BM83_MASK_VALUE
        orr             x15, x15, A53_STL_BM83_MASK_VALUE

check_correct_result_BM_83:
        cmp             x28, x15
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 83
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

a53_stl_win_g_p002_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_gic_p002_n001_end:

        ret

        .end
