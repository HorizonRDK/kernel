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
//      GIC_001
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_gic_p001_n001
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

        .section .section_a53_stl_gic_p001_n001,"ax",%progbits
        .global a53_stl_gic_p001_n001
        .type a53_stl_gic_p001_n001, %function

a53_stl_gic_p001_n001:

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

a53_stl_win_g_p001:

        // Set D,A,I,F bit of PSTATE to 1
        msr             DAIFSet, A53_STL_DAIF_BITS_ALL_1


//-----------------------------------------------------------
// START BASIC MODULE 72
//-----------------------------------------------------------

// Basic Module 72
// ICC_CTLR_EL1 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICC_CTLR_EL3 in x3
        mrs             x3, icc_ctlr_el3

        // Save the value of ICC_CTLR_EL1 in x2
        mrs             x2, icc_ctlr_el1

        // Set icc_ctlr_el3 register to 0x00000410
        ldr             x4, =A53_STL_BM72_INPUT_VALUE_3
        msr             icc_ctlr_el3, x4

        // initialize x14 with 0xAAAAAAAAAAAAAAAA
        ldr             x14, =A53_STL_BM72_INPUT_VALUE_1

        // write ICC_CTLR_EL1 register
        msr             icc_ctlr_el1, x14

        // read back the value of ICC_CTLR_EL1
        mrs             x15, icc_ctlr_el1

        // initialize x28 with golden signature (0x0000000000000402)
        ldr             x28, =A53_STL_BM72_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_72

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM72_INPUT_VALUE_2

        // write ICC_CTLR_EL1 register
        msr             icc_ctlr_el1, x16

        // read back the value of ICC_CTLR_EL1
        mrs             x17, icc_ctlr_el1

        // initialize x28 with golden signature (0x0000000000000400)
        ldr             x28, =A53_STL_BM72_GOLDEN_VALUE_2

check_correct_result_BM_72:
        cmp             x28, x17
        b.ne            error_BM_72
        b               success_BM_72

error_BM_72:
        // restore ICC_CTLR_EL1 register from x2
        msr             icc_ctlr_el1, x2

        // Restore the value of ICC_CTLR_EL3 in x3
        msr             icc_ctlr_el3, x3

        b               error

success_BM_72:
        // restore ICC_CTLR_EL1 register from x2
        msr             icc_ctlr_el1, x2

        // Restore the value of ICC_CTLR_EL3 in x3
        msr             icc_ctlr_el3, x3

//-----------------------------------------------------------
// END BASIC MODULE 72
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 73
//-----------------------------------------------------------

// Basic Module 73
// ICC_CTLR_EL3 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICC_CTLR_EL3 in x2
        mrs             x2, icc_ctlr_el3

        // initialize x14 with 0xAAAAAAAAAAAAAAAA
        ldr             x14, =A53_STL_BM73_INPUT_VALUE_1

        // write ICC_CTLR_EL3 register
        msr             icc_ctlr_el3, x14

        // read back the value of ICC_CTLR_EL3
        mrs             x15, icc_ctlr_el3

        // initialize x28 with golden signature (0x000000000000042A)
        ldr             x28, =A53_STL_BM73_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_73

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM73_INPUT_VALUE_2

        // write ICC_CTLR_EL3 register
        msr             icc_ctlr_el3, x16

        // read back the value of ICC_CTLR_EL3
        mrs             x17, icc_ctlr_el3

        // initialize x28 with golden signature (0x0000000000000455)
        ldr             x28, =A53_STL_BM73_GOLDEN_VALUE_2

check_correct_result_BM_73:
        cmp             x28, x17
        b.ne            error_BM_73
        b               success_BM_73

error_BM_73:
        // restore ICC_CTLR_EL3 register from x2
        msr             icc_ctlr_el3, x2
        b               error

success_BM_73:
        // restore ICC_CTLR_EL3 register from x2
        msr             icc_ctlr_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 73
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 74
//-----------------------------------------------------------

// Basic Module 74
// ICC_PMR_EL1 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICC_PMR_EL1 in x2
        mrs             x2, icc_pmr_el1

        // initialize x14 with 0xAAAAAAAAAAAAAAAA
        ldr             x14, =A53_STL_BM74_INPUT_VALUE_1

        // write ICC_PMR_EL1 register
        msr             icc_pmr_el1, x14

        // read back the value of ICC_PMR_EL1
        mrs             x15, icc_pmr_el1

        // initialize x28 with golden signature (0x00000000000000A8)
        ldr             x28, =A53_STL_BM74_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_74

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM74_INPUT_VALUE_2

        // write ICC_PMR_EL1 register
        msr             icc_pmr_el1, x16

        // read back the value of ICC_PMR_EL1
        mrs             x17, icc_pmr_el1

        // initialize x28 with golden signature (0x0000000000000050)
        ldr             x28, =A53_STL_BM74_GOLDEN_VALUE_2

check_correct_result_BM_74:
        cmp             x28, x17
        b.ne            error_BM_74
        b               success_BM_74

error_BM_74:
        // restore ICC_PMR_EL1 register from x2
        msr             icc_pmr_el1, x2
        b               error

success_BM_74:
        // restore ICC_PMR_EL1 register from x2
        msr             icc_pmr_el1, x2

//-----------------------------------------------------------
// END BASIC MODULE 74
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 75
//-----------------------------------------------------------

// Basic Module 75
// ICC_SRE_EL1 and ICC_SRE_EL3 registers

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICC_SRE_EL3 in x2
        mrs             x2, icc_sre_el3

        // Save the value of ICC_SRE_EL1 in x3
        mrs             x3, icc_sre_el1

        // initialize x14 with 0xAAAAAAAAAAAAAAAB
        ldr             x14, =A53_STL_BM75_INPUT_VALUE_1

        // write ICC_SRE_EL3 register
        msr             icc_sre_el3, x14

        // read back the value of ICC_SRE_EL3
        mrs             x15, icc_sre_el3

        // initialize x28 with golden signature (0x000000000000000B) for ICC_SRE_EL3 register
        ldr             x28, =A53_STL_BM75_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_75

        // read back the value of ICC_SRE_EL1
        mrs             x15, icc_sre_el1

        // initialize x28 with golden signature (0x0000000000000002) for ICC_SRE_EL1 register
        ldr             x28, =A53_STL_BM75_GOLDEN_VALUE_2

        cmp             x28, x15
        b.ne            error_BM_75

        // initialize x14 with 0x5555555555555555
        ldr             x14, =A53_STL_BM75_INPUT_VALUE_2

        // write ICC_SRE_EL3 register
        msr             icc_sre_el3, x14

        // read back the value of ICC_SRE_EL3
        mrs             x15, icc_sre_el3

        // initialize x28 with golden signature (0x0000000000000005) for ICC_SRE_EL3 register
        ldr             x28, =A53_STL_BM75_GOLDEN_VALUE_3

        cmp             x28, x15
        b.ne            success_BM_75

        // read back the value of ICC_SRE_EL1
        mrs             x15, icc_sre_el1

        // initialize x28 with golden signature (0x0000000000000004) for ICC_SRE_EL1 register
        ldr             x28, =A53_STL_BM75_GOLDEN_VALUE_4

check_correct_result_BM_75:
        cmp             x28, x15
        b.ne            error_BM_75
        b               success_BM_75

error_BM_75:
        // restore ICC_SRE_EL1 register from x3
        msr             icc_sre_el1, x3

        // restore ICC_SRE_EL3 register from x2
        msr             icc_sre_el3, x2
        b               error

success_BM_75:
        // restore ICC_SRE_EL1 register from x3
        msr             icc_sre_el1, x3

        // restore ICC_SRE_EL3 register from x2
        msr             icc_sre_el3, x2

//-----------------------------------------------------------
// END BASIC MODULE 75
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 76
//-----------------------------------------------------------

// Basic Module 76
// ICH_AP1R0_EL2 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICH_AP1R0_EL2 in x2
        mrs             x2, ich_ap1r0_el2

        // initialize x14 with 0xAAAAAAAAAAAAAAAA
        ldr             x14, =A53_STL_BM76_INPUT_VALUE_1

        // write ICH_AP1R0_EL2 register
        msr             ich_ap1r0_el2, x14

        // read back the value of ICH_AP1R0_EL2
        mrs             x15, ich_ap1r0_el2

        // initialize x28 with golden signature (0x00000000AAAAAAAA)
        ldr             x28, =A53_STL_BM76_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_76

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM76_INPUT_VALUE_2

        // write ICH_AP1R0_EL2 register
        msr             ich_ap1r0_el2, x16

        // read back the value of ICH_AP1R0_EL2
        mrs             x17, ich_ap1r0_el2

        // initialize x28 with golden signature (0x0000000055555555)
        ldr             x28, =A53_STL_BM76_GOLDEN_VALUE_2

check_correct_result_BM_76:
        cmp             x28, x17
        b.ne            error_BM_76
        b               success_BM_76

error_BM_76:
        // restore ICH_AP1R0_EL2 register from x2
        msr             ich_ap1r0_el2, x2
        b               error

success_BM_76:
        // restore ICH_AP1R0_EL2 register from x2
        msr             ich_ap1r0_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 76
//-----------------------------------------------------------


//-----------------------------------------------------------
// START BASIC MODULE 77
//-----------------------------------------------------------

// Basic Module 77
// ICH_HCR_EL2 register

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Save the value of ICH_HCR_EL2 in x2
        mrs             x2, ich_hcr_el2

        // initialize x14 with 0xAAAAAAAAAAAAAAAA
        ldr             x14, =A53_STL_BM77_INPUT_VALUE_1

        // write ICH_HCR_EL2 register
        msr             ich_hcr_el2, x14

        // read back the value of ICH_HCR_EL2
        mrs             x15, ich_hcr_el2

        // initialize x28 with golden signature (0x00000000A80008AA)
        ldr             x28, =A53_STL_BM77_GOLDEN_VALUE_1

        cmp             x28, x15
        b.ne            error_BM_77

        // initialize x16 with 0x5555555555555555
        ldr             x16, =A53_STL_BM77_INPUT_VALUE_2

        // write ICH_HCR_EL2 register
        msr             ich_hcr_el2, x16

        // read back the value of ICH_HCR_EL2
        mrs             x17, ich_hcr_el2

        // initialize x28 with golden signature (0x0000000050001455)
        ldr             x28, =A53_STL_BM77_GOLDEN_VALUE_2

check_correct_result_BM_77:
        cmp             x28, x17
        b.ne            error_BM_77
        b               success_BM_77

error_BM_77:
        // restore ICH_HCR_EL2 register from x2
        msr             ich_hcr_el2, x2
        b               error

success_BM_77:
        // restore ICH_HCR_EL2 register from x2
        msr             ich_hcr_el2, x2

//-----------------------------------------------------------
// END BASIC MODULE 77
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

a53_stl_win_g_p001_end:

        bl              a53_stl_postamble

        // Restore link register from stack
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE

a53_stl_gic_p001_n001_end:

        ret

        .end
