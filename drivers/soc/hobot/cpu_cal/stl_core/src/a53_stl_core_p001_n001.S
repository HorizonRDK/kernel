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
//      Testing of divide instructions
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      CORE_001
//
//===========================================================================================//
//===========================================================================================
//   Function: a53_stl_core_p001_n001
//      Testing of division instructions
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

        .section .section_a53_stl_core_p001_n001,"ax",%progbits
        .global a53_stl_core_p001_n001
        .type a53_stl_core_p001_n001, %function

a53_stl_core_p001_n001:
			

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]

        // call preamble subroutine
        bl              a53_stl_preamble

        // Set PING status in FCTLR register
        ldr             x2, [sp, A53_STL_STACK_FCTLR_ADDR]
        bl              a53_stl_set_fctlr_ping

//-----------------------------------------------------------
// START BASIC MODULE 01
//-----------------------------------------------------------

// Basic Module 01
// SDIV <Xd>, <Xn>, <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // Load LUT address
        ldr             x0, =DATA_SET_00_SDIV

        // Add offset to LUT in order to point first test element (if offset is #0 start from LUT beginning)
        add             x0, x0, A53_STL_LUT_START_OFFSET

        // Load the LUT end address
        ldr             x1, =DATA_SET_00_END
        ldr             x1, [x1]

loop_SDIV:

        // Load a couple of operands from LUT
        ldp             x2, x3, [x0], A53_STL_LUT_PAIR_INCR
        // Load result for current operands from LUT
        ldr             x26, [x0], A53_STL_LUT_SINGLE_INCR

        // SDIV <Xd>, <Xn>, <Xm>

        sdiv            x18, x2, x3

check_correct_result_BM_01:
        // Check if result is correct
        cmp             x18, x26
        b.ne            error

        // Decrement LUT counter to end by 3 elements
        sub             x1, x1, A53_STL_LUT_CNT_INCR_3

        // Check if used all LUT until immediate offset (all operands if immediate = #0)
        cmp             x1, A53_STL_LUT_END_OFFSET
        bne             loop_SDIV

//-----------------------------------------------------------
// END BASIC MODULE 01
//-----------------------------------------------------------

//-----------------------------------------------------------
// START BASIC MODULE 02
//-----------------------------------------------------------

// Basic Module 02
// UDIV <Xd>, <Xn>, <Xm>

        // Set Exception (0x2) failure mode on FMID [3:0] FFMIR register bits
        ldr             x2, [sp, A53_STL_STACK_FFMIR_ADDR]
        bl              a53_stl_set_ffmir_exception

        // BM signature accumulation register initialization
        mov             x29, A53_STL_SIGN_SEED

        // Set operand registers
        mov             x3, A53_STL_BM01_INPUT_VALUE_1
        mov             x4, A53_STL_BM01_INPUT_VALUE_2

        udiv            x3, x3, x4

        // Rotate right to reduce masking effect
        ror             x29, x29, A53_STL_ROTATE_1
        // Accumulate the result in BM signature
        eor             x29, x3, x29

        // Testing udiv with first operand as 0
        mov             x3, A53_STL_BM01_INPUT_VALUE_3
        udiv            x3, x3, x4

        // Rotate right to reduce masking effect
        ror             x29, x29, A53_STL_ROTATE_1
        // Accumulate the result in BM signature
        eor             x29, x3, x29

        // Set operand registers
        mov             x3, A53_STL_BM01_INPUT_VALUE_2
        mov             x4, A53_STL_BM01_INPUT_VALUE_1

        udiv            x3, x3, x4
        // Rotate right to reduce masking effect
        ror             x29, x29, A53_STL_ROTATE_1
        // Accumulate the result in BM signature
        eor             x29, x3, x29

        // Initialize x28 with golden signature
        ldr             x28, =A53_STL_BM02_GOLDEN_SIGN

check_correct_result_BM_02:
        // Compare local and golden signature
        cmp             x28, x29
        b.ne            error

//-----------------------------------------------------------
// END BASIC MODULE 02
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

a53_stl_core_p001_n001_end:

        ret

        .end
