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
//      Context switch subroutine for tests
//
//   About: Supported Configurations
//      All configurations
//
//   About: Assumption of Use
//      All global assumptions of use apply
//      Local assumptions of use: NONE
//
//   About: TEST_ID
//      N.A.
//
//===========================================================================================//


        #include "../inc/a53_stl_constants.h"
        #include "../inc/a53_stl_utils.h"

//===========================================================================================
//   Function: a53_stl_preamble:
//      Context switch subroutine for tests
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//

        .align 4

        .section .section_a53_stl_preamble,"ax",%progbits
        .global a53_stl_preamble
        .type a53_stl_preamble, %function

a53_stl_preamble:

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]

        bl              a53_stl_save

        // Restore link register from stack
        // Get the lr from 752 offset bytes (752 -> 512 + 240 bytes because of FPU and GPR registers)
#ifdef ALIGN_8
        ldr             x30, [sp, A53_STL_STACK_ADD_752_B]
#else
        ldr             x30, [sp, A53_STL_STACK_ADD_992_B]
#endif
        // init registers in order to obtain indipendency of the MODs

        // Initialise the register bank
        mov             x0,  xzr
        mov             x1,  xzr
        mov             x2,  xzr
        mov             x3,  xzr
        mov             x4,  xzr
        mov             x5,  xzr
        mov             x6,  xzr
        mov             x7,  xzr
        mov             x8,  xzr
        mov             x9,  xzr
        mov             x10, xzr
        mov             x11, xzr
        mov             x12, xzr
        mov             x13, xzr
        mov             x14, xzr
        mov             x15, xzr
        mov             x16, xzr
        mov             x17, xzr
        mov             x18, xzr
        mov             x19, xzr
        mov             x20, xzr
        mov             x21, xzr
        mov             x22, xzr
        mov             x23, xzr
        mov             x24, xzr
        mov             x25, xzr
        mov             x26, xzr
        mov             x27, xzr
        mov             x28, xzr
        mov             x29, xzr

        fmov            d0,  xzr
        fmov            d1,  xzr
        fmov            d2,  xzr
        fmov            d3,  xzr
        fmov            d4,  xzr
        fmov            d5,  xzr
        fmov            d6,  xzr
        fmov            d7,  xzr
        fmov            d8,  xzr
        fmov            d9,  xzr
        fmov            d10, xzr
        fmov            d11, xzr
        fmov            d12, xzr
        fmov            d13, xzr
        fmov            d14, xzr
        fmov            d15, xzr
        fmov            d16, xzr
        fmov            d17, xzr
        fmov            d18, xzr
        fmov            d19, xzr
        fmov            d20, xzr
        fmov            d21, xzr
        fmov            d22, xzr
        fmov            d23, xzr
        fmov            d24, xzr
        fmov            d25, xzr
        fmov            d26, xzr
        fmov            d27, xzr
        fmov            d28, xzr
        fmov            d29, xzr
        fmov            d30, xzr
        fmov            d31, xzr

        // Zero initialise status registers
        msr             spsr_el1, x0

        ret

//===========================================================================================
//   Function: a53_stl_postamble:
//      Context switch subroutine for tests
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//

        .section .section_a53_stl_postamble,"ax",%progbits
        .global a53_stl_postamble
        .type a53_stl_postamble, %function

a53_stl_postamble:

        // Write results on FCTLR[3:0] register bits
#ifdef ALIGN_8
		ldr			  x2, [sp, A53_STL_STACK_E8_OFFSET]   
#else
		ldr             x2, [sp, A53_STL_STACK_1D0_OFFSET]
#endif
        ldr             w3, [x2]

        and             w3, w3, A53_STL_FCTLR_STATUS_MSK_R

        orr             w3, w3, w0

        str             w3, [x2]

        // Save link register into stack
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        str             x30, [sp, A53_STL_STACK_LR_OFFSET]

        bl              a53_stl_restore

        // Restore link register from stack
        // Get the lr from -(752 + 8) offset bytes (752 -> 512 + 240 bytes because of restoring of FPU and GPR registers, 8 bytes because of LR saving)
#ifdef ALIGN_8
        sub             sp, sp, A53_STL_STACK_ADD_752_B
        sub             sp, sp, A53_STL_STACK_PTR_8_BYTE
        ldr             x30, [sp, A53_STL_STACK_LR_OFFSET]
        add             sp, sp, A53_STL_STACK_ADD_752_B
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE
        add             sp, sp, A53_STL_STACK_PTR_8_BYTE
#else
		sub 			sp, sp, A53_STL_STACK_ADD_992_B
		sub 			sp, sp, A53_STL_STACK_PTR_8_BYTE
		ldr 			x30, [sp, A53_STL_STACK_LR_OFFSET]
		add 			sp, sp, A53_STL_STACK_ADD_992_B
		add 			sp, sp, A53_STL_STACK_PTR_8_BYTE
		add 			sp, sp, A53_STL_STACK_PTR_8_BYTE
#endif
        ret

        .end
