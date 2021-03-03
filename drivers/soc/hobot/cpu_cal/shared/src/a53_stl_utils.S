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
//      A53 STL utility functions
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

        .align 4

//===========================================================================================
//   Function: a53_stl_getcpuid
//      Context p subroutine for tests
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .a53_stl_getcpuid,"ax",%progbits
        .global a53_stl_getcpuid
        .type a53_stl_getcpuid, %function

a53_stl_getcpuid:

        mrs             x0, mpidr_el1
        ubfx            x0, x0, #A53_STL_MPIDR_EL1_AFF0_LSB, #A53_STL_MPIDR_EL1_AFF_WIDTH
        ret

//===========================================================================================
//   Function: a53_stl_set_regs_error
//      Set data corruption error in the memory mapped registers FFMIR
//      x2 contains the address of the memory mapped registers
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//

        .section .section_a53_stl_set_regs_error,"ax",%progbits
        .global a53_stl_set_regs_error
        .type a53_stl_set_regs_error, %function

a53_stl_set_regs_error:
        // Set Data Corruption (0x4) failure mode on FMID [3:0] FFMIR register bits
        mov             x0, A53_STL_FFMIR_FMID_DATA_COR
        ldr             x3, [x2, A53_STL_FFMIR_OFFSET]
        and             w3, w3, A53_STL_FFMIR_FMID_CLR_MSK
        orr             w3, w3, w0
        str             w3, [x2, A53_STL_FFMIR_OFFSET]

        // A Basic Module encounter an error
        mov             x0, A53_STL_FCTLR_STATUS_FAIL

        ret

//===========================================================================================
//   Function: a53_stl_set_fctlr_ping
//      Set PING status in the memory mapped registers FCTLR
//      x2 contains the address of the memory mapped registers
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//

        .section .section_a53_stl_set_fctlr_ping,"ax",%progbits
        .global a53_stl_set_fctlr_ping
        .type a53_stl_set_fctlr_ping, %function

a53_stl_set_fctlr_ping:

        // Set PING status in FCTLR register
        ldr             x3, [x2, A53_STL_FCTLR_OFFSET]
        mov             x0, A53_STL_FCTLR_STATUS_PING
        str             w0, [x2, A53_STL_FCTLR_OFFSET]

        ret


//===========================================================================================
//   Function: a53_stl_set_ffmir_exception
//      Set exception error in the memory mapped registers FFMIR
//      x2 contains the address of the memory mapped registers
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//

        .section .section_a53_stl_set_ffmir_exception,"ax",%progbits
        .global a53_stl_set_ffmir_exception
        .type a53_stl_set_ffmir_exception, %function

a53_stl_set_ffmir_exception:
        mov             x0, A53_STL_FFMIR_FMID_EXCEPTION
        ldr             x3, [x2, A53_STL_FFMIR_OFFSET]
        and             w3, w3, A53_STL_FFMIR_FMID_CLR_MSK
        orr             w3, w3, w0
        str             w3, [x2, A53_STL_FFMIR_OFFSET]

        ret

//===========================================================================================
//   Function: fmov_d2d17_x2x15
//      Function to d2-d17 from x2-x15
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_d2d17_x2x15,"ax",%progbits
        .global fmov_d2d17_x2x15
        .type fmov_d2d17_x2x15, %function
fmov_d2d17_x2x15:
        fmov            d2, x2
        fmov            d3, x3
        fmov            d4, x2
        fmov            d5, x3
        fmov            d6, x6
        fmov            d7, x7
        fmov            d8, x6
        fmov            d9, x7
        fmov            d10, x10
        fmov            d11, x11
        fmov            d12, x10
        fmov            d13, x11
        fmov            d14, x14
        fmov            d15, x15
        fmov            d16, x14
        fmov            d17, x15
        ret

//===========================================================================================
//   Function: fmov_v2v17_x2x15
//      Function to d2-d17 from x2-x15
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_v2v17_x2x15,"ax",%progbits
        .global fmov_v2v17_x2x15
        .type fmov_v2v17_x2x15, %function
fmov_v2v17_x2x15:
        fmov            v2.d[1], x2
        fmov            v3.d[1], x3
        fmov            v4.d[1], x2
        fmov            v5.d[1], x3
        fmov            v6.d[1], x6
        fmov            v7.d[1], x7
        fmov            v8.d[1], x6
        fmov            v9.d[1], x7
        fmov            v10.d[1], x10
        fmov            v11.d[1], x11
        fmov            v12.d[1], x10
        fmov            v13.d[1], x11
        fmov            v14.d[1], x14
        fmov            v15.d[1], x15
        fmov            v16.d[1], x14
        fmov            v17.d[1], x15
        ret

//===========================================================================================
//   Function: fmov_d2d9_x2x9
//      Function to d2-d9 from x2-x9
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_d2d9_x2x9,"ax",%progbits
        .global fmov_d2d9_x2x9
        .type fmov_d2d9_x2x9, %function
fmov_d2d9_x2x9:

        fmov            d2, x2
        fmov            d3, x3
        fmov            d4, x4
        fmov            d5, x5
        fmov            d6, x6
        fmov            d7, x7
        fmov            d8, x8
        fmov            d9, x9
        ret

//===========================================================================================
//   Function: fmov_d1d4_x1x2
//      Function to d1-d4 from x1-x2
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_d1d4_x1x2,"ax",%progbits
        .global fmov_d1d4_x1x2
        .type fmov_d1d4_x1x2, %function
fmov_d1d4_x1x2:
        fmov            d1, x1
        fmov            d2, x2
        fmov            d3, x1
        fmov            d4, x2
        ret

//===========================================================================================
//   Function: fmov_v1v4_x3x4
//      Function to v1-v4 from x3-x4
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_v1v4_x3x4,"ax",%progbits
        .global fmov_v1v4_x3x4
        .type fmov_v1v4_x3x4, %function
fmov_v1v4_x3x4:

        fmov            v1.d[1], x3
        fmov            v2.d[1], x4
        fmov            v3.d[1], x3
        fmov            v4.d[1], x4
        ret

//===========================================================================================
//   Function: fmov_v1v4_x1x2
//      Function to v1-v4 from x1-x2
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_v1v4_x1x2,"ax",%progbits
        .global fmov_v1v4_x1x2
        .type fmov_v1v4_x1x2, %function
fmov_v1v4_x1x2:
        fmov            v1.d[1], x1
        fmov            v2.d[1], x2
        fmov            v3.d[1], x1
        fmov            v4.d[1], x2
        ret

//===========================================================================================
//   Function: fmov_x18x25_d18d25
//      Function to x18-x25 from d18-d25
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_x18x25_d18d25,"ax",%progbits
        .global fmov_x18x25_d18d25
        .type fmov_x18x25_d18d25, %function
fmov_x18x25_d18d25:
        fmov            x18, d18
        fmov            x19, d19
        fmov            x20, d20
        fmov            x21, d21
        fmov            x22, d22
        fmov            x23, d23
        fmov            x24, d24
        fmov            x25, d25
        ret

//===========================================================================================
//   Function: fmov_x18x25_v18v25
//      Function to x18-x25 from v18-v25
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_x18x25_v18v25,"ax",%progbits
        .global fmov_x18x25_v18v25
        .type fmov_x18x25_v18v25, %function
fmov_x18x25_v18v25:
        fmov            x18, v18.d[1]
        fmov            x19, v19.d[1]
        fmov            x20, v20.d[1]
        fmov            x21, v21.d[1]
        fmov            x22, v22.d[1]
        fmov            x23, v23.d[1]
        fmov            x24, v24.d[1]
        fmov            x25, v25.d[1]
        ret

//===========================================================================================
//   Function: fmov_d2d17_x4x17
//      Function to d2-d17 from x4-x17
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_d2d17_x4x17,"ax",%progbits
        .global fmov_d2d17_x4x17
        .type fmov_d2d17_x4x17, %function
fmov_d2d17_x4x17:
        fmov            d2, x4
        fmov            d3, x5
        fmov            d4, x4
        fmov            d5, x5
        fmov            d6, x8
        fmov            d7, x9
        fmov            d8, x8
        fmov            d9, x9
        fmov            d10, x12
        fmov            d11, x13
        fmov            d12, x12
        fmov            d13, x13
        fmov            d14, x16
        fmov            d15, x17
        fmov            d16, x16
        fmov            d17, x17
        ret

//===========================================================================================
//   Function: fmov_d1d8_x1x4
//      Function to d1-d8 from x1-x4
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_d1d8_x1x4,"ax",%progbits
        .global fmov_d1d8_x1x4
        .type fmov_d1d8_x1x4, %function
fmov_d1d8_x1x4:
        fmov            d1, x1
        fmov            d2, x2
        fmov            d3, x1
        fmov            d4, x2
        fmov            d5, x3
        fmov            d6, x4
        fmov            d7, x3
        fmov            d8, x4
        ret

//===========================================================================================
//   Function: fmov_d1d8_x1x2x3x4
//      Function to d1-d8 from x1-x2-x3-x4
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_d1d8_x1x2x3x4,"ax",%progbits
        .global fmov_d1d8_x1x2x3x4
        .type fmov_d1d8_x1x2x3x4, %function
fmov_d1d8_x1x2x3x4:
        fmov            d1, x1
        fmov            d2, x2
        fmov            d3, x3
        fmov            d4, x4
        fmov            d5, x1
        fmov            d6, x2
        fmov            d7, x3
        fmov            d8, x4
        ret

//===========================================================================================
//   Function: fmov_v1v8_x1x4
//      Function to v1-v8 from x1-x4
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_v1v8_x1x4,"ax",%progbits
        .global fmov_v1v8_x1x4
        .type fmov_v1v8_x1x4, %function
fmov_v1v8_x1x4:
        fmov            v1.d[1], x1
        fmov            v2.d[1], x2
        fmov            v3.d[1], x1
        fmov            v4.d[1], x2
        fmov            v5.d[1], x3
        fmov            v6.d[1], x4
        fmov            v7.d[1], x3
        fmov            v8.d[1], x4
        ret

//===========================================================================================
//   Function: fmov_d1d8_x1x8
//      Function to d1-d8 from x1-x8
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_d1d8_x1x8,"ax",%progbits
        .global fmov_d1d8_x1x8
        .type fmov_d1d8_x1x8, %function
fmov_d1d8_x1x8:
        fmov            d1, x1
        fmov            d2, x2
        fmov            d3, x3
        fmov            d4, x4
        fmov            d5, x5
        fmov            d6, x6
        fmov            d7, x7
        fmov            d8, x8
        ret

//===========================================================================================
//   Function: fmov_v1v8_x1x8
//      Function to v1-v8 from x1-x8
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_v1v8_x1x8,"ax",%progbits
        .global fmov_v1v8_x1x8
        .type fmov_v1v8_x1x8, %function
fmov_v1v8_x1x8:
        fmov            v1.d[1], x1
        fmov            v2.d[1], x2
        fmov            v3.d[1], x3
        fmov            v4.d[1], x4
        fmov            v5.d[1], x5
        fmov            v6.d[1], x6
        fmov            v7.d[1], x7
        fmov            v8.d[1], x8
        ret

//===========================================================================================
//   Function: mov_x1x29_x0
//      Function to mov x1-x29 from x0
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .mov_x1x29_x0,"ax",%progbits
        .global mov_x1x29_x0
        .type mov_x1x29_x0, %function
mov_x1x29_x0:
        mov             x1, x0
        mov             x2, x0
        mov             x3, x0
        mov             x4, x0
        mov             x5, x0
        mov             x6, x0
        mov             x7, x0
        mov             x8, x0
        mov             x9, x0
        mov             x10, x0
        mov             x11, x0
        mov             x12, x0
        mov             x13, x0
        mov             x14, x0
        mov             x15, x0
        mov             x16, x0
        mov             x17, x0
        mov             x18, x0
        mov             x19, x0
        mov             x20, x0
        mov             x21, x0
        mov             x22, x0
        mov             x23, x0
        mov             x24, x0
        mov             x25, x0
        mov             x26, x0
        mov             x27, x0
        mov             x28, x0
        mov             x29, x0
        ret

//===========================================================================================
//   Function: fmov_d0d31_x0
//      Function to mov d0-d31 from x0
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_d0d31_x0,"ax",%progbits
        .global fmov_d0d31_x0
        .type fmov_d0d31_x0, %function
fmov_d0d31_x0:
        fmov            d0, x0
        fmov            d1, x0
        fmov            d2, x0
        fmov            d3, x0
        fmov            d4, x0
        fmov            d5, x0
        fmov            d6, x0
        fmov            d7, x0
        fmov            d8, x0
        fmov            d9, x0
        fmov            d10, x0
        fmov            d11, x0
        fmov            d12, x0
        fmov            d13, x0
        fmov            d14, x0
        fmov            d15, x0
        fmov            d16, x0
        fmov            d17, x0
        fmov            d18, x0
        fmov            d19, x0
        fmov            d20, x0
        fmov            d21, x0
        fmov            d22, x0
        fmov            d23, x0
        fmov            d24, x0
        fmov            d25, x0
        fmov            d26, x0
        fmov            d27, x0
        fmov            d28, x0
        fmov            d29, x0
        fmov            d30, x0
        fmov            d31, x0
        ret

//===========================================================================================
//   Function: fmov_v0v31_x0
//      Function to fmov to v0-v31 from x0
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_v0v31_x0,"ax",%progbits
        .global fmov_v0v31_x0
        .type fmov_v0v31_x0, %function
fmov_v0v31_x0:
        fmov            v0.d[1], x0
        fmov            v1.d[1], x0
        fmov            v2.d[1], x0
        fmov            v3.d[1], x0
        fmov            v4.d[1], x0
        fmov            v5.d[1], x0
        fmov            v6.d[1], x0
        fmov            v7.d[1], x0
        fmov            v8.d[1], x0
        fmov            v9.d[1], x0
        fmov            v10.d[1], x0
        fmov            v11.d[1], x0
        fmov            v12.d[1], x0
        fmov            v13.d[1], x0
        fmov            v14.d[1], x0
        fmov            v15.d[1], x0
        fmov            v16.d[1], x0
        fmov            v17.d[1], x0
        fmov            v18.d[1], x0
        fmov            v19.d[1], x0
        fmov            v20.d[1], x0
        fmov            v21.d[1], x0
        fmov            v22.d[1], x0
        fmov            v23.d[1], x0
        fmov            v24.d[1], x0
        fmov            v25.d[1], x0
        fmov            v26.d[1], x0
        fmov            v27.d[1], x0
        fmov            v28.d[1], x0
        fmov            v29.d[1], x0
        fmov            v30.d[1], x0
        fmov            v31.d[1], x0
        ret

//===========================================================================================
//   Function: fmov_x1x29_d0d28
//      Function fmov to x1-x29 from d0-d28
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_x1x29_d0d28,"ax",%progbits
        .global fmov_x1x29_d0d28
        .type fmov_x1x29_d0d28, %function
fmov_x1x29_d0d28:
        fmov            x1, d0
        fmov            x2, d1
        fmov            x3, d2
        fmov            x4, d3
        fmov            x5, d4
        fmov            x6, d5
        fmov            x7, d6
        fmov            x8, d7
        fmov            x9, d8
        fmov            x10, d9
        fmov            x11, d10
        fmov            x12, d11
        fmov            x13, d12
        fmov            x14, d13
        fmov            x15, d14
        fmov            x16, d15
        fmov            x17, d16
        fmov            x18, d17
        fmov            x19, d18
        fmov            x20, d19
        fmov            x21, d20
        fmov            x22, d21
        fmov            x23, d22
        fmov            x24, d23
        fmov            x25, d24
        fmov            x26, d25
        fmov            x27, d26
        fmov            x28, d27
        fmov            x29, d28
        ret

//===========================================================================================
//   Function: fmov_d0d23_x2x16
//      Function fmov to d0-d23 from x2-x16
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_d0d23_x2x16,"ax",%progbits
        .global fmov_d0d23_x2x16
        .type fmov_d0d23_x2x16, %function
fmov_d0d23_x2x16:
        fmov            d0, x2
        fmov            d1, x3
        fmov            d2, x4
        fmov            d3, x2
        fmov            d4, x3
        fmov            d5, x4
        fmov            d6, x6
        fmov            d7, x7
        fmov            d8, x8
        fmov            d9, x6
        fmov            d10, x7
        fmov            d11, x8
        fmov            d12, x10
        fmov            d13, x11
        fmov            d14, x12
        fmov            d15, x10
        fmov            d16, x11
        fmov            d17, x12
        fmov            d18, x14
        fmov            d19, x15
        fmov            d20, x16
        fmov            d21, x14
        fmov            d22, x15
        fmov            d23, x16
        ret

//===========================================================================================
//   Function: fmov_x1x29_v0v28
//      Function fmov to x1-x29 from v0-v28
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_x1x29_v0v28,"ax",%progbits
        .global fmov_x1x29_v0v28
        .type fmov_x1x29_v0v28, %function
fmov_x1x29_v0v28:
        fmov            x1, v0.d[1]
        fmov            x2, v1.d[1]
        fmov            x3, v2.d[1]
        fmov            x4, v3.d[1]
        fmov            x5, v4.d[1]
        fmov            x6, v5.d[1]
        fmov            x7, v6.d[1]
        fmov            x8, v7.d[1]
        fmov            x9, v8.d[1]
        fmov            x10, v9.d[1]
        fmov            x11, v10.d[1]
        fmov            x12, v11.d[1]
        fmov            x13, v12.d[1]
        fmov            x14, v13.d[1]
        fmov            x15, v14.d[1]
        fmov            x16, v15.d[1]
        fmov            x17, v16.d[1]
        fmov            x18, v17.d[1]
        fmov            x19, v18.d[1]
        fmov            x20, v19.d[1]
        fmov            x21, v20.d[1]
        fmov            x22, v21.d[1]
        fmov            x23, v22.d[1]
        fmov            x24, v23.d[1]
        fmov            x25, v24.d[1]
        fmov            x26, v25.d[1]
        fmov            x27, v26.d[1]
        fmov            x28, v27.d[1]
        fmov            x29, v28.d[1]
        ret

//===========================================================================================
//   Function: fmov_x15x8_d18d25
//      Function fmov to x15-x8 from d18-d25
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .fmov_x15x8_d18d25,"ax",%progbits
        .global fmov_x15x8_d18d25
        .type fmov_x15x8_d18d25, %function
fmov_x15x8_d18d25:
        fmov            x15, d18
        fmov            x14, d19
        fmov            x13, d20
        fmov            x12, d21
        fmov            x11, d22
        fmov            x10, d23
        fmov            x9, d24
        fmov            x8, d25
        ret

//===========================================================================================
//   Function: check_x26x29_x15x8_x18x24
//      Function compare x26-x26 to x15-x8 and x18-x19
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .check_x26x29_x15x8_x18x24,"ax",%progbits
        .global check_x26x29_x15x8_x18x24
        .type check_x26x29_x15x8_x18x24, %function
check_x26x29_x15x8_x18x24:
        // compare local and golden signature
        cmp             x26, x15
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x26, x14
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x27, x13
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x27, x12
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x28, x11
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x28, x10
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x29, x9
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x29, x8
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x26, x18
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x26, x19
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x27, x20
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x27, x21
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x28, x22
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x28, x23
        b.ne            check_x26x29_x15x8_x18x24_error

        // compare local and golden signature
        cmp             x29, x24
        b.ne            check_x26x29_x15x8_x18x24_error

        b               check_x26x29_x15x8_x18x24_end

check_x26x29_x15x8_x18x24_error:
        // notify an error corrupting the first expected value
        add             x26, x26, A53_STL_ONE_VALUE

check_x26x29_x15x8_x18x24_end:
        ret

//===========================================================================================
//   Function: check_x26x29_x18x24
//      Function compare x26-x26 to x18-x19
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .check_x26x29_x18x24,"ax",%progbits
        .global check_x26x29_x18x24
        .type check_x26x29_x18x24, %function
check_x26x29_x18x24:
        // compare local and golden signature
        cmp             x26, x18
        b.ne            check_x26x29_x18x24_error

        // compare local and golden signature
        cmp             x26, x19
        b.ne            check_x26x29_x18x24_error

        // compare local and golden signature
        cmp             x27, x20
        b.ne            check_x26x29_x18x24_error

        // compare local and golden signature
        cmp             x27, x21
        b.ne            check_x26x29_x18x24_error

        // compare local and golden signature
        cmp             x28, x22
        b.ne            check_x26x29_x18x24_error

        // compare local and golden signature
        cmp             x28, x23
        b.ne            check_x26x29_x18x24_error

        // compare local and golden signature
        cmp             x29, x24
        b.ne            check_x26x29_x18x24_error

        b               check_x26x29_x18x24_end

check_x26x29_x18x24_error:
        // notify an error corrupting the first expected value
        add             x26, x26, A53_STL_ONE_VALUE

check_x26x29_x18x24_end:
        ret

//===========================================================================================
//   Function: cmp_x1x29_x0
//      Function compare x1-x29 to x0
//
//   Parameters:
//      N.A.
//
//   Returns:
//      N.A.
//===========================================================================================//
        .section .cmp_x1x29_x0,"ax",%progbits
        .global cmp_x1x29_x0
        .type cmp_x1x29_x0, %function
cmp_x1x29_x0:
        // compare register value with golden value
        cmp             x1, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x2, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x3, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x4, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x5, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x6, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x7, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x8, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x9, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x10, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x11, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x12, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x13, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x14, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x15, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x16, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x17, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x18, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x19, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x20, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x21, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x22, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x23, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x24, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x25, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x26, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x27, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x28, x0
        b.ne            cmp_x1x29_x0_error

        // compare register value with golden value
        cmp             x29, x0
        b.ne            cmp_x1x29_x0_error

        b               cmp_x1x29_x0_end

cmp_x1x29_x0_error:
        // notify an error corrupting the first expected value
        add             x0, x0, A53_STL_ONE_VALUE

cmp_x1x29_x0_end:
        ret

        .end
