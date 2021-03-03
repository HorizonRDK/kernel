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
//      Declaration of LUT data vectors
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



#ifndef DRIVERS_SOC_HOBOT_CPU_CAL_SHARED_INC_A53_STL_LUT_DATA_H_
#define DRIVERS_SOC_HOBOT_CPU_CAL_SHARED_INC_A53_STL_LUT_DATA_H_
typedef unsigned long long uint64_t;
typedef unsigned int uint32_t;

//#include <stdint.h>

// Definition f the size of the arrays
#define DATA_SET_00_SDIV_DIM           (96u)
#define DATA_SET_01_FADD_DIM           (132u)
#define DATA_SET_02_FDIV_DIM           (36u)
#define DATA_SET_03_FMUL_DIM           (60u)
#define DATA_SET_04_FADD_DIM           (84u)
#define DATA_SET_05_FMUL_DIM           (60u)
#define DATA_SET_06_FDIV_DIM           (36u)
#define DATA_SET_08_FMADD_DIM          (80u)
#define DATA_SET_09_FMULX_VECT_DIM     (48u)
#define DATA_SET_10_FDIV_VECT_DIM      (48u)
#define DATA_SET_11_FRSQRTE_VECT_DIM   (16u)
#define DATA_SET_12_FADDP_VECT_DIM     (80u)
#define DATA_SET_13_FMADD_S_DIM        (48u)
#define DATA_SET_14_FMSUB_DIM          (80u)
#define DATA_SET_15_FMLA_VECT_DIM      (80u)

// DATA_SET_00_SDIV is for SDIV
extern const uint64_t DATA_SET_00_SDIV[DATA_SET_00_SDIV_DIM];
extern const uint64_t DATA_SET_00_END;
// DATA_SET_01_FADD is for FADD double precision
extern const uint64_t DATA_SET_01_FADD[DATA_SET_01_FADD_DIM];
extern const uint64_t DATA_SET_01_END;
// DATA_SET_02_FDIV is for FDIV double precision
extern const uint64_t DATA_SET_02_FDIV[DATA_SET_02_FDIV_DIM];
extern const uint64_t DATA_SET_02_END;
// DATA_SET_03_FMUL is for FMUL double precision
extern const uint64_t DATA_SET_03_FMUL[DATA_SET_03_FMUL_DIM];
extern const uint64_t DATA_SET_03_END;
// DATA_SET_04_FADD is for FADD single precision
extern const uint64_t DATA_SET_04_FADD[DATA_SET_04_FADD_DIM];
extern const uint64_t DATA_SET_04_END;
// DATA_SET_05_FMUL is for FMUL single precision
extern const uint64_t DATA_SET_05_FMUL[DATA_SET_05_FMUL_DIM];
extern const uint64_t DATA_SET_05_END;
// DATA_SET_06_FDIV is for FDIV single precision
extern const uint64_t DATA_SET_06_FDIV[DATA_SET_06_FDIV_DIM];
extern const uint64_t DATA_SET_06_END;
// DATA_SET_08_FMADD is for FMADD double precision
extern const uint64_t DATA_SET_08_FMADD[DATA_SET_08_FMADD_DIM];
extern const uint64_t DATA_SET_08_END;
// DATA_SET_09_FMULX_V is for FMULX double precision
extern const uint64_t DATA_SET_09_FMULX_V[DATA_SET_09_FMULX_VECT_DIM];
extern const uint64_t DATA_SET_09_END;
// DATA_SET_10_FDIV_V is for FDIV double precision
extern const uint64_t DATA_SET_10_FDIV_V[DATA_SET_10_FDIV_VECT_DIM];
extern const uint64_t DATA_SET_10_END;
// DATA_SET_11_FRSQRTE is for FRSQRTE double precision
extern const uint64_t DATA_SET_11_FRSQRTE[DATA_SET_11_FRSQRTE_VECT_DIM];
extern const uint64_t DATA_SET_11_END;
// DATA_SET_12_FADDP_V is for FADDP double precision
extern const uint64_t DATA_SET_12_FADDP_V[DATA_SET_12_FADDP_VECT_DIM];
extern const uint64_t DATA_SET_12_END;
// DATA_SET_13_FMADD_S is for FMADD single precision
extern const uint32_t DATA_SET_13_FMADD_S[DATA_SET_13_FMADD_S_DIM];
extern const uint64_t DATA_SET_13_END;
// DATA_SET_14_FMSUB is for FMSUB single precision
extern const uint64_t DATA_SET_14_FMSUB[DATA_SET_14_FMSUB_DIM];
extern const uint64_t DATA_SET_14_END;
// DATA_SET_15_FMLA_V is for FMSUB single precision
extern const uint64_t DATA_SET_15_FMLA_V[DATA_SET_15_FMLA_VECT_DIM];
extern const uint64_t DATA_SET_15_END;

#endif //DRIVERS_SOC_HOBOT_CPU_CAL_SHARED_INC_A53_STL_LUT_DATA_H_
