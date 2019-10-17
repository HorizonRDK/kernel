/*
 * This Synopsys software and associated documentation (hereinafter the
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you. The
 * Software IS NOT an item of Licensed Software or a Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Products
 * with Synopsys or any supplement thereto. Synopsys is a registered trademark
 * of Synopsys, Inc. Other names included in the SOFTWARE may be the
 * trademarks of their respective owners.
 *
 * The contents of this file are dual-licensed; you may select either version
 * 2 of the GNU General Public License ("GPL") or the BSD-3-Clause license
 * ("BSD-3-Clause"). The GPL is included in the COPYING file accompanying the
 * SOFTWARE. The BSD License is copied below.
 *
 * BSD-3-Clause License:
 * Copyright (c) 2011-2015 Synopsys, Inc. and/or its affiliates.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions, and the following disclaimer, without
 *    modification.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of the above-listed copyright holders may not be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ELPPKA_HW_H_
#define ELPPKA_HW_H_

/* Control/status registers */
enum {
   PKA_CTRL = 0,
   PKA_ENTRY,
   PKA_RC,
   PKA_BUILD_CONF,
   PKA_F_STACK,
   PKA_INST_SINCE_GO,
   PKA_P_STACK,
   PKA_CONF,
   PKA_STATUS,
   PKA_FLAGS,
   PKA_WATCHDOG,
   PKA_CYCLES_SINCE_GO,
   PKA_INDEX_I,
   PKA_INDEX_J,
   PKA_INDEX_K,
   PKA_INDEX_L,
   PKA_IRQ_EN,
   PKA_DTA_JUMP,
   PKA_LFSR_SEED,

   PKA_BANK_SWITCH_A = 20,
   PKA_BANK_SWITCH_B,
   PKA_BANK_SWITCH_C,
   PKA_BANK_SWITCH_D,

   PKA_OPERAND_A_BASE = 0x100,
   PKA_OPERAND_B_BASE = 0x200,
   PKA_OPERAND_C_BASE = 0x300,
   PKA_OPERAND_D_BASE = 0x400,

   /* F/W base for old cores */
   PKA_FIRMWARE_BASE = 0x800,

   /* F/W base for new ("type 2") cores, with fixed RAM/ROM split offset */
   PKA_FIRMWARE_T2_BASE = 0x1000,
   PKA_FIRMWARE_T2_SPLIT = 0x1800
};

#define PKA_MAX_OPERAND_SIZE    512 /* 4096 bits */
#define PKA_ECC521_OPERAND_SIZE  66 /* 528 bits */
#define PKA_OPERAND_BANK_SIZE 0x400

#define PKA_CTRL_GO                31
#define PKA_CTRL_STOP_RQST         27
#define PKA_CTRL_M521_MODE         16
#define PKA_CTRL_M521_MODE_BITS     5
#define PKA_CTRL_BASE_RADIX         8
#define PKA_CTRL_BASE_RADIX_BITS    3
#define PKA_CTRL_PARTIAL_RADIX      0
#define PKA_CTRL_PARTIAL_RADIX_BITS 8

#define PKA_CTRL_M521_ECC521  9

#define PKA_RC_BUSY          31
#define PKA_RC_IRQ           30
#define PKA_RC_WR_PENDING    29
#define PKA_RC_ZERO          28
#define PKA_RC_REASON        16
#define PKA_RC_REASON_BITS    8

#define PKA_BC_FORMAT_TYPE      30
#define PKA_BC_FORMAT_TYPE_BITS  2

/*
 * Bit fields for BUILD_CONF format type 1 (H/W version >= 1.13)
 * Note that format type 2 (H/W version >= 1.17) has the same layout.
 */
#define PKA_BC1_ALU_SZ         19
#define PKA_BC1_ALU_SZ_BITS     2
#define PKA_BC1_RSA_SZ         16
#define PKA_BC1_RSA_SZ_BITS     3
#define PKA_BC1_ECC_SZ         14
#define PKA_BC1_ECC_SZ_BITS     2
#define PKA_BC1_FW_ROM_SZ      11
#define PKA_BC1_FW_ROM_SZ_BITS  3
#define PKA_BC1_FW_RAM_SZ       8
#define PKA_BC1_FW_RAM_SZ_BITS  3
#define PKA_BC1_BANK_SW_D       6
#define PKA_BC1_BANK_SW_D_BITS  2
#define PKA_BC1_BANK_SW_C       4
#define PKA_BC1_BANK_SW_C_BITS  2
#define PKA_BC1_BANK_SW_B       2
#define PKA_BC1_BANK_SW_B_BITS  2
#define PKA_BC1_BANK_SW_A       0
#define PKA_BC1_BANK_SW_A_BITS  2

/* Enumerations for the FW_RAM and FW_ROM fields in format type 1. */
enum {
   PKA_BC1_FW_SZ_0,
   PKA_BC1_FW_SZ_256,
   PKA_BC1_FW_SZ_512,
   PKA_BC1_FW_SZ_1024,
   PKA_BC1_FW_SZ_2048,
};

/* Bit fields for BUILD_CONF format type 0 (H/W version <= 1.12) */
#define PKA_BC_ALU_SZ         19
#define PKA_BC_ALU_SZ_BITS     2
#define PKA_BC_RSA_SZ         16
#define PKA_BC_RSA_SZ_BITS     3
#define PKA_BC_ECC_SZ         14
#define PKA_BC_ECC_SZ_BITS     2
#define PKA_BC_FW_HAS_ROM     13
#define PKA_BC_FW_HAS_RAM     12
#define PKA_BC_FW_ROM_SZ      10
#define PKA_BC_FW_ROM_SZ_BITS  2
#define PKA_BC_FW_RAM_SZ       8
#define PKA_BC_FW_RAM_SZ_BITS  2
#define PKA_BC_BANK_SW_D       6
#define PKA_BC_BANK_SW_D_SZ    2
#define PKA_BC_BANK_SW_C       4
#define PKA_BC_BANK_SW_C_SZ    2
#define PKA_BC_BANK_SW_B       2
#define PKA_BC_BANK_SW_B_SZ    2
#define PKA_BC_BANK_SW_A       0
#define PKA_BC_BANK_SW_A_SZ    2

/* Enumerations for the FW_RAM and FW_ROM fields in format type 0. */
enum {
   PKA_FW_SZ_256,
   PKA_FW_SZ_512,
   PKA_FW_SZ_1024,
   PKA_FW_SZ_2048,
};

#define PKA_STAT_IRQ 30

#define PKA_IRQ_EN_STAT 30

#define PKA_FLAG_ZERO   0
#define PKA_FLAG_MEMBIT 1
#define PKA_FLAG_BORROW 2
#define PKA_FLAG_CARRY  3
#define PKA_FLAG_F0     4
#define PKA_FLAG_F1     5
#define PKA_FLAG_F2     6
#define PKA_FLAG_F3     7

#define PKA_CONF_BYTESWAP 26

#define PKA_DTA_JUMP_PROBABILITY       0
#define PKA_DTA_JUMP_PROBABILITY_BITS 13

#endif
