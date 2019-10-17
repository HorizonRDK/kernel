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
 * Copyright (c) 2015 Synopsys, Inc. and/or its affiliates.
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

#ifndef ELPCLP850_HW_H
#define ELPCLP850_HW_H

/* registers */
#define CLP850_REG_CTRL             0x00
#define CLP850_REG_STAT             0x01
#define CLP850_REG_MODE             0x02
#define CLP850_REG_IE               0x03
#define CLP850_REG_ISTAT            0x04
#define CLP850_REG_ALARM            0x05
#define CLP850_REG_BUILD_ID         0x06
#define CLP850_REG_FEATURES         0x07
#define CLP850_REG_RAND0            0x08
#define CLP850_REG_SEED0            0x0C
#define CLP850_REG_IA_RDATA         0x1C
#define CLP850_REG_IA_WDATA         0x1D
#define CLP850_REG_IA_ADDR          0x1E
#define CLP850_REG_IA_CMD           0x1F

/* CTRL REG */
#define CLP850_REG_CTRL_SEED_NOISE  0x01
#define CLP850_REG_CTRL_SEED_NONCE  0x02
#define CLP850_REG_CTRL_KAT         0x03
#define CLP850_REG_CTRL_ZEROIZE     0x04

/* STAT */
#define _CLP850_REG_STAT_BUSY       31
#define _CLP850_REG_STAT_SECURE      4
#define _CLP850_REG_STAT_NONCE_MODE  3
#define _CLP850_REG_STAT_LAST_CMD    0
#define CLP850_REG_STAT_BUSY(x)           (((x) >> _CLP850_REG_STAT_BUSY) & 1)
#define CLP850_REG_STAT_SECURE(x)         (((x) >> _CLP850_REG_STAT_SECURE) & 1)
#define CLP850_REG_STAT_NONCE_MODE(x)     (((x) >> _CLP850_REG_STAT_NONCE_MODE) & 1)
#define CLP850_REG_STAT_LAST_CMD(x)       (((x) >> _CLP850_REG_STAT_LAST_CMD) & 7)

/* ALARM */
#define CLP850_REG_ALARM_OK              0
#define CLP850_REG_ALARM_KAT_STAT        1
#define CLP850_REG_ALARM_KAT             2
#define CLP850_REG_ALARM_MONOBIT         3
#define CLP850_REG_ALARM_RUN             4
#define CLP850_REG_ALARM_LONG_RUN        5
#define CLP850_REG_ALARM_AUTOCORRELATION 6
#define CLP850_REG_ALARM_POKER           7
#define CLP850_REG_ALARM_REPETITION      8
#define CLP850_REG_ALARM_ADAPTIVE        9

/* MODE */
#define _CLP850_REG_MODE_MAX_REJECTS     16
#define _CLP850_REG_MODE_SECURE_EN        8
#define _CLP850_REG_MODE_NONCE            3
#define _CLP850_REG_MODE_KAT              2
#define _CLP850_REG_MODE_SEC_ALG          1
#define _CLP850_REG_MODE_WAIT_FOR_HT      0

#define _CLP850_REG_MODE_MAX_REJECTS_MASK    255UL
#define _CLP850_REG_MODE_SECURE_EN_MASK        1UL
#define _CLP850_REG_MODE_NONCE_MASK            1UL
#define _CLP850_REG_MODE_KAT_MASK              1UL
#define _CLP850_REG_MODE_SEC_ALG_MASK          1UL
#define _CLP850_REG_MODE_WAIT_FOR_HT_MASK      1UL

#define CLP850_REG_MODE_SET_MAX_REJECTS(y, x) (((y) & ~(_CLP850_REG_MODE_MAX_REJECTS_MASK << _CLP850_REG_MODE_MAX_REJECTS)) | ((x) << _CLP850_REG_MODE_MAX_REJECTS))
#define CLP850_REG_MODE_SET_SECURE_EN(y, x)   (((y) & ~(_CLP850_REG_MODE_SECURE_EN_MASK << _CLP850_REG_MODE_SECURE_EN))     | ((x) << _CLP850_REG_MODE_SECURE_EN))
#define CLP850_REG_MODE_SET_NONCE(y, x)       (((y) & ~(_CLP850_REG_MODE_NONCE_MASK << _CLP850_REG_MODE_NONCE))             | ((x) << _CLP850_REG_MODE_NONCE))
#define CLP850_REG_MODE_SET_KAT(y, x)         (((y) & ~(_CLP850_REG_MODE_KAT_MASK << _CLP850_REG_MODE_KAT))                 | ((x) << _CLP850_REG_MODE_KAT))
#define CLP850_REG_MODE_SET_SEC_ALG(y, x)     (((y) & ~(_CLP850_REG_MODE_SEC_ALG_MASK << _CLP850_REG_MODE_SEC_ALG))         | ((x) << _CLP850_REG_MODE_SEC_ALG))
#define CLP850_REG_MODE_SET_WAIT_FOR_HT(y, x) (((y) & ~(_CLP850_REG_MODE_WAIT_FOR_HT_MASK << _CLP850_REG_MODE_WAIT_FOR_HT)) | ((x) << _CLP850_REG_MODE_WAIT_FOR_HT))

#define CLP850_REG_MODE_GET_MAX_REJECTS(x)  (((x) >> _CLP850_REG_MODE_MAX_REJECTS) & _CLP850_REG_MODE_MAX_REJECTS_MASK)
#define CLP850_REG_MODE_GET_SECURE_EN(x)    (((x) >> _CLP850_REG_MODE_SECURE_EN) & _CLP850_REG_MODE_SECURE_EN_MASK)
#define CLP850_REG_MODE_GET_NONCE(x)        (((x) >> _CLP850_REG_MODE_NONCE) & _CLP850_REG_MODE_NONCE_MASK)
#define CLP850_REG_MODE_GET_KAT(x)          (((x) >> _CLP850_REG_MODE_KAT) & _CLP850_REG_MODE_KAT_MASK)
#define CLP850_REG_MODE_GET_SEC_ALG(x)      (((x) >> _CLP850_REG_MODE_SEC_ALG) & _CLP850_REG_MODE_SEC_ALG_MASK)
#define CLP850_REG_MODE_GET_WAIT_FOR_HT(x)  (((x) >> _CLP850_REG_MODE_WAIT_FOR_HT) & _CLP850_REG_MODE_WAIT_FOR_HT_MASK)

/* IE */
#define CLP850_REG_IE_GLOBAL     31
#define CLP850_REG_IE_ALARM       4
#define CLP850_REG_IE_SEEDED      3
#define CLP850_REG_IE_RND_RDY     2
#define CLP850_REG_IE_KAT_DONE    1
#define CLP850_REG_IE_ZEROIZED    0

/* ISTAT */
#define CLP850_REG_ISTAT_ALARMS   (1ul<<4)
#define CLP850_REG_ISTAT_SEEDED   (1ul<<3)
#define CLP850_REG_ISTAT_RND_RDY  (1ul<<2)
#define CLP850_REG_ISTAT_KAT_DONE (1ul<<1)
#define CLP850_REG_ISTAT_ZEROIZED (1ul<<0)

/* BUILD_ID */
#define _CLP850_REG_BUILD_ID_STEPPING 28
#define _CLP850_REG_BUILD_ID_EPN       0
#define CLP850_REG_BUILD_ID_STEPPING(x)  (((x) >> _CLP850_REG_BUILD_ID_STEPPING) & 31)
#define CLP850_REG_BUILD_ID_EPN(x)       (((x) >> _CLP850_REG_BUILD_ID_EPN) & 0xFFFF)

/* FEATURES */
#define _CLP850_REG_FEATURES_DIAG_LEVEL_TRNG3       7
#define _CLP850_REG_FEATURES_DIAG_LEVEL_ST_HLT      4
#define _CLP850_REG_FEATURES_SECURE_RST_STATE       3
#define CLP850_REG_FEATURES_DIAG_LEVEL_TRNG3(x)  (((x) >> _CLP850_REG_FEATURES_DIAG_LEVEL_TRNG3) & 7)
#define CLP850_REG_FEATURES_DIAG_LEVEL_ST_HLT(x) (((x) >> _CLP850_REG_FEATURES_DIAG_LEVEL_ST_HLT) & 7)
#define CLP850_REG_FEATURES_SECURE_RST_STATE(x)  (((x) >> _CLP850_REG_FEATURES_SECURE_RST_STATE) & 1)

/* IA_CMD */
#define _CLP850_REG_IA_CMD_GO      31
#define _CLP850_REG_IA_CMD_W_nR     0
#define CLP850_REG_IA_CMD_GO       (1ul << _CLP850_REG_IA_CMD_GO)
#define CLP850_REG_IA_CMD_W_nR     (1ul << _CLP850_REG_IA_CMD_W_nR)




#endif
