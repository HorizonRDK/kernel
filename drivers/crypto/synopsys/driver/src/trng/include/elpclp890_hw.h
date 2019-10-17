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
 * Copyright (c) 2012-2016 Synopsys, Inc. and/or its affiliates.
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

#ifndef ELPCLP890_HW_H
#define ELPCLP890_HW_H

/* HW related Parameters */
#define CLP890_RAND_BLK_SIZE_BITS   128

/* registers */
#define CLP890_REG_CTRL             0x00
#define CLP890_REG_MODE             0x01
#define CLP890_REG_SMODE            0x02
#define CLP890_REG_STAT             0x03
#define CLP890_REG_IE               0x04
#define CLP890_REG_ISTAT            0x05
#define CLP890_REG_ALARM            0x06
#define CLP890_REG_BUILD_ID         0x07
#define CLP890_REG_FEATURES         0x08
#define CLP890_REG_RAND0            0x09
#define CLP890_REG_NPA_DATA0        0x0D
#define CLP890_REG_SEED0            0x1D
#define CLP890_REG_IA_RDATA         0x29
#define CLP890_REG_IA_WDATA         0x2A
#define CLP890_REG_IA_ADDR          0x2B
#define CLP890_REG_IA_CMD           0x2C

/* CTRL */
#define CLP890_REG_CTRL_CMD_NOP             0
#define CLP890_REG_CTRL_CMD_GEN_NOISE       1
#define CLP890_REG_CTRL_CMD_GEN_NONCE       2
#define CLP890_REG_CTRL_CMD_CREATE_STATE    3
#define CLP890_REG_CTRL_CMD_RENEW_STATE     4
#define CLP890_REG_CTRL_CMD_REFRESH_ADDIN   5
#define CLP890_REG_CTRL_CMD_GEN_RANDOM      6
#define CLP890_REG_CTRL_CMD_ADVANCE_STATE   7
#define CLP890_REG_CTRL_CMD_KAT             8
#define CLP890_REG_CTRL_CMD_ZEROIZE        15

/* MODE */
#define _CLP890_REG_MODE_ADDIN_PRESENT 4
#define _CLP890_REG_MODE_PRED_RESIST   3
#define _CLP890_REG_MODE_KAT_SEL       2 
#define _CLP890_REG_MODE_KAT_VEC       1
#define _CLP890_REG_MODE_SEC_ALG       0

#define CLP890_REG_MODE_ADDIN_PRESENT (1UL<<_CLP890_REG_MODE_ADDIN_PRESENT)
#define CLP890_REG_MODE_PRED_RESIST   (1UL<<_CLP890_REG_MODE_PRED_RESIST)
#define CLP890_REG_MODE_KAT_SEL       (1UL<<_CLP890_REG_MODE_KAT_SEL)
#define CLP890_REG_MODE_KAT_VEC       (1UL<<_CLP890_REG_MODE_KAT_VEC)
#define CLP890_REG_MODE_SEC_ALG       (1UL<<_CLP890_REG_MODE_SEC_ALG)

/* SMODE */
#define _CLP890_REG_SMODE_MAX_REJECTS 2
#define _CLP890_REG_SMODE_SECURE_EN   1
#define _CLP890_REG_SMODE_NONCE       0

#define CLP890_REG_SMODE_MAX_REJECTS(x) ((x)<<_CLP890_REG_SMODE_MAX_REJECTS)
#define CLP890_REG_SMODE_SECURE_EN(x)   ((x)<<_CLP890_REG_SMODE_SECURE_EN)
#define CLP890_REG_SMODE_NONCE          (1UL<<_CLP890_REG_SMODE_NONCE)

/* STAT */
#define _CLP890_REG_STAT_BUSY         31
#define _CLP890_REG_STAT_DRBG_STATE    7
#define _CLP890_REG_STAT_SECURE        6
#define _CLP890_REG_STAT_NONCE_MODE    5
#define _CLP890_REG_STAT_SEC_ALG       4
#define _CLP890_REG_STAT_LAST_CMD      0

#define CLP890_REG_STAT_BUSY          (1UL<<_CLP890_REG_STAT_BUSY)
#define CLP890_REG_STAT_DRBG_STATE    (1UL<<_CLP890_REG_STAT_DRBG_STATE)
#define CLP890_REG_STAT_SECURE        (1UL<<_CLP890_REG_STAT_SECURE)
#define CLP890_REG_STAT_NONCE_MODE    (1UL<<_CLP890_REG_STAT_NONCE_MODE)
#define CLP890_REG_STAT_SEC_ALG       (1UL<<_CLP890_REG_STAT_SEC_ALG)
#define CLP890_REG_STAT_LAST_CMD(x)   (((x)>>_CLP890_REG_STAT_LAST_CMD)&0xF)

/* IE */
#define _CLP890_REG_IE_GLBL         31
#define _CLP890_REG_IE_DONE          4
#define _CLP890_REG_IE_ALARMS        3
#define _CLP890_REG_IE_NOISE_RDY     2
#define _CLP890_REG_IE_KAT_COMPLETE  1
#define _CLP890_REG_IE_ZEROIZE       0

#define CLP890_REG_IE_GLBL          (1UL<<_CLP890_REG_IE_GLBL)
#define CLP890_REG_IE_DONE          (1UL<<_CLP890_REG_IE_DONE)
#define CLP890_REG_IE_ALARMS        (1UL<<_CLP890_REG_IE_ALARMS)
#define CLP890_REG_IE_NOISE_RDY     (1UL<<_CLP890_REG_IE_NOISE_RDY)
#define CLP890_REG_IE_KAT_COMPLETE  (1UL<<_CLP890_REG_IE_KAT_COMPLETE)
#define CLP890_REG_IE_ZEROIZE       (1UL<<_CLP890_REG_IE_ZEROIZE)

/* ISTAT */
#define _CLP890_REG_ISTAT_DONE          4
#define _CLP890_REG_ISTAT_ALARMS        3
#define _CLP890_REG_ISTAT_NOISE_RDY     2
#define _CLP890_REG_ISTAT_KAT_COMPLETE  1
#define _CLP890_REG_ISTAT_ZEROIZE       0

#define CLP890_REG_ISTAT_DONE          (1UL<<_CLP890_REG_ISTAT_DONE)
#define CLP890_REG_ISTAT_ALARMS        (1UL<<_CLP890_REG_ISTAT_ALARMS)
#define CLP890_REG_ISTAT_NOISE_RDY     (1UL<<_CLP890_REG_ISTAT_NOISE_RDY)
#define CLP890_REG_ISTAT_KAT_COMPLETE  (1UL<<_CLP890_REG_ISTAT_KAT_COMPLETE)
#define CLP890_REG_ISTAT_ZEROIZE       (1UL<<_CLP890_REG_ISTAT_ZEROIZE)

/* ALARMS */
#define CLP890_REG_ALARM_ILLEGAL_CMD_SEQ                      (1UL<<4)
#define CLP890_REG_ALARM_FAILED_TEST_ID_OK                    0
#define CLP890_REG_ALARM_FAILED_TEST_ID_KAT_STAT              1
#define CLP890_REG_ALARM_FAILED_TEST_ID_KAT                   2
#define CLP890_REG_ALARM_FAILED_TEST_ID_MONOBIT               3
#define CLP890_REG_ALARM_FAILED_TEST_ID_RUN                   4
#define CLP890_REG_ALARM_FAILED_TEST_ID_LONGRUN               5
#define CLP890_REG_ALARM_FAILED_TEST_ID_AUTOCORRELATION       6
#define CLP890_REG_ALARM_FAILED_TEST_ID_POKER                 7
#define CLP890_REG_ALARM_FAILED_TEST_ID_REPETITION_COUNT      8
#define CLP890_REG_ALARM_FAILED_TEST_ID_ADAPATIVE_PROPORTION  9

/* BUILD_ID */
#define CLP890_REG_BUILD_ID_STEPPING(x) (((x)>>28)&0xF)
#define CLP890_REG_BUILD_ID_EPN(x)      ((x)&0xFFFF)

/* FEATURES */
#define CLP890_REG_FEATURES_AES_256(x)           (((x)>>9)&1)
#define CLP890_REG_FEATURES_EXTRA_PS_PRESENT(x)  (((x)>>8)&1)
#define CLP890_REG_FEATURES_DIAG_LEVEL_NS(x)     (((x)>>7)&1)
#define CLP890_REG_FEATURES_DIAG_LEVEL_CLP800(x) (((x)>>4)&7)
#define CLP890_REG_FEATURES_DIAG_LEVEL_ST_HLT(x) (((x)>>1)&7)
#define CLP890_REG_FEATURES_SECURE_RST_STATE(x)  ((x)&1)

/* IA_CMD */
#define CLP890_REG_IA_CMD_GO (1UL<<31)
#define CLP890_REG_IA_CMD_WR (1)


#define _CLP890_REG_SMODE_MAX_REJECTS_MASK    255UL
#define _CLP890_REG_SMODE_SECURE_EN_MASK        1UL
#define _CLP890_REG_SMODE_NONCE_MASK            1UL
#define _CLP890_REG_MODE_SEC_ALG_MASK           1UL
#define _CLP890_REG_MODE_ADDIN_PRESENT_MASK     1UL
#define _CLP890_REG_MODE_PRED_RESIST_MASK       1UL
#define _CLP890_REG_MODE_KAT_SEL_MASK           1UL
#define _CLP890_REG_MODE_KAT_VEC_MASK           1UL
#define _CLP890_REG_STAT_DRBG_STATE_MASK        3UL
#define _CLP890_REG_STAT_SECURE_MASK            1UL
#define _CLP890_REG_STAT_NONCE_MASK             1UL

#define CLP890_REG_SMODE_SET_MAX_REJECTS(y, x)  (((y) & ~(_CLP890_REG_SMODE_MAX_REJECTS_MASK << _CLP890_REG_SMODE_MAX_REJECTS)) | ((x) << _CLP890_REG_SMODE_MAX_REJECTS))
#define CLP890_REG_SMODE_SET_SECURE_EN(y, x)    (((y) & ~(_CLP890_REG_SMODE_SECURE_EN_MASK   << _CLP890_REG_SMODE_SECURE_EN))   | ((x) << _CLP890_REG_SMODE_SECURE_EN))
#define CLP890_REG_SMODE_SET_NONCE(y, x)        (((y) & ~(_CLP890_REG_SMODE_NONCE_MASK       << _CLP890_REG_SMODE_NONCE))       | ((x) << _CLP890_REG_SMODE_NONCE))
#define CLP890_REG_SMODE_GET_MAX_REJECTS(x)     (((x) >> _CLP890_REG_SMODE_MAX_REJECTS) & _CLP890_REG_SMODE_MAX_REJECTS_MASK)
#define CLP890_REG_SMODE_GET_SECURE_EN(x)       (((x) >> _CLP890_REG_SMODE_SECURE_EN)   & _CLP890_REG_SMODE_SECURE_EN_MASK)
#define CLP890_REG_SMODE_GET_NONCE(x)           (((x) >> _CLP890_REG_SMODE_NONCE)       & _CLP890_REG_SMODE_NONCE_MASK)

#define CLP890_REG_MODE_SET_SEC_ALG(y, x)       (((y) & ~(_CLP890_REG_MODE_SEC_ALG_MASK       << _CLP890_REG_MODE_SEC_ALG))        | ((x) << _CLP890_REG_MODE_SEC_ALG))
#define CLP890_REG_MODE_SET_PRED_RESIST(y, x)   (((y) & ~(_CLP890_REG_MODE_PRED_RESIST_MASK   << _CLP890_REG_MODE_PRED_RESIST))    | ((x) << _CLP890_REG_MODE_PRED_RESIST))
#define CLP890_REG_MODE_SET_ADDIN_PRESENT(y, x) (((y) & ~(_CLP890_REG_MODE_ADDIN_PRESENT_MASK << _CLP890_REG_MODE_ADDIN_PRESENT))  | ((x) << _CLP890_REG_MODE_ADDIN_PRESENT))
#define CLP890_REG_MODE_SET_KAT_SEL(y, x)       (((y) & ~(_CLP890_REG_MODE_KAT_SEL_MASK << _CLP890_REG_MODE_KAT_SEL)) | ((x) << _CLP890_REG_MODE_KAT_SEL))
#define CLP890_REG_MODE_SET_KAT_VEC(y, x)       (((y) & ~(_CLP890_REG_MODE_KAT_VEC_MASK << _CLP890_REG_MODE_KAT_VEC)) | ((x) << _CLP890_REG_MODE_KAT_VEC))
#define CLP890_REG_MODE_GET_SEC_ALG(x)          (((x) >> _CLP890_REG_MODE_SEC_ALG)       & _CLP890_REG_MODE_SEC_ALG_MASK)
#define CLP890_REG_MODE_GET_PRED_RESIST(x)      (((x) >> _CLP890_REG_MODE_PRED_RESIST)   & _CLP890_REG_MODE_PRED_RESIST_MASK)
#define CLP890_REG_MODE_GET_ADDIN_PRESENT(x)    (((x) >> _CLP890_REG_MODE_ADDIN_PRESENT) & _CLP890_REG_MODE_ADDIN_PRESENT_MASK)
#define CLP890_REG_STAT_GET_DRBG_STATE(x)       (((x) >> _CLP890_REG_STAT_DRBG_STATE) & _CLP890_REG_STAT_DRBG_STATE_MASK)
#define CLP890_REG_STAT_GET_SECURE(x)           (((x) >> _CLP890_REG_STAT_SECURE) & _CLP890_REG_STAT_SECURE_MASK)
#define CLP890_REG_STAT_GET_NONCE(x)            (((x) >> _CLP890_REG_STAT_NONCE_MODE) & _CLP890_REG_STAT_NONCE_MASK)

#endif
