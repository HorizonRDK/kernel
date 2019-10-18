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

#ifndef ELPCLP800_HW_H_
#define ELPCLP800_HW_H_

/* Control/status registers */
enum {
   CLP800_CTRL,
   CLP800_STAT,
   CLP800_MODE,
   CLP800_SMODE,
   CLP800_IE,
   CLP800_ISTAT,
   CLP800_BUILD_ID,
   CLP800_FEATURES,

   CLP800_RAND_BASE = 0x20>>2,
   CLP800_SEED_BASE = 0x40>>2,

   CLP800_AUTO_RQST = 0x60>>2,
   CLP800_AUTO_AGE,

   CLP800_IA_RDATA = 0x70>>2,
   CLP800_IA_WDATA,
   CLP800_IA_ADDR,
   CLP800_IA_CMD,
   CLP800_TTS = 0x80>>2,
};

#define CLP800_REG_MAX 0x80

/* IRQ bitfields (used for both IE and ISTAT registers) */
#define CLP800_IRQ_GLBL_EN    31
#define CLP800_IRQ_RQST_ALARM  3
#define CLP800_IRQ_AGE_ALARM   2
#define CLP800_IRQ_SEED_DONE   1
#define CLP800_IRQ_RAND_RDY    0

#define CLP800_IRQ_GLBL_EN_MASK    (1ul<<CLP800_IRQ_GLBL_EN)
#define CLP800_IRQ_RQST_ALARM_MASK (1ul<<CLP800_IRQ_RQST_ALARM)
#define CLP800_IRQ_AGE_ALARM_MASK  (1ul<<CLP800_IRQ_AGE_ALARM)
#define CLP800_IRQ_SEED_DONE_MASK  (1ul<<CLP800_IRQ_SEED_DONE)
#define CLP800_IRQ_RAND_RDY_MASK   (1ul<<CLP800_IRQ_RAND_RDY)

#define CLP800_IRQ_ALL_MASK ( CLP800_IRQ_GLBL_EN_MASK \
                           | CLP800_IRQ_RQST_ALARM_MASK \
                           | CLP800_IRQ_AGE_ALARM_MASK \
                           | CLP800_IRQ_SEED_DONE_MASK \
                           | CLP800_IRQ_RAND_RDY_MASK )

/* CTRL register commands */
enum {
   CLP800_CMD_NOP,
   CLP800_CMD_GEN_RAND,
   CLP800_CMD_RAND_RESEED,
   CLP800_CMD_NONCE_RESEED,
};

/* STAT register bitfields */
#define CLP800_STAT_RESEEDING          31
#define CLP800_STAT_GENERATING         30
#define CLP800_STAT_SRVC_RQST          27
#define CLP800_STAT_RESEED_REASON      16
#define CLP800_STAT_RESEED_REASON_BITS  3
#define CLP800_STAT_SEEDED              9
#define CLP800_STAT_SECURE              8
#define CLP800_STAT_R256                3
#define CLP800_STAT_NONCE_MODE          2

/* STAT.RESEED_REASON values */
enum {
   CLP800_RESEED_HOST     = 0,
   CLP800_RESEED_NONCE    = 3,
   CLP800_RESEED_PIN      = 4,
   CLP800_RESEED_UNSEEDED = 7,
};

/* MODE register bitfields */
#define CLP800_MODE_R256               3

/* SMODE register bitfields */
#define CLP800_SMODE_MAX_REJECTS      16
#define CLP800_SMODE_MAX_REJECTS_BITS  8
#define CLP800_SMODE_SECURE            8
#define CLP800_SMODE_NONCE             2

/* FEATURES register bitfields */
#define CLP800_FEATURES_DIAG_LEVEL      4
#define CLP800_FEATURES_DIAG_LEVEL_BITS 3
#define CLP800_FEATURES_SECURE_RST      3
#define CLP800_FEATURES_RAND_SEED       2
#define CLP800_FEATURES_RAND_LEN        0
#define CLP800_FEATURES_RAND_LEN_BITS   2

/* BUILD_ID register bitfields */
#define CLP800_BUILD_ID_EPN           0
#define CLP800_BUILD_ID_EPN_BITS     16
#define CLP800_BUILD_ID_STEPPING     28
#define CLP800_BUILD_ID_STEPPING_BITS 4

/* AUTO_RQST register bitfields */
#define CLP800_AUTO_RQST_RQSTS        0
#define CLP800_AUTO_RQST_RQSTS_BITS  16
#define CLP800_AUTO_RQST_RESOLUTION  16ul

/* AUTO_AGE register bitfields */
#define CLP800_AUTO_AGE_AGE           0
#define CLP800_AUTO_AGE_AGE_BITS     16
#define CLP800_AUTO_AGE_RESOLUTION   (1ul << 26)

/* IA_CMD register bitfields */
#define CLP800_IA_CMD_GO   31
#define CLP800_IA_CMD_WRITE 0

#endif
