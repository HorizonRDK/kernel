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
 * Copyright (c) 2013 Synopsys, Inc. and/or its affiliates.
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

#ifndef CLP30HW_H_
#define CLP30HW_H_

#define CLP30_OK         0
#define CLP30_ERR       -1
#define CLP30_FIFO_FULL -2
#define CLP30_JOBS_FULL -3
#define CLP30_SEQ_ROLL  -4

#ifndef CLP36_ENABLED
   #define SA_SIZE               0x80
#else
   /* we always use 256 bytes for CLP36 in case SHA-256 was enabled in the config */
   #define SA_SIZE               0x100
#endif

// registers (shared between CLP30/CLP36)
#define CLP30_REG_IRQ_EN      0x00
#define CLP30_REG_IRQ_STAT    0x04
#define CLP30_REG_DMA_BURST   0x08
#define CLP30_REG_IV_RND      0x10

#define CLP30_REG_OUT_SRC_PTR 0x20
#define CLP30_REG_OUT_DST_PTR 0x24
#define CLP30_REG_OUT_OFFSET  0x28
#define CLP30_REG_OUT_ID      0x2C
#define CLP30_REG_OUT_SAI     0x30
#define CLP30_REG_OUT_POP     0x38
#define CLP30_REG_OUT_STAT    0x3C

#define CLP30_REG_IN_SRC_PTR  0x40
#define CLP30_REG_IN_DST_PTR  0x44
#define CLP30_REG_IN_OFFSET   0x48
#define CLP30_REG_IN_ID       0x4C
#define CLP30_REG_IN_SAI      0x50
#define CLP30_REG_IN_POP      0x58
#define CLP30_REG_IN_STAT     0x5C

#ifdef CLP36_ENABLED
   // registers in the CLP36
   #define CLP36_REG_IRQ_CTRL      0x80
   #define CLP36_REG_OUT_FIFO_STAT 0x88
   #define CLP36_REG_IN_FIFO_STAT  0x8C
#endif

// bitfields
#define CLP30_IRQ_EN_OUT_CMD_EN  0x0001
#define CLP30_IRQ_EN_OUT_STAT_EN 0x0002
#define CLP30_IRQ_EN_IN_CMD_EN   0x0004
#define CLP30_IRQ_EN_IN_STAT_EN  0x0008
#define CLP30_IRQ_EN_GLBL_EN     0x80000000

#define CLP30_IRQ_STAT_OUT_CMD   0x0001
#define CLP30_IRQ_STAT_OUT_STAT  0x0002
#define CLP30_IRQ_STAT_IN_CMD    0x0004
#define CLP30_IRQ_STAT_IN_STAT   0x0008

#define CLP30_OFFSET_SRC         0
#define CLP30_OFFSET_DST         16

#define CLP30_STAT_LENGTH(x)           ((x) & 0xFFFF)
#define CLP30_STAT_ID(x)               (((x)>>16)&0xFF)
#define CLP30_STAT_RET_CODE(x)         (((x)>>24)&0xF)
#define CLP30_STAT_CMD_FIFO_FULL(x)    (((x)>>30)&1)
#define CLP30_STAT_STAT_FIFO_EMPTY(x)  (((x)>>31)&1)

#ifdef CLP36_ENABLED
   // CLP36 bitfields
   #define CLP36_IRQ_CTRL_CMD_CNT(x) ((x)&0x7F)
   #define CLP36_IRQ_CTRL_STAT_CNT(x) (((x)&0x7F)<<16)

   #define CLP36_FIFO_STAT_CMD_CNT(x)     ((x)&0x1FF)
   #define CLP36_FIFO_STAT_CMD_FULL(x)    (((x)>>15)&1)
   #define CLP36_FIFO_STAT_STAT_CNT(x)    (((x)>>16)&0xFF)
   #define CLP36_FIFO_STAT_STAT_EMPTY(x)  (((x)>>31)&1)
#endif

// SA fields
#define CLP30_SA_SEQNUM        0x000
#define CLP30_SA_ANTI_REPLAY   0x008
#define CLP30_SA_AUTH_KEY1     0x010
#define CLP30_SA_CIPH_KEY      0x024
#define CLP30_SA_CIPH_IV       0x044
#define CLP30_SA_SALT          0x044
#ifndef CLP36_ENABLED
   #define CLP30_SA_AUTH_KEY2     0x048
#endif
#define CLP30_SA_REMOTE_SPI    0x054
#define CLP30_SA_HARD_TTL_HI   0x06C
#define CLP30_SA_HARD_TTL_LO   0x070
#define CLP30_SA_SOFT_TTL_HI   0x074
#define CLP30_SA_SOFT_TTL_LO   0x078
#define CLP30_SA_ALG           0x07C
#define CLP30_SA_FLAGS         0x07E
#ifdef CLP36_ENABLED
   #define CLP36_SA_AUTH_KEY2     0x080
#endif

#define SA_CTRL_ACTIVE      0x0001
#define SA_CTRL_SEQ_ROLL    0x0002
#define SA_CTRL_TTL_EN      0x0004
#define SA_CTRL_TTL_CTRL    0x0008
#define SA_CTRL_HDR_TYPE    0x0010
#define SA_CTRL_AR_EN       0x0080
#define SA_CTRL_IPV6        0x1000
#define SA_CTRL_DST_OP_MODE 0x2000
#define SA_CTRL_ESN         0x4000

#define MAC_ALG_NULL               0
#define MAC_ALG_HMAC_MD5_96        1
#define MAC_ALG_HMAC_SHA1_96       2
#define MAC_ALG_HMAC_SHA256_128    3
#define MAC_ALG_HMAC_SHA384_192    -1
#define MAC_ALG_HMAC_SHA512_256    -1
#define MAC_ALG_AES_XCBC_MAC_96    -1
#define MAC_ALG_AES_CMAC_96        -1

#define SA_CTRL_MAC_ALG(x)         (x)

#define CLP30_SA_ALG_CIPH_NULL       (0<<4)
#define CLP30_SA_ALG_CIPH_DES_CBC    (1<<4)
#define CLP30_SA_ALG_CIPH_3DES_CBC   (2<<4)
#define CLP30_SA_ALG_CIPH_AES128_CBC (3<<4)
#define CLP30_SA_ALG_CIPH_AES192_CBC (4<<4)
#define CLP30_SA_ALG_CIPH_AES256_CBC (5<<4)
#ifndef CLP36_ENABLED
   #define CLP30_SA_ALG_CIPH_AES128_GCM (6<<4)
   #define CLP30_SA_ALG_CIPH_AES192_GCM (7<<4)
   #define CLP30_SA_ALG_CIPH_AES256_GCM (8<<4)
   #define CLP30_SA_ALG_CIPH_AES128_CTR (9<<4)
   #define CLP30_SA_ALG_CIPH_AES192_CTR (10<<4)
   #define CLP30_SA_ALG_CIPH_AES256_CTR (11<<4)
#endif

#endif
