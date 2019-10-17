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

#ifndef EAPEHW_H_
#define EAPEHW_H_

#define EAPE_OK         0
#define EAPE_ERR       -1
#define EAPE_FIFO_FULL -2
#define EAPE_JOBS_FULL -3
#define EAPE_SEQ_ROLL  -4

#define EAPE_SA_SIZE               0x100

#define EAPE_WATCHDOG_TIMEOUT      0xFFFFFF

#define EAPE_OUTBOUND         1
#define EAPE_INBOUND          0
#define EAPE_CTX_BUSY         1
#define EAPE_CTX_FREE         0
#define EAPE_JOB_DONE         1
#define EAPE_JOB_IN_PROGRESS  0

// REGISTER DEFINES
#define EAPE_REG_IRQ_EN                         0x00
#define EAPE_REG_IRQ_STAT                       0x04
#define EAPE_REG_IRQ_CTRL                       0x08
#define EAPE_REG_FIFO_STAT                      0x0C
#define EAPE_REG_STAT_WD_CTL                    0x14
#define EAPE_REG_DMA_BURST                      0x18
#define EAPE_REG_OUT_SRC_PTR                    0x20
#define EAPE_REG_OUT_DST_PTR                    0x24
#define EAPE_REG_OUT_OFFSET                     0x28
#define EAPE_REG_OUT_ID                         0x2C
#define EAPE_REG_OUT_SAI                        0x30
#define EAPE_REG_OUT_POP                        0x38
#define EAPE_REG_OUT_STAT                       0x3C
#define EAPE_REG_IN_SRC_PTR                     0x40
#define EAPE_REG_IN_DST_PTR                     0x44
#define EAPE_REG_IN_OFFSET                      0x48
#define EAPE_REG_IN_ID                          0x4C
#define EAPE_REG_IN_SAI                         0x50
#define EAPE_REG_IN_POP                         0x58
#define EAPE_REG_IN_STAT                        0x5C
#define EAPE_REG_CACHE_FLUSH                    0x60
#define EAPE_REG_CACHE_RDY                      0x64
#define EAPE_REG_VERSION0                       0x70
#define EAPE_REG_VERSION1                       0x74
#define EAPE_REG_RNG                            0x100

// IRQ ENABLE REGISTER (0x00) BIT
#define EAPE_IRQ_EN_OUT_CMD_EN                  (1UL<<EAPE_IRQ_EN_OUT_CMD_EN_OFFSET)
#define EAPE_IRQ_EN_IN_CMD_EN                   (1UL<<EAPE_IRQ_EN_IN_CMD_EN_OFFSET)
#define EAPE_IRQ_EN_OUT_STAT_EN                 (1UL<<EAPE_IRQ_EN_OUT_STAT_EN_OFFSET)
#define EAPE_IRQ_EN_IN_STAT_EN                  (1UL<<EAPE_IRQ_EN_IN_STAT_EN_OFFSET)
#define EAPE_IRQ_EN_OUT_STAT_WD_EN              (1UL<<EAPE_IRQ_EN_OUT_STAT_WD_EN_OFFSET)
#define EAPE_IRQ_EN_IN_STAT_WD_EN               (1UL<<EAPE_IRQ_EN_IN_STAT_WD_EN_OFFSET)
#define EAPE_IRQ_EN_GLBL_EN                     (1UL<<EAPE_IRQ_EN_GLBL_EN_OFFSET)

//IRQ ENABLE REGISTER OFFSETS
#define EAPE_IRQ_EN_OUT_CMD_EN_OFFSET        0
#define EAPE_IRQ_EN_IN_CMD_EN_OFFSET         1
#define EAPE_IRQ_EN_OUT_STAT_EN_OFFSET       4
#define EAPE_IRQ_EN_IN_STAT_EN_OFFSET        5
#define EAPE_IRQ_EN_OUT_STAT_WD_EN_OFFSET   12
#define EAPE_IRQ_EN_IN_STAT_WD_EN_OFFSET    13
#define EAPE_IRQ_EN_GLBL_EN_OFFSET          31

//IRQ STAT REGISTER (0x04) BIT MASKS
#define EAPE_IRQ_STAT_OUT_CMD_MASK              (1UL<<EAPE_IRQ_STAT_OUT_CMD_OFFSET)
#define EAPE_IRQ_STAT_IN_CMD_MASK               (1UL<<EAPE_IRQ_STAT_IN_CMD_OFFSET)
#define EAPE_IRQ_STAT_OUT_STAT_MASK             (1UL<<EAPE_IRQ_STAT_OUT_STAT_OFFSET)
#define EAPE_IRQ_STAT_IN_STAT_MASK              (1UL<<EAPE_IRQ_STAT_IN_STAT_OFFSET)
#define EAPE_IRQ_STAT_OUT_STAT_WD_MASK          (1UL<<EAPE_IRQ_STAT_OUT_STAT_WD_OFFSET)
#define EAPE_IRQ_STAT_IN_STAT_WD_MASK           (1UL<<EAPE_IRQ_STAT_IN_STAT_WD_OFFSET)

//IRQ STAT REGISTER OFFSETS
#define EAPE_IRQ_STAT_OUT_CMD_OFFSET          0
#define EAPE_IRQ_STAT_IN_CMD_OFFSET           1
#define EAPE_IRQ_STAT_OUT_STAT_OFFSET         4
#define EAPE_IRQ_STAT_IN_STAT_OFFSET          5
#define EAPE_IRQ_STAT_OUT_STAT_WD_OFFSET     12
#define EAPE_IRQ_STAT_IN_STAT_WD_OFFSET      13

//IRQ CTRL REGISTER (0x08) BIT MASKS
#define EAPE_IRQ_CTRL_OUT_CMD_CNT_MASK          (0x7FUL<<EAPE_IRQ_CTRL_OUT_CMD_CNT_OFFSET)
#define EAPE_IRQ_CTRL_IN_CMD_CNT_MASK           (0x7FUL<<EAPE_IRQ_CTRL_IN_CMD_CNT_OFFSET)
#define EAPE_IRQ_CTRL_OUT_STAT_CNT_MASK         (0x7FUL<<EAPE_IRQ_CTRL_OUT_STAT_CNT_OFFSET)
#define EAPE_IRQ_CTRL_IN_STAT_CNT_MASK          (0x7FUL<<EAPE_IRQ_CTRL_IN_STAT_CNT_OFFSET)

//IRQ CTRL REGISTER OFFSETS
#define EAPE_IRQ_CTRL_OUT_CMD_CNT_OFFSET   0
#define EAPE_IRQ_CTRL_IN_CMD_CNT_OFFSET    8
#define EAPE_IRQ_CTRL_OUT_STAT_CNT_OFFSET 16
#define EAPE_IRQ_CTRL_IN_STAT_CNT_OFFSET  24

//FIFO STAT REGISTER (0x0C) BIT MASKS
#define EAPE_FIFO_STAT_OUT_CMD_CNT_MASK         (0x7FUL<<EAPE_FIFO_STAT_OUT_CMD_CNT_OFFSET)
#define EAPE_FIFO_STAT_OUT_CMD_FULL_MASK        (1UL<<EAPE_FIFO_STAT_OUT_CMD_FULL_OFFSET)
#define EAPE_FIFO_STAT_IN_CMD_CNT_MASK          (0x7FUL<<EAPE_FIFO_STAT_IN_CMD_CNT_OFFSET)
#define EAPE_FIFO_STAT_IN_CMD_FULL_MASK         (1UL<<EAPE_FIFO_STAT_IN_CMD_FULL_OFFSET)
#define EAPE_FIFO_STAT_OUT_STAT_CNT_MASK        (0x7FUL<<EAPE_FIFO_STAT_OUT_STAT_CNT_OFFSET)
#define EAPE_FIFO_STAT_OUT_STAT_EMPTY_MASK      (1UL<<EAPE_FIFO_STAT_OUT_STAT_EMPTY_OFFSET)
#define EAPE_FIFO_STAT_IN_STAT_CNT_MASK         (0x7FUL<<EAPE_FIFO_STAT_IN_STAT_CNT_OFFSET)
#define EAPE_FIFO_STAT_IN_STAT_EMPTY_MASK       (1UL<<EAPE_FIFO_STAT_IN_STAT_EMPTY_OFFSET)

//FIFO STAT REGISTER OFFSETS
#define EAPE_FIFO_STAT_OUT_CMD_CNT_OFFSET    0
#define EAPE_FIFO_STAT_OUT_CMD_FULL_OFFSET   7
#define EAPE_FIFO_STAT_IN_CMD_CNT_OFFSET     8
#define EAPE_FIFO_STAT_IN_CMD_FULL_OFFSET    15
#define EAPE_FIFO_STAT_OUT_STAT_CNT_OFFSET   16
#define EAPE_FIFO_STAT_OUT_STAT_EMPTY_OFFSET 23
#define EAPE_FIFO_STAT_IN_STAT_CNT_OFFSET    24
#define EAPE_FIFO_STAT_IN_STAT_EMPTY_OFFSET  31

//STAT WD TIMER CONTROL REGISTER (0x14) BIT MASKS
#define EAPE_STAT_WD_CTRL_MASK         0x00FFFFFF

//STAT WD TIMER CONTROL REGISTER OFFSETS
#define EAPE_STAT_WD_CTRL_OFFSET       0

//DMA BURST SIZE REGISTER (0x18) BIT MASKS
#define EAPE_DMA_BURST_SIZE_MASK    0x0000001F

//DMA BURST SIZE REGISTER OFFSET
#define EAPE_DMA_BURST_SIZE_OFFSET     0

//OUT SOURCE POINTER REGISTER (0x20) BIT MASK
#define EAPE_OUT_SRC_PTR_MASK       0xFFFFFFF8

//OUT SOURCE POINTER REGISTER OFFSETS
#define EAPE_OUT_SRC_PTR_OFFSET        3

//OUT DESTINATION POINTER REGISTER (0x24) BIT MASK
#define EAPE_OUT_DST_PTR_MASK       0xFFFFFFF8

//OUT DESTINATION POINTER REGISTER OFFSETS
#define EAPE_OUT_DST_PTR_OFFSET        3

//OUT OFFSET REGISTER (0x28) BIT MASKS
#define EAPE_OUT_OFFSET_SRC_MASK       0x00003FFF
#define EAPE_OUT_OFFSET_DST            0x3FFF0000

//OUT OFFSET REGISTER OFFSETS
#define EAPE_OUT_OFFSET_SRC_OFFSET     0
#define EAPE_OUT_OFFSET_DST_OFFSET     16

//OUT SOFTWARE TAG ID (0x2C) BIT MASK
#define EAPE_OUT_ID_MASK            0x000000FF

//OUT SOFTWARE TAG ID OFFSETS
#define EAPE_OUT_ID_OFFSET          0

//OUT SA STRUCT POINTER (0x30) BIT MASK
#define EAPE_OUT_SAI_MASK           0xFFFFFF00

//OUT SA STRUCT POINTER OFFSET
#define EAPE_OUT_SAI_OFFSET         8

//OUT POP REGISTER (0x38) BIT MASK
#define EAPE_OUT_POP_MASK           0x00000001

//OUT POP REGISTER OFFSET
#define EAPE_OUT_POP_OFFSET         0

//OUT STAT REGISTER (0x3C) BIT MASK
#define EAPE_OUT_STAT_LENGTH_MASK      0x0000FFFF
#define EAPE_OUT_STAT_ID_MASK          0x00FF0000
#define EAPE_OUT_STAT_RET_CODE_MASK    0x0F000000
#define EAPE_OUT_STAT_STTL_MASK        0x80000000

//OUT STAT REGISTER OFFSET
#define EAPE_OUT_STAT_LENGTH_OFFSET    0
#define EAPE_OUT_STAT_ID_OFFSET        16
#define EAPE_OUT_STAT_RET_CODE_OFFSET  24
#define EAPE_OUT_STAT_STTL_OFFSET      31

//VERSION 0 REGISTER (0x70) BIT MASK
#define EAPE_VERSION0_VER_MINOR_MASK         (0xFUL<<EAPE_VERSION0_VER_MINOR_OFFSET)
#define EAPE_VERSION0_VER_MAJOR_MASK         (0xFUL<<EAPE_VERSION0_VER_MAJOR_OFFSET)
#define EAPE_VERSION0_DMA_TYPE_MASK          (3UL<<EAPE_VERSION0_DMA_TYPE_OFFSET)
#define EAPE_VERSION0_IPV6_MASK              (1UL<<EAPE_VERSION0_IPV6_OFFSET)
#define EAPE_VERSION0_TX_CACHE_MASK          (1UL<<EAPE_VERSION0_TX_CACHE_OFFSET)
#define EAPE_VERSION0_RX_CACHE_MASK          (1UL<<EAPE_VERSION0_RX_CACHE_OFFSET)
#define EAPE_VERSION0_SAD_64_MASK            (1UL<<EAPE_VERSION0_SAD_64_OFFSET)
#define EAPE_VERSION0_PBM_64_MASK            (1UL<<EAPE_VERSION0_PBM_64_OFFSET)
#define EAPE_VERSION0_PROJECT_MASK           (0xFFFF<<EAPE_VERSION0_PROJECT_OFFSET)

//VERSION 0 REGISTER OFFSET
#define EAPE_VERSION0_VER_MINOR_OFFSET       0
#define EAPE_VERSION0_VER_MAJOR_OFFSET       4
#define EAPE_VERSION0_DMA_TYPE_OFFSET        8
#define EAPE_VERSION0_IPV6_OFFSET            10
#define EAPE_VERSION0_TX_CACHE_OFFSET        11
#define EAPE_VERSION0_RX_CACHE_OFFSET        12
#define EAPE_VERSION0_SAD_64_OFFSET          13
#define EAPE_VERSION0_PBM_64_OFFSET          14
#define EAPE_VERSION0_PROJECT_OFFSET         16

//VERSION 1 REGISTER (0x74) BIT MASK
#define EAPE_VERSION1_FIFO_DEPTH_MASK        (0x7FUL<<EAPE_VERSION1_FIFO_DEPTH_OFFSET)
#define EAPE_VERSION1_RNG_PRESENT_MASK       (1UL<<EAPE_VERSION1_RNG_PRESENT_OFFSET)
#define EAPE_VERSION1_MD5_ENABLED_MASK       (1UL<<EAPE_VERSION1_MD5_ENABLED_OFFSET)
#define EAPE_VERSION1_SHA1_ENABLED_MASK      (1UL<<EAPE_VERSION1_SHA1_ENABLED_OFFSET)
#define EAPE_VERSION1_SHA256_ENABLED_MASK    (1UL<<EAPE_VERSION1_SHA256_ENABLED_OFFSET)
#define EAPE_VERSION1_SHA384_ENABLED_MASK    (1UL<<EAPE_VERSION1_SHA384_ENABLED_OFFSET)
#define EAPE_VERSION1_SHA512_ENABLED_MASK    (1UL<<EAPE_VERSION1_SHA512_ENABLED_OFFSET)
#define EAPE_VERSION1_TX_PIPES_MASK          (7UL<<EAPE_VERSION1_TX_PIPES_OFFSET)
#define EAPE_VERSION1_RX_PIPES_MASK          (7UL<<EAPE_VERSION1_RX_PIPES_OFFSET)
#define EAPE_VERSION1_DESCBC_ENABLED_MASK    (1UL<<EAPE_VERSION1_DESCBC_ENABLED_OFFSET)
#define EAPE_VERSION1_AESCBC_ENABLED_MASK    (1UL<<EAPE_VERSION1_AESCTR_ENABLED_OFFSET)
#define EAPE_VERSION1_AESCTR_ENABLED_MASK    (1UL<<EAPE_VERSION1_AESCTR_ENABLED_OFFSET)
#define EAPE_VERSION1_AESGCM_ENABLED_MASK    (1UL<<EAPE_VERSION1_AESGCM_ENABLED_OFFSET)
#define EAPE_VERSION1_AES128_ENABLED_MASK    (1UL<<EAPE_VERSION1_AES128_ENABLED_OFFSET)
#define EAPE_VERSION1_AES192_ENABLED_MASK    (1UL<<EAPE_VERSION1_AES192_ENABLED_OFFSET)
#define EAPE_VERSION1_AES256_ENABLED_MASK    (1UL<<EAPE_VERSION1_AES256_ENABLED_OFFSET)

//VERSION 1 REGISTER (0x74) OFFSETS
#define EAPE_VERSION1_FIFO_DEPTH_OFFSET         0
#define EAPE_VERSION1_RNG_PRESENT_OFFSET        8
#define EAPE_VERSION1_MD5_ENABLED_OFFSET        10
#define EAPE_VERSION1_SHA1_ENABLED_OFFSET       11
#define EAPE_VERSION1_SHA256_ENABLED_OFFSET     12
#define EAPE_VERSION1_SHA384_ENABLED_OFFSET     13
#define EAPE_VERSION1_SHA512_ENABLED_OFFSET     14
#define EAPE_VERSION1_TX_PIPES_OFFSET           16
#define EAPE_VERSION1_RX_PIPES_OFFSET           20
#define EAPE_VERSION1_DESCBC_ENABLED_OFFSET     24
#define EAPE_VERSION1_AESCBC_ENABLED_OFFSET     25
#define EAPE_VERSION1_AESCTR_ENABLED_OFFSET     26
#define EAPE_VERSION1_AESGCM_ENABLED_OFFSET     27
#define EAPE_VERSION1_AES128_ENABLED_OFFSET     29
#define EAPE_VERSION1_AES192_ENABLED_OFFSET     30
#define EAPE_VERSION1_AES256_ENABLED_OFFSET     31

#define EAPE_CHECK_REG_BIT(x,y)                 (((x)>>y)&1)
#define EAPE_SET_REG_BIT(x,y)                   ((x) | (1UL<<y))
#define EAPE_GET_REG_BITS(x,y,z)                (((x)&y)>>z)
#define SET_BIT_FIELD(x,y,z,w) ((x&~z) | (w<<y))

#define SET_BIT(x)                              (1UL<<(x))
#define CLEAR_BIT(x)                            (0UL<<(x))

//SA DATABASE ENTRIES
#define EAPE_SA_CTRL             0x00
#define EAPE_SA_SPI              0x04
#define EAPE_SA_SEQ_NUM          0x08
#define EAPE_SA_HARD_TTL         0x10
#define EAPE_SA_SOFT_TTL         0x18
#define EAPE_SA_AR_MASK          0x20
#define EAPE_SA_CIPHER_KEY       0x80
#define EAPE_SA_CIPHER_SALT      0xA0
#define EAPE_SA_MAC_KEY          0xC0

//SA CONTROL FIELD FORMAT
#define _SA_CTRL_ACTIVE             0
#define _SA_CTRL_SEQ_ROLL           1
#define _SA_CTRL_TTL_EN             2
#define _SA_CTRL_HDR_TYPE           4
#define _SA_CTRL_AR_EN              7
#define _SA_CTRL_IPV6              12
#define _SA_CTRL_DST_OP_MODE       13
#define _SA_CTRL_ESN_EN            14
#define _SA_CTRL_CKEY_LEN          16
#define _SA_CTRL_MAC_LEN           18
#define _SA_CTRL_CIPH_ALG          24
#define _SA_CTRL_MAC_ALG           28

enum
{
   CKEY_LEN_128 = 0,
   CKEY_LEN_192,
   CKEY_LEN_256
};

//MAC LENGTH
enum
{
   MAC_LEN_64 = 0,
   MAC_LEN_96,
   MAC_LEN_128
};

//ENCRYPTION ALGORITHM
enum
{
   CIPH_ALG_NULL = 0,
   CIPH_ALG_DES_CBC,
   CIPH_ALG_AES_CBC,
   CIPH_ALG_AES_CTR,
   CIPH_ALG_RESERVED_1,         // #4 is reserved...
   CIPH_ALG_AES_GCM,
   CIPH_ALG_AES_GMAC,
};

//MAC ALGORITHM
enum
{
   MAC_ALG_HMAC_NULL = 0,
   MAC_ALG_HMAC_MD5_96,
   MAC_ALG_HMAC_SHA1_96,
   MAC_ALG_HMAC_SHA256_128,
   MAC_ALG_HMAC_SHA384_192,
   MAC_ALG_HMAC_SHA512_256,
};

#define SA_CTRL_ACTIVE        (1UL<<_SA_CTRL_ACTIVE)
#define SA_CTRL_SEQ_ROLL      (1UL<<_SA_CTRL_SEQ_ROLL)
#define SA_CTRL_TTL_EN        (1UL<<_SA_CTRL_TTL_EN)
#define SA_CTRL_HDR_TYPE      (1UL<<_SA_CTRL_HDR_TYPE)
#define SA_CTRL_AR_EN         (1UL<<_SA_CTRL_AR_EN)
#define SA_CTRL_IPV6          (1UL<<_SA_CTRL_IPV6)
#define SA_CTRL_DST_OP_MODE   (1UL<<_SA_CTRL_DST_OP_MODE)
#define SA_CTRL_ESN           (1UL<<_SA_CTRL_ESN_EN)
#define SA_CTRL_CKEY_LEN(x)   ((x)<<_SA_CTRL_CKEY_LEN)
#define SA_CTRL_MAC_LEN(x)    ((x)<<_SA_CTRL_MAC_LEN)
#define SA_CTRL_CIPH_ALG(x)   ((x)<<_SA_CTRL_CIPH_ALG)
#define SA_CTRL_MAC_ALG(x)    ((x)<<_SA_CTRL_MAC_ALG)

//Macros Definitions
#define EAPE_SET_BIT_FIELD(eape, reg, mask, offset, value) \
      pdu_io_write32((eape)->regmap + reg, ((pdu_io_read32((eape)->regmap + reg) & ~mask) | (value << offset))

#define EAPE_GET_BIT_FIELD(eape, reg, mask, offset, value) \
      value = ((pdu_io_read32((eape)->regmap + reg) & mask) >> offset)

#endif
