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

#ifndef EAHW_H_
#define EAHW_H_

// ERRORS
#define EA_OK            0
#define EA_ERROR         -1
#define EA_FIFO_FULL     -2
#define EA_NO_CONTEXT    -3
#define EA_JOBS_FULL     -4

// registers
#define EA_IRQ_EN         0x00
#define EA_IRQ_STAT       0x04
#define EA_IRQ_CTRL       0x08
#define EA_FIFO_STAT      0x0C
#define EA_FIFO_STAT_PRIO 0x10
#define EA_SRC_PTR        0x20
#define EA_DST_PTR        0x24
#define EA_OFFSET         0x28
#define EA_SA_PTR         0x30
#define EA_SW_CTRL        0x3C
#define EA_CTRL           0x40
#define EA_POP            0x50
#define EA_STATUS         0x54
#define EA_STAT_WD_CTRL   0x60
#define EA_VERSION        0x70

#define _EA_IRQ_EN_CMD         0
#define _EA_IRQ_EN_CMD1        1
#define _EA_IRQ_EN_STAT        4
#define _EA_IRQ_EN_STAT_V24    1
#define _EA_IRQ_EN_STAT_WD    12
#define _EA_IRQ_EN_GLBL       31
#define EA_IRQ_EN_CMD         (1UL<<_EA_IRQ_EN_CMD)
#define EA_IRQ_EN_CMD1        (1UL<<_EA_IRQ_EN_CMD1)
#define EA_IRQ_EN_STAT        (1UL<<_EA_IRQ_EN_STAT)
#define EA_IRQ_EN_STAT_V24    (1UL<<_EA_IRQ_EN_STAT_V24)
#define EA_IRQ_EN_STAT_WD     (1UL<<_EA_IRQ_EN_STAT_WD)
#define EA_IRQ_EN_GLBL        (1UL<<_EA_IRQ_EN_GLBL)

#define _EA_IRQ_STAT_CMD       0
#define _EA_IRQ_STAT_CMD1      1
#define _EA_IRQ_STAT_STAT_V24  1
#define _EA_IRQ_STAT_STAT      4
#define _EA_IRQ_STAT_STAT_WD  12
#define EA_IRQ_STAT_CMD       (1UL<<_EA_IRQ_STAT_CMD)
#define EA_IRQ_STAT_CMD1      (1UL<<_EA_IRQ_STAT_CMD1)
#define EA_IRQ_STAT_STAT      (1UL<<_EA_IRQ_STAT_STAT)
#define EA_IRQ_STAT_STAT_V24  (1UL<<_EA_IRQ_STAT_STAT_V24)
#define EA_IRQ_STAT_STAT_WD   (1UL<<_EA_IRQ_STAT_STAT_WD)

#define _EA_IRQ_CTRL_CMD_CNT       0
#define _EA_IRQ_CTRL_CMD1_CNT      8
#define _EA_IRQ_CTRL_STAT_CNT     16
#define _EA_IRQ_CTRL_STAT_CNT_V24 24
#define EA_IRQ_CTRL_CMD_CNT(x)   ((x)<<_EA_IRQ_CTRL_CMD_CNT)
#define EA_IRQ_CTRL_CMD_CNT1(x)  ((x)<<_EA_IRQ_CTRL_CMD1_CNT)
#define EA_IRQ_CTRL_STAT_CNT(x)  ((x)<<_EA_IRQ_CTRL_STAT_CNT)
#define EA_IRQ_CTRL_STAT_CNT_V24(x)  ((x)<<_EA_IRQ_CTRL_STAT_CNT_V24)

#define EA_IRQ_ENABLE_GLBL(ea)      pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) |  EA_IRQ_EN_GLBL)
#define EA_IRQ_ENABLE_CMD(ea, cnt)  pdu_io_write32((ea)->regmap + EA_IRQ_CTRL, (pdu_io_read32((ea)->regmap + EA_IRQ_CTRL)&~EA_IRQ_CTRL_CMD_CNT(63))  | EA_IRQ_CTRL_CMD_CNT(cnt)); \
                                    pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) | EA_IRQ_EN_CMD)
#define EA_IRQ_ENABLE_CMD1(ea, cnt) pdu_io_write32((ea)->regmap + EA_IRQ_CTRL, (pdu_io_read32((ea)->regmap + EA_IRQ_CTRL)&~EA_IRQ_CTRL_CMD1_CNT(63)) | EA_IRQ_CTRL_CMD1_CNT(cnt)); \
                                    pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) | EA_IRQ_EN_CMD1)

#define EA_IRQ_SET_STAT(ea, cnt) \
if ((ea)->spacc->config.pdu_version >= 0x25) { \
   pdu_io_write32((ea)->regmap + EA_IRQ_CTRL, (pdu_io_read32((ea)->regmap + EA_IRQ_CTRL)&~EA_IRQ_CTRL_STAT_CNT(255)) | EA_IRQ_CTRL_STAT_CNT(cnt)); \
} else { \
   pdu_io_write32((ea)->regmap + EA_IRQ_CTRL, (pdu_io_read32((ea)->regmap + EA_IRQ_CTRL)&~EA_IRQ_CTRL_STAT_CNT_V24(127)) | EA_IRQ_CTRL_STAT_CNT_V24(cnt)); \
}


#define EA_IRQ_ENABLE_STAT(ea, cnt) \
if ((ea)->spacc->config.pdu_version >= 0x25) { \
   pdu_io_write32((ea)->regmap + EA_IRQ_CTRL, (pdu_io_read32((ea)->regmap + EA_IRQ_CTRL)&~EA_IRQ_CTRL_STAT_CNT(255)) | EA_IRQ_CTRL_STAT_CNT(cnt)); \
   pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) | EA_IRQ_EN_STAT);                                             \
} else { \
   pdu_io_write32((ea)->regmap + EA_IRQ_CTRL, (pdu_io_read32((ea)->regmap + EA_IRQ_CTRL)&~EA_IRQ_CTRL_STAT_CNT_V24(127)) | EA_IRQ_CTRL_STAT_CNT_V24(cnt)); \
   pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) | EA_IRQ_EN_STAT_V24);                                             \
}

#define EA_IRQ_ENABLE_STAT_WD(ea, cnt) pdu_io_write32((ea)->regmap + EA_STAT_WD_CTRL, cnt); \
                                       pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) |  EA_IRQ_EN_STAT_WD)

#define EA_IRQ_DISABLE_CMD(ea)      pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) & ~EA_IRQ_EN_CMD)
#define EA_IRQ_DISABLE_CMD1(ea)     pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) & ~EA_IRQ_EN_CMD1)
#define EA_IRQ_DISABLE_STAT(ea)  \
if ((ea)->spacc->config.pdu_version >= 0x25) { \
   pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) & ~EA_IRQ_EN_STAT); \
} else { \
   pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) & ~EA_IRQ_EN_STAT_V24); \
}
#define EA_IRQ_DISABLE_STAT_WD(ea)   pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) & ~EA_IRQ_EN_STAT_WD);

#define EA_IRQ_DISABLE_GLBL(ea)     pdu_io_write32((ea)->regmap + EA_IRQ_EN, pdu_io_read32((ea)->regmap + EA_IRQ_EN) & ~EA_IRQ_EN_GLBL)

#define _EA_FIFO_STAT_CMD_CNT       0
#define _EA_FIFO_STAT_CMD_FULL      7
#define _EA_FIFO_STAT_CMD1_CNT      8
#define _EA_FIFO_STAT_CMD1_FULL    15
#define _EA_FIFO_STAT_STAT_CNT     16
#define _EA_FIFO_STAT_STAT_CNT_V24 24
#define _EA_FIFO_STAT_STAT_EMPTY   31

#define EA_FIFO_STAT_CMD_CNT(x)    (((x)>>_EA_FIFO_STAT_CMD_CNT)&0x7F)
#define EA_FIFO_STAT_CMD1_CNT(x)   (((x)>>_EA_FIFO_STAT_CMD1_CNT)&0x7F)
#define EA_FIFO_STAT_CMD_FULL(x)   (((x)>>_EA_FIFO_STAT_CMD_FULL)&0x1)
#define EA_FIFO_STAT_CMD1_FULL(x)   (((x)>>_EA_FIFO_STAT_CMD1_FULL)&0x1)
#define EA_FIFO_STAT_STAT_CNT(x)   (((x)>>_EA_FIFO_STAT_STAT_CNT)&0x7F)
#define EA_FIFO_STAT_STAT_CNT_V24(x)   (((x)>>_EA_FIFO_STAT_STAT_CNT_V24)&0x7F)
#define EA_FIFO_STAT_STAT_EMPTY(x)  (((x)>>_EA_FIFO_STAT_STAT_EMPTY)&0x1)

#define _EA_FIFO_PRIO_MODE          0
#define _EA_FIFO_PRIO_WT0           8
#define _EA_FIFO_PRIO_WT1          12
#define EA_FIFO_PRIO(ea, mode, wt0, wt1) \
   pdu_io_write32((ea)->regmap + EA_FIFO_STAT_PRIO, ((mode << _EA_FIFO_PRIO_MODE) | (wt0 << _EA_FIFO_PRIO_WT0) | (wt1 << _EA_FIFO_PRIO_WT1)))

#define _EA_OFFSET_SRC  0
#define _EA_OFFSET_DST 16
#define EA_OFFSET_SET(src, dst) (((src)<<_EA_OFFSET_SRC)|((dst)<<_EA_OFFSET_DST))

#define _EA_VERSION_CMD0_DEPTH    0
#define _EA_VERSION_CMD1_DEPTH    8
#define _EA_VERSION_SP_DDT_CNT   16
#define _EA_VERSION_IPV6         24
#define _EA_VERSION_AR_WIN_SIZE  25
#define _EA_VERSION_AH_ENABLE    28
#define EA_VERSION_CMD0_DEPTH(x)  ((x >> _EA_VERSION_CMD0_DEPTH) & 0x7F)
#define EA_VERSION_CMD1_DEPTH(x)  ((x >> _EA_VERSION_CMD1_DEPTH) & 0x7F)
#define EA_VERSION_SP_DDT_CNT(x)  ((x >> _EA_VERSION_SP_DDT_CNT) & 0xFF)
#define EA_VERSION_IPV6(x)        ((x >> _EA_VERSION_IPV6) & 1)
#define EA_VERSION_AR_WIN_SIZE(x) ((x >> _EA_VERSION_AR_WIN_SIZE) & 0x03)
#define EA_VERSION_AH_ENABLE(x)   ((x >> _EA_VERSION_AH_ENABLE) & 1)

#define _EA_CTRL_CMD       0
#define _EA_CTRL_FIFO_SEL  1
#define _EA_CTRL_CTX_ID   16
#define _EA_CTRL_LD_CTX   30
#define EA_CTRL_CMD(x)      ((x)<<_EA_CTRL_CMD)
#define EA_CTRL_FIFO_SEL(x) ((x)<<_EA_CTRL_FIFO_SEL)
#define EA_CTRL_CTX_ID(x)   ((x)<<_EA_CTRL_CTX_ID)
#define EA_CTRL_LD_CTX(x)   ((x)<<_EA_CTRL_LD_CTX)
enum {
   EA_CTRL_INBOUND=0,
   EA_CTRL_OUTBOUND
};

#define _EA_STATUS_LEN       0
#define _EA_STATUS_SW_ID    16
#define _EA_STATUS_RET_CODE 24
#define _EA_STATUS_STTL     31
#define EA_STATUS_LEN(x)      (((x)>>_EA_STATUS_LEN)&0xFFFF)
#define EA_STATUS_SW_ID(x)    (((x)>>_EA_STATUS_SW_ID)&0xFF)
#define EA_STATUS_RET_CODE(x) (((x)>>_EA_STATUS_RET_CODE)&0xF)
#define EA_STATUS_STTL(x)     (((x)>>_EA_STATUS_STTL)&0x1)

#define _EA_ACTIVE_CNT_IDLE 0
#define _EA_ACTIVE_CNT_ACT  16
#define EA_ACTIVE_IDLE(x)  (((x)>>_EA_ACTIVE_CNT_IDLE)&0xFFFF)
#define EA_ACTIVE_ACTIVE(x) (((x)>>_EA_ACTIVE_CNT_ACT)&0xFFFF)

/* Error codes */
enum {
   EA_RET_OK,
   EA_RET_HARD_TTL = 3,
   EA_RET_SA_INACTIVE,
   EA_RET_REPLAY,
   EA_RET_ICV_FAIL,
   EA_RET_SEQ_ROLL,
   EA_RET_MEM_ERROR,
   EA_RET_VERS_ERROR,
   EA_RET_PROT_ERROR,
   EA_RET_PYLD_ERROR,
   EA_RET_PAD_ERROR,
   EA_RET_DUMMY_PKT,
};

// SA structure
#define SA_SIZE          0x100
#define SA_CTRL          0x00
#define SA_SPI           0x04
#define SA_SEQ_NUM       0x08
#define SA_HARD_TTL      0x10
#define SA_SOFT_TTL      0x18
#define SA_AR_MASK       0x20
#define SA_CIPHER_KEY    0x80
#define SA_CIPHER_SALT   0xA0
#define SA_MAC_KEY       0xC0

#define _SA_CTRL_ACTIVE        0
#define _SA_CTRL_SEQ_ROLL      1
#define _SA_CTRL_TTL_EN        2
#define _SA_CTRL_HDR_TYPE      4
#define _SA_CTRL_AR_EN         7
#define _SA_CTRL_AR_WIN_SIZE   8
#define _SA_CTRL_IPV6         12
#define _SA_CTRL_DST_OP_MODE  13
#define _SA_CTRL_ESN          14
#define _SA_CTRL_CKEY_LEN     16
#define _SA_CTRL_MAC_LEN      18
#define _SA_CTRL_CIPH_ALG     24
#define _SA_CTRL_MAC_ALG      27

#define SA_CTRL_ACTIVE        (1UL<<_SA_CTRL_ACTIVE)
#define SA_CTRL_SEQ_ROLL      (1UL<<_SA_CTRL_SEQ_ROLL)
#define SA_CTRL_TTL_EN        (1UL<<_SA_CTRL_TTL_EN)
#define SA_CTRL_HDR_TYPE      (1UL<<_SA_CTRL_HDR_TYPE)
#define SA_CTRL_AR_EN         (1UL<<_SA_CTRL_AR_EN)
#define SA_CTRL_AR_WINSIZE(x) ((x)<<_SA_CTRL_AR_WINSIZE)
#define SA_CTRL_IPV6          (1UL<<_SA_CTRL_IPV6)
#define SA_CTRL_DST_OP_MODE   (1UL<<_SA_CTRL_DST_OP_MODE)
#define SA_CTRL_ESN           (1UL<<_SA_CTRL_ESN)
#define SA_CTRL_CKEY_LEN(x)   ((x)<<_SA_CTRL_CKEY_LEN)
#define SA_CTRL_MAC_LEN(x)    ((x)<<_SA_CTRL_MAC_LEN)
#define SA_CTRL_CIPH_ALG(x)   ((x)<<_SA_CTRL_CIPH_ALG)
#define SA_CTRL_MAC_ALG(x)    ((x)<<_SA_CTRL_MAC_ALG)

enum {
   CKEY_LEN_128=0,
   CKEY_LEN_192,
   CKEY_LEN_256
};

enum {
   MAC_LEN_64=0,
   MAC_LEN_96,
   MAC_LEN_128
};

enum {
   CIPH_ALG_NULL=0,
   CIPH_ALG_DES_CBC,
   CIPH_ALG_AES_CBC,
   CIPH_ALG_AES_CTR,
   CIPH_ALG_AES_CCM,
   CIPH_ALG_AES_GCM,
   CIPH_ALG_AES_GMAC,
};

enum {
   MAC_ALG_NULL=0,
   MAC_ALG_HMAC_MD5_96,
   MAC_ALG_HMAC_SHA1_96,
   MAC_ALG_HMAC_SHA256_128,
   MAC_ALG_HMAC_SHA384_192,
   MAC_ALG_HMAC_SHA512_256,
   MAC_ALG_AES_XCBC_MAC_96,
   MAC_ALG_AES_CMAC_96
};

#endif
