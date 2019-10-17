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



#ifndef ELPREHW_H
#define ELPREHW_H

/**********Operation Types ***************/
#define RE_SND_CCS 0x00
#define RE_SND_ALERT 0x01
#define RE_SND_HANDSHAKE 0x02
#define RE_SND_DATA 0x03
#define RE_RCV_RECORD 0x04
#define RE_LOAD_STORE_SA 0xC0000000

#define RE_SA_MAJOR_VERSION 0x03

/********* Helpers *********************/
#define RE_REGISTER_MASK(n) (0xFFFFFFFF ^ n)

/********* SA offsets ******************/
#define RE_SA_FLAGS 0x000
#define RE_SA_READ_STATE_0 0x020
#define RE_SA_READ_STATE_1 0x0A0
#define RE_SA_WRITE_STATE_0 0x120
#define RE_SA_WRITE_STATE_1 0x1A0
#define RE_SA_RC4_READ_STATE 0x200
#define RE_SA_RC4_WRITE_STATE 0x300


#define RE_SA_STATE_CIPHER_PARAMS 0x00
#define RE_SA_STATE_SEQ_NUM 0x08
#define RE_SA_STATE_IV 0x10
#define RE_SA_STATE_KEY 0x20
#define RE_SA_STATE_MACKEY 0x40

#define RE_SA_PARAMSIZE 0x02
#define RE_SA_SEQ_SIZE 0x08
#define RE_SA_KEYSIZE 0x20
#define RE_SA_MACSIZE 0x20
#define RE_SA_IVSIZE 0x10

#define RE_SA_FLAG_READ_ACTIVE 0x0001
#define RE_SA_FLAG_WRITE_ACTIVE 0x0002
#define RE_SA_FLAG_READ_PAGE   0x0004
#define RE_SA_FLAG_WRITE_PAGE  0x0008
#define RE_SA_FLAG_VERSION     0x00F0

#define RE_SA_FLAG_GET_VERSION(n) (n & RE_SA_FLAG_VERSION >> 4)

/********* Register Offsets ************/
#define RE_IRQ_EN       0x00000
#define RE_IRQ_STAT     0x00004
#define RE_IRQ_CTRL     0x00008
#define RE_FIFO_STAT    0x0000C
#define RE_SRC_PTR      0x00020
#define RE_DST_PTR      0x00024
#define RE_OFFSET       0x00028
#define RE_LEN          0x0002C
#define RE_SA_PTR       0x00030
#define RE_SW_CTRL      0x0003C
#define RE_CTRL         0x00040
#define RE_POP          0x00050
#define RE_STATUS       0x00054
#define RE_VERSION      0x00070

// VERSION
#define _RE_VERSION_FIFO_DEPTH       0
#define _RE_VERSION_SP_DDT_CNT      16
#define _RE_VERSION_SSL3_EN         24
#define _RE_VERSION_TLS10_EN        25
#define _RE_VERSION_TLS11_EN        26
#define _RE_VERSION_TLS12_EN        27
#define _RE_VERSION_DTLS10_EN       29
#define _RE_VERSION_DTLS12_EN       30
#define RE_VERSION_FIFO_DEPTH(x)    (((x)>>_RE_VERSION_FIFO_DEPTH)&0x7F)
#define RE_VERSION_SP_DDT_CNT(x)    (((x)>>_RE_VERSION_SP_DDT_CNT)&0x1F)
#define RE_VERSION_SSL3_EN(x)       (((x)>>_RE_VERSION_SSL3_EN)&0x1)
#define RE_VERSION_TLS10_EN(x)      (((x)>>_RE_VERSION_TLS10_EN)&0x1)
#define RE_VERSION_TLS11_EN(x)      (((x)>>_RE_VERSION_TLS11_EN)&0x1)
#define RE_VERSION_TLS12_EN(x)      (((x)>>_RE_VERSION_TLS12_EN)&0x1)
#define RE_VERSION_DTLS10_EN(x)     (((x)>>_RE_VERSION_DTLS10_EN)&0x1)
#define RE_VERSION_DTLS12_EN(x)     (((x)>>_RE_VERSION_DTLS12_EN)&0x1)

// IRQ EN
#define _RE_IRQ_EN_CMD              0
#define _RE_IRQ_EN_STAT             1
#define _RE_IRQ_EN_GLBL            31
#define RE_IRQ_EN_CMD              (1UL<<_RE_IRQ_EN_CMD)
#define RE_IRQ_EN_STAT             (1UL<<_RE_IRQ_EN_STAT)
#define RE_IRQ_EN_GLBL             (1UL<<_RE_IRQ_EN_GLBL)

// IRQ STAT
#define _RE_IRQ_STAT_CMD            0
#define _RE_IRQ_STAT_STAT           1
#define RE_IRQ_STAT_CMD            (1UL<<_RE_IRQ_STAT_CMD)
#define RE_IRQ_STAT_STAT           (1UL<<_RE_IRQ_STAT_STAT)

// IRQ CTRL
#define _RE_IRQ_CTRL_CMD_CNT        0
#define _RE_IRQ_CTRL_STAT_CNT      24
#define RE_REG_IRQ_CTRL_CMD(x)     ((x)<<_RE_IRQ_CTRL_CMD_CNT)
#define RE_REG_IRQ_CTRL_STAT(x)    ((x)<<_RE_IRQ_CTRL_STAT_CNT)


#define RE_LOOP_WAIT  1000000

/********* RE_IRQ_CTRL Masks ***********/

#define RE_IRQ_CTRL_CMD_CNT       0x3F
#define RE_IRQ_CTRL_STAT_CNT      0x7F000000


/********* RE_FIFO_STAT Masks **********/
#define RE_FIFO_CMD_CNT(n) (n & RE_FIFO_STAT_CMD_CNT)      //Return the number of commands
#define RE_FIFO_CMD_FULL(n) ((n & RE_FIFO_STAT_CMD_FULL) == RE_FIFO_STAT_CMD_FULL) //True if the FIFO is full
#define RE_FIFO_STAT_CNT(n) ((n & RE_FIFO_STAT_STAT_CNT) >> 24)  //Return the number of status-es
#define RE_FIFO_STAT_EMPTY(n) ((n & RE_FIFO_STAT_STAT_EMPTY) == RE_FIFO_STAT_STAT_EMPTY) //True if the FIFO is empty

#define RE_FIFO_STAT_CMD_CNT      0x7F
#define RE_FIFO_STAT_CMD_FULL     0x80
#define RE_FIFO_STAT_STAT_CNT     0x7F000000
#define RE_FIFO_STAT_STAT_EMPTY   0x80000000

/********* RE_OFFSET Masks *************/

#define RE_OFFSET_SRC              0x0000FFFF
#define RE_OFFSET_DST              0xFFFF0000


#define SET_RE_SRC_OFFSET(n, m) (n = (n & RE_RESGISTER_MASK(RE_OFFSET_SRC)) ^ (RE_OFFSET_SRC & m))
#define SET_RE_DST_OFFSET(n, m) (n = (n & RE_RESGISTER_MASK(RE_OFFSET_DST)) ^ (RE_OFFSET_DST & (m << 16)))


/********* RE_CTRL Masks ***************/
#define RE_SET_CMD(n, m)  (n = (n & RE_REGISTER_MASK(RE_CTRL_CMD)) ^ (RE_CTRL_CMD & m))
#define RE_SET_CTX_IDX(n, m) (n = (n & RE_REGISTER_MASK(RE_CTRL_CTX_IDX)) ^ (RE_CTRL_CTX_IDX & (m << 16)))
#define RE_SET_LD_CTX(n, m)  (n = (n & RE_REGISTER_MASK(RE_CTRL_LD_CTX)) ^ (RE_CTRL_LD_CTX & (m << 30)))
#define RE_SET_ST_CTX(n, m)  (n = (n & RE_REGISTER_MASK(RE_CTRL_ST_CTS)) ^ (RE_CTRL_ST_CTX & (m << 31)))

#define RE_CTRL_CMD                0x07
#define RE_CTRL_CTX_IDX            0x00FF0000
#define RE_CTRL_LD_CTX             0x40000000
#define RE_CTRL_ST_CTX             0x80000000


/********* RE_STATUS Masks *************/
#define RE_STAT_LEN(n) (n & RE_STATUS_LEN)
#define RE_STAT_SW_ID(n) ((n & RE_STATUS_SW_ID) >> 16)
#define RE_STAT_RET_CODE(n) ((n & RE_STATUS_RET_CODE) >> 24)
#define RE_STAT_PROT(n)  ((n & RE_STATUS_PROT) >> 30)

#define RE_STATUS_LEN              0x7FFF
#define RE_STATUS_SW_ID            0xFF0000
#define RE_STATUS_RET_CODE         0x0F000000
#define RE_STATUS_PROT             0xC0000000


/********* RE_ERROR_CODES **************/
#define RE_ERR_OK                  0x00
#define RE_ERR_MAC_FAIL            0x01
#define RE_ERR_MEM_ERROR           0x02
#define RE_ERR_BAD_PADDING         0x03
#define RE_ERR_FATAL_ALERT         0x04
#define RE_ERR_UNKNOWN_PROT        0x05
#define RE_ERR_BAD_VERSION         0x06
#define RE_ERR_BAD_LENGTH          0x07
#define RE_ERR_INACTIVE            0x08
#define RE_ERR_SEQ_OVFL            0x09
#define RE_ERR_REPLAY              0x0A

#endif //ELPREHW_H
