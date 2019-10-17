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


#ifndef ELP_KEPHW_H
#define ELP_KEPHW_H

#define KEP_REG_IRQ_EN             0x0000
#define KEP_REG_IRQ_STAT           0x0004
#define KEP_REG_IRQ_CTRL           0x0008
#define KEP_REG_FIFO_STAT          0x000C
#define KEP_REG_SRC_PTR            0x0020
#define KEP_REG_DST_PTR            0x0024
#define KEP_REG_OFFSET             0x0028
#define KEP_REG_LEN                0x002C
#define KEP_REG_SW_CTRL            0x003C
#define KEP_REG_CTRL               0x0040
#define KEP_REG_POP                0x0050
#define KEP_REG_STATUS             0x0054

// IRQ EN 
#define _KEP_IRQ_EN_CMD              0
#define _KEP_IRQ_EN_STAT             1
#define _KEP_IRQ_EN_GLBL            31
#define KEP_IRQ_EN_CMD              (1UL<<_KEP_IRQ_EN_CMD)
#define KEP_IRQ_EN_STAT             (1UL<<_KEP_IRQ_EN_STAT)
#define KEP_IRQ_EN_GLBL             (1UL<<_KEP_IRQ_EN_GLBL)

// IRQ STAT
#define _KEP_IRQ_STAT_CMD            0
#define _KEP_IRQ_STAT_STAT           1
#define KEP_IRQ_STAT_CMD            (1UL<<_KEP_IRQ_STAT_CMD)
#define KEP_IRQ_STAT_STAT           (1UL<<_KEP_IRQ_STAT_STAT)

// IRQ CTRL
#define _KEP_IRQ_CTRL_CMD_CNT        0
#define _KEP_IRQ_CTRL_STAT_CNT      24
#define KEP_REG_IRQ_CTRL_CMD(x)     ((x)<<_KEP_IRQ_CTRL_CMD_CNT)
#define KEP_REG_IRQ_CTRL_STAT(x)    ((x)<<_KEP_IRQ_CTRL_STAT_CNT)


// FIFO STAT
#define _KEP_FIFO_CMD_CNT            0
#define _KEP_FIFO_CMD_FULL           7
#define _KEP_FIFO_STAT_CNT          24
#define _KEP_FIFO_STAT_EMPTY        31

#define KEP_FIFO_CMD_CNT(x)         ((x >> _KEP_FIFO_CMD_CNT)&0x7F)
#define KEP_FIFO_CMD_FULL(x)        ((x >> _KEP_FIFO_CMD_FULL)&0x1)
#define KEP_FIFO_STAT_CNT(x)        ((x >> _KEP_FIFO_STAT_CNT)&0x7F)
#define KEP_FIFO_STAT_EMPTY(x)      ((x >> _KEP_FIFO_STAT_EMPTY)&0x1)

// OFFSET
#define _KEP_SRC_OFFSET              0
#define _KEP_DST_OFFSET             16
#define KEP_OFFSET(src, dst)         ( ((src & 0xFFFF) << _KEP_SRC_OFFSET) | ((dst & 0xFFFF) << _KEP_DST_OFFSET) )

// CTRL
#define _KEP_CTRL_CMD                0
#define _KEP_CTRL_OPT                4
#define _KEP_CTRL_CTX_IDX           16
#define KEP_CTRL(cmd, opt, idx)     (  ((cmd & 7) << _KEP_CTRL_CMD) | ((opt & 0xF) << _KEP_CTRL_OPT) | ((idx & 0xFF) << _KEP_CTRL_CTX_IDX) )

// STATUS
#define _KEP_STATUS_SWID            16
#define _KEP_STATUS_RETCODE         24
#define KEP_STATUS_SWID(x)          ((x >> _KEP_STATUS_SWID) & 0xFF)
#define KEP_STATUS_RETCODE(x)       ((x >> _KEP_STATUS_RETCODE) & 0x03)

#endif
