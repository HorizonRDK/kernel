/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __SND_SOC_X2_I2S_REGS_H
#define __SND_SOC_X2_I2S_REGS_H

#define I2SCTL          0x0
#define I2SMOD          0x4
#define I2SWS           0x8
#define I2SCHEN         0xc
#define I2SBSIZE        0x20    //for each channel, not the whole buf size
#define I2SB0ADDR       0x24
#define I2SB0RDY        0x28
#define I2SB1ADDR       0x2c
#define I2SB1RDY        0x30
#define I2SCURADDR      0x34
#define I2SERR          0x38
#define I2SSRCPND       0x80
#define I2SINTMASK      0x84
#define I2SINTSETMASK   0x88
#define I2SINTUNMASK    0x8C

#define I2S_SYSCLK_SET  0X154
#define I2S_SYSCLK_CLR  0X158

#define I2S_CLKCTL_BASE 0X350
#define CLK_MODE        20
#define BCLK_SHIFT      16
#define MCLK_SHIFT      8
#define PRE_MCLK_SHIFT  4


#define I2S_SYS_RST     0X450

#define SYSCTL_SHIFT    18


#define CTL_IMEM_CLR    (1 << 2)
#define CTL_IMEM_EN     (1 << 1)
#define CTL_ENABLE      (1 << 0)


#define MOD_CLK_EDG     (1 << 9)
#define MOD_CP_ZERO     (1 << 8)
#define MOD_LRWS        (1 << 7)
#define MOD_FST_EDG     (1 << 6)
#define MOD_WORD_LEN    (1 << 5)
#define MOD_CH_NUM      (7 << 2)
#define MOD_I2S_MODE    (1 << 1)
#define MOD_MS_MODE     (1 << 0)

#define MOD_1_CH_C      (4 << 7)
#define MOD_1_CH_P      (1 << 7)
#define MOD_2_CH        (0 << 7)
#define MOD_4_CH        (1 << 7)
#define MOD_8_CH        (2 << 7)
#define MOD_16_CH       (3 << 7)


#define WS_H    (0XFF << 8)
#define WS_H    (0XFF) << 0)

#define CHEN_CH15   (1 << 15)
#define CHEN_CH14   (1 << 14)
#define CHEN_CH13   (1 << 13)
#define CHEN_CH12   (1 << 12)
#define CHEN_CH11   (1 << 11)
#define CHEN_CH10   (1 << 10)
#define CHEN_CH9    (1 << 9)
#define CHEN_CH8    (1 << 8)
#define CHEN_CH7    (1 << 7)
#define CHEN_CH6    (1 << 6)
#define CHEN_CH5    (1 << 5)
#define CHEN_CH4    (1 << 4)
#define CHEN_CH3    (1 << 3)
#define CHEN_CH2    (1 << 2)
#define CHEN_CH1    (1 << 1)
#define CHEN_CH0    (1 << 0)

#define ERR_CH15    (1 << 15)
#define ERR_CH14    (1 << 14)
#define ERR_CH13    (1 << 13)
#define ERR_CH12    (1 << 12)
#define ERR_CH11    (1 << 11)
#define ERR_CH10    (1 << 10)
#define ERR_CH9     (1 << 9)
#define ERR_CH8     (1 << 8)
#define ERR_CH7     (1 << 7)
#define ERR_CH6     (1 << 6)
#define ERR_CH5     (1 << 5)
#define ERR_CH4     (1 << 4)
#define ERR_CH3     (1 << 3)
#define ERR_CH2     (1 << 2)
#define ERR_CH1     (1 << 1)
#define ERR_CH0     (1 << 0)

#define SRCPND_BUF1_DONE    (1 << 3)
#define SRCPND_BUF0_DONE    (1 << 2)
#define SRCPND_OVERFLOW     (1 << 1)
#define SRCPND_NOTREADY     (1 << 0)

#define INT_BUF1_DONE   (1 << 3)
#define INT_BUF0_DONE   (1 << 2)
#define INT_OVERFLOW        (1 << 1)
#define INT_NOTREADY        (1 << 0)

#endif /* __SND_SOC_X2_I2S_REGS_H */
