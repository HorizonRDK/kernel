/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __SND_SOC_X2_I2S_REGS_H
#define __SND_SOC_X2_I2S_REGS_H

#include <asm-generic/io.h>

#if 0
#define I2S0_RX_BASE 0xA5007000
#define I2S0_TX_BASE 0xA5007500

#define I2S1_RX_BASE 0xA5008000
#define I2S1_TX_BASE 0xA5008500

//need check
#define AUDMA_IRQ_0 65
#define AUDMA_IRQ_1 64

#define AUDMA_I2S0_RXBUFFER 0x30000000
#define AUDMA_I2S0_TXBUFFER 0x30000000

#define AUDMA_I2S1_RXBUFFER 0x30000000
#define AUDMA_I2S1_TXBUFFER 0x30000000

#define SYSCTRL_BASE 0xA1000000
#define DMA_ADDR

#define I2SCTL          0x0
#define I2SMOD          0x4
#define I2SWS           0x8
#define I2SCHEN         0xc
#define I2SBSIZE        0x20	//for each channel, not the whole buf size
#define I2SB0ADDR       0x24
#define I2SB0RDY        0x28
#define I2SB1ADDR       0x2c
#define I2SB1RDY        0x30
#define I2SCURADDR      0x34
#define I2SERR          0x38

#define I2S_CH0_FRAME_LEN 0x3C
#define I2S_CH1_FRAME_LEN 0x40
#define I2S_CH2_FRAME_LEN 0x44
#define I2S_CH3_FRAME_LEN 0x48
#define I2S_CH4_FRAME_LEN 0x4C
#define I2S_CH5_FRAME_LEN 0x50
#define I2S_CH6_FRAME_LEN 0x54
#define I2S_CH7_FRAME_LEN 0x58
#define I2S_CH8_FRAME_LEN 0x5C
#define I2S_CH9_FRAME_LEN 0x60
#define I2S_CH10_FRAME_LEN 0x64
#define I2S_CH11_FRAME_LEN 0x68
#define I2S_CH12_FRAME_LEN 0x6C
#define I2S_CH13_FRAME_LEN 0x70
#define I2S_CH14_FRAME_LEN 0x74
#define I2S_CH15_FRAME_LEN 0x78

#define I2SSRCPND       0x80
#define I2SINTMASK      0x84
#define I2SINTSETMASK   0x88
#define I2SINTUNMASK    0x8C

#define I2S_SYSCLK_SET  0X154
#define I2S_SYSCLK_CLR  0X158

//#define I2S_CLKCTL_BASE 0X350
#define I2S0_CLKCTL_BASE 0X350
#define I2S1_CLKCTL_BASE 0X360

#define CLK_MODE        20
#define BCLK_SHIFT      16
#define MCLK_SHIFT      8
#define PRE_MCLK_SHIFT  0

#define I2S_SYS_RST     0X450
//#define SYSCTL_SHIFT    18
#define SYSCTL_I2S0RST_SHIFT    18
#define SYSCTL_I2S1RST_SHIFT    19

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

#define MOD_1_CH_C      (4 << 2)
#define MOD_1_CH_P      (1 << 2)
#define MOD_2_CH        (0 << 2)
#define MOD_4_CH        (1 << 2)
#define MOD_8_CH        (2 << 2)
#define MOD_16_CH       (3 << 2)

//#define WS_H    (0XFF << 8)
//mark here,need check
#define WS_H (0XFF << 0)

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

enum i2s_clk_type_e {
	I2S_DIV_BCLK_DIV_SEL,
	I2S_MCLK_DIV_SEL,
	I2S_PRE_MCLK_DIV_SEL,
};

#else

enum i2s_clk_type_e {
	I2S_DIV_BCLK_DIV_SEL,
	I2S_MCLK_DIV_SEL,
	I2S_PRE_MCLK_DIV_SEL,
};

struct x2_i2s {

	struct device *dev;
	spinlock_t lock;

	struct snd_dmaengine_dai_dma_data capture_dma_data;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
	unsigned int streamflag;	//0:capture  1:playback
	//unsigned int irq;//dma irq num

	void __iomem *regaddr_tx;	//tx base
	void __iomem *regaddr_rx;	//rx base
	void __iomem *sysctl_addr;	//sysctl base
	void __iomem *apb_regs;	//apb base

	struct reset_control *rst;
	unsigned int i2sdsp;
	int id;
};

#define INT_BUF1_DONE   (1 << 3)
#define INT_BUF0_DONE   (1 << 2)
#define INT_OVERFLOW        (1 << 1)
#define INT_NOTREADY        (1 << 0)

#define I2S_CTL 0x0
#define I2S_MODE 0x04
#define I2S_DIV_WS 0x08
#define I2S_CH_EN 0x0C
#define I2S_BUF_SIZE 0x20
#define I2S_BUF0_ADDR 0x24
#define I2S_BUF0_RDY 0x28
#define I2S_BUF1_ADDR 0x2c
#define I2S_BUF1_RDY 0x30
#define I2S_BUF_CUR_ADDR 0x34
#define I2S_CH_ERROR 0x38
#define I2S_CH0_FRAME_LEN 0x3C
#define I2S_CH1_FRAME_LEN 0x40
#define I2S_CH2_FRAME_LEN 0x44
#define I2S_CH3_FRAME_LEN 0x48
#define I2S_CH4_FRAME_LEN 0x4C
#define I2S_CH5_FRAME_LEN 0x50
#define I2S_CH6_FRAME_LEN 0x54
#define I2S_CH7_FRAME_LEN 0x58
#define I2S_CH8_FRAME_LEN 0x5C
#define I2S_CH9_FRAME_LEN 0x60
#define I2S_CH10_FRAME_LEN 0x64
#define I2S_CH11_FRAME_LEN 0x68
#define I2S_CH12_FRAME_LEN 0x6C
#define I2S_CH13_FRAME_LEN 0x70
#define I2S_CH14_FRAME_LEN 0x74
#define I2S_CH15_FRAME_LEN 0x78
#define I2S_SRCPND 0x80
#define I2S_INTMASK 0x84
#define I2S_SETMASK 0x88
#define I2S_UNMASK 0x8C

#define I2S_CTL_ENABLE_BIT 0
#define I2S_CTL_IMEM_EN_BIT 1
#define I2S_CTL_IMEM_CLEAR_BIT 2

#define I2S_CTL_ENABLE_FIELD 0x1
#define I2S_CTL_IMEM_EN_FIELD 0x1
#define I2S_CTL_IMEM_CLEAR_FIELD 0x1

#define I2S_MODE_MS_MODE_BIT 0
#define I2S_MODE_I2S_DSP_MODE_BIT 1
#define I2S_MODE_CH_NUM_BIT 2
#define I2S_MODE_WORD_LEN_BIT 5
#define I2S_MODE_FIRST_EDGE_BIT 6
#define I2S_MODE_LR_WS_BIT 7
#define I2S_MODE_COPY_ZERO_SEL_BIT 8
#define I2S_MODE_CLK_EDGE_BIT 9

#define I2S_MODE_MS_MODE_FIELD 0x1
#define I2S_MODE_I2S_DSP_MODE_FIELD 0x1
#define I2S_MODE_CH_NUM_FIELD 0x7
#define I2S_MODE_WORD_LEN_FIELD 0x1
#define I2S_MODE_FIRST_EDGE_FIELD 0x1
#define I2S_MODE_LR_WS_FIELD 0x1
#define I2S_MODE_COPY_ZERO_SEL_FIELD 0x1
#define I2S_MODE_CLK_EDGE_FIELD 0x1

#define I2S_DIV_WS_DIV_WS_L_BIT 0
#define I2S_DIV_WS_DIV_WS_H_BIT 8

#define I2S_DIV_WS_DIV_WS_L_FIELD 0xFF
#define I2S_DIV_WS_DIV_WS_H_FIELD 0xFF

#define I2S_BUF0_RDY_BIT 0
#define I2S_BUF1_RDY_BIT 0

#define I2S_BUF0_RDY_FIELD 0x1
#define I2S_BUF1_RDY_FIELD 0x1

#define I2S_CH_EN_CH0_EN_BIT 0
#define I2S_CH_EN_CH1_EN_BIT 1
#define I2S_CH_EN_CH2_EN_BIT 2
#define I2S_CH_EN_CH3_EN_BIT 3
#define I2S_CH_EN_CH4_EN_BIT 4
#define I2S_CH_EN_CH5_EN_BIT 5
#define I2S_CH_EN_CH6_EN_BIT 6
#define I2S_CH_EN_CH7_EN_BIT 7
#define I2S_CH_EN_CH8_EN_BIT 8
#define I2S_CH_EN_CH9_EN_BIT 9
#define I2S_CH_EN_CH10_EN_BIT 10
#define I2S_CH_EN_CH11_EN_BIT 11
#define I2S_CH_EN_CH12_EN_BIT 12
#define I2S_CH_EN_CH13_EN_BIT 13
#define I2S_CH_EN_CH14_EN_BIT 14
#define I2S_CH_EN_CH15_EN_BIT 15

#define I2S_CH_EN_CH0_EN_FIELD 0x1
#define I2S_CH_EN_CH1_EN_FIELD 0x1
#define I2S_CH_EN_CH2_EN_FIELD 0x1
#define I2S_CH_EN_CH3_EN_FIELD 0x1
#define I2S_CH_EN_CH4_EN_FIELD 0x1
#define I2S_CH_EN_CH5_EN_FIELD 0x1
#define I2S_CH_EN_CH6_EN_FIELD 0x1
#define I2S_CH_EN_CH7_EN_FIELD 0x1
#define I2S_CH_EN_CH8_EN_FIELD 0x1
#define I2S_CH_EN_CH9_EN_FIELD 0x1
#define I2S_CH_EN_CH10_EN_FIELD 0x1
#define I2S_CH_EN_CH11_EN_FIELD 0x1
#define I2S_CH_EN_CH12_EN_FIELD 0x1
#define I2S_CH_EN_CH13_EN_FIELD 0x1
#define I2S_CH_EN_CH14_EN_FIELD 0x1
#define I2S_CH_EN_CH15_EN_FIELD 0x1

#define I2S_CH_ERROR_CH0_ERROR_BIT 0
#define I2S_CH_ERROR_CH1_ERROR_BIT 1
#define I2S_CH_ERROR_CH2_ERROR_BIT 2
#define I2S_CH_ERROR_CH3_ERROR_BIT 3
#define I2S_CH_ERROR_CH4_ERROR_BIT 4
#define I2S_CH_ERROR_CH5_ERROR_BIT 5
#define I2S_CH_ERROR_CH6_ERROR_BIT 6
#define I2S_CH_ERROR_CH7_ERROR_BIT 7
#define I2S_CH_ERROR_CH8_ERROR_BIT 8
#define I2S_CH_ERROR_CH9_ERROR_BIT 9
#define I2S_CH_ERROR_CH10_ERROR_BIT 10
#define I2S_CH_ERROR_CH11_ERROR_BIT 11
#define I2S_CH_ERROR_CH12_ERROR_BIT 12
#define I2S_CH_ERROR_CH13_ERROR_BIT 13
#define I2S_CH_ERROR_CH14_ERROR_BIT 14
#define I2S_CH_ERROR_CH15_ERROR_BIT 15

#define I2S_CH_ERROR_CH0_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH1_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH2_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH3_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH4_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH5_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH6_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH7_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH8_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH9_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH10_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH11_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH12_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH13_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH14_ERROR_FIELD 0x1
#define I2S_CH_ERROR_CH15_ERROR_FIELD 0x1

#define I2S_INT_BUF_NOT_READY_BIT 0
#define I2S_INT_BUF_FLOW_BIT 1
#define I2S_INT_BUF0_TRF_DONE_BIT 2
#define I2S_INT_BUF1_TRF_DONE_BIT 3

#define I2S_INT_BUF_NOT_READY_FIELD 0x1
#define I2S_INT_BUF_FLOW_FIELD 0x1
#define I2S_INT_BUF0_TRF_DONE_FIELD 0x1
#define I2S_INT_BUF1_TRF_DONE_FIELD 0x1

#define I2S_SYS_RST     0X450
#define SYSCTL_I2S0RST_SHIFT    18
#define SYSCTL_I2S1RST_SHIFT    19

#define MOD_CH_NUM      (7 << 2)
#define MOD_1_CH_C      (4 << 2)
#define MOD_1_CH_P      (1 << 2)
#define MOD_2_CH        (0 << 2)
#define MOD_4_CH        (1 << 2)
#define MOD_8_CH        (2 << 2)
#define MOD_16_CH       (3 << 2)

#define MOD_WORD_LEN    (1 << 5)

#define MOD_I2S_MODE    (1 << 1)
#define MOD_MS_MODE     (1 << 0)

#define I2S0_CLKCTL_BASE 0X350
#define I2S1_CLKCTL_BASE 0X360
#define CLK_MODE        20
#define BCLK_SHIFT      16
#define MCLK_SHIFT      8
#define PRE_MCLK_SHIFT  0

#define CTL_IMEM_CLR    (1 << 2)
#define CTL_IMEM_EN     (1 << 1)
#define CTL_ENABLE      (1 << 0)

#define UPDATE_VALUE_FIELD(legacy_val, update_val, bit_pos, bitfield) \
((legacy_val & ~(bitfield << bit_pos)) \
| ((update_val & bitfield) << bit_pos))

#define I2S_DEBUG printk

#endif

#endif /* __SND_SOC_X2_I2S_REGS_H */
