/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __X2_I2S_REG_H
#define __X2_I2S_REG_H

#include <asm-generic/io.h>

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

#define UPDATE_VALUE_FIELD(legacy_val, update_val, bit_pos, bitfield) ((legacy_val & ~(bitfield << bit_pos)) | ((update_val & bitfield) << bit_pos))

#define I2S_DEBUG printk

typedef enum _i2s_mode_ms_mode {
	X2_I2S_MASTER_MODE = 0,
	X2_I2S_SLAVE_MODE = 1,
	X2_I2S_MS_MODE_MAX = 2,
} e_i2s_mode_ms_mode;

typedef enum _i2s_mode_dsp_mode {
	X2_I2S_I2S_MODE = 0,
	X2_I2S_DSP_MODE = 1,
	X2_I2S_I2S_DSP_MAX = 2,
} e_i2s_mode_dsp_mode;

typedef enum _i2s_mode_ch_num {
	X2_I2S_RX_1_CHANNEL = 4,
	X2_I2S_RX_2_CHANNEL = 0,
	X2_I2S_RX_4_CHANNEL = 1,
	X2_I2S_RX_8_CHANNEL = 2,
	X2_I2S_RX_16_CHANNEL = 3,
	X2_I2S_RX_CH_NUM_MAX = 5,
	X2_I2S_TX_1_CHANNEL = 1,
    X2_I2S_TX_2_CHANNEL = 0,
    X2_I2S_TX_CH_NUM_MAX = 2,
} e_i2s_mode_ch_num;

typedef enum _i2s_mode_word_len {
	X2_I2S_8_BITS = 0,
	X2_I2S_16_BITS = 1,
	X2_I2S_WORD_LEN_MAX = 2,
} e_i2s_mode_word_len;

typedef enum _i2s_mode_first_edge {
	X2_I2S_2nd_CLK_EDGE = 0,
	X2_I2S_1st_CLK_EDGE = 1,
	X2_I2S_FIRST_EDGE_MAX = 2,
} e_i2s_mode_first_edge;

typedef enum _i2s_mode_lr_ws {
	X2_I2S_LOW_LEFT = 0,
	X2_I2S_LOW_HIGH = 1,
	X2_I2S_LR_WS_MAX = 2,
} e_i2s_mode_lr_ws;

typedef enum _i2s_mode_copy_zero_sel {
	X2_I2S_TREAT_ZERO = 0,
	X2_I2S_TREAT_COPY = 1,
	X2_I2S_COPY_ZERO_SEL_MAX = 2,
} e_i2s_mode_copy_zero_sel;

typedef enum _i2s_mode_clk_edge {
	X2_I2S_NEGATIVE_EDGE_CAPTURE = 0,
	X2_I2S_POSITIVE_EDGE_CAPTURE = 1,
	X2_I2S_CLK_EDGE_MAX = 2,
} e_i2s_mode_clk_edge;

typedef struct _x2_i2s_hw_cfg {
        e_i2s_mode_ms_mode ms_mode;
        e_i2s_mode_dsp_mode i2s_dsp_mode;
        e_i2s_mode_ch_num ch_num;
        e_i2s_mode_word_len word_len;
        e_i2s_mode_first_edge first_edge;
        e_i2s_mode_lr_ws lr_ws;
        e_i2s_mode_copy_zero_sel copy_zero_sel;
        e_i2s_mode_clk_edge clk_edge;
} x2_i2s_hw_cfg;

#define X2_I2S_FRAME_CACHE_SIZE 15
#define X2_I2S_MASTER_BCLK_KHZ 3072
#define X2_I2S_SLAVE_BCLK_KHZ 3072


typedef struct _x2_i2s_buf {
	struct list_head node;
	void *vptr;
    void *pptr;
} x2_i2s_buf;

typedef struct _x2_i2s_buf_state {
    struct list_head node;
    int buf_no;
} x2_i2s_buf_state;

typedef struct _x2_i2s_frame{
	struct list_head free;
	struct list_head ready;
    struct list_head state;
	x2_i2s_buf *buf0;
	x2_i2s_buf *buf1;
    x2_i2s_buf_state buf0_state;
    x2_i2s_buf_state buf1_state;
    struct semaphore sem;
	void *vbuf;
	void *pbuf;
	int size;
	int num;
} x2_i2s_frame;

typedef struct _x2_i2s{
    void __iomem *rx_regs;
    void __iomem *tx_regs;
    void __iomem *sys_regs;
    void __iomem *apb_regs;
    void __iomem *regs;
    unsigned int irq;
	unsigned int clk_reg;
    unsigned int status;
    unsigned int index;
    int bclk;
    struct device *dev;
	e_i2s_mode_ms_mode state;
	x2_i2s_hw_cfg hw_cfg;
	x2_i2s_frame frame;
    struct reset_control *rst;
} x2_i2s;

#define X2_I2S_DEV_NUMBER 2
extern x2_i2s *g_x2_i2s[X2_I2S_DEV_NUMBER];

static inline unsigned int x2_i2s_read_base_reg(x2_i2s *i2s, int offset)
{
    return readl(i2s->regs+offset);
}

static inline void x2_i2s_iram_clear(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_CTL);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_CTL_IMEM_CLEAR_BIT, I2S_CTL_IMEM_CLEAR_FIELD), i2s->regs + I2S_CTL);
}

static inline void x2_i2s_iram_enable(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_CTL);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_CTL_IMEM_EN_BIT, I2S_CTL_IMEM_EN_FIELD), i2s->regs + I2S_CTL);
}

static inline void x2_i2s_transfer_enable(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_CTL);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_CTL_ENABLE_BIT, I2S_CTL_ENABLE_FIELD), i2s->regs + I2S_CTL);
}

static inline void x2_i2s_disable(x2_i2s *i2s)
{
    writel(0x0, i2s->regs + I2S_CTL);
}

static inline void x2_i2s_ms_mode_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_MODE);
	writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_MODE_MS_MODE_BIT, I2S_MODE_MS_MODE_FIELD), i2s->regs + I2S_MODE);
}

static inline void x2_i2s_interface_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_MODE);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_MODE_I2S_DSP_MODE_BIT, I2S_MODE_I2S_DSP_MODE_FIELD), i2s->regs + I2S_MODE);
}

static inline void x2_i2s_channel_num_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_MODE);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_MODE_CH_NUM_BIT, I2S_MODE_CH_NUM_FIELD), i2s->regs + I2S_MODE);
}

static inline void x2_i2s_sample_depth_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_MODE);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_MODE_WORD_LEN_BIT, I2S_MODE_WORD_LEN_FIELD), i2s->regs + I2S_MODE);
}

static inline void x2_i2s_first_edge_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_MODE);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_MODE_FIRST_EDGE_BIT, I2S_MODE_FIRST_EDGE_FIELD), i2s->regs + I2S_MODE);
}

static inline void x2_i2s_ws_lr_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_MODE);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_MODE_LR_WS_BIT, I2S_MODE_LR_WS_FIELD), i2s->regs + I2S_MODE);
}

static inline void x2_i2s_copy_zero_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_MODE);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_MODE_COPY_ZERO_SEL_BIT, I2S_MODE_COPY_ZERO_SEL_FIELD), i2s->regs + I2S_MODE);
}

static inline void x2_i2s_clk_edge_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_MODE);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_MODE_CLK_EDGE_BIT, I2S_MODE_CLK_EDGE_FIELD), i2s->regs + I2S_MODE);
}

static inline void x2_i2s_ws_lclk_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_DIV_WS);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_DIV_WS_DIV_WS_L_BIT, I2S_DIV_WS_DIV_WS_L_FIELD), i2s->regs + I2S_DIV_WS);
}

static inline void x2_i2s_ws_hclk_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
    reg_val = readl(i2s->regs + I2S_DIV_WS);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_DIV_WS_DIV_WS_H_BIT, I2S_DIV_WS_DIV_WS_H_FIELD), i2s->regs + I2S_DIV_WS);
}

static inline void x2_i2s_channel_en(x2_i2s *i2s, int val)
{
    writel(val, i2s->regs + I2S_CH_EN);
}

static inline void x2_i2s_buf_size_set(x2_i2s *i2s, int val)
{
    writel(val, i2s->regs + I2S_BUF_SIZE);
}

static inline void x2_i2s_buf0_addr_set(x2_i2s *i2s, int val)
{
    writel(val, i2s->regs + I2S_BUF0_ADDR);
}

static inline void x2_i2s_buf0_rdy_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_BUF0_RDY);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_BUF0_RDY_BIT, I2S_BUF0_RDY_FIELD), i2s->regs + I2S_BUF0_RDY);
}

static inline unsigned int x2_i2s_buf0_rdy_get(x2_i2s *i2s)
{
	return readl(i2s->regs + I2S_BUF0_RDY);
}

static inline void x2_i2s_buf1_addr_set(x2_i2s *i2s, int val)
{
    writel(val, i2s->regs + I2S_BUF1_ADDR);
}

static inline void x2_i2s_buf1_rdy_set(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_BUF1_RDY);
    writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_BUF1_RDY_BIT, I2S_BUF1_RDY_FIELD), i2s->regs + I2S_BUF1_RDY);
}

static inline unsigned int x2_i2s_buf1_rdy_get(x2_i2s *i2s)
{
	return readl(i2s->regs + I2S_BUF1_RDY);
}

static inline unsigned int x2_i2s_buf_addr_get(x2_i2s *i2s)
{
    return readl(i2s->regs + I2S_BUF_CUR_ADDR);
}

static inline unsigned int x2_i2s_channel_error_get(x2_i2s *i2s)
{
    return readl(i2s->regs + I2S_CH_ERROR);
}

static inline unsigned int x2_i2s_int_status_get(x2_i2s *i2s)
{
    return readl(i2s->regs + I2S_SRCPND);
}

static inline void x2_i2s_int_buf1_trf_done_set(x2_i2s *i2s)
{
    writel(0x08, i2s->regs + I2S_SRCPND);
}

static inline void x2_i2s_int_buf0_trf_done_set(x2_i2s *i2s)
{
    writel(0x04, i2s->regs + I2S_SRCPND);
}

static inline void x2_i2s_int_buf_flow_set(x2_i2s *i2s)
{
    writel(0x02, i2s->regs + I2S_SRCPND);
}

static inline void x2_i2s_int_buf_not_ready_set(x2_i2s *i2s)
{
    writel(0x01, i2s->regs + I2S_SRCPND);
}

static inline void x2_i2s_int_state_clear(x2_i2s *i2s)
{
	writel(0xF, i2s->regs + I2S_SRCPND);
}

static inline unsigned int x2_i2s_int_mask_get(x2_i2s *i2s)
{
    return readl(i2s->regs + I2S_INTMASK);
}

static inline void x2_i2s_int_buf1_trf_done_mask(x2_i2s *i2s)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_SETMASK);
    writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF1_TRF_DONE_BIT, I2S_INT_BUF1_TRF_DONE_FIELD), i2s->regs + I2S_SETMASK);
}

static inline void x2_i2s_int_buf0_trf_done_mask(x2_i2s *i2s)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_SETMASK);
    writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF0_TRF_DONE_BIT, I2S_INT_BUF0_TRF_DONE_FIELD), i2s->regs + I2S_SETMASK);
}

static inline void x2_i2s_int_buf_flow_mask(x2_i2s *i2s)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_SETMASK);
    writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF_FLOW_BIT, I2S_INT_BUF_FLOW_FIELD), i2s->regs + I2S_SETMASK);
}

static inline void x2_i2s_int_buf_not_ready_mask(x2_i2s *i2s)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_SETMASK);
    writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF_NOT_READY_BIT, I2S_INT_BUF_NOT_READY_FIELD), i2s->regs + I2S_SETMASK);
}

static inline void x2_i2s_int_buf1_trf_done_unmask(x2_i2s *i2s)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_UNMASK);
    writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF1_TRF_DONE_BIT, I2S_INT_BUF1_TRF_DONE_FIELD), i2s->regs + I2S_UNMASK);
}

static inline void x2_i2s_int_buf0_trf_done_unmask(x2_i2s *i2s)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_UNMASK);
    writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF0_TRF_DONE_BIT, I2S_INT_BUF0_TRF_DONE_FIELD), i2s->regs + I2S_UNMASK);
}

static inline void x2_i2s_int_buf_flow_unmask(x2_i2s *i2s)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_UNMASK);
    writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF_FLOW_BIT, I2S_INT_BUF_FLOW_FIELD), i2s->regs + I2S_UNMASK);
}

static inline void x2_i2s_int_buf_not_ready_unmask(x2_i2s *i2s)
{
    unsigned int reg_val;
	reg_val = readl(i2s->regs + I2S_UNMASK);
    writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF_NOT_READY_BIT, I2S_INT_BUF_NOT_READY_FIELD), i2s->regs + I2S_UNMASK);
}

static inline void x2_i2s_int_unmask(x2_i2s *i2s)
{
    writel(0xF, i2s->regs + I2S_UNMASK);
}

#define I2S_CLK_CTRL_PRE_MCLK_DIV_SEL_BIT 0
#define I2S_CLK_CTRL_MCLK_DIV_SEL_BIT 8
#define I2S_CLK_CTRL_DIV_BCLK_DIV_SEL_BIT 16
#define I2S_CLK_CTRL_MST_SLV_MODE_BIT 20
#define I2S_CLK_CTRL_BCLK_PHASE_SEL_BIT 24

#define I2S_CLK_CTRL_PRE_MCLK_DIV_FIELD 0x1F
#define I2S_CLK_CTRL_MCLK_DIV_SEL_FIELD 0x1F
#define I2S_CLK_CTRL_DIV_BCLK_DIV_SEL_FIELD 0x7
#define I2S_CLK_CTRL_MST_SLV_MODE_FIELD 0x1
#define I2S_CLK_CTRL_BCLK_PHASE_SEL_FIELD 0x1

static inline void x2_i2s_clk_pre_mclk_div_config(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
	reg_val = readl(i2s->sys_regs + i2s->clk_reg);
	writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_CLK_CTRL_PRE_MCLK_DIV_SEL_BIT, I2S_CLK_CTRL_PRE_MCLK_DIV_FIELD), i2s->sys_regs + i2s->clk_reg);
}

static inline void x2_i2s_clk_mclk_div_config(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
	reg_val = readl(i2s->sys_regs + i2s->clk_reg);
	writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_CLK_CTRL_MCLK_DIV_SEL_BIT, I2S_CLK_CTRL_MCLK_DIV_SEL_FIELD), i2s->sys_regs + i2s->clk_reg);
}

static inline void x2_i2s_clk_bclk_div_config(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
	reg_val = readl(i2s->sys_regs + i2s->clk_reg);
	writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_CLK_CTRL_DIV_BCLK_DIV_SEL_BIT, I2S_CLK_CTRL_DIV_BCLK_DIV_SEL_FIELD), i2s->sys_regs + i2s->clk_reg);
}

static inline void x2_i2s_clk_ms_mode_config(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
	reg_val = readl(i2s->sys_regs + i2s->clk_reg);
	writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_CLK_CTRL_MST_SLV_MODE_BIT, I2S_CLK_CTRL_MST_SLV_MODE_FIELD), i2s->sys_regs + i2s->clk_reg);
}

static inline void x2_i2s_clk_bclk_phase_config(x2_i2s *i2s, int val)
{
    unsigned int reg_val;
	reg_val = readl(i2s->sys_regs + i2s->clk_reg);
	writel(UPDATE_VALUE_FIELD(reg_val, val, I2S_CLK_CTRL_BCLK_PHASE_SEL_BIT, I2S_CLK_CTRL_BCLK_PHASE_SEL_FIELD), i2s->sys_regs + i2s->clk_reg);
}

static inline void x2_i2s_apb_timeout_enable(x2_i2s *i2s)
{
    writel(0x1, i2s->apb_regs);
}

int x2_i2s_pre_init(x2_i2s *i2s);
int x2_i2s_ms_mode_config(x2_i2s *i2s, int mode);
int x2_i2s_dsp_mode_config(x2_i2s *i2s, int mode);
int x2_i2s_ch_num_config(x2_i2s *i2s, int mode);
int x2_i2s_word_len_config(x2_i2s *i2s, int mode);
int x2_i2s_first_edge_config(x2_i2s *i2s, int mode);
int x2_i2s_lr_ws_config(x2_i2s *i2s, int mode);
int x2_i2s_copy_zero_config(x2_i2s *i2s, int mode);
int x2_i2s_clk_edge_config(x2_i2s *i2s, int mode);
int x2_i2s_buf_swap(x2_i2s *i2s, int buf_no);
int x2_i2s_buf_alloc(x2_i2s *i2s, int size);
int x2_i2s_buf_update(x2_i2s *i2s);
int x2_i2s_sample_rate_set(x2_i2s *i2s, int sample_rate);
int x2_i2s_buf_size_get(x2_i2s *i2s);
int x2_i2s_buf_num_get(x2_i2s *i2s);
int x2_i2s_buf_dealloc(x2_i2s *i2s);
int x2_i2s_start(x2_i2s *i2s);
int x2_i2s_stop(x2_i2s *i2s);
int x2_i2s_pause(x2_i2s *i2s);
int x2_i2s_restart(x2_i2s *i2s);

#endif
