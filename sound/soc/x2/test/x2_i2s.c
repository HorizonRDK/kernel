/***************************************************************************
 *						COPYRIGHT NOTICE
 *			   Copyright 2018 Horizon Robotics, Inc.
 *					   All rights reserved.
 ***************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/irqreturn.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/stddef.h>
#include <linux/gfp.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/page-flags.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include "x2_i2s_reg.h"
enum _X2_I2S_INT_MASK{
	BUF1_TRF_DONE = 0x8,
	BUF0_TRF_DONE = 0x4,
	BUF_FLOW = 0x2,
	BUF_NOT_READY = 0x1,
};

enum _x2_I2S_STATE{
	I2S_STATE_STOP = 0x0,
	I2S_STATE_PAUSE = 0x1,
	I2S_STATE_START = 0x2,
};

enum _x2_I2S_SAMPLE_RATE{
	I2S_SAMPLE_RATE_8_KHZ = 0x0,
	I2S_SAMPLE_RATE_16_KHZ = 0x1,
	I2S_SAMPLE_RATE_32_KHZ = 0x2,
	I2S_SAMPLE_RATE_48_KHZ = 0x3,
	I2S_SAMPLE_RATE_MAX = 0x4,
};

x2_i2s *g_x2_i2s[X2_I2S_DEV_NUMBER];

static int x2_i2s_master_clk_enable(x2_i2s *i2s)
{
	clk_enable(i2s->bclk);
	return 0;
}

static int x2_i2s_slave_clk_enable(x2_i2s *i2s)
{
	clk_disable(i2s->bclk);
	return 0;
}

int x2_i2s_pre_init(x2_i2s *i2s)
{
	int ret;
	clk_enable(i2s->mclk);
	ret = x2_i2s_master_clk_enable(i2s);
	if(ret){
		printk("%s: failed to cpnfig clk for i2s%d.\n", __func__, i2s->index);
		return -EINVAL;
	}
	reset_control_assert(i2s->rst);
	ndelay(100);
	reset_control_deassert(i2s->rst);

	x2_i2s_disable(i2s);
	return 0;
}

int x2_i2s_rxtx_mode_config(x2_i2s *i2s, int mode)
{
	int ret;
	if(mode < X2_I2S_RXTX_MODE_MAX){
		if(X2_I2S_TX_MODE == mode){
			i2s->regs = i2s->tx_regs;
		}
		else if(X2_I2S_RX_MODE == mode){
			i2s->regs = i2s->rx_regs;
		}
		else
			return -EINVAL;
		i2s->rxtx_state = (e_i2s_mode_rxtx_mode)mode;
		ret = x2_i2s_pre_init(i2s);
		if(ret)
			return -EINVAL;
	}else
		return -EINVAL;
}

int x2_i2s_ms_mode_config(x2_i2s *i2s, int mode)
{
	int ret;
	if(mode < X2_I2S_MS_MODE_MAX){
		i2s->state = (e_i2s_mode_ms_mode)mode;
		//x2_i2s_ms_mode_set(i2s, mode);
		i2s->clk = clk_get_rate(i2s->bclk);
		printk("%s: i2s%d clk val %ld.\n", __func__, i2s->index, i2s->clk);
		i2s->hw_cfg.ms_mode = (e_i2s_mode_ms_mode)mode;
		x2_i2s_iram_clear(i2s, 1);
		return 0;
	}
	else
		return -EINVAL;
}

int x2_i2s_dsp_mode_config(x2_i2s *i2s, int mode)
{
	if(mode < X2_I2S_I2S_DSP_MAX){
		x2_i2s_interface_set(i2s, mode);
		i2s->hw_cfg.i2s_dsp_mode = (e_i2s_mode_dsp_mode)mode;
		return 0;
	}
	else
		return -EINVAL;
}

int x2_i2s_ch_num_config(x2_i2s *i2s, int mode)
{
	if(X2_I2S_TX_MODE == i2s->rxtx_state){
		if(mode >= X2_I2S_TX_CH_NUM_MAX)
			return -EINVAL;
	}else if(X2_I2S_RX_MODE == i2s->rxtx_state){
		if(mode >= X2_I2S_RX_CH_NUM_MAX)
			return -EINVAL;
	}else
		return -EINVAL;
	x2_i2s_channel_num_set(i2s, mode);
	i2s->hw_cfg.ch_num = (e_i2s_mode_ch_num)mode;
	return 0;
}

int x2_i2s_word_len_config(x2_i2s *i2s, int mode)
{
	if(mode < X2_I2S_WORD_LEN_MAX){
		x2_i2s_sample_depth_set(i2s, mode);
		i2s->hw_cfg.word_len = (e_i2s_mode_word_len)mode;
		return 0;
	}
	else
		return -EINVAL;
}

int x2_i2s_first_edge_config(x2_i2s *i2s, int mode)
{
	if(mode < X2_I2S_FIRST_EDGE_MAX){
		x2_i2s_first_edge_set(i2s, mode);
		i2s->hw_cfg.first_edge = (e_i2s_mode_first_edge)mode;
		return 0;
	}
	else
		return -EINVAL;
}

int x2_i2s_lr_ws_config(x2_i2s *i2s, int mode)
{
	if(mode < X2_I2S_LR_WS_MAX){
		x2_i2s_ws_lr_set(i2s, mode);
		i2s->hw_cfg.lr_ws = (e_i2s_mode_lr_ws)mode;
		return 0;
	}
	else
		return -EINVAL;
}

int x2_i2s_copy_zero_config(x2_i2s *i2s, int mode)
{
	if(mode < X2_I2S_COPY_ZERO_SEL_MAX){
		x2_i2s_copy_zero_set(i2s, mode);
		i2s->hw_cfg.copy_zero_sel = (e_i2s_mode_copy_zero_sel)mode;
		return 0;
	}
	else
		return -EINVAL;
}

int x2_i2s_clk_edge_config(x2_i2s *i2s, int mode)
{
	if(mode < X2_I2S_CLK_EDGE_MAX){
		x2_i2s_clk_edge_set(i2s, mode);
		i2s->hw_cfg.clk_edge = (e_i2s_mode_clk_edge)mode;
		return 0;
	}
	else
		return -EINVAL;
}

int x2_i2s_buf_swap(x2_i2s *i2s, int buf_no)
{
	x2_i2s_buf *buf;
	struct list_head *free_head = &i2s->frame.free;
	struct list_head *ready_head = &i2s->frame.ready;
	if(X2_I2S_TX_MODE == i2s->rxtx_state){
		if(!list_empty(ready_head)){
			buf = container_of(ready_head->next, x2_i2s_buf, node);
			list_del(&buf->node);
			switch(buf_no){
				case 0:
					x2_i2s_buf0_addr_set(i2s, (int)buf->pptr);
					if(i2s->frame.buf0 != NULL){
						list_add_tail(&i2s->frame.buf0->node, free_head);
					}
					i2s->frame.buf0 = buf;
					x2_i2s_buf0_rdy_set(i2s, 0x1);
					break;
				case 1:
					x2_i2s_buf1_addr_set(i2s, (int)buf->pptr);
					if(i2s->frame.buf1 != NULL){
						list_add_tail(&i2s->frame.buf1->node, free_head);
					}
					i2s->frame.buf1 = buf;
					x2_i2s_buf1_rdy_set(i2s, 0x1);
					break;
				default:
					return -EINVAL;
			}
		}else{
			switch(buf_no){
				case 0:
					I2S_DEBUG("%s: no available entry for master i2s%d buf0, waiting.\n", __func__, i2s->index);
					x2_i2s_buf0_rdy_set(i2s, 0x1); /*repeat the last frame for test*/
/*
					if(i2s->frame.buf0 != NULL){
						list_add_tail(&i2s->frame.buf0->node, free_head);
					}
					i2s->frame.buf0 = NULL;
					list_add_tail(&i2s->frame.buf0_state.node, &i2s->frame.state);
*/
					break;
				case 1:
					I2S_DEBUG("%s: no available entry for slave i2s%d buf1, waiting.\n", __func__, i2s->index);
					x2_i2s_buf1_rdy_set(i2s, 0x1);/*repeat send the last frame for test*/
/*
					if(i2s->frame.buf1 != NULL){
						list_add_tail(&i2s->frame.buf1->node, free_head);
					}
					i2s->frame.buf1 = NULL;
					list_add_tail(&i2s->frame.buf1_state.node, &i2s->frame.state);
*/
					break;
				default:
					return -EINVAL;
			}
			return 1;
		}
	}else if(X2_I2S_RX_MODE == i2s->rxtx_state){
		if(!list_empty(free_head)){
			buf = container_of(free_head->next, x2_i2s_buf, node);
			list_del(&buf->node);
			switch(buf_no){
				case 0:
					x2_i2s_buf0_addr_set(i2s, (int)buf->pptr);
					if(i2s->frame.buf0 != NULL){
						up(&i2s->frame.sem);
						list_add_tail(&i2s->frame.buf0->node, ready_head);
					}
					i2s->frame.buf0 = buf;
					x2_i2s_buf0_rdy_set(i2s, 0x1);
					break;
				case 1:
					x2_i2s_buf1_addr_set(i2s, (int)buf->pptr);
					if(i2s->frame.buf1 != NULL){
						up(&i2s->frame.sem);
						list_add_tail(&i2s->frame.buf1->node, ready_head);
					}
					i2s->frame.buf1 = buf;
					x2_i2s_buf1_rdy_set(i2s, 0x1);
					break;
				default:
					return -EINVAL;
			}
		}else{
			switch(buf_no){
				case 0:
					I2S_DEBUG("%s: no available entry for i2s%d buf0, reuse the latest.\n", __func__, i2s->index);
					x2_i2s_buf0_rdy_set(i2s, 0x1);
					break;
				case 1:
					I2S_DEBUG("%s: no available entry for i2s%d buf1, reuse the latest.\n", __func__, i2s->index);
					x2_i2s_buf1_rdy_set(i2s, 0x1);
					break;
				default:
					return -EINVAL;
			}
			return 1;
		}
	}else{
		printk("%s: invalid i2s mode!\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int x2_i2s_get_channel_num(x2_i2s *i2s, e_i2s_mode_ch_num chan)
{
	if(X2_I2S_RX_MODE == i2s->rxtx_state){
		switch(chan){
			case X2_I2S_RX_1_CHANNEL:
				return 1;
			case X2_I2S_RX_2_CHANNEL:
				return 2;
			case X2_I2S_RX_4_CHANNEL:
				return 4;
			case X2_I2S_RX_8_CHANNEL:
				return 8;
			case X2_I2S_RX_16_CHANNEL:
				return 16;
			default:
				return -EINVAL;
		}
	}else if(X2_I2S_TX_MODE == i2s->rxtx_state){
		switch(chan){
			case X2_I2S_TX_1_CHANNEL:
				return 1;
			case X2_I2S_TX_2_CHANNEL:
				return 2;
			default:
				return -EINVAL;
	   }
	}else
		return -EINVAL;
}

static int x2_i2s_get_word_len(e_i2s_mode_word_len len)
{
	switch(len){
		case X2_I2S_8_BITS:
			return 8;
		case X2_I2S_16_BITS:
			return 16;
		default:
			return -1;
	}
}

int x2_i2s_buf_alloc(x2_i2s *i2s, int size)
{
	int i;
	int channel;
	dma_addr_t phy;
	size = ALIGN(size, 4096);
	i2s->frame.vbuf = dmam_alloc_coherent(i2s->dev, size, &phy, GFP_KERNEL);
	if(!i2s->frame.vbuf){
		printk("%s: failed to malloc the user buffer for i2s%d!\n", __func__, i2s->index);
		return -ENOMEM;
	}
	i2s->frame.pbuf = phy;
	I2S_DEBUG("%s: malloc mmap buf for i2s%d: vaddr:0x%x, paddr:0x%x.\n", __func__, i2s->index, i2s->frame.vbuf, i2s->frame.pbuf);

	INIT_LIST_HEAD(&i2s->frame.free);
	INIT_LIST_HEAD(&i2s->frame.ready);
	INIT_LIST_HEAD(&i2s->frame.state);
	sema_init(&i2s->frame.sem, 0);

	for(i = 0; i < X2_I2S_FRAME_CACHE_SIZE; i++){
		x2_i2s_buf *buf = devm_kzalloc(i2s->dev, sizeof(x2_i2s_buf), GFP_KERNEL);
		if(!buf){
			printk("%s: failed to malloc the buffer for i2s%d!\n", __func__, i2s->index);
			return -ENOMEM;
		}
		buf->vptr = dmam_alloc_coherent(i2s->dev, size, &phy, GFP_KERNEL);
		if(!buf->vptr){
			printk("%s: failed to malloc the cached buffer for i2s%d!\n",  __func__, i2s->index);
			return -ENOMEM;
		}
		buf->pptr = phy;
		list_add_tail(&buf->node, &i2s->frame.free);
		I2S_DEBUG("%s: malloc %d/%d cached buf for i2s%d: vaddr:0x%x paddr:0x%x\n", __func__, i, X2_I2S_FRAME_CACHE_SIZE, i2s->index, buf->vptr, buf->pptr);
	}

	i2s->frame.size = size;

	channel = x2_i2s_get_channel_num(i2s, i2s->hw_cfg.ch_num);
	if(channel <0){
		printk("%s: got invalid channel number for i2s%d!\n", __func__, i2s->index);
		return -EINVAL;
	}

	x2_i2s_buf_size_set(i2s, i2s->frame.size / channel);
	i2s->frame.num = X2_I2S_FRAME_CACHE_SIZE;
	i2s->frame.buf0_state.buf_no = 0;
	i2s->frame.buf1_state.buf_no = 1;
	list_add_tail(&i2s->frame.buf0_state.node, &i2s->frame.state);
	list_add_tail(&i2s->frame.buf1_state.node, &i2s->frame.state);

	x2_i2s_buf_state *buf_state;
	struct list_head *state_head = &i2s->frame.state;
	if(X2_I2S_RX_MODE == i2s->rxtx_state){
		while(!list_empty(state_head)){
			buf_state = container_of(state_head->next, x2_i2s_buf_state, node);
			list_del(&buf_state->node);
			x2_i2s_buf_swap(i2s, buf_state->buf_no);
		}
	}
	return i;
}

int x2_i2s_buf_update(x2_i2s *i2s)
{
	x2_i2s_buf *buf;
	x2_i2s_buf_state *buf_state;
	struct list_head *free_head = &i2s->frame.free;
	struct list_head *ready_head = &i2s->frame.ready;
	struct list_head *state_head = &i2s->frame.state;
	if(X2_I2S_TX_MODE == i2s->rxtx_state){
		if(!list_empty(free_head)){
			buf = container_of(free_head->next, x2_i2s_buf, node);
			list_del(&buf->node);
			I2S_DEBUG("%s: update the transfer buffer for i2s%d from 0x%x to 0x%x\n", __func__, i2s->index, i2s->frame.vbuf, buf->vptr);
			memcpy(buf->vptr, i2s->frame.vbuf, i2s->frame.size);
			list_add_tail(&buf->node, ready_head);
		}else{
			printk("%s: no entry available for master i2s%d!\n", __func__, i2s->index);
			return 1;
		}
		if(!list_empty(state_head)){
			buf_state = container_of(state_head->next, x2_i2s_buf_state, node);
			list_del(&buf_state->node);
			x2_i2s_buf_swap(i2s, buf_state->buf_no);
		}
	}else if(X2_I2S_RX_MODE == i2s->rxtx_state){
		if(!list_empty(ready_head)){
			buf = container_of(ready_head->next, x2_i2s_buf, node);
			list_del(&buf->node);
			printk("i2s%d trying to update the receive buffer, from 0x%x to 0x%x\n", i2s->index, i2s->frame.vbuf, buf->vptr);
			memcpy(i2s->frame.vbuf, buf->vptr, i2s->frame.size);
			list_add_tail(&buf->node, free_head);
		}else{
			printk("%s: no entry available for slave i2s%d!\n", __func__, i2s->index);
			return 1;
		}
	}else{
		printk("%s: invalid mode for i2s%d!\n", __func__, i2s->index);
		return -EINVAL;
	}

	return 0;
}

int x2_i2s_buf_size_get(x2_i2s *i2s)
{
	return i2s->frame.size;
}

int x2_i2s_buf_num_get(x2_i2s *i2s)
{
	struct list_head *list;
	struct list_head *free_head = &i2s->frame.free;
	struct list_head *ready_head = &i2s->frame.ready;
	int num = 0;
	if(X2_I2S_TX_MODE == i2s->rxtx_state){
		list_for_each(list, free_head){
			num++;
		}
	}else if(X2_I2S_RX_MODE == i2s->rxtx_state){
		list_for_each(list, ready_head){
			num++;
		}
	}
	else{
		printk("%s: invalid i2s ms mode!\n", __func__);
		return -EINVAL;
	}
	printk("%s: i2s%d get available entry number 0x%x \n", __func__, i2s->index, num);
	return num;
}

int x2_i2s_buf_dealloc(x2_i2s *i2s)
{
	struct list_head *head;
	struct list_head *next;
	struct list_head *free_head = &i2s->frame.free;
	struct list_head *ready_head = &i2s->frame.ready;
	struct list_head *state_head = &i2s->frame.state;
	x2_i2s_buf *buf;
	list_for_each_safe(head, next, free_head){
		buf = container_of(head, x2_i2s_buf, node);
		list_del(&buf->node);
		dmam_free_coherent(i2s->dev, i2s->frame.size, buf->vptr, buf->pptr);
		devm_kfree(i2s->dev, buf);
	}
	free_head = NULL;

	list_for_each_safe(head, next, ready_head){
		buf = container_of(head, x2_i2s_buf, node);
		list_del(&buf->node);
		dmam_free_coherent(i2s->dev, i2s->frame.size, buf->vptr, buf->pptr);
		devm_kfree(i2s->dev, buf);
	}
	ready_head = NULL;
	state_head = NULL;

	dmam_free_coherent(i2s->dev, i2s->frame.size, i2s->frame.buf0->vptr, i2s->frame.buf0->pptr);
	devm_kfree(i2s->dev, i2s->frame.buf0);
	i2s->frame.buf0 = NULL;

	dmam_free_coherent(i2s->dev, i2s->frame.size, i2s->frame.buf1->vptr, i2s->frame.buf1->pptr);
	devm_kfree(i2s->dev, i2s->frame.buf1);
	i2s->frame.buf1 = NULL;
	i2s->frame.size = 0;
	return 0;
}

static int x2_i2s_channel_enable(x2_i2s *i2s)
{
	int channel;
	x2_i2s_channel_en(i2s, 0);

	if(X2_I2S_RX_MODE == i2s->rxtx_state){
		switch(i2s->hw_cfg.ch_num)
		{
			case X2_I2S_RX_1_CHANNEL:
				channel = 0x1;
				break;
			case X2_I2S_RX_2_CHANNEL:
				channel = 0x3;
				break;
			case X2_I2S_RX_4_CHANNEL:
				channel = 0xF;
				break;
			case X2_I2S_RX_8_CHANNEL:
				channel = 0xFF;
				break;
			case X2_I2S_RX_16_CHANNEL:
				channel = 0xFFFF;
				break;
			default:
				channel = 0;
				break;
		}
	}
	else if(X2_I2S_TX_MODE == i2s->rxtx_state){
		switch(i2s->hw_cfg.ch_num){
			case X2_I2S_TX_1_CHANNEL:
				channel = 0x1;
				break;
			case X2_I2S_TX_2_CHANNEL:
				channel = 0x3;
				break;
		}
	}
	else{
		printk("%s: invalid i2s state for i2s%d.\n", __func__, i2s->index);
		return -EINVAL;
	}

	x2_i2s_channel_en(i2s, channel);
}

int x2_i2s_sample_rate_set(x2_i2s *i2s, int sample_rate)
{
	int div_ws_l, div_ws_h;
	int ws_l, ws_h;
	int channel, word_len;
	if(sample_rate > I2S_SAMPLE_RATE_MAX){
		printk("%s: invalid sample rate fot i2s%d!\n", __func__, i2s->index);
		return -EINVAL;
	}

	channel = x2_i2s_get_channel_num(i2s, i2s->hw_cfg.ch_num);
	word_len = x2_i2s_get_word_len(i2s->hw_cfg.word_len);
	if(X2_I2S_I2S_MODE == i2s->hw_cfg.i2s_dsp_mode){
		div_ws_l = word_len * channel / 2;
		div_ws_h = div_ws_l;
		switch(sample_rate){
			case I2S_SAMPLE_RATE_8_KHZ:
				ws_h = i2s->clk / (8*2) / 1000;
				break;
			case I2S_SAMPLE_RATE_16_KHZ:
				ws_h = i2s->clk / (16*2) / 1000;
				break;
			case I2S_SAMPLE_RATE_32_KHZ:
				ws_h = i2s->clk / (32*2) / 1000;
				break;
			case I2S_SAMPLE_RATE_48_KHZ:
				ws_h = i2s->clk / (48*2) / 1000;
				break;
			default:
				printk("%s: invalid sample rete for i2s%d.\n", __func__, i2s->index);
				return -EINVAL;
		}
		ws_l = ws_h;
		if((ws_l < div_ws_h) | (ws_l & ~0xFF)){
			printk("%s: current clk not support the sample rate for i2s%d.\n", __func__, i2s->index);
			return -EINVAL;
		}
	}
	else if(X2_I2S_DSP_MODE == i2s->hw_cfg.i2s_dsp_mode){
		div_ws_l = word_len * channel;
		div_ws_h = 0;
		switch(sample_rate){
			case I2S_SAMPLE_RATE_8_KHZ:
				ws_l = i2s->clk / 8 / 1000;
				break;
			case I2S_SAMPLE_RATE_16_KHZ:
				ws_l = i2s->clk / 16 / 1000;
				break;
			case I2S_SAMPLE_RATE_32_KHZ:
				ws_l = i2s->clk / 32 / 1000;
				break;
			case I2S_SAMPLE_RATE_48_KHZ:
				ws_l = i2s->clk / 48 / 1000;
				break;
			default:
				printk("%s: invalid sample rete for i2s%d.\n", __func__, i2s->index);
				return -EINVAL;
		}
		ws_h = 0;
		if((ws_l < div_ws_l) | (ws_l & ~0xFF)){
			printk("%s: current clk not support the sample rate for i2s%d.\n", __func__, i2s->index);
			return -EINVAL;
		}
	}else{
		printk("%s: invalid i2s dsp mode for i2s%d!\n", __func__, i2s->index);
		return -EINVAL;
	}

	x2_i2s_ws_lclk_set(i2s, ws_l);
	x2_i2s_ws_hclk_set(i2s, ws_h);
	return 0;
}

int x2_i2s_start(x2_i2s *i2s)
{
	x2_i2s_channel_enable(i2s);

	x2_i2s_int_unmask(i2s);
	x2_i2s_int_buf_flow_mask(i2s);
	x2_i2s_int_buf_not_ready_mask(i2s);

	x2_i2s_int_state_clear(i2s);

	x2_i2s_iram_enable(i2s, 1);
	x2_i2s_transfer_enable(i2s, 1);
	i2s->status = I2S_STATE_START;

	x2_i2s_ms_mode_set(i2s, i2s->state);
	if(X2_I2S_SLAVE_MODE == i2s->state){
	  x2_i2s_slave_clk_enable(i2s);
	}
	I2S_DEBUG("%s: i2s%d started!\n", __func__, i2s->index);
	return 0;
}

int x2_i2s_stop(x2_i2s *i2s)
{
	x2_i2s_iram_enable(i2s, 0);
	x2_i2s_transfer_enable(i2s, 0);
	i2s->status = I2S_STATE_STOP;
	I2S_DEBUG("%s: i2s%d stopped!\n", __func__, i2s->index);
	return 0;
}

int x2_i2s_pause(x2_i2s *i2s)
{
	x2_i2s_iram_enable(i2s, 0);
	i2s->status = I2S_STATE_PAUSE;
	I2S_DEBUG("%s: i2s%d paused!\n", __func__, i2s->index);
	return 0;
}

int x2_i2s_restart(x2_i2s *i2s)
{
	x2_i2s_iram_enable(i2s, 1);
	i2s->status = I2S_STATE_START;
	I2S_DEBUG("%s: i2s%d started!\n", __func__, i2s->index);
	return 0;
}

static irqreturn_t x2_i2s_handler(int irqno, void *data)
{
	x2_i2s *i2s = data;
	int val = x2_i2s_int_status_get(i2s);
	if(val & BUF0_TRF_DONE){
		x2_i2s_int_buf0_trf_done_set(i2s);
		x2_i2s_buf_swap(i2s, 0);
	}

	if(val & BUF1_TRF_DONE){
		x2_i2s_int_buf1_trf_done_set(i2s);
		x2_i2s_buf_swap(i2s, 1);
	}

	return IRQ_HANDLED;
	if(val & BUF_FLOW){
		I2S_DEBUG("%s: i2s%d buf flow int.\n", __func__, i2s->index);
		x2_i2s_int_buf_flow_set(i2s);
	}

	if(val & BUF_NOT_READY){
		I2S_DEBUG("%s: i2s%d buf not ready int.\n", __func__, i2s->index);
		x2_i2s_int_buf_not_ready_set(i2s);
	}

	return IRQ_HANDLED;
}

static ssize_t show_x2_i2s_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	x2_i2s *i2s;
	unsigned int val;

	char *s = buf;
	i2s = (x2_i2s *)dev_get_drvdata(dev);

	if(0 == i2s->regs){
		return sprintf(buf, "Please initialize the i2s!\n");
	}else{
		val = x2_i2s_read_base_reg(i2s, I2S_CTL);
		s += sprintf(s, "ctl: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_MODE);
		s += sprintf(s, "mode: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_DIV_WS);
		s += sprintf(s, "div_ws: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_CH_EN);
		s += sprintf(s, "ch_en: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_BUF_SIZE);
		s += sprintf(s, "buf_size: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_BUF0_ADDR);
		s += sprintf(s, "buf0_addr: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_BUF0_RDY);
		s += sprintf(s, "buf0_rdy: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_BUF1_ADDR);
		s += sprintf(s, "buf1_addr: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_BUF1_RDY);
		s += sprintf(s, "buf1_rdy: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_BUF_CUR_ADDR);
		s += sprintf(s, "buf_cur_addr: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_CH_ERROR);
		s += sprintf(s, "ch_error: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_SRCPND);
		s += sprintf(s, "srcpnd: 0x%08x \n", val);
		val = x2_i2s_read_base_reg(i2s, I2S_INTMASK);
		s += sprintf(s, "intmask: 0x%08x \n", val);
		if(s != buf)
			*(s-1) = '\n';
		return (s-buf);
	}
}

static ssize_t store_x2_i2s_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	x2_i2s *i2s;
	i2s = dev->driver_data;
	return 0;
}

static DEVICE_ATTR(reg_dump, 0644, show_x2_i2s_reg, store_x2_i2s_reg);

static int x2_i2s_probe(struct platform_device *pdev)
{
	static int i2s_num = 0;
	int ret = 0;
	struct resource *res;
	x2_i2s *i2s;

	i2s = devm_kzalloc(&pdev->dev, sizeof(x2_i2s), GFP_KERNEL);
	if(!i2s){
		dev_err(&pdev->dev, "Failed to alloc i2s structure!\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, i2s);

	i2s->dev = &pdev->dev;
	g_x2_i2s[i2s_num] = i2s;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res){
		dev_err(&pdev->dev, "Failed to get IO resource0!\n");
		return -ENOENT;
	}
	i2s->rx_regs = devm_ioremap_resource(&pdev->dev, res);
	if(IS_ERR(i2s->rx_regs)){
		return PTR_ERR(i2s->rx_regs);
	}

	res = NULL;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if(!res){
		dev_err(&pdev->dev, "Failed to get IO resource1!\n");
		return -ENOENT;
	}
	i2s->tx_regs = devm_ioremap_resource(&pdev->dev, res);
	if(IS_ERR(i2s->tx_regs)){
		return PTR_ERR(i2s->tx_regs);
	}

	i2s_num++;

	i2s->mclk = devm_clk_get(&pdev->dev, "i2s_mclk");
	if(IS_ERR(i2s->mclk)){
		dev_err(&pdev->dev, "failed to get i2s_mclk\n");
		return PTR_ERR(i2s->mclk);
	}
	ret = clk_prepare(i2s->mclk);
	if(ret != 0){
		dev_err(&pdev->dev, "failed to prepare i2s_mclk\n");
		return ret;
	}

	i2s->bclk = devm_clk_get(&pdev->dev, "i2s_bclk");
	if(IS_ERR(i2s->bclk)){
		dev_err(&pdev->dev, "failed to get i2s_bclk\n");
		return PTR_ERR(i2s->bclk);
	}
	ret = clk_prepare(i2s->bclk);
	if(ret != 0){
		dev_err(&pdev->dev, "failed to prepare i2s_bclk\n");
		return ret;
	}

	i2s->rst = devm_reset_control_get(&pdev->dev, "i2s");
	if(IS_ERR(i2s->rst)) {
		dev_err(&pdev->dev, "Missing reset controller!\n");
		return PTR_ERR(i2s->rst);
	}

	ret = platform_get_irq(pdev, 0);
	if(ret <= 0){
		dev_err(&pdev->dev, "Failed to get IRQ resource!\n");
		return ret;
	}

	i2s->irq = ret;
	ret = devm_request_irq(&pdev->dev, i2s->irq, x2_i2s_handler,
			0, dev_name(&pdev->dev), i2s);
	if(ret != 0){
		dev_err(&pdev->dev, "Failed to claim IRQ %d!\n", i2s->irq);
		return ret;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_reg_dump);
	if(!ret)
		return ret;

	dev_info(&pdev->dev, "%s: x2 i2s probe success!\n", dev_name(i2s->dev));
	return 0;
}

static const struct of_device_id x2_i2s_of_match[] = {
	{.compatible = "hobot,x2-i2s"},
	{},
};

static struct platform_driver x2_i2s_driver = {
	.probe = x2_i2s_probe,
	.driver = {
		.name = "x2-i2s",
		.of_match_table = of_match_ptr(x2_i2s_of_match),
	},
};

module_platform_driver(x2_i2s_driver);

MODULE_AUTHOR("Hobot");
MODULE_DESCRIPTION("X2 I2S SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:x2-i2s");
