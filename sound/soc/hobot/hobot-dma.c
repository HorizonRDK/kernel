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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/irqreturn.h>
#include <linux/of.h>
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
#ifdef CONFIG_HOBOT_DIAG
#include <soc/hobot/diag.h>
#endif

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "hobot-i2s.h"

#define ST_RUNNING      (1<<0)
#define ST_OPENED       (1<<1)

#define DMA_START   1
#define DMA_STOP    0

#define I2S_PROCESS_NUM 5

#define MAX_IDMA_PERIOD (80 * 1024)
#define MAX_IDMA_BUFFER (160 * 1024)

static unsigned char *dma_vm;
struct process_info {
	struct snd_pcm_substream *substream;
};
struct hobot_pcm_param {
	uint32_t samplefmt;
	uint32_t samplerate;
	uint32_t channels;
	uint32_t buffer_size;
};
struct global_info {
	spinlock_t lock;
	struct snd_dma_buffer *g_dma_buf;

	uint8_t capture_process_num;
	struct process_info process_info[I2S_PROCESS_NUM];
	struct hobot_pcm_param pcm_param;

	uint8_t trigger_flag;
} global_info[2];
struct idma_ctrl_s *g_dma_ctrl[2];

/* play and capture have the same hardware param, so no neeed split two parts */
static const struct snd_pcm_hardware i2sidma_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
	    SNDRV_PCM_INFO_NONINTERLEAVED |
	    SNDRV_PCM_INFO_BLOCK_TRANSFER |
	    SNDRV_PCM_INFO_MMAP |
	    SNDRV_PCM_INFO_MMAP_VALID |
	    SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.buffer_bytes_max = MAX_IDMA_BUFFER,
	.period_bytes_min = 128,
	.period_bytes_max = MAX_IDMA_PERIOD,
	.periods_min = 1,
	.periods_max = 4,
};

static const struct snd_pcm_hardware pdma_hardware = {
        .info = SNDRV_PCM_INFO_INTERLEAVED |
            SNDRV_PCM_INFO_NONINTERLEAVED |
            SNDRV_PCM_INFO_BLOCK_TRANSFER |
            SNDRV_PCM_INFO_MMAP |
            SNDRV_PCM_INFO_MMAP_VALID |
            SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
        .buffer_bytes_max = MAX_IDMA_BUFFER,
        .period_bytes_min = 128,
        .period_bytes_max = MAX_IDMA_PERIOD,
        .periods_min = 1,
        .periods_max = 4,
};

struct idma_ctrl_s {
	spinlock_t lock;
	int state;
	dma_addr_t start;
	dma_addr_t pos;
	dma_addr_t end;
	dma_addr_t lastset;
	dma_addr_t current_addr;
	dma_addr_t period;
	dma_addr_t periodsz;
	size_t bytesnum;
	void *token;
	void (*cb)(void *dt, int bytes_xfer);
	int stream;		/* capture or play */
	int ch_num;		/* paly or capture channel */
	int word_len;
	int id;
	int buffer_num;
	int buffer_int_index;
	int buffer_set_index;
	char *tmp_buf;
	uint32_t idma_count;
	struct timespec tstamp[4];
};

static struct idma_info_s {
	spinlock_t lock;
	void __iomem *regaddr_tx;
	void __iomem *regaddr_rx;

	int idma_irq;

	struct device *dev;
	int stream;
	int id;
} hobot_i2sidma[10];

static int tstamp_mode=0;

static int tstamp_mode_set(const char *val, const struct kernel_param *kp) {
	int ret;
	ret = param_set_bool(val, kp);
	if (ret < 0)
		return ret;

	return ret;
}

static const struct kernel_param_ops tstamp_mode_param_ops = {
	.set = tstamp_mode_set,
	.get = param_get_bool,
};
module_param_cb(tstamp_mode, &tstamp_mode_param_ops, &tstamp_mode, 0644);
MODULE_PARM_DESC(tstamp_mode, "enable/disable tstamp printk");

static int copy_usr_interleaved(struct snd_pcm_substream *substream,
		int channel, unsigned long hwoff,
		void *buf, unsigned long bytes) {
	int i, j, channel_buf_offset;
	char *dma_ptr;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;

	channel_buf_offset = bytes / dma_ctrl->ch_num;
	dma_ptr = runtime->dma_area + hwoff;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (copy_from_user(dma_ctrl->tmp_buf, (void __user *)buf, bytes))
			return -EFAULT;
		for (i = 0; i < bytes;) {
			for (j = 0; j < dma_ctrl->ch_num; j++) {
				memcpy(&dma_ptr[j*channel_buf_offset],
					&dma_ctrl->tmp_buf[i+j*dma_ctrl->word_len],
					dma_ctrl->word_len);
			}
			i = i + dma_ctrl->ch_num * dma_ctrl->word_len;
			dma_ptr = dma_ptr + dma_ctrl->word_len;
		}
		dma_sync_single_for_device(hobot_i2sidma[dma_ctrl->id].dev,
			runtime->dma_addr+hwoff,
            bytes, DMA_TO_DEVICE);
	} else {
		dma_sync_single_for_cpu(hobot_i2sidma[dma_ctrl->id].dev,
				runtime->dma_addr + hwoff,
				bytes, DMA_FROM_DEVICE);
		for (i = 0; i < bytes;) {
			for (j = 0; j < dma_ctrl->ch_num; j++) {
				memcpy(&dma_ctrl->tmp_buf[i+j*dma_ctrl->word_len],
					&dma_ptr[j*channel_buf_offset],
					dma_ctrl->word_len);
			}
			dma_ptr = dma_ptr + dma_ctrl->word_len;
			i = i + dma_ctrl->ch_num * dma_ctrl->word_len;
		}
		if (copy_to_user((void __user *)buf, dma_ctrl->tmp_buf, bytes))
					return -EFAULT;
	}

	return 0;
}

static int copy_usr_noninterleaved(struct snd_pcm_substream *substream,
		int channel, unsigned long hwoff,
		void *buf, unsigned long bytes) {
	char *dma_ptr;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;

	dma_ptr = runtime->dma_area + hwoff;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (copy_from_user(dma_ptr, (void __user *)buf, bytes))
			return -EFAULT;
		dma_sync_single_for_device(hobot_i2sidma[dma_ctrl->id].dev,
			runtime->dma_addr + hwoff,
			bytes, DMA_TO_DEVICE);
	} else {
		dma_sync_single_for_cpu(hobot_i2sidma[dma_ctrl->id].dev,
			runtime->dma_addr + hwoff,
			bytes, DMA_FROM_DEVICE);
		if (copy_to_user((void __user *)buf, dma_ptr, bytes))
			return -EFAULT;
	}

	return 0;
}

static int hobot_copy_usr(struct snd_pcm_substream *substream,
		int channel, unsigned long hwoff,
		void *buf, unsigned long bytes)
{
	char *dma_ptr;
	int channel_buf_offset, i, j, k;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;
	//char *tmp_buf;
	uint8_t count;
	uint8_t bytes_count;
	unsigned long period_bytes;
	uint8_t period_count;
	int8_t ret;

	channel_buf_offset = (dma_ctrl->periodsz) / (dma_ctrl->ch_num);

	if (runtime->access == SNDRV_PCM_ACCESS_RW_INTERLEAVED) {
		period_bytes = dma_ctrl->periodsz;
		count = hwoff / (runtime->dma_bytes / runtime->periods);
	} else {
		period_bytes = dma_ctrl->periodsz / dma_ctrl->ch_num;
	}
	period_count = bytes / period_bytes;

	for (i = 0; i < period_count; i++) {
		if (runtime->access == SNDRV_PCM_ACCESS_RW_INTERLEAVED) {
			if (tstamp_mode == 1) {
				if (copy_to_user((void __user *)buf + (count+i)*sizeof(struct timespec)
						+ i*dma_ctrl->periodsz,
						&dma_ctrl->tstamp[count+i], sizeof(struct timespec)))
						return -EFAULT;
				ret = copy_usr_interleaved(substream, channel, hwoff+i*period_bytes,
					buf+(count+i+1)*sizeof(struct timespec)+i*period_bytes,
					period_bytes);
				if (ret < 0)
					break;
			} else {
				ret = copy_usr_interleaved(substream, channel, hwoff+i*period_bytes,
					buf+i*period_bytes,
					period_bytes);
				if (ret < 0)
					break;
			}
		} else {
			ret = copy_usr_noninterleaved(substream, channel,
				hwoff * dma_ctrl->ch_num + channel * period_bytes +
				k * dma_ctrl->periodsz,
				buf, period_bytes);
			if (ret < 0)
				break;
		}
	}

	return 0;
}

static struct snd_dmaengine_dai_dma_data *hobot_dai_get_dma_data(struct
							      snd_pcm_substream
							      *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;

	return snd_soc_dai_get_dma_data(soc_runtime->cpu_dai, substream);
}

static void idma_getpos(dma_addr_t *src, int stream,
		struct snd_soc_card *snd_card, struct idma_ctrl_s *dma_ctrl)
{
	*src = dma_ctrl->lastset;
}

/* set dma buffer addr and size to register */
static int i2sidma_enqueue(struct snd_pcm_substream *substream)
{
	/* struct snd_pcm_runtime *runtime = substream->runtime; */
	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&dma_ctrl->lock, flags);
	dma_ctrl->token = (void *)substream;
	dev_dbg(hobot_i2sidma[dma_ctrl->id].dev,
		"addr(dma_ctrl->token)=%p\n", dma_ctrl->token);
	spin_unlock_irqrestore(&dma_ctrl->lock, flags);

	if (global_info[dma_ctrl->id].trigger_flag == 1 &&
			substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return 0;

	/* set buf0 ready */
	val = dma_ctrl->start;
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		writel(val, hobot_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_BUF0_ADDR);
		writel(val + dma_ctrl->periodsz,
			hobot_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_BUF1_ADDR);
		dma_ctrl->buffer_int_index = 0;
		dma_ctrl->buffer_set_index = 2;
		val = (dma_ctrl->periodsz) / (dma_ctrl->ch_num);
		writel(val, hobot_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_BUF_SIZE);


	} else {

		writel(val, hobot_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_BUF0_ADDR);
		writel(val + dma_ctrl->periodsz,
			hobot_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_BUF1_ADDR);
		dma_ctrl->buffer_int_index = 0;
		dma_ctrl->buffer_set_index = 0;
		val = (dma_ctrl->periodsz) / (dma_ctrl->ch_num);
		writel(val, hobot_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_BUF_SIZE);

	}

	return 0;
}

static void i2sidma_setcallbk(struct snd_pcm_substream *substream,
			       void (*cb)(void *, int))
{
	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;
	unsigned long flags;

	spin_lock_irqsave(&dma_ctrl->lock, flags);
	dma_ctrl->cb = cb;
	spin_unlock_irqrestore(&dma_ctrl->lock, flags);
}

static void i2sidma_control(int op, int stream, struct idma_ctrl_s *dma_ctrl)
{
	u32 val = 0;

		dev_dbg(hobot_i2sidma[dma_ctrl->id].dev,
			"%s:%d  trigger_flag=%d\n", __func__, __LINE__,
			global_info[dma_ctrl->id].trigger_flag);
		if (global_info[dma_ctrl->id].trigger_flag == 1 &&
				stream == SNDRV_PCM_STREAM_CAPTURE)
			return;

		if (stream == SNDRV_PCM_STREAM_CAPTURE)
			val = readl(hobot_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_CTL);
		else
			val = readl(hobot_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_CTL);

		switch (op) {
		case DMA_START:
			val |= CTL_IMEM_EN;
			val |= CTL_IMEM_CLR;
			break;
		case DMA_STOP:
			val &= ~CTL_IMEM_EN;
			break;
		default:
			return;
		}

		if (stream == SNDRV_PCM_STREAM_CAPTURE)
			writel(val, hobot_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_CTL);
		else
			writel(val, hobot_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_CTL);



}

static void i2sidma_done(void *id, int bytes_xfer)
{
	if (!id)
		return;
	struct snd_pcm_substream *substream = id;
	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;
	if (dma_ctrl && (dma_ctrl->state & ST_RUNNING))
		snd_pcm_period_elapsed(substream);
}

/* upate runtime dma data */
static int i2sidma_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;

	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct idma_info_s *hobot_dma =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	if (hobot_dma == NULL) {
		return -ENOMEM;
	}

	/* update runtime dma info from substream dma info */
	/* substream dma info if getted at idma_open */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		snd_pcm_set_runtime_buffer(substream, global_info[dma_ctrl->id].g_dma_buf);
	else
		snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	runtime->dma_bytes = params_buffer_bytes(params);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (global_info[dma_ctrl->id].trigger_flag == 0) {
			global_info[dma_ctrl->id].pcm_param.samplefmt = params_format(params);
			global_info[dma_ctrl->id].pcm_param.samplerate = params_rate(params);
			global_info[dma_ctrl->id].pcm_param.channels = params_channels(params);
			global_info[dma_ctrl->id].pcm_param.buffer_size = params_buffer_bytes(params);
		} else {
			if (global_info[dma_ctrl->id].pcm_param.samplefmt != params_format(params) ||
				global_info[dma_ctrl->id].pcm_param.samplerate != params_rate(params) ||
				global_info[dma_ctrl->id].pcm_param.channels != params_channels(params) ||
				global_info[dma_ctrl->id].pcm_param.buffer_size != params_buffer_bytes(params)) {
				dev_err(hobot_dma->dev,
					"pcm format is not match with different process\n");
				return -EINVAL;
			}
		}
	}

	/* init dma buffer addr */
	dma_ctrl->start = dma_ctrl->pos = runtime->dma_addr;
	dma_ctrl->period = params_periods(params);
	dma_ctrl->periodsz = params_period_bytes(params);//per frame bytes
	dma_ctrl->end = runtime->dma_addr + runtime->dma_bytes;
	dma_ctrl->lastset = dma_ctrl->start;
	dma_ctrl->ch_num = params_channels(params);
	dma_ctrl->bytesnum = runtime->dma_bytes;//total bytes
	dma_ctrl->buffer_num = dma_ctrl->bytesnum / dma_ctrl->periodsz;

	switch (params_format(params)) { /* 16bit or 8bit. */
	case SNDRV_PCM_FORMAT_S16_LE:
		dma_ctrl->word_len = 2;
		break;
	case SNDRV_PCM_FORMAT_S8:
		dma_ctrl->word_len  = 1;
		break;
	default:
		dev_err(hobot_dma->dev,
			"not supported data format %d\n",
			params_format(params));
		return -EINVAL;
	}

	dev_dbg(hobot_dma->dev,
		"dma_ctrl->period is %llu, dma_ctrl->periodsz bytes is %llu,dma_ctrl->bytesnum is %lu\n", dma_ctrl->period,
		dma_ctrl->periodsz, dma_ctrl->bytesnum);
	dev_dbg(hobot_dma->dev,
		"dma_ctrl->start is 0x%llx,dma_ctrl->end is 0x%llx\n",
		dma_ctrl->start, dma_ctrl->end);
	dev_dbg(hobot_dma->dev,
		"dma_ctrl->buffer_num is %d\n", dma_ctrl->buffer_num);

	/* set dma cb */
	i2sidma_setcallbk(substream, i2sidma_done);

	return 0;
}

/* match hw param */
static int i2sidma_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

/* enable irq */
/* mask overflow irq */
/* mask notready irq */
/* clear all irq */
static int i2sidma_prepare(struct snd_pcm_substream *substream)
{
	unsigned int reg_val;

	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;

	dma_ctrl->pos = dma_ctrl->start;
	/* disable iram */
	i2sidma_control(DMA_STOP, substream->stream, dma_ctrl);
	/* setting bufer base/size/reday register */
	i2sidma_enqueue(substream);

	if (global_info[dma_ctrl->id].trigger_flag == 1 &&
			substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		writel(0xf, hobot_i2sidma[dma_ctrl->id].regaddr_rx + I2S_UNMASK);
		reg_val = readl(hobot_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_SETMASK);
		writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF_FLOW_BIT,
			I2S_INT_BUF_FLOW_FIELD),
			hobot_i2sidma[dma_ctrl->id].regaddr_rx + I2S_SETMASK);


		reg_val = readl(hobot_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_SETMASK);
		writel(UPDATE_VALUE_FIELD
			(reg_val, 0x1, I2S_INT_BUF_NOT_READY_BIT,
				I2S_INT_BUF_NOT_READY_FIELD),
				hobot_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_SETMASK);

		writel(0xF, hobot_i2sidma[dma_ctrl->id].regaddr_rx + I2S_SRCPND);


	} else {/* play */
		writel(0xf, hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_UNMASK);
		reg_val = readl(hobot_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_SETMASK);
		writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF_FLOW_BIT,
			I2S_INT_BUF_FLOW_FIELD),
			hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_SETMASK);


		reg_val = readl(hobot_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_SETMASK);
		writel(UPDATE_VALUE_FIELD
				(reg_val, 0x1, I2S_INT_BUF_NOT_READY_BIT,
				I2S_INT_BUF_NOT_READY_FIELD),
				hobot_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_SETMASK);

		writel(0xF, hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_SRCPND);

	}

	return 0;
}

static int i2sidma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;
	int ret = 0;
	unsigned long flags;

	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct idma_info_s *hobot_dma =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	if (hobot_dma == NULL) {
		return -ENOMEM;
	}

	spin_lock_irqsave(&dma_ctrl->lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dma_ctrl->state |= ST_RUNNING;
		/* enable iram, clear iram */
		i2sidma_control(DMA_START, substream->stream, dma_ctrl);

		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			global_info[dma_ctrl->id].trigger_flag = 1;
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dma_ctrl->state &= ~ST_RUNNING;
		if (global_info[dma_ctrl->id].capture_process_num == 1)
			global_info[dma_ctrl->id].trigger_flag = 0;
		i2sidma_control(DMA_STOP, substream->stream, dma_ctrl);
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
				global_info[dma_ctrl->id].capture_process_num == 1) {
			writel(0x0, hobot_i2sidma[dma_ctrl->id].regaddr_rx + I2S_UNMASK);
		} else {
			writel(0x0, hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_UNMASK);
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}
	spin_unlock_irqrestore(&dma_ctrl->lock, flags);

	dev_info(hobot_dma->dev, "i2sidma_trigger\n");
	return ret;
}

static snd_pcm_uframes_t i2sidma_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl_s *dma_ctrl = runtime->private_data;
	dma_addr_t src;
	unsigned long res;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *snd_card = rtd->card;
	unsigned long flags;

	spin_lock_irqsave(&dma_ctrl->lock, flags);

	idma_getpos(&src, substream->stream, snd_card, dma_ctrl);
	/* pr_err("i2sidma_pointer, get pos is %llx\n", src); */
	res = src - dma_ctrl->start;

	if (tstamp_mode == 1 && substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		uint8_t count;
		struct timespec curr_tstamp;
		snd_pcm_uframes_t hw_ptr;
		hw_ptr = runtime->status->hw_ptr / runtime->period_size;
		count = hw_ptr % runtime->periods;
		snd_pcm_gettime(runtime, (struct timespec *)&curr_tstamp);
		dma_ctrl->tstamp[count] = curr_tstamp;
	}
	spin_unlock_irqrestore(&dma_ctrl->lock, flags);

	return bytes_to_frames(substream->runtime, res);
}


static int i2sidma_mmap(struct snd_pcm_substream *substream,
			 struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long size, offset;
	int ret;
	/* From snd_pcm_lib_mmap_iomem */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	size = vma->vm_end - vma->vm_start;
	offset = vma->vm_pgoff << PAGE_SHIFT;
	ret = io_remap_pfn_range(vma, vma->vm_start,
				 (runtime->dma_addr + offset) >> PAGE_SHIFT,
				 size, vma->vm_page_prot);

	return ret;
}

#ifdef CONFIG_HOBOT_DIAG
static void iis_diag_report(uint8_t errsta, uint32_t status, uint8_t channel)
{
	uint32_t sta;
	unsigned char eventid;
	u8 envdata[5]; // channel num + srcpnd reg value.
	static uint8_t last_status = DiagEventStaUnknown;

	if (channel > 1)
		return;

	if (channel)
		eventid = EventIdSoundI2s1Err;
	else
		eventid = EventIdSoundI2s0Err;

	sta = status;

	if (errsta) {
		envdata[0] = channel;
		memcpy(envdata + 1, (uint8_t *)&sta, sizeof(uint32_t));
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_sound,
				eventid,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				envdata,
				sizeof(uint32_t) + 1);
	} else if (last_status != DiagEventStaSuccess) {
		diag_send_event_stat(
				DiagMsgPrioHigh,
				ModuleDiag_sound,
				eventid,
				DiagEventStaSuccess);
	}
	last_status = !errsta ? DiagEventStaSuccess : DiagEventStaFail;
}
#endif

static irqreturn_t iis_irq0(int irqno, void *dev_id)
{
	struct idma_ctrl_s *dma_ctrl = (struct idma_ctrl_s *)dev_id;
	u32 intstatus;
	u32	addr = 0;
	uint8_t errsta = 0;
	uint8_t i = 0;

	intstatus = readl(hobot_i2sidma[dma_ctrl->id].regaddr_rx + I2S_SRCPND);

	if (intstatus == 0x4) {

		writel(0x4, hobot_i2sidma[dma_ctrl->id].regaddr_rx + I2S_SRCPND);
		dev_dbg(hobot_i2sidma[dma_ctrl->id].dev,
			"intstatus = 0x4,dma_ctrl->lastset is 0x%llx,buffer_int_index is %d,buffer_set_index is %d\n",
			dma_ctrl->lastset, dma_ctrl->buffer_int_index,
			dma_ctrl->buffer_set_index);

		addr = dma_ctrl->start +
			(dma_ctrl->buffer_set_index * dma_ctrl->periodsz);
		dma_ctrl->buffer_set_index += 1;
		if (dma_ctrl->buffer_set_index >= dma_ctrl->buffer_num)
			dma_ctrl->buffer_set_index = 0;


		dma_ctrl->buffer_int_index += 1;
		if (dma_ctrl->buffer_int_index == dma_ctrl->buffer_num)
			dma_ctrl->buffer_int_index = 0;


		dma_ctrl->lastset = dma_ctrl->start +
		(dma_ctrl->buffer_int_index * dma_ctrl->periodsz);
		writel(addr, hobot_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_BUF0_ADDR);
		writel(0x1, hobot_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_BUF0_RDY);

	} else if (intstatus == 0x8) {

		writel(0x8, hobot_i2sidma[dma_ctrl->id].regaddr_rx + I2S_SRCPND);
		dev_dbg(hobot_i2sidma[dma_ctrl->id].dev,
			"intstatus = 0x8,dma_ctrl->lastset is 0x%llx,buffer_int_index is %d,buffer_set_index is %d\n",
			dma_ctrl->lastset, dma_ctrl->buffer_int_index,
			dma_ctrl->buffer_set_index);

		addr = dma_ctrl->start +
			(dma_ctrl->buffer_set_index * dma_ctrl->periodsz);
		dma_ctrl->buffer_set_index += 1;
		if (dma_ctrl->buffer_set_index >= dma_ctrl->buffer_num)
			dma_ctrl->buffer_set_index = 0;


		dma_ctrl->buffer_int_index += 1;
		if (dma_ctrl->buffer_int_index == dma_ctrl->buffer_num)
			dma_ctrl->buffer_int_index = 0;


		dma_ctrl->lastset = dma_ctrl->start +
		(dma_ctrl->buffer_int_index * dma_ctrl->periodsz);
		writel(addr, hobot_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_BUF1_ADDR);
		writel(0x1, hobot_i2sidma[dma_ctrl->id].regaddr_rx + I2S_BUF1_RDY);

	} else {
		dev_err(hobot_i2sidma[dma_ctrl->id].dev,
			"intstatus = 0x%x,INT status exception!\n", intstatus);
		errsta = 1;
		writel(intstatus,
			hobot_i2sidma[dma_ctrl->id].regaddr_rx +I2S_SRCPND);
		writel(dma_ctrl->start,
			hobot_i2sidma[dma_ctrl->id].regaddr_rx +I2S_BUF0_ADDR);
		writel(dma_ctrl->start + dma_ctrl->periodsz,
			hobot_i2sidma[dma_ctrl->id].regaddr_rx +I2S_BUF1_ADDR);

		dma_ctrl->buffer_int_index = 0;
		dma_ctrl->buffer_set_index = 2;

		writel(0x1, hobot_i2sidma[dma_ctrl->id].regaddr_rx +I2S_BUF0_RDY);
		writel(0x1, hobot_i2sidma[dma_ctrl->id].regaddr_rx + I2S_BUF1_RDY);

		if (dma_ctrl->cb)
			dma_ctrl->cb(dma_ctrl->token, dma_ctrl->period);
		goto err;
	}

	/* daemon callback */
	if (dma_ctrl->cb)
		dma_ctrl->cb(dma_ctrl->token, dma_ctrl->period);
	for (i = 1; i < I2S_PROCESS_NUM; i++) {
		if (!global_info[dma_ctrl->id].process_info[i].substream)
			continue;
		struct snd_pcm_substream *substream =
			global_info[dma_ctrl->id].process_info[i].substream;
		struct idma_ctrl_s *dma_ctrl_seed = substream->runtime->private_data;
		if (!dma_ctrl_seed || !dma_ctrl_seed->token) {
			continue;
		}
		dma_ctrl_seed->start = dma_ctrl->start;
		dma_ctrl_seed->lastset = dma_ctrl->lastset;
		dma_ctrl_seed->cb((void *)dma_ctrl_seed->token, dma_ctrl_seed->period);
	}

err:
#ifdef CONFIG_HOBOT_DIAG
	iis_diag_report(errsta, intstatus, 0);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t iis_irq1(int irqno, void *dev_id)
{
	struct idma_ctrl_s *dma_ctrl = (struct idma_ctrl_s *)dev_id;
	u32 intstatus;
	u32	addr = 0;
	uint8_t errsta = 0;

	intstatus = readl(hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_SRCPND);

	if (intstatus == 0x4) {
		writel(0x4, hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_SRCPND);
		addr = dma_ctrl->start +
			(dma_ctrl->buffer_set_index * dma_ctrl->periodsz);
		dma_ctrl->buffer_set_index += 1;
		if (dma_ctrl->buffer_set_index == dma_ctrl->buffer_num)
			dma_ctrl->buffer_set_index = 0;

		dma_ctrl->buffer_int_index += 1;
		if (dma_ctrl->buffer_int_index == dma_ctrl->buffer_num)
			dma_ctrl->buffer_int_index = 0;


		dma_ctrl->lastset = dma_ctrl->start +
		(dma_ctrl->buffer_int_index * dma_ctrl->periodsz);
		writel(addr, hobot_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_BUF0_ADDR);
		writel(0x1, hobot_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_BUF0_RDY);

		dev_dbg(hobot_i2sidma[dma_ctrl->id].dev,
			"intstatus = 0x%x, dma_ctrl->lastset is 0x%llx, \
			buffer_int_index is %d, buffer_set_index is %d\n",
			intstatus, dma_ctrl->lastset, dma_ctrl->buffer_int_index,
			dma_ctrl->buffer_set_index);

	} else if (intstatus == 0x8) {
		writel(0x8, hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_SRCPND);
		addr = dma_ctrl->start +
			(dma_ctrl->buffer_set_index * dma_ctrl->periodsz);
		dma_ctrl->buffer_set_index += 1;
		if (dma_ctrl->buffer_set_index == dma_ctrl->buffer_num)
			dma_ctrl->buffer_set_index = 0;

		dma_ctrl->buffer_int_index += 1;
		if (dma_ctrl->buffer_int_index == dma_ctrl->buffer_num)
			dma_ctrl->buffer_int_index = 0;


		dma_ctrl->lastset = dma_ctrl->start +
		(dma_ctrl->buffer_int_index * dma_ctrl->periodsz);
		writel(addr, hobot_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_BUF1_ADDR);
		writel(0x1, hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_BUF1_RDY);

		dev_dbg(hobot_i2sidma[dma_ctrl->id].dev,
			"intstatus = 0x%x, dma_ctrl->lastset is 0x%llx, \
			buffer_int_index is %d, buffer_set_index is %d\n",
			intstatus, dma_ctrl->lastset, dma_ctrl->buffer_int_index,
			dma_ctrl->buffer_set_index);

	} else {
		dev_err(hobot_i2sidma[dma_ctrl->id].dev,
			"intstatus = 0x%x,INT status exception!\n", intstatus);
		errsta = 1;

		writel(intstatus, hobot_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_SRCPND);

		writel(dma_ctrl->start,
			hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_BUF0_ADDR);
		writel(dma_ctrl->start + dma_ctrl->periodsz,
			hobot_i2sidma[dma_ctrl->id].regaddr_tx + I2S_BUF1_ADDR);
		dma_ctrl->buffer_int_index = 0;
		dma_ctrl->buffer_set_index = 0;

		writel(0x1, hobot_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_BUF0_RDY);
		writel(0x1, hobot_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_BUF1_RDY);

		if (dma_ctrl->cb)
			dma_ctrl->cb(dma_ctrl->token, dma_ctrl->period);
		goto err;
	}

	if (dma_ctrl->cb)
		dma_ctrl->cb(dma_ctrl->token, dma_ctrl->period);

err:
#ifdef CONFIG_HOBOT_DIAG
	iis_diag_report(errsta, intstatus, 0);
#endif
	return IRQ_HANDLED;
}

/* get dam irq, set runtime hw limit */
static int i2sidma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	struct idma_ctrl_s *dma_ctrl;
	struct snd_dmaengine_dai_dma_data *dma_data;
	unsigned long flags;
	int ret;
	uint8_t i;
	uint8_t id_index;

	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct hobot_i2s *i2s = snd_soc_dai_get_drvdata(soc_runtime->cpu_dai);
	struct idma_info_s *hobot_dma =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	if (hobot_dma == NULL) {
		return -ENOMEM;
	}

	dma_data = hobot_dai_get_dma_data(substream);
	dma_ctrl = kzalloc(sizeof(struct idma_ctrl_s), GFP_KERNEL);
	if (dma_ctrl == NULL) {
		return -ENOMEM;
	}

	if (hobot_dma)
		dma_ctrl->id = hobot_dma->id;
	dev_dbg(hobot_dma->dev,
		"%s:%d dma_ctrl->id = %d\n", __func__, __LINE__, dma_ctrl->id);

	spin_lock_irqsave(&global_info[hobot_dma->id].lock, flags);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		for (i = 0; i < I2S_PROCESS_NUM; i++) {
			if (!global_info[hobot_dma->id].process_info[i].substream)
				break;
		}
		if (i == I2S_PROCESS_NUM)
			return -EPERM;

		global_info[hobot_dma->id].process_info[i].substream = substream;
		dev_dbg(hobot_dma->dev,
			"%s substream[%d]=%p\n", __func__,
			i, substream);
		global_info[hobot_dma->id].capture_process_num++;
	}

	/* dma_ctrl->dma_id = dma_data->slave_id; */
	/* set limit hw param to runtime */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		snd_soc_set_runtime_hwparams(substream, &i2sidma_hardware);
	else
		snd_soc_set_runtime_hwparams(substream, &pdma_hardware);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (i2s) {
			ret = request_irq(hobot_i2sidma[dma_ctrl->id].idma_irq, iis_irq0,
				0, "idma0", dma_ctrl);
			if (ret < 0) {
				dev_err(hobot_dma->dev,
					"fail to claim i2s irq , ret = %d\n", ret);
				kfree(dma_ctrl);
				spin_unlock_irqrestore(&global_info[hobot_dma->id].lock, flags);
				return ret;
			}
			g_dma_ctrl[dma_ctrl->id] = dma_ctrl;
		}
	} else {
		ret = request_irq(hobot_i2sidma[dma_ctrl->id].idma_irq, iis_irq1,
			0, "idma1", dma_ctrl);
		if (ret < 0) {
			dev_err(hobot_dma->dev,
				"fail to claim i2s irq , ret = %d\n", ret);
			kfree(dma_ctrl);
			spin_unlock_irqrestore(&global_info[hobot_dma->id].lock, flags);
			return ret;
		}
	}

	dma_ctrl->stream = substream->stream;
	hobot_i2sidma[dma_ctrl->id].stream = substream->stream;

	spin_lock_init(&dma_ctrl->lock);

	dma_ctrl->tmp_buf = kzalloc(i2sidma_hardware.buffer_bytes_max, GFP_KERNEL);
	if (!dma_ctrl->tmp_buf) {
		spin_unlock_irqrestore(&global_info[hobot_dma->id].lock, flags);
		return -ENOMEM;
	}

	/* this critical action */
	runtime->private_data = dma_ctrl;
	spin_unlock_irqrestore(&global_info[hobot_dma->id].lock, flags);

	return 0;
}

static int i2sidma_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl_s *dma_ctrl = runtime->private_data;
	unsigned long flags;
	uint8_t i;

	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct idma_info_s *hobot_dma =
		snd_soc_platform_get_drvdata(soc_runtime->platform);

	if (dma_ctrl == g_dma_ctrl[dma_ctrl->id]
			&& substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		free_irq(hobot_dma->idma_irq, dma_ctrl);
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		free_irq(hobot_dma->idma_irq, dma_ctrl);
	}

	spin_lock_irqsave(&global_info[hobot_dma->id].lock, flags);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		for (i = 1; i < I2S_PROCESS_NUM; i++) {
			if (substream ==
					global_info[dma_ctrl->id].process_info[i].substream) {
				dev_dbg(hobot_dma->dev,
					"%s substream[%d]=%p\n",
					__func__, i, substream);
				global_info[dma_ctrl->id].process_info[i].substream = NULL;
				break;
			}
		}

		global_info[dma_ctrl->id].capture_process_num--;
		if (global_info[dma_ctrl->id].capture_process_num == 0) {
			for (i = 0; i < I2S_PROCESS_NUM; i++) {
				global_info[dma_ctrl->id].process_info[i].substream = NULL;
			}
			memset(&global_info[dma_ctrl->id].pcm_param, 0, sizeof(struct hobot_pcm_param));
		}
	}

	kfree(dma_ctrl->tmp_buf);
	kfree(dma_ctrl);

	spin_unlock_irqrestore(&global_info[hobot_dma->id].lock, flags);

	return 0;
}

static struct snd_pcm_ops i2sidma_ops = {
	.open = i2sidma_open,
	.close = i2sidma_close,
	.ioctl = snd_pcm_lib_ioctl,
	.trigger = i2sidma_trigger,
	.pointer = i2sidma_pointer,
	.mmap = i2sidma_mmap,
	.hw_params = i2sidma_hw_params,
	.hw_free = i2sidma_hw_free,
	.prepare = i2sidma_prepare,
	.copy_user = hobot_copy_usr,
};

/* free capture or playback dma buffer(ioummap) */
static void i2sidma_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		/*dmam_free_coherent(pcm->card->dev, buf->bytes, buf->area,
				   buf->addr);*/
		dma_unmap_single(pcm->card->dev, buf->addr, i2sidma_hardware.buffer_bytes_max,
			DMA_BIDIRECTIONAL);

		buf->area = NULL;
		buf->addr = 0;
	}

}

/* update dma info to substream->dma_buffer by hw design */
/* the ioremap equivalently alloc buffer */
static int preallocate_idma_buffer(struct idma_info_s *dma_info,
		struct snd_pcm *pcm, int stream)
{
	struct snd_dmaengine_dai_dma_data *dma_data;
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	dma_addr_t phy;

	/* now not use dma data */
	dma_data = hobot_dai_get_dma_data(substream);
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	/* Assign PCM buffer pointers */
	buf->dev.type = SNDRV_DMA_TYPE_CONTINUOUS;



	buf->area = kzalloc(i2sidma_hardware.buffer_bytes_max, GFP_DMA | GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	phy = dma_map_single(pcm->card->dev, buf->area,
		i2sidma_hardware.buffer_bytes_max, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(pcm->card->dev, phy)) {
		dev_err(pcm->card->dev, "dma %zu bytes error\n",
				i2sidma_hardware.buffer_bytes_max);
		return -ENOMEM;
	}
	dma_vm = buf->area;
	buf->addr = phy;
	buf->bytes = i2sidma_hardware.buffer_bytes_max;  /* 160K */

	if (buf->area == NULL)
		return -ENOMEM;

	if (dma_info && stream == SNDRV_PCM_STREAM_CAPTURE)
		global_info[dma_info->id].g_dma_buf = buf;
	return 0;
}

static int i2sidma_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	struct idma_info_s *dma_info =
		(struct idma_info_s *)dev_get_drvdata(rtd->platform->dev);
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream)
		ret = preallocate_idma_buffer(dma_info, pcm, SNDRV_PCM_STREAM_PLAYBACK);

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream)
		ret = preallocate_idma_buffer(dma_info, pcm, SNDRV_PCM_STREAM_CAPTURE);

	return ret;
}

static struct snd_soc_platform_driver asoc_i2sidma_platform[10] = {

	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
};

static int asoc_i2sidma_platform_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	int id;
	const char *bind_name;
	int i2s_id;
	u32 dummy_node;

	id = of_alias_get_id(pdev->dev.of_node, "idma");
	if (id < 0)
		id = 0;

	if (id < 2) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&pdev->dev, "Failed to get mem resource0!\n");
			return -ENOENT;
		}
		hobot_i2sidma[id].regaddr_rx =
			devm_ioremap(&pdev->dev, res->start, resource_size(res));
		if (IS_ERR(hobot_i2sidma[id].regaddr_rx)) {
			dev_err(&pdev->dev, "Failed to ioremap regaddr_rx!\n");
			return PTR_ERR(hobot_i2sidma[id].regaddr_rx);
		}
		res = NULL;
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res) {
			dev_err(&pdev->dev, "Failed to get mem resource1!\n");
			return -ENOENT;
		}
		hobot_i2sidma[id].regaddr_tx =
			devm_ioremap(&pdev->dev, res->start, resource_size(res));
		if (IS_ERR(hobot_i2sidma[id].regaddr_tx)) {
			dev_err(&pdev->dev, "Failed to ioremap regaddr_tx!\n");
			return PTR_ERR(hobot_i2sidma[id].regaddr_tx);
		}

		res = NULL;
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (!res) {
			dev_err(&pdev->dev, "Failed to get mem resource2!\n");
			return -ENOENT;
		}


		ret = platform_get_irq(pdev, 0);
		if (ret <= 0) {
			dev_err(&pdev->dev, "Failed to get IRQ resource!\n");
			return ret;
		}
		dev_dbg(&pdev->dev, "ret = %d\n", ret);
		hobot_i2sidma[id].idma_irq = ret;
		hobot_i2sidma[id].id = id;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "dummy_node", &dummy_node);
	if (ret) {
		dev_err(&pdev->dev, "error getting dummy_node info\n");
		return ret;
	}
	dev_dbg(&pdev->dev, "dummy_node=%d\n", dummy_node);
	if (dummy_node == 1) {
		ret = of_property_read_string(pdev->dev.of_node, "i2s-bind", &bind_name);
		if (ret) {
			dev_err(&pdev->dev, "error getting codec dai_link name\n");
			return ret;
		}
		if (!strcmp(bind_name, "i2s0")) {
			i2s_id = 0;
		} else if (!strcmp(bind_name, "i2s1")) {
			i2s_id = 1;
		} else {
			dev_err(&pdev->dev, "invalid bind name %s\n", bind_name);
			return -EINVAL;
		}
		dev_dbg(&pdev->dev, "%s:%d i2s_id=%d\n", __func__, __LINE__, i2s_id);
		hobot_i2sidma[id].id = i2s_id;
		spin_lock_init(&global_info[i2s_id].lock);
	}

	spin_lock_init(&(hobot_i2sidma[id].lock));
	hobot_i2sidma[id].dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, &hobot_i2sidma[id]);

	ret =  devm_snd_soc_register_platform(&pdev->dev,
					      &asoc_i2sidma_platform[id]);
#ifdef CONFIG_HOBOT_DIAG
	if (diag_register(ModuleDiag_sound, EventIdSoundI2s0Err + id,
				5, DIAG_MSG_INTERVAL_MIN, DIAG_MSG_INTERVAL_MAX, NULL) < 0)
		dev_err(&pdev->dev, "i2s%d diag register fail\n", EventIdSoundI2s0Err + id);
#endif

	dev_info(&pdev->dev, "success register platform %d, ret = %d\n", id, ret);

	return ret;

}

static int asoc_i2sidma_platform_remove(struct platform_device *pdev) {
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hobot_i2sidma_of_match[] = {
	{.compatible = "hobot-i2s0-idma", },
	{.compatible = "hobot-i2s1-idma", },
	/* dummy dma */
	{.compatible = "hobot-i2s2-idma", },
	{.compatible = "hobot-i2s3-idma", },
	{.compatible = "hobot-i2s4-idma", },
	{.compatible = "hobot-i2s5-idma", },
	{.compatible = "hobot-i2s6-idma", },
	{.compatible = "hobot-i2s7-idma", },
	{.compatible = "hobot-i2s8-idma", },
	{.compatible = "hobot-i2s9-idma", },
	{}
};

MODULE_DEVICE_TABLE(of, hobot_i2sidma_of_match);
#endif


#ifdef CONFIG_PM
static int hobot_i2s_idma_suspend(struct device *dev) {
	unsigned long val;
	struct idma_info_s *info = (struct idma_info_s *)dev_get_drvdata(dev);

	dev_info(dev, "%s enter suspend......\n", __func__);
	if (!info) {
		return -EINVAL;
	}

	if (!info->regaddr_rx || !info->regaddr_tx) {
		return 0;
	}

	if (info->stream == SNDRV_PCM_STREAM_CAPTURE) {
		val = readl(info->regaddr_rx + I2S_CTL);
	} else {
		val = readl(info->regaddr_tx + I2S_CTL);
	}
	if (val == 0x3) {
		dev_err(dev, "i2s idma busy...\n");
		return -EBUSY;
	}
	return 0;
}

static int hobot_i2s_idma_resume(struct device *dev) {
	dev_info(dev, "%s enter resume......\n", __func__);
	return 0;
}

static const struct dev_pm_ops hobot_i2s_idma_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(hobot_i2s_idma_suspend,
		hobot_i2s_idma_resume)
};
#endif

static struct platform_driver i2s_idma_driver = {
	.driver = {
		   .name = "hobot-i2s-idma",
		   .of_match_table = hobot_i2sidma_of_match,
		   .pm = &hobot_i2s_idma_pm,
		   },

	.probe = asoc_i2sidma_platform_probe,
	.remove = asoc_i2sidma_platform_remove,
};

module_platform_driver(i2s_idma_driver);

MODULE_DESCRIPTION("Hobot ASoC I2SDMA Driver");
MODULE_LICENSE("GPL");
