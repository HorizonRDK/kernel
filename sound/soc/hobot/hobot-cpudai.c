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

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <linux/reset.h>

#include "hobot-i2s.h"

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

#define HOBOT_I2S_RATES    SNDRV_PCM_RATE_8000_96000

#define HOBOT_I2S_FMTS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE)

#define HOBOT_I2S_BCLK_DIV_LIMIE 8

#define HOBOT_DEF_I2S_MS 1;
static int i2s_ms = 2;
static int share_clk = 0;

module_param(i2s_ms, uint, S_IRUGO);
MODULE_PARM_DESC(i2s_ms, "Hobot i2s master/slave mode");

static inline int change_clk(struct device *dev,
        const char *clk_name, unsigned long rate);

#if IS_ENABLED(CONFIG_HOBOT_DMC_CLK)
static int hobot_dai_dpm_callback(struct hobot_dpm *self,
	unsigned long event, int state);
#endif
struct hobot_i2s_master_clk common_clk;

static unsigned int hobot_i2s_read_base_reg(struct hobot_i2s *i2s, int offset)
{
	if (i2s->streamflag == 0)
		return readl(i2s->regaddr_rx+offset);
	else
		return readl(i2s->regaddr_tx+offset);
}


/* enable/disable i2s controller */
static void i2s_transfer_ctl(struct hobot_i2s *i2s, bool on)
{
	pr_debug("ON?=%d\n",on);
	unsigned long val;
	void __iomem *addr;


		if (i2s->streamflag == 0) /* capture */
			addr = i2s->regaddr_rx + I2S_CTL;
		else
			addr = i2s->regaddr_tx + I2S_CTL;
		val = readl(addr);
		if (on)
			val |= CTL_ENABLE;
		else
			val &= ~CTL_ENABLE;

		writel(val, addr);

}


static int i2s_set_sysclk(struct snd_soc_dai *dai,
			   int clk_id, unsigned int rfs, int dir)
{

	return 0;
}

/* this function is be called by machine driver, */
/* set i2s/dsp mode and master/slave mode */
/* when setting master/slave mode, */
/* not only write tx/rx base register */
/* but also write sysctl clock register */
static int i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct hobot_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 val = 0;
	unsigned long flags;

	spin_lock_irqsave(&i2s->lock, flags);

	if (i2s->streamflag == 0) {  /* capture */
		val = readl(i2s->regaddr_rx + I2S_MODE);
	} else {		/* play */
		val = readl(i2s->regaddr_tx + I2S_MODE);
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:	/* i2s mode */
		val &= ~(MOD_I2S_MODE);
		i2s->i2sdsp = 0;
		dev_dbg(i2s->dev, "x2 config i2s mode\n");
		break;
	case SND_SOC_DAIFMT_DSP_A:	/* dsp mode */
	case SND_SOC_DAIFMT_DSP_B:
		i2s->i2sdsp = 1;
		val |= (MOD_I2S_MODE);
		dev_dbg(i2s->dev, "x2 config dsp mode\n");
		break;
	default:
		spin_unlock_irqrestore(&i2s->lock, flags);
		dev_err(i2s->dev, "Format not supported\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	/* Codec is master, and controler is slave. */
	case SND_SOC_DAIFMT_CBM_CFM:

		val &= ~MOD_MS_MODE;
		dev_dbg(i2s->dev, "x2 config master mode\n");
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		val |= MOD_MS_MODE;
		dev_dbg(i2s->dev, "x2 config slave mode\n");
		break;
	default:
		spin_unlock_irqrestore(&i2s->lock, flags);
		dev_err(i2s->dev, "master/slave format not supported\n");
		return -EINVAL;
	}

	if (i2s->streamflag == 0) {	/* capture */
		writel(val, i2s->regaddr_rx + I2S_MODE);
	} else {		/* play */
		if (i2s->ms == 4)
			val |= (0x1<<9); //slave
		writel(val, i2s->regaddr_tx + I2S_MODE);
	}
	spin_unlock_irqrestore(&i2s->lock, flags);
	return 0;
}

static void hobot_i2s_sample_rate_set(struct snd_pcm_substream *substream,
				   struct hobot_i2s *i2s)
{

	int ws_l, ws_h;
        int ret = 0;
        int lrck_div = 0;

	int bclk_div = HOBOT_I2S_BCLK_DIV_LIMIE;
	unsigned long flags;
	spin_lock_irqsave(&i2s->lock, flags);

	/* first get div_ws_l and div_ws_h, the value is equal */
	/* bclk = Fws*(chan*word_len) = Fws*(div_ws_l+1 + div_ws_h+1) */
	/* below fix mclk=4096khz, 4.096M = 1536/25/15 */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {

		if (i2s->i2sdsp == 0) {	/* i2s mode */
			lrck_div = i2s->wordlength * i2s->channel_num;
			if (lrck_div < 32)
				lrck_div = 32;
			if (i2s->samplerate == 32000)
				bclk_div = 6;
			ws_h = ws_l = (lrck_div / 2) - 1;
			i2s->div_ws = ws_l | (ws_h << 8);
			i2s->clk = i2s->samplerate * lrck_div;
			i2s->mclk_set = i2s->clk * bclk_div;
			spin_unlock_irqrestore(&i2s->lock, flags);
			if (i2s->ms == 1) {
				ret = change_clk(i2s->dev, "i2s-mclk",
					i2s->mclk_set);
				if (ret < 0) {
					pr_err("change i2s mclk failed\n");
					return;
				}
				ret = change_clk(i2s->dev, "i2s-bclk",
					i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
					return;
				}
			}else if (i2s->ms == 4 && share_clk == 1){
				pr_info("i2s mode:slave is using master clk in capture mode!\n");
				ret = change_clk(common_clk.dev, "i2s-mclk",
					i2s->mclk_set);
				if (ret < 0) {
					pr_err("change i2s mclk failed\n");
					return;
				}
				ret = change_clk(common_clk.dev, "i2s-bclk",
					i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
					return;
				}
			}
			spin_lock_irqsave(&i2s->lock, flags);
			writel(i2s->div_ws, i2s->regaddr_rx + I2S_DIV_WS);
		} else {/* dsp mode */
			ws_h = 0;
			ws_l = i2s->slot_width - 2;
			i2s->clk = i2s->samplerate * i2s->slot_width;
			pr_err("samplerate:%d,slot_width:%d\n",i2s->samplerate,i2s->slot_width);
			switch (i2s->samplerate) {
			case 44100:
			case 22050:
				i2s->mclk_set = 11289600;
				break;
			case 8000:
				i2s->mclk_set = 4096000;
				break;
			// case 64000:
			// case 32000:
			// 	i2s->mclk_set = 24576000;
			// 	break;
			default:
				i2s->mclk_set = 12288000;
				break;
			}

			if (i2s->mclk_set / i2s->clk > HOBOT_I2S_BCLK_DIV_LIMIE)
				i2s->mclk_set = 4096000;
			spin_unlock_irqrestore(&i2s->lock, flags);
			if (i2s->ms == 1) {
				ret = change_clk(i2s->dev, "i2s-mclk",
					i2s->mclk_set);
				ret = change_clk(i2s->dev, "i2s-bclk",
					i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
					return;
				}
			}else if (i2s->ms == 4  && share_clk == 1){
				pr_info("dsp mode:slave is using master clk in capture mode!\n");
				//i2s->clk = i2s->samplerate * 128; //ES7210 
				ret = change_clk(common_clk.dev, "i2s-mclk",
					i2s->mclk_set);
				if (ret < 0) {
					pr_err("change i2s mclk failed\n");
					return;
				}
				ret = change_clk(common_clk.dev, "i2s-bclk",
					i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
					return;
				}
			}
			spin_lock_irqsave(&i2s->lock, flags);
			writel(ws_l | (ws_h << 8), i2s->regaddr_rx +
				I2S_DIV_WS);
		}

	} else {/* play */
		if (i2s->i2sdsp == 0) {	/* i2s mode */
			lrck_div = i2s->wordlength * i2s->channel_num;
			if (lrck_div < 32)
				lrck_div = 32;
			if (i2s->samplerate == 32000)
				bclk_div = 6;
			ws_h = ws_l = (lrck_div / 2) - 1;
			i2s->div_ws = ws_l | (ws_h << 8);
			i2s->clk = i2s->samplerate * lrck_div;
			i2s->mclk_set = i2s->clk * bclk_div;
			spin_unlock_irqrestore(&i2s->lock, flags);
			if (i2s->ms == 1) {
				ret = change_clk(i2s->dev, "i2s-mclk",
					i2s->mclk_set);
				if (ret < 0) {
					pr_err("change i2s mclk failed\n");
					return;
				}
				ret = change_clk(i2s->dev, "i2s-bclk",
					i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
					return;
				}
			}else if (i2s->ms == 4  && share_clk == 1){
				pr_info("i2s mode:slave is using master clk in playback mode!\n");
				ret = change_clk(common_clk.dev, "i2s-mclk",
					i2s->mclk_set);
				if (ret < 0) {
					pr_err("change i2s mclk failed\n");
					return;
				}
				ret = change_clk(common_clk.dev, "i2s-bclk",
					i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
					return;
				}
			}
			spin_lock_irqsave(&i2s->lock, flags);
			writel(i2s->div_ws, i2s->regaddr_tx + I2S_DIV_WS);
#if 0
			if (i2s->ms == 4) {
				writel(i2s->div_ws, i2s->regaddr_tx + I2S_DIV_WS);
			} else {
			switch(i2s->samplerate) {
			case 22050:
			case 16000:
				lrck_div = 128;
				break;
			case 48000:
				lrck_div = 32;
				break;
			default:
				lrck_div = 64;
				break;
			}

			i2s->clk = i2s->samplerate * lrck_div;
			ws_l = ws_h = (lrck_div / 2) - 1;
			spin_unlock_irqrestore(&i2s->lock, flags);

			clk_disable(i2s->bclk);
			clk_disable(i2s->mclk);
			if (i2s->samplerate == 44100 || i2s->samplerate == 22050 ||
					i2s->samplerate == 8000) {
				mclk_period = 4;
				mclk = mclk_period * i2s->clk;
				ret = change_clk(i2s->dev, "i2s-mclk", mclk);
				if (ret < 0) {
					pr_err("change i2s mclk failed\n");
				}
				ret = change_clk(i2s->dev, "i2s-bclk", i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
				}
			} else {
				ret = change_clk(i2s->dev, "i2s-mclk",
					i2s->mclk_set);
				if (ret < 0) {
					pr_err("change i2s mclk failed\n");
				}
				ret = change_clk(i2s->dev, "i2s-bclk",
					i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
				}
			}
			clk_enable(i2s->mclk);
			clk_enable(i2s->bclk);
			spin_lock_irqsave(&i2s->lock, flags);
			i2s->div_ws = ws_l | (ws_h << 8);
			writel(i2s->div_ws, i2s->regaddr_tx + I2S_DIV_WS);
			}
#endif
		} else {
			ws_h = 0;
			ws_l = i2s->slot_width - 2;
			i2s->clk = i2s->samplerate * i2s->slot_width;
			pr_err("i2s->clk:%d,i2s->slot_width:%d\n",i2s->clk,i2s->slot_width);
			switch (i2s->samplerate) {
			case 44100:
			case 22050:
				i2s->mclk_set = 11289600;
				break;
			case 8000:
				i2s->mclk_set = 4096000;
				break;
			// case 64000:
			// case 32000:
			// 	i2s->mclk_set = 24576000;
			// 	break;
			default:
				i2s->mclk_set = 12288000;
				break;
			}

			if (i2s->mclk_set / i2s->clk > HOBOT_I2S_BCLK_DIV_LIMIE)
				i2s->mclk_set = 4096000;
			spin_unlock_irqrestore(&i2s->lock, flags);
			if (i2s->ms == 1) {
				ret = change_clk(i2s->dev, "i2s-mclk",
					i2s->mclk_set);
				if (ret < 0) {
					pr_err("change i2s mclk failed\n");
					return;
				}
				ret = change_clk(i2s->dev, "i2s-bclk",
					i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
					return;
				}
			}else if (i2s->ms == 4  && share_clk == 1){
				pr_info("dsp mode:slave is using master clk in playback mode!\n");
				//i2s->clk = i2s->samplerate * 256; //ES8156
				ret = change_clk(common_clk.dev, "i2s-mclk",
					i2s->mclk_set);
				if (ret < 0) {
					pr_err("change i2s mclk failed\n");
					return;
				}
				ret = change_clk(common_clk.dev, "i2s-bclk",
					i2s->clk);
				if (ret < 0) {
					pr_err("change i2s bclk failed\n");
					return;
				}
			}
			spin_lock_irqsave(&i2s->lock, flags);
			writel(ws_l | (ws_h << 8), i2s->regaddr_tx +
				I2S_DIV_WS);
		}
	}
	spin_unlock_irqrestore(&i2s->lock, flags);
}


static int i2s_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  struct snd_soc_dai *dai)
{
	struct hobot_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 mod, chan = 0;
	unsigned long flags;

	spin_lock_irqsave(&i2s->lock, flags);
	i2s->channel_num = params_channels(params);
	i2s->samplerate = params_rate(params);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		mod = readl(i2s->regaddr_rx + I2S_MODE);
		mod &= ~MOD_CH_NUM;
		/* setting channel num and enable channel */

		if (i2s->channel_num <= 16 && i2s->channel_num >= 9) {
			mod |= MOD_16_CH;
			chan |= 0xffff;
		} else if (i2s->channel_num <= 8 && i2s->channel_num >= 5) {
			mod |= MOD_8_CH;
			chan |= 0xff;
		} else if (i2s->channel_num <= 4 && i2s->channel_num >= 3) {
			mod |= MOD_4_CH;
			chan |= 0xf;
		} else if (i2s->channel_num == 2) {
			mod |= MOD_2_CH;
			chan |= 0x3;
		} else if (i2s->channel_num == 1) {
			mod |= MOD_1_CH_C;
			chan |= 0x1;
		} else {
			spin_unlock_irqrestore(&i2s->lock, flags);
			dev_err(i2s->dev, "%d channels not supported\n",
				params_channels(params));
			return -EINVAL;
		}
		switch (params_format(params)) {	/* 16bit or 8bit. */
		case SNDRV_PCM_FORMAT_S16_LE:
			mod |= MOD_WORD_LEN;
			i2s->wordlength = 16;
			i2s->slot_width = 128;
			break;
		case SNDRV_PCM_FORMAT_S8:

			i2s->wordlength  = 8;
			i2s->slot_width = 64;
			mod &= ~MOD_WORD_LEN;
			break;
		default:
			spin_unlock_irqrestore(&i2s->lock, flags);
			dev_err(i2s->dev, "not supported data format %d\n",
				params_format(params));
			return -EINVAL;
		}
		/**
		 * NOTE: DSP
		 * 
		 */
		// if (i2s->ms == 4) {
		// 	i2s->slot_width = i2s->channel_num * i2s->wordlength;
		// 	dev_err(i2s->dev,"slot_width = %d,wordlength = %d\n",i2s->slot_width,i2s->wordlength);
		// }
		writel(mod, i2s->regaddr_rx + I2S_MODE);
		writel(chan, i2s->regaddr_rx + I2S_CH_EN);

	} else {/* playback */
		mod = readl(i2s->regaddr_tx + I2S_MODE);
		mod &= ~MOD_CH_NUM;
		/* setting tx channel num and enable channel */
		switch (params_channels(params)) {

		case 2:
			mod |= MOD_2_CH;
			chan |= 0x3;
			break;
		case 1:
			mod |= MOD_1_CH_P;
			chan |= 0x1;
			break;
		default:
			spin_unlock_irqrestore(&i2s->lock, flags);
			dev_err(i2s->dev, "%d channels not supported\n",
				params_channels(params));
			return -EINVAL;
		}
		switch (params_format(params)) {	/* 16bit or 8bit. */
		case SNDRV_PCM_FORMAT_S16_LE:
			mod |= MOD_WORD_LEN;

			i2s->wordlength  = 16;
			break;
		case SNDRV_PCM_FORMAT_S8:
			mod &= ~MOD_WORD_LEN;

			i2s->wordlength  = 8;
			break;
		default:
			spin_unlock_irqrestore(&i2s->lock, flags);
			dev_err(i2s->dev, "not supported data format %d\n",
				params_format(params));
			return -EINVAL;
		}
		writel(mod, i2s->regaddr_tx + I2S_MODE);
		writel(chan, i2s->regaddr_tx + I2S_CH_EN);
	}
	spin_unlock_irqrestore(&i2s->lock, flags);
	dev_err(i2s->dev, "x2 config channel %d, samplerate is %d\n",
		i2s->channel_num, i2s->samplerate);
	dev_dbg(i2s->dev, "x2 config wordlength is %d\n", i2s->wordlength);

	hobot_i2s_sample_rate_set(substream, i2s);
	return 0;
}

/* reset i2s by reset framework */
static int i2s_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct hobot_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;

	//dev_dbg(i2s->dev, "i2s_startup S, i2s->id is %d\n", i2s->id);
	spin_lock_irqsave(&i2s->lock, flags);

	if (i2s->open_flag) {
		spin_unlock_irqrestore(&i2s->lock, flags);
		return -EBUSY;
	}

	/* enable mclk, first disable bclk, if master, enable it later */
	//clk_enable(i2s->mclk);

	reset_control_assert(i2s->rst);
	ndelay(100);
	reset_control_deassert(i2s->rst);

	/* reset done, disable reset module */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		i2s->streamflag = 0;
	else
		i2s->streamflag = 1;

	i2s->open_flag = true;

	spin_unlock_irqrestore(&i2s->lock, flags);
	return 0;
}

static int i2s_trigger(struct snd_pcm_substream *substream,
			int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct hobot_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	unsigned long flags;

	//dev_dbg(i2s->dev, "i2s_trigger cmd is %d\n", cmd);
	mdelay(1);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&i2s->lock, flags);
		if (i2s->streamflag == 0) {
			writel(0x1, i2s->regaddr_rx + I2S_BUF0_RDY);
			writel(0x1, i2s->regaddr_rx + I2S_BUF1_RDY);
		} else {
			writel(0x1, i2s->regaddr_tx + I2S_BUF0_RDY);
			writel(0x1, i2s->regaddr_tx + I2S_BUF1_RDY);
		}
		i2s_transfer_ctl(i2s, true);
		i2s->current_status = 1;
		spin_unlock_irqrestore(&i2s->lock, flags);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&i2s->lock, flags);
		if (i2s->work_mode == 0)
			;
			// i2s_transfer_ctl(i2s, false);
		if (i2s->streamflag == 0) {
			writel(0x0, i2s->regaddr_rx + I2S_BUF0_RDY);
			writel(0x0, i2s->regaddr_rx + I2S_BUF1_RDY);
		} else {
			writel(0x0, i2s->regaddr_tx + I2S_BUF0_RDY);
			writel(0x0, i2s->regaddr_tx + I2S_BUF1_RDY);
		}
		i2s->current_status = 0;
		spin_unlock_irqrestore(&i2s->lock, flags);
		break;
	}

	return 0;
}

/* this function is be called by machine driver, */
/* this func do not any anction, becasuse */
/* machine->param is first called, at this time can not get runtime param */
static int i2s_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	/* no action, because samplerate setting is done by i2s0_hw_params */
	return 0;
}

static int i2s_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai) {
	struct hobot_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;

	spin_lock_irqsave(&i2s->lock, flags);
	i2s->open_flag = false;
	spin_unlock_irqrestore(&i2s->lock, flags);

	return 0;
}

static const struct snd_soc_dai_ops hobot_i2s_dai_ops = {
	.trigger = i2s_trigger,
	.hw_params = i2s_hw_params,
	.set_fmt = i2s_set_fmt,
	.set_clkdiv = i2s_set_clkdiv,
	.set_sysclk = i2s_set_sysclk,
	.startup = i2s_startup,
	.hw_free = i2s_hw_free,
};

static int hobot_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct hobot_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &i2s->playback_dma_data,
				  &i2s->capture_dma_data);
	snd_soc_dai_set_drvdata(dai, i2s);

	return 0;
}


static int hobot_i2s_dai_remove(struct snd_soc_dai *dai)
{
	return 0;
}

static struct snd_soc_dai_driver hobot_i2s_dai_drv[2] = {

	{
		.probe = hobot_i2s_dai_probe,
		.remove = hobot_i2s_dai_remove,

		.playback = {
			     .stream_name = "Playback",
			     .channels_min = 1,
			     .channels_max = 2,
			     .rates = HOBOT_I2S_RATES,
			     .formats = HOBOT_I2S_FMTS,
			     },

		.capture = {
			     .stream_name = "Capture",
			     .channels_min = 1,
			     .channels_max = 12,
			     .rates = HOBOT_I2S_RATES,
			     .formats = HOBOT_I2S_FMTS,
			     },

		.ops = &hobot_i2s_dai_ops,
		.symmetric_rates = 1,
		.name = "hobot-i2s0",
	},
	{
		.probe = hobot_i2s_dai_probe,/* the same as i2s0 dai param */
		.remove = hobot_i2s_dai_remove,/* the same as i2s0 dai param */
		.playback = {
			     .stream_name = "Playback",
			     .channels_min = 1,
			     .channels_max = 2,
			     .rates = HOBOT_I2S_RATES,
			     .formats = HOBOT_I2S_FMTS,
			     },
		.capture = {
			    .stream_name = "Capture",
			    .channels_min = 1,
			    .channels_max = 8,
			    .rates = HOBOT_I2S_RATES,
			    .formats = HOBOT_I2S_FMTS,
			    },
		.ops = &hobot_i2s_dai_ops,/* the same as i2s0 dai param */
		.symmetric_rates = 1,
		.name = "hobot-i2s1",
	},
};
static const struct snd_soc_component_driver hobot_i2s_component_drv[2];
static ssize_t store_hobot_i2s_reg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct hobot_i2s *i2s;

	i2s = dev->driver_data;
	return 0;
}

static ssize_t show_hobot_i2s_reg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct hobot_i2s *i2s = (struct hobot_i2s *)dev_get_drvdata(dev);
	unsigned int val;

	char *s = buf;


		val = hobot_i2s_read_base_reg(i2s, I2S_CTL);
		s += sprintf(s, "ctl: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_MODE);
		s += sprintf(s, "mode: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_DIV_WS);
		s += sprintf(s, "div_ws: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_CH_EN);
		s += sprintf(s, "ch_en: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_BUF_SIZE);
		s += sprintf(s, "buf_size: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_BUF0_ADDR);
		s += sprintf(s, "buf0_addr: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_BUF0_RDY);
		s += sprintf(s, "buf0_rdy: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_BUF1_ADDR);
		s += sprintf(s, "buf1_addr: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_BUF1_RDY);
		s += sprintf(s, "buf1_rdy: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_BUF_CUR_ADDR);
		s += sprintf(s, "buf_cur_addr: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_CH_ERROR);
		s += sprintf(s, "ch_error: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_SRCPND);
		s += sprintf(s, "srcpnd: 0x%08x\n", val);
		val = hobot_i2s_read_base_reg(i2s, I2S_INTMASK);
		s += sprintf(s, "intmask: 0x%08x\n", val);
		if (s != buf)
			*(s-1) = '\n';
		return s-buf;
}

static DEVICE_ATTR(reg_dump, 0644, show_hobot_i2s_reg, store_hobot_i2s_reg);

static inline int change_clk(struct device *dev,
	const char *clk_name, unsigned long rate)
{
		struct clk *clk;
		long round_rate;
		int ret = 0;

		clk = devm_clk_get(dev, clk_name);
		if (IS_ERR(clk)) {
			dev_err(dev, "failed to get: %s\n", clk_name);
			return PTR_ERR_OR_ZERO(clk);
		}
		round_rate = clk_round_rate(clk, rate);
		ret = clk_set_rate(clk, (unsigned long)round_rate);
		if (unlikely(ret < 0)) {
			dev_err(dev, "failed to set clk: %s\n", clk_name);
			return ret;
		}
		dev_info(dev, "%s: clk:%lu, round_rate:%ld",
			clk_name, rate, round_rate);
		return ret;
}

static int hobot_i2s_probe(struct platform_device *pdev)
{
	struct hobot_i2s *i2s;
	int id;
	int ret;
	struct resource *res;
	/* struct snd_soc_dai_driver *soc_dai_driver; */
	id = of_alias_get_id(pdev->dev.of_node, "i2s");
	if (id < 0)
		id = 0;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s) {
		/* dev_err(&pdev->dev, "Can't allocate hobot_i2s0\n"); */
		return -ENOMEM;
	}

	i2s->dev = &pdev->dev;
	i2s->id = id;

	spin_lock_init(&i2s->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get mem resource0!\n");
		return -ENOENT;
	}
	i2s->regaddr_rx = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2s->regaddr_rx)) {
		dev_err(&pdev->dev, "Failed to ioremap regaddr_rx!\n");
		return PTR_ERR_OR_ZERO(i2s->regaddr_rx);
	}
	res = NULL;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get mem resource1!\n");
		return -ENOENT;
	}
	i2s->regaddr_tx = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2s->regaddr_tx)) {
		dev_err(&pdev->dev, "Failed to ioremap regaddr_tx!\n");
		return PTR_ERR_OR_ZERO(i2s->regaddr_tx);
	}

	res = NULL;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get mem resource2!\n");
		return -ENOENT;
	}
	i2s->sysctl_addr =
			devm_ioremap(&pdev->dev,
				res->start, resource_size(res));


	i2s->mclk = devm_clk_get(&pdev->dev, "i2s-mclk");
		if (IS_ERR(i2s->mclk)) {
			dev_err(&pdev->dev, "failed to get i2s-mclk\n");
			return PTR_ERR_OR_ZERO(i2s->mclk);
		}
		ret = clk_prepare_enable(i2s->mclk);
		if (ret != 0) {
			dev_err(&pdev->dev, "failed to prepare i2s-mclk\n");
			return ret;
		}

		i2s->bclk = devm_clk_get(&pdev->dev, "i2s-bclk");
		if (IS_ERR(i2s->bclk)) {
			dev_err(&pdev->dev, "failed to get i2s-bclk\n");
			return PTR_ERR_OR_ZERO(i2s->bclk);
		}
		ret = clk_prepare_enable(i2s->bclk);
		if (ret != 0) {
			dev_err(&pdev->dev, "failed to prepare i2s-bclk\n");
			return ret;
		}

		ret = of_property_read_u32(pdev->dev.of_node,
			"bclk_set", &i2s->bclk_set);
		if (ret < 0) {
			pr_err("failed:get  blck rc %d", ret);
			return ret;
		}
		ret = of_property_read_u32(pdev->dev.of_node,
			"ms", &i2s->ms);

		if (i2s_ms != 2) {
			i2s->ms = i2s_ms;
		}
		if (ret < 0) {
			pr_err("failed:get  ms rc %d", ret);
			return ret;
		}

		/****
		 * Common clk setting start...
		*/
		pr_info("i2s_ms:%d\n",i2s->ms);
		ret = of_property_read_u32(pdev->dev.of_node,
			"share_clk", &share_clk);
		if (ret < 0) {
			pr_err("failed:get share_clk %d", ret);
			share_clk = 0;
			//return ret; //we don't return here
		}
		//When the current i2s is master and only if share_clk flag is enabled on dts
		if(i2s->ms == 1 && share_clk == 1){ 
			common_clk.dev = i2s->dev;
			common_clk.bclk = i2s->bclk;
			common_clk.mclk = i2s->mclk;
		}
		/****
		 * Common clk setting end...
		*/

		ret = of_property_read_u32(pdev->dev.of_node,
					"slot_width", &i2s->slot_width);
		if (ret < 0) {
			pr_err("failed:get	slot_width rc %d", ret);
			return ret;
		}
		ret = of_property_read_u32(pdev->dev.of_node,
					"mclk_set", &i2s->mclk_set);
			if (ret < 0) {
				pr_err("failed:get	blck rc %d", ret);
				return ret;
			}

		reset_control_assert(i2s->rst);
		ndelay(100);
		reset_control_deassert(i2s->rst);
		if (i2s->ms == 4) {
			clk_disable(i2s->bclk);
		}
#if 0
		ret = change_clk(&pdev->dev, "i2s-mclk",
			i2s->mclk_set);
		ret = change_clk(&pdev->dev, "i2s-bclk",
			i2s->bclk_set);
		clk_enable(i2s->mclk);
		clk_enable(i2s->bclk);
		pr_info("change_clk blck ret = %d\n", ret);
#endif
		ret = of_property_read_u32(pdev->dev.of_node,
			"div_ws", &i2s->div_ws);
		if (ret < 0) {
			pr_err("failed:get  div_ws rc %d", ret);
			return ret;
		}

	i2s->rst = devm_reset_control_get(&pdev->dev, "i2s");
	if (IS_ERR(i2s->rst)) {
		dev_err(&pdev->dev, "Missing reset controller!\n");
		return PTR_ERR_OR_ZERO(i2s->rst);
	}

	i2s->open_flag = false;

	/* hobot_i2s is set to cpudai dev data */
	dev_set_drvdata(&pdev->dev, i2s);

	ret =
		devm_snd_soc_register_component(&pdev->dev,
			&hobot_i2s_component_drv[id], &hobot_i2s_dai_drv[id], 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI\n");
		return ret;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_reg_dump);

#if IS_ENABLED(CONFIG_HOBOT_DMC_CLK)
	i2s->dpm.dpm_call = hobot_dai_dpm_callback;
	i2s->dpm.priority = DPM_PRI_LEVEL1;
	hobot_dfs_register(&i2s->dpm, &pdev->dev);
#endif

	pr_err("success register cpu dai%d driver\n", id);
	return 0;

}

/* static const struct snd_soc_component_driver hobot_i2s_component_drv = { */
/* .name = DRV_NAME, */
/* }; */
static int hobot_i2s_remove(struct platform_device *pdev)
{
	struct hobot_i2s *i2s = dev_get_drvdata(&pdev->dev);

	/* struct hobot_i2s *i2s = dev_get_drvdata (&pdev->dev); */
	device_remove_file(&pdev->dev, &dev_attr_reg_dump);
	snd_soc_unregister_component(&pdev->dev);

#if IS_ENABLED(CONFIG_HOBOT_DMC_CLK)
	hobot_dfs_unregister(&i2s->dpm);
#endif

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hobot_i2s_of_match[] = {
	{.compatible = "hobot, hobot-i2s0",},
	{.compatible = "hobot, hobot-i2s1",},
	{}
};

MODULE_DEVICE_TABLE(of, hobot_i2s_of_match);
#endif

#ifdef CONFIG_PM
int hobot_i2s_suspend(struct device *dev) {
	unsigned long value;
	struct hobot_i2s *i2s = (struct hobot_i2s *)dev_get_drvdata(dev);
	dev_info(dev, "%s enter suspend......\n", __func__);
	if (!i2s)
		return -EINVAL;

	if (i2s->streamflag == 0) {
		value = readl(i2s->regaddr_rx + I2S_CTL);
		i2s->suspend_i2smod = readl(i2s->regaddr_rx + I2S_MODE);
		i2s->suspend_i2schen = readl(i2s->regaddr_rx + I2S_CH_EN);
		i2s->suspend_i2sdivws = readl(i2s->regaddr_rx + I2S_DIV_WS);
	} else {
		value = readl(i2s->regaddr_tx + I2S_CTL);
		i2s->suspend_i2smod = readl(i2s->regaddr_tx + I2S_MODE);
		i2s->suspend_i2schen = readl(i2s->regaddr_tx + I2S_CH_EN);
		i2s->suspend_i2sdivws = readl(i2s->regaddr_tx + I2S_DIV_WS);
	}
	if (i2s->current_status == 1) {
		dev_err(dev, "i2s busy...\n");
		return -EBUSY;
	}

	if (i2s->ms == 1) {
		clk_disable(i2s->mclk);
		clk_disable(i2s->bclk);
	}
	return 0;
}

int hobot_i2s_resume(struct device *dev) {
	int ret;
	struct hobot_i2s *i2s =	dev_get_drvdata(dev);
	dev_info(dev, "%s enter resume......\n", __func__);
	if (!i2s)
		return -EINVAL;

	if (i2s->ms == 1) {
		clk_enable(i2s->mclk);
		clk_enable(i2s->bclk);
	}
	if (i2s->streamflag == 0) {
		writel(i2s->suspend_i2smod, i2s->regaddr_rx + I2S_MODE);
		writel(i2s->suspend_i2schen, i2s->regaddr_rx + I2S_CH_EN);
		writel(i2s->suspend_i2sdivws, i2s->regaddr_rx + I2S_DIV_WS);
	} else {
		writel(i2s->suspend_i2smod, i2s->regaddr_tx + I2S_MODE);
		writel(i2s->suspend_i2schen, i2s->regaddr_tx + I2S_CH_EN);
		writel(i2s->suspend_i2sdivws, i2s->regaddr_tx + I2S_DIV_WS);
	}
	return 0;
}

static const struct dev_pm_ops hobot_i2s_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(hobot_i2s_suspend, hobot_i2s_resume)
};
#endif

#if IS_ENABLED(CONFIG_HOBOT_DMC_CLK)
static int hobot_dai_dpm_callback(struct hobot_dpm *self,
	unsigned long event, int state) {
	int ret = 0;
	unsigned long value;
	struct hobot_i2s *i2s = container_of(self, struct hobot_i2s, dpm);
	if(IS_ERR(i2s))
		return -ENODEV;

	if (event == HB_BUS_SIGNAL_START) {
		if (i2s->streamflag == 0) {
			value = readl(i2s->regaddr_rx + I2S_CTL);
		} else {
			value = readl(i2s->regaddr_tx + I2S_CTL);
		}

		if (value == 3) {
			ret = -EBUSY;
		} else {
			ret = hobot_i2s_pm.suspend(i2s->dev);
		}
	} else if (event == HB_BUS_SIGNAL_END) {
		ret = hobot_i2s_pm.resume(i2s->dev);
	}
	return ret;
}
#endif

static struct platform_driver hobot_i2s_driver = {
	.probe = hobot_i2s_probe,
	.remove = hobot_i2s_remove,
	.driver = {
		   .name = "hobot-i2s",
		   .of_match_table = hobot_i2s_of_match,
		   .pm = &hobot_i2s_pm,
		   },
};

module_platform_driver(hobot_i2s_driver);

/* Module information */
MODULE_AUTHOR("Jxy");
MODULE_DESCRIPTION("Hobot I2S Soc Interface");
MODULE_ALIAS("cpu dai driver:hobot-i2s");
MODULE_LICENSE("GPL");
