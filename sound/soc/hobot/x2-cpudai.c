/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

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

#include "x2-i2s.h"

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

#define X2_I2S_RATES    SNDRV_PCM_RATE_8000_96000

#define X2_I2S_FMTS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE)

/* static dma_addr_t i2s0_dma_addr = AUDMA_I2S0_RXBUFFER; */
static int current_channel_num;
static int current_wordlength;
static int current_samplerate;

/* enable/disable i2s controller */
static void i2s_transfer_ctl(struct x2_i2s *i2s, bool on)
{
	unsigned long val;

	if (i2s->streamflag == 0) {	/* capture */
		val = readl(i2s->regaddr_rx + I2S_CTL);
		if (on)
			val |= CTL_ENABLE;
		else
			val &= ~CTL_ENABLE;

		writel(val, i2s->regaddr_rx + I2S_CTL);
	} else {
		val = readl(i2s->regaddr_tx + I2S_CTL);
		if (on)
			val |= CTL_ENABLE;
		else
			val &= ~CTL_ENABLE;

		writel(val, i2s->regaddr_tx + I2S_CTL);
	}

}

static void i2s_set_master(struct x2_i2s *i2s, bool on)
{
	unsigned long val;

	if (i2s->id == 0) {
		val = readl(i2s->sysctl_addr + I2S0_CLKCTL_BASE);
		if (on)
			val &= ~(0x1 << CLK_MODE);
		else
			val |= (0x1 << CLK_MODE);

		writel(val, i2s->sysctl_addr + I2S0_CLKCTL_BASE);
	} else {
		val = readl(i2s->sysctl_addr + I2S1_CLKCTL_BASE);
		if (on)
			val &= ~(0x1 << CLK_MODE);
		else
			val |= (0x1 << CLK_MODE);

		writel(val, i2s->sysctl_addr + I2S1_CLKCTL_BASE);
	}

}

static int i2s_set_sysclk(struct snd_soc_dai *dai,
			   int clk_id, unsigned int rfs, int dir)
{
	/* struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata (dai); */
	/* will do clock setting */
	return 0;
}

/* this function is be called by machine driver, */
/* set i2s/dsp mode and master/slave mode */
/* when setting master/slave mode, */
/* not only write tx/rx base register */
/* but also write sysctl clock register */
static int i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct x2_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 val = 0;
	unsigned long flags;

	spin_lock_irqsave(&i2s->lock, flags);

	if (i2s->streamflag == 0) {	/* capture */
		val = readl(i2s->regaddr_rx + I2S_MODE);
	} else {		/* play */
		val = readl(i2s->regaddr_tx + I2S_MODE);
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:	/* i2s mode */
		val &= ~(MOD_I2S_MODE);
		i2s->i2sdsp = 0;
		break;
	case SND_SOC_DAIFMT_DSP_A:	/* dsp mode */
	case SND_SOC_DAIFMT_DSP_B:
		i2s->i2sdsp = 1;
		val |= (MOD_I2S_MODE);
		break;
	default:
		dev_err(i2s->dev, "Format not supported\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	/* Codec is master, and controler is slave. */
	case SND_SOC_DAIFMT_CBM_CFM:
		val |= MOD_MS_MODE;
		i2s_set_master(i2s, 0);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		val &= ~MOD_MS_MODE;
		i2s_set_master(i2s, 1);
		break;
	default:
		dev_err(i2s->dev, "master/slave format not supported\n");
		return -EINVAL;
	}

	if (i2s->streamflag == 0) {	/* capture */
		writel(val, i2s->regaddr_rx + I2S_MODE);
	} else {		/* play */
		writel(val, i2s->regaddr_tx + I2S_MODE);
	}

	spin_unlock_irqrestore(&i2s->lock, flags);
	return 0;
}

static void x2_i2s_sample_rate_set(struct snd_pcm_substream *substream,
				   struct x2_i2s *i2s)
{

	int ws_l, ws_h;

	u32 val = 0;
	u32 reg_val = 0;
	uint64_t bclk;
	int div;

	/* first get div_ws_l and div_ws_h, the value is equal */
	/* bclk = Fws*(chan*word_len) = Fws*(div_ws_l+1 + div_ws_h+1) */
	/* below fix mclk=4096khz, 4.096M = 1536/25/15 */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (i2s->id == 0) {
			val = readl(i2s->sysctl_addr + I2S0_CLKCTL_BASE);
			val |= (24 << 0);
			val |= (14 << 8);

			bclk =
			    current_samplerate * current_channel_num *
			    current_wordlength;

			/* below code only is for debug, not cover all bclk */
			div = 4096000 / bclk - 1;
			val |= (div << 16);

			writel(val, i2s->sysctl_addr + I2S0_CLKCTL_BASE);
		} else {
			val = readl(i2s->sysctl_addr + I2S1_CLKCTL_BASE);
			val |= (24 << 0);
			val |= (14 << 8);

			bclk =
			    current_samplerate * current_channel_num *
			    current_wordlength;

			/* below code only is for debug, not cover all bclk */
			div = 4096000 / bclk - 1;
			val |= (div << 16);

			writel(val, i2s->sysctl_addr + I2S1_CLKCTL_BASE);
		}


		if (i2s->i2sdsp == 0) {	/* i2s mode */
			ws_l = current_channel_num * current_wordlength / 2 - 1;
			ws_h = ws_l;
			reg_val = readl(i2s->regaddr_rx + I2S_DIV_WS);
			writel(UPDATE_VALUE_FIELD
			       (reg_val, ws_l, I2S_DIV_WS_DIV_WS_L_BIT,
				I2S_DIV_WS_DIV_WS_L_FIELD),
			       i2s->regaddr_rx + I2S_DIV_WS);
			writel(UPDATE_VALUE_FIELD
			       (reg_val, ws_h, I2S_DIV_WS_DIV_WS_H_BIT,
				I2S_DIV_WS_DIV_WS_H_FIELD),
			       i2s->regaddr_rx + I2S_DIV_WS);
			/* setting bclk register */
		} else {
			ws_h = 0;
			ws_l = current_channel_num * current_wordlength - 2;
			reg_val = readl(i2s->regaddr_rx + I2S_DIV_WS);
			writel(UPDATE_VALUE_FIELD
			       (reg_val, ws_l, I2S_DIV_WS_DIV_WS_L_BIT,
				I2S_DIV_WS_DIV_WS_L_FIELD),
			       i2s->regaddr_rx + I2S_DIV_WS);
			writel(UPDATE_VALUE_FIELD
			       (reg_val, ws_h, I2S_DIV_WS_DIV_WS_H_BIT,
				I2S_DIV_WS_DIV_WS_H_FIELD),
			       i2s->regaddr_rx + I2S_DIV_WS);
			/* setting bclk register */
		}

	} else {/* play */
		if (i2s->id == 0) {
			val = readl(i2s->sysctl_addr + I2S0_CLKCTL_BASE);
			val |= (24 << 0);
			val |= (14 << 8);

			bclk =
			    current_samplerate * current_channel_num *
			    current_wordlength;
			div = 4096000 / bclk - 1;
			val |= (div << 16);

			writel(val, i2s->sysctl_addr + I2S0_CLKCTL_BASE);
		} else {
			val = readl(i2s->sysctl_addr + I2S1_CLKCTL_BASE);
			val |= (24 << 0);
			val |= (14 << 8);

			bclk =
			    current_samplerate * current_channel_num *
			    current_wordlength;
			div = 4096000 / bclk - 1;
			val |= (div << 16);

			writel(val, i2s->sysctl_addr + I2S1_CLKCTL_BASE);
		}


		if (i2s->i2sdsp == 0) {	/* i2s mode */
			ws_l = current_channel_num * current_wordlength / 2 - 1;
			ws_h = ws_l;
			reg_val = readl(i2s->regaddr_tx + I2S_DIV_WS);
			writel(UPDATE_VALUE_FIELD
			       (reg_val, ws_l, I2S_DIV_WS_DIV_WS_L_BIT,
				I2S_DIV_WS_DIV_WS_L_FIELD),
			       i2s->regaddr_tx + I2S_DIV_WS);
			writel(UPDATE_VALUE_FIELD
			       (reg_val, ws_h, I2S_DIV_WS_DIV_WS_H_BIT,
				I2S_DIV_WS_DIV_WS_H_FIELD),
			       i2s->regaddr_tx + I2S_DIV_WS);
		} else {
			ws_h = 0;
			ws_l = current_channel_num * current_wordlength - 2;
			reg_val = readl(i2s->regaddr_tx + I2S_DIV_WS);
			writel(UPDATE_VALUE_FIELD
			       (reg_val, ws_l, I2S_DIV_WS_DIV_WS_L_BIT,
				I2S_DIV_WS_DIV_WS_L_FIELD),
			       i2s->regaddr_tx + I2S_DIV_WS);
			writel(UPDATE_VALUE_FIELD
			       (reg_val, ws_h, I2S_DIV_WS_DIV_WS_H_BIT,
				I2S_DIV_WS_DIV_WS_H_FIELD),
			       i2s->regaddr_tx + I2S_DIV_WS);
		}
	}

}

/* this set/enable channel num and 8/16 bit and setting clock */
static int i2s_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  struct snd_soc_dai *dai)
{
	struct x2_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 mod, chan = 0;
	unsigned long flags;

	spin_lock_irqsave(&i2s->lock, flags);

	current_channel_num = params_channels(params);
	current_samplerate = params_rate(params);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		mod = readl(i2s->regaddr_rx + I2S_MODE);
		mod &= ~MOD_CH_NUM;
		/* setting channel num and enable channel */
		switch (params_channels(params)) {
		case 16:
			mod |= MOD_16_CH;
			chan |= 0xffff;
		case 8:
			mod |= MOD_8_CH;
			chan |= 0xff;
			break;
		case 4:
			mod |= MOD_4_CH;
			chan |= 0xf;
			break;
		case 2:
			mod |= MOD_2_CH;
			chan |= 0x3;
			break;
		case 1:
			mod |= MOD_1_CH_C;
			chan |= 0x1;
			break;
		default:
			dev_err(i2s->dev, "%d channels not supported\n",
				params_channels(params));
			spin_unlock_irqrestore(&i2s->lock, flags);
			return -EINVAL;
		}
		switch (params_format(params)) {	/* 16bit or 8bit. */
		case SNDRV_PCM_FORMAT_S16_LE:
			mod |= MOD_WORD_LEN;
			current_wordlength = 16;
			break;
		case SNDRV_PCM_FORMAT_S8:

			current_wordlength = 8;
			mod &= ~MOD_WORD_LEN;
			break;
		default:
			dev_err(i2s->dev, "not supported data format %d\n",
				params_format(params));
			spin_unlock_irqrestore(&i2s->lock, flags);
			return -EINVAL;
		}

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
			dev_err(i2s->dev, "%d channels not supported\n",
				params_channels(params));
			spin_unlock_irqrestore(&i2s->lock, flags);
			return -EINVAL;
		}
		switch (params_format(params)) {	/* 16bit or 8bit. */
		case SNDRV_PCM_FORMAT_S16_LE:
			mod |= MOD_WORD_LEN;

			current_wordlength = 16;
			break;
		case SNDRV_PCM_FORMAT_S8:
			mod &= ~MOD_WORD_LEN;

			current_wordlength = 8;
			break;
		default:
			dev_err(i2s->dev, "not supported data format %d\n",
				params_format(params));
			spin_unlock_irqrestore(&i2s->lock, flags);
			return -EINVAL;
		}
		writel(mod, i2s->regaddr_tx + I2S_MODE);
		writel(chan, i2s->regaddr_tx + I2S_CH_EN);
	}

	x2_i2s_sample_rate_set(substream, i2s);
	spin_unlock_irqrestore(&i2s->lock, flags);
	return 0;
}

/* reset i2s by reset pin and sysctrl register */
static int i2s_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct x2_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned long flags, val;
	int spins = 500;

	spin_lock_irqsave(&i2s->lock, flags);

	reset_control_assert(i2s->rst);
	ndelay(100);
	reset_control_deassert(i2s->rst);

	/* reset i2s module by write sysctrl register */
	if (i2s->id == 0) {
		val = readl(i2s->sysctl_addr + I2S_SYS_RST);
		val |= 0x1 << (SYSCTL_I2S0RST_SHIFT);
		writel(val, i2s->sysctl_addr + I2S_SYS_RST);
		while (--spins)
			cpu_relax();

		val &= ~(0x1 << (SYSCTL_I2S0RST_SHIFT));
		writel(val, i2s->sysctl_addr + I2S_SYS_RST);
	} else {
		val = readl(i2s->sysctl_addr + I2S_SYS_RST);
		val |= 0x1 << (SYSCTL_I2S1RST_SHIFT);
		writel(val, i2s->sysctl_addr + I2S_SYS_RST);
		while (--spins)
			cpu_relax();

		val &= ~(0x1 << (SYSCTL_I2S1RST_SHIFT));
		writel(val, i2s->sysctl_addr + I2S_SYS_RST);
	}


	/* reset done, disable reset module */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		i2s->streamflag = 0;
	else
		i2s->streamflag = 1;

	spin_unlock_irqrestore(&i2s->lock, flags);

	return 0;
}

static int i2s_trigger(struct snd_pcm_substream *substream,
			int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct x2_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	unsigned long flags;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&i2s->lock, flags);
		i2s_transfer_ctl(i2s, 1);
		spin_unlock_irqrestore(&i2s->lock, flags);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&i2s->lock, flags);
		i2s_transfer_ctl(i2s, 0);
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
#if 0
	unsigned long flags;
	u32 val;

	/* struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai); */
	struct x2_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	spin_lock_irqsave(i2s->lock, flags);
	switch (div_id) {
	case I2S_DIV_BCLK_DIV_SEL:
		if (div > 0 && div <= 8) {
			val = readl(i2s->sysctl_addr + I2S0_CLKCTL_BASE);
			val &= ~(0x7 << BCLK_SHIFT);
			val |= div << BCLK_SHIFT;
			writel(val, i2s->sysctl_addr + I2S0_CLKCTL_BASE);
		}
		break;
	case I2S_MCLK_DIV_SEL:
		if (div > 0 && div <= 32) {
			val = readl(i2s->sysctl_addr + I2S0_CLKCTL_BASE);
			val &= ~(0x1f << MCLK_SHIFT);
			val |= div << MCLK_SHIFT;
			writel(val, i2s->sysctl_addr + I2S0_CLKCTL_BASE);
		}
		break;
	case I2S_PRE_MCLK_DIV_SEL:
		if (div > 0 && div <= 32) {
			val = readl(i2s->sysctl_addr + I2S0_CLKCTL_BASE);
			val &= ~(0x11 << PRE_MCLK_SHIFT);
			val |= div << PRE_MCLK_SHIFT;
			writel(val, i2s->sysctl_addr + I2S0_CLKCTL_BASE);
		}
		break;
	default:
		dev_err(i2s->dev, "We don't serve that!\n");
		spin_unlock_irqrestore(i2s->lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(i2s->lock, flags);
	return 0;

#endif
}

static const struct snd_soc_dai_ops x2_i2s_dai_ops = {
	.trigger = i2s_trigger,
	.hw_params = i2s_hw_params,
	.set_fmt = i2s_set_fmt,
	.set_clkdiv = i2s_set_clkdiv,
	.set_sysclk = i2s_set_sysclk,
	.startup = i2s_startup,

};

static int x2_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct x2_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &i2s->playback_dma_data,
				  &i2s->capture_dma_data);
	snd_soc_dai_set_drvdata(dai, i2s);

	return 0;
}

static int x2_i2s_dai_remove(struct snd_soc_dai *dai)
{
	return 0;
}

static struct snd_soc_dai_driver x2_i2s_dai_drv[2] = {

	{
		.probe = x2_i2s_dai_probe,
		.remove = x2_i2s_dai_remove,
		.playback = {
			     .stream_name = "Playback",
			     .channels_min = 1,
			     .channels_max = 2,
			     .rates = X2_I2S_RATES,
			     .formats = X2_I2S_FMTS,
			     },
		.capture = {
			    .stream_name = "Capture",
			    .channels_min = 1,
			    .channels_max = 16,
			    .rates = X2_I2S_RATES,
			    .formats = X2_I2S_FMTS,
			    },
		.ops = &x2_i2s_dai_ops,
		.symmetric_rates = 1,
		.name = "x2-i2s0",
	},
	{
		.probe = x2_i2s_dai_probe,/* the same as i2s0 dai param */
		.remove = x2_i2s_dai_remove,/* the same as i2s0 dai param */
		.playback = {
			     .stream_name = "Playback",
			     .channels_min = 1,
			     .channels_max = 2,
			     .rates = X2_I2S_RATES,
			     .formats = X2_I2S_FMTS,
			     },
		.capture = {
			    .stream_name = "Capture",
			    .channels_min = 1,
			    .channels_max = 16,
			    .rates = X2_I2S_RATES,
			    .formats = X2_I2S_FMTS,
			    },
		.ops = &x2_i2s_dai_ops,/* the same as i2s0 dai param */
		.symmetric_rates = 1,
		.name = "x2-i2s1",
	}
};
static const struct snd_soc_component_driver x2_i2s_component_drv[2];

static int x2_i2s_probe(struct platform_device *pdev)
{
	struct x2_i2s *i2s;
	int id;
	int ret;
	struct resource *res;
	/* struct snd_soc_dai_driver *soc_dai_driver; */
	id = of_alias_get_id(pdev->dev.of_node, "i2s");
	if (id < 0)
		id = 0;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s) {
		/* dev_err(&pdev->dev, "Can't allocate x2_i2s0\n"); */
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
		return PTR_ERR(i2s->regaddr_rx);
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
		return PTR_ERR(i2s->regaddr_tx);
	}

	res = NULL;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get mem resource2!\n");
		return -ENOENT;
	}

	i2s->sysctl_addr =
	    devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(i2s->sysctl_addr)) {

		dev_err(&pdev->dev, "Failed to ioremap sysctl_addr!\n");
		return PTR_ERR(i2s->sysctl_addr);
	}

	res = NULL;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get mem resource3!\n");
		return -ENOENT;
	}
	i2s->apb_regs = devm_ioremap(&pdev->dev,
		res->start, resource_size(res));
	if (IS_ERR(i2s->apb_regs)) {

		dev_err(&pdev->dev, "Failed to ioremap apb_regs!\n");
		return PTR_ERR(i2s->apb_regs);
	}

	i2s->rst = devm_reset_control_get(&pdev->dev, "i2s");
	if (IS_ERR(i2s->rst)) {
		dev_err(&pdev->dev, "Missing reset controller!\n");
		return PTR_ERR(i2s->rst);
	}

#if 0
	/* capture and play share the same dam config, */
	/* at the same time , only one action */
	i2s->playback_dma_data.addr = i2s0_dma_addr;
	/* i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES; */
	/* i2s->playback_dma_data.maxburst = 4; */
	i2s->capture_dma_data.addr = i2s0_dma_addr;
	/* i2s->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES; */
	/* i2s->capture_dma_data.maxburst = 4; */
	i2s->capture_dma_data.slave_id = 0;
	i2s->playback_dma_data.slave_id = 0;	/* this value is fixed */
#endif
	/* x2_i2s is set to cpudai dev data */
	dev_set_drvdata(&pdev->dev, i2s);

	writel(0x1, i2s->apb_regs);	/* enable apb */
	ret =
		devm_snd_soc_register_component(&pdev->dev,
			&x2_i2s_component_drv[id], &x2_i2s_dai_drv[id], 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI\n");
		return ret;
	}
	pr_err("success register cpu dai%d driver\n", id);
	return 0;

}

/* static const struct snd_soc_component_driver x2_i2s_component_drv = { */
/* .name = DRV_NAME, */
/* }; */
static int x2_i2s_remove(struct platform_device *pdev)
{
	/* struct x2_i2s *i2s = dev_get_drvdata (&pdev->dev); */
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id x2_i2s_of_match[] = {
	{.compatible = "hobot,x2-i2s0",},
	{.compatible = "hobot,x2-i2s1",},
	{}
};

MODULE_DEVICE_TABLE(of, x2_i2s_of_match);
#endif

static struct platform_driver x2_i2s_driver = {
	.probe = x2_i2s_probe,
	.remove = x2_i2s_remove,
	.driver = {
		   .name = "x2-i2s",
		   .of_match_table = x2_i2s_of_match,
		   },
};

module_platform_driver(x2_i2s_driver);

/* Module information */
MODULE_AUTHOR("Jxy");
MODULE_DESCRIPTION("X2 I2S Soc Interface");
MODULE_ALIAS("cpu dai driver:x2-i2s");
MODULE_LICENSE("GPL");
