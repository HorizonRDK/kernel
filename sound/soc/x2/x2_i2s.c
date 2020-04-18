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

#include "x2_i2s-regs.h"

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)


enum i2s_clk_type_e {
    I2S_DIV_BCLK_DIV_SEL,
    I2S_MCLK_DIV_SEL,
    I2S_PRE_MCLK_DIV_SEL,
};

enum x2_dai_type_e {
    TYPE_PRI,
    TYPE_SEC,
};

enum x2_i2s_type_e {
    TYPE_CAPTURE,
    TYPE_PALYBACK,
};

struct x2_i2s_dai_data_s {
    u32 quirks;
};

struct i2s_dai_s {
    /* Platform device for this DAI */
    struct platform_device *pdev;
    /* Memory mapped SFR region */
    void __iomem    *regaddr;
    void __iomem    *io_addr;
    void __iomem    *sysctl_addr;
    unsigned int mode;
    unsigned int id;
    /* I2S Controller's core clock */
    struct clk *clk;
    struct i2s_dai_s *x2_dai;
    unsigned int master;
    /* Driver for this DAI */
    struct snd_soc_dai_driver i2s_dai_drv;
    /* DMA parameters */
    struct snd_dmaengine_dai_dma_data dma_playback;
    struct snd_dmaengine_dai_dma_data dma_capture;
    dma_filter_fn filter;
    u32 quirks;

    /* Spinlock protecting access to the device's registers */
    spinlock_t spinlock;
    spinlock_t *lock;

    /* Below fields are only valid if this is the primary FIFO */
    struct clk *clk_table[3];
    struct clk_onecell_data clk_data;
};

/* Lock for cross i/f checks */
static DEFINE_SPINLOCK(lock);

static void i2s_transfer_ctl(struct i2s_dai_s *i2s, bool on)
{
    unsigned long val;

    val = readl(i2s->regaddr + I2SCTL);

    if (on)
        val |= CTL_ENABLE;
    else
        val &= ~CTL_ENABLE;
    writel(val, i2s->regaddr + I2SCTL);
}

static void i2s_set_master(struct i2s_dai_s *i2s, bool on)
{
    unsigned long val;

    val = readl(i2s->sysctl_addr + I2S_CLKCTL_BASE + i2s->id*4);
    if (on)
        val &= ~(0x1 << CLK_MODE);
    else
        val |= (0x1 << CLK_MODE);
    writel(val, i2s->sysctl_addr + I2S_CLKCTL_BASE + i2s->id*4);
}


static int i2s_set_sysclk(struct snd_soc_dai *dai,
      int clk_id, unsigned int rfs, int dir)
{
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    return 0;
}

static int i2s_set_fmt(struct snd_soc_dai *dai,
    unsigned int fmt)
{
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    u32 mod, val = 0;
    unsigned long flags;

    spin_lock_irqsave(i2s->lock, flags);
    val = readl(i2s->regaddr + I2SMOD);
    spin_unlock_irqrestore(i2s->lock, flags);
    /* Format is priority */
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_I2S:
        val |= (MOD_I2S_MODE);
        break;
    default:
        dev_err(&i2s->pdev->dev, "Format not supported\n");
        return -EINVAL;
    }

    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
    case SND_SOC_DAIFMT_CBM_CFM:
        /* Codec is master, and I2S is slave. */
        val |= MOD_MS_MODE;
        i2s->master = 0;
        i2s_set_master(dai,0);
        break;
    case SND_SOC_DAIFMT_CBS_CFS:
        val &= ~MOD_MS_MODE;
        i2s->master = 1;
        i2s_set_master(dai,1);
        break;
    default:
        dev_err(&i2s->pdev->dev, "master/slave format not supported\n");
        return -EINVAL;
    }

    spin_lock_irqsave(i2s->lock, flags);
    writel(val, i2s->regaddr + I2SMOD);
    spin_unlock_irqrestore(i2s->lock, flags);

    return 0;
}

static int i2s_hw_params(struct snd_pcm_substream *substream,
    struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    u32 mod, mask = 0, chen = 0;
    unsigned long flags;

    spin_lock_irqsave(i2s->lock, flags);
    mod = readl(i2s->regaddr + I2SMOD);
    spin_unlock_irqrestore(i2s->lock, flags);
    mod &= ~MOD_CH_NUM;
    switch (params_format(params)) {
    case SNDRV_PCM_FORMAT_S16_LE:
        mod |= MOD_WORD_LEN;
        break;
    case SNDRV_PCM_FORMAT_S8:
        mod &= ~MOD_WORD_LEN;
        break;
    default:
        dev_err(&i2s->pdev->dev, "not supported data format %d\n", params_format(params));
        return -EINVAL;
    }

    if(i2s->mode == TYPE_CAPTURE)
    {
        switch (params_channels(params)) {
        case 16:
            mod |= MOD_16_CH;
            chen |= 0xffff;
        case 8:
            mod |= MOD_8_CH;
            chen |= 0xff;
            break;
        case 4:
            mod |= MOD_4_CH;
            chen |= 0xf;
            break;
        case 2:
            mod |= MOD_2_CH;
            chen |= 0x3;
            break;
        case 1:
            mod |= MOD_1_CH_C;
            chen |= 0x1;
            break;
        default:
            dev_err(&i2s->pdev->dev, "%d channels not supported\n",
                    params_channels(params));
            return -EINVAL;
        }
    }
    else
    {
        switch (params_channels(params)) {
        case 2:
            mod |= MOD_2_CH;
            chen |= 0x3;
            break;
        case 1:
            mod |= MOD_1_CH_C;
            chen |= 0x1;
            break;
        default:
            dev_err(&i2s->pdev->dev, "%d channels not supported\n",
                    params_channels(params));
        }
    }

    spin_lock_irqsave(i2s->lock, flags);
    writel(mod, i2s->regaddr + I2SMOD);
    writel(chen, i2s->regaddr + I2SCHEN);
    spin_unlock_irqrestore(i2s->lock, flags);

    snd_soc_dai_init_dma_data(dai, &i2s->dma_playback, &i2s->dma_capture);

    return 0;
}

/* We set constraints on the substream acc to the version of I2S */
static int i2s_startup(struct snd_pcm_substream *substream,
      struct snd_soc_dai *dai)
{
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    unsigned long flags,val;

    spin_lock_irqsave(i2s->lock, flags);
    val = readl(i2s->sysctl_addr + I2S_SYS_RST);
    val |= 0x1 << (SYSCTL_SHIFT + i2s->id);
    writel(val, i2s->sysctl_addr + I2S_SYS_RST);
    uint spins = 500;
    while (--spins)
        cpu_relax();
    val &= ~(0x1 << (SYSCTL_SHIFT + i2s->id));
    writel(val, i2s->sysctl_addr + I2S_SYS_RST);

    spin_unlock_irqrestore(i2s->lock, flags);

    return 0;
}

static void i2s_shutdown(struct snd_pcm_substream *substream,
    struct snd_soc_dai *dai)
{
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    unsigned long flags;

    spin_lock_irqsave(i2s->lock, flags);

    spin_unlock_irqrestore(i2s->lock, flags);
}

static int i2s_trigger(struct snd_pcm_substream *substream,
    int cmd, struct snd_soc_dai *dai)
{
    int capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
    unsigned long flags;

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        spin_lock_irqsave(i2s->lock, flags);
        i2s_transfer_ctl(i2s, 1);
        spin_unlock_irqrestore(i2s->lock, flags);
        break;
    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        spin_lock_irqsave(i2s->lock, flags);

        i2s_transfer_ctl(i2s, 1);

        spin_unlock_irqrestore(i2s->lock, flags);
        break;
    }

    return 0;
}

static int i2s_set_clkdiv(struct snd_soc_dai *dai,
    int div_id, int div)
{
    unsigned long flags;
    u32 val;

    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    spin_lock_irqsave(i2s->lock, flags);
    switch(div_id)
    {
        case I2S_DIV_BCLK_DIV_SEL:
            if(div>0 && div<=8)
            {
                val = readl(i2s->sysctl_addr + I2S_CLKCTL_BASE + i2s->id*4);
                val &= ~(0x3 << BCLK_SHIFT);
                val |= div << BCLK_SHIFT;
                writel(val, i2s->sysctl_addr + I2S_CLKCTL_BASE + i2s->id*4);
            }
            break;
        case I2S_MCLK_DIV_SEL:
            if(div>0 && div<=32)
            {
                val = readl(i2s->sysctl_addr + I2S_CLKCTL_BASE + i2s->id*4);
                val &= ~(0x3 << MCLK_SHIFT);
                val |= div << MCLK_SHIFT;
                writel(val, i2s->sysctl_addr + I2S_CLKCTL_BASE + i2s->id*4);
            }
            break;
        case I2S_PRE_MCLK_DIV_SEL:
            if(div>0 && div<=32)
            {
                val = readl(i2s->sysctl_addr + I2S_CLKCTL_BASE + i2s->id*4);
                val &= ~(0x3 << PRE_MCLK_SHIFT);
                val |= div << PRE_MCLK_SHIFT;
                writel(val, i2s->sysctl_addr + I2S_CLKCTL_BASE + i2s->id*4);
            }
            break;
        default:
            dev_err(&i2s->pdev->dev, "We don't serve that!\n");
            spin_unlock_irqrestore(i2s->lock, flags);
            return -EINVAL;
    }
    spin_unlock_irqrestore(i2s->lock, flags);
    return 0;
}

#ifdef CONFIG_PM
static int i2s_suspend(struct snd_soc_dai *dai)
{
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    return 0;
}

static int i2s_resume(struct snd_soc_dai *dai)
{
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    return 0;
}
#else
#define i2s_suspend NULL
#define i2s_resume  NULL
#endif

static int x2_i2s_dai_probe(struct snd_soc_dai *dai)
{
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    unsigned long flags,val;

    snd_soc_dai_init_dma_data(dai, &i2s->dma_playback,
                   &i2s->dma_capture);
    spin_lock_irqsave(i2s->lock, flags);

    spin_unlock_irqrestore(i2s->lock, flags);


    return 0;
}

static int x2_i2s_dai_remove(struct snd_soc_dai *dai)
{
    struct i2s_dai_s *i2s = snd_soc_dai_get_drvdata(dai);
    return 0;
}

static const struct snd_soc_dai_ops x2_i2s_dai_ops = {
    .trigger = i2s_trigger,
    .hw_params = i2s_hw_params,
    .set_fmt = i2s_set_fmt,
    .set_clkdiv = i2s_set_clkdiv,
    .set_sysclk = i2s_set_sysclk,
    .startup = i2s_startup,
    .shutdown = i2s_shutdown,
};

static const struct snd_soc_component_driver *x2_i2s_component[2];
//= {
//  .name       = "x2-i2s",
//};

#define X2_I2S_RATES    SNDRV_PCM_RATE_8000_96000

#define X2_I2S_FMTS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE)

static struct i2s_dai_s *i2s_alloc_dai(struct platform_device *pdev, bool sec)
{
    struct i2s_dai_s *i2s;
    int ret;

    i2s = devm_kzalloc(&pdev->dev, sizeof(struct i2s_dai_s), GFP_KERNEL);
    if (i2s == NULL)
        return NULL;

    i2s->pdev = pdev;
    i2s->x2_dai = NULL;
    i2s->i2s_dai_drv.symmetric_rates = 1;
    i2s->i2s_dai_drv.probe = x2_i2s_dai_probe;
    i2s->i2s_dai_drv.remove = x2_i2s_dai_remove;
    i2s->i2s_dai_drv.ops = &x2_i2s_dai_ops;
    i2s->i2s_dai_drv.suspend = i2s_suspend;
    i2s->i2s_dai_drv.resume = i2s_resume;
    i2s->i2s_dai_drv.playback.channels_min = 1;
    i2s->i2s_dai_drv.playback.channels_max = 16;
    i2s->i2s_dai_drv.playback.rates = X2_I2S_RATES;
    i2s->i2s_dai_drv.playback.formats = X2_I2S_FMTS;
    i2s->i2s_dai_drv.capture.channels_min = 1;
    i2s->i2s_dai_drv.capture.channels_max = 16;
    i2s->i2s_dai_drv.capture.rates = X2_I2S_RATES;
    i2s->i2s_dai_drv.capture.formats = X2_I2S_FMTS;
    dev_set_drvdata(&i2s->pdev->dev, i2s);


    return i2s;
}

static void i2s_unregister_clocks(struct i2s_dai_s *i2s)
{
    int i;

    for (i = 0; i < i2s->clk_data.clk_num; i++) {
        if (!IS_ERR(i2s->clk_table[i]))
            clk_unregister(i2s->clk_table[i]);
    }
}

static void i2s_unregister_clock_provider(struct platform_device *pdev)
{
    struct i2s_dai_s *i2s = dev_get_drvdata(&pdev->dev);

    of_clk_del_provider(pdev->dev.of_node);
    i2s_unregister_clocks(i2s);
}

static int i2s_register_clock_provider(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct i2s_dai_s *i2s = dev_get_drvdata(dev);
    struct clk *rclksrc;
    int ret, i;

    if (ret < 0) {
        dev_err(dev, "failed to add clock provider: %d\n", ret);
        i2s_unregister_clocks(i2s);
    }

    return ret;
}

static int x2_i2s_probe(struct platform_device *pdev)
{
    struct i2s_dai_s *x2_dai = NULL;
    struct resource *res;
    u32 regs_base, quirks = 0, idma_addr = 0;
    struct snd_soc_component_driver *i2s_component;
    const struct x2_i2s_dai_data_s *i2s_dai_data;
    int ret,id;

    x2_dai = i2s_alloc_dai(pdev, false);
    if (!x2_dai) {
        dev_err(&pdev->dev, "Unable to alloc I2S_DAI\n");
        return -ENOMEM;
    }
    id = of_alias_get_id(pdev->dev.of_node, "x2-i2s");
    x2_dai->id = id;
    spin_lock_init(&x2_dai->spinlock);
    x2_dai->lock = &x2_dai->spinlock;


    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    x2_dai->regaddr = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(x2_dai->regaddr))
        return PTR_ERR(x2_dai->regaddr);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    x2_dai->io_addr = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(x2_dai->io_addr))
        return PTR_ERR(x2_dai->io_addr);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
    x2_dai->sysctl_addr = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(x2_dai->sysctl_addr))
        return PTR_ERR(x2_dai->sysctl_addr);

    i2s_component = devm_kzalloc(&pdev->dev, sizeof(struct snd_soc_component_driver), GFP_KERNEL);
    sprintf(i2s_component->name, "x2_i2s%d", id);
    x2_i2s_component[id] = i2s_component;
    /*
    x2_dai->clk = devm_clk_get(&pdev->dev, "iis");
    if (IS_ERR(x2_dai->clk)) {
        dev_err(&pdev->dev, "Failed to get iis clock\n");
        return PTR_ERR(x2_dai->clk);
    }

    ret = clk_prepare_enable(x2_dai->clk);
    if (ret != 0) {
        dev_err(&pdev->dev, "failed to enable clock: %d\n", ret);
        return ret;
    }
    */
    x2_dai->dma_playback.slave_id = id;
    x2_dai->dma_capture.slave_id = id;
    if (ret < 0)
        goto err_disable_clk;

    ret = devm_snd_soc_register_component(&x2_dai->pdev->dev,
                    i2s_component,
                    &x2_dai->i2s_dai_drv, 1);
    if (ret < 0)
        goto err_free_dai;

    //ret = i2s_register_clock_provider(pdev);
    //if (!ret)
    //  return 0;

err_free_dai:
err_disable_clk:
    //clk_disable_unprepare(x2_dai->clk);
    return ret;
}

static int x2_i2s_remove(struct platform_device *pdev)
{
    struct i2s_dai_s *i2s;

    i2s = dev_get_drvdata(&pdev->dev);
    i2s->x2_dai = NULL;

    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id x2_i2s_of_match[] = {
    { .compatible = "hobot,x2-i2s" },
    {},
};
MODULE_DEVICE_TABLE(of, x2_i2s_of_match);
#endif


static struct platform_driver x2_i2s_driver = {
    .probe  = x2_i2s_probe,
    .remove = x2_i2s_remove,
    .driver = {
        .name = "x2-i2s",
        .of_match_table = of_match_ptr(x2_i2s_of_match),
        //.pm = &x2_i2s_pm,
    },
};

module_platform_driver(x2_i2s_driver);

/* Module information */
MODULE_DESCRIPTION("X2 I2S Interface");
MODULE_ALIAS("platform:x2-i2s");
MODULE_LICENSE("GPL");
