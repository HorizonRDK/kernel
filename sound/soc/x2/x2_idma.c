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

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "x2_i2s-regs.h"

#define ST_RUNNING      (1<<0)
#define ST_OPENED       (1<<1)

#define DMA_START   1
#define DMA_STOP    0


#define MAX_IDMA_PERIOD (80 * 1024)
#define MAX_IDMA_BUFFER (160 * 1024)


static const struct snd_pcm_hardware idma_hardware = {
    .info = SNDRV_PCM_INFO_INTERLEAVED |
            SNDRV_PCM_INFO_BLOCK_TRANSFER |
            SNDRV_PCM_INFO_MMAP |
            SNDRV_PCM_INFO_MMAP_VALID |
            SNDRV_PCM_INFO_PAUSE |
            SNDRV_PCM_INFO_RESUME,
    .buffer_bytes_max = MAX_IDMA_BUFFER,
    .period_bytes_min = 128,
    .period_bytes_max = MAX_IDMA_PERIOD,
    .periods_min = 1,
    .periods_max = 2,
};

struct idma_ctrl_s {
    spinlock_t  lock;
    unsigned int dma_ch;
    int     state;
    dma_addr_t  start;
    dma_addr_t  pos;
    dma_addr_t  end;
    dma_addr_t  lastset;
    dma_addr_t  period;
    dma_addr_t  periodsz;
    void        *token;
    void        (*cb)(void *dt, int bytes_xfer);
};

static struct idma_info_s {
    spinlock_t  lock;
    void         __iomem  *regs;
    dma_addr_t  bufaddr;
    int idma_irq;
    int ch_num;
} x2_idma[2];

static struct snd_dmaengine_dai_dma_data *x2_dai_get_dma_data(
                struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;

    return snd_soc_dai_get_dma_data(soc_runtime->cpu_dai, substream);
}

static void idma_getpos(dma_addr_t *src, unsigned int dmachannel)
{
    *src = readl(x2_idma[dmachannel].regs + I2SCURADDR);
}

static int idma_enqueue(struct snd_pcm_substream *substream, unsigned int dmachannel)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct idma_ctrl_s *prtd = substream->runtime->private_data;
    u32 val;

    spin_lock(&prtd->lock);
    prtd->token = (void *) substream;
    spin_unlock(&prtd->lock);

    //set buf0 ready
    val = prtd->lastset + prtd->periodsz;
    writel(val, x2_idma[dmachannel].regs + I2SB0ADDR);
    writel(0x1, x2_idma[dmachannel].regs + I2SB0RDY);
    prtd->lastset = prtd->start + prtd->periodsz;

    //set buf1 ready
    val = prtd->lastset + prtd->periodsz;
    writel(val, x2_idma[dmachannel].regs + I2SB1ADDR);
    writel(0x1, x2_idma[dmachannel].regs + I2SB1RDY);

    //set each channl buf size for dma
    val = prtd->periodsz/x2_idma[dmachannel].ch_num;
    writel(val, x2_idma[dmachannel].regs + I2SBSIZE);
    prtd->lastset = prtd->start + prtd->periodsz;

    return 0;
}

static void idma_setcallbk(struct snd_pcm_substream *substream,
                void (*cb)(void *, int))
{
    struct idma_ctrl_s *prtd = substream->runtime->private_data;

    spin_lock(&prtd->lock);
    prtd->cb = cb;
    spin_unlock(&prtd->lock);
}

static void idma_control(int op, unsigned int dmachannel)
{
    u32 val = readl(x2_idma[dmachannel].regs + I2SCTL);

    spin_lock(&x2_idma[dmachannel].lock);

    switch (op) {
    case DMA_START:
        val |= CTL_IMEM_EN;
        val |= CTL_IMEM_CLR;
        break;
    case DMA_STOP:
        val &= ~CTL_IMEM_EN;
        break;
    default:
        spin_unlock(&x2_idma[dmachannel].lock);
        return;
    }

    writel(val, x2_idma[dmachannel].regs + I2SCTL);
    spin_unlock(&x2_idma[dmachannel].lock);
}

static void idma_done(void *id, int bytes_xfer)
{
    struct snd_pcm_substream *substream = id;
    struct idma_ctrl_s *prtd = substream->runtime->private_data;

    if (prtd && (prtd->state & ST_RUNNING))
        snd_pcm_period_elapsed(substream);
}

static int idma_hw_params(struct snd_pcm_substream *substream,
                struct snd_pcm_hw_params *params)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct idma_ctrl_s *prtd = substream->runtime->private_data;

    writel(0xf, x2_idma[prtd->dma_ch].regs + I2SINTUNMASK);//enable irq

    snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
    runtime->dma_bytes = params_buffer_bytes(params);

    prtd->start = prtd->pos = runtime->dma_addr;
    prtd->period = params_periods(params);
    prtd->periodsz = params_period_bytes(params);
    prtd->end = runtime->dma_addr + runtime->dma_bytes;
    prtd->lastset = prtd->start;

    idma_setcallbk(substream, idma_done);

    return 0;
}

static int idma_hw_free(struct snd_pcm_substream *substream)
{
    snd_pcm_set_runtime_buffer(substream, NULL);

    return 0;
}

static int idma_prepare(struct snd_pcm_substream *substream)
{
    struct idma_ctrl_s *prtd = substream->runtime->private_data;
    prtd->pos = prtd->start;

    /* flush the DMA channel */
    idma_control(DMA_STOP, prtd->dma_ch);
    idma_enqueue(substream, prtd->dma_ch);

    return 0;
}

static int idma_trigger(struct snd_pcm_substream *substream, int cmd)
{
    struct idma_ctrl_s *prtd = substream->runtime->private_data;
    int ret = 0;

    spin_lock(&prtd->lock);

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        prtd->state |= ST_RUNNING;
        idma_control(DMA_START,prtd->dma_ch);
        break;

    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        prtd->state &= ~ST_RUNNING;
        idma_control(DMA_STOP,prtd->dma_ch);
        break;

    default:
        ret = -EINVAL;
        break;
    }

    spin_unlock(&prtd->lock);

    return ret;
}

static snd_pcm_uframes_t idma_pointer(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct idma_ctrl_s *prtd = runtime->private_data;
    dma_addr_t src;
    unsigned long res;
    spin_lock(&prtd->lock);

    idma_getpos(&src, prtd->dma_ch);
    res = src - prtd->start;

    spin_unlock(&prtd->lock);

    return bytes_to_frames(substream->runtime, res);
}

static int idma_mmap(struct snd_pcm_substream *substream,
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

static irqreturn_t iis_irq(int irqno, void *dev_id)
{
    struct idma_ctrl_s *prtd = (struct idma_ctrl_s *)dev_id;
    u32 intstatus, addr,val;

    intstatus = readl(x2_idma[prtd->dma_ch].regs + I2SSRCPND);

    if (intstatus & INT_BUF0_DONE) {
        writel(INT_BUF0_DONE, x2_idma[prtd->dma_ch].regs + I2SSRCPND);//clear int

        addr = prtd->lastset + prtd->periodsz - prtd->start;
        addr %= (u32)(prtd->end - prtd->start);
        addr += prtd->start;
        prtd->lastset = addr;

        writel(addr, x2_idma[prtd->dma_ch].regs + I2SB0ADDR);
        writel(0x1, x2_idma[prtd->dma_ch].regs + I2SB0RDY);

        if (prtd->cb)
            prtd->cb(prtd->token, prtd->period);
    }
    if (intstatus & INT_BUF1_DONE) {
        writel(INT_BUF0_DONE, x2_idma[prtd->dma_ch].regs + I2SSRCPND);//clear int

        addr = prtd->lastset + prtd->lastset - prtd->start;
        addr %= (u32)(prtd->end - prtd->start);
        addr += prtd->start;
        prtd->lastset = addr;

        writel(addr, x2_idma[prtd->dma_ch].regs + I2SB1ADDR);
        writel(0x1, x2_idma[prtd->dma_ch].regs + I2SB1RDY);

        if (prtd->cb)
            prtd->cb(prtd->token, prtd->period);
    }

    return IRQ_HANDLED;
}

static int idma_open(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct idma_ctrl_s *prtd;
    int ret;
    struct snd_dmaengine_dai_dma_data *dma_data;
    dma_data = x2_dai_get_dma_data(substream);
    snd_soc_set_runtime_hwparams(substream, &idma_hardware);

    prtd = kzalloc(sizeof(struct idma_ctrl_s), GFP_KERNEL);
    if (prtd == NULL)
        return -ENOMEM;
    prtd->dma_ch = dma_data->slave_id;

    ret = request_irq(x2_idma[prtd->dma_ch].idma_irq, iis_irq, 0, "i2s", prtd);
    if (ret < 0) {
        pr_err("fail to claim i2s irq , ret = %d\n", ret);
        kfree(prtd);
        return ret;
    }

    spin_lock_init(&prtd->lock);

    runtime->private_data = prtd;

    return 0;
}

static int idma_close(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct idma_ctrl_s *prtd = runtime->private_data;

    free_irq(x2_idma[prtd->dma_ch].idma_irq, prtd);

    if (!prtd)
        pr_err("idma_close called with prtd == NULL\n");

    kfree(prtd);

    return 0;
}

static struct snd_pcm_ops idma_ops = {
    .open       = idma_open,
    .close      = idma_close,
    .ioctl      = snd_pcm_lib_ioctl,
    .trigger    = idma_trigger,
    .pointer    = idma_pointer,
    .mmap       = idma_mmap,
    .hw_params  = idma_hw_params,
    .hw_free    = idma_hw_free,
    .prepare    = idma_prepare,
};

static void idma_free(struct snd_pcm *pcm)
{
    struct snd_pcm_substream *substream;
    struct snd_dma_buffer *buf;

    substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
    if (!substream)
        return;

    buf = &substream->dma_buffer;
    if (!buf->area)
        return;

    iounmap((void __iomem *)buf->area);

    buf->area = NULL;
    buf->addr = 0;
}

static int preallocate_idma_buffer(struct snd_pcm *pcm, int stream)
{
    struct snd_pcm_substream *substream = pcm->streams[stream].substream;
    struct snd_dma_buffer *buf = &substream->dma_buffer;
    struct snd_dmaengine_dai_dma_data *dma_data;
    dma_data = x2_dai_get_dma_data(substream);
    buf->dev.dev = pcm->card->dev;
    buf->private_data = NULL;

    /* Assign PCM buffer pointers */
    buf->dev.type = SNDRV_DMA_TYPE_CONTINUOUS;
    buf->addr = x2_idma[dma_data->slave_id].bufaddr;
    buf->bytes = idma_hardware.buffer_bytes_max;
    buf->area = (unsigned char * __force)ioremap(buf->addr, buf->bytes);

    return 0;
}

static int idma_new(struct snd_soc_pcm_runtime *rtd)
{
    struct snd_card *card = rtd->card->snd_card;
    struct snd_pcm *pcm = rtd->pcm;
    int ret;

    ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
    if (ret)
        return ret;

    if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
        ret = preallocate_idma_buffer(pcm,
                SNDRV_PCM_STREAM_PLAYBACK);
    }

    return ret;
}

void idma_reg_addr_init(void __iomem *regs, dma_addr_t addr, int dmachannel)
{
    spin_lock_init(&x2_idma[dmachannel].lock);
    x2_idma[dmachannel].regs = regs;
    x2_idma[dmachannel].bufaddr = addr;
}
EXPORT_SYMBOL_GPL(idma_reg_addr_init);

static struct snd_soc_platform_driver asoc_x2idma_platform = {
    .ops = &idma_ops,
    .pcm_new = idma_new,
    .pcm_free = idma_free,
};

static int asoc_idma_platform_probe(struct platform_device *pdev)
{
    unsigned int id;
    struct resource *mem;
    id = of_alias_get_id(pdev->dev.of_node, "x2-i2s-idma");
    x2_idma[id].idma_irq =platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (x2_idma[id].idma_irq < 0)
        return x2_idma[id].idma_irq;
    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);//register resource
    x2_idma[id].regs = devm_ioremap_resource(&pdev->dev, mem);

    mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);//dma resource
    x2_idma[id].bufaddr = mem->start;

    return devm_snd_soc_register_platform(&pdev->dev, &asoc_x2idma_platform);
}

static struct platform_driver i2s_idma_driver = {
    .driver = {
        .name = "x2-i2s-idma",
    },

    .probe = asoc_idma_platform_probe,
};

module_platform_driver(i2s_idma_driver);


MODULE_DESCRIPTION("X2 ASoC IDMA Driver");
MODULE_LICENSE("GPL");
