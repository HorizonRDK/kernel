/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
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

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "x2-i2s.h"

#define ST_RUNNING      (1<<0)
#define ST_OPENED       (1<<1)

#define DMA_START   1
#define DMA_STOP    0

#define MAX_IDMA_PERIOD (80 * 1024)
#define MAX_IDMA_BUFFER (160 * 1024)
static unsigned char *dma_vm;
static unsigned long debug_flag;

/* play and capture have the same hardware param, so no neeed split two parts */
static const struct snd_pcm_hardware i2sidma_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
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
	int id;
};

static struct idma_info_s {
	spinlock_t lock;
	void __iomem *regaddr_tx;
	void __iomem *regaddr_rx;

	int idma_irq;
} x2_i2sidma[2];

static struct snd_dmaengine_dai_dma_data *x2_dai_get_dma_data(struct
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

	spin_lock(&dma_ctrl->lock);
	dma_ctrl->token = (void *)substream;
	spin_unlock(&dma_ctrl->lock);

	/* set buf0 ready */
	val = dma_ctrl->start;
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		writel(val, x2_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_BUF0_ADDR);
		writel(val + dma_ctrl->periodsz,
			x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_BUF1_ADDR);

		val = (dma_ctrl->periodsz) / (dma_ctrl->ch_num);
		writel(val, x2_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_BUF_SIZE);


	} else {

		writel(val, x2_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_BUF0_ADDR);
		writel(val + dma_ctrl->periodsz,
			x2_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_BUF1_ADDR);
		val = (dma_ctrl->periodsz) / (dma_ctrl->ch_num);
		writel(val, x2_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_BUF_SIZE);

	}

	return 0;
}

static void i2sidma_setcallbk(struct snd_pcm_substream *substream,
			       void (*cb)(void *, int))
{
	struct idma_ctrl_s *dma_ctrl = substream->runtime->private_data;

	spin_lock(&dma_ctrl->lock);
	dma_ctrl->cb = cb;
	spin_unlock(&dma_ctrl->lock);
}

static void i2sidma_control(int op, int stream, struct idma_ctrl_s *dma_ctrl)
{
	u32 val = 0;



		if (stream == SNDRV_PCM_STREAM_CAPTURE)
			val = readl(x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_CTL);
		else
			val = readl(x2_i2sidma[dma_ctrl->id].regaddr_tx +
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
			writel(val, x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_CTL);
		else
			writel(val, x2_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_CTL);



}

static void i2sidma_done(void *id, int bytes_xfer)
{
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

	/* update runtime dma info from substream dma info */
	/* substream dma info if getted at idma_open */
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	runtime->dma_bytes = params_buffer_bytes(params);

	/* init dma buffer addr */
	dma_ctrl->start = dma_ctrl->pos = runtime->dma_addr;
	dma_ctrl->period = params_periods(params);
	dma_ctrl->periodsz = params_period_bytes(params);//per frame bytes
	dma_ctrl->end = runtime->dma_addr + runtime->dma_bytes;
	dma_ctrl->lastset = dma_ctrl->start;
	dma_ctrl->ch_num = params_channels(params);
	dma_ctrl->bytesnum = runtime->dma_bytes;//total bytes
	pr_err("dma_ctrl->period is %d, dma_ctrl->periodsz is %d,dma_ctrl->bytesnum is %d\n", dma_ctrl->period, dma_ctrl->periodsz, dma_ctrl->bytesnum);
	pr_err("dma_ctrl->start is 0x%x,dma_ctrl->end is 0x%x\n", dma_ctrl->start, dma_ctrl->end);

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
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		writel(0xf, x2_i2sidma[dma_ctrl->id].regaddr_rx + I2S_UNMASK);
		reg_val = readl(x2_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_SETMASK);
		writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF_FLOW_BIT,
			I2S_INT_BUF_FLOW_FIELD),
			x2_i2sidma[dma_ctrl->id].regaddr_rx + I2S_SETMASK);


		reg_val = readl(x2_i2sidma[dma_ctrl->id].regaddr_rx +
			I2S_SETMASK);
		writel(UPDATE_VALUE_FIELD
			(reg_val, 0x1, I2S_INT_BUF_NOT_READY_BIT,
				I2S_INT_BUF_NOT_READY_FIELD),
				x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_SETMASK);

		writel(0xF, x2_i2sidma[dma_ctrl->id].regaddr_rx + I2S_SRCPND);


	} else {/* play */
		writel(0xf, x2_i2sidma[dma_ctrl->id].regaddr_tx + I2S_UNMASK);
		reg_val = readl(x2_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_SETMASK);
		writel(UPDATE_VALUE_FIELD(reg_val, 0x1, I2S_INT_BUF_FLOW_BIT,
			I2S_INT_BUF_FLOW_FIELD),
			x2_i2sidma[dma_ctrl->id].regaddr_tx + I2S_SETMASK);


		reg_val = readl(x2_i2sidma[dma_ctrl->id].regaddr_tx +
			I2S_SETMASK);
		writel(UPDATE_VALUE_FIELD
				(reg_val, 0x1, I2S_INT_BUF_NOT_READY_BIT,
				I2S_INT_BUF_NOT_READY_FIELD),
				x2_i2sidma[dma_ctrl->id].regaddr_tx +
				I2S_SETMASK);

		writel(0xF, x2_i2sidma[dma_ctrl->id].regaddr_tx + I2S_SRCPND);

	}

	return 0;
}

static int i2sidma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct idma_ctrl_s *dma_ctr = substream->runtime->private_data;
	int ret = 0;

	spin_lock(&dma_ctr->lock);
	/* pr_err("i2sidma_trigger cmd is %d\n", cmd); */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dma_ctr->state |= ST_RUNNING;
		/* enable iram, clear iram */
		i2sidma_control(DMA_START, substream->stream, dma_ctr);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dma_ctr->state &= ~ST_RUNNING;
		i2sidma_control(DMA_STOP, substream->stream, dma_ctr);
		writel(0x0, x2_i2sidma[dma_ctr->id].regaddr_rx + I2S_UNMASK);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pr_info("start i2sidma trigger\n");
	spin_unlock(&dma_ctr->lock);
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

	spin_lock(&dma_ctrl->lock);

	idma_getpos(&src, substream->stream, snd_card, dma_ctrl);
	/* pr_err("i2sidma_pointer, get pos is %llx\n", src); */
	res = src - dma_ctrl->start;

	spin_unlock(&dma_ctrl->lock);

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


static irqreturn_t iis_irq0(int irqno, void *dev_id)
{
	struct idma_ctrl_s *dma_ctrl = (struct idma_ctrl_s *)dev_id;
	u32 intstatus;
	u32	addr = 0;
	dma_addr_t halfullsize;

	halfullsize = (dma_ctrl->periodsz * dma_ctrl->period/2);
	if (dma_ctrl->stream == SNDRV_PCM_STREAM_CAPTURE) {

		intstatus = readl(x2_i2sidma[0].regaddr_rx + I2S_SRCPND);
		if (debug_flag == 1)
			pr_err("intstatus = 0x%x\n", intstatus);


		if (intstatus & INT_BUF0_DONE) {

			writel(0x4, x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_SRCPND);

			if ((dma_ctrl->lastset + dma_ctrl->periodsz -
					dma_ctrl->start)
				/ dma_ctrl->periodsz == 1) {
				if (debug_flag == 1)
					pr_err("index 0 irq,the head four bystes is 0x%x,0x%x,0x%x,0x%x\n", *dma_vm, *(dma_vm+1), *(dma_vm+2), *(dma_vm+3));
				addr += dma_ctrl->lastset + halfullsize;
				dma_ctrl->lastset = dma_ctrl->start +
					dma_ctrl->periodsz;

			} else if ((dma_ctrl->lastset + dma_ctrl->periodsz -
					dma_ctrl->start)
				/ dma_ctrl->periodsz == 3){
				addr = dma_ctrl->start;
				if (debug_flag == 1)
					pr_err("index 2 irq,the head four bystes is 0x%x,0x%x,0x%x,0x%x\n", *(dma_vm+8192), *(dma_vm+8192+1), *(dma_vm+8192+2), *(dma_vm+8192+3));
				dma_ctrl->lastset = dma_ctrl->start +
				halfullsize + dma_ctrl->periodsz;

			} else {
				pr_err("ERROR IN BUFFER0!,buffer come order error!\n");
				return IRQ_HANDLED;
			}
			if (dma_ctrl->cb)
				dma_ctrl->cb(dma_ctrl->token, dma_ctrl->period);


			//printk("after callback,the corrent pos is
			//0x%x\n",dma_ctrl->lastset);
			writel(addr, x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_BUF0_ADDR);
			writel(0x1, x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_BUF0_RDY);


		}
		if (intstatus & INT_BUF1_DONE) {

			writel(0x8, x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_SRCPND);

			if ((dma_ctrl->lastset + dma_ctrl->periodsz -
				dma_ctrl->start)
				/dma_ctrl->periodsz == 2) {

				if (debug_flag == 1)
					pr_err("index 1 irq,the head four bystes is 0x%x,0x%x,0x%x,0x%x\n", *(dma_vm+4096), *(dma_vm+4096+1), *(dma_vm+4096+2), *(dma_vm+4096+3));
				addr += dma_ctrl->lastset + halfullsize;
				dma_ctrl->lastset = dma_ctrl->start +
				halfullsize;

			} else if ((dma_ctrl->lastset + dma_ctrl->periodsz -
				dma_ctrl->start)
				/dma_ctrl->periodsz == 4){
				if (debug_flag == 1)
					pr_err("index 3 irq,the head four bystes is 0x%x,0x%x,0x%x,0x%x\n", *(dma_vm+12288), *(dma_vm+12288+1), *(dma_vm+12288+2), *(dma_vm+12288+3));
				addr = dma_ctrl->start + dma_ctrl->periodsz;
			     dma_ctrl->lastset = dma_ctrl->start;

			}

			else {
				pr_err("ERROR IN BUFFER1!,buffer come order error!\n");
				return IRQ_HANDLED;
			}

			if (dma_ctrl->cb)
				dma_ctrl->cb(dma_ctrl->token, dma_ctrl->period);

			//printk("after callback,the corrent pos is 0x%x\n",dma_ctrl->lastset);
			writel(addr, x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_BUF1_ADDR);
			writel(0x1, x2_i2sidma[dma_ctrl->id].regaddr_rx +
				I2S_BUF1_RDY);

			}
		//if (intstatus & INT_OVERFLOW) {
		//	}
		//if (intstatus & INT_NOTREADY) {
		//	}

		//printk("dma_ctrl->lastset is 0x%x\n",dma_ctrl->lastset);
	}

	return IRQ_HANDLED;
}
static irqreturn_t iis_irq1(int irqno, void *dev_id)
{

	return IRQ_HANDLED;
}

/* get dam irq, set runtime hw limit */
static int i2sidma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *snd_card = rtd->card;

	struct idma_ctrl_s *dma_ctrl;
	struct snd_dmaengine_dai_dma_data *dma_data;
	int ret;

	dma_data = x2_dai_get_dma_data(substream);

	dma_ctrl = kzalloc(sizeof(struct idma_ctrl_s), GFP_KERNEL);
	if (dma_ctrl == NULL)
		return -ENOMEM;
	/* dma_ctrl->dma_id = dma_data->slave_id; */
	/* set limit hw param to runtime */
	snd_soc_set_runtime_hwparams(substream, &i2sidma_hardware);

	if (!strcmp(snd_card->name, "x2snd0")) {
		dma_ctrl->id = 0;
		ret = request_irq(x2_i2sidma[dma_ctrl->id].idma_irq, iis_irq0,
			0, "idma0", dma_ctrl);
		if (ret < 0) {
			pr_err("fail to claim i2s irq , ret = %d\n", ret);
			kfree(dma_ctrl);
			return ret;
		}

	}
	if (!strcmp(snd_card->name, "x2snd1")) {
		dma_ctrl->id = 1;
		ret = request_irq(x2_i2sidma[dma_ctrl->id].idma_irq, iis_irq1,
			0, "idma1", dma_ctrl);
		if (ret < 0) {
			pr_err("fail to claim i2s irq , ret = %d\n", ret);
			kfree(dma_ctrl);
			return ret;
		}
	}

	dma_ctrl->stream = substream->stream;

	spin_lock_init(&dma_ctrl->lock);

	/* this critical action */
	runtime->private_data = dma_ctrl;

	return 0;
}

static int i2sidma_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl_s *dma_ctrl = runtime->private_data;

	free_irq(x2_i2sidma[dma_ctrl->id].idma_irq, dma_ctrl);
	kfree(dma_ctrl);

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

		dmam_free_coherent(pcm->card->dev, buf->bytes, buf->area,
				   buf->addr);

		buf->area = NULL;
		buf->addr = 0;
	}

}

/* update dma info to substream->dma_buffer by hw design */
/* the ioremap equivalently alloc buffer */
static int preallocate_idma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct snd_dmaengine_dai_dma_data *dma_data;
	dma_addr_t phy;

	/* now not use dma data */
	dma_data = x2_dai_get_dma_data(substream);
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	/* Assign PCM buffer pointers */
	buf->dev.type = SNDRV_DMA_TYPE_CONTINUOUS;



	/* No matter play or capture */
	buf->area = dmam_alloc_coherent(pcm->card->dev,
			i2sidma_hardware.buffer_bytes_max, &phy, GFP_KERNEL);
	dma_vm = buf->area;
	buf->addr = phy;
	buf->bytes = i2sidma_hardware.buffer_bytes_max;	/* 160K */

	if (buf->area == NULL)
		return -ENOMEM;
	return 0;
}

static int i2sidma_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream)
		ret = preallocate_idma_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream)
		ret = preallocate_idma_buffer(pcm, SNDRV_PCM_STREAM_CAPTURE);

	return ret;
}

static struct snd_soc_platform_driver asoc_i2sidma_platform[2] = {

	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	},
	{
		.ops = &i2sidma_ops,
		.pcm_new = i2sidma_new,
		.pcm_free = i2sidma_free,
	}
};
static ssize_t show_debug(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "debug flag is %ld\n", debug_flag);
	if (s != buf)
		*(s-1) = '\n';
	return s-buf;
}

static ssize_t store_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	ret = kstrtoul(buf, 10, &debug_flag);
	if (ret)
		return ret;

	return count;

}
static DEVICE_ATTR(debug, 0644, show_debug, store_debug);

static int asoc_i2sidma_platform_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	int id;

	id = of_alias_get_id(pdev->dev.of_node, "idma");
	if (id < 0)
		id = 0;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get mem resource0!\n");
		return -ENOENT;
	}
	x2_i2sidma[id].regaddr_rx =
		devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(x2_i2sidma[id].regaddr_rx)) {
		dev_err(&pdev->dev, "Failed to ioremap regaddr_rx!\n");
		return PTR_ERR(x2_i2sidma[id].regaddr_rx);
	}
	res = NULL;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get mem resource1!\n");
		return -ENOENT;
	}
	x2_i2sidma[id].regaddr_tx =
		devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(x2_i2sidma[id].regaddr_tx)) {
		dev_err(&pdev->dev, "Failed to ioremap regaddr_tx!\n");
		return PTR_ERR(x2_i2sidma[id].regaddr_tx);
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
	x2_i2sidma[id].idma_irq = ret;

	spin_lock_init(&(x2_i2sidma[id].lock));

	ret =  devm_snd_soc_register_platform(&pdev->dev,
					      &asoc_i2sidma_platform[id]);
	pr_err("success register platform %d, ret = %d\n", id, ret);

	ret = device_create_file(&pdev->dev, &dev_attr_debug);

	return ret;

}

#ifdef CONFIG_OF
static const struct of_device_id x2_i2sidma_of_match[] = {
	{.compatible = "x2-i2s0-idma",},
	{.compatible = "x2-i2s1-idma",},
	{}
};

MODULE_DEVICE_TABLE(of, x2_i2sidma_of_match);
#endif

static struct platform_driver i2s_idma_driver = {
	.driver = {
		   .name = "x2-i2s-idma",
		   .of_match_table = x2_i2sidma_of_match,
		   },

	.probe = asoc_i2sidma_platform_probe,
};

module_platform_driver(i2s_idma_driver);

MODULE_DESCRIPTION("X2 ASoC I2SDMA Driver");
MODULE_LICENSE("GPL");
