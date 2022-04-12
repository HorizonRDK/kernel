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

#include <linux/bitops.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include "dmaengine.h"
#include "hobot_dma.h"
#ifdef CONFIG_HOBOT_DMC_CLK
#include <soc/hobot/hobot_bus.h>
#endif

#define HOBOT_DMA_MAX_CHANS_PER_DEVICE    0x01
#define HOBOT_DMA_ADDR_WIDTH              32
#define HOBOT_DMA_MAX_TRANS_LEN           0x1000000
#define HOBOT_DMA_ALL_IRQ_MASK            (HOBOT_DMA_TXCMP|HOBOT_DMA_TXERR|HOBOT_DMA_CHTERM)


/* Delay loop counter to prevent hardware failure */
#define HOBOT_DMA_LOOP_COUNT              1000000

/**
 * struct hobot_dma_desc_hw - Hardware Descriptor
 * @src_addr: Source address @0x00
 * @dest_addr: Destination address @0x04
 * @tx_len: Length of transfer @0x08
 * @next_desc: Next Descriptor Pointer @0x0C
 */
struct hobot_dma_desc_hw {
	dma_addr_t src_addr;
	dma_addr_t dest_addr;
	size_t tx_len;
	dma_addr_t next_desc;
} __aligned(32);

/**
 * struct hobot_dma_tx_segment - Descriptor segment
 * @hw: Hardware descriptor
 * @node: Node in the descriptor segments list
 * @phys: Physical address of segment
 */
struct hobot_dma_tx_segment {
	struct hobot_dma_desc_hw hw;
	struct list_head node;
	dma_addr_t phys;
} __aligned(32);

/**
 * struct hobot_dma_tx_descriptor - Per Transaction structure
 * @async_tx: Async transaction descriptor
 * @segments: TX segments list
 * @node: Node in the channel descriptors list
 */
struct hobot_dma_tx_descriptor {
	struct dma_async_tx_descriptor async_tx;
	struct list_head segments;
	struct list_head node;
};

/**
 * struct hobot_dma_chan - Driver specific DMA channel structure
 * @xdev: Driver specific device structure
 * @desc_offset: TX descriptor registers offset
 * @lock: Descriptor operation lock
 * @pending_list: Descriptors waiting
 * @active_list: Descriptors ready to submit
 * @done_list: Complete descriptors
 * @common: DMA common channel
 * @desc_pool: Descriptors pool
 * @dev: The dma device
 * @irq: Channel IRQ
 * @id: Channel ID
 * @direction: Transfer direction
 * @has_sg: Support scatter transfers
 * @err: Channel has errors
 * @idle: Check for channel idle
 * @tasklet: Cleanup work after irq
 * @desc_pendingcount: Descriptor pending count
 * @desc_submitcount: Descriptor h/w submitted count
 * @start_transfer: Differentiate b/w DMA IP's transfer
 */
struct hobot_dma_chan {
	struct hobot_dma_device *xdev;
	u32 desc_offset;
	spinlock_t lock;
	struct list_head pending_list;
	struct list_head active_list;
	struct list_head done_list;
	struct dma_chan common;
	struct dma_pool *desc_pool;
	struct device *dev;
	int irq;
	int id;
	enum dma_transfer_direction direction;
	bool has_sg;
	bool err;
	bool idle;
	struct tasklet_struct tasklet;
	u32 desc_pendingcount;
	u32 desc_submitcount;
	void (*start_transfer)(struct hobot_dma_chan *chan);
};

/**
 * struct hobot_dma_device - DMA device structure
 * @regs: I/O mapped base address
 * @dev: Device Structure
 * @common: DMA device structure
 * @chan: Driver specific DMA channel
 * @has_sg: Specifies whether Scatter-Gather is present or not
 * @ext_addr: Indicates 64 bit addressing is supported by dma device
 * @pdev: Platform device structure pointer
 * @nr_channels: Number of channels DMA device supports
 */
struct hobot_dma_device {
	void __iomem *regs;
	struct device *dev;
	struct dma_device common;
	struct hobot_dma_chan *chan[HOBOT_DMA_MAX_CHANS_PER_DEVICE];
	bool has_sg;
	struct platform_device *pdev;
	u32 nr_channels;
#ifdef CONFIG_HOBOT_DMC_CLK
	struct hobot_dpm hb_dpm;
#endif
};

/* Macros */
#define to_hobot_chan(chan) \
	container_of(chan, struct hobot_dma_chan, common)
#define to_dma_tx_descriptor(tx) \
	container_of(tx, struct hobot_dma_tx_descriptor, async_tx)
#define hobot_dma_poll_timeout(chan, reg, val, cond, delay_us, timeout_us) \
	readl_poll_timeout(chan->xdev->regs + reg, val, cond, delay_us, timeout_us)

static int hobot_dma_chan_reset(struct hobot_dma_chan *chan);

/* IO accessors */
static inline u32 hobot_dma_rd(struct hobot_dma_chan *chan, u32 reg)
{
	return ioread32(chan->xdev->regs + reg);
}

static inline void hobot_dma_wr(struct hobot_dma_chan *chan, u32 reg, u32 value)
{
	iowrite32(value, chan->xdev->regs + reg);
}


/**
 * hobot_cdma_alloc_tx_segment - Allocate transaction segment
 * @chan: Driver specific DMA channel
 *
 * Return: The allocated segment on success and NULL on failure.
 */
static struct hobot_dma_tx_segment *hobot_dma_alloc_tx_segment(struct hobot_dma_chan *chan)
{
	struct hobot_dma_tx_segment *segment;
	dma_addr_t phys;

	segment = dma_pool_zalloc(chan->desc_pool, GFP_ATOMIC, &phys);
	if (!segment)
		return NULL;

	segment->phys = phys;

	return segment;
}


/**
 * hobot_cdma_free_tx_segment - Free transaction segment
 * @chan: Driver specific DMA channel
 * @segment: DMA transaction segment
 */
static void hobot_dma_free_tx_segment(struct hobot_dma_chan *chan, struct hobot_dma_tx_segment *segment)
{
	dma_pool_free(chan->desc_pool, segment, segment->phys);
}

/**
 * hobot_dma_tx_descriptor - Allocate transaction descriptor
 * @chan: Driver specific DMA channel
 *
 * Return: The allocated descriptor on success and NULL on failure.
 */
static struct hobot_dma_tx_descriptor *hobot_dma_alloc_tx_descriptor(struct hobot_dma_chan *chan)
{
	struct hobot_dma_tx_descriptor *desc;

	desc = kzalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return NULL;

	INIT_LIST_HEAD(&desc->segments);

	return desc;
}

/**
 * hobot_dma_free_tx_descriptor - Free transaction descriptor
 * @chan: Driver specific DMA channel
 * @desc: DMA transaction descriptor
 */
static void hobot_dma_free_tx_descriptor(struct hobot_dma_chan *chan, struct hobot_dma_tx_descriptor *desc)
{
	struct hobot_dma_tx_segment *dma_segment, *dma_next;

	if (!desc)
		return;

	list_for_each_entry_safe(dma_segment, dma_next, &desc->segments, node) {
		list_del(&dma_segment->node);
		hobot_dma_free_tx_segment(chan, dma_segment);
	}

	kfree(desc);
}

/* Required functions */

/**
 * hobot_dma_free_desc_list - Free descriptors list
 * @chan: Driver specific DMA channel
 * @list: List to parse and delete the descriptor
 */
static void hobot_dma_free_desc_list(struct hobot_dma_chan *chan,
					struct list_head *list)
{
	struct hobot_dma_tx_descriptor *desc, *next;

	list_for_each_entry_safe(desc, next, list, node) {
		list_del(&desc->node);
		hobot_dma_free_tx_descriptor(chan, desc);
	}
}

/**
 * hobot_dma_free_descriptors - Free channel descriptors
 * @chan: Driver specific DMA channel
 */
static void hobot_dma_free_descriptors(struct hobot_dma_chan *chan)
{
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	hobot_dma_free_desc_list(chan, &chan->pending_list);
	hobot_dma_free_desc_list(chan, &chan->done_list);
	hobot_dma_free_desc_list(chan, &chan->active_list);

	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * hobot_dma_free_chan_resources - Free channel resources
 * @dchan: DMA channel
 */
static void hobot_dma_free_chan_resources(struct dma_chan *dchan)
{
	struct hobot_dma_chan *chan = to_hobot_chan(dchan);

	dev_dbg(chan->dev, "Free all channel resources.\n");

	hobot_dma_free_descriptors(chan);

	return;
}

/**
 * hobot_dma_chan_desc_cleanup - Clean channel descriptors
 * @chan: Driver specific DMA channel
 */
static void hobot_dma_chan_desc_cleanup(struct hobot_dma_chan *chan)
{
	struct hobot_dma_tx_descriptor *desc, *next;
	unsigned long flags;
	spin_lock_irqsave(&chan->lock, flags);

	list_for_each_entry_safe(desc, next, &chan->done_list, node) {
		struct dmaengine_desc_callback cb;
		/* Remove from the list of running transactions */
		list_del(&desc->node);
		/* Run the link descriptor callback function */
		dmaengine_desc_get_callback(&desc->async_tx, &cb);
		if (dmaengine_desc_callback_valid(&cb)) {
			spin_unlock_irqrestore(&chan->lock, flags);
			dmaengine_desc_callback_invoke(&cb, NULL);
			spin_lock_irqsave(&chan->lock, flags);
		}
		/* Run any dependencies, then free the descriptor */
		dma_run_dependencies(&desc->async_tx);
		hobot_dma_free_tx_descriptor(chan, desc);
	}

	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * hobot_dma_do_tasklet - Schedule completion tasklet
 * @data: Pointer to the x2 DMA channel structure
 */
static void hobot_dma_do_tasklet(unsigned long data)
{
	struct hobot_dma_chan *chan = (struct hobot_dma_chan *)data;
	hobot_dma_chan_desc_cleanup(chan);
}

/**
 * hobot_dma_alloc_chan_resources - Allocate channel resources
 * @dchan: DMA channel
 *
 * Return: '0' on success and failure value on error
 */
static int hobot_dma_alloc_chan_resources(struct dma_chan *dchan)
{
	unsigned int val = 0;
	struct hobot_dma_chan *chan = to_hobot_chan(dchan);

	dma_cookie_init(dchan);

	if (chan->has_sg) {
		val = hobot_dma_rd(chan, HOBOT_DMA_CTRL_ADDR);
		val |= HOBOT_DMA_EN_LLI;
		val |= HOBOT_DMA_EN_CH;
	} else {
		val |= HOBOT_DMA_EN_CH;
	}
	hobot_dma_wr(chan, HOBOT_DMA_CTRL_ADDR, val);

	return 0;
}

/**
 * hobot_dma_tx_status - Get DMA transaction status
 * @dchan: DMA channel
 * @cookie: Transaction identifier
 * @txstate: Transaction state
 *
 * Return: DMA transaction status
 */
static enum dma_status hobot_dma_tx_status(struct dma_chan *dchan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{
	enum dma_status ret;

	ret = dma_cookie_status(dchan, cookie, txstate);
	if (ret == DMA_COMPLETE || !txstate) {
		return ret;
	}

	return ret;
}

/**
 * hobot_dma_start_transfer - Starts dma transfer
 * @chan: Driver specific channel struct pointer
 */
static void hobot_dma_start_transfer(struct hobot_dma_chan *chan)
{
	unsigned int val = 0;
	struct hobot_dma_tx_descriptor *head_desc, *tail_desc;
	struct hobot_dma_tx_segment *tail_segment;

	if (chan->err)
		return;

	if (!chan->idle)
		return;

	if (list_empty(&chan->pending_list))
		return;

	head_desc = list_first_entry(&chan->pending_list, struct hobot_dma_tx_descriptor, node);
	tail_desc = list_last_entry(&chan->pending_list, struct hobot_dma_tx_descriptor, node);
	tail_segment = list_last_entry(&tail_desc->segments, struct hobot_dma_tx_segment, node);

	if (hobot_dma_chan_reset(chan) < 0)
		return;

	/*mark the idle flag before starting DMA transmission*/
	chan->idle = false;

	if (chan->has_sg) {
		hobot_dma_wr(chan, HOBOT_DMA_LLI_ADDR, (u32)head_desc->async_tx.phys);

		/* Start the transfer */
		hobot_dma_wr(chan, HOBOT_DMA_SOFT_REQ, HOBOT_DMA_START_TX);
	} else {
		/* In simple mode */
		struct hobot_dma_tx_segment *segment;
		struct hobot_dma_desc_hw *hw;

		segment = list_first_entry(&head_desc->segments,
					   struct hobot_dma_tx_segment,
					   node);

		hw = &segment->hw;

		hobot_dma_wr(chan, HOBOT_DMA_SRC_ADDR,  (u32)hw->src_addr);
		hobot_dma_wr(chan, HOBOT_DMA_DEST_ADDR, (u32)hw->dest_addr);
		val = hobot_dma_rd(chan, HOBOT_DMA_CTRL_ADDR);
		val |= HOBOT_DMA_EN_CH;
		val &= 0xFF000000;
		val |= (u32)hw->tx_len;
		hobot_dma_wr(chan, HOBOT_DMA_CTRL_ADDR, val);

		/* Start the transfer */
		hobot_dma_wr(chan, HOBOT_DMA_SOFT_REQ, HOBOT_DMA_START_TX);
	}

	list_splice_tail_init(&chan->pending_list, &chan->active_list);
	chan->desc_pendingcount = 0;
}

/**
 * hobot_dma_issue_pending - Issue pending transactions
 * @dchan: DMA channel
 */
static void hobot_dma_issue_pending(struct dma_chan *dchan)
{
	struct hobot_dma_chan *chan = to_hobot_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	chan->start_transfer(chan);
	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * hobot_dma_complete_descriptor - Mark the active descriptor as complete
 * @chan : x2 DMA channel
 *
 * CONTEXT: hardirq
 */
static void hobot_dma_complete_descriptor(struct hobot_dma_chan *chan)
{
	struct hobot_dma_tx_descriptor *desc, *next;

	/* This function was invoked with lock held */
	if (list_empty(&chan->active_list))
		return;

	list_for_each_entry_safe(desc, next, &chan->active_list, node) {
		list_del(&desc->node);
		dma_cookie_complete(&desc->async_tx);
		list_add_tail(&desc->node, &chan->done_list);
	}
}

/**
 * hobot_dma_chan_reset - Reset DMA channel and enable interrupts
 * @chan: Driver specific DMA channel
 *
 * Return: '0' on success and failure value on error
 */
static int hobot_dma_chan_reset(struct hobot_dma_chan *chan)
{
	int err;
	unsigned int val;

	/* Reset VDMA */
	hobot_dma_wr(chan, HOBOT_DMA_FIFO_RST, HOBOT_DMA_RST_FIFO);

	/* Wait for the hardware to finish reset */
	err = hobot_dma_poll_timeout(chan, HOBOT_DMA_FIFO_RST, val,
				      !(val & HOBOT_DMA_RST_FIFO), 0,
				      HOBOT_DMA_LOOP_COUNT);
	if (err) {
		dev_err(chan->dev, "reset timeout\n");
		return -ETIMEDOUT;
	}

	chan->err = false;
	chan->idle = true;
	chan->desc_submitcount = 0;

	/* Enable interrupts */
	val = HOBOT_DMA_TXCMP | HOBOT_DMA_TXERR | HOBOT_DMA_CHTERM;
	hobot_dma_wr(chan, HOBOT_DMA_INT_UNMASK, val);
	hobot_dma_wr(chan, HOBOT_DMA_INT_SETMASK, 0x0);

	return 0;
}

/**
 * hobot_dma_irq_handler - DMA Interrupt handler
 * @irq: IRQ number
 * @data: Pointer to the x2 DMA channel structure
 *
 * Return: IRQ_HANDLED/IRQ_NONE
 */
static irqreturn_t hobot_dma_irq_handler(int irq, void *data)
{
	struct hobot_dma_chan *chan = data;
	u32 status;
	/* Read the status and ack the interrupts. */
	status = hobot_dma_rd(chan, HOBOT_DMA_SRCPND);
	if (!(status & HOBOT_DMA_ALL_IRQ_MASK)) {
		return IRQ_NONE;
	}
	hobot_dma_wr(chan, HOBOT_DMA_SRCPND, status);

	if (status & HOBOT_DMA_TXERR) {
		dev_dbg(chan->dev, "Channel transfer error!\n");
	}

	if (status & HOBOT_DMA_CHTERM) {
		dev_dbg(chan->dev, "Channel terminate!\n");
	}

	if (status & HOBOT_DMA_TXCMP) {
		spin_lock(&chan->lock);
		hobot_dma_complete_descriptor(chan);
		chan->idle = true;
		//chan->start_transfer(chan);
		spin_unlock(&chan->lock);
	}
	tasklet_schedule(&chan->tasklet);

	return IRQ_HANDLED;
}

/**
 * hobot_dma_append_desc_queue - Queuing descriptor
 * @chan: Driver specific dma channel
 * @desc: dma transaction descriptor
 */
static void hobot_dma_append_desc_queue(struct hobot_dma_chan *chan,
			      struct hobot_dma_tx_descriptor *desc)
{
	struct hobot_dma_tx_descriptor *tail_desc;
	struct hobot_dma_tx_segment *dma_tail_segment;

	if (list_empty(&chan->pending_list))
		goto append;

	/*
	 * Add the hardware descriptor to the chain of hardware descriptors
	 * that already exists in memory.
	 */
	tail_desc = list_last_entry(&chan->pending_list, struct hobot_dma_tx_descriptor, node);

	dma_tail_segment = list_last_entry(&tail_desc->segments, struct hobot_dma_tx_segment, node);
	dma_tail_segment->hw.next_desc = (u32)desc->async_tx.phys;

	/*
	 * Add the software descriptor and all children to the list
	 * of pending transactions
	 */
append:
	list_add_tail(&desc->node, &chan->pending_list);
	chan->desc_pendingcount++;
}

/**
 * hobot_dma_tx_submit - Submit DMA transaction
 * @tx: Async transaction descriptor
 *
 * Return: cookie value on success and failure value on error
 */
static dma_cookie_t hobot_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct hobot_dma_tx_descriptor *desc = to_dma_tx_descriptor(tx);
	struct hobot_dma_chan *chan = to_hobot_chan(tx->chan);
	dma_cookie_t cookie;
	unsigned long flags;
	int err;

	if (chan->err) {
		/*
		 * If reset fails, need to hard reset the system.
		 * Channel is no longer functional
		 */
		err = hobot_dma_chan_reset(chan);
		if (err < 0)
			return err;
	}

	spin_lock_irqsave(&chan->lock, flags);

	cookie = dma_cookie_assign(tx);

	/* Put this transaction onto the tail of the pending queue */
	hobot_dma_append_desc_queue(chan, desc);

	spin_unlock_irqrestore(&chan->lock, flags);

	return cookie;
}

/**
 * hobot_cdma_prep_memcpy - prepare descriptors for a memcpy transaction
 * @dchan: DMA channel
 * @dma_dst: destination address
 * @dma_src: source address
 * @len: transfer length
 * @flags: transfer ack flags
 *
 * Return: Async transaction descriptor on success and NULL on failure
 */
static struct dma_async_tx_descriptor *
hobot_dma_prep_memcpy(struct dma_chan *dchan, dma_addr_t dma_dst,
			dma_addr_t dma_src, size_t len, unsigned long flags)
{
	struct hobot_dma_chan *chan = to_hobot_chan(dchan);
	struct hobot_dma_tx_descriptor *desc;
	struct hobot_dma_tx_segment *segment, *prev;
	struct hobot_dma_desc_hw *hw;

	if (!len || len > HOBOT_DMA_MAX_TRANS_LEN) {
		return NULL;
	}

	desc = hobot_dma_alloc_tx_descriptor(chan);
	if (!desc) {
		return NULL;
	}

	dma_async_tx_descriptor_init(&desc->async_tx, &chan->common);
	desc->async_tx.tx_submit = hobot_dma_tx_submit;
	desc->async_tx.callback  = NULL;
	desc->async_tx.callback_result = NULL;

	/* Allocate the link descriptor from DMA pool */
	segment = hobot_dma_alloc_tx_segment(chan);
	if (!segment) {
		goto error;
	}

	hw = &segment->hw;
	hw->src_addr  = dma_src;
	hw->dest_addr = dma_dst;
	hw->tx_len    = len;

	/* Fill the previous next descriptor with current */
	if (!list_empty(&desc->segments)) {
		prev = list_last_entry(&desc->segments, struct hobot_dma_tx_segment, node);
		prev->hw.next_desc = segment->phys;
	}
	/* Insert the segment into the descriptor segments list. */
	list_add_tail(&segment->node, &desc->segments);

	/* Link the last hardware descriptor with the first. */
	prev = segment;
	segment = list_first_entry(&desc->segments, struct hobot_dma_tx_segment, node);
	desc->async_tx.phys = segment->phys;

	return &desc->async_tx;
error:
	hobot_dma_free_tx_descriptor(chan, desc);
	return NULL;
}

/**
 * hobot_dma_terminate_all - Halt the channel and free descriptors
 * @chan: Driver specific DMA Channel pointer
 */
static int hobot_dma_terminate_all(struct dma_chan *dchan)
{
	unsigned int val;
	struct hobot_dma_chan *chan = to_hobot_chan(dchan);

	/* Halt the DMA engine */
	val = hobot_dma_rd(chan, HOBOT_DMA_CTRL_ADDR);
	val &= ~HOBOT_DMA_EN_CH;
	hobot_dma_wr(chan, HOBOT_DMA_CTRL_ADDR, val);

	/* Remove and free all of the descriptors in the lists */
	hobot_dma_free_descriptors(chan);

	return 0;
}

/**
 * hobot_dma_chan_remove - Per Channel remove function
 * @chan: Driver specific DMA channel
 */
static void hobot_dma_chan_remove(struct hobot_dma_chan *chan)
{
	/* Disable all interrupts */
	hobot_dma_wr(chan, HOBOT_DMA_INT_SETMASK, HOBOT_DMA_ALL_IRQ_MASK);
	hobot_dma_wr(chan, HOBOT_DMA_INT_UNMASK, 0x0);

	if (chan->irq > 0)
		free_irq(chan->irq, chan);

	tasklet_kill(&chan->tasklet);

	list_del(&chan->common.device_node);
	dma_pool_destroy(chan->desc_pool);
	chan->desc_pool = NULL;
}

/**
 * hobot_dma_chan_probe - Per Channel Probing
 * It get channel features from the device tree entry and
 * initialize special channel handling routines
 *
 * @xdev: Driver specific device structure
 * @node: Device node
 *
 * Return: '0' on success and failure value on error
 */
static int hobot_dma_chan_probe(struct hobot_dma_device *xdev, struct device_node *node, int chan_id)
{
	struct hobot_dma_chan *chan;
	u32 width;
	int err;

	/* Allocate and initialize the channel structure */
	chan = devm_kzalloc(xdev->dev, sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	chan->dev = xdev->dev;
	chan->xdev = xdev;
	chan->has_sg = xdev->has_sg;
	chan->desc_pendingcount = 0x0;
	/* This variable enusres that descripotrs are not
	 * Submited when dma engine is in progress. This variable is
	 * Added to avoid pollling for a bit in the status register to
	 * Know dma state in the driver hot path.
	 */
	chan->idle = true;

	spin_lock_init(&chan->lock);
	INIT_LIST_HEAD(&chan->pending_list);
	INIT_LIST_HEAD(&chan->done_list);
	INIT_LIST_HEAD(&chan->active_list);


	width = 32 >> 3; /* Convert bits to bytes */
	xdev->common.copy_align = fls(width - 1);

	chan->direction = DMA_MEM_TO_MEM;
	chan->id = chan_id;

	/* Request the interrupt */
	chan->irq = irq_of_parse_and_map(node, 0);
	err = request_irq(chan->irq, hobot_dma_irq_handler, IRQF_SHARED, "hobot-dma-controller", chan);
	if (err) {
		dev_err(xdev->dev, "unable to request IRQ %d\n", chan->irq);
		return err;
	}

	chan->start_transfer = hobot_dma_start_transfer;

	/* Initialize the tasklet */
	tasklet_init(&chan->tasklet, hobot_dma_do_tasklet, (unsigned long)chan);

	/*
	 * Initialize the DMA channel and add it to the DMA engine channels
	 * list.
	 */
	chan->common.device = &xdev->common;

	list_add_tail(&chan->common.device_node, &xdev->common.channels);
	xdev->chan[chan->id] = chan;

	/* Has this channel already been allocated? */
	if (chan->desc_pool) {
		dev_err(xdev->dev, "hobot_dma_desc_pool already existed\n");
		return -1;
	}
	chan->desc_pool = dma_pool_create("hobot_dma_desc_pool",
				   chan->dev,
				   sizeof(struct hobot_dma_tx_segment),
				   __alignof__(struct hobot_dma_tx_segment),
				   0);
	if (!chan->desc_pool) {
		dev_err(chan->dev, "unable to allocate channel %d descriptor pool\n", chan->id);
		return -ENOMEM;
	}

	/* Reset the channel */
	err = hobot_dma_chan_reset(chan);
	if (err < 0) {
		dev_err(xdev->dev, "Reset channel failed\n");
		return err;
	}

	return 0;
}

/**
 * of_dma_hobot_xlate - Translation function
 * @dma_spec: Pointer to DMA specifier as found in the device tree
 * @ofdma: Pointer to DMA controller data
 *
 * Return: DMA channel pointer on success and NULL on error
 */
static struct dma_chan *of_dma_hobot_xlate(struct of_phandle_args *dma_spec,
						struct of_dma *ofdma)
{
	struct hobot_dma_device *xdev = ofdma->of_dma_data;
	int chan_id = dma_spec->args[0];

	if (chan_id >= xdev->nr_channels || !xdev->chan[chan_id])
		return NULL;

	return dma_get_slave_channel(&xdev->chan[chan_id]->common);
}

#ifdef CONFIG_HOBOT_DMC_CLK
static int hobot_dma_dfs_cb(struct hobot_dpm *dpm,
		unsigned long val, int state)
{
	struct hobot_dma_device *xdev =
		container_of(dpm, struct hobot_dma_device, hb_dpm);

	if (val == HB_BUS_SIGNAL_START) {
		int ch;
		bool dma_idle = true;

		for (ch = 0; ch < xdev->nr_channels; ch++) {
			struct hobot_dma_chan *chan = xdev->chan[ch];
			dma_idle &= chan->idle;
		}

		return dma_idle ? 0 : -EBUSY;
	}

	return 0;
}
#endif

static const struct of_device_id hobot_dma_of_ids[] = {
	{ .compatible = "hobot,hobot-dma", },
	{}
};
MODULE_DEVICE_TABLE(of, hobot_dma_of_ids);

/**
 * hobot_dma_probe - Driver probe function
 * @pdev: Pointer to the platform_device structure
 *
 * Return: '0' on success and failure value on error
 */
static int hobot_dma_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct hobot_dma_device *xdev;
	//struct device_node *child, *np = pdev->dev.of_node;
	struct resource *io;
	int i, err;

	/* Allocate and initialize the DMA engine structure */
	xdev = devm_kzalloc(&pdev->dev, sizeof(*xdev), GFP_KERNEL);
	if (!xdev)
		return -ENOMEM;
	xdev->dev = &pdev->dev;

	/* Request and map I/O memory */
	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	xdev->regs = devm_ioremap_resource(&pdev->dev, io);
	if (IS_ERR(xdev->regs))
		return PTR_ERR_OR_ZERO(xdev->regs);

	/* Retrieve the DMA engine properties from the device tree */
	xdev->has_sg = false;

	/* Set the dma mask bits */
	dma_set_mask(xdev->dev, DMA_BIT_MASK(HOBOT_DMA_ADDR_WIDTH));

	/* Initialize the DMA engine */
	xdev->common.dev = &pdev->dev;

	INIT_LIST_HEAD(&xdev->common.channels);

	dma_cap_set(DMA_MEMCPY, xdev->common.cap_mask);
	xdev->common.device_alloc_chan_resources = hobot_dma_alloc_chan_resources;
	xdev->common.device_free_chan_resources  = hobot_dma_free_chan_resources;
	xdev->common.device_terminate_all        = hobot_dma_terminate_all;
	xdev->common.device_tx_status            = hobot_dma_tx_status;
	xdev->common.device_issue_pending        = hobot_dma_issue_pending;
	xdev->common.device_prep_dma_memcpy      = hobot_dma_prep_memcpy;

	platform_set_drvdata(pdev, xdev);

	/* Initialize the channels */
	xdev->nr_channels = 1;
	for (i=0; i<xdev->nr_channels; i++) {
		hobot_dma_chan_probe(xdev, node, i);
	}

	/* Register the DMA engine with the core */
	dma_async_device_register(&xdev->common);

	err = of_dma_controller_register(node, of_dma_hobot_xlate, xdev);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to register DMA to DT\n");
		dma_async_device_unregister(&xdev->common);
		goto error;
	}

#ifdef CONFIG_HOBOT_DMC_CLK
	xdev->hb_dpm.dpm_call = &hobot_dma_dfs_cb;
	hobot_dfs_register(&xdev->hb_dpm, xdev->dev);
#endif

	dev_info(&pdev->dev, "Hobot DMA Engine Driver Probed!!\n");

	return 0;

error:
	for (i = 0; i < xdev->nr_channels; i++)
		if (xdev->chan[i])
			hobot_dma_chan_remove(xdev->chan[i]);

	return err;
}

/**
 * hobot_dma_remove - Driver remove function
 * @pdev: Pointer to the platform_device structure
 *
 * Return: Always '0'
 */
static int hobot_dma_remove(struct platform_device *pdev)
{
	struct hobot_dma_device *xdev = platform_get_drvdata(pdev);
	int i;

	of_dma_controller_free(pdev->dev.of_node);

	dma_async_device_unregister(&xdev->common);

	for (i = 0; i < xdev->nr_channels; i++) {
		if (xdev->chan[i]) {
			hobot_dma_chan_remove(xdev->chan[i]);
		}
	}

#ifdef CONFIG_HOBOT_DMC_CLK
	hobot_dfs_unregister(&xdev->hb_dpm);
#endif

	return 0;
}

static struct platform_driver hobot_dma_driver = {
	.probe = hobot_dma_probe,
	.remove = hobot_dma_remove,
	.driver = {
		.name = "hobot_dma",
		.of_match_table = hobot_dma_of_ids,
	},
};

static int __init hobot_dma_init(void)
{
	int ret;

	ret = platform_driver_register(&hobot_dma_driver);
	if (ret) {
		printk(KERN_ERR"hobot_dma:probe failed:%d\n", ret);
	}

	return ret;
}

static void __exit hobot_dma_exit(void)
{
	platform_driver_unregister(&hobot_dma_driver);
}

arch_initcall(hobot_dma_init);
module_exit(hobot_dma_exit);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("Hobot DMA driver");
MODULE_LICENSE("GPL v2");
