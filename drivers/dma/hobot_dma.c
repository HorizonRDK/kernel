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


#define X2_DMA_MAX_CHANS_PER_DEVICE    0x01
#define X2_DMA_ADDR_WIDTH              32
#define X2_DMA_MAX_TRANS_LEN           0x1000000
#define X2_DMA_ALL_IRQ_MASK            (X2_DMA_TXCMP|X2_DMA_TXERR|X2_DMA_CHTERM)


/* Delay loop counter to prevent hardware failure */
#define X2_DMA_LOOP_COUNT              1000000

/**
 * struct x2_dma_desc_hw - Hardware Descriptor
 * @src_addr: Source address @0x00
 * @dest_addr: Destination address @0x04
 * @tx_len: Length of transfer @0x08
 * @next_desc: Next Descriptor Pointer @0x0C
 */
struct x2_dma_desc_hw {
	u32 src_addr;
	u32 dest_addr;
	u32 tx_len;
	u32 next_desc;
} __aligned(32);

/**
 * struct x2_dma_tx_segment - Descriptor segment
 * @hw: Hardware descriptor
 * @node: Node in the descriptor segments list
 * @phys: Physical address of segment
 */
struct x2_dma_tx_segment {
	struct x2_dma_desc_hw hw;
	struct list_head node;
	dma_addr_t phys;
} __aligned(32);

/**
 * struct x2_dma_tx_descriptor - Per Transaction structure
 * @async_tx: Async transaction descriptor
 * @segments: TX segments list
 * @node: Node in the channel descriptors list
 */
struct x2_dma_tx_descriptor {
	struct dma_async_tx_descriptor async_tx;
	struct list_head segments;
	struct list_head node;
};

/**
 * struct x2_dma_chan - Driver specific DMA channel structure
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
struct x2_dma_chan {
	struct x2_dma_device *xdev;
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
	void (*start_transfer)(struct x2_dma_chan *chan);
};

/**
 * struct x2_dma_device - DMA device structure
 * @regs: I/O mapped base address
 * @dev: Device Structure
 * @common: DMA device structure
 * @chan: Driver specific DMA channel
 * @has_sg: Specifies whether Scatter-Gather is present or not
 * @ext_addr: Indicates 64 bit addressing is supported by dma device
 * @pdev: Platform device structure pointer
 * @nr_channels: Number of channels DMA device supports
 */
struct x2_dma_device {
	void __iomem *regs;
	struct device *dev;
	struct dma_device common;
	struct x2_dma_chan *chan[X2_DMA_MAX_CHANS_PER_DEVICE];
	bool has_sg;
	struct platform_device *pdev;
	u32 nr_channels;
};

/* Macros */
#define to_x2_chan(chan) \
	container_of(chan, struct x2_dma_chan, common)
#define to_dma_tx_descriptor(tx) \
	container_of(tx, struct x2_dma_tx_descriptor, async_tx)
#define x2_dma_poll_timeout(chan, reg, val, cond, delay_us, timeout_us) \
	readl_poll_timeout(chan->xdev->regs + reg, val, cond, delay_us, timeout_us)

static int x2_dma_chan_reset(struct x2_dma_chan *chan);

/* IO accessors */
static inline u32 x2_dma_rd(struct x2_dma_chan *chan, u32 reg)
{
	return ioread32(chan->xdev->regs + reg);
}

static inline void x2_dma_wr(struct x2_dma_chan *chan, u32 reg, u32 value)
{
	iowrite32(value, chan->xdev->regs + reg);
}


/**
 * x2_cdma_alloc_tx_segment - Allocate transaction segment
 * @chan: Driver specific DMA channel
 *
 * Return: The allocated segment on success and NULL on failure.
 */
static struct x2_dma_tx_segment *x2_dma_alloc_tx_segment(struct x2_dma_chan *chan)
{
	struct x2_dma_tx_segment *segment;
	dma_addr_t phys;

	segment = dma_pool_zalloc(chan->desc_pool, GFP_ATOMIC, &phys);
	if (!segment)
		return NULL;

	segment->phys = phys;

	return segment;
}


/**
 * x2_cdma_free_tx_segment - Free transaction segment
 * @chan: Driver specific DMA channel
 * @segment: DMA transaction segment
 */
static void x2_dma_free_tx_segment(struct x2_dma_chan *chan, struct x2_dma_tx_segment *segment)
{
	dma_pool_free(chan->desc_pool, segment, segment->phys);
}

/**
 * x2_dma_tx_descriptor - Allocate transaction descriptor
 * @chan: Driver specific DMA channel
 *
 * Return: The allocated descriptor on success and NULL on failure.
 */
static struct x2_dma_tx_descriptor *x2_dma_alloc_tx_descriptor(struct x2_dma_chan *chan)
{
	struct x2_dma_tx_descriptor *desc;

	desc = kzalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return NULL;

	INIT_LIST_HEAD(&desc->segments);

	return desc;
}

/**
 * x2_dma_free_tx_descriptor - Free transaction descriptor
 * @chan: Driver specific DMA channel
 * @desc: DMA transaction descriptor
 */
static void x2_dma_free_tx_descriptor(struct x2_dma_chan *chan, struct x2_dma_tx_descriptor *desc)
{
	struct x2_dma_tx_segment *dma_segment, *dma_next;

	if (!desc)
		return;

	list_for_each_entry_safe(dma_segment, dma_next, &desc->segments, node) {
		list_del(&dma_segment->node);
		x2_dma_free_tx_segment(chan, dma_segment);
	}

	kfree(desc);
}

/* Required functions */

/**
 * x2_dma_free_desc_list - Free descriptors list
 * @chan: Driver specific DMA channel
 * @list: List to parse and delete the descriptor
 */
static void x2_dma_free_desc_list(struct x2_dma_chan *chan,
					struct list_head *list)
{
	struct x2_dma_tx_descriptor *desc, *next;

	list_for_each_entry_safe(desc, next, list, node) {
		list_del(&desc->node);
		x2_dma_free_tx_descriptor(chan, desc);
	}
}

/**
 * x2_dma_free_descriptors - Free channel descriptors
 * @chan: Driver specific DMA channel
 */
static void x2_dma_free_descriptors(struct x2_dma_chan *chan)
{
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	x2_dma_free_desc_list(chan, &chan->pending_list);
	x2_dma_free_desc_list(chan, &chan->done_list);
	x2_dma_free_desc_list(chan, &chan->active_list);

	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * x2_dma_free_chan_resources - Free channel resources
 * @dchan: DMA channel
 */
static void x2_dma_free_chan_resources(struct dma_chan *dchan)
{
	struct x2_dma_chan *chan = to_x2_chan(dchan);

	dev_dbg(chan->dev, "Free all channel resources.\n");

	x2_dma_free_descriptors(chan);

	return;
}

/**
 * x2_dma_chan_desc_cleanup - Clean channel descriptors
 * @chan: Driver specific DMA channel
 */
static void x2_dma_chan_desc_cleanup(struct x2_dma_chan *chan)
{
	struct x2_dma_tx_descriptor *desc, *next;
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
		x2_dma_free_tx_descriptor(chan, desc);
	}

	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * x2_dma_do_tasklet - Schedule completion tasklet
 * @data: Pointer to the x2 DMA channel structure
 */
static void x2_dma_do_tasklet(unsigned long data)
{
	struct x2_dma_chan *chan = (struct x2_dma_chan *)data;
	x2_dma_chan_desc_cleanup(chan);
}

/**
 * x2_dma_alloc_chan_resources - Allocate channel resources
 * @dchan: DMA channel
 *
 * Return: '0' on success and failure value on error
 */
static int x2_dma_alloc_chan_resources(struct dma_chan *dchan)
{
	unsigned int val = 0;
	struct x2_dma_chan *chan = to_x2_chan(dchan);

	dma_cookie_init(dchan);

	if (chan->has_sg) {
		val = x2_dma_rd(chan, X2_DMA_CTRL_ADDR);
		val |= X2_DMA_EN_LLI;
		val |= X2_DMA_EN_CH;
	} else {
		val |= X2_DMA_EN_CH;
	}
	x2_dma_wr(chan, X2_DMA_CTRL_ADDR, val);

	return 0;
}

/**
 * x2_dma_tx_status - Get DMA transaction status
 * @dchan: DMA channel
 * @cookie: Transaction identifier
 * @txstate: Transaction state
 *
 * Return: DMA transaction status
 */
static enum dma_status x2_dma_tx_status(struct dma_chan *dchan,
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
 * x2_dma_start_transfer - Starts dma transfer
 * @chan: Driver specific channel struct pointer
 */
static void x2_dma_start_transfer(struct x2_dma_chan *chan)
{
	unsigned int val = 0;
	struct x2_dma_tx_descriptor *head_desc, *tail_desc;
	struct x2_dma_tx_segment *tail_segment;

	if (chan->err)
		return;

	if (!chan->idle)
		return;

	if (list_empty(&chan->pending_list))
		return;

	head_desc = list_first_entry(&chan->pending_list, struct x2_dma_tx_descriptor, node);
	tail_desc = list_last_entry(&chan->pending_list, struct x2_dma_tx_descriptor, node);
	tail_segment = list_last_entry(&tail_desc->segments, struct x2_dma_tx_segment, node);

	if (x2_dma_chan_reset(chan) < 0)
		return;

	if (chan->has_sg) {
		x2_dma_wr(chan, X2_DMA_LLI_ADDR, head_desc->async_tx.phys);

		/* Start the transfer */
		x2_dma_wr(chan, X2_DMA_SOFT_REQ, X2_DMA_START_TX);
	} else {
		/* In simple mode */
		struct x2_dma_tx_segment *segment;
		struct x2_dma_desc_hw *hw;

		segment = list_first_entry(&head_desc->segments,
					   struct x2_dma_tx_segment,
					   node);

		hw = &segment->hw;

		x2_dma_wr(chan, X2_DMA_SRC_ADDR,  hw->src_addr);
		x2_dma_wr(chan, X2_DMA_DEST_ADDR, hw->dest_addr);
		val = x2_dma_rd(chan, X2_DMA_CTRL_ADDR);
		val |= X2_DMA_EN_CH;
		val &= 0xFF000000;
		val |= hw->tx_len;
		x2_dma_wr(chan, X2_DMA_CTRL_ADDR, val);

		/* Start the transfer */
		x2_dma_wr(chan, X2_DMA_SOFT_REQ, X2_DMA_START_TX);
	}

	list_splice_tail_init(&chan->pending_list, &chan->active_list);
	chan->desc_pendingcount = 0;
	chan->idle = false;
}

/**
 * x2_dma_issue_pending - Issue pending transactions
 * @dchan: DMA channel
 */
static void x2_dma_issue_pending(struct dma_chan *dchan)
{
	struct x2_dma_chan *chan = to_x2_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	chan->start_transfer(chan);
	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * x2_dma_complete_descriptor - Mark the active descriptor as complete
 * @chan : x2 DMA channel
 *
 * CONTEXT: hardirq
 */
static void x2_dma_complete_descriptor(struct x2_dma_chan *chan)
{
	struct x2_dma_tx_descriptor *desc, *next;

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
 * x2_dma_chan_reset - Reset DMA channel and enable interrupts
 * @chan: Driver specific DMA channel
 *
 * Return: '0' on success and failure value on error
 */
static int x2_dma_chan_reset(struct x2_dma_chan *chan)
{
	int err;
	unsigned int val;

	/* Reset VDMA */
	x2_dma_wr(chan, X2_DMA_FIFO_RST, x2_DMA_RST_FIFO);

	/* Wait for the hardware to finish reset */
	err = x2_dma_poll_timeout(chan, X2_DMA_FIFO_RST, val,
				      !(val & x2_DMA_RST_FIFO), 0,
				      X2_DMA_LOOP_COUNT);
	if (err) {
		dev_err(chan->dev, "reset timeout\n");
		return -ETIMEDOUT;
	}

	chan->err = false;
	chan->idle = true;
	chan->desc_submitcount = 0;

	/* Enable interrupts */
	val = X2_DMA_TXCMP | X2_DMA_TXERR | X2_DMA_CHTERM;
	x2_dma_wr(chan, X2_DMA_INT_UNMASK, val);
	x2_dma_wr(chan, X2_DMA_INT_SETMASK, 0x0);

	return 0;
}

/**
 * x2_dma_irq_handler - DMA Interrupt handler
 * @irq: IRQ number
 * @data: Pointer to the x2 DMA channel structure
 *
 * Return: IRQ_HANDLED/IRQ_NONE
 */
static irqreturn_t x2_dma_irq_handler(int irq, void *data)
{
	struct x2_dma_chan *chan = data;
	u32 status;
	/* Read the status and ack the interrupts. */
	status = x2_dma_rd(chan, X2_DMA_SRCPND);
	if (!(status & X2_DMA_ALL_IRQ_MASK)) {
		return IRQ_NONE;
	}
	x2_dma_wr(chan, X2_DMA_SRCPND, status);

	if (status & X2_DMA_TXERR) {
		dev_dbg(chan->dev, "Channel transfer error!\n");
	}

	if (status & X2_DMA_CHTERM) {
		dev_dbg(chan->dev, "Channel terminate!\n");
	}

	if (status & X2_DMA_TXCMP) {
		spin_lock(&chan->lock);
		x2_dma_complete_descriptor(chan);
		chan->idle = true;
		//chan->start_transfer(chan);
		spin_unlock(&chan->lock);
	}
	tasklet_schedule(&chan->tasklet);

	return IRQ_HANDLED;
}

/**
 * x2_dma_append_desc_queue - Queuing descriptor
 * @chan: Driver specific dma channel
 * @desc: dma transaction descriptor
 */
static void x2_dma_append_desc_queue(struct x2_dma_chan *chan,
			      struct x2_dma_tx_descriptor *desc)
{
	struct x2_dma_tx_descriptor *tail_desc;
	struct x2_dma_tx_segment *dma_tail_segment;

	if (list_empty(&chan->pending_list))
		goto append;

	/*
	 * Add the hardware descriptor to the chain of hardware descriptors
	 * that already exists in memory.
	 */
	tail_desc = list_last_entry(&chan->pending_list, struct x2_dma_tx_descriptor, node);

	dma_tail_segment = list_last_entry(&tail_desc->segments, struct x2_dma_tx_segment, node);
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
 * x2_dma_tx_submit - Submit DMA transaction
 * @tx: Async transaction descriptor
 *
 * Return: cookie value on success and failure value on error
 */
static dma_cookie_t x2_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct x2_dma_tx_descriptor *desc = to_dma_tx_descriptor(tx);
	struct x2_dma_chan *chan = to_x2_chan(tx->chan);
	dma_cookie_t cookie;
	unsigned long flags;
	int err;

	if (chan->err) {
		/*
		 * If reset fails, need to hard reset the system.
		 * Channel is no longer functional
		 */
		err = x2_dma_chan_reset(chan);
		if (err < 0)
			return err;
	}

	spin_lock_irqsave(&chan->lock, flags);

	cookie = dma_cookie_assign(tx);

	/* Put this transaction onto the tail of the pending queue */
	x2_dma_append_desc_queue(chan, desc);

	spin_unlock_irqrestore(&chan->lock, flags);

	return cookie;
}

/**
 * x2_cdma_prep_memcpy - prepare descriptors for a memcpy transaction
 * @dchan: DMA channel
 * @dma_dst: destination address
 * @dma_src: source address
 * @len: transfer length
 * @flags: transfer ack flags
 *
 * Return: Async transaction descriptor on success and NULL on failure
 */
static struct dma_async_tx_descriptor *
x2_dma_prep_memcpy(struct dma_chan *dchan, dma_addr_t dma_dst,
			dma_addr_t dma_src, size_t len, unsigned long flags)
{
	struct x2_dma_chan *chan = to_x2_chan(dchan);
	struct x2_dma_tx_descriptor *desc;
	struct x2_dma_tx_segment *segment, *prev;
	struct x2_dma_desc_hw *hw;

	if (!len || len > X2_DMA_MAX_TRANS_LEN) {
		return NULL;
	}

	desc = x2_dma_alloc_tx_descriptor(chan);
	if (!desc) {
		return NULL;
	}

	dma_async_tx_descriptor_init(&desc->async_tx, &chan->common);
	desc->async_tx.tx_submit = x2_dma_tx_submit;
	desc->async_tx.callback  = NULL;
	desc->async_tx.callback_result = NULL;

	/* Allocate the link descriptor from DMA pool */
	segment = x2_dma_alloc_tx_segment(chan);
	if (!segment) {
		goto error;
	}

	hw = &segment->hw;
	hw->src_addr  = dma_src;
	hw->dest_addr = dma_dst;
	hw->tx_len    = len;

	/* Fill the previous next descriptor with current */
	if (!list_empty(&desc->segments)) {
		prev = list_last_entry(&desc->segments, struct x2_dma_tx_segment, node);
		prev->hw.next_desc = segment->phys;
	}
	/* Insert the segment into the descriptor segments list. */
	list_add_tail(&segment->node, &desc->segments);

	/* Link the last hardware descriptor with the first. */
	prev = segment;
	segment = list_first_entry(&desc->segments, struct x2_dma_tx_segment, node);
	desc->async_tx.phys = segment->phys;

	return &desc->async_tx;
error:
	x2_dma_free_tx_descriptor(chan, desc);
	return NULL;
}

/**
 * x2_dma_terminate_all - Halt the channel and free descriptors
 * @chan: Driver specific DMA Channel pointer
 */
static int x2_dma_terminate_all(struct dma_chan *dchan)
{
	unsigned int val;
	struct x2_dma_chan *chan = to_x2_chan(dchan);

	/* Halt the DMA engine */
	val = x2_dma_rd(chan, X2_DMA_CTRL_ADDR);
	val &= ~X2_DMA_EN_CH;
	x2_dma_wr(chan, X2_DMA_CTRL_ADDR, val);

	/* Remove and free all of the descriptors in the lists */
	x2_dma_free_descriptors(chan);

	return 0;
}

/**
 * x2_dma_chan_remove - Per Channel remove function
 * @chan: Driver specific DMA channel
 */
static void x2_dma_chan_remove(struct x2_dma_chan *chan)
{
	/* Disable all interrupts */
	x2_dma_wr(chan, X2_DMA_INT_SETMASK, X2_DMA_ALL_IRQ_MASK);
	x2_dma_wr(chan, X2_DMA_INT_UNMASK, 0x0);

	if (chan->irq > 0)
		free_irq(chan->irq, chan);

	tasklet_kill(&chan->tasklet);

	list_del(&chan->common.device_node);
	dma_pool_destroy(chan->desc_pool);
	chan->desc_pool = NULL;
}

/**
 * x2_dma_chan_probe - Per Channel Probing
 * It get channel features from the device tree entry and
 * initialize special channel handling routines
 *
 * @xdev: Driver specific device structure
 * @node: Device node
 *
 * Return: '0' on success and failure value on error
 */
static int x2_dma_chan_probe(struct x2_dma_device *xdev, struct device_node *node, int chan_id)
{
	struct x2_dma_chan *chan;
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
	err = request_irq(chan->irq, x2_dma_irq_handler, IRQF_SHARED, "x2-dma-controller", chan);
	if (err) {
		dev_err(xdev->dev, "unable to request IRQ %d\n", chan->irq);
		return err;
	}

	chan->start_transfer = x2_dma_start_transfer;

	/* Initialize the tasklet */
	tasklet_init(&chan->tasklet, x2_dma_do_tasklet, (unsigned long)chan);

	/*
	 * Initialize the DMA channel and add it to the DMA engine channels
	 * list.
	 */
	chan->common.device = &xdev->common;

	list_add_tail(&chan->common.device_node, &xdev->common.channels);
	xdev->chan[chan->id] = chan;

	/* Has this channel already been allocated? */
	if (chan->desc_pool) {
		dev_err(xdev->dev, "x2_dma_desc_pool already existed\n");
		return -1;
	}
	chan->desc_pool = dma_pool_create("x2_dma_desc_pool",
				   chan->dev,
				   sizeof(struct x2_dma_tx_segment),
				   __alignof__(struct x2_dma_tx_segment),
				   0);
	if (!chan->desc_pool) {
		dev_err(chan->dev, "unable to allocate channel %d descriptor pool\n", chan->id);
		return -ENOMEM;
	}

	/* Reset the channel */
	err = x2_dma_chan_reset(chan);
	if (err < 0) {
		dev_err(xdev->dev, "Reset channel failed\n");
		return err;
	}

	return 0;
}

/**
 * of_dma_x2_xlate - Translation function
 * @dma_spec: Pointer to DMA specifier as found in the device tree
 * @ofdma: Pointer to DMA controller data
 *
 * Return: DMA channel pointer on success and NULL on error
 */
static struct dma_chan *of_dma_x2_xlate(struct of_phandle_args *dma_spec,
						struct of_dma *ofdma)
{
	struct x2_dma_device *xdev = ofdma->of_dma_data;
	int chan_id = dma_spec->args[0];

	if (chan_id >= xdev->nr_channels || !xdev->chan[chan_id])
		return NULL;

	return dma_get_slave_channel(&xdev->chan[chan_id]->common);
}

static const struct of_device_id x2_dma_of_ids[] = {
	{ .compatible = "hobot,x2-dma", },
	{}
};
MODULE_DEVICE_TABLE(of, x2_dma_of_ids);

/**
 * x2_dma_probe - Driver probe function
 * @pdev: Pointer to the platform_device structure
 *
 * Return: '0' on success and failure value on error
 */
static int x2_dma_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct x2_dma_device *xdev;
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
		return PTR_ERR(xdev->regs);

	/* Retrieve the DMA engine properties from the device tree */
	xdev->has_sg = false;

	/* Set the dma mask bits */
	dma_set_mask(xdev->dev, DMA_BIT_MASK(X2_DMA_ADDR_WIDTH));

	/* Initialize the DMA engine */
	xdev->common.dev = &pdev->dev;

	INIT_LIST_HEAD(&xdev->common.channels);

	dma_cap_set(DMA_MEMCPY, xdev->common.cap_mask);
	xdev->common.device_alloc_chan_resources = x2_dma_alloc_chan_resources;
	xdev->common.device_free_chan_resources  = x2_dma_free_chan_resources;
	xdev->common.device_terminate_all        = x2_dma_terminate_all;
	xdev->common.device_tx_status            = x2_dma_tx_status;
	xdev->common.device_issue_pending        = x2_dma_issue_pending;
	xdev->common.device_prep_dma_memcpy      = x2_dma_prep_memcpy;

	platform_set_drvdata(pdev, xdev);

	/* Initialize the channels */
	xdev->nr_channels = 1;
	for (i=0; i<xdev->nr_channels; i++) {
		x2_dma_chan_probe(xdev, node, i);
	}

	/* Register the DMA engine with the core */
	dma_async_device_register(&xdev->common);

	err = of_dma_controller_register(node, of_dma_x2_xlate, xdev);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to register DMA to DT\n");
		dma_async_device_unregister(&xdev->common);
		goto error;
	}

	dev_info(&pdev->dev, "X2 DMA Engine Driver Probed!!\n");

	return 0;

error:
	for (i = 0; i < xdev->nr_channels; i++)
		if (xdev->chan[i])
			x2_dma_chan_remove(xdev->chan[i]);

	return err;
}

/**
 * x2_dma_remove - Driver remove function
 * @pdev: Pointer to the platform_device structure
 *
 * Return: Always '0'
 */
static int x2_dma_remove(struct platform_device *pdev)
{
	struct x2_dma_device *xdev = platform_get_drvdata(pdev);
	int i;

	of_dma_controller_free(pdev->dev.of_node);

	dma_async_device_unregister(&xdev->common);

	for (i = 0; i < xdev->nr_channels; i++) {
		if (xdev->chan[i]) {
			x2_dma_chan_remove(xdev->chan[i]);
		}
	}
	return 0;
}

static struct platform_driver x2_dma_driver = {
	.probe = x2_dma_probe,
	.remove = x2_dma_remove,
	.driver = {
		.name = "x2_dma",
		.of_match_table = x2_dma_of_ids,
	},
};

static int __init x2_dma_init(void)
{
	int ret;

	ret = platform_driver_register(&x2_dma_driver);
	if (ret) {
		printk(KERN_ERR"x2_dma:probe failed:%d\n", ret);
	}

	return ret;
}

static void __exit x2_dma_exit(void)
{
	platform_driver_unregister(&x2_dma_driver);
}

arch_initcall(x2_dma_init);
module_exit(x2_dma_exit);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("X2 DMA driver");
MODULE_LICENSE("GPL v2");
