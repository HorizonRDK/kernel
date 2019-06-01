/*
 * X2 SPI controller driver (master mode only)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/reset.h>

#define X2_SPI_MAX_CS          3
#define X2_SPI_NAME            "x2_spi"
/* X2 SPI register offsets */
#define X2_SPI_TXD_REG         0x00 /* data transmit register */
#define X2_SPI_RXD_REG         0x04 /* data receive register */
#define X2_SPI_CTRL_REG        0x08 /* SPI control register */
#define X2_SPI_SSC_REG         0x0C /* SSN limit configure register */
#define X2_SPI_SFSR_REG        0x10 /* SPI status register */
#define X2_SPI_RFTO_REG        0x14 /* Threshold of receive time out */
#define X2_SPI_TLEN_REG        0x18 /* transfer length */
#define X2_SPI_INST_REG        0x1C /* instruction should be transmitted */
#define X2_SPI_INST_MASK_REG   0x20 /* instruction mask register */
#define X2_SPI_SRCPND_REG      0x24 /* spi interrupt source pending register */
#define X2_SPI_INTMASK_REG     0x28 /* interrupt mask register */
#define X2_SPI_INTSETMASK_REG  0x2C /* interrupt set mask register */
#define X2_SPI_INTUNMASK_REG   0x30 /* interrupt unset mask register */
#define X2_SPI_DMA_CTRL0_REG   0x34 /* DMA control register_0 */
#define X2_SPI_DMA_CTRL1_REG   0x38 /* DMA control register_1 */
#define X2_SPI_TDMA_ADDR0_REG  0x3C /* transmit dma addr_0 */
#define X2_SPI_TDMA_SIZE0_REG  0x40 /* transmit dma size_0 */
#define X2_SPI_TDMA_ADDR1_REG  0x44 /* transmit dma addr_1 */
#define X2_SPI_TDMA_SIZE1_REG  0x48 /* transmit dma size_1 */
#define X2_SPI_RDMA_ADDR0_REG  0x4C /* receive  dma addr_0 */
#define X2_SPI_RDMA_SIZE0_REG  0x50 /* receive  dma size_0 */
#define X2_SPI_RDMA_ADDR1_REG  0x54 /* receive  dma addr_1 */
#define X2_SPI_RDMA_SIZE1_REG  0x58 /* receive  dma size_1 */
#define X2_SPI_TSIZE_REG       0x5C /* current tx_size */
#define X2_SPI_RSIZE_REG       0x60 /* current rx_size */
#define X2_SPI_FIFO_RESET_REG  0x64 /* FIFO reset register */
#define X2_SPI_TDMA_SIZE_REG   0x68 /* tx_dma size for ping-pong */
#define X2_SPI_RDMA_SIZE_REG   0x6C /* rx_dma size for ping-pong */
/* X2 SPI CTRL bit Masks */
#define X2_SPI_SS_OFFSET       30
#define X2_SPI_SS_MASK         0xC0000000
#define X2_SPI_DIVIDER_OFFSET  0
#define X2_SPI_DIVIDER_MASK    0x0000FFFF
#define X2_SPI_CORE_EN         BIT(16)
#define X2_SPI_SSAL            BIT(17)
#define X2_SPI_LSB             BIT(18)
#define X2_SPI_POLARITY        BIT(19)
#define X2_SPI_PHASE           BIT(20)
#define X2_SPI_SLAVE_MODE      BIT(21)
#define X2_SPI_RX_DIS          BIT(22)
#define X2_SPI_TX_DIS          BIT(23)
#define X2_SPI_RX_FIFOE        BIT(24)
#define X2_SPI_TX_FIFOE        BIT(25)
#define X2_SPI_DW_SEL          BIT(26)
#define X2_SPI_SAMP_SEL        BIT(27)
/* X2 SPI SFSR bit Masks */
#define X2_SPI_TIP_MASK        BIT(8)
#define X2_SPI_DATA_RDY        BIT(4)
#define X2_SPI_TF_EMPTY        BIT(5)
/* X2 SPI INT bit Masks */
#define X2_SPI_INT_ALL         0x000007FF
#define X2_SPI_INT_DR          BIT(0)
#define X2_SPI_INT_OE          BIT(1)
#define X2_SPI_INT_TEMPTY      BIT(2)
#define X2_SPI_INT_RX_DMAERR   BIT(3)
#define X2_SPI_INT_TX_DMAERR   BIT(4)
#define X2_SPI_INT_RX_TIMEOUT  BIT(5)
#define X2_SPI_INT_RX_FULL     BIT(6)
#define X2_SPI_INT_TX_EMPTY    BIT(7)
#define X2_SPI_INT_RX_BGDONE   BIT(8)
#define X2_SPI_INT_TX_BGDONE   BIT(9)
#define X2_SPI_INT_DMA_TRDONE  BIT(10)
/* X2 SPI FIFO_RESET bit Masks */
#define X2_SPI_FRST_TXCLEAR    BIT(0)
#define X2_SPI_FRST_RXCLEAR    BIT(1)
#define X2_SPI_FRST_RXDIS      BIT(6)
#define X2_SPI_FRST_TXDIS      BIT(7)
#define X2_SPI_FRST_ABORT      BIT(8)
/* X2 SPI DMA_CTRL0 bit Masks */
#define X2_SPI_DMAC0_RXMAXOS0  BIT(3)
#define X2_SPI_DMAC0_RXMAXOS1  BIT(4)
#define X2_SPI_DMAC0_RXAPBSEL  BIT(6)
#define X2_SPI_DMAC0_TXMAXOS0  BIT(12)
#define X2_SPI_DMAC0_TXMAXOS1  BIT(13)
#define X2_SPI_DMAC0_TXAPBSEL  BIT(15)
/* X2 SPI DMA_CTRL0 bit Masks */
#define X2_SPI_DMAC1_RXDSTART  BIT(0)
#define X2_SPI_DMAC1_TXDSTART  BIT(5)
/* X2 config Macro */
#define msecs_to_loops(t)      (loops_per_jiffy / 1000 * HZ * t)
#define X2_SPI_TIMEOUT         (msecs_to_jiffies(1000))
#define X2_SPI_FREQ_CLK        (187500000)
#define X2_SPI_FIFO_SIZE       (32)
/* X2 SPI operation Macro */
#define X2_SPI_OP_CORE_EN      1
#define X2_SPI_OP_CORE_DIS     0
#define X2_SPI_OP_NONE         (-1)
#define X2_SPI_OP_RX_FIFOE     BIT(0)
#define X2_SPI_OP_TX_FIFOE     BIT(1)
#define X2_SPI_OP_TR_FIFOE     (X2_SPI_OP_RX_FIFOE | X2_SPI_OP_TX_FIFOE)
#define X2_SPI_OP_RX_DIS       BIT(0)
#define X2_SPI_OP_TX_DIS       BIT(1)
#define X2_SPI_OP_TR_DIS       (X2_SPI_OP_RX_DIS | X2_SPI_OP_TX_DIS)

#define x2_spi_rd(dev, reg)       ioread32((dev)->regs_base + (reg))
#define x2_spi_wr(dev, reg, val)  iowrite32((val), (dev)->regs_base + (reg))

struct x2_spi {
    void __iomem *regs_base; /* virt. address of the control registers */
    int irq;                 /* spi interrupt */
    const u8 *txbuf;         /* pointer in the Tx buffer */
    u8 *rxbuf;               /* pointer in the Rx buffer */
    int len;                 /* Number of bytes left to transfer */
	int txcnt;				/* count to be Tx */
	int rxcnt;				/* count to be Rx */
	u16 word_width;			/* Bits per word:16bit or 8bit */
	u16 mode;				/* current mode */
	u32 speed;				/* current speed */
	u32 freq_clk;			/* freq clock */
	struct reset_control *rst; /* reset controller */
	struct completion completion; /* xfer completion */
};

/* spi enable with: tx/rx fifo enable? tx/rx check disable? */
static int x2_spi_en_ctrl(struct x2_spi *x2spi, int en_flag,
		int fifo_flag, int tr_flag)
{
    u32 val = 0;

    val = x2_spi_rd(x2spi, X2_SPI_CTRL_REG);
	if (fifo_flag >= 0) {
		if (fifo_flag & X2_SPI_OP_TX_FIFOE)
			val |= X2_SPI_TX_FIFOE;
		else
			val &= (~X2_SPI_TX_FIFOE);
		if (fifo_flag & X2_SPI_OP_RX_FIFOE)
			val |= X2_SPI_RX_FIFOE;
		else
			val &= (~X2_SPI_RX_FIFOE);
	}
	if (tr_flag >= 0) {
		if (tr_flag & X2_SPI_OP_TX_DIS)
			val |= X2_SPI_TX_DIS;
		else
			val &= (~X2_SPI_TX_DIS);
		if (tr_flag & X2_SPI_OP_RX_DIS)
			val |= X2_SPI_RX_DIS;
		else
			val &= (~X2_SPI_RX_DIS);
	}
    if (en_flag)
        val |= X2_SPI_CORE_EN;
    else
        val &= (~X2_SPI_CORE_EN);

	pr_debug("%s X2_SPI_CTRL_REG=0x%X\n", __func__, val);
    x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);

    return 0;
}

/* spi tx fifo fill from txbuf with txcnt */
static int x2_spi_fill_txfifo(struct x2_spi *x2spi)
{
	int cnt = 0;

	if (x2spi->txbuf) {
		if (x2spi->word_width == 16) {
			while (x2spi->txcnt > 0 && cnt < X2_SPI_FIFO_SIZE) {
				x2_spi_wr(x2spi, X2_SPI_TXD_REG,
			((*x2spi->txbuf) << 8) + *(x2spi->txbuf + 1));
				x2spi->txbuf += 2;
				x2spi->txcnt -= 2;
				cnt += 2;
			}
		} else {
			while (x2spi->txcnt > 0 && cnt < X2_SPI_FIFO_SIZE) {
				x2_spi_wr(x2spi, X2_SPI_TXD_REG, *x2spi->txbuf);
				x2spi->txbuf++;
				x2spi->txcnt--;
				cnt++;
			}
		}
	} else { /* trigger it when no tx */
		x2_spi_wr(x2spi, X2_SPI_TXD_REG, 0);
		cnt = 1;
	}

	x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_TXDIS);
	pr_debug("%s cnt=%d\n", __func__, cnt);
	return cnt;
}

/* spi rx fifo drain to rxbuf with rxcnt */
static int x2_spi_drain_rxfifo(struct x2_spi *x2spi)
{
	int cnt = 0, rd;

	if (x2spi->rxbuf) {
		if (x2spi->word_width == 16) {
			while (x2spi->rxcnt > 0 && cnt < X2_SPI_FIFO_SIZE) {
				rd = x2_spi_rd(x2spi, X2_SPI_RXD_REG);
				*x2spi->rxbuf = rd >> 8;
				*(x2spi->rxbuf + 1) = rd;
				x2spi->rxbuf += 2;
				x2spi->rxcnt -= 2;
				cnt += 2;
			}
		} else {
			while (x2spi->rxcnt > 0 && cnt < X2_SPI_FIFO_SIZE) {
				*x2spi->rxbuf = x2_spi_rd(x2spi,
						X2_SPI_RXD_REG);
				x2spi->rxbuf++;
				x2spi->rxcnt--;
				cnt++;
			}
		}
		x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_RXDIS);
	}

	pr_debug("%s cnt=%d\n", __func__, cnt);
	return cnt;
}

/* spi interrupt handle */
static irqreturn_t x2_spi_int_handle(int irq, void *data)
{
	u32 pnd;
	struct x2_spi *x2spi = (struct x2_spi *)data;

	pnd = x2_spi_rd(x2spi, X2_SPI_SRCPND_REG);
	if (pnd & (X2_SPI_INT_RX_FULL | X2_SPI_INT_DMA_TRDONE))
		x2_spi_drain_rxfifo(x2spi);
	if (pnd & X2_SPI_INT_TX_EMPTY)
		x2_spi_fill_txfifo(x2spi);

	if (pnd)
		x2_spi_wr(x2spi, X2_SPI_SRCPND_REG, pnd);

	if (pnd & X2_SPI_INT_DMA_TRDONE) {
		x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_ABORT);
		complete(&x2spi->completion);
	}

	return IRQ_HANDLED;
}

/* spi config: word_width, mode(pol, pha, lsb, cshigh), speed */
static int x2_spi_config(struct x2_spi *x2spi)
{
	u32 val;
	int divider = 0;

	val = x2_spi_rd(x2spi, X2_SPI_CTRL_REG);
	/* bit width */
	val &= ~(X2_SPI_DW_SEL | X2_SPI_POLARITY | X2_SPI_PHASE |
			 SPI_LSB_FIRST | X2_SPI_SSAL);
	if (x2spi->word_width == 16)
		val |= X2_SPI_DW_SEL;
	/* spi mode */
	if (x2spi->mode & SPI_CPOL)
		val |= X2_SPI_POLARITY;
	if (x2spi->mode & SPI_CPHA)
		val |= X2_SPI_PHASE;
	if (x2spi->mode & SPI_LSB_FIRST)
		val |= X2_SPI_LSB;
	if (x2spi->mode & SPI_CS_HIGH)
		val |= X2_SPI_SSAL;
	/* spi speed */
	if (x2spi->speed) {
		divider = x2spi->freq_clk / x2spi->speed;
		divider = (divider < 2) ? 2 : divider;
		divider = (divider % 2) ? (divider / 2) : (divider / 2 - 1);
		val &= ~X2_SPI_DIVIDER_MASK;
		val |= (divider << X2_SPI_DIVIDER_OFFSET) & X2_SPI_DIVIDER_MASK;
	}

	pr_debug("%s X2_SPI_CTRL_REG=0x%X\n", __func__, val);
	x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);

	return 0;
}

/* spi master setup */
static int x2_spi_setup(struct spi_device *spi)
{
	struct x2_spi *x2spi = spi_master_get_devdata(spi->master);

	x2spi->word_width = spi->bits_per_word;
	x2spi->mode = spi->mode;
	x2spi->speed = spi->max_speed_hz;

	return x2_spi_config(x2spi);
}

/* spi hw reset */
static int x2_spi_reset(struct x2_spi *x2spi)
{
	reset_control_assert(x2spi->rst);
	udelay(2);
	reset_control_deassert(x2spi->rst);
	return 0;
}

/* spi hw init */
static void x2_spi_init_hw(struct x2_spi *x2spi)
{
    u32 val = 0;

    /* First, should reset the whole controller */
	x2_spi_reset(x2spi);

	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_DIS, X2_SPI_OP_NONE,
			X2_SPI_OP_NONE);
    x2_spi_wr(x2spi, X2_SPI_INTSETMASK_REG, X2_SPI_INT_ALL); /* disable all interrupt */
    x2_spi_wr(x2spi, X2_SPI_SRCPND_REG, X2_SPI_INT_ALL); /* clear all interrupt pending */
	/* init rfto */
	x2_spi_wr(x2spi, X2_SPI_RFTO_REG, 0x27F);
	/* no instruction */
	x2_spi_wr(x2spi, X2_SPI_INST_REG, 0x0);
	x2_spi_wr(x2spi, X2_SPI_INST_MASK_REG, 0xFFFFFFFF);
	/* spi master mode */
    val = x2_spi_rd(x2spi, X2_SPI_CTRL_REG);
	val &= (~X2_SPI_SLAVE_MODE);
    val &= (~X2_SPI_SAMP_SEL);
    x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);
	x2_spi_config(x2spi);
	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_EN, 0, 0);

    return;
}

/* This function enables SPI master controller */
static int x2_spi_prepare_xfer_hardware(struct spi_master *master)
{
    struct x2_spi *x2spi = spi_master_get_devdata(master);

	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_EN, X2_SPI_OP_NONE,
			X2_SPI_OP_NONE);

    return 0;
}

static int x2_spi_unprepare_xfer_hardware(struct spi_master *master)
{
    struct x2_spi *x2spi = spi_master_get_devdata(master);

	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_DIS, X2_SPI_OP_NONE,
			X2_SPI_OP_NONE);

    return 0;
}

/**
 * x2_spi_chipselect - Select or deselect the chip select line
 * @spi:     Pointer to the spi_device structure
 * @is_high: Select(0) or deselect (1) the chip select line
 */
static void x2_spi_chipselect(struct spi_device *spi, bool is_high)
{
    u32 val = 0;
    struct x2_spi *x2spi = spi_master_get_devdata(spi->master);

    val = x2_spi_rd(x2spi, X2_SPI_CTRL_REG);
    if (is_high) {
        val |= X2_SPI_SS_MASK;
    } else {
        val &= (~X2_SPI_SS_MASK);
        val |= (spi->chip_select<<X2_SPI_SS_OFFSET);
    }
	pr_debug("%s X2_SPI_CTRL_REG=0x%X\n", __func__, val);
    x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);

    return;
}

/* spi wait for tpi before xfer: return 0 timeout */
static u32 x2_spi_wait_for_tpi(struct x2_spi *x2spi, int timeout_ms)
{
	u32 loop = 1;
	u32 status;

	if (timeout_ms)
		loop = msecs_to_loops(timeout_ms);

	do {
		status = x2_spi_rd(x2spi, X2_SPI_SFSR_REG);
	} while ((status & X2_SPI_TIP_MASK) && --loop);
	if (loop == 0 && ((status & X2_SPI_TIP_MASK) == 0))
		loop = 1;

	pr_debug("%s X2_SPI_SFSR_REG=0x%X\n", __func__, status);
	return loop;
}

/* spi xfer one */
static int x2_spi_transfer_one(struct spi_master *master, struct spi_device *spi,
                                       struct spi_transfer *xfer)
{
	u32 time_left;
	u32 fifo_flag = 0, tr_flag = 0;
	u32 int_umask = 0, dma_ctrl0 = 0, dma_ctrl1 = 0, fifo_reset = 0;
    struct x2_spi *x2spi = spi_master_get_devdata(master);

	if (x2_spi_wait_for_tpi(x2spi, 1) == 0)
		return -EAGAIN;
	reinit_completion(&x2spi->completion);

	if (x2spi->word_width != xfer->bits_per_word ||
		x2spi->speed != xfer->speed_hz) {
		x2spi->word_width = xfer->bits_per_word;
		x2spi->speed = xfer->speed_hz;
		x2spi->mode = spi->mode;
		x2_spi_config(x2spi);
	}

	/* prepare transfer with FIFO no-DMA */
    x2spi->txbuf = (u8 *)xfer->tx_buf;
    x2spi->rxbuf = (u8 *)xfer->rx_buf;
    x2spi->len   = xfer->len;
	if (x2spi->txbuf) {
		x2spi->txcnt = x2spi->len;
		fifo_flag |= X2_SPI_OP_TX_FIFOE;
		int_umask |= X2_SPI_INT_TX_EMPTY | X2_SPI_INT_DMA_TRDONE;
		dma_ctrl0 |= X2_SPI_DMAC0_TXAPBSEL | X2_SPI_DMAC0_TXMAXOS0 |
			X2_SPI_DMAC0_RXMAXOS0;
		dma_ctrl1 |= X2_SPI_DMAC1_TXDSTART;
		fifo_reset |= X2_SPI_FRST_TXCLEAR;
	} else {
		x2spi->txcnt = 0;
		tr_flag |= X2_SPI_OP_TX_DIS;
	}
	if (x2spi->rxbuf) {
		x2spi->rxcnt = x2spi->len;
		fifo_flag |= X2_SPI_OP_RX_FIFOE;
		int_umask |= X2_SPI_INT_RX_FULL | X2_SPI_INT_DMA_TRDONE;
		dma_ctrl0 |= X2_SPI_DMAC0_RXAPBSEL | X2_SPI_DMAC0_TXMAXOS0 |
			X2_SPI_DMAC0_RXMAXOS0;
		dma_ctrl1 |= X2_SPI_DMAC1_RXDSTART;
		fifo_reset |= X2_SPI_FRST_RXCLEAR;
	} else {
		x2spi->rxcnt = 0;
		tr_flag |= X2_SPI_OP_RX_DIS;
	}
	x2_spi_wr(x2spi, X2_SPI_DMA_CTRL0_REG, dma_ctrl0);
	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_EN, fifo_flag, tr_flag);
	x2_spi_wr(x2spi, X2_SPI_TLEN_REG, x2spi->len * 8);
	x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, fifo_reset);
	x2_spi_wr(x2spi, X2_SPI_SRCPND_REG, X2_SPI_INT_ALL);
	/* start transfer */
	pr_debug("%s len=%d txcnt=%d rxcnt=%d\n", __func__,
			 x2spi->len, x2spi->txcnt, x2spi->rxcnt);
	x2_spi_wr(x2spi, X2_SPI_DMA_CTRL1_REG, dma_ctrl1);
	x2_spi_fill_txfifo(x2spi);

	x2_spi_wr(x2spi, X2_SPI_INTUNMASK_REG, int_umask);
	time_left = wait_for_completion_timeout(&x2spi->completion,
					  X2_SPI_TIMEOUT);
	if (x2_spi_wait_for_tpi(x2spi, 1) == 0)
		x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_ABORT);
	x2_spi_wr(x2spi, X2_SPI_INTSETMASK_REG, X2_SPI_INT_ALL);
	if (time_left == 0)
		return -EAGAIN;

	spi_finalize_current_transfer(master);
    return xfer->len;
}

static int x2_spi_probe(struct platform_device *pdev)
{
	int err, spi_id;
    struct x2_spi *x2spi;
    struct spi_master *master;
    struct resource *res;
	char spi_name[20];

    dev_info(&pdev->dev, "enter x2_spi_probe!\n");
    x2spi = devm_kzalloc(&pdev->dev, sizeof(*x2spi), GFP_KERNEL);
    if (!x2spi)
        return -ENOMEM;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    x2spi->regs_base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(x2spi->regs_base))
        return PTR_ERR(x2spi->regs_base);

	spi_id = of_alias_get_id(pdev->dev.of_node, "spi");
	sprintf(spi_name, "spi%d", spi_id);
	x2spi->rst = devm_reset_control_get(&pdev->dev, spi_name);
	if (IS_ERR(x2spi->rst)) {
		dev_err(&pdev->dev, "missing controller reset %s\n", spi_name);
		return PTR_ERR(x2spi->rst);
	}
	err = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				   &x2spi->freq_clk);
	if (err < 0)
		x2spi->freq_clk = X2_SPI_FREQ_CLK;
    x2_spi_init_hw(x2spi);

    x2spi->irq = platform_get_irq(pdev, 0);
    if (x2spi->irq < 0) {
	dev_err(&pdev->dev, "Can't get interrupt resource!\n");
        return x2spi->irq;
    }
	init_completion(&x2spi->completion);
	err = devm_request_irq(&pdev->dev, x2spi->irq, x2_spi_int_handle,
			   0, pdev->name, x2spi);
	if (err < 0) {
		dev_err(&pdev->dev, "can't request interrupt\n");
		return err;
    }

    master = spi_alloc_master(&pdev->dev, sizeof(*x2spi));
    if (master == NULL) {
        dev_err(&pdev->dev, "Unable to allocate SPI Master!\n");
        return -ENOMEM;
    }
    spi_master_set_devdata(master, x2spi);

    master->bus_num = pdev->id;
    master->num_chipselect = X2_SPI_MAX_CS;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST | SPI_CS_HIGH;
	master->setup = x2_spi_setup;
    master->prepare_transfer_hardware = x2_spi_prepare_xfer_hardware;
    master->transfer_one = x2_spi_transfer_one;
    master->unprepare_transfer_hardware = x2_spi_unprepare_xfer_hardware;
    master->set_cs = x2_spi_chipselect;
    master->dev.of_node = pdev->dev.of_node;
    platform_set_drvdata(pdev, master);

    /* register spi controller */
    err = devm_spi_register_master(&pdev->dev, master);
    if (err) {
        dev_err(&pdev->dev, "spi register master failed!\n");
        spi_master_put(master);
        return err;
    }

    return 0;
}

static int x2_spi_remove(struct platform_device *pdev)
{
    struct spi_master *master = platform_get_drvdata(pdev);
    struct x2_spi *x2spi = spi_master_get_devdata(master);

	free_irq(x2spi->irq, x2spi);
	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_DIS, X2_SPI_OP_NONE,
			X2_SPI_OP_NONE);
    spi_unregister_master(master);

    return 0;
}

static const struct of_device_id x2_spi_of_match[] = {
    { .compatible = "hobot,x2-spi" },
    { /* end of table */ }
};
MODULE_DEVICE_TABLE(of, x2_spi_of_match);

static struct platform_driver x2_spi_driver = {
    .probe = x2_spi_probe,
    .remove = x2_spi_remove,
    .driver = {
        .name = X2_SPI_NAME,
        .of_match_table = x2_spi_of_match,
    },
};
module_platform_driver(x2_spi_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("X2 SPI driver");
MODULE_LICENSE("GPL v2");
