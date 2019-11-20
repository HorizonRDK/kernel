/*
 * Hobot X2 QSPI controller driver(master mode)
 *
 * Copyright (C) 2017 - 2018 Horizon, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/mtd/spi-nor.h>
#include <linux/mtd/spinand.h>

/* Uncomment the following macro definition to turn on debugging */
// #define QSPI_DEBUG

/* Generic QSPI register offsets */
#define	QSPI_TX_RX_REG		0x00
#define	QSPI_SCLK_CON		0x04
#define	QSPI_CTL1		0x08
#define QSPI_CTL2		0x0c
#define	QSPI_CTL3		0x10
#define	QSPI_CS			0x14
#define	QSPI_STATUS1		0x18
#define	QSPI_STATUS2		0x1C
#define	QSPI_BATCH_CNT_RX       0x20
#define	QSPI_BATCH_CNT_TX       0x24
#define	QSPI_FIFO_RXTRIG_LVL    0x28
#define	QSPI_FIFO_TXTRIG_LVL	0x2C
#define	QSPI_DUAL_QUAD_MODE	0x30
#define	QSPI_XIP_CFG            0x34

/* Definition of SPI_CTRL1 */
#define	LSB				(0x40)
#define	MSB				(0x00)
#define	CPHA_H				(0x20)
#define	CPHA_L				(0x00)
#define	CPOL_H				(0x10)
#define	CPOL_L				(0x00)
#define	MST_MODE			(0x04)
#define	SLV_MODE			(0x00)

#define	TX_ENABLE			(0x08)
#define	TX_DISABLE			(0x00)
#define	RX_ENABLE			(0x80)
#define	RX_DISABLE			(0x00)

#define	FIFO_WIDTH32			(0x02)
#define	FIFO_WIDTH16			(0x01)
#define	FIFO_WIDTH8			(0x00)
#define	FIFO_DEPTH			(16)
#define	BATCH_MAX_SIZE			(0x10000)

#define	SPI_FIFO_DEPTH			(FIFO_DEPTH)
#define	SPI_BACKUP_OFFSET		(0x10000)	//64K
#define	SPI_CS0				(0x01)

#define	SPI_MODE0			(CPOL_L | CPHA_L)
#define	SPI_MODE1			(CPOL_L | CPHA_H)
#define	SPI_MODE2			(CPOL_H | CPHA_L)
#define	SPI_MODE3			(CPOL_H | CPHA_H)

/* Definition of SPI_CTRL2 */
#define	BATCH_TINT_EN			(0x1L << 7)
#define	BATCH_RINT_EN			(0x1L << 6)
#define	MODDEF				(0x1L << 3)
#define	ERR_INT_EN			(0x1l << 2)
#define	TX_INT_EN			(0x1L << 1)
#define	RX_INT_EN			(0x1L << 0)

/* Definition of SPI_CTRL3 */
#define	BATCH_DISABLE			(0x1L << 4)
#define	BATCH_ENABLE			(0x0L << 4)
#define	FIFO_RESET			(0x03)
#define	SW_RST_TX			(0x02)
#define	SW_RST_RX			(0x01)
#define	RX_DMA_EN			(0x04)
#define	TX_DMA_EN			(0x08)

/* STATUS1 */
#define	BATCH_TXDONE			(0x1L<<5)
#define	BATCH_RXDONE			(0x1L<<4)
#define	MODF_CLR			(0x1L<<3)
#define	TX_ALMOST_EMPTY			(0x1L<<1)
#define	RX_ALMOST_FULL			(0x1L<<0)

/* STATUS2 */
#define	SSN_IN				(0x1L<<7)
#define	TXFIFO_FULL			(0x1L<<5)
#define	TXFIFO_EMPTY			(0x1L<<1)
#define	RXFIFO_EMPTY			(0x1L<<4)
#define	RXFIFO_FULL			(0x1L<<0)

/* SPI_DUAL_QUAD_MODE */
#define	HOLD_OUTPUT			(0x1 << 7)
#define	HOLD_OE				(0x1 << 6)
#define	HOLD_CTL			(0x1 << 5)
#define	WP_OUTPUT			(0x1 << 4)
#define	WP_OE				(0x1 << 3)
#define	WP_CTL				(0x1 << 2)
#define	QPI_ENABLE			(0x1 << 1)
#define	DPI_ENABLE			(0x1 << 0)
#define	SPI_XFER_TX			0
#define	SPI_XFER_RX			1
#define	SPI_XFER_TXRX			2

/* SPI_XIP_CFG */
#define	XIP_EN				(0x1L<<0)
#define	FLASH_SW_CFG			(0x0L<<1)
#define	FLASH_HW_CFG			(0x1L<<1)
#define	FLASH_CTNU_MODE			(0x1<<4)
#define	DUMMY_CYCLE_0			(0x0<<5)
#define	DUMMY_CYCLE_1			(0x1<<5)
#define	DUMMY_CYCLE_2			(0x2<<5)
#define	DUMMY_CYCLE_3			(0x3<<5)
#define	DUMMY_CYCLE_4			(0x4<<5)
#define	DUMMY_CYCLE_5			(0x5<<5)
#define	DUMMY_CYCLE_6			(0x6<<5)
#define	DUMMY_CYCLE_7			(0x7<<5)

#define	XIP_FLASH_ADDR_OFFSET		0x100
/* Batch mode or no batch mode */
#define	SPI_BATCH_MODE			1
#define	SPI_NO_BATCH_MODE		0

#define	QSPI_RX_BUS_WIDTH_SINGLE	1
#define	QSPI_DEFAULT_NUM_CS		1

#define	SCLK_DIV(div, scaler)		((2 << (div)) * (scaler + 1))
#define	SCLK_VAL(div, scaler)		(((div) << 4) | ((scaler) << 0))
#define	MIN(a, b)			(((a) < (b)) ? (a) : (b))

#define	CONFIG_X2_QSPI_REF_CLK 20000000
#define	CONFIG_X2_QSPI_CLK 5000000

#define	X2_QSPI_MAX_CHIPSELECT		1

/* Instruction type */
#define	QSPI_INST_TYPE_SINGLE			0
#define	QSPI_INST_TYPE_DUAL			1
#define	QSPI_INST_TYPE_QUAD			2

struct x2_qspi;

struct x2_qspi_flash_pdata {
	struct spi_nor nor;
	struct x2_qspi *xqspi;
	u8 inst_width;		/* Instruction always in single line mode */
	u8 addr_width;		/* addr width is option 24bit and 32bit */
	u8 data_width;		/* data width is 1, 2, 4 line mode */
	u8 cs;			/* current cs */
};

struct x2_qspi {
	void __iomem *regs;
	struct platform_device *pdev;
	struct completion transfer_complete;

#ifdef CONFIG_X2_SOC
	u32 ref_clk;
	struct clk *pclk;
#else
	int ref_clk;
	int qspi_clk;
#endif
	int irq;
	struct device *dev;
	const void *txbuf;
	void *rxbuf;
	u32 speed_hz;
	u32 mode;
	bool batch_mode;
	struct x2_qspi_flash_pdata flash_pdata[X2_QSPI_MAX_CHIPSELECT];
};

/**
 * x2_qspi_read:	For QSPI controller read operation
 * @xqspi:	Pointer to the x2_qspi structure
 * @offset:	Offset from where to read
 */
static u32 x2_qspi_read(struct x2_qspi *xqspi, u32 offset)
{
	return readl_relaxed(xqspi->regs + offset);
}

/**
 * x2_qspi_write:	For QSPI controller write operation
 * @xqspi:	Pointer to the x2_qspi structure
 * @offset:	Offset where to write
 * @val:	Value to be written
 */
static void x2_qspi_write(struct x2_qspi *xqspi, u32 offset, u32 val)
{
	writel_relaxed(val, (xqspi->regs + offset));
}

int x2_qspi_poll_rx_empty(struct x2_qspi *xqspi,
			  u32 offset, uint32_t mask, uint32_t timeout)
{
	uint32_t reg_val;
	uint32_t ret = -1;

	while (timeout--) {
		reg_val = x2_qspi_read(xqspi, offset);
		udelay(10);
		if (!(reg_val & mask)) {
			ret = 0;
			break;
		}
	}
	return ret;
}

static int qspi_check_status(struct x2_qspi *xqspi, u32 offset, uint32_t mask,
			     uint32_t timeout)
{
	int ret = 0;
	uint32_t val;

	do {
		val = x2_qspi_read(xqspi, offset);
		udelay(10);
		timeout = timeout - 1;
		if (timeout == 0) {
			ret = -1;
			break;
		}
	} while (!(val & mask));

	return ret;
}

int x2_qspi_cfg_line_mode(struct x2_qspi *xqspi, uint32_t line_mode,
			  bool enable)
{
	uint32_t ret = 0, val;

	if ((xqspi == NULL) || (line_mode == 0)) {
		ret = -1;
		pr_err("%s:%d Invalid line mode\n", __func__, __LINE__);
		return ret;
	}

	if (enable) {
		if (line_mode == SPI_RX_DUAL) {
			val = x2_qspi_read(xqspi, QSPI_DUAL_QUAD_MODE);
			val |= DPI_ENABLE;
			x2_qspi_write(xqspi, QSPI_DUAL_QUAD_MODE, val);
		}

		if ((line_mode == SPI_RX_QUAD) || (line_mode == SPI_TX_QUAD)) {
			val = x2_qspi_read(xqspi, QSPI_DUAL_QUAD_MODE);
			val &=
			    ~(HOLD_OUTPUT | HOLD_OE | HOLD_CTL | WP_OUTPUT |
			      WP_OE | WP_CTL | DPI_ENABLE);
			val |= QPI_ENABLE;
			x2_qspi_write(xqspi, QSPI_DUAL_QUAD_MODE, val);
		}
	} else {
		val = x2_qspi_read(xqspi, QSPI_DUAL_QUAD_MODE);
		val |=
		    (HOLD_OUTPUT | HOLD_OE | HOLD_CTL | WP_OUTPUT | WP_OE |
		     WP_CTL);
		val &= ~(QPI_ENABLE | DPI_ENABLE);
		x2_qspi_write(xqspi, QSPI_DUAL_QUAD_MODE, val);
	}

	return ret;
}

static void x2_qspi_chipselect(struct spi_device *qspi, bool is_high)
{
	struct x2_qspi *xqspi = spi_master_get_devdata(qspi->master);

	if (is_high) {
		/* Deselect the slave */
		x2_qspi_write(xqspi, QSPI_CS, 0);
	} else {
		/* Activate the chip select */
		x2_qspi_write(xqspi, QSPI_CS, 1);
	}
}

void qspi_enable_tx(struct x2_qspi *xqspi)
{
	uint32_t val;

	val = x2_qspi_read(xqspi, QSPI_CTL1);
	val |= TX_ENABLE;
	x2_qspi_write(xqspi, QSPI_CTL1, val);
}

void qspi_disable_tx(struct x2_qspi *xqspi)
{
	uint32_t val;

	val = x2_qspi_read(xqspi, QSPI_CTL1);
	val &= ~TX_ENABLE;
	x2_qspi_write(xqspi, QSPI_CTL1, val);
}

void qspi_enable_rx(struct x2_qspi *xqspi)
{
	uint32_t val;

	val = x2_qspi_read(xqspi, QSPI_CTL1);
	val |= RX_ENABLE;
	x2_qspi_write(xqspi, QSPI_CTL1, val);
}

void qspi_disable_rx(struct x2_qspi *xqspi)
{
	uint32_t val;

	val = x2_qspi_read(xqspi, QSPI_CTL1);
	val &= ~RX_ENABLE;
	x2_qspi_write(xqspi, QSPI_CTL1, val);
}

void qspi_batch_mode_set(struct x2_qspi *xqspi, bool enable)
{
	uint32_t val;

	val = x2_qspi_read(xqspi, QSPI_CTL3);
	if (enable)
		val &= ~BATCH_DISABLE;
	else
		val |= BATCH_DISABLE;
	x2_qspi_write(xqspi, QSPI_CTL3, val);
}

void qspi_batch_tx_rx_flag_clr(struct x2_qspi *xqspi, bool is_tx)
{
	uint32_t val;

	val = x2_qspi_read(xqspi, QSPI_STATUS1);
	if (is_tx)
		val |= BATCH_TXDONE;
	else
		val |= BATCH_RXDONE;
	x2_qspi_write(xqspi, QSPI_STATUS1, val);
}

void qspi_reset_fifo(struct x2_qspi *xqspi)
{
	uint32_t val;

	val = x2_qspi_read(xqspi, QSPI_CTL3);
	val |= FIFO_RESET;
	x2_qspi_write(xqspi, QSPI_CTL3, val);

	val &= (~FIFO_RESET);
	x2_qspi_write(xqspi, QSPI_CTL3, val);
}

static uint32_t caculate_qspi_divider(uint32_t hclk, uint32_t sclk)
{
	uint32_t div = 0;
	uint32_t div_min;
	uint32_t scaler;

	/* The maxmium of prescale is 16, according to spec. */
	div_min = hclk / sclk / 16;
	if (div_min >= (1 << 16)) {
		pr_err("error: Invalid QSpi freq\r\n");
		/* Return a max scaler. */
		return SCLK_VAL(0xF, 0xF);
	}

	while (div_min >= 1) {
		div_min >>= 1;
		div++;
	}
	scaler = ((hclk / sclk) / (2 << div)) - 1;

	return SCLK_VAL(div, scaler);
}

#ifdef QSPI_DEBUG
void qspi_dump_reg(struct x2_qspi *xqspi)
{
	uint32_t val = 0, i;

	for (i = QSPI_TX_RX_REG; i <= QSPI_XIP_CFG; i = i + 4) {
		val = x2_qspi_read(xqspi, i);
		printk("reg[0x%08x] ==> [0x%08x]\n", xqspi->regs + i, val);
	}
}

void trace_transfer(const struct spi_transfer *transfer)
{
#define __TRACE_BUF_SIZE__ 128
	int i = 0;
	const u8 *tmpbuf = NULL; // kzalloc(tmpbufsize, GFP_KERNEL);
	char tmp_prbuf[32];
	char *prbuf = kzalloc(__TRACE_BUF_SIZE__, GFP_KERNEL);
	int nbits = 0;

	memset(prbuf, '\0', __TRACE_BUF_SIZE__);
	if( transfer->tx_buf ){
		nbits = transfer->tx_nbits;
		tmpbuf = (const u8 *)transfer->tx_buf;
	}else if(transfer->rx_buf){
		nbits = transfer->rx_nbits;
		tmpbuf = (const u8 *)transfer->rx_buf;
	}else{
		printk("No data\n");
		kfree(prbuf);

		return ;
	}

	snprintf(prbuf, sizeof(prbuf), "%s-%s[B:%d][L:%d] ",
		transfer->rx_buf ? "<" : "", transfer->tx_buf ? ">" : "",
		nbits, transfer->len);

	if(transfer->len) {
		snprintf(tmp_prbuf, sizeof(tmp_prbuf), " ");
		strcat(prbuf, tmp_prbuf);
#define QSPI_DEBUG_DATA_LEN	16
		for (i = 0; i < ((transfer->len < QSPI_DEBUG_DATA_LEN) ?
			transfer->len : QSPI_DEBUG_DATA_LEN); i++) {
			snprintf(tmp_prbuf, sizeof(tmp_prbuf), "%02X ", tmpbuf[i]);
			strcat(prbuf, tmp_prbuf);
		}
	}
	printk("%s\n", prbuf);
	kfree(prbuf);
}

extern const char *__clk_get_name(const struct clk *clk);
void trace_xqspi(struct x2_qspi *xqspi)
{
	printk("struct x2_qspi *xqspi(0x%16X) = {\n", xqspi);
	printk("\t\t.regs[Phy] = 0x%0p\n", xqspi->regs);
	if(xqspi->pdev) {
		printk(".pdev = %s\n", dev_name(&xqspi->pdev->dev));
	}
#ifdef CONFIG_X2_SOC
	printk("\t\t.ref_clk = %u\n", xqspi->ref_clk);
	if(xqspi->pclk) {
		printk("\t\t.pclk = %s@%u\n", __clk_get_name(xqspi->pclk),
			clk_get_rate(xqspi->pclk));
	}
#else
	printk("\t\t.ref_clk = %d\n", xqspi->ref_clk);
	printk("\t\t.qspi_clk = %d\n", xqspi->qspi_clk);
#endif
	printk("\t\t.irq = %d\n", xqspi->irq);
	printk("\t\t.dev = %s\n", dev_name(xqspi->dev));
	// printk("\t\ttxbuf: 0x%p\n", xqspi->txbuf);
	// printk("\t\trxbuf: 0x%p\n", xqspi->rxbuf);
	printk("\t\t.speed_hz = %d\n", xqspi->speed_hz);
	printk("\t\t.mode = 0x%08X\n", xqspi->mode);
	printk("\t\t.batch_mode = %d\n", xqspi->batch_mode);
	printk("}\n");
}

#endif

static void x2_qspi_hw_init(struct x2_qspi *xqspi)
{
	uint32_t qspi_div;
	uint32_t reg_val;

	/* set qspi clk div */
#ifdef CONFIG_X2_SOC
	qspi_div = caculate_qspi_divider(xqspi->ref_clk, xqspi->speed_hz);
	x2_qspi_write(xqspi, QSPI_SCLK_CON, qspi_div);
#else
	qspi_div = caculate_qspi_divider(xqspi->ref_clk, xqspi->qspi_clk);
	x2_qspi_write(xqspi, QSPI_SCLK_CON, qspi_div);
#endif

	/* set qspi work mode */
	reg_val = x2_qspi_read(xqspi, QSPI_CTL1);
	reg_val |= ((SPI_MODE0 & 0x30) | MST_MODE | FIFO_WIDTH8 | MSB);
	x2_qspi_write(xqspi, QSPI_CTL1, reg_val);

	x2_qspi_write(xqspi, QSPI_FIFO_TXTRIG_LVL, FIFO_DEPTH / 2);
	x2_qspi_write(xqspi, QSPI_FIFO_RXTRIG_LVL, FIFO_DEPTH / 2);

	/* Disable all interrupt */
	x2_qspi_write(xqspi, QSPI_CTL2, 0x0);

	/* unselect chip */
	reg_val = x2_qspi_read(xqspi, QSPI_CS);
	reg_val &= 0x0;
	x2_qspi_write(xqspi, QSPI_CS, reg_val);

	/* Always set SPI to one line as init. */
	reg_val = x2_qspi_read(xqspi, QSPI_DUAL_QUAD_MODE);
	reg_val |= 0xfc;
	x2_qspi_write(xqspi, QSPI_DUAL_QUAD_MODE, reg_val);

	/* Disable hardware xip mode */
	reg_val = x2_qspi_read(xqspi, QSPI_XIP_CFG);
	reg_val &= ~(1 << 1);
	x2_qspi_write(xqspi, QSPI_XIP_CFG, reg_val);
}

static int x2_qspi_setup(struct spi_device *qspi)
{
	int ret = 0;
	struct x2_qspi *xqspi = spi_master_get_devdata(qspi->master);

	if (qspi->master->busy)
		return -EBUSY;

	switch (xqspi->mode) {
	case SPI_NBITS_SINGLE:
		break;
	case SPI_NBITS_DUAL:
		qspi->mode |= (SPI_TX_DUAL | SPI_RX_DUAL);
		break;
	case SPI_NBITS_QUAD:
		qspi->mode |= (SPI_TX_QUAD | SPI_RX_QUAD);
		break;
	default:
		ret = -EINVAL;
		pr_err("%s nothing mode matches 0x%08X.\n", __func__, qspi->mode);
		break;
	}

	return ret;
}

int x2_qspi_write_data(struct x2_qspi *xqspi, const void *txbuf, uint32_t len)
{
	uint32_t tx_len;
	uint32_t i;
	const uint8_t *ptr = (const uint8_t *)txbuf;
	uint32_t timeout = 0x1000;
	int32_t err;
	uint32_t tmp_txlen;
	uint32_t remain_len = len;
	uint32_t current_transfer_len = 0;

	qspi_disable_tx(xqspi);
	qspi_reset_fifo(xqspi);
	if (xqspi->batch_mode) {
		/* Enable batch mode */
		qspi_batch_mode_set(xqspi, 1);

		while (remain_len > 0) {
			tx_len = MIN(remain_len, BATCH_MAX_SIZE);

			/* clear BATCH_TXDONE bit */
			qspi_batch_tx_rx_flag_clr(xqspi, 1);

			/* set batch cnt */
			x2_qspi_write(xqspi, QSPI_BATCH_CNT_TX, tx_len);
			tmp_txlen = MIN(tx_len, FIFO_DEPTH);
			for (i = 0; i < tmp_txlen; i++)
				x2_qspi_write(xqspi, QSPI_TX_RX_REG, ptr[i]);

			qspi_enable_tx(xqspi);
			err =
			    qspi_check_status(xqspi, QSPI_STATUS2, TXFIFO_EMPTY,
					      timeout);
			if (err) {
				current_transfer_len = tmp_txlen;
				pr_err("%s:%d qspi send data timeout\n",
				       __func__, __LINE__);
#ifdef QSPI_DEBUG
				qspi_dump_reg(xqspi);
#endif
				goto SPI_ERROR;
			}

			qspi_disable_tx(xqspi);
			remain_len -= tmp_txlen;
			ptr += tmp_txlen;
		}
		/* clear BATCH_TXDONE bit */
		qspi_batch_tx_rx_flag_clr(xqspi, 1);

		qspi_batch_mode_set(xqspi, 0);
	} else {
		/* Disable batch mode */
		qspi_batch_mode_set(xqspi, 0);

		while (remain_len > 0) {
			tx_len = MIN(remain_len, FIFO_DEPTH);

			for (i = 0; i < tx_len; i++)
				x2_qspi_write(xqspi, QSPI_TX_RX_REG, ptr[i]);

			qspi_enable_tx(xqspi);
			err =
			    qspi_check_status(xqspi, QSPI_STATUS2, TXFIFO_EMPTY,
					      timeout);
			if (err) {
				pr_err("%s:%d qspi send data timeout\n",
				       __func__, __LINE__);
#ifdef QSPI_DEBUG
				qspi_dump_reg(xqspi);
#endif
				goto SPI_ERROR;
			}

			qspi_disable_tx(xqspi);
			remain_len -= tx_len;
			ptr += tx_len;
		}
	}
	return len;

SPI_ERROR:
	qspi_disable_tx(xqspi);
	qspi_reset_fifo(xqspi);
	if (xqspi->batch_mode)
		qspi_batch_mode_set(xqspi, 0);
	pr_err("error: qspi tx current_transfer_len: %d\n",
	       current_transfer_len);
	return current_transfer_len;
}

static int x2_qspi_read_data(struct x2_qspi *xqspi, uint8_t *rxbuf,
			     uint32_t len)
{
	int32_t i = 0;
	uint32_t rx_len = 0;
	uint8_t *ptr = rxbuf;
	uint32_t level;
	uint32_t rx_remain = len;
	uint32_t timeout = 0x1000;
	uint32_t tmp_rxlen;
	uint32_t current_receive_len = 0;

	level = x2_qspi_read(xqspi, QSPI_FIFO_RXTRIG_LVL);
	qspi_reset_fifo(xqspi);

	if (xqspi->batch_mode) {
		qspi_batch_mode_set(xqspi, 1);
		do {
			rx_len = MIN(rx_remain, 0xFFFF);
			rx_remain -= rx_len;

			/* clear BATCH_RXDONE bit */
			qspi_batch_tx_rx_flag_clr(xqspi, 0);

			x2_qspi_write(xqspi, QSPI_BATCH_CNT_RX, rx_len);

			qspi_enable_rx(xqspi);

			while (rx_len > 0) {
				if (rx_len > level) {
					tmp_rxlen = level;
					if (x2_qspi_poll_rx_empty
					    (xqspi, QSPI_STATUS2, RXFIFO_EMPTY,
					     timeout)) {
						pr_err
						    ("%s:%d timeout no data fill into rx fifo\n",
						     __func__, __LINE__);
#ifdef QSPI_DEBUG
						qspi_dump_reg(xqspi);
#endif
						goto SPI_ERROR;
					}
					for (i = 0; i < tmp_rxlen; i++)
						ptr[i] =
						    x2_qspi_read(xqspi,
								 QSPI_TX_RX_REG);

					rx_len -= tmp_rxlen;
					ptr += tmp_rxlen;
					current_receive_len += tmp_rxlen;
				} else {
					tmp_rxlen = rx_len;
					if (x2_qspi_poll_rx_empty
					    (xqspi, QSPI_STATUS2, RXFIFO_EMPTY,
					     timeout)) {
						pr_err
						    ("%s:%d timeout no data fill into rx fifo\n",
						     __func__, __LINE__);
#ifdef QSPI_DEBUG
						qspi_dump_reg(xqspi);
#endif
						goto SPI_ERROR;
					}
					for (i = 0; i < tmp_rxlen; i++)
						ptr[i] =
						    x2_qspi_read(xqspi,
								 QSPI_TX_RX_REG);

					rx_len -= tmp_rxlen;
					ptr += tmp_rxlen;
					current_receive_len += tmp_rxlen;
				}
			}
			if (qspi_check_status
			    (xqspi, QSPI_STATUS1, BATCH_RXDONE, timeout)) {
				pr_err("%s:%d timeout loop batch rx done\n",
				       __func__, __LINE__);
#ifdef QSPI_DEBUG
				qspi_dump_reg(xqspi);
#endif
				goto SPI_ERROR;
			}

			qspi_disable_rx(xqspi);
			/* clear BATCH_RXDONE bit */
			qspi_batch_tx_rx_flag_clr(xqspi, 0);
		} while (rx_remain != 0);
	} else {
		qspi_batch_mode_set(xqspi, 0);

		do {
			rx_len = MIN(rx_remain, 0xFFFF);
			rx_remain -= rx_len;
			qspi_enable_rx(xqspi);

			while (rx_len > 0) {
				if (rx_len > level) {
					tmp_rxlen = level;
					if (qspi_check_status
					    (xqspi, QSPI_STATUS2, TXFIFO_EMPTY,
					     timeout)) {
						pr_err
						    ("%s:%d generate read sclk failed\n",
						     __func__, __LINE__);
						goto SPI_ERROR;
					}

					for (i = 0; i < tmp_rxlen; i++)
						x2_qspi_write(xqspi,
							      QSPI_TX_RX_REG,
							      0x0);

					if (x2_qspi_poll_rx_empty
					    (xqspi, QSPI_STATUS2, RXFIFO_EMPTY,
					     timeout)) {
						pr_err
						    ("%s:%d timeout no data fill into rx fifo\n",
						     __func__, __LINE__);
						goto SPI_ERROR;
					}
					for (i = 0; i < tmp_rxlen; i++)
						ptr[i] =
						    x2_qspi_read(xqspi,
								 QSPI_TX_RX_REG);

					rx_len -= tmp_rxlen;
					ptr += tmp_rxlen;
					current_receive_len += tmp_rxlen;
				} else {
					tmp_rxlen = rx_len;
					if (qspi_check_status
					    (xqspi, QSPI_STATUS2, TXFIFO_EMPTY,
					     timeout)) {
						pr_err
						    ("%s:%d generate read sclk failed\n",
						     __func__, __LINE__);
						goto SPI_ERROR;
					}

					for (i = 0; i < tmp_rxlen; i++)
						x2_qspi_write(xqspi,
							      QSPI_TX_RX_REG,
							      0x0);

					if (x2_qspi_poll_rx_empty
					    (xqspi, QSPI_STATUS2, RXFIFO_EMPTY,
					     timeout)) {
						pr_err
						    ("%s:%d timeout no data fill into rx fifo\n",
						     __func__, __LINE__);
#ifdef QSPI_DEBUG
						qspi_dump_reg(xqspi);
#endif
						goto SPI_ERROR;
					}
					for (i = 0; i < tmp_rxlen; i++)
						ptr[i] =
						    x2_qspi_read(xqspi,
								 QSPI_TX_RX_REG);

					rx_len -= tmp_rxlen;
					ptr += tmp_rxlen;
					current_receive_len += tmp_rxlen;
				}
			}
			qspi_disable_rx(xqspi);

		} while (rx_remain != 0);
	}
	qspi_reset_fifo(xqspi);
	return len;

SPI_ERROR:
	qspi_disable_rx(xqspi);
	qspi_reset_fifo(xqspi);
	pr_err("error: spi rx current_receive_len = %d\n", current_receive_len);
	return current_receive_len;
}

#if 0
static void x2_qspi_configure(struct spi_nor *nor, bool is_cmd)
{
	struct x2_qspi_flash_pdata *f_pdata = nor->priv;

	if (is_cmd) {
	}
}

static int qspi_set_protocol(struct spi_nor *nor, const int read)
{
	struct x2_qspi_flash_pdata *f_pdata = nor->priv;

	f_pdata->inst_width = CQSPI_INST_TYPE_SINGLE;
	f_pdata->addr_width = CQSPI_INST_TYPE_SINGLE;
	f_pdata->data_width = CQSPI_INST_TYPE_SINGLE;

	if (read) {
		switch (nor->read_proto) {
		case SNOR_PROTO_1_1_1:
			f_pdata->data_width = CQSPI_INST_TYPE_SINGLE;
			break;
		case SNOR_PROTO_1_1_2:
			f_pdata->data_width = CQSPI_INST_TYPE_DUAL;
			break;
		case SNOR_PROTO_1_1_4:
			f_pdata->data_width = CQSPI_INST_TYPE_QUAD;
			break;
		default:
			return -EINVAL;
		}
	}

	x2_qspi_configure(nor);

	return 0;
}

static int x2_qspi_flash_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				  int len)
{
	int ret;

	ret = x2_qspi_set_protocol(nor, 0);
	if (!ret)
		ret = cqspi_command_read(nor, &opcode, 1, buf, len);

	return ret;
}
#endif

/**
 * x2_qspi_start_transfer:	Initiates the QSPI transfer
 * @master:	Pointer to the spi_master structure which provides
 *		information about the controller.
 * @qspi:	Pointer to the spi_device structure
 * @transfer:	Pointer to the spi_transfer structure which provide information
 *		about next transfer parameters
 *
 * This function fills the TX FIFO, starts the QSPI transfer, and waits for the
 * transfer to be completed.
 *
 * Return:	Number of bytes transferred in the last transfer
 */

static int x2_qspi_start_transfer(struct spi_master *master,
				  struct spi_device *qspi,
				  struct spi_transfer *transfer)
{
	u16 mode = 0;
	int transfer_len = 0;
	struct x2_qspi *xqspi = spi_master_get_devdata(master);

	/*
	 * this 'mode' init by spi core,
	 * depending on tx-bus-width property defined in dts
	 */
	mode = qspi->mode;
	xqspi->txbuf = transfer->tx_buf;
	xqspi->rxbuf = transfer->rx_buf;

	switch (transfer->tx_nbits | transfer->rx_nbits) {
	case SPI_NBITS_DUAL:
		qspi->mode |= (SPI_TX_DUAL | SPI_RX_DUAL);
		break;
	case SPI_NBITS_QUAD:
		qspi->mode |= (SPI_TX_QUAD | SPI_RX_QUAD);
		break;
	default:
		qspi->mode &= ~(SPI_TX_DUAL | SPI_RX_DUAL |
				SPI_TX_QUAD | SPI_RX_QUAD);
		break;
	}
	if (xqspi->txbuf != NULL) {
		if (transfer->len) {
			if (qspi->mode & SPI_TX_QUAD)
				x2_qspi_cfg_line_mode(xqspi, SPI_TX_QUAD, true);
			else if (qspi->mode & SPI_TX_DUAL)
				x2_qspi_cfg_line_mode(xqspi, SPI_TX_DUAL, true);
		}
		transfer_len =
		    x2_qspi_write_data(xqspi, xqspi->txbuf, transfer->len);

		if (transfer->len) {
			if (qspi->mode & SPI_TX_QUAD)
				x2_qspi_cfg_line_mode(xqspi, SPI_TX_QUAD, false);
			else if (qspi->mode & SPI_TX_DUAL)
				x2_qspi_cfg_line_mode(xqspi, SPI_TX_DUAL, false);
		}
	}

	if (xqspi->rxbuf != NULL) {
		if (qspi->mode & SPI_RX_QUAD)
			x2_qspi_cfg_line_mode(xqspi, SPI_RX_QUAD, true);
		if (qspi->mode & SPI_RX_DUAL)
			x2_qspi_cfg_line_mode(xqspi, SPI_RX_DUAL, true);

		transfer_len =
		    x2_qspi_read_data(xqspi, xqspi->rxbuf, transfer->len);

		if (qspi->mode & SPI_RX_QUAD)
			x2_qspi_cfg_line_mode(xqspi, SPI_RX_QUAD, false);
		if (qspi->mode & SPI_RX_DUAL)
			x2_qspi_cfg_line_mode(xqspi, SPI_RX_DUAL, false);

	}

	qspi->mode = mode;
	transfer->len = transfer_len;

#ifdef QSPI_DEBUG
		// trace transfer data when transfer done
		trace_transfer(transfer);
#endif

	spi_finalize_current_transfer(master);
	return transfer->len;
}

/**
 * x2_prepare_transfer_hardware:	Prepares hardware for transfer.
 * @master:	Pointer to the spi_master structure which provides
 *		information about the controller.
 *
 * This function enables SPI master controller.
 *
 * Return:	0 on success; error value otherwise
 */
static int x2_prepare_transfer_hardware(struct spi_master *master)
{
	/* For X2 no used */
	return 0;
}

/**
 * x2_unprepare_transfer_hardware:	Relaxes hardware after transfer
 * @master:	Pointer to the spi_master structure which provides
 *		information about the controller.
 *
 * This function disables the SPI master controller.
 *
 * Return:	Always 0
 */
static int x2_unprepare_transfer_hardware(struct spi_master *master)
{
	/* For X2 no used */
	return 0;
}

static bool x2_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct x2_qspi *xqspi = spi_master_get_devdata(mem->spi->master);
	u32 mode = xqspi->mode;

	//program operate - for spinand_select_op_variant
	switch (op->cmd.opcode) {
	case SPINAND_CMD_PROG_LOAD:
	case SPINAND_CMD_PROG_LOAD_RDM_DATA:
		if (mode == SPI_NBITS_DUAL || mode == SPI_NBITS_SINGLE)
			return true;
		else
			return false;
	case SPINAND_CMD_PROG_LOAD_X4:
	case SPINAND_CMD_PROG_LOAD_RDM_DATA_X4:
		if (mode == SPI_NBITS_QUAD)
			return true;
		else
			return false;
	default:
		break;
	}

	//read operate - for spinand_select_op_variant
	if (op->data.dir == SPI_MEM_DATA_IN && op->data.buswidth == mode
	    && op->addr.buswidth == mode)
		return true;

	//other cmd - for spi_mem_exec_op
	if (!op->data.buswidth || !op->addr.buswidth || !op->dummy.buswidth)
		return true;

	return false;
}

static const struct spi_controller_mem_ops x2_mem_ops = {
	.supports_op = x2_supports_op,
};

/**
 * x2_qspi_probe:	Probe method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function initializes the driver data structures and the hardware.
 *
 * Return:	0 on success; error value otherwise
 */
static int x2_qspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct spi_master *master;
	struct x2_qspi *xqspi;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct device_node *nc;
	u32 num_cs;
	u32 max_speed_hz;

	master = spi_alloc_master(&pdev->dev, sizeof(*xqspi));
	if (!master)
		return -ENOMEM;

	xqspi = spi_master_get_devdata(master);
	master->dev.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, master);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	xqspi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(xqspi->regs)) {
		ret = PTR_ERR(xqspi->regs);
		goto remove_master;
	}

	xqspi->dev = dev;

	xqspi->pclk = devm_clk_get(&pdev->dev, "qspi_aclk");
	if (IS_ERR(xqspi->pclk)) {
		dev_err(dev, "pclk clock not found.\n");
		ret = PTR_ERR(xqspi->pclk);
		goto remove_master;
	}

	ret = clk_prepare_enable(xqspi->pclk);
	if (ret) {
		dev_err(dev, "Unable to enable APB clock.\n");
		goto clk_dis_pclk;
	}

	xqspi->ref_clk = clk_get_rate(xqspi->pclk);

	if (of_property_read_bool(pdev->dev.of_node, "is-batch-mode"))
		xqspi->batch_mode = true;

	/* used for spi-mem select op and meet __spi_validate */
	if (of_property_read_u32(pdev->dev.of_node, "qspi-mode", &xqspi->mode))
		xqspi->mode = 0;

	xqspi->speed_hz = -1;
	for_each_available_child_of_node(pdev->dev.of_node, nc) {
		ret = of_property_read_u32(nc, "spi-max-frequency",
				&max_speed_hz);
		if (!ret) {
			if (max_speed_hz < xqspi->speed_hz)
				xqspi->speed_hz = max_speed_hz;
		} else {
			dev_err(dev, "spi-max-frequency not found\n");
		}
	}
	if (ret)
		dev_err(dev, "tx/rx bus width not found\n");

	ret = of_property_read_u32(pdev->dev.of_node, "num-cs", &num_cs);
	if (ret < 0)
		master->num_chipselect = QSPI_DEFAULT_NUM_CS;
	else
		master->num_chipselect = num_cs;

	master->setup = x2_qspi_setup;
	master->set_cs = x2_qspi_chipselect;
	master->transfer_one = x2_qspi_start_transfer;
	master->prepare_transfer_hardware = x2_prepare_transfer_hardware;
	master->unprepare_transfer_hardware = x2_unprepare_transfer_hardware;
	master->max_speed_hz = xqspi->speed_hz;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_RX_DUAL | SPI_RX_QUAD |
	    SPI_TX_DUAL | SPI_TX_QUAD;

	/* QSPI controller initializations */
	x2_qspi_hw_init(xqspi);
#ifdef QSPI_DEBUG
	trace_xqspi(xqspi);
#endif

	if (master->dev.parent == NULL)
		master->dev.parent = &master->dev;

	ret = spi_register_master(master);
	if (ret)
		goto remove_master;

	return 0;

clk_dis_pclk:
	clk_disable_unprepare(xqspi->pclk);

remove_master:
	spi_master_put(master);
	return ret;
}

/**
 * x2_qspi_remove:	Remove method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees all resources allocated to
 * the device.
 *
 * Return:	0 Always
 */
static int x2_qspi_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
int x2_qspi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct x2_qspi *x2qspi = spi_master_get_devdata(master);

	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

	/* wait to be done */
	qspi_check_status(x2qspi, QSPI_STATUS2, TXFIFO_EMPTY, 0x1000);
	x2_qspi_poll_rx_empty(x2qspi, QSPI_STATUS2, RXFIFO_EMPTY, 0x1000);

	qspi_disable_tx(x2qspi);
	qspi_disable_rx(x2qspi);

	clk_disable_unprepare(x2qspi->pclk);

	return 0;
}

int x2_qspi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct x2_qspi *x2qspi = spi_master_get_devdata(master);

	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);

	clk_prepare_enable(x2qspi->pclk);

	x2_qspi_hw_init(x2qspi);

	return 0;
}
#endif

static const struct dev_pm_ops x2_qspi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(x2_qspi_suspend,
			x2_qspi_resume)
};

static const struct of_device_id x2_qspi_of_match[] = {
	{.compatible = "hobot,x2-qspi-nand", },
	{ /* End of table */ }
};

MODULE_DEVICE_TABLE(of, x2_qspi_of_match);

static struct platform_driver x2_qspi_driver = {
	.probe = x2_qspi_probe,
	.remove = x2_qspi_remove,
	.driver = {
		   .name = "x2_qspi_nand",
		   .of_match_table = x2_qspi_of_match,
		   .pm = &x2_qspi_dev_pm_ops,
		   },
};

module_platform_driver(x2_qspi_driver);

MODULE_AUTHOR("Horizon Robotics, Inc.");
MODULE_DESCRIPTION("X2 QSPI driver");
MODULE_LICENSE("GPL");
