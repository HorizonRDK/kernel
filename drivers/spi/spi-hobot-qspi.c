/*
 * Hobot HB QSPI controller driver(master mode)
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
#include <linux/mtd/spinand.h>
#include <linux/completion.h>
#include <linux/spi/spi-hobot-qspi.h>

#define HB_QSPI_NAME                "hb_qspi_nand"

/* Uncomment the following macro definition to turn on debugging */
// #define QSPI_DEBUG

#ifdef CONFIG_HOBOT_FPGA_X3
#define	CONFIG_HOBOT_QSPI_REF_CLK 10000000
#define	CONFIG_HOBOT_QSPI_CLK 5000000
#endif

struct hb_qspi;

struct hb_qspi_flash_pdata {
	struct hb_qspi *xqspi;
	u8 inst_width;		/* Instruction always in single line mode */
	u8 addr_width;		/* addr width is option 24bit and 32bit */
	u8 data_width;		/* data width is 1, 2, 4 line mode */
	u8 cs;			/* current cs */
};

struct hb_qspi {
	void __iomem *regs;
	struct platform_device *pdev;
	struct completion transfer_complete;

#ifndef CONFIG_HOBOT_FPGA_X3
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
	struct hb_qspi_flash_pdata flash_pdata[HB_QSPI_MAX_CS];
};

/**
 * hb_qspi_read:	For QSPI controller read operation
 * @xqspi:	Pointer to the hb_qspi structure
 * @offset:	Offset from where to read
 */
static u32 hb_qspi_read(struct hb_qspi *xqspi, u32 offset)
{
	return readl_relaxed(xqspi->regs + offset);
}

/**
 * hb_qspi_write:	For QSPI controller write operation
 * @xqspi:	Pointer to the hb_qspi structure
 * @offset:	Offset where to write
 * @val:	Value to be written
 */
static void hb_qspi_write(struct hb_qspi *xqspi, u32 offset, u32 val)
{
	writel_relaxed(val, (xqspi->regs + offset));
}

int hb_qspi_poll_rx_empty(struct hb_qspi *xqspi,
			  u32 offset, uint32_t mask, uint32_t timeout)
{
	uint32_t reg_val;
	uint32_t ret = -1;

	while (timeout--) {
		reg_val = hb_qspi_read(xqspi, offset);
		ndelay(10);
		if (!(reg_val & mask)) {
			ret = 0;
			break;
		}
	}
	return ret;
}

static int qspi_check_status(struct hb_qspi *xqspi, u32 offset, uint32_t mask,
			     uint32_t timeout)
{
	int ret = 0;
	uint32_t val;

	do {
		val = hb_qspi_read(xqspi, offset);
		ndelay(10);
		timeout = timeout - 1;
		if (timeout == 0) {
			ret = -1;
			break;
		}
	} while (!(val & mask));

	return ret;
}

int hb_qspi_cfg_line_mode(struct hb_qspi *xqspi, uint32_t line_mode,
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
			val = hb_qspi_read(xqspi, HB_QSPI_DQM_REG);
			val |= HB_QSPI_INST_DUAL;
			hb_qspi_write(xqspi, HB_QSPI_DQM_REG, val);
		}

		if ((line_mode == SPI_RX_QUAD) || (line_mode == SPI_TX_QUAD)) {
			val = hb_qspi_read(xqspi, HB_QSPI_DQM_REG);
			val &=
			    ~(HB_QSPI_HOLD_OUTPUT | HB_QSPI_HOLD_OE | HB_QSPI_HOLD_CTL
			      | HB_QSPI_WP_OUTPUT | HB_QSPI_WP_OE | HB_QSPI_WP_CTL
				  | HB_QSPI_INST_DUAL);
			val |= HB_QSPI_INST_QUAD;
			hb_qspi_write(xqspi, HB_QSPI_DQM_REG, val);
		}
	} else {
		val = hb_qspi_read(xqspi, HB_QSPI_DQM_REG);
		val |=
		    (HB_QSPI_HOLD_OUTPUT | HB_QSPI_HOLD_OE | HB_QSPI_HOLD_CTL
			| HB_QSPI_WP_OUTPUT | HB_QSPI_WP_OE | HB_QSPI_WP_CTL);
		val &= ~(HB_QSPI_INST_QUAD | HB_QSPI_INST_DUAL);
		hb_qspi_write(xqspi, HB_QSPI_DQM_REG, val);
	}

	return ret;
}

static void hb_qspi_chipselect(struct spi_device *qspi, bool is_high)
{
	struct hb_qspi *xqspi = spi_master_get_devdata(qspi->master);

	if (is_high) {
		/* Deselect the slave */
		hb_qspi_write(xqspi, HB_QSPI_CS_REG, 0);
	} else {
		/* Activate the chip select */
		hb_qspi_write(xqspi, HB_QSPI_CS_REG, 1);
	}
}

void qspi_enable_tx(struct hb_qspi *xqspi)
{
	uint32_t val;

	val = hb_qspi_read(xqspi, HB_QSPI_CTL1_REG);
	val |= HB_QSPI_TX_EN;
	hb_qspi_write(xqspi, HB_QSPI_CTL1_REG, val);
}

void qspi_disable_tx(struct hb_qspi *xqspi)
{
	uint32_t val;

	val = hb_qspi_read(xqspi, HB_QSPI_CTL1_REG);
	val &= ~HB_QSPI_TX_EN;
	hb_qspi_write(xqspi, HB_QSPI_CTL1_REG, val);
}

void qspi_enable_rx(struct hb_qspi *xqspi)
{
	uint32_t val;

	val = hb_qspi_read(xqspi, HB_QSPI_CTL1_REG);
	val |= HB_QSPI_RX_EN;
	hb_qspi_write(xqspi, HB_QSPI_CTL1_REG, val);
}

void qspi_disable_rx(struct hb_qspi *xqspi)
{
	uint32_t val;

	val = hb_qspi_read(xqspi, HB_QSPI_CTL1_REG);
	val &= ~HB_QSPI_RX_EN;
	hb_qspi_write(xqspi, HB_QSPI_CTL1_REG, val);
}

void qspi_batch_mode_set(struct hb_qspi *xqspi, bool enable)
{
	uint32_t val;

	val = hb_qspi_read(xqspi, HB_QSPI_CTL3_REG);
	if (enable)
		val &= ~HB_QSPI_BATCH_DIS;
	else
		val |= HB_QSPI_BATCH_DIS;
	hb_qspi_write(xqspi, HB_QSPI_CTL3_REG, val);
}

void qspi_batch_tx_rx_flag_clr(struct hb_qspi *xqspi, bool is_tx)
{
	uint32_t val;

	val = hb_qspi_read(xqspi, HB_QSPI_ST1_REG);
	if (is_tx)
		val |= HB_QSPI_TBD;
	else
		val |= HB_QSPI_RBD;
	hb_qspi_write(xqspi, HB_QSPI_ST1_REG, val);
}

void qspi_reset_fifo(struct hb_qspi *xqspi)
{
	uint32_t val;

	val = hb_qspi_read(xqspi, HB_QSPI_CTL3_REG);
	val |= HB_QSPI_RST_FIFO;
	hb_qspi_write(xqspi, HB_QSPI_CTL3_REG, val);

	val &= (~HB_QSPI_RST_FIFO);
	hb_qspi_write(xqspi, HB_QSPI_CTL3_REG, val);
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
void qspi_dump_reg(struct hb_qspi *xqspi)
{
	uint32_t val = 0, i;

	for (i = HB_QSPI_DAT_REG; i <= HB_QSPI_XIP_REG; i = i + 4) {
		val = hb_qspi_read(xqspi, i);
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

	snprintf(prbuf, __TRACE_BUF_SIZE__, "%s-%s[B:%d][L:%d] ",
		transfer->rx_buf ? "<" : "", transfer->tx_buf ? ">" : "",
		nbits, transfer->len);

	if(transfer->len) {
		snprintf(tmp_prbuf, sizeof(tmp_prbuf), " ");
		strcat(prbuf, tmp_prbuf);
#define QSPI_DEBUG_DATA_LEN	16
		for (i = 0; i < ((transfer->len < QSPI_DEBUG_DATA_LEN) ?
			transfer->len : QSPI_DEBUG_DATA_LEN); i++) {
			snprintf(tmp_prbuf, 32, "%02X ", tmpbuf[i]);
			strcat(prbuf, tmp_prbuf);
		}
	}
	printk("%s\n", prbuf);
	kfree(prbuf);
}

extern const char *__clk_get_name(const struct clk *clk);
void trace_xqspi(struct hb_qspi *xqspi)
{
	printk("struct hb_qspi *xqspi(0x%16X) = {\n", xqspi);
	printk("\t\t.regs[Phy] = 0x%0p\n", xqspi->regs);
	if(xqspi->pdev) {
		printk(".pdev = %s\n", dev_name(&xqspi->pdev->dev));
	}
#ifndef CONFIG_HOBOT_FPGA_X3
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

static void hb_qspi_hw_init(struct hb_qspi *xqspi)
{
	uint32_t qspi_div;
	uint32_t reg_val;

	/* set qspi clk div */
#ifndef CONFIG_HOBOT_FPGA_X3
	qspi_div = caculate_qspi_divider(xqspi->ref_clk, xqspi->speed_hz);
	hb_qspi_write(xqspi, HB_QSPI_BDR_REG, qspi_div);
#else
	qspi_div = caculate_qspi_divider(xqspi->ref_clk, xqspi->qspi_clk);
	hb_qspi_write(xqspi, HB_QSPI_BDR_REG, qspi_div);
#endif

	/* set qspi work mode */
	reg_val = hb_qspi_read(xqspi, HB_QSPI_CTL1_REG);
	reg_val |= ((SPI_MODE0 & 0x30) | HB_QSPI_MST | HB_QSPI_FW8 | MSB);
	hb_qspi_write(xqspi, HB_QSPI_CTL1_REG, reg_val);

	hb_qspi_write(xqspi, HB_QSPI_TTL_REG, HB_QSPI_TRIG_LEVEL);
	hb_qspi_write(xqspi, HB_QSPI_RTL_REG, HB_QSPI_TRIG_LEVEL);

	/* Disable all interrupt */
	hb_qspi_write(xqspi, HB_QSPI_CTL2_REG, 0x0);

	/* unselect chip */
	reg_val = hb_qspi_read(xqspi, HB_QSPI_CS_REG);
	reg_val &= 0x0;
	hb_qspi_write(xqspi, HB_QSPI_CS_REG, reg_val);

	/* Always set SPI to one line as init. */
	reg_val = hb_qspi_read(xqspi, HB_QSPI_DQM_REG);
	reg_val |= 0xfc;
	hb_qspi_write(xqspi, HB_QSPI_DQM_REG, reg_val);

	/* Disable hardware xip mode */
	reg_val = hb_qspi_read(xqspi, HB_QSPI_XIP_REG);
	reg_val &= ~(1 << 1);
	hb_qspi_write(xqspi, HB_QSPI_XIP_REG, reg_val);
}

static int hb_qspi_setup(struct spi_device *qspi)
{
	int ret = 0;
	struct hb_qspi *xqspi = spi_master_get_devdata(qspi->master);

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

int hb_qspi_write_data(struct hb_qspi *xqspi, const void *txbuf, uint32_t len)
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

	/* Enable batch mode */
	qspi_batch_mode_set(xqspi, xqspi->batch_mode);

	while (remain_len > 0) {
		tx_len = MIN(remain_len, BATCH_MAX_CNT);
		if (xqspi->batch_mode) {
			/* clear HB_QSPI_TBD bit */
			qspi_batch_tx_rx_flag_clr(xqspi, 1);
			/* set batch cnt */
			hb_qspi_write(xqspi, HB_QSPI_TBC_REG, tx_len);
		}
		tmp_txlen = MIN(tx_len, HB_QSPI_FIFO_DEPTH);
		for (i = 0; i < tmp_txlen; i++)
			hb_qspi_write(xqspi, HB_QSPI_DAT_REG, ptr[i]);

		qspi_enable_tx(xqspi);
		err = qspi_check_status(xqspi, HB_QSPI_ST2_REG, HB_QSPI_TX_EP, timeout);
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
	if(xqspi->batch_mode) {
		qspi_batch_tx_rx_flag_clr(xqspi, 1);
		qspi_batch_mode_set(xqspi, 0);
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

static int hb_qspi_read_data(struct hb_qspi *xqspi, uint8_t *rxbuf,
			     uint32_t len)
{
	int32_t i = 0;
	uint32_t rx_len = 0;
	uint8_t *ptr = rxbuf;
	uint32_t rx_remain = len;
	uint32_t timeout = 0x1000;
	uint32_t tmp_rxlen;
	uint32_t current_receive_len = 0;

	qspi_reset_fifo(xqspi);

	qspi_batch_mode_set(xqspi, xqspi->batch_mode);
	do {
		rx_len = MIN(rx_remain, BATCH_MAX_CNT);
		rx_remain -= rx_len;
		if(xqspi->batch_mode) {
		/* clear HB_QSPI_RBD bit */
			qspi_batch_tx_rx_flag_clr(xqspi, 0);
			hb_qspi_write(xqspi, HB_QSPI_RBC_REG, rx_len);
		}
		qspi_enable_rx(xqspi);

		while (rx_len > 0) {
			tmp_rxlen = (rx_len > HB_QSPI_TRIG_LEVEL) ? HB_QSPI_TRIG_LEVEL : rx_len;
			if (!xqspi->batch_mode) {
				if (qspi_check_status(xqspi, HB_QSPI_ST2_REG, HB_QSPI_TX_EP, timeout)) {
					pr_err("%s:%d generate read sclk failed\n", __func__, __LINE__);
						goto SPI_ERROR;
				}
				for (i = 0; i < tmp_rxlen; i++)
					hb_qspi_write(xqspi, HB_QSPI_DAT_REG, 0x00);
			}

			if (hb_qspi_poll_rx_empty(xqspi, HB_QSPI_ST2_REG, HB_QSPI_RX_EP, timeout)) {
				pr_err("%s:%d timeout no data fill into rx fifo\n", __func__, __LINE__);
#ifdef QSPI_DEBUG
					qspi_dump_reg(xqspi);
#endif
				goto SPI_ERROR;
			}
			for (i = 0; i < tmp_rxlen; i++)
				ptr[i] = hb_qspi_read(xqspi, HB_QSPI_DAT_REG);
			rx_len -= tmp_rxlen;
			ptr += tmp_rxlen;
			current_receive_len += tmp_rxlen;
		}
		if (xqspi->batch_mode) {
			if (qspi_check_status(xqspi, HB_QSPI_ST1_REG, HB_QSPI_RBD, timeout)) {
				pr_err("%s:%d timeout loop batch rx done\n", __func__, __LINE__);
#ifdef QSPI_DEBUG
				qspi_dump_reg(xqspi);
#endif
				goto SPI_ERROR;
			}
			qspi_batch_tx_rx_flag_clr(xqspi, 0);
		}
		qspi_disable_rx(xqspi);
	} while (rx_remain != 0);
	qspi_reset_fifo(xqspi);
	return len;

SPI_ERROR:
	qspi_disable_rx(xqspi);
	qspi_reset_fifo(xqspi);
	pr_err("error: spi rx current_receive_len = %d\n", current_receive_len);
	return current_receive_len;
}

/**
 * hb_qspi_start_transfer:	Initiates the QSPI transfer
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

static int hb_qspi_start_transfer(struct spi_master *master,
				  struct spi_device *qspi,
				  struct spi_transfer *transfer)
{
	u16 mode = 0;
	int transfer_len = 0;
	struct hb_qspi *xqspi = spi_master_get_devdata(master);
	unsigned remainder, residue;
	remainder = transfer->len % (HB_QSPI_TRIG_LEVEL);
	residue   = transfer->len - remainder;
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
				hb_qspi_cfg_line_mode(xqspi, SPI_TX_QUAD, true);
			else if (qspi->mode & SPI_TX_DUAL)
				hb_qspi_cfg_line_mode(xqspi, SPI_TX_DUAL, true);
		}

		if (residue > 0) {
			xqspi->batch_mode = 1;
			transfer_len += hb_qspi_write_data(xqspi, xqspi->txbuf, residue);
		}
		if (remainder > 0) {
			xqspi->batch_mode = 0;
			transfer_len += hb_qspi_write_data(xqspi, (void *)xqspi->txbuf + residue, remainder);
		}

		if (transfer->len) {
			if (qspi->mode & SPI_TX_QUAD)
				hb_qspi_cfg_line_mode(xqspi, SPI_TX_QUAD, false);
			else if (qspi->mode & SPI_TX_DUAL)
				hb_qspi_cfg_line_mode(xqspi, SPI_TX_DUAL, false);
		}
	}

	if (xqspi->rxbuf != NULL) {
		if (qspi->mode & SPI_RX_QUAD)
			hb_qspi_cfg_line_mode(xqspi, SPI_RX_QUAD, true);
		if (qspi->mode & SPI_RX_DUAL)
			hb_qspi_cfg_line_mode(xqspi, SPI_RX_DUAL, true);

		if (residue > 0) {
			xqspi->batch_mode = 1;
			transfer_len += hb_qspi_read_data(xqspi, xqspi->rxbuf, residue);
		}
		if (remainder > 0) {
			xqspi->batch_mode = 0;
			transfer_len += hb_qspi_read_data(xqspi, (void *)xqspi->rxbuf + residue, remainder);
		}

		if (qspi->mode & SPI_RX_QUAD)
			hb_qspi_cfg_line_mode(xqspi, SPI_RX_QUAD, false);
		if (qspi->mode & SPI_RX_DUAL)
			hb_qspi_cfg_line_mode(xqspi, SPI_RX_DUAL, false);

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
 * hb_prepare_transfer_hardware:	Prepares hardware for transfer.
 * @master:	Pointer to the spi_master structure which provides
 *		information about the controller.
 *
 * This function enables SPI master controller.
 *
 * Return:	0 on success; error value otherwise
 */
static int hb_prepare_transfer_hardware(struct spi_master *master)
{
	/* For X2 no used */
	return 0;
}

/**
 * hb_unprepare_transfer_hardware:	Relaxes hardware after transfer
 * @master:	Pointer to the spi_master structure which provides
 *		information about the controller.
 *
 * This function disables the SPI master controller.
 *
 * Return:	Always 0
 */
static int hb_unprepare_transfer_hardware(struct spi_master *master)
{
	/* For X2 no used */
	return 0;
}

static bool hb_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct hb_qspi *xqspi = spi_master_get_devdata(mem->spi->master);
	u32 mode = xqspi->mode;

	//program operate - for spinand_select_op_variant
	switch (op->cmd.opcode) {
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

static const struct spi_controller_mem_ops hb_mem_ops = {
	.supports_op = hb_supports_op,
};

/**
 * hb_qspi_probe:	Probe method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function initializes the driver data structures and the hardware.
 *
 * Return:	0 on success; error value otherwise
 */
static int hb_qspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct spi_master *master;
	struct hb_qspi *xqspi;
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

#ifndef CONFIG_HOBOT_FPGA_X3
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
#else
	xqspi->ref_clk = CONFIG_HOBOT_QSPI_REF_CLK;
	xqspi->qspi_clk = CONFIG_HOBOT_QSPI_CLK;
#endif

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
		master->num_chipselect = HB_QSPI_DEF_CS;
	else
		master->num_chipselect = num_cs;

	master->setup = hb_qspi_setup;
	master->set_cs = hb_qspi_chipselect;
	master->transfer_one = hb_qspi_start_transfer;
	master->prepare_transfer_hardware = hb_prepare_transfer_hardware;
	master->unprepare_transfer_hardware = hb_unprepare_transfer_hardware;
	master->max_speed_hz = xqspi->speed_hz;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_RX_DUAL | SPI_RX_QUAD |
	    SPI_TX_DUAL | SPI_TX_QUAD;

	/* QSPI controller initializations */
	hb_qspi_hw_init(xqspi);
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
#ifdef CONFIG_HOBOT_XJ2
	clk_disable_unprepare(xqspi->pclk);
#endif

remove_master:
	spi_master_put(master);
	return ret;
}

/**
 * hb_qspi_remove:	Remove method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees all resources allocated to
 * the device.
 *
 * Return:	0 Always
 */
static int hb_qspi_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
int hb_qspi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct hb_qspi *hbqspi = spi_master_get_devdata(master);

	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

	/* wait to be done */
	qspi_check_status(hbqspi, HB_QSPI_ST2_REG, HB_QSPI_TX_EP, 0x1000);
	hb_qspi_poll_rx_empty(hbqspi, HB_QSPI_ST2_REG, HB_QSPI_RX_EP, 0x1000);

	qspi_disable_tx(hbqspi);
	qspi_disable_rx(hbqspi);
#ifdef CONFIG_HOBOT_XJ2
	clk_disable_unprepare(hbqspi->pclk);
#endif
	return 0;
}

int hb_qspi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct hb_qspi *hbqspi = spi_master_get_devdata(master);

	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);
#ifdef CONFIG_HOBOT_XJ2
	clk_prepare_enable(hbqspi->pclk);
#endif
	hb_qspi_hw_init(hbqspi);

	return 0;
}
#endif

static const struct dev_pm_ops hb_qspi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hb_qspi_suspend,
			hb_qspi_resume)
};

static const struct of_device_id hb_qspi_of_match[] = {
	{.compatible = "hobot,hb-qspi",},
	{ /* End of table */ }
};

MODULE_DEVICE_TABLE(of, hb_qspi_of_match);

static struct platform_driver hb_qspi_driver = {
	.probe = hb_qspi_probe,
	.remove = hb_qspi_remove,
	.driver = {
		.name = HB_QSPI_NAME,
		.of_match_table = hb_qspi_of_match,
		.pm = &hb_qspi_dev_pm_ops,
	},
};

module_platform_driver(hb_qspi_driver);

MODULE_AUTHOR("Horizon Robotics, Inc.");
MODULE_DESCRIPTION("HB QSPI driver");
MODULE_LICENSE("GPL");
