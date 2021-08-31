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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/mtd/spinand.h>
#include <linux/completion.h>
#ifdef CONFIG_HOBOT_DIAG
#include <soc/hobot/diag.h>
#endif
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
#include <soc/hobot/hobot_bus.h>
#endif
#include "./spi-hobot-qspi.h"

#ifdef CONFIG_HOBOT_DIAG
static void hb_qspiflash_diag_report(uint8_t errsta, uint8_t sta_reg);
#endif

struct hb_qspi;

struct hb_qspi_flash_pdata {
	struct hb_qspi *hbqspi;
	uint8_t inst_width;		/* Instruction always in single line mode */
	uint8_t addr_bytes;		/* 24bit(3 byte) or 32bit(4 byte) */
	uint8_t buswidth;		/* data width available: 1, 2, 4 line*/
	uint8_t cs;			/* current cs */
};

struct hb_qspi {
	void __iomem *regs;
	struct platform_device *pdev;
	struct completion xfer_complete;
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	struct mutex xfer_lock;
	struct hobot_dpm hbqspi_dpm;
#endif
	uint32_t ref_clk;
#ifndef CONFIG_HOBOT_FPGA_X3
	struct clk *hclk;
#endif
	uint32_t sclk;
	int irq;
	struct device *dev;
	const void *txbuf;
	void *rxbuf;
	uint32_t buswidth;
	uint32_t spi_mode;
	struct hb_qspi_flash_pdata flash_pdata[HB_QSPI_MAX_CS];
};

#define hb_qspi_rd_reg(dev, reg)	   readl((dev)->regs + (reg))
#define hb_qspi_wr_reg(dev, reg, val)  writel((val), (dev)->regs + (reg))

static inline int hb_qspi_check_status(struct hb_qspi *hbqspi, uint32_t offset,
			     			 uint32_t mask, uint32_t timeout)
{
	int ret = 0, tries = timeout;
	uint32_t val;
	bool if_flip = mask & (HB_QSPI_TX_FULL | HB_QSPI_RX_EP);

	do {
		val = hb_qspi_rd_reg(hbqspi, offset);
		ndelay(1);
		val = if_flip ? ~val : val;
		timeout--;
		if (timeout == 0) {
			ret = -ETIMEDOUT;
			break;
		}
	} while (!(val & mask));

	if (ret)
		pr_err("%s:%s: status check timedout after %d tries.\n",
				__FILE__, __func__, tries);

	return ret;
}

static inline int hb_qspi_batch_done(struct hb_qspi *hbqspi,
								uint32_t mask, uint32_t timeout)
{
	u32 val, trys = 0;

	do {
		val = hb_qspi_rd_reg(hbqspi, HB_QSPI_ST1_REG);
		trys++;
	} while ((!(val & mask)) && (trys < timeout));

	hb_qspi_wr_reg(hbqspi, HB_QSPI_ST1_REG, (val | mask));

	return trys < TRYS_TOTAL_NUM ? 0 : -1;
}

static inline void hb_qspi_cfg_spi_mode(struct hb_qspi *hbqspi,
										uint32_t reg_val)
{
	reg_val &= ~(HB_QSPI_CPOL | HB_QSPI_CPHA);
	switch (hbqspi->spi_mode) {
		case 0:
			reg_val |= HB_QSPI_MODE0;
			break;
		case 1:
			reg_val |= HB_QSPI_MODE1;
			break;
		case 2:
			reg_val |= HB_QSPI_MODE2;
			break;
		case 3:
			reg_val |= HB_QSPI_MODE3;
			break;
		default:
			reg_val |= HB_QSPI_MODE0;
			pr_info_once("Unknown spi mode for qspi: %d, using mode 0!\n",
					hbqspi->spi_mode);
			break;
	}
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL1_REG, reg_val);
#ifdef HB_QSPI_DEBUG
	uint32_t tmp_val;
	tmp_val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	printk_ratelimited("qspi_ctl1_reg:0x%02x\n", tmp_val);
#endif

}

static inline void hb_qspi_cfg_line_mode(struct hb_qspi *hbqspi,
										uint32_t line_mode, bool quad_enable)
{
	uint32_t val;

	if ((hbqspi == NULL) || (line_mode == 0)) {
		pr_info_once("%s:%d Invalide line mode, using 1 line mode!\n"
				, __func__, __LINE__);
		quad_enable = false;
	}
	val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	hb_qspi_cfg_spi_mode(hbqspi, val);
	if (quad_enable) {
		if (line_mode == SPI_RX_DUAL) {
			val = hb_qspi_rd_reg(hbqspi, HB_QSPI_DQM_REG);
			val |= HB_QSPI_DUAL_EN;
			hb_qspi_wr_reg(hbqspi, HB_QSPI_DQM_REG, val);
		}

		if ((line_mode == SPI_RX_QUAD) || (line_mode == SPI_TX_QUAD)) {
			val = hb_qspi_rd_reg(hbqspi, HB_QSPI_DQM_REG);
			val &=
			    ~(HB_QSPI_HOLD_OUTPUT | HB_QSPI_HOLD_OE | HB_QSPI_HOLD_CTL
			      | HB_QSPI_WP_OUTPUT | HB_QSPI_WP_OE | HB_QSPI_WP_CTL
				  | HB_QSPI_DUAL_EN);
			val |= HB_QSPI_QUAD_EN;
			hb_qspi_wr_reg(hbqspi, HB_QSPI_DQM_REG, val);
		}
	} else {
		val = HB_QSPI_DQM_DEFAULT;
		hb_qspi_wr_reg(hbqspi, HB_QSPI_DQM_REG, val);
	}
}

static void hb_qspi_chipselect(struct spi_device *qspi, bool is_high)
{
	struct hb_qspi *hbqspi = spi_master_get_devdata(qspi->master);

	if (is_high) {
		/* Deselect the slave */
		hb_qspi_wr_reg(hbqspi, HB_QSPI_CS_REG, 0);
	} else {
		/* Activate the chip select */
		hb_qspi_wr_reg(hbqspi, HB_QSPI_CS_REG, 1);
	}
}

static inline void hb_qspi_enable_tx(struct hb_qspi *hbqspi)
{
	uint32_t val;

	val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	val |= HB_QSPI_TX_EN;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL1_REG, val);
}

static inline void hb_qspi_disable_tx(struct hb_qspi *hbqspi)
{
	uint32_t val;

	val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	val &= HB_QSPI_TX_DIS;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL1_REG, val);
}

static inline void hb_qspi_enable_rx(struct hb_qspi *hbqspi)
{
	uint32_t val;

	val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	val |= HB_QSPI_RX_EN;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL1_REG, val);
}

static inline void hb_qspi_disable_rx(struct hb_qspi *hbqspi)
{
	uint32_t val;

	val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	val &= HB_QSPI_RX_DIS;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL1_REG, val);
}

static inline void hb_qspi_set_fw(struct hb_qspi *hbqspi, uint32_t fifo_width)
{
	uint32_t val;

	val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	val &= (~HB_QSPI_FW_MASK);
	val |= fifo_width;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL1_REG, val);
}

static inline void hb_qspi_batch_mode_set(struct hb_qspi *hbqspi, bool enable)
{
	uint32_t val;

	val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL3_REG);
	if (enable)
		val &= ~HB_QSPI_BATCH_DIS;
	else
		val |= HB_QSPI_BATCH_DIS;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL3_REG, val);
}

static inline void hb_qspi_reset_fifo(struct hb_qspi *hbqspi)
{
	uint32_t val;

	val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL3_REG);
	val |= HB_QSPI_RST_FIFO;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL3_REG, val);

	val &= (~HB_QSPI_RST_FIFO);
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL3_REG, val);
}

static void hb_qspi_set_speed(struct hb_qspi *hbqspi)
{
	uint32_t div = 0, div_min, scaler, sclk_val;
	hbqspi->ref_clk = clk_get_rate(hbqspi->hclk);

	/* The maxmium of prescale is 16, according to spec. */
	div_min = hbqspi->ref_clk / hbqspi->sclk / 16;
	if (div_min >= (1 << 16)) {
		pr_err("error: Invalid QSpi freq\r\n");
		/* Return a max scaler. */
		sclk_val = SCLK_VAL(0xF, 0xF);
		hb_qspi_wr_reg(hbqspi, HB_QSPI_BDR_REG, sclk_val);
	}

	while (div_min >= 1) {
		div_min >>= 1;
		div++;
	}
	scaler = ((hbqspi->ref_clk / hbqspi->sclk) / (2 << div)) - 1;
	sclk_val = SCLK_VAL(div, scaler);
	hb_qspi_wr_reg(hbqspi, HB_QSPI_BDR_REG, sclk_val);
	hbqspi->sclk = hbqspi->ref_clk / ((scaler + 1) * (2 << div));
	dev_dbg(hbqspi->dev, "hbqspi sclk_con val:0x%x, hclk %d, sclk %d\n",
			 sclk_val, hbqspi->ref_clk, hbqspi->sclk);
}

#if (QSPI_DEBUG > 0)
static void hb_qspi_dump_reg(struct hb_qspi *hbqspi)
{
	uint32_t val = 0, i;

	for (i = HB_QSPI_DAT_REG; i <= HB_QSPI_XIP_REG; i = i + 4) {
		val = hb_qspi_rd_reg(hbqspi, i);
		printk("reg[0x%p] ==> [0x%08x]\n", (hbqspi->regs + i), val);
	}
}

static void trace_hbqspi_transfer(const struct spi_transfer *transfer)
{
#define __TRACE_BUF_SIZE__ 128
#define QSPI_DEBUG_DATA_LEN	16
	int i = 0, nbits = 0, xfer_len, start, end;
	const uint8_t *tmpbuf = NULL;
	char tmp_prbuf[QSPI_DEBUG_DATA_LEN * 2] = { 0 };
	char *prbuf = kzalloc(__TRACE_BUF_SIZE__, GFP_KERNEL);

	memset(prbuf, '\0', __TRACE_BUF_SIZE__);
	if( transfer->tx_buf ){
		nbits = transfer->tx_nbits;
		tmpbuf = (const uint8_t *)transfer->tx_buf;
	}else if(transfer->rx_buf){
		nbits = transfer->rx_nbits;
		tmpbuf = (const uint8_t *)transfer->rx_buf;
	}else{
		printk("No data\n");
		kfree(prbuf);
		return;
	}

	snprintf(prbuf, __TRACE_BUF_SIZE__, "%s-%s[B:%d][L:%d] ",
		transfer->rx_buf ? "<" : "", transfer->tx_buf ? ">" : "",
		nbits, transfer->len);

	if(transfer->len) {
		snprintf(tmp_prbuf, sizeof(tmp_prbuf), " ");
		strcat(prbuf, tmp_prbuf);
		if (transfer->len < 0) {
			xfer_len = -transfer->len;
			start = ((xfer_len < QSPI_DEBUG_DATA_LEN) ?
						0 : (xfer_len - QSPI_DEBUG_DATA_LEN));
			end = xfer_len;
		} else {
			xfer_len = transfer->len;
			start = 0;
			end = (xfer_len < QSPI_DEBUG_DATA_LEN) ? xfer_len : QSPI_DEBUG_DATA_LEN;
		}
		for (i = start; i < end; i++) {
			snprintf(tmp_prbuf, 32, "%02X ", tmpbuf[i]);
			strcat(prbuf, tmp_prbuf);
		}
	}
	printk("%s\n", prbuf);
	kfree(prbuf);
}

extern const char *__clk_get_name(const struct clk *clk);
static void trace_hbqspi(struct hb_qspi *hbqspi)
{
	printk("struct hb_qspi *hbqspi(0x%16p) = {\n", hbqspi);
	printk("\t\t.regs[Phy] = 0x%p\n", hbqspi->regs);
	if(hbqspi->pdev) {
		printk(".pdev = %s\n", dev_name(&hbqspi->pdev->dev));
	}
	printk("\t\t.ref_clk = %u\n", hbqspi->ref_clk);
#ifndef CONFIG_HOBOT_FPGA_X3
	if(hbqspi->hclk) {
		printk("\t\t.hclk = %s@%lu\n", __clk_get_name(hbqspi->hclk),
			clk_get_rate(hbqspi->hclk));
	}
#endif
	printk("\t\t.irq = %d\n", hbqspi->irq);
	printk("\t\t.dev = %s\n", dev_name(hbqspi->dev));
	printk("\t\t.sclk = %d\n", hbqspi->sclk);
	//printk("\t\t.mode = 0x%08X\n", hbqspi->mode);
	printk("}\n");
}

#endif

static void hb_qspi_hw_init(struct hb_qspi *hbqspi)
{
	uint32_t reg_val = 0;

	/* set qspi clk div */
	hb_qspi_set_speed(hbqspi);

	/* clear status and reset fifo */
	reg_val = HB_QSPI_MODF_CLR | HB_QSPI_RBD | HB_QSPI_TBD;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_ST1_REG, reg_val);
	reg_val = HB_QSPI_TXRD_EMPTY | HB_QSPI_RXWR_FULL;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_ST2_REG, reg_val);
	hb_qspi_reset_fifo(hbqspi);

	/* set qspi work mode */
	reg_val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	reg_val &= (~HB_QSPI_FW_MASK);
	reg_val |= (HB_QSPI_MST | HB_QSPI_FW8 | MSB);
	hb_qspi_cfg_spi_mode(hbqspi, reg_val);

	/* Set Rx/Tx fifo trig level  */
	hb_qspi_wr_reg(hbqspi, HB_QSPI_TTL_REG, HB_QSPI_TRIG_LEVEL);
	hb_qspi_wr_reg(hbqspi, HB_QSPI_RTL_REG, HB_QSPI_TRIG_LEVEL);

	reg_val = 0;
#ifndef HB_QSPI_WORK_POLL
	/* init interrupt */
	reg_val = HB_QSPI_ERR_INT;
	reg_val |= (HB_QSPI_RBC_INT | HB_QSPI_TBC_INT);
#endif
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CTL2_REG, reg_val);

	/* unselect chip */
	reg_val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CS_REG);
	reg_val &= 0x0;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_CS_REG, reg_val);

	/* Always set SPI to one line as init. */
	reg_val = HB_QSPI_DQM_DEFAULT;
	hb_qspi_wr_reg(hbqspi, HB_QSPI_DQM_REG, reg_val);

	/* Disable hardware xip mode */
	reg_val = hb_qspi_rd_reg(hbqspi, HB_QSPI_XIP_REG);
	reg_val &= ~(1 << 1);
	hb_qspi_wr_reg(hbqspi, HB_QSPI_XIP_REG, reg_val);
}

static inline int hb_qspi_setup(struct spi_device *qspi)
{
	int ret = 0;
	struct hb_qspi *hbqspi = spi_master_get_devdata(qspi->master);

	if (qspi->master->busy)
		return -EBUSY;

	switch (hbqspi->buswidth) {
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
		pr_err("%s No mode matches 0x%08X.\n", __func__, qspi->mode);
		break;
	}

	return ret;
}

static inline int hb_qspi_rd_byte(struct hb_qspi *hbqspi,
								void *pbuf, uint32_t len)
{
	int64_t i, ret = len;
	uint8_t *dbuf = (uint8_t *) pbuf;

	hb_qspi_reset_fifo(hbqspi);
	hb_qspi_batch_mode_set(hbqspi, 0);
	/* enable rx */
	hb_qspi_enable_rx(hbqspi);

	for (i = 0; i < len; i++) {
		if (hb_qspi_check_status(hbqspi, HB_QSPI_ST2_REG,
							  HB_QSPI_TX_EP, HB_QSPI_TIMEOUT_US)) {
			pr_err("%s:%d generate read sclk failed\n", __func__, __LINE__);
			ret = -1;
			goto rd_err;
		}
		hb_qspi_wr_reg(hbqspi, HB_QSPI_DAT_REG, 0x00);
		if (hb_qspi_check_status(hbqspi, HB_QSPI_ST2_REG,
							  HB_QSPI_RX_EP, HB_QSPI_TIMEOUT_US)) {
			pr_err("%s:%d No data fill into RX DAT\n", __func__, __LINE__);
			ret = -ETIMEDOUT;
			goto rd_err;
		}
		dbuf[i] = hb_qspi_rd_reg(hbqspi, HB_QSPI_DAT_REG) & 0xFF;
	}

rd_err:
	hb_qspi_disable_rx(hbqspi);
	if (ret < 0) {
#if (QSPI_DEBUG > 0)
		hb_qspi_dump_reg(hbqspi);
#endif
	}

	return ret;
}

static inline int hb_qspi_rd_batch(struct hb_qspi *hbqspi,
									void *pbuf, uint32_t len)
{
	uint32_t rx_len, offset = 0;
	int64_t i, len_remain = (int64_t)len, ret = 0;
	uint32_t *dbuf = (uint32_t *) pbuf;

	hb_qspi_reset_fifo(hbqspi);
	/* set fifo width and batch mode */
	hb_qspi_set_fw(hbqspi, HB_QSPI_FW32);
	hb_qspi_batch_mode_set(hbqspi, 1);

	while (len_remain > 0) {
#ifndef HB_QSPI_WORK_POLL
		reinit_completion(&hbqspi->xfer_complete);
#endif
		rx_len = MIN(len_remain, BATCH_MAX_CNT);
		hb_qspi_wr_reg(hbqspi, HB_QSPI_RBC_REG, rx_len);
		/* enable rx */
		hb_qspi_enable_rx(hbqspi);

		for (i = 0; i < rx_len; i += 8) {
			if (hb_qspi_check_status(hbqspi, HB_QSPI_ST1_REG,
								HB_QSPI_RX_AF, HB_QSPI_TIMEOUT_US)) {
				pr_err("%s:%d rx batch fill timeout! len=%u, recv=%lld\n",
					__func__, __LINE__, len, i);
				goto rb_err;
			}
			dbuf[offset++] = hb_qspi_rd_reg(hbqspi, HB_QSPI_DAT_REG);
			dbuf[offset++] = hb_qspi_rd_reg(hbqspi, HB_QSPI_DAT_REG);
		}
#ifdef HB_QSPI_WORK_POLL
		if (hb_qspi_batch_done(hbqspi,
							  HB_QSPI_RBD, HB_QSPI_TIMEOUT_US)) {
			pr_info("%s:%d poll rx batch comp timeout! len=%u, received=%lld\n",
					__func__, __LINE__, len, i);
		}
#else
		if (!wait_for_completion_timeout(&hbqspi->xfer_complete,
									msecs_to_jiffies(HB_QSPI_TIMEOUT_MS))) {
			pr_info("%s:%d rx batch comp timeout! len=%u, received=%lld\n",
					__func__, __LINE__, len, i);
		}
#endif
		hb_qspi_disable_rx(hbqspi);
		len_remain -= rx_len;
		ret += rx_len;
	}

rb_err:
	hb_qspi_disable_rx(hbqspi);
	if (ret != len) {
		ret = -ETIMEDOUT;
#if (QSPI_DEBUG > 0)
		hb_qspi_dump_reg(hbqspi);
#endif
	}

	/* Disable batch mode and rx link */
	hb_qspi_set_fw(hbqspi, HB_QSPI_FW8);
	hb_qspi_batch_mode_set(hbqspi, 0);

	return ret;
}

static inline int hb_qspi_wr_byte(struct hb_qspi *hbqspi,
							const void *pbuf, uint32_t len)
{
	int64_t i, ret = len;
	uint8_t *dbuf = (uint8_t *) pbuf;
#if (QSPI_DEBUG > 0)
#define MAXPRINT 32
	char data_tmp[4];
	char tmpbuf[MAXPRINT * 2];
	memset(tmpbuf, 0, sizeof(tmpbuf));
	for (i = 0; i < ((len < MAXPRINT) ? len : MAXPRINT); i ++) {
		snprintf(data_tmp, sizeof(data_tmp), "%02x ", ((uint8_t *)pbuf)[i]);
		strncat(tmpbuf, data_tmp, sizeof(tmpbuf) - strlen(tmpbuf));
	}
	printk("Wr_byte:  %s\n", tmpbuf);
#endif
	hb_qspi_reset_fifo(hbqspi);
	hb_qspi_batch_mode_set(hbqspi, 0);
	/* enable tx */
	hb_qspi_enable_tx(hbqspi);

	for (i = 0; i < len; i++) {
		if (hb_qspi_check_status(hbqspi, HB_QSPI_ST2_REG,
							  HB_QSPI_TX_FULL, HB_QSPI_TIMEOUT_US)) {
			pr_err("%s:%d TX DAT full before write\n", __func__, __LINE__);
			ret = -ENOBUFS;
			goto wr_err;
		}
		hb_qspi_wr_reg(hbqspi, HB_QSPI_DAT_REG, dbuf[i]);
	}
	/* Check tx complete */
	if (hb_qspi_check_status(hbqspi, HB_QSPI_ST2_REG,
						  HB_QSPI_TX_EP, HB_QSPI_TIMEOUT_US)) {
		pr_err("%s:%d TX Data transfer overtime!\n", __func__, __LINE__);
		ret = -ETIMEDOUT;
		goto wr_err;
	}

wr_err:
	hb_qspi_disable_tx(hbqspi);
	if (ret < 0) {
#if (QSPI_DEBUG > 0)
		hb_qspi_dump_reg(hbqspi);
#endif
	}

	return ret;
}

static inline int hb_qspi_wr_batch(struct hb_qspi *hbqspi,
							const void *pbuf, uint32_t len)
{
	uint32_t tx_len, offset = 0;
	int64_t i, len_remain = (int64_t)len, ret = 0;
	uint32_t *dbuf = (uint32_t *) pbuf;
#if (QSPI_DEBUG > 0)
#define MAXPRINT 32
	char data_tmp[4];
	char tmpbuf[MAXPRINT * 2];
	memset(tmpbuf, 0, sizeof(tmpbuf));
	for (i = 0; i < ((len < MAXPRINT) ? len : MAXPRINT); i ++) {
		snprintf(data_tmp, sizeof(data_tmp), "%02x ", ((uint8_t *)pbuf)[i]);
		strncat(tmpbuf, data_tmp, sizeof(tmpbuf) - strlen(tmpbuf));
	}
	printk("Wr_batch: %s\n", tmpbuf);
#endif
	hb_qspi_reset_fifo(hbqspi);
	/* set fifo width and batch mode */
	hb_qspi_set_fw(hbqspi, HB_QSPI_FW32);
	hb_qspi_batch_mode_set(hbqspi, 1);

	while (len_remain > 0) {
#ifndef HB_QSPI_WORK_POLL
		reinit_completion(&hbqspi->xfer_complete);
#endif
		tx_len = MIN(len_remain, BATCH_MAX_CNT);
		hb_qspi_wr_reg(hbqspi, HB_QSPI_TBC_REG, tx_len);

		/* enable tx */
		hb_qspi_enable_tx(hbqspi);

		for (i = 0; i < tx_len; i += 8) {
			if (hb_qspi_check_status(hbqspi, HB_QSPI_ST1_REG,
								  HB_QSPI_TX_AE, HB_QSPI_TIMEOUT_US)) {
				pr_err("%s:%d tx batch fill timeout! len=%u, received=%lld\n",
					__func__, __LINE__, len, i);
				goto tb_err;
			}
			hb_qspi_wr_reg(hbqspi, HB_QSPI_DAT_REG, dbuf[offset++]);
			hb_qspi_wr_reg(hbqspi, HB_QSPI_DAT_REG, dbuf[offset++]);
		}
#ifdef HB_QSPI_WORK_POLL
		if (hb_qspi_batch_done(hbqspi,
							  HB_QSPI_TBD, HB_QSPI_TIMEOUT_US)) {
			pr_err("%s:%d poll tx batch comp timeout! len=%u, received=%lld\n",
					__func__, __LINE__, len, i);
		}
#else
		if (!wait_for_completion_timeout(&hbqspi->xfer_complete,
									msecs_to_jiffies(HB_QSPI_TIMEOUT_MS))) {
			pr_info("%s:%d tx batch comp timeout! len=%u, tansferred=%lld\n",
					__func__, __LINE__, len, i);
		}
#endif
		len_remain -= tx_len;
		ret += tx_len;
	}

tb_err:
	hb_qspi_disable_tx(hbqspi);
	if (ret != len) {
		ret = -ETIMEDOUT;
#if (QSPI_DEBUG > 0)
		hb_qspi_dump_reg(hbqspi);
#endif
	}

	/* Disable batch mode and tx link */
	hb_qspi_set_fw(hbqspi, HB_QSPI_FW8);
	hb_qspi_batch_mode_set(hbqspi, 0);

	return ret;
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

static inline int hb_qspi_start_transfer(struct spi_master *master,
				  struct spi_device *qspi,
				  struct spi_transfer *transfer)
{
	uint16_t default_mode = 0;
	int transfered = 0;
	struct hb_qspi *hbqspi = spi_master_get_devdata(master);
	unsigned remainder, residue;
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	mutex_lock(&(hbqspi->xfer_lock));
#endif
	remainder = transfer->len % (HB_QSPI_TRIG_LEVEL);
	residue   = transfer->len - remainder;
	/*
	 * this 'mode' init by spi core,
	 * depending on tx-bus-width property defined in dts
	 */
	default_mode = qspi->mode;
	hbqspi->txbuf = transfer->tx_buf;
	hbqspi->rxbuf = transfer->rx_buf;

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
	if (hbqspi->txbuf != NULL) {
		if (transfer->len) {
			if (qspi->mode & SPI_TX_QUAD)
				hb_qspi_cfg_line_mode(hbqspi, SPI_TX_QUAD, true);
			else if (qspi->mode & SPI_TX_DUAL)
				hb_qspi_cfg_line_mode(hbqspi, SPI_TX_DUAL, true);
		}

		if (residue > 0) {
			transfered += hb_qspi_wr_batch(hbqspi, hbqspi->txbuf, residue);
		}
		if (remainder > 0) {
			transfered += hb_qspi_wr_byte(hbqspi,
							(void *)hbqspi->txbuf + residue, remainder);
		}

		if (transfer->len) {
			if (qspi->mode & SPI_TX_QUAD)
				hb_qspi_cfg_line_mode(hbqspi, SPI_TX_QUAD, false);
			else if (qspi->mode & SPI_TX_DUAL)
				hb_qspi_cfg_line_mode(hbqspi, SPI_TX_DUAL, false);
		}
	}

	if (hbqspi->rxbuf != NULL) {
		if (qspi->mode & SPI_RX_QUAD)
			hb_qspi_cfg_line_mode(hbqspi, SPI_RX_QUAD, true);
		if (qspi->mode & SPI_RX_DUAL)
			hb_qspi_cfg_line_mode(hbqspi, SPI_RX_DUAL, true);

		if (residue > 0) {
			transfered += hb_qspi_rd_batch(hbqspi, hbqspi->rxbuf, residue);
		}
		if (remainder > 0) {
			transfered += hb_qspi_rd_byte(hbqspi,
							(void *)hbqspi->rxbuf + residue, remainder);
		}

		if (qspi->mode & SPI_RX_QUAD)
			hb_qspi_cfg_line_mode(hbqspi, SPI_RX_QUAD, false);
		if (qspi->mode & SPI_RX_DUAL)
			hb_qspi_cfg_line_mode(hbqspi, SPI_RX_DUAL, false);

	}

	qspi->mode = default_mode;
	transfer->len = transfered;
#if (QSPI_DEBUG > 1)
	/* trace transfer data when transfer done */
	trace_hbqspi_transfer(transfer);
#endif

	spi_finalize_current_transfer(master);
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	mutex_unlock(&(hbqspi->xfer_lock));
#endif
	return transfered;
}

/**
 * hb_qspi_exec_mem_op() - Initiates the QSPI transfer
 * @mem: the SPI memory
 * @op: the memory operation to execute
 *
 * Executes a memory operation.
 *
 * This function first selects the chip and starts the memory operation.
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */
static int hb_qspi_exec_mem_op(struct spi_mem *mem,
				 const struct spi_mem_op *op)
{
	struct hb_qspi *hbqspi = spi_controller_get_devdata(mem->spi->master);
	int ret = 0, remainder = 0, residue = 0, non_data_size = 0, i;
	uint8_t *non_data_buf = NULL, *tmp_ptr;
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	mutex_lock(&(hbqspi->xfer_lock));
#endif
	dev_dbg(hbqspi->dev,
			"cmd:0x%02x addr_dum_dat_nbytes:%d %d %d bus_widths:%d%d%d%d\n",
			op->cmd.opcode,
			op->addr.nbytes, op->dummy.nbytes, op->data.nbytes,
			op->cmd.buswidth, op->addr.buswidth,
			op->dummy.buswidth, op->data.buswidth);
	/* First deal with non-data transmits: cmd/addr/dummy */
	non_data_size = sizeof(op->cmd.opcode) + op->addr.nbytes
					+ op->dummy.nbytes;
	non_data_buf = kzalloc(non_data_size, GFP_KERNEL);
	memset(non_data_buf, 0x0, non_data_size);
	tmp_ptr = non_data_buf;

	if (op->cmd.opcode) {
		memcpy(tmp_ptr, (u8 *)&op->cmd.opcode, sizeof(op->cmd.opcode));
		tmp_ptr += sizeof(op->cmd.opcode);
	}

	if (op->addr.nbytes) {
		memset(tmp_ptr, 0, sizeof(op->addr.nbytes));
		for (i = 0; i < op->addr.nbytes; i++)
			tmp_ptr[i] = op->addr.val >>
					(8 * (op->addr.nbytes - i - 1));

		tmp_ptr += op->addr.nbytes;
	}

	if (op->dummy.nbytes) {
		memset(tmp_ptr, 0xff, op->dummy.nbytes);
	}

	hb_qspi_cfg_line_mode(hbqspi, SPI_TX_QUAD, false);
	hb_qspi_chipselect(mem->spi, HB_QSPI_CS_EN);

	hbqspi->txbuf = non_data_buf;
	hbqspi->rxbuf = NULL;
	ret += hb_qspi_wr_byte(hbqspi, hbqspi->txbuf, sizeof(op->cmd.opcode));

	hbqspi->txbuf = non_data_buf + sizeof(op->cmd.opcode);
	hbqspi->rxbuf = NULL;
	hb_qspi_cfg_line_mode(hbqspi,
		op->addr.buswidth == 4 ? SPI_TX_QUAD : SPI_TX_DUAL,
		op->addr.buswidth == 1 ? false : true);
	ret += hb_qspi_wr_byte(hbqspi, hbqspi->txbuf,
					non_data_size - sizeof(op->cmd.opcode));

	if(ret != non_data_size) {
		ret = -1;
		goto exec_end;
	}

	ret = 0;
	if (op->data.nbytes) {
		if (op->data.buswidth == 4)
			hb_qspi_cfg_line_mode(hbqspi, SPI_TX_QUAD, true);
		else if (op->data.buswidth == 2)
			hb_qspi_cfg_line_mode(hbqspi, SPI_TX_DUAL, true);
		else
			hb_qspi_cfg_line_mode(hbqspi, SPI_TX_QUAD, false);

		remainder = op->data.nbytes % (HB_QSPI_TRIG_LEVEL);
		residue   = op->data.nbytes - remainder;
		if (op->data.dir == SPI_MEM_DATA_OUT) {
			hbqspi->txbuf = (u8 *)op->data.buf.out;
			hbqspi->rxbuf = NULL;
			if (residue)
				ret += hb_qspi_wr_batch(hbqspi, hbqspi->txbuf, residue);
			if (remainder)
				ret += hb_qspi_wr_byte(hbqspi,
								(void *)hbqspi->txbuf + residue, remainder);
		} else {
			hbqspi->txbuf = NULL;
			hbqspi->rxbuf = (u8 *)op->data.buf.in;
			if (residue)
				ret += hb_qspi_rd_batch(hbqspi,
								(void *)hbqspi->rxbuf, residue);
			if (remainder)
				ret += hb_qspi_rd_byte(hbqspi,
								(void *)hbqspi->rxbuf + residue, remainder);
		}
	}

	dev_dbg(hbqspi->dev, "Data bytes transfered: %d\n", ret);
	if (ret == op->data.nbytes) {
		ret = 0;
	} else {
		ret = -ETIMEDOUT;
	}

exec_end:
	if (non_data_buf)
		kfree(non_data_buf);
	hb_qspi_chipselect(mem->spi, HB_QSPI_CS_DIS);
	hb_qspi_cfg_line_mode(hbqspi, SPI_TX_QUAD, 0);
	if (ret)
		dev_err(hbqspi->dev, "Exec op failed! cmd:%#02x err: %d\n",
				op->cmd.opcode, ret);

#ifdef CONFIG_HOBOT_DIAG
	hb_qspiflash_diag_report(ret, op->cmd.opcode);
#endif
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	mutex_unlock(&(hbqspi->xfer_lock));
#endif
	return ret;
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
	/* For HB not used */
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
	/* For HB not used */
	return 0;
}

static bool hb_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct hb_qspi *hbqspi = spi_controller_get_devdata(mem->spi->master);
	if (op->cmd.buswidth > 1
		|| op->addr.buswidth > hbqspi->buswidth
		|| op->dummy.buswidth > hbqspi->buswidth
		|| op->data.buswidth > hbqspi->buswidth)
		return false;

	return true;
}

static const struct spi_controller_mem_ops hb_mem_ops = {
	.supports_op = hb_supports_op,
	.exec_op = hb_qspi_exec_mem_op,
};

#ifdef CONFIG_HOBOT_DIAG
static void hb_qspiflash_diag_report(uint8_t errsta, uint8_t sta_reg)
{
	static uint8_t last_status = DiagEventStaUnknown;
	uint64_t envdata = 0;
	uint8_t ch_info = 0xff;
	uint8_t err_type = 0xff;
	uint8_t rsv_byte = 0xff;
	uint8_t data_len = 1;
	int total_len = sizeof(sta_reg) + sizeof(data_len) + sizeof(ch_info) + sizeof(err_type) + sizeof(rsv_byte);
	envdata |= sta_reg;
	envdata = (envdata << 8) | data_len;
	envdata = (envdata << 8) | rsv_byte;
	envdata = (envdata << 8) | err_type;
	envdata = (envdata << 8) | ch_info;

	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_qspi,
				EventIdqspiErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&envdata,
				total_len);
	} else if (last_status != DiagEventStaSuccess) {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_qspi,
				EventIdqspiErr,
				DiagEventStaSuccess);
	}
	last_status = !errsta ? DiagEventStaSuccess : DiagEventStaFail;
}
#endif

#ifdef CONFIG_HOBOT_DIAG
static void hb_qspiflash_callback(void *p, size_t len)
{
	return;
}
#endif

#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
static int hbqspi_dpm_callback(struct hobot_dpm *self,
				 unsigned long event, int state)
{
	struct hb_qspi *hbqspi = container_of(self, struct hb_qspi, hbqspi_dpm);

	if (event == HB_BUS_SIGNAL_START) {
		if (!mutex_trylock(&(hbqspi->xfer_lock)))
			return -EBUSY;
		dev_dbg(hbqspi->dev, "%s: Grab hb_qspi lock success\n", __func__);
	} else if (event == HB_BUS_SIGNAL_END) {
		/* hb_qspi_set_speed(hbqspi); */
		mutex_unlock(&(hbqspi->xfer_lock));
		dev_dbg(hbqspi->dev, "%s: Release hb_qspi lock success\n", __func__);
	}

	return 0;
}
#endif

static irqreturn_t hb_qspi_irq_handler(int irq, void *dev_id)
{
#ifdef CONFIG_HOBOT_DIAG
	uint32_t err_status;
	int err = 0;
#endif
	struct hb_qspi *hbqspi = dev_id;
#ifndef HB_QSPI_WORK_POLL
	unsigned int irq_status;
	/* Read interrupt status */
	irq_status = hb_qspi_rd_reg(hbqspi, HB_QSPI_ST1_REG);
	hb_qspi_wr_reg(hbqspi, HB_QSPI_ST1_REG, HB_QSPI_TBD | HB_QSPI_RBD);
	if (irq_status & (HB_QSPI_TBD | HB_QSPI_RBD)) {
		complete(&hbqspi->xfer_complete);
		return IRQ_HANDLED;
	}

#ifdef CONFIG_HOBOT_DIAG
	err_status = hb_qspi_rd_reg(hbqspi, HB_QSPI_ST2_REG);
#if IS_ENABLED(CONFIG_HOBOT_DIAG_INJECT)
	diag_inject_val(ModuleDiag_qspi, EventIdqspiErr, &err_status);
#endif
	if (err_status & (HB_QSPI_RXWR_FULL | HB_QSPI_TXRD_EMPTY))
		err = 1;

	hb_qspiflash_diag_report(err, err_status);
#endif /* CONFIG_HOBOT_DIAG */
#endif /* HB_QSPI_WORK_POLL */
	hb_qspi_wr_reg(hbqspi, HB_QSPI_ST2_REG,
			HB_QSPI_RXWR_FULL | HB_QSPI_TXRD_EMPTY);
	return IRQ_HANDLED;
}

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
	struct hb_qspi *hbqspi;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct device_node *nc;
	uint32_t num_cs, buswidth, max_speed_hz;
	const char* device_name;


	master = spi_alloc_master(&pdev->dev, sizeof(*hbqspi));
	if (!master)
		return -ENOMEM;

	hbqspi = spi_master_get_devdata(master);
	master->dev.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, master);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hbqspi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hbqspi->regs)) {
		ret = PTR_ERR(hbqspi->regs);
		goto remove_master;
	}

	hbqspi->dev = dev;

#ifndef CONFIG_HOBOT_FPGA_X3
	hbqspi->hclk = devm_clk_get(&pdev->dev, "qspi_aclk");
	if (IS_ERR(hbqspi->hclk)) {
		dev_err(dev, "hclk clock not found.\n");
		ret = PTR_ERR(hbqspi->hclk);
		goto remove_master;
	}

	ret = clk_prepare_enable(hbqspi->hclk);
	if (ret) {
		dev_err(dev, "Unable to enable APB clock.\n");
		goto clk_dis_hclk;
	}

	hbqspi->ref_clk = clk_get_rate(hbqspi->hclk);
#else
	hbqspi->ref_clk = HOBOT_QSPI_HCLK_DEF;
	hbqspi->sclk = HOBOT_QSPI_SCLK_DEF;
#endif

	/* used for spi-mem select op and meet __spi_validate */
	if (of_property_read_u32(pdev->dev.of_node, "spi-mode", &hbqspi->spi_mode))
		hbqspi->spi_mode = 0;

	hbqspi->sclk = HB_QSPI_DEF_BDR;
	for_each_available_child_of_node(pdev->dev.of_node, nc) {
		ret = of_property_read_u32(nc, "spi-max-frequency",
				&max_speed_hz);
		if (!ret) {
			if (max_speed_hz < hbqspi->ref_clk / 2)
				hbqspi->sclk = max_speed_hz;
		} else {
			dev_err(dev, "spi-max-frequency not found\n");
		}
	}

	buswidth = 1;
	if (of_property_read_u32(pdev->dev.of_node, "bus-width", &buswidth))
		dev_warn_once(dev, "tx/rx bus width not found, using 1 line mode\n");
	hbqspi->buswidth = buswidth;

	ret = of_property_read_u32(pdev->dev.of_node, "num-cs", &num_cs);
	if (ret < 0)
		master->num_chipselect = HB_QSPI_MAX_CS;
	else
		master->num_chipselect = num_cs;
#ifndef HB_QSPI_WORK_POLL
	init_completion(&hbqspi->xfer_complete);
#endif
	/* Obtain IRQ line */
	hbqspi->irq = platform_get_irq(pdev, 0);
	if (hbqspi->irq < 0) {
		dev_err(dev, "Cannot obtain IRQ.\n");
		goto clk_dis_hclk;
	}
	ret = devm_request_irq(dev, hbqspi->irq, hb_qspi_irq_handler,
							0, pdev->name, hbqspi);
	if (ret) {
		dev_err(dev, "Cannot request IRQ.\n");
		goto clk_dis_hclk;
	}

	if (of_property_read_string(pdev->dev.of_node,
								"device-name", &device_name)) {
		master->dev.init_name = "hb_qspi";
	} else {
		master->dev.init_name =  device_name;
	}

#ifdef CONFIG_HOBOT_DIAG
	if (diag_register(ModuleDiag_qspi, EventIdqspiErr,
			4, 10, DIAG_MSG_INTERVAL_MAX, hb_qspiflash_callback) < 0)
		pr_err("qspi flash diag register fail\n");
#endif

	/* QSPI controller initializations */
	hb_qspi_hw_init(hbqspi);

	pr_debug("hbqspi_refclk:%dHz, hbqspi_sclk:%dHz\n",
			  hbqspi->ref_clk, hbqspi->sclk);
	master->setup = hb_qspi_setup;
	master->set_cs = hb_qspi_chipselect;
	master->transfer_one = hb_qspi_start_transfer;
	master->prepare_transfer_hardware = hb_prepare_transfer_hardware;
	master->unprepare_transfer_hardware = hb_unprepare_transfer_hardware;
	master->mem_ops = &hb_mem_ops;
	master->max_speed_hz = hbqspi->sclk;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->mode_bits = 0;

	switch (hbqspi->spi_mode) {
		case 0:
			master->mode_bits |= SPI_MODE_0;
			break;
		case 1:
			master->mode_bits |= SPI_MODE_1;
			break;
		case 2:
			master->mode_bits |= SPI_MODE_2;
			break;
		case 3:
			master->mode_bits |= SPI_MODE_3;
			break;
		default:
			master->mode_bits |= SPI_MODE_0;
			pr_err("Unknown spi mode for qspi: %d, using mode 0!\n",
					hbqspi->spi_mode);
			break;
	}

	switch (buswidth) {
		case 1:
			break;
		case 2:
			master->mode_bits |= (SPI_TX_DUAL | SPI_RX_DUAL);
			break;
		case 4:
			master->mode_bits |= (SPI_TX_QUAD | SPI_RX_QUAD);
			break;
		case 8:
			master->mode_bits |= (SPI_TX_OCTAL | SPI_RX_OCTAL);
			break;
		default:
			hbqspi->buswidth = 1;
			master->mode_bits &= 0x0FF;
			pr_err("Unknown data width for qspi: %d, using 1bit width!\n",
					buswidth);
			break;
	}
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	mutex_init(&(hbqspi->xfer_lock));
	hbqspi->hbqspi_dpm.dpm_call = &hbqspi_dpm_callback;
	hobot_dpm_register(&(hbqspi->hbqspi_dpm), hbqspi->dev);
#endif
#if (QSPI_DEBUG > 1)
	trace_hbqspi(hbqspi);
#endif

	if (master->dev.parent == NULL)
		master->dev.parent = &master->dev;

	ret = spi_register_master(master);
	if (ret) {
		dev_err(dev, "SPI master registration failed\n");
		goto clk_dis_hclk;
	}

	return 0;

clk_dis_hclk:
	clk_disable_unprepare(hbqspi->hclk);

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
	struct hb_qspi *hbqspi = platform_get_drvdata(pdev);
	dev_dbg(hbqspi->dev, "removed\n");
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	hobot_dpm_unregister(&(hbqspi->hbqspi_dpm));
#endif
	return 0;
}

#ifdef CONFIG_PM_SLEEP
int hb_qspi_suspend(struct device *dev)
{
	int val;
	struct spi_master *master = dev_get_drvdata(dev);
	struct hb_qspi *hbqspi = spi_master_get_devdata(master);

	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

	val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	while (val & (HB_QSPI_RX_EN | HB_QSPI_TX_EN)) {
		val = hb_qspi_rd_reg(hbqspi, HB_QSPI_CTL1_REG);
	}

	hb_qspi_disable_tx(hbqspi);
	hb_qspi_disable_rx(hbqspi);
	clk_disable_unprepare(hbqspi->hclk);
	return 0;
}

int hb_qspi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct hb_qspi *hbqspi = spi_master_get_devdata(master);

	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);

	if (clk_prepare_enable(hbqspi->hclk)) {
		pr_err("%s:%s, clk_enable failed!\n", __FILE__, __func__);
	}
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
