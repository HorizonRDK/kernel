/*
 * Horizon X2 QuadSPI driver.
 *
 * Copyright (C) 2018 Horizon, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This driver is based on drivers/mtd/spi-nor/fsl-quadspi.c from Freescale.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/completion.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/sizes.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>
#include <linux/slab.h>

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
#define LSB		 					(0x40)
#define MSB		 					(0x00)
#define CPHA_H							(0x20)
#define CPHA_L							(0x00)
#define CPOL_H							(0x10)
#define CPOL_L							(0x00)
#define MST_MODE						(0x04)
#define SLV_MODE						(0x00)

#define TX_ENABLE						(0x08)
#define TX_DISABLE						(0x00)
#define RX_ENABLE						(0x80)
#define RX_DISABLE						(0x00)

#define FIFO_WIDTH32						(0x02)
#define FIFO_WIDTH16						(0x01)
#define FIFO_WIDTH8						(0x00)
#define FIFO_DEPTH						(16)
#define BATCH_MAX_SIZE						(0x10000)

#define SPI_FIFO_DEPTH						(FIFO_DEPTH)
#define SPI_BACKUP_OFFSET					(0x10000) //64K
#define SPI_CS0							(0x01)

#define SPI_MODE0 						(CPOL_L | CPHA_L)
#define SPI_MODE1 						(CPOL_L | CPHA_H)
#define SPI_MODE2 						(CPOL_H | CPHA_L)
#define SPI_MODE3 						(CPOL_H | CPHA_H)

/* Definition of SPI_CTRL2 */
#define BATCH_TINT_EN						(0x1L << 7)
#define BATCH_RINT_EN						(0x1L << 6)
#define MODDEF							(0x1L << 3)
#define ERR_INT_EN						(0x1l << 2)
#define TX_INT_EN						(0x1L << 1)
#define RX_INT_EN						(0x1L << 0)

/* Definition of SPI_CTRL3 */
#define BATCH_DISABLE						(0x1L << 4)
#define BATCH_ENABLE						(0x0L << 4)
#define FIFO_RESET						(0x03)
#define SW_RST_TX						(0x02)
#define SW_RST_RX						(0x01)
#define RX_DMA_EN						(0x04)
#define TX_DMA_EN						(0x08)

/* STATUS1 */
#define BATCH_TXDONE						(0x1L<<5)
#define BATCH_RXDONE						(0x1L<<4)
#define MODF_CLR						(0x1L<<3)
#define TX_ALMOST_EMPTY						(0x1L<<1)
#define RX_ALMOST_FULL						(0x1L<<0)

/* STATUS2 */
#define SSN_IN							(0x1L<<7)
#define TXFIFO_FULL						(0x1L<<5)
#define TXFIFO_EMPTY						(0x1L<<1)
#define RXFIFO_EMPTY						(0x1L<<4)
#define RXFIFO_FULL						(0x1L<<0)

/* SPI_DUAL_QUAD_MODE */
#define HOLD_OUTPUT						(0x1 << 7)
#define HOLD_OE							(0x1 << 6)
#define HOLD_CTL						(0x1 << 5)
#define WP_OUTPUT						(0x1 << 4)
#define WP_OE							(0x1 << 3)
#define WP_CTL							(0x1 << 2)
#define QPI_ENABLE						(0x1 << 1)
#define DPI_ENABLE						(0x1 << 0)
#define SPI_XFER_TX						0
#define SPI_XFER_RX						1
#define SPI_XFER_TXRX						2

/* SPI_XIP_CFG */
#define XIP_EN 							(0x1L<<0)
#define FLASH_SW_CFG						(0x0L<<1)
#define FLASH_HW_CFG						(0x1L<<1)
#define FLASH_CTNU_MODE						(0x1<<4)
#define DUMMY_CYCLE_0						(0x0<<5)
#define DUMMY_CYCLE_1						(0x1<<5)
#define DUMMY_CYCLE_2						(0x2<<5)
#define DUMMY_CYCLE_3						(0x3<<5)
#define DUMMY_CYCLE_4						(0x4<<5)
#define DUMMY_CYCLE_5						(0x5<<5)
#define DUMMY_CYCLE_6						(0x6<<5)
#define DUMMY_CYCLE_7						(0x7<<5)

#define XIP_FLASH_ADDR_OFFSET					0x100
/* Batch mode or no batch mode */
#define SPI_BATCH_MODE						1
#define SPI_NO_BATCH_MODE					0

#define QSPI_RX_BUS_WIDTH_SINGLE	1
#define QSPI_DEFAULT_NUM_CS		1

#define SCLK_DIV(div, scaler)   	((2 << (div)) * (scaler + 1))
#define SCLK_VAL(div, scaler)		(((div) << 4) | ((scaler) << 0))
#define MIN(a, b)			(((a) < (b)) ? (a) : (b))

/* Instruction type */
#define QSPI_INST_TYPE_SINGLE			0
#define QSPI_INST_TYPE_DUAL			1
#define QSPI_INST_TYPE_QUAD			2


#define	MAX_CMD_SIZE		6

struct x2_qspi {
        struct spi_nor nor;
        struct mtd_partition *parts;
	void __iomem *iobase;
//TODO
#if 0
	struct clk *clk, *clk_en;
#endif
        struct device *dev;
	struct completion transfer_complete;

	u32 clk_rate;
        u32 data_width;
	bool batch_mode;

        struct mutex lock;

        const void *txbuf;
	void *rxbuf;
        u32 tx_bus_width;
        u32 rx_bus_width;

        int ref_clk;
	int qspi_clk;
	u8 command[MAX_CMD_SIZE];
};

/**
 * x2_qspi_readl:	For QSPI controller read operation
 * @xqspi:	Pointer to the x2_qspi structure
 * @offset:	Offset from where to read
 */
static u32 x2_qspi_readl(struct x2_qspi *xqspi, u32 offset)
{
	return readl_relaxed(xqspi->iobase + offset);
}

/**
 * x2_qspi_writel:	For QSPI controller write operation
 * @xqspi:	Pointer to the x2_qspi structure
 * @offset:	Offset where to write
 * @val:	Value to be written
 */
static void x2_qspi_writel(struct x2_qspi *xqspi, u32 offset,
				      u32 val)
{
	writel_relaxed(val, (xqspi->iobase + offset));
}

int x2_qspi_poll_rx_empty(struct x2_qspi *xqspi,
	u32 offset, uint32_t mask, uint32_t timeout)
{
	uint32_t reg_val;
	uint32_t ret = -1;
	while (timeout--) {
		reg_val = x2_qspi_readl(xqspi, offset);
		if (!(reg_val & mask)) {
			ret = 0;
			break;
		}
	}
	return ret;
}

static int qspi_check_status(struct x2_qspi *xqspi, u32 offset, uint32_t mask, uint32_t timeout)
{
	int ret = 0;
	uint32_t val;

	do {
		val = x2_qspi_readl(xqspi, offset);
		timeout = timeout - 1;
		if (timeout == 0) {
			ret = -1;
			break;
		}
	} while (!(val & mask));

	return ret;
}

static void x2_qspi_chipselect(struct x2_qspi *q, bool is_high)
{
	if (is_high) {
		/* Deselect the slave */
		x2_qspi_writel(q, QSPI_CS, 0);
	} else {
		/* Activate the chip select */
		x2_qspi_writel(q, QSPI_CS, 1);
	}
}

void qspi_enable_tx(struct x2_qspi *xqspi)
{
	uint32_t val;
	val = x2_qspi_readl(xqspi, QSPI_CTL1);
	val |= TX_ENABLE;
	x2_qspi_writel(xqspi, QSPI_CTL1, val);
}

void qspi_disable_tx(struct x2_qspi *xqspi)
{
	uint32_t val;
	val = x2_qspi_readl(xqspi, QSPI_CTL1);
	val &= ~TX_ENABLE;
	x2_qspi_writel(xqspi, QSPI_CTL1, val);
}

void qspi_enable_rx(struct x2_qspi *xqspi)
{
	uint32_t val;
	val = x2_qspi_readl(xqspi, QSPI_CTL1);
	val |= RX_ENABLE;
	x2_qspi_writel(xqspi, QSPI_CTL1, val);
}

void qspi_disable_rx(struct x2_qspi *xqspi)
{
	uint32_t val;
	val = x2_qspi_readl(xqspi, QSPI_CTL1);
	val &= ~RX_ENABLE;
	x2_qspi_writel(xqspi, QSPI_CTL1, val);
}

void qspi_batch_mode_set(struct x2_qspi *xqspi, bool enable)
{
	uint32_t val;
	val = x2_qspi_readl(xqspi, QSPI_CTL3);
	if (enable)
		val &= ~BATCH_DISABLE;
	else
		val |= BATCH_DISABLE;
	x2_qspi_writel(xqspi, QSPI_CTL3, val);
}

void qspi_batch_tx_rx_flag_clr(struct x2_qspi *xqspi, bool is_tx)
{
	uint32_t val;
	val = x2_qspi_readl(xqspi, QSPI_STATUS1);
	if (is_tx)
		val |= BATCH_TXDONE;
	else
		val |= BATCH_RXDONE;
	x2_qspi_writel(xqspi, QSPI_STATUS1, val);
}

void qspi_reset_fifo(struct x2_qspi *xqspi)
{
	uint32_t val;

	val = x2_qspi_readl(xqspi, QSPI_CTL3);
	val |= FIFO_RESET;
	x2_qspi_writel(xqspi, QSPI_CTL3, val);

	val &= (~FIFO_RESET);
	x2_qspi_writel(xqspi, QSPI_CTL3, val);
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

	return SCLK_VAL(div,scaler);
}

static void x2_qspi_hw_init(struct x2_qspi *xqspi)
{
	uint32_t qspi_div;
	uint32_t reg_val;

	/* set qspi clk div */
	qspi_div = caculate_qspi_divider(xqspi->ref_clk, xqspi->qspi_clk);
	x2_qspi_writel(xqspi, QSPI_SCLK_CON, qspi_div);

	/* set qspi work mode */
	reg_val = x2_qspi_readl(xqspi, QSPI_CTL1);
	reg_val |= ((SPI_MODE0 & 0x30) | MST_MODE | FIFO_WIDTH8 | MSB);
	x2_qspi_writel(xqspi, QSPI_CTL1, reg_val);

	x2_qspi_writel(xqspi, QSPI_FIFO_TXTRIG_LVL, FIFO_DEPTH / 2);
	x2_qspi_writel(xqspi, QSPI_FIFO_RXTRIG_LVL, FIFO_DEPTH / 2);

	/* Disable all interrupt */
	x2_qspi_writel(xqspi, QSPI_CTL2, 0x0);

	/* unselect chip */
	reg_val = x2_qspi_readl(xqspi, QSPI_CS);
	reg_val &= 0x0;
	x2_qspi_writel(xqspi, QSPI_CS, reg_val);

	/* Always set SPI to one line as init. */
	reg_val = x2_qspi_readl(xqspi, QSPI_DUAL_QUAD_MODE);
	reg_val |= 0xfc;
	x2_qspi_writel(xqspi, QSPI_DUAL_QUAD_MODE, reg_val);

	/* Disable hardware xip mode */
	reg_val = x2_qspi_readl(xqspi, QSPI_XIP_CFG);
	reg_val &= ~(1 << 1);
	x2_qspi_writel(xqspi, QSPI_XIP_CFG, reg_val);

}

int x2_qspi_transfer(struct x2_qspi *xqspi, const void *txbuf, uint32_t len)
{
	uint32_t tx_len;
	uint32_t i;
	const uint8_t *ptr = (const uint8_t *)txbuf;
	uint32_t timeout = 0x10000;
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
			x2_qspi_writel(xqspi, QSPI_BATCH_CNT_TX, tx_len);
			tmp_txlen = MIN(tx_len, FIFO_DEPTH);
			for (i = 0; i < tmp_txlen; i++)
				x2_qspi_writel(xqspi, QSPI_TX_RX_REG, ptr[i]);

			qspi_enable_tx(xqspi);
			err = qspi_check_status(xqspi, QSPI_STATUS2, TXFIFO_EMPTY, timeout);
			if (err) {
				current_transfer_len = tmp_txlen;
				pr_err("%s:%d qspi send data timeout\n", __func__, __LINE__);
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
				x2_qspi_writel(xqspi, QSPI_TX_RX_REG, ptr[i]);

			qspi_enable_tx(xqspi);
			err = qspi_check_status(xqspi, QSPI_STATUS2, TXFIFO_EMPTY, timeout);
			if (err) {
				pr_err("%s:%d qspi send data timeout\n", __func__, __LINE__);
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
	pr_err("error: qspi tx current_transfer_len: %d\n", current_transfer_len);
	return current_transfer_len;
}

static int x2_qspi_recv(struct x2_qspi *xqspi, uint8_t *rxbuf, uint32_t len)
{
	int32_t i = 0;
	uint32_t rx_len = 0;
	uint8_t *ptr = rxbuf;
	uint32_t level;
	uint32_t rx_remain = len;
	uint32_t timeout = 0x10000;
	uint32_t tmp_rxlen;
	uint32_t current_receive_len = 0;
	level = x2_qspi_readl(xqspi, QSPI_FIFO_RXTRIG_LVL);
	qspi_reset_fifo(xqspi);

	if (xqspi->batch_mode) {
		qspi_batch_mode_set(xqspi, 1);
		do {
			rx_len = MIN(rx_remain, 0xFFFF);
			rx_remain -= rx_len;

			/* clear BATCH_RXDONE bit */
			qspi_batch_tx_rx_flag_clr(xqspi, 0);

			x2_qspi_writel(xqspi, QSPI_BATCH_CNT_RX, rx_len);

			qspi_enable_rx(xqspi);

			while (rx_len > 0) {
				if (rx_len > level) {
					tmp_rxlen = level;
					if (x2_qspi_poll_rx_empty(xqspi, QSPI_STATUS2, RXFIFO_EMPTY, timeout)) {
						pr_err("%s:%d timeout no data fill into rx fifo\n", __func__, __LINE__);
						goto SPI_ERROR;
					}
					for (i = 0; i < tmp_rxlen; i++)
						ptr[i] = x2_qspi_readl(xqspi, QSPI_TX_RX_REG);

					rx_len -= tmp_rxlen;
					ptr += tmp_rxlen;
					current_receive_len += tmp_rxlen;
				} else {
					tmp_rxlen = rx_len;
					if (x2_qspi_poll_rx_empty(xqspi,QSPI_STATUS2, RXFIFO_EMPTY, timeout)) {
						pr_err("%s:%d timeout no data fill into rx fifo\n", __func__, __LINE__);
						goto SPI_ERROR;
					}
					for (i = 0; i < tmp_rxlen; i++)
						ptr[i] = x2_qspi_readl(xqspi, QSPI_TX_RX_REG);

					rx_len -= tmp_rxlen;
					ptr += tmp_rxlen;
					current_receive_len += tmp_rxlen;
				}
			}
			if (qspi_check_status(xqspi, QSPI_STATUS1, BATCH_RXDONE, timeout)) {
				pr_err("%s:%d timeout loop batch rx done\n", __func__, __LINE__);
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
					if (qspi_check_status(xqspi, QSPI_STATUS2, TXFIFO_EMPTY, timeout)) {
						pr_err("%s:%d generate read sclk failed\n", __func__, __LINE__);
						goto SPI_ERROR;
					}

					for (i = 0; i < tmp_rxlen; i++)
						x2_qspi_writel(xqspi, QSPI_TX_RX_REG, 0x0);

					if (x2_qspi_poll_rx_empty(xqspi, QSPI_STATUS2, RXFIFO_EMPTY, timeout)) {
						pr_err("%s:%d timeout no data fill into rx fifo\n", __func__, __LINE__);
						goto SPI_ERROR;
					}
					for (i = 0; i < tmp_rxlen; i++)
						ptr[i] = x2_qspi_readl(xqspi, QSPI_TX_RX_REG);

					rx_len -= tmp_rxlen;
					ptr += tmp_rxlen;
					current_receive_len += tmp_rxlen;
				} else {
					tmp_rxlen = rx_len;
					if (qspi_check_status(xqspi, QSPI_STATUS2, TXFIFO_EMPTY, timeout)) {
						pr_err("%s:%d generate read sclk failed\n", __func__, __LINE__);
						goto SPI_ERROR;
					}

					for (i = 0; i < tmp_rxlen; i++)
						x2_qspi_writel(xqspi, QSPI_TX_RX_REG, 0x0);

					if (x2_qspi_poll_rx_empty(xqspi,QSPI_STATUS2, RXFIFO_EMPTY, timeout)) {
						pr_err("%s:%d timeout no data fill into rx fifo\n", __func__, __LINE__);
						goto SPI_ERROR;
					}
					for (i = 0; i < tmp_rxlen; i++)
						ptr[i] = x2_qspi_readl(xqspi, QSPI_TX_RX_REG);

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

static int x2_qspi_nor_setup(struct x2_qspi *q)
{
        x2_qspi_hw_init(q);
        return 0;
}

static void nor_flash_addr2cmd(struct spi_nor *nor, unsigned int addr, u8 *cmd)
{
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (nor->addr_width * 8 -  8);
	cmd[2] = addr >> (nor->addr_width * 8 - 16);
	cmd[3] = addr >> (nor->addr_width * 8 - 24);
	cmd[4] = addr >> (nor->addr_width * 8 - 32);
}

/* Only dual/quad read ops need read dummy */
static int nor_flash_read_cmdsz(struct spi_nor *nor)
{
        u8 cmd_sz = 0;
        u8 dummy_bytes = 0;

        dummy_bytes = nor->read_dummy / 8;

        cmd_sz = 1 + nor->addr_width + dummy_bytes;
	return cmd_sz;
}

/* For caculate PP/Q_PP/erase ops cmd size */
static int nor_flash_common_cmdsz(struct spi_nor *nor)
{
        return 1 + nor->addr_width;
}

static int x2_qspi_runcmd(struct x2_qspi *q, u8 cmd, int with_addr,
        unsigned int addr, int cmd_len)
{
        int ret = 0;
        int transfer_len = 0;

        /* cmd without addr */
        if (!with_addr) {
                q->command[0] = cmd;
                transfer_len = x2_qspi_transfer(q, q->command, cmd_len);
                if (transfer_len < cmd_len) {
                        ret = -1;
                        pr_err("send cmd without addr failed\n");
                }
        } else {  /* cmd with addr */
                q->command[0] = cmd;
                nor_flash_addr2cmd(&q->nor, addr, q->command);
                transfer_len = x2_qspi_transfer(q, q->command, cmd_len);
                if (transfer_len < cmd_len) {
                        ret = -1;
                        pr_err("send cmd with addr failed, transfer_len:%d\n", transfer_len);
                }

        }
        return ret;
}

static int nor_flash_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	int ret;
	struct x2_qspi *q = nor->priv;
        int recv_len;

        x2_qspi_chipselect(q, false);
	ret = x2_qspi_runcmd(q, opcode, 0, 0, 1);
	if (ret)
		goto err;

	recv_len = x2_qspi_recv(q, buf, len);
        if (recv_len != len) {
                ret = recv_len;
                goto err;
        }
        x2_qspi_chipselect(q, true);
	return 0;
err:
        x2_qspi_chipselect(q, true);
	return ret;

}

static int nor_flash_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct x2_qspi *q = nor->priv;
	int ret = 0;
        int write_data_len;

        x2_qspi_chipselect(q, false);
	if (!buf) {
		ret = x2_qspi_runcmd(q, opcode, 0, 0, 1);
		if (ret)
			goto error;

	} else if (len > 0) {
		ret = x2_qspi_runcmd(q, opcode, 0, 0, 1);
		if (ret)
			goto error;
                write_data_len = x2_qspi_transfer(q, buf, len);
                if (write_data_len != len) {
                        pr_err("write data to flash reg failed\n");
                        goto error;
                }
	} else {
		dev_err(q->dev, "invalid cmd %d\n", opcode);
		ret = -EINVAL;
                goto error;
	}

        x2_qspi_chipselect(q, true);
        return ret;
error:
        x2_qspi_chipselect(q, true);
        return ret;
}

static uint32_t qspi_data_line_config(struct x2_qspi *q)
{
        uint32_t ret = 0, val;

        switch(q->data_width) {
        case QSPI_INST_TYPE_SINGLE:
                /* set qspi controller to single line mode */
                val = x2_qspi_readl(q, QSPI_DUAL_QUAD_MODE);
                val |= (HOLD_OUTPUT | HOLD_OE | HOLD_CTL | WP_OUTPUT | WP_OE | WP_CTL);
                val &= ~(QPI_ENABLE | DPI_ENABLE);
                x2_qspi_writel(q, QSPI_DUAL_QUAD_MODE, val);
                break;
        case QSPI_INST_TYPE_DUAL:
                /* set qspi controller to dual line mode
                 * for write or read data
                 */
                val = x2_qspi_readl(q, QSPI_DUAL_QUAD_MODE);
                val |= DPI_ENABLE;
                x2_qspi_writel(q, QSPI_DUAL_QUAD_MODE, val);
                break;
        case QSPI_INST_TYPE_QUAD:
                /* set qspi controller to quad line mode
                 * for write or read data
                 */
                val = x2_qspi_readl(q, QSPI_DUAL_QUAD_MODE);
                val &= ~(HOLD_OUTPUT | HOLD_OE | HOLD_CTL | WP_OUTPUT | WP_OE | WP_CTL | DPI_ENABLE);
                val |= QPI_ENABLE;
                x2_qspi_writel(q, QSPI_DUAL_QUAD_MODE, val);
                break;
        default:
                return ret;
        }

        return ret;
}

static int qspi_set_protocol(struct spi_nor *nor, bool set, bool read)
{
        struct x2_qspi *q = nor->priv;

        if (set) {
                if (read) {
                        switch (nor->read_proto) {
                        case SNOR_PROTO_1_1_1:
                                q->data_width = QSPI_INST_TYPE_SINGLE;
                                break;
                        case SNOR_PROTO_1_1_2:
                                /* set qspi controller to dual line mode
                                 * for write or read data
                                 */
                                q->data_width = QSPI_INST_TYPE_DUAL;
                                break;
                        case SNOR_PROTO_1_1_4:
                                /* set qspi controller to quad line mode
                                 * for write or read data
                                 */
                                q->data_width = QSPI_INST_TYPE_QUAD;
                                break;
                        default:
                                return -EINVAL;
                        }
                } else {
                        switch (nor->write_proto) {
                        case SNOR_PROTO_1_1_1:
                                /* nothing to done, qspi default is single line mode */
                                q->data_width = QSPI_INST_TYPE_SINGLE;
                                break;
                        case SNOR_PROTO_1_1_2:
                                /* set qspi controller to dual line mode
                                 * for write or read data
                                 */
                                q->data_width = QSPI_INST_TYPE_DUAL;
                                break;
                        case SNOR_PROTO_1_1_4:
                                /* set qspi controller to quad line mode
                                 * for write or read data
                                 */
                                q->data_width = QSPI_INST_TYPE_QUAD;
                                break;
                        default:
                                return -EINVAL;
                        }
                }

        } else {
                /* unset qspi controller to single mode */
                q->data_width = QSPI_INST_TYPE_SINGLE;
        }
        dev_dbg(nor->dev, "set qspi read_proto:0x%x write_proto:0x%x\n",
                nor->read_proto, nor->write_proto);
        qspi_data_line_config(q);

        return 0;
}

static ssize_t nor_flash_write(struct spi_nor *nor, loff_t to,
			      size_t len, const u_char *buf)
{
        int ret;
	struct x2_qspi *q = nor->priv;
	u8 cmd = nor->program_opcode;
        u8 cmd_sz;

        dev_dbg(nor->dev, "write cmd:0x%x offset:0x%llx size:0x%lx\n", cmd, to, len);

        cmd_sz = nor_flash_common_cmdsz(nor);

        x2_qspi_chipselect(q, false);

        /* first send write cmd */
        ret = x2_qspi_runcmd(q, cmd, 1, to, cmd_sz);

        /* send data */
         switch (q->tx_bus_width) {
                case 2:
                        qspi_set_protocol(nor, true, false);
                        break;
                case 4:
                        qspi_set_protocol(nor, true, false);
                        break;
                default:
                        break; /* when power on default is single line */

        }

        ret = x2_qspi_transfer(q, buf, len);
        if (ret != len) {
                ret = -1;
                qspi_set_protocol(nor, false, false);
                x2_qspi_chipselect(q, true);
                pr_err("write data to flash failed\n");
                return ret;
        }
        qspi_set_protocol(nor, false, false);
        x2_qspi_chipselect(q, true);
	return len;
}

static ssize_t nor_flash_read(struct spi_nor *nor, loff_t from,
			     size_t len, u_char *buf)
{
	struct x2_qspi *q = nor->priv;
	u8 cmd = nor->read_opcode;
        u8 cmd_sz;
        int ret;

        dev_dbg(nor->dev, "read cmd:0x%x offset:0x%llx size:0x%lx\n", cmd, from, len);
        cmd_sz = nor_flash_read_cmdsz(nor);

        x2_qspi_chipselect(q, false);

        /* first send read cmd */
        ret = x2_qspi_runcmd(q, cmd, 1, from, cmd_sz);

        /* recv  data */
        switch (q->rx_bus_width) {
                case 2:
                        qspi_set_protocol(nor, true, true);
                        break;
                case 4:
                        qspi_set_protocol(nor, true, true);
                        break;
                default:
                        break; /* when power on default is single line */

        }
        ret = x2_qspi_recv(q, buf, len);
        if (ret != len) {
                pr_err("read data from flash failed\n");
                qspi_set_protocol(nor, false, true);
                x2_qspi_chipselect(q, true);
                return ret;
        }
        qspi_set_protocol(nor, false, true);
        x2_qspi_chipselect(q, true);
	return len;
}

static int nor_flash_erase(struct spi_nor *nor, loff_t offs)
{
	struct x2_qspi *q = nor->priv;
	int ret;
        u8 cmd_sz;
	dev_dbg(nor->dev, "erase(0x%x) %dKiB start 0x%08x\n", nor->erase_opcode,
		nor->mtd.erasesize / 1024, (u32)offs);

        cmd_sz = nor_flash_common_cmdsz(nor);
        x2_qspi_chipselect(q, false);

        ret = x2_qspi_runcmd(q, nor->erase_opcode, 1, offs, cmd_sz);
	if (ret) {
                x2_qspi_chipselect(q, true);
		return ret;
        }

        x2_qspi_chipselect(q, true);

	return 0;
}

static int x2_qspi_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
#if 0
	struct x2_qspi *q = nor->priv;
	int ret;

	mutex_lock(&q->lock);

	ret = fsl_qspi_clk_prep_enable(q);
	if (ret)
		goto err_mutex;

	fsl_qspi_set_base_addr(q, nor);
	return 0;

err_mutex:
	mutex_unlock(&q->lock);
#endif
	return 0;
}

static void x2_qspi_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
#if 0
        struct x2_qspi *q = nor->priv;

        struct fsl_qspi *q = nor->priv;

	fsl_qspi_clk_disable_unprep(q);
	mutex_unlock(&q->lock);
#endif
}


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
        const struct spi_nor_hwcaps hwcaps = {
                .mask = SNOR_HWCAPS_READ |
			SNOR_HWCAPS_READ_FAST |
			SNOR_HWCAPS_READ_1_1_2 |
                        SNOR_HWCAPS_READ_1_1_4 |
                        SNOR_HWCAPS_PP |
                        SNOR_HWCAPS_PP_1_1_4,
        };

        struct device_node *np = pdev->dev.of_node;
        struct device_node *child;
        struct device_node *flash_child;
        struct device *dev = &pdev->dev;
        struct x2_qspi *q;
        struct resource *res;
        struct spi_nor *nor;
        struct mtd_info *mtd;
        int ret , i = 0;
        uint32_t nr_parts = 0;
        const char *label_name;
	u32 out_values[2];
        u32 rx_bus_width;
	u32 tx_bus_width;

        q = devm_kzalloc(dev, sizeof(*q), GFP_KERNEL);
        if (!q)
                return -ENOMEM;

        /* find the resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        q->iobase = devm_ioremap_resource(dev, res);
        if (IS_ERR(q->iobase)) {
                return PTR_ERR(q->iobase);
        }

        q->dev = dev;
        //TODO Current clock tree no supported
#if 0
        xqspi->pclk = devm_clk_get(&pdev->dev, "pclk");
        if (IS_ERR(xqspi->pclk)) {
                dev_err(dev, "pclk clock not found.\n");
                ret = PTR_ERR(xqspi->pclk);
                goto remove_master;
        }

        ret = clk_prepare_enable(xqspi->pclk);
        if (ret) {
                dev_err(dev, "Unable to enable APB clock.\n");
                goto remove_master;
        }

        xqspi->refclk = devm_clk_get(&pdev->dev, "ref_clk");
        if (IS_ERR(xqspi->refclk)) {
                dev_err(dev, "ref_clk clock not found.\n");
                ret = PTR_ERR(xqspi->refclk);
                goto clk_dis_pclk;
        }

        ret = clk_prepare_enable(xqspi->refclk);
        if (ret) {
                dev_err(dev, "Unable to enable device clock.\n");
                goto clk_dis_pclk;
        }
#else
        q->ref_clk = CONFIG_X2_QSPI_REF_CLK;
        q->qspi_clk = CONFIG_X2_QSPI_CLK;
#endif

        ret = x2_qspi_nor_setup(q);
        mutex_init(&q->lock);

        nor = &q->nor;
        mtd = &nor->mtd;

        nor->dev = dev;
        spi_nor_set_flash_node(nor, np);
        nor->priv = q;
        platform_set_drvdata(pdev, q);

        /* fill the hooks */
        nor->read_reg = nor_flash_read_reg;
        nor->write_reg = nor_flash_write_reg;
        nor->read = nor_flash_read;
        nor->write = nor_flash_write;
        nor->erase = nor_flash_erase;
        nor->prepare = x2_qspi_prep;
        nor->unprepare = x2_qspi_unprep;
	mtd->flags   = MTD_CAP_NORFLASH;

        for_each_available_child_of_node(pdev->dev.of_node, child) {

                ret = of_property_read_u32(child, "spi-rx-bus-width",
                        &rx_bus_width);
                if (!ret)
                        q->rx_bus_width = rx_bus_width;
                else
                        q->rx_bus_width = 1;

                ret = of_property_read_u32(child, "spi-tx-bus-width",
                        &tx_bus_width);
                if (!ret)
                        q->tx_bus_width = tx_bus_width;
                else
                        q->tx_bus_width = 1;

                ret = of_property_read_u32(child, "spi-max-frequency", &q->clk_rate);
                if (ret) {
                        dev_err(dev, "spi-max-frequency not found\n");
                        goto mutex_failed;
                }

	        for_each_child_of_node(child, flash_child)
		        nr_parts++;
                q->parts = kcalloc(nr_parts, sizeof(struct mtd_partition),
			    GFP_KERNEL);
	        if (!q->parts) {
		        kfree(q->parts);
		        goto mutex_failed;
	        }

                for_each_child_of_node(child, flash_child) {
                        ret = of_property_read_string(flash_child, "label", &label_name);
                        if (ret) {
                                pr_err("%s: Unable to find label key, ret=%d", __func__,
                                        ret);
                                goto free_parts;
                        }
                        q->parts[i].name = label_name;

                        ret = of_property_read_u32_array(flash_child, "reg",
							 out_values, 2);
			if (!ret) {
				q->parts[i].offset = out_values[0];
				q->parts[i].size = out_values[1];
			} else {
				q->parts[i].offset = 0;
				q->parts[i].size = 0;
			}
		        ++i;
                }
        }

        ret = spi_nor_scan(nor, NULL, &hwcaps);
        if (ret)
                goto mutex_failed;

        ret = mtd_device_register(mtd, q->parts, nr_parts);
        if (ret)
                goto mutex_failed;

        dev_info(dev, "X2 QuadSPI probe ok\n");
        return 0;

free_parts:
        kfree(q->parts);
mutex_failed:
	mutex_destroy(&q->lock);
	dev_err(dev, "X2 QuadSPI probe failed\n");
        return ret;
}

static int x2_qspi_remove(struct platform_device *pdev)
{
	struct x2_qspi *q = platform_get_drvdata(pdev);

	mtd_device_unregister(&q->nor.mtd);
	mutex_destroy(&q->lock);
        kfree(q->parts);

	return 0;
}

static const struct of_device_id x2_qspi_dt_ids[] = {
	{ .compatible = "hobot,x2-qspi-nor"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, x2_qspi_dt_ids);


static struct platform_driver x2_qspi_driver = {
	.driver = {
		.name	= "x2-quadspi",
		.bus	= &platform_bus_type,
		.of_match_table = x2_qspi_dt_ids,
	},
	.probe          = x2_qspi_probe,
	.remove		= x2_qspi_remove,
};
module_platform_driver(x2_qspi_driver);

MODULE_DESCRIPTION("X2 QuadSPI Controller Driver");
MODULE_AUTHOR("Horizon Robotics Inc.");
MODULE_LICENSE("GPL v2");
