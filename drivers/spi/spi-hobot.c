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

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/spinlock_types.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#ifdef CONFIG_HOBOT_DIAG
#include <soc/hobot/diag.h>
#endif
#include <linux/timer.h>
#include <linux/clk.h>
#ifdef CONFIG_SPI_HOBOT_DFS_PROTECT
#include <uapi/linux/sched/types.h>
#endif
#ifdef CONFIG_SPI_HOBOT_SPIDEV_MODULE
#include <linux/gpio.h>
#include "spi-hobot-slave.h"
#endif
#ifdef CONFIG_HOBOT_BUS_CLK_X3
#include <soc/hobot/hobot_bus.h>
#endif

#define VER			"HOBOT-spi_V20.200330"
/* hobot spi master or slave mode select*/
#define MASTER_MODE		(0)
#define SLAVE_MODE		(1)

#define HB_SPI_MAX_CS		(2)
#define HB_SELECT_SLAVE0	(0)
#define HB_SELECT_SLAVE1	(1)
#define HB_SELECT_SLAVE2	(2)

#define HB_SPI_NAME            "hobot_spi"
/* hobot SPI register offsets */
#define HB_SPI_TXD_REG         0x00	/* data transmit register */
#define HB_SPI_RXD_REG         0x04	/* data receive register */
#define HB_SPI_CTRL_REG        0x08	/* SPI control register */
#define HB_SPI_SSC_REG         0x0C	/* SSN limit configure register */
#define HB_SPI_SFSR_REG        0x10	/* SPI status register */
#define HB_SPI_RFTO_REG        0x14	/* Threshold of receive time out */
#define HB_SPI_TLEN_REG        0x18	/* transfer length */
#define HB_SPI_INST_REG        0x1C	/* instruction should be transmitted */
#define HB_SPI_INST_MASK_REG   0x20	/* instruction mask register */
#define HB_SPI_SRCPND_REG      0x24	/* interrupt source pending register */
#define HB_SPI_INTMASK_REG     0x28	/* interrupt mask register */
#define HB_SPI_INTSETMASK_REG  0x2C	/* interrupt set mask register */
#define HB_SPI_INTUNMASK_REG   0x30	/* interrupt unset mask register */
#define HB_SPI_DMA_CTRL0_REG   0x34	/* DMA control register_0 */
#define HB_SPI_DMA_CTRL1_REG   0x38	/* DMA control register_1 */
#define HB_SPI_TDMA_ADDR0_REG  0x3C	/* transmit dma addr_0 */
#define HB_SPI_TDMA_SIZE0_REG  0x40	/* transmit dma size_0 */
#define HB_SPI_TDMA_ADDR1_REG  0x44	/* transmit dma addr_1 */
#define HB_SPI_TDMA_SIZE1_REG  0x48	/* transmit dma size_1 */
#define HB_SPI_RDMA_ADDR0_REG  0x4C	/* receive  dma addr_0 */
#define HB_SPI_RDMA_SIZE0_REG  0x50	/* receive  dma size_0 */
#define HB_SPI_RDMA_ADDR1_REG  0x54	/* receive  dma addr_1 */
#define HB_SPI_RDMA_SIZE1_REG  0x58	/* receive  dma size_1 */
#define HB_SPI_TSIZE_REG       0x5C	/* current tx_size */
#define HB_SPI_RSIZE_REG       0x60	/* current rx_size */
#define HB_SPI_FIFO_RESET_REG  0x64	/* FIFO reset register */
#define HB_SPI_TDMA_SIZE_REG   0x68	/* tx_dma size for ping-pong */
#define HB_SPI_RDMA_SIZE_REG   0x6C	/* rx_dma size for ping-pong */
/* hobot SPI CTRL bit Masks */
#define HB_SPI_SS_OFFSET       30
#define HB_SPI_SS_MASK         0xC0000000
#define HB_SPI_DIVIDER_OFFSET  0
#define HB_SPI_DIVIDER_MASK    0x0000FFFF
#define HB_SPI_CORE_EN         BIT(16)
#define HB_SPI_SSAL            BIT(17)
#define HB_SPI_LSB             BIT(18)
#define HB_SPI_POLARITY        BIT(19)
#define HB_SPI_PHASE           BIT(20)
#define HB_SPI_SLAVE_MODE      BIT(21)
#define HB_SPI_RX_DIS          BIT(22)
#define HB_SPI_TX_DIS          BIT(23)
#define HB_SPI_RX_FIFOE        BIT(24)
#define HB_SPI_TX_FIFOE        BIT(25)
#define HB_SPI_DW_SEL          BIT(26)
#define HB_SPI_SAMP_SEL        BIT(27)
#define HB_SPI_SAMP_CNT0       BIT(28)
#define HB_SPI_SAMP_CNT1       BIT(29)

/* hobot SPI SFSR bit Masks */
#define HB_SPI_TIP_MASK        BIT(8)
#define HB_SPI_DATA_RDY        BIT(4)
#define HB_SPI_TF_EMPTY        BIT(5)
/* hobot SPI INT bit Masks */
#define HB_SPI_INT_ALL         0x000007FF
#define HB_SPI_INT_DR          BIT(0)
#define HB_SPI_INT_OE          BIT(1)
#define HB_SPI_INT_TEMPTY      BIT(2)
#define HB_SPI_INT_RX_DMAERR   BIT(3)
#define HB_SPI_INT_TX_DMAERR   BIT(4)
#define HB_SPI_INT_RX_TIMEOUT  BIT(5)
#define HB_SPI_INT_RX_FULL     BIT(6)
#define HB_SPI_INT_TX_EMPTY    BIT(7)
#define HB_SPI_INT_RX_BGDONE   BIT(8)
#define HB_SPI_INT_TX_BGDONE   BIT(9)
#define HB_SPI_INT_DMA_TRDONE  BIT(10)
/* hobot SPI FIFO_RESET bit Masks */
#define HB_SPI_FRST_TXCLEAR    BIT(0)
#define HB_SPI_FRST_RXCLEAR    BIT(1)
#define HB_SPI_FRST_RXDIS      BIT(6)
#define HB_SPI_FRST_TXDIS      BIT(7)
#define HB_SPI_FRST_ABORT      BIT(8)
/* hobot SPI DMA_CTRL0 bit Masks */
#define HB_SPI_DMAC0_RXBLEN0   BIT(0)	/* dma burst len0 */
#define HB_SPI_DMAC0_RXBLEN1   BIT(1)	/* dma burst len1 */
#define HB_SPI_DMAC0_RXAL      BIT(2)	/* ping pang rx auto link */
#define HB_SPI_DMAC0_RXMAXOS0  BIT(3)	/* outstanding trans0 rx */
#define HB_SPI_DMAC0_RXMAXOS1  BIT(4)	/* outstanding trans1 rx */
#define HB_SPI_DMAC0_RXBG      BIT(5)	/* ping pang mode need set rx */
#define HB_SPI_DMAC0_RXAPBSEL  BIT(6)	/* fifo mode need set rx apb */
#define HB_SPI_DMAC0_TXBLEN0   BIT(9)	/* dma burst len */
#define HB_SPI_DMAC0_TXBLEN1   BIT(10)	/* dma burst len */
#define HB_SPI_DMAC0_TXAL      BIT(11)	/* ping pang tx auto link */
#define HB_SPI_DMAC0_TXMAXOS0  BIT(12)	/* outstanding trans0 tx */
#define HB_SPI_DMAC0_TXMAXOS1  BIT(13)	/* outstanding trans1 tx */
#define HB_SPI_DMAC0_TXBG      BIT(14)	/* ping pang mode need set tx */
#define HB_SPI_DMAC0_TXAPBSEL  BIT(15)	/* fifo mode need set tx apb */
/* hobot SPI DMA_CTRL1 bit Masks */
#define HB_SPI_DMAC1_RXDSTART  BIT(0)
#define HB_SPI_DMAC1_RXDCFG    BIT(1)
#define HB_SPI_DMAC1_TXDSTART  BIT(5)
#define HB_SPI_DMAC1_TXDCFG    BIT(6)

/* hobot config Macro */
#define msecs_to_loops(t)      (loops_per_jiffy / 1000 * HZ * t)
#define HB_SPI_TIMEOUT         (msecs_to_jiffies(2000))
#define HB_SPI_FIFO_SIZE       (32)
/* hobot SPI operation Macro */
#define HB_SPI_OP_CORE_EN      1
#define HB_SPI_OP_CORE_DIS     0
#define HB_SPI_OP_NONE         (-1)
#define HB_SPI_OP_RX_FIFOE     BIT(0)
#define HB_SPI_OP_TX_FIFOE     BIT(1)
#define HB_SPI_OP_TR_FIFOE     (HB_SPI_OP_RX_FIFOE | HB_SPI_OP_TX_FIFOE)
#define HB_SPI_OP_RX_DIS       BIT(0)
#define HB_SPI_OP_TX_DIS       BIT(1)
#define HB_SPI_OP_TR_DIS       (HB_SPI_OP_RX_DIS | HB_SPI_OP_TX_DIS)

#define HB_SPI_DMA_BUFSIZE     (10240)

//#define CONFIG_HB_SPI_FIFO_MODE       /* no dma, fifo mode */
#define CONFIG_HB_SPI_DMA_SINGLE	/* dma signal buffer mode */
//#define CONFIG_HB_SPI_DMA_PPONG       /* dma ping pong buffer mode*/

#define hb_spi_rd(dev, reg)       ioread32((dev)->regs_base + (reg))
#define hb_spi_wr(dev, reg, val)  iowrite32((val), (dev)->regs_base + (reg))

static int debug;
static int slave_tout = 2000;
static int master_tout = 1000;
/*log the error status of the previous interrupt on the SPI IP*/
static int pre_errflg = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "spi: 0 close debug, other open debug");
module_param(slave_tout, int, 0644);
MODULE_PARM_DESC(slave_tout, "spi: slave timeout(sec), default 10 s");
module_param(master_tout, int, 0644);
MODULE_PARM_DESC(master_tout, "spi: master timeout(sec), default 2 s");

#ifdef CONFIG_SPI_HOBOT_SPIDEV_MODULE
char tx_rx_interrupt_conflict_flag;
EXPORT_SYMBOL(tx_rx_interrupt_conflict_flag);
int ap_response_flag;
EXPORT_SYMBOL(ap_response_flag);
#endif

struct hb_spi {
	struct spi_controller *controller;
	struct device *dev;	/* parent device */
	void __iomem *regs_base;/* virt. address of the control registers */
	int irq;		/* spi interrupt */
	u8 *txbuf;		/* pointer in the Tx buffer */
	u8 *rxbuf;		/* pointer in the Rx buffer */
	int len;		/* Number of bytes left to transfer */
	int txcnt;		/* count to be Tx */
	int rxcnt;		/* count to be Rx */
	u16 word_width;		/* Bits per word:16bit or 8bit */
	u32 mode;		/* current mode */
	u32 speed;		/* current speed */
	struct clk *spi_mclk;		/* clk source */
	struct reset_control *rst;	/* reset controller */
	struct completion completion;	/* xfer completion */

	u8 *tx_dma_buf;
	u8 *rx_dma_buf;
	dma_addr_t tx_dma_phys;
	dma_addr_t rx_dma_phys;

	int spi_id;
	int isslave;
	bool slave_aborted;

#ifdef CONFIG_HOBOT_BUS_CLK_X3
	struct mutex dpm_mtx;
	struct hobot_dpm dpm;
#endif

#ifdef CONFIG_SPI_HOBOT_DFS_PROTECT
	struct task_struct *task;
	wait_queue_head_t wq_spi;
#endif
};

#ifdef CONFIG_SPI_HOBOT_SPIDEV_MODULE
/*
info ap when J3 tx&rx buf is ready
ap start spi clk
*/
static int tx_or_rx_flag;//0:tx,1:rx
enum{
	tx_flag,
	rx_flag
};
#endif

static void hb_spi_dump(struct hb_spi *hbspi, char *str, unchar *buf,
	int len)
{
	int i = 0, j = 0, mul = 0, remain = 0;
	int m = 32;

	mul = len / m;
	remain = len % m;

	if (str)
		dev_err(hbspi->dev, "%s %d %d %d\n", str, len, mul, remain);

	for (i = 0; i < mul; i++) {
		printk("0x%04x:%02X %02X %02X %02X %02X %02X %02X %02X "
			"%02X %02X %02X %02X %02X %02X %02X %02X "
			"%02X %02X %02X %02X %02X %02X %02X %02X "
			"%02X %02X %02X %02X %02X %02X %02X %02X\n", i*m,
		buf[i*m + 0], buf[i*m + 1], buf[i*m + 2], buf[i*m + 3],
		buf[i*m + 4], buf[i*m + 5], buf[i*m + 6], buf[i*m + 7],
		buf[i*m + 8], buf[i*m + 9], buf[i*m + 10], buf[i*m + 11],
		buf[i*m + 12], buf[i*m + 13], buf[i*m + 14], buf[i*m + 15],
		buf[i*m + 16], buf[i*m + 17], buf[i*m + 18], buf[i*m + 19],
		buf[i*m + 20], buf[i*m + 21], buf[i*m + 22], buf[i*m + 23],
		buf[i*m + 24], buf[i*m + 25], buf[i*m + 26], buf[i*m + 27],
		buf[i*m + 28], buf[i*m + 29], buf[i*m + 30], buf[i*m + 31]);
	}

	for (j = 0; j < remain; j++)
		printk("%02X", buf[mul*m + j]);
	printk("\n");

	return;
}

/* spi enable with: tx/rx fifo enable? tx/rx check disable? */
static int hb_spi_en_ctrl(struct hb_spi *hbspi, int en_flag,
			  int fifo_flag, int tr_flag)
{
	u64 val = 0;

	val = hb_spi_rd(hbspi, HB_SPI_CTRL_REG);
	if (fifo_flag >= 0) {
		if (fifo_flag & HB_SPI_OP_TX_FIFOE)
			val |= HB_SPI_TX_FIFOE;
		else
			val &= (~HB_SPI_TX_FIFOE);
		if (fifo_flag & HB_SPI_OP_RX_FIFOE)
			val |= HB_SPI_RX_FIFOE;
		else
			val &= (~HB_SPI_RX_FIFOE);
	}
	if (tr_flag >= 0) {
		if (tr_flag & HB_SPI_OP_TX_DIS)
			val |= HB_SPI_TX_DIS;
		else
			val &= (~HB_SPI_TX_DIS);
		if (tr_flag & HB_SPI_OP_RX_DIS)
			val |= HB_SPI_RX_DIS;
		else
			val &= (~HB_SPI_RX_DIS);
	}
	if (en_flag)
		val |= HB_SPI_CORE_EN;
	else
		val &= (~HB_SPI_CORE_EN);

	hb_spi_wr(hbspi, HB_SPI_CTRL_REG, (u32)val);

	if (debug)
		pr_info("%s CTRL=%08X\n", __func__, val);

	return 0;
}

#ifdef CONFIG_HB_SPI_FIFO_MODE
/* spi tx fifo fill from txbuf with txcnt */
static int hb_spi_fill_txfifo(struct hb_spi *hbspi)
{
	int cnt = 0;

	if (hbspi->txbuf) {
		if (hbspi->word_width == 16) {
			while (hbspi->txcnt > 0 && cnt < HB_SPI_FIFO_SIZE) {
				hb_spi_wr(hbspi, HB_SPI_TXD_REG,
					  ((*hbspi->txbuf) << 8) +
					  *(hbspi->txbuf + 1));
				hbspi->txbuf += 2;
				hbspi->txcnt -= 2;
				cnt += 2;
			}
		} else {
			while (hbspi->txcnt > 0 && cnt < HB_SPI_FIFO_SIZE) {
				hb_spi_wr(hbspi, HB_SPI_TXD_REG, *hbspi->txbuf);
				hbspi->txbuf++;
				hbspi->txcnt--;
				cnt++;
			}
		}
	} else {		/* trigger it when no tx */
		hb_spi_wr(hbspi, HB_SPI_TXD_REG, 0);
		cnt = 1;
	}

	hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_TXDIS);
	if (debug > 1)
		pr_info("%s cnt=%d\n", __func__, cnt);
	return cnt;
}

/* spi rx fifo drain to rxbuf with rxcnt */
static int hb_spi_drain_rxfifo(struct hb_spi *hbspi)
{
	int cnt = 0, rd;

	if (hbspi->rxbuf) {
		if (hbspi->word_width == 16) {
			while (hbspi->rxcnt > 0 && cnt < HB_SPI_FIFO_SIZE) {
				rd = hb_spi_rd(hbspi, HB_SPI_RXD_REG);
				*hbspi->rxbuf = rd >> 8;
				*(hbspi->rxbuf + 1) = rd;
				hbspi->rxbuf += 2;
				hbspi->rxcnt -= 2;
				cnt += 2;
			}
		} else {
			while (hbspi->rxcnt > 0 && cnt < HB_SPI_FIFO_SIZE) {
				*hbspi->rxbuf = hb_spi_rd(hbspi,
							  HB_SPI_RXD_REG);
				hbspi->rxbuf++;
				hbspi->rxcnt--;
				cnt++;
			}
		}
		hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_RXDIS);
	}
	if (debug > 1)
		pr_info("%s cnt=%d\n", __func__, cnt);
	return cnt;
}
#endif


#ifdef CONFIG_HB_SPI_DMA_SINGLE
#ifndef CONFIG_SPI_HOBOT_SPIDEV_MODULE
/* spi tx fifo fill from txbuf with txcnt */
static int hb_spi_fill_txdma(struct hb_spi *hbspi)
{
	int cnt = 0;
	u32 dma_ctrl1 = 0;

	if (hbspi->txbuf == NULL) {
		hb_spi_wr(hbspi, HB_SPI_TXD_REG, 0);
		return 0;
	}

	/* no data transfer, send complete */
	if (hbspi->txbuf && hbspi->txcnt == 0) {
		hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_ABORT);
		complete(&hbspi->completion);
		return 0;
	}

	if (debug > 1)
		hb_spi_dump(hbspi, "fill_txdma", hbspi->txbuf, hbspi->txcnt);

	dma_sync_single_for_cpu(hbspi->dev, hbspi->tx_dma_phys,
				HB_SPI_DMA_BUFSIZE, DMA_TO_DEVICE);
	while (hbspi->txcnt > 0 && cnt < HB_SPI_DMA_BUFSIZE) {
		hbspi->tx_dma_buf[cnt] = *hbspi->txbuf;
		hbspi->txbuf++;
		hbspi->txcnt--;
		cnt++;
	}
	dma_sync_single_for_device(hbspi->dev, hbspi->tx_dma_phys,
				   HB_SPI_DMA_BUFSIZE, DMA_TO_DEVICE);
	hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_TXDIS);

	if (cnt) {
		dma_ctrl1 |= HB_SPI_DMAC1_TXDSTART | HB_SPI_DMAC1_TXDCFG;
		if (hbspi->rxbuf)
			dma_ctrl1 |= HB_SPI_DMAC1_RXDSTART
			    | HB_SPI_DMAC1_RXDCFG;
		hb_spi_wr(hbspi, HB_SPI_TLEN_REG, cnt * 8);
		hb_spi_wr(hbspi, HB_SPI_TDMA_SIZE0_REG, cnt);
		hb_spi_wr(hbspi, HB_SPI_RDMA_SIZE0_REG, cnt);
		/* open tx and rx dma at same time avoid rx problem */
		hb_spi_wr(hbspi, HB_SPI_DMA_CTRL1_REG, dma_ctrl1);
	}
	return cnt;
}

static int hb_spi_drain_rxdma(struct hb_spi *hbspi)
{
	int cnt = 0;

	dma_sync_single_for_cpu(hbspi->dev, hbspi->rx_dma_phys,
				HB_SPI_DMA_BUFSIZE, DMA_FROM_DEVICE);
	if (hbspi->rxbuf) {
		while (hbspi->rxcnt > 0 && cnt < HB_SPI_DMA_BUFSIZE) {
			*hbspi->rxbuf = hbspi->rx_dma_buf[cnt];
			hbspi->rxbuf++;
			hbspi->rxcnt--;
			cnt++;
		}
	}
	dma_sync_single_for_device(hbspi->dev, hbspi->rx_dma_phys,
				   HB_SPI_DMA_BUFSIZE, DMA_FROM_DEVICE);
	hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_RXDIS);

	if (debug > 1)
		hb_spi_dump(hbspi, "drain_rxdma", hbspi->rx_dma_buf, cnt);

	return 0;
}
#else
/* spi tx fifo fill from txbuf with txcnt */
static int rx_end_flag;
static spi_header_byte rx_mask;
static int hb_spi_fill_txdma(struct hb_spi *hbspi)
{
	int cnt = 0, fragment_size = 0;
	u32 dma_ctrl1 = 0;
	struct spi_controller *ctlr = hbspi->controller;

	if (hbspi->txbuf == NULL) {
		hb_spi_wr(hbspi, HB_SPI_TXD_REG, 0);
		return 0;
	}

	/* no data transfer, send complete */
	if ((hbspi->txbuf && hbspi->txcnt == 0)
		|| (hbspi->isslave == SLAVE_MODE && rx_end_flag == 0)) {
		rx_end_flag = 0;
		hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_ABORT);
		complete(&hbspi->completion);
		return 0;
	}

	if (debug > 1)
		hb_spi_dump(hbspi, "fill_txdma", hbspi->txbuf, hbspi->txcnt);

	dma_sync_single_for_cpu(hbspi->dev, hbspi->tx_dma_phys,
								HB_SPI_DMA_BUFSIZE, DMA_TO_DEVICE);
	if (hbspi->isslave == SLAVE_MODE) {
		if (tx_or_rx_flag == tx_flag) {
			while (hbspi->txcnt > 0 && cnt < HB_SPI_DMA_BUFSIZE) {
				hbspi->tx_dma_buf[cnt] = *hbspi->txbuf;
				hbspi->txbuf++;
				hbspi->txcnt--;
				cnt++;

				if (hbspi->isslave == SLAVE_MODE && ctlr->custom_flag) {
					fragment_size++;
					if (fragment_size >= SPI_FRAGMENT_SIZE)
						break;
				}
			}
		} else if (tx_or_rx_flag == rx_flag) {
			while (hbspi->txcnt > 0 && cnt < HB_SPI_DMA_BUFSIZE && rx_end_flag == 1) {
				hbspi->tx_dma_buf[cnt] = *hbspi->txbuf;
				hbspi->txbuf++;
				hbspi->txcnt--;
				cnt++;

				if (hbspi->isslave == SLAVE_MODE && ctlr->custom_flag) {
					fragment_size++;
					if (fragment_size >= SPI_FRAGMENT_SIZE)
						break;
				}
			}
		}
	} else {
		while (hbspi->txcnt > 0 && cnt < HB_SPI_DMA_BUFSIZE) {
			hbspi->tx_dma_buf[cnt] = *hbspi->txbuf;
			hbspi->txbuf++;
			hbspi->txcnt--;
			cnt++;
		}
	}

	dma_sync_single_for_device(hbspi->dev, hbspi->tx_dma_phys,
								HB_SPI_DMA_BUFSIZE, DMA_TO_DEVICE);
	hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_TXDIS);

	if (cnt) {
		dma_ctrl1 |= HB_SPI_DMAC1_TXDSTART | HB_SPI_DMAC1_TXDCFG;
		if (hbspi->rxbuf)
			dma_ctrl1 |= HB_SPI_DMAC1_RXDSTART | HB_SPI_DMAC1_RXDCFG;
		hb_spi_wr(hbspi, HB_SPI_TLEN_REG, cnt * 8);
		hb_spi_wr(hbspi, HB_SPI_TDMA_SIZE0_REG, cnt);
		hb_spi_wr(hbspi, HB_SPI_RDMA_SIZE0_REG, cnt);
		/* open tx and rx dma at same time avoid rx problem */
		hb_spi_wr(hbspi, HB_SPI_DMA_CTRL1_REG, dma_ctrl1);
		if (hbspi->isslave == SLAVE_MODE && ctlr->info_ap)
			ctlr->info_ap();
	}
	return cnt;
}

static int hb_spi_drain_rxdma(struct hb_spi *hbspi)
{
	int cnt = 0, fragment_size = 0;
	struct spi_controller *ctlr = hbspi->controller;

	dma_sync_single_for_cpu(hbspi->dev, hbspi->rx_dma_phys,
						HB_SPI_DMA_BUFSIZE, DMA_FROM_DEVICE);
	if (hbspi->rxbuf) {
		while (hbspi->rxcnt > 0 && cnt < HB_SPI_DMA_BUFSIZE) {
			*hbspi->rxbuf = hbspi->rx_dma_buf[cnt];
			hbspi->rxbuf++;
			hbspi->rxcnt--;
			cnt++;

			if (hbspi->isslave == SLAVE_MODE && ctlr->custom_flag) {
				fragment_size++;
				if (fragment_size >= SPI_FRAGMENT_SIZE)
					break;
			}
		}
	}

	if (hbspi->isslave == SLAVE_MODE) {
		rx_mask.byte_value = hbspi->rx_dma_buf[4];
		if (tx_or_rx_flag == rx_flag && rx_mask.element_value.end == 1
			&& rx_mask.byte_value < 0x7) {
			rx_end_flag = 0;
		}
		if (tx_rx_interrupt_conflict_flag == 0 && tx_or_rx_flag == tx_flag
			&& hbspi->rx_dma_buf[0] == SPI_PREAMBLE) {
			tx_rx_interrupt_conflict_flag = 1;
		}
		if (tx_or_rx_flag == tx_flag && rx_mask.element_value.start == 1
			&& hbspi->rx_dma_buf[0] == SPI_PREAMBLE) {
			++ap_response_flag;
		}
	}
	dma_sync_single_for_device(hbspi->dev, hbspi->rx_dma_phys,
								HB_SPI_DMA_BUFSIZE, DMA_FROM_DEVICE);

	hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_RXDIS);

	if (debug > 1)
		hb_spi_dump(hbspi, "drain_rxdma", hbspi->rx_dma_buf, cnt);

	return 0;
}
#endif
#endif

#ifdef CONFIG_HOBOT_DIAG
static void spi_diag_report(uint8_t errsta, uint32_t srcpndreg,
						struct hb_spi *hbspi)
{
	uint8_t env_data[8];

	env_data[0] = (uint8_t)hbspi->spi_id;
	env_data[1] = 0xff;
	env_data[2] = 0;
	env_data[3] = sizeof(uint32_t);
	env_data[4] = srcpndreg & 0xff;
	env_data[5] = (srcpndreg >> 8) & 0xff;
	env_data[6] = (srcpndreg >> 16) & 0xff;
	env_data[7] = (uint8_t)((srcpndreg >> 24) & 0xff);
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_spi,
				(u16)(EventIdSpi0Err + hbspi->spi_id),
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&env_data,
				sizeof(uint8_t) * 8);
	} else {
		diag_send_event_stat(
			DiagMsgPrioHigh,
			ModuleDiag_spi,
			(u16)(EventIdSpi0Err + hbspi->spi_id),
			DiagEventStaSuccess);
	}
}
#endif

/* spi interrupt handle */
static irqreturn_t hb_spi_int_handle(int irq, void *data)
{
	u32 pnd;
	u8 errflg = 0;
	struct hb_spi *hbspi = (struct hb_spi *)data;

	pnd = hb_spi_rd(hbspi, HB_SPI_SRCPND_REG);
#if IS_ENABLED(CONFIG_HOBOT_DIAG_INJECT)
	diag_inject_val(ModuleDiag_spi, EventIdAny, &pnd);
#endif
	if (debug)
		dev_err(hbspi->dev, "irq=%d SRCPND=%08X\n", irq, pnd);
	if (pnd & (HB_SPI_INT_RX_FULL | HB_SPI_INT_DMA_TRDONE)) {
#ifdef	CONFIG_HB_SPI_FIFO_MODE
		hb_spi_drain_rxfifo(hbspi);
#elif defined CONFIG_HB_SPI_DMA_SINGLE
		hb_spi_drain_rxdma(hbspi);
#endif
	}
	if (pnd & HB_SPI_INT_TX_EMPTY) {
#ifdef	CONFIG_HB_SPI_FIFO_MODE
		hb_spi_fill_txfifo(hbspi);
#endif
	}
	if (pnd & HB_SPI_INT_RX_DMAERR) {
		dev_err(hbspi->dev, "INT_RX_DMAERR\n");
		errflg = 1;
	}

	if (pnd & HB_SPI_INT_OE) {
		dev_err(hbspi->dev, "INT_OE\n");
		errflg = 1;
	}

	if (pnd & HB_SPI_INT_TX_DMAERR) {
		dev_err(hbspi->dev, "INT_TX_DMAERR\n");
		errflg = 1;
	}

	if (pnd)
		hb_spi_wr(hbspi, HB_SPI_SRCPND_REG, pnd);

	if (pnd & HB_SPI_INT_DMA_TRDONE) {
#ifdef CONFIG_SPI_HOBOT_DFS_PROTECT
		wake_up_interruptible(&hbspi->wq_spi);
#endif

#ifdef	CONFIG_HB_SPI_FIFO_MODE
		hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_ABORT);
		complete(&hbspi->completion);
#elif defined CONFIG_HB_SPI_DMA_SINGLE
		hb_spi_fill_txdma(hbspi);
#endif
	}

#ifdef CONFIG_HOBOT_DIAG
	if (errflg)
		spi_diag_report(1, pnd, hbspi);
	if (pre_errflg == 1 && errflg == 0)
		spi_diag_report(0, pnd, hbspi);
	pre_errflg = errflg;
#endif

	return IRQ_HANDLED;
}

/* spi config: word_width, mode(pol, pha, lsb, cshigh), speed */
static int hb_spi_config(struct hb_spi *hbspi)
{
	u64 val, divider = 0;

	val = hb_spi_rd(hbspi, HB_SPI_CTRL_REG);
	/* bit width */
	val &= ~(HB_SPI_DW_SEL | HB_SPI_POLARITY | HB_SPI_PHASE |
		 HB_SPI_LSB | HB_SPI_SSAL);
	if (hbspi->word_width == 16)
		val |= HB_SPI_DW_SEL;
	/* spi mode */
	if (hbspi->mode & SPI_CPOL)
		val |= HB_SPI_POLARITY;
	if (hbspi->mode & SPI_CPHA)
		val |= HB_SPI_PHASE;
	if (hbspi->mode & SPI_LSB_FIRST)
		val |= HB_SPI_LSB;
	if (hbspi->mode & SPI_CS_HIGH)
		val |= HB_SPI_SSAL;
	/* spi speed */
	if (hbspi->speed) {
		divider = clk_get_rate(hbspi->spi_mclk) / hbspi->speed;
		divider = (divider < 2) ? 2 : divider;
		divider = (divider % 2) ? (divider / 2) : (divider / 2 - 1);
		val &= ~HB_SPI_DIVIDER_MASK;
		val |= (divider << HB_SPI_DIVIDER_OFFSET) & HB_SPI_DIVIDER_MASK;
	}
	hb_spi_wr(hbspi, HB_SPI_CTRL_REG, (u32)val);

	return 0;
}

/* spi master setup */
static int hb_spi_setup(struct spi_device *spi)
{
	struct hb_spi *hbspi = spi_master_get_devdata(spi->master);

	hbspi->word_width = spi->bits_per_word;
	hbspi->mode = spi->mode;
	hbspi->speed = spi->max_speed_hz;

	if (debug > 2)
		dev_err(hbspi->dev, "%s mode=%d speed=%d\n",
			__func__, hbspi->mode, hbspi->speed);

	return hb_spi_config(hbspi);
}

/* spi hw reset */
static int hb_spi_reset(struct hb_spi *hbspi)
{
	reset_control_assert(hbspi->rst);
	udelay(2);
	reset_control_deassert(hbspi->rst);
	return 0;
}

/* spi hw init */
static void hb_spi_init_hw(struct hb_spi *hbspi)
{
	u64 val = 0;

	/* First, should reset the whole controller */
	hb_spi_reset(hbspi);

	hb_spi_en_ctrl(hbspi, HB_SPI_OP_CORE_DIS, HB_SPI_OP_NONE,
		       HB_SPI_OP_NONE);
	hb_spi_wr(hbspi, HB_SPI_INTSETMASK_REG, HB_SPI_INT_ALL);
	/* clear all interrupt pending */
	hb_spi_wr(hbspi, HB_SPI_SRCPND_REG, HB_SPI_INT_ALL);
	/* init rfto */
	hb_spi_wr(hbspi, HB_SPI_RFTO_REG, 0x27F);
	/* no instruction */
	hb_spi_wr(hbspi, HB_SPI_INST_REG, 0x0);
	hb_spi_wr(hbspi, HB_SPI_INST_MASK_REG, 0xFFFFFFFF);
	/* spi master mode */
	val = hb_spi_rd(hbspi, HB_SPI_CTRL_REG);
	if (hbspi->isslave == SLAVE_MODE)
		val |= HB_SPI_SLAVE_MODE;
	else
		val &= (~HB_SPI_SLAVE_MODE);

	/* using the spi protocol to sampling the data for master mode,
	 * for slave mode, it will send out the data right after the tx edge
	 */
	val &= (~HB_SPI_SAMP_SEL);

	hb_spi_wr(hbspi, HB_SPI_CTRL_REG, (u32)val);

	if (debug)
		dev_err(hbspi->dev, "%s CTRL=%08X\n",
			__func__, hb_spi_rd(hbspi, HB_SPI_CTRL_REG));

	hb_spi_config(hbspi);
	hb_spi_en_ctrl(hbspi, HB_SPI_OP_CORE_EN, 0, 0);
}

/* This function enables SPI master controller */
static int hb_spi_prepare_xfer_hardware(struct spi_master *master)
{
	struct hb_spi *hbspi = spi_master_get_devdata(master);

	hb_spi_en_ctrl(hbspi, HB_SPI_OP_CORE_EN, HB_SPI_OP_NONE,
		       HB_SPI_OP_NONE);

	return 0;
}

static int hb_spi_unprepare_xfer_hardware(struct spi_master *master)
{
	struct hb_spi *hbspi = spi_master_get_devdata(master);

	hb_spi_en_ctrl(hbspi, HB_SPI_OP_CORE_DIS, HB_SPI_OP_NONE,
		       HB_SPI_OP_NONE);

	return 0;
}
// test code
static void hb_spi_reg_check(void)
{
#ifdef CONFIG_HOBOT_XJ3
	void __iomem *pin_reg;
	uint pin_base_addr = 0xA6004000;

	pin_reg = ioremap_nocache(pin_base_addr, 0x200);
	pr_err("spi0_csn0 0xA6004040=0x%08x\n", readl(pin_reg + 0x40));
	pr_err("spi0_csn1 0xA60041E0=0x%08x\n", readl(pin_reg + 0x1E0));

	iounmap(pin_reg);
#endif
}

/**
 * hb_spi_chipselect - Select or deselect the chip select line
 * @spi:     Pointer to the spi_device structure
 * @is_high: Select(0) or deselect (1) the chip select line
 */
static void hb_spi_chipselect(struct spi_device *spi, bool is_high)
{
	u32 val = 0;
	struct hb_spi *hbspi = spi_master_get_devdata(spi->master);

	if (spi->mode & SPI_NO_CS)
		return;

	if (spi->chip_select >= spi->master->num_chipselect) {
		dev_err(hbspi->dev, "Err chip_select=%d num_chipselect=%d\n",
			spi->chip_select, spi->master->num_chipselect);
		return;
	}
	val = hb_spi_rd(hbspi, HB_SPI_CTRL_REG);

	if (debug)
		dev_info(hbspi->dev, "A chip_select=%d CTRL=%08X high=%d\n",
			spi->chip_select, val, is_high);

	if (hbspi->spi_id == 0) {
		val &= (~HB_SPI_SS_MASK);
		val |= (spi->chip_select << HB_SPI_SS_OFFSET);
	} else {
		val &= (~HB_SPI_SS_MASK);
		val |= (HB_SELECT_SLAVE0 << HB_SPI_SS_OFFSET);
	}

	hb_spi_wr(hbspi, HB_SPI_CTRL_REG, val);
	if (debug > 2) {
		val = hb_spi_rd(hbspi, HB_SPI_CTRL_REG);
		dev_err(hbspi->dev, "B chip_select=%d CTRL=%08X high=%d\n",
			spi->chip_select, val, is_high);
		hb_spi_reg_check();
	}
}

/* spi wait for tpi before xfer: return 0 timeout */
static u64 hb_spi_wait_for_tpi(struct hb_spi *hbspi, int timeout_ms)
{
	u64 loop = 1;
	u32 status;

	if (timeout_ms)
		loop = msecs_to_loops(timeout_ms);

	do {
		status = hb_spi_rd(hbspi, HB_SPI_SFSR_REG);
	} while ((status & HB_SPI_TIP_MASK) && --loop);
	if (loop == 0 && ((status & HB_SPI_TIP_MASK) == 0))
		loop = 1;

	if (debug > 2)
		dev_err(hbspi->dev, "%s SFSR=0x%08X\n", __func__, status);

	return loop;
}

static int hb_spi_wait_for_completion(struct hb_spi *hbspi)
{
	uint64_t t_out;
	int ret = 0;

	if (hbspi->isslave == MASTER_MODE) {
		t_out = msecs_to_jiffies(master_tout);
		if (!wait_for_completion_timeout(&hbspi->completion, t_out)) {
			dev_err(hbspi->dev, "master: timeout\n");
			ret = -ETIMEDOUT;
			goto exit_1;
		}
	}

	if (hbspi->isslave == SLAVE_MODE) {
		t_out = msecs_to_jiffies(slave_tout);
		if (!wait_for_completion_timeout(&hbspi->completion, t_out)
			|| hbspi->slave_aborted) {
			dev_err(hbspi->dev, "slave: timeout or aborted\n");
			ret = -ETIMEDOUT;
			goto exit_1;
		}
	}
exit_1:
	return ret;
}

/* spi xfer one */
static int hb_spi_transfer_one(struct spi_master *master,
			       struct spi_device *spi,
			       struct spi_transfer *xfer)
{
	u32 ret;
	u32 fifo_flag = 0, tr_flag = 0;
	u32 int_umask = 0, dma_ctrl0 = 0, dma_ctrl1 = 0, fifo_reset = 0;
	struct hb_spi *hbspi = spi_master_get_devdata(master);
	int ms;
	u32 tlen;

#ifdef CONFIG_HOBOT_BUS_CLK_X3
	mutex_lock(&hbspi->dpm_mtx);
#endif
	hbspi->txbuf = (u8 *) xfer->tx_buf;
	hbspi->rxbuf = (u8 *) xfer->rx_buf;
	hbspi->len = xfer->len;

#ifdef CONFIG_SPI_HOBOT_SPIDEV_MODULE
	if (hbspi->isslave == SLAVE_MODE) {
		if (hbspi->txbuf[0] == SPI_PREAMBLE)
			tx_or_rx_flag = tx_flag;//tx
		else if (hbspi->txbuf[0] == DUMMY_FLAG)
			tx_or_rx_flag = rx_flag;//rx
	}
#endif

	if (debug > 1)
		dev_err(hbspi->dev, "%s len=%d\n", __func__, hbspi->len);
#ifdef CONFIG_HB_SPI_DMA_SINGLE
	if (hbspi->len > HB_SPI_DMA_BUFSIZE) {
		ret = -ENOMEM;
		dev_err(hbspi->dev, "Data is larger than DMA space(%d)\n",
			HB_SPI_DMA_BUFSIZE);
		goto exit_1;
	}
#endif
	/* millisecs to xfer 'len' bytes @ 'cur_speed' */
	ms = xfer->len * 8 * 1000 / hbspi->speed;
	ms += 10;		/* some tolerance */

	if (hb_spi_wait_for_tpi(hbspi, ms) == 0) {
		dev_err(hbspi->dev, "%s %d\n", __func__, __LINE__);
		ret = -EAGAIN;
		goto exit_1;
	}

	reinit_completion(&hbspi->completion);
	hbspi->slave_aborted = false;	//slave

	if (hbspi->word_width != xfer->bits_per_word ||
	    hbspi->speed != xfer->speed_hz) {
		hbspi->word_width = xfer->bits_per_word;
		hbspi->speed = xfer->speed_hz;
		hbspi->mode = spi->mode;
		hb_spi_config(hbspi);

		ms = xfer->len * 8 * 1000 / hbspi->speed;
		ms += 10;
	}

#ifdef	CONFIG_HB_SPI_FIFO_MODE
	/* prepare transfer with FIFO no-DMA */
	if (hbspi->txbuf) {
		hbspi->txcnt = hbspi->len;
		fifo_flag |= HB_SPI_OP_TX_FIFOE;
		int_umask |= HB_SPI_INT_TX_EMPTY | HB_SPI_INT_DMA_TRDONE;
		dma_ctrl0 |= HB_SPI_DMAC0_TXAPBSEL | HB_SPI_DMAC0_TXMAXOS0 |
		    HB_SPI_DMAC0_RXMAXOS0;
		dma_ctrl1 |= HB_SPI_DMAC1_TXDSTART;
		fifo_reset |= HB_SPI_FRST_TXCLEAR;
	} else {
		hbspi->txcnt = 0;
		tr_flag |= HB_SPI_OP_TX_DIS;
	}

	if (hbspi->rxbuf) {
		hbspi->rxcnt = hbspi->len;
		fifo_flag |= HB_SPI_OP_RX_FIFOE;
		int_umask |= HB_SPI_INT_RX_FULL | HB_SPI_INT_DMA_TRDONE;
		dma_ctrl0 |= HB_SPI_DMAC0_RXAPBSEL | HB_SPI_DMAC0_TXMAXOS0 |
		    HB_SPI_DMAC0_RXMAXOS0;
		dma_ctrl1 |= HB_SPI_DMAC1_RXDSTART;
		fifo_reset |= HB_SPI_FRST_RXCLEAR;
	} else {
		hbspi->rxcnt = 0;
		tr_flag |= HB_SPI_OP_RX_DIS;
	}
	tlen = xfer->len * 8;

#elif defined CONFIG_HB_SPI_DMA_SINGLE
	if (hbspi->tx_dma_buf == NULL || hbspi->rx_dma_buf == NULL) {
		dev_err(hbspi->dev, "%s %d\n", __func__, __LINE__);
		ret = -EAGAIN;
		goto exit_1;
	}

	hb_spi_wr(hbspi, HB_SPI_TDMA_ADDR0_REG, (u32)hbspi->tx_dma_phys);
	hb_spi_wr(hbspi, HB_SPI_RDMA_ADDR0_REG, (u32)hbspi->rx_dma_phys);

	if (hbspi->len < HB_SPI_DMA_BUFSIZE) {
		hb_spi_wr(hbspi, HB_SPI_TDMA_SIZE0_REG, hbspi->len);
		hb_spi_wr(hbspi, HB_SPI_RDMA_SIZE0_REG, hbspi->len);
		tlen = hbspi->len * 8;
	} else {
		hb_spi_wr(hbspi, HB_SPI_TDMA_SIZE0_REG, HB_SPI_DMA_BUFSIZE);
		hb_spi_wr(hbspi, HB_SPI_RDMA_SIZE0_REG, HB_SPI_DMA_BUFSIZE);
		tlen = HB_SPI_DMA_BUFSIZE * 8;
	}

	if (hbspi->txbuf) {
		hbspi->txcnt = hbspi->len;
		fifo_flag |= HB_SPI_OP_TX_FIFOE;
		int_umask |= HB_SPI_INT_TX_DMAERR | HB_SPI_INT_DMA_TRDONE;
		dma_ctrl0 |= HB_SPI_DMAC0_TXBLEN1 | HB_SPI_DMAC0_TXBLEN0 |
		    HB_SPI_DMAC0_TXMAXOS1 | HB_SPI_DMAC0_TXMAXOS0;
		dma_ctrl1 |= HB_SPI_DMAC1_TXDSTART | HB_SPI_DMAC1_TXDCFG;
		fifo_reset |= HB_SPI_FRST_TXCLEAR;
	} else {
		hbspi->txcnt = 0;
	}
	if (hbspi->rxbuf) {
		hbspi->rxcnt = hbspi->len;
		fifo_flag |= HB_SPI_OP_RX_FIFOE;
		int_umask |= HB_SPI_INT_RX_DMAERR | HB_SPI_INT_RX_BGDONE;
		dma_ctrl0 |= HB_SPI_DMAC0_RXBLEN1 | HB_SPI_DMAC0_RXBLEN0 |
		    HB_SPI_DMAC0_RXMAXOS1 | HB_SPI_DMAC0_RXMAXOS0;
		dma_ctrl1 |= HB_SPI_DMAC1_RXDSTART | HB_SPI_DMAC1_RXDCFG;
		fifo_reset |= HB_SPI_FRST_RXCLEAR;
	} else {
		hbspi->rxcnt = 0;
	}
#endif

	hb_spi_en_ctrl(hbspi, HB_SPI_OP_CORE_EN, fifo_flag, tr_flag);
	hb_spi_wr(hbspi, HB_SPI_TLEN_REG, tlen);
	hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, fifo_reset);
	hb_spi_wr(hbspi, HB_SPI_SRCPND_REG, HB_SPI_INT_ALL);
	hb_spi_wr(hbspi, HB_SPI_DMA_CTRL0_REG, dma_ctrl0);
	hb_spi_wr(hbspi, HB_SPI_INTUNMASK_REG, int_umask);

#ifdef	CONFIG_HB_SPI_FIFO_MODE
	hb_spi_fill_txfifo(hbspi);
	hb_spi_wr(hbspi, HB_SPI_DMA_CTRL1_REG, dma_ctrl1);
#elif defined CONFIG_HB_SPI_DMA_SINGLE
#ifdef CONFIG_SPI_HOBOT_SPIDEV_MODULE
	if (hbspi->isslave == SLAVE_MODE)
		rx_end_flag = 1;
#endif
	hb_spi_fill_txdma(hbspi);
#endif

	ret = hb_spi_wait_for_completion(hbspi);

	if (hb_spi_wait_for_tpi(hbspi, ms) == 0) {
		dev_err(hbspi->dev, "Err txcnt=%d len=%d\n",
			hbspi->txcnt, hbspi->len);
		hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_ABORT);
	}
	hb_spi_wr(hbspi, HB_SPI_INTSETMASK_REG, HB_SPI_INT_ALL);

	if (ret) {
		hb_spi_wr(hbspi, HB_SPI_FIFO_RESET_REG, HB_SPI_FRST_ABORT);
		if (debug > 1)
			dev_err(hbspi->dev, "Err wait for completion\n");
		//return -EAGAIN;
	}

exit_1:
	spi_finalize_current_transfer(master);

#ifdef CONFIG_HOBOT_BUS_CLK_X3
	mutex_unlock(&hbspi->dpm_mtx);
#endif
	return xfer->len;
}

#ifdef CONFIG_HB_SPI_DMA_SINGLE
static int hb_spi_request_dma(struct hb_spi *hbspi)
{
	struct device *dev = hbspi->dev;
	int ret;

	hbspi->tx_dma_buf = dma_alloc_coherent(dev, HB_SPI_DMA_BUFSIZE,
					       &hbspi->tx_dma_phys, GFP_KERNEL);
	if (!hbspi->tx_dma_buf) {
		ret = -ENOMEM;
		goto err_tx_dma_buf;
	}
	dma_sync_single_for_device(dev, hbspi->tx_dma_phys,
				   HB_SPI_DMA_BUFSIZE, DMA_TO_DEVICE);

	hbspi->rx_dma_buf = dma_alloc_coherent(dev, HB_SPI_DMA_BUFSIZE,
					       &hbspi->rx_dma_phys, GFP_KERNEL);
	if (!hbspi->rx_dma_buf) {
		ret = -ENOMEM;
		goto err_rx_dma_buf;
	}
	dma_sync_single_for_device(dev, hbspi->rx_dma_phys,
				   HB_SPI_DMA_BUFSIZE, DMA_FROM_DEVICE);

	return 0;

err_rx_dma_buf:
	dma_free_coherent(dev, HB_SPI_DMA_BUFSIZE,
			  hbspi->tx_dma_buf, hbspi->tx_dma_phys);
err_tx_dma_buf:
	return ret;
}

static void hb_spi_release_dma(struct hb_spi *hbspi)
{
	struct device *dev = hbspi->dev;

	if (hbspi->tx_dma_buf) {
		dma_free_coherent(dev, HB_SPI_DMA_BUFSIZE,
				  hbspi->tx_dma_buf, hbspi->tx_dma_phys);
	}

	if (hbspi->rx_dma_buf) {
		dma_free_coherent(dev, HB_SPI_DMA_BUFSIZE,
				  hbspi->rx_dma_buf, hbspi->rx_dma_phys);
	}
}
#endif

static int hb_spi_slave_setup(struct spi_device *spi)
{
	return hb_spi_setup(spi);
}

static int hb_spi_slave_prepare_message(
	struct spi_controller *ctlr,
	struct spi_message *msg)
{
	//struct spi_device *spi = msg->spi;
	struct hb_spi *hbspi = spi_controller_get_devdata(ctlr);

	//hb_spi_wr(hbspi, HB_SPI_SRCPND_REG, HB_SPI_INT_ALL);
	hb_spi_en_ctrl(hbspi, HB_SPI_OP_CORE_EN, HB_SPI_OP_NONE,
	       HB_SPI_OP_NONE);

	if (debug > 1)
		dev_err(hbspi->dev, "%s HB_SPI_OP_CORE_EN\n", __func__);

	return 0;
}
static int hb_spi_slave_transfer_one(struct spi_controller *ctlr,
							struct spi_device *spi, struct spi_transfer *xfer)
{
	return hb_spi_transfer_one(ctlr, spi, xfer);
}

static int hb_spi_slave_abort(struct spi_controller *ctlr)
{
	struct hb_spi *hbspi = spi_controller_get_devdata(ctlr);

	if (debug > 1)
		dev_err(hbspi->dev, "%s HB_SPI_OP_CORE_DIS\n", __func__);

	hb_spi_en_ctrl(hbspi, HB_SPI_OP_CORE_DIS, HB_SPI_OP_NONE,
			       HB_SPI_OP_NONE);

	hbspi->slave_aborted = true;
	complete(&hbspi->completion);

	return 0;
}

#ifdef CONFIG_HOBOT_BUS_CLK_X3
static int hb_spi_dpm_callback(struct hobot_dpm *self,
										unsigned long event, int state)
{
	int ret = 0;
	struct hb_spi *hbspi = container_of(self, struct hb_spi, dpm);

	if (event == HB_BUS_SIGNAL_START) {
		if (!mutex_trylock(&hbspi->dpm_mtx))
			return -EBUSY;
		disable_irq(hbspi->irq);
	} else if (event == HB_BUS_SIGNAL_END) {
		enable_irq(hbspi->irq);
		mutex_unlock(&hbspi->dpm_mtx);
	}

	return ret;
}
#endif

#ifdef CONFIG_SPI_HOBOT_DFS_PROTECT
extern void dmc_lock(void);
extern void dmc_unlock(void);

static int hb_spi_sync_thread(void *data)
{
	struct hb_spi *hbspi = data;
	unsigned long flags = 0;

	while (1) {
		dmc_lock();

		wait_event_interruptible_timeout(hbspi->wq_spi, flags, msecs_to_jiffies(200));

		dmc_unlock();

		msleep(5);
	}

	return 0;
}
#endif

static int hb_spi_probe(struct platform_device *pdev)
{
	struct hb_spi *hbspi = NULL;
	struct spi_controller *ctlr = NULL;
	struct resource *res = NULL;
	char spi_name[20];
	char ctrl_mode[16];
	int ret, spi_id, isslave = MASTER_MODE;
	u16 num_cs;
#ifdef CONFIG_SPI_HOBOT_DFS_PROTECT
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
#endif

	/* master or slave mode select */
	isslave = of_property_read_bool(pdev->dev.of_node, "slave");
	if (isslave == MASTER_MODE) {
		ctlr = spi_alloc_master(&pdev->dev, sizeof(*hbspi));
		if (!ctlr) {
			dev_err(&pdev->dev, "failed to alloc spi master\n");
			return -ENOMEM;
		}
	} else if (isslave == SLAVE_MODE) {
		ctlr = spi_alloc_slave(&pdev->dev, sizeof(*hbspi));
		if (!ctlr) {
			dev_err(&pdev->dev, "failed to alloc spi slave, try master\n");
			return -ENOMEM;
		}
    } else {
        dev_err(&pdev->dev, "failed to select mode\n");
        return -ENOMEM;
    }

	hbspi = spi_controller_get_devdata(ctlr);
	ctlr->dev.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, ctlr);
	hbspi->controller = ctlr;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hbspi->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hbspi->regs_base)) {
		ret = PTR_ERR_OR_ZERO(hbspi->regs_base);
		dev_err(&pdev->dev, "failed to determine base address\n");
		goto err_put_ctlr;
	}

	hbspi->dev = &pdev->dev;

	spi_id = of_alias_get_id(pdev->dev.of_node, "spi");
	sprintf(spi_name, "spi%d", spi_id);
	hbspi->rst = devm_reset_control_get(&pdev->dev, spi_name);
	if (IS_ERR(hbspi->rst)) {
		ret = PTR_ERR_OR_ZERO(hbspi->rst);
		dev_err(&pdev->dev, "missing controller reset %s\n", spi_name);
		goto err_put_ctlr;
	}

	hbspi->irq = platform_get_irq(pdev, 0);
	if (hbspi->irq < 0) {
		ret = hbspi->irq;
		dev_err(&pdev->dev, "failed to get irq (%d)\n", hbspi->irq);
		goto err_put_ctlr;
	}
	if (debug)
		dev_err(&pdev->dev, "Suc to get irq (%d)\n", hbspi->irq);

	ret = devm_request_irq(&pdev->dev, hbspi->irq, hb_spi_int_handle,
			       0, pdev->name, hbspi);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register irq (%d)\n", ret);
		goto err_put_ctlr;
	}

	hbspi->spi_mclk = devm_clk_get(&pdev->dev, "spi_mclk");
	if (IS_ERR(hbspi->spi_mclk)) {
		ret = PTR_ERR_OR_ZERO(hbspi->spi_mclk);
		dev_err(&pdev->dev, "failed to get spi_mclk: %d\n", ret);
		goto err_put_ctlr;
	}

	ret = clk_prepare_enable(hbspi->spi_mclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable spi_mclk (%d)\n", ret);
		goto err_put_ctlr;
	}

	init_completion(&hbspi->completion);

	ret = of_property_read_u16(pdev->dev.of_node, "num-cs", &num_cs);
	if (ret < 0)
		ctlr->num_chipselect = HB_SPI_MAX_CS;
	else
		ctlr->num_chipselect = num_cs;

	if (isslave == MASTER_MODE) {
		hbspi->isslave = MASTER_MODE;
		snprintf(ctrl_mode, sizeof(ctrl_mode), "%s", "master");
		ctlr->bus_num = (s16)pdev->id;
		ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST
								| SPI_CS_HIGH | SPI_NO_CS;
		ctlr->setup = hb_spi_setup;
		ctlr->prepare_transfer_hardware = hb_spi_prepare_xfer_hardware;
		ctlr->transfer_one = hb_spi_transfer_one;
		ctlr->unprepare_transfer_hardware = hb_spi_unprepare_xfer_hardware;
		ctlr->set_cs = hb_spi_chipselect;
		ctlr->dev.of_node = pdev->dev.of_node;
	} else if (isslave == SLAVE_MODE) {
		hbspi->isslave = SLAVE_MODE;
		snprintf(ctrl_mode, sizeof(ctrl_mode), "%s", "slave");
		ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;
		ctlr->setup = hb_spi_slave_setup;
		ctlr->prepare_message = hb_spi_slave_prepare_message;
		ctlr->transfer_one = hb_spi_slave_transfer_one;
		ctlr->slave_abort = hb_spi_slave_abort;
	}

	hb_spi_init_hw(hbspi);

#ifdef CONFIG_HB_SPI_DMA_SINGLE
	ret = hb_spi_request_dma(hbspi);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to hobot spi request dma(%d)\n", ret);
		goto clk_dis_mclk;
	}
#endif

	/* register spi controller */
	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret) {
		dev_err(&pdev->dev, "failed to register %s controller(%d)\n",
			ctrl_mode, ret);
		goto clk_dis_mclk;
	}

#ifdef CONFIG_HOBOT_BUS_CLK_X3
	mutex_init(&hbspi->dpm_mtx);
	hbspi->dpm.dpm_call = hb_spi_dpm_callback;
	hobot_dpm_register(&hbspi->dpm, hbspi->dev);
	if (ret)
		dev_err(&pdev->dev, "bus register failed\n");
#endif

#ifdef CONFIG_SPI_HOBOT_DFS_PROTECT
	init_waitqueue_head(&hbspi->wq_spi);
	hbspi->task = kthread_create(hb_spi_sync_thread, (void *)hbspi, "spi-sync");
	if (IS_ERR(hbspi->task)) {
		dev_err(hbspi->dev, "create kthread failed\n");
		goto clk_dis_mclk;
	}
	sched_setscheduler(hbspi->task, SCHED_FIFO, &param);
	wake_up_process(hbspi->task);
#endif

#ifdef CONFIG_HOBOT_DIAG
	/* diag */
	hbspi->spi_id = spi_id;
	if (diag_register(ModuleDiag_spi, (u16)(EventIdSpi0Err + spi_id),
			4, DIAG_MSG_INTERVAL_MIN, DIAG_MSG_INTERVAL_MAX, NULL) < 0)
		dev_err(hbspi->dev, "spi%d diag register fail\n", spi_id);
#endif

	dev_info(&pdev->dev, "ver: %s %s\n", VER, ctrl_mode);

	return 0;

clk_dis_mclk:
	clk_disable_unprepare(hbspi->spi_mclk);
err_put_ctlr:
	spi_controller_put(ctlr);

	return ret;
}

static int hb_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *ctlr = platform_get_drvdata(pdev);
	struct hb_spi *hbspi = spi_controller_get_devdata(ctlr);

#ifdef CONFIG_HOBOT_BUS_CLK_X3
	if (hbspi->dpm.dpm_call)
		hobot_dpm_unregister(&hbspi->dpm);
	hbspi->dpm.dpm_call = NULL;
#endif

	hb_spi_en_ctrl(hbspi, HB_SPI_OP_CORE_DIS, HB_SPI_OP_NONE,
		       HB_SPI_OP_NONE);
	pm_runtime_force_suspend(&pdev->dev);
#ifdef CONFIG_HB_SPI_DMA_SINGLE
	hb_spi_release_dma(hbspi);
#endif
	return 0;
}

#ifdef CONFIG_PM_SLEEP
int hb_spi_suspend(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);
	struct hb_spi *hbspi = spi_controller_get_devdata(ctlr);

	dev_info(hbspi->dev, "enter suspend...\n");

	/* Transfer in process flag, wait SPI to be idle */
	hb_spi_wait_for_tpi(hbspi, 1);

	/* disable spi */
	hb_spi_en_ctrl(hbspi, HB_SPI_OP_CORE_DIS, HB_SPI_OP_NONE,
			HB_SPI_OP_NONE);

	//disable clk to reduce power
	clk_disable_unprepare(hbspi->spi_mclk);

	return 0;
}

int hb_spi_resume(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);
	struct hb_spi *hbspi = spi_controller_get_devdata(ctlr);

	dev_info(hbspi->dev, "enter resume...\n");

	//enable clk to work
	clk_prepare_enable(hbspi->spi_mclk);

	hb_spi_init_hw(hbspi);

	return 0;
}
#else
int hb_spi_suspend(struct device *dev)
{
	return 0;
}

int hb_spi_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops hb_spi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hb_spi_suspend, hb_spi_resume)
};

static const struct of_device_id hb_spi_of_match[] = {
	{.compatible = "hobot,x2-spi"},
	{.compatible = "hobot,hobot-spi"},
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, hb_spi_of_match);

static struct platform_driver hb_spi_driver = {
    .probe = hb_spi_probe,
    .remove = hb_spi_remove,
    .driver = {
        .name = HB_SPI_NAME,
        .of_match_table = hb_spi_of_match,
		.pm = &hb_spi_dev_pm_ops,
    },
};

module_platform_driver(hb_spi_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("HOBOT SPI driver");
MODULE_LICENSE("GPL v2");
