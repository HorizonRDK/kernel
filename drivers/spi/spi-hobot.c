/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright 2019 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/
/**
 * @file	spi-hobot.c
 * @brief	X2 SPI controller driver (master/slave)
 * @version	V2.0
 * @author	Horizon
 * @date
 * @history	20191128 haibo.guo slave mode
 */

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
#include <soc/hobot/diag.h>
#include <linux/timer.h>
#include <linux/clk.h>

#define VER			"HOBOT-spi_V20.200206"
/* x2 spi master or slave mode select*/
#define MASTER_MODE		(0)
#define SLAVE_MODE		(1)

#define X2_SPI_MAX_CS		(3)
#define X2_SPI_NAME            "hobot_spi"
/* X2 SPI register offsets */
#define X2_SPI_TXD_REG         0x00	/* data transmit register */
#define X2_SPI_RXD_REG         0x04	/* data receive register */
#define X2_SPI_CTRL_REG        0x08	/* SPI control register */
#define X2_SPI_SSC_REG         0x0C	/* SSN limit configure register */
#define X2_SPI_SFSR_REG        0x10	/* SPI status register */
#define X2_SPI_RFTO_REG        0x14	/* Threshold of receive time out */
#define X2_SPI_TLEN_REG        0x18	/* transfer length */
#define X2_SPI_INST_REG        0x1C	/* instruction should be transmitted */
#define X2_SPI_INST_MASK_REG   0x20	/* instruction mask register */
#define X2_SPI_SRCPND_REG      0x24	/* interrupt source pending register */
#define X2_SPI_INTMASK_REG     0x28	/* interrupt mask register */
#define X2_SPI_INTSETMASK_REG  0x2C	/* interrupt set mask register */
#define X2_SPI_INTUNMASK_REG   0x30	/* interrupt unset mask register */
#define X2_SPI_DMA_CTRL0_REG   0x34	/* DMA control register_0 */
#define X2_SPI_DMA_CTRL1_REG   0x38	/* DMA control register_1 */
#define X2_SPI_TDMA_ADDR0_REG  0x3C	/* transmit dma addr_0 */
#define X2_SPI_TDMA_SIZE0_REG  0x40	/* transmit dma size_0 */
#define X2_SPI_TDMA_ADDR1_REG  0x44	/* transmit dma addr_1 */
#define X2_SPI_TDMA_SIZE1_REG  0x48	/* transmit dma size_1 */
#define X2_SPI_RDMA_ADDR0_REG  0x4C	/* receive  dma addr_0 */
#define X2_SPI_RDMA_SIZE0_REG  0x50	/* receive  dma size_0 */
#define X2_SPI_RDMA_ADDR1_REG  0x54	/* receive  dma addr_1 */
#define X2_SPI_RDMA_SIZE1_REG  0x58	/* receive  dma size_1 */
#define X2_SPI_TSIZE_REG       0x5C	/* current tx_size */
#define X2_SPI_RSIZE_REG       0x60	/* current rx_size */
#define X2_SPI_FIFO_RESET_REG  0x64	/* FIFO reset register */
#define X2_SPI_TDMA_SIZE_REG   0x68	/* tx_dma size for ping-pong */
#define X2_SPI_RDMA_SIZE_REG   0x6C	/* rx_dma size for ping-pong */
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
#define X2_SPI_SAMP_CNT0       BIT(28)
#define X2_SPI_SAMP_CNT1       BIT(29)

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
#define X2_SPI_DMAC0_RXBLEN0   BIT(0)	/* dma burst len0 */
#define X2_SPI_DMAC0_RXBLEN1   BIT(1)	/* dma burst len1 */
#define X2_SPI_DMAC0_RXAL      BIT(2)	/* ping pang rx auto link */
#define X2_SPI_DMAC0_RXMAXOS0  BIT(3)	/* outstanding trans0 rx */
#define X2_SPI_DMAC0_RXMAXOS1  BIT(4)	/* outstanding trans1 rx */
#define X2_SPI_DMAC0_RXBG      BIT(5)	/* ping pang mode need set rx */
#define X2_SPI_DMAC0_RXAPBSEL  BIT(6)	/* fifo mode need set rx apb */
#define X2_SPI_DMAC0_TXBLEN0   BIT(9)	/* dma burst len */
#define X2_SPI_DMAC0_TXBLEN1   BIT(10)	/* dma burst len */
#define X2_SPI_DMAC0_TXAL      BIT(11)	/* ping pang tx auto link */
#define X2_SPI_DMAC0_TXMAXOS0  BIT(12)	/* outstanding trans0 tx */
#define X2_SPI_DMAC0_TXMAXOS1  BIT(13)	/* outstanding trans1 tx */
#define X2_SPI_DMAC0_TXBG      BIT(14)	/* ping pang mode need set tx */
#define X2_SPI_DMAC0_TXAPBSEL  BIT(15)	/* fifo mode need set tx apb */
/* X2 SPI DMA_CTRL1 bit Masks */
#define X2_SPI_DMAC1_RXDSTART  BIT(0)
#define X2_SPI_DMAC1_RXDCFG    BIT(1)
#define X2_SPI_DMAC1_TXDSTART  BIT(5)
#define X2_SPI_DMAC1_TXDCFG    BIT(6)

/* X2 config Macro */
#define msecs_to_loops(t)      (loops_per_jiffy / 1000 * HZ * t)
#define X2_SPI_TIMEOUT         (msecs_to_jiffies(2000))
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

#define X2_SPI_DMA_BUFSIZE     (10240)

//#define CONFIG_X2_SPI_FIFO_MODE       /* no dma, fifo mode */
#define CONFIG_X2_SPI_DMA_SINGLE	/* dma signal buffer mode */
//#define CONFIG_X2_SPI_DMA_PPONG       /* dma ping pong buffer mode*/

#define x2_spi_rd(dev, reg)       ioread32((dev)->regs_base + (reg))
#define x2_spi_wr(dev, reg, val)  iowrite32((val), (dev)->regs_base + (reg))

struct timer_list spi_diag_timer;
static uint32_t spi_last_err_tm_ms;

static int debug;
static int slave_tout = 10;
static int master_tout = 2;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "spi: 0 close debug, other open debug");
module_param(slave_tout, int, 0644);
MODULE_PARM_DESC(slave_tout, "spi: slave timeout(sec), default 10 s");
module_param(master_tout, int, 0644);
MODULE_PARM_DESC(master_tout, "spi: master timeout(sec), default 2 s");

struct x2_spi {
	struct device *dev;	/* parent device */
	void __iomem *regs_base;/* virt. address of the control registers */
	int irq;		/* spi interrupt */
	u8 *txbuf;		/* pointer in the Tx buffer */
	u8 *rxbuf;		/* pointer in the Rx buffer */
	int len;		/* Number of bytes left to transfer */
	int txcnt;		/* count to be Tx */
	int rxcnt;		/* count to be Rx */
	u16 word_width;		/* Bits per word:16bit or 8bit */
	u16 mode;		/* current mode */
	u32 speed;		/* current speed */
	struct clk *spi_mclk;		/* clk source */
	struct reset_control *rst;	/* reset controller */
	struct completion completion;	/* xfer completion */

	u8 *tx_dma_buf;
	u8 *rx_dma_buf;
	dma_addr_t tx_dma_phys;
	dma_addr_t rx_dma_phys;

	int isslave;
	bool slave_aborted;
};

static void x2_spi_dump(struct x2_spi *x2spi, char *str, unchar *buf,
	int len)
{
	int i = 0, j = 0, mul = 0, remain = 0;
	int m = 32;

	mul = len / m;
	remain = len % m;

	if (str)
		dev_err(x2spi->dev, "%s %d %d %d\n", str, len, mul, remain);

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

	x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);

	if (debug)
		pr_info("%s CTRL=%08X\n", __func__, val);

	return 0;
}

#ifdef CONFIG_X2_SPI_FIFO_MODE
/* spi tx fifo fill from txbuf with txcnt */
static int x2_spi_fill_txfifo(struct x2_spi *x2spi)
{
	int cnt = 0;

	if (x2spi->txbuf) {
		if (x2spi->word_width == 16) {
			while (x2spi->txcnt > 0 && cnt < X2_SPI_FIFO_SIZE) {
				x2_spi_wr(x2spi, X2_SPI_TXD_REG,
					  ((*x2spi->txbuf) << 8) +
					  *(x2spi->txbuf + 1));
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
	} else {		/* trigger it when no tx */
		x2_spi_wr(x2spi, X2_SPI_TXD_REG, 0);
		cnt = 1;
	}

	x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_TXDIS);
	if (debug > 1)
		pr_info("%s cnt=%d\n", __func__, cnt);
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
	if (debug > 1)
		pr_info("%s cnt=%d\n", __func__, cnt);
	return cnt;
}
#endif

#ifdef CONFIG_X2_SPI_DMA_SINGLE
/* spi tx fifo fill from txbuf with txcnt */
static int x2_spi_fill_txdma(struct x2_spi *x2spi)
{
	int cnt = 0;
	u32 dma_ctrl1 = 0;

	if (x2spi->txbuf == NULL) {
		x2_spi_wr(x2spi, X2_SPI_TXD_REG, 0);
		return 0;
	}

	/* no data transfer, send complete */
	if (x2spi->txbuf && x2spi->txcnt == 0) {
		x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_ABORT);
		complete(&x2spi->completion);
		return 0;
	}

	if (debug > 1)
		x2_spi_dump(x2spi, "fill_txdma", x2spi->txbuf, x2spi->txcnt);

	dma_sync_single_for_cpu(x2spi->dev, x2spi->tx_dma_phys,
				X2_SPI_DMA_BUFSIZE, DMA_TO_DEVICE);
	while (x2spi->txcnt > 0 && cnt < X2_SPI_DMA_BUFSIZE) {
		x2spi->tx_dma_buf[cnt] = *x2spi->txbuf;
		x2spi->txbuf++;
		x2spi->txcnt--;
		cnt++;
	}
	dma_sync_single_for_device(x2spi->dev, x2spi->tx_dma_phys,
				   X2_SPI_DMA_BUFSIZE, DMA_TO_DEVICE);
	x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_TXDIS);

	if (cnt) {
		dma_ctrl1 |= X2_SPI_DMAC1_TXDSTART | X2_SPI_DMAC1_TXDCFG;
		if (x2spi->rxbuf)
			dma_ctrl1 |= X2_SPI_DMAC1_RXDSTART
			    | X2_SPI_DMAC1_RXDCFG;
		x2_spi_wr(x2spi, X2_SPI_TLEN_REG, cnt * 8);
		x2_spi_wr(x2spi, X2_SPI_TDMA_SIZE0_REG, cnt);
		x2_spi_wr(x2spi, X2_SPI_RDMA_SIZE0_REG, cnt);
		/* open tx and rx dma at same time avoid rx problem */
		x2_spi_wr(x2spi, X2_SPI_DMA_CTRL1_REG, dma_ctrl1);
	}
	return cnt;
}

static int x2_spi_drain_rxdma(struct x2_spi *x2spi)
{
	int cnt = 0;

	dma_sync_single_for_cpu(x2spi->dev, x2spi->rx_dma_phys,
				X2_SPI_DMA_BUFSIZE, DMA_FROM_DEVICE);
	if (x2spi->rxbuf) {
		while (x2spi->rxcnt > 0 && cnt < X2_SPI_DMA_BUFSIZE) {
			*x2spi->rxbuf = x2spi->rx_dma_buf[cnt];
			x2spi->rxbuf++;
			x2spi->rxcnt--;
			cnt++;
		}
	}
	dma_sync_single_for_device(x2spi->dev, x2spi->rx_dma_phys,
				   X2_SPI_DMA_BUFSIZE, DMA_FROM_DEVICE);
	x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_RXDIS);

	if (debug > 1)
		x2_spi_dump(x2spi, "drain_rxdma", x2spi->rx_dma_buf, cnt);

	return 0;
}
#endif

static void spi_diag_report(uint8_t errsta, uint32_t srcpndreg)
{
	spi_last_err_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_spi,
				EventIdSpiErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&srcpndreg,
				4);
	}
}

static void spi_diag_timer_func(unsigned long data)
{
	uint32_t now_tm_ms;
	unsigned long jiffi;

	now_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (now_tm_ms - spi_last_err_tm_ms > 5500) {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_spi,
				EventIdSpiErr,
				DiagEventStaSuccess);
	}
	jiffi = get_jiffies_64() + msecs_to_jiffies(2100);
	mod_timer(&spi_diag_timer, jiffi); // trrigr again.
}

/* spi interrupt handle */
static irqreturn_t x2_spi_int_handle(int irq, void *data)
{
	u32 pnd;
	u8 errflg = 0;
	struct x2_spi *x2spi = (struct x2_spi *)data;

	pnd = x2_spi_rd(x2spi, X2_SPI_SRCPND_REG);
	if (debug)
		dev_err(x2spi->dev, "irq=%d SRCPND=%08X\n", irq, pnd);
	if (pnd & (X2_SPI_INT_RX_FULL | X2_SPI_INT_DMA_TRDONE)) {
#ifdef	CONFIG_X2_SPI_FIFO_MODE
		x2_spi_drain_rxfifo(x2spi);
#elif defined CONFIG_X2_SPI_DMA_SINGLE
		x2_spi_drain_rxdma(x2spi);
#endif
	}
	if (pnd & X2_SPI_INT_TX_EMPTY) {
#ifdef	CONFIG_X2_SPI_FIFO_MODE
		x2_spi_fill_txfifo(x2spi);
#endif
	}
	if (pnd & X2_SPI_INT_RX_DMAERR) {
		dev_err(x2spi->dev, "INT_RX_DMAERR\n");
		errflg = 1;
	}

	if (pnd & X2_SPI_INT_OE) {
		dev_err(x2spi->dev, "INT_OE\n");
		errflg = 1;
	}

	if (pnd)
		x2_spi_wr(x2spi, X2_SPI_SRCPND_REG, pnd);

	if (pnd & X2_SPI_INT_DMA_TRDONE) {
#ifdef	CONFIG_X2_SPI_FIFO_MODE
		x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_ABORT);
		complete(&x2spi->completion);
#elif defined CONFIG_X2_SPI_DMA_SINGLE
		x2_spi_fill_txdma(x2spi);
#endif
	}
	if (errflg)
		spi_diag_report(1, pnd);

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
		 X2_SPI_LSB | X2_SPI_SSAL);
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
		divider = clk_get_rate(x2spi->spi_mclk) / x2spi->speed;
		divider = (divider < 2) ? 2 : divider;
		divider = (divider % 2) ? (divider / 2) : (divider / 2 - 1);
		val &= ~X2_SPI_DIVIDER_MASK;
		val |= (divider << X2_SPI_DIVIDER_OFFSET) & X2_SPI_DIVIDER_MASK;
	}
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

	if (debug > 2)
		dev_err(x2spi->dev, "%s mode=%d speed=%d\n",
			__func__, x2spi->mode, x2spi->speed);

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
	x2_spi_wr(x2spi, X2_SPI_INTSETMASK_REG, X2_SPI_INT_ALL);
	/* clear all interrupt pending */
	x2_spi_wr(x2spi, X2_SPI_SRCPND_REG, X2_SPI_INT_ALL);
	/* init rfto */
	x2_spi_wr(x2spi, X2_SPI_RFTO_REG, 0x27F);
	/* no instruction */
	x2_spi_wr(x2spi, X2_SPI_INST_REG, 0x0);
	x2_spi_wr(x2spi, X2_SPI_INST_MASK_REG, 0xFFFFFFFF);
	/* spi master mode */
	val = x2_spi_rd(x2spi, X2_SPI_CTRL_REG);
	if (x2spi->isslave == SLAVE_MODE)
		val |= X2_SPI_SLAVE_MODE;
	else
		val &= (~X2_SPI_SLAVE_MODE);
	val &= (~X2_SPI_SAMP_SEL);
	x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);

	if (debug)
		dev_err(x2spi->dev, "%s CTRL=%08X\n",
			__func__, x2_spi_rd(x2spi, X2_SPI_CTRL_REG));

	x2_spi_config(x2spi);
	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_EN, 0, 0);

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
		val |= (spi->chip_select << X2_SPI_SS_OFFSET);
	}
	x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);

	if (debug > 2)
		dev_err(x2spi->dev, "%s CTRL=%08X\n",
			__func__, x2_spi_rd(x2spi, X2_SPI_CTRL_REG));
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

	if (debug > 2)
		dev_err(x2spi->dev, "%s SFSR=0x%08X\n", __func__, status);

	return loop;
}

static int x2_spi_wait_for_completion(struct x2_spi *x2spi)
{
	uint64_t t_out;
	int ret = 0;

	if (x2spi->isslave == MASTER_MODE) {
		t_out = min(max(master_tout, 1), 10);
		t_out = t_out * 1000;
		t_out = msecs_to_jiffies(t_out);
		if (!wait_for_completion_timeout(&x2spi->completion, t_out)) {
			dev_err(x2spi->dev, "master: timeout\n");
			ret = -ETIMEDOUT;
			goto exit_1;
		}
	}

	if (x2spi->isslave == SLAVE_MODE) {
		t_out = min(max(slave_tout, 1), 100);
		t_out = t_out * 1000;
		t_out = msecs_to_jiffies(t_out);
		if (!wait_for_completion_timeout(&x2spi->completion, t_out)
			|| x2spi->slave_aborted) {
			dev_err(x2spi->dev, "slave: timeout or aborted\n");
			ret = -ETIMEDOUT;
			goto exit_1;
		}
	}
exit_1:
	return ret;
}

/* spi xfer one */
static int x2_spi_transfer_one(struct spi_master *master,
			       struct spi_device *spi,
			       struct spi_transfer *xfer)
{
	u32 ret;
	u32 fifo_flag = 0, tr_flag = 0;
	u32 int_umask = 0, dma_ctrl0 = 0, dma_ctrl1 = 0, fifo_reset = 0;
	struct x2_spi *x2spi = spi_master_get_devdata(master);
	int ms;
	u32 tlen;

	x2spi->txbuf = (u8 *) xfer->tx_buf;
	x2spi->rxbuf = (u8 *) xfer->rx_buf;
	x2spi->len = xfer->len;

	if (debug > 1)
		dev_err(x2spi->dev, "%s len=%d\n", __func__, x2spi->len);
#ifdef CONFIG_X2_SPI_DMA_SINGLE
	if (x2spi->len > X2_SPI_DMA_BUFSIZE) {
		ret = -ENOMEM;
		dev_err(x2spi->dev, "Data is larger than DMA space(%d)\n",
			X2_SPI_DMA_BUFSIZE);
		goto exit_1;
	}
#endif
	/* millisecs to xfer 'len' bytes @ 'cur_speed' */
	ms = xfer->len * 8 * 1000 / x2spi->speed;
	ms += 10;		/* some tolerance */

	if (x2spi->isslave == MASTER_MODE) {
		if (x2_spi_wait_for_tpi(x2spi, ms) == 0) {
			dev_err(x2spi->dev, "%s %d\n", __func__, __LINE__);
			return -EAGAIN;
		}
	}
	reinit_completion(&x2spi->completion);
	x2spi->slave_aborted = false;	//slave

	if (x2spi->word_width != xfer->bits_per_word ||
	    x2spi->speed != xfer->speed_hz) {
		x2spi->word_width = xfer->bits_per_word;
		x2spi->speed = xfer->speed_hz;
		x2spi->mode = spi->mode;
		x2_spi_config(x2spi);
	}
#ifdef	CONFIG_X2_SPI_FIFO_MODE
	/* prepare transfer with FIFO no-DMA */
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
	tlen = xfer->len * 8;

#elif defined CONFIG_X2_SPI_DMA_SINGLE
	if (x2spi->tx_dma_buf == NULL || x2spi->rx_dma_buf == NULL) {
		dev_err(x2spi->dev, "%s %d\n", __func__, __LINE__);
		return -EAGAIN;
	}

	x2_spi_wr(x2spi, X2_SPI_TDMA_ADDR0_REG, x2spi->tx_dma_phys);
	x2_spi_wr(x2spi, X2_SPI_RDMA_ADDR0_REG, x2spi->rx_dma_phys);

	if (x2spi->len < X2_SPI_DMA_BUFSIZE) {
		x2_spi_wr(x2spi, X2_SPI_TDMA_SIZE0_REG, x2spi->len);
		x2_spi_wr(x2spi, X2_SPI_RDMA_SIZE0_REG, x2spi->len);
		tlen = x2spi->len * 8;
	} else {
		x2_spi_wr(x2spi, X2_SPI_TDMA_SIZE0_REG, X2_SPI_DMA_BUFSIZE);
		x2_spi_wr(x2spi, X2_SPI_RDMA_SIZE0_REG, X2_SPI_DMA_BUFSIZE);
		tlen = X2_SPI_DMA_BUFSIZE * 8;
	}

	if (x2spi->txbuf) {
		x2spi->txcnt = x2spi->len;
		fifo_flag |= X2_SPI_OP_TX_FIFOE;
		int_umask |= X2_SPI_INT_TX_DMAERR | X2_SPI_INT_DMA_TRDONE;
		dma_ctrl0 |= X2_SPI_DMAC0_TXBLEN1 | X2_SPI_DMAC0_TXBLEN0 |
		    X2_SPI_DMAC0_TXMAXOS1 | X2_SPI_DMAC0_TXMAXOS0;
		dma_ctrl1 |= X2_SPI_DMAC1_TXDSTART | X2_SPI_DMAC1_TXDCFG;
		fifo_reset |= X2_SPI_FRST_TXCLEAR;
	} else {
		x2spi->txcnt = 0;
	}
	if (x2spi->rxbuf) {
		x2spi->rxcnt = x2spi->len;
		fifo_flag |= X2_SPI_OP_RX_FIFOE;
		int_umask |= X2_SPI_INT_RX_DMAERR | X2_SPI_INT_RX_BGDONE;
		dma_ctrl0 |= X2_SPI_DMAC0_RXBLEN1 | X2_SPI_DMAC0_RXBLEN0 |
		    X2_SPI_DMAC0_RXMAXOS1 | X2_SPI_DMAC0_RXMAXOS0;
		dma_ctrl1 |= X2_SPI_DMAC1_RXDSTART | X2_SPI_DMAC1_RXDCFG;
		fifo_reset |= X2_SPI_FRST_RXCLEAR;
	} else {
		x2spi->rxcnt = 0;
	}
#endif

	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_EN, fifo_flag, tr_flag);
	x2_spi_wr(x2spi, X2_SPI_TLEN_REG, tlen);
	x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, fifo_reset);
	x2_spi_wr(x2spi, X2_SPI_SRCPND_REG, X2_SPI_INT_ALL);
	x2_spi_wr(x2spi, X2_SPI_DMA_CTRL0_REG, dma_ctrl0);
	x2_spi_wr(x2spi, X2_SPI_INTUNMASK_REG, X2_SPI_INT_ALL);

#ifdef	CONFIG_X2_SPI_FIFO_MODE
	x2_spi_fill_txfifo(x2spi);
	x2_spi_wr(x2spi, X2_SPI_DMA_CTRL1_REG, dma_ctrl1);
#elif defined CONFIG_X2_SPI_DMA_SINGLE
	x2_spi_fill_txdma(x2spi);
#endif

	ret = x2_spi_wait_for_completion(x2spi);
	if (x2_spi_wait_for_tpi(x2spi, ms) == 0) {
		dev_err(x2spi->dev, "Err txcnt=%d len=%d\n",
			x2spi->txcnt, x2spi->len);
		x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_ABORT);
	}
	x2_spi_wr(x2spi, X2_SPI_INTSETMASK_REG, X2_SPI_INT_ALL);
	if (ret) {
		x2_spi_wr(x2spi, X2_SPI_FIFO_RESET_REG, X2_SPI_FRST_ABORT);
		if (debug > 1)
			dev_err(x2spi->dev, "Err wait for completion\n");
		//return -EAGAIN;
	}
	spi_finalize_current_transfer(master);
exit_1:
	return xfer->len;
}

#ifdef CONFIG_X2_SPI_DMA_SINGLE
static int x2_spi_request_dma(struct x2_spi *x2spi)
{
	struct device *dev = x2spi->dev;
	int ret;

	x2spi->tx_dma_buf = dma_alloc_coherent(dev, X2_SPI_DMA_BUFSIZE,
					       &x2spi->tx_dma_phys, GFP_KERNEL);
	if (!x2spi->tx_dma_buf) {
		ret = -ENOMEM;
		goto err_tx_dma_buf;
	}
	dma_sync_single_for_device(dev, x2spi->tx_dma_phys,
				   X2_SPI_DMA_BUFSIZE, DMA_TO_DEVICE);

	x2spi->rx_dma_buf = dma_alloc_coherent(dev, X2_SPI_DMA_BUFSIZE,
					       &x2spi->rx_dma_phys, GFP_KERNEL);
	if (!x2spi->rx_dma_buf) {
		ret = -ENOMEM;
		goto err_rx_dma_buf;
	}
	dma_sync_single_for_device(dev, x2spi->rx_dma_phys,
				   X2_SPI_DMA_BUFSIZE, DMA_FROM_DEVICE);

	return 0;

err_rx_dma_buf:
	dma_free_coherent(dev, X2_SPI_DMA_BUFSIZE,
			  x2spi->tx_dma_buf, x2spi->tx_dma_phys);
err_tx_dma_buf:
	return ret;
}

static void x2_spi_release_dma(struct x2_spi *x2spi)
{
	struct device *dev = x2spi->dev;

	if (x2spi->tx_dma_buf) {
		dma_free_coherent(dev, X2_SPI_DMA_BUFSIZE,
				  x2spi->tx_dma_buf, x2spi->tx_dma_phys);
	}

	if (x2spi->rx_dma_buf) {
		dma_free_coherent(dev, X2_SPI_DMA_BUFSIZE,
				  x2spi->rx_dma_buf, x2spi->rx_dma_phys);
	}
}
#endif

static int x2_spi_slave_setup(struct spi_device *spi)
{
	return x2_spi_setup(spi);
}

static int x2_spi_slave_prepare_message(
	struct spi_controller *ctlr,
	struct spi_message *msg)
{
	struct spi_device *spi = msg->spi;
	struct x2_spi *x2spi = spi_controller_get_devdata(ctlr);

	//x2_spi_wr(x2spi, X2_SPI_SRCPND_REG, X2_SPI_INT_ALL);
	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_EN, X2_SPI_OP_NONE,
	       X2_SPI_OP_NONE);

       if (debug > 1)
	       dev_err(x2spi->dev, "%s X2_SPI_OP_CORE_EN\n", __func__);

	return 0;
}
static int x2_spi_slave_transfer_one(struct spi_controller *ctlr,
	struct spi_device *spi,
	struct spi_transfer *xfer)
{
	return x2_spi_transfer_one(ctlr, spi, xfer);
}
static int x2_spi_slave_abort(struct spi_controller *ctlr)
{
	struct x2_spi *x2spi = spi_controller_get_devdata(ctlr);

	if (debug > 1)
		dev_err(x2spi->dev, "%s X2_SPI_OP_CORE_DIS\n", __func__);

	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_DIS, X2_SPI_OP_NONE,
			       X2_SPI_OP_NONE);

	x2spi->slave_aborted = true;
	complete(&x2spi->completion);

	return 0;
}

static int x2_spi_probe(struct platform_device *pdev)
{
	struct x2_spi *x2spi = NULL;
	struct spi_controller *ctlr = NULL;
	struct resource *res = NULL;
	char spi_name[20];
	char ctrl_mode[16];
	int ret, spi_id, rate, isslave = MASTER_MODE;

	/* master or slave mode select */
	of_property_read_u32(pdev->dev.of_node, "isslave", &isslave);
	if (isslave == SLAVE_MODE) {
		ctlr = spi_alloc_slave(&pdev->dev, sizeof(*x2spi));
		if (!ctlr) {
			dev_err(&pdev->dev,
				"failed to alloc spi slave, try master\n");
			isslave = MASTER_MODE;
		}
	} else {
		isslave = MASTER_MODE;
	}

	if (isslave == MASTER_MODE) {
		ctlr = spi_alloc_master(&pdev->dev, sizeof(*x2spi));
		if (!ctlr) {
			dev_err(&pdev->dev, "failed to alloc spi master\n");
			return -ENOMEM;
		}
	}

	x2spi = spi_controller_get_devdata(ctlr);
	ctlr->dev.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, ctlr);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	x2spi->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(x2spi->regs_base)) {
		ret = PTR_ERR(x2spi->regs_base);
		dev_err(&pdev->dev, "failed to determine base address\n");
		goto err_put_ctlr;
	}

	x2spi->dev = &pdev->dev;

	spi_id = of_alias_get_id(pdev->dev.of_node, "spi");
	sprintf(spi_name, "spi%d", spi_id);
	x2spi->rst = devm_reset_control_get(&pdev->dev, spi_name);
	if (IS_ERR(x2spi->rst)) {
		ret = PTR_ERR(x2spi->rst);
		dev_err(&pdev->dev, "missing controller reset %s\n", spi_name);
		goto err_put_ctlr;
	}

	x2spi->irq = platform_get_irq(pdev, 0);
	if (x2spi->irq < 0) {
		ret = x2spi->irq;
		dev_err(&pdev->dev, "failed to get irq (%d)\n", x2spi->irq);
		goto err_put_ctlr;
	}
	if (debug)
		dev_err(&pdev->dev, "Suc to get irq (%d)\n", x2spi->irq);

	ret = devm_request_irq(&pdev->dev, x2spi->irq, x2_spi_int_handle,
			       0, pdev->name, x2spi);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register irq (%d)\n", ret);
		goto err_put_ctlr;
	}

	x2spi->spi_mclk = devm_clk_get(&pdev->dev, "spi_mclk");
	if (IS_ERR(x2spi->spi_mclk)) {
		ret = PTR_ERR(x2spi->spi_mclk);
		dev_err(&pdev->dev, "failed to get spi_mclk: %d\n", ret);
		goto err_put_ctlr;
	}

	ret = clk_prepare_enable(x2spi->spi_mclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable spi_mclk (%d)\n", ret);
		goto err_put_ctlr;
	}

	rate = clk_get_rate(x2spi->spi_mclk);
	if (debug)
		dev_err(&pdev->dev, "spi clock is %d\n", rate);

	init_completion(&x2spi->completion);

	if (isslave == MASTER_MODE) {
		x2spi->isslave = MASTER_MODE;
		snprintf(ctrl_mode, sizeof(ctrl_mode), "%s", "master");
		ctlr->bus_num = pdev->id;
		ctlr->num_chipselect = X2_SPI_MAX_CS;
		ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST | SPI_CS_HIGH;
		ctlr->setup = x2_spi_setup;
		ctlr->prepare_transfer_hardware = x2_spi_prepare_xfer_hardware;
		ctlr->transfer_one = x2_spi_transfer_one;
		ctlr->unprepare_transfer_hardware = x2_spi_unprepare_xfer_hardware;
		ctlr->set_cs = x2_spi_chipselect;
		ctlr->dev.of_node = pdev->dev.of_node;
	} else {
		x2spi->isslave = SLAVE_MODE;
		snprintf(ctrl_mode, sizeof(ctrl_mode), "%s", "slave");
		ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;
		ctlr->setup = x2_spi_slave_setup;
		ctlr->prepare_message = x2_spi_slave_prepare_message;
		ctlr->transfer_one = x2_spi_slave_transfer_one;
		ctlr->slave_abort = x2_spi_slave_abort;
	}

	/* register spi controller */
	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret) {
		dev_err(&pdev->dev, "failed to register %s controller(%d)\n",
			ctrl_mode, ret);
		goto clk_dis_mclk;
	}

	x2_spi_init_hw(x2spi);

#ifdef CONFIG_X2_SPI_DMA_SINGLE
	ret = x2_spi_request_dma(x2spi);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to x2 spi request dma(%d)\n", ret);
		goto clk_dis_mclk;
	}
#endif
	/* diag */
	if (diag_register(ModuleDiag_spi, EventIdSpiErr,
						4, 300, 6000, NULL) < 0)
		dev_err(x2spi->dev, "spi diag register fail\n");
	else {
		spi_last_err_tm_ms = 0;
		init_timer(&spi_diag_timer);
		spi_diag_timer.expires =
			get_jiffies_64() + msecs_to_jiffies(1000);
		spi_diag_timer.data = 0;
		spi_diag_timer.function = spi_diag_timer_func;
		add_timer(&spi_diag_timer);
	}

	dev_err(&pdev->dev, "ver: %s %s\n", VER, ctrl_mode);

	return 0;

clk_dis_mclk:
	clk_disable_unprepare(x2spi->spi_mclk);
err_put_ctlr:
	spi_controller_put(ctlr);

	return ret;
}

static int x2_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *ctlr = platform_get_drvdata(pdev);
	struct x2_spi *x2spi = spi_controller_get_devdata(ctlr);

	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_DIS, X2_SPI_OP_NONE,
		       X2_SPI_OP_NONE);
	pm_runtime_force_suspend(&pdev->dev);
#ifdef CONFIG_X2_SPI_DMA_SINGLE
	x2_spi_release_dma(x2spi);
#endif
	return 0;
}

#ifdef CONFIG_PM_SLEEP
int x2_spi_suspend(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);
	struct x2_spi *x2spi = spi_controller_get_devdata(ctlr);

	dev_err(x2spi->dev, "enter suspend...\n");

	/* Transfer in process flag, wait SPI to be idle */
	x2_spi_wait_for_tpi(x2spi, 1);

	/* disable spi */
	x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_DIS, X2_SPI_OP_NONE,
			X2_SPI_OP_NONE);

	//disable clk to reduce power
	clk_disable_unprepare(x2spi->spi_mclk);

	return 0;
}

int x2_spi_resume(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);
	struct x2_spi *x2spi = spi_controller_get_devdata(ctlr);

	dev_err(x2spi->dev, "enter resume...\n");

	//enable clk to work
	clk_prepare_enable(x2spi->spi_mclk);

	x2_spi_init_hw(x2spi);

	return 0;
}
#endif

static const struct dev_pm_ops x2_spi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(x2_spi_suspend,
			x2_spi_resume)
};

static const struct of_device_id x2_spi_of_match[] = {
	{.compatible = "hobot,x2-spi"},
	{.compatible = "hobot,hobot-spi"},
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, x2_spi_of_match);

static struct platform_driver x2_spi_driver = {
    .probe = x2_spi_probe,
    .remove = x2_spi_remove,
    .driver = {
        .name = X2_SPI_NAME,
        .of_match_table = x2_spi_of_match,
		.pm = &x2_spi_dev_pm_ops,
    },
};

module_platform_driver(x2_spi_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("HOOT SPI driver");
MODULE_LICENSE("GPL v2");
