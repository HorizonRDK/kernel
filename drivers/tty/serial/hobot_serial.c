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

#if defined(CONFIG_SERIAL_HOBOT_UART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/suspend.h>
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
#include <linux/device.h>
#include <linux/preempt.h>
#include <linux/delay.h>
#include <soc/hobot/hobot_bus.h>
#endif
#ifdef CONFIG_HOBOT_DIAG
#include <soc/hobot/diag.h>
#endif
#include "hobot_serial.h"

#define HOBOT_UART_TTY_NAME	"ttyS"
#define HOBOT_UART_NAME		"hobot-uart"
#define HOBOT_UART_MAJOR		0	/* use dynamic node allocation */
#define HOBOT_UART_MINOR		0	/* works best with devtmpfs */
#define HOBOT_UART_NR_PORTS	CONFIG_SERIAL_HOBOT_NR_UARTS
#define HOBOT_UART_FIFO_SIZE	32	/* FIFO size */
#define HOBOT_UART_REGISTER_SPACE	0x1000
#define HOBOT_UART_OVERRUN_FIFO_SIZE 16
#define SUPPORT_UART_CONSOLE_TXCTRL

unsigned int early_console_baud;
/*log the error status of the previous interrupt on the UART IP*/
static int pre_errflag = 0;

/* Rx Trigger level */
static int rx_trigger_level = 4;
module_param(rx_trigger_level, uint, S_IRUGO);
MODULE_PARM_DESC(rx_trigger_level, "Rx trigger level, 1-16(uint 4 bytes)");

/* Tx Trigger level */
static int tx_trigger_level = 0;
module_param(tx_trigger_level, uint, S_IRUGO);
MODULE_PARM_DESC(tx_trigger_level, "Tx trigger level, 0-15 (uint: 4 bytes)");

#ifdef SUPPORT_UART_CONSOLE_TXCTRL
static int uarttx_ctrl = 1;
module_param(uarttx_ctrl, int, 0644);
MODULE_PARM_DESC(uarttx_ctrl, "uart Tx ctrl, (0:disable tx 1:enable tx)");

/*uart Tx ctrl character:ESC txd*/
static char txctrl_char[]="\etxd";
#endif

#ifdef CONFIG_HOBOT_TTY_POLL_MODE
#define HOBOT_UART_RX_POLL_TIME	50	/* Unit is ms */
#endif /* CONFIG_HOBOT_TTY_POLL_MODE */
#define TX_IN_PROGRESS_DMA     1
#define TX_IN_PROGRESS_INT     2

#ifdef CONFIG_HOBOT_SERIAL_DEBUGFS
/* for debugfs */
struct dentry *dentry_root;
struct dentry *dentry_data;
struct dentry *dentry_addr;
struct dentry *dentry_status;
static unsigned int dgb_tx_count;
static unsigned int dgb_tx_tail;
static unsigned int dgb_tx_head;
static unsigned int dgb_rx_count;
static unsigned int dgb_addr;
static unsigned char __iomem *dgb_membase;
#endif /* CONFIG_HOBOT_SERIAL_DEBUGFS */

/**
 * struct hobot_uart - device data
 * @port:		Pointer to the UART port
 * @baud:		Current baud rate
 */
struct hobot_uart {
	struct uart_port *port;
	unsigned int baud;
	char name[16];
	struct clk *uartclk;

#ifdef CONFIG_HOBOT_TTY_POLL_MODE
	struct timer_list rx_timer;
#endif				/* CONFIG_HOBOT_TTY_POLL_MODE */

#ifdef CONFIG_HOBOT_TTY_DMA_MODE
	dma_addr_t tx_dma_buf;	/* dma tx buffer bus address */
	unsigned int tx_bytes_requested;

	dma_addr_t rx_dma_buf;	/* dma rx buffer bus address */
	unsigned char *rx_buf;	/* dma rx buffer cpu address */
	unsigned int rx_off;	/* valid data head position */

	bool rx_enabled;
#endif
	int tx_in_progress;
	unsigned int poll_flag;
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	struct hobot_dpm uart_dpm;
	atomic_t dmatx_flag;
	atomic_t uart_start;
#endif
	int uart_id;
	u32 rx_bytes;
	u32 uartoutcnt;
};

enum uart_mode {
	UART_POLL_MODE,
	UART_IRQ_MODE,
	UART_DMA_MODE
};

#define to_hobot_uart(_nb) container_of(_nb, struct hobot_uart, \
		clk_rate_change_nb);

#ifdef HOBOT_UART_DBG
static unsigned int dbg_tx_cnt[1024];
static unsigned int dbg_tx_index = 0;
#endif /* HOBOT_UART_DBG */

#ifdef CONFIG_HOBOT_TTY_DMA_MODE

#define HOBOT_UART_DMA_SIZE	(UART_XMIT_SIZE * 2)
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
static int serial_dpm_callback(struct hobot_dpm *self,
				 unsigned long event, int state)
{
	struct hobot_uart *uart = container_of(self, struct hobot_uart, uart_dpm);
	struct tty_struct *tty = uart->port->state->port.tty;
	if (!atomic_read(&uart->uart_start))
		return 0;
	if (event == HB_BUS_SIGNAL_START) {
		if (!mutex_trylock(&tty->atomic_write_lock))
			return -EBUSY;
		/*check tx done*/
		if (atomic_read(&uart->dmatx_flag)) {
			mutex_unlock(&tty->atomic_write_lock);
			return -EBUSY;
		}
		disable_irq(uart->port->irq);
	} else if (event == HB_BUS_SIGNAL_END) {
			enable_irq(uart->port->irq);
			mutex_unlock(&tty->atomic_write_lock);
	}
	return 0;
}
#endif

static void check_switch_mode(struct uart_port *port, enum uart_mode mode)
{
	unsigned int val = 0;
	struct hobot_uart *hobot_uart = NULL;

	if (port == NULL) {
		pr_err("%s [%d]: uart port is NULL\n", __FILE__, __LINE__);
		return;
	}

	hobot_uart = (struct hobot_uart *)port->private_data;
	if (hobot_uart->poll_flag && (mode == UART_DMA_MODE)) {
		val = readl(port->membase + HOBOT_UART_FCR);
		val |= UART_FCR_RDMA_EN | UART_FCR_TDMA_EN;
		val |= UART_FCR_RFTRL(rx_trigger_level) | UART_FCR_TFTRL(tx_trigger_level);
		writel(val, port->membase + HOBOT_UART_FCR);
		hobot_uart->poll_flag = 0;
	} else if (!hobot_uart->poll_flag && (mode == UART_POLL_MODE)) {
		val = readl(port->membase + HOBOT_UART_FCR);
		val &= ~(UART_FCR_RDMA_EN | UART_FCR_TDMA_EN);
		writel(val, port->membase + HOBOT_UART_FCR);
		hobot_uart->poll_flag = 1;
	} else if (mode == UART_IRQ_MODE) {
		/*reserved for switch to irq mode, do nothing now*/
	}
}

static int hobot_uart_dma_alloc(struct uart_port *port)
{
	int ret = 0;
	struct hobot_uart *hobot_port = port->private_data;

	hobot_port->rx_buf = dma_alloc_coherent(port->dev,
						 HOBOT_UART_DMA_SIZE,
						 &hobot_port->rx_dma_buf, GFP_KERNEL);
	if (!hobot_port->rx_buf) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	hobot_port->rx_off = 0;
	hobot_port->rx_bytes = 0;

	hobot_port->tx_dma_buf = dma_map_single(port->dev,
						 port->state->xmit.buf,
						 UART_XMIT_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(port->dev, hobot_port->tx_dma_buf)) {
		dev_err(port->dev, "dma_map_single tx failed\n");
		ret = -ENOMEM;
		goto alloc_err;
	}

	memset(hobot_port->rx_buf, 0, HOBOT_UART_DMA_SIZE);

	return ret;

alloc_err:
	if (hobot_port->rx_buf) {
		dma_free_coherent(port->dev,
				  HOBOT_UART_DMA_SIZE, hobot_port->rx_buf,
				  hobot_port->rx_dma_buf);
	}

	return ret;
}

static void hobot_uart_dma_tx_start(struct uart_port *port)
{
	unsigned int count = 0;
	unsigned int val = 0;
	struct circ_buf *xmit = &port->state->xmit;
	struct hobot_uart *hobot_port = port->private_data;
	dma_addr_t tx_phys_addr;

	if (xmit->head >= xmit->tail) {
		count = CIRC_CNT(xmit->head, xmit->tail, UART_XMIT_SIZE);
	} else {
		count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
	}

	if (!count) {
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
		/*dma tx done*/
		atomic_set(&hobot_port->dmatx_flag, 0);
#endif
		return;
	}
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	atomic_set(&hobot_port->dmatx_flag, 1);
#endif
#ifdef CONFIG_HOBOT_SERIAL_DEBUGFS
		dgb_tx_count = port->icount.tx;
		dgb_tx_head = port->state->xmit.head;
		dgb_tx_tail = port->state->xmit.tail;
#endif /* CONFIG_HOBOT_SERIAL_DEBUGFS */

#ifdef HOBOT_UART_DBG
	dbg_tx_cnt[dbg_tx_index] = count;
	dbg_tx_index = (dbg_tx_index + 1) & (1024 - 1);
	dbg_tx_cnt[dbg_tx_index] = xmit->head;
	dbg_tx_index = (dbg_tx_index + 1) & (1024 - 1);
	dbg_tx_cnt[dbg_tx_index] = xmit->tail;
	dbg_tx_index = (dbg_tx_index + 1) & (1024 - 1);
#endif /* HOBOT_UART_DBG */
	/*dma tx start*/
	dma_sync_single_for_device(port->dev, hobot_port->tx_dma_buf,
				   UART_XMIT_SIZE, DMA_TO_DEVICE);
	tx_phys_addr = hobot_port->tx_dma_buf + xmit->tail;

	hobot_port->tx_bytes_requested = count;

	val = readl(port->membase + HOBOT_UART_FCR);
	val |= UART_FCR_TDMA_EN;
	writel(val, port->membase + HOBOT_UART_FCR);

	/*Set Transmit DMA size and Start DMA TX */
	writel(tx_phys_addr, port->membase + HOBOT_UART_TXADDR);
	writel(count, port->membase + HOBOT_UART_TXSIZE);
	val = readl(port->membase + HOBOT_UART_TXDMA);
	val &= 0xFFFFFF00;
	val |= (UART_TXLEN(3) | UART_TXSTA);
	writel(val, port->membase + HOBOT_UART_TXDMA);
	hobot_port->tx_in_progress = TX_IN_PROGRESS_DMA;

	return;
}

static void hobot_uart_dma_rx_start(struct uart_port *port)
{
	unsigned int val;
	struct hobot_uart *hobot_port = port->private_data;

	dma_sync_single_for_device(port->dev, hobot_port->rx_dma_buf,
				   HOBOT_UART_DMA_SIZE, DMA_TO_DEVICE);

	/*Set recvice DMA size and Start DMA RX */
	writel(hobot_port->rx_dma_buf, port->membase + HOBOT_UART_RXADDR);
	writel(HOBOT_UART_DMA_SIZE, port->membase + HOBOT_UART_RXSIZE);
	val = readl(port->membase + HOBOT_UART_RXDMA);
	val &= 0xFFFFFF00;
	val |= UART_RXSTA | UART_RXWRAP;
	val &= UART_DMA_RXTHD_CLEAR;
	val |= UART_RXTHD_1K;
	writel(val, port->membase + HOBOT_UART_RXDMA);

	hobot_port->rx_enabled = 1;

	return;
}

static void hobot_uart_dma_txdone(void *dev_id)
{
	unsigned int count = 0;
	struct uart_port *port = (struct uart_port *)dev_id;
	struct circ_buf *xmit = &port->state->xmit;
	struct hobot_uart *hobot_port = port->private_data;

	count = hobot_port->tx_bytes_requested;
	hobot_port->tx_bytes_requested = 0;
	xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
	port->icount.tx += count;
	hobot_port->tx_in_progress = 0;
#ifdef HOBOT_UART_DBG
	dbg_tx_cnt[dbg_tx_index] = xmit->tail;
	dbg_tx_index = (dbg_tx_index + 1) & (1024 - 1);

	dbg_tx_cnt[dbg_tx_index] = xmit->head;
	dbg_tx_index = (dbg_tx_index + 1) & (1024 - 1);
#endif /* HOBOT_UART_DBG */

	if (uart_circ_chars_pending(&port->state->xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(port);
	}

	hobot_uart_dma_tx_start(port);

	return;
}

static void hobot_uart_dma_rxdone(void *dev_id, unsigned int irqstatus)
{
	unsigned int rx_bytes;
	unsigned int count1, count2;
	unsigned int copied;
	struct uart_port *port = (struct uart_port *)dev_id;
	struct tty_port *tty_port = &port->state->port;
	struct hobot_uart *hobot_port = port->private_data;
	unsigned int val;
	char data;

#ifdef SUPPORT_UART_CONSOLE_TXCTRL
	static int input = 0;
	static int output = 0;
#endif

	while (irqstatus & (UART_RXTHD | UART_RXTO | UART_BI | UART_RXDON)) {
		if (irqstatus & UART_BI) {
			irqstatus &= ~UART_BI;
			port->icount.brk++;
			if (uart_handle_break(port))
				continue;
		}
		rx_bytes = readl(port->membase + HOBOT_UART_RXSIZE);
		/*
		 * there are three interrupts, timeout, threshold, and done
		 * if dma size is not changed, we need to clear current state
		 */
		if (hobot_port->rx_bytes == rx_bytes) {
			if (irqstatus & UART_RXTO) {
				irqstatus &= ~UART_RXTO;
				continue;
			}
			if (irqstatus & UART_RXTHD) {
				irqstatus &= ~UART_RXTHD;
				continue;
			}
			if (irqstatus & UART_RXDON) {
				irqstatus &= ~UART_RXDON;
				continue;
			}
		}
		hobot_port->rx_bytes = rx_bytes;
		dma_sync_single_for_cpu(port->dev, hobot_port->rx_dma_buf,
					HOBOT_UART_DMA_SIZE, DMA_FROM_DEVICE);
		data = *(char *)(hobot_port->rx_buf + hobot_port->rx_off);
		if (uart_handle_sysrq_char(port, data)) {
			hobot_port->rx_off++;
			irqstatus = 0;
			if (rx_bytes == hobot_port->rx_off) {
				hobot_port->rx_off &= (HOBOT_UART_DMA_SIZE - 1);
				continue;
			}
		}
		if (rx_bytes >= hobot_port->rx_off) {
			count1 = rx_bytes - hobot_port->rx_off;
			count2 = 0;
		} else {
			count1 = HOBOT_UART_DMA_SIZE - hobot_port->rx_off;
			count2 = rx_bytes;
		}

#ifdef SUPPORT_UART_CONSOLE_TXCTRL
		/*when the console is hobot-uart0, press ESC txd to disable tx*/
		if(!strcmp(hobot_port->name, "hobot-uart0")) {
			switch (input) {
				case 0:
					if (data == '\e') {
						input++;
					}
					break;
				case 1:
				case 2:
					if (data == txctrl_char[input]) {
						input++;
					} else {
						input = 0;
					}
					break;
				case 3:
					if (data == txctrl_char[input]) {
						uarttx_ctrl = 0;
						input = 0;
						output = 1;
					}
					break;
				default:
					input = 0;
			}

			/*when in tx disable state, press ESC to enable tx*/
			if(output == 1 || uarttx_ctrl == 0) {
				if (data == '\e') {
					uarttx_ctrl = 1;
					output = 0;
				}
			}
		}
#endif

#ifdef CONFIG_HOBOT_SERIAL_DEBUGFS
		dgb_rx_count = rx_bytes;
#endif /* CONFIG_HOBOT_SERIAL_DEBUGFS */

		copied = tty_insert_flip_string(tty_port,
						((unsigned char *)(hobot_port->rx_buf +
								   hobot_port->rx_off)),
						count1);
		if (copied != count1) {
			if(hobot_port->uartoutcnt == 100) {
				spin_unlock(&port->lock);
				dev_err(port->dev, "first, rxdata copy to tty layer failed\n");
				spin_lock(&port->lock);
				hobot_port->uartoutcnt = 0;
			}
			hobot_port->uartoutcnt++;
			port->icount.rx += copied;
		} else {
			port->icount.rx += count1;

			hobot_port->rx_off = (hobot_port->rx_off + count1) &
					  (HOBOT_UART_DMA_SIZE - 1);

			if (count2 > 0) {
				copied = tty_insert_flip_string(tty_port,
								((unsigned char
								  *)(hobot_port->rx_buf +
								     hobot_port->rx_off)),
								count2);
				if (copied != count2) {
					if(hobot_port->uartoutcnt == 100) {
						spin_unlock(&port->lock);
						dev_err(port->dev,
							"second, rxdata copy to tty layer failed\n");
						spin_lock(&port->lock);
						hobot_port->uartoutcnt = 0;
					}
					hobot_port->uartoutcnt++;
					port->icount.rx += copied;
				} else {
					port->icount.rx += count2;

					hobot_port->rx_off = (hobot_port->rx_off + count2) &
							  (HOBOT_UART_DMA_SIZE - 1);
				}
			}
		}

		dma_sync_single_for_device(port->dev, hobot_port->rx_dma_buf,
					   HOBOT_UART_DMA_SIZE, DMA_TO_DEVICE);
		spin_unlock(&port->lock);
		tty_flip_buffer_push(&port->state->port);
		spin_lock(&port->lock);
		irqstatus = 0;
	}
	val = readl(port->membase + HOBOT_UART_RXDMA);
	val &= ~UART_RXSTA;
	val |= UART_RXSTA;
	writel(val, port->membase + HOBOT_UART_RXDMA);
	return;
}

#endif /* CONFIG_HOBOT_TTY_DMA_MODE */

#ifdef CONFIG_HOBOT_TTY_IRQ_MODE
/**
 * hobot_uart_handle_rx - Handle the received bytes along with Rx errors.
 * @dev_id: Id of the UART port
 * @irqstatus: The interrupt status register value as read
 * Return: None
 */
static void hobot_uart_handle_rx(void *dev_id, unsigned int irqstatus)
{
	struct uart_port *port = (struct uart_port *)dev_id;
	unsigned int data;
	unsigned int size;
	char status = TTY_NORMAL;

	while (irqstatus) {
		data = readl(port->membase + HOBOT_UART_RDR);
		port->icount.rx++;

#ifdef CONFIG_HOBOT_SERIAL_DEBUGFS
	dgb_rx_count = port->icount.rx;
#endif /* CONFIG_HOBOT_SERIAL_DEBUGFS */

		if (irqstatus & UART_BI) {
			port->icount.brk++;
			status = TTY_BREAK;
			if (uart_handle_break(port))
				continue;
		}

		irqstatus &= port->read_status_mask;
		irqstatus &= ~port->ignore_status_mask;

		if (uart_handle_sysrq_char(port, data))
			continue;

		if (irqstatus & UART_PE) {
			port->icount.parity++;
			status = TTY_PARITY;
		}
		if (irqstatus & UART_FE) {
			port->icount.frame++;
			status = TTY_FRAME;
		}
		if (irqstatus & UART_RXOE) {
			/* Clear overrun fifo */
			for (size = 0; size < HOBOT_UART_OVERRUN_FIFO_SIZE; size++)
				readl(port->membase + HOBOT_UART_RDR);

			port->icount.overrun++;
			tty_insert_flip_char(&port->state->port, 0,
						 TTY_OVERRUN);
		}
		tty_insert_flip_char(&port->state->port, data, status);
		irqstatus = 0;
	}
	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
	spin_lock(&port->lock);
}
#endif /* CONFIG_HOBOT_TTY_IRQ_MODE */

/**
 * hobot_uart_stop_rx - Stop RX
 * @port: Handle to the uart port structure
 */
static void hobot_uart_stop_rx(struct uart_port *port)
{
	unsigned int regval;
	int status;

	/* Disable the receiver */
	regval = readl(port->membase + HOBOT_UART_ENR);
	regval &= ~UART_ENR_RX_EN;
	writel(regval, port->membase + HOBOT_UART_ENR);

	/*stop RXDMA*/
	regval = readl(port->membase + HOBOT_UART_RXDMA);
	regval |= UART_RXSTP;
	writel(regval, port->membase + HOBOT_UART_RXDMA);

	/* Reset FIFO */
	status = readl(port->membase + HOBOT_UART_FCR);
	status |= UART_FCR_RFRST | UART_FCR_TFRST;
	writel(status, port->membase + HOBOT_UART_FCR);
}

#if defined(CONFIG_HOBOT_TTY_IRQ_MODE) || defined(CONFIG_HOBOT_TTY_POLL_MODE)\
	|| !defined(CONFIG_HOBOT_TTY_DMA_MODE)
/**
 * hobot_uart_handle_tx - Handle the bytes to be Txed.
 * @dev_id: Id of the UART port
 * Return: None
 */
static void hobot_uart_handle_tx(void *dev_id, unsigned char in_irq)
{
	struct uart_port *port = (struct uart_port *)dev_id;

	if (uart_circ_empty(&port->state->xmit)) {
		return;
	}

	do {
		while (!
			   (readl(port->membase + HOBOT_UART_LSR) &
			UART_LSR_TX_EMPTY)) {
		}
		/*
		 * Get the data from the UART circular buffer
		 * and write it to the cdns_uart's TX_FIFO
		 * register.
		 */
		writel(port->state->xmit.buf[port->state->xmit.tail],
			   port->membase + HOBOT_UART_TDR);

		port->icount.tx++;

		/*
		 * Adjust the tail of the UART buffer and wrap
		 * the buffer if it reaches limit.
		 */
		port->state->xmit.tail =
			(port->state->xmit.tail + 1) & (UART_XMIT_SIZE - 1);

#ifdef CONFIG_HOBOT_SERIAL_DEBUGFS
		dgb_tx_count = port->icount.tx;
		dgb_tx_head = port->state->xmit.head;
		dgb_tx_tail = port->state->xmit.tail;
#endif /* CONFIG_HOBOT_SERIAL_DEBUGFS */

		/*
		 * Hardware needs to transmit a char to make the interrupt of tx.
		 */
#ifdef CONFIG_HOBOT_TTY_IRQ_MODE
		if (!in_irq) {
			return;
		}
#endif /* CONFIG_HOBOT_TTY_IRQ_MODE */
	} while (!uart_circ_empty(&port->state->xmit));

	if (uart_circ_chars_pending(&port->state->xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	return;
}
#endif /* CONFIG_HOBOT_TTY_IRQ_MODE || CONFIG_HOBOT_TTY_POLL_MODE */

#ifdef CONFIG_HOBOT_DIAG
static void uart_diag_report(uint8_t errsta, uint32_t srcpndreg,
						struct hobot_uart *hbuart)
{
	uint8_t env_data[8];
	env_data[0] = hbuart->uart_id;
	env_data[1] = 0xff;
	env_data[2] = 0;
	env_data[3] = sizeof(uint32_t);
	env_data[4] = srcpndreg & 0xff;
	env_data[5] = (srcpndreg >> 8) & 0xff;
	env_data[6] = (srcpndreg >> 16) & 0xff;
	env_data[7] = (srcpndreg >> 24) & 0xff;
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_uart,
				EventIdUart0Err + hbuart->uart_id,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&env_data,
				sizeof(uint8_t) * 8);
	} else {
		diag_send_event_stat(
			DiagMsgPrioHigh,
			ModuleDiag_uart,
			EventIdUart0Err + hbuart->uart_id,
			DiagEventStaSuccess);
	}
}
#endif

/**
 * hobot_uart_isr - Interrupt handler
 * @irq: Irq number
 * @dev_id: Id of the port
 *
 * Return: IRQHANDLED
 */
static irqreturn_t hobot_uart_isr(int irq, void *dev_id)
{
#ifdef CONFIG_HOBOT_TTY_IRQ_MODE
	struct uart_port *port = (struct uart_port *)dev_id;
	unsigned int status;
	struct hobot_uart *hobot_uart = port->private_data;
	u8 errflag = 0;

	spin_lock(&port->lock);
	status = readl(port->membase + HOBOT_UART_SRC_PND);

	if(status & (UART_RXOE | UART_PE | UART_FE))
		errflag = 1;

	/* Disable the special irq */
	writel(status, port->membase + HOBOT_UART_INT_SETMASK);
	/* Clear irq's status */
	writel(status, port->membase + HOBOT_UART_SRC_PND);

	if (status & (UART_RXFUL | UART_RXOE | UART_BI | UART_PE | UART_FE))
		hobot_uart_handle_rx(dev_id, status);

	if (status & UART_TXEPT) {
		hobot_uart_handle_tx(dev_id, 1);
	}

	writel(status, port->membase + HOBOT_UART_INT_UNMASK);

	spin_unlock(&port->lock);
#elif defined(CONFIG_HOBOT_TTY_DMA_MODE)
	struct uart_port *port = (struct uart_port *)dev_id;
	unsigned int status;
	struct hobot_uart *hobot_uart = port->private_data;
	u8 errflag = 0;

	check_switch_mode(port, UART_DMA_MODE);
	spin_lock(&port->lock);
	status = readl(port->membase + HOBOT_UART_SRC_PND);

	if(status & (UART_RXOE | UART_PE | UART_FE))
		errflag = 1;

	/* Disable the special irq */
	writel(status, port->membase + HOBOT_UART_INT_SETMASK);
	/* Clear irq's status */
	writel(status, port->membase + HOBOT_UART_SRC_PND);
	if (status & (UART_RXTO | UART_RXTHD | UART_BI | UART_RXDON)) {
		hobot_uart_dma_rxdone(dev_id, status);
	}

	if (status & UART_TXDON) {
		hobot_uart_dma_txdone(dev_id);
	}

	status &= ~UART_TXEPT;
	writel(status, port->membase + HOBOT_UART_INT_UNMASK);

	spin_unlock(&port->lock);
#endif /* CONFIG_HOBOT_TTY_IRQ_MODE */

#ifdef CONFIG_HOBOT_DIAG
#if IS_ENABLED(CONFIG_HOBOT_DIAG_INJECT)
	diag_inject_val(ModuleDiag_uart, EventIdAny, &status);
	if (status & (UART_RXOE | UART_PE | UART_FE))
		errflag = 1;
#endif
	if (errflag)
		uart_diag_report(1, status, hobot_uart);
	if (pre_errflag == 1 && errflag == 0)
		uart_diag_report(0, status, hobot_uart);
	pre_errflag = errflag;
#endif

	return IRQ_HANDLED;
}

/**
 * hobot_uart_calc_baud_divs - Calculate baud rate divisors
 * @clk: UART module input clock
 * @baud: Desired baud rate
 * @rbdiv: BDIV value (return value)
 * @rcd: CD value (return value)
 * @div8: Value for clk_sel bit in mod (return value)
 * Return: baud rate, requested baud when possible, or actual baud when there
 *	was too much error, zero if no valid divisors are found.
 */
static unsigned int hobot_uart_calc_baud_divs(unsigned int clk,
					   unsigned int baud, u32 * br_int,
					   u32 * br_frac)
{
	int err = ~0;

	if (br_int == NULL || br_frac == NULL)
		return err;

    if (clk <= 0 || baud <= 0)
		return err;

#ifdef CONFIG_UART_LOW_SPEED_MODE
	*br_int = (clk / (baud * LOW_SPEED_MODE_DIV));
	*br_frac =
		(clk % (baud * LOW_SPEED_MODE_DIV)) * 1024 / (baud *
							  LOW_SPEED_MODE_DIV);
#elif CONFIG_UART_MID_SPEED_MODE
	*br_int = (clk / (baud * MID_SPEED_MODE_DIV));
	*br_frac =
		(clk % (baud * MID_SPEED_MODE_DIV)) * 1024 / (baud *
							  MID_SPEED_MODE_DIV);
#elif CONFIG_UART_HIGH_SPEED_MODE
	*br_int = (clk / (baud * HIGH_SPEED_MODE_DIV));
	*br_frac =
		(clk % (baud * HIGH_SPEED_MODE_DIV)) * 1024 / (baud *
							   HIGH_SPEED_MODE_DIV);
#endif
	return baud;
}

/**
 * hobot_uart_set_baud_rate - Calculate and set the baud rate
 * @port: Handle to the uart port structure
 * @baud: Baud rate to set
 * Return: baud rate, requested baud when possible, or actual baud when there
 *	   was too much error, zero if no valid divisors are found.
 */
static unsigned int hobot_uart_set_baud_rate(struct uart_port *port,
					  unsigned int baud)
{
	unsigned int calc_baud;
	u32 bdiv_int = 0, bdiv_frac = 0;
	u32 bcr_reg = 0;
	struct hobot_uart *hobot_uart = port->private_data;

	calc_baud =
		hobot_uart_calc_baud_divs(port->uartclk, baud, &bdiv_int, &bdiv_frac);

#ifdef CONFIG_UART_LOW_SPEED_MODE
	bcr_reg |= UART_BCR_BRDIV_MODE(0);
#elif CONFIG_UART_MID_SPEED_MODE
	bcr_reg |= UART_BCR_BRDIV_MODE(1);
#elif CONFIG_UART_HIGH_SPEED_MODE
	bcr_reg |= UART_BCR_BRDIV_MODE(2);
#endif

	bcr_reg |=
		(UART_BCR_BRDIV_INT(bdiv_int) | UART_BCR_BRDIV_FRAC(bdiv_frac));
	/* Write new divisors to hardware */
	writel(bcr_reg, port->membase + HOBOT_UART_BCR);

	hobot_uart->baud = baud;
	return calc_baud;
}

/**
 * hobot_uart_start_tx -  Start transmitting bytes
 * @port: Handle to the uart port structure
 */
static void hobot_uart_start_tx(struct uart_port *port)
{
	unsigned int ctrl;
	unsigned int val;
	unsigned int mask = 0;
	struct hobot_uart *hobot_port = port->private_data;

	/*
	 *when uarttx_ctrl=0 and the console is hobot-uart0,disable start_tx
	 *uart_circ_clear the xmit to prevent the xmit from being full
	*/
	if(!uarttx_ctrl && !strcmp(hobot_port->name, "hobot-uart0")) {
		uart_circ_clear(&port->state->xmit);
		return;
	}
	if (uart_tx_stopped(port) || uart_circ_empty(&port->state->xmit)) {
		return;
	}
#ifdef CONFIG_HOBOT_TTY_DMA_MODE
	val = readl(port->membase + HOBOT_UART_TXDMA);
	if (val & UART_TXSTA) {
		return;
	}
#endif /* CONFIG_HOBOT_TTY_DMA_MODE */

#ifdef CONFIG_HOBOT_TTY_IRQ_MODE
	mask = UART_TXEPT;
#elif defined CONFIG_HOBOT_TTY_DMA_MODE
	mask = UART_TXTHD | UART_TXDON;
	check_switch_mode(port, UART_DMA_MODE);

#endif /* CONFIG_HOBOT_TTY_IRQ_MODE */
	writel(mask, port->membase + HOBOT_UART_INT_UNMASK);

	ctrl = readl(port->membase + HOBOT_UART_ENR);
	ctrl |= UART_ENR_TX_EN;
	writel(ctrl, port->membase + HOBOT_UART_ENR);

#ifdef CONFIG_HOBOT_TTY_DMA_MODE
	if (!hobot_port->tx_in_progress)
		hobot_uart_dma_tx_start(port);
#else
	hobot_uart_handle_tx(port, 0);
#endif /* CONFIG_HOBOT_TTY_DMA_MODE */
}

/**
 * hobot_uart_stop_tx - Stop TX
 * @port: Handle to the uart port structure
 */
static void hobot_uart_stop_tx(struct uart_port *port)
{
	unsigned int ctrl;
	int status;
	unsigned int regval;
	struct hobot_uart *hobot_port = port->private_data;

#if defined(CONFIG_HOBOT_TTY_IRQ_MODE) || defined(CONFIG_HOBOT_TTY_DMA_MODE)
	writel(UART_TXEPT | UART_TXTHD | UART_TXDON,
		   port->membase + HOBOT_UART_INT_SETMASK);
#endif /* CONFIG_HOBOT_TTY_IRQ_MODE */

	/* Disable the transmitter */
	ctrl = readl(port->membase + HOBOT_UART_ENR);
	ctrl &= ~UART_ENR_TX_EN;
	writel(ctrl, port->membase + HOBOT_UART_ENR);
	hobot_port->tx_in_progress = 0;

	/*stop TXDMA*/
	regval = readl(port->membase + HOBOT_UART_TXDMA);
	regval |= UART_TXSTP;
	writel(regval, port->membase + HOBOT_UART_TXDMA);

	/* Reset FIFO */
	status = readl(port->membase + HOBOT_UART_FCR);
	status |= UART_FCR_RFRST | UART_FCR_TFRST;
	writel(status, port->membase + HOBOT_UART_FCR);
}

/**
 * hobot_uart_tx_empty -  Check whether TX is empty
 * @port: Handle to the uart port structure
 *
 * Return: TIOCSER_TEMT on success, 0 otherwise
 */
static unsigned int hobot_uart_tx_empty(struct uart_port *port)
{
	unsigned int status;

	status = readl(port->membase + HOBOT_UART_LSR) & UART_LSR_TX_EMPTY;
	return status ? TIOCSER_TEMT : 0;
}

/**
 * hobot_uart_break_ctl - Based on the input ctl we have to start or stop
 *			transmitting char breaks
 * @port: Handle to the uart port structure
 * @ctl: Value based on which start or stop decision is taken
 */
static void hobot_uart_break_ctl(struct uart_port *port, int ctl)
{
	/* Nothing to do */
}

/**
 * hobot_uart_set_termios - termios operations, handling data length, parity,
 *				stop bits, flow control, baud rate
 * @port: Handle to the uart port structure
 * @termios: Handle to the input termios structure
 * @old: Values of the previously saved termios structure
 */
static void hobot_uart_set_termios(struct uart_port *port,
				struct ktermios *termios, struct ktermios *old)
{
	uint32_t ctrl_reg, lcr_reg;
	uint32_t baud, minbaud, maxbaud;
	unsigned long flags;
	struct hobot_uart *hobot_uart = port->private_data;

	spin_lock_irqsave(&port->lock, flags);

	/* Wait for the transmit FIFO to empty before making changes */
	if ((readl(port->membase + HOBOT_UART_ENR) & UART_ENR_TX_EN)) {
		while (!(readl(port->membase + HOBOT_UART_LSR) &
			 UART_LSR_TX_EMPTY)) {
			cpu_relax();
		}
	}

	/* Disable the TX and RX to set baud rate */
	ctrl_reg = readl(port->membase + HOBOT_UART_ENR);
	ctrl_reg &= ~(UART_ENR_TX_EN | UART_ENR_RX_EN);
	writel(ctrl_reg, port->membase + HOBOT_UART_ENR);

#ifdef CONFIG_UART_LOW_SPEED_MODE
	minbaud =
		port->uartclk / (UART_BCR_BRDIV_INT_MASK * LOW_SPEED_MODE_DIV);
	maxbaud = port->uartclk / LOW_SPEED_MODE_DIV;
#elif CONFIG_UART_MID_SPEED_MODE
	minbaud =
		port->uartclk / (UART_BCR_BRDIV_INT_MASK * MID_SPEED_MODE_DIV);
	maxbaud = port->uartclk / MID_SPEED_MODE_DIV;
#elif CONFIG_UART_HIGH_SPEED_MODE
	minbaud =
		port->uartclk / (UART_BCR_BRDIV_INT_MASK * HIGH_SPEED_MODE_DIV);
	maxbaud = port->uartclk / HIGH_SPEED_MODE_DIV;
#endif

	baud = uart_get_baud_rate(port, termios, old, minbaud, maxbaud);
	baud = hobot_uart_set_baud_rate(port, baud);
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);

	/* Set TX/RX Reset */
	ctrl_reg = readl(port->membase + HOBOT_UART_FCR);
	ctrl_reg |= UART_FCR_RFRST | UART_FCR_TFRST;
	writel(ctrl_reg, port->membase + HOBOT_UART_FCR);

	while (readl(port->membase + HOBOT_UART_FCR) &
		   (UART_FCR_RFRST | UART_FCR_TFRST))
		cpu_relax();

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Set the TX enable bit and RX enable bit
	 * to enable the transmitter and receiver.
	 */
	ctrl_reg = readl(port->membase + HOBOT_UART_ENR);
	ctrl_reg |= UART_ENR_TX_EN | UART_ENR_RX_EN;
	writel(ctrl_reg, port->membase + HOBOT_UART_ENR);

	port->read_status_mask = UART_TXEPT | UART_RXOE | UART_RXTO | UART_RXOE;
	port->ignore_status_mask = 0;

	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_PE | UART_FE;

	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_PE | UART_FE | UART_RXOE;

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_RXFUL | UART_PE | UART_FE;

#ifdef CONFIG_HOBOT_TTY_DMA_MODE
	hobot_uart->rx_off = 0;
	hobot_uart->rx_bytes = 0;
	if (hobot_uart->rx_enabled) {
		ctrl_reg = readl(port->membase + HOBOT_UART_RXDMA);
		ctrl_reg |= UART_RXSTA;
		writel(ctrl_reg, port->membase + HOBOT_UART_RXDMA);
	}
#endif /* CONFIG_HOBOT_TTY_DMA_MODE */

	lcr_reg = readl(port->membase + HOBOT_UART_LCR);

	/* Handling Data Size */
	switch (termios->c_cflag & CSIZE) {
	case CS7:
		lcr_reg &= (~UART_LCR_8_BIT);
		break;
	default:
	case CS8:
		lcr_reg |= UART_LCR_8_BIT;
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= CS8;
		break;
	}

	/* Handling Parity and Stop Bits length */
	if (termios->c_cflag & CSTOPB)
		lcr_reg |= UART_LCR_2_STOP;	/* 2 STOP bits */
	else
		lcr_reg &= (~UART_LCR_2_STOP);	/* 1 STOP bit */

	/*
	 * stick  even_parity  parity_en  parity
	 *	 -		  -			  -			no
	 *	 0		  0			  1			odd
	 *	 0		  1			  1			even
	 *	 1		  0			  1			mark
	 *	 1		  1			  1			space
	 */
	if (termios->c_cflag & PARENB) {
		lcr_reg |= UART_LCR_PEN;

		/* Odd or Even parity */
		if (termios->c_cflag & PARODD) {
			lcr_reg &= ~UART_LCR_EPS;
		} else {
			lcr_reg |= UART_LCR_EPS;
		}
		/* Mark or Space parity */
		if (termios->c_cflag & CMSPAR) {
			lcr_reg |= UART_LCR_SP;
		} else {
			lcr_reg &= ~UART_LCR_SP;
		}
	} else {
		lcr_reg &= ~UART_LCR_PEN;
	}

	/* flow control */
	if (termios->c_cflag & CRTSCTS)
		lcr_reg |= UART_LCR_RTS_EN | UART_LCR_CTS_EN;
	else
		lcr_reg &= (~UART_LCR_RTS_EN) & (~UART_LCR_CTS_EN);

	lcr_reg &= UART_LCR_TOI_CLEAR;
	if (baud > HB_BAUD_921600) {
		lcr_reg |= UART_LCR_TOI_256_FRAME;
	} else {
		lcr_reg |= UART_LCR_TOI_32_FRAME;
	}
	writel(lcr_reg, port->membase + HOBOT_UART_LCR);

	spin_unlock_irqrestore(&port->lock, flags);
}

#ifdef CONFIG_HOBOT_TTY_POLL_MODE
static void hobot_uart_rx_polling_func(unsigned long data)
{
	struct uart_port *port = (struct uart_port *)data;
	struct hobot_uart *hobot_uart = port->private_data;
	unsigned char val;
	char status = TTY_NORMAL;

	if (!(readl(port->membase + HOBOT_UART_LSR) & UART_LSR_RXRDY)) {
		mod_timer(&hobot_uart->rx_timer,
			  jiffies + msecs_to_jiffies(HOBOT_UART_RX_POLL_TIME));
		return;
	}

	while ((readl(port->membase + HOBOT_UART_LSR)) & UART_LSR_RXRDY) {
		val = readb(port->membase + HOBOT_UART_RDR);
		port->icount.rx++;
		tty_insert_flip_char(&port->state->port, val, status);
	}

	tty_flip_buffer_push(&port->state->port);

	mod_timer(&hobot_uart->rx_timer,
		  jiffies + msecs_to_jiffies(HOBOT_UART_RX_POLL_TIME));

	return;
}
#endif /* CONFIG_HOBOT_TTY_POLL_MODE */

/**
 * hobot_uart_startup - Called when an application opens a hobot_uart port
 * @port: Handle to the uart port structure
 *
 * Return: 0 on success, negative errno otherwise
 */
static int hobot_uart_startup(struct uart_port *port)
{
	int ret;
	unsigned long flags;
	unsigned int val = 0;
	unsigned int mask;
	struct hobot_uart *hobot_uart = port->private_data;


#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	atomic_set(&hobot_uart->uart_start, 1);
#endif
#ifdef CONFIG_HOBOT_SERIAL_DEBUGFS
	dgb_membase = port->membase;
#endif /* CONFIG_HOBOT_SERIAL_DEBUGFS */

	spin_lock_irqsave(&port->lock, flags);

	/* First Disable Uart */
	val = readl(port->membase + HOBOT_UART_ENR);
	val &= ~UART_ENR_EN;
	writel(val, port->membase + HOBOT_UART_ENR);
	/* Reset TX/RX FIFO */
	val = readl(port->membase + HOBOT_UART_FCR);
	val |= UART_FCR_RFRST | UART_FCR_TFRST;
	writel(val, port->membase + HOBOT_UART_FCR);

	/* Wait hardware auto clear */
	while (readl(port->membase + HOBOT_UART_FCR) &
		   (UART_FCR_RFRST | UART_FCR_TFRST))
		cpu_relax();

	/* Set TX/RX FIFO Trigger level and disable dma tx/rx */
	val = readl(port->membase + HOBOT_UART_FCR);
#ifdef CONFIG_HOBOT_TTY_DMA_MODE
	/* Only enable rx dma */
	val |= UART_FCR_RDMA_EN;
#else
	val &= ~(UART_FCR_RDMA_EN | UART_FCR_TDMA_EN);
#endif /* CONFIG_HOBOT_TTY_DMA_MODE */
	val |=
		UART_FCR_RFTRL(rx_trigger_level) | UART_FCR_TFTRL(tx_trigger_level);
	writel(val, port->membase + HOBOT_UART_FCR);

	val = readl(port->membase + HOBOT_UART_LCR);
	val &= (~0x0F00);
	val |= 5 << 8;
	writel(val, port->membase + HOBOT_UART_LCR);

	/* Clear all pending Interrupt */
	writel(0x7FFF, port->membase + HOBOT_UART_SRC_PND);

#if defined(CONFIG_HOBOT_TTY_IRQ_MODE) || defined(CONFIG_HOBOT_TTY_DMA_MODE)
	mask = UART_TXEPT | UART_TXTHD | UART_TXDON;
	writel(mask, port->membase + HOBOT_UART_INT_SETMASK);

	mask = UART_RXTO | UART_RXOE | UART_BI | UART_RXTHD |
		UART_FE | UART_PE | UART_CTSC | UART_RXDON | UART_RXFUL;
	writel(mask, port->membase + HOBOT_UART_INT_UNMASK);
#else
	writel(UART_IRQ_SRC_MASK, port->membase + HOBOT_UART_INT_SETMASK);
#endif /* #if 0 */

	/* Enable  global uart */
	val = readl(port->membase + HOBOT_UART_ENR);
	val |= (UART_ENR_EN | UART_ENR_RX_EN);
	writel(val, port->membase + HOBOT_UART_ENR);

	spin_unlock_irqrestore(&port->lock, flags);

#ifdef CONFIG_HOBOT_TTY_DMA_MODE
	ret = hobot_uart_dma_alloc(port);
	if (ret < 0) {
		dev_err(port->dev, "could not allocate dma buffers!\n");
		return ret;
	}

	hobot_uart_dma_rx_start(port);
#endif /* CONFIG_HOBOT_TTY_DMA_MODE */

	ret =
		request_irq(port->irq, hobot_uart_isr, IRQF_TRIGGER_HIGH,
			hobot_uart->name, port);
	if (ret) {
		dev_err(port->dev, "request_irq '%d' failed with %d\n",
			port->irq, ret);
		return ret;
	}
#ifdef CONFIG_HOBOT_TTY_POLL_MODE
	setup_timer(&hobot_uart->rx_timer, hobot_uart_rx_polling_func,
			(unsigned long)port);
	mod_timer(&hobot_uart->rx_timer,
		  jiffies + msecs_to_jiffies(HOBOT_UART_RX_POLL_TIME));
#endif

	return 0;
}

/**
 * hobot_uart_shutdown - Called when an application closes a hobot_uart port
 * @port: Handle to the uart port structure
 */
static void hobot_uart_shutdown(struct uart_port *port)
{
	int status;
	unsigned long flags;

	struct hobot_uart *hobot_uart = port->private_data;
#ifdef CONFIG_HOBOT_TTY_POLL_MODE
	del_timer(&hobot_uart->rx_timer);
#endif /* CONFIG_HOBOT_TTY_POLL_MODE */

#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	atomic_set(&hobot_uart->uart_start, 0);
#endif
	spin_lock_irqsave(&port->lock, flags);

	/* Disable interrupts */
	status = readl(port->membase + HOBOT_UART_INT_MASK);
	writel(status, port->membase + HOBOT_UART_INT_SETMASK);
	writel(0x7FFF, port->membase + HOBOT_UART_SRC_PND);

	/* Reset FIFO */
	status = readl(port->membase + HOBOT_UART_FCR);
	status |= UART_FCR_RFRST | UART_FCR_TFRST;
	writel(status, port->membase + HOBOT_UART_FCR);

	/* Disable the TX and RX */
	status = readl(port->membase + HOBOT_UART_ENR);
	status &= ~(UART_ENR_EN | UART_ENR_RX_EN | UART_ENR_TX_EN);
	writel(status, port->membase + HOBOT_UART_ENR);

	spin_unlock_irqrestore(&port->lock, flags);

#ifdef CONFIG_HOBOT_TTY_DMA_MODE
	dma_free_coherent(port->dev, HOBOT_UART_DMA_SIZE,
			(void *)hobot_uart->rx_buf, hobot_uart->rx_dma_buf);
	dma_unmap_single(port->dev, hobot_uart->tx_dma_buf,
			UART_XMIT_SIZE, DMA_TO_DEVICE);
	hobot_uart->rx_off = 0;
	hobot_uart->rx_bytes = 0;
#endif
	free_irq(port->irq, port);
	hobot_uart->tx_in_progress = 0;
}

/**
 * hobot_uart_type - Set UART type to hobot_uart port
 * @port: Handle to the uart port structure
 *
 * Return: string on success, NULL otherwise
 */
static const char *hobot_uart_type(struct uart_port *port)
{
	return port->type == PORT_HOBOT_UART ? HOBOT_UART_NAME : NULL;
}

/**
 * hobot_uart_verify_port - Verify the port params
 * @port: Handle to the uart port structure
 * @ser: Handle to the structure whose members are compared
 *
 * Return: 0 on success, negative errno otherwise.
 */
static int hobot_uart_verify_port(struct uart_port *port,
				   struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_HOBOT_UART)
		return -EINVAL;
	if (port->irq != ser->irq)
		return -EINVAL;
	if (ser->io_type != UPIO_MEM)
		return -EINVAL;
	if (port->iobase != ser->port)
		return -EINVAL;
	if (ser->hub6 != 0)
		return -EINVAL;
	return 0;
}

/**
 * hobot_uart_request_port - Claim the memory region attached to hobot_uart port,
 *				called when the driver adds a hobot_uart port via
 *				uart_add_one_port()
 * @port: Handle to the uart port structure
 *
 * Return: 0 on success, negative errno otherwise.
 */
static int hobot_uart_request_port(struct uart_port *port)
{
	if (!request_mem_region(port->mapbase, HOBOT_UART_REGISTER_SPACE,
				HOBOT_UART_NAME)) {
		return -ENOMEM;
	}

	port->membase = ioremap(port->mapbase, HOBOT_UART_REGISTER_SPACE);
	if (!port->membase) {
		dev_err(port->dev, "Unable to map registers\n");
		release_mem_region(port->mapbase, HOBOT_UART_REGISTER_SPACE);
		return -ENOMEM;
	}
	return 0;
}

/**
 * hobot_uart_release_port - Release UART port
 * @port: Handle to the uart port structure
 *
 * Release the memory region attached to a hobot_uart port. Called when the
 * driver removes a hobot_uart port via uart_remove_one_port().
 */
static void hobot_uart_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, HOBOT_UART_REGISTER_SPACE);
	iounmap(port->membase);
	port->membase = NULL;
}

/**
 * hobot_uart_config_port - Configure UART port
 * @port: Handle to the uart port structure
 * @flags: If any
 */
static void hobot_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE && hobot_uart_request_port(port) == 0)
		port->type = PORT_HOBOT_UART;
}

/**
 * hobot_uart_get_mctrl - Get the modem control state
 * @port: Handle to the uart port structure
 *
 * Return: the modem control state
 */
static unsigned int hobot_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void hobot_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

#ifdef CONFIG_CONSOLE_POLL
static int hobot_uart_poll_get_char(struct uart_port *port)
{
	int c;

#ifdef CONFIG_HOBOT_TTY_DMA_MODE
	check_switch_mode(port, UART_POLL_MODE);
#endif
	/* Check if FIFO is empty */
	if (!(readl(port->membase + HOBOT_UART_LSR) & UART_LSR_RXRDY))
		c = NO_POLL_CHAR;
	else			/* Read a character */
		c = (unsigned char)readl(port->membase + HOBOT_UART_RDR);

	return c;
}

static void hobot_uart_poll_put_char(struct uart_port *port, unsigned char c)
{
	/* Wait until FIFO is empty */
	while (!(readl(port->membase + HOBOT_UART_LSR) & UART_LSR_TX_EMPTY))
		cpu_relax();

	/* Write a character */
	writel(c, port->membase + HOBOT_UART_TDR);


	return;
}
#endif

static void hobot_uart_pm(struct uart_port *port, unsigned int state,
			   unsigned int oldstate)
{
	switch (state) {
	case UART_PM_STATE_OFF:
		pm_runtime_mark_last_busy(port->dev);
		pm_runtime_put_autosuspend(port->dev);
		break;
	default:
		pm_runtime_get_sync(port->dev);
		break;
	}
}

static void hobot_flush_buffer(struct uart_port *port)
{
	struct hobot_uart *hobot_port = port->private_data;

#ifdef CONFIG_HOBOT_TTY_DMA_MODE
	hobot_port->tx_bytes_requested = 0;
#endif
}

static const struct uart_ops hobot_uart_ops = {
	.set_mctrl = hobot_uart_set_mctrl,
	.get_mctrl = hobot_uart_get_mctrl,
	.start_tx = hobot_uart_start_tx,
	.stop_tx = hobot_uart_stop_tx,
	.stop_rx = hobot_uart_stop_rx,
	.tx_empty = hobot_uart_tx_empty,
	.break_ctl = hobot_uart_break_ctl,
	.set_termios = hobot_uart_set_termios,
	.startup = hobot_uart_startup,
	.shutdown = hobot_uart_shutdown,
	.pm = hobot_uart_pm,
	.type = hobot_uart_type,
	.verify_port = hobot_uart_verify_port,
	.request_port = hobot_uart_request_port,
	.release_port = hobot_uart_release_port,
	.config_port = hobot_uart_config_port,
	.flush_buffer   = hobot_flush_buffer,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = hobot_uart_poll_get_char,
	.poll_put_char = hobot_uart_poll_put_char,
#endif
};

static struct uart_port hobot_uart_port[HOBOT_UART_NR_PORTS];

/**
 * hobot_uart_get_port - Configure the port from platform device resource info
 * @id: Port id
 *
 * Return: a pointer to a uart_port or NULL for failure
 */
static struct uart_port *hobot_uart_get_port(int id)
{
	struct uart_port *port;

	/* Try the given port id if failed use default method */
	if (hobot_uart_port[id].mapbase != 0) {
		/* Find the next unused port */
		for (id = 0; id < HOBOT_UART_NR_PORTS; id++)
			if (hobot_uart_port[id].mapbase == 0)
				break;
	}

	if (id >= HOBOT_UART_NR_PORTS)
		return NULL;

	port = &hobot_uart_port[id];

	/* At this point, we've got an empty uart_port struct, initialize it */
	spin_lock_init(&port->lock);
	port->membase = NULL;
	port->irq = 0;
	port->type = PORT_UNKNOWN;
	port->iotype = UPIO_MEM32;
	port->flags = UPF_BOOT_AUTOCONF;
	port->ops = &hobot_uart_ops;
	port->fifosize = HOBOT_UART_FIFO_SIZE;
	port->line = id;
	port->dev = NULL;
#ifdef	CONFIG_HOBOT_FPGA_HAPS_X3
	port->uartclk = 10000000;
#else
	port->uartclk = 20000000;
#endif
	return port;
}

#ifdef CONFIG_SERIAL_HOBOT_UART_CONSOLE
/**
 * hobot_uart_console_wait_tx - Wait for the TX to be full
 * @port: Handle to the uart port structure
 */
static void hobot_uart_console_wait_tx(struct uart_port *port)
{
	while (!(readl(port->membase + HOBOT_UART_LSR) & UART_LSR_TX_EMPTY))
		barrier();
}

/**
 * hobot_uart_console_putchar - write the character to the FIFO buffer
 * @port: Handle to the uart port structure
 * @ch: Character to be written
 */
static void hobot_uart_console_putchar(struct uart_port *port, int ch)
{
	hobot_uart_console_wait_tx(port);
	writel(ch, port->membase + HOBOT_UART_TDR);
}

static void __init hobot_early_write(struct console *con, const char *s,
				  unsigned n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, hobot_uart_console_putchar);
}

static int __init hobot_early_console_setup(struct earlycon_device *device,
					 const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = hobot_early_write;
	early_console_baud = device->baud;

	return 0;
}

OF_EARLYCON_DECLARE(hobot, "hobot,hobot-uart", hobot_early_console_setup);

/**
 * hobot_uart_console_write - perform write operation
 * @co: Console handle
 * @s: Pointer to character array
 * @count: No of characters
 */
static void hobot_uart_console_write(struct console *co, const char *s,
				  unsigned int count)
{
	struct uart_port *port = &hobot_uart_port[co->index];
	unsigned long flags;
	volatile unsigned int ctrl, dma;
	int locked = 1;

	if (!uarttx_ctrl)
		return;
	/* Check if tx dma is enabled. */
	while ((ctrl = readl(port->membase + HOBOT_UART_TXDMA)) & UART_TXSTA) {
		cpu_relax();
	}

	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock_irqsave(&port->lock, flags);
	else
		spin_lock_irqsave(&port->lock, flags);

	/* save and disable dma model */
	dma = readl(port->membase + HOBOT_UART_FCR);
	writel(dma & (~UART_FCR_TDMA_EN), port->membase + HOBOT_UART_FCR);

	/*
	 * Make sure that the tx part is enabled. Set the TX enable bit and
	 * clear the TX disable bit to enable the transmitter.
	 */
	ctrl = readl(port->membase + HOBOT_UART_ENR);
	ctrl |= UART_ENR_TX_EN;
	writel(ctrl, port->membase + HOBOT_UART_ENR);

	uart_console_write(port, s, count, hobot_uart_console_putchar);

	/* wait for transmitter to become empty */
	hobot_uart_console_wait_tx(port);

	/* restore dma model */
	writel(dma, port->membase + HOBOT_UART_FCR);

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
	return;
}

/**
 * hobot_uart_console_setup - Initialize the uart to default config
 * @co: Console handle
 * @options: Initial settings of uart
 *
 * Return: 0 on success, negative errno otherwise.
 */
static int __init hobot_uart_console_setup(struct console *co, char *options)
{
	struct uart_port *port = NULL;
	int baud = early_console_baud;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= HOBOT_UART_NR_PORTS)
		return -EINVAL;

    port = &hobot_uart_port[co->index];

	if (!port->membase) {
		pr_debug("console on " HOBOT_UART_TTY_NAME "%i not present\n",
			 co->index);
		return -ENODEV;
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver hobot_uart_driver;

static struct console hobot_uart_console = {
	.name = HOBOT_UART_TTY_NAME,
	.write = hobot_uart_console_write,
	.device = uart_console_device,
	.setup = hobot_uart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,		/* Specified on the cmdline (e.g. console=tty* ) */
	.data = &hobot_uart_driver,
};

/**
 * hobot_uart_console_init - Initialization call
 *
 * Return: 0 on success, negative errno otherwise
 */
static int __init hobot_uart_console_init(void)
{
	register_console(&hobot_uart_console);

	return 0;
}

console_initcall(hobot_uart_console_init);

#endif /* CONFIG_SERIAL_HOBOT_UART_CONSOLE */

static struct uart_driver hobot_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = HOBOT_UART_NAME,
	.dev_name = HOBOT_UART_TTY_NAME,
	.major = HOBOT_UART_MAJOR,
	.minor = HOBOT_UART_MINOR,
	.nr = HOBOT_UART_NR_PORTS,
#ifdef CONFIG_SERIAL_HOBOT_UART_CONSOLE
	.cons = &hobot_uart_console,
#endif
};

#ifdef CONFIG_PM
/**
 * hobot_uart_suspend - suspend event
 * @device: Pointer to the device structure
 *
 * Return: 0
 */
static int hobot_uart_suspend(struct device *device)
{
	/* Nothing to do, no implement */
	struct uart_port *port = dev_get_drvdata(device);

	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

#if IS_ENABLED(CONFIG_SUSPEND)
	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE)
		return 0;
#endif

	uart_suspend_port(&hobot_uart_driver, port);

	return 0;
}

/**
 * hobot_uart_resume - Resume after a previous suspend
 * @device: Pointer to the device structure
 *
 * Return: 0
 */
static int hobot_uart_resume(struct device *device)
{
	/* Nothing to do, no implement */
	struct uart_port *port = dev_get_drvdata(device);

#if IS_ENABLED(CONFIG_SUSPEND)
	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE)
		return 0;
#endif

	uart_resume_port(&hobot_uart_driver, port);

	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);
	return 0;
}
#endif /* ! CONFIG_PM_SLEEP */

static int __maybe_unused hobot_runtime_suspend(struct device *dev)
{
	/* Nothing to do, no implement */
	return 0;
};

static int __maybe_unused hobot_runtime_resume(struct device *dev)
{
	/* Nothing to do, no implement */
	return 0;
};

/*TODO: Maybe no need */
static const struct dev_pm_ops hobot_uart_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hobot_uart_suspend, hobot_uart_resume)
		SET_RUNTIME_PM_OPS(hobot_runtime_suspend,
				   hobot_runtime_resume, NULL)
};

/* Match table for of_platform binding */
static const struct of_device_id hobot_uart_of_match[] = {
	{.compatible = "hobot,hobot-uart", },
	{}
};

MODULE_DEVICE_TABLE(of, hobot_uart_of_match);

#ifdef CONFIG_HOBOT_SERIAL_DEBUGFS
static int hobot_regaddr_get(void *data, u64 *val)
{
	*val = dgb_addr;
	return 0;
}

static int hobot_regaddr_set(void *data, u64 val)
{
	dgb_addr = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(hobot_addr_fops, hobot_regaddr_get, hobot_regaddr_set, "%llx\n");

static int hobot_regdata_get(void *data, u64 *val)
{
	*val = readl(dgb_membase + dgb_addr);
	return 0;
}

static int hobot_regdata_set(void *data, u64 val)
{
	writel(val, dgb_membase + dgb_addr);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(hobot_data_fops, hobot_regdata_get, hobot_regdata_set, "%llx\n");

static int hobot_serial_statis_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%s\t%s\t%s\t%s\n",
		"tx_cnt", "tx_head", "tx_tail", "rx_cnt");
	seq_printf(s, "%d\t%d\t%d\t%d\n",
		dgb_tx_count, dgb_tx_tail, dgb_tx_head, dgb_rx_count);

	seq_printf(s, "%s\n", "=====================================");
	seq_printf(s, "%s\t", "UART_RDR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_RDR));
	seq_printf(s, "%s\t", "UART_LCR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_LCR));
	seq_printf(s, "%s\t", "UART_ENR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_ENR));
	seq_printf(s, "%s\t", "UART_BCR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_BCR));
	seq_printf(s, "%s\t", "UART_MCR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_MCR));
	seq_printf(s, "%s\t", "UART_TCR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_TCR));
	seq_printf(s, "%s\t", "UART_FCR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_FCR));
	seq_printf(s, "%s\t", "UART_LSR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_LSR));
	seq_printf(s, "%s\t", "UART_MSR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_MSR));
	seq_printf(s, "%s\t", "UART_RXADDR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_RXADDR));
	seq_printf(s, "%s\t", "UART_RXSIZE");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_RXSIZE));
	seq_printf(s, "%s\t", "UART_RXDMA");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_RXDMA));
	seq_printf(s, "%s\t", "UART_TXADDR");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_TXADDR));
	seq_printf(s, "%s\t", "UART_TXSIZE");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_TXSIZE));
	seq_printf(s, "%s\t", "UART_TXDMA");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_TXDMA));
	seq_printf(s, "%s\t", "UART_INTMASK");
	seq_printf(s, "%x\n", readl(dgb_membase + HOBOT_UART_INT_MASK));
	return 0;
}

static int hobot_serial_status_open_seq(struct inode *inode, struct file *file)
{
	return single_open(file, hobot_serial_statis_show, inode->i_private);
}

static const struct file_operations hobot_serial_status_fops = {
	.open = hobot_serial_status_open_seq,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void hobot_uart_debugfs_init(struct platform_device *pdev)
{
	dentry_root = debugfs_create_dir("hobot_serial", NULL);
	if (!dentry_root) {
		dev_warn(&pdev->dev, "Failed to create debugfs root directory\n");
		return;
	}

	dentry_data = debugfs_create_file("regdata", 0644,
			dentry_root, NULL, &hobot_data_fops);
	if (!dentry_data) {
		dev_warn(&pdev->dev, "Failed to create debugfs regdata file\n");
		return;
	}

	dentry_addr = debugfs_create_file("regaddr", 0644,
			dentry_root, NULL, &hobot_addr_fops);
	if (!dentry_addr) {
		dev_warn(&pdev->dev, "Failed to create debugfs regaddr file\n");
		return;
	}

	dentry_status = debugfs_create_file("statis", 0644,
			dentry_root, NULL, &hobot_serial_status_fops);
	if (!dentry_status) {
		dev_warn(&pdev->dev, "Failed to create debugfs status file\n");
		return;
	}
}
#endif /* CONFIG_HOBOT_SERIAL_DEBUGFS */

/**
 * hobot_uart_probe - Platform driver probe
 * @pdev: Pointer to the platform device structure
 *
 * Return: 0 on success, negative errno otherwise
 */
static int hobot_uart_probe(struct platform_device *pdev)
{
	int rc, id, irq;
	struct uart_port *port;
	struct resource *res;
	struct hobot_uart *hobot_uart_data;
	const struct of_device_id *match;

#ifdef CONFIG_HOBOT_SERIAL_DEBUGFS
	hobot_uart_debugfs_init(pdev);
#endif /* CONFIG_HOBOT_SERIAL_DEBUGFS */

	hobot_uart_data = devm_kzalloc(&pdev->dev, sizeof(*hobot_uart_data),
					GFP_KERNEL);
	if (!hobot_uart_data)
		return -ENOMEM;

	match = of_match_node(hobot_uart_of_match, pdev->dev.of_node);
	if (match && match->data) {
		/* Nothing to do, maybe can be used in future */
	}
	/*Initialize the uartoutcnt */
	hobot_uart_data->uartoutcnt = 0;
	hobot_uart_data->rx_bytes = 0;
	hobot_uart_data->uartclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(hobot_uart_data->uartclk)) {
		dev_err(&pdev->dev, "uart_clk clock not found.\n");
		return PTR_ERR(hobot_uart_data->uartclk);
	}
	rc = clk_prepare_enable(hobot_uart_data->uartclk);
	if (rc) {
		dev_err(&pdev->dev, "Unable to enable device clock.\n");
		goto err_out_clk_disable;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		rc = -ENODEV;
		goto err_out;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		rc = -ENXIO;
		goto err_out;
	}

	/* Look for a serialN alias */
	id = of_alias_get_id(pdev->dev.of_node, "serial");
	if (id < 0)
		id = 0;
	sprintf(hobot_uart_data->name, "%s%d", HOBOT_UART_NAME, id);
	/* Initialize the port structure */
	port = hobot_uart_get_port(id);

	if (!port) {
		dev_err(&pdev->dev, "Cannot get uart_port structure\n");
		rc = -ENODEV;
		goto err_out;
	}

	/*
	 * Register the port.
	 * This function also registers this device with the tty layer
	 * and triggers invocation of the config_port() entry point.
	 */
	port->mapbase = res->start;
	port->irq = irq;
	port->dev = &pdev->dev;
	if (0 == IS_ENABLED(CONFIG_HOBOT_FPGA_X2) && 0 == IS_ENABLED(CONFIG_HOBOT_FPGA_X3)) {
		port->uartclk = clk_get_rate(hobot_uart_data->uartclk);
	}
	port->private_data = hobot_uart_data;
	hobot_uart_data->port = port;
	platform_set_drvdata(pdev, port);

	rc = uart_add_one_port(&hobot_uart_driver, port);
	if (rc < 0) {
		dev_err(&pdev->dev, "uart_add_one_port() failed; err=%i\n", rc);
		goto err_out;
	}
#ifdef HOBOT_UART_DBG
	pr_info("====> address of dbg_tx_cnt : 0x%16lx\n", &dbg_tx_cnt);
#endif /* HOBOT_UART_DBG */

#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	hobot_uart_data->uart_dpm.dpm_call = serial_dpm_callback;
	hobot_dpm_register(&hobot_uart_data->uart_dpm, &pdev->dev);
	atomic_set(&hobot_uart_data->uart_start, 0);
	atomic_set(&hobot_uart_data->dmatx_flag, 0);
#endif

#ifdef CONFIG_HOBOT_DIAG
	/* diag */
	hobot_uart_data->uart_id = id;
	if (diag_register(ModuleDiag_uart, EventIdUart0Err + id,
				4, DIAG_MSG_INTERVAL_MIN, DIAG_MSG_INTERVAL_MAX, NULL) < 0)
		dev_err(&pdev->dev, "uart%d diag register fail\n", id);
#endif

	return 0;

err_out_clk_disable:
	clk_disable_unprepare(hobot_uart_data->uartclk);
err_out:
	return rc;
}

/**
 * hobot_uart_remove - called when the platform driver is unregistered
 * @pdev: Pointer to the platform device structure
 *
 * Return: 0 on success, negative errno otherwise
 */
static int hobot_uart_remove(struct platform_device *pdev)
{
	int rc;
	struct uart_port *port = platform_get_drvdata(pdev);
	struct hobot_uart *hobot_uart_data = port->private_data;

	rc = uart_remove_one_port(&hobot_uart_driver, port);
	port->mapbase = 0;
	clk_disable_unprepare(hobot_uart_data->uartclk);
#if IS_ENABLED(CONFIG_HOBOT_BUS_CLK_X3)
	hobot_dpm_unregister(&hobot_uart_data->uart_dpm);
#endif
	return rc;
}

static struct platform_driver hobot_uart_platform_driver = {
	.probe = hobot_uart_probe,
	.remove = hobot_uart_remove,
	.driver = {
		   .name = HOBOT_UART_NAME,
		   .of_match_table = hobot_uart_of_match,
		   .pm = &hobot_uart_dev_pm_ops,
		   },
};

static int __init hobot_uart_init(void)
{
	int retval = 0;

	/* Register the hobot_uart driver with the serial core */
	retval = uart_register_driver(&hobot_uart_driver);
	if (retval)
		return retval;

	/* Register the platform driver */
	retval = platform_driver_register(&hobot_uart_platform_driver);
	if (retval)
		uart_unregister_driver(&hobot_uart_driver);

	return retval;
}

static void __exit hobot_uart_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&hobot_uart_platform_driver);

	/* Unregister the hobot_uart driver */
	uart_unregister_driver(&hobot_uart_driver);
}

module_init(hobot_uart_init);
module_exit(hobot_uart_exit);

MODULE_DESCRIPTION("Driver for HOBOT UART");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
