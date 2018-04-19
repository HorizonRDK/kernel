/*
 * X2 UART driver (For X2 Platform)
 *
 * 2017 - 2018 (C) Horizon Inc.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any
 * later version.
 *
 * This driver has originally been pushed by Horizon using a X2-branding. This
 * still shows in the naming of this file, the kconfig symbols and some symbols
 * in the code.
 */

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
#include <linux/pm_runtime.h>

#define X2_UART_TTY_NAME	"ttyS"
#define X2_UART_NAME		"x2-uart"
#define X2_UART_MAJOR		0	/* use dynamic node allocation */
#define X2_UART_MINOR		0	/* works best with devtmpfs */
#define X2_UART_NR_PORTS	CONFIG_SERIAL_X2_NR_UARTS
#define X2_UART_FIFO_SIZE	32	/* FIFO size */
#define X2_UART_REGISTER_SPACE	0x1000

/* Rx Trigger level */
static int rx_trigger_level = 8;
module_param(rx_trigger_level, uint, S_IRUGO);
MODULE_PARM_DESC(rx_trigger_level, "Rx trigger level, 1-16(uint 4 bytes)");

/* Tx Trigger level */
static int tx_trigger_level = 8;
module_param(tx_trigger_level, uint, S_IRUGO);
MODULE_PARM_DESC(tx_trigger_level, "Tx trigger level, 0-15 (uint: 4 bytes)");


/* the offset of uart registers and BITs for them */
#define X2_UART_RDR			0x0000		/* RO: UART Receive Data Register */
#define X2_UART_TDR			0x0004		/* WO: UART RX Data Register */
#define X2_UART_LCR			0x0008		/* RW: UART Line Control Register */
#define X2_UART_ENR			0x000c      /* RW: UART Enable Regsiter */
#define X2_UART_BCR			0x0010      /* RW: UART BAUD Rate Configuration Register */
#define X2_UART_MCR			0x0014      /* RW: UART Modem Control Register */
#define X2_UART_TCR			0x0018      /* RW: UART Test Contorl Register */
#define X2_UART_FCR			0x001c      /* RW: UART FIFO Configuration Register.*/
#define X2_UART_LSR			0x0020      /* RU: UART Line Status Register.*/
#define X2_UART_MSR			0x0024      /* RU: UART Modem Status Register */
#define X2_UART_RXADDR		0x0028      /* RW: UART Receive DMA Base Address Register.*/
#define X2_UART_RXSIZE		0x002c      /* RW: UART Receive DMA Size Register */
#define X2_UART_RXDMA		0x0030      /* RW: UART Receive DMA Control Register */
#define X2_UART_TXADDR		0x0034      /* RW: UART Transmit DMA Base Address Register */
#define X2_UART_TXSIZE		0x0038      /* RW: UART Transmit DMA Size Register */
#define X2_UART_TXDMA		0x003c      /* RW: UART Transmit DMA Control Register.*/
#define X2_UART_SRC_PND		0x0040      /* WC: UART Interrupt Source Pending Register */
#define X2_UART_INT_MASK	0x0044      /* RO: UART Interrupt Mask Register */
#define X2_UART_INT_SETMASK 0x0048      /* WO: UART Interrupt Set Mask Register */
#define X2_UART_INT_UNMASK  0x004c      /* WO: UART Interrupt Unmask Register.*/
#define X2_UART_IR_EN		0x0100      /* RW: IR mode enable*/
#define X2_UART_IR_CTRL		0x0104      /* RW: IR configuration registers*/
#define X2_UART_IR_FIL		0x0108      /* RW: IR glitch filter time = IR_FIL*Period of cir_clock. Default = 1us*/
#define X2_UART_IR_LEADS	0x010c      /* RW: IR AGC timing cont*/
#define X2_UART_IR_LEADE	0x0110      /* RW: time lag of AGC to generate repeat Register */
#define X2_UART_IR_SLEADE   0x0114      /* RW: IR time lag of AGC to generate command Regsiter */
#define X2_UART_IR_BIT0     0x0118      /* RW: IR time to generate 0 */
#define X2_UART_IR_BIT1     0x011c      /* RW: IR time to generate 1 */
#define X2_UART_IR_BUSY     0x0120      /* RU: IR busy stauts */
#define X2_UART_IR_SRCPND   0x0124      /* WC: IR interrupt source pending register */
#define X2_UART_IR_INTMASK  0x0128      /* RU: IR interrupt mask Register */
#define X2_UART_IR_SETMASK  0x012c      /* WO: IR setmask register */
#define X2_UART_IR_UNMASK   0x0130      /* WO: IR unmask register */
#define X2_UART_IR_DATA_H   0x0134      /* RU: receive high 16 bits data Register*/
#define X2_UART_IR_DATA_L   0x0138      /* RU: receive low 32 bits data Register */
#define X2_UART_IR_TIMEOUT  0x013c      /* RW: IR timeout for high recovery= period *IR_TIMEOUT Register */

/* Uart line control register bits define */
#define	UART_LCR_8_BIT 	(1U << 0)
#define UART_LCR_7_BIT  (0U << 0)
#define UART_LCR_2_STOP	(1U << 1)
#define UART_LCR_1_STOP (~UART_LCR_2_STOP)
#define UART_LCR_PEN 		(1U << 2)
#define UART_LCR_EPS		(1U << 3)
#define UART_LCR_SP		(1U << 4)
#define UART_LCR_CTS_EN	(1U << 5)
#define UART_LCR_RTS_EN	(1U << 6)
#define UART_LCR_SFC_EN	(1U << 7)
#define UART_LCR_TOI_MASK (0xF)
#define UART_LCR_TOI(x)		(((x) & UART_LCR_TOI_MASK) << 8U)
#define UART_LCR_IREN		(1U << 12)
#define UART_LCR_RXPOL    (1U << 13)

/* Uart enable register bits define */
#define UART_ENR_MASK (0x7)
#define UART_ENR_EN 		(UART_ENR_MASK & (1U << 0))	/* uart gloabl enabled bit */
#define UART_ENR_DIS  (UART_ENR_MASK & (~(1U << 0)))
#define UART_ENR_RX_EN	(UART_ENR_MASK & (1U << 1))		/* RX enabled bit */
#define UART_ENR_RX_DIS (UART_ENR_MASK & (~(1U << 1)))  /* RX disable bit */
#define UART_ENR_TX_EN	(UART_ENR_MASK & (1U << 2))		/* TX enabled bit */
#define UART_ENR_TX_DIS	(UART_ENR_MASK & (~(1U << 2)))	/* TX disable bit */

/* Uart BAUD Rate Register Bits define */
#define UART_BCR_BRDIV_INT_MASK (0xFFFF)
#define UART_BCR_BRDIV_INT(x) (((x) & UART_BCR_BRDIV_INT_MASK) << 0u)
#define UART_BCR_BRDIV_FRAC_MASK (0xFFFF)
#define UART_BCR_BRDIV_FRAC(x) (((x) & UART_BCR_BRDIV_FRAC_MASK) << 16U)
#define UART_BCR_BRDIV_MODE_MASK  (0x3)
#define UART_BCR_BRDIV_MODE(x) (((x) & UART_BCR_BRDIV_MODE_MASK) << 28U)
#define LOW_SPEED_MODE_DIV 	16
#define MID_SPEED_MODE_DIV 	8
#define HIGH_SPEED_MODE_DIV	4
#define X2_UART_BCR_MINBAUD 0
#define X2_UART_BCR_MAXBAUD 115200

/* Uart modem control register bits define */
#define UART_MCR_DTRN			(1U << 0)
#define UART_MCR_RTSN			(1U << 1)
#define UART_MCR_LMD_MASK		0x3
#define UART_MCR_LMD(x)		(((x) & X2_MCR_LMD_MASK) << 2U)
#define UART_MCR_XON_MASK		0xFF
#define UART_MCR_XON(x)		(((x) & X2_MCR_XON_MASK) << 8U)
#define UART_MCR_XOFF_MASK	0xFF
#define UART_MCR_XOFF(x)		(((x) & X2_MCR_XOFF_MASK) << 16U)

/* Uart test control register bits define */
#define UART_TCR_LB			(1U << 0)
#define UART_TCR_FFE			(1U << 1)
#define UART_TCR_FPE			(1U << 2)
#define UART_TCR_BRK			(1U << 3)

/* Uart fifo configuration register bits define */
#define UART_FCR_RDMA_EN		(1U << 0)
#define UART_FCR_TDMA_EN		(1U << 1)
#define UART_FCR_RFRST		(1U << 2)
#define UART_FCR_TFRST		(1U << 3)
#define UART_FCR_RFTRL_MASK	0x1F
#define UART_FCR_RFTRL(x)		(((x) & UART_FCR_RFTRL_MASK) << 16U)
#define UART_FCR_TFTRL_MASK	0x1F
#define UART_FCR_TFTRL(x)		(((x) & UART_FCR_TFTRL_MASK) << 24U)

/* Uart line status register bits define */
#define UART_LSR_TX_EMPTY		(1U << 12)
#define UART_LSR_TF_EMPTY 	(1U << 11)
#define UART_LSR_TF_TLR 		(1U << 10)
#define UART_LSR_TXRDY 		(1U << 9)
#define UART_LSR_TXBUSY 		(1U << 8)
#define UART_LSR_RF_TLR 		(1U << 7)
#define UART_LSR_RX_TO 		(1U << 6)
#define UART_LSR_RX_OE		(1U << 5)
#define UART_LSR_B_INT		(1U << 4)
#define UART_LSR_F_ERR		(1U << 3)
#define UART_LSR_P_ERR		(1U << 2)
#define UART_LSR_RXRDY 		(1U << 1)
#define UART_LSR_RXBUSY 		(1U << 0)

/*Uart modem status register bits define */
#define UART_MSR_CTS_N (1U << 0)
#define UART_MSR_DSR_N (1U << 1)
#define UART_MSR_DCD_N (1U << 2)
#define UART_MSR_RI_N  (1U << 3)
#define UART_MSR_CTS_C (1U << 4)
#define UART_MSR_DSR_C (1U << 5)
#define UART_MSR_DCD_C (1U << 6)
#define UART_MSR_RI_C  (1U << 7)

/*Uart receive DMA register bits define */
#define UART_RXTHD_MASK (0xF)
#define UART_DMA_RXTHD(x) (((x) & UART_RXTHD_MASK) << 12u)
#define UART_RXMOS_MASK (0xF)
#define UART_RXMOS(x) (((x) & UART_RXMOS_MASK) << 8U)
#define UART_RXLEN_MASK (0xF)
#define UART_RXLEN(x) (((x) & UART_RXLEN_MASK) << 4U)
#define UART_RXAE (1U << 3)
#define UART_RXWRAP (1U << 2)
#define UART_RXSTP (1U << 1)
#define UART_RXSTA (1U << 0)

/*Uart Transmit DMA register bits define */
#define UART_TXTHD_MASK (0xF)
#define UART_DMA_TXTHD(x) (((x) & UART_TXTHD_MASK) << 12u)
#define UART_TXMOS_MASK (0xF)
#define UART_TXMOS(x) (((x) & UART_TXMOS_MASK) << 8U)
#define UART_TXLEN_MASK (0xF)
#define UART_TXLEN(x) (((x) & UART_TXLEN_MASK) << 4U)
#define UART_TXAE (1U << 3)
#define UART_TXWRAP (1U << 2)
#define UART_TXSTP (1U << 1)
#define UART_TXSTA (1U << 0)

/*
 * UART Interrupt Source Pending Register \
 * UART Interrupt Mask Register \
 * UART Interrupt Set Mask Register \
 * UART Interrupt Unmask Register bits define
 * */
#define UART_TXEPT (1U << 14)
#define UART_TXTHD (1U << 13)
#define UART_TXDON (1U << 12)
#define UART_RXFUL (1U << 11)
#define UART_RXTO  (1U << 10)
#define UART_RXOE  (1U << 9)
#define UART_BI    (1U << 8)
#define UART_FE    (1U << 7)
#define UART_PE    (1U << 6)
#define UART_RXTHD (1U << 5)
#define UART_RXDON (1U << 4)
#define UART_RIC   (1U << 3)
#define UART_DCDC  (1U << 2)
#define UART_DSRC  (1U << 1)
#define UART_CTSC  (1U << 0)

/**
 * struct x2_uart - device data
 * @port:		Pointer to the UART port
 * @baud:		Current baud rate
 */
struct x2_uart {
	struct uart_port	*port;
	unsigned int		baud;
};

#define to_x2_uart(_nb) container_of(_nb, struct x2_uart, \
		clk_rate_change_nb);
/**
 * x2_uart_handle_rx - Handle the received bytes along with Rx errors.
 * @dev_id: Id of the UART port
 * @irqstatus: The interrupt status register value as read
 * Return: None
 */
static void x2_uart_handle_rx(void *dev_id, unsigned int irqstatus)
{
	struct uart_port *port = (struct uart_port *)dev_id;
	unsigned int data;
	char status = TTY_NORMAL;

	//while ((readl(port->membase + x2_UART_LSR) & UART_LSR_RXRDY)) {
	  while (irqstatus & UART_RXFUL) {
		data = readl(port->membase + X2_UART_RDR);
		port->icount.rx++;

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

/**
 * x2_uart_stop_tx - Stop TX
 * @port: Handle to the uart port structure
 */
static void x2_uart_stop_tx(struct uart_port *port)
{
	unsigned int regval;

	/* Disable the transmitter */
	regval = readl(port->membase + X2_UART_ENR);
	regval &= UART_ENR_TX_DIS;
	writel(regval, port->membase + X2_UART_ENR);
}

/**
 * x2_uart_handle_tx - Handle the bytes to be Txed.
 * @dev_id: Id of the UART port
 * Return: None
 */
static void x2_uart_handle_tx(void *dev_id)
{
	struct uart_port *port = (struct uart_port *)dev_id;
	unsigned int numbytes;

	if (uart_circ_empty(&port->state->xmit)) {
		x2_uart_stop_tx(port);
	} else {
		numbytes = port->fifosize;
		while (numbytes && !uart_circ_empty(&port->state->xmit) &&
		       (readl(port->membase + X2_UART_SRC_PND) & UART_TXEPT)) {
			/*
			 * Get the data from the UART circular buffer
			 * and write it to the cdns_uart's TX_FIFO
			 * register.
			 */
			writel(port->state->xmit.buf[port->state->xmit.tail],
				port->membase + X2_UART_TDR);

			port->icount.tx++;

			/*
			 * Adjust the tail of the UART buffer and wrap
			 * the buffer if it reaches limit.
			 */
			port->state->xmit.tail =
				(port->state->xmit.tail + 1) &
					(UART_XMIT_SIZE - 1);

			numbytes--;
		}

		if (uart_circ_chars_pending(
				&port->state->xmit) < WAKEUP_CHARS)
			uart_write_wakeup(port);
	}
}

/**
 * x2_uart_isr - Interrupt handler
 * @irq: Irq number
 * @dev_id: Id of the port
 *
 * Return: IRQHANDLED
 */
static irqreturn_t x2_uart_isr(int irq, void *dev_id)
{
	struct uart_port *port = (struct uart_port *)dev_id;
	unsigned int irqstatus;

	spin_lock(&port->lock);

	/* Read the interrupt status register to determine which
	 * interrupt(s) is/are active and clear them.
	 */
	irqstatus = readl(port->membase + X2_UART_SRC_PND);
	writel(irqstatus, port->membase + X2_UART_SRC_PND);

	if (irqstatus & UART_TXEPT) {
		x2_uart_handle_tx(dev_id);
		irqstatus &= ~UART_TXEPT;
	}
	if (irqstatus & UART_RXFUL)
		x2_uart_handle_rx(dev_id, irqstatus);

	spin_unlock(&port->lock);
	return IRQ_HANDLED;
}


/**
 * x2_uart_calc_baud_divs - Calculate baud rate divisors
 * @clk: UART module input clock
 * @baud: Desired baud rate
 * @rbdiv: BDIV value (return value)
 * @rcd: CD value (return value)
 * @div8: Value for clk_sel bit in mod (return value)
 * Return: baud rate, requested baud when possible, or actual baud when there
 *	was too much error, zero if no valid divisors are found.
 */
static unsigned int x2_uart_calc_baud_divs(unsigned int clk,
		unsigned int baud, u32 *br_int, u32 *br_frac)
{
	int err = ~0;

	if (br_int == NULL || br_frac == NULL)
		return err;

	if (clk <= 0 || baud < 0)
		return err;

#ifdef CONFIG_UART_LOW_SPEED_MODE
	*br_int = (clk / (baud * LOW_SPEED_MODE_DIV));
	*br_frac = (clk % (baud * LOW_SPEED_MODE_DIV)) * 1024 / (baud * LOW_SPEED_MODE_DIV);
#elif CONFIG_UART_MID_SPEED_MODE
	*br_int = (clk / (baud * MID_SPEED_MODE_DIV));
	*br_frac = (clk % (baud * MID_SPEED_MODE_DIV)) * 1024 / (baud * MID_SPEED_MODE_DIV);
#elif CONFIG_UART_HIGH_SPEED_MODE
	*br_int = (clk / (baud * HIGH_SPEED_MODE_DIV));
	*br_frac = (clk % (baud * HIGH_SPEED_MODE_DIV)) * 1024 / (baud * HIGH_SPEED_MODE_DIV);
#endif
	return baud;
}

/**
 * x2_uart_set_baud_rate - Calculate and set the baud rate
 * @port: Handle to the uart port structure
 * @baud: Baud rate to set
 * Return: baud rate, requested baud when possible, or actual baud when there
 *	   was too much error, zero if no valid divisors are found.
 */
static unsigned int x2_uart_set_baud_rate(struct uart_port *port,
		unsigned int baud)
{
	unsigned int calc_baud;
	u32 bdiv_int = 0, bdiv_frac = 0;
	u32 bcr_reg;
	struct x2_uart *x2_uart = port->private_data;

	calc_baud = x2_uart_calc_baud_divs(port->uartclk, baud, &bdiv_int, &bdiv_frac);

	/* Write new divisors to hardware */
	bcr_reg = readl(port->membase + X2_UART_BCR);

#ifdef CONFIG_UART_LOW_SPEED_MODE
	bcr_reg |= UART_BCR_BRDIV_MODE(0);
#elif CONFIG_UART_MID_SPEED_MODE
	bcr_reg |= UART_BCR_BRDIV_MODE(1);
#elif CONFIG_UART_HIGH_SPEED_MODE
	bcr_reg |= UART_BCR_BRDIV_MODE(2);
#endif

	bcr_reg |= bdiv_int | bdiv_frac;
	writel(bcr_reg, port->membase + X2_UART_BCR);

	x2_uart->baud = baud;
	return calc_baud;
}

#if 0
/**
 * x2_uart_clk_notitifer_cb - Clock notifier callback
 * @nb:		Notifier block
 * @event:	Notify event
 * @data:	Notifier data
 * Return:	NOTIFY_OK or NOTIFY_DONE on success, NOTIFY_BAD on error.
 */
static int x2_uart_clk_notifier_cb(struct notifier_block *nb,
		unsigned long event, void *data)
{
	/* Current no implement */
	return 0;
}
#endif

/**
 * x2_uart_start_tx -  Start transmitting bytes
 * @port: Handle to the uart port structure
 */
static void x2_uart_start_tx(struct uart_port *port)
{
	unsigned int status;

	if (uart_tx_stopped(port))
		return;

	/*
	 * Set the TX enable bit and clear the TX disable bit to enable the
	 * transmitter.
	 */
	status = readl(port->membase + X2_UART_ENR);
	status |= UART_ENR_TX_EN;
	writel(status, port->membase + X2_UART_ENR);

	if (uart_circ_empty(&port->state->xmit))
		return;

	x2_uart_handle_tx(port);
}

/**
 * x2_uart_stop_rx - Stop RX
 * @port: Handle to the uart port structure
 */
static void x2_uart_stop_rx(struct uart_port *port)
{
	unsigned int regval;

	/* Disable the receiver */
	regval = readl(port->membase + X2_UART_ENR);
	regval &= UART_ENR_RX_DIS;
	writel(regval, port->membase + X2_UART_ENR);
}


/**
 * x2_uart_tx_empty -  Check whether TX is empty
 * @port: Handle to the uart port structure
 *
 * Return: TIOCSER_TEMT on success, 0 otherwise
 */
static unsigned int x2_uart_tx_empty(struct uart_port *port)
{
	unsigned int status;

	status = readl(port->membase + X2_UART_LSR) &
				UART_LSR_TX_EMPTY;

	return status ? TIOCSER_TEMT : 0;
}

/**
 * x2_uart_break_ctl - Based on the input ctl we have to start or stop
 *			transmitting char breaks
 * @port: Handle to the uart port structure
 * @ctl: Value based on which start or stop decision is taken
 */
static void x2_uart_break_ctl(struct uart_port *port, int ctl)
{
	/* Nothing to do */
}

/**
 * x2_uart_set_termios - termios operations, handling data length, parity,
 *				stop bits, flow control, baud rate
 * @port: Handle to the uart port structure
 * @termios: Handle to the input termios structure
 * @old: Values of the previously saved termios structure
 */
static void x2_uart_set_termios(struct uart_port *port,
				struct ktermios *termios, struct ktermios *old)
{
	uint32_t ctrl_reg, lcr_reg;
	uint32_t baud, minbaud, maxbaud;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* Wait for the transmit FIFO to empty before making changes */
	if ((readl(port->membase + X2_UART_ENR) & UART_ENR_TX_EN)) {
		while (!(readl(port->membase + X2_UART_LSR) &
				UART_LSR_TX_EMPTY)) {
			cpu_relax();
		}
	}

	/* Disable the TX and RX to set baud rate */
	ctrl_reg = readl(port->membase + X2_UART_ENR);
	ctrl_reg &= UART_ENR_TX_DIS | UART_ENR_RX_DIS;
	writel(ctrl_reg, port->membase + X2_UART_ENR);

	minbaud = X2_UART_BCR_MINBAUD;
	maxbaud = X2_UART_BCR_MAXBAUD;
	baud = uart_get_baud_rate(port, termios, old, minbaud, maxbaud);
	baud = x2_uart_set_baud_rate(port, baud);
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);

	/* Set TX/RX Reset */
	ctrl_reg = readl(port->membase + X2_UART_FCR);
	ctrl_reg |= UART_FCR_RFRST | UART_FCR_TFRST;
	writel(ctrl_reg, port->membase + X2_UART_FCR);

	while (readl(port->membase + X2_UART_FCR) &
		(UART_FCR_RFRST | UART_FCR_TFRST))
		cpu_relax();

	/*
	 * Set the TX enable bit and RX enable bit
	 * to enable the transmitter and receiver.
	*/
	ctrl_reg = readl(port->membase + X2_UART_ENR);
	ctrl_reg |= UART_ENR_TX_EN | UART_ENR_RX_EN;
	writel(ctrl_reg, port->membase + X2_UART_ENR);

	port->read_status_mask = UART_TXEPT | UART_RXFUL | UART_RXOE;
	port->ignore_status_mask = 0;

	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_PE | UART_FE;

	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_PE |
			UART_FE | UART_RXOE;

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_RXFUL |
			UART_RXTO | UART_PE | UART_FE | UART_RXOE;

	lcr_reg = readl(port->membase + X2_UART_LCR);

	/* Handling Data Size */
	switch (termios->c_cflag & CSIZE) {
	case CS7:
		lcr_reg |= UART_LCR_7_BIT;
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
		lcr_reg |= UART_LCR_2_STOP; /* 2 STOP bits */
	else
		lcr_reg |= UART_LCR_1_STOP; /* 1 STOP bit */

	termios->c_cflag &= ~CMSPAR;	/* no support mark/space */

	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD) {
			lcr_reg &= ~UART_LCR_EPS;
		} else {
			lcr_reg |= UART_LCR_EPS;
		}
	}

	/* flow control */
	if (termios->c_cflag & CRTSCTS)
		lcr_reg |= UART_LCR_RTS_EN | UART_LCR_CTS_EN;

	writel(lcr_reg, port->membase + X2_UART_LCR);

	spin_unlock_irqrestore(&port->lock, flags);
}

/**
 * x2_uart_startup - Called when an application opens a x2_uart port
 * @port: Handle to the uart port structure
 *
 * Return: 0 on success, negative errno otherwise
 */
static int x2_uart_startup(struct uart_port *port)
{
	int ret;
	unsigned long flags;
	unsigned int val = 0;

	spin_lock_irqsave(&port->lock, flags);

	/* First Disable Uart */
	writel(UART_ENR_DIS, port->membase + X2_UART_ENR);

	/* Set the line Control Register with normal mode,8 data bits,1 stop bit,
	 * no parity.
	 */
	val = readl(port->membase + X2_UART_LCR);
	val |= UART_LCR_8_BIT | UART_LCR_1_STOP | (~UART_LCR_PEN);
	writel(val, port->membase + X2_UART_LCR);

	/* Reset TX/RX FIFO */
	val = readl(port->membase + X2_UART_FCR);
	val |= UART_FCR_RFRST | UART_FCR_TFRST;
	writel(val, port->membase + X2_UART_FCR);

	/* Wait hardware auto clear */
	while (readl(port->membase + X2_UART_FCR) &
		(UART_FCR_RFRST | UART_FCR_TFRST))
		cpu_relax();

	/* Set TX/RX FIFO Trigger level and disable dma tx/rx */
	val = readl(port->membase + X2_UART_FCR);
	val &= ~(UART_FCR_RDMA_EN | UART_FCR_TDMA_EN);
	val |= UART_FCR_RFTRL(rx_trigger_level) | UART_FCR_TFTRL(tx_trigger_level);
	writel(val, port->membase + X2_UART_FCR);

	/* Clear all pending Interrupt */
	writel(0xffffffff, port->membase + X2_UART_SRC_PND);
	writel(0x0, port->membase + X2_UART_INT_SETMASK);
	writel(0xffffffff, port->membase + X2_UART_INT_UNMASK);

	/* Enable  global uart */
	val = readl(port->membase + X2_UART_ENR);
	val |= UART_ENR_EN;
	writel(val, port->membase + X2_UART_ENR);

	spin_unlock_irqrestore(&port->lock, flags);

	ret = request_irq(port->irq, x2_uart_isr, 0, X2_UART_NAME, port);
	if (ret) {
		dev_err(port->dev, "request_irq '%d' failed with %d\n",
			port->irq, ret);
		return ret;
	}

	return 0;
}

/**
 * x2_uart_shutdown - Called when an application closes a x2_uart port
 * @port: Handle to the uart port structure
 */
static void x2_uart_shutdown(struct uart_port *port)
{
	int status;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* Disable interrupts */
	status = readl(port->membase + X2_UART_INT_MASK);
	writel(status, port->membase + X2_UART_SRC_PND);
	writel(0xffffffff, port->membase + X2_UART_INT_SETMASK);

	/* Reset FIFO */
	status = readl(port->membase + X2_UART_FCR);
	status |= UART_FCR_RFRST | UART_FCR_TFRST;
	writel(status, port->membase + X2_UART_FCR);

	/* Disable the TX and RX */
	status = readl(port->membase + X2_UART_ENR);
	status &= UART_ENR_DIS | UART_ENR_RX_DIS | UART_ENR_TX_DIS;
	writel(status, port->membase + X2_UART_ENR);

	spin_unlock_irqrestore(&port->lock, flags);

	free_irq(port->irq, port);
}

/**
 * x2_uart_type - Set UART type to x2_uart port
 * @port: Handle to the uart port structure
 *
 * Return: string on success, NULL otherwise
 */
static const char *x2_uart_type(struct uart_port *port)
{
	return port->type == PORT_X2_UART ? X2_UART_NAME : NULL;
}

/**
 * x2_uart_verify_port - Verify the port params
 * @port: Handle to the uart port structure
 * @ser: Handle to the structure whose members are compared
 *
 * Return: 0 on success, negative errno otherwise.
 */
static int x2_uart_verify_port(struct uart_port *port,
					struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_X2_UART)
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
 * x2_uart_request_port - Claim the memory region attached to x2_uart port,
 *				called when the driver adds a x2_uart port via
 *				uart_add_one_port()
 * @port: Handle to the uart port structure
 *
 * Return: 0 on success, negative errno otherwise.
 */
static int x2_uart_request_port(struct uart_port *port)
{
	if (!request_mem_region(port->mapbase, X2_UART_REGISTER_SPACE,
					 X2_UART_NAME)) {
		return -ENOMEM;
	}

	port->membase = ioremap(port->mapbase, X2_UART_REGISTER_SPACE);
	if (!port->membase) {
		dev_err(port->dev, "Unable to map registers\n");
		release_mem_region(port->mapbase, X2_UART_REGISTER_SPACE);
		return -ENOMEM;
	}
	return 0;
}

/**
 * x2_uart_release_port - Release UART port
 * @port: Handle to the uart port structure
 *
 * Release the memory region attached to a x2_uart port. Called when the
 * driver removes a x2_uart port via uart_remove_one_port().
 */
static void x2_uart_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, X2_UART_REGISTER_SPACE);
	iounmap(port->membase);
	port->membase = NULL;
}

/**
 * x2_uart_config_port - Configure UART port
 * @port: Handle to the uart port structure
 * @flags: If any
 */
static void x2_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE && x2_uart_request_port(port) == 0)
		port->type = PORT_X2_UART;
}

/**
 * x2_uart_get_mctrl - Get the modem control state
 * @port: Handle to the uart port structure
 *
 * Return: the modem control state
 */
static unsigned int x2_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}


static void x2_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{

}

#ifdef CONFIG_CONSOLE_POLL
static int x2_uart_poll_get_char(struct uart_port *port)
{
	int c;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* Check if FIFO is empty */
	if (!readl(port->membase + X2_UART_LSR) & UART_LSR_RXRDY)
		c = NO_POLL_CHAR;
	else /* Read a character */
		c = (unsigned char) readl(port->membase + X2_UART_RDR);

	spin_unlock_irqrestore(&port->lock, flags);

	return c;
}

static void x2_uart_poll_put_char(struct uart_port *port, unsigned char c)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* Wait until FIFO is empty */
	while (!(readl(port->membase + X2_UART_LSR) & UART_LSR_TXRDY))
		cpu_relax();

	/* Write a character */
	writel(c, port->membase + X2_UART_TDR);

	/* Wait until FIFO is empty */
	while (!(readl(port->membase + X2_UART_LSR) & UART_LSR_TX_EMPTY))
		cpu_relax();

	spin_unlock_irqrestore(&port->lock, flags);

	return;
}
#endif

static void x2_uart_pm(struct uart_port *port, unsigned int state,
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


static const struct uart_ops x2_uart_ops = {
	.set_mctrl	= x2_uart_set_mctrl,
	.get_mctrl	= x2_uart_get_mctrl,
	.start_tx	= x2_uart_start_tx,
	.stop_tx	= x2_uart_stop_tx,
	.stop_rx	= x2_uart_stop_rx,
	.tx_empty	= x2_uart_tx_empty,
	.break_ctl	= x2_uart_break_ctl,
	.set_termios	= x2_uart_set_termios,
	.startup	= x2_uart_startup,
	.shutdown	= x2_uart_shutdown,
	.pm		= x2_uart_pm,
	.type		= x2_uart_type,
	.verify_port	= x2_uart_verify_port,
	.request_port	= x2_uart_request_port,
	.release_port	= x2_uart_release_port,
	.config_port	= x2_uart_config_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= x2_uart_poll_get_char,
	.poll_put_char	= x2_uart_poll_put_char,
#endif
};

static struct uart_port x2_uart_port[X2_UART_NR_PORTS];

/**
 * x2_uart_get_port - Configure the port from platform device resource info
 * @id: Port id
 *
 * Return: a pointer to a uart_port or NULL for failure
 */
static struct uart_port *x2_uart_get_port(int id)
{
	struct uart_port *port;

	/* Try the given port id if failed use default method */
	if (x2_uart_port[id].mapbase != 0) {
		/* Find the next unused port */
		for (id = 0; id < X2_UART_NR_PORTS; id++)
			if (x2_uart_port[id].mapbase == 0)
				break;
	}

	if (id >= X2_UART_NR_PORTS)
		return NULL;

	port = &x2_uart_port[id];

	/* At this point, we've got an empty uart_port struct, initialize it */
	spin_lock_init(&port->lock);
	port->membase	= NULL;
	port->irq	= 0;
	port->type	= PORT_UNKNOWN;
	port->iotype	= UPIO_MEM32;
	port->flags	= UPF_BOOT_AUTOCONF;
	port->ops	= &x2_uart_ops;
	port->fifosize	= X2_UART_FIFO_SIZE;
	port->line	= id;
	port->dev	= NULL;
	return port;
}

#ifdef CONFIG_SERIAL_X2_UART_CONSOLE
/**
 * x2_uart_console_wait_tx - Wait for the TX to be full
 * @port: Handle to the uart port structure
 */
static void x2_uart_console_wait_tx(struct uart_port *port)
{
	while (!(readl(port->membase + X2_UART_LSR) & UART_LSR_TX_EMPTY))
		barrier();
}

/**
 * x2_uart_console_putchar - write the character to the FIFO buffer
 * @port: Handle to the uart port structure
 * @ch: Character to be written
 */
static void x2_uart_console_putchar(struct uart_port *port, int ch)
{
	x2_uart_console_wait_tx(port);
	writel(ch, port->membase + X2_UART_TDR);
}

static void __init x2_early_write(struct console *con, const char *s,
				    unsigned n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, x2_uart_console_putchar);
}

static int __init x2_early_console_setup(struct earlycon_device *device,
					   const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = x2_early_write;

	return 0;
}
OF_EARLYCON_DECLARE(x2, "hobot,x2-uart", x2_early_console_setup);

/**
 * x2_uart_console_write - perform write operation
 * @co: Console handle
 * @s: Pointer to character array
 * @count: No of characters
 */
static void x2_uart_console_write(struct console *co, const char *s,
				unsigned int count)
{
	struct uart_port *port = &x2_uart_port[co->index];
	unsigned long flags;
	unsigned int ctrl;
	int locked = 1;

	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock_irqsave(&port->lock, flags);
	else
		spin_lock_irqsave(&port->lock, flags);

	/*
	 * Make sure that the tx part is enabled. Set the TX enable bit and
	 * clear the TX disable bit to enable the transmitter.
	 */
	ctrl = readl(port->membase + X2_UART_ENR);
	ctrl |= UART_ENR_TX_EN;
	writel(ctrl, port->membase + X2_UART_ENR);

	uart_console_write(port, s, count, x2_uart_console_putchar);

	/* wait for transmitter to become empty */
	x2_uart_console_wait_tx(port);

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
}

/**
 * x2_uart_console_setup - Initialize the uart to default config
 * @co: Console handle
 * @options: Initial settings of uart
 *
 * Return: 0 on success, negative errno otherwise.
 */
static int __init x2_uart_console_setup(struct console *co, char *options)
{
	struct uart_port *port = &x2_uart_port[co->index];
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= X2_UART_NR_PORTS)
		return -EINVAL;

	if (!port->membase) {
		pr_debug("console on " X2_UART_TTY_NAME "%i not present\n",
			 co->index);
		return -ENODEV;
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver x2_uart_driver;

static struct console x2_uart_console = {
	.name	= X2_UART_TTY_NAME,
	.write	= x2_uart_console_write,
	.device	= uart_console_device,
	.setup	= x2_uart_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1, /* Specified on the cmdline (e.g. console=tty* ) */
	.data	= &x2_uart_driver,
};

/**
 * x2_uart_console_init - Initialization call
 *
 * Return: 0 on success, negative errno otherwise
 */
static int __init x2_uart_console_init(void)
{
	register_console(&x2_uart_console);
	return 0;
}

console_initcall(x2_uart_console_init);

#endif /* CONFIG_SERIAL_X2_UART_CONSOLE */

static struct uart_driver x2_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= X2_UART_NAME,
	.dev_name	= X2_UART_TTY_NAME,
	.major		= X2_UART_MAJOR,
	.minor		= X2_UART_MINOR,
	.nr		= X2_UART_NR_PORTS,
#ifdef CONFIG_SERIAL_X2_UART_CONSOLE
	.cons		= &x2_uart_console,
#endif
};

#ifdef CONFIG_PM_SLEEP
/**
 * x2_uart_suspend - suspend event
 * @device: Pointer to the device structure
 *
 * Return: 0
 */
static int x2_uart_suspend(struct device *device)
{
	/* Nothing to do, no implement */
	return 0;
}

/**
 * x2_uart_resume - Resume after a previous suspend
 * @device: Pointer to the device structure
 *
 * Return: 0
 */
static int x2_uart_resume(struct device *device)
{
	/* Nothing to do, no implement */
	return 0;
}
#endif /* ! CONFIG_PM_SLEEP */

static int __maybe_unused x2_runtime_suspend(struct device *dev)
{
	/* Nothing to do, no implement */
	return 0;
};

static int __maybe_unused x2_runtime_resume(struct device *dev)
{
	/* Nothing to do, no implement */
	return 0;
};

/*TODO: Maybe no need */
static const struct dev_pm_ops x2_uart_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(x2_uart_suspend, x2_uart_resume)
	SET_RUNTIME_PM_OPS(x2_runtime_suspend,
			   x2_runtime_resume, NULL)
};

/* Match table for of_platform binding */
static const struct of_device_id x2_uart_of_match[] = {
	{ .compatible = "hobot,x2-uart", },
	{}
};
MODULE_DEVICE_TABLE(of,x2_uart_of_match);

/**
 * x2_uart_probe - Platform driver probe
 * @pdev: Pointer to the platform device structure
 *
 * Return: 0 on success, negative errno otherwise
 */
static int x2_uart_probe(struct platform_device *pdev)
{
	int rc, id, irq;
	struct uart_port *port;
	struct resource *res;
	struct x2_uart *x2_uart_data;
	const struct of_device_id *match;

	x2_uart_data = devm_kzalloc(&pdev->dev, sizeof(*x2_uart_data),
		GFP_KERNEL);
	if (!x2_uart_data)
		return -ENOMEM;

	match = of_match_node(x2_uart_of_match, pdev->dev.of_node);
	if (match && match->data) {
		/* Nothing to do, maybe can be used in future */
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

	/* Initialize the port structure */
	port = x2_uart_get_port(id);

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
	port->private_data = x2_uart_data;
	x2_uart_data->port = port;
	platform_set_drvdata(pdev, port);

	rc = uart_add_one_port(&x2_uart_driver, port);
	if (rc) {
		dev_err(&pdev->dev,
			"uart_add_one_port() failed; err=%i\n", rc);
		goto err_out;
	}

	return 0;

err_out:
	return rc;
}

/**
 * x2_uart_remove - called when the platform driver is unregistered
 * @pdev: Pointer to the platform device structure
 *
 * Return: 0 on success, negative errno otherwise
 */
static int x2_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	int rc;

	rc = uart_remove_one_port(&x2_uart_driver, port);
	port->mapbase = 0;

	return rc;
}

static struct platform_driver x2_uart_platform_driver = {
	.probe   = x2_uart_probe,
	.remove  = x2_uart_remove,
	.driver  = {
		.name = X2_UART_NAME,
		.of_match_table = x2_uart_of_match,
		.pm = &x2_uart_dev_pm_ops,
		},
};

static int __init x2_uart_init(void)
{
	int retval = 0;

	/* Register the x2_uart driver with the serial core */
	retval = uart_register_driver(&x2_uart_driver);
	if (retval)
		return retval;

	/* Register the platform driver */
	retval = platform_driver_register(&x2_uart_platform_driver);
	if (retval)
		uart_unregister_driver(&x2_uart_driver);

	return retval;
}

static void __exit x2_uart_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&x2_uart_platform_driver);

	/* Unregister the x2_uart driver */
	uart_unregister_driver(&x2_uart_driver);
}

module_init(x2_uart_init);
module_exit(x2_uart_exit);

MODULE_DESCRIPTION("Driver for X2 UART");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
