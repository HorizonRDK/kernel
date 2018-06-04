#ifndef __X2_SERIAL_H__
#define __X2_SERIAL_H__

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
#define UART_ENR_EN 		(1U << 0)	/* uart gloabl enabled bit */
#define UART_ENR_RX_EN		(1U << 1)		/* RX enabled bit */
#define UART_ENR_TX_EN		(1U << 2)		/* TX enabled bit */

/* Uart BAUD Rate Register Bits define */
#define UART_BCR_BRDIV_INT_MASK (0xFFFF)
#define UART_BCR_BRDIV_INT(x) (((x) & UART_BCR_BRDIV_INT_MASK) << 0U)
#define UART_BCR_BRDIV_FRAC_MASK (0x3FF)
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
#define UART_MCR_LMD_MASK		(0x3)
#define UART_MCR_LMD(x)		(((x) & X2_MCR_LMD_MASK) << 2U)
#define UART_MCR_XON_MASK		(0xFF)
#define UART_MCR_XON(x)		(((x) & X2_MCR_XON_MASK) << 8U)
#define UART_MCR_XOFF_MASK		(0xFF)
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

#define UART_IRQ_SRC_MASK	(0x7FFF)

#endif /* __X2_SERIAL_H__ */
