// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (c) 2020-21
 *
 */
#ifndef INCLUDE_SOC_HOBOT_CR5_SERVICE_H_
#define INCLUDE_SOC_HOBOT_CR5_SERVICE_H_


/* ipi interrupt register */
#define IPI_APU_TRIG_REG	(0x00)
#define IPI_APU_OBS_REG		(0x04)
#define IPI_APU_ISR_REG		(0x08)
#define IPI_APU_IMR_REG		(0x0C)
#define IPI_APU_IER_REG		(0x10)
#define IPI_APU_IDR_REG		(0x14)

/* ipi message buffer register */
#define IPI_APU_REQ_BUFF(n)		(0x400 + n * 0x20)
//#define IPI_APU_RESP_BUFF(n)	(0x500 + n * 0x20)

//#define IPI_RPU_REQ_BUFF(n)		(IPI_RPU_REQ_MSG_BASE + n * 0x20)
#define IPI_RPU_RESP_BUFF(n)	(0x700 + n * 0x20)


/* define RX, TX, and they should be sync. with x2a_ipi.h  */
#define MBOX_TX       1
#define MBOX_RX       2
/* IRQ CH, don't modify */
#define MBOX_IRQ_APU 	1
#define MBOX_IRQ_RPU 	2


#define IPI_CMDBUF_FULLUSE		0xFF

#define GET_IPI_IRQ_MSK(n)	((n == MBOX_RX) ? MBOX_TX : MBOX_RX)

#endif // INCLUDE_SOC_HOBOT_CR5_SERVICE_H_
