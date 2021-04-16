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

#ifndef __HOBOT_DMA_H__
#define __HOBOT_DMA_H__

/* the offset of dma registers */
#define HOBOT_DMA_REG_ADDR_BASE     0xA1005000
#define HOBOT_DMA_REG_ADDR_LEN      0x1000
#define HOBOT_DMA_SOFT_REQ          0x0004  /* RW: Software Request Register */
#define HOBOT_DMA_CHANNEL_STATUS    0x0008  /* RO: Channels Status Register */
#define HOBOT_DMA_SRCPND            0x000c  /* WC: Interrupt Source Pending Register */
#define HOBOT_DMA_INTMASK           0x0010  /* RO: Interrupt Mask Register */
#define HOBOT_DMA_INT_SETMASK       0x0014  /* WO: Interrupt Setmask Register */
#define HOBOT_DMA_INT_UNMASK        0x0018  /* WO: Interrupt Unmask Register */
#define HOBOT_DMA_FIFO_RST          0x001c  /* WO: FIFO Reset Register */
#define HOBOT_DMA_SRC_ADDR          0x0100  /* RW: Request Source Address Register */
#define HOBOT_DMA_DEST_ADDR         0x0104  /* RW: Request Destination Address Register */
#define HOBOT_DMA_CTRL_ADDR         0x0108  /* RW: Request Control Register */
#define HOBOT_DMA_LLI_ADDR          0x010c  /* RW: Request Linked List Item Register */
#define HOBOT_DMA_ERR_STATE         0x0200  /* RO: Error State Register */
#define HOBOT_DMA_CURR_SRC_ADDR     0x0f00  /* RO: Current Transfer Source Address In LLI Mode */
#define HOBOT_DMA_CURR_DEST_ADDR    0x0f04  /* RO: Current Transfer Destination Address in LLI Mode */
#define HOBOT_DMA_CURR_TL_ADDR      0x0f08  /* RO: Current Transfer Length In LLI Mode */
#define HOBOT_DMA_CURR_LLI_ADDR     0x0f0c  /* RO: Nest LLI address In LLI Mode */

/* DMA control and status BIT define */
#define HOBOT_DMA_START_TX          (1U<<00)
#define HOBOT_DMA_RST_FIFO          (1U<<00)
#define HOBOT_DMA_EN_LLI            (1U<<25)
#define HOBOT_DMA_EN_CH             (1U<<24)

#define HOBOT_DMA_CH_BUSY           (1U<<00)
#define HOBOT_DMA_WR_SLVERR         (2U<<00)
#define HOBOT_DMA_WR_DECERR         (3U<<00)
#define HOBOT_DMA_RD_SLVERR         (2U<<02)
#define HOBOT_DMA_RD_DECERR         (3U<<02)

/* DMA interrupt flag BIT */
#define HOBOT_DMA_TXCMP             (1U<<00)
#define HOBOT_DMA_TXERR             (1U<<01)
#define HOBOT_DMA_CHTERM            (1U<<02)

/* DMA buffer length */
#define HOBOT_DMA_TXBUF_LEN         0x1000
#define HOBOT_DMA_RXBUF_LEN         0x1000
#define HOBOT_DMA_TX_PADDR          0x7D000000
#define HOBOT_DMA_RX_PADDR          0x7E000000


/*Debug switch*/
//#define HOBOT_DMA_DEBUG_ON
#ifdef HOBOT_DMA_DEBUG_ON
#define HOBOT_DMA_DEBUG(fmt,args...) printk(KERN_ERR"[%s_%d]"fmt, __FUNCTION__, __LINE__, ##args)
#else
#define HOBOT_DMA_DEBUG(fmt,args...)
#endif

#endif
