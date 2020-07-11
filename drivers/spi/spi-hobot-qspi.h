/* SPDX-License-Identifier: GPL-2.0+
 *
 * Hobot QSPI driver
 *
 * Copyright (C) 2019, Horizon Robotics
 */

#ifndef __LINUX_HOBOT_QSPI_H__
#define __LINUX_HOBOT_QSPI_H__

#define HB_QSPI_NAME                "hb_qspi"

/* Uncomment the following macro definition to use polling in batch mode */
// #define HB_QSPI_WORK_POLL	1

/* The following macro definition to turn on debugging */
/* 1: tracks transfer failure, 2: tracks all transfer */
#define QSPI_DEBUG		0

#ifdef CONFIG_HOBOT_FPGA_X3
#define	CONFIG_HOBOT_QSPI_REF_CLK 10000000
#define	CONFIG_HOBOT_QSPI_CLK 5000000
#endif

#define HB_QSPI_CS(cs, enable) (BIT(cs) & enable)
#define HB_QSPI_CS_EN 0
#define HB_QSPI_CS_DIS 1

#define HB_QSPI_MAX_CS          1
#define HB_QSPI_DEF_BDR         50000000
#define HB_QSPI_TIMEOUT_MS      1
#define HB_QSPI_TIMEOUT_US      (1000 * 1000)
/* HB QSPI register offsets */
#define HB_QSPI_DAT_REG        0x00   /* Transmit data buffer and receive data buffer */
#define HB_QSPI_BDR_REG        0x04   /* Baud-rate control while working as master */
#define HB_QSPI_CTL1_REG       0x08   /* SPI work mode configuration */
#define HB_QSPI_CTL2_REG       0x0C   /* SPI interrupt enabled */
#define HB_QSPI_CTL3_REG       0x10   /* SPI software reset and DMA enable */
#define HB_QSPI_CS_REG         0x14   /* Control the device select output */
#define HB_QSPI_ST1_REG        0x18   /* spi status reg1 */
#define HB_QSPI_ST2_REG        0x1C   /* spi status reg2 */
#define HB_QSPI_RBC_REG        0x20   /* Data number for RX batch transfer */
#define HB_QSPI_TBC_REG        0x24   /* Data number for TX batch transfer */
#define HB_QSPI_RTL_REG        0x28   /* RX FIFO trigger level register */
#define HB_QSPI_TTL_REG        0x2C   /* TX FIFO trigger level register */
#define HB_QSPI_DQM_REG        0x30   /* Dual, Quad flash operation enable register */
#define HB_QSPI_XIP_REG        0x34   /* XIP config register */

/* Definition of SPI_CTRL1 */
#define HB_QSPI_FW_MASK         0x03   /* FIFO WIDTH MASK */
#define HB_QSPI_FW8             0x00   /* FIFO WIDTH 8Bit */
#define HB_QSPI_FW16            0x01   /* FIFO WIDTH 16Bit */
#define HB_QSPI_FW32            0x02   /* FIFO WIDTH 32Bit */
#define HB_QSPI_MST             0x04   /* Working master mode */
#define	HB_QSPI_SLV             0x00   /* Working slave mode */
#define	LSB                     0x40   /* LSB transferred first on SPI bus */
#define	MSB                     0x00   /* MSB transferred first on SPI bus */
#define HB_QSPI_RX_EN           0x80   /* Indicates the Rx direction is working */
#define	HB_QSPI_TX_EN           0x08   /* Indicates the Tx direction is working */
#define	HB_QSPI_TX_DIS          ~HB_QSPI_TX_EN
#define	HB_QSPI_RX_DIS          ~HB_QSPI_RX_EN
#define HB_QSPI_CPOL            0x10
#define HB_QSPI_CPHA            0x20
#define HB_QSPI_MODE0           0x0
#define HB_QSPI_MODE1           HB_QSPI_CPHA
#define HB_QSPI_MODE2           HB_QSPI_CPOL
#define HB_QSPI_MODE3           (HB_QSPI_CPOL | HB_QSPI_CPHA)

/* Definition of SPI_CTRL2 */
#define HB_QSPI_RX_INT          BIT(0) /* Enable the interrupt of RX */
#define HB_QSPI_TX_INT          BIT(1) /* Enable the interrupt of TX */
#define HB_QSPI_ERR_INT         BIT(2) /* Enable spi_wr_full or spi_rd_ept error interrupt */
#define HB_QSPI_MODF_INT        BIT(3) /* Enable the mode fault detecting in ssn0_i */
#define HB_QSPI_RBC_INT         BIT(6) /* Enable spi rx batch complete interrupt */
#define HB_QSPI_TBC_INT         BIT(7) /* Enable spi tx batch complete interrupt */

/* Definition of SPI_CTRL3 */
#define HB_QSPI_RX_RST          BIT(0) /* soft reset for RX fifo, high active */
#define HB_QSPI_TX_RST          BIT(1) /* soft reset for TX fifo, high active */
#define HB_QSPI_RST_FIFO        0x3    /* soft reset rx & tx fifo */
#define HB_QSPI_BATCH_DIS       BIT(4) /* Disable batch_cnt and batch operation */

/* Definition of SPI_STATUS1 */
#define HB_QSPI_RX_AF           BIT(0) /* Rx almost full */
#define HB_QSPI_TX_AE           BIT(1) /* Tx almost empty */
#define HB_QSPI_MODF_CLR        BIT(3) /* Work mode if fault */
#define HB_QSPI_RBD             BIT(4) /* Rx batch done */
#define HB_QSPI_TBD             BIT(5) /* Tx batch done */

/* Definition of SPI_STATUS2 */
#define HB_QSPI_RX_FULL         BIT(0)
#define HB_QSPI_TX_EP           BIT(1)
#define HB_QSPI_RXWR_FULL       BIT(2)
#define HB_QSPI_TXRD_EMPTY      BIT(3)
#define HB_QSPI_RX_EP           BIT(4)
#define HB_QSPI_TX_FULL         BIT(5)
#define HB_QSPI_SSN_IN          BIT(7)

/* Definition of RX/TX trig level */
#define HB_QSPI_FIFO_DEPTH      16
#define HB_QSPI_TRIG_LEVEL      (HB_QSPI_FIFO_DEPTH/2)

/* Definition of DUAL_QUAD_MODE */
#define HB_QSPI_DQM_DEFAULT     0x48
#define HB_QSPI_SING            0x0
#define HB_QSPI_DUAL_EN       BIT(0)
#define HB_QSPI_QUAD_EN       BIT(1)
#define	HB_QSPI_WP_CTL          BIT(2)
#define	HB_QSPI_WP_OE           BIT(3)
#define	HB_QSPI_WP_OUTPUT       BIT(4)
#define	HB_QSPI_HOLD_CTL        BIT(5)
#define	HB_QSPI_HOLD_OE         BIT(6)
#define	HB_QSPI_HOLD_OUTPUT     BIT(7)

/* xfer mode */
#define HB_QSPI_XFER_BYTE       0x0
#define HB_QSPI_XFER_BATCH      BIT(0)
#define HB_QSPI_XFER_DMA        BIT(1)
#define	HB_RX_DMA_EN            BIT(2)
#define	HB_TX_DMA_EN            BIT(3)

/* Read commands */
#define CMD_QUAD_PAGE_PROGRAM       0x32
#define CMD_READ_DUAL_OUTPUT_FAST   0x3b
#define CMD_READ_QUAD_OUTPUT_FAST   0x6b

/* SPI_XIP_CFG */
#define	XIP_EN				    (0x1L<<0)
#define	FLASH_SW_CFG			(0x0L<<1)
#define	FLASH_HW_CFG			(0x1L<<1)
#define	FLASH_CTNU_MODE			(0x1<<4)
#define	DUMMY_CYCLE_0			(0x0<<5)
#define	DUMMY_CYCLE_1			(0x1<<5)
#define	DUMMY_CYCLE_2			(0x2<<5)
#define	DUMMY_CYCLE_3			(0x3<<5)
#define	DUMMY_CYCLE_4			(0x4<<5)
#define	DUMMY_CYCLE_5			(0x5<<5)
#define	DUMMY_CYCLE_6			(0x6<<5)
#define	DUMMY_CYCLE_7			(0x7<<5)
#define	XIP_FLASH_ADDR_OFFSET   0x100

/* Some op macro */
#define HB_QSPI_OP_RX_EN        0x01
#define HB_QSPI_OP_RX_DIS       0x02
#define HB_QSPI_OP_TX_EN        0x03
#define HB_QSPI_OP_TX_DIS       0x04
#define HB_QSPI_OP_BAT_EN       0x05
#define HB_QSPI_OP_BAT_DIS      0x06

#define HB_QSPI_XFER_BEGIN      BIT(0) /* Assert CS before transfer */
#define HB_QSPI_XFER_END        BIT(1) /* Deassert CS after transfer */
#define HB_QSPI_XFER_CMD        BIT(4) /* Transfer data is CMD */

#define HB_QSPI_BUS_SINGLE      0x01   /* Single(Standard) I/O data transfer */
#define HB_QSPI_BUS_DUAL        0x02   /* Dual I/O data transfer */
#define HB_QSPI_BUS_QUAD        0x04   /* Quad I/O data transfer */

#define TRYS_TOTAL_NUM          0x10000
#define BATCH_MAX_CNT           0x10000
#define	SPI_BACKUP_OFFSET       (0x10000)	//64K
/* Macro Functions */
#define	SCLK_DIV(div, scaler)		((2 << (div)) * (scaler + 1))
#define	SCLK_VAL(div, scaler)		(((div) << 4) | ((scaler) << 0))
#define MIN(a, b)   ((a < b) ? (a) : (b))
#define MAX(a, b)   ((a > b) ? (a) : (b))

#endif
