/*
 * Horizon X2 QuadSPI driver.
 *
 * Copyright (C) 2018 Horizon, Inc.
 *
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
#include <x2/diag.h>
#include "x2-qspi.h"

static int first_time;
static int last_err;

/* #define X2_QSPI_WORK_POLL    1 */

struct x2qspi_pdata;
struct x2qspi_flash_pdata {
	struct spi_nor nor;
	struct x2qspi_pdata *x2qspi;
	u32 clk_rate;
	u8  cs;
	bool registered;
};

/**
 * struct x2_qspi_platdata - platform data for x2 QSPI
 *
 * @regs: Point to the base address of QSPI registers
 * @freq: QSPI input clk frequency
 * @speed_hz: Default BUS sck frequency
 * @xfer_mode: 0-Byte 1-batch 2-dma, Default work in byte mode
 */
struct x2qspi_pdata {
	int irq;
	u32 ref_clk;
	u32 cur_cs;
	void __iomem *iobase;
	struct clk *clk;
	struct mutex lock;
	struct platform_device *pdev;
	struct completion xfer_complete;
	struct x2qspi_flash_pdata f_pdata[X2_QSPI_MAX_CS];
};


#define x2qspi_rd(dev, reg)       ioread32((dev)->iobase + (reg))
#define x2qspi_wr(dev, reg, val)  iowrite32((val), (dev)->iobase + (reg))

static void x2_qspi_set_speed(struct spi_nor *nor)
{
	int confr, prescaler, divisor;
	unsigned int max_br, min_br, br_div;
	struct x2qspi_flash_pdata *f_pdata = (struct x2qspi_flash_pdata *)nor->priv;
	struct x2qspi_pdata *x2qspi   = f_pdata->x2qspi;

	if (x2qspi->cur_cs == f_pdata->cs)
		return;
	x2qspi->cur_cs = f_pdata->cs;

	max_br = x2qspi->ref_clk / 2;
	min_br = x2qspi->ref_clk / 1048576;
	if (f_pdata->clk_rate > max_br) {
		f_pdata->clk_rate = max_br;
		pr_err("Warning:speed[%d] > max_br[%d],speed will be set to max_br\n", f_pdata->clk_rate, max_br);
	}
	if (f_pdata->clk_rate < min_br) {
		f_pdata->clk_rate = min_br;
		pr_err("Warning:speed[%d] < min_br[%d],speed will be set to min_br\n", f_pdata->clk_rate, min_br);
	}

	for (prescaler = 15; prescaler >= 0; prescaler--) {
		for (divisor = 15; divisor >= 0; divisor--) {
			br_div = (prescaler + 1) * (2 << divisor);
			if ((x2qspi->ref_clk / br_div) >= f_pdata->clk_rate) {
				confr = (prescaler | (divisor << 4)) & 0xFF;
				x2qspi_wr(x2qspi, X2_QSPI_BDR_REG, confr);
				return;
			}
		}
	}

	return;
}

/* currend driver only support BYTE/DUAL/QUAD for both RX and TX */
static int x2_qspi_set_wire(struct x2qspi_pdata *x2qspi, uint mode)
{
	unsigned int val = 0;

	switch (mode) {
	case X2_QSPI_BUS_DUAL:
		val = X2_QSPI_DUAL;
		break;
	case X2_QSPI_BUS_QUAD:
		val = X2_QSPI_QUAD;
		break;
	case X2_QSPI_BUS_SINGLE:
	default:
		val = 0xFC;
		break;
	}
	x2qspi_wr(x2qspi, X2_QSPI_DQM_REG, val);

	return 0;
}


static void x2_qspi_reset_fifo(struct x2qspi_pdata *x2qspi)
{
	u32 val;

	val = x2qspi_rd(x2qspi, X2_QSPI_CTL3_REG);
	val |= X2_QSPI_RST_ALL;
	x2qspi_wr(x2qspi, X2_QSPI_CTL3_REG, val);
	mdelay(1);
	val = (~X2_QSPI_RST_ALL);
	x2qspi_wr(x2qspi, X2_QSPI_CTL3_REG, val);

	return;
}

/* tx almost empty */
static int x2_qspi_tx_ae(struct x2qspi_pdata *x2qspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = x2qspi_rd(x2qspi, X2_QSPI_ST1_REG);
		trys ++;
	} while ((!(val&X2_QSPI_TX_AE)) && (trys<TRYS_TOTAL_NUM));
	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys<TRYS_TOTAL_NUM ? 0 : -1;
}

/* rx almost full */
static int x2_qspi_rx_af(struct x2qspi_pdata *x2qspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = x2qspi_rd(x2qspi, X2_QSPI_ST1_REG);
		trys ++;
	} while ((!(val&X2_QSPI_RX_AF)) && (trys<TRYS_TOTAL_NUM));
	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys<TRYS_TOTAL_NUM ? 0 : -1;
}

static int __maybe_unused x2_qspi_tb_done(struct x2qspi_pdata *x2qspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = x2qspi_rd(x2qspi, X2_QSPI_ST1_REG);
		trys ++;
	} while ((!(val&X2_QSPI_TBD)) && (trys<TRYS_TOTAL_NUM));
	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);
	x2qspi_wr(x2qspi, X2_QSPI_ST1_REG, (val|X2_QSPI_TBD));


	return trys<TRYS_TOTAL_NUM ? 0 : -1;
}

static int __maybe_unused x2_qspi_rb_done(struct x2qspi_pdata *x2qspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = x2qspi_rd(x2qspi, X2_QSPI_ST1_REG);
		trys ++;
	} while ((!(val&X2_QSPI_RBD)) && (trys<TRYS_TOTAL_NUM));
	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);
	x2qspi_wr(x2qspi, X2_QSPI_ST1_REG, (val|X2_QSPI_RBD));


	return trys<TRYS_TOTAL_NUM ? 0 : -1;
}

static int x2_qspi_tx_full(struct x2qspi_pdata *x2qspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = x2qspi_rd(x2qspi, X2_QSPI_ST2_REG);
		trys ++;
	} while ((val&X2_QSPI_TX_FULL) && (trys<TRYS_TOTAL_NUM));

	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys<TRYS_TOTAL_NUM ? 0 : -1;
}

static int x2_qspi_tx_empty(struct x2qspi_pdata *x2qspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = x2qspi_rd(x2qspi, X2_QSPI_ST2_REG);
		trys ++;
	} while ((!(val&X2_QSPI_TX_EP)) && (trys<TRYS_TOTAL_NUM));

	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys<TRYS_TOTAL_NUM ? 0 : -1;
}

static int x2_qspi_rx_empty(struct x2qspi_pdata *x2qspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = x2qspi_rd(x2qspi, X2_QSPI_ST2_REG);
		trys ++;
	} while ((val&X2_QSPI_RX_EP) && (trys<TRYS_TOTAL_NUM));

	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys<TRYS_TOTAL_NUM ? 0 : -1;
}

/* config fifo width */
static void x2_qspi_set_fw(struct x2qspi_pdata *x2qspi, u32 fifo_width)
{
	u32 val;

	val = x2qspi_rd(x2qspi, X2_QSPI_CTL1_REG);
	val &= (~0x3);
	val |= fifo_width;
	x2qspi_wr(x2qspi, X2_QSPI_CTL1_REG, val);

	return;
}

/*config xfer mode:enable/disable BATCH/RX/TX */
static void x2_qspi_set_xfer(struct x2qspi_pdata *x2qspi, u32 op_flag)
{
	u32 ctl1_val = 0, ctl3_val = 0;

	ctl1_val = x2qspi_rd(x2qspi, X2_QSPI_CTL1_REG);
	ctl3_val = x2qspi_rd(x2qspi, X2_QSPI_CTL3_REG);

	switch (op_flag) {
	case X2_QSPI_OP_RX_EN:
		ctl1_val |= X2_QSPI_RX_EN;
		break;
	case X2_QSPI_OP_RX_DIS:
		ctl1_val &= (~X2_QSPI_RX_EN);
		break;
	case X2_QSPI_OP_TX_EN:
		ctl1_val |= X2_QSPI_TX_EN;
		break;
	case X2_QSPI_OP_TX_DIS:
		ctl1_val &= (~X2_QSPI_TX_EN);
		break;
	case X2_QSPI_OP_BAT_EN:
		ctl3_val &= (~X2_QSPI_BATCH_DIS);
		break;
	case X2_QSPI_OP_BAT_DIS:
		ctl3_val |= X2_QSPI_BATCH_DIS;
		break;
	default:
		pr_err("Op(0x%x) if error, please check it!\n", op_flag);
		break;
	}
	x2qspi_wr(x2qspi, X2_QSPI_CTL1_REG, ctl1_val);
	x2qspi_wr(x2qspi, X2_QSPI_CTL3_REG, ctl3_val);

	return;
}

static int x2qspi_rd_batch(struct x2qspi_pdata *x2qspi, void *pbuf, uint32_t len)
{
	u32 i, rx_len, offset = 0, tmp_len = len, ret = 0;
	u32 *dbuf = (u32 *) pbuf;

	/* Enable batch mode */
	x2_qspi_set_fw(x2qspi, X2_QSPI_FW32);
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_BAT_EN);

	while (tmp_len > 0) {
		reinit_completion(&x2qspi->xfer_complete);
		rx_len = MIN(tmp_len, BATCH_MAX_CNT);
		x2qspi_wr(x2qspi, X2_QSPI_RBC_REG, rx_len);
		/* enbale rx */
		x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_RX_EN);

		for (i=0; i< rx_len; i+=8) {
			if (x2_qspi_rx_af(x2qspi)) {
				ret = -1;
				goto rb_err;
			}
			dbuf[offset++] = x2qspi_rd(x2qspi, X2_QSPI_DAT_REG);
			dbuf[offset++] = x2qspi_rd(x2qspi, X2_QSPI_DAT_REG);
		}
#ifdef X2_QSPI_WORK_POLL
		if (x2_qspi_rb_done(x2qspi)) {
			pr_err("%s_%d:rx failed! len=%d, received=%d, i=%d\n", __func__, __LINE__, len, offset, i);
			ret = -EIO;
			goto rb_err;
		}
#else
		ret = wait_for_completion_timeout(&x2qspi->xfer_complete, msecs_to_jiffies(X2_QSPI_TIMEOUT_MS));
		if (!ret) {
			pr_err("%s_%d:rx failed! len=%d, received=%d, i=%d\n", __func__, __LINE__, len, offset, i);
			ret = -EIO;
			goto rb_err;
		}
		ret = 0;
#endif
		x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_RX_DIS);
		tmp_len = tmp_len - rx_len;
	}

rb_err:
	/* Disable batch mode and rx link */
	x2_qspi_set_fw(x2qspi, X2_QSPI_FW8);
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_BAT_DIS);
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_RX_DIS);

	return ret;
}

static int x2qspi_wr_batch(struct x2qspi_pdata *x2qspi, const void *pbuf, uint32_t len)
{
	u32 i, tx_len, offset = 0, tmp_len = len, ret = 0;
	u32 *dbuf = (u32 *) pbuf;

	x2_qspi_set_fw(x2qspi, X2_QSPI_FW32);
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_BAT_EN);

	while (tmp_len > 0) {
		reinit_completion(&x2qspi->xfer_complete);
		tx_len = MIN(tmp_len, BATCH_MAX_CNT);
		x2qspi_wr(x2qspi, X2_QSPI_TBC_REG, tx_len);

		/* enbale tx */
		x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_TX_EN);

		for (i=0; i<tx_len; i+=8) {
			if (x2_qspi_tx_ae(x2qspi)) {
				ret = -1;
				goto tb_err;
			}
			x2qspi_wr(x2qspi, X2_QSPI_DAT_REG, dbuf[offset++]);
			x2qspi_wr(x2qspi, X2_QSPI_DAT_REG, dbuf[offset++]);
		}
#ifdef X2_QSPI_WORK_POLL
		if (x2_qspi_tb_done(x2qspi)) {
			pr_err("%s_%d:tx failed! len=%d, received=%d, i=%d\n", __func__, __LINE__, len, offset, i);
			ret = -EIO;
			goto tb_err;
		}
#else
		ret = wait_for_completion_timeout(&x2qspi->xfer_complete, msecs_to_jiffies(X2_QSPI_TIMEOUT_MS));
		if (!ret) {
			pr_err("%s_%d:rx failed! len=%d, received=%d, i=%d\n", __func__, __LINE__, len, offset, i);
			ret = -EIO;
			goto tb_err;
		}
		ret = 0;
#endif
		tmp_len = tmp_len - tx_len;
	}

tb_err:
	/* Disable batch mode and tx link */
	x2_qspi_set_fw(x2qspi, X2_QSPI_FW8);
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_BAT_DIS);
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_TX_DIS);

	return ret;
}

static int x2qspi_rd_byte(struct x2qspi_pdata *x2qspi, void *pbuf, uint32_t len)
{
	u32 i, ret = 0;
	u8 *dbuf = (u8 *) pbuf;

	/* enbale rx */
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_RX_EN);

	for (i=0; i<len; i++)
	{
		if (x2_qspi_tx_empty(x2qspi)) {
			ret = -1;
			goto rd_err;
		}
		x2qspi_wr(x2qspi, X2_QSPI_DAT_REG, 0x00);
		if (x2_qspi_rx_empty(x2qspi)) {
			ret = -1;
			goto rd_err;
		}
		dbuf[i] = x2qspi_rd(x2qspi, X2_QSPI_DAT_REG) & 0xFF;
	}

rd_err:
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_RX_DIS);

	if (0 != ret)
		pr_err("%s_%d:read op failed! i=%d\n", __func__, __LINE__, i);

	return ret;
}

static int x2qspi_wr_byte(struct x2qspi_pdata *x2qspi, const void *pbuf, uint32_t len)
{
	u32 i, ret = 0;
	u8 *dbuf = (u8 *) pbuf;

	/* enbale tx */
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_TX_EN);

	for (i=0; i<len; i++)
	{
		if (x2_qspi_tx_full(x2qspi)) {
			ret = -1;
			goto wr_err;
		}
		x2qspi_wr(x2qspi, X2_QSPI_DAT_REG, dbuf[i]);
	}
	/* Check tx complete */
	if (x2_qspi_tx_empty(x2qspi))
		ret = -1;

wr_err:
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_TX_DIS);

	if (0 != ret)
		pr_err("%s_%d:write op failed! i=%d\n", __func__, __LINE__, i);

	return ret;
}

static int x2_qspi_read(struct x2qspi_pdata *x2qspi, void *pbuf, uint32_t len)
{
	u32 ret = 0;
	u32 remainder = len % X2_QSPI_TRIG_LEVEL;
	u32 residue   = len - remainder;

	if (residue > 0)
		ret = x2qspi_rd_batch(x2qspi, pbuf, residue);
	if (remainder > 0)
		ret = x2qspi_rd_byte(x2qspi, (u8 *) pbuf + residue, remainder);
	if (ret < 0)
		pr_err("x2_qspi_read failed!\n");

	return ret;
}

static int x2_qspi_write(struct x2qspi_pdata *x2qspi, const void *pbuf, uint32_t len)
{
	u32 ret = 0;
	u32 remainder = len % X2_QSPI_TRIG_LEVEL;
	u32 residue   = len - remainder;

	if (residue > 0)
		ret = x2qspi_wr_batch(x2qspi, pbuf, residue);
	if (remainder > 0)
		ret = x2qspi_wr_byte(x2qspi, (u8 *) pbuf + residue, remainder);
	if (ret < 0)
		pr_err("x2qspi_write failed!\n");

	return ret;
}

int x2_qspi_xfer(struct spi_nor *nor, unsigned int len, const void *dout,
		 void *din, unsigned long flags)
{
	int ret = 0;
	struct x2qspi_flash_pdata *f_pdata = (struct x2qspi_flash_pdata *)nor->priv;
	struct x2qspi_pdata *x2qspi   = f_pdata->x2qspi;

	if (len == 0) {
		return 0;
	}

	if (flags & X2_QSPI_XFER_BEGIN)
		x2qspi_wr(x2qspi, X2_QSPI_CS_REG, 1<<f_pdata->cs);  /* Assert CS before transfer */

	if (dout) {
		ret = x2_qspi_write(x2qspi, dout, len);
	}
	else if (din) {
		ret = x2_qspi_read(x2qspi, din, len);
	}

	if (flags & X2_QSPI_XFER_END) {
		x2qspi_wr(x2qspi, X2_QSPI_CS_REG, 0);  /* Deassert CS after transfer */
	}

	if (flags & X2_QSPI_XFER_CMD) {
		switch (((u8 *) dout) [0]) {
		case CMD_READ_QUAD_OUTPUT_FAST:
		case CMD_QUAD_PAGE_PROGRAM:
			x2_qspi_set_wire(x2qspi, X2_QSPI_BUS_QUAD);
			break;
		case CMD_READ_DUAL_OUTPUT_FAST:
			x2_qspi_set_wire(x2qspi, X2_QSPI_BUS_DUAL);
			break;
		default:
			x2_qspi_set_wire(x2qspi, X2_QSPI_BUS_SINGLE);
			break;
		}
	} else {
		x2_qspi_set_wire(x2qspi, X2_QSPI_BUS_SINGLE);
	}

	return ret;
}

static int x2_qspi_flash_rd_wr(struct spi_nor *nor, const u8 *cmd, size_t cmd_len,
				const u8 *data_out, u8 *data_in, size_t data_len)
{
	int ret;
	unsigned long flags = X2_QSPI_XFER_BEGIN;

	x2_qspi_set_speed(nor);

	if (data_len == 0)
		flags |= X2_QSPI_XFER_END;

	ret = x2_qspi_xfer(nor, cmd_len, cmd, NULL, flags|X2_QSPI_XFER_CMD);
	if (ret) {
		pr_err("SF: Failed to send command (%d bytes): %d\n", (int)cmd_len, ret);
	} else if (data_len != 0) {
		ret = x2_qspi_xfer(nor, data_len, data_out, data_in, X2_QSPI_XFER_END);
		if (ret)
			pr_err("SF: Failed to transfer %d bytes of data: %d\n", (int)data_len, ret);
	}

	return ret;
}

static void x2_qspi_flash_addr(struct spi_nor *nor, u32 addr, u8 *cmd)
{
        /* cmd[0] is actual command */
        if (nor->mtd.size > SPI_FLASH_16MB_BOUN) {
                cmd[1] = addr >> 24;
                cmd[2] = addr >> 16;
                cmd[3] = addr >> 8;
                cmd[4] = addr >> 0;
        } else {
                cmd[1] = addr >> 16;
                cmd[2] = addr >> 8;
                cmd[3] = addr >> 0;
        }
}

static int x2_qspi_flash_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct x2qspi_flash_pdata *f_pdata = (struct x2qspi_flash_pdata *)nor->priv;
	struct x2qspi_pdata *x2qspi   = f_pdata->x2qspi;

	mutex_lock(&x2qspi->lock);

	return 0;
}

static void x2_qspi_flash_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct x2qspi_flash_pdata *f_pdata = (struct x2qspi_flash_pdata *)nor->priv;
	struct x2qspi_pdata *x2qspi   = f_pdata->x2qspi;

	mutex_unlock(&x2qspi->lock);
}

static int x2_qspi_flash_rd_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	return x2_qspi_flash_rd_wr(nor, &opcode, 1, NULL, buf, len);
}

static int x2_qspi_flash_wr_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	return x2_qspi_flash_rd_wr(nor, &opcode, 1, buf, NULL, len);
}

static ssize_t x2_qspi_flash_read(struct spi_nor *nor, loff_t from,
				size_t len, u_char *read_buf)
{
	u8 cmdsz;

        if (nor->mtd.size > SPI_FLASH_16MB_BOUN)
		cmdsz = SPI_FLASH_CMD_LEN + nor->read_dummy/8;
	else
		cmdsz = SPI_FLASH_CMD_LEN - 1 + nor->read_dummy/8;

	nor->cmd_buf[0] = nor->read_opcode;
	x2_qspi_flash_addr(nor, from, nor->cmd_buf);

	if (x2_qspi_flash_rd_wr(nor, nor->cmd_buf, cmdsz, NULL, read_buf, len))
		return -EIO;

	return len;
}

static ssize_t x2_qspi_flash_write(struct spi_nor *nor, loff_t to,
				size_t len, const u_char *write_buf)
{
	u8 cmdsz = SPI_FLASH_CMD_LEN;

        if (nor->mtd.size > SPI_FLASH_16MB_BOUN)
		cmdsz = SPI_FLASH_CMD_LEN;
	else
		cmdsz = SPI_FLASH_CMD_LEN - 1;

	nor->cmd_buf[0] = nor->program_opcode;
	x2_qspi_flash_addr(nor, to, nor->cmd_buf);

	if (x2_qspi_flash_rd_wr(nor, nor->cmd_buf, cmdsz, write_buf, NULL, len))
		return -EIO;

	return len;
}

static int x2_qspi_flash_erase(struct spi_nor *nor, loff_t offs)
{
	u8 cmdsz = SPI_FLASH_CMD_LEN;

        if (nor->mtd.size > SPI_FLASH_16MB_BOUN)
		cmdsz = SPI_FLASH_CMD_LEN;
	else
		cmdsz = SPI_FLASH_CMD_LEN - 1;

	nor->cmd_buf[0] = nor->erase_opcode;
	x2_qspi_flash_addr(nor, offs, nor->cmd_buf);

	return x2_qspi_flash_rd_wr(nor, nor->cmd_buf, cmdsz, NULL, NULL, 0);
}

static void qspinorflash_diag_report(uint8_t errsta, uint32_t sta_reg)
{
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_norflash,
				EventIdNorflashErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&sta_reg,
				4);
	} else {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_norflash,
				EventIdNorflashErr,
				DiagEventStaSuccess);
	}
}

static void qspi_callback(void *p, size_t len)
{
	first_time = 0;
}

static irqreturn_t x2_qspi_irq_handler(int irq, void *dev_id)
{
	unsigned int irq_status;
	unsigned int err_status;
	int err = 0;
	struct x2qspi_pdata *x2qspi = dev_id;

	/* Read interrupt status */
	irq_status = x2qspi_rd(x2qspi, X2_QSPI_ST1_REG);
	x2qspi_wr(x2qspi, X2_QSPI_ST1_REG, X2_QSPI_TBD | X2_QSPI_RBD);

	err_status = x2qspi_rd(x2qspi, X2_QSPI_ST2_REG);
	x2qspi_wr(x2qspi, X2_QSPI_ST2_REG,
			X2_QSPI_RXWR_FULL | X2_QSPI_TXRD_EMPTY);

	if (irq_status & (X2_QSPI_TBD | X2_QSPI_RBD))
		complete(&x2qspi->xfer_complete);

	if (err_status & (X2_QSPI_RXWR_FULL | X2_QSPI_TXRD_EMPTY))
		err = 1;

	if (first_time == 0) {
		first_time = 1;
		last_err = err;
		qspinorflash_diag_report(err, err_status);
	} else if (last_err != err) {
		last_err = err;
		qspinorflash_diag_report(err, err_status);
	}
	return IRQ_HANDLED;
}

static void x2_qspi_hw_init(struct x2qspi_pdata *x2qspi)
{
	uint32_t val;

	/* disable batch operation and reset fifo */
	val = x2qspi_rd(x2qspi, X2_QSPI_CTL3_REG);
	val |= X2_QSPI_BATCH_DIS;
	x2qspi_wr(x2qspi, X2_QSPI_CTL3_REG, val);
	x2_qspi_reset_fifo(x2qspi);

	/* clear status */
	val = X2_QSPI_MODF | X2_QSPI_RBD | X2_QSPI_TBD;
	x2qspi_wr(x2qspi, X2_QSPI_ST1_REG, val);
	val = X2_QSPI_TXRD_EMPTY | X2_QSPI_RXWR_FULL;
	x2qspi_wr(x2qspi, X2_QSPI_ST2_REG, val);

	/* set qspi work mode */
	val = x2qspi_rd(x2qspi, X2_QSPI_CTL1_REG);
	val |= X2_QSPI_MST;
	val &= (~X2_QSPI_FW_MASK);
	val |= X2_QSPI_FW8;
	x2qspi_wr(x2qspi, X2_QSPI_CTL1_REG, val);

	/* init interrupt */
	val = X2_QSPI_RBC_INT | X2_QSPI_TBC_INT |
		X2_QSPI_ERR_INT;
	x2qspi_wr(x2qspi, X2_QSPI_CTL2_REG, val);

	/* unselect chip */
	x2qspi_wr(x2qspi, X2_QSPI_CS_REG, 0x0);

	/* Always set SPI to one line as init. */
	val = x2qspi_rd(x2qspi, X2_QSPI_DQM_REG);
	val |= 0xfc;
	x2qspi_wr(x2qspi, X2_QSPI_DQM_REG, val);

	/* Disable hardware xip mode */
	val = x2qspi_rd(x2qspi, X2_QSPI_XIP_REG);
	val &= ~(1 << 1);
	x2qspi_wr(x2qspi, X2_QSPI_XIP_REG, val);

	/* Set Rx/Tx fifo trig level  */
	x2qspi_wr(x2qspi, X2_QSPI_RTL_REG, X2_QSPI_TRIG_LEVEL);
	x2qspi_wr(x2qspi, X2_QSPI_TTL_REG, X2_QSPI_TRIG_LEVEL);

	return;
}

static int x2_qspi_setup_flash(struct x2qspi_pdata *x2qspi, struct device_node *np)
{
	int i, ret;
	unsigned int cs;
	struct platform_device *pdev = x2qspi->pdev;
	struct device *dev = &pdev->dev;
	struct x2qspi_flash_pdata *f_pdata;
	struct spi_nor *nor;
	struct mtd_info *mtd;
	const struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ |
			SNOR_HWCAPS_READ_FAST |
			SNOR_HWCAPS_READ_1_1_2 |
			SNOR_HWCAPS_READ_1_1_4 |
			SNOR_HWCAPS_PP |
			SNOR_HWCAPS_PP_1_1_4,
	};

	/* Get flash device data */
	for_each_available_child_of_node(dev->of_node, np) {
		ret = of_property_read_u32(np, "reg", &cs);
		if (ret) {
			dev_err(dev, "Couldn't determine chip select.\n");
			goto err;
		}

		if (cs >= X2_QSPI_MAX_CS) {
			ret = -EINVAL;
			dev_err(dev, "Chip select %d out of range.\n", cs);
			goto err;
		}

		f_pdata = &x2qspi->f_pdata[cs];
		f_pdata->x2qspi = x2qspi;
		f_pdata->cs = cs;

		if (of_property_read_u32(np, "spi-max-frequency", &f_pdata->clk_rate)) {
			dev_err(&pdev->dev, "couldn't determine spi-max-frequency\n");
			f_pdata->clk_rate = X2_QSPI_DEF_BDR;
		}

		nor = &f_pdata->nor;
		mtd = &nor->mtd;

		mtd->priv = nor;

		nor->dev = dev;
		spi_nor_set_flash_node(nor, np);
		nor->priv = f_pdata;

		nor->prepare   = x2_qspi_flash_prep;
		nor->unprepare = x2_qspi_flash_unprep;
		nor->read_reg  = x2_qspi_flash_rd_reg;
		nor->write_reg = x2_qspi_flash_wr_reg;
		nor->read      = x2_qspi_flash_read;
		nor->write     = x2_qspi_flash_write;
		nor->erase     = x2_qspi_flash_erase;

		mtd->name = devm_kasprintf(dev, GFP_KERNEL, "%s.%d", dev_name(dev), cs);
		if (!mtd->name) {
			ret = -ENOMEM;
			goto err;
		}

		ret = spi_nor_scan(nor, NULL, &hwcaps);
		if (ret)
			goto err;

		ret = mtd_device_register(mtd, NULL, 0);
		if (ret)
			goto err;

		f_pdata->registered = true;
	}

	return 0;
err:
	for (i = 0; i < X2_QSPI_MAX_CS; i++)
		if (x2qspi->f_pdata[i].registered)
			mtd_device_unregister(&x2qspi->f_pdata[i].nor.mtd);
	return ret;
}

static int x2_qspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct x2qspi_pdata *x2qspi;
	struct resource *res;

	x2qspi = devm_kzalloc(dev, sizeof(*x2qspi), GFP_KERNEL);
	if (!x2qspi)
		return -ENOMEM;
	mutex_init(&x2qspi->lock);
	x2qspi->pdev = pdev;
	platform_set_drvdata(pdev, x2qspi);

	/* get the module ref-clk and enabel it */
	x2qspi->clk = devm_clk_get(&pdev->dev, "qspi_aclk");
	if (IS_ERR(x2qspi->clk)) {
		dev_err(&pdev->dev, "uart_clk clock not found.\n");
		return PTR_ERR(x2qspi->clk);
	}
	ret = clk_prepare_enable(x2qspi->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable device clock.\n");
		goto probe_clk_failed;
	}
	x2qspi->ref_clk = clk_get_rate(x2qspi->clk);

	/* Obtain and remap controller address. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	x2qspi->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(x2qspi->iobase)) {
		return PTR_ERR(x2qspi->iobase);
	}

	init_completion(&x2qspi->xfer_complete);

	x2qspi->cur_cs = X2_QSPI_MAX_CS;

	/* Obtain IRQ line */
	x2qspi->irq = platform_get_irq(pdev, 0);
	if (x2qspi->irq < 0) {
		dev_err(dev, "Cannot obtain IRQ.\n");
		goto probe_setup_failed;
	}
	ret = devm_request_irq(dev, x2qspi->irq, x2_qspi_irq_handler, 0, pdev->name, x2qspi);
	if (ret) {
		dev_err(dev, "Cannot request IRQ.\n");
		goto probe_setup_failed;
	}

	/* init hardware module */
	x2_qspi_hw_init(x2qspi);

	ret = x2_qspi_setup_flash(x2qspi, np);
	if (ret) {
		dev_err(dev, "X2 QSPI NOR probe failed %d\n", ret);
		goto probe_setup_failed;
	}
	if (diag_register(ModuleDiag_norflash, EventIdNorflashErr,
						4, 10, 5000, qspi_callback) < 0)
		pr_err("qspi norflash diag register fail\n");

	return ret;
probe_setup_failed:
	mutex_destroy(&x2qspi->lock);
probe_clk_failed:
	clk_disable_unprepare(x2qspi->clk);

	dev_err(dev, "X2 QuadSPI probe failed\n");
	return ret;
}

static int x2_qspi_remove(struct platform_device *pdev)
{
	int i;
	struct x2qspi_pdata *x2qspi = platform_get_drvdata(pdev);

	for (i = 0; i < X2_QSPI_MAX_CS; i++)
		if (x2qspi->f_pdata[i].registered)
			mtd_device_unregister(&x2qspi->f_pdata[i].nor.mtd);

	clk_disable_unprepare(x2qspi->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
int x2_qspi_nor_suspend(struct device *dev)
{
	struct x2qspi_pdata *x2qspi = dev_get_drvdata(dev);

	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

	/* wait for idle */
	//x2_qspi_tx_empty(x2qspi);
	//x2_qspi_rx_empty(x2qspi);

	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_TX_DIS);
	x2_qspi_set_xfer(x2qspi, X2_QSPI_OP_RX_DIS);

	clk_disable_unprepare(x2qspi->clk);

	return 0;
}

int x2_qspi_nor_resume(struct device *dev)
{
	struct x2qspi_pdata *x2qspi = dev_get_drvdata(dev);

	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);

	clk_prepare_enable(x2qspi->clk);

	x2_qspi_hw_init(x2qspi);

	return 0;
}
#endif

static const struct dev_pm_ops x2_qspi_nor_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(x2_qspi_nor_suspend,
			x2_qspi_nor_resume)
};

static const struct of_device_id x2_qspi_of_match[] = {
    { .compatible = "hobot,x2-qspi" },
    { /* end of table */ }
};

MODULE_DEVICE_TABLE(of, x2_qspi_of_match);

static struct platform_driver x2_qspi_driver = {
    .probe = x2_qspi_probe,
    .remove = x2_qspi_remove,
    .driver = {
        .name = X2_QSPI_NAME,
        .of_match_table = x2_qspi_of_match,
		.pm = &x2_qspi_nor_dev_pm_ops,
    },
};
module_platform_driver(x2_qspi_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("X2 QSPI driver");
MODULE_LICENSE("GPL v2");

