/*
 * Horizon HB QuadSPI driver.
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
#include <soc/hobot/diag.h>
#include <linux/spi/spi-hobot-qspi.h>

#define HB_QSPI_NAME                "hb_qspi_nor"

static int first_time;
static int last_err;
/* #define HB_QSPI_WORK_POLL    1 */

struct hbqspi_pdata;
struct hbqspi_flash_pdata {
	struct spi_nor nor;
	struct hbqspi_pdata *hbqspi;
	u32 clk_rate;
	u8  cs;
	bool registered;
};

/**
 * struct hb_qspi_platdata - platform data for hb QSPI
 *
 * @regs: Point to the base address of QSPI registers
 * @freq: QSPI input clk frequency
 * @speed_hz: Default BUS sck frequency
 * @xfer_mode: 0-Byte 1-batch 2-dma, Default work in byte mode
 */
struct hbqspi_pdata {
	int irq;
	u32 ref_clk;
	u32 cur_cs;
	void __iomem *iobase;
	struct clk *clk;
	struct mutex lock;
	struct platform_device *pdev;
	struct completion xfer_complete;
	struct hbqspi_flash_pdata f_pdata[HB_QSPI_MAX_CS];
};


#define hbqspi_rd(dev, reg)       ioread32((dev)->iobase + (reg))
#define hbqspi_wr(dev, reg, val)  iowrite32((val), (dev)->iobase + (reg))

static void hb_qspi_set_speed(struct spi_nor *nor)
{
	int confr, prescaler, divisor;
	unsigned int max_br, min_br, br_div;
	struct hbqspi_flash_pdata *f_pdata = (struct hbqspi_flash_pdata *)nor->priv;
	struct hbqspi_pdata *hbqspi   = f_pdata->hbqspi;

	if (hbqspi->cur_cs == f_pdata->cs)
		return;
	hbqspi->cur_cs = f_pdata->cs;

	max_br = hbqspi->ref_clk / 2;
	min_br = hbqspi->ref_clk / 1048576;
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
			if ((hbqspi->ref_clk / br_div) >= f_pdata->clk_rate) {
				confr = (prescaler | (divisor << 4)) & 0xFF;
				hbqspi_wr(hbqspi, HB_QSPI_BDR_REG, confr);
				return;
			}
		}
	}

	return;
}

/* currend driver only support BYTE/DUAL/QUAD for both RX and TX */
static int hb_qspi_set_wire(struct hbqspi_pdata *hbqspi, uint mode)
{
	unsigned int val = 0;

	switch (mode) {
	case HB_QSPI_BUS_DUAL:
		val = HB_QSPI_INST_DUAL;
		break;
	case HB_QSPI_BUS_QUAD:
		val = HB_QSPI_INST_QUAD;
		break;
	case HB_QSPI_BUS_SINGLE:
	default:
		val = 0xFC;
		break;
	}
	hbqspi_wr(hbqspi, HB_QSPI_DQM_REG, val);

	return 0;
}


static void hb_qspi_reset_fifo(struct hbqspi_pdata *hbqspi)
{
	u32 val;

	val = hbqspi_rd(hbqspi, HB_QSPI_CTL3_REG);
	val |= HB_QSPI_RST_FIFO;
	hbqspi_wr(hbqspi, HB_QSPI_CTL3_REG, val);
	mdelay(1);
	val = (~HB_QSPI_RST_FIFO);
	hbqspi_wr(hbqspi, HB_QSPI_CTL3_REG, val);

	return;
}

/* tx almost empty */
static int hb_qspi_tx_ae(struct hbqspi_pdata *hbqspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = hbqspi_rd(hbqspi, HB_QSPI_ST1_REG);
		trys ++;
	} while ((!(val&HB_QSPI_TX_AE)) && (trys < TRYS_TOTAL_NUM));
	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys < TRYS_TOTAL_NUM ? 0 : -1;
}

/* rx almost full */
static int hb_qspi_rx_af(struct hbqspi_pdata *hbqspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = hbqspi_rd(hbqspi, HB_QSPI_ST1_REG);
		trys ++;
	} while ((!(val&HB_QSPI_RX_AF)) && (trys < TRYS_TOTAL_NUM));
	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys < TRYS_TOTAL_NUM ? 0 : -1;
}

static int __maybe_unused hb_qspi_tb_done(struct hbqspi_pdata *hbqspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = hbqspi_rd(hbqspi, HB_QSPI_ST1_REG);
		trys ++;
	} while ((!(val&HB_QSPI_TBD)) && (trys < TRYS_TOTAL_NUM));
	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);
	hbqspi_wr(hbqspi, HB_QSPI_ST1_REG, (val|HB_QSPI_TBD));


	return trys < TRYS_TOTAL_NUM ? 0 : -1;
}

static int __maybe_unused hb_qspi_rb_done(struct hbqspi_pdata *hbqspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = hbqspi_rd(hbqspi, HB_QSPI_ST1_REG);
		trys ++;
	} while ((!(val&HB_QSPI_RBD)) && (trys < TRYS_TOTAL_NUM));
	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);
	hbqspi_wr(hbqspi, HB_QSPI_ST1_REG, (val|HB_QSPI_RBD));


	return trys < TRYS_TOTAL_NUM ? 0 : -1;
}

static int hb_qspi_tx_full(struct hbqspi_pdata *hbqspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = hbqspi_rd(hbqspi, HB_QSPI_ST2_REG);
		trys ++;
	} while ((val&HB_QSPI_TX_FULL) && (trys < TRYS_TOTAL_NUM));

	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys < TRYS_TOTAL_NUM ? 0 : -1;
}

static int hb_qspi_tx_empty(struct hbqspi_pdata *hbqspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = hbqspi_rd(hbqspi, HB_QSPI_ST2_REG);
		trys ++;
	} while ((!(val&HB_QSPI_TX_EP)) && (trys < TRYS_TOTAL_NUM));

	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys < TRYS_TOTAL_NUM ? 0 : -1;
}

static int hb_qspi_rx_empty(struct hbqspi_pdata *hbqspi)
{
	u32 val, trys=0;

	do {
		ndelay(10);
		val = hbqspi_rd(hbqspi, HB_QSPI_ST2_REG);
		trys ++;
	} while ((val&HB_QSPI_RX_EP) && (trys < TRYS_TOTAL_NUM));

	if (trys >= TRYS_TOTAL_NUM)
		pr_err("%s_%d:val=%x, trys=%d\n", __func__, __LINE__, val, trys);

	return trys < TRYS_TOTAL_NUM ? 0 : -1;
}

/* config fifo width */
static void hb_qspi_set_fw(struct hbqspi_pdata *hbqspi, u32 fifo_width)
{
	u32 val;

	val = hbqspi_rd(hbqspi, HB_QSPI_CTL1_REG);
	val &= (~0x3);
	val |= fifo_width;
	hbqspi_wr(hbqspi, HB_QSPI_CTL1_REG, val);

	return;
}

/*config xfer mode:enable/disable BATCH/RX/TX */
static void hb_qspi_set_xfer(struct hbqspi_pdata *hbqspi, u32 op_flag)
{
	u32 ctl1_val = 0, ctl3_val = 0;

	ctl1_val = hbqspi_rd(hbqspi, HB_QSPI_CTL1_REG);
	ctl3_val = hbqspi_rd(hbqspi, HB_QSPI_CTL3_REG);

	switch (op_flag) {
	case HB_QSPI_OP_RX_EN:
		ctl1_val |= HB_QSPI_RX_EN;
		break;
	case HB_QSPI_OP_RX_DIS:
		ctl1_val &= (~HB_QSPI_RX_EN);
		break;
	case HB_QSPI_OP_TX_EN:
		ctl1_val |= HB_QSPI_TX_EN;
		break;
	case HB_QSPI_OP_TX_DIS:
		ctl1_val &= (~HB_QSPI_TX_EN);
		break;
	case HB_QSPI_OP_BAT_EN:
		ctl3_val &= (~HB_QSPI_BATCH_DIS);
		break;
	case HB_QSPI_OP_BAT_DIS:
		ctl3_val |= HB_QSPI_BATCH_DIS;
		break;
	default:
		pr_err("Op(0x%x) if error, please check it!\n", op_flag);
		break;
	}
	hbqspi_wr(hbqspi, HB_QSPI_CTL1_REG, ctl1_val);
	hbqspi_wr(hbqspi, HB_QSPI_CTL3_REG, ctl3_val);

	return;
}

static int hbqspi_rd_batch(struct hbqspi_pdata *hbqspi, void *pbuf, uint32_t len)
{
	u32 i, rx_len, offset = 0, tmp_len = len, ret = 0;
	u32 *dbuf = (u32 *) pbuf;

	/* Enable batch mode */
	hb_qspi_set_fw(hbqspi, HB_QSPI_FW32);
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_BAT_EN);

	while (tmp_len > 0) {
		reinit_completion(&hbqspi->xfer_complete);
		rx_len = MIN(tmp_len, BATCH_MAX_CNT);
		hbqspi_wr(hbqspi, HB_QSPI_RBC_REG, rx_len);
		/* enbale rx */
		hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_RX_EN);

		for (i=0; i< rx_len; i+=8) {
			if (hb_qspi_rx_af(hbqspi)) {
				ret = -1;
				goto rb_err;
			}
			dbuf[offset++] = hbqspi_rd(hbqspi, HB_QSPI_DAT_REG);
			dbuf[offset++] = hbqspi_rd(hbqspi, HB_QSPI_DAT_REG);
		}
#ifdef HB_QSPI_WORK_POLL
		if (hb_qspi_rb_done(hbqspi)) {
			pr_err("%s_%d:rx failed! len=%d, received=%d, i=%d\n", __func__, __LINE__, len, offset, i);
			ret = -EIO;
			goto rb_err;
		}
#else
		ret = wait_for_completion_timeout(&hbqspi->xfer_complete, msecs_to_jiffies(HB_QSPI_TIMEOUT_MS));
		if (!ret) {
			pr_err("%s_%d:rx failed! len=%d, received=%d, i=%d\n", __func__, __LINE__, len, offset, i);
			ret = -EIO;
			goto rb_err;
		}
		ret = 0;
#endif
		hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_RX_DIS);
		tmp_len = tmp_len - rx_len;
	}

rb_err:
	/* Disable batch mode and rx link */
	hb_qspi_set_fw(hbqspi, HB_QSPI_FW8);
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_BAT_DIS);
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_RX_DIS);

	return ret;
}

static int hbqspi_wr_batch(struct hbqspi_pdata *hbqspi, const void *pbuf, uint32_t len)
{
	u32 i, tx_len, offset = 0, tmp_len = len, ret = 0;
	u32 *dbuf = (u32 *) pbuf;

	hb_qspi_set_fw(hbqspi, HB_QSPI_FW32);
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_BAT_EN);

	while (tmp_len > 0) {
		reinit_completion(&hbqspi->xfer_complete);
		tx_len = MIN(tmp_len, BATCH_MAX_CNT);
		hbqspi_wr(hbqspi, HB_QSPI_TBC_REG, tx_len);

		/* enbale tx */
		hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_TX_EN);

		for (i = 0; i < tx_len; i += 8) {
			if (hb_qspi_tx_ae(hbqspi)) {
				ret = -1;
				goto tb_err;
			}
			hbqspi_wr(hbqspi, HB_QSPI_DAT_REG, dbuf[offset++]);
			hbqspi_wr(hbqspi, HB_QSPI_DAT_REG, dbuf[offset++]);
		}
#ifdef HB_QSPI_WORK_POLL
		if (hb_qspi_tb_done(hbqspi)) {
			pr_err("%s_%d:tx failed! len=%d, received=%d, i=%d\n", __func__, __LINE__, len, offset, i);
			ret = -EIO;
			goto tb_err;
		}
#else
		ret = wait_for_completion_timeout(&hbqspi->xfer_complete, msecs_to_jiffies(HB_QSPI_TIMEOUT_MS));
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
	hb_qspi_set_fw(hbqspi, HB_QSPI_FW8);
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_BAT_DIS);
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_TX_DIS);

	return ret;
}

static int hbqspi_rd_byte(struct hbqspi_pdata *hbqspi, void *pbuf, uint32_t len)
{
	u32 i, ret = 0;
	u8 *dbuf = (u8 *) pbuf;

	/* enbale rx */
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_RX_EN);

	for (i = 0; i < len; i++)
	{
		if (hb_qspi_tx_empty(hbqspi)) {
			ret = -1;
			goto rd_err;
		}
		hbqspi_wr(hbqspi, HB_QSPI_DAT_REG, 0x00);
		if (hb_qspi_rx_empty(hbqspi)) {
			ret = -1;
			goto rd_err;
		}
		dbuf[i] = hbqspi_rd(hbqspi, HB_QSPI_DAT_REG) & 0xFF;
	}

rd_err:
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_RX_DIS);

	if (0 != ret)
		pr_err("%s_%d:read op failed! i=%d\n", __func__, __LINE__, i);

	return ret;
}

static int hbqspi_wr_byte(struct hbqspi_pdata *hbqspi, const void *pbuf, uint32_t len)
{
	u32 i, ret = 0;
	u8 *dbuf = (u8 *) pbuf;

	/* enbale tx */
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_TX_EN);

	for (i = 0; i < len; i++)
	{
		if (hb_qspi_tx_full(hbqspi)) {
			ret = -1;
			goto wr_err;
		}
		hbqspi_wr(hbqspi, HB_QSPI_DAT_REG, dbuf[i]);
	}
	/* Check tx complete */
	if (hb_qspi_tx_empty(hbqspi))
		ret = -1;

wr_err:
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_TX_DIS);

	if (0 != ret)
		pr_err("%s_%d:write op failed! i=%d\n", __func__, __LINE__, i);

	return ret;
}

static int hb_qspi_read(struct hbqspi_pdata *hbqspi, void *pbuf, uint32_t len)
{
	u32 ret = 0;
	u32 remainder = len % HB_QSPI_TRIG_LEVEL;
	u32 residue   = len - remainder;

	if (residue > 0)
		ret = hbqspi_rd_batch(hbqspi, pbuf, residue);
	if (remainder > 0)
		ret = hbqspi_rd_byte(hbqspi, (u8 *) pbuf + residue, remainder);
	if (ret < 0)
		pr_err("hb_qspi_read failed!\n");

	return ret;
}

static int hb_qspi_write(struct hbqspi_pdata *hbqspi, const void *pbuf, uint32_t len)
{
	u32 ret = 0;
	u32 remainder = len % HB_QSPI_TRIG_LEVEL;
	u32 residue   = len - remainder;

	if (residue > 0)
		ret = hbqspi_wr_batch(hbqspi, pbuf, residue);
	if (remainder > 0)
		ret = hbqspi_wr_byte(hbqspi, (u8 *) pbuf + residue, remainder);
	if (ret < 0)
		pr_err("hbqspi_write failed!\n");

	return ret;
}

int hb_qspi_xfer(struct spi_nor *nor, unsigned int len, const void *dout,
		 void *din, unsigned long flags)
{
	int ret = 0;
	struct hbqspi_flash_pdata *f_pdata = (struct hbqspi_flash_pdata *)nor->priv;
	struct hbqspi_pdata *hbqspi   = f_pdata->hbqspi;

	if (len == 0) {
		return 0;
	}

	if (flags & HB_QSPI_XFER_BEGIN)
		hbqspi_wr(hbqspi, HB_QSPI_CS_REG, 1 << f_pdata->cs);  /* Assert CS before transfer */

	if (dout) {
		ret = hb_qspi_write(hbqspi, dout, len);
	}
	else if (din) {
		ret = hb_qspi_read(hbqspi, din, len);
	}

	if (flags & HB_QSPI_XFER_END) {
		hbqspi_wr(hbqspi, HB_QSPI_CS_REG, 0);  /* Deassert CS after transfer */
	}

	if (flags & HB_QSPI_XFER_CMD) {
		switch (((u8 *) dout) [0]) {
		case CMD_READ_QUAD_OUTPUT_FAST:
		case CMD_QUAD_PAGE_PROGRAM:
			hb_qspi_set_wire(hbqspi, HB_QSPI_BUS_QUAD);
			break;
		case CMD_READ_DUAL_OUTPUT_FAST:
			hb_qspi_set_wire(hbqspi, HB_QSPI_BUS_DUAL);
			break;
		default:
			hb_qspi_set_wire(hbqspi, HB_QSPI_BUS_SINGLE);
			break;
		}
	} else {
		hb_qspi_set_wire(hbqspi, HB_QSPI_BUS_SINGLE);
	}

	return ret;
}

static int hb_qspi_flash_rd_wr(struct spi_nor *nor, const u8 *cmd, size_t cmd_len,
				const u8 *data_out, u8 *data_in, size_t data_len)
{
	int ret;
	unsigned long flags = HB_QSPI_XFER_BEGIN;

	hb_qspi_set_speed(nor);

	if (data_len == 0)
		flags |= HB_QSPI_XFER_END;

	ret = hb_qspi_xfer(nor, cmd_len, cmd, NULL, flags|HB_QSPI_XFER_CMD);
	if (ret) {
		pr_err("SF: Failed to send command (%d bytes): %d\n", (int)cmd_len, ret);
	} else if (data_len != 0) {
		ret = hb_qspi_xfer(nor, data_len, data_out, data_in, HB_QSPI_XFER_END);
		if (ret)
			pr_err("SF: Failed to transfer %d bytes of data: %d\n", (int)data_len, ret);
	}

	return ret;
}

static void hb_qspi_flash_addr(struct spi_nor *nor, u32 addr, u8 *cmd)
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

static int hb_qspi_flash_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct hbqspi_flash_pdata *f_pdata = (struct hbqspi_flash_pdata *)nor->priv;
	struct hbqspi_pdata *hbqspi   = f_pdata->hbqspi;

	mutex_lock(&hbqspi->lock);

	return 0;
}

static void hb_qspi_flash_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct hbqspi_flash_pdata *f_pdata = (struct hbqspi_flash_pdata *)nor->priv;
	struct hbqspi_pdata *hbqspi   = f_pdata->hbqspi;

	mutex_unlock(&hbqspi->lock);
}

static int hb_qspi_flash_rd_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	return hb_qspi_flash_rd_wr(nor, &opcode, 1, NULL, buf, len);
}

static int hb_qspi_flash_wr_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	return hb_qspi_flash_rd_wr(nor, &opcode, 1, buf, NULL, len);
}

static ssize_t hb_qspi_flash_read(struct spi_nor *nor, loff_t from,
				size_t len, u_char *read_buf)
{
	u8 cmdsz;

        if (nor->mtd.size > SPI_FLASH_16MB_BOUN)
		cmdsz = SPI_FLASH_CMD_LEN + nor->read_dummy/8;
	else
		cmdsz = SPI_FLASH_CMD_LEN - 1 + nor->read_dummy/8;

	nor->cmd_buf[0] = nor->read_opcode;
	hb_qspi_flash_addr(nor, from, nor->cmd_buf);

	if (hb_qspi_flash_rd_wr(nor, nor->cmd_buf, cmdsz, NULL, read_buf, len))
		return -EIO;

	return len;
}

static ssize_t hb_qspi_flash_write(struct spi_nor *nor, loff_t to,
				size_t len, const u_char *write_buf)
{
	u8 cmdsz = SPI_FLASH_CMD_LEN;

        if (nor->mtd.size > SPI_FLASH_16MB_BOUN)
		cmdsz = SPI_FLASH_CMD_LEN;
	else
		cmdsz = SPI_FLASH_CMD_LEN - 1;

	nor->cmd_buf[0] = nor->program_opcode;
	hb_qspi_flash_addr(nor, to, nor->cmd_buf);

	if (hb_qspi_flash_rd_wr(nor, nor->cmd_buf, cmdsz, write_buf, NULL, len))
		return -EIO;

	return len;
}

static int hb_qspi_flash_erase(struct spi_nor *nor, loff_t offs)
{
	u8 cmdsz = SPI_FLASH_CMD_LEN;

        if (nor->mtd.size > SPI_FLASH_16MB_BOUN)
		cmdsz = SPI_FLASH_CMD_LEN;
	else
		cmdsz = SPI_FLASH_CMD_LEN - 1;

	nor->cmd_buf[0] = nor->erase_opcode;
	hb_qspi_flash_addr(nor, offs, nor->cmd_buf);

	return hb_qspi_flash_rd_wr(nor, nor->cmd_buf, cmdsz, NULL, NULL, 0);
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

static irqreturn_t hb_qspi_irq_handler(int irq, void *dev_id)
{
	unsigned int irq_status;
	unsigned int err_status;
	int err = 0;
	struct hbqspi_pdata *hbqspi = dev_id;

	/* Read interrupt status */
	irq_status = hbqspi_rd(hbqspi, HB_QSPI_ST1_REG);
	hbqspi_wr(hbqspi, HB_QSPI_ST1_REG, HB_QSPI_TBD | HB_QSPI_RBD);

	err_status = hbqspi_rd(hbqspi, HB_QSPI_ST2_REG);
	hbqspi_wr(hbqspi, HB_QSPI_ST2_REG,
			HB_QSPI_RXWR_FULL | HB_QSPI_TXRD_EMPTY);

	if (irq_status & (HB_QSPI_TBD | HB_QSPI_RBD))
		complete(&hbqspi->xfer_complete);

	if (err_status & (HB_QSPI_RXWR_FULL | HB_QSPI_TXRD_EMPTY))
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

static void hb_qspi_hw_init(struct hbqspi_pdata *hbqspi)
{
	uint32_t val;

	/* disable batch operation and reset fifo */
	val = hbqspi_rd(hbqspi, HB_QSPI_CTL3_REG);
	val |= HB_QSPI_BATCH_DIS;
	hbqspi_wr(hbqspi, HB_QSPI_CTL3_REG, val);
	hb_qspi_reset_fifo(hbqspi);

	/* clear status */
	val = HB_QSPI_MODF_CLR | HB_QSPI_RBD | HB_QSPI_TBD;
	hbqspi_wr(hbqspi, HB_QSPI_ST1_REG, val);
	val = HB_QSPI_TXRD_EMPTY | HB_QSPI_RXWR_FULL;
	hbqspi_wr(hbqspi, HB_QSPI_ST2_REG, val);

	/* set qspi work mode */
	val = hbqspi_rd(hbqspi, HB_QSPI_CTL1_REG);
	val |= HB_QSPI_MST;
	val &= (~HB_QSPI_FW_MASK);
	val |= HB_QSPI_FW8;
	hbqspi_wr(hbqspi, HB_QSPI_CTL1_REG, val);

	/* init interrupt */
	val = HB_QSPI_RBC_INT | HB_QSPI_TBC_INT |
		HB_QSPI_ERR_INT;
	hbqspi_wr(hbqspi, HB_QSPI_CTL2_REG, val);

	/* unselect chip */
	hbqspi_wr(hbqspi, HB_QSPI_CS_REG, 0x0);

	/* Always set SPI to one line as init. */
	val = hbqspi_rd(hbqspi, HB_QSPI_DQM_REG);
	val |= 0xfc;
	hbqspi_wr(hbqspi, HB_QSPI_DQM_REG, val);

	/* Disable hardware xip mode */
	val = hbqspi_rd(hbqspi, HB_QSPI_XIP_REG);
	val &= ~(1 << 1);
	hbqspi_wr(hbqspi, HB_QSPI_XIP_REG, val);

	/* Set Rx/Tx fifo trig level  */
	hbqspi_wr(hbqspi, HB_QSPI_RTL_REG, HB_QSPI_TRIG_LEVEL);
	hbqspi_wr(hbqspi, HB_QSPI_TTL_REG, HB_QSPI_TRIG_LEVEL);

	return;
}

static int hb_qspi_setup_flash(struct hbqspi_pdata *hbqspi, struct device_node *np)
{
	int i, ret;
	unsigned int cs;
	struct platform_device *pdev = hbqspi->pdev;
	struct device *dev = &pdev->dev;
	struct hbqspi_flash_pdata *f_pdata;
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

		if (cs >= HB_QSPI_MAX_CS) {
			ret = -EINVAL;
			dev_err(dev, "Chip select %d out of range.\n", cs);
			goto err;
		}

		f_pdata = &hbqspi->f_pdata[cs];
		f_pdata->hbqspi = hbqspi;
		f_pdata->cs = cs;

		if (of_property_read_u32(np, "spi-max-frequency", &f_pdata->clk_rate)) {
			dev_err(&pdev->dev, "couldn't determine spi-max-frequency\n");
			f_pdata->clk_rate = HB_QSPI_DEF_BDR;
		}

		nor = &f_pdata->nor;
		mtd = &nor->mtd;

		mtd->priv = nor;

		nor->dev = dev;
		spi_nor_set_flash_node(nor, np);
		nor->priv = f_pdata;

		nor->prepare   = hb_qspi_flash_prep;
		nor->unprepare = hb_qspi_flash_unprep;
		nor->read_reg  = hb_qspi_flash_rd_reg;
		nor->write_reg = hb_qspi_flash_wr_reg;
		nor->read      = hb_qspi_flash_read;
		nor->write     = hb_qspi_flash_write;
		nor->erase     = hb_qspi_flash_erase;

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
	for (i = 0; i < HB_QSPI_MAX_CS; i++)
		if (hbqspi->f_pdata[i].registered)
			mtd_device_unregister(&hbqspi->f_pdata[i].nor.mtd);
	return ret;
}

static int hb_qspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct hbqspi_pdata *hbqspi;
	struct resource *res;

	hbqspi = devm_kzalloc(dev, sizeof(*hbqspi), GFP_KERNEL);
	if (!hbqspi)
		return -ENOMEM;
	mutex_init(&hbqspi->lock);
	hbqspi->pdev = pdev;
	platform_set_drvdata(pdev, hbqspi);

	/* get the module ref-clk and enabel it */
	hbqspi->clk = devm_clk_get(&pdev->dev, "qspi_aclk");
	if (IS_ERR(hbqspi->clk)) {
		dev_err(&pdev->dev, "uart_clk clock not found.\n");
		return PTR_ERR(hbqspi->clk);
	}
	ret = clk_prepare_enable(hbqspi->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable device clock.\n");
		goto probe_clk_failed;
	}
	hbqspi->ref_clk = clk_get_rate(hbqspi->clk);

	/* Obtain and remap controller address. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hbqspi->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(hbqspi->iobase)) {
		return PTR_ERR(hbqspi->iobase);
	}

	init_completion(&hbqspi->xfer_complete);

	hbqspi->cur_cs = HB_QSPI_MAX_CS;

	/* Obtain IRQ line */
	hbqspi->irq = platform_get_irq(pdev, 0);
	if (hbqspi->irq < 0) {
		dev_err(dev, "Cannot obtain IRQ.\n");
		goto probe_setup_failed;
	}
	ret = devm_request_irq(dev, hbqspi->irq, hb_qspi_irq_handler, 0, pdev->name, hbqspi);
	if (ret) {
		dev_err(dev, "Cannot request IRQ.\n");
		goto probe_setup_failed;
	}

	/* init hardware module */
	hb_qspi_hw_init(hbqspi);

	ret = hb_qspi_setup_flash(hbqspi, np);
	if (ret) {
		dev_err(dev, "HB QSPI NOR probe failed %d\n", ret);
		goto probe_setup_failed;
	}
	if (diag_register(ModuleDiag_norflash, EventIdNorflashErr,
						4, 10, 5000, qspi_callback) < 0)
		pr_err("qspi norflash diag register fail\n");

	return ret;
probe_setup_failed:
	mutex_destroy(&hbqspi->lock);
probe_clk_failed:
	clk_disable_unprepare(hbqspi->clk);

	dev_err(dev, "HB QuadSPI probe failed\n");
	return ret;
}

static int hb_qspi_remove(struct platform_device *pdev)
{
	int i;
	struct hbqspi_pdata *hbqspi = platform_get_drvdata(pdev);

	for (i = 0; i < HB_QSPI_MAX_CS; i++)
		if (hbqspi->f_pdata[i].registered)
			mtd_device_unregister(&hbqspi->f_pdata[i].nor.mtd);

	clk_disable_unprepare(hbqspi->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
int hb_qspi_nor_suspend(struct device *dev)
{
	struct hbqspi_pdata *hbqspi = dev_get_drvdata(dev);

	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

	/* wait for idle */
	//hb_qspi_tx_empty(hbqspi);
	//hb_qspi_rx_empty(hbqspi);

	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_TX_DIS);
	hb_qspi_set_xfer(hbqspi, HB_QSPI_OP_RX_DIS);

	clk_disable_unprepare(hbqspi->clk);

	return 0;
}

int hb_qspi_nor_resume(struct device *dev)
{
	struct hbqspi_pdata *hbqspi = dev_get_drvdata(dev);

	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);

	clk_prepare_enable(hbqspi->clk);

	hb_qspi_hw_init(hbqspi);

	return 0;
}
#endif

static const struct dev_pm_ops hb_qspi_nor_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hb_qspi_nor_suspend,
			hb_qspi_nor_resume)
};

static const struct of_device_id hb_qspi_of_match[] = {
    { .compatible = "hobot,hb-qspi-nor" },
    { /* end of table */ }
};

MODULE_DEVICE_TABLE(of, hb_qspi_of_match);

static struct platform_driver hb_qspi_driver = {
    .probe = hb_qspi_probe,
    .remove = hb_qspi_remove,
    .driver = {
        .name = HB_QSPI_NAME,
        .of_match_table = hb_qspi_of_match,
		.pm = &hb_qspi_nor_dev_pm_ops,
    },
};
module_platform_driver(hb_qspi_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("HB QSPI driver");
MODULE_LICENSE("GPL v2");
