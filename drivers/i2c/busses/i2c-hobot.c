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

#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/reset.h>
#include <soc/hobot/diag.h>
#include <linux/clk.h>

#include <linux/i2c-hobot.h>

#define HOBOT_I2C_FIFO_SIZE	16
#define WAIT_IDLE_TIMEOUT	200 /* ms */
#define XFER_TIMEOUT		1000 /* ms */

#define I2C_MAX_DIV			255
#define I2C_SCL_DEFAULT_FREQ	100000 /*I2c SCL default frequency is 100KHZ*/
#define I2C_CONTROL_CLK		24000000

enum {
	i2c_idle,
	i2c_write,
	i2c_read,
	i2c_write_read,
};

struct hobot_i2c_dev {
	struct device *dev;
	volatile struct hobot_i2c_regs_s *i2c_regs;
	struct reset_control *rst;
	struct mutex lock;
	int clkdiv;
	int default_trans_freq;
	int irq;
	struct i2c_adapter adapter;
	struct completion completion;
	struct clk *clk;
	u32 i2c_state;
	u32 msg_err;
	u8 *tx_buf;
	u8 *rx_buf;
	size_t tx_remaining;
	size_t rx_remaining;
	uint8_t i2c_id;
	bool is_suspended;
};

static int hobot_i2c_cfg(struct hobot_i2c_dev *dev, int dir_rd, int timeout_enable)
{
	union cfg_reg_e cfg;

	cfg.all = 0;
	cfg.bit.tran_en = 1;
	cfg.bit.en = 1;
	if (timeout_enable) {
		cfg.bit.to_en = 1;
	} else {
		cfg.bit.to_en = 0;
	}
	cfg.bit.dir_rd = dir_rd;
	cfg.bit.clkdiv = dev->clkdiv;
	iowrite32(0xffff, &dev->i2c_regs->tocnt);
	iowrite32(cfg.all, &dev->i2c_regs->cfg);
	return 0;
}

static int hobot_i2c_reset(struct hobot_i2c_dev *dev)
{
	reset_control_assert(dev->rst);
	udelay(2);
	reset_control_deassert(dev->rst);
	return 0;
}

static int hobot_wait_idle(struct hobot_i2c_dev *dev)
{
	int timeout = WAIT_IDLE_TIMEOUT;
	union status_reg_e status;
	status.all = ioread32(&dev->i2c_regs->status);
	while (status.bit.busy) {
		if (timeout < 0) {
			dev_warn(dev->dev, "timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		usleep_range(1000, 1100);
		status.all = ioread32(&dev->i2c_regs->status);
		timeout--;
	}
	dev->i2c_state = i2c_idle;
	return 0;
}

static void hobot_clear_int(struct hobot_i2c_dev *dev)
{
	iowrite32(0xffffffff, &dev->i2c_regs->srcpnd);
}

static void hobot_mask_int(struct hobot_i2c_dev *dev)
{
	iowrite32(0xffffffff, &dev->i2c_regs->intsetmask);
}

static void hobot_unmask_int(struct hobot_i2c_dev *dev)
{
	union intunmask_reg_e unmask_reg;
	unmask_reg.all = 0;
	unmask_reg.bit.tr_done_mask = 1;
	unmask_reg.bit.to_mask = 1;
	unmask_reg.bit.al_mask = 1;
	unmask_reg.bit.sterr_mask = 1;
	unmask_reg.bit.nack_mask = 1;
	unmask_reg.bit.aerr_mask = 1;
	//TODO: check
	if (dev->i2c_state == i2c_write)
		unmask_reg.bit.xrdy_mask = 1;
	else  if (dev->i2c_state == i2c_read)
		unmask_reg.bit.rrdy_mask = 1;
	else {
		unmask_reg.bit.xrdy_mask = 1;
		unmask_reg.bit.rrdy_mask = 1;
	}
	iowrite32(unmask_reg.all, &dev->i2c_regs->intunmask);
}

static void hobot_fill_txfifo(struct hobot_i2c_dev *dev, int hold)
{
	int left = HOBOT_I2C_FIFO_SIZE;
	union status_reg_e status;

	if (hold) {
		iowrite32(0x1, &dev->i2c_regs->fifo_ctl);
	}

	while (dev->tx_remaining && left) {
		status.all = ioread32(&dev->i2c_regs->status);
		if (status.bit.tx_full) {
			dev_dbg(dev->dev, "tx full\n");
			break;
		}
		iowrite32(*(dev->tx_buf), &dev->i2c_regs->tdata);
		dev->tx_buf++;
		dev->tx_remaining--;
		left--;
	}
	if (hold) {
		iowrite32(0x0, &dev->i2c_regs->fifo_ctl);
	}
}

static void hobot_drain_rxfifo(struct hobot_i2c_dev *dev, int hold)
{
	int left = HOBOT_I2C_FIFO_SIZE;
	union status_reg_e status;

	if (hold) {
		iowrite32(0x2, &dev->i2c_regs->fifo_ctl);
	}

	while (dev->rx_remaining && left) {
		status.all = ioread32(&dev->i2c_regs->status);
		if (status.bit.rx_empty) {
			dev_dbg(dev->dev, "ex empty\n");
			break;
		}
		*(dev->rx_buf) = ioread32(&dev->i2c_regs->rdata) & 0xff;
		dev->rx_buf++;
		dev->rx_remaining--;
		left--;
	}

	if (hold) {
		iowrite32(0x0, &dev->i2c_regs->fifo_ctl);
	}
}

/*
 * i2c diag msg send
 */
static void hobot_i2c_diag_process(u32 errsta, struct hobot_i2c_dev *i2c_contro)
{
	u8 sta;
	u8 envgen_timing;
	u32 int_status;
	u8 envdata[5]; // channel num + srcpnd reg value.
	u8 i2c_event;

	i2c_event = EventIdI2cController0Err + i2c_contro->i2c_id;
	if (errsta) {
		sta = DiagEventStaFail;
		envgen_timing = DiagGenEnvdataWhenErr;
		int_status = i2c_contro->msg_err;
		envdata[0] = i2c_contro->i2c_id;
		memcpy(envdata + 1, (uint8_t *)&int_status, sizeof(u32));
		diag_send_event_stat_and_env_data(DiagMsgPrioHigh,
						ModuleDiag_i2c, i2c_event, sta,
						envgen_timing, envdata, 5);
	} else {
		sta = DiagEventStaSuccess;
		diag_send_event_stat(DiagMsgPrioHigh, ModuleDiag_i2c,
								i2c_event, sta);
	}
}

static irqreturn_t hobot_i2c_isr(int this_irq, void *data)
{
	struct hobot_i2c_dev *dev = data;
	u32 err, comp = 1;
	union sprcpnd_reg_e int_status;

	disable_irq_nosync(this_irq);

	dev_dbg(dev->dev, "hobot_i2c_isr %x %x\n",
			ioread32(&dev->i2c_regs->status), ioread32(&dev->i2c_regs->srcpnd));
	hobot_mask_int(dev);
	int_status.all = ioread32(&dev->i2c_regs->srcpnd);
	hobot_clear_int(dev);

	err = int_status.bit.nack | int_status.bit.sterr | int_status.bit.al |
			int_status.bit.to | int_status.bit.aerr;

	if (err) {
		dev->msg_err = int_status.all;
		dev_dbg(dev->dev, "i2c isr err:%x\n", dev->msg_err);
	} else {
		if (int_status.bit.tr_done || int_status.bit.rrdy || int_status.bit.xrdy) {
			if (dev->i2c_state == i2c_read) {
				if (int_status.bit.tr_done)
					hobot_drain_rxfifo(dev, 0);
				else
					hobot_drain_rxfifo(dev, 1);
				comp = (dev->rx_remaining) ? 0 : 1;
			} else if (dev->i2c_state == i2c_write) {
				comp = (dev->tx_remaining) ? 0 : 1;
				if (int_status.bit.tr_done)
					hobot_fill_txfifo(dev, 0);
				else
					hobot_fill_txfifo(dev, 1);
			} else if (dev->i2c_state == i2c_write_read) {
				if (int_status.bit.xrdy) {
					hobot_fill_txfifo(dev, 1);
				} else if (int_status.bit.tr_done){
					hobot_drain_rxfifo(dev, 0);
				} else {
					hobot_drain_rxfifo(dev, 1);
				}
				comp = (dev->rx_remaining) ? 0 : 1;
			} else {
				;
			}
			if (!comp) {
				hobot_unmask_int(dev);
			}
		}

	}

	hobot_i2c_diag_process(err, dev);

	if (comp)
		complete(&dev->completion);
	enable_irq(this_irq);
	dev_dbg(dev->dev, "i2c isr end\n");
	return IRQ_HANDLED;
}

static int hobot_i2c_xfer_msg(struct hobot_i2c_dev *dev, struct i2c_msg *msg)
{
	unsigned long time_left;
	union ctl_reg_e ctl_reg;
	union dcount_reg_e dcount_reg;

	dev_dbg(dev->dev, "hobot_i2c_xfer_msg %x %x\n", msg->flags, msg->len);

	dev->tx_buf = msg->buf;
	dev->rx_buf = msg->buf;
	dev->msg_err = 0;

	reinit_completion(&dev->completion);
	if (hobot_wait_idle(dev))
		return -ETIMEDOUT;

	hobot_mask_int(dev);
	hobot_clear_int(dev);

	ctl_reg.all = 0;
	dcount_reg.all = 0;
	if (msg->flags & 0x20) {
		/* gw5200 special case */
		dev->tx_remaining = msg->len >> 8;
		dev->rx_remaining = msg->len & 0xff;
		ctl_reg.bit.rd = 1;
		dcount_reg.all = dev->rx_remaining << 16 | dev->tx_remaining;
		iowrite32(dcount_reg.all, &dev->i2c_regs->dcount);
		hobot_fill_txfifo(dev, 0);
		dev->i2c_state = i2c_write_read;
	} else {
		if (msg->flags & I2C_M_RD) {
			dev->rx_remaining = msg->len;
			dev->tx_remaining = 0;
			ctl_reg.bit.rd = 1;
			dcount_reg.bit.r_dcount = msg->len;
			iowrite32(dcount_reg.all, &dev->i2c_regs->dcount);
			dev->i2c_state = i2c_read;
		} else {
			dev->tx_remaining = msg->len;
			dev->rx_remaining = 0;
			ctl_reg.bit.wr = 1;
			dcount_reg.bit.w_dcount = msg->len;
			iowrite32(dcount_reg.all, &dev->i2c_regs->dcount);
			hobot_fill_txfifo(dev, 0);
			dev->i2c_state = i2c_write;
		}
	}

	if (msg->flags & I2C_M_TEN) {
		iowrite32((msg->addr << 1) | BIT(11), &dev->i2c_regs->addr);
	} else {
		iowrite32(msg->addr << 1, &dev->i2c_regs->addr);
	}

	ctl_reg.bit.sta = 1;
	ctl_reg.bit.sto = 1;
	dev_dbg(dev->dev, "ctl_reg.all %x\n", ctl_reg.all);

	hobot_unmask_int(dev);
	iowrite32(ctl_reg.all, &dev->i2c_regs->ctl);
	time_left = wait_for_completion_timeout(&dev->completion,
						msecs_to_jiffies(XFER_TIMEOUT));

	ctl_reg.all = 0;
	ctl_reg.bit.rfifo_clr = 1;
	ctl_reg.bit.tfifo_clr = 1;
	iowrite32(ctl_reg.all, &dev->i2c_regs->ctl);
	iowrite32(0x0, &dev->i2c_regs->fifo_ctl);

	if (!time_left) {
		dev_dbg(dev->dev, "i2c transfer timed out\n");
		return -ETIMEDOUT;
	}

	if (dev->rx_remaining || dev->tx_remaining) {
		dev_dbg(dev->dev, "i2c transfer not complete\n");
	}

	if (likely(!dev->msg_err))
		return 0;

	dev_dbg(dev->dev, "i2c transfer failed: %x\n", dev->msg_err);
	return -EIO;
}

static void recal_clk_div(struct hobot_i2c_dev *dev)
{
	u32 clk_freq = 0;
	int temp_div = 0;
	struct client_request *client_req;

	client_req = (struct client_request *)dev->adapter.algo_data;
	clk_freq = clk_get_rate(dev->clk);
	if (client_req->client_req_freq != 0)
		temp_div = DIV_ROUND_UP(clk_freq, client_req->client_req_freq) - 1;
	else
		temp_div = DIV_ROUND_UP(clk_freq, dev->default_trans_freq) - 1;
	dev->clkdiv = DIV_ROUND_UP(temp_div, 8) - 1;
	if (dev->clkdiv > I2C_MAX_DIV) {
		dev_dbg(dev->dev, "clkdiv too large, set to 255");
		dev->clkdiv = I2C_MAX_DIV;
	}
}

static void reset_client_freq(struct hobot_i2c_dev *dev)
{
	struct client_request *client_req;

	client_req = (struct client_request *)dev->adapter.algo_data;
	client_req->client_req_freq = 0;
}

static int hobot_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct hobot_i2c_dev *dev = i2c_get_adapdata(adap);
	int i;
	int ret = 0;

	if (dev->is_suspended)
		return -EBUSY;

	mutex_lock(&dev->lock);

	hobot_i2c_reset(dev);
	recal_clk_div(dev);
	if (msgs[0].flags & 0x20) {
		hobot_i2c_cfg(dev, 0, 0);
	} else {
		hobot_i2c_cfg(dev, 1, 1);
	}

	for (i = 0; i < num; i++) {
		ret = hobot_i2c_xfer_msg(dev, &msgs[i]);
		if (ret)
			break;
	}

	reset_client_freq(dev);
	mutex_unlock(&dev->lock);

	if (ret == -ETIMEDOUT) {
		usleep_range(500, 1000);
		ret = -EAGAIN;
	}

	return ret ? : i;
}

static int hobot_i2c_doxfer_smbus(struct hobot_i2c_dev *dev, u16 addr, bool write,
									u8 command, int size, u8 *data)
{
	unsigned long time_left;
	union ctl_reg_e ctl_reg;
	union dcount_reg_e dcount_reg;

	reinit_completion(&dev->completion);
	if (hobot_wait_idle(dev))
		return -ETIMEDOUT;

	hobot_mask_int(dev);
	hobot_clear_int(dev);

	dev->msg_err = 0;
	dev->tx_buf = &command;
	dev->tx_remaining = 1;
	ctl_reg.all = 0;
	dcount_reg.all = 0;
	if (write) {
		dcount_reg.bit.w_dcount = size + 1;
		iowrite32(dcount_reg.all, &dev->i2c_regs->dcount);
		hobot_fill_txfifo(dev, 0);
		if (data) {
			dev->tx_remaining = size;
			dev->tx_buf = data;
			hobot_fill_txfifo(dev, 0);
		}
		dev->i2c_state = i2c_write;
		ctl_reg.bit.wr = 1;
	} else {
		dev->rx_remaining = 1;
		dcount_reg.bit.w_dcount = 1;
		iowrite32(dcount_reg.all, &dev->i2c_regs->dcount);
		hobot_fill_txfifo(dev, 0);

		dcount_reg.bit.r_dcount = size;
		iowrite32(dcount_reg.all, &dev->i2c_regs->dcount);
		dev->i2c_state = i2c_read;
		dev->rx_buf = data;
		dev->rx_remaining = size;
		ctl_reg.bit.rd = 1;
	}

	iowrite32(addr << 1, &dev->i2c_regs->addr);

	ctl_reg.bit.sta = 1;
	ctl_reg.bit.sto = 1;
	dev_dbg(dev->dev, "ctl_reg.all %x\n", ctl_reg.all);

	hobot_unmask_int(dev);
	iowrite32(ctl_reg.all, &dev->i2c_regs->ctl);
	time_left = wait_for_completion_timeout(&dev->completion,
						msecs_to_jiffies(XFER_TIMEOUT));

	ctl_reg.all = 0;
	ctl_reg.bit.rfifo_clr = 1;
	ctl_reg.bit.tfifo_clr = 1;
	iowrite32(ctl_reg.all, &dev->i2c_regs->ctl);
	iowrite32(0x0, &dev->i2c_regs->fifo_ctl);

	if (!time_left) {
		dev_dbg(dev->dev, "i2c sbus transfer timed out\n");
		return -ETIMEDOUT;
	}
	if (dev->rx_remaining || dev->tx_remaining) {
		dev_dbg(dev->dev, "i2c sbus transfer not complete\n");
	}
	if (likely(!dev->msg_err))
		return 0;

	dev_dbg(dev->dev, "i2c sbus transfer failed: %x\n", dev->msg_err);
	return -EIO;
}

static int hobot_i2c_xfer_smbus(struct i2c_adapter *adap, u16 addr,
			   unsigned short flags, char read_write,
			   u8 command, int size, union i2c_smbus_data *data)
{
	int ret = 0;
	struct hobot_i2c_dev *dev = i2c_get_adapdata(adap);

	if (dev->is_suspended)
		return -EBUSY;

	mutex_lock(&dev->lock);
	dev_dbg(dev->dev, "hobot_i2c_xfer_smbus addr:%x cmd:%d rw:%d size:%d\n",
				addr, command, read_write, size);

	hobot_i2c_reset(dev);
	recal_clk_div(dev);
	hobot_i2c_cfg(dev, 0, 1);

	switch (size) {
	case I2C_SMBUS_BYTE:
		ret = hobot_i2c_doxfer_smbus(dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, 0, &data->byte);
		break;
	case I2C_SMBUS_BYTE_DATA:
		ret = hobot_i2c_doxfer_smbus(dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, 1, &data->byte);
		break;
	case I2C_SMBUS_WORD_DATA:
		ret = hobot_i2c_doxfer_smbus(dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, 2, (u8 *)&data->word);
		break;
	case I2C_SMBUS_BLOCK_DATA:
		ret = hobot_i2c_doxfer_smbus(dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, data->block[0], &data->block[1]);
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		ret = hobot_i2c_doxfer_smbus(dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, data->block[0], &data->block[1]);
		break;
	default:
		dev_warn(dev->dev, "Unsupported transaction %d\n", size);
		ret = -EOPNOTSUPP;
	}

	reset_client_freq(dev);
	mutex_unlock(&dev->lock);

	if (ret == -ETIMEDOUT) {
		usleep_range(500, 1000);
		ret = -EAGAIN;
	}
	return ret;
}


static u32 hobot_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_10BIT_ADDR | I2C_FUNC_I2C |
		(I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm hobot_i2c_algo = {
	.master_xfer = hobot_i2c_xfer,
#ifndef	CONFIG_HOBOT_FPGA_X3
	.smbus_xfer = hobot_i2c_xfer_smbus,
#endif
	.functionality = hobot_i2c_func,
};

static int hobot_i2c_probe(struct platform_device *pdev)
{
	struct hobot_i2c_dev *dev;
	struct resource *mem, *irq;
	int ret, i2c_id, temp_div;
	struct client_request *client_req = NULL;
	u64 round_rate = 0;
	char i2c_name[20] = {0};
	struct i2c_adapter *adap;

	dev_info(&pdev->dev, "hobot i2c probe start\n");
	dev = devm_kzalloc(&pdev->dev, sizeof(struct hobot_i2c_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err;
	}

	platform_set_drvdata(pdev, dev);
	dev->dev = &pdev->dev;

	mutex_init(&dev->lock);
	init_completion(&dev->completion);
	dev->i2c_state = i2c_idle;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->i2c_regs = devm_ioremap_resource(&pdev->dev, mem);

	i2c_id = of_alias_get_id(pdev->dev.of_node, "i2c");
	sprintf(i2c_name, "i2c%d", i2c_id);
	dev->rst = devm_reset_control_get(&pdev->dev, i2c_name);
	if (IS_ERR(dev->rst)) {
		dev_err(dev->dev, "missing controller reset\n");
		ret = PTR_ERR(dev->rst);
		goto err;
	}

	dev->clk = devm_clk_get(&pdev->dev, "i2c_mclk");
	if (IS_ERR(dev->clk)) {
		dev_err(&pdev->dev, "failed to get i2c_mclk\n");
		ret = PTR_ERR(dev->clk);
		goto err;
	}

	ret = clk_prepare_enable(dev->clk);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to prepare i2c_clk\n");
		goto err_clk;
	}
	clk_disable_unprepare(dev->clk);
	round_rate = clk_round_rate(dev->clk, I2C_CONTROL_CLK);
	dev_info(&pdev->dev, "round_rate:%llu\n", round_rate);
	ret = clk_set_rate(dev->clk, round_rate);
	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "failed to set clk: i2c0_clk\n");
		goto err_clk;
	}

	ret = clk_prepare_enable(dev->clk);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to prepare i2c_clk\n");
		goto err_clk;
	}

	dev->default_trans_freq = I2C_SCL_DEFAULT_FREQ;

	temp_div = DIV_ROUND_UP(round_rate, dev->default_trans_freq) - 1;
	dev->clkdiv = DIV_ROUND_UP(temp_div, 8) - 1;
	if (dev->clkdiv > I2C_MAX_DIV) {
		dev_err(&pdev->dev, "clkdiv too large, set to 255");
		dev->clkdiv = I2C_MAX_DIV;
	}

	dev_info(&pdev->dev, "clkdiv = 0x%x\n", dev->clkdiv);

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		ret = -ENODEV;
		goto err_clk;
	}
	dev->irq = irq->start;

	ret = request_irq(dev->irq, hobot_i2c_isr, IRQF_TRIGGER_HIGH,
			  dev_name(&pdev->dev), dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		ret = -ENODEV;
		goto err_irq;
	}

	if (NULL == client_req)
		client_req = (struct client_request*)
						kmalloc(sizeof(struct client_request), GFP_KERNEL);
	if (!client_req) {
		dev_err(&pdev->dev, "func:%s : there is no memory", __FUNCTION__);
		ret = -ENOMEM;
		goto err_irq;
	}

	client_req->client_req_freq = 0;
	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	strlcpy(adap->name, "Hobot I2C adapter", sizeof(adap->name));
	adap->algo = &hobot_i2c_algo;
	adap->algo_data = (void*)client_req;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	adap->timeout = 4000;
	adap->retries = 3;

	ret = i2c_add_adapter(adap);
	if (ret)
		goto err_irq;

	/*
	 * diag ref init
	 */
	dev->i2c_id = i2c_id;
	dev_info(&pdev->dev, "i2c%d diag register....\n", i2c_id);
	if (diag_register(ModuleDiag_i2c, EventIdI2cController0Err + i2c_id,
			5, 300, 4000, NULL) < 0) {
		dev_err(&pdev->dev, "i2c%d diag register fail\n",
				EventIdI2cController0Err + i2c_id);
	}
	dev_info(&pdev->dev, "hobot_i2c_%d probe done\n", i2c_id);
	return 0;

err_irq:
	free_irq(dev->irq, dev);
err_clk:
	clk_disable_unprepare(dev->clk);
err:
	return ret;
}

static int hobot_i2c_remove(struct platform_device *pdev)
{
	struct hobot_i2c_dev *dev = platform_get_drvdata(pdev);

	free_irq(dev->irq, dev);
	i2c_del_adapter(&dev->adapter);
	if (dev->adapter.algo_data) {
		kfree(dev->adapter.algo_data);
		dev->adapter.algo_data = NULL;
	}

	return 0;
}

static const struct of_device_id hobot_i2c_of_match[] = {
	{.compatible = "hobot,hobot-i2c"},
	{},
};

MODULE_DEVICE_TABLE(of, hobot_i2c_of_match);

static struct platform_driver hobot_i2c_driver = {
	.probe = hobot_i2c_probe,
	.remove = hobot_i2c_remove,
	.driver = {
		   .name = "hobot-i2c",
		   .of_match_table = hobot_i2c_of_match,
		   },
};

static int __init hobot_i2c_init(void)
{
	return platform_driver_register(&hobot_i2c_driver);
}

static void __init hobot_i2c_exit(void)
{
	platform_driver_unregister(&hobot_i2c_driver);
}

subsys_initcall(hobot_i2c_init);
module_exit(hobot_i2c_exit);

MODULE_AUTHOR("Taochao");
MODULE_AUTHOR("qiang.yu@horizon.ai");
MODULE_DESCRIPTION("Hobot I2C bus adapter");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:hobot-i2c");
