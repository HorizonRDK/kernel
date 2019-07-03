/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/reset.h>

#include "i2c-x2.h"

#define X2_I2C_FIFO_SIZE 16

unsigned int i2c_debug_ctl = 0;
module_param(i2c_debug_ctl, uint, S_IRUGO | S_IWUSR);
#define I2C_DEBUG_PRINT(format, args...)    \
    do {                                    \
        if(i2c_debug_ctl)                   \
            printk(format, ## args);        \
    } while(0)

#define X2_I2C_TIMEOUT (msecs_to_jiffies(1000))

enum {
	i2c_idle,
	i2c_write,
	i2c_read,
	i2c_write_read,
};

struct x2_i2c_dev_s {
	struct device *dev;
	volatile struct x2_i2c_regs_s *i2c_regs;
	struct reset_control *rst;
	struct clk *clk;
	int irq;
	struct i2c_adapter adapter;
	struct completion completion;
	u32 msg_err;
	u8 *msg_write_buf;
	u8 *msg_read_buf;
	u32 i2c_state;
	u16 msg_flags;
	size_t msg_write_remaining;
	size_t msg_read_remaining;
};

volatile struct x2_i2c_regs_s *g_i2c_regs[4];
void dump_reg(struct x2_i2c_dev_s *i2c_dev)
{
	if (!i2c_debug_ctl)
		return;
	printk("cfg:[%x] \n", i2c_dev->i2c_regs->cfg.all);
	printk("addr:[%x] \n", i2c_dev->i2c_regs->addr.all);
	printk("dcount:[%x] \n", i2c_dev->i2c_regs->dcount.all);
	printk("ctl:[%x] \n", i2c_dev->i2c_regs->ctl.all);
	//printk( "tdata:[%x] \n", g_i2c_regs->tdata.all);
	//printk( "rdata:[%x] \n", g_i2c_regs->rdata.all);
	printk("status:[%x] \n", i2c_dev->i2c_regs->status.all);
	printk("tocnt:[%x] \n", i2c_dev->i2c_regs->tocnt.all);
	printk("srcpnd:[%x] \n", i2c_dev->i2c_regs->srcpnd.all);
	printk("intmask:[%x] \n", i2c_dev->i2c_regs->intmask.all);
	printk("intsetmask:[%x] \n", i2c_dev->i2c_regs->intsetmask.all);
	printk("intunmask:[%x] \n", i2c_dev->i2c_regs->intunmask.all);
	printk("pmu_delay:[%x] \n", i2c_dev->i2c_regs->pmu_delay.all);
	printk("fifo_ctl:[%x] \n", i2c_dev->i2c_regs->fifo_ctl.all);
}

static int x2_i2c_cfg(struct x2_i2c_dev_s *i2c_dev, int dir_rd, int timeout_enable)
{
	union cfg_reg_e cfg;
	if(!i2c_dev)
		return -1;

	cfg.all = 0;
	cfg.bit.tran_en = 1;
	cfg.bit.en = 1;
	if (timeout_enable) {
		cfg.bit.to_en = 1;
	} else {
		cfg.bit.to_en = 0;
	}
	cfg.bit.dir_rd = dir_rd;
	cfg.bit.clkdiv = 0x4f; /* 300KHZ */
	i2c_dev->i2c_regs->tocnt.all = 0xffff;
	i2c_dev->i2c_regs->cfg.all = cfg.all;
	return 0;
}

static int x2_i2c_reset(struct x2_i2c_dev_s *i2c_dev)
{
	reset_control_assert(i2c_dev->rst);
	udelay(2);
	reset_control_deassert(i2c_dev->rst);
	return 0;
}

static void x2_wait_idle(struct x2_i2c_dev_s *i2c_dev)
{
	uint spins = 100000;
	while (i2c_dev->i2c_regs->status.bit.busy && --spins) {
		cpu_relax();
	}
	i2c_dev->i2c_state = i2c_idle;
	if (!spins) {
		I2C_DEBUG_PRINT("i2c wait_idle timed out\n");
		dump_reg(i2c_dev);
		x2_i2c_reset(i2c_dev);
		x2_i2c_cfg(i2c_dev, 0, 1);
	}
}

static void x2_clear_int(struct x2_i2c_dev_s *i2c_dev)
{
	i2c_dev->i2c_regs->srcpnd.all = 0xffffffff;
}

static void x2_mask_int(struct x2_i2c_dev_s *i2c_dev)
{
	i2c_dev->i2c_regs->intsetmask.all = 0xffffffff;
}

static void x2_unmask_int(struct x2_i2c_dev_s *i2c_dev)
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
	if (i2c_dev->i2c_state == i2c_read) {
		unmask_reg.bit.rrdy_mask = 1;
	} else if (i2c_dev->i2c_state == i2c_write) {
		unmask_reg.bit.xrdy_mask = 1;
	} else if (i2c_dev->i2c_state == i2c_write_read) {
		unmask_reg.bit.rrdy_mask = 1;
	}
	//printk("unmask:%x\n",unmask_reg.all);
	i2c_dev->i2c_regs->intunmask.all = unmask_reg.all;
}

static void x2_fill_txfifo(struct x2_i2c_dev_s *i2c_dev, int hold)
{
	int n = X2_I2C_FIFO_SIZE;
	if (hold) {
		i2c_dev->i2c_regs->fifo_ctl.all = 0x1;
	}
	while (i2c_dev->msg_write_remaining && n) {
		if (i2c_dev->i2c_regs->status.bit.tx_full) {
			I2C_DEBUG_PRINT("tx_full\n");
			break;
		}
		i2c_dev->i2c_regs->tdata.all = *(i2c_dev->msg_write_buf);
		i2c_dev->msg_write_buf++;
		i2c_dev->msg_write_remaining--;
		n--;
	}
	if (hold) {
		i2c_dev->i2c_regs->fifo_ctl.all = 0;
	}
}

static void x2_drain_rxfifo(struct x2_i2c_dev_s *i2c_dev, int hold)
{
	u8 val;
	int n = X2_I2C_FIFO_SIZE;

	if (hold) {
		i2c_dev->i2c_regs->fifo_ctl.all = 0x2;
	}
	
	while (n && i2c_dev->msg_read_remaining) {
		if (i2c_dev->i2c_regs->status.bit.rx_empty) {
			I2C_DEBUG_PRINT("rx empty\n");
			break;
		}
		val = i2c_dev->i2c_regs->rdata.all;
		*(i2c_dev->msg_read_buf) = val;
		i2c_dev->msg_read_buf++;
		i2c_dev->msg_read_remaining--;
		n--;
	}

	if (hold) {
		i2c_dev->i2c_regs->fifo_ctl.all = 0;
	}
}

static irqreturn_t x2_i2c_isr(int this_irq, void *data)
{
	struct x2_i2c_dev_s *i2c_dev = data;
	u32 err, comp = 1;
	union sprcpnd_reg_e int_status;
	//union status_reg_e i2c_status;
	disable_irq_nosync(this_irq);

	I2C_DEBUG_PRINT("x2_i2c_isr %x %x\n", i2c_dev->i2c_regs->status.all, i2c_dev->i2c_regs->srcpnd.all);
	x2_mask_int(i2c_dev);
	int_status.all = i2c_dev->i2c_regs->srcpnd.all;
	//i2c_status.all = i2c_dev->i2c_regs->status.all;
	x2_clear_int(i2c_dev);

	err = int_status.bit.nack | int_status.bit.sterr | 
			int_status.bit.al | int_status.bit.to | 
			int_status.bit.aerr;
	I2C_DEBUG_PRINT("status err  %x\n", err);

	if (err) {
		i2c_dev->msg_err = int_status.all;
		I2C_DEBUG_PRINT("i2c isr err:%x\n", i2c_dev->msg_err);
	} else {
		if (int_status.bit.tr_done || int_status.bit.rrdy || int_status.bit.xrdy) {
			if (i2c_dev->i2c_state == i2c_read) {
				if (int_status.bit.tr_done)
					x2_drain_rxfifo(i2c_dev, 0);
				else 
					x2_drain_rxfifo(i2c_dev, 1);
				comp = (i2c_dev->msg_read_remaining) ? 0 : 1;
			} else if (i2c_dev->i2c_state == i2c_write) {
				comp = (i2c_dev->msg_write_remaining) ? 0 : 1;
				if (int_status.bit.tr_done)
					x2_fill_txfifo(i2c_dev, 0);
				else
					x2_fill_txfifo(i2c_dev, 1);
			} else if (i2c_dev->i2c_state == i2c_write_read){
				if (int_status.bit.xrdy) {
					x2_fill_txfifo(i2c_dev, 1);
				} else if (int_status.bit.tr_done){
					x2_drain_rxfifo(i2c_dev, 0);
				} else {
					x2_drain_rxfifo(i2c_dev, 1);
				}
				comp = (i2c_dev->msg_read_remaining) ? 0 : 1;
			} else {
				;
			}
			if (!comp) {
				x2_unmask_int(i2c_dev);
			}
		}
	}
	if (comp) {
		complete(&i2c_dev->completion);
		i2c_dev->msg_write_buf = NULL;
		i2c_dev->msg_read_buf = NULL;
	}

	enable_irq(this_irq);
	I2C_DEBUG_PRINT("x2 irq end\n");
	return IRQ_HANDLED;
}

static int x2_i2c_xfer_msg(struct x2_i2c_dev_s *i2c_dev, struct i2c_msg *msg)
{
	unsigned long time_left;
	union ctl_reg_e ctl_reg;

	i2c_dev->msg_write_buf = msg->buf;
	i2c_dev->msg_read_buf = msg->buf;
	i2c_dev->msg_flags = msg->flags;
	
	reinit_completion(&i2c_dev->completion);
	i2c_dev->msg_err = 0;

	I2C_DEBUG_PRINT("x2_i2c_xfer_msg %x %x\n", msg->flags, msg->len);
	
	x2_mask_int(i2c_dev);
	i2c_dev->i2c_regs->srcpnd.all = 0xffffffff;	//clear int

	x2_wait_idle(i2c_dev);

	ctl_reg.all = 0;

	if (msg->flags & 0x20) {
		i2c_dev->msg_write_remaining = msg->len >> 8;
		i2c_dev->msg_read_remaining = msg->len & 0xff;
		I2C_DEBUG_PRINT("r %lu w %lu\n", i2c_dev->msg_read_remaining, i2c_dev->msg_write_remaining);
		ctl_reg.bit.rd = 1;
		i2c_dev->i2c_regs->dcount.all = i2c_dev->msg_read_remaining << 16 | i2c_dev->msg_write_remaining;
		x2_fill_txfifo(i2c_dev, 0);
		i2c_dev->i2c_state = i2c_write_read;
	} else {
		if (msg->flags & I2C_M_RD) {
			i2c_dev->msg_read_remaining = msg->len;
			i2c_dev->msg_write_remaining = 0;
			ctl_reg.bit.rd = 1;
			i2c_dev->i2c_regs->dcount.bit.r_dcount = msg->len;
			i2c_dev->i2c_state = i2c_read;
		} else {
			i2c_dev->msg_write_remaining = msg->len;
			i2c_dev->msg_read_remaining = 0;
			ctl_reg.bit.wr = 1;
			i2c_dev->i2c_regs->dcount.bit.w_dcount = msg->len;
			x2_fill_txfifo(i2c_dev, 0);
			i2c_dev->i2c_state = i2c_write;
		}
	}

	if (msg->flags & I2C_M_TEN) {
		i2c_dev->i2c_regs->addr.all = (msg->addr << 1) | BIT(11);
	} else {
		i2c_dev->i2c_regs->addr.all = msg->addr << 1;
	}
	ctl_reg.bit.sta = 1;
	ctl_reg.bit.sto = 1;

	if (msg->flags & 0x20) {
		i2c_dev->i2c_regs->intunmask.all = 0xffffffff;
	} else
		x2_unmask_int(i2c_dev);

	dump_reg(i2c_dev);
	I2C_DEBUG_PRINT("ctl_reg.all %x\n", ctl_reg.all);
	i2c_dev->i2c_regs->ctl.all = ctl_reg.all;
	time_left = wait_for_completion_timeout(&i2c_dev->completion,
						X2_I2C_TIMEOUT);
	x2_wait_idle(i2c_dev);

	i2c_dev->i2c_regs->fifo_ctl.all = 0;
	ctl_reg.all = 0;
	ctl_reg.bit.rfifo_clr = 1;
	ctl_reg.bit.tfifo_clr = 1;
	i2c_dev->i2c_regs->ctl.all = ctl_reg.all;
	i2c_dev->i2c_regs->fifo_ctl.all = 0;

	if (!time_left) {
		I2C_DEBUG_PRINT("i2c transfer timed out\n");
		return -ETIMEDOUT;
	}

	if (i2c_dev->msg_read_remaining || i2c_dev->msg_write_remaining) {
		I2C_DEBUG_PRINT("i2c transfer not compelte\n");
	}

	if (likely(!i2c_dev->msg_err))
		return 0;

	x2_clear_int(i2c_dev);

	I2C_DEBUG_PRINT("i2c transfer failed: %x\n", i2c_dev->msg_err);

	return -EIO;
}

static int x2_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct x2_i2c_dev_s *i2c_dev = i2c_get_adapdata(adap);
	int i;
	int ret = 0;
	
	x2_i2c_reset(i2c_dev);
	if (msgs[0].flags & 0x20) {
		x2_i2c_cfg(i2c_dev, 0, 0);
	} else {
		x2_i2c_cfg(i2c_dev, 1, 1);
	}

	for (i = 0; i < num; i++) {
		ret = x2_i2c_xfer_msg(i2c_dev, &msgs[i]);
		if (ret)
			break;
	}
	return ret ? : i;
}

static int x2_i2c_doxfer_smbus(struct x2_i2c_dev_s *i2c_dev, u16 addr, bool write, u8 command, int size, u8 *data)

{
	unsigned long time_left;
	union ctl_reg_e ctl_reg;
	reinit_completion(&i2c_dev->completion);
	i2c_dev->msg_err = 0;
	x2_mask_int(i2c_dev);
	x2_wait_idle(i2c_dev);
	ctl_reg.all = 0;

	i2c_dev->msg_write_buf = &command;
	if (write) {
		i2c_dev->msg_write_remaining = 1;
		i2c_dev->i2c_regs->dcount.bit.w_dcount = size + 1;
		x2_fill_txfifo(i2c_dev, 0);
		if (data) {
			i2c_dev->msg_write_remaining = size;
			i2c_dev->msg_write_buf = data;
			x2_fill_txfifo(i2c_dev, 0);
		}
		i2c_dev->i2c_state = i2c_write;
		ctl_reg.bit.wr = 1;
	} else {
		i2c_dev->msg_read_remaining = 1;
		i2c_dev->i2c_regs->dcount.bit.w_dcount = 1;
		x2_fill_txfifo(i2c_dev, 0);

		i2c_dev->i2c_regs->dcount.bit.r_dcount = size;
		i2c_dev->i2c_state = i2c_read;
		i2c_dev->msg_read_buf = data;
		i2c_dev->msg_read_remaining = size;
		ctl_reg.bit.rd = 1;

	}
	ctl_reg.bit.sta = 1;
	ctl_reg.bit.sto = 1;

	i2c_dev->i2c_regs->addr.all = addr << 1;

	i2c_dev->i2c_regs->srcpnd.all = 0xffffffff;	//clear int
	x2_unmask_int(i2c_dev);
	i2c_dev->i2c_regs->ctl.all = ctl_reg.all;

	time_left = wait_for_completion_timeout(&i2c_dev->completion,
						X2_I2C_TIMEOUT);

	i2c_dev->i2c_regs->ctl.all = ctl_reg.all;
	i2c_dev->i2c_regs->fifo_ctl.all = 0;
	if (!time_left) {
		I2C_DEBUG_PRINT("i2c sbus transfer timed out\n");
		return -ETIMEDOUT;
	}
	if (i2c_dev->msg_read_remaining || i2c_dev->msg_write_remaining) {
		I2C_DEBUG_PRINT("i2c sbus transfer not compelte\n");
	}
	if (likely(!i2c_dev->msg_err))
		return 0;

	x2_clear_int(i2c_dev);

	I2C_DEBUG_PRINT("i2c sbus transfer failed: %x\n", i2c_dev->msg_err);

	return -EIO;
}

static int x2_i2c_xfer_smbus(struct i2c_adapter *adap, u16 addr,
			   unsigned short flags, char read_write,
			   u8 command, int size, union i2c_smbus_data *data)
{
	int ret = 0;
	struct x2_i2c_dev_s *i2c_dev = i2c_get_adapdata(adap);

	x2_i2c_reset(i2c_dev);
	x2_i2c_cfg(i2c_dev, 0, 1);
	I2C_DEBUG_PRINT("i2c sbus transfer start: addr:%x cmd:%d rw:%d size:%d\n", addr, command, read_write, size);

	switch (size) {
	case I2C_SMBUS_BYTE:
		ret = x2_i2c_doxfer_smbus(i2c_dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, 0, &data->byte);
		break;
	case I2C_SMBUS_BYTE_DATA:
		ret = x2_i2c_doxfer_smbus(i2c_dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, 1, &data->byte);
		break;
	case I2C_SMBUS_WORD_DATA:
		ret = x2_i2c_doxfer_smbus(i2c_dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, 2, (u8 *)&data->word);
		break;
	case I2C_SMBUS_BLOCK_DATA:
		ret = x2_i2c_doxfer_smbus(i2c_dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, data->block[0], &data->block[1]);
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		ret = x2_i2c_doxfer_smbus(i2c_dev, addr,
				read_write == I2C_SMBUS_WRITE,
				command, data->block[0], &data->block[1]);
		break;
	default:
		dev_warn(&adap->dev, "Unsupported transaction %d\n", size);
		return -EOPNOTSUPP;
	}
	return ret;
}


static u32 x2_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_10BIT_ADDR | I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm x2_i2c_algo = {
	.master_xfer = x2_i2c_xfer,
	.smbus_xfer = x2_i2c_xfer_smbus,
	.functionality = x2_i2c_func,
};

static int x2_i2c_probe(struct platform_device *pdev)
{
	struct x2_i2c_dev_s *i2c_dev;
	struct resource *mem, *irq;
	//u32 bus_clk_rate, divider;
	int ret, i2c_id;
	char i2c_name[20];
	//union CFG_REG cfg_reg_e;
	struct i2c_adapter *adap;
	printk("x2_i2c_probe start\n");
	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(struct x2_i2c_dev_s), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2c_dev);
	i2c_dev->dev = &pdev->dev;
	init_completion(&i2c_dev->completion);
	i2c_dev->i2c_state = 0;
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2c_dev->i2c_regs = devm_ioremap_resource(&pdev->dev, mem);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	i2c_id = of_alias_get_id(pdev->dev.of_node, "i2c");
	sprintf(i2c_name, "i2c%d", i2c_id);
	i2c_dev->rst = devm_reset_control_get(&pdev->dev, i2c_name);
	if (IS_ERR(i2c_dev->rst)) {
		dev_err(&pdev->dev, "missing controller reset\n");
		return PTR_ERR(i2c_dev->rst);
	}
	x2_i2c_reset(i2c_dev);

	g_i2c_regs[i2c_id] = i2c_dev->i2c_regs;
	if (IS_ERR((const void *)i2c_dev->i2c_regs))
		return PTR_ERR((const void *)i2c_dev->i2c_regs);
#if 0
/*
    i2c_dev->clk = devm_clk_get(&pdev->dev, "i2c");
    if (IS_ERR(i2c_dev->clk)) {
        if (PTR_ERR(i2c_dev->clk) != -EPROBE_DEFER)
            dev_err(&pdev->dev, "Could not get clock\n");
        return PTR_ERR(i2c_dev->clk);
    }
*/
	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				   &bus_clk_rate);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "Could not read clock-frequency property\n");
		bus_clk_rate = 100000;
	}
	//divider = DIV_ROUND_UP(clk_get_rate(i2c_dev->clk), bus_clk_rate);
	divider = DIV_ROUND_UP(24000000, 8 * bus_clk_rate);

	//if (divider & 1)
	//  divider++;
	if ((divider < X2_I2C_CDIV_MIN) || (divider > X2_I2C_CDIV_MAX)) {
		dev_err(&pdev->dev, "Invalid clock-frequency\n");
		return -ENODEV;
	}
	//x2_i2c_writel(i2c_dev, X2_I2C_CFG, divider); //TODO
	//cfg_reg_e.bit.clkdiv = 7;
#endif
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		return -ENODEV;
	}
	i2c_dev->irq = irq->start;

	ret = request_irq(i2c_dev->irq, x2_i2c_isr, IRQF_TRIGGER_HIGH,
			  dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		return -ENODEV;
	}

	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	strlcpy(adap->name, "x2 I2C adapter", sizeof(adap->name));
	adap->algo = &x2_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	x2_i2c_cfg(i2c_dev, 1, 1);

	ret = i2c_add_adapter(adap);
	if (ret)
		free_irq(i2c_dev->irq, i2c_dev);
	printk("x2_i2c_probe done\n");
	return ret;
}

static int x2_i2c_remove(struct platform_device *pdev)
{
	struct x2_i2c_dev_s *i2c_dev = platform_get_drvdata(pdev);

	free_irq(i2c_dev->irq, i2c_dev);
	i2c_del_adapter(&i2c_dev->adapter);

	return 0;
}

static const struct of_device_id x2_i2c_of_match[] = {
	{.compatible = "hobot,x2-i2c"},
	{},
};

MODULE_DEVICE_TABLE(of, x2_i2c_of_match);

static struct platform_driver x2_i2c_driver = {
	.probe = x2_i2c_probe,
	.remove = x2_i2c_remove,
	.driver = {
		   .name = "i2c-x2",
		   .of_match_table = x2_i2c_of_match,
		   },
};

struct kobject *x2_i2c_kobj;
static ssize_t x2_i2c_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	char *s = buf;
	int i2c_id = 0;
	if (!strcmp(attr->attr.name, "i2c0_dbg"))
		i2c_id = 0;
	else if (!strcmp(attr->attr.name, "i2c1_dbg"))
		i2c_id = 1;
	else if (!strcmp(attr->attr.name, "i2c2_dbg"))
		i2c_id = 2;
	else
		i2c_id = 3;
	//struct x2_i2c_regs_s * i2c_regs;
	if (g_i2c_regs[i2c_id]) {
		s += sprintf(s, "cfg:[%x] \n", g_i2c_regs[i2c_id]->cfg.all);
		s += sprintf(s, "addr:[%x] \n", g_i2c_regs[i2c_id]->addr.all);
		s += sprintf(s, "dcount:[%x] \n", g_i2c_regs[i2c_id]->dcount.all);
		s += sprintf(s, "ctl:[%x] \n", g_i2c_regs[i2c_id]->ctl.all);
		s += sprintf(s, "tdata:[%x] \n", g_i2c_regs[i2c_id]->tdata.all);
		s += sprintf(s, "rdata:[%x] \n", g_i2c_regs[i2c_id]->rdata.all);
		s += sprintf(s, "status:[%x] \n", g_i2c_regs[i2c_id]->status.all);
		s += sprintf(s, "tocnt:[%x] \n", g_i2c_regs[i2c_id]->tocnt.all);
		s += sprintf(s, "srcpnd:[%x] \n", g_i2c_regs[i2c_id]->srcpnd.all);
		s += sprintf(s, "intmask:[%x] \n", g_i2c_regs[i2c_id]->intmask.all);
		s += sprintf(s, "intsetmask:[%x] \n", g_i2c_regs[i2c_id]->intsetmask.all);
		s += sprintf(s, "intunmask:[%x] \n", g_i2c_regs[i2c_id]->intunmask.all);
		s += sprintf(s, "pmu_delay:[%x] \n", g_i2c_regs[i2c_id]->pmu_delay.all);
		s += sprintf(s, "fifo_ctl:[%x] \n", g_i2c_regs[i2c_id]->fifo_ctl.all);
	}
	if (s != buf)
		/* convert the last space to a newline */
		*(s - 1) = '\n';

	return (s - buf);
}

static const char *const i2c_tests[] = {
	"cfg",
	"addr",
	"dcount",
	"ctl",
	"tdata",
	"rdata",
	"status",
	"tocnt",
	"srcpnd",
	"intmask",
	"intsetmask",
	"intunmask",
	"pmdelay",
	"fifoctl",
	"init"
};

static ssize_t x2_i2c_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t n)
{
	int level;
	char *p, *tmpv;
	int len;
	uint regvalue;
	int error = -EINVAL;
	int i2c_id = 0;
	if (!strcmp(attr->attr.name, "i2c0_dbg"))
		i2c_id = 0;
	else if (!strcmp(attr->attr.name, "i2c1_dbg"))
		i2c_id = 1;
	else if (!strcmp(attr->attr.name, "i2c2_dbg"))
		i2c_id = 2;
	else
		i2c_id = 3;

	p = memchr(buf, ' ', n);

	len = p ? p - buf : n;
	tmpv = (char *)(buf + len + 1);
	//if (len == strlen(*s) && !strncmp(buf, "init", len));
	regvalue = simple_strtoul(tmpv, &tmpv, 0);
	level = 0;
	for (level = 0; level <= 14; level++)
		if (len == strlen(i2c_tests[level])
		    && !strncmp(buf, i2c_tests[level], len)) {
			error = 0;
			break;
		}
	//printk("level:%d,val:%\n",level,regvalue);
	if (g_i2c_regs[i2c_id])
		*((int *)g_i2c_regs[i2c_id] + level) = regvalue;
	return error ? error : n;
}

static struct kobj_attribute i2c0_dbg_attr = __ATTR(i2c0_dbg, 0644, x2_i2c_show, x2_i2c_store);
static struct kobj_attribute i2c1_dbg_attr = __ATTR(i2c1_dbg, 0644, x2_i2c_show, x2_i2c_store);
static struct kobj_attribute i2c2_dbg_attr = __ATTR(i2c2_dbg, 0644, x2_i2c_show, x2_i2c_store);
static struct kobj_attribute i2c3_dbg_attr = __ATTR(i2c3_dbg, 0644, x2_i2c_show, x2_i2c_store);

static struct attribute *attributes[] = {
	&i2c0_dbg_attr.attr,
	&i2c1_dbg_attr.attr,
	&i2c2_dbg_attr.attr,
	&i2c3_dbg_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attributes,
};

static int __init x2_i2c_init(void)
{
	platform_driver_register(&x2_i2c_driver);
	x2_i2c_kobj = kobject_create_and_add("x2_i2c", NULL);
	if (!x2_i2c_kobj)
		return -ENOMEM;
	return sysfs_create_group(x2_i2c_kobj, &attr_group);
}

static void __init x2_i2c_exit(void)
{
	platform_driver_unregister(&x2_i2c_driver);
}

subsys_initcall(x2_i2c_init);
module_exit(x2_i2c_exit);

MODULE_AUTHOR("Taochao");
MODULE_DESCRIPTION("x2 I2C bus adapter");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c-x2");
