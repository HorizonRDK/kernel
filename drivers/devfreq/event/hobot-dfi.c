/*
 * Copyright (c) 2020, Horizon.AI Ltd
 * Author: Zhaohui Shi <zhaohui.shi@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/clk.h>
#include <linux/devfreq-event.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#define PORT_NUM 8

#define DDR_PORT_READ_QOS_CTRL 0x0
#define DDR_PORT_WRITE_QOS_CTRL 0x04

#define PERF_MONITOR_ENABLE 0x20
#define PERF_MONITOR_PERIOD 0x24
#define PERF_MONITOR_SRCPND 0x28
#define PERF_MONITOR_ENABLE_INTMASK 0x2c
#define PERF_MONITOR_ENABLE_SETMASK 0x30
#define PERF_MONITOR_ENABLE_UNMASK 0x34

//#define MP_BASE_RID_CFG 0x100
#define MP_BASE_RADDR_TX_NUM 0x110
#define MP_BASE_RDATA_TX_NUM 0x114
#define MP_BASE_RADDR_ST_CYC 0x118
#define MP_BASE_RA2LSTRD_LATENCY 0x11c
//#define MP_BASE_RID_CFG 0x130
#define MP_BASE_WADDR_TX_NUM 0x140
#define MP_BASE_WDATA_TX_NUM 0x144
#define MP_BASE_WADDR_ST_CYC 0x148
#define MP_BASE_WA2BRESP_LATENCY 0x14c

#define MP_REG_OFFSET 0X100

#define RD_CMD_TX_NUM 0x700
#define WR_CMD_TX_NUM 0x704
#define MWR_CMD_TX_NUM 0x708
#define RDWR_SWITCH_NUM 0x70c
#define ACT_CMD_TX_NUM 0x710
#define ACT_CMD_TX_FOR_RD_TX_NUM 0x714
#define WAR_HAZARD_NUM 0x718
#define RAW_HAZARD_NUM 0x71c
#define WAW_HAZARD_NUM 0x720
#define PERCHARGE_CMD_TX_NUM 0x724
#define PERCHARGE_CMD_FOR_RDWR_TX_NUM 0x728

struct ddr_portdata_s {
	unsigned int waddr_num;
	unsigned int wdata_num;
	unsigned int waddr_cyc;
	unsigned int waddr_latency;
	unsigned int raddr_num;
	unsigned int rdata_num;
	unsigned int raddr_cyc;
	unsigned int raddr_latency;
};

struct ddr_monitor_result_s {
	unsigned long long curtime;
	struct ddr_portdata_s portdata[PORT_NUM];
	unsigned int rd_cmd_num;
	unsigned int wr_cmd_num;
	unsigned int mwr_cmd_num;
	unsigned int rdwr_swi_num;
	unsigned int war_haz_num;
	unsigned int raw_haz_num;
	unsigned int waw_haz_num;
	unsigned int act_cmd_num;
	unsigned int act_cmd_rd_num;
	unsigned int per_cmd_num;
	unsigned int per_cmd_rdwr_num;
};

struct dmc_usage {
	u32 access;
	u32 total;
};

/*
 * The dfi controller can monitor DDR load. It has an upper and lower threshold
 * for the operating points. Whenever the usage leaves these bounds an event is
 * generated to indicate the DDR frequency should be changed.
 */
struct hobot_dfi {
	struct devfreq_event_dev *edev;
	struct devfreq_event_desc *desc;
	struct dmc_usage *ch_usages;
	int channel_num;
	struct device *dev;
	void __iomem *base;
	struct clk *clk, *dmc_clk;
	int irq;
	spinlock_t lock;
	/* for record results. */
	struct ddr_monitor_result_s results;
	int record_num, sample_number;
	/* read bytes per transaction */
	int rd_cmd_byte;
	int total;

	unsigned int monitor_period; /* in ms */
};

static void hobot_dfi_start_hardware_counter(struct devfreq_event_dev *edev)
{
	struct hobot_dfi *info = devfreq_event_get_drvdata(edev);
	unsigned long pclk_rate = clk_get_rate(info->clk);

	/* enable count, use software mode
	 * @TODO: Check this algorithm with qishuai.huang
	 */
	writel(pclk_rate / 1000 * info->monitor_period,
		info->base + PERF_MONITOR_PERIOD);
	writel(0xFFF, info->base + PERF_MONITOR_ENABLE);
	writel(0x1, info->base + PERF_MONITOR_ENABLE_UNMASK);
}

static void hobot_dfi_stop_hardware_counter(struct devfreq_event_dev *edev)
{
	struct hobot_dfi *info = devfreq_event_get_drvdata(edev);

	writel(0, info->base + PERF_MONITOR_ENABLE);
	writel(0x1, info->base + PERF_MONITOR_ENABLE_SETMASK);
}

static int hobot_dfi_disable(struct devfreq_event_dev *edev)
{
	struct hobot_dfi *info = devfreq_event_get_drvdata(edev);

	hobot_dfi_stop_hardware_counter(edev);
	clk_disable_unprepare(info->clk);

	return 0;
}

static int hobot_dfi_enable(struct devfreq_event_dev *edev)
{
	struct hobot_dfi *info = devfreq_event_get_drvdata(edev);
	int ret;

	ret = clk_prepare_enable(info->clk);
	if (ret) {
		dev_err(&edev->dev, "failed to enable dfi clk: %d\n", ret);
		return ret;
	}

	hobot_dfi_start_hardware_counter(edev);
	return 0;
}

static int hobot_dfi_set_event(struct devfreq_event_dev *edev)
{
	/* we don't support. */
	return 0;
}

static int hobot_dfi_get_event(struct devfreq_event_dev *edev,
				struct devfreq_event_data *edata)
{
	unsigned long flags;
	struct hobot_dfi *info = devfreq_event_get_drvdata(edev);

	spin_lock_irqsave(&info->lock, flags);
	edata->total_count = info->total;

	edata->load_count = info->results.rd_cmd_num * info->rd_cmd_byte;
	edata->load_count = info->results.wr_cmd_num * 64;
	edata->load_count += info->results.mwr_cmd_num * 64;
	edata->total_count = info->total;
	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}

static void get_port_status(struct hobot_dfi *info, struct ddr_portdata_s *stat,
			void __iomem *base)
{
	stat->raddr_num = readl(base + MP_BASE_RADDR_TX_NUM);
	stat->rdata_num = readl(base + MP_BASE_RDATA_TX_NUM);
	stat->raddr_cyc = readl(base + MP_BASE_RADDR_ST_CYC);
	stat->raddr_latency = readl(base + MP_BASE_RA2LSTRD_LATENCY);

	stat->waddr_num = readl(base + MP_BASE_WADDR_TX_NUM);
	stat->wdata_num = readl(base + MP_BASE_WDATA_TX_NUM);
	stat->waddr_cyc = readl(base + MP_BASE_WADDR_ST_CYC);
	stat->waddr_latency = readl(base + MP_BASE_WA2BRESP_LATENCY);
}

static int ddr_get_port_status(struct hobot_dfi *info)
{
	int i = 0;
	int step;

	struct ddr_monitor_result_s *result =
		&info->results;

	for (i = 0; i < PORT_NUM; i++) {
		step = i * MP_REG_OFFSET;
		if (i >= 6) {
			pr_debug("i:%d step: %08x\n", i, step);
			step += MP_REG_OFFSET;
		}
		get_port_status(info, &result->portdata[i],
				info->base + step);

	}

	result->rd_cmd_num += readl(info->base + RD_CMD_TX_NUM);
	result->wr_cmd_num += readl(info->base + WR_CMD_TX_NUM);
	result->mwr_cmd_num += readl(info->base + MWR_CMD_TX_NUM);
	result->rdwr_swi_num += readl(info->base + RDWR_SWITCH_NUM);
	result->act_cmd_num += readl(info->base + ACT_CMD_TX_NUM);
	result->act_cmd_rd_num += readl(info->base + ACT_CMD_TX_FOR_RD_TX_NUM);
	result->war_haz_num += readl(info->base + WAR_HAZARD_NUM);
	result->raw_haz_num += readl(info->base + RAW_HAZARD_NUM);
	result->waw_haz_num += readl(info->base + WAW_HAZARD_NUM);
	result->per_cmd_num += readl(info->base + PERCHARGE_CMD_TX_NUM);
	result->per_cmd_rdwr_num +=
		readl(info->base + PERCHARGE_CMD_FOR_RDWR_TX_NUM);

	return 0;
}

static irqreturn_t ddr_monitor_isr(int this_irq, void *data)
{
	unsigned long flags;
	struct hobot_dfi *info = data;

	writel(0x1, info->base + PERF_MONITOR_SRCPND);

	spin_lock_irqsave(&info->lock, flags);
	ddr_get_port_status(info);
	spin_unlock_irqrestore(&info->lock, flags);

	return IRQ_HANDLED;
}

static const struct devfreq_event_ops hobot_dfi_ops = {
	.disable = hobot_dfi_disable,
	.enable = hobot_dfi_enable,
	.get_event = hobot_dfi_get_event,
	.set_event = hobot_dfi_set_event,
};

static int hobot_dfi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hobot_dfi *data;
	struct resource *res, *irq;
	struct devfreq_event_desc *desc;
	struct device_node *np = pdev->dev.of_node;

	data = devm_kzalloc(dev, sizeof(struct hobot_dfi), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dmc_clk = devm_clk_get(dev, "dmc_clk");
	if (IS_ERR(data->dmc_clk)) {
		dev_err(dev, "Cannot get the clk dmc_clk\n");
		return PTR_ERR(data->dmc_clk);
	};

	/* every ddr translation will be 64 bit, i.e. 8byte */
	data->total = clk_get_rate(data->dmc_clk) * 8;

	platform_set_drvdata(pdev, data);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		data->base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR_OR_NULL(data->base)) {
		return PTR_ERR(data->base);
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		return -EINVAL;
	}

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0) {
		dev_err(&pdev->dev, "can't find irq\n");
		return data->irq;
	}

	data->clk = devm_clk_get(dev, "pclk_ddr_mon");
	if (IS_ERR(data->clk)) {
		dev_err(dev, "Cannot get the clk dmc_clk\n");
		return PTR_ERR(data->clk);
	};

	data->dev = dev;

	if (devm_request_irq(dev, data->irq, ddr_monitor_isr,
				IRQF_TRIGGER_HIGH, np->name, data)) {
		dev_err(dev, "failed to request IRQ\n");
		return -ENOENT;
	}

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->ops = &hobot_dfi_ops;
	desc->driver_data = data;
	desc->name = np->name;
	data->desc = desc;

	data->edev = devm_devfreq_event_add_edev(&pdev->dev, desc);
	if (IS_ERR(data->edev)) {
		dev_err(&pdev->dev,
			"failed to add devfreq-event device\n");
		return PTR_ERR(data->edev);
	}


	platform_set_drvdata(pdev, data);

	return 0;
}

static const struct of_device_id hobot_dfi_id_match[] = {
	{ .compatible = "hobot,j5-dfi" },
	{ },
};
MODULE_DEVICE_TABLE(of, hobot_dfi_id_match);

static struct platform_driver hobot_dfi_driver = {
	.probe	= hobot_dfi_probe,
	.driver = {
		.name	= "hobot-dfi",
		.of_match_table = hobot_dfi_id_match,
	},
};
module_platform_driver(hobot_dfi_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Zhaohui Shi <zhaohui.shi@horizon.ai>");
MODULE_DESCRIPTION("Hobot DFI driver");
