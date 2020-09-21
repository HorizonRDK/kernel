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

#define pr_fmt(fmt) "hobot-pvt: %s: " fmt, __func__

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include "hobot-pvt.h"

#define HOBOT_PVT_NAME "pvt"
#define HOBOT_PVT_TEMP_NAME "pvt_ts"
#define PVT_SAMPLE_INTERVAL_MS 20
#define PVT_TS_MASK GENMASK(PVT_TS_NUM - 1, 0)

/* skip sample when calculated temp higher than 152C */
static int smpl_threshold = 3900;
module_param(smpl_threshold, int, 0644);

/* Max diff in Celsius */
static int max_diff = 20000;
module_param(max_diff, int, 0644);

/* For test, 0: default; 1: force calib, 2: force uncalib */
static int force_calib = 0;
module_param(force_calib, int, 0644);

/* TS0: CNN0 TS1:CPU TS2:CNN1 TS3: DDR */
static char *ts_map[] = {
	"CNN0",
	"CPU ",
	"CNN1",
	"DDR ",
};

struct pvt_device {
	int irq;
	long cur_temp[PVT_TS_NUM];
	int cur_smpl[PVT_TS_NUM];
	long cur_temp_avg;
	long ref_clk;
	struct device *dev;
	struct clk *clk;
	struct timer_list irq_unmask_timer;
	void __iomem *reg_base;
	void __iomem *efuse_base;
	int updated;
	int ts_mode;
	int cal_A[PVT_TS_NUM];
	int cal_B[PVT_TS_NUM];
	spinlock_t lock;
};

static inline u32 pvt_reg_rd(struct pvt_device *dev, u32 reg)
{
	return ioread32(dev->reg_base + reg);
}

static inline void pvt_reg_wr(struct pvt_device *dev, u32 reg, u32 val)
{
	iowrite32(val, dev->reg_base + reg);
}

/* pvt_n_reg_rd/wr are only use for TS and PD */
static inline u32 pvt_n_reg_rd(struct pvt_device *dev, int num,  u32 reg)
{
	return ioread32(dev->reg_base + reg + num * 0x40);
}

static inline void pvt_n_reg_wr(struct pvt_device *dev, int num,  u32 reg, u32 val)
{
	iowrite32(val, dev->reg_base + reg + num * 0x40);
}


#define ABN_MAGIC -99999
/* return 1 if got abnormal value; 0 if no abnormal value */
int fix_abnormal_temp_value(struct pvt_device *pvt_dev)
{
	long *ts = pvt_dev->cur_temp;
	long avg = 0;
	long abn = ABN_MAGIC;
	long diff;
	long sum_temp = 0;
	u32 sum_smpl = 0;
	int i;
	int abn_i = -1;

	avg = pvt_dev->cur_temp_avg;

	for (i = 0; i < PVT_TS_NUM; i++) {
		diff = abs(ts[i] -avg);

		/* Pick the max deviation sample */
		if (diff > max_diff) {
			if (abn == ABN_MAGIC || (diff > abs(abn -avg))) {
				abn = ts[i];
				abn_i = i;
			}
		}
	}

	/* Set the max deviation sample to average of others*/
	if (abn > ABN_MAGIC && abn_i >= 0) {
		for (i = 0; i < PVT_TS_NUM; i++) {
			if (i == abn_i)
				continue;
			sum_temp += pvt_dev->cur_temp[i];
			sum_smpl += pvt_dev->cur_smpl[i];
		}
		avg = sum_temp / (PVT_TS_NUM - 1);
		pvt_dev->cur_temp[abn_i] = avg;
		pvt_dev->cur_smpl[abn_i] = sum_smpl / (PVT_TS_NUM - 1);
		pvt_dev->cur_temp_avg = avg;
		pr_info("Abnormal temperature detected on TS[%d]=%ld, new avg:%ld\n",
				abn_i, abn, avg);
		return 1;
	}

    return 0;
}

static int pvt_temp_read(struct device *dev, enum hwmon_sensor_types type,
		     u32 attr, int channel, long *val)
{
	struct pvt_device *pvt_dev = dev_get_drvdata(dev);
	long sum = 0;
	long temp = 0;
	long temp_min = 0;
	long temp_max = 0;
	long diff;
	unsigned long flags;
	int i = 0;

	if (!pvt_dev->updated) {
		*val = 30000;
		return 0;
	}

	spin_lock_irqsave(&pvt_dev->lock, flags);

	for (i = 0; i < PVT_TS_NUM; i++) {
		if (pvt_dev->ts_mode == 0) {
			pr_debug("TS is running in calibrated mode\n");

			/* ts mode 1, Calibrated mode */
			temp = pvt_dev->cal_A[i] * 10 +
			        (((pvt_dev->cur_smpl[i] * pvt_dev->cal_B[i] * 10)) >> 12)
					     - (((2047 * pvt_dev->cal_B[i] * 10)) >> 12) - 400;
		} else {
			pr_debug("TS is running in uncalibrated mode\n");

			/* ts mode 2, Uncalibrated mode */
			temp = -40400 + 49 * pvt_dev->cur_smpl[i]
			              + (148 * pvt_dev->cur_smpl[i]) / 1000;
		}

		pvt_dev->cur_temp[i] = temp;

		pr_debug("%s cur_smpl[%d] = %d\n", ts_map[i], i, pvt_dev->cur_smpl[i]);
		pr_debug("%s cur_temp[%d] = %ldmC\n", ts_map[i], i, pvt_dev->cur_temp[i]);
		sum += temp;
	}

	pvt_dev->cur_temp_avg = sum >> 2;

	/* cur_smpl, cur_temp and cur_avg could be change in this function */
	fix_abnormal_temp_value(pvt_dev);

	*val = pvt_dev->cur_temp_avg;

	/* collect reasonable diff data */
	temp_min = temp_max = pvt_dev->cur_temp[0];
	for (i = 0; i < PVT_TS_NUM; i++) {
		if (pvt_dev->cur_temp[i] < temp_min ) {
			temp_min = pvt_dev->cur_temp[i];
			continue;
		}

		if (pvt_dev->cur_temp[i] > temp_max ) {
			temp_max = pvt_dev->cur_temp[i];
		}
	}

	diff = temp_max - temp_min;
	if (diff > 2000)
		pr_debug("avg:%ld, min:%ld, max:%ld, diff:%ld\n",
			pvt_dev->cur_temp_avg, temp_min, temp_max, diff);

	spin_unlock_irqrestore(&pvt_dev->lock, flags);

	return  0;
}


static umode_t pvt_is_visible(const void *data,
		enum hwmon_sensor_types type,
		u32 attr, int channel)
{
	return 0444;
}

static const u32 pvt_ts_chip_config[] = {
	HWMON_C_REGISTER_TZ,
	0
};

static const struct hwmon_channel_info pvt_ts_chip = {
	.type = hwmon_chip,
	.config = pvt_ts_chip_config,
};

static const u32 pvt_config[] = {
	HWMON_T_INPUT,
	0
};

static const struct hwmon_channel_info pvt_ts_temp = {
	.type = hwmon_temp,
	.config = pvt_config,
};

static const struct hwmon_channel_info *pvt_ts_info[] = {
	&pvt_ts_chip,
	&pvt_ts_temp,
	NULL
};

static const struct hwmon_ops pvt_ts_hwmon_ops = {
	.is_visible = pvt_is_visible,
	.read = pvt_temp_read,
	.write = NULL,
};

static const struct hwmon_chip_info pvt_chip_info = {
	.ops = &pvt_ts_hwmon_ops,
	.info = pvt_ts_info,
};

static irqreturn_t pvt_irq_handler(int irq, void *dev_id)
{
	struct pvt_device *pvt_dev = dev_id;
	u32 ts_status = 0;
	u32 irq_status = 0;
	u32 sdif_done = 0;
	u32 sdif_data = 0;
	u32 update_ts_bitmap = 0;
	static u32 cnt = 0;
	int i = 0;

	ts_status = pvt_reg_rd(pvt_dev, IRQ_TS_STATUS_ADDR);

	spin_lock(&pvt_dev->lock);
	for (i = 0; i < PVT_TS_NUM; i++) {
		irq_status = pvt_n_reg_rd(pvt_dev, i, TS_n_IRQ_STATUS_ADDR);
		sdif_done = pvt_n_reg_rd(pvt_dev, i, TS_n_SDIF_DONE_ADDR);
		sdif_data = pvt_n_reg_rd(pvt_dev, i, TS_n_SDIF_DATA_ADDR);

		if (irq_status & TP_IRQ_STS_DONE_BIT)
			pvt_n_reg_wr(pvt_dev, i, TS_n_IRQ_CLEAR_ADDR, TP_IRQ_STS_DONE_BIT);

		if (irq_status & TP_IRQ_STS_FAULT_BIT) {
			pvt_n_reg_wr(pvt_dev, i, TS_n_IRQ_CLEAR_ADDR, TP_IRQ_STS_FAULT_BIT);
			pr_debug("smp[%d] TP_IRQ_STS_FAULT_BIT fault\n", i);
			continue;
		}

		if (sdif_data & TS_SDIF_DATA_FAULT_BIT) {
			pr_debug("smp[%d] TS_SDIF_DATA_FAULT_BIT fault\n", i);
			continue;
		}

		if (sdif_done) {
			/* report abnormal value when higher than 121C */
			if (sdif_data > smpl_threshold) {
				if(cnt++ % 10000 == 0)
					pr_debug("SDIF_DATA:%d, last TS[%d]:%d, cnt:%d\n",
						sdif_data, i, pvt_dev->cur_smpl[i], cnt-1);
				update_ts_bitmap |= BIT(i);
			} else {
				pvt_dev->cur_smpl[i] = sdif_data;
				update_ts_bitmap |= BIT(i);
			}
		}
	}
	spin_unlock(&pvt_dev->lock);

	if (update_ts_bitmap == PVT_TS_MASK) {
		/*
		 * mask irq in isr, and unmask it in timer handler to
		 * reduce irq numbers per seconds from 488 to 10
		 */
		pvt_reg_wr(pvt_dev, IRQ_TS_MASK_ADDR, PVT_TS_MASK);
		pvt_dev->updated = 1;
	}

	return IRQ_HANDLED;
}

static void pvt_init_hw(struct pvt_device *pvt_dev)
{
	u32 val;
	int cnt = 0;
	int i;

	/* Confirm PVT alive */
	val = pvt_reg_rd(pvt_dev, PVT_COMP_ID_ADDR);
	dev_dbg(pvt_dev->dev, "COMP_ID: %08x\n", val);

	val = pvt_reg_rd(pvt_dev, PVT_IP_CFG_ADDR);
	dev_dbg(pvt_dev->dev, "IP_CONFIG: %08x\n", val);

	pvt_reg_wr(pvt_dev, PVT_TM_SCRATCH_ADDR, 0x1234ABCD);
	val = pvt_reg_rd(pvt_dev, PVT_TM_SCRATCH_ADDR);
	if (0x1234ABCD == val)
		dev_info(pvt_dev->dev, "SCRATCH test: %08x, pvt is alive.\n", val);

	/* Only enable TS IRQ for thermal driver */
	val = pvt_reg_rd(pvt_dev, IRQ_EN_ADDR);
	val |= IRQ_EN_TS_BIT;
	pvt_reg_wr(pvt_dev, IRQ_EN_ADDR, val);
	val = pvt_reg_rd(pvt_dev, IRQ_EN_ADDR);
	dev_dbg(pvt_dev->dev, "after enabled irq: IRQ_EN_ADDR: %08x\n", val);

	for (i = 0; i < PVT_TS_NUM; i++) {
		val = pvt_n_reg_rd(pvt_dev, i, TS_n_IRQ_ENABLE_ADDR);
		val |= TP_IRQ_EN_FAULT_BIT | TP_IRQ_EN_DONE_BIT |
			   TP_IRQ_EN_ALARMA_BIT | TP_IRQ_EN_ALARMB_BIT;

		pvt_n_reg_wr(pvt_dev, i, TS_n_IRQ_ENABLE_ADDR, val);
		val = pvt_n_reg_rd(pvt_dev, i, TS_n_IRQ_ENABLE_ADDR);
		dev_dbg(pvt_dev->dev, "after enabled TS irq: IRQ_%d_EN_ADDR: %08x\n", i, val);
	}

	/* Configure Clock Synthesizers from pvt spec, 240MHz to 4MHz  */
	pvt_reg_wr(pvt_dev, TS_CMN_CLK_SYNTH_ADDR, 0x01011D1D);

	dev_dbg(pvt_dev->dev, "TS_CLK_SYN: %08x\n", pvt_reg_rd(pvt_dev, TS_CMN_CLK_SYNTH_ADDR));
	dev_dbg(pvt_dev->dev, "TS_SDIF_STATUS: 0x%08x\n", pvt_reg_rd(pvt_dev, TS_CMN_SDIF_STATUS_ADDR));

	/* enable continue mode, 488 samples/s */
	val = pvt_reg_rd(pvt_dev, TS_CMN_SDIF_STATUS_ADDR);
	while (val & (PVT_SDIF_BUSY_BIT | PVT_SDIF_LOCK_BIT)) {
		val = pvt_reg_rd(pvt_dev, TS_CMN_SDIF_STATUS_ADDR);
		if(cnt++ > 100) {
			dev_warn(pvt_dev->dev, "SDIF status busy or lock, status 0x%08x\n", val);
			break;
		}
		udelay(10);
	}

	dev_dbg(pvt_dev->dev, "set to mode :%d\n", pvt_dev->ts_mode);

	/* set temperature sensor mode via SDIF */
	pvt_reg_wr(pvt_dev, TS_CMN_SDIF_ADDR, 0x89000000 | pvt_dev->ts_mode);
	udelay(10);

	/* read back the temperature sensor mode register */
	pvt_reg_wr(pvt_dev, TS_CMN_SDIF_ADDR, 0x81000000);
	udelay(10);

	for (i = 0; i < PVT_TS_NUM; i++) {
		val = pvt_n_reg_rd(pvt_dev, i, TS_n_SDIF_RDATA_ADDR);
		dev_dbg(pvt_dev->dev, "get ip_cfg: TS_%d_SDIF_RDATA_ADDR: 0x%08x\n", i, val);
	}

	/* set ip_ctrl, 0x88000104 is self-clear run once, 0x88000108 run continuously */
	pvt_reg_wr(pvt_dev, TS_CMN_SDIF_ADDR, 0x88000108);

	return;
}

static void irq_unmask_timer_func(unsigned long data)
{
	struct pvt_device *pvt_dev = (struct pvt_device *)data;

	/* unmake ts irq to get samples */
	pvt_reg_wr(pvt_dev, IRQ_TS_MASK_ADDR, 0);

	mod_timer(&pvt_dev->irq_unmask_timer,
			jiffies + msecs_to_jiffies(PVT_SAMPLE_INTERVAL_MS));
}

static int pvt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pvt_device *pvt_dev;
	struct device *hwmon_dev;
	struct resource *res;
	u32 val;
	int i;

	pvt_dev = devm_kzalloc(&pdev->dev, sizeof(*pvt_dev), GFP_KERNEL);
	if (!pvt_dev)
		return -ENOMEM;

	pvt_dev->dev = &pdev->dev;

	spin_lock_init(&pvt_dev->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pvt_dev->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pvt_dev->reg_base))
		return PTR_ERR(pvt_dev->reg_base);

	pr_debug("resource 0 %pr\n", res);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	pvt_dev->efuse_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pvt_dev->efuse_base))
		return PTR_ERR(pvt_dev->efuse_base);

	pr_debug("resource 1 %pr\n", res);

	/* Obtain IRQ line */
	pvt_dev->irq = platform_get_irq(pdev, 0);
	if (pvt_dev->irq < 0) {
		dev_err(&pdev->dev, "Can't get interrupt resource!\n");
		return pvt_dev->irq;
	}

	ret = devm_request_irq(&pdev->dev, pvt_dev->irq, pvt_irq_handler, 0, pdev->name, pvt_dev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request IRQ.\n");
		return ret;
	}

	pvt_dev->clk = devm_clk_get(&pdev->dev, "sys_pclk");
	if (IS_ERR(pvt_dev->clk)) {
		dev_err(&pdev->dev, "sys_pclk clock not found.\n");
		return PTR_ERR(pvt_dev->clk);
	}

	/*
	 * Use 250MHz when sys_pll is 1.5G
	 * Use clock synth setting for 240Mhz -> 4Mhz in PVT spec
	 */
	pvt_dev->ref_clk = clk_round_rate(pvt_dev->clk, 250000000);
	ret = clk_set_rate(pvt_dev->clk, pvt_dev->ref_clk);
	if (ret) {
		dev_err(&pdev->dev, "set pvt clk rate to %ld failed.\n",
				pvt_dev->ref_clk);
		clk_disable_unprepare(pvt_dev->clk);
		return ret;
	}

	dev_dbg(pvt_dev->dev, "PVT input clk: %lu\n", clk_get_rate(pvt_dev->clk));

	init_timer(&pvt_dev->irq_unmask_timer);
	pvt_dev->irq_unmask_timer.data = (unsigned long)pvt_dev;
	pvt_dev->irq_unmask_timer.function = irq_unmask_timer_func;
	pvt_dev->irq_unmask_timer.expires = jiffies + msecs_to_jiffies(PVT_SAMPLE_INTERVAL_MS);
	add_timer(&pvt_dev->irq_unmask_timer);

	/* use uncalibrated mode by default */
	pvt_dev->ts_mode = 1;

	for(i = 0; i < PVT_TS_NUM; i++) {
		val = ioread32(pvt_dev->efuse_base + i * PVT_TS_NUM);

		pvt_dev->cal_A[i] = ((val & PVT_TS_A_MASK) >> 16);
		pvt_dev->cal_B[i] = (val & PVT_TS_B_MASK);
		if (pvt_dev->cal_A[i] > 0x1500 || pvt_dev->cal_A[i] < 0x500 ||
			pvt_dev->cal_B[i] > 0x6000 || pvt_dev->cal_B[i] < 0x5000 )
			break;

		pr_debug("val:%08x, cal_A[%d]:%d, cal_B[%d]:%d\n",
				val, i, pvt_dev->cal_A[i], i, pvt_dev->cal_B[i]);
	}

	if (PVT_TS_NUM == i) {
		pr_info("TS Calibrated data detected\n");
		pvt_dev->ts_mode = 0;
	}

	/* extra control for mode switching test */
	if (force_calib == 1) {
		pvt_dev->cal_A[0] = 0x100F;
		pvt_dev->cal_B[0] = 0x557A;
		pvt_dev->cal_A[1] = 0x0FFA;
		pvt_dev->cal_B[1] = 0x5565;
		pvt_dev->cal_A[2] = 0x101E;
		pvt_dev->cal_B[2] = 0x558B;
		pvt_dev->cal_A[3] = 0x102A;
		pvt_dev->cal_B[3] = 0x5597;
		pvt_dev->ts_mode = 0;
	} else if (force_calib == 2) {
		pvt_dev->ts_mode = 1;
	}

	pvt_init_hw(pvt_dev);

	hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev,
					HOBOT_PVT_TEMP_NAME, pvt_dev,
					&pvt_chip_info, NULL);
	hobot_vm_probe(&pdev->dev, pvt_dev->reg_base, pvt_dev->efuse_base);

	return PTR_ERR_OR_ZERO(hwmon_dev);


	return ret;
}

static const struct of_device_id pvt_of_match[] = {
	{ .compatible = "hobot,hobot-pvt" },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, pvt_of_match);

static struct platform_driver pvt_driver = {
	.probe = pvt_probe,
	.driver = {
		.name = HOBOT_PVT_NAME,
		.of_match_table = pvt_of_match,
	},
};

module_platform_driver(pvt_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("hobot PVT driver");
MODULE_LICENSE("GPL v2");
