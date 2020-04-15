/*
 * HOBOT PVT driver
 * including Temperature Sensor, Voltage Monitor and Process Detector
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

int test_gain = 0;
module_param(test_gain, int, 0644);

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
	u32 cur_smpl[PVT_TS_NUM];
	long cur_temp_avg;
	long ref_clk;
	struct device *dev;
	struct clk *clk;
	struct timer_list irq_unmask_timer;
	void __iomem *reg_base;
	int ts_mode;
	int updated;
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

	pr_debug("using ts calibration mode %d in [1, 2]\n", pvt_dev->ts_mode + 1);

	spin_lock_irqsave(&pvt_dev->lock, flags);

	for (i = 0; i < PVT_TS_NUM; i++) {
		if (pvt_dev->ts_mode == 0) {
			//ts mode 1
			temp = ((pvt_dev->cur_smpl[i] * 220 - 67 * 4094) >> 12);
		} else {
			//ts mode 2
			/*
			 * use integer instead of float, the difference is less than 1C.
			 * ((int)(pvt_dev->cur_smpl[i] * 204.4 - 43.14 * 4094) >> 12)
			 */
			temp = ((pvt_dev->cur_smpl[i] * 204 - 43 * 4094) >> 12);
		}

		pvt_dev->cur_temp[i] = temp;
		pr_debug("%s cur_smpl[%d] = %d\n", ts_map[i], i, pvt_dev->cur_smpl[i]);
		pr_debug("%s cur_temp[%d] = %ld\n", ts_map[i], i, pvt_dev->cur_temp[i]);
		sum += temp;
	}

	pvt_dev->cur_temp_avg = sum >> 2;
	*val = pvt_dev->cur_temp_avg * 1000 + test_gain;

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
	if (diff > 2)
		pr_info("avg:%ld, min:%ld, max:%ld, diff:%ld\n",
			pvt_dev->cur_temp_avg, temp_min, temp_max, diff);
	else
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
			pr_warn("smp[%d] TP_IRQ_STS_FAULT_BIT fault\n", i);
			continue;
		}

		if (sdif_data & TS_SDIF_DATA_FAULT_BIT) {
			pr_warn("smp[%d] TS_SDIF_DATA_FAULT_BIT fault\n", i);
			continue;
		}

		if (sdif_done) {
			/* abnormal when lower than -29C or higher than 121C */
			if (sdif_data > 3300 || sdif_data < 300) {
				pr_debug("abnormal cur_smpl[%d] : %d, SDIF_DATA:%08x\n",
						i, pvt_dev->cur_smpl[i], sdif_data);
			}

			pvt_dev->cur_smpl[i] = sdif_data;
			update_ts_bitmap |= BIT(i);
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

	/*
	 * TODO CFG alarm if needed
	 */

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

	pvt_dev = devm_kzalloc(&pdev->dev, sizeof(*pvt_dev), GFP_KERNEL);
	if (!pvt_dev)
		return -ENOMEM;

	pvt_dev->ts_mode = 1;
	pvt_dev->dev = &pdev->dev;

	spin_lock_init(&pvt_dev->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pvt_dev->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pvt_dev->reg_base))
		return PTR_ERR(pvt_dev->reg_base);

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
	 * Use 240MHz when sys_pll is 1.2G
	 * Use clock synth setting for 240Mhz -> 4Mhz in PVT spec
	 */
	pvt_dev->ref_clk = clk_round_rate(pvt_dev->clk, 240000000);
	ret = clk_set_rate(pvt_dev->clk, pvt_dev->ref_clk);
	if (ret) {
		dev_err(&pdev->dev, "set pvt clk rate to %ld failed.\n",
				pvt_dev->ref_clk);
		goto probe_clk_failed;
	}

	dev_dbg(pvt_dev->dev, "PVT input clk: %lu\n", clk_get_rate(pvt_dev->clk));

	init_timer(&pvt_dev->irq_unmask_timer);
	pvt_dev->irq_unmask_timer.data = (unsigned long)pvt_dev;
	pvt_dev->irq_unmask_timer.function = irq_unmask_timer_func;
	pvt_dev->irq_unmask_timer.expires = jiffies + msecs_to_jiffies(PVT_SAMPLE_INTERVAL_MS);
	add_timer(&pvt_dev->irq_unmask_timer);

	pvt_init_hw(pvt_dev);

	hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev,
					HOBOT_PVT_TEMP_NAME, pvt_dev,
					&pvt_chip_info, NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);

probe_clk_failed:
	clk_disable_unprepare(pvt_dev->clk);

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
