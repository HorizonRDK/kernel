/*
 * digital temperature sensor driver for X2
 *
 */
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
#include <linux/io.h>
#include <linux/clk.h>

#define X2_TEMP_NAME           "x2_temp"
/* X2 TSENSOR register offsets */
#define X2_TSENSOR_CFG         0x00
#define X2_TSENSOR_EN          0x04
#define X2_TSENSOR_SRCPND      0x08
#define X2_TSENSOR_TM          0x0C
#define X2_TSENSOR_DOUT        0x10
#define X2_TSENSOR_INTMASK     0x20
#define X2_TSENSOR_SETMASK     0x24
#define X2_TSENSOR_UNMASK      0x28
/* X2 TSENSOR operation Macro */
#define X2_TSENSOR_OP_EN       1        /* 0:power-down mode;1:normal operation */
#define X2_TSENSOR_OP_NM       0        /* 0:normal operation;1:test mode */
#define X2_TSENSOR_INT_CLEAN   0x1      /* clear interrupt requset */
#define X2_TSENSOR_INT_EN      0x1      /* enable interrupt */
#define X2_TSENSOR_DEF_CT      0x30000  /* ct=b11 */

#define x2_temp_rd(dev, reg)       ioread32((dev)->regs_base + (reg))
#define x2_temp_wr(dev, reg, val)  iowrite32((val), (dev)->regs_base + (reg))

/* x2 temperature sensor private data */
typedef struct x2_temp {
	int irq;
	s16 cur_temp;
	u32 ref_clk;
	struct clk *clk;
	void __iomem *regs_base;
}x2_temp_s;

static ssize_t x2_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	x2_temp_s *x2temp = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", x2temp->cur_temp/8);
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, x2_temp_show, NULL, 0);

static struct attribute *x2_temp_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(x2_temp);

static irqreturn_t x2_temp_irq_handler(int irq, void *dev_id)
{
	struct x2_temp *x2temp = dev_id;

	/* clear interrupt request and ger temperature */
	x2_temp_wr(x2temp, X2_TSENSOR_SRCPND, X2_TSENSOR_INT_CLEAN);
	x2temp->cur_temp = x2_temp_rd(x2temp, X2_TSENSOR_DOUT);

	return IRQ_HANDLED;
}

static void x2_temp_init_hw(struct x2_temp *x2temp)
{
	u32 val;

	/* config ct & adj */
	val = x2_temp_rd(x2temp, X2_TSENSOR_CFG);
	val |= X2_TSENSOR_DEF_CT;
	x2_temp_wr(x2temp, X2_TSENSOR_CFG, val);
	/* enable x2 temperature sensor module */
	x2_temp_wr(x2temp, X2_TSENSOR_EN, X2_TSENSOR_OP_EN);
	/* temperature sensor work on normal mode */
	x2_temp_wr(x2temp, X2_TSENSOR_TM, X2_TSENSOR_OP_NM);
	/* enable interrupt */
	x2_temp_wr(x2temp, X2_TSENSOR_UNMASK, X2_TSENSOR_INT_EN);

	return;
}

static int x2_temp_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x2_temp *x2temp;
	struct device *hwmon_dev;
	struct resource *res;

	x2temp = devm_kzalloc(&pdev->dev, sizeof(*x2temp), GFP_KERNEL);
	if (!x2temp)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	x2temp->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(x2temp->regs_base))
		return PTR_ERR(x2temp->regs_base);

	/* Obtain IRQ line */
	x2temp->irq = platform_get_irq(pdev, 0);
	if (x2temp->irq < 0) {
		dev_warn(&pdev->dev, "Can't get interrupt resource!\n");
		return x2temp->irq;
	}
	ret = devm_request_irq(&pdev->dev, x2temp->irq, x2_temp_irq_handler, 0, pdev->name, x2temp);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request IRQ.\n");
		return ret;
	}

	/* get the module ref-clk and enabel it */
	x2temp->clk = devm_clk_get(&pdev->dev, "tsensor_clk");
	if (IS_ERR(x2temp->clk)) {
		dev_err(&pdev->dev, "uart_clk clock not found.\n");
		return PTR_ERR(x2temp->clk);
	}
	ret = clk_prepare_enable(x2temp->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable device clock.\n");
		goto probe_clk_failed;
	}
	x2temp->ref_clk = clk_get_rate(x2temp->clk);

	x2_temp_init_hw(x2temp);

	hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev, X2_TEMP_NAME,
				x2temp, x2_temp_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);

probe_clk_failed:
	clk_disable_unprepare(x2temp->clk);

	return ret;
}

static const struct of_device_id x2_temp_of_match[] = {
	{ .compatible = "hobot,x2-temp" },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, x2_temp_of_match);

static struct platform_driver x2_temp_driver = {
	.probe = x2_temp_probe,
	.driver = {
		.name = X2_TEMP_NAME,
		.of_match_table = x2_temp_of_match,
	},
};
module_platform_driver(x2_temp_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("X2 Temperature sensor driver");
MODULE_LICENSE("GPL v2");
