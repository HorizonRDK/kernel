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
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/bitops.h>
#include <linux/io.h>

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
#define X2_TSENSOR_OP_EN       1    //0:power-down mode;1:normal operation
#define X2_TSENSOR_OP_NM       0    //0:normal operation;1:test mode

#define x2_temp_rd(dev, reg)       ioread32((dev)->regs_base + (reg))
#define x2_temp_wr(dev, reg, val)  iowrite32((val), (dev)->regs_base + (reg))

/* x2 temperature sensor private data */
typedef struct x2_temp {
    void __iomem *regs_base; /* virt. address of the control registers */
    int irq;                 /* interrupt */
}x2_temp_s;

static ssize_t x2_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	s16 data;
	x2_temp_s *x2temp = dev_get_drvdata(dev);

	data = x2_temp_rd(x2temp, X2_TSENSOR_DOUT);

	return sprintf(buf, "%d\n", data/8); 
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, x2_temp_show, NULL, 0);

static struct attribute *x2_temp_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(x2_temp);

static void x2_temp_init_hw(struct x2_temp *x2temp)
{
    //u32 val = 0;

	/* enable x2 temperature sensor module */
    x2_temp_wr(x2temp, X2_TSENSOR_EN, X2_TSENSOR_OP_EN);
    /* temperature sensor work on normal mode */
    x2_temp_wr(x2temp, X2_TSENSOR_TM, X2_TSENSOR_OP_NM);

    return;
}

static int x2_temp_probe(struct platform_device *pdev)
{
    //int err;
    struct x2_temp *x2temp;
	struct device *hwmon_dev;
    struct resource *res;

    dev_info(&pdev->dev, "enter x2_temp_probe!\n");
    x2temp = devm_kzalloc(&pdev->dev, sizeof(*x2temp), GFP_KERNEL);
    if (!x2temp)
        return -ENOMEM;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    x2temp->regs_base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(x2temp->regs_base))
        return PTR_ERR(x2temp->regs_base);

    x2temp->irq = platform_get_irq(pdev, 0);
    if (x2temp->irq < 0) {
        dev_warn(&pdev->dev, "Can't get interrupt resource!\n");
        return x2temp->irq;
    }

    x2_temp_init_hw(x2temp);

	hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev, X2_TEMP_NAME,
							   x2temp, x2_temp_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
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
