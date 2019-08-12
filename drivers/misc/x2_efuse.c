/***************************************************************************
 *			COPYRIGHT NOTICE
 *		Copyright 2018 Horizon Robotics, Inc.
 *			All rights reserved.
 ***************************************************************************/
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/string.h>
#include <linux/delay.h>

#define X2_EFUSE_NAME		"x2-efuse"

/* define reg offset */
#define X2_EFUSE_APB_DATA_BASE   0x00
#define X2_EFUSE_APB_DATA_LEN    64          /* step=4byte,count=64 */
#define X2_EFUSE_WRAP_DATA_BASE  0x100
#define X2_EFUSE_WRAP_DATA_LEN   32          /* setp=4byte,count=32 */
#define X2_EFUSE_TIME_REG_BASE   0x200
#define X2_EFUSE_TIME_REG_LEN    12
#define X2_EFUSE_EN_REG          0x230
#define X2_EFUSE_ST_REG          0x234
#define X2_EFUSE_RESET_REG       0x238
#define X2_EFUSE_PG_ADDR_REG     0x23C
#define X2_EFUSE_WRAP_DONW_REG   0x240
#define X2_EFUSE_WARP_BISRCEDIS  0x244
#define X2_EFUSE_WRAP_EN_REG     0x248

#define X2_EFUSE_APB_OP_EN       0x1
#define X2_EFUSE_OP_DIS          0x0
#define X2_EFUSE_OP_RD           0x2
#define X2_EFUSE_OP_PG           0x3
#define X2_EFUSE_OP_REPAIR_RST   0x800

#define X2_EFUSE_APB_BANK        0
#define X2_EFUSE_WARP_BANK       1
#define X2_EFUSE_RETRYS          10000
#define X2_EFUSE_WORD_LEN        32         /* bits per word */
#define X2_EFUSE_TOTAL_BIT       2048
#define X2_EFUSE_TIME_333M       0

static void __iomem *reg_base;
static void __iomem *wrap_enable;

#define x2efuse_rd(reg)       readl((u8 *)(reg_base + reg))
#define x2efuse_wr(reg, val)  writel((val), (u8 *)(reg_base + reg))

struct resource *reg;
struct resource *wrap;

unsigned int rd_time_cfg[1][12] = {
	{269, 19, 3, 5, 3, 41, 42, 5, 3, 3, 19, 6},
};
unsigned int pg_time_cfg[1][12] = {
	{269, 19, 3, 5, 0, 0, 4002, 5, 0, 3, 18, 6},
};

static int fuse_read(u32 bank, u32 word, u32 *val)
{
	unsigned int i = 0, rv;

	switch (bank) {
	case X2_EFUSE_APB_BANK:
		if (word > (X2_EFUSE_APB_DATA_LEN-1)) {
			pr_info("overflow, total number is %d, word can be 0-%d\n",
X2_EFUSE_APB_DATA_LEN, X2_EFUSE_APB_DATA_LEN-1);
			return -EINVAL;
		}

		x2efuse_wr(X2_EFUSE_EN_REG, X2_EFUSE_OP_DIS);
		x2efuse_wr(X2_EFUSE_WRAP_EN_REG, X2_EFUSE_APB_OP_EN);
		for (i = 0; i < X2_EFUSE_TIME_REG_LEN; i++)
			x2efuse_wr(X2_EFUSE_TIME_REG_BASE+(i*4),
rd_time_cfg[X2_EFUSE_TIME_333M][i]);
		x2efuse_wr(X2_EFUSE_EN_REG, X2_EFUSE_OP_RD);
		*val = x2efuse_rd(X2_EFUSE_APB_DATA_BASE+(word*4));
		x2efuse_wr(X2_EFUSE_EN_REG, X2_EFUSE_OP_DIS);
		break;
	case X2_EFUSE_WARP_BANK:
		if (word > (X2_EFUSE_WRAP_DATA_LEN-1)) {
			pr_info("overflow, total number is %d, word can be 0-%d\n",
X2_EFUSE_WRAP_DATA_LEN, X2_EFUSE_WRAP_DATA_LEN-1);
			return -EINVAL;
		}
		rv = readl((void *)wrap_enable);
		rv &= (~X2_EFUSE_OP_REPAIR_RST);
		writel(rv, (void *)wrap_enable);
		x2efuse_wr(X2_EFUSE_WRAP_EN_REG, 0x0);
		i = 0;
		do {
			udelay(10);
			rv = x2efuse_rd(X2_EFUSE_WRAP_DONW_REG);
			i++;
		} while ((!(rv&0x1)) && (i < X2_EFUSE_RETRYS));
		if (i >= X2_EFUSE_RETRYS) {
			pr_info("wrap read operate timeout!\n");
			rv = readl((void *)wrap_enable);
			rv |= X2_EFUSE_OP_REPAIR_RST;
			writel(rv, (void *)wrap_enable);
			return -EIO;
		}
		*val = x2efuse_rd(X2_EFUSE_WRAP_DATA_BASE+(word*4));
		rv = readl((void *)wrap_enable);
		rv |= X2_EFUSE_OP_REPAIR_RST;
		writel(rv, (void *)wrap_enable);
		break;
	default:
		pr_err("bank=0:read data with apb  interface.\n");
		pr_err("bank=1:read data with warp interface.\n");
		return -EINVAL;
	}

	return 0;
}

static ssize_t apb_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	unsigned int i, ret, val, word = 0;
	char temp[50];

	for (i = 0; i < 64; i++, word++) {
		if (!(i % 4)) {
			sprintf(temp, "\nApb_Word 0x%.8x:", word);
			strcat(buf, temp);
		}

		ret = fuse_read(0, word, &val);
		if (ret != 0)
			pr_err("fuse read error");

		sprintf(temp, " %.8x", val);
		strcat(buf, temp);
	}

	strcat(buf, "\n");

	return strlen(buf);
}

static ssize_t apb_store(struct class *class, struct class_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

static ssize_t wrap_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	unsigned int i, ret, val, word = 0;
	char temp[50];

	for (i = 0; i < 32; i++, word++) {
		if (!(i % 4)) {
			sprintf(temp, "\nWrap_Word 0x%.8x:", word);
			strcat(buf, temp);
		}

		ret = fuse_read(1, word, &val);
		if (ret != 0)
			pr_err("fuse read error");

		sprintf(temp, " %.8x", val);
		strcat(buf, temp);
	}

	strcat(buf, "\n");

	return strlen(buf);
}

static ssize_t wrap_store(struct class *class, struct class_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

static struct class_attribute apb_attribute =
	__ATTR(apb_efuse, 0644, apb_show, apb_store);

static struct class_attribute wrap_attribute =
	__ATTR(wrap_efuse, 0644, wrap_show, wrap_store);

static struct attribute *efuse_attributes[] = {
	&apb_attribute.attr,
	&wrap_attribute.attr,
	NULL
};

static const struct attribute_group efuse_group = {
	.attrs = efuse_attributes,
};

static const struct attribute_group *efuse_attr_group[] = {
	&efuse_group,
	NULL,
};

static struct class efuse_class = {
	.name = "x2_efuse",
	.class_groups = efuse_attr_group,
};

/* Match table for of_platform binding */
static const struct of_device_id x2_efuse_of_match[] = {
	{ .compatible = "hobot,x2-efuse", },
	{}
};
MODULE_DEVICE_TABLE(of, x2_efuse_of_match);

/**
 * x2_efuse_probe - Platform driver probe
 * @pdev: Pointer to the platform device structure
 *
 * Return: 0 on success, negative errno otherwise
 */
static int x2_efuse_probe(struct platform_device *pdev)
{
	int rc;

	reg = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!reg) {
		rc = -ENODEV;
		goto err_out;
	}

	if (!request_mem_region(reg->start, resource_size(reg),
					 "x2_efuse")) {
		return -ENOMEM;
	}

	reg_base = ioremap(reg->start, resource_size(reg));
	if (!reg_base) {
		pr_err("Unable to map reg registers\n");
		release_mem_region(reg->start, resource_size(reg));
		return -ENOMEM;
	}

	wrap = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!wrap) {
		rc = -ENODEV;
		goto err_out;
	}

	wrap_enable = ioremap(wrap->start, resource_size(wrap));
	if (!wrap_enable)
		return -ENOMEM;

	pr_info("x2_efuse probe success");
	return 0;
err_out:
	return rc;
}

static int x2_efuse_remove(struct platform_device *pdev)
{
	release_mem_region(reg->start, resource_size(reg));
	iounmap(reg_base);
	iounmap(wrap_enable);
	return 0;
}

static struct platform_driver x2_efuse_platform_driver = {
	.probe   = x2_efuse_probe,
	.remove  = x2_efuse_remove,
	.driver  = {
		.name = X2_EFUSE_NAME,
		.of_match_table = x2_efuse_of_match,
		//.pm = &x2_efuse_dev_pm_ops,
		},
};

static int __init x2_efuse_init(void)
{
	int retval = 0;

	/* Register the platform driver */
	retval = platform_driver_register(&x2_efuse_platform_driver);
	if (retval)
		pr_err("Unable to register platform driver\n");

	retval = class_register(&efuse_class);
	if (retval < 0)
		return retval;

	return retval;
}

static void __exit x2_efuse_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&x2_efuse_platform_driver);
	class_unregister(&efuse_class);
}

module_init(x2_efuse_init);
module_exit(x2_efuse_exit);

MODULE_DESCRIPTION("Driver for X2 EFUSE");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
