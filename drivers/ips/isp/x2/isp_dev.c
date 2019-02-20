/*	isp_dev.c - a device driver for the iic-bus interface

	Copyright (C) 2018 Horizon Inc.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/fs.h>
#include "isp_dev.h"
#include "x2_isp.h"

/* global variable define */
static isp_dev_t *pIspDev = NULL;

/* function */
isp_dev_t *isp_get_dev(void)
{
	return pIspDev;
}

static int dbg_isp_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "The isp module status:\n");
	return 0;
}

ssize_t isp_debug_write(struct file *file, const char __user *buf, size_t size, loff_t *p)
{
	int i;

	char info[255];
	memset(info, 0, 255);
	if (copy_from_user(info, buf, size))
		return size;
	printk("isp:%s\n", info);
	if (!memcmp(info, "regdump", 7)) {
		for (i = X2_ISP_REG_ISP_CONFIG; i <= X2_ISP_REG_HMP_CDR_BYPASS_LINES; i += 0x4) {
			printk("offset:0x%p, value:0x%x \n", (pIspDev->regbase + i - X2_ISP_REG_ISP_CONFIG), readl(pIspDev->regbase + i - X2_ISP_REG_ISP_CONFIG));
		}
		for (i = X2_ISP_REG_HMP_YCC2_W_00; i <= X2_ISP_REG_ISP_CONFIG_DONE; i += 0x4) {
			printk("offset:0x%p, value:0x%x \n", (pIspDev->regbase + i - X2_ISP_REG_ISP_CONFIG), readl(pIspDev->regbase + i - X2_ISP_REG_ISP_CONFIG));
		}
		return size;
	}

	return size;
}

static int dbg_isp_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_isp_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open = dbg_isp_open,
	.read = seq_read,
	.write = isp_debug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init x2_isp_debuginit(void)
{
	(void)debugfs_create_file("x2_isp", S_IRUGO, NULL, NULL, &debug_fops);
	return 0;
}

static int isp_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *pres;

	printk(KERN_INFO "isp_dev_probe()!\n");

	pIspDev = devm_kmalloc(&pdev->dev, sizeof(isp_dev_t), GFP_KERNEL);
	if (!pIspDev) {
		return -ENOMEM;
	}
	memset(pIspDev, 0, sizeof(isp_dev_t));

	pres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pres) {
		devm_kfree(&pdev->dev, pIspDev);
		return -ENOMEM;
	}

	pIspDev->mapbase = pres->start;
	pIspDev->regbase = devm_ioremap_resource(&pdev->dev, pres);
	pIspDev->irq = platform_get_irq(pdev, 0);

	platform_set_drvdata(pdev, pIspDev);
	x2_isp_debuginit();
	return ret;
}

static int isp_dev_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "isp_dev_remove()!\n");
	devm_kfree(&pdev->dev, pIspDev);
	pIspDev = NULL;
	return 0;
}

static const struct of_device_id isp_dev_match[] = {
	{.compatible = "hobot,x2-isp"},
	{}
};

MODULE_DEVICE_TABLE(of, isp_dev_match);

static struct platform_driver isp_dev_driver = {
	.probe	= isp_dev_probe,
	.remove = isp_dev_remove,
	.driver = {
		.name	= ISP_NAME,
		.of_match_table = isp_dev_match,
	},
};

module_platform_driver(isp_dev_driver);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("Image Signal Process module");
MODULE_LICENSE("GPL");
