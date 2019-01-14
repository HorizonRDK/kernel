/*  isp_dev.c - a device driver for the iic-bus interface

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

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include "isp_dev.h"
#include "x2_isp.h"

/* global variable define */
static isp_dev_t *pIspDev = NULL;

/* function */
isp_dev_t *isp_get_dev(void)
{
	return pIspDev;
}

static int isp_dev_probe(struct platform_device *pdev)
{
	int             ret;
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

	pIspDev->mapbase  = pres->start;
	pIspDev->regbase  = ioremap(pIspDev->mapbase, X2_ISP_ISP_CONFIG);
	pIspDev->irq      = platform_get_irq(pdev, 0);

	platform_set_drvdata(pdev, pIspDev);

	return 0;
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
	.probe  = isp_dev_probe,
	.remove = isp_dev_remove,
	.driver = {
		.name   = ISP_NAME,
		.of_match_table = isp_dev_match,
	},
};

module_platform_driver(isp_dev_driver);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("Image Signal Process module");
MODULE_LICENSE("GPL");
