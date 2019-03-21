/*  isp_dev.c - a device driver for the iic-bus interface
 *
 *   Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/device.h>
#include <linux/compiler.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <asm-generic/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/mman.h>
#include "isp_dev.h"
#include "x2_isp.h"
#include "isp_base.h"
#include "x2/x2_ips.h"

/* global variable define */
static struct isp_dev_s *isp_dev;

/* function */
struct isp_dev_s *isp_get_dev(void)
{
	return isp_dev;
}

static void *isp_vmap(phys_addr_t start, size_t size)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);
	prot = pgprot_noncached(PAGE_KERNEL);
	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n", __func__,
		       page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;

		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vm_map_ram(pages, page_count, -1, prot);

	kfree(pages);

	return vaddr;
}

static int isp_dev_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *pres;
	struct resource mapaddr;
	struct device_node *np = NULL;

	dev_info(&pdev->dev, "[%s] is start !\n", __func__);

	isp_dev =
		devm_kmalloc(&pdev->dev, sizeof(struct isp_dev_s), GFP_KERNEL);
	if (!isp_dev)
		return -ENOMEM;
	memset(isp_dev, 0, sizeof(struct isp_dev_s));

	pres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pres) {
		devm_kfree(&pdev->dev, isp_dev);
		return -ENOMEM;
	}

	isp_dev->mapbaseio = pres->start;

	/*io map */
/*
 *	isp_dev->regbase = devm_ioremap_resource(&pdev->dev, pres);
	if(isp_dev->regbase == NULL)
	{
		printk(KERN_INFO "unable to map register\n");
	}
*/

	if (!request_mem_region(pres->start, resource_size(pres),
			"X2_ISP_IO")) {
		ret = -1;
		goto err_out2;
	}
//      isp_dev->regbase = ioremap(pres->start, resource_size(pres));
	isp_dev->regbase = ioremap_nocache(pres->start, resource_size(pres));
	if (!isp_dev->regbase) {
		dev_err(&pdev->dev, "[%s]unable to map register\n", __func__);
		ret = -1;
		goto err_out2;
	}

	/*get mmap address */
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");
		ret = -ENODEV;
	}

	ret = of_address_to_resource(np, 0, &mapaddr);
	if (ret) {
		dev_err(&pdev->dev,
			"No memory address assigned to the region\n");
		goto err_out2;
	}
	isp_dev->mapbase = mapaddr.start;
	isp_dev->memsize = resource_size(&mapaddr);
	isp_dev->vaddr = isp_vmap(isp_dev->mapbase, isp_dev->memsize);

	/*set io addr */
	set_isp_regbase(isp_dev->regbase);

	platform_set_drvdata(pdev, isp_dev);

	return 0;

err_out2:
	devm_kfree(&pdev->dev, isp_dev);
	clr_isp_regbase();

	return ret;
}

static int isp_dev_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "[%s] is remove!\n", __func__);
	vm_unmap_ram(isp_dev->vaddr, isp_dev->memsize / PAGE_SIZE);
	devm_kfree(&pdev->dev, isp_dev);
	clr_isp_regbase();
	isp_dev = NULL;
	return 0;
}

static const struct of_device_id isp_dev_match[] = {
	{.compatible = "hobot,x2-isp"},
	{}
};

MODULE_DEVICE_TABLE(of, isp_dev_match);

static struct platform_driver isp_dev_driver = {
	.probe = isp_dev_probe,
	.remove = isp_dev_remove,
	.driver = {
		   .name = ISP_NAME,
		   .of_match_table = isp_dev_match,
		   },
};

module_platform_driver(isp_dev_driver);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("Image Signal Process module");
MODULE_LICENSE("GPL");
