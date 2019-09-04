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
#include "isp.h"
#include "linux/ion.h"

#define USE_ION_MEM
#define ISP_MEM_SIZE 0x800000

/* global variable define */
static struct isp_dev_s *isp_dev;
extern struct ion_device *hb_ion_dev;


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
	int ret = 0;
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
		ret = -ENOMEM;
		goto err_out1;
	}
//      isp_dev->regbase = ioremap(pres->start, resource_size(pres));
	isp_dev->regbase = ioremap_nocache(pres->start, resource_size(pres));
	if (!isp_dev->regbase) {
		dev_err(&pdev->dev, "[%s]unable to map register\n", __func__);
		ret = -ENOMEM;
		goto err_out1;
	}

#ifdef USE_ION_MEM

	if (!hb_ion_dev) {
		dev_err(&pdev->dev, "NO ION device found!!");
		goto err_out1;
	}

	isp_dev->isp_iclient = ion_client_create(hb_ion_dev, "isp");
	if (!isp_dev->isp_iclient) {
		dev_err(&pdev->dev, "Create ISP ion client failed!!");
		ret = -ENOMEM;
		goto err_out1;
	}

	isp_dev->isp_ihandle = ion_alloc(isp_dev->isp_iclient,
		ISP_MEM_SIZE, 0x20, ION_HEAP_CARVEOUT_MASK, 0);
	if (!isp_dev->isp_ihandle || IS_ERR(isp_dev->isp_ihandle)) {
		dev_err(&pdev->dev, "Create ISP ion client failed!!");
		goto err_out2;
	}
	ret = ion_phys(isp_dev->isp_iclient, isp_dev->isp_ihandle->id,
		&isp_dev->mapbase, (size_t *)&isp_dev->memsize);
	if (ret) {
		dev_err(&pdev->dev, "Get buffer paddr failed!!");
		ion_free(isp_dev->isp_iclient, isp_dev->isp_ihandle);
		isp_dev->mapbase = 0;
		isp_dev->memsize = 0;
		goto err_out2;
	}
	isp_dev->vaddr = ion_map_kernel(isp_dev->isp_iclient,
				isp_dev->isp_ihandle);
	if (IS_ERR(isp_dev->vaddr)) {
		dev_err(&pdev->dev, "Get buffer paddr failed!!");
		ion_free(isp_dev->isp_iclient, isp_dev->isp_ihandle);
		isp_dev->mapbase = 0;
		isp_dev->memsize = 0;
		isp_dev->vaddr = NULL;
		goto err_out2;
	}

#else
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
		goto err_out1;
	}
	isp_dev->mapbase = mapaddr.start;
	isp_dev->memsize = resource_size(&mapaddr);
	isp_dev->vaddr = isp_vmap(isp_dev->mapbase, isp_dev->memsize);
#endif
	/*set io addr */
	set_isp_regbase(isp_dev->regbase);

	platform_set_drvdata(pdev, isp_dev);
	ret = isp_model_init();
	if (ret < 0)
		goto err_out3;

	return 0;
err_out3:
#ifdef USE_ION_MEM
	ion_unmap_kernel(isp_dev->isp_iclient, isp_dev->isp_ihandle);
#else
	vm_unmap_ram(isp_dev->vaddr, isp_dev->memsize / PAGE_SIZE);
#endif
err_out2:
#ifdef USE_ION_MEM
	ion_client_destroy(isp_dev->isp_iclient);
#endif
err_out1:
	devm_kfree(&pdev->dev, isp_dev);
	clr_isp_regbase();

	return ret;
}

static int isp_dev_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "[%s] is remove!\n", __func__);
	isp_model_exit();
#ifdef USE_ION_MEM
	if (isp_dev->isp_iclient) {
		if (isp_dev->isp_ihandle) {
			ion_unmap_kernel(isp_dev->isp_iclient,
				isp_dev->isp_ihandle);
			ion_free(isp_dev->isp_iclient, isp_dev->isp_ihandle);
		}

		ion_client_destroy(isp_dev->isp_iclient);
	}
#else
	vm_unmap_ram(isp_dev->vaddr, isp_dev->memsize / PAGE_SIZE);
#endif
	devm_kfree(&pdev->dev, isp_dev);
	clr_isp_regbase();
	isp_dev = NULL;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
int x2_isp_suspend(struct device *dev)
{
	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

	x2_isp_regs_store();

	set_isp_stop();

	return 0;
}

int x2_isp_resume(struct device *dev)
{
	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);

	x2_isp_regs_restore();

	set_isp_start();

	return 0;
}
#endif

static const struct dev_pm_ops x2_isp_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(x2_isp_suspend,
			x2_isp_resume)
};

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
		   .pm = &x2_isp_dev_pm_ops,
		   },
};

module_platform_driver(isp_dev_driver);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("Image Signal Process module");
MODULE_LICENSE("GPL");
