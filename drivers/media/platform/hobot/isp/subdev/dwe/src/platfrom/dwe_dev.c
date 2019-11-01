/*
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
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/dmaengine.h>
#include <linux/compiler.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include "acamera_logger.h"

#include "dwe_dev.h"


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

/* global variable define */
static struct dwe_dev_s dwe_dev;

/* function */
static int dwe_register(struct platform_device *pdev, struct resource *pres, dwe_subdev_s **psdev)
{
	int ret = 0;
	char dwe_name[25];
	dwe_subdev_s *ptr;
	struct resource *irqs;

	ptr = kzalloc(sizeof(dwe_subdev_s), GFP_KERNEL );
	if (ptr == NULL) {
		LOG(LOG_ERR, "kzalloc is failed \n");
		return -ENOMEM;
	}	
	LOG(LOG_DEBUG, "ptr %p \n", ptr);
	//io map	
	ptr->io_paddr = pres->start;
	ptr->io_memsize = resource_size(pres);
	sprintf(dwe_name, "%s_io", pres->name);	
        if (!request_mem_region(pres->start, resource_size(pres),
                dwe_name)) {
		LOG(LOG_ERR, "request_mem_region is failed! \n");
                ret = -ENOMEM;
                goto reqregion_err;
        }

	ptr->io_vaddr = ioremap_nocache(pres->start, resource_size(pres));
        if (!ptr->io_vaddr) {
		LOG(LOG_ERR, "ioremap_nocache is failed! \n");
		ret = -ENOMEM;
		goto map_err;
        }
	//irq num
	irqs = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (irqs < 0) {
		LOG(LOG_ERR, "get irq source failed! \n");
		ret = -ENODEV; 
		goto irq_err;
        }
       	ptr->irq_num = irqs->start;
	
	*psdev = ptr;
	
	LOG(LOG_DEBUG, "%s,io_paddr %lld, io_vaddr %p, irq_num %d\n", pres->name, ptr->io_paddr, ptr->io_vaddr, ptr->irq_num);

	return ret;
irq_err:
	iounmap(ptr->io_vaddr);
map_err:
	release_mem_region(ptr->io_paddr, ptr->io_memsize);
reqregion_err:
	kfree(ptr);
	ptr = NULL;
	return ret;
}

static void dwe_unregister(dwe_subdev_s *ptr)
{
	
	iounmap(ptr->io_vaddr);
	release_mem_region(ptr->io_paddr, ptr->io_memsize);
	kfree(ptr);
	ptr = NULL;
}

static int dwe_dev_probe(struct platform_device *pdev)
{
	int ret = 0;

	char dwe_name[25];
	struct resource *pres;

	LOG(LOG_DEBUG, "dwe_dev_probe!!!\n");
	
	//get name of this dev	
	pres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pres) {
		LOG(LOG_ERR, "get info failed! \n");
		return -ENOMEM;
	}

	if ( strstr(pres->name, "ldc") != 0) {
		if (dwe_dev.ldc_dev != NULL) {
			LOG(LOG_ERR, "ldc_dev is not null, the addr is %p \n", dwe_dev.ldc_dev);
			dwe_unregister(dwe_dev.ldc_dev);
		}
		ret = dwe_register(pdev, pres, &dwe_dev.ldc_dev);
		if (ret < 0) {
			LOG(LOG_ERR, "ldc register failed !\n");
		}
	} else if ( strstr(pres->name, "dis") != 0) {
		if (dwe_dev.dis_dev != NULL) {
			LOG(LOG_ERR, "dis_dev is not null, the addr is %p \n", dwe_dev.dis_dev);
			dwe_unregister(dwe_dev.dis_dev);
		}
		ret = dwe_register(pdev, pres, &dwe_dev.dis_dev);
		if (ret < 0) {
			LOG(LOG_ERR, "dis register failed !\n");
		}
	} else {
		LOG(LOG_ERR, "name is error \n");
		ret = -1;
	}

	return ret;
}

static int dwe_dev_remove(struct platform_device *pdev)
{
	int ret = 0;

	LOG(LOG_DEBUG, "dwe_dev_remove!!!\n");
	
	if (dwe_dev.ldc_dev != NULL) {
		dwe_unregister(dwe_dev.ldc_dev);
	}
	
	if (dwe_dev.dis_dev != NULL) {
		dwe_unregister(dwe_dev.dis_dev);
	}
	return ret;
}

static const struct of_device_id dwe_dev_match[] = {
	{.compatible = X2A_LDC_NAME},
	{.compatible = X2A_DIS_NAME},
	{}
};

MODULE_DEVICE_TABLE(of, dwe_dev_match);

static struct platform_driver dwe_dev_driver = {
	.probe = dwe_dev_probe,
	.remove = dwe_dev_remove,
	.driver = {
		   .name = X2A_DWE_NAME,
		   .of_match_table = dwe_dev_match,
		   },
};

int __init system_dwe_init( struct dwe_dev_s **ptr )
{
	int rc = 0;

	rc = platform_driver_register(&dwe_dev_driver);
	if (rc == 0)
		*ptr = &dwe_dev;
	LOG(LOG_DEBUG, "%s --%d dwe_dev %p !\n", __func__, __LINE__, &dwe_dev);

	return rc;
}
EXPORT_SYMBOL(system_dwe_init);

void __exit system_dwe_exit( void )
{
    platform_driver_unregister( &dwe_dev_driver );
}
EXPORT_SYMBOL(system_dwe_exit);


MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("DWE dev");
MODULE_LICENSE("GPL v2");
