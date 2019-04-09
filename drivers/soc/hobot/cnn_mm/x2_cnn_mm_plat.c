/*
 * X2 CNN Memory Heap Platform	Driver (found in Hobot Platform)
 *
 * 2017 - 2018 (C) Hobot Inc.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any
 * later version.
 *
 */
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <asm/string.h>
#include <asm/system_misc.h>
#include <asm/barrier.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/printk.h>
#include <asm/memory.h>
#include <linux/genalloc.h>
#include <linux/vmalloc.h>
#include <linux/scatterlist.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>

#include "cnn_mm_heap.h"

struct cnn_device *idev;
EXPORT_SYMBOL(idev);
static int num_heaps;
static struct cnn_heap **heaps;
static u32 phys_offset = 0;

#define X2_CNN_RT_DRV_NAME "cnn-plat"
#if 0
#define CNN0_MODEL_MM_OFFSET	0x40000000
#define CNN0_MODEL_MM_SIZE	0x08000000

#define CNN1_MODEL_MM_OFFSET	0x48000000
#define CNN1_MODEL_MM_SIZE	0x08000000

#define CNN_RT_MM_OFFSET	0x50023000
#define CNN_RT_MM_SIZE		0x07fdd000

#define CNN_RESULT_MM_OFFSET	0x58000000
#define CNN_RESULT_MM_SIZE	0x08000000
#endif
#define CNN_MM_RSV_START        0x40023000
#define CNN_MM_RSV_SZ           0x1FFDD000

static struct cnn_plat_data *x2_cnn_parse_dt(struct platform_device *pdev)
{
	int i = 0, ret = 0;
	const struct device_node *parent = pdev->dev.of_node;
	struct device_node *child = NULL;
	struct cnn_plat_data *pdata = NULL;
	struct cnn_plat_heap *cnn_heaps = NULL;
	struct platform_device *new_dev = NULL;
	u32 val = 0, type = 0;
	const char *name;

	for_each_child_of_node(parent, child)
		num_heaps++;
	if (!num_heaps)
		return ERR_PTR(-EINVAL);

	pr_debug("%s: num_heaps=%d\n", __func__, num_heaps);

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	cnn_heaps = kcalloc(num_heaps, sizeof(struct cnn_plat_heap),
			    GFP_KERNEL);
	if (!cnn_heaps) {
		kfree(pdata);
		return ERR_PTR(-ENOMEM);
	}

	pdata->heaps = cnn_heaps;
	pdata->nr = num_heaps;

	for_each_child_of_node(parent, child) {
		new_dev = of_platform_device_create(child, NULL, &pdev->dev);
		if (!new_dev) {
			pr_err("Failed to create device %s\n", child->name);
			goto out;
		}

		pdata->heaps[i].priv = &new_dev->dev;

		ret = of_property_read_u32(child, "reg", &val);
		if (ret) {
			pr_err("%s: Unable to find reg key, ret=%d", __func__,
			       ret);
			goto out;
		}
		pdata->heaps[i].heap_id = val;

		ret = of_property_read_string(child, "label", &name);
		if (ret) {
			pr_err("%s: Unable to find label key, ret=%d", __func__,
			       ret);
			goto out;
		}
		pdata->heaps[i].heap_name = name;

		ret = of_property_read_u32(child, "type", &type);
		if (ret) {
			pr_err("%s: Unable to find type key, ret=%d", __func__,
			       ret);
			goto out;
		}
		pdata->heaps[i].type = type;
#if 0
		if (!strcmp(pdata->heaps[i].heap_name, "cnn0_model_mm")) {
			pdata->heaps[i].base = CNN0_MODEL_MM_OFFSET;
			pdata->heaps[i].size = CNN0_MODEL_MM_SIZE;
		} else if (!strcmp(pdata->heaps[i].heap_name, "cnn1_model_mm")) {
			pdata->heaps[i].base = CNN1_MODEL_MM_OFFSET;
			pdata->heaps[i].size = CNN1_MODEL_MM_SIZE;
		} else if (!strcmp(pdata->heaps[i].heap_name, "cnn_rt_mm")) {
			pdata->heaps[i].base = CNN_RT_MM_OFFSET;
			pdata->heaps[i].size = CNN_RT_MM_SIZE;
		} else if (!strcmp(pdata->heaps[i].heap_name, "cnn_result_mm")) {
			pdata->heaps[i].base = CNN_RESULT_MM_OFFSET;
			pdata->heaps[i].size = CNN_RESULT_MM_SIZE;
		}
#endif
        if (!strcmp(pdata->heaps[i].heap_name, "cnn_mm")) {
			pdata->heaps[i].base = CNN_MM_RSV_START;
			pdata->heaps[i].size = CNN_MM_RSV_SZ;
        }

		pr_info("%s: heaps[%d]: %s type: %d base: 0x%x size 0x%zx\n",
			__func__, i,
			pdata->heaps[i].heap_name,
			pdata->heaps[i].type,
			pdata->heaps[i].base,
			pdata->heaps[i].size);
		++i;
	}
	return pdata;
out:
	kfree(pdata->heaps);
	kfree(pdata);
	return ERR_PTR(ret);
}

static long x2_cnn_ioctl(struct cnn_client *client, unsigned int cmd,
			   unsigned long arg)
{
        int ret = 0;
        void __user *arg_user;
        arg_user = (void __user *)arg;

        switch (cmd) {
        case CNN_X2_CUSTOM_PHYS:
        {
                struct cnn_phys_data data;
                struct cnn_handle *handle;
                struct dma_buf *dmabuf;

                if (copy_from_user(&data, arg_user, sizeof(data))) {
                        pr_err("%s, PHYS copy_from_user error!\n", __func__);
                        return -EFAULT;
                }

                dmabuf = dma_buf_get((int)data.fd_buffer);
                if (IS_ERR(dmabuf)) {
                        pr_err("%s: dmabuf is error and dmabuf is %p, fd=%d\n",
                                __func__, dmabuf, (int)data.fd_buffer);
                        return PTR_ERR(dmabuf);
                }

                handle = cnn_import_dma_buf(client, dmabuf);
                if (IS_ERR(handle)) {
                        pr_err("%s, PHYS cnn_import_dma_buf error=%p, fd=%d\n",
                                __func__, handle, data.fd_buffer);
                        return PTR_ERR(handle);
                }

                ret = cnn_phys(client, handle, &data.phys, &data.size);
                dma_buf_put(dmabuf);
                cnn_free(client, handle);

                if (ret) {
                        pr_err("%s,  PHYS cnn_phys error=0x%x\n",
                                __func__, ret);
                        return ret;
                } else {
                        data.phys -= phys_offset;
                }

                if (copy_to_user(arg_user, &data, sizeof(data))) {
                        pr_err("%s, PHYS copy_to_user error!\n", __func__);
                        return -EFAULT;
                }

                pr_debug("%s, PHYS paddress=0x%lx size=0x%zx\n", __func__,
                                data.phys, data.size);
                break;
        }
        case CNN_X2_CUSTOM_MSYNC:
        {
                struct cnn_msync_data data;
                if (copy_from_user(&data, arg_user, sizeof(data))) {
                        pr_err("%s, MSYNC, copy_from_user error!\n", __func__);
                        return -EFAULT;
                }

                if (data.vaddr & (PAGE_SIZE - 1)) {
                        pr_err("%s, MSYNC, data.vaddr=0x%lx error!\n", __func__,\
                                data.vaddr);
                        return -EFAULT;
                }

                 __dma_flush_area((const void *)data.vaddr, data.size);
                break;
        }
        case CNN_X2_CUSTOM_INVALIDATE:
        {
                struct dma_buf *dmabuf;
                struct cnn_buffer *buffer;
                unsigned long fd = (unsigned long)arg_user;

                dmabuf = dma_buf_get((int)fd);
                if (IS_ERR(dmabuf)) {
                        pr_err("%s: dmabuf is error and dmabuf is %p, fd=%d\n",
                                __func__, dmabuf, (int)fd);
                        return PTR_ERR(dmabuf);
                }

                buffer = dmabuf->priv;

                dma_sync_sg_for_cpu(NULL, buffer->sg_table->sgl,
                        buffer->sg_table->nents,
                        DMA_FROM_DEVICE);
                dma_buf_put(dmabuf);
                break;
        }
        default:
                pr_err("x2 cnn mm do not support cmd: %d\n", cmd);
                return -ENOTTY;
        }
        
        return ret;
}

struct cnn_client *x2_cnn_client_create(const char *name)
{
	if (IS_ERR(idev)) {
		pr_err("%s, idev is illegal\n", __func__);
		return NULL;
	}
	return cnn_client_create(idev, name);
}
EXPORT_SYMBOL(x2_cnn_client_create);

static int x2_cnn_mm_probe(struct platform_device *pdev)
{
	int i = 0, ret = 0;
	struct cnn_plat_data *pdata = NULL;
	u32 need_free_pdata;
	num_heaps = 0;

	pdata = x2_cnn_parse_dt(pdev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);
	need_free_pdata = 1;


	heaps = kcalloc(pdata->nr, sizeof(struct cnn_heap *), GFP_KERNEL);
	if (!heaps) {
		ret = -ENOMEM;
		goto out1;
	}

	idev = cnn_device_create(&x2_cnn_ioctl);
	if (IS_ERR_OR_NULL(idev)) {
		pr_err("%s,idev is null\n", __func__);
		kfree(heaps);
		ret = PTR_ERR(idev);
		goto out1;
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < num_heaps; i++) {
		struct cnn_plat_heap *heap_data = &pdata->heaps[i];

		heaps[i] = cnn_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			pr_err("%s,heaps is null, i:%d\n", __func__, i);
			ret = PTR_ERR(heaps[i]);
			goto out;
		}
		cnn_device_add_heap(idev, heaps[i]);
	}
	platform_set_drvdata(pdev, idev);

	if (need_free_pdata) {
		kfree(pdata->heaps);
		kfree(pdata);
	}
	return 0;
out:
	for (i = 0; i < num_heaps; i++) {
		if (heaps[i])
			cnn_heap_destroy(heaps[i]);
	}
	kfree(heaps);
out1:
	if (need_free_pdata) {
		kfree(pdata->heaps);
		kfree(pdata);
	}
	return ret;
}

static int x2_cnn_mm_remove(struct platform_device *pdev)
{
	struct cnn_device *idev = platform_get_drvdata(pdev);
	int i;

	cnn_device_destroy(idev);
	for (i = 0; i < num_heaps; i++)
		cnn_heap_destroy(heaps[i]);
	kfree(heaps);

	return 0;
}

static const struct of_device_id x2_cnn_mm_of_match[] = {
	{ .compatible = "hobot,cnn-mm",},
	{}
};

static struct platform_driver x2_cnn_mm_platform_driver = {
	.probe	 = x2_cnn_mm_probe,
	.remove  = x2_cnn_mm_remove,
	.driver  = {
		.name = X2_CNN_RT_DRV_NAME,
		.of_match_table = x2_cnn_mm_of_match,
	},
};

static int __init x2_cnn_mm_init(void)
{
	int retval = 0;

	/* Register the platform driver */
	retval = platform_driver_register(&x2_cnn_mm_platform_driver);
	if (retval)
		pr_err("x2 cnn runtime driver register failed\n");

	return retval;
}

static void __exit x2_cnn_mm_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&x2_cnn_mm_platform_driver);
}

module_init(x2_cnn_mm_init);
module_exit(x2_cnn_mm_exit);

MODULE_DESCRIPTION("Platform Driver For X2 CNN Memory Management");
MODULE_AUTHOR("Hobot Inc.");
MODULE_LICENSE("GPL");
