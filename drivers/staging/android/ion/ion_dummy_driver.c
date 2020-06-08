/*
 * drivers/gpu/ion/ion_dummy_driver.c
 *
 * Copyright (C) 2013 Linaro, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/sizes.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include <linux/ion.h>

extern struct ion_device *ion_device_create(long (*custom_ioctl)
				     (struct ion_client *client,
				      unsigned int cmd,
				      unsigned long arg));

struct ion_device *hb_ion_dev;
EXPORT_SYMBOL(hb_ion_dev);
static struct ion_heap **heaps;
struct dma_chan *dma_ch;
static struct mutex dma_lock;
struct completion dma_completion;

static void *chunk_ptr;

static struct ion_platform_heap dummy_heaps[] = {
		{
			.id	= ION_HEAP_TYPE_SYSTEM,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= "system",
		},
		{
			.id	= ION_HEAP_TYPE_SYSTEM_CONTIG,
			.type	= ION_HEAP_TYPE_SYSTEM_CONTIG,
			.name	= "system contig",
		},
		{
			.id	= ION_HEAP_TYPE_CARVEOUT,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= "carveout",
			.size	= SZ_4M,
		},
		{
			.id	= ION_HEAP_TYPE_CHUNK,
			.type	= ION_HEAP_TYPE_CHUNK,
			.name	= "chunk",
			.size	= SZ_4M,
			.align	= SZ_16K,
			.priv	= (void *)(SZ_16K),
		},
		{
			.id	= ION_HEAP_TYPE_DMA,
			.type	= ION_HEAP_TYPE_DMA,
			.name	= "cma",
		},
};

static struct ion_platform_data dummy_ion_pdata = {
	.nr = ARRAY_SIZE(dummy_heaps),
	.heaps = dummy_heaps,
};

static void ion_dma_cb(void *data)
{
	complete(&dma_completion);
}

#define ION_GET_PHY 0
#define ION_CACHE_INVAL 1
#define ION_CACHE_FLUSH 2
#define ION_MEMCPY 3
struct ion_phy_data {
	struct ion_handle *handle;
	phys_addr_t paddr;
	size_t len;
	uint64_t reserved;
};

static long ion_dummy_ioctl(struct ion_client *client,
			    unsigned int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case ION_GET_PHY:
	{
		struct ion_phy_data phy_data;

		if (copy_from_user(&phy_data, (void __user *)arg,
				   sizeof(struct ion_phy_data))) {
			pr_err("%s:copy from user failed\n",
			       __func__);
			return -EFAULT;
		}

		ret = ion_phys(client, (long)phy_data.handle,
			       &phy_data.paddr, &phy_data.len);
		if (copy_to_user((void __user *)arg, &phy_data,
				 sizeof(struct ion_phy_data)))
			return -EFAULT;

		break;
	}
	case ION_CACHE_FLUSH:
	{
		struct ion_phy_data phy_data;
		void *vaddr;
		ret = 0;

		if (copy_from_user(&phy_data, (void __user *)arg,
				   sizeof(struct ion_phy_data))) {
			pr_err("%s:copy from user failed\n",
			       __func__);
			return -EFAULT;
		}
		vaddr = (void *)phy_data.reserved;

		if ((u64) phy_data.paddr == 0) {
			pr_err("%s:%d invalid paddr:%016llx\n",
				__func__, __LINE__, (u64)phy_data.paddr);
			return -EFAULT;
		}

		dma_sync_single_for_device(NULL, phy_data.paddr, phy_data.len, DMA_TO_DEVICE);
		__flush_dcache_area(page_address(pfn_to_page(PHYS_PFN(phy_data.paddr))), phy_data.len);

		break;
	}
	case ION_CACHE_INVAL:
	{
		struct ion_phy_data phy_data;
		ret = 0;

		if (copy_from_user(&phy_data, (void __user *)arg,
				   sizeof(struct ion_phy_data))) {
			pr_err("%s:copy from user failed\n",
			       __func__);
			return -EFAULT;
		}

		if ((u64) phy_data.paddr == 0) {
			pr_err("%s:%d invalid paddr:%016llx\n",
				__func__, __LINE__, (u64)phy_data.paddr);
			return -EFAULT;
		}

		dma_sync_single_for_cpu(NULL, phy_data.paddr, phy_data.len, DMA_FROM_DEVICE);
		__inval_dcache_area(page_address(pfn_to_page(PHYS_PFN(phy_data.paddr))), phy_data.len);

		break;
	}

	case ION_MEMCPY:
	{
		struct ion_phy_data phy_data;
		struct dma_async_tx_descriptor *tx;
		struct dma_device *dma_dev;
		dma_cookie_t cookie;

		ret = 0;

		if (copy_from_user(&phy_data, (void __user *)arg,
				   sizeof(struct ion_phy_data))) {
			pr_err("%s:copy from user failed\n",
			       __func__);
			return -EFAULT;
		}

		if ((u64) phy_data.paddr == 0) {
			pr_err("%s:%d invalid paddr:%016llx\n",
				__func__, __LINE__, (u64)phy_data.paddr);
			return -EFAULT;
		}

		if (!dma_ch) {
			pr_err("no dma can use for memcpy failed\n");
			return -ENODEV;
		}

		dma_dev = dma_ch->device;

		mutex_lock(&dma_lock);
		reinit_completion(&dma_completion);

		tx = dma_dev->device_prep_dma_memcpy(dma_ch, phy_data.paddr, phy_data.reserved, phy_data.len, 0);
		if (!tx) {
			pr_err("%s: ion memcpy device_prep_dma_memcpy failed\n", __func__);
			ret = -EIO;
			mutex_unlock(&dma_lock);
			break;
		}

		tx->callback = ion_dma_cb;
		tx->callback_result = NULL;
		tx->callback_param = NULL;

		cookie = dmaengine_submit(tx);
		ret = dma_submit_error(cookie);
		if (ret) {
			pr_err("ion memcpy dma_submit_error %d\n", cookie);
			ret = -EIO;
			mutex_unlock(&dma_lock);
			break;
		}

		dma_async_issue_pending(dma_ch);

		wait_for_completion(&dma_completion);
		mutex_unlock(&dma_lock);

		break;
	}

	default:
		return -ENOTTY;
	}

	return ret;
}

int ion_check_in_heap_carveout(phys_addr_t start, size_t size)
{
	struct ion_platform_heap *cvt = &dummy_heaps[ION_HEAP_TYPE_CARVEOUT];

	if (start < cvt->base || start + size > cvt->base + cvt->size)
		return -1;

	return 0;
}

EXPORT_SYMBOL_GPL(ion_check_in_heap_carveout);

static int __init ion_dummy_init(void)
{
	struct device_node *node;
	struct resource ion_pool_reserved;
	dma_cap_mask_t mask;
	int i, err = 0;

	hb_ion_dev = ion_device_create(ion_dummy_ioctl);
	heaps = kcalloc(dummy_ion_pdata.nr, sizeof(struct ion_heap *),
			GFP_KERNEL);
	if (!heaps)
		return -ENOMEM;

	/* phase the carveout heap */
	node = of_find_node_by_path("/reserved-memory");
	if (node) {
		node = of_find_compatible_node(node, NULL, "ion-pool");
		if (node) {
			if (!of_address_to_resource(node, 0, &ion_pool_reserved)) {
				dummy_heaps[ION_HEAP_TYPE_CARVEOUT].base
					= ion_pool_reserved.start;
				dummy_heaps[ION_HEAP_TYPE_CARVEOUT].size
					= resource_size(&ion_pool_reserved);
				pr_info("ION Carveout MEM start 0x%llx, size 0x%lx\n",
					dummy_heaps[ION_HEAP_TYPE_CARVEOUT].base,
					dummy_heaps[ION_HEAP_TYPE_CARVEOUT].size);
			} else {
				pr_err("ion_dummy: Could not allocate carveout\n");
			}
		} else {
			pr_err("ion_dummy: not reserve carveout memory\n");
		}
	} else {
		pr_err("ion_dummy: not reserve memory\n");
	}

	/* Allocate a dummy chunk heap */
	chunk_ptr = alloc_pages_exact(
				dummy_heaps[ION_HEAP_TYPE_CHUNK].size,
				GFP_KERNEL);
	if (chunk_ptr)
		dummy_heaps[ION_HEAP_TYPE_CHUNK].base = virt_to_phys(chunk_ptr);
	else
		pr_err("ion_dummy: Could not allocate chunk\n");

	for (i = 0; i < dummy_ion_pdata.nr; i++) {
		struct ion_platform_heap *heap_data = &dummy_ion_pdata.heaps[i];

		if (heap_data->type == ION_HEAP_TYPE_CARVEOUT &&
							!heap_data->base)
			continue;

		if (heap_data->type == ION_HEAP_TYPE_CHUNK && !heap_data->base)
			continue;

		if (heap_data->type == ION_HEAP_TYPE_DMA) {
			ion_add_cma_heaps(hb_ion_dev);
			continue;
		}

		heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(hb_ion_dev, heaps[i]);
	}

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	dma_ch = dma_request_channel(mask, NULL, NULL);
	if (!dma_ch) {
		pr_err("ion_dummy: dma_request_channel failed\n");
		goto err;
	}
	mutex_init(&dma_lock);
	init_completion(&dma_completion);

	return 0;
err:
	for (i = 0; i < dummy_ion_pdata.nr; ++i)
		ion_heap_destroy(heaps[i]);
	kfree(heaps);

	if (chunk_ptr) {
		free_pages_exact(chunk_ptr,
				 dummy_heaps[ION_HEAP_TYPE_CHUNK].size);
		chunk_ptr = NULL;
	}
	return err;
}
subsys_initcall(ion_dummy_init);

static void __exit ion_dummy_exit(void)
{
	int i;

	ion_device_destroy(hb_ion_dev);
	dma_release_channel(dma_ch);

	for (i = 0; i < dummy_ion_pdata.nr; i++)
		ion_heap_destroy(heaps[i]);
	kfree(heaps);

	if (chunk_ptr) {
		free_pages_exact(chunk_ptr,
				 dummy_heaps[ION_HEAP_TYPE_CHUNK].size);
		chunk_ptr = NULL;
	}
}
__exitcall(ion_dummy_exit);
