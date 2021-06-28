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

#define DFT_CMA_CARVEOUT_SIZE	(272 * 0x100000)
static uint64_t cma_reserved_size;
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
		{
			.id	= ION_HEAP_TYPE_CUSTOM,
			.type	= ION_HEAP_TYPE_CUSTOM,
			.name	= "custom",
			.size	= SZ_4M,
		},
		{
			.id	= ION_HEAP_TYPE_CMA_RESERVED,
			.type	= ION_HEAP_TYPE_CMA_RESERVED,
			.name	= "cma_reserved",
			.size	= SZ_4M,
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

struct ion_cma_carveout {
	struct ion_client *i_cc_client;
	struct ion_handle *i_cc_handle;
	enum ion_heap_type heap_type;
	phys_addr_t start;
	size_t size;
	bool cc_is_valid;
};

static struct ion_cma_carveout ion_cc = { 0 };
static int ion_cma_carveout_range_create(size_t size)
{
	int ret, i;

	if (!ion_cc.cc_is_valid) {
		return 0;
	}

	ion_cc.i_cc_client = ion_client_create(hb_ion_dev, "ion_cma_carveout");
	if (ion_cc.i_cc_client == NULL) {
		pr_err("Create ion cma carveout client failed!!\n");
		return -ENOMEM;
	}

	ion_cc.i_cc_handle = ion_alloc(ion_cc.i_cc_client,
			size, 0x10, ION_HEAP_TYPE_DMA_MASK, 0xffff << 16);

	if ((ion_cc.i_cc_handle == NULL) || IS_ERR(ion_cc.i_cc_handle)) {
		ion_client_destroy(ion_cc.i_cc_client);
		ion_cc.i_cc_handle = NULL;
		ion_cc.i_cc_client = NULL;
		pr_err("Alloc ion cma carveout buffer failed!!\n");
		return -ENOMEM;
	}

	ret = ion_phys(ion_cc.i_cc_client, ion_cc.i_cc_handle->id,
			&ion_cc.start, &ion_cc.size);
	if (ret != 0) {
		ion_free(ion_cc.i_cc_client, ion_cc.i_cc_handle);
		ion_client_destroy(ion_cc.i_cc_client);
		ion_cc.i_cc_handle = NULL;
		ion_cc.i_cc_client = NULL;
		pr_err("Alloced ion cma carveout buffer get phys failed!!\n");
		return -ENOMEM;
	}

	dummy_heaps[ion_cc.heap_type].base = ion_cc.start;
	dummy_heaps[ion_cc.heap_type].size = ion_cc.size;

	for (i = 0; i < dummy_ion_pdata.nr; i++) {
		struct ion_platform_heap *heap_data = &dummy_ion_pdata.heaps[i];

		if (heap_data->type == ion_cc.heap_type &&
							heap_data->base != 0) {
			heaps[i] = ion_heap_create(heap_data);
			if (IS_ERR_OR_NULL(heaps[i])) {
				ion_free(ion_cc.i_cc_client, ion_cc.i_cc_handle);
				ion_client_destroy(ion_cc.i_cc_client);
				ion_cc.i_cc_handle = NULL;
				ion_cc.i_cc_client = NULL;
				dummy_heaps[ion_cc.heap_type].base = 0;
				dummy_heaps[ion_cc.heap_type].size = 0;
				pr_err("Create ion cma carveout heap failed!!\n");
				return PTR_ERR(heaps[i]);
			}
			ion_device_add_heap(hb_ion_dev, heaps[i]);
		}
	}

	return 0;
}

static void ion_cma_carveout_range_discard(void)
{
	int i;

	if (!ion_cc.cc_is_valid) {
		return;
	}

	for (i = 0; i < dummy_ion_pdata.nr; i++) {
		struct ion_platform_heap *heap_data = &dummy_ion_pdata.heaps[i];
		if (heap_data->type == ion_cc.heap_type &&
							heap_data->base != 0) {
			ion_device_del_heap(hb_ion_dev, heaps[i]);
			ion_heap_destroy(heaps[i]);
			heap_data->base = 0;
			heap_data->size = 0;
		}
	}

	if (ion_cc.i_cc_handle != NULL) {
		ion_free(ion_cc.i_cc_client, ion_cc.i_cc_handle);
		ion_cc.i_cc_handle = NULL;
	}

	if (ion_cc.i_cc_client != NULL) {
		ion_client_destroy(ion_cc.i_cc_client);
		ion_cc.i_cc_client = NULL;
	}

	ion_cc.start = 0;
	ion_cc.size = 0;
}

static uint32_t ion_heap_buf_num(struct ion_client *client,
				   uint32_t type)
{
	uint32_t num = 0;
	struct rb_node *n;

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n,
						     struct ion_handle,
						     node);
		if (handle->buffer->heap->type == type) {
			num++;
		}
	}
	mutex_unlock(&client->lock);

	return num;
}

static int ion_cma_carveout_range_resize(size_t size)
{
	uint32_t num = 0;
	struct rb_node *n;
	int ret = 0;

	if ((size == ion_cc.size) || (!ion_cc.cc_is_valid)) {
		return 0;
	}

	for (n = rb_first(&hb_ion_dev->clients); n; n = rb_next(n)) {
		struct ion_client *client = rb_entry(n, struct ion_client,
						     node);
		num = ion_heap_buf_num(client, ion_cc.heap_type);
		if (num > 0) {
			pr_err("cma carveout head is using, can't be resize!!!\n");
			return -ENODEV;
		}

	}

	ion_cma_carveout_range_discard();

	if (size > 0) {
		ret = ion_cma_carveout_range_create(size);
	}

	return ret;
}

static ssize_t ion_cma_carveout_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (uint32_t)(ion_cc.size / 0x100000));
}

static ssize_t ion_cma_carveout_size_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	uint32_t tmp_size;
	int ret;

	ret = sscanf(buf, "%du", &tmp_size);
	if (ret < 0) {
		return 0;
	}

	if (tmp_size >= 0) {
		ion_cma_carveout_range_resize(tmp_size * 0x100000);
	}

	return (ssize_t)len;
}

static DEVICE_ATTR(cma_carveout_size, S_IRUGO | S_IWUSR,
		ion_cma_carveout_size_show,
		ion_cma_carveout_size_store);

static struct attribute *ion_dev_attrs[] = {
	&dev_attr_cma_carveout_size.attr,
	NULL,
};

static struct attribute_group ion_dev_attr_group = {
	.attrs = ion_dev_attrs,
};

#define ION_GET_PHY 0
#define ION_CACHE_INVAL 1
#define ION_CACHE_FLUSH 2
#define ION_MEMCPY 3
#define ION_CACHE_FLUSH_ALL 4
struct ion_phy_data {
	struct ion_handle *handle;
	phys_addr_t paddr;
	size_t len;
	uint64_t reserved;
};

static long ion_dummy_ioctl(struct ion_client *client,
			    unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int cpu;
	struct cpumask mask;

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
		__flush_dcache_area(phys_to_virt(phy_data.paddr), phy_data.len);

		break;
	}

	case ION_CACHE_FLUSH_ALL:
	{
		memcpy(&mask, cpu_online_mask, sizeof(struct cpumask));
		for_each_cpu(cpu, &mask)
			smp_call_function_single(cpu,
				(smp_call_func_t)__flush_dcache_all, NULL, 0);
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
		__inval_dcache_area(phys_to_virt(phy_data.paddr), phy_data.len);

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
	phys_addr_t cma_base;
	size_t cma_size;

	ion_cma_get_info(hb_ion_dev, &cma_base, &cma_size);

	if ((start < cvt->base || start + size > cvt->base + cvt->size)
			&& (start < cma_base || start + size > cma_base + cma_size))
		return -1;

	return 0;
}

EXPORT_SYMBOL_GPL(ion_check_in_heap_carveout);

static int __init ion_dummy_init(void)
{
	struct device_node *node, *rnode;
	struct resource ion_pool_reserved;
	const char *status;
	dma_cap_mask_t mask;
	int i, err = 0;

	hb_ion_dev = ion_device_create(ion_dummy_ioctl);
	heaps = kcalloc(dummy_ion_pdata.nr, sizeof(struct ion_heap *),
			GFP_KERNEL);
	if (!heaps)
		return -ENOMEM;

	rnode = of_find_node_by_path("/reserved-memory");
	if (rnode) {
		node = of_find_compatible_node(rnode, NULL, "ion-pool");
		if (node) {
			status = of_get_property(node, "status", NULL);
			if ((!status) || (strcmp(status, "okay") == 0)
					|| (strcmp(status, "ok") == 0)) {
				if (!of_address_to_resource(node, 0, &ion_pool_reserved)) {
					dummy_heaps[ION_HEAP_TYPE_CARVEOUT].base
						= ion_pool_reserved.start;
					dummy_heaps[ION_HEAP_TYPE_CARVEOUT].size
						= resource_size(&ion_pool_reserved);
					pr_debug("ION Carveout MEM start 0x%llx, size 0x%lx\n",
							dummy_heaps[ION_HEAP_TYPE_CARVEOUT].base,
							dummy_heaps[ION_HEAP_TYPE_CARVEOUT].size);
				}
			}
		}

		node = of_find_compatible_node(rnode, NULL, "shared-dma-pool");
		if (node) {
			err = of_property_read_u64(node, "reserved-size", &cma_reserved_size);
			if (err != 0) {
				if (dummy_heaps[ION_HEAP_TYPE_CARVEOUT].base == 0) {
					cma_reserved_size = DFT_CMA_CARVEOUT_SIZE;
				} else {
					cma_reserved_size = 0;
				}
			}

		}
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

		if ((heap_data->type == ION_HEAP_TYPE_CARVEOUT
				|| heap_data->type == ION_HEAP_TYPE_CMA_RESERVED)
				&& !heap_data->base)
			continue;

		if (heap_data->type == ION_HEAP_TYPE_CHUNK && !heap_data->base)
			continue;

		if (heap_data->type == ION_HEAP_TYPE_CUSTOM)
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

	ion_cc.cc_is_valid = false;
	if (cma_reserved_size != 0) {
		ion_cc.cc_is_valid = true;
		if (dummy_heaps[ION_HEAP_TYPE_CARVEOUT].base == 0) {
			ion_cc.heap_type = ION_HEAP_TYPE_CARVEOUT;
			pr_err("ion_dummy: not reserve carveout memory, try to alloc from cma\n");
		} else {
			ion_cc.heap_type = ION_HEAP_TYPE_CMA_RESERVED;
			pr_err("ion_dummy: user set cma reserved memory, try to alloc from cma\n");
		}

		if (ion_cma_carveout_range_create(cma_reserved_size) == 0) {
			dummy_heaps[ion_cc.heap_type].base = ion_cc.start;
			dummy_heaps[ion_cc.heap_type].size = ion_cc.size;
			pr_debug("ION CMA Carveout MEM start 0x%llx, size 0x%lx\n",
					dummy_heaps[ion_cc.heap_type].base,
					dummy_heaps[ion_cc.heap_type].size);
			err = device_add_group(hb_ion_dev->dev.this_device, &ion_dev_attr_group);
			if (err < 0) {
				pr_debug("Create ion cma carveout size sys node failed\n");
			}
		} else {
			pr_err("ion_dummy: not reserve memory\n");
		}
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
	ion_cma_carveout_range_discard();
	for (i = 0; i < dummy_ion_pdata.nr; ++i)
		ion_heap_destroy(heaps[i]);

	ion_del_cma_heaps(hb_ion_dev);
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

	ion_cma_carveout_range_discard();
	ion_device_destroy(hb_ion_dev);
	dma_release_channel(dma_ch);

	for (i = 0; i < dummy_ion_pdata.nr; i++)
		ion_heap_destroy(heaps[i]);

	ion_del_cma_heaps(hb_ion_dev);
	kfree(heaps);

	if (chunk_ptr) {
		free_pages_exact(chunk_ptr,
				 dummy_heaps[ION_HEAP_TYPE_CHUNK].size);
		chunk_ptr = NULL;
	}
}
__exitcall(ion_dummy_exit);
