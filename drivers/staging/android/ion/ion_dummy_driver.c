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
#include "ion.h"

extern struct ion_device *ion_device_create(long (*custom_ioctl)
				     (struct ion_client *client,
				      unsigned int cmd,
				      unsigned long arg));

static struct ion_device *idev;
static struct ion_heap **heaps;

static void *carveout_ptr;
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
};

static struct ion_platform_data dummy_ion_pdata = {
	.nr = ARRAY_SIZE(dummy_heaps),
	.heaps = dummy_heaps,
};

#define ION_GET_PHY 0
struct ion_phy_data {
	struct ion_handle *handle;
	phys_addr_t paddr;
	size_t len;
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

		ret = ion_phys(client, phy_data.handle,
			       &phy_data.paddr, &phy_data.len);
		if (copy_to_user((void __user *)arg, &phy_data,
				 sizeof(struct ion_phy_data)))
			return -EFAULT;

		break;
	}

	default:
		return -ENOTTY;
	}

	return ret;
}

static int __init ion_dummy_init(void)
{
	struct device_node *node;
	struct resource ion_pool_reserved;
	int i, err;

	idev = ion_device_create(ion_dummy_ioctl);
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
				pr_info("ION Carveout MEM start 0x%x, size 0x%x\n",
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

		heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(idev, heaps[i]);
	}
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
device_initcall(ion_dummy_init);

static void __exit ion_dummy_exit(void)
{
	int i;

	ion_device_destroy(idev);

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
