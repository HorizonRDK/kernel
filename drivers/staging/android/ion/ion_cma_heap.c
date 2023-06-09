/*
 * drivers/staging/android/ion/ion_cma_heap.c
 *
 * Copyright (C) Linaro 2012
 * Author: <benjamin.gaignard@linaro.org> for ST-Ericsson.
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

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/cma.h>
#include <linux/scatterlist.h>
#include <linux/highmem.h>

#include <linux/ion.h>

static struct ion_device *cma_ion_dev = NULL;

struct ion_cma_heap {
	struct ion_heap heap;
	struct cma *cma;
};

#define to_cma_heap(x) container_of(x, struct ion_cma_heap, heap)

/* ION CMA heap operations functions */
static int ion_cma_allocate(struct ion_heap *heap, struct ion_buffer *buffer,
		unsigned long len, unsigned long start_align,
		unsigned long flags)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(heap);
	struct sg_table *table;
	struct page *pages;
	unsigned long size = PAGE_ALIGN(len);
	unsigned long nr_pages = size >> PAGE_SHIFT;
#ifndef CONFIG_HOBOT_XJ3
	unsigned long align = get_order(size);
#else
	unsigned long align = 0;	//order 0 i.e. 1 page align
#endif
	int ret;

#ifndef CONFIG_HOBOT_XJ3
	if (align > CONFIG_CMA_ALIGNMENT)
		align = CONFIG_CMA_ALIGNMENT;
#endif

	pages = cma_alloc(cma_heap->cma, nr_pages, align, GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	/* the page may used for instruction or data which still cached */
	flush_icache_range((unsigned long)page_address(pages),
			(unsigned long)(page_address(pages) + size));
	__inval_dcache_area(page_address(pages), size);
	if (!(buffer->flags & ION_FLAG_UNINITIALIZED)) {
		if (PageHighMem(pages)) {
			unsigned long nr_clear_pages = nr_pages;
			struct page *page = pages;

			while (nr_clear_pages > 0) {
				void *vaddr = kmap_atomic(page);

				memset(vaddr, 0, PAGE_SIZE);
				kunmap_atomic(vaddr);
				page++;
				nr_clear_pages--;
			}
		} else {
			memset(page_address(pages), 0, size);
		}
	}
	__flush_dcache_area(page_address(pages), size);

	table = kmalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		goto err;

	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret)
		goto free_mem;

	sg_set_page(table->sgl, pages, size, 0);

	buffer->priv_virt = pages;
	buffer->sg_table = table;

	return 0;

free_mem:
	kfree(table);
err:
	cma_release(cma_heap->cma, pages, nr_pages);
	return -ENOMEM;
}

static void ion_cma_free(struct ion_buffer *buffer)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct page *pages = buffer->priv_virt;
	unsigned long nr_pages = PAGE_ALIGN(buffer->size) >> PAGE_SHIFT;

	/* release memory */
	cma_release(cma_heap->cma, pages, nr_pages);
	/* release sg table */
	sg_free_table(buffer->sg_table);
	kfree(buffer->sg_table);
}

static int ion_cma_heap_phys(struct ion_heap *heap,
				  struct ion_buffer *buffer,
				  phys_addr_t *addr, size_t *len)
{
	struct sg_table *table = buffer->sg_table;
	struct page *page = sg_page(table->sgl);
	phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));

	*addr = paddr;
	*len = buffer->size;
	return 0;
}

static struct sg_table *ion_cma_heap_map_dma(struct ion_heap *heap,
					     struct ion_buffer *buffer)
{
	return buffer->sg_table;
}

static void ion_cma_heap_unmap_dma(struct ion_heap *heap,
				   struct ion_buffer *buffer)
{
}

static struct ion_heap_ops ion_cma_ops = {
	.allocate = ion_cma_allocate,
	.free = ion_cma_free,
	.phys = ion_cma_heap_phys,
	.map_dma = ion_cma_heap_map_dma,
	.unmap_dma = ion_cma_heap_unmap_dma,
	.map_user = ion_heap_map_user,
	.map_kernel = ion_heap_map_kernel,
	.unmap_kernel = ion_heap_unmap_kernel,
};

static struct ion_heap *__ion_cma_heap_create(struct cma *cma)
{
	struct ion_cma_heap *cma_heap;

	cma_heap = kzalloc(sizeof(*cma_heap), GFP_KERNEL);

	if (!cma_heap)
		return ERR_PTR(-ENOMEM);

	cma_heap->heap.ops = &ion_cma_ops;
	/*
	 * get device from private heaps data, later it will be
	 * used to make the link with reserved CMA memory
	 */
	cma_heap->cma = cma;
	cma_heap->heap.type = ION_HEAP_TYPE_DMA;
	return &cma_heap->heap;
}

int ion_cma_get_info(struct ion_device *dev, phys_addr_t *base, size_t *size)
{
	struct ion_heap *heap;
	struct ion_cma_heap *cma_heap;

	/* 
	 * the dev->lock trys to pretect the heaps list which creatd at
	 * the initialization stage of ion driver and would nerver be 
	 * inserted or destroyed later.So the rw_semaphore acquisition can
	 * be skiped to prevent scheduling in atomic context.
	 */
	if (!in_atomic())
		down_read(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		if (!((1 << heap->type) & ION_HEAP_TYPE_DMA_MASK))
			continue;

		cma_heap = to_cma_heap(heap);
		*base = cma_get_base(cma_heap->cma);
		*size = cma_get_size(cma_heap->cma);
		break;
	}
	if (!in_atomic())
		up_read(&dev->lock);

	return 0;
}

static int __ion_add_cma_heaps(struct cma *cma, void *data)
{
	const char *name;
	struct ion_heap *heap;

	if (!cma_ion_dev)
		return -ENODEV;

	name = cma_get_name(cma);
	if (strcmp(name, "ion_cma"))
		return 0;

	heap = __ion_cma_heap_create(cma);
	if (IS_ERR(heap))
		return PTR_ERR(heap);

	heap->name = cma_get_name(cma);

	ion_device_add_heap(cma_ion_dev, heap);

	return 0;
}

static int __ion_del_cma_heaps(struct cma *cma, void *data)
{
	const char *name;
	struct ion_heap *heap;
	struct ion_cma_heap *cma_heap;

	if (!cma_ion_dev)
		return -ENODEV;

	name = cma_get_name(cma);
	if (strcmp(name, "ion_cma"))
		return 0;

	plist_for_each_entry(heap, &cma_ion_dev->heaps, node) {
		if (heap->type == ION_HEAP_TYPE_DMA) {
			plist_del((struct plist_node *)heap, &cma_ion_dev->heaps);
			break;
		}
	}

	cma_heap = to_cma_heap(heap);
	kfree(cma_heap);
	cma_ion_dev = NULL;

	return 0;
}

int ion_add_cma_heaps(struct ion_device *idev)
{
	if (!cma_ion_dev)
		cma_ion_dev = idev;
	else
		return 0;

	cma_for_each_area(__ion_add_cma_heaps, NULL);
	return 0;
}

int ion_del_cma_heaps(struct ion_device *idev)
{
	cma_for_each_area(__ion_del_cma_heaps, NULL);
	return 0;
}
