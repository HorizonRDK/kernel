/*
 * X2 CNN Memory Management Heap Driver (found in Hobot Platform)
 *
 * 2017 - 2018 (C) Hobot Inc.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundatcnn;
 * either verscnn 2 of the License, or (at your optcnn) any
 * later verscnn.
 *
 */

#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/rtmutex.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/scatterlist.h>
#include <linux/sched/types.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>

#include "cnn_mm_heap.h"

void *cnn_heap_map_kernel(struct cnn_heap *heap,
			  struct cnn_buffer *buffer)
{
	struct scatterlist *sg;
	int i, j;
	void *vaddr;
	pgprot_t pgprot;
	struct sg_table *table = buffer->sg_table;
	int npages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
	struct page **pages = vmalloc(sizeof(struct page *) * npages);
	struct page **tmp = pages;

	if (!pages)
		return ERR_PTR(-ENOMEM);

	if (buffer->flags & CNN_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_writecombine(PAGE_KERNEL);

	for_each_sg(table->sgl, sg, table->nents, i) {
		int npages_this_entry = PAGE_ALIGN(sg->length) / PAGE_SIZE;
		struct page *page = sg_page(sg);

		BUG_ON(i >= npages);
		for (j = 0; j < npages_this_entry; j++)
			*(tmp++) = page++;
	}
	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	vfree(pages);

	if (!vaddr)
		return ERR_PTR(-ENOMEM);

	return vaddr;
}

void cnn_heap_unmap_kernel(struct cnn_heap *heap,
			   struct cnn_buffer *buffer)
{
	vunmap(buffer->vaddr);
}

int cnn_heap_map_user(struct cnn_heap *heap, struct cnn_buffer *buffer,
		      struct vm_area_struct *vma)
{
	struct sg_table *table = buffer->sg_table;
	unsigned long addr = vma->vm_start;
	unsigned long offset = vma->vm_pgoff * PAGE_SIZE;
	struct scatterlist *sg;
	int i;
	int ret;

	for_each_sg(table->sgl, sg, table->nents, i) {
		struct page *page = sg_page(sg);
		unsigned long remainder = vma->vm_end - addr;
		unsigned long len = sg->length;

		if (offset >= sg->length) {
			offset -= sg->length;
			continue;
		} else if (offset) {
			page += offset / PAGE_SIZE;
			len = sg->length - offset;
			offset = 0;
		}
		len = min(len, remainder);
		ret = remap_pfn_range(vma, addr, page_to_pfn(page), len,
				      vma->vm_page_prot);
		if (ret)
			return ret;
		addr += len;
		if (addr >= vma->vm_end)
			return 0;
	}
	return 0;
}

static int cnn_heap_clear_pages(struct page **pages, int num, pgprot_t pgprot)
{
	void *addr = vm_map_ram(pages, num, -1, pgprot);

	if (!addr)
		return -ENOMEM;
#if 0
        memset_io(addr, 0, PAGE_SIZE * num);
#endif
	vm_unmap_ram(addr, num);

	return 0;
}

static int cnn_heap_sglist_zero(struct scatterlist *sgl, unsigned int nents,
				pgprot_t pgprot)
{
	int p = 0;
	int ret = 0;
	struct sg_page_iter piter;
	struct page **page_list;
        u32 nr_pages = 0;
        size_t size;

        size = sg_dma_len(sgl);
        nr_pages = size / PAGE_SIZE;
	page_list = kmalloc_array(nr_pages, sizeof(*page_list), GFP_KERNEL);
	if (!page_list)
		return -ENOMEM;

	for_each_sg_page(sgl, &piter, nents, 0) {
		page_list[p++] = sg_page_iter_page(&piter);
		if (p == nr_pages) {
			ret = cnn_heap_clear_pages(page_list, p, pgprot);
			if (ret)
				return ret;
			p = 0;
		}
	}
	if (p)
		ret = cnn_heap_clear_pages(page_list, p, pgprot);

        kfree(page_list);

	return ret;
}

int cnn_heap_buffer_zero(struct cnn_buffer *buffer)
{
	struct sg_table *table = buffer->sg_table;
	pgprot_t pgprot;

	if (buffer->flags & CNN_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_noncached(PAGE_KERNEL);

	return cnn_heap_sglist_zero(table->sgl, table->nents, pgprot);
}

int cnn_heap_pages_zero(struct page *page, size_t size, pgprot_t pgprot)
{
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	return cnn_heap_sglist_zero(&sg, 1, pgprot);
}

void cnn_heap_freelist_add(struct cnn_heap *heap, struct cnn_buffer *buffer)
{
	spin_lock(&heap->free_lock);
	list_add(&buffer->list, &heap->free_list);
	heap->free_list_size += buffer->size;
	spin_unlock(&heap->free_lock);
	wake_up(&heap->waitqueue);
}

size_t cnn_heap_freelist_size(struct cnn_heap *heap)
{
	size_t size;

	spin_lock(&heap->free_lock);
	size = heap->free_list_size;
	spin_unlock(&heap->free_lock);

	return size;
}

static size_t _cnn_heap_freelist_drain(struct cnn_heap *heap, size_t size,
				       bool skip_pools)
{
	struct cnn_buffer *buffer;
	size_t total_drained = 0;

	if (cnn_heap_freelist_size(heap) == 0)
		return 0;

	spin_lock(&heap->free_lock);
	if (size == 0)
		size = heap->free_list_size;

	while (!list_empty(&heap->free_list)) {
		if (total_drained >= size)
			break;
		buffer = list_first_entry(&heap->free_list, struct cnn_buffer,
					  list);
		list_del(&buffer->list);
		heap->free_list_size -= buffer->size;
		if (skip_pools)
			buffer->private_flags |= CNN_PRIV_FLAG_SHRINKER_FREE;
		total_drained += buffer->size;
		spin_unlock(&heap->free_lock);
		cnn_buffer_destroy(buffer);
		spin_lock(&heap->free_lock);
	}
	spin_unlock(&heap->free_lock);

	return total_drained;
}

size_t cnn_heap_freelist_drain(struct cnn_heap *heap, size_t size)
{
	return _cnn_heap_freelist_drain(heap, size, false);
}

size_t cnn_heap_freelist_shrink(struct cnn_heap *heap, size_t size)
{
	return _cnn_heap_freelist_drain(heap, size, true);
}

static int cnn_heap_deferred_free(void *data)
{
	struct cnn_heap *heap = data;

	while (true) {
		struct cnn_buffer *buffer;

		wait_event_freezable(heap->waitqueue,
				     cnn_heap_freelist_size(heap) > 0);

		spin_lock(&heap->free_lock);
		if (list_empty(&heap->free_list)) {
			spin_unlock(&heap->free_lock);
			continue;
		}
		buffer = list_first_entry(&heap->free_list, struct cnn_buffer,
					  list);
		list_del(&buffer->list);
		heap->free_list_size -= buffer->size;
		spin_unlock(&heap->free_lock);
		cnn_buffer_destroy(buffer);
	}

	return 0;
}

int cnn_heap_init_deferred_free(struct cnn_heap *heap)
{
	struct sched_param param;
        param.sched_priority = 0;

	INIT_LIST_HEAD(&heap->free_list);
	init_waitqueue_head(&heap->waitqueue);
	heap->task = kthread_run(cnn_heap_deferred_free, heap,
				 "%s", heap->name);
	if (IS_ERR(heap->task)) {
		pr_err("%s: creating thread for deferred free failed\n",
		       __func__);
		return PTR_ERR_OR_ZERO(heap->task);
	}
	sched_setscheduler(heap->task, SCHED_IDLE, &param);
	return 0;
}

static unsigned long cnn_heap_shrink_count(struct shrinker *shrinker,
					   struct shrink_control *sc)
{
	struct cnn_heap *heap = container_of(shrinker, struct cnn_heap,
					     shrinker);
	int total = 0;

	total = cnn_heap_freelist_size(heap) / PAGE_SIZE;
	if (heap->ops->shrink)
		total += heap->ops->shrink(heap, sc->gfp_mask, 0);
	return total;
}

static unsigned long cnn_heap_shrink_scan(struct shrinker *shrinker,
					  struct shrink_control *sc)
{
	struct cnn_heap *heap = container_of(shrinker, struct cnn_heap,
					     shrinker);
	int freed = 0;
	int to_scan = sc->nr_to_scan;

	if (to_scan == 0)
		return 0;

	/*
	 * shrink the free list first, no point in zeroing the memory if we're
	 * just going to reclaim it. Also, skip any possible page pooling.
	 */
	if (heap->flags & CNN_HEAP_FLAG_DEFER_FREE)
		freed = cnn_heap_freelist_shrink(heap, to_scan * PAGE_SIZE) /
				PAGE_SIZE;

	to_scan -= freed;
	if (to_scan <= 0)
		return freed;

	if (heap->ops->shrink)
		freed += heap->ops->shrink(heap, sc->gfp_mask, to_scan);
	return freed;
}

void cnn_heap_init_shrinker(struct cnn_heap *heap)
{
	heap->shrinker.count_objects = cnn_heap_shrink_count;
	heap->shrinker.scan_objects = cnn_heap_shrink_scan;
	heap->shrinker.seeks = DEFAULT_SEEKS;
	heap->shrinker.batch = 0;
	register_shrinker(&heap->shrinker);
}

struct cnn_heap *cnn_heap_create(struct cnn_plat_heap *heap_data)
{
	struct cnn_heap *heap = NULL;

	switch (heap_data->type) {
	case CNN_HEAP_TYPE_CARVEOUT:
		heap = cnn_carveout_heap_create(heap_data);
		break;
	default:
		pr_err("%s: Invalid heap type %d\n", __func__,
		       heap_data->type);
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR_OR_NULL(heap)) {
		pr_err("%s: error creating heap %s type %d base %lu size %zu\n",
		       __func__, heap_data->heap_name, heap_data->type,
		       heap_data->base, heap_data->size);
		return ERR_PTR(-EINVAL);
	}

	heap->name = heap_data->heap_name;
	heap->id = heap_data->heap_id;
	return heap;
}
EXPORT_SYMBOL(cnn_heap_create);

void cnn_heap_destroy(struct cnn_heap *heap)
{
	if (!heap)
		return;

	switch (heap->type) {
	case CNN_HEAP_TYPE_CARVEOUT:
		cnn_carveout_heap_destroy(heap);
		break;
	default:
		pr_err("%s: Invalid heap type %d\n", __func__,
		       heap->type);
	}
}
EXPORT_SYMBOL(cnn_heap_destroy);
