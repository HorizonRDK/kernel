#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "cnn_mm_heap.h"

#define CNN_CARVEOUT_ALLOCATE_FAIL	-1

struct cnn_carveout_heap {
	struct cnn_heap heap;
	struct gen_pool *pool;
	phys_addr_t base;
};

static phys_addr_t cnn_carveout_allocate(struct cnn_heap *heap,
					 unsigned long size)
{
	struct cnn_carveout_heap *carveout_heap =
		container_of(heap, struct cnn_carveout_heap, heap);
	unsigned long offset = gen_pool_alloc(carveout_heap->pool, size);

	if (!offset)
		return CNN_CARVEOUT_ALLOCATE_FAIL;

	return offset;
}

static void cnn_carveout_free(struct cnn_heap *heap, phys_addr_t addr,
			      unsigned long size)
{
	struct cnn_carveout_heap *carveout_heap =
		container_of(heap, struct cnn_carveout_heap, heap);

	if (addr == CNN_CARVEOUT_ALLOCATE_FAIL)
		return;
	gen_pool_free(carveout_heap->pool, addr, size);
}

static int cnn_carveout_heap_phys(struct cnn_heap *heap,
				  struct cnn_buffer *buffer,
				  cnn_phys_addr_t *addr, size_t *len)
{
	struct sg_table *table = buffer->sg_table;
	struct page *page = sg_page(table->sgl);
	cnn_phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));

	*addr = paddr;
	*len = buffer->size;
	return 0;
}

static int cnn_carveout_heap_allocate(struct cnn_heap *heap,
				      struct cnn_buffer *buffer,
				      unsigned long size,
				      unsigned long flags)
{
	struct sg_table *table;
	phys_addr_t paddr;
	int ret;

	table = kmalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;
	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret)
		goto err_free;

	paddr = cnn_carveout_allocate(heap, size);
        if (paddr == CNN_CARVEOUT_ALLOCATE_FAIL) {
		ret = -ENOMEM;
		goto err_free_table;
	}

	sg_set_page(table->sgl, pfn_to_page(PFN_DOWN(paddr)), size, 0);
	buffer->sg_table = table;
        buffer->paddr = paddr;

	return 0;

err_free_table:
	sg_free_table(table);
err_free:
	kfree(table);
	return ret;
}

static void cnn_carveout_heap_free(struct cnn_buffer *buffer)
{
	struct cnn_heap *heap = buffer->heap;
	struct sg_table *table = buffer->sg_table;
	struct page *page = sg_page(table->sgl);
	phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));

	cnn_carveout_free(heap, paddr, buffer->size);
	sg_free_table(table);
	kfree(table);
}

static struct cnn_heap_ops carveout_heap_ops = {
	.allocate = cnn_carveout_heap_allocate,
	.free = cnn_carveout_heap_free,
        .phys = cnn_carveout_heap_phys,
	.map_user = cnn_heap_map_user,
	.map_kernel = cnn_heap_map_kernel,
	.unmap_kernel = cnn_heap_unmap_kernel,
};

struct cnn_heap *cnn_carveout_heap_create(struct cnn_plat_heap *heap_data)
{
	struct cnn_carveout_heap *carveout_heap;
	int ret;

	struct page *page;
	size_t size;

	page = pfn_to_page(PFN_DOWN(heap_data->base));
	size = heap_data->size;

	ret = cnn_heap_pages_zero(page, size, pgprot_noncached(PAGE_KERNEL));
	if (ret)
		return ERR_PTR(ret);

	carveout_heap = kzalloc(sizeof(*carveout_heap), GFP_KERNEL);
	if (!carveout_heap)
		return ERR_PTR(-ENOMEM);

	carveout_heap->pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!carveout_heap->pool) {
		kfree(carveout_heap);
		return ERR_PTR(-ENOMEM);
	}
	carveout_heap->base = heap_data->base;
	gen_pool_add(carveout_heap->pool, carveout_heap->base, heap_data->size,
		     -1);
	carveout_heap->heap.ops = &carveout_heap_ops;
	carveout_heap->heap.type = CNN_HEAP_TYPE_CARVEOUT;
	carveout_heap->heap.flags = CNN_HEAP_FLAG_DEFER_FREE;

	return &carveout_heap->heap;
}

void cnn_carveout_heap_destroy(struct cnn_heap *heap)
{
	struct cnn_carveout_heap *carveout_heap =
	     container_of(heap, struct  cnn_carveout_heap, heap);

	gen_pool_destroy(carveout_heap->pool);
	kfree(carveout_heap);
	carveout_heap = NULL;
}
