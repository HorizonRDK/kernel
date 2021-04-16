/* CNN Memory heap include file
 * 2017 - 2018 (C) Hobot Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License verscnn 2, as published by the Free Software Foundatcnn, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _CNN_MM_HEAP_H
#define _CNN_MM_HEAP_H


#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/kref.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/sched.h>
#include <linux/shrinker.h>
#include <linux/miscdevice.h>

#include "cnn_mm_ioctl.h"

struct cnn_handle;
struct cnn_device;
struct cnn_heap;
struct cnn_mapper;
struct cnn_client;
struct cnn_buffer;

/**
 * struct cnn_plat_heap - defines a heap in the given platform
 * @type:	type of the heap from cnn_heap_type enum
 * @heap_id:		unique identifier for heap.  When allocating higher numb ers
 *		will be allocated from first.  At allocatcnn these are passed
 *		as a bit mask and therefore can not exceed CNN_NUM_HEAP_IDS.
 * @heap_name:	used for debug purposes
 * @base:	base address of heap in physical memory if applicable
 * @size:	size of the heap in bytes if applicable
 * @priv:	private info passed from the board file
 *
 * Provided by the board file.
 */
struct cnn_plat_heap {
	enum cnn_heap_type type;
	unsigned int heap_id;
	const char *heap_name;
	cnn_phys_addr_t base;
	size_t size;
	phys_addr_t align;
	void *priv;
};

/**
 * struct cnn_plat_data - array of platform heaps passed from board file
 * @nr:		number of structures in the array
 * @heaps:	array of cnn_heap structions
 *
 * Provided by the board file in the form of platform data to a platform device.
 */
struct cnn_plat_data {
	int nr;
	struct cnn_plat_heap *heaps;
};

/**
 * struct cnn_buffer - metadata for a particular buffer
 * @ref:		reference count
 * @node:		node in the cnn_device buffers tree
 * @dev:		back pointer to the cnn_device
 * @heap:		back pointer to the heap the buffer came from
 * @flags:		buffer specific flags
 * @private_flags:	internal buffer specific flags
 * @size:		size of the buffer
 * @priv_virt:		private data to the buffer representable as
 *			a void *
 * @lock:		protects the buffers cnt fields
 * @kmap_cnt:		number of times the buffer is mapped to the kernel
 * @vaddr:		the kernel mapping if kmap_cnt is not zero
 * @sg_table:		the sg table for the buffer if dmap_cnt is not zero
 */
struct cnn_buffer {
        struct kref ref;
	union {
		struct rb_node node;
		struct list_head list;
	};
	struct cnn_device *dev;
	struct cnn_heap *heap;
	unsigned long flags;
	unsigned long private_flags;
	size_t size;
	void *priv_virt;
	struct mutex lock;
	int kmap_cnt;
	void *vaddr;
        cnn_phys_addr_t paddr;
	struct sg_table *sg_table;
        struct page **pages;
	struct list_head attachments;
        /* used to track orphaned buffers */
	int handle_count;
	char task_comm[TASK_COMM_LEN];
	pid_t pid;

};
void cnn_buffer_destroy(struct cnn_buffer *buffer);

/**
 * struct cnn_device - the metadata of the cnn device node
 * @dev:		the actual misc device
 * @buffers:		an rb tree of all the existing buffers
 * @buffer_lock:	lock protecting the tree of buffers
 * @lock:		rwsem protecting the tree of heaps and clients
 */
struct cnn_device {
	struct miscdevice dev;
	struct rb_root buffers;
	struct mutex buffer_lock;
	struct rw_semaphore lock;
	struct plist_head heaps;
	long (*custom_ioctl)(struct cnn_client *client, unsigned int cmd,
			     unsigned long arg);
        struct rb_root clients;
	struct dentry *debug_root;
	struct dentry *heaps_debug_root;
	struct dentry *clients_debug_root;
	int heap_cnt;
};

/**
 * struct cnn_client - a process/hw block local address space
 * @node:		node in the tree of all clients
 * @dev:		backpointer to ion device
 * @handles:		an rb tree of all the handles in this client
 * @idr:		an idr space for allocating handle ids
 * @lock:		lock protecting the tree of handles
 * @name:		used for debugging
 * @display_name:	used for debugging (unique version of @name)
 * @display_serial:	used for debugging (to make display_name unique)
 * @task:		used for debugging
 *
 * A client represents a list of buffers this client may access.
 * The mutex stored here is used to protect both handles tree
 * as well as the handles themselves, and should be held while modifying either.
 */
struct cnn_client {
	struct rb_node node;
	struct cnn_device *dev;
	struct rb_root handles;
	struct idr idr;
	struct mutex lock;
	const char *name;
	char *display_name;
	int display_serial;
	struct task_struct *task;
	pid_t pid;
	struct dentry *debug_root;
};

/**
 * cnn_handle - a client local reference to a buffer
 * @ref:		reference count
 * @client:		back pointer to the client the buffer resides in
 * @buffer:		pointer to the buffer
 * @node:		node in the client's handle rbtree
 * @kmap_cnt:		count of times this client has mapped to kernel
 * @id:			client-unique id allocated by client->idr
 *
 * Modifications to node, map_cnt or mapping should be protected by the
 * lock in the client.  Other fields are never changed after initialization.
 */
struct cnn_handle {
	struct kref ref;
	struct cnn_client *client;
	struct cnn_buffer *buffer;
	struct rb_node node;
	unsigned int kmap_cnt;
	int id;
};

/**
 * struct cnn_heap_ops - ops to operate on a given heap
 * @allocate:		allocate memory
 * @free:		free memory
 * @phys		get physical address of a buffer (only define on
 *			physically contiguous heaps)
 * @map_kernel		map memory to the kernel
 * @unmap_kernel	unmap memory to the kernel
 * @map_user		map memory to userspace
 *
 * allocate, phys, and map_user return 0 on success, -errno on error.
 * map_dma and map_kernel return pointer on success, ERR_PTR on
 * error. @free will be called with cnn_PRIV_FLAG_SHRINKER_FREE set in
 * the buffer's private_flags when called from a shrinker. In that
 * case, the pages being free'd must be truly free'd back to the
 * system, not put in a page pool or otherwise cached.
 */
struct cnn_heap_ops {
	int (*allocate)(struct cnn_heap *heap,
			struct cnn_buffer *buffer, unsigned long len,
			unsigned long flags);
	void (*free)(struct cnn_buffer *buffer);
        int (*phys)(struct cnn_heap *heap, struct cnn_buffer *buffer,
		    cnn_phys_addr_t *addr, size_t *len);
	void * (*map_kernel)(struct cnn_heap *heap, struct cnn_buffer *buffer);
	void (*unmap_kernel)(struct cnn_heap *heap, struct cnn_buffer *buffer);
	int (*map_user)(struct cnn_heap *mapper, struct cnn_buffer *buffer,
			struct vm_area_struct *vma);
	int (*shrink)(struct cnn_heap *heap, gfp_t gfp_mask, int nr_to_scan);
};

/**
 * heap flags - flags between the heaps and core cnn code
 */
#define CNN_HEAP_FLAG_DEFER_FREE BIT(0)

/**
 * private flags - flags internal to cnn
 */
/*
 * Buffer is being freed from a shrinker functcnn. Skip any possible
 * heap-specific caching mechanism (e.g. page pools). Guarantees that
 * any buffer storage that came from the system allocator will be
 * returned to the system allocator.
 */
#define CNN_PRIV_FLAG_SHRINKER_FREE BIT(0)

/**
 * struct cnn_heap - represents a heap in the system
 * @node:		rb node to put the heap on the device's tree of heaps
 * @dev:		back pointer to the cnn_device
 * @type:		type of heap
 * @ops:		ops struct as above
 * @flags:		flags
 * @id:			id of heap, also indicates priority of this heap when
 *			allocating.  These are specified by platform data and
 *			MUST be unique
 * @name:		used for debugging
 * @shrinker:		a shrinker for the heap
 * @free_list:		free list head if deferred free is used
 * @free_list_size	size of the deferred free list in bytes
 * @lock:		protects the free list
 * @waitqueue:		queue to wait on from deferred free thread
 * @task:		task struct of deferred free thread
 * @debug_show:		called when heap debug file is read to add any
 *			heap specific debug info to output
 *
 * Represents a pool of memory from which buffers can be made.  In some
 * systems the only heap is regular system memory allocated via vmalloc.
 * On others, some blocks might require large physically contiguous buffers
 * that are allocated from a specially reserved heap.
 */
struct cnn_heap {
	struct plist_node node;
	struct cnn_device *dev;
	enum cnn_heap_type type;
	struct cnn_heap_ops *ops;
	unsigned long flags;
	unsigned int id;
	const char *name;
	struct shrinker shrinker;
	struct list_head free_list;
	size_t free_list_size;
	spinlock_t free_lock;
	wait_queue_head_t waitqueue;
	struct task_struct *task;

	int (*debug_show)(struct cnn_heap *heap, struct seq_file *, void *);
};

/**
 * cnn_buffer_cached - this cnn buffer is cached
 * @buffer:		buffer
 *
 * indicates whether this cnn buffer is cached
 */
bool cnn_buffer_cached(struct cnn_buffer *buffer);

/**
 * cnn_buffer_fault_user_mappings - fault in user mappings of this buffer
 * @buffer:		buffer
 *
 * indicates whether userspace mappings of this buffer will be faulted
 * in, this can affect how buffers are allocated from the heap.
 */
bool cnn_buffer_fault_user_mappings(struct cnn_buffer *buffer);

/**
 * cnn_device_create - allocates and returns an ion device
 * @custom_ioctl:	arch specific ioctl function if applicable
 *
 * returns a valid device or -PTR_ERR
 */
struct cnn_device *cnn_device_create(long (*custom_ioctl)
				     (struct cnn_client *client,
				      unsigned int cmd,
				      unsigned long arg));

/**
 * cnn_device_destroy - free and device and it's resource
 * @dev:		the device
 */
void cnn_device_destroy(struct cnn_device *dev);

/**
 * cnn_device_add_heap - adds a heap to the cnn device
 * @heap:		the heap to add
 */
void cnn_device_add_heap(struct cnn_device *idev, struct cnn_heap *heap);

/**
 * some helpers for common operatcnns on buffers using the sg_table
 * and vaddr fields
 */
void *cnn_heap_map_kernel(struct cnn_heap *heap, struct cnn_buffer *buffer);
void cnn_heap_unmap_kernel(struct cnn_heap *heap, struct cnn_buffer *buffer);
int cnn_heap_map_user(struct cnn_heap *heap, struct cnn_buffer *buffer,
		      struct vm_area_struct *vma);
int cnn_heap_buffer_zero(struct cnn_buffer *buffer);
int cnn_heap_pages_zero(struct page *page, size_t size, pgprot_t pgprot);

/**
 * cnn_client_create() -  allocate a client and returns it
 * @dev:		the global cnn device
 * @name:		used for debugging
 */
struct cnn_client *cnn_client_create(struct cnn_device *dev,
				     const char *name);
int cnn_phys(struct cnn_client *client, struct cnn_handle *handle,
	     cnn_phys_addr_t *paddr, size_t *len);

/**
 * cnn_client_destroy() -  free's a client and all it's handles
 * @client:	the client
 *
 * Free the provided client and all it's resources including
 * any handles it is holding.
 */
void cnn_client_destroy(struct cnn_client *client);
/**
 * cnn_alloc - allocate ion memory
 * @client:		the client
 * @len:		size of the allocation
 * @heap_id_mask:	mask of heaps to allocate from, if multiple bits are set
 *			heaps will be tried in order from highest to lowest
 *			id
 * @flags:		heap flags, the low 16 bits are consumed by ion, the
 *			high 16 bits are passed on to the respective heap and
 *			can be heap custom
 *
 * Allocate memory in one of the heaps provided in heap mask and return
 * an opaque handle to it.
 */
struct cnn_handle *cnn_alloc(struct cnn_client *client, size_t len,
			     unsigned int heap_id_mask,
			     unsigned int flags);

/**
 * cnn_free - free a handle
 * @client:	the client
 * @handle:	the handle to free
 *
 * Free the provided handle.
 */
void cnn_free(struct cnn_client *client, struct cnn_handle *handle);


/**
 * cnn_heap_init_shrinker
 * @heap:		the heap
 *
 * If a heap sets the cnn_HEAP_FLAG_DEFER_FREE flag or defines the shrink op
 * this functcnn will be called to setup a shrinker to shrink the freelists
 * and call the heap's shrink op.
 */
void cnn_heap_init_shrinker(struct cnn_heap *heap);

/**
 * cnn_heap_init_deferred_free -- initialize deferred free functcnnality
 * @heap:		the heap
 *
 * If a heap sets the cnn_HEAP_FLAG_DEFER_FREE flag this functcnn will
 * be called to setup deferred frees. Calls to free the buffer will
 * return immediately and the actual free will occur some time later
 */
int cnn_heap_init_deferred_free(struct cnn_heap *heap);

/**
 * cnn_heap_freelist_add - add a buffer to the deferred free list
 * @heap:		the heap
 * @buffer:		the buffer
 *
 * Adds an item to the deferred freelist.
 */
void cnn_heap_freelist_add(struct cnn_heap *heap, struct cnn_buffer *buffer);

/**
 * cnn_heap_freelist_drain - drain the deferred free list
 * @heap:		the heap
 * @size:		amount of memory to drain in bytes
 *
 * Drains the indicated amount of memory from the deferred freelist immediately.
 * Returns the total amount freed.  The total freed may be higher depending
 * on the size of the items in the list, or lower if there is insufficient
 * total memory on the freelist.
 */
size_t cnn_heap_freelist_drain(struct cnn_heap *heap, size_t size);

/**
 * cnn_heap_freelist_shrink - drain the deferred free
 *				list, skipping any heap-specific
 *				pooling or caching mechanisms
 *
 * @heap:		the heap
 * @size:		amount of memory to drain in bytes
 *
 * Drains the indicated amount of memory from the deferred freelist immediately.
 * Returns the total amount freed.  The total freed may be higher depending
 * on the size of the items in the list, or lower if there is insufficient
 * total memory on the freelist.
 *
 * Unlike with @cnn_heap_freelist_drain, don't put any pages back into
 * page pools or otherwise cache the pages. Everything must be
 * genuinely free'd back to the system. If you're free'ing from a
 * shrinker you probably want to use this. Note that this relies on
 * the heap.ops.free callback honoring the cnn_PRIV_FLAG_SHRINKER_FREE
 * flag.
 */
size_t cnn_heap_freelist_shrink(struct cnn_heap *heap,
				size_t size);

/**
 * cnn_heap_freelist_size - returns the size of the freelist in bytes
 * @heap:		the heap
 */
size_t cnn_heap_freelist_size(struct cnn_heap *heap);

/**
 * functions for creating and destroying the built in cnn heaps.
 * architectures can add their own custom architecture specific
 * heaps as appropriate.
 */

struct cnn_heap *cnn_heap_create(struct cnn_plat_heap *);
void cnn_heap_destroy(struct cnn_heap *);

struct cnn_heap *cnn_carveout_heap_create(struct cnn_plat_heap *);
void cnn_carveout_heap_destroy(struct cnn_heap *);

/**
 * functcnns for creating and destroying a heap pool -- allows you
 * to keep a pool of pre allocated memory to use from your heap.  Keeping
 * a pool of memory that is ready for dma, ie any cached mapping have been
 * invalidated from the cache, provides a significant performance benefit on
 * many systems
 */

/**
 * struct cnn_page_pool - pagepool struct
 * @high_count:		number of highmem items in the pool
 * @low_count:		number of lowmem items in the pool
 * @high_items:		list of highmem items
 * @low_items:		list of lowmem items
 * @mutex:		lock protecting this struct and especially the count
 *			item list
 * @gfp_mask:		gfp_mask to use from alloc
 * @order:		order of pages in the pool
 * @list:		plist node for list of pools
 * @cached:		it's cached pool or not
 *
 * Allows you to keep a pool of pre allocated pages to use from your heap.
 * Keeping a pool of pages that is ready for dma, ie any cached mapping have
 * been invalidated from the cache, provides a significant performance benefit
 * on many systems
 */
struct cnn_page_pool {
	int high_count;
	int low_count;
	bool cached;
	struct list_head high_items;
	struct list_head low_items;
	struct mutex mutex;
	gfp_t gfp_mask;
	unsigned int order;
	struct plist_node list;
};

struct cnn_page_pool *cnn_page_pool_create(gfp_t gfp_mask, unsigned int order,
					   bool cached);
void cnn_page_pool_destroy(struct cnn_page_pool *pool);
struct page *cnn_page_pool_alloc(struct cnn_page_pool *pool);
void cnn_page_pool_free(struct cnn_page_pool *pool, struct page *page);

/** cnn_page_pool_shrink - shrinks the size of the memory cached in the pool
 * @pool:		the pool
 * @gfp_mask:		the memory type to reclaim
 * @nr_to_scan:		number of items to shrink in pages
 *
 * returns the number of items freed in pages
 */
int cnn_page_pool_shrink(struct cnn_page_pool *pool, gfp_t gfp_mask,
			 int nr_to_scan);

long cnn_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
struct cnn_handle *cnn_import_dma_buf(struct cnn_client *client,
				      struct dma_buf *dmabuf);

#endif /* _CNN_MM_HEAP_H */
