#include <linux/device.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/memblock.h>
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/idr.h>
#include <linux/sched/task.h>
#include <asm/atomic.h>
#include <linux/printk.h>

#include "cnn_mm_heap.h"

bool cnn_buffer_fault_user_mappings(struct cnn_buffer *buffer)
{
	return (buffer->flags & CNN_FLAG_CACHED) &&
		!(buffer->flags & CNN_FLAG_CACHED_NEEDS_SYNC);
}

bool cnn_buffer_cached(struct cnn_buffer *buffer)
{
	return !!(buffer->flags & CNN_FLAG_CACHED);
}

static inline struct page *cnn_buffer_page(struct page *page)
{
	return (struct page *)((unsigned long)page & ~(1UL));
}

static inline bool cnn_buffer_page_is_dirty(struct page *page)
{
	return !!((unsigned long)page & 1UL);
}

static inline void cnn_buffer_page_dirty(struct page **page)
{
	*page = (struct page *)((unsigned long)(*page) | 1UL);
}

static inline void cnn_buffer_page_clean(struct page **page)
{
	*page = (struct page *)((unsigned long)(*page) & ~(1UL));
}

static void cnn_buffer_add(struct cnn_device *dev,
			   struct cnn_buffer *buffer)
{
	struct rb_node **p = &dev->buffers.rb_node;
	struct rb_node *parent = NULL;
	struct cnn_buffer *entry;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct cnn_buffer, node);

		if (buffer < entry) {
			p = &(*p)->rb_left;
		} else if (buffer > entry) {
			p = &(*p)->rb_right;
		} else {
			pr_err("%s: buffer already found.", __func__);
			BUG();
		}
	}

	rb_link_node(&buffer->node, parent, p);
	rb_insert_color(&buffer->node, &dev->buffers);
}

static struct cnn_buffer *cnn_buffer_create(struct cnn_heap *heap,
					    struct cnn_device *dev,
					    unsigned long len,
					    unsigned long flags)
{
	struct cnn_buffer *buffer;
	struct sg_table *table;
	struct scatterlist *sg;
	int i, ret;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	buffer->heap = heap;
	buffer->flags = flags;
	kref_init(&buffer->ref);

	ret = heap->ops->allocate(heap, buffer, len, flags);

	if (ret) {
		if (!(heap->flags & CNN_HEAP_FLAG_DEFER_FREE))
			goto err2;

		cnn_heap_freelist_drain(heap, 0);
		ret = heap->ops->allocate(heap, buffer, len,
					  flags);
		if (ret)
			goto err2;
	}

	if (buffer->sg_table == NULL) {
		WARN_ONCE(1, "This heap needs to set the sgtable");
		ret = -EINVAL;
		goto err1;
	}

	table = buffer->sg_table;
	buffer->dev = dev;
	buffer->size = len;

	if (cnn_buffer_fault_user_mappings(buffer)) {
		int num_pages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
		struct scatterlist *sg;
		int i, j, k = 0;

		buffer->pages = vmalloc(sizeof(struct page *) * num_pages);
		if (!buffer->pages) {
			ret = -ENOMEM;
			goto err1;
		}

		for_each_sg(table->sgl, sg, table->nents, i) {
			struct page *page = sg_page(sg);

			for (j = 0; j < sg->length / PAGE_SIZE; j++)
				buffer->pages[k++] = page++;
		}
	}

	buffer->dev = dev;
	buffer->size = len;
	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->lock);
	/*
	 * this will set up dma addresses for the sglist -- it is not
	 * technically correct as per the dma api -- a specific
	 * device isn't really taking ownership here.  However, in practice on
	 * our systems the only dma_address space is physical addresses.
	 * Additionally, we can't afford the overhead of invalidating every
	 * allocation via dma_map_sg. The implicit contract here is that
	 * memory coming from the heaps is ready for dma, ie if it has a
	 * cached mapping that mapping has been invalidated
	 */
	for_each_sg(buffer->sg_table->sgl, sg, buffer->sg_table->nents, i) {
		sg_dma_address(sg) = sg_phys(sg);
		sg_dma_len(sg) = sg->length;
	}
	mutex_lock(&dev->buffer_lock);
	cnn_buffer_add(dev, buffer);
	mutex_unlock(&dev->buffer_lock);
	return buffer;

err1:
	heap->ops->free(buffer);
err2:
	kfree(buffer);
	return ERR_PTR(ret);
}

void cnn_buffer_destroy(struct cnn_buffer *buffer)
{
	if (WARN_ON(buffer->kmap_cnt > 0))
		buffer->heap->ops->unmap_kernel(buffer->heap, buffer);
	buffer->heap->ops->free(buffer);
	vfree(buffer->pages);
	kfree(buffer);
}

static void _cnn_buffer_destroy(struct kref *kref)
{
	struct cnn_buffer *buffer = container_of(kref, struct cnn_buffer, ref);
	struct cnn_heap *heap = buffer->heap;
	struct cnn_device *dev = buffer->dev;

	mutex_lock(&dev->buffer_lock);
	rb_erase(&buffer->node, &dev->buffers);
	mutex_unlock(&dev->buffer_lock);

	if (heap->flags & CNN_HEAP_FLAG_DEFER_FREE)
		cnn_heap_freelist_add(heap, buffer);
	else
		cnn_buffer_destroy(buffer);
}

static void cnn_buffer_get(struct cnn_buffer *buffer)
{
	kref_get(&buffer->ref);
}

static int cnn_buffer_put(struct cnn_buffer *buffer)
{
	return kref_put(&buffer->ref, _cnn_buffer_destroy);
}

static void cnn_buffer_add_to_handle(struct cnn_buffer *buffer)
{
	mutex_lock(&buffer->lock);
	buffer->handle_count++;
	mutex_unlock(&buffer->lock);
}

static void cnn_buffer_remove_from_handle(struct cnn_buffer *buffer)
{
	/*
	 * when a buffer is removed from a handle, if it is not in
	 * any other handles, copy the taskcomm and the pid of the
	 * process it's being removed from into the buffer.  At this
	 * point there will be no way to track what processes this buffer is
	 * being used by, it only exists as a dma_buf file descriptor.
	 * The taskcomm and pid can provide a debug hint as to where this fd
	 * is in the system
	 */
	mutex_lock(&buffer->lock);
	buffer->handle_count--;
	BUG_ON(buffer->handle_count < 0);
	if (!buffer->handle_count) {
		struct task_struct *task;

		task = current->group_leader;
		get_task_comm(buffer->task_comm, task);
		buffer->pid = task_pid_nr(task);
	}
	mutex_unlock(&buffer->lock);
}

static struct cnn_handle *cnn_handle_create(struct cnn_client *client,
					    struct cnn_buffer *buffer)
{
	struct cnn_handle *handle;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return ERR_PTR(-ENOMEM);
	kref_init(&handle->ref);
	RB_CLEAR_NODE(&handle->node);
	handle->client = client;
	cnn_buffer_get(buffer);
	cnn_buffer_add_to_handle(buffer);
	handle->buffer = buffer;

	return handle;
}

static void cnn_handle_kmap_put(struct cnn_handle *);

static void cnn_handle_destroy(struct kref *kref)
{
	struct cnn_handle *handle = container_of(kref, struct cnn_handle, ref);
	struct cnn_client *client = handle->client;
	struct cnn_buffer *buffer = handle->buffer;

	mutex_lock(&buffer->lock);
	while (handle->kmap_cnt)
		cnn_handle_kmap_put(handle);
	mutex_unlock(&buffer->lock);

	idr_remove(&client->idr, handle->id);
	if (!RB_EMPTY_NODE(&handle->node))
		rb_erase(&handle->node, &client->handles);

	cnn_buffer_remove_from_handle(buffer);
	cnn_buffer_put(buffer);

	kfree(handle);
}

static void cnn_handle_get(struct cnn_handle *handle)
{
	kref_get(&handle->ref);
}

int cnn_handle_put_nolock(struct cnn_handle *handle)
{
	return kref_put(&handle->ref, cnn_handle_destroy);
}

int cnn_handle_put(struct cnn_handle *handle)
{
	struct cnn_client *client = handle->client;
	int ret;

	mutex_lock(&client->lock);
	ret = cnn_handle_put_nolock(handle);
	mutex_unlock(&client->lock);

	return ret;
}

static struct cnn_handle *cnn_handle_lookup(struct cnn_client *client,
					    struct cnn_buffer *buffer)
{
	struct rb_node *n = client->handles.rb_node;

	while (n) {
		struct cnn_handle *entry = rb_entry(n, struct cnn_handle, node);

		if (buffer < entry->buffer)
			n = n->rb_left;
		else if (buffer > entry->buffer)
			n = n->rb_right;
		else
			return entry;
	}
	return ERR_PTR(-EINVAL);
}

struct cnn_handle *cnn_handle_get_by_id_nolock(struct cnn_client *client,
					       int id)
{
	struct cnn_handle *handle;

	handle = idr_find(&client->idr, id);
	if (handle)
		cnn_handle_get(handle);

	return handle ? handle : ERR_PTR(-EINVAL);
}

struct cnn_handle *cnn_handle_get_by_id(struct cnn_client *client,
					       int id)
{
	struct cnn_handle *handle;

	mutex_lock(&client->lock);
	handle = cnn_handle_get_by_id_nolock(client, id);
	mutex_unlock(&client->lock);

	return handle;
}

static bool cnn_handle_validate(struct cnn_client *client,
				struct cnn_handle *handle)
{
	WARN_ON(!mutex_is_locked(&client->lock));
	return idr_find(&client->idr, handle->id) == handle;
}

static int cnn_handle_add(struct cnn_client *client, struct cnn_handle *handle)
{
	int id;
	struct rb_node **p = &client->handles.rb_node;
	struct rb_node *parent = NULL;
	struct cnn_handle *entry;

	id = idr_alloc(&client->idr, handle, 1, 0, GFP_KERNEL);
	if (id < 0)
		return id;

	handle->id = id;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct cnn_handle, node);

		if (handle->buffer < entry->buffer)
			p = &(*p)->rb_left;
		else if (handle->buffer > entry->buffer)
			p = &(*p)->rb_right;
		else
			WARN(1, "%s: buffer already found.", __func__);
	}

	rb_link_node(&handle->node, parent, p);
	rb_insert_color(&handle->node, &client->handles);

	return 0;
}

struct cnn_handle *cnn_alloc(struct cnn_client *client, size_t len,
			     unsigned int heap_id_mask,
			     unsigned int flags)
{
	struct cnn_handle *handle;
	struct cnn_device *dev = client->dev;
	struct cnn_buffer *buffer = NULL;
	struct cnn_heap *heap;
	int ret;

	pr_debug("%s: len %zu heap_id_mask %u flags %x\n", __func__,
		 len, heap_id_mask, flags);
	/*
	 * traverse the list of heaps available in this system in priority
	 * order.  If the heap type is supported by the client, and matches the
	 * request of the caller allocate from it.  Repeat until allocate has
	 * succeeded or all heaps have been tried
	 */
	len = PAGE_ALIGN(len);

	if (!len)
		return ERR_PTR(-EINVAL);

	down_read(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		/* if the caller didn't specify this heap id */
		if (!((1 << heap->id) & heap_id_mask))
			continue;
		buffer = cnn_buffer_create(heap, dev, len, flags);
		if (!IS_ERR(buffer))
			break;
	}
	up_read(&dev->lock);

	if (buffer == NULL)
		return ERR_PTR(-ENODEV);

	if (IS_ERR(buffer))
		return ERR_CAST(buffer);

	handle = cnn_handle_create(client, buffer);

	/*
	 * cnn_buffer_create will create a buffer with a ref_cnt of 1,
	 * and cnn_handle_create will take a second reference, drop one here
	 */
	cnn_buffer_put(buffer);

	if (IS_ERR(handle))
		return handle;

	mutex_lock(&client->lock);
	ret = cnn_handle_add(client, handle);
	mutex_unlock(&client->lock);
	if (ret) {
		cnn_handle_put(handle);
		handle = ERR_PTR(ret);
	}

	return handle;
}
EXPORT_SYMBOL(cnn_alloc);

void cnn_free_nolock(struct cnn_client *client,
		     struct cnn_handle *handle)
{
	if (!cnn_handle_validate(client, handle)) {
		WARN(1, "%s: invalid handle passed to free.\n", __func__);
		return;
	}
	cnn_handle_put_nolock(handle);
}

void cnn_free(struct cnn_client *client, struct cnn_handle *handle)
{
	BUG_ON(client != handle->client);

	mutex_lock(&client->lock);
	cnn_free_nolock(client, handle);
	mutex_unlock(&client->lock);
}
EXPORT_SYMBOL(cnn_free);

int cnn_phys(struct cnn_client *client, struct cnn_handle *handle,
	     cnn_phys_addr_t *paddr, size_t *len)
{
	struct cnn_buffer *buffer;
	int ret = 0;

	mutex_lock(&client->lock);
	if (!cnn_handle_validate(client, handle)) {
		mutex_unlock(&client->lock);
		return -EINVAL;
	}

	buffer = handle->buffer;

	mutex_unlock(&client->lock);
        *paddr = buffer->paddr;
	return ret;
}
EXPORT_SYMBOL(cnn_phys);


static void *cnn_buffer_kmap_get(struct cnn_buffer *buffer)
{
	void *vaddr;

	if (buffer->kmap_cnt) {
		buffer->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = buffer->heap->ops->map_kernel(buffer->heap, buffer);
	if (WARN_ONCE(vaddr == NULL,
		      "heap->ops->map_kernel should return ERR_PTR on error"))
		return ERR_PTR(-EINVAL);
	if (IS_ERR(vaddr))
		return vaddr;
	buffer->vaddr = vaddr;
	buffer->kmap_cnt++;
	return vaddr;
}

static void *cnn_handle_kmap_get(struct cnn_handle *handle)
{
	struct cnn_buffer *buffer = handle->buffer;
	void *vaddr;

	if (handle->kmap_cnt) {
		handle->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = cnn_buffer_kmap_get(buffer);
	if (IS_ERR(vaddr))
		return vaddr;
	handle->kmap_cnt++;
	return vaddr;
}

static void cnn_buffer_kmap_put(struct cnn_buffer *buffer)
{
	buffer->kmap_cnt--;
	if (!buffer->kmap_cnt) {
		buffer->heap->ops->unmap_kernel(buffer->heap, buffer);
		buffer->vaddr = NULL;
	}
}

static void cnn_handle_kmap_put(struct cnn_handle *handle)
{
	struct cnn_buffer *buffer = handle->buffer;

	if (!handle->kmap_cnt) {
		WARN(1, "%s: Double unmap detected! bailing...\n", __func__);
		return;
	}
	handle->kmap_cnt--;
	if (!handle->kmap_cnt)
		cnn_buffer_kmap_put(buffer);
}

void *cnn_map_kernel(struct cnn_client *client, struct cnn_handle *handle)
{
	struct cnn_buffer *buffer;
	void *vaddr;

	mutex_lock(&client->lock);
	if (!cnn_handle_validate(client, handle)) {
		pr_err("%s: invalid handle passed to map_kernel.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}

	buffer = handle->buffer;

	if (!handle->buffer->heap->ops->map_kernel) {
		pr_err("%s: map_kernel is not implemented by this heap.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-ENODEV);
	}

	mutex_lock(&buffer->lock);
	vaddr = cnn_handle_kmap_get(handle);
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
	return vaddr;
}
EXPORT_SYMBOL(cnn_map_kernel);

void cnn_unmap_kernel(struct cnn_client *client, struct cnn_handle *handle)
{
	struct cnn_buffer *buffer;

	mutex_lock(&client->lock);
	buffer = handle->buffer;
	mutex_lock(&buffer->lock);
	cnn_handle_kmap_put(handle);
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
}
EXPORT_SYMBOL(cnn_unmap_kernel);

static struct mutex debugfs_mutex;
static struct rb_root *cnn_root_client;
static int is_client_alive(struct cnn_client *client)
{
	struct rb_node *node;
	struct cnn_client *tmp;
	struct cnn_device *dev;

	node = cnn_root_client->rb_node;
	dev = container_of(cnn_root_client, struct cnn_device, clients);

	down_read(&dev->lock);
	while (node) {
		tmp = rb_entry(node, struct cnn_client, node);
		if (client < tmp) {
			node = node->rb_left;
		} else if (client > tmp) {
			node = node->rb_right;
		} else {
			up_read(&dev->lock);
			return 1;
		}
	}

	up_read(&dev->lock);
	return 0;
}

static int cnn_debug_client_show(struct seq_file *s, void *unused)
{
	struct cnn_client *client = s->private;
	struct rb_node *n;
	size_t sizes[CNN_NUM_HEAP_IDS] = {0};
	const char *names[CNN_NUM_HEAP_IDS] = {NULL};
	int i;

	mutex_lock(&debugfs_mutex);
	if (!is_client_alive(client)) {
		seq_printf(s, "cnn_client 0x%p dead, can't dump its buffers\n",
			   client);
		mutex_unlock(&debugfs_mutex);
		return 0;
	}

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct cnn_handle *handle = rb_entry(n, struct cnn_handle,
						     node);
		unsigned int id = handle->buffer->heap->id;

		if (!names[id])
			names[id] = handle->buffer->heap->name;
		sizes[id] += handle->buffer->size;
	}
	mutex_unlock(&client->lock);
	mutex_unlock(&debugfs_mutex);

	seq_printf(s, "%16.16s: %16.16s\n", "heap_name", "size_in_bytes");
	for (i = 0; i < CNN_NUM_HEAP_IDS; i++) {
		if (!names[i])
			continue;
		seq_printf(s, "%16.16s: %16zu\n", names[i], sizes[i]);
	}
	return 0;
}

static int cnn_debug_client_open(struct inode *inode, struct file *file)
{
	return single_open(file, cnn_debug_client_show, inode->i_private);
}

static const struct file_operations debug_client_fops = {
	.open = cnn_debug_client_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int cnn_get_client_serial(const struct rb_root *root,
				 const unsigned char *name)
{
	int serial = -1;
	struct rb_node *node;

	for (node = rb_first(root); node; node = rb_next(node)) {
		struct cnn_client *client = rb_entry(node, struct cnn_client,
						     node);

		if (strcmp(client->name, name))
			continue;
		serial = max(serial, client->display_serial);
	}
	return serial + 1;
}

struct cnn_client *cnn_client_create(struct cnn_device *dev,
				     const char *name)
{
	struct cnn_client *client;
	struct task_struct *task;
	struct rb_node **p;
	struct rb_node *parent = NULL;
	struct cnn_client *entry;
	pid_t pid;

	if (!name) {
		pr_err("%s: Name cannot be null\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	get_task_struct(current->group_leader);
	task_lock(current->group_leader);
	pid = task_pid_nr(current->group_leader);
	/*
	 * don't bother to store task struct for kernel threads,
	 * they can't be killed anyway
	 */
	if (current->group_leader->flags & PF_KTHREAD) {
		put_task_struct(current->group_leader);
		task = NULL;
	} else {
		task = current->group_leader;
	}
	task_unlock(current->group_leader);

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		goto err_put_task_struct;

	client->dev = dev;
	client->handles = RB_ROOT;
	idr_init(&client->idr);
	mutex_init(&client->lock);
	client->task = task;
	client->pid = pid;
	client->name = kstrdup(name, GFP_KERNEL);
	if (!client->name)
		goto err_free_client;

	down_write(&dev->lock);
	client->display_serial = cnn_get_client_serial(&dev->clients, name);
	client->display_name = kasprintf(
		GFP_KERNEL, "%s-%d", name, client->display_serial);
	if (!client->display_name) {
		up_write(&dev->lock);
		goto err_free_client_name;
	}
	p = &dev->clients.rb_node;
	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct cnn_client, node);

		if (client < entry)
			p = &(*p)->rb_left;
		else if (client > entry)
			p = &(*p)->rb_right;
	}
	rb_link_node(&client->node, parent, p);
	rb_insert_color(&client->node, &dev->clients);

	client->debug_root = debugfs_create_file(client->display_name, 0664,
						 dev->clients_debug_root,
						 client, &debug_client_fops);
	if (!client->debug_root) {
		char buf[256], *path;

		path = dentry_path(dev->clients_debug_root, buf, 256);
		pr_err("Failed to create client debugfs at %s/%s\n",
		       path, client->display_name);
	}

	up_write(&dev->lock);

	return client;

err_free_client_name:
	kfree(client->name);
err_free_client:
	kfree(client);
err_put_task_struct:
	if (task)
		put_task_struct(current->group_leader);
	return ERR_PTR(-ENOMEM);
}
EXPORT_SYMBOL(cnn_client_create);

void cnn_client_destroy(struct cnn_client *client)
{
	struct cnn_device *dev = client->dev;
	struct rb_node *n;

	mutex_lock(&debugfs_mutex);
	while ((n = rb_first(&client->handles))) {
		struct cnn_handle *handle = rb_entry(n, struct cnn_handle,
						     node);
		cnn_handle_destroy(&handle->ref);
	}

	idr_destroy(&client->idr);

	down_write(&dev->lock);
	if (client->task)
		put_task_struct(client->task);
	rb_erase(&client->node, &dev->clients);
	debugfs_remove_recursive(client->debug_root);
	up_write(&dev->lock);

	kfree(client->display_name);
	kfree(client->name);
	kfree(client);
	mutex_unlock(&debugfs_mutex);
}
EXPORT_SYMBOL(cnn_client_destroy);

static void cnn_buffer_sync_for_device(struct cnn_buffer *buffer,
				       struct device *dev,
				       enum dma_data_direction direction);

static struct sg_table *cnn_map_dma_buf(struct dma_buf_attachment *attachment,
					enum dma_data_direction direction)
{
	struct dma_buf *dmabuf = attachment->dmabuf;
	struct cnn_buffer *buffer = dmabuf->priv;

	cnn_buffer_sync_for_device(buffer, attachment->dev, direction);
	return buffer->sg_table;
}

static void cnn_unmap_dma_buf(struct dma_buf_attachment *attachment,
			      struct sg_table *table,
			      enum dma_data_direction direction)
{
}

void cnn_pages_sync_for_device(struct device *dev, struct page *page,
			       size_t size, enum dma_data_direction dir)
{
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	/*
	 * This is not correct - sg_dma_address needs a dma_addr_t that is valid
	 * for the targeted device, but this works on the currently targeted
	 * hardware.
	 */
	sg_dma_address(&sg) = page_to_phys(page);
	dma_sync_sg_for_device(dev, &sg, 1, dir);
}

struct cnn_vma_list {
	struct list_head list;
	struct vm_area_struct *vma;
};

static void cnn_buffer_sync_for_device(struct cnn_buffer *buffer,
				       struct device *dev,
				       enum dma_data_direction dir)
{
	struct cnn_vma_list *vma_list;
	int pages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
	int i;

	pr_debug("%s: syncing for device %s\n", __func__,
		 dev ? dev_name(dev) : "null");

	if (!cnn_buffer_fault_user_mappings(buffer))
		return;

	mutex_lock(&buffer->lock);
	for (i = 0; i < pages; i++) {
		struct page *page = buffer->pages[i];

		if (cnn_buffer_page_is_dirty(page))
			cnn_pages_sync_for_device(dev, cnn_buffer_page(page),
						  PAGE_SIZE, dir);

		cnn_buffer_page_clean(buffer->pages + i);
	}
	list_for_each_entry(vma_list, &buffer->attachments, list) {
		struct vm_area_struct *vma = vma_list->vma;

		zap_page_range(vma, vma->vm_start, vma->vm_end - vma->vm_start);
	}
	mutex_unlock(&buffer->lock);
}

static int cnn_vm_fault(struct vm_fault *vmf)
{
	struct cnn_buffer *buffer = vmf->vma->vm_private_data;
	unsigned long pfn;
	int ret;

	mutex_lock(&buffer->lock);
	cnn_buffer_page_dirty(buffer->pages + vmf->pgoff);
	BUG_ON(!buffer->pages || !buffer->pages[vmf->pgoff]);

	pfn = page_to_pfn(cnn_buffer_page(buffer->pages[vmf->pgoff]));
	ret = vm_insert_pfn(vmf->vma, (unsigned long)vmf->address, pfn);
	mutex_unlock(&buffer->lock);
	if (ret)
		return VM_FAULT_ERROR;

	return VM_FAULT_NOPAGE;
}

static void cnn_vm_open(struct vm_area_struct *vma)
{
	struct cnn_buffer *buffer = vma->vm_private_data;
	struct cnn_vma_list *vma_list;

	vma_list = kmalloc(sizeof(*vma_list), GFP_KERNEL);
	if (!vma_list)
		return;
	vma_list->vma = vma;
	mutex_lock(&buffer->lock);
	list_add(&vma_list->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);
}

static void cnn_vm_close(struct vm_area_struct *vma)
{
	struct cnn_buffer *buffer = vma->vm_private_data;
	struct cnn_vma_list *vma_list, *tmp;

	mutex_lock(&buffer->lock);
	list_for_each_entry_safe(vma_list, tmp, &buffer->attachments, list) {
		if (vma_list->vma != vma)
			continue;
		list_del(&vma_list->list);
		kfree(vma_list);
		pr_debug("%s: deleting %p\n", __func__, vma);
		break;
	}
	mutex_unlock(&buffer->lock);
}

static const struct vm_operations_struct cnn_vma_ops = {
	.open = cnn_vm_open,
	.close = cnn_vm_close,
	.fault = cnn_vm_fault,
};

static int cnn_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct cnn_buffer *buffer = dmabuf->priv;
	int ret = 0;

	if (!buffer->heap->ops->map_user) {
		pr_err("%s: this heap does not define a method for mapping to userspace\n",
		       __func__);
		return -EINVAL;
	}

	if (cnn_buffer_fault_user_mappings(buffer)) {
		vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND |
							VM_DONTDUMP;
		vma->vm_private_data = buffer;
		vma->vm_ops = &cnn_vma_ops;
		cnn_vm_open(vma);
		return 0;
	}

	if (buffer->flags & CNN_FLAG_CACHED) {
                vma->vm_page_prot = PAGE_KERNEL;
        } else if (buffer->flags & CNN_FLAG_WC) {
                vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
        } else if (buffer->flags & CNN_FLAG_UNCACHE) {
                vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
        } else if (buffer->flags & CNN_FLAG_WT) {
                vma->vm_page_prot =__pgprot(PROT_NORMAL_WT);
        }

	mutex_lock(&buffer->lock);
	/* now map it to userspace */
	ret = buffer->heap->ops->map_user(buffer->heap, buffer, vma);
	mutex_unlock(&buffer->lock);

	if (ret)
		pr_err("%s: failure mapping buffer to userspace\n",
		       __func__);

	return ret;
}

static void cnn_dma_buf_release(struct dma_buf *dmabuf)
{
	struct cnn_buffer *buffer = dmabuf->priv;

	cnn_buffer_put(buffer);
}

static void *cnn_dma_buf_kmap(struct dma_buf *dmabuf, unsigned long offset)
{
	struct cnn_buffer *buffer = dmabuf->priv;

	return buffer->vaddr + offset * PAGE_SIZE;
}

static void cnn_dma_buf_kunmap(struct dma_buf *dmabuf, unsigned long offset,
			       void *ptr)
{
}

static int cnn_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					enum dma_data_direction direction)
{
	struct cnn_buffer *buffer = dmabuf->priv;
	void *vaddr;

	if (!buffer->heap->ops->map_kernel) {
		pr_err("%s: map kernel is not implemented by this heap.\n",
		       __func__);
		return -ENODEV;
	}

	mutex_lock(&buffer->lock);
	vaddr = cnn_buffer_kmap_get(buffer);
	mutex_unlock(&buffer->lock);
	return PTR_ERR_OR_ZERO(vaddr);
}

static int cnn_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
				      enum dma_data_direction direction)
{
	struct cnn_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	cnn_buffer_kmap_put(buffer);
	mutex_unlock(&buffer->lock);

	return 0;
}

static struct dma_buf_ops dma_buf_ops = {
	.map_dma_buf = cnn_map_dma_buf,
	.unmap_dma_buf = cnn_unmap_dma_buf,
	.mmap = cnn_mmap,
	.release = cnn_dma_buf_release,
	.begin_cpu_access = cnn_dma_buf_begin_cpu_access,
	.end_cpu_access = cnn_dma_buf_end_cpu_access,
	.map_atomic = cnn_dma_buf_kmap,
	.unmap_atomic = cnn_dma_buf_kunmap,
	.map = cnn_dma_buf_kmap,
	.unmap = cnn_dma_buf_kunmap,
};

struct dma_buf *cnn_share_dma_buf(struct cnn_client *client,
				  struct cnn_handle *handle)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct cnn_buffer *buffer;
	struct dma_buf *dmabuf;
	bool valid_handle;

	mutex_lock(&client->lock);
	valid_handle = cnn_handle_validate(client, handle);
	if (!valid_handle) {
		WARN(1, "%s: invalid handle passed to share.\n", __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}
	buffer = handle->buffer;
	cnn_buffer_get(buffer);
	mutex_unlock(&client->lock);

	exp_info.ops = &dma_buf_ops;
	exp_info.size = buffer->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buffer;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		cnn_buffer_put(buffer);
		return dmabuf;
	}

	return dmabuf;
}
EXPORT_SYMBOL(cnn_share_dma_buf);

int cnn_share_dma_buf_fd(struct cnn_client *client, struct cnn_handle *handle)
{
	struct dma_buf *dmabuf;
	int fd;

	dmabuf = cnn_share_dma_buf(client, handle);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	if (fd < 0)
		dma_buf_put(dmabuf);

	return fd;
}
EXPORT_SYMBOL(cnn_share_dma_buf_fd);

struct cnn_handle *cnn_import_dma_buf(struct cnn_client *client,
				      struct dma_buf *dmabuf)
{
	struct cnn_buffer *buffer;
	struct cnn_handle *handle;
	int ret;

	/* if this memory came from ion */

	if (dmabuf->ops != &dma_buf_ops) {
		pr_err("%s: can not import dmabuf from another exporter\n",
		       __func__);
		return ERR_PTR(-EINVAL);
	}
	buffer = dmabuf->priv;

	mutex_lock(&client->lock);
	/* if a handle exists for this buffer just take a reference to it */
	handle = cnn_handle_lookup(client, buffer);
	if (!IS_ERR(handle)) {
		cnn_handle_get(handle);
		mutex_unlock(&client->lock);
		goto end;
	}

	handle = cnn_handle_create(client, buffer);
	if (IS_ERR(handle)) {
		mutex_unlock(&client->lock);
		goto end;
	}

	ret = cnn_handle_add(client, handle);
	mutex_unlock(&client->lock);
	if (ret) {
		cnn_handle_put(handle);
		handle = ERR_PTR(ret);
	}

end:
	return handle;
}
EXPORT_SYMBOL(cnn_import_dma_buf);

struct cnn_handle *cnn_import_dma_buf_fd(struct cnn_client *client, int fd)
{
	struct dma_buf *dmabuf;
	struct cnn_handle *handle;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return ERR_CAST(dmabuf);

	handle = cnn_import_dma_buf(client, dmabuf);
	dma_buf_put(dmabuf);
	return handle;
}
EXPORT_SYMBOL(cnn_import_dma_buf_fd);

int cnn_sync_for_device(struct cnn_client *client, int fd)
{
	struct dma_buf *dmabuf;
	struct cnn_buffer *buffer;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	/* if this memory came from ion */
	if (dmabuf->ops != &dma_buf_ops) {
		pr_err("%s: can not sync dmabuf from another exporter\n",
		       __func__);
		dma_buf_put(dmabuf);
		return -EINVAL;
	}
	buffer = dmabuf->priv;

	dma_sync_sg_for_device(NULL, buffer->sg_table->sgl,
			       buffer->sg_table->nents, DMA_BIDIRECTIONAL);
	dma_buf_put(dmabuf);
	return 0;
}

#if 0
int cnn_query_heaps(struct cnn_client *client, struct cnn_heap_query *query)
{
	struct cnn_device *dev = client->dev;
	struct cnn_heap_data __user *buffer = u64_to_user_ptr(query->heaps);
	int ret = -EINVAL, cnt = 0, max_cnt;
	struct cnn_heap *heap;
	struct cnn_heap_data hdata;

	memset(&hdata, 0, sizeof(hdata));

	down_read(&dev->lock);
	if (!buffer) {
		query->cnt = dev->heap_cnt;
		ret = 0;
		goto out;
	}

	if (query->cnt <= 0)
		goto out;

	max_cnt = query->cnt;

	plist_for_each_entry(heap, &dev->heaps, node) {
		strncpy(hdata.name, heap->name, MAX_HEAP_NAME);
		hdata.name[sizeof(hdata.name) - 1] = '\0';
		hdata.type = heap->type;
		hdata.heap_id = heap->id;

		if (copy_to_user(&buffer[cnt], &hdata, sizeof(hdata))) {
			ret = -EFAULT;
			goto out;
		}

		cnt++;
		if (cnt >= max_cnt)
			break;
	}

	query->cnt = cnt;
out:
	up_read(&dev->lock);
	return ret;
}
#endif

static int cnn_release(struct inode *inode, struct file *file)
{
	struct cnn_client *client = file->private_data;

	pr_debug("%s: %d\n", __func__, __LINE__);
	cnn_client_destroy(client);
	return 0;
}

static int cnn_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct cnn_device *dev = container_of(miscdev, struct cnn_device, dev);
	struct cnn_client *client;
	char debug_name[64];

	snprintf(debug_name, 64, "%u", task_pid_nr(current->group_leader));
	client = cnn_client_create(dev, debug_name);
	if (IS_ERR(client))
		return PTR_ERR(client);
	file->private_data = client;

	return 0;
}

/* fix up the cases where the ioctl direction bits are incorrect */
static unsigned int cnn_ioctl_dir(unsigned int cmd)
{
	switch (cmd) {
        case CNN_IOC_SYNC:
	case CNN_IOC_FREE:
	case CNN_IOC_CUSTOM:
		return _IOC_WRITE;

	default:
		return _IOC_DIR(cmd);
	}
}

long cnn_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
        struct cnn_client *client = filp->private_data;
        struct cnn_device *dev = client->dev;
        struct cnn_handle *cleanup_handle = NULL;
        int ret = 0;
        unsigned int dir;
        union cnn_ioctl_arg data;

        dir = cnn_ioctl_dir(cmd);

#ifdef CNN_MM_DEBUG
        pr_info("%s:%d cmd 0x%x size:%d\n", __func__, __LINE__,
                cmd, _IOC_SIZE(cmd));
#endif

        if (_IOC_SIZE(cmd) > sizeof(data)) {
                pr_info("%s:%d invalid cmd:0x%x size:%d\n", __func__, __LINE__
                        ,cmd, _IOC_SIZE(cmd));
                return -EINVAL;
        }

        /*
         * The copy_from_user is unconditional here for both read and write
         * to do the validate. If there is no write for the ioctl, the
         * buffer is cleared
         */
        if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
                pr_info("%s:%d copy data from user failed\n", __func__, __LINE__);
                return -EFAULT;
        }

        if (!(dir & _IOC_WRITE))
                memset(&data, 0, sizeof(data));

        switch (cmd) {
        case CNN_IOC_ALLOC:
        {
                struct cnn_handle *handle;

                handle = cnn_alloc(client, data.allocation.len,
                        data.allocation.heap_id_mask,
                        data.allocation.flags);
                if (IS_ERR(handle)) {
                        return PTR_ERR(handle);
                }
                cnn_phys(client, handle, &data.allocation.phys_addr, data.allocation.len);
                data.allocation.handle = handle->id;

                cleanup_handle = handle;
                break;
        }
        case CNN_IOC_FREE:
        {
                struct cnn_handle *handle;

                mutex_lock(&client->lock);
                handle = cnn_handle_get_by_id_nolock(client, data.handle.handle);
                if (IS_ERR(handle)) {
                        mutex_unlock(&client->lock);
                        return PTR_ERR(handle);
                }
                cnn_free_nolock(client, handle);
                cnn_handle_put_nolock(handle);
                mutex_unlock(&client->lock);
                break;
        }
        case CNN_IOC_SHARE:
        case CNN_IOC_MAP:
        {
                        struct cnn_handle *handle;

                        handle = cnn_handle_get_by_id(client, data.handle.handle);
                        if (IS_ERR(handle)) {
                                return PTR_ERR(handle);
                        }
                        data.fd.fd = cnn_share_dma_buf_fd(client, handle);
                        cnn_handle_put(handle);
                        if (data.fd.fd < 0) {
                                ret = data.fd.fd;
                        }
                        break;
        }
        case CNN_IOC_IMPORT:
        {
                struct cnn_handle *handle;

                handle = cnn_import_dma_buf_fd(client, data.fd.fd);
                if (IS_ERR(handle)) {
                        ret = PTR_ERR(handle);
                }
                else
                        data.handle.handle = handle->id;
                break;
        }
        case CNN_IOC_SYNC:
        {
                ret = cnn_sync_for_device(client, data.fd.fd);
                break;
        }
        case CNN_IOC_CUSTOM:
        {
                if (!dev->custom_ioctl)
                        return -ENOTTY;
                ret = dev->custom_ioctl(client, data.custom.cmd,
                        data.custom.arg);
                break;
        }
	default:
        {
                pr_err("invalid ioctl arg\n");
		return -ENOTTY;
        }
	}

	if (dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			if (cleanup_handle)
				cnn_free(client, cleanup_handle);
			return -EFAULT;
		}
	}
	return ret;
}
static const struct file_operations cnn_fops = {
	.owner          = THIS_MODULE,
	.open           = cnn_open,
	.release        = cnn_release,
	.unlocked_ioctl = cnn_ioctl,
	.compat_ioctl   = cnn_ioctl,
};

static size_t cnn_debug_heap_total(struct cnn_client *client,
				   unsigned int id)
{
	size_t size = 0;
	struct rb_node *n;

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct cnn_handle *handle = rb_entry(n,
						     struct cnn_handle,
						     node);
		if (handle->buffer->heap->id == id)
			size += handle->buffer->size;
	}
	mutex_unlock(&client->lock);
	return size;
}

static int cnn_debug_heap_show(struct seq_file *s, void *unused)
{
	struct cnn_heap *heap = s->private;
	struct cnn_device *dev = heap->dev;
	struct rb_node *n;
	size_t total_size = 0;
	size_t total_orphaned_size = 0;

	seq_printf(s, "%16s %16s %16s\n", "client", "pid", "size");
	seq_puts(s, "----------------------------------------------------\n");

	mutex_lock(&debugfs_mutex);
	for (n = rb_first(&dev->clients); n; n = rb_next(n)) {
		struct cnn_client *client = rb_entry(n, struct cnn_client,
						     node);
		size_t size = cnn_debug_heap_total(client, heap->id);

		if (!size)
			continue;
		if (client->task) {
			char task_comm[TASK_COMM_LEN];

			get_task_comm(task_comm, client->task);
			seq_printf(s, "%16s %16u %16zu\n", task_comm,
				   client->pid, size);
		} else {
			seq_printf(s, "%16s %16u %16zu\n", client->name,
				   client->pid, size);
		}
	}
	mutex_unlock(&debugfs_mutex);

	seq_puts(s, "----------------------------------------------------\n");
	seq_puts(s, "orphaned allocations (info is from last known client):\n");
	mutex_lock(&dev->buffer_lock);
	for (n = rb_first(&dev->buffers); n; n = rb_next(n)) {
		struct cnn_buffer *buffer = rb_entry(n, struct cnn_buffer,
						     node);
		if (buffer->heap->id != heap->id)
			continue;
		total_size += buffer->size;
		if (!buffer->handle_count) {
			seq_printf(s, "%16s %16u %16zu %d %d\n",
				   buffer->task_comm, buffer->pid,
				   buffer->size, buffer->kmap_cnt,
				   atomic_read(&buffer->ref.refcount.refs));
			total_orphaned_size += buffer->size;
		}
	}
	mutex_unlock(&dev->buffer_lock);
	seq_puts(s, "----------------------------------------------------\n");
	seq_printf(s, "%16s %16zu\n", "total orphaned",
		   total_orphaned_size);
	seq_printf(s, "%16s %16zu\n", "total ", total_size);
	if (heap->flags & CNN_HEAP_FLAG_DEFER_FREE)
		seq_printf(s, "%16s %16zu\n", "deferred free",
			   heap->free_list_size);
	seq_puts(s, "----------------------------------------------------\n");

	if (heap->debug_show)
		heap->debug_show(heap, s, unused);

	return 0;
}

static int cnn_debug_heap_open(struct inode *inode, struct file *file)
{
	return single_open(file, cnn_debug_heap_show, inode->i_private);
}

static const struct file_operations debug_heap_fops = {
	.open = cnn_debug_heap_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int debug_shrink_set(void *data, u64 val)
{
	struct cnn_heap *heap = data;
	struct shrink_control sc;
	int objs;

	sc.gfp_mask = GFP_HIGHUSER;
	sc.nr_to_scan = val;

	if (!val) {
		objs = heap->shrinker.count_objects(&heap->shrinker, &sc);
		sc.nr_to_scan = objs;
	}

	heap->shrinker.scan_objects(&heap->shrinker, &sc);
	return 0;
}

static int debug_shrink_get(void *data, u64 *val)
{
	struct cnn_heap *heap = data;
	struct shrink_control sc;
	int objs;

	sc.gfp_mask = GFP_HIGHUSER;
	sc.nr_to_scan = 0;

	objs = heap->shrinker.count_objects(&heap->shrinker, &sc);
	*val = objs;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_shrink_fops, debug_shrink_get,
			debug_shrink_set, "%llu\n");

void cnn_device_add_heap(struct cnn_device *dev, struct cnn_heap *heap)
{
	struct dentry *debug_file;

	if (!heap->ops->allocate || !heap->ops->free)
		pr_err("%s: can not add heap with invalid ops struct.\n",
		       __func__);

	spin_lock_init(&heap->free_lock);
	heap->free_list_size = 0;

	if (heap->flags & CNN_HEAP_FLAG_DEFER_FREE)
		cnn_heap_init_deferred_free(heap);

	if ((heap->flags & CNN_HEAP_FLAG_DEFER_FREE) || heap->ops->shrink)
		cnn_heap_init_shrinker(heap);

	heap->dev = dev;
	down_write(&dev->lock);
	/*
	 * use negative heap->id to reverse the priority -- when traversing
	 * the list later attempt higher id numbers first
	 */
	plist_node_init(&heap->node, -heap->id);
	plist_add(&heap->node, &dev->heaps);
	debug_file = debugfs_create_file(heap->name, 0664,
					 dev->heaps_debug_root, heap,
					 &debug_heap_fops);

	if (!debug_file) {
		char buf[256], *path;

		path = dentry_path(dev->heaps_debug_root, buf, 256);
		pr_err("Failed to create heap debugfs at %s/%s\n",
		       path, heap->name);
	}

	if (heap->shrinker.count_objects && heap->shrinker.scan_objects) {
		char debug_name[64];

		snprintf(debug_name, 64, "%s_shrink", heap->name);
		debug_file = debugfs_create_file(
			debug_name, 0644, dev->heaps_debug_root, heap,
			&debug_shrink_fops);
		if (!debug_file) {
			char buf[256], *path;

			path = dentry_path(dev->heaps_debug_root, buf, 256);
			pr_err("Failed to create heap shrinker debugfs at %s/%s\n",
			       path, debug_name);
		}
	}

	dev->heap_cnt++;
	up_write(&dev->lock);
}
EXPORT_SYMBOL(cnn_device_add_heap);

struct cnn_device *cnn_device_create(long (*custom_ioctl)
				     (struct cnn_client *client,
				      unsigned int cmd,
				      unsigned long arg))
{
	struct cnn_device *idev;
	int ret;

	idev = kzalloc(sizeof(*idev), GFP_KERNEL);
	if (!idev)
		return ERR_PTR(-ENOMEM);

	idev->dev.minor = MISC_DYNAMIC_MINOR;
	idev->dev.name = "cnn_mm";
	idev->dev.fops = &cnn_fops;
	idev->dev.parent = NULL;
	ret = misc_register(&idev->dev);
	if (ret) {
		pr_err("cnn_mm: failed to register misc device.\n");
		kfree(idev);
		return ERR_PTR(ret);
	}

	idev->debug_root = debugfs_create_dir("cnn_mm", NULL);
	if (!idev->debug_root) {
		pr_err("cnn_mm: failed to create debugfs root directory.\n");
		goto debugfs_done;
	}
	idev->heaps_debug_root = debugfs_create_dir("heaps", idev->debug_root);
	if (!idev->heaps_debug_root) {
		pr_err("cnn_mm: failed to create debugfs heaps directory.\n");
		goto debugfs_done;
	}
	idev->clients_debug_root = debugfs_create_dir("clients",
						idev->debug_root);
	if (!idev->clients_debug_root)
		pr_err("cnn_mm: failed to create debugfs clients directory.\n");

debugfs_done:

	idev->custom_ioctl = custom_ioctl;
	idev->buffers = RB_ROOT;
	mutex_init(&idev->buffer_lock);
	init_rwsem(&idev->lock);
	plist_head_init(&idev->heaps);
	idev->clients = RB_ROOT;
	cnn_root_client = &idev->clients;
	mutex_init(&debugfs_mutex);
	return idev;
}
EXPORT_SYMBOL(cnn_device_create);

void cnn_device_destroy(struct cnn_device *dev)
{
	misc_deregister(&dev->dev);
	debugfs_remove_recursive(dev->debug_root);
	/* XXX need to free the heaps and clients ? */
	kfree(dev);
}
EXPORT_SYMBOL(cnn_device_destroy);
