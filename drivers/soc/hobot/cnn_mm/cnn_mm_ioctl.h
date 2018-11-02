#ifndef _CNN_IOCTL_H
#define _CNN_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

typedef int cnn_user_handle_t;
typedef unsigned int cnn_phys_addr_t;

/**
 * enum cnn_heap_types - list of all possible types of heaps
 * @CNN_HEAP_TYPE_SYSTEM:	 memory allocated via vmalloc
 * @CNN_HEAP_TYPE_CARVEOUT:	 memory allocated from a prereserved
 *				 carveout heap, allocatcnns are physically
 *				 contiguous
 * @CNN_NUM_HEAPS:		 helper for iterating over heaps, a bit mask
 *				 is used to identify the heaps, so only 32
 *				 total heap types are supported
 */
enum cnn_heap_type {
	CNN_HEAP_TYPE_SYSTEM,
	CNN_HEAP_TYPE_CARVEOUT,
	CNN_HEAP_TYPE_CUSTOM, /*
			       * must be last so device specific heaps always
			       * are at the end of this enum
			       */
        CNN_NUM_HEAPS = 16,
};

#define CNN_NUM_HEAP_IDS		(sizeof(unsigned int) * 8)

/*
 * mappings of this buffer to uncached(device memory: MT_DEVICE_nGnRnE)
 */
#define CNN_FLAG_UNCACHE 0x0

/*
 * mappings of this buffer should be cached, cnn will do cache maintenance
 * when the buffer is mapped for dma
 */
#define CNN_FLAG_CACHED 0x1

/*
 * mappings of this buffer will created at mmap time, if this is set
 * caches must be managed manually
 */
#define CNN_FLAG_CACHED_NEEDS_SYNC 0x2

/*
 * mappings of this buffer to writecombine(normal memory: MT_NORMAL_NC)
 */
#define CNN_FLAG_WC      0x2

/*
 * mappings of this buffer to writethrough(normal memory: MT_NORMAL_WT)
 */
#define CNN_FLAG_WT      0x3

/**
 * DOC: cnn_mm Userspace API
 *
 * create a client by opening /dev/cnn_mm
 * most operatcnns handled via following ioctls
 *
 */

/**
 * struct cnn_allocation_data - metadata passed from userspace for allocatcnns
 * @len:		size of the allocation
 * @heap_id_mask:	mask of heap ids to allocate from
 * @flags:		flags passed to heap
 * @phys_addr           the phys_addr of memory
 * @handle:		pointer that will be populated with a cookie to use to
 *			refer to this allocation
 *
 * Provided by userspace as an argument to the ioctl
 */
struct cnn_allocation_data {
	unsigned int len;
	unsigned int heap_id_mask;
	unsigned int flags;
	cnn_user_handle_t handle;
        cnn_phys_addr_t phys_addr;
};

#define MAX_HEAP_NAME			32

/**
 * struct cnn_heap_data - data about a heap
 * @name - first 32 characters of the heap name
 * @type - heap type
 * @heap_id - heap id for the heap
 */
struct cnn_heap_data {
	char name[MAX_HEAP_NAME];
	unsigned int type;
	unsigned int heap_id;
};

/**
 * struct cnn_fd_data - metadata passed to/from userspace for a handle/fd pair
 * @handle:	a handle
 * @fd:		a file descriptor representing that handle
 *
 * For CNN_IOC_SHARE or CNN_IOC_MAP userspace populates the handle field with
 * the handle returned from cnn alloc, and the kernel returns the file
 * descriptor to share or map in the fd field.  For CNN_IOC_IMPORT, userspace
 * provides the file descriptor and the kernel returns the handle.
 */
struct cnn_fd_data {
	cnn_user_handle_t handle;
	int fd;
};

/**
 * struct cnn_handle_data - a handle passed to/from the kernel
 * @handle:	a handle
 */
struct cnn_handle_data {
	cnn_user_handle_t handle;
};

/**
 * struct cnn_custom_data - metadata passed to/from userspace for a custom ioctl
 * @cmd:	the custom ioctl function to call
 * @arg:	additional data to pass to the custom ioctl, typically a user
 *		pointer to a predefined structure
 *
 * This works just like the regular cmd and arg fields of an ioctl.
 */
struct cnn_custom_data {
	unsigned int cmd;
	unsigned long arg;
};

union cnn_ioctl_arg {
        struct cnn_fd_data fd;
	struct cnn_allocation_data allocation;
        struct cnn_handle_data handle;
	struct cnn_custom_data custom;
};

#define CNN_IOC_MAGIC		'C'

/**
 * DOC: CNN_IOC_ALLOC - allocate memory
 *
 * Takes an cnn_allocatcnn_data struct and returns it with the handle field
 * populated with the opaque handle for the allocatcnn.
 */
#define CNN_IOC_ALLOC		_IOWR(CNN_IOC_MAGIC, 0, \
				      struct cnn_allocation_data)
/**
 * DOC: CNN_IOC_FREE - free memory
 *
 * Takes an ion_handle_data struct and frees the handle.
 */
#define CNN_IOC_FREE		_IOWR(CNN_IOC_MAGIC, 1, struct cnn_handle_data)

/**
 * DOC: CNN_IOC_MAP - get a file descriptor to mmap
 *
 * Takes an ion_fd_data struct with the handle field populated with a valid
 * opaque handle.  Returns the struct with the fd field set to a file
 * descriptor open in the current address space.  This file descriptor
 * can then be used as an argument to mmap.
 */
#define CNN_IOC_MAP		_IOWR(CNN_IOC_MAGIC, 2, struct cnn_fd_data)

/**
 * DOC: CNN_IOC_SHARE - creates a file descriptor to use to share an allocation
 *
 * Takes an ion_fd_data struct with the handle field populated with a valid
 * opaque handle.  Returns the struct with the fd field set to a file
 * descriptor open in the current address space.  This file descriptor
 * can then be passed to another process.  The corresponding opaque handle can
 * be retrieved via CNN_IOC_IMPORT.
 */
#define CNN_IOC_SHARE		_IOWR(CNN_IOC_MAGIC, 4, struct cnn_fd_data)

/**
 * DOC: CNN_IOC_IMPORT - imports a shared file descriptor
 *
 * Takes an ion_fd_data struct with the fd field populated with a valid file
 * descriptor obtained from CNN_IOC_SHARE and returns the struct with the handle
 * filed set to the corresponding opaque handle.
 */
#define CNN_IOC_IMPORT		_IOWR(CNN_IOC_MAGIC, 5, struct cnn_fd_data)

/**
 * DOC: CNN_IOC_CUSTOM - call architecture specific ion ioctl
 *
 * Takes the argument of the architecture specific ioctl to call and
 * passes appropriate userdata for that ioctl
 */
#define CNN_IOC_CUSTOM		_IOWR(CNN_IOC_MAGIC, 6, struct cnn_custom_data)

/**
 * DOC: CNN_IOC_SYNC - syncs a shared file descriptors to memory
 *
 * Deprecated in favor of using the dma_buf api's correctly (syncing
 * will happen automatically when the buffer is mapped to a device).
 * If necessary should be used after touching a cached buffer from the cpu,
 * this will make the buffer in memory coherent.
 */
#define CNN_IOC_SYNC		_IOWR(CNN_IOC_MAGIC, 7, struct cnn_fd_data)

#endif /* _CNN_IOCTL_H */
