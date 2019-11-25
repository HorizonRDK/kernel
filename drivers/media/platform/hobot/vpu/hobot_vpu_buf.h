#ifndef __HOBOT_VPU_BUF_H__
#define __HOBOT_VPU_BUF_H__

#include "hobot_vpu_utils.h"

typedef struct _video_mm_info_struct {
	unsigned long total_pages;
	unsigned long alloc_pages;
	unsigned long free_pages;
	unsigned long page_size;
} vmem_info_t;

typedef unsigned long long vmem_key_t;

#define VMEM_PAGE_SIZE			(16*1024)
#define MAKE_KEY(_a, _b)		(((vmem_key_t)_a)<<32 | _b)
#define KEY_TO_VALUE(_key)		(_key>>32)

typedef struct page_struct {
	int pageno;
	unsigned long addr;
	int used;
	int alloc_pages;
	int first_pageno;
} page_t;

typedef struct avl_node_struct {
	vmem_key_t key;
	int height;
	page_t *page;
	struct avl_node_struct *left;
	struct avl_node_struct *right;
} avl_node_t;

typedef struct _video_mm_struct {
	avl_node_t *free_tree;
	avl_node_t *alloc_tree;
	page_t *page_list;
	int num_pages;
	unsigned long base_addr;
	unsigned long mem_size;
	int free_page_count;
	int alloc_page_count;
} video_mm_t;

#define VMEM_P_ALLOC(_x)		vmalloc(_x)
#define VMEM_P_FREE(_x)			vfree(_x)

#define VMEM_ASSERT(_exp)		if (!(_exp)) { printk(KERN_INFO "VMEM_ASSERT at %s:%d\n", __FILE__, __LINE__); /*while(1);*/ }
#define VMEM_HEIGHT(_tree)		(_tree == NULL ? -1 : _tree->height)

#define MAX(_a, _b)			(_a >= _b ? _a : _b)

typedef enum {
	LEFT,
	RIGHT
} rotation_dir_t;

typedef struct avl_node_data_struct {
	int key;
	page_t *page;
} avl_node_data_t;

int vmem_init(video_mm_t * mm, unsigned long addr, unsigned long size);
int vmem_exit(video_mm_t * mm);
unsigned long vmem_alloc(video_mm_t * mm, int size, unsigned long pid);
int vmem_free(video_mm_t * mm, unsigned long ptr, unsigned long pid);
int vmem_get_info(video_mm_t * mm, vmem_info_t * info);

#endif /* __HOBOT_VPU_BUF_H__ */
