/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_JPU_BUF_H__
#define __HOBOT_JPU_BUF_H__

#include "hobot_jpu_utils.h"

typedef unsigned long long jmem_key_t;

#define JMEM_PAGE_SIZE           (16*1024)
#define MAKE_KEY(_a, _b)        (((jmem_key_t)_a)<<32 | _b)
#define KEY_TO_VALUE(_key)      (_key>>32)

typedef struct page_struct {
	int pageno;
	unsigned long addr;
	int used;
	int alloc_pages;
	int first_pageno;
} page_t;

typedef struct avl_node_struct {
	jmem_key_t key;
	int height;
	page_t *page;
	struct avl_node_struct *left;
	struct avl_node_struct *right;
} avl_node_t;

typedef struct _jpu_mm_struct {
	avl_node_t *free_tree;
	avl_node_t *alloc_tree;
	page_t *page_list;
	int num_pages;
	unsigned long base_addr;
	unsigned long mem_size;
	int free_page_count;
	int alloc_page_count;
} jpu_mm_t;

#define VMEM_P_ALLOC(_x)         kmalloc(_x, GFP_KERNEL)
#define VMEM_P_FREE(_x)          kfree(_x)

#define VMEM_ASSERT(_exp)        if (!(_exp)) { printk(KERN_INFO "VMEM_ASSERT at %s:%d\n", __FILE__, __LINE__); /*while(1);*/ }
#define VMEM_HEIGHT(_tree)       (_tree==NULL ? -1 : _tree->height)

#define MAX(_a, _b)        	 (_a >= _b ? _a : _b)

typedef enum {
	LEFT,
	RIGHT
} rotation_dir_t;

typedef struct avl_node_data_struct {
	int key;
	page_t *page;
} avl_node_data_t;

int jmem_init(jpu_mm_t * mm, unsigned long addr, unsigned long size);
int jmem_exit(jpu_mm_t * mm);
unsigned long jmem_alloc(jpu_mm_t * mm, int size, unsigned long pid);
int jmem_free(jpu_mm_t * mm, unsigned long ptr, unsigned long pid);

#endif /* __HOBOT_JPU_BUF_H__ */
