/*
 * YAFFS: Yet another Flash File System . A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2018 Aleph One Ltd.
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1 as
 * published by the Free Software Foundation.
 *
 * Note: Only YAFFS headers are LGPL, YAFFS C code is covered by GPL.
 */

#ifndef FS_YAFFS2_YAFFS_ALLOCATOR_H_
#define FS_YAFFS2_YAFFS_ALLOCATOR_H_

#include "./yaffs_guts.h"

void yaffs_init_raw_tnodes_and_objs(struct yaffs_dev *dev);
void yaffs_deinit_raw_tnodes_and_objs(struct yaffs_dev *dev);

struct yaffs_tnode *yaffs_alloc_raw_tnode(struct yaffs_dev *dev);
void yaffs_free_raw_tnode(struct yaffs_dev *dev, struct yaffs_tnode *tn);

struct yaffs_obj *yaffs_alloc_raw_obj(struct yaffs_dev *dev);
void yaffs_free_raw_obj(struct yaffs_dev *dev, struct yaffs_obj *obj);

#endif  // FS_YAFFS2_YAFFS_ALLOCATOR_H_
