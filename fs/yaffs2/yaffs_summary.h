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

#ifndef FS_YAFFS2_YAFFS_SUMMARY_H_
#define FS_YAFFS2_YAFFS_SUMMARY_H_

#include "./yaffs_packedtags2.h"


int yaffs_summary_init(struct yaffs_dev *dev);
void yaffs_summary_deinit(struct yaffs_dev *dev);

int yaffs_summary_add(struct yaffs_dev *dev,
			struct yaffs_ext_tags *tags,
			int chunk_in_block);
int yaffs_summary_fetch(struct yaffs_dev *dev,
			struct yaffs_ext_tags *tags,
			int chunk_in_block);
int yaffs_summary_read(struct yaffs_dev *dev,
			struct yaffs_summary_tags *st,
			int blk);
void yaffs_summary_gc(struct yaffs_dev *dev, int blk);


#endif  // FS_YAFFS2_YAFFS_SUMMARY_H_
