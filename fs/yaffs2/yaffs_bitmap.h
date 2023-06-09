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

/*
 * Chunk bitmap manipulations
 */

#ifndef FS_YAFFS2_YAFFS_BITMAP_H_
#define FS_YAFFS2_YAFFS_BITMAP_H_

#include "./yaffs_guts.h"

void yaffs_verify_chunk_bit_id(struct yaffs_dev *dev, int blk, int chunk);
void yaffs_clear_chunk_bits(struct yaffs_dev *dev, int blk);
void yaffs_clear_chunk_bit(struct yaffs_dev *dev, int blk, int chunk);
void yaffs_set_chunk_bit(struct yaffs_dev *dev, int blk, int chunk);
int yaffs_check_chunk_bit(struct yaffs_dev *dev, int blk, int chunk);
int yaffs_still_some_chunks(struct yaffs_dev *dev, int blk);
int yaffs_count_chunk_bits(struct yaffs_dev *dev, int blk);

#endif  // FS_YAFFS2_YAFFS_BITMAP_H_
