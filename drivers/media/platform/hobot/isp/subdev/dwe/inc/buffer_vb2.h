/*
 *    driver, vb2_buffer interface
 *
 *    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#ifndef _BUFFER_VB2_H_
#define _BUFFER_VB2_H_

#include <linux/videodev2.h>
#include <media/videobuf2-core.h>

#include "buffer_v4l2_stream.h"

/* VB2 control interfaces */
int dwe_vb2_queue_init(struct vb2_queue *q, struct mutex *mlock,
	dwe_v4l2_stream_t *pstream, struct device *dev);
void dwe_vb2_queue_release(struct vb2_queue *q);

#endif
