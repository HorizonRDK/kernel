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

#ifndef _BUFFER_V4L2_STREAM_H_
#define _BUFFER_V4L2_STREAM_H_

#include <linux/videodev2.h>
#include <media/videobuf2-v4l2.h>

typedef struct _dframe_t {
	uint32_t type;        // frame type.
	uint32_t address;     // start of memory block
	uint32_t size;        // total size of the memory in bytes
	uint32_t status;
	void *virt_addr;      // virt address accessed by cpu
	uint32_t frame_id;
} dframe_t;

/* buffer for one video frame */
typedef struct _dwe_v4l2_buffer {
	struct vb2_buffer vb;
	struct list_head list;
} dwe_v4l2_buffer_t;


/**
 * struct dwe_v4l2_stream_t - All internal data for one instance of ISP
 */
typedef struct _dwe_v4l2_stream_t {
	/* Control fields */
	uint32_t ctx_id;
	int stream_started;
	uint32_t last_frame_id;

	/* Stream format */
	struct v4l2_format cur_v4l2_fmt;

	/* Video buffer field*/
	struct list_head stream_buffer_list;
	struct list_head stream_buffer_list_busy;
	spinlock_t slock;

	int fw_frame_seq_count;
} dwe_v4l2_stream_t;

/* stream control interface */
int dwe_stream_put_frame(uint32_t ctx_id, dframe_t *dframes);
int dwe_stream_get_frame(uint32_t ctx_id, dframe_t *dframes);
void dwe_v4l2_stream_deinit_static_resources(void);
int dwe_v4l2_stream_init(dwe_v4l2_stream_t **ppstream, int ctx_num);
void dwe_v4l2_stream_deinit(dwe_v4l2_stream_t *pstream);
int dwe_v4l2_stream_on(dwe_v4l2_stream_t *pstream);
void dwe_v4l2_stream_off(dwe_v4l2_stream_t *pstream);

#endif //buffer_v4l2_stream.h
