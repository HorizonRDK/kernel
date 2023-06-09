/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#ifndef _ISP_V4L2_H_
#define _ISP_V4L2_H_

#include <linux/mutex.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <linux/ion.h>

#include "isp-v4l2-common.h"
#include "isp-v4l2-ctrl.h"
#include "isp-v4l2-stream.h"

typedef struct _isp_v4l2_dev {
    /* device */
    uint32_t ctx_id;
    struct v4l2_device *v4l2_dev;
    struct video_device video_dev;

    /* lock */
    struct mutex mlock;
    struct mutex notify_lock;

    /* file handle array for event notify */
    struct v4l2_fh *fh_ptr[V4L2_STREAM_TYPE_MAX];

    /* streams */
    isp_v4l2_stream_t *pstreams[V4L2_STREAM_TYPE_MAX];
    int32_t stream_id_index[V4L2_STREAM_TYPE_MAX];
    atomic_t stream_on_cnt;

    /* controls */
    isp_v4l2_ctrl_t isp_v4l2_ctrl;

    /* ion memory */
    struct ion_client *client;
    struct ion_handle *handle;
    size_t mem_size;

    /* open counter for stream id */
    atomic_t opened;
    unsigned int stream_mask;
} isp_v4l2_dev_t;


/* V4L2 external interface for probe */
int isp_v4l2_create_instance( struct v4l2_device *v4l2_dev, uint32_t hw_isp_addr );
void isp_v4l2_destroy_instance( void );

/* Stream finder */
int isp_v4l2_find_stream( isp_v4l2_stream_t **ppstream,
                          int ctx_number, isp_v4l2_stream_type_t stream_type );
isp_v4l2_dev_t *isp_v4l2_get_dev( uint32_t ctx_number );

isp_v4l2_stream_type_t fw_to_isp_v4l2_stream_type( acamera_stream_type_t type );

/* Frame ready event */
int isp_v4l2_notify_event( int ctx_num, int stream_id, uint32_t event_type );

#endif
