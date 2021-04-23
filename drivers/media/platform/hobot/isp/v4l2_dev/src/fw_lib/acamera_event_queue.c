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

#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__

#include "acamera_event_queue.h"
#include "acamera_fw.h"
#include "acamera_fsm_mgr.h"
#include "acamera_logger.h"
#include "system_spinlock.h"

void acamera_event_queue_clear( acamera_event_queue_ptr_t p_queue )
{
    unsigned long flags;
    acamera_loop_buf_ptr_t p_buf = &( p_queue->buf );

    flags = system_spinlock_lock( p_queue->lock );
    p_buf->head = p_buf->tail = 0;
    system_spinlock_unlock( p_queue->lock, flags );
}

static void acamera_event_queue_reset( acamera_loop_buf_ptr_t p_buf )
{
    p_buf->head = p_buf->tail = 0;
}

void acamera_event_queue_push( acamera_event_queue_ptr_t p_queue, int event )
{
    int err = 0;
    unsigned long flags;
    acamera_loop_buf_ptr_t p_buf = &( p_queue->buf );

    if (p_queue->lock == NULL) {
        pr_err("lock is not inited.\n");
        return;
    }

    flags = system_spinlock_lock( p_queue->lock );

    if ( ( p_buf->head < 0 ) ||
         ( p_buf->head >= p_buf->data_buf_size ) ||
         ( p_buf->tail < 0 ) ||
         ( p_buf->tail >= p_buf->data_buf_size ) ) {
        err = -1;
    }

    int pos = acamera_loop_buffer_write_u8( p_buf, 0, (uint8_t)event );
    if ( pos != p_buf->tail ) {
        p_buf->head = pos;
    } else {
        err = -2;
        acamera_event_queue_reset( p_buf );
    }

    system_spinlock_unlock( p_queue->lock, flags );

    if ( -1 == err ) {
        LOG( LOG_ERR, "Event Queue Corrupted\n" );
    } else if ( -2 == err ) {
        static uint32_t counter = 0;
        if ( !( counter++ & 0x3F ) )
            LOG( LOG_ERR, "Event Queue overflow\n" );
    }
}

int acamera_event_queue_pop( acamera_event_queue_ptr_t p_queue )
{
    int rc = 0;
    unsigned long flags;
    acamera_loop_buf_ptr_t p_buf = &( p_queue->buf );

    flags = system_spinlock_lock( p_queue->lock );
    if ( p_buf->head == p_buf->tail ) {
        rc = -1;
    }

    if ( ( p_buf->head < 0 ) ||
         ( p_buf->head >= p_buf->data_buf_size ) ||
         ( p_buf->tail < 0 ) ||
         ( p_buf->tail >= p_buf->data_buf_size ) ) {
        rc = -2;
    }

    if ( !rc ) {
        int pos;
        rc = acamera_loop_buffer_read_u8( p_buf, 0 );

        pos = p_buf->tail + 1;
        if ( pos >= p_buf->data_buf_size ) {
            pos -= p_buf->data_buf_size;
        }

        p_buf->tail = pos;
    }

    system_spinlock_unlock( p_queue->lock, flags );

    if ( -2 == rc ) {
        LOG( LOG_ERR, "Event Queue Corrupted\n" );
    }

    return rc;
}

int32_t acamera_event_queue_empty( acamera_event_queue_ptr_t p_queue )
{
    int32_t result = 0;
    unsigned long flags;
    acamera_loop_buf_ptr_t p_buf = &( p_queue->buf );
	struct _acamera_fsm_mgr_t *p_fsm_mgr;
	p_fsm_mgr = container_of(p_queue, struct _acamera_fsm_mgr_t, event_queue);

    flags = system_spinlock_lock( p_queue->lock );
    if ( p_buf->head == p_buf->tail ) {
        result = 1;
    }
    system_spinlock_unlock( p_queue->lock, flags );
	// acamera_evt_process_dbg(p_fsm_mgr->p_ctx);

    return result;
}

int32_t acamera_event_queue_has_mask_event( acamera_event_queue_ptr_t p_queue )
{
	int rc = 0;
    int i = 0;
    int filter_cnt = 1;
    int check_pass = 0;
	int pos, event;
	int debug_flag;
	event_id_t event_id;
    unsigned long flags;
	acamera_loop_buf_ptr_t p_buf = &( p_queue->buf );
	struct _acamera_fsm_mgr_t *p_fsm_mgr;
	p_fsm_mgr = container_of(p_queue, struct _acamera_fsm_mgr_t, event_queue);
	int ctx_id = p_fsm_mgr->ctx_id;
	debug_flag = (1 << ctx_id) & isp_debug_mask;
	flags = system_spinlock_lock( p_queue->lock );

	pos = p_buf->tail;

	while (i < filter_cnt && p_buf->head != pos) {

		if ( pos >= p_buf->data_buf_size ) {
			pos -= p_buf->data_buf_size;
		}

		event = p_buf->p_data_buf[pos];
		event_id = (event_id_t)(event);
        if (event_id == event_id_frame_done || event_id == event_id_frame_error) {
            check_pass = 1;
        }

        i++;
		pos++;
	}

    if (check_pass) {
        rc = 1;
    }

	system_spinlock_unlock( p_queue->lock, flags );
	if (rc == 0 && debug_flag == 1) {
		int head;
		flags = system_spinlock_lock( p_queue->lock );
		pos = p_buf->tail;
		head = p_buf->head;
		printk_ratelimited(TAG_EVT_DBG "tail= %d, head = %d\n", pos, head);
		while (head != pos && pos < p_buf->data_buf_size) {
			event = p_buf->p_data_buf[pos++];
			event_id = (event_id_t)(event);
			pos = pos % p_buf->data_buf_size;
			printk_ratelimited(TAG_EVT_DBG "no process event:[%s]\n", event_name[event_id]);
		}
		system_spinlock_unlock( p_queue->lock, flags );
	}
	return rc;
}

extern const char * const event_name[];
int32_t acamera_event_queue_view( acamera_event_queue_ptr_t p_queue )
{
	int rc = 0;
	int pos, event;
	event_id_t event_id;
    unsigned long flags;
	acamera_loop_buf_ptr_t p_buf = &( p_queue->buf );

	flags = system_spinlock_lock( p_queue->lock );

	pos = p_buf->tail;

	while (p_buf->head != pos) {

		if ( pos >= p_buf->data_buf_size ) {
			pos -= p_buf->data_buf_size;
		}

		event = p_buf->p_data_buf[pos];
		event_id=(event_id_t)(event);

		pr_info("Processing event: %d %s\n",event_id,event_name[event_id]);

		pos++;
	}

	if (pos == p_buf->tail)
		pr_info("event queue empty.\n");

	system_spinlock_unlock( p_queue->lock, flags );

	return rc;
}
