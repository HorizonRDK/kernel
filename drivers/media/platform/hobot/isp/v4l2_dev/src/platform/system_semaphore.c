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

#include "acamera_types.h"
#include "system_semaphore.h"
#include "acamera_fw.h"
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>

extern acamera_firmware_t *acamera_get_firmware_ptr(void);

int32_t system_semaphore_init( semaphore_t *sem )
{
#if USE_SEMA_SYNC
    struct semaphore *sys_sem = kmalloc( sizeof( struct semaphore ), GFP_KERNEL | __GFP_NOFAIL );
    *sem = sys_sem;
    sema_init( sys_sem, 1 );
#else
    wait_queue_head_t *wq = kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL | __GFP_NOFAIL);
    init_waitqueue_head(wq);
    *sem = wq;
#endif

    return 0;
}


int32_t system_semaphore_raise( semaphore_t sem )
{
#if USE_SEMA_SYNC
    struct semaphore *sys_sem = (struct semaphore *)sem;
    up( sys_sem );
#else
    wait_queue_head_t *wq = (wait_queue_head_t *)sem;
    acamera_firmware_t *fw = acamera_get_firmware_ptr();

    atomic_inc(&fw->evt_cnt);
    wake_up(wq);
#endif
    return 0;
}

int32_t system_semaphore_wait( semaphore_t sem, uint32_t timeout_ms )
{
#if USE_SEMA_SYNC
    struct semaphore *sys_sem = (struct semaphore *)sem;

    if ( timeout_ms ) {
        return down_timeout( sys_sem, msecs_to_jiffies( timeout_ms ) );
    } else {
        return down_interruptible( sys_sem );
    }
#else
    wait_queue_head_t *wq = (wait_queue_head_t *)sem;
    acamera_firmware_t *fw = acamera_get_firmware_ptr();

    if (timeout_ms)
        wait_event_interruptible_timeout(*wq, atomic_read(&fw->evt_cnt), msecs_to_jiffies(timeout_ms));
    else
        wait_event_interruptible(*wq, atomic_read(&fw->evt_cnt));

    atomic_dec(&fw->evt_cnt);

    return 0;
#endif
}

int32_t system_semaphore_destroy( semaphore_t sem )
{
#if USE_SEMA_SYNC
    struct semaphore *sys_sem = (struct semaphore *)sem;
    kfree( sys_sem );
#else
    wait_queue_head_t *wq = (wait_queue_head_t *)sem;
    kfree(wq);
#endif
    return 0;
}
