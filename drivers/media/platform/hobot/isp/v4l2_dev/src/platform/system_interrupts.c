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

#include "system_interrupts.h"
#include "acamera_firmware_config.h"
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include "acamera_logger.h"
#include "vio_group_api.h"

//#define ENABLE_BOTTOM_HALF_TASKLET

#ifdef ENABLE_BOTTOM_HALF_TASKLET
#include <linux/list.h>
#include "acamera_isp_config.h"

#define ISP_TASKLET_Q_SIZE 16

// tasklet queue command structure
struct isp_tasklet_q_cmd {
    struct list_head list;
    uint32_t irq_status;
    uint8_t cmd_used;
};

// tasklet structure
struct isp_tasklet_t {
    // tasklet
    atomic_t tasklet_irq_cnt;
    spinlock_t tasklet_lock;
    struct tasklet_struct tasklet_obj;
    // tasklet q
    uint8_t tasklet_q_idx;
    struct list_head tasklet_q;
    struct isp_tasklet_q_cmd tasklet_q_cmd[ISP_TASKLET_Q_SIZE];
};

// gloval variable for tasklet access
static struct isp_tasklet_t isp_tasklet;
static int temp_irq = 0;
#endif

typedef enum {
    ISP_IRQ_STATUS_DEINIT = 0,
    ISP_IRQ_STATUS_ENABLED,
    ISP_IRQ_STATUS_DISABLED,
    ISP_IRQ_STATUS_MAX
} irq_status;

static system_interrupt_handler_t app_handler = NULL;
static void *app_param = NULL;
int interrupt_line_ACAMERA_JUNO_IRQ = -1;
static int interrupt_line_ACAMERA_JUNO_IRQ_FLAGS = -1;
static irq_status interrupt_request_status = ISP_IRQ_STATUS_DEINIT;

#ifdef ENABLE_BOTTOM_HALF_TASKLET
void isp_do_tasklet( unsigned long data )
{
    unsigned long flags;
    uint32_t irq_status;
    struct isp_tasklet_q_cmd *queue_cmd;

    while ( atomic_read( &isp_tasklet.tasklet_irq_cnt ) ) {
        spin_lock_irqsave( &isp_tasklet.tasklet_lock, flags );
        queue_cmd = list_first_entry( &isp_tasklet.tasklet_q, struct isp_tasklet_q_cmd, list );
        if ( !queue_cmd ) {
            atomic_set( &isp_tasklet.tasklet_irq_cnt, 0 );
            spin_unlock_irqrestore( &isp_tasklet.tasklet_lock, flags );
            return;
        }
        atomic_sub( 1, &isp_tasklet.tasklet_irq_cnt );
        list_del( &queue_cmd->list );
        queue_cmd->cmd_used = 0;
        irq_status = queue_cmd->irq_status;

        spin_unlock_irqrestore( &isp_tasklet.tasklet_lock, flags );

        LOG( LOG_INFO, "debug value on tasklet: 0x%x\n", irq_status );
    }
}
#endif
irqreturn_t system_interrupt_handler( int irq, void *dev_id )
{
#ifdef ENABLE_BOTTOM_HALF_TASKLET
    unsigned long flags;
    uint32_t irq_status;
    struct isp_tasklet_q_cmd *queue_cmd;
    irq_status = temp_irq++;
    LOG( LOG_INFO, "debug value: 0x%x\n", irq_status );

    spin_lock_irqsave( &isp_tasklet.tasklet_lock, flags );
    queue_cmd = &isp_tasklet.tasklet_q_cmd[isp_tasklet.tasklet_q_idx];
    if ( queue_cmd->cmd_used ) {
        LOG( LOG_ERR, "Error: isp tasklet queue overflow !\n" );
        list_del( &queue_cmd->list );
    } else {
        atomic_add( 1, &isp_tasklet.tasklet_irq_cnt );
    }
    queue_cmd->irq_status = irq_status;
    queue_cmd->cmd_used = 1;
    isp_tasklet.tasklet_q_idx =
        ( isp_tasklet.tasklet_q_idx + 1 ) % ISP_TASKLET_Q_SIZE;
    list_add_tail( &queue_cmd->list, &isp_tasklet.tasklet_q );
    spin_unlock_irqrestore( &isp_tasklet.tasklet_lock, flags );

    tasklet_schedule( &isp_tasklet.tasklet_obj );

    if ( app_handler )
        app_handler( app_param, 0 );
#else
    LOG( LOG_DEBUG, "interrupt comes in (irq = %d)\n", irq );
    if ( app_handler )
        app_handler( app_param, 0 );
#endif

    return IRQ_HANDLED;
}

void system_interrupts_set_irq( int irq_num, int flags )
{
    interrupt_line_ACAMERA_JUNO_IRQ = irq_num;
    interrupt_line_ACAMERA_JUNO_IRQ_FLAGS = flags & IRQF_TRIGGER_MASK;
    LOG( LOG_INFO, "interrupt id is set to %d\n", interrupt_line_ACAMERA_JUNO_IRQ );
}

void system_interrupts_irq_affinity_set(int suspend)
{
	vio_irq_affinity_set(interrupt_line_ACAMERA_JUNO_IRQ, MOD_ISP, suspend, 0);
}

void system_interrupts_init( void )
{
    int ret = 0;

    if ( interrupt_line_ACAMERA_JUNO_IRQ < 0 ) {
        LOG( LOG_ERR, "invalid irq id ! (id = %d)\n", interrupt_line_ACAMERA_JUNO_IRQ );
    }

    if ( interrupt_request_status != ISP_IRQ_STATUS_DEINIT ) {
        LOG( LOG_WARNING, "irq %d is already initied (status = %d)",
             interrupt_line_ACAMERA_JUNO_IRQ, interrupt_request_status );
        return;
    }
    interrupt_request_status = ISP_IRQ_STATUS_ENABLED;

#ifdef ENABLE_BOTTOM_HALF_TASKLET
    // init tasklet and tasklet_q
    atomic_set( &isp_tasklet.tasklet_irq_cnt, 0 );
    spin_lock_init( &isp_tasklet.tasklet_lock );
    isp_tasklet.tasklet_q_idx = 0;
    INIT_LIST_HEAD( &isp_tasklet.tasklet_q );
    tasklet_init( &isp_tasklet.tasklet_obj, isp_do_tasklet, (unsigned long)&isp_tasklet );
#endif

    // No dev_id for now, but will need this to be shared
    if ( ( ret = request_irq( interrupt_line_ACAMERA_JUNO_IRQ,
                              &system_interrupt_handler, interrupt_line_ACAMERA_JUNO_IRQ_FLAGS, "isp", NULL ) ) ) {
        LOG( LOG_ERR, "Could not get interrupt %d (ret=%d)\n", interrupt_line_ACAMERA_JUNO_IRQ, ret );
    } else {
        vio_irq_affinity_set(interrupt_line_ACAMERA_JUNO_IRQ, MOD_ISP, 0, 0);
        system_interrupts_disable();
        LOG( LOG_INFO, "Interrupt %d requested (flags = 0x%x, ret = %d)\n",
             interrupt_line_ACAMERA_JUNO_IRQ, interrupt_line_ACAMERA_JUNO_IRQ_FLAGS, ret );
    }
}

void system_interrupts_deinit( void )
{

    if ( interrupt_request_status == ISP_IRQ_STATUS_DEINIT ) {
        LOG( LOG_WARNING, "irq %d is already deinitied (status = %d)",
             interrupt_line_ACAMERA_JUNO_IRQ, interrupt_request_status );
    } else {
        //interrupt_request_status = ISP_IRQ_STATUS_DISABLED;
        interrupt_request_status = ISP_IRQ_STATUS_DEINIT;
        // No dev_id for now, but will need this to be shared
        free_irq( interrupt_line_ACAMERA_JUNO_IRQ, NULL );
        LOG( LOG_INFO, "Interrupt %d released\n", interrupt_line_ACAMERA_JUNO_IRQ );
    }
    app_handler = NULL;
    app_param = NULL;

#ifdef ENABLE_BOTTOM_HALF_TASKLET
    // kill tasklet
    tasklet_kill( &isp_tasklet.tasklet_obj );
    atomic_set( &isp_tasklet.tasklet_irq_cnt, 0 );
#endif
}

void system_interrupt_set_handler( system_interrupt_handler_t handler, void *param )
{
    app_handler = handler;
    app_param = param;
}

void system_interrupts_enable( void )
{
    if ( interrupt_request_status == ISP_IRQ_STATUS_DISABLED ) {
        enable_irq( interrupt_line_ACAMERA_JUNO_IRQ );
        interrupt_request_status = ISP_IRQ_STATUS_ENABLED;
    }
}

void system_interrupts_disable( void )
{
    if ( interrupt_request_status == ISP_IRQ_STATUS_ENABLED ) {
        disable_irq( interrupt_line_ACAMERA_JUNO_IRQ );
        interrupt_request_status = ISP_IRQ_STATUS_DISABLED;
    }
}
