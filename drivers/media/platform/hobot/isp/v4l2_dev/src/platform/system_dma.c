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
#include <linux/kernel.h> /* //printk() */
#include <asm/uaccess.h>
#include <linux/gfp.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <asm/types.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/time.h>
#include "acamera_types.h"
#include "acamera_logger.h"
#include "system_dma.h"
#include "vio_group_api.h"

#if FW_USE_HOBOT_DMA
hobot_dma_t g_hobot_dma = {
    .init_cnt = 0,
    .enable_irq_cnt = 0,
    .irq_in_dts = 0,
    .is_busy = 0,
    .nents_total = 0,
    .dma_ctrl_lock = NULL
};
#endif

#if HOBOT_REGISTER_MONITOR
#define HRM_RC_DMA_START 0x33310000
#define HRM_RC_DMA_END   0x33320000
void hobot_rm_check_n_record(uint32_t offset, uint32_t value);
#endif//HOBOT_REGISTER_MONITOR

int32_t system_dma_init( void **ctx )
{
    int32_t i, result = 0;
    int32_t idx;

    if ( ctx != NULL ) {

        *ctx = system_malloc( sizeof( system_dma_device_t ) );
        system_dma_device_t *system_dma_device = (system_dma_device_t *)*ctx;

        if ( !( *ctx ) ) {
            LOG( LOG_CRIT, "No memory for ctx" );
            return -1;
        }

#if FW_USE_SYSTEM_DMA
        struct dma_chan *dma_channel = NULL;
        dma_cap_mask_t mask;
        dma_cap_zero( mask );
        dma_cap_set( DMA_MEMCPY, mask );

        for ( i = 0; i < SYSTEM_DMA_MAX_CHANNEL; i++ ) {
            dma_channel = dma_request_channel( mask, 0, NULL );
            LOG( LOG_INFO, "allocating dma" );
            if ( dma_channel != NULL ) {
                system_dma_device->dma_channel[i] = dma_channel;
            } else {
                kfree( *ctx );
                LOG( LOG_CRIT, "Failed to request DMA channel" );
                return -1;
            }
        }

        for ( idx = 0; idx < FIRMWARE_CONTEXT_NUMBER; idx++ ) {
            for ( i = 0; i < SYSTEM_DMA_TOGGLE_COUNT; i++ ) {
                system_dma_device->sg_device_nents[idx][i] = 0;
                system_dma_device->sg_fwmem_nents[idx][i] = 0;
            }
        }

#elif FW_USE_HOBOT_DMA
       hobot_dma_init(&g_hobot_dma);
    //    hobot_dma_enable_irq(&g_hobot_dma);

        for ( idx = 0; idx < FIRMWARE_CONTEXT_NUMBER; idx++ ) {
            for ( i = 0; i < SYSTEM_DMA_TOGGLE_COUNT; i++ ) {
                system_dma_device->sg_device_nents[idx][i] = 0;
                system_dma_device->sg_fwmem_nents[idx][i] = 0;
            }
        }
#else
        system_dma_device->name = "TSK_DMA";
        for ( idx = 0; idx < FIRMWARE_CONTEXT_NUMBER; idx++ ) {
            for ( i = 0; i < SYSTEM_DMA_TOGGLE_COUNT; i++ ) {
                system_dma_device->sg_device_nents[idx][i] = 0;
                system_dma_device->sg_fwmem_nents[idx][i] = 0;
                system_dma_device->mem_addrs[idx][i] = 0;
            }
        }
#endif
    } else {
        result = -1;
        LOG( LOG_ERR, "Input ctx pointer is NULL" );
    }

    return result;
}


int32_t system_dma_destroy( void *ctx )
{
    int32_t i, result = 0;
    int32_t idx;

    if ( ctx != 0 ) {
        system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;

#if FW_USE_SYSTEM_DMA
        for ( i = 0; i < SYSTEM_DMA_MAX_CHANNEL; i++ ) {
            dma_release_channel( system_dma_device->dma_channel[i] );
        }

        for ( idx = 0; idx < FIRMWARE_CONTEXT_NUMBER; idx++ ) {
            for ( i = 0; i < SYSTEM_DMA_TOGGLE_COUNT; i++ ) {
                if ( system_dma_device->sg_device_nents[idx][i] )
                    sg_free_table( &system_dma_device->sg_device_table[idx][i] );

                if ( system_dma_device->sg_fwmem_nents[idx][i] )
                    sg_free_table( &system_dma_device->sg_device_table[idx][i] );

                if ( system_dma_device->fwmem_pair_flush[idx][i] )
                    kfree( system_dma_device->fwmem_pair_flush[idx][i] );
            }
        }
#elif FW_USE_HOBOT_DMA
        //hobot_dma_disable_irq(&g_hobot_dma);
        hobot_dma_deinit(&g_hobot_dma);

        for ( idx = 0; idx < FIRMWARE_CONTEXT_NUMBER; idx++ ) {
            for ( i = 0; i < SYSTEM_DMA_TOGGLE_COUNT; i++ ) {
                if ( system_dma_device->sg_device_nents[idx][i] )
                    sg_free_table( &system_dma_device->sg_device_table[idx][i] );

                if ( system_dma_device->sg_fwmem_nents[idx][i] )
                    sg_free_table( &system_dma_device->sg_device_table[idx][i] );

                if ( system_dma_device->fwmem_pair_flush[idx][i] )
                    kfree( system_dma_device->fwmem_pair_flush[idx][i] );
            }
        }

#else
        for ( idx = 0; idx < FIRMWARE_CONTEXT_NUMBER; idx++ ) {
            for ( i = 0; i < SYSTEM_DMA_TOGGLE_COUNT; i++ ) {
                if ( system_dma_device->mem_addrs[idx][i] ) {
                    int j;
                    for ( j = 0; j < system_dma_device->sg_device_nents[idx][i]; j++ )
                        iounmap( system_dma_device->mem_addrs[idx][i][j].dev_addr );
                    kfree( system_dma_device->mem_addrs[idx][i] );
                }
            }
        }
#endif

        kfree( ctx );

    } else {
        LOG( LOG_ERR, "Input ctx pointer is NULL" );
    }
    return result;
}

#if FW_USE_HOBOT_DMA
void system_dma_desc_flush(void)
{
    unsigned long flags;

    flags = system_spinlock_lock( g_hobot_dma.dma_ctrl_lock );
    if (!list_empty(&g_hobot_dma.pending_list)) {
        list_splice_tail_init(&g_hobot_dma.pending_list, &g_hobot_dma.free_list);
    }
    if (!list_empty(&g_hobot_dma.active_list)) {
        list_splice_tail_init(&g_hobot_dma.active_list, &g_hobot_dma.free_list);
    }
    if (!list_empty(&g_hobot_dma.done_list)) {
        list_splice_tail_init(&g_hobot_dma.done_list, &g_hobot_dma.free_list);
    }

    g_hobot_dma.nents_total = 0;
    g_hobot_dma.is_busy = 0;

    system_spinlock_unlock( g_hobot_dma.dma_ctrl_lock, flags );
}

void system_dma_irq_affinity_set(int suspend)
{
    vio_irq_affinity_set(g_hobot_dma.irq_in_dts, MOD_IDMA, suspend, 0);
}
#endif

static void dma_complete_func( void *ctx )
{
    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;

    pr_debug("IRQ completion called\n");
#if (FW_USE_HOBOT_DMA==0)
    unsigned int nents_done = atomic_inc_return( &system_dma_device->nents_done );
    if ( nents_done >= system_dma_device->sg_device_nents[system_dma_device->cur_fw_ctx_id][system_dma_device->buff_loc] )
#endif
    {
        if ( system_dma_device->complete_func ) {
            system_dma_device->complete_func( ctx );
            pr_debug("async completed on buff:%d dir:%d", system_dma_device->buff_loc, system_dma_device->direction );
        } else {
            complete( &system_dma_device->comp );
            pr_debug("sync completed on buff:%d dir:%d", system_dma_device->buff_loc, system_dma_device->direction );
        }
    }
}

#if FW_USE_SYSTEM_DMA
//sg from here
int32_t system_dma_sg_device_setup( void *ctx, int32_t buff_loc, dma_addr_pair_t *device_addr_pair, int32_t addr_pairs, uint32_t fw_ctx_id )
{
    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;
    struct scatterlist *sg;
    int i, ret;

    if ( !system_dma_device || !device_addr_pair || !addr_pairs || buff_loc >= SYSTEM_DMA_TOGGLE_COUNT || fw_ctx_id >= FIRMWARE_CONTEXT_NUMBER )
        return -1;

    struct sg_table *table = &system_dma_device->sg_device_table[fw_ctx_id][buff_loc];

    /* Allocate the scatterlist table */
    ret = sg_alloc_table( table, addr_pairs, GFP_KERNEL );
    if ( ret ) {
        LOG( LOG_CRIT, "unable to allocate DMA table\n" );
        return ret;
    }
    system_dma_device->sg_device_nents[fw_ctx_id][buff_loc] = addr_pairs;

    /* Add the DATA FPGA registers to the scatterlist */
    sg = table->sgl;
    for ( i = 0; i < addr_pairs; i++ ) {
        sg_dma_address( sg ) = device_addr_pair[i].address;
        sg_dma_len( sg ) = device_addr_pair[i].size;
        sg = sg_next( sg );
    }
    LOG( LOG_INFO, "dma device setup success %d", system_dma_device->sg_device_nents[fw_ctx_id][buff_loc] );
    return 0;
}

int32_t system_dma_sg_fwmem_setup( void *ctx, int32_t buff_loc, fwmem_addr_pair_t *fwmem_pair, int32_t addr_pairs, uint32_t fw_ctx_id )
{
    //unsigned int nr_pages;
    int i, ret;
    struct scatterlist *sg;
    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;

    if ( !system_dma_device || !fwmem_pair || !addr_pairs || buff_loc >= SYSTEM_DMA_TOGGLE_COUNT || fw_ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "null param problems" );
        return -1;
    }

    struct sg_table *table = &system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc];
    /* Allocate the scatterlist table */
    ret = sg_alloc_table( table, addr_pairs, GFP_KERNEL );
    if ( ret ) {
        LOG( LOG_CRIT, "unable to allocate DMA table\n" );
        return ret;
    }
    system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc] = addr_pairs;
    system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc] = kmalloc( sizeof( fwmem_addr_pair_t ) * addr_pairs, GFP_KERNEL );
    if ( !system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc] ) {
        LOG( LOG_CRIT, "Failed to allocate virtual address pairs for flushing!!" );
        return -1;
    }
    for ( i = 0; i < addr_pairs; i++ ) {
        system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc][i] = fwmem_pair[i];
    }
    sg = table->sgl;
    for ( i = 0; i < addr_pairs; i++ ) {
        sg_set_buf( sg, fwmem_pair[i].address, fwmem_pair[i].size );
        sg = sg_next( sg );
    }

    LOG( LOG_INFO, "fwmem setup success %d", system_dma_device->sg_device_nents[fw_ctx_id][buff_loc] );

    return 0;
}

void system_dma_unmap_sg( void *ctx )
{
    if ( !ctx )
        return;
    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;
    int32_t buff_loc = system_dma_device->buff_loc;
    uint32_t direction = system_dma_device->direction;
    uint32_t cur_fw_ctx_id = system_dma_device->cur_fw_ctx_id;
    int i;
    for ( i = 0; i < SYSTEM_DMA_MAX_CHANNEL; i++ ) {
        struct dma_chan *chan = system_dma_device->dma_channel[i];
        dma_unmap_sg( chan->device->dev, system_dma_device->sg_fwmem_table[cur_fw_ctx_id][buff_loc].sgl, system_dma_device->sg_fwmem_nents[cur_fw_ctx_id][buff_loc], direction );
    }
}

//FW_USE_SYSTEM_DMA
int32_t system_dma_copy_sg( void *ctx, int32_t buff_loc, uint32_t direction, dma_completion_callback complete_func, uint32_t fw_ctx_id )
{
    int32_t i, result = 0;
    if ( !ctx ) {
        LOG( LOG_ERR, "Input ctx pointer is NULL" );
        return -1;
    }

    int32_t async_dma = 0;

    if ( complete_func != NULL ) {
        async_dma = 1;
    }

    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;

    struct scatterlist *dst_sg, *src_sg;
    unsigned int dst_nents, src_nents;
    struct dma_chan *chan = system_dma_device->dma_channel[0]; //probe the first channel
    struct dma_async_tx_descriptor *tx = NULL;
    dma_cookie_t cookie;
    enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_FENCE | DMA_PREP_INTERRUPT;

    if ( direction == SYS_DMA_TO_DEVICE ) {
        dst_sg = system_dma_device->sg_device_table[fw_ctx_id][buff_loc].sgl;
        dst_nents = system_dma_device->sg_device_nents[fw_ctx_id][buff_loc];
        src_sg = system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc].sgl;
        src_nents = system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc];
        direction = DMA_TO_DEVICE;
    } else {
        src_sg = system_dma_device->sg_device_table[fw_ctx_id][buff_loc].sgl;
        src_nents = system_dma_device->sg_device_nents[fw_ctx_id][buff_loc];
        dst_sg = system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc].sgl;
        dst_nents = system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc];
        direction = DMA_FROM_DEVICE;
    }

    if ( src_nents != dst_nents || !src_nents || !dst_nents ) {
        LOG( LOG_ERR, "Unbalance src_nents:%d dst_nents:%d", src_nents, dst_nents );
        return -1;
    }

    //flush memory before transfer
    for ( i = 0; i < src_nents; i++ ) {
        flush_icache_range( (unsigned long)( system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc][i].address ), (unsigned long)( (uintptr_t)system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc][i].address + system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc][i].size ) );
    }

    for ( i = 0; i < SYSTEM_DMA_MAX_CHANNEL; i++ ) {
        chan = system_dma_device->dma_channel[i];

        result = dma_map_sg( chan->device->dev, system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc].sgl, system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc], direction );
        if ( result <= 0 ) {
            LOG( LOG_CRIT, "unable to map %d", result );
            return -1;
        }
        LOG( LOG_DEBUG, "src_nents:%d src_sg:%p dst_nents:%d dst_sg:%p dma map res:%d", src_nents, src_sg, dst_nents, dst_sg, result );
    }

    system_dma_device->cur_fw_ctx_id = fw_ctx_id;

#if (HOBOT_REGISTER_MONITOR)
    hobot_rm_check_n_record(HRM_RC_DMA_START,buff_loc);
#endif
    /*
     * All buffers passed to this function should be ready and mapped
     * for DMA already. Therefore, we don't need to do anything except
     * submit it to the Freescale DMA Engine for processing
     */
    atomic_set( &system_dma_device->nents_done, 0 ); //set the number of nents done
    if ( async_dma == 0 ) {
        system_dma_device->complete_func = NULL; //async mode is not allowed to have callback
        init_completion( &system_dma_device->comp );
    } else {
        system_dma_device->complete_func = complete_func; //call this function if all nents are done;
    }
    system_dma_device->direction = direction;
    system_dma_device->buff_loc = buff_loc;

    if ( !chan->device->device_prep_dma_sg ) {
        LOG( LOG_DEBUG, "missing device_prep_dma_sg %p %p", chan->device->device_prep_dma_sg, chan->device->device_prep_interleaved_dma );

        for ( i = 0; i < src_nents; i++ ) {

            struct dma_chan *chan = system_dma_device->dma_channel[i];

            uint32_t dst = sg_dma_address( dst_sg );
            uint32_t src = sg_dma_address( src_sg );
            uint32_t size_to_copy = sg_dma_len( src_sg );
            LOG( LOG_DEBUG, "src:0x%x (%d) to dst:0x%x (%d)", src, size_to_copy, dst, sg_dma_len( dst_sg ) );

            tx = chan->device->device_prep_dma_memcpy( chan, dst, src, size_to_copy, flags );
            if ( tx ) {
                tx->callback = dma_complete_func;
                tx->callback_param = ctx;
                cookie = tx->tx_submit( tx );
                if ( dma_submit_error( cookie ) ) {
                    LOG( LOG_CRIT, "unable to submit scatterlist DMA\n" );
                    return -ENOMEM;
                }
            } else {
                LOG( LOG_CRIT, "unable to prep scatterlist DMA\n" );
                return -ENOMEM;
            }
            dma_async_issue_pending( chan );
            //}
            dst_sg = sg_next( dst_sg );
            src_sg = sg_next( src_sg );
        }

    } else {
        chan = system_dma_device->dma_channel[0];
        /* setup the scatterlist to scatterlist transfer */
        tx = chan->device->device_prep_dma_sg( chan,
                                               dst_sg, dst_nents,
                                               src_sg, src_nents,
                                               0 );
        if ( tx ) {
            tx->callback = dma_complete_func;
            tx->callback_param = ctx;
            cookie = tx->tx_submit( tx );
            if ( dma_submit_error( cookie ) ) {
                LOG( LOG_CRIT, "unable to submit scatterlist DMA\n" );
                return -ENOMEM;
            }
        } else {
            LOG( LOG_CRIT, "unable to prep scatterlist DMA\n" );
            return -ENOMEM;
        }

        atomic_set( &system_dma_device->nents_done, dst_nents - 1 ); //only need to issue once
        dma_async_issue_pending( chan );
    }


    if ( async_dma == 0 ) {
        LOG( LOG_DEBUG, "scatterlist DMA waiting completion\n" );
        wait_for_completion( &system_dma_device->comp );
        for ( i = 0; i < SYSTEM_DMA_MAX_CHANNEL; i++ ) {
            chan = system_dma_device->dma_channel[i];
            dma_unmap_sg( chan->device->dev, system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc].sgl, system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc], direction );
        }
    }
#if (HOBOT_REGISTER_MONITOR)
    hobot_rm_check_n_record(HRM_RC_DMA_END,buff_loc);
#endif

    LOG( LOG_DEBUG, "scatterlist DMA success\n" );
    return result;
}
#elif FW_USE_HOBOT_DMA
//sg from here
int32_t system_dma_sg_device_setup( void *ctx, int32_t buff_loc, dma_addr_pair_t *device_addr_pair, int32_t addr_pairs, uint32_t fw_ctx_id )
{//for FW_USE_HOBOT_DMA
    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;
    struct scatterlist *sg;
    int i, ret;

    if ( !system_dma_device || !device_addr_pair || !addr_pairs || buff_loc >= SYSTEM_DMA_TOGGLE_COUNT || fw_ctx_id >= FIRMWARE_CONTEXT_NUMBER )
        return -1;

    struct sg_table *table = &system_dma_device->sg_device_table[fw_ctx_id][buff_loc];
    /* Allocate the scatterlist table */
    ret = sg_alloc_table( table, addr_pairs, GFP_KERNEL );
    if ( ret ) {
        LOG( LOG_CRIT, "unable to allocate DMA table\n" );
        return ret;
    }
    system_dma_device->sg_device_nents[fw_ctx_id][buff_loc] = addr_pairs;

    /* Add the DATA FPGA registers to the scatterlist */
    sg = table->sgl;
    for ( i = 0; i < addr_pairs; i++ ) {
        sg_dma_address( sg ) = device_addr_pair[i].address;
        sg_dma_len( sg ) = device_addr_pair[i].size;
        sg = sg_next( sg );
    }
    LOG( LOG_INFO, "dma device setup success %d", system_dma_device->sg_device_nents[fw_ctx_id][buff_loc] );
    return 0;
}

int32_t system_dma_sg_fwmem_setup( void *ctx, int32_t buff_loc, fwmem_addr_pair_t *fwmem_pair, int32_t addr_pairs, uint32_t fw_ctx_id )
{//for FW_USE_HOBOT_DMA
    //unsigned int nr_pages;
    int i, ret;
    struct scatterlist *sg;
    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;

    if ( !system_dma_device || !fwmem_pair || !addr_pairs || buff_loc >= SYSTEM_DMA_TOGGLE_COUNT || fw_ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "null param problems" );
        return -1;
    }

    struct sg_table *table = &system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc];
    /* Allocate the scatterlist table */
    ret = sg_alloc_table( table, addr_pairs, GFP_KERNEL );
    if ( ret ) {
        LOG( LOG_CRIT, "unable to allocate DMA table\n" );
        return ret;
    }
    system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc] = addr_pairs;

#if (HOBOT_DMA_SRAM_PA == HOBOT_DMA_SRAM_DEBUG_DRAM_MODE)
    system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc] = kmalloc( sizeof( fwmem_addr_pair_t ) * addr_pairs, GFP_KERNEL );
    if ( !system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc] ) {
        LOG( LOG_CRIT, "Failed to allocate virtual address pairs for flushing!!" );
        return -1;
    }
    for ( i = 0; i < addr_pairs; i++ ) {
        system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc][i] = fwmem_pair[i];
    }
    sg = table->sgl;
    for ( i = 0; i < addr_pairs; i++ ) {
        sg_set_buf( sg, fwmem_pair[i].address, fwmem_pair[i].size );
        sg = sg_next( sg );
    }
#else
    /* Add the SRAM address to the scatterlist (sram area can not use sg_map to get physical addr) */
    sg = table->sgl;
    for ( i = 0; i < addr_pairs; i++ ) {
        sg_dma_address( sg ) = fwmem_pair[i].phy_addr;
        sg_dma_len( sg ) = (uint32_t)fwmem_pair[i].size;
        sg = sg_next( sg );
    }
#endif
    LOG( LOG_INFO, "fwmem setup success %d", system_dma_device->sg_device_nents[fw_ctx_id][buff_loc] );

    return 0;
}

void system_dma_unmap_sg( void *ctx )
{//for FW_USE_HOBOT_DMA
    return;
}

int32_t system_dma_copy_sg( void *ctx,
                                    int32_t buff_loc,
                                    uint32_t direction,
                                    dma_completion_callback complete_func,
                                    uint32_t fw_ctx_id)
{//for FW_USE_HOBOT_DMA
    int32_t result = 0;

    if ( !ctx ) {
        LOG( LOG_ERR, "Input ctx pointer is NULL" );
        return -1;
    }

    pr_debug("start\n");

    int32_t async_dma = 0;
    if ( complete_func != NULL ) {
        async_dma = 1;
    }

    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;
    struct scatterlist *isp_sram_sg, *dma_sram_sg;
    unsigned int isp_sram_nents, dma_sram_nents;

    isp_sram_sg    = system_dma_device->sg_device_table[fw_ctx_id][buff_loc].sgl;
    isp_sram_nents = system_dma_device->sg_device_nents[fw_ctx_id][buff_loc];
    dma_sram_sg    = system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc].sgl;
    dma_sram_nents = system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc];

    if ( isp_sram_nents != dma_sram_nents || !isp_sram_nents || !dma_sram_nents ) {
        LOG( LOG_ERR, "Unbalance isp_sram_nents:%d dma_sram_nents:%d", isp_sram_nents, dma_sram_nents );
        return -1;
    }

#if (HOBOT_DMA_SRAM_PA == HOBOT_DMA_SRAM_DEBUG_DRAM_MODE)
    int32_t i;
    // 1. flush memory before transfer (if use sram as fw_mem, no need to use cpu cache)
    for ( i = 0; i < isp_sram_nents; i++ ) {
        flush_icache_range( (unsigned long)( system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc][i].address ),
                            (unsigned long)( (uintptr_t)system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc][i].address +
                            system_dma_device->fwmem_pair_flush[fw_ctx_id][buff_loc][i].size ) );
    }
    // 2. map to sg table (if use sram as fw_mem, can not use dma_map_sg to get physical addr)
    result = dma_map_sg( NULL, system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc].sgl,
                         system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc], direction );
    if ( result <= 0 ) {
        LOG( LOG_CRIT, "unable to map %d", result );
        return -1;
    }
#endif

    system_dma_device->cur_fw_ctx_id = fw_ctx_id;
#if(HOBOT_REGISTER_MONITOR)
    hobot_rm_check_n_record(HRM_RC_DMA_START,buff_loc);
#endif

    /*
     * All buffers passed to this function should be ready and mapped
     * for DMA already. Therefore, we don't need to do anything except
     * submit it to the DMA Engine for processing
     */
    atomic_set( &system_dma_device->nents_done, 0 ); //set the number of nents done
    if ( async_dma == 0 ) {
        system_dma_device->complete_func = NULL; //async mode is not allowed to have callback
        init_completion( &system_dma_device->comp );
    } else {
        system_dma_device->complete_func = complete_func; //call this function if all nents are done;
    }
    system_dma_device->direction = direction;
    system_dma_device->buff_loc = buff_loc;

    unsigned long flags;
    idma_descriptor_t *desc = NULL;
    flags = system_spinlock_lock( g_hobot_dma.dma_ctrl_lock );
    if ( !list_empty(&g_hobot_dma.free_list) ) { 
        desc = list_first_entry(&g_hobot_dma.free_list, idma_descriptor_t, node);
        list_del(&desc->node);
    }
    if (!desc) {
        system_spinlock_unlock( g_hobot_dma.dma_ctrl_lock, flags );
        pr_err("idma free list empty.\n");
        return -1;
    }

    desc->ctx_id = fw_ctx_id;
    desc->isp_sram_sg = isp_sram_sg;
    desc->dma_sram_sg = dma_sram_sg;
    desc->isp_sram_nents = isp_sram_nents;
    desc->direction = (direction == SYS_DMA_TO_DEVICE) ? HOBOT_DMA_DIR_WRITE_ISP: HOBOT_DMA_DIR_READ_ISP;
    desc->callback.cb = dma_complete_func;
    desc->callback.cb_data = ctx;

    list_add_tail(&desc->node, &g_hobot_dma.pending_list);
    system_spinlock_unlock( g_hobot_dma.dma_ctrl_lock, flags );

    pr_debug("ctx id %d, isp_sram_nents %d, direction %d one node add to pending_list\n",
		desc->ctx_id, desc->isp_sram_nents, desc->direction);

    if ( async_dma == 0 ) {
        LOG( LOG_DEBUG, "scatterlist DMA waiting completion\n" );
//        wait_for_completion( &system_dma_device->comp );
#if (HOBOT_DMA_SRAM_PA == HOBOT_DMA_SRAM_DEBUG_DRAM_MODE)
        dma_unmap_sg( NULL, system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc].sgl, system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc], direction );
#endif
    }
#if(HOBOT_REGISTER_MONITOR)
    hobot_rm_check_n_record(HRM_RC_DMA_END,buff_loc);
#endif

    LOG( LOG_DEBUG, "scatterlist DMA success\n" );
    return result;
}

void system_dma_wait_done(void *ctx)
{
	system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;

	wait_for_completion( &system_dma_device->comp );
}

int32_t system_dma_copy_multi_sg( dma_cmd* cmd, uint32_t cmd_num)
{//for FW_USE_HOBOT_DMA
    int32_t i, result = 0;
    void *ctx = NULL;
    int32_t buff_loc = 0;
    uint32_t direction = 0;
    dma_completion_callback complete_func = NULL;
    unsigned int nents_cnt = 0;
    uint8_t last_cmd = 0;
    uint32_t fw_ctx_id = 0;

    for(i = 0;i < cmd_num;i++) {
        last_cmd = (i<(cmd_num-1))?0:1;
        ctx = cmd[i].ctx;
        buff_loc = cmd[i].buff_loc;
        direction = cmd[i].direction;
        complete_func = cmd[i].complete_func;
        fw_ctx_id = cmd[i].fw_ctx_id;

        if ( !ctx ) {
            LOG( LOG_ERR, "Input ctx pointer is NULL (cnt=%d)",i );
            return -1;
        }

        if ( complete_func == NULL ) {
            LOG( LOG_ERR, "complete_func is NULL (cnt=%d)",i );
            return -1;
        }

        system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;
        struct scatterlist *isp_sram_sg, *dma_sram_sg;
        unsigned int isp_sram_nents, dma_sram_nents;

        isp_sram_sg    = system_dma_device->sg_device_table[fw_ctx_id][buff_loc].sgl;
        isp_sram_nents = system_dma_device->sg_device_nents[fw_ctx_id][buff_loc];
        dma_sram_sg    = system_dma_device->sg_fwmem_table[fw_ctx_id][buff_loc].sgl;
        dma_sram_nents = system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc];

        if ( isp_sram_nents != dma_sram_nents || !isp_sram_nents || !dma_sram_nents ) {
            LOG( LOG_ERR, "Unbalance isp_sram_nents:%d dma_sram_nents:%d (cnt=%d)", isp_sram_nents, dma_sram_nents, i );
            return -1;
        }
        nents_cnt += isp_sram_nents;
        if (nents_cnt > HOBOT_DMA_MAX_CMD) {
            LOG( LOG_CRIT, "command number (%d) over dma hw limitation (%d)", nents_cnt, HOBOT_DMA_MAX_CMD );
            return -1;
        }

        system_dma_device->cur_fw_ctx_id = fw_ctx_id;
#if(HOBOT_REGISTER_MONITOR)
        hobot_rm_check_n_record(HRM_RC_DMA_START,buff_loc);
#endif

        /*
         * All buffers passed to this function should be ready and mapped
         * for DMA already. Therefore, we don't need to do anything except
         * submit it to the DMA Engine for processing
         */
        atomic_set( &system_dma_device->nents_done, 0 ); //set the number of nents done
        system_dma_device->complete_func = complete_func; //call this function if all nents are done;
        system_dma_device->direction = direction;
        system_dma_device->buff_loc = buff_loc;

    unsigned long flags;
    idma_descriptor_t *desc = NULL;
    flags = system_spinlock_lock( g_hobot_dma.dma_ctrl_lock );
    if ( !list_empty(&g_hobot_dma.free_list) ) {
        desc = list_first_entry(&g_hobot_dma.free_list, idma_descriptor_t, node);
        list_del(&desc->node);
    }
    if (!desc) {
        system_spinlock_unlock( g_hobot_dma.dma_ctrl_lock, flags );
        pr_err("idma free list empty.\n");
        return -1;
    }
	desc->ctx_id = fw_ctx_id;
	desc->isp_sram_sg = isp_sram_sg;
	desc->dma_sram_sg = dma_sram_sg;
	desc->isp_sram_nents = isp_sram_nents;
	desc->direction = (direction == SYS_DMA_TO_DEVICE) ? HOBOT_DMA_DIR_WRITE_ISP: HOBOT_DMA_DIR_READ_ISP;
	desc->callback.cb = dma_complete_func;
	desc->callback.cb_data = ctx;

	if (last_cmd)
		list_add_tail(&desc->node, &g_hobot_dma.pending_list);
	else
		list_add_tail(&desc->node, &g_hobot_dma.active_list);
	hobot_dma_submit_cmd(&g_hobot_dma, desc, last_cmd);
	system_spinlock_unlock( g_hobot_dma.dma_ctrl_lock, flags );

#if(HOBOT_REGISTER_MONITOR)
        hobot_rm_check_n_record(HRM_RC_DMA_END,buff_loc);
#endif
        LOG( LOG_DEBUG, "scatterlist DMA success (cnt=%d, last_cmd=%d)\n", i, last_cmd );
    }
    return result;
}

#else //#if FW_USE_SYSTEM_DMA

int32_t system_dma_sg_device_setup( void *ctx, int32_t buff_loc, dma_addr_pair_t *device_addr_pair, int32_t addr_pairs, uint32_t fw_ctx_id )
{
    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;
    int i;

    if ( !system_dma_device || !device_addr_pair || !addr_pairs || buff_loc >= SYSTEM_DMA_TOGGLE_COUNT || addr_pairs > SYSTEM_DMA_MAX_CHANNEL || fw_ctx_id >= FIRMWARE_CONTEXT_NUMBER )
        return -1;

    system_dma_device->sg_device_nents[fw_ctx_id][buff_loc] = addr_pairs;
    if ( !system_dma_device->mem_addrs[fw_ctx_id][buff_loc] )
        system_dma_device->mem_addrs[fw_ctx_id][buff_loc] = kmalloc( sizeof( mem_addr_pair_t ) * SYSTEM_DMA_MAX_CHANNEL, GFP_KERNEL );

    if ( !system_dma_device->mem_addrs[fw_ctx_id][buff_loc] ) {
        LOG( LOG_CRIT, "Failed to allocate virtual address pairs for flushing!!" );
        return -1;
    }
    for ( i = 0; i < addr_pairs; i++ ) {
        system_dma_device->mem_addrs[fw_ctx_id][buff_loc][i].dev_addr = ioremap( device_addr_pair[i].address, device_addr_pair[i].size );
        system_dma_device->mem_addrs[fw_ctx_id][buff_loc][i].size = device_addr_pair[i].size;
        system_dma_device->mem_addrs[fw_ctx_id][buff_loc][i].sys_back_ptr = ctx;
    }

    LOG( LOG_INFO, "dma device setup success %d", system_dma_device->sg_device_nents[fw_ctx_id][buff_loc] );
    return 0;
}

int32_t system_dma_sg_fwmem_setup( void *ctx, int32_t buff_loc, fwmem_addr_pair_t *fwmem_pair, int32_t addr_pairs, uint32_t fw_ctx_id )
{
    int i;
    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;

    if ( !system_dma_device || !fwmem_pair || !addr_pairs || buff_loc >= SYSTEM_DMA_TOGGLE_COUNT || fw_ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_CRIT, "null param problems" );
        return -1;
    }
    system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc] = addr_pairs;

    if ( !system_dma_device->mem_addrs[fw_ctx_id][buff_loc] )
        system_dma_device->mem_addrs[fw_ctx_id][buff_loc] = kmalloc( sizeof( mem_addr_pair_t ) * SYSTEM_DMA_MAX_CHANNEL, GFP_KERNEL );

    if ( !system_dma_device->mem_addrs[fw_ctx_id][buff_loc] ) {
        LOG( LOG_CRIT, "Failed to allocate virtual address pairs for flushing!!" );
        return -1;
    }

    for ( i = 0; i < addr_pairs; i++ ) {
        system_dma_device->mem_addrs[fw_ctx_id][buff_loc][i].fw_addr = fwmem_pair[i].address;
        system_dma_device->mem_addrs[fw_ctx_id][buff_loc][i].size = fwmem_pair[i].size;
        system_dma_device->mem_addrs[fw_ctx_id][buff_loc][i].sys_back_ptr = ctx;
    }

    LOG( LOG_INFO, "fwmem setup success %d", system_dma_device->sg_device_nents[fw_ctx_id][buff_loc] );

    return 0;
}

static void memcopy_func( unsigned long p_task )
{
    mem_tasklet_t *mem_task = (mem_tasklet_t *)p_task;
    mem_addr_pair_t *mem_addr = (mem_addr_pair_t *)mem_task->mem_data;
    system_dma_device_t *system_dma_device = (system_dma_device_t *)mem_addr->sys_back_ptr;
    void *src_mem = 0;
    void *dst_mem = 0;

    int32_t buff_loc = system_dma_device->buff_loc;
    uint32_t direction = system_dma_device->direction;

    pr_debug("+\n");

    if ( direction == SYS_DMA_TO_DEVICE ) {
        src_mem = mem_addr->fw_addr;
        dst_mem = mem_addr->dev_addr;
    } else {
        dst_mem = mem_addr->fw_addr;
        src_mem = mem_addr->dev_addr;
    }

    memcpy( dst_mem, src_mem, mem_addr->size );

    LOG( LOG_DEBUG, "(%d:%d) d:%p s:%p l:%ld", buff_loc, direction, dst_mem, src_mem, mem_addr->size );

    dma_complete_func( mem_addr->sys_back_ptr );

    pr_debug("-\n");

    return;
}

void system_dma_unmap_sg( void *ctx )
{
    return;
}

int32_t system_dma_copy_sg( void *ctx, int32_t buff_loc, uint32_t direction, dma_completion_callback complete_func, uint32_t fw_ctx_id )
{
    int32_t i, result = 0;
    if ( !ctx ) {
        LOG( LOG_ERR, "Input ctx pointer is NULL" );
        return -1;
    }

    int32_t async_dma = 0;

    if ( complete_func != NULL ) {
        async_dma = 1;
    }

    system_dma_device_t *system_dma_device = (system_dma_device_t *)ctx;


    unsigned int src_nents = system_dma_device->sg_device_nents[fw_ctx_id][buff_loc];
    unsigned int dst_nents = system_dma_device->sg_fwmem_nents[fw_ctx_id][buff_loc];
    if ( src_nents != dst_nents || !src_nents || !dst_nents ) {
        LOG( LOG_CRIT, "Unbalance src_nents:%d dst_nents:%d", src_nents, dst_nents );
        return -1;
    }

#if(HOBOT_REGISTER_MONITOR)
    hobot_rm_check_n_record(HRM_RC_DMA_START,buff_loc);
#endif

    system_dma_device->cur_fw_ctx_id = fw_ctx_id;

    atomic_set( &system_dma_device->nents_done, 0 ); //set the number of nents done
    if ( async_dma == 0 ) {
        system_dma_device->complete_func = NULL; //async mode is not allowed to have callback
        init_completion( &system_dma_device->comp );
    } else {
        system_dma_device->complete_func = complete_func; //call this function if all nents are done;
    }
    system_dma_device->direction = direction;
    system_dma_device->buff_loc = buff_loc;


    for ( i = 0; i < SYSTEM_DMA_MAX_CHANNEL; i++ ) {
        system_dma_device->task_list[fw_ctx_id][buff_loc][i].mem_data = &( system_dma_device->mem_addrs[fw_ctx_id][buff_loc][i] );
        system_dma_device->task_list[fw_ctx_id][buff_loc][i].m_task.data = (unsigned long)&system_dma_device->task_list[fw_ctx_id][buff_loc][i];
        system_dma_device->task_list[fw_ctx_id][buff_loc][i].m_task.func = memcopy_func;
        tasklet_schedule( &system_dma_device->task_list[fw_ctx_id][buff_loc][i].m_task );
    }

    if ( async_dma == 0 ) {
        LOG( LOG_DEBUG, "scatterlist DMA waiting completion\n" );
        wait_for_completion( &system_dma_device->comp );
    }

#if(HOBOT_REGISTER_MONITOR)
    hobot_rm_check_n_record(HRM_RC_DMA_END,buff_loc);
#endif

    LOG( LOG_DEBUG, "scatterlist DMA success\n" );
    return result;
}

#endif //#if FW_USE_SYSTEM_DMA
