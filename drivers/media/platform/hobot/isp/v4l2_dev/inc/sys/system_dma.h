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

#ifndef __SYSTEM_DMA_H__
#define __SYSTEM_DMA_H__

#include "acamera_types.h"
#include "system_stdlib.h"

#define SYSTEM_DMA_TOGGLE_COUNT 2
#define SYSTEM_DMA_MAX_CHANNEL 3

#define SYS_DMA_TO_DEVICE 0
#define SYS_DMA_FROM_DEVICE 1

typedef struct {
    uint32_t address;
    uint32_t size;
} dma_addr_pair_t;

typedef struct {
    void *address;
    size_t size;
    uint32_t phy_addr;
} fwmem_addr_pair_t;

typedef void ( *dma_completion_callback )( void * );

typedef struct {
    void *ctx;
    int32_t buff_loc;
    uint32_t direction;
    dma_completion_callback complete_func;
    uint32_t fw_ctx_id;
} dma_cmd;

#if FW_USE_SYSTEM_DMA
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <asm/dma-mapping.h>
#include <asm/cacheflush.h>

typedef struct {
    //in case scatter and gather is not supported, need more than one channel
    struct dma_chan *dma_channel[SYSTEM_DMA_MAX_CHANNEL];
    //for sg
    struct sg_table sg_device_table[FIRMWARE_CONTEXT_NUMBER][SYSTEM_DMA_TOGGLE_COUNT];
    unsigned int sg_device_nents[FIRMWARE_CONTEXT_NUMBER][SYSTEM_DMA_TOGGLE_COUNT];

    struct sg_table sg_fwmem_table[FIRMWARE_CONTEXT_NUMBER][SYSTEM_DMA_TOGGLE_COUNT];
    unsigned int sg_fwmem_nents[FIRMWARE_CONTEXT_NUMBER][SYSTEM_DMA_TOGGLE_COUNT];

    //for flushing
    fwmem_addr_pair_t *fwmem_pair_flush[FIRMWARE_CONTEXT_NUMBER][SYSTEM_DMA_TOGGLE_COUNT];
    //for callback unmapping
    int32_t buff_loc;
    uint32_t direction;
    uint32_t cur_fw_ctx_id;

    //conf synchronization and completion
    dma_completion_callback complete_func;
    atomic_t nents_done;
    struct completion comp;

} system_dma_device_t;

#elif FW_USE_HOBOT_DMA
#include "hobot_isp_reg_dma.h"
#else
#include <linux/interrupt.h>

typedef struct {
    void *dev_addr;
    void *fw_addr;
    size_t size;
    void *sys_back_ptr;
} mem_addr_pair_t;

typedef struct {
    struct tasklet_struct m_task;
    mem_addr_pair_t *mem_data;
} mem_tasklet_t;


typedef struct {
    char *name;
    unsigned int sg_device_nents[FIRMWARE_CONTEXT_NUMBER][SYSTEM_DMA_TOGGLE_COUNT];

    unsigned int sg_fwmem_nents[FIRMWARE_CONTEXT_NUMBER][SYSTEM_DMA_TOGGLE_COUNT];

    mem_addr_pair_t *mem_addrs[FIRMWARE_CONTEXT_NUMBER][SYSTEM_DMA_TOGGLE_COUNT];

    mem_tasklet_t task_list[FIRMWARE_CONTEXT_NUMBER][SYSTEM_DMA_TOGGLE_COUNT][SYSTEM_DMA_MAX_CHANNEL];

    int32_t buff_loc;
    uint32_t direction;
    uint32_t cur_fw_ctx_id;

    //conf synchronization and completion
    dma_completion_callback complete_func;
    atomic_t nents_done;
    struct completion comp;

} system_dma_device_t;

#endif

/**
 *   Initialize dma channel
 *
 *   This function requests an exclusive dma channel from the system and return a pointer to it
 *
 *   @param ctx - pointer to dma channel or NULL if error.
 *
 *   @return 0 - on success or -1 on error
 */
int32_t system_dma_init( void **ctx );


/**
 *   Destroy dma channel
 *
 *   This function destroys previously allocated by system_dma_init channel.
 *
 *   @param ctx - pointer to dma channel.
 *
 *   @return 0 - on success or -1 on error
 */
int32_t system_dma_destroy( void *ctx );


/**
 *   Copy data from memory to the device asynchronously
 *
 *   This function copies memory allocated by kmalloc to the device physical memory.
 *   Please note that dst_mem MUST be a kernel virtual pointer.
 *
 *   @param ctx - pointer to dma channel.
 *   @param dst_mem - virtual pointer to the memory region
 *   @param dev_phy_addr - device physical address
 *   @param size_to_copy - data size to copy in bytes
 *
 *   @return 0 - on success or -1 on error
 */
int32_t system_dma_copy_device_to_memory_async( void *ctx, void *dst_mem, uint32_t dev_phy_addr, uint32_t size_to_copy, dma_completion_callback complete_func, void *args );


/**
 *   Copy data from dev to memory asynchronously
 *
 *   This function copies device physical memory to memory allocated by kmalloc.
 *   Please note that src_mem MUST be a kernel virtual pointer.
 *
 *   @param ctx - pointer to dma channel.
 *   @param src_mem - virtual pointer to the memory region
 *   @param dev_phy_addr - device physical address
 *   @param size_to_copy - data size to copy in bytes
 *
 *   @return 0 - on success or -1 on error
 */
int32_t system_dma_copy_memory_to_device_async( void *ctx, void *src_mem, uint32_t dev_phy_addr, uint32_t size_to_copy, dma_completion_callback complete_func, void *args );


/**
 *   Copy data from memory to the device
 *
 *   This function copies memory allocated by kmalloc to the device physical memory.
 *   Please note that dst_mem MUST be a kernel virtual pointer.
 *
 *   @param ctx - pointer to dma channel.
 *   @param dst_mem - virtual pointer to the memory region
 *   @param dev_phy_addr - device physical address
 *   @param size_to_copy - data size to copy in bytes
 *
 *   @return 0 - on success or -1 on error
 */
int32_t system_dma_copy_device_to_memory( void *ctx, void *dst_mem, uint32_t dev_phy_addr, uint32_t size_to_copy );


/**
 *   Copy data from dev to memory
 *
 *   This function copies device physical memory to memory allocated by kmalloc.
 *   Please note that src_mem MUST be a kernel virtual pointer.
 *
 *   @param ctx - pointer to dma channel.
 *   @param src_mem - virtual pointer to the memory region
 *   @param dev_phy_addr - device physical address
 *   @param size_to_copy - data size to copy in bytes
 *
 *   @return 0 - on success or -1 on error
 */
int32_t system_dma_copy_memory_to_device( void *ctx, void *src_mem, uint32_t dev_phy_addr, uint32_t size_to_copy );


/**
 *   Setup firmware memory for scatter and gather dma feature from fw memory pairs of virtual address and lenght
 *
 *
 *   @param ctx - pointer to dma channel data or NULL if error.
 *
 *   @return 0 - on success or -1 on error
 */
int32_t system_dma_sg_fwmem_setup( void *ctx, int32_t buff_loc, fwmem_addr_pair_t *fwmem_pair, int32_t addr_pairs, uint32_t fw_ctx_id );

/**
 *   Setup isp device memory for scatter and gather dma feature from pairs of dma bus address and lenght
 *
 *   @param ctx - pointer to dma channel data or NULL if error.
 *   @param buff_loc - points to ping or pong buffer.
 *   @param fwmem_pair - fw memory pairs of virtual address and lenght.
 *
 *   @return 0 - on success or -1 on error
 */
int32_t system_dma_sg_device_setup( void *ctx, int32_t buff_loc, dma_addr_pair_t *device_addr_pair, int32_t addr_pairs, uint32_t fw_ctx_id );

/**
 *   Needed to unmap the virtual address after dma completes
 *
 *   @param ctx - pointer to dma channel data or NULL if error.
 *   @param buff_loc - points to ping or pong buffer.
 *   @param device_addr_pair - pairs of dma bus address and lenght.
 *
 *   @return 0 - on success or -1 on error
 */
void system_dma_unmap_sg( void *ctx );

/**
 *   Setup dma and run it
 *
 *   @param ctx - pointer to dma channel data or NULL if error.
 *   @param buff_loc - points to ping or pong buffer.
 *   @param direction - to the device or from the device.
 *   @param complete_func - function to be called after dma; ctx will be passed to this function for system_dma_destroy
 *
 *   @return 0 - on success or -1 on error
 */
int32_t system_dma_copy_sg( void *ctx, int32_t buff_loc, uint32_t direction, dma_completion_callback complete_func, uint32_t fw_ctx_id );

#if FW_USE_HOBOT_DMA
int32_t system_dma_copy_multi_sg( dma_cmd* cmd,
                                            uint32_t cmd_num);
void system_dma_wait_done(void *ctx);

void system_dma_desc_flush(void);
#endif

#endif // __SYSTEM_DMA_H__
