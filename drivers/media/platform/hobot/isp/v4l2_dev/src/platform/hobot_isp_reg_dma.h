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
#ifndef _HOBOT_ISP_REG_DMA_H_
#define _HOBOT_ISP_REG_DMA_H_

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <asm/dma-mapping.h>
#include <asm/cacheflush.h>
#include "acamera_types.h"
#include "acamera_logger.h"
#include "system_spinlock.h"
#include "hobot_isp_reg_dma_regset.h"

#if FW_USE_HOBOT_DMA

#define HOBOT_DMA_IRQ_INDEX        1
#define HOBOT_DMA_IRQ_NAME         "hobot dma interrupt for irq"
#define HOBOT_DMA_DIR_WRITE_ISP   0
#define HOBOT_DMA_DIR_READ_ISP    1
#define HOBOT_DMA_MAX_CMD         4
#define HOBOT_DMA_MAX_CALLBACK    HOBOT_DMA_MAX_CMD

typedef struct hobot_dma_callback_s {
    dma_completion_callback cb;
    void *cb_data;
}hobot_dma_callback_t;

typedef struct hobot_dma_cmd_s {
    uint32_t dma_sram_addr;
    uint32_t isp_sram_addr;
    uint32_t size;
    uint32_t direction;
}hobot_dma_cmd_t;

typedef struct hobot_dma_s {
    uint32_t  init_cnt;
    uint32_t  enable_irq_cnt;
    uint32_t irq_in_dts;
    uint32_t is_busy;

    hobot_dma_cmd_t hobot_dma_cmds[HOBOT_DMA_MAX_CMD];
    uint32_t nents_total;

    sys_spinlock dma_ctrl_lock;

    struct tasklet_struct tasklet;
    struct list_head pending_list;
    struct list_head active_list;
    struct list_head done_list;
}hobot_dma_t;

typedef struct {
	struct list_head node;
	u32 ctx_id;
	u32 direction;
	u32 isp_sram_nents;
	u32 dma_sram_nents;
	struct scatterlist *isp_sram_sg;
	struct scatterlist *dma_sram_sg;
	hobot_dma_callback_t callback;
} idma_descriptor_t;

typedef struct {
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
//		hobot_dma_t hobot_dma;
} system_dma_device_t;

void hobot_dma_init(hobot_dma_t *hobot_dma);
void hobot_dma_deinit(hobot_dma_t *hobot_dma);
void hobot_dma_enable_irq(hobot_dma_t *hobot_dma);
void hobot_dma_disable_irq(hobot_dma_t *hobot_dma);
void hobot_dma_submit_cmd(hobot_dma_t *hobot_dma, idma_descriptor_t *desc, int last_cmd);
void isp_idma_start_transfer(hobot_dma_t *hobot_dma);
#endif      /* FW_USE_HOBOT_DMA */

#endif      /*_HOBOT_ISP_REG_DMA_H_ */


