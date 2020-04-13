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

#include <linux/module.h>
#include <linux/semaphore.h>
#include <linux/crc16.h>
#include "acamera_fw.h"
#include "acamera_firmware_api.h"
#include "acamera_firmware_config.h"
#include "acamera_command_api.h"
#include "acamera_calibrations.h"
#include "acamera.h"
#include "acamera_math.h"
#include "acamera_isp_config.h"
#include "acamera_isp_core_nomem_settings.h"
#include "system_stdlib.h"
#include "system_dma.h"
#include "acamera_metering_stats_mem_config.h"
#include "acamera_aexp_hist_stats_mem_config.h"
#include "acamera_decompander0_mem_config.h"
#include "acamera_ihist_stats_mem_config.h"

#if FW_HAS_CONTROL_CHANNEL
#include "acamera_ctrl_channel.h"
#endif
#include "application_command_api.h"
#include "runtime_initialization_settings.h"

#if ISP_DMA_RAW_CAPTURE
#include "dma_raw_capture.h"
#endif

#if ACAMERA_ISP_PROFILING
#include "acamera_profiler.h"
#endif

#if ISP_HAS_DMA_INPUT
#include "fpga_dma_input.h"
#include "acamera_fpga_fe_isp_config.h"
#endif


#if ACAMERA_ISP_PROFILING
#ifndef CHECK_STACK_SIZE_DWORDS
#define CHECK_STACK_SIZE_DWORDS 1024
#endif
#define CHECK_STACK_SIZE_START 512
#endif

#include "sensor_fsm.h"
#include "acamera_logger.h"
#include "fsm_param.h"
#include "isp_ctxsv.h"
#include "hobot_isp_reg_dma.h"

static acamera_firmware_t g_firmware;

typedef int (*isp_callback)(int);
extern void isp_register_callback(isp_callback func);
int sif_isp_ctx_sync_func(int ctx_id);
static DECLARE_WAIT_QUEUE_HEAD(wq);
static DECLARE_WAIT_QUEUE_HEAD(wq_fe);

#if FW_USE_HOBOT_DMA
extern hobot_dma_t g_hobot_dma;
#endif

int32_t acamera_set_api_context( uint32_t ctx_num )
{
    int32_t result = 0;
    if ( ctx_num < g_firmware.context_number ) {
        g_firmware.api_context = ctx_num;
        LOG( LOG_INFO, "new api context: %u.", (unsigned int)g_firmware.api_context );
        result = 0;
    } else {
        result = -1;
    }
    return result;
}

uint32_t acamera_get_api_context( void )
{
    uint32_t result = g_firmware.api_context;
    return result;
}

int32_t acamera_get_context_number( void )
{
    int32_t result = g_firmware.context_number;
    return result;
}


void *acamera_get_api_ctx_ptr( void )
{
    return &( g_firmware.fw_ctx[acamera_get_api_context()] );
}

void *acamera_get_ctx_ptr( uint32_t ctx_id )
{
    if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_CRIT, "Error: Invalid ctx_id: %d", ctx_id );
        return NULL;
    }

    return &( g_firmware.fw_ctx[ctx_id] );
}

void acamera_notify_evt_data_avail( void )
{
    system_semaphore_raise( g_firmware.sem_evt_avail );
}

static int32_t validate_settings( acamera_settings *settings, uint32_t ctx_num )
{

    int32_t result = 0;
    int32_t idx = 0;

    if ( settings != NULL ) {
        // validate parameters for all input contexts
            acamera_settings *ctx_set = &settings[ctx_num];
            if ( ctx_set->sensor_init == NULL ||
                 ctx_set->sensor_deinit == NULL ||
                 ctx_set->get_calibrations == NULL ) {
                result = -1;
                LOG( LOG_CRIT, "One of the mandatory input parameters in the init settings is null for the context %d", idx );
            }
#if ISP_HAS_DS1 && defined( ISP_HAS_DMA_WRITER_FSM )
            /*
             * TODO find a better place to check this
             */
            if ( acamera_isp_isp_global_parameter_status_ds1_pipe_read( ctx_set->isp_base ) ) { //check if DS is chickened out
                result = -1;
                LOG( LOG_CRIT, "DS Frames are expected while DS block is not present in hardware for the context %d", idx );
            }
#endif
    } else {
        result = -1;
    }
    return result;
}

static int32_t dma_channel_addresses_setup( void *isp_chan, void *metering_chan, void *sw_context_map,
                                                         uint32_t idx, uint32_t hw_isp_addr, uint32_t sw_isp_phy_addr )
{
    int32_t result = 0;
    //PING ISP
    {
        dma_addr_pair_t isp_dma_addr_pair[2] = {
            {hw_isp_addr + ACAMERA_DECOMPANDER0_MEM_BASE_ADDR, ACAMERA_ISP1_BASE_ADDR - ACAMERA_DECOMPANDER0_MEM_BASE_ADDR},
            {hw_isp_addr + ACAMERA_ISP1_BASE_ADDR, ACAMERA_ISP1_SIZE}};
        if ( system_dma_sg_device_setup( isp_chan, ISP_CONFIG_PING, isp_dma_addr_pair, 2, idx ) ) {
            LOG( LOG_CRIT, "ISP device channel address setup for PING ctx:%d failed", idx );
            result = -1;
        }
        fwmem_addr_pair_t fwmem_add_pair[2] = {
            {(void *)( (uintptr_t)sw_context_map + ACAMERA_DECOMPANDER0_MEM_BASE_ADDR ),
            ACAMERA_ISP1_BASE_ADDR - ACAMERA_DECOMPANDER0_MEM_BASE_ADDR,
            sw_isp_phy_addr + ACAMERA_DECOMPANDER0_MEM_BASE_ADDR},
            {(void *)( (uintptr_t)sw_context_map + ACAMERA_ISP1_BASE_ADDR ),
            ACAMERA_ISP1_SIZE,
            sw_isp_phy_addr + ACAMERA_ISP1_BASE_ADDR   }};
        if ( system_dma_sg_fwmem_setup( isp_chan, ISP_CONFIG_PING, fwmem_add_pair, 2, idx ) ) {
            LOG( LOG_CRIT, "ISP memory channel address setup for PING ctx:%d failed", idx );
            result = -1;
        }
    }
    //PONG ISP
    {
        dma_addr_pair_t isp_dma_addr_pair[2] = {
            {hw_isp_addr + ACAMERA_DECOMPANDER0_MEM_BASE_ADDR + ISP_CONFIG_PING_SIZE, ACAMERA_ISP1_BASE_ADDR - ACAMERA_DECOMPANDER0_MEM_BASE_ADDR},
            {hw_isp_addr + ACAMERA_ISP1_BASE_ADDR + ISP_CONFIG_PING_SIZE, ACAMERA_ISP1_SIZE}};
        if ( system_dma_sg_device_setup( isp_chan, ISP_CONFIG_PONG, isp_dma_addr_pair, 2, idx ) ) {
            LOG( LOG_CRIT, "ISP device channel address setup for PONG ctx:%d failed", idx );
            result = -1;
        }

        fwmem_addr_pair_t fwmem_add_pair[2] = {
            {(void *)( (uintptr_t)sw_context_map + ACAMERA_DECOMPANDER0_MEM_BASE_ADDR ),
            ACAMERA_ISP1_BASE_ADDR - ACAMERA_DECOMPANDER0_MEM_BASE_ADDR,
            sw_isp_phy_addr + ACAMERA_DECOMPANDER0_MEM_BASE_ADDR},
            {(void *)( (uintptr_t)sw_context_map + ACAMERA_ISP1_BASE_ADDR ),
            ACAMERA_ISP1_SIZE,
            sw_isp_phy_addr + ACAMERA_ISP1_BASE_ADDR}};
        if ( system_dma_sg_fwmem_setup( isp_chan, ISP_CONFIG_PONG, fwmem_add_pair, 2, idx ) ) {
            LOG( LOG_CRIT, "ISP memory channel address setup for PONG ctx:%d failed", idx );
            result = -1;
        }
    }
    //PING METERING
    {
        dma_addr_pair_t isp_dma_addr_pair[2] = {
            {hw_isp_addr + ACAMERA_AEXP_HIST_STATS_MEM_BASE_ADDR, ACAMERA_AEXP_HIST_STATS_MEM_SIZE + ACAMERA_IHIST_STATS_MEM_SIZE},
            {hw_isp_addr + ACAMERA_METERING_STATS_MEM_BASE_ADDR, ACAMERA_METERING_STATS_MEM_SIZE}};
        if ( system_dma_sg_device_setup( metering_chan, ISP_CONFIG_PING, isp_dma_addr_pair, 2, idx ) ) {
            LOG( LOG_CRIT, "Metering device channel address setup for PING ctx:%d failed", idx );
            result = -1;
        }

        fwmem_addr_pair_t fwmem_add_pair[2] = {
            {(void *)( (uintptr_t)sw_context_map + ACAMERA_AEXP_HIST_STATS_MEM_BASE_ADDR ),
            ACAMERA_AEXP_HIST_STATS_MEM_SIZE + ACAMERA_IHIST_STATS_MEM_SIZE,
            sw_isp_phy_addr + ACAMERA_AEXP_HIST_STATS_MEM_BASE_ADDR},
            {(void *)( (uintptr_t)sw_context_map + ACAMERA_METERING_STATS_MEM_BASE_ADDR ),
            ACAMERA_METERING_STATS_MEM_SIZE,
            sw_isp_phy_addr + ACAMERA_METERING_STATS_MEM_BASE_ADDR}};
        if ( system_dma_sg_fwmem_setup( metering_chan, ISP_CONFIG_PING, fwmem_add_pair, 2, idx ) ) {
            LOG( LOG_CRIT, "Metering memory channel address setup for PING ctx:%d failed", idx );
            result = -1;
        }
    }
    //PONG METERING
    {
        dma_addr_pair_t isp_dma_addr_pair[2] = {
            {hw_isp_addr + ACAMERA_AEXP_HIST_STATS_MEM_BASE_ADDR, ACAMERA_AEXP_HIST_STATS_MEM_SIZE + ACAMERA_IHIST_STATS_MEM_SIZE},
            {hw_isp_addr + ACAMERA_METERING_STATS_MEM_BASE_ADDR + ISP_CONFIG_PING_SIZE, ACAMERA_METERING_STATS_MEM_SIZE}};
        if ( system_dma_sg_device_setup( metering_chan, ISP_CONFIG_PONG, isp_dma_addr_pair, 2, idx ) ) {
            LOG( LOG_CRIT, "Metering device channel address setup for PONG ctx:%d failed", idx );
            result = -1;
        }

        fwmem_addr_pair_t fwmem_add_pair[2] = {
            {(void *)( (uintptr_t)sw_context_map + ACAMERA_AEXP_HIST_STATS_MEM_BASE_ADDR ),
            ACAMERA_AEXP_HIST_STATS_MEM_SIZE + ACAMERA_IHIST_STATS_MEM_SIZE,
            sw_isp_phy_addr + ACAMERA_AEXP_HIST_STATS_MEM_BASE_ADDR},
            {(void *)( (uintptr_t)sw_context_map + ACAMERA_METERING_STATS_MEM_BASE_ADDR ),
            ACAMERA_METERING_STATS_MEM_SIZE,
            sw_isp_phy_addr + ACAMERA_METERING_STATS_MEM_BASE_ADDR}};
        if ( system_dma_sg_fwmem_setup( metering_chan, ISP_CONFIG_PONG, fwmem_add_pair, 2, idx ) ) {
            LOG( LOG_CRIT, "Metering memory channel address setup for PONG  ctx:%d failed", idx );
            result = -1;
        }
    }
    return result;
}

#if USER_MODULE

int32_t acamera_init( acamera_settings *settings, uint32_t ctx_num )
{
    int32_t result = 0;
    uint32_t idx = 0;

    result = validate_settings( settings, ctx_num );

    if ( result == 0 ) {

#if FW_HAS_CONTROL_CHANNEL
        ctrl_channel_init();
#endif

        g_firmware.api_context = 0;

        system_semaphore_init( &g_firmware.sem_evt_avail );

        if ( ctx_num <= FIRMWARE_CONTEXT_NUMBER ) {

            g_firmware.context_number = ctx_num;

            for ( idx = 0; idx < ctx_num; idx++ ) {
                acamera_context_t *p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[idx];
                LOG( LOG_INFO, "Initialize context %d, context size is %d bytes", idx, sizeof( struct _acamera_context_t ) );
                system_memset( (void *)p_ctx, 0x0, sizeof( struct _acamera_context_t ) );
                // each context has unique id
                p_ctx->context_id = idx;

                // init context
                result = acamera_init_context( p_ctx, &settings[idx], &g_firmware );
                if ( result != 0 ) {
                    LOG( LOG_CRIT, "Failed to initialized the context %d", (int)idx );
                    break;
                }
            }

            if ( result == 0 ) {
                g_firmware.initialized = 1;
            } else {
                g_firmware.initialized = 0;
            }

        } else {
            result = -1;
            LOG( LOG_CRIT, "Failed to initialized the firmware context. Not enough memory. " );
        }

    } else {
        LOG( LOG_CRIT, "Input initializations settings are not correct" );
        result = -1;
    }

    return result;
}

void acamera_update_cur_settings_to_isp( uint32_t fw_ctx_id )
{
}

#else /* #if USER_MODULE */

int acamera_isp_init_context(uint8_t idx)
{
    int i = 0;
    int ret = 0;
    acamera_context_t *p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[idx];

    pr_info("+\n");
    ret = validate_settings( settings, idx );
    if (ret < 0) {
        pr_err("settings[%d] validate not correct\n", idx);
        return ret;
    }

    for (i = 0; g_firmware.initialized == 0 && i < FIRMWARE_CONTEXT_NUMBER; i++) {
        pr_info("copy data from isp hw to sram %d\n", i);
        system_dma_copy_sg( g_firmware.dma_chan_isp_config, ISP_CONFIG_PING, SYS_DMA_FROM_DEVICE, 0, i );
#if FW_USE_HOBOT_DMA
        isp_idma_start_transfer(&g_hobot_dma);
        system_dma_wait_done(g_firmware.dma_chan_isp_config);
#endif
    }

    ret = acamera_init_context( p_ctx, &settings[idx], &g_firmware );
    if (ret == 0) {

        if (g_firmware.initialized != 0) {
            pr_debug("ping/pong have been inited\n");
            goto out;
        }

        // system_dma_copy current software context to the ping and pong
        system_dma_copy_sg( g_firmware.dma_chan_isp_config, ISP_CONFIG_PING, SYS_DMA_TO_DEVICE, 0, idx );
#if FW_USE_HOBOT_DMA
        isp_idma_start_transfer(&g_hobot_dma);
        system_dma_wait_done(g_firmware.dma_chan_isp_config);
#endif
        system_dma_copy_sg( g_firmware.dma_chan_isp_config, ISP_CONFIG_PONG, SYS_DMA_TO_DEVICE, 0, idx );
#if FW_USE_HOBOT_DMA
        isp_idma_start_transfer(&g_hobot_dma);
        system_dma_wait_done(g_firmware.dma_chan_isp_config);
#endif
        g_firmware.initialized = 1;

        acamera_isp_isp_global_mcu_override_config_select_write( 0, 1 ); //put ping pong in slave mode
        g_firmware.dma_flag_isp_config_completed = 1;
        g_firmware.dma_flag_isp_metering_completed = 1;

        // active context, for acamera_get_api_context calling
        acamera_command( idx, TGENERAL, ACTIVE_CONTEXT, idx, COMMAND_SET, &ret );
        pr_info("context-%d copy to isp ping/pong\n", idx);
    } else {
        g_firmware.initialized = 0;
        pr_err("acamera context init failed, ret = %d\n", ret);
    }

out:
    pr_info("-\n");

    return ret;
}

int acamera_isp_firmware_clear(void)
{
    g_firmware.initialized = 0;
    g_firmware.dma_flag_isp_config_completed = 0;
    g_firmware.dma_flag_isp_metering_completed = 0;   

    return 0;
}

int32_t acamera_init( acamera_settings *settings, uint32_t ctx_num )
{
    int32_t result = 0;
    uint32_t idx;

    // disable irq and clear interrupts
    acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_DISABLE_ALL_IRQ );

#if ACAMERA_ISP_PROFILING && ACAMERA_ISP_PROFILING_INIT
    acamera_profiler_init();
    acamera_profiler_start( 0 );
#endif

    g_firmware.api_context = 0;

    system_semaphore_init(&g_firmware.sem_event_process_done);
    system_semaphore_init( &g_firmware.sem_evt_avail );

    if ( ctx_num <= FIRMWARE_CONTEXT_NUMBER ) {

        g_firmware.context_number = ctx_num;

        result = system_dma_init( &g_firmware.dma_chan_isp_config );
        result |= system_dma_init( &g_firmware.dma_chan_isp_metering );
        if ( result == 0 ) {
            LOG( LOG_INFO, "DMA Channels allocated 0x%x and 0x%x", g_firmware.dma_chan_isp_config, g_firmware.dma_chan_isp_metering );
            // allocate memory for dma transfers
            for ( idx = 0; idx < ctx_num; idx++ ) {
                acamera_context_t *p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[idx];
                LOG( LOG_INFO, "Initialize context %d, context size is %d bytes", idx, sizeof( struct _acamera_context_t ) );
                system_memset( (void *)p_ctx, 0x0, sizeof( struct _acamera_context_t ) );
                // each context has unique id
                p_ctx->context_id = idx;

                // dump hw default configuration to the current context

#if FW_USE_HOBOT_DMA
                p_ctx->sw_reg_map.isp_sw_config_map = system_sw_alloc_dma_sram(HOBOT_DMA_SRAM_ONE_ZONE,
                                                                                p_ctx->context_id,
                                                                                &p_ctx->sw_reg_map.isp_sw_phy_addr);
#else
                p_ctx->sw_reg_map.isp_sw_config_map = system_sw_alloc(HOBOT_DMA_SRAM_SIZE);
                p_ctx->sw_reg_map.isp_sw_phy_addr = 0;  // no use when DMA mode disable
#endif
                if ( p_ctx->sw_reg_map.isp_sw_config_map ) {
                    result = dma_channel_addresses_setup( g_firmware.dma_chan_isp_config, g_firmware.dma_chan_isp_metering,
                                        (void *)p_ctx->sw_reg_map.isp_sw_config_map, idx, settings->hw_isp_addr,
                                        p_ctx->sw_reg_map.isp_sw_phy_addr);
                } else {
                    LOG( LOG_CRIT, "Software Context %d failed to allocate", idx );
                    result = -1;
                }

#if FW_HAS_CONTROL_CHANNEL
                ctrl_channel_init(idx);
#endif
                if ( result )
                    break;
            }
        } else {
            result = -1;
            LOG( LOG_CRIT, "Failed to initialize the system DMA engines" );
        }
    } else {
        result = -1;
        LOG( LOG_CRIT, "Failed to initialized the firmware context. Not enough memory. " );
    }

#if ACAMERA_ISP_PROFILING && ACAMERA_ISP_PROFILING_INIT
    acamera_profiler_stop( 0, 1 );
    acamera_profiler_report();
#endif


#if ISP_HAS_DMA_INPUT
    fpga_dma_input_init( &g_firmware );
#endif

    isp_register_callback(sif_isp_ctx_sync_func);

    return result;
}

void acamera_update_cur_settings_to_isp( uint32_t fw_ctx_id )
{
    system_dma_copy_sg( g_firmware.dma_chan_isp_config, ISP_CONFIG_PING, SYS_DMA_TO_DEVICE, NULL, fw_ctx_id );
#if FW_USE_HOBOT_DMA
    isp_idma_start_transfer(&g_hobot_dma);
    system_dma_wait_done(g_firmware.dma_chan_isp_config);
#endif

    system_dma_copy_sg( g_firmware.dma_chan_isp_config, ISP_CONFIG_PONG, SYS_DMA_TO_DEVICE, NULL, fw_ctx_id );
#if FW_USE_HOBOT_DMA
    isp_idma_start_transfer(&g_hobot_dma);
    system_dma_wait_done(g_firmware.dma_chan_isp_config);
#endif
}

#endif /* #if USER_MODULE */

static void acamera_deinit( void )
{
    int idx;
    acamera_context_t *p_ctx;

    system_dma_destroy( g_firmware.dma_chan_isp_config );
    system_dma_destroy( g_firmware.dma_chan_isp_metering );
    for ( idx = 0; idx < g_firmware.context_number; idx++ ) {
#if FW_HAS_CONTROL_CHANNEL
        ctrl_channel_deinit(idx);
#endif
        p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[idx];

        acamera_deinit_context( p_ctx );

        if ( p_ctx->sw_reg_map.isp_sw_config_map != NULL ) {
#if FW_USE_HOBOT_DMA
            system_sw_free_dma_sram( (void *)p_ctx->sw_reg_map.isp_sw_config_map, p_ctx->context_id );
#else
            system_sw_free( (void *)p_ctx->sw_reg_map.isp_sw_config_map );
#endif
            p_ctx->sw_reg_map.isp_sw_config_map = NULL;
        }
    }
}

int32_t acamera_terminate()
{
    acamera_logger_empty(); //empty the logger buffer and print remaining logs
    acamera_deinit();
    system_semaphore_destroy( g_firmware.sem_evt_avail );
    system_semaphore_destroy(g_firmware.sem_event_process_done);

    return 0;
}


#if USER_MODULE
// single context handler
int32_t acamera_interrupt_handler()
{
    return 0;
}
#else

static int cur_ctx_id;
static int last_ctx_id;
static int next_ctx_id;
static void start_processing_frame( void )
{
#if ISP_HAS_DMA_INPUT
    int32_t cur_ctx = fpga_dma_input_current_context( &g_firmware );
    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[cur_ctx];
    LOG( LOG_INFO, "new frame for ctx_num#%d.", cur_ctx );
#else
    pr_debug("last ctx id %d\n", last_ctx_id);
    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[last_ctx_id];
#endif

    // new_frame event to start reading metering memory and run 3A
    acamera_fw_raise_event( p_ctx, event_id_new_frame );
}

void dma_writer_config_done(void)
{
	wake_up(&wq);
}
EXPORT_SYMBOL(dma_writer_config_done);

static void dma_complete_context_func( void *arg )
{
    int ctx_id = 0;
    system_dma_device_t *system_dma_device = (system_dma_device_t *)arg;

    pr_debug("DMA COMPLETION FOR CONTEXT\n");

    g_firmware.dma_flag_isp_config_completed = 1;

    if (system_dma_device != NULL)
	ctx_id = system_dma_device->cur_fw_ctx_id;

    if ( g_firmware.dma_flag_isp_config_completed && g_firmware.dma_flag_isp_metering_completed ) {
	isp_ctx_node_t *cn;
	static int count = 0;
	volatile void *offset;
	acamera_context_t *p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[ctx_id];

	pr_debug("START PROCESSING FROM CONTEXT CALLBACK, ctx_id %d\n", ctx_id);

	if (p_ctx->isp_ctxsv_on) {
		cn = isp_ctx_get_node(ctx_id, ISP_CTX, FREEQ);
		if (!cn) {
			if (count++ >= 150) { //about 5s
				count = 0;
				p_ctx->isp_ctxsv_on = 0;
			}
			cn = isp_ctx_get_node(ctx_id, ISP_CTX, DONEQ);
		}
		if (cn) {
			cn->ctx.frame_id = p_ctx->isp_frame_counter;
			offset = p_ctx->sw_reg_map.isp_sw_config_map + ACAMERA_DECOMPANDER0_MEM_BASE_ADDR;
			memcpy_fromio(cn->base, offset, CTX_SIZE);
			cn->ctx.crc16 = crc16(~0, cn->base, CTX_SIZE);
			isp_ctx_put_node(ctx_id, cn, ISP_CTX, DONEQ);

			pr_debug("ctx dump frame id %d\n", cn->ctx.frame_id);
		}
	}

	if (p_ctx->sif_isp_offline && p_ctx->fsm_mgr.reserved == 0) { // indicate dma writer is disabled
		g_firmware.dma_flag_dma_writer_config_completed = 1;
		dma_writer_config_done();
	}

        start_processing_frame();
    }

    system_dma_unmap_sg( arg );
}

static void dma_complete_metering_func( void *arg )
{
    int ctx_id = 0;
    system_dma_device_t *system_dma_device = (system_dma_device_t *)arg;

    pr_debug("DMA COMPLETION FOR METERING\n");

    g_firmware.dma_flag_isp_metering_completed = 1;

    if (system_dma_device != NULL)
	ctx_id = system_dma_device->cur_fw_ctx_id;

    if ( g_firmware.dma_flag_isp_config_completed && g_firmware.dma_flag_isp_metering_completed ) {
	acamera_context_t *p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[ctx_id];

	pr_debug("START PROCESSING FROM METERING CALLBACK, ctx_id %d\n", ctx_id);

	if (p_ctx->sif_isp_offline && p_ctx->fsm_mgr.reserved == 0) { // indicate dma writer is disabled
		g_firmware.dma_flag_dma_writer_config_completed = 1;
		dma_writer_config_done();
	}

        start_processing_frame();
    }

    system_dma_unmap_sg( arg );
    // after we finish transfer context and metering we can start processing the current data
}

#if ISP_HAS_DMA_INPUT

static int32_t fpga_irq_handler( uint32_t isp_event )
{
    int32_t result = 0;

    switch ( isp_event ) {

    case ISP_INTERRUPT_EVENT_DMA_WRITER1: {
        static uint32_t fe_1_cnt = 0;
        LOG( LOG_INFO, "ISP_INTERRUPT_EVENT_DMA_WRITER1: %d.", fe_1_cnt++ );

        fpga_dma_input_interrupt( &g_firmware, ACAMERA_IRQ_DFE_FRAME_END, 0 );
    } break;

    case ISP_INTERRUPT_EVENT_DMA_WRITER2: {
        static uint32_t fe_2_cnt = 0;
        LOG( LOG_INFO, "ISP_INTERRUPT_EVENT_DMA_WRITER2: %d.", fe_2_cnt++ );

        fpga_dma_input_interrupt( &g_firmware, ACAMERA_IRQ_DFE_FRAME_END, 1 );
    } break;

#if FIRMWARE_CONTEXT_NUMBER > 2
    case ISP_INTERRUPT_EVENT_DMA_WRITER3:

        fpga_dma_input_interrupt( &g_firmware, ACAMERA_IRQ_DFE_FRAME_END, 2 );
        break;
#endif

#if FIRMWARE_CONTEXT_NUMBER > 3
    case ISP_INTERRUPT_EVENT_DMA_WRITER4:
        fpga_dma_input_interrupt( &g_firmware, ACAMERA_IRQ_DFE_FRAME_END, 3 );
        break;
#endif
    default: {
        result = -1;
        LOG( LOG_DEBUG, "Not expected irq event %d for offline mode", (int)isp_event );
    } break;
    }

    return result;
}

// multiple contexts handler
int32_t acamera_interrupt_handler()
{
    int32_t result = 0;
    int32_t irq_bit = ISP_INTERRUPT_EVENT_NONES_COUNT - 1;
    int32_t irq = 0;
    LOG( LOG_INFO, "Interrupt handler called" );

    uint32_t fpga_irq_mask = acamera_fpga_fe_isp_interrupts_interrupt_status_read( 0 );

    // clear fpga irq bits
    acamera_fpga_fe_isp_interrupts_interrupt_clear_write( 0, 0 );
    acamera_fpga_fe_isp_interrupts_interrupt_clear_write( 0, fpga_irq_mask );

    // process interrupts
    if ( fpga_irq_mask > 0 ) {
        while ( fpga_irq_mask > 0 && irq < ISP_INTERRUPT_EVENT_NONES_COUNT ) {
            int32_t lsb = ( fpga_irq_mask & 1 );
            fpga_irq_mask >>= 1;
            if ( lsb ) {
                if ( g_firmware.dma_input.fpga_isp_interrupt_source_read[irq] != NULL ) {
                    uint32_t isp_event = g_firmware.dma_input.fpga_isp_interrupt_source_read[irq]( 0 );

                    fpga_irq_handler( isp_event );

                } else {
                    result = -1;
                    LOG( LOG_ERR, "IRQ mask had interrupt %d but irq handler routine is null", (int)irq );
                }
            }
            irq++;
        }

        if ( fpga_irq_mask != 0 ) {
            LOG( LOG_INFO, "irq_mask is not zero. Some interrupts have been left unprocessed" );
        }
    }

    int32_t last_ctx = fpga_dma_input_last_context( &g_firmware );
    int32_t cur_ctx = fpga_dma_input_current_context( &g_firmware );
    int32_t next_ctx = fpga_dma_input_next_context( &g_firmware );

    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[cur_ctx];

    // read the irq vector from isp
    uint32_t irq_mask = acamera_isp_isp_global_interrupt_status_vector_read( 0 );

    LOG( LOG_INFO, "IRQ MASK is 0x%x, last_ctx: %d, cur_ctx: %d, next_ctx: %d.", irq_mask, last_ctx, cur_ctx, next_ctx );
    printk("IRQ MASK is 0x%x, last_ctx: %d, cur_ctx: %d, next_ctx: %d.", irq_mask, last_ctx, cur_ctx, next_ctx);

    // clear irq vector
    acamera_isp_isp_global_interrupt_clear_write( 0, 0 );
    acamera_isp_isp_global_interrupt_clear_write( 0, 1 );

    if ( irq_mask > 0 ) {

        /*#if defined( ISP_INTERRUPT_EVENT_BROKEN_FRAME ) && defined( ISP_INTERRUPT_EVENT_MULTICTX_ERROR ) && defined( ISP_INTERRUPT_EVENT_DMA_ERROR ) && defined( ISP_INTERRUPT_EVENT_WATCHDOG_EXP ) && defined( ISP_INTERRUPT_EVENT_FRAME_COLLISION )
        //check for errors in the interrupt
        if ( ( irq_mask & 1 << ISP_INTERRUPT_EVENT_BROKEN_FRAME ) ||
             ( irq_mask & 1 << ISP_INTERRUPT_EVENT_MULTICTX_ERROR ) ||
             ( irq_mask & 1 << ISP_INTERRUPT_EVENT_DMA_ERROR ) ||
             ( irq_mask & 1 << ISP_INTERRUPT_EVENT_WATCHDOG_EXP ) ||
             ( irq_mask & 1 << ISP_INTERRUPT_EVENT_FRAME_COLLISION ) ) {

            LOG( LOG_CRIT, "Found error resetting ISP. MASK is 0x%x", irq_mask );

            acamera_fw_error_routine( p_ctx, irq_mask );
            return -1; //skip other interrupts in case of error
        }
#endif*/

        while ( irq_mask > 0 && irq_bit >= 0 ) {
            int32_t irq_is_1 = ( irq_mask & ( 1 << irq_bit ) );
            irq_mask &= ~( 1 << irq_bit );
            if ( irq_is_1 ) {
                // process interrupts
                if ( irq_bit == ISP_INTERRUPT_EVENT_ISP_START_FRAME_START ) {
                    static uint32_t fs_cnt = 0;

                    LOG( LOG_INFO, "FS interrupt: %d", fs_cnt++ );

#if ISP_DMA_RAW_CAPTURE
                    dma_raw_capture_interrupt( &g_firmware, ACAMERA_IRQ_FRAME_END );
#endif

                    if ( g_firmware.dma_flag_isp_metering_completed == 0 || g_firmware.dma_flag_isp_config_completed == 0 ) {
                        LOG( LOG_CRIT, "DMA is not finished, cfg: %d, meter: %d, skip this frame.", g_firmware.dma_flag_isp_config_completed, g_firmware.dma_flag_isp_metering_completed );
                        return -2;
                    }

                    // we must finish all previous processing before scheduling new dma
                    if ( acamera_event_queue_empty( &p_ctx->fsm_mgr.event_queue ) ) {
                        // switch to ping/pong contexts for the next frame

                        // these flags are used for sync of callbacks
                        g_firmware.dma_flag_isp_config_completed = 0;
                        g_firmware.dma_flag_isp_metering_completed = 0;

                        if ( acamera_isp_isp_global_ping_pong_config_select_read( 0 ) == ISP_CONFIG_PONG ) {
                            LOG( LOG_INFO, "Current config is pong" );
                            //            |^^^^^^^^^|
                            // next --->  |  PING   |
                            //            |_________|

                            // use ping for the next frame
                            acamera_isp_isp_global_mcu_ping_pong_config_select_write( 0, ISP_CONFIG_PING );
                            //
                            //            |^^^^^^^^^|
                            // conf --->  |  PONG   |
                            //            |_________|
                            LOG( LOG_INFO, "DMA metering from pong to DDR of size %d", ACAMERA_METERING_STATS_MEM_SIZE );
                            // dma all stat memory only to the software context
                            system_dma_copy_sg( g_firmware.dma_chan_isp_metering, ISP_CONFIG_PING, SYS_DMA_FROM_DEVICE, dma_complete_metering_func, last_ctx );
                            LOG( LOG_INFO, "DMA config from pong to DDR of size %d", ACAMERA_ISP1_SIZE );
                            system_dma_copy_sg( g_firmware.dma_chan_isp_config, ISP_CONFIG_PING, SYS_DMA_TO_DEVICE, dma_complete_context_func, next_ctx );
                        } else {
                            LOG( LOG_INFO, "Current config is ping" );
                            //            |^^^^^^^^^|
                            // next --->  |  PONG   |
                            //            |_________|

                            // use pong for the next frame
                            acamera_isp_isp_global_mcu_ping_pong_config_select_write( 0, ISP_CONFIG_PONG );

                            //            |^^^^^^^^^|
                            // conf --->  |  PING   |
                            //            |_________|

                            LOG( LOG_INFO, "DMA metering from ping to DDR of size %d", ACAMERA_METERING_STATS_MEM_SIZE );
                            // dma all stat memory only to the software context
                            system_dma_copy_sg( g_firmware.dma_chan_isp_metering, ISP_CONFIG_PONG, SYS_DMA_FROM_DEVICE, dma_complete_metering_func, last_ctx );
                            LOG( LOG_INFO, "DMA config from DDR to ping of size %d", ACAMERA_ISP1_SIZE );
                            system_dma_copy_sg( g_firmware.dma_chan_isp_config, ISP_CONFIG_PONG, SYS_DMA_TO_DEVICE, dma_complete_context_func, next_ctx );
                        }
                    } else {
                        LOG( LOG_ERR, "Attempt to start a new frame before processing is done for the prevous frame. Skip this frame" );
                    }
                } else if ( irq_bit == ISP_INTERRUPT_EVENT_ISP_END_FRAME_END ) {
                    static uint32_t fe_cnt = 0;

                    LOG( LOG_INFO, "FE interrupt: %d", fe_cnt++ );

                    fpga_dma_input_interrupt( &g_firmware, ACAMERA_IRQ_FRAME_END, cur_ctx );

                } else {
                    // unhandled irq
                    LOG( LOG_INFO, "Unhandled interrupt bit %d", irq_bit );
                }
            }
            irq_bit--;
        }
    }

    return result;
}

#else
#if FW_USE_HOBOT_DMA
static void set_dma_cmd_queue(dma_cmd *cmd, uint32_t ping_pong_sel)
{
    cmd[0].ctx = g_firmware.dma_chan_isp_metering;
    cmd[0].buff_loc = ping_pong_sel;
    cmd[0].direction = SYS_DMA_FROM_DEVICE;
    cmd[0].complete_func = dma_complete_metering_func;
    cmd[0].fw_ctx_id = last_ctx_id;

    cmd[1].ctx = g_firmware.dma_chan_isp_config;
    cmd[1].buff_loc = ping_pong_sel;
    cmd[1].direction = SYS_DMA_TO_DEVICE;
    cmd[1].complete_func = dma_complete_context_func;
    cmd[1].fw_ctx_id = next_ctx_id;
}
#endif /* FW_USE_HOBOT_DMA*/

void isp_ctx_prepare(int ctx_pre, int ctx_next, int ppf)
{
#if FW_USE_HOBOT_DMA
	dma_cmd cmd[2];
	if (ctx_pre == ctx_next) {
		set_dma_cmd_queue(cmd, ppf);
		system_dma_copy_multi_sg(cmd, 2);
	} else {
		system_dma_copy_sg(g_firmware.dma_chan_isp_metering, ppf, SYS_DMA_FROM_DEVICE, dma_complete_metering_func, ctx_pre);
		system_dma_copy_sg(g_firmware.dma_chan_isp_config, ppf, SYS_DMA_TO_DEVICE, dma_complete_context_func, ctx_next);
		isp_idma_start_transfer(&g_hobot_dma);
	}
#else
	system_dma_copy_sg(g_firmware.dma_chan_isp_metering, ppf, SYS_DMA_FROM_DEVICE, dma_complete_metering_func, ctx_pre);
	system_dma_copy_sg(g_firmware.dma_chan_isp_config, ppf, SYS_DMA_TO_DEVICE, dma_complete_context_func, ctx_next);
#endif
}

extern int ldc_set_ioctl(uint32_t port, uint32_t online);
extern void isp_input_port_size_config(sensor_fsm_ptr_t p_fsm);
extern int ips_get_isp_frameid(void);
int sif_isp_ctx_sync_func(int ctx_id)
{
	acamera_context_ptr_t p_ctx;

	p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[ctx_id];
	if (g_firmware.frame_done == 0 && p_ctx->isp_frame_counter != 0) {
		pr_debug("isp is working now, next ctx id %d.\n", ctx_id);
		wait_event_timeout(wq_fe, g_firmware.frame_done, msecs_to_jiffies(200));
	}

	pr_debug("start isp ctx switch, ctx_id %d\n", ctx_id);

    //in case of non ctx-0 run first
    if (p_ctx->isp_frame_counter == 0)
        last_ctx_id = ctx_id;
    else
	    last_ctx_id = cur_ctx_id;

	cur_ctx_id = ctx_id;
	next_ctx_id = ctx_id;
	p_ctx->sif_isp_offline = 1;
	g_firmware.frame_done = 0;

	isp_input_port_size_config(p_ctx->fsm_mgr.fsm_arr[FSM_ID_SENSOR]->p_fsm);
	ldc_set_ioctl(ctx_id, 0);
retry:
	if (acamera_event_queue_empty(&p_ctx->fsm_mgr.event_queue)) {
		// these flags are used for sync of callbacks
		g_firmware.dma_flag_isp_config_completed = 0;
		g_firmware.dma_flag_isp_metering_completed = 0;
		g_firmware.dma_flag_dma_writer_config_completed = 0;

		// switch to ping/pong contexts for the next frame
		if (acamera_isp_isp_global_ping_pong_config_select_read(0) == ISP_CONFIG_PONG
			|| p_ctx->isp_frame_counter == 0) {
			acamera_isp_isp_global_mcu_ping_pong_config_select_write(0, ISP_CONFIG_PING);
			pr_debug("next is ping, DMA sram -> ping\n");
			isp_ctx_prepare(last_ctx_id, next_ctx_id, ISP_CONFIG_PING);
		} else {
			acamera_isp_isp_global_mcu_ping_pong_config_select_write(0, ISP_CONFIG_PONG);
			pr_debug("next is pong, DMA sram -> pong\n");
			isp_ctx_prepare(last_ctx_id, next_ctx_id, ISP_CONFIG_PONG);
		}
	} else {
		system_semaphore_wait(g_firmware.sem_event_process_done, 0);
		goto retry;
	}

	if (!(g_firmware.dma_flag_isp_config_completed &&
		g_firmware.dma_flag_isp_metering_completed &&
		g_firmware.dma_flag_dma_writer_config_completed)) {

		wait_event_timeout(wq, g_firmware.dma_flag_dma_writer_config_completed, msecs_to_jiffies(1000));
		pr_debug("ISP->SIF: wake up sif feed thread\n");
	} else
		pr_debug("ISP->SIF: do not need waiting, return to sif feed thread\n");

	return 0;
}

// single context handler
int32_t acamera_interrupt_handler()
{
    int32_t result = 0;
    int32_t irq_bit = ISP_INTERRUPT_EVENT_NONES_COUNT - 1;

    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[cur_ctx_id];

#if 0
uint32_t hcs1 = acamera_isp_input_port_hc_size0_read(0);
uint32_t hcs2 = acamera_isp_input_port_hc_size1_read(0);
uint32_t vc = acamera_isp_input_port_vc_size_read(0);
pr_info("hcs1 %d, hcs2 %d, vc %d\n", hcs1, hcs2, vc);

    if (acamera_isp_isp_global_ping_pong_config_select_read(0) == ISP_CONFIG_PONG) {
	printk("pong\n");
	printk("top w/h %x\n", system_hw_read_32(0x30e48));
	printk("metering af w/h %x\n", system_hw_read_32(0x336e8));
	printk("lumvar w/h %x\n", system_hw_read_32(0x33234));
    } else {
	printk("ping\n");
	printk("top w/h %x\n", system_hw_read_32(0x18e88));
	printk("metering af w/h %x\n", system_hw_read_32(0x1b728));
	printk("lumvar w/h %x\n", system_hw_read_32(0x1b274));
    }
#endif

    // read the irq vector from isp
    uint32_t irq_mask = acamera_isp_isp_global_interrupt_status_vector_read( 0 );

    // Update frame counter
    p_ctx->isp_frame_counter = ips_get_isp_frameid();
    pr_debug("IRQ MASK is 0x%x, ctx_id %d, frame id %d\n", irq_mask, cur_ctx_id, p_ctx->isp_frame_counter);

    if(irq_mask&0x8) {
        printk("broken frame status = 0x%x", acamera_isp_isp_global_monitor_broken_frame_status_read(0));
        printk("active width min/max/sum/num = %d/%d/%d/%d", system_hw_read_32(0xb4),system_hw_read_32(0xb8),system_hw_read_32(0xbc),system_hw_read_32(0xc0));
        printk("active high min/max/sum/num = %d/%d/%d/%d", system_hw_read_32(0xc4),system_hw_read_32(0xc8),system_hw_read_32(0xcc),system_hw_read_32(0xd0));
        printk("hblank min/max/sum/num = %d/%d/%d/%d", system_hw_read_32(0xd4),system_hw_read_32(0xd8),system_hw_read_32(0xdc),system_hw_read_32(0xe0));
        printk("vblank min/max/sum/num = %d/%d/%d/%d", system_hw_read_32(0xe4),system_hw_read_32(0xe8),system_hw_read_32(0xec),system_hw_read_32(0xf0));
	printk("input port w/h %x, ping w/h %x, pong w/h %x", system_hw_read_32(0x98L), system_hw_read_32(0x18e88L), system_hw_read_32(0x30e48L));
    }

    // clear irq vector
    acamera_isp_isp_global_interrupt_clear_write( 0, 0 );
    acamera_isp_isp_global_interrupt_clear_write( 0, 1 );

    if ( irq_mask > 0 ) {
#if 0
        //check for errors in the interrupt
        if ( ( irq_mask & 1 << ISP_INTERRUPT_EVENT_BROKEN_FRAME ) ||
             ( irq_mask & 1 << ISP_INTERRUPT_EVENT_MULTICTX_ERROR ) ||
             ( irq_mask & 1 << ISP_INTERRUPT_EVENT_WATCHDOG_EXP ) ||
             ( irq_mask & 1 << ISP_INTERRUPT_EVENT_DMA_ERROR ) ||
             ( irq_mask & 1 << ISP_INTERRUPT_EVENT_FRAME_COLLISION ) ) {

            LOG( LOG_ERR, "Found error resetting ISP. MASK is 0x%x", irq_mask );
            acamera_fw_error_routine( p_ctx, irq_mask );
        }
	if ( irq_mask & 1 << ISP_INTERRUPT_EVENT_DMA_ERROR ) {
		acamera_fw_raise_event( p_ctx, event_id_frame_error );	//move node from busy to free list
		acamera_fw_raise_event( p_ctx, event_id_frame_config );	//get a new buffer put to busy list
            return -1;
	} else
            return -1;
#endif

        while ( irq_mask > 0 && irq_bit >= 0 ) {
            int32_t irq_is_1 = ( irq_mask & ( 1 << irq_bit ) );
            irq_mask &= ~( 1 << irq_bit );
            if ( irq_is_1 ) {
                // process interrupts
                if ( p_ctx->sif_isp_offline == 0 && irq_bit == ISP_INTERRUPT_EVENT_ISP_START_FRAME_START ) {
                    static uint32_t fs_cnt = 0;
                    if ( fs_cnt < 10 ) {
                        LOG( LOG_INFO, "[KeyMsg]: FS interrupt: %d", fs_cnt++ );
                    }

#if ISP_DMA_RAW_CAPTURE
                    dma_raw_capture_interrupt( &g_firmware, ACAMERA_IRQ_FRAME_END );
#endif

                    if ( g_firmware.dma_flag_isp_metering_completed == 0 || g_firmware.dma_flag_isp_config_completed == 0 ) {
                        LOG( LOG_ERR, "DMA is not finished, cfg: %d, meter: %d, skip this frame.", g_firmware.dma_flag_isp_config_completed, g_firmware.dma_flag_isp_metering_completed );
                        return -2;
                    }

                    // we must finish all previous processing before scheduling new dma
                    if ( acamera_event_queue_empty( &p_ctx->fsm_mgr.event_queue ) ) {
                        // switch to ping/pong contexts for the next frame

                        // these flags are used for sync of callbacks
                        g_firmware.dma_flag_isp_config_completed = 0;
                        g_firmware.dma_flag_isp_metering_completed = 0;

                        if ( acamera_isp_isp_global_ping_pong_config_select_read( 0 ) == ISP_CONFIG_PONG ) {
                            LOG( LOG_INFO, "Current config is pong" );
                            //            |^^^^^^^^^|
                            // next --->  |  PING   |
                            //            |_________|

                            // use ping for the next frame
                            acamera_isp_isp_global_mcu_ping_pong_config_select_write( 0, ISP_CONFIG_PING );
                            //
                            //            |^^^^^^^^^|
                            // conf --->  |  PONG   |
                            //            |_________|
			    isp_ctx_prepare(0, 0, ISP_CONFIG_PING);
                        } else {
                            LOG( LOG_INFO, "Current config is ping" );
                            //            |^^^^^^^^^|
                            // next --->  |  PONG   |
                            //            |_________|

                            // use pong for the next frame
                            acamera_isp_isp_global_mcu_ping_pong_config_select_write( 0, ISP_CONFIG_PONG );

                            //            |^^^^^^^^^|
                            // conf --->  |  PING   |
                            //            |_________|
			    isp_ctx_prepare(0, 0, ISP_CONFIG_PONG);
                        }
                    } else {
                        LOG( LOG_ERR, "Attempt to start a new frame before processing is done for the prevous frame. Skip this frame" );
                    } //if ( acamera_event_queue_empty( &p_ctx->fsm_mgr.event_queue ) )
                } else if ( irq_bit == ISP_INTERRUPT_EVENT_ISP_END_FRAME_END ) {
			pr_debug("frame done, ctx id %d\n", cur_ctx_id);
			g_firmware.frame_done = 1;
			wake_up(&wq_fe);
                } else if ( irq_bit == ISP_INTERRUPT_EVENT_FR_Y_WRITE_DONE ) {
					LOG( LOG_INFO, "frame write to ddr done" );
					acamera_fw_raise_event( p_ctx, event_id_frame_done );
                } else if ( irq_bit == ISP_INTERRUPT_EVENT_FR_UV_WRITE_DONE ) {
					//do nothing
				} else {
                    // unhandled irq
                    LOG( LOG_INFO, "Unhandled interrupt bit %d", irq_bit );
                }
            }
			irq_bit--;
        } //while ( irq_mask > 0 && irq_bit >= 0 )
    } //if ( irq_mask > 0 )

    return result;
}
#endif // ISP_HAS_DMA_INPUT

#endif // USER_MODULE

extern int isp_stream_onoff_check(void);
extern void system_interrupts_disable( void );
int32_t acamera_process( void )
{
    int32_t result = 0;
    int32_t idx = 0;
    acamera_context_ptr_t p_ctx = NULL;

    if ( g_firmware.initialized == 1 ) {
        for ( idx = 0; idx < g_firmware.context_number; idx++ ) {
            p_ctx = ( acamera_context_ptr_t ) & ( g_firmware.fw_ctx[idx] );
            acamera_fw_process( p_ctx );
        }
    } else {
        result = -1;
    }

#if FW_HAS_CONTROL_CHANNEL
    ctrl_channel_process();
#endif

    system_semaphore_raise(g_firmware.sem_event_process_done);

    /* disable irq when no stream on, else will blocking when resuming from suspend */
    if (isp_stream_onoff_check() == 0)
        system_interrupts_disable();

    // system_semaphore_wait( g_firmware.sem_evt_avail, FW_EVT_QUEUE_TIMEOUT_MS );
    system_semaphore_wait( g_firmware.sem_evt_avail, 0 );

    return result;
}
