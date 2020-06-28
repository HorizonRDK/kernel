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
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
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

#include "dma_writer.h"
#include "dma_writer_fsm.h"

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

#define GET_BYTE_V(v, pos)      (v >> pos % sizeof(v) * 8 & 0xff)

static acamera_firmware_t g_firmware;

typedef int (*isp_callback)(int);
extern void isp_register_callback(isp_callback func);
int sif_isp_ctx_sync_func(int ctx_id);
static DECLARE_WAIT_QUEUE_HEAD(wq_dma_cfg_done);
static DECLARE_WAIT_QUEUE_HEAD(wq_frame_end);
static DECLARE_WAIT_QUEUE_HEAD(wq_dma_done);

static int cur_ctx_id;
static int last_ctx_id;
static int cur_chn_id;
static int last_chn_id;
static int swap_ctx_id;

#if FW_USE_HOBOT_DMA
extern hobot_dma_t g_hobot_dma;
#endif
extern int isp_stream_onoff_check(void);
extern void system_interrupts_disable( void );

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

acamera_firmware_t *acamera_get_firmware_ptr(void)
{
    return &g_firmware;
}

int acamera_all_hw_contexts_inited(void)
{
    int i = 0;
    int count = 0;
    acamera_context_ptr_t p_ctx;

    for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
        p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[i];
        if (p_ctx && p_ctx->initialized)
            count++;
    }

    if (count >= HW_CONTEXT_NUMBER)
        return 1;

    return 0;
}

void acamera_notify_evt_data_avail( acamera_context_t *p_ctx )
{
    system_semaphore_raise( p_ctx->sem_evt_avail );
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
    int ret = 0;
    acamera_context_t *p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[idx];

    pr_info("+\n");

    ret = validate_settings( settings, idx );
    if (ret < 0) {
        pr_err("settings[%d] validate not correct\n", idx);
        return ret;
    }

    ret = acamera_init_context( p_ctx, &settings[idx], &g_firmware );
    if (ret == 0) {
        if (g_firmware.initialized) {
            pr_debug("firmware have been inited, don't touch isp hardware\n");
            goto out;
        }

        //update for isp fore-end online
        cur_ctx_id = idx;
        last_ctx_id = idx;
        cur_chn_id = p_ctx->dma_chn_idx;
        last_chn_id = p_ctx->dma_chn_idx;

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

int acamera_isp_deinit_context(uint8_t idx)
{
    int ret = 0;
    acamera_context_t *p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[idx];

    acamera_deinit_context(p_ctx);

    return ret;
}

int acamera_isp_firmware_clear(void)
{
    g_firmware.initialized = 0;
    g_firmware.dma_flag_isp_config_completed = 0;
    g_firmware.dma_flag_isp_metering_completed = 0;   

    return 0;
}

extern void ips_set_isp_interrupt(bool enable);
int32_t acamera_init( acamera_settings *settings, uint32_t ctx_num )
{
    int32_t result = 0;
    uint32_t idx;

    // disable irq and clear interrupts
    // acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_DISABLE_ALL_IRQ );
    ips_set_isp_interrupt(0);

#if ACAMERA_ISP_PROFILING && ACAMERA_ISP_PROFILING_INIT
    acamera_profiler_init();
    acamera_profiler_start( 0 );
#endif

    g_firmware.api_context = 0;
    g_firmware.first_frame = 0;
    g_firmware.sif_isp_offline = 0;
    g_firmware.sw_frame_counter = 0;
    g_firmware.iridix_ctrl_flag = 0;
    g_firmware.cache_area = NULL;

    mutex_init(&g_firmware.ctx_chg_lock);

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

                system_semaphore_init( &p_ctx->sem_evt_avail );

                // dump hw default configuration to the current context

#if FW_USE_HOBOT_DMA
                if (idx < HW_CONTEXT_NUMBER) {
                    p_ctx->sw_reg_map.isp_sw_config_map = system_sw_alloc_dma_sram(HOBOT_DMA_SRAM_ONE_ZONE,
                                                                                p_ctx->context_id,
                                                                                &p_ctx->sw_reg_map.isp_sw_phy_addr);
                }
#else
                p_ctx->sw_reg_map.isp_sw_config_map = system_sw_alloc(HOBOT_DMA_SRAM_ONE_ZONE);
                p_ctx->sw_reg_map.isp_sw_phy_addr = 0;  // no use when DMA mode disable
#endif
                if (idx < HW_CONTEXT_NUMBER) {
                    if ( p_ctx->sw_reg_map.isp_sw_config_map ) {
                        result = dma_channel_addresses_setup( g_firmware.dma_chan_isp_config, g_firmware.dma_chan_isp_metering,
                                            (void *)p_ctx->sw_reg_map.isp_sw_config_map, idx, settings->hw_isp_addr,
                                            p_ctx->sw_reg_map.isp_sw_phy_addr);
                    } else {
                        LOG( LOG_CRIT, "Software Context %d failed to allocate", idx );
                        result = -1;
                    }
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

        system_semaphore_destroy( p_ctx->sem_evt_avail );

        if ( p_ctx->sw_reg_map.isp_sw_config_map != NULL ) {
#if FW_USE_HOBOT_DMA
            if (p_ctx->content_side == SIDE_SRAM)
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

    return 0;
}


#if USER_MODULE
// single context handler
int32_t acamera_interrupt_handler()
{
    return 0;
}
#else

void input_port_status(void)
{
    pr_info("broken frame status = 0x%x\n", acamera_isp_isp_global_monitor_broken_frame_status_read(0));
    pr_info("active width min/max/sum/num = %d/%d/%d/%d\n", system_hw_read_32(0xb4),system_hw_read_32(0xb8),system_hw_read_32(0xbc),system_hw_read_32(0xc0));
    pr_info("active high min/max/sum/num = %d/%d/%d/%d\n", system_hw_read_32(0xc4),system_hw_read_32(0xc8),system_hw_read_32(0xcc),system_hw_read_32(0xd0));
    pr_info("hblank min/max/sum/num = %d/%d/%d/%d\n", system_hw_read_32(0xd4),system_hw_read_32(0xd8),system_hw_read_32(0xdc),system_hw_read_32(0xe0));
    pr_info("vblank min/max/sum/num = %d/%d/%d/%d\n", system_hw_read_32(0xe4),system_hw_read_32(0xe8),system_hw_read_32(0xec),system_hw_read_32(0xf0));

    pr_info("dma alarms sts %x\n", system_hw_read_32(0x00054));

    pr_info("input port w/h %x, ping w/h %x, pong w/h %x\n", system_hw_read_32(0x98L), system_hw_read_32(0x18e88L), system_hw_read_32(0x30e48L));
}

// dma writer status debug
void dma_writer_status(acamera_context_ptr_t p_ctx)
{
    uint16_t v = 0;

    v = system_hw_read_32(0x00054);
    pr_info("dma alarms sts %x\n", v);

    pr_info("==ping==\n");
    v = system_hw_read_32(0x1c110);
    pr_info("y wbank sts %x\n", v);
    v = system_hw_read_32(0x1c11c);
    pr_info("y icount, wcount %x\n", v);
    v = system_hw_read_32(0x1c124);
    pr_info("y fail sts %x\n", v);
    v = system_hw_read_32(0x1c128);
    pr_info("y blk sts %x\n", v);

    v = system_hw_read_32(0x1c168);
    pr_info("uv wbank sts %x\n", v);
    v = system_hw_read_32(0x1c174);
    pr_info("uv icount, wcount %x\n", v);
    v = system_hw_read_32(0x1c17c);
    pr_info("uv fail sts %x\n", v);
    v = system_hw_read_32(0x1c180);
    pr_info("uv blk sts %x\n", v);

    pr_info("==pong==\n");
    v = system_hw_read_32(0x1c110 + ISP_CONFIG_PING_SIZE);
    pr_info("y wbank sts %x\n", v);
    v = system_hw_read_32(0x1c11c + ISP_CONFIG_PING_SIZE);
    pr_info("y icount, wcount %x\n", v);
    v = system_hw_read_32(0x1c124 + ISP_CONFIG_PING_SIZE);
    pr_info("y fail sts %x\n", v);
    v = system_hw_read_32(0x1c128 + ISP_CONFIG_PING_SIZE);
    pr_info("y blk sts %x\n", v);

    v = system_hw_read_32(0x1c168 + ISP_CONFIG_PING_SIZE);
    pr_info("uv wbank sts %x\n", v);
    v = system_hw_read_32(0x1c174 + ISP_CONFIG_PING_SIZE);
    pr_info("uv icount, wcount %x\n", v);
    v = system_hw_read_32(0x1c17c + ISP_CONFIG_PING_SIZE);
    pr_info("uv fail sts %x\n", v);
    v = system_hw_read_32(0x1c180 + ISP_CONFIG_PING_SIZE);
    pr_info("uv blk sts %x\n", v);
}

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

    // dma_writer_status(p_ctx);
}

void dma_writer_config_done(void)
{
	wake_up(&wq_dma_cfg_done);
}
EXPORT_SYMBOL(dma_writer_config_done);

static void dma_complete_context_func( void *arg )
{
    int ctx_id = cur_ctx_id;

    pr_debug("DMA COMPLETION FOR CONTEXT\n");

    g_firmware.dma_flag_isp_config_completed = 1;

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
         // indicate dma writer is disabled
        if (p_ctx->p_gfw->sif_isp_offline && p_ctx->fsm_mgr.reserved == 0
            && isp_stream_onoff_check() != 2) {
            g_firmware.handler_flag_interrupt_handle_completed = 1;
        }

        start_processing_frame();
    }
    dma_writer_config_done();
    system_dma_unmap_sg( arg );
}

static void dma_complete_metering_func( void *arg )
{
    int ctx_id = cur_ctx_id;

    pr_debug("DMA COMPLETION FOR METERING\n");

    g_firmware.dma_flag_isp_metering_completed = 1;

    if ( g_firmware.dma_flag_isp_config_completed && g_firmware.dma_flag_isp_metering_completed ) {
        acamera_context_t *p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[ctx_id];

	pr_debug("START PROCESSING FROM METERING CALLBACK, ctx_id %d\n", ctx_id);

         // indicate dma writer is disabled
        if (p_ctx->p_gfw->sif_isp_offline && p_ctx->fsm_mgr.reserved == 0
            && isp_stream_onoff_check() != 2) {
            g_firmware.handler_flag_interrupt_handle_completed = 1;
        }

        start_processing_frame();
    }
    dma_writer_config_done();
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
static int _isp_iridix_ctrl(void)
{
    uint8_t iridix_no = 0xff;
    int giver_ctx_id;
    int accepter_ctx_id;
    acamera_context_ptr_t p_ctx;

    giver_ctx_id = GET_BYTE_V(g_firmware.iridix_ctrl_flag, 1);
    accepter_ctx_id = GET_BYTE_V(g_firmware.iridix_ctrl_flag, 0);

    if (giver_ctx_id >= FIRMWARE_CONTEXT_NUMBER || accepter_ctx_id >= FIRMWARE_CONTEXT_NUMBER) {
       pr_debug("giver id %d or accepter id %d is invalid.\n", giver_ctx_id, accepter_ctx_id);
       return -1;
    }

    pr_debug("giver id %d, accepter id %d\n", giver_ctx_id, accepter_ctx_id);

    p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[giver_ctx_id];
    if (p_ctx && p_ctx->sw_reg_map.isp_sw_config_map != NULL) {
        iridix_no = acamera_isp_iridix_context_no_read(p_ctx->settings.isp_base);

        //1: turn over  2: share
        if (GET_BYTE_V(g_firmware.iridix_ctrl_flag, 2) == 1) {
            acamera_isp_top_bypass_iridix_write(p_ctx->settings.isp_base, 1);
            acamera_isp_iridix_enable_write(p_ctx->settings.isp_base, 0);
            //TODO
            //need maybe load calibration without iridix
        }
    }

    p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[accepter_ctx_id];
    if (iridix_no < HW_CONTEXT_NUMBER && p_ctx && p_ctx->sw_reg_map.isp_sw_config_map != NULL) {
        acamera_isp_iridix_context_no_write(p_ctx->settings.isp_base, iridix_no);
        acamera_isp_iridix_enable_write(p_ctx->settings.isp_base, 1);
        acamera_isp_top_bypass_iridix_write(p_ctx->settings.isp_base, 1);
        //TODO
        //need maybe load calibration with iridix
    }

    g_firmware.iridix_ctrl_flag = 0;

    return 0;
}

static int _all_contexts_frame_counter_status(void)
{
    int i = 0;
    acamera_context_ptr_t p_ctx;

    for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
        p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[i];
        if (p_ctx->isp_frame_counter)
            break;
    }

    if (i >= FIRMWARE_CONTEXT_NUMBER)
        return 0;

    return 1;
}

static int _get_first_swap_ctx_id(void)
{
    int i = 0;
    acamera_context_ptr_t p_ctx;

    for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
        p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[i];
        if (p_ctx->dma_chn_idx != -1)
            break;
    }

    return i;
}

#if FW_USE_HOBOT_DMA
static void set_dma_cmd_queue(dma_cmd *cmd, uint32_t ping_pong_sel)
{
    cmd[0].ctx = g_firmware.dma_chan_isp_metering;
    cmd[0].buff_loc = !ping_pong_sel;
    cmd[0].direction = SYS_DMA_FROM_DEVICE;
    cmd[0].complete_func = dma_complete_metering_func;
    cmd[0].fw_ctx_id = last_chn_id;

    cmd[1].ctx = g_firmware.dma_chan_isp_config;
    cmd[1].buff_loc = ping_pong_sel;
    cmd[1].direction = SYS_DMA_TO_DEVICE;
    cmd[1].complete_func = dma_complete_context_func;
    cmd[1].fw_ctx_id = cur_chn_id;
}
#endif /* FW_USE_HOBOT_DMA*/

void isp_ctx_transfer(int ctx_pre, int ctx_next, int ppf)
{
    pr_debug("chn_pre %d, chn_next %d, ppf %d\n", ctx_pre, ctx_next, ppf);
#if FW_USE_HOBOT_DMA
	dma_cmd cmd[2];
	if (ctx_pre == ctx_next) {
		set_dma_cmd_queue(cmd, ppf);
		system_dma_copy_multi_sg(cmd, 2);
	} else {
		system_dma_copy_sg(g_firmware.dma_chan_isp_metering, !ppf, SYS_DMA_FROM_DEVICE, dma_complete_metering_func, ctx_pre);
		system_dma_copy_sg(g_firmware.dma_chan_isp_config, ppf, SYS_DMA_TO_DEVICE, dma_complete_context_func, ctx_next);
		isp_idma_start_transfer(&g_hobot_dma);
	}
#else
	system_dma_copy_sg(g_firmware.dma_chan_isp_metering, !ppf, SYS_DMA_FROM_DEVICE, dma_complete_metering_func, ctx_pre);
	system_dma_copy_sg(g_firmware.dma_chan_isp_config, ppf, SYS_DMA_TO_DEVICE, dma_complete_context_func, ctx_next);
#endif
}

void _fsm_isp_base_update(acamera_context_ptr_t p_ctx)
{
    int i = 0;
    dma_handle *dh = NULL;

    for(i = 0; i < FSM_ID_MAX; i++) {
        p_ctx->fsm_mgr.fsm_arr[i]->isp_base = (uintptr_t)p_ctx->sw_reg_map.isp_sw_config_map;
    }

    p_ctx->settings.isp_base = (uintptr_t)p_ctx->sw_reg_map.isp_sw_config_map;
    p_ctx->fsm_mgr.isp_base = (uintptr_t)p_ctx->sw_reg_map.isp_sw_config_map;
    dh = ((dma_writer_fsm_const_ptr_t)(p_ctx->fsm_mgr.fsm_arr[FSM_ID_DMA_WRITER]->p_fsm))->handle;
    dh->pipe[dma_fr].settings.isp_base = (uintptr_t)p_ctx->sw_reg_map.isp_sw_config_map;
}

void _ctx_chn_idx_update(int ctx_id)
{
    int i = 0;
    static uint8_t frame_list[FIRMWARE_CONTEXT_NUMBER] = {0};
    acamera_context_ptr_t p_ctx, p_tmp;

    p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[ctx_id];

    if (p_ctx->p_gfw->sw_frame_counter++ < HW_CONTEXT_NUMBER) {
        swap_ctx_id = _get_first_swap_ctx_id();
    } else {
        swap_ctx_id = last_ctx_id;
    }

    //if swap_ctx_id is invalid
    p_tmp = (acamera_context_ptr_t)&g_firmware.fw_ctx[swap_ctx_id];
    i = FIRMWARE_CONTEXT_NUMBER - 3;
    while ((p_tmp->dma_chn_idx < 0 || !p_tmp->initialized) && i >= 0) {
        swap_ctx_id = frame_list[i];
        p_tmp = (acamera_context_ptr_t)&g_firmware.fw_ctx[swap_ctx_id];
        i--;
    }

    if (_all_contexts_frame_counter_status() == 0) {
        last_ctx_id = ctx_id;
        last_chn_id = (p_ctx->dma_chn_idx < 0) ? 0 : p_ctx->dma_chn_idx;
    } else {
	    last_ctx_id = cur_ctx_id;
        last_chn_id = cur_chn_id;
    }

	cur_ctx_id = ctx_id;
    cur_chn_id = (p_ctx->dma_chn_idx < 0) ? 0 : p_ctx->dma_chn_idx;

    pr_debug("sw cnter %d, swap ctx id %d, last ctx id %d\n",
            p_ctx->p_gfw->sw_frame_counter, swap_ctx_id, last_ctx_id);
    pr_debug("last chn id %d, cur ctx id %d, cur chn id %d\n",
            last_chn_id, cur_ctx_id, cur_chn_id);
    for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++)
        pr_debug("%d ", frame_list[i]);

    //record ctx id, according to frame comming sequence
    for (i = 1; i < FIRMWARE_CONTEXT_NUMBER; i++)
        frame_list[i-1] = frame_list[i];
    frame_list[FIRMWARE_CONTEXT_NUMBER-1] = cur_ctx_id;
}

int isp_ctx_prepare(acamera_context_ptr_t p_ctx)
{
    int ret = 0;
    int be_out_ctx_deinited = 0;
    volatile uint8_t *p_ctx_addr;
    acamera_context_ptr_t p_ctx_be_out;

    pr_debug("swap in ctx: id %d, side %s, dma_chn %d, paddr %p\n",
            p_ctx->context_id, p_ctx->content_side ? "ddr" : "sram",
            p_ctx->dma_chn_idx, p_ctx->sw_reg_map.isp_sw_config_map);

    if (p_ctx->sw_reg_map.isp_sw_config_map == NULL) {
        pr_err("ctx id %d exit.\n", p_ctx->context_id);
        return -1;
    }

    //isp ctx swap
    p_ctx_be_out = (acamera_context_ptr_t)&g_firmware.fw_ctx[swap_ctx_id];

    //in case of be-out ctx have been exit
    if (p_ctx_be_out->dma_chn_idx < 0 && p_ctx_be_out->initialized == 0) {
        int i = 0;
        for (i = 0; i < HW_CONTEXT_NUMBER; i++) {
            if (!test_bit(i, &g_firmware.dma_chn_bitmap)) {
                p_ctx_be_out->dma_chn_idx = i;
                set_bit(i, &g_firmware.dma_chn_bitmap);
                break;
            }
        }
        be_out_ctx_deinited = 1;
        pr_debug("ctx id %d have been exit\n", p_ctx_be_out->context_id);
    }

    cur_chn_id = p_ctx_be_out->dma_chn_idx;

    pr_debug("swap out ctx: id %d, side %s, dma_chn %d\n",
            p_ctx_be_out->context_id, p_ctx_be_out->content_side ? "ddr" : "sram",
            p_ctx_be_out->dma_chn_idx);
    pr_debug("be out paddr %p, cache %p\n",
            p_ctx_be_out->sw_reg_map.isp_sw_config_map, p_ctx->p_gfw->cache_area);

    //swap out
    if (be_out_ctx_deinited == 0) {
        memcpy_fromio((void *)p_ctx->p_gfw->cache_area, (void *)p_ctx_be_out->sw_reg_map.isp_sw_config_map, HOBOT_DMA_SRAM_ONE_ZONE);
        p_ctx_be_out->content_side = SIDE_DDR;
        p_ctx_be_out->dma_chn_idx = -1;
    }

    //swap in
    memcpy_toio((void *)p_ctx_be_out->sw_reg_map.isp_sw_config_map, (void *)p_ctx->sw_reg_map.isp_sw_config_map, HOBOT_DMA_SRAM_ONE_ZONE);
    p_ctx->content_side = SIDE_SRAM;
    p_ctx->dma_chn_idx = cur_chn_id;

    //swap pointer
    if (be_out_ctx_deinited == 0) {
        p_ctx_addr = p_ctx_be_out->sw_reg_map.isp_sw_config_map;
        p_ctx_be_out->sw_reg_map.isp_sw_config_map = p_ctx->p_gfw->cache_area;
        p_ctx->p_gfw->cache_area = p_ctx->sw_reg_map.isp_sw_config_map;
        p_ctx->sw_reg_map.isp_sw_config_map = p_ctx_addr;
    } else {
        vfree((void *)p_ctx->sw_reg_map.isp_sw_config_map);
        p_ctx->sw_reg_map.isp_sw_config_map = p_ctx_be_out->sw_reg_map.isp_sw_config_map;
        pr_debug("ctx id %d have swaped in, ctx id %d is exit, free mem\n",
                p_ctx->context_id, p_ctx_be_out->context_id);
    }

    _fsm_isp_base_update(p_ctx);

    pr_debug("swap done ctx: id %d, side %s, dma_chn %d, paddr %p\n",
            p_ctx->context_id, p_ctx->content_side ? "ddr" : "sram",
            p_ctx->dma_chn_idx, p_ctx->sw_reg_map.isp_sw_config_map);
    pr_debug("swaped paddr %p, cache %p\n",
            p_ctx_be_out->sw_reg_map.isp_sw_config_map, p_ctx->p_gfw->cache_area);

    return ret;
}

extern int ldc_set_ioctl(uint32_t port, uint32_t online);
extern int dis_set_ioctl(uint32_t port, uint32_t online);
extern void isp_input_port_size_config(sensor_fsm_ptr_t p_fsm);
extern int ips_get_isp_frameid(void);
extern int dma_writer_configure_pipe( dma_pipe *pipe );
int sif_isp_ctx_sync_func(int ctx_id)
{
    int ret = 0;
	acamera_context_ptr_t p_ctx;
    acamera_fsm_mgr_t *instance;
    // struct timeval tv1, tv2;

    // do_gettimeofday(&tv1);
    ret = mutex_lock_interruptible(&g_firmware.ctx_chg_lock);
    if (ret != 0) {
        pr_err("mutex lock failed, rc = %d\n", ret);
        return ret;
    }

	p_ctx = (acamera_context_ptr_t)&g_firmware.fw_ctx[ctx_id];
    instance = &(((acamera_context_ptr_t)acamera_get_ctx_ptr(last_ctx_id))->fsm_mgr);

    /* wait last ctx done */
    if (instance->reserved) { //dma writer on, isp offline next module
        if ((atomic_read(&g_firmware.dma_done) == 0
            || atomic_read(&g_firmware.frame_done) == 0)
            && _all_contexts_frame_counter_status() != 0) {
            pr_debug("=>isp is working now, next ctx id %d.\n", ctx_id);
            wait_event_timeout(wq_dma_done,
                atomic_read(&g_firmware.dma_done) && atomic_read(&g_firmware.frame_done),
                msecs_to_jiffies(30));
        }
        atomic_set(&g_firmware.dma_done, 0);
        atomic_set(&g_firmware.frame_done, 0);
    } else { //dma writer off, isp online next module
        if (atomic_read(&g_firmware.frame_done) == 0 && _all_contexts_frame_counter_status() != 0) {
            pr_debug("->isp is working now, next ctx id %d.\n", ctx_id);
            wait_event_timeout(wq_frame_end, atomic_read(&g_firmware.frame_done), msecs_to_jiffies(30));
        }
        atomic_set(&g_firmware.frame_done, 0);
    }

	pr_debug("start isp ctx switch, ctx_id %d\n", ctx_id);

    p_ctx->p_gfw->sif_isp_offline = 1;
    _ctx_chn_idx_update(ctx_id);

    if (GET_BYTE_V(g_firmware.iridix_ctrl_flag, 2) != 0) {
        _isp_iridix_ctrl();
    }

	isp_input_port_size_config(p_ctx->fsm_mgr.fsm_arr[FSM_ID_SENSOR]->p_fsm);
	ldc_set_ioctl(ctx_id, 0);
	dis_set_ioctl(ctx_id, 0);

	if (acamera_event_queue_empty(&p_ctx->fsm_mgr.event_queue)
        || acamera_event_queue_has_mask_event(&p_ctx->fsm_mgr.event_queue)) {
        dma_handle *dh = NULL;

		// these flags are used for sync of callbacks
		g_firmware.dma_flag_isp_config_completed = 0;
		g_firmware.dma_flag_isp_metering_completed = 0;
		g_firmware.handler_flag_interrupt_handle_completed = 0;

        if (p_ctx->content_side == SIDE_DDR) {
            ret = isp_ctx_prepare(p_ctx);
            if (ret != 0) {
                pr_err("outside ctx swap failed.\n");
                goto out;
            }
        }

        /* get one vb2 buffer config to dma writer */
        if (p_ctx->fsm_mgr.reserved) { //dma writer on
            // acamera_general_interrupt_hanlder( p_ctx, ACAMERA_IRQ_FRAME_WRITER_FR );
            dh = ((dma_writer_fsm_const_ptr_t)(p_ctx->fsm_mgr.fsm_arr[FSM_ID_DMA_WRITER]->p_fsm))->handle;
            dma_writer_configure_pipe(&dh->pipe[dma_fr]);
        }

        /*
            switch to ping/pong contexts for the next frame.
            ping space is unconfiged, we must config it at first frame,
            in case of different sensor comming.
        */
		if (acamera_isp_isp_global_ping_pong_config_select_read(0) == ISP_CONFIG_PONG
            || p_ctx->p_gfw->sw_frame_counter == 1) {
			acamera_isp_isp_global_mcu_ping_pong_config_select_write(0, ISP_CONFIG_PING);
			pr_debug("next is ping, DMA sram -> ping\n");
			isp_ctx_transfer(last_chn_id, cur_chn_id, ISP_CONFIG_PING);
		} else {
			acamera_isp_isp_global_mcu_ping_pong_config_select_write(0, ISP_CONFIG_PONG);
			pr_debug("next is pong, DMA sram -> pong\n");
			isp_ctx_transfer(last_chn_id, cur_chn_id, ISP_CONFIG_PONG);
		}
	} else {
        system_semaphore_raise( p_ctx->sem_evt_avail );
        pr_err("[s%d] previous frame events are not process done\n", ctx_id);
        // pr_info("sem cnt %d, ev_q head %d tail %d\n",
        //     ((struct semaphore *)p_ctx->sem_evt_avail)->count,
        //     p_ctx->fsm_mgr.event_queue.buf.head,
        //     p_ctx->fsm_mgr.event_queue.buf.tail);
        // acamera_event_queue_view(&p_ctx->fsm_mgr.event_queue);
        goto out;
	}

	if (!(g_firmware.dma_flag_isp_config_completed &&
		g_firmware.dma_flag_isp_metering_completed &&
		g_firmware.handler_flag_interrupt_handle_completed)) {

		wait_event_timeout(wq_dma_cfg_done,
            g_firmware.handler_flag_interrupt_handle_completed &&
            g_firmware.dma_flag_isp_config_completed &&
            g_firmware.dma_flag_isp_metering_completed, msecs_to_jiffies(30));
		pr_debug("ISP->SIF: wake up sif feed thread\n");
	} else
		pr_debug("ISP->SIF: do not need waiting, return to sif feed thread\n");

out:
    mutex_unlock(&g_firmware.ctx_chg_lock);

    // do_gettimeofday(&tv2);
    // pr_debug("cost %ld.%06ld\n", tv2.tv_sec - tv1.tv_sec, tv2.tv_usec - tv1.tv_usec);

	return 0;
}

static int ip_sts_dbg;
module_param(ip_sts_dbg, int, 0644);
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
    pr_debug("[s%d] IRQ MASK is 0x%x, frame id %d\n", cur_ctx_id, irq_mask, p_ctx->isp_frame_counter);

    if(irq_mask&0x8) {
        pr_err("[s%d] broken frame status = 0x%x", cur_ctx_id, acamera_isp_isp_global_monitor_broken_frame_status_read(0));
        // printk("active width min/max/sum/num = %d/%d/%d/%d", system_hw_read_32(0xb4),system_hw_read_32(0xb8),system_hw_read_32(0xbc),system_hw_read_32(0xc0));
        // printk("active high min/max/sum/num = %d/%d/%d/%d", system_hw_read_32(0xc4),system_hw_read_32(0xc8),system_hw_read_32(0xcc),system_hw_read_32(0xd0));
        // printk("hblank min/max/sum/num = %d/%d/%d/%d", system_hw_read_32(0xd4),system_hw_read_32(0xd8),system_hw_read_32(0xdc),system_hw_read_32(0xe0));
        // printk("vblank min/max/sum/num = %d/%d/%d/%d", system_hw_read_32(0xe4),system_hw_read_32(0xe8),system_hw_read_32(0xec),system_hw_read_32(0xf0));
	    // printk("input port w/h %x, ping w/h %x, pong w/h %x", system_hw_read_32(0x98L), system_hw_read_32(0x18e88L), system_hw_read_32(0x30e48L));
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
                if ( p_ctx->p_gfw->sif_isp_offline == 0 && irq_bit == ISP_INTERRUPT_EVENT_ISP_START_FRAME_START ) {
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
                    if ( acamera_event_queue_empty( &p_ctx->fsm_mgr.event_queue )
                        || acamera_event_queue_has_mask_event( &p_ctx->fsm_mgr.event_queue ) ) {
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
                            isp_ctx_transfer(0, 0, ISP_CONFIG_PING);
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
                            isp_ctx_transfer(0, 0, ISP_CONFIG_PONG);
                        }
                    } else {
                        pr_debug("sem cnt %d, ev_q head %d tail %d\n",
                            ((struct semaphore *)p_ctx->sem_evt_avail)->count,
                            p_ctx->fsm_mgr.event_queue.buf.head,
                            p_ctx->fsm_mgr.event_queue.buf.tail);
                        pr_err("prevous frame is not processing done, skip this frame\n");
                    } //if ( acamera_event_queue_empty( &p_ctx->fsm_mgr.event_queue ) )
                } else if ( irq_bit == ISP_INTERRUPT_EVENT_ISP_END_FRAME_END ) {
                    pr_debug("frame done, ctx id %d\n", cur_ctx_id);
                    if (ip_sts_dbg)
                        input_port_status();
                    if (p_ctx->p_gfw->sif_isp_offline) {
                        atomic_set(&g_firmware.frame_done, 1);
                        wake_up(&wq_frame_end);
                        if (p_ctx->fsm_mgr.reserved) //dma writer on
                            wake_up(&wq_dma_done);
                    }
                } else if ( irq_bit == ISP_INTERRUPT_EVENT_FR_Y_WRITE_DONE ) {
                    pr_debug("frame write to ddr done\n");
                    if (p_ctx->p_gfw->sif_isp_offline) {
                        atomic_set(&g_firmware.dma_done, 1);
                        wake_up(&wq_dma_done);
                    }
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

int isp_fw_process( void *data )
{
    uint8_t ctx_id = *((uint8_t *)data);
    acamera_context_ptr_t p_ctx = NULL;

    pr_debug( "isp_fw_process start" );

    if (ctx_id >= FIRMWARE_CONTEXT_NUMBER) {
        return -1;
    }

    p_ctx = ( acamera_context_ptr_t ) & ( g_firmware.fw_ctx[ctx_id] );

    while ( !kthread_should_stop() ) {
        acamera_fw_process( p_ctx );
        system_semaphore_wait( p_ctx->sem_evt_avail, 1 );
    }

    pr_debug( "isp_fw_process stop" );
    return 0;
}