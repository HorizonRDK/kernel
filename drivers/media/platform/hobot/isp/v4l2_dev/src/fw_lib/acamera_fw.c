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
#include <linux/kthread.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>

#include "acamera_fw.h"
#if ACAMERA_ISP_PROFILING
#include "acamera_profiler.h"
#endif

#include "acamera_isp_config.h"
#include "acamera_firmware_config.h"
#include "acamera_command_api.h"
#include "acamera_isp_core_nomem_settings.h"
#include "acamera_metering_stats_mem_config.h"
#include "system_timer.h"
#include "acamera_logger.h"
#include "acamera_sbus_api.h"
#include "sensor_init.h"
#include "sensor_fsm.h"
#include "isp_config_seq.h"
#include "system_stdlib.h"
#include "general_fsm.h"
#include "system_dma.h"
#include "hobot_isp_reg_dma.h"
#include "isp_ctxsv.h"

#if ISP_HAS_META_CB && defined( ISP_HAS_METADATA_FSM )
#include "metadata_api.h"
#endif

#if FW_HAS_CONTROL_CHANNEL
#include "acamera_ctrl_channel.h"
#endif

static const acam_reg_t **p_isp_data = SENSOR_ISP_SEQUENCE_DEFAULT;

extern void acamera_notify_evt_data_avail( acamera_context_t *p_ctx );
extern void acamera_update_cur_settings_to_isp( uint32_t fw_ctx_id );
extern void *acamera_get_ctx_ptr(uint32_t ctx_id);
static void init_stab( acamera_context_ptr_t p_ctx );
#if FW_USE_HOBOT_DMA
extern hobot_dma_t g_hobot_dma;
#endif

void acamera_load_isp_sequence( uintptr_t isp_base, const acam_reg_t **sequence, uint8_t num )
{
    acamera_sbus_t sbus;
    sbus.mask = SBUS_MASK_SAMPLE_32BITS | SBUS_MASK_SAMPLE_16BITS | SBUS_MASK_SAMPLE_8BITS | SBUS_MASK_ADDR_STEP_32BITS | SBUS_MASK_ADDR_32BITS;
    acamera_sbus_init( &sbus, sbus_isp );
    acamera_load_array_sequence( &sbus, isp_base, 0, sequence, num );
}


void acamera_load_sw_sequence( uintptr_t isp_base, const acam_reg_t **sequence, uint8_t num )
{
    acamera_sbus_t sbus;
    sbus.mask = SBUS_MASK_SAMPLE_32BITS | SBUS_MASK_SAMPLE_16BITS | SBUS_MASK_SAMPLE_8BITS | SBUS_MASK_ADDR_STEP_32BITS | SBUS_MASK_ADDR_32BITS;
    acamera_sbus_init( &sbus, sbus_isp_sw );
    acamera_load_array_sequence( &sbus, isp_base, 0, sequence, num );
}


#define IRQ_ID_UNDEFINED 0xFF

void acamera_fw_init( acamera_context_t *p_ctx )
{

#if ACAMERA_ISP_PROFILING
#if ACAMERA_ISP_PROFILING_INIT
    p_ctx->binit_profiler = 0;
    p_ctx->breport_profiler = 0;
#else
    p_ctx->binit_profiler = 0;
    p_ctx->breport_profiler = 0;
#endif
    p_ctx->start_profiling = 500; //start when gframe == 500
    p_ctx->stop_profiling = 1000; //stop  when gframe == 1000
#endif

    p_ctx->irq_flag = 1;

    p_ctx->fsm_mgr.p_ctx = p_ctx;
    p_ctx->fsm_mgr.ctx_id = p_ctx->context_id;
    p_ctx->fsm_mgr.isp_base = p_ctx->settings.isp_base;
    acamera_fsm_mgr_init( &p_ctx->fsm_mgr );

    p_ctx->irq_flag = 0;
    p_ctx->system_state = FW_RUN;
}

void acamera_fw_deinit( acamera_context_t *p_ctx )
{
    p_ctx->fsm_mgr.p_ctx = p_ctx;
    acamera_fsm_mgr_deinit( &p_ctx->fsm_mgr );
}

extern uint8_t isp_safe_start( uint32_t base );
extern uint8_t isp_safe_stop( uint32_t base );
extern void isp_input_port_size_config(sensor_fsm_ptr_t p_fsm);
extern void ips_set_isp_interrupt(bool enable);
extern acamera_firmware_t *acamera_get_firmware_ptr(void);
int acamera_fw_isp_start(int ctx_id)
{
	uint8_t rc = 0;
	acamera_context_t *p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(ctx_id);

    acamera_load_isp_sequence( 0, p_ctx->isp_sequence, SENSOR_ISP_SEQUENCE_DEFAULT_SETTINGS );

    isp_input_port_size_config(p_ctx->fsm_mgr.fsm_arr[FSM_ID_SENSOR]->p_fsm);

    rc = isp_safe_start(p_ctx->settings.isp_base);

	//return the interrupts
	acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_MASK_VECTOR );

    ips_set_isp_interrupt(1);

	acamera_fw_interrupts_enable( p_ctx );

	if (!rc)
		pr_info("done.\n");

	return rc;
}

int acamera_fw_isp_stop(int ctx_id)
{
	uint8_t rc = 0;
	acamera_context_t *p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(ctx_id);
    acamera_firmware_t *fw_ptr = acamera_get_firmware_ptr();

    ips_set_isp_interrupt(0);
	acamera_fw_interrupts_disable( p_ctx );

	//masked all interrupts
	acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_DISABLE_ALL_IRQ );

    rc = isp_safe_stop(p_ctx->settings.isp_base);

    fw_ptr->first_frame = 0;
    fw_ptr->sw_frame_counter = 0;
    fw_ptr->initialized = 0;
    fw_ptr->iridix_ctrl_flag = 0;
    fw_ptr->sif_isp_offline = 0;
    if (fw_ptr->cache_area != NULL) {
        pr_debug("free ddr ctx mem %p\n", p_ctx->p_gfw->cache_area);
        vfree((void *)p_ctx->p_gfw->cache_area);
        fw_ptr->cache_area = NULL;
    }

    if (fw_ptr->backup_context != NULL) {
        vfree((void *)p_ctx->p_gfw->backup_context);
        fw_ptr->backup_context = NULL;
    }

	if (!rc) {
		pr_info("done.\n");
    }

	return rc;
}

#if 0
extern void sensor_sw_init( sensor_fsm_ptr_t p_fsm );
void acamera_fw_error_routine( acamera_context_t *p_ctx, uint32_t irq_mask )
{
    int abort_flag = 0;
    int retry_time = 0;

    //masked all interrupts
    acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_DISABLE_ALL_IRQ );

retry_1:
    //safe stop
    acamera_isp_input_port_mode_request_write( p_ctx->settings.isp_base, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_STOP );

    // check whether the HW is stopped or not.
    uint32_t count = 0;
    while ( acamera_isp_input_port_mode_status_read( p_ctx->settings.isp_base ) != ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_STOP || acamera_isp_isp_global_monitor_fr_pipeline_busy_read( p_ctx->settings.isp_base ) ) {
        //cannot sleep use this delay
        do {
            count++;
        } while ( count % 32 != 0 );

        if ( ( count >> 5 ) > 50 ) {
	    abort_flag = 1;
            LOG( LOG_ERR, "stopping isp failed, timeout: %u.", (unsigned int)count * 1000 );
            break;
        }
    }
    if (abort_flag && retry_time < 3) {
	retry_time++;
	abort_flag = 0;
	goto retry_1;
    }

    if (retry_time >= 3) {
	printk("ISP CRITICAL ERROR\n");
	return;
    }

    acamera_isp_isp_global_global_fsm_reset_write( p_ctx->settings.isp_base, 1 );
    acamera_isp_isp_global_global_fsm_reset_write( p_ctx->settings.isp_base, 0 );

    //skip 4 isp_done irq
    //TODO:

retry_2:
    //clear alarms
    count = 0;
    retry_time = 0;
    acamera_isp_isp_global_monitor_broken_frame_error_clear_write( p_ctx->settings.isp_base, 1 );
    acamera_isp_isp_global_monitor_context_error_clr_write( p_ctx->settings.isp_base, 1 );
    acamera_isp_isp_global_monitor_output_dma_clr_alarm_write( p_ctx->settings.isp_base, 1 );
    acamera_isp_isp_global_monitor_temper_dma_clr_alarm_write( p_ctx->settings.isp_base, 1 );
    while ( system_hw_read_32(0x54) != 0) {
        //cannot sleep use this delay
        do {
            count++;
        } while ( count % 32 != 0 );

        if ( ( count >> 5 ) > 50 ) {
	    abort_flag = 1;
            LOG( LOG_ERR, "alarm and error clear failed, timeout: %u.", (unsigned int)count * 1000 );
            break;
        }
    }
    if (abort_flag && retry_time < 3) {
	retry_time++;
	abort_flag = 0;
	goto retry_2;
    }
    if (retry_time >= 3) {
	printk("ISP CRITICAL ERROR\n");
	return;
    }
    acamera_isp_isp_global_monitor_broken_frame_error_clear_write( p_ctx->settings.isp_base, 0 );
    acamera_isp_isp_global_monitor_context_error_clr_write( p_ctx->settings.isp_base, 0 );
    acamera_isp_isp_global_monitor_output_dma_clr_alarm_write( p_ctx->settings.isp_base, 0 );
    acamera_isp_isp_global_monitor_temper_dma_clr_alarm_write( p_ctx->settings.isp_base, 0 );

    //reconfig isp configuration space
    acamera_load_isp_sequence( 0, p_ctx->isp_sequence, SENSOR_ISP_SEQUENCE_DEFAULT_SETTINGS );

    //read ping/pong select state, sync up with software state

    //safe start
    acamera_isp_input_port_mode_request_write( p_ctx->settings.isp_base, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_START );

    //config isp hw ping/pong space
    acamera_update_cur_settings_to_isp(p_ctx->context_id);

    //config input port
    sensor_sw_init(p_ctx->fsm_mgr.fsm_arr[FSM_ID_SENSOR]->p_fsm);

    //return the interrupts
    acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_MASK_VECTOR );

    LOG( LOG_ERR, "starting isp from error routine" );
}
#else
void acamera_fw_error_routine( acamera_context_t *p_ctx, uint32_t irq_mask )
{
    //masked all interrupts
    acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_DISABLE_ALL_IRQ );
    //safe stop
    acamera_isp_input_port_mode_request_write( p_ctx->settings.isp_base, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_STOP );

    // check whether the HW is stopped or not.
    uint32_t count = 0;
    while ( acamera_isp_input_port_mode_status_read( p_ctx->settings.isp_base ) != ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_STOP || acamera_isp_isp_global_monitor_fr_pipeline_busy_read( p_ctx->settings.isp_base ) ) {
        //cannot sleep use this delay
        do {
            count++;
        } while ( count % 32 != 0 );

        if ( ( count >> 5 ) > 50 ) {
            LOG( LOG_ERR, "stopping isp failed, timeout: %u.", (unsigned int)count * 1000 );
            break;
        }
    }

    acamera_isp_isp_global_global_fsm_reset_write( p_ctx->settings.isp_base, 1 );
    acamera_isp_isp_global_global_fsm_reset_write( p_ctx->settings.isp_base, 0 );

    //return the interrupts
    acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_MASK_VECTOR );

    acamera_isp_input_port_mode_request_write( p_ctx->settings.isp_base, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_START );

    LOG( LOG_NOTICE, "starting isp from error" );
}
#endif

void acamera_fw_process( acamera_context_t *p_ctx )
{
#if ACAMERA_ISP_PROFILING
    if ( ( p_ctx->frame >= p_ctx->start_profiling ) && ( !p_ctx->binit_profiler ) ) {
        acamera_profiler_init();
        p_ctx->binit_profiler = 1;
    }
#endif
    if ( ( p_ctx->system_state == FW_RUN ) ) //need to capture on firmware freeze
    {
        // firmware not frozen
        // 0 means handle all the events and then return.
        acamera_fsm_mgr_process_events( &p_ctx->fsm_mgr, 0 );
    }
#if ACAMERA_ISP_PROFILING
    if ( ( p_ctx->frame >= p_ctx->stop_profiling ) && ( !p_ctx->breport_profiler ) ) {
        acamera_profiler_report();
        p_ctx->breport_profiler = 1;
    }
#endif
}

void acamera_fw_raise_event( acamera_context_t *p_ctx, event_id_t event_id )
{ //dma writer events should be passed for the capture on freeze requirement
    if ( p_ctx->stab.global_freeze_firmware == 0 || event_id == event_id_new_frame
#if defined( ISP_HAS_DMA_WRITER_FSM )
         || event_id == event_id_frame_buffer_fr_ready || event_id == event_id_frame_buffer_ds_ready || event_id == event_id_frame_buffer_metadata
	 || event_id == event_id_frame_config || event_id == event_id_frame_done || event_id == event_id_frame_error
#endif
#if defined( ISP_HAS_METADATA_FSM )
         || event_id == event_id_metadata_ready || event_id == event_id_metadata_update
#endif
#if defined( ISP_HAS_BSP_TEST_FSM )
         || event_id == event_id_bsp_test_interrupt_finished
#endif
         ) {
        acamera_event_queue_push( &p_ctx->fsm_mgr.event_queue, (int)( event_id ) );
        acamera_notify_evt_data_avail(p_ctx);
    }
}

void acamera_fsm_mgr_raise_event( acamera_fsm_mgr_t *p_fsm_mgr, event_id_t event_id )
{ //dma writer events should be passed for the capture on freeze requirement
    if ( p_fsm_mgr->p_ctx->stab.global_freeze_firmware == 0 || event_id == event_id_new_frame
#if defined( ISP_HAS_DMA_WRITER_FSM )
         || event_id == event_id_frame_buffer_fr_ready || event_id == event_id_frame_buffer_ds_ready || event_id == event_id_frame_buffer_metadata
#endif
#if defined( ISP_HAS_BSP_TEST_FSM )
         || event_id == event_id_bsp_test_interrupt_finished
#endif
         ) {
        acamera_event_queue_push( &( p_fsm_mgr->event_queue ), (int)( event_id ) );

        acamera_notify_evt_data_avail(p_fsm_mgr->p_ctx);
    }
}


int32_t acamera_update_calibration_set( acamera_context_ptr_t p_ctx )
{
    int32_t result = 0;
    void *sensor_arg = 0;
	uint32_t sensor_type = 0;
    if ( p_ctx->settings.get_calibrations != NULL ) {
        {
            const sensor_param_t *param = NULL;
            acamera_fsm_mgr_get_param( &p_ctx->fsm_mgr, FSM_PARAM_GET_SENSOR_PARAM, NULL, 0, &param, sizeof( param ) );

            uint32_t cur_mode = param->mode;
            if ( cur_mode < param->modes_num ) {
                sensor_arg = &( param->modes_table[cur_mode] );
            }
		sensor_type = param->sensor_type;
        }
        if ( p_ctx->settings.get_calibrations( p_ctx->context_id, sensor_arg, &p_ctx->acameraCalibrations, sensor_type ) != 0 ) {
            LOG( LOG_ERR, "Failed to get calibration set for. Fatal error" );
        }

#if defined( ISP_HAS_GENERAL_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_RELOAD_CALIBRATION, NULL, 0 );
#endif

// Update some FSMs variables which depends on calibration data.
#if defined( ISP_HAS_AE_BALANCED_FSM ) || defined( ISP_HAS_AE_MANUAL_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_AE_INIT, NULL, 0 );
#endif

#if defined( ISP_HAS_IRIDIX_FSM ) || defined( ISP_HAS_IRIDIX_HIST_FSM ) || defined( ISP_HAS_IRIDIX_MANUAL_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_IRIDIX_INIT, NULL, 0 );
#endif

#if defined( ISP_HAS_COLOR_MATRIX_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_CCM_CHANGE, NULL, 0 );
#endif

#if defined( ISP_HAS_SBUF_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_SBUF_CALIBRATION_UPDATE, NULL, 0 );
#endif
    } else {
        LOG( LOG_ERR, "Calibration callback is null. Failed to get calibrations" );
        result = -1;
    }

    return result;
}


int32_t acamera_init_calibrations( acamera_context_ptr_t p_ctx )
{
    int32_t result = 0;
    void *sensor_arg = 0;
    uint32_t sensor_type = 0;
#ifdef SENSOR_ISP_SEQUENCE_DEFAULT_FULL
    acamera_load_isp_sequence( p_ctx->settings.isp_base, p_ctx->isp_sequence, SENSOR_ISP_SEQUENCE_DEFAULT_FULL );
#endif

    // if "p_ctx->initialized" is 1, that means we are changing the preset and wdr_mode,
    // we need to update the calibration data and update some FSM variables which
    // depends on calibration data.
    if ( p_ctx->initialized == 1 ) {
        acamera_update_calibration_set( p_ctx );
    } else {
        if ( p_ctx->settings.get_calibrations != NULL ) {
            const sensor_param_t *param = NULL;
            acamera_fsm_mgr_get_param( &p_ctx->fsm_mgr, FSM_PARAM_GET_SENSOR_PARAM, NULL, 0, &param, sizeof( param ) );

            uint32_t cur_mode = param->mode;
            if ( cur_mode < param->modes_num ) {
                sensor_arg = &( param->modes_table[cur_mode] );
            }
		sensor_type = param->sensor_type;
            if ( p_ctx->settings.get_calibrations( p_ctx->context_id, sensor_arg, &p_ctx->acameraCalibrations, sensor_type ) != 0 ) {
                LOG( LOG_ERR, "Failed to get calibration set for. Fatal error" );
            }
        } else {
            LOG( LOG_ERR, "Calibration callback is null. Failed to get calibrations" );
            result = -1;
        }
    }
    return result;
}


#if ISP_HAS_META_CB && defined( ISP_HAS_METADATA_FSM )
static void internal_callback_metadata( void *ctx, const firmware_metadata_t *fw_metadata )
{
    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)ctx;

    if ( p_ctx->settings.callback_meta != NULL ) {
        p_ctx->settings.callback_meta( p_ctx->context_id, fw_metadata );
    }
}
#endif

static void configure_all_frame_buffers( acamera_context_ptr_t p_ctx )
{

#if ISP_HAS_WDR_FRAME_BUFFER
    acamera_isp_frame_stitch_frame_buffer_frame_write_on_write( p_ctx->settings.isp_base, 0 );
    aframe_t *frame_stitch_frames = p_ctx->settings.fs_frames;
    uint32_t frame_stitch_frames_num = p_ctx->settings.fs_frames_number;
    if ( frame_stitch_frames != NULL && frame_stitch_frames_num != 0 ) {
        if ( frame_stitch_frames_num == 1 ) {
            LOG( LOG_INFO, "Only one output buffer will be used for frame_stitch." );
            acamera_isp_frame_stitch_frame_buffer_bank0_base_write( p_ctx->settings.isp_base, frame_stitch_frames[0].address );
            acamera_isp_frame_stitch_frame_buffer_bank1_base_write( p_ctx->settings.isp_base, frame_stitch_frames[0].address );
            acamera_isp_frame_stitch_frame_buffer_line_offset_write( p_ctx->settings.isp_base, frame_stitch_frames[0].line_offset );
        } else {
            // double buffering is enabled
            acamera_isp_frame_stitch_frame_buffer_bank0_base_write( p_ctx->settings.isp_base, frame_stitch_frames[0].address );
            acamera_isp_frame_stitch_frame_buffer_bank1_base_write( p_ctx->settings.isp_base, frame_stitch_frames[1].address );
            acamera_isp_frame_stitch_frame_buffer_line_offset_write( p_ctx->settings.isp_base, frame_stitch_frames[0].line_offset );
        }

        acamera_isp_frame_stitch_frame_buffer_frame_write_on_write( p_ctx->settings.isp_base, 1 );
        acamera_isp_frame_stitch_frame_buffer_axi_port_enable_write( p_ctx->settings.isp_base, 1 );

    } else {
        acamera_isp_frame_stitch_frame_buffer_frame_write_on_write( p_ctx->settings.isp_base, 0 );
        acamera_isp_frame_stitch_frame_buffer_axi_port_enable_write( p_ctx->settings.isp_base, 0 );
        LOG( LOG_ERR, "No output buffers for frame_stitch block provided in settings. frame_stitch wdr buffer is disabled" );
    }
#endif


#if ISP_HAS_META_CB && defined( ISP_HAS_METADATA_FSM )
    acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_META_REGISTER_CB, internal_callback_metadata, sizeof( metadata_callback_t ) );
#endif

#if defined( ISP_HAS_DMA_WRITER_FSM )

    fsm_param_dma_pipe_setting_t pipe_fr;

    pipe_fr.pipe_id = dma_fr;
    acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_DMA_PIPE_SETTING, &pipe_fr, sizeof( pipe_fr ) );

    acamera_isp_fr_dma_writer_format_write( p_ctx->settings.isp_base, FW_OUTPUT_FORMAT );
    acamera_isp_fr_uv_dma_writer_format_write( p_ctx->settings.isp_base, FW_OUTPUT_FORMAT_SECONDARY );
#endif


#if ISP_HAS_DS1 && defined( ISP_HAS_DMA_WRITER_FSM )

    fsm_param_dma_pipe_setting_t pipe_ds1;

    pipe_ds1.pipe_id = dma_ds1;
    acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_DMA_PIPE_SETTING, &pipe_ds1, sizeof( pipe_ds1 ) );

    acamera_isp_ds1_dma_writer_format_write( p_ctx->settings.isp_base, FW_OUTPUT_FORMAT );
    acamera_isp_ds1_uv_dma_writer_format_write( p_ctx->settings.isp_base, FW_OUTPUT_FORMAT_SECONDARY );

#endif
}

static void init_stab( acamera_context_ptr_t p_ctx )
{
    p_ctx->stab.global_freeze_firmware = 0;
    p_ctx->stab.global_manual_exposure = 0;

    p_ctx->stab.global_manual_iridix = 0;
    p_ctx->stab.global_manual_sinter = 0;
    p_ctx->stab.global_manual_temper = 0;
    p_ctx->stab.global_manual_awb = 0;
    p_ctx->stab.global_manual_ccm = 0;
    p_ctx->stab.global_manual_saturation = 0;
    p_ctx->stab.global_manual_auto_level = 0;
    p_ctx->stab.global_manual_frame_stitch = 0;
    p_ctx->stab.global_manual_raw_frontend = 0;
    p_ctx->stab.global_manual_black_level = 0;
    p_ctx->stab.global_manual_shading = 0;
    p_ctx->stab.global_manual_demosaic = 0;
    p_ctx->stab.global_manual_cnr = 0;
    p_ctx->stab.global_manual_sharpen = 0;

    p_ctx->stab.global_exposure = 0;
    p_ctx->stab.global_long_integration_time = 0;
    p_ctx->stab.global_short_integration_time = 0;
    p_ctx->stab.global_manual_exposure_ratio = SYSTEM_MANUAL_EXPOSURE_RATIO_DEFAULT;
    p_ctx->stab.global_exposure_ratio = SYSTEM_EXPOSURE_RATIO_DEFAULT;

    p_ctx->stab.global_maximum_iridix_strength = SYSTEM_MAXIMUM_IRIDIX_STRENGTH_DEFAULT;
    p_ctx->stab.global_minimum_iridix_strength = SYSTEM_MINIMUM_IRIDIX_STRENGTH_DEFAULT;
    p_ctx->stab.global_iridix_strength_target = 0;
    p_ctx->stab.global_sinter_threshold_target = 0;
    p_ctx->stab.global_temper_threshold_target = 0;
    p_ctx->stab.global_awb_red_gain = 500;
    p_ctx->stab.global_awb_blue_gain = 400;
    p_ctx->stab.global_awb_green_even_gain = 256;
    p_ctx->stab.global_awb_green_odd_gain = 256;

    p_ctx->stab.global_ccm_matrix[0] = 0x0100;
    p_ctx->stab.global_ccm_matrix[1] = 0x0000;
    p_ctx->stab.global_ccm_matrix[2] = 0x0000;
    p_ctx->stab.global_ccm_matrix[3] = 0x0000;
    p_ctx->stab.global_ccm_matrix[4] = 0x0100;
    p_ctx->stab.global_ccm_matrix[5] = 0x0000;
    p_ctx->stab.global_ccm_matrix[6] = 0x0000;
    p_ctx->stab.global_ccm_matrix[7] = 0x0000;
    p_ctx->stab.global_ccm_matrix[8] = 0x0100;

    p_ctx->stab.global_saturation_target = 0;
    p_ctx->stab.global_ae_compensation = SYSTEM_AE_COMPENSATION_DEFAULT;
    p_ctx->stab.global_calibrate_bad_pixels = 0;
    p_ctx->stab.global_dynamic_gamma_enable = 0;

    isp_ctx_done_queue_clear(p_ctx->context_id);
    p_ctx->isp_ctxsv_on = 0;
    p_ctx->isp_ae_stats_on = 0;
    p_ctx->isp_awb_stats_on = 0;
    p_ctx->isp_frame_counter = 0;

    ((general_fsm_ptr_t)(p_ctx->fsm_mgr.fsm_arr[FSM_ID_GENERAL]->p_fsm))->cnt_for_temper = 0;
}

extern void *get_system_ctx_ptr( void );
extern int acamera_all_hw_contexts_inited(void);
extern int isp_fw_process( void *data );
int32_t acamera_init_context( acamera_context_t *p_ctx, acamera_settings *settings, acamera_firmware_t *g_fw )
{
    int32_t result = 0;
    char name[16] = {0};
    struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
    // keep the context pointer for debug purposes
    p_ctx->context_ref = (uint32_t *)p_ctx;
    p_ctx->p_gfw = g_fw;
    p_ctx->dma_chn_idx = -1;

    pr_info("ctx_id %d +\n", p_ctx->context_id);

    result = mutex_lock_interruptible(&p_ctx->p_gfw->ctx_chg_lock);
    if (result != 0) {
        pr_err("mutex lock failed, rc = %d\n", result);
        return result;
    }

    if (acamera_all_hw_contexts_inited()) {
        p_ctx->content_side = SIDE_DDR;
        p_ctx->sw_reg_map.isp_sw_config_map = vmalloc(HOBOT_DMA_SRAM_ONE_ZONE);
        if (p_ctx->sw_reg_map.isp_sw_config_map == NULL) {
            pr_err("external ctx area alloc failed.\n");
            mutex_unlock(&p_ctx->p_gfw->ctx_chg_lock);
            return -1;
        }
        pr_debug("alloc ddr ctx mem vaddr %p\n", p_ctx->sw_reg_map.isp_sw_config_map);
        memset((void *)p_ctx->sw_reg_map.isp_sw_config_map, 0, HOBOT_DMA_SRAM_ONE_ZONE);
        p_ctx->sw_reg_map.isp_sw_phy_addr = 0;
        if (p_ctx->p_gfw->backup_context) {
            memcpy((void *)p_ctx->sw_reg_map.isp_sw_config_map,
                (void *)p_ctx->p_gfw->backup_context, HOBOT_DMA_SRAM_ONE_ZONE);
        }
        if (p_ctx->p_gfw->cache_area == NULL) {
            p_ctx->p_gfw->cache_area = vmalloc(HOBOT_DMA_SRAM_ONE_ZONE);
            if (p_ctx->p_gfw->cache_area == NULL) {
                pr_err("cache area alloc failed.\n");
                mutex_unlock(&p_ctx->p_gfw->ctx_chg_lock);
                return -1;
            }
            memset((void *)p_ctx->p_gfw->cache_area, 0, HOBOT_DMA_SRAM_ONE_ZONE);
            pr_debug("alloc cache area vaddr %p\n", p_ctx->p_gfw->cache_area);
        }
    } else {
        int i = 0;
        for (i = 0; i < HW_CONTEXT_NUMBER; i++) {
            if (!test_bit(i, &g_fw->dma_chn_bitmap)) {
                p_ctx->dma_chn_idx = i;
                set_bit(i, &g_fw->dma_chn_bitmap);
                break;
            }
        }
        if (p_ctx->dma_chn_idx >= 0 && p_ctx->dma_chn_idx < HW_CONTEXT_NUMBER) {
#if FW_USE_HOBOT_DMA
            p_ctx->sw_reg_map.isp_sw_config_map = system_sw_get_dma_addr(p_ctx->dma_chn_idx);
            if (p_ctx->sw_reg_map.isp_sw_config_map == NULL) {
                pr_err("idma is not remap, get addr failed.\n");
                mutex_unlock(&p_ctx->p_gfw->ctx_chg_lock);
                return -1;
            }
#endif
            p_ctx->content_side = SIDE_SRAM;

            //init one ctx's sram, copy from isp hw(first time init only)
            if (p_ctx->p_gfw->initialized == 0) {
                pr_info("copy data from isp hw to sram %d\n", p_ctx->dma_chn_idx);
                system_dma_copy_sg(g_fw->dma_chan_isp_config,
                        ISP_CONFIG_PING, SYS_DMA_FROM_DEVICE, 0, p_ctx->dma_chn_idx);
#if FW_USE_HOBOT_DMA
                isp_idma_start_transfer(&g_hobot_dma);
                system_dma_wait_done(g_fw->dma_chan_isp_config);
#endif
                //save for other ctx initialized later
                p_ctx->p_gfw->backup_context = vmalloc(HOBOT_DMA_SRAM_ONE_ZONE);
                if (p_ctx->p_gfw->backup_context == NULL) {
                    pr_err("backup_context alloc failed.\n");
                    mutex_unlock(&p_ctx->p_gfw->ctx_chg_lock);
                    return -1;
                }
                memset((void *)p_ctx->p_gfw->backup_context, 0, HOBOT_DMA_SRAM_ONE_ZONE);
                memcpy_fromio((void *)p_ctx->p_gfw->backup_context,
                    (void *)p_ctx->sw_reg_map.isp_sw_config_map, HOBOT_DMA_SRAM_ONE_ZONE);
            } else {
                if (p_ctx->p_gfw->backup_context) {
                    pr_info("copy data from ddr to sram %d\n", p_ctx->dma_chn_idx);
                    memcpy_toio((void *)p_ctx->sw_reg_map.isp_sw_config_map,
                        (void *)p_ctx->p_gfw->backup_context, HOBOT_DMA_SRAM_ONE_ZONE);
                }
            }
        }
    }

    memset(&p_ctx->sts, 0, sizeof(p_ctx->sts));
    mutex_unlock(&p_ctx->p_gfw->ctx_chg_lock);

    pr_info("side %s, chn %d, paddr %p\n",
        p_ctx->content_side ? "ddr" : "sram",
        p_ctx->dma_chn_idx, p_ctx->sw_reg_map.isp_sw_config_map);

    if ( p_ctx->sw_reg_map.isp_sw_config_map != NULL ) {
        // copy settings
        system_memcpy( (void *)&p_ctx->settings, (void *)settings, sizeof( acamera_settings ) );

        p_ctx->settings.isp_base = (uintptr_t)p_ctx->sw_reg_map.isp_sw_config_map;

        // each context is initialized to the default state
        p_ctx->isp_sequence = p_isp_data;

#if defined( SENSOR_ISP_SEQUENCE_DEFAULT_SETTINGS_CONTEXT )
        acamera_load_sw_sequence( p_ctx->settings.isp_base, p_ctx->isp_sequence, SENSOR_ISP_SEQUENCE_DEFAULT_SETTINGS_CONTEXT );
#endif

#if ISP_DMA_RAW_CAPTURE
        dma_raw_capture_init( g_fw );
#endif

        // reset frame counters
        p_ctx->isp_frame_counter_raw = 0;

        if (p_ctx->initialized == 0) {
            acamera_fw_init( p_ctx );
            configure_all_frame_buffers( p_ctx );
        }
        init_stab( p_ctx );

        sprintf(name, "isp_evt%d", p_ctx->context_id);
        p_ctx->evt_thread = kthread_run( isp_fw_process, (void *)&p_ctx->context_id, name );
        sched_setscheduler_nocheck(p_ctx->evt_thread, SCHED_FIFO, &param);
        // set_cpus_allowed_ptr(p_ctx->evt_thread, cpumask_of(p_ctx->context_id % 4));

        if (p_ctx->dma_chn_idx >= 0 && p_ctx->dma_chn_idx < HW_CONTEXT_NUMBER) {
            acamera_isp_iridix_context_no_write(p_ctx->settings.isp_base, p_ctx->dma_chn_idx);
        } else {
            acamera_isp_top_bypass_iridix_write(p_ctx->settings.isp_base, 1);
            acamera_isp_iridix_enable_write(p_ctx->settings.isp_base, 0);
        }

        //acamera_isp_input_port_mode_request_write( p_ctx->settings.isp_base, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_START );

        p_ctx->initialized = 1;
        pr_info("ctx_id %d -\n", p_ctx->context_id);
    } else {
        result = -1;
        LOG( LOG_ERR, "Failed to allocate memory for ISP config context" );
    }

    return result;
}

void acamera_deinit_context( acamera_context_t *p_ctx )
{
    int rc = 0;

    rc = mutex_lock_interruptible(&p_ctx->p_gfw->ctx_chg_lock);
    if (rc != 0) {
        pr_err("ctx_chg_lock mutex lock failed, rc = %d\n", rc);
        return;
    }

    if (p_ctx->content_side == SIDE_DDR && p_ctx->sw_reg_map.isp_sw_config_map) {
        pr_debug("ctx_id %d, free ddr ctx mem\n", p_ctx->context_id);
        vfree((void *)p_ctx->sw_reg_map.isp_sw_config_map);
        p_ctx->sw_reg_map.isp_sw_config_map = NULL;
    }

    // clear all contexts state
    p_ctx->fsm_mgr.reserved = 0;
    p_ctx->initialized = 0;
    p_ctx->system_state = FW_PAUSE;
    if (p_ctx->dma_chn_idx >= 0 && p_ctx->dma_chn_idx < HW_CONTEXT_NUMBER) {
        clear_bit(p_ctx->dma_chn_idx, &(p_ctx->p_gfw->dma_chn_bitmap));
        p_ctx->dma_chn_idx = -1;
    }

    acamera_fw_deinit( p_ctx );
    mutex_unlock(&p_ctx->p_gfw->ctx_chg_lock);
}

void acamera_general_interrupt_hanlder( acamera_context_ptr_t p_ctx, uint8_t event )
{
#ifdef CALIBRATION_INTERRUPTS
    uint32_t *interrupt_counter = _GET_UINT_PTR( p_ctx, CALIBRATION_INTERRUPTS );
    interrupt_counter[event]++;
#endif


    p_ctx->irq_flag++;

    if ( event == ACAMERA_IRQ_FRAME_START ) {
        p_ctx->frame++;
    }

    if ( event == ACAMERA_IRQ_FRAME_END ) {
#if ISP_DMA_RAW_CAPTURE
        p_ctx->isp_frame_counter_raw++;
#endif

// check frame counter sync when there is raw callback
#if ISP_HAS_RAW_CB
        if ( p_ctx->isp_frame_counter_raw != p_ctx->isp_frame_counter ) {
            LOG( LOG_DEBUG, "Sync frame counter : raw = %d, meta = %d",
                 (int)p_ctx->isp_frame_counter_raw, (int)p_ctx->isp_frame_counter );
            p_ctx->isp_frame_counter = p_ctx->isp_frame_counter_raw;
        }
#endif

        acamera_fw_raise_event( p_ctx, event_id_frame_end );

#if defined( ACAMERA_ISP_PROFILING ) && ( ACAMERA_ISP_PROFILING == 1 )
        acamera_profiler_new_frame();
#endif
    }

    if ( ( p_ctx->stab.global_freeze_firmware == 0 )
#if defined( ISP_HAS_DMA_WRITER_FSM )
         || ( event == ACAMERA_IRQ_FRAME_WRITER_FR ) // process interrupts for frame buffer anyway (otherwise picture will be frozen)
         || ( event == ACAMERA_IRQ_FRAME_WRITER_DS ) // process interrupts for frame buffer anyway (otherwise picture will be frozen)
	 || ( event == ACAMERA_IRQ_FRAME_WRITER_FR_DONE )
	 || ( event == ACAMERA_IRQ_FRAME_ERROR )
#endif
#if defined( ISP_HAS_CMOS_FSM )
         || ( event == ACAMERA_IRQ_FRAME_START ) // process interrupts for FS anyway (otherwise exposure will be only short)
#endif
#if defined( ISP_HAS_BSP_TEST_FSM )
         || event == ACAMERA_IRQ_FRAME_END || event == ACAMERA_IRQ_FRAME_START
#endif
         ) {
        // firmware not frozen
        acamera_fsm_mgr_process_interrupt( &p_ctx->fsm_mgr, event );
    }

    p_ctx->irq_flag--;
}
