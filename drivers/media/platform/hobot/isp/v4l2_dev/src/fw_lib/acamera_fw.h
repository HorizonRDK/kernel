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

#if !defined( __ACAMERA_FW_H__ )
#define __ACAMERA_FW_H__

#include <linux/ratelimit.h>

#include "acamera_types.h"
#include "acamera_logger.h"
#include "acamera_calibrations.h"
#include "system_interrupts.h"
#include "system_semaphore.h"
#include "acamera_firmware_api.h"
#include "acamera_isp_core_nomem_settings.h"
#include "acamera_firmware_config.h"
#include "acamera_isp_config.h"


#define FW_PAUSE 0
#define FW_RUN 1

#define ISP_EVT_PRIORITY  38

#define ALIGN_UP(a, size)   ((a+size-1) & (~(size-1)))
#define ALIGN_UP16(d)   ALIGN_UP(d, 16)

struct _acamera_fsm_mgr_t;
struct _acamera_context_t;
struct _dma_raw_capture_t;
struct _acamera_firmware_t;
struct _isp_info_t;
typedef struct _acamera_fsm_mgr_t acamera_fsm_mgr_t;
typedef struct _acamera_context_t acamera_context_t;
typedef struct _acamera_context_t *acamera_context_ptr_t;
typedef struct _dma_raw_capture_t dma_raw_capture_t;
typedef struct _acamera_firmware_t acamera_firmware_t;
typedef struct _isp_info_t isp_info_t;

struct _isp_info_t {
    uint8_t state;
};

struct time_record_s {
	int index;
	struct timeval time[2];
};
#include "acamera_fsm_mgr.h"
#include "fsm_util.h"
#include "fsm_param.h"
#include "sensor_init.h"
#include "acamera_isp_config.h"


#if ISP_DMA_RAW_CAPTURE
#include "dma_raw_capture.h"
#endif

typedef struct _sytem_tab {
    uint8_t global_freeze_firmware;
    uint8_t global_manual_exposure;
    uint8_t global_manual_exposure_ratio;

    uint8_t global_manual_iridix;
    uint8_t global_manual_sinter;
    uint8_t global_manual_temper;
    uint8_t global_manual_awb;
    uint8_t global_manual_ccm;
    uint8_t global_manual_saturation;
    uint8_t global_manual_auto_level;
    uint8_t global_manual_frame_stitch;
    uint8_t global_manual_raw_frontend;
    uint8_t global_manual_black_level;
    uint8_t global_manual_shading;
    uint8_t global_manual_demosaic;
    uint8_t global_manual_cnr;
    uint8_t global_manual_sharpen;

    uint32_t global_exposure;
    uint32_t global_long_integration_time;
    uint32_t global_short_integration_time;
    uint8_t global_exposure_ratio;


    uint8_t global_iridix_strength_target;
    uint8_t global_maximum_iridix_strength;
    uint8_t global_minimum_iridix_strength;
    uint8_t global_sinter_threshold_target;
    uint8_t global_temper_threshold_target;
    uint16_t global_awb_red_gain;
    uint16_t global_awb_green_even_gain;
    uint16_t global_awb_green_odd_gain;
    uint16_t global_awb_blue_gain;
    int16_t global_ccm_matrix[9];
    uint8_t global_saturation_target;

    uint8_t global_ae_compensation;
    uint8_t global_calibrate_bad_pixels;
    uint32_t global_info_preset_num;
} system_tab;

typedef struct _acamera_isp_sw_regs_map {
    volatile uint8_t *isp_sw_config_map;
    uint32_t isp_sw_phy_addr;
} acamera_isp_sw_regs_map;

enum {
    SIDE_SRAM = 0,
    SIDE_DDR = 1,
};

typedef struct _input_port_t {
    uint16_t xtotal;
    uint16_t ytotal;
    uint16_t xoffset;
    uint16_t yoffset;
} input_port_t;

typedef struct _isp_status_t {
    uint32_t fs_irq_cnt;
    uint32_t fe_irq_cnt;
    uint32_t frame_write_done_irq_cnt;

    uint32_t broken_frame;
    uint32_t dma_error;
    uint32_t frame_collision;
    uint32_t watchdog_timeout;
    uint32_t context_manage_error;
    uint32_t temper_lsb_dma_drop;
    uint32_t temper_msb_dma_drop;
    uint32_t fr_y_dma_drop;
    uint32_t fr_uv_dma_drop;
    uint32_t evt_process_drop;
	uint32_t ispctx_dma_error;

    uint32_t qbuf_cnt;
    uint32_t dqbuf_cnt;
    uint32_t free_to_busy_cnt;
    uint32_t free_to_busy_failed_cnt;
    uint32_t busy_to_done_cnt;
    uint32_t busy_to_done_failed_cnt;
#ifdef CONFIG_HOBOT_DIAG
	uint32_t ctx_id;
	atomic_t diag_state;
#endif
} isp_status_t;

struct _acamera_context_t {
    uint32_t irq_flag;

    // profiling routines
    uint32_t binit_profiler;
    uint32_t breport_profiler;
    uint32_t start_profiling;
    uint32_t stop_profiling;
    uint32_t frame;

    uint8_t system_state;
    uint8_t initialized;
    uint8_t hflip;
    uint8_t vflip;
    input_port_t inport;

    acamera_fsm_mgr_t fsm_mgr;
    acamera_firmware_t *p_gfw;

    // current calibration set
    ACameraCalibrations acameraCalibrations;

    // global settings which can be shared through fsms
    system_tab stab;

    uint32_t context_id;
    uint32_t *context_ref;

    // context settings
    acamera_settings settings;

    const acam_reg_t **isp_sequence;

    int32_t isp_sequence_default;
    uint8_t out_format;

    /* frame counters */
    uint32_t isp_frame_counter_raw; // frame counter for raw callback
    uint32_t isp_frame_counter;     // frame counter for frame / metadata callbacks
    uint64_t timestamps;
    struct timeval tv;

    // isp context save on/off
    uint8_t isp_ctxsv_on;
    uint8_t isp_awb_stats_on;
    uint8_t isp_ae_stats_on;
    uint8_t isp_af_stats_on;
    uint8_t isp_ae_5bin_stats_on;
    uint8_t isp_lumvar_stats_on;

    uint8_t antiflicker_enable;

    uint8_t content_side;
    int dma_chn_idx;
	int iridix_chn_idx;
	int isp_sensor_mode;

    semaphore_t sem_evt_avail;
    struct task_struct *evt_thread;

    isp_status_t sts;

    acamera_isp_sw_regs_map sw_reg_map;

	struct timeval frame_process_start;
	struct time_record_s process_start;
	struct timeval event_start;
	struct timeval A_write_time[5];
	struct timeval wake_3a_time;
};

#define TAG_EVT_DBG "[evt_dbg]: "
extern int event_debug;
extern int isp_debug_mask;
extern const char * const event_name[];
extern int threshold;

struct _acamera_firmware_t {
#if ISP_DMA_RAW_CAPTURE
    // dma_capture
    dma_raw_capture_t dma_raw_capture;
#endif

#if ISP_HAS_DMA_INPUT
    dma_input_t dma_input;
#endif

    uint32_t api_context; // the active context for API and to display
    // context instances
    uint32_t context_number;
    acamera_context_t fw_ctx[FIRMWARE_CONTEXT_NUMBER];

    void *dma_chan_isp_config;
    uint32_t dma_flag_isp_config_completed;
    void *dma_chan_isp_metering;
    uint32_t dma_flag_isp_metering_completed;

    uint32_t handler_flag_interrupt_handle_completed;
    atomic_t frame_done;
    atomic_t y_dma_done;
    atomic_t uv_dma_done;
    atomic_t evt_cnt;
    uint8_t first_frame;

    volatile uint8_t *cache_area;
    volatile uint8_t *backup_context;
    unsigned long dma_chn_bitmap;
	unsigned long iridix_chn_bitmap;
    struct mutex ctx_chg_lock;

    uint32_t sw_frame_counter;
    uint32_t initialized;
    uint8_t sif_isp_offline;

    struct work_struct reg_check_work;
    //byte3:status
    //byte2:iridix turn over(1), iridix share(2)
    //byte1:giver ctx id
    //byte0:accepter ctx id
    int iridix_ctrl_flag;
};

void acamera_load_isp_sequence( uintptr_t isp_base, const acam_reg_t **sequence, uint8_t num );
void acamera_load_sw_sequence( uintptr_t isp_base, const acam_reg_t **sequence, uint8_t num );
void acamera_load_array_sequence( acamera_sbus_ptr_t p_sbus, uintptr_t isp_offset, uint32_t size, const acam_reg_t **sequence, int group );


static __inline void acamera_fw_interrupts_enable( acamera_context_ptr_t p_ctx )
{
    if ( p_ctx != NULL ) {
        if ( !( p_ctx->irq_flag ) ) {
            system_interrupts_enable();
        }
    }
}

static __inline void acamera_fw_interrupts_disable( acamera_context_ptr_t p_ctx )
{
    if ( p_ctx != NULL ) {
        if ( !( p_ctx->irq_flag ) ) {
            system_interrupts_disable();
        }
    }
}

static __inline void acamera_isp_interrupts_enable( acamera_fsm_mgr_t *p_fsm_mgr )
{
    if ( p_fsm_mgr != NULL ) {
        acamera_fw_interrupts_enable( p_fsm_mgr->p_ctx );
    }
}

static __inline void acamera_isp_interrupts_disable( acamera_fsm_mgr_t *p_fsm_mgr )
{
    if ( p_fsm_mgr != NULL ) {
        acamera_fw_interrupts_disable( p_fsm_mgr->p_ctx );
    }
}

static __inline void acamera_evt_process_dbg(acamera_context_ptr_t p_ctx)
{
    int last_idx = p_ctx->process_start.index % 2;
    int cur_idx = !last_idx;
    int debug_flag;

    debug_flag = ((event_debug == 1) && ((1 << p_ctx->context_id) & isp_debug_mask));

    if (!debug_flag)
        return;

    printk_ratelimited(TAG_EVT_DBG "==s[%d] time record==\n", p_ctx->context_id);
    printk_ratelimited(TAG_EVT_DBG "cur_irq_fs = %ld.%06ld\n",
            p_ctx->process_start.time[cur_idx].tv_sec,
            p_ctx->process_start.time[cur_idx].tv_usec);
    printk_ratelimited(TAG_EVT_DBG "last_irq_fs = %ld.%06ld\n",
            p_ctx->process_start.time[last_idx].tv_sec,
            p_ctx->process_start.time[last_idx].tv_usec);
    printk_ratelimited(TAG_EVT_DBG "start_processing_frame time = %ld.%06ld\n",
            p_ctx->frame_process_start.tv_sec,
            p_ctx->frame_process_start.tv_usec);
    printk_ratelimited(TAG_EVT_DBG "event_new_frame time = %ld.%06ld\n",
            p_ctx->event_start.tv_sec,
            p_ctx->event_start.tv_usec);
    printk_ratelimited(TAG_EVT_DBG "wake_up_3A time = %ld.%06ld\n",
            p_ctx->wake_3a_time.tv_sec,
            p_ctx->wake_3a_time.tv_usec);
    printk_ratelimited(TAG_EVT_DBG "3A set: ae=%ld.%06ld, awb=%ld.%06ld, af=%ld.%06ld, "
            "gamma=%ld.%06ld, iridix=%ld.%06ld\n",
            p_ctx->A_write_time[0].tv_sec,
            p_ctx->A_write_time[0].tv_usec,
            p_ctx->A_write_time[1].tv_sec,
            p_ctx->A_write_time[1].tv_usec,
            p_ctx->A_write_time[2].tv_sec,
            p_ctx->A_write_time[2].tv_usec,
            p_ctx->A_write_time[3].tv_sec,
            p_ctx->A_write_time[3].tv_usec,
            p_ctx->A_write_time[4].tv_sec,
            p_ctx->A_write_time[4].tv_usec);
}

void acamera_fw_raise_event( acamera_context_ptr_t p_ctx, event_id_t event_id );

void acamera_fw_process( acamera_context_t *p_ctx );

void acamera_fw_init( acamera_context_t *p_ctx );
void acamera_fw_deinit( acamera_context_t *p_ctx );
int32_t acamera_init_context( acamera_context_t *p_ctx, acamera_settings *settings, acamera_firmware_t *g_fw );
void acamera_deinit_context( acamera_context_t *p_ctx );
void acamera_general_interrupt_hanlder( acamera_context_ptr_t p_ctx, uint8_t event );

int32_t acamera_init_calibrations( acamera_context_ptr_t p_ctx );
void acamera_change_resolution( acamera_context_ptr_t p_ctx, uint32_t exposure_correction );
void configure_buffers( acamera_context_ptr_t p_ctx, uint32_t start_addr, uint16_t width, uint16_t height );
void acamera_fw_error_routine( acamera_context_t *p_ctx, uint32_t irq_mask );

#define ACAMERA_MGR2CTX_PTR( p_fsm_mgr ) \
    ( ( p_fsm_mgr )->p_ctx )

#define ACAMERA_MGR2FIRMWARE_PTR( p_fsm_mgr ) \
    ( ( p_fsm_mgr )->p_ctx->p_gfw )

#define ACAMERA_FSM2MGR_PTR( p_fsm ) \
    ( ( p_fsm )->p_fsm_mgr )

#define ACAMERA_FSM2CTX_PTR( p_fsm ) \
    ( ( p_fsm )->p_fsm_mgr->p_ctx )

#endif /* __ACAMERA_FW_H__ */
