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
#include "acamera_fw.h"
#include "acamera_metering_stats_mem_config.h"
#include "acamera_math.h"
#include "acamera_lens_api.h"
#include "system_stdlib.h"
#include "acamera_isp_core_nomem_settings.h"
#include "af_manual_fsm.h"
#include "acamera_ctrl_channel.h"
#include "acamera_command_api.h"

#include "sbuf.h"

#if defined( ISP_METERING_OFFSET_AF )
#define ISP_METERING_OFFSET_AUTO_FOCUS ISP_METERING_OFFSET_AF
#elif defined( ISP_METERING_OFFSET_AF2W )
#define ISP_METERING_OFFSET_AUTO_FOCUS ISP_METERING_OFFSET_AF2W
#else
#error "The AF metering offset is not defined in acamera_metering_mem_config.h"
#endif


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_AF_MANUAL
#else
#define CUR_MOD_NAME LOG_MODULE_AF_MANUAL
#endif

//TODO save lut of af
#define ZOOM_STEP_NUM                  13   //  Zoom Ratio = 1.0 + (ZOOM_STEP_NUM-1) * 0.1
#define AF_INIT_PARAMETER_NUM          21

uint32_t zoom_af_table[ZOOM_STEP_NUM][AF_INIT_PARAMETER_NUM] =
{
 {166400, 166400, 166400, 167680, 167680, 167680,176000,176000,176000,177920,177920,177920,11,6,2,30,131072,131072,262144,65536, 0},
 {136320, 136320, 136320, 137600, 137600, 137600,147200,147200,147200,149120,149120,149120,11,6,2,30,131072,131072,262144,65536, 0},
 {110720, 110720, 110720, 112000, 112000, 112000,120320,120320,120320,122240,122240,122240,11,6,2,30,131072,131072,262144,65536, 0},
 {88960, 88960, 88960, 90240, 90240, 90240, 99200, 99200, 99200,101120,101120,101120,11,6,2,30,131072,131072,262144,65536, 0},
 {72320, 72320, 72320, 73600, 73600, 73600, 81920, 81920, 81920, 83840, 83840, 83840,11,6,2,30,131072,131072,262144,65536, 0},
 {58240, 58240, 58240, 59520, 59520, 59520, 67840, 67840, 67840, 69760, 69760, 69760,11,6,2,30,131072,131072,262144,65536, 0},
 {46080, 46080, 46080, 47360, 47360, 47360, 56960, 56960, 56960, 58880, 58880, 58880,11,6,2,30,131072,131072,262144,65536, 0},
 {36480, 36480, 36480, 37760, 37760, 37760, 46720, 46720, 46720, 48640, 48640, 48640,11,6,2,30,131072,131072,262144,65536, 0},
 {28800, 28800, 28800, 30080, 30080, 30080, 39040, 39040, 39040, 40960, 40960, 40960,11,6,2,30,131072,131072,262144,65536, 0},
 {23040, 23040, 23040, 24320, 24320, 24320, 33280, 33280, 33280, 35200, 35200, 35200,11,6,2,30,131072,131072,262144,65536, 0},
 {18560, 18560, 18560, 19840, 19840, 19840, 28800, 28800, 28800, 30720, 30720, 30720,11,6,2,30,131072,131072,262144,65536, 0},
 {14720, 14720, 14720, 16000, 16000, 16000, 24960, 24960, 24960, 26880, 26880, 26880,11,6,2,30,131072,131072,262144,65536, 0},
 {11520, 11520, 11520, 12800, 12800, 12800, 22400, 22400, 22400, 24320, 24320, 24320,11,6,2,30,131072,131072,262144,65536, 0}
};


void zoom_update_lens_position(AF_fsm_ptr_t p_fsm)
{
	uint32_t count = 0;
	uint32_t ret_data = 0;

	//const lens_param_t *lens_param = p_fsm->lens_ctrl.get_parameters(p_fsm->lens_ctx);
	af_lms_param_t *param = (af_lms_param_t *)_GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_AF_LMS );

	/* the new AF position is updated in sbuf FSM */
	if (p_fsm->zoom_manual_pos != p_fsm->zoom_curr_pos) {
		LOG(LOG_INFO, "last position(%u) move changed.", p_fsm->zoom_curr_pos);
		p_fsm->lens_ctrl.move_zoom( p_fsm->lens_ctx, p_fsm->zoom_manual_pos);

		p_fsm->zoom_curr_pos = p_fsm->zoom_manual_pos;
		count = (p_fsm->zoom_manual_pos - 100) / 10;
		if (count < ZOOM_STEP_NUM) {
			zoom_af_table[count][AF_INIT_PARAMETER_NUM - 1] = param->print_debug;
			//memcpy(param, &zoom_af_table[count][0], (sizeof(af_lms_param_t) - sizeof(uint32_t)));
			acamera_api_calibration(p_fsm->cmn.ctx_id, DYNAMIC_CALIBRATIONS_ID, CALIBRATION_AF_LMS,
				COMMAND_SET, (void *)&zoom_af_table[count][0], sizeof(af_lms_param_t), &ret_data);
		}
	} else {
		LOG(LOG_INFO, "last position(%u) not changed.", p_fsm->zoom_curr_pos);
	}
}

static void af_update_lens_position( AF_fsm_ptr_t p_fsm )
{
    const lens_param_t *lens_param = p_fsm->lens_ctrl.get_parameters( p_fsm->lens_ctx );
    af_lms_param_t *param = (af_lms_param_t *)_GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_AF_LMS );

    /* the new AF position is updated in sbuf FSM */
    if ( p_fsm->last_position != p_fsm->new_pos ) {
        int fw_id = p_fsm->cmn.ctx_id;

        LOG( LOG_INFO, "pso_min %d, pos_max %d, new_pos %d", p_fsm->pos_min, p_fsm->pos_max, p_fsm->new_pos );
        p_fsm->lens_ctrl.move( p_fsm->lens_ctx, p_fsm->new_pos );
        p_fsm->frame_skip_start = 1;
        LOG( LOG_INFO, "ctx: %d, new af applied, position: %u, last_position: %u.", fw_id, p_fsm->new_pos, p_fsm->last_position );

        if ( param->print_debug )
            LOG( LOG_NOTICE, "ctx: %d, new af applied, position: %u, last_position: %u.", fw_id, p_fsm->new_pos, p_fsm->last_position );

        /* update the done position and sharpness when sharpness value changed */
        if ( ( p_fsm->last_sharp_done != p_fsm->new_last_sharp ) && ( p_fsm->new_last_sharp != 0 ) ) {
            p_fsm->last_pos_done = p_fsm->last_position;
            p_fsm->last_sharp_done = p_fsm->new_last_sharp;

            if ( AF_MODE_CALIBRATION == p_fsm->mode && param->print_debug )
                LOG( LOG_NOTICE, "new af calibration, pos: %u, sharp: %d.", p_fsm->last_pos_done / lens_param->min_step, p_fsm->last_sharp_done );
        }

        p_fsm->last_position = p_fsm->new_pos;

        status_info_param_t *p_status_info = (status_info_param_t *)_GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_STATUS_INFO );
        p_status_info->af_info.lens_pos = p_fsm->last_position;
        p_status_info->af_info.focus_value = p_fsm->last_sharp_done;
    } else {
        LOG( LOG_INFO, "last position(%u) not changed.", p_fsm->last_position );
    }
}

void af_set_new_param( AF_fsm_ptr_t p_fsm, sbuf_af_t *p_sbuf_af )
{
    p_fsm->new_pos = p_sbuf_af->af_position;
    p_fsm->new_last_sharp = p_sbuf_af->af_last_sharp;
    p_fsm->skip_frame = p_sbuf_af->frame_to_skip;
    p_fsm->state = p_sbuf_af->state;

    af_update_lens_position( p_fsm );
}

void af_read_stats_data( AF_fsm_ptr_t p_fsm )
{
    uint8_t zones_horiz, zones_vert, x, y;
    uint32_t( *stats )[2];
    sbuf_af_t *p_sbuf_af = NULL;
    struct sbuf_item sbuf;
    int fw_id = p_fsm->cmn.ctx_id;

    memset( &sbuf, 0, sizeof( sbuf ) );
    sbuf.buf_type = SBUF_TYPE_AF;
    sbuf.buf_status = SBUF_STATUS_DATA_EMPTY;

    if ( p_fsm->frame_num )
        p_fsm->frame_num--;

    if ( sbuf_get_item( fw_id, &sbuf ) ) {
        LOG( LOG_ERR, "Error: Failed to get sbuf, return." );
        return;
    }

    p_sbuf_af = (sbuf_af_t *)sbuf.buf_base;
    LOG( LOG_DEBUG, "Get sbuf ok, idx: %u, status: %u, addr: %p.", sbuf.buf_idx, sbuf.buf_status, sbuf.buf_base );

    zones_horiz = acamera_isp_metering_af_nodes_used_horiz_read( p_fsm->cmn.isp_base );
    zones_vert = acamera_isp_metering_af_nodes_used_vert_read( p_fsm->cmn.isp_base );
    stats = p_sbuf_af->stats_data;
    p_sbuf_af->frame_num = p_fsm->frame_num;

    if ( zones_horiz || !zones_vert ) {
        zones_horiz = ISP_DEFAULT_AF_ZONES_HOR;
        zones_vert = ISP_DEFAULT_AF_ZONES_VERT;
    }
    p_sbuf_af->zones_horiz = zones_horiz;
    p_sbuf_af->zones_vert = zones_vert;

    // we need to skip frames after lens movement to get stable stats for next AF step calculation.
    if ( p_fsm->skip_frame ) {
        p_sbuf_af->skip_cur_frame = 1;
        p_fsm->skip_frame--;
    } else {
        p_sbuf_af->skip_cur_frame = 0;
    }

    for ( y = 0; y < zones_vert; y++ ) {
        uint32_t inx = (uint32_t)y * zones_horiz;
        for ( x = 0; x < zones_horiz; x++ ) {
            uint32_t full_inx = inx + x;
            stats[full_inx][0] = acamera_metering_stats_mem_array_data_read( p_fsm->cmn.isp_base, ISP_METERING_OFFSET_AUTO_FOCUS + ( ( full_inx ) << 1 ) + 0 );
            stats[full_inx][1] = acamera_metering_stats_mem_array_data_read( p_fsm->cmn.isp_base, ISP_METERING_OFFSET_AUTO_FOCUS + ( ( full_inx ) << 1 ) + 1 );
        }
    }

    /* read done, set the buffer back for future using  */
    sbuf.buf_status = SBUF_STATUS_DATA_DONE;

    if ( sbuf_set_item( fw_id, &sbuf ) ) {
        LOG( LOG_ERR, "Error: Failed to set sbuf, return." );
        return;
    }
    LOG( LOG_DEBUG, "Set sbuf ok, idx: %u, status: %u, addr: %p.", sbuf.buf_idx, sbuf.buf_status, sbuf.buf_base );
}

void AF_fsm_process_interrupt( AF_fsm_const_ptr_t p_fsm, uint8_t irq_event )
{
    if ( acamera_fsm_util_is_irq_event_ignored( (fsm_irq_mask_t *)( &p_fsm->mask ), irq_event ) )
        return;

    //check if lens was initialised properly
    if ( p_fsm->lens_driver_ok == 0 ) {
        LOG( LOG_INFO, "lens driver is not OK, return" );
        return;
    }

    switch ( irq_event ) {
    case ACAMERA_IRQ_AF2_STATS: // read out the statistic
        af_read_stats_data( (AF_fsm_ptr_t)p_fsm );
        fsm_raise_event( p_fsm, event_id_af_stats_ready );


        break;
    }
}
//================================================================================
void AF_init( AF_fsm_ptr_t p_fsm )
{
    int32_t result = 0;
    af_lms_param_t *param = NULL;
    //check if lens was initialised properly
    if ( !p_fsm->lens_driver_ok ) {
        p_fsm->lens_ctx = NULL;

        if ( ACAMERA_FSM2CTX_PTR( p_fsm )->settings.lens_init != NULL ) {
            result = ACAMERA_FSM2CTX_PTR( p_fsm )->settings.lens_init( &p_fsm->lens_ctx, &p_fsm->lens_ctrl );
            if ( result != -1 && p_fsm->lens_ctx != NULL ) {
                //only on lens init success populate param
                param = (af_lms_param_t *)_GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_AF_LMS );
                p_fsm->lens_driver_ok = 1;
            } else {
                p_fsm->lens_driver_ok = 0;
                return;
            }
        } else {
            p_fsm->lens_driver_ok = 0;
            return;
        }
    }

    if ( param ) {
        p_fsm->pos_min = p_fsm->def_pos_min = param->pos_min;
        p_fsm->pos_inf = p_fsm->def_pos_inf = param->pos_inf;
        p_fsm->pos_macro = p_fsm->def_pos_macro = param->pos_macro;
        p_fsm->pos_max = p_fsm->def_pos_max = param->pos_max;
        p_fsm->def_pos_min_down = param->pos_min_down;
        p_fsm->def_pos_inf_down = param->pos_inf_down;
        p_fsm->def_pos_macro_down = param->pos_macro_down;
        p_fsm->def_pos_max_down = param->pos_max_down;
        p_fsm->def_pos_min_up = param->pos_min_up;
        p_fsm->def_pos_inf_up = param->pos_inf_up;
        p_fsm->def_pos_macro_up = param->pos_macro_up;
        p_fsm->def_pos_max_up = param->pos_max_up;

        p_fsm->mask.repeat_irq_mask = ACAMERA_IRQ_MASK( ACAMERA_IRQ_AF2_STATS );
        AF_request_interrupt( p_fsm, p_fsm->mask.repeat_irq_mask );
    }
}

void AF_deinit( AF_fsm_ptr_t p_fsm )
{
    if ( ACAMERA_FSM2CTX_PTR( p_fsm )->settings.lens_deinit )
        ACAMERA_FSM2CTX_PTR( p_fsm )
            ->settings.lens_deinit( p_fsm->lens_ctx );
}
