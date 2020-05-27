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

#include "acamera_fw.h"
#include "noise_reduction_fsm.h"


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_NOISE_REDUCTION
#else
#define CUR_MOD_NAME LOG_MODULE_NOISE_REDUCTION
#endif


void noise_reduction_fsm_clear( noise_reduction_fsm_t *p_fsm )
{
    p_fsm->temper_ev_previous_frame = 0;
    p_fsm->temper_diff_avg = 0;
    p_fsm->temper_diff_coeff = 10;
    p_fsm->snr_thresh_contrast = 0;

    p_fsm->nr_mode = NOISE_REDUCTION_MODE_ON;
}

void noise_reduction_request_interrupt( noise_reduction_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask )
{
    acamera_isp_interrupts_disable( ACAMERA_FSM2MGR_PTR( p_fsm ) );
    p_fsm->mask.irq_mask |= mask;
    acamera_isp_interrupts_enable( ACAMERA_FSM2MGR_PTR( p_fsm ) );
}

void noise_reduction_fsm_init( void *fsm, fsm_init_param_t *init_param )
{
    noise_reduction_fsm_t *p_fsm = (noise_reduction_fsm_t *)fsm;
    p_fsm->cmn.p_fsm_mgr = init_param->p_fsm_mgr;
    p_fsm->cmn.isp_base = init_param->isp_base;
    p_fsm->p_fsm_mgr = init_param->p_fsm_mgr;

    noise_reduction_fsm_clear( p_fsm );

    noise_reduction_initialize( p_fsm );
    noise_reduction_hw_init( p_fsm );
}

uint8_t noise_reduction_fsm_process_event( noise_reduction_fsm_t *p_fsm, event_id_t event_id )
{
    uint8_t b_event_processed = 0;
    switch ( event_id ) {
    default:
        break;
    case event_id_frame_end:
        noise_reduction_update( p_fsm );
        noise_reduction_initialize( p_fsm );
        b_event_processed = 1;
        break;
    }

    return b_event_processed;
}

int noise_reduction_fsm_set_param( void *fsm, uint32_t param_id,
                                   void *input, uint32_t input_size )
{
    int rc = 0;
    noise_reduction_fsm_t *p_fsm = (noise_reduction_fsm_t *)fsm;

    switch ( param_id ) {
    case FSM_PARAM_SET_NOISE_REDUCTION_MODE: {
        if ( !input || input_size != sizeof( noise_reduction_mode_t ) ) {
            LOG( LOG_ERR, "Invalid param, param_id: %d.", param_id );
            rc = -1;
            break;
        }
        p_fsm->nr_mode = *(noise_reduction_mode_t *)input;

        break;
    }

    default:
        rc = -1;
        break;
    };

    return rc;
}

int noise_reduction_fsm_get_param( void *fsm, uint32_t param_id,
                                    void *input, uint32_t input_size,
                                    void *output, uint32_t output_size )
{
    int rc = 0;
    noise_reduction_fsm_t *p_fsm = (noise_reduction_fsm_t *)fsm;

    switch ( param_id ) {
    case FSM_PARAM_GET_NOISE_REDUCTION_MODE: {

        if ( !output || output_size != sizeof( noise_reduction_mode_t ) ) {
            LOG( LOG_ERR, "Invalid param, param_id: %d.", param_id );
            rc = -1;
            break;
        }

        *(noise_reduction_mode_t *)output = p_fsm->nr_mode;

        break;
    }

    default:
        rc = -1;
        break;
    };

    return rc;
}
