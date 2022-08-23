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
#include "ae_manual_fsm.h"
#include "sbuf.h"


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_AE_MANUAL
#else
#define CUR_MOD_NAME LOG_MODULE_AE_MANUAL
#endif


void AE_fsm_clear( AE_fsm_t *p_fsm )
{
    p_fsm->error_log2 = 0;
    p_fsm->ae_hist_mean = 0;
    p_fsm->exposure_log2 = 0;
    p_fsm->new_exposure_log2 = 0;
    p_fsm->integrator = 0;
    p_fsm->exposure_ratio = 64;
    p_fsm->new_exposure_ratio = 64;
    p_fsm->exposure_ratio_avg = 64;
    p_fsm->ae_roi_api = AE_CENTER_ZONES;
    p_fsm->roi = AE_CENTER_ZONES;
    p_fsm->frame_id_tracking = 0;
    memset(&p_fsm->ae_info, 0, sizeof(p_fsm->ae_info));
    p_fsm->external_ae_enable = 0;
    p_fsm->sensor_ctrl_enable = 0;
}

void AE_request_interrupt( AE_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask )
{
    // acamera_isp_interrupts_disable( ACAMERA_FSM2MGR_PTR( p_fsm ) );
    p_fsm->mask.irq_mask |= mask;
    // acamera_isp_interrupts_enable( ACAMERA_FSM2MGR_PTR( p_fsm ) );
}

void AE_fsm_init( void *fsm, fsm_init_param_t *init_param )
{
    AE_fsm_t *p_fsm = (AE_fsm_t *)fsm;
    p_fsm->cmn.p_fsm_mgr = init_param->p_fsm_mgr;
    p_fsm->cmn.isp_base = init_param->isp_base;
    p_fsm->p_fsm_mgr = init_param->p_fsm_mgr;

    AE_fsm_clear( p_fsm );

    ae_initialize( p_fsm );
}


int AE_fsm_set_param( void *fsm, uint32_t param_id, void *input, uint32_t input_size )
{
    int rc = 0;
    AE_fsm_t *p_fsm = (AE_fsm_t *)fsm;

    switch ( param_id ) {
    case FSM_PARAM_SET_AE_INIT:
        AE_fsm_clear( p_fsm );
        ae_initialize( p_fsm );
        break;

    case FSM_PARAM_SET_AE_NEW_PARAM:
        if ( !input || input_size != sizeof( sbuf_ae_t ) ) {
            LOG( LOG_ERR, "Invalid param, param_id: %d.", param_id );
            rc = -1;
            break;
        }

        ae_set_new_param( p_fsm, (sbuf_ae_t *)input );

        break;

    case FSM_PARAM_SET_AE_ROI: {
        if ( !input || input_size != sizeof( fsm_param_roi_t ) ) {
            LOG( LOG_ERR, "Invalid param, param_id: %d.", param_id );
            rc = -1;
            break;
        }

        fsm_param_roi_t *p_new = (fsm_param_roi_t *)input;
        p_fsm->ae_roi_api = p_new->roi_api;
        p_fsm->roi = p_new->roi;
        p_fsm->param_set = 1;

        break;
    }

    default:
        rc = -1;
        break;
    }

    return rc;
}


int AE_fsm_get_param( void *fsm, uint32_t param_id, void *input, uint32_t input_size, void *output, uint32_t output_size )
{
    int rc = 0;
    AE_fsm_t *p_fsm = (AE_fsm_t *)fsm;

    switch ( param_id ) {
    case FSM_PARAM_GET_AE_INFO: {
        if ( !output || output_size != sizeof( fsm_param_ae_info_t ) ) {
            LOG( LOG_ERR, "Invalid param, param_id: %d.", param_id );
            rc = -1;
            break;
        }

        fsm_param_ae_info_t *p_ae_info = (fsm_param_ae_info_t *)output;

        p_ae_info->exposure_log2 = p_fsm->exposure_log2;
        p_ae_info->ae_hist_mean = p_fsm->ae_hist_mean;
        p_ae_info->exposure_ratio = p_fsm->exposure_ratio;
        p_ae_info->error_log2 = p_fsm->error_log2;
        break;
    }

    case FSM_PARAM_GET_AE_HIST_INFO: {
        if ( !output || output_size != sizeof( fsm_param_ae_hist_info_t ) ) {
            LOG( LOG_ERR, "Invalid param, param_id: %d.", param_id );
            rc = -1;
            break;
        }

        fsm_param_ae_hist_info_t *p_hist_info = (fsm_param_ae_hist_info_t *)output;

        p_hist_info->fullhist_sum = p_fsm->fullhist_sum;
        p_hist_info->fullhist = p_fsm->fullhist;
        p_hist_info->frame_id = p_fsm->frame_id_tracking;

        break;
    }

    case FSM_PARAM_GET_AE_ROI: {
        if ( !output || output_size != sizeof( fsm_param_roi_t ) ) {
            LOG( LOG_ERR, "Invalid param, param_id: %d.", param_id );
            rc = -1;
            break;
        }

        fsm_param_roi_t *p_current = (fsm_param_roi_t *)output;
        p_current->roi_api = p_fsm->ae_roi_api;
        p_current->roi = p_fsm->roi;

        break;
    }

    case FSM_PARAM_GET_AE_STATE: {
        if ( !output || output_size != sizeof( ae_state_t ) ) {
            LOG( LOG_ERR, "Invalid param, param_id: %d.", param_id );
            rc = -1;
            break;
        }

        *(ae_state_t *)output = p_fsm->state;

        break;
    }

    case FSM_PARAM_GET_LUMVAR_STATS: {
        if (!output || output_size != sizeof(p_fsm->lumvar)) {
            LOG(LOG_ERR, "Invalid param, param_id: %d.", param_id);
            rc = -1;
            break;
        }

	memcpy(output, p_fsm->lumvar, sizeof(p_fsm->lumvar));

        break;
    }

    default:
        rc = -1;
        break;
    }

    return rc;
}


uint8_t AE_fsm_process_event( AE_fsm_t *p_fsm, event_id_t event_id )
{
    uint8_t b_event_processed = 0;
    switch ( event_id ) {
    default:
        break;
    case event_id_ae_result_ready:
        if (p_fsm->external_ae_enable == 0) { // user arm3a
                if ( ae_calculate_exposure( p_fsm ) ) {
                        fsm_raise_event( p_fsm, event_id_exposure_changed );
                }
        } else { // user external 3a
                if ( ae_calculate_exposure( p_fsm ) ) {
                        fsm_raise_event(p_fsm, event_id_extern_ae_readly);
                }
        }

        b_event_processed = 1;
        break;
    case event_id_frame_end:
        if (p_fsm->param_set == 1) {
            ae_roi_update(p_fsm);
            p_fsm->param_set = 0;
        }
        b_event_processed = 1;
        break;
    }

    return b_event_processed;
}
