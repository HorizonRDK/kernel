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

#include <linux/crc16.h>
#include <linux/ratelimit.h>
#include "acamera_firmware_api.h"
#include "acamera_fw.h"
#include "acamera_math.h"
#include "acamera_command_api.h"

#include "acamera_aexp_hist_stats_mem_config.h"
#include "acamera_metering_stats_mem_config.h"
#include "acamera_lumvar_stats_mem_config.h"

#include "acamera_math.h"
#include "ae_manual_fsm.h"
#include "vio_group_api.h"

#include "sbuf.h"

#include "isp_ctxsv.h"

#define DEFAULT_AE_EXPOSURE_LOG2 3390000


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_AE_MANUAL
#else
#define CUR_MOD_NAME LOG_MODULE_AE_MANUAL
#endif


void ae_roi_update( AE_fsm_ptr_t p_fsm )
{
    uint16_t horz_zones = acamera_isp_metering_hist_aexp_nodes_used_horiz_read( p_fsm->cmn.isp_base );
    uint16_t vert_zones = acamera_isp_metering_hist_aexp_nodes_used_vert_read( p_fsm->cmn.isp_base );
    uint16_t x, y;

    if (horz_zones > ISP_METERING_ZONES_MAX_H || vert_zones > ISP_METERING_ZONES_MAX_V) {
        printk_ratelimited("ae horiz %d, vert %x error.\n", horz_zones, vert_zones);
        horz_zones = ISP_METERING_ZONES_MAX_H;
        vert_zones = ISP_METERING_ZONES_MAX_V;
    }

    uint16_t *ptr_ae_zone_whgh_h = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_AE_ZONE_WGHT_HOR );
    uint16_t *ptr_ae_zone_whgh_v = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_AE_ZONE_WGHT_VER );

    uint8_t x_start = ( uint8_t )( ( ( ( p_fsm->roi >> 24 ) & 0xFF ) * horz_zones + 128 ) >> 8 );
    uint8_t x_end = ( uint8_t )( ( ( ( p_fsm->roi >> 8 ) & 0xFF ) * horz_zones + 128 ) >> 8 );
    uint8_t y_start = ( uint8_t )( ( ( ( p_fsm->roi >> 16 ) & 0xFF ) * vert_zones + 128 ) >> 8 );
    uint8_t y_end = ( uint8_t )( ( ( ( p_fsm->roi >> 0 ) & 0xFF ) * vert_zones + 128 ) >> 8 );

    uint8_t zone_size_x = x_end - x_start;
    uint8_t zone_size_y = y_end - y_start;
    uint32_t middle_x = zone_size_x * 256 / 2;
    uint32_t middle_y = zone_size_y * 256 / 2;
    uint16_t scale_x = 0;
    uint16_t scale_y = 0;
    uint32_t ae_zone_wght_hor_len = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_AE_ZONE_WGHT_HOR );
    uint32_t ae_zone_wght_ver_len = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_AE_ZONE_WGHT_VER );

    if ( ae_zone_wght_hor_len ) {
        scale_x = ( horz_zones - 1 ) / ae_zone_wght_hor_len + 1;
    } else {
        LOG( LOG_ERR, "ae_zone_wght_hor_len is zero" );
        return;
    }
    if ( ae_zone_wght_ver_len ) {
        scale_y = ( vert_zones - 1 ) / ae_zone_wght_ver_len + 1;
    } else {
        LOG( LOG_ERR, "ae_zone_wght_ver_len is zero" );
        return;
    }

    uint16_t gaus_center_x = ( _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_AE_ZONE_WGHT_HOR ) * 256 / 2 ) * scale_x;
    uint16_t gaus_center_y = ( _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_AE_ZONE_WGHT_VER ) * 256 / 2 ) * scale_y;

    for ( y = 0; y < vert_zones; y++ ) {
        uint8_t ae_coeff = 0;
        for ( x = 0; x < horz_zones; x++ ) {
            if ( y >= y_start && y <= y_end &&
                 x >= x_start && x <= x_end ) {

                uint8_t index_y = ( y - y_start );
                uint8_t index_x = ( x - x_start );
                int32_t distance_x = ( index_x * 256 + 128 ) - middle_x;
                int32_t distance_y = ( index_y * 256 + 128 ) - middle_y;
                uint32_t coeff_x;
                uint32_t coeff_y;

                if ( ( x == x_end && x_start != x_end ) ||
                     ( y == y_end && y_start != y_end ) ) {
                    ae_coeff = 0;
                } else {
                    coeff_x = ( gaus_center_x + distance_x ) / 256;
                    if ( distance_x > 0 && ( distance_x & 0x80 ) )
                        coeff_x--;
                    coeff_y = ( gaus_center_y + distance_y ) / 256;
                    if ( distance_y > 0 && ( distance_y & 0x80 ) )
                        coeff_y--;

                    coeff_x = ptr_ae_zone_whgh_h[coeff_x / scale_x];
                    coeff_y = ptr_ae_zone_whgh_v[coeff_y / scale_y];

                    ae_coeff = ( coeff_x * coeff_y ) >> 4;
                    if ( ae_coeff > 1 )
                        ae_coeff--;
                }
            } else {
                ae_coeff = 0;
            }
            acamera_isp_metering_hist_aexp_zones_weight_write( p_fsm->cmn.isp_base, ISP_METERING_ZONES_MAX_H * y + x, ae_coeff );
        }
    }
}

void ae_initialize( AE_fsm_ptr_t p_fsm )
{
    acamera_isp_metering_aexp_hist_thresh_0_1_write( p_fsm->cmn.isp_base, 820 );
    acamera_isp_metering_aexp_hist_thresh_1_2_write( p_fsm->cmn.isp_base, 1638 );
    acamera_isp_metering_aexp_hist_thresh_3_4_write( p_fsm->cmn.isp_base, 2458 );
    acamera_isp_metering_aexp_hist_thresh_4_5_write( p_fsm->cmn.isp_base, 3276 );

    int i, j;
    for ( i = 0; i < ISP_METERING_ZONES_AE5_V; i++ )
        for ( j = 0; j < ISP_METERING_ZONES_AE5_H; j++ )
            acamera_isp_metering_hist_aexp_zones_weight_write( p_fsm->cmn.isp_base, ISP_METERING_ZONES_MAX_H * i + j, 15 );
    ae_roi_update( p_fsm );


    p_fsm->new_exposure_log2 = DEFAULT_AE_EXPOSURE_LOG2;

    p_fsm->mask.repeat_irq_mask = ACAMERA_IRQ_MASK( ACAMERA_IRQ_AE_STATS ) | ACAMERA_IRQ_MASK( ACAMERA_IRQ_FRAME_END );
    AE_request_interrupt( p_fsm, p_fsm->mask.repeat_irq_mask );

    // set default exposure value
    ae_calculate_exposure( p_fsm );
}

extern int time_takes_check;
uint32_t lumvar[512] = {0};
void ae_read_full_histogram_data( AE_fsm_ptr_t p_fsm )
{
    int i;
    int shift = 0;
    uint32_t sum = 0;
    uint32_t _metering_lut_entry;

    sbuf_ae_t *p_sbuf_ae;
    struct sbuf_item sbuf;
    int fw_id = p_fsm->cmn.ctx_id;
    fsm_param_mon_alg_flow_t ae_flow;
    acamera_context_t *p_ctx = ACAMERA_FSM2CTX_PTR( p_fsm );
    struct timeval tv1, tv2;
    unsigned long irq_interval;

    if (time_takes_check)
        do_gettimeofday(&tv1);

    memset( &sbuf, 0, sizeof( sbuf ) );
    sbuf.buf_type = SBUF_TYPE_AE;
    sbuf.buf_status = SBUF_STATUS_DATA_EMPTY;

    if ( sbuf_get_item( fw_id, &sbuf ) ) {
        LOG( LOG_ERR, "Error: Failed to get sbuf, return." );
        return;
    }

    p_sbuf_ae = (sbuf_ae_t *)sbuf.buf_base;
    pr_debug("Get sbuf ok, idx: %u, status: %u, addr: %p.", sbuf.buf_idx, sbuf.buf_status, sbuf.buf_base );
	if(p_ctx->p_gfw->sif_isp_offline) {
		ae_flow.frame_id_tracking = p_ctx->p_gfw->sw_frame_counter;
	} else {
		ae_flow.frame_id_tracking = acamera_fsm_util_get_cur_frame_id( &p_fsm->cmn);
	}
    p_sbuf_ae->frame_id = ae_flow.frame_id_tracking;

    for ( i = 0; i < ISP_FULL_HISTOGRAM_SIZE; ++i ) {
        uint32_t v = acamera_aexp_hist_stats_mem_array_data_read( p_fsm->cmn.isp_base, i );

        shift = ( v >> 12 ) & 0xF;
        v = v & 0xFFF;
        if ( shift ) {
            v = ( v | 0x1000 ) << ( shift - 1 );
        }

        /* some other FSMs(such as sharpening) in kern-FW also need AE stats data */
        p_fsm->fullhist[i] = v;

        sum += v;
    }

    p_fsm->fullhist_sum = sum;

    int rc = 0;
    rc = system_chardev_lock();
    if (rc == 0 && p_ctx->isp_ae_stats_on) {
	    isp_ctx_node_t *cn;
        struct vio_frame_id frmid;
	    cn = isp_ctx_get_node(fw_id, ISP_AE, FREEQ);
	    if (cn) {
            vio_get_sif_frame_info(fw_id, &frmid);
		    cn->ctx.frame_id = frmid.frame_id;
            cn->ctx.timestamps = frmid.timestamps;
		    memcpy(cn->base, p_fsm->fullhist, sizeof(p_sbuf_ae->stats_data));
		    cn->ctx.crc16 = crc16(~0, cn->base, sizeof(p_sbuf_ae->stats_data));
		    isp_ctx_put_node(fw_id, cn, ISP_AE, DONEQ);

		    pr_debug("ae stats frame id %d\n", cn->ctx.frame_id);
	    }
    }
    if (rc == 0)
        system_chardev_unlock();    


    /* NOTE: the size should match */
    memcpy( p_sbuf_ae->stats_data, p_fsm->fullhist, sizeof( p_sbuf_ae->stats_data ) );
    p_sbuf_ae->histogram_sum = sum;
    LOG( LOG_DEBUG, "histsum: histogram_sum: %u.", p_sbuf_ae->histogram_sum );

    if (p_ctx->isp_ae_5bin_stats_on) {
        p_sbuf_ae->ae_5bin_info.zones_h = (uint16_t)acamera_isp_metering_aexp_nodes_used_horiz_read(p_fsm->cmn.isp_base);
        p_sbuf_ae->ae_5bin_info.zones_v = (uint16_t)acamera_isp_metering_aexp_nodes_used_vert_read(p_fsm->cmn.isp_base);

        if (p_sbuf_ae->ae_5bin_info.zones_h > ISP_METERING_ZONES_AE5_H) {
            printk_ratelimited("5bin zone h %d invalid.\n", p_sbuf_ae->ae_5bin_info.zones_h);
            p_sbuf_ae->ae_5bin_info.zones_h = ISP_METERING_ZONES_AE5_H;
        }
        if (p_sbuf_ae->ae_5bin_info.zones_v > ISP_METERING_ZONES_AE5_V) {
            printk_ratelimited("5bin zone v %d invalid.\n", p_sbuf_ae->ae_5bin_info.zones_v);
            p_sbuf_ae->ae_5bin_info.zones_v = ISP_METERING_ZONES_AE5_V;
        }

        p_sbuf_ae->ae_5bin_info.zones_size = (uint32_t)p_sbuf_ae->ae_5bin_info.zones_v * p_sbuf_ae->ae_5bin_info.zones_h;
        p_sbuf_ae->ae_5bin_info.threshold0_1 = (uint16_t)acamera_isp_metering_aexp_hist_thresh_0_1_read(p_fsm->cmn.isp_base);
        p_sbuf_ae->ae_5bin_info.threshold1_2 = (uint16_t)acamera_isp_metering_aexp_hist_thresh_1_2_read(p_fsm->cmn.isp_base);
        p_sbuf_ae->ae_5bin_info.threshold3_4 = (uint16_t)acamera_isp_metering_aexp_hist_thresh_3_4_read(p_fsm->cmn.isp_base);
        p_sbuf_ae->ae_5bin_info.threshold4_5 = (uint16_t)acamera_isp_metering_aexp_hist_thresh_4_5_read(p_fsm->cmn.isp_base);
        p_sbuf_ae->ae_5bin_info.normal_bin0 = (uint16_t)acamera_isp_metering_aexp_hist_0_read(p_fsm->cmn.isp_base);
        p_sbuf_ae->ae_5bin_info.normal_bin1 = (uint16_t)acamera_isp_metering_aexp_hist_1_read(p_fsm->cmn.isp_base);
        p_sbuf_ae->ae_5bin_info.normal_bin3 = (uint16_t)acamera_isp_metering_aexp_hist_3_read(p_fsm->cmn.isp_base);
        p_sbuf_ae->ae_5bin_info.normal_bin4 = (uint16_t)acamera_isp_metering_aexp_hist_4_read(p_fsm->cmn.isp_base);
        LOG(LOG_DEBUG, "5bin: zones_sum: %u.", p_sbuf_ae->ae_5bin_info.zones_size);

        for ( i = 0; i < p_sbuf_ae->ae_5bin_info.zones_size; i++ ) {
            _metering_lut_entry = acamera_metering_stats_mem_array_data_read(p_fsm->cmn.isp_base, i * 2);
            LOG(LOG_DEBUG, "5bin: %u. data %d \n", i, _metering_lut_entry);
            p_sbuf_ae->hist4[i * 4] = ( uint16_t )(_metering_lut_entry & 0xffff);
            p_sbuf_ae->hist4[i * 4 + 1] = ( uint16_t )((_metering_lut_entry >> 16) & 0xffff);
            _metering_lut_entry = acamera_metering_stats_mem_array_data_read(p_fsm->cmn.isp_base, i * 2 + 1);
            LOG(LOG_DEBUG, "5bin: %u. data %d \n", i, _metering_lut_entry);
            p_sbuf_ae->hist4[i * 4 + 2] = ( uint16_t )(_metering_lut_entry & 0xffff);
            p_sbuf_ae->hist4[i * 4 + 3] = ( uint16_t )((_metering_lut_entry >> 16) & 0xffff);
        }

        // read ae_5bin static
        rc = system_chardev_lock();
        if (rc == 0) {
            isp_ctx_node_t *cn;
            struct vio_frame_id frmid;
            cn = isp_ctx_get_node(fw_id, ISP_AE_5BIN, FREEQ);
            if (cn) {
                vio_get_sif_frame_info(fw_id, &frmid);
                cn->ctx.frame_id = frmid.frame_id;
                cn->ctx.timestamps = frmid.timestamps;
                memcpy(cn->base, p_sbuf_ae->hist4, sizeof(p_sbuf_ae->hist4));
                cn->ctx.crc16 = crc16(~0, cn->base, sizeof(p_sbuf_ae->hist4));
                isp_ctx_put_node(fw_id, cn, ISP_AE_5BIN, DONEQ);

                pr_debug("ae_5bin stats frame id %d\n", cn->ctx.frame_id);
            }
        }
        if (rc == 0)
            system_chardev_unlock();
    }//endif p_ctx->isp_ae_5bin_stats_on

	// read lumvar
    if (p_ctx->isp_lumvar_stats_on) {
        for ( i = 0; i < 512; i++ ) {
            lumvar[i] = acamera_lumvar_stats_mem_array_data_read(p_fsm->cmn.isp_base, i);
            LOG(LOG_DEBUG, "lumvar: %u. data %d \n", i, lumvar[i]);
        }
        rc = system_chardev_lock();
        if (rc == 0) {
            isp_ctx_node_t *cn;
            struct vio_frame_id frmid;
            cn = isp_ctx_get_node(fw_id, ISP_LUMVAR, FREEQ);
            if (cn) {
                vio_get_sif_frame_info(fw_id, &frmid);
                cn->ctx.frame_id = frmid.frame_id;
                cn->ctx.timestamps = frmid.timestamps;
                memcpy(cn->base, lumvar, sizeof(lumvar));
                cn->ctx.crc16 = crc16(~0, cn->base, sizeof(lumvar));
                isp_ctx_put_node(fw_id, cn, ISP_LUMVAR, DONEQ);

                pr_debug("lumvar stats frame id %d\n", cn->ctx.frame_id);
            }
        }
        if (rc == 0)
            system_chardev_unlock();
    }

    /* read done, set the buffer back for future using */
    sbuf.buf_status = SBUF_STATUS_DATA_DONE;

    if ( sbuf_set_item( fw_id, &sbuf ) ) {
        LOG( LOG_ERR, "Error: Failed to set sbuf, return." );
        return;
    }
    LOG( LOG_DEBUG, "Set sbuf ok, idx: %u, status: %u, addr: %p.", sbuf.buf_idx, sbuf.buf_status, sbuf.buf_base );

    ae_flow.frame_id_current = acamera_fsm_util_get_cur_frame_id( &p_fsm->cmn );
    ae_flow.flow_state = MON_ALG_FLOW_STATE_INPUT_READY;
    acamera_fsm_mgr_set_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_SET_MON_AE_FLOW, &ae_flow, sizeof( ae_flow ) );

    if (time_takes_check)
        do_gettimeofday(&tv2);

    if (time_takes_check) {
        if (tv2.tv_usec >= tv1.tv_usec)
            irq_interval = tv2.tv_usec - tv1.tv_usec;
        else
            irq_interval = 1000000 + tv2.tv_usec - tv1.tv_usec;
        pr_debug("cost %ld.%06ld\n", tv2.tv_sec - tv1.tv_sec, irq_interval);
    }

    LOG( LOG_INFO, "AE flow: INPUT_READY: frame_id_tracking: %d, cur frame_id: %u.", ae_flow.frame_id_tracking, ae_flow.frame_id_current );
}

void ae_update_zone_weight_data( AE_fsm_ptr_t p_fsm )
{
        uint32_t i;
        uint32_t num = p_fsm->ae_1024bin_weight.zones_size;
        if (num > ISP_METERING_ZONES_AE5_V * ISP_METERING_ZONES_AE5_H)
                num = ISP_METERING_ZONES_AE5_V * ISP_METERING_ZONES_AE5_H;
        for (i = 0; i < num; i++) {
                acamera_isp_metering_hist_aexp_zones_weight_write( p_fsm->cmn.isp_base, i, p_fsm->ae_1024bin_weight.zones_weight[i]);
        }
}

void AE_fsm_process_interrupt( AE_fsm_const_ptr_t p_fsm, uint8_t irq_event )
{
    if ( acamera_fsm_util_is_irq_event_ignored( (fsm_irq_mask_t *)( &p_fsm->mask ), irq_event ) )
        return;

    switch ( irq_event ) {
    case ACAMERA_IRQ_AE_STATS:
        ae_read_full_histogram_data( (AE_fsm_ptr_t)p_fsm );
        fsm_raise_event( p_fsm, event_id_ae_stats_ready );
        break;
    case ACAMERA_IRQ_FRAME_END:
        ae_update_zone_weight_data( (AE_fsm_ptr_t)p_fsm );
        break;
    }
}

void ae_set_new_param( AE_fsm_ptr_t p_fsm, sbuf_ae_t *p_sbuf_ae )
{
    if ( p_sbuf_ae->frame_id == p_fsm->frame_id_tracking ) {
        pr_info("ae Same frame used twice, frame_id: %u. hw frmid %u", p_sbuf_ae->frame_id,
                acamera_fsm_util_get_cur_frame_id(&p_fsm->cmn));
        return;
    }

    p_fsm->new_exposure_log2 = p_sbuf_ae->ae_exposure;
    p_fsm->new_exposure_ratio = p_sbuf_ae->ae_exposure_ratio;
    p_fsm->frame_id_tracking = p_sbuf_ae->frame_id;
    p_fsm->state = p_sbuf_ae->state;
    //prove extern ae
    p_fsm->external_ae_enable = p_sbuf_ae->external_ae_enable;
    p_fsm->sensor_ctrl_enable = p_sbuf_ae->sensor_ctrl_enable;
    memcpy(&p_fsm->ae_info, &p_sbuf_ae->ae_info, sizeof(p_fsm->ae_info));
    memcpy(&p_fsm->ae_1024bin_weight, &p_sbuf_ae->ae_1024bin_weight, sizeof(p_fsm->ae_1024bin_weight));

    if ( p_fsm->frame_id_tracking ) {
        fsm_param_mon_alg_flow_t ae_flow;
        ae_flow.frame_id_tracking = p_fsm->frame_id_tracking;
        ae_flow.frame_id_current = acamera_fsm_util_get_cur_frame_id( &p_fsm->cmn );
        ae_flow.flow_state = MON_ALG_FLOW_STATE_OUTPUT_READY;

        acamera_fsm_mgr_set_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_SET_MON_AE_FLOW, &ae_flow, sizeof( ae_flow ) );

        LOG( LOG_INFO, "AE flow: OUTPUT_READY: frame_id_tracking: %d, cur frame_id: %u.", ae_flow.frame_id_tracking, ae_flow.frame_id_current );
    }

    fsm_raise_event( p_fsm, event_id_ae_result_ready );
}

static inline uint32_t full_ratio_to_adjaced( const fsm_param_sensor_info_t *sensor_info, uint32_t ratio )
{
    switch ( sensor_info->sensor_exp_number ) {
    case 4:
        return acamera_math_exp2( acamera_log2_fixed_to_fixed( ratio, 6, 8 ) / 3, 8, 6 ) >> 6;
        break;
    case 3:
        return acamera_sqrt32( ratio >> 6 );
        break;
    default:
    case 2:
        return ratio >> 6;
        break;
    }
}

int ae_calculate_exposure( AE_fsm_ptr_t p_fsm )
{
    int rc = 0;
    fsm_param_sensor_info_t sensor_info;
    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_SENSOR_INFO, NULL, 0, &sensor_info, sizeof( sensor_info ) );

    fsm_param_exposure_target_t exp_target;
    exp_target.exposure_log2 = p_fsm->new_exposure_log2;
    exp_target.exposure_ratio = p_fsm->new_exposure_ratio;
    exp_target.frame_id_tracking = p_fsm->frame_id_tracking;
    memcpy(&exp_target.ae_out_info, &p_fsm->ae_info, sizeof(exp_target.ae_out_info));
    exp_target.external_ae_enable = p_fsm->external_ae_enable;
    exp_target.sensor_ctrl_enable = p_fsm->sensor_ctrl_enable;

    ACAMERA_FSM2CTX_PTR( p_fsm )
        ->stab.global_exposure_ratio = full_ratio_to_adjaced( &sensor_info, p_fsm->new_exposure_ratio );

    /* CMOS will skip exposure set operation in some conditions, so we need to check the apply result, ret 0 menas succeed  */
    if ( !acamera_fsm_mgr_set_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_SET_EXPOSURE_TARGET, &exp_target, sizeof( exp_target ) ) ) {
        p_fsm->exposure_log2 = p_fsm->new_exposure_log2;
        p_fsm->exposure_ratio = p_fsm->new_exposure_ratio;

        ACAMERA_FSM2CTX_PTR( p_fsm )
            ->stab.global_exposure = ( acamera_math_exp2( p_fsm->new_exposure_log2, LOG2_GAIN_SHIFT, 6 ) );

        // indicate we have new AE parameter, and should post event.
        rc = 1;

        LOG( LOG_INFO, "AE applied OK, exposure_log2 : %d, exposure_ratio: %u.", exp_target.exposure_log2, exp_target.exposure_ratio );
    } else {
        LOG( LOG_INFO, "AE applied failed, exposure_log2 : %d, exposure_ratio: %u.", exp_target.exposure_log2, exp_target.exposure_ratio );
    }

    return rc;
}
