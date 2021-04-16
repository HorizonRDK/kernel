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

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "acamera_fw.h"
#include "acamera_command_api.h"
#include "acamera_logger.h"
#include "metadata_api.h"
#include "metadata_fsm.h"

#if ISP_HAS_CMOS_FSM
#include "cmos_fsm.h"
#endif


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_METADATA
#else
#define CUR_MOD_NAME LOG_MODULE_METADATA
#endif

//#define DEBUG_METADATA_FSM
#define MM_PROC_DIR   "hbmmst"

extern void *acamera_get_ctx_ptr(uint32_t ctx_id);

firmware_metadata_t g_fw_md[FIRMWARE_CONTEXT_NUMBER];
static int ctx_id;

static struct proc_dir_entry *isp_entry;
static struct proc_dir_entry *isp_proc_dir;

static void isp_proc_gain( struct seq_file *seq, const char *name, int32_t gain_log2 )
{
    int32_t gain_log10 = gain_log2 * 6; // 1% error
    uint16_t gain_dec = ( uint16_t )( ( ( ACAMERA_ABS( gain_log10 ) & ( ( 1 << LOG2_GAIN_SHIFT ) - 1 ) ) * 1000 ) >> LOG2_GAIN_SHIFT );
    int32_t gain_ones = acamera_math_exp2( gain_log2, LOG2_GAIN_SHIFT, LOG2_GAIN_SHIFT ); // 1% error
    uint16_t gain_ones_dec = ( uint16_t )( ( ( ACAMERA_ABS( gain_ones ) & ( ( 1 << LOG2_GAIN_SHIFT ) - 1 ) ) * 1000 ) >> LOG2_GAIN_SHIFT );
    seq_printf(seq, "%s gain %d.%03d = %d.%03d dB\n", name, gain_ones >> LOG2_GAIN_SHIFT, gain_ones_dec, gain_log10 >> LOG2_GAIN_SHIFT, gain_dec );
}

static int isp_proc_show(struct seq_file *seq, void *v)
{
    int i;
    firmware_metadata_t *md;
    const char *modes[4] = {"Linear", "FS HDR", "Native HDR", "FS Linear"};
    const char *cfa_pattern[4] = {"RGGB", "GRBG", "GBRG", "BGGR"};
    acamera_context_t *p_ctx;

    for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {

        p_ctx = acamera_get_ctx_ptr(i);
        if (p_ctx && p_ctx->initialized == 0)
            continue;

        md = &g_fw_md[i];
        if (md->isp_mode > 3)
            md->isp_mode = 0;
        if (md->rggb_start > 3)
            md->rggb_start = 0;

        seq_printf(seq, "===ctx id %d===\n", i);
        // seq_printf(seq, "Format: %d\n", md->image_format );
        seq_printf(seq, "Sensor width: %d\n", md->sensor_width);
        seq_printf(seq, "Sensor height: %d\n", md->sensor_height);
        seq_printf(seq, "Sensor bits: %d\n", md->sensor_bits );
        seq_printf(seq, "RGGB start: %s\n", cfa_pattern[md->rggb_start] );
        seq_printf(seq, "ISP mode: %s\n", modes[md->isp_mode] );
        seq_printf(seq, "FPS: %d.%02d\n", md->fps >> 8, ( ( md->fps & 0xFF ) * 100 ) >> 8 );

        seq_printf(seq, "Integration time: %d lines %lld.%02lld ms\n", md->int_time, md->int_time_ms / 100, md->int_time_ms % 100 );
        seq_printf(seq, "Integration time medium: %d\n", md->int_time_medium );
        seq_printf(seq, "Integration time long: %d\n", md->int_time_long );
        isp_proc_gain( seq, "A", md->again );
        isp_proc_gain( seq, "D", md->dgain );
        isp_proc_gain( seq, "ISP", md->isp_dgain );
        // seq_printf(seq, "Equivalent Exposure: %d lines\n", md->exposure_equiv );
        seq_printf(seq, "Exposure_log2: %d\n", md->exposure );
        // seq_printf(seq, "Gain_log2: %d\n", md->gain_log2 );

        seq_printf(seq, "Lens Position: %d\n", md->lens_pos );

        seq_printf(seq, "Antiflicker: %s\n", md->anti_flicker ? "on" : "off" );

        seq_printf(seq, "WB Gain 00: %d\n", md->gain_00 );
        seq_printf(seq, "WB Gain 01: %d\n", md->gain_01 );
        seq_printf(seq, "WB Gain 10: %d\n", md->gain_10 );
        seq_printf(seq, "WB Gain 11: %d\n", md->gain_11 );
        seq_printf(seq, "Black Level 00: %d\n", md->black_level_00 );
        seq_printf(seq, "Black Level 01: %d\n", md->black_level_01 );
        seq_printf(seq, "Black Level 10: %d\n", md->black_level_10 );
        seq_printf(seq, "Black Level 11: %d\n", md->black_level_11 );

        seq_printf(seq, "LSC table: %d\n", md->lsc_table );
        seq_printf(seq, "LSC blend: %d\n", md->lsc_blend );
        seq_printf(seq, "LSC Mesh strength: %d\n", md->lsc_mesh_strength );

        seq_printf(seq, "AWB rg: %lld\n", md->awb_rgain );
        seq_printf(seq, "AWB bg: %lld\n", md->awb_bgain );
        seq_printf(seq, "AWB temperature: %lld\n", md->awb_cct );

        seq_printf(seq, "Sinter strength: %d\n", md->sinter_strength );
        seq_printf(seq, "Sinter strength1: %d\n", md->sinter_strength1 );
        seq_printf(seq, "Sinter strength4: %d\n", md->sinter_strength4 );
        seq_printf(seq, "Sinter thresh1h: %d\n", md->sinter_thresh_1h );
        seq_printf(seq, "Sinter thresh4h: %d\n", md->sinter_thresh_4h );
        seq_printf(seq, "Sinter SAD: %d\n", md->sinter_sad );

        seq_printf(seq, "Temper strength: %d\n", md->temper_strength );

        seq_printf(seq, "Iridix strength: %d\n", md->iridix_strength );

        seq_printf(seq, "Dp threshold1: %d\n", md->dp_threash1 );
        seq_printf(seq, "Dp slope1: %d\n", md->dp_slope1 );
        seq_printf(seq, "Dp threshold2: %d\n", md->dp_threash2 );
        seq_printf(seq, "Dp slope2: %d\n", md->dp_slope2 );

        seq_printf(seq, "Demosaic NP offset: %d\n", md->demosaic_np_offset );

        seq_printf(seq, "Sharpen strength: %d\n", md->fr_sharpern_strength );

        seq_printf(seq, "CCM R_R: 0x%X\n", md->ccm[CCM_R][CCM_R] );
        seq_printf(seq, "CCM R_G: 0x%X\n", md->ccm[CCM_R][CCM_G] );
        seq_printf(seq, "CCM R_B: 0x%X\n", md->ccm[CCM_R][CCM_B] );
        seq_printf(seq, "CCM G_R: 0x%X\n", md->ccm[CCM_G][CCM_R] );
        seq_printf(seq, "CCM G_G: 0x%X\n", md->ccm[CCM_G][CCM_G] );
        seq_printf(seq, "CCM G_B: 0x%X\n", md->ccm[CCM_G][CCM_B] );
        seq_printf(seq, "CCM B_R: 0x%X\n", md->ccm[CCM_B][CCM_R] );
        seq_printf(seq, "CCM B_G: 0x%X\n", md->ccm[CCM_B][CCM_G] );
        seq_printf(seq, "CCM B_B: 0x%X\n", md->ccm[CCM_B][CCM_B] );
    }

    return 0;
}

static int isp_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, isp_proc_show, NULL);
}

static const struct file_operations isp_proc_ops = {
    .open = isp_proc_open,
    .read = seq_read,
    .release = seq_release,
};

void metadata_initialize( metadata_fsm_t *p_fsm )
{
    LOG( LOG_INFO, "Initializing metadata FSM" );
    memset( &p_fsm->cur_metadata, 0x0, sizeof( firmware_metadata_t ) );
    p_fsm->callback_meta = NULL;

    if (!isp_proc_dir) {
        isp_proc_dir = proc_mkdir(MM_PROC_DIR, NULL);
        if (!isp_proc_dir) {
            pr_err("%s proc dir create failed\n", MM_PROC_DIR);
            goto skip;
        }
    }

    if (!isp_entry) {
        isp_entry = proc_create("isp", 0444, isp_proc_dir, &isp_proc_ops);
        if (!isp_entry) {
            pr_err("isp proc file create failed\n");
            remove_proc_entry(MM_PROC_DIR, NULL);
        }
    }

skip:
    p_fsm->mask.repeat_irq_mask = ACAMERA_IRQ_MASK( ACAMERA_IRQ_FRAME_END );
    metadata_request_interrupt( p_fsm, p_fsm->mask.repeat_irq_mask );
}

#if defined( ACAMERA_ISP_SENSOR_OFFSET_WDR_1_BLACK_00_DATASIZE )
#define SENSOR_OUTPUT_BITS ACAMERA_ISP_SENSOR_OFFSET_WDR_1_BLACK_00_DATASIZE
#elif defined( ACAMERA_ISP_SENSOR_OFFSET_BLACK_00_DATASIZE )
#define SENSOR_OUTPUT_BITS ACAMERA_ISP_SENSOR_OFFSET_BLACK_00_DATASIZE
#elif defined( ACAMERA_ISP_OFFSET_BLACK_00_DATASIZE )
#define SENSOR_OUTPUT_BITS ACAMERA_ISP_OFFSET_BLACK_00_DATASIZE
#endif

void metadata_update_meta( metadata_fsm_t *p_fsm )
{
#if defined( ISP_HAS_CMOS_FSM )
    cmos_control_param_t *param_cmos = (cmos_control_param_t *)_GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_CMOS_CONTROL );
#endif
    firmware_metadata_t *md = &p_fsm->cur_metadata;
    acamera_context_t *p_ctx = ACAMERA_FSM2CTX_PTR( p_fsm );
    uintptr_t isp_base = p_fsm->cmn.isp_base;
    uint32_t wdr_mode = 0;
    uint32_t fps = 0;
    int32_t shading_alpha = -1;
    uint32_t avg_GR = -1;
    uint32_t avg_GB = -1;
    int32_t temperature_detected = -1;

    ctx_id = p_ctx->context_id;
    fsm_param_sensor_info_t sensor_info;
    fsm_param_awb_info_t awb_info;

    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_SENSOR_INFO, NULL, 0, &sensor_info, sizeof( sensor_info ) );

#if defined( ISP_HAS_AF_LMS_FSM ) || defined( ISP_HAS_AF_MANUAL_FSM )
    lens_param_t lens_param;
    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_LENS_PARAM, NULL, 0, &lens_param, sizeof( lens_param ) );
#endif

    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_WDR_MODE, NULL, 0, &wdr_mode, sizeof( wdr_mode ) );

#if ISP_HAS_CMOS_FSM
    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_FPS, NULL, 0, &fps, sizeof( fps ) );
#endif

#if ISP_HAS_COLOR_MATRIX_FSM
    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_SHADING_ALPHA, NULL, 0, &shading_alpha, sizeof( shading_alpha ) );
#endif

#if defined( ISP_HAS_AWB_MESH_NBP_FSM ) || defined( ISP_HAS_AWB_MANUAL_FSM )
    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_AWB_INFO, NULL, 0, &awb_info, sizeof( awb_info ) );
    avg_GR = awb_info.avg_GR;
    avg_GB = awb_info.avg_GB;
    temperature_detected = awb_info.temperature_detected;
#endif


#if ISP_HAS_CMOS_FSM
    exposure_set_t exp_set;
    int32_t frame = 3; // NUMBER_OF_USED_BANKS;
#endif


    LOG( LOG_DEBUG, "updating metadata ( isp_base = 0x%x)", isp_base );

    // Frame counter
    md->frame_id = ACAMERA_FSM2CTX_PTR( p_fsm )->isp_frame_counter;

    // Basic info
    md->image_format = 1;
    md->sensor_width = sensor_info.active_width;
    md->sensor_height = sensor_info.active_height;

    md->sensor_bits = sensor_info.sensor_bits;
    md->rggb_start = acamera_isp_top_rggb_start_pre_mirror_read(isp_base);

    md->isp_mode = wdr_mode;
    md->fps = fps;

#if ISP_HAS_CMOS_FSM
    memset( &exp_set, 0x0, sizeof( exposure_set_t ) );

    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_FRAME_EXPOSURE_SET, &frame, sizeof( frame ), &exp_set, sizeof( exp_set ) );

    md->int_time = exp_set.data.integration_time;
    md->int_time_ms = DIV_ROUND_CLOSEST(100000, sensor_info.lines_per_second) * md->int_time;
    md->int_time_medium = exp_set.data.integration_time_medium;
    md->int_time_long = exp_set.data.integration_time_long;
    md->again = exp_set.info.again_log2;
    md->dgain = exp_set.info.dgain_log2;
    md->isp_dgain = exp_set.info.isp_dgain_log2;
    md->exposure = exp_set.info.exposure_log2;
    md->exposure_equiv = acamera_math_exp2( md->exposure, LOG2_GAIN_SHIFT, 0 );
    md->gain_log2 = ( md->again + md->dgain + md->isp_dgain ) >> LOG2_GAIN_SHIFT;
#endif

// Tuning params
#if defined( ISP_HAS_AF_LMS_FSM ) || defined( ISP_HAS_AF_MANUAL_FSM )
    md->lens_pos = lens_param.next_pos;
#elif defined( ISP_HAS_AF_MON_FSM )
    md->lens_pos = p_ctx->isp.AF_mon_fsm.loc[0].pos;
#else
    md->lens_pos = -1;
#endif
#if defined( ISP_HAS_CMOS_FSM )
    md->anti_flicker = param_cmos->global_antiflicker_enable;
#endif


    md->gain_00 = acamera_isp_white_balance_gain_00_read( isp_base );
    md->gain_01 = acamera_isp_white_balance_gain_01_read( isp_base );
    md->gain_10 = acamera_isp_white_balance_gain_10_read( isp_base );
    md->gain_11 = acamera_isp_white_balance_gain_11_read( isp_base );

    md->black_level_00 = (int)acamera_isp_sensor_offset_pre_shading_offset_00_read( isp_base );
    md->black_level_01 = (int)acamera_isp_sensor_offset_pre_shading_offset_01_read( isp_base );
    md->black_level_10 = (int)acamera_isp_sensor_offset_pre_shading_offset_10_read( isp_base );
    md->black_level_11 = (int)acamera_isp_sensor_offset_pre_shading_offset_11_read( isp_base );

    md->lsc_table = acamera_isp_mesh_shading_mesh_alpha_bank_r_read( isp_base );
    md->lsc_blend = shading_alpha;
    md->lsc_mesh_strength = acamera_isp_mesh_shading_mesh_strength_read( isp_base );

    md->awb_rgain = avg_GR;
    md->awb_bgain = avg_GB;
    md->awb_cct = temperature_detected;

    md->sinter_strength = p_ctx->stab.global_sinter_threshold_target;
    md->sinter_strength1 = acamera_isp_sinter_strength_1_read( isp_base );
    md->sinter_strength4 = acamera_isp_sinter_strength_4_read( isp_base );
    md->sinter_thresh_1h = acamera_isp_sinter_thresh_1h_read( isp_base );
    md->sinter_thresh_4h = acamera_isp_sinter_thresh_4h_read( isp_base );
    if ( acamera_isp_isp_global_parameter_status_sinter_version_read( p_fsm->cmn.isp_base ) ) { //sinter 3 is used
        md->sinter_sad = acamera_isp_sinter_sad_filt_thresh_read( isp_base );
    } else {
        md->sinter_sad = -1;
    }

    md->temper_strength = p_ctx->stab.global_temper_threshold_target;

    md->iridix_strength = acamera_isp_iridix_strength_inroi_read( isp_base );

    md->dp_threash1 = acamera_isp_raw_frontend_dp_threshold_read( isp_base );
    md->dp_slope2 = acamera_isp_raw_frontend_dp_slope_read( isp_base );
    md->dp_threash2 = -1;
    md->dp_slope2 = -1;

    md->demosaic_np_offset = acamera_isp_demosaic_rgb_np_offset_read( isp_base );

    md->fr_sharpern_strength = acamera_isp_fr_sharpen_strength_read( isp_base );

    md->ccm[CCM_R][CCM_R] = acamera_isp_ccm_coefft_r_r_read( isp_base );
    md->ccm[CCM_R][CCM_G] = acamera_isp_ccm_coefft_r_g_read( isp_base );
    md->ccm[CCM_R][CCM_B] = acamera_isp_ccm_coefft_r_b_read( isp_base );
    md->ccm[CCM_G][CCM_R] = acamera_isp_ccm_coefft_g_r_read( isp_base );
    md->ccm[CCM_G][CCM_G] = acamera_isp_ccm_coefft_g_g_read( isp_base );
    md->ccm[CCM_G][CCM_B] = acamera_isp_ccm_coefft_g_b_read( isp_base );
    md->ccm[CCM_B][CCM_R] = acamera_isp_ccm_coefft_b_r_read( isp_base );
    md->ccm[CCM_B][CCM_G] = acamera_isp_ccm_coefft_b_g_read( isp_base );
    md->ccm[CCM_B][CCM_B] = acamera_isp_ccm_coefft_b_b_read( isp_base );

    if (p_ctx->context_id < FIRMWARE_CONTEXT_NUMBER)
        memcpy((void *)&g_fw_md[p_ctx->context_id], (void *)md, sizeof(p_fsm->cur_metadata));
}

#ifdef DEBUG_METADATA_FSM
static void dump_gain( const char *name, int32_t gain_log2 )
{
    int32_t gain_log10 = gain_log2 * 6; // 1% error
    uint16_t gain_dec = ( uint16_t )( ( ( ACAMERA_ABS( gain_log10 ) & ( ( 1 << LOG2_GAIN_SHIFT ) - 1 ) ) * 1000 ) >> LOG2_GAIN_SHIFT );
    int32_t gain_ones = acamera_math_exp2( gain_log2, LOG2_GAIN_SHIFT, LOG2_GAIN_SHIFT ); // 1% error
    uint16_t gain_ones_dec = ( uint16_t )( ( ( ACAMERA_ABS( gain_ones ) & ( ( 1 << LOG2_GAIN_SHIFT ) - 1 ) ) * 1000 ) >> LOG2_GAIN_SHIFT );
    LOG( LOG_INFO, "%s gain %d.%03d = %d.%03d dB", name, gain_ones >> LOG2_GAIN_SHIFT, gain_ones_dec, gain_log10 >> LOG2_GAIN_SHIFT, gain_dec );
}

static void metadata_dump( const firmware_metadata_t *md )
{
    const char *modes[WDR_MODE_COUNT] = {"Linear", "FS HDR", "Native HDR", "FS Linear"};

    LOG( LOG_INFO, "Format: %d", md->image_format );
    LOG( LOG_INFO, "Sensor bits: %d", md->sensor_bits );
    LOG( LOG_INFO, "RGGB start: %d", md->rggb_start );
    LOG( LOG_INFO, "ISP mode: %s", modes[md->isp_mode] );
    LOG( LOG_INFO, "FPS: %d.%02d", md->fps >> 8, ( ( md->fps & 0xFF ) * 100 ) >> 8 );

    LOG( LOG_INFO, "Integration time: %d lines %lld.%02lld ms", md->int_time, md->int_time_ms / 100, md->int_time_ms % 100 );
    LOG( LOG_INFO, "Integration time medium: %d", md->int_time_medium );
    LOG( LOG_INFO, "Integration time long: %d", md->int_time_long );
    dump_gain( "A", md->again );
    dump_gain( "D", md->dgain );
    dump_gain( "ISP", md->isp_dgain );
    LOG( LOG_INFO, "Equivalent Exposure: %d lines", acamera_math_exp2( md->exposure, LOG2_GAIN_SHIFT, 0 ) );
    LOG( LOG_INFO, "Exposure_log2: %d", md->exposure );
    LOG( LOG_INFO, "Gain_log2: %d", md->gain_log2 );

    LOG( LOG_INFO, "Lens Position: %d", md->lens_pos );

    LOG( LOG_INFO, "Antiflicker: %s", md->anti_flicker ? "on" : "off" );

    LOG( LOG_INFO, "Gain 00: %d", md->gain_00 );
    LOG( LOG_INFO, "Gain 01: %d", md->gain_01 );
    LOG( LOG_INFO, "Gain 10: %d", md->gain_10 );
    LOG( LOG_INFO, "Gain 11: %d", md->gain_11 );
    LOG( LOG_INFO, "Black Level 00: %d", md->black_level_00 );
    LOG( LOG_INFO, "Black Level 01: %d", md->black_level_01 );
    LOG( LOG_INFO, "Black Level 10: %d", md->black_level_10 );
    LOG( LOG_INFO, "Black Level 11: %d", md->black_level_11 );

    LOG( LOG_INFO, "LSC table: %d", md->lsc_table );
    LOG( LOG_INFO, "LSC blend: %d", md->lsc_blend );
    LOG( LOG_INFO, "LSC Mesh strength: %d", md->lsc_mesh_strength );

    LOG( LOG_INFO, "AWB rg: %lld", md->awb_rgain );
    LOG( LOG_INFO, "AWB bg: %lld", md->awb_bgain );
    LOG( LOG_INFO, "AWB temperature: %lld", md->awb_cct );

    LOG( LOG_INFO, "Sinter strength: %d", md->sinter_strength );
    LOG( LOG_INFO, "Sinter strength1: %d", md->sinter_strength1 );
    LOG( LOG_INFO, "Sinter strength4: %d", md->sinter_strength4 );
    LOG( LOG_INFO, "Sinter thresh1h: %d", md->sinter_thresh_1h );
    LOG( LOG_INFO, "Sinter thresh4h: %d", md->sinter_thresh_4h );
    LOG( LOG_INFO, "Sinter SAD: %d", md->sinter_sad );

    LOG( LOG_INFO, "Temper strength: %d", md->temper_strength );

    LOG( LOG_INFO, "Iridix strength: %d", md->iridix_strength );

    LOG( LOG_INFO, "Dp threshold1: %d", md->dp_threash1 );
    LOG( LOG_INFO, "Dp slope1: %d", md->dp_slope1 );
    LOG( LOG_INFO, "Dp threshold2: %d", md->dp_threash2 );
    LOG( LOG_INFO, "Dp slope2: %d", md->dp_slope2 );

    LOG( LOG_INFO, "Sharpening directional: %d", md->sharpening_directional );
    LOG( LOG_INFO, "Sharpening unidirectional: %d", md->sharpening_unidirectional );
    LOG( LOG_INFO, "Demosaic NP offset: %d", md->demosaic_np_offset );

    LOG( LOG_INFO, "FR sharpen strength: %d", md->fr_sharpern_strength );
    LOG( LOG_INFO, "DS1 sharpen strength: %d", md->ds1_sharpen_strength );
    LOG( LOG_INFO, "DS2 sharpen strength: %d", md->ds2_sharpen_strength );

    LOG( LOG_INFO, "CCM R_R: 0x%X", md->ccm[CCM_R][CCM_R] );
    LOG( LOG_INFO, "CCM R_G: 0x%X", md->ccm[CCM_R][CCM_G] );
    LOG( LOG_INFO, "CCM R_B: 0x%X", md->ccm[CCM_R][CCM_B] );
    LOG( LOG_INFO, "CCM G_R: 0x%X", md->ccm[CCM_G][CCM_R] );
    LOG( LOG_INFO, "CCM G_G: 0x%X", md->ccm[CCM_G][CCM_G] );
    LOG( LOG_INFO, "CCM G_B: 0x%X", md->ccm[CCM_G][CCM_B] );
    LOG( LOG_INFO, "CCM B_R: 0x%X", md->ccm[CCM_B][CCM_R] );
    LOG( LOG_INFO, "CCM B_G: 0x%X", md->ccm[CCM_B][CCM_G] );
    LOG( LOG_INFO, "CCM B_B: 0x%X", md->ccm[CCM_B][CCM_B] );
}
#endif

void metadata_post_meta( metadata_fsm_const_ptr_t p_fsm )
{
    const firmware_metadata_t *fw_meta = &p_fsm->cur_metadata;

#ifdef DEBUG_METADATA_FSM
    metadata_dump( fw_meta );
#endif

    if ( p_fsm->callback_meta ) {
        LOG( LOG_DEBUG, "Posting metadata to registered callback." );
        p_fsm->callback_meta( ACAMERA_FSM2CTX_PTR( p_fsm ), fw_meta );
    } else {
        /* enable ISP_HAS_META_CB in your set file when you see this log */
        LOG( LOG_INFO, "No metadata callback registered." );
    }
}

void metadata_regist_callback( metadata_fsm_t *p_fsm, metadata_callback_t cb )
{
    p_fsm->callback_meta = cb;
}

void metadata_fsm_process_interrupt( metadata_fsm_const_ptr_t p_fsm, uint8_t irq_event )
{
    if ( acamera_fsm_util_is_irq_event_ignored( (fsm_irq_mask_t *)( &p_fsm->mask ), irq_event ) )
        return;
    switch ( irq_event ) {
    case ACAMERA_IRQ_FRAME_END:
        fsm_raise_event( p_fsm, event_id_metadata_update );
        fsm_raise_event( p_fsm, event_id_metadata_ready );
        break;
    }
}
