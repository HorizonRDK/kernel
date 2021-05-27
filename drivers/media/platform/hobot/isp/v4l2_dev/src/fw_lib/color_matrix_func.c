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

#include "acamera_math.h"

#include "acamera_mesh_shading_mem_config.h"

#include "acamera_math.h"
#include "acamera_command_api.h"
#include "color_matrix_fsm.h"
#include "awb_manual_fsm.h"
#include "acamera_lut3d_mem_config.h"

#include "system_stdlib.h"

#ifndef AWB_LIGHT_SOURCE_A
#define AWB_LIGHT_SOURCE_A 0x01
#endif

#ifndef AWB_LIGHT_SOURCE_D40
#define AWB_LIGHT_SOURCE_D40 0x02
#endif

#ifndef AWB_LIGHT_SOURCE_D50
#define AWB_LIGHT_SOURCE_D50 0x03
#endif


#define SHADING_SET_TABLE

/*
    register          actually light    name in calibration
    --------------------------------------------------------
    bank0/page[0]     A			        A
    bank1/page[1]     U30		        TL84
    bank2/page[2]     TL84		        D50
    bank3/page[3]     D50		        D65
*/
#define MESH_SHADING_LS_A_BANK 0
#define MESH_SHADING_LS_U30_BANK 1
#define MESH_SHADING_LS_TL84_BANK 2
#define MESH_SHADING_LS_D50_BANK 3

#define OV_08835_MESH_SHADING_LS_A_BANK 0
#define OV_08835_MESH_SHADING_LS_D40_BANK 1
#define OV_08835_MESH_SHADING_LS_D50_BANK 2

// threshold for the LSC table hysterisis.
#define AWB_DLS_LIGHT_SOURCE_D40_D50_BORDER_low ( ( AWB_LIGHT_SOURCE_D50_TEMPERATURE + AWB_LIGHT_SOURCE_D40_TEMPERATURE ) >> 1 ) - 200
#define AWB_DLS_LIGHT_SOURCE_D40_D50_BORDER_high ( ( AWB_LIGHT_SOURCE_D40_TEMPERATURE + AWB_LIGHT_SOURCE_D50_TEMPERATURE ) >> 1 ) + 200
#define AWB_DLS_LIGHT_SOURCE_A_D40_BORDER_low ( ( AWB_LIGHT_SOURCE_A_TEMPERATURE + AWB_LIGHT_SOURCE_D40_TEMPERATURE ) >> 1 ) - 200
#define AWB_DLS_LIGHT_SOURCE_A_D40_BORDER_high ( ( AWB_LIGHT_SOURCE_D40_TEMPERATURE + AWB_LIGHT_SOURCE_A_TEMPERATURE ) >> 1 ) + 200
//==============Math Functions========================================================

#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_COLOR_MATRIX
#else
#define CUR_MOD_NAME LOG_MODULE_COLOR_MATRIX
#endif

void color_matrix_update_CCMs( color_matrix_fsm_t *p_fsm );

static void matrix_matrix_multiply( int16_t *a1, int16_t *a2, int16_t *result, int dim1, int dim2, int dim3 )
{
    int i, j, k;

    for ( i = 0; i < dim1; ++i ) {
        for ( j = 0; j < dim3; ++j ) {
            int32_t temp = 0;
            for ( k = 0; k < dim2; ++k )
                temp += ( ( (int32_t)a1[i * dim2 + k] * a2[k * dim3 + j] ) >> 8 );
            result[i * dim3 + j] = (int16_t)temp;
        }
    }
}


uint16_t color_matrix_complement_to_direct( int16_t v )
{
    uint16_t result;

    if ( v >= 0 )
        result = v;
    else {
        result = -v;
        result |= ( 1 << 12 );
    }
    return result;
}

int16_t color_matrix_direct_to_complement( uint16_t v )
{
    int16_t result;
    result = v & ( ~( 1 << 15 ) );
    if ( v & ( 1 << 15 ) )
        result = -result;

    return result;
}

//==============Saturation Related Functions========================================================

void saturation_modulate_strength( color_matrix_fsm_ptr_t p_fsm )
{
    uint16_t strength;
    int32_t total_gain = 0;

    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_CMOS_TOTAL_GAIN, NULL, 0, &total_gain, sizeof( total_gain ) );

    uint16_t log2_gain = total_gain >> ( LOG2_GAIN_SHIFT - 8 );
    uint32_t ccm_saturation_table_idx = CALIBRATION_SATURATION_STRENGTH;
    modulation_entry_t *ccm_saturation_table = _GET_MOD_ENTRY16_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), ccm_saturation_table_idx );
    uint32_t ccm_saturation_table_len = _GET_ROWS( ACAMERA_FSM2CTX_PTR( p_fsm ), ccm_saturation_table_idx );
    strength = acamera_calc_modulation_u16( log2_gain, ccm_saturation_table, ccm_saturation_table_len );
    ACAMERA_FSM2CTX_PTR( p_fsm )
        ->stab.global_saturation_target = ( strength );
}


static void color_mat_calculate_saturation_matrix( int16_t *saturation_matrix, uint8_t saturation )
{
    const int16_t identity[9] = {0x0100, 0x0000, 0x0000, 0x0000, 0x0100, 0x0000, 0x0000, 0x0000, 0x0100};
    const int16_t black_white[9] = {0x004c, 0x0096, 0x001d, 0x004c, 0x0096, 0x001d, 0x004c, 0x0096, 0x001d};
    int i;
    int16_t alpha;
    // (1 - saturation)
    alpha = (int16_t)0x100 - ( (uint16_t)saturation << 1 );

    for ( i = 0; i < 9; ++i ) {
        int16_t result;
        // (1. - saturation) * _black_white
        result = ( (int32_t)alpha * black_white[i] + 0x80 ) >> 8;
        // += (saturation * _identity)
        result = result + ( ( (int32_t)identity[i] * ( (uint16_t)saturation << 1 ) + 0x80 ) >> 8 );
        saturation_matrix[i] = result;
    }
}

//==============Mesh Shading Related Functions========================================================

static void mesh_shading_modulate_strength( color_matrix_fsm_ptr_t p_fsm )
{
    int32_t total_gain = 0;

    if ( ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_manual_shading == 0 ) {
        acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_CMOS_TOTAL_GAIN, NULL, 0, &total_gain, sizeof( total_gain ) );
        uint16_t log2_gain = total_gain >> ( LOG2_GAIN_SHIFT - 8 );
        uint16_t strength = acamera_calc_modulation_u16( log2_gain, _GET_MOD_ENTRY16_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MESH_SHADING_STRENGTH ), _GET_ROWS( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MESH_SHADING_STRENGTH ) );
        acamera_isp_mesh_shading_mesh_strength_write( p_fsm->cmn.isp_base, strength );
	p_fsm->shading_mesh_strength = strength;
    } else {
	acamera_isp_mesh_shading_mesh_strength_write( p_fsm->cmn.isp_base, p_fsm->manual_shading_mesh_strength );
	p_fsm->shading_mesh_strength = p_fsm->manual_shading_mesh_strength;
    }
}


// mode - one of linear, native, fs_lin or fs_hdr. see EWDRModeID enum.
void color_matrix_shading_mesh_reload( color_matrix_fsm_ptr_t p_fsm )
{
    int i, j, k, p;
    uint8_t *mesh_page[4][3] = {{NULL, NULL, NULL}, {NULL, NULL, NULL}, {NULL, NULL, NULL}, {NULL, NULL, NULL}};

    uint8_t mirror = !acamera_isp_top_bypass_mirror_read( p_fsm->cmn.isp_base );

    
    // assume that all mesh tables have identical size
    uint32_t mesh_size = _GET_COLS( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_A_R );


    // determine the shading size. assume the tables have identical dimentions NxN
    uint32_t dim = acamera_sqrt32(mesh_size);

    uint32_t shading_threshold_len = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_TEMPER_THRESHOLD );

    //for mesh shading light switching
    // acamera_isp_top_bypass_mesh_shading_write( p_fsm->cmn.isp_base, 0 );
    acamera_isp_mesh_shading_mesh_page_r_write( p_fsm->cmn.isp_base, 0x0 );
    acamera_isp_mesh_shading_mesh_page_g_write( p_fsm->cmn.isp_base, 0x1 );
    acamera_isp_mesh_shading_mesh_page_b_write( p_fsm->cmn.isp_base, 0x2 );
    acamera_isp_mesh_shading_mesh_width_write( p_fsm->cmn.isp_base, dim-1 );
    acamera_isp_mesh_shading_mesh_height_write( p_fsm->cmn.isp_base, dim-1 );
    acamera_isp_mesh_shading_mesh_scale_write( p_fsm->cmn.isp_base, 1 );
    acamera_isp_mesh_shading_mesh_alpha_mode_write( p_fsm->cmn.isp_base, 2 );


    mesh_page[0][0] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_A_R );
    mesh_page[0][1] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_A_G );
    mesh_page[0][2] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_A_B );
    mesh_page[1][0] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_TL84_R );
    mesh_page[1][1] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_TL84_G );
    mesh_page[1][2] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_TL84_B );
    if (shading_threshold_len == 8) {
        pr_debug("use new shading update logic\n");
        mesh_page[2][0] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_D50_R );
        mesh_page[2][1] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_D50_G );
        mesh_page[2][2] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_D50_B );
    } else {
        pr_debug("use default shading update logic\n");
        mesh_page[2][0] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_D65_R );
        mesh_page[2][1] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_D65_G );
        mesh_page[2][2] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_D65_B );
    }
    mesh_page[3][0] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_D65_R );
    mesh_page[3][1] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_D65_G );
    mesh_page[3][2] = _GET_UCHAR_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_LS_D65_B );

    for ( i = 0; i < 3; ++i ) {
        for ( j = 0; j < dim; ++j ) {
            for ( k = 0; k < dim; ++k ) {
                p = mirror ? ( dim - 1 - k ) : k; // for mirror images shading must be mirrored
                acamera_mesh_shading_mem_array_data_write( p_fsm->cmn.isp_base, i * 32 * 32 + j * 32 + p,     ( (uint32_t)mesh_page[0][i][j * dim + k] << 0  ) + 
                                                                                                               ( (uint32_t)mesh_page[1][i][j * dim + k] << 8  ) + 
                                                                                                               ( (uint32_t)mesh_page[2][i][j * dim + k] << 16 ) + 
                                                                                                               ( (uint32_t)mesh_page[3][i][j * dim + k] << 24 ) );
            }
        }
    }

    acamera_isp_mesh_shading_enable_write( p_fsm->cmn.isp_base, 1 );
    acamera_isp_mesh_shading_mesh_show_write( p_fsm->cmn.isp_base, 0 );
}


//==============Color Matrix Related Functions========================================================
void color_matrix_recalculate( color_matrix_fsm_t *p_fsm )
{
    uint32_t saturation;
    int i;
    // Saturation matrix
    if ( ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_manual_saturation == 0 ) //  Do not update values if manual mode
    {
        saturation_modulate_strength( p_fsm );
    }

    saturation = ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_saturation_target;
    color_mat_calculate_saturation_matrix( p_fsm->color_saturation_matrix, (uint8_t)saturation );
    // New colour management:
    matrix_matrix_multiply( p_fsm->color_saturation_matrix, p_fsm->color_correction_matrix, p_fsm->color_matrix, 3, 3, 3 );

    for ( i = 0; i < 9; ++i ) {
        p_fsm->color_matrix[i] = color_matrix_complement_to_direct( p_fsm->color_matrix[i] );
    }
}

static void write_CCM_to_purple_fringe( color_matrix_fsm_t *p_fsm )
{
    int16_t ccm_coeff[9];
    int16_t *pf_ccm;
    uint32_t ccm_blend_flag = 0;

    uint32_t ccm_threshold_len = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_CCM_TEMPER_THRESHOLD );
    const uint32_t *p_ccm_threshold = _GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_CCM_TEMPER_THRESHOLD );
    if (ccm_threshold_len > 7) {
	ccm_blend_flag = p_ccm_threshold[6];
    }

    switch ( p_fsm->light_source_ccm ) {
    case AWB_LIGHT_SOURCE_A:
	if (ccm_blend_flag) {
		pf_ccm = p_fsm->color_matrix_CCM;
	} else {
		pf_ccm = p_fsm->color_matrix_A;
	}
        break;
    case AWB_LIGHT_SOURCE_D40:
	if (ccm_blend_flag) {
		pf_ccm = p_fsm->color_matrix_CCM;
	} else {
		pf_ccm = p_fsm->color_matrix_D40;
	}
        break;
    case AWB_LIGHT_SOURCE_D50:
	if (ccm_blend_flag) {
		pf_ccm = p_fsm->color_matrix_CCM;
	} else {
		pf_ccm = p_fsm->color_matrix_D50;
	}
        break;
    default:
        pf_ccm = p_fsm->color_matrix_one;
        break;
    }

    //convert from s7p8 to s4p8
    int i;
    for ( i = 0; i < 9; ++i ) {
        if ( pf_ccm[i] < 0 ) {
            ccm_coeff[i] = 4096 - ( pf_ccm[i] );
        } else {
            ccm_coeff[i] = pf_ccm[i];
        }
    }

    acamera_isp_pf_correction_ccm_coeff_rr_write( p_fsm->cmn.isp_base, ccm_coeff[0] );
    acamera_isp_pf_correction_ccm_coeff_rg_write( p_fsm->cmn.isp_base, ccm_coeff[1] );
    acamera_isp_pf_correction_ccm_coeff_rb_write( p_fsm->cmn.isp_base, ccm_coeff[2] );
    acamera_isp_pf_correction_ccm_coeff_gr_write( p_fsm->cmn.isp_base, ccm_coeff[3] );
    acamera_isp_pf_correction_ccm_coeff_gg_write( p_fsm->cmn.isp_base, ccm_coeff[4] );
    acamera_isp_pf_correction_ccm_coeff_gb_write( p_fsm->cmn.isp_base, ccm_coeff[5] );
    acamera_isp_pf_correction_ccm_coeff_br_write( p_fsm->cmn.isp_base, ccm_coeff[6] );
    acamera_isp_pf_correction_ccm_coeff_bg_write( p_fsm->cmn.isp_base, ccm_coeff[7] );
    acamera_isp_pf_correction_ccm_coeff_bb_write( p_fsm->cmn.isp_base, ccm_coeff[8] );
}


void color_matrix_setup( int16_t *p_matrix, uint16_t CCM_R_R, uint16_t CCM_R_G, uint16_t CCM_R_B, uint16_t CCM_G_R, uint16_t CCM_G_G, uint16_t CCM_G_B, uint16_t CCM_B_R, uint16_t CCM_B_G, uint16_t CCM_B_B )
{
    p_matrix[0] = color_matrix_direct_to_complement( CCM_R_R );
    p_matrix[1] = color_matrix_direct_to_complement( CCM_R_G );
    p_matrix[2] = color_matrix_direct_to_complement( CCM_R_B );

    p_matrix[3] = color_matrix_direct_to_complement( CCM_G_R );
    p_matrix[4] = color_matrix_direct_to_complement( CCM_G_G );
    p_matrix[5] = color_matrix_direct_to_complement( CCM_G_B );

    p_matrix[6] = color_matrix_direct_to_complement( CCM_B_R );
    p_matrix[7] = color_matrix_direct_to_complement( CCM_B_G );
    p_matrix[8] = color_matrix_direct_to_complement( CCM_B_B );
}


void color_matrix_initialize( color_matrix_fsm_t *p_fsm )
{


#if FW_DO_INITIALIZATION
    color_matrix_change_CCMs( p_fsm );

    p_fsm->manual_saturation_enabled = 0;
    p_fsm->saturation_target = CM_SATURATION_TARGET;
    ACAMERA_FSM2CTX_PTR( p_fsm )
        ->stab.global_saturation_target = ( p_fsm->saturation_target );

    uint32_t mode = 0;
    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_WDR_MODE, NULL, 0, &mode, sizeof( mode ) );

    color_matrix_shading_mesh_reload( p_fsm );
    p_fsm->shading_alpha = 0;
    p_fsm->shading_direction = 2; //0 ->inc  1->dec 2->do nothing
    p_fsm->shading_source_previous = AWB_LIGHT_SOURCE_D50;
    p_fsm->manual_shading_mesh_strength = 2048;

    acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
    acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
    acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );

    acamera_isp_mesh_shading_mesh_alpha_r_write( p_fsm->cmn.isp_base, 0 );
    acamera_isp_mesh_shading_mesh_alpha_g_write( p_fsm->cmn.isp_base, 0 );
    acamera_isp_mesh_shading_mesh_alpha_b_write( p_fsm->cmn.isp_base, 0 );

    p_fsm->temperature_threshold[0] = 3000;
    p_fsm->temperature_threshold[1] = 3900;
    p_fsm->temperature_threshold[2] = 4100;
    p_fsm->temperature_threshold[3] = 4900;
    p_fsm->temperature_threshold[4] = 2999;
    p_fsm->temperature_threshold[5] = 3899;
    p_fsm->temperature_threshold[6] = 4099;
    p_fsm->temperature_threshold[7] = 4899;

    p_fsm->ccm_threshold[0] = 2750;
    p_fsm->ccm_threshold[1] = 3100;
    p_fsm->ccm_threshold[2] = 3400;
    p_fsm->ccm_threshold[3] = 3600;
    p_fsm->ccm_threshold[4] = 4100;
    p_fsm->ccm_threshold[5] = 4500;

    ACAMERA_FSM2CTX_PTR( p_fsm )
        ->stab.global_manual_saturation = ( 0 );

   {
        uint32_t i = 0;
        uint32_t lut3d_mem_len = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_LUT3D_MEM );
        const uint32_t *p_lut3d_mem = _GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_LUT3D_MEM );
        LOG( LOG_INFO, "lut3d_mem_len: %d", lut3d_mem_len );
        for ( i = 0; i < lut3d_mem_len; i++ ) {
            acamera_lut3d_mem_array_data_write( p_fsm->cmn.isp_base, i, p_lut3d_mem[i] );
        }
    }

#endif // FW_DO_INITIALIZATION
}

void color_matrix_write( color_matrix_fsm_t *p_fsm )
{

     if ( ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_manual_ccm ) {
	//pr_info
         int16_t *ccm_matrix = ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_ccm_matrix;

         acamera_isp_ccm_coefft_r_r_write( p_fsm->cmn.isp_base, ccm_matrix[0] );
         acamera_isp_ccm_coefft_r_g_write( p_fsm->cmn.isp_base, ccm_matrix[1] );
         acamera_isp_ccm_coefft_r_b_write( p_fsm->cmn.isp_base, ccm_matrix[2] );
         acamera_isp_ccm_coefft_g_r_write( p_fsm->cmn.isp_base, ccm_matrix[3] );
         acamera_isp_ccm_coefft_g_g_write( p_fsm->cmn.isp_base, ccm_matrix[4] );
         acamera_isp_ccm_coefft_g_b_write( p_fsm->cmn.isp_base, ccm_matrix[5] );
         acamera_isp_ccm_coefft_b_r_write( p_fsm->cmn.isp_base, ccm_matrix[6] );
         acamera_isp_ccm_coefft_b_g_write( p_fsm->cmn.isp_base, ccm_matrix[7] );
         acamera_isp_ccm_coefft_b_b_write( p_fsm->cmn.isp_base, ccm_matrix[8] );

    } else if ( p_fsm->manual_CCM ) {
        acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
        acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
        acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
        acamera_isp_mesh_shading_mesh_alpha_r_write( p_fsm->cmn.isp_base, 0 );
        acamera_isp_mesh_shading_mesh_alpha_g_write( p_fsm->cmn.isp_base, 0 );
        acamera_isp_mesh_shading_mesh_alpha_b_write( p_fsm->cmn.isp_base, 0 );

        acamera_isp_ccm_coefft_r_r_write( p_fsm->cmn.isp_base, p_fsm->manual_color_matrix[0] );
        acamera_isp_ccm_coefft_r_g_write( p_fsm->cmn.isp_base, p_fsm->manual_color_matrix[1] );
        acamera_isp_ccm_coefft_r_b_write( p_fsm->cmn.isp_base, p_fsm->manual_color_matrix[2] );
        acamera_isp_ccm_coefft_g_r_write( p_fsm->cmn.isp_base, p_fsm->manual_color_matrix[3] );
        acamera_isp_ccm_coefft_g_g_write( p_fsm->cmn.isp_base, p_fsm->manual_color_matrix[4] );
        acamera_isp_ccm_coefft_g_b_write( p_fsm->cmn.isp_base, p_fsm->manual_color_matrix[5] );
        acamera_isp_ccm_coefft_b_r_write( p_fsm->cmn.isp_base, p_fsm->manual_color_matrix[6] );
        acamera_isp_ccm_coefft_b_g_write( p_fsm->cmn.isp_base, p_fsm->manual_color_matrix[7] );
        acamera_isp_ccm_coefft_b_b_write( p_fsm->cmn.isp_base, p_fsm->manual_color_matrix[8] );

	system_memcpy( ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_ccm_matrix,
               p_fsm->manual_color_matrix,
               sizeof (ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_ccm_matrix) );

    } else {

        acamera_isp_ccm_coefft_r_r_write( p_fsm->cmn.isp_base, p_fsm->color_matrix[0] );
        acamera_isp_ccm_coefft_r_g_write( p_fsm->cmn.isp_base, p_fsm->color_matrix[1] );
        acamera_isp_ccm_coefft_r_b_write( p_fsm->cmn.isp_base, p_fsm->color_matrix[2] );
        acamera_isp_ccm_coefft_g_r_write( p_fsm->cmn.isp_base, p_fsm->color_matrix[3] );
        acamera_isp_ccm_coefft_g_g_write( p_fsm->cmn.isp_base, p_fsm->color_matrix[4] );
        acamera_isp_ccm_coefft_g_b_write( p_fsm->cmn.isp_base, p_fsm->color_matrix[5] );
        acamera_isp_ccm_coefft_b_r_write( p_fsm->cmn.isp_base, p_fsm->color_matrix[6] );
        acamera_isp_ccm_coefft_b_g_write( p_fsm->cmn.isp_base, p_fsm->color_matrix[7] );
        acamera_isp_ccm_coefft_b_b_write( p_fsm->cmn.isp_base, p_fsm->color_matrix[8] );

	system_memcpy( ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_ccm_matrix,
               p_fsm->color_matrix,
               sizeof (ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_ccm_matrix) );
    }
}


void color_matrix_update( color_matrix_fsm_t *p_fsm )
{
	uint32_t ccm_blend_flag = 0;

	/* update ccm threshold when CCM_TEMPER_THRESHOLD is right */
	uint32_t ccm_threshold_len = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_CCM_TEMPER_THRESHOLD );
	const uint32_t *p_ccm_threshold = _GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_CCM_TEMPER_THRESHOLD );
	if (ccm_threshold_len > 7) {
		ccm_blend_flag = p_ccm_threshold[6];
	}
	// tuning param enable blend func
	if (ccm_blend_flag) {
		color_matrix_update_CCMs(p_fsm);
	}
    //    For CCM switching
    if ((p_fsm->light_source_change_frames_left != 0) && (!ccm_blend_flag)) {
        int16_t *p_ccm_prev;
        int16_t *p_ccm_cur = p_fsm->color_correction_matrix;
        int16_t *p_ccm_target;
        int16_t delta;

        switch ( p_fsm->light_source_ccm_previous ) {
        case AWB_LIGHT_SOURCE_A:
            p_ccm_prev = p_fsm->color_matrix_A;
            break;
        case AWB_LIGHT_SOURCE_D40:
            p_ccm_prev = p_fsm->color_matrix_D40;
            break;
        case AWB_LIGHT_SOURCE_D50:
            p_ccm_prev = p_fsm->color_matrix_D50;
            break;
        default:
            p_ccm_prev = p_fsm->color_matrix_one;
            break;
        }

        switch ( p_fsm->light_source_ccm ) {
        case AWB_LIGHT_SOURCE_A:
            p_ccm_target = p_fsm->color_matrix_A;
            break;
        case AWB_LIGHT_SOURCE_D40:
            p_ccm_target = p_fsm->color_matrix_D40;
            break;
        case AWB_LIGHT_SOURCE_D50:
            p_ccm_target = p_fsm->color_matrix_D50;
            break;
        default:
            p_ccm_target = p_fsm->color_matrix_one;
            break;
        }
        int i;
        for ( i = 0; i < 9; ++i ) {
            // smooth transition
            // using curr += (target - curr)/frames_left causes no movement in first half
            // for small movements
            if ( p_fsm->light_source_change_frames > 1 ) {
                delta = ( ( p_ccm_target[i] - p_ccm_prev[i] ) * ( p_fsm->light_source_change_frames - p_fsm->light_source_change_frames_left ) ) / ( p_fsm->light_source_change_frames - 1 ); // division by zero is checked
                p_ccm_cur[i] = p_ccm_prev[i] + delta;
            }
        }
    }

    if ( ACAMERA_FSM2CTX_PTR( p_fsm )->stab.global_manual_shading == 0 ) {
        fsm_param_awb_info_t wb_info;
        acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_AWB_INFO, NULL, 0, &wb_info, sizeof( wb_info ) );
		uint32_t shading_threshold_len = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_TEMPER_THRESHOLD );
		const uint32_t *p_shading_threshold = _GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_SHADING_TEMPER_THRESHOLD );

		if (shading_threshold_len >= 8) {
			if ((p_shading_threshold[0] > p_shading_threshold[1]) ||
				(p_shading_threshold[1] > p_shading_threshold[2]) ||
				(p_shading_threshold[2] > p_shading_threshold[3]) ||
				(p_shading_threshold[3] > p_shading_threshold[4]) ||
				(p_shading_threshold[4] > p_shading_threshold[5]) ||
				(p_shading_threshold[5] > p_shading_threshold[6]) ||
				(p_shading_threshold[6] > p_shading_threshold[7])) {
                    pr_err("invalid shading threshold.\n");
			} else {
				p_fsm->temperature_threshold[0] = p_shading_threshold[0];
				p_fsm->temperature_threshold[1] = p_shading_threshold[1];
				p_fsm->temperature_threshold[2] = p_shading_threshold[2];
				p_fsm->temperature_threshold[3] = p_shading_threshold[3];
				p_fsm->temperature_threshold[4] = p_shading_threshold[4];
				p_fsm->temperature_threshold[5] = p_shading_threshold[5];
				p_fsm->temperature_threshold[6] = p_shading_threshold[6];
				p_fsm->temperature_threshold[7] = p_shading_threshold[7];
			}
		}

        if (shading_threshold_len >= 8) {
            pr_debug("use new shading update logic\n");
            // LSC is completely based on alpha blending and the AWB color temp.
            /* only update shading_threshold when SHADING_TEMPER_THRESHOLD is right */
            pr_debug("temperature %d\n", wb_info.temperature_detected);
            // if temp less < 2750 go to A
            if ( ( wb_info.temperature_detected < p_fsm->temperature_threshold[0] ) )
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_A_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_A_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_A_BANK );
                p_fsm->shading_alpha = 0;
                pr_debug("temp < %d\n", p_fsm->temperature_threshold[0]);
            }
            // if current temp between 2750 and 3100 use blending
            else if ( ( wb_info.temperature_detected > p_fsm->temperature_threshold[0] ) && ( wb_info.temperature_detected < p_fsm->temperature_threshold[1] ) )
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_A_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_A_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_A_BANK );
                if ( p_fsm->temperature_threshold[0] != p_fsm->temperature_threshold[1] )
                    p_fsm->shading_alpha = ( 255 * ( wb_info.temperature_detected - p_fsm->temperature_threshold[0] ) ) / ( p_fsm->temperature_threshold[1] - p_fsm->temperature_threshold[0] );
                    pr_debug("temp between %d and %d\n", p_fsm->temperature_threshold[0], p_fsm->temperature_threshold[1]);
            }
            // if current temp between 3100 and 3400 go to u30
            else if( ( wb_info.temperature_detected > p_fsm->temperature_threshold[1] ) && ( wb_info.temperature_detected < p_fsm->temperature_threshold[2] ) )
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_U30_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_U30_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_U30_BANK );
                p_fsm->shading_alpha = 0;
                pr_debug("temp between %d and %d\n", p_fsm->temperature_threshold[1], p_fsm->temperature_threshold[2]);
            }
            // if current temp between 3400 and 3600 use blending
            else if ( ( wb_info.temperature_detected > p_fsm->temperature_threshold[2] ) && ( wb_info.temperature_detected < p_fsm->temperature_threshold[3] ) )
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_U30_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_U30_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_U30_BANK );
                if ( p_fsm->temperature_threshold[2] != p_fsm->temperature_threshold[3] )
                    p_fsm->shading_alpha = ( 255 * ( wb_info.temperature_detected - p_fsm->temperature_threshold[2] ) ) / ( p_fsm->temperature_threshold[3] - p_fsm->temperature_threshold[2] );
                    pr_debug("temp between %d and %d\n", p_fsm->temperature_threshold[2], p_fsm->temperature_threshold[3]);
            }
            // if current temp between 3600 and 4100 go to tl84
            else if ( ( wb_info.temperature_detected > p_fsm->temperature_threshold[3] ) && ( wb_info.temperature_detected < p_fsm->temperature_threshold[4] ) )
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
                p_fsm->shading_alpha = 0;
                pr_debug("temp between %d and %d\n", p_fsm->temperature_threshold[3], p_fsm->temperature_threshold[4]);
            }
            // if current temp between 4100 and 4500 use blending
            else if ( ( wb_info.temperature_detected > p_fsm->temperature_threshold[4] ) && ( wb_info.temperature_detected < p_fsm->temperature_threshold[5] ) )
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_TL84_BANK );
                if ( p_fsm->temperature_threshold[4] != p_fsm->temperature_threshold[5] )
                    p_fsm->shading_alpha = ( 255 * ( wb_info.temperature_detected - p_fsm->temperature_threshold[4] ) ) / ( p_fsm->temperature_threshold[5] - p_fsm->temperature_threshold[4] );
                    pr_debug("temp between %d and %d\n", p_fsm->temperature_threshold[4], p_fsm->temperature_threshold[5]);
            }
            // if current temp > 4500 go to d50
            else if ( ( wb_info.temperature_detected > p_fsm->temperature_threshold[5] ) )
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_D50_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_D50_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, MESH_SHADING_LS_D50_BANK );
                p_fsm->shading_alpha = 0;
                pr_debug("temp > %d\n", p_fsm->temperature_threshold[5]);
            }
        } else {
            pr_debug("use default shading update logic\n");
            //uint32_t fixed_table = 0;
            // if temp less < 3250 go to A
            // LSC is completely based on alpha blending and the AWB color temp.
            if ( ( wb_info.temperature_detected < p_fsm->temperature_threshold[0] ) ) //0->1
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_A_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_A_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_A_BANK );
                p_fsm->shading_alpha = 0;
                p_fsm->shading_source_previous = AWB_LIGHT_SOURCE_A;

            }
            // if  current temp  between 4100 and 3900 use D40
            else if ( ( wb_info.temperature_detected > p_fsm->temperature_threshold[1] ) && ( wb_info.temperature_detected < p_fsm->temperature_threshold[2] ) ) //1->2
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_D40_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_D40_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_D40_BANK );
                p_fsm->shading_direction = 1; //0 ->inc  1->dec
                p_fsm->shading_alpha = 0;
                p_fsm->shading_source_previous = AWB_LIGHT_SOURCE_D40;

            }
            // if  current temp > 4900 go to d65
            else if ( ( wb_info.temperature_detected > p_fsm->temperature_threshold[3] ) ) //2->1
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_D50_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_D50_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_D50_BANK );
                p_fsm->shading_direction = 0; //0 ->inc  1->dec
                p_fsm->shading_alpha = 0;
                p_fsm->shading_source_previous = AWB_LIGHT_SOURCE_D50;

            }

            // if prev if d50 and current temp < 3700 go to d40
            else if ( ( wb_info.temperature_detected > p_fsm->temperature_threshold[4] ) && ( wb_info.temperature_detected < p_fsm->temperature_threshold[5] ) ) //2->0
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_A_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_A_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_A_BANK );

                if ( p_fsm->temperature_threshold[5] != p_fsm->temperature_threshold[4] )
                    p_fsm->shading_alpha = ( 255 * ( wb_info.temperature_detected - p_fsm->temperature_threshold[4] ) ) / ( p_fsm->temperature_threshold[5] - p_fsm->temperature_threshold[4] ); // division by zero is checked
                                                                                                                                                                                                //p_fsm->shading_source_previous = AWB_LIGHT_SOURCE_D40;

            }
            // if  current temp > 4750 go to d65
            else if ( ( wb_info.temperature_detected > p_fsm->temperature_threshold[6] ) && ( wb_info.temperature_detected < p_fsm->temperature_threshold[7] ) ) //2->1
            {
                acamera_isp_mesh_shading_mesh_alpha_bank_r_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_D40_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_g_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_D40_BANK );
                acamera_isp_mesh_shading_mesh_alpha_bank_b_write( p_fsm->cmn.isp_base, OV_08835_MESH_SHADING_LS_D40_BANK );

                if ( p_fsm->temperature_threshold[7] != p_fsm->temperature_threshold[6] )
                    p_fsm->shading_alpha = ( 255 * ( wb_info.temperature_detected - p_fsm->temperature_threshold[6] ) ) / ( p_fsm->temperature_threshold[7] - p_fsm->temperature_threshold[6] ); // division by zero is checked
                                                                                                                                                                                                //p_fsm->shading_source_previous = AWB_LIGHT_SOURCE_D50;
            }
        }

        acamera_isp_mesh_shading_mesh_alpha_r_write( p_fsm->cmn.isp_base, p_fsm->shading_alpha );
        acamera_isp_mesh_shading_mesh_alpha_g_write( p_fsm->cmn.isp_base, p_fsm->shading_alpha );
        acamera_isp_mesh_shading_mesh_alpha_b_write( p_fsm->cmn.isp_base, p_fsm->shading_alpha );
    }

    //this will put the ccm into the right format used in purple fringe and write to registers
    //Write CCM before it is modified by saturation modulation
    write_CCM_to_purple_fringe( p_fsm );

    color_matrix_recalculate( p_fsm );
    color_matrix_write( p_fsm );
    mesh_shading_modulate_strength( p_fsm );

    // update 3d_lut
    {
	fsm_param_awb_info_t wb_info;
	acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_AWB_INFO, NULL, 0, &wb_info, sizeof( wb_info ) );
	uint32_t i = 0;
	const uint32_t *p_lut3d_mem_a = _GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_LUT3D_MEM_A );
        uint32_t lut3d_mem_len_a = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_LUT3D_MEM_A );
	const uint32_t *p_lut3d_mem_d40 = _GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_LUT3D_MEM_D40 );
        uint32_t lut3d_mem_len_d40 = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_LUT3D_MEM_D40 );
	const uint32_t *p_lut3d_mem_d50 = _GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_LUT3D_MEM_D50 );
        uint32_t lut3d_mem_len_d50 = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_LUT3D_MEM_D50 );
	if (p_lut3d_mem_a && p_lut3d_mem_d40 &&  p_lut3d_mem_d50) {
		if (wb_info.temperature_detected < AWB_DLS_LIGHT_SOURCE_A_D40_BORDER) {
			for ( i = 0; i < lut3d_mem_len_a; i++ ) {
				acamera_lut3d_mem_array_data_write(p_fsm->cmn.isp_base, i, p_lut3d_mem_a[i]);
			}
		} else if (wb_info.temperature_detected > AWB_DLS_LIGHT_SOURCE_D40_D50_BORDER) {
			for ( i = 0; i < lut3d_mem_len_d40; i++ ) {
				acamera_lut3d_mem_array_data_write(p_fsm->cmn.isp_base, i, p_lut3d_mem_d40[i]);
			}
		} else {
			for ( i = 0; i < lut3d_mem_len_d50; i++ ) {
				acamera_lut3d_mem_array_data_write(p_fsm->cmn.isp_base, i, p_lut3d_mem_d50[i]);
			}
		}
	}
    }

    if ( p_fsm->light_source_change_frames_left > 0 ) {
        p_fsm->light_source_change_frames_left--;
    }
}

/*
 * func : used for ccm interpolation
 * input : ccm_a + ccm_b + temper + threshold
 *
 */
void color_matrix_interpolation_ccm(int16_t *pa, int16_t *pb, int16_t *pout, uint32_t temper, uint32_t threshold_a, uint32_t threshold_b)
{
       uint32_t i = 0;
       int32_t a_ccm[9] = {0};
       int32_t b_ccm[9] = {0};

        if (temper <= threshold_a) {
               for (i = 0; i < 9; i++) {
                       pout[i] = pa[i];
               }
        } else if (temper >= threshold_b) {
               for (i = 0; i < 9; i++) {
                       pout[i] = pb[i];
               }
        } else {
               for (i = 0; i < 9; i++) {
                       a_ccm[i] = pa[i];
                       b_ccm[i] = pb[i];
                       if (b_ccm[i] >= a_ccm[i]) {
                               pout[i] = (int16_t)(a_ccm[i] + ((((b_ccm[i] - a_ccm[i])*(temper - threshold_a))/(threshold_b - threshold_a))));
                       } else {
                               pout[i] = (int16_t)(a_ccm[i] - ((((a_ccm[i] - b_ccm[i])*(temper - threshold_a))/(threshold_b - threshold_a))));
                       }
               }
	}
}

void color_matrix_change_CCMs( color_matrix_fsm_t *p_fsm )
{
    uint8_t i;
    uint16_t *p_mtrx;
    uint32_t ccm_blend_flag = 0;
    //    For CCM switching

    p_mtrx = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_A_CCM );
    color_matrix_setup( &p_fsm->color_matrix_A[0], p_mtrx[0], p_mtrx[1], p_mtrx[2], p_mtrx[3], p_mtrx[4], p_mtrx[5], p_mtrx[6], p_mtrx[7], p_mtrx[8] );
    if (_GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_U30_CCM ) > 8) {
	p_mtrx = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_U30_CCM );
	color_matrix_setup( &p_fsm->color_matrix_U30[0], p_mtrx[0], p_mtrx[1], p_mtrx[2], p_mtrx[3], p_mtrx[4], p_mtrx[5], p_mtrx[6], p_mtrx[7], p_mtrx[8] );
    }
    p_mtrx = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_D40_CCM );
    color_matrix_setup( &p_fsm->color_matrix_D40[0], p_mtrx[0], p_mtrx[1], p_mtrx[2], p_mtrx[3], p_mtrx[4], p_mtrx[5], p_mtrx[6], p_mtrx[7], p_mtrx[8] );
    p_mtrx = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_D50_CCM );
    color_matrix_setup( &p_fsm->color_matrix_D50[0], p_mtrx[0], p_mtrx[1], p_mtrx[2], p_mtrx[3], p_mtrx[4], p_mtrx[5], p_mtrx[6], p_mtrx[7], p_mtrx[8] );
    color_matrix_setup( p_fsm->color_matrix_one, 256, 0, 0, 0, 256, 0, 0, 0, 256 );

    /* update ccm threshold when CCM_TEMPER_THRESHOLD is right */
	uint32_t ccm_threshold_len = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_CCM_TEMPER_THRESHOLD );
	const uint32_t *p_ccm_threshold = _GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_CCM_TEMPER_THRESHOLD );
	if (ccm_threshold_len > 7) {
		ccm_blend_flag = p_ccm_threshold[6];
	}

	if ((ccm_threshold_len >= 6) && (ccm_blend_flag)) {
		if ((p_ccm_threshold[0] >= p_ccm_threshold[1]) ||
			(p_ccm_threshold[1] >= p_ccm_threshold[2]) ||
			(p_ccm_threshold[2] >= p_ccm_threshold[3]) ||
			(p_ccm_threshold[3] >= p_ccm_threshold[4]) ||
			(p_ccm_threshold[4] >= p_ccm_threshold[5])) {
			// null
		} else {
			p_fsm->ccm_threshold[0] = p_ccm_threshold[0];
			p_fsm->ccm_threshold[1] = p_ccm_threshold[1];
			p_fsm->ccm_threshold[2] = p_ccm_threshold[2];
			p_fsm->ccm_threshold[3] = p_ccm_threshold[3];
			p_fsm->ccm_threshold[4] = p_ccm_threshold[4];
			p_fsm->ccm_threshold[5] = p_ccm_threshold[5];
		}
	}

    /* ccm interpolation when a/u30/d40/d50 is valid */
    /*
     * blend rules
     * ------th0------th1------th2------th3------th4------th5------
     *  a     |  a+u30 |  u30   | u30+d40|   d40  | d40+d50|  d50
     *
     */
	if (ccm_blend_flag) {
		fsm_param_awb_info_t wb_info;
		acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_AWB_INFO, NULL, 0, &wb_info, sizeof( wb_info ) );
		if (_GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_U30_CCM ) > 8) {
			if (wb_info.temperature_detected <= p_fsm->ccm_threshold[2]) {
				color_matrix_interpolation_ccm(&p_fsm->color_matrix_A[0], &p_fsm->color_matrix_U30[0], &p_fsm->color_matrix_CCM[0],
					wb_info.temperature_detected, p_fsm->ccm_threshold[0], p_fsm->ccm_threshold[1]);
			} else if ((wb_info.temperature_detected > p_fsm->ccm_threshold[2]) && (wb_info.temperature_detected <= p_fsm->ccm_threshold[4])) {
				color_matrix_interpolation_ccm(&p_fsm->color_matrix_U30[0], &p_fsm->color_matrix_D40[0], &p_fsm->color_matrix_CCM[0],
					wb_info.temperature_detected, p_fsm->ccm_threshold[2], p_fsm->ccm_threshold[3]);
			} else {
				color_matrix_interpolation_ccm(&p_fsm->color_matrix_D40[0], &p_fsm->color_matrix_D50[0], &p_fsm->color_matrix_CCM[0],
					wb_info.temperature_detected, p_fsm->ccm_threshold[4], p_fsm->ccm_threshold[5]);
			}
		}
	}

    if ((p_fsm->light_source_change_frames_left != 0) && (!ccm_blend_flag)) {
        // In CCM switching
        // call CCM switcher to update CCM
        // (note: this does mean CCM switching takes one frame less than normal)
        color_matrix_update( p_fsm );
    } else {
        // Not moving, update current CCMs

        int16_t *p_ccm_next;

        switch ( p_fsm->light_source_ccm ) {
        case AWB_LIGHT_SOURCE_A:
	    if (ccm_blend_flag) {
		    p_ccm_next = p_fsm->color_matrix_CCM;
	    } else {
		p_ccm_next = p_fsm->color_matrix_A;
	    }
            break;
        case AWB_LIGHT_SOURCE_D40:
	    if (ccm_blend_flag) {
		    p_ccm_next = p_fsm->color_matrix_CCM;
	    } else {
		p_ccm_next = p_fsm->color_matrix_D40;
	    }
            break;
        case AWB_LIGHT_SOURCE_D50:
	    if (ccm_blend_flag) {
		    p_ccm_next = p_fsm->color_matrix_CCM;
	    } else {
		p_ccm_next = p_fsm->color_matrix_D50;
	    }
            break;
        default:
            p_ccm_next = p_fsm->color_matrix_one;
            break;
        }

        for ( i = 0; i < 9; i++ ) {
            p_fsm->color_correction_matrix[i] = p_ccm_next[i];
        }
    }

}

/*
 * CALIBRATION_CCM_TEMPER_THRESHOLD [7]:
 * [0-5] used for ccm threshold
 * [6] used for ccm blend enable/disbale 0:disable blend 1:enable blend
 *
 */
void color_matrix_update_CCMs( color_matrix_fsm_t *p_fsm )
{
	uint8_t i = 0;
	uint16_t *p_mtrx = NULL;

	/* update ccm threshold when CCM_TEMPER_THRESHOLD is right */
	uint32_t ccm_threshold_len = _GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_CCM_TEMPER_THRESHOLD );
	const uint32_t *p_ccm_threshold = _GET_UINT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_CCM_TEMPER_THRESHOLD );

	if (ccm_threshold_len >= 6) {
		if ((p_ccm_threshold[0] >= p_ccm_threshold[1]) ||
			(p_ccm_threshold[1] >= p_ccm_threshold[2]) ||
			(p_ccm_threshold[2] >= p_ccm_threshold[3]) ||
			(p_ccm_threshold[3] >= p_ccm_threshold[4]) ||
			(p_ccm_threshold[4] >= p_ccm_threshold[5])) {
			// null
		} else {
			p_fsm->ccm_threshold[0] = p_ccm_threshold[0];
			p_fsm->ccm_threshold[1] = p_ccm_threshold[1];
			p_fsm->ccm_threshold[2] = p_ccm_threshold[2];
			p_fsm->ccm_threshold[3] = p_ccm_threshold[3];
			p_fsm->ccm_threshold[4] = p_ccm_threshold[4];
			p_fsm->ccm_threshold[5] = p_ccm_threshold[5];
		}
	}

	p_mtrx = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_A_CCM );
	color_matrix_setup( &p_fsm->color_matrix_A[0], p_mtrx[0], p_mtrx[1], p_mtrx[2], p_mtrx[3], p_mtrx[4], p_mtrx[5], p_mtrx[6], p_mtrx[7], p_mtrx[8] );
	if (_GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_U30_CCM ) > 8) {
	    p_mtrx = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_U30_CCM );
	    color_matrix_setup( &p_fsm->color_matrix_U30[0], p_mtrx[0], p_mtrx[1], p_mtrx[2], p_mtrx[3], p_mtrx[4], p_mtrx[5], p_mtrx[6], p_mtrx[7], p_mtrx[8] );
	}
	p_mtrx = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_D40_CCM );
	color_matrix_setup( &p_fsm->color_matrix_D40[0], p_mtrx[0], p_mtrx[1], p_mtrx[2], p_mtrx[3], p_mtrx[4], p_mtrx[5], p_mtrx[6], p_mtrx[7], p_mtrx[8] );
	p_mtrx = _GET_USHORT_PTR( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_D50_CCM );
	color_matrix_setup( &p_fsm->color_matrix_D50[0], p_mtrx[0], p_mtrx[1], p_mtrx[2], p_mtrx[3], p_mtrx[4], p_mtrx[5], p_mtrx[6], p_mtrx[7], p_mtrx[8] );
	color_matrix_setup( p_fsm->color_matrix_one, 256, 0, 0, 0, 256, 0, 0, 0, 256 );

	/* ccm interpolation when a/u30/d40/d50 is valid */
	/*
	 * blend rules
	 * ------th0------th1------th2------th3------th4------th5------
	 *  a     |  a+u30 |  u30   | u30+d40|   d40  | d40+d50|  d50
	 *
	 */
	{
	    fsm_param_awb_info_t wb_info;
	    acamera_fsm_mgr_get_param( p_fsm->cmn.p_fsm_mgr, FSM_PARAM_GET_AWB_INFO, NULL, 0, &wb_info, sizeof( wb_info ) );
	    if (_GET_LEN( ACAMERA_FSM2CTX_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_U30_CCM ) > 8) {
	    	if (wb_info.temperature_detected <= p_fsm->ccm_threshold[2]) {
	    		color_matrix_interpolation_ccm(&p_fsm->color_matrix_A[0], &p_fsm->color_matrix_U30[0], &p_fsm->color_matrix_CCM[0],
	    			wb_info.temperature_detected, p_fsm->ccm_threshold[0], p_fsm->ccm_threshold[1]);
	    	} else if ((wb_info.temperature_detected > p_fsm->ccm_threshold[2]) && (wb_info.temperature_detected <= p_fsm->ccm_threshold[4])) {
	    		color_matrix_interpolation_ccm(&p_fsm->color_matrix_U30[0], &p_fsm->color_matrix_D40[0], &p_fsm->color_matrix_CCM[0],
	    			wb_info.temperature_detected, p_fsm->ccm_threshold[2], p_fsm->ccm_threshold[3]);
	    	} else {
	    		color_matrix_interpolation_ccm(&p_fsm->color_matrix_D40[0], &p_fsm->color_matrix_D50[0], &p_fsm->color_matrix_CCM[0],
	    			wb_info.temperature_detected, p_fsm->ccm_threshold[4], p_fsm->ccm_threshold[5]);
	    	}
	    }
	}

        // Not moving, update current CCMs

        int16_t *p_ccm_next;

        switch ( p_fsm->light_source_ccm ) {
        case AWB_LIGHT_SOURCE_A:
        case AWB_LIGHT_SOURCE_D40:
        case AWB_LIGHT_SOURCE_D50:
            p_ccm_next = p_fsm->color_matrix_CCM;
            break;
        default:
            p_ccm_next = p_fsm->color_matrix_one;
            break;
        }

        for ( i = 0; i < 9; i++ ) {
            p_fsm->color_correction_matrix[i] = p_ccm_next[i];
        }
}
