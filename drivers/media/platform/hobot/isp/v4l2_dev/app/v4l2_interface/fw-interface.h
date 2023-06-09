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

#ifndef _FW_INTERFACE_H_
#define _FW_INTERFACE_H_

#include <linux/videodev2.h>

/* fw-interface isp control interface */
int fw_intf_isp_init( uint32_t hw_isp_addr );
int fw_intf_is_isp_started( void );
void fw_intf_isp_deinit( void );
int fw_intf_isp_start( void );
void fw_intf_isp_stop( void );
int fw_intf_isp_get_current_ctx_id( uint32_t ctx_id );
int fw_intf_isp_get_sensor_info( uint32_t ctx_id, isp_v4l2_sensor_info *sensor_info );
int fw_intf_isp_get_sensor_preset( uint32_t ctx_id );
int fw_intf_isp_set_sensor_preset( uint32_t ctx_id, uint32_t preset );

//calibration
uint32_t fw_calibration_update( uint32_t ctx_id );

/* fw-interface per-stream control interface */
int fw_intf_stream_start( uint32_t ctx_id, isp_v4l2_stream_type_t streamType );
void fw_intf_stream_stop( uint32_t ctx_id, isp_v4l2_stream_type_t streamType );

/* fw-interface sensor hw stream control interface */
int fw_intf_sensor_pause( uint32_t ctx_id );
int fw_intf_sensor_resume( uint32_t ctx_id );

/* fw-interface per-stream config interface */
int fw_intf_stream_set_resolution( uint32_t ctx_id, const isp_v4l2_sensor_info *sensor_info,
                                   isp_v4l2_stream_type_t streamType, struct v4l2_format *f );
int fw_intf_stream_set_output_format( uint32_t ctx_id, isp_v4l2_stream_type_t streamType, uint32_t format );

/* fw-interface isp config interface */
bool fw_intf_validate_control( uint32_t id );
int fw_intf_set_test_pattern( uint32_t ctx_id, int val );
int fw_intf_set_test_pattern_type( uint32_t ctx_id, int val );
int fw_intf_set_af_refocus( uint32_t ctx_id, int val );
int fw_intf_set_af_roi( uint32_t ctx_id, int val );
int fw_intf_set_brightness( uint32_t ctx_id, int val );
int fw_intf_set_contrast( uint32_t ctx_id, int val );
int fw_intf_set_saturation( uint32_t ctx_id, int val );
int fw_intf_set_hue( uint32_t ctx_id, int val );
int fw_intf_set_sharpness( uint32_t ctx_id, int val );
int fw_intf_set_color_fx( uint32_t ctx_id, int val );
int fw_intf_set_hflip( uint32_t ctx_id, int val );
int fw_intf_set_vflip( uint32_t ctx_id, int val );
int fw_intf_set_autogain( uint32_t ctx_id, int val );
int fw_intf_set_gain( uint32_t ctx_id, int val );
int fw_intf_set_exposure_auto( uint32_t ctx_id, int val );
int fw_intf_set_exposure( uint32_t ctx_id, int val );
int fw_intf_set_variable_frame_rate( uint32_t ctx_id, int val );
int fw_intf_set_white_balance_auto( uint32_t ctx_id, int val );
int fw_intf_set_white_balance( uint32_t ctx_id, int val );
int fw_intf_set_focus_auto( uint32_t ctx_id, int val );
int fw_intf_set_focus( uint32_t ctx_id, int val );
int fw_intf_set_output_fr_on_off( uint32_t ctx_id, uint32_t ctrl_val );
int fw_intf_set_output_ds1_on_off( uint32_t ctx_id, uint32_t ctrl_val );
int fw_intf_set_raw_bypass_on_off(uint32_t ctx_id, uint32_t ctrl_val);
int fw_intf_temper_buf_ctrl(uint32_t ctx_id, uint32_t ctrl_val);
int fw_intf_iridix_ctrl(uint32_t ctx_id, uint32_t ctrl_val);
/*******new add interface********************************************/
int fw_intf_set_white_balance_red(uint32_t ctx_id, uint32_t ctrl_val);
int fw_intf_set_white_balance_blue(uint32_t ctx_id, uint32_t ctrl_val);
int fw_intf_set_gamma(uint32_t ctx_id, uint32_t ctrl_val);
int fw_intf_set_hue_auto(uint32_t ctx_id, uint32_t ctrl_val);
int fw_intf_set_white_balance_tmperature(uint32_t ctx_id, uint32_t ctrl_val);
int fw_intf_set_backlight(uint32_t ctx_id, int val);
int fw_intf_set_pan(uint32_t ctx_id, int val);
int fw_intf_set_pan_speed(uint32_t ctx_id, int val);
int fw_intf_set_tilt(uint32_t ctx_id, int val);
int fw_intf_set_tilt_speed(uint32_t ctx_id, int val);
int fw_intf_set_zoom(uint32_t ctx_id, int val);
int fw_intf_set_zoom_continuous(uint32_t ctx_id, int val);
int fw_intf_set_iris(uint32_t ctx_id, int val);
int fw_intf_set_power_line_freq(uint32_t ctx_id, int val);
#endif
