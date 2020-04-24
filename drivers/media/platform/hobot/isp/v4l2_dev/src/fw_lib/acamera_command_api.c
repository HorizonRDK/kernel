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
#include "acamera_command_api.h"


#include "acamera_command_api_impl.h"
#include "acamera.h"
#include "acamera_firmware_api.h"
#include "acamera_fw.h"

#if FW_HAS_CONTROL_CHANNEL
#include "acamera_ctrl_channel.h"
#endif


extern void * acamera_get_ctx_ptr( uint32_t ctx_id );
extern int isp_open_check(void);

uint8_t acamera_command( uint32_t ctx_id, uint8_t command_type, uint8_t command, uint32_t value, uint8_t direction, uint32_t *ret_value){
acamera_context_ptr_t p_ctx = acamera_get_ctx_ptr(ctx_id);
if (p_ctx->initialized == 0) {
	pr_err("context %d is not initialized.\n", ctx_id);
	return -1;
}
if (!isp_open_check())
	return -1;

acamera_fsm_mgr_t *instance = &p_ctx->fsm_mgr;
uint8_t ret = NOT_EXISTS;
switch (command_type){
case  TGENERAL:
	switch (command){
		case  CONTEXT_NUMBER:
			ret = general_context_number(instance, value, direction, ret_value);
			break;
		case  ACTIVE_CONTEXT:
			ret = general_active_context(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  TSELFTEST:
	switch (command){
		case  FW_REVISION:
			ret = selftest_fw_revision(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  TSENSOR:
	switch (command){
		case  SENSOR_STREAMING:
			ret = sensor_streaming(instance, value, direction, ret_value);
			break;
		case  SENSOR_SUPPORTED_PRESETS:
			ret = sensor_supported_presets(instance, value, direction, ret_value);
			break;
		case  SENSOR_PRESET:
			ret = sensor_preset(instance, value, direction, ret_value);
			break;
		case  SENSOR_WDR_MODE:
			ret = sensor_wdr_mode(instance, value, direction, ret_value);
			break;
		case  SENSOR_FPS:
			ret = sensor_fps(instance, value, direction, ret_value);
			break;
		case  SENSOR_WIDTH:
			ret = sensor_width(instance, value, direction, ret_value);
			break;
		case  SENSOR_HEIGHT:
			ret = sensor_height(instance, value, direction, ret_value);
			break;
		case  SENSOR_EXPOSURES:
			ret = sensor_exposures(instance, value, direction, ret_value);
			break;
		case  SENSOR_INFO_PRESET:
			ret = sensor_info_preset(instance, value, direction, ret_value);
			break;
		case  SENSOR_INFO_WDR_MODE:
			ret = sensor_info_wdr_mode(instance, value, direction, ret_value);
			break;
		case  SENSOR_INFO_FPS:
			ret = sensor_info_fps(instance, value, direction, ret_value);
			break;
		case  SENSOR_INFO_WIDTH:
			ret = sensor_info_width(instance, value, direction, ret_value);
			break;
		case  SENSOR_INFO_HEIGHT:
			ret = sensor_info_height(instance, value, direction, ret_value);
			break;
		case  SENSOR_INFO_EXPOSURES:
			ret = sensor_info_exposures(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  TSYSTEM:
	switch (command){
		case  SYSTEM_LOGGER_LEVEL:
			ret = system_logger_level(instance, value, direction, ret_value);
			break;
		case  SYSTEM_LOGGER_MASK:
			ret = system_logger_mask(instance, value, direction, ret_value);
			break;
		case  BUFFER_DATA_TYPE:
			ret = buffer_data_type(instance, value, direction, ret_value);
			break;
		case  TEST_PATTERN_ENABLE_ID:
			ret = test_pattern_enable(instance, value, direction, ret_value);
			break;
		case  TEST_PATTERN_MODE_ID:
			ret = test_pattern(instance, value, direction, ret_value);
			break;
		case  TEMPER_MODE_ID:
			ret = temper_mode(instance, value, direction, ret_value);
			break;
		case  SYSTEM_FREEZE_FIRMWARE:
			ret = system_freeze_firmware(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MANUAL_EXPOSURE:
			ret = system_manual_exposure(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MANUAL_EXPOSURE_RATIO:
			ret = system_manual_exposure_ratio(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MANUAL_INTEGRATION_TIME:
			ret = system_manual_integration_time(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MANUAL_MAX_INTEGRATION_TIME:
			ret = system_manual_max_integration_time(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MANUAL_SENSOR_ANALOG_GAIN:
			ret = system_manual_sensor_analog_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN:
			ret = system_manual_sensor_digital_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MANUAL_ISP_DIGITAL_GAIN:
			ret = system_manual_isp_digital_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MANUAL_AWB:
			ret = system_manual_awb(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MANUAL_SATURATION:
			ret = system_manual_saturation(instance, value, direction, ret_value);
			break;
		case  SYSTEM_EXPOSURE:
			ret = system_exposure(instance, value, direction, ret_value);
			break;
		case  SYSTEM_EXPOSURE_RATIO:
			ret = system_exposure_ratio(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MAX_EXPOSURE_RATIO:
			ret = system_max_exposure_ratio(instance, value, direction, ret_value);
			break;
		case  SYSTEM_INTEGRATION_TIME:
			ret = system_integration_time(instance, value, direction, ret_value);
			break;
		case  SYSTEM_LONG_INTEGRATION_TIME:
			ret = system_long_integration_time(instance, value, direction, ret_value);
			break;
		case  SYSTEM_SHORT_INTEGRATION_TIME:
			ret = system_short_integration_time(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MAX_INTEGRATION_TIME:
			ret = system_max_integration_time(instance, value, direction, ret_value);
			break;
		case  SYSTEM_SENSOR_ANALOG_GAIN:
			ret = system_sensor_analog_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MAX_SENSOR_ANALOG_GAIN:
			ret = system_max_sensor_analog_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_SENSOR_DIGITAL_GAIN:
			ret = system_sensor_digital_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MAX_SENSOR_DIGITAL_GAIN:
			ret = system_max_sensor_digital_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_ISP_DIGITAL_GAIN:
			ret = system_isp_digital_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_MAX_ISP_DIGITAL_GAIN:
			ret = system_max_isp_digital_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_AWB_RED_GAIN:
			ret = system_awb_red_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_AWB_BLUE_GAIN:
			ret = system_awb_blue_gain(instance, value, direction, ret_value);
			break;
		case  SYSTEM_SATURATION_TARGET:
			ret = system_saturation_target(instance, value, direction, ret_value);
			break;
		case  SYSTEM_ANTIFLICKER_ENABLE:
			ret = system_antiflicker_enable(instance, value, direction, ret_value);
			break;
		case  SYSTEM_ANTI_FLICKER_FREQUENCY:
			ret = system_anti_flicker_frequency(instance, value, direction, ret_value);
			break;
		case  CALIBRATION_UPDATE:
			ret = calibration_update(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  TISP_MODULES:
	switch (command){
		case  ISP_MODULES_MANUAL_IRIDIX:
			ret = isp_modules_manual_iridix(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_SINTER:
			ret = isp_modules_manual_sinter(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_TEMPER:
			ret = isp_modules_manual_temper(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_AUTO_LEVEL:
			ret = isp_modules_manual_auto_level(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_FRAME_STITCH:
			ret = isp_modules_manual_frame_stitch(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_RAW_FRONTEND:
			ret = isp_modules_manual_raw_frontend(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_BLACK_LEVEL:
			ret = isp_modules_manual_black_level(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_SHADING:
			ret = isp_modules_manual_shading(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_DEMOSAIC:
			ret = isp_modules_manual_demosaic(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_CNR:
			ret = isp_modules_manual_cnr(instance, value, direction, ret_value);
			break;
		case  ISP_MODULES_MANUAL_SHARPEN:
			ret = isp_modules_manual_sharpen(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  TSTATUS:
	switch (command){
		case  STATUS_INFO_EXPOSURE_LOG2_ID:
			ret = status_info_exposure_log2(instance, value, direction, ret_value);
			break;
		case  STATUS_INFO_GAIN_ONES_ID:
			ret = status_info_gain_ones(instance, value, direction, ret_value);
			break;
		case  STATUS_INFO_GAIN_LOG2_ID:
			ret = status_info_gain_log2(instance, value, direction, ret_value);
			break;
		case  STATUS_INFO_AWB_MIX_LIGHT_CONTRAST:
			ret = status_info_awb_mix_light_contrast(instance, value, direction, ret_value);
			break;
		case  STATUS_INFO_AF_LENS_POS:
			ret = status_info_af_lens_pos(instance, value, direction, ret_value);
			break;
		case  STATUS_INFO_AF_FOCUS_VALUE:
			ret = status_info_af_focus_value(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  TIMAGE:
	switch (command){
		case  DMA_READER_OUTPUT_ID:
			ret = dma_reader_output(instance, value, direction, ret_value);
			break;
		case  FR_FORMAT_BASE_PLANE_ID:
			ret = fr_format_base_plane(instance, value, direction, ret_value);
			break;
		case  DS1_FORMAT_BASE_PLANE_ID:
			ret = ds1_format_base_plane(instance, value, direction, ret_value);
			break;
		case  ORIENTATION_VFLIP_ID:
			ret = orientation_vflip(instance, value, direction, ret_value);
			break;
		case  ORIENTATION_HFLIP_ID:
			ret = orientation_hflip(instance, value, direction, ret_value);
			break;
		case  IMAGE_RESIZE_TYPE_ID:
			ret = image_resize_type(instance, value, direction, ret_value);
			break;
		case  IMAGE_RESIZE_ENABLE_ID:
			ret = image_resize_enable(instance, value, direction, ret_value);
			break;
		case  IMAGE_RESIZE_WIDTH_ID:
			ret = image_resize_width(instance, value, direction, ret_value);
			break;
		case  IMAGE_RESIZE_HEIGHT_ID:
			ret = image_resize_height(instance, value, direction, ret_value);
			break;
		case  IMAGE_CROP_XOFFSET_ID:
			ret = image_crop_xoffset(instance, value, direction, ret_value);
			break;
		case  IMAGE_CROP_YOFFSET_ID:
			ret = image_crop_yoffset(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  TALGORITHMS:
	switch (command){
		case  AF_LENS_STATUS:
			ret = af_lens_status(instance, value, direction, ret_value);
			break;
		case  AF_MODE_ID:
			ret = af_mode(instance, value, direction, ret_value);
			break;
		case  AF_RANGE_LOW_ID:
			ret = af_range_low(instance, value, direction, ret_value);
			break;
		case  AF_RANGE_HIGH_ID:
			ret = af_range_high(instance, value, direction, ret_value);
			break;
		case  AF_ROI_ID:
			ret = af_roi(instance, value, direction, ret_value);
			break;
		case  AF_MANUAL_CONTROL_ID:
			ret = af_manual_control(instance, value, direction, ret_value);
			break;
		case  ZOOM_MANUAL_CONTROL_ID:
			ret = zoom_manual_control(instance, value, direction, ret_value);
			break;
		case  AE_MODE_ID:
			ret = ae_mode(instance, value, direction, ret_value);
			break;
		case  AE_SPLIT_PRESET_ID:
			ret = ae_split_preset(instance, value, direction, ret_value);
			break;
		case  AE_GAIN_ID:
			ret = ae_gain(instance, value, direction, ret_value);
			break;
		case  AE_EXPOSURE_ID:
			ret = ae_exposure(instance, value, direction, ret_value);
			break;
		case  AE_ROI_ID:
			ret = ae_roi(instance, value, direction, ret_value);
			break;
		case  AE_COMPENSATION_ID:
			ret = ae_compensation(instance, value, direction, ret_value);
			break;
		case  AWB_MODE_ID:
			ret = awb_mode(instance, value, direction, ret_value);
			break;
		case  AWB_TEMPERATURE_ID:
			ret = awb_temperature(instance, value, direction, ret_value);
			break;
		case  ANTIFLICKER_MODE_ID:
			ret = antiflicker_mode(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  TSCENE_MODES:
	switch (command){
		case  COLOR_MODE_ID:
			ret = color_mode(instance, value, direction, ret_value);
			break;
		case  BRIGHTNESS_STRENGTH_ID:
			ret = brightness_strength(instance, value, direction, ret_value);
			break;
		case  CONTRAST_STRENGTH_ID:
			ret = contrast_strength(instance, value, direction, ret_value);
			break;
		case  SATURATION_STRENGTH_ID:
			ret = saturation_strength(instance, value, direction, ret_value);
			break;
		case  SHARPENING_STRENGTH_ID:
			ret = sharpening_strength(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  TREGISTERS:
	switch (command){
		case  REGISTERS_ADDRESS_ID:
			ret = register_address(instance, value, direction, ret_value);
			break;
		case  REGISTERS_SIZE_ID:
			ret = register_size(instance, value, direction, ret_value);
			break;
		case  REGISTERS_SOURCE_ID:
			ret = register_source(instance, value, direction, ret_value);
			break;
		case  REGISTERS_VALUE_ID:
			ret = register_value(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  MSENSOR:
	switch (command){
		case  SENSOR_TYPE:
			ret = sensor_type(instance, value, direction, ret_value);
			break;
		case  SENSOR_I2C_CHANNEL:
			ret = sensor_i2c_chnnel(instance, value, direction, ret_value);
			break;
		case  SENSOR_MAX_AGAIN:
			ret = sensor_max_again(instance, value, direction, ret_value);
			break;
		case  SENSOR_MAX_DGAIN:
			ret = sensor_max_dgain(instance, value, direction, ret_value);
			break;
		case  SENSOR_MIN_INTERTIME:
			ret = sensor_min_intertime(instance, value, direction, ret_value);
			break;
		case  SENSOR_MAX_INTERTIME:
			ret = sensor_max_intertime(instance, value, direction, ret_value);
			break;
		case  SENSOR_MAX_LONGTIME:
			ret = sensor_max_longtime(instance, value, direction, ret_value);
			break;
		case  SENSOR_LIMIT_INTERTIME:
			ret = sensor_limit_intertime(instance, value, direction, ret_value);
			break;
		case  SENSOR_LINES_PER_SECOND:
			ret = sensor_lines_per_second(instance, value, direction, ret_value);
			break;
		case  SENSOR_DECOMP_BITS:
			ret = sensor_set_decomp_bits(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
case  LOG_LIST:
	switch (command){
		case  LOG_LIST_LEVEL:
			ret = system_logger_level(instance, value, direction, ret_value);
			break;
		case  LOG_LIST_MASK:
			ret = system_logger_mask(instance, value, direction, ret_value);
			break;
		case  LOGLIST_FW_GENERIC:
		case  LOGLIST_SENSOR:
		case  LOGLIST_CMOS:
		case  LOGLIST_CROP:
		case  LOGLIST_GENERAL:
		case  LOGLIST_AE_MANUAL:
		case  LOGLIST_AWB_MANUAL:
		case  LOGLIST_COLOR_MATRIX:
		case  LOGLIST_IRIDIX8_MANUAL:
		case  LOGLIST_NOISE_REDUCTION:
		case  LOGLIST_SHARPENING:
		case  LOGLIST_MATRIX_YUV:
		case  LOGLIST_GAMMA_MANUAL:
		case  LOGLIST_MONITOR:
		case  LOGLIST_SBUF:
		case  LOGLIST_DMA_WRITER:
		case  LOGLIST_METADATA:
		case  LOGLIST_AF_MANUAL:
		case  LOGLIST_FPGA_DMA_FE:
		case  LOGLIST_SOC_IQ:
		case  LOGLIST_SOC_SENSOR:
		case  LOGLIST_SOC_LENS:
			//ret = sensor_i2c_chnnel(instance, value, direction, ret_value);
			ret = system_loglist_mask(instance, (command - LOGLIST_FW_GENERIC), value, direction, ret_value );
			break;
	}//switch (command)
	break;
case  LDC_PARAM:
	switch (command){
		case  LINE_BUF_ID:
			ret = ldc_line_buf_control(instance, value, direction, ret_value);
			break;
		case  X_PARAMTER_A_ID:
			ret = ldc_x_param_a_control(instance, value, direction, ret_value);
			break;
		case  X_PARAMTER_B_ID:
			ret = ldc_x_param_b_control(instance, value, direction, ret_value);
			break;
		case  Y_PARAMTER_A_ID:
			ret = ldc_y_param_a_control(instance, value, direction, ret_value);
			break;
		case  Y_PARAMTER_B_ID:
			ret = ldc_y_param_b_control(instance, value, direction, ret_value);
			break;
		case  RADIUS_XOFFSET_ID:
			ret = ldc_radius_x_offset_control(instance, value, direction, ret_value);
			break;
		case  RADIUS_YOFFSET_ID:
			ret = ldc_radius_y_offset_control(instance, value, direction, ret_value);
			break;
		case  CENTER_XOFFSET_ID:
			ret = ldc_center_x_offset_control(instance, value, direction, ret_value);
			break;
		case  CENTER_YOFFSET_ID:
			ret = ldc_center_y_offset_control(instance, value, direction, ret_value);
			break;
		case  WOI_YLENGTH_ID:
			ret = ldc_woi_y_length_control(instance, value, direction, ret_value);
			break;
		case  WOI_YSTART_ID:
			ret = ldc_woi_y_stsrt_control(instance, value, direction, ret_value);
			break;
		case  WOI_XLENGTH_ID:
			ret = ldc_woi_x_length_control(instance, value, direction, ret_value);
			break;
		case  WOI_XSTART_ID:
			ret = ldc_woi_x_start_control(instance, value, direction, ret_value);
			break;
		case  LDC_BYPASS_ID:
			ret = ldc_ldc_bypass_control(instance, value, direction, ret_value);
			break;
		case  LDC_PARAM_UPDATE_ID:
			ret = ldc_param_update_control(instance, value, direction, ret_value);
			break;
	}//switch (command)
	break;
}//switch (command_type)

#if FW_HAS_CONTROL_CHANNEL
	ctrl_channel_handle_command( ctx_id, command_type, command, value, direction );
#endif

if(ret!=SUCCESS)
{
	LOG( LOG_WARNING, "API COMMAND FAILED: type %d, cmd %d, value %lu, direction %d, ret_value %lu, result %d",command_type, command, (unsigned long)value, direction, (unsigned long)*ret_value, ret);
}
else
{
	LOG( LOG_INFO, "API type %d, cmd %d, value %lu, direction %d, ret_value %lu, result %d",command_type, command, (unsigned long)value, direction, (unsigned long)*ret_value, ret);
}
	return ret;
}
