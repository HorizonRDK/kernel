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

#ifndef __ACAMERA_FIRMWARE_CONFIG_H__
#define __ACAMERA_FIRMWARE_CONFIG_H__


#define ACAMERA_EVENT_QUEUE_SIZE 256
#define ACAMERA_FW_SHP_DETECTED_MAXIMUM 1023
#define ACAMERA_IRQ_AE_STATS 3
#define ACAMERA_IRQ_AF2_STATS 1
#define ACAMERA_IRQ_ANTIFOG_HIST 14
#define ACAMERA_IRQ_AWB_STATS 4
#define ACAMERA_IRQ_DFE_FRAME_END 17
#define ACAMERA_IRQ_FPGA_FRAME_END 9
#define ACAMERA_IRQ_FRAME_END 0
#define ACAMERA_IRQ_FRAME_START 7
#define ACAMERA_IRQ_FRAME_WRITER_DS 6
#define ACAMERA_IRQ_FRAME_WRITER_FR 5
#define ACAMERA_ISP_PROFILING 0
#define ACAMERA_ISP_PROFILING_INIT 0
#define AE_CENTER_ZONES 65535
#define AF_SPOT_COUNT_X 5
#define AF_SPOT_COUNT_Y 5
#define ANALOG_GAIN_ACCURACY (1<<(LOG2_GAIN_SHIFT-2))
#define AWB_BG_MAX_GAIN 0
#define BLACK_LEVEL_SHIFT_DG 8
#define BLACK_LEVEL_SHIFT_WB 8
#define CALIBRATION_IRIDIX_AVG_COEF_INIT 0
#define CM_SATURATION_TARGET 128
#define CONNECTION_BUFFER_SIZE (26*1024)
#define CONNECTION_IN_THREAD 1
#define DDR_BASE_ADDRESS 0
#define EXPOSURE_DRIVES_LONG_INTEGRATION_TIME 0
#define FILTER_LONG_INT_TIME 0
#define FILTER_SHORT_INT_TIME 0
#define FIRMWARE_CONTEXT_NUMBER 6
#define FPGA_HAS_HDMI 0//1
#define FW_DO_INITIALIZATION 1
#define FW_DS1_OUTPUT_FORMAT_PIPE PIPE_OUT_RGB
#define FW_EVT_QUEUE_TIMEOUT_MS 100
#define FW_FR_OUTPUT_FORMAT_PIPE PIPE_OUT_RGB
#define FW_HAS_CONTROL_CHANNEL 1
#define FW_HAS_CUSTOM_SETTINGS 1
#define FW_INPUT_FORMAT DMA_FORMAT_RAW16
#define FW_OUTPUT_FORMAT DMA_FORMAT_A2R10G10B10
#define FW_OUTPUT_FORMAT_SECONDARY DMA_FORMAT_DISABLE
#define FW_USE_SYSTEM_DMA 0
#define FW_USE_HOBOT_DMA 0
#define HOBOT_DMA_SRAM_DEBUG_DRAM_MODE ((0x7FFFFFFF - HOBOT_DMA_SRAM_SIZE) & 0xfffff000)
#define HOBOT_DMA_SRAM_PA HOBOT_DMA_SRAM_DEBUG_DRAM_MODE //@FIXME:HOBOT_DMA
#define HOBOT_DMA_SRAM_SIZE (128*1024*4) //@FIXME:HOBOT_DMA
#define HOBOT_REGISTER_MONITOR 0
#define HOBOT_REGISTER_MONITOR_FILEWRITE 0
#define HOBOT_REGISTER_MONITOR_DUMP_STACK 1
#define HOBOT_NATIVE_WDR 0
#define HOBOT_SHOW_STRIDE 0//1
#define FW_ZONE_AE 0
#define IRIDIX_BYPASS_MAX_STR_CLIP 0
#define IRIDIX_HAS_CUSTOM_ASYMMETRY 1
#define IRIDIX_HAS_PRE_POST_GAMMA_LUT_LINEAR 0
#define IRIDIX_STRENGTH_BY_EXP_RATIO 0
#define IRIDIX_STRENGTH_TARGET 128
#define ISP_BINARY_SEQUENCE 0
#define ISP_CONTROLS_DMA_READER 1
#define ISP_DEFAULT_AF_ZONES_HOR 15
#define ISP_DEFAULT_AF_ZONES_VERT 15
#define ISP_DISPLAY_MODE 1080
#define ISP_DMA_RAW_BANKS 0
#define ISP_DMA_RAW_CAPTURE 0
#define ISP_FULL_HISTOGRAM_SIZE 1024
#define ISP_FW_BUILD 1
#define ISP_GAMMA_LUT_SIZE 129
#define ISP_HAS_AE_MANUAL_FSM 1
#define ISP_HAS_AF 1
#define ISP_HAS_AF_MANUAL_FSM 1
#define ISP_HAS_AWB_MANUAL_FSM 1
#define ISP_HAS_CMOS_FSM 1
#define ISP_HAS_COLOR_MATRIX_FSM 1
#define ISP_HAS_CONNECTION_BUFFER 0
#define ISP_HAS_CONNECTION_CHARDEV 1
#define ISP_HAS_CONNECTION_DEBUG 0
#define ISP_HAS_CONNECTION_SOCKET 0
#define ISP_HAS_CONNECTION_SOCKET_PORT 0
#define ISP_HAS_CONNECTION_UART 0
#define ISP_HAS_CROP_FSM 1
#define ISP_HAS_DMA_INPUT 0
#define ISP_HAS_DMA_WRITER_FSM 1
#define ISP_HAS_DS1 0
#define ISP_HAS_FPGA_DMA_FE_FSM 1
#define ISP_HAS_FPGA_WRAPPER 0
#define ISP_HAS_GAMMA_MANUAL_FSM 1
#define ISP_HAS_GENERAL_FSM 1
#define ISP_HAS_IRIDIX8_MANUAL_FSM 1
#define ISP_HAS_MATRIX_YUV_FSM 1
#define ISP_HAS_METADATA_FSM 1
#define ISP_HAS_META_CB 1
#define ISP_HAS_MONITOR_FSM 1
#define ISP_HAS_NOISE_REDUCTION_FSM 1
#define ISP_HAS_RAW_CB 0
#define ISP_HAS_SBUF_FSM 1
#define ISP_HAS_SENSOR_FSM 1
#define ISP_HAS_SHARPENING_FSM 1
#define ISP_HAS_SINTER_RADIAL_LUT 1
#define ISP_HAS_TEMPER 3
#define ISP_HAS_WDR_FRAME_BUFFER 0
#define ISP_INPUT_BITS 20
#define ISP_IRQ_DISABLE_ALL_IRQ 4294967295
#define ISP_IRQ_MASK_VECTOR 4289200114
#define ISP_MAX_CALIBRATION_DATA_SIZE (128 * 1024)
#define ISP_MAX_SENSOR_MODES 256
#define ISP_MBLAZE_DMA_COHERENT_DUMMY_ALLOC_BASE 1681915904
#define ISP_MBLAZE_DMA_COHERENT_DUMMY_ALLOC_SIZE 536870912
#define ISP_PIPE_BUFFER_SIZE 6
#define ISP_SENSOR_DRIVER_AD5821 0
#define ISP_SENSOR_DRIVER_AN41908A 0
#define ISP_SENSOR_DRIVER_BU64748 0
#define ISP_SENSOR_DRIVER_DONGWOON 0
#define ISP_SENSOR_DRIVER_DW9800 0
#define ISP_SENSOR_DRIVER_FP5510A 0
#define ISP_SENSOR_DRIVER_LC898201 0
#define ISP_SENSOR_DRIVER_MODEL 0
#define ISP_SENSOR_DRIVER_NULL 0
#define ISP_SENSOR_DRIVER_ROHM 0
#define ISP_SENSOR_DRIVER_V4L2 1
#define ISP_SOC_DMA_BUS_OFFSET 0
#define ISP_V4L2_DMA_COHERENT_DUMMY_ALLOC 1
#define ISP_V4L2_DMA_COHERENT_DUMMY_ALLOC_BASE 0x70000000
#define ISP_V4L2_DMA_COHERENT_DUMMY_ALLOC_SIZE 0x10000000
#define ISP_WDR_DEFAULT_MODE WDR_MODE_LINEAR
#define ISP_WDR_SWITCH 1
#define JUNO_DIRECT_DDR_ACCESS 0
#define KERNEL_MODULE 1
#define LOG2_GAIN_SHIFT 18
#define MAX_DMA_QUEUE_FRAMES ISP_PIPE_BUFFER_SIZE
#define OVEREXPOSE_TO_KEEP_ANTIFLICKER 0
#define PROJECT_DEPLOY 0
#define SENSOR_BINARY_SEQUENCE 0
#define SENSOR_DEFAULT_EXP_NUM 1
#define SENSOR_DEFAULT_PRESET_MODE 0
#define SENSOR_HAS_FLASH 0
#define SENSOR_HW_INTERFACE ACameraDefault
#define SENSOR_MASTER_CLOCK 27
#define SYSTEM_AE_COMPENSATION_DEFAULT 128
#define SYSTEM_ANTI_FLICKER_FREQUENCY_DEFAULT 50
#define SYSTEM_EXPOSURE_PARTITION_VALUE_COUNT 10
#define SYSTEM_EXPOSURE_RATIO_DEFAULT 8
#define SYSTEM_MANUAL_EXPOSURE_RATIO_DEFAULT 0
#define SYSTEM_MAXIMUM_IRIDIX_STRENGTH_DEFAULT 255
#define SYSTEM_MAX_EXPOSURE_RATIO_DEFAULT 16
#define SYSTEM_MINIMUM_IRIDIX_STRENGTH_DEFAULT 0
#define USER_MODULE 0
#define V4L2_FRAME_ID_SYNC 1
#define V4L2_INTERFACE_BUILD 1
#define V4L2_RUNNING_ON_JUNO 0
#define V4L2_SOC_SUBDEV_ENABLE 1
#define V4L2_SOC_SUBDEV_NUMBER 4//3
#define WDR_AUTO_SWITCH_TO WDR_MODE_LINEAR
#define WDR_SWITCH_EXPOSURE_CORRECTION 0
#define WDR_SWITCH_FRAMES 0
#define WDR_SWITCH_THRESHOLD 0
#define WDR_SWITCH_THRESHOLD_HISTERESIS 0
#endif
