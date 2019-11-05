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

#ifndef ACAMERA_LOGGER_H
#define ACAMERA_LOGGER_H

#include <linux/kernel.h>
#include <linux/module.h>

//defines for log level and system maximum value number for levels
#define LOG_DEBUG 0
#define LOG_INFO 1
#define LOG_NOTICE 2
#define LOG_WARNING 3
#define LOG_ERR 4
#define LOG_CRIT 5
#define LOG_NOTHING 6

/* Log Module Index and Masks */
#define LOG_MODULE_FW_GENERIC 0
#define LOG_MODULE_FW_GENERIC_MASK 1
#define LOG_MODULE_SENSOR 1
#define LOG_MODULE_SENSOR_MASK 2
#define LOG_MODULE_CMOS 2
#define LOG_MODULE_CMOS_MASK 4
#define LOG_MODULE_CROP 3
#define LOG_MODULE_CROP_MASK 8
#define LOG_MODULE_GENERAL 4
#define LOG_MODULE_GENERAL_MASK 16
#define LOG_MODULE_AE_MANUAL 5
#define LOG_MODULE_AE_MANUAL_MASK 32
#define LOG_MODULE_AWB_MANUAL 6
#define LOG_MODULE_AWB_MANUAL_MASK 64
#define LOG_MODULE_COLOR_MATRIX 7
#define LOG_MODULE_COLOR_MATRIX_MASK 128
#define LOG_MODULE_IRIDIX8_MANUAL 8
#define LOG_MODULE_IRIDIX8_MANUAL_MASK 256
#define LOG_MODULE_NOISE_REDUCTION 9
#define LOG_MODULE_NOISE_REDUCTION_MASK 512
#define LOG_MODULE_SHARPENING 10
#define LOG_MODULE_SHARPENING_MASK 1024
#define LOG_MODULE_MATRIX_YUV 11
#define LOG_MODULE_MATRIX_YUV_MASK 2048
#define LOG_MODULE_GAMMA_MANUAL 12
#define LOG_MODULE_GAMMA_MANUAL_MASK 4096
#define LOG_MODULE_MONITOR 13
#define LOG_MODULE_MONITOR_MASK 8192
#define LOG_MODULE_SBUF 14
#define LOG_MODULE_SBUF_MASK 16384
#define LOG_MODULE_DMA_WRITER 15
#define LOG_MODULE_DMA_WRITER_MASK 32768
#define LOG_MODULE_METADATA 16
#define LOG_MODULE_METADATA_MASK 65536
#define LOG_MODULE_AF_MANUAL 17
#define LOG_MODULE_AF_MANUAL_MASK 131072
#define LOG_MODULE_FPGA_DMA_FE 18
#define LOG_MODULE_FPGA_DMA_FE_MASK 262144
#define LOG_MODULE_SOC_IQ 19
#define LOG_MODULE_SOC_IQ_MASK 524288
#define LOG_MODULE_SOC_SENSOR 20
#define LOG_MODULE_SOC_SENSOR_MASK 1048576
#define LOG_MODULE_SOC_LENS 21
#define LOG_MODULE_SOC_LENS_MASK 2097152
#define LOG_MODULE_SOC_DWE 22
#define LOG_MODULE_SOC_DWE_MASK 4194304
#define LOG_MODULE_MAX 23 // now we have 22 sub module name
#define LOG_MODULE_ALL 0x7FFFFF



extern void LOG_INTER(const char *const func, const char *const file, const unsigned line, const uint32_t level, const uint32_t log_module, const char *const fmt, ... );
extern void ACAMERA_LOGGER_SET_LEVEL( uint8_t level );
extern void ACAMERA_LOGGER_SET_MASK(uint32_t mask );

#define CUR_MOD_NAME LOG_MODULE_SOC_LENS

#define LOG( level, ... )              \
    do {                                                      \
            LOG_INTER( __func__, __FILE__, __LINE__, \
                                level, CUR_MOD_NAME, __VA_ARGS__ );     \
    } while ( 0 )



#endif // ACAMERA_LOGGER_H
