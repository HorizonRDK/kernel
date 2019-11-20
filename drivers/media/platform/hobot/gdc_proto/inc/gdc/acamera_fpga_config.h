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


#ifndef __ACAMERA_FPGA_CONFIG_H__
#define __ACAMERA_FPGA_CONFIG_H__

/* Copyright should be here */

#include "system_gdc_io.h"

// ------------------------------------------------------------------------------ //
// Instance 'fpga' of module 'fpga_config_misc'
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_BASE_ADDR (0x209000L)
#define ACAMERA_FPGA_SIZE (0x400)

// ------------------------------------------------------------------------------ //
// Group: FPGA
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// FPGA
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: gdc_axi_read_raddr_39_32
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  Higher part [39:32] of address bus read part of GDC out AXI
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_FPGA_AXI_READ_RADDR_39_32_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_FPGA_AXI_READ_RADDR_39_32_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_FPGA_AXI_READ_RADDR_39_32_OFFSET (0x0)
#define ACAMERA_FPGA_FPGA_FPGA_AXI_READ_RADDR_39_32_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_fpga_axi_read_raddr_39_32_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209000L);
    system_gdc_write_32(0x209000L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_fpga_axi_read_raddr_39_32_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209000L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: gdc_axi_write_waddr_39_32
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  Higher part [39:32] of address bus write part of GDC out AXI
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_FPGA_AXI_WRITE_WADDR_39_32_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_FPGA_AXI_WRITE_WADDR_39_32_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_FPGA_AXI_WRITE_WADDR_39_32_OFFSET (0x4)
#define ACAMERA_FPGA_FPGA_FPGA_AXI_WRITE_WADDR_39_32_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_fpga_axi_write_waddr_39_32_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209004L);
    system_gdc_write_32(0x209004L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_fpga_axi_write_waddr_39_32_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209004L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: frame_reader_raddr_39_32
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  Higher part [39:32] of address bus read part of FPGA Frame Reader AXI
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_FRAME_READER_RADDR_39_32_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_FRAME_READER_RADDR_39_32_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_FRAME_READER_RADDR_39_32_OFFSET (0x8)
#define ACAMERA_FPGA_FPGA_FRAME_READER_RADDR_39_32_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_frame_reader_raddr_39_32_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209008L);
    system_gdc_write_32(0x209008L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_frame_reader_raddr_39_32_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209008L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: temper_frame_buffer_highaddr
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_TEMPER_FRAME_BUFFER_HIGHADDR_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_TEMPER_FRAME_BUFFER_HIGHADDR_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_TEMPER_FRAME_BUFFER_HIGHADDR_OFFSET (0xc)
#define ACAMERA_FPGA_FPGA_TEMPER_FRAME_BUFFER_HIGHADDR_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_temper_frame_buffer_highaddr_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20900cL);
    system_gdc_write_32(0x20900cL, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_temper_frame_buffer_highaddr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20900cL) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: dma_writer1_highaddr
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_DMA_WRITER1_HIGHADDR_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_DMA_WRITER1_HIGHADDR_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_DMA_WRITER1_HIGHADDR_OFFSET (0x10)
#define ACAMERA_FPGA_FPGA_DMA_WRITER1_HIGHADDR_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_dma_writer1_highaddr_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209010L);
    system_gdc_write_32(0x209010L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_dma_writer1_highaddr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209010L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: dma_writer2_highaddr
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_DMA_WRITER2_HIGHADDR_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_DMA_WRITER2_HIGHADDR_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_DMA_WRITER2_HIGHADDR_OFFSET (0x14)
#define ACAMERA_FPGA_FPGA_DMA_WRITER2_HIGHADDR_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_dma_writer2_highaddr_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209014L);
    system_gdc_write_32(0x209014L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_dma_writer2_highaddr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209014L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: dma_writer3_highaddr
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_DMA_WRITER3_HIGHADDR_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_DMA_WRITER3_HIGHADDR_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_DMA_WRITER3_HIGHADDR_OFFSET (0x18)
#define ACAMERA_FPGA_FPGA_DMA_WRITER3_HIGHADDR_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_dma_writer3_highaddr_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209018L);
    system_gdc_write_32(0x209018L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_dma_writer3_highaddr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209018L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: dma_writer4_highaddr
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_DMA_WRITER4_HIGHADDR_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_DMA_WRITER4_HIGHADDR_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_DMA_WRITER4_HIGHADDR_OFFSET (0x1c)
#define ACAMERA_FPGA_FPGA_DMA_WRITER4_HIGHADDR_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_dma_writer4_highaddr_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20901cL);
    system_gdc_write_32(0x20901cL, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_dma_writer4_highaddr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20901cL) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: dma_input_highaddr
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_DMA_INPUT_HIGHADDR_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_DMA_INPUT_HIGHADDR_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_DMA_INPUT_HIGHADDR_OFFSET (0x20)
#define ACAMERA_FPGA_FPGA_DMA_INPUT_HIGHADDR_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_dma_input_highaddr_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209020L);
    system_gdc_write_32(0x209020L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_dma_input_highaddr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209020L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: dma_writer_fr_highaddr
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_DMA_WRITER_FR_HIGHADDR_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_DMA_WRITER_FR_HIGHADDR_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_DMA_WRITER_FR_HIGHADDR_OFFSET (0x24)
#define ACAMERA_FPGA_FPGA_DMA_WRITER_FR_HIGHADDR_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_dma_writer_fr_highaddr_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209024L);
    system_gdc_write_32(0x209024L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_dma_writer_fr_highaddr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209024L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: dma_writer_fr_uv_highaddr
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FPGA_DMA_WRITER_FR_UV_HIGHADDR_DEFAULT (0x01)
#define ACAMERA_FPGA_FPGA_DMA_WRITER_FR_UV_HIGHADDR_DATASIZE (8)
#define ACAMERA_FPGA_FPGA_DMA_WRITER_FR_UV_HIGHADDR_OFFSET (0x28)
#define ACAMERA_FPGA_FPGA_DMA_WRITER_FR_UV_HIGHADDR_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fpga_dma_writer_fr_uv_highaddr_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209028L);
    system_gdc_write_32(0x209028L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fpga_dma_writer_fr_uv_highaddr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209028L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Group: Frame Reader
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: format
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_FORMAT_DEFAULT (0)
#define ACAMERA_FPGA_FRAME_READER_FORMAT_DATASIZE (8)
#define ACAMERA_FPGA_FRAME_READER_FORMAT_OFFSET (0x40)
#define ACAMERA_FPGA_FRAME_READER_FORMAT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_frame_reader_format_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209040L);
    system_gdc_write_32(0x209040L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_frame_reader_format_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209040L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: rbase load
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_RBASE_LOAD_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_RBASE_LOAD_DATASIZE (1)
#define ACAMERA_FPGA_FRAME_READER_RBASE_LOAD_OFFSET (0x4c)
#define ACAMERA_FPGA_FRAME_READER_RBASE_LOAD_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_frame_reader_rbase_load_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20904cL);
    system_gdc_write_32(0x20904cL, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_frame_reader_rbase_load_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20904cL) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: rbase load sel
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Selector for rbase_load strobe: 0-field, 1-configuration bit rbase_load
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_RBASE_LOAD_SEL_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_RBASE_LOAD_SEL_DATASIZE (1)
#define ACAMERA_FPGA_FRAME_READER_RBASE_LOAD_SEL_OFFSET (0x4c)
#define ACAMERA_FPGA_FRAME_READER_RBASE_LOAD_SEL_MASK (0x10)

// args: data (1-bit)
static __inline void acamera_fpga_frame_reader_rbase_load_sel_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20904cL);
    system_gdc_write_32(0x20904cL, (((uint32_t) (data & 0x1)) << 4) | (curr & 0xffffffef));
}
static __inline uint8_t acamera_fpga_frame_reader_rbase_load_sel_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20904cL) & 0x10) >> 4);
}
// ------------------------------------------------------------------------------ //
// Register: rbase
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_RBASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_RBASE_DATASIZE (32)
#define ACAMERA_FPGA_FRAME_READER_RBASE_OFFSET (0x50)
#define ACAMERA_FPGA_FRAME_READER_RBASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_frame_reader_rbase_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209050L, data);
}
static __inline uint32_t acamera_fpga_frame_reader_rbase_read(uint32_t base) {
    return system_gdc_read_32(0x209050L);
}
// ------------------------------------------------------------------------------ //
// Register: Line_offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Indicates offset in bytes from the start of one line to the next line. Should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_LINE_OFFSET_DEFAULT (0x1000)
#define ACAMERA_FPGA_FRAME_READER_LINE_OFFSET_DATASIZE (32)
#define ACAMERA_FPGA_FRAME_READER_LINE_OFFSET_OFFSET (0x54)
#define ACAMERA_FPGA_FRAME_READER_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_frame_reader_line_offset_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209054L, data);
}
static __inline uint32_t acamera_fpga_frame_reader_line_offset_read(uint32_t base) {
    return system_gdc_read_32(0x209054L);
}
// ------------------------------------------------------------------------------ //
// Register: axi_port_enable
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_AXI_PORT_ENABLE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_AXI_PORT_ENABLE_DATASIZE (1)
#define ACAMERA_FPGA_FRAME_READER_AXI_PORT_ENABLE_OFFSET (0x58)
#define ACAMERA_FPGA_FRAME_READER_AXI_PORT_ENABLE_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_frame_reader_axi_port_enable_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209058L);
    system_gdc_write_32(0x209058L, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_frame_reader_axi_port_enable_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209058L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: config
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_CONFIG_DEFAULT (0x0000)
#define ACAMERA_FPGA_FRAME_READER_CONFIG_DATASIZE (32)
#define ACAMERA_FPGA_FRAME_READER_CONFIG_OFFSET (0x60)
#define ACAMERA_FPGA_FRAME_READER_CONFIG_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_frame_reader_config_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209060L, data);
}
static __inline uint32_t acamera_fpga_frame_reader_config_read(uint32_t base) {
    return system_gdc_read_32(0x209060L);
}
// ------------------------------------------------------------------------------ //
// Register: status
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_STATUS_DEFAULT (0x0000)
#define ACAMERA_FPGA_FRAME_READER_STATUS_DATASIZE (32)
#define ACAMERA_FPGA_FRAME_READER_STATUS_OFFSET (0x64)
#define ACAMERA_FPGA_FRAME_READER_STATUS_MASK (0xffffffff)

// args: data (32-bit)
static __inline uint32_t acamera_fpga_frame_reader_status_read(uint32_t base) {
    return system_gdc_read_32(0x209064L);
}
// ------------------------------------------------------------------------------ //
// Register: active_width
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_ACTIVE_WIDTH_DEFAULT (0x0780)
#define ACAMERA_FPGA_FRAME_READER_ACTIVE_WIDTH_DATASIZE (16)
#define ACAMERA_FPGA_FRAME_READER_ACTIVE_WIDTH_OFFSET (0x68)
#define ACAMERA_FPGA_FRAME_READER_ACTIVE_WIDTH_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_frame_reader_active_width_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209068L);
    system_gdc_write_32(0x209068L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_frame_reader_active_width_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209068L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: active_height
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_ACTIVE_HEIGHT_DEFAULT (0x0438)
#define ACAMERA_FPGA_FRAME_READER_ACTIVE_HEIGHT_DATASIZE (16)
#define ACAMERA_FPGA_FRAME_READER_ACTIVE_HEIGHT_OFFSET (0x6c)
#define ACAMERA_FPGA_FRAME_READER_ACTIVE_HEIGHT_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_frame_reader_active_height_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x20906cL);
    system_gdc_write_32(0x20906cL, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_frame_reader_active_height_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x20906cL) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Group: Frame Reader UV
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: format
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_FORMAT_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_UV_FORMAT_DATASIZE (8)
#define ACAMERA_FPGA_FRAME_READER_UV_FORMAT_OFFSET (0x80)
#define ACAMERA_FPGA_FRAME_READER_UV_FORMAT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_frame_reader_uv_format_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209080L);
    system_gdc_write_32(0x209080L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_frame_reader_uv_format_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209080L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: disable422_uv_interleave
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_DISABLE422_UV_INTERLEAVE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_UV_DISABLE422_UV_INTERLEAVE_DATASIZE (1)
#define ACAMERA_FPGA_FRAME_READER_UV_DISABLE422_UV_INTERLEAVE_OFFSET (0x8c)
#define ACAMERA_FPGA_FRAME_READER_UV_DISABLE422_UV_INTERLEAVE_MASK (0x2)

// args: data (1-bit)
static __inline void acamera_fpga_frame_reader_uv_disable422_uv_interleave_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20908cL);
    system_gdc_write_32(0x20908cL, (((uint32_t) (data & 0x1)) << 1) | (curr & 0xfffffffd));
}
static __inline uint8_t acamera_fpga_frame_reader_uv_disable422_uv_interleave_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20908cL) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: repeat_downsampled_lines
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_REPEAT_DOWNSAMPLED_LINES_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_UV_REPEAT_DOWNSAMPLED_LINES_DATASIZE (1)
#define ACAMERA_FPGA_FRAME_READER_UV_REPEAT_DOWNSAMPLED_LINES_OFFSET (0x8c)
#define ACAMERA_FPGA_FRAME_READER_UV_REPEAT_DOWNSAMPLED_LINES_MASK (0x4)

// args: data (1-bit)
static __inline void acamera_fpga_frame_reader_uv_repeat_downsampled_lines_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20908cL);
    system_gdc_write_32(0x20908cL, (((uint32_t) (data & 0x1)) << 2) | (curr & 0xfffffffb));
}
static __inline uint8_t acamera_fpga_frame_reader_uv_repeat_downsampled_lines_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20908cL) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: repeat_downsampled_pixels
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_REPEAT_DOWNSAMPLED_PIXELS_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_UV_REPEAT_DOWNSAMPLED_PIXELS_DATASIZE (1)
#define ACAMERA_FPGA_FRAME_READER_UV_REPEAT_DOWNSAMPLED_PIXELS_OFFSET (0x8c)
#define ACAMERA_FPGA_FRAME_READER_UV_REPEAT_DOWNSAMPLED_PIXELS_MASK (0x8)

// args: data (1-bit)
static __inline void acamera_fpga_frame_reader_uv_repeat_downsampled_pixels_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20908cL);
    system_gdc_write_32(0x20908cL, (((uint32_t) (data & 0x1)) << 3) | (curr & 0xfffffff7));
}
static __inline uint8_t acamera_fpga_frame_reader_uv_repeat_downsampled_pixels_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20908cL) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: rbase load
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_LOAD_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_LOAD_DATASIZE (1)
#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_LOAD_OFFSET (0x90)
#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_LOAD_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_frame_reader_uv_rbase_load_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209090L);
    system_gdc_write_32(0x209090L, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_frame_reader_uv_rbase_load_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209090L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: rbase load sel
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Selector for rbase_load strobe: 0-field, 1-configuration bit rbase_load
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_LOAD_SEL_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_LOAD_SEL_DATASIZE (1)
#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_LOAD_SEL_OFFSET (0x90)
#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_LOAD_SEL_MASK (0x10)

// args: data (1-bit)
static __inline void acamera_fpga_frame_reader_uv_rbase_load_sel_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209090L);
    system_gdc_write_32(0x209090L, (((uint32_t) (data & 0x1)) << 4) | (curr & 0xffffffef));
}
static __inline uint8_t acamera_fpga_frame_reader_uv_rbase_load_sel_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209090L) & 0x10) >> 4);
}
// ------------------------------------------------------------------------------ //
// Register: rbase
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_DATASIZE (32)
#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_OFFSET (0x94)
#define ACAMERA_FPGA_FRAME_READER_UV_RBASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_frame_reader_uv_rbase_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209094L, data);
}
static __inline uint32_t acamera_fpga_frame_reader_uv_rbase_read(uint32_t base) {
    return system_gdc_read_32(0x209094L);
}
// ------------------------------------------------------------------------------ //
// Register: Line_offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Indicates offset in bytes from the start of one line to the next line. Should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_LINE_OFFSET_DEFAULT (0x1000)
#define ACAMERA_FPGA_FRAME_READER_UV_LINE_OFFSET_DATASIZE (32)
#define ACAMERA_FPGA_FRAME_READER_UV_LINE_OFFSET_OFFSET (0x98)
#define ACAMERA_FPGA_FRAME_READER_UV_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_frame_reader_uv_line_offset_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209098L, data);
}
static __inline uint32_t acamera_fpga_frame_reader_uv_line_offset_read(uint32_t base) {
    return system_gdc_read_32(0x209098L);
}
// ------------------------------------------------------------------------------ //
// Register: axi_port_enable
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_AXI_PORT_ENABLE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRAME_READER_UV_AXI_PORT_ENABLE_DATASIZE (1)
#define ACAMERA_FPGA_FRAME_READER_UV_AXI_PORT_ENABLE_OFFSET (0x9c)
#define ACAMERA_FPGA_FRAME_READER_UV_AXI_PORT_ENABLE_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_frame_reader_uv_axi_port_enable_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20909cL);
    system_gdc_write_32(0x20909cL, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_frame_reader_uv_axi_port_enable_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20909cL) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: config
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_CONFIG_DEFAULT (0x0000)
#define ACAMERA_FPGA_FRAME_READER_UV_CONFIG_DATASIZE (32)
#define ACAMERA_FPGA_FRAME_READER_UV_CONFIG_OFFSET (0xa0)
#define ACAMERA_FPGA_FRAME_READER_UV_CONFIG_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_frame_reader_uv_config_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x2090a0L, data);
}
static __inline uint32_t acamera_fpga_frame_reader_uv_config_read(uint32_t base) {
    return system_gdc_read_32(0x2090a0L);
}
// ------------------------------------------------------------------------------ //
// Register: status
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_STATUS_DEFAULT (0x0000)
#define ACAMERA_FPGA_FRAME_READER_UV_STATUS_DATASIZE (32)
#define ACAMERA_FPGA_FRAME_READER_UV_STATUS_OFFSET (0xa4)
#define ACAMERA_FPGA_FRAME_READER_UV_STATUS_MASK (0xffffffff)

// args: data (32-bit)
static __inline uint32_t acamera_fpga_frame_reader_uv_status_read(uint32_t base) {
    return system_gdc_read_32(0x2090a4L);
}
// ------------------------------------------------------------------------------ //
// Register: active_width
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_ACTIVE_WIDTH_DEFAULT (0x0780)
#define ACAMERA_FPGA_FRAME_READER_UV_ACTIVE_WIDTH_DATASIZE (16)
#define ACAMERA_FPGA_FRAME_READER_UV_ACTIVE_WIDTH_OFFSET (0xa8)
#define ACAMERA_FPGA_FRAME_READER_UV_ACTIVE_WIDTH_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_frame_reader_uv_active_width_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x2090a8L);
    system_gdc_write_32(0x2090a8L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_frame_reader_uv_active_width_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x2090a8L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: active_height
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRAME_READER_UV_ACTIVE_HEIGHT_DEFAULT (0x0438)
#define ACAMERA_FPGA_FRAME_READER_UV_ACTIVE_HEIGHT_DATASIZE (16)
#define ACAMERA_FPGA_FRAME_READER_UV_ACTIVE_HEIGHT_OFFSET (0xac)
#define ACAMERA_FPGA_FRAME_READER_UV_ACTIVE_HEIGHT_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_frame_reader_uv_active_height_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x2090acL);
    system_gdc_write_32(0x2090acL, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_frame_reader_uv_active_height_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x2090acL) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Group: Horizontal Shift
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: Offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// :Pixel resolution shift offset
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_HORIZONTAL_SHIFT_OFFSET_DEFAULT (0x0)
#define ACAMERA_FPGA_HORIZONTAL_SHIFT_OFFSET_DATASIZE (5)
#define ACAMERA_FPGA_HORIZONTAL_SHIFT_OFFSET_OFFSET (0x120)
#define ACAMERA_FPGA_HORIZONTAL_SHIFT_OFFSET_MASK (0x1f)

// args: data (5-bit)
static __inline void acamera_fpga_horizontal_shift_offset_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209120L);
    system_gdc_write_32(0x209120L, (((uint32_t) (data & 0x1f)) << 0) | (curr & 0xffffffe0));
}
static __inline uint8_t acamera_fpga_horizontal_shift_offset_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209120L) & 0x1f) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Enable
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_HORIZONTAL_SHIFT_ENABLE_DEFAULT (0x0)
#define ACAMERA_FPGA_HORIZONTAL_SHIFT_ENABLE_DATASIZE (1)
#define ACAMERA_FPGA_HORIZONTAL_SHIFT_ENABLE_OFFSET (0x120)
#define ACAMERA_FPGA_HORIZONTAL_SHIFT_ENABLE_MASK (0x10000)

// args: data (1-bit)
static __inline void acamera_fpga_horizontal_shift_enable_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209120L);
    system_gdc_write_32(0x209120L, (((uint32_t) (data & 0x1)) << 16) | (curr & 0xfffeffff));
}
static __inline uint8_t acamera_fpga_horizontal_shift_enable_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209120L) & 0x10000) >> 16);
}
// ------------------------------------------------------------------------------ //
// Group: Video DMA Writer FR
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Full resolution video DMA writer controls
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: Format
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Format
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_FORMAT_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_FORMAT_DATASIZE (8)
#define ACAMERA_FPGA_FR_DMA_WRITER_FORMAT_OFFSET (0x128)
#define ACAMERA_FPGA_FR_DMA_WRITER_FORMAT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fr_dma_writer_format_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209128L);
    system_gdc_write_32(0x209128L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_format_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209128L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Base mode
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base DMA packing mode for RGB/RAW/YUV etc (see ISP guide)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_BASE_MODE_DEFAULT (0x0D)
#define ACAMERA_FPGA_FR_DMA_WRITER_BASE_MODE_DATASIZE (4)
#define ACAMERA_FPGA_FR_DMA_WRITER_BASE_MODE_OFFSET (0x128)
#define ACAMERA_FPGA_FR_DMA_WRITER_BASE_MODE_MASK (0xf)

// args: data (4-bit)
static __inline void acamera_fpga_fr_dma_writer_base_mode_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209128L);
    system_gdc_write_32(0x209128L, (((uint32_t) (data & 0xf)) << 0) | (curr & 0xfffffff0));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_base_mode_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209128L) & 0xf) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Plane select
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Plane select for planar base modes.  Only used if planar outputs required.  Not used.  Should be set to 0
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_PLANE_SELECT_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_PLANE_SELECT_DATASIZE (2)
#define ACAMERA_FPGA_FR_DMA_WRITER_PLANE_SELECT_OFFSET (0x128)
#define ACAMERA_FPGA_FR_DMA_WRITER_PLANE_SELECT_MASK (0xc0)

// args: data (2-bit)
static __inline void acamera_fpga_fr_dma_writer_plane_select_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209128L);
    system_gdc_write_32(0x209128L, (((uint32_t) (data & 0x3)) << 6) | (curr & 0xffffff3f));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_plane_select_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209128L) & 0xc0) >> 6);
}
// ------------------------------------------------------------------------------ //
// Register: single frame
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = All frames are written(after frame_write_on= 1), 1= only 1st frame written ( after frame_write_on =1)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_SINGLE_FRAME_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_SINGLE_FRAME_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_SINGLE_FRAME_OFFSET (0x12c)
#define ACAMERA_FPGA_FR_DMA_WRITER_SINGLE_FRAME_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_single_frame_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20912cL);
    system_gdc_write_32(0x20912cL, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_single_frame_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20912cL) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: frame write on
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 
//    0 = no frames written(when switched from 1, current frame completes writing before stopping),
//    1= write frame(s) (write single or continous frame(s) )
//    
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WRITE_ON_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WRITE_ON_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WRITE_ON_OFFSET (0x12c)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WRITE_ON_MASK (0x2)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_frame_write_on_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20912cL);
    system_gdc_write_32(0x20912cL, (((uint32_t) (data & 0x1)) << 1) | (curr & 0xfffffffd));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_frame_write_on_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20912cL) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: half irate
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = normal operation , 1= write half(alternate) of input frames( only valid for continuous mode)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_HALF_IRATE_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_HALF_IRATE_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_HALF_IRATE_OFFSET (0x12c)
#define ACAMERA_FPGA_FR_DMA_WRITER_HALF_IRATE_MASK (0x4)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_half_irate_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20912cL);
    system_gdc_write_32(0x20912cL, (((uint32_t) (data & 0x1)) << 2) | (curr & 0xfffffffb));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_half_irate_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20912cL) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: axi xact comp
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = dont wait for axi transaction completion at end of frame(just all transfers accepted). 1 = wait for all transactions completed
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_XACT_COMP_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_XACT_COMP_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_XACT_COMP_OFFSET (0x12c)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_XACT_COMP_MASK (0x8)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_axi_xact_comp_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20912cL);
    system_gdc_write_32(0x20912cL, (((uint32_t) (data & 0x1)) << 3) | (curr & 0xfffffff7));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_axi_xact_comp_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20912cL) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: active width
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Active video width in pixels 128-8000
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_ACTIVE_WIDTH_DEFAULT (0x780)
#define ACAMERA_FPGA_FR_DMA_WRITER_ACTIVE_WIDTH_DATASIZE (16)
#define ACAMERA_FPGA_FR_DMA_WRITER_ACTIVE_WIDTH_OFFSET (0x130)
#define ACAMERA_FPGA_FR_DMA_WRITER_ACTIVE_WIDTH_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_fr_dma_writer_active_width_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209130L);
    system_gdc_write_32(0x209130L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_fr_dma_writer_active_width_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209130L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: active height
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Active video height in lines 128-8000
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_ACTIVE_HEIGHT_DEFAULT (0x438)
#define ACAMERA_FPGA_FR_DMA_WRITER_ACTIVE_HEIGHT_DATASIZE (16)
#define ACAMERA_FPGA_FR_DMA_WRITER_ACTIVE_HEIGHT_OFFSET (0x134)
#define ACAMERA_FPGA_FR_DMA_WRITER_ACTIVE_HEIGHT_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_fr_dma_writer_active_height_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209134L);
    system_gdc_write_32(0x209134L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_fr_dma_writer_active_height_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209134L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: bank0_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 0 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_BANK0_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK0_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK0_BASE_OFFSET (0x138)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK0_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fr_dma_writer_bank0_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209138L, data);
}
static __inline uint32_t acamera_fpga_fr_dma_writer_bank0_base_read(uint32_t base) {
    return system_gdc_read_32(0x209138L);
}
// ------------------------------------------------------------------------------ //
// Register: bank1_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 1 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_BANK1_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK1_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK1_BASE_OFFSET (0x13c)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK1_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fr_dma_writer_bank1_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x20913cL, data);
}
static __inline uint32_t acamera_fpga_fr_dma_writer_bank1_base_read(uint32_t base) {
    return system_gdc_read_32(0x20913cL);
}
// ------------------------------------------------------------------------------ //
// Register: bank2_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 2 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_BANK2_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK2_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK2_BASE_OFFSET (0x140)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK2_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fr_dma_writer_bank2_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209140L, data);
}
static __inline uint32_t acamera_fpga_fr_dma_writer_bank2_base_read(uint32_t base) {
    return system_gdc_read_32(0x209140L);
}
// ------------------------------------------------------------------------------ //
// Register: bank3_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 3 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_BANK3_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK3_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK3_BASE_OFFSET (0x144)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK3_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fr_dma_writer_bank3_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209144L, data);
}
static __inline uint32_t acamera_fpga_fr_dma_writer_bank3_base_read(uint32_t base) {
    return system_gdc_read_32(0x209144L);
}
// ------------------------------------------------------------------------------ //
// Register: bank4_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 4 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_BANK4_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK4_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK4_BASE_OFFSET (0x148)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK4_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fr_dma_writer_bank4_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209148L, data);
}
static __inline uint32_t acamera_fpga_fr_dma_writer_bank4_base_read(uint32_t base) {
    return system_gdc_read_32(0x209148L);
}
// ------------------------------------------------------------------------------ //
// Register: max bank
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// highest bank*_base to use for frame writes before recycling to bank0_base, only 0 to 4 are valid
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BANK_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BANK_DATASIZE (3)
#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BANK_OFFSET (0x14c)
#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BANK_MASK (0x7)

// args: data (3-bit)
static __inline void acamera_fpga_fr_dma_writer_max_bank_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20914cL);
    system_gdc_write_32(0x20914cL, (((uint32_t) (data & 0x7)) << 0) | (curr & 0xfffffff8));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_max_bank_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20914cL) & 0x7) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: bank0 restart
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = normal operation, 1= restart bank counter to bank0 for next frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_BANK0_RESTART_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK0_RESTART_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK0_RESTART_OFFSET (0x14c)
#define ACAMERA_FPGA_FR_DMA_WRITER_BANK0_RESTART_MASK (0x8)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_bank0_restart_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20914cL);
    system_gdc_write_32(0x20914cL, (((uint32_t) (data & 0x1)) << 3) | (curr & 0xfffffff7));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_bank0_restart_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20914cL) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: Line_offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 
//    Indicates the offset in bytes from the start of one line to the next line.  
//    This value should be equal to or larger than one line of image data and should be word-aligned
//    
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_LINE_OFFSET_DEFAULT (0x4000)
#define ACAMERA_FPGA_FR_DMA_WRITER_LINE_OFFSET_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_LINE_OFFSET_OFFSET (0x150)
#define ACAMERA_FPGA_FR_DMA_WRITER_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fr_dma_writer_line_offset_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209150L, data);
}
static __inline uint32_t acamera_fpga_fr_dma_writer_line_offset_read(uint32_t base) {
    return system_gdc_read_32(0x209150L);
}
// ------------------------------------------------------------------------------ //
// Register: frame write cancel
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = normal operation, 1= cancel current/future frame write(s), any unstarted AXI bursts cancelled and fifo flushed
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WRITE_CANCEL_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WRITE_CANCEL_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WRITE_CANCEL_OFFSET (0x154)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WRITE_CANCEL_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_frame_write_cancel_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209154L);
    system_gdc_write_32(0x209154L, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_frame_write_cancel_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209154L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: axi_port_enable
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// enables axi, active high, 1=enables axi write transfers, 0= reset axi domain( via reset synchroniser)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_PORT_ENABLE_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_PORT_ENABLE_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_PORT_ENABLE_OFFSET (0x154)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_PORT_ENABLE_MASK (0x2)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_axi_port_enable_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209154L);
    system_gdc_write_32(0x209154L, (((uint32_t) (data & 0x1)) << 1) | (curr & 0xfffffffd));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_axi_port_enable_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209154L) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: wbank curr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// write bank currently active. valid values =0-4. updated at start of frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_CURR_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_CURR_DATASIZE (3)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_CURR_OFFSET (0x158)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_CURR_MASK (0x7)

// args: data (3-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_wbank_curr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209158L) & 0x7) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: wbank last
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// write bank last active. valid values = 0-4. updated at start of frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_LAST_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_LAST_DATASIZE (3)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_LAST_OFFSET (0x158)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_LAST_MASK (0x38)

// args: data (3-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_wbank_last_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209158L) & 0x38) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: wbank active
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1 = wbank_curr is being written to. Goes high at start of writes, low at last write transfer/completion on axi. 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_ACTIVE_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_ACTIVE_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_ACTIVE_OFFSET (0x15c)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_ACTIVE_MASK (0x1)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_wbank_active_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20915cL) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: wbank start
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1 = High pulse at start of frame write to bank. 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_START_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_START_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_START_OFFSET (0x15c)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_START_MASK (0x2)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_wbank_start_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20915cL) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: wbank stop
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1 = High pulse at end of frame write to bank. 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_STOP_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_STOP_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_STOP_OFFSET (0x15c)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBANK_STOP_MASK (0x4)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_wbank_stop_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20915cL) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: wbase curr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// currently active bank base addr - in bytes. updated at start of frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WBASE_CURR_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBASE_CURR_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBASE_CURR_OFFSET (0x160)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBASE_CURR_MASK (0xffffffff)

// args: data (32-bit)
static __inline uint32_t acamera_fpga_fr_dma_writer_wbase_curr_read(uint32_t base) {
    return system_gdc_read_32(0x209160L);
}
// ------------------------------------------------------------------------------ //
// Register: wbase last
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// last active bank base addr - in bytes. Updated at start of frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WBASE_LAST_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBASE_LAST_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBASE_LAST_OFFSET (0x164)
#define ACAMERA_FPGA_FR_DMA_WRITER_WBASE_LAST_MASK (0xffffffff)

// args: data (32-bit)
static __inline uint32_t acamera_fpga_fr_dma_writer_wbase_last_read(uint32_t base) {
    return system_gdc_read_32(0x209164L);
}
// ------------------------------------------------------------------------------ //
// Register: frame icount
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// count of incomming frames (starts) to vdma_writer on video input, non resetable, rolls over, updates at pixel 1 of new frame on video in
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_ICOUNT_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_ICOUNT_DATASIZE (16)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_ICOUNT_OFFSET (0x168)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_ICOUNT_MASK (0xffff)

// args: data (16-bit)
static __inline uint16_t acamera_fpga_fr_dma_writer_frame_icount_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209168L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: frame wcount
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// count of outgoing frame writes (starts) from vdma_writer sent to AXI output, non resetable, rolls over, updates at pixel 1 of new frame on video in
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WCOUNT_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WCOUNT_DATASIZE (16)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WCOUNT_OFFSET (0x16c)
#define ACAMERA_FPGA_FR_DMA_WRITER_FRAME_WCOUNT_MASK (0xffff)

// args: data (16-bit)
static __inline uint16_t acamera_fpga_fr_dma_writer_frame_wcount_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x20916cL) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: clear alarms
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0>1 transition(synchronous detection) causes local axi/video alarm clear
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_CLEAR_ALARMS_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_CLEAR_ALARMS_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_CLEAR_ALARMS_OFFSET (0x170)
#define ACAMERA_FPGA_FR_DMA_WRITER_CLEAR_ALARMS_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_clear_alarms_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209170L);
    system_gdc_write_32(0x209170L, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_clear_alarms_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209170L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: max_burst_length_is_8
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1= Reduce default AXI max_burst_length from 16 to 8, 0= Dont reduce
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BURST_LENGTH_IS_8_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BURST_LENGTH_IS_8_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BURST_LENGTH_IS_8_OFFSET (0x170)
#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BURST_LENGTH_IS_8_MASK (0x2)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_max_burst_length_is_8_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209170L);
    system_gdc_write_32(0x209170L, (((uint32_t) (data & 0x1)) << 1) | (curr & 0xfffffffd));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_max_burst_length_is_8_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209170L) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: max_burst_length_is_4
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1= Reduce default AXI max_burst_length from 16 to 4, 0= Dont reduce( has priority overmax_burst_length_is_8!)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BURST_LENGTH_IS_4_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BURST_LENGTH_IS_4_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BURST_LENGTH_IS_4_OFFSET (0x170)
#define ACAMERA_FPGA_FR_DMA_WRITER_MAX_BURST_LENGTH_IS_4_MASK (0x4)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_max_burst_length_is_4_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209170L);
    system_gdc_write_32(0x209170L, (((uint32_t) (data & 0x1)) << 2) | (curr & 0xfffffffb));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_max_burst_length_is_4_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209170L) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: write timeout disable
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 
//    At end of frame an optional timeout is applied to wait for AXI writes to completed/accepted befotre caneclling and flushing.
//    0= Timeout Enabled, timeout count can decrement.
//    1 = Disable timeout, timeout count can't decrement. 
//    
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WRITE_TIMEOUT_DISABLE_DEFAULT (0)
#define ACAMERA_FPGA_FR_DMA_WRITER_WRITE_TIMEOUT_DISABLE_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_WRITE_TIMEOUT_DISABLE_OFFSET (0x170)
#define ACAMERA_FPGA_FR_DMA_WRITER_WRITE_TIMEOUT_DISABLE_MASK (0x8)

// args: data (1-bit)
static __inline void acamera_fpga_fr_dma_writer_write_timeout_disable_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209170L);
    system_gdc_write_32(0x209170L, (((uint32_t) (data & 0x1)) << 3) | (curr & 0xfffffff7));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_write_timeout_disable_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209170L) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: awmaxwait_limit
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// awvalid maxwait limit(cycles) to raise axi_fail_awmaxwait alarm . zero disables alarm raise.
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_AWMAXWAIT_LIMIT_DEFAULT (0x00)
#define ACAMERA_FPGA_FR_DMA_WRITER_AWMAXWAIT_LIMIT_DATASIZE (8)
#define ACAMERA_FPGA_FR_DMA_WRITER_AWMAXWAIT_LIMIT_OFFSET (0x174)
#define ACAMERA_FPGA_FR_DMA_WRITER_AWMAXWAIT_LIMIT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fr_dma_writer_awmaxwait_limit_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209174L);
    system_gdc_write_32(0x209174L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_awmaxwait_limit_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209174L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: wmaxwait_limit
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// wvalid maxwait limit(cycles) to raise axi_fail_wmaxwait alarm . zero disables alarm raise
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WMAXWAIT_LIMIT_DEFAULT (0x00)
#define ACAMERA_FPGA_FR_DMA_WRITER_WMAXWAIT_LIMIT_DATASIZE (8)
#define ACAMERA_FPGA_FR_DMA_WRITER_WMAXWAIT_LIMIT_OFFSET (0x178)
#define ACAMERA_FPGA_FR_DMA_WRITER_WMAXWAIT_LIMIT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fr_dma_writer_wmaxwait_limit_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209178L);
    system_gdc_write_32(0x209178L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_wmaxwait_limit_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209178L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: wxact_ostand_limit
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// number oustsanding write transactions(bursts)(responses..1 per burst) limit to raise axi_fail_wxact_ostand. zero disables alarm raise
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_WXACT_OSTAND_LIMIT_DEFAULT (0x00)
#define ACAMERA_FPGA_FR_DMA_WRITER_WXACT_OSTAND_LIMIT_DATASIZE (8)
#define ACAMERA_FPGA_FR_DMA_WRITER_WXACT_OSTAND_LIMIT_OFFSET (0x17c)
#define ACAMERA_FPGA_FR_DMA_WRITER_WXACT_OSTAND_LIMIT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fr_dma_writer_wxact_ostand_limit_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x20917cL);
    system_gdc_write_32(0x20917cL, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fr_dma_writer_wxact_ostand_limit_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x20917cL) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: axi_fail_bresp
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate bad  bresp captured 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_BRESP_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_BRESP_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_BRESP_OFFSET (0x180)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_BRESP_MASK (0x1)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_axi_fail_bresp_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209180L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: axi_fail_awmaxwait
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high when awmaxwait_limit reached 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_AWMAXWAIT_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_AWMAXWAIT_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_AWMAXWAIT_OFFSET (0x180)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_AWMAXWAIT_MASK (0x2)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_axi_fail_awmaxwait_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209180L) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: axi_fail_wmaxwait
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high when wmaxwait_limit reached 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_WMAXWAIT_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_WMAXWAIT_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_WMAXWAIT_OFFSET (0x180)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_WMAXWAIT_MASK (0x4)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_axi_fail_wmaxwait_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209180L) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: axi_fail_wxact_ostand
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high when wxact_ostand_limit reached 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_WXACT_OSTAND_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_WXACT_OSTAND_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_WXACT_OSTAND_OFFSET (0x180)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_FAIL_WXACT_OSTAND_MASK (0x8)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_axi_fail_wxact_ostand_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209180L) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: vi_fail_active_width
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate mismatched active_width detected 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_ACTIVE_WIDTH_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_ACTIVE_WIDTH_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_ACTIVE_WIDTH_OFFSET (0x180)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_ACTIVE_WIDTH_MASK (0x10)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_vi_fail_active_width_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209180L) & 0x10) >> 4);
}
// ------------------------------------------------------------------------------ //
// Register: vi_fail_active_height
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate mismatched active_height detected ( also raised on missing field!) 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_ACTIVE_HEIGHT_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_ACTIVE_HEIGHT_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_ACTIVE_HEIGHT_OFFSET (0x180)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_ACTIVE_HEIGHT_MASK (0x20)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_vi_fail_active_height_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209180L) & 0x20) >> 5);
}
// ------------------------------------------------------------------------------ //
// Register: vi_fail_interline_blanks
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate interline blanking below min 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_INTERLINE_BLANKS_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_INTERLINE_BLANKS_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_INTERLINE_BLANKS_OFFSET (0x180)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_INTERLINE_BLANKS_MASK (0x40)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_vi_fail_interline_blanks_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209180L) & 0x40) >> 6);
}
// ------------------------------------------------------------------------------ //
// Register: vi_fail_interframe_blanks
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate interframe blanking below min 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_INTERFRAME_BLANKS_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_INTERFRAME_BLANKS_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_INTERFRAME_BLANKS_OFFSET (0x180)
#define ACAMERA_FPGA_FR_DMA_WRITER_VI_FAIL_INTERFRAME_BLANKS_MASK (0x80)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_vi_fail_interframe_blanks_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209180L) & 0x80) >> 7);
}
// ------------------------------------------------------------------------------ //
// Register: video_alarm
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  active high, problem found on video port(s) ( active width/height or interline/frame blanks failure) 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_VIDEO_ALARM_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_VIDEO_ALARM_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_VIDEO_ALARM_OFFSET (0x184)
#define ACAMERA_FPGA_FR_DMA_WRITER_VIDEO_ALARM_MASK (0x1)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_video_alarm_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209184L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: axi_alarm
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  active high, problem found on axi port(s)( bresp or awmaxwait or wmaxwait or wxact_ostand failure ) 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_ALARM_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_ALARM_DATASIZE (1)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_ALARM_OFFSET (0x184)
#define ACAMERA_FPGA_FR_DMA_WRITER_AXI_ALARM_MASK (0x2)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fr_dma_writer_axi_alarm_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209184L) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: blk_config
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// block configuration (reserved)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_BLK_CONFIG_DEFAULT (0x0000)
#define ACAMERA_FPGA_FR_DMA_WRITER_BLK_CONFIG_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_BLK_CONFIG_OFFSET (0x188)
#define ACAMERA_FPGA_FR_DMA_WRITER_BLK_CONFIG_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fr_dma_writer_blk_config_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x209188L, data);
}
static __inline uint32_t acamera_fpga_fr_dma_writer_blk_config_read(uint32_t base) {
    return system_gdc_read_32(0x209188L);
}
// ------------------------------------------------------------------------------ //
// Register: blk_status
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// block status output (reserved)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FR_DMA_WRITER_BLK_STATUS_DEFAULT (0x0)
#define ACAMERA_FPGA_FR_DMA_WRITER_BLK_STATUS_DATASIZE (32)
#define ACAMERA_FPGA_FR_DMA_WRITER_BLK_STATUS_OFFSET (0x18c)
#define ACAMERA_FPGA_FR_DMA_WRITER_BLK_STATUS_MASK (0xffffffff)

// args: data (32-bit)
static __inline uint32_t acamera_fpga_fr_dma_writer_blk_status_read(uint32_t base) {
    return system_gdc_read_32(0x20918cL);
}
// ------------------------------------------------------------------------------ //
// Group: Video DMA Writer FR - UV
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Full resolution video DMA writer controls
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: Format
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Format
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_FORMAT_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FORMAT_DATASIZE (8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FORMAT_OFFSET (0x190)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FORMAT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fruv_dma_writer_format_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209190L);
    system_gdc_write_32(0x209190L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_format_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209190L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Base mode
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base DMA packing mode for RGB/RAW/YUV etc (see ISP guide)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_BASE_MODE_DEFAULT (0x0D)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BASE_MODE_DATASIZE (4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BASE_MODE_OFFSET (0x190)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BASE_MODE_MASK (0xf)

// args: data (4-bit)
static __inline void acamera_fpga_fruv_dma_writer_base_mode_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209190L);
    system_gdc_write_32(0x209190L, (((uint32_t) (data & 0xf)) << 0) | (curr & 0xfffffff0));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_base_mode_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209190L) & 0xf) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Plane select
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Plane select for planar base modes.  Only used if planar outputs required.  Not used.  Should be set to 0
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_PLANE_SELECT_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_PLANE_SELECT_DATASIZE (2)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_PLANE_SELECT_OFFSET (0x190)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_PLANE_SELECT_MASK (0xc0)

// args: data (2-bit)
static __inline void acamera_fpga_fruv_dma_writer_plane_select_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209190L);
    system_gdc_write_32(0x209190L, (((uint32_t) (data & 0x3)) << 6) | (curr & 0xffffff3f));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_plane_select_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209190L) & 0xc0) >> 6);
}
// ------------------------------------------------------------------------------ //
// Register: single frame
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = All frames are written(after frame_write_on= 1), 1= only 1st frame written ( after frame_write_on =1)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_SINGLE_FRAME_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_SINGLE_FRAME_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_SINGLE_FRAME_OFFSET (0x194)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_SINGLE_FRAME_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_single_frame_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209194L);
    system_gdc_write_32(0x209194L, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_single_frame_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209194L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: frame write on
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 
//    0 = no frames written(when switched from 1, current frame completes writing before stopping),
//    1= write frame(s) (write single or continous frame(s) )
//    
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WRITE_ON_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WRITE_ON_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WRITE_ON_OFFSET (0x194)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WRITE_ON_MASK (0x2)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_frame_write_on_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209194L);
    system_gdc_write_32(0x209194L, (((uint32_t) (data & 0x1)) << 1) | (curr & 0xfffffffd));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_frame_write_on_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209194L) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: half irate
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = normal operation , 1= write half(alternate) of input frames( only valid for continuous mode)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_HALF_IRATE_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_HALF_IRATE_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_HALF_IRATE_OFFSET (0x194)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_HALF_IRATE_MASK (0x4)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_half_irate_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209194L);
    system_gdc_write_32(0x209194L, (((uint32_t) (data & 0x1)) << 2) | (curr & 0xfffffffb));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_half_irate_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209194L) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: axi xact comp
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = dont wait for axi transaction completion at end of frame(just all transfers accepted). 1 = wait for all transactions completed
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_XACT_COMP_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_XACT_COMP_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_XACT_COMP_OFFSET (0x194)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_XACT_COMP_MASK (0x8)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_axi_xact_comp_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209194L);
    system_gdc_write_32(0x209194L, (((uint32_t) (data & 0x1)) << 3) | (curr & 0xfffffff7));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_axi_xact_comp_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209194L) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: active width
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Active video width in pixels 128-8000
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_ACTIVE_WIDTH_DEFAULT (0x780)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_ACTIVE_WIDTH_DATASIZE (16)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_ACTIVE_WIDTH_OFFSET (0x198)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_ACTIVE_WIDTH_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_fruv_dma_writer_active_width_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209198L);
    system_gdc_write_32(0x209198L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_fruv_dma_writer_active_width_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209198L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: active height
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Active video height in lines 128-8000
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_ACTIVE_HEIGHT_DEFAULT (0x438)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_ACTIVE_HEIGHT_DATASIZE (16)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_ACTIVE_HEIGHT_OFFSET (0x19c)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_ACTIVE_HEIGHT_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_fruv_dma_writer_active_height_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x20919cL);
    system_gdc_write_32(0x20919cL, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_fruv_dma_writer_active_height_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x20919cL) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: bank0_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 0 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK0_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK0_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK0_BASE_OFFSET (0x1a0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK0_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fruv_dma_writer_bank0_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x2091a0L, data);
}
static __inline uint32_t acamera_fpga_fruv_dma_writer_bank0_base_read(uint32_t base) {
    return system_gdc_read_32(0x2091a0L);
}
// ------------------------------------------------------------------------------ //
// Register: bank1_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 1 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK1_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK1_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK1_BASE_OFFSET (0x1a4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK1_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fruv_dma_writer_bank1_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x2091a4L, data);
}
static __inline uint32_t acamera_fpga_fruv_dma_writer_bank1_base_read(uint32_t base) {
    return system_gdc_read_32(0x2091a4L);
}
// ------------------------------------------------------------------------------ //
// Register: bank2_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 2 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK2_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK2_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK2_BASE_OFFSET (0x1a8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK2_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fruv_dma_writer_bank2_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x2091a8L, data);
}
static __inline uint32_t acamera_fpga_fruv_dma_writer_bank2_base_read(uint32_t base) {
    return system_gdc_read_32(0x2091a8L);
}
// ------------------------------------------------------------------------------ //
// Register: bank3_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 3 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK3_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK3_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK3_BASE_OFFSET (0x1ac)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK3_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fruv_dma_writer_bank3_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x2091acL, data);
}
static __inline uint32_t acamera_fpga_fruv_dma_writer_bank3_base_read(uint32_t base) {
    return system_gdc_read_32(0x2091acL);
}
// ------------------------------------------------------------------------------ //
// Register: bank4_base
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bank 4 base address for frame buffer, should be word-aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK4_BASE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK4_BASE_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK4_BASE_OFFSET (0x1b0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK4_BASE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fruv_dma_writer_bank4_base_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x2091b0L, data);
}
static __inline uint32_t acamera_fpga_fruv_dma_writer_bank4_base_read(uint32_t base) {
    return system_gdc_read_32(0x2091b0L);
}
// ------------------------------------------------------------------------------ //
// Register: max bank
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// highest bank*_base to use for frame writes before recycling to bank0_base, only 0 to 4 are valid
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BANK_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BANK_DATASIZE (3)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BANK_OFFSET (0x1b4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BANK_MASK (0x7)

// args: data (3-bit)
static __inline void acamera_fpga_fruv_dma_writer_max_bank_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091b4L);
    system_gdc_write_32(0x2091b4L, (((uint32_t) (data & 0x7)) << 0) | (curr & 0xfffffff8));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_max_bank_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091b4L) & 0x7) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: bank0 restart
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = normal operation, 1= restart bank counter to bank0 for next frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK0_RESTART_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK0_RESTART_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK0_RESTART_OFFSET (0x1b4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BANK0_RESTART_MASK (0x8)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_bank0_restart_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091b4L);
    system_gdc_write_32(0x2091b4L, (((uint32_t) (data & 0x1)) << 3) | (curr & 0xfffffff7));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_bank0_restart_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091b4L) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: Line_offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 
//    Indicates the offset in bytes from the start of one line to the next line.  
//    This value should be equal to or larger than one line of image data and should be word-aligned
//    
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_LINE_OFFSET_DEFAULT (0x4000)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_LINE_OFFSET_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_LINE_OFFSET_OFFSET (0x1b8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fruv_dma_writer_line_offset_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x2091b8L, data);
}
static __inline uint32_t acamera_fpga_fruv_dma_writer_line_offset_read(uint32_t base) {
    return system_gdc_read_32(0x2091b8L);
}
// ------------------------------------------------------------------------------ //
// Register: frame write cancel
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0 = normal operation, 1= cancel current/future frame write(s), any unstarted AXI bursts cancelled and fifo flushed
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WRITE_CANCEL_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WRITE_CANCEL_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WRITE_CANCEL_OFFSET (0x1bc)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WRITE_CANCEL_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_frame_write_cancel_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091bcL);
    system_gdc_write_32(0x2091bcL, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_frame_write_cancel_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091bcL) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: axi_port_enable
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// enables axi, active high, 1=enables axi write transfers, 0= reset axi domain( via reset synchroniser)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_PORT_ENABLE_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_PORT_ENABLE_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_PORT_ENABLE_OFFSET (0x1bc)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_PORT_ENABLE_MASK (0x2)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_axi_port_enable_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091bcL);
    system_gdc_write_32(0x2091bcL, (((uint32_t) (data & 0x1)) << 1) | (curr & 0xfffffffd));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_axi_port_enable_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091bcL) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: wbank curr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// write bank currently active. valid values =0-4. updated at start of frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_CURR_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_CURR_DATASIZE (3)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_CURR_OFFSET (0x1c0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_CURR_MASK (0x7)

// args: data (3-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_wbank_curr_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091c0L) & 0x7) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: wbank last
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// write bank last active. valid values = 0-4. updated at start of frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_LAST_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_LAST_DATASIZE (3)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_LAST_OFFSET (0x1c0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_LAST_MASK (0x38)

// args: data (3-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_wbank_last_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091c0L) & 0x38) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: wbank active
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1 = wbank_curr is being written to. Goes high at start of writes, low at last write transfer/completion on axi. 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_ACTIVE_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_ACTIVE_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_ACTIVE_OFFSET (0x1c4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_ACTIVE_MASK (0x1)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_wbank_active_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091c4L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: wbank start
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1 = High pulse at start of frame write to bank. 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_START_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_START_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_START_OFFSET (0x1c4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_START_MASK (0x2)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_wbank_start_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091c4L) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: wbank stop
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1 = High pulse at end of frame write to bank. 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_STOP_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_STOP_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_STOP_OFFSET (0x1c4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBANK_STOP_MASK (0x4)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_wbank_stop_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091c4L) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: wbase curr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// currently active bank base addr - in bytes. updated at start of frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBASE_CURR_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBASE_CURR_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBASE_CURR_OFFSET (0x1c8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBASE_CURR_MASK (0xffffffff)

// args: data (32-bit)
static __inline uint32_t acamera_fpga_fruv_dma_writer_wbase_curr_read(uint32_t base) {
    return system_gdc_read_32(0x2091c8L);
}
// ------------------------------------------------------------------------------ //
// Register: wbase last
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// last active bank base addr - in bytes. Updated at start of frame write
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBASE_LAST_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBASE_LAST_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBASE_LAST_OFFSET (0x1cc)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WBASE_LAST_MASK (0xffffffff)

// args: data (32-bit)
static __inline uint32_t acamera_fpga_fruv_dma_writer_wbase_last_read(uint32_t base) {
    return system_gdc_read_32(0x2091ccL);
}
// ------------------------------------------------------------------------------ //
// Register: frame icount
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// count of incomming frames (starts) to vdma_writer on video input, non resetable, rolls over, updates at pixel 1 of new frame on video in
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_ICOUNT_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_ICOUNT_DATASIZE (16)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_ICOUNT_OFFSET (0x1d0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_ICOUNT_MASK (0xffff)

// args: data (16-bit)
static __inline uint16_t acamera_fpga_fruv_dma_writer_frame_icount_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x2091d0L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: frame wcount
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// count of outgoing frame writes (starts) from vdma_writer sent to AXI output, non resetable, rolls over, updates at pixel 1 of new frame on video in
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WCOUNT_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WCOUNT_DATASIZE (16)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WCOUNT_OFFSET (0x1d4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_FRAME_WCOUNT_MASK (0xffff)

// args: data (16-bit)
static __inline uint16_t acamera_fpga_fruv_dma_writer_frame_wcount_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x2091d4L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: clear alarms
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0>1 transition(synchronous detection) causes local axi/video alarm clear
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_CLEAR_ALARMS_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_CLEAR_ALARMS_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_CLEAR_ALARMS_OFFSET (0x1d8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_CLEAR_ALARMS_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_clear_alarms_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091d8L);
    system_gdc_write_32(0x2091d8L, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_clear_alarms_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091d8L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: max_burst_length_is_8
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1= Reduce default AXI max_burst_length from 16 to 8, 0= Dont reduce
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BURST_LENGTH_IS_8_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BURST_LENGTH_IS_8_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BURST_LENGTH_IS_8_OFFSET (0x1d8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BURST_LENGTH_IS_8_MASK (0x2)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_max_burst_length_is_8_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091d8L);
    system_gdc_write_32(0x2091d8L, (((uint32_t) (data & 0x1)) << 1) | (curr & 0xfffffffd));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_max_burst_length_is_8_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091d8L) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: max_burst_length_is_4
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 1= Reduce default AXI max_burst_length from 16 to 4, 0= Dont reduce( has priority overmax_burst_length_is_8!)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BURST_LENGTH_IS_4_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BURST_LENGTH_IS_4_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BURST_LENGTH_IS_4_OFFSET (0x1d8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_MAX_BURST_LENGTH_IS_4_MASK (0x4)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_max_burst_length_is_4_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091d8L);
    system_gdc_write_32(0x2091d8L, (((uint32_t) (data & 0x1)) << 2) | (curr & 0xfffffffb));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_max_burst_length_is_4_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091d8L) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: write timeout disable
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 
//    At end of frame an optional timeout is applied to wait for AXI writes to completed/accepted befotre caneclling and flushing.
//    0= Timeout Enabled, timeout count can decrement.
//    1 = Disable timeout, timeout count can't decrement. 
//    
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WRITE_TIMEOUT_DISABLE_DEFAULT (0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WRITE_TIMEOUT_DISABLE_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WRITE_TIMEOUT_DISABLE_OFFSET (0x1d8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WRITE_TIMEOUT_DISABLE_MASK (0x8)

// args: data (1-bit)
static __inline void acamera_fpga_fruv_dma_writer_write_timeout_disable_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091d8L);
    system_gdc_write_32(0x2091d8L, (((uint32_t) (data & 0x1)) << 3) | (curr & 0xfffffff7));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_write_timeout_disable_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091d8L) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: awmaxwait_limit
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// awvalid maxwait limit(cycles) to raise axi_fail_awmaxwait alarm . zero disables alarm raise.
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_AWMAXWAIT_LIMIT_DEFAULT (0x00)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AWMAXWAIT_LIMIT_DATASIZE (8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AWMAXWAIT_LIMIT_OFFSET (0x1dc)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AWMAXWAIT_LIMIT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fruv_dma_writer_awmaxwait_limit_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091dcL);
    system_gdc_write_32(0x2091dcL, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_awmaxwait_limit_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091dcL) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: wmaxwait_limit
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// wvalid maxwait limit(cycles) to raise axi_fail_wmaxwait alarm . zero disables alarm raise
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WMAXWAIT_LIMIT_DEFAULT (0x00)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WMAXWAIT_LIMIT_DATASIZE (8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WMAXWAIT_LIMIT_OFFSET (0x1e0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WMAXWAIT_LIMIT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fruv_dma_writer_wmaxwait_limit_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091e0L);
    system_gdc_write_32(0x2091e0L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_wmaxwait_limit_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e0L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: wxact_ostand_limit
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// number oustsanding write transactions(bursts)(responses..1 per burst) limit to raise axi_fail_wxact_ostand. zero disables alarm raise
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_WXACT_OSTAND_LIMIT_DEFAULT (0x00)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WXACT_OSTAND_LIMIT_DATASIZE (8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WXACT_OSTAND_LIMIT_OFFSET (0x1e4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_WXACT_OSTAND_LIMIT_MASK (0xff)

// args: data (8-bit)
static __inline void acamera_fpga_fruv_dma_writer_wxact_ostand_limit_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x2091e4L);
    system_gdc_write_32(0x2091e4L, (((uint32_t) (data & 0xff)) << 0) | (curr & 0xffffff00));
}
static __inline uint8_t acamera_fpga_fruv_dma_writer_wxact_ostand_limit_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e4L) & 0xff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: axi_fail_bresp
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate bad  bresp captured 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_BRESP_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_BRESP_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_BRESP_OFFSET (0x1e8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_BRESP_MASK (0x1)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_axi_fail_bresp_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e8L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: axi_fail_awmaxwait
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high when awmaxwait_limit reached 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_AWMAXWAIT_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_AWMAXWAIT_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_AWMAXWAIT_OFFSET (0x1e8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_AWMAXWAIT_MASK (0x2)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_axi_fail_awmaxwait_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e8L) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: axi_fail_wmaxwait
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high when wmaxwait_limit reached 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_WMAXWAIT_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_WMAXWAIT_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_WMAXWAIT_OFFSET (0x1e8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_WMAXWAIT_MASK (0x4)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_axi_fail_wmaxwait_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e8L) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: axi_fail_wxact_ostand
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high when wxact_ostand_limit reached 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_WXACT_OSTAND_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_WXACT_OSTAND_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_WXACT_OSTAND_OFFSET (0x1e8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_FAIL_WXACT_OSTAND_MASK (0x8)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_axi_fail_wxact_ostand_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e8L) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: vi_fail_active_width
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate mismatched active_width detected 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_ACTIVE_WIDTH_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_ACTIVE_WIDTH_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_ACTIVE_WIDTH_OFFSET (0x1e8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_ACTIVE_WIDTH_MASK (0x10)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_vi_fail_active_width_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e8L) & 0x10) >> 4);
}
// ------------------------------------------------------------------------------ //
// Register: vi_fail_active_height
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate mismatched active_height detected ( also raised on missing field!) 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_ACTIVE_HEIGHT_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_ACTIVE_HEIGHT_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_ACTIVE_HEIGHT_OFFSET (0x1e8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_ACTIVE_HEIGHT_MASK (0x20)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_vi_fail_active_height_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e8L) & 0x20) >> 5);
}
// ------------------------------------------------------------------------------ //
// Register: vi_fail_interline_blanks
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate interline blanking below min 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_INTERLINE_BLANKS_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_INTERLINE_BLANKS_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_INTERLINE_BLANKS_OFFSET (0x1e8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_INTERLINE_BLANKS_MASK (0x40)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_vi_fail_interline_blanks_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e8L) & 0x40) >> 6);
}
// ------------------------------------------------------------------------------ //
// Register: vi_fail_interframe_blanks
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  clearable alarm, high to indicate interframe blanking below min 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_INTERFRAME_BLANKS_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_INTERFRAME_BLANKS_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_INTERFRAME_BLANKS_OFFSET (0x1e8)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VI_FAIL_INTERFRAME_BLANKS_MASK (0x80)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_vi_fail_interframe_blanks_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091e8L) & 0x80) >> 7);
}
// ------------------------------------------------------------------------------ //
// Register: video_alarm
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  active high, problem found on video port(s) ( active width/height or interline/frame blanks failure) 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_VIDEO_ALARM_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VIDEO_ALARM_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VIDEO_ALARM_OFFSET (0x1ec)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_VIDEO_ALARM_MASK (0x1)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_video_alarm_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091ecL) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: axi_alarm
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
//  active high, problem found on axi port(s)( bresp or awmaxwait or wmaxwait or wxact_ostand failure ) 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_ALARM_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_ALARM_DATASIZE (1)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_ALARM_OFFSET (0x1ec)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_AXI_ALARM_MASK (0x2)

// args: data (1-bit)
static __inline uint8_t acamera_fpga_fruv_dma_writer_axi_alarm_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x2091ecL) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: blk_config
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// block configuration (reserved)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_BLK_CONFIG_DEFAULT (0x0000)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BLK_CONFIG_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BLK_CONFIG_OFFSET (0x1f0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BLK_CONFIG_MASK (0xffffffff)

// args: data (32-bit)
static __inline void acamera_fpga_fruv_dma_writer_blk_config_write(uint32_t base, uint32_t data) {
    system_gdc_write_32(0x2091f0L, data);
}
static __inline uint32_t acamera_fpga_fruv_dma_writer_blk_config_read(uint32_t base) {
    return system_gdc_read_32(0x2091f0L);
}
// ------------------------------------------------------------------------------ //
// Register: blk_status
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// block status output (reserved)
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_FRUV_DMA_WRITER_BLK_STATUS_DEFAULT (0x0)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BLK_STATUS_DATASIZE (32)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BLK_STATUS_OFFSET (0x1f4)
#define ACAMERA_FPGA_FRUV_DMA_WRITER_BLK_STATUS_MASK (0xffffffff)

// args: data (32-bit)
static __inline uint32_t acamera_fpga_fruv_dma_writer_blk_status_read(uint32_t base) {
    return system_gdc_read_32(0x2091f4L);
}
// ------------------------------------------------------------------------------ //
// Group: CS YR
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Conversion of RGB to YUV data using a 3x3 color matrix plus offsets
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: Enable matrix
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Color matrix enable: 0=off 1=on
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_MATRIX_DEFAULT (1)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_MATRIX_DATASIZE (1)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_MATRIX_OFFSET (0x230)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_MATRIX_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_yr_cs_conv_enable_matrix_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209230L);
    system_gdc_write_32(0x209230L, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_yr_cs_conv_enable_matrix_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209230L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Enable Horizontal downsample
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Downsample enable: 0=off 1=on
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_HORIZONTAL_DOWNSAMPLE_DEFAULT (0)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_HORIZONTAL_DOWNSAMPLE_DATASIZE (1)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_HORIZONTAL_DOWNSAMPLE_OFFSET (0x230)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_HORIZONTAL_DOWNSAMPLE_MASK (0x2)

// args: data (1-bit)
static __inline void acamera_fpga_yr_cs_conv_enable_horizontal_downsample_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209230L);
    system_gdc_write_32(0x209230L, (((uint32_t) (data & 0x1)) << 1) | (curr & 0xfffffffd));
}
static __inline uint8_t acamera_fpga_yr_cs_conv_enable_horizontal_downsample_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209230L) & 0x2) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: Enable Vertical downsample
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Downsample enable: 0=off 1=on
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_VERTICAL_DOWNSAMPLE_DEFAULT (0)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_VERTICAL_DOWNSAMPLE_DATASIZE (1)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_VERTICAL_DOWNSAMPLE_OFFSET (0x230)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_VERTICAL_DOWNSAMPLE_MASK (0x4)

// args: data (1-bit)
static __inline void acamera_fpga_yr_cs_conv_enable_vertical_downsample_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209230L);
    system_gdc_write_32(0x209230L, (((uint32_t) (data & 0x1)) << 2) | (curr & 0xfffffffb));
}
static __inline uint8_t acamera_fpga_yr_cs_conv_enable_vertical_downsample_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209230L) & 0x4) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: Enable filter
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Downsample enable: 0=off 1=on
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_FILTER_DEFAULT (0)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_FILTER_DATASIZE (1)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_FILTER_OFFSET (0x230)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_FILTER_MASK (0x8)

// args: data (1-bit)
static __inline void acamera_fpga_yr_cs_conv_enable_filter_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209230L);
    system_gdc_write_32(0x209230L, (((uint32_t) (data & 0x1)) << 3) | (curr & 0xfffffff7));
}
static __inline uint8_t acamera_fpga_yr_cs_conv_enable_filter_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209230L) & 0x8) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft 11
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Matrix coefficient for R-Y multiplier
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_11_DEFAULT (0x012A)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_11_DATASIZE (16)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_11_OFFSET (0x200)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_11_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_11_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209200L);
    system_gdc_write_32(0x209200L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_11_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209200L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft 12
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Matrix coefficient for G-Y multiplier
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_12_DEFAULT (0x0000)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_12_DATASIZE (16)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_12_OFFSET (0x204)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_12_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_12_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209204L);
    system_gdc_write_32(0x209204L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_12_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209204L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft 13
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Matrix coefficient for B-Y multiplier
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_13_DEFAULT (0x01D2)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_13_DATASIZE (16)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_13_OFFSET (0x208)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_13_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_13_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209208L);
    system_gdc_write_32(0x209208L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_13_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209208L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft 21
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Matrix coefficient for R-Cb multiplier
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_21_DEFAULT (0x012A)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_21_DATASIZE (16)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_21_OFFSET (0x20c)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_21_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_21_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x20920cL);
    system_gdc_write_32(0x20920cL, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_21_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x20920cL) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft 22
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Matrix coefficient for G-Cb multiplier
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_22_DEFAULT (0x0000)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_22_DATASIZE (16)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_22_OFFSET (0x210)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_22_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_22_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209210L);
    system_gdc_write_32(0x209210L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_22_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209210L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft 23
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Matrix coefficient for B-Cb multiplier
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_23_DEFAULT (0x8088)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_23_DATASIZE (16)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_23_OFFSET (0x214)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_23_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_23_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209214L);
    system_gdc_write_32(0x209214L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_23_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209214L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft 31
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Matrix coefficient for R-Cr multiplier
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_31_DEFAULT (0x012A)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_31_DATASIZE (16)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_31_OFFSET (0x218)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_31_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_31_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209218L);
    system_gdc_write_32(0x209218L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_31_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209218L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft 32
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Matrix coefficient for G-Cr multiplier
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_32_DEFAULT (0x021D)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_32_DATASIZE (16)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_32_OFFSET (0x21c)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_32_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_32_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x20921cL);
    system_gdc_write_32(0x20921cL, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_32_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x20921cL) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft 33
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Matrix coefficient for B-Cr multiplier
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_33_DEFAULT (0x0000)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_33_DATASIZE (16)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_33_OFFSET (0x220)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_33_MASK (0xffff)

// args: data (16-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_33_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209220L);
    system_gdc_write_32(0x209220L, (((uint32_t) (data & 0xffff)) << 0) | (curr & 0xffff0000));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_33_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209220L) & 0xffff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft o1
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Offset for Y
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O1_DEFAULT (0x420)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O1_DATASIZE (11)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O1_OFFSET (0x224)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O1_MASK (0x7ff)

// args: data (11-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_o1_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209224L);
    system_gdc_write_32(0x209224L, (((uint32_t) (data & 0x7ff)) << 0) | (curr & 0xfffff800));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_o1_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209224L) & 0x7ff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft o2
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Offset for Cb
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O2_DEFAULT (0x134)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O2_DATASIZE (11)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O2_OFFSET (0x228)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O2_MASK (0x7ff)

// args: data (11-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_o2_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209228L);
    system_gdc_write_32(0x209228L, (((uint32_t) (data & 0x7ff)) << 0) | (curr & 0xfffff800));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_o2_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209228L) & 0x7ff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Coefft o3
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Offset for Cr
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O3_DEFAULT (0x401)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O3_DATASIZE (11)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O3_OFFSET (0x22c)
#define ACAMERA_FPGA_YR_CS_CONV_COEFFT_O3_MASK (0x7ff)

// args: data (11-bit)
static __inline void acamera_fpga_yr_cs_conv_coefft_o3_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x20922cL);
    system_gdc_write_32(0x20922cL, (((uint32_t) (data & 0x7ff)) << 0) | (curr & 0xfffff800));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_coefft_o3_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x20922cL) & 0x7ff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Clip min Y
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Minimal value for Y.  Values below this are clipped.
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MIN_Y_DEFAULT (0x000)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MIN_Y_DATASIZE (10)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MIN_Y_OFFSET (0x238)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MIN_Y_MASK (0x3ff)

// args: data (10-bit)
static __inline void acamera_fpga_yr_cs_conv_clip_min_y_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209238L);
    system_gdc_write_32(0x209238L, (((uint32_t) (data & 0x3ff)) << 0) | (curr & 0xfffffc00));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_clip_min_y_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209238L) & 0x3ff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Clip max Y
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Maximal value for Y.  Values above this are clipped.
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MAX_Y_DEFAULT (0x3FF)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MAX_Y_DATASIZE (10)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MAX_Y_OFFSET (0x238)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MAX_Y_MASK (0x3ff0000)

// args: data (10-bit)
static __inline void acamera_fpga_yr_cs_conv_clip_max_y_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209238L);
    system_gdc_write_32(0x209238L, (((uint32_t) (data & 0x3ff)) << 16) | (curr & 0xfc00ffff));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_clip_max_y_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209238L) & 0x3ff0000) >> 16);
}
// ------------------------------------------------------------------------------ //
// Register: Clip min UV
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Minimal value for Cb, Cr.  Values below this are clipped.
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MIN_UV_DEFAULT (0x000)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MIN_UV_DATASIZE (10)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MIN_UV_OFFSET (0x23c)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MIN_UV_MASK (0x3ff)

// args: data (10-bit)
static __inline void acamera_fpga_yr_cs_conv_clip_min_uv_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x20923cL);
    system_gdc_write_32(0x20923cL, (((uint32_t) (data & 0x3ff)) << 0) | (curr & 0xfffffc00));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_clip_min_uv_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x20923cL) & 0x3ff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Clip max UV
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Maximal value for Cb, Cr.  Values above this are clipped.
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MAX_UV_DEFAULT (0x3FF)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MAX_UV_DATASIZE (10)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MAX_UV_OFFSET (0x23c)
#define ACAMERA_FPGA_YR_CS_CONV_CLIP_MAX_UV_MASK (0x3ff0000)

// args: data (10-bit)
static __inline void acamera_fpga_yr_cs_conv_clip_max_uv_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x20923cL);
    system_gdc_write_32(0x20923cL, (((uint32_t) (data & 0x3ff)) << 16) | (curr & 0xfc00ffff));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_clip_max_uv_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x20923cL) & 0x3ff0000) >> 16);
}
// ------------------------------------------------------------------------------ //
// Register: Enable dither
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Enables dithering module
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_DITHER_DEFAULT (0x0)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_DITHER_DATASIZE (1)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_DITHER_OFFSET (0x244)
#define ACAMERA_FPGA_YR_CS_CONV_ENABLE_DITHER_MASK (0x1)

// args: data (1-bit)
static __inline void acamera_fpga_yr_cs_conv_enable_dither_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209244L);
    system_gdc_write_32(0x209244L, (((uint32_t) (data & 0x1)) << 0) | (curr & 0xfffffffe));
}
static __inline uint8_t acamera_fpga_yr_cs_conv_enable_dither_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209244L) & 0x1) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Dither amount
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0= dither to 9 bits; 1=dither to 8 bits; 2=dither to 7 bits; 3=dither to 6 bits 
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_DITHER_AMOUNT_DEFAULT (0x0)
#define ACAMERA_FPGA_YR_CS_CONV_DITHER_AMOUNT_DATASIZE (2)
#define ACAMERA_FPGA_YR_CS_CONV_DITHER_AMOUNT_OFFSET (0x244)
#define ACAMERA_FPGA_YR_CS_CONV_DITHER_AMOUNT_MASK (0x6)

// args: data (2-bit)
static __inline void acamera_fpga_yr_cs_conv_dither_amount_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209244L);
    system_gdc_write_32(0x209244L, (((uint32_t) (data & 0x3)) << 1) | (curr & 0xfffffff9));
}
static __inline uint8_t acamera_fpga_yr_cs_conv_dither_amount_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209244L) & 0x6) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: Shift mode
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 0= output is LSB aligned; 1=output is MSB aligned
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_SHIFT_MODE_DEFAULT (0x0)
#define ACAMERA_FPGA_YR_CS_CONV_SHIFT_MODE_DATASIZE (1)
#define ACAMERA_FPGA_YR_CS_CONV_SHIFT_MODE_OFFSET (0x244)
#define ACAMERA_FPGA_YR_CS_CONV_SHIFT_MODE_MASK (0x10)

// args: data (1-bit)
static __inline void acamera_fpga_yr_cs_conv_shift_mode_write(uint32_t base, uint8_t data) {
    uint32_t curr = system_gdc_read_32(0x209244L);
    system_gdc_write_32(0x209244L, (((uint32_t) (data & 0x1)) << 4) | (curr & 0xffffffef));
}
static __inline uint8_t acamera_fpga_yr_cs_conv_shift_mode_read(uint32_t base) {
    return (uint8_t)((system_gdc_read_32(0x209244L) & 0x10) >> 4);
}
// ------------------------------------------------------------------------------ //
// Register: Data mask RY
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Data mask for channel 1 (R or Y).  Bit-wise and of this value and video data.
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_RY_DEFAULT (0x3FF)
#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_RY_DATASIZE (10)
#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_RY_OFFSET (0x258)
#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_RY_MASK (0x3ff)

// args: data (10-bit)
static __inline void acamera_fpga_yr_cs_conv_data_mask_ry_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209258L);
    system_gdc_write_32(0x209258L, (((uint32_t) (data & 0x3ff)) << 0) | (curr & 0xfffffc00));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_data_mask_ry_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209258L) & 0x3ff) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Data mask GU
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Data mask for channel 2 (G or U).  Bit-wise and of this value and video data.
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_GU_DEFAULT (0x3FF)
#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_GU_DATASIZE (10)
#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_GU_OFFSET (0x258)
#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_GU_MASK (0x3ff0000)

// args: data (10-bit)
static __inline void acamera_fpga_yr_cs_conv_data_mask_gu_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x209258L);
    system_gdc_write_32(0x209258L, (((uint32_t) (data & 0x3ff)) << 16) | (curr & 0xfc00ffff));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_data_mask_gu_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x209258L) & 0x3ff0000) >> 16);
}
// ------------------------------------------------------------------------------ //
// Register: Data mask BV
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Data mask for channel 3 (B or V).  Bit-wise and of this value and video data.
// ------------------------------------------------------------------------------ //

#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_BV_DEFAULT (0x3FF)
#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_BV_DATASIZE (10)
#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_BV_OFFSET (0x25c)
#define ACAMERA_FPGA_YR_CS_CONV_DATA_MASK_BV_MASK (0x3ff)

// args: data (10-bit)
static __inline void acamera_fpga_yr_cs_conv_data_mask_bv_write(uint32_t base, uint16_t data) {
    uint32_t curr = system_gdc_read_32(0x20925cL);
    system_gdc_write_32(0x20925cL, (((uint32_t) (data & 0x3ff)) << 0) | (curr & 0xfffffc00));
}
static __inline uint16_t acamera_fpga_yr_cs_conv_data_mask_bv_read(uint32_t base) {
    return (uint16_t)((system_gdc_read_32(0x20925cL) & 0x3ff) >> 0);
}
// ------------------------------------------------------------------------------ //
#endif //__ACAMERA_FPGA_CONFIG_H__
