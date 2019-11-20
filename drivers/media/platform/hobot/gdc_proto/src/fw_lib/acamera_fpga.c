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


#include "acamera_driver_config.h"

#if HAS_FPGA_WRAPPER
#include "acamera_fpga_config.h"
#include "acamera_fpga.h"

/**
 *   FPGA initialization with resolution and y and uv planar addresses
 *
 *
 *   @return 0 - success
 *           -1 - fail.
 */
int acamera_fpga_init( uint32_t active_width, uint32_t active_height,uint32_t total_input,uint32_t * in_addr,uint32_t * in_lineoffset )
{
    if ( active_width == 0 || active_height == 0 || total_input == 0)
        return -1;

    uint32_t base =0;
    //configure fpga reader from here
    acamera_fpga_frame_reader_format_write( base, 13 );

    //acamera_fpga_frame_reader_line_offset_write( base, y_line_offset );
    acamera_fpga_frame_reader_line_offset_write( base, in_lineoffset[0] );
    acamera_fpga_frame_reader_active_width_write( base, active_width );
    acamera_fpga_frame_reader_active_height_write( base, active_height );
    acamera_fpga_frame_reader_axi_port_enable_write( base, 1 );

    //configure fpga writers from here
    acamera_fpga_fr_dma_writer_base_mode_write( base, 13 );
    acamera_fpga_fr_dma_writer_plane_select_write( base, 0 );

    acamera_fpga_fr_dma_writer_active_width_write( base, active_width );
    acamera_fpga_fr_dma_writer_active_height_write( base, active_height );

    //acamera_fpga_fr_dma_writer_bank0_base_write( base, y_base_addr );
    acamera_fpga_fr_dma_writer_bank0_base_write( base, in_addr[0] );
    //acamera_fpga_fr_dma_writer_line_offset_write( base, y_line_offset );
    acamera_fpga_fr_dma_writer_line_offset_write( base, in_lineoffset[0] );
    acamera_fpga_fr_dma_writer_max_bank_write(base,0);

    if(total_input>=2){
        acamera_fpga_frame_reader_uv_format_write( base, 77 );
        acamera_fpga_frame_reader_uv_repeat_downsampled_lines_write( base, 1 );
        acamera_fpga_frame_reader_uv_repeat_downsampled_pixels_write( base, 1 );

        //acamera_fpga_frame_reader_uv_line_offset_write( base, uv_line_offset );
        acamera_fpga_frame_reader_uv_line_offset_write( base, in_lineoffset[1] );
        acamera_fpga_frame_reader_uv_axi_port_enable_write( base, 1 );
        acamera_fpga_frame_reader_uv_active_width_write( base, active_width );
        acamera_fpga_frame_reader_uv_active_height_write( base, active_height );


        acamera_fpga_fruv_dma_writer_base_mode_write( base, 13 );
        acamera_fpga_fruv_dma_writer_plane_select_write( base, 1 );

        acamera_fpga_fruv_dma_writer_active_width_write( base, active_width );
        acamera_fpga_fruv_dma_writer_active_height_write( base, active_height );

        //acamera_fpga_fruv_dma_writer_bank0_base_write( base, uv_base_addr );
        acamera_fpga_fruv_dma_writer_bank0_base_write( base, in_addr[1] );
        //acamera_fpga_fruv_dma_writer_line_offset_write( base, uv_line_offset );
        acamera_fpga_fruv_dma_writer_line_offset_write( base, in_lineoffset[1] );
        acamera_fpga_fruv_dma_writer_max_bank_write(base,0);
    }

    return 0;
}


/**
 *   Update frame reader yuv address and the line offsets
 *
 *
 */
void acamera_fpga_update_frame_reader(  uint32_t total_input, uint32_t * out_addr, uint32_t * out_lineoffset )
{
	uint32_t base=0;

    if(total_input>=1){
        acamera_fpga_frame_reader_rbase_write( base, out_addr[0] );
        acamera_fpga_frame_reader_line_offset_write( base, out_lineoffset[0] );
    }
    if(total_input>=2){
        acamera_fpga_frame_reader_uv_rbase_write( base, out_addr[1] );
        acamera_fpga_frame_reader_uv_line_offset_write( base, out_lineoffset[1] );
    }
    acamera_fpga_frame_reader_rbase_load_write( base, 1 );
    acamera_fpga_frame_reader_uv_rbase_load_write( base, 1 );
    acamera_fpga_frame_reader_rbase_load_write( base, 0 );
    acamera_fpga_frame_reader_uv_rbase_load_write( base, 0 );

}


/**
 *   Get the recent frame writer settings
 *
 *   @return 0 - success
 *           -1 - fail.
 *
 */
int acamera_fpga_get_frame_writer( uint32_t total_input,uint32_t * in_addr,uint32_t * in_lineoffset )
{
    uint32_t base =0;
    if(total_input>=1){
        in_addr[0] = acamera_fpga_fr_dma_writer_wbase_last_read( base );
        in_lineoffset[0] = acamera_fpga_fr_dma_writer_line_offset_read( base );
    }
    if(total_input>=2){
        in_addr[1] = acamera_fpga_fruv_dma_writer_wbase_last_read( base );
        in_lineoffset[1] = acamera_fpga_fruv_dma_writer_line_offset_read( base );
    }

    return 0;
}

#endif

