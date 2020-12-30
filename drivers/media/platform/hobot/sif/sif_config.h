/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_SIF_CONFIG__
#define __HOBOT_SIF_CONFIG__

#include "vio_config.h"

typedef enum _sif_event_e {
	SIF_START  = 1,
	SIF_STOP   = 2,
	SIF_ERROR  = 4,
	SIF_MOTDET = 8,
	/* X2A Extended */
	SIF_OK,
	SIF_POSTPONE,
	SIF_FRAME_DONE,//FIXME: workaround for all buffer own by software, will frame done
} sif_event_t;

typedef enum _sif_fmt_in_e {
	FMT_RAW 		= 0,
	FMT_YUV422_YUYV = 8,
	FMT_YUV422_YVYU,
	FMT_YUV422_UYVY,
	FMT_YUV422_VYUY,
	FMT_MAX = 0x7fffffff,
} sif_fmt_in_t;

typedef enum _sif_pix_len_e {
	PIX_LEN_8,
	PIX_LEN_10,
	PIX_LEN_12,
	PIX_LEN_14,
	PIX_LEN_16,
	PIX_LEN_20,
	PIX_LEN_MAX = 0x7fffffff,
} sif_pix_len_t;

typedef enum _sif_bus_type_e {
	BUS_TYPE_DVP,
	BUS_TYPE_BT1120,
	BUS_TYPE_MIPI,
	BUS_TYPE_XC9080,
	BUS_TYPE_MAX = 0x7fffffff,
} sif_bus_type_t;

typedef struct _frame_id_t {
	uint32_t	enable;
	uint32_t	init_value;
	uint32_t	fix_mode;
} frame_id_t;

typedef struct sif_data_desc {
	uint32_t	format;
	uint32_t	width;
	uint32_t	height;
	uint32_t	pix_length;
} sif_data_desc_t;

typedef struct sif_func_desc {
	uint32_t	enable_mux_out;
	//uint32_t	  enable_mux_reverse;
	uint32_t	enable_pattern;
	uint32_t	enable_frame_id;
	uint32_t	enable_bypass;
	uint32_t	enable_line_shift;
	uint32_t	enable_id_decoder;
	uint32_t	enable_flyby;
	uint32_t	enable_dgain;
	uint32_t	set_init_frame_id;
	uint32_t	set_line_shift_count;
	uint32_t	set_bypass_channels;
	uint32_t	set_mux_out_index;
	//uint32_t	  set_mux_out_reverse;
	uint32_t    short_maxexp_lines;
	uint32_t    medium_maxexp_lines;
	uint32_t 	vc_short_seq; //frame sequence mark in dol mode
	uint32_t 	vc_medium_seq;
	uint32_t 	vc_long_seq;
	uint32_t	set_dgain_short;
	uint32_t	set_dgain_medium;
	uint32_t	set_dgain_long;
} sif_func_desc_t;

typedef struct sif_input_dvp {
	sif_data_desc_t   data;
	sif_func_desc_t   func;
	uint32_t		  enable;
	uint32_t		  vsync_inv;
	uint32_t		  hsync_inv;
	uint32_t		  data_mode;  // for 16/20 bit
} sif_input_dvp_t;

typedef struct sif_input_mipi {
	sif_data_desc_t   data;
	sif_func_desc_t   func;
	uint32_t		  enable;
	uint32_t		  channels;
	uint8_t		  	  vc_index[4];
	uint32_t		  mipi_rx_index;
	uint32_t          ipi_mode;
} sif_input_mipi_t;

//will move to isp input, sif ddr read is just for isp
typedef struct sif_input_ddr {
	uint32_t enable;
	uint32_t stride;
	uint32_t buf_num;
	uint32_t raw_feedback_en;
	sif_data_desc_t   data;
} sif_input_ddr_t;

typedef struct sif_input_bypass {
	uint32_t		  enable_bypass;
	uint32_t		  enable_frame_id;
	uint32_t		  init_frame_id;
	uint32_t	      set_bypass_channels;
} sif_input_bypass_t;

typedef struct sif_input_iar {
	uint32_t		  enable;
	sif_func_desc_t   func;
} sif_input_iar_t;

typedef struct sif_input {
	sif_input_dvp_t 	dvp;
	sif_input_mipi_t	mipi;
	sif_input_iar_t 	iar;
	sif_input_ddr_t     ddr;
} sif_input_t;

typedef enum sif_ddr_out_type_s {
	SIF_DDR_OUT_FOR_ISP = 0,
	SIF_DDR_OUT_FOR_IPU,
	SIF_DDR_OUT_INVALID
}sif_ddr_out_type_e;

typedef struct fps_cfg_s {
	uint32_t skip_frame;  //0:not ctrl; 1: software; 2:hardware
	uint32_t in_fps;
	uint32_t out_fps;
	uint32_t dump_raw;
} fps_cfg_t;

typedef struct sif_output_ddr {
	uint32_t		  enable;
	uint32_t		  mux_index;
	uint32_t		  stride;
	uint32_t		  buffer_num;
	uint32_t		  raw_dump_en;
	sif_ddr_out_type_e sif_ddr_out_type;
	fps_cfg_t		  fps_cfg;
} sif_output_ddr_t;

typedef struct sif_output_isp {
	uint32_t		  enable;
	uint32_t		  dol_exp_num;
	sif_func_desc_t   func;
} sif_output_isp_t;

typedef struct sif_output_ipu {
	uint32_t		  enable_flyby;
} sif_output_ipu_t;

typedef struct sif_output_mipi {
	uint32_t		  enable;
	uint32_t		  channels;
} sif_output_mipi_t;

typedef struct sif_output {
	sif_output_ipu_t	 ipu;
	sif_output_isp_t	 isp;
	sif_output_md_t 	 md;
	sif_output_ddr_t	 ddr;
} sif_output_t;

typedef struct sif_cfg_s {
	sif_input_t 	  input;
	sif_output_t	  output;
} sif_cfg_t;

typedef struct _sif_status_t {
	uint16_t	dvp_width;
	uint16_t	dvp_height;
	uint16_t	bt_width;
	uint16_t	bt_height;
	uint16_t	mipi_width;
	uint16_t	mipi_height;
	uint16_t	sif_width;
	uint16_t	sif_height;
} sif_status_t;

typedef struct _sif_info_t {
	uint16_t	format;
	uint16_t	pix_len;
	uint16_t	bus_type;
	uint16_t	vsync_inv;
	uint16_t	hsync_inv;
	uint16_t	pclk_in_inv;
	uint16_t	pclk_out_inv;
	uint16_t	sif_enable;
	uint16_t	drop_frame;
	uint16_t	dvp2ap_enable;
	uint16_t	bt2ap_enable;
	uint16_t	mipi2ap_enable;
	uint16_t	raw_16bit_mode;
	uint16_t	raw_20bit_mode;
	uint16_t	yuv_10bit_mode;
	uint16_t	xc9080_mode;
	uint16_t	mipi2ap_sel;
	uint16_t	bt_detect_mode;
	uint16_t	bt_in_exchange;
	uint16_t	bt_out_exchange;
	uint16_t	width;
	uint16_t	height;
} sif_info_t;

#endif
