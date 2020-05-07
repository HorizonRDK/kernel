/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_IPU_CONFIG_H__
#define __HOBOT_IPU_CONFIG_H__

#define MAX_DS_NUM      5
#define MAX_ADJ_NUM     3

#define MAX_OSD_LAYER   3
#define MAX_OSD_NUM     3
#define MAX_OSD_COLOR_NUM   15
#define MAX_STA_NUM     8
#define MAX_STA_BIN_NUM   4
#define MAX_OSD_STA_LEVEL_NUM   3

#define MIN_OSD_WIDTH   32
#define MIN_OSD_HEIGHT  2

#define MAX_STA_WIDTH   255
#define MAX_STA_HEIGHT  255
#define MIN_STA_WIDTH   2
#define MIN_STA_HEIGHT  2

#define IPU_SRC_MODE_SIF        0
#define IPU_SRC_MODE_IPS_0      1
#define IPU_SRC_MODE_IPS_1      2
#define IPU_SRC_MODE_DDR        3

typedef struct osd_box_s {
	u8 osd_en;
	u8 overlay_mode;
	u16 start_x;
	u16 start_y;
	u16 width;
	u16 height;
} osd_box_t;

typedef struct osd_color_map_s {
	u8 color_map_update;
	u32 color_map[MAX_OSD_COLOR_NUM];	//colour map buffer addr
} osd_color_map_t;

//for osd draw, Y info sta
typedef struct osd_sta_box_s {
	u8 sta_en;
	u16 start_x;
	u16 start_y;
	u16 width;
	u16 height;
} osd_sta_box_t;

typedef struct ipu_roi_box_s {
	u16 start_x;
	u16 start_y;
	u16 width;
	u16 height;
} ipu_roi_box_t;

typedef struct ipu_scale_info_s {
	u16 tgt_width;
	u16 tgt_height;
	u16 step_x;
	u16 step_y;
	u8 pre_scale_x;
	u8 pre_scale_y;
} ipu_scale_info_t;

typedef enum ipu_input_type_s {
	IPU_FROM_SIF_YUV422 = 0,
	IPU_FROM_ISP_YUV420,
	IPU_FROM_DDR_YUV420 = 3,
	IPU_INPUT_INVALID
} ipu_input_type_e;

typedef struct ipu_us_info_s {
	uint32_t us_stride_y;
	uint32_t us_stride_uv;
	uint8_t us_roi_en;
	ipu_roi_box_t us_roi_info;
	uint8_t us_sc_en;
	ipu_scale_info_t us_sc_info;
	uint16_t buf_num;
} ipu_us_info_t;

typedef struct ipu_ds_info_s {
	uint32_t ds_stride_y;
	uint32_t ds_stride_uv;
	uint8_t ds_roi_en;
	ipu_roi_box_t ds_roi_info;
	uint8_t ds_sc_en;
	ipu_scale_info_t ds_sc_info;
	uint16_t buf_num;
} ipu_ds_info_t;

typedef struct ipu_src_ctrl_s {
	ipu_input_type_e source_sel;	// 0:sif yuv422, 1:isp0 yuv 420, 2:isp1 yuv420, 3:ddr yuv420
	uint32_t src_width;
	uint32_t src_height;
	uint32_t src_stride_y;
	uint32_t src_stride_uv;
	uint16_t frame_id;
	uint8_t us_frame_id_en;
	uint8_t ds_frame_id_en[MAX_DS_NUM];
	uint8_t ds2_to_pym_en;
	uint16_t ddr_in_buf_num;
	int timeout;
	int dq_select_timeout;
} ipu_src_ctrl_t;

typedef struct ipu_cfg_s {
	ipu_src_ctrl_t ctrl_info;
	ipu_us_info_t us_info;
	ipu_ds_info_t ds_info[MAX_DS_NUM];
} ipu_cfg_t;
#endif
