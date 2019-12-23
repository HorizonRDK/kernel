#ifndef __HOBOT_GDC_CONFIG_H__
#define __HOBOT_GDC_CONFIG_H__

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


typedef struct osd_box_s{
 u8  osd_en;
 u8  overlay_mode;
 u16 start_x;
 u16 start_y;
 u16 width;
 u16 height;
 char *colour_map_addr; //colour map buffer addr
} osd_box_t;

//for osd draw, Y info sta
typedef struct osd_sta_box_s{
 u8  sta_en;
 u16 start_x;
 u16 start_y;
 u16 width;
 u16 height;
} osd_sta_box_t;

typedef struct ipu_roi_box_s{
 u16 start_x;
 u16 start_y;
 u16 width;
 u16 height;
} ipu_roi_box_t;

typedef struct ipu_scale_info_s{
 u16 tgt_width;
 u16 tgt_height;
 u16 step_x;
 u16 step_y;
 u8  pre_scale_x;
 u8  pre_scale_y;
} ipu_scale_info_t;

typedef enum ipu_input_type_s {
 IPU_FROM_SIF_YUV422 = 0,
 IPU_FROM_ISP_YUV420,
 IPU_FROM_DDR_YUV420 = 3,
 IPU_INPUT_INVALID
}ipu_input_type_e;

typedef struct ipu_cfg_s{
 ipu_input_type_e  src_sel;  // 0:sif yuv422, 1:ips0 yuv 420, 2:isp1 yuv420, 3:ddr yuv420
 u32 src_Width;
 u32 src_Height;
 u32 src_stride_Y;
 u32 src_stride_UV;
 u32 us_stride_Y;
 u32 us_stride_UV;
 u32 ds_stride_Y[MAX_DS_NUM];
 u32 ds_stride_UV[MAX_DS_NUM];
 u8  us_roi_en;
 u8  ds_roi_en[MAX_DS_NUM];
 ipu_roi_box_t us_roi_info;
 ipu_roi_box_t ds_roi_info[MAX_DS_NUM];
 u8  us_sc_en;
 u8  ds_sc_en[MAX_DS_NUM];
 ipu_scale_info_t  us_sc_info;
 ipu_scale_info_t  ds_sc_info[MAX_DS_NUM];
 u8  us_frame_id_en;
 u8  ds_frame_id_en[MAX_DS_NUM];
 osd_box_t osd_box[MAX_OSD_LAYER][MAX_OSD_NUM];
 osd_sta_box_t sta_box[MAX_OSD_LAYER][MAX_STA_NUM];
 u8  osd_sta_level[MAX_OSD_STA_LEVEL_NUM];
 u16 osd_sta_bin_value[MAX_STA_NUM][MAX_STA_BIN_NUM];
 u8  ds2_to_pym_en;
} ipu_cfg_t;
#endif