/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_OSD_PROCESS_H__
#define __HOBOT_OSD_PROCESS_H__

#include <linux/kthread.h>

#include "osd_config.h"

typedef enum osd_process_type_s {
    OSD_PROC_HW_VGA4 = 0,
    OSD_PROC_VGA4,
    OSD_PROC_NV12,
    OSD_PROC_RECT,
    OSD_PROC_POLYGON,
    OSD_PROC_MOSAIC,
    OSD_PROC_STA,
    OSD_PROC_MAX_TYPE
} osd_process_type_e;

typedef struct osd_process_info_s {
    uint8_t show_en;
    uint8_t invert_en;
    uint32_t osd_level;
    uint32_t fill_color;
    osd_process_type_e proc_type;
    uint32_t width;
    uint32_t height;
    uint32_t start_x;
    uint32_t start_y;
    uint8_t *src_addr;
    uint8_t *src_vga_addr;
    uint64_t src_paddr;
    uint8_t *sta_level;
    uint16_t *sta_bin_value;

    uint32_t frame_id;
    int32_t buffer_index;
    uint32_t image_width;
    uint32_t image_height;
    // for pym buffer, osd in which layer
    uint32_t buf_layer;
    uint8_t *tar_y_addr;
    uint8_t *tar_uv_addr;

    // yuv nv12, for transparent
    uint32_t yuv_bg_transparent;

    // polygon
    uint32_t *polygon_buf;

    struct mutex proc_mutex;
    struct osd_subdev *subdev;
} osd_process_info_t;

int32_t osd_vga4_to_sw(uint32_t *color_map, uint8_t *src_addr,
    uint8_t *tar_addr, uint32_t width, uint32_t height);
int32_t osd_polygon_analyse(osd_polygon_t *polygon, osd_size_t *size);
void osd_process_vga4_workfunc(osd_process_info_t *proc_info);
void osd_process_nv12_workfunc(osd_process_info_t *proc_info);
void osd_process_rect_workfunc(osd_process_info_t *proc_info);
void osd_process_polygon_workfunc(osd_process_info_t *proc_info);
void osd_process_mosaic_workfunc(osd_process_info_t *proc_info);
void osd_process_sta_workfunc(osd_process_info_t *proc_info);
void osd_process_null_workfunc(osd_process_info_t *proc_info);

#endif // __HOBOT_OSD_PROCESS_H__
