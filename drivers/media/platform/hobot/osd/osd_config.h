/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_OSD_CONFIG_H__
#define __HOBOT_OSD_CONFIG_H__

#include <linux/types.h>

#define OSD_POLYGON_MAX_SIDE 10

typedef struct osd_point_s {
    uint32_t x;
    uint32_t y;
} osd_point_t;

typedef struct osd_size_s {
    uint32_t w;
    uint32_t h;
} osd_size_t;

typedef struct osd_line_s {
    osd_point_t p1;
    osd_point_t p2;
    double k;/*PRQA S ALL*/
    double b;/*PRQA S ALL*/
    uint8_t k_flag;     // 1:parallel with x;  2:parallel with y
} osd_line_t;

typedef struct osd_rect_s {
    osd_point_t point;
    osd_size_t size;
} osd_rect_t;

typedef struct osd_polygon_s {
    // uint8_t show_en;
    // uint8_t invert_en;
    // uint32_t color;
    uint32_t side_num;
    osd_point_t point[OSD_POLYGON_MAX_SIDE];  // point of vertex
    osd_line_t line[OSD_POLYGON_MAX_SIDE];

    // osd_point_t start_point;
    // osd_size_t range;
    osd_point_t *start;  // start and end of every line
    osd_point_t *end;
} osd_polygon_t;

#endif //
