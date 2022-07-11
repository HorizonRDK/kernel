/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_OSD_DEV_H__
#define __HOBOT_OSD_DEV_H__

#include <uapi/linux/types.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/types.h>

#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_group_api.h"
#include "../ipu/hobot_dev_ipu.h"
#undef MAX_DEVICE
#include "../pym/hobot_dev_pym.h"
#undef MAX_DEVICE
#include "osd_config.h"
#include "hobot_osd_mem.h"
#include "hobot_osd_process.h"

#define X3_OSD_NAME  "x3-osd"

#define OSD_IOC_MAGIC 'c'

#define OSD_IOC_CREATE_HANDLE           _IOW(OSD_IOC_MAGIC, 0, osd_handle_info_t)
#define OSD_IOC_DESTROY_HANDLE          _IOW(OSD_IOC_MAGIC, 1, int)
#define OSD_IOC_GET_ATTR                _IOWR(OSD_IOC_MAGIC, 2, osd_handle_info_t)
#define OSD_IOC_SET_ATTR                _IOW(OSD_IOC_MAGIC, 3, osd_handle_info_t)
#define OSD_IOC_GET_BUF                 _IOWR(OSD_IOC_MAGIC, 4, osd_buffer_info_t)
#define OSD_IOC_SET_BUF                 _IOW(OSD_IOC_MAGIC, 5, osd_buffer_info_t)
#define OSD_IOC_ATTACH                  _IOW(OSD_IOC_MAGIC, 6, osd_bind_info_t)
#define OSD_IOC_DETACH                  _IOW(OSD_IOC_MAGIC, 7, osd_bind_info_t)
#define OSD_IOC_GET_BIND_ATTR           _IOWR(OSD_IOC_MAGIC, 8, osd_bind_info_t)
#define OSD_IOC_SET_BIND_ATTR           _IOW(OSD_IOC_MAGIC, 9, osd_bind_info_t)
#define OSD_IOC_STA                     _IOWR(OSD_IOC_MAGIC, 10, osd_sta_info_t)
#define OSD_IOC_STA_LEVEL               _IOWR(OSD_IOC_MAGIC, 11, osd_sta_info_t)
#define OSD_IOC_STA_BIN                 _IOWR(OSD_IOC_MAGIC, 12, osd_sta_bin_info_t)
#define OSD_IOC_COLOR_MAP               _IOWR(OSD_IOC_MAGIC, 13, osd_color_map_t)
#define OSD_IOC_PROC_BUF                _IOW(OSD_IOC_MAGIC, 14, osd_proc_buf_info_t)

#define OSD_HANDLE_MAX 256

#define OSD_HW_PROC_NUM 3
#define OSD_COLOR_NUM 15
#define OSD_LEVEL_NUM 4

typedef enum osd_sta_state_e {
    OSD_STA_NULL = 0,
    OSD_STA_REQUEST,
    OSD_STA_PROCESS,
    OSD_STA_DONE,
} osd_sta_state_t;

typedef enum osd_task_state_e {
    OSD_TASK_NULL,
    OSD_TASK_START,
    OSD_TASK_REQUEST_STOP,
    OSD_TASK_STOP,
} osd_task_state_t;

/****sync with hal****/
typedef struct osd_handle_info_s {
    int32_t handle_id;

    osd_size_t size;
    uint32_t fill_color;
    // when enable and a pixel yuv color is key color, it will use origin image
    // 31-24: 0/1  enable/diable
    // 23-16: y value in yuv
    // 15- 8: u value in yuv
    // 7 - 0: v value in yuv
    uint32_t yuv_bg_transparent;   // yuv420 pixel format background transparent

    // ensure if it is attach to pym
    uint32_t attach_pym;
    osd_process_type_e proc_type;
} osd_handle_info_t;

typedef struct osd_buffer_info_s {
    int32_t handle_id;

    // ping-pong index
    int32_t index;
    osd_size_t size;
    uint64_t paddr;
    uint8_t *vaddr;
    osd_process_type_e proc_type;
} osd_buffer_info_t;

typedef struct osd_bind_info_s {
    int32_t instance;
    int32_t chn;
    int32_t handle_id;

    uint8_t show_en;
    uint8_t invert_en;
    // hw process level:0; sw process level :1-3
    uint32_t osd_level;
    // for pym buffer, osd in which layer
    int32_t buf_layer;

    osd_point_t start_point;

    // for polygon
    uint32_t side_num;
    osd_point_t point[OSD_POLYGON_MAX_SIDE];
    uint32_t *polygon_buf;

    // rect, polygon, mosaic attribute will be set when attach
    osd_handle_info_t handle_info;
} osd_bind_info_t;

typedef struct osd_sta_info_s {
    int32_t instance;
    int32_t chn;
    int32_t buf_layer;
    uint8_t sta_level[MAX_OSD_STA_LEVEL_NUM];

    osd_sta_box_t sta_box[MAX_STA_NUM];
} osd_sta_info_t;

typedef struct osd_sta_bin_info_s {
    int32_t instance;
    int32_t chn;

    uint16_t sta_value[MAX_STA_NUM][MAX_STA_BIN_NUM];
} osd_sta_bin_info_t;

typedef struct osd_proc_buf_info_s {
    int32_t handle_id;
    uint8_t invert_en;

    osd_point_t start_point;
    osd_size_t size;

    uint32_t fill_color;
    uint32_t yuv_bg_transparent;   // yuv420 pixel format background transparent

    uint32_t image_width;
    uint32_t image_height;
    uint64_t y_paddr;
    uint32_t y_length;
    uint64_t uv_paddr;
    uint32_t uv_length;

    uint32_t *polygon_buf;
    osd_process_type_e proc_type;
} osd_proc_buf_info_t;
/****sync with hal end****/


typedef struct osd_handle_s {
    struct list_head node;

    osd_handle_info_t info;
    osd_buffer_t buffer;

    atomic_t bind_cnt;
    atomic_t ref_cnt;
    atomic_t need_update;
} osd_handle_t;

typedef struct osd_bind_s {
    struct list_head node;

    osd_bind_info_t bind_info;
    osd_process_info_t proc_info;

    atomic_t need_update;
    atomic_t ref_cnt;
} osd_bind_t;

typedef struct osd_sta_s {
    osd_sta_state_t sta_state;

    uint32_t enable_index;
    int32_t buf_layer;

    uint8_t sta_level[MAX_OSD_STA_LEVEL_NUM];
    osd_sta_box_t sta_box[MAX_STA_NUM];

    volatile uint16_t sta_value[MAX_STA_NUM][MAX_STA_BIN_NUM];

    osd_process_info_t sta_proc[MAX_STA_NUM];

    struct kthread_work work;
} osd_sta_t;


struct osd_video_ctx{
    struct x3_osd_dev *osd_dev;
};

struct osd_subdev {
    int32_t id;

    struct list_head bind_list;
    struct mutex bind_mutex;
    struct mutex sta_mutex;
    osd_sta_t osd_sta;

    struct x3_osd_dev *osd_dev;

    struct vio_osd_info *osd_info;
    struct ipu_osd_cfg *osd_hw_cfg;
    uint32_t osd_hw_limit_y;
    atomic_t osd_hw_need_update;
    atomic_t osd_hw_cnt;

    struct list_head input_frame_list;
    spinlock_t frame_slock;

    struct kthread_work work;
};

struct x3_osd_dev {
    atomic_t open_cnt;

    struct class *class;
    struct cdev cdev;
    dev_t devno;

    struct kthread_work work;
    struct kthread_worker worker;
    struct task_struct *task;
    osd_task_state_t task_state;

    struct ion_client *ion_client;

    struct osd_subdev subdev[VIO_MAX_STREAM][OSD_CHN_MAX];

    struct list_head osd_list;
    struct mutex osd_list_mutex;

    struct mutex            osd_mutex;
};

#endif // __HOBOT_OSD_DEV_H__
