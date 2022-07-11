/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_OSD_MEM_H__
#define __HOBOT_OSD_MEM_H__

#include <linux/ion.h>
#include <linux/list.h>
#include <linux/types.h>

#include "osd_config.h"
#include "vio_framemgr.h"

#define OSD_PING_PONG_BUF 2

typedef enum osd_buf_state_e {
    OSD_BUF_NULL,
    OSD_BUF_CREATE,
    OSD_BUF_PROCESS,
    OSD_BUF_USER,
} osd_buf_state_t;

typedef enum osd_pixel_format_e {
    OSD_PIXEL_FORMAT_NULL,
    OSD_PIXEL_FORMAT_VGA4,
    OSD_PIXEL_FORMAT_NV12,
    OSD_PIXEL_FORMAT_SW_VGA4,
    OSD_PIXEL_FORMAT_POLYGON,
} osd_pixel_format_t;

typedef struct osd_one_buffer_s {
    struct list_head node;

    struct ion_handle *ion_handle;

    atomic_t ref_count;
    osd_buf_state_t state;
    osd_pixel_format_t pixel_fmt;
    size_t length;
    int32_t fd;
    uint8_t *vaddr;
    uint64_t paddr;
} osd_one_buffer_t;

typedef struct osd_buffer_s {
    osd_size_t size;

    osd_one_buffer_t buf[OSD_PING_PONG_BUF];
    osd_one_buffer_t vga_buf[OSD_PING_PONG_BUF];
} osd_buffer_t;

void osd_one_buffer_inc(osd_one_buffer_t *one_buffer);
void osd_one_buffer_inc_by_addr(uint8_t *vaddr);
void osd_one_buffer_inc_by_paddr(uint64_t paddr);
void osd_one_buffer_dec(osd_one_buffer_t *one_buffer);
void osd_one_buffer_dec_by_addr(uint8_t *vaddr);
void osd_one_buffer_dec_by_paddr(uint64_t paddr);
int32_t osd_one_buffer_create(struct ion_client *client, osd_one_buffer_t *one_buffer);
void osd_one_buffer_destroy(struct ion_client *client, osd_one_buffer_t *one_buffer);
void osd_one_buffer_flush(osd_one_buffer_t *one_buffer);

int32_t osd_buffer_create(struct ion_client *client, osd_buffer_t *osd_buffer);
int32_t osd_buffer_create_vga(struct ion_client *client, osd_buffer_t *osd_buffer);
void osd_buffer_destroy(struct ion_client *client, osd_buffer_t *osd_buffer);
void osd_buffer_destroy_vga(struct ion_client *client, osd_buffer_t *osd_buffer);
void osd_buffer_flush(osd_buffer_t *osd_buffer);

struct vio_frame *osd_get_frame(struct list_head *list, int32_t frame_index);
void osd_put_input_frame(struct list_head *list, struct vio_frame *frame);
struct vio_frame *osd_get_input_frame(struct list_head *list);

#endif // __HOBOT_OSD_MEM_H__
