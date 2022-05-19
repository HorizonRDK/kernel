/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/kernel.h>
#include <linux/slab.h>
#include <asm/neon.h>
#include <linux/time.h>

#include "osd_config.h"
#include "hobot_osd_process.h"
#include "hobot_dev_osd.h"

extern osd_color_map_t g_osd_color;

#define OSD_U8_BITS 8u
#define OSD_U16_BITS 16u
#define OSD_NEON_PROC_BITS 128u
#define OSD_NEON_PROC_U8 (OSD_NEON_PROC_BITS / OSD_U8_BITS)
#define OSD_NEON_PROC_U16 (OSD_NEON_PROC_BITS / OSD_U16_BITS)
#define OSD_NEON_PROC_BYTES OSD_NEON_PROC_U8

#define OSD_Y_BITS 8u
#define OSD_U_BITS 8u
#define OSD_V_BITS 8u
#define OSD_UV_BITS (OSD_U_BITS + OSD_V_BITS)
#define OSD_YUV_BITS (OSD_Y_BITS + OSD_UV_BITS)
#define OSD_FILL_F0 0xf0u
#define OSD_FILL_0F 0x0fu
#define OSD_FILL_FF 0xffu
#define OSD_FILL_FFFF 0xffffu
#define OSD_FILL_FF00 0xff00u

extern int g_osd_fps[VIO_MAX_STREAM][VIO_MAX_STREAM];
extern int g_osd_idx[VIO_MAX_STREAM][VIO_MAX_STREAM];
extern long int g_osd_fps_lasttime[VIO_MAX_STREAM][VIO_MAX_STREAM];

void osd_process_workfunc_done(osd_process_info_t *proc_info)
{
    struct osd_subdev *subdev = proc_info->subdev;
    osd_work_frame_t *work_frame;
    struct ipu_subdev *ipu_subdev;
    struct pym_subdev *pym_subdev;
    unsigned long flags;
    int32_t instance;
    struct timeval tmp_tv;

    switch (proc_info->proc_type)
    {
    case OSD_PROC_VGA4:
        osd_one_buffer_dec_by_addr(proc_info->src_vga_addr);
        break;
    case OSD_PROC_NV12:
        osd_one_buffer_dec_by_addr(proc_info->src_addr);
        break;
    default:
        break;
    }

    if (subdev != NULL) {
        spin_lock_irqsave(&subdev->frame_slock, flags);
        work_frame = osd_find_work_frame(&subdev->work_frame_list,
                proc_info->buffer_index);
        if (work_frame != NULL) {
            if (atomic_dec_return(&work_frame->process_cnt) == 0) {
                if (subdev->id == OSD_PYM_OUT) {
                    pym_subdev = container_of(subdev->osd_info, struct pym_subdev, osd_info);
                    instance = pym_subdev->group->instance;
                } else {
                    ipu_subdev = container_of(subdev->osd_info, struct ipu_subdev, osd_info);
                    instance = ipu_subdev->group->instance;
                }
                do_gettimeofday(&tmp_tv);
                g_osd_idx[instance][subdev->id]++;
                if (tmp_tv.tv_sec > g_osd_fps_lasttime[instance][subdev->id]) {
                    g_osd_fps[instance][subdev->id] =
                            g_osd_idx[instance][subdev->id];
                    g_osd_fps_lasttime[instance][subdev->id] =
                            tmp_tv.tv_sec;
                    g_osd_idx[instance][subdev->id] = 0;
                }

                if (subdev->osd_sta.sta_state == OSD_STA_PROCESS) {
                    subdev->osd_sta.sta_state = OSD_STA_DONE;
                }

                list_del(&work_frame->node);

                atomic_dec(&subdev->osd_info->frame_count);
                subdev->osd_info->return_frame(subdev->osd_info, work_frame->frame);
                kfree(work_frame);
                work_frame = NULL;
            }
        }

        spin_unlock_irqrestore(&subdev->frame_slock, flags);
    }
}

/*
 *vga4:     pixel1  pixel0  pixel3  pixel2
 *          |---1byte----|  |---1byte----|
 *
 * yuv420:y:pixel0  pixel1  pixel2  pixel3
 *          |1byte| |1byte| |1byte| |1byte|
 *          ...
 *       uv:pixel0  pixel1  pixel2  pixel3
 *          pixel4  pixel5  pixel6  pixel7
 *          |---2byte----|  |---2byte----|
 *          ...
 *
 * buffer: xx 00 00 xx
 * or:     00 ff ff 00
 * use for transparent realization
 */
int32_t osd_vga4_to_sw(uint32_t *color_map, uint8_t *src_addr,
    uint8_t *tar_addr, uint32_t width, uint32_t height)
{
    uint8_t *vga_addr;
    uint8_t color_index, high_color, low_color;
    uint8_t *addr_y, *addr_y_or;
    uint8_t *addr_uv, *addr_uv_or;
    uint8_t *addr_y_offset, *addr_y_offset_or;
    uint8_t *addr_uv_offset, *addr_uv_offset_or;
    uint16_t color;
    struct timeval time_now = { 0 };
    struct timeval time_next = { 0 };
    time_t time_us;
    uint32_t h, w, base_offset;
    uint32_t *color_map_tmp;

    do_gettimeofday(&time_now);

    vga_addr = src_addr;
    addr_y = tar_addr;
    addr_uv = &addr_y[width * height];
    addr_y_or = &addr_uv[(width * height) >> 1];
    addr_uv_or = &addr_y_or[width * height];

    if (color_map == NULL) {
        color_map_tmp = g_osd_color.color_map;
    } else {
        color_map_tmp = color_map;
    }

    vio_dbg("vga4-->yuv420 width:%d height:%d vga4 addr:%p"
        " yuv420 addr:%p\n",
        width, height, src_addr, tar_addr);

    memset((void *)addr_y, 0x00, (size_t)width * (size_t)height);
    memset((void *)addr_uv, 0x00, ((size_t)width * (size_t)height) >> 1u);
    memset((void *)addr_y_or, (int32_t)OSD_FILL_FF,
        (size_t)width * (size_t)height);
    memset((void *)addr_uv_or, (int32_t)OSD_FILL_FF,
        ((size_t)width * (size_t)height) >> 1u);

    for (h = 0; h < height; h++) {
        base_offset = h * width;
        for (w = 0; w < width; w += 2) {
            color_index = vga_addr[(base_offset + w) >> 1u];
            if (color_index != OSD_FILL_FF) {
                high_color = color_index >> 4;
                low_color = color_index & OSD_FILL_0F;
                addr_y_offset = &addr_y[base_offset + w];
                addr_uv_offset = &addr_uv[((h >> 1u) * width) + w];
                addr_y_offset_or = &addr_y_or[base_offset + w];
                addr_uv_offset_or = &addr_uv_or[((h >> 1u) * width) + w];

                if (high_color != OSD_FILL_0F) {
                    addr_y_offset[1] = (uint8_t)(color_map_tmp[high_color] >>
                        OSD_UV_BITS);
                    color = (uint16_t)(color_map_tmp[high_color] &
                        OSD_FILL_FFFF);
                    *(uint16_t *)addr_uv_offset = (uint16_t)((color << OSD_U_BITS) |
                        (color >> OSD_V_BITS));

                    addr_y_offset_or[1] = 0x00;
                    addr_uv_offset_or[0] = 0x00;
                    addr_uv_offset_or[1] = 0x00;
                }
                if (low_color != OSD_FILL_0F) {
                    addr_y_offset[0] = (uint8_t)(color_map_tmp[low_color] >>
                        OSD_UV_BITS);
                    color = (uint16_t)(color_map_tmp[low_color] &
                        OSD_FILL_FFFF);
                    *(uint16_t *)addr_uv_offset = (uint16_t)((color << OSD_U_BITS) |
                        (color >> OSD_V_BITS));

                    addr_y_offset_or[0] = 0x00;
                    addr_uv_offset_or[0] = 0x00;
                    addr_uv_offset_or[1] = 0x00;
                }
            }
        }
    }
    do_gettimeofday(&time_next);
    time_us = (((time_next.tv_sec * 1000 * 1000) + time_next.tv_usec) -
        ((time_now.tv_sec * 1000 * 1000) + time_now.tv_usec));
    vio_dbg("osd software vga4 -> yuv420 cost %ldus\n",
              time_us);

    return 0;
}

int32_t osd_process_info_check(osd_process_info_t *proc_info,
    uint32_t *crop_width, uint32_t *crop_height)
{
    if ((proc_info->subdev != NULL) &&
        (proc_info->subdev->osd_dev->task_state >= OSD_TASK_REQUEST_STOP)) {
        return -EBUSY;
    }
    if ((__pa(proc_info->tar_y_addr) == 0) ||
        (__pa(proc_info->tar_uv_addr) == 0)) {
        vio_err("osd process type:%d tar_addr y:%lld uv:%lld error\n",
            proc_info->proc_type,
            __pa(proc_info->tar_y_addr), __pa(proc_info->tar_uv_addr));
        return -EINVAL;
    }
    if ((proc_info->tar_y_addr == NULL) || (proc_info->tar_uv_addr == NULL)) {
        vio_err("osd process type:%d tar_addr y:%p uv:%p error\n",
            proc_info->proc_type, proc_info->tar_y_addr, proc_info->tar_uv_addr);
        return -EINVAL;
    }

    switch (proc_info->proc_type) {
    case OSD_PROC_HW_VGA4:
    case OSD_PROC_NV12:
        if (proc_info->src_addr == NULL) {
            vio_err("osd process type:%d src_addr:%p error\n",
                proc_info->proc_type, proc_info->src_addr);
            return -EINVAL;
        }
        break;
    case OSD_PROC_VGA4:
        if (proc_info->src_vga_addr == NULL) {
            vio_err("osd process type:%d src_vga_addr:%p error\n",
                proc_info->proc_type, proc_info->src_vga_addr);
            return -EINVAL;
        }
        break;
    case OSD_PROC_POLYGON:
        if (proc_info->polygon_buf == NULL) {
            vio_err("osd process type:%d polygon_buf:%p error\n",
                proc_info->proc_type, proc_info->polygon_buf);
            return -EINVAL;
        }
        break;
    default:
        break;
    }

    if (proc_info->image_height > (proc_info->start_y + proc_info->height)) {
        *crop_height = proc_info->height;
    } else if (proc_info->image_height > proc_info->start_y) {
        *crop_height = proc_info->image_height - proc_info->start_y;
    } else {
        *crop_height = 0;
    }
    if (proc_info->image_width > (proc_info->start_x + proc_info->width)) {
        *crop_width = ALIGN_DOWN(proc_info->width, 16);
    } else if (proc_info->image_width > proc_info->start_x) {
        *crop_width = ALIGN_DOWN((proc_info->image_width - proc_info->start_x), 16);
    } else {
        *crop_width = 0;
    }
    if ((*crop_width == 0) || (*crop_height == 0)) {
        return -EINVAL;
    }

    return 0;
}

void osd_process_vga4_workfunc(struct kthread_work *work)
{
    osd_process_info_t *proc_info = container_of(work, osd_process_info_t, work);
    uint8_t *tar_y_addr, *tar_uv_addr;
    uint8_t *src_y_addr, *src_y_or_addr, *src_uv_addr, *src_uv_or_addr;
    uint32_t offset;
    struct timeval time_now = { 0 };
    struct timeval time_next = { 0 };
    time_t time_us;
    uint32_t crop_width, crop_height;

    mutex_lock(&proc_info->proc_mutex);

    if (osd_process_info_check(proc_info, &crop_width, &crop_height) < 0) {
        goto exit;
    }

    do_gettimeofday(&time_now);

    offset = proc_info->height * proc_info->width;
    src_y_addr = proc_info->src_vga_addr;
    src_uv_addr = src_y_addr + offset;
    src_y_or_addr = src_uv_addr + (offset / 2);
    src_uv_or_addr = src_y_or_addr + offset;
    offset = proc_info->start_y * proc_info->image_width;
    tar_y_addr = proc_info->tar_y_addr + offset + proc_info->start_x;
    tar_uv_addr = proc_info->tar_uv_addr + offset / 2 + proc_info->start_x;

    kernel_neon_begin();
    asm volatile (
        "CBZ    %[invert_en], vga4_loop_h                           \n"
        "MOV    w26, #0xff                                          \n"
        "DUP    v4.16B, w26                                         \n"
        "vga4_loop_h:                                               \n"
         // process uv if (h % 2 == 1)
        "AND    x19, %[crop_height], #1                             \n"
        "MOV    x20, %[tar_y_addr]                                  \n"
        "MOV    x21, %[tar_uv_addr]                                 \n"
        "MOV    x22, %[src_y_addr]                                  \n"
        "MOV    x23, %[src_uv_addr]                                 \n"
        "MOV    x24, %[src_y_or_addr]                               \n"
        "MOV    x25, %[src_uv_or_addr]                              \n"

        "MOV    x26, #0                                             \n"
        "vga4_loop_w:"
        // proccess y
        "LD1    {v0.16B}, [x20]                                     \n"
        "LD1    {v1.16B}, [x22]                                     \n"
        "LD1    {v2.16B}, [x24]                                     \n"
        "AND    v0.16B, v2.16B, v0.16B                              \n"

        "CBZ    %[invert_en], vga4_y_normal_process                 \n"
        "SUB    v3.16B, v4.16B, v2.16B                              \n"
        "SUB    v1.16B, v3.16B, v1.16B                              \n"
        "vga4_y_normal_process:"
        "ADD    v0.16B, v0.16B, v1.16B                              \n"
        // "vga4_y_start_st:"
        // neon process 16 bytes once
        "ST1    {v0.16B}, [x20], #16                                \n"
        "ADD    x22, x22, #16                                       \n"
        "ADD    x24, x24, #16                                       \n"

        // proccess uv
        "CBZ    x19, vga4_skip_uv_process                           \n"
        "LD1    {v0.16B}, [x21]                                     \n"
        "LD1    {v1.16B}, [x23]                                     \n"
        "LD1    {v2.16B}, [x25]                                     \n"
        "AND    v0.16B, v2.16B, v0.16B                              \n"
        
        "CBZ    %[invert_en], vga4_uv_normal_process                \n"
        "SUB    v3.16B, v4.16B, v2.16B                              \n"
        "SUB    v1.16B, v3.16B, v1.16B                              \n"
        "vga4_uv_normal_process:"
        "ADD    v0.16B, v0.16B, v1.16B                              \n"
        // neon process 16 bytes once
        "ST1    {v0.16B}, [x21], #16                                \n"
        "ADD    x23, x23, #16                                       \n"
        "ADD    x25, x25, #16                                       \n"
        "vga4_skip_uv_process:                                      \n"

        "ADD    x26, x26, #16                                       \n"
        "CMP    x26, %[crop_width]                                  \n"
        "B.LT   vga4_loop_w                                         \n"
        "vga4_loop_w_done:                                          \n"

        // y addr add
        "ADD    %[tar_y_addr], %[tar_y_addr], %[image_width]        \n"
        "ADD    %[src_y_addr], %[src_y_addr], %[osd_width]          \n"
        "ADD    %[src_y_or_addr], %[src_y_or_addr], %[osd_width]    \n"
        // uv addr add
        "CBZ    x19, vga4_skip_uv_addr_add                          \n"
        "ADD    %[tar_uv_addr], %[tar_uv_addr], %[image_width]      \n"
        "ADD    %[src_uv_addr], %[src_uv_addr], %[osd_width]        \n"
        "ADD    %[src_uv_or_addr], %[src_uv_or_addr], %[osd_width]  \n"
        "vga4_skip_uv_addr_add:                                     \n"

        "SUB    %[crop_height], %[crop_height], #1                  \n"

        "CMP    %[crop_height], #0                                  \n"
        "B.NE   vga4_loop_h                                         \n"
        "vga4_loop_h_done:                                          \n"
        :
        : [tar_y_addr]"r"(tar_y_addr),
          [tar_uv_addr]"r"(tar_uv_addr),
          [src_y_addr]"r"(src_y_addr),
          [src_uv_addr]"r"(src_uv_addr),
          [src_y_or_addr]"r"(src_y_or_addr),
          [src_uv_or_addr]"r"(src_uv_or_addr),
          [image_width]"r"(proc_info->image_width),
          [osd_width]"r"(proc_info->width),
          [crop_width]"r"(crop_width),
          [crop_height]"r"(crop_height),
          [invert_en]"r"(proc_info->invert_en)
        : "cc", "memory", "v0", "v1", "v2", "v3", "v4",
            "x19", "x20", "x21", "x22", "x23", "x24", "x25", "x26"    // Clobber List
    );
    kernel_neon_end();

    if (__pa(tar_y_addr) != 0) {
        ion_dcache_flush(__pa(tar_y_addr),
            proc_info->image_width * crop_height);
    }
    if (__pa(tar_uv_addr) != 0) {
        ion_dcache_flush(__pa(tar_uv_addr),
            proc_info->image_width * crop_height / 2);
    }

    do_gettimeofday(&time_next);
    time_us = (((time_next.tv_sec * 1000 * 1000) + time_next.tv_usec) -
        ((time_now.tv_sec * 1000 * 1000) + time_now.tv_usec));
    vio_dbg("osd vga4 process, show:%d invert:%d level:%d, "
        "frame_id:%d buf_index:%d image w:%d h:%d, "
        "x:%d y:%d w:%d h:%d src: y_addr:%p %p uv_addr:%p %p, "
        "image y:%p uv:%p, cost %ldus\n",
        proc_info->show_en, proc_info->invert_en, proc_info->osd_level,
        proc_info->frame_id, proc_info->buffer_index,
        proc_info->image_width, proc_info->image_height,
        proc_info->start_x, proc_info->start_y, proc_info->width,
        proc_info->height, src_y_addr, src_y_or_addr, src_uv_addr, src_uv_or_addr,
        tar_y_addr, tar_uv_addr, time_us);

exit:
    osd_process_workfunc_done(proc_info);
    mutex_unlock(&proc_info->proc_mutex);
}

void osd_process_nv12_workfunc(struct kthread_work *work)
{
    osd_process_info_t *proc_info = container_of(work, osd_process_info_t, work);
    uint8_t *tar_y_addr, *tar_uv_addr, *src_y_addr, *src_uv_addr;
    uint32_t offset;
    struct timeval time_now = { 0 };
    struct timeval time_next = { 0 };
    time_t time_us;
    uint32_t crop_width, crop_height;

    mutex_lock(&proc_info->proc_mutex);
    if (osd_process_info_check(proc_info, &crop_width, &crop_height) < 0) {
        goto exit;
    }

    do_gettimeofday(&time_now);

    offset = proc_info->height * proc_info->width;
    src_y_addr = proc_info->src_addr;
    src_uv_addr = src_y_addr + offset;
    offset = proc_info->start_y * proc_info->image_width;
    tar_y_addr = proc_info->tar_y_addr + offset + proc_info->start_x;
    tar_uv_addr = proc_info->tar_uv_addr + offset / 2 + proc_info->start_x;

    kernel_neon_begin();
    asm volatile (
        "LSR    w19, %w[bgtrans], #24                               \n"
        "CBZ    w19, nv12_bgtrans_done                              \n"
        // y
        "LSR    w20, %w[bgtrans], #16                               \n"
        "AND    w20, w20, #0xff                                     \n"
        "DUP    v0.16B, w20                                         \n"
        // u
        "AND    w20, %w[bgtrans], #0xff                             \n"
        "LSL    w20, w20, #8                                        \n"
        // v
        "LSR    w21, %w[bgtrans], #8                                \n"
        "AND    w21, w21, #0xff                                     \n"

        "ORR    w20, w21, w20                                       \n"
        "DUP    v1.8H, w20                                          \n"
        "nv12_bgtrans_done:"
        "MOV    %w[bgtrans], w19                                    \n"

        "CBZ    %[invert_en], nv12_loop_h                           \n"
        "MOV    w19, #0xff                                          \n"
        "DUP    v2.16B, w19                                         \n"
        "nv12_loop_h:                                               \n"
         // process uv if (h % 2 == 1)
        "AND    x19, %[crop_height], #1                              \n"
        "MOV    x20, %[tar_y_addr]                                  \n"
        "MOV    x21, %[tar_uv_addr]                                 \n"
        "MOV    x22, %[src_y_addr]                                  \n"
        "MOV    x23, %[src_uv_addr]                                 \n"

        "MOV    x24, #0                                             \n"
        "nv12_loop_w:"
        // proccess y
        "LD1    {v3.16B}, [x22]                                     \n"
        "LD1    {v4.16B}, [x23]                                     \n"
        "MOV    v5.16B, v3.16B                                      \n"
        "MOV    v6.16B, v4.16B                                      \n"

        "CBZ    %[invert_en], nv12_y_bgtrans_process                \n"
        "SUB    v5.16B, v2.16B, v3.16B                              \n"
        "SUB    v6.16B, v2.16B, v4.16B                              \n"

        "nv12_y_bgtrans_process:"
        "CBZ    %w[bgtrans], nv12_y_normal_process                  \n"
        "CMEQ   v7.16B, v3.16B, v0.16B                              \n"
        "LD1    {v8.8H}, [x23]                                      \n"
        "CMEQ   v8.8H, v8.8H, v1.8H                                 \n"
        "CMTST  v7.16B, v7.16B, v8.16B                              \n"
        "USHR   v8.16B, v7.16B, #7                                  \n"
        "MVN    v7.16B, v7.16B                                      \n"
        "USHR   v7.16B, v7.16B, #7                                  \n"
        "LD1    {v9.16B}, [x20]                                     \n"
        "LD1    {v10.16B}, [x21]                                    \n"
        "MUL    v5.16B, v5.16B, v7.16B                              \n"
        "MUL    v9.16B, v9.16B, v8.16B                              \n"
        "ADD    v5.16B, v9.16B, v5.16B                              \n"
        "MUL    v6.16B, v6.16B, v7.16B                              \n"
        "MUL    v10.16B, v10.16B, v8.16B                            \n"
        "ADD    v6.16B, v10.16B, v6.16B                             \n"

        "nv12_y_normal_process:"
        // neon process 16 bytes once
        "ST1    {v5.16B}, [x20], #16                                \n"
        "ADD    x22, x22, #16                                       \n"

        // proccess uv
        "CBZ    x19, nv12_skip_uv_process                           \n"
        "ST1    {v6.16B}, [x21]                                     \n"
        "nv12_skip_uv_process:                                      \n"
        "ADD    x21, x21, #16                                       \n"
        "ADD    x23, x23, #16                                       \n"

        "ADD    x24, x24, #16                                       \n"
        "CMP    x24, %[crop_width]                                  \n"
        "B.LT   nv12_loop_w                                         \n"
        "nv12_loop_w_done:                                          \n"

        // y addr add
        "ADD    %[tar_y_addr], %[tar_y_addr], %[image_width]        \n"
        "ADD    %[src_y_addr], %[src_y_addr], %[osd_width]          \n"
        // uv addr add
        "CBZ    x19, nv12_skip_uv_addr_add                          \n"
        "ADD    %[tar_uv_addr], %[tar_uv_addr], %[image_width]      \n"
        "ADD    %[src_uv_addr], %[src_uv_addr], %[osd_width]        \n"
        "nv12_skip_uv_addr_add:                                     \n"

        "SUB    %[crop_height], %[crop_height], #1                  \n"
        "CMP    %[crop_height], #0                                  \n"
        "B.NE   nv12_loop_h                                         \n"
        "nv12_loop_h_done:                                          \n"
        :
        : [tar_y_addr]"r"(tar_y_addr),
          [tar_uv_addr]"r"(tar_uv_addr),
          [src_y_addr]"r"(src_y_addr),
          [src_uv_addr]"r"(src_uv_addr),
          [image_width]"r"(proc_info->image_width),
          [osd_width]"r"(proc_info->width),
          [crop_width]"r"(crop_width),
          [crop_height]"r"(crop_height),
          [invert_en]"r"(proc_info->invert_en),
          [bgtrans]"r"(proc_info->yuv_bg_transparent)
        : "cc", "memory", "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10",
            "x19", "x20", "x21", "x22", "x23", "x24"    // Clobber List
    );
    kernel_neon_end();
    if (__pa(tar_y_addr) != 0) {
        ion_dcache_flush(__pa(tar_y_addr),
            proc_info->image_width * crop_height);
    }
    if (__pa(tar_uv_addr) != 0) {
        ion_dcache_flush(__pa(tar_uv_addr),
            proc_info->image_width * crop_height / 2);
    }

    do_gettimeofday(&time_next);
    time_us = (((time_next.tv_sec * 1000 * 1000) + time_next.tv_usec) -
        ((time_now.tv_sec * 1000 * 1000) + time_now.tv_usec));
    vio_dbg("osd nv12 process, show:%d invert:%d level:%d, "
        "frame_id:%d buf_index:%d image w:%d h:%d, "
        "x:%d y:%d w:%d h:%d bg_trans:0x%x"
        " src: y_addr:%p uv_addr:%p, image y:%p uv:%p, cost %ldus\n",
        proc_info->show_en, proc_info->invert_en, proc_info->osd_level,
        proc_info->frame_id, proc_info->buffer_index,
        proc_info->image_width, proc_info->image_height,
        proc_info->start_x, proc_info->start_y, proc_info->width,
        proc_info->height, proc_info->yuv_bg_transparent,
        src_y_addr, src_uv_addr, tar_y_addr, tar_uv_addr, time_us);

exit:
    osd_process_workfunc_done(proc_info);
    mutex_unlock(&proc_info->proc_mutex);
}

void osd_process_rect_workfunc(struct kthread_work *work)
{
    osd_process_info_t *proc_info = container_of(work, osd_process_info_t, work);
    uint8_t *tar_y_addr, *tar_uv_addr;
    uint32_t yuv_color;
    uint8_t y_color;
    uint16_t uv_color;
    struct timeval time_now = { 0 };
    struct timeval time_next = { 0 };
    time_t time_us;
    uint32_t crop_width, crop_height;

    mutex_lock(&proc_info->proc_mutex);
    if (osd_process_info_check(proc_info, &crop_width, &crop_height) < 0) {
        goto exit;
    }

    do_gettimeofday(&time_now);

    tar_y_addr = proc_info->tar_y_addr +
        proc_info->start_y * proc_info->image_width + proc_info->start_x;
    tar_uv_addr = proc_info->tar_uv_addr +
        proc_info->start_y * proc_info->image_width / 2 + proc_info->start_x;
    yuv_color = g_osd_color.color_map[proc_info->fill_color & OSD_FILL_0F];
    y_color = (yuv_color >> OSD_UV_BITS) & OSD_FILL_FF;
    uv_color = yuv_color & OSD_FILL_FFFF;
    // uint16 store, need invert
    uv_color = (uint16_t)(((uv_color & OSD_FILL_FF) << 8) |
        ((uv_color & OSD_FILL_FF00) >> 8));
    if (proc_info->invert_en == 1u) {
        y_color = (uint8_t)(OSD_FILL_FF - y_color);
        uv_color = (uint16_t)(OSD_FILL_FFFF - uv_color);
    }

    kernel_neon_begin();
    asm volatile (
        "DUP    v0.16B, %w[y_color]                             \n"
        "DUP    V1.16B, %w[uv_color]                            \n"

        "rect_loop_h:                                           \n"
        // process uv if (h % 2 == 1)
        "AND    x19, %[crop_height], #1                         \n"
        "MOV    x20, %[tar_y_addr]                              \n"
        "MOV    x21, %[tar_uv_addr]                             \n"

        "MOV    x22, #0                                         \n"
        "rect_loop_w:"
        // proccess y
        "ST1    {v0.16B}, [x20], #16                            \n"
        // proccess uv
        "CBZ    x19, rect_skip_uv_process                       \n"
        "ST1    {v1.16B}, [x21], #16                            \n"
        "rect_skip_uv_process:                                  \n"

        // neon process 16 bytes once
        "ADD    x22, x22, #16                                   \n"
        "CMP    x22, %[crop_width]                              \n"
        "B.LT   rect_loop_w                                     \n"
        "rect_loop_w_done:                                      \n"

        // y addr add
        "ADD    %[tar_y_addr], %[tar_y_addr], %[image_width]    \n"
        // uv addr add
        "CBZ    x19, rect_skip_uv_addr_add                      \n"
        "ADD    %[tar_uv_addr], %[tar_uv_addr], %[image_width]  \n"
        "rect_skip_uv_addr_add:                                 \n"

        "SUB    %[crop_height], %[crop_height], #1              \n"
        "CMP    %[crop_height], #0                              \n"
        "B.GT   rect_loop_h                                     \n"
        "rect_loop_h_done:                                      \n"
        :
        : [tar_y_addr]"r"(tar_y_addr),
          [tar_uv_addr]"r"(tar_uv_addr),
          [image_width]"r"(proc_info->image_width),
          [osd_width]"r"(proc_info->width),
          [crop_width]"r"(crop_width),
          [crop_height]"r"(crop_height),
          [y_color]"r"(y_color),
          [uv_color]"r"(uv_color)
        //   [invert_en]"r"(proc_info->invert_en)
        : "cc", "memory", "v0", "v1", "x19", "x20", "x21", "x22"    // Clobber List
    );
    kernel_neon_end();

    if (__pa(tar_y_addr) != 0) {
        ion_dcache_flush(__pa(tar_y_addr),
            proc_info->image_width * crop_height);
    }
    if (__pa(tar_uv_addr) != 0) {
        ion_dcache_flush(__pa(tar_uv_addr),
            proc_info->image_width * crop_height / 2);
    }

    do_gettimeofday(&time_next);
    time_us = (((time_next.tv_sec * 1000 * 1000) + time_next.tv_usec) -
        ((time_now.tv_sec * 1000 * 1000) + time_now.tv_usec));

    vio_dbg("osd rect process, show:%d invert:%d level:%d, "
        "frame_id:%d buf_index:%d image w:%d h:%d, "
        "x:%d y:%d w:%d h:%d y_addr:%p uv_addr:%p "
        "color index:%d yuv:0x%x, cost %ldus\n",
        proc_info->show_en, proc_info->invert_en, proc_info->osd_level,
        proc_info->frame_id, proc_info->buffer_index,
        proc_info->image_width, proc_info->image_height,
        proc_info->start_x, proc_info->start_y, proc_info->width,
        proc_info->height, tar_y_addr, tar_uv_addr, proc_info->fill_color,
        yuv_color, time_us);

exit:
    osd_process_workfunc_done(proc_info);
    mutex_unlock(&proc_info->proc_mutex);
}

void osd_process_polygon_workfunc(struct kthread_work *work)
{
    osd_process_info_t *proc_info = container_of(work, osd_process_info_t, work);
    uint8_t *tar_y_addr, *tar_uv_addr;
    uint16_t temp_index[OSD_NEON_PROC_U16];
    int16_t i = 0;
    uint32_t yuv_color;
    uint8_t y_color;
    uint16_t uv_color;
    struct timeval time_now = { 0 };
    struct timeval time_next = { 0 };
    time_t time_us;
    uint32_t crop_width, crop_height;

    mutex_lock(&proc_info->proc_mutex);
    if (osd_process_info_check(proc_info, &crop_width, &crop_height) < 0) {
        goto exit;
    }

    do_gettimeofday(&time_now);

    tar_y_addr = proc_info->tar_y_addr +
        proc_info->start_y * proc_info->image_width + proc_info->start_x;
    tar_uv_addr = proc_info->tar_uv_addr +
        proc_info->start_y * proc_info->image_width / 2 + proc_info->start_x;
    yuv_color = g_osd_color.color_map[proc_info->fill_color & OSD_FILL_0F];
    y_color = (yuv_color >> OSD_UV_BITS) & OSD_FILL_FF;
    uv_color = yuv_color & OSD_FILL_FFFF;
    // uint16 store, need invert
    uv_color = (uint16_t)(((uv_color & OSD_FILL_FF) << 8) |
        ((uv_color & OSD_FILL_FF00) >> 8));
    if (proc_info->invert_en == 1u) {
        y_color = (uint8_t)(OSD_FILL_FF - y_color);
        uv_color = (uint16_t)(OSD_FILL_FFFF - uv_color);
    }

    for (i = 0; i < OSD_NEON_PROC_U16; i++) {
        temp_index[i] = i;
    }

    kernel_neon_begin();
    asm volatile (
        "DUP    v0.8H, %w[y_color]                              \n"
        "DUP    v1.4S, %w[uv_color]                             \n"
        "LD1    {v2.8H}, [%[temp_index]]                        \n"
        "MOV    w19, #1                                         \n"

        "polygon_loop_h:                                        \n"
        // process uv if (h % 2 == 1)
        "AND    x19, %[crop_height], #1                         \n"
        "MOV    x20, %[tar_y_addr]                              \n"
        "MOV    x21, %[tar_uv_addr]                             \n"
        "LDR    w22, [%[polygon_buf]], #4                       \n"
        "LDR    w23, [%[polygon_buf]], #4                       \n"
        "DUP    v3.8H, w22                                      \n"
        "DUP    v4.8H, w23                                      \n"

        "MOV    w24, #0                                         \n"
        "polygon_loop_w:"
        // proccess y
        "DUP    v5.8H, w24                                      \n"
        "ADD    v5.8H, v5.8H, v2.8H                             \n"
        "CMHS   v6.8H, v5.8H, v3.8H                             \n"
        "CMHS   v7.8H, v4.8H, v5.8H                             \n"
        "AND    v5.16B, v6.16B, v7.16B                          \n"
        "MVN    v6.16B, v5.16B                                  \n"

        "USHR   v7.8H, v5.8H, #15                               \n"
        "MUL    v8.8H, v7.8H, v0.8H                             \n"
        "USHR   v7.8H, v6.8H, #15                               \n"
        "LD1    {v9.8B}, [x20]                                  \n"
        "USHLL  v9.8H, v9.8B, #0                                \n"
        "MUL    v9.8H, v7.8H, v9.8H                             \n"
        "ADD    v8.8H, v8.8H, v9.8H                             \n"
        "UQXTN  v8.8B, v8.8H                                    \n"

        "ST1    {v8.8B}, [x20], #8                              \n"

        // proccess uv
        "CBZ    x19, polygon_skip_uv_process                    \n"
        "UADDLP v7.4S, v5.8H                                    \n"
        "CMGT   v7.4S, v7.4S, #0                                \n"
        "MVN    v8.16B, v7.16B                                  \n"
        "USHR   v7.4S, v7.4S, #31                               \n"
        "MUL    v7.4S, v7.4S, v1.4S                             \n"
        "USHR   v8.4S, v8.4S, #31                               \n"
        "LD1    {v9.4H}, [x21]                                  \n"
        "USHLL  v9.4S, v9.4H, #0                                \n"
        "MUL    v8.4S, v8.4S, v9.4S                             \n"
        "ADD    v7.4S, v7.4S, v8.4S                             \n"
        "UQXTN  v7.4H, v7.4S                                    \n"

        "ST1    {v7.4H}, [x21], #8                              \n"
        "polygon_skip_uv_process:                               \n"

        // neon process 16 bytes once
        "ADD    w24, w24, #8                                    \n"
        "CMP    w24, %w[crop_width]                             \n"
        "B.LT   polygon_loop_w                                  \n"
        "polygon_loop_w_done:                                   \n"

        // y addr add
        "ADD    %[tar_y_addr], %[tar_y_addr], %[image_width]    \n"
        // uv addr add
        "CBZ    x19, polygon_skip_uv_addr_add                   \n"
        "ADD    %[tar_uv_addr], %[tar_uv_addr], %[image_width]  \n"
        "polygon_skip_uv_addr_add:                              \n"

        "SUB    %[crop_height], %[crop_height], #1              \n"
        "CMP    %[crop_height], #0                              \n"
        "B.GT   polygon_loop_h                                  \n"
        "polygon_loop_h_done:                                   \n"
        :
        : [tar_y_addr]"r"(tar_y_addr),
          [tar_uv_addr]"r"(tar_uv_addr),
          [image_width]"r"(proc_info->image_width),
          [osd_width]"r"(proc_info->width),
          [crop_width]"r"(crop_width),
          [crop_height]"r"(crop_height),
          [y_color]"r"(y_color),
          [uv_color]"r"(uv_color),
          [polygon_buf]"r"(proc_info->polygon_buf),
          [temp_index]"r"(temp_index)
        : "cc", "memory", "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9",
            "x19", "x20", "x21", "x22", "x23", "x24"
    );
    kernel_neon_end();

    if (__pa(tar_y_addr) != 0) {
        ion_dcache_flush(__pa(tar_y_addr),
            proc_info->image_width * crop_height);
    }
    if (__pa(tar_uv_addr) != 0) {
        ion_dcache_flush(__pa(tar_uv_addr),
            proc_info->image_width * crop_height / 2);
    }

    do_gettimeofday(&time_next);
    time_us = (((time_next.tv_sec * 1000 * 1000) + time_next.tv_usec) -
        ((time_now.tv_sec * 1000 * 1000) + time_now.tv_usec));

    vio_dbg("osd polygon process, show:%d invert:%d level:%d, "
        "frame_id:%d buf_index:%d image w:%d h:%d, "
        "x:%d y:%d w:%d h:%d y_addr:%p uv_addr:%p "
        "polygon_buf:%p color index:%d yuv:0x%x, cost %ldus\n",
        proc_info->show_en, proc_info->invert_en, proc_info->osd_level,
        proc_info->frame_id, proc_info->buffer_index,
        proc_info->image_width, proc_info->image_height,
        proc_info->start_x, proc_info->start_y, proc_info->width,
        proc_info->height, tar_y_addr, tar_uv_addr, proc_info->polygon_buf,
        proc_info->fill_color, yuv_color, time_us);

exit:
    osd_process_workfunc_done(proc_info);
    mutex_unlock(&proc_info->proc_mutex);
}

void osd_process_mosaic_workfunc(struct kthread_work *work)
{
    osd_process_info_t *proc_info = container_of(work, osd_process_info_t, work);
    uint8_t *tar_y_addr, *tar_uv_addr;
    struct timeval time_now = { 0 };
    struct timeval time_next = { 0 };
    time_t time_us;
    uint32_t crop_width, crop_height;

    mutex_lock(&proc_info->proc_mutex);
    if (osd_process_info_check(proc_info, &crop_width, &crop_height) < 0) {
        goto exit;
    }

    do_gettimeofday(&time_now);

    tar_y_addr = proc_info->tar_y_addr +
        proc_info->start_y * proc_info->image_width + proc_info->start_x;
    tar_uv_addr = proc_info->tar_uv_addr +
        proc_info->start_y * proc_info->image_width / 2 + proc_info->start_x;

    kernel_neon_begin();
    asm volatile (
        "mosaic_loop_h:                                     \n"
        // process uv if (h % 2 == 1)
        "AND    x19, %[crop_height], #1                     \n"
        "MOV    x20, %[tar_y_addr]                          \n"
        "MOV    x21, %[tar_uv_addr]                         \n"

        "MOV    x22, #0                                     \n"
        "mosaic_loop_w:"
        // proccess y
        "MOV    x24, x20                                    \n"
        "LDRB   w23, [x20], #16                             \n"
        "DUP    v0.16B, w23                                 \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"
        "ST1   {v0.16B}, [x24], %[image_width]              \n"

        // proccess uv
        "MOV    x24, x21                                    \n"
        "LDRH   w23, [x21], #16                             \n"
        "DUP    v1.8H, w23                                  \n"
        "ST1   {v1.8H}, [x24], %[image_width]               \n"
        "ST1   {v1.8H}, [x24], %[image_width]               \n"
        "ST1   {v1.8H}, [x24], %[image_width]               \n"
        "ST1   {v1.8H}, [x24], %[image_width]               \n"
        "ST1   {v1.8H}, [x24], %[image_width]               \n"
        "ST1   {v1.8H}, [x24], %[image_width]               \n"
        "ST1   {v1.8H}, [x24], %[image_width]               \n"
        "ST1   {v1.8H}, [x24], %[image_width]               \n"

        // neon process 16 bytes once
        "ADD    x22, x22, #16                               \n"
        "CMP    x22, %[crop_width]                          \n"
        "B.LT   mosaic_loop_w                               \n"
        "mosaic_loop_w_done:                                \n"

        // y addr add
        "MOV    x22, #16                                    \n"
        "MUL    x23, %[image_width], x22                    \n"
        "ADD    %[tar_y_addr], %[tar_y_addr], x23           \n"
        // uv addr add
        "MOV    x22, #8                                     \n"
        "MUL    x23, %[image_width], x22                    \n"
        "ADD    %[tar_uv_addr], %[tar_uv_addr], x23         \n"

        "SUB    %[crop_height], %[crop_height], #16         \n"
        "CMP    %[crop_height], #0                          \n"
        "B.GT   mosaic_loop_h                               \n"
        "mosaic_loop_h_done:                                \n"
        :
        : [tar_y_addr]"r"(tar_y_addr),
          [tar_uv_addr]"r"(tar_uv_addr),
          [image_width]"r"(proc_info->image_width),
          [osd_width]"r"(proc_info->width),
          [crop_width]"r"(crop_width),
          [crop_height]"r"(crop_height)
        : "cc", "memory", "v0", "v1", "x19", "x20", "x21", "x22", "x23", "x24"    // Clobber List
    );
    kernel_neon_end();

    if (__pa(tar_y_addr) != 0) {
        ion_dcache_flush(__pa(tar_y_addr),
            proc_info->image_width * crop_height);
    }
    if (__pa(tar_uv_addr) != 0) {
        ion_dcache_flush(__pa(tar_uv_addr),
            proc_info->image_width * crop_height / 2);
    }

    do_gettimeofday(&time_next);
    time_us = (((time_next.tv_sec * 1000 * 1000) + time_next.tv_usec) -
        ((time_now.tv_sec * 1000 * 1000) + time_now.tv_usec));

    vio_dbg("osd mosaic process, show:%d invert:%d level:%d, "
        "frame_id:%d buf_index:%d image w:%d h:%d, "
        "x:%d y:%d w:%d h:%d y_addr:%p uv_addr:%p cost %ldus\n",
        proc_info->show_en, proc_info->invert_en, proc_info->osd_level,
        proc_info->frame_id, proc_info->buffer_index,
        proc_info->image_width, proc_info->image_height,
        proc_info->start_x, proc_info->start_y, proc_info->width,
        proc_info->height, tar_y_addr, tar_uv_addr, time_us);

exit:
    osd_process_workfunc_done(proc_info);
    mutex_unlock(&proc_info->proc_mutex);
}

void osd_process_sta_workfunc(struct kthread_work *work)
{
    osd_process_info_t *proc_info = container_of(work, osd_process_info_t, work);
    uint8_t *y_addr, *y_addr_temp;
    uint32_t h, w;
    struct timeval time_now = { 0 };
    struct timeval time_next = { 0 };
    time_t time_us;
    uint32_t crop_width, crop_height;

    if ((proc_info->subdev != NULL) &&
        (proc_info->subdev->osd_dev->task_state >= OSD_TASK_REQUEST_STOP)) {
        return;
    }
    if (proc_info->tar_y_addr == NULL) {
        vio_err("osd process type:%d tar_addr y:%p error\n",
            proc_info->proc_type, proc_info->tar_y_addr);
        goto exit;
    }

    if (proc_info->image_height > (proc_info->start_y + proc_info->height)) {
        crop_height = proc_info->height;
    } else if (proc_info->image_height > proc_info->start_y) {
        crop_height = proc_info->image_height - proc_info->start_y;
    } else {
        crop_height = 0;
    }
    if (proc_info->image_width > (proc_info->start_x + proc_info->width)) {
        crop_width = proc_info->width;
    } else if (proc_info->image_width > proc_info->start_x) {
        crop_width = proc_info->image_width - proc_info->start_x;
    } else {
        crop_width = 0;
    }

    do_gettimeofday(&time_now);

    y_addr = proc_info->tar_y_addr +
        proc_info->start_y * proc_info->image_width + proc_info->start_x;

    memset(proc_info->sta_bin_value, 0x00, MAX_STA_BIN_NUM * sizeof(uint16_t));
    for (h = 0; h < crop_height; h++) {
        y_addr_temp = y_addr;
        for (w = 0; w < crop_width; w++) {
            if (*y_addr_temp > proc_info->sta_level[2]) {
                proc_info->sta_bin_value[3]++;
            } else if (*y_addr_temp > proc_info->sta_level[1]) {
                proc_info->sta_bin_value[2]++;
            } else if (*y_addr_temp > proc_info->sta_level[0]) {
                proc_info->sta_bin_value[1]++;
            } else {
                proc_info->sta_bin_value[0]++;
            }
            y_addr_temp++;
        }
        y_addr += proc_info->image_width;
    }

    do_gettimeofday(&time_next);
    time_us = (((time_next.tv_sec * 1000 * 1000) + time_next.tv_usec) -
        ((time_now.tv_sec * 1000 * 1000) + time_now.tv_usec));

    vio_dbg("osd sta process, x:%d y:%d w:%d h:%d y_addr:%p "
        "sta_level: %d %d %d bin_value: %d %d %d %d, cost %ldus\n",
        proc_info->start_x, proc_info->start_y, proc_info->width,
        proc_info->height, y_addr,
        proc_info->sta_level[0], proc_info->sta_level[1],
        proc_info->sta_level[2], proc_info->sta_bin_value[0],
        proc_info->sta_bin_value[1], proc_info->sta_bin_value[2],
        proc_info->sta_bin_value[3], time_us);

exit:
    osd_process_workfunc_done(proc_info);
}

void osd_process_null_workfunc(struct kthread_work *work)
{
    osd_process_info_t *proc_info = container_of(work, osd_process_info_t, work);

    mutex_lock(&proc_info->proc_mutex);
    vio_warn("osd process null work, proc_type:%d\n", proc_info->proc_type);

    osd_process_workfunc_done(proc_info);
    mutex_unlock(&proc_info->proc_mutex);
}
