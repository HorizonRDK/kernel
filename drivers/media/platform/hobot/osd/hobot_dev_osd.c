/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#define pr_fmt(fmt) "hobot_dev_osd: " fmt
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/bug.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <asm/atomic.h>

#include "hobot_dev_osd.h"

#define MODULE_NAME "X3 osd"
#define OSD_MAX_DEVICE 1

osd_color_map_t g_osd_color = {
    .color_map_update = 0,
    .color_map = {
        0xff8080, 0x008080, 0x968080, 0x8DC01B, 0x952B15, 0xE10094, 0x545BA7,
        0x9E26C4, 0x34AAB5, 0xD47A9E, 0x4C54FF, 0xD06057, 0x0FC574, 0x3A5E56,
        0x2968C5
    }
};

int g_osd_fps[VIO_MAX_STREAM][VIO_MAX_STREAM] = {0, };
int g_osd_idx[VIO_MAX_STREAM][VIO_MAX_STREAM] = {0, };
long int g_osd_fps_lasttime[VIO_MAX_STREAM][VIO_MAX_STREAM] = {0, };

extern struct ion_device *hb_ion_dev;
extern void* osd_get_info(int32_t instance, int32_t chn);
extern void osd_send_callback(osd_send_frame_callback func);

struct x3_osd_dev *g_osd_dev = NULL;

static int osd_start_worker(struct x3_osd_dev *osd_dev)
{
    int ret = 0;
    struct sched_param param = {0};

    mutex_lock(&osd_dev->osd_mutex);
    osd_dev->task = kthread_run(kthread_worker_fn, &osd_dev->worker, "osd_work");
    if (IS_ERR(osd_dev->task)) {
        mutex_unlock(&osd_dev->osd_mutex);
        vio_err("failed to create buffer task, err(%ld)\n", PTR_ERR(osd_dev->task));
        ret = (int32_t)PTR_ERR(osd_dev->task);
        goto exit;
    }
    param.sched_priority = OSD_TASK_PRIORITY;
    ret = sched_setscheduler_nocheck(osd_dev->task, SCHED_FIFO, &param);
    if (ret) {
        mutex_unlock(&osd_dev->osd_mutex);
        vio_err("sched_setscheduler_nocheck is fail(%d)", ret);
        goto exit;
    }
    osd_dev->task_state = OSD_TASK_START;
    mutex_unlock(&osd_dev->osd_mutex);

    return ret;

exit:
    osd_dev->task = NULL;
    return ret;
}

static void osd_stop_worker(struct x3_osd_dev *osd_dev)
{
    struct task_struct *task;

    mutex_lock(&osd_dev->osd_mutex);
    osd_dev->task_state = OSD_TASK_REQUEST_STOP;
    task = osd_dev->task;
    osd_dev->task = NULL;
    kthread_flush_worker(&osd_dev->worker);
    kthread_stop(task);
    osd_dev->task = NULL;
    osd_dev->task_state = OSD_TASK_STOP;
    mutex_unlock(&osd_dev->osd_mutex);
}

static int x3_osd_open(struct inode *inode, struct file *file)
{
    struct osd_video_ctx *osd_ctx;
    struct x3_osd_dev *osd_dev;
    int ret = 0;

    osd_dev = container_of(inode->i_cdev, struct x3_osd_dev, cdev);
    osd_ctx = kzalloc(sizeof(struct osd_video_ctx), GFP_ATOMIC);
    if (osd_ctx == NULL) {
        vio_err("osd kzalloc failed\n");
        ret = -ENOMEM;
        goto exit;
    }

    osd_ctx->osd_dev = osd_dev;
    file->private_data = osd_ctx;

    if (atomic_inc_return(&osd_dev->open_cnt) == 1) {
        ret = osd_start_worker(osd_dev);
        if (ret < 0) {
            goto exit_free;
        }
    }

    vio_info("osd open node, open count:%d\n", atomic_read(&osd_dev->open_cnt));

    return ret;
exit_free:
    kfree(osd_ctx);
    atomic_dec(&osd_dev->open_cnt);
    osd_dev->task = NULL;
exit:
    return ret;
}

static ssize_t x3_osd_write(struct file *file, const char __user * buf,
                 size_t count, loff_t * ppos)
{
    return 0;
}

static ssize_t x3_osd_read(struct file *file, char __user * buf, size_t size,
                loff_t * ppos)
{
    return 0;
}

static void osd_subdev_clear(struct x3_osd_dev *osd_dev)
{
    int32_t i = 0, j = 0, m = 0;
    struct osd_subdev *subdev;
    osd_bind_t *bind;
    struct vio_frame *frame;
	unsigned long flags;

    for (i = 0; i < VIO_MAX_STREAM; i++) {
        for (j = 0; j < OSD_CHN_MAX; j++) {
            subdev = &osd_dev->subdev[i][j];

            if (subdev->osd_info != NULL) {
                atomic_set(&subdev->osd_info->need_sw_osd, 0);
                atomic_set(&subdev->osd_info->frame_count, 0);
            }

            if (subdev->osd_hw_cfg != NULL) {
                spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
                for (m = 0; m < OSD_HW_PROC_NUM; m++) {
                    memset(&subdev->osd_hw_cfg->osd_box[m], 0, sizeof(osd_box_t));
                    if (subdev->osd_hw_cfg->osd_buf[m] != 0) {
                        osd_one_buffer_dec_by_paddr(subdev->osd_hw_cfg->osd_buf[m]);
                        subdev->osd_hw_cfg->osd_buf[m] = 0;
                    }
                }
                for (m = 0; m < MAX_STA_NUM; m++) {
                    memset(&subdev->osd_hw_cfg->osd_sta[m], 0, sizeof(osd_sta_box_t));
                }
                for (m = 0; m < MAX_OSD_STA_LEVEL_NUM; m++) {
                    subdev->osd_hw_cfg->osd_sta_level[m] = 0;
                }
                subdev->osd_hw_cfg->osd_box_update = 1;
                subdev->osd_hw_cfg->osd_buf_update = 1;
                subdev->osd_hw_cfg->osd_sta_update = 1;
                subdev->osd_hw_cfg->osd_sta_level_update = 1;
                subdev->osd_hw_limit_y = 0;
                atomic_set(&subdev->osd_hw_cnt, 0);
                atomic_set(&subdev->osd_hw_need_update, 0);
                spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
            }

            mutex_lock(&subdev->bind_mutex);
            while (!list_empty(&subdev->bind_list)) {
                bind = list_first_entry(&subdev->bind_list, osd_bind_t, node);
                list_del(&bind->node);
                if (bind->bind_info.polygon_buf != NULL) {
                    kfree(bind->bind_info.polygon_buf);
                    bind->bind_info.polygon_buf = NULL;
                }
                kfree(bind);
            }
            mutex_unlock(&subdev->bind_mutex);

            mutex_lock(&subdev->sta_mutex);
            memset(&subdev->osd_sta, 0, sizeof(osd_sta_t));
            mutex_unlock(&subdev->sta_mutex);

            spin_lock_irqsave(&subdev->frame_slock, flags);
            while (!list_empty(&subdev->input_frame_list)) {
                frame = osd_get_input_frame(&subdev->input_frame_list);
                subdev->osd_info->return_frame(subdev->osd_info, frame);
            }
            spin_unlock_irqrestore(&subdev->frame_slock, flags);

            g_osd_fps[i][j] = 0;
        }
    }
}

static void osd_handle_clear(struct x3_osd_dev *osd_dev)
{
    osd_handle_t *handle;

    mutex_lock(&osd_dev->osd_list_mutex);
    while (!list_empty(&osd_dev->osd_list)) {
        handle = list_first_entry(&osd_dev->osd_list, osd_handle_t, node);
        list_del(&handle->node);

        osd_buffer_destroy(osd_dev->ion_client, &handle->buffer);
        kfree(handle);
    }
    mutex_unlock(&osd_dev->osd_list_mutex);
}

static int x3_osd_close(struct inode *inode, struct file *file)
{
    struct osd_video_ctx *osd_ctx;
    struct x3_osd_dev *osd_dev;
    int ret = 0;

    osd_ctx = file->private_data;
    if (osd_ctx == NULL) {
        vio_err("osd ctx was null\n");
        goto exit;
    }

    osd_dev = osd_ctx->osd_dev;

    if (atomic_dec_return(&osd_dev->open_cnt) == 0) {
        osd_stop_worker(osd_dev);

        osd_subdev_clear(osd_dev);
        osd_handle_clear(osd_dev);
    }

    if (osd_ctx) {
        kfree(osd_ctx);
        osd_ctx = NULL;
    }

    vio_info("osd close node, open count:%d\n", atomic_read(&osd_dev->open_cnt));

exit:
    return ret;
}

static void osd_run_process(osd_process_info_t *process_info)
{
    switch (process_info->proc_type)
    {
    case OSD_PROC_VGA4:
        osd_process_vga4_workfunc(process_info);
        break;
    case OSD_PROC_NV12:
        osd_process_nv12_workfunc(process_info);
        break;
    case OSD_PROC_RECT:
        osd_process_rect_workfunc(process_info);
        break;
    case OSD_PROC_POLYGON:
        osd_process_polygon_workfunc(process_info);
        break;
    case OSD_PROC_MOSAIC:
        osd_process_mosaic_workfunc(process_info);
        break;
    case OSD_PROC_STA:
        osd_process_sta_workfunc(process_info);
        break;
    default:
        osd_process_null_workfunc(process_info);
        break;
    }
}

static void osd_frame_process(struct vio_osd_info *osd_info, struct vio_frame *frame)
{
    struct vio_framemgr *framemgr;
	unsigned long flags;
    struct osd_subdev *osd_subdev;
    struct x3_osd_dev *osd_dev;
    struct ipu_subdev *ipu_subdev;
    struct pym_subdev *pym_subdev;
    uint32_t instance;

    osd_dev = g_osd_dev;
    if (osd_info->id == OSD_PYM_OUT) {
        pym_subdev = container_of(osd_info, struct pym_subdev, osd_info);
	    framemgr = &pym_subdev->framemgr;
        instance = pym_subdev->group->instance;
    } else {
        ipu_subdev = container_of(osd_info, struct ipu_subdev, osd_info);
	    framemgr = &ipu_subdev->framemgr;
        instance = ipu_subdev->group->instance;
    }
    osd_subdev = &osd_dev->subdev[instance][osd_info->id];
    if (osd_dev->task_state >= OSD_TASK_REQUEST_STOP) {
        vio_dbg("osd task request stop, exit\n");
        goto exit;
    }

    if (frame == NULL) {
        framemgr_e_barrier_irqs(framemgr, 0UL, flags);
        frame = get_frame(framemgr, FS_PROCESS);
        framemgr_x_barrier_irqr(framemgr, 0, flags);
    } else {
        // already lock, be called when ipu qbuf
        frame = osd_get_frame(&framemgr->queued_list[frame->state],
            frame->frameinfo.bufferindex);
        framemgr->queued_count[frame->state]--;
    }
    if (frame != NULL) {
        spin_lock_irqsave(&osd_subdev->frame_slock, flags);
        osd_put_input_frame(&osd_subdev->input_frame_list, frame);
        spin_unlock_irqrestore(&osd_subdev->frame_slock, flags);

        atomic_inc(&osd_subdev->osd_info->frame_count);
        kthread_queue_work(&osd_dev->worker, &osd_subdev->work);
    } else {
        vio_warn("ipu/pym call osd, but frame was null\n");
        goto exit;
    }
    return;

exit:
    osd_subdev->osd_info->return_frame(osd_subdev->osd_info, frame);
}

static void osd_print_handle_info(osd_handle_info_t *handle_info)
{
    vio_dbg("[H%d]: fill_color:%d yuv_bg_transparent:%d proc_type:%d"
        " size:%dx%d attach_pym:%d\n",
        handle_info->handle_id, handle_info->fill_color,
        handle_info->yuv_bg_transparent, handle_info->proc_type,
        handle_info->size.w, handle_info->size.h,
        handle_info->attach_pym);
}

static void osd_print_bind_info(osd_bind_info_t *bind_info)
{
    vio_dbg("[S%d][V%d][H%d]: show:%d invert:%d level:%d buf_layer:%d start: (%d, %d)\n",
        bind_info->instance, bind_info->chn, bind_info->handle_id,
        bind_info->show_en, bind_info->invert_en, bind_info->osd_level,
        bind_info->buf_layer, bind_info->start_point.x, bind_info->start_point.y);
    vio_dbg("polygon: side num:%d point:(%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d) "
        "(%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d) buffer:%p\n",
        bind_info->side_num, bind_info->point[0].x, bind_info->point[0].y,
        bind_info->point[1].x, bind_info->point[1].y,
        bind_info->point[2].x, bind_info->point[2].y,
        bind_info->point[3].x, bind_info->point[3].y,
        bind_info->point[4].x, bind_info->point[4].y,
        bind_info->point[5].x, bind_info->point[5].y,
        bind_info->point[6].x, bind_info->point[6].y,
        bind_info->point[7].x, bind_info->point[7].y,
        bind_info->point[8].x, bind_info->point[8].y,
        bind_info->point[9].x, bind_info->point[9].y, bind_info->polygon_buf);
    osd_print_handle_info(&bind_info->handle_info);
}

static int32_t osd_handle_check(osd_handle_info_t *handle_info)
{
    if ((handle_info->handle_id < 0) ||
        (handle_info->handle_id >= OSD_HANDLE_MAX)) {
        vio_err("osd handle id:%d error\n", handle_info->handle_id);
        return -EINVAL;
    }

    if (handle_info->proc_type >= OSD_PROC_MAX_TYPE) {
        vio_err("osd handle:%d proc_type:%d error\n",
            handle_info->handle_id, handle_info->proc_type);
        return -EINVAL;
    }

    return 0;
}

static osd_handle_t *osd_find_handle_node(struct x3_osd_dev *osd_dev, int32_t id)
{
    osd_handle_t *handle = NULL, *temp;

    if (!list_empty(&osd_dev->osd_list)) {
        list_for_each_entry_safe(handle, temp, &osd_dev->osd_list, node) {
            if (handle->info.handle_id == id) {
                return handle;
            }
        }
    }
    return NULL;
}

static osd_bind_t *osd_find_bind_node(struct osd_subdev *osd_subdev, int32_t id,
    int32_t buf_layer)
{
    osd_bind_t *bind = NULL, *temp;

    if (!list_empty(&osd_subdev->bind_list)) {
        list_for_each_entry_safe(bind, temp, &osd_subdev->bind_list, node) {
            if (bind->bind_info.handle_id == id) {
                if ((osd_subdev->id == OSD_PYM_OUT) &&
                    (bind->bind_info.buf_layer != buf_layer)) {
                    continue;
                }
                return bind;
            }
        }
    }
    return NULL;
}

static int32_t handle_find_buffer(osd_handle_t *handle, osd_buf_state_t state,
    osd_one_buffer_t **one_buf, uint8_t is_vga)
{
    int32_t i = 0;

    for (i = 0; i < OSD_PING_PONG_BUF; i++) {
        if (is_vga == 0) {
            if (handle->buffer.buf[i].state == state) {
                *one_buf = &handle->buffer.buf[i];
                return i;
            }
        } else {
            if (handle->buffer.vga_buf[i].state == state) {
                *one_buf = &handle->buffer.vga_buf[i];
                return i;
            }
        }
    }
    return -1;
}

static void osd_hw_set_roi_addr_config(struct osd_subdev *subdev)
{
    osd_bind_t *bind = NULL, *temp;
    uint32_t hw_cnt = 0;
    struct ipu_osd_cfg  *osd_hw_cfg;

    if ((subdev->id >= OSD_IPU_DS2) || (subdev->osd_hw_cfg == NULL)) {
        return;
    }

    spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
    osd_hw_cfg = subdev->osd_hw_cfg;
    list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
        if (hw_cnt >= OSD_HW_PROC_NUM) {
            break;
        }
        if (bind->proc_info.proc_type == OSD_PROC_HW_VGA4) {
            osd_hw_cfg->osd_box[hw_cnt].osd_en = bind->proc_info.show_en;
            osd_hw_cfg->osd_box[hw_cnt].overlay_mode = bind->proc_info.invert_en;
            osd_hw_cfg->osd_box[hw_cnt].start_x = (uint16_t)bind->proc_info.start_x;
            osd_hw_cfg->osd_box[hw_cnt].start_y = (uint16_t)bind->proc_info.start_y;
            osd_hw_cfg->osd_box[hw_cnt].width = (uint16_t)bind->proc_info.width;
            osd_hw_cfg->osd_box[hw_cnt].height = (uint16_t)bind->proc_info.height;

            if (osd_hw_cfg->osd_buf[hw_cnt] != 0) {
                osd_one_buffer_dec_by_paddr(osd_hw_cfg->osd_buf[hw_cnt]);
            }
            osd_hw_cfg->osd_buf[hw_cnt] = (uint32_t)bind->proc_info.src_paddr;
            osd_one_buffer_inc_by_paddr(osd_hw_cfg->osd_buf[hw_cnt]);
            vio_dbg("[V%d][%d]osd hw set osd_en:%d mode:%d "
                "x:%d y:%d w:%d h:%d paddr:0x%x \n",
                subdev->id, hw_cnt, osd_hw_cfg->osd_box[hw_cnt].osd_en,
                osd_hw_cfg->osd_box[hw_cnt].overlay_mode,
                osd_hw_cfg->osd_box[hw_cnt].start_x,
                osd_hw_cfg->osd_box[hw_cnt].start_y,
                osd_hw_cfg->osd_box[hw_cnt].width,
                osd_hw_cfg->osd_box[hw_cnt].height,
                osd_hw_cfg->osd_buf[hw_cnt]);
            hw_cnt++;
        }
    }

    if (hw_cnt > 0) {
        subdev->osd_hw_limit_y = osd_hw_cfg->osd_box[hw_cnt - 1].start_y +
            osd_hw_cfg->osd_box[hw_cnt - 1].height;
    } else {
        subdev->osd_hw_limit_y = 0;
    }
    for (; hw_cnt < OSD_HW_PROC_NUM; hw_cnt++) {
        memset(&osd_hw_cfg->osd_box[hw_cnt], 0, sizeof(osd_box_t));

        if (osd_hw_cfg->osd_buf[hw_cnt] != 0) {
            osd_one_buffer_dec_by_paddr(osd_hw_cfg->osd_buf[hw_cnt]);
        }
        osd_hw_cfg->osd_buf[hw_cnt] = 0;
    }
    osd_hw_cfg->osd_box_update = 1;
    osd_hw_cfg->osd_buf_update = 1;
    atomic_set(&subdev->osd_hw_need_update, 0);
    spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
}

static void osd_hw_set_sta_config(struct osd_subdev *subdev)
{
    struct ipu_osd_cfg  *osd_hw_cfg;

    if ((subdev->id >= OSD_IPU_DS2) || (subdev->osd_hw_cfg == NULL)) {
        return;
    }

    spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
    osd_hw_cfg = subdev->osd_hw_cfg;
    memcpy(osd_hw_cfg->osd_sta, subdev->osd_sta.sta_box,
        MAX_STA_NUM * sizeof(osd_sta_box_t));
    osd_hw_cfg->osd_sta_update = 1;
    memcpy(osd_hw_cfg->osd_sta_level, subdev->osd_sta.sta_level,
        MAX_OSD_STA_LEVEL_NUM * sizeof(uint8_t));
    osd_hw_cfg->osd_sta_level_update = 1;
    spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
}

static int osd_hw_get_sta_bin(struct osd_subdev *subdev)
{
    struct ipu_subdev *ipu_subdev;

    if ((subdev->id >= OSD_IPU_DS2) || (subdev->osd_hw_cfg == NULL)) {
        return -EINVAL;
    }

    ipu_subdev = container_of(subdev->osd_hw_cfg, struct ipu_subdev, osd_cfg);

    return osd_get_sta_bin(ipu_subdev,
        (uint16_t (*)[MAX_STA_BIN_NUM])subdev->osd_sta.sta_value);
}

static void osd_hw_set_color_map(struct osd_subdev *subdev)
{
    struct ipu_osd_cfg  *osd_hw_cfg;

    if ((subdev->id >= OSD_IPU_DS2) || (subdev->osd_hw_cfg == NULL)) {
        return;
    }

    spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
    osd_hw_cfg = subdev->osd_hw_cfg;
    memcpy(osd_hw_cfg->color_map.color_map, g_osd_color.color_map,
        MAX_OSD_COLOR_NUM * sizeof(uint32_t));
    osd_hw_cfg->color_map.color_map_update = 1;
    g_osd_color.color_map_update = 0;
    spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
}

static void osd_sw_set_process_flag(struct osd_subdev *subdev)
{
    osd_bind_t *bind = NULL, *temp;
    struct vio_osd_info *osd_info;

    osd_info = subdev->osd_info;
    list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
        if (bind->proc_info.proc_type != OSD_PROC_HW_VGA4) {
            atomic_set(&osd_info->need_sw_osd, 1);
            return;
        }
    }
    atomic_set(&osd_info->need_sw_osd, 0);
}

static void osd_set_process_handle_info(osd_process_info_t *process_info,
    osd_handle_t *handle)
{
    osd_one_buffer_t *one_buf;
    int index;

    if (process_info->proc_type <= OSD_PROC_NV12) {
        // rect, polygon, mosaic info will be in bind_info
        process_info->fill_color = handle->info.fill_color;
        process_info->width = handle->info.size.w;
        process_info->height = handle->info.size.h;
    }
    switch (process_info->proc_type)
    {
    case OSD_PROC_HW_VGA4:
        if (process_info->subdev != NULL) {
            atomic_set(&process_info->subdev->osd_hw_need_update, 1);
        }
    case OSD_PROC_NV12:
        index = handle_find_buffer(handle, OSD_BUF_PROCESS, &one_buf, 0);
        if (index >= 0) {
            process_info->src_addr = one_buf->vaddr;
            process_info->src_paddr = one_buf->paddr;
        } else {
            vio_err("[H%d] find process buffer failed\n",
                handle->info.handle_id);
        }
        if (process_info->proc_type == OSD_PROC_NV12) {
            process_info->yuv_bg_transparent = handle->info.yuv_bg_transparent;
        }
        break;
    case OSD_PROC_VGA4:
        index = handle_find_buffer(handle, OSD_BUF_PROCESS, &one_buf, 1);
        if (index >= 0) {
            process_info->src_vga_addr = one_buf->vaddr;
        } else {
            vio_err("[H%d] find process buffer failed\n",
                handle->info.handle_id);
        }
        break;
    default:
        break;
    }
}

static void osd_set_process_bind_info(osd_process_info_t *process_info,
    osd_bind_info_t *bind_info)
{
    process_info->show_en = bind_info->show_en;
    process_info->invert_en = bind_info->invert_en;
    process_info->osd_level = bind_info->osd_level;
    process_info->start_x = bind_info->start_point.x;
    process_info->start_y = bind_info->start_point.y;
    process_info->buf_layer = bind_info->buf_layer;

    if (process_info->proc_type >= OSD_PROC_RECT) {
        process_info->fill_color = bind_info->handle_info.fill_color;
        process_info->width = bind_info->handle_info.size.w;
        process_info->height = bind_info->handle_info.size.h;
        process_info->polygon_buf = bind_info->polygon_buf;
    }
    if (process_info->proc_type == OSD_PROC_HW_VGA4) {
        atomic_set(&process_info->subdev->osd_hw_need_update, 1);
    }
}

static void osd_process_set_pym_addr(osd_process_info_t *proc_info,
    struct mp_vio_frame *frame, struct pym_subdev *pym_subdev)
{
    uint32_t buf_layer;
    struct special_buffer *pym_spec;

    buf_layer = proc_info->buf_layer;
    pym_spec = &frame->common_frame.frameinfo.spec;
    if (buf_layer < MAX_PYM_DS_COUNT) {
        proc_info->image_width =
            ALIGN(pym_subdev->pym_cfg.stds_box[buf_layer].tgt_width, 16);
        proc_info->image_height = pym_subdev->pym_cfg.stds_box[buf_layer].tgt_height;

        if ((buf_layer == 0) &&
            (test_bit(PYM_DMA_INPUT, &pym_subdev->pym_dev->state))) {
            proc_info->tar_y_addr = __va(frame->common_frame.frameinfo.addr[0]);
            proc_info->tar_uv_addr = __va(frame->common_frame.frameinfo.addr[1]);
        } else {
            proc_info->tar_y_addr = __va(pym_spec->ds_y_addr[buf_layer]);
            proc_info->tar_uv_addr = __va(pym_spec->ds_uv_addr[buf_layer]);
        }
    } else {
        buf_layer = buf_layer - MAX_PYM_DS_COUNT;
        proc_info->image_width =
            ALIGN(pym_subdev->pym_cfg.stus_box[buf_layer].tgt_width, 16);
        proc_info->image_height = pym_subdev->pym_cfg.stus_box[buf_layer].tgt_height;

        proc_info->tar_y_addr = __va(pym_spec->us_y_addr[buf_layer]);
        proc_info->tar_uv_addr = __va(pym_spec->us_uv_addr[buf_layer]);
    }
}

static void osd_process_addr_inc(osd_process_info_t *proc_info)
{
    switch (proc_info->proc_type)
    {
    case OSD_PROC_VGA4:
        osd_one_buffer_inc_by_addr(proc_info->src_vga_addr);
        break;
    case OSD_PROC_NV12:
        osd_one_buffer_inc_by_addr(proc_info->src_addr);
        break;
    default:
        break;
    }
}

static void osd_process_addr_dec(osd_process_info_t *proc_info)
{
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
}

static void osd_set_process_info_workfunc(struct kthread_work *work)
{
    int32_t i, j;
    osd_bind_t *bind = NULL, *temp;
    struct osd_subdev *subdev;
    struct x3_osd_dev *osd_dev;
    int32_t handle_id;
    osd_handle_t *handle;

    osd_dev = container_of(work, struct x3_osd_dev, work);

    for (i = 0; i < VIO_MAX_STREAM; i++) {
        for (j = 0; j < OSD_CHN_MAX; j++) {
            subdev = &osd_dev->subdev[i][j];
            if (g_osd_color.color_map_update == 1) {
                osd_hw_set_color_map(subdev);
                g_osd_color.color_map_update = 0;
            }
            mutex_lock(&subdev->bind_mutex);
            list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
                mutex_lock(&bind->proc_info.proc_mutex);
                handle_id = bind->bind_info.handle_id;
                mutex_lock(&osd_dev->osd_list_mutex);
                handle = osd_find_handle_node(osd_dev, handle_id);
                if (handle == NULL) {
                    vio_err("[V%d][H%d] attached, but handle was destroyed!\n",
                        subdev->id, handle_id);
                } else {
                    osd_set_process_handle_info(&bind->proc_info, handle);
                }
                mutex_unlock(&osd_dev->osd_list_mutex);

                if (atomic_read(&bind->need_update) > 0) {
                    osd_set_process_bind_info(&bind->proc_info, &bind->bind_info);
                    atomic_set(&bind->need_update, 0);
                }

                mutex_unlock(&bind->proc_info.proc_mutex);
            }
            if (atomic_read(&subdev->osd_hw_need_update) > 0) {
                osd_hw_set_roi_addr_config(subdev);
            }
            mutex_unlock(&subdev->bind_mutex);
        }
    }
}

static void osd_process_workfunc_done(struct osd_subdev *subdev)
{
    struct ipu_subdev *ipu_subdev;
    struct pym_subdev *pym_subdev;
    struct x3_osd_dev *osd_dev;
    int32_t instance;
    struct timeval tmp_tv;

    osd_dev = subdev->osd_dev;
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

    if (atomic_dec_return(&subdev->osd_info->frame_count) > 0) {
        kthread_queue_work(&osd_dev->worker, &subdev->work);
    }
    vio_dbg("[S%d][V%d] %s\n", instance, subdev->id, __func__);
}

static void osd_frame_process_workfunc(struct kthread_work *work)
{
    int32_t i;
    osd_bind_t *bind = NULL, *temp;
    struct osd_subdev *subdev;
    struct x3_osd_dev *osd_dev;
    struct pym_subdev *pym_subdev = NULL;
    struct ipu_subdev *ipu_subdev = NULL;
    struct mp_vio_frame *frame;
    osd_process_info_t *process_info;
	unsigned long flags;
    int32_t instance;

    subdev = container_of(work, struct osd_subdev, work);
    osd_dev = subdev->osd_dev;
    if (subdev->id == OSD_PYM_OUT) {
        pym_subdev = container_of(subdev->osd_info, struct pym_subdev, osd_info);
        instance = pym_subdev->group->instance;
    } else {
        ipu_subdev = container_of(subdev->osd_info, struct ipu_subdev, osd_info);
        instance = ipu_subdev->group->instance;
    }

    spin_lock_irqsave(&subdev->frame_slock, flags);
    frame = (struct mp_vio_frame *)osd_get_input_frame(&subdev->input_frame_list);
    spin_unlock_irqrestore(&subdev->frame_slock, flags);
    if (frame == NULL) {
        vio_err("osd input frame was null!\n");
        goto exit;
    }
    vio_dbg("[S%d][V%d] %s, frame:%d index:%d\n",
        instance, subdev->id, __func__,
        frame->common_frame.frameinfo.frame_id,
        frame->common_frame.frameinfo.bufferindex);

    mutex_lock(&subdev->sta_mutex);
    if (subdev->osd_sta.sta_state == OSD_STA_REQUEST) {
        if (subdev->id >= OSD_IPU_DS2) {
            subdev->osd_sta.sta_state = OSD_STA_PROCESS;
            for (i = 0; i < MAX_STA_NUM; i++) {
                if (osd_dev->task_state >= OSD_TASK_REQUEST_STOP) {
                    vio_dbg("osd task request stop, exit\n");
                    break;
                }
                if (subdev->osd_sta.sta_box[i].sta_en) {
                    // it means sw process sta
                    process_info = &subdev->osd_sta.sta_proc[i];
                    process_info->subdev = subdev;
                    process_info->width = subdev->osd_sta.sta_box[i].width;
                    process_info->height = subdev->osd_sta.sta_box[i].height;
                    process_info->start_x = subdev->osd_sta.sta_box[i].start_x;
                    process_info->start_y = subdev->osd_sta.sta_box[i].start_y;
                    process_info->sta_level = subdev->osd_sta.sta_level;
                    process_info->sta_bin_value = (uint16_t *)subdev->osd_sta.sta_value[i];

                    process_info->frame_id = frame->common_frame.frameinfo.frame_id;
                    process_info->buffer_index = frame->common_frame.frameinfo.bufferindex;
                    if (subdev->id == OSD_PYM_OUT) {
                        osd_process_set_pym_addr(process_info, frame, pym_subdev);
                    } else {
                        process_info->image_width = frame->common_frame.frameinfo.width;
                        process_info->image_height = frame->common_frame.frameinfo.height;
                        process_info->tar_y_addr = __va(frame->common_frame.frameinfo.addr[0]);
                        process_info->tar_uv_addr = __va(frame->common_frame.frameinfo.addr[1]);
                    }
                    process_info->proc_type = OSD_PROC_STA;

                    osd_run_process(process_info);
                }
            }
            subdev->osd_sta.sta_state = OSD_STA_DONE;
        }
        memset(subdev->osd_sta.sta_box, 0, MAX_STA_NUM * sizeof(osd_sta_box_t));
    }
    mutex_unlock(&subdev->sta_mutex);

    mutex_lock(&subdev->bind_mutex);
    for (i = 0; i < OSD_LEVEL_NUM; i++) {
        if (osd_dev->task_state >= OSD_TASK_REQUEST_STOP) {
            vio_dbg("osd task request stop, exit\n");
            break;
        }
        list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
            mutex_lock(&bind->proc_info.proc_mutex);
            if ((bind->proc_info.show_en == 0) || (bind->proc_info.osd_level != i)) {
                mutex_unlock(&bind->proc_info.proc_mutex);
                continue;
            }

            bind->proc_info.frame_id = frame->common_frame.frameinfo.frame_id;
            bind->proc_info.buffer_index = frame->common_frame.frameinfo.bufferindex;
            if (subdev->id == OSD_PYM_OUT) {
                osd_process_set_pym_addr(&bind->proc_info, frame, pym_subdev);
            } else {
                bind->proc_info.image_width = frame->common_frame.frameinfo.width;
                bind->proc_info.image_height = frame->common_frame.frameinfo.height;
                bind->proc_info.tar_y_addr = __va(frame->common_frame.frameinfo.addr[0]);
                bind->proc_info.tar_uv_addr = __va(frame->common_frame.frameinfo.addr[1]);
            }

            if (bind->proc_info.proc_type != OSD_PROC_HW_VGA4) {
                osd_process_addr_inc(&bind->proc_info);
                osd_run_process(&bind->proc_info);
                osd_process_addr_dec(&bind->proc_info);
            }
            mutex_unlock(&bind->proc_info.proc_mutex);
        }
    }

    mutex_unlock(&subdev->bind_mutex);

    osd_process_workfunc_done(subdev);

exit:
    subdev->osd_info->return_frame(subdev->osd_info, (struct vio_frame *)frame);
}

static void handle_update_buffer(osd_handle_t *handle, int32_t index)
{
    int32_t i = 0;

    for (i = 0; i < OSD_PING_PONG_BUF; i++) {
        if (handle->buffer.buf[i].state == OSD_BUF_PROCESS) {
            handle->buffer.buf[i].state = OSD_BUF_CREATE;
        }
        if (handle->buffer.vga_buf[i].state == OSD_BUF_PROCESS) {
            handle->buffer.vga_buf[i].state = OSD_BUF_CREATE;
        }
        if (i == index) {
            if (handle->buffer.buf[i].state != OSD_BUF_NULL) {
                handle->buffer.buf[i].state = OSD_BUF_PROCESS;
            }
            if (handle->buffer.vga_buf[i].state != OSD_BUF_NULL) {
                handle->buffer.vga_buf[i].state = OSD_BUF_PROCESS;
            }
            vio_dbg("[H%d] update buffer index:%d paddr:0x%llx vaddr:%p\n",
                handle->info.handle_id, i, handle->buffer.buf[i].paddr,
                handle->buffer.buf[i].vaddr);
        }
    }
}

static osd_process_type_e osd_hw_check_limit(struct osd_subdev *subdev,
    osd_handle_t *handle, osd_bind_t *bind)
{
    if ((subdev->id >= OSD_IPU_DS2) || (subdev->osd_hw_cfg == NULL) ||
        (bind->bind_info.show_en == 0) || (bind->bind_info.osd_level > 0)||
        (bind->bind_info.start_point.y < subdev->osd_hw_limit_y) ||
        (atomic_read(&subdev->osd_hw_cnt) >= OSD_HW_PROC_NUM)) {
        return OSD_PROC_VGA4;
    } else {
        subdev->osd_hw_limit_y = bind->bind_info.start_point.y + handle->info.size.h;
        atomic_inc(&subdev->osd_hw_cnt);
        return OSD_PROC_HW_VGA4;
    }
}

static int osd_polygon_check_is_change(osd_bind_info_t *old_info,
    osd_bind_info_t *new_info)
{
    int i;

    if (old_info->side_num != new_info->side_num) {
        return 1;
    }
    for (i = 0; i < old_info->side_num; i++) {
        if (old_info->point[i].x != new_info->point[i].x) {
            return 1;
        }
        if (old_info->point[i].y != new_info->point[i].y) {
            return 1;
        }
    }
    return 0;
}

static int osd_vga4_check_need_sw_process(osd_bind_info_t *old_info,
    osd_bind_info_t *new_info)
{
    if (new_info->show_en == 0) {
        return 1;
    }
    if (new_info->osd_level > 0) {
        return 1;
    }
    if (old_info->start_point.y != new_info->start_point.y) {
        return 1;
    }
    return 0;
}

static int osd_create_handle(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    osd_handle_t *handle = NULL, *tmp_handle;
    struct x3_osd_dev *osd_dev;

    osd_dev = osd_ctx->osd_dev;

    handle = kzalloc(sizeof(osd_handle_t), GFP_ATOMIC);
    if (handle == NULL) {
        vio_err("osd kzalloc failed\n");
        ret = -ENOMEM;
        goto exit;
    }

    ret = (int32_t)copy_from_user((void *) &handle->info, (void __user *) arg,
        sizeof(osd_handle_info_t));
    if (ret) {
        vio_err("%s copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit_free;
    }

    osd_print_handle_info(&handle->info);
    if (osd_handle_check(&handle->info) < 0) {
        ret = -EINVAL;
        goto exit_free;
    }

    if (handle->info.proc_type <= OSD_PROC_NV12) {
        handle->buffer.size.w = handle->info.size.w;
        handle->buffer.size.h = handle->info.size.h;
        handle->buffer.buf[0].pixel_fmt =
            (handle->info.proc_type == OSD_PROC_VGA4) ?
            OSD_PIXEL_FORMAT_VGA4 : OSD_PIXEL_FORMAT_NV12;
        handle->buffer.buf[1].pixel_fmt =
            (handle->info.proc_type == OSD_PROC_VGA4) ?
            OSD_PIXEL_FORMAT_VGA4 : OSD_PIXEL_FORMAT_NV12;
        ret = osd_buffer_create(osd_dev->ion_client, &handle->buffer);
        if (ret < 0) {
            vio_err("[H%d] osd alloc buffer size %dx%d failed\n",
                handle->info.handle_id, handle->buffer.size.w,
                handle->buffer.size.h);
            goto exit_free;
        }
        osd_one_buffer_fill(&handle->buffer.buf[0],
            (handle->info.proc_type == OSD_PROC_VGA4) ?
            handle->info.fill_color :
            g_osd_color.color_map[handle->info.fill_color]);
        osd_one_buffer_flush(&handle->buffer.buf[0]);
        osd_one_buffer_fill(&handle->buffer.buf[1],
            (handle->info.proc_type == OSD_PROC_VGA4) ?
            handle->info.fill_color :
            g_osd_color.color_map[handle->info.fill_color]);
        osd_one_buffer_flush(&handle->buffer.buf[1]);
    }

    atomic_set(&handle->bind_cnt, 0);
    atomic_set(&handle->ref_cnt, 1);
    mutex_lock(&osd_dev->osd_list_mutex);
    tmp_handle = osd_find_handle_node(osd_dev, handle->info.handle_id);
    if (tmp_handle != NULL) {
        if (tmp_handle->info.proc_type != handle->info.proc_type)  {
            vio_err("[H%d] %s proc_type:%d %d not match\n",
                tmp_handle->info.handle_id, __func__,
                tmp_handle->info.proc_type, handle->info.proc_type);
            ret = -EINVAL;
        } else {
            atomic_inc(&tmp_handle->ref_cnt);
            vio_info("[H%d] %s already create, count:%d\n",
                handle->info.handle_id, __func__,
                atomic_read(&tmp_handle->ref_cnt));
        }
        mutex_unlock(&osd_dev->osd_list_mutex);
        goto exit_destroy;
    }
    list_add_tail(&handle->node, &osd_dev->osd_list);
    mutex_unlock(&osd_dev->osd_list_mutex);

    vio_info("[H%d] %s done ret:%d\n",
        handle->info.handle_id, __func__, ret);

    return ret;

exit_destroy:
    if (handle != NULL) {
        osd_buffer_destroy(osd_dev->ion_client, &handle->buffer);
    }
exit_free:
    kfree(handle);
exit:
    return ret;
}

static int osd_destroy_handle(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    osd_handle_t *handle = NULL;
    struct x3_osd_dev *osd_dev;
    int32_t id = 0;

    osd_dev = osd_ctx->osd_dev;

    ret = get_user(id, (int32_t __user *) arg);
    if (ret) {
        vio_err("%s copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    // ensure update hw paddr work is done and osd_buffer_destroy can get newest buffer count
    kthread_flush_work(&osd_dev->work);

    mutex_lock(&osd_dev->osd_list_mutex);
    handle = osd_find_handle_node(osd_dev, id);
    if (handle == NULL) {
        mutex_unlock(&osd_dev->osd_list_mutex);
        vio_err("[H%d] %s handle was null\n", id, __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (atomic_read(&handle->ref_cnt) <= 1) {
        if (atomic_read(&handle->bind_cnt) > 0) {
            mutex_unlock(&osd_dev->osd_list_mutex);
            vio_err("[H%d] %s need detach first\n", id, __func__);
            ret = -EINVAL;
            goto exit;
        }

        list_del(&handle->node);
        osd_buffer_destroy(osd_dev->ion_client, &handle->buffer);
        kfree(handle);
    } else {
        atomic_dec(&handle->ref_cnt);
    }
    mutex_unlock(&osd_dev->osd_list_mutex);

    vio_info("[H%d] %s done ret:%d\n", id, __func__, ret);

exit:
    return ret;
}

static int osd_get_attr(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    osd_handle_info_t handle_info;
    osd_handle_t *handle = NULL;
    struct x3_osd_dev *osd_dev;

    osd_dev = osd_ctx->osd_dev;

    ret = (int32_t)copy_from_user((void *) &handle_info, (void __user *) arg,
        sizeof(osd_handle_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    mutex_lock(&osd_dev->osd_list_mutex);
    handle = osd_find_handle_node(osd_dev, handle_info.handle_id);
    if (handle == NULL) {
        mutex_unlock(&osd_dev->osd_list_mutex);
        vio_err("[H%d] %s handle was null\n", handle_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit;
    }
    ret = (int32_t)copy_to_user((void __user *) arg, (void *) &handle->info,
                sizeof(osd_handle_info_t));
    mutex_unlock(&osd_dev->osd_list_mutex);

    vio_dbg("[H%d] %s done ret:%d\n",
        handle_info.handle_id, __func__, ret);

exit:
    return ret;
}

static int osd_set_attr(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    osd_handle_info_t handle_info;
    osd_handle_t *handle = NULL, *tmp_handle;
    struct x3_osd_dev *osd_dev;
    int need_replace = 0;

    osd_dev = osd_ctx->osd_dev;

    ret = (int32_t)copy_from_user((void *) &handle_info, (void __user *) arg,
        sizeof(osd_handle_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }
    osd_print_handle_info(&handle_info);

    tmp_handle = kzalloc(sizeof(osd_handle_t), GFP_ATOMIC);
    if (tmp_handle == NULL) {
        vio_err("osd kzalloc failed\n");
        ret = -ENOMEM;
        goto exit;
    }

    mutex_lock(&osd_dev->osd_list_mutex);
    handle = osd_find_handle_node(osd_dev, handle_info.handle_id);
    if (handle == NULL) {
        mutex_unlock(&osd_dev->osd_list_mutex);
        vio_err("[H%d] %s handle was null\n", handle_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit_free;
    }

    memcpy(&tmp_handle->info, &handle->info, sizeof(osd_handle_info_t));
    atomic_set(&tmp_handle->bind_cnt, atomic_read(&handle->bind_cnt));
    mutex_unlock(&osd_dev->osd_list_mutex);

    if (tmp_handle->info.proc_type != handle_info.proc_type)  {
        vio_err("[%d] %s proc_type:%d %d not match\n",
            tmp_handle->info.handle_id, __func__,
            tmp_handle->info.proc_type, handle_info.proc_type);
        ret = -EINVAL;
        goto exit_free;
    }
    if (handle_info.proc_type <= OSD_PROC_NV12) {
        if ((tmp_handle->info.size.w != handle_info.size.w) ||
            (tmp_handle->info.size.h != handle_info.size.h)) {
            if (atomic_read(&tmp_handle->bind_cnt) == 0) {
                tmp_handle->info.proc_type = handle_info.proc_type;
                tmp_handle->info.size.w = handle_info.size.w;
                tmp_handle->info.size.h = handle_info.size.h;

                tmp_handle->buffer.size.w = handle_info.size.w;
                tmp_handle->buffer.size.h = handle_info.size.h;
                tmp_handle->buffer.buf[0].pixel_fmt =
                    (handle_info.proc_type == OSD_PROC_VGA4) ?
                    OSD_PIXEL_FORMAT_VGA4 : OSD_PIXEL_FORMAT_NV12;
                tmp_handle->buffer.buf[1].pixel_fmt =
                    (handle_info.proc_type == OSD_PROC_VGA4) ?
                    OSD_PIXEL_FORMAT_VGA4 : OSD_PIXEL_FORMAT_NV12;
                ret = osd_buffer_create(osd_dev->ion_client, &tmp_handle->buffer);
                if (ret < 0) {
                    goto exit_free;
                }
                osd_one_buffer_fill(&tmp_handle->buffer.buf[0],
                    (handle_info.proc_type == OSD_PROC_VGA4) ?
                    handle_info.fill_color :
                    g_osd_color.color_map[handle_info.fill_color]);
                osd_one_buffer_flush(&tmp_handle->buffer.buf[0]);
                osd_one_buffer_fill(&tmp_handle->buffer.buf[1],
                    (handle_info.proc_type == OSD_PROC_VGA4) ?
                    handle_info.fill_color :
                    g_osd_color.color_map[handle_info.fill_color]);
                osd_one_buffer_flush(&tmp_handle->buffer.buf[1]);
                need_replace = 1;
            } else {
                vio_err("[H%d] already bind, can`t modify size\n",
                    tmp_handle->info.handle_id);
                ret = -EINVAL;
                goto exit_free;
            }
        }
        tmp_handle->info.fill_color = handle_info.fill_color;
        tmp_handle->info.yuv_bg_transparent = handle_info.yuv_bg_transparent;
    }

    mutex_lock(&osd_dev->osd_list_mutex);
    handle = osd_find_handle_node(osd_dev, handle_info.handle_id);
    if (handle == NULL) {
        mutex_unlock(&osd_dev->osd_list_mutex);
        vio_err("[H%d] %s handle was null\n", handle_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit_buffer_destroy;
    }

    if (need_replace == 1) {
        list_replace(&handle->node, &tmp_handle->node);

        osd_buffer_destroy(osd_dev->ion_client, &handle->buffer);
        kfree(handle);
        handle = NULL;
    } else {
        memcpy(&handle->info, &tmp_handle->info, sizeof(osd_handle_info_t));
        if (atomic_read(&handle->bind_cnt) > 0) {
            kthread_queue_work(&osd_dev->worker, &osd_dev->work);
        }
        kfree(tmp_handle);
        tmp_handle = NULL;
    }
    mutex_unlock(&osd_dev->osd_list_mutex);

    vio_dbg("[H%d] %s done ret:%d\n",
        handle_info.handle_id, __func__, ret);

    return ret;
exit_buffer_destroy:
    if (tmp_handle != NULL) {
        osd_buffer_destroy(osd_dev->ion_client, &tmp_handle->buffer);
    }
exit_free:
    if (tmp_handle != NULL) {
        kfree(tmp_handle);
        tmp_handle = NULL;
    }
exit:
    return ret;
}

static int osd_get_buffer(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    osd_buffer_info_t buffer_info;
    osd_handle_t *handle = NULL;
    struct x3_osd_dev *osd_dev;
    osd_one_buffer_t *one_buf;

    osd_dev = osd_ctx->osd_dev;

    ret = (int32_t)copy_from_user((void *) &buffer_info, (void __user *) arg,
        sizeof(osd_buffer_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    mutex_lock(&osd_dev->osd_list_mutex);
    handle = osd_find_handle_node(osd_dev, buffer_info.handle_id);
    if (handle == NULL) {
        mutex_unlock(&osd_dev->osd_list_mutex);
        vio_err("[H%d] %s handle was null\n", buffer_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (handle->info.proc_type <= OSD_PROC_NV12) {
        buffer_info.proc_type = handle->info.proc_type;
        buffer_info.size.w = handle->buffer.size.w;
        buffer_info.size.h = handle->buffer.size.h;
        if (buffer_info.index < 0) {
            buffer_info.index = handle_find_buffer(handle, OSD_BUF_CREATE, &one_buf, 0);
            if (buffer_info.index < 0) {
                mutex_unlock(&osd_dev->osd_list_mutex);
                vio_err("[H%d] osd find buffer:%d failed\n",
                    buffer_info.handle_id, OSD_BUF_CREATE);
                ret = -ENOBUFS;
                goto exit;
            }
        } else {
            one_buf = &handle->buffer.buf[!!buffer_info.index];
        }
        buffer_info.paddr = one_buf->paddr;
        buffer_info.vaddr = one_buf->vaddr;
    }
    mutex_unlock(&osd_dev->osd_list_mutex);

    ret = (int32_t)copy_to_user((void __user *) arg, (void *) &buffer_info,
                sizeof(osd_buffer_info_t));

    vio_dbg("[H%d] %s done ret:%d\n",
        buffer_info.handle_id, __func__, ret);

exit:
    return ret;
}

static int osd_set_buffer(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    osd_buffer_info_t buffer_info;
    osd_handle_t *handle = NULL;
    struct x3_osd_dev *osd_dev;
    osd_one_buffer_t *one_buf;

    osd_dev = osd_ctx->osd_dev;

    ret = (int32_t)copy_from_user((void *) &buffer_info, (void __user *) arg,
        sizeof(osd_buffer_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    mutex_lock(&osd_dev->osd_list_mutex);
    handle = osd_find_handle_node(osd_dev, buffer_info.handle_id);
    if (handle == NULL) {
        mutex_unlock(&osd_dev->osd_list_mutex);
        vio_err("[H%d] %s handle was null\n", buffer_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (handle->info.proc_type <= OSD_PROC_NV12) {
        if (buffer_info.index == -1) {
            buffer_info.index = handle_find_buffer(handle, OSD_BUF_CREATE, &one_buf, 0);
            if (buffer_info.index < 0) {
                mutex_unlock(&osd_dev->osd_list_mutex);
                vio_err("[H%d] osd find buffer:%d failed\n",
                    buffer_info.handle_id, OSD_BUF_CREATE);
                ret = -EINVAL;
                goto exit;
            }
        }
        osd_one_buffer_flush(&handle->buffer.buf[buffer_info.index]);
        if (handle->buffer.vga_buf[buffer_info.index].state != OSD_BUF_NULL) {
            osd_vga4_to_sw(g_osd_color.color_map,
                handle->buffer.buf[buffer_info.index].vaddr,
                handle->buffer.vga_buf[buffer_info.index].vaddr,
                handle->buffer.size.w, handle->buffer.size.h);
            osd_one_buffer_flush(&handle->buffer.vga_buf[buffer_info.index]);
        }
        handle_update_buffer(handle, buffer_info.index);
        if (atomic_read(&handle->bind_cnt) > 0) {
            kthread_queue_work(&osd_dev->worker, &osd_dev->work);
        }
    }
    mutex_unlock(&osd_dev->osd_list_mutex);

    vio_dbg("[H%d] %s done ret:%d\n",
        buffer_info.handle_id, __func__, ret);

exit:
    return ret;
}

static int osd_attach(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;
    osd_bind_t *bind, *tmp_bind;
    osd_handle_t *handle;
    struct osd_subdev *subdev;
    int buf_index = -1;
    osd_one_buffer_t *one_buf, *vga_buf;
    uint32_t *polygon_buf = NULL;

    bind = kzalloc(sizeof(osd_bind_t), GFP_ATOMIC);
    if (bind == NULL) {
        vio_err("osd kzalloc failed\n");
        ret = -ENOMEM;
        goto exit;
    }

    ret = (int32_t)copy_from_user((void *) &bind->bind_info, (void __user *) arg,
        sizeof(osd_bind_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit_free_bind;
    }
    mutex_init(&bind->proc_info.proc_mutex);
    atomic_set(&bind->ref_cnt, 1);
    osd_print_bind_info(&bind->bind_info);

    osd_dev = osd_ctx->osd_dev;
    subdev = &osd_dev->subdev[bind->bind_info.instance][bind->bind_info.chn];

    if ((bind->bind_info.handle_info.handle_id >= 0) &&
        (bind->bind_info.handle_info.proc_type >= OSD_PROC_RECT)) {
        if (bind->bind_info.handle_info.proc_type == OSD_PROC_POLYGON) {
            polygon_buf = kzalloc
                (2 * bind->bind_info.handle_info.size.h * sizeof(uint32_t),
                GFP_ATOMIC);
            if (polygon_buf == NULL) {
                vio_err("osd kzalloc polygon buffer is fail\n");
                ret = -ENOMEM;
                goto exit_free_bind;
            }
            ret = (int32_t)copy_from_user((void *) polygon_buf,
                    (void __user *) bind->bind_info.polygon_buf,
                    2 * bind->bind_info.handle_info.size.h * sizeof(uint32_t));
            if (ret) {
                vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
                ret = -EFAULT;
                goto exit_free_polygon;
            }
            bind->bind_info.polygon_buf = polygon_buf;
        }
    }

    mutex_lock(&subdev->bind_mutex);
    tmp_bind = osd_find_bind_node(subdev, bind->bind_info.handle_id,
        bind->bind_info.buf_layer);
    if (tmp_bind != NULL) {
        atomic_inc(&tmp_bind->ref_cnt);
        vio_info("[S%d][V%d][H%d] already attach: count:%d\n",
            bind->bind_info.instance, bind->bind_info.chn,
            bind->bind_info.handle_id, atomic_read(&tmp_bind->ref_cnt));
        mutex_unlock(&subdev->bind_mutex);
        goto exit_free_polygon;
    }

    mutex_lock(&osd_dev->osd_list_mutex);
    handle = osd_find_handle_node(osd_dev, bind->bind_info.handle_id);
    if (handle == NULL) {
        mutex_unlock(&osd_dev->osd_list_mutex);
        mutex_unlock(&subdev->bind_mutex);
        vio_err("[S%d][V%d][H%d] %s handle was null!\n",
            bind->bind_info.instance, bind->bind_info.chn,
            bind->bind_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit_free_polygon;
    }

    mutex_lock(&bind->proc_info.proc_mutex);
    bind->proc_info.proc_type = (handle->info.proc_type >= OSD_PROC_RECT) ?
        bind->bind_info.handle_info.proc_type : handle->info.proc_type;
    if (bind->proc_info.proc_type == OSD_PROC_VGA4) {
        bind->proc_info.proc_type = osd_hw_check_limit(subdev, handle, bind);
    }
    bind->proc_info.subdev = subdev;
    mutex_unlock(&bind->proc_info.proc_mutex);

    if (bind->proc_info.proc_type == OSD_PROC_VGA4) {
        if ((handle->buffer.vga_buf[0].state == OSD_BUF_NULL) ||
            (handle->buffer.vga_buf[1].state == OSD_BUF_NULL)) {
            handle->buffer.vga_buf[0].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
            handle->buffer.vga_buf[1].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
            ret = osd_buffer_create_vga(osd_dev->ion_client, &handle->buffer);
            if (ret < 0) {
                mutex_unlock(&osd_dev->osd_list_mutex);
                mutex_unlock(&subdev->bind_mutex);
                goto exit_free_polygon;
            }
            buf_index = handle_find_buffer(handle, OSD_BUF_PROCESS, &one_buf, 0);
            if (buf_index < 0) {
                mutex_unlock(&osd_dev->osd_list_mutex);
                mutex_unlock(&subdev->bind_mutex);
                goto exit_free_polygon;
            }
            vga_buf = &handle->buffer.vga_buf[buf_index];
            osd_vga4_to_sw(g_osd_color.color_map, one_buf->vaddr, vga_buf->vaddr,
                handle->buffer.size.w, handle->buffer.size.h);
            osd_one_buffer_flush(one_buf);
        }
    }
    atomic_inc(&handle->bind_cnt);
    if (subdev->id == OSD_PYM_OUT) {
        handle->info.attach_pym = 1;
    }
    mutex_unlock(&osd_dev->osd_list_mutex);

    if ((bind->proc_info.proc_type != OSD_PROC_HW_VGA4) &&
        (bind->bind_info.osd_level == 0)) {
        // hw level: 0, sw level: 1-3
        bind->bind_info.osd_level = 1;
    }
    atomic_set(&bind->need_update, 1);

    list_add_tail(&bind->node, &subdev->bind_list);
    if (bind->proc_info.proc_type == OSD_PROC_HW_VGA4) {
        atomic_set(&subdev->osd_hw_need_update, 1);
    }
    kthread_queue_work(&osd_dev->worker, &osd_dev->work);

    vio_info("[S%d][V%d][H%d] %s done: ret:%d\n",
        bind->bind_info.instance, bind->bind_info.chn,
        bind->bind_info.handle_id, __func__, ret);

    mutex_unlock(&subdev->bind_mutex);

    return ret;

exit_free_polygon:
    if (polygon_buf != NULL) {
        kfree(polygon_buf);
        polygon_buf = NULL;
    }
exit_free_bind:
    if (bind != NULL) {
        kfree(bind);
        bind = NULL;
    }
exit:
    return ret;
}

static int osd_detach(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;
    osd_bind_info_t bind_info;
    osd_bind_t *bind;
    osd_handle_t *handle;
    struct osd_subdev *subdev;

    ret = (int32_t)copy_from_user((void *) &bind_info, (void __user *) arg,
        sizeof(osd_bind_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    osd_dev = osd_ctx->osd_dev;
    subdev = &osd_dev->subdev[bind_info.instance][bind_info.chn];

    mutex_lock(&subdev->bind_mutex);
    bind = osd_find_bind_node(subdev, bind_info.handle_id, bind_info.buf_layer);
    if (bind == NULL) {
        mutex_unlock(&subdev->bind_mutex);
        vio_err("[S%d][V%d][H%d] %s bind was null!\n",
                bind_info.instance, bind_info.chn,
                bind_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (atomic_dec_return(&bind->ref_cnt) == 0) {
        list_del(&bind->node);
        if (bind->proc_info.proc_type == OSD_PROC_HW_VGA4) {
            atomic_set(&subdev->osd_hw_need_update, 1);
            atomic_dec(&subdev->osd_hw_cnt);
            kthread_queue_work(&osd_dev->worker, &osd_dev->work);
        }
        osd_sw_set_process_flag(subdev);
        mutex_unlock(&subdev->bind_mutex);

        mutex_lock(&osd_dev->osd_list_mutex);
        handle = osd_find_handle_node(osd_dev, bind_info.handle_id);
        if (handle == NULL) {
            vio_err("[S%d][V%d][H%d] %s handle was null!\n",
                bind_info.instance, bind_info.chn,
                bind_info.handle_id, __func__);
        } else {
            atomic_dec(&handle->bind_cnt);
        }
        mutex_unlock(&osd_dev->osd_list_mutex);

        if (bind->bind_info.polygon_buf != NULL) {
            kfree(bind->bind_info.polygon_buf);
            bind->bind_info.polygon_buf = NULL;
        }
        kfree(bind);
        bind = NULL;
    } else {
        mutex_unlock(&subdev->bind_mutex);
    }

    vio_info("[S%d][V%d][H%d] %s done: ret:%d\n",
        bind_info.instance, bind_info.chn,
        bind_info.handle_id, __func__, ret);

exit:
    return ret;
}

static int osd_get_bind_attr(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;
    osd_bind_info_t bind_info;
    osd_bind_t *bind;
    struct osd_subdev *subdev;

    ret = (int32_t)copy_from_user((void *) &bind_info, (void __user *) arg,
        sizeof(osd_bind_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    osd_dev = osd_ctx->osd_dev;
    subdev = &osd_dev->subdev[bind_info.instance][bind_info.chn];

    mutex_lock(&subdev->bind_mutex);
    bind = osd_find_bind_node(subdev, bind_info.handle_id, bind_info.buf_layer);
    if (bind == NULL) {
        mutex_unlock(&subdev->bind_mutex);
        vio_err("[S%d][V%d][H%d] %s bind was null!\n",
                bind_info.instance, bind_info.chn,
                bind_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit;
    }

    ret = (int32_t)copy_to_user((void __user *) arg, (void *) &bind->bind_info,
                sizeof(osd_bind_info_t));
    mutex_unlock(&subdev->bind_mutex);

    vio_dbg("[S%d][V%d][H%d] %s done: ret:%d\n",
        bind_info.instance, bind_info.chn,
        bind_info.handle_id, __func__, ret);

exit:
    return ret;
}

static int osd_set_bind_attr(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;
    osd_bind_t *bind;
    osd_bind_info_t bind_info;
    osd_handle_t *handle;
    struct osd_subdev *subdev;
    int buf_index = -1;
    osd_one_buffer_t *one_buf, *vga_buf;
    uint32_t *polygon_buf = NULL;

    ret = (int32_t)copy_from_user((void *) &bind_info, (void __user *) arg,
        sizeof(osd_bind_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    osd_print_bind_info(&bind_info);
    osd_dev = osd_ctx->osd_dev;
    subdev = &osd_dev->subdev[bind_info.instance][bind_info.chn];

    mutex_lock(&subdev->bind_mutex);
    bind = osd_find_bind_node(subdev, bind_info.handle_id, bind_info.buf_layer);
    if (bind == NULL) {
        mutex_unlock(&subdev->bind_mutex);
        vio_err("[S%d][V%d][H%d] %s bind was null!\n",
                bind_info.instance, bind_info.chn,
                bind_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit;
    }

    mutex_lock(&osd_dev->osd_list_mutex);
    handle = osd_find_handle_node(osd_dev, bind_info.handle_id);
    if (handle == NULL) {
        mutex_unlock(&osd_dev->osd_list_mutex);
        mutex_unlock(&subdev->bind_mutex);
        vio_err("[S%d][V%d][H%d] %s handle was null!\n",
                bind_info.instance, bind_info.chn,
                bind_info.handle_id, __func__);
        ret = -EINVAL;
        goto exit;
    }
    if ((bind->bind_info.handle_info.proc_type == OSD_PROC_POLYGON) &&
        (osd_polygon_check_is_change(&bind->bind_info, &bind_info))) {
        polygon_buf = kzalloc
            (2 * bind->bind_info.handle_info.size.h * sizeof(uint32_t), GFP_ATOMIC);
        if (polygon_buf == NULL) {
            mutex_unlock(&osd_dev->osd_list_mutex);
            mutex_unlock(&subdev->bind_mutex);
            vio_err("osd kzalloc failed\n");
            ret = -ENOMEM;
            goto exit;
        }
        ret = (int32_t)copy_from_user((void *) polygon_buf,
                (void __user *) bind_info.polygon_buf,
                2 * bind->bind_info.handle_info.size.h * sizeof(uint32_t));
        if (ret) {
            mutex_unlock(&osd_dev->osd_list_mutex);
            mutex_unlock(&subdev->bind_mutex);
            vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
            goto exit_free;
        }
    }
    if ((bind->proc_info.proc_type == OSD_PROC_HW_VGA4) &&
        (osd_vga4_check_need_sw_process(&bind->bind_info, &bind_info))) {
        if ((handle->buffer.vga_buf[0].state == OSD_BUF_NULL) ||
            (handle->buffer.vga_buf[1].state == OSD_BUF_NULL)) {
            handle->buffer.vga_buf[0].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
            handle->buffer.vga_buf[1].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
            ret = osd_buffer_create_vga(osd_dev->ion_client, &handle->buffer);
            if (ret < 0) {
                mutex_unlock(&osd_dev->osd_list_mutex);
                mutex_unlock(&subdev->bind_mutex);
                goto exit_free;
            }
            buf_index = handle_find_buffer(handle, OSD_BUF_PROCESS, &one_buf, 0);
            if (buf_index < 0) {
                osd_buffer_destroy_vga(osd_dev->ion_client, &handle->buffer);
                mutex_unlock(&osd_dev->osd_list_mutex);
                mutex_unlock(&subdev->bind_mutex);
                goto exit_free;
            }
            vga_buf = &handle->buffer.vga_buf[buf_index];
            osd_vga4_to_sw(g_osd_color.color_map, one_buf->vaddr, vga_buf->vaddr,
                handle->buffer.size.w, handle->buffer.size.h);
            osd_one_buffer_flush(vga_buf);
        }

        mutex_lock(&bind->proc_info.proc_mutex);
        bind->proc_info.proc_type = OSD_PROC_VGA4;
        mutex_unlock(&bind->proc_info.proc_mutex);
        atomic_set(&subdev->osd_hw_need_update, 1);
        atomic_dec(&subdev->osd_hw_cnt);
    }
    mutex_unlock(&osd_dev->osd_list_mutex);
    if ((polygon_buf != NULL) && (bind->bind_info.polygon_buf != NULL)) {
        kfree(bind->bind_info.polygon_buf);
    } else {
        polygon_buf = bind->bind_info.polygon_buf;
    }
    if ((bind->proc_info.proc_type != OSD_PROC_HW_VGA4) &&
        (bind->bind_info.osd_level == 0)) {
        // hw level: 0, sw level: 1-3
        bind->bind_info.osd_level = 1;
    }
    memcpy(&bind->bind_info, &bind_info, sizeof(osd_bind_info_t));
    bind->bind_info.polygon_buf = polygon_buf;
    atomic_set(&bind->need_update, 1);
    kthread_queue_work(&osd_dev->worker, &osd_dev->work);
    mutex_unlock(&subdev->bind_mutex);

    vio_dbg("[S%d][V%d][H%d] %s done: ret:%d\n",
        bind_info.instance, bind_info.chn,
        bind_info.handle_id, __func__, ret);

    return ret;

exit_free:
    if (polygon_buf != NULL) {
        kfree(polygon_buf);
        polygon_buf = NULL;
    }
exit:
    return ret;
}

static int osd_set_sta(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;
    osd_sta_info_t sta_info;
    struct osd_subdev *subdev;
    int i = 0;

    ret = (int32_t)copy_from_user((void *) &sta_info, (void __user *) arg,
        sizeof(osd_sta_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    osd_dev = osd_ctx->osd_dev;
    subdev = &osd_dev->subdev[sta_info.instance][sta_info.chn];

    mutex_lock(&subdev->sta_mutex);

    subdev->osd_sta.enable_index = MAX_STA_NUM;
    for (i = MAX_STA_NUM - 1; i >= 0; i--) {
        if (sta_info.sta_box[i].sta_en) {
            subdev->osd_sta.enable_index = i;
            break;
        }
    }

    subdev->osd_sta.buf_layer = sta_info.buf_layer;
    memcpy(subdev->osd_sta.sta_level, sta_info.sta_level,
        MAX_OSD_STA_LEVEL_NUM * sizeof(uint8_t));
    memcpy(subdev->osd_sta.sta_box, sta_info.sta_box,
        MAX_STA_NUM * sizeof(osd_sta_box_t));
    subdev->osd_sta.sta_state = OSD_STA_REQUEST;
    if (subdev->id <= OSD_IPU_DS1) {
        osd_hw_set_sta_config(subdev);
        subdev->osd_sta.sta_state = OSD_STA_PROCESS;
    } else {
        atomic_set(&subdev->osd_info->need_sw_osd, 1);
    }
    mutex_unlock(&subdev->sta_mutex);

    vio_info("[S%d][V%d] %s done: ret:%d\n",
        sta_info.instance, sta_info.chn, __func__, ret);

exit:
    return ret;
}

static int osd_set_sta_level(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;
    osd_sta_info_t sta_info;
    struct osd_subdev *subdev;

    ret = (int32_t)copy_from_user((void *) &sta_info, (void __user *) arg,
        sizeof(osd_sta_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    osd_dev = osd_ctx->osd_dev;
    subdev = &osd_dev->subdev[sta_info.instance][sta_info.chn];

    mutex_lock(&subdev->sta_mutex);
    memcpy(subdev->osd_sta.sta_level, sta_info.sta_level,
        MAX_OSD_STA_LEVEL_NUM * sizeof(uint8_t));
    mutex_unlock(&subdev->sta_mutex);

    vio_info("[S%d][V%d] %s done: ret:%d\n",
        sta_info.instance, sta_info.chn, __func__, ret);

exit:
    return ret;
}

static int osd_get_sta_bin_value(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;
    osd_sta_bin_info_t sta_bin_info;
    struct osd_subdev *subdev;
    uint32_t enable_index;
    int i = 0;

    ret = (int32_t)copy_from_user((void *) &sta_bin_info, (void __user *) arg,
        sizeof(osd_sta_bin_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    osd_dev = osd_ctx->osd_dev;
    subdev = &osd_dev->subdev[sta_bin_info.instance][sta_bin_info.chn];
    enable_index = subdev->osd_sta.enable_index;

    if (subdev->osd_sta.sta_state == OSD_STA_NULL) {
        vio_err("[S%d][V%d] need set sta first, now state:%d\n",
            sta_bin_info.instance, sta_bin_info.chn, subdev->osd_sta.sta_state);
        ret = -EFAULT;
        goto exit;
    }
    if (enable_index == MAX_STA_NUM) {
        vio_warn("[S%d][V%d] no enable sta\n",
            sta_bin_info.instance, sta_bin_info.chn);
        goto exit_sta_null;
    }

    for (i = 0; i < 30; i++) {
        if (subdev->id <=  OSD_IPU_DS1) {
            // it means hw process sta
            osd_hw_get_sta_bin(subdev);
        } else {
            kthread_flush_work(&subdev->work);
        }
        if ((subdev->osd_sta.sta_value[enable_index][0] != 0) ||
            (subdev->osd_sta.sta_value[enable_index][1] != 0) ||
            (subdev->osd_sta.sta_value[enable_index][2] != 0) ||
            (subdev->osd_sta.sta_value[enable_index][3] != 0)) {
            break;
        }
        msleep(10);
    }
    if ((subdev->osd_sta.sta_value[enable_index][0] == 0) &&
        (subdev->osd_sta.sta_value[enable_index][1] == 0) &&
        (subdev->osd_sta.sta_value[enable_index][2] == 0) &&
        (subdev->osd_sta.sta_value[enable_index][3] == 0)) {
        vio_err("[S%d][V%d] timeout sta bin was null, now enable_index:%d\n",
            sta_bin_info.instance, sta_bin_info.chn, enable_index);
        ret = -ETIMEDOUT;
        goto exit_sta_null;
    }

    mutex_lock(&subdev->sta_mutex);
    memcpy(sta_bin_info.sta_value, (void *)subdev->osd_sta.sta_value,
        MAX_STA_NUM * MAX_STA_BIN_NUM * sizeof(uint16_t));
    memset((void *)subdev->osd_sta.sta_value, 0,
        MAX_STA_NUM * MAX_STA_BIN_NUM * sizeof(uint16_t));
    subdev->osd_sta.sta_state = OSD_STA_NULL;
    mutex_unlock(&subdev->sta_mutex);

    mutex_lock(&subdev->bind_mutex);
    osd_sw_set_process_flag(subdev);
    mutex_unlock(&subdev->bind_mutex);

    ret = (int32_t)copy_to_user((void __user *) arg, (void *) &sta_bin_info,
            sizeof(osd_sta_bin_info_t));

    vio_info("[S%d][V%d] %s done: ret:%d\n",
        sta_bin_info.instance, sta_bin_info.chn, __func__, ret);

    return ret;

exit_sta_null:
    subdev->osd_sta.sta_state = OSD_STA_NULL;

    mutex_lock(&subdev->bind_mutex);
    osd_sw_set_process_flag(subdev);
    mutex_unlock(&subdev->bind_mutex);
exit:
    return ret;
}

static int osd_set_color_map(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;
    osd_color_map_t osd_color;

    ret = (int32_t)copy_from_user((void *) &osd_color, (void __user *) arg,
        sizeof(osd_color_map_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    osd_dev = osd_ctx->osd_dev;

    if (osd_color.color_map_update) {
        memcpy(g_osd_color.color_map, osd_color.color_map,
            MAX_OSD_COLOR_NUM * sizeof(uint32_t));
        g_osd_color.color_map_update = 1;
        kthread_queue_work(&osd_dev->worker, &osd_dev->work);
    }

exit:
    return ret;
}

static int osd_proc_buf(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
    int ret = 0;
    osd_proc_buf_info_t proc_buf;
    osd_handle_t *handle = NULL;
    struct x3_osd_dev *osd_dev;
    osd_process_info_t proc_info = {0};
    osd_one_buffer_t *one_buf, *vga_buf;
    int buf_index = -1;

    osd_dev = osd_ctx->osd_dev;

    ret = (int32_t)copy_from_user((void *) &proc_buf, (void __user *) arg,
        sizeof(osd_proc_buf_info_t));
    if (ret) {
        vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
        ret = -EFAULT;
        goto exit;
    }

    proc_info.proc_type = proc_buf.proc_type;
    switch (proc_buf.proc_type)
    {
    case OSD_PROC_NV12:
        proc_info.yuv_bg_transparent = proc_buf.yuv_bg_transparent;
    case OSD_PROC_VGA4:
        mutex_lock(&osd_dev->osd_list_mutex);
        handle = osd_find_handle_node(osd_dev, proc_buf.handle_id);
        if (handle == NULL) {
            mutex_unlock(&osd_dev->osd_list_mutex);
            vio_err("[H%d] %s handle was null!\n",
                proc_buf.handle_id, __func__);
            ret = -EINVAL;
            goto exit;
        }
        if (proc_buf.proc_type == OSD_PROC_VGA4) {
            if ((handle->buffer.vga_buf[0].state == OSD_BUF_NULL) ||
                (handle->buffer.vga_buf[1].state == OSD_BUF_NULL)) {
                handle->buffer.size.w = handle->info.size.w;
                handle->buffer.size.h = handle->info.size.h;
                handle->buffer.vga_buf[0].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
                handle->buffer.vga_buf[1].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
                ret = osd_buffer_create_vga(osd_dev->ion_client, &handle->buffer);
                if (ret < 0) {
                    mutex_unlock(&osd_dev->osd_list_mutex);
                    goto exit;
                }
                buf_index = handle_find_buffer(handle, OSD_BUF_PROCESS, &one_buf, 0);
                if (buf_index < 0) {
                    osd_buffer_destroy_vga(osd_dev->ion_client, &handle->buffer);
                    mutex_unlock(&osd_dev->osd_list_mutex);
                    goto exit;
                }
                vga_buf = &handle->buffer.vga_buf[buf_index];
                osd_vga4_to_sw(g_osd_color.color_map, one_buf->vaddr, vga_buf->vaddr,
                    handle->buffer.size.w, handle->buffer.size.h);
                osd_one_buffer_flush(vga_buf);
            }
        }
        osd_set_process_handle_info(&proc_info, handle);
        mutex_unlock(&osd_dev->osd_list_mutex);
        break;
    case OSD_PROC_POLYGON:
        proc_info.polygon_buf = kzalloc(2 * proc_buf.size.h * sizeof(uint32_t),
            GFP_ATOMIC);
        if (proc_info.polygon_buf == NULL) {
            vio_err("osd kzalloc polygon buffer is failed\n");
            ret = -ENOMEM;
            goto exit;
        }
        ret = (int32_t)copy_from_user((void *) proc_info.polygon_buf,
                (void __user *) proc_buf.polygon_buf,
                2 * proc_buf.size.h * sizeof(uint32_t));
        if (ret) {
            vio_err("%s: copy_from_user failed (ret=%d)\n", __func__, ret);
            ret = -EFAULT;
            goto exit;
        }
    case OSD_PROC_RECT:
        proc_info.fill_color = proc_buf.fill_color;
    case OSD_PROC_MOSAIC:
        proc_info.width = proc_buf.size.w;
        proc_info.height = proc_buf.size.h;
        break;
    default:
        vio_err("[H%d] %s Invalid type:%d\n",
            proc_buf.handle_id, __func__, proc_buf.proc_type);
        ret = -EINVAL;
        goto exit;
    }
    proc_info.invert_en = proc_buf.invert_en;
    proc_info.start_x = proc_buf.start_point.x;
    proc_info.start_y = proc_buf.start_point.y;
    proc_info.image_width = proc_buf.image_width;
    proc_info.image_height = proc_buf.image_height;
    proc_info.tar_y_addr = __va(proc_buf.y_paddr);
    proc_info.tar_uv_addr = __va(proc_buf.uv_paddr);
    proc_info.subdev = NULL;

    osd_process_addr_inc(&proc_info);
    osd_run_process(&proc_info);
    osd_process_addr_dec(&proc_info);

    ion_dcache_invalid(proc_buf.y_paddr,
        proc_buf.image_width * proc_buf.image_height);
    ion_dcache_invalid(proc_buf.uv_paddr,
        proc_buf.image_width * proc_buf.image_height / 2);
    ion_dcache_flush(proc_buf.y_paddr,
        proc_buf.image_width * proc_buf.image_height);
    ion_dcache_flush(proc_buf.uv_paddr,
        proc_buf.image_width * proc_buf.image_height / 2);

    if (proc_buf.proc_type == OSD_PROC_POLYGON) {
        kfree(proc_info.polygon_buf);
        proc_info.polygon_buf = NULL;
    }

    vio_dbg("%s handle:%d done: ret(%d)\n",
        __func__, proc_buf.handle_id, ret);

exit:
    return ret;
}

static long x3_osd_ioctl(struct file *file, unsigned int cmd,
              unsigned long arg)
{
    struct osd_video_ctx *osd_ctx;
    int ret = 0;

    osd_ctx = file->private_data;

    if (_IOC_TYPE(cmd) != OSD_IOC_MAGIC)
        return -ENOTTY;

    switch (cmd) {
    case OSD_IOC_CREATE_HANDLE:
        ret = osd_create_handle(osd_ctx, arg);
        break;
    case OSD_IOC_DESTROY_HANDLE:
        ret = osd_destroy_handle(osd_ctx, arg);
        break;
    case OSD_IOC_GET_ATTR:
        ret = osd_get_attr(osd_ctx, arg);
        break;
    case OSD_IOC_SET_ATTR:
        ret = osd_set_attr(osd_ctx, arg);
        break;
    case OSD_IOC_GET_BUF:
        ret = osd_get_buffer(osd_ctx, arg);
        break;
    case OSD_IOC_SET_BUF:
        ret = osd_set_buffer(osd_ctx, arg);
        break;
    case OSD_IOC_ATTACH:
        ret = osd_attach(osd_ctx, arg);
        break;
    case OSD_IOC_DETACH:
        ret = osd_detach(osd_ctx, arg);
        break;
    case OSD_IOC_GET_BIND_ATTR:
        ret = osd_get_bind_attr(osd_ctx, arg);
        break;
    case OSD_IOC_SET_BIND_ATTR:
        ret = osd_set_bind_attr(osd_ctx, arg);
        break;
    case OSD_IOC_STA:
        ret = osd_set_sta(osd_ctx, arg);
        break;
    case OSD_IOC_STA_LEVEL:
        ret = osd_set_sta_level(osd_ctx, arg);
        break;
    case OSD_IOC_STA_BIN:
        ret = osd_get_sta_bin_value(osd_ctx, arg);
        break;
    case OSD_IOC_COLOR_MAP:
        ret = osd_set_color_map(osd_ctx, arg);
        break;
    case OSD_IOC_PROC_BUF:
        ret = osd_proc_buf(osd_ctx, arg);
        break;
    default:
        vio_err("wrong ioctl cmd: %x for osd\n", cmd);
        ret = -EINVAL;
        break;
    }

    return ret;
}

static struct file_operations x3_osd_fops = {
    .owner = THIS_MODULE,
    .open = x3_osd_open,
    .write = x3_osd_write,
    .read = x3_osd_read,
    .release = x3_osd_close,
    .unlocked_ioctl = x3_osd_ioctl,
    .compat_ioctl = x3_osd_ioctl,
};

static int x3_osd_suspend(struct device *dev)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;

    vio_info("%s\n", __func__);
	osd_dev = dev_get_drvdata(dev);
    if (atomic_read(&osd_dev->open_cnt) > 0) {
        osd_stop_worker(osd_dev);
    }

    return ret;
}

static int x3_osd_resume(struct device *dev)
{
    int ret = 0;
    struct x3_osd_dev *osd_dev;

    vio_info("%s\n", __func__);
	osd_dev = dev_get_drvdata(dev);
    if (atomic_read(&osd_dev->open_cnt) > 0) {
        ret = osd_start_worker(g_osd_dev);
    }

    return ret;
}

static int x3_osd_runtime_suspend(struct device *dev)
{
    int ret = 0;

    vio_info("%s\n", __func__);

    return ret;
}

static int x3_osd_runtime_resume(struct device *dev)
{
    int ret = 0;

    vio_info("%s\n", __func__);

    return ret;
}

static const struct dev_pm_ops x3_osd_pm_ops = {
    .suspend = x3_osd_suspend,
    .resume = x3_osd_resume,
    .runtime_suspend = x3_osd_runtime_suspend,
    .runtime_resume = x3_osd_runtime_resume,
};

int x3_osd_device_node_init(struct x3_osd_dev *osd)
{
    int ret = 0;
    struct device *dev = NULL;
    char name[32];

    snprintf(name, sizeof(name), "osd");

    ret = alloc_chrdev_region(&osd->devno, 0, OSD_MAX_DEVICE, "x3_osd");
    if (ret < 0) {
        vio_err("Error %d while alloc chrdev osd", ret);
        goto err_req_cdev;
    }

    cdev_init(&osd->cdev, &x3_osd_fops);
    osd->cdev.owner = THIS_MODULE;
    ret = cdev_add(&osd->cdev, osd->devno, OSD_MAX_DEVICE);
    if (ret) {
        vio_err("Error %d while adding x3 osd cdev", ret);
        goto err;
    }

    if (vps_class)
        osd->class = vps_class;
    else
        osd->class = class_create(THIS_MODULE, name);

    dev = device_create(osd->class, NULL, MKDEV(MAJOR(osd->devno), 0), NULL, name);
    if (IS_ERR(dev)) {
        ret = -EINVAL;
        vio_err("osd device create fail\n");
        goto err;
    }

    return ret;
err:
    class_destroy(osd->class);
err_req_cdev:
    unregister_chrdev_region(osd->devno, OSD_MAX_DEVICE);
    return ret;
}

static ssize_t osd_handle_show(struct device *dev,
                struct device_attribute *attr, char* buf)
{
    uint32_t offset = 0;
    int len;
    struct x3_osd_dev *osd_dev;
    osd_handle_t *handle, *temp;

    osd_dev = dev_get_drvdata(dev);

    mutex_lock(&osd_dev->osd_list_mutex);
    len = snprintf(&buf[offset], PAGE_SIZE - offset,
        "******************osd handle info******************\n");
    offset += len;
    list_for_each_entry_safe(handle, temp, &osd_dev->osd_list, node) {
        len = snprintf(&buf[offset], PAGE_SIZE - offset,
            "[H%d] info: bind_cnt:%d ref_cnt:%d fill_color:%d bg_trans:%d "
            "proc_type:%d size:%dx%d attach_pym:%d \n",
            handle->info.handle_id,
            atomic_read(&handle->bind_cnt), atomic_read(&handle->ref_cnt),
            handle->info.fill_color,
            handle->info.yuv_bg_transparent, handle->info.proc_type,
            handle->info.size.w, handle->info.size.h,
            handle->info.attach_pym);
        offset += len;
        len = snprintf(&buf[offset], PAGE_SIZE - offset,
            "[H%d] buffer: size: %dx%d \n",
            handle->info.handle_id,
            handle->buffer.size.w, handle->buffer.size.h);
        offset += len;
        if (handle->buffer.buf[0].state != OSD_BUF_NULL) {
            len = snprintf(&buf[offset], PAGE_SIZE - offset,
                "    buf[0]: state:%d pixel format:%d length:%ld paddr:0x%llx "
                "vaddr:%p ref_count:%d \n",
                handle->buffer.buf[0].state, handle->buffer.buf[0].pixel_fmt,
                handle->buffer.buf[0].length,
                handle->buffer.buf[0].paddr, handle->buffer.buf[0].vaddr,
                atomic_read(&handle->buffer.buf[0].ref_count));
            offset += len;
        }
        if (handle->buffer.buf[1].state != OSD_BUF_NULL) {
            len = snprintf(&buf[offset], PAGE_SIZE - offset,
                "    buf[1]: state:%d pixel format:%d length:%ld paddr:0x%llx "
                "vaddr:%p ref_count:%d \n",
                handle->buffer.buf[1].state, handle->buffer.buf[1].pixel_fmt,
                handle->buffer.buf[1].length,
                handle->buffer.buf[1].paddr, handle->buffer.buf[1].vaddr,
                atomic_read(&handle->buffer.buf[1].ref_count));
            offset += len;
        }
        if (handle->buffer.vga_buf[0].state != OSD_BUF_NULL) {
            len = snprintf(&buf[offset], PAGE_SIZE - offset,
                "    vga_buf[0]: state:%d pixel format:%d length:%ld paddr:0x%llx "
                "vaddr:%p ref_count:%d \n",
                handle->buffer.vga_buf[0].state, handle->buffer.vga_buf[0].pixel_fmt,
                handle->buffer.vga_buf[0].length,
                handle->buffer.vga_buf[0].paddr, handle->buffer.vga_buf[0].vaddr,
                atomic_read(&handle->buffer.vga_buf[0].ref_count));
            offset += len;
        }
        if (handle->buffer.vga_buf[1].state != OSD_BUF_NULL) {
            len = snprintf(&buf[offset], PAGE_SIZE - offset,
                "    vga_buf[1]: state:%d pixel format:%d length:%ld paddr:0x%llx "
                "vaddr:%p ref_count:%d \n",
                handle->buffer.vga_buf[1].state, handle->buffer.vga_buf[1].pixel_fmt,
                handle->buffer.vga_buf[1].length,
                handle->buffer.vga_buf[1].paddr, handle->buffer.vga_buf[1].vaddr,
                atomic_read(&handle->buffer.vga_buf[1].ref_count));
            offset += len;
        }
        len = snprintf(&buf[offset], PAGE_SIZE - offset, "****************\n");
        offset += len;
    }
    mutex_unlock(&osd_dev->osd_list_mutex);

    return offset;
}

static DEVICE_ATTR(handle_info, S_IRUGO, osd_handle_show, NULL);

static ssize_t osd_bind_show(struct device *dev,
                struct device_attribute *attr, char* buf)
{
    uint32_t offset = 0;
    int i, j, m, len;
    struct x3_osd_dev *osd_dev;
    struct osd_subdev *subdev;
    osd_bind_t *bind, *temp;

    osd_dev = dev_get_drvdata(dev);

    len = snprintf(&buf[offset], PAGE_SIZE - offset,
        "******************osd bind info******************\n");
    offset += len;
    for (j = 0; j < VIO_MAX_STREAM; j++) {
        for (i = 0; i < OSD_CHN_MAX; i++) {
            subdev = &osd_dev->subdev[j][i];

            mutex_lock(&subdev->bind_mutex);
            list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
                len = snprintf(&buf[offset], PAGE_SIZE - offset,
                    "[S%d][V%d][H%d]: show:%d invert:%d level:%d "
                    "buf_layer:%d start: (%d, %d) bind_cnt:%d\n",
                    bind->bind_info.instance, bind->bind_info.chn,
                    bind->bind_info.handle_id, bind->bind_info.show_en,
                    bind->bind_info.invert_en, bind->bind_info.osd_level,
                    bind->bind_info.buf_layer, bind->bind_info.start_point.x,
                    bind->bind_info.start_point.y, atomic_read(&bind->ref_cnt));
                offset += len;
                if (bind->bind_info.handle_info.proc_type == OSD_PROC_RECT) {
                    len = snprintf(&buf[offset], PAGE_SIZE - offset,
                        "rect: size:%dx%d fill_color:%d\n",
                        bind->bind_info.handle_info.size.w,
                        bind->bind_info.handle_info.size.h,
                        bind->bind_info.handle_info.fill_color);
                    offset += len;
                }
                if (bind->bind_info.handle_info.proc_type == OSD_PROC_POLYGON) {
                    len = snprintf(&buf[offset], PAGE_SIZE - offset,
                        "polygon: side num:%d fill_color:%d point:(%d, %d) \n"
                        "(%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d) "
                        "\n(%d, %d) (%d, %d) (%d, %d) buffer:%p\n",
                        bind->bind_info.side_num,
                        bind->bind_info.handle_info.fill_color,
                        bind->bind_info.point[0].x, bind->bind_info.point[0].y,
                        bind->bind_info.point[1].x, bind->bind_info.point[1].y,
                        bind->bind_info.point[2].x, bind->bind_info.point[2].y,
                        bind->bind_info.point[3].x, bind->bind_info.point[3].y,
                        bind->bind_info.point[4].x, bind->bind_info.point[4].y,
                        bind->bind_info.point[5].x, bind->bind_info.point[5].y,
                        bind->bind_info.point[6].x, bind->bind_info.point[6].y,
                        bind->bind_info.point[7].x, bind->bind_info.point[7].y,
                        bind->bind_info.point[8].x, bind->bind_info.point[8].y,
                        bind->bind_info.point[9].x, bind->bind_info.point[9].y,
                        bind->bind_info.polygon_buf);
                    offset += len;
                }
                if (bind->bind_info.handle_info.proc_type == OSD_PROC_MOSAIC) {
                    len = snprintf(&buf[offset], PAGE_SIZE - offset,
                        "mosaic: size:%dx%d\n",
                        bind->bind_info.handle_info.size.w,
                        bind->bind_info.handle_info.size.h);
                    offset += len;
                }
                len = snprintf(&buf[offset], PAGE_SIZE - offset,
                    "******************\n");
                offset += len;
            }
            mutex_unlock(&subdev->bind_mutex);

            if (subdev->osd_hw_cfg != NULL) {
                spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
                for (m = 0; m < OSD_HW_PROC_NUM; m++) {
                    if (subdev->osd_hw_cfg->osd_box[m].osd_en == 0) {
                        continue;
                    }
                    len = snprintf(&buf[offset], PAGE_SIZE - offset,
                        "[V%d][%d]: hw en:%d mode:%d start:(%d, %d) size:(%d, %d)\n",
                        subdev->id, m,
                        subdev->osd_hw_cfg->osd_box[m].osd_en,
                        subdev->osd_hw_cfg->osd_box[m].overlay_mode,
                        subdev->osd_hw_cfg->osd_box[m].start_x,
                        subdev->osd_hw_cfg->osd_box[m].start_y,
                        subdev->osd_hw_cfg->osd_box[m].width,
                        subdev->osd_hw_cfg->osd_box[m].height);
                    offset += len;
                    len = snprintf(&buf[offset], PAGE_SIZE - offset,
                        "******************\n");
                    offset += len;
                }
                spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
            }
        }
    }
    return offset;
}

static DEVICE_ATTR(bind_info, S_IRUGO, osd_bind_show, NULL);

static ssize_t osd_fps_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	u32 offset = 0;
	int i, j, len;
	char *osd_chn[OSD_CHN_MAX] =
		{"osd_ipu_us ", "osd_ipu_ds0", "osd_ipu_ds1", "osd_ipu_ds2",
        "osd_ipu_ds3", "osd_ipu_ds4", "osd_ipu_src", "osd_pym_out"};

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		for (j = 0; j < OSD_CHN_MAX; j++) {
			if (g_osd_fps[i][j] > 0) {
				len = snprintf(&buf[offset], PAGE_SIZE - offset,
					"osd pipe %d:  %s output fps %d\n",
					i, osd_chn[j], g_osd_fps[i][j]);
				offset += len;
				g_osd_fps[i][j] = 0;
			}
		}
	}
	return offset;
}

static DEVICE_ATTR(fps, S_IRUGO, osd_fps_show, NULL);	/*PRQA S 0636*/

void x3_osd_subdev_init(struct x3_osd_dev *osd)
{
    int32_t i = 0, j = 0;
    struct osd_subdev *subdev;
    struct ipu_subdev *ipu_subdev;

    for (i = 0; i < VIO_MAX_STREAM; i++) {
        for (j = 0; j < OSD_CHN_MAX; j++) {
            subdev = &osd->subdev[i][j];
            subdev->osd_dev = osd;
            subdev->id = j;
            subdev->osd_info = osd_get_info(i, j);
            if (j <= OSD_IPU_DS1) {
                ipu_subdev = container_of(subdev->osd_info, struct ipu_subdev, osd_info);
                subdev->osd_hw_cfg = &ipu_subdev->osd_cfg;
            }
            INIT_LIST_HEAD(&subdev->bind_list);
            INIT_LIST_HEAD(&subdev->input_frame_list);
            mutex_init(&subdev->bind_mutex);
            mutex_init(&subdev->sta_mutex);
            spin_lock_init(&subdev->frame_slock);
            atomic_set(&subdev->osd_hw_need_update, 0);
            atomic_set(&subdev->osd_hw_cnt, 0);
            kthread_init_work(&subdev->work, osd_frame_process_workfunc);
        }
    }
}

static int x3_osd_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct x3_osd_dev *osd;
    struct device *dev;

    BUG_ON(!pdev);

    dev = &pdev->dev;

    osd = kzalloc(sizeof(struct x3_osd_dev), GFP_ATOMIC);
    if (!osd) {
        vio_err("osd is NULL");
        ret = -ENOMEM;
        goto exit;
    }

    x3_osd_device_node_init(osd);

    ret = device_create_file(dev, &dev_attr_handle_info);
    if (ret < 0) {
        vio_err("create handle info failed (%d)\n", ret);
        goto exit_create_file;
    }

    ret = device_create_file(dev, &dev_attr_bind_info);
    if (ret < 0) {
        vio_err("create bind info failed (%d)\n", ret);
        goto exit_create_file;
    }

    ret = device_create_file(dev, &dev_attr_fps);
    if (ret < 0) {
        vio_err("create fps failed (%d)\n", ret);
        goto exit_create_file;
    }

    osd->ion_client = ion_client_create(hb_ion_dev, "osd_driver_ion");
	if (IS_ERR(osd->ion_client)) {
		vio_err("osd ion client create failed.");
		goto exit_create_file;
	}

    platform_set_drvdata(pdev, osd);

    kthread_init_worker(&osd->worker);
    kthread_init_work(&osd->work, osd_set_process_info_workfunc);
    INIT_LIST_HEAD(&osd->osd_list);
    mutex_init(&osd->osd_mutex);
    mutex_init(&osd->osd_list_mutex);
    atomic_set(&osd->open_cnt, 0);
    g_osd_dev = osd;

    x3_osd_subdev_init(osd);
    osd_send_callback(osd_frame_process);

    vio_info("[FRT:D] %s(%d)\n", __func__, ret);

    return 0;

exit_create_file:
    kfree(osd);
exit:
    vio_err("[FRT:D] %s(%d)\n", __func__, ret);
    return ret;
}

static int x3_osd_remove(struct platform_device *pdev)
{
    int ret = 0;
    struct x3_osd_dev *osd;

    BUG_ON(!pdev);

    osd = platform_get_drvdata(pdev);

    ion_client_destroy(osd->ion_client);

    device_remove_file(&pdev->dev, &dev_attr_handle_info);
    device_remove_file(&pdev->dev, &dev_attr_bind_info);
    device_remove_file(&pdev->dev, &dev_attr_fps);

    mutex_destroy(&osd->osd_mutex);
    mutex_destroy(&osd->osd_list_mutex);
    device_destroy(osd->class, osd->devno);
    if (!vps_class)
        class_destroy(osd->class);
    cdev_del(&osd->cdev);
    unregister_chrdev_region(osd->devno, OSD_MAX_DEVICE);
    kfree(osd);

    vio_info("%s\n", __func__);

    return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x3_osd_match[] = {
    {
     .compatible = "hobot,x3-osd",
     },
    {},
};

MODULE_DEVICE_TABLE(of, x3_osd_match);

static struct platform_driver x3_osd_driver = {
    .probe = x3_osd_probe,
    .remove = x3_osd_remove,
    .driver = {
           .name = MODULE_NAME,
           .owner = THIS_MODULE,
           .pm = &x3_osd_pm_ops,
           .of_match_table = x3_osd_match,
           }
};

#else
static struct platform_device_id x3_osd_driver_ids[] = {
    {
     .name = MODULE_NAME,
     .driver_data = 0,
     },
    {},
};

MODULE_DEVICE_TABLE(platform, x3_osd_driver_ids);

static struct platform_driver x3_osd_driver = {
    .probe = x3_osd_probe,
    .remove = __devexit_p(x3_osd_remove),
    .id_table = x3_osd_driver_ids,
    .driver = {
           .name = MODULE_NAME,
           .owner = THIS_MODULE,
           .pm = &x3_osd_pm_ops,
           }
};
#endif

static int __init x3_osd_init(void)
{
    int ret = platform_driver_register(&x3_osd_driver);
    if (ret)
        vio_err("platform_driver_register failed: %d\n", ret);

    return ret;
}

late_initcall(x3_osd_init);

static void __exit x3_osd_exit(void)
{
    platform_driver_unregister(&x3_osd_driver);
}

module_exit(x3_osd_exit);

MODULE_AUTHOR("Liang Yilong <v-yilong.liang@horizon.ai>");
MODULE_DESCRIPTION("X3 OSD driver");
MODULE_LICENSE("GPL v2");
// PRQA S --
