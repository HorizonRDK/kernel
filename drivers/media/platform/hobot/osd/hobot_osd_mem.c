/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include "hobot_osd_mem.h"
#include "vio_config.h"

#define OSD_ION_TYPE 16

struct list_head s_osd_buf_list = LIST_HEAD_INIT(s_osd_buf_list);
spinlock_t s_osd_buf_slock = __SPIN_LOCK_UNLOCKED(s_osd_buf_slock);

int32_t osd_ion_alloc(struct ion_client *client, osd_one_buffer_t *buf)
{
    int32_t ret = 0;

    buf->ion_handle = ion_alloc(client, buf->length, PAGE_SIZE,
        ION_HEAP_CARVEOUT_MASK,
        (OSD_ION_TYPE << 16) | ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC);
    if (IS_ERR(buf->ion_handle)) {
        vio_err("osd ion alloc failed\n");
        return -ENOMEM;
    }
    ret = ion_phys(client, buf->ion_handle->id,
                    &buf->paddr,
                    &buf->length);
    if (ret) {
        vio_err("osd ion get phys addr failed\n");
        goto EXIT;
    }
    buf->vaddr = ion_map_kernel(client, buf->ion_handle);
    if (IS_ERR(buf->vaddr)) {
        vio_err("osd ion map failed\n");
        goto EXIT;
    }

    vio_dbg("osd alloc buffer paddr:0x%llx vaddr:%p length:%ld\n",
        buf->paddr, buf->vaddr, buf->length);

    return 0;
EXIT:
    ion_free(client, buf->ion_handle);
    buf->ion_handle = NULL;
    buf->paddr = 0;
    buf->vaddr = NULL;

    return -ENOMEM;
}

void osd_one_buffer_inc(osd_one_buffer_t *buf)
{
    atomic_inc(&buf->ref_count);
}

void osd_one_buffer_inc_by_addr(uint8_t *vaddr)
{
    osd_one_buffer_t *buf, *temp;

    spin_lock(&s_osd_buf_slock);
    list_for_each_entry_safe(buf, temp, &s_osd_buf_list, node) {
        if (buf->vaddr == vaddr) {
            atomic_inc(&buf->ref_count);
        }
    }
    spin_unlock(&s_osd_buf_slock);
}

void osd_one_buffer_inc_by_paddr(uint64_t paddr)
{
    osd_one_buffer_t *buf, *temp;

    spin_lock(&s_osd_buf_slock);
    list_for_each_entry_safe(buf, temp, &s_osd_buf_list, node) {
        if (buf->paddr == paddr) {
            atomic_inc(&buf->ref_count);
            break;
        }
    }
    spin_unlock(&s_osd_buf_slock);
}

void osd_one_buffer_dec(osd_one_buffer_t *buf)
{
    atomic_dec(&buf->ref_count);
}

void osd_one_buffer_dec_by_addr(uint8_t *vaddr)
{
    osd_one_buffer_t *buf, *temp;

    spin_lock(&s_osd_buf_slock);
    list_for_each_entry_safe(buf, temp, &s_osd_buf_list, node) {
        if (buf->vaddr == vaddr) {
            atomic_dec(&buf->ref_count);
        }
    }
    spin_unlock(&s_osd_buf_slock);
}

void osd_one_buffer_dec_by_paddr(uint64_t paddr)
{
    osd_one_buffer_t *buf, *temp;

    spin_lock(&s_osd_buf_slock);
    list_for_each_entry_safe(buf, temp, &s_osd_buf_list, node) {
        if (buf->paddr == paddr) {
            atomic_dec(&buf->ref_count);
            break;
        }
    }
    spin_unlock(&s_osd_buf_slock);
}

int32_t osd_one_buffer_create(struct ion_client *client, osd_one_buffer_t *buf)
{
    if (osd_ion_alloc(client, buf) < 0) {
        return -ENOMEM;
    }

    buf->state = OSD_BUF_CREATE;
    atomic_set(&buf->ref_count, 0);

    spin_lock(&s_osd_buf_slock);
    list_add_tail(&buf->node, &s_osd_buf_list);
    spin_unlock(&s_osd_buf_slock);

    vio_dbg("osd create buffer format:%d, paddr:0x%llx addr:%p length:%ld\n",
        buf->pixel_fmt, buf->paddr, buf->vaddr, buf->length);

    return 0;
}

void osd_one_buffer_destroy(struct ion_client *client, osd_one_buffer_t *buf)
{
    int i = 0;

    for (i = 0; i < 30; i++) {
        // wait for no thread use this buffer
        if (atomic_read(&buf->ref_count) == 0) {
            break;
        }
        msleep(10);
        if (i == 30 - 1) {
            vio_err("osd one buffer paddr:0x%llx desroyed, but ref count is %d\n",
                buf->paddr, atomic_read(&buf->ref_count));
        }
    }
    spin_lock(&s_osd_buf_slock);
    list_del(&buf->node);
    spin_unlock(&s_osd_buf_slock);

    if (buf->ion_handle != NULL) {
        vio_dbg("osd destroy buffer format:%d, paddr:0x%llx addr:%p length:%ld\n",
            buf->pixel_fmt, buf->paddr, buf->vaddr, buf->length);
        ion_free(client, buf->ion_handle);
        buf->ion_handle = NULL;
    }
    buf->paddr = 0;
    buf->vaddr = NULL;
    buf->length = 0;
    buf->pixel_fmt = OSD_PIXEL_FORMAT_NULL;
    buf->state = OSD_BUF_NULL;
}

void osd_one_buffer_flush(osd_one_buffer_t *one_buffer)
{
    ion_dcache_invalid(one_buffer->paddr, one_buffer->length);
    ion_dcache_flush(one_buffer->paddr, one_buffer->length);
}

void osd_one_buffer_fill(osd_one_buffer_t *one_buffer, uint32_t color)
{
    int size = 0;
    uint8_t y_color = 0, u_color = 0, v_color = 0;
    uint8_t *addr = NULL;
    int i = 0;

    if (one_buffer->pixel_fmt == OSD_PIXEL_FORMAT_VGA4) {
        memset(one_buffer->vaddr, (color << 4) | color, one_buffer->length);
    } else if (one_buffer->pixel_fmt == OSD_PIXEL_FORMAT_NV12) {
        y_color = (color >> 16) & 0xff;
        u_color = (color >> 8) & 0xff;
        v_color = color & 0xff;
        size = one_buffer->length * 2 / 3; // get (w * h) size
        addr = one_buffer->vaddr + size;
        memset(one_buffer->vaddr, y_color, size);
        for (i = 0; i < size / 2; i += 2) {
            addr[0] = u_color;
            addr[1] = v_color;
            addr += 2;
        }
    }
}

static size_t osd_calculate_buffer_length(uint32_t width, uint32_t height,
    osd_pixel_format_t pixel_fmt)
{
    if (pixel_fmt == OSD_PIXEL_FORMAT_VGA4) {
        return (width * height / 2);
    } else if (pixel_fmt == OSD_PIXEL_FORMAT_NV12) {
        return width * height * 3 / 2;
    } else if (pixel_fmt == OSD_PIXEL_FORMAT_SW_VGA4) {
        return width * height * 3;
    } else if (pixel_fmt == OSD_PIXEL_FORMAT_POLYGON) {
        return (2 * height * sizeof(uint32_t));
    } else {
        return 0;
    }
}

int32_t osd_buffer_create(struct ion_client *client, osd_buffer_t *osd_buffer)
{
    size_t length;
    int32_t ret = 0, i;

    for (i = 0; i < OSD_PING_PONG_BUF; i++) {
        if ((osd_buffer->buf[i].state == OSD_BUF_NULL) &&
            (osd_buffer->buf[i].pixel_fmt != OSD_PIXEL_FORMAT_NULL)) {
            length = osd_calculate_buffer_length(osd_buffer->size.w, osd_buffer->size.h,
                osd_buffer->buf[i].pixel_fmt);
            osd_buffer->buf[i].length = length;
            ret = osd_one_buffer_create(client, &osd_buffer->buf[i]);
            if (ret < 0) {
                goto EXIT;
            }
            osd_buffer->buf[i].state = OSD_BUF_CREATE + i;
        }
        if ((osd_buffer->vga_buf[i].state == OSD_BUF_NULL) &&
            (osd_buffer->vga_buf[i].pixel_fmt != OSD_PIXEL_FORMAT_NULL)) {
            length = osd_calculate_buffer_length(osd_buffer->size.w, osd_buffer->size.h,
                osd_buffer->vga_buf[i].pixel_fmt);
            osd_buffer->vga_buf[i].length = length;
            ret = osd_one_buffer_create(client, &osd_buffer->vga_buf[i]);
            if (ret < 0) {
                goto EXIT;
            }
            osd_buffer->vga_buf[i].state = OSD_BUF_CREATE;
        }
    }

    return ret;

EXIT:
    osd_buffer_destroy(client, osd_buffer);

    return ret;
}

int32_t osd_buffer_create_vga(struct ion_client *client, osd_buffer_t *osd_buffer)
{
    size_t length;
    int32_t ret = 0, i;

    for (i = 0; i < OSD_PING_PONG_BUF; i++) {
        if ((osd_buffer->vga_buf[i].state == OSD_BUF_NULL) &&
            (osd_buffer->vga_buf[i].pixel_fmt != OSD_PIXEL_FORMAT_NULL)) {
            length = osd_calculate_buffer_length(osd_buffer->size.w, osd_buffer->size.h,
                osd_buffer->vga_buf[i].pixel_fmt);
            osd_buffer->vga_buf[i].length = length;
            ret = osd_one_buffer_create(client, &osd_buffer->vga_buf[i]);
            if (ret < 0) {
                goto EXIT;
            }
            osd_buffer->vga_buf[i].state = osd_buffer->buf[i].state;
        }
    }

    return ret;

EXIT:
    osd_buffer_destroy_vga(client, osd_buffer);

    return ret;
}

void osd_buffer_destroy(struct ion_client *client, osd_buffer_t *osd_buffer)
{
    int32_t i;

    for (i = 0; i < OSD_PING_PONG_BUF; i++) {
        if (osd_buffer->buf[i].state != OSD_BUF_NULL) {
            osd_one_buffer_destroy(client, &osd_buffer->buf[i]);
        }
        if (osd_buffer->vga_buf[i].state != OSD_BUF_NULL) {
            osd_one_buffer_destroy(client, &osd_buffer->vga_buf[i]);
        }
    }
}

void osd_buffer_destroy_vga(struct ion_client *client, osd_buffer_t *osd_buffer)
{
    int32_t i;

    for (i = 0; i < OSD_PING_PONG_BUF; i++) {
        if (osd_buffer->vga_buf[i].state != OSD_BUF_NULL) {
            osd_one_buffer_destroy(client, &osd_buffer->vga_buf[i]);
        }
    }
}

void osd_buffer_flush(osd_buffer_t *osd_buffer)
{
    int32_t i = 0;

    for (i = 0; i < OSD_PING_PONG_BUF; i++) {
        if (osd_buffer->buf[i].state != OSD_BUF_NULL) {
            osd_one_buffer_flush(&osd_buffer->buf[i]);
        }
        if (osd_buffer->vga_buf[i].state != OSD_BUF_NULL) {
            osd_one_buffer_flush(&osd_buffer->vga_buf[i]);
        }
    }
}

struct vio_frame *osd_get_frame(struct list_head *list, int32_t frame_index)
{
    struct vio_frame *frame, *temp;

    list_for_each_entry_safe(frame, temp, list, list) {
		if (frame->frameinfo.bufferindex == frame_index) {
            list_del(&frame->list);
            frame->state = FS_INVALID;
			return frame;
        }
	}

	return NULL;
}

void osd_put_input_frame(struct list_head *list, struct vio_frame *frame)
{
	list_add_tail(&frame->list, list);
}

struct vio_frame *osd_get_input_frame(struct list_head *list)
{
    struct vio_frame *frame;

	frame = list_first_entry(list, struct vio_frame, list);
    list_del(&frame->list);

    return frame;
}
