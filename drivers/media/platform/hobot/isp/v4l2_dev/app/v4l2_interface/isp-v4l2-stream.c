/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/
#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <asm/div64.h>
#include <linux/sched.h>

#include <linux/videodev2.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include "acamera_firmware_api.h"
#include "acamera_firmware_config.h"

#if defined( ISP_HAS_METADATA_FSM )
#include "metadata_api.h"
#endif

#include "acamera_logger.h"
#include "acamera_fw.h"

#include "dma_writer.h"
#include "dma_writer_fsm.h"

#include "isp-v4l2-common.h"
#include "isp-v4l2.h"
#include "fw-interface.h"

#include "isp-v4l2-stream.h"


/* metadata size */
#if defined( ISP_HAS_METADATA_FSM )
#define ISP_V4L2_METADATA_SIZE sizeof( firmware_metadata_t )
#else
#define ISP_V4L2_METADATA_SIZE 4096
#endif

/* max size */
#define ISP_V4L2_MAX_WIDTH 4096
#define ISP_V4L2_MAX_HEIGHT 3000

/* default size & format */
#define ISP_DEFAULT_FORMAT V4L2_PIX_FMT_RGB32

extern void *acamera_get_ctx_ptr( uint32_t ctx_id );
extern int dma_writer_configure_pipe( dma_pipe *pipe );
extern metadata_t *dma_writer_return_metadata(void *handle, dma_type type);
extern void acamera_dma_wr_check(void);

typedef struct _isp_v4l2_fmt {
    const char *name;
    uint32_t fourcc;
    uint8_t depth;
    bool is_yuv;
    uint8_t planes;
} isp_v4l2_fmt_t;

static isp_v4l2_fmt_t isp_v4l2_supported_formats[] =
    {
        {
            .name = "ARGB32",
            .fourcc = V4L2_PIX_FMT_RGB32,
            .depth = 32,
            .is_yuv = false,
            .planes = 1,
        },
        {
            .name = "ARGB30",
            .fourcc = ISP_V4L2_PIX_FMT_ARGB2101010,
            .depth = 32,
            .is_yuv = false,
            .planes = 1,
        },
        {
            .name = "NV12",
            .fourcc = V4L2_PIX_FMT_NV12,
            .depth = 8,
            .is_yuv = true,
            .planes = 2,
        },
/* NOTE: Linux kernel 3.19 doesn't support RAW colorspace,
             V4L2_COLORSPACE_RAW is added in Linux 4.2, we support
             RAW format here just for internal debugging. */
#if ISP_HAS_RAW_CB
        {
            .name = "RAW 16",
            .fourcc = V4L2_PIX_FMT_SBGGR16,
            .depth = 16,
            .is_yuv = false,
            //.planes   = 1,
            .planes = ISP_DMA_RAW_CAPTURE,
        },
#endif
#if ISP_HAS_META_CB
        {
            .name = "META",
            .fourcc = ISP_V4L2_PIX_FMT_META,
            .depth = 8,
            .is_yuv = false,
            .planes = 1,
        },
#endif
};

/* ----------------------------------------------------------------
 * Stream callback interface
 */
#if ISP_HAS_META_CB
void callback_meta( uint32_t ctx_id, const void *fw_metadata )
{
    isp_v4l2_stream_t *pstream = NULL;
    isp_v4l2_buffer_t *pbuf = NULL;
    void *vb2_buf = NULL;
    uint32_t frame_id = 0;
    int rc;
    int cnt = 0;
    struct list_head *p, *n;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer *vvb;
#endif
    struct vb2_buffer *vb;
    unsigned int buf_index;
    unsigned int buf_type;

    //add atomic started here to prevent raise condition with closing
    if ( fw_metadata == NULL ) {
        LOG( LOG_ERR, "metadata is null (ctx %d)", ctx_id );
        return;
    }

    /* find stream pointer */
    rc = isp_v4l2_find_stream( &pstream, ctx_id, V4L2_STREAM_TYPE_META );
    if ( rc < 0 ) {
        LOG( LOG_INFO, "can't find stream on ctx %d (errno = %d)", ctx_id, rc );
        return;
    }

    /* check if stream is on */
    if ( !pstream->stream_started ) {
        LOG( LOG_INFO, "[Stream#%d] stream META is not started yet on ctx %d", pstream->stream_id, ctx_id );
        return;
    }

    /* filter redundant frame id */
    frame_id = *(uint32_t *)fw_metadata;
    pstream->last_frame_id = frame_id;
    LOG( LOG_INFO, "[ctx_id#%d::stream#%d] Meta Frame ID %d.", ctx_id, pstream->stream_id, frame_id );


    /* get buffer from vb2 queue  */
    spin_lock( &pstream->slock );
//debug start
    list_for_each_safe(p, n, &pstream->stream_buffer_list)
	cnt++;
    pr_debug("stream_buffer_list count %d", cnt);
// debug end
    if ( !list_empty( &pstream->stream_buffer_list ) ) {
        pbuf = list_entry( pstream->stream_buffer_list.next, isp_v4l2_buffer_t, list );
        list_del( &pbuf->list );
    }
    spin_unlock( &pstream->slock );
    if ( !pbuf ) {
        LOG( LOG_INFO, "[Stream#%d] No active buffer in queue !", pstream->stream_id );
        return;
    }
    if ( atomic_inc_and_test( &pstream->running ) ) {
        LOG( LOG_ERR, "[Stream#%d] Already deinited stream !", pstream->stream_id );
        return;
    }

    /* fill buffer */
    pstream->fw_frame_seq_count++;

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    vvb = &pbuf->vvb;
    vb = &vvb->vb2_buf;

    buf_index = vb->index;
    buf_type = vb->type;

    vvb->field = V4L2_FIELD_NONE;
    /* update frame id */
    vvb->sequence = *(uint32_t *)fw_metadata;
#else
    vb = &pbuf->vb;

    buf_index = vb->v4l2_buf.index;
    buf_type = vb->v4l2_buf.type;

    vb->v4l2_buf.field = V4L2_FIELD_NONE;
    /* update frame id */
    vb->v4l2_buf.sequence = *(uint32_t *)fw_metadata;
#endif

    vb2_buf = vb2_plane_vaddr( vb, 0 );
    //
    //mutiplanar?
    if ( buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE )
        memcpy( vb2_buf, fw_metadata, pstream->cur_v4l2_fmt.fmt.pix_mp.plane_fmt[0].sizeimage );
    else
        memcpy( vb2_buf, fw_metadata, pstream->cur_v4l2_fmt.fmt.pix.sizeimage );


    /* Put buffer back to vb2 queue */
    vb2_buffer_done( vb, VB2_BUF_STATE_DONE );

    /* Notify buffer ready */
    isp_v4l2_notify_event( pstream->ctx_id, pstream->stream_id, V4L2_EVENT_ACAMERA_FRAME_READY );

    LOG( LOG_DEBUG, "[Stream#%d] vid_cap buffer %d done (size=%d, m.w=%d, m.h=%d)",
         pstream->stream_id, buf_index,
         pstream->cur_v4l2_fmt.fmt.pix.sizeimage,
         ( (firmware_metadata_t *)vb2_buf )->sensor_width, ( (firmware_metadata_t *)vb2_buf )->sensor_height );

    atomic_set( &pstream->running, 0 );
}
#endif

static uint32_t isp_v4l2_format_to_dma_output( uint32_t pixelformat, uint32_t plane_no )
{
    uint32_t dma_format = DMA_FORMAT_DISABLE;

    switch ( pixelformat ) {
    case V4L2_PIX_FMT_SBGGR16:
        dma_format = DMA_FORMAT_RAW16;
        break;
    case V4L2_PIX_FMT_RGB32:
        dma_format = DMA_FORMAT_RGB32;
        break;
    case ISP_V4L2_PIX_FMT_ARGB2101010:
        dma_format = DMA_FORMAT_A2R10G10B10;
        break;
    case V4L2_PIX_FMT_NV12:
        if ( plane_no == 0 )
            dma_format = DMA_FORMAT_NV12_Y;
        else if ( plane_no == 1 )
            dma_format = DMA_FORMAT_NV12_UV;
        break;
    default:
        LOG( LOG_CRIT, "Unknown pixelformat: %d", pixelformat );
        break;
    };

    return dma_format;
}

static int isp_v4l2_stream_get_plane( struct vb2_buffer *vb,
                                      struct v4l2_pix_format_mplane *pix_mp,
                                      aframe_t *aframe, uint32_t plane_no,
                                      isp_v4l2_buffer_t *pbuf )
{
    dma_addr_t addr;
    addr = vb2_dma_contig_plane_dma_addr( vb, plane_no );
    aframe->virt_addr = vb2_plane_vaddr( vb, plane_no );
    aframe->address = (uint32_t)addr;
    pr_debug("plane %d, phy_addr 0x%x\n", plane_no, aframe->address);
    aframe->status = dma_buf_empty;
    aframe->type = isp_v4l2_format_to_dma_output( pix_mp->pixelformat, plane_no );
    aframe->width = pix_mp->width;
    aframe->height = pix_mp->height;
    aframe->line_offset = pix_mp->plane_fmt[plane_no].bytesperline;
    aframe->size = pix_mp->plane_fmt[plane_no].sizeimage;

    return 0;
}

#ifndef HOBOT_DMA_WRITER_FRAME
static void isp_v4l2_stream_put_plane( struct vb2_buffer *vb,
                                       aframe_t *aframe, uint32_t plane_no )
{
    dma_addr_t addr;

    if ( plane_no >= vb->num_planes ) {
        LOG( LOG_CRIT, "Invalid plane_no: %d", plane_no );
        return;
    }

    addr = vb2_dma_contig_plane_dma_addr( vb, plane_no );
    addr -= ISP_SOC_DMA_BUS_OFFSET;

    if ( aframe->address != (uint32_t)addr ) {
        LOG( LOG_CRIT, "Bad dma address %x %x", aframe->address, (uint32_t)addr );
        return;
    }

    aframe->status = dma_buf_purge;
}
#endif

#if ISP_V4L2_DMA_COHERENT_DUMMY_ALLOC == 0
extern struct ion_device *hb_ion_dev;
void *callback_dma_alloc_coherent( uint32_t ctx_id, uint64_t size, uint64_t *dma_addr )
{
    int ret = 0;
    void *virt_addr = NULL;
    isp_v4l2_dev_t *isp_v4l2_dev = isp_v4l2_get_dev( ctx_id );

	isp_v4l2_dev->client = ion_client_create(hb_ion_dev, "isp_temper");
	if (IS_ERR(isp_v4l2_dev->client)) {
		LOG(LOG_ERR, "isp_temper ion client create failed.");
		goto out1;
	}

	isp_v4l2_dev->handle = ion_alloc(isp_v4l2_dev->client,
			size, 0, ION_HEAP_CARVEOUT_MASK, 0);
	if (IS_ERR(isp_v4l2_dev->handle)) {
		LOG(LOG_ERR, "isp_temper ion handle create failed.");
		goto out1;
	}

	ret = ion_phys(isp_v4l2_dev->client, isp_v4l2_dev->handle->id,
			dma_addr, &isp_v4l2_dev->mem_size);
	if (ret) {
		LOG(LOG_ERR, "ion_phys get phy address failed.");
		goto out2;
	}
	virt_addr = ion_map_kernel(isp_v4l2_dev->client, isp_v4l2_dev->handle);
	if (IS_ERR(virt_addr)) {
		LOG(LOG_ERR, "ion_map failed.");
		goto out2;
	}

	return virt_addr;

out2:
	ion_free(isp_v4l2_dev->client, isp_v4l2_dev->handle);
	isp_v4l2_dev->mem_size = 0;
    //global destory, do not call when ion_alloc failed
    ion_client_destroy(isp_v4l2_dev->client);
out1:
	return NULL;
}

void callback_dma_free_coherent( uint32_t ctx_id, uint64_t size, void *virt_addr, uint64_t dma_addr )
{
    isp_v4l2_dev_t *isp_v4l2_dev = isp_v4l2_get_dev( ctx_id );

    if (isp_v4l2_dev->client != NULL && IS_ERR(isp_v4l2_dev->client) == 0) {
	    if (isp_v4l2_dev->handle != NULL && IS_ERR(isp_v4l2_dev->handle) == 0) {
		    ion_unmap_kernel(isp_v4l2_dev->client, isp_v4l2_dev->handle);
		    ion_free(isp_v4l2_dev->client, isp_v4l2_dev->handle);
	    }
	    ion_client_destroy(isp_v4l2_dev->client);
        isp_v4l2_dev->client = NULL;
        isp_v4l2_dev->handle = NULL;
    }
}
#else
/*
 * This dummy allocator is used due to the current limitation
 * of ISP reserved memory. When we get rid of this limitation
 * we can use the above callbacks that get memory from Linux
 */
static uint64_t dummy_base = ISP_V4L2_DMA_COHERENT_DUMMY_ALLOC_BASE;
static uint64_t dummy_size = 0;
static uint64_t dummy_total = ISP_V4L2_DMA_COHERENT_DUMMY_ALLOC_SIZE;
void *callback_dma_alloc_coherent( uint32_t ctx_id, uint64_t size, uint64_t *dma_addr )
{
    void *virt_addr;
    uint64_t addr;

    if ( dummy_size + size > dummy_total ) {
        LOG( LOG_ERR, "Not enough memory" );
        return NULL;
    }

    addr = dummy_base + dummy_size;
    dummy_size += size;

    *dma_addr = addr;

    /* compute bus address */
    *dma_addr -= ISP_SOC_DMA_BUS_OFFSET;

    /* compute virt address */
    virt_addr = (void *)addr;

    return virt_addr;
}

void callback_dma_free_coherent( uint32_t ctx_id, uint64_t size, void *virt_addr, uint64_t dma_addr )
{
}
#endif

/*
 * get one frame from vb2
 */
int callback_stream_get_frame( uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes )
{
    int rc;
    isp_v4l2_stream_type_t v4l2_type;
    isp_v4l2_stream_t *pstream;
    isp_v4l2_buffer_t *pbuf = NULL;
    struct v4l2_format *v4l2_fmt;
    struct v4l2_pix_format_mplane *pix_mp;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer *vvb;
#endif
    struct vb2_buffer *vb;
    int i, factor;
    int cnt = 0;
    struct list_head *p, *n;
    acamera_context_ptr_t p_ctx = acamera_get_ctx_ptr(ctx_id);

    LOG(LOG_INFO,"++ : ctx_id = %d, type = %d, num_planes = %ld",ctx_id,type,num_planes);
    for ( i = 0; i < num_planes; i++ ) {
        aframes[i].status = dma_buf_purge;
    }

    v4l2_type = fw_to_isp_v4l2_stream_type( type );
    if ( v4l2_type == V4L2_STREAM_TYPE_MAX ) {
        LOG(LOG_INFO,"v4l2_type error (v4l2_type = %d, type = %d)",v4l2_type,type);
        return -1;
    }

    /* find stream pointer */
    rc = isp_v4l2_find_stream( &pstream, ctx_id, v4l2_type );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "can't find stream on ctx %d (errno = %d)",
             ctx_id, rc );
        return -1;
    }

    /* check if stream is on */
    if ( !pstream->stream_started ) {
        LOG( LOG_ERR, "[Stream#%d] type: %d is not started yet on ctx %d",
             pstream->stream_id, type, ctx_id );
        return -1;
    }

    v4l2_fmt = &pstream->cur_v4l2_fmt;

    /* try to get an active buffer from vb2 queue  */
    spin_lock( &pstream->slock );
//debug start
    list_for_each_safe(p, n, &pstream->stream_buffer_list)
	cnt++;
    pr_debug("stream_buffer_list count %d", cnt);
//debug end

    cnt = 0;
    list_for_each_safe(p, n, &pstream->stream_buffer_list_busy)
	cnt++;
    pr_debug("stream_buffer_list_busy count %d", cnt);

    if (p_ctx->p_gfw->sif_isp_offline == 1)
        factor = 1;
    else
        factor = 2;

    if (cnt >= factor) {
        pbuf = list_entry(pstream->stream_buffer_list_busy.next, isp_v4l2_buffer_t, list);
        list_del(&pbuf->list);
        list_add_tail(&pbuf->list, &pstream->stream_buffer_list);
    }

    if (!list_empty( &pstream->stream_buffer_list)) {
        pbuf = list_entry( pstream->stream_buffer_list.next, isp_v4l2_buffer_t, list );
        list_del( &pbuf->list );
        list_add_tail( &pbuf->list, &pstream->stream_buffer_list_busy );
    }
    spin_unlock( &pstream->slock );

    if ( !pbuf ) {
        p_ctx->sts.free_to_busy_failed_cnt++;
        LOG( LOG_ERR, "[Stream#%d] ctx_id %d no buffer",
             pstream->stream_id, ctx_id );
        return -1;
    }

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    vvb = &pbuf->vvb;
    vb = &vvb->vb2_buf;
#else
    vb = &pbuf->vb;
#endif

    if ( v4l2_fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ) {
        pix_mp = &v4l2_fmt->fmt.pix_mp;

        if ( pix_mp->num_planes != vb->num_planes ) {
            LOG( LOG_CRIT, "Different num of planes: %d %d",
                 pix_mp->num_planes, vb->num_planes );
            return -1;
        }

        if ( num_planes < vb->num_planes ) {
            LOG( LOG_CRIT, "Bad num of planes: num_planes: %d vb->num_planes: %d",
                 num_planes, vb->num_planes );
            return -1;
        }

        for ( i = 0; i < vb->num_planes; i++ ) {
            isp_v4l2_stream_get_plane( vb, pix_mp, &aframes[i], i, pbuf );
        }
    } else {
        LOG( LOG_CRIT, "Invalid v4l2_fmt_type: %d", v4l2_fmt->type );
        return -1;
    }
    p_ctx->sts.free_to_busy_cnt++;
    LOG(LOG_INFO,"-- : ctx_id = %d, type = %d, num_planes = %ld",ctx_id,type,num_planes);
    return 0;
}

/*
 * give back frame buffer to vb2 for user to dqbuf.
 * called when isp fr y/uv done.
 */
int callback_stream_put_frame( uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes, uint8_t flag )
{
    int rc;
    int cnt = 0, factor = 0;
    isp_v4l2_stream_type_t v4l2_type;
    isp_v4l2_stream_t *pstream;
    isp_v4l2_buffer_t *pbuf = NULL;
    struct list_head *p, *n;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer *vvb;
#endif
    struct vb2_buffer *vb;
    acamera_context_ptr_t p_ctx = acamera_get_ctx_ptr(ctx_id);
    dma_writer_fsm_ptr_t p_fsm = NULL;
    metadata_t *metadata_cb = NULL;

    v4l2_type = fw_to_isp_v4l2_stream_type( type );
    if ( v4l2_type == V4L2_STREAM_TYPE_MAX )
        return -1;

    /* find stream pointer */
    rc = isp_v4l2_find_stream( &pstream, ctx_id, v4l2_type );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "can't find stream on ctx %d (errno = %d)",
             ctx_id, rc );
        return -1;
    }

    /* check if stream is on */
    if ( !pstream->stream_started ) {
        LOG( LOG_ERR, "[Stream#%d] type: %d is not started yet on ctx %d",
             pstream->stream_id, type, ctx_id );
        return -1;
    }

    /* try to get an active buffer from vb2 queue  */
    spin_lock( &pstream->slock );
// debug start
    list_for_each_safe(p, n, &pstream->stream_buffer_list_busy)
	cnt++;
    pr_debug("stream_buffer_list_busy count %d", cnt);
// debug end

    if (p_ctx->p_gfw->sif_isp_offline == 1)
        factor = 0;
    else
        factor = 1;

    if (cnt > factor) {
        pbuf = list_entry( pstream->stream_buffer_list_busy.next, isp_v4l2_buffer_t, list );
        list_del( &pbuf->list );
    }
    spin_unlock( &pstream->slock );

    if ( !pbuf ) {
        p_ctx->sts.busy_to_done_failed_cnt++;
        LOG( LOG_ERR, "[Stream#%d] ctx_id %d no buffer",
             pstream->stream_id, ctx_id );
        acamera_dma_wr_check();
        return -1;
    }

    p_fsm = (dma_writer_fsm_ptr_t)(p_ctx->fsm_mgr.fsm_arr[FSM_ID_DMA_WRITER]->p_fsm);
    metadata_cb = dma_writer_return_metadata(p_fsm->handle, dma_fr);

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    vvb = &pbuf->vvb;
    vb = &vvb->vb2_buf;
    vvb->sequence = metadata_cb->frame_id;
    vvb->field = V4L2_FIELD_NONE;
#else
    vb = &pbuf->vb;
    vb->v4l2_buf.sequence = aframes[0].frame_id;
    vb->v4l2_buf.field = V4L2_FIELD_NONE;
#endif

    if ( num_planes < vb->num_planes ) {
        LOG( LOG_CRIT, "Bad num of planes: num_planes: %d vb->num_planes: %d",
             num_planes, vb->num_planes );
        return -1;
    }

#ifndef HOBOT_DMA_WRITER_FRAME
    int i;
    for ( i = 0; i < vb->num_planes; i++ ) {
        if ( aframes[i].status == dma_buf_busy )
            isp_v4l2_stream_put_plane( vb, &aframes[i], i );
    }
#endif
    int i;
    for ( i = 0; i < vb->num_planes; i++ ) {
        dma_addr_t addr;
        addr = vb2_dma_contig_plane_dma_addr( vb, i );
        pr_debug("plane %d, phy_addr 0x%x\n", i, (uint32_t)addr);
    }

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    // vb->timestamp = ktime_get_ns();
    vb->timestamp = metadata_cb->timestamps;
    vvb->timecode.type = metadata_cb->tv_sec;
    vvb->timecode.flags = metadata_cb->tv_usec;
#else
    v4l2_get_timestamp( &vb->v4l2_buf.timestamp );
#endif

    if (flag == 0) {
        p_ctx->sts.busy_to_done_cnt++;
        /* Put buffer back to vb2 queue */
        vb2_buffer_done( vb, VB2_BUF_STATE_DONE );
        /* Notify buffer ready */
        isp_v4l2_notify_event( pstream->ctx_id, pstream->stream_id, V4L2_EVENT_ACAMERA_FRAME_READY );
    } else {
        vb2_buffer_done( vb, VB2_BUF_STATE_ERROR );
    }

    return 0;
}

int callback_stream_release_frame( uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes )
{
    int rc;
    int cnt = 0;
    isp_v4l2_stream_type_t v4l2_type;
    isp_v4l2_stream_t *pstream;
    isp_v4l2_buffer_t *pbuf = NULL;
    struct list_head *p, *n;
#if 0
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer *vvb;
#endif
    struct vb2_buffer *vb;
#endif

    v4l2_type = fw_to_isp_v4l2_stream_type( type );
    if ( v4l2_type == V4L2_STREAM_TYPE_MAX )
        return -1;

    /* find stream pointer */
    rc = isp_v4l2_find_stream( &pstream, ctx_id, v4l2_type );
    if ( rc < 0 ) {
        LOG( LOG_DEBUG, "can't find stream on ctx %d (errno = %d)",
             ctx_id, rc );
        return -1;
    }

    /* check if stream is on */
    if ( !pstream->stream_started ) {
        LOG( LOG_DEBUG, "[Stream#%d] type: %d is not started yet on ctx %d",
             pstream->stream_id, type, ctx_id );
        return -1;
    }

    /* move node from busy list to free list */
    spin_lock( &pstream->slock );
// debug start
    list_for_each_safe(p, n, &pstream->stream_buffer_list_busy)
	cnt++;
    pr_debug("stream_buffer_list_busy count %d", cnt);
// debug end
    if ( !list_empty( &pstream->stream_buffer_list_busy ) ) {
        pbuf = list_entry( pstream->stream_buffer_list_busy.next, isp_v4l2_buffer_t, list );
        list_move_tail(&pbuf->list, &pstream->stream_buffer_list);
    }
    spin_unlock( &pstream->slock );

#if 0	//open this if user need sense
    if ( !pbuf ) {
        LOG( LOG_INFO, "[Stream#%d] type: %d no empty buffers",
             pstream->stream_id, type );
        return -1;
    }

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    vvb = &pbuf->vvb;
    vb = &vvb->vb2_buf;
    vvb->sequence = aframes[0].frame_id;
    vvb->field = V4L2_FIELD_NONE;
#else
    vb = &pbuf->vb;
    vb->v4l2_buf.sequence = aframes[0].frame_id;
    vb->v4l2_buf.field = V4L2_FIELD_NONE;
#endif

    /* Put buffer back to vb2 queue */
    vb2_buffer_done( vb, VB2_BUF_STATE_ERROR );
#endif

    return 0;
}

/* ----------------------------------------------------------------
 * Stream control interface
 */

/* sensor static informations */
static isp_v4l2_stream_common g_stream_common[FIRMWARE_CONTEXT_NUMBER];

int isp_v4l2_stream_init_static_resources( uint32_t ctx_id )
{
    isp_v4l2_stream_common *sc = &( g_stream_common[ctx_id] );
    int i;

    /* initialize stream common field */
    memset( sc, 0, sizeof( isp_v4l2_stream_common ) );
    //fw_intf_isp_get_sensor_info( ctx_id, &sc->sensor_info );
    sc->snapshot_sizes.frmsize_num = sc->sensor_info.preset_num;
    for ( i = 0; i < sc->sensor_info.preset_num; i++ ) {
        sc->snapshot_sizes.frmsize[i].width = sc->sensor_info.preset[i].width;
        sc->snapshot_sizes.frmsize[i].height = sc->sensor_info.preset[i].height;
    }

    return 0;
}

void isp_v4l2_stream_deinit_static_resources( void )
{
}

int isp_v4l2_stream_init( isp_v4l2_stream_t **ppstream, int stream_id, int ctx_id )
{
    isp_v4l2_stream_t *new_stream = NULL;
    //int current_sensor_preset;
    LOG( LOG_INFO, "[Stream#%d] Initializing stream ...", stream_id );

    /* allocate isp_v4l2_stream_t */
    new_stream = kzalloc( sizeof( isp_v4l2_stream_t ), GFP_KERNEL );
    if ( new_stream == NULL ) {
        LOG( LOG_ERR, "[Stream#%d] Failed to allocate isp_v4l2_stream_t.", stream_id );
        return -ENOMEM;
    }

    /* set default format */

    //all stream multiplanar
    new_stream->cur_v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    /* set input stream info */
    new_stream->stream_common = &( g_stream_common[ctx_id] );

    /* init control fields */
    new_stream->ctx_id = ctx_id;
    new_stream->stream_id = stream_id;
    new_stream->stream_type = V4L2_STREAM_TYPE_MAX;
    new_stream->stream_started = 0;
    new_stream->last_frame_id = 0xFFFFFFFF;

    //format new stream to default isp settings
    isp_v4l2_stream_try_format( new_stream, &( new_stream->cur_v4l2_fmt ) );

    /* init list */
    INIT_LIST_HEAD( &new_stream->stream_buffer_list );
    INIT_LIST_HEAD( &new_stream->stream_buffer_list_busy );

    /* init locks */
    spin_lock_init( &new_stream->slock );

    /* return stream private ptr to caller */
    *ppstream = new_stream;

    return 0;
}

void dma_writer_clear(uint32_t ctx_id);
void isp_v4l2_stream_deinit( isp_v4l2_stream_t *pstream )
{
    if ( !pstream ) {
        LOG( LOG_ERR, "Null stream passed" );
        return;
    }

    LOG( LOG_INFO, "[Stream#%d] Deinitializing stream ...", pstream->stream_id );

dma_writer_clear(pstream->ctx_id);

    /* do stream-off first if it's on */
    isp_v4l2_stream_off( pstream );

    /* release fw_info */
    if ( pstream ) {
        kzfree( pstream );
        pstream = NULL;
    }
}

static void isp_v4l2_stream_buffer_list_release( isp_v4l2_stream_t *pstream,
                                                 struct list_head *stream_buffer_list )
{
    isp_v4l2_buffer_t *buf;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer *vvb;
#endif
    struct vb2_buffer *vb;
    unsigned int buf_index;

    while ( !list_empty( stream_buffer_list ) ) {
        buf = list_entry( stream_buffer_list->next,
                          isp_v4l2_buffer_t, list );
        list_del( &buf->list );

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
        vvb = &buf->vvb;
        vb = &vvb->vb2_buf;

        buf_index = vb->index;
#else
        vb = &buf->vb;

        buf_index = vb->v4l2_buf.index;
#endif

        vb2_buffer_done( vb, VB2_BUF_STATE_ERROR );

        LOG( LOG_INFO, "[Stream#%d] vid_cap buffer %d done",
             pstream->stream_id, buf_index );
    }
}

int isp_v4l2_stream_on( isp_v4l2_stream_t *pstream )
{
    if ( !pstream ) {
        LOG( LOG_ERR, "Null stream passed" );
        return -EINVAL;
    }

    pr_debug("[Stream#%d] called\n", pstream->stream_id );

/* for now, we need memcpy */
#if ISP_HAS_META_CB
    if ( pstream->stream_type != V4L2_STREAM_TYPE_META )
#endif
    {
        /* Resets frame counters */
        pstream->fw_frame_seq_count = 0;
    }
#if ISP_HAS_META_CB
    else { //metadata has no thread
        atomic_set( &pstream->running, 0 );
    }
#endif

    /* hardware stream on */
    if ( fw_intf_stream_start( pstream->ctx_id, pstream->stream_type ) < 0 )
        return -1;

    /* control fields update */
    pstream->stream_started = 1;

    /* get one vb2 buffer config to dma writer - for online only */
    acamera_fsm_mgr_t *instance = &(((acamera_context_ptr_t)acamera_get_ctx_ptr(pstream->ctx_id))->fsm_mgr);
    acamera_context_ptr_t p_ctx = acamera_get_ctx_ptr(pstream->ctx_id);
    if (p_ctx && instance && instance->reserved && p_ctx->p_gfw->sif_isp_offline == 0) { //dma writer on
        dma_handle *dh = NULL;
        dh = ((dma_writer_fsm_const_ptr_t)(p_ctx->fsm_mgr.fsm_arr[FSM_ID_DMA_WRITER]->p_fsm))->handle;
        dma_writer_configure_pipe(&dh->pipe[dma_fr]);
        // acamera_general_interrupt_hanlder(ACAMERA_MGR2CTX_PTR(instance), ACAMERA_IRQ_FRAME_WRITER_FR);
    }

    return 0;
}

void isp_v4l2_stream_off( isp_v4l2_stream_t *pstream )
{
    if ( !pstream ) {
        LOG( LOG_ERR, "Null stream passed" );
        return;
    }

    LOG( LOG_INFO, "[Stream#%d] called", pstream->stream_id );

    // control fields update
    pstream->stream_started = 0;

    fw_intf_stream_stop( pstream->ctx_id, pstream->stream_type );

#if ISP_HAS_META_CB
    if ( pstream->stream_type == V4L2_STREAM_TYPE_META ) {
        while ( atomic_read( &pstream->running ) > 0 ) { //metadata has no thread
            LOG( LOG_INFO, "[Stream#%d] still running %d !", pstream->stream_id, atomic_read( &pstream->running ) );
            schedule();
        }
        atomic_set( &pstream->running, -1 );
    }
#endif

    /* Release all active buffers */
    acamera_fsm_mgr_t *instance = &(((acamera_context_ptr_t)acamera_get_ctx_ptr(pstream->ctx_id))->fsm_mgr);
    if (instance->reserved) { //dma writer on
        spin_lock( &pstream->slock );
        isp_v4l2_stream_buffer_list_release( pstream, &pstream->stream_buffer_list );
        isp_v4l2_stream_buffer_list_release( pstream, &pstream->stream_buffer_list_busy );
        spin_unlock( &pstream->slock );
    }
}


/* ----------------------------------------------------------------
 * Stream configuration interface
 */
static isp_v4l2_fmt_t *isp_v4l2_stream_find_format( uint32_t pixelformat )
{
    isp_v4l2_fmt_t *fmt;
    unsigned int i;

    for ( i = 0; i < ARRAY_SIZE( isp_v4l2_supported_formats ); i++ ) {
        fmt = &isp_v4l2_supported_formats[i];

        if ( fmt->fourcc == pixelformat )
            return fmt;
    }

    return NULL;
}

int isp_v4l2_stream_enum_framesizes( isp_v4l2_stream_t *pstream, struct v4l2_frmsizeenum *fsize )
{
    LOG( LOG_INFO, "[Stream#%d] index: %d, pixel_format: 0x%x.\n",
         pstream->stream_id, fsize->index, fsize->pixel_format );

    if ( !isp_v4l2_stream_find_format( fsize->pixel_format ) )
        return -EINVAL;

    /* check index */
    if ( fsize->index >= pstream->stream_common->snapshot_sizes.frmsize_num ) {
        LOG( LOG_INFO, "[Stream#%d] index (%d) should be smaller than %lu.",
             pstream->stream_id, fsize->index, pstream->stream_common->snapshot_sizes.frmsize_num );
        return -EINVAL;
    }


    /* return framesize */
    fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    fsize->discrete = pstream->stream_common->snapshot_sizes.frmsize[fsize->index];

    return 0;
}

int isp_v4l2_stream_enum_format( isp_v4l2_stream_t *pstream, struct v4l2_fmtdesc *f )
{
    const isp_v4l2_fmt_t *fmt;
    int desc_size = 0;

    /* check index */
    if ( f->index >= ARRAY_SIZE( isp_v4l2_supported_formats ) ) {
        LOG( LOG_INFO, "[Stream#%d] index (%d) should be smaller than %lu.",
             pstream->stream_id, f->index, ARRAY_SIZE( isp_v4l2_supported_formats ) );
        return -EINVAL;
    }

    /* get format from index */
    fmt = &isp_v4l2_supported_formats[f->index];

    /* check description length */
    if ( sizeof( fmt->name ) > sizeof( f->description ) )
        desc_size = sizeof( f->description );
    else
        desc_size = sizeof( fmt->name );

    /* reset flag */
    f->flags = 0;

    /* copy description */
    strlcpy( f->description, fmt->name, desc_size );

    /* copy format code */
    f->pixelformat = fmt->fourcc;

    LOG( LOG_INFO, "[Stream#%d] index: %d, format: 0x%x, desc: %s.\n",
         pstream->stream_id, f->index, f->pixelformat, f->description );

    return 0;
}

int isp_v4l2_stream_try_format( isp_v4l2_stream_t *pstream, struct v4l2_format *f )
{
    isp_v4l2_fmt_t *tfmt;
    int i;
    LOG( LOG_INFO, "[Stream#%d] try fmt type: %u, pixelformat: 0x%x, width: %u, height: %u.\n",
         pstream->stream_id, f->type, f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width, f->fmt.pix_mp.height );

    /* check format and modify */
    tfmt = isp_v4l2_stream_find_format( f->fmt.pix_mp.pixelformat );
    if ( !tfmt ) {
        LOG( LOG_INFO, "[Stream#%d] format 0x%08x is not supported, setting default format 0x%08x.\n",
             pstream->stream_id, f->fmt.pix.pixelformat, ISP_DEFAULT_FORMAT );
        f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        f->fmt.pix_mp.pixelformat = ISP_DEFAULT_FORMAT;
        tfmt = isp_v4l2_stream_find_format( f->fmt.pix_mp.pixelformat );
    }

    //corect the exposure number here
    if ( tfmt->fourcc == V4L2_PIX_FMT_SBGGR16 ) {
        uint32_t spreset = 0, exposures_preset = 0, rev_val = 0;

        if ( fw_intf_is_isp_started() ) {
            acamera_command( pstream->ctx_id, TSENSOR, SENSOR_PRESET, 0, COMMAND_GET, &spreset );
            acamera_command( pstream->ctx_id, TSENSOR, SENSOR_INFO_PRESET, spreset, COMMAND_SET, &rev_val );
            acamera_command( pstream->ctx_id, TSENSOR, SENSOR_INFO_EXPOSURES, 0, COMMAND_GET, &exposures_preset );

            LOG( LOG_INFO, "[Stream#%d] Changing the number of planes according preset %d to exposures %d=>%d.\n", pstream->stream_id, spreset, tfmt->planes, exposures_preset );
            tfmt->planes = (uint8_t)exposures_preset;
        } else {
            tfmt->planes = 1;
        }
    }
/* adjust width, height for META stream */
#if ISP_HAS_META_CB
    if ( f->fmt.pix.pixelformat == ISP_V4L2_PIX_FMT_META ) {
        f->fmt.pix.width = ISP_V4L2_METADATA_SIZE;
        f->fmt.pix.height = 1;
    } else
#endif
    {
        if ( f->fmt.pix.width == 0 || f->fmt.pix.height == 0 ) {
            if ( fw_intf_is_isp_started() ) {
                uint32_t width_cur = 0, height_cur = 0;
                acamera_command( pstream->ctx_id, TSENSOR, SENSOR_WIDTH, 0, COMMAND_GET, &width_cur );
                acamera_command( pstream->ctx_id, TSENSOR, SENSOR_HEIGHT, 0, COMMAND_GET, &height_cur );
                f->fmt.pix.width = width_cur;
                f->fmt.pix.height = height_cur;
            } else {
                f->fmt.pix.width = 1920;
                f->fmt.pix.height = 1080;
            }
        }
        v4l_bound_align_image( &f->fmt.pix.width, 48, ISP_V4L2_MAX_WIDTH, 2,
                               &f->fmt.pix.height, 32, ISP_V4L2_MAX_HEIGHT, 1, 0 );
    }

    f->fmt.pix.field = V4L2_FIELD_NONE;

    pr_debug("after align w %d, h %d\n", f->fmt.pix.width, f->fmt.pix.height);

    //all stream multiplanar
    f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    f->fmt.pix_mp.num_planes = tfmt->planes;
    f->fmt.pix_mp.colorspace = ( tfmt->is_yuv ) ? V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_SRGB;
    for ( i = 0; i < tfmt->planes; i++ ) {
        //( ( ( f->fmt.pix_mp.width * tfmt->depth / 8 ) + 127 ) >> 7 ) << 7;
        f->fmt.pix_mp.plane_fmt[i].bytesperline = ALIGN_UP16(f->fmt.pix_mp.width);
        f->fmt.pix_mp.plane_fmt[i].sizeimage = f->fmt.pix_mp.height * f->fmt.pix_mp.plane_fmt[i].bytesperline;
        if (i == 1 && f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_NV12)
		f->fmt.pix_mp.plane_fmt[i].sizeimage = f->fmt.pix_mp.plane_fmt[i].sizeimage / 2;
        memset( f->fmt.pix_mp.plane_fmt[i].reserved, 0, sizeof( f->fmt.pix_mp.plane_fmt[i].reserved ) );
        pr_debug("plane %d, bytesperline %d, sizeimage %d\n", i,
            f->fmt.pix_mp.plane_fmt[i].bytesperline, f->fmt.pix_mp.plane_fmt[i].sizeimage);
    }

    return 0;
}

int isp_v4l2_stream_get_format( isp_v4l2_stream_t *pstream, struct v4l2_format *f )
{
    if ( !pstream ) {
        LOG( LOG_ERR, "Null stream passed" );
        return -EINVAL;
    }

    *f = pstream->cur_v4l2_fmt;

    LOG( LOG_INFO, "[Stream#%d]   - GET fmt - width: %4u, height: %4u, format: 0x%x.",
         pstream->stream_id,
         f->fmt.pix_mp.width,
         f->fmt.pix_mp.height,
         f->fmt.pix_mp.pixelformat );

    if ( f->fmt.pix_mp.width == 0 || f->fmt.pix_mp.height == 0 || f->fmt.pix_mp.pixelformat == 0 ) { //not formatted yet
        LOG( LOG_NOTICE, "Compliance error, uninitialized format" );
    }

    return 0;
}

extern int sensor_info_check_valid(uint32_t ctx_id, struct v4l2_format *f);
extern int sensor_info_check_exist(uint32_t ctx_id, struct v4l2_format *f);
extern void sensor_info_fill(uint32_t ctx_id, struct v4l2_format *f);
extern int fw_intf_cfa_pattern_ctrl(uint32_t ctx_id, uint8_t ctrl_val);
extern int fw_intf_sif_isp_offline_set(uint32_t ctx_id, uint8_t ctrl_val);
int isp_v4l2_stream_set_format( isp_v4l2_stream_t *pstream, struct v4l2_format *f, struct vb2_queue *q )
{
    int rc = 0;

    if ( !pstream ) {
        LOG( LOG_ERR, "Null stream passed" );
        return -EINVAL;
    }

    pr_debug("[ctx#%d] [Stream#%d] - SET fmt - \
        width: %4u, height: %4u, exp %d, bits %d, cfa %d, offline %d, format: 0x%x.",
         pstream->ctx_id,
         pstream->stream_id,
         f->fmt.pix_mp.width,
         f->fmt.pix_mp.height,
         f->fmt.pix_mp.reserved[0],
         f->fmt.pix_mp.reserved[1],
         f->fmt.pix_mp.reserved[2],
         f->fmt.pix_mp.reserved[3],
         f->fmt.pix_mp.pixelformat );

    /* try format first */
    if (q && vb2_is_busy( q ) == 0)
        isp_v4l2_stream_try_format( pstream, f );

    /* set stream type*/
    switch ( f->fmt.pix.pixelformat ) {
    case V4L2_PIX_FMT_RGB32:
    case ISP_V4L2_PIX_FMT_ARGB2101010:
    case V4L2_PIX_FMT_NV12:
        pstream->stream_type = pstream->stream_id;
        break;
#if ISP_HAS_RAW_CB
    case V4L2_PIX_FMT_SBGGR16:
        pstream->stream_type = V4L2_STREAM_TYPE_RAW;
        break;
#endif
#if ISP_HAS_META_CB
    case ISP_V4L2_PIX_FMT_META:
        pstream->stream_type = V4L2_STREAM_TYPE_META;
        break;
#endif
    default:
        LOG( LOG_ERR, "Shouldn't be here after try_format()." );
        return -EINVAL;
    }

    if (sensor_info_check_valid(pstream->ctx_id, f) == 0) {
        pr_err("exposures %d or bit_width %d invalid\n",
            f->fmt.pix_mp.reserved[0], f->fmt.pix_mp.reserved[1]);
        return -EINVAL;
    }

    if (sensor_info_check_exist(pstream->ctx_id, f) == 0)
        sensor_info_fill(pstream->ctx_id, f);

	 /* update format */
    rc = fw_intf_stream_set_output_format( pstream->ctx_id, pstream->stream_type, f->fmt.pix_mp.pixelformat );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "set format failed ! (rc = %d)", rc );
        return rc;
    }
    /* update resolution */
    rc = fw_intf_stream_set_resolution( pstream->ctx_id, &pstream->stream_common->sensor_info,
                                        pstream->stream_type, f );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "set resolution failed ! (rc = %d)", rc );
        return rc;
    }

    /* update format field */
    pstream->cur_v4l2_fmt = *f;

    fw_intf_cfa_pattern_ctrl(pstream->ctx_id, f->fmt.pix_mp.reserved[2]);
    fw_intf_sif_isp_offline_set(pstream->ctx_id, f->fmt.pix_mp.reserved[3]);

    LOG( LOG_NOTICE, "[Stream#%d]   - New fmt - width: %4u, height: %4u, format: 0x%x, type: %5u. ",
         pstream->stream_id,
         pstream->cur_v4l2_fmt.fmt.pix_mp.width,
         pstream->cur_v4l2_fmt.fmt.pix_mp.height,
         pstream->cur_v4l2_fmt.fmt.pix_mp.pixelformat,
         pstream->cur_v4l2_fmt.type );

    return 0;
}
