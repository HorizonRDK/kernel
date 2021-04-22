/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_JPU_USER_H__
#define __HOBOT_JPU_USER_H__

#include <linux/fs.h>
#include <linux/types.h>
#include "../vpu/inc/hb_media_codec.h"

#define JPU_DEV_NAME                    "jpu"
//#define MAX_NUM_JPU_INSTANCE            64
//#define MAX_NUM_JPU_CORE                1
//#define MAX_INST_HANDLE_SIZE            48

typedef struct _hb_jpu_drv_buffer {
	unsigned int size;
	unsigned long phys_addr;
	/* kernel logical address in use kernel */
	unsigned long base;
	/* virtual user space address */
	unsigned long virt_addr;
} hb_jpu_drv_buffer_t;

typedef struct _hb_jpu_drv_inst {
	unsigned int inst_idx;
	/* for output only */
	int inst_open_count;
} hb_jpu_drv_inst_t;

typedef struct _hb_jpu_drv_intr {
	unsigned int timeout;
	int intr_reason;
	unsigned int inst_idx;
} hb_jpu_drv_intr_t;

typedef struct _hb_jpu_ctx_info {
	int valid;
	media_codec_context_t context;
	// decoder
	mc_jpeg_enc_params_t jpeg_params;
	mc_mjpeg_enc_params_t mjpeg_params;
	mc_rate_control_params_t rc_params;
} hb_jpu_ctx_info_t;

typedef struct _hb_jpu_status_info {
	unsigned int inst_idx;
	mc_inter_status_t status;
	// encoder
	mc_mjpeg_jpeg_output_stream_info_t stream_info;
	// decoder
	mc_mjpeg_jpeg_output_frame_info_t frame_info;
} hb_jpu_status_info_t;

typedef struct _hb_jpu_ion_fd_map {
	int fd;	/* ion fd */
	uint64_t iova;	/* IO virtual address */
} hb_jpu_ion_fd_map_t;

typedef struct _hb_jpu_ion_phys_map {
	uint64_t phys_addr;	/* physical address */
	unsigned int size;
	uint64_t iova;	/* IO virtual address */
} hb_jpu_ion_phys_map_t;

typedef enum _hb_jpu_event_e {
	JPU_EVENT_NONE = 0,
	JPU_PIC_DONE = 1,
	JPU_INST_CLOSED = 2,
	JPU_INST_INTERRUPT = 3,
} hb_jpu_event_t;

#define JDI_IOCTL_MAGIC  'J'

#define JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY          \
    _IO(JDI_IOCTL_MAGIC, 0)
#define JDI_IOCTL_FREE_PHYSICAL_MEMORY              \
    _IO(JDI_IOCTL_MAGIC, 1)
#define JDI_IOCTL_WAIT_INTERRUPT                    \
    _IO(JDI_IOCTL_MAGIC, 2)
#define JDI_IOCTL_SET_CLOCK_GATE                    \
    _IO(JDI_IOCTL_MAGIC, 3)
#define JDI_IOCTL_RESET                             \
    _IO(JDI_IOCTL_MAGIC, 4)
#define JDI_IOCTL_GET_INSTANCE_POOL                 \
    _IO(JDI_IOCTL_MAGIC, 5)
#define JDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO    \
    _IO(JDI_IOCTL_MAGIC, 6)
#define JDI_IOCTL_GET_REGISTER_INFO                 \
    _IO(JDI_IOCTL_MAGIC, 7)
#define JDI_IOCTL_OPEN_INSTANCE                     \
    _IO(JDI_IOCTL_MAGIC, 8)
#define JDI_IOCTL_CLOSE_INSTANCE                    \
    _IO(JDI_IOCTL_MAGIC, 9)
#define JDI_IOCTL_GET_INSTANCE_NUM                  \
    _IO(JDI_IOCTL_MAGIC, 10)
#define JDI_IOCTL_ALLOCATE_INSTANCE_ID              \
    _IO(JDI_IOCTL_MAGIC, 14)
#define JDI_IOCTL_FREE_INSTANCE_ID                  \
    _IO(JDI_IOCTL_MAGIC, 15)
#define JDI_IOCTL_POLL_WAIT_INSTANCE                \
    _IO(JDI_IOCTL_MAGIC, 16)
#define JDI_IOCTL_SET_CTX_INFO				\
    _IO(JDI_IOCTL_MAGIC, 17)
#define JDI_IOCTL_SET_STATUS_INFO				\
    _IO(JDI_IOCTL_MAGIC, 18)
#define JDI_IOCTL_MAP_ION_FD                        \
	_IO(JDI_IOCTL_MAGIC, 19)
#define JDI_IOCTL_UNMAP_ION_FD                      \
	_IO(JDI_IOCTL_MAGIC, 20)
#define JDI_IOCTL_MAP_ION_PHYS                      \
	_IO(JDI_IOCTL_MAGIC, 21)
#define JDI_IOCTL_UNMAP_ION_PHYS                    \
	_IO(JDI_IOCTL_MAGIC, 22)

#endif /* __HOBOT_JPU_USER_H__ */
