/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_VPU_USER_H_
#define __HOBOT_VPU_USER_H_

#include <linux/types.h>
#include "inc/hb_media_codec.h"

#define USE_VMALLOC_FOR_INSTANCE_POOL_MEMORY

#define VPU_DEV_NAME "vpu"

#ifdef USE_MUTEX_IN_KERNEL_SPACE
typedef enum hb_vpu_mutex {
	VPUDRV_MUTEX_VPU,
	VPUDRV_MUTEX_DISP_FALG,
	VPUDRV_MUTEX_RESET,
	VPUDRV_MUTEX_VMEM,
	VPUDRV_MUTEX_MAX
} hb_vpu_mutex_t;
#else
typedef enum hb_vpu_mutex {
	VPUDRV_MUTEX_VPU,
	VPUDRV_MUTEX_DISP_FALG,
	VPUDRV_MUTEX_RESET,
	VPUDRV_MUTEX_VMEM,
	VPUDRV_MUTEX_REV1,
	VPUDRV_MUTEX_MAX
} hb_vpu_mutex_t;

#define VDI_NUM_LOCK_HANDLES                5
#endif

typedef struct _hb_vpu_drv_firmware {
	/* size of this structure */
	unsigned int size;
	unsigned int core_idx;
	unsigned long reg_base_offset;
	unsigned short bit_code[512];
} hb_vpu_drv_firmware_t;

typedef struct _hb_vpu_drv_buffer {
	unsigned int size;
	unsigned long phys_addr;

	/* kernel logical address in use kernel */
	unsigned long base;

	/* virtual user space address */
	unsigned long virt_addr;
} hb_vpu_drv_buffer_t;

typedef struct _hb_vpu_drv_inst {
	unsigned int core_idx;
	unsigned int inst_idx;

	/* for output only */
	int inst_open_count;
} hb_vpu_drv_inst_t;

typedef struct _hb_vpu_ctx_info {
	int valid;
	media_codec_context_t context;
	// encoder
	mc_video_longterm_ref_mode_t ref_mode;
	mc_video_intra_refresh_params_t intra_refr;
	mc_rate_control_params_t rc_params;
	mc_video_deblk_filter_params_t deblk_filter;
	mc_h265_sao_params_t sao_params;
	mc_h264_entropy_params_t entropy_params;
	mc_video_vui_params_t vui_params;
	mc_video_vui_timing_params_t vui_timing;
	mc_video_slice_params_t slice_params;
	hb_u32 force_idr_header;
	hb_s32 enable_idr;
	mc_video_3dnr_enc_params_t noise_rd;
	mc_video_smart_bg_enc_params_t smart_bg;
	mc_video_pred_unit_params_t pred_unit;
	mc_video_transform_params_t transform_params;
	mc_video_roi_params_t roi_params;
	mc_video_mode_decision_params_t mode_decision;
	hb_s32 cam_pipline;
	hb_s32 cam_channel;
} hb_vpu_ctx_info_t;

typedef struct _hb_vpu_status_info {
	unsigned int inst_idx;
	mc_inter_status_t status;
	// encoder
	mc_h264_h265_output_stream_info_t stream_info;
	// decoder
	mc_h264_h265_output_frame_info_t frame_info;
} hb_vpu_status_info_t;

typedef struct _hb_vpu_drv_intr {
	unsigned int timeout;
	int intr_reason;

//#ifdef SUPPORT_MULTI_INST_INTR
	int intr_inst_index;
//#endif
} hb_vpu_drv_intr_t;

typedef enum _hb_vpu_event_e {
	VPU_EVENT_NONE = 0,
	VPU_ENC_PIC_DONE = 1,
	VPU_DEC_PIC_DONE = 2,
	VPU_INST_CLOSED = 3,
	VPU_INST_INTERRUPT = 4,
} hb_vpu_event_t;

#define VDI_IOCTL_MAGIC						'V'
#define VDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY			\
	_IO(VDI_IOCTL_MAGIC, 0)
#define VDI_IOCTL_FREE_PHYSICALMEMORY				\
	_IO(VDI_IOCTL_MAGIC, 1)
#define VDI_IOCTL_WAIT_INTERRUPT				\
	_IO(VDI_IOCTL_MAGIC, 2)
#define VDI_IOCTL_SET_CLOCK_GATE				\
	_IO(VDI_IOCTL_MAGIC, 3)
#define VDI_IOCTL_RESET						\
	_IO(VDI_IOCTL_MAGIC, 4)
#define VDI_IOCTL_GET_INSTANCE_POOL				\
	_IO(VDI_IOCTL_MAGIC, 5)
#define VDI_IOCTL_GET_COMMON_MEMORY				\
	_IO(VDI_IOCTL_MAGIC, 6)
#define VDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO		\
	_IO(VDI_IOCTL_MAGIC, 8)
#define VDI_IOCTL_OPEN_INSTANCE					\
	_IO(VDI_IOCTL_MAGIC, 9)
#define VDI_IOCTL_CLOSE_INSTANCE				\
	_IO(VDI_IOCTL_MAGIC, 10)
#define VDI_IOCTL_GET_INSTANCE_NUM				\
	_IO(VDI_IOCTL_MAGIC, 11)
#define VDI_IOCTL_GET_REGISTER_INFO				\
	_IO(VDI_IOCTL_MAGIC, 12)
#define VDI_IOCTL_GET_FREE_MEM_SIZE				\
	_IO(VDI_IOCTL_MAGIC, 13)
#define VDI_IOCTL_ALLOCATE_INSTANCE_ID				\
	_IO(VDI_IOCTL_MAGIC, 14)
#define VDI_IOCTL_FREE_INSTANCE_ID				\
	_IO(VDI_IOCTL_MAGIC, 15)
#define VDI_IOCTL_POLL_WAIT_INSTANCE				\
	_IO(VDI_IOCTL_MAGIC, 16)
#define VDI_IOCTL_SET_CTX_INFO				\
	_IO(VDI_IOCTL_MAGIC, 17)
#define VDI_IOCTL_SET_STATUS_INFO				\
	_IO(VDI_IOCTL_MAGIC, 18)
#define VDI_IOCTL_VDI_LOCK	\
	_IO(VDI_IOCTL_MAGIC, 23)
#define VDI_IOCTL_VDI_UNLOCK	\
	_IO(VDI_IOCTL_MAGIC, 24)

#endif /* HOBOT_VPU_USER_H_ */
