/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_SIF_H__
#define __HOBOT_SIF_H__

#include <uapi/linux/types.h>
#include <linux/cdev.h>
#include <linux/wait.h>

#include "sif_config.h"
#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_group_api.h"

#define SIF_MUX_MAX	8
#define X3_SIF_NAME  "x3-sif"

#define MAX_DEVICE  2
#define SIF_ERR_COUNT  10

#define X3_VIO_MP_NAME  "vio-mp"
#define X3_VIO_BIND_INFO_DEV_NAME  "vio-bind-info"
#define MAX_DEVICE_VIO_MP  1

#define SIF_IOC_MAGIC 'x'

#define SIF_IOC_INIT             _IOW(SIF_IOC_MAGIC, 0, sif_cfg_t)
#define SIF_IOC_STREAM           _IOW(SIF_IOC_MAGIC, 1, int)
#define SIF_IOC_QBUF        	 _IOW(SIF_IOC_MAGIC, 2, int)
#define SIF_IOC_DQBUF       	 _IOR(SIF_IOC_MAGIC, 3, int)
#define SIF_IOC_REQBUFS       	 _IOW(SIF_IOC_MAGIC, 4, int)
#define SIF_IOC_BIND_GROUP       _IOW(SIF_IOC_MAGIC, 5, int)
#define SIF_IOC_END_OF_STREAM    _IOW(SIF_IOC_MAGIC, 6, int)
#define SIF_IOC_BYPASS    		 _IOW(SIF_IOC_MAGIC, 7, int)
#define SIF_IOC_MD_EVENT    	 _IOR(SIF_IOC_MAGIC, 8, int)
#define SIF_IOC_MD_CFG	    	 _IOW(SIF_IOC_MAGIC, 9, int)
#define SIF_IOC_PATTERN_CFG	     _IOW(SIF_IOC_MAGIC, 10, int)
#define SIF_IOC_USER_STATS       _IOR(SIF_IOC_MAGIC, 11, struct user_statistic)
#define SIF_IOC_MD_ENABLE	     _IO(SIF_IOC_MAGIC, 12)
#define SIF_IOC_MD_DISENABLE	 _IO(SIF_IOC_MAGIC, 13)


#define VIO_MP_IOC_MAGIC 'm'
#define VIO_MP_IOC_BIND_GROUP	 _IOW(VIO_MP_IOC_MAGIC, 0, int)
#define VIO_MP_IOC_GET_REFCOUNT	 _IOR(VIO_MP_IOC_MAGIC, 1, int)

#define VIO_BIND_INFO_MAGIC 'b'
#define VIO_BIND_INFO_UPDATE	_IOW(VIO_BIND_INFO_MAGIC, 0, int)


struct sif_irq_src {
	u32 sif_frm_int;
	u32 sif_out_int;
	u32 sif_err_status;
	u32 sif_in_buf_overflow;
};

struct sif_multi_frame {
	u8 period;
	u8 mux_index;
	u8 trigger_mode;
	u8 enable;
};

struct sif_status_statistic {
	u32 enable[VIO_MAX_STREAM];

	/* driver statistic*/
	u32 fe_normal[VIO_MAX_STREAM][MAX_DEVICE];
	u32 fe_lack_buf[VIO_MAX_STREAM][MAX_DEVICE];

	u32 dq_normal[VIO_MAX_STREAM][MAX_DEVICE];
	u32 dq_err[VIO_MAX_STREAM][MAX_DEVICE];

	u32 pollin_fe[VIO_MAX_STREAM][MAX_DEVICE];
	u32 pollin_comp[VIO_MAX_STREAM][MAX_DEVICE];
	u32 pollerr[VIO_MAX_STREAM][MAX_DEVICE];

	u32 q_normal[VIO_MAX_STREAM][MAX_DEVICE];

	u32 fs_lack_task[VIO_MAX_STREAM];
	u32 hard_mismatch[VIO_MAX_STREAM];
	u32 hard_overflow[VIO_MAX_STREAM];
	u32 hard_buf_err[VIO_MAX_STREAM];

	u32 fs[VIO_MAX_STREAM];
	u32 grp_tsk_left[VIO_MAX_STREAM];

	/* user statistic*/
	struct user_statistic user_stats[VIO_MAX_STREAM][MAX_DEVICE];
};

enum sif_format{
	SIF_FORMAT_RAW,
	SIF_FORMAT_YUV_RAW8 = 1,
	HW_FORMAT_YUV422 = 8
};

enum pixel_length{
	PIXEL_LENGTH_8BIT = 0,
	PIXEL_LENGTH_10BIT = 1,
	PIXEL_LENGTH_12BIT = 2,
	PIXEL_LENGTH_16BIT = 4,
	PIXEL_LENGTH_20BIT = 5,
};

enum sif_out_interrupt_map{
	SIF_ISP_OUT_FS,
	SIF_ISP_OUT_FE,
	SIF_IPU_OUT_FS = 4,
	SIF_IPU_OUT_FE = 5,
};

enum sif_frame_interrupt_map{
	INTR_SIF_MUX0_OUT_FS,
	INTR_SIF_MUX1_OUT_FS,
	INTR_SIF_MUX2_OUT_FS,
	INTR_SIF_MUX3_OUT_FS,
	INTR_SIF_MUX4_OUT_FS,
	INTR_SIF_MUX5_OUT_FS,
	INTR_SIF_MUX6_OUT_FS,
	INTR_SIF_MUX7_OUT_FS,
	INTR_SIF_MUX0_FRAME_DONE,
	INTR_SIF_MUX1_FRAME_DONE,
	INTR_SIF_MUX2_FRAME_DONE,
	INTR_SIF_MUX3_FRAME_DONE,
	INTR_SIF_MUX4_FRAME_DONE,
	INTR_SIF_MUX5_FRAME_DONE,
	INTR_SIF_MUX6_FRAME_DONE,
	INTR_SIF_MUX7_FRAME_DONE,
	INTR_SIF_MIPI_TX_IPI0_FS,
	INTR_SIF_MIPI_TX_IPI1_FS,
	INTR_SIF_MIPI_TX_IPI2_FS,
	INTR_SIF_MIPI_TX_IPI3_FS,
	INTR_SIF_OUT_BUF_ERROR = 27,
	INTR_SIF_IN_SIZE_MISMATCH = 28,
	INTR_SIF_IN_OVERFLOW = 29,
	INTR_SIF_MULTI_FRAME_ID = 31,
};

enum sif_group_state {
	SIF_GROUP_OPEN,
	SIF_GROUP_INIT,
	SIF_GROUP_START,
	SIF_GROUP_SHOT,
	SIF_GROUP_REQUEST_FSTOP,
	SIF_GROUP_FORCE_STOP,
	SIF_GROUP_OTF_INPUT,
	SIF_GROUP_OTF_OUTPUT,
	SIF_GROUP_LEADER,
};

struct sif_pattern_cfg {
	u32 instance;
	u32 framerate;
};
/* device node for multi process */
struct mp_ctx {
	atomic_t		*refcount;
	int			instance;
	struct x3_vio_mp_dev 	*mp_dev;
};

struct x3_vio_mp_dev {
	struct cdev	cdev;
	struct class 	*class;
	dev_t 		devno;
	atomic_t	refcount[VIO_MAX_STREAM];
	spinlock_t	slock;
};

struct sif_video_ctx{
	wait_queue_head_t		done_wq;
	struct vio_framemgr 	*framemgr;
	struct vio_group		*group;
	unsigned long			state;
	u32 event;
	u32 id;
	u32 ctx_index;

	struct x3_sif_dev 	*sif_dev;
	struct sif_subdev	*subdev;
};

enum sif_state {
	/* one the fly output */
	SIF_OTF_OUTPUT = 10,
	/* WDMA flag */
	SIF_DMA_IN_ENABLE,
	SIF_DOL2_MODE,
	SIF_HW_RUN = 20,
	SIF_HW_FORCE_STOP,
};

enum sif_subdev_state {
	SIF_SUBDEV_INIT,
	SIF_SUBDEV_REQBUF,
	SIF_SUBDEV_STREAM_ON,
	SIF_SUBDEV_STREAM_OFF,
};

struct sif_subdev {
	spinlock_t 		slock;
	unsigned long 	val_ctx_mask;
	struct sif_video_ctx	*ctx[VIO_MAX_SUB_PROCESS];
	atomic_t		refcount;
	struct vio_framemgr	framemgr;
	unsigned long 		state;
	struct vio_group 	*group;
	struct x3_sif_dev 	*sif_dev;

	sif_cfg_t sif_cfg;
	u32 mux_index;
	u32 ddr_mux_index;
	u32 dol_num;
	u32 rx_index;
	u8	vc_index;
	u32 ipi_index;
	u32 ipi_channels;
	u32 mux_nums;
	sif_data_desc_t ddrin_fmt;
	sif_data_desc_t fmt;
	struct frame_id 		info;
	bool initial_frameid;
	u32 md_refresh_count;
	u64 bufcount;
	u32 id;
};


struct x3_sif_dev {
	u32 __iomem			*base_reg;
	resource_size_t			regs_start;
	resource_size_t			regs_end;
	int				irq;

	struct cdev cdev;
	struct class *class;
	dev_t devno;
	u32 mismatch_cnt;

	unsigned long		state;
	atomic_t			instance;
	atomic_t			rsccount;
	atomic_t			open_cnt;
	struct mutex			shared_mutex;
	u32 				error_count;
	u32					hblank;
	unsigned long	mux_mask;
	struct sif_status_statistic 	statistic;
	struct vio_bind_info_dev 	*vio_bind_info_dev;

	struct vio_group		*sif_input[VIO_MAX_STREAM];
	struct vio_group		*sif_mux[SIF_MUX_MAX];

	struct sif_subdev		sif_in_subdev[VIO_MAX_STREAM];
	struct sif_subdev		sif_mux_subdev[VIO_MAX_STREAM];

	struct vio_group_task	sifin_task;
	struct vio_group_task	sifout_task[SIF_MUX_MAX];
};

typedef enum HB_SYS_MOD_ID_E {
	HB_ID_SYS = 0,
	HB_ID_VIN,
	HB_ID_VOT,
	HB_ID_VPS,
	HB_ID_RGN,
	HB_ID_AIN,
	HB_ID_AOT,
	HB_ID_VENC,
	HB_ID_VDEC,
	HB_ID_AENC,
	HB_ID_ADEC,
	HB_ID_MAX
} SYS_MOD_ID_E;

typedef struct HB_SYS_MOD_S {
	SYS_MOD_ID_E enModId;
	uint8_t s32DevId;
	uint8_t s32ChnId;
} SYS_MOD_S;

struct hb_in_chn_bind_info_s {
	SYS_MOD_ID_E prev_mod;
	uint8_t prev_dev_id;
	uint8_t prev_chn_id;
};

struct hb_out_chn_bind_info_s {
	SYS_MOD_ID_E next_mod;
	uint8_t next_dev_id;
	uint8_t next_chn_id;
};

#define MAX_INPUT_CHANNEL 8
#define MAX_OUTPUT_CHANNEL 7

struct hb_bind_info_s {
	SYS_MOD_ID_E this_mod;
	uint8_t this_dev_id;
	uint8_t had_show;
	struct hb_in_chn_bind_info_s in[MAX_INPUT_CHANNEL];
	struct hb_out_chn_bind_info_s out[MAX_OUTPUT_CHANNEL];
};

struct hb_bind_info_update_s {
	SYS_MOD_S src_mod;
	SYS_MOD_S dst_mod;
};

/* device node for HAPI bind info */
struct vio_bind_info_dev {
	struct cdev	cdev;
	struct class 	*class;
	dev_t 		devno;
	spinlock_t	slock;
	struct hb_bind_info_s bind_info[VIO_MAX_STREAM][HB_ID_MAX];
};

int sif_get_stride(u32 pixel_length, u32 width);

#endif
