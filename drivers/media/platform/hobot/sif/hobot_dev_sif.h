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

#define FRAME_ID_MAXIMUM 65535u
#define FRAME_ID_SHIFT 65536u

#define SIF_L1_CACHE_BYTES  64
#define MAX_DEVICE  2
#define SIF_ERR_COUNT  10
#define SIF_MUX_BUFF_CNT 4
#define SIF_FE_BOTH 0x3

#define SIF_SEQ_TASK_PRIORITY  39

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
#define SIF_IOC_PATTERN_CFG	 _IOW(SIF_IOC_MAGIC, 10, int)
#define SIF_IOC_USER_STATS       _IOR(SIF_IOC_MAGIC, 11, struct user_statistic)
#define SIF_IOC_MD_ENABLE	 _IO(SIF_IOC_MAGIC, 12)
#define SIF_IOC_MD_DISENABLE	 _IO(SIF_IOC_MAGIC, 13)
#define SIF_IOC_ORDER            _IOWR(SIF_IOC_MAGIC, 14, struct user_seq_info)
#define SIF_STOP_WAKE_UP	 _IO(SIF_IOC_MAGIC, 15)
#define SIF_IOC_MCLK_SET	 _IOW(SIF_IOC_MAGIC, 16, u32)
#define SIF_IOC_IPI_RESET	 _IO(SIF_IOC_MAGIC, 17)
#define SIF_IOC_MIPI_CFG_RESET	 _IO(SIF_IOC_MAGIC, 18)


#define VIO_MP_IOC_MAGIC 'm'
#define VIO_MP_IOC_BIND_GROUP	 _IOW(VIO_MP_IOC_MAGIC, 0, int)
#define VIO_MP_IOC_GET_REFCOUNT	 _IOR(VIO_MP_IOC_MAGIC, 1, int)

#define VIO_BIND_INFO_MAGIC 'b'
#define VIO_BIND_INFO_UPDATE	_IOW(VIO_BIND_INFO_MAGIC, 0, int)
#define VIO_GET_STAT_INFO		_IOW(VIO_BIND_INFO_MAGIC, 1, int)

#define BIT2CHN(chns, chn) (chns & (1 << chn))

struct sif_irq_src {
	u32 sif_frm_int; 	 // out FS status
	u32 sif_out_int; 	 // sif to isp irq status
	u32 sif_err_status;
	u32 sif_in_buf_overflow; 	 // overflow irq status
};

enum sif_frame_state {
	SIF_YUV_MODE,  /*for yuv*/
};

/*recover buff state*/
enum sif_hwidx_process_state {
	SIF_OWNERBIT_RELEASE = 1,
	SIF_RECOVER_BUFF,
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
	SIF_FORMAT_YUV_RAW8 = 1,  // Yuv422-8 received by raw8
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
	INTR_SIF_IN_SIZE_MISMATCH = 28,  // The width and height received by SIF are inconsistent with the actual configuration
	INTR_SIF_IN_OVERFLOW = 29,  // An interrupt error occurred for overflow
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
	struct vio_framemgr 	*framemgr;  // Responsible for buff rotation
	struct vio_group		*group;
	unsigned long			state;
	u32 event;
	u32 id;    	// 0:out node 1:ddrin node
	u32 ctx_index;

	struct x3_sif_dev 	*sif_dev;   // SIF hardware architecture
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
	SIF_SPLICE_OP,  // For splicing
};

enum sif_subdev_state {
	SIF_SUBDEV_INIT,
	SIF_SUBDEV_REQBUF,
	SIF_SUBDEV_STREAM_ON,
	SIF_SUBDEV_STREAM_OFF,
};

typedef struct fps_ctrl_s {
	unsigned int skip_frame;
	unsigned int curr_cnt;
	unsigned int in_fps;
	unsigned int out_fps;
	atomic_t lost_next_frame;
	atomic_t lost_this_frame;
}fps_ctrl_t;

struct splice_info {
	u32 splice_enable;		// Is splicing enabled
	u32 splice_mode;		// 0:Vertical spelling 1:Horizontal spelling
	u32 pipe_num;			// How many pipes spliced
	u32 splice_done;
	u32 frame_done;
	u32 mux_index[SPLICE_PIPE_NUM]; 	// mux occupied by splicing
	u32 splice_rx_index[SPLICE_PIPE_NUM];	// mipi_rx occupied by splicing
	u8	splice_vc_index[SPLICE_PIPE_NUM];	// vc_index occupied by splicing
	u32 splice_ipi_index[SPLICE_PIPE_NUM];	// ipi_index occupied by splicing
};

struct sif_subdev {
	/*protect variable val_ctx_mask for multi process sharing*/
	spinlock_t 		slock;
	unsigned long 	val_ctx_mask;
	struct sif_video_ctx	*ctx[VIO_MAX_SUB_PROCESS];
	atomic_t		refcount;
	struct vio_framemgr	framemgr;
	unsigned long 		state;
	struct vio_group 	*group;
	struct x3_sif_dev 	*sif_dev;

	sif_cfg_t sif_cfg;
	uint32_t format;
	u32 mux_index;
	u32 mux_index1;
	u32 mux_index2;
	u32 ddr_mux_index;
	u32 dol_num;
	u32 rx_index;
	u8  vc_index;
	u32 ipi_index;
	u32 ipi_channels;
	u32 mux_nums;
	u32 overflow;
	/*Save the hardware idx difference of two mux in yuv or splicing scene*/
	u32 hw_gap;
	/*Save last hardware idx*/
	u32 last_hwidx;
	/*recover buff in yuv or splicing scene*/
	u32 frame_drop;
	u32	fdone;  	/*y/uv done*/
	u32 arbit_dead;	 /*arbit deadlock happens*/
	u32 splice_flow_clr;   /*close wdma for splice*/
	sif_data_desc_t ddrin_fmt;
	sif_data_desc_t fmt;
	struct frame_id info;
	bool initial_frameid;
	volatile bool ipi_disable;
	u32 md_refresh_count;
	u32 id;
	struct splice_info  splice_info;
	fps_ctrl_t fps_ctrl;
#ifdef CONFIG_HOBOT_DIAG
	union {
		u32 diag_state;
		struct {
			u32 diag_state_mismatch: 1;
			u32 diag_state_overflow: 1;
			u32 diag_state_outbuf_error: 1;
		};
	};
#endif
	u32 cnt_shift;
};

#define SEQ_KTHREAD_STOP  (1 << VIO_MAX_STREAM)
#define SEQ_KTHREAD_FLUSH (SEQ_KTHREAD_STOP << 1)

struct frame_list {
	spinlock_t slock;
	u32 enable;               // pipe sequence enable
	u64 timeout;              // pipe sequence timeout
	u64 frameid;              // for debug
	u32 pipeline_id;          // pipe id
	u32 queue_count;          // node count
	struct timespec qbuf_ts;  // qbuf last time
	struct list_head queue_list;
};

struct frame_node {
	struct list_head list;
	u64 interval;            // time interval between two frame
	u64 frameid;             // for debug
	struct vio_group *group;
	struct vio_frame *frame;
};

struct sif2isp_seq {
	atomic_t            refcount;
	/* protect variable wait_mask */
	spinlock_t          slock;
	/*
	* wait_mask set to 1 when there is a buff in each way,
	* set to stop when the thread exits,
	* and set to flush when switching the sorting sequence
	*/
	uint32_t            wait_mask;
	uint8_t             seq_num[VIO_MAX_STREAM];
	wait_queue_head_t   wait_queue;
	struct task_struct *seq_kthread;
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
	unsigned long 		frame_state; /*used for yuv intr*/
	atomic_t			instance;
	atomic_t			rsccount;
	atomic_t			isp_init_cnt;
	atomic_t			open_cnt;
	/*
	* configuration of protection register
	* protect open and close mutex
	*/
	struct mutex			shared_mutex;
	u32 				error_count;
	u64 				buff_count[SIF_MUX_MAX];
	u64 				buff_count1[SIF_MUX_MAX];
	u32				hblank;
	u32				ovflow_cnt;	/*Count of overflow occurrences*/
	u32				owner_value; 	/*Ownerbit value in case of DDR deadlock*/
	atomic_t			wdma_used_cnt; 	/*Number of disabled wdma*/
	unsigned long			mux_mask;
	unsigned long			yuv422_mux_mask_a;
	unsigned long			yuv422_mux_mask_b;
	struct sif_status_statistic 	statistic;
	struct vio_bind_info_dev 	*vio_bind_info_dev;

	struct vio_group		*sif_input[VIO_MAX_STREAM];
	struct vio_group		*sif_mux[SIF_MUX_MAX];
	struct vio_group		*sif_mux_multiplexa[SIF_MUX_MAX];
	struct vio_group		*sif_mux_multiplexb[SIF_MUX_MAX];

	struct sif_subdev		sif_in_subdev[VIO_MAX_STREAM];
	struct sif_subdev		sif_mux_subdev[VIO_MAX_STREAM];

	struct vio_group_task	sifin_task;
	struct vio_group_task	sifout_task[SIF_MUX_MAX];
	struct vio_work sifout_work[VIO_MAX_STREAM][VIO_MP_MAX_FRAMES];
	struct vio_work sifin_work[VIO_MAX_STREAM][VIO_MP_MAX_FRAMES];

	struct sif2isp_seq      seq_task;	// ddrin nodes enter ISP processing in order
	struct frame_list frame_queue[VIO_MAX_STREAM];
	struct frame_node frame_nodes[VIO_MAX_STREAM][VIO_MP_MAX_FRAMES];
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
#define MAX_VIO_DEV        64  // max venc num is 64

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
	struct hb_bind_info_s bind_info[MAX_VIO_DEV][HB_ID_MAX];
};

int sif_get_stride(u32 pixel_length, u32 width);
void sif_get_mismatch_status(void);
int32_t mipi_host_reset_ipi(uint32_t port, int32_t ipi, int32_t enable);

#endif
