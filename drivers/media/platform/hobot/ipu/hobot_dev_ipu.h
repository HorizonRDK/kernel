/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_IPU_DEV_H__
#define __HOBOT_IPU_DEV_H__

#include <uapi/linux/types.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/ion.h>

#include "ipu_config.h"

#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_group_api.h"

#define X3_IPU_NAME  "x3-ipu"

#define MAX_DEVICE  8
#define DONE_NODE_ID (MAX_DEVICE - 1)
#define IPU_IOC_MAGIC 'v'

#define IPU_IOC_INIT             _IOW(IPU_IOC_MAGIC, 0, int)
#define IPU_IOC_STREAM           _IOW(IPU_IOC_MAGIC, 1, int)
#define IPU_IOC_QBUF        	 _IOW(IPU_IOC_MAGIC, 2, int)
#define IPU_IOC_DQBUF       	 _IOR(IPU_IOC_MAGIC, 3, int)
#define IPU_IOC_REQBUFS       	 _IOW(IPU_IOC_MAGIC, 4, int)
#define IPU_IOC_EOS       	 	 _IOW(IPU_IOC_MAGIC, 5, int)
#define IPU_IOC_OSD_ROI        	 _IOW(IPU_IOC_MAGIC, 6, int)
#define IPU_IOC_OSD_STA       	 _IOW(IPU_IOC_MAGIC, 7, int)
#define IPU_IOC_OSD_STA_LEVEL    _IOW(IPU_IOC_MAGIC, 8, int)
#define IPU_IOC_OSD_STA_BIN      _IOR(IPU_IOC_MAGIC, 9, int)
#define IPU_IOC_OSD_ADDR         _IOW(IPU_IOC_MAGIC, 10, int)
#define IPU_IOC_BIND_GROUP       _IOW(IPU_IOC_MAGIC, 11, int)
#define IPU_IOC_GET_INDEX      	 _IOR(IPU_IOC_MAGIC, 12, int)
#define IPU_IOC_OSD_COLOR_MAP    _IOW(IPU_IOC_MAGIC, 13, int)
#define IPU_IOC_SCALE_INFO    	 _IOW(IPU_IOC_MAGIC, 14, int)
#define IPU_IOC_SRC_INFO    	 _IOW(IPU_IOC_MAGIC, 15, int)
#define IPU_IOC_USER_STATS       _IOR(IPU_IOC_MAGIC, 16, struct user_statistic)
#define IPU_IOC_KERNEL_ION       _IOWR(IPU_IOC_MAGIC, 17, kernel_ion_t)
#define IPU_IOC_SET_FRAME_SKIP_PARAM    _IOWR(IPU_IOC_MAGIC, 18, int)
#define IPU_IOC_SET_FRAME_RATE_CTRL    _IOWR(IPU_IOC_MAGIC, 19, int)
#define IPU_IOC_KERNEL_ION_CONTINOUS    _IOWR(IPU_IOC_MAGIC, 20, kernel_ion_t)
#define IPU_IOC_WAIT_INIT    	 _IOW(IPU_IOC_MAGIC, 21, int)
#define IPU_IOC_SPLICE_INFO      _IOW(IPU_IOC_MAGIC, 22, int)



#define	IOCTL_RET_VAL_BASE	1
#define	IOCTL_FLAG_MULTI_PROCESS_SHARED	(IOCTL_RET_VAL_BASE + 1)

struct ipu_osd_cfg{
	bool osd_box_update;
	osd_box_t osd_box[MAX_OSD_NUM];
	bool osd_buf_update;
	u32 osd_buf[MAX_OSD_NUM];
	bool osd_sta_update;
	osd_sta_box_t osd_sta[MAX_STA_NUM];
	bool osd_sta_level_update;
	u8 osd_sta_level[MAX_OSD_STA_LEVEL_NUM];
	osd_color_map_t color_map;
};

struct ipu_info_cfg {
	ipu_ds_info_t sc_info;
	u32 info_update;
};

struct ipu_src_cfg {
	ipu_src_ctrl_t src_info;
	u32 info_update;
};

struct ipu_status_statistic {
	u32 enable[VIO_MAX_STREAM];

	/* driver statistic*/
	u32 fe_normal[VIO_MAX_STREAM][MAX_DEVICE];
	u32 fe_lack_buf[VIO_MAX_STREAM][MAX_DEVICE];

	u32 hard_frame_drop[VIO_MAX_STREAM][MAX_DEVICE];
	u32 fs_lack_task[VIO_MAX_STREAM];

	u32 dq_normal[VIO_MAX_STREAM][MAX_DEVICE];
	u32 dq_err[VIO_MAX_STREAM][MAX_DEVICE];

	u32 pollin_fe[VIO_MAX_STREAM][MAX_DEVICE];
	u32 pollin_comp[VIO_MAX_STREAM][MAX_DEVICE];
	u32 pollerr[VIO_MAX_STREAM][MAX_DEVICE];

	u32 q_normal[VIO_MAX_STREAM][MAX_DEVICE];

	u32 fs[VIO_MAX_STREAM];
	u32 grp_tsk_left[VIO_MAX_STREAM];

	u32 tal_fs;
	u32 tal_frm_work;

	/* user statistic*/
	struct user_statistic user_stats[VIO_MAX_STREAM][MAX_DEVICE];

	u32 enable_subdev[VIO_MAX_STREAM];
	u32 err_stat_enable[VIO_MAX_STREAM];
};

struct ipu_video_ctx {
	wait_queue_head_t	done_wq;
	struct vio_framemgr 	*framemgr;
	u32			frm_fst_ind;
	u32			frm_num;
	int			frm_num_usr;
	struct vio_group 	*group;
	u32 			event;
	u32	 		id;
	unsigned long 		state;
	u32 			ctx_index;

	struct x3_ipu_dev 	*ipu_dev;
	struct ipu_subdev	*subdev;

	int belong_pipe;

	int wait_init_index;
};

enum ipu_group_state {
	IPU_GROUP_OPEN,
	IPU_GROUP_INIT,
	IPU_GROUP_START,
	IPU_GROUP_SHOT,
	IPU_GROUP_REQUEST_FSTOP,
	IPU_GROUP_FORCE_STOP,
	IPU_GROUP_OTF_INPUT,
	IPU_GROUP_OTF_OUTPUT,
	IPU_GROUP_LEADER,
};

enum group_id {
	GROUP_ID_SRC,
	GROUP_ID_US,
	GROUP_ID_DS0,
	GROUP_ID_DS1,
	GROUP_ID_DS2,
	GROUP_ID_DS3,
	GROUP_ID_DS4,
	GROUP_ID_MAX,
};

enum ipu_interrupt_map {
	INTR_IPU_FRAME_START,
	INTR_IPU_FRAME_DONE,
	INTR_IPU_US_FRAME_DONE,
	INTR_IPU_DS0_FRAME_DONE,
	INTR_IPU_DS1_FRAME_DONE,
	INTR_IPU_DS2_FRAME_DONE,
	INTR_IPU_DS3_FRAME_DONE,
	INTR_IPU_DS4_FRAME_DONE,
	INTR_IPU_US_FRAME_DROP,
	INTR_IPU_DS0_FRAME_DROP,
	INTR_IPU_DS1_FRAME_DROP,
	INTR_IPU_DS2_FRAME_DROP,
	INTR_IPU_DS3_FRAME_DROP,
	INTR_IPU_DS4_FRAME_DROP,
	INTR_IPU_OSD0_FRAME_DROP,
	INTR_IPU_OSD1_FRAME_DROP,
	INTR_IPU_OSD2_FRAME_DROP,
	INTR_IPU_PRE_0_FRAME_DONE,
	INTR_IPU_PRE_1_FRAME_DONE,
};

enum ipu_status {
	IPU_OTF_INPUT,
	IPU_DMA_INPUT,
	IPU_OTF_OUTPUT,
	IPU_DS2_DMA_OUTPUT,
	IPU_HW_CONFIG,
	IPU_HW_RUN,
	IPU_HW_FORCE_STOP,
	IPU_REUSE_SHADOW0,
};

enum ipu_subdev_state {
	IPU_SUBDEV_INIT,
	IPU_SUBDEV_REQBUF
};

struct ipu_frame_skip_info {
	unsigned int frame_skip_step;
	unsigned int frame_skip_num;
};

struct ipu_frame_rate_ctrl {
	unsigned int src_frame_rate;
	unsigned int dst_frame_rate;
};

enum frame_rate_strategy {
	NULL_SKIP_MODE,
	FIX_INTERVAL_SKIP_MODE,
	BALANCE_SKIP_MODE
};

struct ipu_wait_init_info {
	u32 instance;
	int pid;
};

struct ipu_subdev {
	spinlock_t 		slock;
	struct ipu_video_ctx	*ctx[VIO_MAX_SUB_PROCESS];
	unsigned long		val_ctx_mask;
	atomic_t		refcount;
	struct vio_framemgr	framemgr;
	struct frame_info 	frameinfo;
	unsigned long 		state;
	struct vio_group 	*group;
	struct x3_ipu_dev 	*ipu_dev;
	u32 			id;

	struct ipu_osd_cfg osd_cfg;
	struct ipu_info_cfg info_cfg;
	struct ipu_src_cfg src_cfg;
	ipu_cfg_t ipu_cfg;
	ipu_ds_info_t scale_cfg;
	atomic_t pre_enable_flag;
	u32 cur_enable_flag;

	u8 poll_mask;

	unsigned int enable_frame_cnt;
	unsigned int curr_frame_cnt;
	atomic_t lost_next_frame;
	atomic_t lost_this_frame;

	unsigned int frame_rate_change_strategy;
	unsigned int frame_skip_step;
	unsigned int frame_skip_num;

	bool frame_is_skipped;

	int	wait_init_pid[VIO_MAX_SUB_PROCESS];
};

struct x3_ipu_dev {
	/* channel information */
	u32 __iomem *base_reg;
	resource_size_t regs_start;
	resource_size_t regs_end;
	int irq;
	dev_t devno;
	unsigned long state;

	struct class *class;
	struct cdev cdev;
	u32    wr_fifo_thred0;
	u32    wr_fifo_thred1;
	u32	   line_delay;
	atomic_t instance;
	atomic_t rsccount;
	atomic_t open_cnt;
	atomic_t sensor_fcount;
	atomic_t backup_fcount;
	atomic_t enable_cnt;
	u32 reuse_shadow0_count;
	u32 frame_drop_count;
	struct ipu_status_statistic statistic;

	struct ipu_subdev subdev[VIO_MAX_STREAM][MAX_DEVICE];
	struct vio_group *group[VIO_MAX_STREAM];
	struct vio_group_task gtask;
	struct vio_work vwork[VIO_MAX_STREAM][VIO_MP_MAX_FRAMES];

	struct ion_client *ion_client;

	u64		pipe_done_count[VIO_MAX_STREAM];
};

#endif
