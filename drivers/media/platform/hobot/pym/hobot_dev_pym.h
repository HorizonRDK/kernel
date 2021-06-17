/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_PYM_H__
#define __HOBOT_PYM_H__

#include <uapi/linux/types.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/ion.h>
#include "pym_config.h"
#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_group_api.h"

#define MAX_DEVICE  2
#define X3_PYM_NAME  "x3-pym"

#define PYM_IOC_MAGIC 'p'

#define PYM_IOC_INIT             _IOW(PYM_IOC_MAGIC, 0, pym_cfg_t)
#define PYM_IOC_STREAM           _IOW(PYM_IOC_MAGIC, 1, int)
#define PYM_IOC_QBUF        	 _IOW(PYM_IOC_MAGIC, 2, int)
#define PYM_IOC_DQBUF       	 _IOR(PYM_IOC_MAGIC, 3, int)
#define PYM_IOC_REQBUFS       	 _IOW(PYM_IOC_MAGIC, 4, int)
#define PYM_IOC_END_OF_STREAM    _IOW(PYM_IOC_MAGIC, 5, int)
#define PYM_IOC_BIND_GROUP       _IOW(PYM_IOC_MAGIC, 6, int)
#define PYM_IOC_GET_INDEX	 _IOR(PYM_IOC_MAGIC, 7, int)
#define PYM_IOC_USER_STATS       _IOR(PYM_IOC_MAGIC, 8, struct user_statistic)
#define PYM_IOC_SCALE_INFO	 _IOR(PYM_IOC_MAGIC, 9, pym_cfg_t)
#define PYM_IOC_SCALE_INFO_CH	 _IOR(PYM_IOC_MAGIC, 10, pym_scale_ch_t)
#define PYM_IOC_KERNEL_ION       _IOWR(PYM_IOC_MAGIC, 11, kernel_ion_t)


struct pym_status_statistic {
	u32 enable[VIO_MAX_STREAM];

	/* driver statistic*/
	u32 fe_normal[VIO_MAX_STREAM];
	u32 fe_lack_buf[VIO_MAX_STREAM];

	u32 hard_frame_drop_us[VIO_MAX_STREAM];
	u32 hard_frame_drop_ds[VIO_MAX_STREAM];
	u32 soft_frame_drop_us[VIO_MAX_STREAM];
	u32 soft_frame_drop_ds[VIO_MAX_STREAM];

	u32 fs_lack_task[VIO_MAX_STREAM];

	u32 dq_normal[VIO_MAX_STREAM];
	u32 dq_err[VIO_MAX_STREAM];

	u32 pollin_fe[VIO_MAX_STREAM];
	u32 pollin_comp[VIO_MAX_STREAM];
	u32 pollerr[VIO_MAX_STREAM];

	u32 q_normal[VIO_MAX_STREAM];

	u32 fs[VIO_MAX_STREAM];
	u32 grp_tsk_left[VIO_MAX_STREAM];

	u32 tal_fs;
	u32 tal_frm_work;

	/* user statistic*/
	struct user_statistic user_stats[VIO_MAX_STREAM];
	u32 err_stat_enable[VIO_MAX_STREAM];
};

struct pym_video_ctx{
	wait_queue_head_t		done_wq;
	struct vio_framemgr 	*framemgr;
	u32			frm_fst_ind;
	u32			frm_num;
	int                     frm_num_usr;
	struct vio_group		*group;
	unsigned long			state;
	u32 event;
	u32 id;
	u32 ctx_index;

	struct x3_pym_dev 	*pym_dev;
	struct pym_subdev	*subdev;

	int belong_pipe;
};

enum subdev_id {
	SUBDEV_ID_OUT,
	SUBDEV_ID_SRC,
	SUBDEV_ID_MAX,
};
enum process_role {
	PYM_MASTER = 0,
	PYM_SLAVER = 1
};

enum pym_interrupt_map {
	INTR_PYM_DS_FRAME_DROP,
	INTR_PYM_US_FRAME_DROP,
	INTR_PYM_FRAME_DONE,
	INTR_PYM_FRAME_START,
};

enum pym_status {
	PYM_OTF_INPUT,
	PYM_DMA_INPUT,
	PYM_REUSE_SHADOW0,
	PYM_HW_CONFIG,
	PYM_HW_RUN,
	PYM_HW_FORCE_STOP,
};

enum pym_subdev_state {
	PYM_SUBDEV_INIT,
	PYM_SUBDEV_REQBUF,
	PYM_SUBDEV_STREAM_ON,
	PYM_SUBDEV_STREAM_OFF,
};

struct pym_subdev {
	spinlock_t 		slock;
	unsigned long 	val_ctx_mask;
	struct pym_video_ctx	*ctx[VIO_MAX_SUB_PROCESS];
	atomic_t		refcount;
	struct vio_framemgr	framemgr;
	struct frame_info 	frameinfo;
	unsigned long 		state;
	struct vio_group 	*group;
	struct x3_pym_dev 	*pym_dev;
	u32 id;

	pym_cfg_t pym_cfg;
	u32 update_all;
	bool disable_flag;
	pym_scale_ch_t pym_cfg_ch;
	bool update_ch;

	u8 poll_mask;
};

/**
 * struct x3_pym_dev is used to describe pyramid module
 * @instance: used as pipeline id.
 * @rsccount: rsccount>0 pym is already enable
 * @open_cnt: open_cnt>0 pym pm_qos/clk is already enable
 * @shared_slock: lock access to this struct
 * @sensor_fcount: frame start interrupt count
 * @backup_fcount: count the number of prepared frames in frame_work func
 * @enable_cnt: used for serial enable and disable operations
 * @subdev: describe pym that support time-sharing for VIO_MAX_STREAM pipeline
 * @group: pointer to iscore.chain[VIO_MAX_STREAM].group[GROUP_ID_IPU]
 * @gtask: describe group's task, hold kthread worker
 * @vwork: describe frame's work, hold kthread work entry for each frames(VIO_MP_MAX_FRAMES)
 */
struct x3_pym_dev {
	/* channel information */
	u32 __iomem *base_reg;
	resource_size_t regs_start;
	resource_size_t regs_end;
	int irq;
	dev_t devno;
	unsigned long state;

	struct class *class;
	struct cdev cdev;

	atomic_t instance;
	atomic_t rsccount;
	atomic_t open_cnt;
	spinlock_t shared_slock;
	atomic_t sensor_fcount;
	atomic_t backup_fcount;
	atomic_t enable_cnt;

	struct pym_status_statistic statistic;
	struct pym_subdev subdev[VIO_MAX_STREAM][MAX_DEVICE];
	struct vio_group *group[VIO_MAX_STREAM];
	struct vio_group_task gtask;	
	struct vio_work vwork[VIO_MAX_STREAM][VIO_MP_MAX_FRAMES];

	struct ion_client *ion_client;
};

#endif
