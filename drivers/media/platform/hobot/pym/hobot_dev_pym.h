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
#include "pym_config.h"
#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_group_api.h"

#define MAX_DEVICE  1
#define X3_PYM_NAME  "x3-pym"

#define PYM_IOC_MAGIC 'p'

#define PYM_IOC_INIT             _IOW(PYM_IOC_MAGIC, 0, pym_cfg_t)
#define PYM_IOC_STREAM           _IOW(PYM_IOC_MAGIC, 1, int)
#define PYM_IOC_QBUF        	 _IOW(PYM_IOC_MAGIC, 2, int)
#define PYM_IOC_DQBUF       	 _IOR(PYM_IOC_MAGIC, 3, int)
#define PYM_IOC_REQBUFS       	 _IOW(PYM_IOC_MAGIC, 4, int)
#define PYM_IOC_END_OF_STREAM    _IOW(PYM_IOC_MAGIC, 5, int)
#define PYM_IOC_BIND_GROUP       _IOW(PYM_IOC_MAGIC, 6, int)

struct pym_video_ctx {
	wait_queue_head_t done_wq;
	struct vio_framemgr framemgr;
	struct vio_group *group;
	unsigned long state;

	struct x3_pym_dev *pym_dev;
	pym_cfg_t pym_cfg;
	u32 event;
	bool leader;
};

enum group_id {
	GROUP_ID_SRC,
	GROUP_ID_MAX,
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
};


struct x3_pym_dev {
	/* channel information */
	u32 __iomem *base_reg;
	resource_size_t regs_start;
	resource_size_t regs_end;
	int irq;
	unsigned long state;

	struct class *class;
	struct cdev cdev;

	atomic_t instance;
	atomic_t rsccount;
	atomic_t open_cnt;
	spinlock_t shared_slock;

	struct vio_group *group[VIO_MAX_STREAM];
	struct vio_group_task gtask;
};

#endif
