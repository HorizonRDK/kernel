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

#include "ipu_config.h"

#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_group_api.h"

#define X2A_IPU_NAME  "x2a-ipu"

#define MAX_DEVICE  8
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

struct ipu_video_ctx {
	wait_queue_head_t done_wq;
	struct vio_framemgr framemgr;
	struct vio_group *group;
	u32 event;
	bool leader;
	u32 id;
	unsigned long state;

	struct x2a_ipu_dev *ipu_dev;
	struct ipu_osd_cfg osd_cfg;
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
};

struct x2a_ipu_dev {
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

	struct vio_group *group[VIO_MAX_STREAM];
	struct vio_group_task gtask;
};

#endif
