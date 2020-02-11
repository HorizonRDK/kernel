/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_GDC_DEV_H__
#define __HOBOT_GDC_DEV_H__

#include <uapi/linux/types.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/wait.h>

#include "gdc_config.h"
#include "vio_config.h"
#include "vio_framemgr.h"

#define X2A_GDC_NAME  "x2a-gdc"

#define GDC_MAX_DEVICE  1
#define GDC_IOC_MAGIC 'g'

#define GDC_IOC_PROCESS          _IOW(GDC_IOC_MAGIC, 0, int)

#define ACAMERA_GDC_MAX_INPUT 3

// each configuration addresses and size
typedef struct gdc_config {
	u32 config_addr;   //gdc config address
	u32 config_size;   //gdc config size in 32bit
	u32 input_width;  //gdc input width resolution
	u32 input_height; //gdc input height resolution
	u32 input_stride;  //gdc input stride (pixel)
	u32 output_width;  //gdc output width resolution
	u32 output_height; //gdc output height resolution
	u32 output_stride;  //gdc output stride (pixel)
	u8  div_width;     //use in dividing UV dimensions; actually a shift right
	u8  div_height;    //use in dividing UV dimensions; actually a shift right
	u32 total_planes;
	u8 sequential_mode; //sequential processing
} gdc_config_t;

// overall gdc settings and state
typedef struct gdc_settings {
	gdc_config_t gdc_config; //array of gdc configuration and sizes

	u32 Out_buffer_addr[ACAMERA_GDC_MAX_INPUT];     //start memory to write gdc output framse
	u32 In_buffer_addr[ACAMERA_GDC_MAX_INPUT]; 	//start memory to read gdc input framse

	u32 total_planes;
	u8 seq_planes_pos; //sequential plance current index
} gdc_settings_t;

struct gdc_video_ctx{
	wait_queue_head_t		done_wq;
	struct x2a_gdc_dev 		*gdc_dev;
	struct gdc_group		*group;
	bool is_waiting_gdc;
};

enum gdc_group_state {
	GDC_GROUP_OPEN,
	GDC_GROUP_INIT,
	GDC_GROUP_START,
	GDC_GROUP_SHOT,
	GDC_GROUP_REQUEST_FSTOP,
	GDC_GROUP_FORCE_STOP,
	GDC_GROUP_OTF_INPUT,
	GDC_GROUP_OTF_OUTPUT,
	GDC_GROUP_LEADER,
};

enum gdc_interrupt_map {
	INTR_GDC_BUSY,
	INTR_GDC_ERROR,
	INTR_GDC_CONF_ERROR = 8,
	INTR_GDC_USER_ABORT,
	INTR_GDC_AXI_READER_ERROR,
	INTR_GDC_AXI_WRITER_ERROR,
	INTR_GDC_UNALIGNED_ACCESS,
	INTR_GDC_INCOMPATIBLE_CONF,
};

enum gdc_dev_state {
	GDC_DEV_NONE,
	GDC_DEV_PROCESS,
	GDC_DEV_FREE,
};

struct gdc_group{
	struct gdc_video_ctx *sub_ctx[GDC_MAX_DEVICE];
	unsigned long				state;
	u32 instance;
};

struct x2a_gdc_dev {
	/* channel information */
	u32 __iomem			*base_reg;
	resource_size_t			regs_start;
	resource_size_t			regs_end;
	int				irq;
	unsigned long			state;
	spinlock_t shared_slock;

	struct class *class;
	struct cdev cdev;
	dev_t devno;

	atomic_t			instance;
	u32 hw_id;

	struct gdc_group group[VIO_MAX_STREAM];
	struct semaphore smp_gdc_enable;
};

#endif
