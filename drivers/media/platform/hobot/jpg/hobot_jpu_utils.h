/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_JPU_UTILS_H__
#define __HOBOT_JPU_UTILS_H__

#include <linux/wait.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/slab.h>

//#include "../../../../../test_code/jpg/jpuapi/jpuconfig.h"

#include "hobot_jpu_config.h"
#include "hobot_jpu_user.h"

/* if the platform driver knows the name of this driver */
/* JPU_PLATFORM_DEVICE_NAME */
/* #define JPU_SUPPORT_PLATFORM_DRIVER_REGISTER	*/
#define JPU_SUPPORT_PLATFORM_DRIVER_REGISTER

/* if this driver knows the dedicated video memory address */
//#define JPU_SUPPORT_RESERVED_VIDEO_MEMORY

#define JPU_PLATFORM_DEVICE_NAME    "hb_jpu"
#define JPU_JPEG_ACLK_NAME           "jpg_aclk"
#define JPU_JPEG_CCLK_NAME           "jpg_cclk"

typedef enum _hb_jpu_event_e {
	JPU_EVENT_NONE = 0,
	JPU_PIC_DONE = 1,
	JPU_INST_CLOSED = 2,
} hb_jpu_event_t;

typedef struct _hb_jpu_platform_data {
	int ip_ver;
	int clock_rate;
	int min_rate;
} hb_jpu_platform_data_t;

/* To track the allocated memory buffer */
typedef struct _hb_jpu_drv_buffer_pool {
	struct list_head list;
	hb_jpu_drv_buffer_t jb;
	struct file *filp;
} hb_jpu_drv_buffer_pool_t;

/* To track the instance index and buffer in instance pool */
typedef struct _hb_jpu_drv_instance_list {
	struct list_head list;
	unsigned long inst_idx;
	struct file *filp;
} hb_jpu_drv_instance_list_t;

typedef struct _hb_jpu_drv_instance_pool {
	unsigned char codecInstPool[MAX_NUM_JPU_INSTANCE][MAX_INST_HANDLE_SIZE];
} hb_jpu_drv_instance_pool_t;

typedef struct jpu_drv_context_t {
	struct fasync_struct *async_queue;
	u32 open_count;		/*!<< device reference count. Not instance count */
	u32 interrupt_reason[MAX_NUM_JPU_INSTANCE];
} jpu_drv_context_t;

typedef struct _hb_jpu_dev {
	struct device *device;
	hb_jpu_platform_data_t *plat_data;
	struct resource *jpu_mem;
	void __iomem *regs_base;
	int irq;
	int jpu_dev_num;
	int major;
	int minor;
	struct class *jpu_class;
	struct cdev cdev;
	struct device *jpu_dev;
	struct kobject *jpu_kobj;
	struct clk *jpu_aclk;
	struct clk *jpu_cclk;
#ifdef CONFIG_ION_HOBOT
	struct ion_client *jpu_ion_client;
#endif

	wait_queue_head_t interrupt_wait_q[MAX_NUM_JPU_INSTANCE];
	int interrupt_flag[MAX_NUM_JPU_INSTANCE];
	u32 interrupt_reason[MAX_NUM_JPU_INSTANCE];
	hb_jpu_event_t poll_event[MAX_NUM_JPU_INSTANCE];
	wait_queue_head_t poll_wait_q[MAX_NUM_JPU_INSTANCE];
	spinlock_t poll_spinlock;

	struct fasync_struct *async_queue;
	u32 open_count;		/*!<< device reference count. Not instance count */
	int jpu_open_ref_count;

	struct mutex jpu_mutex;
	struct semaphore jpu_sem;
	spinlock_t jpu_spinlock;
	struct list_head jbp_head;
	struct list_head inst_list_head;

	hb_jpu_drv_buffer_t instance_pool;
	hb_jpu_drv_buffer_t common_memory;
  u32 inst_index;
} hb_jpu_dev_t;

typedef struct _hb_jpu_priv {
	hb_jpu_dev_t *jpu_dev;
	u32 inst_index;
} hb_jpu_priv_t;

#endif /* __HOBOT_JPU_UTILS_H__ */
