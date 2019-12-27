/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_VPU_UTILS_H__
#define __HOBOT_VPU_UTILS_H__

#include <linux/wait.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>

#include "hobot_vpu_user.h"
#include "hobot_vpu_config.h"
//#include "../../../../../test_code/vsp/vpuapi/vpuconfig.h"

// TODO remove this
/* if this driver knows the dedicated video memory address */
#define VPU_SUPPORT_RESERVED_VIDEO_MEMORY

#if defined(linux) || defined(__linux) || defined(ANDROID) || defined(CNM_FPGA_HAPS_INTERFACE)
#define SUPPORT_MULTI_INST_INTR
#endif

#define VPU_PLATFORM_DEVICE_NAME "hb_vpu"
#define VPU_CLK_NAME "vcodec"

typedef enum _hb_vpu_event_e {
	VPU_EVENT_NONE = 0,
	VPU_ENC_PIC_DONE = 1,
	VPU_DEC_PIC_DONE = 2,
	VPU_INST_CLOSED = 3,
} hb_vpu_event_t;

typedef struct _hb_vpu_driver_data {
	char *fw_name;
} hb_vpu_driver_data_t;

typedef struct _hb_vpu_platform_data {
	int ip_ver;
	int clock_rate;
	int min_rate;
} hb_vpu_platform_data_t;

/* To track the allocated memory buffer */
typedef struct _hb_vpu_drv_buffer_pool {
	struct list_head list;
	hb_vpu_drv_buffer_t vb;
	struct file *filp;
} hb_vpu_drv_buffer_pool_t;

/* To track the instance index and buffer in instance pool */
typedef struct _hb_vpu_instance_list {
	struct list_head list;
	unsigned long inst_idx;
	unsigned long core_idx;
	struct file *filp;
} hb_vpu_instance_list_t;

typedef struct _hb_vpu_instance_pool {
	unsigned char
	 codec_inst_pool[MAX_NUM_VPU_INSTANCE][MAX_INST_HANDLE_SIZE];
} hb_vpu_instance_pool_t;

typedef struct _hb_vpu_dev {
	struct device *device;
	hb_vpu_platform_data_t *plat_data;
	hb_vpu_driver_data_t *drv_data;
	struct resource *vpu_mem;
	void __iomem *regs_base;
	int irq;
	int vpu_dev_num;
	int major;
	int minor;
	struct class *vpu_class;
	struct cdev cdev;
	struct device *vpu_dev;
	struct kobject *vpu_kobj;
	struct clk *vpu_clk;
#ifdef CONFIG_ION_HOBOT
	struct ion_client *vpu_ion_client;
#endif

#ifdef SUPPORT_MULTI_INST_INTR
	wait_queue_head_t poll_wait_q[MAX_NUM_VPU_INSTANCE];
	hb_vpu_event_t poll_event[MAX_NUM_VPU_INSTANCE];
	spinlock_t poll_spinlock;
	wait_queue_head_t interrupt_wait_q[MAX_NUM_VPU_INSTANCE];
	int interrupt_flag[MAX_NUM_VPU_INSTANCE];
	struct kfifo interrupt_pending_q[MAX_NUM_VPU_INSTANCE];
	spinlock_t vpu_kfifo_lock;
	unsigned long interrupt_reason[MAX_NUM_VPU_INSTANCE];
#else
	wait_queue_head_t interrupt_wait_q;
	int interrupt_flag;
	unsigned long interrupt_reason;
#endif

	struct fasync_struct *async_queue;
	u32 open_count;		/*!<< device reference count. Not instance count */
	int vpu_open_ref_count;

	hb_vpu_drv_firmware_t bit_fm_info[MAX_NUM_VPU_CORE];

	u32 vpu_reg_store[MAX_NUM_VPU_CORE][64];
	struct mutex vpu_mutex;
	struct semaphore vpu_sem;
	spinlock_t vpu_spinlock;
	struct list_head vbp_head;
	struct list_head inst_list_head;

	hb_vpu_drv_buffer_t instance_pool;
	hb_vpu_drv_buffer_t common_memory;
} hb_vpu_dev_t;

typedef struct _hb_vpu_priv {
	hb_vpu_dev_t *vpu_dev;
	u32 inst_index;
} hb_vpu_priv_t;

#endif /* __HOBOT_VPU_UTILS_H__ */
