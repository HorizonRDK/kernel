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
#include <linux/pm_qos.h>
#include <linux/ion.h>
#include "hobot_vpu_user.h"
#include "hobot_vpu_config.h"

// TODO remove this
/* if this driver knows the dedicated video memory address */
//#define VPU_SUPPORT_RESERVED_VIDEO_MEMORY

#if defined(linux) || defined(__linux) || defined(ANDROID) || defined(CNM_FPGA_HAPS_INTERFACE)
#define SUPPORT_MULTI_INST_INTR
#endif

#define VPU_PLATFORM_DEVICE_NAME "hb_vpu"
#define VPU_ACLK_NAME "vpu_aclk"
#define VPU_VCPU_BPU_CLK_NAME "vpu_bclk"
#define VPU_VCE_CLK_NAME "vpu_cclk"

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
	unsigned char codec_inst_pool[MAX_NUM_VPU_INSTANCE][MAX_INST_HANDLE_SIZE];
//#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
	hb_vpu_drv_buffer_t vpu_common_buffer;
	int vpu_instance_num;
	int instance_pool_inited;
	void* pendingInst;
	int pendingInstIdxPlus1;
	int doSwResetInstIdxPlus1;
//#endif
} hb_vpu_instance_pool_t;

typedef struct _hb_vpu_dev {
	struct device *device;
	hb_vpu_platform_data_t *plat_data;
	hb_vpu_driver_data_t *drv_data;
	struct resource *vpu_mem;
	void __iomem *regs_base;
	struct resource *vpu_reset;
	void __iomem *rst_regs_base;
	int irq;
	int vpu_dev_num;
	int major;
	int minor;
	struct class *vpu_class;
	struct cdev cdev;
	struct device *vpu_dev;
	struct kobject *vpu_kobj;
	struct clk *vpu_aclk;
	struct clk *vpu_bclk;
	struct clk *vpu_cclk;
	struct ion_client *vpu_ion_client;

	wait_queue_head_t poll_int_wait_q[MAX_NUM_VPU_INSTANCE];
	int64_t poll_int_event[MAX_NUM_VPU_INSTANCE];
	wait_queue_head_t poll_wait_q[MAX_NUM_VPU_INSTANCE];
	int64_t poll_event[MAX_NUM_VPU_INSTANCE];
	int64_t total_poll[MAX_NUM_VPU_INSTANCE];
	int64_t total_release[MAX_NUM_VPU_INSTANCE];
#ifdef SUPPORT_MULTI_INST_INTR
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
	spinlock_t irq_spinlock;
	int irq_trigger;

	struct fasync_struct *async_queue;
	u32 open_count;		/*!<< device reference count. Not instance count */
	int vpu_open_ref_count;
	unsigned long vpu_freq;

	hb_vpu_drv_firmware_t bit_fm_info[MAX_NUM_VPU_CORE];

	u32 vpu_reg_store[MAX_NUM_VPU_CORE][64];
	struct mutex vpu_mutex;
	struct semaphore vpu_sem;
	spinlock_t vpu_spinlock;
	spinlock_t vpu_info_spinlock;
	struct list_head vbp_head;
	struct list_head inst_list_head;

	hb_vpu_drv_buffer_t instance_pool;
	hb_vpu_drv_buffer_t common_memory;
	hb_vpu_ctx_info_t vpu_ctx[MAX_NUM_VPU_INSTANCE];
	hb_vpu_status_info_t vpu_status[MAX_NUM_VPU_INSTANCE];

	struct dentry *debug_root;
	struct dentry *debug_file_venc;
	struct dentry *debug_file_vdec;

#ifdef USE_MUTEX_IN_KERNEL_SPACE
	/* PID aquiring the vdi lock */
	pid_t current_vdi_lock_pid[VPUDRV_MUTEX_MAX];
	struct semaphore vpu_vdi_sem;
	struct semaphore vpu_vdi_disp_sem;
	struct semaphore vpu_vdi_reset_sem;
	struct semaphore vpu_vdi_vmem_sem;
#endif
	struct pm_qos_request vpu_pm_qos_req;
} hb_vpu_dev_t;

typedef struct _hb_vpu_priv {
	hb_vpu_dev_t *vpu_dev;
	u32 inst_index;
	u32 is_irq_poll;
} hb_vpu_priv_t;

struct vpu_clk_dev {
	struct cdev	cdev;
	struct class 	*class;
	dev_t 		devno;
	// spinlock_t	slock;
};

#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
#define VPU_WAKE_MODE 0
#define VPU_SLEEP_MODE 1
typedef enum {
	VPUAPI_RET_SUCCESS,
	VPUAPI_RET_FAILURE, // an error reported by FW
	VPUAPI_RET_TIMEOUT,
	VPUAPI_RET_STILL_RUNNING,
	VPUAPI_RET_INVALID_PARAM,
	VPUAPI_RET_MAX
} VpuApiRet;
int vpuapi_close(hb_vpu_dev_t *dev, u32 core, u32 inst);
int vpuapi_dec_set_stream_end(hb_vpu_dev_t *dev, u32 core, u32 inst);
int vpuapi_dec_clr_all_disp_flag(hb_vpu_dev_t *dev, u32 core, u32 inst);
int vpuapi_get_output_info(hb_vpu_dev_t *dev, u32 core, u32 inst, u32 *error_reason);
#if defined(CONFIG_PM)
int vpu_sleep_wake(hb_vpu_dev_t *dev, u32 core, int mode);
#endif
int vpu_do_sw_reset(hb_vpu_dev_t *dev, u32 core, u32 inst, u32 error_reason);
int vpu_close_instance(hb_vpu_dev_t *dev, u32 core, u32 inst);
int vpu_check_is_decoder(hb_vpu_dev_t *dev, u32 core, u32 inst);
#endif

#endif /* __HOBOT_VPU_UTILS_H__ */
