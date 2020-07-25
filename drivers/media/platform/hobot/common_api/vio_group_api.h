/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __VIO_GROUP_API_H__
#define __VIO_GROUP_API_H__

#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/wait.h>

#include "vio_framemgr.h"
#include "vio_config.h"

#define MAX_SUB_DEVICE  8
#define MAX_SHADOW_NUM 4

#define GROUP_ID_SIF_OUT	0
#define GROUP_ID_SIF_IN		1
#define GROUP_ID_IPU		2
#define GROUP_ID_PYM		3
#define GROUP_ID_NUMBER		4

#define X3_IAR_INTERFACE
#define SET_CPU_AFFINITY

#define IPU0_IDLE    BIT(12)
#define PYM_IDLE	BIT(14)

#define SIF_CAP_FS		0
#define SIF_CAP_FE		1
#define SIF_IN_FS		2
#define SIF_IN_FE		3
#define IPU_FS			4
#define IPU_US_FE		5
#define IPU_DS0_FE		6
#define IPU_DS1_FE		7
#define IPU_DS2_FE		8
#define IPU_DS3_FE		9
#define IPU_DS4_FE		10
#define PYM_FS			11
#define PYM_FE			12
#define GDC_FS			13
#define GDC_FE			14
#define ISP_FS			15
#define ISP_FE			16
#define STAT_NUM		17

#define MAX_DELAY_FRAMES 5

enum vio_group_task_state {
	VIO_GTASK_START,
	VIO_GTASK_REQUEST_STOP,
	VIO_GTASK_SHOT,
	VIO_GTASK_SHOT_STOP,
};

enum vio_group_state {
	VIO_GROUP_OPEN,
	VIO_GROUP_INIT,
	VIO_GROUP_START,
	VIO_GROUP_FORCE_STOP,
	VIO_GROUP_OTF_INPUT,
	VIO_GROUP_OTF_OUTPUT,
	VIO_GROUP_DMA_INPUT,
	VIO_GROUP_DMA_OUTPUT,
	VIO_GROUP_LEADER,
};

struct vio_group_task {
	struct task_struct		*task;
	struct kthread_worker	worker;
	unsigned long				state;
	atomic_t			refcount;
	struct semaphore    hw_resource;
	u32 id;
};

struct vio_group {
	spinlock_t 			slock;
	void *sub_ctx[MAX_SUB_DEVICE];
	struct frame_id frameid;
	unsigned long state;
	atomic_t rcount; /* request count */
	u32 id;
	u32 instance;
	u32 sema_flag;
	u32 target_sema;
	bool get_timestamps;
	bool leader;
	u32 output_flag;
	atomic_t node_refcount;
	struct vio_group		*next;
	struct vio_group		*prev;
	struct vio_group		*head;
	struct vio_chain		*chain;
	struct vio_group_task *gtask;
	void (*frame_work)(struct vio_group *group);
};

struct vio_work {
	struct kthread_work work;
	struct vio_group *group;
};

struct vio_video_ctx {
	wait_queue_head_t		done_wq;
	struct vio_framemgr 	framemgr;
	struct vio_group		*group;
	unsigned long			state;

	u32 id;
	u32 event;
	bool leader;
};

struct statinfo {
	int framid;
	struct timeval g_tv;
};

struct vio_chain {
	struct vio_group group[GROUP_ID_NUMBER];
	struct statinfo statinfo[MAX_DELAY_FRAMES][STAT_NUM];
	unsigned long statinfoidx;
	unsigned long state;
};

struct vio_core {
       struct vio_chain chain[VIO_MAX_STREAM];
       atomic_t rsccount;
};

typedef int (*isp_callback)(int);

int vio_group_task_start(struct vio_group_task *group_task);
int vio_group_task_stop(struct vio_group_task *group_task);
void vio_group_start_trigger(struct vio_group *group, struct vio_frame *frame);
void vio_group_start_trigger_mp(struct vio_group *group, struct vio_frame *frame);

struct vio_group *vio_get_chain_group(int instance, u32 group_id);
int vio_bind_chain_groups(struct vio_group *src_group, struct vio_group *dts_group);
int vio_init_chain(int instance);
void vio_bind_group_done(int instance);
void vio_get_frame_id(struct vio_group *group);
int vio_group_init_mp(u32 group_id);
void vio_reset_module(u32 module);
void vio_group_done(struct vio_group *group);
void vio_dwe_clk_enable(void);
void vio_dwe_clk_disable(void);
void vio_set_stat_info(u32 instance, u32 stat_type, u16 frameid);
void vio_print_stat_info(u32 instance);
int vio_print_delay(s32 instance, s8* buf, u32 size);
void vio_exchanage_stat_info(u32 instance);
void vio_clear_stat_info(u32 instance);

#ifdef X3_IAR_INTERFACE
extern u32 ipu_get_iar_display_type(u8 *pipeline, u8 *channel);
extern int32_t ipu_set_display_addr(uint32_t disp_layer, u32 yaddr, u32 caddr);
#endif

extern void ips_set_module_reset(unsigned long module);
extern int ips_set_clk_ctrl(unsigned long module, bool enable);
extern int ips_set_bus_ctrl(unsigned int cfg);
extern int ips_get_bus_ctrl(void);

extern int ips_get_bus_status(void);
extern int ips_set_md_cfg(sif_output_md_t *cfg);
extern int ips_disable_md(void);
extern int ips_set_md_refresh(bool enable);
extern int ips_set_md_resolution(u32 width, u32 height);
extern int ips_get_md_event(void);
extern int ips_set_md_fmt(u32 fmt);
extern void ips_set_iram_size(u32 iram_size);

extern int vio_clk_enable(const char *name);
extern int vio_clk_disable(const char *name);
extern int vio_set_clk_rate(const char *name, ulong frequency);
extern ulong vio_get_clk_rate(const char *name);
extern int ion_check_in_heap_carveout(phys_addr_t start, size_t size);

extern struct class *vps_class;
extern int sif_mclk_freq;

#endif
