#ifndef __X2A_PYM_H__
#define __X2A_PYM_H__

#include <uapi/linux/types.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include "pym_config.h"
#include "../vio_config.h"
#include "../vio_framemgr.h"
#include "../vio_group_api.h"

#define MAX_SHADOW_NUM 4
#define X2A_PYM_NAME  "x2a-pym"

#define MAX_DEVICE  1
#define PYM_IOC_MAGIC 'p'

#define PYM_IOC_INIT             _IOW(PYM_IOC_MAGIC, 0, pym_cfg_t)
#define PYM_IOC_STREAM           _IOW(PYM_IOC_MAGIC, 1, int)
#define PYM_IOC_QBUF        	 _IOW(PYM_IOC_MAGIC, 2, int)
#define PYM_IOC_DQBUF       	 _IOR(PYM_IOC_MAGIC, 3, int)
#define PYM_IOC_REQBUFS       	 _IOW(PYM_IOC_MAGIC, 4, int)
#define PYM_IOC_EOS       	 	 _IOW(PYM_IOC_MAGIC, 5, int)


struct roi_rect{
    u16 roi_x;
    u16 roi_y;
    u16 roi_width;
    u16 roi_height;
};

struct pym_video_ctx{
	wait_queue_head_t		done_wq;
	struct x2a_pym_dev 		*pym_dev;

	struct vio_framemgr 	framemgr;
	struct pym_group		*group;

	unsigned long			state;
	struct semaphore 		smp_resource;
	bool leader;
};

enum pym_group_state {
	PYM_GROUP_OPEN,
	PYM_GROUP_INIT,
	PYM_GROUP_START,
	PYM_GROUP_SHOT,
	PYM_GROUP_REQUEST_FSTOP,
	PYM_GROUP_FORCE_STOP,
	PYM_GROUP_LEADER,
};

enum group_id{
	GROUP_ID_SRC,
	GROUP_ID_MAX,
};

enum pym_interrupt_map{
	INTR_PYM_DS_FRAME_DROP,
	INTR_PYM_US_FRAME_DROP,
	INTR_PYM_FRAME_DONE,
	INTR_PYM_FRAME_START,
};

enum pym_status{
	PYM_OTF_INPUT,
	PYM_DMA_INPUT,
};
struct pym_group{
	struct pym_video_ctx *sub_ctx[MAX_DEVICE];
	unsigned long				state;
	u32 instance;
};

struct x2a_pym_dev {
	/* channel information */
	u32 __iomem			*base_reg;
	resource_size_t			regs_start;
	resource_size_t			regs_end;
	int				irq;
	unsigned long			state;

	struct class *class;
	struct cdev cdev;

	atomic_t			instance;
	atomic_t			rsccount;
	struct semaphore smp_pym_enable;
	spinlock_t			shared_slock;

	struct pym_group group[VIO_MAX_STREAM];
	struct vio_group_task gtask;
};

#endif
