#ifndef __IPU_DRV_DUAL_H__
#define __IPU_DRV_DUAL_H__

#include "ipu_dev.h"

struct ipu_dual_cdev {
	const char *name;
	struct x2_ipu_data *ipu;
	void *vaddr;
	struct class *class;
	struct cdev cdev;
	dev_t dev_num;
	spinlock_t slock;
	struct mutex mutex_lock;
	wait_queue_head_t event_head;
	bool pymid_done;
	bool stop;
	int8_t done_idx;
	uint32_t err_status;
	unsigned long ipuflags;
	slot_ddr_info_dual_t s_info;
	int open_counter;
};


#endif
