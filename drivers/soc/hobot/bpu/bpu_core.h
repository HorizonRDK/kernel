/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * Zhang Guoying <guoying.zhang@horizon.ai>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#ifndef __BPU_CORE_H__
#define __BPU_CORE_H__
#include <linux/interrupt.h>
#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
#include <linux/devfreq.h>
#endif
#include "bpu.h"
#include "bpu_prio.h"

#ifdef CONFIG_X2_BPU
#include "x2_bpu.h"
#elif defined CONFIG_J5_BPU
#include "j5_bpu.h"
#else
#define FC_MAX_DEPTH			1024
#define FC_DEPTH				FC_MAX_DEPTH
#define HW_ID_MAX	0xFFF
#define BPU_PRIO_NUM			0x1
#define FC_PRIO_ID(prio, id)	((id) & 0xFF) | ((prio) << 8)
#define FC_ID(id)				((id) & 0xFF)
#define FC_PRIO(id)				(0)
#endif

enum bpu_core_status_cmd {
	IF_BUSY = 0,
	/* get inventory fc num */
	INVENTORY_NUM,
	STATUS_CMD_MAX,
};

struct bpu_core_hw_ops;

#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
struct bpu_core_dvfs {
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;
	struct devfreq_dev_profile profile;
	u64 rate;
	u64 volt;
	/* store freq level num */
	u32 level_num;
};
#endif

struct bpu_core {
	struct list_head node;
	struct device *dev;
	struct bpu *host;
	struct miscdevice miscdev;
	atomic_t open_counter;

	atomic_t hw_id_counter[BPU_PRIO_NUM];

	/* use for bpu core pending */
	atomic_t pend_flag;

	struct bpu_prio *prio_sched;
	/*
	 * limit the fc number which write to
	 * hw fifo at the same time, which can
	 * influence prio sched max time.
	 */
	int fc_buf_limit;

	int irq;
	void __iomem *base;
	/*
	 * which store bpu read fc base
	 * alloc when core enable and
	 * free when core disable
	 */
	void *fc_base;
	dma_addr_t fc_base_addr;

	struct regulator *regulator;
	struct clk *aclk;
	struct clk *mclk;
	struct reset_control *rst;

	DECLARE_KFIFO_PTR(run_fc_fifo[BPU_PRIO_NUM], struct bpu_fc);
	DECLARE_KFIFO_PTR(done_fc_fifo, struct bpu_fc);

	struct mutex mutex_lock;
	spinlock_t spin_lock;

	/* list to store user */
	struct list_head user_list;

	/* key point time for statistics */
	struct timeval k_point;

	struct bpu_core_hw_ops *hw_ops;
	int index;

#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
	struct bpu_core_dvfs *dvfs;
#else
	void *dvfs;
#endif
	/* bpu core ctrl */
	int running_task_num;
	struct completion no_task_comp;
	/* > 0; auto change by governor; <=0: manual levle, 0 is highest */
	int power_level;

	int hw_enabled;

	struct tasklet_struct tasklet;
	/* the flowing for statistics */
	struct timeval last_done_point;
	/* run time in statistical period */
	struct timeval p_start_point;

	uint64_t p_run_time;
	/* running ratio */
	uint8_t ratio;

	uint64_t reserved[2];
};

struct bpu_core_hw_ops {
	int (*enable) (struct bpu_core *);
	int (*disable) (struct bpu_core *);
	int (*reset) (struct bpu_core *);
	int (*set_clk) (struct bpu_core *, unsigned long);
	int (*set_volt) (struct bpu_core *, int);
	/*
	 * write real fc to hw, return > 0: actual write fc num
	 * param include the offset pos in the bpu_fc raw slices
	 */
	int (*write_fc) (struct bpu_core *, struct bpu_fc *fc, unsigned int);
	/* get the fc process return */
	int (*read_fc) (struct bpu_core *, u32 *, u32 *);

	/* get bpu hw core running status */
	int (*status) (struct bpu_core *, int);

	/* debug info for hw info */
	int (*debug) (struct bpu_core *, int);
};

#endif
