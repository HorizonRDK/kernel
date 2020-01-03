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
#include "bpu.h"

struct bpu_core_hw_ops;

struct bpu_core {
	struct list_head node;
	struct device *dev;
	struct bpu *host;
	struct miscdevice miscdev;
	atomic_t open_counter;

	atomic_t hw_id_counter;

	int irq;
	void __iomem *base;
	/*
	 * which store bpu read fc base
	 * alloc when core enable and
	 * free when core disable
	 */
	void *fc_base;

	struct regulator *regulator;
	struct clk *aclk;
	struct clk *mclk;
	struct reset_control *rst;

	DECLARE_KFIFO_PTR(run_fc_fifo, struct bpu_fc);
	DECLARE_KFIFO_PTR(done_fc_fifo, struct bpu_fc);

	struct mutex mutex_lock;
	spinlock_t spin_lock;

	/* list to store user */
	struct list_head user_list;

	/* key point time for statistics */
	struct timeval k_point;

	struct bpu_core_hw_ops *hw_ops;
	int index;

	/* bpu core ctrl */
	int queue;
	long clk;
	int power_level;

	struct tasklet_struct tasklet;
	/* the flowing for statistics */
	struct timeval last_done_point;
	/* run time in statistical period */
	struct timeval p_start_point;

	uint64_t p_run_time;
	/* running ratio */
	uint8_t ratio;

	/* for debug */
	struct attribute_group *debug_group;
};

struct bpu_core_hw_ops {
	int (*enable) (struct bpu_core *);
	int (*disable) (struct bpu_core *);
	int (*reset) (struct bpu_core *);
	int (*set_clk) (struct bpu_core *, unsigned long);
	int (*set_volt) (struct bpu_core *, int);
	/* write real fc to hw */
	int (*write_fc) (struct bpu_core *, struct bpu_fc *fc);
	/* get the fc process return */
	int (*read_fc) (struct bpu_core *);

	/* debug info for hw info */
	int (*debug) (struct bpu_core *);
};

#ifdef CONFIG_X2_BPU
#include "x2_bpu.h"
#else
#define HW_ID_MAX	0xFFF
#endif

#endif
