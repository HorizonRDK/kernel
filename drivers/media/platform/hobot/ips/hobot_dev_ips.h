/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_DEV_IPS_H__
#define __HOBOT_DEV_IPS_H__

#include <uapi/linux/types.h>
#include <linux/cdev.h>
#include <linux/wait.h>

#include "vio_config.h"

struct x2a_ips_dev {
	u32 __iomem			*base_reg;
	resource_size_t			regs_start;
	resource_size_t			regs_end;
	int				irq;
	spinlock_t			shared_slock;
};

struct vio_clk {
	const char *name;
	struct clk *clk;
};

int ips_set_clk_ctrl(unsigned long module, bool enable);
int ips_set_bus_ctrl(unsigned int cfg);
int ips_get_bus_ctrl(void);

int ips_get_bus_status(void);

int vio_clk_enable(const char *name);
int vio_clk_disable(const char *name);
int vio_set_clk_rate(const char *name, ulong frequency);
ulong vio_get_clk_rate(const char *name);
#endif
