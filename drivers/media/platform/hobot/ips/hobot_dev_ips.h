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
#include "sif_config.h"

#define IRAM_MAX_RANG	0x140000

struct x3_ips_dev {
	u32 __iomem			*base_reg;
	resource_size_t			regs_start;
	resource_size_t			regs_end;
	int				irq;
	spinlock_t			shared_slock;
	wait_queue_head_t		done_wq;
	u32 event;
	u32 iram_used_size;
};

struct vio_clk {
	const char *name;
	struct clk *clk;
};

void ips_set_module_reset(unsigned long module);
int ips_set_clk_ctrl(unsigned long module, bool enable);
int ips_set_bus_ctrl(unsigned int cfg);
int ips_get_bus_ctrl(void);

int ips_get_bus_status(void);
int ips_set_md_cfg(sif_output_md_t *cfg);
int ips_disable_md(void);
int ips_set_md_refresh(bool enable);
int ips_set_md_resolution(u32 width, u32 height);
int ips_get_md_event(void);
int ips_set_md_fmt(u32 fmt);
void ips_set_iram_size(u32 iram_size);

int vio_clk_enable(const char *name);
int vio_clk_disable(const char *name);
int vio_set_clk_rate(const char *name, ulong frequency);
ulong vio_get_clk_rate(const char *name);
#endif
