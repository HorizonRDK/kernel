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

int ips_set_clk_ctrl(unsigned long module, bool enable);


#endif
