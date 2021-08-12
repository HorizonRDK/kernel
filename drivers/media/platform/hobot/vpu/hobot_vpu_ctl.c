/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#include <linux/delay.h>
#include <linux/io.h>
#include "hobot_vpu_ctl.h"
#include "hobot_vpu_debug.h"

int hb_vpu_hw_reset(hb_vpu_dev_t *dev)
{
	// TODO realize this feature
	vpu_debug(5, "request vpu reset from application %p.\n", dev->rst_regs_base);
	writel(0x3, dev->rst_regs_base);
	udelay(100);
	writel(0x0, dev->rst_regs_base);
	return 0;
}
