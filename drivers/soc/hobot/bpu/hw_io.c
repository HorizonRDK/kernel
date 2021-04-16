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

#include <linux/types.h>
#include <linux/slab.h>
#include <asm/delay.h>
#include <linux/device.h>
#include <linux/reset.h>
#include "bpu.h"
#include "bpu_ctrl.h"
#include "hw_io.h"

uint32_t bpu_core_reg_read(const struct bpu_core *core, uint32_t offset)
{
	return readl(core->base + offset);/*PRQA S ALL*/
}

void bpu_core_reg_write(const struct bpu_core *core,
		uint32_t offset, uint32_t val)
{
	writel(val, core->base + offset);/*PRQA S ALL*/
}

uint32_t bpu_core_safe_reg_read(const struct bpu_core *core, uint32_t offset)
{
	uint32_t val1, val2, val3;

	val1 = bpu_core_reg_read(core, offset);
	val2 = bpu_core_reg_read(core, offset);

	if (val1 == val2) {
		return val1;
	}

	val3 = bpu_core_reg_read(core, offset);
	if (val1 == val3) {
		return val1;
	}

	if (val2 == val3) {
		return val2;
	}

	pr_err("BPU Core safe reg[0x%x] read failed!!\n", offset);/*PRQA S ALL*/

	return val1;
}

/* compare write and expected value to guarant write success */
int32_t bpu_core_safe_reg_write(const struct bpu_core *core,
		uint32_t offset, uint32_t val)
{
	uint32_t tmp_val;

	bpu_core_reg_write(core, offset, val);
	tmp_val = bpu_core_safe_reg_read(core, offset);

	if (tmp_val == val) {
		return 0;
	} else {
		/* may be report error to dignose*/
		return -EINVAL;
	}
}

int32_t bpu_core_hw_reset(const struct bpu_core *core, uint32_t delay_time)
{
	int32_t ret;

	if (core->rst == NULL) {
		/* some platform do not has rst, so just Warning */
		pr_err("No reset ctrl have!!!\n");/*PRQA S ALL*/
		return 0;
	}

	ret = bpu_reset_ctrl(core->rst, 1);
	if (ret < 0) {
		pr_err("reset assert failed\n");/*PRQA S ALL*/
		return ret;
	}
	udelay(delay_time);/*PRQA S ALL*/
	ret = bpu_reset_ctrl(core->rst, 0);
	if (ret < 0) {
		pr_err("reset deassert failed\n");/*PRQA S ALL*/
		return ret;
	}

	return ret;
}
