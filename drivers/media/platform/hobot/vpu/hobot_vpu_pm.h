/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_VPU_PM_H__
#define __HOBOT_VPU_PM_H__

#include <linux/clk.h>
#include "hobot_vpu_utils.h"

int hb_vpu_init_pm(struct device *dev);
void hb_vpu_final_pm(struct device *dev);

int hb_vpu_clock_on(void);
void hb_vpu_clock_off(void);
int hb_vpu_power_on(void);
int hb_vpu_power_off(void);

int hb_vpu_clk_get(hb_vpu_dev_t *dev);
void hb_vpu_clk_put(hb_vpu_dev_t *dev);
int hb_vpu_clk_enable(hb_vpu_dev_t *dev);
void hb_vpu_clk_disable(hb_vpu_dev_t *dev);

#endif /* __HOBOT_VPU_PM_H__ */
