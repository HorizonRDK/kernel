/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_JPU_PM_H__
#define __HOBOT_JPU_PM_H__

#include <linux/clk.h>
#include "hobot_jpu_utils.h"

int hb_jpu_init_pm(struct device *dev);
void hb_jpu_final_pm(struct device *dev);

int hb_jpu_clock_on(void);
void hb_jpu_clock_off(void);
int hb_jpu_power_on(void);
int hb_jpu_power_off(void);

int hb_jpu_clk_get(hb_jpu_dev_t *dev);
void hb_jpu_clk_put(hb_jpu_dev_t *dev);
int hb_jpu_clk_enable(hb_jpu_dev_t *dev);
void hb_jpu_clk_disable(hb_jpu_dev_t *dev);

#endif /* __HOBOT_JPU_PM_H__ */
