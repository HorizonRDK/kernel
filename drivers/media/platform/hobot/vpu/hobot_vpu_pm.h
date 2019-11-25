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

struct clk *hb_vpu_clk_get(struct device *dev);
void hb_vpu_clk_put(struct clk *clk);
int hb_vpu_clk_enable(struct clk *clk);
void hb_vpu_clk_disable(struct clk *clk);

#endif /* __HOBOT_VPU_PM_H__ */
