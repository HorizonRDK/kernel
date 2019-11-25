#include <linux/platform_device.h>
#include "hobot_vpu_pm.h"
#include "hobot_vpu_debug.h"

int hb_vpu_init_pm(struct device *dev)
{
	return 0;
}

void hb_vpu_final_pm(struct device *dev)
{
}

struct clk *hb_vpu_clk_get(struct device *dev)
{
	// TODO get the clock name from dts
	return clk_get(dev, VPU_CLK_NAME);
}

void hb_vpu_clk_put(struct clk *clk)
{
	if (!(clk == NULL || IS_ERR(clk)))
		clk_put(clk);
}

int hb_vpu_clk_enable(struct clk *clk)
{
	if (!(clk == NULL || IS_ERR(clk))) {
		/* the bellow is for C&M EVB. */
		/*
		   {
		   struct clk *s_vpuext_clk = NULL;
		   s_vpuext_clk = clk_get(NULL, "vcore");
		   if (s_vpuext_clk)
		   {
		   DPRINTK("[VPUDRV] vcore clk=%p\n", s_vpuext_clk);
		   clk_enable(s_vpuext_clk);
		   }

		   DPRINTK("[VPUDRV] vbus clk=%p\n", s_vpuext_clk);
		   if (s_vpuext_clk)
		   {
		   s_vpuext_clk = clk_get(NULL, "vbus");
		   clk_enable(s_vpuext_clk);
		   }
		   }
		 */
		/* for C&M EVB. */

		vpu_debug(5, "vpu_clk_enable\n");
		//customers needs implementation to turn on clock like clk_enable(clk)
		return 1;
	}

	return 0;
}

void hb_vpu_clk_disable(struct clk *clk)
{
	if (!(clk == NULL || IS_ERR(clk))) {
		vpu_debug(5, "vpu_clk_disable\n");
		//customers needs implementation to turn off clock like clk_disable(clk)
	}
}
