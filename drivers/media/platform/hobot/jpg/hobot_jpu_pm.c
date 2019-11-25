#include <linux/platform_device.h>
#include "hobot_jpu_pm.h"
#include "hobot_jpu_debug.h"

int hb_jpu_init_pm(struct device *dev)
{
	return 0;
}

void hb_jpu_final_pm(struct device *dev)
{
}

struct clk *hb_jpu_clk_get(struct device *dev)
{
	return clk_get(dev, JPU_CLK_NAME);
}

void hb_jpu_clk_put(struct clk *clk)
{
	if (!(clk == NULL || IS_ERR(clk)))
		clk_put(clk);
}

int hb_jpu_clk_enable(struct clk *clk)
{

	if (clk) {
		jpu_debug(5, "jpu_clk_enable\n");
		// You need to implement it
		return 1;
	}
	return 0;
}

void hb_jpu_clk_disable(struct clk *clk)
{
	if (!(clk == NULL || IS_ERR(clk))) {
		jpu_debug(5, "jpu_clk_disable\n");
		// You need to implement it
	}
}
