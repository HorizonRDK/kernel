/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
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

int hb_jpu_clk_get(hb_jpu_dev_t *dev)
{
	int rate = 0;
	if (!dev) {
		return -1;
	}
	dev->jpu_cclk = devm_clk_get(dev->device, JPU_JPEG_CLK_NAME);
	if ((!dev->jpu_cclk) || clk_prepare_enable(dev->jpu_cclk)) {
		dev_err(dev->device, "failed to get clk(%s).\n", JPU_JPEG_CLK_NAME);
		dev->jpu_cclk = NULL;
		return -1;
	}

	rate = clk_get_rate(dev->jpu_cclk);
	dev_info(dev->device, "%s clock is %d\n", JPU_JPEG_CLK_NAME, rate);

	return 0;
}

void hb_jpu_clk_put(hb_jpu_dev_t *dev)
{
	if (!dev)
		return;
	clk_disable_unprepare(dev->jpu_cclk);
}

int hb_jpu_clk_enable(hb_jpu_dev_t *dev)
{
	int ret = 0;
	if (!dev)
		return -1;
	ret = clk_prepare_enable(dev->jpu_cclk);
	if (ret) {
		dev_err(dev->device, "failed to enable clk(%s).\n", JPU_JPEG_CLK_NAME);
		return -1;
	}

	return 0;
}

void hb_jpu_clk_disable(hb_jpu_dev_t *dev)
{
	if (!dev)
		return;
	clk_disable_unprepare(dev->jpu_cclk);
}
