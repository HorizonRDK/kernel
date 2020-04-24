/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
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

int hb_vpu_clk_get(hb_vpu_dev_t *dev)
{
	int rate = 0;
	if (!dev) {
		return -1;
	}

	dev->vpu_aclk = devm_clk_get(dev->device, VPU_ACLK_NAME);
	if ((!dev->vpu_aclk) || clk_prepare_enable(dev->vpu_aclk)) {
		dev_err(dev->device, "failed to get clk(%s).\n", VPU_ACLK_NAME);
		dev->vpu_aclk = NULL;
		dev->vpu_bclk = NULL;
		dev->vpu_cclk = NULL;
		return -1;
	}

	rate = clk_get_rate(dev->vpu_aclk);
	dev_info(dev->device, "%s clock is %d\n", VPU_ACLK_NAME, rate);

	dev->vpu_bclk = devm_clk_get(dev->device, VPU_VCPU_BPU_CLK_NAME);
	if ((!dev->vpu_bclk) || clk_prepare_enable(dev->vpu_bclk)) {
		dev_err(dev->device, "failed to get clk(%s).\n", VPU_VCPU_BPU_CLK_NAME);
		clk_disable_unprepare(dev->vpu_aclk);
		dev->vpu_aclk = NULL;
		dev->vpu_bclk = NULL;
		dev->vpu_cclk = NULL;
		return -1;
	}

	rate = clk_get_rate(dev->vpu_bclk);
	dev_info(dev->device, "%s clock is %d\n", VPU_VCPU_BPU_CLK_NAME, rate);

	dev->vpu_cclk = devm_clk_get(dev->device, VPU_VCE_CLK_NAME);
	if ((!dev->vpu_cclk) || clk_prepare_enable(dev->vpu_cclk)) {
		dev_err(dev->device, "failed to get clk(%s).\n", VPU_VCE_CLK_NAME);
		clk_disable_unprepare(dev->vpu_aclk);
		clk_disable_unprepare(dev->vpu_bclk);
		dev->vpu_aclk = NULL;
		dev->vpu_bclk = NULL;
		dev->vpu_cclk = NULL;
		return -1;
	}

	rate = clk_get_rate(dev->vpu_cclk);
	dev_info(dev->device, "%s clock is %d\n", VPU_VCE_CLK_NAME, rate);

	return 0;
}

void hb_vpu_clk_put(hb_vpu_dev_t *dev)
{
	if (!dev)
		return;
	clk_disable_unprepare(dev->vpu_aclk);
	clk_disable_unprepare(dev->vpu_bclk);
	clk_disable_unprepare(dev->vpu_cclk);
}

int hb_vpu_clk_enable(hb_vpu_dev_t *dev)
{
	int ret = 0;
	if (!dev)
		return -1;
	ret = clk_prepare_enable(dev->vpu_aclk);
	if (ret) {
		dev_err(dev->device, "failed to enable clk(%s).\n", VPU_ACLK_NAME);
		return -1;
	}
	ret = clk_prepare_enable(dev->vpu_bclk);
	if (ret) {
		clk_disable_unprepare(dev->vpu_aclk);
		dev_err(dev->device, "failed to enable clk(%s).\n", VPU_VCPU_BPU_CLK_NAME);
		return -1;
	}
	ret = clk_prepare_enable(dev->vpu_cclk);
	if (ret) {
		clk_disable_unprepare(dev->vpu_aclk);
		clk_disable_unprepare(dev->vpu_bclk);
		dev_err(dev->device, "failed to enable clk(%s).\n", VPU_VCE_CLK_NAME);
		return -1;
	}

	return 0;
}

void hb_vpu_clk_disable(hb_vpu_dev_t *dev)
{
	if (!dev)
		return;
	clk_disable_unprepare(dev->vpu_aclk);
	clk_disable_unprepare(dev->vpu_bclk);
	clk_disable_unprepare(dev->vpu_cclk);
}
