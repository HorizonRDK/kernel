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

int hb_vpu_clk_get(hb_vpu_dev_t *dev, unsigned long freq)
{
	int rate = 0, round_rate = 0;
	int ret = 0;
	if (!dev) {
		return -1;
	}
	if (freq <= 0 || freq > MAX_VPU_FREQ) {
		freq = MAX_VPU_FREQ;
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
	if (dev->vpu_bclk) {
		rate = clk_get_rate(dev->vpu_bclk);
		if (rate != freq) {
			round_rate = clk_round_rate(dev->vpu_bclk, freq);
			ret = clk_set_rate(dev->vpu_bclk, round_rate);
			if (ret) {
				dev_err(dev->device, "failed to set clk(%s).\n", VPU_VCPU_BPU_CLK_NAME);
				clk_disable_unprepare(dev->vpu_aclk);
				dev->vpu_aclk = NULL;
				dev->vpu_bclk = NULL;
				dev->vpu_cclk = NULL;
				return -1;
			}
		}
	}
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
	if (dev->vpu_cclk) {
		rate = clk_get_rate(dev->vpu_cclk);
		if (rate != freq) {
			round_rate = clk_round_rate(dev->vpu_bclk, freq);
			ret = clk_set_rate(dev->vpu_cclk, round_rate);
			if (ret) {
				dev_err(dev->device, "failed to set clk(%s).\n", VPU_VCE_CLK_NAME);
				clk_disable_unprepare(dev->vpu_aclk);
				clk_disable_unprepare(dev->vpu_bclk);
				dev->vpu_aclk = NULL;
				dev->vpu_bclk = NULL;
				dev->vpu_cclk = NULL;
				return -1;
			}
		}
	}
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

int hb_vpu_clk_enable(hb_vpu_dev_t *dev,  unsigned long freq)
{
	int rate = 0, round_rate = 0;
	int ret = 0;
	if (!dev)
		return -1;

	if (freq <= 0 || freq > MAX_VPU_FREQ) {
		freq = MAX_VPU_FREQ;
	}
	ret = clk_prepare_enable(dev->vpu_aclk);
	if (ret) {
		dev_err(dev->device, "failed to enable clk(%s).\n", VPU_ACLK_NAME);
		return -1;
	}

	rate = clk_get_rate(dev->vpu_bclk);
	if (rate != freq) {
		round_rate = clk_round_rate(dev->vpu_bclk, freq);
		ret = clk_set_rate(dev->vpu_bclk, round_rate);
		if (ret) {
			dev_err(dev->device, "failed to set clk(%s).\n", VPU_VCPU_BPU_CLK_NAME);
			clk_disable_unprepare(dev->vpu_aclk);
			return -1;
		}
		dev_info(dev->device, "%s clock is %d\n", VPU_VCPU_BPU_CLK_NAME, round_rate);
	}
	ret = clk_prepare_enable(dev->vpu_bclk);
	if (ret) {
		clk_disable_unprepare(dev->vpu_aclk);
		dev_err(dev->device, "failed to enable clk(%s).\n", VPU_VCPU_BPU_CLK_NAME);
		return -1;
	}

	rate = clk_get_rate(dev->vpu_cclk);
	if (rate != freq) {
		round_rate = clk_round_rate(dev->vpu_bclk, freq);
		ret = clk_set_rate(dev->vpu_cclk, round_rate);
		if (ret) {
			dev_err(dev->device, "failed to set clk(%s).\n", VPU_VCE_CLK_NAME);
			clk_disable_unprepare(dev->vpu_aclk);
			clk_disable_unprepare(dev->vpu_bclk);
			return -1;
		}
		dev_info(dev->device, "%s clock is %d\n", VPU_VCE_CLK_NAME, round_rate);
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
