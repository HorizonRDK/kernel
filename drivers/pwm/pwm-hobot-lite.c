/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright	2020 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/
#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

/* the offset of pwm registers */
#define LPWM_EN       0x00
#define LPWM0_CFG     0x04
#define LPWM1_CFG     0x08
#define LPWM2_CFG     0x0C
#define LPWM3_CFG     0x10
#define LPWM_SW_TRIG  0x14
#define LPWM_RST      0x20

#define LPWM_INT_EN   BIT(4)
#define LPWM_MODE_PPS_TRIG BIT(5)
#define LPWM_NAME     "hobot-lpwm"
#define LPWM_NPWM 4

struct hobot_lpwm_chip {
	struct pwm_chip chip;
	int irq;
	char name[8];
	struct clk *clk;
	void __iomem *base;
	int offset[LPWM_NPWM];
};

#define to_hobot_lpwm_chip(_chip) \
	container_of(_chip, struct hobot_lpwm_chip, chip)

/* IO accessors */
static inline u32 hobot_lpwm_rd(struct hobot_lpwm_chip *hobot_chip, u32 reg)
{
	return ioread32(hobot_chip->base + reg);
}

static inline void hobot_lpwm_wr(struct hobot_lpwm_chip *hobot_chip,
						u32 reg, u32 value)
{
	iowrite32(value, hobot_chip->base + reg);
}

static int hobot_lpwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns)
{
	u32 cfg, reg;
	int period, high;
	struct hobot_lpwm_chip *lpwm = to_hobot_lpwm_chip(chip);

	if (period_ns > 0 && (period_ns < 250000 || period_ns > 1000000000)) {
		pr_err("lpwm only support period in [250000ns~1000000000ns]\n");
		return -ERANGE;
	}

	if (duty_ns > 0 && (duty_ns < 250000 || duty_ns > 4000000)) {
		pr_err("lpwm only support duty_cycle in [250000ns~1000000000ns]\n");
		return -ERANGE;
	}

	/* config pwm freq */
	period = div64_u64((uint64_t)period_ns, (uint64_t)250000) - 1;
	high = div64_u64((uint64_t)duty_ns, (uint64_t)250000) - 1;

	pr_debug("set period_ns: %d, duty_ns: %d\n", period_ns, duty_ns);

	reg = (pwm->hwpwm * 0x4) + LPWM0_CFG;
	cfg = hobot_lpwm_rd(lpwm, reg);
	cfg &= 0xFFF;//keep offset values
	cfg = period << 12 | high << 24 | cfg;

	pr_debug("set pwm:%d period_ns:%d, duty_ns:%d, cfg:0x%08x to reg:0x%08x\n",
		pwm->hwpwm, period_ns, duty_ns, cfg, reg);

	hobot_lpwm_wr(lpwm, reg, cfg);

	return 0;
}

static int hobot_lpwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	struct hobot_lpwm_chip *lpwm = to_hobot_lpwm_chip(chip);

	val = hobot_lpwm_rd(lpwm, LPWM_EN);
	val |= (1 << pwm->hwpwm);
	val |= LPWM_EN;
	hobot_lpwm_wr(lpwm, LPWM_EN, val);

	hobot_lpwm_wr(lpwm, LPWM_SW_TRIG, 1);

	pr_debug("enable lpwm%d, LPWM_EN: 0x%08x, LPWM_CFG: 0x%08x \n",
		pwm->hwpwm, hobot_lpwm_rd(lpwm, LPWM_EN),
		hobot_lpwm_rd(lpwm, (pwm->hwpwm * 4) + LPWM0_CFG));

	return 0;
}

static void hobot_lpwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	struct hobot_lpwm_chip *lpwm = to_hobot_lpwm_chip(chip);

	val = hobot_lpwm_rd(lpwm, LPWM_EN);
	val &= (~(1 << pwm->hwpwm));

	hobot_lpwm_wr(lpwm, LPWM_EN, val);

	return;
}

static const struct pwm_ops hobot_lpwm_ops = {
	.config  = hobot_lpwm_config,
	.enable  = hobot_lpwm_enable,
	.disable = hobot_lpwm_disable,
	.owner   = THIS_MODULE,
};

static ssize_t lpwm_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hobot_lpwm_chip *lpwm = dev_get_drvdata(dev);
	int i;
	size_t len = 0;

	for (i = 0; i < LPWM_NPWM; i++)
		len += snprintf(buf+len, 32, "%d:%dus\n", i, lpwm->offset[i]);

	return len;
}

static ssize_t lpwm_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct hobot_lpwm_chip *lpwm = dev_get_drvdata(dev);
	int offset_us;
	int val;
	char *p = (char *)buf;
	char *token;
	int i;

	count = count > 64 ? 64 : count;

	for (i = 0; i < LPWM_NPWM; i++) {
		token = strsep(&p, " ");
		sscanf(token, "%d", &offset_us);
		if (offset_us < 250 || offset_us > 1000000) {
			pr_info("lpwm offset should be in [250, 1000000] microseconds\n");
			return -EINVAL;
		}

		lpwm->offset[i] = offset_us;
	}


	for (i = 0; i < LPWM_NPWM; i++) {
		val = hobot_lpwm_rd(lpwm, (i * 0x4) + LPWM0_CFG);
		val &= 0xFFFFF000;
		val |= lpwm->offset[i] / 250 - 1;
		hobot_lpwm_wr(lpwm, (i * 0x4) + LPWM0_CFG, val);
	}

	return count;
}
static DEVICE_ATTR_RW(lpwm_offset);

static int hobot_lpwm_probe(struct platform_device *pdev)
{
	int ret;
	struct hobot_lpwm_chip *lpwm;
	struct resource *res;

	lpwm = devm_kzalloc(&pdev->dev, sizeof(struct hobot_lpwm_chip), GFP_KERNEL);
	if (!lpwm)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lpwm->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(lpwm->base))
		return PTR_ERR(lpwm->base);

	snprintf(lpwm->name, sizeof(lpwm->name), "%s", LPWM_NAME);

	lpwm->clk = devm_clk_get(&pdev->dev, "lpwm_mclk");
	if (IS_ERR(lpwm->clk)) {
		pr_err("lpwm_mclk clock not found.\n");
		return PTR_ERR(lpwm->clk);
	}

	ret = clk_prepare_enable(lpwm->clk);
	if (ret) {
		pr_err("failed to enable lpwm_mclk clock\n");
		return ret;
	}

	lpwm->chip.dev  = &pdev->dev;
	lpwm->chip.ops  = &hobot_lpwm_ops;
	lpwm->chip.npwm = LPWM_NPWM;
	lpwm->chip.base = -1;

	ret = pwmchip_add(&lpwm->chip);
	if (ret < 0) {
		pr_err("failed to add LPWM chip, error %d\n", ret);
		return ret;
	}

	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_lpwm_offset.attr)) {
		pr_err("lpwm_offset create failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, lpwm);


	/* reset all regs, use sw_trigger by default */
	hobot_lpwm_wr(lpwm, LPWM_RST, 1);

	pr_info("%s registered\n", lpwm->name);

	return 0;
}

static int hobot_lpwm_remove(struct platform_device *pdev)
{
	struct hobot_lpwm_chip *lpwm = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < lpwm->chip.npwm; i++)
		pwm_disable(&lpwm->chip.pwms[i]);

	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_lpwm_offset.attr);

	return pwmchip_remove(&lpwm->chip);
}

static const struct of_device_id hobot_lpwm_dt_ids[] = {
	{ .compatible = "hobot,hobot-lpwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hobot_lpwm_dt_ids);

static struct platform_driver hobot_lpwm_driver = {
	.driver = {
		.name = "hobot-lpwm",
		.of_match_table = hobot_lpwm_dt_ids,
	},
	.probe = hobot_lpwm_probe,
	.remove = hobot_lpwm_remove,
};
module_platform_driver(hobot_lpwm_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("HOBOT LPWM driver");
MODULE_LICENSE("GPL v2");
