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
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>

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
#define LPWM_PPS  4
#define LPWM_NPIN 5

unsigned int swtrig_period = 0;
module_param(swtrig_period, uint, 0644);

struct hobot_lpwm_chip {
	struct pwm_chip chip;
	int irq;
	char name[8];
	struct clk *clk;
	void __iomem *base;
	int offset[LPWM_NPWM];
	struct hrtimer swtrig_timer;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins[LPWM_NPIN];
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
	int ret;

	if (period_ns > 0 && (period_ns < 10000 || period_ns > 40960000)) {
		pr_err("lpwm only support period in [10000ns~40960000ns]\n");
		return -ERANGE;
	}

	if (duty_ns > 0 && (duty_ns < 10000 || duty_ns > 160000)) {
		pr_err("lpwm only support duty_cycle in [10000ns~160000ns]\n");
		return -ERANGE;
	}

	if (duty_ns >= period_ns)
		duty_ns = period_ns - 1;

	/* config pwm freq */
	period = div64_u64((uint64_t)period_ns, (uint64_t)10000) - 1;
	high = div64_u64((uint64_t)duty_ns, (uint64_t)10000) - 1;

	ret = clk_prepare_enable(lpwm->clk);
	if (ret) {
		pr_err("failed to enable lpwm_mclk clock\n");
		return ret;
	}

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

static enum hrtimer_restart swtig_timer_func(struct hrtimer *hrt)
{
	struct hobot_lpwm_chip *lpwm =
		container_of(hrt, struct hobot_lpwm_chip, swtrig_timer);

	hobot_lpwm_wr(lpwm, LPWM_SW_TRIG, 1);

	hrtimer_forward_now(hrt, ms_to_ktime(swtrig_period));

	pr_debug("swtrig_period %d\n", swtrig_period);
	return HRTIMER_RESTART;
}

static int hobot_lpwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct hobot_lpwm_chip *lpwm = to_hobot_lpwm_chip(chip);
	u32 val;
	int ret;

	if (!__clk_is_enabled(lpwm->clk)) {
		ret = clk_prepare_enable(lpwm->clk);
		if (ret) {
			pr_err("failed to enable lpwm_mclk clock\n");
			return ret;
		}
	}

	if (lpwm->pinctrl != NULL && lpwm->pins[pwm->hwpwm] != NULL)
		pinctrl_select_state(lpwm->pinctrl, lpwm->pins[pwm->hwpwm]);

	val = hobot_lpwm_rd(lpwm, LPWM_EN);
	val |= (1 << pwm->hwpwm);
	hobot_lpwm_wr(lpwm, LPWM_EN, val);

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

	if (hrtimer_active(&lpwm->swtrig_timer))
		hrtimer_cancel(&lpwm->swtrig_timer);
	hobot_lpwm_wr(lpwm, LPWM_EN, val);

	if (__clk_is_enabled(lpwm->clk))
		clk_disable_unprepare(lpwm->clk);

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
		if (offset_us < 10 || offset_us > 40960) {
			pr_info("lpwm offset should be in [10, 40960] microseconds\n");
			return -EINVAL;
		}

		lpwm->offset[i] = offset_us;
	}


	for (i = 0; i < LPWM_NPWM; i++) {
		val = hobot_lpwm_rd(lpwm, (i * 0x4) + LPWM0_CFG);
		val &= 0xFFFFF000;
		val |= lpwm->offset[i] / 10 - 1;
		hobot_lpwm_wr(lpwm, (i * 0x4) + LPWM0_CFG, val);
	}

	return count;
}
static DEVICE_ATTR_RW(lpwm_offset);


static ssize_t lpwm_swtrig_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct hobot_lpwm_chip *lpwm = dev_get_drvdata(dev);
	int val;

	sscanf(buf, "%d", &val);
	pr_info("trigger lpwms, val:%d\n", val);
	val = (val == 0) ? 1 : val;

	if (swtrig_period) {
		hrtimer_start(&lpwm->swtrig_timer, ms_to_ktime(swtrig_period), HRTIMER_MODE_REL);
	}
	hobot_lpwm_wr(lpwm, LPWM_SW_TRIG, val);
	return count;
}
static DEVICE_ATTR_WO(lpwm_swtrig);

static ssize_t lpwm_ppstrig_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct hobot_lpwm_chip *lpwm = dev_get_drvdata(dev);
	int val;

	sscanf(buf, "%d", &val);
	pr_info("pps trigger lpwms, val:%d\n", val);

	if (val == 0) {
		val = hobot_lpwm_rd(lpwm, LPWM_EN);
		val &= ~(LPWM_MODE_PPS_TRIG);
	} else {
		if (lpwm->pinctrl != NULL && lpwm->pins[LPWM_PPS] != NULL)
			pinctrl_select_state(lpwm->pinctrl, lpwm->pins[LPWM_PPS]);
		udelay(100);
		val = hobot_lpwm_rd(lpwm, LPWM_EN);
		val |= LPWM_MODE_PPS_TRIG;
	}
	hobot_lpwm_wr(lpwm, LPWM_EN, val);

	return count;
}
static DEVICE_ATTR_WO(lpwm_ppstrig);


static int hobot_lpwm_probe(struct platform_device *pdev)
{
	int ret;
	struct hobot_lpwm_chip *lpwm;
	struct resource *res;
	int i;
	char buf[16];

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

	lpwm->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lpwm->pinctrl)) {
		pr_err("Failed to get a pinctrl state holder, check dts.\n");
		return -ENODEV;
	}

	hrtimer_init(&lpwm->swtrig_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL | HRTIMER_MODE_PINNED);
	lpwm->swtrig_timer.function = swtig_timer_func;
	for (i = 0; i < LPWM_NPWM; i++) {
		memset(buf, 0, sizeof(buf));
		snprintf(buf, sizeof(buf), "lpwm%d", i);

		lpwm->pins[i] = pinctrl_lookup_state(lpwm->pinctrl, buf);
		if (lpwm->pins[i] == NULL) {
			pr_err("lpwm%d pinctrl is not found, check dts.\n", i);
			return -ENODEV;
		}
	}
	lpwm->pins[LPWM_PPS] = pinctrl_lookup_state(lpwm->pinctrl, "lpwm_pps");
	if (lpwm->pins[LPWM_PPS] == NULL) {
		pr_err("lpwm_pps pinctrl is not found, check dts.\n");
		return -ENODEV;
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

	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_lpwm_swtrig.attr)) {
		pr_err("lpwm_swtrig create failed\n");
		return -ENOMEM;
	}

	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_lpwm_ppstrig.attr)) {
		pr_err("lpwm_ppstrig create failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, lpwm);


	/* reset all regs, use sw_trigger by default */
	hobot_lpwm_wr(lpwm, LPWM_RST, 1);

	clk_disable_unprepare(lpwm->clk);

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
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_lpwm_swtrig.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_lpwm_ppstrig.attr);
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
