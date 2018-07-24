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
#define X2_PWM_EN             0x00
#define X2_PWM_TIME_SLICE     0x04
#define X2_PWM_FREQ           0x08
#define X2_PWM_FREQ1          0x0C
#define X2_PWM_RATIO          0x14
#define X2_PWM_SRCPND         0x1C
#define X2_PWM_INTMASK        0x20
#define X2_PWM_SETMASK        0x24
#define X2_PWM_UNMASK         0x28

#define X2_PWM_INT_EN         (1U<<3)
#define X2_PWM_NPWM           3         /* number of channels per pwm chip(controller) */
#define X2_PWM_CLK            24000000
#define X2_PWM_NAME           "x2-pwm"

struct x2_pwm_chip {
	struct pwm_chip chip;
	int irq;
	char name[8];
	void __iomem *base;
};

#define PWM_ENABLE	BIT(31)
#define PWM_PIN_LEVEL	BIT(30)

#define to_x2_pwm_chip(_chip) \
	container_of(_chip, struct x2_pwm_chip, chip)

/* IO accessors */
static inline u32 x2_pwm_rd(struct x2_pwm_chip *x2_chip, u32 reg)
{
	return ioread32(x2_chip->base + reg);
}

static inline void x2_pwm_wr(struct x2_pwm_chip *x2_chip, u32 reg, u32 value)
{
	iowrite32(value, x2_chip->base + reg);
}

static irqreturn_t x2_pwm_irq_handler(int irq, void *data)
{
	u32 status = 0;
	struct x2_pwm_chip *x2 = (struct x2_pwm_chip *)data;

	status = x2_pwm_rd(x2, X2_PWM_SRCPND);
	x2_pwm_wr(x2, X2_PWM_SRCPND, status);
	dev_info(x2->chip.dev, "pwm_irq_handler\n");

	return IRQ_HANDLED;
}

static int x2_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm, int duty_ns, int period_ns)
{
	u32 val, reg, offset;
	int pwm_freq, pwm_ratio;
	struct x2_pwm_chip *x2 = to_x2_pwm_chip(chip);

	/* config pwm freq */
	pwm_freq = div64_u64(X2_PWM_CLK*period_ns, (unsigned long long)NSEC_PER_SEC);
	if(0xFFF < pwm_freq) {
		return -ERANGE;
	}
	reg = pwm->hwpwm > 2 ? X2_PWM_FREQ1 : X2_PWM_FREQ;
	offset = (pwm->hwpwm % 2) * 16;
	val = x2_pwm_rd(x2, reg);
	val &= ~(0xFFF<<offset);
	val |= pwm_freq<<offset;
	x2_pwm_wr(x2, reg, val);

	/* config pwm duty */
	pwm_ratio = div64_u64((unsigned long long)duty_ns * 256, period_ns);
	val = x2_pwm_rd(x2, X2_PWM_RATIO);
	offset = pwm->hwpwm * 8;
	val &= ~(0xFF<<offset);
	val |= pwm_ratio << offset;
	x2_pwm_wr(x2, X2_PWM_RATIO, val);

	return 0;
}

static int x2_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	struct x2_pwm_chip *x2 = to_x2_pwm_chip(chip);

	val = x2_pwm_rd(x2, X2_PWM_EN);
	val |= (1<<pwm->hwpwm);
	val |= X2_PWM_INT_EN;
	x2_pwm_wr(x2, X2_PWM_EN, val);

	return 0;
}

static void x2_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	struct x2_pwm_chip *x2 = to_x2_pwm_chip(chip);

	val = x2_pwm_rd(x2, X2_PWM_EN);
	val &= (~(1<<pwm->hwpwm));
	if (!(val & 0x7))
		val &= (~X2_PWM_INT_EN);
	x2_pwm_wr(x2, X2_PWM_EN, val);

	return;
}

static const struct pwm_ops x2_pwm_ops = {
	.config  = x2_pwm_config,
	.enable  = x2_pwm_enable,
	.disable = x2_pwm_disable,
	.owner   = THIS_MODULE,
};

static int x2_pwm_probe(struct platform_device *pdev)
{
	int ret, id;
	struct x2_pwm_chip *x2;
	struct resource *res;
	struct device_node *node = pdev->dev.of_node;

	x2 = devm_kzalloc(&pdev->dev, sizeof(struct x2_pwm_chip), GFP_KERNEL);
	if (!x2)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	x2->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(x2->base))
		return PTR_ERR(x2->base);

	/* Look for a serialN alias */
	id = of_alias_get_id(node, "pwm");
	if (id < 0) {
		dev_err(&pdev->dev, "Get id of pwm:%d is err!\n", id);
		id = 0;
	}
	sprintf(x2->name, "%s%d", X2_PWM_NAME, id);

	x2->irq = irq_of_parse_and_map(node, 0);
	if (0 == x2->irq) {
		dev_err(&pdev->dev, "IRQ map failed!\n");
		return -EINVAL;
	} else {
		ret = request_irq(x2->irq, x2_pwm_irq_handler, IRQF_SHARED, x2->name, x2);
		if (ret) {
			dev_err(&pdev->dev, "unable to request IRQ %d\n", x2->irq);
			return ret;
		} else {
			x2_pwm_wr(x2, X2_PWM_SETMASK, 0);
			x2_pwm_wr(x2, X2_PWM_UNMASK, 1);
		}
	}

	x2->chip.dev  = &pdev->dev;
	x2->chip.ops  = &x2_pwm_ops;
	x2->chip.npwm = X2_PWM_NPWM;
	x2->chip.base = -1;
	//x2->chip.of_xlate = of_pwm_simple_xlate;
	//x2->chip.of_pwm_n_cells = 2;

	ret = pwmchip_add(&x2->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip, error %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, x2);

	return 0;
}

static int x2_pwm_remove(struct platform_device *pdev)
{
	struct x2_pwm_chip *x2 = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < x2->chip.npwm; i++)
		pwm_disable(&x2->chip.pwms[i]);

	return pwmchip_remove(&x2->chip);
}

static const struct of_device_id x2_pwm_dt_ids[] = {
	{ .compatible = "hobot,x2-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, x2_pwm_dt_ids);

static struct platform_driver x2_pwm_driver = {
	.driver = {
		.name = "x2-pwm",
		.of_match_table = x2_pwm_dt_ids,
	},
	.probe = x2_pwm_probe,
	.remove = x2_pwm_remove,
};
module_platform_driver(x2_pwm_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("X2 PWM driver");
MODULE_LICENSE("GPL v2");
