
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define GPIO_GROUP_NUM 15

#define GOIO_CFG	0x0
#define GOIO_PE		0x4
#define GOIO_CTL	0x8
#define GOIO_IN_VALUE	0xc

#define OUT_DIR_SHIFT	16

#define X2_GPIO_MIN 1
#define X2_GPIO_MAX 118

#define GROUP0_MAX	14
#define GROUP1_MAX	30
#define GROUP2_MAX	45
#define GROUP3_MAX	61
#define GROUP4_MAX	77
#define GROUP5_MAX	93
#define GROUP6_MAX	109
#define GROUP7_MAX	118

void __iomem *pinctl_regbase;

struct x2_gpio_dev {
	struct device *dev;
	void __iomem *regbase;
};

enum
{
	GPIO_IN  = 0,
	GPIO_OUT = 1
};

enum
{
	GPIO_LOW  = 0,
	GPIO_HIGH = 1
};

struct __gpio_group
{
	unsigned int start;
	unsigned int end;
	unsigned int regoffset;
};
static struct __gpio_group gpio_groups[] = {
	[0] = {
		.start	= X2_GPIO_MIN,
		.end	= GROUP0_MAX,
		.regoffset	= 0x0,
	},
	[1] = {
		.start	= GROUP0_MAX+1,
		.end	= GROUP1_MAX,
		.regoffset	= 0x10,
	},
	[2] = {
		.start	= GROUP1_MAX+1,
		.end	= GROUP2_MAX,
		.regoffset	= 0x20,
	},
	[3] = {
		.start	= GROUP2_MAX+1,
		.end	= GROUP3_MAX,
		.regoffset	= 0x30,
	},
	[4] = {
		.start	= GROUP3_MAX+1,
		.end	= GROUP4_MAX,
		.regoffset	= 0x40,
	},
	[5] = {
		.start	= GROUP4_MAX+1,
		.end	= GROUP5_MAX,
		.regoffset	= 0x50,
	},
	[6] = {
		.start	= GROUP5_MAX+1,
		.end	= GROUP6_MAX,
		.regoffset	= 0x60,
	},
	[7] = {
		.start	= GROUP6_MAX+1,
		.end	= GROUP7_MAX,
		.regoffset	= 0x70,
	},
};
unsigned int find_gpio_group_index(unsigned int gpio)
{
	if (gpio < X2_GPIO_MIN || gpio > X2_GPIO_MAX)
		return -1;

	if (gpio/(GROUP3_MAX+1)) {
		if (gpio/(GROUP5_MAX+1)) {
			if (gpio/(GROUP6_MAX+1))
				return 7;
			else
				return 6;
		} else {
			if (gpio/(GROUP4_MAX+1))
				return 5;
			else
				return 4;
		}
	} else {
		if (gpio/(GROUP1_MAX+1)) {
			if (gpio/(GROUP2_MAX+1))
				return 3;
			else
				return 2;
		} else {
			if (gpio/(GROUP0_MAX+1))
				return 1;
			else
				return 0;
		}
	}
}

static int x2_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u32 value,ret;
	int index;
	void __iomem	*regaddr;
	unsigned int gpio=chip->base+offset;
	struct x2_gpio_dev *gpiodev = gpiochip_get_data(chip);

	index = find_gpio_group_index(gpio);
	if(index < 0)
		return -1;
	regaddr = gpiodev->regbase + gpio_groups[index].regoffset;

	value = readl(regaddr + GOIO_IN_VALUE);
	value &= (0x1 << (gpio - gpio_groups[index].start));
	if(value)
		return 1;
	return 0;
}

static void x2_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	u32 value,ret;
	int index;
	void __iomem	*regaddr;
	unsigned int gpio=chip->base+offset;
	struct x2_gpio_dev *gpiodev = gpiochip_get_data(chip);

	index = find_gpio_group_index(gpio);
	if(index < 0)
		return;
	regaddr = gpiodev->regbase + gpio_groups[index].regoffset;
	value = readl(regaddr + GOIO_CTL);
	if(val == GPIO_LOW)
		value &= ~(0x1 << (gpio - gpio_groups[index].start));
	else if (val == GPIO_HIGH)
		value |= (0x1 << (gpio - gpio_groups[index].start));

	writel(value, regaddr + GOIO_CTL);
}

static int x2_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	u32 value,ret;
	int index;
	void __iomem	*regaddr;
	unsigned int gpio=chip->base+offset;
	struct x2_gpio_dev *gpiodev = gpiochip_get_data(chip);

	index = find_gpio_group_index(gpio);
	if(index < 0)
		return;
	regaddr = gpiodev->regbase + gpio_groups[index].regoffset;
	value = readl(regaddr + GOIO_CTL);
	value &= ~(0x1 << (gpio - gpio_groups[index].start + OUT_DIR_SHIFT));
	writel(value, regaddr + GOIO_CTL);
	return 0;
}

static int x2_gpio_direction_output(struct gpio_chip *chip,
									unsigned offset,
									int val)
{
	u32 value,ret;
	int index;
	void __iomem	*regaddr;
	unsigned int gpio=chip->base+offset;
	struct x2_gpio_dev *gpiodev = gpiochip_get_data(chip);

	index = find_gpio_group_index(gpio);
	if(index < 0)
		return;
	regaddr = gpiodev->regbase + gpio_groups[index].regoffset;
	value = readl(regaddr + GOIO_CTL);
	value |= (0x1 << (gpio - gpio_groups[index].start + OUT_DIR_SHIFT));

	if(val == GPIO_LOW)
		value &= ~(0x1 << (gpio - gpio_groups[index].start));
	else if (val == GPIO_HIGH)
		value |= (0x1 << (gpio - gpio_groups[index].start));

	writel(value, regaddr + GOIO_CTL);

	return 0;
}

static int x2_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	unsigned int gpio=chip->base+offset;

	return 0;//(x2_gpio2irq(gpio));
}

static struct gpio_chip x2_gpio =
{
		.base			  = X2_GPIO_MIN,
		.ngpio			  = X2_GPIO_MAX - X2_GPIO_MIN + 1,
		.direction_input  = x2_gpio_direction_input,
		.direction_output = x2_gpio_direction_output,
		.get			  = x2_gpio_get,
		.set			  = x2_gpio_set,
		.to_irq			  = x2_gpio_to_irq,
		.request		  = NULL,
		.free			  = NULL,
};

static int __init x2_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct x2_gpio_dev *gpio_dev;
	struct resource *mem;

	x2_gpio.label = pdev->name;
	printk("x2_gpio_probe\n");
	gpio_dev = devm_kzalloc(&pdev->dev, sizeof(*gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;
	gpio_dev->dev = &pdev->dev;
	platform_set_drvdata(pdev, gpio_dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpio_dev->regbase = devm_ioremap_resource(&pdev->dev, mem);
	pinctl_regbase = gpio_dev->regbase;
	gpiochip_add_data(&x2_gpio, gpio_dev);
	if (ret < 0)
		return ret;

	return 0;
}

static int x2_gpio_remove(struct platform_device *dev)
{
	gpiochip_remove(&x2_gpio);
	return 0;
}

static const struct of_device_id x2_gpio_of_match[] = {
	{ .compatible = "hobot,x2-gpio" },
	{},
};
static struct platform_driver x2_gpio_driver = {
	.probe	= x2_gpio_probe,
	.remove = x2_gpio_remove,
	.driver		= {
		.name	= "x2_gpio",
		.of_match_table = x2_gpio_of_match,
	},
};
static int __init x2_gpio_init(void)
{
	int rc;

	rc = platform_driver_register(&x2_gpio_driver);

	if(!rc)
		pr_info("[GPIO]x2 GPIO initialized\n");

	return rc;
}

static void __exit x2_gpio_exit(void)
{
	platform_driver_unregister(&x2_gpio_driver);
}

postcore_initcall(x2_gpio_init);
module_exit(x2_gpio_exit);
