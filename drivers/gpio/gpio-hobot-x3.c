/*
 * Hobot pin controller
 *
 *	Copyright (C) 2018 Hobot
 * *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/bitops.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>

#define X3_NUM_IOS 121

/* GPIO register offsets from  base address */
#define X3_IO_CTL 0x8
#define X3_IO_IN_VALUE 0xc
#define X3_IO_DIR_SHIFT 16

/* gpio_irq register offsets from bank base address */
#define X3_IO_INT_SEL 0x100
#define X3_IO_INT_EN 0x104
#define X3_IO_INT_POS 0x108
#define X3_IO_INT_NEG 0x10c
#define X3_IO_INT_WIDTH 0x110
#define X3_IO_INT_MASK 0x120
#define X3_IO_INT_SETMASK 0x124
#define X3_IO_INT_UNMASK 0x128
#define X3_IO_INT_SRCPND 0x12c

/* each bank has 16 gpio */
#define X3_GPIO_NR 16

#define X3_IO_MIN 0
#define X3_IO_MAX 120

/*This for gpio to judge.*/
#define X3_IO_MINN 1
#define X3_IO_MAXX 121

#define GROUP0_MAX 15
#define GROUP1_MAX 31
#define GROUP2_MAX 47
#define GROUP3_MAX 63
#define GROUP4_MAX 79
#define GROUP5_MAX 95
#define GROUP6_MAX 111
#define GROUP7_MAX 120

#define GPIO_IRQ_NO_BIND (-1)

enum { GPIO_LOW = 0, GPIO_HIGH = 1 };

enum {
    GPIO_IRQ_BANK0,
    GPIO_IRQ_BANK1,
    GPIO_IRQ_BANK2,
    GPIO_IRQ_BANK3,
    GPIO_IRQ_BANK_NUM,
};

struct __io_group {
    unsigned int start;
    unsigned int end;
    unsigned int regoffset;
};

struct x3_gpio {
    raw_spinlock_t lock;
    struct device *dev;
    struct gpio_chip *chip;
    struct irq_domain *domain;
    void __iomem *regbase;
    int irq;
    int irqbase;
    DECLARE_BITMAP(gpio_irq_enabled_mask, X3_NUM_IOS);
    unsigned long irqbind[GPIO_IRQ_BANK_NUM];
};

static struct __io_group io_groups[] = {
    [0] =
        {
            .start = 0,  // X3_IO_MIN,  special case, gpio0 is remove but bit is
                         // remain
            .end = GROUP0_MAX,
            .regoffset = 0x0,
        },
    [1] =
        {
            .start = GROUP0_MAX + 1,
            .end = GROUP1_MAX,
            .regoffset = 0x10,
        },
    [2] =
        {
            .start = GROUP1_MAX + 1,
            .end = GROUP2_MAX,
            .regoffset = 0x20,
        },
    [3] =
        {
            .start = GROUP2_MAX + 1,
            .end = GROUP3_MAX,
            .regoffset = 0x30,
        },
    [4] =
        {
            .start = GROUP3_MAX + 1,
            .end = GROUP4_MAX,
            .regoffset = 0x40,
        },
    [5] =
        {
            .start = GROUP4_MAX + 1,
            .end = GROUP5_MAX,
            .regoffset = 0x50,
        },
    [6] =
        {
            .start = GROUP5_MAX + 1,
            .end = GROUP6_MAX,
            .regoffset = 0x60,
        },
    [7] =
        {
            .start = GROUP6_MAX + 1,
            .end = GROUP7_MAX,
            .regoffset = 0x70,
        },
};

unsigned int find_io_group_index(unsigned int io) {
    if (io < X3_IO_MIN || io > X3_IO_MAX)
        return -1;

    if (io / (GROUP3_MAX + 1)) {
        if (io / (GROUP5_MAX + 1)) {
            if (io / (GROUP6_MAX + 1))
                return 7;
            else
                return 6;
        } else {
            if (io / (GROUP4_MAX + 1))
                return 5;
            else
                return 4;
        }
    } else {
        if (io / (GROUP1_MAX + 1)) {
            if (io / (GROUP2_MAX + 1))
                return 3;
            else
                return 2;
        } else {
            if (io / (GROUP0_MAX + 1))
                return 1;
            else
                return 0;
        }
    }
}

int find_irqbank(struct x3_gpio *gpo, unsigned long gpio) {
    int i = 0;

    for (i = 0; i < GPIO_IRQ_BANK_NUM; i++) {
        if (gpo->irqbind[i] == gpio)
            return i;
    }
    return GPIO_IRQ_NO_BIND;
}

void init_irqbank(struct x3_gpio *gpo) {
    int i = 0;

    for (i = 0; i < GPIO_IRQ_BANK_NUM; i++) {
        gpo->irqbind[i] = GPIO_IRQ_NO_BIND;
    }
}

int request_irqbank(struct x3_gpio *gpo, unsigned long gpio) {
    int i = 0, index = GPIO_IRQ_NO_BIND;

    index = find_irqbank(gpo, gpio);
    if (index == GPIO_IRQ_NO_BIND) {
        for (i = 0; i < GPIO_IRQ_BANK_NUM; i++) {
            if (gpo->irqbind[i] == GPIO_IRQ_NO_BIND) {
                gpo->irqbind[i] = gpio;
                index = i;
                break;
            }
        }
    } else {
        dev_err(gpo->dev, "gpio(%ld) has be binded\n", gpio);
        return GPIO_IRQ_NO_BIND;
    }
    return index;
}

void release_irqbank(struct x3_gpio *gpo, unsigned long gpio) {
    int index = GPIO_IRQ_NO_BIND;

    index = find_irqbank(gpo, gpio);
    if (index != GPIO_IRQ_NO_BIND) {
        gpo->irqbind[index] = GPIO_IRQ_NO_BIND;
    }
}

static int x3_gpio_get(struct gpio_chip *chip, unsigned offset) {
    u32 value;
    int index;
    void __iomem *regaddr;
    unsigned int gpio = chip->base + offset;
    struct x3_gpio *gpo = gpiochip_get_data(chip);

    index = find_io_group_index(gpio);
    if (index < 0)
        return -1;
    regaddr = gpo->regbase + io_groups[index].regoffset;

    value = readl(regaddr + X3_IO_IN_VALUE);
    value &= (0x1 << (gpio - io_groups[index].start));
    if (value)
        return 1;
    return 0;
}

static void x3_gpio_set(struct gpio_chip *chip, unsigned offset, int val) {
    u32 value;
    int index;
    void __iomem *regaddr;
    unsigned int gpio = chip->base + offset;
    struct x3_gpio *gpo = gpiochip_get_data(chip);

    index = find_io_group_index(gpio);
    if (index < 0)
        return;
    regaddr = gpo->regbase + io_groups[index].regoffset;
    value = readl(regaddr + X3_IO_CTL);
    if (val == GPIO_LOW)
        value &= ~(0x1 << (gpio - io_groups[index].start));
    else if (val == GPIO_HIGH)
        value |= (0x1 << (gpio - io_groups[index].start));
    writel(value, regaddr + X3_IO_CTL);
}

static int x3_gpio_direction_input(struct gpio_chip *chip, unsigned offset) {
    u32 value;
    int index;
    void __iomem *regaddr;
    unsigned int gpio = chip->base + offset;
    struct x3_gpio *gpo = gpiochip_get_data(chip);

    index = find_io_group_index(gpio);
    if (index < 0)
        return -ENODEV;
    regaddr = gpo->regbase + io_groups[index].regoffset;
    value = readl(regaddr + X3_IO_CTL);
    value &= ~(0x1 << (gpio - io_groups[index].start + X3_IO_DIR_SHIFT));
    writel(value, regaddr + X3_IO_CTL);

    return 0;
}

static int x3_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
                                    int val) {
    u32 value;
    int index;
    void __iomem *regaddr;
    unsigned int gpio = chip->base + offset;
    struct x3_gpio *gpo = gpiochip_get_data(chip);

    index = find_io_group_index(gpio);
    if (index < 0)
        return -ENODEV;
    regaddr = gpo->regbase + io_groups[index].regoffset;
    value = readl(regaddr + X3_IO_CTL);
    value |= (0x1 << (gpio - io_groups[index].start + X3_IO_DIR_SHIFT));

    if (val == GPIO_LOW)
        value &= ~(0x1 << (gpio - io_groups[index].start));
    else if (val == GPIO_HIGH)
        value |= (0x1 << (gpio - io_groups[index].start));

    writel(value, regaddr + X3_IO_CTL);

    return 0;
}

static int x3_gpio_to_irq(struct gpio_chip *chip, unsigned offset) { return 0; }

static int x3_gpio_request(struct gpio_chip *chip, unsigned int offset) {
    int ret;
    struct x3_gpio *gpo = gpiochip_get_data(chip);

    ret = pinctrl_request_gpio(chip->base + offset);
    if (ret)
        return ret;
    dev_dbg(gpo->dev, "%s(%d)\n", __func__, __LINE__);
    // x2_pinctrl_fsel_set(pctrl, chip->base + offset, 0x3);

    return 0;
}
static void x3_gpio_free(struct gpio_chip *chip, unsigned int offset) {
    pinctrl_free_gpio(chip->base + offset);
}

static struct gpio_chip x3_gpio_chip = {
    .base = X3_IO_MIN,
    .ngpio = X3_IO_MAX - X3_IO_MIN + 1,
    .direction_input = x3_gpio_direction_input,
    .direction_output = x3_gpio_direction_output,
    .get = x3_gpio_get,
    .set = x3_gpio_set,
    .to_irq = x3_gpio_to_irq,
    .request = x3_gpio_request,
    .free = x3_gpio_free,
};

/* IRQ chip handlers */

static void x3_gpio_irq_enable(struct irq_data *data) {
    unsigned long flags;
    u32 value;
    int bank;
    void __iomem *regaddr;
    struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
    struct x3_gpio *gpo = gpiochip_get_data(chip);
    unsigned gpio = irqd_to_hwirq(data) + chip->base;

    dev_dbg(gpo->dev, "%s: gpio(%d)", __func__, gpio);
    set_bit(gpio, gpo->gpio_irq_enabled_mask);
    bank = find_irqbank(gpo, gpio);
    if (bank < GPIO_IRQ_BANK0 || bank > GPIO_IRQ_BANK3)
        return;
    regaddr = gpo->regbase + X3_IO_INT_EN;

    raw_spin_lock_irqsave(&gpo->lock, flags);

    value = readl(regaddr);
    value |= 0x1 << bank;
    writel(value, regaddr);

    raw_spin_unlock_irqrestore(&gpo->lock, flags);
}

static void x3_gpio_irq_disable(struct irq_data *data) {
    unsigned long flags;
    u32 value;
    int bank;
    void __iomem *regaddr;
    struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
    struct x3_gpio *gpo = gpiochip_get_data(chip);
    unsigned gpio = irqd_to_hwirq(data) + chip->base;

    clear_bit(gpio, gpo->gpio_irq_enabled_mask);
    bank = find_irqbank(gpo, gpio);
    if (bank < GPIO_IRQ_BANK0 || bank > GPIO_IRQ_BANK3)
        return;
    raw_spin_lock_irqsave(&gpo->lock, flags);

    regaddr = gpo->regbase + X3_IO_INT_EN;
    value = readl(regaddr);
    value &= ~(0x1 << bank);
    writel(value, regaddr);

    raw_spin_unlock_irqrestore(&gpo->lock, flags);
}

static void x3_gpio_irq_unmask(struct irq_data *data) {
    unsigned long flags;
    u32 value;
    int bank;
    struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
    struct x3_gpio *gpo = gpiochip_get_data(chip);
    unsigned gpio = irqd_to_hwirq(data) + chip->base;

    bank = find_irqbank(gpo, gpio);
    if (bank < GPIO_IRQ_BANK0 || bank > GPIO_IRQ_BANK3)
        return;
    raw_spin_lock_irqsave(&gpo->lock, flags);

    value = readl(gpo->regbase + X3_IO_INT_UNMASK);
    value |= 0x1 << bank;
    writel(value, gpo->regbase + X3_IO_INT_UNMASK);

    raw_spin_unlock_irqrestore(&gpo->lock, flags);
}

static void x3_gpio_irq_mask(struct irq_data *data) {
    unsigned long flags;
    u32 value;
    int bank;
    struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
    struct x3_gpio *gpo = gpiochip_get_data(chip);
    unsigned gpio = irqd_to_hwirq(data) + chip->base;

    bank = find_irqbank(gpo, gpio);
    if (bank < GPIO_IRQ_BANK0 || bank > GPIO_IRQ_BANK3)
        return;
    raw_spin_lock_irqsave(&gpo->lock, flags);

    value = readl(gpo->regbase + X3_IO_INT_SETMASK);
    value |= 0x1 << bank;
    writel(value, gpo->regbase + X3_IO_INT_SETMASK);

    raw_spin_unlock_irqrestore(&gpo->lock, flags);
}

static int x3_gpio_irq_set_type(struct irq_data *data, unsigned int type) {
    unsigned long flags;
    u32 value1, value2, gpio;
    int bank;
    struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
    struct x3_gpio *gpo = gpiochip_get_data(chip);

    gpio = irqd_to_hwirq(data) + chip->base;
    bank = find_irqbank(gpo, gpio);
    if (bank < GPIO_IRQ_BANK0 || bank > GPIO_IRQ_BANK3)
        return -EINVAL;
    value1 = readl(gpo->regbase + X3_IO_INT_POS);
    value2 = readl(gpo->regbase + X3_IO_INT_NEG);
    switch (type) {
        case IRQ_TYPE_EDGE_RISING:
            value1 |= BIT(bank);
            value2 &= ~BIT(bank);
            break;
        case IRQ_TYPE_EDGE_FALLING:
            value1 &= ~BIT(bank);
            value2 |= BIT(bank);
            break;
        case IRQ_TYPE_EDGE_BOTH:
            value1 |= BIT(bank);
            value2 |= BIT(bank);
            break;
        default:
            return -EINVAL;
    }
    raw_spin_lock_irqsave(&gpo->lock, flags);

    writel(value1, gpo->regbase + X3_IO_INT_POS);
    writel(value2, gpo->regbase + X3_IO_INT_NEG);

    raw_spin_unlock_irqrestore(&gpo->lock, flags);

    return 0;
}

static int x3_gpio_irq_reqres(struct irq_data *data) {
    unsigned long gpio_num, value, flags;
    int bank;
    struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
    struct x3_gpio *gpo = gpiochip_get_data(chip);

    if (gpiochip_lock_as_irq(chip, data->hwirq)) {
            dev_err(gpo->dev, "unable to lock HW IRQ %lu for IRQ\n",
                    data->hwirq);
            return -EINVAL;
    }

    gpio_num = irqd_to_hwirq(data) + chip->base;
    dev_dbg(gpo->dev, "start to bind gpio_irq(%ld)\n", gpio_num);
    bank = request_irqbank(gpo, gpio_num);
    if (bank == GPIO_IRQ_NO_BIND) {
        dev_err(gpo->dev, "gpio(%ld) irq bind failed\n", gpio_num);
        return -EINVAL;
    }

    raw_spin_lock_irqsave(&gpo->lock, flags);
    value = readl(gpo->regbase + X3_IO_INT_SEL);
    value &= ~(0xff << bank * 8);
    value |= (gpio_num << bank * 8);
    writel(value, gpo->regbase + X3_IO_INT_SEL);
    raw_spin_unlock_irqrestore(&gpo->lock, flags);

    dev_dbg(gpo->dev, "gpio(%ld) irq bind to bank(%d) ok\n", gpio_num, bank);
    return 0;
}

static void x3_gpio_irq_relres(struct irq_data *data) {
    unsigned long gpio_num, value, flags;
    int bank;
    struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
    struct x3_gpio *gpo = gpiochip_get_data(chip);

    gpio_num = irqd_to_hwirq(data) + chip->base;
    bank = find_irqbank(gpo, gpio_num);
    if (bank == GPIO_IRQ_NO_BIND) {
        return;
    }
    release_irqbank(gpo, gpio_num);

    raw_spin_lock_irqsave(&gpo->lock, flags);
    value = readl(gpo->regbase + X3_IO_INT_SEL);
    value &= ~(0xff << bank * 8);
    writel(value, gpo->regbase + X3_IO_INT_SEL);
    raw_spin_unlock_irqrestore(&gpo->lock, flags);

    gpiochip_unlock_as_irq(chip, data->hwirq);
    dev_dbg(gpo->dev, "gpio(%ld) irq binded on bank(%d) release ok\n", gpio_num,
           bank);
}

static void x3_gpio_generic_handler(struct irq_desc *desc) {
    u32 value, gpio, i;
    void __iomem *regaddr;
    struct gpio_chip *chip = irq_desc_get_handler_data(desc);
    struct irq_chip *irqchip = irq_desc_get_chip(desc);
    struct x3_gpio *gpo = gpiochip_get_data(chip);

    dev_dbg(gpo->dev, "%s: irq(%d), hwirq(%ld)", __func__, desc->irq_data.irq,
            desc->irq_data.hwirq);
    chained_irq_enter(irqchip, desc);

    regaddr = gpo->regbase + X3_IO_INT_SRCPND;
    value = readl(regaddr);
    for (i = 0; i < GPIO_IRQ_BANK_NUM; i++) {
        gpio = gpo->irqbind[i];
        if ((value & BIT(i)) && gpio &&
            test_bit(gpio, gpo->gpio_irq_enabled_mask)) {
            generic_handle_irq(gpio_to_irq(gpio));
        }
    }
    writel(value, regaddr);
    chained_irq_exit(irqchip, desc);
}

static struct irq_chip x3_gpio_irq_chip = {
    .name = "x3-gpio",
    .irq_enable = x3_gpio_irq_enable,
    .irq_disable = x3_gpio_irq_disable,
    .irq_set_type = x3_gpio_irq_set_type,
    .irq_ack = NULL,
    .irq_mask = x3_gpio_irq_mask,
    .irq_unmask = x3_gpio_irq_unmask,
    .irq_request_resources = x3_gpio_irq_reqres,
    .irq_release_resources = x3_gpio_irq_relres,
};

static int x3_gpio_probe(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    struct x3_gpio *gpo;
    struct resource *res, *irqs;
    int ret;

    dev_info(dev, "x3 gpio probe start!!!\n");
    gpo = devm_kzalloc(dev, sizeof(*gpo), GFP_KERNEL);
    if (!gpo)
        return -ENOMEM;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    gpo->regbase = devm_ioremap_resource(dev, res);
    if (IS_ERR(gpo->regbase))
        return PTR_ERR(gpo->regbase);

    init_irqbank(gpo);
    bitmap_zero(gpo->gpio_irq_enabled_mask, X3_NUM_IOS);
    gpo->dev = dev;
    gpo->chip = &x3_gpio_chip;
    gpo->chip->parent = dev;
    gpo->chip->of_node = dev->of_node;

    raw_spin_lock_init(&gpo->lock);

    dev_info(dev, "x3 gpio add gpiochip start!!!\n");
    ret = gpiochip_add_data(gpo->chip, gpo);
    if (ret) {
        dev_err(dev, "could not add GPIO chip\n");
        return ret;
    }
    /*
     * irq_chip support
     */
    /*
     * Initialise all interrupts to disabled so we don't get
     * spurious ones on a dirty boot and hit the BUG_ON in the
     * handler.
     */
    dev_info(dev, "diable all gpio irq\n");
    writel(0x0, gpo->regbase + X3_IO_INT_SETMASK);
    writel(0xf, gpo->regbase + X3_IO_INT_UNMASK);
    writel(0x0, gpo->regbase + X3_IO_INT_EN);

    platform_set_drvdata(pdev, gpo);
    gpo->irqbase =
        devm_irq_alloc_descs(gpo->dev, -1, 0, gpo->chip->ngpio, NUMA_NO_NODE);
    dev_info(gpo->dev, "irqbase:%d", gpo->irqbase);
    if (gpo->irqbase < 0) {
        dev_err(gpo->dev, "Faild to allocate IRQ numbers\n");
        return -ENODEV;
    }
    dev_info(dev, "x3 irq add start!!!\n");
    ret = gpiochip_irqchip_add(gpo->chip, &x3_gpio_irq_chip, gpo->irqbase,
                               handle_level_irq, IRQ_TYPE_NONE);
    if (ret) {
        dev_err(dev, "could not add irqchip\n");
        gpiochip_remove(gpo->chip);
        return ret;
    }
    dev_info(dev, "x3 irq add ok!!!\n");

    irqs = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (irqs->start < 0) {
        dev_err(dev, "invalid IRQ\n");
        gpiochip_remove(gpo->chip);
        return -ENODEV;
    }
    gpiochip_set_chained_irqchip(gpo->chip, &x3_gpio_irq_chip, irqs->start,
                                 x3_gpio_generic_handler);
    dev_info(dev, "x3 irqchip ops add ok!!!\n");
    /*if (!of_property_read_u32_array(pdev->dev.of_node, "gpioirq-bank-cfg",
                                    gpo->irqbind, GPIO_IRQ_BANK_NUM)) {
        x3_set_gpio_irq_to_bank(gpo);
    }*/

    dev_info(dev, "x3 gpio probe ok!!!\n");

    return 0;
}

static const struct of_device_id x3_gpio_of_match[] = {
    {.compatible = "hobot,x3-gpio"}, {}};

static struct platform_driver x3_gpio_driver = {
    .driver =
        {
            .name = "gpio-x3",
            .of_match_table = x3_gpio_of_match,
        },
    .probe = x3_gpio_probe,
};

static int __init x3_gpio_init(void) {
    return platform_driver_register(&x3_gpio_driver);
}
module_init(x3_gpio_init);
