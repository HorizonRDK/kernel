/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/of.h>
#include <linux/spinlock.h>

#define X2_MAX_NR_RESETS        64

#define RESET_REG_OFFSET_SHIFT  8
#define RESET_REG_OFFSET_MASK   0xff00
#define RESET_REG_BIT_MASK      0x1f

struct x2_reset_data {
        spinlock_t      lock;
        void __iomem    *membase;
        struct reset_controller_dev rcdev;
};

#define to_x2_reset_data(p)     \
        container_of((p), struct x2_reset_data, rcdev)

static int x2_reset_assert(struct reset_controller_dev *rcdev,
        unsigned long id)
{
        void __iomem    *regaddr;
        uint32_t reg_val, offset;
        unsigned long flags;
        u8 bit;
        struct x2_reset_data *data = to_x2_reset_data(rcdev);

        if (rcdev == NULL || id < 0)
                return -EINVAL;

        spin_lock_irqsave(&data->lock, flags);
        offset = (id & RESET_REG_OFFSET_MASK) >> RESET_REG_OFFSET_SHIFT;
        regaddr = data->membase + offset;

        reg_val = readl(regaddr);
        bit = (id & RESET_REG_BIT_MASK);
        reg_val |= BIT(bit);
        writel(reg_val, regaddr);

        spin_unlock_irqrestore(&data->lock, flags);

        return 0;
}

static int x2_reset_deassert(struct reset_controller_dev *rcdev,
        unsigned long id)
{
        void __iomem    *regaddr;
        uint32_t reg_val, offset;
        unsigned long flags;
        u8 bit;
        struct x2_reset_data *data = to_x2_reset_data(rcdev);

        if (rcdev == NULL || id < 0)
                return -EINVAL;

        spin_lock_irqsave(&data->lock, flags);
        offset = (id & RESET_REG_OFFSET_MASK) >> RESET_REG_OFFSET_SHIFT;
        regaddr = data->membase + offset;

        reg_val = readl(regaddr);
        bit = (id & RESET_REG_BIT_MASK);
        reg_val &= ~(BIT(bit));
        writel(reg_val, regaddr);

        spin_unlock_irqrestore(&data->lock, flags);
        return 0;
}

static int x2_reset_status(struct reset_controller_dev *rcdev,
        unsigned long id)
{
        return 0;
}

static const struct reset_control_ops x2_reset_ops = {
        .assert     = x2_reset_assert,
        .deassert   = x2_reset_deassert,
        .status     = x2_reset_status,
};

static int x2_reset_of_xlate(struct reset_controller_dev *rcdev,
            const struct of_phandle_args *reset_spec)
{
    u32 offset;
    u8 bit;

    if(reset_spec->args[0] > 0x50 || reset_spec->args[1] > 31)
        return -EINVAL;

    offset = (reset_spec->args[0] << RESET_REG_OFFSET_SHIFT) & RESET_REG_OFFSET_MASK;
    bit = reset_spec->args[1] & RESET_REG_BIT_MASK;

    return (offset | bit);
}

static int x2_reset_probe(struct platform_device *pdev)
{
        struct x2_reset_data *data;
        struct resource *res;
        struct device *dev = &pdev->dev;
        struct device_node *np = dev->of_node;
        u32 modrst_offset;

        /*
         * The binding was mainlined without the required property.
         * Do not continue, when we encounter an old DT.
         */
        if (!of_find_property(pdev->dev.of_node, "#reset-cells", NULL)) {
                dev_err(&pdev->dev, "%s missing #reset-cells property\n",
                        pdev->dev.of_node->full_name);
                return -EINVAL;
        }

        data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
        if (!data)
                return -ENOMEM;

        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        data->membase = devm_ioremap_resource(&pdev->dev, res);
        if (IS_ERR(data->membase))
                return PTR_ERR(data->membase);

        if (of_property_read_u32(np, "x2,modrst-offset", &modrst_offset)) {
                dev_warn(dev, "missing x2,modrst-offset property, assuming 0x400!\n");
                modrst_offset = 0x400;
        }
        data->membase += modrst_offset;

        spin_lock_init(&data->lock);

        data->rcdev.owner = THIS_MODULE;
        data->rcdev.nr_resets = X2_MAX_NR_RESETS;
        data->rcdev.ops = &x2_reset_ops;
        data->rcdev.of_node = pdev->dev.of_node;
        data->rcdev.of_xlate = x2_reset_of_xlate;
        data->rcdev.of_reset_n_cells = 2;

        return devm_reset_controller_register(dev, &data->rcdev);

}
static const struct of_device_id x2_reset_dt_ids[] = {
        { .compatible = "hobot,x2-reset", },
        { },
};

static struct platform_driver x2_reset_driver = {
        .probe  = x2_reset_probe,
        .driver = {
                .name       = KBUILD_MODNAME,
                .of_match_table = x2_reset_dt_ids,
        },
};

static int __init x2_reset_init(void)
{
    return platform_driver_register(&x2_reset_driver);
}

arch_initcall(x2_reset_init);
