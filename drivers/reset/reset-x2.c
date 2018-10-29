#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/of.h>
#include <linux/spinlock.h>

#define X2_MAX_NR_RESETS        64

#define CPUSYS_SW_RESET_MASK    0xfff
#define CPUSYS_MODULE_ID_MASK    0x1000
#define SYS_ROMC_RST_MASK               (0x1 << 0)
#define SYS_SRAMC_RST_MASK              (0x1 << 1)
#define SYS_GIC_RST_MASK                (0x1 << 2)
#define SYS_BIFSPI_RST_MASK             (0x1 << 3)
#define SYS_BIFSPI_SHREG_RST_MASK       (0x1 << 4)
#define SYS_BIFSD_RST_MASK              (0x1 << 5)
#define SYS_QSPI_RST_MASK               (0x1 << 6)
#define SYS_DMAC_RST_MASK               (0x1 << 7)
#define SYS_TIMER0_RST_MASK             (0x1 << 8)
#define SYS_TIMER1_RST_MASK             (0x1 << 9)
#define SYS_TIMER2_RST_MASK             (0x1 << 10)
#define SYS_EFUSE_RSET_NASK             (0x1 << 11)

#define DBGSYS_SW_RESET_MASK    0x1f
#define DBGSYS_MODULE_ID_MASK    0x100
#define DBG_PLCK_RST_MASK               (0x1 << 0)
#define DBG_ATB_ATCLK_RST_MASK          (0x1 << 1)
#define DBG_ATB_TSCLK_RST_MASK          (0x1 << 2)
#define DBG_ARMV8_PIL_RST_MASK          (0x1 << 3)
#define DBG_CS_TPIU_CLKIN_RST_MASK      (0x1 << 4)

#define CNNSYS_SW_RESET_MASK    0xf
#define CNNSYS_MODULE_ID_MASK   0x10
#define CNNSYS_CNN0_RST_MASK    (0x1 << 0)
#define CNNSYS_CNN1_RST_MASK    (0x1 << 1)

#define DDRSYS_SW_RESET_MASK    0xfff
#define DDRSYS_MODULE_ID_MASK   0x2000

#define VIOSYS_SW_RESET_MASK    0xff
#define VIOSYS_MODULE_ID_MASK   0x200

#define PERISYS_SW_RESET_MASK   0xfffff
#define PERISYS_MODULE_ID_MASK  0x100000

#define CPUSYS_SW_RESET_OFFSET    0x0
#define DBGSYS_SW_RESET_OFFSET    0x10
#define CNNSYS_SW_RESET_OFFSET    0x20
#define DDRSYS_SW_RESET_OFFSET    0x30
#define VIOSYS_SW_RESET_OFFSET    0x40
#define PERISYS_SW_RESET_OFFSET   0x50

struct x2_reset_data {
        spinlock_t      lock;
        void __iomem    *membase;
        struct reset_controller_dev rcdev;
};

#define to_x2_reset_data(p)		\
        container_of((p), struct x2_reset_data, rcdev)

static int x2_reset_assert(struct reset_controller_dev *rcdev,
        unsigned long id)
{
        uint32_t reg_val;
        unsigned long flags;
        struct x2_reset_data *data = to_x2_reset_data(rcdev);

        if (rcdev == NULL || id < 0)
                return -EINVAL;

        spin_lock_irqsave(&data->lock, flags);
        if (id & CPUSYS_MODULE_ID_MASK) {
                data->membase += CPUSYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= CPUSYS_SW_RESET_MASK;
                reg_val |= id;
                writel(reg_val, data->membase);

        } else if (id & DBGSYS_MODULE_ID_MASK) {
                data->membase += DBGSYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= DBGSYS_SW_RESET_MASK;
                reg_val |= id;
                writel(reg_val, data->membase);

        } else if (id & CNNSYS_MODULE_ID_MASK) {
                data->membase += CNNSYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= CNNSYS_SW_RESET_MASK;
                reg_val |= id;
                writel(reg_val, data->membase);

        } else if (id & DDRSYS_MODULE_ID_MASK) {
                data->membase += PERISYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= DDRSYS_SW_RESET_MASK;
                reg_val |= id;
                writel(reg_val, data->membase);
        } else if (id & VIOSYS_MODULE_ID_MASK) {
                data->membase += VIOSYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= VIOSYS_SW_RESET_MASK;
                reg_val |= id;
                writel(reg_val, data->membase);

        } else if (id & PERISYS_MODULE_ID_MASK) {
                data->membase += PERISYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= PERISYS_SW_RESET_MASK;
                reg_val |= id;
                writel(reg_val, data->membase);

        } else {
                spin_unlock_irqrestore(&data->lock, flags);
                pr_err("Invalid rst module id\n");
                return -EINVAL;
        }

        spin_unlock_irqrestore(&data->lock, flags);

        return 0;
}

static int x2_reset_deassert(struct reset_controller_dev *rcdev,
        unsigned long id)
{
        uint32_t reg_val;
        unsigned long flags;
        struct x2_reset_data *data = to_x2_reset_data(rcdev);

        if (rcdev == NULL || id < 0)
                return -EINVAL;

        spin_lock_irqsave(&data->lock, flags);
        if (id & CPUSYS_MODULE_ID_MASK) {
                data->membase += CPUSYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= CPUSYS_SW_RESET_MASK;
                reg_val &= ~id;
                writel(reg_val, data->membase);

        } else if (id & DBGSYS_MODULE_ID_MASK) {
                data->membase += DBGSYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= DBGSYS_SW_RESET_MASK;
                reg_val &= ~id;
                writel(reg_val, data->membase);

        } else if (id & CNNSYS_MODULE_ID_MASK) {
                data->membase += CNNSYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= CNNSYS_SW_RESET_MASK;
                reg_val &= ~id;
                writel(reg_val, data->membase);

        } else if (id & DDRSYS_SW_RESET_MASK) {
                data->membase += PERISYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= DDRSYS_SW_RESET_MASK;
                reg_val &= ~id;
                writel(reg_val, data->membase);
        } else if (id & VIOSYS_SW_RESET_MASK) {
                data->membase += VIOSYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= VIOSYS_SW_RESET_MASK;
                reg_val &= ~id;
                writel(reg_val, data->membase);

        } else if (id & PERISYS_SW_RESET_MASK) {
                data->membase += PERISYS_SW_RESET_OFFSET;
                reg_val = readl(data->membase);

                id &= PERISYS_SW_RESET_MASK;
                reg_val &= ~id;
                writel(reg_val, data->membase);

        } else {
                spin_unlock_irqrestore(&data->lock, flags);
                pr_err("Invalid rst module id\n");
                return -EINVAL;
        }

        spin_unlock_irqrestore(&data->lock, flags);
        return 0;
}

static int x2_reset_status(struct reset_controller_dev *rcdev,
        unsigned long id)
{
        return 0;
}

static const struct reset_control_ops x2_reset_ops = {
        .assert		= x2_reset_assert,
        .deassert	= x2_reset_deassert,
        .status		= x2_reset_status,
};

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
                dev_warn(dev, "missing x2,modrst-offset property, assuming 0x4000!\n");
                modrst_offset = 0x4000;
        }
        data->membase += modrst_offset;

        spin_lock_init(&data->lock);

        data->rcdev.owner = THIS_MODULE;
        data->rcdev.nr_resets = X2_MAX_NR_RESETS;
        data->rcdev.ops = &x2_reset_ops;
        data->rcdev.of_node = pdev->dev.of_node;

        return devm_reset_controller_register(dev, &data->rcdev);

}
static const struct of_device_id x2_reset_dt_ids[] = {
        { .compatible = "hobot,x2-reset", },
        { },
};

static struct platform_driver x2_reset_driver = {
        .probe	= x2_reset_probe,
        .driver = {
                .name		= KBUILD_MODNAME,
                .of_match_table	= x2_reset_dt_ids,
        },
};

static int __init x2_reset_init(void)
{
        return platform_driver_register(&x2_reset_driver);
}

arch_initcall(x2_reset_init);
