/***************************************************************************
 *   Copyright (C) 2021 by horizon.                                        *
 *   dinggao.pan@horizon.ai                                                *
 *                                                                         *
 *   Diag inject test file.                                                *
 *                                                                         *
 ***************************************************************************/
#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__
#include <asm-generic/errno-base.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <soc/hobot/diag.h>
#include "./diag_inject.h"

#define MODULE_NAME "diag_inject"
#define NODE_NAME "diag_inject_ctl"
#define FIRST_MINOR 0
#define MINOR_CNT 1

static dev_t dev;
static struct cdev diag_inject_dev;
static struct class *diag_inject_class;
/* module_sta[DIAG_SW_MAX] is the module control,
 * determine if injection is enabled */
static bool module_sta[DIAG_SW_MAX + 1U] = { false };
/* inject_vals[DIAG_SW_MAX] keeps track of the total number of injected val */
static uint32_t inject_vals[DIAG_SW_MAX + 1U] = { 0x0 };

struct diag_sw_range {
	uint32_t base;
	uint32_t ceil;
};

struct diag_sw_range diag_sw_range_tbl[ModuleIdMax] = {
	DIAG_SW_RANGE(DRV),
	DIAG_SW_RANGE(I2C),
	DIAG_SW_RANGE(VIO),
	DIAG_SW_RANGE(BPU),
	DIAG_SW_RANGE(I2S),
	DIAG_SW_RANGE(BIF),
	DIAG_SW_RANGE(ETH),
	DIAG_SW_RANGE(SPI),
	DIAG_SW_RANGE(EMMC),
	DIAG_SW_RANGE(QSPI),
	DIAG_SW_RANGE(ALU),
	DIAG_SW_RANGE(MPU),
	DIAG_SW_RANGE(UART),
};

#define DIAG_SW_INDEX_GET(module, event) \
		(diag_sw_range_tbl[module - 1U].base + event - 1U)

/* IO CTRL functions */
static void inject_enable(void)
{
    module_sta[DIAG_SW_MAX] = true;
    return;
}

static int inject_val_set(uint16_t module_id, uint16_t event,
				uint32_t value, bool if_clr)
{
	uint32_t index;

    if (module_id > ModuleIdMax) {
        pr_err("%s:%s, id: %d over the limit\n",
                __FILE__, __func__, module_id);
        return -EINVAL;
    }
    pr_debug("%s: module_id:0x%x, setting inject_val:0x%x, if_clr:0x%x\n",
                __func__, module_id, value, if_clr);
	index = DIAG_SW_INDEX_GET(module_id, event);
	if (index > DIAG_SW_MAX) {
		pr_err("module:0x%x event:0x%x invalid\n", module_id, event);
		return -EINVAL;
	}
    if (if_clr) {
        if (module_id == 0) {
            memset(module_sta, 0x0, sizeof(module_sta));
            memset(inject_vals, 0x0, sizeof(inject_vals));
            inject_vals[DIAG_SW_MAX] = 0;
        } else {
            if (module_sta[index] == true) {
                inject_vals[DIAG_SW_MAX]--;
				if ((int32_t)inject_vals[DIAG_SW_MAX] < 0) {
					inject_vals[DIAG_SW_MAX] = 0;
				}
            }
            module_sta[index] = false;
            inject_vals[index] = 0x0;
        }
    } else {
        if (module_id != 0) {
            if (module_sta[index] == false) {
                inject_vals[DIAG_SW_MAX]++;
                module_sta[index] = true;
            }
            inject_vals[index] = value;
        } else {
            pr_info("%s: module_id 0 is reserved for management\n", __func__);
            return -EINVAL;
        }
    }

    return 0;
}

static int inject_val_chk(uint16_t module_id, uint16_t event, uint32_t *out_val)
{
	uint32_t index;

    if (module_id > ModuleIdMax) {
        pr_err("%s:%s, id: %d over the limit\n",
                __FILE__, __func__, module_id);
        return -EINVAL;
    }

	if ((module_id == 0) && (event == 0)) {
		*out_val = inject_vals[DIAG_SW_MAX];
		return 0;
	}
	index = DIAG_SW_INDEX_GET(module_id, event);
	if (index > DIAG_SW_MAX) {
		pr_err("module:0x%x event:0x%x invalid\n", module_id, event);
		return -EINVAL;
	}
    *out_val = inject_vals[index];
    pr_debug("%s: module_id:0x%x, getting inject_val:0x%x\n", __func__, module_id, *out_val);
    return 0;
}

static int inject_en_chk(uint16_t module_id, uint16_t event, uint32_t *out_val)
{
	uint32_t index;

    if (module_id > ModuleIdMax) {
        pr_err("%s:%s, id: %d over the limit\n",
                __FILE__, __func__, module_id);
        return -EINVAL;
    }
	if (module_id == 0) {
		index = DIAG_SW_MAX;
	} else {
		index = DIAG_SW_INDEX_GET(module_id, event);
		if (index > DIAG_SW_MAX) {
			pr_err("module:0x%x event:0x%x invalid\n", module_id, event);
			return -EINVAL;
		}
	}
    *out_val = module_sta[index];
    pr_debug("%s: module_id:0x%x, getting module_sta:0x%x\n", __func__, module_id, *out_val);
    return 0;
}

/* Diag process functions */
static bool inject_registered(uint16_t module_id, uint16_t event)
{
	uint32_t index;
	struct diag_sw_range *range;

    if (module_id > ModuleIdMax) {
        pr_err("%s:%s, id: %d over the limit\n", __FILE__, __func__, module_id);
        return false;
    }
	range = &diag_sw_range_tbl[module_id - 1U];
	if (event == EventIdAny) {
		for (index = range->base; index <= range->ceil; index++) {
			if (module_sta[index])
				return true;
		}
		return false;
	} else {
		index = DIAG_SW_INDEX_GET(module_id, event);
		if (index > DIAG_SW_MAX) {
			pr_err("module:0x%x event:0x%x invalid\n", module_id, event);
			return false;
		}
		return module_sta[index];
	}
}

static int inject_value_get(uint16_t module_id, uint16_t event,
				uint32_t *out_val)
{
	uint32_t index;
	struct diag_sw_range *range;

    if (module_id > ModuleIdMax) {
        pr_err("%s:%s, id: %d over the limit\n",
                __FILE__, __func__, module_id);
        return -EINVAL;
    }
    pr_debug("%s:%d module_id: 0x%x, out_val:0x%x\n",
                __func__, __LINE__, module_id, *out_val);

	range = &diag_sw_range_tbl[module_id - 1U];
	if (event == EventIdAny) {
		for (index = range->base; index <= range->ceil; index++) {
			if (module_sta[DIAG_SW_MAX] && module_sta[index]) {
				module_sta[index] = false;
				*out_val = inject_vals[index];
				if (--inject_vals[DIAG_SW_MAX] == 0) {
					module_sta[DIAG_SW_MAX] = false;
					break;
				}
			}
		}
	} else {
		index = DIAG_SW_INDEX_GET(module_id, event);
		if (index > DIAG_SW_MAX) {
			pr_err("module:0x%x event:0x%x invalid\n", module_id, event);
			return -EINVAL;
		}
		if (module_sta[DIAG_SW_MAX] && module_sta[index]) {
			if (--inject_vals[DIAG_SW_MAX] == 0)
				module_sta[DIAG_SW_MAX] = false;
			module_sta[index] = false;
			*out_val = inject_vals[index];
		}
	}
	pr_debug("%s:%d module_id: 0x%x, out_val:0x%x\n",
				__func__, __LINE__, module_id, *out_val);

	return 0;
}

static int diag_inject_open(struct inode *i, struct file *f)
{
    return 0;
}

static int diag_inject_close(struct inode *i, struct file *f)
{
    return 0;
}

static long diag_inject_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    diag_inject_data_t inject_data;
    int ret = 0;
    switch (cmd)
    {
        case INJECT_EN:
            inject_enable();
            return 0;
        case INJECT_VAL_CHK:
            memset(&inject_data, 0, sizeof(inject_data));
            if (copy_from_user(&inject_data, (diag_inject_data_t *)arg, sizeof(diag_inject_data_t)))
                return -EACCES;
            ret = inject_val_chk(inject_data.module_id, inject_data.event_id,
							&(inject_data.inject_val));
            if (ret != 0)
                return ret;
            if (copy_to_user((diag_inject_data_t *)arg, &inject_data, sizeof(diag_inject_data_t)))
                ret = -EACCES;
            return ret;
        case INJECT_EN_CHK:
            memset(&inject_data, 0, sizeof(inject_data));
            if (copy_from_user(&inject_data, (diag_inject_data_t *)arg, sizeof(diag_inject_data_t)))
                return -EACCES;
            ret = inject_en_chk(inject_data.module_id, inject_data.event_id,
							&(inject_data.inject_val));
            if (ret != 0)
                return ret;
            if (copy_to_user((diag_inject_data_t *)arg, &inject_data, sizeof(diag_inject_data_t)))
                ret = -EACCES;
            return ret;
        case INJECT_VAL_SET:
            if (copy_from_user(&inject_data, (diag_inject_data_t *)arg, sizeof(diag_inject_data_t)))
                return -EACCES;
            return inject_val_set(inject_data.module_id, inject_data.event_id,
							inject_data.inject_val, false);
        case INJECT_VAL_CLR:
            if (copy_from_user(&inject_data, (diag_inject_data_t *)arg, sizeof(diag_inject_data_t)))
                return -EACCES;
            return inject_val_set(inject_data.module_id, inject_data.event_id,
							inject_data.inject_val, true);
        default:
            pr_err("%s: ioctl 0x%x not implemented!\n", __func__, cmd);
            return -EINVAL;
    }

    return 0;
}

static struct file_operations diag_inject_fops =
{
    .owner = THIS_MODULE,
    .open = diag_inject_open,
    .release = diag_inject_close,
    .unlocked_ioctl = diag_inject_ioctl
};

static int __init diag_inject_init(void) {
    int ret = 0;
    struct device *dev_ret = NULL;

    pr_info("Register %s module!\n", MODULE_NAME);
    inject_vals[0] = 0;
    ret = diag_inject_ops_register(&inject_registered, &inject_value_get);

    if ((ret = alloc_chrdev_region(&dev, FIRST_MINOR, MINOR_CNT, NODE_NAME)) < 0) {
        pr_err("ERR: %s module register failed at chr_dev region alloc!\n", MODULE_NAME);
        return ret;
    }

    cdev_init(&diag_inject_dev, &diag_inject_fops);

    if ((ret = cdev_add(&diag_inject_dev, dev, MINOR_CNT)) < 0) {
        pr_err("ERR: %s module register failed at chr_dev add!\n", MODULE_NAME);
        return ret;
    }
    if (IS_ERR(diag_inject_class = class_create(THIS_MODULE, "char"))) {
        pr_err("ERR: %s module register failed at class create!\n", MODULE_NAME);
        cdev_del(&diag_inject_dev);
        unregister_chrdev_region(dev, MINOR_CNT);
        return PTR_ERR(dev_ret);
    }

    if (IS_ERR(dev_ret = device_create(diag_inject_class, NULL, dev, NULL, NODE_NAME))) {
        pr_err("ERR: %s module register failed at device create!\n", MODULE_NAME);
        class_destroy(diag_inject_class);
        cdev_del(&diag_inject_dev);
        unregister_chrdev_region(dev, MINOR_CNT);
        return PTR_ERR(dev_ret);
    }

    return ret;
}

static void __exit diag_inject_exit(void) {
    pr_info("Unregister %s module!\n", MODULE_NAME);

    device_destroy(diag_inject_class, dev);
    class_destroy(diag_inject_class);
    cdev_del(&diag_inject_dev);
    unregister_chrdev_region(dev, MINOR_CNT);
    diag_inject_ops_unregister();
}

module_init(diag_inject_init);
module_exit(diag_inject_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dinggao Pan");
MODULE_DESCRIPTION("Diagnose error injection emulation module.");
MODULE_VERSION("0.01");
