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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <soc/hobot/hb_pinctrl_cdev.h>


#define DRIVER_NAME "pinctrl_cdev"

/* pin schmitt */
#define SCHMITT1_EN	(1 << 8)
#define SCHMITT2_EN	(1 << 9)

/* pin pull */
#define PULL1_TYPE  (1 << 7)
#define PULL1_EN    (1 << 6)
#define PULL2_UP    (1 << 8)
#define PULL2_DN    (1 << 7)

/* pin drive strength */
#define GET_DRIVE_VAL(reg)          ((reg >> 2) & 0xF)
#define SET_DRIVE_VAL(reg, drive)   {reg &= ~(0xF << 2); reg |= (drive << 2);}

/* pin function */
#define GET_FUNC_VAL(reg)           (reg & 0x3)
#define SET_FUNC_VAL(reg, func)     {reg &= ~0x3; reg |= func;}

static const u16 pull_sechmitt_type[] = {
    0xF3F8,  /* GPIO[0] */
    0xFFF0,  /* GPIO[1] */
    0xFE00,  /* GPIO[2] */
    0xFFFF,  /* GPIO[3] */
    0xFFFC,  /* GPIO[4] */
    0x7FFF,  /* GPIO[5] */
    0x8000,  /* GPIO[6] */
    0x00F1,  /* GPIO[7] */
};
static const u16 drive_strength_type[] = {
    0xF3F8,  /* GPIO[0] */
    0xFFF0,  /* GPIO[1] */
    0x0000,  /* GPIO[2] */
    0x0020,  /* GPIO[3] */
    0x0000,  /* GPIO[4] */
    0x0000,  /* GPIO[5] */
    0x0000,  /* GPIO[6] */
    0x0000,  /* GPIO[7] */
};

/*
 * type 1 -> bit5:2(0000b~1111b==3mA~45mA{ 3, 6,  9, 12, 17, 20, 22, 25, 33, 35, 37, 39, 41, 42.5, 44, 45})
 * type 2 -> bit5:2(0000b~1111b==6mA~45mA{ 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 41, 42.5, 44, 45})
 *
 */
static const u8 driver_strength_map[2][16] = {
    { 3, 6,  9, 12, 17, 20, 22, 25, 33, 35, 37, 39, 41, 42, 44, 45},
    { 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 41, 42, 44, 45},
};

enum call_type {
	HB_PINCTRL_CALL_KERNEL,
	HB_PINCTRL_CALL_USER,
};

struct hb_pinctrl_cdev_t {
    const char *name;
    struct class *class;
    struct device *device;
    struct cdev cdev;
    dev_t dev_num;
    int major_num;
};

struct hb_pinctrl_pin_type_t {
    u8 pull;
    u8 sechmitt;
    u8 drive;
};

struct hb_pinctrl_device_t {
    struct device *dev;
    struct resource *res;
    void __iomem *ioaddr;
    struct hb_pinctrl_cdev_t pinctrl_cdev;
    struct hb_pinctrl_pin_type_t *pin_type;
    u32 pin_range;
	atomic_t ready;
};

struct hb_pinctrl_device_t hb_pinctrl_dev;

static inline bool pin_num_check(struct hb_pinctrl_config config)
{
    return config.pin_num < PINCTRL_IOCTRL_MAX_PINS;
}

static u16 drive_strength_round(u8 drive_val, u8 drive_type, u8 round_type)
{
    u8 i = 0;
    u16 res = 0;
    u16 min_index = 0;
    u16 max_index = ARRAY_SIZE(driver_strength_map[drive_type]) - 1;

    if (drive_type >= ARRAY_SIZE(driver_strength_map))
        return -ENAVAIL;
    if (drive_val >= driver_strength_map[drive_type][max_index])
        return max_index;
    else if (drive_val <= driver_strength_map[drive_type][min_index])
        return min_index;
    for (i = min_index; i <= max_index; i++) {
        if (drive_val >= driver_strength_map[drive_type][i]
            && drive_val < driver_strength_map[drive_type][i+1])
            break;
    }
    switch (round_type) {
    case DRIVE_SET_CEIL:
        res = i;
        break;
    case DRIVE_SET_FLOOR:
        if (drive_val == driver_strength_map[drive_type][i])
            res = i;
        else
            res = i+1;
        res = i + 1;
        break;
    case DRIVE_SET_ROUND:
    default:
        if (drive_val - driver_strength_map[drive_type][i]
            <= driver_strength_map[drive_type][i+1] - drive_val)
            res = i;
        else
            res = i+1;
        break;
    }
    return res;
}

static int set_sechmitt_state(struct hb_pinctrl_device_t *dev,
    struct hb_pinctrl_config *config, u32 *reg_value)
{

    u32 pin_num = config->pin_num;
    if (config->pin_sechmitt >= IOCTL_PIN_SECHMITT_MAX) {
        dev_err(dev->dev, "pin sechmitt parameter error");
        return -ENAVAIL;
    }
    // write sechmitt status
    if (dev->pin_type[pin_num].sechmitt == 0) {
        if (config->pin_sechmitt == IOCTL_PIN_SECHMITT_DIS)
            *reg_value &= ~SCHMITT1_EN;
        else
            *reg_value |= SCHMITT1_EN;
    } else {
        if (config->pin_sechmitt == IOCTL_PIN_SECHMITT_DIS)
            *reg_value &= ~SCHMITT2_EN;
        else
            *reg_value |= SCHMITT2_EN;
    }
    return 0;
}

static void get_sechmitt_state(struct hb_pinctrl_device_t *dev,
    struct hb_pinctrl_config *config, u32 reg_value)
{

    u32 pin_num = config->pin_num;
    // read sechmitt status
    if (dev->pin_type[pin_num].sechmitt == 0) {
        if (reg_value & SCHMITT1_EN)
            config->pin_sechmitt = IOCTL_PIN_SECHMITT_EN;
        else
            config->pin_sechmitt = IOCTL_PIN_SECHMITT_DIS;
    } else {
        if (reg_value & SCHMITT2_EN)
            config->pin_sechmitt = IOCTL_PIN_SECHMITT_EN;
        else
            config->pin_sechmitt = IOCTL_PIN_SECHMITT_DIS;
    }
}

static int set_pull_state(struct hb_pinctrl_device_t *dev,
    struct hb_pinctrl_config *config, u32 *reg_value)
{

    u32 pin_num = config->pin_num;
    if (config->pin_pull >= IOCTL_PIN_PULL_MAX) {
        dev_err(dev->dev, "pin pull parameter error");
        return -ENAVAIL;
    }
    // write pull status
    if (dev->pin_type[pin_num].pull == 0) {
        /*
         * type 0 -> bit7(0==pulldown, 1==pullup)
         *           bit6(0==pull disable, 1==pull enable)
         *
         */
        dev_dbg(dev->dev, "pull type 0");
        if (config->pin_pull == IOCTL_PIN_PULL_UP) {
            *reg_value |= PULL1_EN;
            *reg_value |= PULL1_TYPE;
        } else if (config->pin_pull == IOCTL_PIN_PULL_DOWN) {
            *reg_value |= PULL1_EN;
            *reg_value &= ~PULL1_TYPE;
        } else {
            *reg_value &= ~PULL1_EN;
        }
    } else {
        /*
         * type 2 -> bit8(0==pullup enable, 1==pullup enable)
         *           bit7(0==pulldown diable, 1==pulldown enable)
         */
        dev_dbg(dev->dev, "pull type 1");
        if (config->pin_pull == IOCTL_PIN_PULL_UP) {
            *reg_value |= PULL2_UP;
            *reg_value &= ~PULL2_DN;
        } else if (config->pin_pull == IOCTL_PIN_PULL_DOWN) {
            *reg_value &= ~PULL2_UP;
            *reg_value |= PULL2_DN;
        } else {
            *reg_value &= ~PULL2_UP;
            *reg_value &= ~PULL2_DN;
        }
    }
    return 0;
}

static void get_pull_state(struct hb_pinctrl_device_t *dev,
    struct hb_pinctrl_config *config, u32 reg_value)
{

    u32 pin_num = config->pin_num;
    // read pull status
    if (dev->pin_type[pin_num].pull == 0) {
       /*
        * type 0 -> bit7(0==pulldown, 1==pullup)
        *           bit6(0==pull disable, 1==pull enable)
        *
        */
        dev_dbg(dev->dev, "pull type 0");
        if (reg_value & PULL1_EN && reg_value & PULL1_TYPE)
            config->pin_pull = IOCTL_PIN_PULL_UP;
        else if (reg_value & PULL1_EN && !(reg_value & PULL1_TYPE))
            config->pin_pull = IOCTL_PIN_PULL_DOWN;
        else
            config->pin_pull = IOCTL_PIN_PULL_DIS;
    } else {
       /*
        * type 2 -> bit8(0==pullup enable, 1==pullup enable)
        *           bit7(0==pulldown diable, 1==pulldown enable)
        */
        dev_dbg(dev->dev, "pull type 1");
        if (reg_value & PULL2_UP && !(reg_value & PULL2_DN))
            config->pin_pull =  IOCTL_PIN_PULL_UP;
        else if (!(reg_value & PULL2_UP) && reg_value & PULL2_DN)
            config->pin_pull =  IOCTL_PIN_PULL_DOWN;
        else
            config->pin_pull =  IOCTL_PIN_PULL_DIS;
    }
}

static int set_drive_state(struct hb_pinctrl_device_t *dev,
    struct hb_pinctrl_config *config, u32 *reg_value)
{

    u16 drive_index;
    u32 pin_num = config->pin_num;
    /*
    if (config->pin_drive >= (0x1 << 4)) {
        dev_err(dev->dev, "pin drive parameter error");
        return -ENAVAIL;
    }
    */
    // write drive strength
    drive_index = drive_strength_round(config->pin_drive,
                    dev->pin_type[pin_num].drive, DRIVE_SET_ROUND);
    if (drive_index < 0) {
        dev_err(dev->dev, "get drive index failed");
        return drive_index;
    }
    dev_dbg(dev->dev, "%d, drive_index=%x", __LINE__, drive_index);
    SET_DRIVE_VAL(*reg_value, drive_index);
    return 0;
}

static void get_drive_state(struct hb_pinctrl_device_t *dev,
    struct hb_pinctrl_config *config, u32 reg_value)
{

    u32 pin_num = config->pin_num;
    if (dev->pin_type[pin_num].drive == 0) {
        config->pin_drive = driver_strength_map[0][GET_DRIVE_VAL(reg_value)];
    } else {
        config->pin_drive = driver_strength_map[1][GET_DRIVE_VAL(reg_value)];
    }
}

static int set_func_state(struct hb_pinctrl_device_t *dev,
    struct hb_pinctrl_config *config, u32 *reg_value)
{

    if (config->pin_func >= IOCTL_PIN_FUNC_MAX) {
        dev_err(dev->dev, "pin function parameter error");
        return -ENAVAIL;
    }
    SET_FUNC_VAL(*reg_value, config->pin_func);
    return 0;
}

static void get_func_state(struct hb_pinctrl_device_t *dev,
    struct hb_pinctrl_config *config, u32 reg_value) {

    config->pin_func = GET_FUNC_VAL(reg_value);
}

static int hb_pinctrl_set(struct hb_pinctrl_device_t *dev,
				unsigned long arg, enum call_type c_type)
{
    int ret, i;
    u16 pin_num;
    u32 reg_value;
    void __iomem *addr;
    struct hb_pinctrl_ioctrl_data ioctrl_data;
    struct hb_pinctrl_config *config;
    char s[6][10] = {
        "pins", "sechmitt", "pull", "drive", "function", "reg_value"
    };

    dev_dbg(dev->dev, "start %s", __func__);
	if (c_type == HB_PINCTRL_CALL_USER) {
		ret = copy_from_user(&ioctrl_data,
				(struct hb_pinctrl_ioctrl_data __user *)arg, sizeof(ioctrl_data));
		if (ret) {
			dev_err(dev->dev, "pinctrl_cdev: ioctl copy from user error\n");
			return -EFAULT;
		}

		if (ioctrl_data.npins > PINCTRL_IOCTRL_MAX_PINS) {
			dev_err(dev->dev, "npins over pinctrl max pins");
			return -EINVAL;
		}

		config = memdup_user(ioctrl_data.config,
					ioctrl_data.npins * sizeof(struct hb_pinctrl_config));
		if (IS_ERR(config)) {
			dev_err(dev->dev, "memdup_user error");
			return PTR_ERR(config);
		}
	} else {
        ioctrl_data = *((struct hb_pinctrl_ioctrl_data *)(arg));
		config = ioctrl_data.config;
	}
    dev_dbg(dev->dev, "%-10s %-10s %-10s %-10s %-10s %-10s",
        s[0], s[1], s[2], s[3], s[4], s[5]);
    for (i = 0; i < ioctrl_data.npins; ++i) {
        pin_num = config[i].pin_num;
        addr = dev->ioaddr + pin_num * 0x4;
        reg_value = readl(addr);

        // check ioctrl parametermeters from userspace
        if (!pin_num_check(config[i])) {
            dev_err(dev->dev, "pin number error");
            ret = -ENAVAIL;
            goto error;
        }
        // set sechmitt
        if (config[i].set_mask & PINCTRL_SET_SECHMITT) {
            ret = set_sechmitt_state(dev, &config[i], &reg_value);
            if (ret) {
                goto error;
            }
        }

        // set pull
        if (config[i].set_mask & PINCTRL_SET_PULL) {
            ret = set_pull_state(dev, &config[i], &reg_value);
            if (ret) {
                goto error;
            }
        }

	    // set drive
        if (config[i].set_mask & PINCTRL_SET_DRIVE) {
            ret = set_drive_state(dev, &config[i], &reg_value);
            if (ret) {
                goto error;
            }
        }

	    // set pin function
        if (config[i].set_mask & PINCTRL_SET_FUNC) {
            ret = set_func_state(dev, &config[i], &reg_value);
            if (ret) {
                goto error;
            }
        }
        dev_dbg(dev->dev, "%-10d %-10d %-10d %-10d %-10d 0x%-8x",
            config[i].pin_num, config[i].pin_sechmitt, config[i].pin_pull,
            config[i].pin_drive, config[i].pin_func, reg_value);

        writel(reg_value, addr);
    }

    return 0;
error:
	if (c_type == HB_PINCTRL_CALL_USER) {
		kfree(config);
	}
    return ret;
}

static int hb_pinctrl_get(struct hb_pinctrl_device_t *dev,
					unsigned long arg, enum call_type c_type)
{
    int ret, i;
    u16 pin_num;
    u32 reg_value;
    void __iomem *addr;
    struct hb_pinctrl_ioctrl_data ioctrl_data;
    struct hb_pinctrl_config *config;
    char s[6][10] =
        {"pins", "sechmitt", "pull", "drive", "function", "reg_value"};

    dev_dbg(dev->dev, "start %s", __func__);
	if (c_type == HB_PINCTRL_CALL_USER) {
		ret = copy_from_user(&ioctrl_data,
				(struct hb_pinctrl_ioctrl_data __user *)arg, sizeof(ioctrl_data));
		if (ret) {
			dev_err(dev->dev, "pinctrl_cdev: ioctl copy from user error\n");
			return -EFAULT;
		}

		if (ioctrl_data.npins > PINCTRL_IOCTRL_MAX_PINS) {
			dev_err(dev->dev, "npins over pinctrl max pins");
			return -EINVAL;
		}

		config = memdup_user(ioctrl_data.config,
					ioctrl_data.npins * sizeof(struct hb_pinctrl_config));
		if (IS_ERR(config)) {
			dev_err(dev->dev, "memdup_user error");
			return PTR_ERR(config);
		}
	} else {
        ioctrl_data = *((struct hb_pinctrl_ioctrl_data *)(arg));
		config = ioctrl_data.config;
	}

    dev_dbg(dev->dev, "%-10s %-10s %-10s %-10s %-10s %-10s",
        s[0], s[1], s[2], s[3], s[4], s[5]);
    for (i = 0; i < ioctrl_data.npins; ++i) {
        pin_num = config[i].pin_num;
        addr = dev->ioaddr + pin_num * 0x4;
        reg_value = readl(addr);

        // check ioctrl parametermeters from userspace
        if (!pin_num_check(config[i])) {
            dev_err(dev->dev, "pin number error");
            ret = -ENAVAIL;
            goto error;
        }
        // get sechmitt
        get_sechmitt_state(dev, &config[i], reg_value);

        // get pull
        get_pull_state(dev, &config[i], reg_value);

        // get drive
        get_drive_state(dev, &config[i], reg_value);

        // get func
        get_func_state(dev, &config[i], reg_value);

        dev_dbg(dev->dev, "%-10d %-10d %-10d %-10d %-10d 0x%-8x",
            config[i].pin_num, config[i].pin_sechmitt, config[i].pin_pull,
            config[i].pin_drive, config[i].pin_func, reg_value);
    }
	if (c_type == HB_PINCTRL_CALL_USER) {
		ret = copy_to_user(ioctrl_data.config, config,
				ioctrl_data.npins * sizeof(struct hb_pinctrl_config));
		if (ret) {
			dev_err(dev->dev, "copy to userspace failed");
			ret = -EFAULT;
			goto error;
		}
	}
    return 0;
error:
	if (c_type == HB_PINCTRL_CALL_USER) {
		kfree(config);
	}
    return ret;
}

static DEFINE_MUTEX(pinctrl_dev_open_tmux);
static int pinctrl_dev_open(struct inode *inode, struct file *file)
{
    return 0;
}

static DEFINE_MUTEX(pinctrl_dev_write_mutex);
static ssize_t pinctrl_dev_write(struct file *file, const char __user *buf,
        size_t count, loff_t *ppos)
{
	return 0;
}

static DEFINE_MUTEX(pinctrl_dev_read_mutex);
static ssize_t pinctrl_dev_read(struct file *file, char __user *buf,
    size_t size, loff_t *ppos)
{
	return 0;
}

static DEFINE_MUTEX(pinctrl_dev_ioctl_mutex);
static long pinctrl_dev_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
    struct hb_pinctrl_device_t *dev = &hb_pinctrl_dev;

    dev_dbg(dev->dev, "%s, cmd=%d", __func__, cmd);
    switch (cmd) {
    case PINCTRL_SET:
        return hb_pinctrl_set(dev, arg, HB_PINCTRL_CALL_USER);
    case PINCTRL_GET:
        return hb_pinctrl_get(dev, arg, HB_PINCTRL_CALL_USER);
    default:
        break;
    }
    return 0;
}



static int pinctrl_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations pinctrl_dev_fops = {
    .owner    = THIS_MODULE,
    .open     = pinctrl_dev_open,
    .write    = pinctrl_dev_write,
    .read     = pinctrl_dev_read,
    .release  = pinctrl_dev_release,
    .unlocked_ioctl = pinctrl_dev_ioctl,
    .compat_ioctl   = pinctrl_dev_ioctl,
};

int hobot_pinctrl_set(struct hb_pinctrl_ioctrl_data *ioctl_data)
{
	if (!atomic_read(&hb_pinctrl_dev.ready))
		return -ENODEV;
	return hb_pinctrl_set(&hb_pinctrl_dev, (unsigned long)ioctl_data,
				HB_PINCTRL_CALL_KERNEL);
}
EXPORT_SYMBOL_GPL(hobot_pinctrl_set);

int hobot_pinctrl_get(struct hb_pinctrl_ioctrl_data *ioctl_data)
{
	if (!atomic_read(&hb_pinctrl_dev.ready))
		return -ENODEV;
	return hb_pinctrl_get(&hb_pinctrl_dev, (unsigned long)ioctl_data,
				HB_PINCTRL_CALL_KERNEL);
}
EXPORT_SYMBOL_GPL(hobot_pinctrl_get);

static int hb_pinctrl_dev_init(struct hb_pinctrl_device_t *dev)
{
    int ret = 0;
    dev_t devno;
    struct cdev *p_cdev = &dev->pinctrl_cdev.cdev;

    dev_dbg(dev->dev, "hb_pinctrl dev init enter");
    ret = alloc_chrdev_region(&devno, 0, 1, "pinctrl dev");
    if (ret < 0) {
        dev_err(dev->dev, "Error %d while alloc chrdev pinctrl dev", ret);
		goto alloc_chrdev_error;
    }
    dev->pinctrl_cdev.major_num = MAJOR(devno);
    cdev_init(p_cdev, &pinctrl_dev_fops);
    p_cdev->owner = THIS_MODULE;
    ret = cdev_add(p_cdev, devno, 1);
    if (ret) {
	    dev_err(dev->dev, "Error %d while adding example pinctrl cdev", ret);
		goto cdev_add_error;
    }
    // create class for pinctrl_cdev
    dev->pinctrl_cdev.class = class_create(THIS_MODULE, "pinctrl_drv");
    if (IS_ERR(dev->pinctrl_cdev.class)) {
        dev_err(dev->dev, "[%s] class create error\n", __func__);
		ret = PTR_ERR(dev->pinctrl_cdev.class);
		goto class_create_error;
    }
    // create device for pinctrl_cdev
    dev->pinctrl_cdev.device = device_create(dev->pinctrl_cdev.class, NULL,
        devno, NULL, "pinctrl_dev");
    if (IS_ERR(dev->pinctrl_cdev.device)) {
		dev_err(dev->dev, "[%s] device create error\n", __func__);
		ret = PTR_ERR(dev->pinctrl_cdev.device);
		goto device_create_error;
	}
    return 0;

device_create_error:
	class_destroy(dev->pinctrl_cdev.class);
class_create_error:
	cdev_del(&dev->pinctrl_cdev.cdev);
cdev_add_error:
	unregister_chrdev_region(MKDEV(dev->pinctrl_cdev.major_num , 0), 1);
alloc_chrdev_error:
	return ret;
}

static void hb_pinctrl_dev_exit(struct hb_pinctrl_device_t *dev)
{
    dev_info(dev->dev, "pinctrl cdev exit");

    device_destroy(dev->pinctrl_cdev.class,
            MKDEV(dev->pinctrl_cdev.major_num, 0));
    class_destroy(dev->pinctrl_cdev.class);
    cdev_del(&dev->pinctrl_cdev.cdev);
    unregister_chrdev_region(MKDEV(dev->pinctrl_cdev.major_num, 0), 1);
}

static int hb_pinctrl_probe(struct platform_device *pdev)
{
    int ret = 0, i = 0;
    struct resource *res;
    void __iomem *addr;
    u32 reg_info[2];
    const char s[4][10] = {"pins", "driver", "pull", "sechmitt"};
    struct hb_pinctrl_device_t *dev = &hb_pinctrl_dev;

    dev_info(&pdev->dev, "start probe");
    dev->dev = &pdev->dev;

    // get io register start address and length from device tree
    ret = of_property_read_u32_array(pdev->dev.of_node, "reg-info", reg_info,
				ARRAY_SIZE(reg_info));
    if (ret) {
        dev_err(dev->dev, "get reg_info failed");
        goto err_res;
    }
    // ioremap register to ram
    res = devm_kmalloc(&pdev->dev, sizeof(struct resource), GFP_KERNEL);
    if (!res) {
        dev_err(dev->dev, "malloc resource failed");
        goto err_res;
    }
    dev->res = res;
	dev->res->start = reg_info[0];
    dev->res->end = dev->res->start + reg_info[1] - 1;
    dev->res->flags = IORESOURCE_MEM;

    addr = devm_ioremap(dev->dev, dev->res->start,
                dev->res->end - dev->res->start + 1);
    if (IS_ERR(addr)) {
        dev_err(dev->dev, "ioremap platform resource failed");
        ret = PTR_ERR(addr);
        goto err_res;
    }
    dev->ioaddr = addr;

    ret = of_property_read_u32(dev->dev->of_node, "gpio-range",
            &dev->pin_range);
    if (ret) {
        dev_err(dev->dev, "get gpio-range failed");
        goto err_res;
    }
    if (dev->pin_range > PINCTRL_IOCTRL_MAX_PINS) {
        dev_err(dev->dev, "gpio-range too long");
        goto err_res;
    }
    dev->pin_type = devm_kmalloc_array(dev->dev, dev->pin_range,
                        sizeof(struct hb_pinctrl_pin_type_t), GFP_KERNEL);
    if (IS_ERR(dev->pin_type)) {
        dev_err(dev->dev, "kmalloc pin_type failed");
        goto err_res;
    }

    dev_dbg(dev->dev, "%-10s %-10s %-10s %-10s", s[0], s[1], s[2], s[3]);
    for (i = 0; i <= dev->pin_range; ++i) {
        dev->pin_type[i].drive = !!(drive_strength_type[i/16] & (0x1 << i%16));
        dev->pin_type[i].pull = !!(pull_sechmitt_type[i/16] & (0x1 << i%16));
        dev->pin_type[i].sechmitt = dev->pin_type[i].pull;
        dev_dbg(dev->dev, "%-10d %-10d %-10d %-10d", i, dev->pin_type[i].drive,
            dev->pin_type[i].pull, dev->pin_type[i].sechmitt);
    }
    ret = hb_pinctrl_dev_init(dev);
    if (ret) {
        dev_err(dev->dev, "pinctrl cdev init failed");
        goto err_res;
    }
	atomic_set(&dev->ready, 1);
    dev_info(dev->dev, "probe ok");
    return 0;
err_res:
    return ret;
}

static int hb_pinctrl_remove(struct platform_device *pdev)
{
	struct hb_pinctrl_device_t *dev = &hb_pinctrl_dev;

	atomic_set(&dev->ready, 0);
    hb_pinctrl_dev_exit(&hb_pinctrl_dev);
    return 0;
}

static const struct of_device_id hb_pinctrl_of_match[] = {
    {
        .compatible = "hobot,pinctrl-cdev",
    },
	{},
};

MODULE_DEVICE_TABLE(of, hb_pinctrl_of_match);

static struct platform_driver hb_pinctrl_driver = {
    .probe = hb_pinctrl_probe,
    .remove = hb_pinctrl_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = hb_pinctrl_of_match,
        .owner = THIS_MODULE,
    }
};

module_platform_driver(hb_pinctrl_driver);
MODULE_AUTHOR("Yu Kong <yu.kong@horizon.ai>");
MODULE_DESCRIPTION("hobot pinctrl cdev");
MODULE_LICENSE("GPL");
