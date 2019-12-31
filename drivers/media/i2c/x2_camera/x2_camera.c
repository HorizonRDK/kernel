/*
 * Copyright (C) 2018-2020  Horizon Robotics, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <asm/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include "soc/hobot/hobot_sensor_dev.h"

static int g_debug;
module_param(g_debug, int, 0644);

typedef struct _x2_camera_s {
	sensor_dev_t        curr_sensor;
	struct i2c_client  *client;
} x2_camera_t;

x2_camera_t  *g_x2_camera = NULL;

static int x2_camera_write_byte(struct i2c_client *client, uint32_t addr, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	printk(KERN_ERR "x2_camera i2c write error addr: 0%x reg: 0x%x ret %d !!!\n", addr, reg, ret);

	return ret;
}

static int x2_camera_read_byte(struct i2c_client *client, uint32_t addr, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[1];
	int ret;

	buf[0] = reg & 0xFF;

	msg[0].addr = addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	printk(KERN_ERR "x2_camera i2c read error addr: 0x%x reg: 0x%x ret %d !!!\n", addr, reg, ret);

	return ret;
}

/* sensor register write */
static int x2_camera_write(struct i2c_client *client, uint32_t addr, u16 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	msg.addr = addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	printk(KERN_ERR "x2_camera i2c write error addr: 0%x reg: 0x%x ret %d !!!\n", addr, reg, ret);

	return ret;
}

/* sensor register read */
static int x2_camera_read(struct i2c_client *client, uint32_t addr, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;

	msg[0].addr = addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	printk(KERN_ERR "x2_camera i2c read error addr: 0x%x reg: 0x%x ret %d !!!\n", addr, reg, ret);

	return ret;
}

static int x2_camera_write_array(struct i2c_client *client, uint32_t addr, uint8_t reg_size, sensor_reg_setting_t *regs)
{
	int i, ret = 0;

	for (i = 0; i < regs->size; i++) {
		if (g_debug >= 1) {
			printk(KERN_INFO "0x%x: 0x%x\n", regs->reg_setting[i].reg_addr, regs->reg_setting[i].reg_data);
		}
		if (SENSOR_I2C_BYTE_DATA == reg_size) {
			ret = x2_camera_write_byte(client, addr, regs->reg_setting[i].reg_addr, regs->reg_setting[i].reg_data);
			if (ret)
				return ret;
		} else {
			ret = x2_camera_write(client, addr, regs->reg_setting[i].reg_addr, regs->reg_setting[i].reg_data);
			if (ret)
				return ret;
		}
	}

	return ret;
}
/* ----------------------------------------------------------------------- */

static int x2_camera_s_stream(x2_camera_t *x2_camera, int enable)
{
	struct i2c_client *client = x2_camera->client;
	printk(KERN_INFO "x2_camera stream ctrl %d\n", enable);
	if (x2_camera->curr_sensor.chip) {
		if (enable) {
			if (NULL != x2_camera->curr_sensor.start) {
				printk(KERN_INFO "x2_camera enable %s streaming\n", x2_camera->curr_sensor.name);
				if (!strcmp(x2_camera->curr_sensor.name, "ov10635")) {
					if (x2_camera_write_byte(client, 0x60 >> 1, 0x33, 0x03)) {
						printk(KERN_ERR "error starting 964 stream for %s\n", x2_camera->curr_sensor.name);
						return -1;
					}
				}
				if (x2_camera_write_array(client,
										  x2_camera->curr_sensor.chip->slave_addr,
										  x2_camera->curr_sensor.chip->reg_size,
										  x2_camera->curr_sensor.start)) {
					printk(KERN_ERR "error starting stream %s\n", x2_camera->curr_sensor.name);
					return -1;
				}
			}
		} else {
			if (NULL != x2_camera->curr_sensor.stop) {
				printk(KERN_INFO "x2_camera disable %s streaming\n", x2_camera->curr_sensor.name);
				if (x2_camera_write_array(client,
										  x2_camera->curr_sensor.chip->slave_addr,
										  x2_camera->curr_sensor.chip->reg_size,
										  x2_camera->curr_sensor.start)) {
					printk(KERN_ERR "error stoping stream %s\n", x2_camera->curr_sensor.name);
					return -1;
				}
				if (!strcmp(x2_camera->curr_sensor.name, "ov10635")) {
					if (x2_camera_write_byte(client, 0x60 >> 1, 0x33, 0x0D)) {
						printk(KERN_ERR "error stoping 964 stream for %s\n", x2_camera->curr_sensor.name);
						return -1;
					}
				}
			}
		}
	}

	return 0;
}

static int x2_camera_init(x2_camera_t *x2_camera, uint32_t val)
{
	struct i2c_client *client = x2_camera->client;
	printk(KERN_INFO "x2_camera init %d\n", val);
	if (NULL == x2_camera->curr_sensor.name) {
		printk(KERN_ERR "x2_camera no sensor found\n");
		return -1;
	}
	if (NULL != x2_camera->curr_sensor.chip) {
		uint8_t             chip_id[2] = {0, };
		printk(KERN_INFO "x2_camera check %s chip id\n", x2_camera->curr_sensor.name);
		if (SENSOR_I2C_BYTE_DATA == x2_camera->curr_sensor.chip->reg_size) {
			if (x2_camera_read_byte(client,
									x2_camera->curr_sensor.chip->slave_addr,
									x2_camera->curr_sensor.chip->chip_id.reg_addr,
									&chip_id[0])) {
				printk(KERN_ERR "get chip id error\n");
				goto err;
			}
			if (x2_camera->curr_sensor.chip->chip_id.reg_data != chip_id[0]) {
				printk(KERN_ERR "chip id not match, expect: 0x%x got: 0x%x\n",
					   x2_camera->curr_sensor.chip->chip_id.reg_data, chip_id[0]);
				goto err;
			}
		} else {
			if (x2_camera_read(client,
							   x2_camera->curr_sensor.chip->slave_addr,
							   x2_camera->curr_sensor.chip->chip_id.reg_addr,
							   &chip_id[0])) {
				printk(KERN_ERR "get chip id error\n");
				goto err;
			}
			if (x2_camera_read(client,
							   x2_camera->curr_sensor.chip->slave_addr,
							   x2_camera->curr_sensor.chip->chip_id.reg_addr + 1,
							   &chip_id[1])) {
				printk(KERN_ERR "get chip id error\n");
				goto err;
			}
			if (x2_camera->curr_sensor.chip->chip_id.reg_data != (chip_id[0] << 8 | chip_id[1])) {
				printk(KERN_ERR "chip id not match, expect: 0x%x got: 0x%x\n",
					   x2_camera->curr_sensor.chip->chip_id.reg_data, (chip_id[0] << 8 | chip_id[1]));
				goto err;
			}
		}
		printk(KERN_INFO "x2_camera check %s chip id success\n", x2_camera->curr_sensor.name);
	}
	if (val) {
		if (NULL != x2_camera->curr_sensor.init) {
			printk(KERN_INFO "x2_camera %s init\n", x2_camera->curr_sensor.name);
			if (x2_camera_write_array(client,
									  x2_camera->curr_sensor.chip->slave_addr,
									  x2_camera->curr_sensor.chip->reg_size,
									  x2_camera->curr_sensor.init)) {
				printk(KERN_ERR "error initializing %s\n", x2_camera->curr_sensor.name);
				goto err;
			}
			printk(KERN_INFO "x2_camera %s init success\n", x2_camera->curr_sensor.name);
		}
	}
	return 0;
err:
	memset(&x2_camera->curr_sensor, 0, sizeof(x2_camera->curr_sensor));
	return -1;
}

static int x2_camera_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "x2 camera open\n");
	return 0;
}

static int x2_camera_open(struct inode *inode, struct file *file)
{
	return single_open(file, x2_camera_show, inode->i_private);
}

static long x2_camera_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = g_x2_camera->client;
	x2_camera_t       *x2_camera = g_x2_camera;
	int                ret = 0;
	//printk(KERN_INFO "%s\n", __func__);
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != 'V')
		return -ENOTTY;

	switch (cmd) {
	case X2VIDIOC_PROP:
		{
			sensor_cfg_t  cfg;
			sensor_dev_t  sensor;
			int           index = 0;
			int           try = 3;
			if (!arg) {
				printk(KERN_ERR "x2 camera prop error, sensor cfg should not be NULL\n");
				return -EINVAL;
			}
			ret = copy_from_user((void *)&cfg, (void __user *)arg, sizeof(sensor_cfg_t));
			if (ret) {
				printk(KERN_ERR "x2 camera prop error, copy data from user failed %d\n", ret);
				return -EINVAL;
			}
			do {
				memset(&sensor, 0, sizeof(sensor_dev_t));
				ret = sensor_dev_prop(&sensor, &cfg, index);
				if (ret < 0) {
					printk(KERN_ERR "x2_camera no matched sensor found\n");
					return -ENODEV;
				} else {
					printk(KERN_INFO "x2_camera sensor found: %s\n", sensor.name);
					memcpy(&x2_camera->curr_sensor, &sensor, sizeof(x2_camera->curr_sensor));
					index = ret;
					ret  = x2_camera_init(x2_camera, 1);
					if (0 == ret) {
						printk(KERN_INFO "x2_camera sensor %s init success\n", sensor.name);
						return 0;
					}
					memset(&x2_camera->curr_sensor, 0, sizeof(x2_camera->curr_sensor));
					printk(KERN_INFO "x2_camera sensor %s init failed\n", sensor.name);
				}
			} while (try -- );
		}
		break;
	case X2VIDIOC_CSIPARAM:
		{
			if (!arg) {
				printk(KERN_ERR "x2 camera get csi param error, input args should not be NULL\n");
				return -EINVAL;
			}
			if (NULL == x2_camera->curr_sensor.name) {
				printk(KERN_ERR "x2 camera get csi param error, no avaliable device found\n");
				return -ENODEV;
			}
			ret = copy_to_user((void __user *)arg, (void *)x2_camera->curr_sensor.mipi, sizeof(mipi_param_t));
			if (ret) {
				printk(KERN_ERR "x2 camera get csi param error, copy data to user failed %d\n", ret);
				return -EINVAL;
			}
		}
		break;
	case X2VIDIOC_I2CREAD:
		{
			x2_camera_i2c_t  reg = {0, };
			uint8_t data = 0;
			if (!arg) {
				printk(KERN_ERR "x2 camera i2c read error, reg should not be NULL\n");
				return -EINVAL;
			}
			ret = copy_from_user((void *)&reg, (void __user *)arg, sizeof(x2_camera_i2c_t));
			if (ret) {
				printk(KERN_ERR "x2 camera i2c read error, copy data from user failed %d\n", ret);
				return -EINVAL;
			}
			if (reg.reg_size == 1) {
				if (x2_camera_read_byte(client, reg.i2c_addr, (uint8_t)reg.reg, (uint8_t *)&data)) {
					printk(KERN_ERR "i2c read error\n");
					return -EINVAL;
				}
			} else {
				if (x2_camera_read(client, reg.i2c_addr, reg.reg, &data)) {
					printk(KERN_ERR "i2c read error\n");
					return -EINVAL;
				}
			}
			reg.data = data;
			//printk(KERN_ERR "x2 camera i2c read 0x%x 0x%x: 0x%x\n", reg.i2c_addr, reg.reg, reg.data);
			if (copy_to_user((void __user *)arg, (void *)&reg, sizeof(x2_camera_i2c_t))) {
				printk(KERN_ERR "x2 camera i2c read error, copy data to user failed %d\n", ret);
				return -EINVAL;
			}
		}
		break;
	case X2VIDIOC_I2CWRITE:
		{
			x2_camera_i2c_t  reg = {0, };
			if (!arg) {
				printk(KERN_ERR "x2 camera i2c write error, reg should not be NULL\n");
				return -EINVAL;
			}
			ret = copy_from_user((void *)&reg, (void __user *)arg, sizeof(x2_camera_i2c_t));
			if (ret) {
				printk(KERN_ERR "x2 camera i2c write error, copy data from user failed %d\n", ret);
				return -EINVAL;
			}
			if (reg.reg_size == 1) {
				if (x2_camera_write_byte(client, reg.i2c_addr, (uint8_t)reg.reg, (uint8_t)reg.data)) {
					printk(KERN_ERR "i2c write error\n");
					return -EINVAL;
				}
			} else {
				if (x2_camera_write(client, reg.i2c_addr, (uint16_t)reg.reg, (uint8_t)reg.data)) {
					printk(KERN_ERR "i2c write error\n");
					return -EINVAL;
				}
			}
			//printk(KERN_ERR "x2 camera i2c write 0x%x 0x%x: 0x%x\n", reg.i2c_addr, reg.reg, reg.data);
		}
		break;
	case X2VIDIOC_STREAM_ON:
		{
			if (x2_camera_s_stream(x2_camera, true)) {
				printk(KERN_ERR "stream on error\n");
				return -EINVAL;
			}
		}
		break;
	case X2VIDIOC_STREAM_OFF:
		{
			if (x2_camera_s_stream(x2_camera, false)) {
				printk(KERN_ERR "stream on error\n");
				return -EINVAL;
			}
		}
		break;
	default:
		return -ENOENT;
	}
	return 0;
}

static const struct file_operations x2_camera_fops = {
	.owner		= THIS_MODULE,
	.open		= x2_camera_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.unlocked_ioctl = x2_camera_ioctl,
	.compat_ioctl = x2_camera_ioctl,
};

static int    x2_camera_major = 0;
struct cdev   x2_camera_cdev;
extern struct class  *vps_class;
static struct device *x2_camera_dev;

static int x2_camera_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	x2_camera_t        *x2_camera;
	dev_t               devno;
	struct cdev        *p_cdev = &x2_camera_cdev;
	int                 ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	x2_camera = devm_kzalloc(&client->dev, sizeof(x2_camera_t), GFP_KERNEL);
	if (!x2_camera)
		return -ENOMEM;
	x2_camera->client = client;

	ret = alloc_chrdev_region(&devno, 0, 1, "x2_camera");
	if (ret < 0) {
		printk(KERN_ERR "Error %d while alloc chrdev x2_fpgactrl", ret);
		goto err;
	}
	x2_camera_major = MAJOR(devno);
	cdev_init(p_cdev, &x2_camera_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		printk(KERN_ERR "Error %d while adding x2 fpgactrl cdev", ret);
		goto err;
	}

	x2_camera_dev = device_create(vps_class, NULL, MKDEV(x2_camera_major, 0), NULL, "x2_camera");
	if (IS_ERR(x2_camera_dev)) {
		printk(KERN_ERR "[%s] deivce create error\n", __func__);
		ret = PTR_ERR(x2_camera_dev);
		goto err;
	}

	i2c_set_clientdata(client, x2_camera);

	client->flags = I2C_CLIENT_SCCB;
	printk(KERN_INFO "chip found @ 0x%02x (%s)\n", client->addr << 1, client->adapter->name);

	memset(&x2_camera->curr_sensor, 0, sizeof(x2_camera->curr_sensor));
	g_x2_camera = x2_camera;
	printk(KERN_INFO "!!!!x2_camera probe OK!!!\n");
	return 0;
err:
	cdev_del(&x2_camera_cdev);
	unregister_chrdev_region(MKDEV(x2_camera_major, 0), 1);
	if (x2_camera) {
		devm_kfree(&client->dev, x2_camera);
	}
	return ret;
}

static int x2_camera_remove(struct i2c_client *client)
{
	x2_camera_t *x2_camera = i2c_get_clientdata(client);
	cdev_del(&x2_camera_cdev);
	unregister_chrdev_region(MKDEV(x2_camera_major, 0), 1);
	if (x2_camera) {
		devm_kfree(&client->dev, x2_camera);
	}
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id x2_camera_of_match[] = {
	{.compatible = "x2,camera", .data = NULL},
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, x2_camera_of_match);
#endif

static const struct i2c_device_id x2_camera_id[] = {
	{"x2_camera", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, x2_camera_id);

static struct i2c_driver x2_camera_driver = {
	.driver = {
		.name	= "x2_camera",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(x2_camera_of_match),
#endif
	},
	.probe = x2_camera_probe,
	.remove = x2_camera_remove,
	.id_table = x2_camera_id,
};
module_i2c_driver(x2_camera_driver);

MODULE_DESCRIPTION("OmniVision x2_camera sensor driver");
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_LICENSE("GPL v2");
