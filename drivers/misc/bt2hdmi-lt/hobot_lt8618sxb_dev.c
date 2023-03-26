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
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include "hobot_lt8618sxb_config.h"

#include "hobot_lt8618sxb.h"

struct gpio_desc *lt8618sxb_reset_gpio = NULL;
struct x2_lt8618sxb_s *g_x2_lt8618sxb;
#ifndef CONFIG_HOBOT_X3_UBUNTU
static void reset_hdmi_converter(void)
{
	return;
}

static int disp_config_hdmi(unsigned short vmode,
			unsigned short VideoFormat, unsigned short Afs)
{
	return 0;
}

#endif
#define LT8618_IO_MAGIC		'F'

#define LT8618_IOW(num, dtype)	_IOW(LT8618_IO_MAGIC, num, dtype)
#define LT8618_IOR(num, dtype)	_IOR(LT8618_IO_MAGIC, num, dtype)
#define LT8618_IOWR(num, dtype)	_IOWR(LT8618_IO_MAGIC, num, dtype)
#define LT8618_IO(num)		    _IO(LT8618_IO_MAGIC, num)

#define LT8618_SET_RESOLUTION_RATIO		LT8618_IOW(101, unsigned int)
#define LT8618_GET_EDID_RESOLUTION_RATIO		LT8618_IOR(100, hobot_lt8618_sync_t)
//#define LT8618_SET_POLARITY	    LT8618_IOW(102, unsigned int)
//#define LT8618_GET_POLARITY		LT8618_IOR(103, unsigned int)
//#define LT8618_ENABLE	        LT8618_IO(104)
//#define LT8618_DISABLE	        LT8618_IO(105)


static hobot_lt8618_ioctl_t lt8618_config = {0};
#if 1
static int hobot_lt8618_open(struct inode *inode, struct file *file)
{
    file->private_data = &lt8618_config;
	
	return 0;
}

static int hobot_lt8618_release(struct inode *inode, struct file *file)
{
	return 0;
}
#endif

static long lt8618_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	hobot_lt8618_ioctl_t *lt8618_iot = file->private_data;
	int r = 0;

	switch (cmd) {

	case LT8618_GET_EDID_RESOLUTION_RATIO: {
		hobot_lt8618_sync_t sync_t;
		printk("LT8618_GET_EDID_RESOLUTION_RATIO\n");
		r = LT8618SXB_Read_EDID(&sync_t);
		if(r!=0){
			r = -EFAULT;
		}
		if (copy_to_user((void __user *)arg, &sync_t, sizeof(hobot_lt8618_sync_t)))
			r = -EFAULT;
		break;
	}
	case LT8618_SET_RESOLUTION_RATIO: {
		if (copy_from_user(&lt8618_iot->ratio, (void __user *)arg, sizeof(int))) {
			r = -EFAULT;
		    break;
		}
		printk("LT8618_SET_RESOLUTION_RATIO lt8618_iot->ratio = %d\n",lt8618_iot->ratio);
		Resolution_change(lt8618_iot->ratio);
        //r = pwm_config(fl_pwm->pwm, fl_pwm->config.duty_ns, fl_pwm->config.period_ns);
		break;
	}
	default:
		r = -ENOTTY;
		break;
	}

	return r;
}

static const struct file_operations lt8618_ioctl_fops = {
	.owner   = THIS_MODULE,
	.open    = hobot_lt8618_open,
	.release = hobot_lt8618_release,
    .unlocked_ioctl = lt8618_ioctl,
};

static struct miscdevice lt8618_ioctl_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "lt8618_ioctl",
	.fops	= &lt8618_ioctl_fops,
};



static int x2_lt8618sxb_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret = 0;
	
	pr_debug("x2 lt8618sxb probe start.\n");
	misc_register(&lt8618_ioctl_dev);
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;
	
	g_x2_lt8618sxb =
	    devm_kzalloc(&client->dev, sizeof(struct x2_lt8618sxb_s),
			 GFP_KERNEL);
	if (!g_x2_lt8618sxb)
		return -ENOMEM;
	g_x2_lt8618sxb->client = client;
	
	mutex_init(&g_x2_lt8618sxb->lt8618sxb_mutex);

	lt8618sxb_reset_gpio = devm_gpiod_get_optional(&client->dev, "rst", GPIOD_OUT_LOW);
	if (IS_ERR(lt8618sxb_reset_gpio)) {
		/* lt8618sxb_reset_gpio GPIO not available */
		pr_info("optional-gpio not found\n");
		goto err;
	}

	LT8618SXB_Reset();

	ret = LT8618SXB_Chip_ID();
	if (ret != 0) {
		pr_err("not found lt8618sxb device, exit probe!!!\n");
		goto err;
	}
#ifndef CONFIG_HOBOT_X3_UBUNTU
	display_type = HDMI_TYPE;
#endif

	i2c_set_clientdata(client, g_x2_lt8618sxb);

	client->flags = I2C_CLIENT_SCCB;
	LT8618SXB_DEBUG("chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	LT8618SX_Initial();
#ifndef CONFIG_HOBOT_X3_UBUNTU
	hdmi_register_config_callback(disp_config_hdmi);
	hdmi_register_stop_output_callback(reset_hdmi_converter);
#endif
	pr_debug("x2_lt8618sxb probe OK!!!\n");
	return 0;

err:
	if (g_x2_lt8618sxb) {
		devm_kfree(&client->dev, g_x2_lt8618sxb);
		g_x2_lt8618sxb = NULL;
	}
	return ret;
}

static int x2_lt8618sxb_remove(struct i2c_client *client)
{
	struct x2_lt8618sxb_s *x2_lt8618sxb = i2c_get_clientdata(client);
	misc_deregister(&lt8618_ioctl_dev);
	if (x2_lt8618sxb)
		devm_kfree(&client->dev, x2_lt8618sxb);
	return 0;
}

static const struct of_device_id x2_lt8618sxb_of_match[] = {
	{ .compatible = "lt,lt8618sxb", .data = NULL },
	{}
};

MODULE_DEVICE_TABLE(of, x2_lt8618sxb_of_match);

static struct i2c_driver x2_lt8618sxb_driver = {
	.driver = {
		   .name = "x2_lt8618sxb",
		   .of_match_table = x2_lt8618sxb_of_match,
		   },
	.probe = x2_lt8618sxb_probe,
	.remove = x2_lt8618sxb_remove,
};

module_i2c_driver(x2_lt8618sxb_driver);

MODULE_DESCRIPTION("x2_lt8618sxb_converter_driver");
MODULE_AUTHOR("GuoRui <rui.guo@hobot.cc>");
MODULE_LICENSE("GPL v2");
