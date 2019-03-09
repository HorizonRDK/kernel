/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>

#include "x2_lt9211.h"

int lt9211_reset_pin;
int lcd_reset_pin;

struct pwm_device *lcd_backlight_pwm;

struct x2_lt9211_s *g_x2_lt9211;


static int lt9211_open(struct inode *inode, struct file *filp)
{
	struct x2_lt9211_s  *lt9211_p;

	lt9211_p = container_of(inode->i_cdev, struct x2_lt9211_s, cdev);
	filp->private_data = lt9211_p;
	return 0;
}

static long lt9211_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
{
	int ret = 0;
	unsigned long on_time = 0;

	ret = copy_from_user((void *)&on_time, (void __user *)p,
			sizeof(unsigned long));
	if (ret) {
		pr_err("%s copy data from user failed %d\n",
				__func__, ret);
		return -EINVAL;
	}
	if (on_time > 20000000 || on_time == 0) {
		pr_err("LCD back light debug: duty time exceed the max value!!\n");
		return -EINVAL;
	}

	pwm_disable(lcd_backlight_pwm);

	ret = pwm_config(lcd_backlight_pwm, on_time, 20000000); // 0.5ms
	if (ret) {
		pr_err("\nError config pwm!!!!\n");
		return ret;
	}
	ret = pwm_set_polarity(lcd_backlight_pwm, PWM_POLARITY_NORMAL);
	if (ret) {
		pr_err("\nError set pwm polarity!!!!\n");
		return ret;
	}
	ret = pwm_enable(lcd_backlight_pwm);
	if (ret) {
		pr_err("\nError enable pwm!!!!\n");
		return ret;
	}
	return 0;
}

static ssize_t lt9211_write(struct file *filp, const char __user *ubuf,
		size_t len, loff_t *ppos)
{
	return len;
}


static ssize_t lt9211_read(struct file *filp, char __user *ubuf,
		size_t len, loff_t *offp)
{
	unsigned int size;

	size = 0;
	return size;
}


int lt9211_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}


static int lcd_backlight_init(void)
{
	int ret = 0;

	lcd_backlight_pwm = pwm_request(0, "lcd-pwm");
	if (lcd_backlight_pwm == NULL) {
		pr_err("\nNo pwm device 0!!!!\n");
		return -ENODEV;
	}
	/**
	 * pwm_config(struct pwm_device *pwm, int duty_ns,
			     int period_ns) - change a PWM device configuration
	 * @pwm: PWM device
	 * @duty_ns: "on" time (in nanoseconds)
	 * @period_ns: duration (in nanoseconds) of one cycle
	 *
	 * Returns: 0 on success or a negative error code on failure.
	 */
	ret = pwm_config(lcd_backlight_pwm, 20000000, 20000000); // 0.5ms
	if (ret) {
		pr_err("\nError config pwm!!!!\n");
		return ret;
	}
	ret = pwm_set_polarity(lcd_backlight_pwm, PWM_POLARITY_NORMAL);
	if (ret) {
		pr_err("\nError set pwm polarity!!!!\n");
		return ret;
	}
	ret = pwm_enable(lcd_backlight_pwm);
	if (ret) {
		pr_err("\nError enable pwm!!!!\n");
		return ret;
	}
	return 0;
}

static const struct file_operations x2_lt9211_fops = {
	.owner	= THIS_MODULE,
	.open	= lt9211_open,
	.read	= lt9211_read,
	.write	= lt9211_write,
	.release	= lt9211_release,
	.unlocked_ioctl	= lt9211_ioctl,
};


static int x2_lt9211_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	struct device *x2_lt9211_dev;
	dev_t devno;
	int ret = 0;
	unsigned int convert_type = BT1120_TO_RGB888;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	g_x2_lt9211 = devm_kzalloc(&client->dev, sizeof(struct x2_lt9211_s),
			GFP_KERNEL);
	if (!g_x2_lt9211)
		return -ENOMEM;
	g_x2_lt9211->client = client;

	ret = alloc_chrdev_region(&devno, 0, 1, "x2_lt9211");
	if (ret < 0) {
		pr_err("Error %d while alloc chrdev x2_fpgactrl", ret);
		goto err;
	}
	g_x2_lt9211->dev_num = devno;
	g_x2_lt9211->name = "x2_lt9211";

	g_x2_lt9211->major = MAJOR(devno);
	g_x2_lt9211->minor = MINOR(devno);

	cdev_init(&(g_x2_lt9211->cdev), &x2_lt9211_fops);
	g_x2_lt9211->cdev.owner = THIS_MODULE;
	ret = cdev_add(&(g_x2_lt9211->cdev), devno, 1);
	if (ret) {
		pr_err("Error %d while adding x2 fpgactrl cdev", ret);
		goto err;
	}

	g_x2_lt9211->x2_lt9211_classes = class_create(THIS_MODULE, "x2_lt9211");
	if (IS_ERR(g_x2_lt9211->x2_lt9211_classes)) {
		pr_err("[%s:%d] class_create error\n",
				__func__, __LINE__);
		ret = PTR_ERR(g_x2_lt9211->x2_lt9211_classes);
		goto err;
	}
	x2_lt9211_dev = device_create(g_x2_lt9211->x2_lt9211_classes,
			NULL, MKDEV(g_x2_lt9211->major, 0),
			NULL, g_x2_lt9211->name);
	if (IS_ERR(x2_lt9211_dev)) {
		pr_err("[%s] deivce create error\n", __func__);
		ret = PTR_ERR(x2_lt9211_dev);
		goto err;
	}

	ret = of_property_read_u32(x2_lt9211_dev->of_node,
			"x2_lt9211_reset_pin", &lt9211_reset_pin);
	if (ret)
		dev_err(x2_lt9211_dev, "Filed to get x2_lt9211_reset_pin\n");
	ret = of_property_read_u32(x2_lt9211_dev->of_node,
			"x2_lcd_reset_pin", &lcd_reset_pin);
	if (ret)
		dev_err(x2_lt9211_dev, "Filed to get x2_lcd_reset_pin\n");
//	ret = of_property_read_u32(x2_lt9211_dev->of_node,
//	"x2_lcd_pwm_pin", &lcd_pwm_pin);
//	if (ret) {
//		dev_err(x2_lt9211_dev, "Filed to get x2_lcd_pwm_pin\n");
//	}

	i2c_set_clientdata(client, g_x2_lt9211);

	client->flags = I2C_CLIENT_SCCB;
	pr_info("chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);
	ret = lcd_backlight_init();
	if (ret)
		pr_info("\nlcd backlight init err!\n");

	ret = lt9211_dsi_lcd_init(convert_type);
	if (ret)
		pr_info("\nlt9211 and dsi panel init err!\n");

	pr_info("x2_lt9211 probe OK!!!\n");
	return 0;
err:
	class_destroy(g_x2_lt9211->x2_lt9211_classes);
	cdev_del(&g_x2_lt9211->cdev);
	unregister_chrdev_region(MKDEV(g_x2_lt9211->major, 0), 1);
	if (g_x2_lt9211)
		devm_kfree(&client->dev, g_x2_lt9211);
	return ret;
}

static int x2_lt9211_remove(struct i2c_client *client)
{
	struct x2_lt9211_s *x2_lt9211 = i2c_get_clientdata(client);

	class_destroy(x2_lt9211->x2_lt9211_classes);
	cdev_del(&x2_lt9211->cdev);
	unregister_chrdev_region(MKDEV(g_x2_lt9211->major, 0), 1);
	if (x2_lt9211)
		devm_kfree(&client->dev, x2_lt9211);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id x2_lt9211_of_match[] = {
	{ .compatible = "x2,lt9211", .data = NULL },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, x2_lt9211_of_match);
#endif

static const struct i2c_device_id x2_lt9211_id[] = {
	{"x2_lt9211", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, x2_lt9211_id);

static struct i2c_driver x2_lt9211_driver = {
	.driver = {
		.name = "x2_lt9211",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(x2_lt9211_of_match),
#endif
	},
	.probe = x2_lt9211_probe,
	.remove = x2_lt9211_remove,
	.id_table = x2_lt9211_id,
};
module_i2c_driver(x2_lt9211_driver);

MODULE_DESCRIPTION("x2_lt9211_converter_driver");
MODULE_AUTHOR("GuoRui <rui.guo@hobot.cc>");
MODULE_LICENSE("GPL");
