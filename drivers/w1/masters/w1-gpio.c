/*
 * w1-gpio - GPIO w1 bus master driver
 *
 * Copyright (C) 2007 Ville Syrjala <syrjala@sci.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/w1-gpio.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/delay.h>

#include <linux/w1.h>

static void w1_delay(unsigned long tm)
{
	udelay(tm);
}
static u8 w1_gpio_set_pullup(void *data, int delay)
{
	struct w1_gpio_platform_data *pdata = data;

	if (delay) {
		pdata->pullup_duration = delay;
	} else {
		if (pdata->pullup_duration) {
			gpio_direction_output(pdata->pin, 1);

			msleep(pdata->pullup_duration);

			gpio_direction_input(pdata->pin);
		}
		pdata->pullup_duration = 0;
	}

	return 0;
}

static void w1_gpio_write_bit_dir(void *data, u8 bit)
{
	struct w1_gpio_platform_data *pdata = data;
	if (bit)
		gpio_direction_input(pdata->pin);
	else
		gpio_direction_output(pdata->pin, 0);
}

static void w1_gpio_write_bit_val(void *data, u8 bit)
{
	struct w1_gpio_platform_data *pdata = data;
	gpio_direction_output(pdata->pin, bit);
	gpio_set_value(pdata->pin, bit);
}

static u8 w1_gpio_read_bit_val(void *data)
{
	struct w1_gpio_platform_data *pdata = data;
	gpio_direction_input(pdata->pin);
	return gpio_get_value(pdata->pin) ? 1 : 0;
}

/* this useful when write value 1/0 by wilbur */
  /* type =1 ->write io; type =0 ->read io*/
static u8 w1_gpio_touch_bit(void *data, u8 bit, u8 type)
{
		//struct w1_gpio_platform_data *pdata = data;
		uint32_t ret = 0;
		//读
		if (type == 0x0) {
			//output
			w1_gpio_write_bit_val(data, 0);
			w1_delay(1);
			ret = w1_gpio_read_bit_val(data);
			w1_delay(13);
			return ret;
		}
		//写
		if (type == 0x1) {
			if (bit) {  /* write one */
				//output
				w1_gpio_write_bit_val(data, 0);
				w1_delay(1);
				//释放信号线，写1到slave
				w1_gpio_write_bit_val(data, 1);
				w1_delay(13); //slave读取1的时间
				return 0;
				} else {
					/* write zero */
					//一直将信号线拉低，写0；
					w1_gpio_write_bit_val(data, 0);
	//slave读取0的时间
					w1_delay(10);
	//将信号线恢复为1,等待下一个周期
					w1_gpio_write_bit_val(data, 1);
					w1_delay(6);
					return 0;
			}
		}
		return -1;
}

static void w1_gpio_write_byte(void *data, u8 byte)
{
	int i;

	for (i = 0; i < 8; ++i)
		w1_gpio_touch_bit(data, ((byte >> i) & 0x1), 1);
}

static u8 w1_gpio_read_byte(void *data)
{
	int i;
	u8 res = 0;

	for (i = 0; i < 8; ++i)
		res |= (w1_gpio_touch_bit(data, 1, 0) << i);
	return res;
}

static u8 w1_gpio_reset_bus(void *data)
{
		u8 result = 0x0;

		w1_gpio_write_bit_val(data, 0);//output
		 /* minimum 480, max ? us*/
		w1_delay(50);
		w1_gpio_write_bit_val(data, 1);//释放总线
		w1_delay(10);

		result = w1_gpio_read_bit_val(data);
		/* minimum 70 (above) + 430 = 500 us
		 * There aren't any timing requirements between a reset and
		 * the following transactions.  Sleeping is safe here.
		 */
		/* w1_delay(430); min required time */
		w1_delay(100);
		return result;
}

#if defined(CONFIG_OF)
static const struct of_device_id w1_gpio_dt_ids[] = {
	{ .compatible = "w1-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, w1_gpio_dt_ids);
#endif

static int w1_gpio_probe_dt(struct platform_device *pdev)
{
	struct w1_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	int gpio;
	int ret;
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	//if (of_get_property(np, "linux,open-drain", NULL))
		pdata->is_open_drain = 1;

	//gpio = of_get_gpio(np, 0);
	ret = of_property_read_u32(pdev->dev.of_node,
		"onewire_gpio", &gpio);
	if (ret) {
		if (gpio != -EPROBE_DEFER)
			dev_err(&pdev->dev,
					"Failed to parse gpio property for data pin (%d)\n",
					gpio);

		return gpio;
	}
	pdata->pin = gpio;

	gpio = of_get_gpio(np, 1);
	if (gpio == -EPROBE_DEFER)
		return gpio;
	/* ignore other errors as the pullup gpio is optional */
	pdata->ext_pullup_enable_pin = gpio;

	pdev->dev.platform_data = pdata;

	return 0;
}

static int w1_gpio_probe(struct platform_device *pdev)
{
	struct w1_bus_master *master;
	struct w1_gpio_platform_data *pdata;
	int err;
	if (of_have_populated_dt()) {
		err = w1_gpio_probe_dt(pdev);
		if (err < 0)
			return err;
	}

	pdata = dev_get_platdata(&pdev->dev);

	if (!pdata) {
		dev_err(&pdev->dev, "No configuration data\n");
		return -ENXIO;
	}

	master = devm_kzalloc(&pdev->dev, sizeof(struct w1_bus_master),
			GFP_KERNEL);
	if (!master) {
		dev_err(&pdev->dev, "Out of memory\n");
		return -ENOMEM;
	}

	err = devm_gpio_request(&pdev->dev, pdata->pin, "w1");
	if (err) {
		dev_err(&pdev->dev, "gpio_request (pin) failed\n");
		return err;
	}

	if (gpio_is_valid(pdata->ext_pullup_enable_pin)) {
		err = devm_gpio_request_one(&pdev->dev,
				pdata->ext_pullup_enable_pin, GPIOF_INIT_LOW,
				"w1 pullup");
		if (err < 0) {
			dev_err(&pdev->dev, "gpio_request_one failed\n");
			return err;
		}
	}
	master->reset_bus = w1_gpio_reset_bus;
	master->data = pdata;
	master->read_bit = w1_gpio_read_bit_val;
	master->read_byte = w1_gpio_read_byte;
	master->write_byte = w1_gpio_write_byte;
	if (pdata->is_open_drain) {
		//gpio_direction_output(pdata->pin, 1);
		gpio_direction_input(pdata->pin);
		master->write_bit = w1_gpio_write_bit_val;
	} else {
		gpio_direction_input(pdata->pin);
		master->write_bit = w1_gpio_write_bit_dir;
		master->set_pullup = w1_gpio_set_pullup;
	}

	err = w1_add_master_device(master);
	if (err) {
		dev_err(&pdev->dev, "w1_add_master device failed\n");
		return err;
	}

	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(1);

	if (gpio_is_valid(pdata->ext_pullup_enable_pin))
		gpio_set_value(pdata->ext_pullup_enable_pin, 1);

	platform_set_drvdata(pdev, master);

	return 0;
}


static int w1_gpio_remove(struct platform_device *pdev)
{
	struct w1_bus_master *master = platform_get_drvdata(pdev);
	struct w1_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);

	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(0);

	if (gpio_is_valid(pdata->ext_pullup_enable_pin))
		gpio_set_value(pdata->ext_pullup_enable_pin, 0);

	w1_remove_master_device(master);

	return 0;
}

static int __maybe_unused w1_gpio_suspend(struct device *dev)
{
	struct w1_gpio_platform_data *pdata = dev_get_platdata(dev);

	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(0);

	return 0;
}

static int __maybe_unused w1_gpio_resume(struct device *dev)
{
	struct w1_gpio_platform_data *pdata = dev_get_platdata(dev);

	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(1);

	return 0;
}

static SIMPLE_DEV_PM_OPS(w1_gpio_pm_ops, w1_gpio_suspend, w1_gpio_resume);

static struct platform_driver w1_gpio_driver = {
	.driver = {
		.name	= "w1-gpio",
		.pm	= &w1_gpio_pm_ops,
		.of_match_table = of_match_ptr(w1_gpio_dt_ids),
	},
	.probe = w1_gpio_probe,
	.remove = w1_gpio_remove,
};

module_platform_driver(w1_gpio_driver);

MODULE_DESCRIPTION("GPIO w1 bus master driver");
MODULE_AUTHOR("Ville Syrjala <syrjala@sci.fi>");
MODULE_LICENSE("GPL");
