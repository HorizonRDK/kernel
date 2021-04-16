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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

#define EMPTY_DEFAULT_ADVALUE		1000
#define DRIFT_DEFAULT_ADVALUE		50
#define INVALID_ADVALUE			-1

#if 1
#define key_dbg(bdata, format, arg...)		\
	dev_info(&bdata->input->dev, format, ##arg)
#else
#define key_dbg(bdata, format, arg...)
#endif

#define DEBOUNCE_JIFFIES	(10 / (MSEC_PER_SEC / HZ))	/* 10ms */
#define ADC_SAMPLE_JIFFIES	(100 / (MSEC_PER_SEC / HZ))	/* 100ms */

struct hobot_keys_button {
	struct device *dev;
	u32 code;		/* key code */
	const char *desc;	/* key label */
	u32 state;		/* key up & down state */
	int adc_value;		/* adc only */
	int adc_state;		/* adc only */
	struct timer_list timer;
};

struct hobot_keys_drvdata {
	int nbuttons;
	bool in_suspend;
	int result;
	int rep;
	int drift_advalue;
	struct input_dev *input;
	struct delayed_work adc_poll_work;
	struct iio_channel *chan;
	struct hobot_keys_button *button;
};

static struct input_dev *sinput_dev;

static int hobot_key_adc_iio_read(struct hobot_keys_drvdata *data)
{
	struct iio_channel *channel = data->chan;
	int val, ret;

	if (!channel)
		return INVALID_ADVALUE;
	ret = iio_read_channel_raw(channel, &val);
	if (ret < 0) {
		pr_err("read channel() error: %d\n", ret);
		return ret;
	}
	return val / 1000;
}

static void adc_key_poll(struct work_struct *work)
{
	struct hobot_keys_drvdata *ddata;
	int i, result = -1;

	ddata = container_of(work, struct hobot_keys_drvdata, adc_poll_work.work);
	if (!ddata->in_suspend) {
		result = hobot_key_adc_iio_read(ddata);

		if (result > INVALID_ADVALUE &&
		    result < (EMPTY_DEFAULT_ADVALUE - ddata->drift_advalue))
			ddata->result = result;
		for (i = 0; i < ddata->nbuttons; i++) {
			struct hobot_keys_button *button = &ddata->button[i];

			if (!button->adc_value)
				continue;
			if (result < button->adc_value + ddata->drift_advalue &&
			    result > button->adc_value - ddata->drift_advalue)
				button->adc_state = 1;
			else
				button->adc_state = 0;
			if (button->state != button->adc_state)
				mod_timer(&button->timer, jiffies + DEBOUNCE_JIFFIES);
		}
	}

	schedule_delayed_work(&ddata->adc_poll_work, ADC_SAMPLE_JIFFIES);
}

static void keys_timer(unsigned long _data)
{
	struct hobot_keys_button *button = (struct hobot_keys_button *)_data;
	struct hobot_keys_drvdata *pdata = dev_get_drvdata(button->dev);
	struct input_dev *input = pdata->input;
	int state;

	state = !!button->adc_state;

	if (button->state != state) {
		button->state = state;
		input_event(input, EV_KEY, button->code, button->state);
		key_dbg(pdata, "key[%s]: report event[%d] state[%d]\n",
			button->desc, button->code, button->state);
		input_event(input, EV_KEY, button->code, button->state);
		input_sync(input);
	}

	if (state)
		mod_timer(&button->timer, jiffies + DEBOUNCE_JIFFIES);
}

static int hobot_keys_parse_dt(struct hobot_keys_drvdata *pdata,
				struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child_node;
	struct iio_channel *chan;
	int ret, i = 0;
	u32 code, adc_value, drift;

	if (of_property_read_u32(node, "adc-drift", &drift))
		pdata->drift_advalue = DRIFT_DEFAULT_ADVALUE;
	else
		pdata->drift_advalue = (int)drift;

	chan = iio_channel_get(&pdev->dev, NULL);
	if (IS_ERR(chan)) {
		dev_info(&pdev->dev, "no io-channels defined\n");
		chan = NULL;
		return -ENAVAIL;
	}
	pdata->chan = chan;

	for_each_child_of_node(node, child_node) {
		if (of_property_read_u32(child_node, "linux,code", &code)) {
			dev_err(&pdev->dev, "Missing linux,code property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}

		pdata->button[i].code = code;
		pdata->button[i].desc = of_get_property(child_node, "label", NULL);
		if (of_property_read_u32(child_node, "hobot,adc_value", &adc_value)) {
			dev_err(&pdev->dev,
				"Missing hobot,adc_value property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}
		pdata->button[i].adc_value = adc_value;
		i++;
	}

	return 0;

error_ret:
	return ret;
}

static int keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct hobot_keys_drvdata *ddata = NULL;
	struct hobot_keys_button *button = NULL;
	struct input_dev *input = NULL;
	int i, error = 0;
	int key_num = 0;

	key_num = of_get_child_count(np);
	if (key_num == 0)
		dev_info(dev, "no key defined\n");

	ddata = devm_kzalloc(dev, sizeof(struct hobot_keys_drvdata), GFP_KERNEL);

	button = devm_kzalloc(dev, sizeof(struct hobot_keys_button), GFP_KERNEL);

	input = devm_input_allocate_device(dev);
	if (!ddata || !input || !button) {
		error = -ENOMEM;
		return error;
	}

	platform_set_drvdata(pdev, ddata);
	dev_set_drvdata(&pdev->dev, ddata);
	input->name = "hobot-keypad";
	input->phys = "gpio-keys/input0";
	input->dev.parent = dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	ddata->input = input;
	ddata->button = button;

	/* parse info from dt */
	ddata->nbuttons = key_num;
	dev_info(dev, "key_num=%d", key_num);
	error = hobot_keys_parse_dt(ddata, pdev);
	if (error)
		goto fail0;

	/* Enable auto repeat feature of Linux input subsystem */
	if (ddata->rep)
		__set_bit(EV_REP, input->evbit);

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, error: %d\n",
		       error);
		goto fail0;
	}
	sinput_dev = input;

	for (i = 0; i < ddata->nbuttons; i++) {
		struct hobot_keys_button *button = &ddata->button[i];

		if (button->code) {
			setup_timer(&button->timer, keys_timer, (unsigned long)button);
		}
		input_set_capability(input, EV_KEY, button->code);
	}
	for (i = 0; i < ddata->nbuttons; i++) {
		ddata->button[i].dev = &pdev->dev;
	}

	input_set_capability(input, EV_KEY, KEY_WAKEUP);
	/* adc polling work */
	if (ddata->chan) {
		INIT_DELAYED_WORK(&ddata->adc_poll_work, adc_key_poll);
		schedule_delayed_work(&ddata->adc_poll_work, ADC_SAMPLE_JIFFIES);
	}

	return error;
fail0:
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int keys_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hobot_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(dev, 0);
	for (i = 0; i < ddata->nbuttons; i++)
		del_timer_sync(&ddata->button[i].timer);
	if (ddata->chan)
		cancel_delayed_work_sync(&ddata->adc_poll_work);
	input_unregister_device(input);

	sinput_dev = NULL;

	return 0;
}

static const struct of_device_id hobot_key_match[] = {
	{ .compatible = "hobot,hobot-adc-key" },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, pvt_of_match);

static struct platform_driver keys_device_driver = {
	.probe      = keys_probe,
	.remove     = keys_remove,
	.driver     = {
		.name   = "hb-keypad",
		.owner  = THIS_MODULE,
		.of_match_table = hobot_key_match,
	},
};

static int __init hobot_keys_driver_init(void)
{
	return platform_driver_register(&keys_device_driver);
}

static void __exit hobot_keys_driver_exit(void)
{
	platform_driver_unregister(&keys_device_driver);
}

late_initcall_sync(hobot_keys_driver_init);
module_exit(hobot_keys_driver_exit);

MODULE_AUTHOR("Hobot, Inc.");
MODULE_DESCRIPTION("Hobot keys driver");
MODULE_LICENSE("GPL v2");
