/*   Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/dmaengine.h>
#include <linux/compiler.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/mman.h>
#include <linux/device.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "inc/camera_dev.h"
#include "inc/camera_ctrl.h"
#include "inc/camera_subdev.h"
#include "inc/camera_i2c.h"
#include "inc/camera_sys_api.h"

#define ARGS_TO_PTR(arg) ((struct sensor_arg *)arg)

/* global variable define */
typedef struct _camera_subdev_ctx {
	struct v4l2_subdev camera;
} camera_subdev_ctx;

static camera_subdev_ctx *camera_ctx;
static event_header_t event_header;

static int camera_gpio_request(u32 gpio)
{
	int ret = 0;
	char gpio_name[CAMERA_GPIO_NAME_LENGTH];

	memset(gpio_name, 0, CAMERA_GPIO_NAME_LENGTH);
	snprintf(gpio_name, CAMERA_GPIO_NAME_LENGTH, "gpio_%d", gpio);

	pr_info("gpio %d gpio_name %s-----\n", gpio, gpio_name);
	ret = gpio_request(gpio, gpio_name);
	if (ret < 0) {
		pr_err("gpio %d request failed!", gpio);
		goto err_request;
	}
	ret = gpio_direction_output(gpio, 0);
	if (ret < 0) {
		pr_err("gpio %d set direction failed!", gpio);
		gpio_free(gpio);
		goto err_set;
	}
	return ret;
err_set:
	gpio_free(gpio);
err_request:
	return ret;
}

int camera_gpio_info_config(gpio_info_t *gpio_info)
{
	int ret = 0;
	u32 gpio = gpio_info->gpio;
	u32 gpio_level = gpio_info->gpio_level;

	ret = camera_gpio_request(gpio);
	if(ret < 0) {
		pr_err("line %d gpio_request error gpio %d\n",
				__LINE__, gpio);
		return -1;
	}
	if(GPIO_HIGH == gpio_level) {
		gpio_set_value(gpio, 1);
	} else {
		gpio_set_value(gpio, 0);
	}
	mdelay(5);
	gpio_free(gpio);
	return ret;
}

static int camera_subdev_status(struct v4l2_subdev *sd)
{
	pr_info("log status called");
	return 0;
}

static void write_sensor_work(struct work_struct *data)
{
	int ret = 0;
	LIST_HEAD(event_list);
	event_node_t *event_p, *event_tmp;
	spin_lock(&event_header.lock);
	list_splice_init(&event_header.list_busy, &event_list);
	spin_unlock(&event_header.lock);
	list_for_each_entry_safe(event_p, event_tmp, &event_list, list_node) {
		if (event_p->cmd == SENSOR_UPDATE) {
			ret = camera_sys_priv_set(event_p->port,
				&event_p->priv_param);
			if(ret < 0) {
			pr_err("SENSOR_UPDATE error port%d\n", event_p->port);
			}
		} else if (event_p->cmd == SENSOR_AWB_UPDATE) {
			ret = camera_sys_priv_awb_set(event_p->port,
				&event_p->priv_param);
			if(ret < 0) {
			pr_err("SENSOR_AWB_UPDATE error port%d\n", event_p->port);
			}
		} else {
			pr_err("unknown cmd %d\n", event_p->cmd);
		}
	}
	spin_lock(&event_header.lock);
	list_splice_init(&event_list, &event_header.list_free);
	spin_unlock(&event_header.lock);
}

static void cmd_add_to_work(uint32_t cmd, void *arg)
{
	struct list_head *list;
	event_node_t *event_p;
	int port;

	port = ARGS_TO_PTR(arg)->port;
	if(camera_mod[port]->write_flag == 1) {
		pr_err("port %d sensor sts checking now, drop write!\n", port);
		return;
	}
	spin_lock(&event_header.lock);
	if (!list_empty(&event_header.list_free)) {
		list = event_header.list_free.next;
		list_del(list);
		event_p = list_entry(list, event_node_t, list_node);
		event_p->port = ARGS_TO_PTR(arg)->port;
		event_p->cmd = cmd;
		memcpy(&event_p->priv_param,
				ARGS_TO_PTR(arg)->sensor_priv,
				sizeof(sensor_priv_t));
		list_add(list, &event_header.list_busy);
	}
	spin_unlock(&event_header.lock);
	schedule_work(&event_header.updata_work);
}

static long camera_subdev_ioctl(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	int ret = 0;
	if (ARGS_TO_PTR(arg)->port >= CAMERA_TOTAL_NUMBER) {
		pr_err("Failed to control dwe_ioctl :%d\n", ARGS_TO_PTR(arg)->port);
		return -1;
	}
	switch (cmd) {
	case SENSOR_UPDATE:
		// ctrl by user
		set_sensor_aexp_info(ARGS_TO_PTR(arg)->port, ARGS_TO_PTR(arg)->sensor_priv);
		sensor_ctrl_wakeup(ARGS_TO_PTR(arg)->port);
		// ctrl by user end
		cmd_add_to_work(cmd, arg);
		break;
	case SENSOR_GET_PARAM:
		ret = camera_sys_get_param(ARGS_TO_PTR(arg)->port,
				ARGS_TO_PTR(arg)->sensor_data);
		if(ret < 0) {
		   pr_err("camera_sys_get_param error port%d\n", ARGS_TO_PTR(arg)->port);
		}
		break;
	case SENSOR_STREAM_ON:
		ret = camera_sys_stream_on(ARGS_TO_PTR(arg)->port);
		if(ret < 0) {
		    pr_err("camera_sys_stream_on error port%d\n", ARGS_TO_PTR(arg)->port);
		}
		break;
	case SENSOR_STREAM_OFF:
		ret = camera_sys_stream_off(ARGS_TO_PTR(arg)->port);
		if(ret < 0) {
		    pr_err("camera_sys_stream_off error port%d\n", ARGS_TO_PTR(arg)->port);
		}
		break;
	case SENSOR_WRITE:
		ret = camera_sys_sensor_write(ARGS_TO_PTR(arg)->port,
                        ARGS_TO_PTR(arg)->address,
                        ARGS_TO_PTR(arg)->w_data);
		if(ret < 0) {
		    pr_err("camera_sys_sensor_write error port%d address 0x%x\n",
				ARGS_TO_PTR(arg)->port, ARGS_TO_PTR(arg)->address);
		}
		break;
	case SENSOR_READ:
		ret = camera_sys_sensor_read(ARGS_TO_PTR(arg)->port,
                        ARGS_TO_PTR(arg)->address,
                        ARGS_TO_PTR(arg)->r_data);
		if(ret < 0) {
		    pr_err("camera_sys_sensor_read error port%d, address 0x%x\n",
				ARGS_TO_PTR(arg)->port, ARGS_TO_PTR(arg)->address);
		}
		break;
	case SENSOR_ALLOC_ANALOG_GAIN:
		/* ret = camera_sys_alloc_again(ARGS_TO_PTR(arg)->port, ARGS_TO_PTR(arg)->a_gain);
		if(ret < 0) {
		    pr_err("camera_sys_alloc_again error port%d\n", ARGS_TO_PTR(arg)->port);
		} */
		break;
	case SENSOR_ALLOC_DIGITAL_GAIN:
		/* ret = camera_sys_alloc_dgain(ARGS_TO_PTR(arg)->port, ARGS_TO_PTR(arg)->d_gain);
		if(ret < 0) {
		    pr_err("camera_sys_alloc_dgain error port%d\n", ARGS_TO_PTR(arg)->port);
		} */
		break;
	case SENSOR_ALLOC_INTEGRATION_TIME:
		//camera_sys_alloc_intergration_time(ARGS_TO_PTR(arg)->port,
		//		ARGS_TO_PTR(arg)->integration_time);
		break;
	case SENSOR_AWB_UPDATE:
		cmd_add_to_work(cmd, arg);
		break;
	default:
		pr_err("Unknown camera_subdev_ioctl cmd %d", cmd);
		ret = -1;
		break;
	}
	return ret;
}

static int camera_subdev_init(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;

	if (val < CAMERA_TOTAL_NUMBER) {
		return ret;
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int camera_subdev_reset(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;
	if (val < CAMERA_TOTAL_NUMBER) {
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static const struct v4l2_subdev_core_ops core_ops = {
	.log_status = camera_subdev_status,
	.init = camera_subdev_init,
	.reset = camera_subdev_reset,
	.ioctl = camera_subdev_ioctl,
};

static const struct v4l2_subdev_ops camera_ops = {
	.core = &core_ops,
};

static int32_t camera_subdev_probe(struct platform_device *pdev)
{
	int32_t ret = 0;
	int i, event_size;
	event_node_t *event_arr;
	event_size = 2 * CAMERA_TOTAL_NUMBER * sizeof(event_node_t);
	event_arr = devm_kmalloc(&pdev->dev, event_size, GFP_KERNEL);
	if (event_arr == NULL) {
		pr_err("kzalloc event_node failed!");
		return -ENOMEM;
	}

	camera_ctx = kzalloc(sizeof(camera_subdev_ctx), GFP_KERNEL);
	if (camera_ctx == NULL) {
		pr_err("kzalloc is failed!");
		return -ENOMEM;
	}
	v4l2_subdev_init(&camera_ctx->camera, &camera_ops);
	camera_ctx->camera.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	snprintf(camera_ctx->camera.name, V4L2_SUBDEV_NAME_SIZE,
		"%s", V4L2_CAMERA_NAME);

	camera_ctx->camera.dev = &pdev->dev;
	ret = v4l2_async_register_subdev(&camera_ctx->camera);

	pr_info("register v4l2 lens device. result%d", ret);

	INIT_LIST_HEAD(&event_header.list_free);
	INIT_LIST_HEAD(&event_header.list_busy);
	spin_lock_init(&event_header.lock);
	INIT_WORK(&event_header.updata_work, write_sensor_work);
	for (i = 0; i < 2 * CAMERA_TOTAL_NUMBER; i++)
		list_add(&event_arr[i].list_node, &event_header.list_free);
	return ret;
}

static int camera_subdev_remove(struct platform_device *pdev)
{
	v4l2_async_unregister_subdev(&camera_ctx->camera);

	return 0;
}

static struct platform_device *camera_subdev;

static struct platform_driver camera_subdev_driver = {
	.probe = camera_subdev_probe,
	.remove = camera_subdev_remove,
	.driver = {
		.name = "camera_v4l2",
		.owner = THIS_MODULE,
	},
};

int __init camera_subdev_driver_init(void)
{
	int ret = 0;

	camera_subdev = platform_device_register_simple(
		"camera_v4l2", -1, NULL, 0);
	ret = platform_driver_register(&camera_subdev_driver);
	if(ret < 0) {
		goto init_err;
	}
	camera_cdev_init();
	camera_ctrldev_init();
	return ret;
init_err:

	platform_driver_unregister(&camera_subdev_driver);
	platform_device_unregister(camera_subdev);
	return ret;
}

void __exit camera_subdev_driver_exit(void)
{
	camera_cdev_exit();
	camera_ctrldev_exit();
	platform_driver_unregister(&camera_subdev_driver);
	platform_device_unregister(camera_subdev);
	pr_info("[KeyMsg] camera subdevice exit done");
}

late_initcall(camera_subdev_driver_init);
module_exit(camera_subdev_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Horizon Inc.");

