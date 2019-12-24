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

#include "inc/camera_dev.h"
#include "inc/camera_subdev.h"
#include "inc/camera_i2c.h"
#include "inc/camera_sys_api.h"

#define ARGS_TO_PTR(arg) ((struct sensor_arg *)arg)

/* global variable define */
typedef struct _camera_subdev_ctx {
	struct v4l2_subdev camera;
} camera_subdev_ctx;

static camera_subdev_ctx *camera_ctx;

static int camera_subdev_status(struct v4l2_subdev *sd)
{
	pr_info("log status called");
	return 0;
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
		ret = camera_sys_priv_set(ARGS_TO_PTR(arg)->port,
				ARGS_TO_PTR(arg)->sensor_priv);
		if(ret < 0) {
			pr_err("SENSOR_UPDATE error port %d\n", ARGS_TO_PTR(arg)->port);
		}
		break;
	case SENSOR_GET_PARAM:
		camera_sys_get_param(ARGS_TO_PTR(arg)->port,
				ARGS_TO_PTR(arg)->sensor_data);
		break;
	case SENSOR_ALLOC_ANALOG_GAIN:
		pr_err("analog gain error port %p\n", ARGS_TO_PTR(arg)->a_gain);
	//	camera_sys_alloc_again(ARGS_TO_PTR(arg)->port, ARGS_TO_PTR(arg)->a_gain);
		break;
	case SENSOR_ALLOC_DIGITAL_GAIN:
		pr_err("digaital gain error port %p\n", ARGS_TO_PTR(arg)->d_gain);
	//	camera_sys_alloc_dgain(ARGS_TO_PTR(arg)->port, ARGS_TO_PTR(arg)->d_gain);
		break;
	case SENSOR_ALLOC_INTEGRATION_TIME:
	//	camera_sys_alloc_intergration_time(ARGS_TO_PTR(arg)->port,
	//			ARGS_TO_PTR(arg)->integration_time);
		break;
	default:
		pr_err("Unknown lens ioctl cmd %d", cmd);
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

	pr_info("v4l2_subdev_init\n");
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

	pr_info("register v4l2 lens device. result %d", ret);
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

	pr_info("[KeyMsg] camera subdevice init");

	camera_subdev = platform_device_register_simple(
		"camera_v4l2", -1, NULL, 0);
	ret = platform_driver_register(&camera_subdev_driver);
	if(ret < 0) {
		goto init_err;
	}
	pr_info("%s --%d, platform_driver_register\n",
		__func__, __LINE__);
	camera_cdev_init();
	pr_info("[KeyMsg] camera subdevice init done: %d", ret);
	return ret;
init_err:

	platform_driver_unregister(&camera_subdev_driver);
	platform_device_unregister(camera_subdev);
	return ret;
}

void __exit camera_subdev_driver_exit(void)
{
	pr_info("[KeyMsg] camera subdevice exit");

	camera_cdev_exit();
	platform_driver_unregister(&camera_subdev_driver);
	platform_device_unregister(camera_subdev);
	pr_info("[KeyMsg] camera subdevice exit done");
}

module_init(camera_subdev_driver_init);
module_exit(camera_subdev_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Horizon Inc.");

