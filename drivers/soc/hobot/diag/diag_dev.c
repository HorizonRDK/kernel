/*
 * Copyright (C) 2018-2020 Horizon Robotics Co., Ltd.
 *
 * fchm_driver.h
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

/* PRQA S ALL ++ */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/param.h>
#include <linux/random.h>
#include "diag_dev.h"
#include "diag_drv.h"
/* PRQA S ALL -- */

static struct class  *g_diag_dev_class;
static int32_t diag_app_ready;

#define SELFTEST_MIN_MS 20
#define SELFTEST_MAX_MS 100

#if 0
ssize_t test_enable_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return strlen(buf);
}

static void enable_callback(uint16_t module_id, uint16_t event_id)
{
	struct diag_module *module = NULL;
	struct diag_event event;
	uint8_t *paylod = NULL;
	int32_t event_idx = -1;

	event.module_id = module_id;
	event.event_id = event_id;
	module = is_module_event_registered(event, &event_idx);
	if (module == NULL) {
		pr_err("module not registered: module_id = %d, event_id = %d\n",
				event.module_id, event.event_id);
		return;
	}
	if (module->event_id[event_idx].id_handle.cb)
		module->event_id[event_idx].id_handle.cb(paylod);
	else
		pr_debug("module_id:%d, event_id:%d callback is NULL\n",
				event.module_id, event.event_id);
}

ssize_t test_enable_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	enable_callback(ModuleDiag_eth, EventIdEthDmaBusErr);
	return count;
}

static struct class_attribute test_attribute =
	__ATTR(test_enable, 0644, test_enable_show, test_enable_store);

static struct attribute *diag_attributes[] = {
	&test_attribute.attr,
	NULL
};

static const struct attribute_group diag_group = {
	.attrs = diag_attributes,
};

static const struct attribute_group *diag_attr_group[] = {
	&diag_group,
	NULL,
};

static struct class diag_class = {
	.name = "diag_test",
	.class_groups = diag_attr_group,
};
#endif

/*
 * diag driver or diag app is ready?
 * 1:ready, 0:not
 */
int32_t diag_is_ready(void)
{
	if (diag_app_ready == 0) {
		return 0; // diag core had not init or diag app not ready.
	}
	return 1;
}

static int32_t diag_dev_open(struct inode *inode, struct file *file)	/* PRQA S 3206 */
{
	return 0;
}

static DEFINE_MUTEX(diag_dev_ioctl_mutex); /* PRQA S ALL */
static long diag_dev_ioctl(struct file *file, uint32_t cmd, unsigned long arg) /* PRQA S ALL */
{
	struct diag_event event = {0};
	struct diag_register_info register_info;
	int32_t ret = 0;
	uint8_t op;

	register_info.module_id = (uint8_t)ModuleDiagDriver;
	register_info.event_cnt = 1;
	register_info.event_handle[0].event_id = (uint8_t)EventIdKernelToUserSelfTest;
	register_info.event_handle[0].min_snd_ms = SELFTEST_MIN_MS;
	register_info.event_handle[0].max_snd_ms = SELFTEST_MAX_MS;
	register_info.event_handle[0].cb = NULL;

	mutex_lock(&diag_dev_ioctl_mutex);
	switch (cmd) {
	case IOCS_DIAG_DEV_SELF_TEST:
			ret = diagnose_register(&register_info);
			if (ret < 0) {
				break;
			}

			event.module_id = (uint16_t)ModuleDiagDriver;
			event.event_id = (uint16_t)EventIdKernelToUserSelfTest;
			event.event_prio = (uint8_t)DiagMsgPrioLow;
			event.event_sta = (uint8_t)DiagEventStaSuccess;

			ret = diagnose_send_event(&event);
			if (ret < 0) {
				break;
			}
			ret = diag_event_unregister((uint8_t)ModuleDiagDriver,
							EventIdKernelToUserSelfTest);
			if (ret < 0) {
				break;
			}
		break;
	case IOCS_DIAGDRIVER_STA:
		if (copy_from_user((uint8_t *)&op, (uint8_t __user *)arg, 1)) { /* PRQA S ALL */
			pr_err("diag dev: IOCS_DIAG_STATUS copy from user error\n");	/* PRQA S ALL */
			ret = -EINVAL;
			break;
		}
		if (op == (uint8_t)DIAGDRIVER_STOPWORK) {
			diag_app_ready = 0;
		} else if (op == (uint8_t)DIAGDRIVER_STARTWORK) {
			diag_app_ready = 1;
		} else { /* to do */}
		break;

	default: /* PRQA S 2020 */
		ret = -EINVAL;
	}

	mutex_unlock(&diag_dev_ioctl_mutex);
	return ret;
}

static int32_t diag_dev_close(struct inode *inode, struct file *file) /* PRQA S ALL */
{
	return 0;
}

static const  struct file_operations diag_dev_fops = { /* PRQA S 3218 */
	.owner		=	THIS_MODULE,
	.open		=	diag_dev_open,
	.release	=	diag_dev_close,
	.unlocked_ioctl	=	diag_dev_ioctl,
	.compat_ioctl	=	diag_dev_ioctl,
};

static int32_t diag_dev_major;
static struct cdev diag_dev_cdev;
int32_t __init diag_dev_init(void)
{
	int32_t diag_dev_ver[2] = {1, 1};
	int32_t	ret;
	dev_t	devno;
	struct cdev  *p_cdev = &diag_dev_cdev;
	struct device *g_diag_dev;

	diag_dev_major = 0;
	pr_debug("diag dev init enter\n"); /* PRQA S ALL */
	ret = alloc_chrdev_region(&devno, 0, 1, "diag dev");
	if (ret < 0) {
		pr_err("Error %d while alloc chrdev diag dev", ret); /* PRQA S 3200 */
		goto alloc_chrdev_error;	/* PRQA S 2001 */
	}
	diag_dev_major = MAJOR(devno);	/* PRQA S ALL */
	cdev_init(p_cdev, &diag_dev_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
/* PRQA S ALL ++ */
	if (0 != ret) {
		pr_err("Error %d while adding example diag cdev", ret);
		goto cdev_add_error;
	}
	g_diag_dev_class = class_create(THIS_MODULE, "diag_dev");
	if (IS_ERR(g_diag_dev_class)) {
		pr_err("[%s:%d] class_create error\n",
			__func__, __LINE__);
		ret = PTR_ERR(g_diag_dev_class);
		goto class_create_error;
	}
	g_diag_dev = device_create(g_diag_dev_class, NULL,
		MKDEV(diag_dev_major, 0), NULL, "diag_dev");
	if (IS_ERR(g_diag_dev)) {
		pr_err("[%s] device create error\n", __func__);
		ret = PTR_ERR(g_diag_dev);
		goto device_create_error;
	}

	pr_info("diag dev [ver:%d.%d] init done\n",
			diag_dev_ver[0], diag_dev_ver[1]);

	return 0;

device_create_error:
	class_destroy(g_diag_dev_class);
class_create_error:
	cdev_del(&diag_dev_cdev);
cdev_add_error:
	unregister_chrdev_region(MKDEV(diag_dev_major, 0), 1);
alloc_chrdev_error:
		return ret;
}

void __exit diag_dev_release(void)
{
	pr_info("diag dev exit\n");
	device_destroy(g_diag_dev_class, MKDEV(diag_dev_major, 0));
	class_destroy(g_diag_dev_class);
	cdev_del(&diag_dev_cdev);
	unregister_chrdev_region(MKDEV(diag_dev_major, 0), 1);
}

module_init(diag_dev_init);
module_exit(diag_dev_release);

MODULE_AUTHOR("bo01.chen@horizon.ai");
MODULE_DESCRIPTION("diag netlink module");
MODULE_LICENSE("GPL");
/* PRQA S ALL -- */
