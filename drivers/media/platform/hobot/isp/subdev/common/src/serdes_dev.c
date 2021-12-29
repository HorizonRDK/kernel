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

#define pr_fmt(fmt) "[isp_drv:cam]: %s: " fmt, __func__

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/spinlock_types.h>
#include <linux/printk.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/gpio.h>

#include "inc/serdes_dev.h"

#define SERDES_TOTAL_NUMBER 8
#define SERDES_NAME "serdes"


serdes_charmod_s *serdes_mod[SERDES_TOTAL_NUMBER];
static int serdes_fop_open(struct inode *pinode, struct file *pfile)
{
	uint32_t tmp = 0;
	serdes_charmod_s *serdes_cdev = NULL;
	int minor = iminor(pinode);

	pr_info("serdes_fop_open begin %d \n", __LINE__);
	for (tmp = 0; tmp < SERDES_TOTAL_NUMBER; tmp++) {
		if(serdes_mod[tmp] &&
				serdes_mod[tmp]->dev_minor_id == minor ) {
				serdes_cdev = serdes_mod[tmp];
				pr_info(" serdes %d is open !\n", tmp);
				break;
			}
	}
	if (serdes_cdev == NULL) {
		pr_err("serdes cdev null\n");
		return -EINVAL;
	}
	mutex_lock(&serdes_cdev->slock);
	if (serdes_cdev->user_num > 0) {
		pr_info("more than one pthred use !\n");
	} else {
		serdes_cdev->init_num = 0;
		serdes_cdev->pre_state = SERDES_PRE_STATE_UNLOCK;
		pfile->private_data = serdes_cdev;
		bitmap_zero(serdes_cdev->gpio_req_mask, GPIO_MAX_NUM);
		pr_info("user_mutex init !\n");
	}
	serdes_cdev->user_num++;
	pfile->private_data = serdes_cdev;
	mutex_unlock(&serdes_cdev->slock);
	pr_info("line %d serdes_index %d user_num %d\n",
		__LINE__, serdes_cdev->serdes_index, serdes_cdev->user_num);
	return 0;
}

static int serdes_gpio_request(u32 gpio, int init)
{
	int ret = 0;
	char gpio_name[GPIO_NAME_LENGTH];

	memset(gpio_name, 0, GPIO_NAME_LENGTH);
	snprintf(gpio_name, GPIO_NAME_LENGTH, "%s_gpio_%d",
		SERDES_NAME, gpio);

	pr_info("gpio %d req %s\n", gpio, gpio_name);
	ret = gpio_request(gpio, gpio_name);
	if (ret < 0) {
		pr_err("gpio %d request failed!", gpio);
		return ret;
	}
	ret = gpio_direction_output(gpio, init);
	if (ret < 0) {
		pr_err("gpio %d set direction failed!", gpio);
		gpio_free(gpio);
	}
	return ret;
}

int serdes_gpio_info_config(serdes_charmod_s *serdes_cdev,
		gpio_info_t *gpio_info)
{
	int ret = 0;
	u32 gpio;
	int gpio_v;

	if (!serdes_cdev || !gpio_info) {
		pr_err("line %d param invalid\n", __LINE__);
		return -1;
	}

	gpio = gpio_info->gpio;
	gpio_v = (GPIO_HIGH == gpio_info->gpio_level) ? 1 : 0;

	if (gpio >= GPIO_MAX_NUM) {
		pr_err("line %d gpio %d exceed max gpio error\n",
				__LINE__, gpio);
		return -1;
	}

	if (!test_bit(gpio, serdes_cdev->gpio_req_mask)) {
		ret = serdes_gpio_request(gpio, gpio_v);
		if(ret < 0) {
			pr_err("line %d gpio_request error gpio %d\n",
				   __LINE__, gpio);
			return -1;
		}
		set_bit(gpio, serdes_cdev->gpio_req_mask);
	}
	gpio_set_value(gpio, gpio_v);
	pr_info("gpio %d gpio_v %d\n", gpio, gpio_v);
	return ret;
}

void serdes_gpio_all_free(serdes_charmod_s * serdes_cdev)
{
	int i;

	if (!serdes_cdev) {
		return;
	}

	for_each_set_bit(i, serdes_cdev->gpio_req_mask, GPIO_MAX_NUM) {
		pr_info("gpio %d free\n", i);
		gpio_free(i);
	}
	bitmap_zero(serdes_cdev->gpio_req_mask, GPIO_MAX_NUM);
}

static int serdes_fop_release(struct inode *pinode, struct file *pfile)
{
	serdes_charmod_s *serdes_cdev = pfile->private_data;

	mutex_lock(&serdes_cdev->slock);
	if(serdes_cdev->user_num > 0)
		serdes_cdev->user_num--;
	if (serdes_cdev->user_num <= 0) {
		serdes_cdev->pre_state = SERDES_PRE_STATE_UNLOCK;
		serdes_gpio_all_free(serdes_cdev);
		serdes_cdev->init_num = 0;
	}
	pfile->private_data = NULL;
	mutex_unlock(&serdes_cdev->slock);
	pr_info("line %d serdes_index %d user_num %d\n",
		__LINE__, serdes_cdev->serdes_index, serdes_cdev->user_num);
	return 0;
}
static long serdes_fop_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	serdes_charmod_s *serdes_cdev = pfile->private_data;

	switch(cmd) {
		case SERDES_SET_INIT_CNT:
			if (copy_from_user((void *)&serdes_cdev->init_num,
				(void __user *)arg, sizeof(int))) {
				pr_err("ioctl set user init count err !\n");
				return -EINVAL;
			}
			if (serdes_cdev->init_num == 1)
				pr_info("ioctl set init cnt %d init_num %d\n",
					__LINE__, serdes_cdev->init_num);
			if (serdes_cdev->init_num == 0)
				pr_info("ioctl set init cnt %d init_num %d\n",
					__LINE__, serdes_cdev->init_num);
			break;
		case SERDES_GET_INIT_CNT:
			if (copy_to_user((void __user *)arg,
				(void *)&serdes_cdev->init_num,
				sizeof(int))) {
				pr_err("ioctl get user init count err !\n");
				return -EINVAL;
			}
			break;
		case SERDES_USER_LOCK:
			if (mutex_lock_interruptible(&serdes_cdev->user_mutex)) {
				pr_err("mutex_lock_interruptible lock error \n");
				return -EINVAL;
			}
			if (serdes_cdev->pre_state == SERDES_PRE_STATE_UNLOCK) {
				serdes_cdev->pre_state = SERDES_PRE_STATE_LOCK;
				mutex_unlock(&serdes_cdev->user_mutex);
			} else {
				mutex_unlock(&serdes_cdev->user_mutex);
			wait_again:
				if (mutex_lock_interruptible(&serdes_cdev->user_mutex)) {
					pr_err("ioctl sensor user lock error! \n");
					return -EINVAL;
				}
				if (serdes_cdev->pre_state == SERDES_PRE_STATE_LOCK) {
					mutex_unlock(&serdes_cdev->user_mutex);
					pr_err("ioctl sensor user lock failed!!\n");
					goto wait_again;
				}
				serdes_cdev->pre_state = SERDES_PRE_STATE_LOCK;
				mutex_unlock(&serdes_cdev->user_mutex);
				pr_info("SENSOR_USER_LOCK pre_state %d init_num %d user_num %d\n",
					serdes_cdev->pre_state, serdes_cdev->init_num, serdes_cdev->user_num);
			}
			break;
		case SERDES_USER_UNLOCK:
			if (mutex_lock_interruptible(&serdes_cdev->user_mutex)) {
				pr_err("ioctl sensor user lock error!\n");
				return -EINVAL;
			}
			serdes_cdev->pre_state = SERDES_PRE_STATE_UNLOCK;
			mutex_unlock(&serdes_cdev->user_mutex);
			break;
		case SERDES_GPIO_CONTROL: {
			mutex_lock(&serdes_cdev->slock);
				gpio_info_t gpio_info;
				if (arg == 0) {
					pr_err("arg is null !\n");
					mutex_unlock(&serdes_cdev->slock);
					return -EINVAL;
				}
				if (copy_from_user((void *)&gpio_info, (void __user *)arg,
					sizeof(gpio_info_t))) {
					pr_err("gpio_info_t copy is err !\n");
					mutex_unlock(&serdes_cdev->slock);
					return -EINVAL;
				}
				ret = serdes_gpio_info_config(serdes_cdev, &gpio_info);
				if(ret < 0) {
					pr_err("serdes_gpio_info_config err !\n");
					mutex_unlock(&serdes_cdev->slock);
					return -EINVAL;
				}
			mutex_unlock(&serdes_cdev->slock);
			}
			break;
		default: {
			pr_err("ioctl cmd is err \n");
			ret = -1;
		}
			break;
	}

	return ret;
}

const struct file_operations serdes_fops = {
	.owner = THIS_MODULE,
	.open = serdes_fop_open,
	.release = serdes_fop_release,
	.unlocked_ioctl = serdes_fop_ioctl,
	.compat_ioctl = serdes_fop_ioctl,
};

int __init serdes_dev_init(uint32_t index)
{
	int ret = 0;

	if (index >= SERDES_TOTAL_NUMBER) {
		return -ENXIO;
	}
	serdes_mod[index] = kzalloc(sizeof(serdes_charmod_s), GFP_KERNEL);
	if (serdes_mod[index] == NULL) {
		pr_err("%s --%d kzalloc !\n", __func__, __LINE__);
		return -ENOMEM;
	}
	snprintf(serdes_mod[index]->name, CHAR_DEVNAME_LEN, "serdes_%d", index);
	serdes_mod[index]->serdes_chardev.name = serdes_mod[index]->name;
	serdes_mod[index]->serdes_chardev.minor = MISC_DYNAMIC_MINOR;
	serdes_mod[index]->serdes_chardev.fops = &serdes_fops;

	ret = misc_register(&serdes_mod[index]->serdes_chardev);
	if (ret) {
		pr_err("%s --%d, register failed, err %d !\n",
					__func__, __LINE__, ret);
		goto register_err;
	}
	serdes_mod[index]->dev_minor_id = serdes_mod[index]->serdes_chardev.minor;
	serdes_mod[index]->serdes_index = index;
	mutex_init(&(serdes_mod[index]->slock));
	mutex_init(&(serdes_mod[index]->user_mutex));
	return ret;

register_err:
	kzfree(serdes_mod[index]);
	return ret;
}

void __exit serdes_dev_exit(int index)
{
	if ((index < SERDES_TOTAL_NUMBER) && (serdes_mod[index] != NULL)) {
		mutex_destroy(&(serdes_mod[index]->slock));
		mutex_destroy(&(serdes_mod[index]->user_mutex));
		misc_deregister(&serdes_mod[index]->serdes_chardev);
		kzfree(serdes_mod[index]);
		serdes_mod[index] = NULL;
	}
}

void serdes_cdev_exit(void)
{
	uint32_t tmp = 0;

	for (tmp = 0; tmp < SERDES_TOTAL_NUMBER; tmp++) {
		serdes_dev_exit(tmp);
	}
}

int serdes_cdev_init(void)
{
	int ret = 0;
	uint32_t tmp = 0;

	for (tmp = 0; tmp < SERDES_TOTAL_NUMBER; tmp++) {
		ret = serdes_dev_init(tmp);
		if (ret < 0) {
			pr_info("serdes_dev_init %d is failed\n", tmp);
			goto devinit_err;
		}
	}

	return ret;
devinit_err:
	serdes_cdev_exit();
	return ret;
}

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("serdes_char dev of XJ3");
MODULE_LICENSE("GPL");

