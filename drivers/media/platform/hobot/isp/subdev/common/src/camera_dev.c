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

#include "inc/camera_dev.h"
#include "inc/camera_subdev.h"
#include "inc/camera_i2c.h"
#include "inc/camera_sys_api.h"
#include "inc/camera_spi.h"

camera_charmod_s *camera_mod[CAMERA_TOTAL_NUMBER];
extern void sif_get_mismatch_status(void);
extern int mipi_host_int_fatal_show(int port);
static int camera_fop_open(struct inode *pinode, struct file *pfile)
{
	uint32_t tmp = 0;
	camera_charmod_s *camera_cdev = NULL;
	int minor = iminor(pinode);

	pr_info("camera_fop_open begin %d \n", __LINE__);
	for (tmp = 0; tmp < CAMERA_TOTAL_NUMBER; tmp++) {
		if(camera_mod[tmp] &&
				camera_mod[tmp]->dev_minor_id == minor ) {
				camera_cdev = camera_mod[tmp];
				pr_info(" tmp %d is open !\n", tmp);
				break;
			}
	}
	if (camera_cdev == NULL) {
		pr_err("camera cdev null\n");
		return -EINVAL;
	}
	mutex_lock(&camera_cdev->slock);
	if (camera_cdev->user_num > 0) {
		pr_info("more than one pthred use !\n");
	} else {
		camera_cdev->mst_file = pfile;
		camera_cdev->start_num = 0;
		camera_cdev->init_num = 0;
		camera_cdev->pre_state = SENSOR_PRE_STATE_UNLOCK;
		pfile->private_data = camera_cdev;
		pr_info("user_mutex init !\n");
	}
	camera_cdev->user_num++;
	pfile->private_data = camera_cdev;
	mutex_unlock(&camera_cdev->slock);
	pr_info("line %d user_num %d\n", __LINE__, camera_cdev->user_num);
	return 0;
}

static int camera_fop_release(struct inode *pinode, struct file *pfile)
{
	camera_charmod_s *camera_cdev = pfile->private_data;
	int port = camera_cdev->port;

	mutex_lock(&camera_cdev->slock);
	camera_cdev->user_num--;
	camera_cdev->mst_file = NULL;
	if (camera_cdev->user_num <= 0) {
		camera_cdev->pre_state = SENSOR_PRE_STATE_UNLOCK;
		camera_sys_stream_off(camera_cdev->port);
		camera_i2c_release(camera_cdev->port);
		memset(&camera_mod[camera_cdev->port]->camera_param, 0,
			sizeof(sensor_turning_data_t));
		kzfree(camera_mod[port]->camera_state_register_info.\
			deserial_register_info.register_table);
		kzfree(camera_mod[port]->camera_state_register_info.\
			serial_register_info.register_table);
		kzfree(camera_mod[port]->camera_state_register_info.\
			sensor_register_info.register_table);
		camera_cdev->start_num = 0;
		camera_cdev->init_num = 0;
	}
	pfile->private_data = NULL;
	mutex_unlock(&camera_cdev->slock);
	pr_info("line %d user_num %d  camera_cdev->start_num %d \n",
			__LINE__, camera_cdev->user_num, camera_cdev->start_num);
	return 0;
}
static long camera_fop_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	camera_charmod_s *camera_cdev = pfile->private_data;
	sensor_turning_data_t turning_data;
	sensor_turning_data_ex_t turning_param_ex;
	camera_state_register_t camera_register_data;
	int mst_flg = 0;

	mutex_lock(&camera_cdev->slock);
	if (camera_cdev->mst_file == NULL) {
		camera_cdev->mst_file = pfile;
		mst_flg = 1;
	} else if (camera_cdev->mst_file != pfile) {
		mst_flg = 0;
	} else {
		mst_flg = 1;
	}
	mutex_unlock(&camera_cdev->slock);

	switch(cmd) {
		case SENSOR_TURNING_PARAM: {
			if (arg == 0) {
				pr_err("arg is null !\n");
				return -EINVAL;
			}
			if (mst_flg == 0) {
				pr_info("this file is not master,cmd tunning!\n");
				return 0;
			}
			if (copy_from_user((void *)&turning_data, (void __user *)arg,
				sizeof(sensor_turning_data_t))) {
				pr_err("copy is err !\n");
				return -EINVAL;
			}
			if(turning_data.bus_type == I2C_BUS) { // i2c
				camera_i2c_open(camera_cdev->port, turning_data.bus_num,
						turning_data.sensor_name, turning_data.sensor_addr);
			} else if (turning_data.bus_type == SPI_BUS) { // spi
				camera_spi_open(camera_cdev->port, turning_data.bus_num, turning_data.cs,
						turning_data.spi_mode, turning_data.spi_speed, turning_data.sensor_name);
			}
			ret = camera_sys_turining_set(camera_cdev->port, &turning_data);
		}
			break;
		case SENSOR_TURNING_PARAM_EX: {
			if (arg == 0) {
				pr_err("arg is null !\n");
				return -EINVAL;
			}
			if (copy_from_user((void *)&turning_param_ex, (void __user *)arg,
				sizeof(sensor_turning_data_ex_t))) {
				pr_err("SENSOR_TURNING_PARAM_EX copy is err !\n");
				return -EINVAL;
			}
			ret = camera_turning_param_config(camera_cdev->port, &turning_param_ex);
			}
			break;
		case SENSOR_OPEN_CNT:
			if (copy_to_user((void __user *)arg,
				(void *)&camera_cdev->user_num,
				sizeof(uint32_t))) {
				pr_err("ioctl copy to user is error! %d\n", __LINE__);
				return -EINVAL;
			}
			break;
		case SENSOR_SET_START_CNT:
			if (copy_from_user((void *)&camera_cdev->start_num,
				(void __user *)arg, sizeof(int))) {
				pr_err("ioctl set user start count err !\n");
				return -EINVAL;
			}
			if (camera_cdev->start_num == 1)
				pr_info("ioctl set start cnt %d start_num %d\n",
					__LINE__, camera_cdev->start_num);
			if (camera_cdev->start_num == 0)
				pr_info("ioctl set start cnt %d start_num %d\n",
					__LINE__, camera_cdev->start_num);
			break;
		case SENSOR_GET_START_CNT:
			if (copy_to_user((void __user *)arg,
				(void *)&camera_cdev->start_num,
				sizeof(int))) {
				pr_err("ioctl get user start count err !\n");
				return -EINVAL;
			}
			break;
		case SENSOR_SET_INIT_CNT:
			if (copy_from_user((void *)&camera_cdev->init_num,
				(void __user *)arg, sizeof(int))) {
				pr_err("ioctl set user init count err !\n");
				return -EINVAL;
			}
			if (camera_cdev->init_num == 1)
				pr_info("ioctl set init cnt %d init_num %d\n",
					__LINE__, camera_cdev->init_num);
			if (camera_cdev->init_num == 0)
				pr_info("ioctl set init cnt %d init_num %d\n",
					__LINE__, camera_cdev->init_num);
			break;
		case SENSOR_GET_INIT_CNT:
			if (copy_to_user((void __user *)arg,
				(void *)&camera_cdev->init_num,
				sizeof(int))) {
				pr_err("ioctl get user init count err !\n");
				return -EINVAL;
			}
			break;
		case SENSOR_USER_LOCK:
			if (mutex_lock_interruptible(&camera_cdev->user_mutex)) {
				pr_err("mutex_lock_interruptible lock error \n");
				return -EINVAL;
			}
			if (camera_cdev->pre_state == SENSOR_PRE_STATE_UNLOCK) {
				camera_cdev->pre_state = SENSOR_PRE_STATE_LOCK;
				mutex_unlock(&camera_cdev->user_mutex);
			} else {
				camera_cdev->pre_done = false;
				mutex_unlock(&camera_cdev->user_mutex);
			wait_again:
				if(wait_event_interruptible(camera_cdev->pre_wq, camera_cdev->pre_done)) {
					pr_err("wait_event_interruptible error! \n");
					return -EINVAL;
				}
				if (mutex_lock_interruptible(&camera_cdev->user_mutex)) {
					pr_err("ioctl sensor user lock error! \n");
					return -EINVAL;
				}
				if (camera_cdev->pre_state == SENSOR_PRE_STATE_LOCK) {
					camera_cdev->pre_done = false;
					mutex_unlock(&camera_cdev->user_mutex);
					pr_err("ioctl sensor user lock failed!!\n");
					goto wait_again;
				}
				camera_cdev->pre_state = SENSOR_PRE_STATE_LOCK;
				mutex_unlock(&camera_cdev->user_mutex);
				pr_info("SENSOR_USER_LOCK  pre_state %d =start_num %d =init_num %d user_num %d\n",
					camera_cdev->pre_state, camera_cdev->start_num, camera_cdev->init_num, camera_cdev->user_num);
			}
			break;
		case SENSOR_USER_UNLOCK:
			if (mutex_lock_interruptible(&camera_cdev->user_mutex)) {
				pr_err("ioctl sensor user lock error!\n");
				return -EINVAL;
			}
			camera_cdev->pre_state = SENSOR_PRE_STATE_UNLOCK;
			camera_cdev->pre_done = true;
			mutex_unlock(&camera_cdev->user_mutex);
			wake_up_interruptible(&camera_cdev->pre_wq);
			break;
		case SENSOR_AE_SHARE:
			if (copy_from_user((void *)&camera_cdev->ae_share_flag,
				(void __user *)arg, sizeof(uint32_t))) {
				pr_err("ioctl ae share flag set err !\n");
				return -EINVAL;
			}
			pr_err("ae_share %d \n", camera_cdev->ae_share_flag);
			break;
		case SENSOR_GPIO_CONTROL: {
				gpio_info_t gpio_info;
				if (arg == 0) {
					pr_err("arg is null !\n");
					return -EINVAL;
				}
				if (copy_from_user((void *)&gpio_info, (void __user *)arg,
					sizeof(gpio_info_t))) {
					pr_err("gpio_info_t copy is err !\n");
					return -EINVAL;
				}
				ret = camera_gpio_info_config(&gpio_info);
				if(ret < 0) {
					pr_err("camera_gpio_info_config err !\n");
					return -EINVAL;
				}
			}
			break;
		case CAMERA_REG_PARAM: {
			if (arg == 0) {
				pr_err("arg is null !\n");
				return -EINVAL;
			}
			if (mst_flg == 0) {
				pr_info("this file is not master,cmd tunning!\n");
				return 0;
			}
			if (copy_from_user((void *)&camera_register_data, (void __user *)arg,
				sizeof(camera_register_data))) {
				pr_err("copy is err !\n");
				return -EINVAL;
			}
		}
			if(camera_mod[camera_cdev->port]->client == NULL) {
				camera_i2c_open(camera_cdev->port,
						camera_register_data.bus_num,
						NULL, 0);
			}
			ret = camera_state_register_set(camera_cdev->port,
					&camera_register_data);
			if (ret < 0) {
				pr_err("camera state register set is error!\n");
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

static int camera_copy_from_user_alloc(void *user_addr,
		uint16_t **kernel_addr, int bit_width, int count)
{
	*kernel_addr = kzalloc(count*bit_width, GFP_KERNEL);
	if(*kernel_addr == NULL) {
		pr_err("camera register table kzalloc fail!\n");
		return -ENOMEM;
	}
	if (copy_from_user((void *)*kernel_addr,
				(void __user *)user_addr, count*bit_width)) {
		pr_err("copy form user is err!\n");
		return -EINVAL;
	}
	return 0;
}

int camera_state_register_set(uint32_t port,
		camera_state_register_t *camera_register_data)
{
	int ret = 0;
	int buf_len = 0;
	uint16_t *tmp_buf;
	int reg_width;

	pr_err("camera register set begin\n");
	camera_mod[port]->port = port;
	memcpy(&camera_mod[port]->camera_state_register_info, camera_register_data,
			sizeof(camera_state_register_t));

	/* Deserial register table copy form user */
	reg_width = camera_register_data->deserial_register_info.reg_width;
	buf_len = camera_register_data->deserial_register_info.register_table_size;
	tmp_buf = camera_register_data->deserial_register_info.register_table;
	if (buf_len) {
		ret = camera_copy_from_user_alloc(tmp_buf,
			&camera_mod[port]->camera_state_register_info.\
				deserial_register_info.register_table,
			reg_width,
			buf_len);
		if(ret < 0) {
			return ret;
		}
	}

	/* Serial register table copy form user */
	reg_width = camera_register_data->serial_register_info.reg_width;
	buf_len = camera_register_data->serial_register_info.register_table_size;
	tmp_buf = camera_register_data->serial_register_info.register_table;
	if (buf_len) {
		ret = camera_copy_from_user_alloc(tmp_buf,
			&camera_mod[port]->camera_state_register_info.\
				serial_register_info.register_table,
			reg_width,
			buf_len);
		if(ret < 0) {
			kzfree(camera_mod[port]->camera_state_register_info.\
				deserial_register_info.register_table);
			camera_mod[port]->camera_state_register_info.\
				deserial_register_info.register_table = NULL;
			return ret;
		}
	}

	/* Sensor register table copy form user */
	reg_width = camera_register_data->sensor_register_info.reg_width;
	buf_len = camera_register_data->sensor_register_info.register_table_size;
	tmp_buf = camera_register_data->sensor_register_info.register_table;
	if (buf_len) {
		ret = camera_copy_from_user_alloc(tmp_buf,
			&camera_mod[port]->camera_state_register_info.\
				sensor_register_info.register_table,
			reg_width,
			buf_len);
		if (ret < 0) {
			kzfree(camera_mod[port]->camera_state_register_info.\
				deserial_register_info.register_table);
			camera_mod[port]->camera_state_register_info.\
				deserial_register_info.register_table = NULL;
			kzfree(camera_mod[port]->camera_state_register_info.\
				serial_register_info.register_table);
			camera_mod[port]->camera_state_register_info.\
				serial_register_info.register_table = NULL;
		}
	}
	if (ret == 0) {
		camera_mod[port]->camera_check_flag = 1;
	}
	return ret;
}

static int camera_register_status_read(int port, uint8_t bus_num,
		uint16_t slave_addr, register_info_t register_info)
{
	int ret = 0, i;
	uint16_t reg_addr;
	uint8_t reg_width;
	uint8_t value_width;
	uint16_t value;
	int size;

	reg_width = register_info.reg_width;
	value_width = register_info.value_width;
	size = register_info.register_table_size;
	if (register_info.register_table == NULL) {
		pr_err("register table is NULL.\n");
	} else {
		for(i = 0; i < size; i++) {
			reg_addr = register_info.register_table[i];
			ret = camrea_i2c_adapter_read(port, slave_addr, reg_addr,
					reg_width, &value, value_width/8);
			if (ret < 0) {
				pr_err("i2c read error!!!\n");
			}
			pr_err("i2c_bus:0x%x, reg_addr:0x%x, value:0x%x\n",
					bus_num, reg_addr, value);
		}
	}
	return ret;
}
int camera_register_status_print(int port)
{
	int ret = 0;
	uint8_t bus_num;
	uint8_t deserial_addr;
	uint8_t serial_addr;
	uint8_t sensor_addr;
	uint8_t mipi_index;

	if (camera_mod[port]->camera_check_flag == 0) {
		pr_info("camera check register is not set!\n");
		return 0;
	}
	camera_mod[port]->write_flag = 1;
	pr_err("camera port %d register status print begin\n", port);
	camera_state_register_t camera_register_data;
	memcpy(&camera_register_data,
		&camera_mod[port]->camera_state_register_info,
		sizeof(camera_state_register_t));

	bus_num = camera_register_data.bus_num;
	deserial_addr = camera_register_data.deserial_addr;
	serial_addr = camera_register_data.serial_addr;
	sensor_addr = camera_register_data.sensor_addr;

	/* Deserial register print */
	pr_err("deserial 0x%x register statues:\n", deserial_addr);
	camera_register_status_read(port, bus_num, deserial_addr,
			camera_register_data.deserial_register_info);

	/* Serial register print */
	pr_err("Serial 0x%x register statues:\n", serial_addr);
	camera_register_status_read(port, bus_num, serial_addr,
			camera_register_data.serial_register_info);

	/* Sensor register print */
	pr_err("Sensor 0x%x register statues:\n", sensor_addr);
	camera_register_status_read(port, bus_num, sensor_addr,
			camera_register_data.sensor_register_info);
	camera_mod[port]->write_flag = 0;

	/* SIF status check */
	sif_get_mismatch_status();

	/* MIPI status check */
	mipi_index = camera_register_data.mipi_index;
	ret = mipi_host_int_fatal_show(mipi_index);
	return ret;
}
EXPORT_SYMBOL(camera_register_status_print);

const struct file_operations camera_fops = {
	.owner = THIS_MODULE,
	.open = camera_fop_open,
	.release = camera_fop_release,
	.unlocked_ioctl = camera_fop_ioctl,
	.compat_ioctl = camera_fop_ioctl,
};

int __init camera_dev_init(uint32_t port)
{
	int ret = 0;

	if (port >= CAMERA_TOTAL_NUMBER) {
		return -ENXIO;
	}
	camera_mod[port] = kzalloc(sizeof(camera_charmod_s), GFP_KERNEL);
	if (camera_mod[port] == NULL) {
		pr_err("%s --%d kzalloc !\n", __func__, __LINE__);
		return -ENOMEM;
	}
	snprintf(camera_mod[port]->name, CHAR_DEVNAME_LEN, "port_%d", port);
	camera_mod[port]->camera_chardev.name = camera_mod[port]->name;
	camera_mod[port]->camera_chardev.minor = MISC_DYNAMIC_MINOR;
	camera_mod[port]->camera_chardev.fops = &camera_fops;

	ret = misc_register(&camera_mod[port]->camera_chardev);
	if (ret) {
		pr_err("%s --%d, register failed, err %d !\n",
					__func__, __LINE__, ret);
		goto register_err;
	}
	camera_mod[port]->dev_minor_id = camera_mod[port]->camera_chardev.minor;
	camera_mod[port]->port = port;
	mutex_init(&(camera_mod[port]->slock));
	mutex_init(&(camera_mod[port]->user_mutex));
	init_waitqueue_head(&camera_mod[port]->pre_wq);
	pr_info("port %d %s register success !\n", port, camera_mod[port]->name);
	return ret;

register_err:
	kzfree(camera_mod[port]);
	return ret;
}

void __exit camera_dev_exit(int port)
{
	if ((port < CAMERA_TOTAL_NUMBER) && (camera_mod[port] != NULL)) {
		mutex_destroy(&(camera_mod[port]->slock));
		mutex_destroy(&(camera_mod[port]->user_mutex));
		misc_deregister(&camera_mod[port]->camera_chardev);
		kzfree(camera_mod[port]);
		camera_mod[port] = NULL;
	}
	pr_info("camera_dev_exit port %d successfully.\n", port);
}

void camera_cdev_exit(void)
{
	uint32_t tmp = 0;

	for (tmp = 0; tmp < CAMERA_TOTAL_NUMBER; tmp++) {
		camera_dev_exit(tmp);
	}
}

int camera_cdev_init(void)
{
	int ret = 0;
	uint32_t tmp = 0;

	for (tmp = 0; tmp < CAMERA_TOTAL_NUMBER; tmp++) {
		ret = camera_dev_init(tmp);
		if (ret < 0) {
			pr_info("camera_dev_init %d is failed\n", tmp);
			goto devinit_err;
		}
	}

	return ret;
devinit_err:
	camera_cdev_exit();
	return ret;
}

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("camera_char dev of x3");
MODULE_LICENSE("GPL");

