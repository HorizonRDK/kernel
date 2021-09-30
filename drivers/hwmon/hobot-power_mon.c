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

#define pr_fmt(fmt) "[hobot-power_mon]: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

struct power_data {
	struct i2c_adapter *i2c_adapter;
	int start;
	int major;
	int minor;
	struct cdev cdev;
	dev_t dev_num;
	struct class *classes;
	wait_queue_head_t wq_head;
	spinlock_t lock;
	struct task_struct *task;
};

struct power_data* hobot_power = NULL;

struct power_monitor_result_s {
	unsigned long long curtime;
	u16 ina226_shunt[4];
	u16 ina226_current[4];
	u16 pmic_vol[5];
	u16 pmic_cur[5];
};

struct power_monitor_result_s* power_ddr_info = NULL;
char * power_result_buf = NULL;
unsigned int g_power_current_index = 0;
volatile unsigned int g_power_record_num = 0;
unsigned int g_power_monitor_poriod = 5;
module_param(g_power_monitor_poriod, uint, 0644);
unsigned int g_power_control_bit = 0xf7;
module_param(g_power_control_bit, uint, 0644);

#define TOTAL_RECORD_NUM 400
#define TOTAL_RESULT_SIZE (160*1024)
ktime_t g_power_ktime_start;

#define POW_MONITOR_CUR	_IOWR('p', 0, struct power_monitor_result_s)

int i2c_read_word_ex(int dev, int reg, u16* result)
{
	union i2c_smbus_data data;
	int status;
	status = i2c_smbus_xfer(hobot_power->i2c_adapter, dev, 0, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data);
	*result = ((data.word & 0xff) << 8) | ((data.word >> 8) & 0xff);
	if(status) {
		printk("i2c read err:%d\n",status);
	}
	return status;
}

int i2c_write_word_ex(int dev, int reg, u16 value)
{
	union i2c_smbus_data data;
	data.word = ((value & 0xff) << 8) | ((value >> 8) & 0xff);
	return i2c_smbus_xfer(hobot_power->i2c_adapter, dev, 0, I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA, &data);
}

u16 i2c_read_word(int dev, int reg, u16* result)
{
	union i2c_smbus_data data;
	int status;
	status = i2c_smbus_xfer(hobot_power->i2c_adapter, dev, 0, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data);
	if(status) {
		printk("i2c read err:%d\n",status);
	}
	*result = data.word;
	return status;
}

int i2c_write_word(int dev, int reg, u16 value)
{
	union i2c_smbus_data data;
	data.word = value;
	return i2c_smbus_xfer(hobot_power->i2c_adapter, dev, 0, I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA, &data);
}

int i2c_write_byte(int dev, int reg, u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_xfer(hobot_power->i2c_adapter, dev, 0, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data);
}


int cfg_INA226(int devaddr, int resolution)
{
	int ret = 0;
	u16 reg_value = 0;
	reg_value = i2c_read_word_ex(devaddr, 0x00, &reg_value);
	reg_value |= 0x1 << 15;
	ret |= i2c_write_word_ex(devaddr, 0x00, reg_value);
	//i2c_write_word_ex(devaddr, 0x00, 0x4005);//0x4205);//test
	ret |= i2c_write_word_ex(devaddr, 0x00, 0x4305);//1ms avg 1
	ret |= i2c_write_word_ex(devaddr, 0x05, resolution);
	return ret;
}

int read_INA226_value(struct power_monitor_result_s* res_info)
{
	int ret = 0;
	if (g_power_control_bit & BIT(0)) {
		ret |= i2c_read_word_ex(0x40, 0x01, &res_info->ina226_shunt[0]);
		ret |= i2c_read_word_ex(0x40, 0x04, &res_info->ina226_current[0]);
	}

	if (g_power_control_bit & BIT(1)) {
		ret |= i2c_read_word_ex(0x41, 0x01, &res_info->ina226_shunt[1]);
		ret |= i2c_read_word_ex(0x41, 0x04, &res_info->ina226_current[1]);
	}

	if (g_power_control_bit & BIT(2)) {
		ret |= i2c_read_word_ex(0x44, 0x01, &res_info->ina226_shunt[2]);
		ret |= i2c_read_word_ex(0x44, 0x04, &res_info->ina226_current[2]);
	}

	if (g_power_control_bit & BIT(3)) {
		ret |= i2c_read_word_ex(0x45, 0x01, &res_info->ina226_shunt[3]);
		ret |= i2c_read_word_ex(0x45, 0x04, &res_info->ina226_current[3]);
	}
	return ret;
}

int read_pmic_value(struct power_monitor_result_s* res_info)
{
	int ret = 0;
	if (g_power_control_bit & BIT(4)) {
		ret |= i2c_write_byte(0x42, 0x00, 0x00);
	//	ret |= i2c_read_word(0x42, 0x8b, &res_info->pmic_vol[0]);
	//	res_info->pmic_vol[0] = res_info->pmic_vol[0] & 0x7ff;
		ret |= i2c_read_word(0x42, 0x8c, &res_info->pmic_cur[0]);
		res_info->pmic_cur[0] = res_info->pmic_cur[0] & 0x7ff;
	}

	if (g_power_control_bit & BIT(5)) {
		ret |= i2c_write_byte(0x42, 0x00, 0x01);
	//	ret |= i2c_read_word(0x42, 0x8b, &res_info->pmic_vol[1]);
	//	res_info->pmic_vol[1] = res_info->pmic_vol[1] & 0x7ff;
		ret |= i2c_read_word(0x42, 0x8c, &res_info->pmic_cur[1]);
		res_info->pmic_cur[1] = res_info->pmic_cur[1] & 0x7ff;
	}

	if (g_power_control_bit & BIT(6)) {
		ret |= i2c_write_byte(0x42, 0x00, 0x02);
	//	ret |= i2c_read_word(0x42, 0x8b, &res_info->pmic_vol[2]);
	//	res_info->pmic_vol[2] = res_info->pmic_vol[2] & 0x7ff;
		ret |= i2c_read_word(0x42, 0x8c, &res_info->pmic_cur[2]);
		res_info->pmic_cur[2] = res_info->pmic_cur[2] & 0x7ff;
	}

	if (g_power_control_bit & BIT(7)) {
		ret |= i2c_write_byte(0x42, 0x00, 0x03);
	//	ret |= i2c_read_word(0x42, 0x8b, &res_info->pmic_vol[3]);
	//	res_info->pmic_vol[3] = res_info->pmic_vol[3] & 0x7ff;
		ret |= i2c_read_word(0x42, 0x8c, &res_info->pmic_cur[3]);
		res_info->pmic_cur[3] = res_info->pmic_cur[3] & 0x7ff;
	}

//	ret |= i2c_write_byte(0x42, 0x00, 0x04);
//	ret |= i2c_read_word(0x42, 0x8b, &res_info->pmic_vol[4]);
//	res_info->pmic_vol[4] = res_info->pmic_vol[4] & 0x7ff;
//	ret |= i2c_read_word(0x42, 0x8c, &res_info->pmic_cur[4]);
//	res_info->pmic_cur[4] = res_info->pmic_cur[4] & 0x7ff;
	return ret;
}

static int power_mon_thread(void *data)
{
	printk("power_mon_thread run\n");
	hobot_power->start = 1;
	do {
		if(power_ddr_info) {
			ktime_t ktime;
			ktime = ktime_sub(ktime_get(), g_power_ktime_start);
			power_ddr_info[g_power_current_index].curtime = ktime_to_ms(ktime);
			if (read_INA226_value(&power_ddr_info[g_power_current_index])) {
				printk("Get ina226 value error\n");
				if (g_power_monitor_poriod)
					msleep(g_power_monitor_poriod);
				continue;
			}
			if (read_pmic_value(&power_ddr_info[g_power_current_index])) {
				printk("Get pmic value error\n");
				if (g_power_monitor_poriod)
					msleep(g_power_monitor_poriod);
				continue;
			}
			g_power_current_index = (g_power_current_index + 1) % TOTAL_RECORD_NUM;
			g_power_record_num ++;
			if (g_power_record_num >= 200)
				wake_up_interruptible(&hobot_power->wq_head);
			}
			if (g_power_monitor_poriod)
				msleep(g_power_monitor_poriod);
	} while (!kthread_should_stop());
	hobot_power->start = 0;
	printk("power_mon_thread exit\n");
	return 0;
}

int power_start(void)
{
	if (hobot_power && !power_ddr_info) {
		printk("power_start\n");
		//enable_irq(g_power_dev->irq);
		power_ddr_info = vmalloc(sizeof(struct power_monitor_result_s) * TOTAL_RECORD_NUM);
		g_power_current_index = 0;
		g_power_record_num = 0;
		g_power_ktime_start = ktime_get();
		if (hobot_power->task == NULL) {
			hobot_power->task = kthread_run(power_mon_thread, (void *)NULL, "power_mon_thread");
			if (IS_ERR(hobot_power->task)) {
				hobot_power->task = NULL;
				return -1;
			}
		}
	}
	return 0;
}

int power_stop(void)
{
	if (hobot_power->task)
		kthread_stop(hobot_power->task);
	hobot_power->task = NULL;
	while(hobot_power->start);
	if (hobot_power && power_ddr_info) {
		printk("power_stop\n");
		vfree(power_ddr_info);
		//vfree(power_result_buf);
		power_ddr_info = NULL;
	}
	return 0;
}

static int get_monitor_data(char* buf, int size)
{
	int j;
	int start = 0;
	int cur = 0;
	int length = 0;
	volatile int num = 0;
	if (!power_ddr_info) {
		printk("monitor not started \n");
		return 0;
	}
	if (g_power_record_num > 0) {

		spin_lock_irq(&hobot_power->lock);
		num = g_power_record_num;
		if (num >= TOTAL_RECORD_NUM)
			num = TOTAL_RECORD_NUM;
		start = (g_power_current_index + TOTAL_RECORD_NUM - num) % TOTAL_RECORD_NUM;
		g_power_record_num = 0;
		spin_unlock_irq(&hobot_power->lock);
		for (j = 0; j < num; j++) {
			cur = (start + j) % TOTAL_RECORD_NUM;
			memcpy(power_result_buf + j * sizeof(struct power_monitor_result_s), power_ddr_info + cur, sizeof(struct power_monitor_result_s));
			length ++;
		}
	}
	return length;
}

static int power_mon_open(struct inode *pinode, struct file *pfile)
{
	printk(KERN_INFO "power_mon_open()!\n");
	power_start();
	return 0;
}

static int power_mon_release(struct inode *pinode, struct file *pfile)
{
	printk(KERN_INFO "power_mon_release()!\n");
	power_stop();
	return 0;
}

static ssize_t power_mon_read(struct file *pfile, char *puser_buf, size_t len, loff_t *poff)
{
	int result_len = 0;
	wait_event_interruptible(hobot_power->wq_head, g_power_record_num > 200);
	result_len = get_monitor_data(power_result_buf, 80*1024);
	return result_len;
}

static ssize_t power_mon_write(struct file *pfile, const char *puser_buf, size_t len, loff_t *poff)
{
	printk(KERN_INFO "power_mon_write()!\n");
	return 0;
}

static long power_mon_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case POW_MONITOR_CUR:
		{
			int cur = 0;
			if (!arg) {
				printk(KERN_ERR "hobot pow_monitor get cur error\n");
				return -EINVAL;
			}
			cur  = (g_power_current_index - 1 + TOTAL_RECORD_NUM) % TOTAL_RECORD_NUM;
			if ( copy_to_user((void __user *)arg, (void *)(power_ddr_info + cur), sizeof(struct power_monitor_result_s)) ) {
				printk(KERN_ERR "hobot pow_monitor get cur error, copy data to user failed\n");
				return -EINVAL;
			}
		}
		break;
	default:
		break;
	}
	return 0;
}

int power_mmap(struct file *filp, struct vm_area_struct *pvma)
{

	printk("power_mmap! \n");

	if (remap_pfn_range(pvma, pvma->vm_start,
						virt_to_pfn(power_result_buf),
						pvma->vm_end - pvma->vm_start,
						pvma->vm_page_prot)) {
		printk(KERN_ERR "power_mmap fail\n");
		return -EAGAIN;
	}
	return 0;
}

struct file_operations power_mon_fops = {
	.owner			= THIS_MODULE,
	.mmap 			= power_mmap,
	.open			= power_mon_open,
	.read			= power_mon_read,
	.write			= power_mon_write,
	.release		= power_mon_release,
	.unlocked_ioctl = power_mon_ioctl,
};

int power_cdev_create(void)
{
	int ret = 0;
	struct device *device = NULL;

	printk(KERN_INFO "power_cdev_create()\n");

	hobot_power->classes = class_create(THIS_MODULE, "hobot_power_mon");
	if (IS_ERR(hobot_power->classes)) {
		pr_err("class_create failed\n");
		return PTR_ERR(hobot_power->classes);
	}

	ret = alloc_chrdev_region(&hobot_power->dev_num, 0, 1, "hobot_power_mon");
	if (ret != 0) {
		pr_err("alloc_chrdev_region failed, ret:%d\n", ret);
		return ret;
	}

	hobot_power->major = MAJOR(hobot_power->dev_num);
	hobot_power->minor = MINOR(hobot_power->dev_num);

	cdev_init(&hobot_power->cdev, &power_mon_fops);
	hobot_power->cdev.owner = THIS_MODULE;
	ret = cdev_add(&hobot_power->cdev, hobot_power->dev_num, 1);
	if (ret != 0) {
		pr_err("cdev_add failed,ret:%d\n", ret);
		return ret;
	}

	device = device_create(hobot_power->classes,
			NULL, hobot_power->dev_num, NULL, "hobot_power_mon");
	if (IS_ERR(device)) {
		pr_err("cdev_create failed\n");
		return PTR_ERR(device);
	}

	return ret;
}

void power_dev_remove(void)
{
	printk(KERN_INFO "power_dev_remove()\n");

	cdev_del(&hobot_power->cdev);

	unregister_chrdev_region(hobot_power->dev_num, 1);
}


static int power_mon_probe(struct platform_device *pdev)
{
	int ret = 0;

	hobot_power = kmalloc(sizeof(struct power_data), GFP_KERNEL);
	if (!hobot_power) {
		printk(KERN_ERR"Unable to alloc power_data\n");
		return -ENOMEM;
	}
	hobot_power->i2c_adapter = i2c_get_adapter(1);
	if (!hobot_power->i2c_adapter) {
		printk(KERN_ERR"Unable to get i2c1 adapter\n");
		return -EINVAL;
	}
	ret |= cfg_INA226(0x40, 0x2800);
	ret |= cfg_INA226(0x41, 0x2800);
	ret |= cfg_INA226(0x44, 0x2800);
	ret |= cfg_INA226(0x45, 0x2800);
	if (ret != 0) {
		pr_err("cfg_INA226 failed, ret:%d\n", ret);
		return ret;
	}
	power_result_buf = kmalloc(TOTAL_RESULT_SIZE, GFP_KERNEL);
	if (power_result_buf == NULL) {
		pr_err("kmalloc failed\n");
		return -1;
	}
	hobot_power->task = NULL;
	init_waitqueue_head(&hobot_power->wq_head);
	spin_lock_init(&hobot_power->lock);
	ret = power_cdev_create();
	if (ret != 0)
		pr_err("power_cdev_create failed, ret:%d\n", ret);
	return ret;
}

static int power_mon_remove(struct platform_device *pdev)
{
	kfree(power_result_buf);
	kfree(hobot_power);
	power_result_buf = NULL;
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id power_mon_of_match[] = {
	{.compatible = "hobot,hobot-power-mon"},
	{},
};
MODULE_DEVICE_TABLE(of, power_mon_of_match);
#endif

static struct platform_driver power_mon_driver = {
	.probe = power_mon_probe,
	.remove = power_mon_remove,
	.driver = {
		.name = "hobot-power-mon",
		.of_match_table = of_match_ptr(power_mon_of_match),
	},
};


module_platform_driver(power_mon_driver);

MODULE_LICENSE("GPL");
