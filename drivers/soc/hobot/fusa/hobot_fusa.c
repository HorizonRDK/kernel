/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright 2020 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/
/**
 * @FileName	hobot-fusa.c
 * @version	1.0
 * @author      ming.yu
 * @CreatedTime 2020-06-21
 * @Log		
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/reset.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/pgtable.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/pwm.h>
#include <soc/hobot/hobot_fusa_fatal.h>

#define VER		"V0.1"
typedef struct _hobot_fusa_dev {
	struct class *class;
	struct device *device;
	struct cdev cdev;
	dev_t devno;
#ifndef CONFIG_HOBOT_XJ3
	uint32_t error_pin;
#else
	struct pwm_device *pwm_dev;
#endif
} hobot_fusa_dev_t;
hobot_fusa_dev_t g_fusa_device = {0};
static int fusa_open(struct inode *inode, struct file *filp);
static int fusa_release(struct inode *inode, struct file *filp);
static long fusa_compat_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg);

const struct file_operations fusa_fops = {
	.owner = THIS_MODULE,
	.open = fusa_open,
	.release = fusa_release,
	.unlocked_ioctl = fusa_compat_ioctl,
	.compat_ioctl = fusa_compat_ioctl
};


static int set_fusa_error_pin(unsigned int value)
{
	int ret = 0;
#ifdef CONFIG_HOBOT_XJ3
	struct pwm_device *pwm_dev = NULL;
#endif

	pr_err("%s:%d set error pin to %d\n", __func__, __LINE__, value);
	if (value != ERR_PIN_FATAL && value != ERR_PIN_NORMAL)
		return -1;
#ifndef CONFIG_HOBOT_XJ3
	ret = gpio_direction_output(g_fusa_device.error_pin, value);
	if (ret != 0) {
		pr_err("%s:%d, set gpio %d failed\n",
			__func__,
			__LINE__,
			g_fusa_device.error_pin);
	}
#else
	if (!g_fusa_device.pwm_dev) {
		pr_err("fusa errpin not ready");
		return -1;
	}
	pwm_dev = g_fusa_device.pwm_dev;
	if (value == ERR_PIN_FATAL) {
		pwm_disable(pwm_dev);
	}
	if (value == ERR_PIN_NORMAL) {
		ret = pwm_enable(pwm_dev);
		if (ret < 0) {
			pr_err("set fusa errpin to normal failed\n");
		}
	}
#endif
	return ret;
}

void panic_diagnose_notify()
{
	/*set error pin*/
	set_fusa_error_pin(ERR_PIN_FATAL);

	/* nofity diagnose, do nothing now*/
}

EXPORT_SYMBOL(panic_diagnose_notify);


static int fusa_open(struct inode *inode, struct file *filp)
{
#ifdef CONFIG_HOBOT_XJ3
	struct pwm_device *pwm_dev = NULL;
	hobot_fusa_dev_t *fusa_dev = NULL;

	fusa_dev = container_of(inode->i_cdev, hobot_fusa_dev_t, cdev);
	pwm_dev = fusa_dev->pwm_dev;
	filp->private_data = fusa_dev;
	pwm_enable(pwm_dev);
#endif
	return 0;
}

static int fusa_release(struct inode *inode, struct file *filp)
{
#ifdef CONFIG_HOBOT_XJ3
	struct pwm_device *pwm_dev = NULL;
	hobot_fusa_dev_t *fusa_dev = NULL;

	fusa_dev = container_of(inode->i_cdev, hobot_fusa_dev_t, cdev);
	pwm_dev = fusa_dev->pwm_dev;
	filp->private_data = NULL;
	/* do not turn off pwm signle when fusa errpin release,
	 * or J3SOC may be reset by MCU */
	// pwm_disable(pwm_dev);
#endif
	return 0;
}

static long fusa_compat_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long args)
{
	uint32_t value;
	int32_t ret = 0;
	struct timeval ts;
	signed long ts_usec;

	if (_IOC_TYPE(cmd) != FUSA_FATAL_MAGIC) {
		pr_err("Invaild MAGIC_NUM\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FUSA_SET_ERROR_PIN:
		value = *(uint32_t *)args;
		do_gettimeofday(&ts);
		ts_usec=ts.tv_sec*1000000+ts.tv_usec;
		printk("KERNEL INFO: ready to operate fusa pin, ts is [%ld]\n",ts_usec);
		ret  = set_fusa_error_pin(value);
		break;
	}
	return ret;
}

static void hobot_fusa_free_chardev(hobot_fusa_dev_t *fusa_dev)
{
	device_destroy(fusa_dev->class, fusa_dev->devno);
	class_destroy(fusa_dev->class);
	cdev_del(&fusa_dev->cdev);
	unregister_chrdev_region(fusa_dev->devno, 1);
}

static int hobot_fusa_fatal_init_chrdev(hobot_fusa_dev_t *fusa_dev)
{
	int rc;
	// Allocate a major and minor number region for the character device
	rc = alloc_chrdev_region(&fusa_dev->devno, 0, 1, "fusa_api");
	if (rc < 0) {
		pr_err("Unable to allocate character device region.\n");
		goto ret;
	}

	// Create a device class for our device
	fusa_dev->class = class_create(THIS_MODULE, "fusa_api");
	if (IS_ERR(fusa_dev->class)) {
		pr_err("Unable to create a device class.\n");
		rc = PTR_ERR(fusa_dev->class);
		goto free_chrdev_region;
	}

	/* Create a device for our module. This will create a file on the
	 * filesystem, under "/dev/dev->chrdev_name".
	 */
	fusa_dev->device = device_create(fusa_dev->class,
					 NULL,
					 fusa_dev->devno,
					 NULL,
					 "fusa_api");
	if (IS_ERR(fusa_dev->device)) {
		pr_err("failed to create device\n");
		goto class_cleanup;
	}
	// Register our character device with the kernel
	cdev_init(&fusa_dev->cdev, &fusa_fops);
	rc = cdev_add(&fusa_dev->cdev, fusa_dev->devno, 1);
	if (rc < 0) {
		pr_err("Unable to add a character device.\n");
		goto device_cleanup;
	}

	return 0;

device_cleanup:
	device_destroy(fusa_dev->class, fusa_dev->devno);
class_cleanup:
	class_destroy(fusa_dev->class);
free_chrdev_region:
	unregister_chrdev_region(fusa_dev->devno, 1);
ret:
	return rc;
}

static int fusa_fatal_probe(struct platform_device *pdev)
{
	int ret = 0;
#ifdef CONFIG_HOBOT_XJ3
	struct pwm_device *pwm_dev = NULL;
	struct pwm_args pwm_args;
	struct pwm_state pwm_state;
#endif

	pr_err("hobot fusa fatal api start");
#ifndef CONFIG_HOBOT_XJ3
	ret = of_property_read_u32(pdev->dev.of_node,
				   "fusa_error_pin",
				   &g_fusa_device.error_pin);

	if (ret) {
		pr_err("get fusa error pin error\n");
		return -1;
	}

	ret = devm_gpio_request(&pdev->dev,
				   g_fusa_device.error_pin,
				   "fusa_error_pin");
	if (ret) {
		pr_err("request fusa error pin failed\n");
		return -EINVAL;
	}
	//gpio_direction_output(g_fusa_device.error_pin, 0);
#else
	pwm_dev = devm_pwm_get(&pdev->dev, "errpin");
	if (IS_ERR(pwm_dev)) {
		pr_err("get fusa pwm device failed\n");
		return -EINVAL;
	}
	pwm_get_args(pwm_dev, &pwm_args);
	pr_info("fusa fatal pwm args:\nperiod:%d polarity:%d\n",
				pwm_args.period, pwm_args.polarity);

	pwm_config(pwm_dev, pwm_args.period / 2, pwm_args.period);
	pwm_enable(pwm_dev);

	pwm_get_state(pwm_dev, &pwm_state);
	pr_info("fusa fatal info:\nperiod:%d duty_cycle:%d enabled:%d\n",
				pwm_state.period, pwm_state.duty_cycle, pwm_state.enabled);

	g_fusa_device.pwm_dev = pwm_dev;
#endif

	ret = hobot_fusa_fatal_init_chrdev(&g_fusa_device);
	if (ret != 0) {
		dev_err(&pdev->dev, "Err %s\n", VER);
		return -EINVAL;
	}
	dev_err(&pdev->dev, "Suc %s\n", VER);

	platform_set_drvdata(pdev, &g_fusa_device);
	return 0;
}

static int fusa_fatal_remove(struct platform_device *pdev)
{
	hobot_fusa_free_chardev(&g_fusa_device);
	if(g_fusa_device.pwm_dev)
		devm_pwm_put(&pdev->dev, g_fusa_device.pwm_dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id hobot_fusa_fatal_of_match[] = {
	/* SoC-specific compatible strings w/ soc_ctl_map */
	{.compatible = "hobot,fusa_fatal"},
	{ /* end of table */ }
};

static struct platform_driver hobot_fusa_fatal_driver = {
	.driver = {
		   .name = "fusa_fatal",
		   .of_match_table = hobot_fusa_fatal_of_match,
		   },
	.probe = fusa_fatal_probe,
	.remove = fusa_fatal_remove,
};

static int __init hobot_fusa_fatal_init(void)
{
	int retval = 0;
	/* Register the platform driver */
	retval = platform_driver_register(&hobot_fusa_fatal_driver);
	if (retval)
		pr_err("hobot fusa fatak api drvier failed\n");

	return retval;
}

static void __exit hobot_fusa_fatal_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&hobot_fusa_fatal_driver);
}

module_init(hobot_fusa_fatal_init);
module_exit(hobot_fusa_fatal_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hobot Inc.");
MODULE_DESCRIPTION("Driver for the HobotRobotics fusa fatal_api driver");
MODULE_VERSION("1.0");
