/*****************************************************************************
 ****			 COPYRIGHT NOTICE			****
 ****		 Copyright	2017 Horizon Robotics, Inc.	****
 ****			 All rights reserved.			****
 *****************************************************************************/
/**
 *
 * @author	guozheng.li(guozheng.li@horizon.ai)
 * @date	2019/06/16
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/major.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/platform_device.h>
#include <linux/time.h>

#define NODE_NUM	1
//#define MCU_IRQ_MODE  0

#define MCU_IRQ_IOC_MAGIC '$'

//SOC发送到MCU的中断触发类型
#define MCU_IOC_TRI_EDGE	_IOW(MCU_IRQ_IOC_MAGIC, 0, int)
//MCU发送到SOC的中断触发类型
#define MCU_IOC_IRQ_EDGE	_IOW(MCU_IRQ_IOC_MAGIC, 1, int)
//MCU发送通知管脚电平
#define MCU_IOC_IO_INPUT	_IOR(MCU_IRQ_IOC_MAGIC, 2, int)
//SOC发送时间同步中断
#define MCU_IOC_SYNC_TIME	_IOR(MCU_IRQ_IOC_MAGIC, 3, struct timeval)

struct mcu_irq_cdev {
	struct cdev cdev;
	const char *name;
	struct device *device[NODE_NUM];
	int major;
	int minor;
	int num_nodes;
	dev_t dev_num;
	struct class *drv_class;
	struct fasync_struct *mcuirq_async;
	wait_queue_head_t event_queue;
	spinlock_t lock;
	uint32_t event;
	unsigned long spinlock_flags;

	int time_sync_gpio;	/* triger mcu irq */
	int mcu_tx_data_gpio;	/* mcu triger */
	int tri_edge;
	int irq_edge;
	int virq;
};

struct mcu_irq_cdev g_mcu_irq_cdev;

static int mcu_irq_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &g_mcu_irq_cdev;
	return 0;
}

static ssize_t mcu_irq_read(struct file *filp, char __user *buf,
			    size_t size, loff_t *ppos)
{
	return 0;
}
#ifdef MCU_IRQ_MODE
static irqreturn_t mcu_data_ready_irq(int irq, void *devid)
{
	struct mcu_irq_cdev *info;

	info = (struct mcu_irq_cdev *)devid;
	info->event = 1;
	wake_up_interruptible(&info->event_queue);
	kill_fasync(&info->mcuirq_async, SIGIO, POLL_IN);
	return IRQ_HANDLED;
}
#endif
static ssize_t mcu_irq_write(struct file *filp, const char __user *buf,
			     size_t size, loff_t *ppos)
{
	struct mcu_irq_cdev *info;

	info = filp->private_data;
	if (info->tri_edge == IRQ_TYPE_EDGE_FALLING) {
		gpio_direction_output(info->time_sync_gpio, 0);
		mdelay(2);
		gpio_direction_output(info->time_sync_gpio, 1);
		mdelay(1);
	} else {
		gpio_direction_output(info->time_sync_gpio, 1);
		mdelay(2);
		gpio_direction_output(info->time_sync_gpio, 0);
		mdelay(1);
	}
	return 0;
}

static unsigned int mcu_irq_poll(struct file *filp,
				 struct poll_table_struct *wait)
{
	struct mcu_irq_cdev *info;
	unsigned int mask = 0;

	info = filp->private_data;
	poll_wait(filp, &info->event_queue, wait);
	spin_lock_irqsave(&info->lock, info->spinlock_flags);
	if (info->event != 0) {
		mask = EPOLLIN;
		info->event = 0;
	}
	spin_unlock_irqrestore(&info->lock, info->spinlock_flags);
	return mask;
}

static int mcu_irq_fasync(int fd, struct file *filp, int on)
{
	struct mcu_irq_cdev *info;

	info = filp->private_data;
	return fasync_helper(fd, filp, on, &info->mcuirq_async);
}

static long mcu_irq_ioctl(struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	int gpio_value;
	struct mcu_irq_cdev *info;
	struct timeval sync_time;

	info = filp->private_data;

	if (_IOC_TYPE(cmd) != MCU_IRQ_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case MCU_IOC_TRI_EDGE:
		info->tri_edge = (int)arg;
		pr_info("soc triger edge =%d\n", info->tri_edge);
		if (info->tri_edge == IRQ_TYPE_EDGE_FALLING) {
			gpio_direction_output(info->time_sync_gpio, 1);
			mdelay(1);
		} else {
			gpio_direction_output(info->time_sync_gpio, 0);
			mdelay(1);
		}
		retval = 0;
		break;
	case MCU_IOC_IRQ_EDGE:
		info->irq_edge = (int)arg;
		retval = irq_set_irq_type(info->virq, info->irq_edge);
		retval = 0;
		if (retval < 0) {
			retval = -EINVAL;
			pr_err("%s irq edge set err\n", __func__);
		}
		pr_info("mcu send irq edge=%d\n", info->irq_edge);
		break;
	case MCU_IOC_IO_INPUT:
		gpio_value = gpio_get_value(info->mcu_tx_data_gpio);
		if (copy_to_user
		    ((void __user *)arg, &gpio_value, _IOC_SIZE(cmd))) {
			pr_err("%s: copy data to userspace failed\n", __func__);
			retval = -EFAULT;
		}
		break;
	case MCU_IOC_SYNC_TIME:
		if (info->tri_edge == IRQ_TYPE_EDGE_FALLING) {
			gpio_direction_output(info->time_sync_gpio, 0);
			do_gettimeofday(&sync_time);
			mdelay(2);
			gpio_direction_output(info->time_sync_gpio, 1);
			mdelay(1);
		} else {
			gpio_direction_output(info->time_sync_gpio, 1);
			do_gettimeofday(&sync_time);
			mdelay(2);
			gpio_direction_output(info->time_sync_gpio, 0);
			mdelay(1);
		}
		if (copy_to_user
		    ((void __user *)arg, &sync_time, _IOC_SIZE(cmd))) {
			pr_err("%s: copy data to userspace failed\n", __func__);
			retval = -EFAULT;
		}
		break;
	default:
		retval = -EINVAL;
		break;
	}

	return retval;
}

static int mcu_irq_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations mcu_irq_fops = {
	.owner = THIS_MODULE,
	.read = mcu_irq_read,
	.write = mcu_irq_write,
	.unlocked_ioctl = mcu_irq_ioctl,
	.open = mcu_irq_open,
	.release = mcu_irq_release,
	.fasync = mcu_irq_fasync,
	.poll = mcu_irq_poll,
};

static int mcu_irq_init_chrdev(struct mcu_irq_cdev *cdev)
{
	int rc = 0;
	dev_t dev;
	int devnum, i, minor, major;

	rc = alloc_chrdev_region(&dev, 0, cdev->num_nodes, cdev->name);
	if (rc) {
		pr_err("Failed to obtain major/minors");
		return rc;
	}

	cdev->major = major = MAJOR(dev);
	cdev->minor = minor = MINOR(dev);

	cdev_init(&cdev->cdev, &mcu_irq_fops);
	cdev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&cdev->cdev, MKDEV(major, minor), cdev->num_nodes);
	if (rc) {
		pr_err("Failed to add cdev. Aborting.\n");
		goto unregister_chrdev;
	}

	cdev->drv_class = class_create(THIS_MODULE, cdev->name);
	if (IS_ERR(cdev->drv_class)) {
		rc = IS_ERR(cdev->drv_class);
		pr_err("Failed to class_create. Aborting.\n");
		goto dest_class;
	}

	for (i = minor, devnum = 0; devnum < cdev->num_nodes; devnum++, i++) {

		cdev->device[i] = device_create(cdev->drv_class,
						NULL,
						MKDEV(major, i),
						NULL, "%s%d", cdev->name,
						devnum);

		if (IS_ERR(cdev->device[i])) {
			pr_err("Failed to create %s%d device. Aborting.\n",
			       cdev->name, devnum);
			rc = -ENODEV;
			goto unroll_device_create;
		}
	}

	return 0;

unroll_device_create:
	devnum--;
	i--;
	for (; devnum >= 0; devnum--, i--)
		device_destroy(cdev->drv_class, MKDEV(major, i));

	class_destroy(cdev->drv_class);
dest_class:
	cdev_del(&cdev->cdev);
unregister_chrdev:
	unregister_chrdev_region(MKDEV(major, minor), cdev->num_nodes);

	return rc;
}

static int mcu_irq_free_chrdev(struct mcu_irq_cdev *cdev)
{
	int i;

	for (i = 0; i < NODE_NUM; i++)
		device_destroy(cdev->drv_class, MKDEV(cdev->major, i));
	cdev_del(&cdev->cdev);
	unregister_chrdev_region(MKDEV(cdev->major, 0), cdev->dev_num);
	class_destroy(cdev->drv_class);

	return 0;
}

static int mcu_irq_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct mcu_irq_cdev *info;

	info = &g_mcu_irq_cdev;
	rc = of_property_read_u32(pdev->dev.of_node, "mcu_tri_pin",
				  &info->time_sync_gpio);
	if (rc) {
		dev_err(&pdev->dev, "Filed to get mcu_tri_pin\n");
		rc = -ENODEV;
		goto fail;
	}
	rc = of_property_read_u32(pdev->dev.of_node, "mcu_irq_pin",
				  &info->mcu_tx_data_gpio);
	if (rc) {
		dev_err(&pdev->dev, "Filed to get mcu_irq_pin\n");
		rc = -ENODEV;
		goto fail;
	}

	info->name = "mcu_irq";
	info->num_nodes = 1;

	rc = mcu_irq_init_chrdev(info);
	if (rc)
		goto fail;

	pr_info("time_sync_gpio=%d mcu_tx_data_gpio=%d\n", info->time_sync_gpio,
		info->mcu_tx_data_gpio);

#ifdef MCU_IRQ_MODE
	if (gpio_is_valid(info->mcu_tx_data_gpio)) {

		rc = gpio_request(info->mcu_tx_data_gpio, "mcu_irq_gpio");
		if (rc < 0) {
			pr_err("%s gpio_request err\n", __func__);
			rc = -ENODEV;
			goto gpio_fail;
		}
		info->virq = gpio_to_irq(info->mcu_tx_data_gpio);
		rc = devm_request_irq(info->device[0], info->virq,
				      mcu_data_ready_irq, IRQ_TYPE_NONE,
				      "mcu_data_ready_irq", (void *)info);
		if (rc < 0) {
			pr_err("%s request_irq err\n", __func__);
			rc = -ENODEV;
			goto gpio_fail;
		}
		rc = gpio_direction_input(info->mcu_tx_data_gpio);
		if (rc < 0) {
			pr_err("%s gpio_direction_input err\n", __func__);
			rc = -ENODEV;
			goto gpio_fail;
		}
	}
#endif
	rc = gpio_request(info->time_sync_gpio, "time_sync_gpio");
	if (rc < 0) {
		pr_err("mcu irq: Err get time_sync_gpio ret = %d\n", rc);
		rc = -ENODEV;
		goto gpio_fail;
	}

	init_waitqueue_head(&info->event_queue);
	spin_lock_init(&info->lock);
	return 0;

gpio_fail:
	mcu_irq_free_chrdev(info);
fail:
	return rc;
}

static int mcu_irq_remove(struct platform_device *pdev)
{
	struct mcu_irq_cdev *info;

	info = &g_mcu_irq_cdev;
	disable_irq(info->virq);
	devm_free_irq(info->device[0], info->virq, (void *)info);
	gpio_free(info->time_sync_gpio);
	gpio_free(info->mcu_tx_data_gpio);
	mcu_irq_free_chrdev(info);
	return 0;
}

static const struct of_device_id mcu_irq_of_match[] = {
	{.compatible = "hobot,mcu_irq"},
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, mcu_irq_of_match);

static struct platform_driver mcu_irq_driver = {
	.probe = mcu_irq_probe,
	.remove = mcu_irq_remove,
	.driver = {
		   .name = "mcu_irq",
		   .of_match_table = mcu_irq_of_match,
		   },
};

module_platform_driver(mcu_irq_driver);

MODULE_DESCRIPTION("Driver for mcu irq");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL v2");
