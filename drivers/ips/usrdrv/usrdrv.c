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
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include "usrdrv.h"

#define CONFIG_USRDRV_DEBUG

/************************************************
* Driver Struct
************************************************/
typedef struct usrdrv_map_info_s {
	uint64_t phys_addr;
	uint64_t virt_addr;
	uint64_t length;
} usrdrv_map_info_t;

typedef struct usrdrv_dev_pri_s {
	uint32_t reserve;
} usrdrv_dev_pri_t;

typedef struct usrdrv_file_pri_s {
	struct device *usrdrv_device;
	uint32_t irq_num;
	uint8_t irq_name[USRDRV_IRQNAME_MAX];
	uint8_t irq_enable;
	usrdrv_map_info_t map;
	uint32_t event;
	spinlock_t event_lock;
	wait_queue_head_t event_queue;
	uint32_t irq_auto_fire;
	uint32_t irq_auto_fire_cnt;
} usrdrv_file_pri_t;

/************************************************
* Global variable
************************************************/
static struct class *g_usrdrv_class;
static struct device *g_usrdrv_dev;
static int usrdrv_major;
struct cdev usrdrv_cdev;

/************************************************
* Utility
************************************************/
#define usrdrv_file_pri(f) ((usrdrv_file_pri_t *)((f)->private_data))
#define usrdrv_dev_pri(f) (dev_get_drvdata(usrdrv_file_pri(f)->usrdrv_device))

#ifdef CONFIG_USRDRV_DEBUG
#define usrdrvinfo(format, ...)   printk(KERN_INFO format "\n", ##__VA_ARGS__)
#define usrdrverr(format, ...)    printk(KERN_ERR format "\n", ##__VA_ARGS__)
#else				//CONFIG_USRDRV_DEBUG
#define usrdrvinfo(format, ...)
#define usrdrverr(format, ...)    printk(KERN_ERR format "\n", ##__VA_ARGS__)
#endif				//CONFIG_USRDRV_DEBUG

#ifndef VM_RESERVED		/*for kernel up to 3.7.0 version */
#define VM_RESERVED   (VM_DONTEXPAND | VM_DONTDUMP)
#endif
/************************************************
* Functions
************************************************/
static irqreturn_t usrdrv_irq(int irq, void *data)
{
	usrdrv_file_pri_t *fpri;
	if (NULL == data) {
		usrdrverr("usrdrv irq input data error!");
		return IRQ_HANDLED;
	}

	fpri = (usrdrv_file_pri_t *) data;

	//@fixme 2, need add a variable for interrupt enable/disable
	spin_lock(&fpri->event_lock);
	disable_irq_nosync(fpri->irq_num);
	fpri->event |= true;
	fpri->irq_enable = 0;
	spin_unlock(&fpri->event_lock);
	if (fpri->event)
		wake_up_interruptible(&fpri->event_queue);
	return IRQ_HANDLED;
}

static int usrdrv_open(struct inode *inode, struct file *file)
{
	usrdrv_file_pri_t *fpri = NULL;

	fpri = (usrdrv_file_pri_t *) kzalloc(sizeof(usrdrv_file_pri_t), GFP_KERNEL);

	if (fpri == NULL) {
		printk(KERN_ERR "file_pri malloc failed\n");
		return -1;
	}

	file->private_data = fpri;
	spin_lock_init(&fpri->event_lock);
	init_waitqueue_head(&fpri->event_queue);
	fpri->irq_enable = 0;
	fpri->event = 0;
	fpri->usrdrv_device = g_usrdrv_dev;

	return 0;
}

static int usrdrv_close(struct inode *inode, struct file *file)
{
	if (file->private_data) {
		kfree(file->private_data);
		file->private_data = NULL;
	}
	return 0;
}

void usrdrv_irq_set_pending(unsigned int irq)
{
	irq_set_irqchip_state(irq, IRQCHIP_STATE_PENDING, 1);
}

static bool usrdrv_irq_get_mask(unsigned int irq)
{
	bool masked;
	irq_get_irqchip_state(irq, IRQCHIP_STATE_MASKED, &masked);
	return masked;
}

static unsigned int usrdrv_poll(struct file *file,
				struct poll_table_struct *wait)
{
	usrdrv_file_pri_t *fpri = usrdrv_file_pri(file);
	unsigned int mask = 0;
	unsigned long flags;

	//@fixme 1, it maybe have chance to lost interrupt before poll_wait()
	//@fixme 2, need add a variable for interrupt enable/disable
	spin_lock_irqsave(&fpri->event_lock, flags);
	if ((fpri->irq_enable == 0) || usrdrv_irq_get_mask(fpri->irq_num) == 1) {
		fpri->irq_enable = 1;
		enable_irq(fpri->irq_num);
	} else {
		usrdrverr("usrdrv irq input data error!");
		usrdrverr("auto fire %d, irq_enable=%d, irq_auto_fire_cnt=%d",
				fpri->irq_num, fpri->irq_enable,
				fpri->irq_auto_fire_cnt);
	}
	if (fpri->irq_auto_fire) {
		usrdrv_irq_set_pending(fpri->irq_num);
		fpri->irq_auto_fire_cnt++;
	}
	spin_unlock_irqrestore(&fpri->event_lock, flags);

	poll_wait(file, &fpri->event_queue, wait);
	spin_lock_irqsave(&fpri->event_lock, flags);
	//mask = EPOLLHUP; //for USRDRV_STOP
	//mask = EPOLLPRI; //for USRDRV_MOTDET
	//mask = EPOLLERR; //for USRDRV_ERROR
	if (fpri->event) {
		mask = EPOLLIN | EPOLLET;
		fpri->event = 0;
	}
	spin_unlock_irqrestore(&fpri->event_lock, flags);
	return mask;
}

static int usrdrv_mmap(struct file *file, struct vm_area_struct *vm)
{
	unsigned long pfn;
	usrdrv_file_pri_t *fpri = usrdrv_file_pri(file);

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = vm->vm_pgoff;
	fpri->map.phys_addr = pfn << PAGE_SHIFT;
	fpri->map.virt_addr = vm->vm_start;
	fpri->map.length = vm->vm_end - vm->vm_start;
	printk(KERN_INFO "[%s] phys=%llx, virt=%llx, length=%llu\n",
			__func__,
			fpri->map.phys_addr,
			fpri->map.virt_addr,
			fpri->map.length);

	return remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			       vm->vm_page_prot) ? -EAGAIN : 0;
}

static long usrdrv_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	usrdrv_file_pri_t *fpri = usrdrv_file_pri(file);

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != USRDRV_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case USRDRV_IRQ_REGISTER: {
		ioctl_irq_info_t irq_info;
		struct device_node *np;
		uint32_t irq_in_dts;
		if (copy_from_user
		    ((void *)&irq_info, (void __user *)arg,
		     sizeof(ioctl_irq_info_t))) {
			usrdrverr
			("ERROR: usrdrv copy data from user failed\n");
			return -EINVAL;
		}

		np = of_find_compatible_node(NULL, NULL,
					     "hobot,usrdrv");
		if (!np) {
			return -ENODEV;
		}

		/* PTP2 IRQ */
		irq_in_dts = irq_of_parse_and_map(np, irq_info.irq_index);
		fpri->irq_num = irq_in_dts;
		strncpy(fpri->irq_name, irq_info.irq_name, USRDRV_IRQNAME_MAX);

		printk("usrdrv irq index=%d, request_irq: num=%d,handler=%p,flag=%x,name=%s,data=%p\n",
				irq_info.irq_index, fpri->irq_num, usrdrv_irq,
				IRQF_TRIGGER_HIGH, fpri->irq_name, fpri);

		ret = request_irq(fpri->irq_num, usrdrv_irq,
				IRQF_TRIGGER_HIGH, fpri->irq_name,
				fpri);

		if (ret)
			usrdrverr("ERROR: usrdrv request_irq() failed: %d\n", ret);

		//@fixme why need disable first
		disable_irq_nosync(fpri->irq_num);
		fpri->irq_auto_fire_cnt = 0;
	}
	break;
	case USRDRV_IRQ_UNREGISTER: {
		const char *devname;
		devname = free_irq(fpri->irq_num, fpri);
		printk("free irq %d, devname %p\n", fpri->irq_num,
		       devname);
		if (fpri->irq_auto_fire)
			printk("irq %d: irq_auto_fire_cnt = %d\n",
			       fpri->irq_num, fpri->irq_auto_fire_cnt);
	}
	break;
	case USRDRV_IRQ_ENABLE: {
		uint32_t irq_enable;
		//unsigned long flags;
		if (copy_from_user((void *)&irq_enable, (void __user *)arg,
					sizeof(uint32_t))) {
			usrdrverr("ERROR: usrdrv copy data from user failed\n");
			return -EINVAL;
		}
		//spin_lock_irqsave(&fpri->event_lock, flags);
		if (irq_enable) {
			printk("usrdrv enable\n");
			if (fpri->irq_enable == 0) {
				enable_irq(fpri->irq_num);
				fpri->irq_enable = 1;
			}
		} else {
			printk("usrdrv disable\n");
			if (fpri->irq_enable == 1) {
				disable_irq_nosync(fpri->irq_num);
				fpri->irq_enable = 0;
			}
		}
		//spin_unlock_irqrestore(&fpri->event_lock, flags);
	}
	break;
	case USRDRV_IRQ_AUTO_FIRE: {
		uint32_t irq_auto_fire;
		if (copy_from_user((void *)&irq_auto_fire, (void __user *)arg,
					sizeof(uint32_t))) {
			usrdrverr("ERROR: usrdrv copy data from user failed\n");
			return -EINVAL;
		}
		if (irq_auto_fire) {
			printk("usrdrv auto fire: enable\n");
			fpri->irq_auto_fire = 1;
		} else {
			printk("usrdrv auto fire: disable\n");
			fpri->irq_auto_fire = 0;
		}
	}
	break;
	default:
		usrdrverr("usrdrv cmd 0x%x not support\n", cmd);
		break;
	}
	return ret;
}

static struct file_operations usrdrv_fops = {
	.owner = THIS_MODULE,
	.open = usrdrv_open,
	.poll = usrdrv_poll,
	.release = usrdrv_close,
	.unlocked_ioctl = usrdrv_ioctl,
	.compat_ioctl = usrdrv_ioctl,
	.mmap = usrdrv_mmap,
};

static int __init usrdrv_module_init(void)
{
	int ret = 0;
	dev_t devno;
	struct cdev *p_cdev = &usrdrv_cdev;
	usrdrv_dev_pri_t *dpri = NULL;

	printk(KERN_INFO "usrdrv driver init enter\n");
	dpri = kzalloc(sizeof(usrdrv_dev_pri_t), GFP_KERNEL);
	if (dpri == NULL) {
		printk(KERN_ERR "usrdrv malloc failed\n");
		return -1;
	}

	ret = alloc_chrdev_region(&devno, 0, 1, "usrdrv");
	if (ret < 0) {
		printk(KERN_ERR "Error %d while alloc chrdev usrdrv", ret);
		goto err;
	}

	usrdrv_major = MAJOR(devno);
	cdev_init(p_cdev, &usrdrv_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		printk(KERN_ERR "Error %d while adding x2 usrdrv cdev", ret);
		goto err;
	}

	g_usrdrv_class = class_create(THIS_MODULE, "usrdrv");
	if (IS_ERR(g_usrdrv_class)) {
		printk(KERN_INFO "[%s:%d] class_create error\n", __func__,
		       __LINE__);
		ret = PTR_ERR(g_usrdrv_class);
		goto err;
	}

	g_usrdrv_dev = device_create(g_usrdrv_class, NULL, MKDEV(usrdrv_major, 0),
			(void *)dpri, "usrdrv");
	if (IS_ERR(g_usrdrv_dev)) {
		printk(KERN_ERR "[%s] deivce create error\n", __func__);
		ret = PTR_ERR(g_usrdrv_dev);
		goto err;
	}

	printk(KERN_INFO "usrdrv driver init exit\n");
	return 0;
err:
	class_destroy(g_usrdrv_class);
	cdev_del(&usrdrv_cdev);
	unregister_chrdev_region(MKDEV(usrdrv_major, 0), 1);
	kzfree(dpri);
	return ret;
}

static void __exit usrdrv_module_exit(void)
{
	usrdrv_dev_pri_t *dpri = dev_get_drvdata(g_usrdrv_dev);
	printk(KERN_INFO "[%s:%d] usrdrv_exit\n", __func__, __LINE__);
	device_destroy(g_usrdrv_class, MKDEV(usrdrv_major, 0));
	class_destroy(g_usrdrv_class);
	cdev_del(&usrdrv_cdev);
	unregister_chrdev_region(MKDEV(usrdrv_major, 0), 1);
	kzfree(dpri);
	return;
}

module_init(usrdrv_module_init);
module_exit(usrdrv_module_exit);
static const struct of_device_id usrdrv_of_match[] = {
	{.compatible = "hobot,usrdrv"},
	{},
};

MODULE_DEVICE_TABLE(of, usrdrv_of_match);

MODULE_LICENSE("GPL v2");
