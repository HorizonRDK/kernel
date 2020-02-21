/***************************************************************************
 *						COPYRIGHT NOTICE
 *			   Copyright 2018 Horizon Robotics, Inc.
 *					   All rights reserved.
 ***************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/mm.h>
#include <linux/types.h>
#include <linux/major.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <asm/io.h>
#include <linux/mutex.h>
#include <soc/hobot/hobot_iar.h>

struct iar_mmap_s {
	const char	*name;
	int major;
	int minor;
	struct cdev cdev;
	dev_t dev_num;
	struct class *iar_class;
	frame_buf_t *framebuf_user[IAR_CHANNEL_4];
};
struct iar_mmap_s *g_iar_mmap;

static int iar_mmap_open(struct inode *inode, struct file *filp)
{
	struct iar_mmap_s *iarcdev_p;
	unsigned int channel;
	iarcdev_p = container_of(inode->i_cdev, struct iar_mmap_s, cdev);
	channel = MINOR(inode->i_rdev) - iarcdev_p->minor;
	iarcdev_p->framebuf_user[channel] = x2_iar_get_framebuf_addr(channel);
	filp->private_data = iarcdev_p->framebuf_user[channel];
	printk("iar_mmap_open:%d\n", channel);
	return 0;
}

static long iar_mmap_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
{
	int ret = 0;
	return ret;
}

static ssize_t iar_mmap_write(struct file *filp, const char __user *ubuf,
							  size_t len, loff_t *ppos)
{
	return len;
}

static ssize_t iar_mmap_read(struct file *filp, char __user *ubuf,
							 size_t len, loff_t *offp)
{
	unsigned int size;
	size = 0;
	return size;
}

int iar_mmap_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

int iar_mmap_mmap(struct file *filp, struct vm_area_struct *pvma)
{

	frame_buf_t *framebuf = filp->private_data;
	printk("iar_mmap! \n");

	if (!framebuf || (pvma->vm_end - pvma->vm_start) > MAX_FRAME_BUF_SIZE) {
		return -ENOMEM;
	}

	pvma->vm_flags |= VM_IO;
	pvma->vm_flags |= VM_LOCKED;
	if (remap_pfn_range(pvma, pvma->vm_start,
						framebuf->paddr >> PAGE_SHIFT,
						pvma->vm_end - pvma->vm_start,
						pvma->vm_page_prot)) {
		printk(KERN_ERR "iar_mmap fail\n");
		return -EAGAIN;
	}
	printk("iar_mmap end!:%llx \n", framebuf->paddr);
	return 0;
}

static const struct file_operations iar_mmap_ops = {
	.owner = THIS_MODULE,
	.mmap = iar_mmap_mmap,
	.open = iar_mmap_open,
	.release = iar_mmap_release,
	.write = iar_mmap_write,
	.read = iar_mmap_read,
	.unlocked_ioctl = iar_mmap_ioctl,
};

int __init iar_mmap_init(void)
{
	int error, index;

	g_iar_mmap = kmalloc(sizeof(struct iar_mmap_s), GFP_KERNEL);
	if (!g_iar_mmap) {
		printk(KERN_ERR "Unable to alloc IAR DEV\n");
		return -ENOMEM;
	}
	g_iar_mmap->iar_class = fb_class;

	g_iar_mmap->name = "iar_mmap";
	error = alloc_chrdev_region(&g_iar_mmap->dev_num, 0, IAR_CHANNEL_MAX, g_iar_mmap->name);
	if (!error) {
		g_iar_mmap->major = MAJOR(g_iar_mmap->dev_num);
		g_iar_mmap->minor = MINOR(g_iar_mmap->dev_num);
	}

	cdev_init(&g_iar_mmap->cdev, &iar_mmap_ops);

	error = cdev_add(&g_iar_mmap->cdev, g_iar_mmap->dev_num, IAR_CHANNEL_MAX);
	if (error) {
		unregister_chrdev_region(g_iar_mmap->dev_num, IAR_CHANNEL_MAX);
		return error;
	}

	//device_create(g_iar_mmap->iar_class, NULL, g_iar_mmap->dev_num, NULL, g_iar_mmap->name);

	for (index = 0; index < IAR_CHANNEL_MAX; index++) {
		char name[64];
		dev_t dev = MKDEV(g_iar_mmap->major, g_iar_mmap->minor) + index;
		sprintf(name, "iar_channel_%d", index);
		device_create(g_iar_mmap->iar_class, NULL, dev, NULL, name);
	}

	return 0;
}

void __exit iar_mmap_exit(void)
{
	device_destroy(g_iar_mmap->iar_class, g_iar_mmap->dev_num);
	cdev_del(&g_iar_mmap->cdev);
	unregister_chrdev_region(g_iar_mmap->dev_num, 1);
}

module_init(iar_mmap_init);
module_exit(iar_mmap_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: x2");

