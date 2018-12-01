/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
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
#include <asm/io.h>
#include <linux/mutex.h>
#include "x2_iar.h"

#define IAR_DMA_MODE

#define IAR_CDEV_MAGIC 'R'
#define IAR_GETVALUE        _IOR(IAR_CDEV_MAGIC,0x11, unsigned int)
#define IAR_START           _IO(IAR_CDEV_MAGIC,0x12)
#define IAR_STOP            _IO(IAR_CDEV_MAGIC,0x13)
#define IAR_CHANNEL_CFG     _IOW(IAR_CDEV_MAGIC,0x14, channel_base_cfg_t)
#define IAR_DISPLAY_UPDATE  _IOW(IAR_CDEV_MAGIC,0x15, update_cmd_t)
#define IAR_GAMMA_CFG       _IOW(IAR_CDEV_MAGIC,0x17, gamma_cfg_t)
#define IAR_SCALE_CFG       _IOW(IAR_CDEV_MAGIC,0x18, upscaling_cfg_t)
#define IAR_OUTPUT_CFG      _IOW(IAR_CDEV_MAGIC,0x19, output_cfg_t)

typedef struct _update_cmd_t {
	unsigned int enable_flag[IAR_CHANNEL_MAX];
	unsigned int frame_size[IAR_CHANNEL_MAX];
	frame_buf_t srcframe[IAR_CHANNEL_MAX];
} update_cmd_t;

struct iar_cdev_s {
	const char *name;
	int major;
	int minor;
	struct cdev cdev;
	dev_t dev_num;
	struct class *iar_classes;
	struct completion completion;
	frame_buf_t *framebuf_user[IAR_CHANNEL_MAX];
	struct mutex iar_mutex;
};
struct iar_cdev_s *g_iar_cdev;

//cache clean*inv tmp use, replace when has formal version
extern void __dma_map_area(const void *, size_t, int);
extern void __dma_unmap_area(const void *, size_t, int);
void dma_cache_sync(const void *vir_addr, size_t size, int direction)
{
	switch (direction) {
	case DMA_FROM_DEVICE:	/* invalidate only */
		__dma_unmap_area(vir_addr, size, direction);
		break;
	case DMA_TO_DEVICE:	/* writeback only */
		__dma_map_area(vir_addr, size, direction);
		break;
	default:
		BUG();
	}
}

void invalid_cache(unsigned char *data, int len)
{
	dma_cache_sync(data, len, DMA_FROM_DEVICE);
}

EXPORT_SYMBOL(invalid_cache);

void clean_cache(unsigned char *data, int len)
{
	dma_cache_sync(data, len, DMA_TO_DEVICE);
}

EXPORT_SYMBOL(clean_cache);

int32_t iar_write_framebuf_poll(uint32_t channel, void __user * srcaddr,
				uint32_t size)
{
	frame_buf_t *bufaddr;
	bufaddr = iar_get_framebuf_addr(channel);
	if (copy_from_user(bufaddr->vaddr, srcaddr, size))
		return -EFAULT;

	IAR_DEBUG_PRINT
	    ("iar_write_framebuf_poll :%d vaddr:0x%p paddr:0x%llx,size:%d\n",
	     channel, bufaddr->vaddr, bufaddr->paddr, size);

	return iar_switch_buf(channel);

}

static void iar_edma_callback(void *data)
{
	complete(&g_iar_cdev->completion);
}

int32_t iar_write_framebuf_dma(uint32_t channel, phys_addr_t srcaddr,
			       uint32_t size)
{
	struct dma_chan *ch;
	int ret = 0;
	struct dma_async_tx_descriptor *tx;
	dma_cap_mask_t mask;
	dma_cookie_t cookie;
	frame_buf_t *bufaddr;

	reinit_completion(&g_iar_cdev->completion);
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	ch = dma_request_channel(mask, NULL, NULL);
	if (!ch) {
		printk(KERN_ERR "%s: dma_request_channel failed\n", __func__);
		return -1;
	}
	bufaddr = iar_get_framebuf_addr(channel);

	tx = ch->device->device_prep_dma_memcpy(ch, bufaddr->paddr, srcaddr,
						size, 0);
	if (!tx) {
		printk(KERN_ERR "%s: device_prep_dma_memcpy failed\n",
		       __func__);
		return -EIO;
	}

	tx->callback = iar_edma_callback;
	tx->callback_result = NULL;
	tx->callback_param = g_iar_cdev;
	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret) {
		printk(KERN_ERR "dma_submit_error %d\n", cookie);
		return -EIO;
	}
	dma_async_issue_pending(ch);
	ret =
	    wait_for_completion_timeout(&g_iar_cdev->completion,
					msecs_to_jiffies(1000));
	dma_release_channel(ch);
	if (!ret) {
		printk(KERN_ERR "%s: timeout !!\n", __func__);
		return -EIO;
	}
	iar_switch_buf(channel);
	IAR_DEBUG_PRINT("DMA trans done\n");

	return 0;
}

int32_t iar_display_update(update_cmd_t * update_cmd)
{
	int index = 0;
	int ret = -1;
	for (index = IAR_CHANNEL_1; index < IAR_CHANNEL_MAX; index++) {
		if (index == IAR_CHANNEL_2 || index == IAR_CHANNEL_4)
			continue;	//TODO, now channnel 2 and 4 is disable
		IAR_DEBUG_PRINT("update_cmd->enable_flag[index]:%d addr:0x%p\n",
				update_cmd->enable_flag[index],
				update_cmd->srcframe[index].vaddr);
		if (update_cmd->enable_flag[index]
		    && update_cmd->frame_size[index] < MAX_FRAME_BUF_SIZE) {
#ifdef IAR_DMA_MODE
			if (g_iar_cdev->framebuf_user[index]) {
				clean_cache(g_iar_cdev->framebuf_user[index]->
					    vaddr,
					    update_cmd->frame_size[index]);
				ret =
				    iar_write_framebuf_dma(index,
							   g_iar_cdev->
							   framebuf_user
							   [index]->paddr,
							   update_cmd->
							   frame_size[index]);
			}
#else
			if (update_cmd->srcframe[index].vaddr)
				ret =
				    iar_write_framebuf_poll(index,
							    update_cmd->srcframe
							    [index].vaddr,
							    update_cmd->
							    frame_size[index]);
#endif
		}
	}
	return ret;
}

static int iar_cdev_open(struct inode *inode, struct file *filp)
{
	struct iar_cdev_s *iarcdev_p;

	iarcdev_p = container_of(inode->i_cdev, struct iar_cdev_s, cdev);
	filp->private_data = iarcdev_p;
	return iar_open();
}

static long iar_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
{
	int ret;
	void __user *arg = (void __user *)p;
	mutex_lock(&g_iar_cdev->iar_mutex);
	switch (cmd) {
	case IAR_START:
		{
			IAR_DEBUG_PRINT("IAR_START \n");
			ret = iar_start(1);
		}
		break;
	case IAR_STOP:
		{
			IAR_DEBUG_PRINT("IAR_STOP \n");
			ret = iar_stop();
		}
		break;
	case IAR_DISPLAY_UPDATE:
		{
			update_cmd_t update_cmd;
			IAR_DEBUG_PRINT("IAR_DISPLAY_UPDATE \n");
			if (copy_from_user
			    (&update_cmd, arg, sizeof(update_cmd_t)))
				return -EFAULT;
			ret = iar_display_update(&update_cmd);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_CHANNEL_CFG:
		{
			channel_base_cfg_t channel_cfg;
			IAR_DEBUG_PRINT("IAR_CHANNEL_CFG \n");
			if (copy_from_user
			    (&channel_cfg, arg, sizeof(channel_base_cfg_t)))
				return -EFAULT;
			ret = iar_channel_base_cfg(&channel_cfg);
		}
		break;
	case IAR_GAMMA_CFG:
		{
			gamma_cfg_t gamma_cfg;
			IAR_DEBUG_PRINT("IAR_GAMMA_CFG \n");
			if (copy_from_user
			    (&gamma_cfg, arg, sizeof(gamma_cfg_t)))
				return -EFAULT;
			ret = iar_gamma_cfg(&gamma_cfg);
		}
		break;
	case IAR_SCALE_CFG:
		{
			upscaling_cfg_t upscaling_cfg;
			IAR_DEBUG_PRINT("IAR_SCALE_CFG \n");
			if (copy_from_user
			    (&upscaling_cfg, arg, sizeof(upscaling_cfg_t)))
				return -EFAULT;
			ret = iar_upscaling_cfg(&upscaling_cfg);
		}
		break;
	case IAR_OUTPUT_CFG:
		{
			output_cfg_t output_cfg;
			IAR_DEBUG_PRINT("IAR_OUTPUT_CFG \n");
			if (copy_from_user
			    (&output_cfg, arg, sizeof(output_cfg_t)))
				return -EFAULT;
			ret = iar_output_cfg(&output_cfg);
		}
		break;
	default:
		ret = -EPERM;
		break;
	}
	mutex_unlock(&g_iar_cdev->iar_mutex);
	return ret;
}

static ssize_t iar_cdev_write(struct file *filp, const char __user * ubuf,
			      size_t len, loff_t * ppos)
{
	return len;
}

static ssize_t iar_cdev_read(struct file *filp, char __user * ubuf,
			     size_t len, loff_t * offp)
{
	unsigned int size;
	size = 0;
	return size;
}

int iar_cdev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return iar_close();
}

static const struct file_operations iar_cdev_ops = {
	.owner = THIS_MODULE,
	.open = iar_cdev_open,
	.release = iar_cdev_release,
	.write = iar_cdev_write,
	.read = iar_cdev_read,
	.unlocked_ioctl = iar_cdev_ioctl,
};

struct kobject *x2_iar_kobj;
static ssize_t x2_iar_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	char *s = buf;

	x2_iar_dump();

	return (s - buf);
}

static ssize_t x2_iar_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t n)
{
	int error = -EINVAL;
	return error ? error : n;
}

static struct kobj_attribute iar_test_attr = {
	.attr = {
		 .name = __stringify(iar_test_attr),
		 .mode = 0644,
		 },
	.show = x2_iar_show,
	.store = x2_iar_store,
};

static struct attribute *attributes[] = {
	&iar_test_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attributes,
};

int __init iar_cdev_init(void)
{
	int error;

	g_iar_cdev = kmalloc(sizeof(struct iar_cdev_s), GFP_KERNEL);
	if (!g_iar_cdev) {
		printk(KERN_ERR "Unable to alloc IAR DEV\n");
		return -ENOMEM;
	}
	g_iar_cdev->name = "iar_cdev";
	mutex_init(&g_iar_cdev->iar_mutex);
	init_completion(&g_iar_cdev->completion);
	g_iar_cdev->iar_classes = class_create(THIS_MODULE, g_iar_cdev->name);
	if (IS_ERR(g_iar_cdev->iar_classes))
		return PTR_ERR(g_iar_cdev->iar_classes);

	error =
	    alloc_chrdev_region(&g_iar_cdev->dev_num, 0, 1, g_iar_cdev->name);
	if (!error) {
		g_iar_cdev->major = MAJOR(g_iar_cdev->dev_num);
		g_iar_cdev->minor = MINOR(g_iar_cdev->dev_num);
	}

	cdev_init(&g_iar_cdev->cdev, &iar_cdev_ops);

	error = cdev_add(&g_iar_cdev->cdev, g_iar_cdev->dev_num, 1);
	if (error) {
		unregister_chrdev_region(g_iar_cdev->dev_num, 1);
		return error;
	}

	device_create(g_iar_cdev->iar_classes, NULL, g_iar_cdev->dev_num, NULL,
		      g_iar_cdev->name);

	g_iar_cdev->framebuf_user[IAR_CHANNEL_1] =
	    x2_iar_get_framebuf_addr(IAR_CHANNEL_1);
	g_iar_cdev->framebuf_user[IAR_CHANNEL_3] =
	    x2_iar_get_framebuf_addr(IAR_CHANNEL_3);

	x2_iar_kobj = kobject_create_and_add("x2_iar", NULL);
	if (!x2_iar_kobj)
		return -ENOMEM;
	return sysfs_create_group(x2_iar_kobj, &attr_group);

	return 0;
}

void __exit iar_cdev_exit(void)
{
	device_destroy(g_iar_cdev->iar_classes, g_iar_cdev->dev_num);
	class_destroy(g_iar_cdev->iar_classes);
	cdev_del(&g_iar_cdev->cdev);
	unregister_chrdev_region(g_iar_cdev->dev_num, 1);

	if (x2_iar_kobj) {
		sysfs_remove_group(x2_iar_kobj, &attr_group);
		kobject_del(x2_iar_kobj);
	}
}

module_init(iar_cdev_init);
module_exit(iar_cdev_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: x2");
