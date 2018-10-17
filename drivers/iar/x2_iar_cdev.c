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

#ifdef CONFIG_X2_FPGA
#define IAR_REG_READ_1(addr) readl(addr)
#define IAR_REG_WRITE_1(value, addr) writel(value, addr)
#else
extern int32_t bifdev_get_cpchip_reg(uint32_t addr, int32_t * value);
extern int32_t bifdev_set_cpchip_reg(uint32_t addr, int32_t value);
uint32_t iar_read_reg_1(uint32_t addr)
{
	int32_t value;
	if (bifdev_get_cpchip_reg(addr, &value) < 0) {
		printk(KERN_ERR "bifdev_get_cpchip_reg err %x\n", addr);
		return 0;
	} else
		return value;
}

void iar_write_reg_1(uint32_t addr, int32_t value)
{
	if (bifdev_set_cpchip_reg(addr, value))
		printk(KERN_ERR "bifdev_set_cpchip_reg err %x\n", addr);
}

#define IAR_REG_READ_1(addr) iar_read_reg_1(addr)
#define IAR_REG_WRITE_1(value, addr) iar_write_reg_1(addr, value)
#endif

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
	frame_buf_t framebuf_user;
	struct mutex iar_mutex;
#ifndef CONFIG_X2_FPGA
	void *tmpbufaddr;
#endif
};
struct iar_cdev_s *g_iar_cdev;

static void iar_edma_callback(void *data)
{
	complete(&g_iar_cdev->completion);
}

int32_t iar_write_framebuf_poll(uint32_t channel, void __user * srcaddr,
				uint32_t size)
{
	int curindex = 0;
	frame_buf_t *bufaddr;
#ifdef CONFIG_X2_FPGA
	bufaddr = iar_get_framebuf_addr(channel);
	copy_from_user(bufaddr->vaddr, srcaddr, size);
#else
	extern int32_t bifdev_set_cpchip_ddr(uint32_t addr, uint16_t size,
					     uint8_t * value);
	bufaddr = iar_get_framebuf_addr(channel);
	copy_from_user(g_iar_cdev->tmpbufaddr, srcaddr, size);
	//bifdev_set_cpchip_ddr(bufaddr->paddr, size, g_iar_cdev->tmpbufaddr);
	uint32_t src = g_iar_cdev->tmpbufaddr;
	uint32_t dst = bufaddr->paddr;
	int cursize = size;
	while (cursize > 0) {
		if (cursize > 4096) {
			bifdev_set_cpchip_ddr(dst, 4096, src);
		} else {
			bifdev_set_cpchip_ddr(dst, cursize, src);
			break;
		}
		src += 4096;
		dst += 4096;
		cursize -= 4096;

	}
#endif

	return iar_switch_buf(channel);

}

int32_t iar_write_framebuf_dma(uint32_t channel, phys_addr_t srcaddr,
			       uint32_t size)
{
	struct dma_chan *ch;
	int ret = 0;
	int curindex = 0;
	struct dma_async_tx_descriptor *tx;
	dma_addr_t *src, *dst;
	void *src_virt, *dst_virt;
	dma_cap_mask_t mask;
	dma_cookie_t cookie;
	frame_buf_t *bufaddr;

	reinit_completion(&g_iar_cdev->completion);
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	ch = dma_request_channel(mask, NULL, NULL);
	if (IS_ERR(ch)) {
		ret = PTR_ERR(ch);
		printk(KERN_ERR "%s: dma_request_channel failed: %d\n",
		       __func__, ret);
		return -1;
	}
	bufaddr = iar_get_framebuf_addr(channel);
	tx = ch->device->device_prep_dma_memcpy(ch, srcaddr, bufaddr->paddr,
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

	return 0;
}

int32_t iar_display_update(update_cmd_t * update_cmd)
{
	int index = 0;
	frame_buf_t *bufaddr;
	int ret = 0;
	for (index = 0; index < IAR_CHANNEL_MAX; index++) {
		if (index == 1 || index == 3)
			continue;	//TODO, now channnel 2 and 4 is disable

		if (update_cmd->enable_flag[index]) {
#ifdef IAR_DMA_MODE
			if (update_cmd->srcframe[index].paddr)
				ret |=
				    iar_write_framebuf_dma(index,
							   update_cmd->srcframe
							   [index].paddr,
							   update_cmd->frame_size
							   [index]);
#else
			if (update_cmd->srcframe[index].vaddr)
				ret |=
				    iar_write_framebuf_poll(index,
							    update_cmd->srcframe
							    [index].vaddr,
							    update_cmd->frame_size
							    [index]);
#endif
		}
	}
	return ret;
}

static int iar_cdev_open(struct inode *inode, struct file *filp)
{
	dev_t device = inode->i_rdev;
	struct iar_cdev_s *iarcdev_p;

	iarcdev_p = container_of(inode->i_cdev, struct iar_cdev_s, cdev);
	filp->private_data = &iarcdev_p;

	return iar_open();
}

static long iar_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
{
	int ret;
	unsigned int result, region;
	void __user *arg = (void __user *)p;
	mutex_lock(&g_iar_cdev->iar_mutex);
	switch (cmd) {
	case IAR_START:
		{
			ret = iar_start(1);
		}
		break;
	case IAR_STOP:
		{
			ret = iar_stop();
		}
		break;
	case IAR_DISPLAY_UPDATE:
		{
			update_cmd_t update_cmd;
			copy_from_user(&update_cmd, arg, sizeof(update_cmd_t));
			ret = iar_display_update(&update_cmd);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_CHANNEL_CFG:
		{
			channel_base_cfg_t channel_cfg;
			copy_from_user(&channel_cfg, arg,
				       sizeof(channel_base_cfg_t));
			ret = iar_channel_base_cfg(&channel_cfg);
		}
		break;
	case IAR_GAMMA_CFG:
		{
			gamma_cfg_t gamma_cfg;
			copy_from_user(&gamma_cfg, arg, sizeof(gamma_cfg_t));
			ret = iar_gamma_cfg(&gamma_cfg);
		}
		break;
	case IAR_SCALE_CFG:
		{
			upscaling_cfg_t upscaling_cfg;
			copy_from_user(&upscaling_cfg, arg,
				       sizeof(upscaling_cfg_t));
			ret = iar_upscaling_cfg(&upscaling_cfg);
		}
		break;
	case IAR_OUTPUT_CFG:
		{
			output_cfg_t output_cfg;
			copy_from_user(&output_cfg, arg, sizeof(output_cfg_t));
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
	int i = 0;
	phys_addr_t regaddr = 0xA4001000;
	for (; regaddr <= 0xA4001404; regaddr += 0x4) {
		int regval = IAR_REG_READ_1(regaddr);
		printk("iar reg:[0x%x]: 0x%x \n", regaddr, regval);
	}

	return (s - buf);
}

static ssize_t x2_iar_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t n)
{
	int level;
	char *p, *tmpv;
	int len;
	uint regvalue;
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
	int ret;

	g_iar_cdev = kmalloc(sizeof(struct iar_cdev_s), GFP_KERNEL);
	if (!g_iar_cdev) {
		printk(KERN_ERR "Unable to alloc IAR DEV\n");
		return -ENOMEM;
	}
	g_iar_cdev->name = "iar_cdev";
	mutex_init(&g_iar_cdev->iar_mutex);
	init_completion(&g_iar_cdev->completion);
#ifndef CONFIG_X2_FPGA
	g_iar_cdev->tmpbufaddr = vmalloc(1920 * 1080 * 4);
#endif
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

	ret =
	    device_create(g_iar_cdev->iar_classes, NULL, g_iar_cdev->dev_num,
			  NULL, g_iar_cdev->name);

	x2_iar_kobj = kobject_create_and_add("x2_iar", NULL);
	if (!x2_iar_kobj)
		return -ENOMEM;
	return sysfs_create_group(x2_iar_kobj, &attr_group);

	if (ret)
		return ret;

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
