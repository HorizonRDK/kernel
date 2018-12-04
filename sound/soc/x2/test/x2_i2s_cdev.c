/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/module.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/semaphore.h>

#include "x2_i2s_reg.h"

typedef struct _x2_i2s_cdev {
    const char *name;
    int major;
    int minor;
    struct cdev cdev;
    dev_t dev_num;
    struct class *class;
} x2_i2s_cdev;

static x2_i2s_cdev *g_x2_i2s_cdev;

#define I2S_CDEV_MAGIC 'T'
#define I2S_MS_MODE_SET 		_IOW(I2S_CDEV_MAGIC, 0x100, int)
#define I2S_DSP_MODE_SET	 	_IOW(I2S_CDEV_MAGIC, 0x101, int)
#define I2S_CH_NUM_SET 			_IOW(I2S_CDEV_MAGIC, 0x102, int)
#define I2S_WORD_LEN_SET 		_IOW(I2S_CDEV_MAGIC, 0x103, int)
#define I2S_FIRST_EDGE_SET 		_IOW(I2S_CDEV_MAGIC, 0x104, int)
#define I2S_LR_WS_SET 			_IOW(I2S_CDEV_MAGIC, 0x105, int)
#define I2S_COPY_ZERO_SEL_SET 	_IOW(I2S_CDEV_MAGIC, 0x106, int)
#define I2S_CLK_EDGE_SET 		_IOW(I2S_CDEV_MAGIC, 0x107, int)
#define I2S_ALLOC_BUF 			_IOW(I2S_CDEV_MAGIC, 0x108, int)
#define I2S_SAMPLE_RATE_SET 	_IOW(I2S_CDEV_MAGIC, 0x109, int)
#define I2S_BUFSIZE_GET			_IOR(I2S_CDEV_MAGIC, 0x110, int)
#define I2S_BUFNUM_GET			_IOR(I2S_CDEV_MAGIC, 0x111, int)
#define I2S_UPDATE_BUF 			_IO(I2S_CDEV_MAGIC, 0x120)
#define I2S_DEALLOC_BUF 		_IO(I2S_CDEV_MAGIC, 0x121)
#define I2S_START 				_IO(I2S_CDEV_MAGIC, 0x122)
#define I2S_PAUSE 				_IO(I2S_CDEV_MAGIC, 0x123)
#define I2S_RESTART 			_IO(I2S_CDEV_MAGIC, 0x124)
#define I2S_STOP 				_IO(I2S_CDEV_MAGIC, 0x125)
#define I2S_READ 				_IO(I2S_CDEV_MAGIC, 0x126)

int x2_i2s_open(struct inode *inode, struct file *filp)
{
    unsigned int minor;
    x2_i2s *i2s;

    minor = iminor(inode);
    i2s = g_x2_i2s[minor];
    i2s->index = minor;
	filp->private_data = i2s;

    I2S_DEBUG("Open i2s%d.\n", minor);
    return 0;
}

static long x2_i2s_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
{
	int val;
    x2_i2s *i2s = filp->private_data;

	switch(cmd)
	{
		case I2S_MS_MODE_SET:
			return x2_i2s_ms_mode_config(i2s, p);
		case I2S_DSP_MODE_SET:
			return x2_i2s_dsp_mode_config(i2s, p);
		case I2S_CH_NUM_SET:
			return x2_i2s_ch_num_config(i2s, p);
		case I2S_WORD_LEN_SET:
			return x2_i2s_word_len_config(i2s, p);
		case I2S_FIRST_EDGE_SET:
			return x2_i2s_first_edge_config(i2s, p);
		case I2S_LR_WS_SET:
			return x2_i2s_lr_ws_config(i2s, p);
		case I2S_COPY_ZERO_SEL_SET:
			return x2_i2s_copy_zero_config(i2s, p);
		case I2S_CLK_EDGE_SET:
			return x2_i2s_clk_edge_config(i2s, p);
		case I2S_ALLOC_BUF:
			return x2_i2s_buf_alloc(i2s, p);
		case I2S_SAMPLE_RATE_SET:
			return x2_i2s_sample_rate_set(i2s, p);
		case I2S_BUFSIZE_GET:
			val = x2_i2s_buf_size_get(i2s);
		    return copy_to_user((void __user *)p, (void *)&val, sizeof(val));
		case I2S_BUFNUM_GET:
			val = x2_i2s_buf_num_get(i2s);
		    return copy_to_user((void __user *)p, (void *)&val, sizeof(val));
		case I2S_UPDATE_BUF:
			return x2_i2s_buf_update(i2s);
		case I2S_DEALLOC_BUF:
            return x2_i2s_buf_dealloc(i2s);
		case I2S_START:
			return x2_i2s_start(i2s);
		case I2S_PAUSE:
			return x2_i2s_pause(i2s);
		case I2S_RESTART:
			return x2_i2s_restart(i2s);
		case I2S_STOP:
			return x2_i2s_stop(i2s);
		case I2S_READ:
            down(&i2s->frame.sem);
            return 0;
        default:
            return -EINVAL;
	}
}

int x2_i2s_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
    return 0;
}

int x2_i2s_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret = -ENXIO;
    x2_i2s *i2s = filp->private_data;
	unsigned long pfn;

	pfn = (unsigned long)i2s->frame.pbuf  >> PAGE_SHIFT;
	SetPageReserved(pfn_to_page(pfn));
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret = remap_pfn_range(vma, vma->vm_start, pfn, vma->vm_end - vma->vm_start, vma->vm_page_prot);
	return ret;
}

static struct fasync_struct *async;

int x2_i2s_async(int fd, struct file *filp, int mode)
{
	return fasync_helper(fd, filp, 1, &async);
}

void fasync_inform_user(void)
{
	kill_fasync(&async, SIGIO, POLL_IN);
}

static const struct file_operations x2_i2s_cdev_ops = {
    .owner = THIS_MODULE,
    .open = x2_i2s_open,
    .release = x2_i2s_release,
    .unlocked_ioctl = x2_i2s_ioctl,
	.fasync = x2_i2s_async,
	.mmap = x2_i2s_mmap,
};

static int __init x2_i2s_cdev_init(void)
{
    int ret, i;
    g_x2_i2s_cdev = kmalloc(sizeof(x2_i2s_cdev), GFP_KERNEL);
    if(!g_x2_i2s_cdev){
        return -ENOMEM;
    }
    g_x2_i2s_cdev->name = "i2s";

    g_x2_i2s_cdev->class = class_create(THIS_MODULE, "i2s-class");
    if(IS_ERR(g_x2_i2s_cdev->class)){
        return PTR_ERR(g_x2_i2s_cdev->class);
    }

    alloc_chrdev_region(&g_x2_i2s_cdev->dev_num, 0, X2_I2S_DEV_NUMBER, g_x2_i2s_cdev->name);

    cdev_init(&g_x2_i2s_cdev->cdev, &x2_i2s_cdev_ops);

    ret = cdev_add(&g_x2_i2s_cdev->cdev, g_x2_i2s_cdev->dev_num, X2_I2S_DEV_NUMBER);
    if(ret){
        unregister_chrdev_region(g_x2_i2s_cdev->dev_num, X2_I2S_DEV_NUMBER);
    }

    for(i=0; i < X2_I2S_DEV_NUMBER; i++){
        device_create(g_x2_i2s_cdev->class, NULL, MKDEV(MAJOR(g_x2_i2s_cdev->dev_num),i), NULL, "i2s-%d", i);
    }
    return 0;
}

static void __exit x2_i2s_cdev_exit(void)
{
    int i;
    for(i=0; i < X2_I2S_DEV_NUMBER; i++){
        device_destroy(g_x2_i2s_cdev->class, MKDEV(MAJOR(g_x2_i2s_cdev->dev_num),i));
    }
    cdev_del(&g_x2_i2s_cdev->cdev);
    unregister_chrdev_region(g_x2_i2s_cdev->dev_num, X2_I2S_DEV_NUMBER);
    class_destroy(g_x2_i2s_cdev->class);
    return;
}

module_init(x2_i2s_cdev_init);
module_exit(x2_i2s_cdev_exit);

MODULE_AUTHOR("Hobot");
MODULE_DESCRIPTION("X2 I2S Char Device");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:x2-i2s-cdev");
