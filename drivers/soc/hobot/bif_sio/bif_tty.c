/*****************************************************************************
 ****			 COPYRIGHT NOTICE			****
 ****		 Copyright	2017 Horizon Robotics, Inc.	****
 ****			 All rights reserved.			****
 *****************************************************************************/
/**
 * BIF tty driver for read write
 * @author	guozheng.li(guozheng.li@horizon.ai)
 * @date	2018/12/20
 */
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/crc16.h>
#include "../bif_base/bif_base.h"
#include "../bif_base/bif_api.h"
#include "bif_tty.h"
#include "ringbuf.h"

#define _DEBUG_PRINTF_
#include "debug.h"

#define BIF_SIO_VER	"HOBOT-bifsio_V20.190712"

/* define how many bif serial port */
//#define BIF_SIO_NR_PORTS      CONFIG_BIF_SIO_NR
#define BIF_SIO_NR_PORTS	4

#define TTY_BUF_SIZE	(512)

/* cdev node private info */
struct bif_tty_node {

	unsigned int index;
	void *rbuf_phy_addr;
	void *wbuf_phy_addr;
	char *rbuf;
	char *wbuf;
	struct ringbuf_t *rb_self;
	struct ringbuf_t *rb_other;
	struct mutex r_lock;
	struct mutex w_lock;
	struct mutex rb_lock;
};

/* cdev info */
struct bif_tty_cdev {

	struct cdev cdev;
	const char *name;
	int major;
	int minor;
	int num_nodes;
	dev_t dev_num;
	int channel;
	struct class *drv_class;
	wait_queue_head_t wq;

	struct bif_tty_node *tb_node[BIF_SIO_NR_PORTS];

	/* only as record for free */
#ifdef CONFIG_HOBOT_BIF_AP
	char *rw_addr;
#else
	unsigned long phy_addr_rw;
	void *vir_addr_rw;
#endif

};

static struct bif_tty_cdev *g_tb_dev;

static struct bif_tty_cdev *bif_tty_get(void)
{
	return g_tb_dev;
}

static void tty_buf_info(struct bif_tty_node *p)
{
	tty_debug_log("index:%d\n"
		      "self :head=%4d tail=%4d\n"
		      "other:head=%4d tail=%4d\n",
		      p->index,
		      p->rb_self->head, p->rb_self->tail,
		      p->rb_other->head, p->rb_other->tail);
}

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_tty_reset_rwbuf(struct bif_tty_cdev *cdev)
{
	int i;
	void *addr = NULL;
	struct bif_tty_node *tmp;

	addr = bif_query_address(BUFF_SMD);
	if (addr == (void *)-1 || addr == NULL) {
		tty_err_log("irq reset ap addr failed\n");
		return 0;
	}
	for (i = 0; i < cdev->num_nodes; i++) {
		tmp = cdev->tb_node[i];
		tmp->wbuf_phy_addr = addr + i * 2 * TTY_BUF_SIZE;
		tmp->rbuf_phy_addr = tmp->wbuf_phy_addr + TTY_BUF_SIZE;
		tty_debug_log("index:%d phy_r=0x%p phy_w=0x%p\n",
			      i, tmp->rbuf_phy_addr, tmp->wbuf_phy_addr);
	}
	return 0;
}
#endif

static int bif_tty_reset_otherbase(struct bif_tty_cdev *cdev)
{
	int i;
	struct ringbuf_t *tmp;

	tty_debug_log("enter\n");

	tmp = bif_query_otherbase(BUFF_SMD);
	if (tmp == (void *)-1 || tmp == NULL) {
		tty_err_log("irq wait other address fail\n");
		return 0;
	}

	for (i = 0; i < cdev->num_nodes; i++) {
		cdev->tb_node[i]->rb_other = &tmp[i];
		tty_debug_log("index:%d rb_other=0x%p\n", i, &tmp[i]);
	}
	return 0;
}

static int bif_tty_check_other(struct ringbuf_t *rb)
{
	int rc = 0;

	if (rb->head >= rb->size || rb->tail >= rb->size) {
#ifdef CONFIG_HOBOT_BIF_AP
		rc = bif_sync_base();
#else
		rc = bif_sync_ap();
#endif
	}
	return rc;
}

static irqreturn_t bif_tty_irq_handler(int irq, void *data)
{
	struct bif_tty_cdev *cdev = bif_tty_get();

	tty_debug_log("enter\n");
#ifdef CONFIG_HOBOT_BIF_AP
	if (cdev->tb_node[0]->rbuf_phy_addr == NULL)
		bif_tty_reset_rwbuf(cdev);
#endif
	if (cdev->tb_node[0]->rb_other == NULL)
		bif_tty_reset_otherbase(cdev);

	wake_up(&cdev->wq);
	return IRQ_HANDLED;
}

static void bif_tty_irq(void)
{
	tty_debug_log("enter\n");
	bif_send_irq(BUFF_SMD);
}

#ifdef CONFIG_HOBOT_BIF_AP
/**
 * bif_tty_get_phychannel
 * @brief: get phy channel, spi or sd
 */
static int bif_tty_get_phychannel(struct bif_tty_cdev *cdev)
{
	char *channel = bif_get_str_bus(BUFF_SMD);

	tty_debug_log("enter\n");
	if (channel == NULL) {
		tty_err_log("bif_get_str_bus err");
		return -1;
	}
	cdev->channel = bifget_bifbustype(channel);
	tty_debug_log("channel = %d\n", cdev->channel);
	return 0;
}

static int bif_sync_cpbuf(unsigned long phy_addr,
			  unsigned int blklen, unsigned char *buffer)
{
	int ret;
	struct bif_tty_cdev *cdev = bif_tty_get();

	tty_debug_log("enter\n");
	ret = bifread(cdev->channel, (void *)phy_addr, blklen, buffer);
	if (!ret) {
		tty_err_log("ret=%d\n", ret);
		return -1;
	}
	tty_dump_log(buffer, blklen);
	tty_debug_log("read cp succ leave\n");
	return 0;
}

static int bif_sync_apbuf(unsigned long phy_addr,
			  unsigned int blklen, unsigned char *buffer)
{
	int ret;
	struct bif_tty_cdev *cdev = bif_tty_get();

	tty_debug_log("enter\n");
	tty_dump_log(buffer, blklen);
	ret = bifwrite(cdev->channel, (void *)phy_addr, blklen, buffer);
	if (!ret) {
		tty_err_log("ret=%d\n", ret);
		return -1;
	}
	tty_debug_log("ap write succ leave\n");
	return 0;
}
#endif

static unsigned int bif_ringbuf_self_free(struct bif_tty_node *bt_node)
{
	struct ringbuf_t *self = bt_node->rb_self;
	struct ringbuf_t *other = bt_node->rb_other;

	if (self->head >= other->tail)
		return ringbuf_capacity(self) - (self->head - other->tail);
	else
		return other->tail - self->head - 1;
}

static unsigned int bif_ringbuf_other_used(struct bif_tty_node *bt_node)
{
	struct ringbuf_t *self = bt_node->rb_self;
	struct ringbuf_t *other = bt_node->rb_other;

	if (other->head >= self->tail)
		return (other->head - self->tail);
	else
		return ringbuf_capacity(self) - self->tail + other->head + 1;
}

static unsigned int bif_ringbuf_cal_crc(struct bif_tty_node *bt_node)
{
	struct ringbuf_t *self = bt_node->rb_self;

	mutex_lock(&bt_node->rb_lock);
	self->crc = crc16(0xFFFF, (unsigned char *)&self->head, 12);
	mutex_unlock(&bt_node->rb_lock);
	return 0;
}

static unsigned int bif_ringbuf_cmp_crc(struct ringbuf_t *rb)
{
	unsigned int crc;

	crc = crc16(0xFFFF, (unsigned char *)&rb->head, 12);
	if (crc == rb->crc)
		return 0;
	else
		return 1;
}

static int bif_tty_open(struct inode *inode, struct file *filp)
{
	unsigned int minor;
	struct bif_tty_node *node_tmp;
	struct bif_tty_cdev *cdev = bif_tty_get();

	tty_debug_log("enter\n");
	minor = iminor(inode);
	node_tmp = cdev->tb_node[minor];
	node_tmp->index = minor;
	filp->private_data = NULL;

#ifdef CONFIG_HOBOT_BIF_AP
	if (node_tmp->rbuf_phy_addr == NULL) {
		tty_err_log("ap bif base space is null\n");
		return -1;
	}
#endif
	if (node_tmp->rb_other == NULL) {
		tty_err_log("other side info = null\n");
		return -2;
	}
	bif_tty_check_other(node_tmp->rb_other);
	filp->private_data = node_tmp;
	tty_buf_info(node_tmp);
	return 0;
}

static int bif_tty_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static long bif_tty_ioctl(struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
	tty_debug_log("cmd=%d\n", cmd);
	return 0;
}

static ssize_t bif_tty_read(struct file *filp, char __user *buf,
			    size_t size, loff_t *ppos)
{
	int rc = 0;
	unsigned int count = 0;
	struct bif_tty_node *bt_node = filp->private_data;
	struct ringbuf_t *pself = bt_node->rb_self;

	tty_debug_log("enter\n");
	tty_buf_info(bt_node);

	if (bt_node == NULL) {
		tty_err_log("node not init\n");
		return -ENOENT;
	}

	if (bif_ringbuf_other_used(bt_node) == 0) {
		tty_debug_log("rc=%d\n", rc);
		return 0;
	}
#ifdef CONFIG_HOBOT_BIF_AP
	memset(bt_node->rbuf, 0, pself->size);
	tty_dump_log(bt_node->rbuf, pself->size);
	bif_sync_cpbuf((unsigned long)(bt_node->rbuf_phy_addr),
		       pself->size, bt_node->rbuf);
#endif
	mutex_lock(&bt_node->r_lock);
	count = bif_ringbuf_other_used(bt_node);
	if (size > count)
		size = count;
	rc = ringbuf_read(pself, buf, bt_node->rbuf, size);
	if (rc < 0)
		goto fail;
	bif_ringbuf_cal_crc(bt_node);
	bif_tty_irq();
	tty_buf_info(bt_node);
fail:
	mutex_unlock(&bt_node->r_lock);
	tty_debug_log("rc=%d leave\n", rc);
	return rc;
}

static ssize_t bif_tty_write(struct file *filp, const char __user *buf,
			     size_t size, loff_t *ppos)
{
	int rc = 0;
	size_t offset = 0;
	unsigned int count = 0;
	struct bif_tty_node *bt_node = filp->private_data;
	struct ringbuf_t *pself = bt_node->rb_self;
	struct bif_tty_cdev *cdev = bif_tty_get();

	tty_debug_log("enter\n");
	tty_buf_info(bt_node);

	if (bt_node == NULL) {
		tty_err_log("node not init\n");
		return -ENOENT;
	}

	if (size == 0)
		return 0;

	/*
	 * if write data longer that buffer size, need transfer more
	 * than one time
	 */
	mutex_lock(&bt_node->w_lock);
	while (size > 0) {
		count = bif_ringbuf_self_free(bt_node);
		if (count == 0) {
			tty_err_log("rest=%d < size=%d\n", count, (int)size);
			rc = wait_event_interruptible(cdev->wq,
						      bif_ringbuf_self_free
						      (bt_node) > 0);
			if (rc < 0) {
				tty_err_log("wait_event_interruptible rc<0\n");
				goto fail;
			}
			count = bif_ringbuf_self_free(bt_node);
		}
		if (count >= size)
			count = size;
		rc = ringbuf_write(pself, bt_node->wbuf, buf + offset, count);
		if (rc < 0)
			goto fail;
		bif_ringbuf_cal_crc(bt_node);
#ifdef CONFIG_HOBOT_BIF_AP
		bif_sync_apbuf((unsigned long)(bt_node->wbuf_phy_addr),
			       pself->size, bt_node->wbuf);
#endif
		bif_tty_irq();
		size -= count;
		offset += count;
		tty_buf_info(bt_node);
	}

fail:
	mutex_unlock(&bt_node->w_lock);
	tty_debug_log("rc=%d leave\n", rc);
	if (rc > 0)
		rc = offset;	//offset is the length of write data
	return rc;
}

static loff_t bif_tty_llseek(struct file *filp, loff_t offset, int orig)
{
	/* not support */
	tty_debug_log("leave\n");
	return 0;
}

static const struct file_operations bif_tty_fops = {
	.owner = THIS_MODULE,
	.llseek = bif_tty_llseek,
	.read = bif_tty_read,
	.write = bif_tty_write,
	.unlocked_ioctl = bif_tty_ioctl,
	.open = bif_tty_open,
	.release = bif_tty_release,
};

static int bif_tty_free(struct bif_tty_cdev *cdev)
{
	int i;
	struct bif_tty_node *node_tmp;

#ifdef CONFIG_HOBOT_BIF_AP
	kfree(cdev->rw_addr);
#else
	bif_dma_free(2 * cdev->num_nodes * TTY_BUF_SIZE,
		     cdev->vir_addr_rw, cdev->phy_addr_rw, 0);
#endif
	for (i = 0; i < cdev->num_nodes; i++) {
		node_tmp = cdev->tb_node[i];
		mutex_destroy(&node_tmp->w_lock);
		mutex_destroy(&node_tmp->r_lock);
		mutex_destroy(&node_tmp->rb_lock);
	}
	kfree(cdev->tb_node[0]);
	kfree(cdev);
	return 0;
}

static struct bif_tty_cdev *bif_tty_create_dev(void)
{
	struct bif_tty_cdev *cdev = NULL;
	struct bif_tty_node *node_tmp;
	int i;

	cdev = kzalloc(sizeof(struct bif_tty_cdev), GFP_KERNEL);
	if (!cdev) {
		tty_err_log("cdev alloc mem fail\n");
		return NULL;
	}

	cdev->name = "ttyBIF";
	cdev->num_nodes = BIF_SIO_NR_PORTS;
	init_waitqueue_head(&cdev->wq);
	g_tb_dev = cdev;

	node_tmp = kcalloc(cdev->num_nodes, sizeof(struct bif_tty_node),
			   GFP_KERNEL);
	if (!node_tmp)
		goto fail;

	for (i = 0; i < cdev->num_nodes; i++) {
		cdev->tb_node[i] = &node_tmp[i];
		mutex_init(&cdev->tb_node[i]->w_lock);
		mutex_init(&cdev->tb_node[i]->r_lock);
		mutex_init(&cdev->tb_node[i]->rb_lock);
	}
	return cdev;

fail:
	kfree(cdev);
	return NULL;
}

/**
 * bif_tty_alloc_base
 * @brief: alloc bifbase share zone, store head and tail info
 */
static int bif_tty_set_base(struct bif_tty_cdev *cdev)
{
	int rc = 0;
	int i;
	unsigned int count;
	struct ringbuf_t *rb_tmp;

	tty_debug_log("enter\n");
	rb_tmp = bif_alloc_base(BUFF_SMD,
				cdev->num_nodes * sizeof(struct ringbuf_t));
	if (rb_tmp == (void *)-1 || rb_tmp == NULL) {
		rc = -ENOSPC;
		tty_err_log("bif_alloc_base failed\n");
		goto fail;
	}
	for (i = 0; i < cdev->num_nodes; i++) {
		cdev->tb_node[i]->rb_self = &rb_tmp[i];

		if (bif_ringbuf_cmp_crc(&rb_tmp[i]) != 0
			|| rb_tmp[i].head >= TTY_BUF_SIZE
			|| rb_tmp[i].tail >= TTY_BUF_SIZE
			|| rb_tmp[i].size != TTY_BUF_SIZE) {
			cdev->tb_node[i]->rb_self->head = 0;
			cdev->tb_node[i]->rb_self->tail = 0;
			cdev->tb_node[i]->rb_self->size = TTY_BUF_SIZE;
		}
	}
	/* rb_base_other can be null */
	rb_tmp = bif_query_otherbase(BUFF_SMD);
	if (rb_tmp == (void *)-1
	    || rb_tmp == NULL || rb_tmp == (void *)-ERESTARTSYS) {
		rc = 0;
		tty_err_log("query other address failed\n");
		goto fail;
	}

	for (i = 0; i < cdev->num_nodes; i++) {
		cdev->tb_node[i]->rb_other = &rb_tmp[i];
		bif_tty_check_other(cdev->tb_node[i]->rb_other);
		/* check invalid count */
		count = bif_ringbuf_other_used(cdev->tb_node[i]);
		if (count > 0) {
			cdev->tb_node[i]->rb_self->tail += count;
			cdev->tb_node[i]->rb_self->tail %=
						cdev->tb_node[i]->rb_self->size;
			tty_err_log("other invalid count %d\n", count);
		}
	}

fail:
	return rc;
}

/**
 * bif_tty_alloc_rwbuf
 * @brief: alloc rwbuf for sio read and write
 */
static int bif_tty_set_rwbuf(struct bif_tty_cdev *cdev)
{
	void *vir_addr;
	void *phy_addr;
	int i;

	tty_debug_log("enter\n");

	/*
	 * at ap side, data save to buff firstly,
	 * then write to cp side phy addr
	 */
#ifdef CONFIG_HOBOT_BIF_AP
	vir_addr = kzalloc(2 * cdev->num_nodes * TTY_BUF_SIZE, GFP_KERNEL);
	if (!vir_addr)
		goto fail;

	phy_addr = bif_query_address(BUFF_SMD);
	if (phy_addr == (void *)-1
	    || phy_addr == NULL || phy_addr == (void *)-ERESTARTSYS) {

		phy_addr = NULL;
		tty_err_log("query ap address failed\n");
	}
	cdev->rw_addr = vir_addr;
	if (phy_addr != NULL) {
		for (i = 0; i < cdev->num_nodes; i++) {
			cdev->tb_node[i]->wbuf_phy_addr
			    = phy_addr + 2 * i * TTY_BUF_SIZE;
			cdev->tb_node[i]->rbuf_phy_addr
			    = cdev->tb_node[i]->wbuf_phy_addr + TTY_BUF_SIZE;
		}
	}
#else
	/*
	 * at cp side, data save to buff directily,
	 * buff phy addr can be read by ap
	 */
	vir_addr = bif_dma_alloc(2 * cdev->num_nodes * TTY_BUF_SIZE,
				 (dma_addr_t *) &cdev->phy_addr_rw, GFP_KERNEL,
				 0);
	if (vir_addr == (void *)-1 || vir_addr == NULL) {
		tty_err_log("vir_addr = 0x%p phy_addr = 0x%p\n",
			    vir_addr, (void *)cdev->phy_addr_rw);
		goto fail;
	}
	phy_addr = (void *)cdev->phy_addr_rw;
	cdev->vir_addr_rw = vir_addr;
	bif_register_address(BUFF_SMD, (void *)cdev->phy_addr_rw);
#endif
	for (i = 0; i < cdev->num_nodes; i++) {
		cdev->tb_node[i]->rbuf = vir_addr + 2 * i * TTY_BUF_SIZE;
		cdev->tb_node[i]->wbuf = cdev->tb_node[i]->rbuf + TTY_BUF_SIZE;
	}
	tty_debug_log("buff_addr_phy = 0x%p\n", phy_addr);
	return 0;
fail:
//      kfree(cdev);//hobot 20190711
	return -ENOMEM;
}

static int bif_tty_create_chrdev(struct bif_tty_cdev *cdev)
{
	int rc = 0;
	dev_t dev;
	int devnum, i, minor, major;
	struct device *device;

	tty_debug_log("enter\n");

	rc = alloc_chrdev_region(&dev, 0, cdev->num_nodes, cdev->name);
	if (rc) {
		tty_err_log("Failed to obtain major/minors");
		return rc;
	}

	cdev->major = major = MAJOR(dev);
	cdev->minor = minor = MINOR(dev);

	cdev_init(&cdev->cdev, &bif_tty_fops);
	cdev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&cdev->cdev, MKDEV(major, minor), cdev->num_nodes);
	if (rc) {
		tty_err_log("Failed to add cdev. Aborting.\n");
		goto unregister_chrdev;
	}

	cdev->drv_class = class_create(THIS_MODULE, cdev->name);
	if (IS_ERR(cdev->drv_class)) {
		rc = IS_ERR(cdev->drv_class);
		tty_err_log("Failed to class_create. Aborting.\n");
		goto dest_class;
	}

	for (i = minor, devnum = 0; devnum < cdev->num_nodes; devnum++, i++) {
		device = device_create(cdev->drv_class,
				       NULL,
				       MKDEV(major, i),
				       NULL, "%s%d", cdev->name, devnum);

		if (IS_ERR(device)) {
			tty_err_log("Failed to create %s%d device. Aborting.\n",
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

static int bif_tty_free_chrdev(struct bif_tty_cdev *cdev)
{
	int i;

	for (i = 0; i < cdev->num_nodes; i++)
		device_destroy(cdev->drv_class, MKDEV(cdev->major, i));
	cdev_del(&cdev->cdev);
	unregister_chrdev_region(MKDEV(cdev->major, 0), cdev->num_nodes);
	class_destroy(cdev->drv_class);
	return 0;
}

static int __init bif_tty_pre_init(void)
{
	int rc;
	struct bif_tty_cdev *tty_dev = NULL;

	pr_info("bifsio: %s\n", BIF_SIO_VER);
	tty_dev = bif_tty_create_dev();
	if (tty_dev == NULL)
		return -ENOMEM;

	rc = bif_tty_set_base(tty_dev);
	if (rc)
		goto fail_1;

	rc = bif_tty_set_rwbuf(tty_dev);
	if (rc)
		goto fail_1;

#ifdef CONFIG_HOBOT_BIF_AP
	rc = bif_tty_get_phychannel(tty_dev);
	if (rc)
		goto fail_1;
#endif
	rc = bif_tty_create_chrdev(tty_dev);
	if (rc)
		goto fail_2;

	bif_register_irq(BUFF_SMD, bif_tty_irq_handler);
	bif_tty_irq();

	tty_debug_log("leave\n");
	return 0;

fail_2:
#ifdef CONFIG_HOBOT_BIF_AP
	kfree(tty_dev->rw_addr);
#else
	bif_dma_free(2 * tty_dev->num_nodes * TTY_BUF_SIZE,
		     tty_dev->vir_addr_rw, tty_dev->phy_addr_rw, 0);
#endif
fail_1:
	kfree(tty_dev->tb_node[0]);
	kfree(tty_dev);
	tty_debug_log("rc=%d leave\n", rc);
//      bif_tty_free(tty_dev);//hobot 20190711
	return rc;
}

static void __exit bif_tty_pre_exit(void)
{
	struct bif_tty_cdev *cdev = bif_tty_get();

	bif_register_irq(BUFF_SMD, NULL);
	bif_tty_free_chrdev(cdev);
	bif_tty_free(cdev);
}

static int bif_tty_probe(struct platform_device *pdev)
{
	return bif_tty_pre_init();
}

static int bif_tty_remove(struct platform_device *pdev)
{
	bif_tty_pre_exit();

	return 0;
}

static const struct of_device_id bifsio_of_match[] = {
	{.compatible = "hobot,bifsio"},
	{},
};

static struct platform_driver bifsio_driver = {
	.driver = {
		   .name = "bifsio",
		   .of_match_table = bifsio_of_match,
		   },
	.probe = bif_tty_probe,
	.remove = bif_tty_remove,
};

static int bif_tty_init(void)
{
	int ret = 0;

#ifdef CONFIG_HOBOT_BIF_AP
	ret = bif_tty_pre_init();
#else
	ret = platform_driver_register(&bifsio_driver);
#endif
	return ret;
}

static void __exit bif_tty_exit(void)
{
#ifdef CONFIG_HOBOT_BIF_AP
	bif_tty_pre_exit();
#else
	platform_driver_unregister(&bifsio_driver);
#endif
}

late_initcall(bif_tty_init);
module_exit(bif_tty_exit);

MODULE_DESCRIPTION("Driver for Bif tty");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL v2");
