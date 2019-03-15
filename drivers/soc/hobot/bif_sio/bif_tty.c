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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include "../bif_base/bif_base.h"
#include "../bif_base/bif_api.h"
#include "bif_tty.h"
#include "ringbuf.h"

#define _DEBUG_PRINTF_
#include "debug.h"

#define BIF_SIO_VER	"1.01"
#define NODE_NUM	4
#define TTY_BUF_SIZE	(512)

/* cdev node private info */
struct bif_tty_node {

	unsigned int index;
#ifdef CONFIG_HOBOT_BIF_AP
	void *addr_phy_r;
	void *addr_phy_w;
#endif
	struct ringbuf_t *rb_self;
	struct ringbuf_t *rb_other;

	spinlock_t r_lock;
	spinlock_t w_lock;
};

/* cdev info */
struct bif_tty_cdev {

	struct cdev cdev;
	const char *name;
	int major;
	int minor;
	int num_nodes;
	dev_t dev_num;
	struct class *drv_class;
	wait_queue_head_t wq;

	void *buff_addr_vir;
	void *buff_addr_phy;

	/*
	 * ap side, each node read/write form cp store into buff_rw first
	 */
#ifdef CONFIG_HOBOT_BIF_AP
	char *buff_rw;
#endif
	/*
	 * alloc space from bif base
	 */
	struct ringbuf_t *rb_base_self;
	/*
	 * ponit to sapce of other base
	 */
	struct ringbuf_t *rb_base_other;
	/*
	 * as private struct of each dev node
	 */
	struct bif_tty_node *tb_node;
};

static struct bif_tty_cdev *g_tb_dev;

static void ttyinfo_dbg(struct bif_tty_node *p)
{
	tty_debug_log("index:%d\n"
		      "self :head=%4d tail=%4d rbuf=0x%p wbuf=0x%p\n"
		      "other:head=%4d tail=%4d rbuf=0x%p wbuf=0x%p\n",
		      p->index,
		      p->rb_self->head, p->rb_self->tail,
		      p->rb_self->rbuf, p->rb_self->wbuf,
		      p->rb_other->head, p->rb_other->tail,
		      p->rb_other->rbuf, p->rb_other->wbuf);
}

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_tty_reset_ap_addr(struct bif_tty_cdev *cdev)
{
	int i;
	void *addr = NULL;
	struct bif_tty_node *tmp;

	addr = bif_query_address(BUFF_SMD);
	if (addr == (void *)-1 || addr == NULL) {
		tty_err_log("irq reset ap addr failed\n");
		return 0;
	}
	cdev->buff_addr_phy = addr;
	for (i = 0; i < NODE_NUM; i++) {
		tmp = &cdev->tb_node[i];

		tmp->addr_phy_r = cdev->buff_addr_phy + i * 2 * TTY_BUF_SIZE;
		tmp->addr_phy_w = tmp->addr_phy_r + TTY_BUF_SIZE;

		tty_debug_log("index:%d phy_r=0x%p phy_w=0x%p\n",
			      i, tmp->addr_phy_r, tmp->addr_phy_w);
	}
	return 0;
}
#endif

static int bif_tty_reset_otherbase(struct bif_tty_cdev *cdev)
{
	int i;
	void *addr = NULL;
	struct bif_tty_node *tmp;

	addr = bif_query_otherbase(BUFF_SMD);
	if (addr == (void *)-1 || addr == NULL) {
		tty_err_log("irq wait other address fail\n");
		return 0;
	}

	cdev->rb_base_other = (struct ringbuf_t *)addr;
	for (i = 0; i < NODE_NUM; i++) {
		tmp = &cdev->tb_node[i];
		tmp->rb_other = &cdev->rb_base_other[i];
		tty_debug_log("index:%d rb_other=0x%p\n", i, tmp->rb_other);
	}
	return 0;
}

static irqreturn_t bif_tty_irq_handler(int irq, void *data)
{

#ifdef CONFIG_HOBOT_BIF_AP
	if (g_tb_dev->buff_addr_phy == NULL)
		bif_tty_reset_ap_addr(g_tb_dev);
#endif

	if (g_tb_dev->rb_base_other == NULL)
		bif_tty_reset_otherbase(g_tb_dev);

	wake_up(&g_tb_dev->wq);
	return IRQ_HANDLED;
}

static void bif_tty_irq(void)
{
	tty_debug_log("enter\n");
	bif_send_irq(BUFF_SMD);
}

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_sync_cpbuf(unsigned long phy_addr,
			  unsigned int blklen, unsigned char *buffer)
{
	int ret;

	tty_debug_log("enter\n");
#ifdef CONFIG_HOBOT_BIFSD
	ret = bif_sd_read((void *)phy_addr, blklen, buffer);
	if (!ret) {
		tty_err_log("ret=%d\n", ret);
		return -1;
	}
#elif defined CONFIG_HOBOT_BIFSPI
	ret = bif_spi_read((void *)phy_addr, blklen, buffer);
	if (!ret) {
		tty_err_log("ret=%d\n", ret);
		return -1;
	}
#endif
	tty_dump_log(buffer, blklen);
	tty_debug_log("read cp succ leave\n");
	return 0;
}

static int bif_sync_apbuf(unsigned long phy_addr,
			  unsigned int blklen, unsigned char *buffer)
{
	int ret;

	tty_debug_log("enter\n");
	tty_dump_log(buffer, blklen);
#ifdef CONFIG_HOBOT_BIFSD
	ret = bif_sd_write((void *)phy_addr, blklen, buffer);
	if (!ret) {
		tty_err_log("ret=%d\n", ret);
		return -1;
	}
#elif defined CONFIG_HOBOT_BIFSPI
	ret = bif_spi_write((void *)phy_addr, blklen, buffer);
	if (!ret) {
		tty_err_log("ret=%d\n", ret);
		return -1;
	}
#endif
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


static int bif_tty_open(struct inode *inode, struct file *filp)
{
	unsigned int minor;
	struct bif_tty_node *node_tmp;

	tty_debug_log("enter\n");
	minor = iminor(inode);
	node_tmp = &g_tb_dev->tb_node[minor];
	node_tmp->index = minor;
	filp->private_data = NULL;

#ifdef CONFIG_HOBOT_BIF_AP
	if (node_tmp->addr_phy_r == NULL) {
		tty_err_log("ap bif base space is null\n");
		return -1;
	}
#endif
	if (node_tmp->rb_other == NULL) {
		tty_err_log("other side info = null\n");
		return -2;
	}
	filp->private_data = node_tmp;
	ttyinfo_dbg(node_tmp);
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
	ttyinfo_dbg(bt_node);

	if (bt_node == NULL) {
		tty_err_log("node not init\n");
		return -ENOENT;
	}

	if (bif_ringbuf_other_used(bt_node) == 0) {
		tty_err_log("rc=%d\n", rc);
		return 0;
	}
#ifdef CONFIG_HOBOT_BIF_AP
	memset(pself->rbuf, 0, pself->size);
	tty_dump_log(pself->rbuf, pself->size);
	bif_sync_cpbuf((unsigned long)(bt_node->addr_phy_r),
		       pself->size, pself->rbuf);
#endif
	spin_lock(&bt_node->r_lock);
	count = bif_ringbuf_other_used(bt_node);
	if (size > count)
		size = count;
	rc = ringbuf_read(pself, buf, size);
	if (rc < 0)
		goto fail;

	bif_tty_irq();
	ttyinfo_dbg(bt_node);
fail:
	spin_unlock(&bt_node->r_lock);
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

	tty_debug_log("enter\n");
	ttyinfo_dbg(bt_node);

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
	spin_lock(&bt_node->w_lock);
	while (size > 0) {
		count = bif_ringbuf_self_free(bt_node);
		if (count == 0) {
			tty_err_log("rest=%d < size=%d\n", count, (int)size);
			wait_event(g_tb_dev->wq,
				   bif_ringbuf_self_free(bt_node) > 0);
			count = bif_ringbuf_self_free(bt_node);
		}
		if (count >= size)
			count = size;
		rc = ringbuf_write(pself, buf + offset, count);
		if (rc < 0)
			goto fail;

#ifdef CONFIG_HOBOT_BIF_AP
		bif_sync_apbuf((unsigned long)(bt_node->addr_phy_w),
			       pself->size, pself->wbuf);
#endif
		bif_tty_irq();
		size -= count;
		offset += count;
		ttyinfo_dbg(bt_node);
	}

fail:
	spin_unlock(&bt_node->w_lock);
	tty_debug_log("rc=%d leave\n", rc);
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

#ifdef CONFIG_HOBOT_BIF_AP
	kfree(cdev->buff_rw);
#endif
	kfree(cdev->tb_node);
	kfree(cdev);
	return 0;
}

static int bif_tty_alloc_base(struct bif_tty_cdev *cdev)
{
	int rc = 0;
	void *addr = NULL;

	tty_debug_log("enter\n");
	addr = bif_alloc_base(BUFF_SMD,
			      cdev->num_nodes * sizeof(struct ringbuf_t));
	if (addr == (void *)-1 || addr == NULL) {
		rc = -ENOSPC;
		tty_err_log("ret=%d\n", rc);
	} else {
		rc = 0;
		cdev->rb_base_self = (struct ringbuf_t *)addr;
		bif_tty_irq();
	}

	return rc;
}

static int bif_tty_query_otherbase(struct bif_tty_cdev *cdev)
{
	void *addr = NULL;

	tty_debug_log("enter\n");
	addr = bif_query_otherbase(BUFF_SMD);
	if (addr == (void *)-1
		|| addr == NULL
		|| addr == (void *)-ERESTARTSYS) {

		cdev->rb_base_other = NULL;
		tty_err_log("query other address failed\n");
	} else
		cdev->rb_base_other = (struct ringbuf_t *)addr;

	return 0;
}

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_tty_query_address(struct bif_tty_cdev *cdev)
{
	void *addr = NULL;

	addr = bif_query_address(BUFF_SMD);
	if (addr == (void *)-1
		|| addr == NULL
		|| addr == (void *)-ERESTARTSYS) {

		addr = NULL;
		tty_err_log("query ap address failed\n");
	}
	cdev->buff_addr_phy = addr;

	return 0;
}
#else
static int bif_tty_reg_address(struct bif_tty_cdev *cdev)
{
	bif_register_address(BUFF_SMD, (void *)cdev->buff_addr_phy);
	bif_tty_irq();
	return 0;
}
#endif

static int bif_tty_init_node(struct bif_tty_cdev *cdev)
{
	int i;
	struct bif_tty_node *tmp;

	tty_debug_log("enter\n");
	for (i = 0; i < NODE_NUM; i++) {
		tmp = &cdev->tb_node[i];

		tmp->rb_self = &cdev->rb_base_self[i];
		if (cdev->rb_base_other)
			tmp->rb_other = &cdev->rb_base_other[i];
		else
			tmp->rb_other = NULL;
#ifdef CONFIG_HOBOT_BIF_AP
		if (cdev->buff_addr_phy) {
			tmp->addr_phy_r =
			    cdev->buff_addr_phy + i * 2 * TTY_BUF_SIZE;
			tmp->addr_phy_w = tmp->addr_phy_r + TTY_BUF_SIZE;
		} else {
			tmp->addr_phy_r = NULL;
			tmp->addr_phy_w = NULL;
		}
		tty_debug_log("index:%d phy_r=0x%p phy_w=0x%p\n",
			      i, tmp->addr_phy_r, tmp->addr_phy_w);
#endif
		spin_lock_init(&tmp->r_lock);
		spin_lock_init(&tmp->w_lock);

	}
	return 0;
}

static int bif_tty_init_ringbuff(struct bif_tty_cdev *cdev)
{
	int i;
	struct bif_tty_node *tmp;

	tty_debug_log("enter\n");
	for (i = 0; i < NODE_NUM; i++) {
		tmp = &cdev->tb_node[i];

		tmp->rb_self->head = 0;
		tmp->rb_self->tail = 0;
		tmp->rb_self->size = TTY_BUF_SIZE;
#ifdef CONFIG_HOBOT_BIF_AP
		/*
		 * ap side write data to alloc buff first
		 */
		tmp->rb_self->rbuf = cdev->buff_rw + i * 2 * TTY_BUF_SIZE;
		tmp->rb_self->wbuf = tmp->rb_self->rbuf + TTY_BUF_SIZE;
#else
		/*
		 * cp side write data to alloc buff directly
		 */
		tmp->rb_self->wbuf = cdev->buff_addr_vir + i * 2 * TTY_BUF_SIZE;
		tmp->rb_self->rbuf = tmp->rb_self->wbuf + TTY_BUF_SIZE;
#endif
		tty_debug_log("index:%d rbuf=0x%p wbuf=0x%p\n",
			      i, tmp->rb_self->rbuf, tmp->rb_self->wbuf);
	}
	return 0;
}

static struct bif_tty_cdev *bif_tty_init_dev(void)
{
	struct bif_tty_cdev *tb_cdev = NULL;

	tb_cdev = kzalloc(sizeof(struct bif_tty_cdev), GFP_KERNEL);
	if (!tb_cdev) {
		tty_err_log("cdev alloc mem fail\n");
		return NULL;
	}

	tb_cdev->name = "ttyBIF";
	tb_cdev->num_nodes = NODE_NUM;
	init_waitqueue_head(&tb_cdev->wq);
	g_tb_dev = tb_cdev;
	return tb_cdev;
}

static int bif_tty_alloc(struct bif_tty_cdev *cdev)
{
#ifndef CONFIG_HOBOT_BIF_AP
	void *vir_addr;
	unsigned long phy_addr;
#endif
	tty_debug_log("enter\n");

	if (bif_tty_alloc_base(cdev) != 0) {
		tty_debug_log("err\n");
		return -ENOMEM;
	}

#ifdef CONFIG_HOBOT_BIF_AP
	cdev->buff_rw = kzalloc(2 * NODE_NUM * TTY_BUF_SIZE, GFP_KERNEL);
	if (!cdev->buff_rw)
		goto err_alloc_ap_buff;
#else
	vir_addr = bif_alloc_cp(BUFF_SMD, 2 * NODE_NUM * TTY_BUF_SIZE,
				&phy_addr);
	if (vir_addr == (void *)-1
		|| vir_addr == NULL
		|| phy_addr == 0) {
		tty_err_log("err vir_addr = 0x%p vir_addr = 0x%lx\n",
			vir_addr, phy_addr);
		goto err_alloc_cp_buff;
	}
	cdev->buff_addr_vir = vir_addr;
	cdev->buff_addr_phy = (void *)phy_addr;
	tty_debug_log("buff_addr_phy = 0x%p\n", cdev->buff_addr_phy);
#endif
	cdev->tb_node = kzalloc(NODE_NUM * sizeof(struct bif_tty_node),
				GFP_KERNEL);
	if (!cdev->tb_node)
		goto err_alloc_node;

	return 0;

err_alloc_node:
#ifdef CONFIG_HOBOT_BIF_AP
	kfree(cdev->buff_rw);
err_alloc_ap_buff:
#else
err_alloc_cp_buff:
#endif
	kfree(cdev);
	return -ENOMEM;
}

static int bif_tty_init_chrdev(struct bif_tty_cdev *cdev)
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

	for (i = 0; i < NODE_NUM; i++)
		device_destroy(cdev->drv_class, MKDEV(cdev->major, i));
	cdev_del(&cdev->cdev);
	unregister_chrdev_region(MKDEV(cdev->major, 0), cdev->dev_num);
	class_destroy(cdev->drv_class);

	return 0;
}

static int bif_tty_init_base(struct bif_tty_cdev *cdev)
{

	bif_tty_query_otherbase(cdev);
#ifdef CONFIG_HOBOT_BIF_AP
	bif_tty_query_address(cdev);
#else
	bif_tty_reg_address(cdev);
#endif
	bif_tty_init_node(cdev);
	bif_tty_init_ringbuff(cdev);
	tty_debug_log("leave\n");
	return 0;
}

static int __init bif_tty_init(void)
{
	int rc;
	struct bif_tty_cdev *tty_dev = NULL;

	tty_err_log("VER:%s\n", BIF_SIO_VER);

	bif_register_irq(BUFF_SMD, bif_tty_irq_handler);

	tty_dev = bif_tty_init_dev();
	if (tty_dev == NULL)
		return -ENOMEM;

	rc = bif_tty_alloc(tty_dev);
	if (rc)
		goto fail_alloc;

	rc = bif_tty_init_chrdev(tty_dev);
	if (rc)
		goto fail;

	bif_tty_init_base(tty_dev);

	tty_debug_log("leave\n");
	return 0;

fail:
	tty_debug_log("rc=%d leave\n", rc);
	bif_tty_free(tty_dev);
fail_alloc:
	return rc;
}

late_initcall(bif_tty_init);

static void __exit bif_tty_exit(void)
{
	bif_tty_free_chrdev(g_tb_dev);
	bif_tty_free(g_tb_dev);
}

module_exit(bif_tty_exit);

MODULE_DESCRIPTION("Driver for Bif tty");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL v2");
