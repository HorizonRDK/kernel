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
#include <linux/uaccess.h>
#include "../bif_base/bif_base.h"
#include "../bif_base/bif_api.h"
#include "bif_tty.h"
#include "ringbuf.h"

#define _DEBUG_PRINTF_
#include "debug.h"

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
	 * ap side, each node read form cp store into buff_r
	 * write to cp write buff_w first
	 */
#ifdef CONFIG_HOBOT_BIF_AP
	char *buff_r;
	char *buff_w;
#endif
	/*
	 * alloc space from bif base
	 */
	struct ringbuf_t (*rb_base_self)[NODE_NUM];
	/*
	 * ponit to sapce of other base
	 */
	struct ringbuf_t (*rb_base_other)[NODE_NUM];

	/*
	 * as private struct of each dev node
	 */
	struct bif_tty_node (*tb_node)[NODE_NUM];
};

static struct bif_tty_cdev *tb_dev;

static void ttyinfo_dbg(struct bif_tty_node *p)
{
	tty_debug_log("index:%d self:head=%d tail=%d other:head=%d tail=%d\n",
		      p->index,
		      p->rb_self->head,
		      p->rb_self->tail, p->rb_other->head, p->rb_other->tail);
}

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_tty_reset_node_addr(struct bif_tty_cdev *cdev)
{
	int i;
	void *addr = NULL;
	struct bif_tty_node *tmp;

	addr = bif_query_address(BUFF_SMD);
	if (addr == (void *)-1 || addr == NULL) {
		pr_err("%s(%d) wait other address fail\n", __func__, __LINE__);
		return 0;
	}
	cdev->buff_addr_phy = addr;
	for (i = 0; i < NODE_NUM; i++) {
		tmp = cdev->tb_node[i];

		tmp->addr_phy_r = cdev->buff_addr_phy + i * 2 * TTY_BUF_SIZE;
		tmp->addr_phy_w = tmp->addr_phy_r + TTY_BUF_SIZE;
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
		pr_err("%s(%d) wait other address fail\n", __func__, __LINE__);
		return 0;
	}

	cdev->rb_base_other = (struct ringbuf_t(*)[])addr;
	for (i = 0; i < NODE_NUM; i++) {
		tmp = cdev->tb_node[i];
		tmp->rb_other = cdev->rb_base_other[i];
	}
	return 0;
}

static irqreturn_t bif_tty_irq_handler(int irq, void *data)
{

#ifdef CONFIG_HOBOT_BIF_AP
	if (tb_dev->buff_addr_phy == NULL)
		bif_tty_reset_node_addr(tb_dev);
#endif

	if (tb_dev->rb_base_other == NULL)
		bif_tty_reset_otherbase(tb_dev);

	wake_up(&tb_dev->wq);
	return IRQ_HANDLED;
}

static int bif_tty_open(struct inode *inode, struct file *filp)
{
	unsigned int minor;
	struct bif_tty_node *node_tmp;

	tty_debug_log("%s(%d)\n", __func__, __LINE__);
	minor = iminor(inode);
	node_tmp = tb_dev->tb_node[minor];
	node_tmp->index = minor;
	filp->private_data = node_tmp;
	ttyinfo_dbg(node_tmp);

#ifdef CONFIG_HOBOT_BIF_AP
	if (node_tmp->addr_phy_r == NULL) {
		pr_err("%s(%d)ap node_tmp addr_phy_r=null\n", __func__,
		       __LINE__);
		return -1;
	}
#endif
	if (node_tmp->rb_other == NULL) {
		pr_err("%s(%d)other info = null\n", __func__, __LINE__);
		return -2;
	}
	return 0;
}

static int bif_tty_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static long bif_tty_ioctl(struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
	tty_debug_log("%s(%d) cmd=%d\n", __func__, __LINE__, cmd);
	return 0;
}

static void bif_tty_irq(void)
{
	tty_debug_log("%s(%d)\n", __func__, __LINE__);
	bif_send_irq(BUFF_SMD);
}

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_sync_cpbuf(unsigned long phy_addr,
			  unsigned int blklen, unsigned char *buffer)
{
	int ret;

	tty_debug_log("%s(%d) enter\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIFSD
	ret = bif_sd_read(phy_addr / 512, blklen / 512, buffer);
	if (!ret) {
		pr_err("%s(%d) ret=%d\n", __func__, __LINE__, ret);
		return -1;
	}
#elif defined CONFIG_HOBOT_BIFSPI
	ret = bif_spi_read(&phy_addr, blklen, buffer);
	if (!ret) {
		pr_err("%s(%d) ret=%d\n", __func__, __LINE__, ret);
		return -1;
	}
#endif
	tty_dump_log(buffer, 128);
	tty_debug_log("%s(%d)read cp Succ Leave...\n", __func__, __LINE__);
	return 0;
}

static int bif_sync_apbuf(unsigned long phy_addr,
			  unsigned int blklen, unsigned char *buffer)
{
	int ret;

	tty_debug_log("%s(%d) enter\n", __func__, __LINE__);
	tty_dump_log(buffer, 128);
#ifdef CONFIG_HOBOT_BIFSD
	ret = bif_sd_write(phy_addr / 512, blklen / 512, buffer);
	if (!ret) {
		pr_err("%s(%d) ret=%d\n", __func__, __LINE__, ret);
		return -1;
	}
#elif defined CONFIG_HOBOT_BIFSPI
	ret = bif_spi_write(&phy_addr, blklen, buffer);
	if (!ret) {
		pr_err("%s(%d) ret=%d\n", __func__, __LINE__, ret);
		return -1;
	}
#endif
	tty_debug_log("%s(%d)ap write Succ Leave...\n", __func__, __LINE__);
	return 0;
}
#endif

static unsigned int bif_ringbuf_free(struct bif_tty_node *bt_node)
{
	struct ringbuf_t *self = bt_node->rb_self;
	struct ringbuf_t *other = bt_node->rb_other;

	if (self->head >= other->tail)
		return ringbuf_capacity(self) - (self->head - other->tail);
	else
		return other->tail - self->head - 1;
}

static unsigned int bif_ringbuf_used(struct bif_tty_node *bt_node)
{
	return ringbuf_capacity(bt_node->rb_self) - bif_ringbuf_free(bt_node);
}

static int bif_ringbuf_is_empty(struct bif_tty_node *bt_node)
{
	return bif_ringbuf_free(bt_node) == ringbuf_capacity(bt_node->rb_self);
}

static ssize_t bif_tty_read(struct file *filp, char __user *buf,
			    size_t size, loff_t *ppos)
{
	unsigned int count = size;
	int rc = 0;
	struct bif_tty_node *bt_node = filp->private_data;
	struct ringbuf_t *pself = bt_node->rb_self;

	tty_debug_log("%s(%d) enter\n", __func__, __LINE__);
	ttyinfo_dbg(bt_node);

	if (bif_ringbuf_is_empty(bt_node)) {
		pr_err("%s(%d)...rc=%d\n", __func__, __LINE__, rc);
		return 0;
	}
#ifdef CONFIG_HOBOT_BIF_AP
	memset(pself->rbuf, 0, pself->size);
	tty_dump_log(pself->rbuf, 128);
	bif_sync_cpbuf((unsigned long)(bt_node->addr_phy_r),
		       pself->size, pself->rbuf);
#endif

	count = bif_ringbuf_used(bt_node);
	if (size > count)
		size = count;
	rc = ringbuf_read(pself, buf, size);
	if (rc < 0)
		goto fail;

	bif_tty_irq();
	ttyinfo_dbg(bt_node);
fail:
	tty_debug_log("%s(%d)...rc=%d leave\n", __func__, __LINE__, rc);
	return rc;
}

static ssize_t bif_tty_write(struct file *filp, const char __user *buf,
			     size_t size, loff_t *ppos)
{
	int rc = 0;
	unsigned int count = size;
	struct bif_tty_node *bt_node = filp->private_data;
	struct ringbuf_t *pself = bt_node->rb_self;

	tty_debug_log("%s(%d) enter\n", __func__, __LINE__);
	ttyinfo_dbg(bt_node);

	if (size > (ringbuf_capacity(pself))) {
		rc = -EPERM;
		pr_err("%s(%d)...rc=%d\n", __func__, __LINE__, rc);
		return rc;
	}
	count = bif_ringbuf_free(bt_node);
	if (size > count) {
		wait_event(tb_dev->wq, bif_ringbuf_free(bt_node) >= size);
		tty_debug_log("%s(%d)...count=%d\n", __func__, __LINE__, count);
	}

	rc = ringbuf_write(pself, buf, size);
	if (rc < 0)
		goto fail;

#ifdef CONFIG_HOBOT_BIF_AP
	bif_sync_apbuf((unsigned long)(bt_node->addr_phy_w),
		       pself->size, pself->wbuf);
#endif
	bif_tty_irq();
	ttyinfo_dbg(bt_node);
fail:
	tty_debug_log("%s(%d)...rc=%d leave\n", __func__, __LINE__, rc);
	return rc;
}

static loff_t bif_tty_llseek(struct file *filp, loff_t offset, int orig)
{
	/* not support */
	tty_debug_log("%s(%d)...leave\n", __func__, __LINE__);
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

static int bif_tty_alloc(struct bif_tty_cdev *cdev)
{

	cdev = kzalloc(sizeof(struct bif_tty_cdev), GFP_KERNEL);
	if (!cdev)
		goto err_alloc_cdev;

	cdev->name = "ttyBIF";
	cdev->num_nodes = NODE_NUM;

#ifdef CONFIG_HOBOT_BIF_AP
	cdev->buff_r = kzalloc(NODE_NUM * TTY_BUF_SIZE, GFP_KERNEL);
	if (!cdev->buff_r)
		goto err_alloc_buff_r;

	cdev->buff_w = kzalloc(NODE_NUM * TTY_BUF_SIZE, GFP_KERNEL);
	if (!cdev->buff_w)
		goto err_alloc_buff_w;

#endif
	cdev->tb_node = kzalloc(NODE_NUM * sizeof(struct bif_tty_node),
				GFP_KERNEL);
	if (!cdev->tb_node)
		goto err_alloc_node;

	return 0;

err_alloc_node:
#ifdef CONFIG_HOBOT_BIF_AP
	kfree(cdev->buff_w);

err_alloc_buff_w:
	kfree(cdev->buff_r);

err_alloc_buff_r:
#endif
	kfree(cdev);
err_alloc_cdev:
	return -ENOMEM;
}

static int bif_tty_free(struct bif_tty_cdev *cdev)
{

#ifdef CONFIG_HOBOT_BIF_AP
	kfree(cdev->buff_r);
	kfree(cdev->buff_w);
#endif

	kfree(cdev->tb_node);
	kfree(cdev);
	return 0;
}

static int bif_tty_alloc_base(struct bif_tty_cdev *cdev)
{
	int rc = 0;
	void *addr = NULL;

	tty_debug_log("%s(%d) enter\n", __func__, __LINE__);
	addr = bif_alloc_base(BUFF_SMD,
			      cdev->num_nodes * sizeof(struct ringbuf_t));
	if (addr == (void *)-1 || addr == NULL) {
		rc = -ENOSPC;
		pr_err("%s(%d)...ret=%d\n", __func__, __LINE__, rc);
	} else {
		rc = 0;
		cdev->rb_base_self = (struct ringbuf_t(*)[])addr;
		bif_tty_irq();
	}

	return rc;
}

static int bif_tty_query_otherbase(struct bif_tty_cdev *cdev)
{
	void *addr = NULL;

	tty_debug_log("%s(%d) enter\n", __func__, __LINE__);
	addr = bif_query_otherbase_wait(BUFF_SMD);
	if (addr == (void *)-1
		|| addr == NULL
		|| addr == (void *)-ERESTARTSYS) {

		addr = NULL;
		pr_err("%s(%d) query other address fail\n", __func__, __LINE__);
	}

	cdev->rb_base_other = (struct ringbuf_t(*)[])addr;
	return 0;
}

#ifdef CONFIG_HOBOT_BIF_CP
static int bif_tty_alloc_buff(struct bif_tty_cdev *cdev)
{
	int rc = 0;
	void *addr = NULL;
	void *addr_phy = NULL;

	addr = kzalloc((2 * NODE_NUM + 1) * TTY_BUF_SIZE, GFP_ATOMIC);
	if (addr == NULL) {
		rc = -ENOMEM;
		pr_err("%s(%d)...rc=%d\n", __func__, __LINE__, rc);
		goto fail;
	}

	addr = (void *)(((unsigned long)addr / 512) * 512);
	addr_phy = (void *)virt_to_phys(addr);
	bif_register_address(BUFF_SMD, (void *)addr_phy);
	bif_tty_irq();
	cdev->buff_addr_vir = addr;
	cdev->buff_addr_phy = addr_phy;

	tty_debug_log("%s(%d)cp...addr=%lx addr_phy=%lx\n",
		      __func__,
		      __LINE__, (unsigned long)addr, (unsigned long)addr_phy);

fail:
	return rc;
}
#endif

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_tty_query_buff(struct bif_tty_cdev *cdev)
{
	void *addr = NULL;

	addr = bif_query_address_wait(BUFF_SMD);
	if (addr == (void *)-1
		|| addr == NULL
		|| addr == (void *)-ERESTARTSYS) {

		addr = NULL;
		tty_debug_log("%s(%d)ap wait timeout\n", __func__, __LINE__);
	}
	cdev->buff_addr_phy = addr;

	return 0;
}
#endif

static int bif_tty_init_node(struct bif_tty_cdev *cdev)
{
	int i;
	struct bif_tty_node *tmp;

	for (i = 0; i < NODE_NUM; i++) {
		tmp = cdev->tb_node[i];

		tmp->rb_self = cdev->rb_base_self[i];
		tmp->rb_other = cdev->rb_base_other[i];
#ifdef CONFIG_HOBOT_BIF_AP
		tmp->addr_phy_r = cdev->buff_addr_phy + i * 2 * TTY_BUF_SIZE;
		tmp->addr_phy_w = tmp->addr_phy_r + TTY_BUF_SIZE;
#endif
	}
	return 0;
}

static int bif_tty_init_ringbuff(struct bif_tty_cdev *cdev)
{
	int i;
	struct bif_tty_node *tmp;

	for (i = 0; i < NODE_NUM; i++) {
		tmp = cdev->tb_node[i];

		tmp->rb_self->head = 0;
		tmp->rb_self->tail = 0;
		tmp->rb_self->size = TTY_BUF_SIZE;
#ifdef CONFIG_HOBOT_BIF_AP
		/*
		 * ap side write data to alloc buff first
		 */
		tmp->rb_self->rbuf = cdev->buff_r + i * 2 * TTY_BUF_SIZE;
		tmp->rb_self->wbuf = tmp->rb_self->rbuf + TTY_BUF_SIZE;
#else
		/*
		 * cp side write data to alloc buff directly
		 */
		tmp->rb_self->wbuf = cdev->buff_addr_vir + i * 2 * TTY_BUF_SIZE;
		tmp->rb_self->rbuf = tmp->rb_self->wbuf + TTY_BUF_SIZE;
#endif
	}
	return 0;
}

static int bif_tty_init_chrdev(struct bif_tty_cdev *cdev)
{
	int rc;
	dev_t dev;
	int devnum, i, minor, major;
	struct device *device;

	rc = alloc_chrdev_region(&dev, 0, cdev->num_nodes, cdev->name);
	if (rc) {
		pr_err("Failed to obtain major/minors");
		return rc;
	}
	cdev->major = major = MAJOR(dev);
	cdev->minor = minor = MINOR(dev);

	cdev->drv_class = class_create(THIS_MODULE, cdev->name);
	if (IS_ERR(cdev->drv_class)) {
		rc = IS_ERR(cdev->drv_class);
		pr_err("Failed to class_create. Aborting.\n");
		goto dest_class;
	}

	cdev_init(&cdev->cdev, &bif_tty_fops);
	cdev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&cdev->cdev, MKDEV(major, minor), cdev->num_nodes);
	if (rc) {
		pr_err("Failed to add cdev. Aborting.\n");
		goto unregister_chrdev;
	}

	for (i = minor, devnum = 0; devnum < cdev->num_nodes; devnum++, i++) {

		device = device_create(cdev->drv_class,
				       NULL,
				       MKDEV(major, i),
				       NULL, "%s%d", cdev->name, devnum);

		if (IS_ERR(device)) {
			pr_err("Failed to create %s%d device. Aborting.\n",
			       cdev->name, devnum);
			rc = -ENODEV;
			goto unroll_device_create;
		}
	}

	return 0;		/* succeed */

unroll_device_create:
	devnum--;
	i--;
	for (; devnum >= 0; devnum--, i--)
		device_destroy(cdev->drv_class, MKDEV(major, i));

	cdev_del(&cdev->cdev);

dest_class:
	class_destroy(cdev->drv_class);

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
	int rc;

	tty_debug_log("%s(%d) enter\n", __func__, __LINE__);

	bif_register_irq(BUFF_SMD, bif_tty_irq_handler);
	rc = bif_tty_alloc_base(cdev);
	if (rc)
		goto fail;

	bif_tty_query_otherbase(cdev);
#ifdef CONFIG_HOBOT_BIF_CP
	rc = bif_tty_alloc_buff(cdev);
	if (rc)
		goto fail;
#else
	bif_tty_query_buff(cdev);
#endif
	bif_tty_init_node(cdev);
	bif_tty_init_ringbuff(cdev);
	tty_debug_log("%s(%d)...leave\n", __func__, __LINE__);
	return 0;
fail:
	pr_err("%s(%d)...rc=%d\n", __func__, __LINE__, rc);
	return rc;

}

static int __init bif_tty_init(void)
{
	int rc;

	tty_debug_log("%s(%d) enter\n", __func__, __LINE__);

	rc = bif_tty_alloc(tb_dev);
	if (rc)
		goto fail_alloc;

	rc = bif_tty_init_chrdev(tb_dev);
	if (rc)
		goto fail;

	rc = bif_tty_init_base(tb_dev);
	if (rc)
		goto fail;

	tty_debug_log("%s(%d)...leave\n", __func__, __LINE__);
	return 0;

fail:
	bif_tty_free(tb_dev);
fail_alloc:
	return rc;
}

module_init(bif_tty_init);

static void __exit bif_tty_exit(void)
{
	bif_tty_free_chrdev(tb_dev);
	bif_tty_free(tb_dev);
}

module_exit(bif_tty_exit);

MODULE_DESCRIPTION("Driver for Bif tty");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL v2");
