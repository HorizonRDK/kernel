/*
 * X2 CNN driver (found in Hobot Platform)
 *
 * 2017 - 2018 (C) Hobot Inc.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any
 * later version.
 *
 */
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/cacheflush.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of_reserved_mem.h>
#include <linux/reset.h>
#include <linux/printk.h>
#include <linux/poll.h>
#include <linux/debugfs.h>

#include "x2_cnn_host.h"

static DEFINE_MUTEX(x2_cnn_mutex);
static char *g_chrdev_name = "cnn";
static struct dentry *cnn0_debugfs_root;
static struct dentry *cnn1_debugfs_root;
static inline u32 x2_cnn_reg_read(struct x2_cnn_dev *dev, u32 off)
{
	return readl(dev->cnn_base + off);
}

static inline void x2_cnn_reg_write(struct x2_cnn_dev *dev,
	u32 off, u32 val)
{
	writel(val, dev->cnn_base + off);
}

int fc_fifo_stat_info(struct seq_file *m, void *data)
{
	struct cnn_info_node *node = (struct cnn_info_node *)m->private;
	struct x2_cnn_dev *dev = node->cnn_dev;
	u32 fc_head_idx, fc_tail_idx,
	    fc_head_flag, fc_tail_flag,
	    fc_depth;

	fc_head_idx = x2_cnn_reg_read(dev, X2_CNN_FC_HEAD);
	fc_head_flag = fc_head_idx & X2_CNN_FC_IDX_FLAG;
	fc_head_idx &= X2_CNN_MAX_FC_LEN_MASK;

	fc_tail_idx = x2_cnn_reg_read(dev, X2_CNN_FC_TAIL);
	fc_tail_flag = fc_head_idx & X2_CNN_FC_IDX_FLAG;
	fc_tail_idx &= X2_CNN_MAX_FC_LEN_MASK;

	fc_depth = x2_cnn_reg_read(dev, X2_CNN_FC_LEN);
	seq_printf(m, "fc_len	 fc_head    fc_head_flag    fc_tail    fc_tail_flag\n");
	seq_printf(m, "%d	 %d	    %d		    %d	       %d\n",
		fc_depth, fc_head_idx, fc_head_flag, fc_tail_idx, fc_tail_flag);

	return 0;
}

int fc_dump_info(struct seq_file *m, void *data)
{
	int i;
	struct cnn_info_node *node = (struct cnn_info_node *)m->private;
	struct x2_cnn_dev *dev = node->cnn_dev;
	unsigned char *fc_buf = kmalloc(CNN_FC_SPACE_LEN, GFP_KERNEL);

	memset(fc_buf, 0, CNN_FC_SPACE_LEN);

	memcpy(fc_buf, dev->fc_base, CNN_FC_SPACE_LEN);
	seq_printf(m, "0x%x ", fc_buf[0]);
	//for (i = 1; i <= CNN_FC_SPACE_LEN; i++) {
	for (i = 1; i < CNN_FC_SPACE_LEN; i++) {
		seq_printf(m, "[%d] 0x%x ", i, fc_buf[i]);
		if (i % 15 == 0)
			seq_puts(m, "\n");
	}
	return 0;
}

static struct cnn_debugfs_info cnn_debugfs_list[] = {
	{"fc_fifo_stat", fc_fifo_stat_info},
	{"fc_dump", fc_dump_info},
};
#define CNN_DEBUGFS_ENTRIES ARRAY_SIZE(cnn_debugfs_list)

static int cnn_debugfs_open(struct inode *inode, struct file *file)
{
	struct cnn_info_node *node = inode->i_private;

	return single_open(file, node->info_ent->show, node);
}


static const struct file_operations cnn_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = cnn_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void cnn_flush_dcache_range(void *addr, size_t len)
{
	__flush_dcache_area(addr, len);
}

/*
 * x2_cnn_fc_gap_mem_init - init memory pattern for
 * function call gap(only for debug)
 *
 */
void x2_cnn_fc_gap_mem_init(struct x2_cnn_dev *dev)
{
	int mem_pattern = 0x5a;
	void *cnn_fc_start_gap_virt, *cnn_fc_end_gap_virt;

	cnn_fc_start_gap_virt =
		phys_to_virt(dev->fc_phys_base - CNN_FC_GAP_LEN);
	memset(cnn_fc_start_gap_virt, mem_pattern, CNN_FC_GAP_LEN);

	cnn_fc_end_gap_virt =
		phys_to_virt(dev->fc_phys_base + dev->fc_mem_size);
	memset(cnn_fc_end_gap_virt, mem_pattern, CNN_FC_GAP_LEN);
}


/**
 * x2_cnn_reset_assert - asserts reset using reset framework
 * @rstc: pointer to reset_control
 *
 * Return: 0 on success or error on failure
 */
static int x2_cnn_reset_assert(struct reset_control *rstc)
{
	int rc = 0;

	rc = reset_control_assert(rstc);
	if (rc < 0) {
		pr_err("%s: failed\n", __func__);
		return rc;
	}

	return rc;
}

/**
 * x2_cnn_reset_release - de-asserts reset using reset framework
 * @rstc: pointer to reset_control
 *
 * Return: 0 on success or error on failure
 */
static int x2_cnn_reset_release(struct reset_control *rstc)
{
	int rc = 0;

	rc = reset_control_deassert(rstc);
	if (rc < 0) {
		pr_err("%s: failed\n", __func__);
		return rc;
	}

	return rc;
}

static void x2_cnnbus_wm_set(struct x2_cnn_dev *dev, u32 reg_off,
	u32 wd_maxlen, u32 wd_endian, u32 wd_priority)
{
	u32 reg_val;

	reg_val = x2_cnn_reg_read(dev, reg_off);
	reg_val &= ~(X2_CNN_WD_MAXLEN_M_MASK |
		X2_CNN_WD_ENDIAN_M_MASK |
		X2_CNN_WD_PRIORITY_M_MASK);

	reg_val |= X2_CNN_WD_MAXLEN_M(wd_maxlen) |
		X2_CNN_WD_ENDIAN_M(wd_endian) |
		X2_CNN_WD_PRIORITY_M(wd_priority);

	x2_cnn_reg_write(dev, reg_off, reg_val);
}

static void x2_cnnbus_rm_set(struct x2_cnn_dev *dev, u32 reg_off,
	u32 rd_maxlen, u32 rd_endian, u32 rd_priority)
{
	u32 reg_val;

	reg_val = x2_cnn_reg_read(dev, reg_off);
	reg_val &= ~(X2_CNN_RD_MAXLEN_M_MASK |
		X2_CNN_RD_ENDIAN_M_MASK |
		X2_CNN_RD_PRIORITY_M_MASK);

	reg_val |= X2_CNN_RD_MAXLEN_M(rd_maxlen) |
		X2_CNN_RD_ENDIAN_M(rd_endian) |
		X2_CNN_RD_PRIORITY_M(rd_priority);

	x2_cnn_reg_write(dev, reg_off, reg_val);
}

static void x2_cnn_set_default_fc_depth(struct x2_cnn_dev *dev, int fc_depth)
{
	u32 reg_val;

	if (fc_depth >= 1023)
		fc_depth = 1023;

	reg_val = x2_cnn_reg_read(dev, X2_CNN_FC_LEN);
	reg_val &=  ~(X2_CNN_PE0_FC_LENGTH_MASK);

	reg_val |= X2_CNN_PE0_FC_LENGTH(fc_depth);
	x2_cnn_reg_write(dev, X2_CNN_FC_LEN, reg_val);
}

static void x2_cnn_set_fc_base(struct x2_cnn_dev *dev)
{
	u32 reg_val;

	reg_val = x2_cnn_reg_read(dev, X2_CNN_FC_BASE);
	reg_val &=  ~(X2_CNN_PE0_FC_BASE_MASK);

	reg_val |= X2_CNN_PE0_FC_BASE(dev->fc_phys_base);
	x2_cnn_reg_write(dev, X2_CNN_FC_BASE, reg_val);
}

static int x2_cnn_hw_init(struct x2_cnn_dev *dev)
{

	/* Config axi write master */
	x2_cnnbus_wm_set(dev, X2_CNNBUS_CTRL_WM_0, 0x80, 0xf, 0x1);
	x2_cnnbus_wm_set(dev, X2_CNNBUS_CTRL_WM_1, 0x80, 0xf, 0x2);
	x2_cnnbus_wm_set(dev, X2_CNNBUS_CTRL_WM_2, 0x8, 0x0, 0x3);
	x2_cnnbus_wm_set(dev, X2_CNNBUS_CTRL_WM_3, 0x80, 0x0, 0x4);

	/* Config axi read master */
	x2_cnnbus_rm_set(dev, X2_CNNBUS_CTRL_RM_0, 0x80, 0xf, 0x4);
	x2_cnnbus_rm_set(dev, X2_CNNBUS_CTRL_RM_1, 0x80, 0xf, 0x4);
	x2_cnnbus_rm_set(dev, X2_CNNBUS_CTRL_RM_2, 0x8, 0xf, 0x4);
	x2_cnnbus_rm_set(dev, X2_CNNBUS_CTRL_RM_3, 0x80, 0x8, 0x5);
	x2_cnnbus_rm_set(dev, X2_CNNBUS_CTRL_RM_4, 0x80, 0xf, 0x4);
	x2_cnnbus_rm_set(dev, X2_CNNBUS_CTRL_RM_5, 0x80, 0x0, 0x6);

	/* Set axibus id */
	x2_cnn_reg_write(dev, X2_CNNBUS_AXIID, 0x0);

	return 0;
}


/**
 * x2_cnn_get_resets - sw reset cnn controller
 * @cnn_dev: pointer cnn dev struct
 * @cnn_id:  cnn id (0/1)
 *
 * Return: 0 on success or error on failure
 */

static int x2_cnn_get_resets(struct x2_cnn_dev *cnn_dev, int cnn_id)
{
	char *name;
	struct reset_control *rst_temp;

	if (cnn_id == 0) {
		cnn_dev->cnn0_rst =
			devm_reset_control_get(cnn_dev->dev, "cnn0_rst");
		if (IS_ERR(cnn_dev->cnn0_rst)) {
			name = "cnn0_rst";
			rst_temp = cnn_dev->cnn0_rst;
			goto error;
		}
	} else if (cnn_id == 1) {
		cnn_dev->cnn1_rst =
			devm_reset_control_get(cnn_dev->dev, "cnn1_rst");
		if (IS_ERR(cnn_dev->cnn1_rst)) {
			name = "cnn1_rst";
			rst_temp = cnn_dev->cnn1_rst;
			goto error;
		}
	}

	return 0;
error:
	dev_err(cnn_dev->dev, "failed to get %s reset signal\n", name);
	return PTR_ERR(rst_temp);
}

/**
 * x2_cnn_hw_reset_reinit - sw reset cnn controller
 * @cnn_dev: pointer cnn dev struct
 * @cnn_id:  cnn id (0/1)
 *
 * Return: 0 on success or error on failure
 */
static int x2_cnn_hw_reset_reinit(struct x2_cnn_dev *cnn_dev, int cnn_id)
{
	int ret = 0;

	switch (cnn_id) {
	case 0:
		x2_cnn_reset_assert(cnn_dev->cnn0_rst);
		udelay(1);
		x2_cnn_reset_release(cnn_dev->cnn0_rst);
		break;
	case 1:
		x2_cnn_reset_assert(cnn_dev->cnn1_rst);
		udelay(1);
		x2_cnn_reset_release(cnn_dev->cnn1_rst);
		break;

	default:
		pr_err("%s: Invalid cnn_id\n", __func__);
		ret = -EINVAL;
		break;
	}

	if (!ret) {
		x2_cnn_hw_init(cnn_dev);
		x2_cnn_set_fc_base(cnn_dev);
		x2_cnn_set_default_fc_depth(cnn_dev, 1023);
	}
	return ret;
}

static irqreturn_t x2_cnn_interrupt_handler(int irq, void *dev_id)
{
	struct x2_cnn_dev *dev = (struct x2_cnn_dev *)dev_id;
	unsigned long flags;
	u32 irq_status;

	spin_lock_irqsave(&dev->cnn_spin_lock, flags);

	irq_status = x2_cnn_reg_read(dev, X2_CNNINT_STATUS);
	x2_cnn_reg_write(dev, X2_CNNINT_MASK, 0x1);
	dev->x2_cnn_int_num = x2_cnn_reg_read(dev, X2_CNNINT_NUM);

	x2_cnn_reg_write(dev, X2_CNNINT_MASK, 0x0);

	spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);

	pr_info("!!!!!!xxxx x2 cnn interrupt\n");
	tasklet_schedule(&dev->tasklet);
	return IRQ_HANDLED;
}

static ssize_t x2_cnn_read(struct file *filp, char __user *userbuf,
	size_t count, loff_t *f_pos)
{
	return 0;
}

static ssize_t x2_cnn_write(struct file *filp,
	const char __user *userbuf, size_t count, loff_t *f_pos)
{
	return 0;
}

static void x2_cnn_set_fc_tail_idx(struct x2_cnn_dev *dev, u32 fc_tail_idx)
{
	u32 reg_val;

	reg_val = x2_cnn_reg_read(dev, X2_CNN_FC_TAIL);
	reg_val &=  ~(X2_CNN_PE0_FC_TAIL_MASK);

	reg_val |= X2_CNN_PE0_FC_TAIL(fc_tail_idx);
	x2_cnn_reg_write(dev, X2_CNN_FC_TAIL, reg_val);
}

#ifdef CNN_DEBUG
static void dump_fc_fifo_status(struct x2_cnn_dev *dev)
{
	u32 fc_head_idx, fc_tail_idx,
	    fc_head_flag, fc_tail_flag,
	    fc_depth;

	fc_head_idx = x2_cnn_reg_read(dev, X2_CNN_FC_HEAD);
	fc_head_flag = fc_head_idx & X2_CNN_FC_IDX_FLAG;
	fc_head_idx &= X2_CNN_MAX_FC_LEN_MASK;

	fc_tail_idx = x2_cnn_reg_read(dev, X2_CNN_FC_TAIL);
	fc_tail_flag = fc_head_idx & X2_CNN_FC_IDX_FLAG;
	fc_tail_idx &= X2_CNN_MAX_FC_LEN_MASK;

	fc_depth = x2_cnn_reg_read(dev, X2_CNN_FC_LEN);

	pr_info("X2-CNN function call fifo status:\n");
	pr_info("current function call length:%d\n", fc_depth);
	pr_info("head_idx:%d head_flag:%d\n", fc_head_idx, fc_head_flag);
	pr_info("tail_idx:%d tail_flag:%d\n", fc_tail_idx, fc_tail_flag);

}
#endif

/**
 * x2_cnn_get_fc_fifo_spaces - get available spaces from cnn fc fifo queue
 * @dev: pointer to struct x2_cnn_dev
 *
 * Return: free_fc_fifo = 0 indicate no spaces
 *	   free_fc_fifo > 0 indicate have free_fc_fifo spaces can used
 *	   others failed
 */

static u32 x2_cnn_get_fc_fifo_spaces(struct x2_cnn_dev *dev)
{
	u32 free_fc_fifo = 0;
	u32 fc_head_idx, fc_tail_idx,
	    fc_head_flag, fc_tail_flag;
	u32 fc_depth;

	fc_depth = x2_cnn_reg_read(dev, X2_CNN_FC_LEN);

	fc_head_idx = x2_cnn_reg_read(dev, X2_CNN_FC_HEAD);
	fc_head_flag = fc_head_idx & X2_CNN_FC_IDX_FLAG;

	fc_tail_idx = x2_cnn_reg_read(dev, X2_CNN_FC_TAIL);
	fc_tail_flag = fc_head_idx & X2_CNN_FC_IDX_FLAG;

	if (fc_head_flag != fc_tail_flag)
		free_fc_fifo = fc_head_idx - fc_tail_idx;
	else
		free_fc_fifo = fc_depth - fc_tail_idx + fc_head_idx + 1;

	pr_info("fc_depth:0x%x,get fifo spaces return val:%d\n",
			fc_depth, free_fc_fifo);
	return free_fc_fifo;
}

/**
 * x2_cnn_fc_fifo_enqueue - fill the function call into the reserved memory
 * @dev: pointer to struct x2_cnn_dev
 * fc_buf: function call buf struct
 *
 * Return: 0 success, others failed
 */
static u32 x2_cnn_fc_fifo_enqueue(struct x2_cnn_dev *dev,
		struct x2_cnn_fc_info *fc_buf)
{
	u32 rc = 0;
	u32 free_fc_fifo = 0;
	u32 fc_head_idx, fc_tail_idx,
	    fc_head_flag, fc_tail_flag;
	u32 fc_depth, insert_fc_cnt, residue_fc_cnt;

	fc_depth = x2_cnn_reg_read(dev, X2_CNN_FC_LEN);
	fc_head_idx = x2_cnn_reg_read(dev, X2_CNN_FC_HEAD);
	fc_head_flag = fc_head_idx & X2_CNN_FC_IDX_FLAG;

	fc_tail_idx = x2_cnn_reg_read(dev, X2_CNN_FC_TAIL);
	fc_tail_flag = fc_head_idx & X2_CNN_FC_IDX_FLAG;

	if (fc_head_flag != fc_tail_flag)
		free_fc_fifo = fc_head_idx - fc_tail_idx;
	else
		free_fc_fifo = fc_depth - fc_tail_idx + fc_head_idx + 1;

	if (fc_buf->fc_cnt > free_fc_fifo) {
		rc = -1;
		pr_err("no available fc fifo spaces\n");
		return rc;
	}
	fc_tail_idx &= X2_CNN_MAX_FC_LEN_MASK;

	if ((fc_tail_idx + fc_buf->fc_cnt)  > fc_depth) {
		insert_fc_cnt = fc_depth - fc_tail_idx + 1;
		memcpy(dev->fc_base + fc_tail_idx * X2_CNN_FC_SIZE,
			fc_buf->fc_info, insert_fc_cnt * X2_CNN_FC_SIZE);

		if (fc_tail_flag)
			fc_tail_flag = 0;
		else
			fc_tail_flag = X2_CNN_FC_IDX_FLAG;

		residue_fc_cnt = fc_buf->fc_cnt - insert_fc_cnt;
		if (residue_fc_cnt > 0) {
			memcpy(dev->fc_base,
			fc_buf->fc_info + (insert_fc_cnt * X2_CNN_FC_SIZE),
			residue_fc_cnt * X2_CNN_FC_SIZE);
		}
		x2_cnn_set_fc_tail_idx(dev, residue_fc_cnt | fc_tail_flag);
	} else {
		memcpy(dev->fc_base + (fc_tail_idx * X2_CNN_FC_SIZE),
			fc_buf->fc_info, fc_buf->fc_cnt * X2_CNN_FC_SIZE);

		x2_cnn_set_fc_tail_idx(dev,
				fc_tail_flag | (fc_tail_idx + fc_buf->fc_cnt));
	}

#ifdef CNN_DEBUG
	dump_fc_fifo_status(dev);
#endif
	return rc;
}

static u32 x2_cnn_get_int_num(struct x2_cnn_dev *dev)
{
	return dev->x2_cnn_int_num;
}

static int x2_cnn_open(struct inode *inode, struct file *filp)
{
	int rc = 0;
	struct x2_cnn_dev *devdata;

	mutex_lock(&x2_cnn_mutex);

	devdata = container_of(inode->i_cdev, struct x2_cnn_dev, i_cdev);
	filp->private_data = devdata;

	mutex_unlock(&x2_cnn_mutex);

	x2_cnn_reg_read(devdata, X2_CNN_FC_LEN);

	return rc;
}

static int x2_cnn_release(struct inode *inode, struct file *filp)
{
	struct x2_cnn_dev *devdata;

	mutex_lock(&x2_cnn_mutex);
	devdata = filp->private_data;
	mutex_unlock(&x2_cnn_mutex);
	return 0;
}

static unsigned int cnn_ioctl_dir(unsigned int cmd)
{
	switch (cmd) {
	case CNN_IOC_GET_FC_STA:
	case CNN_IOC_GET_INT_NUM:
		return _IOC_READ;
	case CNN_IOC_RST:
	case CNN_IOC_FC_ENQUEUE:
		return _IOC_WRITE;
	default:
		return _IOC_DIR(cmd);
	}
}

static long x2_cnn_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int  rc = 0;
	struct x2_cnn_dev *dev;
	u32 dir;
	union cnn_ioctl_arg data;
	void *kernel_fc_data;

	int count = 0;

	dir = cnn_ioctl_dir(cmd);

	if (_IOC_TYPE(cmd) != CNN_IOCTL_MAGIC) {
		pr_err("ioctl command magic number does not match.\n");
		return -EINVAL;
	}

	if (_IOC_SIZE(cmd) > sizeof(data))
		return -EINVAL;

	/*
	 * The copy_from_user is unconditional here for both read and write
	 * to do the validate. If there is no write for the ioctl, the
	 * buffer is cleared
	 */
	if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
		pr_err("%s: copy data failed from userspace\n", __func__);
		return -EFAULT;
	}

	if (!(dir & _IOC_WRITE))
		memset(&data, 0, sizeof(data));

	dev = file->private_data;

	switch (cmd) {
	case CNN_IOC_GET_FC_STA:
		mutex_lock(&dev->cnn_lock);
		data.fc_status.free_fc_fifo_cnt =
			x2_cnn_get_fc_fifo_spaces(dev);
		mutex_unlock(&dev->cnn_lock);
		break;
	case CNN_IOC_FC_ENQUEUE:
		//size_t size_tmp = data.fc_data.fc_cnt * X2_CNN_FC_SIZE;

		kernel_fc_data = kmalloc(data.fc_data.fc_cnt * X2_CNN_FC_SIZE,
					GFP_KERNEL);
		if (!kernel_fc_data) {
			pr_err("kmalloc kernel_fc_data failed\n");
			return -ENOMEM;
		}
		if (copy_from_user(kernel_fc_data,
				(void __user *)data.fc_data.fc_info,
				data.fc_data.fc_cnt * X2_CNN_FC_SIZE)) {
			pr_err("%s: copy fc data failed from userspace\n",
				__func__);
			return -EFAULT;
		}
		data.fc_data.fc_info = kernel_fc_data;

		struct hbrt_x2_funccall_s *tmp_ptr =
			(struct hbrt_x2_funccall_s *)data.fc_data.fc_info;

		pr_debug("!!!!func: %s, line: %d\n"
				"tmp_ptr->dyn_base_addr5 is 0x%x,\n"
				"tmp_ptr->dyn_base_addr4 is 0x%x,\n"
				 "tmp_ptr->interrupt_num is %d,\n"
				 "tmp_ptr->instruction_length is %d,\n"
				 "tmp_ptr->instruction_address is 0x%x,\n"
				 "tmp_ptr->dyn_base_addr0 is 0x%x,\n"
				 "tmp_ptr->dyn_base_addr1 is 0x%x,\n"
				 "tmp_ptr->dyn_base_addr3 is 0x%x.\n",
				 __func__, __LINE__,
				tmp_ptr->dyn_base_addr5,
				tmp_ptr->dyn_base_addr4,
				tmp_ptr->interrupt_num,
				tmp_ptr->instruction_length,
				tmp_ptr->instruction_address,
				tmp_ptr->dyn_base_addr0,
				tmp_ptr->dyn_base_addr1,
				tmp_ptr->dyn_base_addr3);

		mutex_lock(&dev->cnn_lock);
		rc = x2_cnn_fc_fifo_enqueue(dev, &data.fc_data);
		if (rc < 0) {
			mutex_unlock(&dev->cnn_lock);
			pr_err("%s: failed to fill fc fifo\n", __func__);
			return rc;
		}
		mutex_unlock(&dev->cnn_lock);
		kfree(kernel_fc_data);
		break;
	case CNN_IOC_RST:
		mutex_lock(&dev->cnn_lock);
		rc = x2_cnn_hw_reset_reinit(dev, data.rst_data.cnn_rst_id);
		if (rc < 0) {
			mutex_unlock(&dev->cnn_lock);
			pr_err("%s: failed to reset cnn%d\n",
				__func__, data.rst_data.cnn_rst_id);
			return rc;
		}
		mutex_unlock(&dev->cnn_lock);
		break;
	case CNN_IOC_GET_INT_NUM:
		mutex_lock(&dev->cnn_lock);
		data.int_num_data.cnn_int_num = x2_cnn_get_int_num(dev);
		dev->x2_cnn_int_num = 0;
		mutex_unlock(&dev->cnn_lock);
		break;
	default:
		pr_err("%s: Invalid ioctl Argument\n", __func__);
		return -EINVAL;

	}

	if (dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			pr_err("%s: copy data to userspace failed\n", __func__);
			return -EFAULT;
		}
	}
	return rc;
}

static u32 x2_cnn_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct x2_cnn_dev *dev = filp->private_data;
	poll_wait(filp, &dev->cnn_int_wait, wait);

	mutex_lock(&dev->cnn_lock);
	if (dev->irq_triggered) {
		mask |= POLLIN | POLLRDNORM;
		dev->irq_triggered = 0;
	}
	mutex_unlock(&dev->cnn_lock);

	return mask;
}

static const struct file_operations cnn_fops = {
	.owner		= THIS_MODULE,
	.read		= x2_cnn_read,
	.poll		= x2_cnn_poll,
	.write		= x2_cnn_write,
	.open		= x2_cnn_open,
	.release	= x2_cnn_release,
	.unlocked_ioctl = x2_cnn_ioctl,
};

static int x2_cnn_init_chrdev(struct x2_cnn_dev *dev)
{
	int rc;

	// Allocate a major and minor number region for the character device
	rc = alloc_chrdev_region(&dev->dev_num, dev->minor_num,
			dev->num_devices, dev->chrdev_name);
	if (rc < 0) {
		pr_err("Unable to allocate character device region.\n");
		goto ret;
	}

	// Create a device class for our device
	dev->dev_class = class_create(THIS_MODULE, dev->chrdev_name);
	if (IS_ERR(dev->dev_class)) {
		pr_err("Unable to create a device class.\n");
		rc = PTR_ERR(dev->dev_class);
		goto free_chrdev_region;
	}

	/* Create a device for our module. This will create a file on the
	 * filesystem, under "/dev/dev->chrdev_name".
	 */
	device_create(dev->dev_class, dev->dev, dev->dev_num, NULL,
		dev->chrdev_name);

	// Register our character device with the kernel
	cdev_init(&dev->i_cdev, &cnn_fops);
	rc = cdev_add(&dev->i_cdev, dev->dev_num, dev->num_devices);
	if (rc < 0) {
		pr_err("Unable to add a character device.\n");
		goto device_cleanup;
	}

	return 0;

device_cleanup:
	device_destroy(dev->dev_class, dev->dev_num);
free_chrdev_region:
	unregister_chrdev_region(dev->dev_num, dev->num_devices);
ret:
	return rc;
}

/**
 * x2_cnn_do_tasklet - Schedule wake up tasklet
 * @data: Pointer to cnn_dev structure
 */
static void x2_cnn_do_tasklet(unsigned long data)
{
	struct x2_cnn_dev *dev = (struct x2_cnn_dev *)data;

	mutex_lock(&dev->cnn_lock);
	wake_up(&dev->cnn_int_wait);
	dev->irq_triggered = 1;
	mutex_unlock(&dev->cnn_lock);
}

static void *cnn_ram_vmap(phys_addr_t start, size_t size,
		unsigned int memtype)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	switch (memtype) {
	case CNN_MT_WB:
		prot = PAGE_KERNEL;
		break;
	case CNN_MT_UC:
		prot = pgprot_noncached(PAGE_KERNEL);
		break;
	case CNN_MT_WC:
		prot = pgprot_writecombine(PAGE_KERNEL);
		break;
	case CNN_MT_WT:
		prot = __pgprot(PROT_NORMAL_WT);
		break;
	default:
		/* Default set normal memory(cacheable) */
		prot = PAGE_KERNEL;
	}

	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n",
		       __func__, page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;

		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vm_map_ram(pages, page_count, -1, prot);
	kfree(pages);

	return vaddr;
}

int cnn_debugfs_remove_files(const struct cnn_debugfs_info *files, int count,
			     struct x2_cnn_dev *dev)
{
	struct list_head *pos, *q;
	struct cnn_info_node *tmp;
	int i;

	mutex_lock(&dev->debugfs_lock);
	for (i = 0; i < count; i++) {
		list_for_each_safe(pos, q, &dev->debugfs_list) {
			tmp = list_entry(pos, struct cnn_info_node, list);
			if (tmp->info_ent == &files[i]) {
				debugfs_remove(tmp->dent);
				list_del(pos);
				kfree(tmp);
			}
		}
	}
	mutex_unlock(&dev->debugfs_lock);
	return 0;
}

int cnn_debugfs_cleanup(struct x2_cnn_dev *dev)
{

	if (!dev->debugfs_root)
		return 0;
	cnn_debugfs_remove_files(cnn_debugfs_list, CNN_DEBUGFS_ENTRIES, dev);

	debugfs_remove_recursive(dev->debugfs_root);
	dev->debugfs_root = NULL;

	return 0;
}


int cnn_debugfs_create_files(const struct cnn_debugfs_info *files, int count,
			     struct dentry *root, struct x2_cnn_dev *dev)
{
	struct dentry *ent;
	struct cnn_info_node *tmp;
	int i, ret;

	for (i = 0; i < count; i++) {
		tmp = kmalloc(sizeof(struct cnn_info_node), GFP_KERNEL);
		if (tmp == NULL) {
			ret = -1;
			goto fail;
		}
		ent = debugfs_create_file(files[i].name, S_IFREG | S_IRUGO,
			root, tmp, &cnn_debugfs_fops);
		if (!ent) {
			pr_err("Cannot create /sys/kernel/debug/dri/%pd/%s\n",
				root, files[i].name);
			kfree(tmp);
			ret = -1;
			goto fail;
		}

		tmp->cnn_dev = dev;
		tmp->dent = ent;
		tmp->info_ent = &files[i];

		mutex_lock(&dev->debugfs_lock);
		list_add(&tmp->list, &dev->debugfs_list);
		mutex_unlock(&dev->debugfs_lock);
	}
	return 0;

fail:
	cnn_debugfs_remove_files(files, count, dev);
	return ret;

}

int cnn_debugfs_init(struct x2_cnn_dev *dev, int cnn_id, struct dentry *root)
{
	char name[8];
	int ret;

	INIT_LIST_HEAD(&dev->debugfs_list);
	mutex_init(&dev->debugfs_lock);
	snprintf(name, 5, "%s%d", g_chrdev_name, cnn_id);
	dev->debugfs_root = debugfs_create_dir(name, root);
	if (!dev->debugfs_root) {
		pr_err("Cannot create /sys/kernel/debug/%s\n", name);
		return -1;
	}

	ret = cnn_debugfs_create_files(cnn_debugfs_list, CNN_DEBUGFS_ENTRIES,
				       dev->debugfs_root, dev);
	if (ret) {
		debugfs_remove(dev->debugfs_root);
		dev->debugfs_root = NULL;
		pr_err("Failed to create core drm debugfs files\n");
		return ret;
	}

	return 0;
}


int x2_cnn_probe(struct platform_device *pdev)
{
	int rc;
	struct x2_cnn_dev *cnn_dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	struct device_node *mem_np = NULL;
	struct resource mem_reserved;
	int cnn_id;
	char dev_name[8];

	cnn_dev = devm_kzalloc(&pdev->dev, sizeof(*cnn_dev), GFP_KERNEL);
	if (!cnn_dev)
		return -ENOMEM;
	cnn_dev->dev = &pdev->dev;

	cnn_dev->irq = irq_of_parse_and_map(np, 0);
	if (cnn_dev->irq < 0) {
		dev_err(&pdev->dev, "no cnn irq found\n");
		rc = -1;
		goto err_out;
	}

	/* get cnn controller base addr */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		rc = -ENODEV;
		goto err_out;
	}
	cnn_dev->cnn_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(cnn_dev->cnn_base)) {
		rc = PTR_ERR(cnn_dev->cnn_base);
		pr_err("%s:%d err_out get cnn_base failed\n",
				__func__, __LINE__);
		goto err_out;
	}

	rc = of_property_read_u32(np, "cnn-id", &cnn_id);
	if (rc < 0) {
		dev_err(cnn_dev->dev, "missing cnn-id property\n");
		goto err_out;
	}

	if ((cnn_id != 0) && (cnn_id != 1)) {
		pr_err("Invalid cnn id\n");
		rc = -1;
		goto err_out;
	}

	rc = x2_cnn_get_resets(cnn_dev, cnn_id);
	if (rc < 0) {
		pr_err("failed get cnn%d resets\n", cnn_id);
		goto err_out;
	}

	rc = request_irq(cnn_dev->irq, x2_cnn_interrupt_handler, 0,
			X2_CNN_DRV_NAME, cnn_dev);
	if (rc) {
		dev_err(cnn_dev->dev, "request_irq '%d' failed with %d\n",
			cnn_dev->irq, rc);
		goto err_out;
	}
	cnn_dev->irq_triggered = 0;

	init_waitqueue_head(&cnn_dev->cnn_int_wait);

	/* request memory address */
	mem_np = of_parse_phandle(np, "memory-region", 0);
	if (!mem_np) {
		/*FIXME: may can use ion to alloc this space */
		dev_err(cnn_dev->dev,
			"No %s specified\n", "memory-region");
		goto err_out;
	}
	rc = of_address_to_resource(mem_np, 0, &mem_reserved);
	if (rc) {
		dev_err(cnn_dev->dev,
			"No memory address assigned to the region\n");
		goto err_out;
	}

	if (resource_size(&mem_reserved)
			< (CNN_FC_GAP_LEN * 3 + CNN_FC_SPACE_LEN * 2)) {
		dev_err(&pdev->dev,
			"No enough memory for function call\n");
		rc = -1;
		goto err_out;
	}

	cnn_dev->fc_phys_base = mem_reserved.start + CNN_FC_GAP_LEN
		+ (CNN_FC_SPACE_LEN + CNN_FC_GAP_LEN) * cnn_id;
	cnn_dev->fc_mem_size  = CNN_FC_SPACE_LEN;
	cnn_dev->fc_base = cnn_ram_vmap(cnn_dev->fc_phys_base,
				cnn_dev->fc_mem_size, CNN_MT_UC);

	x2_cnn_reg_write(cnn_dev,
			X2_CNN_FC_LEN, (cnn_dev->fc_mem_size / 64) - 1);
	pr_info("Cnn fc phy base = 0x%x, len = 0x%x, default fc len = 0x%x\n",
			cnn_dev->fc_phys_base,
			cnn_dev->fc_mem_size,
			(cnn_dev->fc_mem_size / 64) - 1);

	x2_cnn_reg_write(cnn_dev, X2_CNN_FC_BASE,
			cnn_dev->fc_phys_base);

	x2_cnn_fc_gap_mem_init(cnn_dev);

	mutex_init(&cnn_dev->cnn_lock);
	spin_lock_init(&cnn_dev->cnn_spin_lock);

	/* Create the chardev for cnn0 and cnn1 */
	cnn_dev->chrdev_name = dev_name;
	snprintf(cnn_dev->chrdev_name, sizeof(dev_name),
			"%s%d", g_chrdev_name, cnn_id);
	cnn_dev->minor_num = MINOR_NUMBER;
	cnn_dev->num_devices = NUM_DEVICES;
	rc = x2_cnn_init_chrdev(cnn_dev);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"Failed create char dev for cnn%d\n", cnn_id);
		goto err_out;
	}

	/* Initialize the tasklet */
	tasklet_init(&cnn_dev->tasklet, x2_cnn_do_tasklet,
			(unsigned long)cnn_dev);
	if (cnn_id == 0) {
		rc = cnn_debugfs_init(cnn_dev, cnn_id, cnn0_debugfs_root);
		if (rc)
			pr_err("init cnn%d debugfs failed\n", cnn_id);
	} else if (cnn_id == 1) {
		rc = cnn_debugfs_init(cnn_dev, cnn_id, cnn1_debugfs_root);
		if (rc)
			pr_err("init cnn%d debugfs failed\n", cnn_id);
	}

	platform_set_drvdata(pdev, cnn_dev);

	x2_cnn_reg_write(cnn_dev, X2_CNNINT_MASK, 0x0);
	pr_info("x2 cnn%d probe OK!\n", cnn_id);
	return rc;

err_out:
	devm_kfree(&pdev->dev, cnn_dev);
	return rc;
}

static int x2_cnn_remove(struct platform_device *pdev)
{
	u32 cnn_int_mask;
	struct x2_cnn_dev *dev = platform_get_drvdata(pdev);

	cnn_int_mask = x2_cnn_reg_read(dev, X2_CNNINT_MASK);
	cnn_int_mask |= X2_CNN_PE0_INT_MASK(1);
	x2_cnn_reg_write(dev, X2_CNNINT_MASK, cnn_int_mask);


	tasklet_kill(&dev->tasklet);
	vm_unmap_ram(dev->fc_base, dev->fc_mem_size / PAGE_SIZE);
	dev->cnn_base = 0;
	cnn_debugfs_cleanup(dev);
	return 0;
}

static const struct of_device_id x2_cnn_of_match[] = {
	{ .compatible = "hobot,x2-cnn-host",},
	{}
};

static struct platform_driver x2_cnn_platform_driver = {
	.probe	 = x2_cnn_probe,
	.remove  = x2_cnn_remove,
	.driver  = {
		.name = X2_CNN_DRV_NAME,
		.of_match_table = x2_cnn_of_match,
	},
};
static int __init x2_cnn_init(void)
{
	int retval = 0;

	/* Register the platform driver */
	retval = platform_driver_register(&x2_cnn_platform_driver);
	if (retval)
		pr_err("x2 cnn driver register failed\n");
	return retval;

}

static void __exit x2_cnn_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&x2_cnn_platform_driver);
}

module_init(x2_cnn_init);
module_exit(x2_cnn_exit);

MODULE_DESCRIPTION("Driver for X2 CNN");
MODULE_AUTHOR("Hobot Inc.");
MODULE_LICENSE("GPL");
