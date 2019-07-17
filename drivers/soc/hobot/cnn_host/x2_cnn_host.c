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
#include <linux/time.h>
#include <linux/kfifo.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HOBOT_CNN_DEVFREQ
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/pm_opp.h>
#endif
#include "x2_cnn_host.h"

#define NETLINK_BPU 24
#define MOD_FRQ_DONE 0X0
#define MOD_FRQ      0X01
extern struct sock *cnn_netlink_init(int unit);
extern int cnn_netlink_send(struct sock *sock, int group, void *msg, int len);
struct sock *cnn_nl_sk;
#define FC_TIME_CNT 53

static DEFINE_MUTEX(x2_cnn_mutex);
static char *g_chrdev_name = "cnn";
static struct dentry *cnn0_debugfs_root;
static struct dentry *cnn1_debugfs_root;
static struct x2_cnn_dev *cnn0_dev;
static struct x2_cnn_dev *cnn1_dev;
static int profiler_frequency;
static int profiler_enable;
static int fc_time_enable;
static int ratio0;
static int ratio1;
static int queue0;
static int queue1;
static int bpu0_clk;
static int bpu1_clk;
static int bpu0_power;
static int bpu1_power;
static struct timer_list check_timer;
static struct mutex enable_lock;

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
	fc_tail_flag = fc_tail_idx & X2_CNN_FC_IDX_FLAG;
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
		cnn_dev->cnn_rst =
			devm_reset_control_get(cnn_dev->dev, "cnn0_rst");
		if (IS_ERR(cnn_dev->cnn_rst)) {
			name = "cnn0_rst";
			rst_temp = cnn_dev->cnn_rst;
			goto error;
		}
	} else if (cnn_id == 1) {
		cnn_dev->cnn_rst =
			devm_reset_control_get(cnn_dev->dev, "cnn1_rst");
		if (IS_ERR(cnn_dev->cnn_rst)) {
			name = "cnn1_rst";
			rst_temp = cnn_dev->cnn_rst;
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
	x2_cnn_reset_assert(cnn_dev->cnn_rst);
	udelay(1);
	x2_cnn_reset_release(cnn_dev->cnn_rst);

	x2_cnn_hw_init(cnn_dev);
	x2_cnn_set_fc_base(cnn_dev);
	x2_cnn_set_default_fc_depth(cnn_dev, 1023);
	return ret;
}
static void lock_bpu(struct x2_cnn_dev *dev)
{
	int zero_flag = 0;

	mutex_lock(&dev->cnn_lock);
	if (dev->zero_int_cnt > 0) {
		zero_flag = dev->zero_int_cnt;
		dev->wait_nega_flag = 1;
	}
	mutex_unlock(&dev->cnn_lock);
	if (zero_flag)
		wait_for_completion(&dev->nega_completion);

	mutex_lock(&dev->cnn_lock);
	if ((atomic_read(&dev->wait_fc_cnt) > 0)) {
		atomic_set(&dev->hw_flg, MOD_FRQ);
		/*wait bpu idle*/
		wait_for_completion(&dev->bpu_completion);
	}
}

static void unlock_bpu(struct x2_cnn_dev *dev)
{

	atomic_set(&dev->hw_flg, MOD_FRQ_DONE);
	mutex_unlock(&dev->cnn_lock);
}

static irqreturn_t x2_cnn_interrupt_handler(int irq, void *dev_id)
{
	int ret;
	struct x2_cnn_dev *dev = (struct x2_cnn_dev *)dev_id;
	unsigned long flags;
	u32 irq_status;
	u32 tmp_irq;
	struct x2_int_info tmp;

	spin_lock_irqsave(&dev->cnn_spin_lock, flags);

	irq_status = x2_cnn_reg_read(dev, X2_CNNINT_STATUS);
	x2_cnn_reg_write(dev, X2_CNNINT_MASK, 0x1);
	tmp_irq = x2_cnn_reg_read(dev, X2_CNNINT_NUM);

	x2_cnn_reg_write(dev, X2_CNNINT_MASK, 0x0);
	spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
	do {
		ret = kfifo_out(&dev->int_info_fifo, &tmp,
						sizeof(struct x2_int_info));
		if (!ret) {
			spin_lock_irqsave(&dev->cnn_spin_lock, flags);
			dev->cnn_int_num.cnn_int_num[dev->cnn_int_num.cnn_int_count] = tmp_irq;
			dev->cnn_int_num.cnn_int_interval[dev->cnn_int_num.cnn_int_count] = 0;
			if (dev->cnn_int_num.cnn_int_count < CNN_INT_NUM)
				dev->cnn_int_num.cnn_int_count++;
			spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
			break;
		}

		spin_lock_irqsave(&dev->cnn_spin_lock, flags);
		dev->cnn_int_num.cnn_int_num[dev->cnn_int_num.cnn_int_count] = tmp.int_num;
		do_gettimeofday(&tmp.end_time);
		dev->cnn_int_num.cnn_int_interval[dev->cnn_int_num.cnn_int_count] =
			(tmp.end_time.tv_sec * 1000000 + tmp.end_time.tv_usec)
			- (tmp.start_time.tv_sec * 1000000 + tmp.start_time.tv_usec);
		if (dev->cnn_int_num.cnn_int_count < CNN_INT_NUM - 1) {
			dev->cnn_int_num.cnn_int_count++;
			dev->real_int_cnt++;
		} else {
			spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
			break;
		}

		spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
		atomic_sub(tmp.fc_total, &dev->wait_fc_cnt);

	} while (tmp.int_num != tmp_irq);

	pr_debug("!!!!!!xxxx x2 cnn interrupt\n");
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

static void x2_cnn_set_fc_start_time(struct x2_cnn_dev *dev, int int_num,
				     int count)
{
	int ret;
	struct x2_int_info x2_int;

	count &= X2_CNN_MAX_FC_LEN_MASK;
	if (fc_time_enable) {
		dev->fc_time[dev->time_tail].fc_count = count;
		dev->fc_time[dev->time_tail].int_num = int_num;
	}
	if (atomic_read(&dev->wait_fc_cnt) > 0) {
		if (fc_time_enable) {
			spin_lock_bh(&dev->set_time_lock);
			dev->fc_time[dev->time_tail].time_flag = 1;
			spin_unlock_bh(&dev->set_time_lock);
		}
		/*bpu busy,check int_num*/
		if (int_num) {
			if (fc_time_enable) {
				dev->time_tail++;
				dev->time_tail %= FC_TIME_CNT;
			}
			if ((dev->zero_int_start_time.tv_sec == 0)
					&& (dev->zero_int_start_time.tv_usec == 0)) {
				do_gettimeofday(&x2_int.start_time);
			} else {
				x2_int.start_time = dev->zero_int_start_time;
				dev->zero_int_start_time.tv_sec = 0;
				dev->zero_int_start_time.tv_usec = 0;
			}
			x2_int.fc_total = dev->zero_int_cnt + 1;
			x2_int.int_num = int_num;
			ret = kfifo_in(&dev->int_info_fifo, &x2_int,
				       sizeof(struct x2_int_info));
			if (ret < sizeof(struct x2_int_info))
				pr_err("%s[%d]:x2 interrupt info fifo no space\n",
					__func__, __LINE__);
			dev->zero_int_cnt = 0;
			if (dev->wait_nega_flag) {
				complete(&dev->nega_completion);
				dev->wait_nega_flag = 0;
			}

		} else {
			if (!dev->zero_int_cnt) {
				do_gettimeofday(&dev->zero_int_start_time);
			}
			dev->zero_int_cnt++;
		}
	} else {
		if (fc_time_enable) {
			spin_lock_bh(&dev->set_time_lock);
			do_gettimeofday(&dev->fc_time[dev->time_tail].start_time);
			spin_unlock_bh(&dev->set_time_lock);
		}

		if (int_num) {
			if (fc_time_enable) {
				dev->time_tail++;
				dev->time_tail %= FC_TIME_CNT;
			}
			if ((dev->zero_int_start_time.tv_sec == 0)
					&& (dev->zero_int_start_time.tv_usec == 0)) {
				do_gettimeofday(&x2_int.start_time);
			} else {
				x2_int.start_time = dev->zero_int_start_time;
				dev->zero_int_start_time.tv_sec = 0;
				dev->zero_int_start_time.tv_usec = 0;
			}
			x2_int.fc_total = dev->zero_int_cnt + 1;
			x2_int.int_num = int_num;
			dev->zero_int_cnt = 0;
			if (dev->wait_nega_flag) {
				complete(&dev->nega_completion);
				dev->wait_nega_flag = 0;
			}
			ret = kfifo_in(&dev->int_info_fifo, &x2_int,
					sizeof(struct x2_int_info));
			if (ret < sizeof(struct x2_int_info))
				pr_err("%s[%d]:x2 interrupt info fifo no space\n",
					__func__, __LINE__);

		} else {
			if (!dev->zero_int_cnt) {
				do_gettimeofday(&dev->zero_int_start_time);
			}
			dev->zero_int_cnt++;
		}
	}
	atomic_inc(&dev->wait_fc_cnt);
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
	fc_tail_flag = fc_tail_idx & X2_CNN_FC_IDX_FLAG;
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
	fc_tail_flag = fc_tail_idx & X2_CNN_FC_IDX_FLAG;

	fc_head_idx &= X2_CNN_MAX_FC_LEN_MASK;
	fc_tail_idx &= X2_CNN_MAX_FC_LEN_MASK;

	if (fc_head_flag != fc_tail_flag)
		free_fc_fifo = fc_head_idx - fc_tail_idx;
	else
		free_fc_fifo = fc_depth - fc_tail_idx + fc_head_idx + 1;

	pr_debug("fc_depth:0x%x, get fifo spaces return val:%d, head:[%d], tail:[%d]\n",
			fc_depth, free_fc_fifo, fc_head_idx, fc_tail_idx);
	return free_fc_fifo;
}
/**
 * x2_cnn_clock_up - start cnn
 * @dev: pointer to struct x2_cnn_dev
 */
static void x2_cnn_clock_up(struct x2_cnn_dev *dev)
{
	int ret;
	unsigned int tmp;

	lock_bpu(dev);
	pr_info("%s\n", __func__);
	tmp = readl(dev->cnn_pmu);

	tmp &= ~(1 << dev->iso_bit);
	writel(tmp, dev->cnn_pmu);
	udelay(5);

	x2_cnn_reset_release(dev->cnn_rst);
	if (!__clk_is_enabled(dev->cnn_aclk))
		clk_enable(dev->cnn_aclk);
	if (!__clk_is_enabled(dev->cnn_mclk))
		clk_enable(dev->cnn_mclk);
	x2_cnn_hw_init(dev);
	x2_cnn_set_fc_base(dev);
	x2_cnn_set_default_fc_depth(dev, 1023);
	unlock_bpu(dev);
}
/**
 * x2_cnn_clock_down - stop cnn
 * @dev: pointer to struct x2_cnn_dev
 */
static void x2_cnn_clock_down(struct x2_cnn_dev *dev)
{
	unsigned int tmp;

	lock_bpu(dev);
	pr_info("%s\n", __func__);
	if (__clk_is_enabled(dev->cnn_aclk))
		clk_disable(dev->cnn_aclk);
	if (__clk_is_enabled(dev->cnn_mclk))
		clk_disable(dev->cnn_mclk);
	tmp = readl(dev->cnn_pmu);

	tmp |= (1 << dev->iso_bit);
	writel(tmp, dev->cnn_pmu);
	udelay(5);

	x2_cnn_reset_assert(dev->cnn_rst);
	unlock_bpu(dev);
}

/**
 * x2_cnn_power_up - start cnn
 * @dev: pointer to struct x2_cnn_dev
 */
static void x2_cnn_power_up(struct x2_cnn_dev *dev)
{
	int ret;
	unsigned int tmp;

	lock_bpu(dev);
	pr_info("%s\n", __func__);
	ret = regulator_enable(dev->cnn_regulator);
	if (ret != 0)
		dev_err(dev->dev, "regulator enable error\n");
	tmp = readl(dev->cnn_pmu);

	tmp &= ~(1 << dev->iso_bit);
	writel(tmp, dev->cnn_pmu);
	udelay(5);

	x2_cnn_reset_release(dev->cnn_rst);
	if (!__clk_is_enabled(dev->cnn_aclk))
		clk_enable(dev->cnn_aclk);
	if (!__clk_is_enabled(dev->cnn_mclk))
		clk_enable(dev->cnn_mclk);
	x2_cnn_hw_init(dev);
	x2_cnn_set_fc_base(dev);
	x2_cnn_set_default_fc_depth(dev, 1023);
	unlock_bpu(dev);
}
/**
 * x2_cnn_power_down - stop cnn
 * @dev: pointer to struct x2_cnn_dev
 */
static void x2_cnn_power_down(struct x2_cnn_dev *dev)
{
	unsigned int tmp;
	pr_info("%s\n", __func__);

	lock_bpu(dev);
	if (__clk_is_enabled(dev->cnn_aclk))
		clk_disable(dev->cnn_aclk);
	if (__clk_is_enabled(dev->cnn_mclk))
		clk_disable(dev->cnn_mclk);
	tmp = readl(dev->cnn_pmu);

	tmp |= (1 << dev->iso_bit);
	writel(tmp, dev->cnn_pmu);
	udelay(5);

	x2_cnn_reset_assert(dev->cnn_rst);
	regulator_disable(dev->cnn_regulator);
	unlock_bpu(dev);
}
/**
 * x2_cnn_fc_fifo_enqueue - fill the function call into the reserved memory
 * @dev: pointer to struct x2_cnn_dev
 * fc_buf: function call buf struct
 *
 * Return: 0 success, others failed
 */
static int x2_cnn_fc_fifo_enqueue(struct x2_cnn_dev *dev,
		struct x2_cnn_fc_info *fc_buf)
{
	u32 rc = 0;
	u32 i;
	u32 free_fc_fifo = 0;
	u32 fc_head_idx, fc_tail_idx,
	    fc_head_flag, fc_tail_flag;
	u32 fc_depth, insert_fc_cnt, residue_fc_cnt;
	u32 count;
	struct hbrt_x2_funccall_s *tmp_ptr = NULL;

	if (!regulator_is_enabled(dev->cnn_regulator) ||
	    !__clk_is_enabled(dev->cnn_aclk) ||
	    !__clk_is_enabled(dev->cnn_mclk))
		return -1;
	fc_depth = x2_cnn_reg_read(dev, X2_CNN_FC_LEN);
	fc_head_idx = x2_cnn_reg_read(dev, X2_CNN_FC_HEAD);
	fc_head_flag = fc_head_idx & X2_CNN_FC_IDX_FLAG;

	fc_tail_idx = x2_cnn_reg_read(dev, X2_CNN_FC_TAIL);
	fc_tail_flag = fc_tail_idx & X2_CNN_FC_IDX_FLAG;
	tmp_ptr = (struct hbrt_x2_funccall_s *)fc_buf->fc_info;

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
	count = fc_tail_idx;
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

		for (i = 0; i < insert_fc_cnt; i++) {
			x2_cnn_set_fc_start_time(dev,
						 tmp_ptr->interrupt_num,
						 count);
			tmp_ptr++;
			count++;
		}
		x2_cnn_set_fc_tail_idx(dev, residue_fc_cnt | fc_tail_flag);
	} else {
		memcpy(dev->fc_base + (fc_tail_idx * X2_CNN_FC_SIZE),
			fc_buf->fc_info, fc_buf->fc_cnt * X2_CNN_FC_SIZE);

		for (i = 0; i < fc_buf->fc_cnt; i++) {
			x2_cnn_set_fc_start_time(dev,
						 tmp_ptr->interrupt_num,
						 count);
			tmp_ptr++;
			count++;
		}
		x2_cnn_set_fc_tail_idx(dev,
				fc_tail_flag | (fc_tail_idx + fc_buf->fc_cnt));
	}

#ifdef CNN_DEBUG
	dump_fc_fifo_status(dev);
#endif
	return rc;
}

static struct x2_cnn_int_num x2_cnn_get_int_num(struct x2_cnn_dev *dev)
{
	return dev->cnn_int_num;
}

static int x2_cnn_open(struct inode *inode, struct file *filp)
{
	int rc = 0;
	struct x2_cnn_dev *devdata;


	mutex_lock(&x2_cnn_mutex);

	devdata = container_of(inode->i_cdev, struct x2_cnn_dev, i_cdev);
	filp->private_data = devdata;
	if (!regulator_is_enabled(devdata->cnn_regulator) ||
	    !__clk_is_enabled(devdata->cnn_aclk) ||
	    !__clk_is_enabled(devdata->cnn_mclk)) {
		mutex_unlock(&x2_cnn_mutex);

		return -1;
	}

	mutex_unlock(&x2_cnn_mutex);

	x2_cnn_reg_read(devdata, X2_CNN_FC_LEN);

	return rc;
}

static int x2_cnn_release(struct inode *inode, struct file *filp)
{
	struct x2_cnn_dev *devdata;

	mutex_lock(&x2_cnn_mutex);
	devdata = filp->private_data;
	devdata->cnn_int_num.cnn_int_count = 0;
	atomic_set(&devdata->wait_fc_cnt, 0);
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
	struct hbrt_x2_funccall_s *tmp_ptr = NULL;
	unsigned long flags;

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
			kfree(kernel_fc_data);
			return -EFAULT;
		}
		data.fc_data.fc_info = kernel_fc_data;

		tmp_ptr = (struct hbrt_x2_funccall_s *)data.fc_data.fc_info;

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
		spin_lock_irqsave(&dev->cnn_spin_lock, flags);
		data.int_num_data = x2_cnn_get_int_num(dev);
		dev->cnn_int_num.cnn_int_count = 0;
		spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
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
	unsigned long flags;

	struct x2_cnn_dev *dev = filp->private_data;

	poll_wait(filp, &dev->cnn_int_wait, wait);
	spin_lock_irqsave(&dev->cnn_spin_lock, flags);
	if (dev->irq_triggered) {
		mask |= POLLIN | POLLRDNORM;
		dev->irq_triggered = 0;
	}
	spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);

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
	unsigned long flags;
	int ret;
	int int_cnt;

	spin_lock_irqsave(&dev->cnn_spin_lock, flags);
	int_cnt = dev->real_int_cnt;
	dev->real_int_cnt = 0;
	spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);

	spin_lock_irqsave(&dev->cnn_spin_lock, flags);
	wake_up(&dev->cnn_int_wait);
	dev->irq_triggered = 1;
	spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
	ret = cnn_netlink_send(dev->irq_sk, dev->core_index,
			&dev->cnn_int_num, sizeof(dev->cnn_int_num));
	if ((ret < 0) && (ret != -ESRCH)) {
		pr_err("CNN trigger irq[%d] failed errno[%d]!\n",
				dev->cnn_int_num.cnn_int_num[0], ret);
	} else if (ret == sizeof(dev->cnn_int_num)) {
		spin_lock_irqsave(&dev->cnn_spin_lock, flags);
		dev->cnn_int_num.cnn_int_count = 0;
		spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
	}
	if (fc_time_enable) {
		int head_tmp;

		do {
			spin_lock(&dev->set_time_lock);
			do_gettimeofday(&dev->fc_time[dev->time_head].end_time);
			head_tmp = dev->time_head + 1;
			head_tmp %= FC_TIME_CNT;

				if (dev->fc_time[head_tmp].time_flag == 1) {

					dev->fc_time[head_tmp].start_time.tv_sec =
						dev->fc_time[dev->time_head].end_time.tv_sec;
					dev->fc_time[head_tmp].start_time.tv_usec =
						dev->fc_time[dev->time_head].end_time.tv_usec;

					dev->fc_time[head_tmp].time_flag = 0;
				}
			dev->time_head++;
			dev->time_head %= FC_TIME_CNT;
			spin_unlock(&dev->set_time_lock);
		} while (--int_cnt);
	}
	if (atomic_read(&dev->hw_flg)) {
		if (atomic_read(&dev->wait_fc_cnt) == 0)
			complete(&dev->bpu_completion);
	}
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

#ifdef CONFIG_HOBOT_CNN_DEVFREQ
static int cnnfreq_target(struct device *dev, unsigned long *freq,
				   u32 flags)
{
	struct x2_cnn_dev *cnn_dev = dev_get_drvdata(dev);
	struct x2_cnnfreq *cnnfreq = cnn_dev->cnnfreq;
	struct dev_pm_opp *opp;
	unsigned long old_clk_rate = cnnfreq->rate;
	unsigned long rate, target_volt, target_rate;
	int err;

	lock_bpu(cnn_dev);

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp)) {
		err = PTR_ERR(opp);
		goto out;
	}
	rate = dev_pm_opp_get_freq(opp);
	target_volt = dev_pm_opp_get_voltage(opp);

	target_rate = clk_round_rate(cnn_dev->cnn_mclk, rate);
	if ((long)target_rate <= 0)
		target_rate = rate;

	if (cnnfreq->rate == target_rate) {
		if (cnnfreq->volt == target_volt) {
			err = 0;
			goto out;
		}
		err = regulator_set_voltage(cnn_dev->cnn_regulator,
					target_volt,INT_MAX);
		if (err) {
			dev_err(dev, "Cannot set voltage %lu uV\n",
				target_volt);
			goto out;
		}
	}

	if (old_clk_rate < target_rate) {
		err = regulator_set_voltage(cnn_dev->cnn_regulator,
					target_volt, INT_MAX);
		if (err) {
			dev_err(dev, "Cannot set voltage %lu uV\n",
				target_volt);
			goto out;
		}
	}

	err = clk_set_rate(cnn_dev->cnn_mclk, target_rate);
	if (err) {
		dev_err(dev, "Cannot set frequency %lu (%d)\n",
			target_rate, err);
		regulator_set_voltage(cnn_dev->cnn_regulator,
				cnnfreq->volt, INT_MAX);
		goto out;
	}

	cnnfreq->rate = clk_get_rate(cnn_dev->cnn_mclk);

	if (cnnfreq->rate != target_rate) {
		dev_err(dev, "Get wrong frequency, Request %lu, Current %lu\n",
			target_rate, cnnfreq->rate);
		regulator_set_voltage(cnn_dev->cnn_regulator,
				cnnfreq->volt, INT_MAX);
		goto out;
	} else if (old_clk_rate > target_rate) {
		err = regulator_set_voltage(cnn_dev->cnn_regulator,
				target_volt,INT_MAX);
		if (err) {
			dev_err(dev, "Cannot set vol %lu uV\n", target_volt);
			goto out;
		}
	}

	cnnfreq->volt = target_volt;
out:
	unlock_bpu(cnn_dev);
	return err;
}

static int cnnfreq_get_cur_freq(struct device *dev,
					 unsigned long *freq)
{
	struct x2_cnn_dev *cnn_dev = dev_get_drvdata(dev);

	*freq = cnn_dev->cnnfreq->rate;

	return 0;
}

static struct devfreq_dev_profile hobot_cnnfreq_profile = {
	.polling_ms	= 0,
	.target		= cnnfreq_target,
	.get_cur_freq	= cnnfreq_get_cur_freq,
};

static int cnnfreq_init_freq_table(struct device *dev,
					    struct devfreq_dev_profile *devp)
{
	int count;
	int i = 0;
	unsigned long freq = 0;
	struct dev_pm_opp *opp;

	count = dev_pm_opp_get_opp_count(dev);
	if (count < 0) {
		return count;
	}

	devp->freq_table = kmalloc_array(count, sizeof(devp->freq_table[0]),
				GFP_KERNEL);
	if (!devp->freq_table)
		return -ENOMEM;

	for (i = 0; i < count; i++, freq++) {
		opp = dev_pm_opp_find_freq_ceil(dev, &freq);
		if (IS_ERR(opp))
			break;

		devp->freq_table[i] = freq;
	}

	if (count != i)
		dev_warn(dev, "Unable to enumerate all OPPs (%d!=%d)\n",
			 count, i);

	devp->max_state = i;
	return 0;
}

static int x2_cnnfreq_register(struct x2_cnn_dev *cnn_dev)
{
	struct device *dev = cnn_dev->dev;
	struct x2_cnnfreq *data;
	struct devfreq_dev_profile *devp = &hobot_cnnfreq_profile;
	const char *gov_name;

	dev_err(dev, "%s probe\n", __func__);

	data = devm_kzalloc(dev, sizeof(struct x2_cnnfreq), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (dev_pm_opp_of_add_table(dev)) {
		dev_err(dev, "Invalid operating-points in device tree.\n");
		return -EINVAL;
	}

	if (cnnfreq_init_freq_table(dev, devp))
		return -EFAULT;

	data->rate = clk_get_rate(cnn_dev->cnn_mclk);
	data->volt = regulator_get_voltage(cnn_dev->cnn_regulator);

	devp->initial_freq = data->rate;
	data->min = devp->freq_table[0];
	data->max = devp->freq_table[devp->max_state ? devp->max_state - 1 : 0];

	cnn_dev->cnnfreq = data;

	if (of_property_read_string(dev->of_node, "governor", &gov_name))
		gov_name = "performance";

	data->devfreq = devm_devfreq_add_device(dev, devp,
						gov_name, NULL);
	if (IS_ERR(data->devfreq))
		return PTR_ERR(data->devfreq);

	data->devfreq->min_freq = data->min;
	data->devfreq->max_freq = data->max;
	devm_devfreq_register_opp_notifier(dev, data->devfreq);

	of_devfreq_cooling_register(dev->of_node, data->devfreq);
	dev_err(dev, "%s end\n", __func__);
	return 0;
}
#endif

int x2_cnn_probe(struct platform_device *pdev)
{
	int rc;
	struct x2_cnn_dev *cnn_dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	struct resource *pmu;
	struct device_node *mem_np = NULL;
	struct resource mem_reserved;
	int cnn_id;
	char dev_name[8];

	cnn_dev = devm_kzalloc(&pdev->dev, sizeof(*cnn_dev), GFP_KERNEL);
	if (!cnn_dev)
		return -ENOMEM;
	cnn_dev->dev = &pdev->dev;
	cnn_dev->fc_time = devm_kzalloc(&pdev->dev,
					sizeof(struct x2_fc_time) * FC_TIME_CNT,
					GFP_KERNEL);
	if (!cnn_dev->fc_time)
		return -ENOMEM;
	cnn_dev->time_head = 0;
	cnn_dev->time_tail = 0;
	memset(cnn_dev->fc_time, 0, sizeof(struct x2_fc_time) * FC_TIME_CNT);
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

	if (cnn_id == 0) {
		rc = cnn_debugfs_init(cnn_dev, cnn_id, cnn0_debugfs_root);
		if (rc)
			pr_err("init cnn%d debugfs failed\n", cnn_id);
		cnn0_dev = cnn_dev;
		/*get regulator*/
		cnn_dev->cnn_regulator = regulator_get(cnn_dev->dev, "cnn0");
		if (cnn_dev->cnn_regulator == NULL)
			pr_info("get regu err\n");
		if (IS_ERR(cnn_dev->cnn_regulator))
			pr_info("get err1\n");

		/*get cnn clock and prepare*/
		cnn_dev->cnn_aclk = devm_clk_get(cnn_dev->dev, "cnn0_aclk");
		if (IS_ERR(cnn_dev->cnn_aclk))
			pr_info("get cnn0 aclock err\n");
		cnn_dev->cnn_mclk = devm_clk_get(cnn_dev->dev, "cnn0_mclk");
		if (IS_ERR(cnn_dev->cnn_mclk))
			pr_info("get cnn0 mclock err\n");
	} else if (cnn_id == 1) {
		rc = cnn_debugfs_init(cnn_dev, cnn_id, cnn1_debugfs_root);
		if (rc)
			pr_err("init cnn%d debugfs failed\n", cnn_id);
		cnn1_dev = cnn_dev;
		/*get regulator*/
		cnn_dev->cnn_regulator = regulator_get(cnn_dev->dev, "cnn1");
		if (cnn_dev->cnn_regulator == NULL)
			pr_info("get regu err\n");
		if (IS_ERR(cnn_dev))
			pr_info("get err1\n");
		/*get cnn clock and prepare*/
		cnn_dev->cnn_aclk = devm_clk_get(cnn_dev->dev, "cnn1_aclk");
		if (IS_ERR(cnn_dev->cnn_aclk))
			pr_info("get cnn1 aclock err\n");
		cnn_dev->cnn_mclk = devm_clk_get(cnn_dev->dev, "cnn1_mclk");
		if (IS_ERR(cnn_dev->cnn_mclk))
			pr_info("get cnn1 mclock err\n");
	}

	rc = regulator_enable(cnn_dev->cnn_regulator);
	if (rc != 0)
		dev_err(cnn_dev->dev, "regulator enalbe error\n");
	rc = clk_prepare_enable(cnn_dev->cnn_aclk);
	if (rc)
		pr_info("cnn aclock prepare error\n");
	rc = clk_prepare_enable(cnn_dev->cnn_mclk);
	if (rc)
		pr_info("cnn mclock prepare error\n");

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
	pmu = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pmu) {
		rc = -ENODEV;
		goto err_out;
	}
	cnn_dev->cnn_pmu = ioremap(pmu->start, 4);
	if (IS_ERR(cnn_dev->cnn_pmu)) {
		rc = PTR_ERR(cnn_dev->cnn_pmu);
		pr_err("%s:%d err_out get cnn_pmu failed\n",
				__func__, __LINE__);
		goto err_out;
	}
	rc = of_property_read_u32(np, "iso-bit", &cnn_dev->iso_bit);
	if (rc < 0) {
		dev_err(cnn_dev->dev, "missing iso-bit property\n");
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

	cnn_dev->core_index = cnn_id;
	cnn_dev->fc_phys_base = mem_reserved.start + CNN_FC_GAP_LEN
		+ (CNN_FC_SPACE_LEN + CNN_FC_GAP_LEN) * cnn_id;
	cnn_dev->fc_mem_size  = CNN_FC_SPACE_LEN;
	cnn_dev->fc_base = cnn_ram_vmap(cnn_dev->fc_phys_base,
				cnn_dev->fc_mem_size, CNN_MT_UC);
	atomic_set(&cnn_dev->wait_fc_cnt, 0);
	atomic_set(&cnn_dev->hw_flg, 0);
	cnn_dev->zero_int_cnt = 0;
	cnn_dev->real_int_cnt = 0;
	cnn_dev->wait_nega_flag = 0;
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
	spin_lock_init(&cnn_dev->set_time_lock);
	spin_lock_init(&cnn_dev->cnn_spin_lock);

	rc = kfifo_alloc(&cnn_dev->int_info_fifo,
		    sizeof(struct x2_int_info) * x2_cnn_reg_read(cnn_dev, X2_CNN_FC_LEN), GFP_KERNEL);
	if (rc < 0) {
		pr_err("kfifo alloc error\n");
		goto err_out;
	}
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

	if (!cnn_nl_sk) {
		cnn_nl_sk = cnn_netlink_init(NETLINK_BPU);
		if (!cnn_nl_sk) {
			pr_err("Fail init cnn%d irq netlink notifiy failed\n", cnn_id);
			goto err_out;
		}
	}

	cnn_dev->irq_sk = cnn_nl_sk;

	/* Initialize the tasklet */
	tasklet_init(&cnn_dev->tasklet, x2_cnn_do_tasklet,
			(unsigned long)cnn_dev);

	platform_set_drvdata(pdev, cnn_dev);

	/*
	 * init bpu completion and negative completion
	 * for lock_bpu
	 */
	init_completion(&cnn_dev->bpu_completion);
	init_completion(&cnn_dev->nega_completion);

	x2_cnn_reg_write(cnn_dev, X2_CNNINT_MASK, 0x0);
	pr_info("x2 cnn%d probe OK!!\n", cnn_id);
#ifdef CONFIG_HOBOT_CNN_DEVFREQ
	x2_cnnfreq_register(cnn_dev);
#endif
	return rc;

err_out:
	devm_kfree(&pdev->dev, cnn_dev);
	return rc;
}
void cnn_fc_time_kfifo_clean(struct x2_cnn_dev *dev)
{
	kfifo_free(&dev->int_info_fifo);
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
	regulator_put(dev->cnn_regulator);
	cnn_fc_time_kfifo_clean(dev);
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

static void x2_cnn_check_func(unsigned long arg)
{
	static int time;
	static int cnn0_busy;
	static int cnn1_busy;
	static int fre;
	static int fre_tmp;

	int cnn0_head = x2_cnn_reg_read(cnn0_dev, X2_CNN_FC_HEAD);
	int cnn0_tail = x2_cnn_reg_read(cnn0_dev, X2_CNN_FC_TAIL);

	int cnn1_head = x2_cnn_reg_read(cnn1_dev, X2_CNN_FC_HEAD);
	int cnn1_tail = x2_cnn_reg_read(cnn1_dev, X2_CNN_FC_TAIL);

	fre_tmp = profiler_frequency;
	if (fre != fre_tmp) {
		/* frequency changed*/
		fre = fre_tmp;
		cnn0_busy = 0;
		cnn1_busy = 0;
		time = 0;

	}
	if (cnn0_head != cnn0_tail || (atomic_read(&cnn0_dev->wait_fc_cnt) > 0))
		cnn0_busy++;
	if (cnn1_head != cnn1_tail || (atomic_read(&cnn1_dev->wait_fc_cnt) > 0))
		cnn1_busy++;

	if (++time == fre) {
		time = 0;
		ratio0 = cnn0_busy * 100 / fre;
		ratio1 = cnn1_busy * 100 / fre;
		cnn0_busy = 0;
		cnn1_busy = 0;
	}
	check_timer.expires = jiffies + HZ / fre;
	add_timer(&check_timer);
}
static ssize_t fre_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", profiler_frequency);
}


static ssize_t fre_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &profiler_frequency);
	if (ret < 0) {
		pr_info("%s sscanf error\n", __func__);
		return 0;
	}
	pr_info("func:%s, FRE:%d\n", __func__, profiler_frequency);
	if (profiler_frequency <= 0 || profiler_frequency > HZ) {
		pr_warn("err!, profiler frequency range is 0~%d HZ\n", HZ);
		return -1;
	}
	if (!profiler_enable)
		check_timer.expires = jiffies + HZ / profiler_frequency;
	else
		mod_timer(&check_timer, jiffies + HZ / profiler_frequency);
	return count;
}

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	return sprintf(buf, "%d\n", profiler_enable);
}

static ssize_t fc_time_enable_show(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	return sprintf(buf, "%d\n", fc_time_enable);
}

static ssize_t fc_time_enable_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;

	lock_bpu(cnn0_dev);
	lock_bpu(cnn1_dev);

	memset(cnn0_dev->fc_time, 0,
		   sizeof(struct x2_fc_time) * FC_TIME_CNT);
	memset(cnn1_dev->fc_time, 0,
		   sizeof(struct x2_fc_time) * FC_TIME_CNT);

	ret = sscanf(buf, "%du", &fc_time_enable);
	unlock_bpu(cnn0_dev);
	unlock_bpu(cnn1_dev);
	return count;
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
						  const char *buf, size_t count)
{
	int ret;
	static int enable_tmp;

	mutex_lock(&enable_lock);
	ret = sscanf(buf, "%du", &profiler_enable);
	mutex_unlock(&enable_lock);
	if (ret < 0) {
		pr_info("%s sscanf error\n", __func__);
		return 0;
	}
	pr_info("set bpu profiler enable:%d\n", profiler_enable);
	if (profiler_enable) {
		if (!profiler_frequency) {
			pr_warn("cnn profiler frequency is 0, please set frequency!\n");
			profiler_enable = 0;
			return -1;
		}
		mutex_lock(&enable_lock);
		if (!enable_tmp) {
			add_timer(&check_timer);
			enable_tmp = profiler_enable;
		}
		mutex_unlock(&enable_lock);
	} else {
		mutex_lock(&enable_lock);
		if (enable_tmp) {
			del_timer(&check_timer);
			enable_tmp = profiler_enable;
		}
		mutex_unlock(&enable_lock);
	}
	return count;
}


static ssize_t ratio0_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	return sprintf(buf, "%d\n", ratio0);
}

static ssize_t ratio1_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	return sprintf(buf, "%d\n", ratio1);
}
static ssize_t queue0_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	mutex_lock(&cnn0_dev->cnn_lock);
	queue0 = x2_cnn_get_fc_fifo_spaces(cnn0_dev);
	mutex_unlock(&cnn0_dev->cnn_lock);
	return sprintf(buf, "%d\n", queue0);
}
static ssize_t queue1_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	mutex_lock(&cnn1_dev->cnn_lock);
	queue1 = x2_cnn_get_fc_fifo_spaces(cnn0_dev);
	mutex_unlock(&cnn1_dev->cnn_lock);
	return sprintf(buf, "%d\n", queue1);
}

static ssize_t fc_time_show(struct x2_cnn_dev *dev, char *buf)
{
	int ret = 0;
	int sum = 0;
	int head, tail;
	unsigned long flags;
	struct x2_fc_time tmp[FC_TIME_CNT];
	int elapse_time = 0;
	char buf_start[24] = {0};
	char buf_end[24] = {0};

	spin_lock_irqsave(&dev->set_time_lock, flags);
	memcpy(tmp, dev->fc_time, sizeof(struct x2_fc_time) * FC_TIME_CNT);
	tail = dev->time_head;
	spin_unlock_irqrestore(&dev->set_time_lock, flags);

	if (tmp[tail].start_time.tv_sec == 0)
		head = 0;
	else {
		head = tail + 3;
		head %= FC_TIME_CNT;
	}

	ret = sprintf(buf, "%-6s%-17s\t%-17s\t%-17s\t%s\n",
					"num",
					"intterupt",
					"start time",
					"end_time",
					"exe_time");
	sum += ret;

	do {
		memset(buf_start, 0, 24);
		memset(buf_end, 0, 24);
		sprintf(buf_start, "%lds %ldus",
			tmp[head].start_time.tv_sec,
			tmp[head].start_time.tv_usec);
		sprintf(buf_end, "%lds %ldus",
			tmp[head].end_time.tv_sec,
			tmp[head].end_time.tv_usec);

		elapse_time = tmp[head].end_time.tv_sec * 1000 +
			      tmp[head].end_time.tv_usec / 1000 -
			      tmp[head].start_time.tv_sec * 1000 -
			      tmp[head].start_time.tv_usec / 1000;
		ret = sprintf(buf + sum, "%-6d%-17d\t%-17s\t%-17s\t%4dms\n",
				tmp[head].fc_count, tmp[head].int_num,
				buf_start,
				buf_end,
				elapse_time);
		sum += ret;
		head++;
		head %= FC_TIME_CNT;
	} while (head != tail);
	return sum;
}

static ssize_t bpu0_fc_time_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = fc_time_show(cnn0_dev, buf);
	return ret;
}
static ssize_t bpu1_fc_time_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = fc_time_show(cnn1_dev, buf);
	return ret;

}

static ssize_t bpu0_clock_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int ret_aclk = __clk_is_enabled(cnn0_dev->cnn_aclk);
	int ret_mclk = __clk_is_enabled(cnn0_dev->cnn_mclk);

	return sprintf(buf, "%d\n", (ret_aclk && ret_mclk));
}
static ssize_t bpu0_clock_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &bpu0_clk);
	if (bpu0_clk) {
		if (!__clk_is_enabled(cnn0_dev->cnn_aclk) ||
			!__clk_is_enabled(cnn0_dev->cnn_mclk))
			x2_cnn_clock_up(cnn0_dev);
	} else {
		if (__clk_is_enabled(cnn0_dev->cnn_aclk) ||
			__clk_is_enabled(cnn0_dev->cnn_mclk))
			x2_cnn_clock_down(cnn0_dev);
	}

	return count;

}

static ssize_t bpu1_clock_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int ret_aclk = __clk_is_enabled(cnn1_dev->cnn_aclk);
	int ret_mclk = __clk_is_enabled(cnn1_dev->cnn_mclk);

	return sprintf(buf, "%d\n", (ret_aclk && ret_mclk));
}
static ssize_t bpu1_clock_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &bpu1_clk);
	if (bpu1_clk) {
		if (!__clk_is_enabled(cnn1_dev->cnn_aclk) ||
			!__clk_is_enabled(cnn1_dev->cnn_mclk))
			x2_cnn_clock_up(cnn1_dev);
	} else {
		if (__clk_is_enabled(cnn1_dev->cnn_aclk) ||
			__clk_is_enabled(cnn1_dev->cnn_mclk))
			x2_cnn_clock_down(cnn1_dev);
	}
	return count;
}
static ssize_t bpu0_power_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	int ret = regulator_is_enabled(cnn0_dev->cnn_regulator);

	return sprintf(buf, "%d\n", ret);

}
static ssize_t bpu0_power_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &bpu0_power);
	if (bpu0_power) {
		if (!regulator_is_enabled(cnn0_dev->cnn_regulator))
			x2_cnn_power_up(cnn0_dev);
	} else {
		if (regulator_is_enabled(cnn0_dev->cnn_regulator))
			x2_cnn_power_down(cnn0_dev);
	}
	return count;
}
static ssize_t bpu1_power_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	int ret = regulator_is_enabled(cnn1_dev->cnn_regulator);

	return sprintf(buf, "%d\n", ret);

}
static ssize_t bpu1_power_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &bpu1_power);
	if (bpu1_power) {
		if (!regulator_is_enabled(cnn1_dev->cnn_regulator))
			x2_cnn_power_up(cnn1_dev);
	} else {
		if (regulator_is_enabled(cnn1_dev->cnn_regulator))
			x2_cnn_power_down(cnn1_dev);
	}
	return count;
}

static struct kobj_attribute pro_frequency = __ATTR(profiler_frequency, 0664,
						    fre_show, fre_store);
static struct kobj_attribute pro_enable    = __ATTR(profiler_enable, 0664,
						    enable_show, enable_store);
static struct kobj_attribute fc_enable    = __ATTR(fc_time_enable, 0664,
						    fc_time_enable_show,
						    fc_time_enable_store);
static struct kobj_attribute pro_ratio0    = __ATTR(ratio0, 0444,
						    ratio0_show, NULL);
static struct kobj_attribute pro_ratio1    = __ATTR(ratio1, 0444,
						    ratio1_show, NULL);
static struct kobj_attribute pro_queue0    = __ATTR(queue0, 0444,
						    queue0_show, NULL);
static struct kobj_attribute pro_queue1    = __ATTR(queue1, 0444,
						    queue1_show, NULL);
static struct kobj_attribute bpu0_fc_time  = __ATTR(fc_time, 0444,
						    bpu0_fc_time_show, NULL);
static struct kobj_attribute bpu1_fc_time  = __ATTR(fc_time, 0444,
						    bpu1_fc_time_show, NULL);
static struct kobj_attribute bpu0_clk_en   = __ATTR(clock_enable, 0644,
						    bpu0_clock_show,
						    bpu0_clock_store);
static struct kobj_attribute bpu1_clk_en   = __ATTR(clock_enable, 0644,
						    bpu1_clock_show,
						    bpu1_clock_store);
static struct kobj_attribute bpu0_power_en = __ATTR(power_enable, 0644,
						    bpu0_power_show,
						    bpu0_power_store);
static struct kobj_attribute bpu1_power_en  = __ATTR(power_enable, 0644,
						     bpu1_power_show,
						     bpu1_power_store);

static struct attribute *bpu_attrs[] = {
	&pro_frequency.attr,
	&pro_enable.attr,
	&fc_enable.attr,
	NULL,
};
static struct attribute *bpu0_attrs[] = {

	&pro_ratio0.attr,
	&pro_queue0.attr,
	&bpu0_fc_time.attr,
	&bpu0_clk_en.attr,
	&bpu0_power_en.attr,
	NULL,
};
static struct attribute *bpu1_attrs[] = {
	&pro_ratio1.attr,
	&pro_queue1.attr,
	&bpu1_fc_time.attr,
	&bpu1_clk_en.attr,
	&bpu1_power_en.attr,
	NULL,
};


static struct attribute_group bpu_attr_group = {
	.attrs = bpu_attrs,
};
static struct attribute_group bpu0_attr_group = {
	.name = "bpu0",
	.attrs = bpu0_attrs,
};
static struct attribute_group bpu1_attr_group = {
	.name = "bpu1",
	.attrs = bpu1_attrs,
};

static const struct attribute_group *bpu_attr_groups[] = {
	&bpu_attr_group,
	&bpu0_attr_group,
	&bpu1_attr_group,
	NULL,
};
struct bus_type bpu_subsys = {
	.name = "bpu",
};
static int __init x2_cnn_init(void)
{
	int retval = 0;

	/* Register the platform driver */
	retval = platform_driver_register(&x2_cnn_platform_driver);
	if (retval)
		pr_err("x2 cnn driver register failed\n");

	/*register bpu sys node*/
	if (subsys_system_register(&bpu_subsys, bpu_attr_groups))
		pr_err("fialed to register bpu subsystem");
	/*set bpu profiler timer*/
	init_timer(&check_timer);
	check_timer.function = &x2_cnn_check_func;
	mutex_init(&enable_lock);
	fc_time_enable = 0;

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
