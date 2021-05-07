/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/devfreq-event.h>

#include "./hobot_ddr_monitor.h"
#include "./hobot_ddr_axictrl.h"
#include "./hobot_ddr_mpu.h"
#include "./hobot_ddr_ecc.h"

static DEFINE_MUTEX(ddr_mo_mutex);

struct ddr_mon {
	struct platform_device *pdev;
	void __iomem *regaddr;
	phys_addr_t res_paddr;
	uint32_t res_memsize;
	void *res_vaddr;
	int irq;
	int major;
	int minor;
	struct cdev cdev;
	dev_t dev_num;
	struct class *ddr_mon_cls;
	wait_queue_head_t wq_head;
	int refcnt;
	spinlock_t lock;
	struct clk *ddr_mclk;
	struct devfreq_event_desc *desc;
	struct devfreq_event_dev *edev;
};

#ifdef CONFIG_HOBOT_XJ3

#define PORT_NUM 8
#define SYS_PCLK_HZ 250000000

/* Define DDRC registers for DDR detect */
#define uMCTL2_BASE 0xA2D00000
#define uMCTL2_MRCTRL0 0x10
#define uMCTL2_MRCTRL1 0x14
#define uMCTL2_MRSTAT  0x18

#else

#define PORT_NUM 6
#define SYS_PCLK_HZ 333333333

#endif

struct ddr_mon* ddrmon = NULL;
int rd_cmd_bytes = 64;
int wr_cmd_bytes = 64;

struct ddr_portdata_s {
	unsigned int waddr_num;
	unsigned int wdata_num;
	unsigned int waddr_cyc;
	unsigned int waddr_latency;
	unsigned int raddr_num;
	unsigned int rdata_num;
	unsigned int raddr_cyc;
	unsigned int raddr_latency;
#ifdef CONFIG_HOBOT_XJ3
	unsigned int max_rtrans_cnt;
	unsigned int max_rvalid_cnt;
	unsigned int acc_rtrans_cnt;
	unsigned int min_rtrans_cnt;

	unsigned int max_wtrans_cnt;
	unsigned int max_wready_cnt;
	unsigned int acc_wtrans_cnt;
	unsigned int min_wtrans_cnt;
#endif
};

struct ddr_mon_rec {
	unsigned long long curtime;
	struct ddr_portdata_s portdata[PORT_NUM];
	unsigned int rd_cmd_num;
	unsigned int wr_cmd_num;
	unsigned int mwr_cmd_num;
	unsigned int rdwr_swi_num;
	unsigned int war_haz_num;
	unsigned int raw_haz_num;
	unsigned int waw_haz_num;
	unsigned int act_cmd_num;
	unsigned int act_cmd_rd_num;
	unsigned int per_cmd_num;
	unsigned int per_cmd_rdwr_num;
};

typedef struct ddr_mon_cfg {
       unsigned int sample_period;
       unsigned int sample_number;
} ddr_mon_cfg;

#define TOTAL_RECORD_NUM 400
#define TOTAL_RESULT_SIZE (400*1024)

struct ddr_mon_rec* ddr_info = NULL;
struct ddr_mon_rec* ddr_info_bc = NULL;
unsigned int cur_idx = 0;
unsigned int mon_period = 100000;//unit is us, use 100ms by default
unsigned int g_sample_number = 0;
unsigned int g_rec_num = 0;

module_param(cur_idx, uint, 0644);
module_param(mon_period, uint, 0644);

static int hobot_dfi_disable(struct devfreq_event_dev *edev)
{
	pr_debug("%s %d\n", __func__, __LINE__);
	ddr_monitor_stop();

	return 0;
}

static int hobot_dfi_enable(struct devfreq_event_dev *edev)
{

	pr_debug("%s %d\n", __func__, __LINE__);
	ddr_monitor_start();

	return 0;
}

static int hobot_dfi_set_event(struct devfreq_event_dev *edev)
{
	/* we don't support. */
	return 0;
}

extern unsigned long hobot_dmc_cur_rate(void);
static int hobot_dfi_get_event(struct devfreq_event_dev *edev,
				struct devfreq_event_data *edata)
{
	unsigned long flags;
	struct ddr_mon_rec* cur_info;
	unsigned long read_cnt = 0;
	unsigned long write_cnt = 0;
	unsigned long mwrite_cnt = 0;
	unsigned long cur_ddr_rate;
	int cur;
	void __iomem *clk_reg = NULL;

	spin_lock_irqsave(&ddrmon->lock, flags);

	if (g_rec_num <= 0) {
		/* set to max load when not ready*/
		edata->load_count = 9999;
		edata->total_count = 10000;
		spin_unlock_irqrestore(&ddrmon->lock, flags);
		return 0;
	}

	cur = (cur_idx - 1 + TOTAL_RECORD_NUM) % TOTAL_RECORD_NUM;

	cur_ddr_rate = hobot_dmc_cur_rate();
	if (unlikely(cur_ddr_rate == 0)) {
		/* we trust ddr_mclk before changing ddr clk */
		cur_ddr_rate = clk_get_rate(ddrmon->ddr_mclk) * 4;
		pr_debug("get cur_ddr_rate %ld from ddr_mclk\n", cur_ddr_rate);
	}

	cur_info = &ddr_info[cur];

	read_cnt = cur_info->rd_cmd_num * rd_cmd_bytes * (1000000 / mon_period) >> 20;

	write_cnt = cur_info->wr_cmd_num * wr_cmd_bytes * (1000000 / mon_period) >> 20;

	mwrite_cnt = cur_info->mwr_cmd_num * wr_cmd_bytes * (1000000 / mon_period) >> 20;

	edata->load_count = read_cnt + write_cnt + mwrite_cnt;
	edata->total_count = (cur_ddr_rate * 4) >> 20;

	spin_unlock_irqrestore(&ddrmon->lock, flags);

	clk_reg = ioremap(0xa1000030, 32);
	pr_debug("rd:%6ld, wr:%6ld, mwr:%6ld, load:%6ld, total:%6ld, "\
		"ddr_clk:%4ldMHz, clk_reg:%08x\n",
		read_cnt, write_cnt, mwrite_cnt, edata->load_count,
		edata->total_count, cur_ddr_rate / 1000000, readl(clk_reg));

	iounmap(clk_reg);

	return 0;
}

static const struct devfreq_event_ops hobot_dfi_ops = {
	.disable = hobot_dfi_disable,
	.enable = hobot_dfi_enable,
	.get_event = hobot_dfi_get_event,
	.set_event = hobot_dfi_set_event,
};


typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define DDR_MONITOR_READ	_IOWR('m', 0, reg_t)
#define DDR_MONITOR_WRITE	_IOW('m', 1, reg_t)
#define DDR_MONITOR_CUR	_IOWR('m', 2, struct ddr_mon_rec)
#define DDR_MONITOR_SAMPLE_CONFIG_SET  _IOW('m', 3, ddr_mon_cfg)

static int ddr_monitor_open(struct inode *pinode, struct file *pfile)
{
	pr_debug("\n");

	if (ddrmon && !ddr_info) {
		/* allocate double size, below half for ddr_info bouncing buffer */
		ddr_info = vmalloc(sizeof(struct ddr_mon_rec) * TOTAL_RECORD_NUM * 2);
		if (ddr_info == NULL) {
			pr_err("failed to allocate ddr_info buffer, size:%lx\n",
				sizeof(struct ddr_mon_rec) * TOTAL_RECORD_NUM * 2);
			return -ENOMEM;
		}

		/*
		 * we have to use a boucing buffer since ddr_info can't be copied to used
		 * in atomic context in ddr_monitor_read.
		 */
		ddr_info_bc = ddr_info + TOTAL_RECORD_NUM;

		cur_idx = 0;
		g_rec_num = 0;
	}

	return 0;
}

static int ddr_monitor_release(struct inode *pinode, struct file *pfile)
{
	pr_debug("\n");

	ddr_monitor_stop();

	return 0;
}

static ssize_t ddr_monitor_read(struct file *pfile, char *puser_buf, size_t len, loff_t *poff)
{
	unsigned long flags;
	int ret;
	int offset = 0;
	int rec_num;

	wait_event_interruptible(ddrmon->wq_head, g_rec_num >= g_sample_number);

	spin_lock_irqsave(&ddrmon->lock, flags);

	if (cur_idx >= g_sample_number) {
		memcpy(ddr_info_bc, &ddr_info[cur_idx - g_sample_number],
			g_sample_number * sizeof(struct ddr_mon_rec));
	} else {
		offset = (g_sample_number - cur_idx) * sizeof(struct ddr_mon_rec);
		memcpy((void *)ddr_info_bc, &ddr_info[TOTAL_RECORD_NUM - (g_sample_number - cur_idx)], offset);

		memcpy((void *)ddr_info_bc + offset, &ddr_info[0], cur_idx * sizeof(struct ddr_mon_rec));
	}
	rec_num = g_sample_number;
	g_rec_num = 0;
	spin_unlock_irqrestore(&ddrmon->lock, flags);

	ret = copy_to_user(puser_buf, ddr_info_bc, rec_num * sizeof(struct ddr_mon_rec));
	if (ret) {
		pr_err("%s:%d copy to user error :%d\n", __func__, __LINE__, ret);
		return -EFAULT;
	}
	ret = rec_num * sizeof(struct ddr_mon_rec);

	return ret;
}

static ssize_t ddr_monitor_write(struct file *pfile, const char *puser_buf, size_t len, loff_t *poff)
{
	pr_debug("\n");

	return 0;
}

static long ddr_monitor_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	void __iomem *iomem = ddrmon->regaddr;
	unsigned int temp = 0;
	int cur = 0;
	reg_t reg;

	ddr_mon_cfg sample_config;
	sample_config.sample_period = 1;
	sample_config.sample_number = 50;

	if ( NULL == iomem ) {
		pr_err("ddr_monitor no iomem\n");
		return -EINVAL;
	}

	switch (cmd) {
	case DDR_MONITOR_READ:
		if (!arg) {
			pr_err("reg read error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			pr_err("reg read error, copy data from user failed\n");
			return -EINVAL;
		}
		reg.value = readl(iomem + reg.offset);
		if ( copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg)) ) {
			pr_err("reg read error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case DDR_MONITOR_WRITE:
		if ( !arg ) {
			pr_err("reg write error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			pr_err("reg write error, copy data from user failed\n");
			return -EINVAL;
		}
		writel(reg.value, iomem + reg.offset );
		break;
	case DDR_MONITOR_CUR:
		if (!arg) {
			pr_err("get cur error\n");
			return -EINVAL;
		}
		cur  = (cur_idx - 1 + TOTAL_RECORD_NUM) % TOTAL_RECORD_NUM;
		if ( copy_to_user((void __user *)arg, (void *)(ddr_info + cur), sizeof(struct ddr_mon_rec)) ) {
			pr_err("get cur error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case DDR_MONITOR_SAMPLE_CONFIG_SET:
		if (!arg) {
			dev_err(&ddrmon->pdev->dev,
					"sampletime set arg should not be NULL\n");
			return -EINVAL;
		}
		if (copy_from_user((void *)&sample_config,
					(void __user *)arg, sizeof(ddr_mon_cfg))) {
			dev_err(&ddrmon->pdev->dev,
					"sampletime set copy_from_user failed\n");
			return -EINVAL;
		}
		/* sample_period's unit is ms */
		temp = sample_config.sample_period;
		g_sample_number = sample_config.sample_number;
		if (g_sample_number > TOTAL_RECORD_NUM)
			g_sample_number = TOTAL_RECORD_NUM;
		mon_period = temp * 1000;
		ddr_monitor_start();

		break;
	default:

		break;
	}
	return 0;
}

int ddr_monitor_mmap(struct file *filp, struct vm_area_struct *pvma)
{

	if (remap_pfn_range(pvma, pvma->vm_start,
						virt_to_pfn(ddrmon->res_vaddr),
						pvma->vm_end - pvma->vm_start,
						pvma->vm_page_prot)) {
		pr_err("ddr_monitor_mmap fail\n");
		return -EAGAIN;
	}

	return 0;
}

struct file_operations ddr_monitor_fops = {
	.owner			= THIS_MODULE,
	.mmap 			= ddr_monitor_mmap,
	.open			= ddr_monitor_open,
	.read			= ddr_monitor_read,
	.write			= ddr_monitor_write,
	.release		= ddr_monitor_release,
	.unlocked_ioctl = ddr_monitor_ioctl,
};

int ddr_monitor_cdev_create(void)
{
	int ret = 0;
	int error;

	pr_debug("\n");

	ddrmon->ddr_mon_cls = class_create(THIS_MODULE, "ddr_monitor");
	if (IS_ERR(ddrmon->ddr_mon_cls))
		return PTR_ERR(ddrmon->ddr_mon_cls);

	error = alloc_chrdev_region(&ddrmon->dev_num, 0, 1, "ddr_monitor");

	if (!error) {
		ddrmon->major = MAJOR(ddrmon->dev_num);
		ddrmon->minor = MINOR(ddrmon->dev_num);
	}

	if (ret < 0)
		return ret;

	cdev_init(&ddrmon->cdev, &ddr_monitor_fops);
	ddrmon->cdev.owner	= THIS_MODULE;

	cdev_add(&ddrmon->cdev, ddrmon->dev_num, 1);

	device_create(ddrmon->ddr_mon_cls, NULL, ddrmon->dev_num, NULL, "ddrmonitor");
	if (ret)
		return ret;

	return ret;
}

void ddr_monitor_dev_remove(void)
{
	pr_debug("\n");

	cdev_del(&ddrmon->cdev);

	unregister_chrdev_region(ddrmon->dev_num, 1);
}

int ddr_monitor_start(void)
{
	pr_debug("%s: ddrmon->refcnt :%d\n", __func__, ddrmon->refcnt);

	if (!ddr_info) {
		/* allocate double size, below half for ddr_info bouncing buffer */
		ddr_info = vmalloc(sizeof(struct ddr_mon_rec) * TOTAL_RECORD_NUM * 2);
		if (ddr_info == NULL) {
			pr_err("failed to allocate ddr_info buffer, size:%lx\n",
				sizeof(struct ddr_mon_rec) * TOTAL_RECORD_NUM * 2);
			return -ENOMEM;
		}

		ddr_info_bc = ddr_info + TOTAL_RECORD_NUM;

		cur_idx = 0;
		g_rec_num = 0;
	}

	writel(0xFFF, ddrmon->regaddr + PERF_MONITOR_ENABLE);
	writel(0x1, ddrmon->regaddr + PERF_MONITOR_ENABLE_UNMASK);
	pr_info("set PERF_MONITOR_PERIOD to %dms\n", mon_period/1000);
	writel((SYS_PCLK_HZ /1000000) * mon_period, ddrmon->regaddr + PERF_MONITOR_PERIOD);
	ddrmon->refcnt++;

	return 0;
}

int ddr_monitor_stop(void)
{
	pr_debug("%s: ddrmon->refcnt :%d\n", __func__, ddrmon->refcnt);

	ddrmon->refcnt--;
	if (ddrmon->refcnt > 0) {
		return 0;
	} else if (ddrmon->refcnt < 0) {
		pr_err("ddr monitor start and stop unpaired: %d\n", ddrmon->refcnt);
		return -1;
	}

	if (ddrmon && ddr_info) {
		writel(0, ddrmon->regaddr + PERF_MONITOR_ENABLE);
		writel(0x1, ddrmon->regaddr + PERF_MONITOR_ENABLE_SETMASK);
		vfree(ddr_info);
		ddr_info = NULL;
		return 0;
	} else {
		pr_err("%s:device not ready!\n", __func__);
		return -1;
	}
}

int ddr_get_port_status(void)
{
	int i = 0;
	int step;

	ddr_info[cur_idx].curtime = ktime_to_ms(ktime_get());

	for (i = 0; i < PORT_NUM; i++) {
		step = i * MP_REG_OFFSET;
#ifdef CONFIG_HOBOT_XJ3
		if (i >= 6)
			step += MP_REG_OFFSET;
#endif

		ddr_info[cur_idx].portdata[i].raddr_num = readl(ddrmon->regaddr + MP_BASE_RADDR_TX_NUM + step);
		ddr_info[cur_idx].portdata[i].rdata_num = readl(ddrmon->regaddr + MP_BASE_RDATA_TX_NUM + step);
		ddr_info[cur_idx].portdata[i].raddr_cyc = readl(ddrmon->regaddr + MP_BASE_RADDR_ST_CYC + step);
		ddr_info[cur_idx].portdata[i].raddr_latency = readl(ddrmon->regaddr + MP_BASE_RA2LSTRD_LATENCY + step);

#ifdef CONFIG_HOBOT_XJ3
		ddr_info[cur_idx].portdata[i].max_rtrans_cnt = readl(ddrmon->regaddr + MP_BASE_MAX_RTRANS_CNT + step);
		ddr_info[cur_idx].portdata[i].max_rvalid_cnt = readl(ddrmon->regaddr + MP_BASE_MAX_RVALID_CNT + step);
		ddr_info[cur_idx].portdata[i].acc_rtrans_cnt = readl(ddrmon->regaddr + MP_BASE_ACC_RTRANS_CNT + step);
		ddr_info[cur_idx].portdata[i].min_rtrans_cnt = readl(ddrmon->regaddr + MP_BASE_MIN_RTRANS_CNT + step);

		ddr_info[cur_idx].portdata[i].max_wtrans_cnt = readl(ddrmon->regaddr + MP_BASE_MAX_WTRANS_CNT + step);
		ddr_info[cur_idx].portdata[i].max_wready_cnt = readl(ddrmon->regaddr + MP_BASE_MAX_WREADY_CNT + step);
		ddr_info[cur_idx].portdata[i].acc_wtrans_cnt = readl(ddrmon->regaddr + MP_BASE_ACC_WTRANS_CNT + step);
		ddr_info[cur_idx].portdata[i].min_wtrans_cnt = readl(ddrmon->regaddr + MP_BASE_MIN_WTRANS_CNT + step);
#endif
		ddr_info[cur_idx].portdata[i].waddr_num = readl(ddrmon->regaddr + MP_BASE_WADDR_TX_NUM + step);
		ddr_info[cur_idx].portdata[i].wdata_num = readl(ddrmon->regaddr + MP_BASE_WDATA_TX_NUM + step);
		ddr_info[cur_idx].portdata[i].waddr_cyc = readl(ddrmon->regaddr + MP_BASE_WADDR_ST_CYC + step);
		ddr_info[cur_idx].portdata[i].waddr_latency = readl(ddrmon->regaddr + MP_BASE_WA2BRESP_LATENCY + step);
	}

	ddr_info[cur_idx].rd_cmd_num = readl(ddrmon->regaddr + RD_CMD_TX_NUM);
	ddr_info[cur_idx].wr_cmd_num = readl(ddrmon->regaddr + WR_CMD_TX_NUM);
	ddr_info[cur_idx].mwr_cmd_num = readl(ddrmon->regaddr + MWR_CMD_TX_NUM);
	ddr_info[cur_idx].rdwr_swi_num = readl(ddrmon->regaddr + RDWR_SWITCH_NUM);
	ddr_info[cur_idx].act_cmd_num = readl(ddrmon->regaddr + ACT_CMD_TX_NUM);
	ddr_info[cur_idx].act_cmd_rd_num = readl(ddrmon->regaddr + ACT_CMD_TX_FOR_RD_TX_NUM);
	ddr_info[cur_idx].war_haz_num = readl(ddrmon->regaddr + WAR_HAZARD_NUM);
	ddr_info[cur_idx].raw_haz_num = readl(ddrmon->regaddr + RAW_HAZARD_NUM);
	ddr_info[cur_idx].waw_haz_num = readl(ddrmon->regaddr + WAW_HAZARD_NUM);
	ddr_info[cur_idx].per_cmd_num = readl(ddrmon->regaddr + PERCHARGE_CMD_TX_NUM);
	ddr_info[cur_idx].per_cmd_rdwr_num = readl(ddrmon->regaddr + PERCHARGE_CMD_FOR_RDWR_TX_NUM);

	cur_idx = (cur_idx + 1) % TOTAL_RECORD_NUM;
	g_rec_num++;

	if (g_rec_num >= g_sample_number)
		wake_up_interruptible(&ddrmon->wq_head);

	return 0;
}

static irqreturn_t ddr_monitor_isr(int this_irq, void *data)
{
	writel(0x1, ddrmon->regaddr + PERF_MONITOR_SRCPND);

	spin_lock(&ddrmon->lock);
	ddr_get_port_status();
	spin_unlock(&ddrmon->lock);

	return IRQ_HANDLED;
}


static unsigned int read_ctl_value;
static unsigned int write_ctl_value;
static ssize_t cpu_read_ctl_store(struct device_driver *drv,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~0x0f;
	tmp |= read_ctl_value;
	writel(tmp, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t cpu_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bifdma_read_ctl_store(struct device_driver *drv,
				     const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 4);
	tmp |= (read_ctl_value << 4);
	writel(tmp, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t bifdma_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 4;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bpu0_read_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 8);
	tmp |= (read_ctl_value << 8);
	writel(tmp, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t bpu0_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 8;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}

static ssize_t bpu1_read_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 12);
	tmp |= (read_ctl_value << 12);
	writel(tmp, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t bpu1_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 12;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}
static ssize_t vio0_read_ctl_store(struct device_driver *drv,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 16);
	tmp |= (read_ctl_value << 16);
	writel(tmp, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t vio0_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 16;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}

#ifdef CONFIG_HOBOT_XJ3
static ssize_t vpu_read_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 20);
	tmp |= (read_ctl_value << 20);
	writel(tmp, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t vpu_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 20;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}

static ssize_t vio1_read_ctl_store(struct device_driver *drv,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 24);
	tmp |= (read_ctl_value << 24);
	writel(tmp, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t vio1_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 24;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
#endif

static ssize_t periph_read_ctl_store(struct device_driver *drv,
				     const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;
	int shift = 20;

#ifdef CONFIG_HOBOT_XJ3
	shift = 28;
#endif

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << shift);
	tmp |= (read_ctl_value << shift);
	writel(tmp, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t periph_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int shift = 20;

#ifdef CONFIG_HOBOT_XJ3
	shift = 28;
#endif

	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= shift;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t all_read_ctl_store(struct device_driver *drv,
				  const char *buf, size_t count)
{
	mutex_lock(&ddr_mo_mutex);

	sscanf(buf, "%x", &read_ctl_value);
	writel(read_ctl_value, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);

	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t all_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;

	tmp = readl(ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	len = sprintf(buf, "%08x:\n", tmp);
	len += sprintf(buf + len, "P0_CPU:    %d\n", tmp & 0xF);
	len += sprintf(buf + len, "P1_BIFDMA: %d\n", (tmp >> 4) & 0xF);
	len += sprintf(buf + len, "P2_CNN0:   %d\n", (tmp >> 8) & 0xF);
	len += sprintf(buf + len, "P3_CNN1:   %d\n", (tmp >> 12) & 0xF);
	len += sprintf(buf + len, "P4_VIO0:   %d\n", (tmp >> 16) & 0xF);
#ifdef CONFIG_HOBOT_XJ2
	len += sprintf(buf + len, "P5_PERI:   %d\n", (tmp >> 20) & 0xF);
#else
	len += sprintf(buf + len, "P5_VPU:    %d\n", (tmp >> 20) & 0xF);
	len += sprintf(buf + len, "P6_VIO1:   %d\n", (tmp >> 24) & 0xF);
	len += sprintf(buf + len, "P7_PERI:   %d\n", (tmp >> 28) & 0xF);
#endif

	return len;
}


static struct driver_attribute cpu_read_ctl = __ATTR(cpu, 0664,
						   cpu_read_ctl_show,
						   cpu_read_ctl_store);
static struct driver_attribute bifdma_read_ctl = __ATTR(bifdma, 0664,
						      bifdma_read_ctl_show,
						      bifdma_read_ctl_store);
static struct driver_attribute bpu0_read_ctl = __ATTR(bpu0, 0664,
						    bpu0_read_ctl_show,
						    bpu0_read_ctl_store);
static struct driver_attribute bpu1_read_ctl = __ATTR(bpu1, 0664,
						    bpu1_read_ctl_show,
						    bpu1_read_ctl_store);
static struct driver_attribute vio0_read_ctl    = __ATTR(vio0, 0664,
						      vio0_read_ctl_show,
						      vio0_read_ctl_store);
#ifdef CONFIG_HOBOT_XJ3
static struct driver_attribute vpu_read_ctl    = __ATTR(vpu, 0664,
						      vpu_read_ctl_show,
						      vpu_read_ctl_store);
static struct driver_attribute vio1_read_ctl    = __ATTR(vio1, 0664,
						      vio1_read_ctl_show,
						      vio1_read_ctl_store);
#endif
static struct driver_attribute periph_read_ctl = __ATTR(peri, 0664,
						      periph_read_ctl_show,
						      periph_read_ctl_store);
static struct driver_attribute all_read_ctl    = __ATTR(all, 0664,
						      all_read_ctl_show,
						      all_read_ctl_store);

static struct attribute *read_qctrl_attrs[] = {
	&cpu_read_ctl.attr,
	&bifdma_read_ctl.attr,
	&bpu0_read_ctl.attr,
	&bpu1_read_ctl.attr,
	&vio0_read_ctl.attr,
#ifdef CONFIG_HOBOT_XJ3
	&vpu_read_ctl.attr,
	&vio1_read_ctl.attr,
#endif
	&periph_read_ctl.attr,
	&all_read_ctl.attr,
	NULL,
};


static ssize_t cpu_write_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~0x0f;
	tmp |= write_ctl_value;
	writel(tmp, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t cpu_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bifdma_write_ctl_store(struct device_driver *drv,
				      const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 4);
	tmp |= (write_ctl_value << 4);
	writel(tmp, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bifdma_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 4;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bpu0_write_ctl_store(struct device_driver *drv,
				    const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 8);
	tmp |= (write_ctl_value << 8);
	writel(tmp, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bpu0_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 8;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}

static ssize_t bpu1_write_ctl_store(struct device_driver *drv,
				    const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 12);
	tmp |= (write_ctl_value << 12);
	writel(tmp, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bpu1_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 12;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t vio0_write_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 16);
	tmp |= (write_ctl_value << 16);
	writel(tmp, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t vio0_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 16;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}

#ifdef CONFIG_HOBOT_XJ3
static ssize_t vpu_write_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 20);
	tmp |= (write_ctl_value << 20);
	writel(tmp, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t vpu_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 20;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}

static ssize_t vio1_write_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 24);
	tmp |= (write_ctl_value << 24);
	writel(tmp, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t vio1_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 24;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
#endif

static ssize_t periph_write_ctl_store(struct device_driver *drv,
				      const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;
	int shift = 20;

#ifdef CONFIG_HOBOT_XJ3
	shift = 28;
#endif

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << shift);
	tmp |= (write_ctl_value << shift);
	writel(tmp, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t periph_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int shift = 20;

#ifdef CONFIG_HOBOT_XJ3
	shift = 28;
#endif

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= shift;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t all_write_ctl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	mutex_lock(&ddr_mo_mutex);

	sscanf(buf, "%x", &write_ctl_value);
	writel(write_ctl_value, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);

	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t all_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;

	tmp = readl(ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	len = sprintf(buf, "%08x:\n", tmp);
	len += sprintf(buf + len, "P0_CPU:    %d\n", tmp & 0xF);
	len += sprintf(buf + len, "P1_BIFDMA: %d\n", (tmp >> 4) & 0xF);
	len += sprintf(buf + len, "P2_CNN0:   %d\n", (tmp >> 8) & 0xF);
	len += sprintf(buf + len, "P3_CNN1:   %d\n", (tmp >> 12) & 0xF);
	len += sprintf(buf + len, "P4_VIO0:   %d\n", (tmp >> 16) & 0xF);
#ifdef CONFIG_HOBOT_XJ2
	len += sprintf(buf + len, "P5_PERI:   %d\n", (tmp >> 20) & 0xF);
#else
	len += sprintf(buf + len, "P5_VPU:    %d\n", (tmp >> 20) & 0xF);
	len += sprintf(buf + len, "P6_VIO1:   %d\n", (tmp >> 24) & 0xF);
	len += sprintf(buf + len, "P7_PERI:   %d\n", (tmp >> 28) & 0xF);
#endif

	return len;
}


static struct driver_attribute cpu_write_ctl = __ATTR(cpu, 0664,
						    cpu_write_ctl_show,
						    cpu_write_ctl_store);
static struct driver_attribute bifdma_write_ctl = __ATTR(bifdma, 0664,
						       bifdma_write_ctl_show,
						       bifdma_write_ctl_store);
static struct driver_attribute bpu0_write_ctl = __ATTR(bpu0, 0664,
						     bpu0_write_ctl_show,
						     bpu0_write_ctl_store);
static struct driver_attribute bpu1_write_ctl = __ATTR(bpu1, 0664,
						     bpu1_write_ctl_show,
						     bpu1_write_ctl_store);
static struct driver_attribute vio0_write_ctl    = __ATTR(vio0, 0664,
						       vio0_write_ctl_show,
						       vio0_write_ctl_store);
#ifdef CONFIG_HOBOT_XJ3
static struct driver_attribute vpu_write_ctl    = __ATTR(vpu, 0664,
						       vpu_write_ctl_show,
						       vpu_write_ctl_store);
static struct driver_attribute vio1_write_ctl    = __ATTR(vio1, 0664,
						       vio1_write_ctl_show,
						       vio1_write_ctl_store);
#endif

static struct driver_attribute periph_write_ctl = __ATTR(peri, 0664,
						       periph_write_ctl_show,
						       periph_write_ctl_store);
static struct driver_attribute all_write_ctl    = __ATTR(all, 0664,
						       all_write_ctl_show,
						       all_write_ctl_store);

static struct attribute *write_qctl_attrs[] = {
	&cpu_write_ctl.attr,
	&bifdma_write_ctl.attr,
	&bpu0_write_ctl.attr,
	&bpu1_write_ctl.attr,
	&vio0_write_ctl.attr,
#ifdef CONFIG_HOBOT_XJ3
	&vpu_write_ctl.attr,
	&vio1_write_ctl.attr,
#endif
	&periph_write_ctl.attr,
	&all_write_ctl.attr,
	NULL,
};


static struct attribute_group read_attr_group = {
	.name = "read_qos_ctrl",
	.attrs = read_qctrl_attrs,
};
static struct attribute_group write_attr_group = {
	.name = "write_qos_ctrl",
	.attrs = write_qctl_attrs,
};



static const struct attribute_group *ddr_attr_groups[] = {
	&read_attr_group,
	&write_attr_group,
#ifdef CONFIG_HOBOT_XJ3
	&axibus_group,
#endif
	NULL,
};
struct bus_type ddr_monitor_subsys = {
	.name = "ddr_monitor",
};


static int ddr_monitor_get_ddr_type(struct platform_device *pdev)
{
	struct resource *pres;
	void __iomem *ddrc_base;
	u32 reg_val;

	pres = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pres) {
		devm_kfree(&pdev->dev, ddrmon);
		return -ENOMEM;
	}

	ddrc_base = ioremap(pres->start, pres->end - pres->start);
	if (IS_ERR(ddrc_base)) {
		pr_err("%s:DDRC base address map failed\n", __func__);
		return PTR_ERR(ddrc_base);
	}

	reg_val = readl(ddrc_base);
	if (reg_val & 0x10) {
		pr_info("DDR4 detected\n");
		/* on ddr4, rd and wr cmd bytes could be different according to ddr params
		 * currently it's 32 byte on two ddr4 modules
		 * */
		rd_cmd_bytes = 32;
		wr_cmd_bytes = 32;
	} else if (reg_val & 0x20) {
		pr_info("LPDDR4 detected\n");
		rd_cmd_bytes = 64;
		wr_cmd_bytes = 64;
	} else {
		pr_err("Can't detected DDR type\n");
	}

	iounmap(ddrc_base);
	ddrc_base = NULL;

	return 0;

}

static int ddr_monitor_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *pres;
	struct devfreq_event_desc *desc;

	pr_debug("\n");

	ddrmon = devm_kmalloc(&pdev->dev, sizeof(struct ddr_mon), GFP_KERNEL);
	if (!ddrmon) {
		return -ENOMEM;
	}
	memset(ddrmon, 0, sizeof(struct ddr_mon));

	pres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pres) {
		devm_kfree(&pdev->dev, ddrmon);
		return -ENOMEM;
	}

	ddrmon->regaddr = devm_ioremap_resource(&pdev->dev, pres);
	if (IS_ERR(ddrmon->regaddr)) {
		pr_err("ddr monitor reg address map failed\n");
		return PTR_ERR(ddrmon->regaddr);
	}

	ddrmon->irq = platform_get_irq(pdev, 0);

	ret = request_irq(ddrmon->irq, ddr_monitor_isr, IRQF_TRIGGER_HIGH,
			  dev_name(&pdev->dev), ddrmon);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		return -ENODEV;
	}

#ifdef CONFIG_HOBOT_XJ3
	ret = ddr_mpu_init(pdev);
	if (ret < 0)
		pr_info("mpu is not supported\n");

	if (ddr_monitor_get_ddr_type(pdev) < 0)
		return -ENODEV;

#endif

	desc = devm_kzalloc(&pdev->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->ops = &hobot_dfi_ops;
	desc->driver_data = ddrmon;
	desc->name = pdev->dev.of_node->name;
	ddrmon->desc = desc;

	ddrmon->edev = devm_devfreq_event_add_edev(&pdev->dev, desc);
	if (IS_ERR(ddrmon->edev)) {
		dev_err(&pdev->dev,
			"failed to add devfreq-event device %ld\n",
			PTR_ERR(ddrmon->edev));

		return PTR_ERR(ddrmon->edev);
	}

	platform_set_drvdata(pdev, ddrmon);
	ddr_monitor_cdev_create();
	init_waitqueue_head(&ddrmon->wq_head);
	spin_lock_init(&ddrmon->lock);

#ifdef CONFIG_HOBOT_XJ3
	writel(0x04032211, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	writel(0x04032211, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
#else
	writel(0x21100, ddrmon->regaddr + DDR_PORT_READ_QOS_CTRL);
	writel(0x21100, ddrmon->regaddr + DDR_PORT_WRITE_QOS_CTRL);
#endif

	ddrmon->ddr_mclk = devm_clk_get(&pdev->dev, "ddr_mclk");
	if (IS_ERR(ddrmon->ddr_mclk)) {
		pr_err("failed to get ddr_mclk");
		return -ENODEV;
	}

	if (hobot_ddr_axictrl_init(pdev) < 0)
		return -ENODEV;

	if (hobot_ddr_ecc_init(pdev) < 0)
		return -ENODEV;

	pr_info("ddr monitor init finished.");

	return ret;
}

static int ddr_monitor_remove(struct platform_device *pdev)
{
	pr_info("\n");

	ddr_monitor_dev_remove();
	devm_kfree(&pdev->dev, ddrmon);
	ddrmon = NULL;

	ddr_mpu_deinit(pdev);
	hobot_ddr_ecc_deinit(pdev);
	hobot_ddr_axictrl_deinit(pdev);

	return 0;
}

static const struct of_device_id ddr_monitor_match[] = {
	{.compatible = "hobot,x2-ddrmonitor"},
	{.compatible = "hobot,ddrmonitor"},
	{}
};

MODULE_DEVICE_TABLE(of, ddr_monitor_match);

static struct platform_driver ddr_monitor_driver = {
	.probe	= ddr_monitor_probe,
	.remove = ddr_monitor_remove,
	.driver = {
		.name	= "ddr_monitor",
		.of_match_table = ddr_monitor_match,
		.groups = ddr_attr_groups,
	},
};

module_platform_driver(ddr_monitor_driver);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("ddr performance monitor module");
MODULE_LICENSE("GPL");
