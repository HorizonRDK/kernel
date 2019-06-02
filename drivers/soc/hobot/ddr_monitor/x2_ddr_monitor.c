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

#include "x2_ddr_monitor.h"
static DEFINE_MUTEX(ddr_mo_mutex);

struct ddr_monitor_dev_s {
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
	struct class *ddr_monitor_classes;
	wait_queue_head_t wq_head;
	spinlock_t lock;
};

struct ddr_monitor_dev_s* g_ddr_monitor_dev = NULL;

struct ddr_portdata_s {
	unsigned int waddr_num;
	unsigned int wdata_num;
	unsigned int waddr_cyc;
	unsigned int waddr_latency;
	unsigned int raddr_num;
	unsigned int rdata_num;
	unsigned int raddr_cyc;
	unsigned int raddr_latency;
};

struct ddr_monitor_result_s {
	unsigned long long curtime;
	struct ddr_portdata_s portdata[6];
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

#define TOTAL_RECORD_NUM 400
#define TOTAL_RESULT_SIZE (400*1024)
ktime_t g_ktime_start;

struct ddr_monitor_result_s* ddr_info = NULL;
char * result_buf = NULL;
unsigned int g_current_index = 0;
volatile unsigned int g_record_num = 0;
unsigned int g_monitor_poriod = 1000;

module_param(g_current_index, uint, 0644);
//module_param(g_record_num, uint, 0644);
module_param(g_monitor_poriod, uint, 0644);

static int dbg_ddr_monitor_show(struct seq_file *s, void *unused)
{
#if 0
	int i,j;
	int start = 0;
	int cur = 0;
	volatile int num = 0;
	printk("g_record_num:%d\n", g_record_num);
	seq_printf(s, "The ddr_monitor module status: g_record_num:%d\n", g_record_num);
	if (!ddr_info) {
		seq_printf(s, "ddr_monitor not started \n");
		return 0;
	}
	if (g_record_num > 0) {
		num = g_record_num;
		printk("g_record_num@@@@@@:%d, num:%d\n", g_record_num, num);
		if (num >= TOTAL_RECORD_NUM)
			num = TOTAL_RECORD_NUM;
		start = (g_current_index + TOTAL_RECORD_NUM - num) % TOTAL_RECORD_NUM;
		g_record_num = 0;

		seq_printf(s, "The ddr_monitor module status: num:%d\n", num);
		for (j = 0; j < num; j++) {
			seq_printf(s, "\nTime %llu ", ddr_info[cur].curtime);
			seq_printf(s, "Read: ");
			cur = (start + j) % TOTAL_RECORD_NUM;
			for (i = 0; i < 6; i++)
			{
				if (ddr_info[cur].portdata[i].raddr_num) {
					seq_printf(s, "p[%d](bw:%u stall:%u delay:%u) ", i, \
						(ddr_info[cur].portdata[i].rdata_num * 16 * (1000/g_monitor_poriod)) >> 20, \
						ddr_info[cur].portdata[i].raddr_cyc / ddr_info[cur].portdata[i].rdata_num, \
						ddr_info[cur].portdata[i].raddr_latency / ddr_info[cur].portdata[i].rdata_num);
				} else {
					seq_printf(s, "p[%d](bw:%u stall:%u delay:%u) ", i, 0, 0, 0);
				}
			}
			seq_printf(s, "ddrc:%u MB/s; ", (ddr_info[cur].rd_cmd_num * 64 * (1000/g_monitor_poriod)) >> 20);
			seq_printf(s, "Write: ");
			for (i = 0; i < 6; i++)
			{
				if (ddr_info[cur].portdata[i].waddr_num) {
					seq_printf(s, "p[%d](bw:%u stall:%u delay:%u) ", i, \
						(ddr_info[cur].portdata[i].wdata_num * 16 * (1000/g_monitor_poriod)) >> 20, \
						ddr_info[cur].portdata[i].waddr_cyc / ddr_info[cur].portdata[i].wdata_num, \
						ddr_info[cur].portdata[i].waddr_latency / ddr_info[cur].portdata[i].wdata_num);
				} else {
					seq_printf(s, "p[%d](bw:%u stall:%u delay:%u) ", i, 0, 0, 0);
				}
			}
			seq_printf(s, "ddrc %u MB/s, mask %u MB/s\n", (ddr_info[cur].wr_cmd_num * 64 * (1000/g_monitor_poriod)) >> 20, (ddr_info[cur].mwr_cmd_num * 64 * (1000/g_monitor_poriod)) >> 20);
		}
		seq_printf(s, "The ddr_monitor module status end: g_record_num:%d\n", g_record_num);
	}
#endif
	return 0;
}

ssize_t ddr_monitor_debug_write(struct file *file, const char __user *buf, size_t size, loff_t *p)
{
	char info[255];
	memset(info, 0, 255);
	if (copy_from_user(info, buf, size))
		return size;
	printk("ddr_monitor:%s\n", info);
	if (!memcmp(info, "ddrstart", 7)) {
		ddr_monitor_start();
		return size;
	} else if (!memcmp(info, "ddrstop", 7)) {
		ddr_monitor_stop();
		return size;
	}

	return size;
}

static int get_monitor_data(char* buf)
{
	int i,j;
	int start = 0;
	int cur = 0;
	int length = 0;
	volatile int num = 0;
	if (!ddr_info) {
		length += sprintf(buf + length, "ddr_monitor not started \n");
		return 0;
	}
	if (g_record_num > 0) {

		spin_lock_irq(&g_ddr_monitor_dev->lock);
		num = g_record_num;
		if (num >= TOTAL_RECORD_NUM)
			num = TOTAL_RECORD_NUM;
		start = (g_current_index + TOTAL_RECORD_NUM - num) % TOTAL_RECORD_NUM;
		g_record_num = 0;
		spin_unlock_irq(&g_ddr_monitor_dev->lock);
		for (j = 0; j < num; j++) {
			cur = (start + j) % TOTAL_RECORD_NUM;
			length += sprintf(buf + length, "Time %llu ", ddr_info[cur].curtime);
			length += sprintf(buf + length, "Read: ");
			for (i = 0; i < 6; i++)
			{
				if (ddr_info[cur].portdata[i].raddr_num) {
					length += sprintf(buf + length, "p[%d](bw:%u stall:%u delay:%u) ", i, \
						(ddr_info[cur].portdata[i].rdata_num * 16 * (1000000/g_monitor_poriod)) >> 20, \
						ddr_info[cur].portdata[i].raddr_cyc / ddr_info[cur].portdata[i].rdata_num, \
						ddr_info[cur].portdata[i].raddr_latency / ddr_info[cur].portdata[i].rdata_num);
				} else {
					length += sprintf(buf + length, "p[%d](bw:%u stall:%u delay:%u) ", i, 0, 0, 0);
				}
			}
			length += sprintf(buf + length, "ddrc:%u MB/s; ", (ddr_info[cur].rd_cmd_num * 64 * (1000000/g_monitor_poriod)) >> 20);
			length += sprintf(buf + length, "Write: ");
			for (i = 0; i < 6; i++) {
				if (ddr_info[cur].portdata[i].waddr_num) {
					length += sprintf(buf + length, "p[%d](bw:%u stall:%u delay:%u) ", i, \
						(ddr_info[cur].portdata[i].wdata_num * 16 * (1000000/g_monitor_poriod)) >> 20, \
						ddr_info[cur].portdata[i].waddr_cyc / ddr_info[cur].portdata[i].wdata_num, \
						ddr_info[cur].portdata[i].waddr_latency / ddr_info[cur].portdata[i].wdata_num);
				} else {
					length += sprintf(buf + length, "p[%d](bw:%u stall:%u delay:%u) ", i, 0, 0, 0);
				}
			}
				length += sprintf(buf + length, "ddrc %u MB/s, mask %u MB/s\n", (ddr_info[cur].wr_cmd_num * 64 * (1000000/g_monitor_poriod)) >> 20, (ddr_info[cur].mwr_cmd_num * 64 * (1000000/g_monitor_poriod)) >> 20);
		}
	}
	return length;
}

static int dbg_ddr_monitor_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_ddr_monitor_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open = dbg_ddr_monitor_open,
	.read = seq_read,
	.write = ddr_monitor_debug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init x2_ddr_monitor_debuginit(void)
{
	(void)debugfs_create_file("x2_ddrmonitor", S_IRUGO, NULL, NULL, &debug_fops);
	return 0;
}

typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define DDR_MONITOR_READ	_IOWR('m', 0, reg_t)
#define DDR_MONITOR_WRITE	_IOW('m', 1, reg_t)
#define DDR_MONITOR_CUR	_IOWR('m', 2, struct ddr_monitor_result_s)

static int ddr_monitor_mod_open(struct inode *pinode, struct file *pfile)
{
	printk(KERN_INFO "ddr_monitor_mod_open()!\n");
	ddr_monitor_start();
	return 0;
}

static int ddr_monitor_mod_release(struct inode *pinode, struct file *pfile)
{
	printk(KERN_INFO "ddr_monitor_mod_release()!\n");
	ddr_monitor_stop();
	return 0;
}

static ssize_t ddr_monitor_mod_read(struct file *pfile, char *puser_buf, size_t len, loff_t *poff)
{
	int result_len = 0;
	wait_event_interruptible(g_ddr_monitor_dev->wq_head, g_record_num > 200);
	result_len = get_monitor_data(result_buf);
	//if( result_len < len)
		//copy_to_user(puser_buf, result_buf, result_len);
	//else
	//	printk("buf not enough");
	return result_len;
}

static ssize_t ddr_monitor_mod_write(struct file *pfile, const char *puser_buf, size_t len, loff_t *poff)
{
	printk(KERN_INFO "ddr_monitor_mod_write()!\n");
	return 0;
}

static long ddr_monitor_mod_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	void __iomem *iomem = g_ddr_monitor_dev->regaddr;
	reg_t reg;
	if ( NULL == iomem ) {
		printk(KERN_ERR "x2 ddr_monitor no iomem\n");
		return -EINVAL;
	}
	switch (cmd) {
	case DDR_MONITOR_READ:
		if (!arg) {
			printk(KERN_ERR "x2 ddr_monitor reg read error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR "x2 ddr_monitor reg read error, copy data from user failed\n");
			return -EINVAL;
		}
		reg.value = readl(iomem + reg.offset);
		if ( copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg)) ) {
			printk(KERN_ERR "x2 ddr_monitor reg read error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case DDR_MONITOR_WRITE:
		if ( !arg ) {
			printk(KERN_ERR "x2 ddr_monitor reg write error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR "x2 ddr_monitor reg write error, copy data from user failed\n");
			return -EINVAL;
		}
		//printk("addr:0x%x, value:0x%x",iomem + reg.offset, reg.value);
		writel(reg.value, iomem + reg.offset );
		break;
	case DDR_MONITOR_CUR:
		{
			int cur = 0;
			if (!arg) {
				printk(KERN_ERR "x2 ddr_monitor get cur error\n");
				return -EINVAL;
			}
			cur  = (g_current_index - 1 + TOTAL_RECORD_NUM) % TOTAL_RECORD_NUM;
			if ( copy_to_user((void __user *)arg, (void *)(ddr_info + cur), sizeof(struct ddr_monitor_result_s)) ) {
				printk(KERN_ERR "x2 ddr_monitor get cur error, copy data to user failed\n");
				return -EINVAL;
			}
		}
		break;
	default:

		break;
	}
	return 0;
}

int ddr_monitor_mmap(struct file *filp, struct vm_area_struct *pvma)
{

	printk("ddr_monitor_mmap! \n");

	if (remap_pfn_range(pvma, pvma->vm_start,
						virt_to_pfn(g_ddr_monitor_dev->res_vaddr),
						pvma->vm_end - pvma->vm_start,
						pvma->vm_page_prot)) {
		printk(KERN_ERR "ddr_monitor_mmap fail\n");
		return -EAGAIN;
	}
	printk("ddr_monitor_mmap end!:%p \n", g_ddr_monitor_dev->res_vaddr);
	return 0;
}

struct file_operations ddr_monitor_mod_fops = {
	.owner			= THIS_MODULE,
	.mmap 			= ddr_monitor_mmap,
	.open			= ddr_monitor_mod_open,
	.read			= ddr_monitor_mod_read,
	.write			= ddr_monitor_mod_write,
	.release		= ddr_monitor_mod_release,
	.unlocked_ioctl = ddr_monitor_mod_ioctl,
};

int ddr_monitor_cdev_create(void)
{
	int ret = 0;
	int error;

	printk(KERN_INFO "ddr_monitor_cdev_create()\n");

	g_ddr_monitor_dev->ddr_monitor_classes = class_create(THIS_MODULE, "x2_ddr_monitor");
	if (IS_ERR(g_ddr_monitor_dev->ddr_monitor_classes))
		return PTR_ERR(g_ddr_monitor_dev->ddr_monitor_classes);

	error = alloc_chrdev_region(&g_ddr_monitor_dev->dev_num, 0, 1, "x2_ddr_monitor");

	if (!error) {
		g_ddr_monitor_dev->major = MAJOR(g_ddr_monitor_dev->dev_num);
		g_ddr_monitor_dev->minor = MINOR(g_ddr_monitor_dev->dev_num);
	}

	if (ret < 0)
		return ret;

	cdev_init(&g_ddr_monitor_dev->cdev, &ddr_monitor_mod_fops);
	g_ddr_monitor_dev->cdev.owner	= THIS_MODULE;

	cdev_add(&g_ddr_monitor_dev->cdev, g_ddr_monitor_dev->dev_num, 1);

	device_create(g_ddr_monitor_dev->ddr_monitor_classes, NULL, g_ddr_monitor_dev->dev_num, NULL, "x2_ddrmonitor");
	if (ret)
		return ret;

	return ret;
}

void ddr_monitor_dev_remove(void)
{
	printk(KERN_INFO "ddr_monitor_dev_remove()\n");

	cdev_del(&g_ddr_monitor_dev->cdev);

	unregister_chrdev_region(g_ddr_monitor_dev->dev_num, 1);
}

int ddr_monitor_start(void)
{
	if (g_ddr_monitor_dev && !ddr_info) {
		printk("ddr_monitor_start\n");
		//enable_irq(g_ddr_monitor_dev->irq);
		ddr_info = vmalloc(sizeof(struct ddr_monitor_result_s) * TOTAL_RECORD_NUM);
		result_buf = g_ddr_monitor_dev->res_vaddr;//vmalloc(1024*80);
		g_current_index = 0;
		g_record_num = 0;
		//writel(0x51616 * g_monitor_poriod, g_ddr_monitor_dev->regaddr + PERF_MONITOR_PERIOD);
		writel(g_monitor_poriod * 1000 /3, g_ddr_monitor_dev->regaddr + PERF_MONITOR_PERIOD);
		writel(0x1, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE_UNMASK);
		writel(0xff, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE);
		g_ktime_start = ktime_get();
	}
	return 0;
}

int ddr_monitor_stop(void)
{
	if (g_ddr_monitor_dev && ddr_info) {
		printk("ddr_monitor_stop\n");
		//disable_irq(g_ddr_monitor_dev->irq);
		writel(0, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE);
		writel(0x1, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE_SETMASK);
		vfree(ddr_info);
		//vfree(result_buf);
		result_buf = NULL;
		ddr_info = NULL;
	}
	return 0;
}

int ddr_get_port_status(void)
{
	int i = 0;
	ktime_t ktime;
	ktime = ktime_sub(ktime_get(), g_ktime_start);
	ddr_info[g_current_index].curtime = ktime_to_us(ktime);
	for (i = 0; i < 6; i++) {
		ddr_info[g_current_index].portdata[i].raddr_num = readl(g_ddr_monitor_dev->regaddr + MP_BASE_RADDR_TX_NUM + i * MP_REG_OFFSET);
		ddr_info[g_current_index].portdata[i].rdata_num = readl(g_ddr_monitor_dev->regaddr + MP_BASE_RDATA_TX_NUM + i * MP_REG_OFFSET);
		ddr_info[g_current_index].portdata[i].raddr_cyc = readl(g_ddr_monitor_dev->regaddr + MP_BASE_RADDR_ST_CYC + i * MP_REG_OFFSET);
		ddr_info[g_current_index].portdata[i].raddr_latency = readl(g_ddr_monitor_dev->regaddr + MP_BASE_RA2LSTRD_LATENCY + i * MP_REG_OFFSET);

		ddr_info[g_current_index].portdata[i].waddr_num = readl(g_ddr_monitor_dev->regaddr + MP_BASE_WADDR_TX_NUM + i * MP_REG_OFFSET);
		ddr_info[g_current_index].portdata[i].wdata_num = readl(g_ddr_monitor_dev->regaddr + MP_BASE_WDATA_TX_NUM + i * MP_REG_OFFSET);
		ddr_info[g_current_index].portdata[i].waddr_cyc = readl(g_ddr_monitor_dev->regaddr + MP_BASE_WADDR_ST_CYC + i * MP_REG_OFFSET);
		ddr_info[g_current_index].portdata[i].waddr_latency = readl(g_ddr_monitor_dev->regaddr + MP_BASE_WA2BRESP_LATENCY + i * MP_REG_OFFSET);
	}

	ddr_info[g_current_index].rd_cmd_num = readl(g_ddr_monitor_dev->regaddr + RD_CMD_TX_NUM);
	ddr_info[g_current_index].wr_cmd_num= readl(g_ddr_monitor_dev->regaddr + WR_CMD_TX_NUM);
	ddr_info[g_current_index].mwr_cmd_num = readl(g_ddr_monitor_dev->regaddr + MWR_CMD_TX_NUM);
	ddr_info[g_current_index].rdwr_swi_num = readl(g_ddr_monitor_dev->regaddr + RDWR_SWITCH_NUM);
	ddr_info[g_current_index].act_cmd_num = readl(g_ddr_monitor_dev->regaddr + ACT_CMD_TX_NUM);
	ddr_info[g_current_index].act_cmd_rd_num = readl(g_ddr_monitor_dev->regaddr + ACT_CMD_TX_FOR_RD_TX_NUM);
	ddr_info[g_current_index].war_haz_num = readl(g_ddr_monitor_dev->regaddr + WAR_HAZARD_NUM);
	ddr_info[g_current_index].raw_haz_num = readl(g_ddr_monitor_dev->regaddr + RAW_HAZARD_NUM);
	ddr_info[g_current_index].waw_haz_num = readl(g_ddr_monitor_dev->regaddr + WAW_HAZARD_NUM);
	ddr_info[g_current_index].per_cmd_num = readl(g_ddr_monitor_dev->regaddr + PERCHARGE_CMD_TX_NUM);
	ddr_info[g_current_index].per_cmd_rdwr_num = readl(g_ddr_monitor_dev->regaddr + PERCHARGE_CMD_FOR_RDWR_TX_NUM);

	//ddr_info[g_current_index].curtime = jiffies;
	g_current_index = (g_current_index + 1) % TOTAL_RECORD_NUM;
	g_record_num ++;

	if (g_record_num >= 200)
		wake_up_interruptible(&g_ddr_monitor_dev->wq_head);
	return 0;
}

static irqreturn_t ddr_monitor_isr(int this_irq, void *data)
{
	//printk("ddrisr\n");
	writel(0x1, g_ddr_monitor_dev->regaddr + PERF_MONITOR_SRCPND);
	ddr_get_port_status();
	return IRQ_HANDLED;
}
static unsigned int read_ctl_value;
static unsigned int write_ctl_value;
static ssize_t cpu_read_ctl_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~0x0f;
	tmp |= read_ctl_value;
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t cpu_read_ctl_show(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bifdma_read_ctl_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 4);
	tmp |= (read_ctl_value << 4);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bifdma_read_ctl_show(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 4;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bpu0_read_ctl_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 8);
	tmp |= (read_ctl_value << 8);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bpu0_read_ctl_show(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 8;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}

static ssize_t bpu1_read_ctl_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 12);
	tmp |= (read_ctl_value << 12);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bpu1_read_ctl_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 12;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t vio_read_ctl_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 16);
	tmp |= (read_ctl_value << 16);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t vio_read_ctl_show(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 16;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t periph_read_ctl_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 20);
	tmp |= (read_ctl_value << 20);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t periph_read_ctl_show(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 20;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t all_read_ctl_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = 0;
	tmp = read_ctl_value | (read_ctl_value << 4) |
		(read_ctl_value << 8) | (read_ctl_value << 12) |
		(read_ctl_value << 16) | (read_ctl_value << 20);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t all_read_ctl_show(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	return sprintf(buf, "%x\n", tmp);
}


static struct kobj_attribute cpu_read_ctl = __ATTR(cpu, 0664,
						   cpu_read_ctl_show,
						   cpu_read_ctl_store);
static struct kobj_attribute bifdma_read_ctl = __ATTR(bifdma, 0664,
						      bifdma_read_ctl_show,
						      bifdma_read_ctl_store);
static struct kobj_attribute bpu0_read_ctl = __ATTR(bpu0, 0664,
						    bpu0_read_ctl_show,
						    bpu0_read_ctl_store);
static struct kobj_attribute bpu1_read_ctl = __ATTR(bpu1, 0664,
						    bpu1_read_ctl_show,
						    bpu1_read_ctl_store);
static struct kobj_attribute vio_read_ctl    = __ATTR(vio, 0664,
						      vio_read_ctl_show,
						      vio_read_ctl_store);
static struct kobj_attribute periph_read_ctl = __ATTR(peripheral, 0664,
						      periph_read_ctl_show,
						      periph_read_ctl_store);
static struct kobj_attribute all_read_ctl    = __ATTR(all, 0664,
						      all_read_ctl_show,
						      all_read_ctl_store);

static struct attribute *read_qctrl_attrs[] = {
	&cpu_read_ctl.attr,
	&bifdma_read_ctl.attr,
	&bpu0_read_ctl.attr,
	&bpu1_read_ctl.attr,
	&vio_read_ctl.attr,
	&periph_read_ctl.attr,
	&all_read_ctl.attr,
	NULL,
};


static ssize_t cpu_write_ctl_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~0x0f;
	tmp |= write_ctl_value;
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t cpu_write_ctl_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bifdma_write_ctl_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 4);
	tmp |= (write_ctl_value << 4);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bifdma_write_ctl_show(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 4;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bpu0_write_ctl_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 8);
	tmp |= (write_ctl_value << 8);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bpu0_write_ctl_show(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 8;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}

static ssize_t bpu1_write_ctl_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 12);
	tmp |= (write_ctl_value << 12);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bpu1_write_ctl_show(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 12;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t vio_write_ctl_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 16);
	tmp |= (write_ctl_value << 16);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t vio_write_ctl_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 16;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t periph_write_ctl_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 20);
	tmp |= (write_ctl_value << 20);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t periph_write_ctl_show(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 20;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t all_write_ctl_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15) {
		pr_err("set value error,you should set 0~15\n");
		return 0;
	}
	tmp = 0;
	tmp = write_ctl_value | (write_ctl_value << 4) |
		(write_ctl_value << 8) | (write_ctl_value << 12) |
		(write_ctl_value << 16) | (write_ctl_value << 20);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t all_write_ctl_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	return sprintf(buf, "%x\n", tmp);
}


static struct kobj_attribute cpu_write_ctl = __ATTR(cpu, 0664,
						    cpu_write_ctl_show,
						    cpu_write_ctl_store);
static struct kobj_attribute bifdma_write_ctl = __ATTR(bifdma, 0664,
						       bifdma_write_ctl_show,
						       bifdma_write_ctl_store);
static struct kobj_attribute bpu0_write_ctl = __ATTR(bpu0, 0664,
						     bpu0_write_ctl_show,
						     bpu0_write_ctl_store);
static struct kobj_attribute bpu1_write_ctl = __ATTR(bpu1, 0664,
						     bpu1_write_ctl_show,
						     bpu1_write_ctl_store);
static struct kobj_attribute vio_write_ctl    = __ATTR(vio, 0664,
						       vio_write_ctl_show,
						       vio_write_ctl_store);
static struct kobj_attribute periph_write_ctl = __ATTR(peripheral, 0664,
						       periph_write_ctl_show,
						       periph_write_ctl_store);
static struct kobj_attribute all_write_ctl    = __ATTR(all, 0664,
						       all_write_ctl_show,
						       all_write_ctl_store);

static struct attribute *write_qctl_attrs[] = {
	&cpu_write_ctl.attr,
	&bifdma_write_ctl.attr,
	&bpu0_write_ctl.attr,
	&bpu1_write_ctl.attr,
	&vio_write_ctl.attr,
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
	NULL,
};
struct bus_type ddr_monitor_subsys = {
	.name = "ddr_monitor",
};

static int ddr_monitor_probe(struct platform_device *pdev)
{
	int ret = 0;
	//struct device_node *np = NULL;
	struct resource *pres;
	//struct resource mem_res;
	printk(KERN_INFO "ddr_monitor_probe()!");

	g_ddr_monitor_dev = devm_kmalloc(&pdev->dev, sizeof(struct ddr_monitor_dev_s), GFP_KERNEL);
	if (!g_ddr_monitor_dev) {
		return -ENOMEM;
	}
	memset(g_ddr_monitor_dev, 0, sizeof(struct ddr_monitor_dev_s));

	pres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pres) {
		devm_kfree(&pdev->dev, g_ddr_monitor_dev);
		return -ENOMEM;
	}

	g_ddr_monitor_dev->regaddr = devm_ioremap_resource(&pdev->dev, pres);
	g_ddr_monitor_dev->irq = platform_get_irq(pdev, 0);

	ret = request_irq(g_ddr_monitor_dev->irq, ddr_monitor_isr, IRQF_TRIGGER_HIGH,
			  dev_name(&pdev->dev), g_ddr_monitor_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		return -ENODEV;
	}
	#if 0
	/* request memory address */
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");
		return ret;
	}

	ret = of_address_to_resource(np, 0, &mem_res);
	if (ret) {
		dev_err(&pdev->dev, "No memory address assigned to the region\n");
		return -1;
	}
	g_ddr_monitor_dev->res_paddr = mem_res.start;
	g_ddr_monitor_dev->res_memsize = resource_size(&mem_res);
	g_ddr_monitor_dev->res_vaddr = memremap(mem_res.start, g_ddr_monitor_dev->res_memsize, MEMREMAP_WB);
	#endif
	g_ddr_monitor_dev->res_vaddr = kmalloc(TOTAL_RESULT_SIZE, GFP_KERNEL);
	if (g_ddr_monitor_dev->res_vaddr == NULL) {
		dev_err(&pdev->dev, "memory alloc fail\n");
		return -1;
	}
	platform_set_drvdata(pdev, g_ddr_monitor_dev);
	x2_ddr_monitor_debuginit();
	ddr_monitor_cdev_create();
	init_waitqueue_head(&g_ddr_monitor_dev->wq_head);
	spin_lock_init(&g_ddr_monitor_dev->lock);

	writel(0x21100, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	writel(0x21100, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);

	/*register bpu sys node*/
	if (subsys_system_register(&ddr_monitor_subsys, ddr_attr_groups))
		pr_err("fialed to register bpu subsystem");
	return ret;
}

static int ddr_monitor_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "ddr_monitor_remove()!\n");
	ddr_monitor_dev_remove();
	devm_kfree(&pdev->dev, g_ddr_monitor_dev);
	g_ddr_monitor_dev = NULL;

	return 0;
}

static const struct of_device_id ddr_monitor_match[] = {
	{.compatible = "hobot,x2-ddrmonitor"},
	{}
};

MODULE_DEVICE_TABLE(of, ddr_monitor_match);

static struct platform_driver ddr_monitor_driver = {
	.probe	= ddr_monitor_probe,
	.remove = ddr_monitor_remove,
	.driver = {
		.name	= "ddr_monitor",
		.of_match_table = ddr_monitor_match,
	},
};

module_platform_driver(ddr_monitor_driver);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("ddr performance monitor module");
MODULE_LICENSE("GPL");
