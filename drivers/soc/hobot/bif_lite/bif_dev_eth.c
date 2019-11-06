/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2019 Horizon Robotics, Inc.
 *			 All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "hbipc_lite.h"
#include "hbipc_errno.h"
#include "hbipc_eth.h"
#include "bif_dev_eth.h"

#define VERSION "2.9.0"
#define VERSION_LEN (16)
static char version_str[VERSION_LEN];

#ifdef CONFIG_NO_DTS_AP
/* module parameters */
static int frame_len_max_ap = 262144;
module_param(frame_len_max_ap, int, 0644);
#endif

/* ioctl cmd */
#define BIF_IOC_MAGIC  'c'
#define BIF_IO_PROVIDER_INIT     _IO(BIF_IOC_MAGIC, 0)
#define BIF_IO_PROVIDER_DEINIT   _IO(BIF_IOC_MAGIC, 1)
#define BIF_IO_ACCEPT            _IO(BIF_IOC_MAGIC, 2)
#define BIF_IO_START_SERVER      _IO(BIF_IOC_MAGIC, 3)
#define BIF_IO_STOP_SERVER       _IO(BIF_IOC_MAGIC, 4)
#define BIF_IO_CONNECT           _IO(BIF_IOC_MAGIC, 5)
#define BIF_IO_DISCONNECT        _IO(BIF_IOC_MAGIC, 6)
#define BIF_IO_SET_USR_TIMEOUT   _IO(BIF_IOC_MAGIC, 7)
#define BIF_IO_GET_FRAME_LIMIT   _IO(BIF_IOC_MAGIC, 8)
#define BIF_IO_SET_LIB_VERSION   _IO(BIF_IOC_MAGIC, 9)

//#define WORK_COUNT (100)
struct x2_bif_data {
	int users;
	char *send_frame;
	struct task_struct *recv_task;
};
static struct x2_bif_data bif_data;

struct transfer_feature {
	int block;
	int usr_timeout;
};

static struct class  *g_bif_class;
static struct device *g_bif_dev;

static struct domain_info domain_config = {.domain_name = "X2ETH001",
	.domain_id = X2ETH,
	.device_name = "/dev/x2_eth",
	.type = SOC_AP,
	.mode = INTERRUPT_MODE,
	.crc_enable = 1,
	{.channel = ETHERNET,
	.type = SOC_AP,
	.mode = INTERRUPT_MODE}
};

static struct comm_domain domain;

/* proc debug fs */
#define BIF_DEV_ETH_DIR "bif_dev_eth"
#define BIF_DEV_ETH_STATTISTICS "statistics"
#define BIF_DEV_ETH_INFO "info"
#define BIF_DEV_ETH_SERVER_INFO "server_info"
#define BIF_DEV_ETH_ERROR "error_statistics"
static struct proc_dir_entry *bif_dev_eth_entry;
static struct proc_dir_entry *bif_dev_eth_statistics_entry;
static struct proc_dir_entry *bif_dev_eth_info_entry;
static struct proc_dir_entry *bif_dev_eth_server_info_entry;
static struct proc_dir_entry *bif_dev_eth_error_statistics_entry;

static int bif_dev_eth_statistics_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "bif_dev_eth dev layer statistics:\n\
irq_handler_count = %d\nrx_work_func_count = %d\n\
rx_flowcontrol_count = %d\naccept_count = %d\n\
write_call_count = %d\nwrite_real_count = %d\n\
read_call_count = %d\nread_real_count = %d\n\
write_resend_count = %d\nwrite_resend_over_count = %d\n\
bif_dev_eth hbipc layer statistics:\n\
interrupt_recv_count = %d\nmanage_recv_count = %d\n\
data_recv_count = %d\nmanage_frame_count = %d\n\
data_frame_count = %d\nup_sem_count = %d\n\
send_manage_count = %d\n\
mang_resend_count = %d\nmang_resend_over_count = %d\n\
concede_manage_send_count = %d\nconcede_data_send_count = %d\n\
concede_data_recv_count = %d\n\
bif_dev_eth transfer layer statistics:\n\
trig_count = %d\nretrig_count = %d\n",
	domain.domain_statistics.irq_handler_count,
	domain.domain_statistics.rx_work_func_count,
	domain.domain_statistics.rx_flowcontrol_count,
	domain.domain_statistics.accept_count,
	domain.domain_statistics.write_call_count,
	domain.domain_statistics.write_real_count,
	domain.domain_statistics.read_call_count,
	domain.domain_statistics.read_real_count,
	domain.domain_statistics.write_resend_count,
	domain.domain_statistics.write_resend_over_count,
	domain.domain_statistics.interrupt_recv_count,
	domain.domain_statistics.manage_recv_count,
	domain.domain_statistics.data_recv_count,
	domain.domain_statistics.manage_frame_count,
	domain.domain_statistics.data_frame_count,
	domain.domain_statistics.up_sem_count,
	domain.domain_statistics.send_manage_count,
	domain.domain_statistics.mang_resend_count,
	domain.domain_statistics.mang_resend_over_count,
	domain.domain_statistics.concede_manage_send_count,
	domain.domain_statistics.concede_data_send_count,
	domain.domain_statistics.concede_data_recv_count,
	domain.channel.channel_statistics.trig_count,
	domain.channel.channel_statistics.retrig_count);

	return 0;
}

static int bif_dev_eth_info_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "kernel version = %s\n"
"user lib version = %s\n"
"buffer index:\n"
"init_tx_remote_info = %d\ninit_tx_local_info = %d\n"
"init_rx_local_info = %d\ninit_rx_remote_info = %d\n"
"sync_tx_remote_info = %d\nsync_tx_local_info = %d\n"
"sync_rx_local_info = %d\nsync_rx_remote_info = %d\n"
"hardware channel concerned\n"
"channel = %d\nbuffer_id = %d\ntransfer_align = %d\n"
"memory limit concerned:\n"
#if __SIZEOF_POINTER__ == 4
"base_addr = %x\nframe_len_max = %d\nfrag_len_max = %d\n"
#else
"base_addr = %lx\nframe_len_max = %d\nfrag_len_max = %d\n"
#endif
"valid_frag_len_max = %d\nrag_num = %d\nframe_cache_max = %d\n"
"memory layout concerned:\n"
#if __SIZEOF_POINTER__ == 4
"rx_local_info_offset = %x\nrx_remote_info_offset = %x\n"
"tx_local_info_offset = %x\ntx_remote_info_offset = %x\n"
"rx_buffer_offset = %x\ntx_buffer_offset = %x\n"
#else
"rx_local_info_offset = %lx\nrx_remote_info_offset = %lx\n"
"tx_local_info_offset = %lx\ntx_remote_info_offset = %lx\n"
"rx_buffer_offset = %lx\ntx_buffer_offset = %lx\n"
#endif
"total_mem_size = %d\n"
"transfer feature:\n"
"ap_type = %d\nworking_mode = %d\n"
"crc_enable = %d\n",
	VERSION,
	version_str,
	domain.channel.init_tx_remote_info,
	domain.channel.init_tx_local_info,
	domain.channel.init_rx_local_info,
	domain.channel.init_rx_remote_info,
	domain.channel.sync_tx_remote_info,
	domain.channel.sync_tx_local_info,
	domain.channel.sync_rx_local_info,
	domain.channel.sync_rx_remote_info,
	domain.channel.channel,
	domain.channel.buffer_id,
	domain.channel.transfer_align,
	domain.channel.base_addr,
	domain.channel.frame_len_max,
	domain.channel.frag_len_max,
	domain.channel.valid_frag_len_max,
	domain.channel.frag_num,
	domain.channel.frame_cache_max,
	domain.channel.rx_local_info_offset,
	domain.channel.rx_remote_info_offset,
	domain.channel.tx_local_info_offset,
	domain.channel.tx_remote_info_offset,
	domain.channel.rx_buffer_offset,
	domain.channel.tx_buffer_offset,
	domain.channel.total_mem_size,
	domain.channel.type,
	domain.channel.mode,
	domain.channel.crc_enable);

	return 0;
}

static int bif_dev_eth_server_info_proc_show(struct seq_file *m, void *v)
{
	struct provider_server_map *map = &domain.map;
	struct provider_server *relation = NULL;
	int i = 0;
	int j = 0;
	short int *provider_id_factor = NULL;
	int pid = 0;

	mutex_lock(&domain.connect_mutex);

	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i) {
		if (map->map_array[i].valid) {
			relation = map->map_array + i;
			seq_printf(m, "server_id:\n");
			for (j = 0; j < 16; ++j)
				seq_printf(m, "%x ", relation->server_id[j]);
			seq_printf(m, "\n");
			seq_printf(m, "provider_id:\n");
			seq_printf(m, "%d\n", relation->provider_id);
			provider_id_factor = (short int *)(relation->server_id);
			pid = relation->provider_id - *provider_id_factor;
			seq_printf(m, "pid:\n");
			seq_printf(m, "%d\n", pid);
		}
	}

	mutex_unlock(&domain.connect_mutex);

	return 0;
}

static int bif_dev_eth_error_statistics_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "rx_error_assemble_frag = %d\n\
rx_error_crc_check = %d\nrx_error_malloc_frame = %d\n\
rx_error_no_frag = %d\nrx_error_read_frag = %d\n\
rx_error_sync_index = %d\nrx_error_update_index = %d\n\
rx_error_drop_frag_count = %d\n",
	domain.channel.error_statistics.rx_error_assemble_frag,
	domain.channel.error_statistics.rx_error_crc_check,
	domain.channel.error_statistics.rx_error_malloc_frame,
	domain.channel.error_statistics.rx_error_no_frag,
	domain.channel.error_statistics.rx_error_read_frag,
	domain.channel.error_statistics.rx_error_sync_index,
	domain.channel.error_statistics.rx_error_update_index,
	domain.channel.error_statistics.rx_error_drop_frag_count
	);

	return 0;
}

static ssize_t bif_dev_eth_statistics_proc_write(struct file *file,
const char __user *buffer, size_t count, loff_t *ppos)
{
	memset(&(domain.domain_statistics), 0,
	sizeof(domain.domain_statistics));
	memset(&(domain.channel.channel_statistics), 0,
		sizeof(domain.channel.channel_statistics));

	return count;
}

static ssize_t bif_dev_eth_info_proc_write(struct file *file,
const char __user *buffer, size_t count, loff_t *ppos)
{
	return count;
}

static ssize_t bif_dev_eth_server_info_proc_write(struct file *file,
const char __user *buffer, size_t count, loff_t *ppos)
{
	return count;
}

static ssize_t bif_dev_eth_error_statistics_proc_write(struct file *file,
const char __user *buffer, size_t count, loff_t *ppos)
{
	memset(&(domain.channel.error_statistics), 0,
		sizeof(domain.channel.error_statistics));

	return count;
}

static int bif_dev_eth_statistics_proc_open(struct inode *inode,
struct file *file)
{
	return single_open(file, bif_dev_eth_statistics_proc_show, NULL);
}

static int bif_dev_eth_info_proc_open(struct inode *inode,
struct file *file)
{
	return single_open(file, bif_dev_eth_info_proc_show, NULL);
}

static int bif_dev_eth_server_info_proc_open(struct inode *inode,
struct file *file)
{
	return single_open(file, bif_dev_eth_server_info_proc_show, NULL);
}

static int bif_dev_eth_error_statistics_proc_open(struct inode *inode,
struct file *file)
{
	return single_open(file, bif_dev_eth_error_statistics_proc_show, NULL);
}

static const struct file_operations bif_dev_eth_statistics_proc_ops = {
	.owner    = THIS_MODULE,
	.open     = bif_dev_eth_statistics_proc_open,
	.read     = seq_read,
	.write    = bif_dev_eth_statistics_proc_write,
	.llseek   = seq_lseek,
	.release  = single_release,
};

static const struct file_operations bif_dev_eth_info_proc_ops = {
	.owner    = THIS_MODULE,
	.open     = bif_dev_eth_info_proc_open,
	.read     = seq_read,
	.write    = bif_dev_eth_info_proc_write,
	.llseek   = seq_lseek,
	.release  = single_release,
};

static const struct file_operations bif_dev_eth_server_info_proc_ops = {
	.owner    = THIS_MODULE,
	.open     = bif_dev_eth_server_info_proc_open,
	.read     = seq_read,
	.write    = bif_dev_eth_server_info_proc_write,
	.llseek   = seq_lseek,
	.release  = single_release,
};

static const struct file_operations bif_dev_eth_error_statistics_proc_ops = {
	.owner    = THIS_MODULE,
	.open     = bif_dev_eth_error_statistics_proc_open,
	.read     = seq_read,
	.write    = bif_dev_eth_error_statistics_proc_write,
	.llseek   = seq_lseek,
	.release  = single_release,
};

static int init_bif_dev_eth_debug_port(void)
{
	bif_dev_eth_entry = proc_mkdir(BIF_DEV_ETH_DIR, NULL);
	if (!bif_dev_eth_entry) {
		pr_info("create /proc/%s fail\n", BIF_DEV_ETH_DIR);
		goto create_top_dir_error;
	}

	bif_dev_eth_statistics_entry = proc_create(BIF_DEV_ETH_STATTISTICS,
	0777, bif_dev_eth_entry, &bif_dev_eth_statistics_proc_ops);
	if (!bif_dev_eth_statistics_entry) {
		pr_info("create /proc/%s/%s fail\n", BIF_DEV_ETH_DIR,
			BIF_DEV_ETH_STATTISTICS);
		goto create_statistics_file_error;
	}

	bif_dev_eth_info_entry = proc_create(BIF_DEV_ETH_INFO,
	0777, bif_dev_eth_entry, &bif_dev_eth_info_proc_ops);
	if (!bif_dev_eth_info_entry) {
		pr_info("create /proc/%s/%s fail\n", BIF_DEV_ETH_DIR,
			BIF_DEV_ETH_INFO);
		goto create_info_file_error;
	}

	bif_dev_eth_server_info_entry = proc_create(BIF_DEV_ETH_SERVER_INFO,
	0777, bif_dev_eth_entry, &bif_dev_eth_server_info_proc_ops);
	if (!bif_dev_eth_server_info_entry) {
		pr_info("create /proc/%s/%s fail\n", BIF_DEV_ETH_DIR,
			BIF_DEV_ETH_SERVER_INFO);
		goto create_server_info_file_error;
	}

	bif_dev_eth_error_statistics_entry = proc_create(BIF_DEV_ETH_ERROR,
	0777, bif_dev_eth_entry, &bif_dev_eth_error_statistics_proc_ops);
	if (!bif_dev_eth_error_statistics_entry) {
		pr_info("create /proc/%s/%s fail\n", BIF_DEV_ETH_DIR,
			BIF_DEV_ETH_ERROR);
		goto create_error_statistics_file_error;
	}

	return 0;
create_error_statistics_file_error:
	remove_proc_entry(BIF_DEV_ETH_SERVER_INFO, bif_dev_eth_entry);
create_server_info_file_error:
	remove_proc_entry(BIF_DEV_ETH_INFO, bif_dev_eth_entry);
create_info_file_error:
	remove_proc_entry(BIF_DEV_ETH_STATTISTICS, bif_dev_eth_entry);
create_statistics_file_error:
	remove_proc_entry(BIF_DEV_ETH_DIR, NULL);
create_top_dir_error:
	return -1;
}

static void remove_bif_dev_eth_debug_port(void)
{
	remove_proc_entry(BIF_DEV_ETH_ERROR, bif_dev_eth_entry);
	remove_proc_entry(BIF_DEV_ETH_SERVER_INFO, bif_dev_eth_entry);
	remove_proc_entry(BIF_DEV_ETH_INFO, bif_dev_eth_entry);
	remove_proc_entry(BIF_DEV_ETH_STATTISTICS, bif_dev_eth_entry);
	remove_proc_entry(BIF_DEV_ETH_DIR, NULL);
}

static int recv_thread_stop;
static int hbeth_recv_thread(void *data)
{
	int ret = 0;
	int surplus_frame = 0;
	unsigned long remaining_time = 0;

	pr_info("%s start.......\n", __func__);
	while (1) {
		if (recv_thread_stop) {
			pr_info("recv_thread stop\n");
			break;
		}

		ret = recv_frame_eth(&domain);
		if (ret < 0) {
wait_for_connect:
			if (!hbeth_check_ready()) {
				msleep(100);
				if (recv_thread_stop)
					break;
				else
					goto wait_for_connect;
			}
		}
thread_flow_control:
		surplus_frame = domain_stock_frame_num(&domain);
		if (surplus_frame > domain.channel.frame_cache_max) {
			++domain.domain_statistics.rx_flowcontrol_count;
			remaining_time = msleep_interruptible(20);
			if (!remaining_time)
				goto thread_flow_control;
			else
				pr_notice("thread_flow_control interruptible\n");
		}
	}

	return 0;
}

static void x2_mem_layout_set(struct domain_info *domain_inf)
{
	domain_inf->channel_cfg.frame_len_max = FRAME_LEN_MAX;
	domain_inf->channel_cfg.frame_cache_max = FRAME_CACHE_MAX;
}

static int x2_bif_data_init(struct x2_bif_data *bif_data)
{
	bif_data->users = 0;

	bif_data->send_frame = kmalloc(FRAME_LEN_MAX, GFP_KERNEL);
	if (!(bif_data->send_frame)) {
		pr_info("malloc_send_frame error\n");
		goto malloc_send_frame_error;
	}

	if (init_bif_dev_eth_debug_port() < 0) {
		pr_info("init_debug_port error\n");
		goto init_debug_port_error;
	}

	return 0;
init_debug_port_error:
	kfree(bif_data->send_frame);
malloc_send_frame_error:
	return -1;
}

static void x2_bif_data_deinit(struct x2_bif_data *bif_data)
{
	bif_data->users = 0;
	remove_bif_dev_eth_debug_port();
	recv_thread_stop = 1;
	kfree(bif_data->send_frame);
}

static DEFINE_MUTEX(open_mutex);
#define CONNECT_CHECK_MAX (5)
#define MULTIPLE_FACTOR (20)
static int bif_lite_init_success;
static int recv_thread_start;
static int x2_bif_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct transfer_feature *feature = NULL;
	int i = 0;
#ifdef CONFIG_HOBOT_BIF_AP
	struct send_mang_data data;
#endif

	mutex_lock(&open_mutex);

	feature = kmalloc(sizeof(struct transfer_feature), GFP_KERNEL);
	if (!feature) {
		pr_info("malloc transfer feature error\n");
		ret = -ENOMEM;
		goto err;
	}

	memset(feature, 0, sizeof(struct transfer_feature));
	if (file->f_flags & O_NONBLOCK)
		feature->block = 0;
	else
		feature->block = 1;

	if (!bif_data.users) {
		file->private_data = feature;
		if (!bif_lite_init_success) {
		#ifdef CONFIG_HOBOT_BIF_AP
			// check connect status
			hbeth_ap_start_connect();
			for (i = 0; i < CONNECT_CHECK_MAX * MULTIPLE_FACTOR; ++i) {
				if (hbeth_check_ready())
					break;
				else
					msleep(100);
			}
			if (i < CONNECT_CHECK_MAX * MULTIPLE_FACTOR) {
				// create & start recv thread
				if (!recv_thread_start) {
					bif_data.recv_task = kthread_create(hbeth_recv_thread, NULL, "hbeth_recv_thread");
					if (IS_ERR(bif_data.recv_task)) {
						pr_info("kthread_create error\n");
						ret = -EPERM;
						goto err;
					}
					wake_up_process(bif_data.recv_task);
					recv_thread_start = 1;
				}

				// send query server manage frame
				data.domain_id = domain.domain_id;
				ret = mang_frame_send2opposite(&domain,
					MANAGE_CMD_QUERY_SERVER, &data);
				if (ret < 0) {
					pr_info("send query server message error\n");
					ret = -EPERM;
					goto err;
				}
				bif_lite_init_success = 1;
			} else {
				pr_err("hbeth_check_ready error\n");
				ret = -EPERM;
				goto err;
			}
		#else
			// bind socket
			ret = hbeth_bind_sock();
			if (ret < 0) {
				pr_err("hbeth_bind_sock error\n");
				ret = -EPERM;
				goto err;
			} else {
				// check connect status
				for (i = 0; i < CONNECT_CHECK_MAX * MULTIPLE_FACTOR; ++i) {
					if (hbeth_check_ready())
						break;
					else
						msleep(100);
				}
				if (i < CONNECT_CHECK_MAX * MULTIPLE_FACTOR) {
					// start recv thread
					if (!recv_thread_start) {
						bif_data.recv_task = kthread_create(hbeth_recv_thread, NULL, "hbeth_recv_thread");
						if (IS_ERR(bif_data.recv_task)) {
							pr_info("kthread_create error\n");
							ret = -EPERM;
							goto err;
						}
						wake_up_process(bif_data.recv_task);
						recv_thread_start = 1;
					}
					bif_lite_init_success = 1;
				} else {
					pr_err("hbeth_check_ready error\n");
					ret = -EPERM;
					goto err;
				}
			}
		#endif
			++bif_data.users;
		} else {
			// check connect status
			for (i = 0; i < CONNECT_CHECK_MAX; ++i) {
				if (hbeth_check_ready())
					break;
				else
					msleep(100);
			}
			if (i >= CONNECT_CHECK_MAX) {
				pr_err("hbeth_check_ready error\n");
				ret = -EPERM;
				goto err;
			}
			++bif_data.users;
		}
	} else {
		file->private_data = feature;
		// check connect status
		for (i = 0; i < CONNECT_CHECK_MAX; ++i) {
			if (hbeth_check_ready())
				break;
			else
				msleep(100);
		}
		if (i >= CONNECT_CHECK_MAX) {
			pr_err("hbeth_check_ready error\n");
			ret = -EPERM;
			goto err;
		}
		++bif_data.users;
	}

	mutex_unlock(&open_mutex);

	return 0;
err:
	mutex_unlock(&open_mutex);
	return ret;
}

static DEFINE_MUTEX(write_mutex);
static ssize_t x2_bif_write(struct file *file, const char __user *buf,
size_t count, loff_t *ppos)
{
	int ret = 0;
	struct send_mang_data data;
	int status = 0;
	#if __SIZEOF_POINTER__ == 4
	unsigned int addr_low = 0;
	#endif
	// just for debug
	struct hbipc_header *header = NULL;

	++domain.domain_statistics.write_call_count;

	mutex_lock(&write_mutex);

	if (copy_from_user(&data, (const char __user *)buf, sizeof(data))) {
		hbipc_error("copy_from_user_fail\n");
		ret = -EFAULT;
		goto error;
	}

	if (!is_valid_session(&domain, &data, NULL, NULL)) {
		hbipc_error("invalid session: %d_%d_%d\n", data.domain_id,
			data.provider_id, data.client_id);
		ret = -1;
		data.result = HBIPC_ERROR_INVALID_SESSION;
		status = copy_to_user((void __user *)buf, &data, sizeof(data));
		if (status)
			ret = -EFAULT;
		goto error;
	}

	#if __SIZEOF_POINTER__ == 4
	addr_low = (unsigned int)data.buffer;
	status = copy_from_user(bif_data.send_frame,
	(const char __user *)addr_low,
	data.len);
	#else
	status = copy_from_user(bif_data.send_frame,
	(const char __user *)data.buffer,
	data.len);
	#endif

	if (status) {
		hbipc_error("copy user frame error\n");
		ret = -EFAULT;
		goto error;
	}

	// concede manage frame
	if (domain.manage_send) {
		++domain.domain_statistics.concede_manage_send_count;
		msleep_interruptible(5);
	}

	mutex_lock(&domain.write_mutex);

	// just for debug
	header = (struct hbipc_header *)bif_data.send_frame;

	ret = bif_tx_put_frame_domain(&domain, bif_data.send_frame, data.len);
	if (ret < 0) {
		if (ret == BIF_TX_ERROR_TRANS)
			data.result = HBIPC_ERROR_HW_TRANS_ERROR;
		else if (ret == BIF_TX_ERROR_TIMEOUT)
			data.result = HBIPC_ERROR_SEND_USER_TIMEOUT;
		ret = -1;
		status = copy_to_user((void __user *)buf, &data, sizeof(data));
		if (status)
			ret = -EFAULT;
		mutex_unlock(&domain.write_mutex);
		goto error;
	}

	mutex_unlock(&domain.write_mutex);
	mutex_unlock(&write_mutex);

	++domain.domain_statistics.write_real_count;

	return ret;
error:
	mutex_unlock(&write_mutex);
	return ret;
}

static DEFINE_MUTEX(read_mutex);
static ssize_t x2_bif_read(struct file *file, char __user *buf, size_t size,
loff_t *ppos)
{
	int ret = 0;
	struct send_mang_data data;
	int status = 0;
	struct bif_frame_cache *frame = NULL;
	struct session_desc *session_des = NULL;
	struct hbipc_header *header = NULL;
	struct list_head *pos = NULL;
	int session_list_empty = 0;
	struct transfer_feature *feature = NULL;
	#if __SIZEOF_POINTER__ == 4
	unsigned int addr_low = 0;
	#endif

	//mutex_lock(&read_mutex);
	++domain.domain_statistics.read_call_count;

	if (copy_from_user(&data, (const char __user *)buf, sizeof(data))) {
		hbipc_error("copy_from_user_fail\n");
		ret = -EFAULT;
		goto error;
	}

	session_des = is_valid_session(&domain, &data, NULL, NULL);
	if (!session_des) {
		hbipc_error("invalid session: %d_%d_%d\n", data.domain_id,
			data.provider_id, data.client_id);
		ret = -1;
		data.result = HBIPC_ERROR_INVALID_SESSION;
		status = copy_to_user((void __user *)buf, &data, sizeof(data));
		if (status)
			ret = -EFAULT;
		goto error;
	}

	feature = (struct transfer_feature *)file->private_data;
	if (feature->block) {
		// block
		if (!feature->usr_timeout) {
			// block without timeout
			ret = down_interruptible(&session_des->frame_count_sem);
			if (ret < 0) {
				pr_info("down_interruptible\n");
				goto error;
			} else {
				spin_lock(&(session_des->recv_list.lock));
				session_list_empty = list_empty(&(session_des->recv_list.list));
				if (!session_list_empty) {
					pos = session_des->recv_list.list.next;
					list_del(pos);
	}
				spin_unlock(&(session_des->recv_list.lock));

				if (session_list_empty) {
					ret = -1;
					data.result = HBIPC_ERROR_INVALID_SESSION;
					status = copy_to_user((void __user *)buf, &data, sizeof(data));
					if (status)
						ret = -EFAULT;
					goto error;
				}
			}
		} else {
			// block with timeout
			ret = down_timeout(&session_des->frame_count_sem,
				msecs_to_jiffies(feature->usr_timeout));
		if (ret < 0) {
			data.result = HBIPC_ERROR_RECV_USER_TIMEOUT;
			status = copy_to_user((void __user *)buf, &data, sizeof(data));
			if (status)
				ret = -EFAULT;
			goto error;
		} else {
			spin_lock(&(session_des->recv_list.lock));
			session_list_empty = list_empty(&(session_des->recv_list.list));
			if (!session_list_empty) {
				pos = session_des->recv_list.list.next;
				list_del(pos);
			}
	spin_unlock(&(session_des->recv_list.lock));

	if (session_list_empty) {
		ret = -1;
		data.result = HBIPC_ERROR_INVALID_SESSION;
		status = copy_to_user((void __user *)buf, &data, sizeof(data));
		if (status)
			ret = -EFAULT;
		goto error;
	}
		}
		}
	} else {
		// nonblock
		spin_lock(&(session_des->recv_list.lock));
		session_list_empty = list_empty(&(session_des->recv_list.list));
		if (!session_list_empty) {
			pos = session_des->recv_list.list.next;
			list_del(pos);
		}
		spin_unlock(&(session_des->recv_list.lock));

		if (session_list_empty) {
			ret = 0;
			goto error;
		}
	}

	frame = list_entry(pos, struct bif_frame_cache, frame_cache_list);
	if (frame->framelen - HBIPC_HEADER_LEN > data.len) {
		#if __SIZEOF_POINTER__ == 4
		hbipc_error("recv buf overflow:%d_%d\n",
			frame->framelen - HBIPC_HEADER_LEN,
			data.len);
		#else
		hbipc_error("recv buf overflow:%ld_%d\n",
			frame->framelen - HBIPC_HEADER_LEN,
			data.len);
		#endif
		spin_lock(&(session_des->recv_list.lock));
		list_add(pos, &session_des->recv_list.list);
		spin_unlock(&(session_des->recv_list.lock));
		if (feature->block)
			up(&session_des->frame_count_sem);
		ret = -1;
		data.result = HBIPC_ERROR_RECV_OVERFLOW;
		status = copy_to_user((void __user *)buf, &data,
		sizeof(data));
		if (status)
			ret = -EFAULT;
		goto error;
	}
	header = (struct hbipc_header *)frame->framecache;
	#if __SIZEOF_POINTER__ == 4
	addr_low = (unsigned int)data.buffer;
	status = copy_to_user((void __user *)addr_low,
	frame->framecache + HBIPC_HEADER_LEN, header->length);
	#else
	status = copy_to_user((void __user *)data.buffer,
	frame->framecache + HBIPC_HEADER_LEN, header->length);
	#endif

	if (status) {
		spin_lock(&(session_des->recv_list.lock));
		list_add(pos, &session_des->recv_list.list);
		spin_unlock(&(session_des->recv_list.lock));
		if (feature->block)
			up(&session_des->frame_count_sem);
		ret = -EFAULT;
	} else {
		ret = header->length;
		// consume a data frame really
		spin_lock(&(session_des->recv_list.lock));
		--session_des->recv_list.frame_count;
		//list_del(pos);
		//kfree(frame);
		spin_unlock(&(session_des->recv_list.lock));
		bif_del_session_frame_domain(&domain, frame);
	}

	//mutex_unlock(&read_mutex);
	++domain.domain_statistics.read_real_count;

	return ret;
error:
	//mutex_unlock(&read_mutex);
	return ret;
}

static int x2_bif_close(struct inode *inode, struct file *file)
{
	struct send_mang_data data;

	mutex_lock(&open_mutex);

	pr_info("pid = %d\n", current->pid);
	pr_info("tgid = %d\n", current->tgid);
	pr_info("surplus frame: %d\n", domain.channel.rx_frame_count);
	kfree(file->private_data);
	file->private_data = NULL;
#ifndef CONFIG_HOBOT_BIF_AP
	data.domain_id = domain.domain_id;
	data.provider_id = current->tgid;
	// chencheng reconstitution
	//unregister_server_provider_abnormal(&data);
	unregister_server_provider_abnormal(&domain, &data);
#else
	data.domain_id = domain.domain_id;
	data.client_id = current->tgid;
	// chencheng reconstitution
	//disconnect_stopserver_abnormal(&data);
	disconnect_stopserver_abnormal(&domain, &data);
#endif

	--bif_data.users;

	mutex_unlock(&open_mutex);

	return 0;
}

static unsigned int x2_bif_poll(struct file *file,
	struct poll_table_struct *wait)
{
	return 0;
}

static DEFINE_MUTEX(ioctl_mutex);
static long x2_bif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct send_mang_data data;
	int status = 0;
	struct session_desc *connect = NULL;
	struct session_desc *session_des = NULL;
	struct transfer_feature *feature = NULL;

	mutex_lock(&ioctl_mutex);

	switch (cmd) {
	case BIF_IO_PROVIDER_INIT:
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		ret = register_server_provider(&domain, &data);
		if (ret < 0) {
			hbipc_error("register_provider error\n");
			data.result = ret;
			status = copy_to_user((void __user *)arg, &data,
			sizeof(data));
			if (status)
				ret = -EFAULT;
			break;
		}
		break;
	case BIF_IO_PROVIDER_DEINIT:
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		ret = unregister_server_provider(&domain, &data);
		if (ret < 0) {
			hbipc_error("unregister_provider error\n");
			data.result = ret;
			status = copy_to_user((void __user *)arg, &data,
			sizeof(data));
			if (status)
				ret = -EFAULT;
			break;
		}
		break;
	case BIF_IO_ACCEPT:
		++domain.domain_statistics.accept_count;
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		ret = accept_session(&domain, &data, &connect);
		if (ret < 0) {
			data.result = ret;
		} else if (ret == 0) {
			data.result = 0;
			data.domain_id = connect->domain_id;
			data.provider_id = connect->provider_id;
			data.client_id = connect->client_id;
		} else {
			data.result = HBIPC_ERROR_NO_VALID_SESSION;
			ret = -1;
		}
		status = copy_to_user((void __user *)arg, &data,
		sizeof(data));
		if (status) {
			if (ret == 0) {
				connect->connected = 0;
				++domain.unaccept_session_count;
			}
			ret = -EFAULT;
		}
		break;
	case BIF_IO_START_SERVER:
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		ret = start_server(&domain, &data);
		status = copy_to_user((void __user *)arg, &data,
		sizeof(data));
		if (status)
			ret = -EFAULT;
		break;
	case BIF_IO_STOP_SERVER:
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		ret = stop_server(&domain, &data);
		status = copy_to_user((void __user *)arg, &data,
		sizeof(data));
		if (status)
			ret = -EFAULT;
		break;
	case BIF_IO_CONNECT:
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		session_des = is_valid_session(&domain, &data, NULL, NULL);
		if (session_des) {
			data.result = HBIPC_ERROR_REPEAT_CONNECT;
			ret = -1;
			goto connect_out;
		}

		ret = register_connect(&domain, &data);
		if (ret < 0) {
			hbipc_error("register connect error\n");
			data.result = ret;
		}
connect_out:
		status = copy_to_user((void __user *)arg, &data,
		sizeof(data));
		if (status)
			ret = -EFAULT;
		break;
	case BIF_IO_DISCONNECT:
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		ret = unregister_connect(&domain, &data);
		if (ret < 0) {
			hbipc_error("unregister connect error\n");
			data.result = ret;
		}

		status = copy_to_user((void __user *)arg, &data,
		sizeof(data));
		if (status)
			ret = -EFAULT;
		break;
	case BIF_IO_SET_USR_TIMEOUT:
		feature = (struct transfer_feature *)file->private_data;
		feature->usr_timeout = (int)arg;
		pr_info("usr_timeout = %dms\n", feature->usr_timeout);
		hbeth_set_sendtimeout(feature->usr_timeout);
		break;
	case BIF_IO_GET_FRAME_LIMIT:
		status = copy_to_user((void __user *)arg, &frame_len_max_g,
		sizeof(frame_len_max_g));
		if (status)
			ret = -EFAULT;
		break;
	case BIF_IO_SET_LIB_VERSION:
		if (copy_from_user(version_str, (const char __user *)arg,
			VERSION_LEN)) {
			hbipc_error("set lib version error\n");
			ret = -EFAULT;
		}
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&ioctl_mutex);

	return ret;
}

static const struct file_operations bif_fops = {
	.owner           = THIS_MODULE,
	.open            = x2_bif_open,
	.write           = x2_bif_write,
	.read            = x2_bif_read,
	.poll            = x2_bif_poll,
	.release         = x2_bif_close,
	.unlocked_ioctl  = x2_bif_ioctl,
	.compat_ioctl    = x2_bif_ioctl,
};

static const struct of_device_id bif_lite_of_match[] = {
	{.compatible = "hobot,bif_lite_eth"},
	{},
};

static int bif_major;
static struct cdev bif_cdev;

#ifdef CONFIG_HOBOT_BIF_AP
static void bif_dev_sd_clear(void)
{
	clear_server_cp_manager(&domain);
	bif_lite_init_success = 0;
}
#endif

#ifndef CONFIG_NO_DTS_AP
static int bif_lite_probe(struct platform_device *pdev)
{
	int           ret = 0;
	dev_t         devno;
	struct cdev  *p_cdev = &bif_cdev;
	int frame_len_max = 0;

	pr_info("biflite_eth version: %s\n", VERSION);
#if 0
	unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;
#endif
	ret = of_property_read_u32(pdev->dev.of_node,
	"frame_len_max", &frame_len_max);
	if (ret) {
		bif_err("get frame_len_max error\n");
		goto error;
	} else
		frame_len_max_g = frame_len_max;

	x2_mem_layout_set(&domain_config);

	bif_major = 0;
	ret = alloc_chrdev_region(&devno, 0, 1, "x2_eth");
	if (ret < 0) {
		bif_debug("Error %d while alloc chrdev eth", ret);
		goto alloc_chrdev_error;
	}
	bif_major = MAJOR(devno);
	cdev_init(p_cdev, &bif_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		bif_debug("Error %d while adding x2 eth cdev", ret);
		goto cdev_add_error;
	}
	g_bif_class = class_create(THIS_MODULE, "x2_eth");
	if (IS_ERR(g_bif_class)) {
		bif_debug("[%s:%d] class_create error\n",
			__func__, __LINE__);
		ret = PTR_ERR(g_bif_class);
		goto class_create_error;
	}
	g_bif_dev = device_create(g_bif_class, NULL,
		MKDEV(bif_major, 0), NULL, "x2_eth");
	if (IS_ERR(g_bif_dev)) {
		bif_debug("[%s] device create error\n", __func__);
		ret = PTR_ERR(g_bif_dev);
		goto device_create_error;
	}

	if (x2_bif_data_init(&bif_data) < 0) {
		bif_debug("x2_bif_data_init error\n");
		goto bif_data_init_error;
	}

	ret = domain_init(&domain, &domain_config);
	if (ret < 0) {
		pr_info("domain_init error\n");
		goto domain_init_error;
	}
#ifdef CONFIG_HOBOT_BIF_AP
	domain_register_high_level_clear(&domain, bif_dev_sd_clear);
#endif
	bif_debug("bif driver init exit\n");
	return 0;
domain_init_error:
	x2_bif_data_deinit(&bif_data);
bif_data_init_error:
	device_destroy(g_bif_class, MKDEV(bif_major, 0));
device_create_error:
	class_destroy(g_bif_class);
class_create_error:
	cdev_del(&bif_cdev);
cdev_add_error:
	unregister_chrdev_region(MKDEV(bif_major, 0), 1);
alloc_chrdev_error:
error:
	return ret;
}
#endif

#ifdef CONFIG_NO_DTS_AP
static int bif_lite_probe_param(void)
{
	int           ret = 0;
	dev_t         devno;
	struct cdev  *p_cdev = &bif_cdev;

	pr_info("biflite_eth version: %s\n", VERSION);
#if 0
	unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;
#endif
	frame_len_max_g = frame_len_max_ap;

	x2_mem_layout_set(&domain_config);

	bif_major = 0;
	ret = alloc_chrdev_region(&devno, 0, 1, "x2_eth");
	if (ret < 0) {
		bif_debug("Error %d while alloc chrdev eth", ret);
		goto alloc_chrdev_error;
	}
	bif_major = MAJOR(devno);
	cdev_init(p_cdev, &bif_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		bif_debug("Error %d while adding x2 eth cdev", ret);
		goto cdev_add_error;
	}
	g_bif_class = class_create(THIS_MODULE, "x2_eth");
	if (IS_ERR(g_bif_class)) {
		bif_debug("[%s:%d] class_create error\n",
			__func__, __LINE__);
		ret = PTR_ERR(g_bif_class);
		goto class_create_error;
	}
	g_bif_dev = device_create(g_bif_class, NULL,
		MKDEV(bif_major, 0), NULL, "x2_eth");
	if (IS_ERR(g_bif_dev)) {
		bif_debug("[%s] device create error\n", __func__);
		ret = PTR_ERR(g_bif_dev);
		goto device_create_error;
	}

	if (x2_bif_data_init(&bif_data) < 0) {
		bif_debug("x2_bif_data_init error\n");
		goto bif_data_init_error;
	}

	ret = domain_init(&domain, &domain_config);
	if (ret < 0) {
		pr_info("domain_init error\n");
		goto domain_init_error;
	}

#ifdef CONFIG_HOBOT_BIF_AP
	domain_register_high_level_clear(&domain, bif_dev_sd_clear);
#endif
	bif_debug("bif driver init exit\n");
	return 0;
domain_init_error:
	x2_bif_data_deinit(&bif_data);
bif_data_init_error:
	device_destroy(g_bif_class, MKDEV(bif_major, 0));
device_create_error:
	class_destroy(g_bif_class);
class_create_error:
	cdev_del(&bif_cdev);
cdev_add_error:
	unregister_chrdev_region(MKDEV(bif_major, 0), 1);
alloc_chrdev_error:
		return ret;
}
#endif

#ifndef CONFIG_NO_DTS_AP
static int bif_lite_remove(struct platform_device *pdev)
{
#ifdef CONFIG_HOBOT_BIF_AP
	domain_unregister_high_level_clear(&domain);
#endif
	domain_deinit(&domain);
	x2_bif_data_deinit(&bif_data);
	device_destroy(g_bif_class, MKDEV(bif_major, 0));
	class_destroy(g_bif_class);
	cdev_del(&bif_cdev);
	unregister_chrdev_region(MKDEV(bif_major, 0), 1);

	return 0;
}
#endif

#ifdef CONFIG_NO_DTS_AP
static int bif_lite_remove_param(void)
{
#ifdef CONFIG_HOBOT_BIF_AP
	domain_unregister_high_level_clear(&domain);
#endif
	domain_deinit(&domain);
	x2_bif_data_deinit(&bif_data);
	device_destroy(g_bif_class, MKDEV(bif_major, 0));
	class_destroy(g_bif_class);
	cdev_del(&bif_cdev);
	unregister_chrdev_region(MKDEV(bif_major, 0), 1);

	return 0;
}
#endif

#ifndef CONFIG_NO_DTS_AP
static struct platform_driver bif_lite_driver = {
	.driver = {
		.name = "bif_lite_eth",
		.of_match_table = bif_lite_of_match,
	},
	.probe = bif_lite_probe,
	.remove = bif_lite_remove,
};
#endif

static int __init bif_module_init(void)
{
	int           ret = 0;

#ifdef CONFIG_NO_DTS_AP
	ret = bif_lite_probe_param();
	if (ret)
		bif_err("register bif_lite_probe_param error\n");
#else
	ret = platform_driver_register(&bif_lite_driver);
	if (ret)
		bif_err("register bif_lite_driver error\n");
#endif

	return ret;
}

static void __exit bif_module_exit(void)
{
#ifdef CONFIG_NO_DTS_AP
	bif_lite_remove_param();
#else
	platform_driver_unregister(&bif_lite_driver);
#endif
}

//module_init(bif_module_init);
late_initcall(bif_module_init);
module_exit(bif_module_exit);
MODULE_LICENSE("GPL v2");
