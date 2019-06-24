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
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "hbipc_lite.h"
#include "hbipc_errno.h"
#include "bif_dev_spi.h"

#ifdef CONFIG_HI3519V101
/* module parameters */
static char *ap_type_str = "soc-ap";
static char *working_mode_str = "interrupt-mode";
module_param(ap_type_str, charp, 0644);
module_param(working_mode_str, charp, 0644);
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

//#define WORK_COUNT (100)
struct x2_bif_data {
	int users;
	int irq_pin;
	int irq_num;
	int tri_pin;
	char *send_frame;
	struct workqueue_struct *work_queue;
	//struct work_struct work[WORK_COUNT];
	struct work_struct work;
	//int work_index;
	struct completion ready_to_recv;
	struct task_struct *recv_task;
};
static struct x2_bif_data bif_data;

static struct class  *g_bif_class;
static struct device *g_bif_dev;

static struct domain_info domain_config = {.domain_name = "X2BIF001",
	.domain_id = 0,
	.device_name = "/dev/x2_bif",
	.type = SOC_AP,
	.mode = INTERRUPT_MODE,
	{.channel = BIF_SPI,
	.type = SOC_AP,
	.mode = INTERRUPT_MODE}
};

static struct comm_domain domain;

/* proc debug fs */
#define BIF_DEV_SPI_DIR "bif_dev_spi"
#define BIF_DEV_SPI_STATTISTICS "statistics"
static struct proc_dir_entry *bif_dev_spi_entry;
static struct proc_dir_entry *bif_dev_spi_statistics_entry;

static int bif_dev_spi_statistics_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "bif_dev_sd dev layer statistics:\n\
irq_handler_count = %d\nrx_work_func_count = %d\n\
rx_flowcontrol_count = %d\naccept_count = %d\n\
write_call_count = %d\nwrite_real_count = %d\n\
read_call_count = %d\nread_real_count = %d\n\
bif_dev_sd hbipc layer statistics:\n\
interrupt_recv_count = %d\nmanage_recv_count = %d\n\
data_recv_count = %d\nmanage_frame_count = %d\n\
data_frame_count = %d\nup_sem_count = %d\n\
send_manage_count = %d\n\
bif_dev_sd transfer layer statistics:\n\
resend_count = %d\nresend_over_count = %d\n\
trig_count = %d\nretrig_count = %d\n",
	domain.domain_statistics.irq_handler_count,
	domain.domain_statistics.rx_work_func_count,
	domain.domain_statistics.rx_flowcontrol_count,
	domain.domain_statistics.accept_count,
	domain.domain_statistics.write_call_count,
	domain.domain_statistics.write_real_count,
	domain.domain_statistics.read_call_count,
	domain.domain_statistics.read_real_count,
	domain.domain_statistics.interrupt_recv_count,
	domain.domain_statistics.manage_recv_count,
	domain.domain_statistics.data_recv_count,
	domain.domain_statistics.manage_frame_count,
	domain.domain_statistics.data_frame_count,
	domain.domain_statistics.up_sem_count,
	domain.domain_statistics.send_manage_count,
	domain.channel.channel_statistics.resend_count,
	domain.channel.channel_statistics.resend_over_count,
	domain.channel.channel_statistics.trig_count,
	domain.channel.channel_statistics.retrig_count);

	return 0;
}

static ssize_t bif_dev_spi_statistics_proc_write(struct file *file,
const char __user *buffer, size_t count, loff_t *ppos)
{
	memset(&(domain.domain_statistics), 0,
	sizeof(domain.domain_statistics));
	memset(&(domain.channel.channel_statistics), 0,
		sizeof(domain.channel.channel_statistics));

	return count;
}

static int bif_dev_spi_statistics_proc_open(struct inode *inode,
struct file *file)
{
	return single_open(file, bif_dev_spi_statistics_proc_show, NULL);
}

static const struct file_operations bif_dev_spi_statistics_proc_ops = {
	.owner    = THIS_MODULE,
	.open     = bif_dev_spi_statistics_proc_open,
	.read     = seq_read,
	.write    = bif_dev_spi_statistics_proc_write,
	.llseek   = seq_lseek,
	.release  = single_release,
};

static int init_bif_dev_spi_debug_port(void)
{
	bif_dev_spi_entry = proc_mkdir(BIF_DEV_SPI_DIR, NULL);
	if (!bif_dev_spi_entry) {
		pr_info("create /proc/%s fail\n", BIF_DEV_SPI_DIR);
		goto create_top_dir_error;
	}

	bif_dev_spi_statistics_entry = proc_create(BIF_DEV_SPI_STATTISTICS,
	0777, bif_dev_spi_entry, &bif_dev_spi_statistics_proc_ops);
	if (!bif_dev_spi_statistics_entry) {
		pr_info("create /proc/%s/%s fail\n", BIF_DEV_SPI_DIR,
			BIF_DEV_SPI_STATTISTICS);
		goto create_statistics_file_error;
	}

	return 0;
create_statistics_file_error:
	remove_proc_entry(BIF_DEV_SPI_DIR, NULL);
create_top_dir_error:
	return -1;
}

static void remove_bif_dev_spi_debug_port(void)
{
	remove_proc_entry(BIF_DEV_SPI_STATTISTICS, bif_dev_spi_entry);
	remove_proc_entry(BIF_DEV_SPI_DIR, NULL);
}

static int domain_deinit_stop;
static void rx_work_func(struct work_struct *work)
{
	int surplus_frame = 0;
	unsigned long remaining_time = 0;

	while (1) {
		if (wait_for_completion_interruptible(&bif_data.ready_to_recv) < 0) {
			pr_info("wait_for_completion_interruptible\n");
			break;
		}

		if (domain_deinit_stop) {
			pr_info("domain_deinit\n");
			break;
		}
		++domain.domain_statistics.rx_work_func_count;

		reinit_completion(&bif_data.ready_to_recv);

		// read to no frame
		// handle rx flow control
		while (recv_frame_interrupt(&domain) >= 0) {
rx_flow_control:
			surplus_frame = domain_stock_frame_num(&domain);

			if (surplus_frame > domain.channel.frame_cache_max) {
				++domain.domain_statistics.rx_flowcontrol_count;
				remaining_time = msleep_interruptible(20);
				if (!remaining_time)
					goto rx_flow_control;
				else {
					pr_info("rx_flow_control interruptible\n");
					break;
				}
			}
		}
	}
}

static irqreturn_t hbipc_irq_handler(int irq, void *data)
{
	//recv_frame_interrupt_new(&domain);

	//if (queue_work(bif_data.work_queue, &(bif_data.work[bif_data.work_index++])) == false)
	//	pr_info("queue_work fail\n");
	//bif_data.work_index %= WORK_COUNT;

	complete(&bif_data.ready_to_recv);
	++domain.domain_statistics.irq_handler_count;

	return IRQ_HANDLED;
}

static int recv_thread(void *data)
{
	pr_info("%s start.......\n", __func__);
	while (1) {
		recv_handle_data_frame(&domain);
		usleep_range(1000, 2000);
	}

	return 0;
}

static void x2_mem_layout_set(struct domain_info *domain_inf)
{
	domain_inf->channel_cfg.frame_len_max = FRAME_LEN_MAX;
	domain_inf->channel_cfg.frag_len_max = FRAG_LEN_MAX;
	domain_inf->channel_cfg.frag_num = FRAG_NUM;
	domain_inf->channel_cfg.frame_cache_max = FRAME_CACHE_MAX;
	domain_inf->channel_cfg.rx_local_info_offset = RX_LOCAL_INFO_OFFSET;
	domain_inf->channel_cfg.rx_remote_info_offset = RX_REMOTE_INFO_OFFSET;
	domain_inf->channel_cfg.tx_local_info_offset = TX_LOCAL_INFO_OFFSET;
	domain_inf->channel_cfg.tx_remote_info_offset = TX_REMOTE_INFO_OFFSET;
	domain_inf->channel_cfg.rx_buffer_offset = RX_BUFFER_OFFSET;
	domain_inf->channel_cfg.tx_buffer_offset = TX_BUFFER_OFFSET;
	domain_inf->channel_cfg.total_mem_size = TOTAL_MEM_SIZE;
}

static int x2_bif_data_init(struct x2_bif_data *bif_data)
{
	bif_data->users = 0;
	bif_data->irq_pin = -1;
	bif_data->tri_pin = -1;
	bif_data->irq_num = -1;

	bif_data->send_frame = kmalloc(FRAME_LEN_MAX, GFP_KERNEL);
	if (!(bif_data->send_frame)) {
		pr_info("malloc_send_frame error\n");
		goto malloc_send_frame_error;
	}

	if (init_bif_dev_spi_debug_port() < 0) {
		pr_info("init_debug_port error\n");
		goto init_debug_port_error;
	}

	if (domain_config.mode == INTERRUPT_MODE) {
		// interrupt-mode
		bif_data->work_queue =
		create_singlethread_workqueue("bifspi_workqueue");
		if (!bif_data->work_queue) {
			pr_info("create_singlethread_workqueue error\n");
			goto engine_error;
		}
		init_completion(&bif_data->ready_to_recv);
		INIT_WORK(&bif_data->work, rx_work_func);
	if (queue_work(bif_data->work_queue, &bif_data->work) == false) {
		pr_info("queue_work fail\n");
			destroy_workqueue(bif_data->work_queue);
			goto engine_error;
		}
	} else {
		// poling-mode
		bif_data->recv_task =
		kthread_create(recv_thread, NULL, "bifspi_recv_thread");
		if (IS_ERR(bif_data->recv_task)) {
			pr_info("kthread_create error\n");
			goto engine_error;
		}
	}

	return 0;
engine_error:
	remove_bif_dev_spi_debug_port();
init_debug_port_error:
	kfree(bif_data->send_frame);
malloc_send_frame_error:
	return -1;
}

static void x2_bif_data_deinit(struct x2_bif_data *bif_data)
{
	bif_data->users = 0;
	bif_data->irq_pin = -1;
	bif_data->tri_pin = -1;
	bif_data->irq_num = -1;
	remove_bif_dev_spi_debug_port();
	domain_deinit_stop = 1;
	complete(&bif_data->ready_to_recv);
	destroy_workqueue(bif_data->work_queue);
	kfree(bif_data->send_frame);
}

static DEFINE_MUTEX(open_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
static int bif_lite_init_success;
#endif
static int x2_bif_open(struct inode *inode, struct file *file)
{
#ifdef CONFIG_HOBOT_BIF_AP
	int ret = 0;
#endif

	mutex_lock(&open_mutex);

	if (!bif_data.users) {
		file->private_data = g_bif_dev;
#ifdef CONFIG_HOBOT_BIF_AP
		if (!bif_lite_init_success) {
			if (bif_lite_init_domain(&domain) < 0) {
				pr_info("1: bif_lite_init error\n");
				ret = -EPERM;
				goto err;
			} else {
				bif_lite_init_success = 1;
				if (domain.mode == INTERRUPT_MODE)
					bif_lite_irq_register_domain(&domain,
					hbipc_irq_handler);
			}
		} else {
			if (bif_lite_init_domain(&domain) < 0) {
				pr_info("2: bif_lite_init error\n");
				ret = -EPERM;
				goto err;
			}
		}
#endif
		recv_handle_stock_frame(&domain);
		if (domain.mode == POLLING_MODE)
			wake_up_process(bif_data.recv_task);
		++bif_data.users;
	} else {
#ifdef CONFIG_HOBOT_BIF_AP
		if (bif_lite_init_domain(&domain) < 0) {
			pr_info("3: bif_lite_init error\n");
			ret = -EPERM;
			goto err;
		}
#endif
		file->private_data = g_bif_dev;
		++bif_data.users;
	}

	mutex_unlock(&open_mutex);

	return 0;
#ifdef CONFIG_HOBOT_BIF_AP
err:
	mutex_unlock(&open_mutex);
	return ret;
#endif

}

static DEFINE_MUTEX(write_mutex);
static ssize_t x2_bif_write(struct file *file, const char __user *buf,
size_t count, loff_t *ppos)
{
	int ret = 0;
	struct send_mang_data data;
	int status = 0;

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

	if (copy_from_user(bif_data.send_frame,
	(const char __user *)data.buffer,
	data.len)) {
		hbipc_error("copy user frame error\n");
		ret = -EFAULT;
		goto error;
	}

	mutex_lock(&domain.write_mutex);
	ret = bif_tx_put_frame_domain(&domain, bif_data.send_frame, data.len);
	if (ret < 0) {
		data.result = HBIPC_ERROR_HW_TRANS_ERROR;
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
#if 0
	ret = recv_handle_data_frame(session_des, &frame);
	if (ret == 0) {
		if (frame->framelen - HBIPC_HEADER_LEN > data.len) {
			hbipc_error("recv buf overflow\n");
			ret = -1;
			data.result = HBIPC_ERROR_RECV_OVERFLOW;
			status = copy_to_user((void __user *)buf, &data,
			sizeof(data));
			if (status)
				ret = -EFAULT;
			goto error;
		}
		header = (struct hbipc_header *)frame->framecache;
		status = copy_to_user((void __user *)data.buffer,
		frame->framecache + HBIPC_HEADER_LEN, header->length);
		if (status) {
			ret = -EFAULT;
		} else {
			ret = header->length;
			// consume a data frame really
			--session_des->recv_list.frame_count;
			mutex_lock(&domain.read_mutex);
			bif_del_frame_from_list(frame);
			mutex_unlock(&domain.read_mutex);
		}
	} else {
		// no specific data frame get
		ret = 0;
	}
#endif
	ret = down_interruptible(&session_des->frame_count_sem);
	if (ret < 0) {
		printk("down_interruptible\n");
		goto error;
	}

	spin_lock(&(session_des->recv_list.lock));
	session_list_empty = list_empty(&(session_des->recv_list.list));
	spin_unlock(&(session_des->recv_list.lock));

	if (session_list_empty) {
		ret = -1;
		data.result = HBIPC_ERROR_INVALID_SESSION;
		status = copy_to_user((void __user *)buf, &data, sizeof(data));
		if (status)
			ret = -EFAULT;
		goto error;
	}

	spin_lock(&(session_des->recv_list.lock));
	pos = session_des->recv_list.list.next;
	list_del(pos);
	spin_unlock(&(session_des->recv_list.lock));

	frame = list_entry(pos, struct bif_frame_cache, frame_cache_list);
	if (frame->framelen - HBIPC_HEADER_LEN > data.len) {
		hbipc_error("recv buf overflow:%ld_%d\n",
			frame->framelen - HBIPC_HEADER_LEN,
			data.len);
		spin_lock(&(session_des->recv_list.lock));
		list_add(pos, &session_des->recv_list.list);
		spin_unlock(&(session_des->recv_list.lock));
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
	status = copy_to_user((void __user *)data.buffer,
	frame->framecache + HBIPC_HEADER_LEN, header->length);
	if (status) {
		spin_lock(&(session_des->recv_list.lock));
		list_add(pos, &session_des->recv_list.list);
		spin_unlock(&(session_des->recv_list.lock));
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
			data.result = 0;
			ret = -1;
		}
		status = copy_to_user((void __user *)arg, &data,
		sizeof(data));
		if (status)
			ret = -EFAULT;
		break;
	case BIF_IO_START_SERVER:
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		// attempt to read manage frame
		recv_handle_manage_frame(&domain);

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
	{.compatible = "hobot,bif_lite_spi"},
	{},
};

#if 0
static irqreturn_t bif_lite_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}
#endif

static int bif_major;
static struct cdev bif_cdev;

#ifndef CONFIG_HI3519V101
static int bif_lite_probe(struct platform_device *pdev)
{
	int           ret = 0;
	dev_t         devno;
	struct cdev  *p_cdev = &bif_cdev;
#if 0
	unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;
#endif
	if (of_find_property(pdev->dev.of_node, "mcu-ap", NULL)) {
		// mcu-ap
		domain_config.type = MCU_AP;
		domain_config.channel_cfg.type = MCU_AP;
		frame_len_max_g = 4 * 1024;
		frag_len_max_g = 128;
	} else {
		// soc-ap
		domain_config.type = SOC_AP;
		domain_config.channel_cfg.type = SOC_AP;
	}

	if (of_find_property(pdev->dev.of_node, "polling-mode", NULL)) {
		// polling-mode
		domain_config.mode = POLLING_MODE;
		domain_config.channel_cfg.mode = POLLING_MODE;
	} else {
		// interrupt-mode
		domain_config.mode = INTERRUPT_MODE;
		domain_config.channel_cfg.mode = INTERRUPT_MODE;
	}

	x2_mem_layout_set(&domain_config);

	bif_major = 0;
	ret = alloc_chrdev_region(&devno, 0, 1, "x2_bif");
	if (ret < 0) {
		bif_debug("Error %d while alloc chrdev bif", ret);
		goto alloc_chrdev_error;
	}
	bif_major = MAJOR(devno);
	cdev_init(p_cdev, &bif_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		bif_debug("Error %d while adding x2 bif cdev", ret);
		goto cdev_add_error;
	}
	g_bif_class = class_create(THIS_MODULE, "x2_bif");
	if (IS_ERR(g_bif_class)) {
		bif_debug("[%s:%d] class_create error\n",
			__func__, __LINE__);
		ret = PTR_ERR(g_bif_class);
		goto class_create_error;
	}
	g_bif_dev = device_create(g_bif_class, NULL,
		MKDEV(bif_major, 0), NULL, "x2_bif");
	if (IS_ERR(g_bif_dev)) {
		bif_debug("[%s] device create error\n", __func__);
		ret = PTR_ERR(g_bif_dev);
		goto device_create_error;
	}

	if (x2_bif_data_init(&bif_data) < 0) {
		bif_debug("x2_bif_data_init error\n");
		goto bif_data_init_error;
	}
#if 0
	ret = of_property_read_u32(pdev->dev.of_node,
	"bif_lite_irq_pin", &bif_data.irq_pin);
	if (ret) {
		bif_err("get bif_lite_irq_pin error\n");
		goto get_bif_lite_irq_pin_error;
	}

	ret = gpio_request(bif_data.irq_pin, "bif-lite-irq-pin");
	if (ret) {
		bif_err("requset irq pin error\n");
		goto request_irq_pin_error;
	}

	bif_data.irq_num = gpio_to_irq(bif_data.irq_pin);
	irq_set_irq_type(bif_data.irq_num, IRQ_TYPE_EDGE_FALLING);
	ret = request_threaded_irq(bif_data.irq_num,
	bif_lite_irq_handler, NULL, flags,
	"bif-lite-driver", (void *)&bif_data);
	if (ret) {
		bif_err("request irq error\n");
		goto request_irq_error;
	} else
		bif_debug("irq_pin = %d irq_num = %d\n",
	bif_data.irq_pin, bif_data.irq_num);

	ret = of_property_read_u32(pdev->dev.of_node,
	"bif_lite_tri_pin", &bif_data.tri_pin);
	if (ret) {
		bif_err("get bif_lite_tri_pin error\n");
		goto get_bif_lite_tri_pin_error;
	}
	ret = gpio_request(bif_data.tri_pin, "bif-lite-tri-pin");
	if (ret) {
		bif_err("request tri pin error\n");
		goto request_tri_error;
	} else
		bif_debug("tri_pin = %d\n", bif_data.tri_pin);
	gpio_direction_output(bif_data.tri_pin, 1);
#endif
	ret = domain_init(&domain, &domain_config);
	if (ret < 0) {
		pr_info("domain_init error\n");
		goto domain_init_error;
	}
#ifndef CONFIG_HOBOT_BIF_AP
	ret = bif_lite_init_domain(&domain);
	//ret = bif_lite_init(); chencheng resonstitution

	if (ret < 0) {
		pr_info("bif_lite_init error\n");
		goto bif_lite_init_error;
	}
	
	if (domain.mode == INTERRUPT_MODE)
		bif_lite_irq_register_domain(&domain, hbipc_irq_handler);
#endif
	bif_debug("bif driver init exit\n");
	return 0;
#if 0
request_tri_error:
get_bif_lite_tri_pin_error:
	free_irq(bif_data.irq_num, (void *)&bif_data);
request_irq_error:
	gpio_free(bif_data.irq_pin);
request_irq_pin_error:
get_bif_lite_irq_pin_error:
#endif
#ifndef CONFIG_HOBOT_BIF_AP
bif_lite_init_error:
	domain_deinit(&domain);
#endif
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

#ifdef CONFIG_HI3519V101
static int bif_lite_probe_param(void)
{
	int           ret = 0;
	dev_t         devno;
	struct cdev  *p_cdev = &bif_cdev;
#if 0
	unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;
#endif
	if (!strcmp(ap_type_str, "soc-ap")) {
		domain_config.type = SOC_AP;
		domain_config.channel_cfg.type = SOC_AP;
	} else if (!strcmp(ap_type_str, "mcu-ap")) {
		domain_config.type = MCU_AP;
		domain_config.channel_cfg.type = MCU_AP;
		frame_len_max_g = 4 * 1024;
		frag_len_max_g = 128;
	} else {
		pr_info("Error ap type\n");
		return -1;
	}

	if (!strcmp(working_mode_str, "interrupt-mode")) {
		domain_config.mode = INTERRUPT_MODE;
		domain_config.channel_cfg.mode = INTERRUPT_MODE;
	} else if (!strcmp(working_mode_str, "polling-mode")) {
		domain_config.mode = POLLING_MODE;
		domain_config.channel_cfg.mode = POLLING_MODE;
	} else {
		pr_info("Error working mode\n");
		return -1;
	}

	x2_mem_layout_set(&domain_config);

	bif_major = 0;
	ret = alloc_chrdev_region(&devno, 0, 1, "x2_bif");
	if (ret < 0) {
		bif_debug("Error %d while alloc chrdev bif", ret);
		goto alloc_chrdev_error;
	}
	bif_major = MAJOR(devno);
	cdev_init(p_cdev, &bif_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		bif_debug("Error %d while adding x2 bif cdev", ret);
		goto cdev_add_error;
	}
	g_bif_class = class_create(THIS_MODULE, "x2_bif");
	if (IS_ERR(g_bif_class)) {
		bif_debug("[%s:%d] class_create error\n",
			__func__, __LINE__);
		ret = PTR_ERR(g_bif_class);
		goto class_create_error;
	}
	g_bif_dev = device_create(g_bif_class, NULL,
		MKDEV(bif_major, 0), NULL, "x2_bif");
	if (IS_ERR(g_bif_dev)) {
		bif_debug("[%s] device create error\n", __func__);
		ret = PTR_ERR(g_bif_dev);
		goto device_create_error;
	}

	if (x2_bif_data_init(&bif_data) < 0) {
		bif_debug("x2_bif_data_init error\n");
		goto bif_data_init_error;
	}
#if 0
	ret = of_property_read_u32(pdev->dev.of_node,
	"bif_lite_irq_pin", &bif_data.irq_pin);
	if (ret) {
		bif_err("get bif_lite_irq_pin error\n");
		goto get_bif_lite_irq_pin_error;
	}

	ret = gpio_request(bif_data.irq_pin, "bif-lite-irq-pin");
	if (ret) {
		bif_err("requset irq pin error\n");
		goto request_irq_pin_error;
	}

	bif_data.irq_num = gpio_to_irq(bif_data.irq_pin);
	irq_set_irq_type(bif_data.irq_num, IRQ_TYPE_EDGE_FALLING);
	ret = request_threaded_irq(bif_data.irq_num,
	bif_lite_irq_handler, NULL, flags,
	"bif-lite-driver", (void *)&bif_data);
	if (ret) {
		bif_err("request irq error\n");
		goto request_irq_error;
	} else
		bif_debug("irq_pin = %d irq_num = %d\n",
	bif_data.irq_pin, bif_data.irq_num);

	ret = of_property_read_u32(pdev->dev.of_node,
	"bif_lite_tri_pin", &bif_data.tri_pin);
	if (ret) {
		bif_err("get bif_lite_tri_pin error\n");
		goto get_bif_lite_tri_pin_error;
	}
	ret = gpio_request(bif_data.tri_pin, "bif-lite-tri-pin");
	if (ret) {
		bif_err("request tri pin error\n");
		goto request_tri_error;
	} else
		bif_debug("tri_pin = %d\n", bif_data.tri_pin);
	gpio_direction_output(bif_data.tri_pin, 1);
#endif
	ret = domain_init(&domain, &domain_config);
	if (ret < 0) {
		pr_info("domain_init error\n");
		goto domain_init_error;
	}
#ifndef CONFIG_HOBOT_BIF_AP
	ret = bif_lite_init_domain(&domain);
	//ret = bif_lite_init(); chencheng resonstitution

	if (ret < 0) {
		pr_info("bif_lite_init error\n");
		goto bif_lite_init_error;
	}

	if (domain.mode == INTERRUPT_MODE)
		bif_lite_irq_register_domain(&domain, hbipc_irq_handler);
#endif

	bif_debug("bif driver init exit\n");
	return 0;
#if 0
request_tri_error:
get_bif_lite_tri_pin_error:
	free_irq(bif_data.irq_num, (void *)&bif_data);
request_irq_error:
	gpio_free(bif_data.irq_pin);
request_irq_pin_error:
get_bif_lite_irq_pin_error:
#endif
#ifndef CONFIG_HOBOT_BIF_AP
bif_lite_init_error:
	domain_deinit(&domain);
#endif
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

#ifndef CONFIG_HI3519V101
static int bif_lite_remove(struct platform_device *pdev)
{
	//bif_lite_exit(); chencheng resonstitution
	bif_lite_exit_domain(&domain);
	domain_deinit(&domain);
	x2_bif_data_deinit(&bif_data);
#if 0
	free_irq(bif_data.irq_num, (void *)&bif_data);
	gpio_free(bif_data.irq_pin);
	gpio_free(bif_data.tri_pin);
#endif
	device_destroy(g_bif_class, MKDEV(bif_major, 0));
	class_destroy(g_bif_class);
	cdev_del(&bif_cdev);
	unregister_chrdev_region(MKDEV(bif_major, 0), 1);

	return 0;
}
#endif

#ifdef CONFIG_HI3519V101
static int bif_lite_remove_param(void)
{
	//bif_lite_exit(); chencheng resonstitution
	bif_lite_exit_domain(&domain);
	domain_deinit(&domain);
	x2_bif_data_deinit(&bif_data);
#if 0
	free_irq(bif_data.irq_num, (void *)&bif_data);
	gpio_free(bif_data.irq_pin);
	gpio_free(bif_data.tri_pin);
#endif
	device_destroy(g_bif_class, MKDEV(bif_major, 0));
	class_destroy(g_bif_class);
	cdev_del(&bif_cdev);
	unregister_chrdev_region(MKDEV(bif_major, 0), 1);

	return 0;
}
#endif

#ifndef CONFIG_HI3519V101
static struct platform_driver bif_lite_driver = {
	.driver = {
		.name = "bif_lite_spi",
		.of_match_table = bif_lite_of_match,
	},
	.probe = bif_lite_probe,
	.remove = bif_lite_remove,
};
#endif

static int __init bif_module_init(void)
{
	int           ret = 0;

#ifdef CONFIG_HI3519V101
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
#ifdef CONFIG_HI3519V101
	bif_lite_remove_param();
#else
	platform_driver_unregister(&bif_lite_driver);
#endif
}

//module_init(bif_module_init);
late_initcall(bif_module_init);
module_exit(bif_module_exit);
MODULE_LICENSE("GPL v2");
