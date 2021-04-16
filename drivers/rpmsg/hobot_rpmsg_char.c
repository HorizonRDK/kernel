/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2020 Horizon Robotics, Inc.
 *			 All rights reserved.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/virtio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/rwsem.h>
#include <linux/errno.h>
#include <linux/rpmsg/hobot_rpmsg.h>
#include <linux/time.h>

//#define TIMESTAMP_DEBUG

struct debug_statistics {
	int send_frame_count;
	int send_nonblock_fail_count;
	int send_block_timeout_count;
	int recv_frame_count;
	int recv_nonblock_fail_count;
	int recv_block_timeout_count;
	int alloc_skb_fail_count;
	int alloc_skb_count;
	int crc_error_count;
};

struct rpmsg_char_port {
	struct rpmsg_device *rpdev;
	char server_name[RPMSG_NAME_SIZE];
	struct proc_dir_entry *server_entry;
	struct debug_statistics statistics;
	struct proc_dir_entry *statistics_entry;
	dev_t devno;
	struct cdev cdev;
	int sleep_ms;
	struct list_head list;
	struct rw_semaphore valid_rwsem;
	int valid;
	struct mutex write_mutex;
	char *write_buffer;
	spinlock_t queue_lock;
	struct sk_buff_head queue;
	wait_queue_head_t readq;
	spinlock_t users_lock;
	int crc_check;
	int users;
};

struct global_data {
	struct rpmsg_device_id *id_table;
	int nr_server;
	struct proc_dir_entry *top_entry;
	struct class *rpmsg_chrdev_class;
	dev_t devno_from;
	int rpmsg_chrdev_major;
	struct list_head chrdev_list;
};
struct global_data gd;

static int proc_statistics_show(struct seq_file *m, void *v)
{
	struct rpmsg_char_port *port =
	(struct rpmsg_char_port *)(m->private);

	seq_printf(m, "valid = %d\n", port->valid);
	seq_printf(m, "send_frame_count = %d\n",
	port->statistics.send_frame_count);
	seq_printf(m, "send_nonblock_fail_count = %d\n",
	port->statistics.send_nonblock_fail_count);
	seq_printf(m, "send_block_timeout_count = %d\n",
	port->statistics.send_block_timeout_count);
	seq_printf(m, "recv_frame_count = %d\n",
	port->statistics.recv_frame_count);
	seq_printf(m, "recv_nonblock_fail_count = %d\n",
	port->statistics.recv_nonblock_fail_count);
	seq_printf(m, "recv_block_timeout_count = %d\n",
	port->statistics.recv_block_timeout_count);
	seq_printf(m, "alloc_skb_fail_count = %d\n",
	port->statistics.alloc_skb_fail_count);
	seq_printf(m, "alloc_skb_count = %d\n",
	port->statistics.alloc_skb_count);
	seq_printf(m, "crc_error_count = %d\n",
	port->statistics.crc_error_count);

	return 0;
}

static ssize_t proc_statistics_write(struct file *file,
const char __user *buffer, size_t count, loff_t *ppos)
{
	struct rpmsg_char_port *port = (struct rpmsg_char_port *)
		(((struct seq_file *)file->private_data)->private);

	memset(&port->statistics, 0, sizeof(port->statistics));

	return count;
}

static int proc_statistics_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_statistics_show, PDE_DATA(inode));
}

static const struct file_operations statistics_ops = {
	.owner = THIS_MODULE,
	.open = proc_statistics_open,
	.read = seq_read,
	.write = proc_statistics_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int proc_top_dir_init(struct platform_device *pdev,
struct global_data *pgd)
{
	pgd->top_entry = proc_mkdir("rpmsg_server", NULL);
	if (!pgd->top_entry) {
		pr_err("create top_entry error\n");
		goto create_top_entry_err;
	}

	return 0;
create_top_entry_err:
	return -1;
}

static void proc_top_dir_deinit(struct platform_device *pdev,
struct global_data *pgd)
{
	remove_proc_entry("rpmsg_server", NULL);
}

#define DEFAULT_BUFFER_SIZE (512)
static int rpmsg_char_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_char_port *port = NULL;
	int ret = 0;

	list_for_each_entry(port, &gd.chrdev_list, list) {
		if (!strcmp(port->server_name, rpdev->id.name))
			break;
	}
	pr_info("%s server probe\n", port->server_name);

	down_write(&port->valid_rwsem);

	if (port->valid) {
		pr_info("%s server already valid\n", port->server_name);
		up_write(&port->valid_rwsem);
		goto out;
	}

	port->write_buffer = (char *)kmalloc(DEFAULT_BUFFER_SIZE, GFP_KERNEL);
	if (!port->write_buffer) {
		pr_err("allocate write_buffer error\n");
		ret = -1;
		up_write(&port->valid_rwsem);
		goto out;
	}
	port->rpdev = rpdev;
	port->valid = 1;
	// enable annouce create
	rpdev->announce = 1;
	dev_set_drvdata(&rpdev->dev, port);

	up_write(&port->valid_rwsem);
out:
	return ret;
}

static void rpmsg_char_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_char_port *port = dev_get_drvdata(&rpdev->dev);
	struct sk_buff *skb = NULL;

	down_write(&port->valid_rwsem);
	// invalidate endpoint / rpmsg_device
	port->valid = 0;
	port->rpdev = NULL;
	rpdev->announce = 0;
	kfree(port->write_buffer);
	port->write_buffer = NULL;

	// free unreceived sk_buff
	spin_lock(&port->queue_lock);
	while (!skb_queue_empty(&port->queue)) {
		skb = skb_dequeue(&port->queue);
		kfree_skb(skb);
	}
	spin_unlock(&port->queue_lock);
	up_write(&port->valid_rwsem);

	// wakeup any blocked readers
	wake_up_interruptible(&port->readq);
}

static int rpmsg_char_cb(struct rpmsg_device *rpdev, void *data,
int len, void *priv, u32 src)
{
	struct rpmsg_char_port *port = dev_get_drvdata(&rpdev->dev);
	struct sk_buff *skb = NULL;
#ifdef TIMESTAMP_DEBUG
	struct timespec ts = current_kernel_time();
	pr_info("%s:%lds_%ldns\n", __func__, ts.tv_sec, ts.tv_nsec);
#endif

	// in our scenario, we call vring_interrupt in interrupt bottom half
	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb) {
		pr_err("alloc_skb fail\n");
		++port->statistics.alloc_skb_fail_count;
		return -ENOMEM;
	}
	++port->statistics.alloc_skb_count;

	skb_put_data(skb, data, len);
	spin_lock(&port->queue_lock);
	skb_queue_tail(&port->queue, skb);
	spin_unlock(&port->queue_lock);

	// wake up any blocked readers
	wake_up_interruptible(&port->readq);

	return 0;
}

static struct rpmsg_driver rpmsg_char_driver = {
	.drv.name = KBUILD_MODNAME,
	.drv.owner = THIS_MODULE,
	.probe = rpmsg_char_probe,
	.callback = rpmsg_char_cb,
	.remove = rpmsg_char_remove,
};

static const struct of_device_id hobot_rpmsg_char_dt_ids[] = {
	{.compatible = "hobot,rpmsg-char", },
	{ /* sentinel */ }
};

static int dts_init(struct platform_device *pdev, struct global_data *pgd)
{
	int nr_server = 0;
	const char *str = NULL;
	int ret = 0;
	int i = 0;

	ret = of_property_read_u32(pdev->dev.of_node, "nr-server",
	&nr_server);
	if (ret) {
		pr_err("get nr-server error\n");
		goto no_revoke_err;
	}
	pgd->nr_server = nr_server;
	pr_info("nr_server = %d\n", pgd->nr_server);

	pgd->id_table = (struct rpmsg_device_id *)kmalloc(
	(nr_server + 1) * sizeof(struct rpmsg_device_id), GFP_KERNEL);
	if (!pgd->id_table) {
		pr_err("kmalloc id_table error\n");
		goto no_revoke_err;
	}
	memset(pgd->id_table, 0, (nr_server + 1) * sizeof(struct rpmsg_device_id));

	for (i = 0; i < nr_server; ++i) {
		ret = of_property_read_string_index(pdev->dev.of_node,
		"server-list-string", i, &str);
		if (ret < 0) {
			pr_err("get server-list-string[%d] error\n", i);
			goto need_free_err;
		}

		if (strlen(str) > RPMSG_NAME_SIZE) {
			pr_err("server-list-string[%d] too long\n", i);
			goto need_free_err;
		}
		pr_info("server-list-string[%d] = %s\n", i, str);

		strcpy(pgd->id_table[i].name, str);
	}
	rpmsg_char_driver.id_table = pgd->id_table;

	return 0;
need_free_err:
	kfree(pgd->id_table);
no_revoke_err:
	return -1;
}

static void dts_deinit(struct platform_device *pdev, struct global_data *pgd)
{
	kfree(pgd->id_table);
}

static int rpmsg_chrdev_open(struct inode *inode, struct file *file)
{
	struct rpmsg_char_port *port = container_of(inode->i_cdev,
	struct rpmsg_char_port, cdev);
	file->private_data = port;

	down_read(&port->valid_rwsem);

	if (!port->valid) {
		//pr_err("%s server not start\n", port->server_name);
		up_read(&port->valid_rwsem);
		return -ENODEV;
	}

	spin_lock(&port->users_lock);
	++port->users;
	spin_unlock(&port->users_lock);

	up_read(&port->valid_rwsem);

	return 0;
}

static int rpmsg_chrdev_release(struct inode *inode, struct file *file)
{
	struct rpmsg_char_port *port = (struct rpmsg_char_port *)(file->private_data);

	spin_lock(&port->users_lock);
	--port->users;
	spin_unlock(&port->users_lock);

	down_read(&port->valid_rwsem);

	if (!port->valid) {
		//pr_err("%s server not start\n", port->server_name);
		up_read(&port->valid_rwsem);
		return -ENODEV;
	}

	up_read(&port->valid_rwsem);

	return 0;
}

#define DATA_OFFSET      (2)
#define CRC_VAL_SIZE     (2)

static short const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static unsigned short crc16(unsigned char *data, int length)
{
	unsigned short result = 0;
	unsigned short tableno = 0;
	int i = 0;

	for (i = 0; i < length; i++) {
		tableno = ((result & 0xff) ^ (data[i] & 0xff));
		result = ((result >> 8) & 0xff) ^ crc16_table[tableno];
		}

	return result;
}

static ssize_t rpmsg_chrdev_read(struct file *file, char __user *buf,
size_t size, loff_t *ppos)
{
	struct rpmsg_char_port *port = (struct rpmsg_char_port *)(file->private_data);
	int ret = 0;
	struct sk_buff *skb = NULL;
	unsigned short crc_val_expect = 0;
	unsigned short crc_val_cal = 0;
	int actual_len = 0;
#ifdef TIMESTAMP_DEBUG
	struct timespec ts = current_kernel_time();
#endif

	down_read(&port->valid_rwsem);

	if (!port->valid) {
		//pr_err("%s server not start\n", port->server_name);
		ret = -ENODEV;
		goto unlock_valid_out;
	}

	spin_lock(&port->queue_lock);
	if (skb_queue_empty(&port->queue)) {
		if (file->f_flags & O_NONBLOCK) {
			++port->statistics.recv_nonblock_fail_count;
			ret = -ENOMEM;
			goto unlock_queue_out;
		} else {
			spin_unlock(&port->queue_lock);
			up_read(&port->valid_rwsem);
			if (port->sleep_ms) {
				ret = wait_event_interruptible_timeout(port->readq,
				!skb_queue_empty(&port->queue) || !port->valid,
				msecs_to_jiffies(port->sleep_ms));
				down_read(&port->valid_rwsem);
				spin_lock(&port->queue_lock);

				if (!ret) {
					++port->statistics.recv_block_timeout_count;
					ret = -ETIMEDOUT;
					goto unlock_queue_out;
				}
			} else {
				ret = wait_event_interruptible(port->readq,
				!skb_queue_empty(&port->queue) || !port->valid);
				down_read(&port->valid_rwsem);
				spin_lock(&port->queue_lock);
			}

			if (ret < 0) {
				// interrupted by signal
				pr_err("read interrupted by signal\n");
				ret = -ERESTART;
				goto unlock_queue_out;
			}
		}
	}

	if (!port->valid) {
		//pr_err("%s server not start\n", port->server_name);
		ret = -ENODEV;
		goto unlock_queue_out;
	}

	skb = skb_dequeue(&port->queue);
	if (!skb) {
		pr_err("skb_dequeue error;");
		ret = -EFAULT;
		goto unlock_queue_out;
	}

	if (port->crc_check)
		actual_len = skb->len - CRC_VAL_SIZE;
	else
		actual_len = skb->len;

	if (actual_len > size) {
		// reclaim the sk_buff
		skb_queue_head(&port->queue, skb);
		pr_err("message is too big[%d > %ld]\n", actual_len, size);
		ret = -EMSGSIZE;
		goto unlock_queue_out;
	} else {
		spin_unlock(&port->queue_lock);
		if (port->crc_check) {
			memcpy(&crc_val_expect, skb->data, CRC_VAL_SIZE);
			crc_val_cal = crc16(skb->data + DATA_OFFSET,
			skb->len - CRC_VAL_SIZE);
			if (crc_val_cal != crc_val_expect) {
				pr_err("crc error[%d_%d]\n", crc_val_cal, crc_val_expect);
				++port->statistics.crc_error_count;
				ret = -EBADMSG;
				kfree_skb(skb);
				goto unlock_valid_out;
			}
			ret = copy_to_user(buf, skb->data + DATA_OFFSET,
			skb->len - CRC_VAL_SIZE);
		} else {
			ret = copy_to_user(buf, skb->data, skb->len);
		}
		if (ret) {
			pr_err("copy_to_user error\n");
			kfree_skb(skb);
			ret = -EFAULT;
			goto unlock_valid_out;
		}
		ret = actual_len;
		kfree_skb(skb);
	}

	up_read(&port->valid_rwsem);
	++port->statistics.recv_frame_count;
#ifdef TIMESTAMP_DEBUG
	pr_info("enter %s:%lds_%ldns\n", __func__, ts.tv_sec, ts.tv_nsec);
	ts = current_kernel_time();
	pr_info("left %s:%lds_%ldns\n", __func__, ts.tv_sec, ts.tv_nsec);
#endif
	return ret;
unlock_queue_out:
	spin_unlock(&port->queue_lock);
unlock_valid_out:
	up_read(&port->valid_rwsem);
	return ret;
}

static ssize_t rpmsg_chrdev_read_from_kernel(struct file *file,
char *buf, size_t size, loff_t *ppos)
{
	struct rpmsg_char_port *port = (struct rpmsg_char_port *)(file->private_data);
	int ret = 0;
	struct sk_buff *skb = NULL;
	unsigned short crc_val_expect = 0;
	unsigned short crc_val_cal = 0;
	int actual_len = 0;

	down_read(&port->valid_rwsem);

	if (!port->valid) {
		//pr_err("%s server not start\n", port->server_name);
		ret = -ENODEV;
		goto unlock_valid_out;
	}

	spin_lock(&port->queue_lock);
	if (skb_queue_empty(&port->queue)) {
		if (file->f_flags & O_NONBLOCK) {
			++port->statistics.recv_nonblock_fail_count;
			ret = -ENOMEM;
			goto unlock_queue_out;
		} else {
			spin_unlock(&port->queue_lock);
			up_read(&port->valid_rwsem);
			if (port->sleep_ms) {
				ret = wait_event_interruptible_timeout(port->readq,
				!skb_queue_empty(&port->queue) || !port->valid,
				msecs_to_jiffies(port->sleep_ms));
				down_read(&port->valid_rwsem);
				spin_lock(&port->queue_lock);

				if (!ret) {
					++port->statistics.recv_block_timeout_count;
					ret = -ETIMEDOUT;
					goto unlock_queue_out;
				}
			} else {
				ret = wait_event_interruptible(port->readq,
				!skb_queue_empty(&port->queue) || !port->valid);
				down_read(&port->valid_rwsem);
				spin_lock(&port->queue_lock);
			}

			if (ret < 0) {
				// interrupted by signal
				pr_err("read interrupted by signal\n");
				ret = -ERESTART;
				goto unlock_queue_out;
			}
		}
	}

	if (!port->valid) {
		//pr_err("%s server not start\n", port->server_name);
		ret = -ENODEV;
		goto unlock_queue_out;
	}

	skb = skb_dequeue(&port->queue);
	if (!skb) {
		pr_err("skb_dequeue error;");
		ret = -EFAULT;
		goto unlock_queue_out;
	}

	if (port->crc_check)
		actual_len = skb->len - CRC_VAL_SIZE;
	else
		actual_len = skb->len;

	if (actual_len > size) {
		// reclaim the sk_buff
		skb_queue_head(&port->queue, skb);
		pr_err("message is too big[%d > %ld]\n", actual_len, size);
		ret = -EMSGSIZE;
		goto unlock_queue_out;
	} else {
		spin_unlock(&port->queue_lock);
		if (port->crc_check) {
			memcpy(&crc_val_expect, skb->data, CRC_VAL_SIZE);
			crc_val_cal = crc16(skb->data + DATA_OFFSET,
			skb->len - CRC_VAL_SIZE);
			if (crc_val_cal != crc_val_expect) {
				pr_err("crc error[%d_%d]\n", crc_val_cal, crc_val_expect);
				++port->statistics.crc_error_count;
				ret = -EBADMSG;
				kfree_skb(skb);
				goto unlock_valid_out;
			}
			memcpy(buf, skb->data + DATA_OFFSET,
			skb->len - CRC_VAL_SIZE);
		} else {
			memcpy(buf, skb->data, skb->len);
		}
		ret = actual_len;
		kfree_skb(skb);
	}

	up_read(&port->valid_rwsem);
	++port->statistics.recv_frame_count;
	return ret;
unlock_queue_out:
	spin_unlock(&port->queue_lock);
unlock_valid_out:
	up_read(&port->valid_rwsem);
	return ret;
}

#define MS_PER_INTERVAL (5)
static ssize_t rpmsg_chrdev_write(struct file *file, const char __user *buf,
size_t count, loff_t *ppos)
{
	struct rpmsg_char_port *port = (struct rpmsg_char_port *)(file->private_data);
	int ret = 0;
	int sleep_count = 0;
	int remaining_time = 0;
	unsigned short crc_val = 0;

	down_read(&port->valid_rwsem);

	if (!port->valid) {
		//pr_err("%s server not start\n", port->server_name);
		ret = -ENODEV;
		goto unlock_valid_out;
	}

	if (port->crc_check) {
		if (count + CRC_VAL_SIZE > DEFAULT_BUFFER_SIZE) {
			ret = -EMSGSIZE;
			goto unlock_valid_out;
		}
	} else {
		if (count > DEFAULT_BUFFER_SIZE) {
			ret = -EMSGSIZE;
			goto unlock_valid_out;
		}
	}

	mutex_lock(&port->write_mutex);

	if (port->crc_check)
		ret = copy_from_user(port->write_buffer + DATA_OFFSET, buf, count);
	else
		ret = copy_from_user(port->write_buffer, buf, count);
	if (ret) {
		pr_err("copy_from_user error\n");
		ret = -EFAULT;
		goto unlock_write_out;
	}

	if (port->crc_check) {
		crc_val = crc16(port->write_buffer + DATA_OFFSET, count);
		memcpy(port->write_buffer, &crc_val, CRC_VAL_SIZE);
	}

	if (file->f_flags & O_NONBLOCK) {
		// nonblock
		if (port->crc_check)
			ret = rpmsg_trysend(port->rpdev->ept, port->write_buffer,
			count + CRC_VAL_SIZE);
		else
			ret = rpmsg_trysend(port->rpdev->ept, port->write_buffer, count);
		if (ret < 0) {
			++port->statistics.send_nonblock_fail_count;
			if (ret == -EMSGSIZE)
				goto unlock_write_out;

			ret = -ENOMEM;
			goto unlock_write_out;
		}
	} else {
		while (1) {
			if (port->crc_check)
				ret = rpmsg_trysend(port->rpdev->ept, port->write_buffer,
				count + CRC_VAL_SIZE);
			else
				ret = rpmsg_trysend(port->rpdev->ept, port->write_buffer,
			count);
			if (ret < 0) {
				if (ret == -EMSGSIZE)
					goto unlock_write_out;
				up_read(&port->valid_rwsem);
				remaining_time = msleep_interruptible(MS_PER_INTERVAL);
				down_read(&port->valid_rwsem);
				if (!port->valid) {
					//pr_err("%s server not start\n", port->server_name);
					ret = -ENODEV;
					goto unlock_write_out;
				}

				if (!remaining_time) {
					sleep_count += MS_PER_INTERVAL;
					if (port->sleep_ms && (sleep_count >= port->sleep_ms)) {
						++port->statistics.send_block_timeout_count;
						ret = -ETIMEDOUT;
						goto unlock_write_out;
					}
				} else {
					// interrupted by signal
					pr_err("write interrupted by signal\n");
					ret = -ERESTART;
					goto unlock_write_out;
				}
			} else {
				break;
			}
		}
	}
	++port->statistics.send_frame_count;
	ret = count;
unlock_write_out:
	mutex_unlock(&port->write_mutex);
unlock_valid_out:
	up_read(&port->valid_rwsem);
	return ret;
}

static ssize_t rpmsg_chrdev_write_from_kernel(struct file *file,
const char *buf, size_t count, loff_t *ppos)
{
	struct rpmsg_char_port *port = (struct rpmsg_char_port *)(file->private_data);
	int ret = 0;
	int sleep_count = 0;
	int remaining_time = 0;
	unsigned short crc_val = 0;

	down_read(&port->valid_rwsem);

	if (!port->valid) {
		//pr_err("%s server not start\n", port->server_name);
		ret = -ENODEV;
		goto unlock_valid_out;
	}

	if (port->crc_check) {
		if (count + CRC_VAL_SIZE > DEFAULT_BUFFER_SIZE) {
			ret = -EMSGSIZE;
			goto unlock_valid_out;
		}
	} else {
		if (count > DEFAULT_BUFFER_SIZE) {
			ret = -EMSGSIZE;
			goto unlock_valid_out;
		}
	}

	mutex_lock(&port->write_mutex);

	if (port->crc_check)
		memcpy(port->write_buffer + DATA_OFFSET, buf, count);
	else
		memcpy(port->write_buffer, buf, count);

	if (port->crc_check) {
		crc_val = crc16(port->write_buffer + DATA_OFFSET, count);
		memcpy(port->write_buffer, &crc_val, CRC_VAL_SIZE);
	}

	if (file->f_flags & O_NONBLOCK) {
		// nonblock
		if (port->crc_check)
			ret = rpmsg_trysend(port->rpdev->ept, port->write_buffer,
			count + CRC_VAL_SIZE);
		else
			ret = rpmsg_trysend(port->rpdev->ept, port->write_buffer, count);
		if (ret < 0) {
			++port->statistics.send_nonblock_fail_count;
			if (ret == -EMSGSIZE)
				goto unlock_write_out;

			ret = -ENOMEM;
			goto unlock_write_out;
		}
	} else {
		while (1) {
			if (port->crc_check)
				ret = rpmsg_trysend(port->rpdev->ept, port->write_buffer,
				count + CRC_VAL_SIZE);
			else
				ret = rpmsg_trysend(port->rpdev->ept, port->write_buffer,
			count);
			if (ret < 0) {
				if (ret == -EMSGSIZE)
					goto unlock_write_out;
				up_read(&port->valid_rwsem);
				remaining_time = msleep_interruptible(MS_PER_INTERVAL);
				down_read(&port->valid_rwsem);
				if (!port->valid) {
					//pr_err("%s server not start\n", port->server_name);
					ret = -ENODEV;
					goto unlock_write_out;
				}

				if (!remaining_time) {
					sleep_count += MS_PER_INTERVAL;
					if (port->sleep_ms && (sleep_count >= port->sleep_ms)) {
						++port->statistics.send_block_timeout_count;
						ret = -ETIMEDOUT;
						goto unlock_write_out;
					}
				} else {
					// interrupted by signal
					pr_err("write interrupted by signal\n");
					ret = -ERESTART;
					goto unlock_write_out;
				}
			} else {
				break;
			}
		}
	}
	++port->statistics.send_frame_count;
	ret = count;
unlock_write_out:
	mutex_unlock(&port->write_mutex);
unlock_valid_out:
	up_read(&port->valid_rwsem);
	return ret;
}

#define RPMSG_CHRDEV_IOC_MAGIC  'h'
#define RPMSG_IO_SET_TIMEOUT        _IO(RPMSG_CHRDEV_IOC_MAGIC, 0)
#define RPMSG_IO_SET_CRC_CHECK      _IO(RPMSG_CHRDEV_IOC_MAGIC, 1)
static long rpmsg_chrdev_ioctl(struct file *file, unsigned int cmd,
unsigned long arg)
{
	struct rpmsg_char_port *port = (struct rpmsg_char_port *)(file->private_data);
	int ret = 0;

	down_read(&port->valid_rwsem);

	if (!port->valid) {
		//pr_err("%s server not start\n", port->server_name);
		up_read(&port->valid_rwsem);
		return -ENODEV;
	}

	switch (cmd) {
	case RPMSG_IO_SET_TIMEOUT:
		port->sleep_ms = (int)arg;
		break;
	case RPMSG_IO_SET_CRC_CHECK:
		port->crc_check = 1;
		break;
	default:
		pr_err("invalid ioctl cmd\n");
		ret = -EINVAL;
		break;
	}

	up_read(&port->valid_rwsem);

	return ret;
}

static const struct file_operations rpmsg_chrdev_fops = {
	.owner = THIS_MODULE,
	.open = rpmsg_chrdev_open,
	.release = rpmsg_chrdev_release,
	.read = rpmsg_chrdev_read,
	.write = rpmsg_chrdev_write,
	.unlocked_ioctl = rpmsg_chrdev_ioctl,
	.compat_ioctl = rpmsg_chrdev_ioctl,
};

static int create_chrport(struct global_data *pgd,
int index)
{
	struct rpmsg_char_port *port = NULL;
	dev_t devno = 0;
	int ret = 0;
	struct device *cdevice = NULL;

	// allocate memory
	port = (struct rpmsg_char_port *)kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port) {
		pr_err("malloc %d rpmsg_char_port error\n", index);
		goto malloc_err;
	}
	strcpy(port->server_name, pgd->id_table[index].name);

	// create character device
	devno = MKDEV(pgd->rpmsg_chrdev_major, index);
	cdev_init(&port->cdev, &rpmsg_chrdev_fops);
	port->cdev.owner = THIS_MODULE;
	ret = cdev_add(&port->cdev, devno, 1);
	if (ret < 0) {
		pr_err("cdev_add[%d] error\n", index);
		goto cdev_add_err;
	}
	port->devno = devno;

	// create device node
	cdevice = device_create(pgd->rpmsg_chrdev_class, NULL, devno,
	NULL, port->server_name);
	if (IS_ERR(cdevice)) {
		pr_err("device_create[/dev/%s] error\n", port->server_name);
		goto device_create_err;
	}

	// create /proc node
	port->server_entry = proc_mkdir(port->server_name, pgd->top_entry);
	if (!port->server_entry) {
		pr_err("create server_entry error\n");
		goto create_server_entry_err;
	}

	port->statistics_entry = proc_create_data("statistics", 0777,
	port->server_entry, &statistics_ops, port);
	if (!port->statistics_entry) {
		pr_err("create statistics_entry error\n");
		goto create_statistics_entry_err;
	}

	init_rwsem(&port->valid_rwsem);
	mutex_init(&port->write_mutex);
	port->valid = 0;
	spin_lock_init(&port->users_lock);
	port->users = 0;
	port->sleep_ms = 0;
	spin_lock_init(&port->queue_lock);
	skb_queue_head_init(&port->queue);
	init_waitqueue_head(&port->readq);
	INIT_LIST_HEAD(&port->list);
	port->crc_check = 0;

	list_add_tail(&port->list, &pgd->chrdev_list);

	return 0;
create_statistics_entry_err:
	remove_proc_entry(port->server_name, pgd->top_entry);
create_server_entry_err:
	device_destroy(pgd->rpmsg_chrdev_class, devno);
device_create_err:
	cdev_del(&port->cdev);
cdev_add_err:
	kfree(port);
malloc_err:
	return -1;
}

static void destroy_chrport(struct rpmsg_char_port *port,
struct global_data *pgd)
{
	remove_proc_entry("statistics", port->server_entry);
	remove_proc_entry(port->server_name, pgd->top_entry);
	device_destroy(pgd->rpmsg_chrdev_class, port->devno);
	cdev_del(&port->cdev);
	kfree(port);
}

static int rpmsg_char_init(struct global_data *pgd)
{
	int ret = 0;
	int i = 0;
	struct rpmsg_char_port *port = NULL;
	struct rpmsg_char_port *n = NULL;
	dev_t devno = 0;

	ret = alloc_chrdev_region(&devno, 0, pgd->nr_server,
	"rpmsg_chrdev");
	if (ret < 0) {
		pr_err("alloc_chrdev_region error\n");
		goto alloc_chrdev_region_err;
	}
	pgd->devno_from = devno;
	pgd->rpmsg_chrdev_major = MAJOR(devno);

	pgd->rpmsg_chrdev_class = class_create(THIS_MODULE, "rpmsg_chrdev");
	if (IS_ERR(pgd->rpmsg_chrdev_class)) {
		pr_err("class_create error\n");
		goto class_create_err;
	}

	INIT_LIST_HEAD(&pgd->chrdev_list);

	for (i = 0; i < pgd->nr_server; ++i) {
		ret = create_chrport(pgd, i);
		if (ret < 0) {
			pr_err("create_chrport[%d] error\n", i);
			break;
		}
	}
	if (i < pgd->nr_server)
		goto create_chrdev_err;

	return 0;
create_chrdev_err:
	list_for_each_entry_safe(port, n, &pgd->chrdev_list, list) {
		list_del(&port->list);
		destroy_chrport(port, pgd);
	}
class_create_err:
	unregister_chrdev_region(devno, pgd->nr_server);
alloc_chrdev_region_err:
	return -1;
}

static void rpmsg_char_deinit(struct global_data *pgd)
{
	struct rpmsg_char_port *port = NULL;
	struct rpmsg_char_port *n = NULL;

	list_for_each_entry_safe(port, n, &pgd->chrdev_list, list) {
		list_del(&port->list);
		destroy_chrport(port, pgd);
	}
	class_destroy(pgd->rpmsg_chrdev_class);
	unregister_chrdev_region(pgd->devno_from, pgd->nr_server);
}

static int hobot_rpmsg_char_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("hobot_rpmsg_char_probe start\n");

	ret = dts_init(pdev, &gd);
	if (ret < 0) {
		pr_err("dts_init error\n");
		goto dts_init_err;
	}

	ret = proc_top_dir_init(pdev, &gd);
	if (ret < 0) {
		pr_err("proc_top_dir_init error\n");
		goto create_top_dir_err;
	}

	ret = rpmsg_char_init(&gd);
	if (ret < 0) {
		pr_err("rpmsg_char_init error\n");
		goto rpmsg_char_init_err;
	}

	ret = register_rpmsg_driver(&rpmsg_char_driver);
	if (ret < 0) {
		pr_err("register_rpmsg_driver error\n");
		goto register_rpmsg_driver_err;
	}

	pr_info("hobot_rpmsg_char_probe end\n");

	return 0;
register_rpmsg_driver_err:
	rpmsg_char_deinit(&gd);
rpmsg_char_init_err:
	proc_top_dir_deinit(pdev, &gd);
create_top_dir_err:
	dts_deinit(pdev, &gd);
dts_init_err:
	return -1;
}

static struct platform_driver hobot_rpmsg_char_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "hobot-rpmsg-char",
		.of_match_table = hobot_rpmsg_char_dt_ids,
	},
	.probe = hobot_rpmsg_char_probe,
};

static int __init hobot_rpmsg_char_init(void)
{
	int ret = 0;

	pr_info("hobot_rpmsg_char_init start\n");

	ret = platform_driver_register(&hobot_rpmsg_char_driver);
	if (ret)
		pr_err("platform_driver_register error\n");

	pr_info("hobot_rpmsg_char_init end\n");

	return ret;
}

#define rpmsg_error pr_err
#define rpmsg_debug (void)

struct rpmsg_handle {
	char server_name[RPMSG_NAME_SIZE];
	struct file *fp;
};

enum rpmsg_err_enum {
	INVALID_ARG = 1,
	INVALID_SERVER,
	OUT_OF_RES,
	KER_USR_TRANS,
	SEND_BUF_OVERSIZE,
	NO_MEM,
	TIMEOUT,
	SIGNAL_STOP,
	RECV_BUF_OVERFLOW,
	NOT_START_SERVER,
	CRC_ERROR,
	RPMSG_ERR_NR
};

struct error_message {
	int error_code;
	char *error_message;
};
struct error_message message[RPMSG_ERR_NR] = {
	{0, NULL},
	{INVALID_ARG, "invalid arguments"},
	{INVALID_SERVER, "server not support"},
	{OUT_OF_RES, "local resource allocate fail"},
	{KER_USR_TRANS, "kernel & userspace data transfer error"},
	{SEND_BUF_OVERSIZE, "send message is too big"},
	{NO_MEM, "no comminication buffer available"},
	{TIMEOUT, "transfer block timeout"},
	{SIGNAL_STOP, "operation stopped by signal"},
	{RECV_BUF_OVERFLOW, "receive buffer in not enough"},
	{NOT_START_SERVER, "server not start"},
	{CRC_ERROR, "crc check error"},
};

/*
* hb_rpmsg_connect_server - establish connection with communication service
* @server_name: communication service name
* @flags: transfer features, such as block or nonblock
* @timeout: timeout in block transfer mode in microseconds
* @handle: out parameter, return communication handle
* return: on success return 0; if fail return negative error code
*/
#define DEVICE_NAME_LEN_MAX (64)
#define DEVICE_NAME_PREFIX "/dev/"
int hb_rpmsg_connect_server(char *server_name, int flags, int timeout,
struct rpmsg_handle **handle)
{
	int ret = 0;
	int block = flags & RPMSG_F_BLOCK;
	int crc_check = flags & RPMSG_F_CRC_CHECK;
	int open_flags = 0;
	struct rpmsg_handle *phandle = NULL;
	char device_name[DEVICE_NAME_LEN_MAX] = {0};
	struct file *fp = NULL;
	int errno = 0;
	struct rpmsg_char_port *port = NULL;

	// parameter check
	if (!server_name || !handle) {
		rpmsg_error("invalid pointer\n");
		ret = RPMSG_ERR_INVALID_ARG;
		goto no_revoke_out;
	}

	if (strlen(server_name) > RPMSG_NAME_SIZE) {
		rpmsg_error("server_name is too long\n");
		ret = RPMSG_ERR_INVALID_ARG;
		goto no_revoke_out;
	}

	if (block && timeout < 0) {
		rpmsg_error("block with negative timeout\n");
		ret = RPMSG_ERR_INVALID_ARG;
		goto no_revoke_out;
	}

	// malloc rpmsg handle
	phandle = (struct rpmsg_handle *)kmalloc(
		sizeof(struct rpmsg_handle), GFP_KERNEL);
	if (!phandle) {
		rpmsg_error("malloc rpmsg handle fail\n");
		ret = RPMSG_ERR_OUT_OF_RES;
		goto no_revoke_out;
	}
	strcpy(phandle->server_name, server_name);

	// open device
	strcpy(device_name, DEVICE_NAME_PREFIX);
	strcat(device_name, phandle->server_name);
	open_flags = O_RDWR;
	if (!block)
		open_flags |= O_NONBLOCK;
	fp = filp_open(device_name, open_flags, 0644);
	if (IS_ERR(fp)) {
		errno = PTR_ERR(fp);
		switch (errno) {
		case -ENODEV:
			//rpmsg_debug("%s server not start\n", phandle->server_name);
			ret = RPMSG_ERR_NOT_START_SERVER;
			break;
		case -ENOENT:
			//rpmsg_debug("%s server not support\n", phandle->server_name);
		ret = RPMSG_ERR_INVALID_SERVER;
			break;
		default:
			//rpmsg_error("unclassed error[%d]\n", errno);
			ret = -RPMSG_ERR_NR;
			break;
		}
		goto free_handle_out;
	}

	// set timeout
	port = (struct rpmsg_char_port *)(fp->private_data);
	if (block && timeout >= 0)
		port->sleep_ms = timeout;
	phandle->fp = fp;

	// set crc check
	if (crc_check)
		port->crc_check = 1;

	*handle = phandle;
	return 0;
free_handle_out:
	kfree(phandle);
no_revoke_out:
	return ret;
}
EXPORT_SYMBOL(hb_rpmsg_connect_server);

/*
* hb_rpmsg_disconnect_server - abolish connection with communication service specified by handle
* @handle: communicatin handle want to disconnect
* return: on success return 0; if fail return negative error code
*/
int hb_rpmsg_disconnect_server(struct rpmsg_handle *handle)
{
	int ret = 0;

	// parameter check
	if (!handle) {
		rpmsg_error("invalid pointer\n");
		ret = RPMSG_ERR_INVALID_ARG;
		goto out;
	}

	// close device
	ret = filp_close(handle->fp, NULL);
	if (ret < 0) {
		switch(ret) {
		case -ENODEV:
			//rpmsg_error("%s server not start\n", handle->server_name);
			ret = RPMSG_ERR_NOT_START_SERVER;
			break;
		case -ENOENT:
			rpmsg_error("%s server not support\n", handle->server_name);
		ret = RPMSG_ERR_INVALID_SERVER;
			break;
		default:
			rpmsg_error("unclassed error[%d]\n", ret);
			ret = -RPMSG_ERR_NR;
			break;
		}
	}
	kfree(handle);
out:
	return ret;
}
EXPORT_SYMBOL(hb_rpmsg_disconnect_server);

/*
* hb_rpmsg_send - send frame on specific communication service
* @handle: communication handle want to send
* @buf: contain frame data want to send
* @len: data length want to send
* return: on success return the number of bytes sent; if fail return negative error code
*/
int hb_rpmsg_send(struct rpmsg_handle *handle, char *buf, int len)
{
	int ret = 0;
	loff_t offset = 0;

	// parameter check
	if (!handle || !buf || len <= 0) {
		rpmsg_error("invalid pointer\n");
		ret = RPMSG_ERR_INVALID_ARG;
		goto out;
	}

	// send frame
	ret = rpmsg_chrdev_write_from_kernel(handle->fp, buf, len, &offset);
	if (ret < 0) {
		switch (ret) {
		case -EMSGSIZE:
			rpmsg_error("message is too big\n");
			ret = RPMSG_ERR_SEND_BUF_OVERSIZE;
			break;
		case -EFAULT:
			rpmsg_error("transfer data error\n");
			ret = RPMSG_ERR_KER_USR_TRANS;
			break;
		case -ENOMEM:
			rpmsg_error("no tx buffer\n");
			ret = RPMSG_ERR_NO_MEM;
			break;
		case -ETIMEDOUT:
			rpmsg_error("send timeout\n");
			ret = RPMSG_ERR_TIMEOUT;
			break;
		case -ERESTART:
			rpmsg_error("stop by signal\n");
			ret = RPMSG_ERR_SIGNAL_STOP;
			break;
		case -ENODEV:
			//rpmsg_error("%s server not start\n", handle->server_name);
			ret = RPMSG_ERR_NOT_START_SERVER;
			break;
		case -ENOENT:
			rpmsg_error("%s server not support\n", handle->server_name);
			ret = RPMSG_ERR_INVALID_SERVER;
			break;
		default:
			rpmsg_error("unclassed error[%d]\n", ret);
			ret = -RPMSG_ERR_NR;
			break;
		}
	}
out:
	return ret;
}
EXPORT_SYMBOL(hb_rpmsg_send);

/*
* hb_rpmsg_recv - receive frame on specific communication service
* @handle: communication handle want to send
* @buf: contain received data
* @len: received data length
* return: on success return the number of bytes received; if fail return negative error code
*/
int hb_rpmsg_recv(struct rpmsg_handle *handle, char *buf, int len)
{
	int ret = 0;
	loff_t offset = 0;

	// parameter check
	if (!handle || !buf || len <= 0) {
		rpmsg_error("invalid pointer\n");
		ret = RPMSG_ERR_INVALID_ARG;
		goto out;
	}

	// receive frame
	ret = rpmsg_chrdev_read_from_kernel(handle->fp, buf, len, &offset);
	if (ret < 0) {
		switch (ret) {
		case -ENOMEM:
			rpmsg_error("no message\n");
			ret = RPMSG_ERR_NO_MEM;
			break;
		case -ETIMEDOUT:
			rpmsg_error("recv timeout\n");
			ret = RPMSG_ERR_TIMEOUT;
			break;
		case -ERESTART:
			rpmsg_error("stop by signal\n");
			ret = RPMSG_ERR_SIGNAL_STOP;
			break;
		case -EMSGSIZE:
			rpmsg_error("message is too big\n");
			ret = RPMSG_ERR_RECV_BUF_OVERFLOW;
			break;
		case -EFAULT:
			rpmsg_error("transfer data error\n");
			ret = RPMSG_ERR_KER_USR_TRANS;
			break;
		case -ENODEV:
			//rpmsg_error("%s server not start\n", handle->server_name);
			ret = RPMSG_ERR_NOT_START_SERVER;
			break;
		case -ENOENT:
			rpmsg_error("%s server not support\n", handle->server_name);
			ret = RPMSG_ERR_INVALID_SERVER;
			break;
		case -EBADMSG:
			rpmsg_error("crc check error\n");
			ret = RPMSG_ERR_CRC_CHECK;
			break;
		default:
			rpmsg_error("unclassed error[%d]\n", ret);
			ret = -RPMSG_ERR_NR;
			break;
		}
	}
out:
	return ret;
}
EXPORT_SYMBOL(hb_rpmsg_recv);

/*
* hb_rpmsg_error_message - return string type error message according to error_code
* @error_code: error code return from rpmsg lib API
* return: string type error message
*/
#define INVALID_ERROR_CODE "invalid error code"
#define UNCLASSED_ERROR_CODE "unclassed error code"
char *hb_rpmsg_error_message(int error_code)
{
	error_code = -error_code;

	if ((error_code < 1) || (error_code > RPMSG_ERR_NR))
		return INVALID_ERROR_CODE;
	else if (error_code == RPMSG_ERR_NR)
		return UNCLASSED_ERROR_CODE;
	else
		return message[error_code].error_message;
}
EXPORT_SYMBOL(hb_rpmsg_error_message);

MODULE_AUTHOR("Horizon Robotics, Inc");
MODULE_DESCRIPTION("Hobot virtio remote processor messaging character device driver");
MODULE_LICENSE("GPL");
subsys_initcall(hobot_rpmsg_char_init);

