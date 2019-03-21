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
#include "bif_lite_utility.h"
#include "bif_platform.h"
#include "hbipc_lite.h"
#include "hbipc_errno.h"

/*reg driver info*/
#define BIF_MAJOR 249
/*reg mode attr*/

/* ioctl cmd */
#define BIF_IOC_MAGIC  'c'
#define BIF_IO_PROVIDER_INIT     _IO(BIF_IOC_MAGIC, 0)
#define BIF_IO_PROVIDER_DEINIT   _IO(BIF_IOC_MAGIC, 1)
#define BIF_IO_ACCEPT            _IO(BIF_IOC_MAGIC, 2)
#define BIF_IO_START_SERVER      _IO(BIF_IOC_MAGIC, 3)
#define BIF_IO_STOP_SERVER       _IO(BIF_IOC_MAGIC, 4)
#define BIF_IO_CONNECT           _IO(BIF_IOC_MAGIC, 5)
#define BIF_IO_DISCONNECT        _IO(BIF_IOC_MAGIC, 6)

struct x2_bif_data {
	unsigned int users;
};
struct x2_bif_data bif_data;

static struct class  *g_bif_class;
struct device *g_bif_dev;

int x2_bif_data_init(struct x2_bif_data *bif_data)
{
	bif_data->users = 0;

	return 0;
}

static DEFINE_MUTEX(open_mutex);
static int x2_bif_open(struct inode *inode, struct file *file)
{
	mutex_lock(&open_mutex);

	if (!bif_data.users) {
		file->private_data = g_bif_dev;
		bif_start();

		++bif_data.users;
		//hbipc_debug("real x2_bif_open: %d\n",bif_data.users);
	} else {
		file->private_data = g_bif_dev;
		++bif_data.users;
		//hbipc_debug("ref x2_bif_open: %d\n", bif_data.users);
	}

	mutex_unlock(&open_mutex);

	return 0;
}

static DEFINE_MUTEX(write_mutex);
#define PHY_LAYER_LEN_MAX (16 * 1024)
char send_frame[PHY_LAYER_LEN_MAX];
static ssize_t x2_bif_write(struct file *file, const char __user *buf,
size_t count, loff_t *ppos)
{
	int ret = 0;
	struct send_mang_data data;
	int status = 0;

	mutex_lock(&write_mutex);

	if (copy_from_user(&data, (const char __user *)buf, sizeof(data))) {
		hbipc_error("copy_from_user_fail\n");
		ret = -EFAULT;
		goto error;
	}

	if (!is_valid_session(&data, NULL, NULL)) {
		hbipc_error("invalid session\n");
		ret = -1;
		data.result = HBIPC_ERROR_INVALID_SESSION;
		status = copy_to_user((void __user *)buf, &data, sizeof(data));
		if (status)
			ret = -EFAULT;
		goto error;
	}

	if (copy_from_user(send_frame, (const char __user *)data.buffer,
	data.len)) {
		hbipc_error("copy user frame error\n");
		ret = -EFAULT;
		goto error;
	}

	mutex_lock(&domain.write_mutex);
	ret = bif_tx_put_frame(send_frame, data.len);
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

	mutex_lock(&read_mutex);

	if (copy_from_user(&data, (const char __user *)buf, sizeof(data))) {
		hbipc_error("copy_from_user_fail\n");
		ret = -EFAULT;
		goto error;
	}

	session_des = is_valid_session(&data, NULL, NULL);
	if (!session_des) {
		hbipc_error("invalid session\n");
		ret = -1;
		data.result = HBIPC_ERROR_INVALID_SESSION;
		status = copy_to_user((void __user *)buf, &data, sizeof(data));
		if (status)
			ret = -EFAULT;
		goto error;
	}

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
			bif_del_frame_from_list(frame);
		}
	} else {
		// no specific data frame get
		ret = 0;
	}

	mutex_unlock(&read_mutex);

	return ret;
error:
	mutex_unlock(&read_mutex);
	return ret;
}

static int x2_bif_close(struct inode *inode, struct file *file)
{
	mutex_lock(&open_mutex);

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
	int provider_id = 0;
	struct provider_server *relation = NULL;

	mutex_lock(&ioctl_mutex);

	switch (cmd) {
	case BIF_IO_PROVIDER_INIT:
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		ret = register_server_provider(&data);
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

		ret = unregister_server_provider(&data);
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
		if (copy_from_user(&data, (const char __user *)arg,
		sizeof(data))) {
			hbipc_error("copy_from_user_fail\n");
			ret = -1;
			break;
		}

		ret = accept_session(&data, &connect);
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

		ret = start_server(&data, &provider_id);
		if (ret > 0) {
			data.provider_id = ret;
			register_map_with_lock(&data);
			ret = 0;
		}
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

		ret = get_map_index_with_lock(&domain.map, data.provider_id);
		if (ret < 0) {
			hbipc_error("invalid provider\n");
			data.result = HBIPC_ERROR_INVALID_PROVIDERID;
		} else {
			unregister_map_with_lock(&data);
			ret = 0;
		}
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

		ret = get_map_index_with_lock(&domain.map, data.provider_id);
		if (ret < 0) {
			hbipc_error("invalid provider\n");
			data.result = HBIPC_ERROR_INVALID_PROVIDERID;
		} else {
			relation = domain.map.map_array + ret;
			memcpy(data.server_id, relation->server_id, UUID_LEN);

			ret = register_connect(&data);
			if (ret < 0) {
				hbipc_error("register connect error\n");
				data.result = ret;
			}
		}
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

		ret = get_map_index_with_lock(&domain.map, data.provider_id);
		if (ret < 0) {
			hbipc_error("invalid provider\n");
			data.result = HBIPC_ERROR_INVALID_PROVIDERID;
		} else {
			relation = domain.map.map_array + ret;
			memcpy(data.server_id, relation->server_id, UUID_LEN);

			ret = unregister_connect(&data);
			if (ret < 0) {
				hbipc_error("unregister connect error\n");
				data.result = ret;
			}
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

static const  struct file_operations bif_fops = {
	.owner           = THIS_MODULE,
	.open            = x2_bif_open,
	.write           = x2_bif_write,
	.read            = x2_bif_read,
	.poll            = x2_bif_poll,
	.release         = x2_bif_close,
	.unlocked_ioctl  = x2_bif_ioctl,
	.compat_ioctl    = x2_bif_ioctl,
};

static int    bif_major;
struct cdev   bif_cdev;
static int __init bif_module_init(void)
{
	int           ret = 0;
	dev_t         devno;
	struct cdev  *p_cdev = &bif_cdev;

	bif_major = 0;
	bif_debug("bif driver init enter\n");
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

	bif_lite_init();
	x2_bif_data_init(&bif_data);
	domain_init(&domain, &domain_config);

	bif_debug("bif driver init exit\n");
	return 0;
device_create_error:
	class_destroy(g_bif_class);
class_create_error:
	cdev_del(&bif_cdev);
cdev_add_error:
	unregister_chrdev_region(MKDEV(bif_major, 0), 1);
alloc_chrdev_error:
		return ret;
}

static void __exit bif_module_exit(void)
{
	bif_debug("[%s:%d] bif_exit\n", __func__, __LINE__);
	domain_deinit(&domain);
	device_destroy(g_bif_class, MKDEV(bif_major, 0));
	class_destroy(g_bif_class);
	cdev_del(&bif_cdev);
	unregister_chrdev_region(MKDEV(bif_major, 0), 1);
}
module_init(bif_module_init);
module_exit(bif_module_exit);
MODULE_LICENSE("GPL v2");
