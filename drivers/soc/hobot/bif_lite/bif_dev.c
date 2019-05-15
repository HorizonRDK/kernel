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
#include "bif_lite_utility.h"
#include "bif_platform.h"
#include "hbipc_lite.h"
#include "hbipc_errno.h"

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
	int users;
	int irq_pin;
	int irq_num;
	int tri_pin;
};
struct x2_bif_data bif_data;

static struct class  *g_bif_class;
struct device *g_bif_dev;

int x2_bif_data_init(struct x2_bif_data *bif_data)
{
	bif_data->users = 0;
	bif_data->irq_pin = -1;
	bif_data->tri_pin = -1;
	bif_data->irq_num = -1;

	return 0;
}

static DEFINE_MUTEX(open_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
static int bif_lite_init_success = 0;
#endif
static int x2_bif_open(struct inode *inode, struct file *file)
{
	mutex_lock(&open_mutex);

	if (!bif_data.users) {
		file->private_data = g_bif_dev;
#ifdef CONFIG_HOBOT_BIF_AP
		if (!bif_lite_init_success) {
			if (bif_lite_init() < 0) {
				printk("bif_lite_init error\n");
				return -EPERM;
			} else
				bif_lite_init_success = 1;
				bif_lite_register_irq(hbipc_irq_handler);
		}
#endif
		//bif_start();
		recv_handle_stock_frame();
		++bif_data.users;
	} else {
		file->private_data = g_bif_dev;
		++bif_data.users;
	}

	mutex_unlock(&open_mutex);

	return 0;
}

static DEFINE_MUTEX(write_mutex);
//#define PHY_LAYER_LEN_MAX (16 * 1024)
#define PHY_LAYER_LEN_MAX (128 * 1024)
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
	struct list_head *pos = NULL;

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
	if (ret < 0)
		goto error;

	if (list_empty(&(session_des->recv_list.list))) {
		ret = -1;
		data.result = HBIPC_ERROR_INVALID_SESSION;
		status = copy_to_user((void __user *)buf, &data, sizeof(data));
		if (status)
			ret = -EFAULT;
		goto error;
	}

	pos = session_des->recv_list.list.next;
	frame = list_entry(pos, struct bif_frame_cache, frame_cache_list);
	if (frame->framelen - HBIPC_HEADER_LEN > data.len) {
		hbipc_error("recv buf overflow\n");
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
		ret = -EFAULT;
	} else {
		ret = header->length;
		// consume a data frame really
		--session_des->recv_list.frame_count;
		mutex_lock(&domain.read_mutex);
		spin_lock(&(session_des->recv_list.lock));
		bif_del_frame_from_list(frame);
		spin_unlock(&(session_des->recv_list.lock));
		mutex_unlock(&domain.read_mutex);
	}

	mutex_unlock(&read_mutex);

	return ret;
error:
	mutex_unlock(&read_mutex);
	return ret;
}

static int x2_bif_close(struct inode *inode, struct file *file)
{
	struct send_mang_data data;

	mutex_lock(&open_mutex);

	pr_info("pid = %d\n", current->pid);
#ifndef CONFIG_HOBOT_BIF_AP
	data.domain_id = domain.domain_id;
	data.provider_id = current->pid;
	unregister_server_provider_abnormal(&data);
#else
	data.domain_id = domain.domain_id;
	data.client_id = current->pid;
	disconnect_stopserver_abnormal(&data);
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
	//int provider_id = 0;
	struct provider_server *relation = NULL;
	struct session_desc *session_des = NULL;
	struct provider_start_desc *provider_start_des = NULL;

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
		printk("accept\n");
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
		recv_handle_manage_frame();
		// check whethrer map created
		mutex_lock(&domain.connect_mutex);
		ret = get_map_index_from_server(&domain.map, &data);
		// map have been created
		if (ret >= 0) {
			relation = domain.map.map_array + ret;
			ret = get_start_index(&relation->start_list,
			data.client_id);
			if (ret < 0) {
				// current client didn't start this provider
				if (relation->start_list.count <= 0)
					data.result =
				HBIPC_ERROR_RMT_RES_ALLOC_FAIL;
				else {
					provider_start_des =
					relation->start_list.start_array +
					relation->start_list.first_avail;
					provider_start_des->valid = 1;
					provider_start_des->client_id =
					data.client_id;
					--relation->start_list.count;
					relation->start_list.first_avail =
		get_start_list_first_avail_index(&relation->start_list);
		data.provider_id = relation->provider_id;
			ret = 0;
		}
			} else {
		// current client has already started this provider
				data.result = HBIPC_ERROR_REPEAT_STARTSERVER;
				ret = 0;
			}
		}
		mutex_unlock(&domain.connect_mutex);
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

		mutex_lock(&domain.connect_mutex);
		ret = get_map_index(&domain.map, data.provider_id);
		if (ret < 0) {
			hbipc_error("invalid provider\n");
			data.result = HBIPC_ERROR_INVALID_PROVIDERID;
		} else {
			relation = domain.map.map_array + ret;
			ret =
			get_start_index(&relation->start_list, data.client_id);
			if (ret < 0)
				data.result = HBIPC_ERROR_INVALID_PROVIDERID;
			else {
				provider_start_des =
				relation->start_list.start_array + ret;
				provider_start_des->valid = 0;
				++relation->start_list.count;
				relation->start_list.first_avail =
		get_start_list_first_avail_index(&relation->start_list);
			}
		}
		mutex_unlock(&domain.connect_mutex);
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

		session_des = is_valid_session(&data, NULL, NULL);
		if (session_des) {
			data.result = HBIPC_ERROR_REPEAT_CONNECT;
			ret = -1;
			goto connect_out;
		}

			ret = register_connect(&data);
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

			ret = unregister_connect(&data);
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

static const struct of_device_id bif_lite_of_match[] = {
	{.compatible = "hobot,bif_lite"},
	{},
};

#if 0
static irqreturn_t bif_lite_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}
#endif

static int    bif_major;
struct cdev   bif_cdev;

#ifndef CONFIG_HI3519V101
static int bif_lite_probe(struct platform_device *pdev)
{
	int           ret = 0;
	dev_t         devno;
	struct cdev  *p_cdev = &bif_cdev;
#if 0
	unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;
#endif

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

	x2_bif_data_init(&bif_data);
#if 0
	ret = of_property_read_u32(pdev->dev.of_node, "bif_lite_irq_pin", &bif_data.irq_pin);
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
	ret = request_threaded_irq(bif_data.irq_num, bif_lite_irq_handler, NULL, flags, "bif-lite-driver", (void *)&bif_data);
	if (ret) {
		bif_err("request irq error\n");
		goto request_irq_error;
	} else
		bif_debug("irq_pin = %d irq_num = %d\n", bif_data.irq_pin, bif_data.irq_num);

	ret = of_property_read_u32(pdev->dev.of_node, "bif_lite_tri_pin", &bif_data.tri_pin);
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
#ifndef CONFIG_HOBOT_BIF_AP
	ret = bif_lite_init();

	if (ret < 0) {
		bif_err("bif_lite_init error\n");
		goto bif_lite_init_error;
	}
	bif_lite_register_irq(hbipc_irq_handler);
#endif
	domain_init(&domain, &domain_config);

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
	device_destroy(g_bif_class, MKDEV(bif_major, 0));
#endif
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

	x2_bif_data_init(&bif_data);
#if 0
	ret = of_property_read_u32(pdev->dev.of_node, "bif_lite_irq_pin", &bif_data.irq_pin);
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
	ret = request_threaded_irq(bif_data.irq_num, bif_lite_irq_handler, NULL, flags, "bif-lite-driver", (void *)&bif_data);
	if (ret) {
		bif_err("request irq error\n");
		goto request_irq_error;
	} else
		bif_debug("irq_pin = %d irq_num = %d\n", bif_data.irq_pin, bif_data.irq_num);

	ret = of_property_read_u32(pdev->dev.of_node, "bif_lite_tri_pin", &bif_data.tri_pin);
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
#ifndef CONFIG_HOBOT_BIF_AP
	ret = bif_lite_init();

	if (ret < 0) {
		bif_err("bif_lite_init error\n");
		goto bif_lite_init_error;
	}
	bif_lite_register_irq(hbipc_irq_handler);
#endif
	domain_init(&domain, &domain_config);

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
	device_destroy(g_bif_class, MKDEV(bif_major, 0));
#endif
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
	domain_deinit(&domain);
	bif_lite_exit();
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
	domain_deinit(&domain);
	bif_lite_exit();
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
		.name = "bif_lite",
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
