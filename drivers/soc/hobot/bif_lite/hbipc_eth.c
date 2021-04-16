#include <linux/in.h>
#include <linux/inet.h>
#include <linux/socket.h>
#include <net/sock.h>
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
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "hbipc_eth.h"

#ifdef CONFIG_NO_DTS_AP
/* module parameters */
static char *ip_addr;
static char *if_name;
module_param(ip_addr, charp, 0644);
module_param(if_name, charp, 0644);
#endif

#define BUF_LEN (20)
static unsigned short portnum = 9527;
struct hbipc_eth_ctx_t {
	int ready;
	spinlock_t ready_lock;
	int bind_flag;
	char ip_addr[BUF_LEN];
	char if_name[BUF_LEN];
	struct workqueue_struct *work_queue;
	struct work_struct work;
	struct completion start_to_connect;
	struct socket *server_sock;
	struct socket *client_sock;
	struct mutex release_client_lock;
	struct sockaddr_in server_addr;
	struct ifreq ifr;
};

struct hbipc_eth_ctx_t hbipc_eth_ctx;

#define CONNECT_RETRY_INTERVAL (100)
static int establish_connect_stop;
static void establish_connect(struct work_struct *work)
{
	int ready_local = 0;
	int ret = 0;

	while (1) {
		if (wait_for_completion_interruptible(&hbipc_eth_ctx.start_to_connect) < 0) {
			pr_err("wait_for_completion_interruptible\n");
			break;
		}
		pr_info("start to connect...\n");

		if (establish_connect_stop) {
			pr_err("establish_connect_stop_1\n");
			break;
		}

		spin_lock(&hbipc_eth_ctx.ready_lock);
		ready_local = hbipc_eth_ctx.ready;
		spin_unlock(&hbipc_eth_ctx.ready_lock);
		if (ready_local) {
			pr_info("already connect\n");
			continue;
		}

		// release invalid client_sock
		mutex_lock(&hbipc_eth_ctx.release_client_lock);
		if (hbipc_eth_ctx.client_sock) {
			sock_release(hbipc_eth_ctx.client_sock);
			hbipc_eth_ctx.client_sock = NULL;
			pr_info("release 1\n");
		}
		mutex_unlock(&hbipc_eth_ctx.release_client_lock);
#ifndef CONFIG_HOBOT_BIF_AP
		// CP as TCP server, accept
		ret = kernel_accept(hbipc_eth_ctx.server_sock, &hbipc_eth_ctx.client_sock, 0);
		if (ret < 0) {
			pr_err("kernel_accept error[%d]\n", ret);
			continue;
		}
		pr_info("CP connect success\n");
		spin_lock(&hbipc_eth_ctx.ready_lock);
		hbipc_eth_ctx.ready = 1;
		spin_unlock(&hbipc_eth_ctx.ready_lock);
#else
		// AP as TCP client, connect
		ret = sock_create(AF_INET, SOCK_STREAM, 0, &hbipc_eth_ctx.client_sock);
		if (ret < 0) {
			pr_err("create client sock error[%d]\n", ret);
			continue;
		}

		hbipc_eth_ctx.server_addr.sin_family = AF_INET;
		hbipc_eth_ctx.server_addr.sin_port = htons(portnum);
		hbipc_eth_ctx.server_addr.sin_addr.s_addr = in_aton(hbipc_eth_ctx.ip_addr);

		while (1) {
		reconnect:
			ret = kernel_connect(hbipc_eth_ctx.client_sock,
				(struct sockaddr *)&hbipc_eth_ctx.server_addr, sizeof(struct sockaddr_in), O_NONBLOCK);
			if (ret < 0) {
				msleep(CONNECT_RETRY_INTERVAL);
				if (establish_connect_stop) {
					pr_err("establish_connect_stop_2\n");
					break;
				}
				goto reconnect;
			}
			break;
		}
		if (establish_connect_stop) {
			pr_err("establish_connect_stop_3\n");
			break;
		}
		pr_info("AP connect success\n");
		spin_lock(&hbipc_eth_ctx.ready_lock);
		hbipc_eth_ctx.ready = 1;
		spin_unlock(&hbipc_eth_ctx.ready_lock);
#endif
	}
	// release invalid client_sock
	mutex_lock(&hbipc_eth_ctx.release_client_lock);
	if (hbipc_eth_ctx.client_sock) {
		sock_release(hbipc_eth_ctx.client_sock);
		hbipc_eth_ctx.client_sock = NULL;
		pr_info("release 2\n");
	}
	mutex_unlock(&hbipc_eth_ctx.release_client_lock);
}

static void start_reconnect(void)
{
	spin_lock(&hbipc_eth_ctx.ready_lock);
	hbipc_eth_ctx.ready = 0;
	spin_unlock(&hbipc_eth_ctx.ready_lock);
	complete(&hbipc_eth_ctx.start_to_connect);
}

static int hbipc_eth_ctx_init(struct hbipc_eth_ctx_t *hbipc_eth_ctx)
{
#ifndef CONFIG_HOBOT_BIF_AP
	int ret = 0;
#endif

	hbipc_eth_ctx->work_queue = create_singlethread_workqueue("hbipc_eth_workqueue");
	if (!hbipc_eth_ctx->work_queue) {
		pr_err("create_singlethread_workqueue error\n");
		goto create_wrokqueue_error;
	}

	spin_lock_init(&hbipc_eth_ctx->ready_lock);
	init_completion(&hbipc_eth_ctx->start_to_connect);
	INIT_WORK(&hbipc_eth_ctx->work, establish_connect);

	if (queue_work(hbipc_eth_ctx->work_queue, &hbipc_eth_ctx->work) == false) {
		pr_err("queue work error\n");
		goto queue_work_error;
	}
#ifndef CONFIG_HOBOT_BIF_AP
	ret = sock_create(AF_INET, SOCK_STREAM, 0, &hbipc_eth_ctx->server_sock);
	if (ret) {
		pr_err("create server_sock error\n");
		goto create_server_sock_error;
	}
#endif
	mutex_init(&hbipc_eth_ctx->release_client_lock);
	return 0;
#ifndef CONFIG_HOBOT_BIF_AP
create_server_sock_error:
	establish_connect_stop = 1;
	complete(&hbipc_eth_ctx->start_to_connect);
#endif
queue_work_error:
	destroy_workqueue(hbipc_eth_ctx->work_queue);
create_wrokqueue_error:
	return -1;
}

static void hbipc_eth_ctx_deinit(struct hbipc_eth_ctx_t *hbipc_eth_ctx)
{
#ifndef CONFIG_HOBOT_BIF_AP
	sock_release(hbipc_eth_ctx->server_sock);
#endif
	establish_connect_stop = 1;
	complete(&hbipc_eth_ctx->start_to_connect);
	destroy_workqueue(hbipc_eth_ctx->work_queue);
	mutex_lock(&hbipc_eth_ctx->release_client_lock);
	if (hbipc_eth_ctx->client_sock) {
		sock_release(hbipc_eth_ctx->client_sock);
		hbipc_eth_ctx->client_sock = NULL;
		pr_info("release 3\n");
	}
	mutex_unlock(&hbipc_eth_ctx->release_client_lock);
	mutex_destroy(&hbipc_eth_ctx->release_client_lock);
}

#ifndef CONFIG_NO_DTS_AP
static int hbipc_eth_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct property *prop = NULL;
	int prop_len = 0;

	pr_info("%s\n", __func__);

#ifndef CONFIG_HOBOT_BIF_AP
	prop = of_find_property(pdev->dev.of_node, "if_name", &prop_len);
	if (!prop) {
		pr_err("get if_name error\n");
		goto error;
	}
	strncpy(hbipc_eth_ctx.if_name, prop->value, prop_len);
	pr_info("if_name = %s[%d]\n", hbipc_eth_ctx.if_name, prop_len);
#else
	prop = of_find_property(pdev->dev.of_node, "ip_addr", &prop_len);
	if (!prop) {
		pr_err("get ip_addr error\n");
		goto error;
	}
	strncpy(hbipc_eth_ctx.ip_addr, prop->value, prop_len);
	pr_info("ip_addr = %s[%d]\n", hbipc_eth_ctx.ip_addr, prop_len);
#endif

	ret = hbipc_eth_ctx_init(&hbipc_eth_ctx);
	if (ret < 0) {
		pr_err("hbipc_eth_ctx_init error\n");
		goto error;
	}

#ifdef CONFIG_HOBOT_BIF_AP
	//complete(&hbipc_eth_ctx.start_to_connect);
#endif
	return 0;
error:
	return -1;
}
#endif

#ifdef CONFIG_NO_DTS_AP
static int hbipc_eth_probe_param(void)
{
	int ret = 0;

	pr_info("%s\n", __func__);

#ifndef CONFIG_HOBOT_BIF_AP
	if (!if_name) {
		pr_err("get if_name error\n");
		goto error;
	}
	strncpy(hbipc_eth_ctx.if_name, if_name, strlen(if_name) + 1);
	pr_info("if_name = %s[%d]\n", hbipc_eth_ctx.if_name, strlen(if_name) + 1);
#else
	if (!ip_addr) {
		pr_err("get ip_addr error\n");
		goto error;
	}
	strncpy(hbipc_eth_ctx.ip_addr, ip_addr, strlen(ip_addr) + 1);
	pr_info("ip_addr = %s[%d]\n", hbipc_eth_ctx.ip_addr, strlen(ip_addr) + 1);
#endif

	ret = hbipc_eth_ctx_init(&hbipc_eth_ctx);
	if (ret < 0) {
		pr_err("hbipc_eth_ctx_init error\n");
		goto error;
	}

#ifdef CONFIG_HOBOT_BIF_AP
		//complete(&hbipc_eth_ctx.start_to_connect);
#endif
	return 0;
error:
	return -1;
}
#endif

#ifndef CONFIG_NO_DTS_AP
static int hbipc_eth_remove(struct platform_device *dev)
{
	hbipc_eth_ctx_deinit(&hbipc_eth_ctx);

	return 0;
}
#endif

#ifdef CONFIG_NO_DTS_AP
static int hbipc_eth_remove_param(void)
{
	hbipc_eth_ctx_deinit(&hbipc_eth_ctx);

	return 0;
}
#endif

#ifndef CONFIG_NO_DTS_AP
static const struct of_device_id hbipc_eth_of_match[] = {
	{.compatible = "hobot,hbipc_eth"},
	{},
};

static struct platform_driver hbipc_eth_driver = {
	.driver = {
		.name = "hbipc_eth",
		.of_match_table = hbipc_eth_of_match,
	},
	.probe = hbipc_eth_probe,
	.remove = hbipc_eth_remove,
};
#endif

static int __init hbipc_eth_init(void)
{
	int ret = 0;

#ifdef CONFIG_NO_DTS_AP
	ret = hbipc_eth_probe_param();
	if (ret)
		pr_err("hbipc_eth_probe_param error\n");
#else
	ret = platform_driver_register(&hbipc_eth_driver);
	if (ret)
		pr_err("register hbipc_eth_driver error\n");
#endif

	return ret;
}

static void __exit hbipc_eth_exit(void)
{
#ifdef CONFIG_NO_DTS_AP
	hbipc_eth_remove_param();
#else
	platform_driver_unregister(&hbipc_eth_driver);
#endif
}

late_initcall(hbipc_eth_init);
module_exit(hbipc_eth_exit);

/* export function */
int hbeth_bind_sock(void)
{
	int ret = 0;

	if (hbipc_eth_ctx.bind_flag) {
		// already bind
		pr_info("already bind\n");
		return 0;
	}

	// get IP addr from interface name
	strcpy(hbipc_eth_ctx.ifr.ifr_name, hbipc_eth_ctx.if_name);
	pr_info("ifr.ifr_name = %s\n", hbipc_eth_ctx.ifr.ifr_name);
	ret = kernel_sock_ioctl(hbipc_eth_ctx.server_sock, SIOCGIFADDR,
		(unsigned long)&hbipc_eth_ctx.ifr);
	if (ret < 0) {
		pr_err("kernel_sock_ioctl get IP addr error[%d]\n", ret);
		goto error;
	}
	hbipc_eth_ctx.server_addr.sin_family = AF_INET;
	hbipc_eth_ctx.server_addr.sin_port = htons(portnum);
	hbipc_eth_ctx.server_addr.sin_addr = ((struct sockaddr_in *)&(hbipc_eth_ctx.ifr.ifr_addr))->sin_addr;
	pr_info("ip_addr = %d\n", hbipc_eth_ctx.server_addr.sin_addr.s_addr);

	ret = kernel_bind(hbipc_eth_ctx.server_sock, (struct sockaddr *)&hbipc_eth_ctx.server_addr,
		sizeof(struct sockaddr_in));
	if (ret < 0) {
		pr_err("kernel_bind error[%d]\n", ret);
		goto error;
	}

	ret = kernel_listen(hbipc_eth_ctx.server_sock, 10);
	if (ret < 0) {
		pr_err("kernel_listen error[%d]\n", ret);
		goto error;
	}

	// start CP establish connect
	complete(&hbipc_eth_ctx.start_to_connect);

	hbipc_eth_ctx.bind_flag = 1;

	// wait for connect
	msleep(5 * CONNECT_RETRY_INTERVAL);

	return 0;
error:
	return -1;
}
EXPORT_SYMBOL(hbeth_bind_sock);

int hbeth_set_sendtimeout(int timeout_ms)
{
	struct timeval tv;
	int ret = 0;

	memset(&tv, 0, sizeof(tv));
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	ret = kernel_setsockopt(hbipc_eth_ctx.client_sock, SOL_SOCKET,
		SO_SNDTIMEO, (char *)&tv, sizeof(tv));
	if (ret < 0) {
		pr_err("set send timeout error[%d]\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(hbeth_set_sendtimeout);

int hbeth_sendframe(void *buf, int len)
{
	struct kvec vec;
	struct msghdr msg;
	int ret = 0;

	memset(&msg, 0, sizeof(msg));
	vec.iov_base = buf;
	vec.iov_len = len;

	ret = kernel_sendmsg(hbipc_eth_ctx.client_sock, &msg, &vec, 1, len);
	if (ret < 0) {
		pr_err("kernel_sendmsg error[%d]\n", ret);
		start_reconnect();
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(hbeth_sendframe);

int hbeth_recvframe(void *buf, int len)
{
	struct kvec vec;
	struct msghdr msg;
	void *ptr = NULL;
	int count = 0;
	int nread = 0;

	memset(&msg, 0, sizeof(msg));
	ptr = buf;
	count = len;
	while (count > 0) {
		vec.iov_base = ptr;
		vec.iov_len = count;
		nread = kernel_recvmsg(hbipc_eth_ctx.client_sock, &msg, &vec, 1, count, 0);
		if (nread < 0) {
			pr_err("kernel_recvmsg error[%d]\n", nread);
			start_reconnect();
			return -1;
		} else if (nread == 0) {
			// read EOF
			pr_err("opposite end closed\n");
			start_reconnect();
			break;
		} else {
			count -= nread;
			ptr += nread;
		}
	}

	// return >= 0
	return len - count;
}
EXPORT_SYMBOL(hbeth_recvframe);

int hbeth_check_ready(void)
{
	int ready_local = 0;

	spin_lock(&hbipc_eth_ctx.ready_lock);
	ready_local = hbipc_eth_ctx.ready;
	spin_unlock(&hbipc_eth_ctx.ready_lock);

	return ready_local;
}
EXPORT_SYMBOL(hbeth_check_ready);

void hbeth_ap_start_connect(void)
{
	// start AP establish connect
	complete(&hbipc_eth_ctx.start_to_connect);
}
EXPORT_SYMBOL(hbeth_ap_start_connect);

MODULE_LICENSE("GPL");
