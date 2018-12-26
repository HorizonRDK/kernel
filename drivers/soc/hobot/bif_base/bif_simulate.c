/************************************************************
 ****			 COPYRIGHT NOTICE			 ****
 ****		 Copyright	2018 Horizon Robotics, Inc.		 ****
 ****			 All rights reserved.			 ****
 ************************************************************/
/**
 * BIF simulate driver for base by netlink
 * @author		haibo.guo(haibo.guo@horizon.ai)
 * @date		2018/12/26
 */
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include <linux/timer.h>
#include <linux/jiffies.h>

#include <net/netlink.h>
#include <net/sock.h>

#include <linux/completion.h>
#include <linux/list.h>
#include <linux/spinlock.h>

#include "bif_simulate.h"

void pr_buff(unsigned char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (i == 0)
			pr_info("0x%08x:", i);
		else
			if (i % 16 == 0)
				pr_info("\n0x%08x:", i);
		pr_info("%02X ", buf[i]);
	}
}

void pr_buff_16(unsigned char *buff, int len_16)
{
	int i;

	for (i = 0; i < len_16; i++) {
		pr_info("%02x%02x%02x%02x%02x%02x%02x%02x ",
		     buff[i * 16 + 0], buff[i * 16 + 1], buff[i * 16 + 2],
		     buff[i * 16 + 3], buff[i * 16 + 4], buff[i * 16 + 5],
		     buff[i * 16 + 6], buff[i * 16 + 7]);
		pr_info
		    ("%02x%02x%02x%02x%02x%02x%02x%02x\n",
			buff[i * 16 + 8], buff[i * 16 + 9], buff[i * 16 + 10],
			buff[i * 16 + 11], buff[i * 16 + 12], buff[i * 16 + 13],
			buff[i * 16 + 14], buff[i * 16 + 15]);
	}
}

#define BIF_NETLINK			(25)
#define BIF_NETLINK_PORTID		(2222)
#define BIF_ETHER_MAXSIZE		(512*3)	//(512*3)
#define BIF_NETLINK_MAXSIZE		(BIF_ETHER_MAXSIZE+16)	//1552
#define MAX_PAYLOAD			(BIF_NETLINK_MAXSIZE+16)
#define MAX(a, b)				((a) > (b) ? (a) : (b))
#define MIN(a, b)				((a) < (b) ? (a) : (b))
/*
 *| op | len | data ...|
 *| 1B | 2B  |
 */
#define OP_REQ_ADDR				(0x01)
#define OP_ACK_ADDR				(0x02)
#define OP_SEND_CP_IRQ			(0x03)
#define OP_ACK_CP_IRQ			(0x04)
#define OP_SEND_AP_IRQ			(0x05)
#define OP_ACK_AP_IRQ			(0x06)
#define OP_REQ_WRITE_DATA		(0x07)
#define OP_ACK_WRITE_DATA		(0x08)
#define OP_REQ_READ_DATA		(0x09)
#define OP_ACK_READ_DATA		(0x0A)

#define DATA_HEADER_SIZE		(3)
#define QUEUE_MAX				(20)
struct queue_data {
	unsigned char tail;
	unsigned char head;
	spinlock_t lock_tail, lock_head;
	unsigned char data[QUEUE_MAX][BIF_NETLINK_MAXSIZE];
};

struct extern_local {
	struct sock *nl_sk;
	struct task_struct *tx_task;	/* netlink tx */
	struct task_struct *rx_task;	/* netlink rx */
	wait_queue_head_t tx_wq;	/* tx wait queue */
	wait_queue_head_t rx_wq;	/* rx wait queue */
	unsigned int pid;
	spinlock_t lock_pid;
	spinlock_t lock_list;

	struct bif_base_info bif_base;
	struct queue_data rxop, txop;

	int pidport;
	int buffer_id;
	int netlink_start;
} local;

/* list */
struct todo_struct {
	unsigned int pid;
	unsigned int phy;
	unsigned char *ptr;

	struct completion cp;
	struct list_head list;
};
struct list_head todo_list;
unsigned char *gptr;

#ifndef CONFIG_HOBOT_BIF_AP
extern void *bif_vir_addr;
#endif

static void queue_init(qdata_t *queue)
{
	int i = 0;
	qdata_t *q = queue;

	q->tail = 0;
	q->head = 0;
	spin_lock_init(&q->lock_head);
	spin_lock_init(&q->lock_tail);
	for (i = 0; i < QUEUE_MAX; i++)
		memset(q->data[i], 0, BIF_NETLINK_MAXSIZE);
}

static int queue_push(qdata_t *queue, unsigned char *data, unsigned int len)
{
	qdata_t *q = queue;
	unsigned char *pbuf = NULL;
	unsigned int packetlen = len;
	unsigned long flags = 0;

	spin_lock_irqsave(&q->lock_tail, flags);

	pbuf = q->data[(q->tail) % QUEUE_MAX];

	if ((q->tail + 1) % QUEUE_MAX == q->head) {
		pr_warn("push:queue full! q->tail=%d,q->head=%d,drop packetlen=%d\n",
		     q->tail, q->head, packetlen);
		spin_unlock_irqrestore(&q->lock_tail, flags);
		return 1;
	}

	packetlen = MIN(packetlen, BIF_NETLINK_MAXSIZE);
	memset(pbuf, 0, BIF_NETLINK_MAXSIZE);
	memcpy(pbuf, data, packetlen);
	q->tail = (q->tail + 1) % QUEUE_MAX;
	spin_unlock_irqrestore(&q->lock_tail, flags);

	return 0;
}

static void *queue_pop(qdata_t *queue)
{
	qdata_t *q = queue;
	void *data = NULL;
	unsigned long flags = 0;

	spin_lock_irqsave(&q->lock_head, flags);

	if (q->head == q->tail) {
		spin_unlock_irqrestore(&q->lock_head, flags);
		return NULL;
	}

	data = (void *)q->data[(q->head) % QUEUE_MAX];
	q->head = (q->head + 1) % QUEUE_MAX;
	spin_unlock_irqrestore(&q->lock_head, flags);

	return data;
}

static void packet_analyse(unsigned char *pbuf)
{
	unsigned char buffer_id = 0;
	unsigned long phy_addr = 0;
	unsigned short packetlen = 0;
	unsigned short datalen = 0;
	unsigned short len = 0;
	unsigned char cmd = 0;
	unsigned int cur_pid = 0;
	void *vir_addr = NULL;
	unsigned char *rbuf = pbuf;
	unsigned char *sbuf = gptr;
	//unsigned char sbuf[MAX_PAYLOAD] = {0};
	unsigned long flags = 0;

	struct todo_struct *todo_node = NULL;
	struct list_head *list_node = NULL;
	struct list_head *n = NULL;

	if ((rbuf == NULL) || (sbuf == NULL)) {
		pr_warn("<<<POP: Warn rbuf or sbuf is NULL\n");
		return;
	}
	cmd = rbuf[0];
	datalen = rbuf[1] << 8 | rbuf[2];
	cur_pid = rbuf[3] << 24 | rbuf[4] << 16 | rbuf[5] << 8 | rbuf[6] << 0;
	datalen = MIN(datalen, (BIF_NETLINK_MAXSIZE - DATA_HEADER_SIZE));
	packetlen = datalen + DATA_HEADER_SIZE;
	switch (cmd) {
	case OP_REQ_ADDR:	/* CP process... */
		pr_info("<(%08x) OP_REQ_ADDR\n", cur_pid);

		buffer_id = rbuf[7] % BUFF_MAX;
		phy_addr = (unsigned long)t_bif_query_address(buffer_id);

		sbuf[0] = OP_ACK_ADDR;
		sbuf[1] = 0;
		sbuf[2] = 9;
		sbuf[3] = cur_pid >> 24 & 0xff;
		sbuf[4] = cur_pid >> 16 & 0xff;
		sbuf[5] = cur_pid >> 8 & 0xff;
		sbuf[6] = cur_pid >> 0 & 0xff;
		sbuf[7] = buffer_id;
		sbuf[8] = phy_addr >> 24 & 0xff;
		sbuf[9] = phy_addr >> 16 & 0xff;
		sbuf[10] = phy_addr >> 8 & 0xff;
		sbuf[11] = phy_addr >> 0 & 0xff;
		packetlen = 12;

		queue_push(&local.txop, sbuf, packetlen);
		wake_up_interruptible(&local.tx_wq);
		break;
	case OP_ACK_ADDR:	/* AP process... */
		pr_info("<(%08x) OP_ACK_ADDR\n", cur_pid);

		buffer_id = rbuf[7] % BUFF_MAX;
		phy_addr =
		    rbuf[8] << 24 | rbuf[9] << 16 | rbuf[10] << 8 | rbuf[11] <<
		    0;
		local.bif_base.address_list[buffer_id] = (void *)phy_addr;

		pr_info("buffer_id=%d, phy_addr=0x%lx\n", buffer_id, phy_addr);

		spin_lock_irqsave(&local.lock_list, flags);
		list_for_each_safe(list_node, n, &todo_list) {
			todo_node =
			    list_entry(list_node, struct todo_struct, list);
			if (todo_node->pid == cur_pid) {
				todo_node->ptr = rbuf;
				complete(&todo_node->cp);
				break;
			}
		}
		spin_unlock_irqrestore(&local.lock_list, flags);
		break;

	case OP_REQ_WRITE_DATA:	/* CP process... */
		pr_info("<(%08x) OP_REQ_WRITE_DATA\n", cur_pid);

		if (datalen > 7)
			len = datalen - 8;
		else
			len = 0;
		phy_addr =
		    rbuf[7] << 24 | rbuf[8] << 16 | rbuf[9] << 8 | rbuf[10] <<
		    0;
#ifndef CONFIG_HOBOT_BIF_AP
		if (phy_addr == BIF_BASE_ADDR + 512)
			vir_addr = bif_vir_addr + 512;
		else
#endif
			vir_addr = phys_to_virt(phy_addr);
		if (vir_addr != NULL) {
			len = MIN(len, BIF_ETHER_MAXSIZE);
			memcpy(vir_addr, rbuf + 11, len);
		}

		//pr_info("REQ write phy=%lx,vir=%p datalen=%04x,len=%d\n",
		//	phy_addr,vir_addr, datalen, len);


		sbuf[0] = OP_ACK_WRITE_DATA;
		sbuf[1] = 0;
		sbuf[2] = 8;
		memcpy(sbuf + 3, rbuf + 3, 4);	/* cur_pid */
		memcpy(sbuf + 7, rbuf + 7, 4);	/*phy_addr */
		packetlen = 11;

		queue_push(&local.txop, sbuf, packetlen);
		wake_up_interruptible(&local.tx_wq);
		break;
	case OP_ACK_WRITE_DATA:	/* AP process... */
		pr_info("<(%08x) OP_ACK_WRITE_DATA\n", cur_pid);

		spin_lock_irqsave(&local.lock_list, flags);
		list_for_each_safe(list_node, n, &todo_list) {
			todo_node =
			    list_entry(list_node, struct todo_struct, list);
			if (todo_node->pid == cur_pid) {
				todo_node->ptr = rbuf;
				complete(&todo_node->cp);
				break;
			}
		}
		spin_unlock_irqrestore(&local.lock_list, flags);
		break;

	case OP_REQ_READ_DATA:	/* CP process... */
		pr_info("<(%08x) OP_REQ_READ_DATA\n", cur_pid);

		phy_addr =
		    rbuf[7] << 24 | rbuf[8] << 16 | rbuf[9] << 8 | rbuf[10] <<
		    0;
		len = rbuf[11] << 8 | rbuf[12];

#ifndef CONFIG_HOBOT_BIF_AP
		if (phy_addr == BIF_BASE_ADDR)
			vir_addr = bif_vir_addr;
		else
#endif
			vir_addr = phys_to_virt(phy_addr);
		if (vir_addr != NULL) {
			len = MIN(len, BIF_ETHER_MAXSIZE);
			memcpy(sbuf + 11, vir_addr, len);
		} else
			len = 0;
		datalen = len + 8;
		sbuf[0] = OP_ACK_READ_DATA;
		sbuf[1] = datalen >> 8 & 0xff;
		sbuf[2] = datalen >> 0 & 0xff;
		memcpy(sbuf + 3, rbuf + 3, 4);	/* cur_pid */
		memcpy(sbuf + 7, rbuf + 7, 4);	/*phy_addr */
		packetlen = datalen + 3;

		queue_push(&local.txop, sbuf, packetlen);
		wake_up_interruptible(&local.tx_wq);
		break;
	case OP_ACK_READ_DATA:	/* AP process... */
		pr_info("<(%08x) OP_ACK_READ_DATA\n", cur_pid);

		if (datalen > 7)
			len = datalen - 8;
		else
			len = 0;
		phy_addr =
		    rbuf[7] << 24 | rbuf[8] << 16 | rbuf[9] << 8 | rbuf[10] <<
		    0;

		spin_lock_irqsave(&local.lock_list, flags);
		list_for_each_safe(list_node, n, &todo_list) {
			todo_node =
			    list_entry(list_node, struct todo_struct, list);
			if (todo_node->pid == cur_pid) {
				todo_node->ptr = rbuf;
				complete(&todo_node->cp);
				break;
			}
		}
		if (todo_node == NULL)
			pr_err("err todo_node NULL\n");
		spin_unlock_irqrestore(&local.lock_list, flags);
		break;

	case OP_SEND_AP_IRQ:	/* CP process... */
		pr_info("<(%08x) OP_SEND_AP_IRQ\n", cur_pid);

		local.bif_base.irq_func[local.buffer_id %
					BUFF_MAX] (local.buffer_id % BUFF_MAX,
						   NULL);

		sbuf[0] = OP_ACK_AP_IRQ;
		sbuf[1] = 0;
		sbuf[2] = 5;
		memcpy(sbuf + 3, rbuf + 3, 4);	//cur_pid
		sbuf[7] = rbuf[7];	//irq
		packetlen = 8;

		queue_push(&local.txop, sbuf, packetlen);	//push
		wake_up_interruptible(&local.tx_wq);	//wakeup
		break;
	case OP_ACK_AP_IRQ:	//AP process...
		pr_info("<(%08x) OP_ACK_AP_IRQ\n", cur_pid);

		spin_lock_irqsave(&local.lock_list, flags);
		list_for_each_safe(list_node, n, &todo_list) {
			todo_node =
			    list_entry(list_node, struct todo_struct, list);
			if (todo_node->pid == cur_pid) {
				todo_node->ptr = rbuf;
				complete(&todo_node->cp);
				break;
			}
		}
		spin_unlock_irqrestore(&local.lock_list, flags);
		break;

	case OP_SEND_CP_IRQ:	//AP process...
		pr_info("<[%08x] OP_SEND_CP_IRQ\n", cur_pid);

		local.bif_base.irq_func[local.buffer_id %
					BUFF_MAX] (local.buffer_id % BUFF_MAX,
						   NULL);

		sbuf[0] = OP_ACK_CP_IRQ;
		sbuf[1] = 0;
		sbuf[2] = 5;
		memcpy(sbuf + 3, rbuf + 3, 4);	//cur_pid
		sbuf[7] = rbuf[7];	//irq
		packetlen = 8;

		queue_push(&local.txop, sbuf, packetlen);	//push
		wake_up_interruptible(&local.tx_wq);	//wakeup
		break;
	case OP_ACK_CP_IRQ:	//CP process..
		pr_info("<[%08x] OP_ACK_CP_IRQ\n", cur_pid);

		spin_lock_irqsave(&local.lock_list, flags);
		list_for_each_safe(list_node, n, &todo_list) {
			todo_node =
			    list_entry(list_node, struct todo_struct, list);
			if (todo_node->pid == cur_pid) {
				todo_node->ptr = rbuf;
				complete(&todo_node->cp);
				break;
			}
		}
		spin_unlock_irqrestore(&local.lock_list, flags);
		break;

	default:
		pr_warn("default: Err packet\n");
		break;
	}
}

int bif_netlink_send(unsigned int portid, unsigned char *message, int len)
{
	struct sk_buff *skb_1 = NULL;
	struct nlmsghdr *nlh = NULL;
	unsigned int cur_pid = 0;
	int ret = 0;

	if (!local.netlink_start)
		return -1;

	if (!message || !local.nl_sk || len < 1) {
		pr_err("%s() message or nl_sk NULL. or len < 1!\n", __func__);
		return -1;
	}

	skb_1 = nlmsg_new(NLMSG_SPACE(len), GFP_ATOMIC);
	if (!skb_1) {
		pr_err("%s() alloc_skb error!\n", __func__);
		return -2;
	}

	cur_pid =
	    message[3] << 24 | message[4] << 16 | message[5] << 8 | message[6]
	    << 0;

	nlh = nlmsg_put(skb_1, 0, 0, 0, len, 0);
	if (nlh == NULL) {
		pr_err("%s() nlmsg_put failaure\n", __func__);
		if (skb_1)
			nlmsg_free(skb_1);
		return -3;
	}

	NETLINK_CB(skb_1).portid = 0;
	NETLINK_CB(skb_1).dst_group = 0;
	memcpy(NLMSG_DATA(nlh), message, len);

	if (local.nl_sk) {
		ret = netlink_unicast(local.nl_sk, skb_1, portid, MSG_DONTWAIT);
		if (ret < 0)
			pr_err(">%08x NetSend: Err ret=%d\n", cur_pid, ret);
	} else
		pr_err(">%08x NetSend: err ...\n", cur_pid);

	return ret;
}

void bif_netlink_input(struct sk_buff *__skb)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	unsigned int rlen = 0;
	unsigned char *pbuf = NULL;

	if (!__skb)
		return;

	skb = skb_get(__skb);
	if (skb->len < NLMSG_SPACE(0)) {
		if (skb)
			nlmsg_free(skb);
		return;
	}

	if (!local.netlink_start) {
		if (skb)
			nlmsg_free(skb);
		return;
	}

	nlh = nlmsg_hdr(skb);
	rlen = nlh->nlmsg_len - NLMSG_SPACE(0);

	//pr_info("%s() pid:%d, space:%d, size:%d, rlen:%d\n", __func__,
	//	nlh->nlmsg_pid, NLMSG_SPACE(0), nlh->nlmsg_len, rlen);

	pbuf = NLMSG_DATA(nlh);
	rlen = MIN(rlen, BIF_NETLINK_MAXSIZE);
	if (rlen > 0) {
		queue_push(&local.rxop, pbuf, rlen);
		wake_up_interruptible(&local.rx_wq);
	}

	if (skb)
		nlmsg_free(skb);
}

int bif_tx_thread(void *data)
{
	unsigned short packetlen = 0;
	unsigned char *tbuf = NULL;

	while (!kthread_should_stop()) {
		if (local.netlink_start)
			wait_event_interruptible_timeout(local.tx_wq,
							 local.txop.tail !=
							 local.txop.head,
							 HZ * 60);

		while (local.txop.tail != local.txop.head) {
			if (!local.netlink_start)
				break;

			tbuf = (unsigned char *)queue_pop(&local.txop);
			if (tbuf) {
				packetlen = tbuf[1] << 8 | tbuf[2];
				packetlen += 3;
				packetlen = MIN(packetlen, BIF_NETLINK_MAXSIZE);
				bif_netlink_send(BIF_NETLINK_PORTID, tbuf,
						 packetlen);
			}
		}
	}

	return 0;
}

int bif_tx_thread_init(void)
{
	int err;

	local.tx_task = kthread_create(bif_tx_thread, NULL, "bif_nltx");

	if (IS_ERR(local.tx_task)) {
		pr_err("%s() Unable to start kernel thread.", __func__);
		err = PTR_ERR(local.tx_task);
		local.tx_task = NULL;
		return err;
	}

	wake_up_process(local.tx_task);
	return 0;
}

int bif_rx_thread(void *data)
{
	unsigned char *rbuf = NULL;

	while (!kthread_should_stop()) {
		if (local.netlink_start)
			wait_event_interruptible_timeout(local.rx_wq,
							 local.rxop.tail !=
							 local.rxop.head,
							 HZ * 60);

		while (local.rxop.tail != local.rxop.head) {
			if (!local.netlink_start)
				break;

			rbuf = (unsigned char *)queue_pop(&local.rxop);
			if (rbuf)
				packet_analyse(rbuf);
		}
	}
	return 0;
}

int bif_rx_thread_init(void)
{
	int err;

	local.rx_task = kthread_create(bif_rx_thread, NULL, "bif_nlrx");

	if (IS_ERR(local.rx_task)) {
		pr_err("%s() Unable to start kernel thread.", __func__);
		err = PTR_ERR(local.rx_task);
		local.rx_task = NULL;
		return err;
	}

	wake_up_process(local.rx_task);
	return 0;
}

int bif_netlink_init(void)
{
	struct netlink_kernel_cfg nkc;

	pr_info("%s() init, begin...\n", __func__);
	local.netlink_start = 0;

#if 0
	local.nl_wq = create_workqueue("nl_wq");
	if (!local.nl_wq) {
		pr_err("%s: [netlink] create_workqueue error!\n", __func__);
		return -1;
	}
#endif
	gptr = kmalloc(MAX_PAYLOAD, GFP_ATOMIC | __GFP_ZERO);
	if (gptr == NULL)
		return -ENOMEM;

	queue_init(&local.txop);
	queue_init(&local.rxop);
	spin_lock_init(&local.lock_pid);
	spin_lock_init(&local.lock_list);

	INIT_LIST_HEAD(&todo_list);	//list

	nkc.groups = 0;
	nkc.flags = 0;
	//nkc.flags = NL_CFG_F_NONROOT_RECV | NL_CFG_F_NONROOT_SEND;
	nkc.input = bif_netlink_input;
	nkc.cb_mutex = NULL;
	nkc.bind = NULL;
	nkc.unbind = NULL;
	nkc.compare = NULL;
	local.nl_sk = netlink_kernel_create(&init_net, BIF_NETLINK, &nkc);
	if (!local.nl_sk) {
		pr_err("%s: create netlink socket error!\n", __func__);
		return -1;
	}
	pr_info("%s: create netlink socket success!\n", __func__);

	local.pidport = BIF_NETLINK_PORTID;

	init_waitqueue_head(&local.tx_wq);
	init_waitqueue_head(&local.rx_wq);
	bif_tx_thread_init();
	bif_rx_thread_init();

	local.netlink_start = 1;
	pr_info("%s() init, end\n", __func__);
	return 0;
}

void bif_netlink_exit(void)
{
	unsigned char ch = 0;

	pr_info("%s() exit, begin...\n", __func__);
	local.netlink_start = 0;

	pr_info("%s() exit, local.tx_task\n", __func__);
	if (local.tx_task) {
		ch = local.txop.tail;
		local.txop.tail = (local.txop.head + 1) % QUEUE_MAX;
		wake_up_all(&local.tx_wq);
		if (local.tx_task) {
			kthread_stop(local.tx_task);
			local.tx_task = NULL;
		}
		local.txop.tail = ch;
	}

	pr_info("%s() exit, local.rx_task\n", __func__);
	if (local.rx_task) {
		ch = local.rxop.tail;
		local.rxop.tail = (local.rxop.head + 1) % QUEUE_MAX;
		wake_up_all(&local.rx_wq);
		if (local.rx_task) {
			kthread_stop(local.rx_task);
			local.rx_task = NULL;
		}
		local.rxop.tail = ch;
	}

	pr_info("%s() exit, local.nl_sk\n", __func__);
	if (local.nl_sk) {
		netlink_kernel_release(local.nl_sk);
		local.nl_sk = NULL;
	}

	pr_info("%s() exit, gptr\n", __func__);

	kfree(gptr);

	pr_info("%s() exit, end\n", __func__);
}

int t_bif_register_address(enum BUFF_ID buffer_id, void *address)
{
	if (buffer_id >= BUFF_MAX)
		return -1;

	local.bif_base.address_list[buffer_id] = address;
	local.bif_base.buffer_count++;

	return 0;
};

int t_bif_register_irq(enum BUFF_ID buffer_id, irq_handler_t irq_handler)
{
	if (buffer_id >= BUFF_MAX)
		return -1;

	local.bif_base.irq_func[buffer_id] = irq_handler;
	local.buffer_id = buffer_id;

	return buffer_id;
}

int t_bif_send_irq(int irq)
{
	unsigned char sbuf[8] = { 0 };
	unsigned int packetlen = 0;
	unsigned int cur_pid = 0;
	unsigned long flags = 0;
	struct todo_struct *todo_node = NULL;
	void *ptr = NULL;

	if (!local.netlink_start)
		return -1;
	ptr = kmalloc(sizeof(struct todo_struct), GFP_ATOMIC | __GFP_ZERO);
	if (ptr == NULL)
		return 0;
	todo_node = (struct todo_struct *)ptr;

	spin_lock_irqsave(&local.lock_pid, flags);
	cur_pid = local.pid++;
	spin_unlock_irqrestore(&local.lock_pid, flags);

#ifdef CONFIG_HOBOT_BIF_AP
	sbuf[0] = OP_SEND_AP_IRQ;
	pr_info(">(%08x) OP_SEND_AP_IRQ\n", cur_pid);
#else
	sbuf[0] = OP_SEND_CP_IRQ;
	pr_info(">[%08x] OP_SEND_CP_IRQ\n", cur_pid);
#endif

	sbuf[1] = 0;
	sbuf[2] = 5;
	sbuf[3] = cur_pid >> 24 & 0xff;
	sbuf[4] = cur_pid >> 16 & 0xff;
	sbuf[5] = cur_pid >> 8 & 0xff;
	sbuf[6] = cur_pid >> 0 & 0xff;
	sbuf[7] = irq & 0xff;
	packetlen = 7;

	todo_node->pid = cur_pid;
	init_completion(&todo_node->cp);

	spin_lock_irqsave(&local.lock_list, flags);
	list_add_tail(&todo_node->list, &todo_list);
	spin_unlock_irqrestore(&local.lock_list, flags);

	queue_push(&local.txop, sbuf, packetlen);	//push
	wake_up_interruptible(&local.tx_wq);	//wakeup

	wait_for_completion_interruptible_timeout(&todo_node->cp,
						  msecs_to_jiffies(1000));

	spin_lock_irqsave(&local.lock_list, flags);
	if (todo_node) {
		list_del(&todo_node->list);
		kfree(todo_node);
	}
	spin_unlock_irqrestore(&local.lock_list, flags);

	return 0;
}

void *t_bif_query_address(enum BUFF_ID buffer_id)
{
#ifdef CONFIG_HOBOT_BIF_AP
	unsigned char sbuf[8] = { 0 };
	unsigned int packetlen = 0;
	unsigned int cur_pid = 0;
	unsigned long flags = 0;
	struct todo_struct *todo_node = NULL;
	void *ptr = NULL;

	if (buffer_id >= BUFF_MAX)
		return (void *)-1;

	ptr = kmalloc(sizeof(struct todo_struct), GFP_ATOMIC | __GFP_ZERO);
	if (ptr == NULL)
		return (void *)-1;
	todo_node = (struct todo_struct *)ptr;

	spin_lock_irqsave(&local.lock_pid, flags);
	cur_pid = local.pid++;
	spin_unlock_irqrestore(&local.lock_pid, flags);

	todo_node->pid = cur_pid;
	init_completion(&todo_node->cp);

	spin_lock_irqsave(&local.lock_list, flags);
	list_add_tail(&todo_node->list, &todo_list);
	spin_unlock_irqrestore(&local.lock_list, flags);

	pr_info(">(%08x) OP_REQ_ADDR\n", cur_pid);

	while (1) {
		sbuf[0] = OP_REQ_ADDR;
		sbuf[1] = 0;
		sbuf[2] = 5;
		sbuf[3] = cur_pid >> 24 & 0xff;
		sbuf[4] = cur_pid >> 16 & 0xff;
		sbuf[5] = cur_pid >> 8 & 0xff;
		sbuf[6] = cur_pid >> 0 & 0xff;
		sbuf[7] = buffer_id;
		packetlen = 8;

		if (bif_netlink_send(BIF_NETLINK_PORTID, sbuf, packetlen) < 0) {
			ssleep(1);
			pr_info("%s() waiting for Ack...\n", __func__);
		} else {
			wait_for_completion_interruptible_timeout
			    (&todo_node->cp, msecs_to_jiffies(3000));
			break;
		}
	}

	spin_lock_irqsave(&local.lock_list, flags);
	if (todo_node) {
		list_del(&todo_node->list);
		kfree(todo_node);
	}
	spin_unlock_irqrestore(&local.lock_list, flags);
#endif

	return local.bif_base.address_list[buffer_id];
}

void *t_bif_query_address_wait(enum BUFF_ID buffer_id)
{
#ifdef CONFIG_HOBOT_BIF_AP
	unsigned char sbuf[8] = { 0 };
	unsigned int packetlen = 0;
	unsigned int cur_pid = 0;
	unsigned long flags = 0;
	struct todo_struct *todo_node = NULL;
	void *ptr = NULL;

	if (buffer_id >= BUFF_MAX)
		return (void *)-1;

	ptr = kmalloc(sizeof(struct todo_struct), GFP_ATOMIC | __GFP_ZERO);
	if (ptr == NULL)
		return (void *)-1;
	todo_node = (struct todo_struct *)ptr;

	spin_lock_irqsave(&local.lock_pid, flags);
	cur_pid = local.pid++;
	spin_unlock_irqrestore(&local.lock_pid, flags);

	todo_node->pid = cur_pid;
	init_completion(&todo_node->cp);

	spin_lock_irqsave(&local.lock_list, flags);
	list_add_tail(&todo_node->list, &todo_list);
	spin_unlock_irqrestore(&local.lock_list, flags);

	pr_info(">(%08x) OP_REQ_ADDR\n", cur_pid);

	while (1) {
		sbuf[0] = OP_REQ_ADDR;
		sbuf[1] = 0;
		sbuf[2] = 5;
		sbuf[3] = cur_pid >> 24 & 0xff;
		sbuf[4] = cur_pid >> 16 & 0xff;
		sbuf[5] = cur_pid >> 8 & 0xff;
		sbuf[6] = cur_pid >> 0 & 0xff;
		sbuf[7] = buffer_id;
		packetlen = 8;

		if (bif_netlink_send(BIF_NETLINK_PORTID, sbuf, packetlen) < 0) {
			ssleep(2);
			pr_info("%s() waiting for Ack...\n", __func__);
		} else {
			wait_for_completion(&todo_node->cp);
			break;
		}
	}

	spin_lock_irqsave(&local.lock_list, flags);
	if (todo_node) {
		list_del(&todo_node->list);
		kfree(todo_node);
	}
	spin_unlock_irqrestore(&local.lock_list, flags);
#endif

	return local.bif_base.address_list[buffer_id];
}

int t_bif_read(void *addr, unsigned int count, char *buf)
{
	unsigned long phy_addr = (unsigned long)addr;
	void *pbuf = buf;
	unsigned short len = MIN(count, BIF_ETHER_MAXSIZE);
	unsigned char sbuf[16] = { 0 };
	unsigned short packetlen = 0, datalen = 0;
	unsigned int cur_pid = 0;
	unsigned long flags = 0;
	unsigned int ack_pid = 0;
	unsigned long ack_addr = 0;
	unsigned short ack_len = 0;
	int ret = 0;
	struct todo_struct *todo_node = NULL;
	void *ptr = NULL;

	if (!local.netlink_start)
		return 0;

	ptr = kmalloc(sizeof(struct todo_struct), GFP_ATOMIC | __GFP_ZERO);
	if (ptr == NULL)
		return 0;
	todo_node = (struct todo_struct *)ptr;

	spin_lock_irqsave(&local.lock_pid, flags);
	cur_pid = local.pid++;
	spin_unlock_irqrestore(&local.lock_pid, flags);

	pr_info(">(%08x) OP_REQ_READ_DATA\n", cur_pid);

	datalen = 10;
	sbuf[0] = OP_REQ_READ_DATA;
	sbuf[1] = datalen >> 8 & 0xff;
	sbuf[2] = datalen >> 0 & 0xff;
	sbuf[3] = cur_pid >> 24 & 0xff;
	sbuf[4] = cur_pid >> 16 & 0xff;
	sbuf[5] = cur_pid >> 8 & 0xff;
	sbuf[6] = cur_pid >> 0 & 0xff;
	sbuf[7] = phy_addr >> 24 & 0xff;
	sbuf[8] = phy_addr >> 16 & 0xff;
	sbuf[9] = phy_addr >> 8 & 0xff;
	sbuf[10] = phy_addr >> 0 & 0xff;
	sbuf[11] = len >> 8 & 0xff;
	sbuf[12] = len >> 0 & 0xff;
	packetlen = datalen + 3;

	todo_node->pid = cur_pid;
	todo_node->phy = phy_addr;
	init_completion(&todo_node->cp);

	spin_lock_irqsave(&local.lock_list, flags);
	list_add_tail(&todo_node->list, &todo_list);
	spin_unlock_irqrestore(&local.lock_list, flags);

	//pr_info("req read phy_addr=0x%lx, len=%d, packetlen=0x%04x\n",
	//phy_addr, len, packetlen);

	queue_push(&local.txop, sbuf, packetlen);
	wake_up_interruptible(&local.tx_wq);

	ret =
	    wait_for_completion_interruptible_timeout(&todo_node->cp,
						      msecs_to_jiffies(2000));
	if (ret == 0) {
		pr_info("(%08x) Err, read timeout! phy_addr=0x%lx, len=%d\n",
			cur_pid, phy_addr, len);
		len = 0;
	}

	if (todo_node->ptr == NULL) {
		pr_info("<(%08x) Err todo_node->ptr NULL, phy_addr=0x%lx\n",
			cur_pid, phy_addr);
		len = 0;
	} else {
		ack_len = todo_node->ptr[1] << 8 | todo_node->ptr[2];
		ack_pid =
		    todo_node->ptr[3] << 24 |
		    todo_node->ptr[4] << 16 |
		    todo_node->ptr[5] << 8 | todo_node->ptr[6] << 0;
		ack_addr =
		    todo_node->ptr[7] << 24 |
		    todo_node->ptr[8] << 16 |
		    todo_node->ptr[9] << 8 | todo_node->ptr[10] << 0;
		ack_len = (ack_len > 7) ? (ack_len - 8) : 0;
		/* 4Byte id, 4Byte phy_addr */

		if (ack_addr != phy_addr) {
			pr_info("(%08x) Err, phy_addr:0x%lx, ack_addr:0x%lx\n",
				cur_pid, phy_addr, ack_addr);
			len = 0;
		}
		if (ack_len != len) {
			pr_info
			    ("(%08x) Err, phy_addr:0x%lx, len:%d, ack_len:%d\n",
			     cur_pid, phy_addr, len, ack_len);
			len = 0;
		} else {
			len = MIN(len, ack_len);
			if (len)
				memcpy(pbuf, todo_node->ptr + 11, len);
		}
	}

	spin_lock_irqsave(&local.lock_list, flags);
	if (todo_node) {
		list_del(&todo_node->list);
		kfree(todo_node);
	}
	spin_unlock_irqrestore(&local.lock_list, flags);

	return len;
}

int t_bif_write(void *addr, unsigned int count, char *buf)
{
	unsigned long phy_addr = (unsigned long)addr;
	void *pbuf = buf;
	unsigned short len = MIN(count, BIF_ETHER_MAXSIZE);
	unsigned short packetlen = 0, datalen = 0;
	unsigned char *sbuf = NULL;
	unsigned int cur_pid = 0;
	unsigned long flags = 0;
	struct todo_struct *todo_node = NULL;
	void *ptr = NULL;

	if (!local.netlink_start)
		return -1;

	sbuf = kmalloc(len + 16, GFP_ATOMIC | __GFP_ZERO);
	if (sbuf == NULL)
		return 0;

	ptr = kmalloc(sizeof(struct todo_struct), GFP_ATOMIC | __GFP_ZERO);
	if (ptr == NULL) {
		kfree(sbuf);
		return 0;
	}
	todo_node = (struct todo_struct *)ptr;

	spin_lock_irqsave(&local.lock_pid, flags);
	cur_pid = local.pid++;
	spin_unlock_irqrestore(&local.lock_pid, flags);

	pr_info(">(%08x) OP_REQ_WRITE_DATA\n", cur_pid);

	datalen = len + 8;
	sbuf[0] = OP_REQ_WRITE_DATA;
	sbuf[1] = datalen >> 8 & 0xff;
	sbuf[2] = datalen >> 0 & 0xff;
	sbuf[3] = cur_pid >> 24 & 0xff;
	sbuf[4] = cur_pid >> 16 & 0xff;
	sbuf[5] = cur_pid >> 8 & 0xff;
	sbuf[6] = cur_pid >> 0 & 0xff;
	sbuf[7] = phy_addr >> 24 & 0xff;
	sbuf[8] = phy_addr >> 16 & 0xff;
	sbuf[9] = phy_addr >> 8 & 0xff;
	sbuf[10] = phy_addr >> 0 & 0xff;
	memcpy(sbuf + 11, pbuf, len);
	packetlen = datalen + 3;

	todo_node->pid = cur_pid;
	todo_node->phy = phy_addr;
	init_completion(&todo_node->cp);

	spin_lock_irqsave(&local.lock_list, flags);
	list_add_tail(&todo_node->list, &todo_list);
	spin_unlock_irqrestore(&local.lock_list, flags);

   //pr_info("req write phy_addr=0x%lx, len=%d, packetlen=0x%04x\n",
   //phy_addr, len, packetlen);
   //pr_buff_16(sbuf, 1);
	queue_push(&local.txop, sbuf, packetlen);
	wake_up_interruptible(&local.tx_wq);
	wait_for_completion_interruptible_timeout(&todo_node->cp,
						  msecs_to_jiffies(2000));

	spin_lock_irqsave(&local.lock_list, flags);
	if (todo_node) {
		list_del(&todo_node->list);
		kfree(todo_node);
	}
	spin_unlock_irqrestore(&local.lock_list, flags);

	kfree(sbuf);

	return len;
}

int t_bif_sd_read(void *addr, unsigned int count, char *buf)
{
	return t_bif_read(addr, count, buf);
}

int t_bif_sd_write(void *addr, unsigned int count, char *buf)
{
	return t_bif_write(addr, count, buf);
}

int t_bif_spi_read(void *addr, unsigned int count, char *buf)
{
	return t_bif_read(addr, count, buf);
}

int t_bif_spi_write(void *addr, unsigned int count, char *buf)
{
	return t_bif_write(addr, count, buf);
}
