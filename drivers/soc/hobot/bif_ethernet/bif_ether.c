/********************************************************************
 ****                    COPYRIGHT NOTICE                        ****
 ****        Copyright  2018 Horizon Robotics, Inc.              ****
 ****                    All rights reserved.                    ****
 ********************************************************************/
/**
 * BIF ether driver for net
 * @author			haibo.guo(haibo.guo@horizon.ai)
 * @date			2018/11/21
 */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/icmp.h>
#include <linux/if_ether.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/jiffies.h>
#include <linux/wait.h>
#include "bif_ether.h"
#include "../bif_base/bif_base.h"
#include "../bif_base/bif_api.h"

#define VER_CP	"BIFCP_ETH_V10"
#define VER_AP	"BIFAP_ETH_V10"
/*#define BIF_IFF_NOARP	//forbid ARP */

#ifdef CONFIG_HOBOT_BIF_AP
#define BIF_ETH_VER		VER_AP
#else
#define BIF_ETH_VER		VER_CP
/* use reserved memory*/
#define BIF_USE_RESERVED_MEM
#endif

#define BIF_ETH_NAME	"bif_eth0"
#define BIF_ETH_FRAME_LEN	ETH_FRAME_LEN	/* ether frame length */
#define QUEUE_MAX		ETHER_QUERE_SIZE
#define BIF_ETH_SIZE	(512*3)
#define MAX_SKB_BUFFERS	(20)	/* number of buffers for keeping TX-data */
#define MAX(a, b)		((a) > (b) ? (a) : (b))
#define MIN(a, b)		((a) < (b) ? (a) : (b))

static struct net_device *bif_net;
static struct bif_ether_info *bif_cp, *bif_ap, *self, *other;
static unsigned long self_phy;
static unsigned long other_phy;
void *self_vir;
void *other_vir;

static unsigned char bif_start;
static unsigned char eth_irq;	/* bif send irq */

struct work_struct rx_work;
struct task_struct *tx_task;
struct completion tx_cp;
wait_queue_head_t tx_wq;
spinlock_t lock_full;

struct sk_buff *skbs[MAX_SKB_BUFFERS];	/* pointers to tx-skbs */
unsigned char skbs_tail;
unsigned char skbs_head;
enum BUFF_ID buf_id;
unsigned char query_addr_flg;

/* Information that need to be kept for each board. */
struct net_local {
	char ver[16];		/* ver */
};

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_sync_cpbuf(unsigned long phy_addr, unsigned int blklen,
			  unsigned char *buffer)
{
	unsigned long cur_phy = phy_addr;
#ifdef CONFIG_HOBOT_BIFSD
	unsigned int cur_blklen = 512;
#endif

	if (!bif_start)
		return -1;
#ifdef CONFIG_HOBOT_BIFSD
	if (blklen <= 512)
		cur_blklen = 512;
	else if (blklen <= 1024)
		cur_blklen = 1024;
	else
		cur_blklen = 1536;
	if (bif_sd_read((void *)cur_phy, cur_blklen, buffer))
		return -1;
#else
#ifdef CONFIG_HOBOT_BIFSPI
	if (bif_spi_read((void *)cur_phy, blklen, buffer))
		return -1;
#endif
#endif
	return 0;
}

static int bif_sync_apbuf(unsigned long phy_addr, unsigned int blklen,
			  unsigned char *buffer)
{
	unsigned long cur_phy = phy_addr;
#ifdef CONFIG_HOBOT_BIFSD
	unsigned int cur_blklen = 512;
#endif

	if (!bif_start)
		return -1;
#ifdef CONFIG_HOBOT_BIFSD
	if (blklen <= 512)
		cur_blklen = 512;
	else if (blklen <= 1024)
		cur_blklen = 1024;
	else
		cur_blklen = 1536;
	if (bif_sd_write((void *)cur_phy, cur_blklen, buffer))
		return -1;
#else
#ifdef CONFIG_HOBOT_BIFSPI
	if (bif_spi_write((void *)cur_phy, blklen, buffer))
		return -1;
#endif
#endif
	return 0;
}
#endif

static void bif_eth_query_addr(int wait_flag)
{
	void *vir_addr;

	pr_info("bif_eth query address(wait_flag=%d)...\n", wait_flag);
#ifdef CONFIG_HOBOT_BIF_AP
	if (wait_flag)
		vir_addr = bif_query_address_wait(buf_id);
	else
		vir_addr = bif_query_address(buf_id);
	if (vir_addr == (void *)-1) {
		pr_warn("%s() Warn bif query address\n", __func__);
		query_addr_flg = 0;
	} else {
		self_phy =
		    (unsigned long)vir_addr + (ETHER_QUERE_SIZE * BIF_ETH_SIZE);
		other_phy = (unsigned long)vir_addr;

		/*cp side init */
		if (wait_flag)
			vir_addr = bif_query_otherbase_wait(buf_id);
		else
			vir_addr = bif_query_otherbase(buf_id);
		if (vir_addr == (void *)-1) {
			pr_warn("bif_eth: Warn bif query otherbase\n");
			query_addr_flg = 0;
		} else {
			bif_cp = (struct bif_ether_info *)(vir_addr);
			query_addr_flg = 1;
		}
	}
	self = bif_ap;
	other = bif_cp;
#else
	/*ap side init */
	if (wait_flag)
		vir_addr = bif_query_otherbase_wait(buf_id);
	else
		vir_addr = bif_query_otherbase(buf_id);
	if (vir_addr == (void *)-1) {
		pr_warn("bif_eth: Warn bif query otherbase\n");
		query_addr_flg = 0;
	} else {
		bif_ap = (struct bif_ether_info *)(vir_addr);
		query_addr_flg = 1;
	}
	self = bif_cp;
	other = bif_ap;
#endif

	if (query_addr_flg) {
		pr_info("self_phy=%lx,other_phy=%lx\n", self_phy, other_phy);
		pr_info("self_vir=%lx,other_vir=%lx\n",
			(unsigned long)self_vir, (unsigned long)other_vir);
		pr_info("bif_cp=%lx,bif_ap=%lx\n",
			(unsigned long)bif_cp, (unsigned long)bif_ap);
		bif_start = 1;
	}
}

static void bif_eth_irq(void)
{
	if (!bif_start)
		return;
	/* send irq to bif_base */
	bif_send_irq(eth_irq);
}

static void work_net_rx(struct work_struct *work)
{
	struct net_device *dev = bif_net;
	struct sk_buff *skb = NULL;
	unsigned char *cur_ptr = NULL;
	unsigned long cur_phy = 0;
	unsigned short elen = 0;
	unsigned long flags = 0;

	if (dev == NULL || self == NULL || other == NULL || other_phy == 0
	    || other_vir == NULL || !bif_start)
		return;

	if (self->queue_full) {
		complete(&tx_cp);
		spin_lock_irqsave(&lock_full, flags);
		self->queue_full = 0;
		spin_unlock_irqrestore(&lock_full, flags);
	}

	while (self->recv_head != other->send_tail) {
		if (dev == NULL || !bif_start)
			return;
		cur_phy = other_phy +
			(self->recv_head % QUEUE_MAX) * BIF_ETH_SIZE;
		elen = MIN(other->elen[self->recv_head % QUEUE_MAX],
			ETH_FRAME_LEN);

		if (elen < ETH_HLEN) {
			pr_warn("%s: %s() Warn packet empty\n",
				dev->name, __func__);
			dev->stats.rx_length_errors++;
			self->recv_head = (self->recv_head + 1) % QUEUE_MAX;
			continue;
		}
#ifdef CONFIG_HOBOT_BIF_AP
		cur_ptr = other_vir;
		if (bif_sync_cpbuf(cur_phy, elen, cur_ptr) != 0) {
			pr_err("%s: %s() Err bif sync cpbuf\n",
				dev->name, __func__);
			dev->stats.rx_frame_errors++;
			goto next_step;
		}
#else
		cur_ptr = other_vir +
			(self->recv_head % QUEUE_MAX) * BIF_ETH_SIZE;
#endif
		skb = netdev_alloc_skb(dev, elen);
		if (skb == NULL) {
			pr_err("%s: %s() Err Memory alloc skb\n",
				dev->name, __func__);
			continue;
		} else {
			skb_put(skb, elen);
			memcpy(skb->data, cur_ptr, elen);
			skb->len = elen;
			//pr_info("%s() received %d byte packet of type %x\n",
			//__func__, elen, (skb->data[ETH_ALEN+ETH_ALEN] << 8) |
			//skb->data[ETH_ALEN+ETH_ALEN+1]);
#ifdef BIF_IFF_NOARP
			/* mac */
			if (memcmp(skb->data, dev->dev_addr, ETH_ALEN))
				memcpy(skb->data, dev->dev_addr, ETH_ALEN);
#endif
			skb->protocol = eth_type_trans(skb, dev);
			netif_rx(skb);
			dev->stats.rx_packets++;
			dev->stats.rx_bytes += elen;
		}
#ifdef CONFIG_HOBOT_BIF_AP
next_step:
#endif
		self->recv_head = (self->recv_head + 1) % QUEUE_MAX;
		if (other->queue_full == 0) {
			if (2 *
			    ((QUEUE_MAX + self->recv_head -
			      other->send_tail) % QUEUE_MAX) == QUEUE_MAX)
				bif_eth_irq();
		}
	}

	if (other->queue_full)
		bif_eth_irq();
}

irqreturn_t bit_eth_irq_handler(int irq, void *data)
{
	if (!bif_start)
		return IRQ_NONE;
	if (((skbs_tail + 1) % MAX_SKB_BUFFERS) != skbs_head) {
		if (bif_net)
			netif_wake_queue(bif_net);	/* wakeup netif queue */
	}
	schedule_work(&rx_work);

	return IRQ_HANDLED;
}

/* Open/initialize the board.  This is called (in the current kernel)
 *  sometime after booting when the 'ifconfig' program is run.
 *  This routine should set everything up anew at each open, even
 *  registers that "should" only need to be set once at boot, so that
 *  there is non-reboot way to recover if something goes wrong.
 **/
static int net_open(struct net_device *dev)
{
	pr_info("%s: %s()\n", dev->name, __func__);
	netif_start_queue(dev);
	bif_start = 1;

	return 0;
}

/* The inverse routine to net_open(). */
static int net_close(struct net_device *dev)
{
	pr_info("%s: %s()\n", dev->name, __func__);
	netif_stop_queue(dev);
	bif_start = 0;		//stop

	return 0;
}

static int net_send_thread(void *arg)
{
	struct net_device *dev = bif_net;
	struct sk_buff *skb = NULL;
	unsigned char *cur_ptr = NULL;
	unsigned long cur_phy = 0;
	unsigned short elen = 0;
	unsigned long flags = 0;

	while (!kthread_should_stop()) {

		if (query_addr_flg == 0)
			bif_eth_query_addr(1);
		if (dev == NULL || self == NULL || other == NULL ||
			self_phy == 0 || self_vir == NULL || !bif_start)
			continue;
		if (bif_start && dev)
			if (wait_event_interruptible_timeout(tx_wq,
				skbs_tail != skbs_head, HZ * 10) == 0)
				continue;

		while (skbs_head != skbs_tail) {
			if (!bif_start || dev == NULL)
				break;
			if ((skbs_tail + 1) % MAX_SKB_BUFFERS != skbs_head)
				netif_wake_queue(dev);
			else
				netif_stop_queue(dev);

			skb = skbs[(skbs_head) % MAX_SKB_BUFFERS];
			if (skb == NULL) {
				skbs_head = (skbs_head + 1) % MAX_SKB_BUFFERS;
				continue;
			}

			if (((self->send_tail + 1) % QUEUE_MAX) !=
				other->recv_head) {
				cur_phy = self_phy + (self->send_tail %
					QUEUE_MAX) * BIF_ETH_SIZE;
				elen = MIN(skb->len + 1, ETH_FRAME_LEN);

				if (elen < ETH_HLEN) {
					pr_warn("%s: %s() Warn empty\n",
						dev->name, __func__);
					skbs_head = (skbs_head +
						1) % MAX_SKB_BUFFERS;
					if (skb)
						dev_consume_skb_any(skb);
					continue;
				}
#ifdef CONFIG_HOBOT_BIF_AP
				cur_ptr = self_vir;
				memcpy(cur_ptr, skb->data, elen);
				if (bif_sync_apbuf(cur_phy, elen,
					cur_ptr) != 0) {
					pr_err("%s: %s() Err bif sync apbuf\n",
						dev->name, __func__);
					dev->stats.tx_errors++;
					goto next_step;
				} else {
					dev->stats.tx_packets++;
					dev->stats.tx_bytes += elen;
				}
#else
				cur_ptr = self_vir + (self->send_tail %
					QUEUE_MAX) * BIF_ETH_SIZE;
				memcpy(cur_ptr, skb->data, elen);
				dev->stats.tx_packets++;
				dev->stats.tx_bytes += elen;
#endif
				if (skb)
					dev_consume_skb_any(skb);
				skbs_head = (skbs_head + 1) % MAX_SKB_BUFFERS;
				self->elen[self->send_tail % QUEUE_MAX] = elen;
				self->send_tail =
					(self->send_tail + 1) % QUEUE_MAX;
#ifdef CONFIG_HOBOT_BIF_AP
next_step:
#endif
				if (self->queue_full == 0) {
					if (2*((QUEUE_MAX + self->send_tail -
						other->recv_head) % QUEUE_MAX)
						== QUEUE_MAX)
						bif_eth_irq();
				}
			}

			if (((self->send_tail + 1) % QUEUE_MAX) ==
				other->recv_head) {
				spin_lock_irqsave(&lock_full, flags);
				self->queue_full = 1;
				spin_unlock_irqrestore(&lock_full, flags);
				bif_eth_irq();
				if (bif_start)
					wait_for_completion_timeout(&tx_cp,
						msecs_to_jiffies(500));
			}
		}

		bif_eth_irq();
	}

	return 0;
}

static int net_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	int ret = NETDEV_TX_OK;

	if (!bif_start || query_addr_flg == 0) {
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	if ((skbs_tail + 1) % MAX_SKB_BUFFERS == skbs_head) {
		netif_stop_queue(dev);
		ret = NETDEV_TX_BUSY;
	} else {
		skbs[(skbs_tail) % MAX_SKB_BUFFERS] = skb;
		skbs_tail = (skbs_tail + 1) % MAX_SKB_BUFFERS;
		ret = NETDEV_TX_OK;
	}
	/*
	pr_info("%s: %s(),skbs_tail=%d skbs_head=%d\n", dev->name, __func__,
		skbs_tail, skbs_head);
	*/
	wake_up_interruptible(&tx_wq);

	return ret;
}

static struct net_device_stats *net_get_stats(struct net_device *dev)
{
	return &dev->stats;
}

static void net_timeout(struct net_device *dev)
{
	pr_warn("%s: %s()\n", dev->name, __func__);
	netif_wake_queue(dev);
}

static void set_multicast_list(struct net_device *dev)
{
	;
}

static int set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *saddr = addr;

	if (!is_valid_ether_addr(saddr->sa_data))
		return -EADDRNOTAVAIL;
	memcpy(dev->dev_addr, saddr->sa_data, ETH_ALEN);
	pr_info("%s: Setting MAC address to %pM\n", dev->name, dev->dev_addr);

	return 0;
}

static const struct net_device_ops bif_netdev_ops = {
	.ndo_open = net_open,
	.ndo_stop = net_close,
//	.ndo_tx_timeout = net_timeout,
	.ndo_start_xmit = net_send_packet,
	.ndo_get_stats = net_get_stats,
	.ndo_set_rx_mode = set_multicast_list,
	.ndo_set_mac_address = set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_change_mtu = eth_change_mtu,
};

static int bif_net_init(void)
{
	struct net_device *dev;
	struct net_local *lp;
	void *vir_addr;

	pr_info("bif_eth: init begin...\n");
	bif_start = 0;

	dev = alloc_etherdev(sizeof(struct net_local));
	if (!dev) {
		pr_err("%s() Err alloc_etherdev!\n", __func__);
		return -ENOMEM;
	}
	sprintf(dev->name, "%s", BIF_ETH_NAME);
#if 0
	dev->dev_addr[0] = 0x00;
	dev->dev_addr[1] = 0x12;
	dev->dev_addr[2] = 0x34;
	dev->dev_addr[3] = 0x56;
	dev->dev_addr[4] = 0x78;
	dev->dev_addr[5] = 0x88;
	get_random_bytes((void *)&dev->dev_addr[3], 3);
#else
	random_ether_addr(dev->dev_addr);
#endif

	dev->netdev_ops = &bif_netdev_ops;
#ifdef BIF_IFF_NOARP
	dev->flags |= IFF_NOARP;	//ARP
#endif
	/* Initialize the net local structure. */
	lp = netdev_priv(dev);
	memset(lp, 0, sizeof(*lp));
	sprintf(lp->ver, "%s", BIF_ETH_VER);
	buf_id = BUFF_ETH;
	eth_irq = BUFF_ETH;

	/* register netdev */
	register_netdev(dev);
	bif_net = dev;

	spin_lock_init(&lock_full);
	init_completion(&tx_cp);
	init_waitqueue_head(&tx_wq);
	tx_task = kthread_create(net_send_thread, NULL, "bif_ether");
	if (IS_ERR(tx_task)) {
		pr_err("%s() Unable to start kernel thread.", __func__);
		tx_task = NULL;
		return PTR_ERR(tx_task);
	}
#ifdef CONFIG_HOBOT_BIF_AP
	self_vir = kmalloc(BIF_ETH_SIZE, GFP_ATOMIC | __GFP_ZERO);
	if (self_vir == NULL)
		return -ENOMEM;
	other_vir = kmalloc(BIF_ETH_SIZE, GFP_ATOMIC | __GFP_ZERO);
	if (other_vir == NULL)
		return -ENOMEM;

	/*ap side init */
	vir_addr = bif_alloc_base(buf_id, sizeof(struct bif_ether_info));
	if (vir_addr == NULL)
		return -ENOMEM;

	bif_ap = (struct bif_ether_info *)(vir_addr);
#else
#ifdef BIF_USE_RESERVED_MEM
	self_vir = bif_alloc_cp(buf_id, 2*ETHER_QUERE_SIZE*BIF_ETH_SIZE,
		&self_phy);
#else
	self_vir = dma_alloc_coherent(NULL, 2*ETHER_QUERE_SIZE*BIF_ETH_SIZE,
		&(dma_addr_t)self_phy, GFP_ATOMIC | __GFP_ZERO);
#endif
	bif_register_address(buf_id, (void *)(unsigned long)self_phy);
	other_phy = self_phy + (ETHER_QUERE_SIZE*BIF_ETH_SIZE);
	other_vir = self_vir + (ETHER_QUERE_SIZE*BIF_ETH_SIZE);

	/*cp side init */
	vir_addr = bif_alloc_base(buf_id, sizeof(struct bif_ether_info));
	if (vir_addr == NULL)
		return -ENOMEM;

	bif_cp = (struct bif_ether_info *)(vir_addr);

#endif

	pr_info("%s: ver=%s,id=%d\n", dev->name, lp->ver, buf_id);
	bif_eth_query_addr(0);
	bif_register_irq(buf_id, bit_eth_irq_handler);

	INIT_WORK(&rx_work, work_net_rx);
	wake_up_process(tx_task);

	if (query_addr_flg)
		bif_start = 1;
	else{
#ifdef CONFIG_HOBOT_BIF_AP
		pr_info("self_vir=%lx,other_vir=%lx\n",
			(unsigned long)self_vir, (unsigned long)other_vir);
		pr_info("bif_ap=%lx\n", (unsigned long)bif_ap);
#else
		pr_info("self_phy=%lx,other_phy=%lx\n", self_phy, other_phy);
		pr_info("self_vir=%lx,other_vir=%lx\n",
			(unsigned long)self_vir, (unsigned long)other_vir);
		pr_info("bif_cp=%lx\n", (unsigned long)bif_cp);
#endif
	}
	pr_info("bif_eth: init end...\n");

	return 0;
}

static void bif_net_exit(void)
{
	unsigned char ch = 0;

	pr_info("bif_eth: exit begin...\n");
	bif_start = 0;

	netif_stop_queue(bif_net);
	complete_all(&tx_cp);

	if (tx_task) {
		ch = skbs_tail;
		skbs_tail = (skbs_head + 1) % MAX_SKB_BUFFERS;
		wake_up_all(&tx_wq);
		if (tx_task) {
			kthread_stop(tx_task);
			tx_task = NULL;
		}
		skbs_tail = ch;
	}

	while (skbs_tail != skbs_head) {
		if (skbs[skbs_head % MAX_SKB_BUFFERS]) {
			dev_kfree_skb(skbs[skbs_head]);
			skbs[skbs_head % MAX_SKB_BUFFERS] = NULL;
		}
		skbs_head = (skbs_head + 1) % MAX_SKB_BUFFERS;
	}

	unregister_netdev(bif_net);

#ifdef CONFIG_HOBOT_BIF_AP
	kfree(self_vir);
	kfree(other_vir);
#else
#ifndef BIF_USE_RESERVED_MEM
	if (self_vir)
		dma_free_coherent(NULL, (2*ETHER_QUERE_SIZE*BIF_ETH_SIZE),
			self_vir, self_phy);
#endif
#endif

	free_netdev(bif_net);

	pr_info("bif_eth: exit end...\n");
}

//module_init(bif_net_init);
late_initcall(bif_net_init);
module_exit(bif_net_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("By:hobot, 2018 horizon robotics.");
