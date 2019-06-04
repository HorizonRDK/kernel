/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright	2019 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/
/**
 * BIF ether driver for net
 * @version	2.0
 * @author	haibo.guo(haibo.guo@horizon.ai)
 * @date	2019/04/04
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

#define BIFETH_CPVER		"BIFETH_CPV21"
#define BIFETH_APVER		"BIFETH_APV21"
#define BIFETH_NAME		"bifeth0"
//#define BIFETH_RESERVED_MEM
#define BIFETH_MEMATTRS		0	//DMA_ATTR_WRITE_BARRIER
//#define BIFNET_HALF_FULL_IRQ
//#define BIFETH_IFF_NOARP	//forbid ARP
#define BIFETH_VER_SIZE		(16)
#define BIFETH_BLOCK_SIZE	(BIFSD_BLOCK)
#define BIFETH_SIZE		(3*(BIFETH_BLOCK_SIZE))
#define BIFETH_FRAME_LEN	(ETH_FRAME_LEN)
#define QUEUE_MAX		(ETHER_QUERE_SIZE)
#define ALLOC_ETH_SIZE		((BIFETH_SIZE) * (ETHER_QUERE_SIZE))
#define ALLOC_SIZE		(ALLOC_ETH_SIZE)

#define MAX_SKB_BUFFERS		(20)
#define MAX(a, b)		((a) > (b) ? (a) : (b))
#define MIN(a, b)		((a) < (b) ? (a) : (b))
#define CP_TOUT			(100)
#define WP_TOUT			(1000)
#define ISVAL(t, h, q, v)	(((q)+(t)-(h))%(q) == (v))
//wait_for_completion_timeout
//wait_for_completion_interruptible_timeout
#define bifnet_wc	wait_for_completion_interruptible_timeout

struct bifnet_local {
	int start;
	char ver[BIFETH_VER_SIZE];
	struct bif_ether_info *cp, *ap, *self, *other;
	ulong self_phy;
	ulong other_phy;
	void *self_vir;
	void *other_vir;

	struct workqueue_struct *rx_workqueue;
	struct work_struct rx_work;
	struct task_struct *tx_task;
	struct completion tx_cp;
	wait_queue_head_t tx_wq;
	spinlock_t lock_full;

	struct sk_buff *skbs[MAX_SKB_BUFFERS];
	unsigned char skbs_tail;
	unsigned char skbs_head;

	struct bifplat_info *plat;
	enum BUFF_ID bifnet_id;
	int bifnet_channel;
	char *bifnet;
	int query_ok;

};

static struct net_device *bifnet;
extern int bifget_bifbustype(char *str_bustype);
extern int bifdebug;
#define pr_bif(fmt, args...) do {if (bifdebug) pr_err(fmt, ##args); } while (0)

static struct net_device *get_bifnet(void)
{
	return bifnet;
}

static int get_bifnet_channel(void *p)
{
	struct bifnet_local *pl = (struct bifnet_local *)p;

	if (!pl || !pl->start || !pl->plat)
		return BIFBUS_NO;

	if (pl->plat->plat_type == PLAT_AP)
		return bifget_bifbustype(pl->bifnet);
	else
		return BIFBUS_NO;
}
static int get_rmode(void)
{
	return bif_get_rmode();
}

static int bifnet_sync_cpbuf(void *p, ulong phy_addr, uint blklen,
	unchar *buffer)
{
	ulong cur_phy = phy_addr;
	unsigned int cur_len;
	struct bifnet_local *pl = (struct bifnet_local *)p;

	if (!pl || !pl->start || !pl->plat)
		return -3;

	if (pl->plat->plat_type == PLAT_AP) {
		pl->bifnet_channel = get_bifnet_channel((void *)pl);
		if (pl->bifnet_channel == BIFBUS_SD)
			cur_len = MULTI(blklen, BIFSD_BLOCK);
		else if (pl->bifnet_channel == BIFBUS_SPI)
			cur_len = MULTI(blklen, BIFSPI_BLOCK);
		else
			return -2;
		if (bifread(pl->bifnet_channel,
			(void *)cur_phy, cur_len, buffer))
			return -1;
	}

	return 0;
}

static int bifnet_sync_apbuf(void *p, ulong phy_addr, uint blklen,
	unchar *buffer)
{
	ulong cur_phy = phy_addr;
	unsigned int cur_len;
	struct bifnet_local *pl = (struct bifnet_local *)p;

	if (!pl || !pl->start || !pl->plat)
		return -3;

	if (pl->plat->plat_type == PLAT_AP) {
		pl->bifnet_channel = get_bifnet_channel((void *)pl);
		if (pl->bifnet_channel == BIFBUS_SD)
			cur_len = MULTI(blklen, BIFSD_BLOCK);
		else if (pl->bifnet_channel == BIFBUS_SPI)
			cur_len = MULTI(blklen, BIFSPI_BLOCK);
		else
			return -2;
		if (bifwrite(pl->bifnet_channel,
			(void *)cur_phy, cur_len, buffer))
			return -1;
	}

	return 0;

}
static int bifnet_sync_cp(void *p)
{
	struct bifnet_local *pl = (struct bifnet_local *)p;

	if (!pl || !pl->start || !pl->plat)
		return -3;

	if (pl->plat->plat_type == PLAT_AP) {
		if (get_rmode() == BUFF_ETH)
			return bif_sync_base();
	}

	return 0;
}

static int bifnet_sync_ap(void *p)
{
	struct bifnet_local *pl = (struct bifnet_local *)p;

	if (!pl || !pl->start || !pl->plat)
		return -3;

	if (pl->plat->plat_type == PLAT_AP) {
		if (get_rmode() == BUFF_ETH)
			return bif_sync_ap();
	}

	return 0;
}

static void bifnet_query_addr(void *p, int wait_flag)
{
	void *vir_addr;
	void *phy_addr;
	struct bifnet_local *pl = (struct bifnet_local *)p;

	if (!pl || !pl->plat)
		return;

	//pr_info("bifeth：query address(wait flag=%d)...\n", wait_flag);
	if (pl->plat->plat_type == PLAT_AP) {
		if (wait_flag)
			phy_addr = bif_query_address_wait(pl->bifnet_id);
		else
			phy_addr = bif_query_address(pl->bifnet_id);
		if (phy_addr == (void *)-1) {
			pr_bif("bifnet：%s() Warn bif query address\n",
				__func__);
			pl->query_ok = 0;
		} else {
			pl->self_phy = (ulong)phy_addr + ALLOC_SIZE;
			pl->other_phy = (ulong)phy_addr;

			/*cp side init */
			if (wait_flag)
				vir_addr =
					bif_query_otherbase_wait(pl->bifnet_id);
			else
				vir_addr = bif_query_otherbase(pl->bifnet_id);
			if (vir_addr == (void *)-1) {
				pr_bif("bifnet: Warn bif query otherbase\n");
				pl->query_ok = 0;
			} else {
				pl->cp = (struct bif_ether_info *)(vir_addr);
				pl->query_ok = 1;
			}
		}
		pl->self = pl->ap;
		pl->other = pl->cp;
	} else {
		/*ap side init */
		if (wait_flag)
			vir_addr = bif_query_otherbase_wait(pl->bifnet_id);
		else
			vir_addr = bif_query_otherbase(pl->bifnet_id);
		if (vir_addr == (void *)-1) {
			//pr_warn("bif_eth: Warn bif query otherbase\n");
			pl->query_ok = 0;
		} else {
			pl->ap = (struct bif_ether_info *)(vir_addr);
			pl->query_ok = 1;
		}
		pl->self = pl->cp;
		pl->other = pl->ap;
	}

	if (pl->query_ok) {
		pr_info("self_phy=0x%lx,other_phy=0x%lx\n",
			pl->self_phy, pl->other_phy);
		pr_info("self_vir=0x%lx,other_vir=0x%lx\n",
			(ulong)pl->self_vir, (ulong)pl->other_vir);
		pr_info("cp=0x%lx,ap=0x%lx\n", (ulong)pl->cp, (ulong)pl->ap);
		pl->start = 1;
	}
}

static void bifnet_irq(void *p)
{
	struct bifnet_local *pl = (struct bifnet_local *)p;

	if (!pl || !pl->plat)
		return;

	bifnet_sync_ap((void *)pl);
	/* send irq to bif_base */
	bif_send_irq(pl->bifnet_id);
}
static int rw(void *p, ulong phy, uint len, unchar *vir, int bus, int flg)
{
	unsigned int cur_bus = 0;
	enum PLAT_TYPE cur_plat;

	struct bifnet_local *pl = (struct bifnet_local *)p;

	if (!pl || !pl->start || !pl->plat)
		return -3;

	cur_plat = pl->plat->plat_type;

	if (cur_plat == PLAT_CP)
		return 0;

	cur_bus = get_bifnet_channel((void *)pl);
	if (cur_bus == bus) {
		if (flg)
			return bifnet_sync_cpbuf((void *)pl, phy, len, vir);
		else
			return bifnet_sync_apbuf((void *)pl, phy, len, vir);
	} else
		return 0;
}
static void bifnet_rx_work(struct work_struct *work)
{
	struct sk_buff *skb = NULL;
	unsigned char *cur_vir = NULL;
	unsigned long cur_phy = 0;
	unsigned short cur_elen = 0;
	unsigned int cur_bus = 0;
	unsigned char cur_tail = 0;
	unsigned char cur_head = 0;
	unsigned long flags = 0;
	struct net_device *dev = get_bifnet();
	//struct bifbase_local *pl =
	//	container_of(work, struct bifnet_local, rx_work);
	struct bifnet_local *pl = netdev_priv(dev);

	if (!dev || !pl || !pl->start || !pl->self || !pl->other ||
		!pl->other_phy || !pl->other_vir)
		return;

	bifnet_sync_cp((void *)pl);

	if (pl->self->queue_full) {
		//pr_info("rf\n");
		spin_lock_irqsave(&pl->lock_full, flags);
		pl->self->queue_full = 0;
		spin_unlock_irqrestore(&pl->lock_full, flags);
		if (pl->plat->plat_type == PLAT_AP)
			complete(&pl->tx_cp);
	}
	cur_bus = get_bifnet_channel((void *)pl);
	cur_head = pl->self->recv_head;
	cur_tail = pl->other->send_tail;
	if (cur_head != cur_tail) {
		if (rw((void *)pl, pl->other_phy, ALLOC_SIZE,
			pl->other_vir, BIFBUS_SD, 1)) {
			pr_err("%s: Err bif sync cpbuf\n", dev->name);
			return;
		}
	}
	while (cur_head != cur_tail) {
		if (!dev || !pl->start)
			return;

		cur_phy = pl->other_phy + (cur_head % QUEUE_MAX) * BIFETH_SIZE;
		cur_vir = pl->other_vir + (cur_head % QUEUE_MAX) * BIFETH_SIZE;
		cur_elen = pl->other->elen[cur_head % QUEUE_MAX];
		cur_elen = MIN(cur_elen, ETH_FRAME_LEN);
		if (cur_elen < ETH_HLEN) {
			pr_warn("%s: Warn packet empty\n", dev->name);
			dev->stats.rx_length_errors++;
			cur_head = (cur_head + 1) % QUEUE_MAX;
			continue;
		}

		if (rw((void *)pl, cur_phy, cur_elen,
			cur_vir, BIFBUS_SPI, 1)) {
			pr_err("%s: Err bif sync cpbuf\n", dev->name);
			dev->stats.rx_frame_errors++;
			goto next_step;
		}

		skb = netdev_alloc_skb(dev, cur_elen);
		if (!skb) {
			pr_err("%s: Err Memory alloc skb\n", dev->name);
			continue;
		}

		skb_put(skb, cur_elen);
		memcpy(skb->data, cur_vir, cur_elen);
		skb->len = cur_elen;
		pr_bif("received %d byte packet of type %x\n", cur_elen,
			(skb->data[ETH_ALEN+ETH_ALEN] << 8) |
			skb->data[ETH_ALEN+ETH_ALEN+1]);
#ifdef BIFETH_IFF_NOARP	/* mac */
		if (memcmp(skb->data, dev->dev_addr, ETH_ALEN))
			memcpy(skb->data, dev->dev_addr, ETH_ALEN);
#endif
		skb->protocol = eth_type_trans(skb, dev);
		netif_rx(skb);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += cur_elen;

next_step:
		cur_head = (cur_head + 1) % QUEUE_MAX;
		//if (cur_bus != BIFBUS_SD)
			pl->self->recv_head = cur_head;

#ifdef BIFNET_HALF_FULL_IRQ
		if (pl->other->queue_full && ISVAL(cur_tail, cur_head,
			QUEUE_MAX, QUEUE_MAX/2)) {
			//pr_info("rh\n");
			pl->self->recv_head = cur_head;
			bifnet_irq((void *)pl);
		}
#endif
	}

	if (pl->other->queue_full) {
		//pr_info("rf\n");
		pl->self->recv_head = cur_head;
		bifnet_irq((void *)pl);
	}
}

irqreturn_t bitnet_irq_handler(int irq, void *data)
{
	struct net_device *dev = get_bifnet();
	struct bifnet_local *pl = netdev_priv(dev);

	pr_bif("bifeth: handler irq=%d...\n", irq);

	if (!dev || !pl || !pl->start)
		return IRQ_NONE;

			netif_wake_queue(dev);	/* wakeup netif queue */

	queue_work(pl->rx_workqueue, &pl->rx_work);
	//schedule_work(&pl->rx_work);

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
	struct bifnet_local *pl = netdev_priv(dev);

	pr_info("%s: %s()\n", dev->name, __func__);
	netif_start_queue(dev);
	pl->start = 1;
	bifnet_irq((void *)pl);

	return 0;
}

/* The inverse routine to net_open(). */
static int net_close(struct net_device *dev)
{
	struct bifnet_local *pl = netdev_priv(dev);

	pr_info("%s: %s()\n", dev->name, __func__);
	netif_stop_queue(dev);
	pl->start = 0;

	return 0;
}

static int net_send_thread(void *arg)
{
	struct sk_buff *skb = NULL;
	unsigned char *cur_vir = NULL;
	unsigned long cur_phy = 0;
	unsigned short cur_elen = 0;
	unsigned long flags = 0;
	unsigned int cur_bus = 0;
	unsigned char cur_tail = 0;
	unsigned char cur_head = 0;
	enum PLAT_TYPE cur_plat;
	unsigned short slen[ETHER_QUERE_SIZE];
	unsigned char i = 0;
	unsigned int qflag = 0;
	unsigned int send_flag = 0;

	struct net_device *dev = (struct net_device *)arg;
	//struct net_device *dev = get_bifnet();
	struct bifnet_local *pl = netdev_priv(dev);

	while (!kthread_should_stop()) {

		if (!dev || !pl || !pl->query_ok)
			bifnet_query_addr((void *)pl, 1);
		if (!dev || !pl || !pl->start || !pl->self || !pl->other ||
			!pl->self_phy || !pl->self_vir)
			continue;
		if (pl->start && dev) {
			if (!wait_event_interruptible_timeout(pl->tx_wq,
				pl->skbs_tail != pl->skbs_head,
				msecs_to_jiffies(5*WP_TOUT))) {
				qflag = 0;
				continue;
			}
		}

		cur_bus = get_bifnet_channel((void *)pl);
		cur_tail = pl->self->send_tail;
		cur_plat = pl->plat->plat_type;
		while (pl->skbs_head != pl->skbs_tail) {
			if (!pl->start || !dev)
				break;
			if (ISVAL((pl->skbs_tail + 1), pl->skbs_head,
				MAX_SKB_BUFFERS, MAX_SKB_BUFFERS)) {
				netif_stop_queue(dev);
			//pr_info("s%d-%d\n", pl->skbs_tail, pl->skbs_head);
			} else {
				netif_wake_queue(dev);
				//pr_info("w\n");
			}
			skb = pl->skbs[pl->skbs_head % MAX_SKB_BUFFERS];
			if (!skb) {
				pl->skbs_head =
					(pl->skbs_head + 1) % MAX_SKB_BUFFERS;
				continue;
			}

			cur_head = pl->other->recv_head;
			if ((cur_tail + 1) % QUEUE_MAX == cur_head)
				goto next_step;

			cur_elen = MIN(skb->len + 1, ETH_FRAME_LEN);
			if (cur_elen < ETH_HLEN) {
				pr_warn("%s: Warn empty\n", dev->name);
				pl->skbs_head = (pl->skbs_head + 1) %
					MAX_SKB_BUFFERS;
				if (skb)
					dev_consume_skb_any(skb);
				continue;
			}

			cur_phy = pl->self_phy +
				(cur_tail % QUEUE_MAX) * BIFETH_SIZE;
			cur_vir = pl->self_vir +
				(cur_tail % QUEUE_MAX) * BIFETH_SIZE;
			memset(cur_vir, 0, BIFETH_SIZE);
			memcpy(cur_vir, skb->data, cur_elen);

			if (rw((void *)pl, cur_phy, cur_elen,
				cur_vir, BIFBUS_SPI, 0)) {
				pr_err("%s: Err sync apbuf\n", dev->name);
						dev->stats.tx_errors++;
						goto next_step;
				}

				if (skb)
					dev_consume_skb_any(skb);
			pl->skbs_head = (pl->skbs_head + 1) % MAX_SKB_BUFFERS;
			dev->stats.tx_packets++;
			dev->stats.tx_bytes += cur_elen;
			slen[cur_tail % QUEUE_MAX] = cur_elen;
			cur_tail = (cur_tail + 1) % QUEUE_MAX;
			if (cur_bus != BIFBUS_SD) {
				pl->self->send_tail = cur_tail;
				pl->self->elen[cur_tail % QUEUE_MAX] = cur_elen;
			}
#ifdef BIFNET_HALF_FULL_IRQ
			if (!pl->self->queue_full && !qflag && ISVAL(cur_tail,
				cur_head, QUEUE_MAX, QUEUE_MAX/2)) {
				if (rw((void *)pl, cur_phy, cur_elen, cur_vir,
					cur_bus, 0)) {
					pr_err("%s: Err sync apbuf\n",
						dev->name);
				}
				pl->self->send_tail = cur_tail;
				for (i = 0; i < QUEUE_MAX; i++)
					pl->self->elen[i] = slen[i];
						bifnet_irq((void *)pl);
					qflag = 1;
				}
#endif
next_step:
			if ((cur_tail + 1) % QUEUE_MAX == cur_head &&
				!pl->self->queue_full) {
				//pr_info("f%d-%d\n", cur_tail, cur_head);
				send_flag = 1;
				if (rw((void *)pl, pl->self_phy, ALLOC_SIZE,
					pl->self_vir, BIFBUS_SD, 0)) {
					pr_err("%s: Err sync apbuf\n",
						dev->name);
					break;
				}
				pl->self->send_tail = cur_tail;
				for (i = 0; i < QUEUE_MAX; i++)
					pl->self->elen[i] = slen[i];
				spin_lock_irqsave(&pl->lock_full, flags);
				pl->self->queue_full = 1;
				spin_unlock_irqrestore(&pl->lock_full, flags);
				bifnet_irq((void *)pl);
				if (cur_plat == PLAT_AP)
					if (!bifnet_wc(&pl->tx_cp,
						msecs_to_jiffies(5*CP_TOUT)))
						pr_info("t\n");
			} else
				send_flag = 0;
		}

		netif_wake_queue(dev);
		if (!send_flag) {
			//pr_info("l%d-%d\n", cur_tail, cur_head);
			if (rw((void *)pl, pl->self_phy, ALLOC_SIZE,
				pl->self_vir, BIFBUS_SD, 0)) {
				pr_err("%s: Err sync apbuf\n", dev->name);
			}
			pl->self->send_tail = cur_tail;
			for (i = 0; i < QUEUE_MAX; i++)
				pl->self->elen[i] = slen[i];
			bifnet_irq((void *)pl);
		}
	}

	return 0;
}

static int net_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	int ret = NETDEV_TX_OK;
	struct bifnet_local *pl = netdev_priv(dev);

	if (!pl->start || !pl->query_ok) {
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	if ((pl->skbs_tail + 1) % MAX_SKB_BUFFERS == pl->skbs_head) {
		netif_stop_queue(dev);
		ret = NETDEV_TX_BUSY;
	} else {
		pl->skbs[(pl->skbs_tail) % MAX_SKB_BUFFERS] = skb;
		pl->skbs_tail = (pl->skbs_tail + 1) % MAX_SKB_BUFFERS;
		ret = NETDEV_TX_OK;
	}
	/*
	pr_info("%s: %s(),skbs_tail=%d skbs_head=%d\n", dev->name, __func__,
		pl->skbs_tail, pl->skbs_head);
	*/
	wake_up_interruptible(&pl->tx_wq);

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

static const struct net_device_ops bifnet_ops = {
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

static int bifnet_init(void)
{
	struct net_device *dev;
	struct bifnet_local *pl;
	void *vir_addr;
	int ret = 0;

	pr_info("bifnet: init begin...\n");
	dev = alloc_etherdev(sizeof(struct bifnet_local));
	if (!dev) {
		pr_err("bifnet: %s() Err alloc etherdev!\n", __func__);
		ret = -ENOMEM;
		goto exit_1;
	}

	sprintf(dev->name, "%s", BIFETH_NAME);
	dev->netdev_ops = &bifnet_ops;
#ifdef BIFETH_IFF_NOARP
	dev->flags |= IFF_NOARP;	//ARP
#endif
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
	//Initialize the net local structure.
	pl = netdev_priv(dev);
	memset(pl, 0, sizeof(*pl));
	pl->start = 0;
	pl->bifnet_id = BUFF_ETH;
	pl->plat = (struct bifplat_info *)bif_get_plat_info();
	pl->bifnet = bif_get_str_bus(pl->bifnet_id);
	if (!pl->plat) {
		ret = -1;
		goto exit_2;
	}

	//register netdev
	register_netdev(dev);
	bifnet = dev;

	spin_lock_init(&pl->lock_full);
	init_completion(&pl->tx_cp);
	init_waitqueue_head(&pl->tx_wq);
	pl->tx_task =
		kthread_create(net_send_thread, (void *)bifnet, BIFETH_NAME);
	if (IS_ERR(pl->tx_task)) {
		pr_err("bifnet: %s() Err Unable to start kernel thread",
			__func__);
		ret = PTR_ERR(pl->tx_task);
		pl->tx_task = NULL;
		goto exit_3;
	}
	if (pl->plat->plat_type == PLAT_AP) {
		sprintf(pl->ver, "%s", BIFETH_APVER);
		pl->self_vir = kmalloc(2 * ALLOC_SIZE, GFP_ATOMIC | __GFP_ZERO);
		if (pl->self_vir == NULL) {
			ret = -ENOMEM;
			goto exit_4;
		}
		pl->other_vir = pl->self_vir + ALLOC_SIZE;

		/*ap side init */
		vir_addr = bif_alloc_base(pl->bifnet_id,
			sizeof(struct bif_ether_info));
		if (vir_addr == NULL) {
			ret = -ENOMEM;
			goto exit_5;
		}
		pl->ap = (struct bif_ether_info *)(vir_addr);
	} else {
		sprintf(pl->ver, "%s", BIFETH_CPVER);
#ifdef BIFETH_RESERVED_MEM
		pr_info("bifnet: call bif_alloc_cp()\n");
		pl->self_vir = bif_alloc_cp(pl->bifnet_id, 2 * ALLOC_SIZE,
			&pl->self_phy);
#else
		pr_info("bifnet: call bif_dma_alloc() %d\n", BIFETH_MEMATTRS);
		pl->self_vir = bif_dma_alloc(2 * ALLOC_SIZE,
			(dma_addr_t *)&pl->self_phy,
			GFP_KERNEL, BIFETH_MEMATTRS);
#endif
		if (pl->self_vir == (void *)-1 || pl->self_phy == 0) {
			ret = -ENOMEM;
			goto exit_4;
		}

		bif_register_address(pl->bifnet_id,
			(void *)(ulong)pl->self_phy);
		pl->other_phy = pl->self_phy + ALLOC_SIZE;
		pl->other_vir = pl->self_vir + ALLOC_SIZE;

		/*cp side init */
		vir_addr = bif_alloc_base(pl->bifnet_id,
			sizeof(struct bif_ether_info));
		if (vir_addr == NULL) {
			ret = -ENOMEM;
			goto exit_5;
		}
		pl->cp = (struct bif_ether_info *)(vir_addr);
	}

	pr_info("%s: ver=%s,id=%d\n", dev->name, pl->ver, pl->bifnet_id);
	bifnet_query_addr((void *)pl, 0);
	bif_register_irq(pl->bifnet_id, bitnet_irq_handler);

	//pl->rx_workqueue = create_workqueue("bifnet_rx_wq");
	pl->rx_workqueue = create_singlethread_workqueue("bifnet_rx_wq");
	INIT_WORK(&pl->rx_work, bifnet_rx_work);
	wake_up_process(pl->tx_task);

	if (pl->query_ok)
		pl->start = 1;
	else {
		if (pl->plat->plat_type == PLAT_AP) {
			pr_info("self_vir=0x%lx,other_vir=0x%lx\n",
				(ulong)pl->self_vir, (ulong)pl->other_vir);
			pr_info("ap=0x%lx\n", (ulong)pl->ap);
		} else {
			pr_info("self_phy=0x%lx,other_phy=0x%lx\n",
				pl->self_phy, pl->other_phy);
			pr_info("self_vir=0x%lx,other_vir=0x%lx\n",
				(ulong)pl->self_vir, (ulong)pl->other_vir);
			pr_info("cp=0x%lx\n", (ulong)pl->cp);
		}
	}
	pr_info("bifeth: init end...\n");

	if (ret) {
exit_5:
		if (pl->plat->plat_type == PLAT_AP)
			kfree(pl->self_vir);
		else {
#ifndef BIFETH_RESERVED_MEM
			bif_dma_free(2 * ALLOC_SIZE,
				(dma_addr_t *)&pl->self_phy,
				GFP_KERNEL, BIFETH_MEMATTRS);
#endif
		}
exit_4:
		kthread_stop(pl->tx_task);
exit_3:
		unregister_netdev(dev);
exit_2:
		free_netdev(dev);
exit_1:
		pr_err("bifnet: Err init failed.\n");
	} else {
		pr_info("bifnet: Suc init success.\n");
		pl->start = 1;
	}

	return ret;
}

static void bifnet_exit(void)
{
	unsigned char ch = 0;
	struct net_device *dev = get_bifnet();
	struct bifnet_local *pl = netdev_priv(dev);

	pr_info("bifeth: exit begin...\n");
	pl->start = 0;

	netif_stop_queue(dev);
	complete_all(&pl->tx_cp);

	if (pl->tx_task) {
		ch = pl->skbs_tail;
		pl->skbs_tail = (pl->skbs_head + 1) % MAX_SKB_BUFFERS;
		wake_up_all(&pl->tx_wq);
		if (pl->tx_task) {
			kthread_stop(pl->tx_task);
			pl->tx_task = NULL;
		}
		pl->skbs_tail = ch;
	}

	while (pl->skbs_tail != pl->skbs_head) {
		if (pl->skbs[pl->skbs_head % MAX_SKB_BUFFERS]) {
			dev_kfree_skb(pl->skbs[pl->skbs_head]);
			pl->skbs[pl->skbs_head % MAX_SKB_BUFFERS] = NULL;
		}
		pl->skbs_head = (pl->skbs_head + 1) % MAX_SKB_BUFFERS;
	}

	unregister_netdev(dev);

	if (pl->plat->plat_type == PLAT_AP) {
		kfree(pl->self_vir);
	} else {
#ifndef BIFETH_RESERVED_MEM
		bif_dma_free(2 * ALLOC_SIZE, (dma_addr_t *)&pl->self_phy,
			GFP_KERNEL, BIFETH_MEMATTRS);
#endif
	}

	if (pl->rx_workqueue)
		destroy_workqueue(pl->rx_workqueue);

	free_netdev(dev);

	pr_info("bifeth: exit end...\n");
}

late_initcall(bifnet_init);
//module_init(bif_net_init);
module_exit(bifnet_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("bif ethernet module");
