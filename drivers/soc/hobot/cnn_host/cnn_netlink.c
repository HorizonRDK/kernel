/*
 * X2 CNN netlink driver (found in Hobot Platform)
 *
 * 2017 - 2018 (C) Hobot Inc.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any
 * later version.
 *
 */

#include <linux/export.h>
#include <linux/etherdevice.h>
#include <linux/netlink.h>
#include <asm/byteorder.h>
#include <net/sock.h>

#define ND_MAX_GROUP		30
#define ND_IFINDEX_LEN		sizeof(int)
#define ND_NLMSG_SPACE(len)	(NLMSG_SPACE(len) + ND_IFINDEX_LEN)
#define ND_NLMSG_DATA(nlh)	((void *)((char *)NLMSG_DATA(nlh) + \
						  ND_IFINDEX_LEN))
#define ND_NLMSG_S_LEN(len)	(len + ND_IFINDEX_LEN)
#define ND_NLMSG_R_LEN(nlh)	(nlh->nlmsg_len - ND_IFINDEX_LEN)
#define ND_NLMSG_IFIDX(nlh)	NLMSG_DATA(nlh)
#define ND_MAX_MSG_LEN		(1024 * 32)

#define CNN_NL_IRQ_TYPE 18

static void cnn_netlink_rcv_cb(struct sk_buff *skb)
{
	struct nlmsghdr	*nlh;
	//struct net_device *dev;
	u32 mlen;
	void *msg;
	int ifindex;

	if (skb->len < NLMSG_HDRLEN) {
		pr_err("cnn nl cbb - invalid skb length\n");
		return;
	}

	nlh = (struct nlmsghdr *)skb->data;

	if (skb->len < nlh->nlmsg_len || nlh->nlmsg_len > ND_MAX_MSG_LEN) {
		pr_err("cnn nl cb - invalid length (%d,%d)\n",
				skb->len, nlh->nlmsg_len);
		return;
	}

	memcpy(&ifindex, ND_NLMSG_IFIDX(nlh), ND_IFINDEX_LEN);
	msg = ND_NLMSG_DATA(nlh);
	mlen = ND_NLMSG_R_LEN(nlh);

	printk("cnn nl cb msg len = %d\n", mlen);

	/*
	   dev = dev_get_by_index(&init_net, ifindex);
	   if (dev) {
	   rcv_cb(dev, nlh->nlmsg_type, msg, mlen);
	   dev_put(dev);
	   } else {
	   pr_err("nl cb - dev (%d) not found\n", ifindex);
	   }
	   */
}

static void cnn_netlink_rcv(struct sk_buff *skb)
{
	cnn_netlink_rcv_cb(skb);
}

struct sock *cnn_netlink_init(int unit)
{
	struct sock *sock;
	struct netlink_kernel_cfg cfg = {
		.input  = cnn_netlink_rcv,
	};

	sock = netlink_kernel_create(&init_net, unit, &cfg);

	return sock;
}

void cnn_netlink_exit(struct sock *sock)
{
	netlink_kernel_release(sock);
}
int cnn_netlink_send(struct sock *sock, int group, void *msg, int len)
{
	static u32 seq;
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh;
	int ret = 0;
	u16 type = CNN_NL_IRQ_TYPE;

	if (group > ND_MAX_GROUP)
		return -EINVAL;

	if (!netlink_has_listeners(sock, group + 1))
		return -ESRCH;

	skb = alloc_skb(NLMSG_SPACE(len), GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	seq++;

	nlh = nlmsg_put(skb, 0, seq, type, len, 0);
	memcpy(NLMSG_DATA(nlh), msg, len);
	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	ret = netlink_broadcast(sock, skb, 0, group + 1, GFP_ATOMIC);
	if (!ret)
		return len;

	if (ret != -ESRCH)
		pr_err("nl broadcast g=%d, t=%d, l=%d, r=%d\n",
				group, type, len, ret);
	else if (netlink_has_listeners(sock, group + 1))
		return -EAGAIN;

	return ret;
}
