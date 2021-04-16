#include <linux/netlink.h>
#include <net/sock.h>
#include "ipu_message.h"

#define NETLINK_IPU         30
#define USER_PORT           100

static struct sock *g_nlsk = NULL;
static struct net g_ipu_net;

int send_to_usr(char *pbuf, uint16_t len)
{
	struct sk_buff *nl_skb;
	struct nlmsghdr *nlh;
	int ret = 0;

	/* create sk_buff space */
	nl_skb = nlmsg_new(len, GFP_ATOMIC);
	if (!nl_skb) {
		printk("[ipu] netlink alloc fail\n");
		return -1;
	}

	/* init netlink head */
	nlh = nlmsg_put(nl_skb, 0, 0, NETLINK_IPU, len, 0);
	if (!nlh) {
		printk("[ipu] nlmsg put fail\n");
		nlmsg_free(nl_skb);
		return -2;
	}
	/* copy data & send */
	memcpy(nlmsg_data(nlh), pbuf, len);
	ret = netlink_unicast(g_nlsk, nl_skb, USER_PORT, MSG_DONTWAIT);

	return ret;
}

static void netlink_recv(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = NULL;
	char *umsg = NULL;
	int ret = 1;

	if (skb->len >= nlmsg_total_size(0)) {
		nlh = nlmsg_hdr(skb);
		umsg = NLMSG_DATA(nlh);
		if (umsg) {
			/* kernel recevice from user */
			send_to_usr((char *)&ret, sizeof(int));
		}
	}
}

struct netlink_kernel_cfg nl_cfg = {
	.input = netlink_recv,      /* recv callback */
};

static int __init ipu_netlink_init(void)
{
	g_nlsk = (struct sock *)netlink_kernel_create(&g_ipu_net, NETLINK_IPU, &nl_cfg);
	if (!g_nlsk) {
		printk("[ipu] create netlink kernel fail\n");
		return -1;
	}
	return 0;
}

static void __exit ipu_netlink_exit(void)
{
	if (g_nlsk) {
		netlink_kernel_release(g_nlsk);
		g_nlsk = NULL;
	}
}

module_init(ipu_netlink_init);
module_exit(ipu_netlink_exit);
