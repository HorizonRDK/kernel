/*************************************************************
 ****			 COPYRIGHT NOTICE	          ****
 ****		 Copyright	2019 Horizon Robotics,Inc ****
 ****			 All rights reserved.             ****
 *************************************************************/
/**
 * netlink module,  freight station.
 * @author		bo01.chen(bo01.chen@horizon.ai)
 * @date		2019/4/11
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <net/sock.h>
#include <linux/netlink.h>

#define MSG_LEN     200
#define USER_PORT   100

#define DEBUG

#ifdef DEBUG
#define diag_debug printk
#else
#define diag_debug(format, ...) do {} while (0)
#endif

#define diag_error printk

struct sock *nlsk;

int send_diag_msg(char *pbuf, uint16_t len)
{
	struct sk_buff *nl_skb;
	struct nlmsghdr *nlh;

	int ret = 0;

	/* create netlink skbuffer. */
	nl_skb = nlmsg_new(len, GFP_ATOMIC);
	if (!nl_skb) {
		diag_error("netlink alloc failure\n");
		return -1;
	}

	/* set netlink header */
	nlh = nlmsg_put(nl_skb, 0, 0, NETLINK_DIAG, len, 0);
	if (nlh == NULL) {
		diag_error("nlmsg_put failaure\n");
		nlmsg_free(nl_skb);
		return -1;
	}

	/* send data*/
	memcpy(nlmsg_data(nlh), pbuf, len);
	ret = netlink_unicast(nlsk, nl_skb, USER_PORT, MSG_DONTWAIT);

	return ret;
}
EXPORT_SYMBOL(send_diag_msg);

static void netlink_rcv_msg(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = NULL;
	char *umsg = NULL;
	char *kmsg = "kernel space: hello users!!!";

	if (skb->len >= nlmsg_total_size(0)) {
		nlh = nlmsg_hdr(skb);
		umsg = NLMSG_DATA(nlh);
		if (umsg) {
			diag_debug("userspace: %s\n", umsg);
			send_diag_msg(kmsg, strlen(kmsg));
		}
	}
}

struct netlink_kernel_cfg cfg = {
	.input  = netlink_rcv_msg, /* set recv callback */
};

int diag_netlink_init(void)
{
	/* create netlink socket */
	nlsk = netlink_kernel_create(&init_net, NETLINK_DIAG, &cfg);
	if (nlsk == NULL) {
		diag_error("netlink_kernel_create error\n");
		return -1;
	}
	diag_debug("diag netlink init\n");
	return 0;
}

void diag_netlink_exit(void)
{
	if (nlsk) {
		netlink_kernel_release(nlsk); /* release */
		nlsk = NULL;
	}
	diag_debug("diag netlink exit!\n");
}

module_init(diag_netlink_init);
module_exit(diag_netlink_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("bo01.chen@horizon.ai");
MODULE_DESCRIPTION("fusa netlink module");
