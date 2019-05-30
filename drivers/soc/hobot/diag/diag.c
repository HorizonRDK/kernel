/*************************************************************
 ****			 COPYRIGHT NOTICE	          ****
 ****		 Copyright	2019 Horizon Robotics,Inc ****
 ****			 All rights reserved.             ****
 *************************************************************/
/**
 * netlink module,  freight station
 * @author		bo01.chen(bo01.chen@horizon.ai)
 * @date		2019/4/11
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <net/sock.h>
#include <linux/netlink.h>
//#include <x2/diag.h>
#include <x2/diag.h>
#include "diag_dev.h"

#define NETLINK_DIAG 30
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
struct completion diag_dev_completion;

static uint32_t diag_checksum(uint8_t *pbuff, uint32_t len)
{
	uint32_t i;
	uint32_t totalsum;

	i = len;
	totalsum = 0;
	while (i--)
		totalsum += *(pbuff + i);

	return totalsum;
}

/*
 * due to the discontinuity of memory addresses
 * of struct diag_msg, so, copy diag msg to a
 * continuous memory area
 * @msg diag_msg
 */
static uint8_t *diag_msg_copy_to_buffer(diag_msg *msg)
{
	uint8_t *p;

	p = vmalloc(sizeof(diag_msg_hdr) + msg->head.len + 4);
	if (p == NULL) {
		diag_error("vmallc faile\n");
		goto exit;
	}

	memcpy(p, (uint8_t *)msg, sizeof(diag_msg_hdr));
	memcpy(p + sizeof(diag_msg_hdr), msg->data, msg->head.len);
	memcpy(p + sizeof(diag_msg_hdr) + msg->head.len,
					(uint8_t *)&(msg->checksum), 4);

exit:
	return p;
}

/*
 * due to the discontinuity of memory addresses
 * of struct env_data_pack, so, copy env data
 * pack to a continuous memory area
 * @phead env data pack header ptr
 * @pdata env data
 * @len env data len
 */
static uint8_t *diag_msg_copy_envdata_to_buffer(
	env_data_head * phead,
	uint8_t *pdata,
	uint32_t len)
{
	uint8_t *p;

	p = vmalloc(sizeof(env_data_head) + len);
	if (p == NULL) {
		diag_error("%s: vmallc faile\n", __func__);
		goto exit;
	}

	memcpy(p, phead, sizeof(env_data_head));
	memcpy(p + sizeof(env_data_head), pdata, len);

exit:
	return p;
}

/*
 * use netlink to send data to the userspace.
 * @pbuf data ptr
 * @len data len
 * @return -1:error, >=0:OK
 */
static int diag_send_msg(char *pbuf, uint32_t len)
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
	if (ret < 0) {
		diag_error("diag_send_msg %d bytes\n", len);
		return -1;
	}

	return len;
}

/*
 * send event sta and it's env data to the diag app.
 * @event event id
 * @sta event sta
 * @env_data_type When is the environmental data generated.
 * @env_data env data
 * @env_len env data len.
 * @return -1:error, >=0:OK
 */
static DEFINE_MUTEX(diag_msg_snd_mutex);
int diag_send_event_stat_and_env_data(
	diag_event_id event,
	diag_event_stat sta,
	env_data_type env_typ,
	uint8_t *env_data,
	size_t env_len)
{
	diag_msg msg;
	uint8_t *p;
	uint8_t *penvbuff;
	uint8_t ver;
	size_t length;
	size_t total_len = 0;
	report_event_sta_pack event_pack;
	env_data_pack envpack;
	uint32_t frag_cnt;
	uint32_t i;
	uint32_t actual_snd_data_len;
	uint32_t actual_snd_data_rsv_len;

	mutex_lock(&diag_msg_snd_mutex);

	if (event >= EVENT_MAX ||
		sta == DIAG_EVENT_UNKNOWN) {
		goto error;
	}

	/*
	 * event and it's stat report.
	 */
	memset((uint8_t *)&msg, 0x00, sizeof(diag_msg));
	msg.head.packt_ident = DIAG_MSG_HEAD;
	ver = DIAG_MSG_VER;
	msg.head.version |= (ver & 0x0f);
	msg.head.version |= ((~ver << 4) & 0xf0);
	msg.head.type = REPORT_EVENT_STAT;
	event_pack.event_id = event;
	event_pack.sta = sta;
	msg.head.len = sizeof(report_event_sta_pack);
	msg.data = (uint8_t *)&event_pack;
	p = diag_msg_copy_to_buffer(&msg);
	if (p == NULL) {
		diag_error("diag_msg_copy_to_buffer return NULL\n");
		goto error;
	}
	length = sizeof(diag_msg_hdr) + msg.head.len;
	msg.checksum = diag_checksum(p, length);
	memcpy(p + length, &(msg.checksum), 4);
	diag_send_msg(p, length + 4);
	vfree(p);
	total_len += length + 4;

	/*
	 * if env data exist, then send env data.
	 */
	if (env_len == 0 || env_data == NULL) {
		diag_error("env len = 0 or env data is nULL\n");
		goto error;
	}

	/* set env data pack */
	msg.head.type = SEND_ENV_DATA;
	envpack.head.pack_info.event_id = event;
	envpack.head.pack_info.sta = sta;
	envpack.head.type = env_typ;
	envpack.data = env_data;

	//env_len += sizeof(env_data_head);
	actual_snd_data_len = FRAGMENT_SIZE - sizeof(env_data_head);
	actual_snd_data_rsv_len = env_len % actual_snd_data_len;
	frag_cnt = env_len / actual_snd_data_len;
	if (actual_snd_data_rsv_len != 0)
		frag_cnt++;

	diag_debug("%d fragment will send\n", frag_cnt);
	/* send each fragment.*/
	for (i = 0; i < frag_cnt; i++) {
		if (i == 0)
			msg.head.start = 0x01;
		else
			msg.head.start = 0x00;

		if (i == (frag_cnt - 1))
			msg.head.end = 0x01;
		else
			msg.head.end = 0x00;

		msg.head.seq = i;

		if (i == (frag_cnt - 1))
			msg.head.len = sizeof(env_data_head) +
						actual_snd_data_rsv_len;
		else
			msg.head.len = FRAGMENT_SIZE;

		penvbuff = diag_msg_copy_envdata_to_buffer(
			&(envpack.head),
			envpack.data + i*actual_snd_data_len,
			(i == (frag_cnt - 1))?actual_snd_data_rsv_len :
			actual_snd_data_len);

		if (penvbuff == NULL) {
			diag_error("penvbuff at %d return NULL\n", i);
			goto error;
		}

		msg.data = penvbuff;
		p = diag_msg_copy_to_buffer(&msg);
		vfree(penvbuff);
		if (p == NULL) {
			diag_error("copy msg to buffer\%d return NULL\n", i);
			goto error;
		}

		length = sizeof(diag_msg_hdr) + msg.head.len;
		msg.checksum = diag_checksum(p, length);
		memcpy(p + length, &(msg.checksum), 4);
		diag_send_msg(p, length + 4);
		vfree(p);
		total_len += length + 4;
	}

	mutex_unlock(&diag_msg_snd_mutex);
	return total_len;

error:
	mutex_unlock(&diag_msg_snd_mutex);
	return -1;
}
EXPORT_SYMBOL(diag_send_event_stat_and_env_data);

/*
 * diag rcv msg from userspace.
 */
static void netlink_rcv_msg(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = NULL;
	char *umsg = NULL;
	//char *kmsg = "kernel space: hello users!!!";
	unsigned char *data;

	data = kmalloc(100, GFP_KERNEL);
	memset(data, 0x01, 100);
	if (!data) {
		diag_error("kmalloc fail\n");
		return;
	}

	if (skb->len >= nlmsg_total_size(0)) {
		nlh = nlmsg_hdr(skb);
		umsg = NLMSG_DATA(nlh);
		diag_debug("diag:kernel rcv msg: %s\n", umsg);
		if (umsg && (strncmp(umsg, "self test ok", 12)) == 0) {
			mdelay(200);
			complete(&diag_dev_completion);
			diag_debug("diag: complete snd ok\n");
		} else {
			diag_error("diag: self test fail\n");
		}
	}

	kfree(data);
}

struct netlink_kernel_cfg cfg = {
	.input  = netlink_rcv_msg, /* set recv callback */
};

int diag_netlink_init(void)
{
	int ret;

	/* create netlink socket */
	nlsk = (struct sock *)netlink_kernel_create(
		&init_net,
		NETLINK_DIAG,
		&cfg);
	if (nlsk == NULL) {
		diag_error("netlink_kernel_create error !\n");
		return -1;
	}
	diag_debug("diag netlink init exit\n");
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

