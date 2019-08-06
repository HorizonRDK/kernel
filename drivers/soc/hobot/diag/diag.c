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
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/ratelimit.h>
#include <linux/printk.h>
#include <x2/diag.h>
#include "diag_dev.h"

//#define NETLINK_DIAG 30
#define MSG_LEN     200
#define USER_PORT   100

#define USE_KMALLOC 1
#define USE_VMALLOC 2

//#define DEBUG
static int diag_ver[2] __initdata = {1, 0};
struct sock *nlsk;
struct completion diag_dev_completion;

static int diag_is_ready(void);
static struct id_register_struct *diag_id_in_register_list(struct diag_msg_id *id);
static void
diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(struct id_info *pidinfo);

int diag_had_init = -1;
int diag_app_ready = -1;

//DEFINE_SPINLOCK(diag_id_unmask_list_spinlock); // no need??? yes
LIST_HEAD(diag_id_unmask_list);
uint32_t diag_id_unmask_list_num;

struct work_struct	diag_work;

uint32_t diag_list_max_number;

/* group 1 list */
static LIST_HEAD(diag_group1_high_list);
static DEFINE_SPINLOCK(diag_group1_high_list_spinlock);
static uint32_t diag_group1_high_list_num;

static LIST_HEAD(diag_group1_mid_list);
static DEFINE_SPINLOCK(diag_group1_mid_list_spinlock);
static uint32_t diag_group1_mid_list_num;

static LIST_HEAD(diag_group1_low_list);
static DEFINE_SPINLOCK(diag_group1_low_list_spinlock);
static uint32_t diag_group1_low_list_num;

#define LIST_IDLE 1
#define LIST_WAITE_FOR_PROCESS 2
#define LIST_PROCESSING 3
static int diag_group1_list_state;
static DEFINE_SPINLOCK(diag_group1_list_state_spinlock);

/* group 2 list*/
static LIST_HEAD(diag_group2_high_list);
static DEFINE_SPINLOCK(diag_group2_high_list_spinlock);
static uint32_t diag_group2_high_list_num;

static LIST_HEAD(diag_group2_mid_list);
static DEFINE_SPINLOCK(diag_group2_mid_list_spinlock);
static uint32_t diag_group2_mid_list_num;

static LIST_HEAD(diag_group2_low_list);
static DEFINE_SPINLOCK(diag_group2_low_list_spinlock);
static uint32_t diag_group2_low_list_num;

static int diag_group2_list_state;
static DEFINE_SPINLOCK(diag_group2_list_state_spinlock);

//static LIST_HEAD(diag_all_id_record_list);
//static DEFINE_SPINLOCK(diag_all_id_record_list_spinlock);

static DEFINE_SPINLOCK(diag_register_list_spinlock);
static LIST_HEAD(diag_register_list);

/*
 * checksum
 */
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
 * @malloc_choose use kmalloc or vmalloc
 */
static uint8_t *diag_msg_copy_to_buffer(struct diag_msg *msg,
					uint8_t malloc_choose)
{
	uint8_t *p;

	if (malloc_choose == USE_VMALLOC)
		p = vmalloc(sizeof(struct diag_msg_hdr) + msg->head.len + 4);
	else if (malloc_choose == USE_KMALLOC)
		p = kmalloc(sizeof(struct diag_msg_hdr) + msg->head.len + 4,
					GFP_ATOMIC);

	if (p == NULL) {
		pr_err("malloc faile\n");
		goto exit;
	}

	memcpy(p, (uint8_t *)msg, sizeof(struct diag_msg_hdr));
	memcpy(p + sizeof(struct diag_msg_hdr), msg->data, msg->head.len);
	memcpy(p + sizeof(struct diag_msg_hdr) + msg->head.len,
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
 * @malloc_choose use kmalloc or vmalloc
 */
static uint8_t *diag_msg_copy_envdata_to_buffer(
	struct env_data_head *phead,
	uint8_t *pdata,
	uint32_t len,
	uint8_t malloc_choose)
{
	uint8_t *p;

	if (malloc_choose == USE_VMALLOC)
		p = vmalloc(sizeof(struct env_data_head) + len);
	else if (malloc_choose == USE_KMALLOC)
		p = kmalloc(sizeof(struct env_data_head) + len, GFP_ATOMIC);

	if (p == NULL) {
		pr_err("%s: malloc faile\n", __func__);
		goto exit;
	}

	memcpy(p, phead, sizeof(struct env_data_head));
	memcpy(p + sizeof(struct env_data_head), pdata, len);

exit:
	return p;
}

/*
 * use netlink to send data to the userspace.
 * @pbuf data ptr
 * @len data len
 * @return -1:error, >=0:OK
 */
static DEFINE_MUTEX(diag_netlink_send_mutex);
static int diag_send_msg(char *pbuf, uint32_t len)
{
	struct sk_buff *nl_skb;
	struct nlmsghdr *nlh;
	int ret = 0;

	/* create netlink skbuffer. */
	nl_skb = nlmsg_new(len, GFP_ATOMIC);
	if (!nl_skb) {
		pr_err("netlink alloc failure\n");
		return -1;
	}

	/* set netlink header */
	nlh = nlmsg_put(nl_skb, 0, 0, NETLINK_DIAG, len, 0);
	if (nlh == NULL) {
		pr_err("nlmsg_put failaure\n");
		nlmsg_free(nl_skb);
		return -1;
	}

	/* send data*/
	memcpy(nlmsg_data(nlh), pbuf, len);
	mutex_lock(&diag_netlink_send_mutex);
	ret = netlink_unicast(nlsk, nl_skb, USER_PORT, MSG_DONTWAIT);
	mutex_unlock(&diag_netlink_send_mutex);
	if (ret < 0) {
		pr_err("netlink unicast snd fail\n");
		return -1;
	}

	return len;
}
#if 0
/*
 * check diag_msg_id exist in one list
 * @return 1: exist, 0: not exist
 */
static int _diag_id_exist_in_global_list(struct diag_msg_id *id,
					struct list_head *lst)
{
	struct id_info *idinfo;
	int ret = 0;

	if (!list_empty(lst)) {
		list_for_each_entry(idinfo, lst, idlst) {
			if (idinfo->id.module_id == id->module_id &&
				idinfo->id.event_id == id->event_id) {
				ret = 1; // exist
				break;
			}
		}
	}

	return ret;
}
#endif
#if 0
/*
 * check diag_msg_id exist in global list
 * @return 1: exist, 0: not exist
 */
static int diag_id_exist_in_global_list(struct diag_msg_id *id)
{
	unsigned long flags;
	int ret = 0;

	switch (id->msg_pri) {
	case DiagMsgPrioHigh:
		spin_lock_irqsave(&diag_group1_high_list_spinlock, flags);
		ret = _diag_id_exist_in_global_list(id, &diag_group1_high_list);
		spin_unlock_irqrestore(&diag_group1_high_list_spinlock, flags);
		if (ret)
			goto exit;

		spin_lock_irqsave(&diag_group2_high_list_spinlock, flags);
		ret = _diag_id_exist_in_global_list(id, &diag_group2_high_list);
		spin_unlock_irqrestore(&diag_group2_high_list_spinlock, flags);
		break;

	case DiagMsgPrioMid:
		spin_lock_irqsave(&diag_group1_mid_list_spinlock, flags);
		ret = _diag_id_exist_in_global_list(id, &diag_group1_mid_list);
		spin_unlock_irqrestore(&diag_group1_mid_list_spinlock, flags);
		if (ret)
			goto exit;
		spin_lock_irqsave(&diag_group2_mid_list_spinlock, flags);
		ret = _diag_id_exist_in_global_list(id, &diag_group2_mid_list);
		spin_unlock_irqrestore(&diag_group2_mid_list_spinlock, flags);
		break;

	case DiagMsgPrioLow:
		spin_lock_irqsave(&diag_group1_low_list_spinlock, flags);
		ret = _diag_id_exist_in_global_list(id, &diag_group1_low_list);
		spin_unlock_irqrestore(&diag_group1_low_list_spinlock, flags);
		if (ret)
			goto exit;

		spin_lock_irqsave(&diag_group2_low_list_spinlock, flags);
		ret = _diag_id_exist_in_global_list(id, &diag_group2_low_list);
		spin_unlock_irqrestore(&diag_group2_low_list_spinlock, flags);
		break;

	}

exit:
	return ret;
}
#endif
static int diag_insert_idinfo_to_group1_list(struct id_info *id)
{
	unsigned long flags;

	switch (id->id.msg_pri) {
	case DiagMsgPrioHigh:
		spin_lock_irqsave(&diag_group1_high_list_spinlock, flags);
		if (diag_group1_high_list_num >= diag_list_max_number) {
			pr_err("diag group1 high list num over max\n");
			spin_unlock_irqrestore(&diag_group1_high_list_spinlock, flags);
			goto error;
		}
		list_add_tail(&(id->idlst), &diag_group1_high_list);
		diag_group1_high_list_num++;
		spin_unlock_irqrestore(&diag_group1_high_list_spinlock, flags);
		pr_debug("insert idinfo struct to group1 high list\n");
		break;

	case DiagMsgPrioMid:
		spin_lock_irqsave(&diag_group1_mid_list_spinlock, flags);
		if (diag_group1_mid_list_num >= diag_list_max_number) {
			pr_err("diag group1 mid list num over max\n");
			spin_unlock_irqrestore(&diag_group1_mid_list_spinlock, flags);
			goto error;
		}
		list_add_tail(&(id->idlst), &diag_group1_mid_list);
		diag_group1_mid_list_num++;
		spin_unlock_irqrestore(&diag_group1_mid_list_spinlock, flags);
		pr_debug("insert idinfo struct to group1 mid list\n");
		break;

	case DiagMsgPrioLow:
		spin_lock_irqsave(&diag_group1_low_list_spinlock, flags);
		if (diag_group1_low_list_num >= diag_list_max_number) {
			pr_err("diag group1 low list num over max\n");
			spin_unlock_irqrestore(&diag_group1_low_list_spinlock, flags);
			goto error;
		}
		list_add_tail(&(id->idlst), &diag_group1_low_list);
		diag_group1_low_list_num++;
		spin_unlock_irqrestore(&diag_group1_low_list_spinlock, flags);
		pr_debug("insert idinfo struct to group1 low list\n");
		break;

	default:
		pr_err("diag group1 insert list: id msg_pri error\n");
		goto error;
	}

	return 0;

error:
	return -1;
}

static int diag_insert_idinfo_to_group2_list(struct id_info *id)
{
	unsigned long flags;

	switch (id->id.msg_pri) {
	case DiagMsgPrioHigh:
		spin_lock_irqsave(&diag_group2_high_list_spinlock, flags);
		if (diag_group2_high_list_num >= diag_list_max_number) {
			pr_err("diag group2 high list num over max\n");
			spin_unlock_irqrestore(&diag_group2_high_list_spinlock, flags);
			goto error;
		}
		list_add_tail(&(id->idlst), &diag_group2_high_list);
		diag_group2_high_list_num++;
		spin_unlock_irqrestore(&diag_group2_high_list_spinlock, flags);
		pr_debug("insert idinfo struct to group2 high list\n");
		break;

	case DiagMsgPrioMid:
		spin_lock_irqsave(&diag_group2_mid_list_spinlock, flags);
		if (diag_group2_mid_list_num >= diag_list_max_number) {
			pr_err("diag group2 mid list num over max\n");
			spin_unlock_irqrestore(&diag_group2_mid_list_spinlock, flags);
			goto error;
		}
		list_add_tail(&(id->idlst), &diag_group2_mid_list);
		diag_group2_mid_list_num++;
		spin_unlock_irqrestore(&diag_group2_mid_list_spinlock, flags);
		pr_debug("insert idinfo struct to group2 mid list\n");
		break;

	case DiagMsgPrioLow:
		spin_lock_irqsave(&diag_group2_low_list_spinlock, flags);
		if (diag_group2_low_list_num >= diag_list_max_number) {
			pr_err("diag group2 low list num over max\n");
			spin_unlock_irqrestore(&diag_group2_low_list_spinlock, flags);
			goto error;
		}
		list_add_tail(&(id->idlst), &diag_group2_low_list);
		diag_group2_low_list_num++;
		spin_unlock_irqrestore(&diag_group2_low_list_spinlock, flags);
		pr_debug("insert idinfo struct to group2 low list\n");
		break;

	default:
		pr_err("diag group2 insert list: id msg_pri error\n");
		goto error;
	}

	return 0;

error:
	return -1;
}

/*
 * check mask id
 * @return 1:unmask id in list 0:unmask id not in list,
 * so can not send this msg id to userspace.
 */
int diag_unmask_id_in_list(struct diag_msg_id *id)
{
	//unsigned long flags;
	int ret = 0;
	struct diag_msg_id_unmask_struct *id_unmask;

	//spin_lock_irqsave(&diag_id_unmask_list_spinlock, flags);
	list_for_each_entry(id_unmask, &diag_id_unmask_list, mask_lst) {
		if (id_unmask->module_id == id->module_id &&
		    id_unmask->event_id == id->event_id) {
			ret = 1;
			break;
		}
	}
	//spin_unlock_irqrestore(&diag_id_unmask_list_spinlock, flags);
	return ret;
}

static struct id_register_struct *diag_id_in_register_list(struct diag_msg_id *id)
{
	//unsigned long flags;
	struct id_register_struct *idreg;
	struct id_register_struct *result = NULL;

	//spin_lock_irqsave(&diag_register_list_spinlock, flags); // need comment??? yes
	list_for_each_entry(idreg, &diag_register_list, id_register_lst) {
		if (idreg->id.module_id == id->module_id &&
			idreg->id.event_id == id->event_id) {
			result = idreg;
			break;
		}
	}
	//spin_unlock_irqrestore(&diag_register_list_spinlock, flags);

	return result;
}

static int diag_id_snd_condition_is_ok(struct id_register_struct *p, uint8_t current_sta)
{
	unsigned long diff;
	int ret = 0;
	unsigned long flags;

	diff = jiffies_to_msecs(get_jiffies_64()) - p->last_snd_time_ms;
	//pr_debug("%s:time diff: 0x%x\n", __func__, diff);
	spin_lock_irqsave(&diag_register_list_spinlock, flags);
	if ((diff > p->max_time_out_snd_ms) ||
		((diff > p->min_snd_ms) && (current_sta != p->last_sta))) {
		p->last_snd_time_ms = jiffies_to_msecs(get_jiffies_64());
		p->last_sta = current_sta;
		ret = 1;
	}
	spin_unlock_irqrestore(&diag_register_list_spinlock, flags);

	return ret;
}

#if 0
static struct id_info *diag_id_exist_in_record_list(uint16_t module_id, uint16_t event_id)
{
	struct id_info *pidinfo;
	struct id_info *result_idinfo;
	unsigned long flags;
	int ret;

	result_idinfo = NULL;
	ret = 0;
	spin_lock_irqsave(&diag_all_id_record_list_spinlock, flags);
	list_for_each_entry(pidinfo, &diag_all_id_record_list, idlst) {
		if (pidinfo->id.module_id == module_id &&
			pidinfo->id.event_id == event_id) {
			result_idinfo = pidinfo;
			break;
		}
	}
	spin_unlock_irqrestore(&diag_all_id_record_list_spinlock, flags);

	return result_idinfo;
}
#endif
#if 0
static int diag_add_or_update_a_valid_id_to_record_list(uint16_t module_id,
				uint16_t event_id, uint8_t event_sta)
{
	struct id_info *pidinfo;
	int ret;
	unsigned long flags;

	ret = -1;

	pidinfo = diag_id_exist_in_record_list(module_id, event_id);
	if (pidinfo != NULL) {
		// update
		spin_lock_irqsave(&diag_all_id_record_list_spinlock, flags);
		pidinfo->event_sta = event_sta;
		pidinfo->record_jiffies = get_jiffies_64();
		spin_unlock_irqrestore(&diag_all_id_record_list_spinlock, flags);
		ret = 0;
	} else {
		/* alloc idinfo struct */
		pidinfo = (struct id_info *)kmalloc(sizeof(struct id_info), GFP_ATOMIC);
		if (!pidinfo) {
			ret = -1;
			pr_err("%s: kmalloc struct id_info fail\n", __func__);
			goto exit;
		}
		pidinfo->id.module_id = module_id;
		pidinfo->id.event_id = event_id;
		pidinfo->event_sta = event_sta;
		pidinfo->record_jiffies = get_jiffies_64();
		spin_lock_irqsave(&diag_all_id_record_list_spinlock, flags);
		list_add_tail(&(pidinfo->idlst), &diag_all_id_record_list);
		spin_unlock_irqrestore(&diag_all_id_record_list_spinlock, flags);
		ret = 0;
	}

exit:
	return ret;

}
#endif
#if 0
static int diag_id_send_condition_satisfied(uint16_t module_id,
			uint16_t event_id, uint8_t event_sta)
{
	struct id_info *resultpidinfo;
	unsigned long flags;
	int ret;
	uint32_t diff_ms;
	u64 tmp_jiffies;
	uint8_t sta;

	ret = 0;
	resultpidinfo = diag_id_exist_in_record_list(module_id, event_id);
	if (resultpidinfo == NULL) {
		ret = 1;
		goto exit;
	}

	spin_lock_irqsave(&diag_all_id_record_list_spinlock, flags);
	tmp_jiffies = resultpidinfo->record_jiffies;
	sta = resultpidinfo->event_sta;
	spin_unlock_irqrestore(&diag_all_id_record_list_spinlock, flags);

	if (sta == event_sta) { // status had not changed, so condition not satisfied.
		ret = 0;
		goto exit;
	}

	diff_ms = jiffies_to_msecs(get_jiffies_64() - tmp_jiffies);
	if (diff_ms > DIAG_ID_SEND_INTERVAL_MS) {
		ret = 1;
		goto exit;
	}

	/* time condition check. */

exit:
	return ret;
}

/*
 * check: can update env data buffer?
 * If the environment data you send is too large at last,the env buffer maybe
 * not been released(work queue had not process or had not finished), so, this
 * time you want to send data agin(change value in the old buffer), as a result,
 * you can't change any value that the pointer points to. Otherwise,the data
 * will be messed up in the old env buffer.
 */
int diag_can_update_envdata_buffer(uint16_t module_id, uint16_t event_id)
{
	struct diag_msg_id tmpid;

	tmpid.module_id = module_id;
	tmpid.event_id = event_id;

	/* check id exist in all group? */
	if (diag_id_exist_in_global_list(&tmpid)) {
		pr_err("can not change the env buffer, lase send had not finished!!!\n");
		return 0;
	}

	return 1;
}
EXPORT_SYMBOL(diag_can_update_envdata_buffer);
#endif

/*
 * diag driver or diag app is ready?
 * 1:ready, 0:not
 */
static int diag_is_ready(void)
{
	if ((diag_had_init == -1) ||
		(diag_app_ready == -1))
		return 0; // diag core had not init or diag app not ready.

	return 1;
}

/*
 * send event sta and it's env data to the diag app.
 * @msg_prio
 * @module id
 * @event id
 * @event_sta event sta
 * @env_data_gen_timing When is the environmental data generated.
 * @env_data env data ptr
 * @env_len env data len.
 * @return -1:error, -2:not ready, >=0:OK,
 */
//static DEFINE_SPINLOCK(diag_msg_snd_spinlock);
static int diag_do_send_event_stat_and_env_data(
	uint8_t msg_prio,
	uint16_t module_id,
	uint16_t event_id,
	uint8_t event_sta,
	uint8_t env_data_gen_timing,
	uint8_t *env_data,
	size_t env_len)
{
	struct id_info *pidinfo = NULL;
	struct diag_msg_id *id;
	int idle;
	unsigned long flags;
	struct diag_msg_id tmpid;
	struct id_register_struct *pid_reg;
	size_t envdatalength;
	int i;

	tmpid.module_id = module_id;
	tmpid.event_id = event_id;
	tmpid.msg_pri = msg_prio;
	id = &tmpid;

	/* assert */
	if (id->module_id == 0 || id->module_id >= ModuleIdMax ||
		id->event_id == 0 || id->event_id >= EVENT_ID_MAX   ||
		id->msg_pri == 0 || id->msg_pri >= DiagMsgPrioMax ||
		event_sta == 0 || event_sta >= DiagEventStaMax
		) {
		pr_err("id over range\n");
		goto error;
	}

	if (!diag_is_ready()) {
		pr_debug("diag not ready\n");
		goto not_ready;
	}

	/* unmask id check. */
	if (!diag_unmask_id_in_list(id)) {
		//pr_err("id had masked\n");
		//goto error;
		pr_warning("id had masked\n");
		goto ok; // goto ok is correct ???
	}

	/* check id exist in all group? */
	//if (diag_id_exist_in_global_list(id)) {
	//	pr_err("id had exist in gloabal list\n");
	//	goto error;
	//}

#if 0
	if (!diag_id_send_condition_satisfied(id->module_id, id->event_id, event_sta)) {
		pr_err("send condition had not satisfied\n");
		goto error;
	}

	/*add a valid id to the gloabl record list*/
	if (diag_add_or_update_a_valid_id_to_record_list(id->module_id,
									id->event_id, event_sta) < 0) {
		pr_err("can not update on or add id to gloabal record list\n");
		goto error;
	}
#endif
	pid_reg = diag_id_in_register_list(id);
	if (!pid_reg) {
		pr_err("diag id not registered\n");
		goto error;
	}

	if (!diag_id_snd_condition_is_ok(pid_reg, event_sta)) {
		pr_debug("moduleid:%d, eventid:%d,snd condition not satisfiled\n",
				pid_reg->id.module_id, pid_reg->id.event_id);
		goto error;
	}

	/* alloc idinfo struct */
	pidinfo = (struct id_info *)kmalloc(sizeof(struct id_info), GFP_ATOMIC);
	if (!pidinfo) {
		pr_err("kmalloc struct id_info fail\n");
		goto error;
	}

	pidinfo->id.module_id = id->module_id;
	pidinfo->id.event_id = id->event_id;
	pidinfo->id.msg_pri = id->msg_pri;
	pidinfo->event_sta = event_sta;
	pidinfo->env_data_gen_timing = env_data_gen_timing;
	envdatalength = env_len > pid_reg->envdata_max_size ?
				pid_reg->envdata_max_size:env_len;
	if (envdatalength != 0 && env_data != NULL) {
		// if had no envdata buffer alloc require,err
		if (pid_reg->register_had_env_flag != 1) {
			pr_err("err:had no envbuffer when registerd, but now hope to snd envdata\n");
			goto error;
		}
		for (i = 0; i < ENVDATA_BUFFER_NUM; i++) {
			spin_lock_irqsave(&(pid_reg->envdata_buff[i].env_data_buffer_lock), flags);
			if (pid_reg->envdata_buff[i].flags == 0) {
				memcpy(pid_reg->envdata_buff[i].pdata, env_data, envdatalength);
				pid_reg->envdata_buff[i].flags = 1;
				pidinfo->pdata = pid_reg->envdata_buff[i].pdata;
				pidinfo->envlen = envdatalength;
				spin_unlock_irqrestore(&(pid_reg->envdata_buff[i].env_data_buffer_lock), flags);
				break;
			}
			spin_unlock_irqrestore(&(pid_reg->envdata_buff[i].env_data_buffer_lock), flags);
		}
		if (i >= ENVDATA_BUFFER_NUM) {
			pr_debug("env data buffer all busy\n");
			goto error;
		}
	} else {
		pidinfo->pdata = NULL;
		pidinfo->envlen = 0;
	}

	/* insert idinfo struct to global list */
	spin_lock_irqsave(&diag_group1_list_state_spinlock, flags);
	if (diag_group1_list_state == LIST_IDLE ||
		diag_group1_list_state == LIST_WAITE_FOR_PROCESS)
		idle = 1;
	else
		idle = 0;
	spin_unlock_irqrestore(&diag_group1_list_state_spinlock, flags);

	if (idle) {
		if (diag_insert_idinfo_to_group1_list(pidinfo) == -1) {
			pr_err("insert idinfo to group1 list error\n");
			goto env_err;
		}
		spin_lock_irqsave(&diag_group1_list_state_spinlock, flags);
		diag_group1_list_state = LIST_WAITE_FOR_PROCESS;
		spin_unlock_irqrestore(&diag_group1_list_state_spinlock, flags);
	} else {
		spin_lock_irqsave(&diag_group2_list_state_spinlock, flags);
		if (diag_group2_list_state == LIST_IDLE ||
		diag_group2_list_state == LIST_WAITE_FOR_PROCESS)
			idle = 1;
		else
			idle = 0;
		spin_unlock_irqrestore(&diag_group2_list_state_spinlock, flags);

		if (idle) {
			if (diag_insert_idinfo_to_group2_list(pidinfo) == -1) {
				pr_err("insert idinfo to group2 list error\n");
				goto env_err;
			}
			spin_lock_irqsave(&diag_group2_list_state_spinlock, flags);
			diag_group2_list_state = LIST_WAITE_FOR_PROCESS;
			spin_unlock_irqrestore(&diag_group2_list_state_spinlock, flags);
		} else {
			pr_debug("all list is busy\n");
			goto env_err;
		}
	}

	schedule_work(&diag_work);

ok:
	return 0;

env_err:
	diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(pidinfo);

error:
	if (pidinfo)
		kfree(pidinfo);

	//spin_unlock(&diag_msg_snd_spinlock);
	return -1;

not_ready:
	return -2;
}

/*
 * send event sta and it's env data to the diag app.
 * @msg_prio
 * @module id
 * @event id
 * @event_sta event sta
 * @env_data_gen_timing When is the environmental data generated.
 * @env_data env data ptr
 * @env_len env data len.
 * @return -1:error, >=0:OK
 */
//static DEFINE_SPINLOCK(diag_msg_snd_spinlock);
int diag_send_event_stat_and_env_data(
	uint8_t msg_prio,
	uint16_t module_id,
	uint16_t event_id,
	uint8_t event_sta,
	uint8_t env_data_gen_timing,
	uint8_t *env_data,
	size_t env_len)
{
	int ret;
/*
	static DEFINE_RATELIMIT_STATE(diag_snd_envdata_ratelimit, 10 * HZ, 1000);

	if (!__ratelimit(&diag_snd_envdata_ratelimit)) {
		pr_err("diag snd env data too fast\n");
		ret = -1;
		goto exit;
	}
*/
	ret = diag_do_send_event_stat_and_env_data(msg_prio, module_id, event_id,
						event_sta, env_data_gen_timing,
						 env_data, env_len);
//exit:
	return ret;

}
EXPORT_SYMBOL(diag_send_event_stat_and_env_data);

/*
 * send event and sta
 * @msg_prio
 * @module id
 * @event id
 * @event_sta event sta
 * @return -1:error, >=0:OK
 */
int diag_send_event_stat(
		uint8_t msg_prio,
		uint16_t module_id,
		uint16_t event_id,
		uint8_t event_sta
	)
{
	int ret;
/*
	static DEFINE_RATELIMIT_STATE(diag_snd_ratelimit, 10 * HZ, 2000);

	if (!__ratelimit(&diag_snd_ratelimit)) {
		pr_err("diag snd too fast\n");
		ret = -1;
		goto exit;
	}
*/
	ret = diag_do_send_event_stat_and_env_data(msg_prio, module_id,
					event_id, event_sta, 0, NULL, 0);
//exit:
	return ret;
}
EXPORT_SYMBOL(diag_send_event_stat);

static int _diag_send_event_stat_and_env_data(
	struct diag_msg_id *id,
	uint8_t event_sta,
	uint8_t env_data_gen_timing,
	uint8_t *env_data,
	size_t env_len)
{
	struct diag_msg msg;
	uint8_t *p;
	uint8_t *penvbuff;
	uint8_t ver;
	size_t length;
	size_t total_len = 0;
	struct report_event_sta_pack event_pack;
	struct env_data_pack envpack;
	uint32_t frag_cnt;
	uint32_t i;
	uint32_t actual_snd_data_len;
	uint32_t actual_snd_data_rsv_len;
	uint8_t malloc_type;

	/*
	 * event and it's stat report.
	 */
	memset((uint8_t *)&msg, 0x00, sizeof(struct diag_msg));
	msg.head.packt_ident = DIAG_MSG_HEAD;
	ver = DIAG_MSG_VER;
	msg.head.version |= (ver & 0x0f);
	msg.head.version |= ((~ver << 4) & 0xf0);
	msg.head.frame_type = DIAG_MSG_TYPE_REPORT_EVENT_STAT;
	event_pack.id.module_id = id->module_id;
	event_pack.id.msg_pri = id->msg_pri;
	event_pack.id.event_id = id->event_id;

	event_pack.event_sta = event_sta;
	msg.head.len = sizeof(struct report_event_sta_pack);
	msg.data = (uint8_t *)&event_pack;
	malloc_type = USE_VMALLOC;
	p = diag_msg_copy_to_buffer(&msg, malloc_type);
	if (p == NULL) {
		pr_err("diag_msg_copy_to_buffer return NULL\n");
		goto error;
	}
	length = sizeof(struct diag_msg_hdr) + msg.head.len;
	msg.checksum = diag_checksum(p, length);
	memcpy(p + length, &(msg.checksum), 4);
	diag_send_msg(p, length + 4);
	if (malloc_type == USE_VMALLOC)
		vfree(p);
	else
		kfree(p);

	total_len += length + 4;

	/*
	 * if env data exist, then send env data.
	 */
	if (env_len != 0 && env_data == NULL) {
		pr_err("env len != 0 but env data is nULL\n");
		goto error;
	}

	if (env_len == 0)
		goto snd_ok; // have no env data to be sent.

	/* set env data pack */
	msg.head.frame_type = DIAG_MSG_TYPE_SEND_ENV_DATA;
	envpack.head.pack_info.id.module_id = id->module_id;
	envpack.head.pack_info.id.event_id = id->event_id;
	envpack.head.pack_info.id.msg_pri = id->msg_pri;
	envpack.head.pack_info.event_sta = event_sta;
	envpack.head.env_data_gen_timing = env_data_gen_timing;
	envpack.data = env_data;

	//env_len += sizeof(env_data_head);
	actual_snd_data_len = FRAGMENT_SIZE - sizeof(struct env_data_head);
	actual_snd_data_rsv_len = env_len % actual_snd_data_len;
	frag_cnt = env_len / actual_snd_data_len;
	if (actual_snd_data_rsv_len != 0)
		frag_cnt++;

	//pr_debug("%d fragment will send\n", frag_cnt);
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
			msg.head.len = sizeof(struct env_data_head) +
						actual_snd_data_rsv_len;
		else
			msg.head.len = FRAGMENT_SIZE;

		malloc_type = USE_VMALLOC;
		penvbuff = diag_msg_copy_envdata_to_buffer(
			&(envpack.head),
			envpack.data + i*actual_snd_data_len,
			(i == (frag_cnt - 1))?actual_snd_data_rsv_len :
			actual_snd_data_len,
			malloc_type);

		if (penvbuff == NULL) {
			pr_err("penvbuff at %d return NULL\n", i);
			goto error;
		}

		msg.data = penvbuff;
		//malloc_type = USE_KMALLOC;
		p = diag_msg_copy_to_buffer(&msg, malloc_type);
		if (malloc_type == USE_VMALLOC)
			vfree(penvbuff);
		else
			kfree(penvbuff);

		if (p == NULL) {
			pr_err("copy msg to buffer\%d return NULL\n", i);
			goto error;
		}

		length = sizeof(struct diag_msg_hdr) + msg.head.len;
		msg.checksum = diag_checksum(p, length);
		memcpy(p + length, &(msg.checksum), 4);
		diag_send_msg(p, length + 4);
		if (malloc_type == USE_VMALLOC)
			vfree(p);
		else
			kfree(p);

		total_len += length + 4;
	}

snd_ok:
	return total_len;

error:
	return -1;
}

/*
 * down id_info struct form main list, and free it`s memery, then inser it to tmplist.
 */
static int diag_down_entry_form_main_list_insert_to_tmplist(
			struct id_info *pidinfo,
			struct list_head *tmplist)
{
#if 0
	struct id_info *p;
	uint8_t *envda;

	p = (struct id_info *)kmalloc(sizeof(struct id_info), GFP_ATOMIC);
	if (p == NULL) {
		pr_err("diag down entry form main list insert to tmplist kmalloc fail\n");
		goto error;
	}
	p->id.module_id = pidinfo->id.module_id;
	p->id.event_id = pidinfo->id.event_id;
	p->id.msg_pri = pidinfo->id.msg_pri;
	p->event_sta = pidinfo->event_sta;
	p->env_data_gen_timing = pidinfo->env_data_gen_timing;
	p->envlen = pidinfo->envlen;
	p->pdata = pidinfo->pdata;

	/* need to copy env data ? */
	if (p->envlen != 0 && p->pdata != NULL) {
		envda = vmalloc(p->envlen);
		if (envda == NULL) {
			pr_err("vmalloc mem for tmplist fail\n");
			goto error;
		}
		p->pdata = envda;
		memcpy(p->pdata, pidinfo->pdata, p->envlen);
	}
	kfree(pidinfo); // now,free main list entry memery.
#endif

	list_add_tail(&(pidinfo->idlst), tmplist); // wonderful!!!, copy and insert finished.

	return 0;

//error:
	//kfree(pidinfo);
	//return -1;
}

static void
diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(struct id_info *pidinfo)
{
	struct id_register_struct *pid_reg_delet;
	unsigned long flags;
	int i;

	pid_reg_delet = diag_id_in_register_list(&(pidinfo->id));
	if (!pid_reg_delet) {
		pr_err("%s: diag id not registered\n", __func__);
	} else {
		if (pid_reg_delet->register_had_env_flag) {
			spin_lock_irqsave(&diag_register_list_spinlock, flags);
			for (i = 0; i < ENVDATA_BUFFER_NUM; i++) {
				if (pid_reg_delet->envdata_buff[i].pdata == pidinfo->pdata) {
					pid_reg_delet->envdata_buff[i].flags = 0;
					pr_debug("pidinfo released form tmplist and envbuff flag had clean\n");
				}
			}
			spin_unlock_irqrestore(&diag_register_list_spinlock, flags);
		}
	}

	kfree(pidinfo);
}

static int diag_item_in_register_list_envdatabuffer_index(struct id_info *pidinfo,
												struct id_register_struct **result)
{
	struct id_register_struct *pid_reg_delet = NULL;
	int i;
	unsigned long flags;

	if (!pidinfo->pdata)
		return -1;

	pid_reg_delet = diag_id_in_register_list(&(pidinfo->id));
	if (!pid_reg_delet) {
		pr_debug("%s: ptr is NULL\n", __func__);
		return -1;
	}

	spin_lock_irqsave(&diag_register_list_spinlock, flags);
	for (i = 0; i < ENVDATA_BUFFER_NUM; i++) {
		if (pid_reg_delet->envdata_buff[i].pdata == pidinfo->pdata)
			break;
	}
	spin_unlock_irqrestore(&diag_register_list_spinlock, flags);
	if (i >= ENVDATA_BUFFER_NUM)
		return -1;

	*result = pid_reg_delet;

	return i;
}

static void diag_work_handler(struct work_struct *work)
{
	unsigned long flags;
	unsigned long irqsta;
	int group_process = 0;
	struct id_info *pidinfo;
	struct id_info *next;
	struct list_head tmp_list;
	//struct id_info *pidinfo_snd;
	int ret;
	//struct id_register_struct *pid_reg_down;
	//struct diag_msg_id id;
	//int i;
	int index;
	struct id_register_struct *result_tmp;

	INIT_LIST_HEAD(&tmp_list);

	/* process group1 list */
	spin_lock_irqsave(&diag_group1_list_state_spinlock, flags);
	if (diag_group1_list_state == LIST_WAITE_FOR_PROCESS) {
		diag_group1_list_state = LIST_PROCESSING;
		group_process = 1;
	}
	spin_unlock_irqrestore(&diag_group1_list_state_spinlock, flags);

	if (group_process) {
		spin_lock_irqsave(&diag_group1_high_list_spinlock, irqsta);
		list_for_each_entry_safe(pidinfo, next, &diag_group1_high_list, idlst) {
			list_del(&(pidinfo->idlst));
			if (diag_down_entry_form_main_list_insert_to_tmplist(pidinfo, &tmp_list) < 0) {
				diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(pidinfo);
				pr_err("diag down entry form group1 high list insert to tmplist fail\n");
			} else
				pr_debug("diag down entry form group1 high list\n");
		}
		diag_group1_high_list_num = 0;
		spin_unlock_irqrestore(&diag_group1_high_list_spinlock, irqsta);

		spin_lock_irqsave(&diag_group1_mid_list_spinlock, irqsta);
		list_for_each_entry_safe(pidinfo, next, &diag_group1_mid_list, idlst) {
			list_del(&(pidinfo->idlst));
			if (diag_down_entry_form_main_list_insert_to_tmplist(pidinfo, &tmp_list) < 0) {
				diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(pidinfo);
				pr_err("diag down entry form group1 mid list insert to tmplist fail\n");
			}
			else
				pr_debug("diag down entry form group1 mid list\n");
		}
		diag_group1_mid_list_num = 0;
		spin_unlock_irqrestore(&diag_group1_mid_list_spinlock, irqsta);

		spin_lock_irqsave(&diag_group1_low_list_spinlock, irqsta);
		list_for_each_entry_safe(pidinfo, next, &diag_group1_low_list, idlst) {
			list_del(&(pidinfo->idlst));
			if (diag_down_entry_form_main_list_insert_to_tmplist(pidinfo, &tmp_list) < 0) {
				diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(pidinfo);
				pr_err("diag down entry form group1 low list insert to tmplist fail\n");
			} else
				pr_debug("diag down entry form group1 low list\n");
		}
		diag_group1_low_list_num = 0;
		diag_group1_list_state = LIST_IDLE;
		spin_unlock_irqrestore(&diag_group1_low_list_spinlock, irqsta);
	} else {
		group_process = 0;
		/* process group2 list */
		spin_lock_irqsave(&diag_group2_list_state_spinlock, flags);
		if (diag_group2_list_state == LIST_WAITE_FOR_PROCESS) {
			diag_group2_list_state = LIST_PROCESSING;
			group_process = 1;
		}
		spin_unlock_irqrestore(&diag_group2_list_state_spinlock, flags);

		if (group_process) {
			spin_lock_irqsave(&diag_group2_high_list_spinlock, irqsta);
			list_for_each_entry_safe(pidinfo, next, &diag_group2_high_list, idlst) {
				list_del(&(pidinfo->idlst));
				if (diag_down_entry_form_main_list_insert_to_tmplist(pidinfo, &tmp_list) < 0) {
					diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(pidinfo);
					pr_err("diag down entry form group2 high list insert to tmplist fail\n");
				} else
					pr_debug("diag down entry form group2 high list\n");
			}
			diag_group1_high_list_num = 0;
			spin_unlock_irqrestore(&diag_group2_high_list_spinlock, irqsta);

			spin_lock_irqsave(&diag_group2_mid_list_spinlock, irqsta);
			list_for_each_entry_safe(pidinfo, next, &diag_group2_mid_list, idlst) {
				list_del(&(pidinfo->idlst));
				if (diag_down_entry_form_main_list_insert_to_tmplist(pidinfo, &tmp_list) < 0) {
					diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(pidinfo);
					pr_err("diag down entry form group2 mid list insert to tmplist fail\n");
				} else
					pr_debug("diag down entry form group2 mid list\n");
			}
			diag_group1_mid_list_num = 0;
			spin_unlock_irqrestore(&diag_group2_mid_list_spinlock, irqsta);

			spin_lock_irqsave(&diag_group2_low_list_spinlock, irqsta);
			list_for_each_entry_safe(pidinfo, next, &diag_group2_low_list, idlst) {
				list_del(&(pidinfo->idlst));
				if (diag_down_entry_form_main_list_insert_to_tmplist(pidinfo, &tmp_list) < 0) {
					diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(pidinfo);
					pr_err("diag down entry form group2 low list insert to tmplist fail\n");
				} else
					pr_debug("diag down entry form group2 low list\n");
			}
			diag_group1_low_list_num = 0;
			diag_group2_list_state = LIST_IDLE;
			spin_unlock_irqrestore(&diag_group2_low_list_spinlock, irqsta);
		} else {
			pr_err("work handler: have no group to process\n");
		}
	}

	/* now, all group entry had insert to tmplist, and main group list memery had free,
	 * then send all entry to userspace.
	 */
	if (!list_empty(&tmp_list)) {
		list_for_each_entry_safe(pidinfo, next, &tmp_list, idlst) {
			list_del(&(pidinfo->idlst));
			index = diag_item_in_register_list_envdatabuffer_index(pidinfo, &result_tmp);
			if (index >= 0)
				spin_lock_irqsave(&(result_tmp->envdata_buff[index].env_data_buffer_lock), flags);
			ret = _diag_send_event_stat_and_env_data(
					&(pidinfo->id),
					pidinfo->event_sta,
					pidinfo->env_data_gen_timing,
					pidinfo->pdata,
					pidinfo->envlen);
			if (ret < 0) {
				pr_err("send tmplist to userspace fail\n");
			}
			if (index >= 0)
				spin_unlock_irqrestore(&(result_tmp->envdata_buff[index].env_data_buffer_lock), flags);
			diag_clear_registered_envbuffer_flag_and_release_idinfo_struct(pidinfo);
		}
	} else {
		pr_err("tmplist is empty, no data send to userspace\n");
	}
}

int diag_register(uint16_t module_id, uint16_t event_id, size_t envdata_max_size,
		uint32_t min_snd_ms, uint32_t max_time_out_snd_ms,
		void (*rcvcallback)(void *p, size_t len))
{
	//int ret;
	struct id_register_struct *pregister = NULL;
	int i;
	unsigned long flags;
	struct diag_msg_id tmp_id;

	if (envdata_max_size > ENVDATA_MAX_SIZE) {
		pr_err("diag register: envdata_size overun\n");
		goto err;
	}

	tmp_id.module_id = module_id;
	tmp_id.event_id = event_id;
	pregister = diag_id_in_register_list(&tmp_id);
	if (pregister) {
		pr_err("module id: %d, event id: %d register err[had registed]\n",
				module_id, event_id);
		goto err;
	}

	pregister = kmalloc(sizeof(struct id_register_struct), GFP_KERNEL);
	if (!pregister) {
		pr_err("diag register: kmalloc error\n");
		goto err;
	}

	/* init register struct */
	pregister->id.module_id = module_id;
	pregister->id.event_id = event_id;
	pregister->min_snd_ms = min_snd_ms;
	pregister->envdata_max_size = envdata_max_size;
	pregister->msg_rcvcallback = rcvcallback;
	pregister->last_sta = DiagEventStaUnknown;
	pregister->current_sta = DiagEventStaUnknown;
	pregister->last_snd_time_ms = 0;
	pregister->max_time_out_snd_ms = max_time_out_snd_ms;
	for (i = 0; i < ENVDATA_BUFFER_NUM; i++) {
		pregister->envdata_buff[i].pdata = NULL;
		pregister->envdata_buff[i].flags = 0;
		spin_lock_init(&(pregister->envdata_buff[i].env_data_buffer_lock));
	}

	/* alloc env data buffer, if requried */
	if (envdata_max_size != 0) {
		for (i = 0; i < ENVDATA_BUFFER_NUM; i++) {
			pregister->envdata_buff[i].pdata = vmalloc(envdata_max_size);
			if (!(pregister->envdata_buff[i].pdata)) {
				pr_err("diag register: malloc error\n");
				goto kmerr;
			}
		}
		pregister->register_had_env_flag = 1;
	}
	spin_lock_irqsave(&diag_register_list_spinlock, flags);
	list_add_tail(&(pregister->id_register_lst), &diag_register_list);
	spin_unlock_irqrestore(&diag_register_list_spinlock, flags);
	pr_debug("module id: %d, event id: %d register OK\n", module_id, event_id);
	return 0;

kmerr:
	for (i = 0; i < ENVDATA_BUFFER_NUM; i++) {
		if (pregister->envdata_buff[i].pdata) {
			vfree(pregister->envdata_buff[i].pdata);
		}
	}

	if (pregister)
		kfree(pregister);
err:
	return -1;
}
EXPORT_SYMBOL(diag_register);

/*
 * diag rcv msg from userspace.
 */
static void netlink_rcv_msg(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = NULL;
	char *umsg = NULL;
	//char *kmsg = "kernel space: hello users!!!";
	unsigned char *data;

	data = kmalloc(100, GFP_ATOMIC);
	if (!data) {
		pr_err("kmalloc fail\n");
		return;
	}
	memset(data, 0x01, 100);

	if (skb->len >= nlmsg_total_size(0)) {
		nlh = nlmsg_hdr(skb);
		umsg = NLMSG_DATA(nlh);
		//pr_debug("diag:kernel rcv msg: %s\n", umsg);
		if (umsg && (strncmp(umsg, "self test ok", 12)) == 0) {
			mdelay(200);
			complete(&diag_dev_completion);
			//pr_debug("diag: complete snd ok\n");
		} else {
			pr_err("diag: self test fail\n");
		}
	}

	kfree(data);
}

struct netlink_kernel_cfg cfg = {
	.input  = netlink_rcv_msg, /* set recv callback */
};

int diag_netlink_init(void)
{
	/* create netlink socket */
	nlsk = (struct sock *)netlink_kernel_create(
		&init_net,
		NETLINK_DIAG,
		&cfg);
	if (nlsk == NULL) {
		pr_err("netlink_kernel_create error !\n");
		return -1;
	}

	/* group 1 */
	diag_group1_high_list_num = 0;
	diag_group1_mid_list_num = 0;
	diag_group1_low_list_num = 0;
	diag_group1_list_state = LIST_IDLE;

	/* group 2 */
	diag_group2_high_list_num = 0;
	diag_group2_mid_list_num = 0;
	diag_group2_low_list_num = 0;
	diag_group2_list_state = LIST_IDLE;

	diag_list_max_number = DIAG_LIST_MAX_DEFAULT;

	INIT_WORK(&diag_work, diag_work_handler);
	INIT_LIST_HEAD(&diag_id_unmask_list);
	diag_id_unmask_list_num = 0;
	diag_had_init = 1;
	pr_info("diag netlink [ver:%d.%d] init exit\n",
			diag_ver[0], diag_ver[1]);
	return 0;
}

void diag_netlink_exit(void)
{
	if (nlsk) {
		netlink_kernel_release(nlsk); /* release */
		nlsk = NULL;
	}
	pr_debug("diag netlink exit!\n");
}
