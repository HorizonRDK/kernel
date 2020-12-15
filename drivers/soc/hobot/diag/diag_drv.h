/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *  Author: leye.wang<leye.wang@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __DIAGNOSE_DRIVER_HOBOT_H__
#define __DIAGNOSE_DRIVER_HOBOT_H__ /* PRQA S 0603, 0602 */

#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/netlink.h>

#include <soc/hobot/diag.h> /* PRQA S 1599 */

// #define SYSFS_DEBUG
extern int32_t diag_is_ready(void);

/*
 * information report to upstairs
 */
struct diag_module_event {
	uint16_t module_id;
	uint16_t event_id;
	uint8_t event_sta;
	uint8_t payload[ENV_PAYLOAD_SIZE];
	uint8_t env_len;
	uint8_t when;
	uint8_t ifenv;

	struct list_head list;
};

/*
 * information for each event_id.
 */
struct diag_event_id {
	struct diag_event_id_handle id_handle;
	uint8_t last_sta;				//event last status
	uint32_t last_snd_time;			//event last send time
};

/*
 * allocate for each module.
 */
struct diag_module {
	uint16_t module_id;
	struct diag_event_id event_id[EVENT_ID_MAX];
	uint8_t event_cnt;

	struct list_head list;
};

struct diag {
	/* list for all modules */
	struct list_head module_list;
	struct mutex module_list_mutex;

	/* three priority level list */
	struct list_head low_prio_list;
	struct list_head mid_prio_list;
	struct list_head hig_prio_list;
	spinlock_t low_spinlock;
	spinlock_t mid_spinlock;
	spinlock_t hig_spinlock;

	/* list for netlink send */
	//struct list_head netlink_snd_list;

	struct list_head empty_list;
	spinlock_t empty_spinlock;

	struct work_struct diag_work;

	struct sock *netlink_sock;
	struct mutex netlink_mutex;

	struct diag_module_event *module_event;

#ifdef SYSFS_DEBUG
	struct class *diag_class;
	struct device *diag_device;
	dev_t diag_dev_num;
#endif
};

#endif  /* end of __DIAGNOSE_DRIVER_HOBOT_H__ */
