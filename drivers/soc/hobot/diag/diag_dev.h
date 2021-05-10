/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef DIAG_DEV_H
#define DIAG_DEV_H
#include <linux/completion.h>
#include <soc/hobot/diag.h>

#define IOCS_DIAG_DEV_SELF_TEST 0x01
#define IOCS_DIAG_DEV_UNMASK_ID 0x03
#define IOCS_DIAGDRIVER_STA 0x07
#define DIAGDRIVER_STARTWORK 0x01
#define DIAGDRIVER_STOPWORK 0x02

extern int diag_app_ready;
extern struct completion diag_dev_completion;
extern int  diag_dev_init(void);
extern void diag_dev_release(void);
extern int diag_netlink_init(void);
extern void diag_netlink_exit(void);
extern int diag_unmask_id_in_list(struct diag_msg_id *id);
#endif