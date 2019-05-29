/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   bo01.chen@horizon.ai                                                  *
 *                                                                         *
 *   Diag dev header file.                                                 *
 *	 version: v1.0                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef DIAG_DEV_H
#define DIAG_DEV_H
#include <linux/completion.h>

#define IOC_IOC_DIAG_DEV_SELF_TEST 0x01

extern struct completion diag_dev_completion;

extern int  diag_dev_init(void);
extern void diag_dev_release(void);
extern int diag_netlink_init(void);
extern void diag_netlink_exit(void);

#endif
