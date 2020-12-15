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
#include <soc/hobot/diag.h>

#define IOCS_DIAG_DEV_SELF_TEST 0x01
#define IOCS_DIAG_DEV_UNMASK_ID 0x03
#define IOCS_DIAGDRIVER_STA 0x07
#define IOCS_SEND_SFMU_ERROR 0x08
#define IOCS_FUSA_TEST_POLLING 0x09
#define DIAGDRIVER_STARTWORK 0x01
#define DIAGDRIVER_STOPWORK 0x02
#define SELFTEST_SIZE 4

extern int32_t  diag_dev_init(void);
extern void diag_dev_release(void);
#endif
