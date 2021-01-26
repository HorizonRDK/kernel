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

#ifndef INCLUDE_SOC_HOBOT_HOBOT_EFUSE_H_
#define INCLUDE_SOC_HOBOT_HOBOT_EFUSE_H_
#define EFS_NS 0

/********************************************
 * define secure efuse field
 ********************************************
*/

#define SYSCTL_REG_BASE	0xA1000900
#define SYSCTL_REG_LEN  0x20
#define FW_BASE_REG_ADDR 0xA6008000
#define FW_BASE_SIZE     0x200
#define HW_EFS_CFG	  0x0
#define HW_EFS_BANK_ADDR	0x4
#define HW_EFS_PROG_DATA	0x8
#define HW_EFS_READ_DATA	0xc
#define HW_EFS_STATUS	  0x10
#define HW_EFS_SW_RESET	  0x14

#define HW_EFUSE_READ_OK	  1
#define HW_EFUSE_READ_UNLOCK	  2
#define HW_EFUSE_READ_FAIL	  3
#define HW_EFUSE_WRITE_LOCKED_BNK   4
#define HW_EFUSE_WRITE_FAIL   5
#define HW_EFUSE_LOCK_BNK	  31
#define HW_EFS_CFG_EFS_EN	  (1 << 31)
#define HW_EFS_CFG_EFS_PROG_RD	  (1 << 1)
#define HW_EFS_CFG_EFS_PROG_WR	  (3)
#define HW_EFS_EFS_RDY			(1)
#define HW_EFS_EFS_RDY_PASS		(7)

#define EFUSE_NS_OFF		0x0
#define NS_EFS_RDONE		0x80

#define NS_EFUSE_NUM		32
#define EFUSE_PROG_NUM		31

#define SYSCTL_DUMMY0		0x700

#define TEST_MODE_SHIFT		31

#define RPU_BIFSPI_SLV_DIS	  0x400
#define RPU_BIFSD_SLV_DIS	  0x404
#define RPU_DEFAULT_ADDR	  0x480

#define SEFUSE_NON_SECURE_CHIP	  (1<<4)


#define RET_API_EFUSE_OK    0
#define RET_API_EFUSE_FAIL  0xffffffff

/********************************************
 * define non-secure efuse field
 ********************************************
*/
#define NSEFUSE_BNK_SWCFG				23
#define NSEFUSE_BNK_USB2_UTMI_CFG		24

#define RSA_PUB_KEY_SIZE 256
#define RSA_SIGN_SIZE  256
#define AES_KEY_SIZE 16
#define AES_IV_SIZE 16

#define		SW_EFUSE_FAIL	0xffffffff
#define		SW_EFUSE_OK		0x0
#define		SW_EFUSE_AES_KEY_BANK_LOCK		1
#define		SW_EFUSE_AES_KEY_BANK_NOLOCK	0
#define		SW_EFUSE_RSA_HASH_BANK_LOCK		1
#define		SW_EFUSE_RSA_HASH_BANK_NOLOCK	0
#define		SCOMP_VERIFY_OK			0
#define		SCOMP_VERIFY_FAIL		-1

#define CPU_READ	0
#define CPU_WRITE	1

#define EFUSE_IOCTL_MAGIC 'E'
struct io_efuse_data {
	uint32_t bank;
	uint32_t bank_value;
	bool     lock;
};
struct hobot_efuse_dev {
	dev_t dev_num;
	struct class *dev_class;
	struct device *dev;
	struct cdev i_cdev;
};
#define EFUSE_IOC_GET_BANK	(_IOR(EFUSE_IOCTL_MAGIC, \
					0, struct io_efuse_data))
#define EFUSE_IOC_SET_BANK	(_IOW(EFUSE_IOCTL_MAGIC, \
					1, struct io_efuse_data))
#endif  //  INCLUDE_SOC_HOBOT_HOBOT_EFUSE_H_
