/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_JPU_REG_H__
#define __HOBOT_JPU_REG_H__

#include <linux/io.h>

#define JPU_READL(addr)                 (readl(dev->regs_base + (addr)))
#define JPU_WRITEL(addr, val)           (writel((val), dev->regs_base + (addr)))

#define JPU_REG_BASE_ADDR               0xA8010000	//0x75300000
#define JPU_REG_SIZE                    0x300

#define NPT_BASE                        0x00	/// Reference X2AJ2A address MAP
#define NPT_REG_SIZE                    0x300	/// 64KB
#define MJPEG_PIC_STATUS_REG(_inst_no)  (NPT_BASE+(_inst_no*NPT_REG_SIZE)+0x004)
#define MJPEG_PIC_START_REG(_inst_no)   (NPT_BASE + ((_inst_no) * NPT_REG_SIZE) + 0x000)
#define NPT_PROC_BASE                   (NPT_BASE + (4U * NPT_REG_SIZE))
#define MJPEG_INST_CTRL_START_REG       (NPT_PROC_BASE + 0x000)
#define MJPEG_INST_CTRL_STATUS_REG      (NPT_PROC_BASE + 0x004)

typedef enum {
	INST_CTRL_IDLE = 0,
	INST_CTRL_LOAD = 1,
	INST_CTRL_RUN = 2,
	INST_CTRL_PAUSE = 3,
	INST_CTRL_ENC = 4,
	INST_CTRL_PIC_DONE = 5,
	INST_CTRL_SLC_DONE = 6
} InstCtrlStates;

#endif /* __HOBOT_JPU_REG_H__ */
