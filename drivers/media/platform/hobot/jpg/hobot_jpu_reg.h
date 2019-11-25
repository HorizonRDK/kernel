#ifndef __HOBOT_JPU_REG_H__
#define __HOBOT_JPU_REG_H__

#include <linux/io.h>

#define JPU_READL(addr)			(readl(dev->regs_base + (addr)))
#define JPU_WRITEL(addr, val)	(writel((val), dev->regs_base + (addr)))

#define JPU_REG_BASE_ADDR           		0xA8010000	//0x75300000
#define JPU_REG_SIZE                		0x300

#define NPT_BASE                                0x00	/// Reference X2AJ2A address MAP
#define NPT_REG_SIZE                            0x300	/// 64KB
#define MJPEG_PIC_STATUS_REG(_inst_no)          (NPT_BASE + (_inst_no*NPT_REG_SIZE) + 0x004)

#endif /* __HOBOT_JPU_REG_H__ */
