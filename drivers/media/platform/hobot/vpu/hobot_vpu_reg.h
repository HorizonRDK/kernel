/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_VPU_REG_H__
#define __HOBOT_VPU_REG_H__

#include <linux/io.h>

typedef enum {
	INT_WAVE5_INIT_VPU = 0,
	INT_WAVE5_WAKEUP_VPU = 1,
	INT_WAVE5_SLEEP_VPU = 2,
	INT_WAVE5_CREATE_INSTANCE = 3,
	INT_WAVE5_FLUSH_INSTANCE = 4,
	INT_WAVE5_DESTORY_INSTANCE = 5,
	INT_WAVE5_INIT_SEQ = 6,
	INT_WAVE5_SET_FRAMEBUF = 7,
	INT_WAVE5_DEC_PIC = 8,
	INT_WAVE5_ENC_PIC = 8,
	INT_WAVE5_ENC_SET_PARAM = 9,
#ifdef SUPPORT_SOURCE_RELEASE_INTERRUPT
	INT_WAVE5_ENC_SRC_RELEASE = 10,
#endif
	INT_WAVE5_ENC_LOW_LATENCY = 13,
	INT_WAVE5_DEC_QUERY = 14,
	INT_WAVE5_BSBUF_EMPTY = 15,
	INT_WAVE5_BSBUF_FULL = 15,
} Wave5InterruptBit;
#endif

#define VPU_READL(addr)			(readl(dev->regs_base	\
								+ dev->bit_fm_info[core].reg_base_offset	\
								+ (addr)))
#define VPU_WRITEL(addr, val)	(writel((val), dev->regs_base	\
								+ dev->bit_fm_info[core].reg_base_offset	\
								+ (addr)))

/* if the platform driver knows this driver */
/* the definition of VPU_REG_BASE_ADDR and VPU_REG_SIZE are not meaningful */

#define VPU_REG_BASE_ADDR 			0xA8000000
#define VPU_REG_SIZE 				(0x4000*MAX_NUM_VPU_CORE)

/* implement to power management functions */
#define BIT_BASE				0x0000
#define BIT_CODE_RUN				(BIT_BASE + 0x000)
#define BIT_CODE_DOWN				(BIT_BASE + 0x004)
#define BIT_INT_CLEAR				(BIT_BASE + 0x00C)
#define BIT_INT_STS				(BIT_BASE + 0x010)
#define BIT_CODE_RESET				(BIT_BASE + 0x014)
#define BIT_INT_REASON				(BIT_BASE + 0x174)
#define BIT_BUSY_FLAG				(BIT_BASE + 0x160)
#define BIT_RUN_COMMAND				(BIT_BASE + 0x164)
#define BIT_RUN_INDEX				(BIT_BASE + 0x168)
#define BIT_RUN_COD_STD				(BIT_BASE + 0x16C)

/* WAVE5 registers */
#define W5_REG_BASE				0x0000
#define W5_VPU_BUSY_STATUS			(W5_REG_BASE + 0x0070)
#define W5_VPU_INT_REASON_CLEAR			(W5_REG_BASE + 0x0034)
#define W5_VPU_VINT_CLEAR			(W5_REG_BASE + 0x003C)
#define W5_VPU_VPU_INT_STS			(W5_REG_BASE + 0x0044)
#define W5_VPU_INT_REASON 			(W5_REG_BASE + 0x004c)
#define W5_RET_FAIL_REASON			(W5_REG_BASE + 0x010C)

#ifdef SUPPORT_MULTI_INST_INTR
#define W5_RET_BS_EMPTY_INST			(W5_REG_BASE + 0x01E4)
#define W5_RET_QUEUE_CMD_DONE_INST		(W5_REG_BASE + 0x01E8)
#define W5_RET_SEQ_DONE_INSTANCE_INFO 		(W5_REG_BASE + 0x01FC)

/* WAVE5 INIT, WAKEUP */
#define W5_PO_CONF 				(W5_REG_BASE + 0x0000)
#define W5_VPU_VINT_ENABLE			(W5_REG_BASE + 0x0048)

#define W5_VPU_RESET_REQ			(W5_REG_BASE + 0x0050)
#define W5_VPU_RESET_STATUS			(W5_REG_BASE + 0x0054)

#define W5_VPU_REMAP_CTRL			(W5_REG_BASE + 0x0060)
#define W5_VPU_REMAP_VADDR			(W5_REG_BASE + 0x0064)
#define W5_VPU_REMAP_PADDR			(W5_REG_BASE + 0x0068)
#define W5_VPU_REMAP_CORE_START		(W5_REG_BASE + 0x006C)
#define W5_CMD_INSTANCE_INFO		(W5_REG_BASE + 0x0110)
#define W5_RST_BLOCK_ALL			(0x3fffffff)

#define W5_REMAP_CODE_INDEX			0

/* WAVE5 registers */
#define W5_ADDR_CODE_BASE			(W5_REG_BASE + 0x0110)
#define W5_CODE_SIZE				(W5_REG_BASE + 0x0114)
#define W5_CODE_PARAM				(W5_REG_BASE + 0x0118)
#define W5_INIT_VPU_TIME_OUT_CNT		(W5_REG_BASE + 0x0130)

#define W5_HW_OPTION				(W5_REG_BASE + 0x012C)

#define W5_RET_SUCCESS				(W5_REG_BASE + 0x0108)

#define W5_COMMAND				(W5_REG_BASE + 0x0100)
#define W5_VPU_HOST_INT_REQ			(W5_REG_BASE + 0x0038)

/* Product register */
#define VPU_PRODUCT_CODE_REGISTER		(BIT_BASE + 0x1044)
#if defined(VPU_SUPPORT_PLATFORM_DRIVER_REGISTER) && defined(CONFIG_PM)
static u32 s_vpu_reg_store[MAX_NUM_VPU_CORE][64];
#endif

#define W5_MAX_CODE_BUF_SIZE			(512*1024)
#define W5_CMD_INIT_VPU				(0x0001)
#define W5_CMD_SLEEP_VPU			(0x0004)
#define W5_CMD_WAKEUP_VPU			(0x0002)
#define W5_DESTROY_INSTANCE			(0x0020)

#endif /*__HOBOT_VPU_REG_H__*/
