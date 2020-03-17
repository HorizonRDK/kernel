/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * Zhang Guoying <guoying.zhang@horizon.ai>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#ifndef __J5_BPU_H__
#define __J5_BPU_H__

/* the following marco mast define for every platform */
#define FC_SIZE					(128u)
#define FC_ID_OFFSET			(116u)
#define FC_MAX_DEPTH			(1024u)
#define FC_DEPTH				FC_MAX_DEPTH
#define HW_ID_MAX				0xFFF

#define BPU_PRIO_NUM			(2u)
static inline uint32_t FC_PRIO_ID(uint32_t prio, uint32_t id)
{
	return ((id) & 0x1FF) | ((prio & 0x7) << 9);
}
static inline uint32_t FC_ID(uint32_t id)
{
	return ((id) & 0x1FF);
}
static inline uint32_t FC_PRIO(uint32_t id)
{
	return ((id) >> 9);
}

/*************************************************************
 * J5 cnn register offset list
 *************************************************************/
#define   CNNBUS_CTRL_WM_0		    (0x00u)
#define   CNNBUS_CTRL_WM_1		    (0x04u)
#define   CNNBUS_CTRL_WM_2		    (0x08u)
#define   CNNBUS_CTRL_WM_3		    (0x0Cu)
#define   CNNBUS_CTRL_RM_0		    (0x40u)
#define   CNNBUS_CTRL_RM_1		    (0x44u)
#define   CNNBUS_CTRL_RM_2		    (0x48u)
#define   CNNBUS_CTRL_RM_3		    (0x4Cu)
#define   CNNBUS_CTRL_RM_4		    (0x50u)
#define   CNNBUS_CTRL_RM_5		    (0x54u)
#define   CNNBUS_AXIID		    (0x80)
#define   CNNINT_MASK		    (0x84)
#define   CNNINT_STATUS		    (0x88)
#define   CNNINT_NUM			    (0x8C)
#define   CNNINT_ERR_NUM		    (0x90)
#define   CNN_FC_BASE		    (0x94)
#define   CNN_FC_HEAD		    (0x98)
#define   CNN_FC_TAIL		    (0x9C)
#define   CNN_FC_LEN			    (0xA0)
#define   CNNINT_INST_NUM		    (0xA4)
#define   CNN_INT_PERF_CNT			(0xA8)
#define   CNN_BPU_BUSY				(0xB0)
#define   CNN_FC_CNT_SET			(0xB4)
#define   CNN_FC_CNT_RET			(0xB8)
#define   CNN_ALL_INT_CNT			(0xB8)

/*************************************************************
 * register bit
 *************************************************************/

/*    J5_CNNBUS_CTRL_WM_0    */
#define   J5_CNN_WD_MAXLEN_M_MASK		(0xff00u)
static inline uint32_t J5_CNN_WD_MAXLEN_M(uint32_t n)
{
	return (((n) & 0xffu) << 0x8u);
}
static inline uint32_t J5_CNN_WD_MAXLEN_M_SHIT(uint32_t n)
{
	return (((n) & 0xffu) >> 0x8u);
}

#define   J5_CNN_WD_ENDIAN_M_MASK		(0xf0u)
static inline uint32_t J5_CNN_WD_ENDIAN_M(uint32_t n)
{
	return (((n) & 0xfu) << 0x4u);
}
static inline uint32_t J5_CNN_WD_ENDIAN_M_SHIT(uint32_t n)
{
	return (((n) & 0xfu) >> 0x4u);
}

#define   J5_CNN_WD_PRIORITY_M_MASK		(0xfu)
static inline uint32_t J5_CNN_WD_PRIORITY_M(uint32_t n)
{
	return (((n) & 0xfu) << 0x0u);
}
static inline uint32_t J5_CNN_WD_PRIORITY_M_SHIT(uint32_t n)
{
	return (((n) & 0xfu) >> 0x0u);
}

/*    J5_CNN_CNNBUS_CTRL_RM_0	 */
#define   J5_CNN_RD_MAXLEN_M_MASK		(0xff00u)
static inline uint32_t J5_CNN_RD_MAXLEN_M(uint32_t n)
{
	return (((n) & 0xffu) << 0x8u);
}
static inline uint32_t J5_CNN_RD_MAXLEN_M_SHIT(uint32_t n)
{
	return (((n) & 0xffu) >> 0x8u);
}

#define   J5_CNN_RD_ENDIAN_M_MASK		(0xf0u)
static inline uint32_t J5_CNN_RD_ENDIAN_M(uint32_t n)
{
	return (((n) & 0xfu) << 0x4u);
}
static inline uint32_t J5_CNN_RD_ENDIAN_M_SHIT(uint32_t n)
{
	return (((n) & 0xfu) >> 0x4u);
}

#define   J5_CNN_RD_PRIORITY_M_MASK		(0xfu)
static inline uint32_t J5_CNN_RD_PRIORITY_M(uint32_t n)
{
	return (((n) & 0xfu) << 0x0u);
}
static inline uint32_t J5_CNN_RD_PRIORITY_M_SHIT(uint32_t n)
{
	return (((n) & 0xfu) >> 0x0u);
}

/*    J5_CNN_CNNBUS_AXIID    */
#define   J5_CNN_AXIID_MASK			(0xfu << 0x0u)
static inline uint32_t J5_CNN_AXIID(uint32_t n)
{
	return (((n) & 0xfu) << 0x0u);
}
static inline uint32_t J5_CNN_AXIID_SHIT(uint32_t n)
{
	return (((n) & 0xfu) >> 0x0u);
}

/*    J5_CNN_CNNINT_MASK    */
#define   J5_CNN_PE0_INT_MASK_MASK		(0x1u << 0x0u)
static inline uint32_t J5_CNN_PE0_INT_MASK(uint32_t n)
{
	return (((n) & 0x1u) >> 0x0u);
}
static inline uint32_t J5_CNN_PE0_INT_MASK_SHIT(uint32_t n)
{
	return (((n) & 0x1u) >> 0x0u);
}

/*    J5_CNN_CNNINT_STATUS    */
#define   J5_CNN_PE0_INT_STATUS_RO		(0x1u << 0x0u)
static inline uint32_t J5_CNN_PE0_INT_STATUS_RO_SHIT(uint32_t n)
{
	return (((n) & 0x1u) >> 0x0u);
}

/*    J5_CNN_CNNINT_NUM    */
#define   J5_CNN_INT_NUM_RO			(0xffffu << 0x0u)
static inline uint32_t J5_CNN_INT_NUM_RO_SHIT(uint32_t n)
{
	return (((n) & 0xffffu) >> 0x0u);
}

/*    J5_CNN_CNNINT_ERR_NUM    */
#define   J5_CNN_INT_ERR_NUM_RO			(0xffffu << 0x0u)
static inline uint32_t J5_CNN_INT_ERR_NUM_RO_SHIT(uint32_t n)
{
	return (((n) & 0xffffu) >> 0x0u);
}

/*    J5_CNN_CNNINT_FC_BASE    */
#define   J5_CNN_PE0_FC_BASE_MASK		(0xffffffffu << 0x0u)
static inline uint32_t J5_CNN_PE0_FC_BASE(uint32_t n)
{
	return (((n) & 0xffffffffu) << 0x0u);
}
static inline uint32_t J5_CNN_PE0_FC_BASE_SHIT(uint32_t n)
{
	return (((n) & 0xffffffffu) >> 0x0u);
}

/*    J5_CNN_CNNINT_FC_HEAD    */
#define   J5_CNN_PE0_FC_HEAD_RO			(0x7ffu << 0x0u)
static inline uint32_t J5_CNN_PE0_FC_HEAD_RO_SHIT(uint32_t n)
{
	return (((n) & 0x7ffu) >> 0x0u);
}

/*    J5_CNN_CNNINT_FC_TAIL    */
#define   J5_CNN_PE0_FC_TAIL_MASK		(0x7ffu << 0x0u)
static inline uint32_t J5_CNN_PE0_FC_TAIL(uint32_t n)
{
	return (((n) & 0x7ffu) << 0x0u);
}
static inline uint32_t J5_CNN_PE0_FC_TAIL_SHIT(uint32_t n)
{
	return (((n) & 0x7ffu) >> 0x0u);
}

/*    J5_CNN_CNNINT_FC_LEN    */
#define   J5_CNN_PE0_FC_LENGTH_MASK		(0x3ffu << 0x0u)
static inline uint32_t J5_CNN_PE0_FC_LENGTH(uint32_t n)
{
	return (((n) & 0x3ffu) << 0x0u);
}
static inline uint32_t J5_CNN_PE0_FC_LENGTH_SHIT(uint32_t n)
{
	return (((n) & 0x3ffu) >> 0x0u);
}

/*    J5_CNN_CNNINT_INST_NUM	*/
#define   J5_CNN_INST_NUMBER_RO			(0xffffffffu << 0x0u)
static inline uint32_t J5_CNN_INST_NUMBER_RO_SHIT(uint32_t n)
{
	return (((n) & 0xffffffffu) >> 0x0u);
}

#define J5_CNN_MAX_FC_LEN_MASK	0x3ffu
#define J5_CNN_FC_IDX_FLAG		0x400u
#define J5_CNN_FC_SIZE			FC_SIZE

#define CNN_FC_GAP_LEN			0x1000u
#define CNN_FC_SPACE_LEN		(0x400u * 0x40u)

#define BPU_PMU_REG				0xa6000010u
static inline uint32_t BPU_ISO_BIT(uint32_t index)
{
	return (0x1u << (((index) << 2u) + 1u));
}
//#define BPU_ISO_BIT(index)		(0x1u << (((index) << 2u) + 1u))

#define BPU_MAX_CORE_NUM		2u
#define BPU_ERR_IRQ_MASK		0xF000u
#define BPU_IRQ_MASK			0x0FFFu

extern struct bpu_core_hw_ops j5_hw_ops;

#endif
