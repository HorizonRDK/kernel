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
#ifndef __X2_BPU_H__
#define __X2_BPU_H__

/* the following marco mast define for every platform */
#define FC_SIZE					64
#define FC_ID_OFFSET			30
#define FC_MAX_DEPTH			1024
#define FC_DEPTH				FC_MAX_DEPTH
#define HW_ID_MAX				0xFFF
#define BPU_PRIO_NUM			0x1
#define FC_PRIO_ID(prio, id)	((id) & 0x1FF) | ((prio & 0x7) << 9)
#define FC_ID(id)				((id) & 0x1FF)
#define FC_PRIO(id)				(0)

/*************************************************************
 * x2 cnn register offset list
 *************************************************************/
#define   CNNBUS_CTRL_WM_0		    (0x00)
#define   CNNBUS_CTRL_WM_1		    (0x04)
#define   CNNBUS_CTRL_WM_2		    (0x08)
#define   CNNBUS_CTRL_WM_3		    (0x0C)
#define   CNNBUS_CTRL_RM_0		    (0x40)
#define   CNNBUS_CTRL_RM_1		    (0x44)
#define   CNNBUS_CTRL_RM_2		    (0x48)
#define   CNNBUS_CTRL_RM_3		    (0x4C)
#define   CNNBUS_CTRL_RM_4		    (0x50)
#define   CNNBUS_CTRL_RM_5		    (0x54)
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

/*************************************************************
 * register bit
 *************************************************************/

/*    X2_CNNBUS_CTRL_WM_0    */
#define   X2_CNN_WD_MAXLEN_M(n)			(((n) & 0xff) << 0x8)
#define   X2_CNN_WD_MAXLEN_M_MASK		(0xff << 0x8)
#define   X2_CNN_WD_MAXLEN_M_SHIT(n)		(((n) & 0xff) >> 0x8)
#define   X2_CNN_WD_ENDIAN_M(n)			(((n) & 0xf) << 0x4)
#define   X2_CNN_WD_ENDIAN_M_MASK		(0xf << 0x4)
#define   X2_CNN_WD_ENDIAN_M_SHIT(n)		(((n) & 0xf) >> 0x4)
#define   X2_CNN_WD_PRIORITY_M(n)		(((n) & 0xf) << 0x0)
#define   X2_CNN_WD_PRIORITY_M_MASK		(0xf << 0x0)
#define   X2_CNN_WD_PRIORITY_M_SHIT(n)		(((n) & 0xf) >> 0x0)

/*    X2_CNN_CNNBUS_CTRL_RM_0	 */
#define   X2_CNN_RD_MAXLEN_M(n)			(((n) & 0xff) << 0x8)
#define   X2_CNN_RD_MAXLEN_M_MASK		(0xff << 0x8)
#define   X2_CNN_RD_MAXLEN_M_SHIT(n)		(((n) & 0xff) >> 0x8)
#define   X2_CNN_RD_ENDIAN_M(n)			(((n) & 0xf) << 0x4)
#define   X2_CNN_RD_ENDIAN_M_MASK		(0xf << 0x4)
#define   X2_CNN_RD_ENDIAN_M_SHIT(n)		(((n) & 0xf) >> 0x4)
#define   X2_CNN_RD_PRIORITY_M(n)		(((n) & 0xf) << 0x0)
#define   X2_CNN_RD_PRIORITY_M_MASK		(0xf << 0x0)
#define   X2_CNN_RD_PRIORITY_M_SHIT(n)		(((n) & 0xf) >> 0x0)

/*    X2_CNN_CNNBUS_AXIID    */
#define   X2_CNN_AXIID(n)			(((n) & 0xf) << 0x0)
#define   X2_CNN_AXIID_MASK			(0xf << 0x0)
#define   X2_CNN_AXIID_SHIT(n)			(((n) & 0xf) >> 0x0)

/*    X2_CNN_CNNINT_MASK    */
#define   X2_CNN_PE0_INT_MASK(n)		(((n) & 0x1) << 0x0)
#define   X2_CNN_PE0_INT_MASK_MASK		(0x1 << 0x0)
#define   X2_CNN_PE0_INT_MASK_SHIT(n)		(((n) & 0x1) >> 0x0)

/*    X2_CNN_CNNINT_STATUS    */
#define   X2_CNN_PE0_INT_STATUS_RO		(0x1 << 0x0)
#define   X2_CNN_PE0_INT_STATUS_RO_SHIT(n)	(((n) & 0x1) >> 0x0)

/*    X2_CNN_CNNINT_NUM    */
#define   X2_CNN_INT_NUM_RO			(0xffff << 0x0)
#define   X2_CNN_INT_NUM_RO_SHIT(n)		(((n) & 0xffff) >> 0x0)

/*    X2_CNN_CNNINT_ERR_NUM    */
#define   X2_CNN_INT_ERR_NUM_RO			(0xffff << 0x0)
#define   X2_CNN_INT_ERR_NUM_RO_SHIT(n)		(((n) & 0xffff) >> 0x0)

/*    X2_CNN_CNNINT_FC_BASE    */
#define   X2_CNN_PE0_FC_BASE(n)			(((n) & 0xffffffff) << 0x0)
#define   X2_CNN_PE0_FC_BASE_MASK		(0xffffffff << 0x0)
#define   X2_CNN_PE0_FC_BASE_SHIT(n)		(((n) & 0xffffffff) >> 0x0)

/*    X2_CNN_CNNINT_FC_HEAD    */
#define   X2_CNN_PE0_FC_HEAD_RO			(0x7ff << 0x0)
#define   X2_CNN_PE0_FC_HEAD_RO_SHIT(n)		(((n) & 0x7ff) >> 0x0)

/*    X2_CNN_CNNINT_FC_TAIL    */
#define   X2_CNN_PE0_FC_TAIL(n)			(((n) & 0x7ff) << 0x0)
#define   X2_CNN_PE0_FC_TAIL_MASK		(0x7ff << 0x0)
#define   X2_CNN_PE0_FC_TAIL_SHIT(n)		(((n) & 0x7ff) >> 0x0)

/*    X2_CNN_CNNINT_FC_LEN    */
#define   X2_CNN_PE0_FC_LENGTH(n)		(((n) & 0x3ff) << 0x0)
#define   X2_CNN_PE0_FC_LENGTH_MASK		(0x3ff << 0x0)
#define   X2_CNN_PE0_FC_LENGTH_SHIT(n)		(((n) & 0x3ff) >> 0x0)

/*    X2_CNN_CNNINT_INST_NUM	*/
#define   X2_CNN_INST_NUMBER_RO			(0xffffffff << 0x0)
#define   X2_CNN_INST_NUMBER_RO_SHIT(n)		(((n) & 0xffffffff) >> 0x0)

#define X2_CNN_MAX_FC_LEN_MASK	0x3ff
#define X2_CNN_FC_IDX_FLAG		0x400
#define X2_CNN_FC_SIZE			0x40

#define CNN_FC_GAP_LEN			0x1000
#define CNN_FC_SPACE_LEN		(0x400 * 0x40)

#define BPU_PMU_REG				0xa6000010
#define BPU_ISO_BIT(index)		(0x1 << (((index) << 2) + 1))

#define BPU_MAX_CORE_NUM		2
#define BPU_ERR_IRQ_MASK		0xF000
#define BPU_IRQ_MASK			0x0FFF

extern struct bpu_core_hw_ops x2_hw_ops;

#endif
