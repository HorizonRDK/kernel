/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_VPU_REG_H__
#define __HOBOT_VPU_REG_H__

#include <linux/io.h>
#include "hobot_vpu_utils.h"

#define VPU_READL(addr)			(readl(dev->regs_base	\
								+ dev->bit_fm_info[core].reg_base_offset	\
								+ (addr)))
#define VPU_WRITEL(addr, val)	(writel((val), dev->regs_base	\
								+ dev->bit_fm_info[core].reg_base_offset	\
								+ (addr)))

/* if the platform driver knows this driver */
/* the definition of VPU_REG_BASE_ADDR and VPU_REG_SIZE are not meaningful */

#define VPU_REG_BASE_ADDR			0xA8000000
#define VPU_REG_SIZE				(0x4000*MAX_NUM_VPU_CORE)

#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
#define VPU_BUSY_CHECK_TIMEOUT          10000
#define VPU_DEC_TIMEOUT                 60000

// BIT_RUN command
enum {
	DEC_SEQ_INIT = 1,
	ENC_SEQ_INIT = 1,
	DEC_SEQ_END = 2,
	ENC_SEQ_END = 2,
	PIC_RUN = 3,
	SET_FRAME_BUF = 4,
	ENCODE_HEADER = 5,
	ENC_PARA_SET = 6,
	DEC_PARA_SET = 7,
	DEC_BUF_FLUSH = 8,
	RC_CHANGE_PARAMETER    = 9,
	VPU_SLEEP = 10,
	VPU_WAKE = 11,
	ENC_ROI_INIT = 12,
	FIRMWARE_GET = 0xf
};
/**
* @brief This is an enumeration for declaring return codes from API function calls.
The meaning of each return code is the same for all of the API functions, but 
the reasons of non-successful return might be different. Some details of those
reasons are briefly described in the API definition chapter. In this chapter, the basic 
meaning of each return code is presented.
*/
typedef enum {
    RETCODE_SUCCESS,                    /**< This means that operation was done successfully.  */  /* 0  */
    RETCODE_FAILURE,                    /**< This means that operation was not done successfully. When un-recoverable decoder error happens such as header parsing errors, this value is returned from VPU API.  */
    RETCODE_INVALID_HANDLE,             /**< This means that the given handle for the current API function call was invalid (for example, not initialized yet, improper function call for the given handle, etc.).  */
    RETCODE_INVALID_PARAM,              /**< This means that the given argument parameters (for example, input data structure) was invalid (not initialized yet or not valid anymore). */
    RETCODE_INVALID_COMMAND,            /**< This means that the given command was invalid (for example, undefined, or not allowed in the given instances).  */
    RETCODE_ROTATOR_OUTPUT_NOT_SET,     /**< This means that rotator output buffer was not allocated even though postprocessor (rotation, mirroring, or deringing) is enabled. */  /* 5  */
    RETCODE_ROTATOR_STRIDE_NOT_SET,     /**< This means that rotator stride was not provided even though postprocessor (rotation, mirroring, or deringing) is enabled.  */
    RETCODE_FRAME_NOT_COMPLETE,         /**< This means that frame decoding operation was not completed yet, so the given API function call cannot be allowed.  */
    RETCODE_INVALID_FRAME_BUFFER,       /**< This means that the given source frame buffer pointers were invalid in encoder (not initialized yet or not valid anymore).  */
    RETCODE_INSUFFICIENT_FRAME_BUFFERS, /**< This means that the given numbers of frame buffers were not enough for the operations of the given handle. This return code is only received when calling VPU_DecRegisterFrameBuffer() or VPU_EncRegisterFrameBuffer() function. */
    RETCODE_INVALID_STRIDE,             /**< This means that the given stride was invalid (for example, 0, not a multiple of 8 or smaller than picture size). This return code is only allowed in API functions which set stride.  */   /* 10 */
    RETCODE_WRONG_CALL_SEQUENCE,        /**< This means that the current API function call was invalid considering the allowed sequences between API functions (for example, missing one crucial function call before this function call).  */
    RETCODE_CALLED_BEFORE,              /**< This means that multiple calls of the current API function for a given instance are invalid. */
    RETCODE_NOT_INITIALIZED,            /**< This means that VPU was not initialized yet. Before calling any API functions, the initialization API function, VPU_Init(), should be called at the beginning.  */
    RETCODE_USERDATA_BUF_NOT_SET,       /**< This means that there is no memory allocation for reporting userdata. Before setting user data enable, user data buffer address and size should be set with valid value. */
    RETCODE_MEMORY_ACCESS_VIOLATION,    /**< This means that access violation to the protected memory has been occurred. */   /* 15 */
    RETCODE_VPU_RESPONSE_TIMEOUT,       /**< This means that VPU response time is too long, time out. */
    RETCODE_INSUFFICIENT_RESOURCE,      /**< This means that VPU cannot allocate memory due to lack of memory. */
    RETCODE_NOT_FOUND_BITCODE_PATH,     /**< This means that BIT_CODE_FILE_PATH has a wrong firmware path or firmware size is 0 when calling VPU_InitWithBitcode() function.  */
    RETCODE_NOT_SUPPORTED_FEATURE,      /**< This means that HOST application uses an API option that is not supported in current hardware.  */
    RETCODE_NOT_FOUND_VPU_DEVICE,       /**< This means that HOST application uses the undefined product ID. */   /* 20 */
    RETCODE_CP0_EXCEPTION,              /**< This means that coprocessor exception has occurred. (WAVE only) */
    RETCODE_STREAM_BUF_FULL,            /**< This means that stream buffer is full in encoder. */
    RETCODE_ACCESS_VIOLATION_HW,        /**< This means that GDI access error has occurred. It might come from violation of write protection region or spec-out GDI read/write request. (WAVE only) */
    RETCODE_QUERY_FAILURE,              /**< This means that query command was not successful. (WAVE5 only) */
    RETCODE_QUEUEING_FAILURE,           /**< This means that commands cannot be queued. (WAVE5 only) */
    RETCODE_VPU_STILL_RUNNING,          /**< This means that VPU cannot be flushed or closed now, because VPU is running. (WAVE5 only) */
    RETCODE_REPORT_NOT_READY,           /**< This means that report is not ready for Query(GET_RESULT) command. (WAVE5 only) */
    RETCODE_VLC_BUF_FULL,               /**< This means that VLC buffer is full in encoder. (WAVE5 only) */
    RETCODE_INVALID_SFS_INSTANCE,       /**< This means that current instance can't run sub-framesync. (already an instance was running with sub-frame sync (WAVE5 only) */
#ifdef AUTO_FRM_SKIP_DROP
    RETCODE_FRAME_DROP,                 /**< This means that frame is dropped. HOST application don't have to wait INT_BIT_PIC_RUN.  (CODA9 only) */
#endif
} RetCode;

typedef enum {
	INT_WAVE5_INIT_VPU          = 0,
	INT_WAVE5_WAKEUP_VPU        = 1,
	INT_WAVE5_SLEEP_VPU         = 2,
	INT_WAVE5_CREATE_INSTANCE   = 3,
	INT_WAVE5_FLUSH_INSTANCE    = 4,
	INT_WAVE5_DESTORY_INSTANCE  = 5,
	INT_WAVE5_INIT_SEQ          = 6,
	INT_WAVE5_SET_FRAMEBUF      = 7,
	INT_WAVE5_DEC_PIC           = 8,
	INT_WAVE5_ENC_PIC           = 8,
	INT_WAVE5_ENC_SET_PARAM     = 9,
	INT_WAVE5_DEC_QUERY         = 14,
	INT_WAVE5_BSBUF_EMPTY       = 15,
	INT_WAVE5_BSBUF_FULL        = 15,
} Wave5InterruptBit;

typedef enum {
	W5_INIT_VPU 	   = 0x0001,
	W5_WAKEUP_VPU	   = 0x0002,
	W5_SLEEP_VPU	   = 0x0004,
	W5_CREATE_INSTANCE = 0x0008,			/* queuing command */
	W5_FLUSH_INSTANCE  = 0x0010,
	W5_DESTROY_INSTANCE= 0x0020,			/* queuing command */
	W5_INIT_SEQ 	   = 0x0040,			/* queuing command */
	W5_SET_FB		   = 0x0080,
	W5_DEC_PIC		   = 0x0100,			/* queuing command */
	W5_ENC_PIC		   = 0x0100,			/* queuing command */
	W5_ENC_SET_PARAM   = 0x0200,			/* queuing command */
	W5_QUERY		   = 0x4000,
	W5_UPDATE_BS	   = 0x8000,
	W5_RESET_VPU	   = 0x10000,
	W5_MAX_VPU_COMD = 0x10000,
} W5_VPU_COMMAND;

#else

typedef enum {
	W4_INT_INIT_VPU			= 0,
	W4_INT_DEC_PIC_HDR		= 1,
	W4_INT_FINI_SEQ			= 2,
	W4_INT_DEC_PIC			= 3,
	W4_INT_SET_FRAMEBUF		= 4,
	W4_INT_FLUSH_DEC		= 5,
	W4_INT_GET_FW_VERSION	= 9,
	W4_INT_QUERY_DEC		= 10,
	W4_INT_SLEEP_VPU		= 11,
	W4_INT_WAKEUP_VPU		= 12,
	W4_INT_CHANGE_INT		= 13,
	W4_INT_CREATE_INSTANCE  = 14,
	W4_INT_BSBUF_EMPTY	    = 15,   /*!<< Bitstream buffer empty */
	W4_INT_ENC_SLICE_INT    = 15,
} Wave4InterruptBit;

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
#define W5_VPU_INT_REASON_CLEAR		(W5_REG_BASE + 0x0034)
#define W5_VPU_VINT_CLEAR			(W5_REG_BASE + 0x003C)
#define W5_VPU_VPU_INT_STS			(W5_REG_BASE + 0x0044)
#define W5_VPU_INT_REASON 			(W5_REG_BASE + 0x004c)
#define W5_RET_FAIL_REASON			(W5_REG_BASE + 0x010C)

#ifdef SUPPORT_MULTI_INST_INTR
#define W5_RET_BS_EMPTY_INST			(W5_REG_BASE + 0x01E4)
#define W5_RET_QUEUE_CMD_DONE_INST		(W5_REG_BASE + 0x01E8)
#define W5_RET_SEQ_DONE_INSTANCE_INFO		(W5_REG_BASE + 0x01FC)
#endif
/* WAVE5 INIT, WAKEUP */
#define W5_PO_CONF				(W5_REG_BASE + 0x0000)
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
#define W5_QUERY_OPTION			(W5_REG_BASE + 0x0104)

#define W5_VPU_FIO_DATA			(W5_REG_BASE + 0x0024)
#define W5_VPU_FIO_CTRL_ADDR           (W5_REG_BASE + 0x0020)
#define W5_VCPU_CUR_PC                 (W5_REG_BASE + 0x0004)
#define W5_CMD_REG_END                  0x00000200
#define WAVE5_MAX_CODE_BUF_SIZE         (1024*1024)
#define WAVE5_UPPER_PROC_AXI_ID     0x0
#define W5_VPU_RET_VPU_CONFIG0                  (W5_REG_BASE + 0x0098)
#define W5_VPU_RET_VPU_CONFIG1                  (W5_REG_BASE + 0x009C)
#define WAVE5_PROC_AXI_ID           0x0
#define WAVE5_PRP_AXI_ID            0x0
#define WAVE5_FBD_Y_AXI_ID          0x0
#define WAVE5_FBC_Y_AXI_ID          0x0
#define WAVE5_FBD_C_AXI_ID          0x0
#define WAVE5_FBC_C_AXI_ID          0x0
#define WAVE5_SEC_AXI_ID            0x0
#define WAVE5_PRI_AXI_ID            0x0
/************************************************************************/
/* GDI register for Debugging                                           */
/************************************************************************/
#define W5_GDI_BASE                         0x8800
#define W5_GDI_BUS_CTRL                     (W5_GDI_BASE + 0x0F0)
#define W5_GDI_BUS_STATUS                   (W5_GDI_BASE + 0x0F4)

#define W5_BACKBONE_BASE_VCPU               0xFE00
#define W5_BACKBONE_BUS_CTRL_VCPU           (W5_BACKBONE_BASE_VCPU + 0x010)
#define W5_BACKBONE_BUS_STATUS_VCPU         (W5_BACKBONE_BASE_VCPU + 0x014)
#define W5_BACKBONE_PROG_AXI_ID             (W5_BACKBONE_BASE_VCPU + 0x00C)

#define W5_BACKBONE_BASE_VCORE0             0x8E00
#define W5_BACKBONE_BUS_CTRL_VCORE0         (W5_BACKBONE_BASE_VCORE0 + 0x010)
#define W5_BACKBONE_BUS_STATUS_VCORE0       (W5_BACKBONE_BASE_VCORE0 + 0x014)

#define W5_BACKBONE_BASE_VCORE1             0x9E00  // for dual-core product
#define W5_BACKBONE_BUS_CTRL_VCORE1         (W5_BACKBONE_BASE_VCORE1 + 0x010)
#define W5_BACKBONE_BUS_STATUS_VCORE1       (W5_BACKBONE_BASE_VCORE1 + 0x014)

#define W5_COMBINED_BACKBONE_BASE           0xFE00
#define W5_COMBINED_BACKBONE_BUS_CTRL       (W5_COMBINED_BACKBONE_BASE + 0x010)
#define W5_COMBINED_BACKBONE_BUS_STATUS     (W5_COMBINED_BACKBONE_BASE + 0x014)
/************************************************************************/
/* DECODER - QUERY : UPDATE_DISP_FLAG                                   */
/************************************************************************/
#define W5_CMD_DEC_SET_DISP_IDC             (W5_REG_BASE + 0x0118)
#define W5_CMD_DEC_CLR_DISP_IDC             (W5_REG_BASE + 0x011C)
/************************************************************************/
/* DECODER - QUERY : GET_RESULT                                         */
/************************************************************************/
#define W5_CMD_DEC_ADDR_REPORT_BASE         (W5_REG_BASE + 0x0114)
#define W5_CMD_DEC_REPORT_SIZE              (W5_REG_BASE + 0x0118)
#define W5_CMD_DEC_REPORT_PARAM             (W5_REG_BASE + 0x011C)

#define W5_RET_DEC_DECODING_SUCCESS         (W5_REG_BASE + 0x01DC)
#ifdef SUPPORT_SW_UART
#define W5_SW_UART_STATUS					(W5_REG_BASE + 0x01D4)
#define W5_SW_UART_TX_DATA					(W5_REG_BASE + 0x01D8)
//#define W5_RET_DEC_WARN_INFO                (W5_REG_BASE + 0x01D4)
//#define W5_RET_DEC_ERR_INFO                 (W5_REG_BASE + 0x01D8)
#else
#define W5_RET_DEC_WARN_INFO                (W5_REG_BASE + 0x01D4)
#define W5_RET_DEC_ERR_INFO                 (W5_REG_BASE + 0x01D8)
#endif
#define W5_BS_OPTION                            (W5_REG_BASE + 0x0120)
#define WAVE5_SYSERR_VPU_STILL_RUNNING          0x00001000

typedef enum {
    GET_VPU_INFO        = 0,
    SET_WRITE_PROT      = 1,
    GET_RESULT          = 2,
    UPDATE_DISP_FLAG    = 3,
    GET_BW_REPORT       = 4,
    GET_BS_RD_PTR       = 5,    // for decoder
    GET_BS_WR_PTR       = 6,    // for encoder
    GET_SRC_BUF_FLAG    = 7,    // for encoder
    SET_BS_RD_PTR       = 8,    // for decoder
    GET_DEBUG_INFO      = 0x61,
} QUERY_OPT;

/* Product register */
#define VPU_PRODUCT_CODE_REGISTER		(W5_REG_BASE + 0x1044)

#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
#else
#define W5_MAX_CODE_BUF_SIZE	(2*1024*1024)
#define W5_CMD_INIT_VPU				(0x0001)
#define W5_CMD_SLEEP_VPU			(0x0004)
#define W5_CMD_WAKEUP_VPU			(0x0002)
#define W5_DESTROY_INSTANCE			(0x0020)
#define W5_QUERY				(0x4000)

#define VPU_ISSUE_COMMAND(core, cmd) \
			do {	\
				VPU_WRITEL(W5_VPU_BUSY_STATUS, 1);	\
				VPU_WRITEL(W5_COMMAND, cmd);	\
				VPU_WRITEL(W5_VPU_HOST_INT_REQ, 1);	\
			} while (0)
#endif

#endif /*__HOBOT_VPU_REG_H__*/
