/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/debugfs.h>

#include "hobot_vpu_ctl.h"
#include "hobot_vpu_debug.h"
#include "hobot_vpu_pm.h"
#include "hobot_vpu_reg.h"
#include "hobot_vpu_utils.h"

int vpu_debug_flag = 5;
int vpu_debug_info = 0;
int vpu_pf_bw_debug = 0;
unsigned long vpu_clk_freq = MAX_VPU_FREQ;

#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
#define VPU_INIT_VIDEO_MEMORY_SIZE_IN_BYTE (62*1024*1024)
#define VPU_DRAM_PHYSICAL_BASE 0x60000000	// 0x86C00000
static video_mm_t s_vmem = { 0 };
static hb_vpu_drv_buffer_t s_video_memory = { 0 };
#endif

/* this definition is only for chipsnmedia FPGA board env */
/* so for SOC env of customers can be ignored */

/*for kernel up to 3.7.0 version*/
#ifndef VM_RESERVED
#define VM_RESERVED   (VM_DONTEXPAND | VM_DONTDUMP)
#endif

DECLARE_BITMAP(vpu_inst_bitmap, MAX_NUM_VPU_INSTANCE);

module_param(vpu_clk_freq, ulong, 0644);
module_param(vpu_debug_info, int, 0644);
module_param(vpu_pf_bw_debug, int, 0644);


#ifdef USE_MUTEX_IN_KERNEL_SPACE
static int vdi_lock_base(hb_vpu_dev_t *dev, unsigned int type,
			int fromUserspace)
{
	int ret;
	// DPRINTK("[VPUDRV]+%s type=%d, pid=%d, tgid=%d, signal_pending_state=%d \n",
	//__FUNCTION__, type, current->pid, current->tgid,
	//signal_pending_state(TASK_INTERRUPTIBLE, current));
	if (type == VPUDRV_MUTEX_VPU) {
		ret = down_interruptible(&dev->vpu_vdi_sem);
		if (fromUserspace == 0) {
			if (ret == -EINTR) {
				ret = down_killable(&dev->vpu_vdi_sem);
			}
		}
	} else if (type == VPUDRV_MUTEX_DISP_FALG) {
		ret = down_interruptible(&dev->vpu_vdi_disp_sem);
		if (fromUserspace == 0) {
			if (ret == -EINTR) {
				ret = down_killable(&dev->vpu_vdi_disp_sem);
			}
		}
	} else if (type == VPUDRV_MUTEX_RESET) {
		ret = down_interruptible(&dev->vpu_vdi_reset_sem);
		if (fromUserspace == 0) {
			if (ret == -EINTR) {
				ret = down_killable(&dev->vpu_vdi_reset_sem);
			}
		}
	} else if (type == VPUDRV_MUTEX_VMEM) {
		ret = down_interruptible(&dev->vpu_vdi_vmem_sem);
		if (fromUserspace == 0) {
			if (ret == -EINTR) {
				ret = down_killable(&dev->vpu_vdi_vmem_sem);
			}
		}
	} else {
		vpu_err("unknown MUTEX_TYPE type=%d\n", type);
		return 0;
	}
	if (ret == 0) {
		dev->current_vdi_lock_pid[type] = current->tgid;
	} else {
		vpu_err("down_interruptible error ret=%d\n", ret);
	}

	// DPRINTK("[VPUDRV]+%s ret=%d \n", __FUNCTION__, ret);
	return ret;
}

static int vdi_lock_user(hb_vpu_dev_t *dev, unsigned int type)
{
	return vdi_lock_base(dev, type, 1);
}

static int vdi_lock(hb_vpu_dev_t *dev, unsigned int type)
{
	return vdi_lock_base(dev, type, 0);
}

static void vdi_unlock(hb_vpu_dev_t *dev, unsigned int type)
{
	// DPRINTK("[VPUDRV]+%s, type=%d\n", __FUNCTION__, type);
	if (type >= VPUDRV_MUTEX_VPU && type < VPUDRV_MUTEX_MAX) {
		dev->current_vdi_lock_pid[type] = 0;
	} else {
		vpu_err("unknown MUTEX_TYPE type=%d\n", type);
		return;
	}
	if (type == VPUDRV_MUTEX_VPU) {
		up(&dev->vpu_vdi_sem);
	} else if (type == VPUDRV_MUTEX_DISP_FALG) {
		up(&dev->vpu_vdi_disp_sem);
	} else if (type == VPUDRV_MUTEX_RESET) {
		up(&dev->vpu_vdi_reset_sem);
	} else if (type == VPUDRV_MUTEX_VMEM) {
		up(&dev->vpu_vdi_vmem_sem);
	}
	// DPRINTK("[VPUDRV]-%s\n", __FUNCTION__);
}
#endif

#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
static hb_vpu_instance_pool_t *get_instance_pool_handle(
		hb_vpu_dev_t *dev, u32 core)
{
	int instance_pool_size_per_core;
	void *vip_base;

	if (core > MAX_NUM_VPU_CORE)
		return NULL;

	/* instance_pool.size  assigned to the size of all core once call
	VDI_IOCTL_GET_INSTANCE_POOL by user. */
	instance_pool_size_per_core = (dev->instance_pool.size/MAX_NUM_VPU_CORE);
	vip_base = (void *)(dev->instance_pool.base +
			(instance_pool_size_per_core * core));

	return (hb_vpu_instance_pool_t *)vip_base;
}
#define FIO_TIMEOUT         100

static void WriteVpuFIORegister(hb_vpu_dev_t *dev, u32 core,
				u32 addr, u32 data)
{
	unsigned int ctrl;
	unsigned int count = 0;
	if (!dev) {
		return;
	}
	VPU_WRITEL(W5_VPU_FIO_DATA, data);
	ctrl  = (addr&0xffff);
	ctrl |= (1<<16);    /* write operation */
	VPU_WRITEL(W5_VPU_FIO_CTRL_ADDR, ctrl);

	count = FIO_TIMEOUT;
	while (count--) {
		ctrl = VPU_READL(W5_VPU_FIO_CTRL_ADDR);
		if (ctrl & 0x80000000) {
			break;
		}
	}
}
static u32 ReadVpuFIORegister(hb_vpu_dev_t *dev, u32 core, u32 addr)
{
	u32 ctrl;
	u32 count = 0;
	u32 data  = 0xffffffff;

	ctrl  = (addr&0xffff);
	ctrl |= (0<<16);    /* read operation */
	VPU_WRITEL(W5_VPU_FIO_CTRL_ADDR, ctrl);
	count = FIO_TIMEOUT;
	while (count--) {
		ctrl = VPU_READL(W5_VPU_FIO_CTRL_ADDR);
		if (ctrl & 0x80000000) {
			data = VPU_READL(W5_VPU_FIO_DATA);
			break;
		}
	}

	return data;
}

static int vpuapi_wait_reset_busy(hb_vpu_dev_t *dev, u32 core)
{
	int ret;
	u32 val;
	int product_code;
	unsigned long timeout = jiffies + VPU_BUSY_CHECK_TIMEOUT;

	product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
	while (1) {
		if (PRODUCT_CODE_W_SERIES(product_code)) {
			val = VPU_READL(W5_VPU_RESET_STATUS);
		} else {
			return -1;
		}
		if (val == 0) {
			ret = VPUAPI_RET_SUCCESS;
			break;
		}

		if (time_after(jiffies, timeout)) {
			vpu_err("vpuapi_wait_reset_busy after BUSY timeout");
			ret = VPUAPI_RET_TIMEOUT;
			break;
		}
		udelay(0);	// delay more to give idle time to OS;
	}

	return ret;
}

static int vpuapi_wait_vpu_busy(hb_vpu_dev_t *dev, u32 core)
{
	int ret;
	u32 val = 0;
	u32 cmd;
	u32 pc;
	int product_code;
	unsigned long timeout = jiffies + VPU_BUSY_CHECK_TIMEOUT;

	product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
	while(1) {
		if (PRODUCT_CODE_W_SERIES(product_code)) {
			val = VPU_READL(W5_VPU_BUSY_STATUS);
			cmd = VPU_READL(W5_COMMAND);
			pc = VPU_READL(W5_VCPU_CUR_PC);
		} else {
			return -1;
		}
		if (val == 0) {
			ret = VPUAPI_RET_SUCCESS;
			break;
		}

		if (time_after(jiffies, timeout)) {
			vpu_err("timeout cmd=0x%x, pc=0x%x\n", cmd, pc);
			ret = VPUAPI_RET_TIMEOUT;
			break;
		}
		//vpu_debug(5, "%s cmd=0x%x, pc=0x%x\n", __FUNCTION__, cmd, pc);
		udelay(0);	// delay more to give idle time to OS;
	}

	return ret;
}

static int vpuapi_wait_bus_busy(hb_vpu_dev_t *dev, u32 core,
				u32 bus_busy_reg_addr)
{
	int ret;
	u32 val;
	int product_code;
	unsigned long timeout = jiffies + VPU_BUSY_CHECK_TIMEOUT;

	product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
	ret = VPUAPI_RET_SUCCESS;
	while (1) {
		if (PRODUCT_CODE_W_SERIES(product_code)) {
			val = ReadVpuFIORegister(dev, core, bus_busy_reg_addr);
			if (val == 0x3f)
				break;
		} else {
			return -1;
		}

		if (time_after(jiffies, timeout)) {
			vpu_err("timeout \n");
			ret = VPUAPI_RET_TIMEOUT;
			break;
		}
		udelay(0);	// delay more to give idle time to OS;
	}

	return ret;
}

// PARAMETER
/// mode 0 => wake
/// mode 1 => sleep
// return
static int wave_sleep_wake(hb_vpu_dev_t *dev, u32 core, int mode)
{
	u32 val;
	vpu_debug(5, "%s core=%d, mode=%d\n", __FUNCTION__, core, mode);
	if (mode == VPU_SLEEP_MODE) {
		if (vpuapi_wait_vpu_busy(dev, core) == VPUAPI_RET_TIMEOUT) {
			return VPUAPI_RET_TIMEOUT;
		}

		VPU_WRITEL(W5_VPU_BUSY_STATUS, 1);
		VPU_WRITEL(W5_COMMAND, W5_SLEEP_VPU);
		VPU_WRITEL(W5_VPU_HOST_INT_REQ, 1);

		if (vpuapi_wait_vpu_busy(dev, core) == VPUAPI_RET_TIMEOUT) {
			return VPUAPI_RET_TIMEOUT;
		}

		if (VPU_READL(W5_RET_SUCCESS) == 0) {
			val = VPU_READL(W5_RET_FAIL_REASON);
			if (val == RETCODE_VPU_STILL_RUNNING) {
				return VPUAPI_RET_STILL_RUNNING;
			} else {
				return VPUAPI_RET_FAILURE;
			}
		}
	} else {
		int i;
		u32 val;
		u32 remapSize;
		u32 codeBase;
		u32 codeSize;

		VPU_WRITEL(W5_PO_CONF, 0);
		for (i = W5_CMD_REG_END; i < W5_CMD_REG_END; i++) {
#if defined(SUPPORT_SW_UART) || defined(SUPPORT_SW_UART_V2)
			if (i == W5_SW_UART_STATUS)
				continue;
#endif

			if (i == W5_RET_BS_EMPTY_INST || i == W5_RET_QUEUE_CMD_DONE_INST
				|| i == W5_RET_SEQ_DONE_INSTANCE_INFO)
				continue;

			VPU_WRITEL(i, 0);
		}
		codeBase = dev->common_memory.phys_addr;
		codeSize = (WAVE5_MAX_CODE_BUF_SIZE&~0xfff);
		remapSize = (codeSize >> 12) & 0x1ff;

		val = 0x80000000 | (WAVE5_UPPER_PROC_AXI_ID<<20) |
			(W5_REMAP_CODE_INDEX << 12) | (0 << 16) | (1<<11) | remapSize;

		VPU_WRITEL(W5_VPU_REMAP_CTRL,     val);
		VPU_WRITEL(W5_VPU_REMAP_VADDR,    0x00000000);    /* DO NOT CHANGE! */
		VPU_WRITEL(W5_VPU_REMAP_PADDR,    codeBase);
		VPU_WRITEL(W5_ADDR_CODE_BASE,     codeBase);
		VPU_WRITEL(W5_CODE_SIZE,          codeSize);
		VPU_WRITEL(W5_CODE_PARAM,         (WAVE5_UPPER_PROC_AXI_ID << 4) | 0);
		VPU_WRITEL(W5_HW_OPTION, 0);

		// encoder
		val  = (1<<INT_WAVE5_ENC_SET_PARAM);
		val |= (1<<INT_WAVE5_ENC_PIC);
		val |= (1<<INT_WAVE5_BSBUF_FULL);
#ifdef SUPPORT_SOURCE_RELEASE_INTERRUPT
		val |= (1<<INT_WAVE5_ENC_SRC_RELEASE);
#endif
		// decoder
		val |= (1<<INT_WAVE5_INIT_SEQ);
		val |= (1<<INT_WAVE5_DEC_PIC);
		val |= (1<<INT_WAVE5_BSBUF_EMPTY);
		VPU_WRITEL(W5_VPU_VINT_ENABLE,  val);

		val = VPU_READL(W5_VPU_RET_VPU_CONFIG0);
		if (((val>>16)&1) == 1) {
			val = ((WAVE5_PROC_AXI_ID<<28)  |
					(WAVE5_PRP_AXI_ID<<24)   |
					(WAVE5_FBD_Y_AXI_ID<<20) |
					(WAVE5_FBC_Y_AXI_ID<<16) |
					(WAVE5_FBD_C_AXI_ID<<12) |
					(WAVE5_FBC_C_AXI_ID<<8)  |
					(WAVE5_PRI_AXI_ID<<4)    |
					(WAVE5_SEC_AXI_ID<<0));
			WriteVpuFIORegister(dev, core, W5_BACKBONE_PROG_AXI_ID, val);
		}

		VPU_WRITEL(W5_VPU_BUSY_STATUS, 1);
		VPU_WRITEL(W5_COMMAND, W5_WAKEUP_VPU);
		VPU_WRITEL(W5_VPU_REMAP_CORE_START, 1);

		if (vpuapi_wait_vpu_busy(dev, core) == VPUAPI_RET_TIMEOUT) {
			vpu_err("%s timeout pc=0x%x\n", __FUNCTION__,
				VPU_READL(W5_VCPU_CUR_PC));
			return VPUAPI_RET_TIMEOUT;
		}

		val = VPU_READL(W5_RET_SUCCESS);
		if (val == 0) {
			vpu_err("%s  VPUAPI_RET_FAILURE pc=0x%x \n", __FUNCTION__,
				VPU_READL(W5_VCPU_CUR_PC));
			return VPUAPI_RET_FAILURE;
		}
	}
	vpu_debug(5, "-%s core=%d, mode=%d\n", __FUNCTION__, core, mode);
	return VPUAPI_RET_SUCCESS;
}

static int wave_close_instance(hb_vpu_dev_t *dev, u32 core, u32 inst)
{
	int ret;
	u32 error_reason = 0;
	unsigned long timeout = jiffies + VPU_DEC_TIMEOUT;

	vpu_debug(5, "[+] core=%d, inst=%d\n", core, inst);
	if (vpu_check_is_decoder(dev, core, inst) == 1) {
		ret = vpuapi_dec_set_stream_end(dev, core, inst);
		ret = vpuapi_dec_clr_all_disp_flag(dev, core, inst);
	}
	while ((ret = vpuapi_close(dev, core, inst)) == VPUAPI_RET_STILL_RUNNING) {
		ret = vpuapi_get_output_info(dev, core, inst, &error_reason);
		vpu_debug(5, "core=%d, inst=%d, ret=%d, error_reason=0x%x\n",
			core, inst,  ret, error_reason);
		if (ret == VPUAPI_RET_SUCCESS) {
			if ((error_reason & 0xf0000000)) {
				if (vpu_do_sw_reset(dev, core, inst, error_reason)
					== VPUAPI_RET_TIMEOUT) {
					break;
				}
			}
		}

		if (vpu_check_is_decoder(dev, core, inst) == 1) {
			ret = vpuapi_dec_set_stream_end(dev, core, inst);
			ret = vpuapi_dec_clr_all_disp_flag(dev, core, inst);
		}

		mdelay(10);	// delay for vpuapi_close

		if (time_after(jiffies, timeout)) {
			vpu_err("vpuapi_close flow timeout ret=%d, inst=%d\n", ret, inst);
			vpu_debug(5, "[-] ret=%d\n", ret);
			return 0;
		}
	}

	vpu_debug(5, "[-] ret=%d\n", ret);
	return 1;
}

int vpu_check_is_decoder(hb_vpu_dev_t *dev, u32 core, u32 inst)
{
	u32 is_decoder;
	unsigned char *codec_inst;
	hb_vpu_instance_pool_t *vip = get_instance_pool_handle(dev, core);

	if (vip == NULL) {
		return 0;
	}

	codec_inst = &vip->codec_inst_pool[inst][0];
	// indicates isDecoder in CodecInst structure in vpuapifunc.h
	codec_inst = codec_inst + (sizeof(u32) * 7);
	memcpy(&is_decoder, codec_inst, 4);

	vpu_debug(5, "%s is_decoder=0x%x\n", __FUNCTION__, is_decoder);
	return (is_decoder == 1)?1:0;
}

int vpu_close_instance(hb_vpu_dev_t *dev, u32 core, u32 inst)
{
	u32 product_code;
	int success;
	product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
	vpu_debug(5, "[+] core=%d, inst=%d, product_code=0x%x\n",
		core, inst, product_code);
	if (PRODUCT_CODE_W_SERIES(product_code)) {
		success = wave_close_instance(dev, core, inst);
	} else {
		vpu_err("vpu_close_instance Unknown product id : %08x\n", product_code);
		success = 0;
	}
	vpu_debug(5, "[-] success=%d\n", success);
	return success;
}

#if defined(CONFIG_PM)
int vpu_sleep_wake(hb_vpu_dev_t *dev, u32 core, int mode)
{
	int inst;
	int ret;
	int product_code;
	u32 error_reason = VPUAPI_RET_SUCCESS;
	unsigned long timeout = jiffies + VPU_DEC_TIMEOUT;
	product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
	if (PRODUCT_CODE_W_SERIES(product_code)) {
		if (mode == VPU_SLEEP_MODE) {
			while((ret = wave_sleep_wake(dev, core, VPU_SLEEP_MODE)) ==
				VPUAPI_RET_STILL_RUNNING) {
				for (inst = 0; inst < MAX_NUM_VPU_INSTANCE; inst++) {
					ret = vpuapi_get_output_info(dev, core, inst, &error_reason);
					if (ret == VPUAPI_RET_SUCCESS) {
						if ((error_reason & 0xf0000000)) {
							if (vpu_do_sw_reset(dev, core, inst, error_reason)
								== VPUAPI_RET_TIMEOUT) {
								break;
							}
						}
					}
				}

				for (inst = 0; inst < MAX_NUM_VPU_INSTANCE; inst++) {
				}
				mdelay(10);
				if (time_after(jiffies, timeout)) {
					return VPUAPI_RET_TIMEOUT;
				}
			}
		} else {
			ret = wave_sleep_wake(dev, core, VPU_WAKE_MODE);
		}
	} else {
		vpu_err("vpu_sleep_wake Unknown product id : %08x\n", product_code);
		ret = VPUAPI_RET_FAILURE;
	}
	return ret;
}
#endif
// PARAMETER
// reset_mode
// 0 : safely
// 1 : force
int vpuapi_sw_reset(hb_vpu_dev_t *dev, u32 core, u32 inst, int reset_mode)
{
	u32 val = 0;
	int product_code;
	int ret;
	u32 supportDualCore;
	u32 supportBackbone;
	u32 supportVcoreBackbone;
	u32 supportVcpuBackbone;
#if defined(SUPPORT_SW_UART) || defined(SUPPORT_SW_UART_V2)
	u32 regSwUartStatus;
#endif
	vdi_lock(dev, VPUDRV_MUTEX_VPU);

	product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
	if (!PRODUCT_CODE_W_SERIES(product_code)) {
		vpu_err("Doesn't support swreset for coda \n");
		vdi_unlock(dev, VPUDRV_MUTEX_VPU);
		return VPUAPI_RET_INVALID_PARAM;
	}

	VPU_WRITEL(W5_VPU_BUSY_STATUS, 0);

	vpu_debug(5, "mode=%d\n", reset_mode);

	if (reset_mode == 0) {
		ret = wave_sleep_wake(dev, core, VPU_SLEEP_MODE);
		vpu_debug(5, "Sleep done ret=%d\n", ret);
		if (ret != VPUAPI_RET_SUCCESS) {
			vdi_unlock(dev, VPUDRV_MUTEX_VPU);
			return ret;
		}
	}

	val = VPU_READL(W5_VPU_RET_VPU_CONFIG0);
	if (((val>>16) & 0x1) == 0x01) {
		supportBackbone = 1;
	} else {
		supportBackbone = 0;
	}
	if (((val>>22) & 0x1) == 0x01) {
		supportVcoreBackbone = 1;
	} else {
		supportVcoreBackbone = 0;
	}
	if (((val>>28) & 0x1) == 0x01) {
		supportVcpuBackbone = 1;
	} else {
		supportVcpuBackbone = 0;
	}

	val = VPU_READL(W5_VPU_RET_VPU_CONFIG1);
	if (((val>>26) & 0x1) == 0x01) {
		supportDualCore = 1;
	} else {
		supportDualCore = 0;
	}

	if (supportBackbone == 1) {
		if (supportDualCore == 1) {
			WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCORE0, 0x7);
			if (vpuapi_wait_bus_busy(dev, core, W5_BACKBONE_BUS_STATUS_VCORE0) !=
				VPUAPI_RET_SUCCESS) {
				WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCORE0, 0x00);
				vdi_unlock(dev, VPUDRV_MUTEX_VPU);
				return VPUAPI_RET_TIMEOUT;
			}

			WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCORE1, 0x7);
			if (vpuapi_wait_bus_busy(dev, core, W5_BACKBONE_BUS_STATUS_VCORE1) !=
				VPUAPI_RET_SUCCESS) {
				WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCORE1, 0x00);
				vdi_unlock(dev, VPUDRV_MUTEX_VPU);
				return VPUAPI_RET_TIMEOUT;
			}
		} else {
			if (supportVcoreBackbone == 1) {
				if (supportVcpuBackbone == 1) {
					// Step1 : disable request
					WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCPU, 0xFF);

					// Step2 : Waiting for completion of bus transaction
					if (vpuapi_wait_bus_busy(dev, core, W5_BACKBONE_BUS_STATUS_VCPU)
						!= VPUAPI_RET_SUCCESS) {
						WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCPU, 0x00);
						vdi_unlock(dev, VPUDRV_MUTEX_VPU);
						return VPUAPI_RET_TIMEOUT;
					}
				}
				// Step1 : disable request
				WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCORE0, 0x7);

				// Step2 : Waiting for completion of bus transaction
				if (vpuapi_wait_bus_busy(dev, core, W5_BACKBONE_BUS_STATUS_VCORE0)
					!= VPUAPI_RET_SUCCESS) {
					WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCORE0, 0x00);
					vdi_unlock(dev, VPUDRV_MUTEX_VPU);
					return VPUAPI_RET_TIMEOUT;
				}
			} else {
				// Step1 : disable request
				WriteVpuFIORegister(dev, core, W5_COMBINED_BACKBONE_BUS_CTRL, 0x7);

				// Step2 : Waiting for completion of bus transaction
				if (vpuapi_wait_bus_busy(dev, core, W5_COMBINED_BACKBONE_BUS_STATUS)
					!= VPUAPI_RET_SUCCESS) {
					WriteVpuFIORegister(dev, core, W5_COMBINED_BACKBONE_BUS_CTRL, 0x00);
					vdi_unlock(dev, VPUDRV_MUTEX_VPU);
					return VPUAPI_RET_TIMEOUT;
				}
			}
		}
	} else {
		// Step1 : disable request
		WriteVpuFIORegister(dev, core, W5_GDI_BUS_CTRL, 0x100);

		// Step2 : Waiting for completion of bus transaction
		if (vpuapi_wait_bus_busy(dev, core, W5_GDI_BUS_STATUS) != VPUAPI_RET_SUCCESS) {
			WriteVpuFIORegister(dev, core, W5_GDI_BUS_CTRL, 0x00);
			vdi_unlock(dev, VPUDRV_MUTEX_VPU);
			return VPUAPI_RET_TIMEOUT;
		}
	}

#if defined(SUPPORT_SW_UART) || defined(SUPPORT_SW_UART_V2)
	regSwUartStatus = VPU_READL(W5_SW_UART_STATUS);
#endif
	val = W5_RST_BLOCK_ALL;
	VPU_WRITEL(W5_VPU_RESET_REQ, val);

	if (vpuapi_wait_reset_busy(dev, core) != VPUAPI_RET_SUCCESS) {
		VPU_WRITEL(W5_VPU_RESET_REQ, 0);
		vdi_unlock(dev, VPUDRV_MUTEX_VPU);
		return VPUAPI_RET_TIMEOUT;
	}

	VPU_WRITEL(W5_VPU_RESET_REQ, 0);
#if defined(SUPPORT_SW_UART) || defined(SUPPORT_SW_UART_V2)
	VPU_WRITEL(W5_SW_UART_STATUS, regSwUartStatus); // enable SW UART.
#endif

	vpu_debug(5, "VPU_RESET done RESET_REQ=0x%x\n", val);

	// Step3 : must clear GDI_BUS_CTRL after done SW_RESET
	if (supportBackbone == 1) {
		if (supportDualCore == 1) {
			WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCORE0, 0x00);
			WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCORE1, 0x00);
		} else {
			if (supportVcoreBackbone == 1) {
				if (supportVcpuBackbone == 1) {
					WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCPU, 0x00);
				}
				WriteVpuFIORegister(dev, core, W5_BACKBONE_BUS_CTRL_VCORE0, 0x00);
			} else {
				WriteVpuFIORegister(dev, core, W5_COMBINED_BACKBONE_BUS_CTRL, 0x00);
			}
		}
	} else {
		WriteVpuFIORegister(dev, core, W5_GDI_BUS_CTRL, 0x00);
	}


	ret = wave_sleep_wake(dev, core, VPU_WAKE_MODE);

	vpu_debug(5, "Wake done ret = %d\n", ret);
	vdi_unlock(dev, VPUDRV_MUTEX_VPU);

	return ret;
}

static int wave_issue_command(hb_vpu_dev_t *dev, u32 core, u32 inst,
				u32 cmd)
{
	int ret;
	u32 codec_mode;
	unsigned char *codec_inst;
	hb_vpu_instance_pool_t *vip = get_instance_pool_handle(dev, core);

	if (vip == NULL) {
		return VPUAPI_RET_INVALID_PARAM;
	}

	codec_inst = &vip->codec_inst_pool[inst][0];
	// indicates codecMode in CodecInst structure in vpuapifunc.h
	codec_inst = codec_inst + (sizeof(u32) * 3);
	memcpy(&codec_mode, codec_inst, 4);

	VPU_WRITEL(W5_CMD_INSTANCE_INFO, (codec_mode << 16)|(inst&0xffff));
	VPU_WRITEL(W5_VPU_BUSY_STATUS, 1);
	VPU_WRITEL(W5_COMMAND, cmd);

	VPU_WRITEL(W5_VPU_HOST_INT_REQ, 1);

	ret = vpuapi_wait_vpu_busy(dev, core);

	return ret;
}

static int wave_send_query_command(hb_vpu_dev_t *dev,
			unsigned long core, unsigned long inst, u32 queryOpt)
{
	int ret;
	VPU_WRITEL(W5_QUERY_OPTION, queryOpt);
	VPU_WRITEL(W5_VPU_BUSY_STATUS, 1);
	ret = wave_issue_command(dev, core, inst, W5_QUERY);
	if (ret != VPUAPI_RET_SUCCESS) {
		vpu_err("fail1 ret=%d\n", ret);
		return ret;
	}

	if (VPU_READL(W5_RET_SUCCESS) == 0) {
		vpu_err("success=%d\n", VPU_READL(W5_RET_SUCCESS));
		return VPUAPI_RET_FAILURE;
	}

	return VPUAPI_RET_SUCCESS;
}

int vpuapi_get_output_info(hb_vpu_dev_t *dev, u32 core,
		u32 inst, u32 *error_reason)
{
	int ret = VPUAPI_RET_SUCCESS;
	u32 val;

	vdi_lock(dev, VPUDRV_MUTEX_VPU);
	VPU_WRITEL(W5_CMD_DEC_ADDR_REPORT_BASE, 0);
	VPU_WRITEL(W5_CMD_DEC_REPORT_SIZE,      0);
	VPU_WRITEL(W5_CMD_DEC_REPORT_PARAM,     0);

	ret = wave_send_query_command(dev, core, inst, GET_RESULT);
	if (ret != VPUAPI_RET_SUCCESS) {
		vdi_unlock(dev, VPUDRV_MUTEX_VPU);
		return ret;
	}

	vpu_debug(5, "[+] success=%d, fail_reason=0x%x, error_reason=0x%x\n",
		VPU_READL(W5_RET_DEC_DECODING_SUCCESS),
		VPU_READL(W5_RET_FAIL_REASON),
		VPU_READL(W5_RET_DEC_ERR_INFO));
	val = VPU_READL(W5_RET_DEC_DECODING_SUCCESS);
	if ((val & 0x01) == 0) {
#ifdef SUPPORT_SW_UART
		*error_reason = 0;
#else
		*error_reason = VPU_READL(W5_RET_DEC_ERR_INFO);
#endif
	} else {
		*error_reason = 0x00;
	}

	if (ret != VPUAPI_RET_SUCCESS) {
		vpu_debug(5, "[-] ret=%d\n", ret);
	}
	vdi_unlock(dev, VPUDRV_MUTEX_VPU);
	return ret;
}

int vpuapi_dec_clr_all_disp_flag(hb_vpu_dev_t *dev, u32 core, u32 inst)
{
	int ret = VPUAPI_RET_SUCCESS;
	u32 val;

	vdi_lock(dev, VPUDRV_MUTEX_VPU);

	VPU_WRITEL(W5_CMD_DEC_CLR_DISP_IDC, 0xffffffff);
	VPU_WRITEL(W5_CMD_DEC_SET_DISP_IDC, 0);
	ret = wave_send_query_command(dev, core, inst, UPDATE_DISP_FLAG);
	if (ret != VPUAPI_RET_SUCCESS) {
		vpu_err("ret=%d\n", ret);
		vdi_unlock(dev, VPUDRV_MUTEX_VPU);
		return ret;
	}

	val = VPU_READL(W5_RET_SUCCESS);
	if (val == 0) {
		ret = VPUAPI_RET_FAILURE;
	}

	if (ret != VPUAPI_RET_SUCCESS) {
		vpu_err("ret=%d\n", ret);
	}
	vdi_unlock(dev, VPUDRV_MUTEX_VPU);
	return ret;
}

int vpuapi_dec_set_stream_end(hb_vpu_dev_t *dev, u32 core, u32 inst)
{
	int ret = VPUAPI_RET_SUCCESS;
	u32 val;
	int product_code;

	vdi_lock(dev, VPUDRV_MUTEX_VPU);

	product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
	if (PRODUCT_CODE_W_SERIES(product_code)) {
		VPU_WRITEL(W5_BS_OPTION, (1/*STREAM END*/<<1) | (1/*explictEnd*/));
		// keep not to be changed
		// WriteVpuRegister(core, W5_BS_WR_PTR, pDecInfo->streamWrPtr);

		ret = wave_issue_command(dev, core, inst, W5_UPDATE_BS);
		if (ret != VPUAPI_RET_SUCCESS) {
			vpu_err("ret=%d\n", ret);
			vdi_unlock(dev, VPUDRV_MUTEX_VPU);
			return ret;
		}

		val = VPU_READL(W5_RET_SUCCESS);
		if (val == 0) {
			ret = VPUAPI_RET_FAILURE;
			vpu_err("ret=%d\n", ret);
			vdi_unlock(dev, VPUDRV_MUTEX_VPU);
			return ret;
		}
	} else {
		ret = VPUAPI_RET_FAILURE;
	}

	if (ret != VPUAPI_RET_SUCCESS) {
		vpu_err("ret=%d\n", ret);
	}
	vdi_unlock(dev, VPUDRV_MUTEX_VPU);
	return ret;
}

int vpuapi_close(hb_vpu_dev_t *dev, u32 core, u32 inst)
{
	int ret = VPUAPI_RET_SUCCESS;
	u32 val;
	int product_code;


	vdi_lock(dev, VPUDRV_MUTEX_VPU);
	product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
	vpu_debug(5, "[+] core=%d, inst=%d, product_code=0x%x\n", core, inst,
		product_code);

	if (PRODUCT_CODE_W_SERIES(product_code)) {
		ret = wave_issue_command(dev, core, inst, W5_DESTROY_INSTANCE);
		if (ret != VPUAPI_RET_SUCCESS) {
			vpu_err("ret=%d\n", ret);
			vdi_unlock(dev, VPUDRV_MUTEX_VPU);
			return ret;
		}

		val = VPU_READL(W5_RET_SUCCESS);
		if (val == 0) {
			val = VPU_READL(W5_RET_FAIL_REASON);
			if (val == WAVE5_SYSERR_VPU_STILL_RUNNING) {
				ret = VPUAPI_RET_STILL_RUNNING;
			} else {
				ret = VPUAPI_RET_FAILURE;
			}
		} else {
			ret = VPUAPI_RET_SUCCESS;
		}
	} else {
			ret = VPUAPI_RET_FAILURE;
	}

	if (ret != VPUAPI_RET_SUCCESS) {
		vpu_err("ret=%d\n", ret);
	}
	vpu_debug(5, "[-] ret=%d\n", ret);
	vdi_unlock(dev, VPUDRV_MUTEX_VPU);
	return ret;
}

int vpu_do_sw_reset(hb_vpu_dev_t *dev, u32 core, u32 inst, u32 error_reason)
{
	int ret;
	hb_vpu_instance_pool_t *vip = get_instance_pool_handle(dev, core);
	int doSwResetInstIdx;

	vpu_debug(5, "[+] core=%d, inst=%d, error_reason=0x%x\n",
		core, inst, error_reason);
	if (vip == NULL)
		return VPUAPI_RET_FAILURE;

	vdi_lock(dev, VPUDRV_MUTEX_RESET);
	ret = VPUAPI_RET_SUCCESS;
	doSwResetInstIdx = vip->doSwResetInstIdxPlus1 - 1;
	if (doSwResetInstIdx == inst || (error_reason & 0xf0000000)) {
		ret = vpuapi_sw_reset(dev, core, inst, 0);
		if (ret == VPUAPI_RET_STILL_RUNNING) {
			vpu_debug(5, "VPU is still running\n");
			vip->doSwResetInstIdxPlus1 = (inst + 1);
		} else if (ret == VPUAPI_RET_SUCCESS) {
			vpu_debug(5, "success\n");
			vip->doSwResetInstIdxPlus1 = 0;
		} else {
			vpu_err("Fail result=0x%x\n", ret);
			vip->doSwResetInstIdxPlus1 = 0;
		}
	}
	vdi_unlock(dev, VPUDRV_MUTEX_RESET);
	vpu_debug(5, "[-] vip->doSwResetInstIdx=%d, ret=%d\n",
		vip->doSwResetInstIdxPlus1, ret);
	mdelay(10);
	return ret;
}
#endif

static int vpu_alloc_dma_buffer(hb_vpu_dev_t *dev, hb_vpu_drv_buffer_t * vb)
{
	if (!vb || !dev)
		return -1;

#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	vb->phys_addr = (unsigned long)vmem_alloc(&s_vmem, vb->size, 0);
	if ((unsigned long)vb->phys_addr == (unsigned long)-1) {
		vpu_err("Physical memory allocation error size=%d\n", vb->size);
		return -1;
	}

	vb->base = (unsigned long)(s_video_memory.base +
				   (vb->phys_addr - s_video_memory.phys_addr));
#else
	vb->base = (unsigned long)dma_alloc_coherent(dev->device, PAGE_ALIGN(vb->size),
						     (dma_addr_t
						      *) (&vb->phys_addr),
						     GFP_DMA | GFP_KERNEL);
	if ((void *)(vb->base) == NULL) {
		vpu_err("Physical memory allocation error size=%d\n", vb->size);
		return -1;
	}
#endif
	return 0;
}

static void vpu_free_dma_buffer(hb_vpu_dev_t *dev, hb_vpu_drv_buffer_t * vb)
{
	if (!vb || !dev)
		return;

#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (vb->base)
		vmem_free(&s_vmem, vb->phys_addr, 0);
#else
	if (vb->base)
		dma_free_coherent(dev->device, PAGE_ALIGN(vb->size), (void *)vb->base,
				  vb->phys_addr);
#endif
}

static int vpu_free_instances(struct file *filp)
{
	hb_vpu_instance_list_t *vil, *n;
	hb_vpu_instance_pool_t *vip;
	void *vip_base;
	int instance_pool_size_per_core;
#ifdef USE_MUTEX_IN_KERNEL_SPACE
#else
	void *vdi_mutexes_base;
	const int PTHREAD_MUTEX_T_DESTROY_VALUE = 0xdead10cc;
	int core;
	unsigned long timeout = jiffies + HZ;
#endif
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;

	vpu_debug_enter();
	if (!filp) {
		vpu_err("failed to free instances, filp is null.");
		return -1;
	}
	priv = filp->private_data;
	dev = priv->vpu_dev;
	if (!dev) {
		vpu_err("failed to free instances, dev is null.");
		return -1;
	}
	/* s_instance_pool.size assigned to the size of all core once call VDI_IOCTL_GET_INSTANCE_POOL by user. */
	instance_pool_size_per_core =
	    (dev->instance_pool.size / MAX_NUM_VPU_CORE);

	list_for_each_entry_safe(vil, n, &dev->inst_list_head, list) {
		if (vil->filp == filp) {
			vip_base = (void *)(dev->instance_pool.base +
					    (instance_pool_size_per_core *
					     vil->core_idx));
			vpu_debug(5,
				  "vpu_free_instances detect instance crash instIdx=%d, "
				  "coreIdx=%d, vip_base=%p, instance_pool_size_per_core=%d\n",
				  (int)vil->inst_idx, (int)vil->core_idx,
				  vip_base, (int)instance_pool_size_per_core);
#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
#else
			core = vil->core_idx;
			VPU_WRITEL(W5_CMD_INSTANCE_INFO,
				(-1 << 16)|(vil->inst_idx&MAX_VPU_INSTANCE_IDX));
			VPU_ISSUE_COMMAND(vil->core_idx, W5_DESTROY_INSTANCE);
			while (VPU_READL(W5_VPU_BUSY_STATUS)) {
				if (time_after(jiffies, timeout)) {
					vpu_err("Timeout to do command %d",
						W5_DESTROY_INSTANCE);
					break;
				}
			}
			if (VPU_READL(W5_RET_SUCCESS) == 0) {
				int error = VPU_READL(W5_RET_FAIL_REASON);
				if (error == 0x1000) {
					VPU_WRITEL(W5_QUERY_OPTION, GET_RESULT);
					VPU_WRITEL(W5_CMD_INSTANCE_INFO,
						(-1 << 16)|(vil->inst_idx&MAX_VPU_INSTANCE_IDX));
					VPU_ISSUE_COMMAND(vil->core_idx, W5_QUERY);
					while (VPU_READL(W5_VPU_BUSY_STATUS)) {
						if (time_after(jiffies, timeout)) {
							vpu_err("Timeout to do command %d",
								W5_DESTROY_INSTANCE);
							break;
						}
					}
					VPU_WRITEL(W5_CMD_INSTANCE_INFO,
						(-1 << 16)|(vil->inst_idx&MAX_VPU_INSTANCE_IDX));
					VPU_ISSUE_COMMAND(vil->core_idx, W5_DESTROY_INSTANCE);
					while (VPU_READL(W5_VPU_BUSY_STATUS)) {
						if (time_after(jiffies, timeout)) {
							vpu_err("Timeout to do command %d",
								W5_DESTROY_INSTANCE);
							break;
						}
					}
				} else {
					vpu_err("Command %d failed [0x%x]",
						W5_DESTROY_INSTANCE, VPU_READL(W5_RET_FAIL_REASON));
				}
			}
#endif
			vip = (hb_vpu_instance_pool_t *) vip_base;
			if (vip) {
				/* only first 4 byte is key point(inUse of CodecInst in vpuapi)
				   to free the corresponding instance. */
				memset(&vip->codec_inst_pool[vil->inst_idx],
				       0x00, 4);
#ifdef USE_MUTEX_IN_KERNEL_SPACE
#else
#define PTHREAD_MUTEX_T_HANDLE_SIZE 4
				vdi_mutexes_base =
				    (vip_base +
				     (instance_pool_size_per_core -
				      PTHREAD_MUTEX_T_HANDLE_SIZE * 4));
				vpu_debug(5,
					  "vpu_free_instances : force to destroy "
					  "vdi_mutexes_base=%p in userspace \n",
					  vdi_mutexes_base);
				if (vdi_mutexes_base) {
					int i;
					for (i = 0; i < 4; i++) {
						memcpy(vdi_mutexes_base,
						       &PTHREAD_MUTEX_T_DESTROY_VALUE,
						       PTHREAD_MUTEX_T_HANDLE_SIZE);
						vdi_mutexes_base +=
						    PTHREAD_MUTEX_T_HANDLE_SIZE;
					}
				}
#endif
#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
				vpu_close_instance(dev, (u32)vil->core_idx, (u32)vil->inst_idx);
#endif
			}
			dev->vpu_open_ref_count--;
			list_del(&vil->list);
			kfree(vil);
			test_and_clear_bit(vil->inst_idx, vpu_inst_bitmap);
			spin_lock(&dev->poll_spinlock);
			dev->poll_event[vil->inst_idx] = VPU_INST_CLOSED;
			spin_unlock(&dev->poll_spinlock);
			wake_up_interruptible(&dev->poll_wait_q[vil->inst_idx]);
			memset(&dev->vpu_ctx[vil->inst_idx], 0x00,
				sizeof(dev->vpu_ctx[vil->inst_idx]));
			memset(&dev->vpu_status[vil->inst_idx], 0x00,
				sizeof(dev->vpu_status[vil->inst_idx]));
		}
	}
	vpu_debug_leave();

	return 0;
}

static int vpu_free_buffers(struct file *filp)
{
	hb_vpu_drv_buffer_pool_t *pool, *n;
	hb_vpu_drv_buffer_t vb;
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;
	vpu_debug_enter();
	if (!filp) {
		vpu_err("failed to free vpu buffers, filp is null.");
		return -1;
	}
	priv = filp->private_data;
	dev = priv->vpu_dev;

	if (!dev) {
		vpu_err("failed to free vpu buffers, dev is null.");
		return -1;
	}

	list_for_each_entry_safe(pool, n, &dev->vbp_head, list) {
		if (pool->filp == filp) {
			vb = pool->vb;
			if (vb.base) {
				vpu_free_dma_buffer(dev, &vb);
				list_del(&pool->list);
				kfree(pool);
			}
		}
	}
	vpu_debug_leave();

	return 0;
}

#ifdef SUPPORT_MULTI_INST_INTR
static inline u32 vpu_filter_inst_idx(u32 reg_val)
{
	u32 inst_idx;
	int i;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (((reg_val >> i) & 0x01) == 1)
			break;
	}
	inst_idx = i;
	return inst_idx;
}

static s32 vpu_get_inst_idx(hb_vpu_dev_t * dev, u32 * reason,
			    u32 empty_inst, u32 done_inst, u32 seq_inst)
{
	s32 inst_idx;
	u32 reg_val;
	u32 int_reason;

	int_reason = *reason;
	vpu_debug(7, "int_reason=0x%x, empty_inst=0x%x, done_inst=0x%x\n",
		  int_reason, empty_inst, done_inst);

	if (int_reason & (1 << INT_WAVE5_BSBUF_EMPTY)) {
		reg_val = (empty_inst & MAX_VPU_INSTANCE_IDX);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_BSBUF_EMPTY);
		vpu_debug(7, "W5_RET_BS_EMPTY_INST reg_val=0x%x, inst_idx=%d\n",
			  reg_val, inst_idx);
		goto GET_VPU_INST_IDX_HANDLED;
	}

	if (int_reason & (1 << INT_WAVE5_INIT_SEQ)) {
		reg_val = (seq_inst & MAX_VPU_INSTANCE_IDX);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_INIT_SEQ);
		vpu_debug(7, "RET_SEQ_DONE_INSTANCE_INFO INIT_SEQ reg_val=0x%x,"
			  "inst_idx=%d\n", reg_val, inst_idx);
		goto GET_VPU_INST_IDX_HANDLED;
	}

	if (int_reason & (1 << INT_WAVE5_DEC_PIC)) {
		reg_val = (done_inst & MAX_VPU_INSTANCE_IDX);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_DEC_PIC);
		vpu_debug(7, "W5_RET_QUEUE_CMD_DONE_INST DEC_PIC reg_val=0x%x,"
			  "inst_idx=%d\n", reg_val, inst_idx);

		goto GET_VPU_INST_IDX_HANDLED;
	}

	if (int_reason & (1 << INT_WAVE5_ENC_SET_PARAM)) {
		reg_val = (seq_inst & MAX_VPU_INSTANCE_IDX);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_ENC_SET_PARAM);
		vpu_debug(7,
			  "RET_SEQ_DONE_INSTANCE_INFO ENC_SET_PARAM reg_val=0x%x,"
			  "inst_idx=%d\n", reg_val, inst_idx);
		goto GET_VPU_INST_IDX_HANDLED;
	}

#ifdef SUPPORT_SOURCE_RELEASE_INTERRUPT
	if (int_reason & (1 << INT_WAVE5_ENC_SRC_RELEASE)) {
		reg_val = (done_inst & MAX_VPU_INSTANCE_IDX);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_ENC_SRC_RELEASE);
		vpu_debug(7, "W5_RET_QUEUE_CMD_DONE_INST ENC_SET_PARAM "
			  "reg_val=0x%x, inst_idx=%d\n", reg_val, inst_idx);
		goto GET_VPU_INST_IDX_HANDLED;
	}
#endif

	inst_idx = -1;
	*reason = 0;
	vpu_err("UNKNOWN INTERRUPT REASON: %08x\n", int_reason);

GET_VPU_INST_IDX_HANDLED:
	vpu_debug(7, "inst_idx=%d. *reason=0x%x\n", inst_idx, *reason);

	return inst_idx;
}
#endif

static irqreturn_t vpu_irq_handler(int irq, void *dev_id)
{
	hb_vpu_dev_t *dev = (hb_vpu_dev_t *) dev_id;

	/* this can be removed. it also work in VPU_WaitInterrupt of API function */
	int core;
	int product_code;
#ifdef SUPPORT_MULTI_INST_INTR
	u32 intr_reason = 0;
	s32 intr_inst_index = 0;
#endif
	unsigned long flags_mp;

#ifdef VPU_IRQ_CONTROL
	spin_lock_irqsave(&dev->irq_spinlock, flags_mp);
	disable_irq_nosync(dev->irq);
	dev->irq_trigger = 1;
	spin_unlock_irqrestore(&dev->irq_spinlock, flags_mp);
#endif

	for (core = 0; core < MAX_NUM_VPU_CORE; core++) {
		if (dev->bit_fm_info[core].size == 0) {
			/* it means that we didn't get an information the current core
			   from API layer. No core activated. */
			vpu_err("bit_fm_info[core].size is zero\n");
			continue;
		}
		product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);

		if (PRODUCT_CODE_W_SERIES(product_code)) {
			if (VPU_READL(W5_VPU_VPU_INT_STS)) {
#ifdef SUPPORT_MULTI_INST_INTR
				u32 empty_inst;
				u32 done_inst;
				u32 seq_inst;
				u32 i, reason, reason_clr;

				reason = VPU_READL(W5_VPU_INT_REASON);
				empty_inst = VPU_READL(W5_RET_BS_EMPTY_INST);
				done_inst =
				    VPU_READL(W5_RET_QUEUE_CMD_DONE_INST);
				seq_inst =
				    VPU_READL(W5_RET_SEQ_DONE_INSTANCE_INFO);
				reason_clr	= reason;

				vpu_debug(7, "vpu_irq_handler reason=0x%x, "
					  "empty_inst=0x%x, done_inst=0x%x, other_inst=0x%x \n",
					  reason, empty_inst, done_inst,
					  seq_inst);

				for (i=0; i < MAX_NUM_VPU_INSTANCE; i++) {
					if (0 == empty_inst && 0 == done_inst && 0 == seq_inst)
						break;
					intr_reason = reason;
					intr_inst_index =
						vpu_get_inst_idx(dev, &intr_reason,
							empty_inst, done_inst,
							seq_inst);
					if (intr_inst_index >= 0
					    && intr_inst_index < MAX_NUM_VPU_INSTANCE) {
						if (intr_reason ==
						    (1 << INT_WAVE5_BSBUF_EMPTY)) {
							empty_inst =
							    empty_inst & ~(1 <<
									   intr_inst_index);
							VPU_WRITEL(W5_RET_BS_EMPTY_INST,
								   empty_inst);
							if (0 == empty_inst) {
								reason &= ~(1 << INT_WAVE5_BSBUF_EMPTY);
							}
							vpu_debug(7,
								  "W5_RET_BS_EMPTY_INST Clear "
								  "empty_inst=0x%x, intr_inst_index=%d\n",
								  empty_inst,
								  intr_inst_index);
						}
						if (intr_reason == (1 << INT_WAVE5_DEC_PIC)) {
							done_inst = done_inst & ~(1 << intr_inst_index);
							VPU_WRITEL(W5_RET_QUEUE_CMD_DONE_INST, done_inst);
							if (0 == done_inst) {
								reason &= ~(1 << INT_WAVE5_DEC_PIC);
							}
							vpu_debug(7, "W5_RET_QUEUE_CMD_DONE_INST Clear "
								"done_inst=0x%x, intr_inst_index=%d\n",
								done_inst, intr_inst_index);
						}
						if ((intr_reason == (1 << INT_WAVE5_INIT_SEQ))
							|| (intr_reason ==
							(1 << INT_WAVE5_ENC_SET_PARAM))) {
							seq_inst = seq_inst & ~(1 << intr_inst_index);
							VPU_WRITEL(W5_RET_SEQ_DONE_INSTANCE_INFO, seq_inst);
							if (0 == seq_inst) {
								reason &= ~(1 << INT_WAVE5_INIT_SEQ
									| 1 << INT_WAVE5_ENC_SET_PARAM);
							}
							vpu_debug(7,
								  "W5_RET_QUEUE_CMD_DONE_INST "
								  "Clear done_inst=0x%x, intr_inst_index=%d\n",
								  done_inst,
								  intr_inst_index);
						}
						if (!kfifo_is_full
						    (&dev->interrupt_pending_q
						     [intr_inst_index])) {
							if (intr_reason ==
							    (1 << INT_WAVE5_ENC_PIC)) {
								u32 ll_intr_reason =
								    (1 <<
								     INT_WAVE5_ENC_PIC);
								kfifo_in_spinlocked
								    (&dev->interrupt_pending_q
								     [intr_inst_index],
								     &ll_intr_reason,
								     sizeof(u32),
								     &dev->vpu_kfifo_lock);
							} else {
								kfifo_in_spinlocked
								    (&dev->interrupt_pending_q
								     [intr_inst_index],
								     &intr_reason,
								     sizeof(u32),
								     &dev->vpu_kfifo_lock);
							}
						} else {
							vpu_err
							    ("kfifo_is_full kfifo_count=%d \n",
							     kfifo_len
							     (&dev->interrupt_pending_q
							      [intr_inst_index]));
						}
					} else {
						vpu_err("intr_inst_index is wrong "
							"intr_inst_index=%d \n",
							intr_inst_index);
					}
				}
				if (0 != reason)
					vpu_err("INTERRUPT REASON REMAINED: %08x\n", reason);
				VPU_WRITEL(W5_VPU_INT_REASON_CLEAR, reason_clr);
#else
				dev->interrupt_reason =
				    VPU_READL(W5_VPU_INT_REASON);
				VPU_WRITEL(W5_VPU_INT_REASON_CLEAR,
					   dev->interrupt_reason);
#endif

				VPU_WRITEL(W5_VPU_VINT_CLEAR, 0x1);
			}
		} else if (!PRODUCT_CODE_W_SERIES(product_code)) {
			if (VPU_READL(BIT_INT_STS)) {
#ifdef SUPPORT_MULTI_INST_INTR
				intr_reason = VPU_READL(BIT_INT_REASON);
				// in case of coda seriese. treats intr_inst_index is already 0
				intr_inst_index = 0;
				kfifo_in_spinlocked(&dev->interrupt_pending_q
						    [intr_inst_index],
						    &intr_reason, sizeof(u32),
						    &dev->vpu_kfifo_lock);
#else
				dev->interrupt_reason =
				    VPU_READL(BIT_INT_REASON);
#endif
				VPU_WRITEL(BIT_INT_CLEAR, 0x1);
			}
		} else {
			vpu_err("Unknown product id : %08x\n",
				  product_code);
			continue;
		}
#ifdef SUPPORT_MULTI_INST_INTR
		vpu_debug(7, "product: 0x%08x intr_reason: 0x%08x\n\n",
			  product_code, intr_reason);
#else
		vpu_debug(7, "product: 0x%08x intr_reason: 0x%08x\n",
			  product_code, dev->interrupt_reason);
#endif
	}

	/* notify the interrupt to user space */
	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

#ifdef SUPPORT_MULTI_INST_INTR
	if (intr_inst_index >= 0 && intr_inst_index < MAX_NUM_VPU_INSTANCE) {
		dev->interrupt_flag[intr_inst_index] = 1;
		wake_up_interruptible(&dev->interrupt_wait_q[intr_inst_index]);
	}
#else
	dev->interrupt_flag = 1;
	wake_up_interruptible(&dev->interrupt_wait_q);
#endif

	return IRQ_HANDLED;
}

static hb_vpu_driver_data_t vpu_drv_data = {
	.fw_name = "vpu.bin",
};

static const struct of_device_id vpu_of_match[] = {
	{
	 .compatible = "hobot,hobot_vpu",
	 .data = &vpu_drv_data,
	 },
	{},
};

static void vpu_parse_dts(struct device_node *np, hb_vpu_dev_t * vpu_dev)
{
	hb_vpu_platform_data_t *pdata = vpu_dev->plat_data;

	if (!np)
		return;

	of_property_read_u32(np, "ip_ver", &pdata->ip_ver);
	of_property_read_u32(np, "clock_rate", &pdata->clock_rate);
	of_property_read_u32(np, "min_rate", &pdata->min_rate);
}

static void *vpu_get_drv_data(struct platform_device *pdev)
{
	hb_vpu_driver_data_t *drv_data = NULL;

	if (pdev->dev.of_node) {
		const struct of_device_id *id;
		id = of_match_node(of_match_ptr(vpu_of_match),
				   pdev->dev.of_node);
		if (id) {
			drv_data = (hb_vpu_driver_data_t *) id->data;
		} else {
			drv_data = (hb_vpu_driver_data_t *)
			    platform_get_device_id(pdev)->driver_data;
		}
	}
	return drv_data;
}

static int vpu_open(struct inode *inode, struct file *filp)
{
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;
	vpu_debug_enter();

	priv = kzalloc(sizeof(hb_vpu_priv_t), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev = container_of(inode->i_cdev, hb_vpu_dev_t, cdev);
	if (!dev) {
		vpu_err("failed to get vpu dev data");
		return -1;
	}

	spin_lock(&dev->vpu_spinlock);
	if (dev->open_count == 0) {
		dev->vpu_freq = vpu_clk_freq;
		pm_qos_add_request(&dev->vpu_pm_qos_req, PM_QOS_DEVFREQ, 8300);
	}
	dev->open_count++;
	priv->vpu_dev = dev;
	priv->inst_index = -1;
	filp->private_data = (void *)priv;
	spin_unlock(&dev->vpu_spinlock);
	hb_vpu_clk_enable(dev, dev->vpu_freq);

	vpu_debug_leave();
	return 0;
}

/*static int vpu_ioctl(struct inode *inode, struct file *filp, u_int cmd,
	u_long arg) // for kernel 2.6.9 of C&M*/
static long vpu_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	int ret = 0;
	int inst_index;
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;

	priv = filp->private_data;
	dev = priv->vpu_dev;
	if (!dev) {
		vpu_err("failed to get vpu dev data");
		return -1;
	}

	switch (cmd) {
	case VDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY:
		{
			hb_vpu_drv_buffer_pool_t *vbp;
			vpu_debug(5, "[+]VDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY\n");

			if ((ret = down_interruptible(&dev->vpu_sem)) == 0) {
				vbp = kzalloc(sizeof(*vbp), GFP_KERNEL);
				if (!vbp) {
					up(&dev->vpu_sem);
					return -ENOMEM;
				}

				ret =
				    copy_from_user(&(vbp->vb),
						   (hb_vpu_drv_buffer_t *) arg,
						   sizeof(hb_vpu_drv_buffer_t));
				if (ret) {
					kfree(vbp);
					up(&dev->vpu_sem);
					return -EFAULT;
				}

				ret = vpu_alloc_dma_buffer(dev, &(vbp->vb));
				if (ret == -1) {
					ret = -ENOMEM;
					kfree(vbp);
					up(&dev->vpu_sem);
					break;
				}
				ret =
				    copy_to_user((void __user *)arg, &(vbp->vb),
						 sizeof(hb_vpu_drv_buffer_t));
				if (ret) {
					kfree(vbp);
					ret = -EFAULT;
					up(&dev->vpu_sem);
					break;
				}

				vbp->filp = filp;
				spin_lock(&dev->vpu_spinlock);
				list_add(&vbp->list, &dev->vbp_head);
				spin_unlock(&dev->vpu_spinlock);

				up(&dev->vpu_sem);
			}
			vpu_debug(5, "[-]VDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY\n");
		}
		break;
	case VDI_IOCTL_FREE_PHYSICALMEMORY:
		{
			hb_vpu_drv_buffer_pool_t *vbp, *n;
			hb_vpu_drv_buffer_t vb;
			vpu_debug(5, "[+]VDI_IOCTL_FREE_PHYSICALMEMORY\n");

			if ((ret = down_interruptible(&dev->vpu_sem)) == 0) {
				ret =
				    copy_from_user(&vb,
						   (hb_vpu_drv_buffer_t *) arg,
						   sizeof(hb_vpu_drv_buffer_t));
				if (ret) {
					up(&dev->vpu_sem);
					return -EACCES;
				}

				if (vb.base)
					vpu_free_dma_buffer(dev, &vb);

				spin_lock(&dev->vpu_spinlock);
				list_for_each_entry_safe(vbp, n, &dev->vbp_head,
							 list) {
					if (vbp->vb.base == vb.base) {
						list_del(&vbp->list);
						kfree(vbp);
						break;
					}
				}
				spin_unlock(&dev->vpu_spinlock);

				up(&dev->vpu_sem);
			}
			vpu_debug(5, "[-]VDI_IOCTL_FREE_PHYSICALMEMORY\n");
		}
		break;
	case VDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO:
		{
#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
			vpu_debug(5,
				  "[+]VDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO\n");
			if (s_video_memory.base != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &s_video_memory,
						 sizeof(hb_vpu_drv_buffer_t));
				if (ret != 0)
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}
			vpu_debug(5,
				  "[-]VDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO\n");
#endif
		}
		break;

	case VDI_IOCTL_WAIT_INTERRUPT:
		{
			hb_vpu_drv_intr_t info;
#ifdef SUPPORT_MULTI_INST_INTR
			u32 intr_inst_index;
			u32 intr_reason_in_q;
			u32 interrupt_flag_in_q;
#endif
			unsigned long flags_mp;
			//vpu_debug(5, "[+]VDI_IOCTL_WAIT_INTERRUPT\n");

			ret = copy_from_user(&info, (hb_vpu_drv_intr_t *) arg,
					     sizeof(hb_vpu_drv_intr_t));
			if (ret != 0) {
				return -EFAULT;
			}
#ifdef SUPPORT_MULTI_INST_INTR
			intr_inst_index = info.intr_inst_index;

			intr_reason_in_q = 0;
			interrupt_flag_in_q =
			    kfifo_out_spinlocked(&dev->interrupt_pending_q
						 [intr_inst_index],
						 &intr_reason_in_q, sizeof(u32),
						 &dev->vpu_kfifo_lock);
			if (interrupt_flag_in_q > 0) {
				dev->interrupt_reason[intr_inst_index] =
				    intr_reason_in_q;
				vpu_debug(7,
					  "Interrupt Remain : intr_inst_index=%d, "
					  "intr_reason_in_q=0x%x, interrupt_flag_in_q=%d\n",
					  intr_inst_index, intr_reason_in_q,
					  interrupt_flag_in_q);
				goto INTERRUPT_REMAIN_IN_QUEUE;
			}
#endif
#ifdef SUPPORT_MULTI_INST_INTR
#ifdef SUPPORT_TIMEOUT_RESOLUTION
			ktime_t kt;
			kt = ktime_set(0, info.timeout * 1000 * 1000);
			ret =
			    wait_event_interruptible_hrtimeout
			    (dev->interrupt_wait_q[intr_inst_index],
			     dev->interrupt_flag[intr_inst_index]
			     != 0, kt);
#else
			ret =
			    wait_event_interruptible_timeout
			    (dev->interrupt_wait_q[intr_inst_index],
			     dev->interrupt_flag[intr_inst_index]
			     != 0, msecs_to_jiffies(info.timeout));
#endif
#else
			ret =
			    wait_event_interruptible_timeout
			    (dev->interrupt_wait_q, dev->interrupt_flag != 0,
			     msecs_to_jiffies(info.timeout));
#endif
#ifdef SUPPORT_TIMEOUT_RESOLUTION
			if (ret == -ETIME) {
				//DPRINTK("[VPUDRV][-]VDI_IOCTL_WAIT_INTERRUPT timeout = %d \n",
				//info.timeout);
				break;
			}
#else
			if (!ret) {
				ret = -ETIME;
				break;
			}
#endif
#if 0
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}
#endif

#ifdef SUPPORT_MULTI_INST_INTR
			intr_reason_in_q = 0;
			interrupt_flag_in_q =
			    kfifo_out_spinlocked(&dev->interrupt_pending_q
						 [intr_inst_index],
						 &intr_reason_in_q, sizeof(u32),
						 &dev->vpu_kfifo_lock);
			if (interrupt_flag_in_q > 0) {
				dev->interrupt_reason[intr_inst_index] =
				    intr_reason_in_q;
			} else {
				dev->interrupt_reason[intr_inst_index] = 0;
			}
#endif
#ifdef SUPPORT_MULTI_INST_INTR
			//vpu_debug(5, "inst_index(%d), s_interrupt_flag(%d),"
			//	  "reason(0x%08lx)\n", intr_inst_index,
			//	  dev->interrupt_flag[intr_inst_index],
			//	  dev->interrupt_reason[intr_inst_index]);
#else
			//vpu_debug(5, "s_interrupt_flag(%d), reason(0x%08lx)\n",
			//	  dev->interrupt_flag, dev->interrupt_reason);
#endif

#ifdef SUPPORT_MULTI_INST_INTR
INTERRUPT_REMAIN_IN_QUEUE:
			info.intr_reason =
			    dev->interrupt_reason[intr_inst_index];
			dev->interrupt_flag[intr_inst_index] = 0;
			dev->interrupt_reason[intr_inst_index] = 0;
#else
			info.intr_reason = dev->interrupt_reason;
			dev->interrupt_flag = 0;
			dev->interrupt_reason = 0;
#endif

#ifdef VPU_IRQ_CONTROL
			spin_lock_irqsave(&dev->irq_spinlock, flags_mp);
			if (dev->irq_trigger == 1) {
				enable_irq(dev->irq);
				dev->irq_trigger = 0;
			}
			spin_unlock_irqrestore(&dev->irq_spinlock, flags_mp);
#endif
			ret = copy_to_user((void __user *)arg, &info,
					   sizeof(hb_vpu_drv_intr_t));
			//vpu_debug(5, "[-]VDI_IOCTL_WAIT_INTERRUPT\n");
			if (ret != 0) {
				return -EFAULT;
			}
		}
		break;
#ifdef USE_MUTEX_IN_KERNEL_SPACE
		case VDI_IOCTL_VDI_LOCK: {
			unsigned int mutex_type;
			ret = copy_from_user(&mutex_type, (unsigned int *)arg,
					sizeof(unsigned int));
			if (ret != 0)
				return -EFAULT;

			ret = vdi_lock_user(dev, mutex_type);
		}
		break;
		case VDI_IOCTL_VDI_UNLOCK: {
			unsigned int mutex_type;
			ret = copy_from_user(&mutex_type, (unsigned int *)arg,
					sizeof(unsigned int));
			if (ret != 0)
				return -EFAULT;
			vdi_unlock(dev, mutex_type);
		}
		break;
#endif
	case VDI_IOCTL_SET_CLOCK_GATE:
		{
			u32 clkgate;

			vpu_debug(5, "[+]VDI_IOCTL_SET_CLOCK_GATE\n");
			if (get_user(clkgate, (u32 __user *) arg))
				return -EFAULT;
#ifdef VPU_SUPPORT_CLOCK_CONTROL
			if (clkgate)
				hb_vpu_clk_enable(dev, dev->vpu_freq);
			else
				hb_vpu_clk_disable(dev);
#endif
			vpu_debug(5, "[-]VDI_IOCTL_SET_CLOCK_GATE\n");
		}
		break;

	case VDI_IOCTL_GET_INSTANCE_POOL:
		{
			vpu_debug(5, "[+]VDI_IOCTL_GET_INSTANCE_POOL\n");
			if ((ret = down_interruptible(&dev->vpu_sem)) == 0) {
				if (dev->instance_pool.base != 0) {
					ret =
					    copy_to_user((void __user *)arg,
							 &dev->instance_pool,
							 sizeof
							 (hb_vpu_drv_buffer_t));
					if (ret != 0)
						ret = -EFAULT;
				} else {
					ret =
					    copy_from_user(&dev->instance_pool,
							   (hb_vpu_drv_buffer_t
							    *) arg,
							   sizeof
							   (hb_vpu_drv_buffer_t));
					if (ret == 0) {
#ifdef USE_VMALLOC_FOR_INSTANCE_POOL_MEMORY
						dev->instance_pool.base = (unsigned long)vmalloc(
							PAGE_ALIGN(dev->instance_pool.size));
						dev->instance_pool.phys_addr =
						    dev->instance_pool.base;

						if (dev->instance_pool.base !=
						    0)
#else
						if (vpu_alloc_dma_buffer
						    (dev, &dev->instance_pool,
						     dev) != -1)
#endif
						{
							/*clearing memory */
							memset((void *)
							       dev->instance_pool.base,
							       0x0,
							       PAGE_ALIGN(dev->instance_pool.size));
							ret =
							    copy_to_user((void
									  __user
									  *)arg,
									 &dev->instance_pool,
									 sizeof
									 (hb_vpu_drv_buffer_t));
							if (ret == 0) {
								/* success to get memory for instance pool */
								vpu_debug(5,
									  "[-]VDI_IOCTL_GET_INSTANCE_POOL\n");
								up(&dev->vpu_sem);
								break;
							}
						}

					}
					ret = -EFAULT;
				}
				up(&dev->vpu_sem);
			}
			vpu_debug(5, "[-]VDI_IOCTL_GET_INSTANCE_POOL\n");
		}
		break;

	case VDI_IOCTL_GET_COMMON_MEMORY:
		{
			vpu_debug(5, "[+]VDI_IOCTL_GET_COMMON_MEMORY\n");
			if (dev->common_memory.base != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &dev->common_memory,
						 sizeof(hb_vpu_drv_buffer_t));
				if (ret != 0)
					ret = -EFAULT;
			} else {
				ret = copy_from_user(&dev->common_memory,
						     (hb_vpu_drv_buffer_t *)
						     arg,
						     sizeof
						     (hb_vpu_drv_buffer_t));
				if (ret == 0) {
					if (vpu_alloc_dma_buffer
					    (dev, &dev->common_memory) != -1) {
						ret =
						    copy_to_user((void __user *)
								 arg,
								 &dev->common_memory,
								 sizeof
								 (hb_vpu_drv_buffer_t));
						if (ret == 0) {
							/* success to get memory for common memory */
							vpu_debug(5,
								  "[-]VDI_IOCTL_GET_COMMON_MEMORY\n");
							break;
						}
					}
				}
				ret = -EFAULT;
			}
			vpu_debug(5, "[-]VDI_IOCTL_GET_COMMON_MEMORY\n");
		}
		break;

	case VDI_IOCTL_OPEN_INSTANCE:
		{
			hb_vpu_drv_inst_t inst_info;
			hb_vpu_instance_list_t *vil, *vil_tmp, *n;
			vpu_debug(5, "[+]VDI_IOCTL_OPEN_INSTANCE");

			vil = kzalloc(sizeof(*vil), GFP_KERNEL);
			if (!vil)
				return -ENOMEM;

			if (copy_from_user
			    (&inst_info, (hb_vpu_drv_inst_t *) arg,
			     sizeof(hb_vpu_drv_inst_t))) {
				kfree(vil);
				return -EFAULT;
			}
			if (inst_info.core_idx >= MAX_NUM_VPU_CORE
			    || inst_info.inst_idx >= MAX_NUM_VPU_INSTANCE) {
				kfree(vil);
				return -EINVAL;
			}

			vil->inst_idx = inst_info.inst_idx;
			vil->core_idx = inst_info.core_idx;
			vil->filp = filp;

			spin_lock(&dev->vpu_spinlock);

			list_for_each_entry_safe(vil_tmp, n,
						 &dev->inst_list_head, list) {
				if (vil_tmp->inst_idx == inst_info.inst_idx
				    && vil_tmp->core_idx ==
				    inst_info.core_idx) {
					kfree(vil);
					vpu_err
					    ("Failed to open instance due to same id(%d, %d)",
					     (int)inst_info.core_idx,
					     (int)inst_info.inst_idx);
					spin_unlock(&dev->vpu_spinlock);
					return -EINVAL;
				}
			}
			list_add(&vil->list, &dev->inst_list_head);

			/* counting the current open instance number */
			inst_info.inst_open_count = 0;
			list_for_each_entry_safe(vil, n, &dev->inst_list_head,
						 list) {
				if (vil->core_idx == inst_info.core_idx)
					inst_info.inst_open_count++;
			}
#ifdef SUPPORT_MULTI_INST_INTR
			kfifo_reset(&dev->interrupt_pending_q
				    [inst_info.inst_idx]);
#endif
			spin_unlock(&dev->vpu_spinlock);

			/* flag just for that vpu is in opened or closed */
			dev->vpu_open_ref_count++;

			if (copy_to_user((void __user *)arg, &inst_info,
					 sizeof(hb_vpu_drv_inst_t))) {
				kfree(vil);
				return -EFAULT;
			}

			vpu_debug(5, "[-]VDI_IOCTL_OPEN_INSTANCE core_idx=%d, "
				  "inst_idx=%d, s_vpu_open_ref_count=%d, inst_open_count=%d\n",
				  (int)inst_info.core_idx,
				  (int)inst_info.inst_idx,
				  dev->vpu_open_ref_count,
				  inst_info.inst_open_count);
		}
		break;

	case VDI_IOCTL_CLOSE_INSTANCE:
		{
			hb_vpu_drv_inst_t inst_info;
			hb_vpu_instance_list_t *vil, *n;
			u32 found = 0;

			vpu_debug(5, "[+]VDI_IOCTL_CLOSE_INSTANCE\n");
			if (copy_from_user
			    (&inst_info, (hb_vpu_drv_inst_t *) arg,
			     sizeof(hb_vpu_drv_inst_t)))
				return -EFAULT;
			if (inst_info.core_idx >= MAX_NUM_VPU_CORE
			    || inst_info.inst_idx >= MAX_NUM_VPU_INSTANCE)
				return -EINVAL;

			spin_lock(&dev->vpu_spinlock);
			list_for_each_entry_safe(vil, n, &dev->inst_list_head,
						 list) {
				if (vil->inst_idx == inst_info.inst_idx
				    && vil->core_idx == inst_info.core_idx) {
					list_del(&vil->list);
					kfree(vil);
					found = 1;
					break;
				}
			}

			if (0 == found) {
				spin_unlock(&dev->vpu_spinlock);
				return -EINVAL;
			}

			// TODO can't find inst_idex, return false?
			/* counting the current open instance number */
			inst_info.inst_open_count = 0;
			list_for_each_entry_safe(vil, n, &dev->inst_list_head,
						 list) {
				if (vil->core_idx == inst_info.core_idx)
					inst_info.inst_open_count++;
			}
#ifdef SUPPORT_MULTI_INST_INTR
			kfifo_reset(&dev->interrupt_pending_q
				    [inst_info.inst_idx]);
#endif
			spin_unlock(&dev->vpu_spinlock);

			spin_lock(&dev->poll_spinlock);
			dev->poll_event[inst_info.inst_idx] = VPU_INST_CLOSED;
			spin_unlock(&dev->poll_spinlock);
			wake_up_interruptible(&dev->poll_wait_q[inst_info.inst_idx]);

			/* flag just for that vpu is in opened or closed */
			dev->vpu_open_ref_count--;

			if (copy_to_user((void __user *)arg, &inst_info,
					 sizeof(hb_vpu_drv_inst_t)))
				return -EFAULT;

			vpu_debug(5, "[-]VDI_IOCTL_CLOSE_INSTANCE core_idx=%d, "
				  "inst_idx=%d, s_vpu_open_ref_count=%d, inst_open_count=%d\n",
				  (int)inst_info.core_idx,
				  (int)inst_info.inst_idx,
				  dev->vpu_open_ref_count,
				  inst_info.inst_open_count);
		}
		break;
	case VDI_IOCTL_GET_INSTANCE_NUM:
		{
			hb_vpu_drv_inst_t inst_info;
			hb_vpu_instance_list_t *vil, *n;
			vpu_debug(5, "[+]VDI_IOCTL_GET_INSTANCE_NUM\n");

			ret =
			    copy_from_user(&inst_info,
					   (hb_vpu_drv_inst_t *) arg,
					   sizeof(hb_vpu_drv_inst_t));
			if (ret != 0) {
				ret = -EFAULT;
				break;
			}

			spin_lock(&dev->vpu_spinlock);
			inst_info.inst_open_count = 0;
			list_for_each_entry_safe(vil, n, &dev->inst_list_head,
						 list) {
				if (vil->core_idx == inst_info.core_idx)
					inst_info.inst_open_count++;
			}
			spin_unlock(&dev->vpu_spinlock);

			ret = copy_to_user((void __user *)arg, &inst_info,
					   sizeof(hb_vpu_drv_inst_t));

			vpu_debug(5, "VDI_IOCTL_GET_INSTANCE_NUM core_idx=%d, "
				  "inst_idx=%d, open_count=%d\n",
				  (int)inst_info.core_idx,
				  (int)inst_info.inst_idx,
				  inst_info.inst_open_count);
		}
		break;

	case VDI_IOCTL_RESET:
		{
			vpu_debug(5, "[+]VDI_IOCTL_RESET\n");
			hb_vpu_hw_reset();
			vpu_debug(5, "[-]VDI_IOCTL_RESET\n");
		}
		break;

	case VDI_IOCTL_GET_REGISTER_INFO:
		{
			hb_vpu_drv_buffer_t reg_buf;
			vpu_debug(5, "[+]VDI_IOCTL_GET_REGISTER_INFO\n");
			reg_buf.phys_addr = dev->vpu_mem->start;
			reg_buf.virt_addr = (unsigned long)dev->regs_base;
			reg_buf.size = resource_size(dev->vpu_mem);
			ret = copy_to_user((void __user *)arg, &reg_buf,
					   sizeof(hb_vpu_drv_buffer_t));
			if (ret != 0)
				ret = -EFAULT;
			vpu_debug(5, "[-]VDI_IOCTL_GET_REGISTER_INFO "
				  "vpu_register.phys_addr==0x%lx, s_vpu_register.virt_addr=0x%lx,"
				  "s_vpu_register.size=%d\n", reg_buf.phys_addr,
				  reg_buf.virt_addr, reg_buf.size);
		}
		break;
	case VDI_IOCTL_ALLOCATE_INSTANCE_ID:
		{
			vpu_debug(5, "[+]VDI_IOCTL_ALLOCATE_INSTANCE_ID\n");
			spin_lock(&dev->vpu_spinlock);
			inst_index =
			    find_first_zero_bit(vpu_inst_bitmap,
						MAX_NUM_VPU_INSTANCE);
			if (inst_index < MAX_NUM_VPU_INSTANCE) {
				set_bit(inst_index, vpu_inst_bitmap);
				spin_lock(&dev->poll_spinlock);
				dev->poll_event[inst_index] = VPU_EVENT_NONE;
				spin_unlock(&dev->poll_spinlock);
			} else {
				inst_index = -1;
			}
			priv->inst_index = inst_index; // it's useless
			spin_unlock(&dev->vpu_spinlock);

			ret =
			    copy_to_user((void __user *)arg, &inst_index,
					 sizeof(int));
			if (ret != 0)
				ret = -EFAULT;
			vpu_debug(5,
				  "[-]VDI_IOCTL_ALLOCATE_INSTANCE_ID id = %d\n",
				  inst_index);
		}
		break;
	case VDI_IOCTL_FREE_INSTANCE_ID:
		{
			vpu_debug(5, "[+]VDI_IOCTL_FREE_INSTANCE_ID\n");
			ret =
			    copy_from_user(&inst_index, (int *)arg,
					   sizeof(int));
			if (ret != 0
			    || (inst_index < 0
				|| inst_index >= MAX_NUM_VPU_INSTANCE)) {
				vpu_err
				    ("VDI_IOCTL_FREE_INSTANCE_ID invalid instance id.");
				return -EFAULT;
			}
			spin_lock(&dev->vpu_spinlock);
			clear_bit(inst_index, vpu_inst_bitmap);
			memset(&dev->vpu_ctx[inst_index], 0x00,
				sizeof(dev->vpu_ctx[inst_index]));
			memset(&dev->vpu_status[inst_index], 0x00,
				sizeof(dev->vpu_status[inst_index]));
			spin_unlock(&dev->vpu_spinlock);

			vpu_debug(5,
				"[-]VDI_IOCTL_FREE_INSTANCE_ID clear id = %d\n",
				inst_index);
		}
		break;

		case VDI_IOCTL_POLL_WAIT_INSTANCE: {
			hb_vpu_drv_intr_t info;
			hb_vpu_priv_t *priv;
			u32 intr_inst_index;
			//vpu_debug(5, "[+]VDI_IOCTL_POLL_WAIT_INSTANCE\n");

			ret = copy_from_user(&info, (hb_vpu_drv_intr_t *) arg,
						 sizeof(hb_vpu_drv_intr_t));
			if (ret != 0) {
				vpu_err
					("JDI_IOCTL_POLL_WAIT_INSTANCE copy from user fail.\n");
				return -EFAULT;
			}
			intr_inst_index = info.intr_inst_index;
			priv = filp->private_data;
			if (intr_inst_index >= 0 &&
				intr_inst_index < MAX_NUM_VPU_INSTANCE) {
				if (info.intr_reason == 0) {
					priv->inst_index = intr_inst_index;
				} else if (info.intr_reason == VPU_ENC_PIC_DONE ||
					info.intr_reason == VPU_DEC_PIC_DONE ||
					info.intr_reason == VPU_INST_CLOSED) {
					spin_lock(&dev->poll_spinlock);
					dev->poll_event[intr_inst_index] = VPU_ENC_PIC_DONE;
					spin_unlock(&dev->poll_spinlock);
					wake_up_interruptible(&dev->poll_wait_q[intr_inst_index]);
				} else {
					vpu_err
						("VDI_IOCTL_POLL_WAIT_INSTANCE invalid instance reason"
						"(%d) or index(%d).\n",
						info.intr_reason, intr_inst_index);
					return -EINVAL;
				}
			} else {
				return -EINVAL;
			}
			//vpu_debug(5, "[-]VDI_IOCTL_POLL_WAIT_INSTANCE\n");
		}
		break;
		case VDI_IOCTL_SET_CTX_INFO: {
				hb_vpu_ctx_info_t info;
				//vpu_debug(5, "[+]VDI_IOCTL_SET_CTX_INFO\n");
				ret = copy_from_user(&info, (hb_vpu_ctx_info_t *) arg,
							 sizeof(hb_vpu_ctx_info_t));
				if (ret != 0) {
					vpu_err
						("VDI_IOCTL_SET_CTX_INFO copy from user fail.\n");
					return -EFAULT;
				}
				inst_index = info.context.instance_index;
				if (inst_index < 0 || inst_index >= MAX_NUM_VPU_INSTANCE) {
					vpu_err
						("Invalid instance index %d.\n", inst_index);
					return -EINVAL;
				}
				spin_lock(&dev->vpu_info_spinlock);
				dev->vpu_ctx[inst_index] = info;
				spin_unlock(&dev->vpu_info_spinlock);
				//vpu_debug(5, "[-]VDI_IOCTL_SET_CTX_INFO\n");
				break;
			}
		case VDI_IOCTL_SET_STATUS_INFO: {
				//vpu_debug(5, "[+]VDI_IOCTL_SET_STATUS_INFO\n");
				hb_vpu_status_info_t info;
				ret = copy_from_user(&info, (hb_vpu_status_info_t *) arg,
							 sizeof(hb_vpu_status_info_t));
				if (ret != 0) {
					vpu_err
						("VDI_IOCTL_SET_STATUS_INFO copy from user fail.\n");
					return -EFAULT;
				}
				inst_index = info.inst_idx;
				if (inst_index < 0 || inst_index >= MAX_NUM_VPU_INSTANCE) {
					vpu_err
						("Invalid instance index %d.\n", inst_index);
					return -EINVAL;
				}
				spin_lock(&dev->vpu_info_spinlock);
				dev->vpu_status[inst_index] = info;
				spin_unlock(&dev->vpu_info_spinlock);
				//vpu_debug(5, "[-]VDI_IOCTL_SET_STATUS_INFO\n");
				break;
			}
	default:
		{
			vpu_err("No such IOCTL, cmd is %d\n", cmd);
		}
		break;
	}

	return ret;
}

static ssize_t vpu_read(struct file *filp, char __user * buf, size_t len,
			loff_t * ppos)
{
	vpu_debug_enter();
	vpu_debug_leave();
	return -1;
}

static ssize_t vpu_write(struct file *filp, const char __user * buf, size_t len,
			 loff_t * ppos)
{
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;

	vpu_debug_enter();
	priv = filp->private_data;
	dev = priv->vpu_dev;

	if (!dev) {
		vpu_err("failed to get vpu dev data");
		return -1;
	}

	/* DPRINTK("[VPUDRV] vpu_write len=%d\n", (int)len); */
	if (!buf) {
		vpu_err("vpu_write buf = NULL error \n");
		return -EFAULT;
	}

	if (len == sizeof(hb_vpu_drv_firmware_t)) {
		hb_vpu_drv_firmware_t *bit_firmware_info;

		bit_firmware_info =
		    kmalloc(sizeof(hb_vpu_drv_firmware_t), GFP_KERNEL);
		if (!bit_firmware_info) {
			vpu_err
			    ("vpu_write bit_firmware_info allocation error \n");
			return -EFAULT;
		}

		if (copy_from_user(bit_firmware_info, buf, len)) {
			vpu_err("vpu_write copy_from_user error "
				"for bit_firmware_info\n");
			kfree(bit_firmware_info);
			return -EFAULT;
		}

		if (bit_firmware_info->size == sizeof(hb_vpu_drv_firmware_t)) {
			vpu_debug(5,
				  "vpu_write set bit_firmware_info coreIdx=0x%x, "
				  "reg_base_offset=0x%x size=0x%x, bit_code[0]=0x%x\n",
				  bit_firmware_info->core_idx,
				  (int)bit_firmware_info->reg_base_offset,
				  bit_firmware_info->size,
				  bit_firmware_info->bit_code[0]);

			if (bit_firmware_info->core_idx > MAX_NUM_VPU_CORE) {
				vpu_err
				    ("vpu_write coreIdx[%d] is exceeded than "
				     "MAX_NUM_VPU_CORE[%d]\n",
				     bit_firmware_info->core_idx,
				     MAX_NUM_VPU_CORE);
				return -ENODEV;
			}

			memcpy((void *)
			       &dev->bit_fm_info[bit_firmware_info->core_idx],
			       bit_firmware_info,
			       sizeof(hb_vpu_drv_firmware_t));
			kfree(bit_firmware_info);
			vpu_debug_leave();
			return len;
		}

		kfree(bit_firmware_info);
	}

	vpu_err("vpu_write wrong length \n");
	return -1;
}

static int vpu_release(struct inode *inode, struct file *filp)
{
	int ret = 0, j = 0;
	u32 open_count;
#ifdef SUPPORT_MULTI_INST_INTR
	int i;
#endif
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;
	vpu_debug_enter();
	dev = container_of(inode->i_cdev, hb_vpu_dev_t, cdev);
	if (!dev) {
		vpu_err("failed to get vpu dev data");
		return -1;
	}
	priv = filp->private_data;
	hb_vpu_clk_disable(dev);

	if ((ret = down_interruptible(&dev->vpu_sem)) == 0) {
		/* found and free the not handled buffer by user applications */
		spin_lock(&dev->vpu_spinlock);	//check this place

		vpu_free_buffers(filp);

#ifdef USE_MUTEX_IN_KERNEL_SPACE
		for (j = 0; j < VPUDRV_MUTEX_MAX; j++) {
			if (dev->current_vdi_lock_pid[j] == current->tgid) {
				vpu_debug(5, "MUTEX_TYPE: %d, VDI_LOCK_PID: %d, current->pid: %d, "
					"current->tgid=%d\n", j, dev->current_vdi_lock_pid[j],
					current->pid, current->tgid);
				vdi_unlock(dev, j);
			}
		}
#endif

		/* found and free the not closed instance by user applications */
		vpu_free_instances(filp);
		dev->open_count--;
		open_count = dev->open_count;
		spin_unlock(&dev->vpu_spinlock);
		if (open_count == 0) {
			pm_qos_remove_request(&dev->vpu_pm_qos_req);

#ifdef SUPPORT_MULTI_INST_INTR
			for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
				kfifo_reset(&dev->interrupt_pending_q[i]);
			}
#endif
			if (dev->instance_pool.base) {
				vpu_debug(5, "free instance pool\n");
#ifdef USE_VMALLOC_FOR_INSTANCE_POOL_MEMORY
				vfree((const void *)dev->instance_pool.base);
#else
				vpu_free_dma_buffer(dev, &dev->instance_pool);
#endif
				dev->instance_pool.base = 0;
			}

			for (j = 0; j < MAX_NUM_VPU_INSTANCE; j++)
				test_and_clear_bit(j, vpu_inst_bitmap);	// TODO should clear bit during every close
#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
			for (j = 0; j < VPUDRV_MUTEX_MAX; j++) {
				if (dev->current_vdi_lock_pid[j] != 0) {
					vpu_debug(5, "RELEASE MUTEX_TYPE: %d, VDI_LOCK_PID: %d, "
						"current->pid: %d\n", j, dev->current_vdi_lock_pid[j],
						current->pid);
					vdi_unlock(dev, i);
				}
			}
#endif
		}
	}
	kfree(priv);
	up(&dev->vpu_sem);
	vpu_debug_leave();
	return 0;
}

static int vpu_fasync(int fd, struct file *filp, int mode)
{
	int ret = 0;
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;
	vpu_debug_enter();
	priv = filp->private_data;
	dev = priv->vpu_dev;
	if (!dev) {
		vpu_err("failed to get vpu dev data");
		return -1;
	}

	ret = fasync_helper(fd, filp, mode, &dev->async_queue);
	vpu_debug_leave();
	return ret;
}

static int vpu_map_to_register(struct file *filp, struct vm_area_struct *vm)
{
	unsigned long pfn;
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;
	int ret;
	vpu_debug_enter();

	if (!filp || !vm) {
		vpu_err("failed to map register, filp or vm is null.");
		return -1;
	}

	priv = filp->private_data;
	dev = priv->vpu_dev;

	if (!dev) {
		vpu_err("failed to map register, dev is null.");
		return -1;
	}

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = dev->vpu_mem->start >> PAGE_SHIFT;
	ret = remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			      vm->vm_page_prot) ? -EAGAIN : 0;
	vpu_debug_leave();
	return ret;
}

static int vpu_map_to_physical_memory(struct file *filp,
				      struct vm_area_struct *vm)
{
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;
	int ret;
	vpu_debug_enter();

	if (!filp || !vm) {
		vpu_err("failed to map register, filp or vm is null.");
		return -1;
	}

	priv = filp->private_data;
	dev = priv->vpu_dev;

	if (!dev) {
		vpu_err("failed to map register, dev is null.");
		return -1;
	}

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	ret = remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			      vm->vm_end - vm->vm_start,
			      vm->vm_page_prot) ? -EAGAIN : 0;
	vpu_debug_leave();
	return ret;
}

static int vpu_map_to_instance_pool_memory(struct file *filp,
					   struct vm_area_struct *vm)
{
#ifdef USE_VMALLOC_FOR_INSTANCE_POOL_MEMORY
	int ret;
	long length;
	unsigned long start;
	char *vmalloc_area_ptr;
	unsigned long pfn;
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;

	vpu_debug_enter();

	if (!filp || !vm) {
		vpu_err("failed to map instances, filp or vm is null.");
		return -1;
	}

	priv = filp->private_data;
	dev = priv->vpu_dev;

	if (!dev) {
		vpu_err("failed to map  instances, dev is null.");
		return -1;
	}

	length = vm->vm_end - vm->vm_start;
	start = vm->vm_start;
	vmalloc_area_ptr = (char *)dev->instance_pool.base;

	vm->vm_flags |= VM_RESERVED;

	/* loop over all pages, map it page individually */
	while (length > 0) {
		pfn = vmalloc_to_pfn(vmalloc_area_ptr);
		if ((ret =
		     remap_pfn_range(vm, start, pfn, PAGE_SIZE,
				     PAGE_SHARED)) < 0) {
			return ret;
		}
		start += PAGE_SIZE;
		vmalloc_area_ptr += PAGE_SIZE;
		length -= PAGE_SIZE;
	}
	vpu_debug_leave();

	return 0;
#else
	vpu_debug_enter();
	int ret;
	vm->vm_flags |= VM_RESERVED;
	ret = remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			      vm->vm_end - vm->vm_start,
			      vm->vm_page_prot) ? -EAGAIN : 0;
	vpu_debug_leave();
	return ret;
#endif
}

/*!
 * @brief memory map interface for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_mmap(struct file *filp, struct vm_area_struct *vm)
{
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;
	vpu_debug_enter();
	priv = filp->private_data;
	dev = priv->vpu_dev;

#ifdef USE_VMALLOC_FOR_INSTANCE_POOL_MEMORY
	if (vm->vm_pgoff == 0)
		return vpu_map_to_instance_pool_memory(filp, vm);

	if (vm->vm_pgoff == (dev->vpu_mem->start >> PAGE_SHIFT))
		return vpu_map_to_register(filp, vm);

	return vpu_map_to_physical_memory(filp, vm);
#else
	if (vm->vm_pgoff) {
		if (vm->vm_pgoff ==
		    (dev->instance_pool.phys_addr >> PAGE_SHIFT))
			return vpu_map_to_instance_pool_memory(filp, vm);

		return vpu_map_to_physical_memory(filp, vm);
	} else {
		return vpu_map_to_register(filp, vm);
	}
#endif
}

static unsigned int vpu_poll(struct file *filp, struct poll_table_struct *wait)
{
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;
	unsigned int mask = 0;

	priv = filp->private_data;
	dev = priv->vpu_dev;
	if (priv->inst_index < 0 || priv->inst_index >= MAX_NUM_VPU_INSTANCE) {
		return EPOLLERR;
	}
	poll_wait(filp, &dev->poll_wait_q[priv->inst_index], wait);
	spin_lock(&dev->poll_spinlock);
	if (VPU_ENC_PIC_DONE == dev->poll_event[priv->inst_index]
		|| VPU_DEC_PIC_DONE == dev->poll_event[priv->inst_index]) {
		mask = EPOLLIN | EPOLLET;
	} else if (VPU_INST_CLOSED == dev->poll_event[priv->inst_index]) {
		mask = EPOLLHUP;
	} else if (VPU_EVENT_NONE == dev->poll_event[priv->inst_index]) {
		mask = 0;
	} else {
		mask = EPOLLERR;
	}
	dev->poll_event[priv->inst_index] = VPU_EVENT_NONE;
	spin_unlock(&dev->poll_spinlock);
	return mask;
}

static struct file_operations vpu_fops = {
	.owner = THIS_MODULE,
	.open = vpu_open,
	.read = vpu_read,
	.write = vpu_write,
	/*.ioctl = vpu_ioctl, // for kernel 2.6.9 of C&M */
	.unlocked_ioctl = vpu_ioctl,
	.release = vpu_release,
	.fasync = vpu_fasync,
	.mmap = vpu_mmap,
	.poll = vpu_poll,
};

//////////////// venc
static char *get_profile(hb_vpu_ctx_info_t *vpu_ctx)
{
	if (vpu_ctx->context.codec_id == MEDIA_CODEC_ID_H264) {
		switch (vpu_ctx->context.video_enc_params.h264_enc_config.h264_profile)
		 {
			case MC_H264_PROFILE_BP:
				return "bp";
			case MC_H264_PROFILE_MP:
				return "mp";
			case MC_H264_PROFILE_EXTENDED:
				return "extended";
			case MC_H264_PROFILE_HP:
				return "hp";
			case MC_H264_PROFILE_HIGH10:
				return "high10";
			case MC_H264_PROFILE_HIGH422:
				return "high422";
			case MC_H264_PROFILE_HIGH444:
				return "high444";
			default:
				return "unspecified";
		}
	} else {
		return "unspecified";
	}
}

static char *get_level(hb_vpu_ctx_info_t *vpu_ctx)
{
	if (vpu_ctx->context.codec_id == MEDIA_CODEC_ID_H264) {
		switch (vpu_ctx->context.video_enc_params.h264_enc_config.h264_level) {
			case MC_H264_LEVEL1:
				return "level1";
			case MC_H264_LEVEL1b:
				return "level1b";
			case MC_H264_LEVEL1_1:
				return "level1_1";
			case MC_H264_LEVEL1_2:
				return "level1_2";
			case MC_H264_LEVEL1_3:
				return "level1_3";
			case MC_H264_LEVEL2:
				return "level2";
			case MC_H264_LEVEL2_1:
				return "level2_1";
			case MC_H264_LEVEL2_2:
				return "level2_2";
			case MC_H264_LEVEL3:
				return "level3";
			case MC_H264_LEVEL3_1:
				return "level3_1";
			case MC_H264_LEVEL3_2:
				return "level3_2";
			case MC_H264_LEVEL4:
				return "level4";
			case MC_H264_LEVEL4_1:
				return "level4_1";
			case MC_H264_LEVEL4_2:
				return "level4_2";
			case MC_H264_LEVEL5:
				return "level5";
			case MC_H264_LEVEL5_1:
				return "level5_1";
			case MC_H264_LEVEL5_2:
				return "level5_2";
			default:
				return "unspecified";
		}
	} else if (vpu_ctx->context.codec_id == MEDIA_CODEC_ID_H265) {
		switch (vpu_ctx->context.video_enc_params.h265_enc_config.h265_level) {
			case MC_H265_LEVEL1:
				return "level1";
			case MC_H265_LEVEL2:
				return "level2";
			case MC_H265_LEVEL2_1:
				return "level2_1";
			case MC_H265_LEVEL3:
				return "level3";
			case MC_H265_LEVEL3_1:
				return "level3_1";
			case MC_H265_LEVEL4:
				return "level4";
			case MC_H265_LEVEL4_1:
				return "level4_1";
			case MC_H265_LEVEL5:
				return "level5";
			case MC_H265_LEVEL5_1:
				return "level5_1";
			default:
				return "unspecified";
		}
	} else {
		return "---";
	}
}

static char *get_codec(hb_vpu_ctx_info_t *vpu_ctx)
{
	switch (vpu_ctx->context.codec_id) {
		case MEDIA_CODEC_ID_H264:
			return "h264";
		case MEDIA_CODEC_ID_H265:
			return "h265";
		case MEDIA_CODEC_ID_MJPEG:
			return "mjpg";
		case MEDIA_CODEC_ID_JPEG:
			return "jpeg";
		default:
			break;
	}
	return "unspecified";
}

static void rcparam_show(struct seq_file *s, hb_vpu_ctx_info_t *vpu_ctx) {
	mc_rate_control_params_t *rc =
		&(vpu_ctx->context.video_enc_params.rc_params);
	if (rc->mode == MC_AV_RC_MODE_H264CBR) {
		seq_printf(s, "%7d %7s %12d %8d %8d %10d %13d %15d %18d %8d %8d "
			"%8d %8d %8d %8d %13d %12d %13d %12d\n",
			vpu_ctx->context.instance_index,
			"h264cbr",
			rc->h264_cbr_params.intra_period,
			rc->h264_cbr_params.intra_qp,
			rc->h264_cbr_params.bit_rate,
			rc->h264_cbr_params.frame_rate,
			rc->h264_cbr_params.initial_rc_qp,
			rc->h264_cbr_params.vbv_buffer_size,
			rc->h264_cbr_params.mb_level_rc_enalbe,
			rc->h264_cbr_params.min_qp_I,
			rc->h264_cbr_params.max_qp_I,
			rc->h264_cbr_params.min_qp_P,
			rc->h264_cbr_params.max_qp_P,
			rc->h264_cbr_params.min_qp_B,
			rc->h264_cbr_params.max_qp_B,
			rc->h264_cbr_params.hvs_qp_enable,
			rc->h264_cbr_params.hvs_qp_scale,
			rc->h264_cbr_params.qp_map_enable,
			rc->h264_cbr_params.max_delta_qp);
	} else if (rc->mode == MC_AV_RC_MODE_H264VBR) {
		seq_printf(s, "%7d %7s %12d %8d %10d %13d\n",
			vpu_ctx->context.instance_index,
			"h264vbr",
			rc->h264_vbr_params.intra_period,
			rc->h264_vbr_params.intra_qp,
			rc->h264_vbr_params.frame_rate,
			rc->h264_vbr_params.qp_map_enable);
	} else if (rc->mode == MC_AV_RC_MODE_H264AVBR) {
		seq_printf(s, "%7d %8s %12d %8d %8d %10d %13d %15d %18d %8d %8d "
			"%8d %8d %8d %8d %13d %12d %13d %12d\n",
			vpu_ctx->context.instance_index,
			"h264avbr",
			rc->h264_avbr_params.intra_period,
			rc->h264_avbr_params.intra_qp,
			rc->h264_avbr_params.bit_rate,
			rc->h264_avbr_params.frame_rate,
			rc->h264_avbr_params.initial_rc_qp,
			rc->h264_avbr_params.vbv_buffer_size,
			rc->h264_avbr_params.mb_level_rc_enalbe,
			rc->h264_avbr_params.min_qp_I,
			rc->h264_avbr_params.max_qp_I,
			rc->h264_avbr_params.min_qp_P,
			rc->h264_avbr_params.max_qp_P,
			rc->h264_avbr_params.min_qp_B,
			rc->h264_avbr_params.max_qp_B,
			rc->h264_avbr_params.hvs_qp_enable,
			rc->h264_avbr_params.hvs_qp_scale,
			rc->h264_avbr_params.qp_map_enable,
			rc->h264_avbr_params.max_delta_qp);
	} else if (rc->mode == MC_AV_RC_MODE_H264FIXQP) {
		seq_printf(s, "%7d %9s %12d %10d %10d %10d %10d\n",
			vpu_ctx->context.instance_index,
			"h264fixqp",
			rc->h264_fixqp_params.intra_period,
			rc->h264_fixqp_params.frame_rate,
			rc->h264_fixqp_params.force_qp_I,
			rc->h264_fixqp_params.force_qp_P,
			rc->h264_fixqp_params.force_qp_B);
	} else if (rc->mode == MC_AV_RC_MODE_H264QPMAP) {
		seq_printf(s, "%7d %9s %12d %10d %18d\n",
			vpu_ctx->context.instance_index,
			"h264qpmap",
			rc->h264_qpmap_params.intra_period,
			rc->h264_qpmap_params.frame_rate,
			rc->h264_qpmap_params.qp_map_array_count);
	} else if (rc->mode == MC_AV_RC_MODE_H265CBR) {
		seq_printf(s, "%7d %7s %12d %8d %8d %10d %13d %15d %19d %8d %8d "
			"%8d %8d %8d %8d %13d %12d %13d %12d\n",
			vpu_ctx->context.instance_index,
			"h265cbr",
			rc->h265_cbr_params.intra_period,
			rc->h265_cbr_params.intra_qp,
			rc->h265_cbr_params.bit_rate,
			rc->h265_cbr_params.frame_rate,
			rc->h265_cbr_params.initial_rc_qp,
			rc->h265_cbr_params.vbv_buffer_size,
			rc->h265_cbr_params.ctu_level_rc_enalbe,
			rc->h265_cbr_params.min_qp_I,
			rc->h265_cbr_params.max_qp_I,
			rc->h265_cbr_params.min_qp_P,
			rc->h265_cbr_params.max_qp_P,
			rc->h265_cbr_params.min_qp_B,
			rc->h265_cbr_params.max_qp_B,
			rc->h265_cbr_params.hvs_qp_enable,
			rc->h265_cbr_params.hvs_qp_scale,
			rc->h265_cbr_params.qp_map_enable,
			rc->h265_cbr_params.max_delta_qp);
	} else if (rc->mode == MC_AV_RC_MODE_H265VBR) {
		seq_printf(s, "%7d %7s %12d %8d %10d %13d\n",
			vpu_ctx->context.instance_index,
			"h265vbr",
			rc->h265_vbr_params.intra_period,
			rc->h265_vbr_params.intra_qp,
			rc->h265_vbr_params.frame_rate,
			rc->h265_vbr_params.qp_map_enable);
	} else if (rc->mode == MC_AV_RC_MODE_H265AVBR) {
		seq_printf(s, "%7d %7s %12d %8d %8d %10d %13d %15d %19d %8d %8d "
			"%8d %8d %8d %8d %13d %12d %13d %12d\n",
			vpu_ctx->context.instance_index,
			"h265avbr",
			rc->h265_avbr_params.intra_period,
			rc->h265_avbr_params.intra_qp,
			rc->h265_avbr_params.bit_rate,
			rc->h265_avbr_params.frame_rate,
			rc->h265_avbr_params.initial_rc_qp,
			rc->h265_avbr_params.vbv_buffer_size,
			rc->h265_avbr_params.ctu_level_rc_enalbe,
			rc->h265_avbr_params.min_qp_I,
			rc->h265_avbr_params.max_qp_I,
			rc->h265_avbr_params.min_qp_P,
			rc->h265_avbr_params.max_qp_P,
			rc->h265_avbr_params.min_qp_B,
			rc->h265_avbr_params.max_qp_B,
			rc->h265_avbr_params.hvs_qp_enable,
			rc->h265_avbr_params.hvs_qp_scale,
			rc->h265_avbr_params.qp_map_enable,
			rc->h265_avbr_params.max_delta_qp);
	} else if (rc->mode == MC_AV_RC_MODE_H265FIXQP) {
		seq_printf(s, "%7d %9s %12d %10d %10d %10d %10d\n",
			vpu_ctx->context.instance_index,
			"h265fixqp",
			rc->h265_fixqp_params.intra_period,
			rc->h265_fixqp_params.frame_rate,
			rc->h265_fixqp_params.force_qp_I,
			rc->h265_fixqp_params.force_qp_P,
			rc->h265_fixqp_params.force_qp_B);
	} else if (rc->mode == MC_AV_RC_MODE_H265QPMAP) {
		seq_printf(s, "%7d %9s %12d %10d %18d\n",
			vpu_ctx->context.instance_index,
			"h265qpmap",
			rc->h265_qpmap_params.intra_period,
			rc->h265_qpmap_params.frame_rate,
			rc->h265_qpmap_params.qp_map_array_count);
	}

	seq_printf(s, "\n");
}

static int vpu_venc_show(struct seq_file *s, void *unused)
{
	int i;
	int output = 0;
	mc_video_gop_params_t *gop = NULL;
	mc_video_slice_params_t *slice_params = NULL;
	mc_h264_entropy_params_t *entropy_params = NULL;
	hb_vpu_dev_t *dev = (hb_vpu_dev_t *)s->private;
	if (dev == NULL)
		return 0;

	// seq_printf(s, "-----------------------venc-------------------------\n");
	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "----encode enc param----\n");
				seq_printf(s, "%7s %7s %11s %11s %5s %6s %7s %10s %15s %11s "
				"%10s %6s %6s\n", "enc_idx", "enc_id", "profile", "level",
				"width", "height", "pix_fmt", "fbuf_count", "extern_buf_flag",
				"bsbuf_count", "bsbuf_size", "mirror", "rotate");
			}
			seq_printf(s, "%7d %7s %11s %11s %5d %6d %7d "\
				"%10d %15d %11d %10d %6d %6d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				get_profile(&dev->vpu_ctx[i]),
				get_level(&dev->vpu_ctx[i]),
				dev->vpu_ctx[i].context.video_enc_params.width,
				dev->vpu_ctx[i].context.video_enc_params.height,
				dev->vpu_ctx[i].context.video_enc_params.pix_fmt,
				dev->vpu_ctx[i].context.video_enc_params.frame_buf_count,
				dev->vpu_ctx[i].context.video_enc_params.external_frame_buf,
				dev->vpu_ctx[i].context.video_enc_params.bitstream_buf_count,
				dev->vpu_ctx[i].context.video_enc_params.bitstream_buf_size,
				dev->vpu_ctx[i].context.video_enc_params.mir_direction,
				dev->vpu_ctx[i].context.video_enc_params.rot_degree);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H264CBR)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264cbr param----\n");
				seq_printf(s, "%7s %7s %12s %8s %8s %10s %13s %15s %18s %8s "
					"%8s %8s %8s %8s %8s %13s %12s %13s %12s\n",
					"enc_idx", "rc_mode", "intra_period", "intra_qp",
					"bit_rate",	"frame_rate", "initial_rc_qp",
					"vbv_buffer_size", "mb_level_rc_enalbe", "min_qp_I",
					"max_qp_I", "min_qp_P", "max_qp_P", "min_qp_B", "max_qp_B",
					"hvs_qp_enable", "hvs_qp_scale",
					"qp_map_enable", "max_delta_qp");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H264VBR)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264vbr param----\n");
				seq_printf(s, "%7s %7s %12s %8s %10s %13s\n",
					"enc_idx", "rc_mode", "intra_period", "intra_qp",
					"frame_rate", "qp_map_enable\n");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H264AVBR)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264avbr param----\n");
				seq_printf(s, "%7s %8s %12s %8s %8s %10s %13s %15s %18s %8s "
					"8s %8s %8s %8s %8s %13s %12s %13s %12s\n",
					"enc_idx", "rc_mode", "intra_period", "intra_qp", "bit_rate"
					"frame_rate", "initial_rc_qp", "vbv_buffer_size",
					"mb_level_rc_enalbe", "min_qp_I", "max_qp_I", "min_qp_P",
					"max_qp_P", "min_qp_B", "max_qp_B",	"hvs_qp_enable",
					"hvs_qp_scale", "qp_map_enable", "max_delta_qp");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H264FIXQP)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264fixqp param----\n");
				seq_printf(s, "%7s %9s %12s %10s %10s %10s %10s\n",
					"enc_idx", "rc_mode", "intra_period", "frame_rate",
					"force_qp_I", "force_qp_P", "force_qp_B");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H264QPMAP)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264qpmap param----\n");
				seq_printf(s, "%7s %9s %12s %10s %18s\n", "enc_idx", "rc_mode",
					"intra_period", "frame_rate", "qp_map_array_count");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H265CBR)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265cbr param----\n");
				seq_printf(s, "%7s %7s %12s %8s %8s %10s %13s %15s %19s %8s "
					"%8s %8s %8s %8s %8s %13s %12s %13s %12s\n",
					"enc_idx", "rc_mode", "intra_period", "intra_qp",
					"bit_rate",	"frame_rate", "initial_rc_qp",
					"vbv_buffer_size", "ctu_level_rc_enalbe", "min_qp_I",
					"max_qp_I", "min_qp_P", "max_qp_P", "min_qp_B", "max_qp_B",
					"hvs_qp_enable", "hvs_qp_scale",
					"qp_map_enable", "max_delta_qp");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H265VBR)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265vbr param----\n");
				seq_printf(s, "%7s %7s %12s %8s %10s %13s\n",
					"enc_idx", "rc_mode", "intra_period", "intra_qp",
					"frame_rate", "qp_map_enable\n");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H265AVBR)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265avbr param----\n");
				seq_printf(s, "%7s %7s %12s %8s %8s %10s %13s %15s %19s %8s "
					"%8s %8s %8s %8s %8s %13s %12s %13s %12s\n",
					"enc_idx", "rc_mode", "intra_period", "intra_qp",
					"bit_rate",	"frame_rate", "initial_rc_qp",
					"vbv_buffer_size", "ctu_level_rc_enalbe", "min_qp_I",
					"max_qp_I", "min_qp_P", "max_qp_P", "min_qp_B", "max_qp_B",
					"hvs_qp_enable", "hvs_qp_scale",
					"qp_map_enable", "max_delta_qp");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H265FIXQP)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265fixqp param----\n");
				seq_printf(s, "%7s %9s %12s %10s %10s %10s %10s\n",
					"enc_idx", "rc_mode", "intra_period", "frame_rate",
					"force_qp_I", "force_qp_P", "force_qp_B");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.video_enc_params.rc_params.mode
				== MC_AV_RC_MODE_H265QPMAP)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265qpmap param----\n");
				seq_printf(s, "%7s %9s %12s %10s %18s\n", "enc_idx", "rc_mode",
					"intra_period", "frame_rate", "qp_map_array_count");
			}
			rcparam_show(s, &(dev->vpu_ctx[i]));
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode gop param----\n");
				seq_printf(s, "%7s %7s %14s %15s %21s\n", "enc_idx", "enc_id",
				"gop_preset_idx", "custom_gop_size", "decoding_refresh_type");
			}
			gop =
				&(dev->vpu_ctx[i].context.video_enc_params.gop_params);
			seq_printf(s, "%7d %7s %14d %15d %21d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				gop->gop_preset_idx,
				gop->custom_gop_size,
				gop->decoding_refresh_type);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode intra refresh----\n");
				seq_printf(s, "%7s %7s %18s %17s\n", "enc_idx", "enc_id",
					"intra_refresh_mode", "intra_refresh_arg");
			}
			seq_printf(s, "%7d %7s %18d %17d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				dev->vpu_ctx[i].intra_refr.intra_refresh_mode,
				dev->vpu_ctx[i].intra_refr.intra_refresh_arg);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode longterm ref----\n");
				seq_printf(s, "%7s %7s %12s %19s %25s\n", "enc_idx", "enc_id",
					"use_longterm", "longterm_pic_period",
					"longterm_pic_using_period");
			}
			seq_printf(s, "%7d %7s %12d %19d %25d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				dev->vpu_ctx[i].ref_mode.use_longterm,
				dev->vpu_ctx[i].ref_mode.longterm_pic_period,
				dev->vpu_ctx[i].ref_mode.longterm_pic_using_period);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder) {
			mc_video_roi_params_t *roi_params =
						&(dev->vpu_ctx[i].roi_params);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode roi_params----\n");
				seq_printf(s, "%7s %7s %10s %19s\n", "enc_idx",
					"enc_id", "roi_enable", "roi_map_array_count");
			}
			seq_printf(s, "%7d %7s %10d %19d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				roi_params->roi_enable,
				roi_params->roi_map_array_count);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H265)) {
			mc_video_mode_decision_params_t *mode_decision =
						&(dev->vpu_ctx[i].mode_decision);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode mode_decision 1----\n");
				seq_printf(s, "%7s %7s %20s %15s %15s %15s %15s %28s "
					"%24s %27s %28s %24s %27s %28s %24s %27s\n", "enc_idx",
					"enc_id", "mode_decision_enable", "pu04_delta_rate",
					"pu08_delta_rate", "pu16_delta_rate",
					"pu32_delta_rate", "pu04_intra_planar_delta_rate",
					"pu04_intra_dc_delta_rate",
					"pu04_intra_angle_delta_rate",
					"pu08_intra_planar_delta_rate", "pu08_intra_dc_delta_rate",
					"pu08_intra_angle_delta_rate",
					"pu16_intra_planar_delta_rate", "pu16_intra_dc_delta_rate",
					"pu16_intra_angle_delta_rate");
			}
			seq_printf(s, "%7d %7s %20d %15d %15d %15d %15d %28d "
				"%24d %27d %28d %24d %27d %28d %24d %27d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				mode_decision->mode_decision_enable,
				mode_decision->pu04_delta_rate,
				mode_decision->pu08_delta_rate,
				mode_decision->pu16_delta_rate,
				mode_decision->pu32_delta_rate,
				mode_decision->pu04_intra_planar_delta_rate,
				mode_decision->pu04_intra_dc_delta_rate,
				mode_decision->pu04_intra_angle_delta_rate,
				mode_decision->pu08_intra_planar_delta_rate,
				mode_decision->pu08_intra_dc_delta_rate,
				mode_decision->pu08_intra_angle_delta_rate,
				mode_decision->pu16_intra_planar_delta_rate,
				mode_decision->pu16_intra_dc_delta_rate,
				mode_decision->pu16_intra_angle_delta_rate);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H265)) {
			mc_video_mode_decision_params_t *mode_decision =
						&(dev->vpu_ctx[i].mode_decision);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode mode_decision 2----\n");
				seq_printf(s, "%7s %7s %28s %24s %27s %21s %21s %21s "
					"%21s %21s %21s %21s %21s %21s\n", "enc_idx",
					"enc_id", "pu32_intra_planar_delta_rate",
					"pu32_intra_dc_delta_rate",
					"pu32_intra_angle_delta_rate", "cu08_intra_delta_rate",
					"cu08_inter_delta_rate", "cu08_merge_delta_rate",
					"cu16_intra_delta_rate", "cu16_inter_delta_rate",
					"cu16_merge_delta_rate", "cu32_intra_delta_rate",
					"cu32_inter_delta_rate", "cu32_merge_delta_rate");
			}
			seq_printf(s, "%7d %7s %28d %24d %27d %21d %21d %21d "
				"%21d %21d %21d %21d %21d %21d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				mode_decision->pu32_intra_planar_delta_rate,
				mode_decision->pu32_intra_dc_delta_rate,
				mode_decision->pu32_intra_angle_delta_rate,
				mode_decision->cu08_intra_delta_rate,
				mode_decision->cu08_inter_delta_rate,
				mode_decision->cu08_merge_delta_rate,
				mode_decision->cu16_intra_delta_rate,
				mode_decision->cu16_inter_delta_rate,
				mode_decision->cu16_merge_delta_rate,
				mode_decision->cu32_intra_delta_rate,
				mode_decision->cu32_inter_delta_rate,
				mode_decision->cu32_merge_delta_rate);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H264)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264 entropy params----\n");
				seq_printf(s, "%7s %7s %19s\n", "enc_idx",
					"enc_id", "entropy_coding_mode");
			}
			entropy_params =
					&(dev->vpu_ctx[i].entropy_params);
			seq_printf(s, "%7d %7s %19s\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				entropy_params->entropy_coding_mode == 0?"CAVLC":"CABAC");
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H264)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264 slice params----\n");
				seq_printf(s, "%7s %7s %15s %14s\n", "enc_idx", "enc_id",
					"h264_slice_mode", "h264_slice_arg");
			}
			slice_params =
					&(dev->vpu_ctx[i].slice_params);
			seq_printf(s, "%7d %7s %15d %14d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				slice_params->h264_slice.h264_slice_mode,
				slice_params->h264_slice.h264_slice_arg);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H264)) {
			mc_video_deblk_filter_params_t *deblk_filter =
					&(dev->vpu_ctx[i].deblk_filter);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264 deblk filter----\n");
				seq_printf(s, "%7s %7s %29s %26s %22s\n", "enc_idx",
					"enc_id", "disable_deblocking_filter_idc",
					"slice_alpha_c0_offset_div2", "slice_beta_offset_div2");
			}
			seq_printf(s, "%7d %7s %29d %26d %22d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				deblk_filter->h264_deblk.disable_deblocking_filter_idc,
				deblk_filter->h264_deblk.slice_alpha_c0_offset_div2,
				deblk_filter->h264_deblk.slice_beta_offset_div2);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H264)) {
			mc_h264_timing_params_t *timing =
					&(dev->vpu_ctx[i].vui_timing.h264_timing);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264 timing----\n");
				seq_printf(s, "%7s %7s %21s %14s %21s\n", "enc_idx",
					"enc_id", "vui_num_units_in_tick",
					"vui_time_scale", "fixed_frame_rate_flag");
			}
			seq_printf(s, "%7d %7s %21d %14d %21d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				timing->vui_num_units_in_tick,
				timing->vui_time_scale,
				timing->fixed_frame_rate_flag);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H264)) {
			mc_h264_intra_pred_params_t *intra_pred_params =
					&(dev->vpu_ctx[i].pred_unit.h264_intra_pred);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264_intra_pred----\n");
				seq_printf(s, "%7s %7s %27s\n", "enc_idx",
					"enc_id", "constrained_intra_pred_flag");
			}
			seq_printf(s, "%7d %7s %27d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				intra_pred_params->constrained_intra_pred_flag);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H264)) {
			mc_h264_transform_params_t *h264_transform =
					&(dev->vpu_ctx[i].transform_params.h264_transform);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h264_transform----\n");
				seq_printf(s, "%7s %7s %20s %19s %19s %24s\n", "enc_idx",
					"enc_id", "transform_8x8_enable", "chroma_cb_qp_offset",
					"chroma_cr_qp_offset", "user_scaling_list_enable");
			}
			seq_printf(s, "%7d %7s %20d %19d %19d %24d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				h264_transform->transform_8x8_enable,
				h264_transform->chroma_cb_qp_offset,
				h264_transform->chroma_cr_qp_offset,
				h264_transform->user_scaling_list_enable);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H265)) {
			mc_h265_transform_params_t *h265_transform =
					&(dev->vpu_ctx[i].transform_params.h265_transform);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265_transform----\n");
				seq_printf(s, "%7s %7s %19s %19s %24s\n", "enc_idx",
					"enc_id", "chroma_cb_qp_offset",
					"chroma_cr_qp_offset", "user_scaling_list_enable");
			}
			seq_printf(s, "%7d %7s %19d %19d %24d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				h265_transform->chroma_cb_qp_offset,
				h265_transform->chroma_cr_qp_offset,
				h265_transform->user_scaling_list_enable);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H265)) {
			mc_h265_pred_unit_params_t *pred_unit_params =
					&(dev->vpu_ctx[i].pred_unit.h265_pred_unit);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265_pred_unit----\n");
				seq_printf(s, "%7s %7s %16s %27s %35s %13s\n", "enc_idx",
					"enc_id", "intra_nxn_enable", "constrained_intra_pred_flag",
					"strong_intra_smoothing_enabled_flag", "max_num_merge");
			}
			seq_printf(s, "%7d %7s %16d %27d %35d %13d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				pred_unit_params->intra_nxn_enable,
				pred_unit_params->constrained_intra_pred_flag,
				pred_unit_params->strong_intra_smoothing_enabled_flag,
				pred_unit_params->max_num_merge);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H265)) {
			mc_h265_timing_params_t *timing =
					&(dev->vpu_ctx[i].vui_timing.h265_timing);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265 timing----\n");
				seq_printf(s, "%7s %7s %21s %14s %33s\n", "enc_idx",
					"enc_id", "vui_num_units_in_tick",
					"vui_time_scale", "vui_num_ticks_poc_diff_one_minus1");
			}
			seq_printf(s, "%7d %7s %21d %14d %33d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				timing->vui_num_units_in_tick,
				timing->vui_time_scale,
				timing->vui_num_ticks_poc_diff_one_minus1);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H265)) {
			mc_video_slice_params_t *slice_params =
					&(dev->vpu_ctx[i].slice_params);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265 slice params----\n");
				seq_printf(s, "%7s %7s %27s %26s %25s %24s\n",
					"enc_idx", "enc_id", "h265_independent_slice_mode",
					"h265_independent_slice_arg", "h265_dependent_slice_mode",
					"h265_dependent_slice_arg");
			}
			seq_printf(s, "%7d %7s %27d %26d %25d %24d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				slice_params->h265_slice.h265_independent_slice_mode,
				slice_params->h265_slice.h265_independent_slice_arg,
				slice_params->h265_slice.h265_dependent_slice_mode,
				slice_params->h265_slice.h265_dependent_slice_arg);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H265)) {
			mc_video_deblk_filter_params_t *deblk_filter =
					&(dev->vpu_ctx[i].deblk_filter);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265 deblk filter----\n");
				seq_printf(s, "%7s %7s %37s %22s %20s %44s\n", "enc_idx",
					"enc_id", "slice_deblocking_filter_disabled_flag",
					"slice_beta_offset_div2", "slice_tc_offset_div2",
					"slice_loop_filter_across_slices_enabled_flag");
			}
			seq_printf(s, "%7d %7s %37d %22d %20d %44d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				deblk_filter->h265_deblk.slice_deblocking_filter_disabled_flag,
				deblk_filter->h265_deblk.slice_beta_offset_div2,
				deblk_filter->h265_deblk.slice_tc_offset_div2,
		deblk_filter->h265_deblk.slice_loop_filter_across_slices_enabled_flag);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder &&
			(dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H265)) {
			mc_h265_sao_params_t *sao_params = &(dev->vpu_ctx[i].sao_params);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode h265 sao param----\n");
				seq_printf(s, "%7s %7s %30s\n", "enc_idx", "enc_id",
					"sample_adaptive_offset_enabled_flag\n");
			}
			seq_printf(s, "%7d %7s %30d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				sao_params->sample_adaptive_offset_enabled_flag);
		}
	}

	spin_lock(&dev->vpu_info_spinlock);
	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && dev->vpu_ctx[i].context.encoder) {
			mc_inter_status_t *status =	&(dev->vpu_status[i].status);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode status----\n");
				seq_printf(s, "%7s %7s %17s %18s %15s %14s %19s %20s\n",
					"enc_idx", "enc_id", "cur_input_buf_cnt",
					"cur_output_buf_cnt", "left_recv_frame", "left_enc_frame",
					"total_input_buf_cnt", "total_output_buf_cnt");
			}
			seq_printf(s, "%7d %7s %17d %18d %15d %14d %19d %20d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				status->cur_input_buf_cnt,
				status->cur_output_buf_cnt,
				status->left_recv_frame,
				status->left_enc_frame,
				status->total_input_buf_cnt,
				status->total_output_buf_cnt);
		}
	}
	spin_unlock(&dev->vpu_info_spinlock);

	return 0;
}

static int vpu_venc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vpu_venc_show, inode->i_private);
}

static const struct file_operations vpu_venc_fops = {
	.open = vpu_venc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//////////////// vdec
static int vpu_vdec_show(struct seq_file *s, void *unused)
{
	int i;
	int output = 0;
	hb_vpu_dev_t *dev = (hb_vpu_dev_t *)s->private;

	if (dev == NULL)
		return 0;

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && (dev->vpu_ctx[i].context.encoder == 0)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "----decode param----\n");
				seq_printf(s, "%7s %7s %9s %7s %18s %19s %15s\n",
					"dec_idx", "dec_id", "feed_mode", "pix_fmt",
					"bitstream_buf_size", "bitstream_buf_count",
					"frame_buf_count");
			}
			seq_printf(s, "%7d %7s %9d %7d %18d %19d %15d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				dev->vpu_ctx[i].context.video_dec_params.feed_mode,
				dev->vpu_ctx[i].context.video_dec_params.pix_fmt,
				dev->vpu_ctx[i].context.video_dec_params.bitstream_buf_size,
				dev->vpu_ctx[i].context.video_dec_params.bitstream_buf_count,
				dev->vpu_ctx[i].context.video_dec_params.frame_buf_count);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && (dev->vpu_ctx[i].context.encoder == 0)) {
			if (dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H264) {
				mc_h264_dec_config_t *h264_dec_config =
					&dev->vpu_ctx[i].context.video_dec_params.h264_dec_config;
				if (output == 0) {
					output = 1;
					seq_printf(s, "----h264 decode param----\n");
					seq_printf(s, "%7s %7s %14s %9s %13s\n",
						"dec_idx", "dec_id", "reorder_enable",
						"skip_mode", "bandwidth_Opt");
				}
				seq_printf(s, "%7d %7s %14d %9d %13d\n",
					dev->vpu_ctx[i].context.instance_index,
					get_codec(&dev->vpu_ctx[i]),
					h264_dec_config->reorder_enable,
					h264_dec_config->skip_mode,
					h264_dec_config->bandwidth_Opt);
			}
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && (dev->vpu_ctx[i].context.encoder == 0)) {
			if (dev->vpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_H265) {
				mc_h265_dec_config_t *h265_dec_config =
					&dev->vpu_ctx[i].context.video_dec_params.h265_dec_config;
				if (output == 0) {
					output = 1;
					seq_printf(s, "----h265 decode param----\n");
					seq_printf(s, "%7s %7s %14s %9s %13s %10s %20s %28s\n",
						"dec_idx", "dec_id", "reorder_enable", "skip_mode",
						"bandwidth_Opt", "cra_as_bla", "dec_temporal_id_mode",
						"target_dec_temporal_id_plus1");
				}
				seq_printf(s, "%7d %7s %14d %9d %13d %10d %20d %28d\n",
					dev->vpu_ctx[i].context.instance_index,
					get_codec(&dev->vpu_ctx[i]),
					h265_dec_config->reorder_enable,
					h265_dec_config->skip_mode,
					h265_dec_config->bandwidth_Opt,
					h265_dec_config->cra_as_bla,
					h265_dec_config->dec_temporal_id_mode,
					h265_dec_config->target_dec_temporal_id_plus1);
			}
		}
	}

	spin_lock(&dev->vpu_info_spinlock);
	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && (dev->vpu_ctx[i].context.encoder == 0)) {
			mc_h264_h265_output_frame_info_t *frameinfo
								= &(dev->vpu_status[i].frame_info);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n----decode frameinfo----\n");
				seq_printf(s, "%7s %7s %13s %14s\n", "dec_idx", "dec_id",
					"display_width", "display_height");
			}
			seq_printf(s, "%7d %7s %13d %14d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				frameinfo->display_width,
				frameinfo->display_height);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		if (dev->vpu_ctx[i].valid && (dev->vpu_ctx[i].context.encoder == 0)) {
			mc_inter_status_t *status =	&(dev->vpu_status[i].status);
			if (output == 0) {
				output = 1;
				seq_printf(s, "----decode status----\n");
				seq_printf(s, "%7s %7s %17s %18s %19s %20s\n", "dec_idx",
					"dec_id", "cur_input_buf_cnt", "cur_output_buf_cnt",
					"total_input_buf_cnt", "total_output_buf_cnt");
			}
			seq_printf(s, "%7d %7s %17d %18d %19d %20d\n",
				dev->vpu_ctx[i].context.instance_index,
				get_codec(&dev->vpu_ctx[i]),
				status->cur_input_buf_cnt,
				status->cur_output_buf_cnt,
				status->total_input_buf_cnt,
				status->total_output_buf_cnt);
		}
	}
	spin_unlock(&dev->vpu_info_spinlock);

	return 0;
}

static int vpu_vdec_open(struct inode *inode, struct file *file)
{
	return single_open(file, vpu_vdec_show, inode->i_private);
}

static const struct file_operations vpu_vdec_fops = {
	.open = vpu_vdec_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int vpu_probe(struct platform_device *pdev)
{
	hb_vpu_dev_t *dev = NULL;
	struct resource *res = NULL;
	int err = 0;
	int i;

	dev_dbg(&pdev->dev, "%s()\n", __func__);
	dev = devm_kzalloc(&pdev->dev, sizeof(hb_vpu_dev_t), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Not enough memory for VPU device.\n");
		err = -ENOMEM;
		goto ERR_RESOURSE;
	}
	dev->device = &pdev->dev;

	dev->plat_data = pdev->dev.platform_data;
	dev->plat_data =
	    devm_kzalloc(&pdev->dev, sizeof(hb_vpu_platform_data_t),
			 GFP_KERNEL);
	if (!dev->plat_data) {
		dev_err(&pdev->dev,
			"Not enough memory for VPU platform data\n");
		err = -ENOMEM;
		goto ERR_RESOURSE;
	}
	vpu_parse_dts(dev->device->of_node, dev);

	dev->drv_data = vpu_get_drv_data(pdev);

	err = hb_vpu_init_pm(dev->device);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to setup vpu clock & power\n");
		goto ERR_INIT_PM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get memory resource\n");
		err = -ENOENT;
		goto ERR_RES_MEM;
	}
	dev->vpu_mem = request_mem_region(res->start, resource_size(res),
					  pdev->name);
	if (!dev->vpu_mem) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		err = -ENOENT;
		goto ERR_REQ_MEM;
	}
	dev->regs_base = ioremap_nocache(dev->vpu_mem->start,
					 resource_size(dev->vpu_mem));
	if (!dev->regs_base) {
		dev_err(&pdev->dev, "failed to ioremap address region\n");
		err = -ENOENT;
		goto ERR_IO_REMAP;
	}
	dev_dbg(&pdev->dev,
		"vpu IO memory resource: physical base addr = 0x%llx,"
		"virtual base addr = %p\n", dev->vpu_mem->start,
		dev->regs_base);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get irq resource\n");
		err = -ENOENT;
		goto ERR_RES_IRQ;
	}
	dev->irq = res->start;
	// TODO Add top half irq and bottom half irq?
	err = request_threaded_irq(dev->irq, vpu_irq_handler, NULL,
				   IRQF_ONESHOT, pdev->name, dev);
	if (err) {
		dev_err(&pdev->dev,
			"failed to install register interrupt handler\n");
		goto ERR_REQ_IRQ;
	}
	dev_dbg(&pdev->dev, "vpu irq number: irq = %d\n", dev->irq);

	dev->vpu_class = class_create(THIS_MODULE, VPU_DEV_NAME);
	if (IS_ERR(dev->vpu_class)) {
		dev_err(&pdev->dev, "failed to create class\n");
		err = PTR_ERR(dev->vpu_class);
		goto ERR_CREATE_CLASS;
	}

	/* get the major number of the character device */
	err = alloc_chrdev_region(&dev->vpu_dev_num, 0, 1, VPU_DEV_NAME);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to allocate character device\n");
		goto ERR_ALLOCATE_CHR;
	} else {
		dev->major = MAJOR(dev->vpu_dev_num);
		dev->minor = MINOR(dev->vpu_dev_num);
	}
	dev_dbg(&pdev->dev, "vpu device number: major = %d, minor = %d\n",
		dev->major, dev->minor);

	/* initialize the device structure and register the device with the kernel */
	cdev_init(&dev->cdev, &vpu_fops);
	err = cdev_add(&dev->cdev, dev->vpu_dev_num, 1);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to add character device\n");
		goto ERR_ADD_CHR;
	}

	dev->vpu_dev = device_create(dev->vpu_class, NULL, dev->vpu_dev_num,
				     NULL, VPU_DEV_NAME);
	if (IS_ERR(dev->vpu_dev)) {
		err = PTR_ERR(dev->vpu_dev);
		dev->vpu_dev = NULL;
		goto ERR_CREATE_DEV;
	}

	platform_set_drvdata(pdev, dev);

	dev->vpu_freq = MAX_VPU_FREQ;
	err = hb_vpu_clk_get(dev, dev->vpu_freq);
	if (err < 0) {
		goto ERR_GET_CLK;
	}
	hb_vpu_clk_put(dev);

#ifdef CONFIG_ION_HOBOT
	dev->vpu_ion_client = ion_client_create(ion_exynos, "vpu");
	if (IS_ERR(dev->vpu_ion_client)) {
		dev_err(&pdev->dev, "failed to ion_client_create\n");
		err = PTR_ERR(dev->vpu_dev);
		goto ERR_ION_CLIENT;
	}
#endif

#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (s_vmem.base_addr == 0) {
		/// *6 For test FHD and UHD stream,
		s_video_memory.size = VPU_INIT_VIDEO_MEMORY_SIZE_IN_BYTE * 6;
		s_video_memory.phys_addr = VPU_DRAM_PHYSICAL_BASE;
		s_video_memory.base =
			(unsigned long)ioremap_nocache(s_video_memory.phys_addr,
			PAGE_ALIGN(s_video_memory.size));
		if (!s_video_memory.base) {
			dev_err(&pdev->dev,
				"fail to remap video memory physical phys_addr=0x%lx,"
				"base==0x%lx, size=%d\n",
				s_video_memory.phys_addr, s_video_memory.base,
				(int)s_video_memory.size);
			err = -ENOMEM;
			goto ERR_RESERVED_MEM;
		}

		if (vmem_init
		    (&s_vmem, s_video_memory.phys_addr,
		     s_video_memory.size) < 0) {
			err = -ENOMEM;
			dev_err(&pdev->dev, "fail to init vmem system\n");
			goto ERROR_INIT_VMEM;
		}
	}
	dev_dbg(&pdev->dev,
		"success to probe vpu device with reserved video memory"
		"phys_addr==0x%lx, base = =0x%lx\n", s_video_memory.phys_addr,
		s_video_memory.base);
#endif

	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		init_waitqueue_head(&dev->poll_wait_q[i]);
	}
	spin_lock_init(&dev->poll_spinlock);
#ifdef SUPPORT_MULTI_INST_INTR
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		init_waitqueue_head(&dev->interrupt_wait_q[i]);
	}

	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		err =
		    kfifo_alloc(&dev->interrupt_pending_q[i],
				MAX_INTERRUPT_QUEUE * sizeof(u32), GFP_KERNEL);
		if (err) {
			dev_err(&pdev->dev,
				"failed to do kfifo_alloc failed 0x%x\n", err);
			goto ERR_ALLOC_FIFO;
		}
	}
	spin_lock_init(&dev->vpu_kfifo_lock);
#else
	init_waitqueue_head(&dev->interrupt_wait_q);
#endif
	spin_lock_init(&dev->irq_spinlock);
	dev->irq_trigger = 0;

	dev->async_queue = NULL;
	dev->open_count = 0;
	mutex_init(&dev->vpu_mutex);
	sema_init(&dev->vpu_sem, 1);
	spin_lock_init(&dev->vpu_spinlock);
	spin_lock_init(&dev->vpu_info_spinlock);

	INIT_LIST_HEAD(&dev->vbp_head);
	INIT_LIST_HEAD(&dev->inst_list_head);

	dev->common_memory.size = SIZE_COMMON;
	err = vpu_alloc_dma_buffer(dev, &dev->common_memory);
	if (err) {
		dev->common_memory.size = 0;
		dev_err(&pdev->dev,
			"failed to allocate common memory 0x%x\n", err);
		goto ERR_ALLOC_FIFO;
	}

#ifdef USE_MUTEX_IN_KERNEL_SPACE
	for (i=0; i < VPUDRV_MUTEX_MAX; i++) {
		dev->current_vdi_lock_pid[i] = 0;
	}
	sema_init(&dev->vpu_vdi_sem, 1);
	sema_init(&dev->vpu_vdi_disp_sem, 1);
	sema_init(&dev->vpu_vdi_reset_sem, 1);
	sema_init(&dev->vpu_vdi_vmem_sem, 1);
#endif

	dev->debug_root = debugfs_create_dir("vpu", NULL);
	if (!dev->debug_root) {
		pr_err("hai: failed to create debugfs root directory.\n");
		goto ERR_ALLOC_FIFO;
	}

	dev->debug_file_venc = debugfs_create_file("venc", 0664,
						dev->debug_root,
						dev, &vpu_venc_fops);
	if (!dev->debug_file_venc) {
		char buf[256], *path;

		path = dentry_path_raw(dev->debug_root, buf, 256);
		pr_err("Failed to create client debugfs at %s/%s\n",
			path, "venc");
	}

	dev->debug_file_vdec = debugfs_create_file("vdec", 0664,
						dev->debug_root,
						dev, &vpu_vdec_fops);
	if (!dev->debug_file_vdec) {
		char buf[256], *path;

		path = dentry_path_raw(dev->debug_root, buf, 256);
		pr_err("Failed to create client debugfs at %s/%s\n",
			path, "vdec");
	}

	return 0;

ERR_ALLOC_FIFO:
#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	vmem_exit(&s_vmem);
ERROR_INIT_VMEM:
ERR_RESERVED_MEM:
#endif
#ifdef CONFIG_ION_HOBOT
	ion_client_destroy(dev->vpu_ion_client);
ERR_ION_CLIENT:
#endif
	hb_vpu_clk_put(dev);
ERR_GET_CLK:
	device_destroy(dev->vpu_class, dev->vpu_dev_num);
ERR_CREATE_DEV:
	cdev_del(&dev->cdev);
ERR_ADD_CHR:
	unregister_chrdev_region(dev->vpu_dev_num, 1);
ERR_ALLOCATE_CHR:
	class_destroy(dev->vpu_class);
ERR_CREATE_CLASS:
	free_irq(dev->irq, dev);
ERR_REQ_IRQ:
ERR_RES_IRQ:
	iounmap(dev->regs_base);
ERR_IO_REMAP:
	release_mem_region(dev->vpu_mem->start, resource_size(dev->vpu_mem));
ERR_REQ_MEM:
ERR_RES_MEM:
	hb_vpu_final_pm(dev->device);
ERR_INIT_PM:
ERR_RESOURSE:
	return err;
}

static int vpu_remove(struct platform_device *pdev)
{
#ifdef SUPPORT_MULTI_INST_INTR
	int i;
#endif
	hb_vpu_dev_t *dev;

	dev_dbg(&pdev->dev, "%s()\n", __func__);

	dev = platform_get_drvdata(pdev);

	debugfs_remove_recursive(dev->debug_file_venc);
	debugfs_remove_recursive(dev->debug_file_vdec);
	debugfs_remove_recursive(dev->debug_root);

	if (dev->instance_pool.base) {
#ifdef USE_VMALLOC_FOR_INSTANCE_POOL_MEMORY
		vfree((const void *)dev->instance_pool.base);
#else
		vpu_free_dma_buffer(dev, &dev->instance_pool);
#endif
		dev->instance_pool.base = 0;
	}

	if (dev->common_memory.base) {
		vpu_free_dma_buffer(dev, &dev->common_memory);
		dev->common_memory.base = 0;
	}
#ifdef SUPPORT_MULTI_INST_INTR
	for (i = 0; i < MAX_NUM_VPU_INSTANCE; i++) {
		kfifo_free(&dev->interrupt_pending_q[i]);
	}
#endif

#ifdef VPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (s_video_memory.base) {
		iounmap((void *)s_video_memory.base);
		s_video_memory.base = 0;
		vmem_exit(&s_vmem);
	}
#endif

#ifdef CONFIG_ION_HOBOT
	ion_client_destroy(dev->vpu_ion_client);
#endif

	//hb_vpu_clk_disable(dev);
	//hb_vpu_clk_put(dev);
	device_destroy(dev->vpu_class, dev->vpu_dev_num);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->vpu_dev_num, 1);
	class_destroy(dev->vpu_class);
	free_irq(dev->irq, dev);
	iounmap(dev->regs_base);
	release_mem_region(dev->vpu_mem->start, resource_size(dev->vpu_mem));
	hb_vpu_final_pm(dev->device);

	return 0;
}

#if CONFIG_PM_SLEEP
static int vpu_suspend(struct platform_device *pdev, pm_message_t state)
{
	int core;
#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
	int ret;
#else
	int i;
	/* vpu wait timeout to 1sec */
	unsigned long timeout = jiffies + HZ;
	int product_code;
#endif
	hb_vpu_dev_t *dev;

	vpu_debug_enter();
	dev = (hb_vpu_dev_t *) platform_get_drvdata(pdev);
	hb_vpu_clk_enable(dev, dev->vpu_freq);

	if (dev->vpu_open_ref_count > 0) {
		for (core = 0; core < MAX_NUM_VPU_CORE; core++) {
			if (dev->bit_fm_info[core].size == 0)
				continue;
#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
			ret = vpu_sleep_wake(dev, core, VPU_SLEEP_MODE);
			if (ret != VPUAPI_RET_SUCCESS) {
				goto DONE_SUSPEND;
			}
#else
			product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
			if (PRODUCT_CODE_W_SERIES(product_code)) {
				while (VPU_READL(W5_VPU_BUSY_STATUS)) {
					if (time_after(jiffies, timeout)) {
						vpu_debug(5,
							  "SLEEP_VPU BUSY timeout");
						goto DONE_SUSPEND;
					}
				}

				VPU_ISSUE_COMMAND(core, W5_CMD_SLEEP_VPU);

				while (VPU_READL(W5_VPU_BUSY_STATUS)) {
					if (time_after(jiffies, timeout)) {
						vpu_debug(5,
							  "SLEEP_VPU BUSY timeout");
						goto DONE_SUSPEND;
					}
				}

				if (VPU_READL(W5_RET_SUCCESS) == 0) {
					vpu_debug(5, "SLEEP_VPU failed [0x%x]",
						  VPU_READL
						  (W5_RET_FAIL_REASON));
					goto DONE_SUSPEND;
				}
			} else if (!PRODUCT_CODE_W_SERIES(product_code)) {
				while (VPU_READL(BIT_BUSY_FLAG)) {
					if (time_after(jiffies, timeout))
						goto DONE_SUSPEND;
				}

				for (i = 0; i < 64; i++) {
					dev->vpu_reg_store[core][i] =
					    VPU_READL(BIT_BASE +
						      (0x100 + (i * 4)));
				}
			} else {
				vpu_debug(5,
					  "[VPUDRV] Unknown product id : %08x\n",
					  product_code);
				goto DONE_SUSPEND;
			}
#endif
		}
	}

	hb_vpu_clk_disable(dev);
	vpu_debug_leave();
	return 0;

DONE_SUSPEND:
	hb_vpu_clk_disable(dev);
	vpu_debug_leave();
	return -EAGAIN;
}

static int vpu_resume(struct platform_device *pdev)
{
	int core;
#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
	int ret;
#endif
#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
#else
	int i;
	u32 val;
	unsigned long timeout = jiffies + HZ;	/* vpu wait timeout to 1sec */
	int product_code;

	unsigned long code_base;
	u32 code_size;
	u32 remap_size;
	int regVal;
	u32 hwOption = 0;
#endif
	hb_vpu_dev_t *dev;

	vpu_debug_enter();
	dev = (hb_vpu_dev_t *) platform_get_drvdata(pdev);
	hb_vpu_clk_enable(dev, dev->vpu_freq);

	for (core = 0; core < MAX_NUM_VPU_CORE; core++) {
		if (dev->bit_fm_info[core].size == 0) {
			continue;
		}
#ifdef USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE
		ret = vpu_sleep_wake(dev, core, VPU_WAKE_MODE);
		if (ret != VPUAPI_RET_SUCCESS) {
			goto DONE_WAKEUP;
		}
#else
		product_code = VPU_READL(VPU_PRODUCT_CODE_REGISTER);
		if (PRODUCT_CODE_W_SERIES(product_code)) {
			code_base = dev->common_memory.phys_addr;
			/* ALIGN TO 4KB */
			code_size = (W5_MAX_CODE_BUF_SIZE & ~0xfff);
			if (code_size < dev->bit_fm_info[core].size * 2) {
				goto DONE_WAKEUP;
			}

			regVal = 0;
			VPU_WRITEL(W5_PO_CONF, regVal);

			/* Reset All blocks */
			regVal = W5_RST_BLOCK_ALL;
			VPU_WRITEL(W5_VPU_RESET_REQ, regVal);

			/* Waiting reset done */
			while (VPU_READL(W5_VPU_RESET_STATUS)) {
				if (time_after(jiffies, timeout))
					goto DONE_WAKEUP;
			}

			VPU_WRITEL(W5_VPU_RESET_REQ, 0);

			/* remap page size */
			remap_size = (code_size >> 12) & 0x1ff;
			regVal =
			    0x80000000 | (W5_REMAP_CODE_INDEX << 12) | (0 << 16)
			    | (1 << 11) | remap_size;
			VPU_WRITEL(W5_VPU_REMAP_CTRL, regVal);
			VPU_WRITEL(W5_VPU_REMAP_VADDR, 0x00000000);	/* DO NOT CHANGE! */
			VPU_WRITEL(W5_VPU_REMAP_PADDR, code_base);
			VPU_WRITEL(W5_ADDR_CODE_BASE, code_base);
			VPU_WRITEL(W5_CODE_SIZE, code_size);
			VPU_WRITEL(W5_CODE_PARAM, 0);
			//VPU_WRITEL(W5_INIT_VPU_TIME_OUT_CNT, timeout);

			VPU_WRITEL(W5_HW_OPTION, hwOption);

			/* Interrupt */
			if (product_code == WAVE521_CODE ||
				product_code == WAVE521C_CODE) {
				regVal  = (1 << INT_WAVE5_ENC_SET_PARAM);
				regVal |= (1 << INT_WAVE5_ENC_PIC);
				regVal |= (1 << INT_WAVE5_INIT_SEQ);
				regVal |= (1 << INT_WAVE5_DEC_PIC);
				regVal |= (1 << INT_WAVE5_BSBUF_EMPTY);
			} else if (product_code == WAVE420_CODE) {
				regVal  = (1 << W4_INT_DEC_PIC_HDR);
				regVal |= (1 << W4_INT_DEC_PIC);
				regVal |= (1 << W4_INT_QUERY_DEC);
				regVal |= (1 << W4_INT_SLEEP_VPU);
				regVal |= (1 << W4_INT_BSBUF_EMPTY);
			} else {
				// decoder
				regVal  = (1 << INT_WAVE5_INIT_SEQ);
				regVal |= (1 << INT_WAVE5_DEC_PIC);
				regVal |= (1 << INT_WAVE5_BSBUF_EMPTY);
			}

			VPU_WRITEL(W5_VPU_VINT_ENABLE, regVal);

			VPU_ISSUE_COMMAND(core, W5_CMD_INIT_VPU);
			VPU_WRITEL(W5_VPU_REMAP_CORE_START, 1);

			while (VPU_READL(W5_VPU_BUSY_STATUS)) {
				if (time_after(jiffies, timeout))
					goto DONE_WAKEUP;
			}

			if (VPU_READL(W5_RET_SUCCESS) == 0) {
				vpu_err("WAKEUP_VPU failed [0x%x]",
					VPU_READL(W5_RET_FAIL_REASON));
				goto DONE_WAKEUP;
			}
		} else if (!PRODUCT_CODE_W_SERIES(product_code)) {
			VPU_WRITEL(BIT_CODE_RUN, 0);

			/*---- LOAD BOOT CODE*/
			for (i = 0; i < 512; i++) {
				val = dev->bit_fm_info[core].bit_code[i];
				VPU_WRITEL(BIT_CODE_DOWN, ((i << 16) | val));
			}

			for (i = 0; i < 64; i++)
				VPU_WRITEL(BIT_BASE + (0x100 + (i * 4)),
					   dev->vpu_reg_store[core][i]);

			VPU_WRITEL(BIT_BUSY_FLAG, 1);
			VPU_WRITEL(BIT_CODE_RESET, 1);
			VPU_WRITEL(BIT_CODE_RUN, 1);

			while (VPU_READL(BIT_BUSY_FLAG)) {
				if (time_after(jiffies, timeout))
					goto DONE_WAKEUP;
			}
		} else {
			vpu_err("[VPUDRV] Unknown product id : %08x\n",
				product_code);
			goto DONE_WAKEUP;
		}
#endif
	}

	if (dev->vpu_open_ref_count == 0)
		hb_vpu_clk_disable(dev);

DONE_WAKEUP:
	if (dev->vpu_open_ref_count > 0)
		hb_vpu_clk_enable(dev, dev->vpu_freq);

	vpu_debug_leave();
	return 0;
}
#else
#define    vpu_suspend    NULL
#define    vpu_resume    NULL
#endif /* !CONFIG_PM_SLEEP */

/* Power management */
/*static const struct dev_pm_ops vpu_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(vpu_suspend, vpu_resume)
};*/

// static const struct attribute_group *vpu_attr_groups[] = {};
static struct platform_driver vpu_driver = {
	.probe = vpu_probe,
	.remove = vpu_remove,
	.suspend = vpu_suspend,
	.resume = vpu_resume,
	.driver = {
		   .name = VPU_PLATFORM_DEVICE_NAME,
		   .of_match_table = vpu_of_match,
		   //.pm = &vpu_pm_ops
		   // .groups = vpu_attr_groups,
		   },
};

module_platform_driver(vpu_driver);

MODULE_AUTHOR("Hobot");
MODULE_DESCRIPTION("Hobot video processing unit linux driver");
MODULE_LICENSE("GPL v2");
