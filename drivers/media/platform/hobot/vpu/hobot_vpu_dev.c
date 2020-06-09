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
	void *vdi_mutexes_base;
	const int PTHREAD_MUTEX_T_DESTROY_VALUE = 0xdead10cc;
	hb_vpu_dev_t *dev;
	hb_vpu_priv_t *priv;
	unsigned long timeout = jiffies + HZ;
	int core;

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
			core = vil->core_idx;
			VPU_WRITEL(W5_CMD_INSTANCE_INFO, (-1 << 16)|(vil->inst_idx&0xffff));
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
						(-1 << 16)|(vil->inst_idx&0xffff));
					VPU_ISSUE_COMMAND(vil->core_idx, W5_QUERY);
					while (VPU_READL(W5_VPU_BUSY_STATUS)) {
						if (time_after(jiffies, timeout)) {
							vpu_err("Timeout to do command %d",
								W5_DESTROY_INSTANCE);
							break;
						}
					}
					VPU_WRITEL(W5_CMD_INSTANCE_INFO,
						(-1 << 16)|(vil->inst_idx&0xffff));
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

			vip = (hb_vpu_instance_pool_t *) vip_base;
			if (vip) {
				/* only first 4 byte is key point(inUse of CodecInst in vpuapi)
				   to free the corresponding instance. */
				memset(&vip->codec_inst_pool[vil->inst_idx],
				       0x00, 4);
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
			}
			dev->vpu_open_ref_count--;
			list_del(&vil->list);
			kfree(vil);
			test_and_clear_bit(vil->inst_idx, vpu_inst_bitmap);
			spin_lock(&dev->poll_spinlock);
			dev->poll_event[vil->inst_idx] = VPU_INST_CLOSED;
			spin_unlock(&dev->poll_spinlock);
			wake_up_interruptible(&dev->poll_wait_q[vil->inst_idx]);
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
		reg_val = (empty_inst & 0xffff);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_BSBUF_EMPTY);
		vpu_debug(7, "W5_RET_BS_EMPTY_INST reg_val=0x%x, inst_idx=%d\n",
			  reg_val, inst_idx);
		goto GET_VPU_INST_IDX_HANDLED;
	}

	if (int_reason & (1 << INT_WAVE5_INIT_SEQ)) {
		reg_val = (seq_inst & 0xffff);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_INIT_SEQ);
		vpu_debug(7, "RET_SEQ_DONE_INSTANCE_INFO INIT_SEQ reg_val=0x%x,"
			  "inst_idx=%d\n", reg_val, inst_idx);
		goto GET_VPU_INST_IDX_HANDLED;
	}

	if (int_reason & (1 << INT_WAVE5_DEC_PIC)) {
		reg_val = (done_inst & 0xffff);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_DEC_PIC);
		vpu_debug(7, "W5_RET_QUEUE_CMD_DONE_INST DEC_PIC reg_val=0x%x,"
			  "inst_idx=%d\n", reg_val, inst_idx);

		if (int_reason & (1 << INT_WAVE5_ENC_LOW_LATENCY)) {
			u32 ll_inst_idx;
			reg_val = (done_inst >> 16);
			ll_inst_idx = vpu_filter_inst_idx(reg_val);
			if (ll_inst_idx == inst_idx)
				*reason =
				    ((1 << INT_WAVE5_DEC_PIC) |
				     (1 << INT_WAVE5_ENC_LOW_LATENCY));
			vpu_debug(7,
				  "W5_RET_QUEUE_CMD_DONE_INST DEC_PIC and"
				  "ENC_LOW_LATENCY reg_val=0x%x, inst_idx=%d, ll_inst_idx=%d\n",
				  reg_val, inst_idx, ll_inst_idx);
		}
		goto GET_VPU_INST_IDX_HANDLED;
	}

	if (int_reason & (1 << INT_WAVE5_ENC_SET_PARAM)) {
		reg_val = (seq_inst & 0xffff);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_ENC_SET_PARAM);
		vpu_debug(7,
			  "RET_SEQ_DONE_INSTANCE_INFO ENC_SET_PARAM reg_val=0x%x,"
			  "inst_idx=%d\n", reg_val, inst_idx);
		goto GET_VPU_INST_IDX_HANDLED;
	}

#ifdef SUPPORT_SOURCE_RELEASE_INTERRUPT
	if (int_reason & (1 << INT_WAVE5_ENC_SRC_RELEASE)) {
		reg_val = (done_inst & 0xffff);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_ENC_SRC_RELEASE);
		vpu_debug(7, "W5_RET_QUEUE_CMD_DONE_INST ENC_SET_PARAM "
			  "reg_val=0x%x, inst_idx=%d\n", reg_val, inst_idx);
		goto GET_VPU_INST_IDX_HANDLED;
	}
#endif

	if (int_reason & (1 << INT_WAVE5_ENC_LOW_LATENCY)) {
		reg_val = (done_inst >> 16);
		inst_idx = vpu_filter_inst_idx(reg_val);
		*reason = (1 << INT_WAVE5_ENC_LOW_LATENCY);
		vpu_debug(7, "W5_RET_QUEUE_CMD_DONE_INST ENC_LOW_LATENCY"
			  "reg_val=0x%x, inst_idx=%d\n", reg_val, inst_idx);
		goto GET_VPU_INST_IDX_HANDLED;
	}

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
						if ((intr_reason ==
						     (1 << INT_WAVE5_ENC_LOW_LATENCY)))
						{
							done_inst = (done_inst >> 16);
							done_inst =
							    done_inst & ~(1 <<
									  intr_inst_index);
							done_inst = (done_inst << 16);
							VPU_WRITEL
							    (W5_RET_QUEUE_CMD_DONE_INST,
							     done_inst);
							if (0 == done_inst) {
								reason &= ~(1 << INT_WAVE5_ENC_LOW_LATENCY);
							}
							vpu_debug(7,
								  "W5_RET_QUEUE_CMD_DONE_INST "
								  "INT_WAVE5_ENC_LOW_LATENCY Clear "
								  "done_inst=0x%x, intr_inst_index=%d\n",
								  done_inst,
								  intr_inst_index);
						}
						if (!kfifo_is_full
						    (&dev->interrupt_pending_q
						     [intr_inst_index])) {
							if (intr_reason ==
							    ((1 << INT_WAVE5_ENC_PIC) |
							     (1 <<
							      INT_WAVE5_ENC_LOW_LATENCY))) {
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
		vpu_debug(7, "product: 0x%08x intr_reason: 0x%08lx\n",
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
						dev->instance_pool.size =
						    PAGE_ALIGN
						    (dev->instance_pool.size);
						dev->instance_pool.base =
						    (unsigned long)
						    vmalloc
						    (dev->instance_pool.size);
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
							       dev->instance_pool.size);
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

		/* found and free the not closed instance by user applications */
		vpu_free_instances(filp);
		dev->open_count--;
		open_count = dev->open_count;
		spin_unlock(&dev->vpu_spinlock);
		if (open_count == 0) {
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
	int i;
	int core;
	/* vpu wait timeout to 1sec */
	unsigned long timeout = jiffies + HZ;
	int product_code;
	hb_vpu_dev_t *dev;

	vpu_debug_enter();
	dev = (hb_vpu_dev_t *) platform_get_drvdata(pdev);
	hb_vpu_clk_enable(dev, dev->vpu_freq);

	if (dev->vpu_open_ref_count > 0) {
		for (core = 0; core < MAX_NUM_VPU_CORE; core++) {
			if (dev->bit_fm_info[core].size == 0)
				continue;
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
	int i;
	int core;
	u32 val;
	unsigned long timeout = jiffies + HZ;	/* vpu wait timeout to 1sec */
	int product_code;

	unsigned long code_base;
	u32 code_size;
	u32 remap_size;
	int regVal;
	u32 hwOption = 0;
	hb_vpu_dev_t *dev;

	vpu_debug_enter();
	dev = (hb_vpu_dev_t *) platform_get_drvdata(pdev);
	hb_vpu_clk_enable(dev, dev->vpu_freq);

	for (core = 0; core < MAX_NUM_VPU_CORE; core++) {
		if (dev->bit_fm_info[core].size == 0) {
			continue;
		}

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
			VPU_WRITEL(W5_INIT_VPU_TIME_OUT_CNT, timeout);

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
