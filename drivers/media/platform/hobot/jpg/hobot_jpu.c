#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/version.h>

#include "hobot_jpu_buf.h"
#include "hobot_jpu_ctl.h"
#include "hobot_jpu_debug.h"
#include "hobot_jpu_pm.h"
#include "hobot_jpu_reg.h"
#include "hobot_jpu_utils.h"

int jpu_debug_flag = 7;

#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
#define JPU_INIT_VIDEO_MEMORY_SIZE_IN_BYTE (16*1024*1024)
#define JPU_DRAM_PHYSICAL_BASE (0x63E00000)	//(0x8AA00000)
static jpu_mm_t s_jmem;
static hb_jpu_drv_buffer_t s_video_memory = { 0 };
#endif

/* this definition is only for chipsnmedia FPGA board env */
/* so for SOC env of customers can be ignored */

/*for kernel up to 3.7.0 version*/
#ifndef VM_RESERVED
#define VM_RESERVED   (VM_DONTEXPAND | VM_DONTDUMP)
#endif

#ifdef CONFIG_PM
/* implement to power management functions */
#endif

DECLARE_BITMAP(jpu_inst_bitmap, MAX_NUM_JPU_INSTANCE);

static ssize_t jpu_debug_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	// TODO add dump interface.

	return (s - buf);
}

static ssize_t jpu_debug_store(struct kobject *kobj,
			       struct kobj_attribute *attr, const char *buf,
			       size_t n)
{
	int error = -EINVAL;
	return error ? error : n;
}

static struct kobj_attribute jpu_debug_attr = {
	.attr = {
		 .name = __stringify(jpu_debug_attr),
		 .mode = 0644,
		 },
	.show = jpu_debug_show,
	.store = jpu_debug_store,
};

static struct attribute *attributes[] = {
	&jpu_debug_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attributes,
};

static int jpu_alloc_dma_buffer(hb_jpu_drv_buffer_t * jb)
{
	if (!jb)
		return -1;
#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
	jb->phys_addr = (unsigned long long)jmem_alloc(&s_jmem, jb->size, 0);
	if ((unsigned long)jb->phys_addr == (unsigned long)-1) {
		jpu_debug(5, "Physical memory allocation error size=%d\n",
			  jb->size);
		return -1;
	}

	jb->base = (unsigned long)(s_video_memory.base + (jb->phys_addr
							  -
							  s_video_memory.
							  phys_addr));
#else
	jb->base = (unsigned long)dma_alloc_coherent(NULL, PAGE_ALIGN(jb->size),
						     (dma_addr_t *) (&jb->
								     phys_addr),
						     GFP_DMA | GFP_KERNEL);
	if ((void *)(jb->base) == NULL) {
		jpu_debug(5, "Physical memory allocation error size=%d\n",
			  jb->size);
		return -1;
	}
#endif /* JPU_SUPPORT_RESERVED_VIDEO_MEMORY */
	return 0;
}

static void jpu_free_dma_buffer(hb_jpu_drv_buffer_t * jb)
{
	if (!jb) {
		return;
	}

	if (jb->base)
#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
		jmem_free(&s_jmem, jb->phys_addr, 0);
#else
		dma_free_coherent(0, PAGE_ALIGN(jb->size), (void *)jb->base,
				  jb->phys_addr);
#endif /* JPUR_SUPPORT_RESERVED_VIDEO_MEMORY */
}

static int jpu_free_instances(struct file *filp)
{
	hb_jpu_drv_instance_list_t *vil, *n;
	hb_jpu_drv_instance_pool_t *vip;
	void *vip_base;
	int instance_pool_size_per_core;
	void *jdi_mutexes_base;
	const int PTHREAD_MUTEX_T_DESTROY_VALUE = 0xdead10cc;
	hb_jpu_dev_t *dev;
	jpu_debug_enter();
	if (!filp) {
		jpu_err("failed to free jpu buffers, filp is null.");
		return -1;
	}
	dev = filp->private_data;

	if (!dev) {
		jpu_err("failed to free jpu buffers, dev is null.");
		return -1;
	}
	/* s_instance_pool.size  assigned to the size of all core once call 
	   JDI_IOCTL_GET_INSTANCE_POOL by user. */
	instance_pool_size_per_core =
	    (dev->instance_pool.size / MAX_NUM_JPU_CORE);

	list_for_each_entry_safe(vil, n, &dev->inst_list_head, list) {
		if (vil->filp == filp) {
			vip_base = (void *)(dev->instance_pool.base +
					    instance_pool_size_per_core);
			jpu_debug(5,
				  "jpu_free_instances detect instance crash " 
				  "instIdx=%d, vip_base=%p, instance_pool_size_per_core=%d\n",
				  (int)vil->inst_idx, vip_base,
				  (int)instance_pool_size_per_core);
			vip = (hb_jpu_drv_instance_pool_t *) vip_base;
			if (vip) {
				/* only first 4 byte is key point(inUse of CodecInst in jpuapi)
				   to free the corresponding instance. */
				memset(&vip->codecInstPool[vil->inst_idx], 0x00,
				       4);
#define PTHREAD_MUTEX_T_HANDLE_SIZE 4
				jdi_mutexes_base =
				    (vip_base +
				     (instance_pool_size_per_core -
				      PTHREAD_MUTEX_T_HANDLE_SIZE * 4));
				jpu_debug(5,
					  "jpu_free_instances : force to destroy "
					  "jdi_mutexes_base=%p in userspace \n",
					  jdi_mutexes_base);
				if (jdi_mutexes_base) {
					int i;
					for (i = 0; i < 4; i++) {
						memcpy(jdi_mutexes_base,
						       &PTHREAD_MUTEX_T_DESTROY_VALUE,
						       PTHREAD_MUTEX_T_HANDLE_SIZE);
						jdi_mutexes_base +=
						    PTHREAD_MUTEX_T_HANDLE_SIZE;
					}
				}
			}
			dev->jpu_open_ref_count--;
			list_del(&vil->list);
			kfree(vil);
			test_and_clear_bit(vil->inst_idx, jpu_inst_bitmap);
		}
	}

	jpu_debug_leave();

	return 1;
}

static int jpu_free_buffers(struct file *filp)
{
	hb_jpu_drv_buffer_pool_t *pool, *n;
	hb_jpu_drv_buffer_t jb;
	hb_jpu_dev_t *dev;
	jpu_debug_enter();
	if (!filp) {
		jpu_err("failed to free jpu buffers, filp is null.");
		return -1;
	}
	dev = filp->private_data;

	if (!dev) {
		jpu_err("failed to free jpu buffers, dev is null.");
		return -1;
	}

	list_for_each_entry_safe(pool, n, &dev->jbp_head, list) {
		if (pool->filp == filp) {
			jb = pool->jb;
			if (jb.base) {
				jpu_free_dma_buffer(&jb);
				list_del(&pool->list);
				kfree(pool);
			}
		}
	}
	jpu_debug_leave();

	return 0;
}

static irqreturn_t jpu_irq_handler(int irq, void *dev_id)
{
	hb_jpu_dev_t *dev = (hb_jpu_dev_t *) dev_id;
	int i;
	u32 flag;

	jpu_debug_enter();

#ifdef JPU_IRQ_CONTROL
	disable_irq_nosync(dev->irq);
#endif

	for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
		flag = JPU_READL(MJPEG_PIC_STATUS_REG(i));
		if (flag != 0) {
			break;
		}
	}

	dev->interrupt_reason[i] = flag;
	dev->interrupt_flag[i] = 1;
	jpu_debug(5, "[%d] INTERRUPT FLAG: %08x, %08x\n", i,
		  dev->interrupt_reason[i], MJPEG_PIC_STATUS_REG(i));

	// notify the interrupt to userspace
	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

	wake_up_interruptible(&dev->interrupt_wait_q[i]);

	jpu_debug_leave();

	return IRQ_HANDLED;
}

static void jpu_parse_dts(struct device_node *np, hb_jpu_dev_t * jpu_dev)
{
	hb_jpu_platform_data_t *pdata = jpu_dev->plat_data;

	if (!np)
		return;

	of_property_read_u32(np, "ip_ver", &pdata->ip_ver);
	of_property_read_u32(np, "clock_rate", &pdata->clock_rate);
	of_property_read_u32(np, "min_rate", &pdata->min_rate);
}

static int jpu_open(struct inode *inode, struct file *filp)
{
	hb_jpu_dev_t *dev;
	jpu_debug_enter();

	dev = container_of(inode->i_cdev, hb_jpu_dev_t, cdev);
	if (!dev) {
		jpu_err("failed to get jpu dev data");
		return -1;
	}

	spin_lock(&dev->jpu_spinlock);
	dev->open_count++;
	filp->private_data = (void *)dev;
	spin_unlock(&dev->jpu_spinlock);

	jpu_debug_leave();

	return 0;
}

static long jpu_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	int ret = 0;
	hb_jpu_dev_t *dev = (hb_jpu_dev_t *) filp->private_data;
	jpu_debug_enter();
	if (!dev) {
		jpu_err("failed to get jpu dev data");
		return -1;
	}

	switch (cmd) {
	case JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY:
		{
			hb_jpu_drv_buffer_pool_t *jbp;

			jpu_debug(5, "[+]JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY\n");
			if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
				jbp =
				    kzalloc(sizeof(hb_jpu_drv_buffer_pool_t),
					    GFP_KERNEL);
				if (!jbp) {
					up(&dev->jpu_sem);
					return -ENOMEM;
				}

				ret =
				    copy_from_user(&(jbp->jb),
						   (hb_jpu_drv_buffer_t *) arg,
						   sizeof(hb_jpu_drv_buffer_t));
				if (ret) {
					kfree(jbp);
					up(&dev->jpu_sem);
					return -EFAULT;
				}

				ret = jpu_alloc_dma_buffer(&(jbp->jb));
				if (ret == -1) {
					ret = -ENOMEM;
					kfree(jbp);
					up(&dev->jpu_sem);
					break;
				}
				ret =
				    copy_to_user((void __user *)arg, &(jbp->jb),
						 sizeof(hb_jpu_drv_buffer_t));
				if (ret) {
					kfree(jbp);
					ret = -EFAULT;
					up(&dev->jpu_sem);
					break;
				}

				jbp->filp = filp;
				spin_lock(&dev->jpu_spinlock);
				list_add(&jbp->list, &dev->jbp_head);
				spin_unlock(&dev->jpu_spinlock);

				up(&dev->jpu_sem);
			}
			jpu_debug(5, "[-]JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY\n");
		}
		break;

	case JDI_IOCTL_FREE_PHYSICAL_MEMORY:
		{
			hb_jpu_drv_buffer_pool_t *jbp, *n;
			hb_jpu_drv_buffer_t jb;

			jpu_debug(5, "[+]VDI_IOCTL_FREE_PHYSICALMEMORY\n");
			if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
				ret =
				    copy_from_user(&jb,
						   (hb_jpu_drv_buffer_t *) arg,
						   sizeof(hb_jpu_drv_buffer_t));
				if (ret) {
					up(&dev->jpu_sem);
					return -EACCES;
				}

				if (jb.base)
					jpu_free_dma_buffer(&jb);

				spin_lock(&dev->jpu_spinlock);
				list_for_each_entry_safe(jbp, n, &dev->jbp_head,
							 list) {
					if (jbp->jb.base == jb.base) {
						list_del(&jbp->list);
						kfree(jbp);
						break;
					}
				}
				spin_unlock(&dev->jpu_spinlock);

				up(&dev->jpu_sem);
			}
			jpu_debug(5, "[-]VDI_IOCTL_FREE_PHYSICALMEMORY\n");
		}
		break;

	case JDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO:
		{
#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
			if (s_video_memory.base != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &s_video_memory,
						 sizeof(hb_jpu_drv_buffer_t));
				if (ret != 0)
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}
#endif /* JPU_SUPPORT_RESERVED_VIDEO_MEMORY */
		}
		break;

	case JDI_IOCTL_WAIT_INTERRUPT:
		{
			hb_jpu_drv_intr_t info;
			u32 instance_no;

			jpu_debug(5, "[+]JDI_IOCTL_WAIT_INTERRUPT\n");
			ret = copy_from_user(&info, (hb_jpu_drv_intr_t *) arg,
					     sizeof(hb_jpu_drv_intr_t));
			if (ret != 0)
				return -EFAULT;

			instance_no = info.inst_idx;
			jpu_debug(5, "INSTANCE NO: %d\n", instance_no);
			ret =
			    wait_event_interruptible_timeout(dev->
							     interrupt_wait_q
							     [instance_no],
							     dev->
							     interrupt_flag
							     [instance_no] != 0,
							     msecs_to_jiffies
							     (info.timeout));
			if (!ret) {
				jpu_debug(5, "INSTANCE NO: %d ETIME\n",
					  instance_no);
				ret = -ETIME;
				break;
			}
#if 0
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				jpu_debug(5, "INSTANCE NO: %d ERESTARTSYS\n",
					  instance_no);
				break;
			}
#endif

			jpu_debug(5,
				  "INST(%d) s_interrupt_flag(%d), reason(0x%08x)\n",
				  instance_no, dev->interrupt_flag[instance_no],
				  dev->interrupt_reason[instance_no]);

			info.intr_reason = dev->interrupt_reason[instance_no];
			dev->interrupt_flag[instance_no] = 0;
			dev->interrupt_reason[instance_no] = 0;
			ret = copy_to_user((void __user *)arg, &info,
					   sizeof(hb_jpu_drv_intr_t));
#ifdef JPU_IRQ_CONTROL
			enable_irq(dev->irq);
#endif
			jpu_debug(5, "[-]VDI_IOCTL_WAIT_INTERRUPT\n");
			if (ret != 0)
				return -EFAULT;
		}
		break;

	case JDI_IOCTL_SET_CLOCK_GATE:
		{
			u32 clkgate;
			jpu_debug(5, "[+]JDI_IOCTL_SET_CLOCK_GATE\n");

			if (get_user(clkgate, (u32 __user *) arg))
				return -EFAULT;

#ifdef JPU_SUPPORT_CLOCK_CONTROL
			if (clkgate)
				hb_jpu_clk_enable(dev->jpu_clk);
			else
				hb_jpu_clk_disable(dev->jpu_clk);
#endif /* JPU_SUPPORT_CLOCK_CONTROL */
			jpu_debug(5, "[-]JDI_IOCTL_SET_CLOCK_GATE\n");
		}
		break;

	case JDI_IOCTL_GET_INSTANCE_POOL:
		jpu_debug(5, "[+]JDI_IOCTL_GET_INSTANCE_POOL\n");

		if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
			if (dev->instance_pool.base != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &dev->instance_pool,
						 sizeof(hb_jpu_drv_buffer_t));
			} else {
				ret = copy_from_user(&dev->instance_pool,
						     (hb_jpu_drv_buffer_t *)
						     arg,
						     sizeof
						     (hb_jpu_drv_buffer_t));
				if (ret == 0) {
					dev->instance_pool.size =
					    PAGE_ALIGN(dev->instance_pool.size);
					dev->instance_pool.base =
					    (unsigned long)vmalloc(dev->
								   instance_pool.
								   size);
					dev->instance_pool.phys_addr =
					    dev->instance_pool.base;

					if (dev->instance_pool.base != 0) {
						/*clearing memory */
						memset((void *)dev->
						       instance_pool.base, 0x0,
						       dev->instance_pool.size);
						ret =
						    copy_to_user((void __user *)
								 arg,
								 &dev->
								 instance_pool,
								 sizeof
								 (hb_jpu_drv_buffer_t));
						if (ret == 0) {
							/* success to get memory for instance pool */
							jpu_debug(5,
								  "[-]JDI_IOCTL_GET_INSTANCE_POOL");
							up(&dev->jpu_sem);
							break;
						}
					}
					ret = -EFAULT;
				}
			}
			up(&dev->jpu_sem);
		}

		jpu_debug(5,
			  "[-]JDI_IOCTL_GET_INSTANCE_POOL: %s base: %lx, size: %d\n",
			  (ret == 0 ? "OK" : "NG"), dev->instance_pool.base,
			  dev->instance_pool.size);
		break;

	case JDI_IOCTL_OPEN_INSTANCE:
		{
			hb_jpu_drv_inst_t inst_info;
			hb_jpu_drv_instance_list_t *jil, *jil_tmp, *n;

			jpu_debug(5, "[+]JDI_IOCTL_OPEN_INSTANCE\n");

			jil = kzalloc(sizeof(*jil), GFP_KERNEL);
			if (copy_from_user
			    (&inst_info, (hb_jpu_drv_inst_t *) arg,
			     sizeof(hb_jpu_drv_inst_t))) {

				kfree(jil);
				return -EFAULT;
			}
			if (inst_info.inst_idx >= MAX_NUM_JPU_INSTANCE) {
				kfree(jil);
				return -EINVAL;
			}

			jil->inst_idx = inst_info.inst_idx;
			jil->filp = filp;

			spin_lock(&dev->jpu_spinlock);

			list_for_each_entry_safe(jil_tmp, n,
						 &dev->inst_list_head, list) {
				if (jil_tmp->inst_idx == inst_info.inst_idx) {
					kfree(jil);
					jpu_err
					    ("Failed to open instance due to same id(%d)",
					     (int)inst_info.inst_idx);
					spin_unlock(&dev->jpu_spinlock);
					return -EINVAL;
				}
			}

			list_add(&jil->list, &dev->inst_list_head);

			/* counting the current open instance number */
			inst_info.inst_open_count = 0;
			list_for_each_entry_safe(jil, n, &dev->inst_list_head,
						 list) {
				inst_info.inst_open_count++;
			}
			/* flag just for that jpu is in opened or closed */
			dev->jpu_open_ref_count++;
			spin_unlock(&dev->jpu_spinlock);

			if (copy_to_user((void __user *)arg, &inst_info,
					 sizeof(hb_jpu_drv_inst_t))) {
				return -EFAULT;
			}

			jpu_debug(5, "[-]JDI_IOCTL_OPEN_INSTANCE inst_idx=%d, "
				  "s_jpu_open_ref_count=%d, inst_open_count=%d\n",
				  (int)inst_info.inst_idx,
				  dev->jpu_open_ref_count,
				  inst_info.inst_open_count);
		}
		break;

	case JDI_IOCTL_CLOSE_INSTANCE:
		{
			hb_jpu_drv_inst_t inst_info;
			hb_jpu_drv_instance_list_t *jil, *n;

			jpu_debug(5, "[+]JDI_IOCTL_CLOSE_INSTANCE\n");
			if (copy_from_user
			    (&inst_info, (hb_jpu_drv_inst_t *) arg,
			     sizeof(hb_jpu_drv_inst_t)))
				return -EFAULT;
			if (inst_info.inst_idx >= MAX_NUM_JPU_INSTANCE) {
				return -EINVAL;
			}

			spin_lock(&dev->jpu_spinlock);
			list_for_each_entry_safe(jil, n, &dev->inst_list_head,
						 list) {
				if (jil->inst_idx == inst_info.inst_idx) {
					list_del(&jil->list);
					kfree(jil);
					break;
				}
			}

			/* counting the current open instance number */
			inst_info.inst_open_count = 0;
			list_for_each_entry_safe(jil, n, &dev->inst_list_head,
						 list) {
				inst_info.inst_open_count++;
			}
			/* flag just for that jpu is in opened or closed */
			dev->jpu_open_ref_count--;
			spin_unlock(&dev->jpu_spinlock);

			if (copy_to_user((void __user *)arg, &inst_info,
					 sizeof(hb_jpu_drv_inst_t)))
				return -EFAULT;

			jpu_debug(5, "[-]JDI_IOCTL_CLOSE_INSTANCE inst_idx=%d, "
				  "s_jpu_open_ref_count=%d, inst_open_count=%d\n",
				  (int)inst_info.inst_idx,
				  dev->jpu_open_ref_count,
				  inst_info.inst_open_count);
		}
		break;

	case JDI_IOCTL_GET_INSTANCE_NUM:
		{
			hb_jpu_drv_inst_t inst_info;
			hb_jpu_drv_instance_list_t *jil, *n;

			jpu_debug(5, "[+]JDI_IOCTL_GET_INSTANCE_NUM\n");

			ret =
			    copy_from_user(&inst_info,
					   (hb_jpu_drv_inst_t *) arg,
					   sizeof(hb_jpu_drv_inst_t));
			if (ret != 0) {
				ret = -EFAULT;
				break;
			}
			spin_lock(&dev->jpu_spinlock);
			inst_info.inst_open_count = 0;
			list_for_each_entry_safe(jil, n, &dev->inst_list_head,
						 list) {
				inst_info.inst_open_count++;
			}
			spin_unlock(&dev->jpu_spinlock);

			ret = copy_to_user((void __user *)arg, &inst_info,
					   sizeof(hb_jpu_drv_inst_t));

			jpu_debug(5,
				  "[-]JDI_IOCTL_GET_INSTANCE_NUM inst_idx=%d, "
				  "open_count=%d\n", (int)inst_info.inst_idx,
				  inst_info.inst_open_count);
		}
		break;

	case JDI_IOCTL_RESET:
		jpu_debug(5, "[+]JDI_IOCTL_RESET\n");
		hb_jpu_hw_reset();
		jpu_debug(5, "[-]JDI_IOCTL_RESET\n");
		break;

	case JDI_IOCTL_GET_REGISTER_INFO:
		{
			hb_jpu_drv_buffer_t reg_buf;
			jpu_debug(5, "[+]JDI_IOCTL_GET_REGISTER_INFO\n");
			reg_buf.phys_addr = dev->jpu_mem->start;
			reg_buf.virt_addr = (unsigned long)dev->regs_base;
			reg_buf.size = resource_size(dev->jpu_mem);
			ret = copy_to_user((void __user *)arg, &reg_buf,
					   sizeof(hb_jpu_drv_buffer_t));
			if (ret != 0)
				ret = -EFAULT;
			jpu_debug(5, "[-]JDI_IOCTL_GET_REGISTER_INFO "
				  "jpu_register.phys_addr==0x%lx, s_jpu_register.virt_addr=0x%lx,"
				  "s_jpu_register.size=%d\n", reg_buf.phys_addr,
				  reg_buf.virt_addr, reg_buf.size);
		}
		break;
	case JDI_IOCTL_ALLOCATE_INSTANCE_ID:
		{
			int inst_index;
			jpu_debug(5, "[+]JDI_IOCTL_ALLOCATE_INSTANCE_ID\n");
			spin_lock(&dev->jpu_spinlock);
			inst_index =
			    find_first_zero_bit(jpu_inst_bitmap,
						MAX_NUM_JPU_INSTANCE);
			if (inst_index < MAX_NUM_JPU_INSTANCE) {
				set_bit(inst_index, jpu_inst_bitmap);
			} else {
				inst_index = -1;
			}
			spin_unlock(&dev->jpu_spinlock);

			ret =
			    copy_to_user((void __user *)arg, &inst_index,
					 sizeof(int));
			if (ret != 0)
				ret = -EFAULT;
			jpu_debug(5,
				  "[-]JDI_IOCTL_ALLOCATE_INSTANCE_ID id = %d\n",
				  inst_index);
		}
		break;
	case JDI_IOCTL_FREE_INSTANCE_ID:
		{
			int inst_index;
			jpu_debug(5, "[+]JDI_IOCTL_FREE_INSTANCE_ID\n");
			ret =
			    copy_from_user(&inst_index, (int *)arg,
					   sizeof(int));
			if (ret != 0
			    || (inst_index < 0
				|| inst_index >= MAX_NUM_JPU_INSTANCE)) {
				jpu_err
				    ("JDI_IOCTL_FREE_INSTANCE_ID invalid instance id.");
				return -EFAULT;
			}
			spin_lock(&dev->jpu_spinlock);
			clear_bit(inst_index, jpu_inst_bitmap);
			spin_unlock(&dev->jpu_spinlock);

			jpu_debug(5,
				  "[-]JDI_IOCTL_FREE_INSTANCE_ID clear id = %d\n",
				  inst_index);
		}
		break;

	default:
		{
			jpu_err("No such IOCTL, cmd is %d\n", cmd);
		}
		break;
	}
	return ret;
}

static ssize_t jpu_read(struct file *filp, char __user * buf, size_t len,
			loff_t * ppos)
{
	jpu_debug_enter();
	jpu_debug_leave();
	return -1;
}

static ssize_t jpu_write(struct file *filp, const char __user * buf,
			 size_t len, loff_t * ppos)
{
	jpu_debug_enter();
	/* DPRINTK("jpu_write len=%d\n", (int)len); */
	if (!buf) {
		jpu_debug(5, "jpu_write buf = NULL error \n");
		return -EFAULT;
	}
	jpu_debug_leave();

	return -1;
}

static int jpu_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	u32 open_count;
	int i;

	hb_jpu_dev_t *dev;
	jpu_debug_enter();
	dev = container_of(inode->i_cdev, hb_jpu_dev_t, cdev);
	if (!dev) {
		jpu_err("failed to get jpu dev data");
		return -1;
	}

	if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
		/* found and free the not handled buffer by user applications */
		jpu_free_buffers(filp);

		/* found and free the not closed instance by user applications */
		jpu_free_instances(filp);
		jpu_debug(5, "open_count: %d\n", dev->open_count);
		spin_lock(&dev->jpu_spinlock);
		dev->open_count--;
		open_count = dev->open_count;
		spin_unlock(&dev->jpu_spinlock);
		if (open_count == 0) {
			if (dev->instance_pool.base) {
				jpu_debug(5, "free instance pool\n");
				vfree((const void *)dev->instance_pool.base);
				dev->instance_pool.base = 0;
			}
			for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++)
				test_and_clear_bit(i, jpu_inst_bitmap);
		}
	}
	up(&dev->jpu_sem);

	jpu_debug_leave();

	return 0;
}

static int jpu_fasync(int fd, struct file *filp, int mode)
{
	int ret = 0;
	hb_jpu_dev_t *dev;
	jpu_debug_enter();
	dev = filp->private_data;
	if (!dev) {
		jpu_err("failed to get jpu dev data");
		return -1;
	}

	ret = fasync_helper(fd, filp, mode, &dev->async_queue);
	jpu_debug_leave();
	return ret;
}

static int jpu_map_to_register(struct file *filp, struct vm_area_struct *vm)
{
	unsigned long pfn;
	hb_jpu_dev_t *dev;
	int ret;
	jpu_debug_enter();

	if (!filp || !vm) {
		jpu_err("failed to map register, filp or vm is null.");
		return -1;
	}

	dev = filp->private_data;

	if (!dev) {
		jpu_err("failed to map register, dev is null.");
		return -1;
	}

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = dev->jpu_mem->start >> PAGE_SHIFT;
	ret = remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			      vm->vm_page_prot) ? -EAGAIN : 0;
	jpu_debug_leave();
	return ret;
}

static int jpu_map_to_physical_memory(struct file *filp,
				      struct vm_area_struct *vm)
{
	hb_jpu_dev_t *dev;
	int ret;
	jpu_debug_enter();

	if (!filp || !vm) {
		jpu_err("failed to map register, filp or vm is null.");
		return -1;
	}

	dev = filp->private_data;

	if (!dev) {
		jpu_err("failed to map register, dev is null.");
		return -1;
	}

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	ret = remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			      vm->vm_end - vm->vm_start,
			      vm->vm_page_prot) ? -EAGAIN : 0;
	jpu_debug_leave();
	return ret;
}

static int jpu_map_to_instance_pool_memory(struct file *filp,
					   struct vm_area_struct *vm)
{
	int ret;
	long length;
	unsigned long start;
	char *vmalloc_area_ptr;
	unsigned long pfn;
	hb_jpu_dev_t *dev;

	jpu_debug_enter();

	if (!filp || !vm) {
		jpu_err("failed to map instances, filp or vm is null.");
		return -1;
	}

	dev = filp->private_data;

	if (!dev) {
		jpu_err("failed to map  instances, dev is null.");
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

	jpu_debug_leave();

	return 0;
}

/*!
* @brief memory map interface for jpu file operation
* @return  0 on success or negative error code on error
*/
static int jpu_mmap(struct file *filp, struct vm_area_struct *vm)
{
	hb_jpu_dev_t *dev;
	jpu_debug_enter();
	dev = filp->private_data;

	if (vm->vm_pgoff == 0)
		return jpu_map_to_instance_pool_memory(filp, vm);

	if (vm->vm_pgoff == (dev->jpu_mem->start >> PAGE_SHIFT))
		return jpu_map_to_register(filp, vm);

	return jpu_map_to_physical_memory(filp, vm);
}

struct file_operations jpu_fops = {
	.owner = THIS_MODULE,
	.open = jpu_open,
	.read = jpu_read,
	.write = jpu_write,
	.unlocked_ioctl = jpu_ioctl,
	.release = jpu_release,
	.fasync = jpu_fasync,
	.mmap = jpu_mmap,
};

static int jpu_probe(struct platform_device *pdev)
{
	hb_jpu_dev_t *dev;
	struct resource *res = NULL;
	int err = 0;
	int i;

	dev_dbg(&pdev->dev, "%s()\n", __func__);
	dev = devm_kzalloc(&pdev->dev, sizeof(hb_jpu_dev_t), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Not enough memory for JPU device.\n");
		err = -ENOMEM;
		goto ERR_RESOURSE;
	}
	dev->device = &pdev->dev;

	dev->plat_data = pdev->dev.platform_data;
	dev->plat_data =
	    devm_kzalloc(&pdev->dev, sizeof(hb_jpu_platform_data_t),
			 GFP_KERNEL);
	if (!dev->plat_data) {
		dev_err(&pdev->dev,
			"Not enough memory for JPU platform data\n");
		err = -ENOMEM;
		goto ERR_RESOURSE;
	}
	jpu_parse_dts(dev->device->of_node, dev);

	err = hb_jpu_init_pm(dev->device);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to setup jpu clock & power\n");
		goto ERR_INIT_PM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get memory resource\n");
		err = -ENOENT;
		goto ERR_RES_MEM;
	}
	dev->jpu_mem = request_mem_region(res->start, resource_size(res),
					  pdev->name);
	if (!dev->jpu_mem) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		err = -ENOENT;
		goto ERR_REQ_MEM;
	}
	dev->regs_base = ioremap_nocache(dev->jpu_mem->start,
					 resource_size(dev->jpu_mem));
	if (!dev->regs_base) {
		dev_err(&pdev->dev, "failed to ioremap address region\n");
		err = -ENOENT;
		goto ERR_IO_REMAP;
	}
	dev_dbg(&pdev->dev,
		"jpu IO memory resource: physical base addr = 0x%llx,"
		"virtual base addr = %p\n", dev->jpu_mem->start,
		dev->regs_base);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get irq resource\n");
		err = -ENOENT;
		goto ERR_RES_IRQ;
	}
	dev->irq = res->start;
	// TODO Add top half irq and bottom half irq?
	err = request_threaded_irq(dev->irq, jpu_irq_handler, NULL,
				   IRQF_ONESHOT, pdev->name, dev);
	if (err) {
		dev_err(&pdev->dev,
			"failed to install register interrupt handler\n");
		goto ERR_REQ_IRQ;
	}
	dev_dbg(&pdev->dev, "jpu irq number: irq = %d\n", dev->irq);

	dev->jpu_class = class_create(THIS_MODULE, pdev->name);
	if (IS_ERR(dev->jpu_class)) {
		dev_err(&pdev->dev, "failed to create class\n");
		err = PTR_ERR(dev->jpu_class);
		goto ERR_CREATE_CLASS;
	}

	/* get the major number of the character device */
	err = alloc_chrdev_region(&dev->jpu_dev_num, 0, 1, JPU_DEV_NAME);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to allocate character device\n");
		goto ERR_ALLOCATE_CHR;
	} else {
		dev->major = MAJOR(dev->jpu_dev_num);
		dev->minor = MINOR(dev->jpu_dev_num);
	}
	dev_dbg(&pdev->dev, "jpu device number: major = %d, minor = %d\n",
		dev->major, dev->minor);

	/* initialize the device structure and register the device with the kernel */
	cdev_init(&dev->cdev, &jpu_fops);
	err = cdev_add(&dev->cdev, dev->jpu_dev_num, 1);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to add character device\n");
		goto ERR_ADD_CHR;
	}

	dev->jpu_dev = device_create(dev->jpu_class, NULL, dev->jpu_dev_num,
				     NULL, JPU_DEV_NAME);
	if (IS_ERR(dev->jpu_dev)) {
		err = PTR_ERR(dev->jpu_dev);
		dev->jpu_dev = NULL;
		goto ERR_CREATE_DEV;
	}

	/* create sysfs interface */
	dev->jpu_kobj = kobject_create_and_add(pdev->name, NULL);
	if (!dev->jpu_kobj) {
		dev_err(&pdev->dev, "failed to create kobj\n");
		err = -ENOMEM;
		goto ERR_CREATE_KOBJ;
	}
	err = sysfs_create_group(dev->jpu_kobj, &attr_group);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to create sysfs group\n");
		goto ERR_CREATE_SYSFS;
	}

	platform_set_drvdata(pdev, dev);

	dev->jpu_clk = hb_jpu_clk_get(&pdev->dev);
	if (!dev->jpu_clk) {
		dev_err(&pdev->dev, "not support clock controller.\n");
	} else {
		dev_err(&pdev->dev, "get clock controller s_jpu_clk = %p\n",
			dev->jpu_clk);
	}
	hb_jpu_clk_enable(dev->jpu_clk);

#ifdef CONFIG_ION_HOBOT
	dev->jpu_ion_client = ion_client_create(ion_exynos, JPU_DEV_NAME);
	if (IS_ERR(dev->jpu_ion_client)) {
		dev_err(&pdev->dev, "failed to ion_client_create\n");
		err = PTR_ERR(dev->jpu_dev);
		goto ERR_ION_CLIENT;
	}
#endif

#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (s_jmem.base_addr == 0) {
		s_video_memory.size = JPU_INIT_VIDEO_MEMORY_SIZE_IN_BYTE;
		s_video_memory.phys_addr = JPU_DRAM_PHYSICAL_BASE;
		//s_video_memory.base = (unsigned long)ioremap_nocache(s_video_memory.phys_addr, PAGE_ALIGN(s_video_memory.size));
		s_video_memory.base = (unsigned long)__va(s_video_memory.phys_addr);	// pfn_to_kaddr(pfn) //__VA(s_video_memory.phys_addr);
		//s_video_memory.base = phys_to_virt(s_video_memory.phys_addr);
		if (!s_video_memory.base) {
			dev_err(&pdev->dev,
				"fail to remap video memory physical phys_addr=0x%lx,"
				"base==0x%lx, size=%d\n",
				s_video_memory.phys_addr, s_video_memory.base,
				(int)s_video_memory.size);
			err = -ENOMEM;
			goto ERR_RESERVED_MEM;
		}

		if (jmem_init
		    (&s_jmem, s_video_memory.phys_addr,
		     s_video_memory.size) < 0) {
			err = -ENOMEM;
			dev_err(&pdev->dev, "fail to init jmem system\n");
			goto ERROR_INIT_VMEM;
		}
		dev_dbg(&pdev->dev,
			"success to probe jpu device with reserved video memory"
			"phys_addr==0x%lx, base = =0x%lx\n",
			s_video_memory.phys_addr, s_video_memory.base);
	}
#endif
	for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
		init_waitqueue_head(&dev->interrupt_wait_q[i]);
	}

	dev->async_queue = NULL;
	dev->open_count = 0;
	mutex_init(&dev->jpu_mutex);
	sema_init(&dev->jpu_sem, 1);
	dev->jpu_spinlock = __SPIN_LOCK_UNLOCKED(jpu_spinlock);

	INIT_LIST_HEAD(&dev->jbp_head);
	INIT_LIST_HEAD(&dev->inst_list_head);

	return 0;

#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
	jmem_exit(&s_jmem);
ERROR_INIT_VMEM:
ERR_RESERVED_MEM:
#endif
#ifdef CONFIG_ION_HOBOT
	ion_client_destroy(dev->jpu_ion_client);
ERR_ION_CLIENT:
#endif
	hb_jpu_clk_enable(dev->jpu_clk);
	hb_jpu_clk_put(dev->jpu_clk);
	sysfs_remove_group(dev->jpu_kobj, &attr_group);
ERR_CREATE_SYSFS:
	kobject_del(dev->jpu_kobj);
ERR_CREATE_KOBJ:
	device_destroy(dev->jpu_class, dev->jpu_dev_num);
ERR_CREATE_DEV:
	cdev_del(&dev->cdev);
ERR_ADD_CHR:
	unregister_chrdev_region(dev->jpu_dev_num, 1);
ERR_ALLOCATE_CHR:
	class_destroy(dev->jpu_class);
ERR_CREATE_CLASS:
	free_irq(dev->irq, dev);
ERR_REQ_IRQ:
ERR_RES_IRQ:
	iounmap(dev->regs_base);
ERR_IO_REMAP:
	release_mem_region(dev->jpu_mem->start, resource_size(dev->jpu_mem));
ERR_REQ_MEM:
ERR_RES_MEM:
	hb_jpu_final_pm(dev->device);
ERR_INIT_PM:
ERR_RESOURSE:
	return err;
}

static int jpu_remove(struct platform_device *pdev)
{
	hb_jpu_dev_t *dev;

	dev_dbg(&pdev->dev, "%s()\n", __func__);

	dev = platform_get_drvdata(pdev);

	if (dev->instance_pool.base) {
		vfree((const void *)dev->instance_pool.base);
		dev->instance_pool.base = 0;
	}
#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (s_video_memory.base) {
		iounmap((void *)s_video_memory.base);
		s_video_memory.base = 0;
		jmem_exit(&s_jmem);
	}
#endif

#ifdef CONFIG_ION_HOBOT
	ion_client_destroy(dev->jpu_ion_client);
#endif

	hb_jpu_clk_disable(dev->jpu_clk);
	hb_jpu_clk_put(dev->jpu_clk);
	sysfs_remove_group(dev->jpu_kobj, &attr_group);
	kobject_del(dev->jpu_kobj);
	device_destroy(dev->jpu_class, dev->jpu_dev_num);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->jpu_dev_num, 1);
	class_destroy(dev->jpu_class);
	free_irq(dev->irq, dev);
	iounmap(dev->regs_base);
	release_mem_region(dev->jpu_mem->start, resource_size(dev->jpu_mem));
	hb_jpu_final_pm(dev->device);

	return 0;
}

#ifdef CONFIG_PM
static int jpu_suspend(struct platform_device *pdev, pm_message_t state)
{
	hb_jpu_dev_t *dev;

	jpu_debug_enter();
	dev = (hb_jpu_dev_t *) platform_get_drvdata(pdev);
	if (!dev) {
		jpu_err("The jpu dev is NULL!");
		return -1;
	}
	hb_jpu_clk_disable(dev->jpu_clk);
	jpu_debug_enter();
	return 0;

}

static int jpu_resume(struct platform_device *pdev)
{
	hb_jpu_dev_t *dev;

	jpu_debug_enter();
	dev = (hb_jpu_dev_t *) platform_get_drvdata(pdev);
	if (!dev) {
		jpu_err("The jpu dev is NULL!");
		return -1;
	}
	hb_jpu_clk_enable(dev->jpu_clk);
	jpu_debug_enter();
	return 0;
}
#else
#define    jpu_suspend    NULL
#define    jpu_resume    NULL
#endif /* !CONFIG_PM */

/* Power management */
/*static const struct dev_pm_ops jpu_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(jpu_suspend, jpu_resume)
};*/

static const struct of_device_id jpu_of_match[] = {
	{
	 .compatible = "hobot,x2a_jpu",
	 .data = NULL,
	 },
	{},
};

static struct platform_driver jpu_driver = {
	.probe = jpu_probe,
	.remove = jpu_remove,
	.suspend = jpu_suspend,
	.resume = jpu_resume,
	.driver = {
		   .name = JPU_PLATFORM_DEVICE_NAME,
		   .of_match_table = jpu_of_match,
		   //.pm = &jpu_pm_ops
		   },
};

module_platform_driver(jpu_driver);

MODULE_AUTHOR("HOBOT");
MODULE_DESCRIPTION("Hobot JPEG processing unit linux driver");
MODULE_LICENSE("Proprietary");
