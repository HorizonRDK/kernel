/*
 *    isp.c - driver, char device interface
 *
 *    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <asm-generic/io.h>
#include <linux/string.h>
#include <linux/wait.h>
#include "isp.h"
#include "isp_dev.h"
#include "isp_dev_regs.h"

#include "isp_base.h"

#define X2_ISP_NAME	"x2-isp"
#define X2_ISP_TIMEOUT (msecs_to_jiffies(5000))
#define X2_ISP_PING_STATION 0x10500000
#define X2_ISP_PANG_STATION 0x10600000

/* global variable define */
static int isp_major = ISP_MAJOR;
//module_param(isp_major, int, S_IRUGO);
#define Set_Bit(bit, val) (val = (uint32_t)(val | (0x1 << bit)))

struct isp_mod_s *isp_mod_data;
unsigned char *isp_cdr_addr;
static struct device *g_isp_dev;
//static struct fasync_struct *pisp_async;

void x2_isp_isr(unsigned int status, void *data)
{
	unsigned long flags;
	struct isp_mod_s *isp_dev = NULL;

	if (data == NULL)
		return;

	isp_dev = data;
//	spin_lock_irqsave(&isp_dev->slock, flags);
	if (status & ISP_FRAME_START) {
		isp_dev->irq_status = 0x00;
		struct con_reg_s wreg_data;

		wreg_data.isp_reg_addr = HMP_CDR_ADDR0;
		switch (isp_dev->cdr_sw) {
		case ping:
			wreg_data.isp_reg_data = X2_ISP_PING_STATION;
			set_isp_reg(&wreg_data);
			isp_dev->cdr_sw = pang;
			break;
		case pang:
			wreg_data.isp_reg_data = X2_ISP_PANG_STATION;
			set_isp_reg(&wreg_data);
			isp_dev->cdr_sw = ping;
			break;
		default:
			break;
		}
	}
/*
 *	if (status & ISP_HIST_FRAME_DONE){
 *		Set_Bit(27, isp_dev->irq_status);
 *	} else if (status & ISP_HIST_FRAME_DROP) {
 *		Set_Bit(23, isp_dev->irq_status);
 *	}
 *
 *	if (status & ISP_RSUM_FRAME_DONE){
 *		Set_Bit(26, isp_dev->irq_status);
 *	} else if (status & ISP_RSUM_FRAME_DROP) {
 *		Set_Bit(22, isp_dev->irq_status);
 *	}
 *
 *	if (status & ISP_GRID_FRAME_DONE){
 *		Set_Bit(25, isp_dev->irq_status);
 *	} else if (status & ISP_GRID_FRAME_DROP) {
 *		Set_Bit(21, isp_dev->irq_status);
 *	}
 *
 *	if (status & ISP_TILE_FRAME_DONE){
 *		Set_Bit(24, isp_dev->irq_status);
 *	} else if (status & ISP_TILE_FRAME_DROP) {
 *		Set_Bit(20, isp_dev->irq_status);
 *	}
 */

	if (status & ISP_STF_FRAME_FINISH) {
		//send a complete to receive data
		isp_dev->isp_condition = 1;
		wake_up_interruptible(&isp_dev->isp_waitq);
		Set_Bit(15, isp_dev->irq_status);
	} else if (status & ISP_STF_FRAME_DROP) {
		Set_Bit(16, isp_dev->irq_status);
	}

	if (status & ISP_HMP_FRAME_DROP)
		Set_Bit(15, isp_dev->irq_status);
	else if (status & ISP_HMP_FRAME_FINISH)
		Set_Bit(13, isp_dev->irq_status);

//	spin_unlock_irqrestore(&isp_dev->slock, flags);
}

static int isp_stf_addr_fill(struct isp_stf_s *info)
{
	info->isp_crd_addr = SET_TILE_ADDR;
	info->isp_tile_size = 0;
	info->isp_grid_addr = SET_GRID_ADDR;
	info->isp_grid_size = 0;
	info->isp_rsum_addr = SET_RSUM_ADDR;
	info->isp_rsum_size = 0;
	info->isp_hist_addr = SET_HIST_ADDR;
	info->isp_hist_size = 0;
	info->isp_crd_addr = SET_CDR_ADDR0;
	info->isp_crd_size = 0;
	return 0;
}

static int isp_init(void)
{
	int ret = 0;
	struct isp_dev_s *pispdev;

	pispdev = isp_get_dev();
	if (pispdev == NULL) {
		dev_info(g_isp_dev, "[%s] get isp_dev is failed!\n", __func__,
			 __LINE__);
		return -1;
	}
	isp_mod_data->cdr_sw = ping;
	dev_info(g_isp_dev, "[%s] is success !\n", __func__, __LINE__);
//	isp_cfg_init();
	return ret;
}

static int isp_start(struct isp_mod_s *dev)
{
//	spin_lock(&dev->slock);
//	isp_config_enable();
//	set_isp_stf_addr();
	ips_irq_enable(ISP_INT);
//	set_isp_start();
//	dev->runflags = (uint32_t) (ISP_RUNNING);
//	spin_unlock(&dev->slock);

	return 0;
}

static int isp_stop(struct isp_mod_s *dev)
{
//	spin_lock(&dev->slock);
	//set_isp_stop();
//	ips_irq_disable(ISP_INT);
	dev->runflags = (uint32_t) (ISP_STOPPING);
//	spin_unlock(&dev->slock);

	return 0;
}

static int isp_mod_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	struct isp_mod_s *pdev;

	dev_info(g_isp_dev, "[%s]\n", __func__);

	pdev = dev_get_drvdata(g_isp_dev);
	pfile->private_data = pdev;

	return ret;
}

static int isp_mod_release(struct inode *pinode, struct file *pfile)
{
	struct isp_dev_s *pispdev;

	dev_info(g_isp_dev, "[%s]\n", __func__);
	pispdev = isp_get_dev();
	if (!pispdev)
		return -ENOMEM;
	pfile->private_data = NULL;

	return 0;
}

static ssize_t isp_mod_read(struct file *pfile, char *puser_buf,
	size_t len, loff_t *poff)
{
	int ret = 0;
	struct isp_mod_s *isp_cdev = pfile->private_data;

	ret = wait_event_interruptible_timeout
		(isp_cdev->isp_waitq, isp_cdev->isp_condition, X2_ISP_TIMEOUT);

	isp_cdev->isp_condition = 0;

	if (copy_to_user
	    ((void __user *)puser_buf, (void *)&isp_cdev->cdr_sw,
	     sizeof(uint32_t))) {
		dev_err(g_isp_dev, "[%s: %d] isp copy data form user failed!\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	if (ret == 0)
		return -1;

	return ret;
}

static ssize_t isp_mod_write(struct file *pfile, const char *puser_buf,
			     size_t len, loff_t *poff)
{
	dev_info(g_isp_dev, "[%s]\n", __func__);

	return 0;
}

long isp_mod_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	uint32_t d;
	int ret = 0;
	struct con_reg_s reg;
	struct isp_mod_s *isp_cdev = pfile->private_data;

	switch (cmd) {
	case ISPC_START:{
			printk(KERN_ERR "x2 isp ioctl is start\n");
		//	isp_start(isp_cdev);
		}
		break;
	case ISPC_STOP:{
			printk(KERN_ERR "x2 isp ioctl is end \n");
		//	isp_stop(isp_cdev);
		}
		break;
	case ISP_READ:
		if (!arg) {
			dev_err(g_isp_dev,
				"[%s: %d] isp reg write error !\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg,
			sizeof(reg))) {
			dev_err(g_isp_dev,
				"[%s: %d] isp copy data form user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		reg.isp_reg_data = get_isp_reg(reg.isp_reg_addr);
		if (copy_to_user((void __user *)arg, (void *)&reg,
			sizeof(reg))) {
			dev_err(g_isp_dev,
				"[%s: %d] isp copy data to user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;
	case ISP_WRITE:
		if (!arg) {
			dev_err(g_isp_dev,
				"[%s: %d] isp reg write error !\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg,
			sizeof(reg))) {
			dev_err(g_isp_dev,
				"[%s: %d] isp copy data form user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		write_isp_reg(&reg);
		break;
	case ISPC_READ_REG:{
			struct con_reg_s rreg_data;

			if (copy_from_user
			    ((void *)&rreg_data, (void __user *)arg,
			     sizeof(struct con_reg_s))) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data form user failed!\n",
					__func__, __LINE__);
			} else {
				spin_lock(&isp_cdev->slock);
				rreg_data.isp_reg_data =
				    get_isp_reg(rreg_data.isp_reg_addr);
				spin_unlock(&isp_cdev->slock);
				if (copy_to_user
				    ((void __user *)arg, (void *)&rreg_data,
				     sizeof(struct con_reg_s))) {
					dev_err(g_isp_dev,
						"[%s: %d] isp copy data to user failed!\n",
						__func__, __LINE__);
				}
			}

		}
		break;
	case ISPC_WRITE_REG:{
			struct con_reg_s wreg_data;

			if (copy_from_user
			    ((void *)&wreg_data, (void __user *)arg,
			     sizeof(wreg_data))) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data form user failed!\n",
					__func__, __LINE__);
			} else {
				spin_lock(&isp_cdev->slock);
				set_isp_reg(&wreg_data);
				spin_unlock(&isp_cdev->slock);
			}
		}
		break;
	case ISPC_SET_WBG:{
			struct ae_input_s ae_data;

			if (copy_from_user
			    ((void *)&ae_data, (void __user *)arg,
			     sizeof(struct ae_input_s))) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data form user failed!\n",
					__func__, __LINE__);
			} else {
				spin_lock(&isp_cdev->slock);
				set_isp_wbg(&ae_data);
				spin_unlock(&isp_cdev->slock);
			}
		}
		break;
	case ISPC_GET_ADDR:{
			struct isp_stf_s stf_addr_data;

			if (copy_to_user
			    ((void __user *)arg, (void *)&stf_addr_data,
			     sizeof(stf_addr_data))) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data to user failed!\n",
					__func__, __LINE__);
			}
		}
		break;
	case ISPC_WRITE_STRING:{
			struct isp_iodata_s m_reg_data_write;
			uint32_t *p_iodata_write_t = NULL;

			if (copy_from_user
			    ((void *)&m_reg_data_write, (void __user *)arg,
			     sizeof(struct isp_iodata_s))) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data from user failed!\n",
					__func__, __LINE__);
				return -EINVAL;
			}
			p_iodata_write_t =
			    kzalloc(m_reg_data_write.u_datalen, GFP_KERNEL);
			if (p_iodata_write_t == NULL) {
				dev_err(g_isp_dev,
					"[%s: %d] isp kzalloc memory failed!\n",
					__func__, __LINE__);
				return -EINVAL;
			}
			if (copy_from_user
			    ((void *)p_iodata_write_t,
			     (void __user *)m_reg_data_write.p_databuffer,
			     sizeof(uint32_t) * m_reg_data_write.u_datalen)) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data from user failed!\n",
					__func__, __LINE__);
				goto error_flag_write;
			}
			/*write regs */
			spin_lock(&isp_cdev->slock);
			set_isp_regs(p_iodata_write_t,
				     m_reg_data_write.u_datalen);
			spin_unlock(&isp_cdev->slock);

			goto finish;

error_flag_write:
			kzfree(p_iodata_write_t);
		}
		break;
	case ISPC_READ_STRING:{
			struct isp_iodata_s m_reg_data_read;
			uint32_t *p_iodata_read_t = NULL;

			if (copy_from_user
			    ((void *)&m_reg_data_read, (void __user *)arg,
			     sizeof(struct isp_iodata_s))) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data from user failed!\n",
					__func__, __LINE__);
				return -EINVAL;
			}
			p_iodata_read_t =
			    kzalloc(m_reg_data_read.u_datalen *
				    sizeof(uint32_t), GFP_KERNEL);
			if (p_iodata_read_t == NULL) {
				dev_err(g_isp_dev,
					"[%s: %d] isp kzalloc memory failed!\n",
					__func__, __LINE__);
				return -EINVAL;
			}
			if (copy_from_user
			    ((void *)p_iodata_read_t,
			     (void __user *)m_reg_data_read.p_databuffer,
			     sizeof(uint32_t) * m_reg_data_read.u_datalen)) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data from user failed!\n",
					__func__, __LINE__);
				goto err_flag_read;
			}
			/*read regs */
			spin_lock(&isp_cdev->slock);
			get_isp_regs(p_iodata_read_t,
				     m_reg_data_read.u_datalen);
			spin_unlock(&isp_cdev->slock);

			if (copy_to_user
			    ((void __user *)m_reg_data_read.p_databuffer,
			     (void *)p_iodata_read_t,
			     sizeof(uint32_t) * m_reg_data_read.u_datalen)) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data to user failed!\n",
					__func__, __LINE__);
				goto err_flag_read;
			}

			goto finish;

err_flag_read:
			kzfree(p_iodata_read_t);
			return -EINVAL;

		}
		break;
	case ISPC_READ_REGS:{
			struct isp_ioreg_s rioregs;

			if (copy_from_user
			    ((void *)&rioregs, (void __user *)arg,
			     sizeof(struct isp_ioreg_s))) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data from user failed!\n",
					__func__, __LINE__);
			} else {
				spin_lock(&isp_cdev->slock);
				isp_read_regs(rioregs.udataaddr,
					      rioregs.udatalen,
					      rioregs.p_databuffer);
				spin_unlock(&isp_cdev->slock);
				if (copy_to_user
				    ((void __user *)arg,
				     (void *)rioregs.p_databuffer,
				     sizeof(rioregs.udatalen))) {
					dev_err(g_isp_dev,
						"[%s: %d] isp copy data to user failed!\n",
						__func__, __LINE__);
				}
			}

		}
		break;
	case ISPC_WRITE_REGS:{
			struct isp_ioreg_s wioregs;

			if (copy_from_user
			    ((void *)&wioregs, (void __user *)arg,
			     sizeof(struct isp_ioreg_s))) {
				dev_err(g_isp_dev,
					"[%s: %d] isp copy data from user failed!\n",
					__func__, __LINE__);
			} else {
				spin_lock(&isp_cdev->slock);
				isp_write_regs(wioregs.udataaddr,
					       wioregs.udatalen,
					       wioregs.p_databuffer);
				spin_unlock(&isp_cdev->slock);
			}
		}
		break;
	default:
		break;
	}

finish:
	return ret;
}

static int isp_mmap_fault(struct vm_fault *vmf)
{

	dev_err(g_isp_dev, "[%s] is failed!\n", __func__);

	return VM_FAULT_NOPAGE;
}

static void isp_mmap_open(struct vm_area_struct *vmf)
{
	dev_info(g_isp_dev, "[%s] is open!\n", __func__);
}

const struct vm_operations_struct isp_remap_vm_ops = {
	.open = isp_mmap_open,
	.fault = isp_mmap_fault,
};

static int isp_mod_mmap(struct file *pfile, struct vm_area_struct *pvma)
{
	struct isp_dev_s *pispdev = isp_get_dev();

	if (!pispdev)
		return -ENOMEM;

	pvma->vm_page_prot = pgprot_noncached(pvma->vm_page_prot);
	pvma->vm_flags |= VM_IO;
	pvma->vm_flags |= VM_LOCKED;

	pvma->vm_ops = &isp_remap_vm_ops;

	isp_mmap_open(pvma);

	if ((pvma->vm_end - pvma->vm_start) >= 0x50000) {
		if (remap_pfn_range
		    (pvma, pvma->vm_start, pispdev->mapbase >> PAGE_SHIFT,
		     pvma->vm_end - pvma->vm_start, pvma->vm_page_prot)) {
			dev_err(g_isp_dev, "[%s] map cdr is failed!\n",
				__func__);
			return -EAGAIN;
		}
	} else {
		if (remap_pfn_range
		    (pvma, pvma->vm_start,
		     (pispdev->mapbaseio - 0x200) >> PAGE_SHIFT,
		     pvma->vm_end - pvma->vm_start, pvma->vm_page_prot)) {
			dev_err(g_isp_dev, "[%s] map io is failed!\n",
				__func__);
			return -EAGAIN;
		}
	}

	return 0;
}

const struct file_operations isp_mod_fops = {
	.owner = THIS_MODULE,
	.open = isp_mod_open,
	.read = isp_mod_read,
	.write = isp_mod_write,
	.release = isp_mod_release,
	.unlocked_ioctl = isp_mod_ioctl,
	.compat_ioctl = isp_mod_ioctl,
	.mmap = isp_mod_mmap,
};

static int __init isp_dev_init(void)
{
	int ret;
	dev_t devno;

	struct isp_mod_s *isp_mod = NULL;
	struct isp_dev_s *pispdev = isp_get_dev();

	isp_mod = kzalloc(sizeof(struct isp_mod_s), GFP_KERNEL);
//	if (!isp_mod) {
//		dev_err(g_isp_dev, "[%s] isp_mod kzalloc is failed!\n",
//			__func__);
//		ret = -ENOMEM;
//	}

	ret = alloc_chrdev_region(&isp_mod->dev_num, 0, 1, "x2_isp");
	if (ret < 0) {
		dev_err(g_isp_dev, "[%s]  alloc_chrdev_region is failed!\n",
			__func__);
		goto fail_malloc;
	}

	cdev_init(&isp_mod->mcdev, &isp_mod_fops);
	isp_mod->name = X2_ISP_NAME;

	ret = cdev_add(&isp_mod->mcdev, isp_mod->dev_num, ISP_NR_DEVS);
	if (ret) {
		dev_err(g_isp_dev, "[%s] x2_isp region is failed!\n", __func__);
		goto fail_malloc;
	}

	isp_mod->class = class_create(THIS_MODULE, "x2_isp");
	if (IS_ERR(isp_mod->class)) {
		dev_err(g_isp_dev, "[%s] class_create is failed!\n", __func__);
		ret = PTR_ERR(isp_mod->class);
		goto fail_malloc;
	}
	g_isp_dev =
	    device_create(isp_mod->class, NULL, isp_mod->dev_num,
			  (void *)isp_mod, "x2_isp");
	if (IS_ERR(g_isp_dev)) {
		dev_err(g_isp_dev, "[%s] device_create is failed!\n", __func__);
		ret = PTR_ERR(g_isp_dev);
		goto fail_malloc;
	}
	spin_lock_init(&isp_mod->slock);
//	init_completion(&isp_mod->isp_completion);
	init_waitqueue_head(&isp_mod->isp_waitq);
	isp_mod->isp_condition = 0;

	isp_cdr_addr = pispdev->vaddr;
	if (isp_cdr_addr == NULL)
		dev_err(g_isp_dev, "[%s] reserved cdr is failed!\n", __func__);

	dev_info(g_isp_dev, "[%s] isp cdr addr  is [%x]\n", __func__,
		 isp_cdr_addr);

	isp_stf_addr_fill(&isp_mod->isp_stf_memory);

	isp_mod_data = isp_mod;

	isp_init();
	/*register isr */
	ips_irq_disable(ISP_INT);
	ips_register_irqhandle(ISP_INT, x2_isp_isr, isp_mod_data);

	return 0;

fail_malloc:
	class_destroy(isp_mod->class);
	cdev_del(&isp_mod->mcdev);
	unregister_chrdev_region(isp_mod->dev_num, 1);
	kzfree(isp_mod);

	return ret;
}

static void __exit isp_dev_exit(void)
{
	struct isp_mod_s *isp_mod = dev_get_drvdata(g_isp_dev);

	dev_info(g_isp_dev, "[%s] is success!\n", __func__);
	device_destroy(isp_mod->class, isp_mod->dev_num);
	class_destroy(isp_mod->class);
	unregister_chrdev_region(isp_mod->dev_num, 1);
	kzfree(isp_mod);
}

late_initcall(isp_dev_init);
//module_init(isp_dev_init);
module_exit(isp_dev_exit);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("Image Signal Process for X2 of chip");
MODULE_LICENSE("GPL");
