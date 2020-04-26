/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/ion.h>
#include <linux/crc16.h>
#include "system_chardev.h"
#include "acamera_logger.h"
#include "acamera_tuning.h"
#include "acamera_fw.h"
#include "acamera_command_api.h"
#include "isp_ctxsv.h"

#define SYSTEM_CHARDEV_FIFO_SIZE 4096
#define SYSTEM_CHARDEV_NAME "ac_isp"

#define ISPIOC_REG_RW   _IOWR('P', 0, struct metadata_t)
#define ISPIOC_LUT_RW   _IOWR('P', 1, struct metadata_t)
#define ISPIOC_COMMAND  _IOWR('P', 2, struct metadata_t)
#define ISPIOC_BUF_PACTET  _IOWR('P', 3, isp_packet_s)
#define ISPIOC_GET_CTX _IOWR('P', 4, isp_ctx_r_t)
#define ISPIOC_PUT_CTX _IOWR('P', 5, isp_ctx_r_t)
#define ISPIOC_FILL_CTX _IOWR('P', 6, isp_ctx_w_t)

#define CHECK_CODE	0xeeff

struct metadata_t {
        void *ptr;
        uint8_t chn;
        uint8_t dir;
        uint8_t id;
        uint32_t elem;
};

// for register
struct regs_t {
        uint32_t addr;
        uint8_t m;
        uint8_t n;
        uint32_t v;
};

// for command
struct kv_t {
        uint32_t k;
        uint32_t v;
};

typedef struct _isp_packet_s {
        uint32_t buf[5];
        void *pdata;
} isp_packet_s;

#define BUF_LENGTH  1024

struct isp_dev_context {
    uint8_t dev_inited;
    int dev_minor_id;
    char *dev_name;
    int dev_opened;

    // for ctx dump
    struct ion_client *client;
    struct ion_handle *handle;
    void *vir_addr;
    phys_addr_t phy_addr;
    size_t mem_size;

    struct miscdevice isp_dev;
    struct mutex fops_lock;

    struct kfifo isp_kfifo_in;
    struct kfifo isp_kfifo_out;

    wait_queue_head_t kfifo_in_queue;
    wait_queue_head_t kfifo_out_queue;
};

extern struct ion_device *hb_ion_dev;
extern int32_t acamera_set_api_context(uint32_t ctx_num);
extern void system_reg_rw(struct regs_t *rg, uint8_t dir);

/* First item for ACT Control, second for user-FW */
static struct isp_dev_context isp_dev_ctx;

static int isp_fops_open( struct inode *inode, struct file *f )
{
    int rc;
    struct isp_dev_context *p_ctx = NULL;
    int minor = iminor( inode );

    LOG( LOG_INFO, "client is opening..., minor: %d.", minor );

    if ( !isp_dev_ctx.dev_inited ) {
        LOG( LOG_ERR, "dev is not inited, failed to open." );
        return -ERESTARTSYS;
    }

    if ( minor == isp_dev_ctx.dev_minor_id )
        p_ctx = &isp_dev_ctx;

    if ( !p_ctx ) {
        LOG( LOG_ERR, "Fatal error, isp dev contexts is wrong, contents dump:" );
        LOG( LOG_ERR, "    minor_id: %d, name: %s.", isp_dev_ctx.dev_minor_id, isp_dev_ctx.dev_name );

        return -ERESTARTSYS;
    }

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed of dev: %s.", p_ctx->dev_name );
        goto lock_failure;
    }

    f->private_data = p_ctx;
    p_ctx->dev_opened++;

    LOG( LOG_INFO, "open(%s) succeed.", p_ctx->dev_name );

#if 0
    if ( p_ctx->dev_opened ) {
        LOG( LOG_ERR, "open(%s) failed, already opened.", p_ctx->dev_name );
        rc = -EBUSY;
    } else {
        p_ctx->dev_opened = 1;
        rc = 0;
        LOG( LOG_INFO, "open(%s) succeed.", p_ctx->dev_name );

        LOG( LOG_INFO, "Bf set, private_data: %p.", f->private_data );
        f->private_data = p_ctx;
        LOG( LOG_INFO, "Af set, private_data: %p.", f->private_data );
    }
#endif
    mutex_unlock( &p_ctx->fops_lock );

lock_failure:
    return rc;
}

static int isp_fops_release( struct inode *inode, struct file *f )
{
    int rc;
    struct isp_dev_context *p_ctx = (struct isp_dev_context *)f->private_data;

    if ( p_ctx != &isp_dev_ctx ) {
        LOG( LOG_ERR, "Inalid paramter: %p.", p_ctx );
        return -EINVAL;
    }

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed of dev: %s.", p_ctx->dev_name );
        return rc;
    }

    p_ctx->dev_opened--;

    if ( p_ctx->dev_opened <= 0 ) {
        p_ctx->dev_opened = 0;
        f->private_data = NULL;
        kfifo_reset( &p_ctx->isp_kfifo_in );
        kfifo_reset( &p_ctx->isp_kfifo_out );
    } else {
        pr_info("device name is %s, dev_opened reference count: %d.\n", p_ctx->dev_name, p_ctx->dev_opened);
    }

    mutex_unlock( &p_ctx->fops_lock );

    return 0;
}

static ssize_t isp_fops_write( struct file *file, const char __user *buf, size_t count, loff_t *ppos )
{
    int rc;
    unsigned int copied;
    struct isp_dev_context *p_ctx = (struct isp_dev_context *)file->private_data;

    /* isp_dev_uf device only support read at the moment */
    if ( p_ctx != &isp_dev_ctx ) {
        LOG( LOG_ERR, "Inalid paramter: %p.", p_ctx );
        return -EINVAL;
    }

    if ( mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
        LOG( LOG_ERR, "Fatal error: access lock failed." );
        return -ERESTARTSYS;
    }

    rc = kfifo_from_user( &p_ctx->isp_kfifo_in, buf, count, &copied );

    /* awake any reader */
    wake_up_interruptible( &p_ctx->kfifo_in_queue );

    LOG( LOG_DEBUG, "wake up reader." );

    mutex_unlock( &p_ctx->fops_lock );

    return rc ? rc : copied;
}

static ssize_t isp_fops_read( struct file *file, char __user *buf, size_t count, loff_t *ppos )
{
    int rc;
    unsigned int copied;
    struct isp_dev_context *p_ctx = (struct isp_dev_context *)file->private_data;

    if ( p_ctx != &isp_dev_ctx ) {
        LOG( LOG_ERR, "Inalid paramter: %p.", p_ctx );
        return -EINVAL;
    }

    if ( mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
        LOG( LOG_ERR, "Fatal error: access lock failed." );
        return -ERESTARTSYS;
    }

    if ( kfifo_is_empty( &p_ctx->isp_kfifo_out ) ) {
        long time_out_in_jiffies = 1;      /* jiffies is depend on HW, in x86 Ubuntu, it's 4 ms, 1 is 4ms. */
        mutex_unlock( &p_ctx->fops_lock ); /* unlock before we return or go sleeping */

        /* return if it's a non-blocking reading  */
        if ( file->f_flags & O_NONBLOCK )
            return -EAGAIN;

        /* wait for the event */
        LOG( LOG_DEBUG, "input FIFO is empty, wait for data, timeout_in_jiffies: %ld, HZ: %d.", time_out_in_jiffies, HZ );
	rc = wait_event_interruptible(p_ctx->kfifo_out_queue,
		!kfifo_is_empty(&p_ctx->isp_kfifo_out));
        LOG( LOG_DEBUG, "data is coming or timeout, kfifo_out size: %u, rc: %d.", kfifo_len( &p_ctx->isp_kfifo_out ), rc );

        /* after wake up, we need to re-gain the mutex */
        if ( mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
            LOG( LOG_ERR, "Fatal error: access lock failed." );
            return -ERESTARTSYS;
        }
    }

    rc = kfifo_to_user( &p_ctx->isp_kfifo_out, buf, count, &copied );
    mutex_unlock( &p_ctx->fops_lock );

    return rc ? rc : copied;
}

#if 0
/** move to isp_fops_ioctl*/
static long isp_fops_ioctl_bak(struct file *pfile, unsigned int cmd,
	unsigned long arg)
{
	long ret = 0;
	isp_packet_s packet;
	uint32_t buf[BUF_LENGTH];
	uint32_t *buf_m = NULL;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	switch (cmd) {
	case DISP_BUF_PACTET: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&packet, (void __user *)arg,
			sizeof(isp_packet_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		if (packet.buf[0] < BUF_LENGTH * 4) {
			buf_m = buf;
		} else {
			buf_m = kzalloc(sizeof(uint32_t) * packet.buf[0], GFP_KERNEL);
			if (buf_m == NULL) {
				LOG(LOG_ERR, "kzalloc is failed!\n");
				return -EINVAL;
			}
		}
		memcpy(buf_m, packet.buf, sizeof(packet.buf));
		if (packet.pdata != NULL) {
			if (copy_from_user((void *)(buf_m + (sizeof(packet.buf)
				/ sizeof(uint32_t))), (void __user *)packet.pdata, packet.buf[4])) {
				LOG(LOG_ERR, "copy is err !\n");
				ret = -EINVAL;
				goto err_flag;
			}
		}
		process_ioctl_buf(buf_m);
		if (packet.pdata != NULL) {
			if (copy_to_user((void __user *)packet.pdata,
				(void *)(buf_m + (sizeof(packet.buf)
				/ sizeof(uint32_t))), packet.buf[4])) {
				LOG(LOG_ERR, "copy is err !\n");
				ret = -EINVAL;
				goto err_flag;
			}
		}
		if (copy_to_user((void __user *)arg, (void *)buf_m, sizeof(isp_packet_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			ret = -EINVAL;
		}
err_flag:
		if (packet.buf[0] >= (BUF_LENGTH * 4)) {
			kzfree(buf_m);
		}
	}
	break;
	default: {
		LOG(LOG_ERR, "---cmd is err---\n");
		ret = -1;
	}
	break;
	}

	return ret;
}
#endif

extern void *acamera_get_ctx_ptr(uint32_t ctx_id);
static long isp_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	uint32_t ret_value = 0;
	struct metadata_t md;
	struct metadata_t *pmd = (struct metadata_t *)arg;

	if (!isp_dev_ctx.dev_inited) {
		LOG(LOG_ERR, "dev is not inited, failed to ioctl.");
		return -1;
	}

	mutex_lock(&isp_dev_ctx.fops_lock);

	acamera_set_api_context(md.chn);

	switch (cmd) {
	case ISPIOC_REG_RW:
	{
        	uint8_t i = 0;
		uint32_t s = 0;
		struct regs_t *rg;
		if (copy_from_user(&md, (void __user *)arg, sizeof(md))) {
			ret = -EFAULT;
			break;
		}
		s = md.elem * sizeof(struct regs_t);
		md.ptr = kzalloc(s, GFP_KERNEL);

		if (copy_from_user(md.ptr, pmd->ptr, s)) {
			ret = -EFAULT;
			break;
		}

		rg = md.ptr;

		for (i = 0; i < md.elem; i++) {
			system_reg_rw(&rg[i], md.dir);
		}

		if (md.dir == COMMAND_GET) {
			if (copy_to_user(pmd->ptr, md.ptr, s)) {
				ret = -EFAULT;
				break;
			}
		}

		kfree(md.ptr);
	}
		break;

	case ISPIOC_LUT_RW:
	{
		uint32_t s = 0;

		if (copy_from_user(&md, (void __user *)arg, sizeof(md))) {
			ret = -EFAULT;
			break;
		}
		s = md.elem & 0xffff;
		md.ptr = kzalloc(s, GFP_KERNEL);

		if (copy_from_user(md.ptr, pmd->ptr, s)) {
			ret = -EFAULT;
			break;
		}

		ret = acamera_api_calibration(md.chn, 0, md.id, md.dir, md.ptr, s, &ret_value);
		if (ret == SUCCESS && md.dir == COMMAND_GET) {
			if (copy_to_user(pmd->ptr, md.ptr, s)) {
				ret = -EFAULT;
				break;
			}
		}

		kfree(md.ptr);
	}
		break;

	case ISPIOC_COMMAND:
	{
        	uint8_t i = 0;
		uint8_t type = 0xff;
		uint32_t s = 0;
		struct kv_t *kv;
		if (copy_from_user(&md, (void __user *)arg, sizeof(md))) {
			ret = -EFAULT;
			break;
		}
		s = md.elem * sizeof(struct kv_t);
		md.ptr = kzalloc(s, GFP_KERNEL);

		if (copy_from_user(md.ptr, pmd->ptr, s)) {
			ret = -EFAULT;
			break;
		}

		kv = md.ptr;

		for (i = 0; i < md.elem; i++) {
			if (kv[i].v == CHECK_CODE) {
				type = kv[i].k;
				continue;
			}

			ret = acamera_command(md.chn, type, kv[i].k, kv[i].v, md.dir, &ret_value);
			if (ret == SUCCESS && md.dir == COMMAND_GET)
				kv[i].v = ret_value;
		}

		if (md.dir == COMMAND_GET) {
			if (copy_to_user(pmd->ptr, md.ptr, s)) {
				ret = -EFAULT;
				break;
			}
		}

		kfree(md.ptr);
	}
		break;

	case ISPIOC_GET_CTX:
	{
		isp_ctx_r_t ctx;
		isp_ctx_node_t *cn = NULL;
		acamera_context_t *p_ctx;

		if (copy_from_user(&ctx, (void __user *)arg, sizeof(ctx))) {
			ret = -EFAULT;
			break;
		}

		// isp context save on
		p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(ctx.ctx_id);
		if (ctx.type == ISP_CTX)
			p_ctx->isp_ctxsv_on = 1;
		else if (ctx.type == ISP_AE)
			p_ctx->isp_ae_stats_on = 1;
		else if (ctx.type == ISP_AWB)
			p_ctx->isp_awb_stats_on = 1;

		cn = isp_ctx_get_node(ctx.ctx_id, ctx.type, DONEQ);
		if (cn && copy_to_user((void __user *)arg, (void *)&cn->ctx, sizeof(ctx)))
			ret = -EFAULT;
	}
		break;

	case ISPIOC_PUT_CTX:
	{
		isp_ctx_r_t ctx;
		if (copy_from_user(&ctx, (void __user *)arg, sizeof(ctx))) {
			ret = -EFAULT;
			break;
		}
		isp_ctx_put(ctx.ctx_id, ctx.type, ctx.idx);
	}
		break;

	case ISPIOC_FILL_CTX:
	{
		volatile void *ptr;
		uint32_t crc = 0;
		isp_ctx_w_t ctx;
		acamera_context_t *p_ctx;

		if (copy_from_user(&ctx, (void __user *)arg, sizeof(ctx))) {
			ret = -EFAULT;
			break;
		}

		crc = crc16(~0, ctx.ptr, CTX_SIZE);
		if (crc != ctx.crc16) {
			LOG(LOG_ERR, "isp context set failed.");
			ret = -EFAULT;
			break;
		}

		p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(ctx.ctx_id);
		ptr = p_ctx->sw_reg_map.isp_sw_config_map + CTX_OFFSET;
		memcpy_toio(ptr, ctx.ptr, CTX_SIZE);
	}
		break;

	case ISPIOC_BUF_PACTET: {
		isp_packet_s packet;
		uint32_t buf[BUF_LENGTH];
		uint32_t *buf_m = NULL;

		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			ret = -1;
			break;
		}
		if (copy_from_user((void *)&packet, (void __user *)arg,
			sizeof(isp_packet_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			ret = -EINVAL;
			break;
		}
		if (packet.buf[0] < BUF_LENGTH * 4) {
			buf_m = buf;
		} else {
			buf_m = kzalloc(sizeof(uint32_t) * packet.buf[0], GFP_KERNEL);
			if (buf_m == NULL) {
				LOG(LOG_ERR, "kzalloc is failed!\n");
				ret = -EINVAL;
				break;
			}
		}
		memcpy(buf_m, packet.buf, sizeof(packet.buf));
		if (packet.pdata != NULL) {
			if (copy_from_user((void *)(buf_m + (sizeof(packet.buf)
				/ sizeof(uint32_t))), (void __user *)packet.pdata, packet.buf[4])) {
				LOG(LOG_ERR, "copy is err !\n");
				ret = -EINVAL;
				goto err_flag;
			}
		}
		process_ioctl_buf(buf_m);
		if (packet.pdata != NULL) {
			if (copy_to_user((void __user *)packet.pdata,
				(void *)(buf_m + (sizeof(packet.buf)
				/ sizeof(uint32_t))), packet.buf[4])) {
				LOG(LOG_ERR, "copy is err !\n");
				ret = -EINVAL;
				goto err_flag;
			}
		}
		if (copy_to_user((void __user *)arg, (void *)buf_m, sizeof(isp_packet_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			ret = -EINVAL;
		}
err_flag:
		if (packet.buf[0] >= (BUF_LENGTH * 4)) {
			kzfree(buf_m);
		}
	}
	break;

	default:
		LOG(LOG_ERR, "command %d not support.\n", cmd);
		break;
	}

	mutex_unlock(&isp_dev_ctx.fops_lock);

	return ret;
}

int isp_fops_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct isp_dev_context *p_ctx = (struct isp_dev_context *)file->private_data;

	if ((vma->vm_end - vma->vm_start) > p_ctx->mem_size) {
		LOG(LOG_ERR, "mmap size exceed valid range %d.", p_ctx->mem_size);
		return -ENOMEM;
	}
	vma->vm_flags |= VM_IO;
	vma->vm_flags |= VM_LOCKED;
	vma->vm_pgoff = p_ctx->phy_addr >> PAGE_SHIFT;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		LOG(LOG_ERR, "mmap phy addr %d failed.", p_ctx->phy_addr);
		return -EAGAIN;
	}

	return 0;
}

static struct file_operations isp_fops = {
	.owner = THIS_MODULE,
	.open = isp_fops_open,
	.release = isp_fops_release,
	.read = isp_fops_read,
	.write = isp_fops_write,
	.llseek = noop_llseek,
	.unlocked_ioctl = isp_fops_ioctl,
	.compat_ioctl = isp_fops_ioctl,
	.mmap = isp_fops_mmap,
};

void *isp_dev_get_vir_addr(void)
{
	return isp_dev_ctx.vir_addr;
}
EXPORT_SYMBOL(isp_dev_get_vir_addr);

int isp_dev_mem_alloc(void)
{
	int ret;
	struct isp_dev_context *p_ctx = NULL;

	p_ctx = &isp_dev_ctx;

	p_ctx->client = ion_client_create(hb_ion_dev, "ac_isp");
	if (IS_ERR(p_ctx->client)) {
		LOG(LOG_ERR, "ac_isp ion client create failed.");
		ret = -ENOMEM;
		goto out1;
	}

	p_ctx->handle = ion_alloc(p_ctx->client,
			TOTAL_MEM_SIZE, 0, ION_HEAP_CARVEOUT_MASK, 0);
	if (IS_ERR(p_ctx->handle)) {
		LOG(LOG_ERR, "ac_isp ion handle create failed.");
		ret = -ENOMEM;
		goto out1;
	}

	ret = ion_phys(p_ctx->client, p_ctx->handle->id,
			&p_ctx->phy_addr, &p_ctx->mem_size);
	if (ret) {
		LOG(LOG_ERR, "ion_phys get phy address failed.");
		goto out2;
	}
	p_ctx->vir_addr = ion_map_kernel(p_ctx->client, p_ctx->handle);
	if (IS_ERR(p_ctx->vir_addr)) {
		LOG(LOG_ERR, "ion_map failed.");
		ret = -ENOMEM;
		goto out2;
	}

	return 0;

out2:
	ion_free(p_ctx->client, p_ctx->handle);
	p_ctx->phy_addr = 0;
	p_ctx->mem_size = 0;
	ion_client_destroy(p_ctx->client);
out1:
	return ret;
}

static int isp_dev_context_init( struct isp_dev_context *p_ctx )
{
    int rc;

    p_ctx->isp_dev.minor = MISC_DYNAMIC_MINOR;
    p_ctx->isp_dev.fops = &isp_fops;

    rc = misc_register( &p_ctx->isp_dev );
    if ( rc ) {
        LOG( LOG_ERR, "Error: register ISP device failed, ret: %d.", rc );
        return rc;
    }

    p_ctx->dev_minor_id = p_ctx->isp_dev.minor;

    rc = kfifo_alloc( &p_ctx->isp_kfifo_in, SYSTEM_CHARDEV_FIFO_SIZE, GFP_KERNEL );
    if ( rc ) {
        LOG( LOG_ERR, "Error: kfifo_in alloc failed, ret: %d.", rc );
        goto failed_kfifo_in_alloc;
    }

    rc = kfifo_alloc( &p_ctx->isp_kfifo_out, SYSTEM_CHARDEV_FIFO_SIZE, GFP_KERNEL );
    if ( rc ) {
        LOG( LOG_ERR, "Error: kfifo_out alloc failed, ret: %d.", rc );
        goto failed_kfifo_out_alloc;
    }

    mutex_init( &p_ctx->fops_lock );
    init_waitqueue_head( &p_ctx->kfifo_in_queue );
    init_waitqueue_head( &p_ctx->kfifo_out_queue );

    rc = isp_dev_mem_alloc();
    if (rc)
	goto failed_kfifo_out_alloc;

    isp_ctx_queue_init();

    p_ctx->dev_inited = 1;

    LOG( LOG_INFO, "isp_dev_context(%s) init OK.", p_ctx->dev_name );

    return 0;

failed_kfifo_out_alloc:
    kfifo_free( &p_ctx->isp_kfifo_in );
failed_kfifo_in_alloc:
    misc_deregister( &p_ctx->isp_dev );

    return rc;
}

int system_chardev_init( void )
{
    int rc;

    struct isp_dev_context *p_ctx0 = NULL;

    LOG( LOG_INFO, "system init" );

    p_ctx0 = &isp_dev_ctx;
    memset( p_ctx0, 0, sizeof( *p_ctx0 ) );
    p_ctx0->isp_dev.name = SYSTEM_CHARDEV_NAME;
    p_ctx0->dev_name = SYSTEM_CHARDEV_NAME;
    rc = isp_dev_context_init( p_ctx0 );
    if ( rc ) {
        LOG( LOG_ERR, "Error: isp_dev_context_init failed for dev: %s.", p_ctx0->isp_dev.name );
        return rc;
    }

    return 0;
}

extern void *acamera_get_api_ctx_ptr( void );
extern int isp_open_check(void);
int system_chardev_read( char *data, int size )
{
    int rc;

    if ( !isp_dev_ctx.dev_inited ) {
        LOG( LOG_ERR, "dev is not inited, failed to read." );
        return -1;
    }

    mutex_lock( &isp_dev_ctx.fops_lock );

    if ( kfifo_is_empty( &isp_dev_ctx.isp_kfifo_in ) ) {
        long time_out_in_jiffies = 2;           /* jiffies is depend on HW, in x86 Ubuntu, it's 4 ms, 2 is 8ms. */
        mutex_unlock( &isp_dev_ctx.fops_lock ); /* unlock before we return or go sleeping */

        /* wait for the event */
        LOG( LOG_DEBUG, "input FIFO is empty, wait for data, timeout_in_jiffies: %ld, HZ: %d.", time_out_in_jiffies, HZ );
        // rc = wait_event_interruptible_timeout( isp_dev_ctx.kfifo_in_queue, !kfifo_is_empty( &isp_dev_ctx.isp_kfifo_in ), time_out_in_jiffies );
		rc = wait_event_interruptible( isp_dev_ctx.kfifo_in_queue, !kfifo_is_empty( &isp_dev_ctx.isp_kfifo_in ) );
        LOG( LOG_DEBUG, "data is coming or timeout, kfifo_in size: %u, rc: %d.", kfifo_len( &isp_dev_ctx.isp_kfifo_in ), rc );

        /* after wake up, we need to re-gain the mutex */
        mutex_lock( &isp_dev_ctx.fops_lock );
    }

	acamera_context_ptr_t context_ptr = (acamera_context_ptr_t)acamera_get_api_ctx_ptr();
	if (context_ptr->initialized == 0 || isp_open_check() == 0) {
		rc = -1;
	} else {
		rc = kfifo_out( &isp_dev_ctx.isp_kfifo_in, data, size );
	}

    mutex_unlock( &isp_dev_ctx.fops_lock );
    return rc;
}

int system_chardev_write( const char *data, int size )
{
    int rc;

    if ( !isp_dev_ctx.dev_inited ) {
        LOG( LOG_ERR, "dev is not inited, failed to write." );
        return -1;
    }

    mutex_lock( &isp_dev_ctx.fops_lock );

    rc = kfifo_in( &isp_dev_ctx.isp_kfifo_out, data, size );

    /* awake any reader */
    wake_up_interruptible( &isp_dev_ctx.kfifo_out_queue );
    LOG( LOG_DEBUG, "wake up reader who wait on kfifo out." );

    mutex_unlock( &isp_dev_ctx.fops_lock );
    return rc;
}

int system_chardev_destroy( void )
{
    if ( isp_dev_ctx.dev_inited ) {
        kfifo_free( &isp_dev_ctx.isp_kfifo_in );

        kfifo_free( &isp_dev_ctx.isp_kfifo_out );

        misc_deregister( &isp_dev_ctx.isp_dev );

	if (IS_ERR(isp_dev_ctx.client) == 0) {
		if (IS_ERR(isp_dev_ctx.handle) == 0) {
			ion_unmap_kernel(isp_dev_ctx.client, isp_dev_ctx.handle);
			ion_free(isp_dev_ctx.client, isp_dev_ctx.handle);
		}
		ion_client_destroy(isp_dev_ctx.client);
	}

        LOG( LOG_INFO, "misc_deregister dev: %s.", isp_dev_ctx.dev_name );
    } else {
        LOG( LOG_INFO, "dev not inited, do nothing." );
    }

    return 0;
}
