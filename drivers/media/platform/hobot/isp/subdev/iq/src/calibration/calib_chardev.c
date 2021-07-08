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

#include "calib_chardev.h"
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "system_timer.h"
#include "acamera_firmware_settings.h"

#define CHARDEV_CALIB_NAME "ac_calib"


// PRQA S 0591 ++
#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_SOC_IQ
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_IQ
#endif


/* First item for ACT Control, second for user-FW */
static struct calib_dev_s calib_dev_ctx;
static struct calib_param_s calib_param_ctx;

extern uint32_t get_calibrations_static_fs_lin_dummy( ACameraCalibrations *c );
extern uint32_t get_calibrations_dynamic_fs_lin_dummy( ACameraCalibrations *c );

extern void *soc_iq_get_lut_data_ptr(uint8_t ctx_id);

static int calib_destory(uint8_t port)
{
	uint32_t tmp = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG( LOG_ERR, "port %d is not existance.", port );
		return -CALIB_PORT_ERR;
	}
	struct calib_data_s *calib_data = calib_param_ctx.plist[port];
	if(calib_data == NULL) {
		LOG( LOG_ERR, "calib_data is null. port %d", port );
		return -CALIB_NULL_ERR;
	}
	LOG( LOG_INFO, "calibration deinit is runing.");
	ACameraCalibrations *c = soc_iq_get_lut_data_ptr(port);
	if (c != NULL) {
		get_calibrations_dynamic_fs_lin_dummy(c);
		get_calibrations_static_fs_lin_dummy(c);
	}

	for(tmp = 0; tmp < CALIBRATION_TOTAL_SIZE; tmp++) {
		if (calib_data->plut[tmp].ptr == NULL) {
			LOG( LOG_DEBUG, "memory of port %d is not existance.", port );
		} else {
			LOG( LOG_DEBUG, "%d memory of port %d is free.", tmp, port );
			kfree(calib_data->plut[tmp].ptr);
			calib_data->plut[tmp].ptr = NULL;
		}	
	}

	kfree(calib_data);
	calib_param_ctx.plist[port] = NULL;
	calib_param_ctx.p_num &= ~(1 << port);

	LOG( LOG_DEBUG, "calib destory is success.");
	
	return 0;
}

static int calib_create(camera_calib_t *ptr)
{
	int ret = 0;
	uint32_t tmp = 0;
	uint32_t tmp_c = 0;
	void *ptmp = NULL;
	struct calib_data_s *calib_data = NULL;
	
	LOG( LOG_INFO, "calibration init is runing.");
	
	if ( ptr->port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG( LOG_ERR, "port %d is not existance.", ptr->port );
		return -CALIB_PORT_ERR;
	}

	if (calib_param_ctx.plist[ptr->port]) {
		ret = calib_destory(ptr->port);
	}

	calib_data = kzalloc(sizeof(struct calib_data_s), GFP_KERNEL);
	if (!calib_data) {
		LOG( LOG_ERR, "calib_data malloc is failed.");
		return -CALIB_MALLOC_ERR;
	} else {
		LOG( LOG_DEBUG, "malloc dev %p.", calib_data );
	}

	memcpy(calib_data->name, ptr->name, CALIB_NUM_LENGTH);	
	if ( !ptr->plut) {
		LOG( LOG_ERR, "ptr is null!");
		ret = -CALIB_NULL_ERR;
		goto copy_err;
	}	
	//copy_from_user should small 4096bits
	if (copy_from_user((void *)calib_data->plut, (void __user *)ptr->plut,
		sizeof(LookupTable) * CALIBRATION_TOTAL_SIZE)) {
		LOG(LOG_ERR, "copy data is failed.");
		ret = -CALIB_COPY_ERR;
		goto copy_err;
	}
		
	for(tmp = 0; tmp < CALIBRATION_TOTAL_SIZE; tmp++) {
		if (calib_data->plut[tmp].ptr != NULL) {

			tmp_c = (calib_data->plut[tmp].rows * 
				calib_data->plut[tmp].cols *
				calib_data->plut[tmp].width );
 
			ptmp = NULL;
			ptmp = kzalloc(tmp_c, GFP_KERNEL);
			if (!ptmp) {
				LOG( LOG_ERR, "%s calib_data malloc is failed.", __func__ );
				ret = -CALIB_MALLOC_ERR;
				goto copy_err;
			} else {
				if (copy_from_user((void *)ptmp, (void __user *)calib_data->plut[tmp].ptr, tmp_c)) {
					kfree(ptmp);
					calib_data->plut[tmp].ptr = NULL;
					LOG( LOG_ERR, "num %d copy is failed.", tmp);
					ret = -CALIB_COPY_ERR;
					goto copy_err;
				} else {
					calib_data->plut[tmp].ptr = ptmp;	
					LOG( LOG_DEBUG, "malloc dev %p.", calib_data );
				}
				ptmp = NULL;
			}

		} else {
			LOG( LOG_INFO, "num %d is null.", tmp);
		}

		LOG( LOG_DEBUG, "the num of %d calib data is in addr %p, rows %d, cols %d, width %d!", tmp, calib_data->plut[tmp].ptr, calib_data->plut[tmp].rows, calib_data->plut[tmp].cols, calib_data->plut[tmp].width);	
	}

	calib_param_ctx.plist[ptr->port] = calib_data; 
	calib_param_ctx.p_num |= (1 << ptr->port);  			
	
	LOG( LOG_INFO, "calibration is init success.");
	
	return ret;
copy_err:
	calib_destory(ptr->port);
	return ret;
}

static int calib_setpart(camera_calib_t *ptr)
{
	int ret = 0;
	uint32_t tmp = 0;
	uint32_t tmp_c = 0;
	void *ptmp = NULL;

	if (ptr->port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG( LOG_ERR, "port %d is not existance.", ptr->port );
		return -CALIB_PORT_ERR;
	}
	struct calib_data_s *calib_data = calib_param_ctx.plist[ptr->port];
	if(calib_data == NULL) {
		LOG( LOG_ERR, "calib_data is null. port %d", ptr->port );
		return -CALIB_NULL_ERR;
	}
	LOG( LOG_INFO, "%s is runing.", __func__ );
	tmp = ptr->num;
	if (calib_data->plut[tmp].ptr != NULL) {
		ptmp = calib_data->plut[tmp].ptr;
		tmp_c = (calib_data->plut[tmp].rows * 
			calib_data->plut[tmp].cols *
			calib_data->plut[tmp].width );
 
		if (tmp_c != ptr->tsize) {
			LOG( LOG_ERR, "%s  size is error.", __func__);
			return -CALIB_SIZE_ERR;
		} else {
			LOG( LOG_DEBUG, "%s the num is %d, the size is %d .",
				__func__, tmp, tmp_c );
		}
	
		if (copy_from_user((void *)ptmp, (void __user *)ptr->pstr, ptr->tsize)) {
			kfree(ptmp);
			calib_data->plut[tmp].ptr = NULL;
			LOG( LOG_ERR, "%s -- %d copy data is failed.", __func__, __LINE__);
			ret = -CALIB_COPY_ERR;
		}
	} else {
		LOG( LOG_ERR, "the calib of %d is null.", tmp);
		ret = -CALIB_NULL_ERR;
	}
		
	return ret;
}

static int calib_getpart(camera_calib_t *ptr)
{
	int ret = 0;
	uint32_t tmp = 0;
	char *ptmp = NULL;
	uint32_t tmp_c = 0;

	if (ptr->port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG( LOG_ERR, "port %d is not existance.", ptr->port );
		return -CALIB_PORT_ERR;
	}
	struct calib_data_s *calib_data = calib_param_ctx.plist[ptr->port];
	if(calib_data == NULL) {
		LOG( LOG_ERR, "calib_data is null. port %d", ptr->port );
		return -CALIB_NULL_ERR;
	}
	LOG( LOG_INFO, "%s is runing.", __func__ );
	tmp = ptr->num;
	if (calib_data->plut[tmp].ptr != NULL) {
		ptmp = calib_data->plut[tmp].ptr;
		tmp_c = (calib_data->plut[tmp].rows * 
			calib_data->plut[tmp].cols *
			calib_data->plut[tmp].width );
 
		if (tmp_c != ptr->tsize) {
			LOG( LOG_ERR, "%s  size is error.", __func__);
			return -CALIB_SIZE_ERR;
		} else {
			LOG( LOG_DEBUG, "the num is %d, the size is %d.", tmp, tmp_c );
		}
	
		if (copy_to_user((void __user *)ptr->pstr, (void *)ptmp, calib_data->tsize)) {
			LOG( LOG_ERR, "%s -- %d copy data is failed.", __func__, __LINE__);
			ret = -CALIB_COPY_ERR;
		}
	} else {
		LOG( LOG_ERR, "the calib of %d is null.", tmp);
		ret = -CALIB_NULL_ERR;
	}
		
	return ret;
}

int register_calib( ACameraCalibrations *c, uint8_t port )
{
	int ret = 0;
	uint32_t tmp = 0;
	LookupTable *ptmp = NULL;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG( LOG_ERR, "port %d is not existance.", port );
		return -CALIB_PORT_ERR;
	}
	struct calib_data_s *calib_data = calib_param_ctx.plist[port];
	if(calib_data == NULL) {
		LOG( LOG_ERR, "calib_data is null. port %d", port );
		return -CALIB_NULL_ERR;
	}
	LOG( LOG_INFO, "%s is runing.", __func__ );
	ptmp = calib_data->plut;
	for(tmp = 0; tmp < CALIBRATION_TOTAL_SIZE; tmp++) {
		if (calib_data->plut[tmp].ptr != NULL) { 
			c->calibrations[tmp] = ptmp;
		} else {
			c->calibrations[tmp] = NULL;
		}
		ptmp++;	
	}

	LOG( LOG_INFO, "calibration of port %d is register success !", port );
		
	return ret;
}

int unregister_calib( ACameraCalibrations *c, uint8_t port )
{
	int ret = 0;
	uint32_t tmp = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG( LOG_ERR, "port %d is not existance.", port );
		return -CALIB_PORT_ERR;
	}
	struct calib_data_s *calib_data = calib_param_ctx.plist[port];
	if(calib_data == NULL) {
		LOG( LOG_ERR, "calib_data is null. port %d", port );
		return -CALIB_NULL_ERR;
	}
	LOG( LOG_INFO, "%s is runing.", __func__ );
	for(tmp = 0; tmp < CALIBRATION_TOTAL_SIZE; tmp++) {
		c->calibrations[tmp] = NULL;
	}

	LOG( LOG_INFO, "calibration of port %d is unregister success !", port );
		
	return ret;
}

static int calib_fops_open( struct inode *inode, struct file *f )
{
    int rc;
    struct calib_dev_s *p_ctx = NULL;
    int minor = iminor( inode );

    LOG( LOG_INFO, "client is opening..., minor: %d.", minor );

    if ( !calib_dev_ctx.dev_inited ) {
        LOG( LOG_ERR, "dev is not inited, failed to open." );
        return -ERESTARTSYS;
    }

    if ( minor == calib_dev_ctx.dev_minor_id )
        p_ctx = &calib_dev_ctx;

    if ( !p_ctx ) {
        LOG( LOG_ERR,"Fatal error, calib dev contexts is wrong, contents dump:" );
        LOG( LOG_ERR, "minor_id: %d, name: %s.", calib_dev_ctx.dev_minor_id, calib_dev_ctx.dev_name );

        return -ERESTARTSYS;
    }

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR,"Error: lock failed of dev: %s.", p_ctx->dev_name );
        goto lock_failure;
    }

        p_ctx->dev_opened++;
        f->private_data = p_ctx;
#if 0
    if ( p_ctx->dev_opened ) {
        LOG( LOG_ERR,"open(%s) failed, already opened.", p_ctx->dev_name );
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

static int calib_fops_release( struct inode *inode, struct file *f )
{
    int rc;
    struct calib_dev_s *p_ctx = (struct calib_dev_s *)f->private_data;

    if ( p_ctx != &calib_dev_ctx ) {
        LOG( LOG_ERR, "Inalid paramter: %p.", p_ctx );
        return -EINVAL;
    }

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed of dev: %s.", p_ctx->dev_name );
        return rc;
    }

    if ( p_ctx->dev_opened ) {
        p_ctx->dev_opened--;
        f->private_data = NULL;
    } else {
        LOG( LOG_ERR, "Fatal error: wrong state of dev: %s, dev_opened: %d.", p_ctx->dev_name, p_ctx->dev_opened );
        rc = -EINVAL;
    }

    mutex_unlock( &p_ctx->fops_lock );

    return 0;
}

static ssize_t calib_fops_write( struct file *file, const char __user *buf, size_t count, loff_t *ppos )
{
    int rc = 0;
    //struct calib_dev_s *p_ctx = (struct calib_dev_s *)file->private_data;
    
    return rc;
}

static ssize_t calib_fops_read( struct file *file, char __user *buf, size_t count, loff_t *ppos )
{
    int rc = 0;
    //struct calib_dev_s *p_ctx = (struct calib_dev_s *)file->private_data;

    return rc;
}

static long calib_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	struct calib_dev_s *p_ctx = (struct calib_dev_s *)file->private_data;
	camera_calib_t pcalib;
		
	rc = mutex_lock_interruptible( &p_ctx->fops_lock );
	
	switch (cmd) {
	case AC_CALIB_INIT: {
		if (!arg) {
			goto arg_err;
		}
		if (copy_from_user((void *)&pcalib, (void __user *)arg, sizeof(camera_calib_t))) {
			LOG( LOG_ERR, "%s -- %d copy data is failed.", __func__, __LINE__);
			goto copy_err;
		}
		rc = calib_create(&pcalib);	
	}
	break;
	case AC_CALIB_RELEASE: {
		if (!arg)
			goto arg_err;
		
		if (copy_from_user((void *)&pcalib, (void __user *)arg, sizeof(camera_calib_t))) {
			LOG( LOG_ERR, "%s -- %d copy data is failed.", __func__, __LINE__);
			goto copy_err;
		}
		rc = calib_destory(pcalib.port);	
	}
	break;
	case AC_CALIB_SETPART: {
		if (!arg)
			goto arg_err;
		
		if (copy_from_user((void *)&pcalib, (void __user *)arg, sizeof(camera_calib_t))) {
			LOG( LOG_ERR, "%s -- %d copy data is failed.", __func__, __LINE__);
			goto copy_err;
		}
		rc = calib_setpart(&pcalib);	
	}
	break;
	case AC_CALIB_GETPART: {
		if (!arg)
			goto arg_err;
		
		if (copy_from_user((void *)&pcalib, (void __user *)arg, sizeof(camera_calib_t))) {
			LOG( LOG_ERR, "%s -- %d copy data is failed.", __func__, __LINE__);
			goto copy_err;
		}
		rc = calib_getpart(&pcalib);	
	}
	break;
	default :
        	LOG( LOG_ERR, "%s -- %d cmd is error.", __func__, __LINE__ );
	}

	mutex_unlock( &p_ctx->fops_lock );
        return rc;

arg_err:
	rc = -CALIB_NULL_ERR;
copy_err:
	rc = -CALIB_COPY_ERR;
	mutex_unlock( &p_ctx->fops_lock );
	return -1;
}


static struct file_operations calib_fops = {
    .owner = THIS_MODULE,
    .open = calib_fops_open,
    .release = calib_fops_release,
    .read = calib_fops_read,
    .write = calib_fops_write,
    .unlocked_ioctl = calib_fops_ioctl,
    .compat_ioctl   = calib_fops_ioctl,
    //.llseek = noop_llseek,
};

static int calib_dev_s_init( struct calib_dev_s *p_ctx )
{
    int rc;

    LOG( LOG_INFO, "%s is runing.", __func__ );
    
    p_ctx->calib_dev.minor = MISC_DYNAMIC_MINOR;
    p_ctx->calib_dev.fops = &calib_fops;

    rc = misc_register( &p_ctx->calib_dev );
    if ( rc ) {
        LOG( LOG_ERR, "Error: register CALIB device failed, ret: %d.", rc );
        return rc;
    }

    p_ctx->dev_minor_id = p_ctx->calib_dev.minor;

    mutex_init( &p_ctx->fops_lock );

    p_ctx->dev_inited = 1;

    LOG( LOG_INFO, "calib_dev_s(%s) init OK.", p_ctx->dev_name );

    return 0;
}

int system_calib_init( void )
{
    int rc;

    struct calib_dev_s *p_ctx0 = NULL;

    LOG( LOG_INFO, "%s is runing.", __func__ );

    p_ctx0 = &calib_dev_ctx;
    memset( p_ctx0, 0, sizeof( *p_ctx0 ) );
    memset( &calib_param_ctx, 0, sizeof(struct calib_param_s) );
    p_ctx0->calib_dev.name = CHARDEV_CALIB_NAME;
    p_ctx0->dev_name = CHARDEV_CALIB_NAME;
    rc = calib_dev_s_init( p_ctx0 );
    if ( rc ) {
        LOG( LOG_ERR, "Error: calib_dev_s_init failed for dev: %s.", p_ctx0->calib_dev.name );
        return rc;
    }

    return 0;
}

void system_calib_destroy( void )
{
    uint32_t tmp = 0;
   
    LOG( LOG_INFO, "%s is runing.", __func__ );
    
    if ( calib_dev_ctx.dev_inited ) {
	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER;tmp++) {
		calib_destory(tmp);
	}
        misc_deregister( &calib_dev_ctx.calib_dev );
        LOG( LOG_INFO, "misc_deregister dev: %s.", calib_dev_ctx.dev_name );
    } else {
        LOG( LOG_INFO, "dev not inited, do nothing." );
    }
}

// PRQA S --
