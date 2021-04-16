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

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <asm/io.h>

#include "system_log.h"
#include "gdc_drv.h"


#ifdef BUILD_GDC0
#define GDC_STRING  "hobot,gdc0"
#else
#define GDC_STRING  "hobot,gdc1"
#endif

//entry functions to gdc_main
extern int gdc_fw_init( void );
extern void gdc_fw_exit( void );
extern int gdc_process(gdc_process_settings_t *gdc_process_setting);
extern void gdc_force_stop(void);
extern void gdc_get_status(gdc_status_t *status_update);

//need to set system dependent irq and memory area
extern void system_interrupts_set_irq( int irq_num, int flags );
extern int32_t init_gdc_io( resource_size_t addr , resource_size_t size );


typedef struct gdc_drv_context_t {
    u32 open_count;                     /*!<< device reference count.*/
    gdc_status_t gdc_status;
} gdc_drv_context_t;

DEFINE_MUTEX(fops_lock);
static gdc_drv_context_t s_gdc_drv_context;

static int gdc_open(struct inode *inode, struct file *filp)
{
    LOG(LOG_INFO, "+\n");

    mutex_lock(&fops_lock);
    s_gdc_drv_context.open_count++;
    filp->private_data = (void *)(&s_gdc_drv_context);
    mutex_unlock(&fops_lock);

    LOG(LOG_INFO, "- : open_count = %d \n", s_gdc_drv_context.open_count);

    return 0;
}

static int gdc_release(struct inode *inode, struct file *filp)
{
    LOG(LOG_INFO,"+\n");

    mutex_lock(&fops_lock);
    s_gdc_drv_context.open_count--;
    mutex_unlock(&fops_lock);

    LOG(LOG_INFO,"- : open_count = %d \n", s_gdc_drv_context.open_count);

    return 0;
}


static long gdc_ioctl(struct file *filp, u_int cmd, u_long arg)
{
    int ret = 0;
    gdc_process_settings_t process_setting;
    switch (cmd) {
        case GDC_IOCTL_PROCESS:
            LOG(LOG_DEBUG,"GDC_IOCTL_PROCESS： + \n");
            mutex_lock(&fops_lock);
            ret = copy_from_user(&process_setting,
                                 (gdc_process_settings_t *)arg,
                                 sizeof(gdc_process_settings_t));
            if (ret) {
                LOG(LOG_ERR,"GDC_IOCTL_PROCESS： copy from user failed (ret=%d) \n", ret);
                mutex_unlock(&fops_lock);
                return -EFAULT;
            }
            ret = gdc_process(&process_setting);
            if(ret == 0) {
                s_gdc_drv_context.gdc_status.gdc_process_setting = process_setting;
                s_gdc_drv_context.gdc_status.status_result = gdc_busy;
            }
            mutex_unlock(&fops_lock);
            LOG(LOG_DEBUG,"GDC_IOCTL_PROCESS： - \n");
            break;

        case GDC_IOCTL_GET_STATUS:
            LOG(LOG_DEBUG,"GDC_IOCTL_GET_STATUS + \n");
            mutex_lock(&fops_lock);
            gdc_get_status(&s_gdc_drv_context.gdc_status);
            LOG(LOG_INFO,"GDC_IOCTL_GET_STATUS: sensor_id=%d, frame_id=%d, time_stamp=%llu, status=%d \n",
                    s_gdc_drv_context.gdc_status.gdc_process_setting.image_info.sensor_id,
                    s_gdc_drv_context.gdc_status.gdc_process_setting.image_info.frame_id,
                    s_gdc_drv_context.gdc_status.gdc_process_setting.image_info.time_stamp,
                    s_gdc_drv_context.gdc_status.status_result);
            ret = copy_to_user((void __user *)arg, &s_gdc_drv_context.gdc_status, sizeof(gdc_status_t));
            if (ret) {
                LOG(LOG_ERR,"GDC_IOCTL_GET_STATUS： copy to user failed (ret=%d) \n", ret);
                mutex_unlock(&fops_lock);
                return -EFAULT;
            }
            mutex_unlock(&fops_lock);
            LOG(LOG_DEBUG,"GDC_IOCTL_GET_STATUS - \n");
            break;

        case GDC_IOCTL_FORCE_STOP:
            LOG(LOG_DEBUG,"GDC_IOCTL_FORCE_STOP + \n");
            mutex_lock(&fops_lock);
            gdc_force_stop();
            mutex_unlock(&fops_lock);
            LOG(LOG_DEBUG,"GDC_IOCTL_FORCE_STOP - \n");
            break;

        default:
            LOG(LOG_ERR, "No such IOCTL, cmd is %d\n", cmd);
            ret = -EFAULT;
            break;
    }

    return ret;
}


static const struct of_device_id gdc_dt_match[] = {
    {.compatible = GDC_STRING},
    {}};

MODULE_DEVICE_TABLE( of, gdc_dt_match );

static struct platform_driver gdc_platform_driver = {
    .driver = {
        .name = GDC_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = gdc_dt_match,
    },
};

struct file_operations gdc_fops = {
    .owner = THIS_MODULE,
    .open = gdc_open,
    .unlocked_ioctl = gdc_ioctl,
    .release = gdc_release,
};

static int s_gdc_major;
static struct cdev s_gdc_cdev;

static int32_t gdc_platform_probe( struct platform_device *pdev )
{
    int32_t rc = 0;
    struct resource *gdc_res;

    // Initialize irq
	printk("%s.%d",__FUNCTION__,__LINE__);
    gdc_res = platform_get_resource( pdev, IORESOURCE_IRQ, 0 );

    if ( gdc_res ) {
        LOG( LOG_INFO, "gdc irq = %d, flags = 0x%x !\n", (int)gdc_res->start, (int)gdc_res->flags );
        system_interrupts_set_irq( gdc_res->start, gdc_res->flags );
    } else {
        LOG( LOG_ERR, "Error, no gdc irq found from DT\n" );
        return -1;
    }

    // Initialize gdc register
    gdc_res = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    if(gdc_res){
        LOG( LOG_INFO, "gdc address = 0x%x, end = 0x%x !\n", (int)gdc_res->start, (int)gdc_res->end );
        if(init_gdc_io(gdc_res->start,gdc_res->end-gdc_res->start)!=0){
            LOG( LOG_ERR, "Error on mapping gdc memory! \n" );
        }
    }else{
        LOG( LOG_ERR, "Error, no IORESOURCE_MEM DT!\n" );
    }

    // get the major number of the character device
    if ((alloc_chrdev_region(&s_gdc_major, 0, 1, GDC_DEV_NAME)) < 0) {
        LOG(LOG_ERR,  "could not allocate major number\n");
        return -EBUSY;
    }

    // initialize the device structure and register the device with the kernel
    cdev_init(&s_gdc_cdev, &gdc_fops);
    if ((cdev_add(&s_gdc_cdev, s_gdc_major, 1)) < 0) {
        LOG(LOG_ERR, "could not allocate chrdev\n");
        return -EBUSY;
    }

    gdc_fw_init();
    return rc;
}

static int __init fw_module_init( void )
{
    int32_t rc = 0;
    LOG( LOG_INFO, "Juno gdc fw_module_init\n" );
printk("%s.%d",__FUNCTION__,__LINE__);
    // init driver sturcture
    memset(&s_gdc_drv_context,0,sizeof(gdc_drv_context_t));
    rc = platform_driver_probe( &gdc_platform_driver,
                                gdc_platform_probe );
    return rc;
}

static void __exit fw_module_exit( void )
{
    LOG( LOG_ERR, "Juno gdc fw_module_exit\n" );
    gdc_fw_exit();
    platform_driver_unregister( &gdc_platform_driver );
}

module_init( fw_module_init );
module_exit( fw_module_exit );
MODULE_LICENSE( "GPL v2" );
MODULE_AUTHOR( "ARM IVG" );
