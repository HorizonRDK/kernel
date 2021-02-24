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
#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kfifo.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include "acamera_logger.h"
#include "system_stdlib.h"
#include "acamera_command_api.h"
#include "acamera_ctrl_channel.h"

#define CTRL_CHANNEL_FIFO_INPUT_SIZE ( 4 * 1024 )
#define CTRL_CHANNEL_FIFO_OUTPUT_SIZE ( 8 * 1024 )

struct ctrl_channel_dev_context {
    uint8_t dev_inited;
    int dev_minor_id;
    char dev_name[16];
    int dev_opened;

    struct device *ctrl_dev;
    dev_t ctrl_devno;
    struct cdev ctrl_cdev;
    struct mutex fops_lock;

    struct kfifo ctrl_kfifo_in;
    struct kfifo ctrl_kfifo_out;

    struct ctrl_cmd_item cmd_item;
};

static struct ctrl_channel_dev_context ctrl_channel_ctx[FIRMWARE_CONTEXT_NUMBER];
extern struct class *vps_class;

static int ctrl_channel_fops_open( struct inode *inode, struct file *f )
{
    int rc;
    int minor = iminor( inode );
    struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx[minor];

    LOG( LOG_INFO, "client is opening..., minor: %d.", minor );

    if ( minor != p_ctx->dev_minor_id ) {
        LOG( LOG_ERR, "Not matched ID, minor_id: %d, exptect: %d(dev name: %s)", minor, p_ctx->dev_minor_id, p_ctx->dev_name );
        return -ERESTARTSYS;
    }

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed of dev: %s.", p_ctx->dev_name );
        goto lock_failure;
    }

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

    mutex_unlock( &p_ctx->fops_lock );

lock_failure:
    return rc;
}

static int ctrl_channel_fops_release( struct inode *inode, struct file *f )
{
    int rc;
    int minor = iminor( inode );
    struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx[minor];

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed of dev: %s.", p_ctx->dev_name );
        return rc;
    }

    if ( p_ctx->dev_opened ) {
        p_ctx->dev_opened = 0;
        f->private_data = NULL;
        kfifo_reset( &p_ctx->ctrl_kfifo_in );
        kfifo_reset( &p_ctx->ctrl_kfifo_out );
        LOG( LOG_INFO, "close(%s) succeed, private_data: %p.", p_ctx->dev_name, p_ctx );
    } else {
        LOG( LOG_ERR, "Fatal error: wrong state of dev: %s, dev_opened: %d.", p_ctx->dev_name, p_ctx->dev_opened );
        rc = -EINVAL;
    }

    mutex_unlock( &p_ctx->fops_lock );

    return 0;
}

static ssize_t ctrl_channel_fops_write( struct file *file, const char __user *buf, size_t count, loff_t *ppos )
{
    int rc;
    unsigned int copied;
    struct ctrl_channel_dev_context *p_ctx = (struct ctrl_channel_dev_context *)file->private_data;

    if (p_ctx->dev_opened == 0) {
        LOG( LOG_ERR, "ctrl channel ac_isp4uf[%d] is not opened", p_ctx->dev_minor_id);
        return -EINVAL;
    }

    if ( mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
        LOG( LOG_ERR, "Fatal error: access lock failed." );
        return -ERESTARTSYS;
    }

    rc = kfifo_from_user( &p_ctx->ctrl_kfifo_in, buf, count, &copied );

    mutex_unlock( &p_ctx->fops_lock );

    LOG( LOG_DEBUG, "wake up reader." );

    return rc ? rc : copied;
}

static ssize_t ctrl_channel_fops_read( struct file *file, char __user *buf, size_t count, loff_t *ppos )
{
    int rc = 0;
    unsigned int copied = 0;
    struct ctrl_channel_dev_context *p_ctx = (struct ctrl_channel_dev_context *)file->private_data;

    if (p_ctx->dev_opened == 0) {
        LOG( LOG_ERR, "ctrl channel ac_isp4uf[%d] is not opened", p_ctx->dev_minor_id);
        return -EINVAL;
    }

    if ( mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
        LOG( LOG_ERR, "Fatal error: access lock failed." );
        return -ERESTARTSYS;
    }

    if ( !kfifo_is_empty( &p_ctx->ctrl_kfifo_out ) ) {
        rc = kfifo_to_user( &p_ctx->ctrl_kfifo_out, buf, count, &copied );
    }

    mutex_unlock( &p_ctx->fops_lock );

    return rc ? rc : copied;
}

static struct file_operations isp_fops = {
    .owner = THIS_MODULE,
    .open = ctrl_channel_fops_open,
    .release = ctrl_channel_fops_release,
    .read = ctrl_channel_fops_read,
    .write = ctrl_channel_fops_write,
    .llseek = noop_llseek,
};

#if defined( CTRL_CHANNEL_BIDIRECTION )
int ctrl_channel_read( char *data, int size )
{
    int rc = 0;

    if ( !ctrl_channel_ctx.dev_inited ) {
        LOG( LOG_ERR, "dev is not inited, failed to read." );
        return -1;
    }

    mutex_lock( &ctrl_channel_ctx.fops_lock );

    if ( !kfifo_is_empty( &ctrl_channel_ctx.ctrl_kfifo_in ) ) {
        rc = kfifo_out( &ctrl_channel_ctx.ctrl_kfifo_in, data, size );
    }

    mutex_unlock( &ctrl_channel_ctx.fops_lock );

    return rc;
}
#endif

static int ctrl_channel_write( const struct ctrl_cmd_item *p_cmd, const void *data, uint32_t data_size )
{
    int rc;
    int left_len = 0;

    if ( !ctrl_channel_ctx[p_cmd->cmd_ctx_id].dev_inited ) {
        LOG( LOG_ERR, "dev is not inited, failed to write." );
        return -1;
    }

    mutex_lock( &ctrl_channel_ctx[p_cmd->cmd_ctx_id].fops_lock );

    if (kfifo_is_full(&ctrl_channel_ctx[p_cmd->cmd_ctx_id].ctrl_kfifo_out)) {
        pr_err("fifo full, skip this set:\n");
        pr_err("ctx id: %u, category: %u, cmd_id: %u, cmd_direction: %u, cmd_value: %u.",
        p_cmd->cmd_ctx_id, p_cmd->cmd_category, p_cmd->cmd_id, p_cmd->cmd_direction, p_cmd->cmd_value);
        mutex_unlock( &ctrl_channel_ctx[p_cmd->cmd_ctx_id].fops_lock );
        return -1;
    }

    left_len = kfifo_avail(&ctrl_channel_ctx[p_cmd->cmd_ctx_id].ctrl_kfifo_out);
    if (left_len < data_size + sizeof(struct ctrl_cmd_item)) {
        pr_err("fifo no enough space, skip this set:\n");
        pr_err("ctx id: %u, category: %u, cmd_id: %u, cmd_direction: %u, cmd_value: %u.",
        p_cmd->cmd_ctx_id, p_cmd->cmd_category, p_cmd->cmd_id, p_cmd->cmd_direction, p_cmd->cmd_value);
        mutex_unlock( &ctrl_channel_ctx[p_cmd->cmd_ctx_id].fops_lock );
        return -1;
    }

    rc = kfifo_in( &ctrl_channel_ctx[p_cmd->cmd_ctx_id].ctrl_kfifo_out, p_cmd, sizeof( struct ctrl_cmd_item ) );
    if ( data )
        kfifo_in( &ctrl_channel_ctx[p_cmd->cmd_ctx_id].ctrl_kfifo_out, data, data_size );

    mutex_unlock( &ctrl_channel_ctx[p_cmd->cmd_ctx_id].fops_lock );

    return rc;
}

int ctrl_channel_init(int ctx_id)
{
    int rc;

    if (ctx_id >= FIRMWARE_CONTEXT_NUMBER) {
        rc = -1;
        pr_err("ctx_id %d exceed valid range\n", ctx_id);
        return rc;
    }

    memset( &ctrl_channel_ctx[ctx_id], 0, sizeof(struct ctrl_channel_dev_context) );
    sprintf(ctrl_channel_ctx[ctx_id].dev_name, CTRL_CHANNEL_DEV_NAME, ctx_id);

    rc = alloc_chrdev_region(&ctrl_channel_ctx[ctx_id].ctrl_devno, ctx_id, 1, ctrl_channel_ctx[ctx_id].dev_name);
    if (rc < 0) {
        LOG( LOG_ERR, "Error: register ISP ctrl channel device failed, ret: %d.", rc );
        return rc;
    }

    cdev_init(&ctrl_channel_ctx[ctx_id].ctrl_cdev, &isp_fops);
    ctrl_channel_ctx[ctx_id].ctrl_cdev.owner = THIS_MODULE;
    rc = cdev_add(&ctrl_channel_ctx[ctx_id].ctrl_cdev, ctrl_channel_ctx[ctx_id].ctrl_devno, 1);
    if (rc < 0) {
        LOG( LOG_ERR, "Error: cdev add ISP ctrl channel device failed, ret: %d.", rc );
        unregister_chrdev_region(ctrl_channel_ctx[ctx_id].ctrl_devno, 1);
        return rc;
    }
    ctrl_channel_ctx[ctx_id].dev_minor_id = MINOR(ctrl_channel_ctx[ctx_id].ctrl_devno);

    ctrl_channel_ctx[ctx_id].ctrl_dev = device_create(vps_class, NULL,
                    MKDEV(MAJOR(ctrl_channel_ctx[ctx_id].ctrl_devno), ctx_id),
                    NULL, ctrl_channel_ctx[ctx_id].dev_name);
    if (IS_ERR(ctrl_channel_ctx[ctx_id].ctrl_dev)) {
            rc = -EINVAL;
            pr_err("create device %s failed\n", ctrl_channel_ctx[ctx_id].dev_name);
            goto failed_kfifo_in_alloc;
    }

    mutex_init( &ctrl_channel_ctx[ctx_id].fops_lock );

    rc = kfifo_alloc( &ctrl_channel_ctx[ctx_id].ctrl_kfifo_in, CTRL_CHANNEL_FIFO_INPUT_SIZE, GFP_KERNEL );
    if ( rc ) {
        LOG( LOG_ERR, "Error: kfifo_in alloc failed, ret: %d.", rc );
        goto failed_kfifo_in_alloc;
    }

    rc = kfifo_alloc( &ctrl_channel_ctx[ctx_id].ctrl_kfifo_out, CTRL_CHANNEL_FIFO_OUTPUT_SIZE, GFP_KERNEL );
    if ( rc ) {
        LOG( LOG_ERR, "Error: kfifo_out alloc failed, ret: %d.", rc );
        goto failed_kfifo_out_alloc;
    }

    ctrl_channel_ctx[ctx_id].dev_inited = 1;

    pr_debug( "ctrl_channel_dev_context(%s@%d) init OK.", ctrl_channel_ctx[ctx_id].dev_name, ctrl_channel_ctx[ctx_id].dev_minor_id );

    return 0;

failed_kfifo_out_alloc:
    kfifo_free( &ctrl_channel_ctx[ctx_id].ctrl_kfifo_in );
failed_kfifo_in_alloc:
    cdev_del( &ctrl_channel_ctx[ctx_id].ctrl_cdev );
    unregister_chrdev_region(ctrl_channel_ctx[ctx_id].ctrl_devno, 1);

    LOG( LOG_ERR, "Error: init failed for dev: %s.", ctrl_channel_ctx[ctx_id].dev_name );
    return rc;
}

void ctrl_channel_deinit(int ctx_id)
{
    if ( ctrl_channel_ctx[ctx_id].dev_inited ) {
        kfifo_free( &ctrl_channel_ctx[ctx_id].ctrl_kfifo_in );

        kfifo_free( &ctrl_channel_ctx[ctx_id].ctrl_kfifo_out );

        cdev_del( &ctrl_channel_ctx[ctx_id].ctrl_cdev );
        unregister_chrdev_region(ctrl_channel_ctx[ctx_id].ctrl_devno, 1);

        pr_info("cdev del dev: %s.", ctrl_channel_ctx[ctx_id].dev_name );
    } else {
        LOG( LOG_INFO, "dev not inited, do nothing." );
    }
}

static uint8_t is_uf_needed_command( uint8_t command_type, uint8_t command, uint8_t direction )
{
    uint8_t rc = 0;

    // we only support SET in user-FW.
    if ( COMMAND_GET == direction )
        return rc;

    switch ( command_type ) {
#ifdef TALGORITHMS
    case TALGORITHMS:
        LOG( LOG_INFO, "TALGORITHMS is supported, cmd_type: %u, cmd: %u, direction: %u",
             command_type, command, direction );
        rc = 1;
        break;
#endif

#ifdef TGENERAL
    case TGENERAL:
        switch ( command ) {
        case ACTIVE_CONTEXT:
            LOG( LOG_INFO, "TGENERAL is supported, cmd_type: %u, cmd: %u, direction: %u",
                 command_type, command, direction );
            rc = 1;
            break;
        }

        break;
#endif

#ifdef TSENSOR
    case TSENSOR:
        LOG( LOG_INFO, "TSENSOR is supported, cmd_type: %u, cmd: %u, direction: %u",
             command_type, command, direction );
        rc = 1;
        break;
#endif

#ifdef TSYSTEM
    case TSYSTEM:

        if ( command != SYSTEM_FREEZE_FIRMWARE ) {
            rc = 1;
            LOG( LOG_INFO, "TSYSTEM is supported, cmd_type: %u, cmd: %u, direction: %u",
                 command_type, command, direction );
        } else {
            LOG( LOG_INFO, "SYSTEM_FREEZE_FIRMWARE is not needed" );
        }

        break;
#endif

#ifdef TSCENE_MODES
    case TSCENE_MODES:
        LOG( LOG_INFO, "TSCENE_MODES is supported, cmd_type: %u, cmd: %u, direction: %u",
             command_type, command, direction );
        rc = 1;
        break;
#endif

#ifdef TISP_MODULES
    case TISP_MODULES:
        LOG( LOG_INFO, "TISP_MODULES is supported, cmd_type: %u, cmd: %u, direction: %u",
             command_type, command, direction );
        rc = 1;
        break;
#endif

#ifdef MSENSOR
    case MSENSOR:
        switch ( command ) {
	case SENSOR_TYPE:
            LOG( LOG_INFO, "MSENSOR is supported, cmd_type: %u, cmd: %u, direction: %u",
                 command_type, command, direction );
            rc = 1;
            break;
        }
#endif

#ifdef LOG_LIST
    case LOG_LIST:
        LOG( LOG_INFO, "LOG_LIST is supported, cmd_type: %u, cmd: %u, direction: %u",
             command_type, command, direction );
	if (direction == COMMAND_SET) {
		rc = 1;
	}
        break;
#endif
    }

    return rc;
}

static uint8_t is_uf_needed_calibration( uint8_t command_type, uint8_t command, uint8_t direction )
{
    uint8_t rc = 1;

    // we only support SET in user-FW.
    if ( COMMAND_GET == direction )
        return 0;

    if ((command == CALIBRATION_SINTER_STRENGTH)
	|| (command == CALIBRATION_SINTER_STRENGTH1)
	|| (command == CALIBRATION_SINTER_STRENGTH4)
	|| (command == CALIBRATION_SINTER_THRESH1)
	|| (command == CALIBRATION_SINTER_THRESH4)
	|| (command == CALIBRATION_BLACK_LEVEL_R)
	|| (command == CALIBRATION_BLACK_LEVEL_GR)
	|| (command == CALIBRATION_BLACK_LEVEL_GB)
	|| (command == CALIBRATION_BLACK_LEVEL_B)
	|| (command == CALIBRATION_SHADING_LS_D65_R)
	|| (command == CALIBRATION_SHADING_LS_D65_G)
	|| (command == CALIBRATION_SHADING_LS_D65_B)
	|| (command == CALIBRATION_SHADING_LS_TL84_R)
	|| (command == CALIBRATION_SHADING_LS_TL84_G)
	|| (command == CALIBRATION_SHADING_LS_TL84_B)
	|| (command == CALIBRATION_SHADING_LS_A_R)
	|| (command == CALIBRATION_SHADING_LS_A_G)
	|| (command == CALIBRATION_SHADING_LS_A_B)
	|| (command == CALIBRATION_SHARP_ALT_D)
	|| (command == CALIBRATION_SHARP_ALT_UD)
	|| (command == CALIBRATION_SHARP_ALT_DU)
	|| (command == CALIBRATION_SHARPEN_FR)
	|| (command == CALIBRATION_TEMPER_STRENGTH)
	|| (command == CALIBRATION_DEMOSAIC_NP_OFFSET)
	|| (command == CALIBRATION_CNR_UV_DELTA12_SLOPE)
	|| (command == CALIBRATION_RGB2YUV_CONVERSION)
	|| (command == CALIBRATION_DP_SLOPE)
	|| (command == CALIBRATION_DP_THRESHOLD)
	|| (command == CALIBRATION_STITCHING_LM_MOV_MULT)
	|| (command == CALIBRATION_STITCHING_LM_NP)
	|| (command == CALIBRATION_STITCHING_MS_MOV_MULT)
	|| (command == CALIBRATION_STITCHING_MS_NP)
	|| (command == CALIBRATION_STITCHING_SVS_MOV_MULT)
	|| (command == CALIBRATION_STITCHING_SVS_NP)
	|| (command == CALIBRATION_CUSTOM_SETTINGS_CONTEXT)
	|| (command == CALIBRATION_SHADING_RADIAL_IR)
	|| (command == CALIBRATION_SHADING_RADIAL_CENTRE_AND_MULT)
	|| (command == CALIBRATION_TEMPER_STRENGTH)
	|| (command == CALIBRATION_DECOMPANDER0_MEM)
	|| (command == CALIBRATION_DECOMPANDER1_MEM)
	|| (command == CALIBRATION_CA_CORRECTION)
	|| (command == CALIBRATION_GAMMA)
	|| (command == CALIBRATION_USER_TEMPER_NOISE_LUT)
	|| (command == CALIBRATION_USER_SINTER_LUT)
	) {
	    rc = 0;
    }

    return rc;
}


void ctrl_channel_handle_command( uint32_t cmd_ctx_id, uint8_t type, uint8_t command, uint32_t value, uint8_t direction )
{
    struct ctrl_cmd_item *p_cmd = &ctrl_channel_ctx[cmd_ctx_id].cmd_item;

    if ( !ctrl_channel_ctx[cmd_ctx_id].dev_inited ) {
        LOG( LOG_ERR, "FW ctrl channel is not inited." );
        return;
    }

    if ( !ctrl_channel_ctx[cmd_ctx_id].dev_opened ) {
        LOG( LOG_DEBUG, "FW ctrl channel is not opened, skip." );
        return;
    }

    /* If user-FW is not needed this command, we do nothing */
    if ( !is_uf_needed_command( type, command, direction ) ) {
        return;
    }

    p_cmd->cmd_category = CTRL_CMD_CATEGORY_API_COMMAND;
    p_cmd->cmd_len = sizeof( struct ctrl_cmd_item );

    p_cmd->cmd_ctx_id = cmd_ctx_id;
    p_cmd->cmd_type = type;
    p_cmd->cmd_id = command;
    p_cmd->cmd_direction = direction;
    p_cmd->cmd_value = value;

    LOG( LOG_INFO, "api_command: cmd_ctx_id: %u, cmd_type: %u, cmd_id: %u, cmd_direction: %u, cmd_value: %u.", p_cmd->cmd_ctx_id, p_cmd->cmd_type, p_cmd->cmd_id, p_cmd->cmd_direction, p_cmd->cmd_value );

    ctrl_channel_write( p_cmd, NULL, 0 );
}

void ctrl_channel_handle_api_calibration( uint32_t cmd_ctx_id, uint8_t type, uint8_t id, uint8_t direction, void *data, uint32_t data_size )
{
    struct ctrl_cmd_item *p_cmd = &ctrl_channel_ctx[cmd_ctx_id].cmd_item;

    if ( !ctrl_channel_ctx[cmd_ctx_id].dev_inited ) {
        LOG( LOG_ERR, "FW ctrl channel is not inited." );
        return;
    }

    if ( !ctrl_channel_ctx[cmd_ctx_id].dev_opened ) {
        LOG( LOG_INFO, "FW ctrl channel is not opened, skip." );
        return;
    }

    /* If user-FW is not needed this command, we do nothing */
    if ( !is_uf_needed_calibration( type, id, direction ) ) {
        return;
    }

    p_cmd->cmd_category = CTRL_CMD_CATEGORY_API_CALIBRATION;
    p_cmd->cmd_len = sizeof( struct ctrl_cmd_item ) + data_size;
    p_cmd->cmd_type = type;

    p_cmd->cmd_ctx_id = cmd_ctx_id;
    p_cmd->cmd_id = id;
    p_cmd->cmd_direction = direction;
    p_cmd->cmd_value = data_size;

    LOG( LOG_INFO, "api_alibration: cmd_type: %u, cmd_id: %u, cmd_direction: %u, cmd_value: %u.", p_cmd->cmd_type, p_cmd->cmd_id, p_cmd->cmd_direction, p_cmd->cmd_value );

    ctrl_channel_write( p_cmd, data, data_size );
}

/* Below empty interface functions is used for build in kern-FW */
void ctrl_channel_process( void )
{
}
