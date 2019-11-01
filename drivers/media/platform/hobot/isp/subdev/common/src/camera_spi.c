/*   Copyright (C) 2018 Horizon Inc.
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
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dmaengine.h>
#include <linux/compiler.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/mman.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "inc/camera_spi.h"
#include "inc/camera_subdev.h"
#include "inc/camera_dev.h"
#include "inc/acamera_logger.h"

int camera_spi_release(uint32_t port)
{
	spi_unregister_device(camera_mod[port]->spidev);
    camera_mod[port]->spidev = NULL;
	return 0;
}

int camera_spi_open(uint32_t port, uint32_t spi_bus, uint32_t cs,
		uint32_t mode, uint32_t speed, char *sensor_name)
{
	struct spi_master *master;
	struct spi_board_info spi_board = { {0} };

	master = spi_busnum_to_master(spi_bus);
	if(master) {
		return -ENODEV;
	}
	strlcpy(spi_board.modalias, sensor_name,
			sizeof(spi_board.modalias));

	spi_board.mode		= mode;
	spi_board.bus_num	= spi_bus;
	spi_board.chip_select	= cs;
	spi_board.max_speed_hz	= speed;

	if(camera_mod[port]->spidev) {
		 camera_spi_release(port);
	}
	camera_mod[port]->spidev = spi_new_device(master,
			&camera_mod[port]->spi_board);
    if (!camera_mod[port]->spidev) {
			spi_master_put(master);
        	return -ENOMEM;
    }

	LOG(LOG_INFO, "the %s is open success !", camera_mod[port]->spidev);
    return 0;
}


int camera_spi_write(uint32_t port, const void *buf, uint32_t length)
{
	 struct spi_transfer t = {
	   .tx_buf = buf,
	   .len  = length,
	  };

	return spi_sync_transfer(camera_mod[port]->spidev, &t, 1);
}

int camera_spi_read(uint32_t port, void *buf, uint32_t length)
{
	 struct spi_transfer t = {
	   .rx_buf = buf,
	   .len  = length,
	  };

	return spi_sync_transfer(camera_mod[port]->spidev, &t, 1);
}
