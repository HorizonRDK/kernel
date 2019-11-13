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
#include <linux/printk.h>
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

    return 0;
}

int camera_spi_reg16_write(uint32_t port, uint32_t chip_id,
		uint32_t reg_addr, void *buf, uint32_t length)
{
	char *senbuf = NULL;
	int ret = 0;

	senbuf = kmalloc(length + 3, GFP_KERNEL);
	memset(senbuf, 0, length + 3);
	senbuf[0] = chip_id | 0x0;
	senbuf[1] = (reg_addr >> 8) & 0x00ff;
	senbuf[2] = reg_addr & 0xff;
	memcpy(&senbuf[3], buf, length);

	struct spi_transfer tx = {
	   .tx_buf = senbuf,
	   .len  = length + 3,
	};

	ret = spi_sync_transfer(camera_mod[port]->spidev, &tx, 1);
	if (ret < 0) {
		pr_err("spi_sync_transfer error\n");
	}
	return ret;
}

int camera_spi_reg8_write(uint32_t port, uint32_t chip_id,
		uint32_t reg_addr, void *buf, uint32_t length)
{
	char *senbuf = NULL;
	int ret = 0;

	senbuf = kmalloc(length + 2, GFP_KERNEL);
	memset(senbuf, 0, length + 2);
	senbuf[0] = chip_id | 0x0;
	senbuf[1] = reg_addr & 0xff;
	memcpy(&senbuf[2], buf, length);

	struct spi_transfer tx = {
	   .tx_buf = senbuf,
	   .len  = length + 2,
	};

	ret = spi_sync_transfer(camera_mod[port]->spidev, &tx, 1);
	if (ret < 0) {
		pr_err("spi_sync_transfer error\n");
	}
	return ret;
}

int camera_spi_write(uint32_t port, uint32_t chip_id, uint32_t reg_width,
		uint32_t reg_addr, void *buf, uint32_t length)
{
	int ret = 0;

	if(reg_addr == 16) {
		ret = camera_spi_reg16_write(port, chip_id, reg_addr, buf, length);
	} else if (reg_addr == 8) {
		ret = camera_spi_reg8_write(port, chip_id, reg_addr, buf, length);
	}
	if(ret < 0) {
		pr_err("camera_spi_write error\n");
		return -1;
	}
	return ret;
}

int camera_spi_reg8_read(uint32_t port, uint32_t chip_id,
		uint32_t reg_addr, void *buf, uint32_t length)
{
	char *recvbuf = NULL;
	int ret = 0;

	recvbuf = kmalloc(length + 2, GFP_KERNEL);
	memset(recvbuf, 0, length + 2);
	recvbuf[0] = chip_id | 0x80;
	recvbuf[1] = reg_addr & 0xff;
	memcpy(&recvbuf[2], buf, length);

	struct spi_transfer rx = {
	   .rx_buf = recvbuf,
	   .len  = length + 2,
	};

	ret = spi_sync_transfer(camera_mod[port]->spidev, &rx, 1);
	if (ret < 0) {
		pr_err("spi_sync_transfer error\n");
	}
	memcpy(buf, &recvbuf[2], length);
	return ret;
}

int camera_spi_reg16_read(uint32_t port, uint32_t chip_id,
		uint32_t reg_addr, void *buf, uint32_t length)
{
	char *recvbuf = NULL;
	int ret = 0;

	recvbuf = kmalloc(length + 3, GFP_KERNEL);
	memset(recvbuf, 0, length + 3);
	recvbuf[0] = chip_id | 0x80;
	recvbuf[1] = (reg_addr >> 8) & 0x00ff;
	recvbuf[2] = reg_addr & 0xff;
	memcpy(&recvbuf[3], buf, length);

	struct spi_transfer rx = {
	   .rx_buf = recvbuf,
	   .len  = length + 3,
	};

	ret = spi_sync_transfer(camera_mod[port]->spidev, &rx, 1);
	if (ret < 0) {
		pr_err("spi_sync_transfer error\n");
	}
	memcpy(buf, &recvbuf[3], length);
	return ret;
}

int camera_spi_read(uint32_t port, uint32_t chip_id, uint32_t reg_width,
		uint32_t reg_addr, void *buf, uint32_t length)
{
	int ret = 0;

	if(reg_addr == 16) {
		ret = camera_spi_reg16_read(port, chip_id, reg_addr, buf, length);
	} else if (reg_addr == 8) {
		ret = camera_spi_reg8_read(port, chip_id, reg_addr, buf, length);
	}
	if(ret < 0) {
		pr_err("camera_spi_read error\n");
		return -1;
	}
	return ret;
}

