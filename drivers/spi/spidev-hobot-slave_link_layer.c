/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include "spi-hobot-slave.h"

/*
link_layer test
-----------------------------------------------
| offset | 0          | 1			   | 2    |
-----------------------------------------------
|meanings| sync number| rolling counter| CRC16|
-----------------------------------------------
*/
//#define SPI_CRC16
#define SPI_CHECKSUM16

//sync number test
static int spi_link_layer_sync_num_correct(struct spidev_data *spidev)
{
	int ret = -1;
	if (spidev->rx_buffer[SPI_SYNC_CODE_OFFSET] == SPI_PREAMBLE)
		ret = sync_num_correct;
	else if (spidev->rx_buffer[SPI_SYNC_CODE_OFFSET] == DUMMY_FLAG)
		ret = dummy_correct;
	else
		ret = unknown_error;
	return ret;
}
static int spi_link_layer_set_sync_num(char *src_buf)
{
	int ret = 1;
	src_buf[SPI_SYNC_CODE_OFFSET] = SPI_PREAMBLE;
	if (src_buf[SPI_SYNC_CODE_OFFSET] != SPI_PREAMBLE) {
		ret = -1;
		spi_err_log("sync_num set error!\n");
	}
	return ret;
}

char spi_recv_rc = 0, spi_send_rc = 0;
static int spi_link_layer_rolling_count_correct(struct spidev_data *spidev)
{
	int ret = -1;
	if (spidev->rx_buffer[SPI_ROLLING_COUNTER_OFFSET] != (char)(spi_recv_rc)) {
		spi_recv_rc = spidev->rx_buffer[SPI_ROLLING_COUNTER_OFFSET];
		ret  = rolling_count_error;
		spi_err_log("rolling data error!\n");
	} else {
		ret  = rolling_count_correct;
	}
	++spi_recv_rc;
	return ret;
}
static int spi_link_layer_set_rolling_count(char *src_buf)
{
	int ret = 1;
	src_buf[SPI_ROLLING_COUNTER_OFFSET] = (char)spi_send_rc;
	if (src_buf[SPI_ROLLING_COUNTER_OFFSET] != (char)spi_send_rc) {
		ret = -1;
		spi_err_log("rolling data error!\n");
	}
	++spi_send_rc;
	return ret;
}
#if defined SPI_CRC16
void InvertUint8(unsigned char *dBuf, unsigned char *srcBuf)
{
	int i;
	unsigned char tmp[4] = {0};

	for (i = 0; i < 8; i++) {
		if (srcBuf[0] & (1 << i))
		tmp[0] |= 1 << (7-i);
    }
	dBuf[0] = tmp[0];
}
void InvertUint16(unsigned short *dBuf, unsigned short *srcBuf)
{
	int i;
	unsigned short tmp[4] = {0};

	for (i = 0; i < 16; i++) {
		if (srcBuf[0] & (1 << i))
		tmp[0] |= 1 << (15 - i);
	}
	dBuf[0] = tmp[0];
}
unsigned short CRC16_CCITT(unsigned char *data, unsigned int datalen)
{
	unsigned short wCRCin = 0x0000;
	unsigned short wCPoly = 0x1021;
	unsigned char wChar = 0;
	int i = 0;
	while (datalen--) {
		wChar = *(data++);
		InvertUint8(&wChar, &wChar);
		wCRCin ^= (wChar << 8);
		for (i = 0; i < 8; i++) {
			if (wCRCin & 0x8000)
				wCRCin = (wCRCin << 1) ^ wCPoly;
			else
				wCRCin = wCRCin << 1;
		}
	}
	InvertUint16(&wCRCin, &wCRCin);
	return wCRCin;
}

static int crc16_test(char *buf)
{
	int ret = 1;
	unsigned short crc16_recv = *(unsigned short *)(buf + SPI_CRC_OFFSET);
	unsigned short crc16_recv_temp = 0;

	buf[SPI_CRC_OFFSET] = 0;
	buf[SPI_CRC_OFFSET + 1] = 0;
	crc16_recv_temp = CRC16_CCITT(buf, SPI_FRAGMENT_SIZE);
	buf[SPI_CRC_OFFSET] = crc16_recv;
	buf[SPI_CRC_OFFSET + 1] = crc16_recv >> 8;

	if (crc16_recv != crc16_recv_temp)
		ret = -1;
	return ret;
}
static int crc16_set(char *buf)
{
	int ret = 1;
	unsigned short crc16_set = 0;

	buf[SPI_CRC_OFFSET] = 0;
	buf[SPI_CRC_OFFSET + 1] = 0;
	crc16_set = CRC16_CCITT(buf, SPI_FRAGMENT_SIZE);
	buf[SPI_CRC_OFFSET] = crc16_set;
	buf[SPI_CRC_OFFSET + 1] = crc16_set >> 8;
	if (*(unsigned short *)(buf + SPI_CRC_OFFSET) != crc16_set)
		ret = 0;
	return ret;
}

static int spi_link_layer_CRC16_correct(struct spidev_data *spidev)//crc16 test
{
	int ret = 1;
	if (crc16_test(spidev->rx_buffer) == -1) {
		ret = crc16_error;
		spi_err_log("crc16 error!\n");
	} else
		ret = crc16_correct;
	return ret;
}
static int spi_link_layer_set_CRC16(char *src_buf)
{
	int ret = 1;
	if (crc16_set(src_buf) == 0) {
		ret = -1;
		spi_err_log("crc16 set error!\n");
	}
	return ret;
}
#elif defined SPI_CHECKSUM16
static unsigned short Checksum16_Calc(unsigned char *data,
											unsigned int datalen)
{
	unsigned short checksum = 0;
	unsigned int i;
	unsigned int ck_no = datalen / 2;
	unsigned short * dataPtr = (unsigned short *)data;
	for (i = 0; i < ck_no; i++) {
		checksum += dataPtr[i];
	}
	return (checksum);
}
static int Checksum16_test(char *buf)
{
	int ret = 1;
	unsigned short checksum16_recv = *(unsigned short *)(buf + SPI_CRC_OFFSET);
	unsigned short checksum16_recv_temp = 0;

	buf[SPI_CRC_OFFSET] = 0;
	buf[SPI_CRC_OFFSET + 1] = 0;
	checksum16_recv_temp = Checksum16_Calc(buf, SPI_FRAGMENT_SIZE);
	buf[SPI_CRC_OFFSET] = (u8)checksum16_recv;
	buf[SPI_CRC_OFFSET + 1] = (u8)(checksum16_recv >> 8);
	if (checksum16_recv != checksum16_recv_temp)
		ret = -1;
	return ret;
}
static int Checksum16_set(char *buf)
{
	int ret = 1;
	unsigned short checksum16_set = 0;

	buf[SPI_CRC_OFFSET] = 0;
	buf[SPI_CRC_OFFSET + 1] = 0;
	checksum16_set = Checksum16_Calc(buf, SPI_FRAGMENT_SIZE);
	buf[SPI_CRC_OFFSET] = (u8)checksum16_set;
	buf[SPI_CRC_OFFSET + 1] = (u8)(checksum16_set >> 8);
	if (*(unsigned short *)(buf + SPI_CRC_OFFSET) != checksum16_set)
		ret = 0;
	return ret;
}
/* crc16 test */
static int spi_link_layer_Checksum16_correct(struct spidev_data *spidev)
{
	int ret = 1;
	if (Checksum16_test(spidev->rx_buffer) == -1) {
		ret = crc16_error;
		spi_err_log("Checksum16 error!\n");
	} else
		ret = crc16_correct;
	return ret;
}
static int spi_link_layer_set_Checksum16(char *src_buf)
{
	int ret = 1;
	if (Checksum16_set(src_buf) == 0) {
		ret = -1;
		spi_err_log("Checksum16 set error!\n");
	}
	return ret;
}

#endif
static int spi_link_layer_correct(struct spidev_data *spidev)//link test
{
	int ret = 0;
	ret = spi_link_layer_sync_num_correct(spidev);
	if (ret == dummy_correct) {
		ret = link_sync_dummy;
	} else if (ret == sync_num_correct) {
		if (spi_link_layer_rolling_count_correct(spidev) == rolling_count_error) {
			spi_err_log("slave error rc:0x%x\n",
					spidev->rx_buffer[SPI_ROLLING_COUNTER_OFFSET]);
//			ret = link_rolling_count_error;
		}
#if defined SPI_CRC16
		if (spi_link_layer_CRC16_correct(spidev) == crc16_error) {
			spi_err_log("slave error crc:0x%x\n",
					*(unsigned short *)(spidev->rx_buffer + SPI_CRC_OFFSET));
			ret = link_crc16_error;
		}
#elif defined SPI_CHECKSUM16
		if (spi_link_layer_Checksum16_correct(spidev) == crc16_error) {
			spi_err_log("slave error Checksum16:0x%x\n",
					*(unsigned short *)(spidev->rx_buffer + SPI_CRC_OFFSET));
			ret = link_crc16_error;
		}
#endif
		if (ret == sync_num_correct)
			ret = link_correct;
	} else if (ret == unknown_error) {
		ret = link_sync_unknown_error;
		spi_err_log("slave error sync_num:0x%x\n",
				spidev->rx_buffer[SPI_SYNC_CODE_OFFSET]);
	}
//	spidev->rx_buffer = spidev->rx_buffer + SPI_MASK_OFFSET;
	return ret;
}
int spi_link_layer_correct_interface(struct spidev_data *spidev)
{
	return spi_link_layer_correct(spidev);
}

static int spi_link_layer_set(char *src_buf)
{
	int ret = 1;
#if defined SPI_CRC16
	if (spi_link_layer_set_sync_num(src_buf) == -1 ||
		spi_link_layer_set_rolling_count(src_buf) == -1 ||
		spi_link_layer_set_CRC16(src_buf) == -1) {
		ret = -1;
		spi_err_log("link_layer set error\n");
	}
#elif defined SPI_CHECKSUM16
	if (spi_link_layer_set_sync_num(src_buf) == -1 ||
		spi_link_layer_set_rolling_count(src_buf) == -1 ||
		spi_link_layer_set_Checksum16(src_buf) == -1) {
		ret = -1;
		spi_err_log("link_layer set error\n");
	}
#endif
	return ret;
}
int spi_link_layer_set_interface(char *src_buf)//link set
{
	return spi_link_layer_set(src_buf);
}
