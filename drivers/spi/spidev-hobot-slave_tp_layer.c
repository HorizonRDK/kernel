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
#include <linux/workqueue.h>
#include "spi-hobot-slave.h"

/*
tp_layer test
-----------------------------------------------
| offset | 0          | 1	   | 2           |
-----------------------------------------------
|meanings| mask number| counter| total length|
-----------------------------------------------
*/
//sync number test

static unsigned int cur_custom_frame_len = 0;
static unsigned int cur_custom_frame_count = -1;
static unsigned int cur_custom_frame_odd = 0;
int start_count = 0;
int middle_count = 0;
int end_count = 0;
static int
spi_tp_rx_start_or_end(struct spidev_data *spidev)
{
	char ret = -1;
	spi_header_byte spi_rx_mask;
	spi_rx_mask.byte_value = spidev->rx_buffer[SPI_MASK_OFFSET];
	if (spi_rx_mask.element_value.start == 1
		&& spi_rx_mask.element_value.end == 0
		&& spi_rx_mask.element_value.consecutive == 0) {
		ret = start_frame;
	} else if (spi_rx_mask.element_value.start == 0
		&& spi_rx_mask.element_value.end == 1 &&
		spi_rx_mask.element_value.consecutive == 1) {
		ret = end_frame;
	} else if (spi_rx_mask.element_value.start == 1
		&& spi_rx_mask.element_value.end == 1 &&
		spi_rx_mask.element_value.consecutive == 0) {
		ret = single_frame;
	} else if (spi_rx_mask.element_value.start == 0
		&& spi_rx_mask.element_value.end == 0
		&& spi_rx_mask.element_value.consecutive == 1) {
		ret = middle_frame;
	}
	if (spidev->level > SPI_NO_DEBUG) {
		spi_debug_log("c=%d, start=%d, con=%d end=%d\n",
			spidev->rx_buffer[SPI_COUNT_OFFSET],
			spi_rx_mask.element_value.start,
			spi_rx_mask.element_value.consecutive,
			spi_rx_mask.element_value.end);
	}
	if (ret == -1) {
		spi_err_log("mask byte error 0x%x\n",
					spidev->rx_buffer[SPI_MASK_OFFSET]);
	}
	return ret;
}
static int
spi_tp_tx_start_or_end(struct spidev_data *spidev)
{
	char ret = -1;
	spi_header_byte spi_tx_mask;
	spi_tx_mask.byte_value = spidev->tx_buffer[SPI_MASK_OFFSET];
	if (spi_tx_mask.element_value.start) {
		ret = start_frame;
	}
	if (spi_tx_mask.element_value.end) {
		ret = end_frame;
	}
	if (spi_tx_mask.element_value.start
		&& spi_tx_mask.element_value.end) {
		ret = single_frame;
	}
	if (spi_tx_mask.element_value.start  == 0
		&& spi_tx_mask.element_value.end == 0
		&& spi_tx_mask.element_value.consecutive == 1) {
		ret = middle_frame;
	}
	if (spidev->level > SPI_NO_DEBUG) {
		spi_debug_log("c=%d, start=%d, end=%d\n",
						spidev->tx_buffer[SPI_COUNT_OFFSET],
						spi_tx_mask.element_value.start,
						spi_tx_mask.element_value.end);
	}
	return ret;
}

int
spi_tp_tx_start_or_end_interface(struct spidev_data *spidev)
{
	return spi_tp_tx_start_or_end(spidev);
}
DEFINE_SPINLOCK(spi_tp_spinlock);
static void
copy_rxbuf2tmpbuf(struct spidev_data *spidev, int start, int length)
{
	unsigned long spi_tp_spinlock_flags;
	spin_lock_irqsave(&spi_tp_spinlock, spi_tp_spinlock_flags);
	memcpy(spidev->spi_recv_buf.rx_tmp_buf_pos, spidev->rx_buffer + start, length);
	spidev->spi_recv_buf.rx_tmp_buf_pos += length;
	spidev->spi_recv_buf.rx_tmp_len += length;
	spidev->spi_recv_buf.empty_len -= length;
	spin_unlock_irqrestore(&spi_tp_spinlock, spi_tp_spinlock_flags);
}

static void reset_tmpbuf(struct spidev_data *spidev, int empty_len)
{
	spidev->spi_recv_buf.rx_tmp_buf_pos = spidev->spi_recv_buf.rx_tmp_buf;
	spidev->spi_recv_buf.rx_tmp_len = 0;
	spidev->spi_recv_buf.empty_len = empty_len;
	spidev->spi_recv_buf.new_frame_start = 0;
	cur_custom_frame_len = 0;
	cur_custom_frame_odd = 0;
	cur_custom_frame_count = 0;
}
static int
spi_tp_resolve_fragment(struct spidev_data *spidev,
									int *rest_fragment_count)
{
	int ret = -1, tmp_count = 0, rx_tmp_buf_overflow = 0;
	int i, tmp_cur_custom_frame_len = 0;

	tmp_count = spidev->rx_buffer[SPI_COUNT_OFFSET];
	ret = spi_tp_rx_start_or_end(spidev);
	if (ret == start_frame) {
		++start_count;
		spidev->spi_statistic.start_frag_count++;
		if (spidev->spi_recv_buf.new_frame_start == 1) {
			reset_tmpbuf(spidev, RX_TEMP_BUFFER_LEN);
			spidev->spi_recv_buf.new_frame_start = 1;
			spidev->spi_statistic.lost_end_frag_error++;
		} else {
			reset_tmpbuf(spidev, RX_TEMP_BUFFER_LEN);
			spidev->spi_recv_buf.new_frame_start = 1;
		}
		cur_custom_frame_count = tmp_count;
		cur_custom_frame_len =
			*(unsigned short *)(spidev->rx_buffer + SPI_TOTAL_LENGTH_OFFSET);
		if (cur_custom_frame_len > SPI_FRAGMENT_VALID_SIZE) {
			*rest_fragment_count =
				(cur_custom_frame_len-SPI_FRAGMENT_VALID_SIZE)
				/ (SPI_FRAGMENT_VALID_SIZE+2);
			cur_custom_frame_odd =
				(cur_custom_frame_len - SPI_FRAGMENT_VALID_SIZE)
				% (SPI_FRAGMENT_VALID_SIZE+2);
			if (!cur_custom_frame_odd)
				cur_custom_frame_odd = SPI_FRAGMENT_VALID_SIZE+2;
			else
				*rest_fragment_count += 1;
		} else {
			cur_custom_frame_odd = cur_custom_frame_len;
			*rest_fragment_count = 0;
		}
		if (spidev->spi_recv_buf.empty_len >= SPI_FRAGMENT_VALID_SIZE) {
			copy_rxbuf2tmpbuf(spidev, SPI_DATA_OFFSET,
							SPI_FRAGMENT_VALID_SIZE);
		} else {
			rx_tmp_buf_overflow = 1;
		}
		if (spidev->level > SPI_NO_DEBUG) {
			for (i = 0; i < SPI_FRAGMENT_SIZE; ++i) {
				spi_debug_log("%x", spidev->rx_buffer[i]);
				if (!((i + 1) % 8))
					spi_debug_log("\n");
			}
			spi_debug_log("\n");
			spi_debug_log("len = %d\n", cur_custom_frame_len);
			spi_debug_log("odd = %d\n", cur_custom_frame_odd);
		}

	} else if (ret == end_frame) {
		++end_count;
		if (spidev->spi_recv_buf.empty_len >= cur_custom_frame_odd
				&& tmp_count == cur_custom_frame_count + 1) {
			copy_rxbuf2tmpbuf(spidev, SPI_DATA_OFFSET-2,
									cur_custom_frame_odd);
			cur_custom_frame_count = tmp_count;
		} else {
			if (tmp_count != cur_custom_frame_count + 1) {
				spi_err_log("lost middle\n");
				reset_tmpbuf(spidev, RX_TEMP_BUFFER_LEN);
				spidev->spi_statistic.lost_middle_frag_error++;
				return -3;
			}
			if (spidev->spi_recv_buf.empty_len <
											cur_custom_frame_odd)
				rx_tmp_buf_overflow = 2;
		}
		if (spidev->spi_recv_buf.rx_tmp_len == cur_custom_frame_len) {
			spidev->spi_recv_buf.new_frame_start = 0;
			tmp_cur_custom_frame_len = cur_custom_frame_len;
			cur_custom_frame_len = 0;
			cur_custom_frame_odd = 0;
			cur_custom_frame_count = 0;
			return tmp_cur_custom_frame_len;
		} else { // lost middle fragment error
			spi_err_log("lost middle\n");
			reset_tmpbuf(spidev, RX_TEMP_BUFFER_LEN);
			spidev->spi_statistic.lost_middle_frag_error++;
			return -3;
		}
	} else if (ret == single_frame) {
		cur_custom_frame_len =
			*(unsigned short *)(spidev->rx_buffer +
								SPI_TOTAL_LENGTH_OFFSET);
		if (spidev->spi_recv_buf.empty_len >= cur_custom_frame_len) {
			copy_rxbuf2tmpbuf(spidev, SPI_DATA_OFFSET,
								cur_custom_frame_len);
			return cur_custom_frame_len;
		} else {
			rx_tmp_buf_overflow = 3;
			spi_err_log("rx_tmp_buf_overflow len 0x%x \n",
						cur_custom_frame_len);
			if (spidev->level > SPI_NO_DEBUG) {
				for (i = 0; i < SPI_FRAGMENT_SIZE; ++i) {
					spi_debug_log("%x", spidev->rx_buffer[i]);
					if (!((i + 1) % 8))
						spi_debug_log("\n");
				}
				spi_debug_log("\n");
			}
		}
	} else if (ret == middle_frame) {
		++middle_count;
		if (spidev->spi_recv_buf.new_frame_start == 1
			&& tmp_count == cur_custom_frame_count + 1
			&& spidev->spi_recv_buf.rx_tmp_len
								< cur_custom_frame_len) {
			if (spidev->spi_recv_buf.empty_len
							>= SPI_FRAGMENT_VALID_SIZE+2) {
				copy_rxbuf2tmpbuf(spidev, SPI_DATA_OFFSET-2,
								SPI_FRAGMENT_VALID_SIZE + 2);
				cur_custom_frame_count = tmp_count;

			} else {
				rx_tmp_buf_overflow = 4;
			}
		} else {
			if (tmp_count != cur_custom_frame_count + 1) {
				spi_err_log("lost middle\n");
				reset_tmpbuf(spidev, RX_TEMP_BUFFER_LEN);
				spidev->spi_statistic.lost_middle_frag_error++;
				return -3;
			}
			if (spidev->spi_recv_buf.new_frame_start == 0) {
				reset_tmpbuf(spidev, RX_TEMP_BUFFER_LEN);
				spi_err_log("lost middle\n");
				spidev->spi_statistic.lost_start_frag_error++;
				return -3;
			}
			if (spidev->spi_recv_buf.rx_tmp_len
							>= cur_custom_frame_len) {
				reset_tmpbuf(spidev, RX_TEMP_BUFFER_LEN);
				spi_err_log("lost middle\n");
				spidev->spi_statistic.lost_start_frag_error++;
				return -3;
			}
		}
	}
	if (rx_tmp_buf_overflow) {
		spi_err_log("rx_tmp_buf overflow %d\n", rx_tmp_buf_overflow);
		rx_tmp_buf_overflow = 0;
		reset_tmpbuf(spidev, RX_TEMP_BUFFER_LEN);
		spidev->spi_statistic.rx_tmp_buf_overflow++;
		return -2;
	}
	return -1;
}
int spi_tp_resolve_fragment_interface(struct spidev_data *spidev,
													int *rest_fragment_count)
{
	return spi_tp_resolve_fragment(spidev, rest_fragment_count);
}

static char spi_slave_frag_count_set = 0;
static int spi_slave_frag_count_total = 0;
static int spi_slave_frag_last_copy = 0;
static int spi_slave_frag_index = 0;

static int spi_slave_tp_frag(char *src_buf, char *frag_buf, int data_length)
{
	static spi_header_byte send_spi_header;
	u32 copy_length = 0;
	int tmp_data_length = data_length;
	memset(&send_spi_header, 0, sizeof(send_spi_header));
	if (src_buf == NULL || frag_buf == NULL || data_length <= 0) {
		spi_err_log("input param error!\n");
		return -1;
	}
	if (spi_slave_frag_count_total == 0) {
		if (data_length <= SPI_FRAGMENT_VALID_SIZE) {
			spi_slave_frag_count_total = 1;
			spi_slave_frag_last_copy = data_length;
		} else {
			tmp_data_length -= SPI_FRAGMENT_VALID_SIZE;
			spi_slave_frag_count_total += 1;
			spi_slave_frag_count_total +=
				tmp_data_length / (SPI_FRAGMENT_VALID_SIZE + 2);
			spi_slave_frag_last_copy =
				tmp_data_length % (SPI_FRAGMENT_VALID_SIZE + 2);
			if (spi_slave_frag_last_copy)
				++spi_slave_frag_count_total;
		}
	}
	if (spi_slave_frag_count_total == 1) {
		copy_length = data_length;
		send_spi_header.element_value.start = 1;
		send_spi_header.element_value.consecutive = 0;
		send_spi_header.element_value.end = 1;
		frag_buf[0] = send_spi_header.byte_value;
		frag_buf[1] = spi_slave_frag_count_set;
		frag_buf[2] = (u8)(short)data_length & 0xff;
		frag_buf[3] = (u8)((short)data_length>>8) & 0xff;
		memcpy(frag_buf + 4, src_buf, copy_length);
		spi_slave_frag_count_total = 0;
		spi_slave_frag_last_copy = 0;
	//	printk("1 spi_slave_tp_frag \n");
		return 2;// frag finish
	}
	if (spi_slave_frag_count_set == 0) {
		send_spi_header.element_value.start = 1;
		send_spi_header.element_value.consecutive = 0;
		send_spi_header.element_value.end = 0;
		copy_length = SPI_FRAGMENT_VALID_SIZE;
		frag_buf[0] = send_spi_header.byte_value;
		frag_buf[1] = spi_slave_frag_count_set;
		frag_buf[2] = (u8)(short)data_length & 0xff;
		frag_buf[3] = (u8)((short)data_length>>8) & 0xff;
		++spi_slave_frag_count_set;
		memcpy(frag_buf + 4, src_buf+spi_slave_frag_index, copy_length);
		spi_slave_frag_index += copy_length;
		return 0;
	} else if (spi_slave_frag_count_set
				== (spi_slave_frag_count_total - 1)) {
		send_spi_header.element_value.start = 0;
		send_spi_header.element_value.consecutive = 0;
		send_spi_header.element_value.end = 1;
		frag_buf[0] = send_spi_header.byte_value;
		frag_buf[1] = spi_slave_frag_count_set;
		++spi_slave_frag_count_set;
		if (spi_slave_frag_last_copy == 0)
			copy_length = SPI_FRAGMENT_VALID_SIZE+2;
		else
			copy_length = spi_slave_frag_last_copy;
		memcpy(frag_buf + 2, src_buf+spi_slave_frag_index, copy_length);
		spi_slave_frag_index = 0;
		spi_slave_frag_count_total = 0;
		spi_slave_frag_last_copy = 0;
		spi_slave_frag_count_set = 0;
	//	printk("2 spi_slave_tp_frag \n");
		return 2;// frag finish
	} else {
		send_spi_header.element_value.start = 0;
		send_spi_header.element_value.consecutive = 1;
		send_spi_header.element_value.end = 0;
		copy_length = SPI_FRAGMENT_VALID_SIZE+2;
		frag_buf[0] = send_spi_header.byte_value;
		frag_buf[1] = spi_slave_frag_count_set;
		++spi_slave_frag_count_set;
		memcpy(frag_buf + 2, src_buf+spi_slave_frag_index, copy_length);
		spi_slave_frag_index += copy_length;
		return 1;
	}
}


int spi_slave_tp_frag_interface(char *src_buf, char *frag_buf, int data_length)
{
	int ret = 0;
	ret = spi_slave_tp_frag(src_buf, frag_buf, data_length);
	return ret;
}

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("hobot spidev slave module(TP layer)");
MODULE_LICENSE("GPL v2");
