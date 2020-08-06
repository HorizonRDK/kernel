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
#include <linux/kthread.h>
#include <linux/debugfs.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include "spi-hobot-slave.h"

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
				| SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

#define SPI_IO_NOTIFY_MCU_INIT		_IO(SPI_IOC_MAGIC, 6)
#define SPI_IO_GET_ERROR_STATUS		_IO(SPI_IOC_MAGIC, 7)
#define SPI_IO_SET_TIMEOUT    		_IO(SPI_IOC_MAGIC, 8)

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#define BUFSIZE			(4096)
#define BUFSIZE_10K		(10 << 10)
#define HOBOT_SPIDEV	"hobot_spidev"
/*-------------------------------------------------------------------------*/
#define TIMEOUT_MS_MAX (20)

struct spi_error_status {
	unsigned int preamble_error;
	unsigned int rx_tmp_buf_overflow;
	unsigned int rx_data_kfifo_overflow;
};

static struct class *spidev_class;
static struct spidev_data *g_spidev = NULL;


/* MCU response */
int ap_response_flag;

static irqreturn_t spidev_irq(int irq, void *dev_id)
{
	static u16 index = 0;
	struct spidev_data *spidev = dev_id;

	spidev->spi_statistic.eint_count++;
	if (!queue_work(spidev->work_queue, &spidev->work[index++]))
		dev_err(&spidev->spi->dev, "queue_work failed index = %d\n", index);

	index %= WORK_COUNT;

	return IRQ_HANDLED;
}
static void J32ap_recv_ok_ack(struct spidev_data *spidev)
{
	gpio_direction_output(spidev->ack_pin, 1);
	spidev->spi_statistic.ack_count++;
	gpio_direction_output(spidev->ack_pin, 0);
	gpio_direction_output(spidev->ack_pin, 1);
}

void hb_spi_info_ap(void)
{
	gpio_direction_output(g_spidev->tri_pin, 1);
	gpio_direction_output(g_spidev->tri_pin, 0);
	g_spidev->spi_statistic.spi_info_ap_count++;
	gpio_direction_output(g_spidev->tri_pin, 1);
}
EXPORT_SYMBOL(hb_spi_info_ap);

static int save_spi_data(struct spidev_data *spidev, int length)
{
	int i = 0;
	if (length <= 0) {
		if(length != -1)
			spi_err_log("save_spi_data length error length %d\n", length);
		return -1;
	}
	if (kfifo_avail(&spidev->rx_fifo.data) >= length) {
		kfifo_in(&spidev->rx_fifo.data,
				spidev->spi_recv_buf.rx_tmp_buf, length);
		kfifo_put(&spidev->rx_fifo.len, length);
		up(&spidev->sem);
		if (spidev->level > SPI_NO_DEBUG) {
			printk("enqueue:%d B", length);
			printk("save_spi_data:rx_tmp_buf\n");
			for (i = 0; i < length; ++i) {
				printk(" 0x%x ", spidev->spi_recv_buf.rx_tmp_buf[i]);
				if (!((i + 1) % 8))
					printk("\n");
			}
			printk("\n");
		}
		spidev->spi_recv_buf.rx_tmp_buf_pos = spidev->spi_recv_buf.rx_tmp_buf;
		spidev->spi_recv_buf.rx_tmp_len = 0;
		spidev->spi_recv_buf.empty_len = RX_TEMP_BUFFER_LEN;
		spidev->spi_statistic.enqueued_frame++;
		J32ap_recv_ok_ack(spidev);
		return 1;
	} else {
		spi_err_log("rx_data_kfifo overflow,length %d \n", length);
		spidev->spi_statistic.rx_data_kfifo_overflow++;
		return -2;
	}
}
int frag_count = 0;
static int recv_one_frag_call = 0;
static int recv_one_fragment(struct spidev_data *spidev,
									int *rest_fragment_count)
{
	int status = 0;
	int i = 0, ret = 0, rest_fragment;
	int rest_fragment_count_tmp = *rest_fragment_count;
	char *tmp_rx_buf = spidev->rx_buffer;
	if ((!spidev->tx_dummy_buffer) || (!spidev->rx_buffer)) {
		spi_err_log("null pointer\n");
		return -4; // A10 resource is free, must return
	}
	++frag_count;

	if (spidev->level > SPI_NO_DEBUG)
		printk_ratelimited(KERN_INFO "status = %d\n", status);

	if (spidev->level > SPI_HEADER_DEBUG) {
		printk("rx_buf\n");
		for (i = 0; i < BUFSIZE; ++i) {
			printk(" 0x%x ", spidev->rx_buffer[i]);
			if (!((i + 1) % 8))
				printk("\n");
		}
		printk("\n");
	}
	while (rest_fragment_count_tmp > 0) {
		spidev->rx_buffer = spidev->rx_buffer + SPI_FRAGMENT_SIZE;
#ifdef SPI_SLAVE_LINK_LAYER

		if (spidev->level > SPI_HEADER_DEBUG) {
			printk("rx_buf\n");
			for (i = 0; i < SPI_FRAGMENT_SIZE; ++i) {
				printk(" 0x%x ", spidev->rx_buffer[i]);
				if (!((i + 1) % 8))
					printk("\n");
			}
			printk("\n");
		}
		ret = spi_link_layer_correct_interface(spidev);
		if (ret !=  link_correct) {
			dev_err(&spidev->spi->dev,
						"rx_buf link_layer test fail ret %d\n", ret);
			if (spidev->level > SPI_HEADER_DEBUG) {
				printk("rx_buf\n");
				for (i = 0; i < SPI_FRAGMENT_SIZE; ++i) {
					printk(" 0x%x ", spidev->rx_buffer[i]);
					if (!((i + 1) % 8))
						printk("\n");
				}
				printk("\n");
			}
			return -3;
		}
		if (recv_one_frag_call == 0) {
			spidev->spi_statistic.tx_recv_one_frag_resolve_count++;
		}
		if (recv_one_frag_call == 1) {
			spidev->spi_statistic.rx_recv_one_frag_resolve_count++;
		}
		status = spi_tp_resolve_fragment_interface(spidev, &rest_fragment);
		if (status == -1) {
			rest_fragment_count_tmp -= 1;
		} else {
			break;
		}
#endif
	}
	if (status < 0)
		dev_err(&spidev->spi->dev, "error status %d\n", status);
	spidev->rx_buffer = tmp_rx_buf;

	return status;
}
static int recv_rest_fragment(struct spidev_data *spidev,
									int *rest_fragment_count)
{
	struct spi_device *spi = spidev->spi;
	struct spi_controller *ctrl = spi->controller;



	struct spi_transfer t;
	int status = 0;
	int i = 0, ret = 0, rest_fragment;
	int rest_fragment_count_tmp = *rest_fragment_count;
	char *tmp_rx_buf = spidev->rx_buffer;
	if ((!spidev->tx_dummy_buffer) || (!spidev->rx_buffer)) {
		spi_err_log("null pointer\n");
		return -4; // A10 resource is free, must return
	}

	memset(&t, 0, sizeof(struct spi_transfer));
	t.tx_buf = spidev->tx_dummy_buffer;
	t.rx_buf = spidev->rx_buffer;
	t.len = (*rest_fragment_count * SPI_FRAGMENT_SIZE);

	spidev->spi_statistic.spidev_sync_3++;
	status = ctrl->transfer_one(ctrl, spi, &t);
	++frag_count;

	if (spidev->level > SPI_NO_DEBUG)
		printk_ratelimited("status = %d\n", status);

	if (spidev->level > SPI_HEADER_DEBUG) {
		printk("rx_buf\n");
		for (i = 0; i < BUFSIZE; ++i) {
			printk(" 0x%x ", spidev->rx_buffer[i]);
			if (!((i + 1) % 8))
				printk("\n");
		}
		printk("\n");
	}
	while (rest_fragment_count_tmp > 0) {
#ifdef SPI_SLAVE_LINK_LAYER
		if (spidev->level > SPI_HEADER_DEBUG) {
			printk("rx_buf\n");
			for (i = 0; i < SPI_FRAGMENT_SIZE; ++i) {
				printk(" 0x%x ", spidev->rx_buffer[i]);
				if (!((i + 1) % 8))
					printk("\n");
			}
			printk("\n");
		}
		ret = spi_link_layer_correct_interface(spidev);
		if (ret !=  link_correct) {
			dev_err(&spidev->spi->dev, "rx_buf link_layer test fail ret %d\n", ret);
			if (spidev->level > SPI_HEADER_DEBUG) {
				printk("rx_buf\n");
				for (i = 0; i < SPI_FRAGMENT_SIZE; ++i) {
					printk(" 0x%x ", spidev->rx_buffer[i]);
					if (!((i + 1) % 8))
						printk("\n");
				}
				printk("\n");
			}
			return -3;
		}

		if (recv_one_frag_call == 0) {
			spidev->spi_statistic.tx_recv_one_frag_resolve_count++;
		}
		if (recv_one_frag_call == 1) {
			spidev->spi_statistic.rx_recv_one_frag_resolve_count++;
		}
		status = spi_tp_resolve_fragment_interface(spidev, &rest_fragment);
		if (status == -1) {
			rest_fragment_count_tmp -= 1;
		} else {
			break;
		}
#endif
		spidev->rx_buffer = spidev->rx_buffer + SPI_FRAGMENT_SIZE;
	}
	if (status < 0)
		spi_err_log("error status %d\n", status);
	spidev->rx_buffer = tmp_rx_buf;

	return status;
}

static void rx_assemble_work(struct work_struct *work)
{
	struct spidev_data *spidev = g_spidev;
	struct spi_device *spi = spidev->spi;
	struct spi_controller *ctrl = spi->controller;
	struct spi_transfer t;
	int status = 0;
	int i, ret = 0, rest_fragment_count = 0;
	mutex_lock(&spidev->buf_lock);
	preempt_disable();
	spidev->spi_statistic.rx_assemble_count++;

	if (ap_response_flag) {
		spidev->spi_statistic.int_bottom_pass++;//pass irq bottom
		--ap_response_flag;
		mutex_unlock(&spidev->buf_lock);
		preempt_enable_no_resched();
		return;
	}

	// fix the last one work null pointer error
	if ((!spidev->tx_dummy_buffer) || (!spidev->rx_buffer)) {
		spi_err_log("null pointer\n");
		mutex_unlock(&spidev->buf_lock);
		preempt_enable_no_resched();
		return;
	}
	memset(&t, 0, sizeof(struct spi_transfer));
	t.tx_buf = spidev->tx_dummy_buffer;
	t.rx_buf = spidev->rx_buffer;
	t.len = BUFSIZE;

	spidev->spi_statistic.spidev_sync_2++;
	preempt_enable_no_resched();
	status = ctrl->transfer_one(ctrl, spi, &t);
	++frag_count;

	if (spidev->level > SPI_NO_DEBUG)
		printk_ratelimited("status = %d\n", status);

	if (spidev->level > SPI_HEADER_DEBUG) {
		printk("rx_buf\n");
		for (i = 0; i < SPI_FRAGMENT_SIZE; ++i) {
			printk(" 0x%x", spidev->rx_buffer[i]);
			if (!((i + 1) % 8))
				printk("\n");
		}
		printk("\n");
	}

#ifdef SPI_SLAVE_LINK_LAYER
	ret = spi_link_layer_correct_interface(spidev);
	if (ret !=  link_correct) {
		dev_err(&spidev->spi->dev,
					"rx_buf link_layer test fail ret %d\n", ret);
		if (spidev->level > SPI_HEADER_DEBUG) {
			printk("rx_buf\n");
			for (i = 0; i < SPI_FRAGMENT_SIZE; ++i) {
				printk(" 0x%x ", spidev->rx_buffer[i]);
				if (!((i + 1) % 8))
					printk("\n");
			}
			printk("\n");
		}
		mutex_unlock(&spidev->buf_lock);
		return;
	}
#endif
	spidev->spi_statistic.assemble_resolve_count++;
#ifdef SPI_TP_LAYER
		status = spi_tp_resolve_fragment_interface(spidev, &rest_fragment_count);
		if (status == -1) {//recv not finish
			recv_one_frag_call = 1;
			status = recv_one_fragment(spidev, &rest_fragment_count);
		}
		if (status < 0) {
			spi_err_log("error status %d", status);
			goto out;
		}
#else
		memcpy(spidev->spi_recv_buf.rx_tmp_buf_pos, spidev->rx_buffer, status);
#endif
		save_spi_data(spidev, status);
out:
	mutex_unlock(&spidev->buf_lock);
}

static ssize_t spidev_read(struct file *filp,
				char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned int frame_len = 0;
	ssize_t status = 0;
	unsigned int copied = 0;
	struct spidev_data *spidev = filp->private_data;

	if (filp->f_flags & O_NONBLOCK) {
		if (down_trylock(&spidev->sem))
			return 0;
	} else {
		if (down_interruptible(&spidev->sem))
			return -ERESTARTSYS;
	}

	if (!kfifo_peek(&spidev->rx_fifo.len, &frame_len)) {
		printk_ratelimited(KERN_INFO "hobot_spidev: no data\n");
		status = 0;
		goto err;
	}

	if (count < frame_len) {
		dev_err(&spidev->spi->dev, "data length error\n");
		status = -EMSGSIZE;
		goto err;
	}

	if (!kfifo_get(&spidev->rx_fifo.len, &frame_len)) {
		dev_err(&spidev->spi->dev, "kfifo empty\n");
		status = -EFAULT;
		goto err;
	}
	if (spidev->level > SPI_NO_DEBUG)
		printk_ratelimited(KERN_INFO "frame_len = %d\n", frame_len);

	status = kfifo_to_user(&spidev->rx_fifo.data, buf, frame_len, &copied);
	if (spidev->level > SPI_NO_DEBUG)
		printk_ratelimited(KERN_INFO "copied = %d\n", copied);

	if (copied != frame_len) {
		spidev->spi_statistic.kfifo_copy_error++;
		dev_err(&spidev->spi->dev, "kfifo copy failed\n");
	}

err:
	up(&spidev->sem);
	return status ? status : copied;
}
extern char tx_rx_interrupt_conflict_flag;

static int spi_send_message(struct spidev_data *spidev,
									char *src_buf, unsigned int len)
{
	struct spi_device *spi = spidev->spi;
	struct spi_controller *ctrl = spi->controller;

	char *tmp_src_buf = src_buf;
	char *tmp_tx_buf = spidev->tx_buffer;
	char *tmp_rx_buf = spidev->rx_buffer;
	struct spi_transfer t;


	int status = 0, i = 0, data_length = len;
	int frag_count = 0, tmp_rest_rx_frag_count;
	char ret = 0, tmp_rx_end_flag = 0, tp_frag_finish_flag = 0;


	while (tp_frag_finish_flag != 2) {	/* ret = 2 frag finish */
#ifdef SPI_TP_LAYER
		tp_frag_finish_flag = spi_slave_tp_frag_interface(tmp_src_buf,
											tmp_tx_buf + 4, data_length);
		if (tp_frag_finish_flag == -1) {
			dev_err(&spidev->spi->dev, "tp layer set error\n");
			status = -1;
			goto done;
		}
#endif

#ifdef SPI_SLAVE_LINK_LAYER
		if (spi_link_layer_set_interface(tmp_tx_buf) == -1) {
			dev_err(&spidev->spi->dev, "link layer set error\n");
			status = -2;
			goto done;
		}
#endif
		if (spidev->level > SPI_NO_DEBUG) {
			printk("spi_send_message: \n");
			for (i = 0; i < SPI_FRAGMENT_SIZE; i++) {
				printk(" 0x%x ", tmp_tx_buf[i]);
				if (i % 8 == 0)
					printk("\n");
			}
			printk("\n");
		}

		tmp_tx_buf += SPI_FRAGMENT_SIZE;
		frag_count++;
	}

	t.tx_buf = spidev->tx_buffer;
	t.rx_buf = spidev->rx_buffer;
	t.len = SPI_FRAGMENT_SIZE * frag_count;

	spidev->spi_statistic.spidev_sync_1++;
	status = ctrl->transfer_one(ctrl, spi, &t);
	if (status < 0) {
		dev_err(&spidev->spi->dev, "transfer_one failed, status %d\n", status);
		goto done;
	}

	if (spidev->level > SPI_NO_DEBUG) {
		printk("spi_send_message, recv data: \n");
		for (i = 0; i < SPI_FRAGMENT_SIZE * frag_count; i++) {
			printk(" 0x%x ", spidev->rx_buffer[i]);
			if (i % 8 == 0)
				printk("\n");
		}
		printk("\n");
	}

	if (tx_rx_interrupt_conflict_flag == 1) {
		tx_rx_interrupt_conflict_flag = 0;
		i = 0;
		while (i++ < frag_count) {
#ifdef SPI_SLAVE_LINK_LAYER
			ret = spi_link_layer_correct_interface(spidev);
			if (ret == link_sync_dummy) {
				spidev->rx_buffer += SPI_FRAGMENT_SIZE;
				continue;
			} else if (ret == link_correct) {
				spidev->spi_statistic.interrupt_conflict_1++;
			} else {
				spidev->spi_statistic.preamble_error++;
				spi_err_log("preamble_error error\n");
				status = -3;
				goto done;
			}
#endif
#ifdef SPI_TP_LAYER
			tmp_rx_end_flag = save_spi_data(spidev,
				spi_tp_resolve_fragment_interface(spidev, &tmp_rest_rx_frag_count));
			if (spi_tp_tx_start_or_end_interface(spidev) == start_frame
				|| spi_tp_tx_start_or_end_interface(spidev) == single_frame) {
				spidev->spi_statistic.interrupt_conflict_2++;
//					++ap_response_flag;
			} else {
				tmp_rest_rx_frag_count--;
			}
#endif
			spidev->rx_buffer += SPI_FRAGMENT_SIZE;
		}
		spidev->rx_buffer = tmp_rx_buf;
		if (tmp_rx_end_flag == -1) {//  =-1 -> rx continue
			tmp_rx_end_flag = save_spi_data(spidev,
								recv_rest_fragment(spidev, &tmp_rest_rx_frag_count));
		}
	}
done:
	spidev->rx_buffer = tmp_rx_buf;
//	preempt_enable_no_resched();

	return status;
}

static int spidev_send_kthread(void *data)
{
	struct spidev_data *spidev = data;
	int retval = 0, data_len = 0;

	while (1) {
		if (wait_event_interruptible(spidev->wait_queue, spidev->wait_condition)) {
			dev_err(&spidev->spi->dev, "wait_event_interruptible rc < 0\n");
			return -1;
		}
		spidev->wait_condition = 0;

		if (spidev->kthread_stop)
			break;

		if (spidev->noblock) {
			while (kfifo_get(&spidev->tx_fifo.len, &data_len)) {
				retval = -EFAULT;
				if (kfifo_out(&spidev->tx_fifo.data,
						spidev->tx_swap_buffer, data_len) != data_len) {
					dev_err(&spidev->spi->dev, "get fifo data failed\n");
					continue;
				}

				mutex_lock(&spidev->buf_lock);
				retval = spi_send_message(spidev,
									spidev->tx_swap_buffer, data_len);
				mutex_unlock(&spidev->buf_lock);

				if (retval < 0)
					dev_err(&spidev->spi->dev, "send message failed\n");
			}
		} else {
			retval = spi_send_message(spidev,
								spidev->tx_swap_buffer, spidev->data_len);
			if (retval < 0)
				dev_err(&spidev->spi->dev, "send message failed\n");
			complete(&spidev->completion);
		}
	}

	return 0;
}

static ssize_t spidev_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;
	unsigned int copied = 0;
	struct spidev_data *spidev = filp->private_data;

	if (filp->f_flags & O_NONBLOCK) {
		spidev->noblock = 1;
		spidev->wait_condition = 1;

		if (kfifo_avail(&spidev->tx_fifo.data) < count) {
			dev_err(&spidev->spi->dev, "tx fifo overflow\n");
			retval = -EOVERFLOW;
		}

		retval = kfifo_from_user(&spidev->tx_fifo.data, buf, count, &copied);
		if (spidev->level > SPI_NO_DEBUG)
			printk_ratelimited(KERN_INFO "copied = %d\n", copied);
		kfifo_put(&spidev->tx_fifo.len, count);

		wake_up_interruptible(&spidev->wait_queue);

		return count;
	} else {
		spidev->noblock = 0;
		mutex_lock(&spidev->buf_lock);
		if (copy_from_user(spidev->tx_swap_buffer,
								(const u8 __user *)buf, count)) {
			dev_err(&spidev->spi->dev, "copy from user failed\n");
			mutex_unlock(&spidev->buf_lock);
			return -EFAULT;
		}
		spidev->data_len = count;

		spidev->wait_condition = 1;
		wake_up_interruptible(&spidev->wait_queue);

		if (!wait_for_completion_interruptible_timeout
					(&spidev->completion, spidev->timeout)) {
			dev_err(&spidev->spi->dev, "completion timeout\n");
			mutex_unlock(&spidev->buf_lock);
			return -ETIME;
		}

		mutex_unlock(&spidev->buf_lock);
		return count;
	}
}

static long spidev_ioctl(struct file *filp,
					unsigned int cmd, unsigned long arg)
{
	int			retval = 0;
	struct spidev_data	*spidev;
	struct spi_device	*spi;
	u32			tmp;
	u32 timeout = 0;
	struct spi_error_status error_status;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spidev->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MODE32:
		retval = put_user(spi->mode & SPI_MODE_MASK,
					(__u32 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = put_user(spidev->speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
	case SPI_IOC_WR_MODE32:
		if (cmd == SPI_IOC_WR_MODE)
			retval = get_user(tmp, (u8 __user *)arg);
		else
			retval = get_user(tmp, (u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u16)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval >= 0)
				spidev->speed_hz = tmp;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
			spi->max_speed_hz = save;
		}
		break;
	case SPI_IO_GET_ERROR_STATUS:
		error_status.preamble_error = spidev->spi_statistic.preamble_error;
		spidev->spi_statistic.preamble_error = 0;

		error_status.rx_data_kfifo_overflow =
							spidev->spi_statistic.rx_data_kfifo_overflow;
		spidev->spi_statistic.rx_data_kfifo_overflow = 0;

		error_status.rx_tmp_buf_overflow =
							spidev->spi_statistic.rx_tmp_buf_overflow;
		spidev->spi_statistic.rx_tmp_buf_overflow = 0;

		if (copy_to_user((void __user *)arg,
							&error_status, sizeof(struct spi_error_status))) {
			dev_err(&spi->dev, "copy to user failed\n");
			retval = -EFAULT;
		}
		break;
	case SPI_IO_SET_TIMEOUT:
		if (copy_from_user(&timeout, (void __user *)arg, sizeof(u32))) {
			dev_err(&spi->dev, "copy from user failed\n");
			retval = -EFAULT;
			break;
		}
		if (timeout <= 0 || timeout > TIMEOUT_MS_MAX) {
			dev_err(&spi->dev, "timeout illegal\n");
			retval = -EFAULT;
			break;
		}
		spidev->timeout = msecs_to_jiffies(timeout);
		break;
	default:
		dev_err(&spi->dev, "invalid cmd\n");
		break;
	}

	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}


extern char spi_recv_rc, spi_send_rc;
static int spidev_open(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int	status = -ENXIO, i;

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	/* just init one time */
	if (spidev->users > 0) {
		dev_err(&spidev->spi->dev, "more than one user\n");
		status = -EBUSY;
		goto err_find_dev;
	}

	if (!spidev->tx_buffer) {
		spidev->tx_buffer = kmalloc(BUFSIZE, GFP_KERNEL);
		if (!spidev->tx_buffer) {
			dev_err(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_find_dev;
		}
	}

	if (!spidev->rx_buffer) {
		spidev->rx_buffer = kmalloc(BUFSIZE, GFP_KERNEL);
		if (!spidev->rx_buffer) {
			dev_err(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

	/* alloc tx_dummy_buffer */
	if (!spidev->tx_dummy_buffer) {
		spidev->tx_dummy_buffer = kmalloc(BUFSIZE, GFP_KERNEL);
		if (!spidev->tx_dummy_buffer) {
			dev_err(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_tx_dummy_buf;
		}
		memset(spidev->tx_dummy_buffer, 0xFD, BUFSIZE);
	}

	/* alloc tx_swap_buffer */
	if (!spidev->tx_swap_buffer) {
		spidev->tx_swap_buffer = kmalloc(BUFSIZE, GFP_KERNEL);
		if (!spidev->tx_swap_buffer) {
			dev_err(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_tx_dummy_buf;
		}
	}

	/* request tri pin */
	status = gpio_request(spidev->tri_pin, "tri-pin");
	if (status) {
		dev_err(&spidev->spi->dev, "request tri pin failed\n");
		goto err_request_tri_pin;
	}
	gpio_direction_output(spidev->tri_pin, 1);

	/* request irq pin */
	status = gpio_request(spidev->irq_pin, "irq-pin");
	if (status) {
		dev_err(&spidev->spi->dev, "request irq pin failed\n");
		goto err_request_irq_pin;
	}

	/* request interrupt */
	spidev->irq_num = gpio_to_irq(spidev->irq_pin);
	dev_info(&spidev->spi->dev, "irq_num = %d\n", spidev->irq_num);
	status = request_threaded_irq(spidev->irq_num, NULL, spidev_irq,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, HOBOT_SPIDEV, spidev);
	if (status) {
		dev_err(&spidev->spi->dev, "request irq failed\n");
		goto err_request_irq;
	}

	/* request ack pin */
	status = gpio_request(spidev->ack_pin, "ack-pin");
	if (status) {
		dev_err(&spidev->spi->dev, "request ack pin failed\n");
		goto err_request_ack_pin;
	}
	gpio_direction_output(spidev->ack_pin, 1);

	spidev->timeout = msecs_to_jiffies(TIMEOUT_MS_MAX);

	/* initialize rx_tmp_buf */
	spidev->spi_recv_buf.rx_tmp_buf = kmalloc(RX_TEMP_BUFFER_LEN, GFP_KERNEL);
	if (!spidev->spi_recv_buf.rx_tmp_buf) {
		dev_err(&spidev->spi->dev, "alloc rx_tmp_buf failed\n");
		status = -ENOMEM;
		goto err_alloc_rx_tmp_buf;
	}
	spidev->spi_recv_buf.rx_tmp_buf_pos = spidev->spi_recv_buf.rx_tmp_buf;
	spidev->spi_recv_buf.new_frame_start = 0;
	spidev->spi_recv_buf.rx_tmp_len = 0;
	spidev->spi_recv_buf.empty_len = RX_TEMP_BUFFER_LEN;
	spin_lock_init(&spidev->spi_recv_buf.rx_tmp_buf_lock);

	/* initialize rx kfifo */
	if (kfifo_alloc(&spidev->rx_fifo.data, DATA_FIFO_SIZE, GFP_KERNEL)) {
		dev_err(&spidev->spi->dev, "alloc rx kfifo failed\n");
		status = -ENOMEM;
		goto err_alloc_rx_kfifo;
	}
	INIT_KFIFO(spidev->rx_fifo.len);

	/* initialize tx kfifo */
	if (kfifo_alloc(&spidev->tx_fifo.data, DATA_FIFO_SIZE, GFP_KERNEL)) {
		dev_err(&spidev->spi->dev, "alloc tx kfifo failed\n");
		status = -ENOMEM;
		goto err_alloc_tx_kfifo;
	}
	INIT_KFIFO(spidev->tx_fifo.len);

	/* create single thread workqueue */
	spidev->work_queue = create_singlethread_workqueue("spidev_workqueue");
	if (!spidev->work_queue) {
		dev_err(&spidev->spi->dev, "create workqueue failed\n");
		status = -ENOMEM;
		goto err_create_workqueue;
	}

	init_waitqueue_head(&spidev->wait_queue);
	spidev->wait_condition = 0;
	spidev->kthread_stop = 0;

	spidev->send_kthread = kthread_run(spidev_send_kthread,
								spidev, "spidev-send-kthread");
	if (IS_ERR(spidev->send_kthread)) {
		dev_err(&spidev->spi->dev, "create kthread failed\n");
		status = -ENOMEM;
		goto err_create_kthread;
	}

	spi_recv_rc = 0;
	spi_send_rc = 0;

	for (i = 0; i < WORK_COUNT; i++)
		INIT_WORK(&spidev->work[i], rx_assemble_work);

	sema_init(&spidev->sem, 0);
	init_completion(&spidev->completion);
	ap_response_flag = 0;

	spidev->users++;
	filp->private_data = spidev;
	g_spidev = spidev;

	mutex_unlock(&device_list_lock);

	return 0;

err_create_kthread:
	destroy_workqueue(spidev->work_queue);
err_create_workqueue:
	kfifo_free(&spidev->tx_fifo.data);
err_alloc_tx_kfifo:
	kfifo_free(&spidev->rx_fifo.data);
err_alloc_rx_kfifo:
	kfree(spidev->spi_recv_buf.rx_tmp_buf);
	spidev->spi_recv_buf.rx_tmp_buf = NULL;
err_alloc_rx_tmp_buf:
	gpio_free(spidev->ack_pin);
err_request_ack_pin:
	free_irq(spidev->irq_num, spidev);
err_request_irq:
	gpio_free(spidev->irq_pin);
err_request_irq_pin:
	gpio_free(spidev->tri_pin);
err_request_tri_pin:
	kfree(spidev->tx_swap_buffer);
	spidev->tx_swap_buffer = NULL;
err_alloc_tx_dummy_buf:
	kfree(spidev->rx_buffer);
	spidev->rx_buffer = NULL;
err_alloc_rx_buf:
	kfree(spidev->tx_buffer);
	spidev->tx_buffer = NULL;
err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spidev_data *spidev;
	int	dofree;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	spidev->users--;

	spidev->kthread_stop = 1;
	spidev->wait_condition = 1;
	wake_up_interruptible(&spidev->wait_queue);
	kthread_stop(spidev->send_kthread);
	spidev->wait_condition = 0;

	destroy_workqueue(spidev->work_queue);

	kfifo_free(&spidev->tx_fifo.data);
	kfifo_free(&spidev->tx_fifo.data);

	kfree(spidev->spi_recv_buf.rx_tmp_buf);
	spidev->spi_recv_buf.rx_tmp_buf = NULL;

	gpio_free(spidev->ack_pin);
	free_irq(spidev->irq_num, (void *)spidev);
	gpio_free(spidev->irq_pin);
	gpio_free(spidev->tri_pin);

	mutex_lock(&spidev->buf_lock);
	kfree(spidev->tx_buffer);
	spidev->tx_buffer = NULL;

	kfree(spidev->rx_buffer);
	spidev->rx_buffer = NULL;

	kfree(spidev->tx_dummy_buffer);
	spidev->tx_dummy_buffer = NULL;

	kfree(spidev->tx_swap_buffer);
	spidev->tx_swap_buffer = NULL;
	mutex_unlock(&spidev->buf_lock);

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi)
		spidev->speed_hz = spidev->spi->max_speed_hz;

	dofree = (spidev->spi == NULL);
	spin_unlock_irq(&spidev->spi_lock);

	spi_recv_rc = 0;
	spi_send_rc = 0;

	if (dofree)
		kfree(spidev);

	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	.llseek =	no_llseek,
	.read = 	spidev_read,
	.write =	spidev_write,
	.unlocked_ioctl = spidev_ioctl,
	.open = 	spidev_open,
	.release =	spidev_release,
};

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "x2_spidev,slave" },
	{ .compatible = "hobot,spidev_slave" },
	{},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);

static int debug_level_show(struct seq_file *seq, void *v)
{
	struct spidev_data *spidev = seq->private;

	seq_printf(seq, "debug_level = %d\n", spidev->level);

    return 0;
}

static int debug_level_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_level_show, inode->i_private);
}

static ssize_t debug_level_write(struct file *file,
				const char __user *buffer, size_t count, loff_t *ppos)
{
    unsigned int level = 0;
    struct seq_file *seq = file->private_data;
	struct spidev_data *spidev = seq->private;
	struct spi_device *spi = spidev->spi;

    level = simple_strtoul(buffer, NULL, 10);

	if (level < SPI_DEBUG_LEVEL_MIN)
		spidev->level = SPI_DEBUG_LEVEL_MIN;
	else if (level > SPI_DEBUG_LEVEL_MAX)
		spidev->level = SPI_DEBUG_LEVEL_MAX;
	else
		spidev->level = level;

	dev_info(&spi->dev, "debug_level = %d\n", spidev->level);

    return count;
}

static struct file_operations debug_level_ops = {
	.owner   = THIS_MODULE,
	.open    = debug_level_open,
	.read    = seq_read,
	.write   = debug_level_write,
	.llseek  = seq_lseek,
	.release = single_release,
};
	
static ssize_t statistics_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
    struct seq_file *seq = file->private_data;
	struct spidev_data *spidev = seq->private;

	memset(&spidev->spi_statistic, 0, sizeof(struct spi_statistic));

	return count;
}

static int statistics_show(struct seq_file *seq, void *v)
{
	struct spidev_data *spidev = seq->private;
	struct spi_statistic *statistic = &spidev->spi_statistic;
	
	seq_printf(seq,
		"spi_statistics:\n"
		"start_frag_count = %d\n"
		"enqueued_frame = %d\n"
		"lost_start_frag_error = %d\n"
		"lost_middle_frag_error = %d"
		"lost_end_frag_error = %d\n"
		"rx_tmp_buf_overflow = %d\n"
		"rx_data_kfifo_overflow = %d\n"
		"preamble_error = %d\n"
		"kfifo_copy_error = %d\n"
		"interrupt_conflict_1 = %d"
		"interrupt_conflict_2 = %d\n"
		"eint_count = %d\n"
		"spidev_sync_1 = %d\n"
		"spidev_sync_2 = %d\n"
		"spidev_sync_3 = %d\n"
		"spi_info_ap_count = %d\n"
		"int_bottom_pass = %d\n"
		"rx_assemble_count = %d\n"
		"assemble_resolve_count = %d\n"
		"tx_recv_one_frag_resolve_count = %d\n"
		"rx_recv_one_frag_resolve_count = %d\n"
		"tx_custom_frame_count = %d\n"
		"ack_count = %d\n",
		statistic->start_frag_count,
		statistic->enqueued_frame,
		statistic->lost_start_frag_error,
		statistic->lost_middle_frag_error,
		statistic->lost_end_frag_error,
		statistic->rx_tmp_buf_overflow,
		statistic->rx_data_kfifo_overflow,
		statistic->preamble_error,
		statistic->kfifo_copy_error,
		statistic->interrupt_conflict_1,
		statistic->interrupt_conflict_2,
		statistic->eint_count,
		statistic->spidev_sync_1,
		statistic->spidev_sync_2,
		statistic->spidev_sync_3,
		statistic->spi_info_ap_count,
		statistic->int_bottom_pass,
		statistic->rx_assemble_count,
		statistic->assemble_resolve_count,
		statistic->tx_recv_one_frag_resolve_count, // call is 0
		statistic->rx_recv_one_frag_resolve_count, // call is 1
		statistic->tx_custom_frame_count,
		statistic->ack_count);
	
	return 0;
}

static int statistics_open(struct inode *inode, struct file *file)
{
	return single_open(file, statistics_show, inode->i_private);
}

static struct file_operations statistics_ops = {
	.owner   = THIS_MODULE,
	.open    = statistics_open,
	.read    = seq_read,
	.write   = statistics_write,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int hobot_spidev_debug_init(struct spidev_data	*spidev)
{
	int ret = 0;
	struct spi_device *spi = spidev->spi;

	spidev->root_dir = debugfs_create_dir(HOBOT_SPIDEV, NULL);
	if (spidev->root_dir == NULL) {
		dev_err(&spi->dev, "debugfs mkdir failed\n");
		return -EINVAL;
	}

	spidev->debug_level = debugfs_create_file("debug_level",
				0664, spidev->root_dir, spidev, &debug_level_ops);
	if (spidev->debug_level == NULL) {
		dev_err(&spi->dev, "debugfs create debug_level failed\n");
		ret = -EINVAL;
		goto create_err;
	}

	spidev->statistics = debugfs_create_file("statistics",
				0664, spidev->root_dir, spidev, &statistics_ops);
	if (spidev->statistics == NULL) {
		dev_err(&spi->dev, "debugfs create statistics failed\n");
		ret = -EINVAL;
		goto create_err;
	}

	return ret;

create_err:
	debugfs_remove_recursive(spidev->root_dir);
	return ret;
}

static void hobot_spidev_debug_deinit(struct spidev_data	*spidev)
{
	debugfs_remove_recursive(spidev->root_dir);
}

static int spidev_probe(struct spi_device *spi)
{
	struct spidev_data	*spidev;
	int			status;
	unsigned long		minor;

	dev_info(&spi->dev, "hobot spidev probe begin\n");	
	/*
	 * spidev should never be referenced in DT without a specific
	 * compatible string, it is a Linux implementation thing
	 * rather than a description of the hardware.
	 */
	if (spi->dev.of_node && !of_match_device(spidev_dt_ids, &spi->dev)) {
		dev_err(&spi->dev, "buggy DT: spidev listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
			!of_match_device(spidev_dt_ids, &spi->dev));
	}

	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;
	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);
	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
				    spidev, HOBOT_SPIDEV"%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
		goto device_create_err;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	spidev->speed_hz = spi->max_speed_hz;
	status = of_property_read_u32(spi->dev.of_node,
								"tri_pin", &spidev->tri_pin);
	if (status) {
		dev_err(&spi->dev, "get tri pin failed\n");
		goto get_pin_err;
	}
	dev_info(&spi->dev, "tri_pin = %d", spidev->tri_pin);

	status = of_property_read_u32(spi->dev.of_node,
								"irq_pin", &spidev->irq_pin);
	if (status) {
		dev_err(&spi->dev, "get irq pin failed\n");
		goto get_pin_err;
	}
	dev_info(&spi->dev, "irq_pin = %d", spidev->irq_pin);

	status = of_property_read_u32(spi->dev.of_node,
								"ack_pin", &spidev->ack_pin);
	if (status) {
		dev_err(&spi->dev, "get ack pin failed\n");
		goto get_pin_err;
	}
	dev_info(&spi->dev, "ack_pin = %d", spidev->ack_pin);

	status = hobot_spidev_debug_init(spidev);
	if (status) {
		dev_err(&spi->dev, "debug init failed\n");
		goto get_pin_err;
	}

	spi_set_drvdata(spi, spidev);

	dev_info(&spi->dev, "hobot spidev probe success\n");	

	return status;

get_pin_err:
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	mutex_unlock(&device_list_lock);
device_create_err:
	kfree(spidev);
	return status;
}

static int spidev_remove(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);

	hobot_spidev_debug_deinit(spidev);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spin_unlock_irq(&spidev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0) {
		kfree(spidev);
		g_spidev = NULL;
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		HOBOT_SPIDEV,
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (status < 0)
		return status;

	spidev_class = class_create(THIS_MODULE, HOBOT_SPIDEV);
	if (IS_ERR(spidev_class)) {
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
		return PTR_ERR(spidev_class);
	}

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		class_destroy(spidev_class);
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	}

	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("hobot spidev slave module");
MODULE_LICENSE("GPL v2");
