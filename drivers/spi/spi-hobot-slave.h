/*
 * Copyright 2018 <Copyright hobot>
 */

#ifndef __SPI_HOBOT_SLAVE_H__
#define __SPI_HOBOT_SLAVE_H__
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
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

//time_test
#include <linux/sched.h>
#include <linux/dcache.h>
#include <linux/string.h>
#include <asm/fcntl.h>
#include <asm/processor.h>
#include <asm/uaccess.h>

//#define _SPI_DEBUG_PRINTF_

#if defined _SPI_DEBUG_PRINTF_
#define spi_debug_log(format, ...) pr_debug("spi_slave: %s, %d "format, \
		__func__, __LINE__, ##__VA_ARGS__)

#define spi_err_log(format, ...) pr_err("spi_slave: %s, %d "format, \
		__func__, __LINE__, ##__VA_ARGS__)

#define spi_dump_log(buf, len) _dbg_printhex(buf, len)
#else
#define spi_debug_log(...)
#define spi_err_log(...) pr_err(__VA_ARGS__) /* err log always open */
#define spi_dump_log(buf, len)
#endif


#define SPI_SLAVE_LINK_LAYER
#define SPI_TP_LAYER

struct spi_statistic
{
	unsigned int start_frag_count;
	unsigned int enqueued_frame;
	unsigned int lost_start_frag_error;
	unsigned int lost_middle_frag_error;
	unsigned int lost_end_frag_error;
	unsigned int rx_tmp_buf_overflow;
	unsigned int rx_data_kfifo_overflow;
	unsigned int preamble_error;
	unsigned int kfifo_copy_error;
	unsigned int interrupt_conflict_1;
	unsigned int interrupt_conflict_2;
	unsigned int eint_count;
	unsigned int spidev_sync_1;
	unsigned int spidev_sync_2;
	unsigned int spidev_sync_3;
	unsigned int spi_info_ap_count;
	unsigned int int_bottom_pass;
	unsigned int rx_assemble_count;
	unsigned int assemble_resolve_count;
	unsigned int tx_recv_one_frag_resolve_count; // call is 0
	unsigned int rx_recv_one_frag_resolve_count; // call is 1
	unsigned int tx_custom_frame_count;
	unsigned int ack_count;
};

struct rx_tmp_buffer
{
	char *rx_tmp_buf;
	char *rx_tmp_buf_pos;
	int rx_tmp_len;
	int new_frame_start;
	int empty_len;
	spinlock_t rx_tmp_buf_lock;
};

#define DATA_FIFO_SIZE	(64 << 10)
#define LEN_FIFO_SIZE	(4 << 10)

struct data_fifo {
	struct kfifo data;
	DECLARE_KFIFO(len, unsigned int, LEN_FIFO_SIZE);
};

#define WORK_COUNT (64)

struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;
	u32			speed_hz;

	/* New element */
	int tri_pin;
	int irq_pin, irq_num;
	int ack_pin;

	struct dentry *root_dir;
	struct dentry *debug_level;
	struct dentry *statistics;
	int level;
	struct spi_statistic spi_statistic;

	u32 timeout;

	u8 *tx_dummy_buffer;

	u8 noblock;
	u32 data_len;
	u8 *tx_swap_buffer;

	struct rx_tmp_buffer spi_recv_buf;

	struct data_fifo rx_fifo;
	struct data_fifo tx_fifo;

	struct workqueue_struct *work_queue;

	wait_queue_head_t wait_queue;
	int wait_condition;
	int kthread_stop;
	struct task_struct *send_kthread;

	struct work_struct work[WORK_COUNT];
	struct semaphore sem;
	struct completion completion;
};

#define SPI_FRAGMENT_SIZE (256)
#define SPI_FRAGMENT_VALID_SIZE (256-8)

#define SPI_SYNC_CODE_OFFSET (0)
#define SPI_ROLLING_COUNTER_OFFSET    (1)
#define SPI_CRC_OFFSET    (2)
#define SPI_MASK_OFFSET      (4)
#define SPI_COUNT_OFFSET      (5)
#define SPI_TOTAL_LENGTH_OFFSET      (6)
#define SPI_DATA_OFFSET      (8)
#define SPI_DATA_LEN_OFFSET (9)

#define SPI_NO_DEBUG				(0)
#define SPI_HEADER_DEBUG			(1)
#define SPI_DETAIL_DEBUG			(2)
#define SPI_DEBUG_LEVEL_MIN   		(0)
#define SPI_DEBUG_LEVEL_MAX 		(2)

#define RX_TEMP_BUFFER_LEN (4 *1024)
#define TX_TEMP_BUFFER_LEN (4 *1024)

#define SPI_PREAMBLE (0x47)
#define DUMMY_FLAG   (0xFD)
extern int frag_count;
extern int ap_response_flag;
extern int recv_start_frag_flag;
extern int start_count;
extern int middle_count;
extern int end_count;

enum{
	sync_num_correct = 0,
	dummy_correct,
	unknown_error
};
enum{
	rolling_count_correct = 3,
	rolling_count_error
};
enum{
	crc16_correct = 5,
	crc16_error
};
enum{
	link_sync_correct = 7,
	link_correct,
	link_sync_dummy,
	link_sync_unknown_error,
	link_rolling_count_error,
	link_crc16_error
};
enum
{
	start_frame = 0,
	end_frame,
	single_frame,
	middle_frame
};
typedef struct
{
	unsigned char start : 1;
	unsigned char consecutive:1;
	unsigned char end : 1;
	unsigned char reserve:5;
} spi_mask;
typedef union
{
	spi_mask element_value;
	unsigned char byte_value;
} spi_header_byte;

extern int spi_tp_resolve_fragment_interface(struct spidev_data *spidev,
		int *rest_fragment_count);
extern int spi_slave_tp_frag_interface(char *src_buf, char *frag_buf,
		int data_length);
extern int spi_link_layer_correct_interface(struct spidev_data *spidev);
extern int spi_link_layer_set_interface(char *src_buf);
extern int spi_tp_tx_start_or_end_interface(struct spidev_data *spidev);
void hb_spi_info_ap(void);

#endif  // __SPI_HOBOT_SLAVE_H__
