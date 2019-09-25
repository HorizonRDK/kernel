/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2019 Horizon Robotics, Inc.
 *			 All rights reserved.
 */

#ifndef _BIF_LITE_H_
#define _BIF_LITE_H_
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include "../bif_base/bif_base.h"
#include "../bif_base/bif_api.h"
#include "bif_platform.h"

#define BIF_TX_ERROR_NO_MEM   (-1)
#define BIF_TX_ERROR_TRANS    (-2)

struct bif_frame_cache {
	int framelen;
	struct list_head frame_cache_list;
	unsigned char framecache[];
};

struct frag_flag {
	unsigned char start : 1;
	unsigned char end : 1;
	unsigned char pack1 : 6;
	unsigned char crc16_1 : 8;
	unsigned char crc16_2 : 8;
	unsigned char pack2 : 8;
};

struct frag_info {
	struct frag_flag flag;
	int id;
	int len;
};
#define FRAG_INFO_LEN (sizeof(struct frag_info))

struct bif_tx_ring_info {
	int send_tail;
};

struct bif_rx_ring_info {
	 int recv_head;
};

struct bif_rx_cache {
	unsigned short datalen;
	unsigned char datacache[];
};

enum channel_id {
	BIF_SPI,
	BIF_SD,
	NORM_SPI,
};

enum ap_type {
	SOC_AP,
	MCU_AP,
};

enum working_mode {
	INTERRUPT_MODE,
	POLLING_MODE,
};

struct channel_config {
	// hardware channel concerned
	enum channel_id channel;
	// memory limit concerned
	int frame_len_max;
	int frag_len_max;
	int frag_num;
	int frame_cache_max;
	// memory layout concerned
	addr_t rx_local_info_offset;
	addr_t rx_remote_info_offset;
	addr_t tx_local_info_offset;
	addr_t tx_remote_info_offset;
	addr_t rx_buffer_offset;
	addr_t tx_buffer_offset;
	int total_mem_size;
	// transfer feature concerned
	int block;
	enum ap_type type;
	enum working_mode mode;
	int crc_enable;
};

struct current_frame_info {
	int received_len;
	int malloc_len;
	unsigned char *pos_p;
	int pre_id;
};

struct comm_channel_statistics {
	int trig_count;
	int retrig_count;
};

struct comm_channel_error_statistics {
	int rx_error_sync_index;
	int rx_error_no_frag;
	int rx_error_read_frag;
	int rx_error_crc_check;
	int rx_error_malloc_frame;
	int rx_error_assemble_frag;
	int rx_error_update_index;
};

struct comm_channel {
	// hardware channel concerned
	enum channel_id channel;
	enum BUFF_ID buffer_id;
	int transfer_align;
	addr_t base_addr;
	addr_t base_addr_phy;
	// memory limit concerned
	int frame_len_max;
	int frag_len_max;
	int valid_frag_len_max;
	int frag_num;
	int frame_cache_max;
	// memory layout concerned
	addr_t rx_local_info_offset;
	addr_t rx_remote_info_offset;
	addr_t tx_local_info_offset;
	addr_t tx_remote_info_offset;
	addr_t rx_buffer_offset;
	addr_t tx_buffer_offset;
	int total_mem_size;
	struct bif_tx_ring_info *tx_local_info;
	struct bif_rx_ring_info *tx_remote_info;
	struct bif_tx_ring_info *rx_remote_info;
	struct bif_rx_ring_info *rx_local_info;
#ifdef CONFIG_HOBOT_BIF_AP
	char *rx_local_info_tmp_buf;
#endif
	struct mutex ring_info_lock;
	struct bif_frame_cache *rx_frame_cache_p;
	int rx_frame_count;
	spinlock_t rx_frame_count_lock;
	// transfer buffer concerned
	struct current_frame_info current_frame;
	struct bif_rx_cache *recv_frag;
	unsigned char *send_fragment;
	// transfer feature concerned
	int block;
	struct comm_channel_statistics channel_statistics;
	struct comm_channel_error_statistics error_statistics;
	enum ap_type type;
	enum working_mode mode;
	int crc_enable;
	int frame_start;
	int channel_ready;
	int tx_frag_avail;
	int tx_frag_index;
	int ap_abnormal_sync;
	// buffer index info
	int init_tx_remote_info;
	int init_tx_local_info;
	int init_rx_local_info;
	int init_rx_remote_info;
	int sync_tx_remote_info;
	int sync_tx_local_info;
	int sync_rx_local_info;
	int sync_rx_remote_info;
};

int channel_init(struct comm_channel *channel, struct channel_config *config);
void channel_deinit(struct comm_channel *channel);
int bif_lite_init(struct comm_channel *channel);
void bif_lite_exit(struct comm_channel *channel);
int bif_lite_register_irq(struct comm_channel *channel,
irq_handler_t handler);
void bif_del_frame_from_list(struct comm_channel *channel,
struct bif_frame_cache *frame);
int bif_rx_get_frame(struct comm_channel *channel,
struct bif_frame_cache **frame);
int bif_tx_put_frame(struct comm_channel *channel, void *data, int len);
int bif_rx_get_stock_frame(struct comm_channel *channel,
struct bif_frame_cache **frame);
int bif_start(struct comm_channel *channel);
int bif_stop(struct comm_channel *channel);
void bif_frame_decrease_count(struct comm_channel *channel);
void bif_del_frame_from_session_list(struct comm_channel *channel,
struct bif_frame_cache *frame);
int channel_stock_frame_num(struct comm_channel *channel);

#endif
