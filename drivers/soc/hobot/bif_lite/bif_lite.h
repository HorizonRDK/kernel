#ifndef _BIF_LITE_H_
#define _BIF_LITE_H_
#include <linux/interrupt.h>
#include "../bif_base/bif_base.h"
#include "../bif_base/bif_api.h"
#include "bif_platform.h"

struct bif_frame_cache {
	int framelen;
	struct list_head frame_cache_list;
	unsigned char framecache[];
};

struct frag_flag {
	unsigned char start : 1;
	unsigned char end : 1;
	unsigned char pack1 : 6;
	unsigned char crc12_1 : 8;
	unsigned char crc12_2 : 4;
	unsigned char pack2 : 4;
	unsigned char pack3 : 8;
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
};

struct current_frame_info {
	int received_len;
	int malloc_len;
	unsigned char *pos_p;
	int pre_id;
};

struct comm_channel {
	// hardware channel concerned
	enum channel_id channel;
	enum BUFF_ID buffer_id;
	int transfer_align;
	addr_t base_addr;
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
	struct bif_frame_cache *rx_frame_cache_p;
	int rx_frame_count;
	// transfer buffer concerned
	struct current_frame_info current_frame;
	struct bif_rx_cache *recv_frag;
	unsigned char *send_fragment;
	// transfer feature concerned
	int block;
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

#endif
