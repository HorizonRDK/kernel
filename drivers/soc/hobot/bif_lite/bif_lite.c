#include "bif_lite.h"

#define RING_INFO_ALIGN (512)

static unsigned short crc16(unsigned char  *input, unsigned  int length)
{
	unsigned  int i;
	unsigned short poly = 0x1021;
	unsigned short result = 0xFFFF;
	unsigned char value;

	while (length--) {
		value = *(input++);
		result ^= (value << 8);

		for (i = 0; i < 8; i++) {
			if (result & 0x8000)
				result = (result << 1) ^ poly;
			else
				result = result << 1;
		}
	}

	return result;
}

#ifdef CONFIG_HOBOT_BIF_AP
static int swap_bytes_order(unsigned char *value, uint16_t size)
{
	uint16_t i = 0;
	uint32_t *p = (uint32_t *)value;

	for (i = 0; i < size; ) {
		*p = __builtin_bswap32(*p);
		++p;
		i += 4;
	}

	return 0;
}
#endif

static void bif_set_base_addr(struct comm_channel *channel,
addr_t bif_base_addr)
{
	channel->base_addr = bif_base_addr;
}

static int bif_write_cp_ddr_channel(struct comm_channel *channel,
void *src, addr_t offset, int len)
{
	void *dst = NULL;

	if (!src) {
		bif_debug("src NULL\n");
		return -1;
	}
	dst = (void *)(channel->base_addr + offset);

#ifndef CONFIG_HOBOT_BIF_AP
	bif_memcpy(dst, src, len);
#else
	if (channel->channel == BIF_SPI) {
		if (bif_spi_write(dst, len, src))
			return -1;
	} else if (channel->channel == BIF_SD) {
		if (bif_sd_write(dst, len, src))
			return -1;
	}
#endif

	return 0;
}

static int bif_read_cp_ddr_channel(struct comm_channel *channel,
void *dst, addr_t offset, int len)
{
	void *src = NULL;

	if (!dst) {
		bif_debug("dst NULL\n");
		return -1;
	}
	src = (void *)(channel->base_addr + offset);

#ifndef CONFIG_HOBOT_BIF_AP
	bif_memcpy(dst, src, len);
#else
	if (channel->channel == BIF_SPI) {
		if (bif_spi_read(src, len, dst))
			return -1;
	} else if (channel->channel == BIF_SD) {
		if (bif_sd_read(src, len, dst))
			return -1;
	}
#endif

	return 0;
}

static inline int bif_tx_get_available_buffer(
struct comm_channel *channel, int *index, int *count, int expect_count)
{
	int ret = 0;
	char tx_remote_info_buf[ALIGN(sizeof(struct bif_rx_ring_info),
		RING_INFO_ALIGN)];
	struct bif_rx_ring_info *tx_remote_info_tmp =
		(struct bif_rx_ring_info *)tx_remote_info_buf;

	*index = -1;
	ret = bif_read_cp_ddr_channel(channel, tx_remote_info_tmp,
		channel->tx_remote_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		return ret;
	}
#if 1
	bif_debug("tx_local_info->send_tail = %d\n",
		channel->tx_local_info->send_tail);
	bif_debug("tx_remote_info_tmp->recv_head = %d\n",
		tx_remote_info_tmp->recv_head);
#endif
	if (tx_remote_info_tmp->recv_head == -1)
		if (channel->tx_local_info->send_tail ==
			channel->frag_num - 1)
			return -1;

	if (channel->tx_local_info->send_tail ==
		tx_remote_info_tmp->recv_head)
		return -1;

	*index = (channel->tx_local_info->send_tail) % channel->frag_num;

	if (channel->tx_local_info->send_tail >
		tx_remote_info_tmp->recv_head) {
		*count = tx_remote_info_tmp->recv_head +
			channel->frag_num -
			channel->tx_local_info->send_tail;
	} else if (channel->tx_local_info->send_tail <
		tx_remote_info_tmp->recv_head) {
		*count = tx_remote_info_tmp->recv_head -
			channel->tx_local_info->send_tail;
	} else
		*count = 0;

	if (expect_count > *count)
		return -1;

	*(channel->tx_remote_info) = *tx_remote_info_tmp;

	return ret;
}

static inline addr_t bif_tx_index_to_addr(
struct comm_channel *channel, int index)
{
	addr_t offset;

	if (index < 0)
		return 0;
	if (index >= channel->frag_num)
		return 0;
	offset = channel->tx_buffer_offset + index * channel->frag_len_max;

	return offset;
}

static inline void bif_tx_prepare_before_write(
struct comm_channel *channel)
{

}

static inline int bif_tx_update_after_write(
struct comm_channel *channel, int index,
struct frag_info *fragment_info)
{
	channel->tx_local_info->send_tail =
		(channel->tx_local_info->send_tail + 1) %
		channel->frag_num;

	return 0;
}

static inline int bif_tx_update_to_cp_ddr(struct comm_channel *channel)
{
	int ret = 0;
	unsigned long remainning_time = 0;

	ret = bif_write_cp_ddr_channel(channel, channel->tx_local_info,
		channel->tx_local_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0)
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
	if (channel->channel == BIF_SPI)
		swap_bytes_order((unsigned char *)(channel->tx_local_info),
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
#endif
re_trig:
	if (bif_send_irq(channel->buffer_id) < 0) {
		printk("re_trig\n");
		remainning_time = bif_sleep(200);
		if (!remainning_time)
			goto re_trig;
		else {
			pr_info("re_trig sleep interrupt\n");
			return ret;
		}
	}

	return ret;
}

static inline int bif_tx_put_data_to_buffer(
struct comm_channel *channel, int index, void *data,
struct frag_info *fragment_info)
{
	addr_t offset = 0;
	int ret = 0;
	unsigned short fragment_len = 0;

	if (!fragment_info) {
		ret = -EPERM;
		goto err;
	}
	if (fragment_info->len > channel->valid_frag_len_max) {
		ret = -EPERM;
		goto err;
	}
	if (!data) {
		ret = -EFAULT;
		goto err;
	}
	offset = bif_tx_index_to_addr(channel, index);
	if (offset < 0) {
		ret = -ENOMEM;
		goto err;
	}
	bif_tx_prepare_before_write(channel);
	bif_debug("index = %d len = %d\n", index, fragment_info->len);
	bif_debug("id = %d start = %d  end = %d\n", fragment_info->id,
		fragment_info->flag.start, fragment_info->flag.end);

	bif_memcpy(channel->send_fragment, fragment_info,
		sizeof(struct frag_info));
	bif_memcpy(channel->send_fragment + sizeof(struct frag_info),
		data, fragment_info->len);
	fragment_len = sizeof(struct frag_info) + fragment_info->len;
	//printk("fragment_len = %d\n", fragment_len);

	// just write valid part of fragment
	ret = bif_write_cp_ddr_channel(channel, channel->send_fragment,
	offset,
	ALIGN(fragment_len, channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto err;
	}

	ret = bif_tx_update_after_write(channel, index,
		fragment_info);
	if (ret < 0) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto err;
	}
err:
	return ret;
}

static inline int  bif_tx_cut_fragment(struct comm_channel *channel,
unsigned char *data, int len)
{
	int frag_count = 0;
	int last_copy = 0;
	int i = 0;
	unsigned char *frag_p = data;
	int ret = 0;
	int count = 0;
	int index = 0;
	struct frag_info fragment_info;
	unsigned long remainning_time = 0;
	//int retry_count = 0;

	// calculate fragment count & last copy byte
	frag_count = len / channel->valid_frag_len_max;
	if (len % channel->valid_frag_len_max)
		++frag_count;
	last_copy = len % channel->valid_frag_len_max;
	if (last_copy == 0)
		last_copy = channel->valid_frag_len_max;

	bif_debug("len = %d l_c = %d fr_c = %d\n",
		len, last_copy, frag_count);
resend:
	ret = bif_tx_get_available_buffer(channel, &index,
		&count, frag_count);
	//if (ret < 0)
		//goto err;
	if (ret < 0) {
		// retry more, but trigger less
		//++retry_count;
		//if (!(retry_count % 40)) {
		//	printk("re_send\n");
		//	bif_send_irq(channel->buffer_id);
		//}
		remainning_time = bif_sleep(5);
		if (!remainning_time)
			goto resend;
		else {
			pr_info("bif_sleep interruptible\n");
			goto err;
		}
	}

	bif_debug("count =  %d\n", count);
	bif_debug("index =  %d\n", index);

	for (i = 0; i < frag_count; ++i) {
		if (i == 0)
			fragment_info.flag.start = 1;
		else
			fragment_info.flag.start = 0;

		if (i == frag_count - 1)
			fragment_info.flag.end = 1;
		else
			fragment_info.flag.end = 0;

		fragment_info.id = frag_count - i;

		if (fragment_info.flag.start == 1) {
			if (fragment_info.flag.end == 0)
				fragment_info.len =
				channel->valid_frag_len_max;
			else
				fragment_info.len = len;
		} else{
			if (fragment_info.flag.end == 0)
				fragment_info.len =
				channel->valid_frag_len_max;
			else
				fragment_info.len = last_copy;
		}

		ret = bif_tx_put_data_to_buffer(channel, index,
			frag_p, &fragment_info);
		if (ret < 0) {
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			goto err;
		}
		index++;
		index = index % channel->frag_num;
		frag_p += channel->valid_frag_len_max;
	}
	bif_tx_update_to_cp_ddr(channel);
err:
	return ret;
}

int bif_tx_put_frame(struct comm_channel *channel, void *data, int len)
{
	int ret;

	ret = bif_tx_cut_fragment(channel, data, len);

	return ret;
}
EXPORT_SYMBOL(bif_tx_put_frame);

static inline int bif_rx_get_available_buffer(
struct comm_channel *channel, int *index, int *count)
{
	int ret = 0;
	char rx_remote_info_buf[ALIGN(sizeof(struct bif_tx_ring_info),
RING_INFO_ALIGN)];

	struct bif_tx_ring_info *rx_remote_info_tmp =
(struct bif_tx_ring_info *)rx_remote_info_buf;

	ret = bif_read_cp_ddr_channel(channel, rx_remote_info_tmp,
		channel->rx_remote_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto err;
	}
#if 0
	pr_info("rx_r.s_t %d\n", rx_remote_info_tmp->send_tail);
	pr_info("rx_l.r_h %d\n", rx_local_info->recv_head);
#endif
	if (rx_remote_info_tmp->send_tail ==
		channel->rx_local_info->recv_head + 1) {
		*count = 0;
		*index = -1;
	} else if (rx_remote_info_tmp->send_tail >
		channel->rx_local_info->recv_head) {
		*count = rx_remote_info_tmp->send_tail
			- channel->rx_local_info->recv_head - 1;
		*index = (channel->rx_local_info->recv_head + 1) %
			channel->frag_num;
	} else if (rx_remote_info_tmp->send_tail <=
		channel->rx_local_info->recv_head) {
		*count = rx_remote_info_tmp->send_tail + channel->frag_num
			- channel->rx_local_info->recv_head - 1;
		*index = (channel->rx_local_info->recv_head + 1) %
			channel->frag_num;
	}

	if (*count > 0)
		*(channel->rx_remote_info) = *rx_remote_info_tmp;
err:
		return ret;
}

static inline addr_t bif_rx_index_to_addr(
struct comm_channel *channel, int index)
{
	return channel->rx_buffer_offset + index * channel->frag_len_max;
}

static inline void bif_rx_prepare_before_read(
struct comm_channel *channel)
{

}

static inline int bif_rx_update_after_read(struct comm_channel *channel,
int count)
{
	int ret = 0;

	channel->rx_local_info->recv_head += count;
	channel->rx_local_info->recv_head =
		(channel->rx_local_info->recv_head) % channel->frag_num;
	ret = bif_write_cp_ddr_channel(channel, channel->rx_local_info,
		channel->rx_local_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}

#ifdef CONFIG_HOBOT_BIF_AP
	if (channel->channel == BIF_SPI)
		swap_bytes_order((unsigned char *)(channel->rx_local_info),
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
#endif
	return 0;
err:
	return ret;
}

static inline int bif_rx_add_frame_to_list(
struct comm_channel *channel, struct bif_frame_cache *frame_cache_tmp)
{
	int ret = 0;

	bif_lock();
	list_add_tail(&frame_cache_tmp->frame_cache_list,
		&(channel->rx_frame_cache_p->frame_cache_list));
	++channel->rx_frame_count;
	bif_unlock();
	return ret;
}

static inline int bif_rx_reassemble_fragment(
struct comm_channel *channel, struct bif_frame_cache *frame_p,
struct bif_rx_cache *cache_tmp, struct frag_info *fragment_info)
{
	int ret = 0;

	bif_debug("id: %d start: %d  end: %d  len: %d\n", fragment_info->id,
		fragment_info->flag.start,
		fragment_info->flag.end,
		fragment_info->len);
	if (!frame_p) {
		ret = -ENOMEM;
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto err;
	}
	if (!fragment_info) {
		ret = -ENOMEM;
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto err;
	}
	if (fragment_info->id == 0) {
		ret = -EPERM;
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto err;
	}

	if (fragment_info->flag.start == 1) {
		channel->current_frame.received_len = 0;
		channel->current_frame.malloc_len =
		fragment_info->id * channel->valid_frag_len_max;

		if (fragment_info->len > channel->current_frame.malloc_len) {
			ret = -EPERM;
			bif_err("bif_err: %s  %d\n", __func__, __LINE__);
			goto err;
		}

		bif_memcpy(frame_p->framecache,
			cache_tmp->datacache + sizeof(struct frag_info),
			fragment_info->len);
		channel->current_frame.pos_p =
			frame_p->framecache + fragment_info->len;
		channel->current_frame.received_len += fragment_info->len;
	} else {
		if (channel->current_frame.pre_id != (fragment_info->id + 1)) {
			ret = -EPERM;
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			goto err;
		}
		if (channel->current_frame.received_len == 0) {
			ret = -EPERM;
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			goto err;
		}
		if (!(channel->current_frame.pos_p)) {
			ret = -EPERM;
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			goto err;
		}
		if (fragment_info->len + channel->current_frame.received_len >
			channel->current_frame.malloc_len) {
			ret = -EPERM;
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			goto err;
		}
		bif_memcpy(channel->current_frame.pos_p,
cache_tmp->datacache + sizeof(struct frag_info), fragment_info->len);
		channel->current_frame.pos_p += fragment_info->len;
		channel->current_frame.received_len += fragment_info->len;
	}

	if (fragment_info->flag.end == 1) {
		if (fragment_info->id != 1) {
			ret = -EPERM;
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			goto err;
		}
		bif_debug("good frame\n");
		frame_p->framelen = channel->current_frame.received_len;
		ret = bif_rx_add_frame_to_list(channel, frame_p);
		if (ret < 0) {
			ret = -EPERM;
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			goto err;
		} else{
			frame_p = NULL;
			ret = 1;
		}
	}

	channel->current_frame.pre_id = fragment_info->id;
err:
	return ret;
}

static inline int bif_rx_get_cache_from_buffer(
struct comm_channel *channel)
{
	int index = 0;
	int count = 0;
	int ret = 0;
	int count_tmp = 0;
	int index_tmp = 0;
	addr_t offset = 0;
	struct bif_rx_cache *cache_tmp = channel->recv_frag;
	struct bif_frame_cache *frame_p = NULL;
	unsigned int malloc_len = 0;
	unsigned int frame_used_frag_count = 0;
	struct frag_info *fragment_info = NULL;
#if 0
	if (channel->rx_frame_count > channel->frame_cache_max) {
		//too much cache, cost too much mem, need to wait a moment
		bif_debug("too much cache\n");
		return 1;
	}
#endif

	ret = bif_rx_get_available_buffer(channel, &index, &count);
	bif_debug("ret = %d  index = %d  count = %d\n", ret, index, count);
	if (ret < 0)
		goto err;
	if (count == 0) {
		ret = -EAGAIN;
		goto err;
	}
	if (count > channel->frag_num) {
		ret = -EFAULT;
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto err;
	}

	bif_rx_prepare_before_read(channel);
	index_tmp = index;
	for (count_tmp = 0; count_tmp < count; count_tmp++) {
		ret = 0;
		offset = bif_rx_index_to_addr(channel, index_tmp);
		// read a fragment every time
		ret = bif_read_cp_ddr_channel(channel, cache_tmp->datacache,
			offset, channel->frag_len_max);
		if (ret < 0) {
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			ret = -3;
			break;
		}
		// get fragment_info from fragment header
		fragment_info = (struct frag_info *)(cache_tmp->datacache);

		// at start fragment, malloc frame buffer
		if (fragment_info->flag.start == 1) {
			malloc_len = fragment_info->id
				* channel->valid_frag_len_max;
			frame_p =
			bif_malloc(sizeof(struct bif_frame_cache) + malloc_len);
			if (!frame_p) {
				bif_err("bif_err: %s %d\n",
					__func__, __LINE__);
				ret = -1;
				break;
			}
		}

		cache_tmp->datalen = fragment_info->len;
		ret = bif_rx_reassemble_fragment(channel, frame_p,
			cache_tmp, fragment_info);
		if (ret < 0) {
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			ret = -4;
			break;
		}
		// reassemble fragment function return 1,
		// when received a whole frame
		if (ret == 1) {
			frame_p = NULL;
			frame_used_frag_count = count_tmp + 1;
		}
		index_tmp = (index_tmp + 1) % channel->frag_num;
#if 0
		if (channel->rx_frame_count > channel->frame_cache_max) {
			//too much cache, cost too much mem,
			//need to wait a moment
			bif_debug("too much cache\n");
			ret = -5;
			break;
		}
#endif
	}

	if ((ret == -1) || (ret == -2) || (ret == -3) || (ret == -4)) {
		if (frame_p)
			bif_free(frame_p);
	}
	if (frame_used_frag_count) {
		ret = bif_rx_update_after_read(channel,
			frame_used_frag_count);
		if (ret < 0) {
			bif_err("bif_err: %s %d\n", __func__, __LINE__);
			goto err;
		}
	}

	return ret;
err:
	return ret;
}

static inline int bif_clear_list(struct comm_channel *channel)
{
	int ret = 0;
	struct bif_frame_cache *frame_cache_tmp = NULL;

	bif_lock();
	if (list_empty(&(channel->rx_frame_cache_p->frame_cache_list))) {
		ret = 0;
		goto err;
	}
	do {
		frame_cache_tmp = list_first_entry(
			&(channel->rx_frame_cache_p->frame_cache_list),
			struct bif_frame_cache,
			frame_cache_list);

		if (frame_cache_tmp) {
			list_del(&frame_cache_tmp->frame_cache_list);
			bif_free(frame_cache_tmp);
			bif_debug("clear! %p", frame_cache_tmp);
		}
	} while (!(list_empty(&(channel->rx_frame_cache_p->frame_cache_list))));

	channel->rx_frame_count = 0;
	bif_unlock();
	return 0;
err:
	bif_unlock();
	return ret;
}

static inline int bif_detect_frame_from_list(
struct comm_channel *channel, struct bif_frame_cache **frame)
{
	int ret = 0;
	struct bif_frame_cache *frame_cache_tmp = NULL;

	bif_lock();
	if (list_empty(&(channel->rx_frame_cache_p->frame_cache_list))) {
		ret = -ENOMEM;
		goto err;
	}
	frame_cache_tmp = list_first_entry(
		&(channel->rx_frame_cache_p->frame_cache_list),
		struct bif_frame_cache,
		frame_cache_list);

	if (frame_cache_tmp)
		*frame = frame_cache_tmp;
	else {
		ret = -ENOMEM;
		goto err;
	}

	bif_unlock();
	return 0;
err:
	bif_unlock();
	return ret;
}

void bif_del_frame_from_list(struct comm_channel *channel,
struct bif_frame_cache *frame)
{
	bif_lock();
	if (frame) {
		list_del(&frame->frame_cache_list);
		--channel->rx_frame_count;
		bif_free(frame);
	}
	bif_unlock();
}
EXPORT_SYMBOL(bif_del_frame_from_list);

void bif_frame_decrease_count(struct comm_channel *channel)
{
	--channel->rx_frame_count;
}
EXPORT_SYMBOL(bif_frame_decrease_count);

void bif_del_frame_from_session_list(struct comm_channel *channel,
struct bif_frame_cache *frame)
{
	--channel->rx_frame_count;
	bif_free(frame);
}
EXPORT_SYMBOL(bif_del_frame_from_session_list);

int bif_rx_get_frame(struct comm_channel *channel,
struct bif_frame_cache **frame)
{
	int ret = 0;

	ret = bif_detect_frame_from_list(channel, frame);
	if (ret == 0)
		return 0;

	ret = bif_rx_get_cache_from_buffer(channel);
	if (ret < 0)
		goto err;

	ret = bif_detect_frame_from_list(channel, frame);
err:
	return ret;
}
EXPORT_SYMBOL(bif_rx_get_frame);

int bif_rx_get_stock_frame(struct comm_channel *channel,
struct bif_frame_cache **frame)
{
	int ret = 0;

	ret = bif_detect_frame_from_list(channel, frame);

	return ret;
}
EXPORT_SYMBOL(bif_rx_get_stock_frame);

static inline int bif_sync_before_start(struct comm_channel *channel)
{
	int ret = 0;

	ret = bif_read_cp_ddr_channel(channel, channel->tx_remote_info,
		channel->tx_remote_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		return ret;
	}
	pr_info("sync: tx_remote_info %d\n",
	channel->tx_remote_info->recv_head);

	ret = bif_read_cp_ddr_channel(channel, channel->tx_local_info,
		channel->tx_local_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		return ret;
	}
	pr_info("sync: tx_local_info %d\n",
	channel->tx_local_info->send_tail);

	ret = bif_read_cp_ddr_channel(channel, channel->rx_remote_info,
		channel->rx_remote_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		return ret;
	}
	pr_info("sync: rx_remote_info %d\n",
	channel->rx_remote_info->send_tail);

	ret = bif_read_cp_ddr_channel(channel, channel->rx_local_info,
		channel->rx_local_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		return ret;
	}
	pr_info("sync: rx_local_info %d\n",
	channel->rx_local_info->recv_head);
	return 0;
}

static inline int bif_init_cp_ddr(struct comm_channel *channel)
{
	int ret = 0;
	struct bif_rx_ring_info *tx_remote_info_tmp = NULL;
	struct bif_tx_ring_info *tx_local_info_tmp = NULL;
	struct bif_rx_ring_info *rx_local_info_tmp = NULL;
	struct bif_tx_ring_info *rx_remote_info_tmp = NULL;

	tx_remote_info_tmp = bif_malloc(
	ALIGN(sizeof(struct bif_rx_ring_info), channel->transfer_align));
	if (!tx_remote_info_tmp) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}
	bif_memset(tx_remote_info_tmp, 0,
	ALIGN(sizeof(struct bif_rx_ring_info), channel->transfer_align));
	tx_remote_info_tmp->recv_head = -1;
	ret = bif_write_cp_ddr_channel(channel, tx_remote_info_tmp,
		channel->tx_remote_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}

	tx_local_info_tmp = bif_malloc(
	ALIGN(sizeof(struct bif_tx_ring_info), channel->transfer_align));
	if (!tx_local_info_tmp) {
		bif_err("%s  %d\n", __func__, __LINE__);
		goto err;
	}
	bif_memset(tx_local_info_tmp, 0,
	ALIGN(sizeof(struct bif_tx_ring_info), channel->transfer_align));
	ret = bif_write_cp_ddr_channel(channel, tx_local_info_tmp,
		channel->tx_local_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}

	rx_local_info_tmp = bif_malloc(
	ALIGN(sizeof(struct bif_rx_ring_info), channel->transfer_align));
	if (!rx_local_info_tmp) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}
	bif_memset(rx_local_info_tmp, 0,
	ALIGN(sizeof(struct bif_rx_ring_info), channel->transfer_align));
	rx_local_info_tmp->recv_head = -1;
	ret = bif_write_cp_ddr_channel(channel, rx_local_info_tmp,
		channel->rx_local_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}

	rx_remote_info_tmp = bif_malloc(
	ALIGN(sizeof(struct bif_tx_ring_info), channel->transfer_align));
	if (!rx_remote_info_tmp) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}
	bif_memset(rx_remote_info_tmp, 0,
	ALIGN(sizeof(struct bif_tx_ring_info), channel->transfer_align));
	ret = bif_write_cp_ddr_channel(channel, rx_remote_info_tmp,
		channel->rx_remote_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}
err:
	if (tx_remote_info_tmp)
		bif_free(tx_remote_info_tmp);

	if (tx_local_info_tmp)
		bif_free(tx_local_info_tmp);

	if (rx_local_info_tmp)
		bif_free(rx_local_info_tmp);

	if (rx_remote_info_tmp)
		bif_free(rx_remote_info_tmp);

	return ret;
}

int bif_start(struct comm_channel *channel)
{
	bif_clear_list(channel);
	return 0;
}
EXPORT_SYMBOL(bif_start);

int bif_stop(struct comm_channel *channel)
{
	bif_clear_list(channel);
	return 0;
}
EXPORT_SYMBOL(bif_stop);

int bif_lite_init(struct comm_channel *channel)
{
	int ret = 0;
#ifndef CONFIG_HOBOT_BIF_AP
	addr_t base_addr_tmp;
#endif
	addr_t base_addr_tmp_phy;

	bif_debug("%s init,  begin...\n", __func__);
#ifndef CONFIG_HOBOT_BIF_AP
	//base_addr_tmp = (addr_t)bif_alloc_cp(BUFF_LITE, TOTAL_MEM_SIZE,
	//	(unsigned long *)&base_addr_tmp_phy);
	base_addr_tmp = (addr_t)bif_dma_alloc(channel->total_mem_size,
	(dma_addr_t *)&base_addr_tmp_phy, GFP_KERNEL, 0);
	pr_info("bif_alloc_cp: vir_addr = %lx phy_addr = %lx total = %d\n",
	base_addr_tmp, base_addr_tmp_phy, channel->total_mem_size);

	if (base_addr_tmp <= 0) {
		ret = -EFAULT;
		bif_debug("bif_alloc_cp fail\n ");
		goto err;
	}

	// CP set virtual address, register physical address
	bif_set_base_addr(channel, base_addr_tmp);
	ret = bif_register_address(channel->buffer_id,
		(void *)(unsigned long)base_addr_tmp_phy);
	if (ret < 0)
		goto err;
#else
	// AP get and set physical address
	ret = bif_sync_base();
	if (ret < 0)
		goto err;
	base_addr_tmp_phy = (addr_t)bif_query_address(channel->buffer_id);
	if (base_addr_tmp_phy == -1) {
		ret = -1;
		goto err;

	}
	pr_info("base_addr_tmp_phy %lx\n", base_addr_tmp_phy);
	bif_set_base_addr(channel, base_addr_tmp_phy);
#endif
#ifndef CONFIG_HOBOT_BIF_AP
	ret = bif_init_cp_ddr(channel);
	if (ret < 0)
		goto err;
#endif
	ret = bif_sync_before_start(channel);
	if (ret < 0)
		goto err;

	bif_debug("%s init,  end\n", __func__);

	return 0;
err:
	return ret;
}
EXPORT_SYMBOL(bif_lite_init);

void bif_lite_exit(struct comm_channel *channel)
{
	bif_debug("%s exit, begin...\n", __func__);

	bif_debug("%s exit, end\n", __func__);
}
EXPORT_SYMBOL(bif_lite_exit);

int bif_lite_register_irq(struct comm_channel *channel,
irq_handler_t handler)
{
	int ret = 0;

	ret = bif_register_irq(channel->buffer_id, handler);

	return ret;
}
EXPORT_SYMBOL(bif_lite_register_irq);

static void dump_channel_info(struct comm_channel *channel)
{
	pr_info("==== hardware channel concerned ====\n");
	pr_info("channel = %d\n", channel->channel);
	pr_info("buffer_id = %d\n", channel->buffer_id);
	pr_info("transfer_align = %d\n", channel->transfer_align);
	pr_info("==== memory limit concerned ====\n");
	pr_info("base_addr = %lx\n", channel->base_addr);
	pr_info("frame_len_max = %d\n", channel->frame_len_max);
	pr_info("frag_len_max = %d\n", channel->frag_len_max);
	pr_info("valid_frag_len_max = %d\n", channel->valid_frag_len_max);
	pr_info("frag_num = %d\n", channel->frag_num);
	pr_info("frame_cache_max = %d\n", channel->frame_cache_max);
	pr_info("==== memory layout concerned ====\n");
	pr_info("rx_local_info_offset = %lx\n",
	channel->rx_local_info_offset);
	pr_info("rx_remote_info_offset = %lx\n",
	channel->rx_remote_info_offset);
	pr_info("tx_local_info_offset = %lx\n",
	channel->tx_local_info_offset);
	pr_info("tx_remote_info_offset = %lx\n",
	channel->tx_remote_info_offset);
	pr_info("rx_buffer_offset = %lx\n",
	channel->rx_buffer_offset);
	pr_info("tx_buffer_offset = %lx\n",
	channel->tx_buffer_offset);
	pr_info("total_mem_size = %d\n",
	channel->total_mem_size);
}

int channel_init(struct comm_channel *channel, struct channel_config *config)
{
	// hardware channel concerned
	channel->channel = config->channel;
	if (channel->channel == BIF_SPI) {
		channel->buffer_id = BUFF_SIO;
		channel->transfer_align = 16; // bif_spi 16B align
	} else if (channel->channel == BIF_SD) {
		channel->buffer_id = BUFF_LITE;
		channel->transfer_align = 512; // bif_sd 512 align
	}

	// memory limit concerned
	channel->frame_len_max = config->frame_len_max;
	channel->frag_len_max = config->frag_len_max;
	channel->valid_frag_len_max = channel->frag_len_max - FRAG_INFO_LEN;
	channel->frag_num = config->frag_num;
	channel->frame_cache_max = config->frame_cache_max;

	// memory layout concerned
	channel->rx_local_info_offset = config->rx_local_info_offset;
	channel->rx_remote_info_offset = config->rx_remote_info_offset;
	channel->tx_local_info_offset = config->tx_local_info_offset;
	channel->tx_remote_info_offset = config->tx_remote_info_offset;
	channel->rx_buffer_offset = config->rx_buffer_offset;
	channel->tx_buffer_offset = config->tx_buffer_offset;
	channel->total_mem_size = config->total_mem_size;
	channel->tx_local_info =
	bif_malloc(ALIGN(sizeof(struct bif_tx_ring_info),
	channel->transfer_align));
	if (!channel->tx_local_info) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto malloc_tx_local_fail;
	}

	channel->tx_remote_info =
	bif_malloc(ALIGN(sizeof(struct bif_rx_ring_info),
	channel->transfer_align));
	if (!channel->tx_remote_info) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto malloc_tx_remote_fail;
	}

	channel->rx_remote_info =
	bif_malloc(ALIGN(sizeof(struct bif_tx_ring_info),
	channel->transfer_align));
	if (!channel->rx_remote_info) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto malloc_rx_remote_fail;
	}

	channel->rx_local_info =
	bif_malloc(ALIGN(sizeof(struct bif_rx_ring_info),
	channel->transfer_align));
	if (!channel->rx_local_info) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto malloc_rx_local_fail;
	}

	channel->rx_frame_cache_p = bif_malloc(sizeof(struct bif_frame_cache));
	if (!channel->rx_frame_cache_p) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto malloc_frame_cache_fail;
	}
	INIT_LIST_HEAD(&(channel->rx_frame_cache_p->frame_cache_list));

	// transfer buffer concerned
	channel->recv_frag = bif_malloc(sizeof(struct bif_rx_cache) +
	channel->frag_len_max);
	if (!channel->recv_frag) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto malloc_recv_frag_fail;
	}

	channel->send_fragment = bif_malloc(channel->frag_len_max);
	if (!channel->send_fragment) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto malloc_send_frag_fail;
	}

	channel->current_frame.received_len = 0;
	channel->current_frame.malloc_len = 0;
	channel->current_frame.pre_id = 0;
	channel->current_frame.pos_p = NULL;

	// transfer feature concerned
	channel->block = config->block;

	dump_channel_info(channel);

	return 0;
malloc_send_frag_fail:
	bif_free(channel->recv_frag);
malloc_recv_frag_fail:
	bif_free(channel->rx_frame_cache_p);
malloc_frame_cache_fail:
	bif_free(channel->rx_local_info);
malloc_rx_local_fail:
	bif_free(channel->rx_remote_info);
malloc_rx_remote_fail:
	bif_free(channel->tx_remote_info);
malloc_tx_remote_fail:
	bif_free(channel->tx_local_info);
malloc_tx_local_fail:
	return -1;
}
EXPORT_SYMBOL(channel_init);

void channel_deinit(struct comm_channel *channel)
{
	bif_free(channel->send_fragment);
	bif_free(channel->recv_frag);
	bif_free(channel->rx_frame_cache_p);
	bif_free(channel->rx_local_info);
	bif_free(channel->rx_remote_info);
	bif_free(channel->tx_remote_info);
	bif_free(channel->tx_local_info);
}
EXPORT_SYMBOL(channel_deinit);
