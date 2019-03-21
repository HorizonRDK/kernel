#include "bif_lite.h"
#ifndef CONFIG_HOBOT_BIF_AP
#include "../bif_base/bif_base.h"
#else
#include "bif_base.h"
#endif

static int bif_lite_start;
static struct bif_tx_ring_info *tx_local_info;
static struct bif_rx_ring_info *tx_remote_info;
static struct bif_tx_ring_info *rx_remote_info;
static struct bif_rx_ring_info *rx_local_info;
static struct bif_frame_cache *rx_frame_cache_p;
static int rx_frame_count;

static unsigned short crc16(unsigned char  *input, unsigned  int length)
{
	unsigned  int i;
	unsigned short poly = 0x1021;
	unsigned short result = 0xFFFF;
	unsigned char  value;

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

static inline int  bif_tx_get_available_buffer(int *index,
	int *count,
	int expect_count)
{
	int ret = 0;
	struct bif_rx_ring_info tx_remote_info_tmp;

	*index = -1;
	ret = bif_read_cp_ddr(&tx_remote_info_tmp,
		TX_REMOTE_INFO_OFFSET,
		sizeof(struct bif_rx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		return ret;
	}
#if 1
	bif_debug("tx_l.s_t %d\n", tx_local_info->send_tail);
	bif_debug("tx_r.r_h %d\n", tx_remote_info_tmp.recv_head);
#endif

	if (tx_remote_info_tmp.recv_head == -1)
		if (tx_local_info->send_tail == BUFFER_NUM-1)
			return  -1;

	if (tx_local_info->send_tail == tx_remote_info_tmp.recv_head)
		return  -1;

	*index =   (tx_local_info->send_tail) % BUFFER_NUM;

	if (tx_local_info->send_tail  > tx_remote_info_tmp.recv_head) {
		*count = tx_remote_info_tmp.recv_head+
			BUFFER_NUM-tx_local_info->send_tail;
	} else  if (tx_local_info->send_tail  < tx_remote_info_tmp.recv_head) {
		*count = tx_remote_info_tmp.recv_head-tx_local_info->send_tail;
	} else
		*count = 0;

	if (expect_count > *count)
		return	-1;

#if 0
	bif_debug("*count %d\n", *count);
	bif_debug("expect_count %d\n",  expect_count);
	bif_debug("*index %d\n", *index);
#endif

	*tx_remote_info = tx_remote_info_tmp;

	return ret;
}

static inline addr_t  bif_tx_index_to_addr(int index)
{
	addr_t  offset;

	if (index < 0)
		return 0;
	if (index >= BUFFER_NUM)
		return 0;
	offset =  TX_BUFFER_OFFSET +  index *  BUFFER_LEN;
	return offset;
}

static inline void  bif_tx_prepare_before_write(void)
{
}

static inline int   bif_tx_update_after_write(int index,
	struct frag_info  *fragment_info)
{
	tx_local_info->send_tail = (tx_local_info->send_tail  +  1) %
	BUFFER_NUM;
	tx_local_info->fragment_info[index] = *fragment_info;
#if 0
	bif_debug("after_write tx_local_info.send_tail %d\n",
	tx_local_info.send_tail);
	int i;
	unsigned char  *p;

	p = (unsigned char *)&tx_local_info;
	for (i = 0; i < sizeof(struct bif_tx_ring_info); i++)
		bif_debug(" %02x\n", *(p + i));

#endif
	return 0;
}

static inline int   bif_tx_update_to_cp_ddr(void)
{
	int ret = 0;

	tx_local_info->check_sum = 0;
	tx_local_info->check_sum = crc16((unsigned char *)tx_local_info,
		sizeof(struct bif_tx_ring_info));

	bif_debug("check_s %d\n", tx_local_info->check_sum);

	ret = bif_write_cp_ddr(tx_local_info,
		TX_LOCAL_INFO_OFFSET,
		sizeof(struct bif_tx_ring_info));
	if (ret < 0)
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);

	return ret;
}

static inline int bif_tx_put_data_to_buffer(int index,
	void *data,
	struct frag_info  *fragment_info)
{
	addr_t offset;
	int ret = 0;

	if (fragment_info == NULL) {
		ret =  -EPERM;
		goto err;
	}
	if (fragment_info->len  >  BUFFER_LEN) {
		ret =  -EPERM;
		goto err;
	}
	if (data == NULL) {
		ret =  -EFAULT;
		goto err;
	}
	offset = bif_tx_index_to_addr(index);
	if (offset < 0) {
		ret =  -ENOMEM;
		goto err;
	}
	bif_tx_prepare_before_write();
	bif_debug("index %d, len %d\n", index,    fragment_info->len);
	bif_debug("id %d start %d  end %d\n", fragment_info->id,
		fragment_info->start,
		fragment_info->end);
	ret = bif_write_cp_ddr(data,  offset,   fragment_info->len);
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	ret = bif_tx_update_after_write(index, fragment_info);
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
#if 0
	unsigned char *value;

	value = (unsigned char *)data;
	if (*value != len) {
		bif_debug("tx value %02x\n", *value);
		bif_debug("len %02x\n", len);
	}
#endif

err:
	return ret;
}

static inline int  bif_tx_cut_fragment(unsigned char *data,  int len)
{
	int frag_count = 0;
	int last_copy = 0;
	int i = 0;
	unsigned char *frag_p = data;
	int ret = 0;
	int count;
	int index;

	struct frag_info fragment_info;

	// calculate fragment count & last copy byte
	frag_count  =  len / BUFFER_LEN;
	if (len % BUFFER_LEN)
		++frag_count;
	last_copy = len % BUFFER_LEN;
	if (last_copy == 0)
		last_copy = BUFFER_LEN;

	bif_debug("len %d l_c %d fr_c %d\n", len,  last_copy,   frag_count);

	ret = bif_tx_get_available_buffer(&index, &count,  frag_count);
	if (ret < 0)
		goto err;

	bif_debug("ava c %d\n", count);
	bif_debug("ava i %d\n", index);

	for (i = 0; i  <  frag_count; ++i) {
		if (i  ==  0)
			fragment_info.start  =  1;
		else
			fragment_info.start  =  0;

		if (i  ==  frag_count-1)
			fragment_info.end = 1;
		else
			fragment_info.end = 0;

		fragment_info.id = frag_count-i;

		if (fragment_info.start  == 1) {
			if (fragment_info.end  ==  0)
				fragment_info.len = BUFFER_LEN;
			else
				fragment_info.len = len;
		} else{
			if (fragment_info.end  ==  0)
				fragment_info.len = BUFFER_LEN;
			else
				fragment_info.len = last_copy;
		}

		ret = bif_tx_put_data_to_buffer(index, frag_p,  &fragment_info);
		if (ret < 0) {
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			goto err;
		}
		index++;
		index = index%BUFFER_NUM;
		frag_p  +=  BUFFER_LEN;

	}
	bif_tx_update_to_cp_ddr();

err:
	return ret;

}

int bif_tx_put_frame(void *data,  int len)
{
//for fragment cut case,  we need to handle more
	int ret;

	ret = bif_tx_cut_fragment(data,  len);
	return ret;
}

static inline int  bif_rx_get_available_buffer(int *index,
	int *count)
{
	int ret = 0;
	unsigned short  check_sum = 0;
	unsigned short  check_sum_expect = 0;
	struct bif_tx_ring_info *rx_remote_info_tmp;

	rx_remote_info_tmp = bif_malloc(sizeof(struct bif_tx_ring_info));
	if (rx_remote_info_tmp == NULL)
		goto err;

	ret = bif_read_cp_ddr(rx_remote_info_tmp,
		RX_REMOTE_INFO_OFFSET,
		sizeof(struct bif_tx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
#if 1
	//int i;
	bif_debug("rx_r.s_t %d\n", rx_remote_info_tmp->send_tail);
	bif_debug("rx_l.r_h %d\n", rx_local_info->recv_head);
	//for (i = 0;i < BUFFER_NUM;i++)
	{
	//	bif_debug("i%d, frag_len %d\n", i,
	//rx_remote_info.fragment_info[i].len);
	}
#endif

	 check_sum = rx_remote_info_tmp->check_sum;
	 rx_remote_info_tmp->check_sum = 0;
	 check_sum_expect =  crc16((unsigned char *)rx_remote_info_tmp,
		sizeof(struct bif_tx_ring_info));

	if (check_sum != check_sum_expect) {
		*count = 0;
		*index = -1;
		bif_debug("check_s %d   %d\n", check_sum,   check_sum_expect);
		goto err;
	}

	if (rx_remote_info_tmp->send_tail  ==
		rx_local_info->recv_head + 1) {
		*count = 0;
		*index = -1;
	} else if (rx_remote_info_tmp->send_tail  >
		rx_local_info->recv_head) {
		*count = rx_remote_info_tmp->send_tail
			-rx_local_info->recv_head-1;
		*index = (rx_local_info->recv_head + 1) % BUFFER_NUM;
	} else  if (rx_remote_info_tmp->send_tail  <=
		rx_local_info->recv_head) {
		*count = rx_remote_info_tmp->send_tail + BUFFER_NUM
			-rx_local_info->recv_head-1;
		*index = (rx_local_info->recv_head + 1) % BUFFER_NUM;
	}

	if (*count > 0)
		*rx_remote_info = *rx_remote_info_tmp;

err:
	if (rx_remote_info_tmp)
		bif_free(rx_remote_info_tmp);

	return ret;
}

static inline  addr_t bif_rx_index_to_addr(int index)
{
	return RX_BUFFER_OFFSET +  index *  BUFFER_LEN;
}
static inline  void  bif_rx_prepare_before_read(void)
{
}

static  inline int  bif_rx_update_after_read(int count)
{
	int ret = 0;

	rx_local_info->recv_head  +=  count;
	rx_local_info->recv_head  =  (rx_local_info->recv_head) % BUFFER_NUM;
	ret = bif_write_cp_ddr(rx_local_info,
		RX_LOCAL_INFO_OFFSET,
		sizeof(struct bif_rx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	return 0;
err:
	return ret;
}

static inline int bif_rx_add_frame_to_list(
	struct bif_frame_cache  *frame_cache_tmp)
{
	int ret = 0;

	bif_lock();
	list_add_tail(&frame_cache_tmp->frame_cache_list,
		&rx_frame_cache_p->frame_cache_list);
	rx_frame_count++;
//	bif_debug("rx_frame_count %d\n", rx_frame_count);
	bif_unlock();
	return ret;
}

static inline int  bif_rx_reassemble_fragment(
	struct bif_frame_cache *frame_p,
	struct bif_rx_cache *cache_tmp,
	struct frag_info  *fragment_info)
{
	static int received_len;
	static int malloc_len;
	static unsigned char  *pos_p;
	static int pre_id;
	int ret = 0;

	bif_debug("id %d start %d  end %d  len %d\n", fragment_info->id,
		fragment_info->start,
		fragment_info->end,
		fragment_info->len);
	if (frame_p == NULL) {
		ret = -ENOMEM;
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	if (fragment_info == NULL) {
		ret = -ENOMEM;
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	if (fragment_info->id == 0) {
		ret =  -EPERM;
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}

	if (fragment_info->start == 1) {
		received_len = 0;
		malloc_len = fragment_info->id*BUFFER_LEN;

		if (fragment_info->len > malloc_len) {
			ret =  -EPERM;
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			goto err;
		}

		bif_memcpy(frame_p->framecache,
			cache_tmp->datacache,
			fragment_info->len);
		pos_p = frame_p->framecache + fragment_info->len;
		received_len += fragment_info->len;
	} else {
		if (pre_id != (fragment_info->id + 1)) {
			ret =  -EPERM;
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			goto err;
		}
		if (received_len == 0) {
			ret =  -EPERM;
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			goto err;
		}
		if (!pos_p) {
			ret =  -EPERM;
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			goto err;
		}
		if (fragment_info->len + received_len  >  malloc_len) {
			ret =  -EPERM;
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			goto err;
		}
		bif_memcpy(pos_p, cache_tmp->datacache, fragment_info->len);
		pos_p += fragment_info->len;
		received_len += fragment_info->len;
	}

	if (fragment_info->end == 1) {
		if (fragment_info->id != 1) {
			ret =  -EPERM;
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			goto err;
		}
		bif_debug("good frame\n");
		frame_p->framelen = received_len;
		ret = bif_rx_add_frame_to_list(frame_p);
		if (ret < 0) {
			ret =  -EPERM;
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			goto err;
		} else{
			frame_p = NULL;
			ret = 1;
		}
	}

	pre_id = fragment_info->id;

err:

	return ret;
}

static inline int bif_rx_get_cache_from_buffer(void)
{
	int index = 0;
	int count = 0;
	int ret = 0;
	int count_tmp = 0;
	int index_tmp = 0;
	addr_t  offset = 0;
	struct bif_rx_cache *cache_tmp = NULL;
	struct bif_frame_cache *frame_p = NULL;
	unsigned int malloc_len = 0;
	unsigned int frame_used_frag_count = 0;

	if (rx_frame_count > FRAME_CACHE_MAX) {
		bif_debug("too much cache!");
		return 1;
		//too much cache,  cost too much mem, need to wait a moment
	}

	ret = bif_rx_get_available_buffer(&index,  &count);
	bif_debug("ret %d  index %d  count %d\n", ret,  index, count);
	if (ret < 0)
		goto err;

	if (count == 0) {
		ret =  -EAGAIN;
		goto err;
	}
	if (count > BUFFER_NUM) {
		ret =  -EFAULT;
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}

	bif_rx_prepare_before_read();
	index_tmp = index;
	for (count_tmp = 0;   count_tmp < count; count_tmp++) {
		ret = 0;
		cache_tmp = NULL;
#if 1
		bif_debug("off %llx  ind %d  cou %d len %d\n",
			offset,
			index_tmp,
			count_tmp,
			rx_remote_info->fragment_info[index_tmp].len);
#endif
		if (rx_remote_info->fragment_info[index_tmp].start == 1) {
			malloc_len =
			rx_remote_info->fragment_info[index_tmp].id
			*BUFFER_LEN;
			frame_p =
			bif_malloc(sizeof(struct bif_frame_cache) + malloc_len);
			if (frame_p == NULL) {
				bif_err("bif_err: %s  %d\n",
					__func__,  __LINE__);
				ret = -1;
				break;
			}
		}
		cache_tmp = bif_malloc(sizeof(struct bif_rx_cache)+
			rx_remote_info->fragment_info[index_tmp].len);
		if (cache_tmp == NULL) {
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			ret =  -2;
			break;
		}
		offset = bif_rx_index_to_addr(index_tmp);
		ret = bif_read_cp_ddr(cache_tmp->datacache, offset,
			rx_remote_info->fragment_info[index_tmp].len);
		if (ret < 0) {
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			ret =  -3;
			break;
		}
		cache_tmp->datalen =
		rx_remote_info->fragment_info[index_tmp].len;
		ret = bif_rx_reassemble_fragment(frame_p, cache_tmp,
			&rx_remote_info->fragment_info[index_tmp]);
		if (ret < 0) {
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			ret = -4;
			break;
		}
		if (ret == 1) {
			frame_p = NULL;
			frame_used_frag_count = count_tmp + 1;
		}
		index_tmp =  (index_tmp +  1) % BUFFER_NUM;

		bif_free(cache_tmp);
		cache_tmp = NULL;

		if (rx_frame_count > FRAME_CACHE_MAX) {
			bif_debug("too much cache!");
			ret = -5;
			break;
	//too much cache,  cost too much mem, need to wait a moment
		}
	}

	if ((ret == -1) || (ret == -2) || (ret == -3) || (ret == -4)) {
		if (cache_tmp)
			bif_free(cache_tmp);

		if (frame_p)
			bif_free(frame_p);

	}
	if (frame_used_frag_count) {
		ret = bif_rx_update_after_read(frame_used_frag_count);
		if (ret < 0) {
			bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
			goto err;
		}
	}
	return ret;
err:
	return ret;
}

static inline int bif_clear_list(void)
{
	int ret = 0;
	struct bif_frame_cache  *frame_cache_tmp;

	bif_lock();
	if (list_empty(&rx_frame_cache_p->frame_cache_list)) {
		ret =  0;
		goto err;
	}
	do {
		frame_cache_tmp  =  list_first_entry(
			&rx_frame_cache_p->frame_cache_list,
			struct bif_frame_cache,
			frame_cache_list);

		if (frame_cache_tmp) {
			list_del(&frame_cache_tmp->frame_cache_list);
			bif_free(frame_cache_tmp);
			bif_debug("clear! %p", frame_cache_tmp);
		}
	} while (!(list_empty(&rx_frame_cache_p->frame_cache_list)));

	rx_frame_count = 0;
	bif_unlock();
	return 0;
err:
	bif_unlock();
	return ret;
}

static inline int  bif_detect_frame_from_list(
	struct bif_frame_cache  **frame)
{
	int ret;
	struct bif_frame_cache  *frame_cache_tmp;

	bif_lock();
	if (list_empty(&rx_frame_cache_p->frame_cache_list)) {
		ret =  -ENOMEM;
		goto err;
	}
	frame_cache_tmp  =  list_first_entry(
		&rx_frame_cache_p->frame_cache_list,
		struct bif_frame_cache,
		frame_cache_list);

	if (frame_cache_tmp)
		*frame = frame_cache_tmp;
	else
		*frame = NULL;

	bif_unlock();
	return 0;
err:
	bif_unlock();
	return ret;
}

void	bif_del_frame_from_list(struct bif_frame_cache   *frame)
{
	bif_lock();
	if (frame) {
		list_del(&frame->frame_cache_list);
		rx_frame_count--;
		bif_free(frame);
	}
	bif_unlock();
}

int bif_rx_get_frame(struct bif_frame_cache   **frame)
{
	int ret = 0;

	ret = bif_detect_frame_from_list(frame);
	if (ret == 0)
		return 0;
	ret =  bif_rx_get_cache_from_buffer();
	if (ret < 0)
		goto err;

	ret = bif_detect_frame_from_list(frame);
err:

	return ret;
}
static inline int bif_sync_before_start(void)
{
	int ret = 0;

	ret = bif_read_cp_ddr(tx_remote_info,
		TX_REMOTE_INFO_OFFSET,
		sizeof(struct bif_rx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		return ret;
	}
	ret = bif_read_cp_ddr(tx_local_info,
		TX_LOCAL_INFO_OFFSET,
		sizeof(struct bif_tx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		return ret;
	}
	ret = bif_read_cp_ddr(rx_remote_info,
		RX_REMOTE_INFO_OFFSET,
		sizeof(struct bif_tx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		return ret;
	}
	ret = bif_read_cp_ddr(rx_local_info,
		RX_LOCAL_INFO_OFFSET,
		sizeof(struct bif_rx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		return ret;
	}
	return 0;
}

static inline  int bif_init_cp_ddr(void)
{
	int ret = 0;
	struct bif_rx_ring_info *tx_remote_info_tmp = NULL;
	struct bif_tx_ring_info *tx_local_info_tmp = NULL;
	struct bif_rx_ring_info *rx_local_info_tmp = NULL;
	struct bif_tx_ring_info *rx_remote_info_tmp = NULL;

	tx_remote_info_tmp = bif_malloc(sizeof(struct bif_rx_ring_info));
	if (tx_remote_info_tmp == NULL) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	bif_memset(tx_remote_info_tmp, 0, sizeof(struct bif_rx_ring_info));
	tx_remote_info_tmp->recv_head = -1;
	ret = bif_write_cp_ddr(tx_remote_info_tmp,
		TX_REMOTE_INFO_OFFSET,
		sizeof(struct bif_rx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	tx_local_info_tmp = bif_malloc(sizeof(struct bif_tx_ring_info));
	if (tx_local_info_tmp == NULL) {
		bif_err("%s  %d\n", __func__,  __LINE__);
		goto err;
	}
	bif_memset(tx_local_info_tmp, 0, sizeof(struct bif_tx_ring_info));
	ret = bif_write_cp_ddr(tx_local_info_tmp,
		TX_LOCAL_INFO_OFFSET,
		sizeof(struct bif_tx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	rx_local_info_tmp = bif_malloc(sizeof(struct bif_rx_ring_info));
	if (rx_local_info_tmp == NULL) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	bif_memset(rx_local_info_tmp, 0, sizeof(struct bif_rx_ring_info));
	rx_local_info_tmp->recv_head = -1;
	ret = bif_write_cp_ddr(rx_local_info_tmp,
		RX_LOCAL_INFO_OFFSET,
		sizeof(struct bif_rx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	rx_remote_info_tmp = bif_malloc(sizeof(struct bif_tx_ring_info));
	if (rx_remote_info_tmp == NULL) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		goto err;
	}
	bif_memset(rx_remote_info_tmp, 0, sizeof(struct bif_tx_ring_info));
	ret = bif_write_cp_ddr(rx_remote_info_tmp,
		RX_REMOTE_INFO_OFFSET,
		sizeof(struct bif_tx_ring_info));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
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

int bif_start(void)
{
	bif_init_cp_ddr();
	bif_sync_before_start();
	bif_clear_list();
	return 0;
}

int bif_stop(void)
{
	bif_clear_list();
	return 0;
}

int bif_lite_init(void)
{
	int ret = 0;
	addr_t base_addr_tmp;
	addr_t base_addr_tmp_phy;
	int bif_lite_start  =  0;

	bif_lite_start = 0;
	bif_debug("%s() init,  begin...\n", __func__);

#ifndef CONFIG_HOBOT_BIF_AP
	base_addr_tmp  =  bif_alloc_cp(BUFF_LITE,  TOTAL_MEM_SIZE,
		(unsigned long *)&base_addr_tmp_phy);

	if (base_addr_tmp <= 0) {
		ret =  -EFAULT;
		bif_debug("bif_alloc_cp fail\n ");
		goto err;
	}

	bif_set_base_addr(base_addr_tmp);
	bif_register_address(BUFF_LITE,
		(void *)(unsigned long)base_addr_tmp_phy);

#else
	bif_base_probe();
	bif_sync_base();
	base_addr_tmp_phy  =  bif_query_address(BUFF_LITE);
	bif_debug("base_addr_tmp_phy %llx\n", base_addr_tmp_phy);
	base_addr_tmp  =  bif_phys_to_vaddr(base_addr_tmp_phy,
		TOTAL_MEM_SIZE);
	if (base_addr_tmp == 0) {
		ret =  -EFAULT;
		goto err;
	}
	bif_set_base_addr(base_addr_tmp_phy);

#endif
	bif_debug("base_addr %llx    %llx\n", base_addr_tmp, base_addr_tmp_phy);
	tx_local_info = bif_malloc(sizeof(struct bif_tx_ring_info));
	if (tx_local_info == NULL) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		return  -ENOMEM;
	}
	tx_remote_info = bif_malloc(sizeof(struct bif_rx_ring_info));
	if (tx_remote_info == NULL) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		return  -ENOMEM;
	}
	rx_remote_info = bif_malloc(sizeof(struct bif_tx_ring_info));
	if (rx_remote_info == NULL) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		return  -ENOMEM;
	}
	rx_local_info = bif_malloc(sizeof(struct bif_rx_ring_info));
	if (rx_local_info == NULL) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		return  -ENOMEM;
	}

	bif_init_cp_ddr();
	bif_sync_before_start();

	rx_frame_cache_p = bif_malloc(sizeof(struct bif_frame_cache));
	if (rx_frame_cache_p == NULL) {
		bif_err("bif_err: %s  %d\n", __func__,  __LINE__);
		return  -ENOMEM;
	}
	INIT_LIST_HEAD(&rx_frame_cache_p->frame_cache_list);

	bif_lite_start  =  1;
	bif_debug("%s() init,  end\n", __func__);
err:
	return ret;
}

void bif_lite_exit(void)
{
	bif_debug("%s() exit,  begin...\n", __func__);
	bif_lite_start  =  0;
	if (tx_local_info)
		bif_free(tx_local_info);

	if (tx_remote_info)
		bif_free(tx_remote_info);

	if (rx_remote_info)
		bif_free(rx_remote_info);

	if (rx_local_info)
		bif_free(rx_local_info);

	bif_debug("%s() exit,  end\n", __func__);
}
