/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2019 Horizon Robotics, Inc.
 *			 All rights reserved.
 */

#include "bif_lite.h"

#define RING_INFO_ALIGN (512)

static short const CRC16Table[256] = {
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static unsigned short crc16(unsigned char *dataIn, int length)
{
	unsigned short result = 0;
	unsigned short tableNo = 0;
	int i = 0;

	for (i = 0; i < length; i++) {
		tableNo = ((result & 0xff) ^ (dataIn[i] & 0xff));
		result = ((result >> 8) & 0xff) ^ CRC16Table[tableNo];
	}

	return result;
}

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

	mutex_lock(&channel->channel_sleep_lock);
	if (channel->channel_sleep_flag) {
		pr_info("write cp wait\n");
		mutex_unlock(&channel->channel_sleep_lock);
		wait_event_freezable(channel->channel_sleep_wq, !channel->channel_sleep_flag);
		pr_info("write cp wake\n");
		mutex_lock(&channel->channel_sleep_lock);
	}
#ifndef CONFIG_HOBOT_BIF_AP
	if (channel->type == MCU_AP)
		swap_bytes_order((unsigned char *)src, len);
	//bif_memcpy(dst, src, len);
	memcpy(dst, src, len);
#else
	if (channel->channel == BIF_SPI) {
		if (bif_spi_write(dst, len, src)) {
			mutex_unlock(&channel->channel_sleep_lock);
			return -1;
		}
	} else if (channel->channel == BIF_SD) {
		if (bif_sd_write(dst, len, src)) {
			mutex_unlock(&channel->channel_sleep_lock);
			return -1;
	}
	}
#endif
	mutex_unlock(&channel->channel_sleep_lock);

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

	mutex_lock(&channel->channel_sleep_lock);
	if (channel->channel_sleep_flag) {
		pr_info("read cp wait\n");
		mutex_unlock(&channel->channel_sleep_lock);
		wait_event_freezable(channel->channel_sleep_wq, !channel->channel_sleep_flag);
		pr_info("read cp wake\n");
		mutex_lock(&channel->channel_sleep_lock);
	}
#ifndef CONFIG_HOBOT_BIF_AP
	//bif_memcpy(dst, src, len);
	memcpy(dst, src, len);
	if (channel->type == MCU_AP)
		swap_bytes_order((unsigned char *)dst, len);
#else

	if (channel->channel == BIF_SPI) {
		if (bif_spi_read(src, len, dst)) {
			mutex_unlock(&channel->channel_sleep_lock);
			return -1;
		}
	} else if (channel->channel == BIF_SD) {
		if (bif_sd_read(src, len, dst)) {
			mutex_unlock(&channel->channel_sleep_lock);
			return -1;
	}
	}
#endif
	mutex_unlock(&channel->channel_sleep_lock);

	return 0;
}

// we can adjust this time when clear server scheme complete
#define ERROR_SYNC_DELAY (1000)
static inline int bif_tx_get_available_buffer(
struct comm_channel *channel, int *index, int *count, int expect_count)
{
	int ret = 0;
	char tx_remote_info_buf[ALIGN(sizeof(struct bif_rx_ring_info),
		RING_INFO_ALIGN)];
	struct bif_rx_ring_info *tx_remote_info_tmp =
		(struct bif_rx_ring_info *)tx_remote_info_buf;
#ifdef CONFIG_HOBOT_BIF_AP
	int remaining_time = 0;
#endif

	*index = -1;
#ifdef CONFIG_HOBOT_BIF_AP
	if (channel->hw_trans_error) {
		pr_info("tx get available delay\n");
		remaining_time = msleep_interruptible(ERROR_SYNC_DELAY);
		if (remaining_time) {
			pr_info("tx get available sleep interruptible\n");
			return -1;
		}
	}
	channel->hw_trans_error = 0;
#endif
	ret = bif_read_cp_ddr_channel(channel, tx_remote_info_tmp,
		channel->tx_remote_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
	channel->hw_trans_error = 1;
	if (channel->higher_level_clear)
		channel->higher_level_clear();
#endif
		return -2;
	}
#ifdef CONFIG_HOBOT_BIF_AP
	if (tx_remote_info_tmp->recv_head == -1) {
		if (!channel->ap_abnormal_sync) {
			channel->tx_local_info->send_tail = 0;
			channel->ap_abnormal_sync = 1;
			pr_info("tx CP reboot!!!\n");
		}
	} else
		channel->ap_abnormal_sync = 0;
#endif
#if 0
	pr_info("tx_local_info->send_tail = %d\n",
		channel->tx_local_info->send_tail);
	pr_info("tx_remote_info_tmp->recv_head = %d\n",
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
	if (ret < 0) {
		printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
	channel->hw_trans_error = 1;
	if (channel->higher_level_clear)
		channel->higher_level_clear();
#endif
	}
#ifdef CONFIG_HOBOT_BIF_AP
	if ((channel->type == SOC_AP) &&
		(channel->channel == BIF_SPI))
		swap_bytes_order((unsigned char *)(channel->tx_local_info),
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
#else
	if ((channel->type == MCU_AP) &&
		(channel->channel == BIF_SPI))
		swap_bytes_order((unsigned char *)(channel->tx_local_info),
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
#endif
	if (channel->mode == INTERRUPT_MODE) {
		while (1) {
			ret = bif_send_irq(channel->buffer_id);
			++channel->channel_statistics.trig_count;
			if (ret < 0) {
				remainning_time = bif_sleep(200);
				if (!remainning_time)
					++channel->channel_statistics.retrig_count;
				else {
					pr_info("re_trig sleep interrupt\n");
					break;
				}
			} else
				break;
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
	int fragment_len = 0;
	unsigned short crc16_value = 0;
	struct frag_info *fragment_info_p = NULL;

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

	//bif_memcpy(channel->send_fragment, fragment_info,
	//	sizeof(struct frag_info));
	memcpy(channel->send_fragment, fragment_info,
		sizeof(struct frag_info));
	//bif_memcpy(channel->send_fragment + sizeof(struct frag_info),
	//	data, fragment_info->len);
	memcpy(channel->send_fragment + sizeof(struct frag_info),
		data, fragment_info->len);

	if (channel->crc_enable) {
		crc16_value = crc16(channel->send_fragment, channel->frag_len_max);
		fragment_info_p = (struct frag_info *)channel->send_fragment;
		fragment_info_p->flag.crc16_1 = crc16_value & 0xff;
		fragment_info_p->flag.crc16_2 = (crc16_value >> 8) & 0xff;
		//pr_info("crc16_1 = %d\ncrc16_2 = %d\n", fragment_info_p->flag.crc16_1, fragment_info_p->flag.crc16_2);

		// write whole fragment
		ret = bif_write_cp_ddr_channel(channel, channel->send_fragment,
		offset, channel->frag_len_max);
		if (ret < 0) {
			printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
			channel->hw_trans_error = 1;
			if (channel->higher_level_clear)
				channel->higher_level_clear();
#endif
			goto err;
		}
	} else {
		fragment_len = sizeof(struct frag_info) + fragment_info->len;

		// just write valid part of fragment
		ret = bif_write_cp_ddr_channel(channel, channel->send_fragment,
		offset,
		ALIGN(fragment_len, channel->transfer_align));
		if (ret < 0) {
			printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
			channel->hw_trans_error = 1;
			if (channel->higher_level_clear)
				channel->higher_level_clear();
#endif
			goto err;
		}
	}

	ret = bif_tx_update_after_write(channel, index,
		fragment_info);
	if (ret < 0) {
		printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
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
	struct frag_info fragment_info;

	// calculate fragment count & last copy byte
	frag_count = len / channel->valid_frag_len_max;
	if (len % channel->valid_frag_len_max)
		++frag_count;
	last_copy = len % channel->valid_frag_len_max;
	if (last_copy == 0)
		last_copy = channel->valid_frag_len_max;

	bif_debug("len = %d l_c = %d fr_c = %d\n",
		len, last_copy, frag_count);
#if 0
	if (channel->tx_frag_avail < frag_count) {
		ret = bif_tx_get_available_buffer(channel,
		&channel->tx_frag_index,
		&count, frag_count);
	if (ret < 0) {
		ret = BIF_TX_ERROR_NO_MEM;
			goto err;
		} else
			channel->tx_frag_avail = count;
		}
#endif
	ret = bif_tx_get_available_buffer(channel,
		&channel->tx_frag_index,
		&count, frag_count);
	if (ret == -1) {
		ret = BIF_TX_ERROR_NO_MEM;
		goto err;
	} else if (ret == -2) {
		ret = BIF_TX_ERROR_TRANS;
		goto err;
	}

	bif_debug("count =  %d\n", count);
	bif_debug("index =  %d\n", channel->tx_frag_index);

	memset(&fragment_info, 0, sizeof(fragment_info));
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

		ret = bif_tx_put_data_to_buffer(channel, channel->tx_frag_index,
			frag_p, &fragment_info);
		if (ret < 0) {
			ret = BIF_TX_ERROR_TRANS;
			printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
			goto err;
		}
		channel->tx_frag_index++;
		channel->tx_frag_index =
		channel->tx_frag_index % channel->frag_num;
		frag_p += channel->valid_frag_len_max;
	}
	bif_tx_update_to_cp_ddr(channel);
	//channel->tx_frag_avail -= frag_count;
err:
	return ret;
}

int bif_tx_put_frame(struct comm_channel *channel, void *data, int len)
{
	int ret;
#ifdef CONFIG_HOBOT_BIF_ETHERNET
	if (channel->channel == ETHERNET) {
		ret = hbeth_sendframe(data, len);
		if (ret < 0) {
			if (ret == -EAGAIN)
				ret = BIF_TX_ERROR_TIMEOUT;
			else {
				ret = BIF_TX_ERROR_TRANS;
			#ifdef CONFIG_HOBOT_BIF_AP
				if (channel->higher_level_clear)
					channel->higher_level_clear();
			#endif
			}
		}
	} else
#endif
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
#ifdef CONFIG_HOBOT_BIF_AP
	struct bif_rx_ring_info *rx_local_info_tmp =
(struct bif_rx_ring_info *)channel->rx_local_info_tmp_buf;
	int remaining_time = 0;
#endif
#ifdef CONFIG_HOBOT_BIF_AP
	if (channel->hw_trans_error) {
		pr_info("rx get available delay\n");
		remaining_time = msleep_interruptible(ERROR_SYNC_DELAY);
		if (remaining_time) {
			ret = -1;
			pr_info("rx get available sleep interruptible\n");
			goto err;
		}
	}
	channel->hw_trans_error = 0;
#endif

	ret = bif_read_cp_ddr_channel(channel, rx_remote_info_tmp,
		channel->rx_remote_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
		channel->hw_trans_error = 1;
		if (channel->higher_level_clear)
			channel->higher_level_clear();
#endif
		goto err;
	}

#ifdef CONFIG_HOBOT_BIF_AP
	ret = bif_read_cp_ddr_channel(channel,
		channel->rx_local_info_tmp_buf,
		channel->rx_local_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
		channel->hw_trans_error = 1;
		if (channel->higher_level_clear)
			channel->higher_level_clear();
		goto err;
	}
	if (rx_local_info_tmp->recv_head == -1) {
		channel->rx_local_info->recv_head = -1;
		pr_info("rx CP reboot!!!!\n");
	}
#endif
#if 0
	pr_info("rx_r.s_t %d\n", rx_remote_info_tmp->send_tail);
	pr_info("rx_l.r_h %d\n", channel->rx_local_info->recv_head);
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
		printk_ratelimited(KERN_INFO "bif_err: %s  %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
		channel->hw_trans_error = 1;
		if (channel->higher_level_clear)
			channel->higher_level_clear();
#endif
		goto err;
	}

#ifdef CONFIG_HOBOT_BIF_AP
	if ((channel->type == SOC_AP) &&
		(channel->channel == BIF_SPI))
		swap_bytes_order((unsigned char *)(channel->rx_local_info),
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
#else
	if ((channel->type == MCU_AP) &&
		(channel->channel == BIF_SPI))
		swap_bytes_order((unsigned char *)(channel->rx_local_info),
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
#endif

	return 0;
err:
	return ret;
}

int bif_rx_add_frame_to_list(
struct comm_channel *channel, struct bif_frame_cache *frame_cache_tmp)
{
	int ret = 0;

	bif_lock();
	list_add_tail(&frame_cache_tmp->frame_cache_list,
		&(channel->rx_frame_cache_p->frame_cache_list));
	spin_lock(&channel->rx_frame_count_lock);
	++channel->rx_frame_count;
	spin_unlock(&channel->rx_frame_count_lock);
	bif_unlock();
	return ret;
}
EXPORT_SYMBOL(bif_rx_add_frame_to_list);

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

		//bif_memcpy(frame_p->framecache,
		//	cache_tmp->datacache + sizeof(struct frag_info),
		//	fragment_info->len);
		memcpy(frame_p->framecache,
			cache_tmp->datacache + sizeof(struct frag_info),
			fragment_info->len);
		channel->current_frame.pos_p =
			frame_p->framecache + fragment_info->len;
		channel->current_frame.received_len += fragment_info->len;
	} else {
		if (channel->current_frame.pre_id != (fragment_info->id + 1)) {
			ret = -EPERM;
			bif_err("bif_err: %s %d[%d_%d]\n", __func__, __LINE__,
			channel->current_frame.pre_id, fragment_info->id + 1);
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
		//bif_memcpy(channel->current_frame.pos_p,
		//cache_tmp->datacache +
		//sizeof(struct frag_info), fragment_info->len);
		memcpy(channel->current_frame.pos_p,
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
	unsigned short crc16_value_get = 0;
	unsigned short crc16_value_want = 0;
#if 0
	if (channel->rx_frame_count > channel->frame_cache_max) {
		//too much cache, cost too much mem, need to wait a moment
		bif_debug("too much cache\n");
		return 1;
	}
#endif
	ret = bif_rx_get_available_buffer(channel, &index, &count);
#if 0
	pr_info("ret = %d index = %d count = %d\n", ret, index, count);
#endif
	if (ret < 0) {
		++channel->error_statistics.rx_error_sync_index;
		goto err;
	}
	if (count == 0) {
		// in definitely, it's not an error, just add it
		++channel->error_statistics.rx_error_no_frag;
		ret = -EAGAIN;
		goto err;
	}
	if (count > channel->frag_num) {
		ret = -EFAULT;
		printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
		printk_ratelimited(KERN_INFO "%d_%d\n", count, channel->frag_num);
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
			++channel->error_statistics.rx_error_read_frag;
			// do not update any rx index if read fragment occurred
			frame_used_frag_count = 0;
			printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
			channel->hw_trans_error = 1;
			if (channel->higher_level_clear)
				channel->higher_level_clear();
#endif
			break;
		}

		// get fragment_info from fragment header
		fragment_info = (struct frag_info *)(cache_tmp->datacache);

		// handle crc checksum
		if (channel->crc_enable) {
			//pr_info("crc16_1 = %d\ncrc16_2 = %d\n", fragment_info->flag.crc16_1, fragment_info->flag.crc16_2);
			crc16_value_get = fragment_info->flag.crc16_2 << 8 | fragment_info->flag.crc16_1;
			fragment_info->flag.crc16_1 = 0;
			fragment_info->flag.crc16_2 = 0;
			crc16_value_want = crc16(cache_tmp->datacache, channel->frag_len_max);
			if (crc16_value_want != crc16_value_get) {
				++channel->error_statistics.rx_error_crc_check;
				printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
				if (channel->frame_start) {
					if (frame_p) {
						bif_free(frame_p);
						frame_p = NULL;
					}
					channel->frame_start = 0;
				}
				goto next_frag;
			}
		}

		// at start fragment, malloc frame buffer
		if (fragment_info->flag.start == 1) {
			malloc_len = fragment_info->id
				* channel->valid_frag_len_max;
			frame_p =
			bif_malloc(sizeof(struct bif_frame_cache) + malloc_len);
			if (!frame_p) {
				++channel->error_statistics.rx_error_malloc_frame;
				printk_ratelimited(KERN_INFO "bif_err: %s %d\n",
					__func__, __LINE__);
				printk_ratelimited(KERN_INFO "surplus frame: %d\n",
				channel->rx_frame_count);
				break;
			} else
				channel->frame_start = 1;
		} else {
			// not start flag, if frame_start is false
			// continue to next fragment
			if (!channel->frame_start) {
				++channel->error_statistics.rx_error_drop_frag_count;
				goto next_frag;
			}
		}

		cache_tmp->datalen = fragment_info->len;
		ret = bif_rx_reassemble_fragment(channel, frame_p,
			cache_tmp, fragment_info);
		if (ret < 0) {
			++channel->error_statistics.rx_error_assemble_frag;
			printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
			if (channel->frame_start) {
				if (frame_p) {
					bif_free(frame_p);
					frame_p = NULL;
				}
				channel->frame_start = 0;
			}
			goto next_frag;
		}
		// reassemble fragment function return 1,
		// when received a whole frame
		if (ret == 1) {
			frame_p = NULL;
			channel->frame_start = 0;
		}
next_frag:
		frame_used_frag_count = count_tmp + 1;
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

	if (count_tmp < count) {
		// error break
		if (channel->frame_start) {
			if (frame_p) {
				bif_free(frame_p);
				frame_p = NULL;
			}
			channel->frame_start = 0;
		}
	}

	if (frame_used_frag_count) {
		ret = bif_rx_update_after_read(channel,
			frame_used_frag_count);
		if (ret < 0) {
			++channel->error_statistics.rx_error_update_index;
			printk_ratelimited(KERN_INFO "bif_err: %s %d\n", __func__, __LINE__);
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

	spin_lock(&channel->rx_frame_count_lock);
	channel->rx_frame_count = 0;
	spin_unlock(&channel->rx_frame_count_lock);
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
		bif_free(frame);
		spin_lock(&channel->rx_frame_count_lock);
		--channel->rx_frame_count;
		spin_unlock(&channel->rx_frame_count_lock);
	}
	bif_unlock();
}
EXPORT_SYMBOL(bif_del_frame_from_list);

void bif_frame_decrease_count(struct comm_channel *channel)
{
	spin_lock(&channel->rx_frame_count_lock);
	--channel->rx_frame_count;
	spin_unlock(&channel->rx_frame_count_lock);
}
EXPORT_SYMBOL(bif_frame_decrease_count);

void bif_del_frame_from_session_list(struct comm_channel *channel,
struct bif_frame_cache *frame)
{
	spin_lock(&channel->rx_frame_count_lock);
	--channel->rx_frame_count;
	spin_unlock(&channel->rx_frame_count_lock);
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
#if 0
	int i = 0;
	unsigned char *p = NULL;
#endif
	struct bif_rx_ring_info *tx_remote_info_tmp = NULL;
	struct bif_tx_ring_info *tx_local_info_tmp = NULL;
	struct bif_rx_ring_info *rx_local_info_tmp = NULL;
	struct bif_tx_ring_info *rx_remote_info_tmp = NULL;

	tx_remote_info_tmp = bif_malloc(
	ALIGN(sizeof(struct bif_rx_ring_info),
	channel->transfer_align));
	if (!tx_remote_info_tmp) {
		ret = -1;
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}
	ret = bif_read_cp_ddr_channel(channel, tx_remote_info_tmp,
		channel->tx_remote_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
		channel->hw_trans_error = 1;
#endif
		goto err;
	}
#if 0
	p = (unsigned char *)(tx_remote_info_tmp);
	for (i = 0; i < ALIGN(sizeof(struct bif_rx_ring_info),
	channel->transfer_align); ++i) {
		pr_info("%x ", p[i]);
		if (!((i + 1) % 16))
			pr_info("\n");
	}
	pr_info("\n");
	pr_info("sync: tx_remote_info %d\n", tx_remote_info_tmp->recv_head);
#endif
	channel->sync_tx_remote_info = tx_remote_info_tmp->recv_head;

	tx_local_info_tmp = bif_malloc(
	ALIGN(sizeof(struct bif_tx_ring_info),
	channel->transfer_align));
	if (!tx_local_info_tmp) {
		ret = -1;
		bif_err("%s  %d\n", __func__, __LINE__);
		goto err;
	}
	ret = bif_read_cp_ddr_channel(channel, tx_local_info_tmp,
		channel->tx_local_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
		channel->hw_trans_error = 1;
#endif
		goto err;
	}
#if 0
	p = (unsigned char *)(tx_local_info_tmp);
	for (i = 0; i < ALIGN(sizeof(struct bif_tx_ring_info),
	channel->transfer_align); ++i) {
		pr_info("%x ", p[i]);
		if (!((i + 1) % 16))
			pr_info("\n");
	}
	pr_info("\n");
	pr_info("sync: tx_local_info %d\n", tx_local_info_tmp->send_tail);
#endif
	channel->sync_tx_local_info = tx_local_info_tmp->send_tail;

	rx_local_info_tmp = bif_malloc(
	ALIGN(sizeof(struct bif_rx_ring_info),
	channel->transfer_align));
	if (!rx_local_info_tmp) {
		ret = -1;
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}
	ret = bif_read_cp_ddr_channel(channel, rx_local_info_tmp,
		channel->rx_local_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
	channel->hw_trans_error = 1;
#endif
		goto err;
	}
#if 0
	p = (unsigned char *)(rx_local_info_tmp);
	for (i = 0; i < ALIGN(sizeof(struct bif_rx_ring_info),
	channel->transfer_align); ++i) {
		pr_info("%x ", p[i]);
		if (!((i + 1) % 16))
			pr_info("\n");
	}
	pr_info("\n");
	pr_info("sync: rx_local_info %d\n", rx_local_info_tmp->recv_head);
#endif
	channel->sync_rx_local_info = rx_local_info_tmp->recv_head;

	rx_remote_info_tmp = bif_malloc(
	ALIGN(sizeof(struct bif_tx_ring_info),
	channel->transfer_align));
	if (!rx_remote_info_tmp) {
		ret = -1;
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
		goto err;
	}
	ret = bif_read_cp_ddr_channel(channel, rx_remote_info_tmp,
		channel->rx_remote_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
	channel->hw_trans_error = 1;
#endif
		goto err;
	}
#if 0
	p = (unsigned char *)(rx_remote_info_tmp);
	for (i = 0; i < ALIGN(sizeof(struct bif_tx_ring_info),
	channel->transfer_align); ++i) {
		pr_info("%x ", p[i]);
		if (!((i + 1) % 16))
			pr_info("\n");
}
	pr_info("\n");
	pr_info("sync: rx_remote_info %d\n", rx_remote_info_tmp->send_tail);
#endif
	channel->sync_rx_remote_info = rx_remote_info_tmp->send_tail;

	// check whether CP sync
	if ((!channel->channel_ready) || ((tx_remote_info_tmp->recv_head == -1) &&
		(rx_local_info_tmp->recv_head == -1))) {
		pr_info("sync info\n");
		#if 0
		bif_memcpy(channel->tx_remote_info, tx_remote_info_tmp,
			ALIGN(sizeof(struct bif_rx_ring_info),
			channel->transfer_align));
		bif_memcpy(channel->tx_local_info, tx_local_info_tmp,
			ALIGN(sizeof(struct bif_tx_ring_info),
			channel->transfer_align));
		bif_memcpy(channel->rx_local_info, rx_local_info_tmp,
			ALIGN(sizeof(struct bif_rx_ring_info),
			channel->transfer_align));
		bif_memcpy(channel->rx_remote_info, rx_remote_info_tmp,
			ALIGN(sizeof(struct bif_tx_ring_info),
			channel->transfer_align));
		#endif
		memcpy(channel->tx_remote_info, tx_remote_info_tmp,
			ALIGN(sizeof(struct bif_rx_ring_info),
			channel->transfer_align));
		memcpy(channel->tx_local_info, tx_local_info_tmp,
			ALIGN(sizeof(struct bif_tx_ring_info),
			channel->transfer_align));
		memcpy(channel->rx_local_info, rx_local_info_tmp,
			ALIGN(sizeof(struct bif_rx_ring_info),
			channel->transfer_align));
		memcpy(channel->rx_remote_info, rx_remote_info_tmp,
			ALIGN(sizeof(struct bif_tx_ring_info),
			channel->transfer_align));
	}
	channel->channel_ready = 1;
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
#ifdef CONFIG_HOBOT_BIF_AP
		channel->hw_trans_error = 1;
#endif
		goto err;
	}

	ret = bif_read_cp_ddr_channel(channel, tx_remote_info_tmp,
		channel->tx_remote_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
	channel->hw_trans_error = 1;
#endif
		goto err;
	}
	//pr_info("init: tx_remote_info %d\n", tx_remote_info_tmp->recv_head);
	channel->init_tx_remote_info = tx_remote_info_tmp->recv_head;

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
#ifdef CONFIG_HOBOT_BIF_AP
		channel->hw_trans_error = 1;
#endif
		goto err;
	}

	ret = bif_read_cp_ddr_channel(channel, tx_local_info_tmp,
		channel->tx_local_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
	channel->hw_trans_error = 1;
#endif
		goto err;
	}
	//pr_info("init: tx_local_info %d\n", tx_local_info_tmp->send_tail);
	channel->init_tx_local_info = tx_local_info_tmp->send_tail;

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
#ifdef CONFIG_HOBOT_BIF_AP
		channel->hw_trans_error = 1;
#endif
		goto err;
	}

	ret = bif_read_cp_ddr_channel(channel, rx_local_info_tmp,
		channel->rx_local_info_offset,
		ALIGN(sizeof(struct bif_rx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
	channel->hw_trans_error = 1;
#endif
		goto err;
	}
	//pr_info("init: rx_local_info_tmp %d\n", rx_local_info_tmp->recv_head);
	channel->init_rx_local_info = rx_local_info_tmp->recv_head;

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
#ifdef CONFIG_HOBOT_BIF_AP
		channel->hw_trans_error = 1;
#endif
		goto err;
	}

	ret = bif_read_cp_ddr_channel(channel, rx_remote_info_tmp,
		channel->rx_remote_info_offset,
		ALIGN(sizeof(struct bif_tx_ring_info),
		channel->transfer_align));
	if (ret < 0) {
		bif_err("bif_err: %s  %d\n", __func__, __LINE__);
#ifdef CONFIG_HOBOT_BIF_AP
	channel->hw_trans_error = 1;
#endif
		goto err;
	}
	//pr_info("init: rx_remote_info_tmp %d\n", rx_remote_info_tmp->send_tail);
	channel->init_rx_remote_info = rx_remote_info_tmp->send_tail;
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
	#if __SIZEOF_POINTER__ == 4
	pr_info("bif_dma_alloc: vir_addr = %x phy_addr = %x total = %d\n",
	base_addr_tmp, base_addr_tmp_phy, channel->total_mem_size);
	#else
	pr_info("bif_dma_alloc: vir_addr = %lx phy_addr = %lx total = %d\n",
	base_addr_tmp, base_addr_tmp_phy, channel->total_mem_size);
	#endif

	if (base_addr_tmp <= 0) {
		ret = -EFAULT;
		pr_info("bif_alloc_cp fail\n ");
		goto err;
	}
	channel->base_addr_phy = base_addr_tmp_phy;

	// CP set virtual address, register physical address
	bif_set_base_addr(channel, base_addr_tmp);
	ret = bif_register_address(channel->buffer_id,
		(void *)(unsigned long)base_addr_tmp_phy);
	if (ret < 0)
		goto err;
#else
	// AP get and set physical address
	mutex_lock(&channel->channel_sleep_lock);
	if (channel->channel_sleep_flag) {
		pr_info("bif sync base wait\n");
		mutex_unlock(&channel->channel_sleep_lock);
		wait_event_freezable(channel->channel_sleep_wq, !channel->channel_sleep_flag);
		pr_info("bif sync base wake\n");
		mutex_lock(&channel->channel_sleep_lock);
	}
	ret = bif_sync_base();
	mutex_unlock(&channel->channel_sleep_lock);
	if (ret < 0)
		goto err;
	base_addr_tmp_phy = (addr_t)bif_query_address(channel->buffer_id);
	if (base_addr_tmp_phy == -1) {
		ret = -1;
		goto err;

	}
	#if __SIZEOF_POINTER__ == 4
	pr_info("base_addr_tmp_phy %x\n", base_addr_tmp_phy);
	#else
	pr_info("base_addr_tmp_phy %lx\n", base_addr_tmp_phy);
	#endif
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
	pr_debug("==== hardware channel concerned ====\n");
	pr_debug("channel = %d\n", channel->channel);
	pr_debug("buffer_id = %d\n", channel->buffer_id);
	pr_debug("transfer_align = %d\n", channel->transfer_align);
	pr_debug("==== memory limit concerned ====\n");
	#if __SIZEOF_POINTER__ == 4
	pr_debug("base_addr = %x\n", channel->base_addr);
	#else
	pr_debug("base_addr = %lx\n", channel->base_addr);
	#endif
	pr_debug("frame_len_max = %d\n", channel->frame_len_max);
	pr_debug("frag_len_max = %d\n", channel->frag_len_max);
	pr_debug("valid_frag_len_max = %d\n", channel->valid_frag_len_max);
	pr_debug("frag_num = %d\n", channel->frag_num);
	pr_debug("frame_cache_max = %d\n", channel->frame_cache_max);
	pr_debug("==== memory layout concerned ====\n");
	#if __SIZEOF_POINTER__ == 4
	pr_debug("rx_local_info_offset = %x\n",
	channel->rx_local_info_offset);
	pr_debug("rx_remote_info_offset = %x\n",
	channel->rx_remote_info_offset);
	pr_debug("tx_local_info_offset = %x\n",
	channel->tx_local_info_offset);
	pr_debug("tx_remote_info_offset = %x\n",
	channel->tx_remote_info_offset);
	pr_debug("rx_buffer_offset = %x\n",
	channel->rx_buffer_offset);
	pr_debug("tx_buffer_offset = %x\n",
	channel->tx_buffer_offset);
	#else
	pr_debug("rx_local_info_offset = %lx\n",
	channel->rx_local_info_offset);
	pr_debug("rx_remote_info_offset = %lx\n",
	channel->rx_remote_info_offset);
	pr_debug("tx_local_info_offset = %lx\n",
	channel->tx_local_info_offset);
	pr_debug("tx_remote_info_offset = %lx\n",
	channel->tx_remote_info_offset);
	pr_debug("rx_buffer_offset = %lx\n",
	channel->rx_buffer_offset);
	pr_debug("tx_buffer_offset = %lx\n",
	channel->tx_buffer_offset);
	#endif
	pr_debug("total_mem_size = %d\n",
	channel->total_mem_size);
	pr_debug("ap_type = %d\n", channel->type);
	pr_debug("working_mode = %d\n", channel->mode);
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
	} else if (channel->channel == ETHERNET)
		channel->transfer_align = 1;

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
#ifdef CONFIG_HOBOT_BIF_AP
	channel->rx_local_info_tmp_buf =
	bif_malloc(ALIGN(sizeof(struct bif_rx_ring_info),
	RING_INFO_ALIGN));
	if (!channel->rx_local_info_tmp_buf) {
		bif_err("bif_err: %s %d\n", __func__, __LINE__);
		goto malloc_rx_local_tmp_buf_fail;
	}
#endif
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
	spin_lock_init(&channel->rx_frame_count_lock);
	channel->type = config->type;
	channel->mode = config->mode;
	channel->crc_enable = config->crc_enable;
	mutex_init(&channel->channel_sleep_lock);
	//init_completion(&channel->channel_sleep_complete);
	init_waitqueue_head(&channel->channel_sleep_wq);
	dump_channel_info(channel);

	return 0;
malloc_send_frag_fail:
	bif_free(channel->recv_frag);
malloc_recv_frag_fail:
	bif_free(channel->rx_frame_cache_p);
malloc_frame_cache_fail:
#ifdef CONFIG_HOBOT_BIF_AP
	bif_free(channel->rx_local_info_tmp_buf);
#else
	bif_free(channel->rx_local_info);
#endif
#ifdef CONFIG_HOBOT_BIF_AP
malloc_rx_local_tmp_buf_fail:
	bif_free(channel->rx_local_info);
#endif
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
#ifdef CONFIG_HOBOT_BIF_AP
	bif_free(channel->rx_local_info_tmp_buf);
#endif
	mutex_destroy(&channel->channel_sleep_lock);
}
EXPORT_SYMBOL(channel_deinit);

int channel_stock_frame_num(struct comm_channel *channel)
{
	int count_ret = 0;

	spin_lock(&channel->rx_frame_count_lock);
	count_ret = channel->rx_frame_count;
	spin_unlock(&channel->rx_frame_count_lock);

	return count_ret;
}
EXPORT_SYMBOL(channel_stock_frame_num);

int channel_register_high_level_clear(struct comm_channel *channel,
clear_func_t clear_func)
{
	channel->higher_level_clear = clear_func;

	return 0;
}
EXPORT_SYMBOL(channel_register_high_level_clear);

void channel_unregister_high_level_clear(struct comm_channel *channel)
{
	channel->higher_level_clear = NULL;
}
EXPORT_SYMBOL(channel_unregister_high_level_clear);
