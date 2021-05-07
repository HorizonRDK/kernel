/* Copyright (C) 2020 - 2021  Horizon */
// SPDX-License-Identifier: GPL-2.0
/*Utility Bulk channel transfer*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/sched/signal.h>

#include "u_hbusb.h"//NOLINT


#define HBUSB_DEV_DEBUG

#define FRAME_CTRL_START_MAGIC	0xA5626474  /*a5bct*/

#define HBUSB_SYNC_RX_VALID_FRAMEBUF	_IO('B', 0x01)
#define HBUSB_SYNC_TX_VALID_FRAMEBUF	_IO('B', 0x02)
#define HBUSB_DEV_PRESENT	_IO('B', 0x03)


const char hbusb_class_name[] = {"hbusb"};
static struct hbusb_dev *global_hbusb_dev;

#define HBUSB_TX_MAX_PACKET 1024

/*---------------------function declare--------------------------*/
static void lowlevel_rx_complete(struct usb_ep *ep,
						struct usb_request *req);
static int lowlevel_rx_submit(struct hbusb_rx_chan *rx);
static int lowlevel_tx_submit(struct hbusb_tx_chan *tx);
static void lowlevel_tx_complete(struct usb_ep *ep,
						struct usb_request *req);
static void tx_frame_info_var_init(struct hbusb_tx_chan *tx);
static void tx_frame_ctrl_var_init(struct hbusb_tx_chan *tx);
static void rx_frame_info_var_init(struct hbusb_rx_chan *rx);
static void rx_frame_ctrl_var_init(struct hbusb_rx_chan *rx);
/*------------------------------------------------------------*/

/*------------------------lowlevel read--------------------------*/
void rx_ctrl_head_event(struct hbusb_rx_chan *rx)
{
	int tmp_size;
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;
	struct hbusb_frame_head *head =
		(struct hbusb_frame_head *)frame_ctrl->rx_ctrl_head_buf;
	struct hbusb_channel *chan = rx->parent_chan;

	if (head->start_end_magic_num != FRAME_CTRL_START_MAGIC)
		/*if start magic is not right, we direct return
		 * we do not update and wait for head info*/
		return;

	if (head->sgs_num > frame_info->sg_num) {
		/*warnning is not a valid data, */
		dev_err(chan->dev, "recv sgs num is more than user want[%d:%d]\n",
				head->sgs_num, frame_info->sg_num);
		frame_info->error_val	= -HBUSB_SG_NUM_NOT_EQUAL;
		frame_ctrl->hbusb_recv_event = EVENT_HBUSB_RX_FINISHED;
		return;
	} else {
		 frame_info->sg_num = head->sgs_num;
	}

	if (head->user_ctrl_info_len != 0) {
		tmp_size = (head->user_ctrl_info_len < USER_CTRL_INFO_MAX_SIZE) ?
				head->user_ctrl_info_len : USER_CTRL_INFO_MAX_SIZE;
		frame_ctrl->rx_ctrl_head_actual_size = tmp_size;
	}

	frame_ctrl->recv_slice_phys = frame_info->sg_info[0].slice_addr[0];
	frame_ctrl->recv_req_size = frame_info->sg_info[0].slice_size[0];
	frame_ctrl->curr_sg_index_per_frame_info = 0;
	frame_ctrl->curr_slice_index_per_sg = 0;
	frame_ctrl->hbusb_recv_event = EVENT_HBUSB_RX_USER_DATA;
}

void rx_user_data_event(struct hbusb_rx_chan *rx)
{
#if 0
	struct hbusb_frame_head *head =  NULL;
#endif
	struct frame_rx_ctrl	*frame_ctrl = &rx->frame_ctrl;
	struct frame_info		*frame_info = frame_ctrl->frame_info;
	struct frame_sg_info *sg_info	= NULL;
	int sg_index = 0, slice_index = 0;

	/* judge recv buf is usb head, if it is head,
	 * may be some wrong, need to wait next data
	 */
	if (frame_ctrl->recv_slice_size == HBUSB_CTRL_FRAME_SIZE) {
		/*0. err handle when recevie head in data buf */
#if 0
		head = (struct hbusb_frame_head *)frame_ctrl->recv_slice_buf;
		if (head->start_end_magic_num == FRAME_CTRL_START_MAGIC) {
			memcpy(frame_ctrl->rx_ctrl_head_buf,
				(void *)frame_ctrl->recv_slice_addr, HBUSB_CTRL_FRAME_SIZE);
			frame_ctrl->recv_slice_size = HBUSB_CTRL_FRAME_SIZE;
			frame_ctrl->hbusb_recv_event = EVENT_HBUSB_RX_HEAD_IN_DATABUF;
			frame_info->error_val = -HBUSB_RECV_NOT_COMPLETE;
			return;
		}
#else
		frame_ctrl->recv_slice_size = HBUSB_CTRL_FRAME_SIZE;
		frame_ctrl->hbusb_recv_event = EVENT_HBUSB_RX_HEAD_IN_DATABUF;
		frame_info->error_val = -HBUSB_RECV_NOT_COMPLETE;
		return;
#endif
	}

	/*we first handle received slice num info*/
	sg_index = frame_ctrl->curr_sg_index_per_frame_info;
	slice_index = frame_ctrl->curr_slice_index_per_sg;
	sg_info = &frame_info->sg_info[sg_index];
	sg_info->slice_actual_size[slice_index] = frame_ctrl->recv_slice_size;
	if (frame_ctrl->recv_slice_size < HBUSB_DMA_BUF_MAX_SIZE) {
		if ((frame_ctrl->recv_slice_size == 0)
			&& (frame_ctrl->curr_slice_index_per_sg == 0)) {
				/*1. this mean a not vaild info in first slice of sg info,
					use the same slice buf to recv next send data */
				return;
		}
		/* 2.This mean the last slice of per sg_info,
				we need point to next sg*/
		goto next_sg;
	} else {
		/*3.recv valid data in current slice goto next slice*/
		goto next_slice_of_sg;
	}

next_slice_of_sg:
	frame_ctrl->curr_slice_index_per_sg++;
	if (frame_ctrl->curr_slice_index_per_sg >= sg_info->slice_num) {
		/* 4.we have no more slice per sg, we need point to next sg*/
		goto next_sg;
	}

	sg_index = frame_ctrl->curr_sg_index_per_frame_info;
	slice_index = frame_ctrl->curr_slice_index_per_sg;
	sg_info = &frame_info->sg_info[sg_index];
	frame_ctrl->recv_slice_phys = sg_info->slice_addr[slice_index];
	frame_ctrl->recv_req_size = sg_info->slice_size[slice_index];
	return;
next_sg:
	frame_info->sg_actual_num++;
	frame_ctrl->curr_sg_index_per_frame_info++;
	frame_ctrl->curr_slice_index_per_sg = 0;
	if (frame_ctrl->curr_sg_index_per_frame_info >= frame_info->sg_num) {
		/*5.we have fill full user sg num, return finish*/
		goto recv_finished;
	}

	sg_index = frame_ctrl->curr_sg_index_per_frame_info;
	slice_index = frame_ctrl->curr_slice_index_per_sg;
	sg_info = &frame_info->sg_info[sg_index];
	frame_ctrl->recv_slice_phys = sg_info->slice_addr[slice_index];
	frame_ctrl->recv_req_size = sg_info->slice_size[slice_index];
	return;
recv_finished:
	frame_ctrl->hbusb_recv_event = EVENT_HBUSB_RX_FINISHED;
	return;
}

static int hbusb_rx_event(struct hbusb_rx_chan *rx)
{
	int status = 0;
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;

	/*recv data and preapre to recv next buf by last event*/
	switch (frame_ctrl->hbusb_recv_event) {
	case EVENT_HBUSB_RX_CTRL_HEAD:
		rx_ctrl_head_event(rx);
		break;
	case EVENT_HBUSB_RX_USER_DATA:
		rx_user_data_event(rx);
		break;
	default:
		status = -ENODATA;
		break;
	}

	if ((frame_ctrl->hbusb_recv_event == EVENT_HBUSB_TX_FINISHED)
		||(frame_ctrl->hbusb_recv_event == EVENT_HBUSB_RX_HEAD_IN_DATABUF)) {
		/* if we complete send all user data
		 * wakeup user write function complete
		 */
		rx->cond = FLAG_USER_RX_NOT_WAIT;
		wake_up_interruptible(&rx->wait);
	}

	return status;
}

static void lowlevel_rx_complete(struct usb_ep *ep,
						struct usb_request *req)
{
	int status;
	unsigned long		flags;
	struct hbusb_rx_chan		*rx = req->context;
	struct frame_rx_ctrl	*frame_ctrl = &rx->frame_ctrl;

	if (req == NULL)
		return;

	spin_lock_irqsave(&rx->ep_lock, flags);
	if ((req != NULL) && (rx->req !=req)) {
		spin_unlock_irqrestore(&rx->ep_lock, flags);
		usb_ep_free_request(ep, req);
		return;
	}

	frame_ctrl->recv_slice_size = req->actual;
	usb_ep_free_request(ep, req);
	rx->req = NULL;

	status = hbusb_rx_event(rx);
	if (status
		|| (frame_ctrl->hbusb_recv_event == EVENT_HBUSB_TX_FINISHED)
		||(frame_ctrl->hbusb_recv_event == EVENT_HBUSB_RX_HEAD_IN_DATABUF)) {
		/*when tx finished, */
		spin_unlock_irqrestore(&rx->ep_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&rx->ep_lock, flags);

	do {
		status = lowlevel_rx_submit(rx);
	} while ((status != -EIOCBQUEUED)
			&& (status != -ENODEV));
}

void hbusb_prepare_rx_req_buf(struct hbusb_rx_chan *rx)
{
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;
	struct usb_request	*req = rx->req;

	switch (frame_ctrl->hbusb_recv_event) {
	case EVENT_HBUSB_RX_CTRL_HEAD:
		req->buf = frame_ctrl->rx_ctrl_head_buf;
		req->length = HBUSB_CTRL_FRAME_SIZE;
		req->complete = lowlevel_rx_complete;
		req->context = rx;
		req->hb_direct_dma = 0;
		break;
	case EVENT_HBUSB_RX_USER_DATA:
		req->dma = frame_ctrl->recv_slice_phys;
		req->length = frame_ctrl->recv_req_size;
		req->complete = lowlevel_rx_complete;
		req->context = rx;
		req->hb_direct_dma = 1;
		req->dma_mapped = 1;
		break;
	}
}
static int lowlevel_rx_submit(struct hbusb_rx_chan *rx)
{
	int					status = 0;
	unsigned long		flags;
	struct usb_ep		*ep;
	struct usb_request	*req;
	struct hbusb_channel	*chan = rx->parent_chan;

	if (atomic_read(&chan->chan_running)
			== HBUSB_CHANNEL_NOT_RUNNING)
		return -ENODEV;

	spin_lock_irqsave(&rx->ep_lock, flags);
	ep = rx->ep;
	req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (req == NULL) {
		status = -ENOMEM;
		goto fail0;
	}
	rx->req = req;

	/*prepare usb request member*/
	hbusb_prepare_rx_req_buf(rx);

	status = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (unlikely(status != 0)) {
		dev_err(chan->dev, "Failed to queue usb endpoint\n");
		goto fail1;
	}
	spin_unlock_irqrestore(&rx->ep_lock, flags);

	return -EIOCBQUEUED;
fail1:
	usb_ep_free_request(ep, req);
fail0:
	spin_unlock_irqrestore(&rx->ep_lock, flags);
	return status;
}
/*------------------------------------------------------------*/

/*------------------------lowlevel write--------------------------*/
void tx_complete_ctrl_head_event(struct hbusb_tx_chan *tx)
{
	struct usb_request	*req = tx->req;
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;

	if (req->actual)
		frame_ctrl->tx_ctrl_head_actual_size
					= frame_info->elem.ctrl_info.size;
	frame_ctrl->hbusb_send_event = EVENT_HBUSB_TX_USER_DATA;
	frame_ctrl->curr_sg_index_per_frame_info = 0;
	frame_ctrl->curr_slice_index_per_sg = 0;
}

void tx_complete_user_data_event(struct hbusb_tx_chan *tx)
{
	int sg_index = 0, slice_index = 0;
	struct usb_request	*req = tx->req;
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;
	struct frame_sg_info *sg_info	= NULL;

	/*save send length*/
	frame_ctrl->hbusb_send_event = EVENT_HBUSB_TX_USER_DATA;
	sg_index = frame_ctrl->curr_sg_index_per_frame_info;
	slice_index = frame_ctrl->curr_slice_index_per_sg;
	sg_info = &frame_info->sg_info[sg_index];
	sg_info->slice_actual_size[slice_index] = req->actual;

	/*prepare next send slice buf and size*/
/*next_slice_of_sg:*/
	frame_ctrl->curr_slice_index_per_sg++;
	if (frame_ctrl->curr_slice_index_per_sg >= sg_info->slice_num) {
		goto next_sg;
	}
	return;
next_sg:
	frame_ctrl->curr_slice_index_per_sg = 0;
	frame_ctrl->curr_sg_index_per_frame_info++;
	if (frame_ctrl->curr_sg_index_per_frame_info
						>= frame_info->sg_num) {
		goto tx_finish;
	}
	return;
tx_finish:
	frame_ctrl->curr_sg_index_per_frame_info = 0;
	frame_ctrl->hbusb_send_event = EVENT_HBUSB_TX_FINISHED;
	return;
}

static void hbusb_complete_tx_event(struct hbusb_tx_chan *tx)
{
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;

	switch (frame_ctrl->hbusb_send_event) {
	case EVENT_HBUSB_TX_CTRL_HEAD:
		tx_complete_ctrl_head_event(tx);
		break;
	case EVENT_HBUSB_TX_USER_DATA:
		tx_complete_user_data_event(tx);
		break;
	default:
		frame_ctrl->hbusb_send_event = EVENT_HBUSB_TX_FINISHED;
		break;
	}

	if (frame_ctrl->hbusb_send_event == EVENT_HBUSB_TX_FINISHED) {
		/* if we complete send all user data
		 * wakeup user write function complete
		 */
		tx->cond = FLAG_USER_TX_NOT_WAIT;
		wake_up_interruptible(&tx->wait);
	}
}

void tx_start_ctrl_head_event(struct hbusb_tx_chan *tx)
{
	struct usb_request	*req = tx->req;
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;

	req->buf = frame_ctrl->tx_ctrl_head_buf;
	req->length = HBUSB_CTRL_FRAME_SIZE;
	req->complete = lowlevel_tx_complete;
	req->context = tx;
	req->zero = 1;
	req->hb_direct_dma = 0;
}

void tx_start_user_data_event(struct hbusb_tx_chan *tx)
{
	int sg_index = 0, slice_index = 0;
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;
	struct usb_request	*req = tx->req;
	struct frame_sg_info *sg_info	= NULL;

	/*Make sure in this funciton,current sg_index and silce index is valid*/
	sg_index = frame_ctrl->curr_sg_index_per_frame_info;
	slice_index = frame_ctrl->curr_slice_index_per_sg;
	sg_info = &frame_info->sg_info[sg_index];
	frame_ctrl->send_slice_phys = sg_info->slice_addr[slice_index];
	frame_ctrl->send_slice_size = sg_info->slice_size[slice_index];

	/*preapre req buf*/
	req->dma = frame_ctrl->send_slice_phys;
	req->length = frame_ctrl->send_slice_size;
	req->complete = lowlevel_tx_complete;
	req->context = tx;
	req->hb_direct_dma = 1;
	req->dma_mapped = 1;
}

static int hbusb_start_tx_event(struct hbusb_tx_chan *tx)
{
	int status = 0;
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;

	switch (frame_ctrl->hbusb_send_event) {
	case EVENT_HBUSB_TX_CTRL_HEAD:
		tx_start_ctrl_head_event(tx);
		break;
	case EVENT_HBUSB_TX_USER_DATA:
		tx_start_user_data_event(tx);
		break;
	default:
		status = -ENODATA;
		break;
	}

	return status;
}

static void lowlevel_tx_complete(struct usb_ep *ep,
						struct usb_request *req)
{
	int status;
	unsigned long		flags;
	struct hbusb_tx_chan		*tx = req->context;
	struct frame_tx_ctrl	*frame_ctrl = &tx->frame_ctrl;

	if (req == NULL)
		return;

	spin_lock_irqsave(&tx->ep_lock, flags);
	if ((req != NULL) && (tx->req !=req)) {
		spin_unlock_irqrestore(&tx->ep_lock, flags);
		usb_ep_free_request(ep, req);
		return;
	}

	hbusb_complete_tx_event(tx);
	usb_ep_free_request(ep, req);
	tx->req = NULL;

	if (frame_ctrl->hbusb_send_event == EVENT_HBUSB_TX_FINISHED) {
		/*when tx finished, */
		spin_unlock_irqrestore(&tx->ep_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&tx->ep_lock, flags);

	do {
		status = lowlevel_tx_submit(tx);
	} while ((status != -EIOCBQUEUED)
			&& (status != -ENODEV));
}

static int lowlevel_tx_submit(struct hbusb_tx_chan *tx)
{
	int					status = 0;
	unsigned long		flags;
	struct usb_ep		*ep;
	struct usb_request	*req;
	struct hbusb_channel	*chan = tx->parent_chan;

	if (atomic_read(&chan->chan_running)
			== HBUSB_CHANNEL_NOT_RUNNING)
		return -ENODEV;

	spin_lock_irqsave(&tx->ep_lock, flags);
	ep = tx->ep;
	req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (req == NULL) {
		status = -ENOMEM;
		goto fail0;
	}
	tx->req = req;

	status = hbusb_start_tx_event(tx);
	if (status == -ENODATA) {
		goto fail1;
	}

	status = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (unlikely(status != 0)) {
		dev_err(chan->dev, "Failed to queue usb endpoint\n");
		goto fail1;
	}
	spin_unlock_irqrestore(&tx->ep_lock, flags);

	return -EIOCBQUEUED;
fail1:
	usb_ep_free_request(ep, req);
fail0:
	spin_unlock_irqrestore(&tx->ep_lock, flags);
	return status;
}
/*-------------------------------------------------------------*/

/*--------------------hbusb file operations-------------------------*/
static void tx_frame_ctrl_var_init(struct hbusb_tx_chan *tx);

/*this function can be call by close or hbusb_disconnect*/
void release_usb_request(struct hbusb_channel *chan,
					spinlock_t		  *ep_lock,
					struct usb_ep **ep,
					struct usb_request **req)
{
	unsigned long		flags;
	struct usb_ep		*temp_ep = NULL;
	struct usb_request	*temp_req = NULL;

	if (ep_lock == NULL)
		return;

	spin_lock_irqsave(ep_lock, flags);
	if ((*req) != NULL && (*ep) != NULL) {
		temp_req = *req;
		temp_ep = *ep;
		/*we set channel rx/tx->req is NULL,
		 *complete will return by flag
		 */
		*req = NULL;
	} else {
		spin_unlock_irqrestore(ep_lock, flags);
		return;
	}
	spin_unlock_irqrestore(ep_lock, flags);

	if (atomic_read(&chan->chan_running) != HBUSB_CHANNEL_NOT_RUNNING)
		/*usb_ep_dequeue when usb plug out will block*/
		usb_ep_dequeue(temp_ep, temp_req);
}

static void hbusb_release_work_func(struct work_struct *work)
{
	struct hbusb_channel *chan = container_of(work, struct hbusb_channel,
						close_work.work);
	struct hbusb_rx_chan *rx = &chan->rx;
	struct hbusb_tx_chan *tx = &chan->tx;

	if (chan->rx_config_en) {
		/*stop rx channel req*/
		release_usb_request(chan, &rx->ep_lock, &rx->ep, &rx->req);
	}

	if (chan->tx_config_en) {
		/*stop tx channel req*/
		release_usb_request(chan, &tx->ep_lock, &tx->ep, &tx->req);
	}

	atomic_set(&chan->chan_running, HBUSB_CHANNEL_NOT_RUNNING);
	atomic_set(&chan->chan_opened, HBUSB_CHANNEL_CLOSED);
}

void hbusb_chan_open_init(struct hbusb_channel *chan)
{
	struct hbusb_rx_chan *rx = &chan->rx;
	struct hbusb_tx_chan *tx = &chan->tx;

	if (chan->rx_config_en) {
		/*init rx channel*/
		rx->cond = FLAG_USER_RX_NOT_WAIT;
		init_waitqueue_head(&rx->wait);
		mutex_init(&chan->read_lock);
	}

	if (chan->tx_config_en) {
		/*init tx channel*/
		tx->cond = FLAG_USER_TX_NOT_WAIT;
		init_waitqueue_head(&tx->wait);
		mutex_init(&chan->write_lock);
	}
}

int hbusb_chan_start_transfer(struct hbusb_channel *chan)
{
	struct hbusb_rx_chan *rx = &chan->rx;
	struct hbusb_tx_chan *tx = &chan->tx;

	if (chan->rx_config_en) {
		/*init and submit rx channel*/
		rx_frame_ctrl_var_init(rx);
		rx_frame_info_var_init(rx);
	}

	if (chan->tx_config_en) {
		/*init tx channel*/
		tx_frame_ctrl_var_init(tx);
		tx_frame_info_var_init(tx);
	}

	return 0;
}

static int hbusb_open(struct inode *inode, struct file *filp)
{
	struct hbusb_dev *hbusb_dev;
	int minor = iminor(inode);
	struct hbusb_channel *chan;

	hbusb_dev = global_hbusb_dev;
	chan = &hbusb_dev->chan[minor];

	if (atomic_read(&chan->chan_opened) == HBUSB_CHANNEL_OPENED) {
		/*close <--> open too fast, last close time not execte timely*/
		msleep(2 * WRITE_FOR_CALLBACK_COMPLETE);
		if (atomic_read(&chan->chan_opened) == HBUSB_CHANNEL_OPENED) {
			pr_err("Failed to repeate to open bct channel\n");
			return -EBUSY;
		}
	}

	filp->private_data = chan;
	hbusb_chan_open_init(chan);

	atomic_set(&chan->chan_opened, HBUSB_CHANNEL_OPENED);
	if (atomic_read(&hbusb_dev->usb_plugin) == HBUSB_DEV_PLUGIN) {
		atomic_set(&chan->chan_running, HBUSB_CHANNEL_RUNNING);
		(void)hbusb_chan_start_transfer(chan);
	}

	return 0;
}

static int hbusb_release(struct inode *inode, struct file *filp)
{
	struct hbusb_channel		*chan = filp->private_data;

	if (chan == NULL) {
		pr_err("Failed to close hbusb, chan not been open\n");
		return -ENODEV;
	}

	/*delay some close operation to delay*/
	schedule_delayed_work(&chan->close_work,
			WRITE_FOR_CALLBACK_COMPLETE * HZ / 1000);

	filp->private_data = NULL;
	return 0;
}

/*---------------------hbusb rx operations-------------------------*/
int parse_rx_param_to_frame_info(struct hbusb_rx_chan	*rx,
									void __user *user_elem)
{
	int i, j, slice_count, tmp_length;
	unsigned long		flags;
	struct hbusb_channel	*chan = rx->parent_chan;
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;
	elem_t *elem = &frame_info->elem;
	struct frame_sg_info *sg_info = NULL;
	scatterlist_t *user_sg = NULL;

	if (copy_from_user(elem, user_elem, sizeof(elem_t))) {
		dev_err(chan->dev, "Failed to copy elem from user param\n");
		return -EFAULT;
	}

	if ((elem->ctrl_info.size == 0 && elem->data_info.num_sgs == 0)
			|| (elem->data_info.num_sgs > HBUSB_SG_MAX_NUM)) {
		dev_err(chan->dev, "rx elem length is not valid\n");
		return -EFAULT;
	}

	/*handle data info*/
	frame_info->sg_num = elem->data_info.num_sgs;
	for (i = 0; i < frame_info->sg_num; i++) {
		user_sg = &elem->data_info.sg[i];

		sg_info = &frame_info->sg_info[i];
		if (user_sg->length > HBUSB_MAX_SIZE_PER_SG) {
			dev_err(chan->dev, "sg[%d] length is too big\n", user_sg->length);
			return -EFAULT;
		}

		slice_count = user_sg->length / HBUSB_DMA_BUF_MAX_SIZE;
		slice_count += (user_sg->length % HBUSB_DMA_BUF_MAX_SIZE) ? 1 : 0;
		tmp_length = user_sg->length;
		sg_info->slice_num = slice_count;
		for (j = 0; j < slice_count; j++) {
			sg_info->slice_addr[j] =
				user_sg->phy_addr + (j * HBUSB_DMA_BUF_MAX_SIZE);
			sg_info->slice_size[j] = (tmp_length > HBUSB_DMA_BUF_MAX_SIZE)
						? HBUSB_DMA_BUF_MAX_SIZE : tmp_length;
			tmp_length -= sg_info->slice_size[j];
			sg_info->slice_actual_size[j] = 0;
		}
	}

	spin_lock_irqsave(&rx->ep_lock, flags);
	frame_info->sg_actual_num = 0;
	frame_info->error_val = 0;
	frame_ctrl->hbusb_recv_event = EVENT_HBUSB_RX_CTRL_HEAD;
	spin_unlock_irqrestore(&rx->ep_lock, flags);

	return 0;
}

void prepare_to_rx_head_info(struct hbusb_rx_chan *rx)
{
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;

	if (frame_ctrl->hbusb_recv_event
		== EVENT_HBUSB_RX_HEAD_IN_DATABUF) {
		rx_ctrl_head_event(rx);
	} else {
		frame_ctrl->recv_req_size = HBUSB_DMA_BUF_MAX_SIZE;
		frame_ctrl->hbusb_recv_event = EVENT_HBUSB_RX_CTRL_HEAD;
	}
}

int ret_frame_info_to_rx_param(struct hbusb_rx_chan	*rx,
								void __user *user_elem)
{
	int i, j = 0;
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;
	struct hbusb_channel *chan = rx->parent_chan;
	elem_t *elem = &frame_info->elem;
	struct frame_sg_info *sg_info = NULL;
	scatterlist_t *user_sg = NULL;

	if (elem->ctrl_info.size != 0) {
		elem->ctrl_info.actual_size = frame_ctrl->rx_ctrl_head_actual_size;
		if (copy_to_user((void __user *)elem->ctrl_info.buf,
			frame_ctrl->rx_ctrl_head_buf + USB_FRAME_HEAD_SIZE,
			elem->ctrl_info.actual_size)) {
			dev_err(chan->dev, "Failed to copy rx elem ctrl info to user param\n");
		}
	}

	for (i = 0; i < elem->data_info.num_sgs; i++) {
		sg_info = &frame_info->sg_info[i];
		user_sg = &elem->data_info.sg[i];
		for (j = 0, user_sg->actual_length = 0; j < sg_info->slice_num; j++)
			user_sg->actual_length += sg_info->slice_actual_size[j];
	}

	if (copy_to_user(user_elem, elem, sizeof(elem_t))) {
		dev_err(chan->dev, "Failed to copy rx  elem data info to user param\n");
		return -EFAULT;
	}

	return 0;
}

static int hbusb_sync_rx_valid_framebuf(struct file *filp,
								void __user *user_elem)
{
	int status;
	struct hbusb_channel	*chan = filp->private_data;
	struct hbusb_rx_chan	*rx = &chan->rx;

	/*parse user param to struct frame info*/
	status = parse_rx_param_to_frame_info(rx, user_elem);
	if (status < 0) {
		dev_err(chan->dev, "Failed to parse user rx param\n");
		return -EFAULT;
	}
	prepare_to_rx_head_info(rx);

	/*init rx start transfer state and submit request*/
	rx->cond = FLAG_USER_RX_WAIT;
	status = lowlevel_rx_submit(rx);
	if (status != -EIOCBQUEUED) {
		rx->cond = FLAG_USER_RX_NOT_WAIT;
		return status;
	}

	/*wait for request complete*/
	if (rx->cond == FLAG_USER_RX_WAIT) {
		status = wait_event_interruptible(rx->wait, rx->cond);
		if (status == -ERESTARTSYS) {
			/*if have kill signal, release_usb_request
			 *will be called in dev release function
			 */
			dev_info(chan->dev, "wait read data signal interrupted\n");
			return -EINTR;
		} else if (rx->cond != FLAG_USER_RX_NOT_WAIT) {
			release_usb_request(chan, &rx->ep_lock,
				&rx->ep, &rx->req);
			return -ENODEV;
		}
	}

	status = ret_frame_info_to_rx_param(rx, user_elem);
	if (status < 0) {
		dev_err(chan->dev, "Failed to return rxparam to user\n");
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------*/

/*-----------------------hbusb tx operations---------------------------*/
void wrap_hbusb_ctrl_frame_info(struct hbusb_tx_chan *tx)
{
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;
	struct hbusb_frame_head *head = frame_ctrl->tx_ctrl_head_buf;

	frame_ctrl->seq_num++;
	if (frame_ctrl->seq_num == 0) {
		frame_ctrl->seq_num = 1;
	}

	head->start_end_magic_num = FRAME_CTRL_START_MAGIC;
	head->user_ctrl_info_len = frame_info->elem.ctrl_info.size;
	head->sgs_num = frame_info->sg_num;
	head->sequene_num = frame_ctrl->seq_num;
}

int parse_tx_param_to_frame_info(struct hbusb_tx_chan	*tx,
									void __user *user_elem)
{
	int i, j, slice_count, tmp_length;
	unsigned long		flags;
	struct hbusb_channel	*chan = tx->parent_chan;
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;
	elem_t *elem = &frame_info->elem;
	struct frame_sg_info *sg_info = NULL;
	scatterlist_t *user_sg = NULL;

	if (copy_from_user(elem, user_elem, sizeof(elem_t))) {
		dev_err(chan->dev, "Failed to copy elem from user param\n");
		return -EFAULT;
	}

	if ((elem->ctrl_info.size == 0 && elem->data_info.num_sgs == 0)
			|| (elem->ctrl_info.size > USER_CTRL_INFO_MAX_SIZE)
			|| (elem->data_info.num_sgs > HBUSB_SG_MAX_NUM)) {
		dev_err(chan->dev, "tx elem length is not valid\n");
		return -EFAULT;
	}

	/*handle ctrl info*/
	if (elem->ctrl_info.size != 0) {
		if (copy_from_user(frame_ctrl->tx_ctrl_head_buf + USB_FRAME_HEAD_SIZE,
			(void __user *)elem->ctrl_info.buf, elem->ctrl_info.size)) {
			dev_err(chan->dev, "sgs num is not valid\n");
			return -EFAULT;
		}
	}

	/*handle data info*/
	frame_info->sg_num = elem->data_info.num_sgs;
	for (i = 0; i < frame_info->sg_num; i++) {
		user_sg = &elem->data_info.sg[i];
		sg_info = &frame_info->sg_info[i];
		if (user_sg->length > HBUSB_MAX_SIZE_PER_SG) {
			dev_err(chan->dev, "sg[%d] length is too big\n", user_sg->length);
			return -EFAULT;
		}

		slice_count = user_sg->length / HBUSB_DMA_BUF_MAX_SIZE;
		slice_count += (user_sg->length % HBUSB_DMA_BUF_MAX_SIZE) ? 1 : 0;
		tmp_length = user_sg->length;
		sg_info->slice_num = slice_count;
		for (j = 0; j < slice_count; j++) {
			sg_info->slice_addr[j] =
				user_sg->phy_addr + j * HBUSB_DMA_BUF_MAX_SIZE;
			sg_info->slice_size[j] = (tmp_length > HBUSB_DMA_BUF_MAX_SIZE)
						? HBUSB_DMA_BUF_MAX_SIZE : tmp_length;
			tmp_length -= sg_info->slice_size[j];
			sg_info->slice_actual_size[j] = 0;
		}

		if ((user_sg->length % HBUSB_TX_MAX_PACKET) == 0) {
			/*if the length can not offer short packet, we add ZLP slice*/
			sg_info->slice_addr[j] = 0;
			sg_info->slice_size[j] = 0;
			sg_info->slice_num++;
		}
	}

	spin_lock_irqsave(&tx->ep_lock, flags);
	frame_info->sg_actual_num = 0;
	frame_info->error_val = 0;
	frame_ctrl->hbusb_send_event = EVENT_HBUSB_TX_CTRL_HEAD;
	spin_unlock_irqrestore(&tx->ep_lock, flags);

	return 0;
}

int ret_frame_info_to_tx_param(struct hbusb_tx_chan	*tx,
								void __user *user_elem)
{
	int i, j = 0;
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;
	struct hbusb_channel *chan = tx->parent_chan;
	elem_t *elem = &frame_info->elem;
	struct frame_sg_info *sg_info = NULL;
	scatterlist_t *user_sg = NULL;

	if (elem->ctrl_info.size != 0) {
		elem->ctrl_info.actual_size = frame_ctrl->tx_ctrl_head_actual_size;
	}

	for (i = 0; i < elem->data_info.num_sgs; i++) {
		sg_info = &frame_info->sg_info[i];
		user_sg = &elem->data_info.sg[i];
		for (j = 0, user_sg->actual_length = 0; j < sg_info->slice_num; j++)
			user_sg->actual_length += sg_info->slice_actual_size[j];
	}

	if (copy_to_user(user_elem, elem, sizeof(elem_t))) {
		dev_err(chan->dev, "Failed to copy result elem to user param\n");
		return -EFAULT;
	}

	return 0;
}

static int hbusb_sync_tx_valid_framebuf(struct file *filp,
								void __user *user_elem)
{
	int status;
	struct hbusb_channel	*chan = filp->private_data;
	struct hbusb_tx_chan	*tx = &chan->tx;

	/*parse user param to struct frame info*/
	status = parse_tx_param_to_frame_info(tx, user_elem);
	if (status < 0) {
		dev_err(chan->dev, "Failed to parse user param\n");
		return -EFAULT;
	}
	wrap_hbusb_ctrl_frame_info(tx);

	/*init tx start transfer state and submit request*/
	tx->cond = FLAG_USER_TX_WAIT;
	status = lowlevel_tx_submit(tx);
	if (status != -EIOCBQUEUED) {
		tx->cond = FLAG_USER_TX_NOT_WAIT;
		return status;
	}

	/*wait for request complete*/
	if (tx->cond == FLAG_USER_TX_WAIT) {
		status = wait_event_interruptible(tx->wait, tx->cond);
		if (status == -ERESTARTSYS) {
			/*if have kill signal, release_usb_request
			 *will be called in dev release function
			 */
			dev_info(chan->dev, "wait write data signal interrupted\n");
			return -EINTR;
		} else if (tx->cond != FLAG_USER_TX_NOT_WAIT) {
			release_usb_request(chan, &tx->ep_lock,
				&tx->ep, &tx->req);
			return -ENODEV;
		}
	}

	status = ret_frame_info_to_tx_param(tx, user_elem);
	if (status < 0) {
		dev_err(chan->dev, "Failed to return user param\n");
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------*/

static long hbusb_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int status = 0;
	void __user *argp = (void __user *)arg;
	struct hbusb_channel		*chan = filp->private_data;
	struct hbusb_dev		*dev = global_hbusb_dev;
	unsigned int 				usb_plug_val = 0;

	switch (cmd) {
	case HBUSB_SYNC_RX_VALID_FRAMEBUF:
		if (!chan->rx_config_en) {
			dev_err(chan->dev, "usb channel not support read\n");
			return -EPERM;
		}

		if (mutex_lock_interruptible(&chan->read_lock)) {
			dev_info(chan->dev, "write lock signal interrupted\n");
			return -EINTR;
		}

		status = hbusb_sync_rx_valid_framebuf(filp, argp);
		mutex_unlock(&chan->read_lock);
		break;
	case HBUSB_SYNC_TX_VALID_FRAMEBUF:
		if (!chan->tx_config_en) {
			dev_err(chan->dev, "usb channel not support write\n");
			return -EPERM;
		}

		if (mutex_lock_interruptible(&chan->write_lock)) {
			dev_info(chan->dev, "write lock signal interrupted\n");
			return -EINTR;
		}

		status = hbusb_sync_tx_valid_framebuf(filp, argp);
		mutex_unlock(&chan->write_lock);
		break;
	case HBUSB_DEV_PRESENT:
		usb_plug_val = atomic_read(&dev->usb_plugin);
		*(unsigned int *)argp = usb_plug_val;
		break;
	default:
		return -EINVAL;
	}

	return status;
}

static const struct file_operations f_hbusb_fops = {
	.owner			= THIS_MODULE,
	.open			= hbusb_open,
	.release		= hbusb_release,
	.unlocked_ioctl	= hbusb_ioctl,
};
/*--------------------------------------------------------*/

/*--------------------hbusb channel init------------------------*/
static void rx_frame_info_var_init(struct hbusb_rx_chan *rx)
{
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;

	memset(frame_info, 0, sizeof (struct frame_info));
}

static void rx_frame_ctrl_var_init(struct hbusb_rx_chan *rx)
{
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;

	frame_ctrl->curr_sg_index_per_frame_info = 0;
	frame_ctrl->curr_slice_index_per_sg = 0;
	frame_ctrl->recv_slice_phys = 0;
	frame_ctrl->recv_slice_size = 0;
	frame_ctrl->recv_req_size = 0;
	frame_ctrl->seq_num = 0;
	frame_ctrl->rx_ctrl_head_actual_size = 0;
	frame_ctrl->hbusb_recv_event = EVENT_HBUSB_TX_CTRL_HEAD;
}

static int hbusb_rx_chan_init(struct hbusb_channel *chan)
{
	struct hbusb_rx_chan *rx = &chan->rx;
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;

	rx->req = NULL;
	spin_lock_init(&rx->ep_lock);

	frame_ctrl->rx_ctrl_head_buf =
	kmalloc(HBUSB_DMA_BUF_MAX_SIZE, GFP_KERNEL);
	if (frame_ctrl->rx_ctrl_head_buf == NULL)
		return -ENOMEM;

	frame_ctrl->frame_info = kmalloc(sizeof (struct frame_info),
							GFP_KERNEL);
	if (frame_ctrl->frame_info == NULL)
		goto fail;

	rx_frame_ctrl_var_init(rx);
	rx_frame_info_var_init(rx);
	rx->cond = FLAG_USER_RX_NOT_WAIT;
	init_waitqueue_head(&rx->wait);
	rx->parent_chan = chan;

	return 0;
fail:
	kfree(frame_ctrl->rx_ctrl_head_buf);
	frame_ctrl->rx_ctrl_head_buf = NULL;
	return -EINVAL;
}

static void hbusb_rx_chan_deinit(struct hbusb_channel *chan)
{
	struct hbusb_rx_chan *rx = &chan->rx;
	struct frame_rx_ctrl *frame_ctrl = &rx->frame_ctrl;

	rx_frame_ctrl_var_init(rx);
	rx_frame_info_var_init(rx);

	rx->cond = FLAG_USER_TX_NOT_WAIT;
	init_waitqueue_head(&rx->wait);
	rx->parent_chan = NULL;

	kfree(frame_ctrl->frame_info);
	frame_ctrl->frame_info = NULL;

	kfree(frame_ctrl->rx_ctrl_head_buf);
	frame_ctrl->rx_ctrl_head_buf = NULL;
}

static void tx_frame_info_var_init(struct hbusb_tx_chan *tx)
{
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;
	struct frame_info *frame_info = frame_ctrl->frame_info;

	memset(frame_info, 0, sizeof (struct frame_info));
}

static void tx_frame_ctrl_var_init(struct hbusb_tx_chan *tx)
{
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;

	frame_ctrl->curr_sg_index_per_frame_info = 0;
	frame_ctrl->curr_slice_index_per_sg = 0;
	frame_ctrl->send_slice_phys = 0;
	frame_ctrl->send_slice_size = 0;
	frame_ctrl->seq_num = 0;
	frame_ctrl->tx_ctrl_head_actual_size = 0;
	frame_ctrl->hbusb_send_event = EVENT_HBUSB_TX_CTRL_HEAD;
}

static int hbusb_tx_chan_init(struct hbusb_channel *chan)
{
	struct hbusb_tx_chan *tx = &chan->tx;
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;

	tx->req = NULL;
	spin_lock_init(&tx->ep_lock);

	frame_ctrl->tx_ctrl_head_buf =
		kmalloc(HBUSB_DMA_BUF_MAX_SIZE, GFP_KERNEL);
	if (frame_ctrl->tx_ctrl_head_buf == NULL)
		return -ENOMEM;

	frame_ctrl->frame_info = kmalloc(sizeof (struct frame_info),
							GFP_KERNEL);
	if (frame_ctrl->frame_info == NULL)
		goto fail;

	tx_frame_ctrl_var_init(tx);
	tx_frame_info_var_init(tx);
	tx->cond = FLAG_USER_TX_NOT_WAIT;
	init_waitqueue_head(&tx->wait);
	tx->parent_chan = chan;

	return 0;
fail:
	kfree(frame_ctrl->tx_ctrl_head_buf);
	frame_ctrl->tx_ctrl_head_buf = NULL;
	return -EINVAL;
}

static void hbusb_tx_chan_deinit(struct hbusb_channel *chan)
{
	struct hbusb_tx_chan *tx = &chan->tx;
	struct frame_tx_ctrl *frame_ctrl = &tx->frame_ctrl;

	tx_frame_ctrl_var_init(tx);
	tx_frame_info_var_init(tx);

	tx->cond = FLAG_USER_TX_NOT_WAIT;
	init_waitqueue_head(&tx->wait);
	tx->parent_chan = NULL;

	kfree(frame_ctrl->frame_info);
	frame_ctrl->frame_info = NULL;

	kfree(frame_ctrl->tx_ctrl_head_buf);
	frame_ctrl->tx_ctrl_head_buf = NULL;
}

static int hbusb_channels_init(struct hbusb_dev	*hbusb_dev)
{
	int i = 0;
	int status;

	for (i = 0; i < hbusb_dev->dev_chan_num; i++) {
		/*rx channel init*/
		if (i < hbusb_dev->epout_num) {
			mutex_init(&hbusb_dev->chan[i].read_lock);
			status = hbusb_rx_chan_init(&hbusb_dev->chan[i]);
			if (status)
				goto fail;
			hbusb_dev->chan[i].rx_config_en = 1;
		}

		/*tx channel init*/
		if (i < hbusb_dev->epin_num) {
			mutex_init(&hbusb_dev->chan[i].write_lock);
			status = hbusb_tx_chan_init(&hbusb_dev->chan[i]);
			if (status)
				goto fail;
			hbusb_dev->chan[i].tx_config_en = 1;
		}

		/*create device node*/
		hbusb_dev->chan[i].dev = device_create(hbusb_dev->dev_class, NULL,
					MKDEV(hbusb_dev->cdev_major, i), NULL,
					"hbusb%d", i);
		hbusb_dev->chan[i].chan_minor = i;

		/*init timer*/
		INIT_DELAYED_WORK(&hbusb_dev->chan[i].close_work,
					hbusb_release_work_func);
	}

	return 0;
fail:
	return status;
}

static void hbusb_channels_deinit(struct hbusb_dev	*hbusb_dev)
{
	int i = 0;
	int major;

	major = hbusb_dev->cdev_major;
	for (i = 0; i < hbusb_dev->dev_chan_num; i++) {
		if (hbusb_dev->chan[i].dev == NULL)
			continue;
		cancel_delayed_work_sync(&hbusb_dev->chan[i].close_work);
		device_destroy(hbusb_dev->dev_class, MKDEV(major, i));
		hbusb_dev->chan[i].dev = NULL;
		if (hbusb_dev->chan[i].tx_config_en) {
			hbusb_tx_chan_deinit(&hbusb_dev->chan[i]);
			hbusb_dev->chan[i].tx_config_en = 0;
		}

		if (hbusb_dev->chan[i].rx_config_en) {
			hbusb_rx_chan_deinit(&hbusb_dev->chan[i]);
			hbusb_dev->chan[i].rx_config_en = 0;
		}
		hbusb_dev->chan[i].chan_minor = -1;
	}
}
/*-------------------------------------------------------*/
static int hbusb_device_struct_register(struct device *dev)
{
	int status;
	struct hbusb_dev		*pdev;

	pdev = dev_get_drvdata(dev);
	if (pdev == NULL)
		return -ENOMEM;

	dev_set_name(dev, "hbusb%d", 0);
	dev->release = &hbusb_release_default;
	status = device_register(dev);
	if (status) {
		put_device(dev);
		return status;
	}

	return 0;
}

int hbusb_register_cdev(struct device *dev)
{
	int status;
	struct hbusb_dev		*pdev;

	pdev = dev_get_drvdata(dev);
	if (pdev == NULL)
		return -ENOMEM;

	status = hbusb_device_struct_register(dev);
	if (status < 0)
		return status;

	pdev->dev_class = class_create(THIS_MODULE, hbusb_class_name);
	if (IS_ERR(pdev->dev_class))
		return -ENOMEM;

	status = hbusb_channels_init(pdev);
	if (status < 0) {
		pr_err("Failed to init hbusb all channels\n");
		return status;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(hbusb_register_cdev);

struct device *hbusb_setup_default(unsigned int epin_num,
						unsigned int epout_num)
{
	int			  status;
	struct hbusb_dev		*pdev;

	pdev = kzalloc(sizeof(struct hbusb_dev), GFP_KERNEL);
	if (pdev == NULL)
		return ERR_PTR(-ENOMEM);

	global_hbusb_dev = pdev;

	pdev->epin_num = epin_num;
	pdev->epout_num = epout_num;
	pdev->dev_chan_num = max_t(unsigned int, pdev->epin_num, pdev->epout_num);
	status = __register_chrdev(0, 0, pdev->dev_chan_num,
						HBUSB_DEVICE_NAME, &f_hbusb_fops);
	if (status < 0)
		goto fail0;
	pdev->cdev_major = status;

	dev_set_drvdata(&pdev->dev, pdev);
	return &pdev->dev;

fail0:
	global_hbusb_dev = NULL;
	kfree(pdev);
	return NULL;
}
EXPORT_SYMBOL_GPL(hbusb_setup_default);

void hbusb_release_default(struct device *dev)
{
	struct hbusb_dev	*pdev = NULL;

	pdev = dev_get_drvdata(dev);
	if (pdev == NULL)
		return;

	__unregister_chrdev(pdev->cdev_major, 0,
					pdev->dev_chan_num, HBUSB_DEVICE_NAME);
	global_hbusb_dev = NULL;
	kfree(pdev);
}
EXPORT_SYMBOL_GPL(hbusb_release_default);

static void hbusb_device_struct_unregister(struct device *dev)
{
	device_unregister(dev);
}

void hbusb_cleanup(struct device *dev)
{
	struct hbusb_dev *pdev;

	pdev = dev_get_drvdata(dev);
	if (pdev != NULL) {
		hbusb_channels_deinit(pdev);
		class_destroy(pdev->dev_class);
		hbusb_device_struct_unregister(dev);
	}
}
EXPORT_SYMBOL_GPL(hbusb_cleanup);


void hbusb_set_gadget(struct device *dev, struct usb_gadget *g)
{
	struct hbusb_dev *pdev;

	pdev = dev_get_drvdata(dev);
	if (pdev != NULL)
		pdev->gadget = g;
}
EXPORT_SYMBOL_GPL(hbusb_set_gadget);

int hbusb_connect(struct    hbusb *link)
{
	int						i = 0;
	int						result = 0;
	struct hbusb_dev		*dev = link->ioport;
	unsigned long			flags;
	struct hbusb_channel		*chan;
	struct hbusb_rx_chan		*rx;
	struct hbusb_tx_chan		*tx;

	if (!dev)
		return -EINVAL;

	atomic_set(&dev->usb_plugin, HBUSB_DEV_PLUGIN);
	for (i = 0; i < dev->dev_chan_num; i++) {
		chan = &dev->chan[i];

		if (chan->rx_config_en) {
			rx = &chan->rx;
			link->bulkout[i]->driver_data = rx;
			result = usb_ep_enable(link->bulkout[i]);
			if (result != 0) {
				DBG(dev, "enable %s --> %d\n",
					link->bulkout[i]->name, result);
				goto fail;
			}
			spin_lock_irqsave(&rx->ep_lock, flags);
			rx->ep = link->bulkout[i];
			spin_unlock_irqrestore(&rx->ep_lock, flags);
		}

		if (chan->tx_config_en) {
			tx = &chan->tx;
			link->bulkin[i]->driver_data = tx;
			result = usb_ep_enable(link->bulkin[i]);
			if (result != 0) {
				DBG(dev, "enable %s --> %d\n",
					link->bulkin[i]->name, result);
				goto fail;
			}
			spin_lock_irqsave(&tx->ep_lock, flags);
			tx->ep = link->bulkin[i];
			spin_unlock_irqrestore(&tx->ep_lock, flags);
		}

		if (atomic_read(&chan->chan_opened) == HBUSB_CHANNEL_OPENED) {
			atomic_set(&chan->chan_running, HBUSB_CHANNEL_RUNNING);
			result = hbusb_chan_start_transfer(chan);
			if (result < 0) {
				pr_err("Failed to start hbusb channel transfer when usb is plugin\n");
				goto fail;
			}
		}
	}

	return 0;
fail:
	for (i = 0; i < dev->dev_chan_num; i++) {
		chan = &dev->chan[i];
		if (chan->rx_config_en) {
			rx = &chan->rx;
			if (rx->ep != NULL)
				(void) usb_ep_disable(link->bulkout[i]);
			spin_lock_irqsave(&rx->ep_lock, flags);
			rx->ep = NULL;
			spin_unlock_irqrestore(&rx->ep_lock, flags);
		}

		if (chan->tx_config_en) {
			tx = &chan->tx;
			if (tx->ep != NULL)
				(void) usb_ep_disable(link->bulkin[i]);
			spin_lock_irqsave(&tx->ep_lock, flags);
			tx->ep = NULL;
			spin_unlock_irqrestore(&tx->ep_lock, flags);
		}

		atomic_set(&chan->chan_running,
					HBUSB_CHANNEL_NOT_RUNNING);
		atomic_set(&dev->usb_plugin, HBUSB_DEV_PLUGOUT);
	}

	return result;
}
EXPORT_SYMBOL_GPL(hbusb_connect);

void hbusb_disconnect(struct      hbusb *link)
{
	int i;
	struct hbusb_dev		*dev = link->ioport;
	struct hbusb_channel	*chan;
	struct hbusb_tx_chan	*tx;
	struct hbusb_rx_chan	*rx;
	unsigned long			flags;

	WARN_ON(!dev);
	if (!dev)
		return;

	DBG(dev, "%s\n", __func__);

	/* disable endpoints, forcing (synchronous) completion
	 * of all pending i/o.  then free the request objects
	 * and forget about the endpoints.
	 */
	for (i = 0; i < dev->dev_chan_num; i++) {
		chan = &dev->chan[i];
		atomic_set(&chan->chan_running, HBUSB_CHANNEL_NOT_RUNNING);
	}
	atomic_set(&dev->usb_plugin, HBUSB_DEV_PLUGOUT);

	for (i = 0; i < dev->dev_chan_num; i++) {
		chan = &dev->chan[i];

		if (chan->rx_config_en) {
			rx = &chan->rx;
			rx->req = NULL;
			usb_ep_disable(link->bulkout[i]);
			spin_lock_irqsave(&rx->ep_lock, flags);
			rx->ep = NULL;
			spin_unlock_irqrestore(&rx->ep_lock, flags);
			if (rx->cond == FLAG_USER_RX_WAIT) {
				rx->cond = FLAG_USER_RX_FORCE_QUIT;
				wake_up_interruptible(&rx->wait);
			}
			link->bulkout[i]->driver_data = NULL;
			link->bulkout[i]->desc = NULL;
		}

		if (chan->tx_config_en) {
			tx = &chan->tx;
			tx->req = NULL;
			usb_ep_disable(link->bulkin[i]);
			spin_lock_irqsave(&tx->ep_lock, flags);
			tx->ep = NULL;
			spin_unlock_irqrestore(&tx->ep_lock, flags);
			if (tx->cond == FLAG_USER_TX_WAIT) {
				tx->cond = FLAG_USER_TX_FORCE_QUIT;
				wake_up_interruptible(&tx->wait);
			}
			link->bulkin[i]->driver_data = NULL;
			link->bulkin[i]->desc = NULL;
		}
	}
}
EXPORT_SYMBOL_GPL(hbusb_disconnect);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("horizon.ai");
