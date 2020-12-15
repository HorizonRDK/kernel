/* Copyright (C) 2020 - 2021  Horizon */
/* SPDX-License-Identifier: GPL-2.0 */
#ifndef DRIVERS_USB_GADGET_FUNCTION_HOBOT_HBUSB_U_HBUSB_H_
#define DRIVERS_USB_GADGET_FUNCTION_HOBOT_HBUSB_U_HBUSB_H_

#include <linux/err.h>
#include <linux/usb/composite.h>

#define HBUSB_MINOR_BASE   0
#define HBUSB_MINOR_COUNT  1
#define HBUSB_DEVICE_NAME  "hbusb"

#define HBUSB_DMA_BUF_MAX_SIZE 0x400000
#define HBUSB_BUF_SLICE_NUM (HBUSB_MAX_SIZE_PER_SG/HBUSB_DMA_BUF_MAX_SIZE)

/*-------------------hbusb dev macro--------------------*/
#define WRITE_FOR_CALLBACK_COMPLETE			200
enum {
	HBUSB_CHANNEL_CLOSED,
	HBUSB_CHANNEL_OPENED
};

enum {
	HBUSB_DEV_PLUGOUT,
	HBUSB_DEV_PLUGIN
};

enum {
	HBUSB_CHANNEL_NOT_RUNNING,
	HBUSB_CHANNEL_RUNNING,
};
/*---------------------------------------------------*/

/*-------------------tx/rx flag enum--------------------*/
enum {
	FLAG_RX_FRAME_NOT_VALID,
	FLAG_RX_FRAME_VALID
};

enum err_code{
	HBUSB_SG_NUM_NOT_EQUAL = 2,
	HBUSB_RECV_NOT_COMPLETE
};

enum {
	EVENT_HBUSB_TX_CTRL_HEAD = 1,
	EVENT_HBUSB_TX_USER_DATA,
	EVENT_HBUSB_TX_CTRL_END,
	EVENT_HBUSB_TX_FINISHED
};

enum {
	EVENT_HBUSB_RX_CTRL_HEAD = 1,
	EVENT_HBUSB_RX_USER_DATA,
	EVENT_HBUSB_RX_CTRL_END,
	EVENT_HBUSB_RX_FINISHED,
	EVENT_HBUSB_RX_HEAD_IN_DATABUF
};

enum {
	FLAG_USER_TX_WAIT,
	FLAG_USER_TX_NOT_WAIT,
	FLAG_USER_TX_FORCE_QUIT
};
enum {
	FLAG_USER_RX_WAIT,
	FLAG_USER_RX_NOT_WAIT,
	FLAG_USER_RX_FORCE_QUIT,
};
/*-------------------------------------------------*/

/*-------------------elem info--------------------*/
#define HBUSB_SG_MAX_NUM			16

typedef struct {
        void *virt_addr;
        unsigned long phy_addr;
        int length;
        int actual_length;
} scatterlist_t;

typedef struct {
	scatterlist_t sg[HBUSB_SG_MAX_NUM];
	int num_sgs;
} data_elem_t;

typedef struct {
	void *buf;
	int	size;
	int	actual_size;
} ctrl_elem_t;

typedef struct {
	ctrl_elem_t ctrl_info;
	data_elem_t data_info;
} elem_t;
/*----------------------------------------------------*/

/*----------------------------------------------------------------------*/
#define HBUSB_MAX_SIZE_PER_SG		0x1000000

struct frame_sg_info {
	/*add extera one array member for ZLP slice*/
	unsigned long slice_addr[HBUSB_BUF_SLICE_NUM + 1]; /*buf is 4M size align*/
	int slice_size[HBUSB_BUF_SLICE_NUM + 1];
	int slice_actual_size[HBUSB_BUF_SLICE_NUM + 1];
	int slice_num;
};

struct frame_info {
	elem_t elem;
	struct frame_sg_info sg_info[HBUSB_SG_MAX_NUM];
	int sg_num;
	int sg_actual_num;
	int error_val;
};
/*---------------------------------------------------------------*/

/*---------------RX/TX frame head info----------------*/
struct hbusb_frame_head {
	unsigned int	start_end_magic_num;
	int				user_ctrl_info_len;
	int				sgs_num;
	unsigned int	sequene_num;
	unsigned char	reserved[112];
} __packed;

/*all frame head 128 byte*/
#define USB_FRAME_HEAD_SIZE	(sizeof(struct hbusb_frame_head))
#define USER_CTRL_INFO_MAX_SIZE 512
/*make sure HBUSB_CTRL_FRAME_SIZE is not interger multiples of 1024, 
 * or In host, 1024xn size will not lzp
 */

#define HBUSB_CTRL_FRAME_SIZE (USB_FRAME_HEAD_SIZE + USER_CTRL_INFO_MAX_SIZE)
#define HBUSB_CTRL_FRMAE_SPACE 0x400000
/*--------------------------------------------------*/

/*----------------rx channel frame ctrl-----------------*/
struct frame_rx_ctrl {
	/* frame info */
	struct frame_info	*frame_info;

	/* rx ctrl head buf*/
	void __iomem		*rx_ctrl_head_buf;
	int 				rx_ctrl_head_actual_size;

	/*curr_use_frame_info: used by usb ctrl to send data to hardware.*/
	int curr_sg_index_per_frame_info;
	int curr_slice_index_per_sg;

	/*one time temp libusb recv param*/
	unsigned long recv_slice_phys;
	int 		recv_slice_size;

	unsigned int		seq_num;

	/*tx_envent*/
	unsigned int 		hbusb_recv_event;
};

struct hbusb_rx_chan {
	spinlock_t				ep_lock;
	struct usb_ep			*ep;
	struct usb_request		*req;
	struct frame_rx_ctrl	frame_ctrl;

	/*wait for rx valid data complete*/
	size_t					cond;
	wait_queue_head_t		wait;

	/*channel index*/
	struct hbusb_channel		*parent_chan;
};
/*---------------------------------------------------*/

/*-----------------tx channel frame ctrl------------------*/
struct frame_tx_ctrl {
	/* frame info */
	struct frame_info	*frame_info;

	/* tx ctrl head buf*/
	void __iomem		*tx_ctrl_head_buf;
	int 				tx_ctrl_head_actual_size;

	/*curr_use_frame_info: used by usb ctrl to send data to hardware.
	 */
	int curr_sg_index_per_frame_info;
	int curr_slice_index_per_sg;

	/*one time temp libusb recv param*/
	unsigned long send_slice_phys;
	int 		send_slice_size;

	unsigned int		seq_num;

	/*tx_envent*/
	unsigned int 		hbusb_send_event;
};

struct hbusb_tx_chan {
	spinlock_t				ep_lock;
	struct usb_ep			*ep;
	struct usb_request		*req;
	struct frame_tx_ctrl	frame_ctrl;

	/*wait for valid data complete*/
	size_t					cond;
	wait_queue_head_t		wait;

	/*channel index*/
	struct hbusb_channel		*parent_chan;
};
/*---------------------------------------------------*/

/*------------------hbusb dev struct info-------------------*/
struct hbusb_channel {
	struct device			*dev;
	struct hbusb_rx_chan		rx;
	struct hbusb_tx_chan		tx;

	/*read/write control*/
	struct mutex		read_lock;
	struct mutex		write_lock;

	/*chan open flag*/
	atomic_t			chan_opened;
	atomic_t			chan_running;

	int					chan_minor;

	/*close delay work*/
	struct delayed_work close_work;
};

struct hbusb_dev {
	struct device		dev;
	int                 cdev_major;
	struct class		*dev_class;
	struct usb_gadget	*gadget;

	/*per channel map one minor device*/
	struct hbusb_channel chan[HBUSB_MINOR_COUNT];

	/*hbusb dev plugin*/
	atomic_t			usb_plugin;
};
/*---------------------------------------------------*/

struct hbusb {
	struct usb_function		func;

	/* updated by hbusb_{connect,disconnect} */
	struct hbusb_dev			*ioport;

	/* endpoints handle super speeds */
	struct usb_ep			*bulkin[HBUSB_MINOR_COUNT];
	struct usb_ep			*bulkout[HBUSB_MINOR_COUNT];
};

struct f_hbusb_opts {
	struct usb_function_instance	func_inst;
	struct device                   *dev;
	bool				bound;

	/*
	 * Read/write access to configfs attributes is handled by configfs.
	 *
	 * This is to protect the data from concurrent access by read/write
	 * and create symlink/remove symlink.
	 */
	struct mutex			lock;
	int				refcnt;
};

int hbusb_register_cdev(struct device *dev);
struct device *hbusb_setup_default(void);
void hbusb_release_default(struct device *dev);
void hbusb_cleanup(struct device *dev);
void hbusb_set_gadget(struct device *dev, struct usb_gadget *g);
int hbusb_connect(struct hbusb *link);
void hbusb_disconnect(struct hbusb *link);

#endif /* DRIVERS_USB_GADGET_FUNCTION_HOBOT_HBUSB_U_HBUSB_H_ */
