/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef DRIVERS_SOC_HOBOT_VCAM_HB_VCAM_H_
#define DRIVERS_SOC_HOBOT_VCAM_HB_VCAM_H_


extern struct ion_device	*hb_ion_dev;	// for ion create client
/*
 * for notify vio free frame
 */
extern int vio_register_notify(struct notifier_block *nb);
extern int vio_unregister_notify(struct notifier_block *nb);
extern int vcam_notify_frame_done(unsigned long val, void *v);

/* for image format */
#define VCAM_YUV_420_8		0x18
#define VCAM_YUV_420_10		0x19
#define VCAM_YUV_422_8		0x1E
#define VCAM_YUV_422_10		0x1F
#define VCAM_RAW_8			0x2A
#define VCAM_RAW_10			0x2B
#define VCAM_RAW_12			0x2C
#define VCAM_RAW_14			0x2D

/* for msg type */
enum vcam_cmd {
	CMD_VCAM_INIT = 1,
	CMD_VCAM_START,
	CMD_VCAM_NEXT_REQUEST,
	CMD_VCAM_STOP,
	CMD_VCAM_DEINIT,
	CMD_VCAM_MAX
};

/* for vcam image info */
struct vcam_img_info_t {
	int width;
	int heigh;
	int stride;
	int format;
};

/* for vcam slot info */
struct vcam_slot_info_t {
	int						slot_id;
	int						cam_id;
	int64_t					timestamp;
	struct vcam_img_info_t	img_info;
};

/*
 * for vcam group info
 * group_size = slot_size * slot_num;
 * every_group_addr = base + g_id * group_size;
 */
struct vcam_group_info_t {
	int				g_id;		// group id
	uint64_t		base;		// first group paddr
	int				slot_size;	// a slot size
	int				slot_num;	// a group have slot_num slot
	int				flag;		// 0 free 1 busy
};

/* for vcam msg info */
struct hb_vcam_msg_t {
	int				info_type;
	struct vcam_group_info_t	group_info;
	struct vcam_slot_info_t		slot_info;
};

/* for vcam device & driver region */
struct vcam_cdev_t {
	const char *name;
	struct class *class;
	struct device *device;
	struct cdev cdev;
	dev_t dev_num;
};

/* main struct */
struct vcam_ctx_t {
	struct vcam_cdev_t vcam_cdev;
	struct hb_vcam_msg_t vcam_msg;
	struct notifier_block notifier_block;
	wait_queue_head_t event_head;
	wait_queue_head_t next_frame;
	wait_queue_head_t frame_done;
	struct ion_client *vcam_iclient;
	struct ion_handle *vcam_ihandle;
	int	ion_cnt;
	phys_addr_t p_base;
	void *v_base;
	struct task_struct *vcam_task;
	int free_group;
};

/* init info */
struct vcam_init_info_t {
	int width;
	int heigh;
	int stride;
	int format;
};

/* vcam send info for vio */
struct addr_info_t {
	uint16_t width;
	uint16_t height;
	uint16_t step;
	uint64_t y_paddr;
	uint64_t c_paddr;
	uint64_t y_vaddr;
	uint64_t c_vaddr;
};

/* vcam send info for vio */
struct vcam_to_ipu_t {
	int g_id;
	int slot_id;
	int cam_id;
	int64_t timestamp;
	struct addr_info_t src_img;
};

struct mem_info_t {
	int size;
	uint64_t base;
};
#endif // DRIVERS_SOC_HOBOT_VCAM_HB_VCAM_H_
