#ifndef __X2_CNN_H__
#define __X2_CNN_H__

#include <asm/ioctl.h>

#define X2_CNN_DRV_NAME		"x2_cnn"

/*************************************************************
 * x2 cnn register offset list
 *************************************************************/
#define   X2_CNNBUS_CTRL_WM_0		    (0x0)
#define   X2_CNNBUS_CTRL_WM_1		    (0x4)
#define   X2_CNNBUS_CTRL_WM_2		    (0x8)
#define   X2_CNNBUS_CTRL_WM_3		    (0xC)
#define   X2_CNNBUS_CTRL_RM_0		    (0x40)
#define   X2_CNNBUS_CTRL_RM_1		    (0x44)
#define   X2_CNNBUS_CTRL_RM_2		    (0x48)
#define   X2_CNNBUS_CTRL_RM_3		    (0x4C)
#define   X2_CNNBUS_CTRL_RM_4		    (0x50)
#define   X2_CNNBUS_CTRL_RM_5		    (0x54)
#define   X2_CNNBUS_AXIID		    (0x80)
#define   X2_CNNINT_MASK		    (0x84)
#define   X2_CNNINT_STATUS		    (0x88)
#define   X2_CNNINT_NUM			    (0x8C)
#define   X2_CNNINT_ERR_NUM		    (0x90)
#define   X2_CNN_FC_BASE		    (0x94)
#define   X2_CNN_FC_HEAD		    (0x98)
#define   X2_CNN_FC_TAIL		    (0x9C)
#define   X2_CNN_FC_LEN			    (0xA0)
#define   X2_CNNINT_INST_NUM		    (0xA4)

/*************************************************************
 * register bit
 *************************************************************/

/*    X2_CNNBUS_CTRL_WM_0    */
#define   X2_CNN_WD_MAXLEN_M(n)			(((n) & 0xff) << 0x8)
#define   X2_CNN_WD_MAXLEN_M_MASK		(0xff << 0x8)
#define   X2_CNN_WD_MAXLEN_M_SHIT(n)		(((n) & 0xff) >> 0x8)
#define   X2_CNN_WD_ENDIAN_M(n)			(((n) & 0xf) << 0x4)
#define   X2_CNN_WD_ENDIAN_M_MASK		(0xf << 0x4)
#define   X2_CNN_WD_ENDIAN_M_SHIT(n)		(((n) & 0xf) >> 0x4)
#define   X2_CNN_WD_PRIORITY_M(n)		(((n) & 0xf) << 0x0)
#define   X2_CNN_WD_PRIORITY_M_MASK		(0xf << 0x0)
#define   X2_CNN_WD_PRIORITY_M_SHIT(n)		(((n) & 0xf) >> 0x0)

/*    X2_CNN_CNNBUS_CTRL_RM_0	 */
#define   X2_CNN_RD_MAXLEN_M(n)			(((n) & 0xff) << 0x8)
#define   X2_CNN_RD_MAXLEN_M_MASK		(0xff << 0x8)
#define   X2_CNN_RD_MAXLEN_M_SHIT(n)		(((n) & 0xff) >> 0x8)
#define   X2_CNN_RD_ENDIAN_M(n)			(((n) & 0xf) << 0x4)
#define   X2_CNN_RD_ENDIAN_M_MASK		(0xf << 0x4)
#define   X2_CNN_RD_ENDIAN_M_SHIT(n)		(((n) & 0xf) >> 0x4)
#define   X2_CNN_RD_PRIORITY_M(n)		(((n) & 0xf) << 0x0)
#define   X2_CNN_RD_PRIORITY_M_MASK		(0xf << 0x0)
#define   X2_CNN_RD_PRIORITY_M_SHIT(n)		(((n) & 0xf) >> 0x0)

/*    X2_CNN_CNNBUS_AXIID    */
#define   X2_CNN_AXIID(n)			(((n) & 0xf) << 0x0)
#define   X2_CNN_AXIID_MASK			(0xf << 0x0)
#define   X2_CNN_AXIID_SHIT(n)			(((n) & 0xf) >> 0x0)

/*    X2_CNN_CNNINT_MASK    */
#define   X2_CNN_PE0_INT_MASK(n)		(((n) & 0x1) << 0x0)
#define   X2_CNN_PE0_INT_MASK_MASK		(0x1 << 0x0)
#define   X2_CNN_PE0_INT_MASK_SHIT(n)		(((n) & 0x1) >> 0x0)

/*    X2_CNN_CNNINT_STATUS    */
#define   X2_CNN_PE0_INT_STATUS_RO		(0x1 << 0x0)
#define   X2_CNN_PE0_INT_STATUS_RO_SHIT(n)	(((n) & 0x1) >> 0x0)

/*    X2_CNN_CNNINT_NUM    */
#define   X2_CNN_INT_NUM_RO			(0xffff << 0x0)
#define   X2_CNN_INT_NUM_RO_SHIT(n)		(((n) & 0xffff) >> 0x0)

/*    X2_CNN_CNNINT_ERR_NUM    */
#define   X2_CNN_INT_ERR_NUM_RO			(0xffff << 0x0)
#define   X2_CNN_INT_ERR_NUM_RO_SHIT(n)		(((n) & 0xffff) >> 0x0)

/*    X2_CNN_CNNINT_FC_BASE    */
#define   X2_CNN_PE0_FC_BASE(n)			(((n) & 0xffffffff) << 0x0)
#define   X2_CNN_PE0_FC_BASE_MASK		(0xffffffff << 0x0)
#define   X2_CNN_PE0_FC_BASE_SHIT(n)		(((n) & 0xffffffff) >> 0x0)

/*    X2_CNN_CNNINT_FC_HEAD    */
#define   X2_CNN_PE0_FC_HEAD_RO			(0x7ff << 0x0)
#define   X2_CNN_PE0_FC_HEAD_RO_SHIT(n)		(((n) & 0x7ff) >> 0x0)

/*    X2_CNN_CNNINT_FC_TAIL    */
#define   X2_CNN_PE0_FC_TAIL(n)			(((n) & 0x7ff) << 0x0)
#define   X2_CNN_PE0_FC_TAIL_MASK		(0x7ff << 0x0)
#define   X2_CNN_PE0_FC_TAIL_SHIT(n)		(((n) & 0x7ff) >> 0x0)

/*    X2_CNN_CNNINT_FC_LEN    */
#define   X2_CNN_PE0_FC_LENGTH(n)		(((n) & 0x3ff) << 0x0)
#define   X2_CNN_PE0_FC_LENGTH_MASK		(0x3ff << 0x0)
#define   X2_CNN_PE0_FC_LENGTH_SHIT(n)		(((n) & 0x3ff) >> 0x0)

/*    X2_CNN_CNNINT_INST_NUM	*/
#define   X2_CNN_INST_NUMBER_RO			(0xffffffff << 0x0)
#define   X2_CNN_INST_NUMBER_RO_SHIT(n)		(((n) & 0xffffffff) >> 0x0)

#define X2_CNN_MAX_FC_LEN_MASK	0x3ff
#define X2_CNN_FC_IDX_FLAG	0x400
#define X2_CNN_FC_SIZE		0x40

// Default minor number for the device
#define MINOR_NUMBER		    0
// The default number of character devices for CNN
#define NUM_DEVICES		    1


#define CNN_FC_GAP0_BASE  0x32500000
#define CNN_FC_GAP0_SIZE  0x1000

#define CNN0_FC_PHYS_BASE 0x32501000
#define CNN0_FC_SPACE_SIZE (0x400*0x40)

#define CNN_FC_GAP1_BASE  0x32511000
#define CNN_FC_GAP1_SIZE  0x1000

#define CNN1_FC_PHYS_BASE 0x32512000
#define CNN1_FC_SPACE_SIZE (0x400*0x40)

#define CNN_FC_GAP2_BASE  0x32522000
#define CNN_FC_GAP2_SIZE  0x1000




struct x2_cnn_dev {
	void __iomem	*cnn_base;
	u32		irq;
	void *fc_base;
	unsigned int fc_phys_base;
	unsigned int fc_mem_size;

	struct device *dev;
	int num_devices;
	unsigned int minor_num;
	dev_t dev_num;
	char *chrdev_name;
	struct class *dev_class;
	struct cdev i_cdev;

	struct mutex cnn_lock;
	spinlock_t cnn_spin_lock;
	int irq_triggered;

	struct reset_control *cnn0_rst;
	struct reset_control *cnn1_rst;

	/* wait queue for wait cnn interrupt occur */
	wait_queue_head_t cnn_int_wait;
	u32 x2_cnn_int_num;

	struct tasklet_struct tasklet;
	struct dentry *debugfs_root;
	struct list_head debugfs_list;
	struct mutex debugfs_lock; /* Protects debugfs_list. */

};

struct cnn_debugfs_info {
	const char *name;
	int (*show)(struct seq_file*, void*); /** show callback */
};

struct cnn_info_node {
	struct list_head list;
	struct x2_cnn_dev *cnn_dev;
	const struct cnn_debugfs_info *info_ent;
	struct dentry *dent;
};

struct x2_cnn_allocation {
	size_t size;		    // Size of the buffer
	void *user_addr;	    // User virtual address of the buffer
	void *kern_addr;	    // Kernel virtual address of the buffer
	dma_addr_t dma_addr;	    // DMA bus address of the buffer
};

struct	x2_cnn_fc_info {
	void *fc_info;
	int fc_cnt;
};

struct x2_cnn_fc_status {
	u32 free_fc_fifo_cnt;
};

struct x2_cnn_rst_data {
	u32 cnn_rst_id;
};



struct hbrt_x2_funccall_s {
	uint32_t dyn_base_addr5;
	uint32_t dyn_base_addr4;

	uint16_t resizer_perf_flag;
	uint8_t dest_height_m1;
	uint8_t dest_width_m1;

	uint16_t step_h;
	uint16_t step_w;

	uint16_t y_source_bottom;
	uint16_t y_source_right;
	uint16_t y_source_top;
	uint16_t y_source_left;
	uint16_t y_source_stride;
	uint16_t y_source_height_m1;
	uint16_t y_source_width_m1;

	uint16_t interrupt_num;

	uint32_t uv_source_address;
	uint32_t y_source_address;
	uint32_t instruction_length;

	uint32_t dyn_base_addr3;
	uint32_t dyn_base_addr2;
	uint32_t dyn_base_addr1;
	uint32_t dyn_base_addr0;
	uint32_t instruction_address;
};

struct x2_cnn_int_num {
	u32 cnn_int_num;
};

union cnn_ioctl_arg {
	struct x2_cnn_fc_info fc_data;
	struct x2_cnn_fc_status fc_status;
	struct x2_cnn_rst_data rst_data;
	struct x2_cnn_int_num int_num_data;
};

#define CNN_MT_WB 0x1
#define CNN_MT_UC 0x2
#define CNN_MT_WC 0x3
#define CNN_MT_WT 0x4

#define CNN_IOCTL_MAGIC 'C'
#define CNN_NUM_IOCTLS	6
#define CNN_IOC_GET_FC_STA	(_IOR(CNN_IOCTL_MAGIC, \
					0, struct x2_cnn_fc_status))
#define CNN_IOC_FC_ENQUEUE	(_IOW(CNN_IOCTL_MAGIC, \
					1, struct x2_cnn_fc_info))
#define CNN_IOC_RST		(_IOW(CNN_IOCTL_MAGIC, \
					2, struct x2_cnn_rst_data))
#define CNN_IOC_GET_INT_NUM	(_IOR(CNN_IOCTL_MAGIC, \
					3, struct x2_cnn_int_num))
#endif	/* __X2_CNN_H__ */
