#ifndef __HOBOT_CNN_H__
#define __HOBOT_CNN_H__

#include <asm/ioctl.h>
#include <linux/kfifo.h>

#define CNN_DRV_NAME		"hobot_cnn"
#define CNN_INT_NUM 6

/*************************************************************
 * bpu register offset list
 *************************************************************/
#define   CNNBUS_CTRL_WM_0		    (0x0)
#define   CNNBUS_CTRL_WM_1		    (0x4)
#define   CNNBUS_CTRL_WM_2		    (0x8)
#define   CNNBUS_CTRL_WM_3		    (0xC)
#define   CNNBUS_CTRL_RM_0		    (0x40)
#define   CNNBUS_CTRL_RM_1		    (0x44)
#define   CNNBUS_CTRL_RM_2		    (0x48)
#define   CNNBUS_CTRL_RM_3		    (0x4C)
#define   CNNBUS_CTRL_RM_4		    (0x50)
#define   CNNBUS_CTRL_RM_5		    (0x54)
#define   CNNBUS_AXIID		    (0x80)
#define   CNNINT_MASK		    (0x84)
#define   CNNINT_STATUS		    (0x88)
#define   CNNINT_NUM			    (0x8C)
#define   CNNINT_ERR_NUM		    (0x90)
#define   CNN_FC_BASE		    (0x94)
#define   CNN_FC_HEAD		    (0x98)
#define   CNN_FC_TAIL		    (0x9C)
#define   CNN_FC_LEN			    (0xA0)
#define   CNNINT_INST_NUM		    (0xA4)
#define   CNN_BUSY_STATUS		    (0xB0)

/*************************************************************
 * register bit
 *************************************************************/

/*    CNNBUS_CTRL_WM_0    */
#define   CNN_WD_MAXLEN_M(n)			(((n) & 0xff) << 0x8)
#define   CNN_WD_MAXLEN_M_MASK		(0xff << 0x8)
#define   CNN_WD_MAXLEN_M_SHIT(n)		(((n) & 0xff) >> 0x8)
#define   CNN_WD_ENDIAN_M(n)			(((n) & 0xf) << 0x4)
#define   CNN_WD_ENDIAN_M_MASK		(0xf << 0x4)
#define   CNN_WD_ENDIAN_M_SHIT(n)		(((n) & 0xf) >> 0x4)
#define   CNN_WD_PRIORITY_M(n)		(((n) & 0xf) << 0x0)
#define   CNN_WD_PRIORITY_M_MASK		(0xf << 0x0)
#define   CNN_WD_PRIORITY_M_SHIT(n)		(((n) & 0xf) >> 0x0)

/*    CNN_CNNBUS_CTRL_RM_0	 */
#define   CNN_RD_MAXLEN_M(n)			(((n) & 0xff) << 0x8)
#define   CNN_RD_MAXLEN_M_MASK		(0xff << 0x8)
#define   CNN_RD_MAXLEN_M_SHIT(n)		(((n) & 0xff) >> 0x8)
#define   CNN_RD_ENDIAN_M(n)			(((n) & 0xf) << 0x4)
#define   CNN_RD_ENDIAN_M_MASK		(0xf << 0x4)
#define   CNN_RD_ENDIAN_M_SHIT(n)		(((n) & 0xf) >> 0x4)
#define   CNN_RD_PRIORITY_M(n)		(((n) & 0xf) << 0x0)
#define   CNN_RD_PRIORITY_M_MASK		(0xf << 0x0)
#define   CNN_RD_PRIORITY_M_SHIT(n)		(((n) & 0xf) >> 0x0)

/*    CNN_CNNBUS_AXIID    */
#define   CNN_AXIID(n)			(((n) & 0xf) << 0x0)
#define   CNN_AXIID_MASK			(0xf << 0x0)
#define   CNN_AXIID_SHIT(n)			(((n) & 0xf) >> 0x0)

/*    CNN_CNNINT_MASK    */
#define   CNN_PE0_INT_MASK(n)		(((n) & 0x1) << 0x0)
#define   CNN_PE0_INT_MASK_MASK		(0x1 << 0x0)
#define   CNN_PE0_INT_MASK_SHIT(n)		(((n) & 0x1) >> 0x0)

/*    CNN_CNNINT_STATUS    */
#define   CNN_PE0_INT_STATUS_RO		(0x1 << 0x0)
#define   CNN_PE0_INT_STATUS_RO_SHIT(n)	(((n) & 0x1) >> 0x0)

/*    CNN_CNNINT_NUM    */
#define   CNN_INT_NUM_RO			(0xffff << 0x0)
#define   CNN_INT_NUM_RO_SHIT(n)		(((n) & 0xffff) >> 0x0)

/*    CNN_CNNINT_ERR_NUM    */
#define   CNN_INT_ERR_NUM_RO			(0xffff << 0x0)
#define   CNN_INT_ERR_NUM_RO_SHIT(n)		(((n) & 0xffff) >> 0x0)

/*    CNN_CNNINT_FC_BASE    */
#define   CNN_PE0_FC_BASE(n)			(((n) & 0xffffffff) << 0x0)
#define   CNN_PE0_FC_BASE_MASK		(0xffffffff << 0x0)
#define   CNN_PE0_FC_BASE_SHIT(n)		(((n) & 0xffffffff) >> 0x0)

/*    CNN_CNNINT_FC_HEAD    */
#define   CNN_PE0_FC_HEAD_RO			(0x7ff << 0x0)
#define   CNN_PE0_FC_HEAD_RO_SHIT(n)		(((n) & 0x7ff) >> 0x0)

/*    CNN_CNNINT_FC_TAIL    */
#define   CNN_PE0_FC_TAIL(n)			(((n) & 0x7ff) << 0x0)
#define   CNN_PE0_FC_TAIL_MASK		(0x7ff << 0x0)
#define   CNN_PE0_FC_TAIL_SHIT(n)		(((n) & 0x7ff) >> 0x0)

/*    CNN_CNNINT_FC_LEN    */
#define   CNN_PE0_FC_LENGTH(n)		(((n) & 0x3ff) << 0x0)
#define   CNN_PE0_FC_LENGTH_MASK		(0x3ff << 0x0)
#define   CNN_PE0_FC_LENGTH_SHIT(n)		(((n) & 0x3ff) >> 0x0)

/*    CNN_CNNINT_INST_NUM	*/
#define   CNN_INST_NUMBER_RO			(0xffffffff << 0x0)
#define   CNN_INST_NUMBER_RO_SHIT(n)		(((n) & 0xffffffff) >> 0x0)

#define CNN_MAX_FC_LEN_MASK	0x3ff
#define CNN_FC_IDX_FLAG	0x400
#define CNN_FC_SIZE		0x40

// Default minor number for the device
#define MINOR_NUMBER		    0
// The default number of character devices for CNN
#define NUM_DEVICES		    1

#define CNN_FC_GAP_LEN 0x1000
#define CNN_FC_SPACE_LEN (0x400 * 0x40)

struct hobot_fc_time {
	unsigned int fc_count;
	unsigned int int_num;
	int time_flag;
	struct timeval start_time;
	struct timeval end_time;
};

struct hobot_bpu_int_num {
	u32 cnn_int_num[CNN_INT_NUM];
	u64 cnn_int_interval[CNN_INT_NUM];
	u32 cnn_int_count;
};

struct cnn_user_info {
	struct hobot_bpu_dev *cnn_dev;
	wait_queue_head_t cnn_int_wait;
	int irq_triggered;
	struct hobot_bpu_int_num cnn_int_num;
};

struct bpu_int_info {
	struct cnn_user_info **p_user_info;
	unsigned int fc_total;
	unsigned int int_num;
	unsigned int hw_id;
	struct timeval start_time;
	struct timeval end_time;
};

#ifdef CONFIG_HOBOT_CNN_DEVFREQ
struct hobot_bpufreq {
	struct devfreq *devfreq;
	struct mutex lock;

	u64 rate;
	u64 volt;
	struct thermal_cooling_device *cooling;
	struct devfreq_dev_profile devp;
};
#endif

struct hobot_bpu_dev {
	void __iomem	*cnn_base;
	u32		irq;
	void *fc_base;
	dma_addr_t fc_phys_base;
	unsigned int fc_mem_size;

	struct device *dev;
	int num_devices;
	unsigned int minor_num;
	dev_t dev_num;
	char *chrdev_name;
	struct class *dev_class;
	struct cdev i_cdev;
	int core_index;

	/* core running time statistics */
	atomic_t hw_id_counter;
	uint64_t run_time;
	struct timeval int_point;

	struct mutex cnn_lock;
	spinlock_t set_time_lock;
	spinlock_t cnn_spin_lock;
	spinlock_t kfifo_lock;

	struct reset_control *cnn_rst;

	/* wait queue for wait cnn interrupt occur */
	atomic_t wait_fc_cnt;
	atomic_t hw_flg;
	struct tasklet_struct tasklet;
	struct task_struct *maintain_task;
	unsigned int maintain_head;
	struct dentry *debugfs_root;
	struct list_head debugfs_list;
	struct mutex debugfs_lock; /* Protects debugfs_list. */
	struct hobot_fc_time *fc_time;
	unsigned int time_head;
	unsigned int time_tail;
	struct kfifo int_info_fifo;
	struct regulator *cnn_regulator;
	struct clk *cnn_aclk;
	struct clk *cnn_mclk;
	void __iomem *cnn_pmu;
	u32 iso_bit;
	struct completion bpu_completion;
	struct completion nega_completion;
	int zero_int_cnt;
	struct timeval zero_int_start_time;
	unsigned int real_int_cnt;
	unsigned int wait_nega_flag;
#ifdef CONFIG_HOBOT_CNN_DEVFREQ
	struct hobot_bpufreq *cnnfreq;
#endif
	struct timer_list cnn_timer;
	unsigned int head_value;
	unsigned int inst_num;
	unsigned int ref_cnt;
	unsigned int disable_bpu;
	struct dentry *cnn_debugfs_root;
	unsigned int has_regulator;
	unsigned int bpu_detached;
};

struct cnn_debugfs_info {
	const char *name;
	int (*show)(struct seq_file*, void*); /** show callback */
};

struct cnn_info_node {
	struct list_head list;
	struct hobot_bpu_dev *cnn_dev;
	const struct cnn_debugfs_info *info_ent;
	struct dentry *dent;
};

struct hobot_bpu_allocation {
	size_t size;		    // Size of the buffer
	void *user_addr;	    // User virtual address of the buffer
	void *kern_addr;	    // Kernel virtual address of the buffer
	dma_addr_t dma_addr;	    // DMA bus address of the buffer
};

struct	hobot_bpu_fc_info {
	void *fc_info;
	int fc_cnt;
};

struct hobot_bpu_fc_status {
	u32 free_fc_fifo_cnt;
};

struct hobot_bpu_rst_data {
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

union cnn_ioctl_arg {
	struct hobot_bpu_fc_info fc_data;
	struct hobot_bpu_fc_status fc_status;
	struct hobot_bpu_rst_data rst_data;
	struct hobot_bpu_int_num int_num_data;
	int pid_fc_mask;
	uint64_t core_run_time;
};

#define CNN_MT_WB 0x1
#define CNN_MT_UC 0x2
#define CNN_MT_WC 0x3
#define CNN_MT_WT 0x4

#define CNN_IOCTL_MAGIC 'C'
#define CNN_NUM_IOCTLS	6
#define CNN_IOC_GET_FC_STA	(_IOR(CNN_IOCTL_MAGIC, \
					0, struct hobot_bpu_fc_status))
#define CNN_IOC_FC_ENQUEUE	(_IOW(CNN_IOCTL_MAGIC, \
					1, struct hobot_bpu_fc_info))
#define CNN_IOC_RST		(_IOW(CNN_IOCTL_MAGIC, \
					2, struct hobot_bpu_rst_data))
#define CNN_IOC_GET_INT_NUM	(_IOR(CNN_IOCTL_MAGIC, \
					3, struct hobot_bpu_int_num))
#define CNN_IOC_GET_ID_MASK	(_IOR(CNN_IOCTL_MAGIC, 4, int))
#define CNN_IOC_GET_CORE_RUNTIME	(_IOR(CNN_IOCTL_MAGIC, 5, uint64_t))
#endif	/* __HOBOT_CNN_H__ */
