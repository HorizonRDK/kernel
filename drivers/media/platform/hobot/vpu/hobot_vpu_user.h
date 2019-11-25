#ifndef __HOBOT_VPU_USER_H_
#define __HOBOT_VPU_USER_H_

#include <linux/types.h>

#define USE_VMALLOC_FOR_INSTANCE_POOL_MEMORY

#define VPU_DEV_NAME "vpu"

typedef struct _hb_vpu_drv_firmware {
	/* size of this structure */
	unsigned int size;
	unsigned int core_idx;
	unsigned long reg_base_offset;
	unsigned short bit_code[512];
} hb_vpu_drv_firmware_t;

typedef struct _hb_vpu_drv_buffer {
	unsigned int size;
	unsigned long phys_addr;

	/* kernel logical address in use kernel */
	unsigned long base;

	/* virtual user space address */
	unsigned long virt_addr;
} hb_vpu_drv_buffer_t;

typedef struct _hb_vpu_drv_inst {
	unsigned int core_idx;
	unsigned int inst_idx;

	/* for output only */
	int inst_open_count;
} hb_vpu_drv_inst_t;

typedef struct _hb_vpu_drv_intr {
	unsigned int timeout;
	int intr_reason;

//#ifdef SUPPORT_MULTI_INST_INTR
	int intr_inst_index;
//#endif
} hb_vpu_drv_intr_t;

#define VDI_IOCTL_MAGIC						'V'
#define VDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY			\
	_IOWR(VDI_IOCTL_MAGIC, 0, hb_vpu_drv_buffer_t)
#define VDI_IOCTL_FREE_PHYSICALMEMORY				\
	_IOWR(VDI_IOCTL_MAGIC, 1, hb_vpu_drv_buffer_t)
#define VDI_IOCTL_WAIT_INTERRUPT				\
	_IOWR(VDI_IOCTL_MAGIC, 2, hb_vpu_drv_intr_t)
#define VDI_IOCTL_SET_CLOCK_GATE				\
	_IOW(VDI_IOCTL_MAGIC, 3, unsigned int)
#define VDI_IOCTL_RESET						\
	_IO(VDI_IOCTL_MAGIC, 4)
#define VDI_IOCTL_GET_INSTANCE_POOL				\
	_IOWR(VDI_IOCTL_MAGIC, 5, hb_vpu_drv_buffer_t)
#define VDI_IOCTL_GET_COMMON_MEMORY				\
	_IOWR(VDI_IOCTL_MAGIC, 6, hb_vpu_drv_buffer_t)
#define VDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO		\
	_IOWR(VDI_IOCTL_MAGIC, 8, hb_vpu_drv_buffer_t)
#define VDI_IOCTL_OPEN_INSTANCE					\
	_IOWR(VDI_IOCTL_MAGIC, 9, hb_vpu_drv_inst_t)
#define VDI_IOCTL_CLOSE_INSTANCE				\
	_IOWR(VDI_IOCTL_MAGIC, 10, hb_vpu_drv_inst_t)
#define VDI_IOCTL_GET_INSTANCE_NUM				\
	_IOWR(VDI_IOCTL_MAGIC, 11, hb_vpu_drv_inst_t)
#define VDI_IOCTL_GET_REGISTER_INFO				\
	_IOWR(VDI_IOCTL_MAGIC, 12, hb_vpu_drv_buffer_t)
#define VDI_IOCTL_GET_FREE_MEM_SIZE				\
	_IOWR(VDI_IOCTL_MAGIC, 13, int)
#define VDI_IOCTL_ALLOCATE_INSTANCE_ID				\
	_IOR(VDI_IOCTL_MAGIC, 14, int)
#define VDI_IOCTL_FREE_INSTANCE_ID				\
	_IOW(VDI_IOCTL_MAGIC, 15, int)
#endif /* HOBOT_VPU_USER_H_ */
