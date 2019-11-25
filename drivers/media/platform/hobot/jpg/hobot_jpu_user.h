#ifndef __HOBOT_JPU_USER_H__
#define __HOBOT_JPU_USER_H__

#include <linux/fs.h>
#include <linux/types.h>

#define JPU_DEV_NAME                    "jpu"
//#define MAX_NUM_JPU_INSTANCE            64
//#define MAX_NUM_JPU_CORE                1
//#define MAX_INST_HANDLE_SIZE            48

typedef struct _hb_jpu_drv_buffer {
	unsigned int size;
	unsigned long phys_addr;
	/* kernel logical address in use kernel */
	unsigned long base;
	/* virtual user space address */
	unsigned long virt_addr;
} hb_jpu_drv_buffer_t;

typedef struct _hb_jpu_drv_inst {
	unsigned int inst_idx;
	/* for output only */
	int inst_open_count;
} hb_jpu_drv_inst_t;

typedef struct _hb_jpu_drv_intr {
	unsigned int timeout;
	int intr_reason;
	unsigned int inst_idx;
} hb_jpu_drv_intr_t;

#define JDI_IOCTL_MAGIC  'J'

#define JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY          \
    _IOWR(JDI_IOCTL_MAGIC, 0, hb_jpu_drv_buffer_t)
#define JDI_IOCTL_FREE_PHYSICAL_MEMORY               \
    _IOWR(JDI_IOCTL_MAGIC, 1, hb_jpu_drv_buffer_t)
#define JDI_IOCTL_WAIT_INTERRUPT                    \
    _IOWR(JDI_IOCTL_MAGIC, 2, hb_jpu_drv_intr_t)
#define JDI_IOCTL_SET_CLOCK_GATE                    \
    _IOWR(JDI_IOCTL_MAGIC, 3, unsigned int)
#define JDI_IOCTL_RESET                             \
    _IO(JDI_IOCTL_MAGIC, 4)
#define JDI_IOCTL_GET_INSTANCE_POOL                 \
    _IOWR(JDI_IOCTL_MAGIC, 5, hb_jpu_drv_buffer_t)
#define JDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO    \
    _IOWR(JDI_IOCTL_MAGIC, 6, hb_jpu_drv_buffer_t)
#define JDI_IOCTL_GET_REGISTER_INFO                 \
    _IOWR(JDI_IOCTL_MAGIC, 7, hb_jpu_drv_buffer_t)
#define JDI_IOCTL_OPEN_INSTANCE                     \
    _IOWR(JDI_IOCTL_MAGIC, 8, hb_jpu_drv_inst_t)
#define JDI_IOCTL_CLOSE_INSTANCE                    \
    _IOWR(JDI_IOCTL_MAGIC, 9, hb_jpu_drv_inst_t)
#define JDI_IOCTL_GET_INSTANCE_NUM                  \
    _IOWR(JDI_IOCTL_MAGIC, 10, hb_jpu_drv_inst_t)
#define JDI_IOCTL_ALLOCATE_INSTANCE_ID		\
    _IOR(JDI_IOCTL_MAGIC, 14, int)
#define JDI_IOCTL_FREE_INSTANCE_ID		\
    _IOW(JDI_IOCTL_MAGIC, 15, int)

#endif /* __HOBOT_JPU_USER_H__ */
