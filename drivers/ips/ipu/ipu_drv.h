#ifndef __IPU_DRV_H__
#define __IPU_DRV_H__

#include "ipu_dev.h"

#define IPU_IOC_MAGIC       'm'

#define IPUC_INIT               _IOW(IPU_IOC_MAGIC, 0, ipu_init_t)
#define IPUC_GET_IMG            _IOR(IPU_IOC_MAGIC, 1, int)
#define IPUC_CNN_DONE           _IO(IPU_IOC_MAGIC, 2)
#define IPUC_GET_DONE_INFO      _IOR(IPU_IOC_MAGIC, 3, info_h_t)
#define IPUC_GET_ERR_STATUS     _IOR(IPU_IOC_MAGIC, 4, uint32_t)
#define IPUC_DUMP_REG           _IO(IPU_IOC_MAGIC, 5)
#define IPUC_START              _IO(IPU_IOC_MAGIC, 6)
#define IPUC_STOP               _IO(IPU_IOC_MAGIC, 7)

typedef enum {
	IPUC_SET_DDR = 1,
	IPUC_SET_CROP,
	IPUC_SET_SCALE,
	IPUC_SET_PYMID,
	IPUC_SET_BASE,
	IPUC_SET_FRAME_ID,
} ipu_cmd_e;

#endif
