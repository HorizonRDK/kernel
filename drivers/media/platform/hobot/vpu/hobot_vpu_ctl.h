#ifndef __HOBOT_VPU_CTL_H__
#define __HOBOT_VPU_CTL_H__

#include <linux/types.h>
#include "hobot_vpu_reg.h"

#define VPU_ISSUE_COMMAND(core, cmd) \
			do {	\
				VPU_WRITEL(W5_VPU_BUSY_STATUS, 1);	\
				VPU_WRITEL(W5_COMMAND, cmd);	\
				VPU_WRITEL(W5_VPU_HOST_INT_REQ, 1);	\
			} while(0)

int hb_vpu_hw_reset(void);

#endif /*__HOBOT_VPU_CTL_H__*/
