/***************************************************************************
 *   Copyright (C) 2021 by horizon.                                        *
 *   dinggao.pan@horizon.ai                                                *
 *                                                                         *
 *   Diag inject test file.                                                *
 *                                                                         *
 ***************************************************************************/
#ifndef DRIVERS_SOC_HOBOT_DIAG_DIAG_INJECT_H_
#define DRIVERS_SOC_HOBOT_DIAG_DIAG_INJECT_H_
#include <linux/ioctl.h>
typedef struct {
    uint16_t module_id;
    uint32_t inject_val;
} diag_inject_data_t;
#define INJECT_EN _IO('q', 0x1)
#define INJECT_VAL_SET _IOW('q', 0x2, diag_inject_data_t *)
#define INJECT_VAL_CHK _IOR('q', 0x3, diag_inject_data_t *)
#define INJECT_EN_CHK _IOR('q', 0x4, diag_inject_data_t *)
#define INJECT_VAL_CLR _IOR('q', 0x5, diag_inject_data_t *)

#endif  // DRIVERS_SOC_HOBOT_DIAG_DIAG_INJECT_H_
