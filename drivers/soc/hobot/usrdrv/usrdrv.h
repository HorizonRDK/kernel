/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @author   Jesse.Huang (Jesse.Huang@hobot.cc)
 * @date     2019/11/27
 * @version  V1.0
 * @par      Horizon Robotics
 */

#ifndef DRIVERS_IPS_USRDRV_USRDRV_H_
#define DRIVERS_IPS_USRDRV_USRDRV_H_
/*reg driver info*/
#define USRDRV_MAJOR 253
/*reg mode attr*/

#define USRDRV_IOC_MAGIC        'v'
#define USRDRV_IRQ_REGISTER     _IOW(USRDRV_IOC_MAGIC, 0, ioctl_irq_info_t)
#define USRDRV_IRQ_REGISTER     _IOW(USRDRV_IOC_MAGIC, 0, ioctl_irq_info_t)
#define USRDRV_IRQ_UNREGISTER   _IOW(USRDRV_IOC_MAGIC, 1, uint32_t)
#define USRDRV_IRQ_ENABLE       _IOW(USRDRV_IOC_MAGIC, 2, uint32_t)
#define USRDRV_IRQ_AUTO_FIRE    _IOW(USRDRV_IOC_MAGIC, 3, uint32_t)

#define USRDRV_IRQNAME_MAX	100
/************************************************
* IOCTL Struct
************************************************/
typedef struct ioctl_irq_info_s {
	uint32_t irq_index;
	uint8_t irq_name[USRDRV_IRQNAME_MAX];
} ioctl_irq_info_t;
#endif //DRIVERS_IPS_USRDRV_USRDRV_H_