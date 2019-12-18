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
#ifndef DRIVERS_IPS_USRDRV_INC_SCRIPT_H_
#define DRIVERS_IPS_USRDRV_INC_SCRIPT_H_
uint32_t usrdrv_run_script(volatile unsigned char *base, char *name);
void usrdrv_list_script(void);
#endif //DRIVERS_IPS_USRDRV_INC_SCRIPT_H_
