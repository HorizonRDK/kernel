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

#ifndef DRIVERS_IPS_USRDRV_INC_CLOCK_DUMP_H_
#define DRIVERS_IPS_USRDRV_INC_CLOCK_DUMP_H_
void clk_dump_list(void);
uint32_t clk_dump(volatile unsigned char *base, char *clk_name);
#endif //DRIVERS_IPS_USRDRV_INC_CLOCK_DUMP_H_
