/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 *********************************************************************/

#ifndef __LENS_API_H__
#define __LENS_API_H__

#include <linux/types.h>

int lens_driver_init(uint16_t port, uint32_t param_id);
int lens_driver_get_param(uint16_t port, uint32_t param_id);
int lens_api_af_init(uint16_t chn);
int lens_api_af_move(uint16_t chn, uint32_t pos);
void lens_api_af_stop(uint16_t chn);
uint8_t lens_api_af_get_status(uint16_t chn);
uint32_t lens_api_af_get_pos(uint16_t chn);
int lens_api_af_get_param(uint16_t chn);
void lens_api_af_write_reg(uint16_t chn, uint32_t addr, uint32_t data);
void lens_api_af_read_reg(uint16_t chn, uint32_t addr, uint32_t *data);
int lens_api_zoom_init(uint16_t chn);
int lens_api_zoom_move(uint16_t chn, uint32_t pos);
void lens_api_zoom_stop(uint16_t chn);
uint8_t lens_api_zoom_get_status(uint16_t chn);
uint32_t lens_api_zoom_get_pos(uint16_t chn);
int lens_api_zoom_get_param(uint16_t chn);
void lens_api_zoom_write_reg(uint16_t chn, uint32_t addr, uint32_t data);
void lens_api_zoom_read_reg(uint16_t chn, uint32_t addr, uint32_t *data);
void lens_api_zoom_stop(uint16_t chn);

#endif /* __X3_LENS_API_H__ */  
