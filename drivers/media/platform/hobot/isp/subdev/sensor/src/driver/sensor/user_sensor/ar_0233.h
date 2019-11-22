#ifndef __AR_0233_H__
#define __AR_0233_H__

#include <linux/types.h>

enum ar0233_mode_e {
        AR0233_NORMAL_M = 0x00,
        AR0233_DOL2_M = 0x01,
        AR0233_DOL3_M = 0x02,
        AR0233_PWL = 0x03
};

typedef struct _ar0233_param_t {
        uint32_t VMAX;
        uint32_t HMAX;
        uint32_t FSC_DOL2;
        uint32_t FSC_DOL3;
        uint32_t RHS1;
        uint32_t RHS2;
        uint32_t lines_per_second;
        uint32_t gain_max;
        uint32_t exposure_time_max;
        uint32_t exposure_time_min;
	uint32_t exposure_time_long_max;
        uint16_t active_width;
        uint16_t active_height;
        uint8_t  lane;
        uint32_t clk;
        uint16_t frame;
        enum ar0233_mode_e ar0233_mode_save;
} ar0233_param_t;


struct sensor_operations *ar0233_ops_register(void);

#endif
