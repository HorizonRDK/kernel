#ifndef __OS8A10_H__
#define __OS8A10_H__

#include <linux/types.h>

enum os8a10_mode_e {
	OS8A10_NORMAL_M = 0x00,
	OS8A10_DOL2_M = 0x01,
	OS8A10_DOL3_M = 0x02
};

typedef struct _os8a10_param_t {
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
	enum os8a10_mode_e os8a10_mode_save;
} os8a10_param_t;


struct os8a10_NOR_s {
	uint32_t gain_num;
	uint32_t line_num;
};

struct os8a10_DOL2_s {
	uint32_t gain_num;
	uint32_t line_num;
};

struct os8a10_DOL3_s {
	uint32_t gain_num;
	uint32_t line_num;
};

struct os8a10_CONTROL_s {
	uint8_t gain_mode;
	struct os8a10_NOR_s nomal;
	struct os8a10_DOL2_s dol2;
	struct os8a10_DOL3_s dol3;
};

struct sensor_operations *os8a10_ops_register(void);

#endif
