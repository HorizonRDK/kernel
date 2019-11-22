#ifndef __IMX_385_H__
#define __IMX_385_H__

#include <linux/types.h>

#define IMX385_GAIN        	(0x3014)
#define IMX385_SHS1             (0x3020)
#define IMX385_SHS2             (0x3023)
#define IMX385_SHS3             (0x3026)
#define IMX385_VAMX	        (0x3018)
#define IMX385_HAMX	        (0x301B)
#define IMX385_RHS1		(0x302C)
#define IMX385_CSI_LANE_MODE    (0x337F)
#define IMX385_INCKFREQ1        (0x3380)
#define IMX385_PIC_SIZE_V       (0x3357)

enum imx385_mode_e {
        imx385_NORMAL_M = 0x00,
        imx385_DOL2_M = 0x01
};

typedef struct _imx385_param_t {
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
	enum imx385_mode_e imx385_mode_save;
} imx385_param_t;


struct imx385_NOR_s {
        uint32_t gain_num;
        uint32_t line_num;
};

struct imx385_DOL2_s {
        uint32_t gain_num;
        uint32_t line_num;
};


struct imx385_CONTROL_s {
	uint8_t imx385_gain_mode;
       	struct imx385_NOR_s nomal;
       	struct imx385_DOL2_s dol2;
};

struct sensor_operations *imx385_ops_register(void);

#endif
