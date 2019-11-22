#ifndef __IMX_290_H__
#define __IMX_290_H__

#include <linux/types.h>

#define IMX290_GAIN        	(0x3014)
#define IMX290_SHS1             (0x3020)
#define IMX290_SHS2             (0x3024)
#define IMX290_SHS3             (0x3028)
#define IMX290_VAMX	        (0x3018)
#define IMX290_HAMX	        (0x301C)
#define IMX290_RHS1		(0x3030)
#define IMX290_RHS2		(0x3034)
#define IMX290_FPGC		(0x3010)
#define IMX290_FPGC_1		(0x30f0)
#define IMX290_FPGC_2		(0x30f4)
#define IMX290_GAIN1		(0x30f2)
#define IMX290_GAIN2		(0x30f6)
#define IMX290_CSI_LANE_MODE    (0x3443)
#define IMX290_INCKSEL6         (0x3164)
#define IMX290_X_SIZE           (0x3472)
#define IMX290_Y_SIZE           (0x3418)

enum imx290_mode_e {
        imx290_NORMAL_M = 0x00,
        imx290_DOL2_M = 0x01,
        imx290_DOL3_M = 0x02
};

typedef struct _imx290_param_t {
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
	enum imx290_mode_e imx290_mode_save;
} imx290_param_t;


struct imx290_NOR_s {
        uint32_t gain_num;
        uint32_t line_num;
};

struct imx290_DOL2_s {
        uint32_t gain_num;
        uint32_t line_num;
};

struct imx290_DOL3_s {
        uint32_t gain_num;
        uint32_t line_num;
};

struct imx290_CONTROL_s {
	uint8_t gain_mode;
       	struct imx290_NOR_s nomal;
       	struct imx290_DOL2_s dol2;
	struct imx290_DOL3_s dol3;
};

struct sensor_operations *imx290_ops_register(void);

#endif
