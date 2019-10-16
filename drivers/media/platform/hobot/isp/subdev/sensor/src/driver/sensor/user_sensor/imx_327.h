#ifndef __IMX_327_H__
#define __IMX_327_H__

#include <linux/types.h>

#define IMX327_GAIN        	(0x3014)
#define IMX327_SHS1             (0x3020)
#define IMX327_SHS2             (0x3024)
#define IMX327_SHS3             (0x3028)
#define IMX327_VAMX	        (0x3018)
#define IMX327_HAMX	        (0x301C)
#define IMX327_RHS1		(0x3030)
#define IMX327_RHS2		(0x3034)
#define IMX327_FPGC		(0x3010)
#define IMX327_FPGC_1		(0x30f0)
#define IMX327_FPGC_2		(0x30f4)
#define IMX327_GAIN1		(0x30f2)
#define IMX327_GAIN2		(0x30f6)
#define IMX327_CSI_LANE_MODE    (0x3443)
#define IMX327_INCKSEL6         (0x3164)
#define IMX327_X_SIZE           (0x3472)
#define IMX327_Y_SIZE           (0x3418)

enum imx327_mode_e {
	NORMAL_M = 0x00,
	DOL2_M = 0x01,
	DOL3_M = 0x02
};

typedef struct _imx327_param_t {
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
	uint16_t active_width;
	uint16_t active_height;
	uint8_t  lane;
	uint32_t clk;
	uint16_t frame;
	enum imx327_mode_e imx327_mode_save;
} imx327_param_t;


struct imx327_NOR_s {
	uint32_t gain_num;
	uint32_t line_num;
};

struct imx327_DOL2_s {
	uint32_t gain_num;
	uint32_t line_num;
};

struct imx327_DOL3_s {
	uint32_t gain_num;
	uint32_t line_num;
};

struct imx327_CONTROL_s {
	uint8_t gain_mode;
	struct imx327_NOR_s nomal;
	struct imx327_DOL2_s dol2;
	struct imx327_DOL3_s dol3;
};

struct sensor_operations *imx327_ops_register(void);

#endif
