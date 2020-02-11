#ifndef __SENSOR_I2C_H__
#define __SENSOR_I2C_H__

#include <linux/types.h>

#define IMX_290
#define IMX_385

#define SENSOR_I2C 0
#define SENSOR_NUM 0


struct sensor_priv_old {
	int32_t analog_gain;
	int32_t digital_gain;
	uint16_t int_time;
	uint16_t int_time_M;
	uint16_t int_time_L; 
};

struct _setting_param_t {
        uint32_t lines_per_second;
        uint32_t analog_gain_max;
        uint32_t digital_gain_max;
        uint32_t exposure_time_max;
        uint32_t exposure_time_min;
        uint32_t exposure_time_limit;
	uint32_t exposure_time_long_max;
        uint16_t active_width;
        uint16_t active_height;
	uint32_t fps;
};



struct sensor_operations {
	uint8_t param_enable;
        int (* sensor_hw_reset_enable) (void);
        int (* sensor_hw_reset_disable) (void);
	int32_t (* sensor_alloc_analog_gain) ( uint8_t chn, int32_t gain );
	int32_t (*sensor_alloc_digital_gain) ( uint8_t chn, int32_t gain );
	void (* sensor_alloc_integration_time) ( uint8_t chn, uint16_t *int_time, uint16_t *int_time_M, uint16_t *int_time_L );
	void (* sensor_update) (uint8_t chn, struct sensor_priv_old updata);
	uint16_t (* sensor_get_id) ( uint8_t chn );
	void (* sensor_disable_isp) ( uint8_t chn );
	uint32_t (* read_register) ( uint8_t chn, uint32_t address );
	void (* write_register) ( uint8_t chn, uint32_t address, uint32_t data );
	void (* stop_streaming) ( uint8_t chn );
	void (* start_streaming) ( uint8_t chn );
	int (* sensor_init) ( uint8_t chn, uint8_t mode);
	void (* sesor_get_para)(uint8_t chn, struct _setting_param_t *user_para);
};


struct sensor_operations *sensor_chn_open(uint8_t chn, uint32_t i2c_chn, uint8_t sensor_num);
int sensor_chn_release(uint8_t chn);
int sensor_i2c_read(uint8_t chn, uint16_t reg_addr, uint8_t bit_width, char *buf, size_t count);
int sensor_i2c_write(uint8_t chn, uint16_t reg_addr, uint8_t bit_width, const char *buf, size_t count);

#endif /* __X2_ISP_H__ */  
