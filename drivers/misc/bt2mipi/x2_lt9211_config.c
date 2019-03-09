/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/cdev.h>
//#include <asm/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

#include "x2_lt9211.h"

struct video_timing video_384x292_60Hz = {
	6, 5, 5, 384, 400, 4, 2, 2, 292, 300, 3000};
struct video_timing video_640x480_60Hz = {
	8, 96, 40, 640, 800, 33, 2, 10, 480, 525, 25000};
struct video_timing video_720x480_60Hz = {
	16, 62, 60, 720, 858, 9, 6, 30, 480, 525, 27000};
struct video_timing video_1280x720_60Hz = {
	110, 40, 220, 1280, 1650, 5, 5, 20, 720, 750, 74250};
struct video_timing video_1280x720_30Hz = {
	110, 40, 220, 1280, 1650, 5, 5, 20, 720, 750, 74250};
struct video_timing video_1366x768_60Hz = {
	26, 110, 110, 1366, 1592, 13, 6, 13, 768, 800, 81000};
struct video_timing video_1920x1080_30Hz = {
	88, 44, 148, 1920, 2200, 4, 5, 36, 1080, 1125, 74250};
struct video_timing video_1920x1080_60Hz = {
	88, 44, 148, 1920, 2200, 4, 5, 36, 1080, 1125, 148500};
struct video_timing video_3840x1080_60Hz = {
	176, 88, 296, 3840, 4400, 4, 5, 36, 1080, 1125, 297000};
struct video_timing video_1920x1200_60Hz = {
	48, 32, 80, 1920, 2080, 3, 6, 26, 1200, 1235, 154000};
struct video_timing video_3840x2160_30Hz = {
	176, 88, 296, 3840, 4400, 8, 10, 72, 2160, 2250, 297000};
struct video_timing video_3840x2160_60Hz = {
	176, 88, 296, 3840, 4400, 8, 10, 72, 2160, 2250, 594000};
struct video_timing video_1920x1080_25Hz = {
	528, 44, 148, 1920, 2640, 4, 5, 36, 1080, 1125, 74250};


struct video_timing TimingStr;
enum VideoFormat Video_Format;

//extern int lt9211_reset_pin;
//extern int lcd_reset_pin;
//extern int lcd_pwm_pin;

//extern struct x2_lt9211_s  *g_x2_lt9211;

int set_lt9211_config(struct fb_info *fb, unsigned int convert_type)
{
	int ret = 0;

	ret = lt9211_reset();
	if (ret) {
		pr_err("Err reset lt9211!!\n");
		return ret;
	}
	if (convert_type == BT1120_TO_RGB888) {
		ret = lt9211_system_int_to_RGB();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_ttl_rx_phy_to_RGB();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_BT_video_check_to_RGB();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		//ret = lt9211_set_timing_para_to_RGB();
		//if (ret) {
			//pr_info("%s() Err init lt9211
			//system ret= %d\n", __func__, ret);
			//return ret;
		//}
		//Htotal
		ret = x2_write_lt9211(0xff, 0x85);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x20, (uint8_t)(TimingStr.hact >> 8));
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x21, (uint8_t)(TimingStr.hact));
		if (ret < 0)
			return ret;
		//HFP
		//ret = x2_write_lt9211(0x22, (uint8_t)(TimingStr.hfp >> 8));
		ret = x2_write_lt9211(0x22,
				(uint8_t)(fb->var.left_margin >> 8));
		if (ret < 0)
			return ret;
		//ret = x2_write_lt9211(0x23, (uint8_t)(TimingStr.hfp));
		ret = x2_write_lt9211(0x23, (uint8_t)(fb->var.left_margin));
		if (ret < 0)
			return ret;
		//HSW
		//ret = x2_write_lt9211(0x24, (uint8_t)(TimingStr.hs >> 8));
		ret = x2_write_lt9211(0x24, (uint8_t)(fb->var.hsync_len >> 8));
		if (ret < 0)
			return ret;
		//ret = x2_write_lt9211(0x25, (uint8_t)(TimingStr.hs));
		ret = x2_write_lt9211(0x25, (uint8_t)(fb->var.hsync_len));
		if (ret < 0)
			return ret;
		//VFP
		//ret = x2_write_lt9211(0x38, (uint8_t)(TimingStr.vfp >> 8));
		ret = x2_write_lt9211(0x38,
				(uint8_t)(fb->var.upper_margin >> 8));
		if (ret < 0)
			return ret;
		//ret = x2_write_lt9211(0x39, (uint8_t)(TimingStr.vfp);
		ret = x2_write_lt9211(0x39, (uint8_t)(fb->var.upper_margin));
		if (ret < 0)
			return ret;
		//VSW
		//ret = x2_write_lt9211(0x3c, (uint8_t)(TimingStr.vs >> 8));
		ret = x2_write_lt9211(0x3c, (uint8_t)(fb->var.vsync_len >> 8));
		if (ret < 0)
			return ret;
		//ret = x2_write_lt9211(0x3d, (uint8_t)(TimingStr.vs);
		ret = x2_write_lt9211(0x3d, (uint8_t)(fb->var.vsync_len));
		if (ret < 0)
			return ret;

		msleep(100);
		ret = lt9211_tx_digital_RGB();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_RGB_tx_phy();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		//msleep(10);
		usleep_range(10000, 11000);
		ret = lt9211_RGB_tx_pll();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_rx_csc();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
	} else if (convert_type == BT1120_TO_MIPI) {
		ret = lt9211_system_int_to_mipi();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_ttl_rx_phy_to_mipi();
		if (ret) {
			pr_info("%s() Err config lt9211 ttl rx phy ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_csc_to_mipi();
		if (ret) {
			pr_info("%s() Err config lt9211 csc ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_video_check_to_mipi();
		if (ret) {
			pr_info("%s() Err check lt9211 video ret= %d\n",
					__func__, ret);
			return ret;
		}
		//ret = lt9211_set_timing_para_to_mipi();
		//if (ret) {
			//pr_info("%s() Err set lt9211 timing
			//parameter ret= %d\n", __FUNCTION__, ret);
			//return ret;
		//}

		ret = x2_write_lt9211(0xff, 0x85);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x2c, (uint8_t)(TimingStr.htotal >> 8));
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x2d, (uint8_t)(TimingStr.htotal));
		if (ret < 0)
			return ret;
		//HFP
		//ret = x2_write_lt9211(0x30, (uint8_t)(TimingStr.hfp >> 8));
		ret = x2_write_lt9211(0x30,
				(uint8_t)(fb->var.left_margin >> 8));
		if (ret < 0)
			return ret;
		//ret = x2_write_lt9211(0x31, (uint8_t)(TimingStr.hfp));
		ret = x2_write_lt9211(0x31, (uint8_t)(fb->var.left_margin));
		if (ret < 0)
			return ret;
		//HSW
		//ret = x2_write_lt9211(0x34, (uint8_t)(TimingStr.hs >> 8));
		ret = x2_write_lt9211(0x34, (uint8_t)(fb->var.hsync_len >> 8));
		if (ret < 0)
			return ret;
		//ret = x2_write_lt9211(0x35, (uint8_t)(TimingStr.hs));
		ret = x2_write_lt9211(0x35, (uint8_t)(fb->var.hsync_len));
		if (ret < 0)
			return ret;
		//VFP
		//ret = x2_write_lt9211(0x38, (uint8_t)(TimingStr.vfp >> 8));
		ret = x2_write_lt9211(0x38,
				(uint8_t)(fb->var.upper_margin >> 8));
		if (ret < 0)
			return ret;
		//ret = x2_write_lt9211(0x39, (uint8_t)(TimingStr.vfp));
		ret = x2_write_lt9211(0x39, (uint8_t)(fb->var.upper_margin));
		if (ret < 0)
			return ret;
		//VSW
		//ret = x2_write_lt9211(0x3c, (uint8_t)(TimingStr.vs >> 8));
		ret = x2_write_lt9211(0x3c, (uint8_t)(fb->var.vsync_len >> 8));
		if (ret < 0)
			return ret;
		//ret = x2_write_lt9211(0x3d, (uint8_t)(TimingStr.vs));
		ret = x2_write_lt9211(0x3d, (uint8_t)(fb->var.vsync_len));
		if (ret < 0)
			return ret;

		ret = lt9211_mipi_tx_phy();
		if (ret) {
			pr_info("%s() Err config lt9211 mipi tx phy ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_mipi_tx_pll();
		if (ret) {
			pr_info("%s() Err config lt9211 mipi tx pll ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = init_panel();
		if (ret) {
			pr_info("%s() Err init dsi panel ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_set_tx_timing();
		if (ret) {
			pr_info("%s() Err set lt9211 tx timing ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_mipi_tx_digital();
		if (ret) {
			pr_info("%s() Err config lt9211 mipi tx digital ret= %d\n",
					__func__, ret);
			return ret;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(set_lt9211_config);

int x2_lt9211_write_byte(struct i2c_client *client, uint32_t addr,
		uint8_t reg, uint8_t val)
{
	struct i2c_msg msg;
	uint8_t buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	pr_err("lt9211 i2c write error addr:0%x reg:0x%x ret %d !\n",
			addr, reg, ret);

	return ret;
}

int x2_write_lt9211(uint8_t reg_addr, uint8_t reg_val)
{
	return x2_lt9211_write_byte(g_x2_lt9211->client,
			LT9211_SLAVE_ADDR, reg_addr, reg_val);
}

int x2_lt9211_read_byte(struct i2c_client *client, uint32_t addr,
		uint8_t reg, uint8_t *val)
{
	struct i2c_msg msg[2];
	uint8_t buf[1];
	int ret;

	buf[0] = reg & 0xFF;

	msg[0].addr = addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}
	pr_err("lt9211 i2c read error addr: 0x%x reg: 0x%x ret %d !!!\n",
			addr, reg, ret);
	return ret;
}

int x2_read_lt9211(uint8_t reg_addr, uint8_t *reg_val)
{
	return x2_lt9211_read_byte(g_x2_lt9211->client,
			LT9211_SLAVE_ADDR, reg_addr, reg_val);
}

int lt9211_dsi_lcd_init(unsigned int convert_type)
{
	//unsigned int type = convert_type;
	int ret = 0;

	ret = lt9211_reset();
	if (ret) {
		pr_err("Err reset lt9211!!\n");
		return ret;
	}
	lt9211_config(convert_type);
	return 0;
}

int lt9211_reset(void)
{
	int ret;

	ret = gpio_request(lt9211_reset_pin, "x2_lt9211_reset_pin");
	if (ret) {
		pr_info("%s() Err get trigger pin ret= %d\n",
					__func__, ret);
		return -ENODEV;
	}
	gpio_direction_output(lt9211_reset_pin, 0);
	msleep(100);
	gpio_direction_output(lt9211_reset_pin, 1);
	msleep(100);

	return 0;
}

int lt9211_config(unsigned int convert_type)
{
	int ret = 0;

	ret = lt9211_chip_id();
	if (ret) {
		pr_info("%s() Err get lt9211 chip id ret= %d\n",
					__func__, ret);
		return ret;
	}
	if (convert_type == BT1120_TO_MIPI) {
		ret = lt9211_system_int_to_mipi();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_ttl_rx_phy_to_mipi();
		if (ret) {
			pr_info("%s() Err config lt9211 ttl rx phy ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_csc_to_mipi();
		if (ret) {
			pr_info("%s() Err config lt9211 csc ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_video_check_to_mipi();
		if (ret) {
			pr_info("%s() Err check lt9211 video ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_set_timing_para_to_mipi();
		if (ret) {
			pr_info("%s() Err set lt9211 timing parameter ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_mipi_tx_phy();
		if (ret) {
			pr_info("%s() Err config lt9211 mipi tx phy ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_mipi_tx_pll();
		if (ret) {
			pr_info("%s() Err config lt9211 mipi tx pll ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = init_panel();
		if (ret) {
			pr_info("%s() Err init dsi panel ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_set_tx_timing();
		if (ret) {
			pr_info("%s() Err set lt9211 tx timing ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_mipi_tx_digital();
		if (ret) {
			pr_info("%s() Err config lt9211 mipi tx digital ret= %d\n",
					__func__, ret);
			return ret;
		}
	} else if (convert_type == BT1120_TO_RGB888) {
		ret = lt9211_system_int_to_RGB();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_ttl_rx_phy_to_RGB();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_BT_video_check_to_RGB();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_set_timing_para_to_RGB();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		msleep(100);
		ret = lt9211_tx_digital_RGB();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_RGB_tx_phy();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		msleep(20);
		ret = lt9211_RGB_tx_pll();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_rx_csc();
		if (ret) {
			pr_info("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		//LT9211_ClockCheckDebug();
		//LT9211_VideoCheckDebug();
		//LT9211_BT_Set();
	}
	return 0;
}

int lt9211_chip_id(void)
{
	int ret = 0;
	uint8_t chip_id[3] = {0x0};

	ret = x2_write_lt9211(0xff, 0x81);
	if (ret < 0)
		return ret;
	ret = x2_read_lt9211(0x00, &(chip_id[0]));
	if (ret < 0)
		return ret;
	pr_info("\nLT9211 Chip ID:%x,", chip_id[0]);
	ret = x2_read_lt9211(0x01, &chip_id[1]);
	if (ret < 0)
		return ret;
	pr_info("%x,", chip_id[1]);
	ret = x2_read_lt9211(0x02, &chip_id[2]);
	if (ret < 0)
		return ret;
	pr_info("%x\n", chip_id[2]);
	return 0;

}

int lt9211_system_int_to_mipi(void)
{
	int ret = 0;

	/*system clock init*/
	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x01, 0x18);
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x06, 0x61);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x07, 0xa8);//fm for sys_clk
	if (ret < 0)
		return ret;

	/*reduced power consumption*/
	ret = x2_write_lt9211(0xff, 0x81);//clock gating
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x31, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x32, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x34, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x35, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x37, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x3b, 0x00);
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0xff, 0x87);//init plltx
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x14, 0x08);//default value
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x15, 0x00);//default value
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x18, 0x0f);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x22, 0x08);//default value
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x23, 0x00);//default value
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x26, 0x0f);
	if (ret < 0)
		return ret;
	return 0;
}

int lt9211_system_int_to_RGB(void)
{
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x01, 0x18);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x06, 0x61);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x07, 0xa8);
	if (ret < 0)
		return ret;
	return 0;
}

int lt9211_ttl_rx_phy_to_mipi(void)
{
	int ret = 0;

	//select ttl input
	ret = x2_write_lt9211(0xff, 0x85);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x88, 0x90);
	if (ret < 0)
		return ret;

	//Data mapping
	ret = x2_write_lt9211(0x45, 0x40);//b100:GBR
	if (ret < 0)
		return ret;

	//De mode, SYNC_GEN enable
	//HDMI_WriteI2C_Byte(0x47,0x40);
	//8BIT
	ret = x2_write_lt9211(0x48, 0x40);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x00, 0x14);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x28, 0x40);//8Bit:41  16Bit:40
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x61, 0x09);
	if (ret < 0)
		return ret;
	return 0;
}

int lt9211_ttl_rx_phy_to_RGB(void)
{
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x28, 0x40);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x61, 0x09);
	if (ret < 0)
		return ret;

	//Data mapping
	ret = x2_write_lt9211(0xff, 0x85);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x88, 0x90);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x45, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x48, 0x18);//8BIT
	if (ret < 0)
		return ret;
	return 0;
}

int lt9211_csc_to_mipi(void)
{
	int ret;

	ret = x2_write_lt9211(0xff, 0xf9);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x90, 0x03);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x91, 0x4f);
	if (ret < 0)
		return ret;
	return 0;
}

int lt9211_video_check_to_mipi(void)
{
	uint16_t hact, vact;
	uint8_t hact_h, hact_l, vact_h, vact_l;
	uint16_t htotal;
	uint8_t htotal_h, htotal_l;
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0)
		return ret;

	while (1) {
		ret = x2_read_lt9211(0x8b, &vact_h);
		if (ret < 0)
			return ret;
		ret = x2_read_lt9211(0x8c, &vact_l);
		if (ret < 0)
			return ret;
		vact = (((uint16_t)vact_h) << 8) + (uint8_t)vact_l;
		ret = x2_read_lt9211(0x8d, &hact_h);
		if (ret < 0)
			return ret;
		ret = x2_read_lt9211(0x8e, &hact_l);
		if (ret < 0)
			return ret;
		hact = (((uint8_t)hact_h) << 8) + (uint8_t)hact_l;
		ret = x2_read_lt9211(0x8f, &htotal_h);
		if (ret < 0)
			return ret;
		ret = x2_read_lt9211(0x90, &htotal_l);
		if (ret < 0)
			return ret;
		htotal = (((uint8_t)htotal_h) << 8) + (uint8_t)htotal_l;

		pr_info("\nlt9211 video chech debug: hact = 0x%x",
									hact);
		pr_info("\nlt9211 video chech debug:vact = 0x%x", vact);
		pr_info("\nlt9211 video chech debug:htotal = 0x%x",
									htotal);

		if ((hact == video_1920x1080_60Hz.hact) &&
				(vact == video_1920x1080_60Hz.vact) &&
				(htotal == video_1920x1080_60Hz.htotal)) {
			pr_info("\nVideo_check = video_1920x1080_60Hz");
			Video_Format = video_1920x1080_60Hz_vic;
			TimingStr = video_1920x1080_60Hz;
			break;
		}
		if ((hact == video_640x480_60Hz.hact) &&
				(vact == video_640x480_60Hz.vact) &&
				(htotal == video_640x480_60Hz.htotal)) {
			pr_info("\nVideo_check = video_640x480_60Hz");
			Video_Format = video_640x480_60Hz_vic;
			TimingStr = video_640x480_60Hz;
			break;
		}
		pr_info("\nVideo_check None\n");
		Video_Format = video_none;
		msleep(2000);
	}
	return 0;
}

int lt9211_BT_video_check_to_RGB(void)
{
	int ret = 0;
	uint16_t hact, vact, htotal = 0;
	uint8_t hact_h, hact_l, vact_h, vact_l, htotal_h, htotal_l;
	uint8_t bt_flag = 0;

	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x20, 0x33);
	if (ret < 0)
		return ret;

	msleep(100);
	while (1) {
		ret = x2_write_lt9211(0xff, 0x85);
		if (ret < 0)
			return ret;
		ret = x2_read_lt9211(0x4f, &bt_flag);
		if (ret < 0)
			return ret;
		if (bt_flag & 0x40) {
			pr_info("\nBT1120 flag is set\n");
			ret = x2_write_lt9211(0xff, 0x86);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x8b, &vact_h);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x8c, &vact_l);
			if (ret < 0)
				return ret;
			vact = (uint16_t)vact_h << 8 | (uint16_t)vact_l;
			ret = x2_read_lt9211(0x8d, &hact_h);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x8e, &hact_l);
			if (ret < 0)
				return ret;
			hact = ((uint16_t)hact_h << 8 | (uint16_t)hact_l) - 4;

			ret = x2_read_lt9211(0x8f, &htotal_h);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x90, &htotal_l);
			if (ret < 0)
				return ret;
			vact = (uint16_t)htotal_h << 8 | (uint16_t)htotal_l;
			pr_info("hact = 0x%x.\n", hact);
			pr_info("vact = 0x%x.\n", vact);
			pr_info("htotal = 0x%x.\n", htotal);

			if ((vact == video_1920x1080_25Hz.vact)
				&& (hact == video_1920x1080_25Hz.hact)
				&& (htotal == video_1920x1080_25Hz.htotal)) {
				pr_info("Video_check = video_1920x1080_25Hz.\n");
				Video_Format = video_1920x1080_25Hz_vic;
				TimingStr = video_1920x1080_25Hz;
				break;
			}
		}
	}
	return 0;
}

int lt9211_set_timing_para_to_mipi(void)
{
	int ret = 0;
	//Htotal

	ret = x2_write_lt9211(0xff, 0x85);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x2c, (uint8_t)(TimingStr.htotal >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x2d, (uint8_t)(TimingStr.htotal));
	if (ret < 0)
		return ret;
	//HFP
	ret = x2_write_lt9211(0x30, (uint8_t)(TimingStr.hfp >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x31, (uint8_t)(TimingStr.hfp));
	if (ret < 0)
		return ret;
	//HSW
	ret = x2_write_lt9211(0x34, (uint8_t)(TimingStr.hs >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x35, (uint8_t)(TimingStr.hs));
	if (ret < 0)
		return ret;
	//VFP
	ret = x2_write_lt9211(0x38, (uint8_t)(TimingStr.vfp >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x39, (uint8_t)(TimingStr.vfp));
	if (ret < 0)
		return ret;
	//VSW
	ret = x2_write_lt9211(0x3c, (uint8_t)(TimingStr.vs >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x3d, (uint8_t)(TimingStr.vs));
	if (ret < 0)
		return ret;
	return 0;
}

int lt9211_set_timing_para_to_RGB(void)
{
	int ret = 0;

	//Htotal
	ret = x2_write_lt9211(0xff, 0x85);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x20, (uint8_t)(TimingStr.hact >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x21, (uint8_t)(TimingStr.hact));
	if (ret < 0)
		return ret;
	//HFP
	ret = x2_write_lt9211(0x22, (uint8_t)(TimingStr.hfp >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x23, (uint8_t)(TimingStr.hfp));
	if (ret < 0)
		return ret;
	//HSW
	ret = x2_write_lt9211(0x24, (uint8_t)(TimingStr.hs >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x25, (uint8_t)(TimingStr.hs));
	if (ret < 0)
		return ret;
	//VFP
	ret = x2_write_lt9211(0x38, (uint8_t)(TimingStr.vfp >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x39, (uint8_t)(TimingStr.vfp));
	if (ret < 0)
		return ret;
	//VSW
	ret = x2_write_lt9211(0x3c, (uint8_t)(TimingStr.vs >> 8));
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x3d, (uint8_t)(TimingStr.vs));
	if (ret < 0)
		return ret;
	return 0;
}

int lt9211_mipi_tx_phy(void)
{
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x62, 0x00);//ttl output disable
	if (ret < 0)
		return ret;
	//mipi 0x32:Port A/B Enable 0x12:portB 0x22:port A
	ret = x2_write_lt9211(0x3b, 0x32);
	//mipi 0x32:Port A/B Enable 0x12:portB 0x22:port A
	if (ret < 0)
		return ret;
	return 0;
}

int lt9211_RGB_tx_phy(void)
{
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;
	if ((LT9211_OutPutModde == OUTPUT_RGB888) ||
			(LT9211_OutPutModde == OUTPUT_BT656_8BIT) ||
			(LT9211_OutPutModde == OUTPUT_BT1120_16BIT)) {
		ret = x2_write_lt9211(0x62, 0x01);//ttl output enable
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x6b, 0xff);
		if (ret < 0)
			return ret;
	} else if ((LT9211_OutPutModde == OUTPUT_LVDS_2_PORT) ||
			(LT9211_OutPutModde == OUTPUT_LVDS_1_PORT)) {
		/* dual-port lvds tx phy */
		ret = x2_write_lt9211(0x62, 0x00);//ttl output disable
		if (ret < 0)
			return ret;
		if (LT9211_OutPutModde == OUTPUT_LVDS_2_PORT) {
			ret = x2_write_lt9211(0x3b, 0xb8);
			if (ret < 0)
				return ret;
		} else {
			ret = x2_write_lt9211(0x3b, 0x38);
			if (ret < 0)
				return ret;
		}
		// HDMI_WriteI2C_Byte(0x3b,0xb8); //dual port lvds enable
		ret = x2_write_lt9211(0x3e, 0x92);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x3f, 0x48);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x40, 0x31);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x43, 0x80);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x44, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x45, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x49, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x4a, 0x01);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x4e, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x4f, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x50, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x53, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x54, 0x01);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0xff, 0x81);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x20, 0x7b);
		if (ret < 0)
			return ret;
		//mlrx mltx calib reset
		ret = x2_write_lt9211(0x20, 0xff);
		if (ret < 0)
			return ret;
	}
	return 0;
}

int lt9211_mipi_tx_pll(void)
{
	uint32_t pixelclk, pixelclk0, pixelclk1;
	uint8_t pixelclk0_8, pixelclk1_8, pixelclk_8;
	uint8_t byteclk;
	uint8_t loopx;
	uint8_t value;
	uint8_t read_result_reg1f = 0;
	uint8_t read_result_reg20 = 0;
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x36, 0x03);//b7:txpll_pd
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0x38, 0x46);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x39, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x3a, 0x00);
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x00, 0x14);
	if (ret < 0)
		return ret;
	msleep(100);

	ret = x2_read_lt9211(0x08, &pixelclk0_8);
	if (ret < 0)
		return ret;

	pixelclk0_8 = pixelclk0_8 & 0x0f;
	pixelclk0 = ((uint32_t)pixelclk0_8) << 8;
	ret = x2_read_lt9211(0x09, &pixelclk1_8);
	if (ret < 0)
		return ret;
	pixelclk1 = pixelclk0 + (uint32_t)pixelclk1_8;
	pixelclk1 = pixelclk1 << 8;
	ret = x2_read_lt9211(0x0a, &pixelclk_8);
	if (ret < 0)
		return ret;
	pixelclk = pixelclk1 + (uint32_t)pixelclk_8;

	byteclk = pixelclk/1000;
	value = (byteclk*8)/25;

	ret = x2_write_lt9211(0xff, 0x87);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x37, value);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x38, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x39, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x13, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x13, 0x80);
	if (ret < 0)
		return ret;
	msleep(100);
	//Check Tx PLL cal
	for (loopx = 0; loopx < 10; loopx++) {
		ret = x2_write_lt9211(0xff, 0x87);
		if (ret < 0)
			return ret;
		ret = x2_read_lt9211(0x09, &read_result_reg1f);
		if (ret < 0)
			return ret;
		if (read_result_reg1f & 0x80) {
			ret = x2_read_lt9211(0x09, &read_result_reg20);
			if (ret < 0)
				return ret;
			if (read_result_reg20 & 0x80)
				pr_info("\nLT9211 tx pll lock");
			else
				pr_info("\nLT9211 tx pll unlocked");
			pr_info("\nLT9211 tx pll cal done");
			break;
		}
		pr_info("\nLT9211 tx pll unlocked");
	}
	return 0;
}

int lt9211_RGB_tx_pll(void)
{
	int ret = 0;
	uint8_t loopx;
	uint8_t read_result_reg1f, read_result_reg20;

	if (LT9211_OutPutModde == OUTPUT_BT656_8BIT) {
		ret = x2_write_lt9211(0xff, 0x82);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x2d, 0x40);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x30, 0x50);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x33, 0x55);
		if (ret < 0)
			return ret;
	} else if ((LT9211_OutPutModde == OUTPUT_LVDS_2_PORT) ||
			(LT9211_OutPutModde == OUTPUT_LVDS_1_PORT) ||
			(LT9211_OutPutModde == OUTPUT_RGB888) ||
			(LT9211_OutPutModde == OUTPUT_BT1120_16BIT)) {
		ret = x2_write_lt9211(0xff, 0x82);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x36, 0x01);
		//b7:txpll_pd
		if (ret < 0)
			return ret;
		if (LT9211_OutPutModde == OUTPUT_LVDS_1_PORT) {
			ret = x2_write_lt9211(0x37, 0x29);
			if (ret < 0)
				return ret;
		} else {
			ret = x2_write_lt9211(0x37, 0x2a);
			if (ret < 0)
				return ret;
		}
		ret = x2_write_lt9211(0x38, 0x06);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x39, 0x30);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x3a, 0x8e);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0xff, 0x87);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x37, 0x14);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x13, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x13, 0x80);
		if (ret < 0)
			return ret;
		msleep(100);
		//check tx pll cal
		for (loopx = 0; loopx < 10; loopx++) {
			ret = x2_write_lt9211(0xff, 0x87);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x1f, &read_result_reg1f);
			if (ret < 0)
				return ret;
			if (read_result_reg1f & 0x80) {
				ret = x2_read_lt9211(0x20, &read_result_reg20);
				if (ret < 0)
					return ret;
				if (read_result_reg20 & 0x80)
					pr_info("LT9211 tx pll lock!\n");
				else
					pr_info("LT9211 tx pll unlocked!\n");
				pr_info("LT9211 tx pll cal done!\n");
				break;
			}
			pr_info("LT9211 tx pll unlocked\n");
		}
	}
	pr_info("system success.\n");
	return 0;
}

int lt9211_set_tx_timing(void)
{
	int ret = 0;
	uint8_t sync_polarity;
	uint16_t hact, vact;
	uint8_t hact_h, hact_l, vact_h, vact_l;
	uint16_t hs;
	uint8_t vs;
	uint8_t hs_h, hs_l;
	uint16_t hbp;
	uint8_t vbp;
	uint8_t hbp_h, hbp_l;
	uint16_t htotal, vtotal;
	uint8_t htotal_h, htotal_l, vtotal_h, vtotal_l;
	uint16_t hfp;
	uint8_t vfp;
	uint8_t hfp_h, hfp_l;
	struct video_timing TimingStr;

	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_read_lt9211(0x70, &sync_polarity);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}

	ret = x2_read_lt9211(0x71, &vs);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}

	ret = x2_read_lt9211(0x72, &hs_h);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_read_lt9211(0x73, &hs_l);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	hs = (((uint16_t)hs_h) << 8) + (uint16_t)hs_l;

	ret = x2_read_lt9211(0x74, &vbp);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_read_lt9211(0x75, &vfp);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}

	ret = x2_read_lt9211(0x76, &hbp_h);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_read_lt9211(0x77, &hbp_l);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	hbp = (((uint16_t)hbp_h) << 8) + (uint16_t)hbp_l;

	ret = x2_read_lt9211(0x78, &hfp_h);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_read_lt9211(0x79, &hfp_l);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	hfp = (((uint16_t)hfp_h) << 8) + (uint16_t)hfp_l;

	ret = x2_read_lt9211(0x7a, &vtotal_h);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_read_lt9211(0x7b, &vtotal_l);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	vtotal = (((uint16_t)vtotal_h) << 8) + (uint16_t)vtotal_l;

	ret = x2_read_lt9211(0x7c, &htotal_h);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_read_lt9211(0x7d, &htotal_l);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	htotal = (((uint16_t)htotal_h) << 8) + (uint16_t)htotal_l;

	ret = x2_read_lt9211(0x7e, &vact_h);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_read_lt9211(0x7f, &vact_l);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	vact = (((uint16_t)vact_h) << 8) + (uint16_t)vact_l;

	ret = x2_read_lt9211(0x80, &hact_h);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_read_lt9211(0x81, &hact_l);
	if (ret < 0) {
		pr_err("%s() Err read lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	hact = (((uint16_t)hact_h) << 8) + (uint16_t)hact_l;

	TimingStr.hact = hact;
	TimingStr.vact = vact;
	TimingStr.vs = vs;
	TimingStr.vbp = vbp;
	TimingStr.vfp = vfp;

	ret = x2_write_lt9211(0xff, 0xd4);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x04, 0x01);//hs[7:0] not care
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x05, 0x01);//hbp[7:0] not care
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x06, 0x01);//hfp[7:0] not care
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x07, (uint8_t)(TimingStr.hact >> 8));
	//hactive[15:8]
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x08, (uint8_t)(TimingStr.hact));
	//hactive[7:0]
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	//vfp[7:0]
	ret = x2_write_lt9211(0x09,
		(uint8_t)(TimingStr.vs) + (uint8_t)(TimingStr.vbp) - 1);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x0a, 0x00);//bit[3:0]:vbp[11:8]
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x0b, 0x01);//vbp[7:0]
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	//vcat[15:8]
	ret = x2_write_lt9211(0x0c, (uint8_t)(TimingStr.vact >> 8));
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	//vcat[7:0]
	ret = x2_write_lt9211(0x0d, (uint8_t)(TimingStr.vact));
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x0e, 0x00);//vfp[11:8]
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x0f, 0x00);//vfp[7:0]
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}

	pr_info("\nsync_polarity = %x", sync_polarity);
	pr_info("\nhfp, hs, hbp, hact, htotal = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		hfp, hs, hbp, hact, htotal);
	pr_info("\nvfp, vs, vbp, vact, vtotal = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		vfp, vs, vbp, vact, vtotal);
	return 0;
}

int lt9211_mipi_tx_digital(void)
{
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x40, 0x80);//Select MIPI TX
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	/*port src sel*/
	ret = x2_write_lt9211(0x41, 0x01);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x42, 0x23);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x43, 0x40);//pt0_tx_src_sel
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x44, 0x12);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x45, 0x34);//pt1_tx_src_scl
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}

	ret = x2_write_lt9211(0xff, 0xd4);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x16, 0x55);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x10, 0x01);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	//read byteclk change big, the value change
	ret = x2_write_lt9211(0x11, 0x50);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	//bit[5:4]:lane num, bit[2]:bllp,bit[1:0]:vid_mode
	ret = x2_write_lt9211(0x13, 0x13);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	//bit[5:4]:data typ,bit[2:0]:fmt sel 000:rgb888
	ret = x2_write_lt9211(0x14, 0x20);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x21, 0x00);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							__func__, ret);
		return ret;
	}
	return 0;
}

int lt9211_tx_digital_RGB(void)
{
	int ret = 0;

	pr_info("LT9211 outpud mode ");
	if (LT9211_OutPutModde == OUTPUT_RGB888) {
		pr_info("set to OUTPUT_RGB888\n");
		ret = x2_write_lt9211(0xff, 0x85);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x88, 0x50);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x60, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x6d, 0x03);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x6e, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0xff, 0x81);
		if (ret < 0)
			return ret;
		//bit7:ttltx_pixclk_en;bit6:ttltx_BT_clk_en
		ret = x2_write_lt9211(0x36, 0xc0);
		if (ret < 0)
			return ret;
	} else if (LT9211_OutPutModde == OUTPUT_BT656_8BIT) {
		pr_info("set to OUTPUT_BT656_8BIT\n");
		ret = x2_write_lt9211(0xff, 0x85);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x88, 0x40);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x60, 0x34);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x6d, 0x00);
		//0x08 YC SWAP
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x6e, 0x07);
		//low 16bit
		if (ret < 0)
			return ret;

		ret = x2_write_lt9211(0xff, 0x81);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x0d, 0xfd);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x0d, 0xff);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0xff, 0x81);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x36, 0xc0);
		//bit7: ttl tx_pixclk_en; bit6: ttl tx_BT_clk_en
		if (ret < 0)
			return ret;
	} else if (LT9211_OutPutModde == OUTPUT_BT1120_16BIT) {
		pr_info("set to OUTPUT_BT1120_16BIT\n");
		ret = x2_write_lt9211(0xff, 0x85);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x88, 0x40);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x60, 0x33);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x6d, 0x08);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x6e, 0x06);
		//high 16bit
		if (ret < 0)
			return ret;

		ret = x2_write_lt9211(0xff, 0x81);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x0d, 0xfd);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x0d, 0xff);
		if (ret < 0)
			return ret;
	} else if ((LT9211_OutPutModde == OUTPUT_LVDS_2_PORT) ||
			(LT9211_OutPutModde == OUTPUT_LVDS_1_PORT)) {
		pr_info("set to OUTPUT_LVDS\n");
		//lvds tx controller
		ret = x2_write_lt9211(0xff, 0x85);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x59, 0x50);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x5a, 0xaa);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x5b, 0xaa);
		if (ret < 0)
			return ret;
		if (LT9211_OutPutModde == OUTPUT_LVDS_2_PORT) {
			//lvds tx port sel 01:dual; 00:single
			ret = x2_write_lt9211(0x5c, 0x01);
			if (ret < 0)
				return ret;
		} else {
			ret = x2_write_lt9211(0x5c, 0x00);
			if (ret < 0)
				return ret;
		}
		ret = x2_write_lt9211(0x88, 0x50);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0xa1, 0x77);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0xff, 0x86);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x40, 0x40);//tx_src_sel
		if (ret < 0)
			return ret;
		//port src sel
		ret = x2_write_lt9211(0x41, 0x34);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x42, 0x10);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x43, 0x23);//pt0_tx_src_sel
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x44, 0x41);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x45, 0x02);//pt1_tx_src_sel
		if (ret < 0)
			return ret;

#ifdef lvds_format_JEIDA
		ret = x2_write_lt9211(0xff, 0x85);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x59, 0xd0);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0xff, 0xd8);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x11, 0x40);
		if (ret < 0)
			return ret;
#endif
	}
	return 0;
}

int lt9211_rx_csc(void)
{
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0xf9);
	if (ret < 0)
		return ret;
	if (LT9211_OutPutModde == OUTPUT_RGB888) {
		if (Video_Input_Mode == Input_RGB888) {
			ret = x2_write_lt9211(0x86, 0x00);
			if (ret < 0)
				return ret;
			ret = x2_write_lt9211(0x87, 0x00);
			if (ret < 0)
				return ret;
		} else if (Video_Input_Mode == Input_YCbCr444) {
			ret = x2_write_lt9211(0x86, 0x0f);
			if (ret < 0)
				return ret;
			ret = x2_write_lt9211(0x87, 0x00);
			if (ret < 0)
				return ret;
		} else if (Video_Input_Mode == Input_YCbCr422_16BIT) {
			ret = x2_write_lt9211(0x86, 0x00);
			if (ret < 0)
				return ret;
			ret = x2_write_lt9211(0x87, 0x03);
			if (ret < 0)
				return ret;
		}
	} else if ((LT9211_OutPutModde == OUTPUT_BT656_8BIT) ||
			(LT9211_OutPutModde == OUTPUT_BT1120_16BIT) ||
			(LT9211_OutPutModde == OUTPUT_YCbCr422_16BIT)) {
		if (Video_Input_Mode == Input_RGB888) {
			ret = x2_write_lt9211(0x86, 0x0f);
			if (ret < 0)
				return ret;
			ret = x2_write_lt9211(0x87, 0x30);
			if (ret < 0)
				return ret;
		} else if (Video_Input_Mode == Input_YCbCr444) {
			ret = x2_write_lt9211(0x86, 0x00);
			if (ret < 0)
				return ret;
			ret = x2_write_lt9211(0x87, 0x30);
			if (ret < 0)
				return ret;
		} else if (Video_Input_Mode == Input_YCbCr422_16BIT) {
			ret = x2_write_lt9211(0x86, 0x00);
			if (ret < 0)
				return ret;
			ret = x2_write_lt9211(0x87, 0x00);
			if (ret < 0)
				return ret;
		}
	} else if (LT9211_OutPutModde == OUTPUT_YCbCr444) {
		if (Video_Input_Mode == Input_RGB888) {
			ret = x2_write_lt9211(0x86, 0x0f);
			if (ret < 0)
				return ret;
			ret = x2_write_lt9211(0x87, 0x00);
			if (ret < 0)
				return ret;
		} else if (Video_Input_Mode == Input_YCbCr444) {
			ret = x2_write_lt9211(0x86, 0x00);
			if (ret < 0)
				return ret;
			ret = x2_write_lt9211(0x87, 0x00);
			if (ret < 0)
				return ret;
		} else if (Video_Input_Mode == Input_YCbCr422_16BIT) {
			ret = x2_write_lt9211(0x86, 0x00);
			if (ret < 0)
				return ret;
			ret = x2_write_lt9211(0x87, 0x03);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

