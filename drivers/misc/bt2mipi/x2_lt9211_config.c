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

//#define MIPI_PATTEN

struct video_timing video_384x292_60Hz = {
	6, 5, 5, 384, 400, 4, 2, 2, 292, 300, 3000};
//struct video_timing video_800x480_60Hz = {
//	40, 48, 40, 800, 928, 12, 3, 30, 480, 525, 29232};
struct video_timing video_800x480_60Hz = {
	100, 48, 52, 800, 1000, 42, 2, 31, 480, 555, 33300};
//      hfp  hs  hbp hact vtotalvfp vs vbp vact vtotal clk
//struct video_timing video_800x480_60Hz = {
//	100, 46, 52, 800, 1000, 42, 4, 31, 480, 555, 33300};

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

struct video_timing video = {
	48, 24, 24, 720, 816, 12, 2, 10, 1280, 1304, 71980};
struct video_timing video_720x1280_60Hz = {
	72, 24, 24, 720, 840, 12, 2, 10, 1280, 1304, 71980};


struct video_timing TimingStr;
enum VideoFormat Video_Format;

//extern int display_type;
//extern int lt9211_reset_pin;
//extern int lcd_reset_pin;
//extern int lcd_pwm_pin;

//extern struct x2_lt9211_s  *g_x2_lt9211;
int set_lt9211_output_ctrl(unsigned int on_off)
{
	uint8_t read_value;
	int ret;

	if (display_type == LCD_7_TYPE) {
		if (on_off == 0) {
			pr_debug("lt9211 rgb output off!\n");
			ret = x2_write_lt9211(0xff, 0x82);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x62, &read_value);
			if (ret < 0)
				return ret;
			read_value = read_value & 0xfe;
			ret = x2_write_lt9211(0x62, read_value);
			if (ret < 0)
				return ret;
		} else {
			pr_debug("lt9211 rgb output on!\n");
			ret = x2_write_lt9211(0xff, 0x82);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x62, &read_value);
			if (ret < 0)
				return ret;
			read_value = read_value | 0x01;
			ret = x2_write_lt9211(0x62, read_value);
			if (ret < 0)
				return ret;
		}
	} else if (display_type == MIPI_720P) {
		if (on_off == 0) {
			pr_debug("lt9211 mipi output off!\n");
			ret = x2_write_lt9211(0xff, 82);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x3B, &read_value);
			if (ret < 0)
				return ret;
			read_value = read_value & 0xbd;
			ret = x2_write_lt9211(0x3b, read_value);
			if (ret < 0)
				return ret;
		} else {
			pr_debug("lt9211 mipi output on!\n");
			ret = x2_write_lt9211(0xff, 82);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x3B, &read_value);
			if (ret < 0)
				return ret;
			read_value = read_value | 0x42;
			ret = x2_write_lt9211(0x3b, read_value);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

int set_lt9211_config(struct fb_info *fb, unsigned int convert_type)
{
	int ret = 0;
	uint8_t value_4f = 0;
	struct video_timing timing;
	uint8_t sync_polarity = 0;
	uint8_t vs = 0;
	uint8_t hs_l, hs_h = 0;
	uint8_t vbp = 0;
	uint8_t vfp = 0;
	uint8_t hbp_l, hbp_h = 0;
	uint8_t hfp_l, hfp_h = 0;
	uint8_t vtotal_l, vtotal_h = 0;
	uint8_t htotal_l, htotal_h = 0;
	uint8_t vact_l, vact_h = 0;
	uint8_t hact_l, hact_h = 0;
	uint16_t hs, hbp, hfp, vtotal, htotal, vact, hact;

	pr_info("Framebuffer begin set lt9211 config!!!\n");
	//ret = lt9211_reset_first();
	//if (ret) {
	//	pr_err("Err reset lt9211!!\n");
	//	return ret;
	//}

	if (convert_type == BT1120_TO_RGB888) {
/*		ret = lt9211_chip_id();
		if (ret) {
			pr_err("%s() Err get lt9211 chip id ret= %d\n",
						__func__, ret);
			return ret;
		}
*/
		ret = lt9211_system_int_to_RGB();
		if (ret) {
			pr_err("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_ttl_rx_phy_to_RGB();
		if (ret) {
			pr_err("%s() Err init lt9211 system ret= %d\n",
						__func__, ret);
			return ret;
		}
	//	ret = x2_write_lt9211(0xff, 0x85);
	//	ret = x2_read_lt9211(0x4f, &value_4f);
	//	LT9211_DEBUG("value 0x4f is 0x%x.\n", value_4f);
	//	if(value_4f != 0x40){
	//		while(1);
	//	}

//		msleep(500);

		TimingStr = video_800x480_60Hz;
		lt9211_set_timing_para_to_RGB();

//		msleep(500);
		msleep(20);
		//msleep(500);
		//800x480 timing parameter
		//hfp:40 hs:48 hbp:40 hact:800 htotal:928 vfp:12 vs:3
		//vbp:30 vact:480 vtotal:525 pclk:29232
#ifdef CHECK_TIMING
		//video check debug
		x2_write_lt9211(0xff, 0x86);
		x2_write_lt9211(0x20, 0x00);
		msleep(50);
		//msleep(500);
		x2_read_lt9211(0x70, &sync_polarity);
		LT9211_DEBUG("sync_polarity is %d.\n", sync_polarity);

		x2_read_lt9211(0x71, &vs);
		LT9211_DEBUG("vs is %d(2)", vs);

		x2_read_lt9211(0x72, &hs_h);
		x2_read_lt9211(0x73, &hs_l);
		hs = (((uint16_t)hs_h) << 8) + (uint16_t)hs_l;
		LT9211_DEBUG("hs is %d(48).\n", hs);

		x2_read_lt9211(0x74, &vbp);
		LT9211_DEBUG("vbp is %d(31)", vbp);

		x2_read_lt9211(0x75, &vfp);
		LT9211_DEBUG("vfp is %d(42)", vfp);

		x2_read_lt9211(0x76, &hbp_h);
		x2_read_lt9211(0x77, &hbp_l);
		hbp = (((uint16_t)hbp_h) << 8) + (uint16_t)hbp_l;
		LT9211_DEBUG("hbp is %d(52).\n", hbp);

		x2_read_lt9211(0x78, &hfp_h);
		x2_read_lt9211(0x79, &hfp_l);
		hfp = (((uint16_t)hfp_h) << 8) + (uint16_t)hfp_l;
		LT9211_DEBUG("hfp is %d(100).\n", hfp);

		x2_read_lt9211(0x7a, &vtotal_h);
		x2_read_lt9211(0x7b, &vtotal_l);
		vtotal = (((uint16_t)vtotal_h) << 8) + (uint16_t)vtotal_l;
		LT9211_DEBUG("vtotal is %d(555).\n", vtotal);

		x2_read_lt9211(0x7c, &htotal_h);
		x2_read_lt9211(0x7d, &htotal_l);
		htotal = (((uint16_t)htotal_h) << 8) + (uint16_t)htotal_l;
		LT9211_DEBUG("htotal is %d(1000).\n", htotal);

		x2_read_lt9211(0x7e, &vact_h);
		x2_read_lt9211(0x7f, &vact_l);
		vact = (((uint16_t)vact_h) << 8) + (uint16_t)vact_l;
		LT9211_DEBUG("vact is %d(480).\n", vact);

		x2_read_lt9211(0x80, &hact_h);
		x2_read_lt9211(0x81, &hact_l);
		hact = (((uint16_t)hact_h) << 8) + (uint16_t)hact_l;
		LT9211_DEBUG("hact is %d(800).\n", hact);


	//	lt9211r_patten(&video_800x480_60Hz);
		//msleep(100);
#endif
		ret = lt9211_tx_digital_RGB();
		if (ret) {
			pr_err("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_RGB_tx_phy();
		if (ret) {
			pr_err("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		//msleep(10);
		usleep_range(10000, 11000);
		ret = lt9211_RGB_tx_pll();
		if (ret) {
			pr_err("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
	//	while(1);
		ret = lt9211_rx_csc();
		if (ret) {
			pr_err("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
	} else if (convert_type == BT1120_TO_MIPI) {
#ifdef MIPI_PATTEN
		ret = lt9211_system_int_to_mipi_pattern();
		if (ret) {
			pr_err("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}

//		lt9211_csc_to_mipi();

		lt9211_mipi_patten(&video);

		ret = lt9211_mipi_tx_pll();
		if (ret) {
			pr_err("%s:Err config lt9211 mipi tx pll ret= %d\n",
					__func__, ret);
			return ret;
		}

		ret = lt9211_mipi_tx_phy();
		if (ret) {
			pr_err("%s:Err config lt9211 mipi tx phy ret= %d\n",
					__func__, ret);
			return ret;
		}

		ret = lt9211_set_mipi_tx_timing(&video);
		if (ret) {
			pr_err("%s:Err set lt9211 tx timing ret= %d\n",
					__func__, ret);
			return ret;
		}

		ret = init_panel(display_type);
		if (ret) {
			pr_err("%s:Err init dsi panel ret= %d\n",
					__func__, ret);
			return ret;
		}

		ret = lt9211_mipi_tx_digital();
		if (ret) {
			pr_err("%sErr config mipi tx digital ret= %d\n",
					__func__, ret);
			return ret;
		}

//		ret = lt9211_mipi_clock_debug();
//		if (ret) {
//			printk(KERN_ERR"%s() Err config lt9211 mipi clock debug
//			ret= %d\n", __func__, ret);
//			return ret;
//		}

#else
		pr_debug("%s: 1.begin lt9211 system int to mipi\n",
							__func__);
		//ret = lt9211_system_int_to_RGB();
		ret = lt9211_system_int_to_mipi();
		if (ret) {
			pr_err("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		pr_debug("%s: 2.begin lt9211 ttl rx phy to mipi\n", __func__);
		ret = lt9211_ttl_rx_phy_to_mipi();
		if (ret) {
			pr_err("%s() Err config lt9211 ttl rx phy ret= %d\n",
					__func__, ret);
			return ret;
		}
		pr_debug("%s: 3.begin lt9211 video check to mipi\n", __func__);
		ret = lt9211_video_check_to_mipi();
		if (ret) {
			pr_err("%s() Err check lt9211 video ret= %d\n",
					__func__, ret);
			return ret;
		}
/*
		ret = lt9211_csc_to_mipi();
		if (ret) {
			LT9211_DEBUG("%s() Err config lt9211 csc ret= %d\n",
					__func__, ret);
			return ret;
		}
*/
		pr_debug("%s: 4.begin lt9211 set timing para to mipi\n",
							__func__);
		ret = lt9211_set_timing_para_to_mipi();
		if (ret) {
			pr_err("%s:Err set timing parameter ret= %d\n",
							__func__, ret);
			return ret;
		}
		pr_debug("%s: 5.begin lt9211 mipi clock debug\n", __func__);
		ret = lt9211_mipi_clock_debug();
		if (ret) {
			pr_err("%s:Err mipi clock debug ret= %d\n",
						__func__, ret);
			return ret;
		}
/*
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
*/
		pr_debug("%s: 6.begin lt9211 mipi tx phy\n", __func__);
		ret = lt9211_mipi_tx_phy();
		if (ret) {
			pr_err("%s() Err config lt9211 mipi tx phy ret= %d\n",
					__func__, ret);
			return ret;
		}
		pr_debug("%s: 7.begin lt9211 mipi tx pll\n", __func__);
		ret = lt9211_mipi_tx_pll();
		if (ret) {
			pr_err("%s() Err config lt9211 mipi tx pll ret= %d\n",
					__func__, ret);
			return ret;
		}
		pr_debug("%s: 8.begin lt9211 set tx timing\n", __func__);
		ret = lt9211_set_tx_timing();
		if (ret) {
			pr_err("%s() Err set lt9211 tx timing ret= %d\n",
					__func__, ret);
			return ret;
		}

		pr_debug("%s: 9.begin init panel\n", __func__);
		ret = init_panel(display_type);
		if (ret) {
			pr_err("%s() Err init dsi panel ret= %d\n",
					__func__, ret);
			return ret;
		}
		pr_debug("%s: 10.begin lt9211 mipi tx digital\n", __func__);
		ret = lt9211_mipi_tx_digital();
		if (ret) {
			pr_err("%s() Err config lt9211 mipi tx digital ret= %d\n",
					__func__, ret);
			return ret;
		}
		pr_debug("%s: 11.begin lt9211 csc to mipi\n", __func__);
		ret = lt9211_csc_to_mipi();
		if (ret) {
			pr_err("%s() Err config lt9211 csc ret= %d\n",
					__func__, ret);
			return ret;
		}
#endif
	}

	LT9211_DEBUG("Framebuffer config lt9211 on line end!!!\n");
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

	ret = lt9211_reset_first();
	if (ret) {
		pr_err("Err reset lt9211!!\n");
		return ret;
	}
//	lt9211_config(convert_type);
	return 0;
}

int lt9211_reset(void)
{
	int ret;

//	ret = gpio_request(85, "x2_lt9211_reset_pin");
//	if (ret) {
//		LT9211_DEBUG("%s() Err get trigger pin ret= %d\n",
//					__func__, ret);
//		return -ENODEV;
//	}
	LT9211_DEBUG("gpio request 85 succeed!!!!");
	gpio_direction_output(lt9211_reset_pin, 0);
	LT9211_DEBUG("lt9211 reset pin output low!!!!!\n");
	msleep(1000);
	gpio_direction_output(lt9211_reset_pin, 1);
	LT9211_DEBUG("lt9211 reset pin output high!!!!\n");
	msleep(1000);
	LT9211_DEBUG("lt9211 reset again success!\n");
	return 0;
}

int lt9211_reset_first(void)
{
	int ret;

	ret = gpio_request(lt9211_reset_pin, "x2_lt9211_reset_pin");
	if (ret) {
		LT9211_DEBUG("%s() Err get trigger pin ret= %d\n",
					__func__, ret);
		return -ENODEV;
	}
	LT9211_DEBUG("gpio request 85 succeed!!!!");
	gpio_direction_output(lt9211_reset_pin, 0);
	LT9211_DEBUG("lt9211 reset pin output low!!!!!\n");
	msleep(100);
	gpio_direction_output(lt9211_reset_pin, 1);
	LT9211_DEBUG("lt9211 reset pin output high!!!!\n");
	msleep(100);

	LT9211_DEBUG("lt9211 reset first success!!!\n");
	return 0;
}

int lt9211_config(unsigned int convert_type)
{
	int ret = 0;

	LT9211_DEBUG("lt9211 config begin!!!\n");
	ret = lt9211_chip_id();
	if (ret) {
		LT9211_DEBUG("%s() Err get lt9211 chip id ret= %d\n",
					__func__, ret);
		return ret;
	}
	if (convert_type == BT1120_TO_MIPI) {
		ret = lt9211_system_int_to_mipi();
		if (ret) {
			LT9211_DEBUG("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_ttl_rx_phy_to_mipi();
		if (ret) {
			LT9211_DEBUG("%s() Err config lt9211 ttl rx phy ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_csc_to_mipi();
		if (ret) {
			LT9211_DEBUG("%s() Err config lt9211 csc ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_video_check_to_mipi();
		if (ret) {
			LT9211_DEBUG("%s() Err check lt9211 video ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_set_timing_para_to_mipi();
		if (ret) {
			LT9211_DEBUG("%s() Err set lt9211 timing parameter ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_mipi_tx_phy();
		if (ret) {
			LT9211_DEBUG("%s() Err config lt9211 mipi tx phy ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_mipi_tx_pll();
		if (ret) {
			LT9211_DEBUG("%s() Err config lt9211 mipi tx pll ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = init_panel(display_type);
		if (ret) {
			LT9211_DEBUG("%s() Err init dsi panel ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_set_tx_timing();
		if (ret) {
			LT9211_DEBUG("%s() Err set lt9211 tx timing ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_mipi_tx_digital();
		if (ret) {
			LT9211_DEBUG("%s() Err config lt9211 mipi tx digital ret= %d\n",
					__func__, ret);
			return ret;
		}
	} else if (convert_type == BT1120_TO_RGB888) {
		LT9211_DEBUG("begin config lt9211, config bt1120 convert to RGB888!!!\n");
		//TimingStr = video_720x480_60Hz;
		ret = lt9211_system_int_to_RGB();
		if (ret) {
			LT9211_DEBUG("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_ttl_rx_phy_to_RGB();
		if (ret) {
			LT9211_DEBUG("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_BT_video_check_to_RGB();
		if (ret) {
			LT9211_DEBUG("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_set_timing_para_to_RGB();
		if (ret) {
			LT9211_DEBUG("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		msleep(100);
		ret = lt9211_tx_digital_RGB();
		if (ret) {
			LT9211_DEBUG("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_RGB_tx_phy();
		if (ret) {
			LT9211_DEBUG("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		msleep(20);
		ret = lt9211_RGB_tx_pll();
		if (ret) {
			LT9211_DEBUG("%s() Err init lt9211 system ret= %d\n",
					__func__, ret);
			return ret;
		}
		ret = lt9211_rx_csc();
		if (ret) {
			LT9211_DEBUG("%s() Err init lt9211 system ret= %d\n",
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
	LT9211_DEBUG("\nLT9211 Chip ID:%x,", chip_id[0]);
	ret = x2_read_lt9211(0x01, &chip_id[1]);
	if (ret < 0)
		return ret;
	LT9211_DEBUG("%x,", chip_id[1]);
	ret = x2_read_lt9211(0x02, &chip_id[2]);
	if (ret < 0)
		return ret;
	LT9211_DEBUG("%x\n", chip_id[2]);
	if ((chip_id[0] == 0x18) && (chip_id[1] == 0x1) && (chip_id[2] == 0xe1))
		return 0;
	else
		return -1;
}

int lt9211_system_int_to_mipi_pattern(void)
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
/*	ret = x2_write_lt9211(0xff, 0x81);//clock gating
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
*/
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

	ret = x2_write_lt9211(0xff, 0x81);//rpt reset
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x0b, 0xfe);
	if (ret < 0)
		return ret;

	return 0;
}

int lt9211_system_int_to_RGB(void)
{
	int ret = 0;

	LT9211_DEBUG("step1:begin to config lt9211 system int to RGB!!!\n");
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
	LT9211_DEBUG("step1:end to config lt9211 system int to RGB!!!\n");
	return 0;
}
/*
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
*/
int lt9211_ttl_rx_phy_to_mipi(void)
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
	ret = x2_write_lt9211(0x63, 0x00);
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0xff, 0x85);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x88, 0x90);
	if (ret < 0)
		return ret;
	//ret = x2_write_lt9211(0x45, 0x70);
	ret = x2_write_lt9211(0x45, 0x60);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x48, 0x18);
	if (ret < 0)
		return ret;

	return 0;
}



int lt9211_ttl_rx_phy_to_RGB(void)
{
	int ret = 0;

	LT9211_DEBUG("step2:begin to config lt9211 ttl rx phy to RGB!!!\n");
	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x28, 0x40);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x61, 0x09);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x63, 0x00);
	if (ret < 0)
		return ret;

	//Data mapping
	ret = x2_write_lt9211(0xff, 0x85);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x88, 0x90);
	if (ret < 0)
		return ret;
//	ret = x2_write_lt9211(0x45, 0x70);
//	if (ret < 0)
//		return ret;
	ret = x2_write_lt9211(0x45, 0x60);
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0x48, 0x18);//16BIT
	if (ret < 0)
		return ret;
	LT9211_DEBUG("step2:end to config lt9211 ttl rx phy to RGB!!!\n");
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
	ret = x2_write_lt9211(0x91, 0x03);
	if (ret < 0)
		return ret;
	return 0;
}

/*
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

		LT9211_DEBUG("\nlt9211 video chech debug: hact = 0x%x",
									hact);
		LT9211_DEBUG("\nlt9211 video chech debug:vact = 0x%x", vact);
		LT9211_DEBUG("\nlt9211 video chech debug:htotal = 0x%x",
									htotal);

		if ((hact == video_1920x1080_60Hz.hact) &&
				(vact == video_1920x1080_60Hz.vact) &&
				(htotal == video_1920x1080_60Hz.htotal)) {
			LT9211_DEBUG("\nVideo_check = video_1920x1080_60Hz");
			Video_Format = video_1920x1080_60Hz_vic;
			TimingStr = video_1920x1080_60Hz;
			break;
		}
		if ((hact == video_640x480_60Hz.hact) &&
				(vact == video_640x480_60Hz.vact) &&
				(htotal == video_640x480_60Hz.htotal)) {
			LT9211_DEBUG("\nVideo_check = video_640x480_60Hz");
			Video_Format = video_640x480_60Hz_vic;
			TimingStr = video_640x480_60Hz;
			break;
		}
		LT9211_DEBUG("\nVideo_check None\n");
		Video_Format = video_none;
		msleep(2000);
	}
	return 0;
}
*/
int lt9211_video_check_to_mipi(void)
{
	uint16_t hact, vact;
	uint8_t hact_h, hact_l, vact_h, vact_l;
	uint16_t htotal;
	uint8_t htotal_h, htotal_l;
	uint8_t read_result;
	int ret = 0;

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
		ret = x2_read_lt9211(0x4f, &read_result);
		if (ret < 0)
			return ret;
		pr_debug("854F value is 0x%x\n", read_result);
		if (read_result & 0x40) {
			//BT1120 flag is set
			ret = x2_write_lt9211(0xff, 0x86);
			if (ret < 0)
				return ret;
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
			hact = hact - 4;
			ret = x2_read_lt9211(0x8f, &htotal_h);
			if (ret < 0)
				return ret;
			ret = x2_read_lt9211(0x90, &htotal_l);
			if (ret < 0)
				return ret;
			htotal = (((uint8_t)htotal_h) << 8) + (uint8_t)htotal_l;

			pr_debug("\nlt9211 video chech debug: hact = 0x%x",
					hact);
			pr_debug("\nlt9211 video chech debug:vact = 0x%x",
					vact);
			pr_debug("\nlt9211 video chech debug:htotal = 0x%x",
					htotal);

			if ((hact == video_720x1280_60Hz.hact) &&
					(vact == video_720x1280_60Hz.vact)) {
				pr_info("\nVideo_check = video_720x1280_60Hz");
				Video_Format = video_720x1280_60Hz_vic;
				TimingStr = video_720x1280_60Hz;
				break;
			}
			pr_info("\nVideo_check None\n");
			Video_Format = video_none;
			msleep(2000);
		}
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
		LT9211_DEBUG("lt9211 bt format flag is 0x%x\n", bt_flag);
		if (ret < 0)
			return ret;
		if (bt_flag & 0x40) {
			LT9211_DEBUG("\nBT1120 flag is set\n");
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
			LT9211_DEBUG("hact = 0x%x.\n", hact);
			LT9211_DEBUG("vact = 0x%x.\n", vact);
			LT9211_DEBUG("htotal = 0x%x.\n", htotal);

			if ((vact == video_1920x1080_25Hz.vact)
				&& (hact == video_1920x1080_25Hz.hact)
				&& (htotal == video_1920x1080_25Hz.htotal)) {
				LT9211_DEBUG("Video_check = video_1920x1080_25Hz.\n");
				Video_Format = video_1920x1080_25Hz_vic;
				TimingStr = video_1920x1080_25Hz;
				break;
			}
		}
	}
	return 0;
}
/*
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
*/
int lt9211_set_timing_para_to_mipi(void)
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

int lt9211_set_timing_para_to_RGB(void)
{
	int ret = 0;

	LT9211_DEBUG("step: begin lt9211 set timing para to RGB!!!\n");
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
	LT9211_DEBUG("step: end lt9211 set timing para to RGB!!!\n");
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
	ret = x2_write_lt9211(0x3b, 0x32);//mipi_en
	//mipi 0x32:Port A/B Enable 0x12:portB 0x22:port A
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x40, 0x80);//tx src sel
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0x41, 0x01);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x42, 0x23);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x43, 0x40);//PORTA mipi lane swap
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x44, 0x12);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x45, 0x34);//PORTB mipi lane swap
	if (ret < 0)
		return ret;

	return 0;
}

int lt9211_RGB_tx_phy(void)
{
	int ret = 0;

	LT9211_DEBUG("step: begin lt9211 RGB tx phy!!!\n");
	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;
	if ((LT9211_OutPutModde == OUTPUT_RGB888) ||
			(LT9211_OutPutModde == OUTPUT_BT656_8BIT) ||
			(LT9211_OutPutModde == OUTPUT_BT1120_16BIT)) {
		LT9211_DEBUG("Output mode is RGB888\n");
		ret = x2_write_lt9211(0x62, 0x01);//ttl output enable
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x63, 0xff);
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
	LT9211_DEBUG("step: end lt9211 RGB tx phy!!!\n");
	return 0;
}

/*
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
				LT9211_DEBUG("\nLT9211 tx pll lock");
			else
				LT9211_DEBUG("\nLT9211 tx pll unlocked");
			LT9211_DEBUG("\nLT9211 tx pll cal done");
			break;
		}
		LT9211_DEBUG("\nLT9211 tx pll unlocked");
	}
	return 0;
}
*/

int lt9211_mipi_tx_pll(void)
{
	int ret = 0;
	int loopx = 0;
	uint8_t read_result = 0;

	ret = x2_write_lt9211(0xff, 0x82);
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0x36, 0x03);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x37, 0x28);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x38, 0x04);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x3a, 0x93);
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0xff, 0x87);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x13, 0x00);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x13, 0x80);
	if (ret < 0)
		return ret;

	msleep(100);

	//check tx pll cal done
	for (loopx = 0; loopx < 10; loopx++) {
		x2_write_lt9211(0xff, 0x87);
		ret = x2_read_lt9211(0x1f, &read_result);
		if (read_result & 0x80) {
			ret = x2_read_lt9211(0x20, &read_result);
			if (read_result & 0x80)
				pr_debug("\r\nLT9211 tx pll lock");
			else
				pr_debug("\r\nLT9211 tx pll unlocked");
			pr_debug("\r\nLT9211 tx pll cal done");
			break;
		}
		pr_debug("\r\nLT9211 tx pll unlocked");
	}
	return 0;
}

int lt9211_RGB_tx_pll(void)
{
	int ret = 0;
	uint8_t loopx;
	uint8_t read_result_reg1f, read_result_reg20;

	LT9211_DEBUG("setp: begin config lt9211 RGB tx pll!!!\n");
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
		LT9211_DEBUG("Output mode is RGB888.\n");
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
					LT9211_DEBUG("LT9211 tx pll lock!\n");
				else
					LT9211_DEBUG("LT9211 tx pll unlocked!\n");
				LT9211_DEBUG("LT9211 tx pll cal done!\n");
				break;
			} else {
				LT9211_DEBUG("value of 0x1f & 0x80 = 0, lt9211 tx pll unlocked\n");
			}
		}
	}
	LT9211_DEBUG("system success.\n");
	LT9211_DEBUG("step: end config lt9211 rgb tx pll!!!\n");
	return 0;
}
/*
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

	LT9211_DEBUG("\nsync_polarity = %x", sync_polarity);
	LT9211_DEBUG("\nhfp,hs,hbp,hact,htotal = 0x%x,0x%x,0x%x,0x%x,0x%x",
		hfp, hs, hbp, hact, htotal);
	LT9211_DEBUG("\nvfp,vs,vbp,vact,vtotal = 0x%x,0x%x,0x%x,0x%x,0x%x",
		vfp, vs, vbp, vact, vtotal);
	return 0;
}
*/
int lt9211_set_tx_timing(void)
{
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0xd4);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x04, 0x08);//hs[7:0] not care
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x05, 0x08);//hbp[7:0] not care
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x06, 0x08);//hfp[7:0] not care
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
	ret = x2_write_lt9211(0x09, (uint8_t)(TimingStr.vs));
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
	ret = x2_write_lt9211(0x0b, (uint8_t)(TimingStr.vbp));//vbp[7:0]
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
	ret = x2_write_lt9211(0x0e, (uint8_t)(TimingStr.vfp >> 8));//vfp[11:8]
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x0f, (uint8_t)(TimingStr.vfp));//vfp[7:0]
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}

	return 0;
}


int lt9211_set_mipi_tx_timing(struct video_timing *video_format)
{
	uint16_t hact, vact;
	uint16_t hs, vs;
	uint16_t hbp, vbp;
	uint16_t htotal, vtotal;
	uint16_t hfp, vfp;
	//struct Timing TimingStr;
	struct video_timing TimingStr;

	vs = video_format->vs;
	hs = video_format->hs;
	vbp = video_format->vbp;
	vfp = video_format->vfp;
	hbp = video_format->hbp;
	hfp = video_format->hfp;
	vtotal = video_format->vtotal;
	htotal = video_format->htotal;
	vact = video_format->vact;
	hact = video_format->hact;

	TimingStr.hact = hact;
	TimingStr.vact = vact;
	TimingStr.vs = vs;
	TimingStr.vbp = vbp;
	TimingStr.vfp = vfp;

	x2_write_lt9211(0xff, 0xd4);
	x2_write_lt9211(0x04, 0x08); //hs[7:0] not care
	x2_write_lt9211(0x05, 0x08); //hbp[7:0] not care
	x2_write_lt9211(0x06, 0x08); //hfp[7:0] not care
	x2_write_lt9211(0x07, (uint8_t)(hact >> 8)); //hactive[15:8]
	x2_write_lt9211(0x08, (uint8_t)(hact)); //hactive[7:0]

	x2_write_lt9211(0x09, (uint8_t)(vs)); //vfp[7:0]
	x2_write_lt9211(0x0a, 0x00); //bit[3:0]:vbp[11:8]
	x2_write_lt9211(0x0b, (uint8_t)(vbp)); //vbp[7:0]
	x2_write_lt9211(0x0c, (uint8_t)(vact >> 8)); //vcat[15:8]
	x2_write_lt9211(0x0d, (uint8_t)(vact)); //vcat[7:0]
	x2_write_lt9211(0x0e, (uint8_t)(vfp >> 8)); //vfp[11:8]
	x2_write_lt9211(0x0f, (uint8_t)(vfp)); //vfp[7:0]

	return 0;
}

/*
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
	//port src sel
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
*/
int lt9211_mipi_tx_digital(void)
{
	int ret = 0;
	uint8_t read_result;

	ret = x2_write_lt9211(0xff, 0xd4);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x1c, 0x30);//Select MIPI TX
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	/*port src sel*/
	ret = x2_write_lt9211(0x1d, 0x0a);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x1e, 0x06);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x1f, 0x0a);//pt0_tx_src_sel
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


	ret = x2_write_lt9211(0xff, 0xd4);//pt1_tx_src_scl
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}

//	ret = x2_write_lt9211(0xff, 0xd4);
//	if (ret < 0) {
//		pr_err("%s() Err write lt9211 reg, ret= %d\n",
//							 __func__, ret);
//		return ret;
//	}
	ret = x2_write_lt9211(0x16, 0x55);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	//ret = x2_write_lt9211(0x10, 0x01);
	ret = x2_write_lt9211(0x10, 0x01);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
	//read byteclk change big, the value change
	//ret = x2_write_lt9211(0x11, 0x50);
	ret = x2_write_lt9211(0x11, 0x3b);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
#ifdef MIPI_PATTEN
	//bit[5:4]:lane num, bit[2]:bllp,bit[1:0]:vid_mode
	ret = x2_write_lt9211(0x13, 0x0f);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}
#else
	ret = x2_write_lt9211(0x12, 0x3e);
	if (ret < 0) {
		pr_err("%s() Err write lt9211 reg, ret= %d\n",
							 __func__, ret);
		return ret;
	}

#endif
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

#ifdef MIPI_PATTEN
#else
	x2_write_lt9211(0x13, MIPI_LaneNum);
	//bit[5:4]:lane num, bit[2]:bllp,bit[1:0]:vid_mode
	pr_debug("set mipi 4 lane\n");
	if (MIPI_VideoMode == MIPI_BurstMode) {
		pr_debug("Set to Burst Mode\n");
		x2_read_lt9211(0x13, &read_result);
		x2_write_lt9211(0x13, read_result | 0x0f);
		x2_write_lt9211(0x21, 0x01);
	} else if (MIPI_VideoMode == MIPI_NonBurst_SyncPulse_Mode) {
		pr_debug("Set to NonBurst SyncPulse Mode\n");
		x2_read_lt9211(0x13, &read_result);
		x2_write_lt9211(0x13, read_result | 0x01);
		x2_write_lt9211(0x21, 0x00);
	} else if (MIPI_VideoMode == MIPI_NonBurst_SyncEvent_Mode) {
		pr_debug("Set to NonBurst SyncEvent Mode\n");
		x2_read_lt9211(0x13, &read_result);
		x2_write_lt9211(0x13, read_result | 0x02);
		x2_write_lt9211(0x21, 0x00);
	}
#endif

	return 0;
}

int lt9211_tx_digital_RGB(void)
{
	int ret = 0;

	LT9211_DEBUG("step: begin config lt9211 tx digital RGB!!!\n");
	LT9211_DEBUG("LT9211 outpud mode ");
	if (LT9211_OutPutModde == OUTPUT_RGB888) {
		LT9211_DEBUG("set to OUTPUT_RGB888\n");
		ret = x2_write_lt9211(0xff, 0x85);
		if (ret < 0)
			return ret;
		//debug !!!!!!!!!!!!!!!!!!!!!!!!
		ret = x2_write_lt9211(0x88, 0x90);
		if (ret < 0)
			return ret;
//		ret = x2_write_lt9211(0x88, 0xc0);
//		if (ret < 0)
//			return ret;
		ret = x2_write_lt9211(0x60, 0x00);
		if (ret < 0)
			return ret;
		ret = x2_write_lt9211(0x6d, 0x00);//BGR sequence
		if (ret < 0)
			return ret;
//		ret = x2_write_lt9211(0x6e, 0x00);//normal
//		if (ret < 0)
//			return ret;
		ret = x2_write_lt9211(0x6e, 0x80);//swap b
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
		LT9211_DEBUG("set to OUTPUT_BT656_8BIT\n");
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
		LT9211_DEBUG("set to OUTPUT_BT1120_16BIT\n");
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
		LT9211_DEBUG("set to OUTPUT_LVDS\n");
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
	LT9211_DEBUG("step: end config lt9211 tx digital RGB!!!\n");
	return 0;
}

int lt9211_rx_csc(void)
{
	int ret = 0;

	LT9211_DEBUG("step: config lt9211 rx csc!!!\n");
	ret = x2_write_lt9211(0xff, 0xf9);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x90, 0x03);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x91, 0x03);
	if (ret < 0)
		return ret;

	LT9211_DEBUG("step: end config lt9211 rx csc!!!\n");
	return 0;
}

void lt9211r_patten(struct video_timing *video_format)
{
	uint32_t pclk_khz;
	uint8_t dessc_pll_post_div;
	uint32_t pcr_m, pcr_k;

	pclk_khz = video_format->pclk_khz;

	x2_write_lt9211(0xff, 0xf9);
	x2_write_lt9211(0x3e, 0x80);

	x2_write_lt9211(0xff, 0x85);
	x2_write_lt9211(0x88, 0xc0);

	x2_write_lt9211(0xa1, 0x64);
	x2_write_lt9211(0xa2, 0xff);

	x2_write_lt9211(0xa3,
		(uint8_t)((video_format->hs + video_format)));

	x2_write_lt9211(0xa3,
		(uint8_t)((video_format->hs + video_format->hbp) / 256));
	x2_write_lt9211(0xa4,
		(uint8_t)((video_format->hs + video_format->hbp) % 256));
	//h_start

	x2_write_lt9211(0xa5,
		(uint8_t)((video_format->vs + video_format->vbp) % 256));
	//v_start

	x2_write_lt9211(0xa6, (uint8_t)(video_format->hact / 256));
	x2_write_lt9211(0xa7, (uint8_t)(video_format->hact % 256));
	//hactive

	x2_write_lt9211(0xa8, (uint8_t)(video_format->vact / 256));
	x2_write_lt9211(0xa9, (uint8_t)(video_format->vact % 256));
	//vactive

	x2_write_lt9211(0xaa, (uint8_t)(video_format->htotal / 256));
	x2_write_lt9211(0xab, (uint8_t)(video_format->htotal % 256));
	//htotal

	x2_write_lt9211(0xac, (uint8_t)(video_format->vtotal / 256));
	x2_write_lt9211(0xad, (uint8_t)(video_format->vtotal % 256));
	//vtotal

	x2_write_lt9211(0xae, (uint8_t)(video_format->hs / 256));
	x2_write_lt9211(0xaf, (uint8_t)(video_format->hs % 256));
	//hsa

	x2_write_lt9211(0xb0, (uint8_t)(video_format->vs % 256));
	//vsa
	//dessc pll to generate pixel clk
	x2_write_lt9211(0xff, 0x82);//dessc pll
	x2_write_lt9211(0x2d, 0x48);//pll ref select xtal

	if (pclk_khz < 44000) {
		x2_write_lt9211(0x35, 0x83);
		dessc_pll_post_div = 16;
	} else if (pclk_khz < 88000) {
		x2_write_lt9211(0x35, 0x82);
		dessc_pll_post_div = 8;
	} else if (pclk_khz < 176000) {
		x2_write_lt9211(0x35, 0x81);
		dessc_pll_post_div = 4;
	} else if (pclk_khz < 352000) {
		x2_write_lt9211(0x35, 0x80);
		dessc_pll_post_div = 0;
	}

	pcr_m = (pclk_khz * dessc_pll_post_div) / 25;
	pcr_k = pcr_m % 1000;
	pcr_m = pcr_m / 1000;

	pcr_k <<= 14;

	//pixel clk
	x2_write_lt9211(0xff, 0xd0);//pcr
	x2_write_lt9211(0x2d, 0x7f);
	x2_write_lt9211(0x31, 0x00);

	x2_write_lt9211(0x26, 0x80 | ((uint8_t)pcr_m));
	x2_write_lt9211(0x27, (uint8_t)((pcr_k >> 16) & 0xff));//K
	x2_write_lt9211(0x28, (uint8_t)((pcr_k >> 8) & 0xff));//K
	x2_write_lt9211(0x29, (uint8_t)(pcr_k & 0xff));//K

}

void lt9211_mipi_patten(struct video_timing *video_format)
{
	uint32_t pclk_khz;
	uint8_t dessc_pll_post_div;
	uint32_t pcr_m, pcr_k;

	pclk_khz = video_format->pclk_khz;

	x2_write_lt9211(0xff, 0xf9);
	x2_write_lt9211(0x3e, 0x80);

	x2_write_lt9211(0xff, 0x85);
	x2_write_lt9211(0x88, 0xd0);

	x2_write_lt9211(0xa1, 0x77);
	x2_write_lt9211(0xa2, 0xff);

	x2_write_lt9211(0xa3,
		(uint8_t)((video_format->hs+video_format->hbp)/256));
	x2_write_lt9211(0xa4, (uint8_t)
			((video_format->hs+video_format->hbp)%256));//h_start

	x2_write_lt9211(0xa5, (uint8_t)
			((video_format->vs+video_format->vbp)%256));//v_start

	x2_write_lt9211(0xa6, (uint8_t)(video_format->hact/256));
	x2_write_lt9211(0xa7, (uint8_t)(video_format->hact%256)); //hactive

	x2_write_lt9211(0xa8, (uint8_t)(video_format->vact/256));
	x2_write_lt9211(0xa9, (uint8_t)(video_format->vact%256));  //vactive

	x2_write_lt9211(0xaa, (uint8_t)(video_format->htotal/256));
	x2_write_lt9211(0xab, (uint8_t)(video_format->htotal%256));//htotal

	x2_write_lt9211(0xac, (uint8_t)(video_format->vtotal/256));
	x2_write_lt9211(0xad, (uint8_t)(video_format->vtotal%256));//vtotal

	x2_write_lt9211(0xae, (uint8_t)(video_format->hs/256));
	x2_write_lt9211(0xaf, (uint8_t)(video_format->hs%256));   //hsa

	x2_write_lt9211(0xb0, (uint8_t)(video_format->vs%256));    //vsa

	//dessc pll to generate pixel clk
	x2_write_lt9211(0xff, 0x82); //dessc pll
	x2_write_lt9211(0x2d, 0x48); //pll ref select xtal

	if (pclk_khz < 44000) {
		x2_write_lt9211(0x35, 0x83);
		dessc_pll_post_div = 16;
	} else if (pclk_khz < 88000) {
		x2_write_lt9211(0x35, 0x82);
		dessc_pll_post_div = 8;
	} else if (pclk_khz < 176000) {
		x2_write_lt9211(0x35, 0x81);
		dessc_pll_post_div = 4;
	} else if (pclk_khz < 352000) {
		x2_write_lt9211(0x35, 0x80);
		dessc_pll_post_div = 0;
	}

	pcr_m = (pclk_khz * dessc_pll_post_div) / 25;
	pcr_k = pcr_m % 1000;
	pcr_m = pcr_m / 1000;

	pcr_k <<= 14;

	//pixel clk
	x2_write_lt9211(0xff, 0xd0); //pcr
	x2_write_lt9211(0x2d, 0x7f);
	x2_write_lt9211(0x31, 0x00);

	x2_write_lt9211(0x26, 0x80 | ((uint8_t)pcr_m));
	x2_write_lt9211(0x27, (uint8_t)((pcr_k >> 16) & 0xff)); //K
	x2_write_lt9211(0x28, (uint8_t)((pcr_k >> 8) & 0xff)); //K
	x2_write_lt9211(0x29, (uint8_t)(pcr_k & 0xff)); //K
}

int lt9211_mipi_clock_debug(void)
{
	uint8_t sync_polarity;
	uint8_t vs, hs_h, hs_l, vbp, vfp, hbp_h, hbp_l,
		hfp_h, hfp_l, vtotal_h, vtotal_l, htotal_h,
		htotal_l, vact_h, vact_l, hact_h, hact_l;
	uint16_t hs, hbp, hfp, vtotal, htotal, vact, hact;
	int ret = 0;

	x2_write_lt9211(0xff, 0x86);
	x2_write_lt9211(0x20, 0x00);

	msleep(100);

	ret = x2_read_lt9211(0x70, &sync_polarity);

	ret = x2_read_lt9211(0x71, &vs);

	ret = x2_read_lt9211(0x72, &hs_h);
	ret = x2_read_lt9211(0x73, &hs_l);
	hs = (((uint16_t)hs_h) << 8) + (uint16_t)hs_l;

	ret = x2_read_lt9211(0x74, &vbp);

	ret = x2_read_lt9211(0x75, &vfp);

	ret = x2_read_lt9211(0x76, &hbp_h);
	ret = x2_read_lt9211(0x77, &hbp_l);
	hbp = (((uint16_t)hbp_h) << 8) + (uint16_t)hbp_l;

	ret = x2_read_lt9211(0x78, &hfp_h);
	ret = x2_read_lt9211(0x79, &hfp_l);
	hfp = (((uint16_t)hfp_h) << 8) + (uint16_t)hfp_l;

	ret = x2_read_lt9211(0x7a, &vtotal_h);
	ret = x2_read_lt9211(0x7b, &vtotal_l);
	vtotal = (((uint16_t)vtotal_h) << 8) + (uint16_t)vtotal_l;

	ret = x2_read_lt9211(0x7c, &htotal_h);
	ret = x2_read_lt9211(0x7d, &htotal_l);
	htotal = (((uint16_t)htotal_h) << 8) + (uint16_t)htotal_l;

	ret = x2_read_lt9211(0x7e, &vact_h);
	ret = x2_read_lt9211(0x7f, &vact_l);
	vact = (((uint16_t)vact_h) << 8) + (uint16_t)vact_l;

	ret = x2_read_lt9211(0x80, &hact_h);
	ret = x2_read_lt9211(0x81, &hact_l);
	hact = (((uint16_t)hact_h) << 8) + (uint16_t)hact_l;

	pr_debug("\nhfp=%d, hs=%d, hbp=%d, hact=%d, htotal=%d\n",
			hfp, hs, hbp, hact, htotal);
	pr_debug("\nvfp=%d, vs=%d, vbp=%d, vact=%d, vtotal=%d\n",
			vfp, vs, vbp, vact, vtotal);

	return 0;
}
