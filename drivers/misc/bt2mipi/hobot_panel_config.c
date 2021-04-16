/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

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

#include "hobot_lt9211.h"

#define LPT_DI 0x39
#define SPT_DI 0x15

#define CMDNUM (10)

//extern struct x2_lt9211_s  *g_x2_lt9211;

//extern int lt9211_reset_pin;
//extern int lcd_reset_pin;
//extern int lcd_pwm_pin;
static uint8_t dcsrst[] = {0x05, 0x01, 0x00};

static uint8_t sleep_out[] = {0x05, 0x11, 0x00};
static uint8_t sleep_in[] = {0x05, 0x10, 0x00};
static uint8_t	display_on[] = {0x05, 0x29, 0x00};

static uint8_t dcs0[] = {LPT_DI, 0xff, 0x98, 0x81, 0x03};
static uint8_t dcs1[] = {LPT_DI, 0xff, 0x98, 0x81, 0x04};
static uint8_t dcs2[] = {LPT_DI, 0xff, 0x98, 0x81, 0x01};
static uint8_t dcs3[] = {LPT_DI, 0xff, 0x98, 0x81, 0x00};

int init_panel(unsigned int display_type)
{
	//uint8_t dcslen = 0;
	int ret = 0;
	pr_debug("\nlt9211 init panel begin\n");

	ret = x2_write_lt9211(0xff, 0x81);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x0e, 0xef);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x0e, 0xff);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x0b, 0xfe);
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0xff, 0x86);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x47, 0x01);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x48, 0x01);
	if (ret < 0)
		return ret;

	ret = x2_write_lt9211(0xff, 0xd4);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x20, 0x2f);
	if (ret < 0)
		return ret;
	ret = x2_write_lt9211(0x21, 0x10);
	if (ret < 0)
		return ret;
//	ret = LCD_reset();
//	if (ret)
//		return ret;
	msleep(50);

	if (display_type == LCD_7_TYPE) {
		pr_debug("\nStart initial panel\n");

		generic_short_write_1P(0xFF, 0x00);
		generic_short_write_1P(0xBA, 0x03);
		generic_short_write_1P(0x36, 0x00);
		generic_short_write_1P(0xB0, 0x00);
		generic_short_write_1P(0xD3, 0x08);
		generic_short_write_1P(0xD4, 0x0E);

		generic_short_write_1P(0xD5, 0x0F);
		generic_short_write_1P(0xD6, 0x48);
		generic_short_write_1P(0xD7, 0x00);
		generic_short_write_1P(0xD9, 0x00);
		generic_short_write_1P(0xFB, 0x01);
		generic_short_write_1P(0xFF, 0xEE);
		generic_short_write_1P(0x40, 0x00);
		generic_short_write_1P(0x41, 0x00);
		generic_short_write_1P(0x42, 0x00);
		generic_short_write_1P(0xFB, 0x01);

		generic_short_write_1P(0xFF, 0x01);
		generic_short_write_1P(0xFB, 0x01);
		generic_short_write_1P(0x01, 0x55);
		generic_short_write_1P(0x04, 0x0C);
		generic_short_write_1P(0x05, 0x3A);
		generic_short_write_1P(0x06, 0x10); //50
		generic_short_write_1P(0x07, 0xD0);
		generic_short_write_1P(0x0A, 0x0F);
		generic_short_write_1P(0x0D, 0x7F);
		generic_short_write_1P(0x0E, 0x7F);
		generic_short_write_1P(0x0F, 0x70);
		generic_short_write_1P(0x10, 0x63);
		generic_short_write_1P(0x11, 0x3C);
		generic_short_write_1P(0x12, 0x5C);
		generic_short_write_1P(0x15, 0x60);
		generic_short_write_1P(0x16, 0x11);
		generic_short_write_1P(0x17, 0x11);
		generic_short_write_1P(0x5B, 0xCA);
		generic_short_write_1P(0x5C, 0x00);
		generic_short_write_1P(0x5D, 0x00);
		generic_short_write_1P(0x5F, 0x1B);
		generic_short_write_1P(0x60, 0xD5);
		generic_short_write_1P(0x61, 0xF7);
		generic_short_write_1P(0x6C, 0xAB);
		generic_short_write_1P(0x6D, 0x44);

		//3Gamma2.5
		generic_short_write_1P(0xFF, 0x01);
		generic_short_write_1P(0xFB, 0x01);

		generic_short_write_1P(0x75, 0x00);
		generic_short_write_1P(0x76, 0x08);
		generic_short_write_1P(0x77, 0x00);
		generic_short_write_1P(0x78, 0x49);
		generic_short_write_1P(0x79, 0x00);
		generic_short_write_1P(0x7A, 0x7A);
		generic_short_write_1P(0x7B, 0x00);
		generic_short_write_1P(0x7C, 0x98);
		generic_short_write_1P(0x7D, 0x00);
		generic_short_write_1P(0x7E, 0xAF);
		generic_short_write_1P(0x7F, 0x00);
		generic_short_write_1P(0x80, 0xC3);
		generic_short_write_1P(0x81, 0x00);
		generic_short_write_1P(0x82, 0xD7);
		generic_short_write_1P(0x83, 0x00);
		generic_short_write_1P(0x84, 0xE4);
		generic_short_write_1P(0x85, 0x00);
		generic_short_write_1P(0x86, 0xF3);
		generic_short_write_1P(0x87, 0x01);
		generic_short_write_1P(0x88, 0x23);
		generic_short_write_1P(0x89, 0x01);
		generic_short_write_1P(0x8A, 0x48);
		generic_short_write_1P(0x8B, 0x01);
		generic_short_write_1P(0x8C, 0x85);
		generic_short_write_1P(0x8D, 0x01);
		generic_short_write_1P(0x8E, 0xB4);
		generic_short_write_1P(0x8F, 0x01);
		generic_short_write_1P(0x90, 0xFD);
		generic_short_write_1P(0x91, 0x02);
		generic_short_write_1P(0x92, 0x3A);
		generic_short_write_1P(0x93, 0x02);
		generic_short_write_1P(0x94, 0x3B);
		generic_short_write_1P(0x95, 0x02);
		generic_short_write_1P(0x96, 0x6F);
		generic_short_write_1P(0x97, 0x02);
		generic_short_write_1P(0x98, 0xAE);
		generic_short_write_1P(0x99, 0x02);
		generic_short_write_1P(0x9A, 0xD3);
		generic_short_write_1P(0x9B, 0x03);
		generic_short_write_1P(0x9C, 0x08);
		generic_short_write_1P(0x9D, 0x03);
		generic_short_write_1P(0x9E, 0x29);
		generic_short_write_1P(0x9F, 0x03);
		generic_short_write_1P(0xA0, 0x5C);
		generic_short_write_1P(0xA2, 0x03);
		generic_short_write_1P(0xA3, 0x67);
		generic_short_write_1P(0xA4, 0x03);
		generic_short_write_1P(0xA5, 0x6F);
		generic_short_write_1P(0xA6, 0x03);
		generic_short_write_1P(0xA7, 0x89);
		generic_short_write_1P(0xA9, 0x03);
		generic_short_write_1P(0xAA, 0xA3);
		generic_short_write_1P(0xAB, 0x03);
		generic_short_write_1P(0xAC, 0xBD);
		generic_short_write_1P(0xAD, 0x03);
		generic_short_write_1P(0xAE, 0xC0);
		generic_short_write_1P(0xAF, 0x03);
		generic_short_write_1P(0xB0, 0xE5);
		generic_short_write_1P(0xB1, 0x03);
		generic_short_write_1P(0xB2, 0xF5);

		generic_short_write_1P(0xB3, 0x00);
		generic_short_write_1P(0xB4, 0x08);
		generic_short_write_1P(0xB5, 0x00);
		generic_short_write_1P(0xB6, 0x49);
		generic_short_write_1P(0xB7, 0x00);
		generic_short_write_1P(0xB8, 0x7A);
		generic_short_write_1P(0xB9, 0x00);
		generic_short_write_1P(0xBA, 0x98);
		generic_short_write_1P(0xBB, 0x00);
		generic_short_write_1P(0xBC, 0xAF);
		generic_short_write_1P(0xBD, 0x00);
		generic_short_write_1P(0xBE, 0xC3);
		generic_short_write_1P(0xBF, 0x00);
		generic_short_write_1P(0xC0, 0xD7);
		generic_short_write_1P(0xC1, 0x00);
		generic_short_write_1P(0xC2, 0xE4);
		generic_short_write_1P(0xC3, 0x00);
		generic_short_write_1P(0xC4, 0xF3);
		generic_short_write_1P(0xC5, 0x01);
		generic_short_write_1P(0xC6, 0x23);
		generic_short_write_1P(0xC7, 0x01);
		generic_short_write_1P(0xC8, 0x48);
		generic_short_write_1P(0xC9, 0x01);
		generic_short_write_1P(0xCA, 0x85);
		generic_short_write_1P(0xCB, 0x01);
		generic_short_write_1P(0xCC, 0xB4);
		generic_short_write_1P(0xCD, 0x01);
		generic_short_write_1P(0xCE, 0xFD);
		generic_short_write_1P(0xCF, 0x02);
		generic_short_write_1P(0xD0, 0x3A);
		generic_short_write_1P(0xD1, 0x02);
		generic_short_write_1P(0xD2, 0x3B);
		generic_short_write_1P(0xD3, 0x02);
		generic_short_write_1P(0xD4, 0x6F);
		generic_short_write_1P(0xD5, 0x02);
		generic_short_write_1P(0xD6, 0xAE);
		generic_short_write_1P(0xD7, 0x02);
		generic_short_write_1P(0xD8, 0xD3);
		generic_short_write_1P(0xD9, 0x03);
		generic_short_write_1P(0xDA, 0x08);
		generic_short_write_1P(0xDB, 0x03);
		generic_short_write_1P(0xDC, 0x29);
		generic_short_write_1P(0xDD, 0x03);
		generic_short_write_1P(0xDE, 0x5C);
		generic_short_write_1P(0xDF, 0x03);
		generic_short_write_1P(0xE0, 0x67);
		generic_short_write_1P(0xE1, 0x03);
		generic_short_write_1P(0xE2, 0x6F);
		generic_short_write_1P(0xE3, 0x03);
		generic_short_write_1P(0xE4, 0x89);
		generic_short_write_1P(0xE5, 0x03);
		generic_short_write_1P(0xE6, 0xA3);
		generic_short_write_1P(0xE7, 0x03);
		generic_short_write_1P(0xE8, 0xBD);
		generic_short_write_1P(0xE9, 0x03);
		generic_short_write_1P(0xEA, 0xC0);
		generic_short_write_1P(0xEB, 0x03);
		generic_short_write_1P(0xEC, 0xE5);
		generic_short_write_1P(0xED, 0x03);
		generic_short_write_1P(0xEE, 0xF5);

		generic_short_write_1P(0xEF, 0x00);
		generic_short_write_1P(0xF0, 0x08);
		generic_short_write_1P(0xF1, 0x00);
		generic_short_write_1P(0xF2, 0x44);
		generic_short_write_1P(0xF3, 0x00);
		generic_short_write_1P(0xF4, 0x76);
		generic_short_write_1P(0xF5, 0x00);
		generic_short_write_1P(0xF6, 0x96);
		generic_short_write_1P(0xF7, 0x00);
		generic_short_write_1P(0xF8, 0xAE);
		generic_short_write_1P(0xF9, 0x00);
		generic_short_write_1P(0xFA, 0xC3);

		generic_short_write_1P(0xFF, 0x02);
		generic_short_write_1P(0xFB, 0x01);

		generic_short_write_1P(0x00, 0x00);
		generic_short_write_1P(0x01, 0xD4);
		generic_short_write_1P(0x02, 0x00);
		generic_short_write_1P(0x03, 0xE4);
		generic_short_write_1P(0x04, 0x00);
		generic_short_write_1P(0x05, 0xF3);
		generic_short_write_1P(0x06, 0x01);
		generic_short_write_1P(0x07, 0x23);
		generic_short_write_1P(0x08, 0x01);
		generic_short_write_1P(0x09, 0x48);
		generic_short_write_1P(0x0A, 0x01);
		generic_short_write_1P(0x0B, 0x84);
		generic_short_write_1P(0x0C, 0x01);
		generic_short_write_1P(0x0D, 0xB4);
		generic_short_write_1P(0x0E, 0x01);
		generic_short_write_1P(0x0F, 0xFE);
		generic_short_write_1P(0x10, 0x02);
		generic_short_write_1P(0x11, 0x38);
		generic_short_write_1P(0x12, 0x02);
		generic_short_write_1P(0x13, 0x3A);
		generic_short_write_1P(0x14, 0x02);
		generic_short_write_1P(0x15, 0x70);
		generic_short_write_1P(0x16, 0x02);
		generic_short_write_1P(0x17, 0xAB);
		generic_short_write_1P(0x18, 0x02);
		generic_short_write_1P(0x19, 0xD3);
		generic_short_write_1P(0x1A, 0x03);
		generic_short_write_1P(0x1B, 0x08);
		generic_short_write_1P(0x1C, 0x03);
		generic_short_write_1P(0x1D, 0x2C);
		generic_short_write_1P(0x1E, 0x03);
		generic_short_write_1P(0x1F, 0x59);
		generic_short_write_1P(0x20, 0x03);
		generic_short_write_1P(0x21, 0x68);
		generic_short_write_1P(0x22, 0x03);
		generic_short_write_1P(0x23, 0x71);
		generic_short_write_1P(0x24, 0x03);
		generic_short_write_1P(0x25, 0x8A);
		generic_short_write_1P(0x26, 0x03);
		generic_short_write_1P(0x27, 0xA4);
		generic_short_write_1P(0x28, 0x03);
		generic_short_write_1P(0x29, 0xBE);
		generic_short_write_1P(0x2A, 0x03);
		generic_short_write_1P(0x2B, 0xE4);
		generic_short_write_1P(0x2D, 0x03);
		generic_short_write_1P(0x2F, 0xF2);
		generic_short_write_1P(0x30, 0x03);
		generic_short_write_1P(0x31, 0xFB);

		generic_short_write_1P(0x32, 0x00);
		generic_short_write_1P(0x33, 0x08);
		generic_short_write_1P(0x34, 0x00);
		generic_short_write_1P(0x35, 0x44);
		generic_short_write_1P(0x36, 0x00);
		generic_short_write_1P(0x37, 0x76);
		generic_short_write_1P(0x38, 0x00);
		generic_short_write_1P(0x39, 0x96);
		generic_short_write_1P(0x3A, 0x00);
		generic_short_write_1P(0x3B, 0xAE);
		generic_short_write_1P(0x3D, 0x00);
		generic_short_write_1P(0x3F, 0xC3);
		generic_short_write_1P(0x40, 0x00);
		generic_short_write_1P(0x41, 0xD4);
		generic_short_write_1P(0x42, 0x00);
		generic_short_write_1P(0x43, 0xE4);
		generic_short_write_1P(0x44, 0x00);
		generic_short_write_1P(0x45, 0xF3);
		generic_short_write_1P(0x46, 0x01);
		generic_short_write_1P(0x47, 0x23);
		generic_short_write_1P(0x48, 0x01);
		generic_short_write_1P(0x49, 0x48);
		generic_short_write_1P(0x4A, 0x01);
		generic_short_write_1P(0x4B, 0x84);
		generic_short_write_1P(0x4C, 0x01);
		generic_short_write_1P(0x4D, 0xB4);
		generic_short_write_1P(0x4E, 0x01);
		generic_short_write_1P(0x4F, 0xFE);
		generic_short_write_1P(0x50, 0x02);
		generic_short_write_1P(0x51, 0x38);
		generic_short_write_1P(0x52, 0x02);
		generic_short_write_1P(0x53, 0x3A);
		generic_short_write_1P(0x54, 0x02);
		generic_short_write_1P(0x55, 0x70);
		generic_short_write_1P(0x56, 0x02);
		generic_short_write_1P(0x58, 0xAB);
		generic_short_write_1P(0x59, 0x02);
		generic_short_write_1P(0x5A, 0xD3);
		generic_short_write_1P(0x5B, 0x03);
		generic_short_write_1P(0x5C, 0x08);
		generic_short_write_1P(0x5D, 0x03);
		generic_short_write_1P(0x5E, 0x2C);
		generic_short_write_1P(0x5F, 0x03);
		generic_short_write_1P(0x60, 0x59);
		generic_short_write_1P(0x61, 0x03);
		generic_short_write_1P(0x62, 0x68);
		generic_short_write_1P(0x63, 0x03);
		generic_short_write_1P(0x64, 0x71);
		generic_short_write_1P(0x65, 0x03);
		generic_short_write_1P(0x66, 0x8A);
		generic_short_write_1P(0x67, 0x03);
		generic_short_write_1P(0x68, 0xA4);
		generic_short_write_1P(0x69, 0x03);
		generic_short_write_1P(0x6A, 0xBE);
		generic_short_write_1P(0x6B, 0x03);
		generic_short_write_1P(0x6C, 0xE4);
		generic_short_write_1P(0x6D, 0x03);
		generic_short_write_1P(0x6E, 0xF2);
		generic_short_write_1P(0x6F, 0x03);
		generic_short_write_1P(0x70, 0xFB);

		generic_short_write_1P(0x71, 0x00);
		generic_short_write_1P(0x72, 0x02);
		generic_short_write_1P(0x73, 0x00);
		generic_short_write_1P(0x74, 0x34);
		generic_short_write_1P(0x75, 0x00);
		generic_short_write_1P(0x76, 0x64);
		generic_short_write_1P(0x77, 0x00);
		generic_short_write_1P(0x78, 0x85);
		generic_short_write_1P(0x79, 0x00);
		generic_short_write_1P(0x7A, 0x9D);
		generic_short_write_1P(0x7B, 0x00);
		generic_short_write_1P(0x7C, 0xB2);
		generic_short_write_1P(0x7D, 0x00);
		generic_short_write_1P(0x7E, 0xC4);
		generic_short_write_1P(0x7F, 0x00);
		generic_short_write_1P(0x80, 0xD4);
		generic_short_write_1P(0x81, 0x00);
		generic_short_write_1P(0x82, 0xE3);
		generic_short_write_1P(0x83, 0x01);
		generic_short_write_1P(0x84, 0x15);
		generic_short_write_1P(0x85, 0x01);
		generic_short_write_1P(0x86, 0x3C);
		generic_short_write_1P(0x87, 0x01);
		generic_short_write_1P(0x88, 0x7A);
		generic_short_write_1P(0x89, 0x01);
		generic_short_write_1P(0x8A, 0xAB);
		generic_short_write_1P(0x8B, 0x01);
		generic_short_write_1P(0x8C, 0xF8);
		generic_short_write_1P(0x8D, 0x02);
		generic_short_write_1P(0x8E, 0x35);
		generic_short_write_1P(0x8F, 0x02);
		generic_short_write_1P(0x90, 0x37);
		generic_short_write_1P(0x91, 0x02);
		generic_short_write_1P(0x92, 0x6C);
		generic_short_write_1P(0x93, 0x02);
		generic_short_write_1P(0x94, 0xAA);
		generic_short_write_1P(0x95, 0x02);
		generic_short_write_1P(0x96, 0xD2);
		generic_short_write_1P(0x97, 0x03);
		generic_short_write_1P(0x98, 0x0B);
		generic_short_write_1P(0x99, 0x03);
		generic_short_write_1P(0x9A, 0x36);
		generic_short_write_1P(0x9B, 0x03);
		generic_short_write_1P(0x9C, 0x7D);
		generic_short_write_1P(0x9D, 0x03);
		generic_short_write_1P(0x9E, 0xA7);
		generic_short_write_1P(0x9F, 0x03);
		generic_short_write_1P(0xA0, 0xFB);
		generic_short_write_1P(0xA2, 0x03);
		generic_short_write_1P(0xA3, 0xFB);
		generic_short_write_1P(0xA4, 0x03);
		generic_short_write_1P(0xA5, 0xFB);
		generic_short_write_1P(0xA6, 0x03);
		generic_short_write_1P(0xA7, 0xFC);
		generic_short_write_1P(0xA9, 0x03);
		generic_short_write_1P(0xAA, 0xFE);
		generic_short_write_1P(0xAB, 0x03);
		generic_short_write_1P(0xAC, 0xFE);
		generic_short_write_1P(0xAD, 0x03);
		generic_short_write_1P(0xAE, 0xFE);

		generic_short_write_1P(0xAF, 0x00);
		generic_short_write_1P(0xB0, 0x02);
		generic_short_write_1P(0xB1, 0x00);
		generic_short_write_1P(0xB2, 0x34);
		generic_short_write_1P(0xB3, 0x00);
		generic_short_write_1P(0xB4, 0x64);
		generic_short_write_1P(0xB5, 0x00);
		generic_short_write_1P(0xB6, 0x85);
		generic_short_write_1P(0xB7, 0x00);
		generic_short_write_1P(0xB8, 0x9D);
		generic_short_write_1P(0xB9, 0x00);
		generic_short_write_1P(0xBA, 0xB2);
		generic_short_write_1P(0xBB, 0x00);
		generic_short_write_1P(0xBC, 0xC4);
		generic_short_write_1P(0xBD, 0x00);
		generic_short_write_1P(0xBE, 0xD4);
		generic_short_write_1P(0xBF, 0x00);
		generic_short_write_1P(0xC0, 0xE3);
		generic_short_write_1P(0xC1, 0x01);
		generic_short_write_1P(0xC2, 0x15);
		generic_short_write_1P(0xC3, 0x01);
		generic_short_write_1P(0xC4, 0x3C);
		generic_short_write_1P(0xC5, 0x01);
		generic_short_write_1P(0xC6, 0x7A);
		generic_short_write_1P(0xC7, 0x01);
		generic_short_write_1P(0xC8, 0xAB);
		generic_short_write_1P(0xC9, 0x01);
		generic_short_write_1P(0xCA, 0xF8);
		generic_short_write_1P(0xCB, 0x02);
		generic_short_write_1P(0xCC, 0x35);
		generic_short_write_1P(0xCD, 0x02);
		generic_short_write_1P(0xCE, 0x37);
		generic_short_write_1P(0xCF, 0x02);
		generic_short_write_1P(0xD0, 0x6C);
		generic_short_write_1P(0xD1, 0x02);
		generic_short_write_1P(0xD2, 0xAA);
		generic_short_write_1P(0xD3, 0x02);
		generic_short_write_1P(0xD4, 0xD2);
		generic_short_write_1P(0xD5, 0x03);
		generic_short_write_1P(0xD6, 0x0B);
		generic_short_write_1P(0xD7, 0x03);
		generic_short_write_1P(0xD8, 0x36);
		generic_short_write_1P(0xD9, 0x03);
		generic_short_write_1P(0xDA, 0x7D);
		generic_short_write_1P(0xDB, 0x03);
		generic_short_write_1P(0xDC, 0xA7);
		generic_short_write_1P(0xDD, 0x03);
		generic_short_write_1P(0xDE, 0xFB);
		generic_short_write_1P(0xDF, 0x03);
		generic_short_write_1P(0xE0, 0xFB);
		generic_short_write_1P(0xE1, 0x03);
		generic_short_write_1P(0xE2, 0xFB);
		generic_short_write_1P(0xE3, 0x03);
		generic_short_write_1P(0xE4, 0xFC);
		generic_short_write_1P(0xE5, 0x03);
		generic_short_write_1P(0xE6, 0xFE);
		generic_short_write_1P(0xE7, 0x03);
		generic_short_write_1P(0xE8, 0xFE);
		generic_short_write_1P(0xE9, 0x03);
		generic_short_write_1P(0xEA, 0xFE);

		generic_short_write_1P(0xFF, 0x05);

		generic_short_write_1P(0xFB, 0x01);
		generic_short_write_1P(0x00, 0x3F);
		generic_short_write_1P(0x01, 0x3F);
		generic_short_write_1P(0x02, 0x3F);
		generic_short_write_1P(0x03, 0x3F);
		generic_short_write_1P(0x04, 0x38);
		generic_short_write_1P(0x05, 0x3F);
		generic_short_write_1P(0x06, 0x3F);
		generic_short_write_1P(0x07, 0x19);
		generic_short_write_1P(0x08, 0x1D);
		generic_short_write_1P(0x09, 0x3F);
		generic_short_write_1P(0x0A, 0x3F);
		generic_short_write_1P(0x0B, 0x1B);
		generic_short_write_1P(0x0C, 0x17);
		generic_short_write_1P(0x0D, 0x3F);
		generic_short_write_1P(0x0E, 0x3F);
		generic_short_write_1P(0x0F, 0x08);
		generic_short_write_1P(0x10, 0x3F);
		generic_short_write_1P(0x11, 0x10);
		generic_short_write_1P(0x12, 0x3F);
		generic_short_write_1P(0x13, 0x3F);
		generic_short_write_1P(0x14, 0x3F);
		generic_short_write_1P(0x15, 0x3F);
		generic_short_write_1P(0x16, 0x3F);
		generic_short_write_1P(0x17, 0x3F);
		generic_short_write_1P(0x18, 0x38);
		generic_short_write_1P(0x19, 0x18);
		generic_short_write_1P(0x1A, 0x1C);
		generic_short_write_1P(0x1B, 0x3F);
		generic_short_write_1P(0x1C, 0x3F);
		generic_short_write_1P(0x1D, 0x1A);
		generic_short_write_1P(0x1E, 0x16);
		generic_short_write_1P(0x1F, 0x3F);
		generic_short_write_1P(0x20, 0x3F);
		generic_short_write_1P(0x21, 0x3F);
		generic_short_write_1P(0x22, 0x3F);
		generic_short_write_1P(0x23, 0x06);
		generic_short_write_1P(0x24, 0x3F);
		generic_short_write_1P(0x25, 0x0E);
		generic_short_write_1P(0x26, 0x3F);
		generic_short_write_1P(0x27, 0x3F);
		generic_short_write_1P(0x54, 0x06);
		generic_short_write_1P(0x55, 0x05);
		generic_short_write_1P(0x56, 0x04);
		generic_short_write_1P(0x58, 0x03);
		generic_short_write_1P(0x59, 0x1B);
		generic_short_write_1P(0x5A, 0x1B);
		generic_short_write_1P(0x5B, 0x01);
		generic_short_write_1P(0x5C, 0x32);
		generic_short_write_1P(0x5E, 0x18);
		generic_short_write_1P(0x5F, 0x20);
		generic_short_write_1P(0x60, 0x2B);
		generic_short_write_1P(0x61, 0x2C);
		generic_short_write_1P(0x62, 0x18);
		generic_short_write_1P(0x63, 0x01);
		generic_short_write_1P(0x64, 0x32);
		generic_short_write_1P(0x65, 0x00);
		generic_short_write_1P(0x66, 0x44);
		generic_short_write_1P(0x67, 0x11);
		generic_short_write_1P(0x68, 0x01);
		generic_short_write_1P(0x69, 0x01);
		generic_short_write_1P(0x6A, 0x04);
		generic_short_write_1P(0x6B, 0x2C);
		generic_short_write_1P(0x6C, 0x08);
		generic_short_write_1P(0x6D, 0x08);
		generic_short_write_1P(0x78, 0x00);
		generic_short_write_1P(0x79, 0x00);
		generic_short_write_1P(0x7E, 0x00);
		generic_short_write_1P(0x7F, 0x00);
		generic_short_write_1P(0x80, 0x00);
		generic_short_write_1P(0x81, 0x00);
		generic_short_write_1P(0x8D, 0x00);
		generic_short_write_1P(0x8E, 0x00);
		generic_short_write_1P(0x8F, 0xC0);
		generic_short_write_1P(0x90, 0x73);
		generic_short_write_1P(0x91, 0x10);
		generic_short_write_1P(0x92, 0x07);
		generic_short_write_1P(0x96, 0x11);
		generic_short_write_1P(0x97, 0x14);
		generic_short_write_1P(0x98, 0x00);
		generic_short_write_1P(0x99, 0x00);
		generic_short_write_1P(0x9A, 0x00);
		generic_short_write_1P(0x9B, 0x61);
		generic_short_write_1P(0x9C, 0x15);
		generic_short_write_1P(0x9D, 0x30);
		generic_short_write_1P(0x9F, 0x0F);
		generic_short_write_1P(0xA2, 0xB0);
		generic_short_write_1P(0xA7, 0x0A);
		generic_short_write_1P(0xA9, 0x00);
		generic_short_write_1P(0xAA, 0x70);
		generic_short_write_1P(0xAB, 0xDA);
		generic_short_write_1P(0xAC, 0xFF);
		generic_short_write_1P(0xAE, 0xF4);
		generic_short_write_1P(0xAF, 0x40);
		generic_short_write_1P(0xB0, 0x7F);
		generic_short_write_1P(0xB1, 0x16);
		generic_short_write_1P(0xB2, 0x53);
		generic_short_write_1P(0xB3, 0x00);
		generic_short_write_1P(0xB4, 0x2A);
		generic_short_write_1P(0xB5, 0x3A);
		generic_short_write_1P(0xB6, 0xF0);
		generic_short_write_1P(0xBC, 0x85);
		generic_short_write_1P(0xBD, 0xF4);
		generic_short_write_1P(0xBE, 0x33);
		generic_short_write_1P(0xBF, 0x13);
		generic_short_write_1P(0xC0, 0x77);
		generic_short_write_1P(0xC1, 0x77);
		generic_short_write_1P(0xC2, 0x77);
		generic_short_write_1P(0xC3, 0x77);
		generic_short_write_1P(0xC4, 0x77);
		generic_short_write_1P(0xC5, 0x77);
		generic_short_write_1P(0xC6, 0x77);
		generic_short_write_1P(0xC7, 0x77);
		generic_short_write_1P(0xC8, 0xAA);
		generic_short_write_1P(0xC9, 0x2A);
		generic_short_write_1P(0xCA, 0x00);
		generic_short_write_1P(0xCB, 0xAA);
		generic_short_write_1P(0xCC, 0x92);
		generic_short_write_1P(0xCD, 0x00);
		generic_short_write_1P(0xCE, 0x18);
		generic_short_write_1P(0xCF, 0x88);
		generic_short_write_1P(0xD0, 0xAA);
		generic_short_write_1P(0xD1, 0x00);
		generic_short_write_1P(0xD2, 0x00);
		generic_short_write_1P(0xD3, 0x00);
		generic_short_write_1P(0xD6, 0x02);
		generic_short_write_1P(0xD7, 0x31);
		generic_short_write_1P(0xD8, 0x7E);
		generic_short_write_1P(0xED, 0x00);
		generic_short_write_1P(0xEE, 0x00);
		generic_short_write_1P(0xEF, 0x70);
		generic_short_write_1P(0xFA, 0x03);
		generic_short_write_1P(0xFF, 0x00);

		dcs_pkt_write(sleep_out[0],
			(sizeof(sleep_out) - 1), &sleep_out[1]);
		msleep(120);
		dcs_pkt_write(display_on[0],
			(sizeof(display_on) - 1), &display_on[1]);
		msleep(20);
	} else if (display_type == MIPI_720P) {
		pr_debug("\nlt9211 config mipi 720 dsi panel\n");
		dcs_pkt_write(dcs0[0], sizeof(dcs0) - 1, &dcs0[1]);
		generic_short_write_1P(0x01, 0x00);
		generic_short_write_1P(0x02, 0x00);
		generic_short_write_1P(0x03, 0x72);
		generic_short_write_1P(0x04, 0x00);
		generic_short_write_1P(0x05, 0x00);
		generic_short_write_1P(0x06, 0x09);
		generic_short_write_1P(0x07, 0x00);
		generic_short_write_1P(0x08, 0x00);
		generic_short_write_1P(0x09, 0x01);
		generic_short_write_1P(0x0a, 0x00);
		generic_short_write_1P(0x0b, 0x00);
		generic_short_write_1P(0x0c, 0x01);
		generic_short_write_1P(0x0d, 0x00);
		generic_short_write_1P(0x0e, 0x00);
		generic_short_write_1P(0x0f, 0x00);
		generic_short_write_1P(0x10, 0x00);
		generic_short_write_1P(0x11, 0x00);
		generic_short_write_1P(0x12, 0x00);
		generic_short_write_1P(0x13, 0x00);
		generic_short_write_1P(0x14, 0x00);
		generic_short_write_1P(0x15, 0x00);
		generic_short_write_1P(0x16, 0x00);
		generic_short_write_1P(0x17, 0x00);
		generic_short_write_1P(0x18, 0x00);
		generic_short_write_1P(0x19, 0x00);
		generic_short_write_1P(0x1a, 0x00);
		generic_short_write_1P(0x1b, 0x00);
		generic_short_write_1P(0x1c, 0x00);
		generic_short_write_1P(0x1d, 0x00);
		generic_short_write_1P(0x1e, 0x40);
		generic_short_write_1P(0x1f, 0x80);
		generic_short_write_1P(0x20, 0x05);
		generic_short_write_1P(0x21, 0x02);
		generic_short_write_1P(0x22, 0x00);
		generic_short_write_1P(0x23, 0x00);
		generic_short_write_1P(0x24, 0x00);
		generic_short_write_1P(0x25, 0x00);
		generic_short_write_1P(0x26, 0x00);
		generic_short_write_1P(0x27, 0x00);
		generic_short_write_1P(0x28, 0x33);
		generic_short_write_1P(0x29, 0x02);
		generic_short_write_1P(0x2a, 0x00);
		generic_short_write_1P(0x2b, 0x00);
		generic_short_write_1P(0x2c, 0x00);
		generic_short_write_1P(0x2d, 0x00);
		generic_short_write_1P(0x2e, 0x00);
		generic_short_write_1P(0x2f, 0x00);
		generic_short_write_1P(0x30, 0x00);
		generic_short_write_1P(0x31, 0x00);
		generic_short_write_1P(0x32, 0x00);
		generic_short_write_1P(0x33, 0x00);
		generic_short_write_1P(0x34, 0x04);
		generic_short_write_1P(0x35, 0x00);
		generic_short_write_1P(0x36, 0x00);
		generic_short_write_1P(0x37, 0x00);
		generic_short_write_1P(0x38, 0x3c);
		generic_short_write_1P(0x39, 0x00);
		generic_short_write_1P(0x3a, 0x40);
		generic_short_write_1P(0x3b, 0x40);
		generic_short_write_1P(0x3c, 0x00);
		generic_short_write_1P(0x3d, 0x00);
		generic_short_write_1P(0x3e, 0x00);
		generic_short_write_1P(0x3f, 0x00);
		generic_short_write_1P(0x40, 0x00);
		generic_short_write_1P(0x41, 0x00);
		generic_short_write_1P(0x42, 0x00);
		generic_short_write_1P(0x43, 0x00);
		generic_short_write_1P(0x44, 0x00);
		generic_short_write_1P(0x50, 0x01);
		generic_short_write_1P(0x51, 0x23);
		generic_short_write_1P(0x52, 0x45);
		generic_short_write_1P(0x53, 0x67);
		generic_short_write_1P(0x54, 0x89);
		generic_short_write_1P(0x55, 0xab);
		generic_short_write_1P(0x56, 0x01);
		generic_short_write_1P(0x57, 0x23);
		generic_short_write_1P(0x58, 0x45);
		generic_short_write_1P(0x59, 0x67);
		generic_short_write_1P(0x5a, 0x89);
		generic_short_write_1P(0x5b, 0xab);
		generic_short_write_1P(0x5c, 0xcd);
		generic_short_write_1P(0x5d, 0xef);
		generic_short_write_1P(0x5e, 0x11);
		generic_short_write_1P(0x5f, 0x01);
		generic_short_write_1P(0x60, 0x00);
		generic_short_write_1P(0x61, 0x15);
		generic_short_write_1P(0x62, 0x14);
		generic_short_write_1P(0x63, 0x0e);
		generic_short_write_1P(0x64, 0x0f);
		generic_short_write_1P(0x65, 0x0c);
		generic_short_write_1P(0x66, 0x0d);
		generic_short_write_1P(0x67, 0x06);
		generic_short_write_1P(0x68, 0x02);
		generic_short_write_1P(0x69, 0x02);
		generic_short_write_1P(0x6a, 0x02);
		generic_short_write_1P(0x6b, 0x02);
		generic_short_write_1P(0x6c, 0x02);
		generic_short_write_1P(0x6d, 0x02);
		generic_short_write_1P(0x6e, 0x07);
		generic_short_write_1P(0x6f, 0x02);
		generic_short_write_1P(0x70, 0x02);
		generic_short_write_1P(0x71, 0x02);
		generic_short_write_1P(0x72, 0x02);
		generic_short_write_1P(0x73, 0x02);
		generic_short_write_1P(0x74, 0x02);
		generic_short_write_1P(0x75, 0x01);
		generic_short_write_1P(0x76, 0x00);
		generic_short_write_1P(0x77, 0x14);
		generic_short_write_1P(0x78, 0x15);
		generic_short_write_1P(0x79, 0x0e);
		generic_short_write_1P(0x7a, 0x0f);
		generic_short_write_1P(0x7b, 0x0c);
		generic_short_write_1P(0x7c, 0x0d);
		generic_short_write_1P(0x7d, 0x06);
		generic_short_write_1P(0x7e, 0x02);
		generic_short_write_1P(0x7f, 0x07);
		generic_short_write_1P(0x80, 0x02);
		generic_short_write_1P(0x81, 0x02);
		generic_short_write_1P(0x82, 0x02);
		generic_short_write_1P(0x83, 0x02);
		generic_short_write_1P(0x84, 0x07);
		generic_short_write_1P(0x85, 0x02);
		generic_short_write_1P(0x86, 0x02);
		generic_short_write_1P(0x87, 0x02);
		generic_short_write_1P(0x88, 0x02);
		generic_short_write_1P(0x89, 0x02);
		generic_short_write_1P(0x8a, 0x02);

		dcs_pkt_write(dcs1[0], sizeof(dcs1) - 1, &dcs1[1]);
		generic_short_write_1P(0x6c, 0x15);
		generic_short_write_1P(0x6e, 0x2a);
		generic_short_write_1P(0x6f, 0x33);
		generic_short_write_1P(0x3a, 0x94);
		generic_short_write_1P(0x8d, 0x1a);
		generic_short_write_1P(0x87, 0xba);
		generic_short_write_1P(0x26, 0x76);
		generic_short_write_1P(0xb2, 0xd1);
		generic_short_write_1P(0xb5, 0x06);

		dcs_pkt_write(dcs2[0], sizeof(dcs2) - 1, &dcs2[1]);
		generic_short_write_1P(0x22, 0x0a);
		generic_short_write_1P(0x31, 0x00);
		generic_short_write_1P(0x53, 0x8a);
		generic_short_write_1P(0x55, 0x8a);
		generic_short_write_1P(0x50, 0xae);
		generic_short_write_1P(0x51, 0xae);
		generic_short_write_1P(0x60, 0x28);

		/*
		 *generic_short_write_1P(0x61, 0x00);
		 *generic_short_write_1P(0x62, 0x19);
		 *generic_short_write_1P(0x63, 0x10);
		 */

		generic_short_write_1P(0xa0, 0x0f);
		generic_short_write_1P(0xa1, 0x1b);
		generic_short_write_1P(0xa2, 0x28);
		generic_short_write_1P(0xa3, 0x12);
		generic_short_write_1P(0xa4, 0x15);
		generic_short_write_1P(0xa5, 0x28);
		generic_short_write_1P(0xa6, 0x1b);
		generic_short_write_1P(0xa7, 0x1e);
		generic_short_write_1P(0xa8, 0x79);
		generic_short_write_1P(0xa9, 0x1b);
		generic_short_write_1P(0xaa, 0x27);
		generic_short_write_1P(0xab, 0x69);
		generic_short_write_1P(0xac, 0x19);
		generic_short_write_1P(0xad, 0x18);
		generic_short_write_1P(0xae, 0x4c);
		generic_short_write_1P(0xaf, 0x21);
		generic_short_write_1P(0xb0, 0x28);
		generic_short_write_1P(0xb1, 0x52);
		generic_short_write_1P(0xb2, 0x65);
		generic_short_write_1P(0xb3, 0x3f);
		generic_short_write_1P(0xc0, 0x04);
		generic_short_write_1P(0xc1, 0x1b);
		generic_short_write_1P(0xc2, 0x27);
		generic_short_write_1P(0xc3, 0x13);
		generic_short_write_1P(0xc4, 0x15);
		generic_short_write_1P(0xc5, 0x28);
		generic_short_write_1P(0xc6, 0x1c);
		generic_short_write_1P(0xc7, 0x1e);
		generic_short_write_1P(0xc8, 0x79);
		generic_short_write_1P(0xc9, 0x1a);
		generic_short_write_1P(0xca, 0x27);
		generic_short_write_1P(0xcb, 0x69);
		generic_short_write_1P(0xcc, 0x1a);
		generic_short_write_1P(0xcd, 0x18);
		generic_short_write_1P(0xce, 0x4c);
		generic_short_write_1P(0xcf, 0x21);
		generic_short_write_1P(0xd0, 0x27);
		generic_short_write_1P(0xd1, 0x52);
		generic_short_write_1P(0xd2, 0x65);
		generic_short_write_1P(0xd3, 0x3f);

		dcs_pkt_write(dcs3[0], sizeof(dcs3) - 1, &dcs3[1]);
		generic_short_write_1P(0x35, 0x00);
		generic_short_write_1P(0x3a, 0x70);

		msleep(20);
		dcs_pkt_write(sleep_out[0],
				sizeof(sleep_out) - 1, &sleep_out[1]);
		msleep(120);
		dcs_pkt_write(display_on[0],
				sizeof(display_on) - 1, &display_on[1]);

		pr_debug("finish initial mipi dsi 720p panel!\n");
	}

	pr_info("\nFinish initial panel");
	return 0;

}

int LCD_reset(void)
{
	int ret;

	ret = gpio_request(lcd_reset_pin, "x2_lcd_reset_pin");
	if (ret) {
		pr_info("%s() Err get lcd reset pin ret= %d\n",
				__func__, ret);
		return ret;
	}
	gpio_direction_output(lcd_reset_pin, 1);
	msleep(100);
	gpio_direction_output(lcd_reset_pin, 0);
	usleep_range(10000, 15000);
	gpio_direction_output(lcd_reset_pin, 1);
	msleep(50);

	return ret;
}

int generic_short_write_1P(uint8_t data0, uint8_t data1)
{
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0xd4);
	if (ret < 0) {
		pr_err("%s() Err write lcd init reg, ret= %d\n",
					 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x01, 0x0c);
	if (ret < 0) {
		pr_err("%s() Err write lcd init reg, ret= %d\n",
					 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x02, 0x04);
	if (ret < 0) {
		pr_err("%s() Err write lcd init reg, ret= %d\n",
					 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x03, 0x15);
	if (ret < 0) {
		pr_err("%s() Err write lcd init reg, ret= %d\n",
					 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x03, data0);
	if (ret < 0) {
		pr_err("%s() Err write lcd init reg, ret= %d\n",
					 __func__, ret);
		return ret;
	}
	ret = x2_write_lt9211(0x03, data1);
	if (ret < 0) {
		pr_err("%s() Err write lcd init reg, ret= %d\n",
					 __func__, ret);
		return ret;
	}
	//msleep(1);
	usleep_range(1000, 1100);
	ret = x2_write_lt9211(0x01, 0x00);
	if (ret < 0) {
		pr_err("%s() Err write lcd init reg, ret= %d\n",
					 __func__, ret);
		return ret;
	}
	return 0;
}

int dcs_pkt_write(uint8_t dcs_di, uint8_t len, uint8_t *ptr)
{
	uint8_t i = 0;
	int ret = 0;

	ret = x2_write_lt9211(0xff, 0xd4);
	if (ret < 0) {
		pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
		return ret;
	}
	if (len == 2) {
		ret = x2_write_lt9211(0x01, 0x0c);
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}
		ret = x2_write_lt9211(0x02, 0x04);
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}
		ret = x2_write_lt9211(0x03, dcs_di);
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}
		ret = x2_write_lt9211(0x03, *ptr);
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}
		ret = x2_write_lt9211(0x03, *(ptr + 1));
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}
	} else {
		ret = x2_write_lt9211(0x01, 0x0e);
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}
		ret = x2_write_lt9211(0x02, len + 6);
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}
		ret = x2_write_lt9211(0x03, dcs_di);
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}
		ret = x2_write_lt9211(0x03, len);
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}
		ret = x2_write_lt9211(0x03, 0x00);
		if (ret < 0) {
			pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
			return ret;
		}

		for (i = 0; i < len; i++) {
			ret = x2_write_lt9211(0x03, *ptr);
			if (ret < 0) {
				pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
				return ret;
			}
			ptr++;
		}
	}
	//msleep(1);
	usleep_range(1000, 1100);
	ret = x2_write_lt9211(0x01, 0x00);
	if (ret < 0) {
		pr_err("%s() Err write lcd init reg, ret= %d\n",
						 __func__, ret);
		return ret;
	}
	return 0;
}
