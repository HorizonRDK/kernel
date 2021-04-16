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

#include <linux/string.h>
#include <linux/printk.h>
#include "soc/hobot/hobot_sensor_dev.h"

static sensor_size_t g_support_preview_sizes[] = {
	{4096, 2160}, /* true 4K */
	{3840, 2160}, /* 4K */
	{2592, 1944}, /* 5MP */
	{2592, 1936}, /* 5MP */
	{2048, 1536}, /* 3MP QXGA */
	{1920, 1080}, /* 1080p */
	{1280, 960},
	{1280, 720},  /* 720P */
	{960, 720},
	{864, 480},   /* FWVGA */
	{800, 480},   /* WVGA */
	{768, 432},
	{720, 480},
	{640, 480},   /* VGA */
	{480, 640},   /* VGA portrait */
	{576, 432},
	{480, 360},   /* HVGA */
	{384, 288},
	{352, 288},   /* CIF */
	{320, 240},   /* QVGA */
	{240, 320},   /* QVGA portrait */
	{240, 160},   /* SQVGA */
	{176, 144},   /* QCIF */
	{144, 176},   /* QCIF portrait */
	{160, 120}
};

extern sensor_info_t ov5648_mipi_raw_info;
extern sensor_info_t ov10635_dvp_yuv_info;
extern sensor_info_t ov10635_mipi_yuv_info;
extern sensor_info_t ov10642_dvp_raw_info;
extern sensor_info_t ov10640_mipi_yuv_info;
extern sensor_info_t ov13855_mipi_raw_info;
extern sensor_info_t ds953_mipi_yuv_info;
extern sensor_info_t ds964_mipi_yuv_info;
extern sensor_info_t ds964_mipi_raw_info;

static sensor_info_t *g_sensor_libs[] = {
	&ov5648_mipi_raw_info,
	&ov10635_dvp_yuv_info,
	&ov10635_mipi_yuv_info,
	&ov10642_dvp_raw_info,
	&ov10640_mipi_yuv_info,
	&ov13855_mipi_raw_info,
	&ds964_mipi_yuv_info,
	&ds964_mipi_raw_info,
};

int sensor_dev_prop(sensor_dev_t *sensor_dev, sensor_cfg_t *cfg, uint8_t index)
{
	uint8_t       lib_index = 0;
	uint8_t       curr_index = index ? index + 1 : index;
	printk(KERN_INFO "search sensor bus: %d format: %d pixlen: %d width: %d height: %d lane: %d index: %d\n",
		   cfg->bus, cfg->format, cfg->pixlen, cfg->width, cfg->height, cfg->lane, index);
	for (lib_index = curr_index; lib_index < (sizeof(g_sensor_libs) / sizeof(void *)); lib_index++) {
		uint8_t        init_index = 0;
		sensor_info_t *info = g_sensor_libs[lib_index];
		if (cfg->bus != info->bus) {
			continue;
		}
		if (cfg->format != info->format) {
			continue;
		}
		for (init_index = 0; init_index < info->size; init_index++) {
			sensor_init_t *curr = &info->init[init_index];
			if (cfg->pixlen != curr->pixlen) {
				continue;
			}
			if (cfg->bus == SENSOR_BUS_MIPI && cfg->lane != curr->lane) {
				continue;
			}
			if (cfg->width != curr->size.width || cfg->height != curr->size.height) {
				continue;
			}
			sensor_dev->name = info->name;
			if ((info->chip.chip_id.reg_addr != info->chip.chip_id.reg_data) && info->chip.chip_id.reg_data) {
				sensor_dev->chip = &info->chip;
			} else {
				sensor_dev->chip = NULL;
			}
			sensor_dev->power = &info->power;
			sensor_dev->init = &curr->sensor_settings->init_settings;
			sensor_dev->start = &info->start;
			sensor_dev->stop = &info->stop;
			sensor_dev->order = &info->order;
			if (cfg->bus == SENSOR_BUS_MIPI) {
				sensor_dev->mipi = &curr->sensor_settings->mipi_params;
				printk(KERN_INFO "mipi csi params: mipiclk %d, linelenth %d, framelenth %d, fps %d, settle %d\n",
					   sensor_dev->mipi->mipiclk, sensor_dev->mipi->linelenth, sensor_dev->mipi->framelenth,
					   sensor_dev->mipi->fps, sensor_dev->mipi->settle);
			}
			printk(KERN_INFO "sensor found: %s\n", sensor_dev->name);
			return lib_index;
		}
	}
	printk(KERN_INFO "no matched sensor found\n");
	return -1;
}
