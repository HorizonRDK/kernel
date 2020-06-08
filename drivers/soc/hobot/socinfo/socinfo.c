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

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/kobject.h>

#include "socinfo.h"

#define SOCINFO_NAME		"x3-socinfo"
#define BUF_LEN		128

const char *soc_id;
const char *bootmode;
const char *socuid;
const char *origin_board_id;
const char *board_id;
const char *ddr_vender;
const char *ddr_type;
const char *ddr_freq;
const char *ddr_size;
const char *som_name;
const char *base_board_name;
const char *board_name;

#if 0
unsigned int x2_board_id[] = {
	0x100, 0x101, 0x102, 0x103, 0x200, 0x201, 0x202, 0x203, 0x204, 0x205,
	0x300, 0x301, 0x302, 0x303, 0x304, 0x400, 0x401, 0x104, 0x105, 0x106,
	0x107,
};

struct hobot_board_info board_of_id[] = {
	[0] = {X2_SVB, "X2-SVB"},
	[1] = {X2_DEV, "X2-DEV"},
	[2] = {X2SOM1V8, "X2SOM1V8"},
	[3] = {X2SOM3V3, "X2SOM3V3"},
	[4] = {J2_SVB, "J2-SVB"},
	[5] = {J2SOM, "J2SOM"},
	[6] = {J2_Mono, "J2-Mono"},
	[7] = {J2_DEV, "J2-DEV"},
	[8] = {J2_SK, "J2-SK"},
	[9] = {J2_SAM, "J2-SAM"},
	[10] = {J2_Quad, "J2-Quad*"},
	[11] = {J2_QuadJ2A, "J2-QuadJ2A"},
	[12] = {J2_QuadJ2B, "J2-QuadJ2B"},
	[13] = {J2_QuadJ2C, "J2-QuadJ2C"},
	[14] = {J2_QuadJ2D, "J2-QuadJ2D"},
	[15] = {J2_mm, "J2-mm"},
        [16] = {J2_mm_s202, "J2-mm-s202"},
	[17] = {X2_SOMFULL, "X2-SOMFULL"},
	[18] = {X2_96BOARD, "X2_96BOARD"},
	[19] = {X2_DEV512M, "X2-DEV512M"},
	[20] = {X2_XIAOMI, "X2-XIAOMI"},
	[21] = {Unknown, "Unknown"},
};

static int parse_boardid(uint32_t board_id)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(x2_board_id); i++) {
		if (board_id == x2_board_id[i])
			break;
	}

	if(i > ARRAY_SIZE(x2_board_id))
		i = ARRAY_SIZE(x2_board_id);
	return i;
}
#endif

ssize_t id_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (board_id == NULL)
		return 0;

	snprintf(buf, BUF_LEN, "%s", board_id);
	strcat(buf, "\n");

	return strlen(buf);
}

ssize_t origin_id_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (origin_board_id == NULL)
		return 0;

	snprintf(buf, BUF_LEN, "%s\n", origin_board_id);

	return strlen(buf);
}


ssize_t name_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	int index;
	char name[128] = { 0 };

	/* add base board name */
	index = simple_strtoul(base_board_name, NULL, 16);
	switch (index) {
	case BASE_BOARD_X3_DVB:
		snprintf(name, sizeof(name), "x3dvb");
		break;
	case BASE_BOARD_J3_DVB:
		snprintf(name, sizeof(name), "j3dvb");
		break;
	case BASE_BOARD_CVB:
		snprintf(name, sizeof(name), "cvb");
		break;
	default:
		snprintf(name, sizeof(name), "x3dvb");
		break;
	}

	/* add som name */
	index = simple_strtoul(som_name, NULL, 16);
	switch (index) {
	case SOM_TYPE_X3:
		snprintf(name, sizeof(name), "%sx3", name);
		break;
	case SOM_TYPE_J3:
		snprintf(name, sizeof(name), "%sj3", name);
		break;
	default:
		snprintf(name, sizeof(name), "%sx3", name);
		break;
	}

	/* add ddr vender */
	index = simple_strtoul(ddr_vender, NULL, 16);
	switch (index) {
	case DDR_MANU_HYNIX:
		snprintf(name, sizeof(name), "%s-hynix", name);
		break;
	case DDR_MANU_MICRON:
		snprintf(name, sizeof(name), "%s-micron", name);
		break;
	case DDR_MANU_SAMSUNG:
		snprintf(name, sizeof(name), "%s-samsung", name);
		break;
	default:
		snprintf(name, sizeof(name), "%s-hynix", name);
		break;
	}

	/* add ddr size */
	index = simple_strtoul(ddr_size, NULL, 16);
	if (index == 0 || index > 4)
		index = 1;
	snprintf(name, sizeof(name), "%s%dG", name, index);

	/* add ddr freq */
	index = simple_strtoul(ddr_freq, NULL, 16);
	switch (index) {
	case DDR_FREQC_667:
		snprintf(name, sizeof(name), "%s-667", name);
		break;
	case DDR_FREQC_1600:
		snprintf(name, sizeof(name), "%s-1600", name);
		break;
	case DDR_FREQC_2133:
		snprintf(name, sizeof(name), "%s-2133", name);
		break;
	case DDR_FREQC_2666:
		snprintf(name, sizeof(name), "%s-2666", name);
		break;
	case DDR_FREQC_3200:
		snprintf(name, sizeof(name), "%s-3200", name);
		break;
	case DDR_FREQC_3733:
		snprintf(name, sizeof(name), "%s-3733", name);
		break;
	case DDR_FREQC_4266:
		snprintf(name, sizeof(name), "%s-4266", name);
		break;

	default:
		snprintf(name, sizeof(name), "%s-2666", name);
		break;
	}

	snprintf(buf, BUF_LEN, "%s\n", name);

	return strlen(buf);
}

ssize_t boot_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (!buf || !bootmode)
		return 0;
	snprintf(buf, BUF_LEN, "%s\n", bootmode);

	return strlen(buf);
}

ssize_t socuid_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (!buf)
		return 0;
	snprintf(buf, BUF_LEN, "%s\n", socuid);

	return strlen(buf);
}

ssize_t ddr_vender_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (!buf)
		return 0;
	snprintf(buf, BUF_LEN, "%s\n", ddr_vender);

	return strlen(buf);
}

ssize_t ddr_name_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (!buf)
		return 0;
	snprintf(buf, BUF_LEN, "%s\n", ddr_type);

	return strlen(buf);
}

ssize_t ddr_freq_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (!buf)
		return 0;
	snprintf(buf, BUF_LEN, "%s\n", ddr_freq);

	return strlen(buf);
}

ssize_t ddr_size_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (!buf)
		return 0;
	snprintf(buf, BUF_LEN, "%s\n", ddr_size);

	return strlen(buf);
}

ssize_t som_name_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (!buf)
		return 0;
	snprintf(buf, BUF_LEN, "%s\n", som_name);

	return strlen(buf);
}

ssize_t base_board_name_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	if (!buf)
		return 0;
	snprintf(buf, BUF_LEN, "%s\n", base_board_name);

	return strlen(buf);
}

ssize_t soc_store(struct class *class, struct class_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

static struct class_attribute name_attribute =
	__ATTR(board_name, 0644, name_show, soc_store);

static struct class_attribute id_attribute =
	__ATTR(board_id, 0644, id_show, soc_store);

static struct class_attribute origin_id_attribute =
	__ATTR(origin_board_id, 0644, origin_id_show, soc_store);

static struct class_attribute ddr_vender_attribute =
	__ATTR(ddr_vender, 0644, ddr_vender_show, soc_store);

static struct class_attribute ddr_name_attribute =
	__ATTR(ddr_type, 0644, ddr_name_show, soc_store);

static struct class_attribute ddr_freq_attribute =
	__ATTR(ddr_freq, 0644, ddr_freq_show, soc_store);

static struct class_attribute ddr_size_attribute =
	__ATTR(ddr_size, 0644, ddr_size_show, soc_store);

static struct class_attribute som_name_attribute =
	__ATTR(som_name, 0644, som_name_show, soc_store);

static struct class_attribute base_board_name_attribute =
	__ATTR(base_board_name, 0644, base_board_name_show, soc_store);

static struct class_attribute boot_attribute =
	__ATTR(boot_mode, 0644, boot_show, soc_store);

static struct class_attribute socuid_attribute =
	__ATTR(soc_uid, 0644, socuid_show, soc_store);

static struct attribute *socinfo_attributes[] = {
	&name_attribute.attr,
	&id_attribute.attr,
	&origin_id_attribute.attr,
	&ddr_vender_attribute.attr,
	&ddr_name_attribute.attr,
	&ddr_freq_attribute.attr,
	&ddr_size_attribute.attr,
	&som_name_attribute.attr,
	&base_board_name_attribute.attr,
	&boot_attribute.attr,
	&socuid_attribute.attr,
	NULL
};

static const struct attribute_group socinfo_group = {
	.attrs = socinfo_attributes,
};

static const struct attribute_group *socinfo_attr_group[] = {
	&socinfo_group,
	NULL,
};

static struct class socinfo_class = {
	.name = "socinfo",
	.class_groups = socinfo_attr_group,
};

/* Match table for of_platform binding */
static const struct of_device_id socinfo_of_match[] = {
	{ .compatible = "hobot,x3-socinfo", },
	{}
};
MODULE_DEVICE_TABLE(of, socinfo_of_match);

static int socinfo_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = of_property_read_string(pdev->dev.of_node, "board_name",
		&board_name);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "board_id",
		&board_id);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "origin_board_id",
		&origin_board_id);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "ddr_vender",
		&ddr_vender);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "ddr_type",
		&ddr_type);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "ddr_freq",
		&ddr_freq);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "ddr_size",
		&ddr_size);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "som_name",
		&som_name);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "base_board_name",
		&base_board_name);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "boot_mode",
		&bootmode);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "socuid", &socuid);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}

	ret = class_register(&socinfo_class);
	if (ret < 0)
		return ret;

	return 0;
}

static int socinfo_remove(struct platform_device *pdev)
{
	class_unregister(&socinfo_class);
	return 0;
}

static struct platform_driver socinfo_platform_driver = {
	.probe   = socinfo_probe,
	.remove  = socinfo_remove,
	.driver  = {
		.name = SOCINFO_NAME,
		.of_match_table = socinfo_of_match,
		},
};

static int __init socinfo_init(void)
{
	int retval = 0;
	/* Register the platform driver */
	retval = platform_driver_register(&socinfo_platform_driver);
	if (retval)
		pr_err("Unable to register platform driver\n");

	return retval;
}

static void __exit socinfo_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&socinfo_platform_driver);

}

module_init(socinfo_init);
module_exit(socinfo_exit);

MODULE_DESCRIPTION("Driver for SOCINFO");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
