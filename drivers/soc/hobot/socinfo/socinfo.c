/*
 * X2 EFUSE driver (For X2 Platform)
 *
 * 2017 - 2018 (C) Horizon Inc.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any
 * later version.
 *
 * This driver has originally been pushed by Horizon using a X2-branding. This
 * still shows in the naming of this file, the kconfig symbols and some symbols
 * in the code.
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

#define SOCINFO_NAME		"x2-socinfo"

static struct kobject *k_obj;

const char *soc_id;

unsigned int x2_board_id[] = {
	0x100, 0x101, 0x102, 0x103, 0x200, 0x201, 0x202, 0x203,
	0x300, 0x301, 0x302, 0x303, 0x304, 0x400
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
	[8] = {J2_Quad, "J2-Quad*"},
	[9] = {J2_QuadJ2A, "J2-QuadJ2A"},
	[10] = {J2_QuadJ2B, "J2-QuadJ2B"},
	[11] = {J2_QuadJ2C, "J2-QuadJ2C"},
	[12] = {J2_QuadJ2D, "J2-QuadJ2D"},
	[13] = {J2_mm, "J2-mm"},
	[14] = {Unknown, "Unknown"},
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

ssize_t id_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	strcpy(buf, "board_id:");
	strcat(buf, soc_id);
	strcat(buf, "\n");

	return strlen(buf);
}

ssize_t id_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

ssize_t name_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	int index;
	uint32_t board_id;

	board_id = simple_strtoul(soc_id, NULL, 16);
	index = parse_boardid(board_id);

	strcat(buf, "board_name:");
	strcat(buf, board_of_id[index].board_id_string);
	strcat(buf, "\n");

	return strlen(buf);
}

ssize_t name_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static struct kobj_attribute id_attribute =
	__ATTR(board_id, 0644, id_show, id_store);

static struct kobj_attribute name_attribute =
	__ATTR(board_name, 0644, name_show, name_store);

static struct attribute *socinfo_attributes[] = {
	&id_attribute.attr,
	&name_attribute.attr,
	NULL
};

static const struct attribute_group socinfo_attr_group = {
	.attrs = socinfo_attributes,
};

/* Match table for of_platform binding */
static const struct of_device_id socinfo_of_match[] = {
	{ .compatible = "hobot,x2-socinfo", },
	{}
};
MODULE_DEVICE_TABLE(of, socinfo_of_match);

static int socinfo_probe(struct platform_device *pdev)
{
	int ret;

	ret = of_property_read_string(pdev->dev.of_node, "board_id", &soc_id);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return ret;
	}
	return 0;
}

static int socinfo_remove(struct platform_device *pdev)
{
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

	k_obj = kobject_create_and_add("socinfo", NULL);
	if (k_obj == NULL)
		pr_err("socinfo sys node create error\n");

	if (sysfs_create_group(k_obj, &socinfo_attr_group))
		pr_err("hw sys node create error\n");
	return retval;
}

static void __exit socinfo_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&socinfo_platform_driver);
	if (k_obj) {
		sysfs_remove_group(k_obj, &socinfo_attr_group);
		kobject_put(k_obj);
	}
}

module_init(socinfo_init);
module_exit(socinfo_exit);

MODULE_DESCRIPTION("Driver for SOCINFO");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
