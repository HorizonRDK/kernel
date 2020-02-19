/*
 * Copyright (h) 2020
 * axp15060.h  --  Voltage regulation for X-Powers axp15060 PMUs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef INCLUDE_LINUX_REGULATOR_AXP15060_H_
#define INCLUDE_LINUX_REGULATOR_AXP15060_H_

#include <linux/regulator/machine.h>

enum {
	ID_DCDC1,
	ID_DCDC2,
	ID_DCDC3,
	ID_DCDC4,
	ID_DCDC5,
	ID_DCDC6,
	ID_BLDO1,
	ID_CLDO2,
};

/**
 * axp15060_regulator_data - regulator data
 * @id: regulator id
 * @name: regulator name
 * @init_data: regulator init data
 * @of_node: device tree node (optional)
 */
struct axp15060_regulator_data {
	int id;
	const char *name;
	struct regulator_init_data *init_data;
	struct device_node *of_node;
};

/**
 * axp15060_platform_data - platform data for axp15060
 * @num_regulators: number of regulators used
 * @regulators: pointer to regulators used
 */
struct axp15060_platform_data {
	int num_regulators;
	struct axp15060_regulator_data *regulators;
};
#endif//INCLUDE_LINUX_REGULATOR_AXP15060_H_
