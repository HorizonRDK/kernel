/*
 * pf5024.h  --  Voltage regulation for NXP pf5024 PMUs
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

#ifndef __LINUX_REGULATOR_PF5024_H
#define __LINUX_REGULATOR_PF5024_H

#include <linux/regulator/machine.h>

enum {
	ID_DCDC1,
	ID_DCDC2,
	ID_DCDC3,
	ID_DCDC4,
};

/**
 * pf5024_regulator_data - regulator data
 * @id: regulator id
 * @name: regulator name
 * @init_data: regulator init data
 * @of_node: device tree node (optional)
 */
struct pf5024_regulator_data {
	int id;
	const char *name;
	struct regulator_init_data *init_data;
	struct device_node *of_node;
};

/**
 * pf5024_platform_data - platform data for pf5024
 * @num_regulators: number of regulators used
 * @regulators: pointer to regulators used
 */
struct pf5024_platform_data {
	int num_regulators;
	struct pf5024_regulator_data *regulators;
};
#endif
