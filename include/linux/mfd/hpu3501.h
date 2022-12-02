/*
 * Copyright (C) 2022 Horizon Robotics
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#ifndef INCLUDE_LINUX_MFD_HPU3501_H_
#define INCLUDE_LINUX_MFD_HPU3501_H_

/**
 * struct hpu3501_board
 * @hpu3501_regulator_data: regulator initialization values
 *
 * Board data may be used to initialize regulator.
 */

struct hpu3501_board {
	struct regulator_init_data *hpu3501_pmic_init_data;
	struct rtc_init_data *hpu3501_rtc_init_data;
};

/**
 * struct hpu3501_dev - hpu3501 sub-driver chip access routines
 * @read_dev() - I2C register read function
 * @write_dev() - I2C register write function
 *
 * Device data may be used to access the HPU3501 chip
 */

struct hpu3501_dev {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct regmap		*regmap;
};

#endif // INCLUDE_LINUX_MFD_HPU3501_H_
