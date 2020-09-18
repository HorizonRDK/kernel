// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 *
 * Author:
 *	Dinggao Pan <dinggao.pan@horizon.ai>
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mtd/spinand.h>

#define SPINAND_MFR_ESMT			0x2C

#define ESMT_STATUS_ECC_1TO3_BITFLIPS	(1 << 4)
#define ESMT_STATUS_ECC_4TO6_BITFLIPS	(3 << 4)
#define ESMT_STATUS_ECC_7TO8_BITFLIPS   (5 << 4)

static SPINAND_OP_VARIANTS(read_cache_variants,
		SPINAND_PAGE_READ_FROM_CACHE_QUADIO_OP(0, 2, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X4_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_DUALIO_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X2_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(true, 0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(false, 0, 1, NULL, 0));

static SPINAND_OP_VARIANTS(write_cache_variants,
		SPINAND_PROG_LOAD_X4(true, 0, NULL, 0),
		SPINAND_PROG_LOAD(true, 0, NULL, 0));

static SPINAND_OP_VARIANTS(update_cache_variants,
		SPINAND_PROG_LOAD_X4(false, 0, NULL, 0),
		SPINAND_PROG_LOAD(false, 0, NULL, 0));

static int f50d4g41xb_ooblayout_ecc(struct mtd_info *mtd, int section,
									struct mtd_oob_region *region)
{
	if (section)
		return -ERANGE;

	region->offset = 128;
	region->length = 128;

	return 0;
}

static int f50d4g41xb_ooblayout_free(struct mtd_info *mtd, int section,
									struct mtd_oob_region *region)
{
	if (section)
		return -ERANGE;

	/* Reserve 1 byte for the BBM. */
	region->offset = 1;
	region->length = 127;

	return 0;
}

static int f50d4g41xb_ecc_get_status(struct spinand_device *spinand,
									u8 status)
{
	switch (status & STATUS_ECC_MASK) {
	case STATUS_ECC_NO_BITFLIPS:
		return 0;

	case ESMT_STATUS_ECC_1TO3_BITFLIPS:
		return 3;

    case ESMT_STATUS_ECC_4TO6_BITFLIPS:
		return 6;

	case ESMT_STATUS_ECC_7TO8_BITFLIPS:
		return 8;

	case STATUS_ECC_UNCOR_ERROR:
		return -EBADMSG;

	default:
		break;
	}

	return -EINVAL;
}

static const struct mtd_ooblayout_ops f50d4g41xb_ooblayout = {
	.ecc = f50d4g41xb_ooblayout_ecc,
	.free = f50d4g41xb_ooblayout_free,
};

static const struct spinand_info esmt_spinand_table[] = {
	SPINAND_INFO("F50D4G41XB", 0x35,
		     NAND_MEMORG(1, 4096, 256, 64, 2048, 40, 1, 1, 1),
		     NAND_ECCREQ(8, 544),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
									&write_cache_variants,
									&update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&f50d4g41xb_ooblayout,
						f50d4g41xb_ecc_get_status)),
};

static int esmt_spinand_detect(struct spinand_device *spinand)
{
	u8 *id = spinand->id.data;
	int ret;

	if (id[1] != SPINAND_MFR_ESMT)
		return 0;

	ret = spinand_match_and_init(spinand, esmt_spinand_table,
				     ARRAY_SIZE(esmt_spinand_table),
				     id[2]);
	if (ret)
		return ret;

	return 1;
}

static const struct spinand_manufacturer_ops esmt_spinand_manuf_ops = {
	.detect = esmt_spinand_detect,
};

const struct spinand_manufacturer esmt_spinand_manufacturer = {
	.id = SPINAND_MFR_ESMT,
	.name = "esmt",
	.ops = &esmt_spinand_manuf_ops,
};
