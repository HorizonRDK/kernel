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

#define SPINAND_MFR_XTX			0x0B

#define XTX_STATUS_ECC_MASK		GENMASK(7, 4)
#define XTX_STATUS_ECC_1_BITFLIPS 	(1 << 4)
#define XTX_STATUS_ECC_2_BITFLIPS 	(2 << 4)
#define XTX_STATUS_ECC_3_BITFLIPS 	(3 << 4)
#define XTX_STATUS_ECC_4_BITFLIPS 	(4 << 4)
#define XTX_STATUS_ECC_5_BITFLIPS 	(5 << 4)
#define XTX_STATUS_ECC_6_BITFLIPS 	(6 << 4)
#define XTX_STATUS_ECC_7_BITFLIPS 	(7 << 4)
#define XTX_STATUS_ECC_8_BITFLIPS 	(8 << 4)
#define XTX_STATUS_ECC_UNCOR_ERROR  (0xF << 4)

static SPINAND_OP_VARIANTS(read_cache_variants,
		/* SPINAND_PAGE_READ_FROM_CACHE_QUADIO_OP(0, 2, NULL, 0), */
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

static int xt26q04c_ooblayout_ecc(struct mtd_info *mtd, int section,
									struct mtd_oob_region *region)
{
	if (section)
		return -ERANGE;

	region->offset = 128;
	region->length = 104;

	return 0;
}

static int xt26q04c_ooblayout_free(struct mtd_info *mtd, int section,
									struct mtd_oob_region *region)
{
	if (section)
		return -ERANGE;

	/* Reserve 1 byte for the BBM. */
	region->offset = 1;
	region->length = 127;

	return 0;
}

static int xt26q04c_ecc_get_status(struct spinand_device *spinand,
									u8 status)
{
    switch (status & XTX_STATUS_ECC_MASK)
    {
		case STATUS_ECC_NO_BITFLIPS:
			return 0;

		case XTX_STATUS_ECC_1_BITFLIPS:
			return 1;

		case XTX_STATUS_ECC_2_BITFLIPS:
			return 2;

		case XTX_STATUS_ECC_3_BITFLIPS:
			return 3;

		case XTX_STATUS_ECC_4_BITFLIPS:
			return 4;

		case XTX_STATUS_ECC_5_BITFLIPS:
			return 5;

		case XTX_STATUS_ECC_6_BITFLIPS:
			return 6;

		case XTX_STATUS_ECC_7_BITFLIPS:
			return 7;

		case XTX_STATUS_ECC_8_BITFLIPS:
			return 8;

		case XTX_STATUS_ECC_UNCOR_ERROR:
			return -EBADMSG;

		default:
			break;
    }

	return -EINVAL;
}

static const struct mtd_ooblayout_ops xt26q04c_ooblayout = {
	.ecc = xt26q04c_ooblayout_ecc,
	.free = xt26q04c_ooblayout_free,
};

static const struct spinand_info xtx_spinand_table[] = {
	SPINAND_INFO("XTX26Q04C", 0x23,
		     NAND_MEMORG(1, 4096, 256, 64, 2048, 40, 1, 1, 1),
		     NAND_ECCREQ(8, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
									&write_cache_variants,
									&update_cache_variants),
		     SPINAND_HAS_QE_BIT,
		     SPINAND_ECCINFO(&xt26q04c_ooblayout,
						xt26q04c_ecc_get_status)),
};

static int xtx_spinand_detect(struct spinand_device *spinand)
{
	u8 *id = spinand->id.data;
	int ret;

	if (id[1] != SPINAND_MFR_XTX)
		return 0;

	ret = spinand_match_and_init(spinand, xtx_spinand_table,
				     ARRAY_SIZE(xtx_spinand_table),
				     id[2]);
	if (ret)
		return ret;

	return 1;
}

static const struct spinand_manufacturer_ops xtx_spinand_manuf_ops = {
	.detect = xtx_spinand_detect,
};

const struct spinand_manufacturer xtx_spinand_manufacturer = {
	.id = SPINAND_MFR_XTX,
	.name = "xtx",
	.ops = &xtx_spinand_manuf_ops,
};
