// SPDX-License-Identifier: GPL-2.0
/*
 * Author:
 *	Chuanhong Guo <gch981213@gmail.com>
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mtd/spinand.h>

#define SPINAND_MFR_GIGADEVICE			0xC8

#define GIGADEVICE_STATUS_ECC_1TO7_BITFLIPS	(1 << 4)
#define GIGADEVICE_STATUS_ECC_8_BITFLIPS	(3 << 4)

// Add for GD5F1GQ4UC, SPI_MEM_OP_ADDR is 3byte, 1byte before is dummy
#define SPINAND_PAGE_READ_FROM_CACHE_X4_OP_GD5F1GQ4UC(addr, ndummy, buf, len)	\
	SPI_MEM_OP(SPI_MEM_OP_CMD(0x6b, 1),				\
				SPI_MEM_OP_ADDR(3, addr, 1),				\
				SPI_MEM_OP_DUMMY(ndummy, 1),				\
				SPI_MEM_OP_DATA_IN(len, buf, 4))

// Add for GD5F1GQ4UC
static SPINAND_OP_VARIANTS(gd5f1gq4uc_read_cache_variants,
		//SPINAND_PAGE_READ_FROM_CACHE_QUADIO_OP(0, 2, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X4_OP_GD5F1GQ4UC(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_DUALIO_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X2_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(true, 0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(false, 0, 1, NULL, 0));

static SPINAND_OP_VARIANTS(read_cache_variants,
		//SPINAND_PAGE_READ_FROM_CACHE_QUADIO_OP(0, 2, NULL, 0),
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

static int gd5fxgq4xa_ooblayout_ecc(struct mtd_info *mtd, int section,
									struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	region->offset = (16 * section) + 8;
	region->length = 8;

	return 0;
}

static int gd5fxgq4xa_ooblayout_free(struct mtd_info *mtd, int section,
									struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	if (section) {
		region->offset = 16 * section;
		region->length = 8;
	} else {
		/* section 0 has one byte reserved for bad block mark */
		region->offset = 1;
		region->length = 7;
	}
	return 0;
}

static int gd5fxgq4xa_ecc_get_status(struct spinand_device *spinand,
									u8 status)
{
	switch (status & STATUS_ECC_MASK) {
	case STATUS_ECC_NO_BITFLIPS:
		return 0;

	case GIGADEVICE_STATUS_ECC_1TO7_BITFLIPS:
		/* 1-7 bits are flipped. return the maximum. */
		return 7;

	case GIGADEVICE_STATUS_ECC_8_BITFLIPS:
		return 8;

	case STATUS_ECC_UNCOR_ERROR:
		return -EBADMSG;

	default:
		break;
	}

	return -EINVAL;
}

static int gd5f1gq4u_ooblayout_ecc(struct mtd_info *mtd, int section,
									struct mtd_oob_region *region)
{
	if (section)
		return -ERANGE;

	region->offset = 64;
	region->length = 64;

	return 0;
}

static int gd5f1gq4u_ooblayout_free(struct mtd_info *mtd, int section,
									struct mtd_oob_region *region)
{
	if (section)
		return -ERANGE;

	/* Reserve 1 byte for the BBM. */
	region->offset = 1;
	region->length = 63;

	return 0;
}

static int gd5f1gq4u_ecc_get_status(struct spinand_device *spinand,
									u8 status)
{
	switch (status & STATUS_ECC_MASK) {
	case STATUS_ECC_NO_BITFLIPS:
		return 0;

	case GIGADEVICE_STATUS_ECC_1TO7_BITFLIPS:
		return 7;

	case GIGADEVICE_STATUS_ECC_8_BITFLIPS:
		return 8;

	case STATUS_ECC_UNCOR_ERROR:
		return -EBADMSG;

	default:
		break;
	}

	return -EINVAL;
}

static const struct mtd_ooblayout_ops gd5fxgq4xa_ooblayout = {
	.ecc = gd5fxgq4xa_ooblayout_ecc,
	.free = gd5fxgq4xa_ooblayout_free,
};

static const struct mtd_ooblayout_ops gd5f1gq4u_ooblayout = {
	.ecc = gd5f1gq4u_ooblayout_ecc,
	.free = gd5f1gq4u_ooblayout_free,
};

static const struct spinand_info gigadevice_spinand_table[] = {
	SPINAND_INFO("GD5F1GQ4UC", 0xA1,
		     NAND_MEMORG(1, 2048, 64, 64, 1024, 20, 1, 1, 1),
		     NAND_ECCREQ(8, 528),
		     // Use special read_cache_variants
		     SPINAND_INFO_OP_VARIANTS(&gd5f1gq4uc_read_cache_variants,
									&write_cache_variants,
									&update_cache_variants),
		     SPINAND_HAS_QE_BIT,
		     SPINAND_ECCINFO(&gd5f1gq4u_ooblayout,
						gd5f1gq4u_ecc_get_status)),
	SPINAND_INFO("GD5F4GQ4UA", 0xF4,
		     NAND_MEMORG(1, 2048, 64, 64, 4096, 20, 1, 1, 1),
		     NAND_ECCREQ(8, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
									&write_cache_variants,
									&update_cache_variants),
		     SPINAND_HAS_QE_BIT,
		     SPINAND_ECCINFO(&gd5fxgq4xa_ooblayout,
						gd5fxgq4xa_ecc_get_status)),
};

static int gigadevice_spinand_detect(struct spinand_device *spinand)
{
	u8 *id = spinand->id.data;
	int ret;

	if (id[0] != SPINAND_MFR_GIGADEVICE)
		return 0;

	ret = spinand_match_and_init(spinand, gigadevice_spinand_table,
				     ARRAY_SIZE(gigadevice_spinand_table),
				     id[1]);
	if (ret)
		return ret;

	return 1;
}

static const struct spinand_manufacturer_ops gigadevice_spinand_manuf_ops = {
	.detect = gigadevice_spinand_detect,
};

const struct spinand_manufacturer gigadevice_spinand_manufacturer = {
	.id = SPINAND_MFR_GIGADEVICE,
	.name = "GigaDevice",
	.ops = &gigadevice_spinand_manuf_ops,
};
