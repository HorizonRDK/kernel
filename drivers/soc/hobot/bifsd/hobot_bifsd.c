/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright 2018-2020 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/
/**
 * HobotRobotics BIFSD Controller Interface
 * @version	2.0
 * @Revision History:
 * @2018-10-24   shaochuan.zhang@horizon.ai  Draft
 * @2020-02-10   haibo.guo, acc/bypass and byte/sector dts config
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/reset.h>
#include <linux/gpio.h>
#include "hobot_bifsd.h"
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <soc/hobot/diag.h>

#define BIFSD_CONFIG_X2
#define VER			"HOBOT-bifsd_V20.200330"
#define OUT_RANGE_ADDR		(0xFFFFFFFF)
#define CAPACITY_DEFAULT	(0x80000000)	/* 2G */
#define ZERO_ADDR_OFFSET	(512)		/* when addr 0 ,offset to*/
#define WORK_MODE_DEFAULT	(WORK_MODE_ACC)
//#define	OPT_MODE_DEFAULT	(OPT_MODE_BYTE)
#define	OPT_MODE_DEFAULT	(OPT_MODE_SECTOR)
/*****************************************************************************/
/* static Global Variables                                                   */
/*****************************************************************************/
struct bif_sd *bifsd_info;
struct timer_list bifsd_diag_timer;
static uint32_t bifsd_last_err_tm_ms;

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "bifsd: 0 close debug, 1 open debug");

int bifsd_hobot_priv_init(struct bif_sd *sd);
static void bifsd_set_work_mode(struct bif_sd *sd)
{
	if (sd->work_mode == WORK_MODE_ACC)
		mmc_disable_acc_bypass(sd);	// acc
	else
		mmc_enable_acc_bypass(sd);	// bypass
}

static void bifsd_reset(struct bif_sd *sd)
{
	reset_control_assert(sd->rst);
	mdelay(5);
	reset_control_deassert(sd->rst);
}

static int bifsd_clk_on(const struct bif_sd *bif_info)
{
	int ret = 0;

	if (bif_info == NULL)
		return ret;

	if (bif_info->aclk != NULL && !__clk_is_enabled(bif_info->aclk)) {
		ret = clk_prepare_enable(bif_info->aclk);
	}
	return ret;
}

static int bifsd_clk_off(const struct bif_sd *bif_info)
{
	int ret = 0;

	if (bif_info == NULL)
		return ret;

	if (bif_info->aclk != NULL && __clk_is_enabled(bif_info->aclk))
		clk_disable_unprepare(bif_info->aclk);

	return ret;
}

ssize_t bifsd_show(struct kobject *driver,
	struct kobj_attribute *attr, char *buf)
{
	if (bifsd_info == NULL) {
		pr_err("%s bifsd_info == null\n", __func__);
		return -ENXIO;
	}
	if (!strcmp(attr->attr.name, "range_addr_max")) {
		return snprintf(buf, PAGE_SIZE, "0x%08x\n",
			bifsd_info->range_addr_max);
	} else if (!strcmp(attr->attr.name, "work_mode")) {
		return snprintf(buf, PAGE_SIZE, "%d\n", bifsd_info->work_mode);
	} else if (!strcmp(attr->attr.name, "opt_mode")) {
		return snprintf(buf, PAGE_SIZE, "%d\n", bifsd_info->opt_mode);
	} else if (!strcmp(attr->attr.name, "capacity")) {
		return snprintf(buf, PAGE_SIZE, "0x%08x\n", bifsd_info->capacity);
	} else {
		return -EINVAL;
	}
}

ssize_t bifsd_store(struct kobject *driver,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	u32 out = 0;

	if (bifsd_info == NULL) {
		pr_err("%s bifsd_info == null\n", __func__);
		return -ENXIO;
	}
	ret = (kstrtou32(buf, 0, &out));
	if (ret != 0) {
		pr_err("%s input data err\n", __func__);
		return -EINVAL;
	}

	if (!strcmp(attr->attr.name, "range_addr_max")) {
		bifsd_info->range_addr_max = (out & 0xFFFFFFFF);
		mmc_set_out_range_addr(bifsd_info);
	} else if (!strcmp(attr->attr.name, "work_mode")) {
		if (out != bifsd_info->work_mode) {
			if (out == WORK_MODE_ACC)
				bifsd_info->work_mode = WORK_MODE_ACC;
			else
				bifsd_info->work_mode = WORK_MODE_BYPASS;
			bifsd_set_work_mode(bifsd_info);
		}
	} else if (!strcmp(attr->attr.name, "opt_mode")) {
		if (out != bifsd_info->opt_mode) {
			if (out == OPT_MODE_BYTE)
				bifsd_info->opt_mode = OPT_MODE_BYTE;
			else
				bifsd_info->opt_mode = OPT_MODE_SECTOR;

			if (bifsd_info->cd_gpio)
				gpio_direction_output(bifsd_info->cd_gpio, 0);

			bifsd_reset(bifsd_info);
			bifsd_hobot_priv_init(bifsd_info);

			if (bifsd_info->cd_gpio)
				gpio_direction_output(bifsd_info->cd_gpio, 1);
		}

	} else {
		return -EINVAL;
	}

	return count;
}

static struct kobj_attribute range_addr_max_attr = {
	.attr   = {
		.name = __stringify(range_addr_max),
		.mode = 0644,
	},
	.show   = bifsd_show,
	.store  = bifsd_store,
};

static struct kobj_attribute work_mode_attr = {
	.attr   = {
		.name = __stringify(work_mode),
		.mode = 0644,
	},
	.show   = bifsd_show,
	.store  = bifsd_store,
};

static struct kobj_attribute opt_mode_attr = {
	.attr	= {
		.name = __stringify(opt_mode),
		.mode = 0644,
	},
	.show	= bifsd_show,
	.store	= bifsd_store,
};

static struct kobj_attribute capacity_attr = {
	.attr	= {
		.name = __stringify(capacity),
		.mode = 0644,
	},
	.show	= bifsd_show,
};

static struct attribute *bifsd_attributes[] = {
	&range_addr_max_attr.attr,
	&work_mode_attr.attr,
	&opt_mode_attr.attr,
	&capacity_attr.attr,
	NULL,
};

static struct attribute_group bifsd_group = {
	.attrs = bifsd_attributes,
};

void set_bifsd_info(struct bif_sd *sd)
{
	bifsd_info = sd;
}

struct bif_sd *get_bifsd_info(void)
{
	return bifsd_info;
}

void update_cid_val(struct bif_sd *sd)
{
	unsigned char i;
	unsigned char update_flag;
	u32 reg_val[4];
	volatile u32 *cid_reg = (volatile u32 *)EMMC_CID;

	for (i = 0; i < 4; i++) {
		reg_val[i] = sd_readl(sd, CID, 4 * i);
	}
	update_flag = reg_val[3] & 0xFF000000;
	if (update_flag) {
		return;
	} else {
		for (i = 0; i < 4; i++) {
			sd_writel(sd, CID, reg_val[i], 4 * i);
		}
		cid_reg[3] |= 0x01000000;
	}

	sd_writel(sd, MEM_MGMT, 0x01, 0);
	return;
}

void mmc_cid_config(struct bif_sd *sd)
{
	int i;
	int tbl_size;
	u32 *cid_tbl_val = NULL;
	card_type_t card_type = MMC_MODE;
	const u32 cid_sd_card_tbl_val[] = {
		/* CID Tbl Addr Ofst    VALUE to be initialised */
		/*0x00 */ 0xc3aa004a,
		/*0x04 */ 0x3610432c,
		/*0x08 */ 0x53483235,
		/*0x0C */ 0x00015048,
	};
	const u32 cid_mmc_4_4_card_tbl_val[] = {
		/* CID Tbl Addr Ofst    VALUE to be initialised */
		/*0x00 */ 0xc3aa004a,
		/*0x04 */ 0x3610432c,
		/*0x08 */ 0x53483235,
		/*0x0C */ 0x00010048,
	};
	const u32 cid_mmc_4_2_card_tbl_val[] = {
		/* CSD Tbl Addr Ofst VALUE to be initialised */
		/*0x00 */ 0xc3aa004a,
		/*0x04 */ 0x3610432c,
		/*0x08 */ 0x53483235,
		/*0x0C */ 0x00015048,
	};

	switch (card_type) {
	case sd_card:
	case sdhc_card:
	case sdxc_card:
		cid_tbl_val = (u32 *) & cid_sd_card_tbl_val;
		break;
	case mmc_card_4_2:
		cid_tbl_val = (u32 *) & cid_mmc_4_2_card_tbl_val;
		break;
	case mmc_card_4_4:
	case mmc_card_4_5:
	case mmc_card_5_0:
		cid_tbl_val = (u32 *) & cid_mmc_4_4_card_tbl_val;
		break;
	default:
		dev_err(sd->dev, "card type falt\n");
		break;
	}

	tbl_size = 4;
	for (i = 0; i < tbl_size; i++) {
		sd_writel(sd, CID, cid_tbl_val[i], 4 * i);
	}
}

void mmc_csd_config(struct bif_sd *sd)
{
	int i;
	u32 *csd_tbl_val;
	u32 e = 0, m = 0;
	card_type_t card_type = MMC_MODE;
	const u32 csd_sd_card_tbl_val[] = {
		/* CSD Tbl Addr Ofst    VALUE to be initialised */
		/*0x00 */ 0x08966004,
		/*0x04 */ 0xffc00344,
		/*0x08 */ 0x325f5980,
		/*0x0C */ 0x000026ff,
	};
	const u32 csd_sdhc_card_tbl_val[] = {
		/* CSD Tbl Addr Ofst    VALUE to be initialised */
		/*0x00 */ 0x08966004,
		/*0x04 */ 0xffc00344,
		/*0x08 */ 0x325f5980,
		/*0x0C */ 0x004026ff,
	};
	u32 csd_mmc_card_tbl_val[] = {
		/* CSD Tbl Addr Ofst    VALUE to be initialised */
		/*0x00 */ 0xe7966004,
		/*0x04 */ 0xffc0031c,
		/*0x08 */ 0x2a3f599b,
		/*0x0C */ 0x009086ff,
	};

	switch (card_type) {
	case sd_card:
		csd_tbl_val = (u32 *) & csd_sd_card_tbl_val;
		break;
	case sdhc_card:
	case sdxc_card:
		csd_tbl_val = (u32 *) & csd_sdhc_card_tbl_val;
		break;
	case mmc_card_4_2:
	case mmc_card_4_4:
	case mmc_card_4_5:
	case mmc_card_5_0:
#if 0
		e = 7;
		csd_mmc_card_tbl_val[47/32] &= ~(0x7 << (47%32));
		csd_mmc_card_tbl_val[47/32] |= (e << (47%32));
		e = e + 2;
		m = (sd->capacity >> e) - 1;
		csd_mmc_card_tbl_val[62/32] &= ~(0x3 << (62%32));
		csd_mmc_card_tbl_val[62/32] |= ((m & 3) << (62%32));
		csd_mmc_card_tbl_val[64/32] &= ~(0x3ff << (64%32));
		csd_mmc_card_tbl_val[64/32] |= (((m >> 2) & 0x3ff) << (64%32));
#endif
		csd_tbl_val = (u32 *) & csd_mmc_card_tbl_val;
//		pr_err("capacity=0x%x e=0x%x m=0x%x\n", sd->capacity, e, m);
		break;
	default:
		return;
	}
	for (i = 0; i < 4; i++)
		sd_writel(sd, CSD, csd_tbl_val[i], 4 * i);
}

void mmc_extended_csd_config(struct bif_sd *sd)
{
	int i;
	int tbl_size;
	u32 *csd_tbl_val;
	card_type_t card_type = MMC_MODE;
	const u32 ext_csd_mmc_4_2_tbl_val[] = {
		/* Extend CSD Tbl Addr Ofst VALUE to be initialised */
		/*CSD_SD_STANDARD_CARD:192bits  */
		/*0x00 */ 0x00000000,
		/*0x04 */ 0x02020000,
		/*0x08 */ 0x55555502,
		/*0x0C */ 0x461e1e55,
		/*0x10 */ 0x008c8c46,
		/*0x14 */ 0x00000000,
	};

	const u32 ext_csd_mmc_4_4_tbl_val[] = {
		/* Extend CSD Tbl Addr Ofst VALUE to be initialised */
		/*CSD_SD_STANDARD_CARD:624bits */
		/*0x00 */ 0x00000000,
		/*0x04 */ 0x00000000,
		/*0x08 */ 0x00000000,
		/*0x0C */ 0x00000000,
		/*0x10 */ 0x00000000,
		/*0x14 */ 0x00000000,
		/*0x18 */ 0x00000000,
		/*0x1c */ 0x00000000,
		/*0x20 */ 0x00480000,
		/*0x24 */ 0x02020300,
		/*0x28 */ 0x55555555,
		/*0x2c */ 0x46461e1e,
		/*0x30 */ 0x00008c8c,
		/*0x34 */ 0x01010000,
		/*0x38 */ 0x00010001,
		/*0x3c */ 0x01010100,
		/*0x40 */ 0x00000000,
		/*0x44 */ 0x00000000,
		/*0x48 */ 0x00000000,
	};
	const u32 ext_csd_mmc_4_5_tbl_val[] = {
		/* Extend CSD Tbl Addr Ofst VALUE to be initialised */
		/*CSD_SD_STANDARD_CARD:1168bits */
		/*0x00 */ 0x00000000,
		/*0x04 */ 0x00000000,
		/*0x08 */ 0x00000000,
		/*0x0c */ 0x00000000,
		/*0x10 */ 0x00000000,
		/*0x14 */ 0x00000000,
		/*0x18 */ 0x00000000,
		/*0x1c */ 0x00000000,
		/*0x20 */ 0x00000000,
		/*0x24 */ 0x00000000,
		/*0x28 */ 0x00000000,
		/*0x2c */ 0x00000000,
		/*0x30 */ 0x00000000,
		/*0x34 */ 0x00000000,
		/*0x38 */ 0x00000000,
		/*0x3c */ 0x00000000,
		/*0x40 */ 0x00000000,
		/*0x44 */ 0x00000000,
		/*0x48 */ 0x00000000,
		/*0x4c */ 0x00000148,
		/*0x50 */ 0x03000000,
		/*0x54 */ 0x00000202,
		/*0x58 */ 0x55555555,
		/*0x5c */ 0x46461e1e,
		/*0x60 */ 0x00008c8c,
		/*0x64 */ 0x01010000,
		/*0x68 */ 0x01010001,
		/*0x6c */ 0x01010100,
		/*0x70 */ 0x00000001,
		/*0x74 */ 0x00000000,
		/*0x78 */ 0x00000000,
		/*0x7c */ 0x00000000,
		/*0x80 */ 0x00000001,
		/*0x84 */ 0x00000000,
		/*0x88 */ 0x00000000,
		/*0x8c */ 0x00000000,
		/*0x90 */ 0x00000000,
	};

	u32 ext_csd_mmc_5_0_tbl_val[] = {
		/* Extend CSD Tbl Addr Ofst VALUE to be initialised */
		/*CSD_SD_STANDARD_CARD:1784bits */
		/*0xA4 */ 0x00000000,	//[0]
		/*0xA8 */ 0x00000000,
		/*0xAC */ 0x00000000,
		/*0xB0 */ 0x00000000,
		/*0xB4 */ 0x00000000,	//[16]
		/*0xB8 */ 0x00000000,
		/*0xBC */ 0x00000000,
		/*0xC0 */ 0x00000000,
		/*0xC4 */ 0x00000000,	//[32] FLUSH_CACHE [32]
		/*0xC8 */ 0x00000000,
		/*0xCC */ 0x00000000,
		/*0xD0 */ 0x00000000,
		/*0xD4 */ 0x00000000,	//[48]
		/*0xD8 */ 0x00000000,
		/*0xDC */ 0x00000000,
		/*0xE0 */ 0x00000000,	//INI_TIMEOUT_EMU [60] DATA_SECTOR_SIZE [61]
		/*0xE4 */ 0x00000000,	//[64] VENDOR_SPECIFIC_FIELD [127:64]
		/*0xE8 */ 0x00000000,
		/*0xEC */ 0x00000000,
		/*0xF0 */ 0x00000000,
		/*0xF4 */ 0x00000000,	//[80]
		/*0xF8 */ 0x00000000,
		/*0xFC */ 0x48000000,	//[88] ENH_START_ADDR [139:136]
		/*0x100 */ 0x00000001,	//[92] ENH_SIZE_MULT [142:140]
		/*0x104 */ 0x02030000,	//[96] GP_SIZE_MULT_GP0-3 [154:143]
		/*0x108 */ 0x55000002,	//[100]
		/*0x10C */ 0x1e555555,	//[104] PARTITION_SETTING_COMPLETED[155]
		/*0x110 */ 0x8c46461e,	//[108] PARTITIONS_ATTRIBUTE [156]
		/*0x114 */ 0x0001008c,	//[112] PARTITIONING_SUPPORT [160]
		/*0x118 */ 0x01000100,	//[116] BKOPS_START [164]
		/*0x11C */ 0x01000100,	//[120] RPMB_SIZE_MULT [168] No support
		/*0x120 */ 0x01010001,	//[124]
		/*0x124 */ 0x00000101,	//[128] PARTITION_CONFIG [179]
		/*0x128 */ 0x00000000,	//[132] BUS_WIDTH [183]
		/*0x12C */ 0x00000000,	//[136] STROBE_SUPPORT [184]
		/*0x130 */ 0x00000000,	//[140] CMD_SET_REV [189]
		/*0x134 */ 0x00020007,	//[144] EXT_CSD_REV [192]
		/*0x138 */ 0x0a0a1f13,	//[148] DEVICE_TYPE [196]
		/*0x13C */ 0x8888eeee,	//[152]
		/*0x140 */ 0x460f1e00,	//[156]
		/*0x144 */ 0x0014780f,	//[160]
		/*0x148 */ 0x03a3e000,	//[164] SEC_COUNT [215:212]
//		/*0x148 */ 0x00400000,	// [164] SEC_COUNT [215:212] 0x00400000*512
		/*0x14C */ 0x0a0a1410,	//[168]
		/*0x150 */ 0x09010108,	//[172] HC_WP_GRP_SIZE [221]
		/*0x154 */ 0x00200808,	//[176] BOOT_SIZE_MULT [226]
//		/*0x154 */ 0x00000008,	// [176] BOOT_SIZE_MULT [226] No boot
		/*0x158 */ 0x55c8f400,	//[180] BOOT_INFO [228]
		/*0x15C */ 0x0a640001,	//[184]
		/*0x160 */ 0x99eeeeee,	//[188]
		/*0x164 */ 0x00001e01,	//[188]
		/*0x168 */ 0x32000000,	//[196]
		/*0x16C */ 0x0000000a,	//[200]
		/*0x170 */ 0x0002ee00,	//[204]
		/*0x174 */ 0x00000000,	//[208]
		/*0x178 */ 0x00000000,	//[212]
		/*0x17C */ 0x01202001,	//[216]
		/*0x180 */ 0x00000001,	//[220]
	};

	switch (card_type) {
	case mmc_card_4_2:
		csd_tbl_val = (u32 *) & ext_csd_mmc_4_2_tbl_val;
		tbl_size =
		    sizeof(ext_csd_mmc_4_2_tbl_val) /
		    sizeof(ext_csd_mmc_4_2_tbl_val[0]);
		break;
	case mmc_card_4_4:
		csd_tbl_val = (u32 *) & ext_csd_mmc_4_4_tbl_val;
		tbl_size =
		    sizeof(ext_csd_mmc_4_4_tbl_val) /
		    sizeof(ext_csd_mmc_4_4_tbl_val[0]);
		break;
	case mmc_card_4_5:
		csd_tbl_val = (u32 *) & ext_csd_mmc_4_5_tbl_val;
		tbl_size =
		    sizeof(ext_csd_mmc_4_5_tbl_val) /
		    sizeof(ext_csd_mmc_4_5_tbl_val[0]);
		break;
	case mmc_card_5_0:
		//ext_csd_mmc_5_0_tbl_val[(0x148 - 0xA4)/4] = sd->capacity / 512;
		csd_tbl_val = (u32 *) & ext_csd_mmc_5_0_tbl_val;
		tbl_size =
		    sizeof(ext_csd_mmc_5_0_tbl_val) /
		    sizeof(ext_csd_mmc_5_0_tbl_val[0]);
		break;
	default:
		return;
	}

	for (i = 0; i < tbl_size; i++) {
		sd_writel(sd, EXTENDED_CSD, csd_tbl_val[i], 4 * i);
	}
}

void mmc_set_sd_status_reg(struct bif_sd *sd)
{
	int i;
	const u32 sd_status_tbl_val[] = {
		/*0x00 */ 0x01160000,
		/*0x04 */ 0x02199000,
		/*0x08 */ 0x00000032,
		/*0x0C */ 0x00000000,
	};
	for (i = 0; i < 4; i++) {
		sd_writel(sd, STATUS, sd_status_tbl_val[i], 4 * i);
	}
}

void mmc_set_sd_scr_reg(struct bif_sd *sd)
{
	int i;
	u32 *scr_tbl_val;
	card_type_t card_type = MMC_MODE;
	const u32 sd_tbl_val[] = {
		/*0x00 */ 0x00000000,
		/*0x04 */ 0x01b50000,
	};
	const u32 sdhc_tbl_val[] = {
		/*0x00 */ 0x00000000,
		/*0x04 */ 0x02b50000,
	};
	const u32 sdxc_tbl_val[] = {
		/*0x00 */ 0x00000000,
		/*0x04 */ 0x02b58003,
	};
	switch (card_type) {
	case sd_card:
		scr_tbl_val = (u32 *) & sd_tbl_val;
		break;
	case sdhc_card:
		scr_tbl_val = (u32 *) & sdhc_tbl_val;
		break;
	case sdxc_card:
		scr_tbl_val = (u32 *) & sdxc_tbl_val;
		break;
	default:
		return;
	}

	for (i = 0; i < 2; i++) {
		sd_writel(sd, SCR, scr_tbl_val[i], 4 * i);
	}
}

void device_mode_select(struct bif_sd *sd)
{
	u32 reg_val = 0;
	card_type_t card_type = MMC_MODE;
	reg_val = sd_readl(sd, PROGRAM_REG, 0);
	reg_val &= 0xFFFFFFFC;

	switch (card_type) {
	case sd_card:
	case sdhc_card:
	case sdxc_card:
		reg_val |= BIT(0);
		break;
	case mmc_card_4_2:
	case mmc_card_4_4:
	case mmc_card_4_5:
	case mmc_card_5_0:
		reg_val |= BIT(1);
		break;
	default:
		dev_err(sd->dev, "card type fault\n");
		break;
	}

	reg_val |= BIT(4);

	sd_writel(sd, PROGRAM_REG, reg_val, 0);
}

void sd_card_init(struct bif_sd *sd)
{
	u32 reg_val;
	card_type_t card_type = MMC_MODE;
	if ((card_type == sd_card) || (card_type == sdhc_card)
	    || (card_type == sdxc_card)) {
		mmc_set_sd_status_reg(sd);
		mmc_set_sd_scr_reg(sd);
		sd_writel(sd, SECURITY_INT_ENABLE, EMMC_SECURITY_INT_ENABLE_VAL,
			  0);

		reg_val = sd_readl(sd, BLOCK_COUNT_SECURITY, 0);
		reg_val &= 0xFFFF0000;
		reg_val |= BIT(31);
		sd_writel(sd, BLOCK_COUNT_SECURITY, reg_val, 0);
	}
}

/* Initialize Bifsd device */
int bifsd_hobot_priv_init(struct bif_sd *sd)
{
	if (debug)
		pr_err("%s\n", __func__);
	sd_writel(sd, INT_ENABLE_1, 0xFFFFCFFF, 0);
	sd_writel(sd, INT_ENABLE_2, 0x007FFFFF, 0);
	mmc_set_power_up(sd);
	mmc_config_ocr_reg(sd);
	mmc_extended_csd_config(sd);
	mmc_set_hard_reset_cnt(sd);
	mmc_csd_config(sd);
	mmc_cid_config(sd);
	mmc_set_out_range_addr(sd);

	device_mode_select(sd);

	bifsd_set_work_mode(sd);

	if (sd->opt_mode == OPT_MODE_BYTE)
		sd->addr_offset = ZERO_ADDR_OFFSET;
	else
		sd->addr_offset = ZERO_ADDR_OFFSET >> 9;
#ifndef HUGO_PLM
	bifsd_config_timing(sd);
#endif
	card_power_up(sd);

	return 0;
}

static const struct bifsd_drv_data hobot_drv_data = {
	.init = bifsd_hobot_priv_init,
};

static const struct of_device_id bifsd_hobot_of_match[] = {
	/* SoC-specific compatible strings w/ soc_ctl_map */
	{.compatible = "hobot,bifsd", .data = &hobot_drv_data },
	{.compatible = "hobot,hobot-bifsd", .data = &hobot_drv_data },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, bifsd_hobot_of_match);

static void bifsd_diag_report(uint8_t errsta, uint32_t irqstareg)
{
	uint32_t sta;

	sta = irqstareg;
	bifsd_last_err_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_bif,
				EventIdBifSdErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&sta,
				4);
	}
}

static void bifsd_diag_timer_func(unsigned long data)
{
	uint32_t now_tm_ms;
	unsigned long jiffi;

	now_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (now_tm_ms - bifsd_last_err_tm_ms > 6000) {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_bif,
				EventIdBifSdErr,
				DiagEventStaSuccess);
	}
	jiffi = get_jiffies_64() + msecs_to_jiffies(3000);
	mod_timer(&bifsd_diag_timer, jiffi); // trriger again.
}

static irqreturn_t bifsd_interrupt(int irq, void *dev_id)
{
	struct bif_sd *sd = dev_id;
	u32 pending_1;
	u32 pending_2;
	u32 pending;
	u32 reg_val;
	int ret;

	pending_1 = sd_readl(sd, INT_STATUS_1, 0);
	pending_2 = sd_readl(sd, INT_STATUS_2, 0);

	if (debug > 1)
		pr_err("%08x %08x\n", pending_1, pending_2);

	if ((pending_2 & MMC_DATA_CRC_ERR) ||
		(pending_2 & MMC_CMD_CRC_ERR) ||
		(pending_2 & MMC_PACKED_FAILURE) ||
		(pending_2 & MMC_PWD_CMD_FAIL)) {
		bifsd_diag_report(1, pending_2);
	}

	/* Protocol read without count */
	if ((pending_1 & MMC_MULTI_BLOCK_READ_WRITE)
	    && (pending_2 & MMC_SECURITY_PROTOCOL_READ)) {
		/* Read address from master */
		reg_val = sd_readl(sd, ARGUMENT_REG, 0);

		/* need set dma address for master read */
		sd_writel(sd, DMA_ADDR, reg_val, 0);
		sd_writel(sd, MEM_MGMT, reg_val, 0);

		/* the block count is the slave set */
		sd_writel(sd, BLOCK_CNT, sd->block_cnt, 0);
		sd_writel(sd, INT_STATUS_2, MMC_SECURITY_PROTOCOL_READ, 0);
		sd_writel(sd, INT_STATUS_1, MMC_MULTI_BLOCK_READ_WRITE, 0);
		pending_1 &= ~MMC_MULTI_BLOCK_READ_WRITE;
		pending_2 &= ~MMC_SECURITY_PROTOCOL_READ;
	}
	/* Protocol write without count */
	if ((pending_1 & MMC_MULTI_BLOCK_READ_WRITE)
	    && (pending_2 & MMC_SECURITY_PROTOCOL_WRITE)) {
		/* Write address from master */
		reg_val = sd_readl(sd, ARGUMENT_REG, 0);
		/* need set dma address for master write */
		sd_writel(sd, DMA_ADDR, reg_val, 0);
		sd_writel(sd, MEM_MGMT, 0x01, 0);

		/* the block count is the slave set */
		sd_writel(sd, BLOCK_CNT, sd->block_cnt, 0);
		sd_writel(sd, INT_STATUS_1, MMC_MULTI_BLOCK_READ_WRITE, 0);
		sd_writel(sd, INT_STATUS_2, MMC_SECURITY_PROTOCOL_WRITE, 0);
		pending_1 &= ~MMC_MULTI_BLOCK_READ_WRITE;
		pending_2 &= ~MMC_SECURITY_PROTOCOL_WRITE;
	}

	if (pending_1 != 0) {
		pending = pending_1;
		if (pending & MMC_IDLE_CMD) {
			/* CMD0 received then reinit EMMC Card */
			if (sd->drv_data && sd->drv_data->init) {
				ret = sd->drv_data->init(sd);
				if (ret) {
					dev_err(sd->dev,
						"implementation specific init failed\n");
				}
			}
			sd_writel(sd, INT_STATUS_1, MMC_IDLE_CMD, 0);
			pending &= ~MMC_IDLE_CMD;
		}

		if (pending & MMC_SET_BLOCK_LEN) {
			sd->block_len = sd_readl(sd, BLOCK_LEN, 0);
			sd_writel(sd, INT_STATUS_1, MMC_SET_BLOCK_LEN, 0);
			pending &= ~MMC_SET_BLOCK_LEN;
		}

		if (pending & MMC_SET_BLOCK_CNT) {
			reg_val = sd_readl(sd, ERASE_BLOCK_CNT, 0);
			if (reg_val)
				sd->block_cnt = reg_val & 0x0000ffff;
			sd_writel(sd, INT_STATUS_1, MMC_SET_BLOCK_CNT, 0);
			pending &= ~MMC_SET_BLOCK_CNT;
		}

		if (pending & MMC_INACTIVE_CMD) {
			sd_writel(sd, INT_STATUS_1, MMC_INACTIVE_CMD, 0);
			pending &= ~MMC_INACTIVE_CMD;
		}

		if (pending & MMC_BLK_READ) {
			if (pending & MMC_MULTI_BLOCK_READ_WRITE) {
				if (pending & MMC_STOP_CMD) {
					pr_info("clear %d", MMC_STOP_CMD);
					sd_writel(sd, INT_STATUS_1,
						  MMC_STOP_CMD |
						  MMC_MULTI_BLOCK_READ_WRITE,
						  0);
					pending &= ~(MMC_STOP_CMD |
						MMC_MULTI_BLOCK_READ_WRITE);
					/* TBD */
				} else {
					reg_val = sd_readl(sd, ARGUMENT_REG, 0);
#ifdef BIFSD_CONFIG_X2
					if (reg_val)
						sd->rd_buf = reg_val;
#else
					if (debug)
						pr_err("MBR ARG 0x%x CNT %d\n",
						reg_val, sd->block_cnt);
					if (sd->opt_mode == OPT_MODE_SECTOR)
						reg_val = reg_val >> 9;
					if (reg_val)
						sd->rd_buf = reg_val;
					else
						sd->rd_buf = sd->addr_offset;
#endif
					sd_writel(sd, DMA_ADDR, sd->rd_buf, 0);
					sd_writel(sd, MEM_MGMT, 0x01, 0);

					sd_writel(sd, BLOCK_CNT, sd->block_cnt,
						  0);
					sd_writel(sd, INT_STATUS_1,
						  MMC_BLK_READ |
						  MMC_MULTI_BLOCK_READ_WRITE,
						  0);
					pending &= ~(MMC_BLK_READ |
						MMC_MULTI_BLOCK_READ_WRITE);
				}
			} else {
				reg_val = sd_readl(sd, ARGUMENT_REG, 0);
#ifdef BIFSD_CONFIG_X2
				if (reg_val)
					sd->rd_buf = reg_val;
#else
				if (debug)
					pr_err("BR ARG 0x%x\n", reg_val);
				if (sd->opt_mode == OPT_MODE_SECTOR)
					reg_val = reg_val >> 9;

				if (reg_val)
					sd->rd_buf = reg_val;
				else
					sd->rd_buf = sd->addr_offset;
#endif
				sd_writel(sd, DMA_ADDR, sd->rd_buf, 0);
				sd_writel(sd, MEM_MGMT, 0x01, 0);
				sd_writel(sd, BLOCK_CNT, 0x01, 0);
				sd_writel(sd, INT_STATUS_1, MMC_BLK_READ, 0);
				pending &= ~MMC_BLK_READ;
			}
		}

		if (pending & MMC_READ_BLOCK_CNT) {
			sd->state = STATE_TX_DATA_COMP;
			sd_writel(sd, INT_STATUS_1, MMC_READ_BLOCK_CNT, 0);
			pending &= ~MMC_READ_BLOCK_CNT;
		}

		if (pending & MMC_STOP_CMD) {
			sd_writel(sd, DMA_ADDR, 0, 0);

			reg_val = sd_readl(sd, PROGRAM_REG, 0);
			if (reg_val & BIT(3))
				sd_writel(sd, MEM_MGMT, 0x21, 0);

			sd_writel(sd, INT_STATUS_1, MMC_STOP_CMD, 0);
			pending &= ~MMC_STOP_CMD;
		}

		if (pending & MMC_BLK_WRITE) {
			if (pending & MMC_MULTI_BLOCK_READ_WRITE) {
				reg_val = sd_readl(sd, ARGUMENT_REG, 0);
#ifdef BIFSD_CONFIG_X2
				if (reg_val)
					sd->wr_buf = reg_val;
#else
				if (debug)
					pr_err("MBW ARG 0x%x CNT %d\n",
					reg_val, sd->block_cnt);
				if (sd->opt_mode == OPT_MODE_SECTOR)
					reg_val = reg_val >> 9;

				if (reg_val)
					sd->wr_buf = reg_val;
				else
					sd->wr_buf = sd->addr_offset;
#endif
				sd_writel(sd, DMA_ADDR, sd->wr_buf, 0);
				sd_writel(sd, MEM_MGMT, 0x01, 0);

				sd_writel(sd, BLOCK_CNT, sd->block_cnt, 0);
				sd_writel(sd, INT_STATUS_1,
					  MMC_BLK_WRITE |
					  MMC_MULTI_BLOCK_READ_WRITE, 0);
				pending &= ~(MMC_BLK_WRITE |
					  MMC_MULTI_BLOCK_READ_WRITE);
			} else {
				reg_val = sd_readl(sd, ARGUMENT_REG, 0);
#ifdef BIFSD_CONFIG_X2
				if (reg_val)
					sd->wr_buf = reg_val;
#else
				if (debug)
					pr_err("BW ARG 0x%x\n", reg_val);
				if (sd->opt_mode == OPT_MODE_SECTOR)
					reg_val = reg_val >> 9;

				if (reg_val)
					sd->wr_buf = reg_val;
				else
					sd->wr_buf = sd->addr_offset;
#endif
				sd_writel(sd, DMA_ADDR, sd->wr_buf, 0);
				sd_writel(sd, MEM_MGMT, 0x01, 0);
				sd_writel(sd, BLOCK_CNT, 0x01, 0);
				sd_writel(sd, INT_STATUS_1, MMC_BLK_WRITE, 0);
				pending &= ~MMC_BLK_WRITE;
				sd->state = STATE_RX_DATA_COMP;
			}
		}

		if (pending & MMC_WRITE_BLOCK_CNT) {
			reg_val = sd_readl(sd, PROGRAM_REG, 0);
			if (reg_val & BIT(3))
				sd_writel(sd, MEM_MGMT, 0x21, 0);

			sd->state = STATE_RX_DATA_COMP;
			sd_writel(sd, INT_STATUS_1, MMC_WRITE_BLOCK_CNT, 0);
			pending &= ~MMC_WRITE_BLOCK_CNT;
		}

		if (pending & MMC_CID_UPDATE) {
			update_cid_val(sd);

			sd_writel(sd, INT_STATUS_1, MMC_CID_UPDATE, 0);
			pending &= ~MMC_CID_UPDATE;
		}

		if (pending & MMC_CSD_UPDATE) {
			sd_writel(sd, MEM_MGMT, 0x21, 0);
			sd_writel(sd, INT_STATUS_1, MMC_CSD_UPDATE, 0);
			pending &= ~MMC_CSD_UPDATE;
		}

		if (pending & MMC_NUM_WELL_WRITE_BLOCK) {
			sd_writel(sd, DMA_ADDR, sd->wr_buf, 0);
			sd_writel(sd, MEM_MGMT, 0x01, 0);
			sd_writel(sd, INT_STATUS_1, MMC_NUM_WELL_WRITE_BLOCK,
				  0);
			pending &= ~MMC_NUM_WELL_WRITE_BLOCK;
		}

		if (pending & MMC_GENERAL_READ_WRITE) {
			reg_val = sd_readl(sd, ARGUMENT_REG, 0);
			if (reg_val) {
				sd_writel(sd, DMA_ADDR, sd->rd_buf, 0);
				sd_writel(sd, MEM_MGMT, 0x01, 0);
			} else {
				sd_writel(sd, DMA_ADDR, sd->wr_buf, 0);
				sd_writel(sd, MEM_MGMT, 0x01, 0);
			}

			sd_writel(sd, INT_STATUS_1, MMC_GENERAL_READ_WRITE, 0);
			pending &= ~MMC_GENERAL_READ_WRITE;
		}

		if (pending & MMC_BLOCK_COUNT_CLEAR) {
			sd_writel(sd, INT_STATUS_1, MMC_BLOCK_COUNT_CLEAR, 0);
			pending &= ~MMC_BLOCK_COUNT_CLEAR;
		}

		if (pending & MMC_VOLTAGE_SWITCH) {
			sd_writel(sd, INT_STATUS_1, MMC_VOLTAGE_SWITCH, 0);
			pending &= ~MMC_VOLTAGE_SWITCH;
		}

		if (pending & MMC_GO_PRE_IDLE) {
			/* go pre-idel state */
			if (sd->drv_data && sd->drv_data->init) {
				ret = sd->drv_data->init(sd);
				if (ret) {
					dev_err(sd->dev,
						"implementation specific init failed\n");
				}
			}

			sd_writel(sd, INT_STATUS_1, MMC_GO_PRE_IDLE, 0);
			pending &= ~MMC_GO_PRE_IDLE;
		}

		if (pending & MMC_CMD_61) {
			reg_val = sd_readl(sd, ARGUMENT_REG, 0);
			/* need set dma address for master read */
			sd_writel(sd, DMA_ADDR, reg_val, 0);
			sd_writel(sd, MEM_MGMT, 0x01, 0);

			/* the block count must be send from master */
			sd_writel(sd, BLOCK_CNT, sd->read_blk_num, 0);
			sd_writel(sd, INT_STATUS_1, MMC_CMD_61, 0);
			pending &= ~MMC_CMD_61;
		}

		if (pending & MMC_CMD6_ALWAYS) {
			sd->cmd_argu = sd_readl(sd, ARGUMENT_REG, 0);

			sd_writel(sd, MEM_MGMT, 0x21, 0);
			sd_writel(sd, INT_STATUS_1, MMC_CMD6_ALWAYS, 0);
			pending &= ~MMC_CMD6_ALWAYS;
		}

		if (pending & MMC_CMD55) {
			sd->cmd_argu = sd_readl(sd, ARGUMENT_REG, 0);
			sd_writel(sd, INT_STATUS_1, MMC_CMD55, 0);
			pending &= ~MMC_CMD55;
		}

		if (pending) {
			pr_err("pending_1=%08x not process\n", pending);
			sd_writel(sd, INT_STATUS_1, pending, 0);
		}
	}

	if (pending_2 != 0) {
		pending = pending_2;
		if (pending & MMC_CMD12) {
			/* Clear Int state */
			sd_writel(sd, INT_STATUS_2, MMC_CMD12, 0);
			pending &= ~MMC_CMD12;
		}

		if (pending & MMC_DATA_CRC_ERR) {
			if (!debug)
				pr_err("%08x\n", pending_2);
			/* Clear Int state */
			sd_writel(sd, INT_STATUS_2, MMC_DATA_CRC_ERR, 0);
			pending &= ~MMC_DATA_CRC_ERR;
		}
		if (pending & MMC_CMD_CRC_ERR) {
			if (!debug)
				pr_err("%08x\n", pending_2);
			/* Clear Int state */
			sd_writel(sd, INT_STATUS_2, MMC_CMD_CRC_ERR, 0);
			pending &= ~MMC_CMD_CRC_ERR;
		}

		if (pending & MMC_SLEEP_CMD) {
			/* Enter sleep state */
			sd_writel(sd, MEM_MGMT, 0x21, 0);
			sd_writel(sd, INT_STATUS_2, MMC_SLEEP_CMD, 0);
			pending &= ~MMC_SLEEP_CMD;
		}

		if (pending & MMC_AWAKE_CMD) {
			/* Enter Standby state */
			sd_writel(sd, MEM_MGMT, 0x21, 0);
			sd_writel(sd, INT_STATUS_2, MMC_AWAKE_CMD, 0);
			pending &= ~MMC_AWAKE_CMD;
		}

		if (pending & MMC_UPDATE_EXT_CSD) {
			u32 old_val;
			u32 val;
			unsigned char val_offset;
			u32 new_val;
			u32 update_val;
			u32 update_addr = 0;

			val = sd_readl(sd, ARGUMENT_REG, 0);
			val &= 0x00FF0000;
			val = val >> 16;

			val_offset = val % 4;
			if (val_offset) {
				val -= val_offset;
			}
			if (val >= 128) {
				update_addr = val - 48;
			} else if (val < 80) {
				update_addr = val;
			} else {
				dev_err(sd->dev,
					"update ext csd register error\n");
			}

			update_val = sd_readl(sd, UPDATE_EXT_CSD, 0);
			old_val = sd_readl(sd, EXTENDED_CSD, update_addr);

			old_val = old_val & (~(0xFF << val_offset * 8));
			new_val = update_val << val_offset * 8;
			new_val = old_val | new_val;

			sd_writel(sd, EXTENDED_CSD, new_val, update_addr);

			sd_writel(sd, MEM_MGMT, 0x21, 0);
			sd_writel(sd, INT_STATUS_2, MMC_UPDATE_EXT_CSD, 0);
			pending &= ~MMC_UPDATE_EXT_CSD;
		}

		if (pending & MMC_HARDWARE_RESET) {
			sd_writel(sd, INT_STATUS_2, MMC_HARDWARE_RESET, 0);
			pending &= ~MMC_HARDWARE_RESET;
		}

		if (pending & MMC_CARD_SELECT) {
			/* The core moves from stand by state to transfer state */
			sd_writel(sd, INT_STATUS_2, MMC_CARD_SELECT, 0);
			pending &= ~MMC_CARD_SELECT;
		}

		if (pending & MMC_CARD_STATUS_CHANGE) {
			/* The core moves from stand by state to transfer state */
			sd_writel(sd, INT_STATUS_2, MMC_CARD_STATUS_CHANGE, 0);
			pending &= ~MMC_CARD_STATUS_CHANGE;
		}

		if (pending) {
			pr_err("pending_2=%08x not process\n", pending);
			sd_writel(sd, INT_STATUS_2, pending, 0);
		}
	}

	return IRQ_HANDLED;
}

int bifsd_pltfm_register(struct platform_device *pdev,
			 const struct bifsd_drv_data *drv_data)
{
	struct resource *regs;
	struct resource mem_reserved;
	struct resource *sysctrl;
	struct device_node *np = NULL;
	struct bif_sd *sd;
	int ret;
	int cd_gpio = 0;
	unsigned int value32;

	sd = devm_kzalloc(&pdev->dev, sizeof(struct bif_sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	sd->irq = platform_get_irq(pdev, 0);
	if (sd->irq < 0)
		return sd->irq;

	sd->drv_data = drv_data;
	sd->dev = &pdev->dev;
	sd->irq_flags = 0;
	sd->range_addr_max = OUT_RANGE_ADDR;
	sd->work_mode = WORK_MODE_DEFAULT;
	sd->opt_mode = OPT_MODE_DEFAULT;
	sd->capacity = CAPACITY_DEFAULT;
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sd->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(sd->regs))
		return PTR_ERR(sd->regs);

	sysctrl = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	sd->sysctrl_reg = ioremap(sysctrl->start, 0x10);
	if (IS_ERR(sd->sysctrl_reg))
		return PTR_ERR(sd->sysctrl_reg);
	/* Get registers' physical base address */
	sd->phy_regs = regs->start;
	ret = of_property_read_u32(pdev->dev.of_node,
				   "range_addr_max", &value32);
#if 0
	if (ret == 0)
		sd->range_addr_max = value32;

	ret = of_property_read_u32(pdev->dev.of_node, "work_mode", &value32);
	if (ret == 0) {
		if (value32)
			sd->work_mode = WORK_MODE_BYPASS;
		else
			sd->work_mode = WORK_MODE_ACC;
	}
	ret = of_property_read_u32(pdev->dev.of_node, "opt_mode", &value32);
	if (ret == 0) {
		if (value32)
			sd->opt_mode = OPT_MODE_SECTOR;
		else
			sd->opt_mode = OPT_MODE_BYTE;
	}
	if (sd->opt_mode == OPT_MODE_BYTE)
		sd->addr_offset = ZERO_ADDR_OFFSET;
	else
		sd->addr_offset = ZERO_ADDR_OFFSET >> 9;

	ret = of_property_read_u32(pdev->dev.of_node, "capacity", &value32);
	if (ret == 0) {
		if (value32 > 512)
			sd->capacity = value32;
	}
#endif
	/* request memory address */
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");
	}

	ret = of_address_to_resource(np, 0, &mem_reserved);
	if (ret) {
		dev_err(&pdev->dev,
			"No memory address assigned to the region\n");
	} else {
		sd->paddr = (void __iomem *)mem_reserved.start;
		sd->vaddr =
		    (void __iomem *)memremap(mem_reserved.start,
					     resource_size(&mem_reserved),
					     MEMREMAP_WB);
		dev_err(&pdev->dev,
			"Allocate reserved memory, vaddr: 0x%0llx, paddr: 0x%0llx\n",
			(uint64_t) sd->paddr, (uint64_t) sd->vaddr);
	}
	sd->aclk = devm_clk_get(&pdev->dev, "sys_bifsd_aclk");
	if (IS_ERR(sd->aclk) || (sd->aclk == NULL)) {
		sd->aclk = NULL;
		dev_err(&pdev->dev, "get bif_sd aclk failed\n");
		return 1;
	}
	bifsd_clk_on(sd);

	//if (!device_property_read_u32(sd->dev, "cd-gpio", &cd_gpio))
	//	sd->cd_gpio = cd_gpio;
	set_bifsd_info(sd);
	platform_set_drvdata(pdev, sd);

	sd->rst = devm_reset_control_get(&pdev->dev, "bifsd");
	if (IS_ERR(sd->rst)) {
		dev_err(&pdev->dev, "missing controller reset\n");
		return PTR_ERR(sd->rst);
	}

	bifsd_reset(sd);

	if (drv_data && drv_data->init) {
		ret = drv_data->init(sd);
		if (ret) {
			dev_err(sd->dev,
				"implementation specific init failed\n");
			devm_kfree(&pdev->dev, sd);
			goto err;
		}
	}
	ret = devm_request_irq(sd->dev, sd->irq, bifsd_interrupt,
			       sd->irq_flags, "bifsd", sd);
	if (ret)
		goto err;

	if (cd_gpio) {
		gpio_request(sd->cd_gpio, NULL);
		gpio_direction_output(sd->cd_gpio, 1);
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &bifsd_group);
	if (ret != 0)
		dev_err(&pdev->dev, "sysfs_create_group failed\n");

err:
	return ret;
}

EXPORT_SYMBOL_GPL(bifsd_pltfm_register);

static int bifsd_probe(struct platform_device *pdev)
{
	int ret;

	const struct bifsd_drv_data *drv_data;
	const struct of_device_id *match;


	match = of_match_node(bifsd_hobot_of_match, pdev->dev.of_node);
	drv_data = match->data;
	ret = bifsd_pltfm_register(pdev, drv_data);
	if (ret) {
		dev_err(&pdev->dev, "Err %s\n", VER);
	} else {
		/* diag */
		diag_register(ModuleDiag_bif, EventIdBifSdErr,
					8, 300, 5000, NULL);
		bifsd_last_err_tm_ms = 0;
		init_timer(&bifsd_diag_timer);
		bifsd_diag_timer.expires = get_jiffies_64()
						+ msecs_to_jiffies(1000);
		bifsd_diag_timer.data = 0;
		bifsd_diag_timer.function = bifsd_diag_timer_func;
		add_timer(&bifsd_diag_timer);
		dev_err(&pdev->dev, "Suc %s\n", VER);
	}

	return ret;
}

static int bifsd_remove(struct platform_device *pdev)
{
	struct bif_sd *sd = platform_get_drvdata(pdev);

	dev_err(&pdev->dev, "remove\n");
	if(sd->cd_gpio){
		gpio_direction_output(sd->cd_gpio, 0);
		gpio_free(sd->cd_gpio);
	}

	/* Disable interrupt */
	sd_writel(sd, INT_STATUS_1, 0xFFFFFFFF, 0);
	sd_writel(sd, INT_STATUS_2, 0xFFFFFFFF, 0);
	sd_writel(sd, INT_ENABLE_1, 0, 0);
	sd_writel(sd, INT_ENABLE_2, 0, 0);

	bifsd_clk_off(sd);

	devm_kfree(&pdev->dev, sd);
	del_timer_sync(&bifsd_diag_timer);
	return 0;
}

EXPORT_SYMBOL(bifsd_remove);

#ifdef CONFIG_PM_SLEEP
int bifsd_suspend(struct device *dev)
{
	struct bif_sd *sd = dev_get_drvdata(dev);

	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

	/* Disable interrupt */
	sd_writel(sd, INT_STATUS_1, 0xFFFFFFFF, 0);
	sd_writel(sd, INT_STATUS_2, 0xFFFFFFFF, 0);
	sd_writel(sd, INT_ENABLE_1, 0, 0);
	sd_writel(sd, INT_ENABLE_2, 0, 0);

	return 0;
}

int bifsd_resume(struct device *dev)
{
	struct bif_sd *sd = dev_get_drvdata(dev);

	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);

	bifsd_reset(sd);

	bifsd_hobot_priv_init(sd);

	if (sd->cd_gpio) {
		gpio_request(sd->cd_gpio, NULL);
		gpio_direction_output(sd->cd_gpio, 1);
	}

	return 0;
}
#endif

static struct dev_pm_ops bifsd_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bifsd_suspend,
			bifsd_resume)
};

static struct platform_driver bifsd_hobot_driver = {
	.driver = {
		   .name = "bifsd",
		   .of_match_table = bifsd_hobot_of_match,
		   .pm = &bifsd_dev_pm_ops,
		   },
	.probe = bifsd_probe,
	.remove = bifsd_remove,
};

module_platform_driver(bifsd_hobot_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hobot Inc.");
MODULE_DESCRIPTION("Driver for the HobotRobotics Bifsd Controller");
MODULE_VERSION("2.0");
