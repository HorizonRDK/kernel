/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/io.h>
#include "vio_hw_common_api.h"
#include "vio_config.h"

#define pr_fmt(fmt) "vio_hw_common_api: " fmt
//#define DEBUG_HW_SFR
u32 vio_hw_get_reg(void __iomem *base_addr, const struct vio_reg_def *reg)
{
	u32 reg_value;

#if CONFIG_QEMU_TEST
	reg_value = *(u32 *) ((u8 *) base_addr + reg->sfr_offset);
#else
	reg_value = readl(base_addr + reg->sfr_offset); /*PRQA S 0497,1006,1021,3238*/
#endif

#ifdef DEBUG_HW_SFR
	vio_info("[GET_REG] reg:[%s][0x%04X], reg_value(R):[0x%08X]\n",
			reg->reg_name, reg->sfr_offset, reg_value);

#endif /*  */
	return reg_value;
}
EXPORT_SYMBOL(vio_hw_get_reg);

void vio_hw_set_reg(void __iomem *base_addr, const struct vio_reg_def *reg,
		u32 val)
{

#ifdef DEBUG_HW_SFR
	vio_info("[SET_REG] reg:[%s][0x%04X], reg_value(W):[0x%08X]\n",
			reg->reg_name, reg->sfr_offset, val);

#endif /*  */
#if CONFIG_QEMU_TEST
	*(u32 *) ((u8 *) base_addr + reg->sfr_offset) = val;
#else
	writel(val, base_addr + reg->sfr_offset); /*PRQA S 0431,0497,1006,1021,3238*/
#endif
}
EXPORT_SYMBOL(vio_hw_set_reg);

u32 vio_hw_get_field(void __iomem *base_addr,
		const struct vio_reg_def *reg,
		const struct vio_field_def *field)
{
	u32 reg_value;
	u32 field_value;

#if CONFIG_QEMU_TEST
	reg_value = *(u32 *) ((u8 *) base_addr + reg->sfr_offset);
#else
	reg_value = readl(base_addr + reg->sfr_offset); /*PRQA S 0497,1006,1021,3238*/
#endif

	field_value = vio_hw_get_field_value(reg_value, field);

#ifdef DEBUG_HW_SFR
	vio_info("[GET_FIELD] reg:[%s][0x%04X], field:[0x%08X] reg_value(R):[0x%08X] val(R):[%d]\n",
		reg->reg_name, reg->sfr_offset, field->reg, reg_value, field_value);
#endif
	return field_value;
}
EXPORT_SYMBOL(vio_hw_get_field);

void vio_hw_set_owner_field(void __iomem *base_addr,
		const struct vio_reg_def *reg,
		const struct vio_field_def *field, u32 val)
{
	u32 reg_value = 0;
	u32 pre_value;

	/* previous value reading */
#if CONFIG_QEMU_TEST
	reg_value = *(u32 *) ((u8 *) base_addr + reg->sfr_offset);

#else
	pre_value = readl(base_addr + (reg->sfr_offset)); /*PRQA S 0497,1006,1021,3238*/
#endif

#ifdef DEBUG_HW_SFR
	vio_info("[SET_FIELD] reg:[%s][0x%04X], field:[0x%08X] reg_value(W):[0x%08X] val(W):[%d] pre_value[0x%x]\n",
		reg->reg_name, reg->sfr_offset, field->reg, reg_value, val, pre_value);
#endif /*  */

	reg_value = vio_hw_set_field_value(reg_value, field, val);
	/* store reg value */
#if CONFIG_QEMU_TEST
	*(u32 *) (base_addr + reg->sfr_offset) = reg_value;
#else
	writel(reg_value, base_addr + (reg->sfr_offset)); /*PRQA S 0431,0497,1006,1021,3238*/
#endif
}
EXPORT_SYMBOL(vio_hw_set_owner_field);

/* need protect in caller */
void vio_hw_set_field(void __iomem *base_addr,
		const struct vio_reg_def *reg,
		const struct vio_field_def *field, u32 val)
{
	u32 reg_value;
	u32 pre_value;

	/* previous value reading */
#if CONFIG_QEMU_TEST
	reg_value = *(u32 *) ((u8 *) base_addr + reg->sfr_offset);

#else
	reg_value = readl(base_addr + (reg->sfr_offset)); /*PRQA S 0497,1006,1021,3238*/
#endif

	pre_value = reg_value;

	reg_value = vio_hw_set_field_value(reg_value, field, val);

#ifdef DEBUG_HW_SFR
	vio_info("[SET_FIELD] reg:[%s][0x%04X], field:[0x%08X] reg_value(W):[0x%08X] val(W):[%d] pre_value[0x%x]\n",
		reg->reg_name, reg->sfr_offset, field->reg, reg_value, val, pre_value);

#endif /*  */
	/* store reg value */
#if CONFIG_QEMU_TEST
	*(u32 *) (base_addr + reg->sfr_offset) = reg_value;
#else
	writel(reg_value, base_addr + (reg->sfr_offset)); /*PRQA S 0431,0497,1006,1021,3238*/
#endif
}
EXPORT_SYMBOL(vio_hw_set_field);

u32 vio_hw_get_field_value(u32 reg_value,
		const struct vio_field_def *field)
{
	u32 field_mask = 0;
	u32 field_value = 0;

	field_mask = (field->bit_width >= 32) ?
		0xFFFFFFFF : ((1 << field->bit_width) - 1);
	field_value = (reg_value >> (field->bit_start)) & (field_mask);

	return field_value;
}
EXPORT_SYMBOL(vio_hw_get_field_value);

u32 vio_hw_set_field_value(u32 reg_value, const struct vio_field_def *field,
		u32 val)
{
	u32 field_mask = 0;

	field_mask =(field->bit_width >= 32) ?
		0xFFFFFFFF : ((1 << field->bit_width) - 1);

	/* bit clear */
	reg_value &= ~(field_mask << field->bit_start);

	/* setting value */
	reg_value |= (val & field_mask) << (field->bit_start);

	return reg_value;
}
EXPORT_SYMBOL(vio_hw_set_field_value);

void vio_hw_dump_regs(void __iomem *base_addr,
		const struct vio_reg_def *regs, u32 total_cnt)
{
	u32 i = 0;
	u32 reg_value = 0;

	for(i = 0; i < total_cnt; i++) {
		reg_value = readl(base_addr + regs[i].sfr_offset); /*PRQA S 0497,1006,1021,3238*/
		vio_info("[DUMP] reg:[%s][0x%04X], value:[0x%08X]\n",
			regs[i].reg_name, regs[i].sfr_offset, reg_value);
	}
}
EXPORT_SYMBOL(vio_hw_dump_regs);
