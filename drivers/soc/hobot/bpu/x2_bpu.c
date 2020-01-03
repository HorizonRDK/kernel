/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * Zhang Guoying <guoying.zhang@horizon.ai>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/types.h>
#include <linux/slab.h>
#include <asm/delay.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>
#include "bpu.h"
#include "bpu_core.h"
#include "x2_bpu.h"

#define USE_KMALLOC_FC
#undef USE_KMALLOC_FC

#define CNN_MT_WB 0x1
#define CNN_MT_UC 0x2
#define CNN_MT_WC 0x3
#define CNN_MT_WT 0x4

static void *bpu_ram_vmap(phys_addr_t start, size_t size,
		unsigned int memtype)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	switch (memtype) {
	case CNN_MT_WB:
		prot = PAGE_KERNEL;
		break;
	case CNN_MT_UC:
		prot = pgprot_noncached(PAGE_KERNEL);
		break;
	case CNN_MT_WC:
		prot = pgprot_writecombine(PAGE_KERNEL);
		break;
	case CNN_MT_WT:
		prot = __pgprot(PROT_NORMAL_WT);
		break;
	default:
		/* Default set normal memory(cacheable) */
		prot = PAGE_KERNEL;
	}

	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n",
		       __func__, page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;

		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vm_map_ram(pages, page_count, -1, prot);
	kfree(pages);

	return vaddr;
}

static inline u32 x2_bpu_reg_read(struct bpu_core *core, u32 offset)
{
	return readl(core->base + offset);
}

static inline void x2_bpu_reg_write(struct bpu_core *core, u32 offset, u32 val)
{
	writel(val, core->base + offset);
}

static void x2_cnnbus_wm_set(struct bpu_core *core, u32 reg_off,
	u32 wd_maxlen, u32 wd_endian, u32 wd_priority)
{
	u32 reg_val;

	reg_val = x2_bpu_reg_read(core, reg_off);
	reg_val &= ~(X2_CNN_WD_MAXLEN_M_MASK |
		X2_CNN_WD_ENDIAN_M_MASK |
		X2_CNN_WD_PRIORITY_M_MASK);

	reg_val |= X2_CNN_WD_MAXLEN_M(wd_maxlen) |
		X2_CNN_WD_ENDIAN_M(wd_endian) |
		X2_CNN_WD_PRIORITY_M(wd_priority);

	x2_bpu_reg_write(core, reg_off, reg_val);
}

static void x2_cnnbus_rm_set(struct bpu_core *core, u32 reg_off,
	u32 rd_maxlen, u32 rd_endian, u32 rd_priority)
{
	u32 reg_val;

	reg_val = x2_bpu_reg_read(core, reg_off);
	reg_val &= ~(X2_CNN_RD_MAXLEN_M_MASK |
		X2_CNN_RD_ENDIAN_M_MASK |
		X2_CNN_RD_PRIORITY_M_MASK);

	reg_val |= X2_CNN_RD_MAXLEN_M(rd_maxlen) |
		X2_CNN_RD_ENDIAN_M(rd_endian) |
		X2_CNN_RD_PRIORITY_M(rd_priority);

	x2_bpu_reg_write(core, reg_off, reg_val);
}

static int x2_bpu_hw_init(struct bpu_core *core)
{

	/* Config axi write master */
	x2_cnnbus_wm_set(core, CNNBUS_CTRL_WM_0, 0x80, 0xf, 0x1);
	x2_cnnbus_wm_set(core, CNNBUS_CTRL_WM_1, 0x80, 0xf, 0x2);
	x2_cnnbus_wm_set(core, CNNBUS_CTRL_WM_2, 0x8, 0x0, 0x3);
	x2_cnnbus_wm_set(core, CNNBUS_CTRL_WM_3, 0x80, 0x0, 0x4);

	/* Config axi read master */
	x2_cnnbus_rm_set(core, CNNBUS_CTRL_RM_0, 0x80, 0xf, 0x4);
	x2_cnnbus_rm_set(core, CNNBUS_CTRL_RM_1, 0x80, 0xf, 0x4);
	x2_cnnbus_rm_set(core, CNNBUS_CTRL_RM_2, 0x8, 0xf, 0x4);
	x2_cnnbus_rm_set(core, CNNBUS_CTRL_RM_3, 0x80, 0x8, 0x5);
	x2_cnnbus_rm_set(core, CNNBUS_CTRL_RM_4, 0x80, 0xf, 0x4);
	x2_cnnbus_rm_set(core, CNNBUS_CTRL_RM_5, 0x80, 0x0, 0x6);

	/* Set axibus id */
	x2_bpu_reg_write(core, CNNBUS_AXIID, 0x0);

	return 0;
}

static int x2_bpu_iso_clear(struct bpu_core *core)
{
	void __iomem *bpu_pmu_base;
	u32 reg_val;
	int ret = 0;

	if (!core)
		return -ENODEV;

	bpu_pmu_base = ioremap(BPU_PMU_REG, 4);
	if (!bpu_pmu_base)
		return -ENOMEM;

	reg_val = readl(bpu_pmu_base);
	reg_val &= ~BPU_ISO_BIT(core->index);
	writel(reg_val, bpu_pmu_base);

	iounmap(bpu_pmu_base);

	udelay(5);

	if (core->rst) {
		ret = reset_control_deassert(core->rst);
		if (ret < 0)
			dev_err(core->dev, "bpu core reset deassert failed\n");
	}

	return ret;
}

static int x2_bpu_iso_set(struct bpu_core *core)
{
	void __iomem *bpu_pmu_base;
	u32 reg_val;
	int ret = 0;

	if (!core)
		return -ENODEV;

	bpu_pmu_base = ioremap(BPU_PMU_REG, 4);
	if (!bpu_pmu_base)
		return -ENOMEM;

	reg_val = readl(bpu_pmu_base);
	reg_val |= BPU_ISO_BIT(core->index);
	writel(reg_val, bpu_pmu_base);

	iounmap(bpu_pmu_base);

	udelay(5);

	if (core->rst) {
		ret = reset_control_assert(core->rst);
		if (ret < 0)
			dev_err(core->dev, "bpu core reset deassert failed\n");
	}

	return ret;
}

static int bpu_to_reset(struct reset_control *rst, int time)
{
	int ret = 0;

	if (!rst) {
		pr_err("No reset ctrl null\n");
		return -ENODEV;
	}

	ret = reset_control_assert(rst);
	if (ret < 0) {
		pr_err("reset assert failed\n");
		return ret;
	}
	udelay(time);
	ret = reset_control_deassert(rst);
	if (ret < 0) {
		pr_err("reset deassert failed\n");
		return ret;
	}

	return ret;
}

static int x2_bpu_reset(struct bpu_core *core)
{
	int ret;

	if (!core) {
		pr_err("Reset invalid bpu core!\n");
		return -ENODEV;
	}

	ret = bpu_to_reset(core->rst, 1);
	if (ret) {
		dev_err(core->dev, "bpu core reset failed\n");
		return ret;
	}

	return x2_bpu_hw_init(core);
}

static int x2_bpu_enable(struct bpu_core *core)
{
	int tmp_fc_depth;
	u32 reg_val;
	int ret;

	if (!core) {
		pr_err("Enable invalid bpu core!\n");
		return -ENODEV;
	}

	if (core->fc_base) {
		dev_err(core->dev, "bpu core already enable\n");
		return -EBUSY;
	}

	if (core->aclk) {
		if (__clk_is_enabled(core->aclk))
			clk_disable_unprepare(core->aclk);
	}

	if (core->mclk) {
		if (__clk_is_enabled(core->mclk))
			clk_disable_unprepare(core->mclk);
	}

	if (core->rst) {
		ret = reset_control_assert(core->rst);
		if (ret < 0)
			dev_err(core->dev, "bpu core reset assert failed\n");
	}

	if (core->regulator) {
		ret = regulator_enable(core->regulator);
		if (ret < 0) {
			dev_err(core->dev, "bpu core power enable failed\n");
			return ret;
		}
	}

	x2_bpu_iso_clear(core);

	if (core->aclk) {
		if (!__clk_is_enabled(core->aclk)) {
			ret = clk_prepare_enable(core->aclk);
			if (ret)
				pr_info("bpu core aclk prepare error\n");

		}
	}

	if (core->mclk) {
		if (!__clk_is_enabled(core->mclk)) {
			ret = clk_prepare_enable(core->mclk);
			if (ret)
				pr_info("bpu core mclk prepare error\n");

		}
	}

	x2_bpu_reset(core);

	atomic_set(&core->hw_id_counter, 1);

	/* The following to init fc info */

#ifdef USE_KMALLOC_FC
	core->fc_base = kzalloc(FC_SIZE * FC_DEPTH, GFP_KERNEL);
#else
	if (core->index == 0)
	core->fc_base = bpu_ram_vmap(0x020c0000, FC_SIZE * FC_DEPTH + 0x2000, CNN_MT_UC);
    else
	core->fc_base = bpu_ram_vmap(0x020d1000, FC_SIZE * FC_DEPTH + 0x2000, CNN_MT_UC);
#endif
	if (!core->fc_base) {
		dev_err(core->dev, "bpu core alloc fc mem failed\n");
		return -ENOMEM;
	}
	memset(core->fc_base, 0x5a, 0x1000);
	core->fc_base += 0x1000;

	tmp_fc_depth = FC_DEPTH;

	if (tmp_fc_depth > FC_MAX_DEPTH)
		tmp_fc_depth = FC_MAX_DEPTH;

	x2_bpu_reg_write(core, CNNINT_MASK, 0x0);

	/* tell bpu fc depth */
	reg_val = x2_bpu_reg_read(core, CNN_FC_LEN);
	reg_val &=  ~(X2_CNN_PE0_FC_LENGTH_MASK);
	/* hw depth start from 0 */
	reg_val |= X2_CNN_PE0_FC_LENGTH(tmp_fc_depth - 1);
	x2_bpu_reg_write(core, CNN_FC_LEN, reg_val);

	/* tell bpu fc base */
	reg_val = x2_bpu_reg_read(core, CNN_FC_BASE);
	reg_val &=  ~(X2_CNN_PE0_FC_BASE_MASK);
	reg_val |= X2_CNN_PE0_FC_BASE(virt_to_phys(core->fc_base));

#ifndef USE_KMALLOC_FC
	if (core->index == 0)
	reg_val = 0x020c1000;
	else
	reg_val = 0x020d2000;
#endif
	x2_bpu_reg_write(core, CNN_FC_BASE, reg_val);

	return 0;
}

static int x2_bpu_disable(struct bpu_core *core)
{
	int ret = 0;
	
	if (!core) {
		pr_err("Disable invalid bpu core!\n");
		return -ENODEV;
	}

	if (!core->fc_base) {
		dev_err(core->dev, "bpu core did not enabled\n");
		return -EBUSY;
	}
	/*TODO: block write and wait run fifo process done */

	if (core->aclk) {
		if (__clk_is_enabled(core->aclk))
			clk_disable_unprepare(core->aclk);
	}

	if (core->mclk) {
		if (__clk_is_enabled(core->mclk))
			clk_disable_unprepare(core->mclk);
	}

	x2_bpu_iso_set(core);

	if (core->regulator)
		regulator_disable(core->regulator);

	/* free */
#ifdef USE_KMALLOC_FC
	kfree(core->fc_base);
#else
	vm_unmap_ram(core->fc_base - 0x1000, FC_SIZE * FC_DEPTH / PAGE_SIZE);
#endif
	core->fc_base = NULL;

	return ret;
}

static int x2_bpu_set_clk(struct bpu_core *core, unsigned long clk)
{
	int ret = 0;

	if (!core) {
		pr_err("Set invalid bpu core clk!\n");
		return -ENODEV;
	}

	ret = clk_set_rate(core->mclk, clk);
	if (ret) {
		dev_err(core->dev, "Cannot set frequency %lu\n", clk);
		return ret;
	}

	return ret;
}

static int x2_bpu_set_volt(struct bpu_core *core, int volt)
{
	int ret = 0;

	if (!core) {
		pr_err("Set invalid bpu core voltage!\n");
		return -ENODEV;
	}

	if (volt <= 0) {
		pr_err("Set invalid value bpu core voltage!\n");
		return -EINVAL;
	}

	if (core->regulator) {
		ret = regulator_set_voltage(core->regulator,
					volt, INT_MAX);
		if (ret) {
			dev_err(core->dev, "Cannot set voltage %u uV\n", volt);
			return ret;
		}
	}

	return ret;
}

static void x2_bpu_set_update_tail(struct bpu_core *core, u32 tail_index)
{
	u32 tmp_reg;

	tmp_reg = x2_bpu_reg_read(core, CNN_FC_TAIL);
	tmp_reg &= ~(X2_CNN_PE0_FC_TAIL_MASK);

	tmp_reg |= X2_CNN_PE0_FC_TAIL(tail_index);
	x2_bpu_reg_write(core, CNN_FC_TAIL, tmp_reg);
}

static int x2_bpu_fc_equeue(struct bpu_core *core, void *fc_data, int fc_num)
{
	u32 free_fc_fifo = 0;
	u32 head_index, tail_index,
	    fc_head_flag, fc_tail_flag;
	u32 fc_depth, insert_fc_cnt, residue_fc_cnt;
	u32 ret = 0;
	u32 count;

	fc_depth = x2_bpu_reg_read(core, CNN_FC_LEN);

	head_index = x2_bpu_reg_read(core, CNN_FC_HEAD);
	fc_head_flag = head_index & X2_CNN_FC_IDX_FLAG;

	tail_index = x2_bpu_reg_read(core, CNN_FC_TAIL);
	fc_tail_flag = tail_index & X2_CNN_FC_IDX_FLAG;

	head_index &= X2_CNN_MAX_FC_LEN_MASK;
	tail_index &= X2_CNN_MAX_FC_LEN_MASK;
	if (fc_head_flag != fc_tail_flag)
		free_fc_fifo = head_index - tail_index;
	else
		free_fc_fifo = fc_depth - tail_index + head_index + 1;

	if (fc_num > free_fc_fifo) {
		pr_err("no available fc fifo spaces[%d] to [%d]\n",
				fc_num, free_fc_fifo);
		return -EBUSY;
	}

	count = tail_index;

	if ((tail_index + fc_num) > fc_depth) {
		insert_fc_cnt = fc_depth - tail_index + 1;
		memcpy(core->fc_base + tail_index * X2_CNN_FC_SIZE,
			fc_data, insert_fc_cnt * X2_CNN_FC_SIZE);

		flush_fc_area(core->fc_base + tail_index * X2_CNN_FC_SIZE,
					insert_fc_cnt * X2_CNN_FC_SIZE);

		if (fc_tail_flag)
			fc_tail_flag = 0;
		else
			fc_tail_flag = X2_CNN_FC_IDX_FLAG;

		residue_fc_cnt = fc_num - insert_fc_cnt;
		if (residue_fc_cnt > 0) {
			memcpy(core->fc_base,
			fc_data + (insert_fc_cnt * X2_CNN_FC_SIZE),
			residue_fc_cnt * X2_CNN_FC_SIZE);
		}

		flush_fc_area(core->fc_base,
				residue_fc_cnt * X2_CNN_FC_SIZE);

		ret = fc_tail_flag | residue_fc_cnt;
	} else {
		memcpy(core->fc_base + (tail_index * X2_CNN_FC_SIZE),
			fc_data, fc_num * X2_CNN_FC_SIZE);

		flush_fc_area(core->fc_base + tail_index * X2_CNN_FC_SIZE,
						fc_num * X2_CNN_FC_SIZE);

		ret = fc_tail_flag | (tail_index + fc_num);
	}

	return ret;
}

static int x2_bpu_write_fc(struct bpu_core *core, struct bpu_fc *fc)
{
	uint16_t *tmp_fc_id;
	int update_tail;
	int fc_num;

	if (!core) {
		pr_err("Write invalid bpu core!\n");
		return -ENODEV;
	}

	if (!core->fc_base) {
		dev_err(core->dev, "bpu core not enable\n");
		return -ENODEV;
	}

	if (!fc || !fc->fc_data) {
		dev_err(core->dev, "bpu core write invalid fc\n");
		return -EINVAL;
	}

	fc_num = fc->info.length / X2_CNN_FC_SIZE;

	/* set the last hw fc id to upper set */
	tmp_fc_id = (uint16_t *)(fc->fc_data
			+ ((fc_num - 1) * FC_SIZE  + FC_ID_OFFSET));
	*tmp_fc_id = fc->hw_id;

	update_tail = x2_bpu_fc_equeue(core, fc->fc_data, fc_num);
	if (update_tail < 0) {
		dev_err(core->dev, "bpu core set fc error[%d]\n", update_tail);
		return update_tail;
	}

	fc->index = (update_tail & (~X2_CNN_FC_IDX_FLAG)) - 1;

	x2_bpu_set_update_tail(core, update_tail);

	return fc->info.length;
}

static int x2_bpu_read_fc(struct bpu_core *core)
{
	u32 irq_status;
	u32 tmp_id;
	int ret;

	if (!core) {
		pr_err("Read invalid bpu core!\n");
		return -ENODEV;
	}

	/* the status just need read on X2 */
	irq_status = x2_bpu_reg_read(core, CNNINT_STATUS);

	x2_bpu_reg_write(core, CNNINT_MASK, 0x1);
	tmp_id = x2_bpu_reg_read(core, CNNINT_NUM);
	x2_bpu_reg_write(core, CNNINT_MASK, 0x0);

	if (tmp_id & 0xf000)
		ret = -1 * tmp_id;
	else
		ret = tmp_id;

	return ret;
}

const struct bpu_core_hw_ops x2_hw_ops = {
	.enable		= x2_bpu_enable,
	.disable	= x2_bpu_disable,
	.reset		= x2_bpu_reset,
	.set_clk	= x2_bpu_set_clk,
	.set_volt	= x2_bpu_set_volt,
	.write_fc	= x2_bpu_write_fc,
	.read_fc	= x2_bpu_read_fc,
	.debug		= NULL,
};

MODULE_DESCRIPTION("Driver for Horizon X2/J2 SOC BPU");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
