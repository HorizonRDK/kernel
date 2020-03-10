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
#include <linux/device.h>
#include <linux/reset.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>
#include "bpu.h"
#include "bpu_core.h"
#include "x2_bpu.h"

#define DEFAULT_BURST_LEN 0x80

static inline uint32_t x2_bpu_reg_read(struct bpu_core *core, uint32_t offset)
{
	return readl(core->base + offset);
}

static inline void x2_bpu_reg_write(struct bpu_core *core,
		uint32_t offset, uint32_t val)
{
	writel(val, core->base + offset);
}

static void x2_cnnbus_wm_set(struct bpu_core *core, uint32_t reg_off,
	uint32_t wd_maxlen, uint32_t wd_endian, uint32_t wd_priority)
{
	uint32_t reg_val;

	reg_val = x2_bpu_reg_read(core, reg_off);
	reg_val &= ~(X2_CNN_WD_MAXLEN_M_MASK |
		X2_CNN_WD_ENDIAN_M_MASK |
		X2_CNN_WD_PRIORITY_M_MASK);

	reg_val |= X2_CNN_WD_MAXLEN_M(wd_maxlen) |
		X2_CNN_WD_ENDIAN_M(wd_endian) |
		X2_CNN_WD_PRIORITY_M(wd_priority);

	x2_bpu_reg_write(core, reg_off, reg_val);
}

static void x2_cnnbus_rm_set(struct bpu_core *core, uint32_t reg_off,
	uint32_t rd_maxlen, uint32_t rd_endian, uint32_t rd_priority)
{
	uint32_t reg_val;

	reg_val = x2_bpu_reg_read(core, reg_off);
	reg_val &= ~(X2_CNN_RD_MAXLEN_M_MASK |
		X2_CNN_RD_ENDIAN_M_MASK |
		X2_CNN_RD_PRIORITY_M_MASK);

	reg_val |= X2_CNN_RD_MAXLEN_M(rd_maxlen) |
		X2_CNN_RD_ENDIAN_M(rd_endian) |
		X2_CNN_RD_PRIORITY_M(rd_priority);

	x2_bpu_reg_write(core, reg_off, reg_val);
}

static int32_t x2_bpu_hw_init(struct bpu_core *core)
{

	if (!core->reserved[0])
		core->reserved[0] = DEFAULT_BURST_LEN;
	/* Config axi write master */
	x2_cnnbus_wm_set(core, CNNBUS_CTRL_WM_0, core->reserved[0], 0xf, 0x1);
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

static int32_t x2_bpu_iso_clear(struct bpu_core *core)
{
	void __iomem *bpu_pmu_base;
	uint32_t reg_val;
	int32_t ret = 0;

	if (core == NULL)
		return -ENODEV;

	bpu_pmu_base = ioremap(BPU_PMU_REG, 4);
	if (bpu_pmu_base == NULL)
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

static int32_t x2_bpu_iso_set(struct bpu_core *core)
{
	void __iomem *bpu_pmu_base;
	uint32_t reg_val;
	int32_t ret = 0;

	if (core == NULL)
		return -ENODEV;

	bpu_pmu_base = ioremap(BPU_PMU_REG, 4);
	if (bpu_pmu_base == NULL)
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

static int32_t bpu_to_reset(struct reset_control *rst, int32_t time)
{
	int32_t ret = 0;

	if (rst == NULL) {
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

static int32_t x2_bpu_reset(struct bpu_core *core)
{
	int32_t ret;

	if (core == NULL) {
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

static int32_t x2_bpu_enable(struct bpu_core *core)
{
	uint32_t tmp_fc_depth;
	uint32_t reg_val;
	int32_t ret;

	if (core == NULL) {
		pr_err("Enable invalid bpu core!\n");
		return -ENODEV;
	}

	if (core->fc_base) {
		dev_err(core->dev, "bpu core already enable\n");
		return 0;
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

	/* The following to init fc info */

	core->fc_base = dma_alloc_coherent(core->dev,
			FC_SIZE * FC_DEPTH, &core->fc_base_addr, GFP_KERNEL);
	if (!core->fc_base) {
		dev_err(core->dev, "bpu core alloc fc mem failed\n");
		return -ENOMEM;
	}

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
	reg_val |= X2_CNN_PE0_FC_BASE((uint32_t)core->fc_base_addr);

	x2_bpu_reg_write(core, CNN_FC_BASE, reg_val);

	return 0;
}

static int32_t x2_bpu_disable(struct bpu_core *core)
{
	int32_t ret = 0;
	
	if (core == NULL) {
		pr_err("Disable invalid bpu core!\n");
		return -ENODEV;
	}
	if (core->fc_base == NULL) {
		dev_err(core->dev, "bpu core already disabled\n");
		return 0;
	}

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
	if (core->fc_base) {
		dma_free_coherent(core->dev, FC_SIZE * FC_DEPTH,
				core->fc_base, core->fc_base_addr);
		core->fc_base = NULL;
	}

	return ret;
}

static int32_t x2_bpu_set_clk(struct bpu_core *core, uint64_t rate)
{
	uint64_t last_rate;
	int32_t ret = 0;

	if (core == NULL) {
		pr_err("Set invalid bpu core clk!\n");
		return -ENODEV;
	}

	if (core->mclk == NULL)
		return ret;

	last_rate = clk_get_rate(core->mclk);

	if (last_rate == rate)
		return 0;

	ret = clk_set_rate(core->mclk, rate);
	if (ret) {
		dev_err(core->dev, "Cannot set frequency %llu (%d)\n",
				rate, ret);
		return ret;
	}

	/* check if rate set success, when not, user need recover volt */
	if (clk_get_rate(core->mclk) != rate) {
		dev_err(core->dev,
				"Get wrong frequency, Request %llu, Current %lu\n",
				rate, clk_get_rate(core->mclk));
		return -EINVAL;
	}

	return ret;
}

static int32_t x2_bpu_set_volt(struct bpu_core *core, int32_t volt)
{
	int32_t ret = 0;

	if (core == NULL) {
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

static void x2_bpu_set_update_tail(struct bpu_core *core, uint32_t tail_index)
{
	uint32_t tmp_reg;

	tmp_reg = x2_bpu_reg_read(core, CNN_FC_TAIL);
	tmp_reg &= ~(X2_CNN_PE0_FC_TAIL_MASK);

	tmp_reg |= X2_CNN_PE0_FC_TAIL(tail_index);
	x2_bpu_reg_write(core, CNN_FC_TAIL, tmp_reg);
}

static int32_t x2_bpu_fc_equeue(struct bpu_core *core,
		void *fc_data, int32_t *fc_num)
{
	int32_t free_fc_fifo = 0;
	uint32_t head_index, tail_index,
	    fc_head_flag, fc_tail_flag;
	uint32_t fc_depth, insert_fc_cnt, residue_fc_cnt;
	uint32_t ret = 0;
	uint32_t count;

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

	if (core->fc_buf_limit > 0) {
		free_fc_fifo =
			core->fc_buf_limit - (fc_depth + 1 - free_fc_fifo);
		/* running fc num need not larger then limit */
		if(free_fc_fifo <= 0)
			return -EBUSY;
	}

	if (*fc_num > free_fc_fifo)
		*fc_num = free_fc_fifo;

	count = tail_index;

	if ((tail_index + *fc_num) > fc_depth) {
		insert_fc_cnt = fc_depth - tail_index + 1;
		memcpy(core->fc_base + tail_index * X2_CNN_FC_SIZE,
			fc_data, insert_fc_cnt * X2_CNN_FC_SIZE);

		if (fc_tail_flag)
			fc_tail_flag = 0;
		else
			fc_tail_flag = X2_CNN_FC_IDX_FLAG;

		residue_fc_cnt = *fc_num - insert_fc_cnt;
		if (residue_fc_cnt > 0) {
			memcpy(core->fc_base,
			fc_data + (insert_fc_cnt * X2_CNN_FC_SIZE),
			residue_fc_cnt * X2_CNN_FC_SIZE);
		}

		ret = fc_tail_flag | residue_fc_cnt;
	} else {
		memcpy(core->fc_base + (tail_index * X2_CNN_FC_SIZE),
			fc_data, *fc_num * X2_CNN_FC_SIZE);

		ret = fc_tail_flag | (tail_index + *fc_num);
	}

	return ret;
}

static int32_t x2_bpu_write_fc(struct bpu_core *core,
		struct bpu_fc *fc, uint32_t offpos)
{
	uint16_t *tmp_fc_id;
	int32_t update_tail;
	int32_t fc_num;

	if (core == NULL) {
		pr_err("Write invalid bpu core!\n");
		return -ENODEV;
	}

	if (core->fc_base == NULL) {
		dev_err(core->dev, "bpu core not enable\n");
		return -ENODEV;
	}

	if ((fc == NULL)|| (fc->fc_data == NULL)) {
		dev_err(core->dev, "bpu core write invalid fc\n");
		return -EINVAL;
	}

	if (offpos >= fc->info.slice_num) {
		dev_err(core->dev, "bpu fc write invalid offset, \
				offpos = %d, slice_num = %d\n",
				offpos, fc->info.slice_num);
		return -EINVAL;
	}

	fc_num = fc->info.slice_num - offpos;

	/* set the last hw fc id to upper set */
	tmp_fc_id = (uint16_t *)(fc->fc_data + (offpos * X2_CNN_FC_SIZE)
			+ ((fc_num - 1) * FC_SIZE  + FC_ID_OFFSET));
	*tmp_fc_id = fc->hw_id;

	update_tail = x2_bpu_fc_equeue(core,
			fc->fc_data + (offpos * X2_CNN_FC_SIZE), &fc_num);
	if (update_tail < 0) {
		return update_tail;
	}

	fc->index = (update_tail & (~X2_CNN_FC_IDX_FLAG)) - 1;

	x2_bpu_set_update_tail(core, update_tail);

	return fc_num;
}

static int32_t x2_bpu_read_fc(struct bpu_core *core,
		uint32_t *tmp_id, uint32_t *err)
{
	uint32_t irq_status;
	int32_t ret;

	if (core == NULL) {
		pr_err("Read invalid bpu core!\n");
		return -ENODEV;
	}

	/* the status just need read on X2 */
	irq_status = x2_bpu_reg_read(core, CNNINT_STATUS);

	x2_bpu_reg_write(core, CNNINT_MASK, 0x1);
	*tmp_id = x2_bpu_reg_read(core, CNNINT_NUM);
	x2_bpu_reg_write(core, CNNINT_MASK, 0x0);

	if (*tmp_id & 0xf000)
		*err = *tmp_id;
	else
		*err = 0;

	ret = 1;

	return ret;
}

static int32_t x2_bpu_status(struct bpu_core *core, uint32_t cmd)
{
	uint32_t head_index, tail_index;

	if (core == NULL) {
		pr_err("Get Status from invalid bpu core!\n");
		return -ENODEV;
	}

	head_index = x2_bpu_reg_read(core, CNN_FC_HEAD);

	tail_index = x2_bpu_reg_read(core, CNN_FC_TAIL);

	if (head_index == tail_index)
		return 0;

	return 1;
}

/* x2 use core reserved[0] to store burst_len set */
static ssize_t bpu_core_burst_len_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bpu_core *core = dev_get_drvdata(dev);

	if (core->reserved[0] == 0)
		core->reserved[0] = DEFAULT_BURST_LEN;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			(int)(core->reserved[0] * 16));
}

static ssize_t bpu_core_burst_len_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct bpu_core *core = dev_get_drvdata(dev);
	int32_t ret;
	int32_t tmp_val;

	ret = sscanf(buf, "%du", &tmp_val);
	if (ret < 0) {
		dev_err(core->dev, "BPU core%d sscanf burst error\n",
				core->index);
		return 0;
	}

	if (tmp_val % 16 > 0) {
		dev_err(core->dev, "burst len must align 16");
	}

	core->reserved[0] = tmp_val / 16;

	if (core->reserved[0] <= 0)
		core->reserved[0] = 1;
	else if (core->reserved[0] > 0x80)
		core->reserved[0] = 0x80;

	dev_info(core->dev, "BPU core%d set burst len:%d\n",
			core->index, (int)core->reserved[0]);

	return len;
}

static DEVICE_ATTR(burst_len, S_IRUGO | S_IWUSR,
		bpu_core_burst_len_show, bpu_core_burst_len_store);

static struct attribute *bpu_core_hw_attrs[] = {
	&dev_attr_burst_len.attr,
	NULL,
};

static struct attribute_group bpu_core_hw_attr_group = {
	.attrs = bpu_core_hw_attrs,
};

static int32_t x2_bpu_debug(struct bpu_core *core, int32_t state)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("NO bpu core for debug!\n");
		return -ENODEV;
	}

	if (state == 1) {
		ret = device_add_group(core->dev, &bpu_core_hw_attr_group);
		if (ret < 0) {
			dev_err(core->dev, "Create bpu core%d hw debug group failed\n",
					core->index);
			return ret;
		}
	}

	if (state == 0)
		device_remove_group(core->dev, &bpu_core_hw_attr_group);

	return 0;
}

struct bpu_core_hw_ops x2_hw_ops = {
	.enable		= x2_bpu_enable,
	.disable	= x2_bpu_disable,
	.reset		= x2_bpu_reset,
	.set_clk	= x2_bpu_set_clk,
	.set_volt	= x2_bpu_set_volt,
	.write_fc	= x2_bpu_write_fc,
	.read_fc	= x2_bpu_read_fc,
	.status		= x2_bpu_status,
	.debug		= x2_bpu_debug,
};

MODULE_DESCRIPTION("Driver for Horizon X2/J2 SOC BPU");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
