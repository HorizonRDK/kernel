/*
 * Copyright (C) 2019 Horizon Robotics
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
#include "bpu_ctrl.h"
#include "j5_bpu.h"

/*
 * J5 has 2 fc fifo(low level fifo / high level fifo)
 * the bpu core fc base is low level fifo, high level
 * fifo not support resizer fc.
 */

#define DEFAULT_BURST_LEN (0x80u)
#define HEXBITS (16u)

static inline uint32_t j5_bpu_reg_read(const struct bpu_core *core, uint32_t offset)
{
	return readl(core->base + offset);/*PRQA S ALL*/
}

static inline void j5_bpu_reg_write(const struct bpu_core *core,
		uint32_t offset, uint32_t val)
{
	writel(val, core->base + offset);/*PRQA S ALL*/
}

static void j5_cnnbus_wm_set(const struct bpu_core *core, uint32_t reg_off,
	uint32_t wd_maxlen, uint32_t wd_endian, uint32_t wd_priority)
{
	uint32_t reg_val;

	reg_val = j5_bpu_reg_read(core, reg_off);
	reg_val &= ~(J5_CNN_WD_MAXLEN_M_MASK |
		J5_CNN_WD_ENDIAN_M_MASK |
		J5_CNN_WD_PRIORITY_M_MASK);

	reg_val |= J5_CNN_WD_MAXLEN_M(wd_maxlen) |
		J5_CNN_WD_ENDIAN_M(wd_endian) |
		J5_CNN_WD_PRIORITY_M(wd_priority);

	j5_bpu_reg_write(core, reg_off, reg_val);
}

static void j5_cnnbus_rm_set(const struct bpu_core *core, uint32_t reg_off,
	uint32_t rd_maxlen, uint32_t rd_endian, uint32_t rd_priority)
{
	uint32_t reg_val;

	reg_val = j5_bpu_reg_read(core, reg_off);
	reg_val &= ~(J5_CNN_RD_MAXLEN_M_MASK |
		J5_CNN_RD_ENDIAN_M_MASK |
		J5_CNN_RD_PRIORITY_M_MASK);

	reg_val |= J5_CNN_RD_MAXLEN_M(rd_maxlen) |
		J5_CNN_RD_ENDIAN_M(rd_endian) |
		J5_CNN_RD_PRIORITY_M(rd_priority);

	j5_bpu_reg_write(core, reg_off, reg_val);
}

static int32_t j5_bpu_hw_init(struct bpu_core *core)
{
	if (core->reserved[0] == 0u) {
		core->reserved[0] = DEFAULT_BURST_LEN;
	}
// PRQA S ALL ++
	/* Config axi write master */
	j5_cnnbus_wm_set(core, CNNBUS_CTRL_WM_0, core->reserved[0], 0xf, 0x1);
	j5_cnnbus_wm_set(core, CNNBUS_CTRL_WM_1, 0x80, 0xf, 0x2);
	j5_cnnbus_wm_set(core, CNNBUS_CTRL_WM_2, 0x8, 0x0, 0x3);
	j5_cnnbus_wm_set(core, CNNBUS_CTRL_WM_3, 0x80, 0x0, 0x4);

	/* Config axi read master */
	j5_cnnbus_rm_set(core, CNNBUS_CTRL_RM_0, 0x80, 0xf, 0x4);
	j5_cnnbus_rm_set(core, CNNBUS_CTRL_RM_1, 0x80, 0xf, 0x4);
	j5_cnnbus_rm_set(core, CNNBUS_CTRL_RM_2, 0x8, 0xf, 0x4);
	j5_cnnbus_rm_set(core, CNNBUS_CTRL_RM_3, 0x80, 0x8, 0x5);
	j5_cnnbus_rm_set(core, CNNBUS_CTRL_RM_4, 0x80, 0xf, 0x4);
	j5_cnnbus_rm_set(core, CNNBUS_CTRL_RM_5, 0x80, 0x0, 0x6);
// PRQA S ALL --
	/* Set axibus id */
	j5_bpu_reg_write(core, CNNBUS_AXIID, 0x0);

	return 0;
}

static int32_t bpu_to_reset(struct reset_control *rst, uint32_t delay_time)
{
	int32_t ret;

	if (rst == NULL) {
		pr_err("No reset ctrl null\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	ret = reset_control_assert(rst);
	if (ret < 0) {
		pr_err("reset assert failed\n");/*PRQA S ALL*/
		return ret;
	}
	udelay(delay_time);/*PRQA S ALL*/
	ret = reset_control_deassert(rst);
	if (ret < 0) {
		pr_err("reset deassert failed\n");/*PRQA S ALL*/
		return ret;
	}

	return ret;
}

static int32_t j5_bpu_reset(struct bpu_core *core)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("Reset invalid bpu core!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	ret = bpu_to_reset(core->rst, 1);
	if (ret != 0) {
		dev_err(core->dev, "bpu core reset failed\n");
		return ret;
	}

	return j5_bpu_hw_init(core);
}

static int32_t j5_bpu_enable(struct bpu_core *core)
{
	uint32_t tmp_fc_depth;
	uint32_t reg_val;
	int32_t ret;

	if (core == NULL) {
		pr_err("Enable invalid bpu core!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (core->fc_base != NULL) {
		dev_err(core->dev, "bpu core already enable\n");
		return 0;
	}

	ret = bpu_core_clk_off(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core clk disable failed\n");
	}

	if (core->rst != NULL) {
		ret = reset_control_assert(core->rst);
		if (ret < 0) {
			dev_err(core->dev, "bpu core reset assert failed\n");
		}
	}

	ret = bpu_core_power_on(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core power enable failed\n");
	}

	/* j5_bpu_iso_clear */

	ret = bpu_core_clk_on(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core clk enable failed\n");
	}

	ret = j5_bpu_reset(core);
	if (ret != 0) {
		dev_err(core->dev, "bpu core hw reset failed\n");
	}

	/* The following to init fc info */

	core->fc_base = dma_alloc_coherent(core->dev,/*PRQA S ALL*/
			FC_SIZE * FC_DEPTH, &core->fc_base_addr, GFP_KERNEL);
	if (core->fc_base == NULL) {
		dev_err(core->dev, "bpu core alloc fc mem failed\n");
		return -ENOMEM;
	}

	tmp_fc_depth = FC_DEPTH;

	j5_bpu_reg_write(core, CNNINT_MASK, 0x0);

	/* tell bpu fc depth */
	reg_val = j5_bpu_reg_read(core, CNN_FC_LEN);
	reg_val &=  ~(J5_CNN_PE0_FC_LENGTH_MASK);
	/* hw depth start from 0 */
	reg_val |= J5_CNN_PE0_FC_LENGTH(tmp_fc_depth - 1u);
	j5_bpu_reg_write(core, CNN_FC_LEN, reg_val);

	/* tell bpu fc base */
	reg_val = J5_CNN_PE0_FC_BASE((uint32_t)core->fc_base_addr);

	j5_bpu_reg_write(core, CNN_FC_BASE, reg_val);

	return 0;
}

static int32_t j5_bpu_disable(struct bpu_core *core)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("Disable invalid bpu core!\n");/*PRQA S ALL*/
		return -ENODEV;
	}
	if (core->fc_base == NULL) {
		dev_err(core->dev, "bpu core already disabled\n");
		return 0;
	}

	/*TODO: block write and wait run fifo process done */

	ret = bpu_core_clk_off(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core clk disable failed\n");
	}

	/* j5_bpu_iso_set */

	ret = bpu_core_power_off(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core power disable failed\n");
	}

	dma_free_coherent(core->dev, FC_SIZE * FC_DEPTH,
			core->fc_base, core->fc_base_addr);/*PRQA S ALL*/
	core->fc_base = NULL;

	return ret;
}

static int32_t j5_bpu_set_clk(const struct bpu_core *core, uint64_t rate)
{
	uint64_t last_rate;
	int32_t ret = 0;

	if (core == NULL) {
		pr_err("Set invalid bpu core clk!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (core->mclk == NULL) {
		return ret;
	}

	last_rate = clk_get_rate(core->mclk);

	if (last_rate == rate) {
		return 0;
	}

	ret = clk_set_rate(core->mclk, rate);
	if (ret != 0) {
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

static int32_t j5_bpu_set_volt(const struct bpu_core *core, int32_t volt)
{
	int32_t ret = 0;

	if (core == NULL) {
		pr_err("Set invalid bpu core voltage!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (volt <= 0) {
		pr_err("Set invalid value bpu core voltage!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	if (core->regulator != NULL) {
		ret = regulator_set_voltage(core->regulator,
					volt, INT_MAX);
		if (ret != 0) {
			dev_err(core->dev, "Cannot set voltage %u uV\n", volt);
			return ret;
		}
	}

	return ret;
}

static void j5_bpu_set_update_tail(const struct bpu_core *core, uint32_t tail_index)
{
	uint32_t tmp_reg;

	tmp_reg = j5_bpu_reg_read(core, CNN_FC_TAIL);
	tmp_reg &= ~(J5_CNN_PE0_FC_TAIL_MASK);

	tmp_reg |= J5_CNN_PE0_FC_TAIL(tail_index);
	j5_bpu_reg_write(core, CNN_FC_TAIL, tmp_reg);
}

static int32_t j5_bpu_fc_equeue(const struct bpu_core *core,
		const struct bpu_hw_fc fc_data[], uint32_t *fc_num)
{
	uint32_t free_fc_fifo;
	uint32_t head_index, tail_index,
	    fc_head_flag, fc_tail_flag;
	uint32_t fc_depth, insert_fc_cnt, residue_fc_cnt;
	uint32_t ret;

	fc_depth = j5_bpu_reg_read(core, CNN_FC_LEN);

	head_index = j5_bpu_reg_read(core, CNN_FC_HEAD);
	fc_head_flag = head_index & J5_CNN_FC_IDX_FLAG;

	tail_index = j5_bpu_reg_read(core, CNN_FC_TAIL);
	fc_tail_flag = tail_index & J5_CNN_FC_IDX_FLAG;

	head_index &= J5_CNN_MAX_FC_LEN_MASK;
	tail_index &= J5_CNN_MAX_FC_LEN_MASK;
	if (fc_head_flag != fc_tail_flag) {
		free_fc_fifo = head_index - tail_index;
	} else {
		free_fc_fifo = fc_depth - tail_index + head_index + 1u;
	}

	if (core->fc_buf_limit > 0) {
		free_fc_fifo =
			(uint32_t)core->fc_buf_limit - (fc_depth + 1u - free_fc_fifo);
		/* running fc num need not larger then limit */
		if (free_fc_fifo <= 0u) {
			return -EBUSY;
		}
	}

	if (*fc_num > free_fc_fifo) {
		*fc_num = free_fc_fifo;
	}

	if ((tail_index + *fc_num) > fc_depth) {
		insert_fc_cnt = fc_depth - tail_index + 1u;
		(void)memcpy(&core->fc_base[tail_index], fc_data,/*PRQA S ALL*/
			((uint64_t)insert_fc_cnt * J5_CNN_FC_SIZE));

		if (fc_tail_flag != 0u) {
			fc_tail_flag = 0;
		} else {
			fc_tail_flag = J5_CNN_FC_IDX_FLAG;
		}

		residue_fc_cnt = *fc_num - insert_fc_cnt;
		if (residue_fc_cnt > 0u) {
			(void)memcpy(core->fc_base, &fc_data[insert_fc_cnt],/*PRQA S ALL*/
				(uint64_t)residue_fc_cnt * J5_CNN_FC_SIZE);
		}

		ret = fc_tail_flag | residue_fc_cnt;
	} else {
		(void)memcpy(&core->fc_base[tail_index], fc_data,/*PRQA S ALL*/
			(uint64_t)*fc_num * J5_CNN_FC_SIZE);

		ret = fc_tail_flag | (tail_index + *fc_num);
	}

	return (int32_t)ret;
}

static int32_t j5_bpu_write_fc(const struct bpu_core *core,
		struct bpu_fc *fc, uint32_t offpos)
{
	uint16_t *tmp_fc_id;
	uint32_t update_tail;
	uint32_t fc_num;
	int32_t ret;

	if (core == NULL) {
		pr_err("Write invalid bpu core!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (core->fc_base == NULL) {
		dev_err(core->dev, "bpu core not enable\n");
		return -ENODEV;
	}

	if ((fc == NULL) || (fc->fc_data == NULL)) {
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

	/* set the first hw fc id to special id for  trigger follow sched */
	if ((fc_num > 1u) || ((fc_num == 1u) && (fc->hw_id == 0u))) {
		tmp_fc_id = (uint16_t *)(&(fc->fc_data[offpos].data[FC_ID_OFFSET]));/*PRQA S ALL*/
		*tmp_fc_id = (uint16_t)HW_ID_MAX;
	}

	/* set the last hw fc id to upper set */
	if(fc->hw_id != 0u) {
		tmp_fc_id = (uint16_t *)(&(fc->fc_data[offpos + fc_num - 1u].data[FC_ID_OFFSET]));/*PRQA S ALL*/
		*tmp_fc_id = (uint16_t)fc->hw_id;
	}

	ret = j5_bpu_fc_equeue(core, &fc->fc_data[offpos], &fc_num);
	if (ret < 0) {
		return ret;
	} else {
		update_tail = (uint32_t)ret;
	}

	fc->index = (update_tail & (~J5_CNN_FC_IDX_FLAG)) - 1u;

	j5_bpu_set_update_tail(core, (uint32_t)update_tail);

	return (int32_t)fc_num;
}

static int32_t j5_bpu_read_fc(const struct bpu_core *core,
		uint32_t *tmp_id, uint32_t *err)
{
	uint32_t irq_status;
	uint32_t ret_fc_num;

	if (core == NULL) {
		pr_err("Read invalid bpu core!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	/* the status just need read on J5 */
	irq_status = j5_bpu_reg_read(core, CNNINT_STATUS);
	pr_debug("BPU Core irq status = 0x%x\n", irq_status);/*PRQA S ALL*/

	j5_bpu_reg_write(core, CNNINT_MASK, 0x1);
	*tmp_id = j5_bpu_reg_read(core, CNNINT_NUM);
	j5_bpu_reg_write(core, CNNINT_MASK, 0x0);

	ret_fc_num = j5_bpu_reg_read(core, CNN_ALL_INT_CNT);
	if (ret_fc_num > 1u) {
		pr_debug("BPU Core%d postpone get %d fcs\n",/*PRQA S ALL*/
				core->index, ret_fc_num);
	}

	*err = j5_bpu_reg_read(core, CNNINT_ERR_NUM);

	return (int32_t)ret_fc_num;
}

/* J5 use core reserved[0] to store burst_len set */
static ssize_t bpu_core_burst_len_show(struct device *dev, struct device_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/

	if (core->reserved[0] == 0u) {
		core->reserved[0] = DEFAULT_BURST_LEN;
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n",
			(core->reserved[0] * HEXBITS));
}

static ssize_t bpu_core_burst_len_store(struct device *dev, struct device_attribute *attr,/*PRQA S ALL*/
		const char *buf, size_t len)
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	int32_t ret;
	uint32_t tmp_val;

	ret = sscanf(buf, "%du", &tmp_val);
	if (ret < 0) {
		dev_err(core->dev, "BPU core%d sscanf burst error\n",
				core->index);
		return 0;
	}

	if ((tmp_val % HEXBITS) > 0u) {
		dev_err(core->dev, "burst len must align 16");
	}

	core->reserved[0] = (uint64_t)tmp_val / HEXBITS;

	if (core->reserved[0] <= 0u) {
		core->reserved[0] = 1;
	}
	if (core->reserved[0] > DEFAULT_BURST_LEN) {
		core->reserved[0] = DEFAULT_BURST_LEN;
	}

	dev_info(core->dev, "BPU core%d set burst len:%lld\n",
			core->index, core->reserved[0]);

	return (ssize_t)len;
}
// PRQA S ALL ++
static DEVICE_ATTR(burst_len, S_IRUGO | S_IWUSR,
		bpu_core_burst_len_show, bpu_core_burst_len_store);

static struct attribute *bpu_core_hw_attrs[] = {
	&dev_attr_burst_len.attr,
	NULL,
};

static struct attribute_group bpu_core_hw_attr_group = {
	.attrs = bpu_core_hw_attrs,
};
// PRQA S ALL --
static int32_t j5_bpu_debug(const struct bpu_core *core, int32_t state)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("NO bpu core for debug!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (state == 1) {
		ret = device_add_group(core->dev, &bpu_core_hw_attr_group);
		if (ret < 0) {
			dev_err(core->dev, "Create bpu core%d hw debug group failed\n",
				core->index);
		}
	}

	if (state == 0) {
		device_remove_group(core->dev, &bpu_core_hw_attr_group);
	}

	return 0;
}

struct bpu_core_hw_ops j5_hw_ops = {
	.enable		= j5_bpu_enable,
	.disable	= j5_bpu_disable,
	.reset		= j5_bpu_reset,
	.set_clk	= j5_bpu_set_clk,
	.set_volt	= j5_bpu_set_volt,
	.write_fc	= j5_bpu_write_fc,
	.read_fc	= j5_bpu_read_fc,
	.status		= NULL,
	.debug		= j5_bpu_debug,
};

// PRQA S ALL ++
MODULE_DESCRIPTION("Driver for Horizon J5 SOC BPU");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
// PRQA S ALL --
