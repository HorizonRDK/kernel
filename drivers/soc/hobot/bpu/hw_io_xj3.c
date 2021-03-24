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
#include <soc/hobot/diag.h>
#include "bpu.h"
#include "bpu_core.h"
#include "bpu_ctrl.h"
#include "hw_io.h"

#define CORE_PE_TYPE_OFFSET 4

#define DEFAULT_BURST_LEN (0x80u)
static int32_t bpu_err_flag;

static int soc_is_x3e(void)
{
	int32_t chipid;

	void __iomem *chipid_reg = ioremap_nocache(0xa6008070, 4);
	chipid = readl(chipid_reg);
	iounmap(chipid_reg);

	if (((chipid>>12)&0x1) == 0x1) {
		return 1;
	}

	return 0;
}

static void cnnbus_wm_set(struct bpu_core *core, uint32_t reg_off,
	uint32_t wd_maxlen, uint32_t wd_endian, uint32_t wd_priority)
{
	uint32_t reg_val;

	reg_val = bpu_core_reg_read(core, reg_off);
	reg_val &= ~(CNN_WD_MAXLEN_M_MASK |
		CNN_WD_ENDIAN_M_MASK |
		CNN_WD_PRIORITY_M_MASK);

	reg_val |= CNN_WD_MAXLEN_M(wd_maxlen) |
		CNN_WD_ENDIAN_M(wd_endian) |
		CNN_WD_PRIORITY_M(wd_priority);

	bpu_core_reg_write(core, reg_off, reg_val);
}

static void cnnbus_rm_set(struct bpu_core *core, uint32_t reg_off,
	uint32_t rd_maxlen, uint32_t rd_endian, uint32_t rd_priority)
{
	uint32_t reg_val;

	reg_val = bpu_core_reg_read(core, reg_off);
	reg_val &= ~(CNN_RD_MAXLEN_M_MASK |
		CNN_RD_ENDIAN_M_MASK |
		CNN_RD_PRIORITY_M_MASK);

	reg_val |= CNN_RD_MAXLEN_M(rd_maxlen) |
		CNN_RD_ENDIAN_M(rd_endian) |
		CNN_RD_PRIORITY_M(rd_priority);

	bpu_core_reg_write(core, reg_off, reg_val);
}

static int32_t bpu_core_hw_init(struct bpu_core *core)
{
	uint32_t reg_val;
	uint32_t tmp_fc_depth;

	if (core->reserved[0] == 0u) {
		core->reserved[0] = DEFAULT_BURST_LEN;
	}
// PRQA S ALL ++
	/* Config axi write master */
	cnnbus_wm_set(core, CNNBUS_CTRL_WM_0, core->reserved[0], 0xf, 0x1);
	cnnbus_wm_set(core, CNNBUS_CTRL_WM_1, 0x80, 0xf, 0x2);
	cnnbus_wm_set(core, CNNBUS_CTRL_WM_2, 0x8, 0x0, 0x3);
	cnnbus_wm_set(core, CNNBUS_CTRL_WM_3, 0x80, 0x0, 0x4);

	/* Config axi read master */
	cnnbus_rm_set(core, CNNBUS_CTRL_RM_0, 0x80, 0xf, 0x4);
	cnnbus_rm_set(core, CNNBUS_CTRL_RM_1, 0x80, 0xf, 0x4);
	cnnbus_rm_set(core, CNNBUS_CTRL_RM_2, 0x8, 0xf, 0x4);
	cnnbus_rm_set(core, CNNBUS_CTRL_RM_3, 0x80, 0x8, 0x5);
	cnnbus_rm_set(core, CNNBUS_CTRL_RM_4, 0x80, 0xf, 0x4);
	cnnbus_rm_set(core, CNNBUS_CTRL_RM_5, 0x80, 0x0, 0x6);
// PRQA S ALL --
	/* Set axibus id */
	bpu_core_reg_write(core, CNNBUS_AXIID, 0x0);

	bpu_core_reg_write(core, CNNINT_MASK, 0x0);

	tmp_fc_depth = FC_DEPTH;
	/* tell bpu fc depth */
	reg_val = bpu_core_reg_read(core, CNN_FC_LEN);
	reg_val &=  ~(CNN_PE0_FC_LENGTH_MASK);
	/* hw depth start from 0 */
	reg_val |= CNN_PE0_FC_LENGTH(tmp_fc_depth - 1);
	bpu_core_reg_write(core, CNN_FC_LEN, reg_val);

	/* tell bpu fc base */
	reg_val = CNN_PE0_FC_BASE((uint32_t)core->fc_base_addr[0]);

	bpu_core_reg_write(core, CNN_FC_BASE, reg_val);

	return 0;
}

static int32_t bpu_core_iso_clear(struct bpu_core *core)
{
	uint32_t reg_val;
	int32_t ret = 0;

	if (core == NULL) {
		return -ENODEV;
	}

	/* x2/x3 reserved mem is pmu function */
	if (core->reserved_base != NULL) {
		reg_val = readl(core->reserved_base);
		reg_val &= ~BPU_ISO_BIT(core->index);
		writel(reg_val, core->reserved_base);
	}

	udelay(5);

	if (core->rst != NULL) {
		ret = bpu_reset_ctrl(core->rst, 0);
		if (ret < 0) {
			dev_err(core->dev, "bpu core reset deassert failed\n");
		}
	}

	return ret;
}

static int32_t bpu_core_iso_set(struct bpu_core *core)
{
	uint32_t reg_val;
	int32_t ret = 0;

	if (core == NULL) {
		return -ENODEV;
	}

	/* x2/x3 reserved mem is pmu function */
	if (core->reserved_base != NULL) {
		reg_val = readl(core->reserved_base);
		reg_val |= BPU_ISO_BIT(core->index);
		writel(reg_val, core->reserved_base);
	}

	udelay(5);

	if (core->rst != NULL) {
		ret = bpu_reset_ctrl(core->rst, 1);
		if (ret < 0) {
			dev_err(core->dev, "bpu core reset assert failed\n");
		}
	}

	return ret;
}

static int32_t bpu_core_hw_rst(struct bpu_core *core)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("Reset invalid bpu core!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	ret = bpu_core_hw_reset(core, 1);
	if (ret != 0) {
		dev_err(core->dev, "bpu core reset failed\n");
		return ret;
	}

	return bpu_core_hw_init(core);
}

static void bpu_diag_test(void *p, size_t len)
{
	bpu_err_flag = 1;
}

static int32_t bpu_core_hw_enable(struct bpu_core *core)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("Enable invalid bpu core!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (core->fc_base[0] != NULL) {
		dev_err(core->dev, "bpu core already enable\n");
		return 0;
	}

	ret = bpu_core_clk_off(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core clk disable failed\n");
	}

	if (core->rst != NULL) {
		ret = bpu_reset_ctrl(core->rst, 1);
		if (ret < 0) {
			dev_err(core->dev, "bpu core reset assert failed\n");
		}
	}

	ret = bpu_core_power_on(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core power enable failed\n");
	}

	bpu_core_iso_clear(core);

	ret = bpu_core_clk_on(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core clk enable failed\n");
	}

	/* The following to init fc info */

	core->fc_base[0] = dma_alloc_coherent(core->dev,/*PRQA S ALL*/
			FC_SIZE * FC_DEPTH, &core->fc_base_addr[0], GFP_KERNEL);
	if (core->fc_base[0] == NULL) {
		dev_err(core->dev, "bpu core alloc fc mem failed\n");
		return -ENOMEM;
	}

	ret = bpu_core_hw_rst(core);
	if (ret != 0) {
		dev_err(core->dev, "bpu core hw reset failed\n");
	}

	if ((EventIdBpu0Err + core->index) <= EventIdBpu1Err) {
		if (diag_register(ModuleDiag_bpu, EventIdBpu0Err + core->index, 5, 300,
					7000, bpu_diag_test) < 0)
			pr_debug("bpu%d diag register fail\n", core->index);
	} else {
		dev_err(core->dev, "bpu event id overun: max = 2,but now is:%d\n",
			EventIdBpu0Err + core->index);
	}

	return 0;
}

static int32_t bpu_core_hw_disable(struct bpu_core *core)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("Disable invalid bpu core!\n");/*PRQA S ALL*/
		return -ENODEV;
	}
	if (core->fc_base[0] == NULL) {
		dev_err(core->dev, "bpu core already disabled\n");
		return 0;
	}

	/*TODO: block write and wait run fifo process done */

	ret = bpu_core_clk_off(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core clk disable failed\n");
	}

	bpu_core_iso_set(core);

	ret = bpu_core_power_off(core);
	if (ret < 0) {
		dev_err(core->dev, "bpu core power disable failed\n");
	}

	dma_free_coherent(core->dev, FC_SIZE * FC_DEPTH,
			core->fc_base[0], core->fc_base_addr[0]);/*PRQA S ALL*/
	core->fc_base[0] = NULL;

	return ret;
}

static void bpu_core_set_update_tail(const struct bpu_core *core, uint32_t tail_index)
{
	uint32_t tmp_reg;

	tmp_reg = bpu_core_reg_read(core, CNN_FC_TAIL);
	tmp_reg &= ~(CNN_PE0_FC_TAIL_MASK);

	tmp_reg |= CNN_PE0_FC_TAIL(tail_index);
	bpu_core_reg_write(core, CNN_FC_TAIL, tmp_reg);
}

static int32_t bpu_core_fc_equeue(const struct bpu_core *core,
		const struct bpu_hw_fc fc_data[], uint32_t *fc_num, bool bind)
{
	uint32_t free_fc_fifo;
	uint32_t head_index, tail_index,
	    fc_head_flag, fc_tail_flag;
	uint32_t fc_depth, insert_fc_cnt, residue_fc_cnt;
	uint32_t ret = 0;

	fc_depth = bpu_core_reg_read(core, CNN_FC_LEN);

	head_index = bpu_core_reg_read(core, CNN_FC_HEAD);
	fc_head_flag = head_index & CNN_FC_IDX_FLAG;

	tail_index = bpu_core_reg_read(core, CNN_FC_TAIL);
	fc_tail_flag = tail_index & CNN_FC_IDX_FLAG;

	head_index &= CNN_MAX_FC_LEN_MASK;
	tail_index &= CNN_MAX_FC_LEN_MASK;
	if (fc_head_flag != fc_tail_flag) {
		free_fc_fifo = head_index - tail_index;
	} else {
		free_fc_fifo = fc_depth - tail_index + head_index + 1u;
	}

	if ((core->fc_buf_limit > 0) && (!bind)) {
		free_fc_fifo =
			(uint32_t)core->fc_buf_limit - (fc_depth + 1u - free_fc_fifo);
		/* running fc num need not larger then limit */
		if (free_fc_fifo <= 0u) {
			return -EBUSY;
		}
	}

	if (*fc_num > free_fc_fifo) {
		if (bind) {
			pr_info("Hint: Make sure BPU model use right priority and compile mode!\n");
		}
		*fc_num = free_fc_fifo;
	}

	if ((tail_index + *fc_num) > fc_depth) {
		insert_fc_cnt = fc_depth - tail_index + 1u;
		(void)memcpy(&(core->fc_base[0])[tail_index], &fc_data[0],/*PRQA S ALL*/
			((uint64_t)insert_fc_cnt * CNN_FC_SIZE));

		if (fc_tail_flag != 0u) {
			fc_tail_flag = 0;
		} else {
			fc_tail_flag = CNN_FC_IDX_FLAG;
		}

		residue_fc_cnt = *fc_num - insert_fc_cnt;
		if (residue_fc_cnt > 0u) {
			(void)memcpy(&(core->fc_base[0])[0], &fc_data[insert_fc_cnt],/*PRQA S ALL*/
				(uint64_t)residue_fc_cnt * CNN_FC_SIZE);
		}

		ret = fc_tail_flag | residue_fc_cnt;
	} else {
		(void)memcpy(&(core->fc_base[0])[tail_index], &fc_data[0],/*PRQA S ALL*/
			(uint64_t)*fc_num * CNN_FC_SIZE);

		ret = fc_tail_flag | (tail_index + *fc_num);
	}

	return (int32_t)ret;
}

static int32_t bpu_core_hw_write_fc(const struct bpu_core *core,
		struct bpu_fc *fc, uint32_t offpos)
{
	uint16_t *tmp_fc_id;
	uint32_t update_tail;
	uint32_t fc_num;
	int32_t ret;

	if (core == NULL) {
		pr_err("Write invalid bpu core!\n");
		return -ENODEV;
	}

	if (core->fc_base[0] == NULL) {
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

	ret = bpu_core_fc_equeue(core, &fc->fc_data[offpos], &fc_num, fc->bind);
	if (ret < 0) {
		return ret;
	} else {
		update_tail = (uint32_t)ret;
	}

	fc->index = update_tail & (~CNN_FC_IDX_FLAG);

	bpu_core_set_update_tail(core, (uint32_t)update_tail);

	return (int32_t)fc_num;
}

static void report_bpu_diagnose_msg(u32 err, int core_index)
{
	u32 ret;
	u8 bpu_event;
	u8 bpu_diag_envdata[5]; // core_id(1ybte) + error_code(4bytes)

	ret = err & 0xf000;
	bpu_event = EventIdBpu0Err + core_index;

	if (ret == 0x1000 || ret == 0x2000 || ret == 0x3000 ||
		ret == 0x4000 || ret == 0x5000 || ret == 0x6000 ||
		ret == 0x7000 || ret == 0x8000 || ret == 0x9000 ) {
		bpu_diag_envdata[0] = (u8)core_index;
		memcpy(bpu_diag_envdata + 1, (uint8_t *)&ret, sizeof(u32));
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh, ModuleDiag_bpu,
				bpu_event, DiagEventStaFail,
				DiagGenEnvdataWhenErr, bpu_diag_envdata, 5);
	} else {
		diag_send_event_stat(DiagMsgPrioMid, ModuleDiag_bpu,
					bpu_event, DiagEventStaSuccess);
	}
}

static int32_t bpu_core_hw_read_fc(const struct bpu_core *core,
		uint32_t *tmp_id, uint32_t *err)
{
	uint32_t irq_status;

	if (core == NULL) {
		pr_err("Read invalid bpu core!\n");
		return -ENODEV;
	}

	/* the status just need read on X2 */
	irq_status = bpu_core_reg_read(core, CNNINT_STATUS);
	pr_debug("BPU Core irq status = 0x%x\n", irq_status);/*PRQA S ALL*/

	bpu_core_reg_write(core, CNNINT_MASK, 0x1);
	*tmp_id = bpu_core_reg_read(core, CNNINT_NUM);
	bpu_core_reg_write(core, CNNINT_MASK, 0x0);

	if (*tmp_id & 0xf000) {
		*err = *tmp_id;
	} else {
		*err = 0;
	}

	if (bpu_err_flag == 1) {
		report_bpu_diagnose_msg(0x1000, core->index);
		bpu_err_flag = 0;
		return 1;
	}
	report_bpu_diagnose_msg(*err, core->index);

	return 1;
}

static int32_t bpu_core_hw_set_clk(const struct bpu_core *core, uint64_t rate)
{
	uint64_t last_rate;
	int32_t ret = 0;

	if (soc_is_x3e() == 1) {
		if (rate > 600000000) {
			rate = 600000000;
		}
	}

	if (core == NULL) {
		pr_err("Set invalid bpu core clk!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (core->mclk == NULL) {
		return ret;
	}

	last_rate = bpu_clk_get_rate(core->mclk);

	if (last_rate == rate) {
		return 0;
	}

	ret = bpu_clk_set_rate(core->mclk, rate);
	if (ret != 0) {
		dev_err(core->dev, "Cannot set frequency %llu (%d)\n",
				rate, ret);
		return ret;
	}

	/* check if rate set success, when not, user need recover volt */
	if (bpu_clk_get_rate(core->mclk) != rate) {
		dev_err(core->dev,
				"Get wrong frequency, Request %llu, Current %llu\n",
				rate, bpu_clk_get_rate(core->mclk));
		return -EINVAL;
	}

	return ret;
}

/* bpu preread 4 fc in fifo, +2 for margin */
#define LAG_FC_NUM	6
static int32_t bpu_core_hw_status(struct bpu_core *core, uint32_t cmd)
{
	static uint32_t head_index[BPU_MAX_CORE_NUM], tail_index[BPU_MAX_CORE_NUM];
	uint32_t tmp_head_index, tmp_tail_index;
	static uint32_t inst_num[BPU_MAX_CORE_NUM];
	struct timeval tmp_point;
	uint64_t tmp_pass_time;
	struct bpu_fc tmp_bpu_fc;
	uint32_t tmp_inst_num;
	int ret = 0;

	if (core == NULL) {
		pr_err("Get Status from invalid bpu core!\n");
		return -ENODEV;
	}

	tmp_head_index = bpu_core_reg_read(core, CNN_FC_HEAD);
	tmp_tail_index = bpu_core_reg_read(core, CNN_FC_TAIL);
	tmp_inst_num = bpu_core_reg_read(core, CNNINT_INST_NUM);

	switch (cmd) {
	case (uint32_t)BUSY_STATE:/*PRQA S ALL*/
		if (tmp_head_index == tmp_tail_index) {
			ret = 0;
		} else {
			ret = 1;
		}
		break;
	case (uint32_t)WORK_STATE:/*PRQA S ALL*/
		if ((tmp_head_index == head_index[core->index])
				&& (tmp_inst_num == inst_num[core->index])
				&& (tmp_head_index != tmp_tail_index)) {

			ret = kfifo_peek(&core->run_fc_fifo[0], &tmp_bpu_fc);/*PRQA S ALL*/
			if (ret < 1) {
				ret = 0;
			} else {
				if(tmp_bpu_fc.info.process_time > 0) {
					do_gettimeofday(&tmp_point);
					tmp_pass_time = (((uint64_t)tmp_point.tv_sec
							* (uint64_t)SECTOUS)
							+ (uint64_t)tmp_point.tv_usec)
							- (((uint64_t)tmp_bpu_fc.start_point.tv_sec
							* (uint64_t)SECTOUS)
							+ (uint64_t)tmp_bpu_fc.start_point.tv_usec);
					if ((tmp_pass_time < (LAG_FC_NUM * tmp_bpu_fc.info.process_time))) {
						ret = 1;
					} else {
						ret = 0;
					}
				} else {
					ret = 0;
				}
			}
		} else {
			ret = (int32_t)tmp_head_index + 1;
		}
		break;
	case (uint32_t)UPDATE_STATE:/*PRQA S ALL*/
		/* do nothing just update regs*/
		break;
	case (uint32_t)TYPE_STATE:
		if (soc_is_x3e()) {
			ret = (1u << CORE_PE_TYPE_OFFSET) | CORE_TYPE_4PE;
		} else {
			ret = CORE_TYPE_4PE;
		}
		break;
	default:
		pr_err("Invalid bpu state cmd[%d]\n", cmd);/*PRQA S ALL*/
		ret = -EINVAL;
		break;
	}

	head_index[core->index] = tmp_head_index;
	tail_index[core->index] = tmp_tail_index;
	inst_num[core->index] = tmp_inst_num;
	return ret;
}

/* X2 use core reserved[0] to store burst_len set */
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

static int32_t bpu_core_hw_debug(const struct bpu_core *core, int32_t state)
{
	int32_t ret;

	if (core == NULL) {
		pr_err("NO bpu core for debug!\n");
		return -ENODEV;
	}

	if (state == 1) {
		ret = bpu_device_add_group(core->dev, &bpu_core_hw_attr_group);
		if (ret < 0) {
			dev_err(core->dev, "Create bpu core%d hw debug group failed\n",
				core->index);
		}
	}

	if (state == 0) {
		bpu_device_remove_group(core->dev, &bpu_core_hw_attr_group);
	}

	return 0;
}

struct bpu_core_hw_ops hw_ops = {
	.enable		= bpu_core_hw_enable,
	.disable	= bpu_core_hw_disable,
	.reset		= bpu_core_hw_rst,
	.set_clk	= bpu_core_hw_set_clk,
	.set_volt	= NULL,
	.write_fc	= bpu_core_hw_write_fc,
	.read_fc	= bpu_core_hw_read_fc,
	.status		= bpu_core_hw_status,
	.debug		= bpu_core_hw_debug,
};

// PRQA S ALL ++
MODULE_DESCRIPTION("Driver for Horizon XJ3/XJ2 SOC BPU");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
// PRQA S ALL --
