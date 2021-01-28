/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2020 Horizon Robotics, Inc.
 *			 All rights reserved.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/genalloc.h>
#include <linux/pfn.h>
#include <linux/idr.h>
#include <linux/completion.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/time.h>
#include <linux/io.h>
#include "remoteproc_internal.h"

//#define TIMESTAMP_DEBUG

#define MCORE_DEV_IDX  (0)
#define DCORE0_DEV_IDX  (1)
#define DCORE1_DEV_IDX  (2)
#define RCORE_DEV_IDX  (3)

#define J3_CPUSYS_CLKEN_SET (0x104)
#define J3_CPUSYS_CLKEN_CLR (0x108)
#define cr5_aclk_cken_set_bit (14)

#define J3_CPUSYS_SW_RSTEN (0x400)
#define cr5_rst_en_bit (13)

#define J3_CR5_BOOT_ADDR0 (0x600)

/* ipi interrupt register */
#define J3_IPI_BASE_OFFSET         (0xB000)
#define J3_IPI_APU_TRIG_REG        (J3_IPI_BASE_OFFSET + 0x0)
#define J3_IPI_APU_OBS_REG         (J3_IPI_BASE_OFFSET + 0x04)
#define J3_IPI_APU_ISR_REG         (J3_IPI_BASE_OFFSET + 0x08)
#define J3_IPI_APU_IMR_REG         (J3_IPI_BASE_OFFSET + 0x0C)
#define J3_IPI_APU_IER_REG         (J3_IPI_BASE_OFFSET + 0x10)
#define J3_IPI_APU_IDR_REG         (J3_IPI_BASE_OFFSET + 0x14)

#define J3_cr5_mbox_irq_bit 	   (0x1)


/* ipi message buffer register */
#define J3_IPI_APU_REQ_BUFF(n)     (J3_IPI_BASE_OFFSET + 0x400 + n * 0x20)
//#define J3_MBOX_IRQ_APU    1
//#define J3_MBOX_IRQ_RPU    2
#define J3_IPI_CMDBUF_FULLUSE              0xFF



#define AONSYS_CCLK_CFG0_OFFSET    (0x0)
#define cgm_mstar_en_bit           (3)

#define ANOSYS_SFT_ARES1_OFFSET    (0x24)
#define mstar_sft_ares_bit         (0)

#define STAR_CFG_OFFSET            (0x40)
#define starcfg_inittcm_en_bit0    (0)
#define starcfg_inittcm_en_bit1    (1)
#define starcfg_cpuwait_bit        (4)

#define ACORE_GEN_VIC_OFFSET       (0x200)
#define a_to_m0_int_req_bit        (8)
#define a_to_vdsp0_int_req_bit     (16)
#define a_to_vdsp1_int_req_bit     (24)

#define ACORE_VIC_OFFSET           (0x204)
#define m0_to_a_int_status_bit     (8)
#define vdsp0_to_a_int_status_bit  (16)
#define vdsp1_to_a_int_status_bit  (24)

#define AON_MCORE_VIC_OFFSET       (0x214)
#define a_to_m0_int_en_bit         (9)

#define VDSP0_VIC_OFFSET           (0x224)
#define a_to_vdsp0_int_en_bit      (9)

#define ISP1_MCORE_VIC             (0x234)
#define a_to_vdsp1_int_en_bit      (9)

#define CVSYSAPB_REG_BASE (0x4A010000)
#define VDSP0_CTL_STATUS_OFFSET (0x4C)
#define VDSP0_ALTRESETVEC_OFFSET (0x48)
#define SOFT_RST_EN_OFFSET (0x10)

#define MCORE_IPC_INT_REG_RANGE   (0x300)
#define DCORE_IPC_INT_REG_RANGE   (0x300)
#define RCORE_IPC_INT_REG_RANGE   (0xB600)

struct debug_statistics {
	int irq_handler_count;
};

struct rproc_ipc_ops {
	int (*interrupt_judge)(void *ipc_int_va);
	void (*trigger_interrupt)(void *ipc_int_va);
	void (*clear_interrupt)(void *ipc_int_va);
	void (*start_remoteproc)(void *ipc_int_va);
	void (*release_remoteproc)(void *ipc_int_va);
	void (*pre_load_remoteproc)(void *ipc_int_va);
};

/**
 * struct hobot_rproc_pdata - hobot remote processor instance state
 * @rproc: rproc handle
 * @fw_ops: local firmware operations
 * @default_fw_ops: default rproc firmware operations
 * @mem_pools: list of gen_pool for firmware mmio_sram memory and their
 *             power domain IDs
 * @mems: list of rproc_mem_entries for firmware
 * @vring0: IRQ number used for vring0, just trigger rvq interrupt
 * @ipi_dest_mask: IPI destination mask for the IPI channel
 * @device_index: identify mcore & dcore remoteproc instance
 * @ipc_int_pa: ipc interrupt register base physical address
 * @ipc_int_va: ipc interrupt register base virtual address
 * @work_queue: interrupt bottom half singlethread workqueue
 * @work: interrupt bottom half work
 * @notify_complete: synchronization method between interrupt top & bottom half
 * @should_stop: work stop identification
 * @statistics: debug statistics
 * @top_entry: proc filesystem top entry
 * @info_entry: proc filesystem info entry
 * @statistics_entry: proc filesystem statistics entry
 * @fix_map_mode: whether use fix map mode
 * @ipc_ops: ipc memory operations
 */
struct hobot_rproc_pdata {
	struct rproc *rproc;
	struct rproc_fw_ops fw_ops;
	const struct rproc_fw_ops *default_fw_ops;
	int vring0;
	int device_index;
	u32 ipc_int_pa;
	void __iomem *ipc_int_va;
	struct workqueue_struct *work_queue;
	struct work_struct work;
	struct completion notify_complete;
	int should_stop;
	struct debug_statistics statistics;
	struct proc_dir_entry *top_entry;
	struct proc_dir_entry *info_entry;
	struct proc_dir_entry *statistics_entry;
	int fix_map_mode;
	struct rproc_ipc_ops ipc_ops;
};

static bool autoboot __read_mostly;
static void __iomem *cvsysapb_reg_va;

static int hobot_rcore_rproc_start(struct rproc *rproc)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;

	pdata->ipc_ops.start_remoteproc(pdata->ipc_int_va);

	return 0;
}

static int hobot_rcore_rproc_stop(struct rproc *rproc)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;

	pdata->ipc_ops.release_remoteproc(pdata->ipc_int_va);

	return 0;
}

static void hobot_rcore_rproc_kick(struct rproc *rproc, int vqid)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;

	wmb();
	pdata->ipc_ops.trigger_interrupt(pdata->ipc_int_va);
}

static void *hobot_rcore_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct rproc_mem_entry *mem = NULL;
	void *va = NULL;

	list_for_each_entry(mem, &rproc->mappings, node) {
		int offset = da - mem->da;

		if (offset < 0)
			continue;

		if (offset + len > mem->len)
			continue;

		va = mem->va + offset;
		break;
	}

	pr_info("convert: 0x%llx --> 0x%p\n", da, va);
	return va;
}

static int hobot_rcore_rproc_pre_load(struct rproc *rproc)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;
	struct rproc_mem_entry *mem = NULL;

	if (!rproc->register_virtio) {
		// mcore boot sequence
		pdata->ipc_ops.pre_load_remoteproc(pdata->ipc_int_va);

		mem = list_first_entry(&rproc->mappings,
					struct rproc_mem_entry, node);
		//3. set first dev mem address to rcore register
		writel(mem->da, pdata->ipc_int_va + J3_CR5_BOOT_ADDR0);
		msleep(10);

		// clear sram
		list_for_each_entry(mem, &rproc->mappings, node)
			memset_io(mem->va, 0, mem->len);
	}

	return 0;
}

static struct rproc_ops hobot_rcore_rproc_ops = {
	.start = hobot_rcore_rproc_start,
	.stop = hobot_rcore_rproc_stop,
	.kick = hobot_rcore_rproc_kick,
	.da_to_va = hobot_rcore_rproc_da_to_va,
	.pre_load = hobot_rcore_rproc_pre_load,
};

static int hobot_mcore_rproc_start(struct rproc *rproc)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;

	pdata->ipc_ops.start_remoteproc(pdata->ipc_int_va);

	return 0;
}

static int hobot_mcore_rproc_stop(struct rproc *rproc)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;

	pdata->ipc_ops.release_remoteproc(pdata->ipc_int_va);

	return 0;
}

static void hobot_mcore_rproc_kick(struct rproc *rproc, int vqid)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;

	wmb();
	pdata->ipc_ops.trigger_interrupt(pdata->ipc_int_va);
}

static void *hobot_mcore_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct rproc_mem_entry *mem = NULL;
	void *va = NULL;

	list_for_each_entry(mem, &rproc->mappings, node) {
		int offset = da - mem->da;

		if (offset < 0)
			continue;

		if (offset + len > mem->len)
			continue;

		va = mem->va + offset;
		break;
	}

	pr_info("convert: 0x%llx --> 0x%p\n", da, va);
	return va;
}

static int hobot_mcore_rproc_pre_load(struct rproc *rproc)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;
	struct rproc_mem_entry *mem = NULL;

	if (!rproc->register_virtio) {
		// mcore boot sequence
		pdata->ipc_ops.pre_load_remoteproc(pdata->ipc_int_va);

		// clear sram
		list_for_each_entry(mem, &rproc->mappings, node)
			memset_io(mem->va, 0, mem->len);
		}

	return 0;
}

static struct rproc_ops hobot_mcore_rproc_ops = {
	.start = hobot_mcore_rproc_start,
	.stop = hobot_mcore_rproc_stop,
	.kick = hobot_mcore_rproc_kick,
	.da_to_va = hobot_mcore_rproc_da_to_va,
	.pre_load = hobot_mcore_rproc_pre_load,
};

static int hobot_dcore_rproc_start(struct rproc *rproc)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;

	pdata->ipc_ops.start_remoteproc(pdata->ipc_int_va);

	return 0;
}

static int hobot_dcore_rproc_stop(struct rproc *rproc)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;

	pdata->ipc_ops.release_remoteproc(pdata->ipc_int_va);

	return 0;
}

static void hobot_dcore_rproc_kick(struct rproc *rproc, int vqid)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;

	wmb();
	pdata->ipc_ops.trigger_interrupt(pdata->ipc_int_va);
}

static void *hobot_dcore_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct rproc_mem_entry *mem = NULL;
	void *va = NULL;

	// find suitable mem entry
	list_for_each_entry(mem, &rproc->mappings, node) {
		int offset = da - mem->da;

		if (offset < 0)
			continue;

		if (offset + len > mem->len)
			continue;

		va = mem->va + offset;
		break;
	}

	pr_info("convert: 0x%llx --> 0x%p\n", da, va);
	return va;
}

static int hobot_dcore_rproc_pre_load(struct rproc *rproc)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;
	struct rproc_mem_entry *mem = NULL;

	if (!rproc->register_virtio) {
		pdata->ipc_ops.pre_load_remoteproc(pdata->ipc_int_va);
		// clear sram
		//list_for_each_entry(mem, &rproc->mappings, node)
			//pr_info("mem->va is 0x%x,mem->len is 0x%x\n",mem->va,mem->len);
			//msleep(10);
			//memset_io(mem->va, 0, mem->len);
	}

	return 0;
}

static struct rproc_ops hobot_dcore_rproc_ops = {
	.start = hobot_dcore_rproc_start,
	.stop = hobot_dcore_rproc_stop,
	.kick = hobot_dcore_rproc_kick,
	.da_to_va = hobot_dcore_rproc_da_to_va,
	.pre_load = hobot_dcore_rproc_pre_load,
};

int rcore_interrupt_judge(void *ipc_int_va)
{
	unsigned int val = 0;
	val = readl(ipc_int_va + J3_IPI_APU_ISR_REG);
	if (val & (0x1 << J3_cr5_mbox_irq_bit))
		return 1;
	else
		return 0;
}

unsigned int find_firstzero_bit(unsigned int* bitmask)
{
        unsigned int i = 0;
        unsigned int bitcheck = *bitmask;
        if (bitcheck == J3_IPI_CMDBUF_FULLUSE) {
                *bitmask = 0;
                return i;
        }

        while (bitcheck) {
                bitcheck &= ~(1 << i);
                i++;
        }
        return i;
}

void rcore_trigger_interrupt(void *ipc_int_va)
{
	static unsigned int bitmask = 0;
        unsigned int bufidx;

        /* wait for CR5 response */
        bufidx = find_firstzero_bit(&bitmask);
        /* write msg */
        writel(0, ipc_int_va + J3_IPI_APU_REQ_BUFF(bufidx));
        /* trigger CR5 interrupt */
        writel((0x1 << J3_cr5_mbox_irq_bit), ipc_int_va + J3_IPI_APU_TRIG_REG);
        bitmask |= 1 << bufidx;
}

void rcore_clear_interrupt(void *ipc_int_va)
{
	//pr_dbg("rcore_clear_interrupt\n");

	writel((0x1 << J3_cr5_mbox_irq_bit), ipc_int_va + J3_IPI_APU_IER_REG);
	writel((0x1 << J3_cr5_mbox_irq_bit), ipc_int_va + J3_IPI_APU_ISR_REG);
        readl(ipc_int_va + J3_IPI_APU_ISR_REG); /* flush posted write */

        /* unmask CR5 interrupt */
        writel((0x1 << J3_cr5_mbox_irq_bit), ipc_int_va + J3_IPI_APU_IDR_REG);
}

void rcore_start_remoteproc(void *ipc_int_va)
{
	unsigned int val = 0;

	val = readl(ipc_int_va + J3_CPUSYS_CLKEN_SET);
	val |= (0x1 << cr5_aclk_cken_set_bit);
	writel(val, ipc_int_va + J3_CPUSYS_CLKEN_SET);
	msleep(10);

	// let rcore run
	val = readl(ipc_int_va + J3_CPUSYS_SW_RSTEN);
	val &= ~(0x1 << cr5_rst_en_bit);
	writel(val, ipc_int_va + J3_CPUSYS_SW_RSTEN);
	msleep(10);

	pr_info("rcore_start_remoteproc\n");
}

void rcore_release_remoteproc(void *ipc_int_va)
{
	unsigned int val = 0;

	// let rcore stop run
	val = readl(ipc_int_va + J3_CPUSYS_SW_RSTEN);
	val |= (0x1 << cr5_rst_en_bit);
	writel(val, ipc_int_va + J3_CPUSYS_SW_RSTEN);

	//disable clock
	val = readl(ipc_int_va + J3_CPUSYS_CLKEN_CLR);
	val |= (0x1 << cr5_aclk_cken_set_bit);
	writel(val, ipc_int_va + J3_CPUSYS_CLKEN_CLR);

	pr_info("rcore_release_remoteproc\n");
}

void rcore_pre_load_remoteproc(void *ipc_int_va)
{
	unsigned int val = 0;

	//1. rcore clock enable
	val = readl(ipc_int_va + J3_CPUSYS_CLKEN_SET);
	val |= (0x1 << cr5_aclk_cken_set_bit);
	writel(val, ipc_int_va + J3_CPUSYS_CLKEN_SET);
	msleep(10);

	//2. keep rcore reset state
	val = readl(ipc_int_va + J3_CPUSYS_SW_RSTEN);
	val |= (0x1 << cr5_rst_en_bit);
	writel(val, ipc_int_va + J3_CPUSYS_SW_RSTEN);
	msleep(10);

	pr_info("rcore_pre_load_remoteproc\n");
}

static struct rproc_ipc_ops hobot_rcore_ipc_ops = {
	.interrupt_judge = rcore_interrupt_judge,
	.trigger_interrupt = rcore_trigger_interrupt,
	.clear_interrupt = rcore_clear_interrupt,
	.start_remoteproc = rcore_start_remoteproc,
	.release_remoteproc = rcore_release_remoteproc,
	.pre_load_remoteproc = rcore_pre_load_remoteproc,
};

int mcore_interrupt_judge(void *ipc_int_va)
{
	unsigned int val = 0;

	val = readl(ipc_int_va + ACORE_VIC_OFFSET);
	if (val & (1 << m0_to_a_int_status_bit))
		return 1;
	else
		return 0;
}

void mcore_trigger_interrupt(void *ipc_int_va)
{
	unsigned int val = 0;

	val = readl(ipc_int_va + AON_MCORE_VIC_OFFSET);
	val |= (1 << a_to_m0_int_en_bit);
	writel(val, (ipc_int_va + AON_MCORE_VIC_OFFSET));

	val = readl(ipc_int_va + ACORE_GEN_VIC_OFFSET);
	val |= (1 << a_to_m0_int_req_bit);
	writel(val, (ipc_int_va + ACORE_GEN_VIC_OFFSET));

	//pr_info("mcore_trigger_interrupt\n");
}

void mcore_clear_interrupt(void *ipc_int_va)
{
	unsigned int val = 0;

	val = readl(ipc_int_va + ACORE_VIC_OFFSET);
	val &= ~(1 << m0_to_a_int_status_bit);
	writel(val, (ipc_int_va + ACORE_VIC_OFFSET));

	//pr_info("mcore_clear_interrupt\n");
}

void mcore_start_remoteproc(void *ipc_int_va)
{
	unsigned int val = 0;

	// mstar wait release
	val = readl(ipc_int_va + ACORE_VIC_OFFSET);
	val &= ~(1 << starcfg_cpuwait_bit);
	writel(val, (ipc_int_va + STAR_CFG_OFFSET));

	pr_info("mcore_start_remoteproc\n");
}

void mcore_release_remoteproc(void *ipc_int_va)
{
	unsigned int val = 0;

	// mstar reset hold
	val = readl(ipc_int_va + ANOSYS_SFT_ARES1_OFFSET);
	val |= (1 << mstar_sft_ares_bit);
	pr_info();
	writel(val, (ipc_int_va + ANOSYS_SFT_ARES1_OFFSET));

	// mstar wait hold
	val = readl(ipc_int_va + STAR_CFG_OFFSET);
	val |= ((1 << starcfg_inittcm_en_bit0) |
		(1 << starcfg_inittcm_en_bit1) |
		(1 << starcfg_cpuwait_bit));
	writel(val, (ipc_int_va + STAR_CFG_OFFSET));
	msleep(10);

	// mstar reset release
	val = readl(ipc_int_va + ANOSYS_SFT_ARES1_OFFSET);
	val &= ~(1 << mstar_sft_ares_bit);
	writel(val, (ipc_int_va + ANOSYS_SFT_ARES1_OFFSET));

	pr_info("mcore_release_remoteproc\n");
}

void mcore_pre_load_remoteproc(void *ipc_int_va)
{
	unsigned int val = 0;

	// mstar clock disable
	val = readl(ipc_int_va + AONSYS_CCLK_CFG0_OFFSET);
	val &= ~(1 << cgm_mstar_en_bit);
	writel(val, (ipc_int_va + AONSYS_CCLK_CFG0_OFFSET));

	// mstar reset hold
	val = readl(ipc_int_va + ANOSYS_SFT_ARES1_OFFSET);
	val |= (1 << mstar_sft_ares_bit);
	writel(val, (ipc_int_va + ANOSYS_SFT_ARES1_OFFSET));

	// mstar wait hold
	val = readl(ipc_int_va + STAR_CFG_OFFSET);
	val |= ((1 << starcfg_inittcm_en_bit0) |
		(1 << starcfg_inittcm_en_bit1) |
		(1 << starcfg_cpuwait_bit));
	writel(val, (ipc_int_va + STAR_CFG_OFFSET));
	msleep(10);

	// mstar clock enable
	val = readl(ipc_int_va + AONSYS_CCLK_CFG0_OFFSET);
	val |= (1 << cgm_mstar_en_bit);
	writel(val, (ipc_int_va + AONSYS_CCLK_CFG0_OFFSET));

	// mstar reset release
	val = readl(ipc_int_va + ANOSYS_SFT_ARES1_OFFSET);
	val &= ~(1 << mstar_sft_ares_bit);
	writel(val, (ipc_int_va + ANOSYS_SFT_ARES1_OFFSET));
	msleep(10);

	pr_info("mcore_pre_load_remoteproc\n");
}

static struct rproc_ipc_ops hobot_mcore_ipc_ops = {
	.interrupt_judge = mcore_interrupt_judge,
	.trigger_interrupt = mcore_trigger_interrupt,
	.clear_interrupt = mcore_clear_interrupt,
	.start_remoteproc = mcore_start_remoteproc,
	.release_remoteproc = mcore_release_remoteproc,
	.pre_load_remoteproc = mcore_pre_load_remoteproc,
};

int dcore0_interrupt_judge(void *ipc_int_va)
{
	unsigned int val = 0;

	val = readl(ipc_int_va + ACORE_VIC_OFFSET);
	if (val & (1 << vdsp0_to_a_int_status_bit))
		return 1;
	else
		return 0;
}

void dcore0_trigger_interrupt(void *ipc_int_va)
{
	unsigned int temp = 0;

	temp = readl(ipc_int_va + 0x224);
	temp |= (0x1 << 9);
	writel(temp, (ipc_int_va + 0x224));

	temp = readl(ipc_int_va + 0x200);
	temp |= (0x1 << 16);
	writel(temp, (ipc_int_va + 0x200));
}

void dcore0_clear_interrupt(void *ipc_int_va)
{
	unsigned int val = 0;
	val = readl(ipc_int_va + 0x204);
	writel(0x0, (ipc_int_va + 0x204));
}

void dcore0_start_remoteproc(void *ipc_int_va)
{
	writel(0x2, cvsysapb_reg_va + VDSP0_CTL_STATUS_OFFSET);//vdsp0_RunStall
}

void dcore0_release_remoteproc(void *ipc_int_va)
{
	writel(0x3, cvsysapb_reg_va + SOFT_RST_EN_OFFSET);//reset
	msleep(500);
	iounmap(cvsysapb_reg_va);
}

void dcore0_pre_load_remoteproc(void *ipc_int_va)
{
	cvsysapb_reg_va = ioremap_nocache(CVSYSAPB_REG_BASE, DCORE_IPC_INT_REG_RANGE);
	writel(0x3, cvsysapb_reg_va + VDSP0_CTL_STATUS_OFFSET); //vdsp0_StartVectorSel
	//vdsp0_AltResetVec
	writel(0x40000400, cvsysapb_reg_va + VDSP0_ALTRESETVEC_OFFSET);
	writel(0x1, cvsysapb_reg_va + SOFT_RST_EN_OFFSET);//release vdsp0 reset
	writel(0x07060904, cvsysapb_reg_va + 0x28);//litmmu 0x5xxxxxxx to 0x9xxxxxxx
}

static struct rproc_ipc_ops hobot_dcore0_ipc_ops = {
	.interrupt_judge = dcore0_interrupt_judge,
	.trigger_interrupt = dcore0_trigger_interrupt,
	.clear_interrupt = dcore0_clear_interrupt,
	.start_remoteproc = dcore0_start_remoteproc,
	.release_remoteproc = dcore0_release_remoteproc,
	.pre_load_remoteproc = dcore0_pre_load_remoteproc,
};

int dcore1_interrupt_judge(void *ipc_int_va)
{
	unsigned int val = 0;

	val = readl(ipc_int_va + ACORE_VIC_OFFSET);
	if (val & (1 << vdsp1_to_a_int_status_bit))
		return 1;
	else
		return 0;
}

void dcore1_trigger_interrupt(void *ipc_int_va)
{
	unsigned int val = 0;

	val = readl(ipc_int_va + ISP1_MCORE_VIC);
	val |= (1 << a_to_vdsp1_int_en_bit);
	writel(val, (ipc_int_va + ISP1_MCORE_VIC));

	val = readl(ipc_int_va + ACORE_GEN_VIC_OFFSET);
	val |= (1 << a_to_vdsp1_int_req_bit);
	writel(val, (ipc_int_va + ACORE_GEN_VIC_OFFSET));
}

void dcore1_clear_interrupt(void *ipc_int_va)
{
	unsigned int val = 0;

	val = readl(ipc_int_va + ACORE_VIC_OFFSET);
	val &= ~(1 << vdsp1_to_a_int_status_bit);
	writel(val, (ipc_int_va + ACORE_VIC_OFFSET));
}

void dcore1_start_remoteproc(void *ipc_int_va)
{
}

void dcore1_release_remoteproc(void *ipc_int_va)
{
}

void dcore1_pre_load_remoteproc(void *ipc_int_va)
{
}

static struct rproc_ipc_ops hobot_dcore1_ipc_ops = {
	.interrupt_judge = dcore1_interrupt_judge,
	.trigger_interrupt = dcore1_trigger_interrupt,
	.clear_interrupt = dcore1_clear_interrupt,
	.start_remoteproc = dcore1_start_remoteproc,
	.release_remoteproc = dcore1_release_remoteproc,
	.pre_load_remoteproc = dcore1_pre_load_remoteproc,
};

static const struct of_device_id hobot_remoteproc_dt_ids[] = {
	{.compatible = "hobot,remoteproc-mcore", },
	{.compatible = "hobot,remoteproc-rcore", },
	{.compatible = "hobot,remoteproc-dcore1", },
	{.compatible = "hobot,remoteproc-dcore0", },
	{ /* sentinel */ }
};

static int ipc_interrupt_init(struct platform_device *pdev)
{
	int ret = 0;
	struct hobot_rproc_pdata *pdata = (struct hobot_rproc_pdata *)(
	pdev->dev.driver_data);

	ret = of_property_read_u32(pdev->dev.of_node, "ipc-int-base-addr",
	&pdata->ipc_int_pa);
	if (ret) {
		dev_err(&pdev->dev, "get ipc-int-base-addr error\n");
		goto err;
	}

	if (pdata->device_index == MCORE_DEV_IDX) {
		pdata->ipc_int_va = ioremap_nocache(pdata->ipc_int_pa,
		MCORE_IPC_INT_REG_RANGE);
		pdata->ipc_ops = hobot_mcore_ipc_ops;
	} else if (pdata->device_index == RCORE_DEV_IDX) {
		pdata->ipc_int_va = ioremap_nocache(pdata->ipc_int_pa,
		RCORE_IPC_INT_REG_RANGE);
		pdata->ipc_ops = hobot_rcore_ipc_ops;
	} else if (pdata->device_index == DCORE0_DEV_IDX) {
		pdata->ipc_int_va = ioremap_nocache(pdata->ipc_int_pa,
		DCORE_IPC_INT_REG_RANGE);
		pdata->ipc_ops = hobot_dcore0_ipc_ops;
	} else {
		pdata->ipc_int_va = ioremap_nocache(pdata->ipc_int_pa,
		DCORE_IPC_INT_REG_RANGE);
		pdata->ipc_ops = hobot_dcore1_ipc_ops;
	}
	if (!pdata->ipc_int_va) {
		dev_err(&pdev->dev, "ioremap ipc interrupt register range error\n");
		goto err;
	}

	pr_info("ipc_int_pa = 0x%x\n", pdata->ipc_int_pa);
	pr_info("ipc_int_va = 0x%p\n", pdata->ipc_int_va);

	return 0;
err:
	return -1;
}

static void ipc_interrupt_deinit(struct platform_device *pdev)
{
	struct hobot_rproc_pdata *pdata = (struct hobot_rproc_pdata *)(
	pdev->dev.driver_data);

	iounmap(pdata->ipc_int_va);
}

static int hobot_remoteproc_work_cb(int id, void *ptr, void *data)
{
	struct rproc *rproc = (struct rproc *)data;
	rproc_virtio_interrupt(rproc, id);

	return 0;
}

static void hobot_remoteproc_work(struct work_struct *work)
{
	struct hobot_rproc_pdata *pdata =
	container_of(work, struct hobot_rproc_pdata, work);

	while (1) {
		if (wait_for_completion_interruptible(&pdata->notify_complete) < 0) {
			pr_err("wait_for_completion_interruptible error\n");
			break;
		}

		if (pdata->should_stop) {
			pr_err("work over\n");
			break;
		}

		idr_for_each(&(pdata->rproc->notifyids), hobot_remoteproc_work_cb,
		pdata->rproc);
	}
}

static irqreturn_t hobot_remoteproc_isr(int irq, void *param)
{
	struct hobot_rproc_pdata *pdata = (struct hobot_rproc_pdata *)param;
#ifdef TIMESTAMP_DEBUG
	struct timespec ts = current_kernel_time();
	pr_info("%s:%lds_%ldns\n", __func__, ts.tv_sec, ts.tv_nsec);
#endif
	if (pdata->ipc_ops.interrupt_judge(pdata->ipc_int_va)) {
		complete(&pdata->notify_complete);
		++pdata->statistics.irq_handler_count;
		pdata->ipc_ops.clear_interrupt(pdata->ipc_int_va);

		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static int irq_init(struct platform_device *pdev)
{
	struct hobot_rproc_pdata *pdata = (struct hobot_rproc_pdata *)(
	pdev->dev.driver_data);
	int ret = 0;

	pdata->vring0 = platform_get_irq(pdev, 0);
	if (pdata->vring0 < 0) {
		dev_err(&pdev->dev, "platform_get_irq error\n");
		goto err;
	}

	if (pdata->device_index == MCORE_DEV_IDX)
		pdata->work_queue =
		create_singlethread_workqueue("mcore_workqueue");
	else if (pdata->device_index == RCORE_DEV_IDX)
		pdata->work_queue =
		create_singlethread_workqueue("rcore_workqueue");
	else if (pdata->device_index == DCORE0_DEV_IDX)
		pdata->work_queue =
		create_singlethread_workqueue("dcore0_workqueue");
	else
		pdata->work_queue =
		create_singlethread_workqueue("dcore1_workqueue");
	if (!pdata->work_queue) {
		dev_err(&pdev->dev, "create_singlethread_workqueue error\n");
		goto err;
	}

	init_completion(&pdata->notify_complete);
	INIT_WORK(&pdata->work, hobot_remoteproc_work);
	pdata->should_stop = 0;

	if (queue_work(pdata->work_queue, &pdata->work) == false) {
		dev_err(&pdev->dev, "queue_work error\n");
		goto destroy_workqueu_out;
	}

	if (pdata->device_index == MCORE_DEV_IDX)
		ret = request_threaded_irq(pdata->vring0, hobot_remoteproc_isr,
		NULL, IRQF_SHARED, "mcore_irq", pdata);
	else if (pdata->device_index == RCORE_DEV_IDX)
		ret = request_threaded_irq(pdata->vring0, hobot_remoteproc_isr,
		NULL, IRQF_SHARED, "rcore_irq", pdata);
	else if (pdata->device_index == DCORE0_DEV_IDX)
		ret = request_threaded_irq(pdata->vring0, hobot_remoteproc_isr,
		NULL, IRQF_SHARED, "dcore0_irq", pdata);
	else
		ret = request_threaded_irq(pdata->vring0, hobot_remoteproc_isr,
		NULL, IRQF_SHARED, "dcore1_irq", pdata);
	if (ret) {
		dev_err(&pdev->dev, "request_threaded_irq error\n");
		goto stop_work_out;
	}

	return 0;
stop_work_out:
	pdata->should_stop = 1;
	complete(&pdata->notify_complete);
destroy_workqueu_out:
	destroy_workqueue(pdata->work_queue);
err:
	return -1;
}

static void irq_deinit(struct platform_device *pdev)
{
	struct hobot_rproc_pdata *pdata = (struct hobot_rproc_pdata *)(
	pdev->dev.driver_data);
	free_irq(pdata->vring0, pdata);
	pdata->should_stop = 1;
	complete(&pdata->notify_complete);
	destroy_workqueue(pdata->work_queue);
}

static int proc_info_show(struct seq_file *m, void *v)
{
	struct hobot_rproc_pdata *pdata =
	(struct hobot_rproc_pdata *)(m->private);
	unsigned int val = 0;

	if (pdata->device_index == MCORE_DEV_IDX)
		seq_printf(m, "mcore_info\n");
	else if (pdata->device_index == DCORE0_DEV_IDX)
		seq_printf(m, "dcore0_info\n");
	else
		seq_printf(m, "dcore1_info\n");
	seq_printf(m, "fix_map_mode = %d\n", pdata->fix_map_mode);
	seq_printf(m, "vring0 = %d\n", pdata->vring0);
	seq_printf(m, "ipc_int_pa = 0x%x\n", pdata->ipc_int_pa);
	seq_printf(m, "ipc_int_va = 0x%p\n", pdata->ipc_int_va);

	val = readl(pdata->ipc_int_va + AONSYS_CCLK_CFG0_OFFSET);
	seq_printf(m, "AONSYS_CCLK_CFG0_OFFSET[0x%x] = 0x%x\n",
	AONSYS_CCLK_CFG0_OFFSET, val);

	val = readl(pdata->ipc_int_va + ANOSYS_SFT_ARES1_OFFSET);
	seq_printf(m, "ANOSYS_SFT_ARES1_OFFSET[0x%x] = 0x%x\n",
	ANOSYS_SFT_ARES1_OFFSET, val);

	val = readl(pdata->ipc_int_va + STAR_CFG_OFFSET);
	seq_printf(m, "STAR_CFG_OFFSET[0x%x] = 0x%x\n",
	STAR_CFG_OFFSET, val);

	val = readl(pdata->ipc_int_va + ACORE_GEN_VIC_OFFSET);
	seq_printf(m, "ACORE_GEN_VIC_OFFSET[0x%x] = 0x%x\n",
	ACORE_GEN_VIC_OFFSET, val);

	val = readl(pdata->ipc_int_va + ACORE_VIC_OFFSET);
	seq_printf(m, "ACORE_VIC_OFFSET[0x%x] = 0x%x\n",
	ACORE_VIC_OFFSET, val);

	val = readl(pdata->ipc_int_va + AON_MCORE_VIC_OFFSET);
	seq_printf(m, "AON_MCORE_VIC_OFFSET[0x%x] = 0x%x\n",
	AON_MCORE_VIC_OFFSET, val);

	val = readl(pdata->ipc_int_va + VDSP0_VIC_OFFSET);
	seq_printf(m, "VDSP0_VIC_OFFSET[0x%x] = 0x%x\n",
	VDSP0_VIC_OFFSET, val);

	val = readl(pdata->ipc_int_va + ISP1_MCORE_VIC);
	seq_printf(m, "ISP1_MCORE_VIC[0x%x] = 0x%x\n",
	ISP1_MCORE_VIC, val);

	return 0;
}

static ssize_t proc_info_write(struct file *file, const char __user *buffer,
size_t count, loff_t *ppos)
{
	// do nothing, prevent modifing hobot_rproc_pdata

	return count;
}

static int proc_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_info_show, PDE_DATA(inode));
}

static const struct file_operations info_ops = {
	.owner = THIS_MODULE,
	.open = proc_info_open,
	.read = seq_read,
	.write = proc_info_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int proc_statistics_show(struct seq_file *m, void *v)
{
	struct hobot_rproc_pdata *pdata =
	(struct hobot_rproc_pdata *)(m->private);

	if (pdata->device_index == MCORE_DEV_IDX)
		seq_printf(m, "mcore_statistics\n");
	else if (pdata->device_index == RCORE_DEV_IDX)
		seq_printf(m, "rcore_statistics\n");
	else if (pdata->device_index == DCORE0_DEV_IDX)
		seq_printf(m, "dcore0_statistics\n");
	else
		seq_printf(m, "dcore1_statistics");
	seq_printf(m, "irq_handler_count = %d\n",
	pdata->statistics.irq_handler_count);

	return 0;
}

static ssize_t proc_statistics_write(struct file *file,
const char __user *buffer, size_t count, loff_t *ppos)
{
	struct hobot_rproc_pdata *pdata = (struct hobot_rproc_pdata *)
	(((struct seq_file *)file->private_data)->private);

	memset(&pdata->statistics, 0, sizeof(pdata->statistics));

	return count;
}

static int proc_statistics_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_statistics_show, PDE_DATA(inode));
}

static const struct file_operations statistics_ops = {
	.owner = THIS_MODULE,
	.open = proc_statistics_open,
	.read = seq_read,
	.write = proc_statistics_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int proc_node_init(struct platform_device *pdev)
{
	struct hobot_rproc_pdata *pdata = (struct hobot_rproc_pdata *)(
	pdev->dev.driver_data);

	// make top_entry
	if (pdata->device_index == MCORE_DEV_IDX)
		pdata->top_entry = proc_mkdir("mcore", NULL);
	else if (pdata->device_index == RCORE_DEV_IDX)
		pdata->top_entry = proc_mkdir("rcore", NULL);
	else if (pdata->device_index == DCORE0_DEV_IDX)
		pdata->top_entry = proc_mkdir("dcore0", NULL);
	else
		pdata->top_entry = proc_mkdir("dcore1", NULL);
	if (!pdata->top_entry) {
		pr_err("create top_entry error\n");
		goto create_top_entry_err;
	}

	// make info entry
	pdata->info_entry = proc_create_data("info", 0777,
	pdata->top_entry, &info_ops, pdata);
	if (!pdata->info_entry) {
		pr_err("create info_entry error\n");
		goto create_info_entry_err;
	}

	// make statistics entry
	pdata->statistics_entry = proc_create_data("statistics", 0777,
	pdata->top_entry, &statistics_ops, pdata);
	if (!pdata->statistics_entry) {
		pr_err("create statistics_entry error\n");
		goto create_statistics_entry_err;
	}

	return 0;
create_statistics_entry_err:
	remove_proc_entry("info", pdata->top_entry);
create_info_entry_err:
	if (pdata->device_index == MCORE_DEV_IDX)
		remove_proc_entry("mcore", NULL);
	else if (pdata->device_index == RCORE_DEV_IDX)
		remove_proc_entry("rcore", NULL);
	else if (pdata->device_index == DCORE0_DEV_IDX)
		remove_proc_entry("dcore0", NULL);
	else
		remove_proc_entry("dcore1", NULL);
create_top_entry_err:
	return -1;
}

static void proc_node_deinit(struct platform_device *pdev)
{
	struct hobot_rproc_pdata *pdata = (struct hobot_rproc_pdata *)(
	pdev->dev.driver_data);

	remove_proc_entry("statistics", pdata->top_entry);
	remove_proc_entry("info", pdata->top_entry);
	if (pdata->device_index == MCORE_DEV_IDX)
		remove_proc_entry("mcore", NULL);
	if (pdata->device_index == RCORE_DEV_IDX)
		remove_proc_entry("rcore", NULL);
	else if (pdata->device_index == DCORE0_DEV_IDX)
		remove_proc_entry("dcore0", NULL);
	else
		remove_proc_entry("dcore1", NULL);
}

static struct resource_table hobot_rproc_empty_rsc_table = {
	.ver = 1,
	.num = 0,
};

static struct resource_table *hobot_rproc_find_rsc_table(
struct rproc *rproc, const struct firmware *fw, int *tablesz)
{
	struct hobot_rproc_pdata *pdata = rproc->priv;
	struct resource_table *rsc = NULL;

	rsc = pdata->default_fw_ops->find_rsc_table(rproc, fw, tablesz);
	if (!rsc) {
		pr_err("return empty rsc table\n");
		*tablesz = sizeof(hobot_rproc_empty_rsc_table);
		return &hobot_rproc_empty_rsc_table;
	} else {
		return rsc;
	}
}

static int hobot_remoteproc_probe(struct platform_device *pdev)
{
	const struct of_device_id *device_id = NULL;
	int device_index = 0;
	struct rproc *rproc = NULL;
	struct hobot_rproc_pdata *pdata = NULL;
	int ret = 0;

	pr_info("hobot_remoteproc_probe start\n");

	device_id = of_match_device(hobot_remoteproc_dt_ids, &pdev->dev);
	if (!device_id) {
		pr_err("invalid match device\n");
		goto no_revoke_err;
	}
	pr_info("compatible = %s\n", device_id->compatible);

	if (!strcmp(device_id->compatible, "hobot,remoteproc-mcore"))
		device_index = MCORE_DEV_IDX;
	else if (!strcmp(device_id->compatible, "hobot,remoteproc-rcore"))
		device_index = RCORE_DEV_IDX;
	else if (!strcmp(device_id->compatible, "hobot,remoteproc-dcore0"))
		device_index = DCORE0_DEV_IDX;
	else
		device_index = DCORE1_DEV_IDX;

	if (device_index == MCORE_DEV_IDX)
		rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev),
		&hobot_mcore_rproc_ops, NULL, sizeof(struct hobot_rproc_pdata));
	if (device_index == RCORE_DEV_IDX)
		rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev),
		&hobot_rcore_rproc_ops, NULL, sizeof(struct hobot_rproc_pdata));
	else
		rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev),
		&hobot_dcore_rproc_ops, NULL, sizeof(struct hobot_rproc_pdata));
	if (!rproc) {
		dev_err(&pdev->dev, "rproc_alloc error\n");
		goto no_revoke_err;
	}
	pdata = rproc->priv;
	pdata->rproc = rproc;
	pdata->device_index = device_index;

	ret = of_property_read_u32(pdev->dev.of_node, "fix-map-mode",
	&pdata->fix_map_mode);
	if (ret) {
		dev_err(&pdev->dev, "get fix-map-mode error\n");
		goto free_rproc_out;
	}
	pr_info("fix_map_mode = %d", pdata->fix_map_mode);
	rproc->fix_map_mode = pdata->fix_map_mode;

	platform_set_drvdata(pdev, pdata);

	/* FIXME: it may need to extend to 64/48 bit */
	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(48));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask: %d\n", ret);
		goto free_rproc_out;
	}

	ret = ipc_interrupt_init(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "ipc_interrupt_init error\n");
		goto free_rproc_out;
	}

	ret = irq_init(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "irq_init error\n");
		goto deinit_ipc_out;
	}

	ret = proc_node_init(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "proc_node_init error\n");
		goto deinit_irq_out;
	}

	// we will complete remoteproc init
	rproc->auto_boot = autoboot;
	memcpy(&pdata->fw_ops, rproc->fw_ops, sizeof(pdata->fw_ops));
	pdata->fw_ops.find_rsc_table = hobot_rproc_find_rsc_table;
	pdata->default_fw_ops = rproc->fw_ops;
	rproc->fw_ops = &pdata->fw_ops;

	ret = rproc_add(pdata->rproc);
	if (ret) {
		dev_err(&pdev->dev, "rproc_add error\n");
		goto deinit_proc_out;
	}

	pr_info("hobot_remoteproc_probe end\n");

	return 0;
deinit_proc_out:
	proc_node_deinit(pdev);
deinit_irq_out:
	irq_deinit(pdev);
deinit_ipc_out:
	ipc_interrupt_deinit(pdev);
free_rproc_out:
	rproc_free(pdata->rproc);
no_revoke_err:
	return -1;
}

static struct platform_driver hobot_remoteproc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "hobot-remoteproc",
		.of_match_table = hobot_remoteproc_dt_ids,
	},
	.probe = hobot_remoteproc_probe,
};

static int __init hobot_remoteproc_init(void)
{
	int ret = 0;

	pr_info("hobot_remoteproc_init start\n");

	ret = platform_driver_register(&hobot_remoteproc_driver);
	if (ret)
		pr_err("platform_driver_register error\n");

	pr_info("hobot_remoteproc_init end\n");

	return ret;
}

MODULE_AUTHOR("Horizon Robotics, Inc");
MODULE_DESCRIPTION("Hobot remote processor device");
MODULE_LICENSE("GPL");
module_init(hobot_remoteproc_init);

