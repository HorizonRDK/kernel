/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":" fmt

#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/ion.h>
#include <linux/cma.h>
#include <linux/dma-contiguous.h>

#include "./hobot_ddr_mpu.h"

#define RPU_VIOLT_USR  0xffffffff
#define RPU_VIOLT_ADDR 0xffffffff
#define MPU_VIOLT_USR_RESERVED  0xffffffff
#define MPU_VIOLT_ADDR_RESERVED 0xffffffff

struct cma_info {
	char *name;
	phys_addr_t cma_start;
	unsigned long cma_size;
};

static struct mpu_protection mpu_prt;

extern u32 read_axibus_reg(void);

static void mpu_work_handler(struct work_struct *w);
static DECLARE_DELAYED_WORK(mpu_work, mpu_work_handler);

static int print_stack = 0;
module_param(print_stack, int, 0644);

static int print_axicfg = 1;
module_param(print_axicfg, int, 0644);

char *fw_user_id_tbl[FW_USR_ID_FW_USR_ID_MAX] = {
    "CA53",                     //0
    "CR5",
    "BIFSPI",
    "BIFSD",
    "DMAC",
    "USB",                      //5
    "AES",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "CNN0_Fetch",               //12
    "CNN1_Fetch",
    "CNN0_Other",
    "CNN1_Other",
    "UART0",
    "UART1",                    //17
    "UART2",
    "UART3",
    "SPI0",
    "SPI1",
    "SPI2",                     //22
    "I2S0",
    "I2S1",
    "SDIO_0",                   //25
    "SDIO_1",
    "SDIO_2",
    "GMAC",
    "VIO_M0",                   //29
    "VPU/JPG",                  //30
    "VIO_M1"                    //31
};

char *fw_slv_id[SLV_DIS_ID_MAX] = {
    "DDR",
    "SRAM",
    "VIO_IRAM",
    "QSPI",
    "DBG_SYS",
    "CPU_SYS",
    "DDR_SYS",
    "CNN_SYS",
    "VIO_SYS",
    "PERI_SYS",
    "PMU_SYS",
    "SEC_REG",
    "VSP_SYS",
    "USB"
};

char *mpu_vio_block[MPU_BLOCK_NUM] = {
    "BPU0_FETCH_RANGE_VIOLT",
    "BPU1_FETCH_RANGE_VIOLT",
    "DRAM_BPU_RANGE_VIOLT",
    "DRAM_MAX_RANGE_VIOLT",
    "MPU_ADDR0_RANGE_VIOLT",
    "MPU_ADDR1_RANGE_VIOLT",
    "MPU_ADDR2_RANGE_VIOLT",
    "MPU_ADDR3_RANGE_VIOLT",
};

u32 vio_address_tbl[FW_PORT_ID_NUM][3] = {
    /* BIT map; user off; addr offset */
    {FW_PORT_ID_RPU_BSPI, RPU_BIFSPI_VIOLT_SLV, RPU_VIOLT_ADDR},
    {FW_PORT_ID_RPU_BSD, RPU_BIFSD_VIOLT_SLV, RPU_VIOLT_ADDR},
    {FW_PORT_ID_RPU_CPU, RPU_VIOLT_USR, RPU_VIOLT_ADDR},

    {FW_PORT_MPU_Reserved, MPU_VIOLT_USR_RESERVED, MPU_VIOLT_ADDR_RESERVED},
    {FW_PORT_MPU_Reserved, MPU_VIOLT_USR_RESERVED, MPU_VIOLT_ADDR_RESERVED},
    {FW_PORT_MPU_Reserved, MPU_VIOLT_USR_RESERVED, MPU_VIOLT_ADDR_RESERVED},
    {FW_PORT_MPU_Reserved, MPU_VIOLT_USR_RESERVED, MPU_VIOLT_ADDR_RESERVED},
    {FW_PORT_MPU_Reserved, MPU_VIOLT_USR_RESERVED, MPU_VIOLT_ADDR_RESERVED},

    {FW_PORT_ID_MPU_CNN0, MPU_CNN0_VIOLT_USER, MPU_CNN0_VIOLT_ADDR},
    {FW_PORT_ID_MPU_CNN1, MPU_CNN1_VIOLT_USER, MPU_CNN1_VIOLT_ADDR},
    {FW_PORT_ID_MPU_CPU, MPU_CPU_VIOLT_USER, MPU_CPU_VIOLT_ADDR},
    {FW_PORT_ID_MPU_R5, MPU_R5_VIOLT_USER, MPU_R5_VIOLT_ADDR},
    {FW_PORT_ID_MPU_CPUSS, MPU_CPUSS_VIOLT_USER, MPU_CPUSS_VIOLT_ADDR},
    {FW_PORT_ID_MPU_VIOSS, MPU_VIOSS_VIOLT_USER, MPU_VIOSS_VIOLT_ADDR},
    {FW_PORT_ID_MPU_VSPSS, MPU_VSPSS_VIOLT_USER, MPU_VSPSS_VIOLT_ADDR},
    {FW_PORT_ID_MPU_VSPVIO, MPU_VSPVIOSS_VIOLT_USER, MPU_VSPVIOSS_VIOLT_ADDR},
    {FW_PORT_ID_MPU_PERI, MPU_PERISS_VIOLT_USER, MPU_PERISS_VIOLT_ADDR}
};


#define RPU_VIOLATION_MAX  2
#define RPU_VIOLATION_BSPI 0
#define RPU_VIOLATION_BSD  1

static void mpu_work_handler(struct work_struct *w)
{
	u32 axicfg = read_axibus_reg();

	pr_err("mpu violation: axicfg: 0x%08x\n", axicfg);

	return;
}

static void dump_mpu_regs(void)
{
	void __iomem *base = mpu_prt.secreg;

	pr_err("0xA6008200: 0x%08x BPU0_FETCH_S_RANGE\n", readl(base + MPU_BPU0_FETCH_S_RANGE));
	pr_err("0xA6008204: 0x%08x BPU0_FETCH_E_RANGE\n", readl(base + MPU_BPU0_FETCH_E_RANGE));
	pr_err("0xA6008208: 0x%08x BPU1_FETCH_S_RANGE\n", readl(base + MPU_BPU1_FETCH_S_RANGE));
	pr_err("0xA600820C: 0x%08x BPU1_FETCH_E_RANGE\n", readl(base + MPU_BPU1_FETCH_E_RANGE));
	pr_err("0xA6008210: 0x%08x DRAM_BPU_S_RANGE  \n", readl(base + MPU_DRAM_BPU_S_RANGE));
	pr_err("0xA6008214: 0x%08x DRAM_BPU_E_RANGE  \n", readl(base + MPU_DRAM_BPU_E_RANGE));
	pr_err("0xA6008218: 0x%08x DRAM_MAX_S_RANGE  \n", readl(base + MPU_DRAM_MAX_S_RANGE));
	pr_err("0xA600821C: 0x%08x DRAM_MAX_E_RANGE  \n", readl(base + MPU_DRAM_MAX_E_RANGE));
	pr_err("0xA6008220: 0x%08x DRAM_CNN_S_RANGE  \n", readl(base + MPU_DRAM_BPU_S_RANGE));
	pr_err("0xA6008224: 0x%08x DRAM_CNN_E_RANGE  \n", readl(base + MPU_DRAM_BPU_E_RANGE));
	pr_err("0xA6008240: 0x%08x BPU0_FETCH_USER   \n", readl(base + MPU_BPU0_FETCH_USER));
	pr_err("0xA6008244: 0x%08x BPU1_FETCH_USER   \n", readl(base + MPU_BPU1_FETCH_USER));
	pr_err("0xA6008248: 0x%08x DRAM_BPU_USER     \n", readl(base + MPU_DRAM_BPU_USER));
	pr_err("0xA600824C: 0x%08x DRAM_MAX_USER     \n", readl(base + MPU_DRAM_MAX_USER));
	pr_err("0xA6008250: 0x%08x DRAM_CNN_USER     \n", readl(base + MPU_DRAM_BPU_USER));
	pr_err("0xA6008300: 0x%08x MPU_S_RANGE0      \n", readl(base + MPU_S_RANGE0));
	pr_err("0xA6008304: 0x%08x MPU_E_RANGE0      \n", readl(base + MPU_E_RANGE0));
	pr_err("0xA6008308: 0x%08x MPU_S_RANGE1      \n", readl(base + MPU_S_RANGE1));
	pr_err("0xA600830C: 0x%08x MPU_E_RANGE1      \n", readl(base + MPU_E_RANGE1));
	pr_err("0xA6008310: 0x%08x MPU_S_RANGE2      \n", readl(base + MPU_S_RANGE2));
	pr_err("0xA6008314: 0x%08x MPU_E_RANGE2      \n", readl(base + MPU_E_RANGE2));
	pr_err("0xA6008318: 0x%08x MPU_S_RANGE3      \n", readl(base + MPU_S_RANGE3));
	pr_err("0xA600831C: 0x%08x MPU_E_RANGE3      \n", readl(base + MPU_E_RANGE3));
	pr_err("0xA6008320: 0x%08x MPU_RANGE0_RUSER  \n", readl(base + MPU_RANGE0_RUSER));
	pr_err("0xA6008324: 0x%08x MPU_RANGE1_RUSER  \n", readl(base + MPU_RANGE1_RUSER));
	pr_err("0xA6008328: 0x%08x MPU_RANGE2_RUSER  \n", readl(base + MPU_RANGE2_RUSER));
	pr_err("0xA600832C: 0x%08x MPU_RANGE3_RUSER  \n", readl(base + MPU_RANGE3_RUSER));
	pr_err("0xA6008330: 0x%08x MPU_RANGE0_WUSER  \n", readl(base + MPU_RANGE0_WUSER));
	pr_err("0xA6008334: 0x%08x MPU_RANGE1_WUSER  \n", readl(base + MPU_RANGE1_WUSER));
	pr_err("0xA6008338: 0x%08x MPU_RANGE2_WUSER  \n", readl(base + MPU_RANGE2_WUSER));
	pr_err("0xA600833C: 0x%08x MPU_RANGE3_WUSER  \n", readl(base + MPU_RANGE3_WUSER));
	pr_err("0xA6008380: 0x%08x MPU_DEFAULT_ADDR  \n", readl(base + MPU_DEFAULT_ADDR));
}

static void check_mpu_violation(int check_write, int port_id)
{
    u32 user_id = 0;
    u32 addr = 0;
	int j;
	int reg_offset = 0;
	char *rw_type = check_write ? "write" : "read";

	if (check_write)
		reg_offset = 0x80;

	/*
	 * RUSER/RADDR and WUSER/WADDR registers address difference are 0x80,
	 * so we can use same code for RUSER and WUSER checking with this offset
	 */
	user_id = readl(mpu_prt.sysctrl +
			vio_address_tbl[port_id][1] + reg_offset);
	addr = readl(mpu_prt.sysctrl +
			vio_address_tbl[port_id][2] + reg_offset);

	if (user_id != 0 || addr != 0) {
		pr_err("user_id_reg:0x%08x, addr_reg_addr=0x%08x",
				vio_address_tbl[port_id][1] + reg_offset + 0xA1000900,
				vio_address_tbl[port_id][2] + reg_offset + 0xA1000900);
		pr_err("%s checking: user_id:%08x, addr_reg=0x%08x\n",
				rw_type, user_id, addr);
	}

	if (user_id != 0) {
		for (j = 0; j < FW_USR_ID_FW_USR_ID_MAX; j++) {
			if (user_id & (1 << j))
				pr_err("!!MPU %s violation!!! IP = [%s]\n",
					rw_type, fw_user_id_tbl[j]);
		}
	}

	if (addr != 0) {
		pr_debug("!!MPU %s violation!!! page address = 0x%08x\n",
				rw_type, (addr >> 16) << 12);

		for (j = 0; j < MPU_BLOCK_NUM; j++) {
			if (addr & (1 << j))
				pr_err("%s violation block name: [%s]\n",
					rw_type, mpu_vio_block[j]);
		}
	}
}

static void check_rpu_violation(int port_id)
{
    const char *str_bspi = "BIFSPI";
    const char *str_bsd = "BIFSD";
    const char *str_rpu_vio = "";
    u32 rpu_vio_sta = 0;
	int j;

	if (port_id == RPU_VIOLATION_BSPI) {
		rpu_vio_sta =
			readl(mpu_prt.sysctrl + RPU_BIFSPI_VIOLT_SLV);
		str_rpu_vio = str_bspi;
	} else if (port_id == RPU_VIOLATION_BSD) {
		rpu_vio_sta =
			readl(mpu_prt.sysctrl + RPU_BIFSD_VIOLT_SLV);
		str_rpu_vio = str_bsd;
	} else {
		pr_err("RPU slave id is not supported.\n");
	}

	/* RPU violation */
	for (j = 0; j < SLV_DIS_ID_MAX; j++) {
		if (rpu_vio_sta & (1 << j))
			pr_err("!!RPU Violation!! %s are not surposed to access: [%s]\n",
				 str_rpu_vio, fw_slv_id[j]);
	}
}

static irqreturn_t mpu_protection_isr(int this_irq, void *data)
{
    u32 vio_sta = 0;
    int i = 0;

    vio_sta = readl(mpu_prt.sysctrl + FW_IRQ_STA);

	pr_err("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    pr_err("mpu violation status: [0x%08x]\n", vio_sta);

    for (i = 0; i < FW_PORT_ID_NUM; i++) {
        if (vio_sta & (1 << i))
			break;
	}

	if (i > RPU_VIOLATION_MAX) {
		check_mpu_violation(0, i);
		check_mpu_violation(1, i);
	} else {
		check_rpu_violation(i);
	}

	/* Clear IRQ */
	writel((vio_sta & (1 << i)),  mpu_prt.sysctrl + FW_IRQ_CLR);
	pr_err("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

	if (print_stack)
		dump_stack();

	pr_err("\n");

	if (print_axicfg)
		queue_delayed_work(mpu_prt.wq, &mpu_work, 0);

	dump_mpu_regs();

	return IRQ_HANDLED;
}

/*
 * This is cma_for_each_area iterator, get cma_start address and size
 * for matched cma name.
 */
static int cma_get_range(struct cma *cma, void *data)
{
	struct cma_info *info = (struct cma_info *)data;
	char *name = cma_get_name(cma);

	if (!strncmp(info->name, name, strlen(name))) {
		info->cma_start = cma_get_base(cma);
		info->cma_size = cma_get_size(cma);
		pr_debug("got cma name:%s, cma_base:0x%08x, size:0x%08x\n",
			info->name, info->cma_start, info->cma_size);
		return 1;
	}

	return 0;
}

static int ion_is_in_range(u32 phy_start, u32 phy_end)
{
	struct device_node *node;
	struct resource ion_res;
	const char *status;

	node = of_find_node_by_path("/reserved-memory");
	if (node) {
		node = of_find_compatible_node(node, NULL, "ion-pool");
		if (node) {
			status = of_get_property(node, "status", NULL);
			if ((!status) || (strcmp(status, "okay") == 0)
					|| (strcmp(status, "ok") == 0)) {
				if (!of_address_to_resource(node, 0, &ion_res)) {
					pr_debug("%s:ION Carveout MEM start 0x%llx, size 0x%lx\n",
							__func__, ion_res.start, ion_res.end);
					if((phy_start <= ion_res.start) &&
						(phy_end >= (ion_res.end))) {
						pr_debug("ion is in range :%08x - %08x\n",
								phy_start, phy_end);
						return 1;
					}
				}
			}
		}
	}

	return 0;
}

/*
 * MPU Protect ranges:
 * range0: bl31 + .text + .rodata, CPU only
 * range1: from .init to cma start, block BPU,VIO,VPU
 * range2: from ion_cma end to reserved cma start, block BPU,VIO,VPU
 */
static int ddr_mpu_protect_kernel(void)
{
	u32 phy_start0 = 0;
	u32 phy_end0   = virt_to_phys(__end_rodata);
	u32 phy_start1 = virt_to_phys(_sinittext);
	u32 phy_end1   = 0; /* cma start */
	u32 phy_start2 = 0; /* cma end   */
	u32 phy_end2   = 0; /* end of memory */
	struct cma_info ion_cma = {"ion_cma", 0, 0};
	struct cma_info reserved_cma = {"reserved", 0, 0};

	cma_for_each_area(cma_get_range, &ion_cma);
	cma_for_each_area(cma_get_range, &reserved_cma);

	phy_end1 = (u32)ion_cma.cma_start;
	phy_start2 = (u32)ion_cma.cma_start + (u32)ion_cma.cma_size;
	phy_end2 = reserved_cma.cma_start;

	pr_debug("Protect range0 [0x%08x - 0x%08x]\n", phy_start0, phy_end0);
	pr_debug("Protect range1 [0x%08x - 0x%08x]\n", phy_start1, phy_end1);
	pr_debug("Protect range2 [0x%08x - 0x%08x]\n", phy_start2, phy_end2);

	/* range 0: protect BL31, kernel .text and .rodata; CPU only */
	writel(phy_end0 >> 12, mpu_prt.secreg + MPU_E_RANGE0);
	writel(0xF1FFFFFE, mpu_prt.secreg + MPU_RANGE0_WUSER);

	/* range 1, from kernel .init to CMA start, block BPU,VIO,VPU */
	writel(phy_start1 >> 12, mpu_prt.secreg + MPU_S_RANGE1);
	writel(phy_end1 >> 12, mpu_prt.secreg + MPU_E_RANGE1);
	if (ion_is_in_range(phy_start1, phy_end1))
		writel(0xE0000000, mpu_prt.secreg + MPU_RANGE1_WUSER);
	else
		writel(0xE000F000, mpu_prt.secreg + MPU_RANGE1_WUSER);

	/* range 2, from CMA end to memory end, block BPU,VIO,VPU */
	writel(phy_start2 >> 12, mpu_prt.secreg + MPU_S_RANGE2);
	writel(phy_end2 >> 12, mpu_prt.secreg + MPU_E_RANGE2);
	if (ion_is_in_range(phy_start2, phy_end2))
		writel(0xE0000000, mpu_prt.secreg + MPU_RANGE2_WUSER);
	else
		writel(0xE000F000, mpu_prt.secreg + MPU_RANGE2_WUSER);

	return 0;
}


int ddr_mpu_init(struct platform_device *pdev)
{
	struct resource *pres;
	int ret;

	/* support MPU start */
	pres = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!pres) {
		pr_err("DDR controller base is not detected.\n");
		return -ENODEV;
	}

	pr_debug("sysctrl resource :%pr\n", pres);

	mpu_prt.sysctrl = devm_ioremap_resource(&pdev->dev, pres);
	if (IS_ERR(mpu_prt.sysctrl)) {
		pr_err("mpu base address map failed, %ld\n", PTR_ERR(mpu_prt.sysctrl));
		return PTR_ERR(mpu_prt.sysctrl);
	}

	/* support MPU start */
	pres = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!pres) {
		pr_err("DDR controller base is not detected.\n");
		return -ENODEV;
	}

	pr_debug("sec reg resource :%pr\n", pres);

	mpu_prt.secreg = devm_ioremap_resource(&pdev->dev, pres);
	if (IS_ERR(mpu_prt.secreg)) {
		pr_err("secreg base address map failed, %ld\n", PTR_ERR(mpu_prt.secreg));
		return PTR_ERR(mpu_prt.secreg);
	}

	mpu_prt.irq = platform_get_irq(pdev, 2);
	if (mpu_prt.irq < 0) {
		pr_info("sec_fw irq is not in dts\n");
		return -ENODEV;
	}

	ret = request_irq(mpu_prt.irq, mpu_protection_isr, IRQF_TRIGGER_HIGH,
				  "mpu_prt", &mpu_prt);
	if (ret) {
		dev_err(&pdev->dev, "Could not request mpu_prt irq %d\n",
				mpu_prt.irq);
		return -ENODEV;
	}

	mpu_prt.wq = create_singlethread_workqueue("mpu_wq");
	if (mpu_prt.wq == NULL) {
		dev_err(&pdev->dev, "failed to create mpu workqueue\n");
		return -ENODEV;
	}

	ddr_mpu_protect_kernel();

	return 0;
}

int ddr_mpu_deinit(struct platform_device *pdev)
{
	destroy_workqueue(mpu_prt.wq);

	return  0;
}
