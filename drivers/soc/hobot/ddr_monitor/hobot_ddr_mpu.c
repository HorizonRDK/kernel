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

#include "./hobot_ddr_mpu.h"

#define RPU_VIOLT_USR  0xffffffff
#define RPU_VIOLT_ADDR 0xffffffff
#define MPU_VIOLT_USR_RESERVED  0xffffffff
#define MPU_VIOLT_ADDR_RESERVED 0xffffffff

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

	return IRQ_HANDLED;
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

	pr_info("sysctrl resource :%pr\n", pres);

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

	pr_info("sec reg resource :%pr\n", pres);

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

	return 0;
}

int ddr_mpu_deinit(struct platform_device *pdev)
{
	destroy_workqueue(mpu_prt.wq);

	return  0;
}
