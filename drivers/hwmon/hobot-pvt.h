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

/*
 * PVT sensors on HOBOT X3
 */

#ifndef __HOBOT_PVT_H__
#define __HOBOT_PVT_H__

#define PVT_COMMON_OFFSET    0x0000
#define PVT_INTERRUPT_OFFSET 0x0040

#define PVT_TS_BASE_OFFSET   0x0080
#define PVT_TS_IP_OFFSET     0x00C0
#define PVT_TS_EACH_SIZE     0x0040
#define PVT_TS_NUM           4

#define PVT_PD_BASE_OFFSET   0x0200
#define PVT_PD_IP_OFFSET     0x0240
#define PVT_PD_EACH_SIZE     0x0040
#define PVT_PD_NUM           3

#define PVT_VM_COM_OFFSET    0x0400
#define PVT_VM_IP_OFFSET     0x0600
#define PVT_VM_EACH_SIZE     0x0200
#define PVT_VM_NUM           16


// TS  4 PD = 3 VM = 1 CH = 16
#define PVT_START_ADDR   0x0000
#define PVT_END_ADDR     0x0040
#define PVT_SIZE         0x0040

#define IRQ_START_ADDR   0x0040
#define IRQ_END_ADDR     0x0080
#define IRQ_SIZE         0x0040

#define TS_START_ADDR   0x0080
#define TS_END_ADDR     0x01c0
#define TS_SIZE         0x0140

#define PD_START_ADDR   0x0200
#define PD_END_ADDR     0x0300
#define PD_SIZE         0x0100

#define VM_START_ADDR   0x0400
#define VM_END_ADDR     0x0a00
#define VM_SIZE         0x0600

#define PVT_COMP_ID_ADDR         0x0000
#define PVT_IP_CFG_ADDR          0x0004
#define PVT_ID_NUM_ADDR          0x0008
#define PVT_TM_SCRATCH_ADDR      0x000c
#define PVT_REG_LOCK_ADDR        0x0010
#define PVT_LOCK_STATUS_ADDR     0x0014
#define PVT_TAM_STATUS_ADDR      0x0018
#define PVT_TAM_CLEAR_ADDR       0x001c
#define PVT_TMR_CTRL_ADDR        0x0020
#define PVT_TMR_STATUS_ADDR      0x0024
#define PVT_TMR_IRQ_CLEAR_ADDR   0x0028
#define PVT_TMR_IRQ_TEST_ADDR    0x002c

#define IRQ_EN_ADDR           0x0040
#define IRQ_EN_TMR_BIT		BIT(0)
#define IRQ_EN_TS_BIT		BIT(1)
#define IRQ_EN_VM_BIT		BIT(2)
#define IRQ_EN_PD_BIT		BIT(3)
#define IRQ_RES0_ADDR         0x0044
#define IRQ_RES1_ADDR         0x0048
#define IRQ_RES2_ADDR         0x004c
#define IRQ_TR_MASK_ADDR      0x0050
#define IRQ_TS_MASK_ADDR      0x0054
#define IRQ_VM_MASK_ADDR      0x0058
#define IRQ_PD_MASK_ADDR      0x005c
#define IRQ_TR_STATUS_ADDR    0x0060
#define IRQ_TS_STATUS_ADDR    0x0064
#define IRQ_VM_STATUS_ADDR    0x0068
#define IRQ_PD_STATUS_ADDR    0x006c
#define IRQ_TR_RAW_ADDR       0x0070
#define IRQ_TS_RAW_ADDR       0x0074
#define IRQ_VM_RAW_ADDR       0x0078
#define IRQ_PD_RAW_ADDR       0x007c

#define TS_CMN_CLK_SYNTH_ADDR      0x0080
#define TS_CMN_SDIF_DISABLE_ADDR   0x0084
#define TS_CMN_SDIF_STATUS_ADDR    0x0088
#define PVT_SDIF_BUSY_BIT	BIT(0)
#define PVT_SDIF_LOCK_BIT	BIT(1)
#define TS_CMN_SDIF_ADDR           0x008c
#define TS_CMN_SDIF_HALT_ADDR      0x0090
#define TS_CMN_SDIF_CTRL_ADDR      0x0094
#define TS_CMN_SMPL_CTRL_ADDR      0x00a0
#define TS_SMPL_DISCARD_BIT      BIT(2)
#define TS_SMPL_CTR_HOLD_BIT     BIT(1)
#define TS_SMPL_CTR_DISABLE_BIT  BIT(0)
#define TS_CMN_SMPL_CLR_ADDR       0x00a4
#define TS_CMN_SMPL_CNT_ADDR       0x00a8


#define TS_n_IRQ_ENABLE_ADDR      0x00c0
#define TP_IRQ_EN_FAULT_BIT    BIT(0)
#define TP_IRQ_EN_DONE_BIT     BIT(1)
#define TP_IRQ_EN_ALARMA_BIT   BIT(3)
#define TP_IRQ_EN_ALARMB_BIT   BIT(4)

#define TS_n_IRQ_STATUS_ADDR      0x00c4
#define TP_IRQ_STS_FAULT_BIT   BIT(0)
#define TP_IRQ_STS_DONE_BIT    BIT(1)
#define TP_IRQ_STS_ALARMA_BIT  BIT(3)
#define TP_IRQ_ST_ALARMB_BIT   BIT(4)

#define TS_n_IRQ_CLEAR_ADDR       0x00c8
#define TP_IRQ_CLR_FAULT_BIT   BIT(0)
#define TP_IRQ_CLR_DONE_BIT    BIT(1)
#define TP_IRQ_CLR_ALARMA_BIT  BIT(3)
#define TP_IRQ_CLR_ALARMB_BIT  BIT(4)

#define TS_n_IRQ_CLEAR_ADDR       0x00c8
#define TS_n_IRQ_TEST_ADDR        0x00cc
#define TS_n_SDIF_RDATA_ADDR      0x00d0
#define TS_n_SDIF_DONE_ADDR       0x00d4
#define TS_n_SDIF_DATA_ADDR       0x00d8
#define TS_SDIF_DATA_FAULT_BIT    BIT(17)
#define TS_n_RES0_ADDR            0x00dc
#define TS_n_ALARMA_CFG_ADDR      0x00e0
#define TS_n_ALARMB_CFG_ADDR      0x00e4
#define TS_n_SMPL_HILO_ADDR       0x00e8
#define TS_n_HILO_RESET_ADDR      0x00ec

/*
#define TS_00_IRQ_ENABLE_ADDR      0x00c0
#define TS_00_IRQ_STATUS_ADDR      0x00c4
#define TS_00_IRQ_CLEAR_ADDR       0x00c8
#define TS_00_IRQ_TEST_ADDR        0x00cc
#define TS_00_SDIF_RDATA_ADDR      0x00d0
#define TS_00_SDIF_DONE_ADDR       0x00d4
#define TS_00_SDIF_DATA_ADDR       0x00d8
#define TS_00_RES0_ADDR            0x00dc
#define TS_00_ALARMA_CFG_ADDR      0x00e0
#define TS_00_ALARMB_CFG_ADDR      0x00e4
#define TS_00_SMPL_HILO_ADDR       0x00e8
#define TS_00_HILO_RESET_ADDR      0x00ec
#define TS_01_IRQ_ENABLE_ADDR      0x0100
#define TS_01_IRQ_STATUS_ADDR      0x0104
#define TS_01_IRQ_CLEAR_ADDR       0x0108
#define TS_01_IRQ_TEST_ADDR        0x010c
#define TS_01_SDIF_RDATA_ADDR      0x0110
#define TS_01_SDIF_DONE_ADDR       0x0114
#define TS_01_SDIF_DATA_ADDR       0x0118
#define TS_01_RES0_ADDR            0x011c
#define TS_01_ALARMA_CFG_ADDR      0x0120
#define TS_01_ALARMB_CFG_ADDR      0x0124
#define TS_01_SMPL_HILO_ADDR       0x0128
#define TS_01_HILO_RESET_ADDR      0x012c
#define TS_02_IRQ_ENABLE_ADDR      0x0140
#define TS_02_IRQ_STATUS_ADDR      0x0144
#define TS_02_IRQ_CLEAR_ADDR       0x0148
#define TS_02_IRQ_TEST_ADDR        0x014c
#define TS_02_SDIF_RDATA_ADDR      0x0150
#define TS_02_SDIF_DONE_ADDR       0x0154
#define TS_02_SDIF_DATA_ADDR       0x0158
#define TS_02_RES0_ADDR            0x015c
#define TS_02_ALARMA_CFG_ADDR      0x0160
#define TS_02_ALARMB_CFG_ADDR      0x0164
#define TS_02_SMPL_HILO_ADDR       0x0168
#define TS_02_HILO_RESET_ADDR      0x016c
#define TS_03_IRQ_ENABLE_ADDR      0x0180
#define TS_03_IRQ_STATUS_ADDR      0x0184
#define TS_03_IRQ_CLEAR_ADDR       0x0188
#define TS_03_IRQ_TEST_ADDR        0x018c
#define TS_03_SDIF_RDATA_ADDR      0x0190
#define TS_03_SDIF_DONE_ADDR       0x0194
#define TS_03_SDIF_DATA_ADDR       0x0198
#define TS_03_RES0_ADDR            0x019c
#define TS_03_ALARMA_CFG_ADDR      0x01a0
#define TS_03_ALARMB_CFG_ADDR      0x01a4
#define TS_03_SMPL_HILO_ADDR       0x01a8
#define TS_03_HILO_RESET_ADDR      0x01ac
*/

#define PD_CMN_CLK_SYNTH_ADDR      0x0200
#define PD_CMN_SDIF_DISABLE_ADDR   0x0204
#define PD_CMN_SDIF_STATUS_ADDR    0x0208
#define PD_CMN_SDIF_ADDR           0x020c
#define PD_CMN_SDIF_HALT_ADDR      0x0210
#define PD_CMN_SDIF_CTRL_ADDR      0x0214
#define PD_CMN_SMPL_CTRL_ADDR      0x0220
#define PD_CMN_SMPL_CLR_ADDR       0x0224
#define PD_CMN_SMPL_CNT_ADDR       0x0228

#define PD_n_IRQ_ENABLE_ADDR      0x0240
#define PD_n_IRQ_STATUS_ADDR      0x0244
#define PD_n_IRQ_CLEAR_ADDR       0x0248
#define PD_n_IRQ_TEST_ADDR        0x024c
#define PD_n_SDIF_RDATA_ADDR      0x0250
#define PD_n_SDIF_DONE_ADDR       0x0254
#define PD_n_SDIF_DATA_ADDR       0x0258
#define PD_n_RES0_ADDR            0x025c
#define PD_n_ALARMA_CFG_ADDR      0x0260
#define PD_n_ALARMB_CFG_ADDR      0x0264
#define PD_n_SMPL_HILO_ADDR       0x0268
#define PD_n_HILO_RESET_ADDR      0x026c
/*
#define PD_00_IRQ_ENABLE_ADDR      0x0240
#define PD_00_IRQ_STATUS_ADDR      0x0244
#define PD_00_IRQ_CLEAR_ADDR       0x0248
#define PD_00_IRQ_TEST_ADDR        0x024c
#define PD_00_SDIF_RDATA_ADDR      0x0250
#define PD_00_SDIF_DONE_ADDR       0x0254
#define PD_00_SDIF_DATA_ADDR       0x0258
#define PD_00_RES0_ADDR            0x025c
#define PD_00_ALARMA_CFG_ADDR      0x0260
#define PD_00_ALARMB_CFG_ADDR      0x0264
#define PD_00_SMPL_HILO_ADDR       0x0268
#define PD_00_HILO_RESET_ADDR      0x026c
#define PD_01_IRQ_ENABLE_ADDR      0x0280
#define PD_01_IRQ_STATUS_ADDR      0x0284
#define PD_01_IRQ_CLEAR_ADDR       0x0288
#define PD_01_IRQ_TEST_ADDR        0x028c
#define PD_01_SDIF_RDATA_ADDR      0x0290
#define PD_01_SDIF_DONE_ADDR       0x0294
#define PD_01_SDIF_DATA_ADDR       0x0298
#define PD_01_RES0_ADDR            0x029c
#define PD_01_ALARMA_CFG_ADDR      0x02a0
#define PD_01_ALARMB_CFG_ADDR      0x02a4
#define PD_01_SMPL_HILO_ADDR       0x02a8
#define PD_01_HILO_RESET_ADDR      0x02ac
#define PD_02_IRQ_ENABLE_ADDR      0x02c0
#define PD_02_IRQ_STATUS_ADDR      0x02c4
#define PD_02_IRQ_CLEAR_ADDR       0x02c8
#define PD_02_IRQ_TEST_ADDR        0x02cc
#define PD_02_SDIF_RDATA_ADDR      0x02d0
#define PD_02_SDIF_DONE_ADDR       0x02d4
#define PD_02_SDIF_DATA_ADDR       0x02d8
#define PD_02_RES0_ADDR            0x02dc
#define PD_02_ALARMA_CFG_ADDR      0x02e0
#define PD_02_ALARMB_CFG_ADDR      0x02e4
#define PD_02_SMPL_HILO_ADDR       0x02e8
#define PD_02_HILO_RESET_ADDR      0x02ec
*/

#define VM_CMN_CLK_SYNTH_ADDR          0x0400
#define VM_CMN_SDIF_DISABLE_ADDR       0x0404
#define VM_CMN_SDIF_STATUS_ADDR        0x0408
#define VM_CMN_SDIF_ADDR               0x040c
#define VM_CMN_SDIF_HALT_ADDR          0x0410
#define VM_CMN_SDIF_CTRL_ADDR          0x0414
#define VM_CMN_SMPL_CTRL_ADDR          0x0420
#define VM_CMN_SMPL_CLR_ADDR           0x0424
#define VM_CMN_SMPL_CNT_ADDR           0x0428

#define VM_IRQ_ENABLE_ADDR          0x0600
#define VM_IRQ_EN_FAULT_BIT         BIT(0)
#define VM_IRQ_EN_DONE_BIT          BIT(1)

#define VM_IRQ_STATUS_ADDR          0x0604
#define VM_IRQ_STS_FAULT_BIT        BIT(0)
#define VM_IRQ_STS_DONE_BIT         BIT(1)

#define VM_IRQ_CLEAR_ADDR           0x0608
#define VM_IRQ_TEST_ADDR            0x060c
#define VM_IRQ_ALARMA_ENABLE_ADDR   0x0610
#define VM_IRQ_ALARMA_STATUS_ADDR   0x0614
#define VM_IRQ_ALARMA_CLR_ADDR      0x0618
#define VM_IRQ_ALARMA_TEST_ADDR     0x061c
#define VM_IRQ_ALARMB_ENABLE_ADDR   0x0620
#define VM_IRQ_ALARMB_STATUS_ADDR   0x0624
#define VM_IRQ_ALARMB_CLR_ADDR      0x0628
#define VM_IRQ_ALARMB_TEST_ADDR     0x062c
#define VM_SDIF_RDATA_ADDR          0x0630
#define VM_SDIF_DONE_ADDR           0x0634

#define VM_CH_n_SDIF_DATA_ADDR     0x0640
#define VM_SDIF_DATA_FAULT_BIT     BIT(17)
/*
#define VM_00_CH_00_SDIF_DATA_ADDR     0x0640
#define VM_00_CH_01_SDIF_DATA_ADDR     0x0644
#define VM_00_CH_02_SDIF_DATA_ADDR     0x0648
#define VM_00_CH_03_SDIF_DATA_ADDR     0x064c
#define VM_00_CH_04_SDIF_DATA_ADDR     0x0650
#define VM_00_CH_05_SDIF_DATA_ADDR     0x0654
#define VM_00_CH_06_SDIF_DATA_ADDR     0x0658
#define VM_00_CH_07_SDIF_DATA_ADDR     0x065c
#define VM_00_CH_08_SDIF_DATA_ADDR     0x0660
#define VM_00_CH_09_SDIF_DATA_ADDR     0x0664
#define VM_00_CH_10_SDIF_DATA_ADDR     0x0668
#define VM_00_CH_11_SDIF_DATA_ADDR     0x066c
#define VM_00_CH_12_SDIF_DATA_ADDR     0x0670
#define VM_00_CH_13_SDIF_DATA_ADDR     0x0674
#define VM_00_CH_14_SDIF_DATA_ADDR     0x0678
#define VM_00_CH_15_SDIF_DATA_ADDR     0x067c
*/

#define VM_CH_n_ALARMA_CFG_ADDR    0x0680
#define VM_CH_n_ALARMB_CFG_ADDR    0x0684
#define VM_CH_n_SMPL_HILO_ADDR     0x0688
#define VM_CH_n_HILO_RESET_ADDR    0x068c


#define VM_00_CH_00_ALARMA_CFG_ADDR    0x0680
#define VM_00_CH_00_ALARMB_CFG_ADDR    0x0684
#define VM_00_CH_00_SMPL_HILO_ADDR     0x0688
#define VM_00_CH_00_HILO_RESET_ADDR    0x068c
#define VM_00_CH_01_ALARMA_CFG_ADDR    0x0690
#define VM_00_CH_01_ALARMB_CFG_ADDR    0x0694
#define VM_00_CH_01_SMPL_HILO_ADDR     0x0698
#define VM_00_CH_01_HILO_RESET_ADDR    0x069c
#define VM_00_CH_02_ALARMA_CFG_ADDR    0x06a0
#define VM_00_CH_02_ALARMB_CFG_ADDR    0x06a4
#define VM_00_CH_02_SMPL_HILO_ADDR     0x06a8
#define VM_00_CH_02_HILO_RESET_ADDR    0x06ac
#define VM_00_CH_03_ALARMA_CFG_ADDR    0x06b0
#define VM_00_CH_03_ALARMB_CFG_ADDR    0x06b4
#define VM_00_CH_03_SMPL_HILO_ADDR     0x06b8
#define VM_00_CH_03_HILO_RESET_ADDR    0x06bc
#define VM_00_CH_04_ALARMA_CFG_ADDR    0x06c0
#define VM_00_CH_04_ALARMB_CFG_ADDR    0x06c4
#define VM_00_CH_04_SMPL_HILO_ADDR     0x06c8
#define VM_00_CH_04_HILO_RESET_ADDR    0x06cc
#define VM_00_CH_05_ALARMA_CFG_ADDR    0x06d0
#define VM_00_CH_05_ALARMB_CFG_ADDR    0x06d4
#define VM_00_CH_05_SMPL_HILO_ADDR     0x06d8
#define VM_00_CH_05_HILO_RESET_ADDR    0x06dc
#define VM_00_CH_06_ALARMA_CFG_ADDR    0x06e0
#define VM_00_CH_06_ALARMB_CFG_ADDR    0x06e4
#define VM_00_CH_06_SMPL_HILO_ADDR     0x06e8
#define VM_00_CH_06_HILO_RESET_ADDR    0x06ec
#define VM_00_CH_07_ALARMA_CFG_ADDR    0x06f0
#define VM_00_CH_07_ALARMB_CFG_ADDR    0x06f4
#define VM_00_CH_07_SMPL_HILO_ADDR     0x06f8
#define VM_00_CH_07_HILO_RESET_ADDR    0x06fc
#define VM_00_CH_08_ALARMA_CFG_ADDR    0x0700
#define VM_00_CH_08_ALARMB_CFG_ADDR    0x0704
#define VM_00_CH_08_SMPL_HILO_ADDR     0x0708
#define VM_00_CH_08_HILO_RESET_ADDR    0x070c
#define VM_00_CH_09_ALARMA_CFG_ADDR    0x0710
#define VM_00_CH_09_ALARMB_CFG_ADDR    0x0714
#define VM_00_CH_09_SMPL_HILO_ADDR     0x0718
#define VM_00_CH_09_HILO_RESET_ADDR    0x071c
#define VM_00_CH_10_ALARMA_CFG_ADDR    0x0720
#define VM_00_CH_10_ALARMB_CFG_ADDR    0x0724
#define VM_00_CH_10_SMPL_HILO_ADDR     0x0728
#define VM_00_CH_10_HILO_RESET_ADDR    0x072c
#define VM_00_CH_11_ALARMA_CFG_ADDR    0x0730
#define VM_00_CH_11_ALARMB_CFG_ADDR    0x0734
#define VM_00_CH_11_SMPL_HILO_ADDR     0x0738
#define VM_00_CH_11_HILO_RESET_ADDR    0x073c
#define VM_00_CH_12_ALARMA_CFG_ADDR    0x0740
#define VM_00_CH_12_ALARMB_CFG_ADDR    0x0744
#define VM_00_CH_12_SMPL_HILO_ADDR     0x0748
#define VM_00_CH_12_HILO_RESET_ADDR    0x074c
#define VM_00_CH_13_ALARMA_CFG_ADDR    0x0750
#define VM_00_CH_13_ALARMB_CFG_ADDR    0x0754
#define VM_00_CH_13_SMPL_HILO_ADDR     0x0758
#define VM_00_CH_13_HILO_RESET_ADDR    0x075c
#define VM_00_CH_14_ALARMA_CFG_ADDR    0x0760
#define VM_00_CH_14_ALARMB_CFG_ADDR    0x0764
#define VM_00_CH_14_SMPL_HILO_ADDR     0x0768
#define VM_00_CH_14_HILO_RESET_ADDR    0x076c
#define VM_00_CH_15_ALARMA_CFG_ADDR    0x0770
#define VM_00_CH_15_ALARMB_CFG_ADDR    0x0774
#define VM_00_CH_15_SMPL_HILO_ADDR     0x0778
#define VM_00_CH_15_HILO_RESET_ADDR    0x077c

#define HOBOT_EFUSE_BASE		0xA6008000
#define HOBOT_SEC_EFUSE_BASE	0xA6008100
#define PVT_TS_A_MASK			0xFFFF0000
#define PVT_TS_B_MASK			0x0000FFFF
#define PVT_VM_K3_MASK          0xFFFF0000
#define PVT_VM_N0_MASK          0x00000FFF

extern int hobot_vm_probe(struct device *dev, void __iomem *reg_base,
			void __iomem *efuse_base);
#ifdef CONFIG_PM
extern int hobot_vm_resume(struct device *dev);
#endif

#endif /* __HOBOT_PVT_H__ */
