/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/
#ifndef _HOBOT_ISP_REG_DMA_REGSET_H_
#define _HOBOT_ISP_REG_DMA_REGSET_H_

#include "system_hw_io.h"

#define BIT0        (0x1)
#define BIT1        (0x2)
#define BIT2        (0x4)
#define BIT3        (0x8)
#define BIT4        (0x10)
#define BIT5        (0x20)
#define BIT6        (0x40)
#define BIT7        (0x80)
#define BIT8        (0x100)
#define BIT9        (0x200)
#define BIT10       (0x400)
#define BIT11       (0x800)
#define BIT12       (0x1000)
#define BIT13       (0x2000)
#define BIT15       (0x8000)
#define BIT14       (0x4000)
#define BIT16       (0x10000)
#define BIT17       (0x20000)
#define BIT18       (0x40000)
#define BIT19       (0x80000)
#define BIT20       (0x100000)
#define BIT21       (0x200000)
#define BIT22       (0x400000)
#define BIT23       (0x800000)
#define BIT24       (0x1000000)
#define BIT25       (0x2000000)
#define BIT26       (0x4000000)
#define BIT28       (0x8000000)
#define BIT27       (0x10000000)
#define BIT29       (0x20000000)
#define BIT30       (0x40000000)
#define BIT31       (0x80000000)

#define IMI_RAM_BASE_ADDR       0x80400000
#define IMI_RAM_SIZE            0x20000
#define IMI_RAM_NUM             0x4
#define IMI_RAM_TOTAL_SIZE      (IMI_RAM_SIZE*IMI_RAM_NUM)

#define DMAC_ISP_BASE_ADDR      0xA100D000
#define DMAC_ISP_REG_SIZE       0x1000

//// reg offset ////
#define DMA_CONTROL_REG         0x00
#define DMA_TRAN_SIZE           0x04
#define DMA_RECV_SIZE           0x08
#define DMA_LOAD_SRAM_ADDR      0x0C
#define DMA_LOAD_AHB_ADDR       0x10
#define DMA_START_DMA_CTRL      0x14
#define DMA_CONTROL_STATUS      0x18
#define DMA_INT_CTRL            0x1C
#define DMA_ARBITER_CTRL        0x20
#define DMA_SRAM_CTRL           0x24
#define DMA_CONTROL2_REG        0x28
#define DMA_CONTROL3_REG        0x2C
#define DMA_SW_RST_REG          0x30

#define DMA_CMD_QUEUE_CTRL      0x34
#define DMA_TRAN_SIZE_CH1       0x38
#define DMA_RECV_SIZE_CH1       0x3C
#define DMA_LOAD_SRAM_ADDR_CH1  0x40
#define DMA_LOAD_AHB_ADDR_CH1   0x44
#define DMA_TRAN_SIZE_CH2       0x48
#define DMA_RECV_SIZE_CH2       0x4C
#define DMA_LOAD_SRAM_ADDR_CH2  0x50
#define DMA_LOAD_AHB_ADDR_CH2   0x54
#define DMA_TRAN_SIZE_CH3       0x58
#define DMA_RECV_SIZE_CH3       0x5C
#define DMA_LOAD_SRAM_ADDR_CH3  0x60
#define DMA_LOAD_AHB_ADDR_CH3   0x64

#define dma_isp_read_32         system_dmac_isp_hw_read_32
#define dma_isp_write_32        system_dmac_isp_hw_write_32

/// reg content ///
/// DMA_CONTROL_REG ///
static __inline void dma_isp_reg_mastlock_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_CONTROL_REG);
    dma_isp_write_32(DMA_CONTROL_REG, (((data & BIT0)) << 3) | (curr & ~BIT3));
}
static __inline uint32_t dma_isp_reg_mastlock_read(void) {
    return ((dma_isp_read_32(DMA_CONTROL_REG) & BIT3) >> 3);
}

#define DMA_TRANDFER_MEM_TO_ISP     BIT0
#define DMA_TRANDFER_ISP_TO_MEM     BIT1
static __inline void dma_isp_reg_dma_mode_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_CONTROL_REG);
    dma_isp_write_32(DMA_CONTROL_REG, ((data & (BIT0|BIT1|BIT2)) | (curr & ~(BIT0|BIT1|BIT2))));
}
static __inline uint32_t dma_isp_reg_dma_mode_read(void) {
    return (dma_isp_read_32(DMA_CONTROL_REG) & (BIT0|BIT1|BIT2));
}

/// DMA_TRAN_SIZE ///
static __inline void dma_isp_reg_dma_tran_size_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_TRAN_SIZE);
    dma_isp_write_32(DMA_TRAN_SIZE, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_dma_tran_size_read(void) {
    return (dma_isp_read_32(DMA_TRAN_SIZE) & (0x7FFF));
}

/// DMA_RECV_SIZE       ///
static __inline void dma_isp_reg_dma_recv_size_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_RECV_SIZE);
    dma_isp_write_32(DMA_RECV_SIZE, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_dma_recv_size_read(void) {
    return (dma_isp_read_32(DMA_RECV_SIZE) & (0x7FFF));
}

/// DMA_LOAD_SRAM_ADDR  ///
static __inline void dma_isp_reg_load_sram_addr_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_LOAD_SRAM_ADDR);
    dma_isp_write_32(DMA_LOAD_SRAM_ADDR, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_load_sram_addr_read(void) {
    return (dma_isp_read_32(DMA_LOAD_SRAM_ADDR) & (0x7FFF));
}

/// DMA_LOAD_AHB_ADDR   ///
static __inline void dma_isp_reg_load_ahb_addr_write(uint32_t data) {
    dma_isp_write_32(DMA_LOAD_AHB_ADDR, data);
}
static __inline uint32_t dma_isp_reg_load_ahb_addr_read(void) {
    return (dma_isp_read_32(DMA_LOAD_AHB_ADDR));
}

/// DMA_START_DMA_CTRL  ///
static __inline void dma_isp_reg_start_dma_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_START_DMA_CTRL);
    dma_isp_write_32(DMA_START_DMA_CTRL, ((data & (BIT0)) | (curr & ~(BIT0))));
}
static __inline uint32_t dma_isp_reg_start_dma_read(void) {
    return (dma_isp_read_32(DMA_START_DMA_CTRL) & (BIT0));
}

/// DMA_CONTROL_STATUS  ///
static __inline uint32_t dma_isp_dma_int_read(void) {
    return (dma_isp_read_32(DMA_CONTROL_STATUS) & (BIT0));
}

/// DMA_INT_CTRL        ///
#define DMA_INT_ENABLE              0
#define DMA_INT_DISABLE             1
static __inline void dma_isp_reg_mask_int_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_INT_CTRL);
    dma_isp_write_32(DMA_INT_CTRL, (((data & (BIT0))<<1) | (curr & ~(BIT1))));
}
static __inline uint32_t dma_isp_reg_mask_int_read(void) {
    return ((dma_isp_read_32(DMA_INT_CTRL) & BIT1)>>1);
}

static __inline void dma_isp_reg_clr_int_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_INT_CTRL);
    dma_isp_write_32(DMA_INT_CTRL, ((data & (BIT0)) | (curr & ~(BIT0))));
}
static __inline uint32_t dma_isp_reg_clr_int_read(void) {
    return (dma_isp_read_32(DMA_INT_CTRL) & (BIT0));
}

/// DMA_ARBITER_CTRL    ///
static __inline void dma_isp_reg_arbiter_bypass_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_ARBITER_CTRL);
    dma_isp_write_32(DMA_ARBITER_CTRL, ((data & (BIT0)) | (curr & ~(BIT0))));
}
static __inline uint32_t dma_isp_reg_arbiter_bypass_read(void) {
    return (dma_isp_read_32(DMA_ARBITER_CTRL) & (BIT0));
}

/// DMA_SRAM_CTRL       ///
static __inline void dma_isp_reg_dma_sram_ch_en_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_SRAM_CTRL);
    dma_isp_write_32(DMA_SRAM_CTRL, (((data & BIT0)) << 2) | (curr & ~BIT2));
}
static __inline uint32_t dma_isp_reg_dma_sram_ch_en_read(void) {
    return ((dma_isp_read_32(DMA_SRAM_CTRL) & BIT2) >> 2);
}

#define DMA_SRAM_CH0     0
#define DMA_SRAM_CH1     1
#define DMA_SRAM_CH2     2
#define DMA_SRAM_CH3     3
static __inline void dma_isp_reg_dma_sram_ch_sel_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_SRAM_CTRL);
    dma_isp_write_32(DMA_SRAM_CTRL, ((data & (BIT0|BIT1)) | (curr & ~(BIT0|BIT1))));
}
static __inline uint32_t dma_isp_reg_dma_sram_ch_sel_read(void) {
    return (dma_isp_read_32(DMA_SRAM_CTRL) & (BIT0|BIT1));
}

/// DMA_CONTROL2_REG    ///


/// DMA_CONTROL3_REG    ///


/// DMA_SW_RST_REG      ///
static __inline void dma_isp_reg_dma_sw_rst_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_SW_RST_REG);
    dma_isp_write_32(DMA_SW_RST_REG, ((data & (BIT0)) | (curr & ~(BIT0))));
}
static __inline uint32_t dma_isp_reg_dma_sw_rst_read(void) {
    return (dma_isp_read_32(DMA_SW_RST_REG) & (BIT0));
}


///	DMA_CMD_QUEUE_CTRL  ///
#define DMA_CH0     BIT0
#define DMA_CH1     BIT1
#define DMA_CH2     BIT2
#define DMA_CH3     BIT3
static __inline void dma_isp_reg_cmd_q_func_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_CMD_QUEUE_CTRL);
    dma_isp_write_32(DMA_CMD_QUEUE_CTRL, (((data & 0xff)) << 8) | (curr & ~(0xff00)));
}
static __inline uint32_t dma_isp_reg_cmd_q_func_read(void) {
    return ((dma_isp_read_32(DMA_CMD_QUEUE_CTRL) & 0xff) >> 8);
}
static __inline void dma_isp_reg_cmd_q_set_write(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_CMD_QUEUE_CTRL);
    dma_isp_write_32(DMA_CMD_QUEUE_CTRL, ((data & 0xff) | (curr & ~(0xff))));
}
static __inline uint32_t dma_isp_reg_cmd_q_set_read(void) {
    return (dma_isp_read_32(DMA_CMD_QUEUE_CTRL) & (0xff));
}

///////////////////////////////////////
///	DMA_TRAN_SIZE_CH1   ///
static __inline void dma_isp_reg_dma_tran_size_write_ch1(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_TRAN_SIZE_CH1);
    dma_isp_write_32(DMA_TRAN_SIZE_CH1, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_dma_tran_size_read_ch1(void) {
    return (dma_isp_read_32(DMA_TRAN_SIZE_CH1) & (0x7FFF));
}

///	DMA_RECV_SIZE_CH1   ///
static __inline void dma_isp_reg_dma_recv_size_write_ch1(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_RECV_SIZE_CH1);
    dma_isp_write_32(DMA_RECV_SIZE_CH1, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_dma_recv_size_read_ch1(void) {
    return (dma_isp_read_32(DMA_RECV_SIZE_CH1) & (0x7FFF));
}

///	DMA_LOAD_SRAM_ADDR_CH1  ///
static __inline void dma_isp_reg_load_sram_addr_write_ch1(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_LOAD_SRAM_ADDR_CH1);
    dma_isp_write_32(DMA_LOAD_SRAM_ADDR_CH1, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_load_sram_addr_read_ch1(void) {
    return (dma_isp_read_32(DMA_LOAD_SRAM_ADDR_CH1) & (0x7FFF));
}

///	DMA_LOAD_AHB_ADDR_CH1 ///
static __inline void dma_isp_reg_load_ahb_addr_write_ch1(uint32_t data) {
    dma_isp_write_32(DMA_LOAD_AHB_ADDR_CH1, data);
}
static __inline uint32_t dma_isp_reg_load_ahb_addr_read_ch1(void) {
    return (dma_isp_read_32(DMA_LOAD_AHB_ADDR_CH1));
}

///////////////////////////////////////
///	DMA_TRAN_SIZE_CH2   ///
static __inline void dma_isp_reg_dma_tran_size_write_ch2(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_TRAN_SIZE_CH2);
    dma_isp_write_32(DMA_TRAN_SIZE_CH2, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_dma_tran_size_read_ch2(void) {
    return (dma_isp_read_32(DMA_TRAN_SIZE_CH2) & (0x7FFF));
}

///	DMA_RECV_SIZE_CH2   ///
static __inline void dma_isp_reg_dma_recv_size_write_ch2(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_RECV_SIZE_CH2);
    dma_isp_write_32(DMA_RECV_SIZE_CH2, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_dma_recv_size_read_ch2(void) {
    return (dma_isp_read_32(DMA_RECV_SIZE_CH2) & (0x7FFF));
}

///	DMA_LOAD_SRAM_ADDR_CH2  ///
static __inline void dma_isp_reg_load_sram_addr_write_ch2(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_LOAD_SRAM_ADDR_CH2);
    dma_isp_write_32(DMA_LOAD_SRAM_ADDR_CH2, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_load_sram_addr_read_ch2(void) {
    return (dma_isp_read_32(DMA_LOAD_SRAM_ADDR_CH2) & (0x7FFF));
}

///	DMA_LOAD_AHB_ADDR_CH2   ///
static __inline void dma_isp_reg_load_ahb_addr_write_ch2(uint32_t data) {
    dma_isp_write_32(DMA_LOAD_AHB_ADDR_CH2, data);
}
static __inline uint32_t dma_isp_reg_load_ahb_addr_read_ch2(void) {
    return (dma_isp_read_32(DMA_LOAD_AHB_ADDR_CH2));
}

///////////////////////////////////////
///	DMA_TRAN_SIZE_CH3   ///
static __inline void dma_isp_reg_dma_tran_size_write_ch3(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_TRAN_SIZE_CH3);
    dma_isp_write_32(DMA_TRAN_SIZE_CH3, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_dma_tran_size_read_ch3(void) {
    return (dma_isp_read_32(DMA_TRAN_SIZE_CH3) & (0x7FFF));
}

///	DMA_RECV_SIZE_CH3   ///
static __inline void dma_isp_reg_dma_recv_size_write_ch3(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_RECV_SIZE_CH3);
    dma_isp_write_32(DMA_RECV_SIZE_CH3, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_dma_recv_size_read_ch3(void) {
    return (dma_isp_read_32(DMA_RECV_SIZE_CH3) & (0x7FFF));
}

///	DMA_LOAD_SRAM_ADDR_CH3   ///
static __inline void dma_isp_reg_load_sram_addr_write_ch3(uint32_t data) {
    uint32_t curr = dma_isp_read_32(DMA_LOAD_SRAM_ADDR_CH3);
    dma_isp_write_32(DMA_LOAD_SRAM_ADDR_CH3, ((data & (0x7FFF)) | (curr & ~(0x7FFF))));
}
static __inline uint32_t dma_isp_reg_load_sram_addr_read_ch3(void) {
    return (dma_isp_read_32(DMA_LOAD_SRAM_ADDR_CH3) & (0x7FFF));
}

///	DMA_LOAD_AHB_ADDR_CH3   ///
static __inline void dma_isp_reg_load_ahb_addr_write_ch3(uint32_t data) {
    dma_isp_write_32(DMA_LOAD_AHB_ADDR_CH3, data);
}
static __inline uint32_t dma_isp_reg_load_ahb_addr_read_ch3(void) {
    return (dma_isp_read_32(DMA_LOAD_AHB_ADDR_CH3));
}

static inline int hobot_idma_is_busy(void)
{
    int ret = 0;

    ret = (dma_isp_reg_start_dma_read() == 1) && (dma_isp_dma_int_read() == 0);

    return ret;
}

#endif      /* _HOBOT_ISP_REG_DMA_REGSET_H_ */

