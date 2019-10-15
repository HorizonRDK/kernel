/*****************************************************************************/
/*                                                                           */
/*                     HozironRobtics MMC MAC SOFTWARE                       */
/*                                                                           */
/*                  Horizon Robtics SYSTEMS LTD                              */
/*                           COPYRIGHT(C) 2018                               */
/*                                                                           */
/*  This program  is  proprietary to  Ittiam  Systems  Private  Limited  and */
/*  is protected under china Copyright Law as an unpublished work. Its use   */
/*  and  disclosure  is  limited by  the terms  and  conditions of a license */
/*  agreement. It may not be copied or otherwise  reproduced or disclosed to */
/*  persons outside the licensee's organization except in accordance with the*/
/*  terms  and  conditions   of  such  an  agreement.  All  copies  and      */
/*  reproductions shall be the property of HorizonRobtics Systems Private    */
/*    Limited and must bear this notice in its entirety.                     */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name         : x2_bifsd.h                                        */
/*                                                                           */
/*  Description       : This file contains all the declarations related to   */
/*                      MMC host interface.                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         10 24 2018   shaochuan.zhang@horizon.ai  Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifndef X2_BIFSD_DEV_H
#define X2_BIFSD_DEV_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/of.h>
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#ifdef CONFIG_X2A_FPGA
#define HUGO_PLM
#endif
/* MMC AHB related Registers */
#define EMMC_AHB_BASE    0xA1007000

#define EMMC_PROGRAM_REG          (0x00)
#define EMMC_OCR                  (0x04)
#define EMMC_CSD                  (0x08)
#define EMMC_CID                  (0x18)
#define EMMC_CARD_STATE           (0x28)
#define EMMC_SCR                  (0x2C)
#define EMMC_OUT_RANGE_ADDR       (0x34)
#define EMMC_INT_ENABLE_1         (0x38)
#define EMMC_INT_STATUS_1         (0x3C)
#define EMMC_MEM_MGMT             (0x40)
#define EMMC_ARGUMENT_REG         (0x44)
#define EMMC_DMA_ADDR             (0x48)
#define EMMC_BLOCK_CNT            (0x4C)
#define EMMC_PASSWORD             (0x50)
#define EMMC_STATUS               (0x60)
#define EMMC_BLOCK_LEN            (0x70)
#define EMMC_ERASE_BLOCK_CNT      (0x74)
#define EMMC_ERASE_START_ADDR     (0x78)
#define EMMC_ERASE_END_ADDR       (0x7C)
#define EMMC_SET_WRITE_PROTECT    (0x80)
#define EMMC_CLEAR_WRITE_PROTECT  (0x84)
#define EMMC_HARD_RESET_CNT       (0x8C)
#define EMMC_DMA_CNT              (0x90)
#define EMMC_UPDATE_EXT_CSD       (0x94)
#define EMMC_BOOT_BLOCK_CNT       (0x98)
#define EMMC_INT_ENABLE_2         (0x9C)
#define EMMC_INT_STATUS_2         (0xA0)
#define EMMC_EXTENDED_CSD         (0xA4)
#define EMMC_PASSWORD_LEN         (0x240)
#define EMMC_POWER_UP             (0x244)
#define EMMC_PKT_CNT              (0x254)
#define EMMC_TIMING               (0x258)
#define EMMC_QUEUE_STATUS         (0x268)
#define EMMC_FIFO_READ            (0x26C)

#define EMMC_SECURITY_INT_ENABLE  (0x248)
#define EMMC_SECURITY_INT_STATUS  (0x24C)
#define EMMC_BLOCK_COUNT_SECURITY (0x250)
#define EMMC_ACC_CONFIG           (0x590)

#define OUT_RANGE_ADDR 0xFFFFFFFF
#define EMMC_BLK_CNT 32
#define EMMC_SECURITY_INT_ENABLE_VAL 0x00000FFF

/* Interrupt status register_1 */
#define MMC_CSD_UPDATE			(1<<0)
#define MMC_CID_UPDATE			(1<<1)
#define MMC_SET_BLOCK_CNT		(1<<2)
#define MMC_STOP_CMD			(1<<3)
#define MMC_IDLE_CMD			(1<<4)
#define MMC_INACTIVE_CMD		(1<<5)
#define MMC_SET_BLOCK_LEN		(1<<6)
#define MMC_CMD6_ALWAYS			(1<<7)
#define MMC_CMD_61      		(1<<8)
#define MMC_CMD_40      		(1<<9)
#define MMC_BLK_READ			(1<<10)
#define MMC_BLK_WRITE			(1<<11)
#define MMC_WRITE_BLOCK_CNT		(1<<12)
#define MMC_READ_BLOCK_CNT		(1<<13)
#define MMC_VOLTAGE_SWITCH		(1<<14)
#define MMC_SPEED_CLASS_CTL		(1<<15)
#define MMC_CMD55			(1<<16)
#define MMC_NUM_WELL_WRITE_BLOCK	(1<<17)
#define MMC_START_ADDR_ERASE		(1<<18)
#define MMC_END_ADDR_ERASE		(1<<19)
#define MMC_ERASE_CMD			(1<<20)
#define MMC_FORCE_ERASE			(1<<21)
#define MMC_SET_PASSWD			(1<<22)
#define MMC_CLEAR_PASSWD		(1<<23)
#define MMC_LOCK_CARD			(1<<24)
#define MMC_UPLOCK_CARD			(1<<25)
#define MMC_WRITE_PROTECT		(1<<26)
#define MMC_CLEAR_PROTECT		(1<<27)
#define MMC_WRITE_PROT_STATUS		(1<<28)
#define MMC_GENERAL_READ_WRITE		(1<<29)
#define MMC_BLOCK_COUNT_CLEAR		(1<<30)
#define MMC_MULTI_BLOCK_READ_WRITE	(1<<31)

#define MMC_BOOT_START			(1<<0)
#define MMC_BOOT_STOP			(1<<1)
#define MMC_SLEEP_CMD			(1<<2)
#define MMC_AWAKE_CMD			(1<<3)
#define MMC_BKOPS_START			(1<<4)
#define MMC_HIGH_PRIORITY		(1<<5)
#define MMC_DATA_CRC_ERR		(1<<6)
#define MMC_CMD_CRC_ERR			(1<<7)
#define MMC_CMD12			(1<<8)
#define MMC_CARD_SELECT			(1<<9)
#define MMC_CARD_DESELECT		(1<<10)
#define MMC_SANITIZE_START		(1<<11)
#define MMC_flUSH_CACHE			(1<<12)
#define MMC_TCASE_SUPPORT		(1<<13)
#define MMC_PACKED_CMD			(1<<14)
#define MMC_REAL_TIME_CLK		(1<<15)
#define MMC_GET_WRITE_PROT		(1<<16)
#define MMC_PARTITION_SETTING_COMPLETED	(1<<17)
#define MMC_UPDATE_EXT_CSD		(1<<18)
#define MMC_HARDWARE_RESET		(1<<19)
#define MMC_GO_PRE_IDLE			(1<<20)
#define MMC_PACKED_COMPLETION		(1<<21)
#define MMC_PACKED_FAILURE		(1<<22)
#define MMC_CARD_STATUS_CHANGE		(1<<23)
#define MMC_SECURITY_PROTOCOL_READ	(1<<24)
#define MMC_SECURITY_PROTOCOL_WRITE	(1<<25)
#define MMC_PWD_CMD_FAIL		(1<<26)
#define MMC_CMD1			(1<<27)
#define MMC_CMD48			(1<<28)
#define MMC_CMD47			(1<<29)
#define MMC_CMD46			(1<<30)
#define MMC_FIFO_NOT_EMPTY		(1<<31)

#define NAC_NON_HS_200_400_VAL (2)
#define NAC_HS_200_400_VAL (5)
#define NCR_VALUE (0)
#define NCRC_VALUE (3)

#define MMC_CPU_CLOCK 40000000
#define HR_TIMER_BASE 0xA1002000

/* Events from the bifsd core */
#define BIFSD_DATA_RX_COMP		0x0001

/* Register access macros */
#define sd_readl(dev, reg, offset)                     \
	readl_relaxed((dev)->regs + EMMC_##reg + offset)
#define sd_writel(dev, reg, value, offset)                             \
	writel_relaxed((value), (dev)->regs + EMMC_##reg + offset)

#define WAKE_RX_WORK(rx_info) \
	do { \
		queue_work((sd)->bifsd_rxwq, &sd->BifsdRxWork); \
	} while (0)

#define MMC_MODE mmc_card_5_0

//#define BIFSD_TX_RX_TEST_MODE
/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
typedef enum {
	MEDIA_CONNECTED = 0x1,	/* First chunk */
	MEDIA_DISCONNECTED = 0x2	/* Last chunk  */
} ASYNC_EVENT;

typedef enum {
	sd_card,
	sdhc_card,
	sdxc_card,
	mmc_card_4_2,
	mmc_card_4_4,
	mmc_card_4_5,
	mmc_card_5_0
} card_type_t;

enum bifsd_state {
	STATE_IDLE = 0,
	STATE_TX_DATA_COMP,
	STATE_RX_DATA_COMP
};

/*****************************************************************************/
/* Structures                                                                */
/******************************************************************************/
struct bifsd_req {
	struct list_head list;
	struct bifsd *sd;
	void *buf;
	u32 blk_cnt;
	u32 len;
};

struct bif_sd {
	spinlock_t lock;
	spinlock_t irq_lock;
	u32 cd_gpio;
	void __iomem *regs;
	void __iomem *sysctrl_reg;
	struct reset_control *rst;
	void __iomem *fifo_reg;
	void __iomem *paddr;
	void __iomem *vaddr;
	struct tasklet_struct tasklet;
	/* Registers's physical base address */
	resource_size_t phy_regs;
	const struct bifsd_drv_data *drv_data;
	struct device *dev;
	unsigned long irq_flags;	/* IRQ flags */
	int irq;
	enum bifsd_state state;
	u32 block_len;
	u32 block_cnt;
	u32 read_blk_num;   /* the number of block data that send to master */
	u32 rx_blk_num;
	u32 rd_buf;
	u32 wr_buf;
	u32 cmd_argu;
	u32 range_addr_max; /* access max addr */

	struct bifsd_req *bifsd_rx_reqs;
	struct list_head bifsd_rx_freeq;
	int bifsd_rx_freecount;
	spinlock_t bifsd_rx_freeq_lock;
	struct list_head bifsd_rx_postq;
	int bifsd_rx_postcount;
	spinlock_t bifsd_rx_postq_lock;

	struct workqueue_struct *bifsd_rxwq;
	struct work_struct BifsdRxWork;
	atomic_t bifsd_rx_dpc_tskcnt;
	spinlock_t bifsd_rxqlock;
};

struct bifsd_drv_data {
	int (*init) (struct bif_sd * sd);
};

/*****************************************************************************/
/* Extern variable declarations                                              */
/*****************************************************************************/

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void bifsd_register_notify(struct notifier_block *nb);
extern void bifsd_unregister_notify(struct notifier_block *nb);
extern void bifsd_notify_rx_data(void *data);
/*****************************************************************************/
/* INLINE Functions                                                          */
/*****************************************************************************/

/* This function config power up register */

static inline void mmc_set_power_up(struct bif_sd *sd)
{
	card_type_t mmc_type = MMC_MODE;
	u32 reg_val = 0;

	if ((mmc_type == sd_card) || (mmc_type == sdhc_card)
	    || (mmc_type == sdxc_card)) {
		reg_val |= BIT(0);
	} else {
		reg_val |= BIT(2);
	}
	sd_writel(sd, POWER_UP, reg_val, 0);
}

/* This function Set hardware reset count
   that the number of AHB clock cycles equivalent to 1Microseconds */

static inline void mmc_set_hard_reset_cnt(struct bif_sd *sd)
{
	u32 pwr_cnt_val = 0x1F4;

	sd_writel(sd, HARD_RESET_CNT, pwr_cnt_val, 0);
}

/*OCR value - 0xff8000(standard capacity card);
  0x40ff8000(high capacity card);
  0xff8080(dual voltage card);
  0x41ff8000(Extended capacity card)*/
static inline void mmc_config_ocr_reg(struct bif_sd *sd)
{
	card_type_t mmc_type = MMC_MODE;
	u32 reg_val = 0;
	if ((mmc_type == sd_card) || (mmc_type == sdhc_card)
	    || (mmc_type == sdxc_card)) {
		reg_val = 0x40ff8000;
	} else {
		reg_val = 0x40000080;
	}

	sd_writel(sd, OCR, reg_val, 0);
}

static inline void card_power_up(struct bif_sd *sd)
{
	u32 reg_val = 0;

	reg_val = sd_readl(sd, OCR, 0);
	reg_val |= BIT(31);
	sd_writel(sd, OCR, reg_val, 0);
}

static inline void bifsd_config_timing(struct bif_sd *sd)
{
	u32 reg_val;

	reg_val =
	    (NAC_NON_HS_200_400_VAL | (NAC_HS_200_400_VAL << 4) |
	     (NCR_VALUE << 8) | (NCRC_VALUE << 16));
	sd_writel(sd, TIMING, reg_val, 0);
}

static inline void mmc_set_out_range_addr(struct bif_sd *sd)
{
	sd_writel(sd, OUT_RANGE_ADDR, sd->range_addr_max, 0);
}

static inline void mmc_disable_acc_bypass(struct bif_sd *sd)
{
	u32 reg_val;
	reg_val = readl(sd->sysctrl_reg);
	reg_val |= 0x07;
	reg_val &= 0xFFFFFFFE;
	writel(reg_val, sd->sysctrl_reg);
}

static inline void mmc_enable_acc_bypass(struct bif_sd *sd)
{
	writel(0x01, sd->sysctrl_reg);
}
#endif /* X2_BIFSD_DEV_H */
