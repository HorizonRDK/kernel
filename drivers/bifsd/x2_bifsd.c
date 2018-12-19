/*
 * (C) Copyright 2015 - 2018 HobotRobotics, Inc.
 *
 * HobotRobotics SD Controller Interface
 *
 */
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/reset.h>
#include <linux/gpio.h>
#include "x2_bifsd.h"
#include <linux/slab.h>

#define BIFSD_DETECT_GPIO 77
/*****************************************************************************/
/* static Global Variables                                                   */
/*****************************************************************************/
struct bif_sd *bifsd_info;
/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

void set_sd_info(struct bif_sd *sd)
{
    bifsd_info = sd;
}
struct bif_sd *get_sd_info(void)
{
    return bifsd_info;
}
static struct bifsd_req *bifsd_deq(struct bif_sd *sd,
	struct list_head *q, int *counter, spinlock_t *qlock)
{
	struct bifsd_req *req;

	spin_lock_bh(qlock);
	if (list_empty(q)) {
		spin_unlock_bh(qlock);
		return NULL;
	}
	req = list_entry(q->next, struct bifsd_req, list);
	list_del_init(q->next);
	if (counter)
		(*counter)--;
	spin_unlock_bh(qlock);

	return req;
}
static void bifsd_enq(struct bif_sd *sd,
	struct list_head *q, struct bifsd_req *req, int *counter,
	spinlock_t *qlock)
{
	spin_lock_bh(qlock);
	list_add_tail(&req->list, q);
	if (counter)
		(*counter)++;
	spin_unlock_bh(qlock);
}
static struct bifsd_req *bifsd_qinit(struct bif_sd *sd, struct list_head *q, int qsize)
{
	int i;
	struct bifsd_req *req, *reqs;

	reqs = kcalloc(qsize, sizeof(struct bifsd_req), GFP_ATOMIC);

	if (reqs == NULL)
		return NULL;

	req = reqs;

	for (i = 0; i < qsize; i++) {
        req->buf = 0;
        req->len = 0;
        req->blk_cnt = 0;
		INIT_LIST_HEAD(&req->list);
		list_add_tail(&req->list, q);
		req++;
	}
	return reqs;
}
static void bifsd_free_q(struct bif_sd *sd, struct list_head *q, bool pending, spinlock_t *qlock)
{
	struct bifsd_req *req, *next;
	int i = 0;

	spin_lock_bh(qlock);
	list_for_each_entry_safe(req, next, q, list) {
        spin_unlock_bh(qlock);
        i++;
        list_del_init(&req->list);
        spin_lock_bh(qlock);
	}
	spin_unlock_bh(qlock);
}
static int bifsd_process_rx_data(struct bif_sd *sd)
{
    struct bifsd_req *req;
    req = bifsd_deq(sd, &sd->bifsd_rx_postq,
                 &sd->bifsd_rx_postcount, &sd->bifsd_rx_postq_lock);
    if (!req) {
        dev_err(sd->dev, "no req to send\n");
        dev_err(sd->dev, "free:%d, post:%d\n", sd->bifsd_rx_freecount,
                sd->bifsd_rx_postcount);
    }
    atomic_dec(&sd->bifsd_rx_dpc_tskcnt);
    /* wait transfer */
    /*#########################*/
    sd->rd_buf = (u64)req->buf;
    sd->block_cnt = req->blk_cnt;

    req->buf = 0;
    req->blk_cnt = 0;

    bifsd_enq(sd, &sd->bifsd_rx_freeq, req,
            &sd->bifsd_rx_freecount, &sd->bifsd_rx_freeq_lock);
    /* set gpio for triger interrupt to host */

    /*############################*/
    return 0;
}
static void bifsd_rx_dpc(struct bif_sd *sd)
{
	bifsd_process_rx_data(sd);
}
static void bifsd_RxWorker(struct work_struct *work)
{
	struct bif_sd *sd = container_of(work, struct bif_sd, BifsdRxWork);
	if (atomic_read(&sd->bifsd_rx_dpc_tskcnt) > 0) {
		bifsd_rx_dpc(sd);
	}
}
static int sddev_notify(struct notifier_block *self,
			       unsigned long action, void *data)
{
    struct bif_sd *sd = get_sd_info();
	struct bifsd_req *req;
	switch (action) {
	case BIFSD_DATA_RX_COMP:
        req = bifsd_deq(sd, &sd->bifsd_rx_freeq,
                            &sd->bifsd_rx_freecount, &sd->bifsd_rx_freeq_lock);
        if (!req) {
            dev_err(sd->dev, "no req to send\n");
            dev_err(sd->dev, "free:%d, post:%d\n", sd->bifsd_rx_freecount,
                    sd->bifsd_rx_postcount);
        }
        /* TBD:analsis buf from bifsd */
        req->buf = data;
        /* TBD:analsis data len */
        bifsd_enq(sd, &sd->bifsd_rx_postq, req,
                &sd->bifsd_rx_postcount, &sd->bifsd_rx_postq_lock);

        atomic_inc(&sd->bifsd_rx_dpc_tskcnt);
	WAKE_RX_WORK(sd);
    default:
        break;
	}
	return NOTIFY_OK;
}
static struct notifier_block sddev_nb = {
	.notifier_call = sddev_notify,
    .priority = INT_MAX	/* Need to be called first of all */
};
void update_cid_val(struct bif_sd *sd)
{
    unsigned char i;
    unsigned char update_flag;
    u32 reg_val[4];
    volatile u32 *cid_reg = (volatile u32 *)EMMC_CID;

    for (i = 0; i < 4; i++)
    {
        reg_val[i] = sd_readl(sd, CID, 4*i);
    }
    update_flag = reg_val[3] & 0xFF000000;
    if(update_flag)
    {
        return;
    }
    else
    {
        for (i = 0; i < 4; i++)
        {
            sd_writel(sd, CID, reg_val[i], 4*i);
        }
        cid_reg[3] |= 0x01000000;
    }

    sd_writel(sd, MEM_MGMT, 0x01, 0);
    return;
}
void mmc_cid_config(struct bif_sd *sd)
{
    int i;
    int tbl_size;
    u32 *cid_tbl_val = NULL;
    card_type_t card_type = MMC_MODE;
    const u32 cid_sd_card_tbl_val[] = {
        /* CID Tbl Addr Ofst    VALUE to be initialised */
        /*0x00*/         0xc3aa004a,
        /*0x04*/         0x3610432c,
        /*0x08*/         0x53483235,
        /*0x0C*/         0x00015048,
    };
    const u32 cid_mmc_4_4_card_tbl_val[] = {
        /* CID Tbl Addr Ofst    VALUE to be initialised */
        /*0x00*/         0xc3aa004a,
        /*0x04*/         0x3610432c,
        /*0x08*/         0x53483235,
        /*0x0C*/         0x00010048,
    };
    const u32 cid_mmc_4_2_card_tbl_val[] = {
        /* CSD Tbl Addr Ofst VALUE to be initialised */
        /*0x00*/         0xc3aa004a,
        /*0x04*/         0x3610432c,
        /*0x08*/         0x53483235,
        /*0x0C*/         0x00015048,
    };

    switch(card_type)
    {
    case sd_card:
    case sdhc_card:
    case sdxc_card:
        cid_tbl_val = (u32 *)&cid_sd_card_tbl_val;
        break;
    case mmc_card_4_2:
        cid_tbl_val = (u32 *)&cid_mmc_4_2_card_tbl_val;
        break;
    case mmc_card_4_4:
    case mmc_card_4_5:
    case mmc_card_5_0:
        cid_tbl_val = (u32 *)&cid_mmc_4_4_card_tbl_val;
        break;
    default:
        dev_err(sd->dev, "card type falt\n");
        break;
    }

    tbl_size = 4;
    for (i = 0; i < tbl_size; i++) {
        sd_writel(sd, CID, cid_tbl_val[i], 4*i);
    }
}
void mmc_csd_config(struct bif_sd *sd)
{
    int i;
    u32 *csd_tbl_val;
    card_type_t card_type = MMC_MODE;
    const u32 csd_sd_card_tbl_val [] = {
        /* CSD Tbl Addr Ofst    VALUE to be initialised */
        /*0x00*/         0x08966004,
        /*0x04*/         0xffc00344,
        /*0x08*/         0x325f5980,
        /*0x0C*/         0x000026ff,
    };
    const u32 csd_sdhc_card_tbl_val [] = {
        /* CSD Tbl Addr Ofst    VALUE to be initialised */
        /*0x00*/         0x08966004,
        /*0x04*/         0xffc00344,
        /*0x08*/         0x325f5980,
        /*0x0C*/         0x004026ff,
    };
    const u32 csd_mmc_card_tbl_val [] = {
        /* CSD Tbl Addr Ofst    VALUE to be initialised */
        /*0x00*/         0xe7966004,
        /*0x04*/         0xffc0031c,
        /*0x08*/         0x2a3f599b,
        /*0x0C*/         0x009086ff,
    };

    switch(card_type)
    {
    case sd_card:
        csd_tbl_val = (u32 *)&csd_sd_card_tbl_val;
        break;
    case sdhc_card:
    case sdxc_card:
        csd_tbl_val = (u32 *)&csd_sdhc_card_tbl_val;
        break;
    case mmc_card_4_2:
    case mmc_card_4_4:
    case mmc_card_4_5:
    case mmc_card_5_0:
        csd_tbl_val = (u32 *)&csd_mmc_card_tbl_val;
        break;
    default:
        return;
    }
    for (i = 0; i < 4; i++) {
        sd_writel(sd, CSD, csd_tbl_val[i], 4*i);
    }
}

void mmc_extended_csd_config(struct bif_sd *sd)
{
    int i;
    int tbl_size;
    u32 *csd_tbl_val;
    card_type_t card_type = MMC_MODE;
    const u32 ext_csd_mmc_4_2_tbl_val [] = {
        /* Extend CSD Tbl Addr Ofst VALUE to be initialised */
        /*CSD_SD_STANDARD_CARD:192bits  */
        /*0x00*/         0x00000000,
        /*0x04*/         0x02020000,
        /*0x08*/         0x55555502,
        /*0x0C*/         0x461e1e55,
        /*0x10*/         0x008c8c46,
        /*0x14*/         0x00000000,
    };

    const u32 ext_csd_mmc_4_4_tbl_val [] = {
        /* Extend CSD Tbl Addr Ofst VALUE to be initialised */
        /*CSD_SD_STANDARD_CARD:624bits */
        /*0x00*/         0x00000000,
        /*0x04*/         0x00000000,
        /*0x08*/         0x00000000,
        /*0x0C*/         0x00000000,
        /*0x10*/         0x00000000,
        /*0x14*/         0x00000000,
        /*0x18*/         0x00000000,
        /*0x1c*/         0x00000000,
        /*0x20*/         0x00480000,
        /*0x24*/         0x02020300,
        /*0x28*/         0x55555555,
        /*0x2c*/         0x46461e1e,
        /*0x30*/         0x00008c8c,
        /*0x34*/         0x01010000,
        /*0x38*/         0x00010001,
        /*0x3c*/         0x01010100,
        /*0x40*/         0x00000000,
        /*0x44*/         0x00000000,
        /*0x48*/         0x00000000,
    };
    const u32 ext_csd_mmc_4_5_tbl_val [] = {
        /* Extend CSD Tbl Addr Ofst VALUE to be initialised */
        /*CSD_SD_STANDARD_CARD:1168bits*/

        /*0x00*/         0x00000000,
        /*0x04*/         0x00000000,
        /*0x08*/         0x00000000,
        /*0x0c*/         0x00000000,
        /*0x10*/         0x00000000,
        /*0x14*/         0x00000000,
        /*0x18*/         0x00000000,
        /*0x1c*/         0x00000000,
        /*0x20*/         0x00000000,
        /*0x24*/         0x00000000,
        /*0x28*/         0x00000000,
        /*0x2c*/         0x00000000,
        /*0x30*/         0x00000000,
        /*0x34*/         0x00000000,
        /*0x38*/         0x00000000,
        /*0x3c*/         0x00000000,
        /*0x40*/         0x00000000,
        /*0x44*/         0x00000000,
        /*0x48*/         0x00000000,
        /*0x4c*/         0x00000148,
        /*0x50*/         0x03000000,
        /*0x54*/         0x00000202,
        /*0x58*/         0x55555555,
        /*0x5c*/         0x46461e1e,
        /*0x60*/         0x00008c8c,
        /*0x64*/         0x01010000,
        /*0x68*/         0x01010001,
        /*0x6c*/         0x01010100,
        /*0x70*/         0x00000001,
        /*0x74*/         0x00000000,
        /*0x78*/         0x00000000,
        /*0x7c*/         0x00000000,
        /*0x80*/         0x00000001,
        /*0x84*/         0x00000000,
        /*0x88*/         0x00000000,
        /*0x8c*/         0x00000000,
        /*0x90*/         0x00000000,
    };

    const u32 ext_csd_mmc_5_0_tbl_val [] = {
        /* Extend CSD Tbl Addr Ofst VALUE to be initialised */
        /*CSD_SD_STANDARD_CARD:1784bits */
        /*0xA4*/           0x00000000,
        /*0xA8*/           0x00000000,
        /*0xAC*/           0x00000000,
        /*0xB0*/           0x00000000,
        /*0xB4*/           0x00000000,
        /*0xB8*/           0x00000000,
        /*0xBC*/           0x00000000,
        /*0xC0*/           0x00000000,
        /*0xC4*/           0x00000000,
        /*0xC8*/           0x00000000,
        /*0xCC*/           0x00000000,
        /*0xD0*/           0x00000000,
        /*0xD4*/           0x00000000,
        /*0xD8*/           0x00000000,
        /*0xDC*/           0x00000000,
        /*0xE0*/           0x00000000,
        /*0xE4*/           0x00000000,
        /*0xE8*/           0x00000000,
        /*0xEC*/           0x00000000,
        /*0xF0*/           0x00000000,
        /*0xF4*/           0x00000000,
        /*0xF8*/           0x00000000,
        /*0xFC*/           0x48000000,
        /*0x100*/           0x00000001,
        /*0x104*/           0x02030000,
        /*0x108*/           0x55000002,
        /*0x10C*/           0x1e555555,
        /*0x110*/           0x8c46461e,
        /*0x114*/           0x0000008c,
        /*0x118*/           0x01000100,
        /*0x11C*/           0x01000100,
        /*0x120*/           0x01010001,
        /*0x124*/           0x00000101,
        /*0x128*/           0x00000000,
        /*0x12C*/           0x00000000,
        /*0x130*/           0x00000000,
        /*0x134*/           0x00020007,
        /*0x138*/           0x0a0a1f13,
        /*0x13C*/           0x8888eeee,
        /*0x140*/           0x460f1e00,
        /*0x144*/           0x0014780f,
        /*0x148*/           0x03a3e000,
        /*0x14C*/           0x0a0a1410,
        /*0x150*/           0x09010108,
        /*0x154*/           0x00200808,
        /*0x158*/           0x55c8f400,
        /*0x15C*/           0x0a640001,
        /*0x160*/           0x99eeeeee,
        /*0x164*/           0x00001e01,
        /*0x168*/           0x32000000,
        /*0x16C*/           0x0000000a,
        /*0x170*/           0x0002ee00,
        /*0x174*/           0x00000000,
        /*0x178*/           0x00000000,
        /*0x17C*/           0x01202001,
        /*0x180*/           0x00000001,
    };
    switch(card_type)
    {
    case mmc_card_4_2:
        csd_tbl_val = (u32 *)&ext_csd_mmc_4_2_tbl_val;
        tbl_size = sizeof(ext_csd_mmc_4_2_tbl_val)/sizeof(ext_csd_mmc_4_2_tbl_val[0]);
        break;
    case mmc_card_4_4:
        csd_tbl_val = (u32 *)&ext_csd_mmc_4_4_tbl_val;
        tbl_size = sizeof(ext_csd_mmc_4_4_tbl_val)/sizeof(ext_csd_mmc_4_4_tbl_val[0]);
        break;
    case mmc_card_4_5:
        csd_tbl_val = (u32 *)&ext_csd_mmc_4_5_tbl_val;
        tbl_size = sizeof(ext_csd_mmc_4_5_tbl_val)/sizeof(ext_csd_mmc_4_5_tbl_val[0]);
        break;
    case mmc_card_5_0:
        csd_tbl_val = (u32 *)&ext_csd_mmc_5_0_tbl_val;
        tbl_size = sizeof(ext_csd_mmc_5_0_tbl_val)/sizeof(ext_csd_mmc_5_0_tbl_val[0]);
        break;
    default:
        return;
    }

    for (i = 0; i < tbl_size; i++) {
        sd_writel(sd, EXTENDED_CSD, csd_tbl_val[i], 4*i);
    }
}
void mmc_set_sd_status_reg(struct bif_sd *sd)
{
    int i;
    const u32 sd_status_tbl_val [] = {
        /*0x00*/         0x01160000,
        /*0x04*/         0x02199000,
        /*0x08*/         0x00000032,
        /*0x0C*/         0x00000000,
    };
    for (i = 0; i < 4; i++) {
        sd_writel(sd, STATUS, sd_status_tbl_val[i], 4*i);
    }
}
void mmc_set_sd_scr_reg(struct bif_sd *sd)
{
    int i;
    u32 *scr_tbl_val;
    card_type_t card_type = MMC_MODE;
    const u32 sd_tbl_val [] = {
        /*0x00*/         0x00000000,
        /*0x04*/         0x01b50000,
    };
    const u32 sdhc_tbl_val [] = {
        /*0x00*/         0x00000000,
        /*0x04*/         0x02b50000,
    };
    const u32 sdxc_tbl_val [] = {
        /*0x00*/         0x00000000,
        /*0x04*/         0x02b58003,
    };
    switch(card_type)
    {
    case sd_card:
        scr_tbl_val = (u32 *)&sd_tbl_val;
        break;
    case sdhc_card:
        scr_tbl_val = (u32 *)&sdhc_tbl_val;
        break;
    case sdxc_card:
        scr_tbl_val = (u32 *)&sdxc_tbl_val;
        break;
    default:
        return;
    }

    for (i = 0; i < 2; i++) {
        sd_writel(sd, SCR, scr_tbl_val[i], 4*i);
    }
}

void device_mode_select(struct bif_sd *sd)
{
    u32 reg_val = 0;
    card_type_t card_type = MMC_MODE;
    reg_val = sd_readl(sd, PROGRAM_REG, 0);
    reg_val &=0xFFFFFFFC;

    switch(card_type)
    {
    case sd_card:
    case sdhc_card:
    case sdxc_card:
        reg_val |= BIT(0);
        break;
    case mmc_card_4_2:
    case mmc_card_4_4:
    case mmc_card_4_5:
    case mmc_card_5_0:
        reg_val |= BIT(1);
        break;
    default:
        dev_err(sd->dev, "card type fault\n");
        break;
    }

    reg_val |= BIT(4);

    sd_writel(sd, PROGRAM_REG, reg_val, 0);
}

void sd_card_init(struct bif_sd *sd)
{
    u32 reg_val;
    card_type_t card_type = MMC_MODE;
    if((card_type == sd_card)||(card_type == sdhc_card)||(card_type == sdxc_card))
    {
        mmc_set_sd_status_reg(sd);
        mmc_set_sd_scr_reg(sd);
        sd_writel(sd, SECURITY_INT_ENABLE, EMMC_SECURITY_INT_ENABLE_VAL, 0);

        reg_val = sd_readl(sd, BLOCK_COUNT_SECURITY, 0);
        reg_val &=0xFFFF0000;
        reg_val |= BIT(31);
        sd_writel(sd, BLOCK_COUNT_SECURITY, reg_val, 0);
    }
}

/* Initialize Bifsd device */
int bifsd_hobot_priv_init(struct bif_sd *sd)
{
    sd_writel(sd, INT_ENABLE_1, 0xFFFFCFFF, 0);
    sd_writel(sd, INT_ENABLE_2, 0x007FFFFF, 0);
    mmc_set_power_up(sd);
    mmc_config_ocr_reg(sd);
    mmc_extended_csd_config(sd);
    mmc_set_hard_reset_cnt(sd);
    mmc_csd_config(sd);
    mmc_cid_config(sd);
    mmc_set_out_range_addr(sd);

    device_mode_select(sd);
    mmc_disable_acc_bypass(sd);
#ifndef HUGO_PLM
    bifsd_config_timing(sd);
#endif
    card_power_up(sd);

    return 0;
}

static const struct bifsd_drv_data hobot_drv_data = {
	.init			= bifsd_hobot_priv_init,
};

static const struct of_device_id bifsd_hobot_of_match[] = {
	/* SoC-specific compatible strings w/ soc_ctl_map */
	{
		.compatible = "hobot,bifsd",
        .data = &hobot_drv_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bifsd_hobot_of_match);

static void bifsd_tasklet_func(unsigned long priv)
{
	struct bif_sd *sd = (struct bif_sd *)priv;
    enum bifsd_state state;

	spin_lock(&sd->lock);
	state = sd->state;

    switch (state) {
		case STATE_IDLE:
			break;

		case STATE_RX_DATA_COMP:
            #ifdef BIFSD_TX_RX_TEST_MODE
            bifsd_notify_rx_data((void *)(u64)sd->wr_buf);
            #endif
            /* TBD:analsis the data that received from host */
            break;
		case STATE_TX_DATA_COMP:
            /* TBD:release buffer and so on */

			break;
    }

	spin_unlock(&sd->lock);
}

static irqreturn_t bifsd_interrupt(int irq, void *dev_id)
{
    struct bif_sd *sd = dev_id;
	u32 pending_1;
    u32 pending_2;
    u32 pending;
    u32 reg_val;
    int ret;

	pending_1 = sd_readl(sd, INT_STATUS_1, 0);
    pending_2 = sd_readl(sd, INT_STATUS_2, 0);

    /* Protocol read without count */
    if((pending_1 &  MMC_MULTI_BLOCK_READ_WRITE) && (pending_2 & MMC_SECURITY_PROTOCOL_READ))
    {
        /* Read address from master */
        reg_val = sd_readl(sd, ARGUMENT_REG, 0);

        /* need set dma address for master read */
        sd_writel(sd, DMA_ADDR, reg_val, 0);
        sd_writel(sd, MEM_MGMT, reg_val, 0);

        /* the block count is the slave set */
        sd_writel(sd, BLOCK_CNT, sd->block_cnt, 0);
        sd_writel(sd, INT_STATUS_2, MMC_SECURITY_PROTOCOL_READ, 0);
        sd_writel(sd, INT_STATUS_1, MMC_MULTI_BLOCK_READ_WRITE, 0);
    }
    /* Protocol write without count */
    if((pending_1 &  MMC_MULTI_BLOCK_READ_WRITE) && (pending_2 & MMC_SECURITY_PROTOCOL_WRITE))
    {
        /* Write address from master */
        reg_val = sd_readl(sd, ARGUMENT_REG, 0);
        /* need set dma address for master write*/
        sd_writel(sd, DMA_ADDR, reg_val, 0);
        sd_writel(sd, MEM_MGMT, 0x01, 0);

        /* the block count is the slave set */
        sd_writel(sd, BLOCK_CNT, sd->block_cnt, 0);
        sd_writel(sd, INT_STATUS_1, MMC_MULTI_BLOCK_READ_WRITE, 0);
        sd_writel(sd, INT_STATUS_2, MMC_SECURITY_PROTOCOL_WRITE, 0);
    }

    if(pending_1 != 0)
    {
        pending = pending_1;
        if(pending & MMC_IDLE_CMD)
        {
            /* CMD0 received then reinit EMMC Card */
            if (sd->drv_data && sd->drv_data->init) {
		        ret = sd->drv_data->init(sd);
		        if (ret) {
			        dev_err(sd->dev, "implementation specific init failed\n");
		        }
	        }
            sd_writel(sd, INT_STATUS_1, MMC_IDLE_CMD, 0);
        }

        if(pending & MMC_SET_BLOCK_LEN)
        {
            sd->block_len = sd_readl(sd, BLOCK_LEN, 0);
            sd_writel(sd, INT_STATUS_1, MMC_SET_BLOCK_LEN, 0);
        }

        if(pending & MMC_SET_BLOCK_CNT)
        {
            reg_val = sd_readl(sd, ERASE_BLOCK_CNT, 0);
            if(reg_val)
            sd->block_cnt = reg_val & 0x0000ffff;
            sd_writel(sd, INT_STATUS_1, MMC_SET_BLOCK_CNT, 0);
        }

        if(pending & MMC_INACTIVE_CMD)
        {
            sd_writel(sd, INT_STATUS_1, MMC_INACTIVE_CMD, 0);
        }

        if(pending & MMC_BLK_READ)
        {
            if(pending & MMC_MULTI_BLOCK_READ_WRITE)
            {
                if(pending & MMC_STOP_CMD)
                {
                    /* TBD */
                }
                else
                {
                    reg_val = sd_readl(sd, ARGUMENT_REG, 0);
                    if(reg_val)
                    sd->rd_buf = reg_val;

                    sd_writel(sd, DMA_ADDR, sd->rd_buf, 0);
                    sd_writel(sd, MEM_MGMT, 0x01, 0);

                    sd_writel(sd, BLOCK_CNT, sd->block_cnt, 0);
                    sd_writel(sd,INT_STATUS_1,  MMC_BLK_READ | MMC_MULTI_BLOCK_READ_WRITE, 0);
                }
            }
            else
            {
                reg_val = sd_readl(sd, ARGUMENT_REG, 0);
                if(reg_val)
                sd->rd_buf = reg_val;

                sd_writel(sd, DMA_ADDR, sd->rd_buf, 0);
                sd_writel(sd, MEM_MGMT, 0x01, 0);
                sd_writel(sd, BLOCK_CNT, 0x01, 0);
                sd_writel(sd, INT_STATUS_1, MMC_BLK_READ, 0);
            }
        }

        if(pending & MMC_READ_BLOCK_CNT)
        {
            sd->state = STATE_TX_DATA_COMP;
            tasklet_hi_schedule(&sd->tasklet);
            sd_writel(sd, INT_STATUS_1, MMC_READ_BLOCK_CNT, 0);
        }

        if(pending & MMC_STOP_CMD)
        {
            sd_writel(sd, DMA_ADDR, 0, 0);

            reg_val = sd_readl(sd, PROGRAM_REG, 0);
            if(reg_val & BIT(3))
                sd_writel(sd, MEM_MGMT, 0x21, 0);

            sd_writel(sd, INT_STATUS_1, MMC_STOP_CMD, 0);
        }

        if(pending & MMC_BLK_WRITE)
        {
            if(pending & MMC_MULTI_BLOCK_READ_WRITE)
            {
                reg_val = sd_readl(sd, ARGUMENT_REG, 0);
                if(reg_val)
                sd->wr_buf = reg_val;

                sd_writel(sd, DMA_ADDR, sd->wr_buf, 0);
                sd_writel(sd, MEM_MGMT, 0x01, 0);

                sd_writel(sd, BLOCK_CNT, sd->block_cnt, 0);
                sd_writel(sd, INT_STATUS_1, MMC_BLK_WRITE | MMC_MULTI_BLOCK_READ_WRITE, 0);
            }
            else
            {
                reg_val = sd_readl(sd, ARGUMENT_REG, 0);
                if(reg_val)
                sd->wr_buf = reg_val;

                sd_writel(sd, DMA_ADDR, sd->wr_buf, 0);
                sd_writel(sd, MEM_MGMT, 0x01, 0);
                sd_writel(sd, BLOCK_CNT, 0x01, 0);
                sd_writel(sd, INT_STATUS_1, MMC_BLK_WRITE, 0);

                sd->state = STATE_RX_DATA_COMP;
                tasklet_hi_schedule(&sd->tasklet);
            }
        }

        if(pending & MMC_WRITE_BLOCK_CNT)
        {
            reg_val = sd_readl(sd, PROGRAM_REG, 0);
            if(reg_val & BIT(3))
                sd_writel(sd, MEM_MGMT, 0x21, 0);

            sd->state = STATE_RX_DATA_COMP;
            tasklet_hi_schedule(&sd->tasklet);
            sd_writel(sd, INT_STATUS_1, MMC_WRITE_BLOCK_CNT, 0);
        }

        if(pending & MMC_CID_UPDATE)
        {
            update_cid_val(sd);

            sd_writel(sd, INT_STATUS_1, MMC_CID_UPDATE, 0);
        }

        if(pending & MMC_CSD_UPDATE)
        {
            sd_writel(sd, MEM_MGMT, 0x21, 0);
            sd_writel(sd, INT_STATUS_1, MMC_CSD_UPDATE, 0);
        }

        if(pending & MMC_NUM_WELL_WRITE_BLOCK)
        {
            sd_writel(sd, DMA_ADDR, sd->wr_buf, 0);
            sd_writel(sd, MEM_MGMT, 0x01, 0);
            sd_writel(sd, INT_STATUS_1, MMC_NUM_WELL_WRITE_BLOCK, 0);
        }

        if(pending & MMC_GENERAL_READ_WRITE)
        {
            reg_val = sd_readl(sd, ARGUMENT_REG, 0);
            if(reg_val)
            {
                sd_writel(sd, DMA_ADDR, sd->rd_buf, 0);
                sd_writel(sd, MEM_MGMT, 0x01, 0);
            }
            else
            {
                sd_writel(sd, DMA_ADDR, sd->wr_buf, 0);
                sd_writel(sd, MEM_MGMT, 0x01, 0);
            }

            sd_writel(sd, INT_STATUS_1, MMC_GENERAL_READ_WRITE, 0);
        }

        if(pending & MMC_BLOCK_COUNT_CLEAR)
        {
            sd_writel(sd, INT_STATUS_1, MMC_BLOCK_COUNT_CLEAR, 0);
        }

        if(pending & MMC_VOLTAGE_SWITCH)
        {
            sd_writel(sd, INT_STATUS_1, MMC_VOLTAGE_SWITCH, 0);
        }

        if(pending &  MMC_GO_PRE_IDLE)
        {
            /* go pre-idel state */
            if (sd->drv_data && sd->drv_data->init) {
		        ret = sd->drv_data->init(sd);
		        if (ret) {
			        dev_err(sd->dev, "implementation specific init failed\n");
		        }
	        }

            sd_writel(sd, INT_STATUS_1, MMC_GO_PRE_IDLE, 0);
        }

        if(pending & MMC_CMD_61)
        {
            reg_val = sd_readl(sd, ARGUMENT_REG, 0);
            /* need set dma address for master read */
            sd_writel(sd, DMA_ADDR, reg_val, 0);
            sd_writel(sd, MEM_MGMT, 0x01, 0);

            /* the block count must be send from master */
            sd_writel(sd, BLOCK_CNT, sd->read_blk_num, 0);
            sd_writel(sd, INT_STATUS_1, MMC_CMD_61, 0);
        }

        if(pending & MMC_CMD6_ALWAYS)
        {
            sd->cmd_argu = sd_readl(sd, ARGUMENT_REG, 0);

            sd_writel(sd, MEM_MGMT, 0x21, 0);
            sd_writel(sd, INT_STATUS_1,  MMC_CMD6_ALWAYS, 0);
        }

        if(pending & MMC_CMD55)
        {
            sd->cmd_argu = sd_readl(sd, ARGUMENT_REG, 0);
            sd_writel(sd, INT_STATUS_1, MMC_CMD55, 0);
        }
    }

    if(pending_2 != 0)
    {
        pending = pending_2;
        if(pending & MMC_CMD12)
        {
            /* Clear Int state */
            sd_writel(sd, INT_STATUS_2, MMC_CMD12, 0);
        }

        if(pending & MMC_SLEEP_CMD)
        {
            /* Enter sleep state */
            sd_writel(sd, MEM_MGMT, 0x21, 0);
            sd_writel(sd, INT_STATUS_2, MMC_SLEEP_CMD, 0);
        }

        if(pending & MMC_AWAKE_CMD)
        {
            /* Enter Standby state */
            sd_writel(sd, MEM_MGMT, 0x21, 0);
            sd_writel(sd, INT_STATUS_2, MMC_AWAKE_CMD, 0);
        }

        if(pending & MMC_UPDATE_EXT_CSD)
        {
            u32 old_val;
            u32 val;
            unsigned char val_offset;
            u32 new_val;
            u32 update_val;
            u32 update_addr = 0;

            val = sd_readl(sd, ARGUMENT_REG, 0);
            val &= 0x00FF0000;
            val = val >> 16;

            val_offset = val % 4;
            if(val_offset)
            {
                val -= val_offset;
            }
            if(val >= 128)
            {
                update_addr = val - 48;
            }
            else if(val < 80)
            {
                update_addr = val;
            }
            else
            {
                dev_err(sd->dev, "update ext csd register error\n");
            }

            update_val = sd_readl(sd, UPDATE_EXT_CSD, 0);
            old_val = sd_readl(sd, EXTENDED_CSD, update_addr);

            old_val = old_val & (~(0xFF << val_offset*8));
            new_val = update_val << val_offset*8;
            new_val = old_val | new_val;

            sd_writel(sd, EXTENDED_CSD, new_val, update_addr);

            sd_writel(sd, MEM_MGMT, 0x21, 0);
            sd_writel(sd, INT_STATUS_2, MMC_UPDATE_EXT_CSD, 0);
        }

        if(pending & MMC_HARDWARE_RESET)
        {
            sd_writel(sd, INT_STATUS_2, MMC_HARDWARE_RESET, 0);
        }

        if(pending & MMC_CARD_SELECT)
        {
            /* The core moves from stand by state to transfer state */
            sd_writel(sd, INT_STATUS_2, MMC_CARD_SELECT, 0);
        }

        if(pending & MMC_CARD_STATUS_CHANGE)
        {
            /* The core moves from stand by state to transfer state */
            sd_writel(sd, INT_STATUS_2, MMC_CARD_STATUS_CHANGE, 0);
        }
    }

    return IRQ_HANDLED;
}
int bifsd_pltfm_register(struct platform_device *pdev,
			  const struct bifsd_drv_data *drv_data)
{
	struct resource	*regs;
    struct resource mem_reserved;
    struct resource	*sysctrl;
    struct device_node *np = NULL;
	struct bif_sd *sd;
    int ret;

	sd = devm_kzalloc(&pdev->dev, sizeof(struct bif_sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	sd->irq = platform_get_irq(pdev, 0);
	if (sd->irq < 0)
		return sd->irq;

	sd->drv_data = drv_data;
	sd->dev = &pdev->dev;
	sd->irq_flags = 0;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sd->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(sd->regs))
		return PTR_ERR(sd->regs);

    sysctrl = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    sd->sysctrl_reg = ioremap(sysctrl->start, 0x10);
	if (IS_ERR(sd->sysctrl_reg))
		return PTR_ERR(sd->sysctrl_reg);
	/* Get registers' physical base address */
	sd->phy_regs = regs->start;

    /* request memory address */
    np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
    if (!np) {
        dev_err(&pdev->dev, "No %s specified\n", "memory-region");
    }

    ret = of_address_to_resource(np, 0, &mem_reserved);
    if (ret) {
        dev_err(&pdev->dev, "No memory address assigned to the region\n");
    }
    else
    {
        sd->paddr = (void __iomem *)mem_reserved.start;
        sd->vaddr = (void __iomem *)memremap(mem_reserved.start, resource_size(&mem_reserved), MEMREMAP_WB);
        dev_err(&pdev->dev, "Allocate reserved memory, vaddr: 0x%0llx, paddr: 0x%0llx\n",
                (uint64_t)sd->paddr, (uint64_t)sd->vaddr);
    }
    set_sd_info(sd);
	platform_set_drvdata(pdev, sd);

    sd->rst = devm_reset_control_get(&pdev->dev, "bifsd");
    if (IS_ERR(sd->rst)) {
        dev_err(&pdev->dev, "missing controller reset\n");
        return PTR_ERR(sd->rst);
    }
    reset_control_assert(sd->rst);
    udelay(2);
    reset_control_deassert(sd->rst);

	if (drv_data && drv_data->init) {
		ret = drv_data->init(sd);
		if (ret) {
			dev_err(sd->dev,
				"implementation specific init failed\n");
            devm_kfree(&pdev->dev, sd);
			goto err;
		}
	}

    tasklet_init(&sd->tasklet, bifsd_tasklet_func, (unsigned long)sd);
	ret = devm_request_irq(sd->dev, sd->irq, bifsd_interrupt,
			       sd->irq_flags, "bifsd", sd);
	if (ret)
		goto err;

    INIT_LIST_HEAD(&sd->bifsd_rx_postq);
    INIT_LIST_HEAD(&sd->bifsd_rx_freeq);

    sd->bifsd_rx_reqs = bifsd_qinit(sd, &sd->bifsd_rx_freeq, 32);
	if (!sd->bifsd_rx_reqs)
		goto err;

    INIT_WORK(&sd->BifsdRxWork, bifsd_RxWorker);

	sd->bifsd_rxwq = create_singlethread_workqueue("bifsd_rxwq");
	if (!sd->bifsd_rxwq) {
		dev_err(sd->dev,"insufficient memory to create rxworkqueue.\n");
		goto err;
	}
    bifsd_register_notify(&sddev_nb);

    gpio_request(BIFSD_DETECT_GPIO, NULL);
    gpio_direction_output(BIFSD_DETECT_GPIO, 0);

 err:
    return ret;
}
EXPORT_SYMBOL_GPL(bifsd_pltfm_register);

static int bifsd_probe(struct platform_device *pdev)
{
	const struct bifsd_drv_data *drv_data;
	const struct of_device_id *match;
    pr_err("%s\n",__func__);
	match = of_match_node(bifsd_hobot_of_match, pdev->dev.of_node);
	drv_data = match->data;
	return bifsd_pltfm_register(pdev, drv_data);
}

static int bifsd_remove(struct platform_device *pdev)
{
    struct bif_sd *sd = platform_get_drvdata(pdev);

    /* Disable interrupt */
    sd_writel(sd, INT_STATUS_1, 0xFFFFFFFF, 0);
    sd_writel(sd, INT_STATUS_2, 0xFFFFFFFF, 0);
    sd_writel(sd, INT_ENABLE_1, 0, 0);
    sd_writel(sd, INT_ENABLE_2, 0, 0);

	bifsd_free_q(sd, &sd->bifsd_rx_freeq, false, &sd->bifsd_rx_freeq_lock);

    bifsd_unregister_notify(&sddev_nb);
    devm_kfree(&pdev->dev, sd);
    return 0;
}
EXPORT_SYMBOL(bifsd_remove);

static struct platform_driver bifsd_hobot_driver = {
	.driver = {
		.name = "bifsd",
		.of_match_table = bifsd_hobot_of_match,
	},
	.probe = bifsd_probe,
	.remove = bifsd_remove,
};

module_platform_driver(bifsd_hobot_driver);

MODULE_DESCRIPTION("Driver for the HobotRobotics Bifsd Controller");
MODULE_AUTHOR("shaochuan.zhang <shaochuan.zhang@horizon.ai>");
MODULE_LICENSE("GPL");
