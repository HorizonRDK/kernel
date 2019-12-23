/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_I2C_H__
#define __HOBOT_I2C_H__
struct cfg_bits_s {
	unsigned int clkdiv:8;
	unsigned int en:1;
	unsigned int tran_en:1;
	unsigned int pmu_en:1;
	unsigned int to_en:1;
	unsigned int pmc_soft_en:1;
	unsigned int pmu_pri:4;
	unsigned int dir_rd:1;
	unsigned int ack:1;
};
union cfg_reg_e {
	unsigned int all;
	struct cfg_bits_s bit;
};

struct saddr_bits_s {
	unsigned int rsv:1;
	unsigned int saddr:10;
	unsigned int ten_addr:1;
};
union saddr_reg_e {
	unsigned int all;
	struct saddr_bits_s bit;
};

struct dcount_bits_s {
	unsigned int w_dcount:16;
	unsigned int r_dcount:16;
};
union dcount_reg_e {
	unsigned int all;
	struct dcount_bits_s bit;
};

struct ctl_bits_s {
	unsigned int rsv:2;
	unsigned int rd:1;
	unsigned int wr:1;
	unsigned int sto:1;
	unsigned int sta:1;
	unsigned int rfifo_clr:1;
	unsigned int tfifo_clr:1;
	unsigned int pfifo_clr:1;
};
union ctl_reg_e {
	unsigned int all;
	struct ctl_bits_s bit;
};

struct tdata_bits_s {
	unsigned int tx_data:8;
};
union tdata_reg_e {
	unsigned int all;
	struct tdata_bits_s bit;
};

struct rdata_bits_s {
	unsigned int rx_data:8;
};
union rdata_reg_e {
	unsigned int all;
	struct rdata_bits_s bit;
};

struct status_bits_s {
	unsigned int busy:1;
	unsigned int rxack:1;
	unsigned int al:1;
	unsigned int rx_full:1;
	unsigned int rx_empty:1;
	unsigned int tx_full:1;
	unsigned int tx_empty:1;
	unsigned int bb_pmu:1;
};

union status_reg_e {
	unsigned int all;
	struct status_bits_s bit;
};

struct tocnt_bits_s {
	unsigned int to_cnt:16;
};
union tocnt_reg_e {
	unsigned int all;
	struct tocnt_bits_s bit;
};

struct sprcpnd_bits_s {
	unsigned int al:1;
	unsigned int nack:1;
	unsigned int tr_done:1;
	unsigned int rrdy:1;
	unsigned int xrdy:1;
	unsigned int aerr:1;
	unsigned int to:1;
	unsigned int sterr:1;
	unsigned int rsv1:1;
	unsigned int nack_pmu:1;
	unsigned int tr_done_pmu:1;
	unsigned int to_pmu:1;
	unsigned int rsv2:1;
	unsigned int ov_pmu:1;
};
union sprcpnd_reg_e {
	unsigned int all;
	struct sprcpnd_bits_s bit;
};

struct intmask_bits_s {
	unsigned int al_mask:1;
	unsigned int nack_mask:1;
	unsigned int tr_done_mask:1;
	unsigned int rrdy_mask:1;
	unsigned int xrdy_mask:1;
	unsigned int aerr_mask:1;
	unsigned int to_mask:1;
	unsigned int sterr_mask:1;
	unsigned int rsv1:1;
	unsigned int nack_pmu_mask:1;
	unsigned int tr_done_pmu_mask:1;
	unsigned int to_pmu_mask:1;
	unsigned int rsv2:1;
	unsigned int ov_pmu_mask:1;
};
union intmask_reg_e {
	unsigned int all;
	struct intmask_bits_s bit;
};

struct intsetmask_bits_s {
	unsigned int al_mask:1;
	unsigned int nack_mask:1;
	unsigned int tr_done_mask:1;
	unsigned int rrdy_mask:1;
	unsigned int xrdy_mask:1;
	unsigned int aerr_mask:1;
	unsigned int to_mask:1;
	unsigned int sterr_mask:1;
	unsigned int rsv1:1;
	unsigned int nack_pmu_mask:1;
	unsigned int tr_done_pmu_mask:1;
	unsigned int to_pmu_mask:1;
	unsigned int rsv2:1;
	unsigned int ov_pmu_mask:1;
};
union intsetmask_reg_e {
	unsigned int all;
	struct intsetmask_bits_s bit;
};

struct intunmask_bits_s {
	unsigned int al_mask:1;
	unsigned int nack_mask:1;
	unsigned int tr_done_mask:1;
	unsigned int rrdy_mask:1;
	unsigned int xrdy_mask:1;
	unsigned int aerr_mask:1;
	unsigned int to_mask:1;
	unsigned int sterr_mask:1;
	unsigned int rsv1:1;
	unsigned int nack_pmu_mask:1;
	unsigned int tr_done_pmu_mask:1;
	unsigned int to_pmu_mask:1;
	unsigned int rsv2:1;
	unsigned int ov_pmu_mask:1;
};
union intunmask_reg_e {
	unsigned int all;
	struct intunmask_bits_s bit;
};

struct pmu_delay_bits_s {
	unsigned int pmu_delay:1;
};
union pmu_delay_reg_e {
	unsigned int all;
	struct pmu_delay_bits_s bit;
};

struct fifo_ctl_bits_s {
	unsigned int rx_full_hold:1;
	unsigned int tx_empty_hold:1;
};
union fifo_ctl_reg_e {
	unsigned int all;
	struct fifo_ctl_bits_s bit;
};

struct x2_i2c_regs_s {
	union cfg_reg_e cfg;
	union saddr_reg_e addr;
	union dcount_reg_e dcount;
	union ctl_reg_e ctl;
	union tdata_reg_e tdata;
	union rdata_reg_e rdata;
	union status_reg_e status;
	union tocnt_reg_e tocnt;
	union sprcpnd_reg_e srcpnd;
	union intmask_reg_e intmask;
	union intsetmask_reg_e intsetmask;
	union intunmask_reg_e intunmask;
	union pmu_delay_reg_e pmu_delay;
	union fifo_ctl_reg_e fifo_ctl;
};

struct client_request{
	u32 client_req_freq;
};
#endif
