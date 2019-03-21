#ifndef __ISP_DEV_REGS_H__
#define __ISP_DEV_REGS_H__

/* reg address info */
#define BIT_ISP_CONFIG_DONE_EN      (0x01)
#define BIT_ISP_CONFIG_DONE         (0x00)
#define BIT_ISP_EN                  (0x1F)

#define ISP_CONFIG              (0x000)
#define ISP_VIDEO_IN_SIZE       (0x004)

#define DPP_BL_B_0              (0x008)
#define DPP_BL_B_1              (0x00C)
#define DPP_BL_B_2              (0x010)
#define DPP_BL_B_3              (0x014)
#define DPP_DS_S                (0x018)

#define DPP_AMP_W_0             (0x01C)
#define DPP_AMP_W_1             (0x020)
#define DPP_AMP_W_2             (0x024)
#define DPP_AMP_W_3             (0x028)
#define DPP_AMP_B_0             (0x02C)
#define DPP_AMP_B_1             (0x030)
#define DPP_AMP_B_2             (0x034)
#define DPP_AMP_B_3             (0x038)
#define DPP_LMT_L_0             (0x03C)
#define DPP_LMT_L_1             (0x040)
#define DPP_LMT_L_2             (0x044)
#define DPP_LMT_L_3             (0x048)
#define DPP_LMT_H_0             (0x04C)
#define DPP_LMT_H_1             (0x050)
#define DPP_LMT_H_2             (0x054)
#define DPP_LMT_H_3             (0x058)

#define HMP_DEG_W               (0x05C)
#define HMP_DEG_B               (0x060)

#define HMP_WBG_W_0             (0x064)
#define HMP_WBG_W_1             (0x068)
#define HMP_WBG_W_2             (0x06C)
#define HMP_WBG_W_3             (0x070)
#define HMP_WBG_B_0             (0x074)
#define HMP_WBG_B_1             (0x078)
#define HMP_WBG_B_2             (0x07C)
#define HMP_WBG_B_3             (0x080)

#define HMP_CB1_W_0             (0x084)
#define HMP_CB1_W_1             (0x088)
#define HMP_CB1_W_2             (0x08C)
#define HMP_CB1_W_3             (0x090)
#define HMP_CB1_B_0             (0x094)
#define HMP_CB1_B_1             (0x098)
#define HMP_CB1_B_2             (0x09C)
#define HMP_CB1_B_3             (0x0A0)

#define HMP_FMA_PRE_W_0         (0x0A4)
#define HMP_FMA_PRE_W_1         (0x0A8)
#define HMP_FMA_PRE_W_2         (0x0AC)
#define HMP_FMA_PRE_W_3         (0x0B0)
#define HMP_FMA_PRE_B_0         (0x0B4)
#define HMP_FMA_PRE_B_1         (0x0B8)
#define HMP_FMA_PRE_B_2         (0x0BC)
#define HMP_FMA_PRE_B_3         (0x0C0)

#define HMP_GMA_LUT_SEL         (0x0C4)

#define HMP_GMA_LUT1_W_0        (0x0C8)
#define HMP_GMA_LUT1_W_1        (0x0CC)
#define HMP_GMA_LUT1_W_2        (0x0D0)
#define HMP_GMA_LUT1_W_3        (0x0D4)
#define HMP_GMA_LUT1_W_4        (0x0D8)
#define HMP_GMA_LUT1_W_5        (0x0DC)
#define HMP_GMA_LUT1_W_6        (0x0E0)
#define HMP_GMA_LUT1_W_7        (0x0E4)
#define HMP_GMA_LUT1_W_8        (0x0E8)
#define HMP_GMA_LUT1_W_9        (0x0EC)
#define HMP_GMA_LUT1_W_10       (0x0F0)
#define HMP_GMA_LUT1_W_11       (0x0F4)
#define HMP_GMA_LUT1_W_12       (0x0F8)
#define HMP_GMA_LUT1_W_13       (0x0FC)
#define HMP_GMA_LUT1_W_14       (0x100)
#define HMP_GMA_LUT1_W_15       (0x104)
#define HMP_GMA_LUT1_W_16       (0x108)
#define HMP_GMA_LUT1_W_17       (0x10C)
#define HMP_GMA_LUT1_W_18       (0x110)
#define HMP_GMA_LUT1_W_19       (0x114)
#define HMP_GMA_LUT1_W_20       (0x118)
#define HMP_GMA_LUT1_W_21       (0x11C)
#define HMP_GMA_LUT1_W_22       (0x120)
#define HMP_GMA_LUT1_W_23       (0x124)
#define HMP_GMA_LUT1_W_24       (0x128)
#define HMP_GMA_LUT1_W_25       (0x12C)
#define HMP_GMA_LUT1_W_26       (0x130)
#define HMP_GMA_LUT1_W_27       (0x134)
#define HMP_GMA_LUT1_W_28       (0x138)
#define HMP_GMA_LUT1_W_29       (0x13C)
#define HMP_GMA_LUT1_W_30       (0x140)

#define HMP_GMA_LUT2_W_0        (0x144)
#define HMP_GMA_LUT2_W_1        (0x148)
#define HMP_GMA_LUT2_W_2        (0x14C)
#define HMP_GMA_LUT2_W_3        (0x150)
#define HMP_GMA_LUT2_W_4        (0x154)
#define HMP_GMA_LUT2_W_5        (0x158)
#define HMP_GMA_LUT2_W_6        (0x15C)
#define HMP_GMA_LUT2_W_7        (0x160)
#define HMP_GMA_LUT2_W_8        (0x164)
#define HMP_GMA_LUT2_W_9        (0x168)
#define HMP_GMA_LUT2_W_10       (0x16C)
#define HMP_GMA_LUT2_W_11       (0x170)
#define HMP_GMA_LUT2_W_12       (0x174)
#define HMP_GMA_LUT2_W_13       (0x178)
#define HMP_GMA_LUT2_W_14       (0x17C)
#define HMP_GMA_LUT2_W_15       (0x180)
#define HMP_GMA_LUT2_W_16       (0x184)
#define HMP_GMA_LUT2_W_17       (0x188)
#define HMP_GMA_LUT2_W_18       (0x18C)
#define HMP_GMA_LUT2_W_19       (0x190)
#define HMP_GMA_LUT2_W_20       (0x194)
#define HMP_GMA_LUT2_W_21       (0x198)
#define HMP_GMA_LUT2_W_22       (0x19C)
#define HMP_GMA_LUT2_W_23       (0x1A0)
#define HMP_GMA_LUT2_W_24       (0x1A4)
#define HMP_GMA_LUT2_W_25       (0x1A8)
#define HMP_GMA_LUT2_W_26       (0x1AC)
#define HMP_GMA_LUT2_W_27       (0x1B0)
#define HMP_GMA_LUT2_W_28       (0x1B4)
#define HMP_GMA_LUT2_W_29       (0x1B8)
#define HMP_GMA_LUT2_W_30       (0x1BC)

#define HMP_GMA_LUT3_W_0        (0x1C0)
#define HMP_GMA_LUT3_W_1        (0x1C4)
#define HMP_GMA_LUT3_W_2        (0x1C8)
#define HMP_GMA_LUT3_W_3        (0x1CC)
#define HMP_GMA_LUT3_W_4        (0x1D0)
#define HMP_GMA_LUT3_W_5        (0x1D4)
#define HMP_GMA_LUT3_W_6        (0x1D8)
#define HMP_GMA_LUT3_W_7        (0x1DC)
#define HMP_GMA_LUT3_W_8        (0x1E0)
#define HMP_GMA_LUT3_W_9        (0x1E4)
#define HMP_GMA_LUT3_W_10       (0x1E8)
#define HMP_GMA_LUT3_W_11       (0x1EC)
#define HMP_GMA_LUT3_W_12       (0x1F0)
#define HMP_GMA_LUT3_W_13       (0x1F4)
#define HMP_GMA_LUT3_W_14       (0x1F8)
#define HMP_GMA_LUT3_W_15       (0x1FC)
#define HMP_GMA_LUT3_W_16       (0x200)
#define HMP_GMA_LUT3_W_17       (0x204)
#define HMP_GMA_LUT3_W_18       (0x208)
#define HMP_GMA_LUT3_W_19       (0x20C)
#define HMP_GMA_LUT3_W_20       (0x210)
#define HMP_GMA_LUT3_W_21       (0x214)
#define HMP_GMA_LUT3_W_22       (0x218)
#define HMP_GMA_LUT3_W_23       (0x21C)
#define HMP_GMA_LUT3_W_24       (0x220)
#define HMP_GMA_LUT3_W_25       (0x224)
#define HMP_GMA_LUT3_W_26       (0x228)
#define HMP_GMA_LUT3_W_27       (0x22C)
#define HMP_GMA_LUT3_W_28       (0x230)
#define HMP_GMA_LUT3_W_29       (0x234)
#define HMP_GMA_LUT3_W_30       (0x238)

#define HMP_GMA_POST_W_0        (0x23C)
#define HMP_GMA_POST_W_1        (0x240)
#define HMP_GMA_POST_W_2        (0x244)
#define HMP_GMA_POST_W_3        (0x248)
#define HMP_GMA_POST_B_0        (0x24C)
#define HMP_GMA_POST_B_1        (0x250)
#define HMP_GMA_POST_B_2        (0x254)
#define HMP_GMA_POST_B_3        (0x258)

#define HMP_CCM_W_00            (0x25C)
#define HMP_CCM_W_01            (0x260)
#define HMP_CCM_W_02            (0x264)
#define HMP_CCM_W_10            (0x268)
#define HMP_CCM_B_11            (0x26C)
#define HMP_CCM_B_12            (0x270)
#define HMP_CCM_B_20            (0x274)
#define HMP_CCM_B_21            (0x278)
#define HMP_CCM_B_22            (0x27C)

#define HMP_CCM_B_0             (0x280)
#define HMP_CCM_B_1             (0x284)
#define HMP_CCM_B_2             (0x288)

#define HMP_CB2_W_0             (0x28C)
#define HMP_CB2_W_1             (0x290)
#define HMP_CB2_W_2             (0x294)
#define HMP_CB2_B_0             (0x298)
#define HMP_CB2_B_1             (0x29C)
#define HMP_CB2_B_2             (0x2A0)

#define HMP_YCC1_W_00           (0x2A4)
#define HMP_YCC1_W_01           (0x2A8)
#define HMP_YCC1_W_02           (0x2AC)
#define HMP_YCC1_W_10           (0x2B0)
#define HMP_YCC1_w_11           (0x2B4)
#define HMP_YCC1_w_12           (0x2B8)
#define HMP_YCC1_w_20           (0x2BC)
#define HMP_YCC1_w_21           (0x2C0)
#define HMP_YCC1_w_22           (0x2C4)

#define HMP_YCC1_B_0            (0x2C8)
#define HMP_YCC1_B_1            (0x2CC)
#define HMP_YCC1_B_2            (0x2D0)

#define HMP_CDR_OFST_H          (0x2D4)
#define HMP_CDR_OFST_V          (0x2D8)
#define HMP_CDR_SIZE_H          (0x2DC)
#define HMP_CDR_SIZE_V          (0x2E0)
#define HMP_CDR_CONT_H          (0x2E4)
#define HMP_CDR_CONT_V          (0x2E8)

#define HMP_CDR_BINS            (0x2EC)
#define HMP_CDR_BINS_S0         (0x2F0)
#define HMP_CDR_BINS_S1         (0x2F4)

#define HMP_CDR_CLIP_S          (0x2F8)
#define HMP_CDR_CLIP_L          (0x2FC)
#define HMP_CDR_CLIP_H          (0x300)

#define HMP_CDR_ALPHA_0         (0x304)
#define HMP_CDR_ALPHA_1         (0x308)
#define HMP_CDR_ALPHA_2         (0x30C)

#define HMP_CDR_NORM_S          (0x310)
#define HMP_CDR_NORM_DIV        (0x314)

#define HMP_CDR_Y_C_ON          (0x318)
#define HMP_CDR_BYPASS_LINES    (0x31C)

#define HMP_YCC2_W_00           (0x400)
#define HMP_YCC2_W_01           (0x404)
#define HMP_YCC2_W_02           (0x408)
#define HMP_YCC2_W_10           (0x40C)
#define HMP_YCC2_w_11           (0x410)
#define HMP_YCC2_w_12           (0x414)
#define HMP_YCC2_w_20           (0x418)
#define HMP_YCC2_w_21           (0x41C)
#define HMP_YCC2_w_22           (0x420)

#define HMP_YCC2_B_0            (0x424)
#define HMP_YCC2_B_1            (0x428)
#define HMP_YCC2_B_2            (0x42C)

#define STF_SADP_LPF_W_0        (0x430)
#define STF_SADP_LPF_W_1        (0x434)
#define STF_SADP_LPF_W_2        (0x438)
#define STF_SADP_LPF_W_3        (0x43C)
#define STF_SADP_LPF_W_4        (0x440)

#define STF_SADP_ROI_OFST_H     (0x444)
#define STF_SADP_ROI_OFST_V     (0x448)
#define STF_SADP_ROI_STDE_H     (0x44C)
#define STF_SADP_ROI_STDE_V     (0x450)
#define STF_SADP_ROI_SCNT_H     (0x454)
#define STF_SADP_ROI_SCNT_V     (0x458)

#define STF_IPP_SCB_W_0         (0x45C)
#define STF_IPP_SCB_W_1         (0x460)
#define STF_IPP_SCB_W_2         (0x464)
#define STF_IPP_SCB_B_0         (0x468)
#define STF_IPP_SCB_B_1         (0x46C)
#define STF_IPP_SCB_B_2         (0x470)

#define STF_SGMA_PRE_W_0        (0x474)
#define STF_SGMA_PRE_W_1        (0x478)
#define STF_SGMA_PRE_W_2        (0x47C)
#define STF_SGMA_PRE_B_0        (0x480)
#define STF_SGMA_PRE_B_1        (0x484)
#define STF_SGMA_PRE_B_2        (0x488)

#define STF_SGMA_LUT_W_0        (0x48C)
#define STF_SGMA_LUT_W_1        (0x490)
#define STF_SGMA_LUT_W_2        (0x494)
#define STF_SGMA_LUT_W_3        (0x498)
#define STF_SGMA_LUT_W_4        (0x49C)
#define STF_SGMA_LUT_W_5        (0x4A0)
#define STF_SGMA_LUT_W_6        (0x4A4)
#define STF_SGMA_LUT_W_7        (0x4A8)
#define STF_SGMA_LUT_W_8        (0x4AC)
#define STF_SGMA_LUT_W_9        (0x4B0)
#define STF_SGMA_LUT_w_10       (0x4B4)
#define STF_SGMA_LUT_w_11       (0x4B8)
#define STF_SGMA_LUT_w_12       (0x4BC)
#define STF_SGMA_LUT_w_13       (0x4C0)
#define STF_SGMA_LUT_w_14       (0x4C4)
#define STF_SGMA_LUT_w_15       (0x4C8)
#define STF_SGMA_LUT_w_16       (0x4CC)
#define STF_SGMA_LUT_w_17       (0x4D0)
#define STF_SGMA_LUT_w_18       (0x4D4)
#define STF_SGMA_LUT_w_19       (0x4D8)
#define STF_SGMA_LUT_W_20       (0x4DC)
#define STF_SGMA_LUT_W_21       (0x4E0)
#define STF_SGMA_LUT_W_22       (0x4E4)
#define STF_SGMA_LUT_W_23       (0x4E8)
#define STF_SGMA_LUT_W_24       (0x4EC)
#define STF_SGMA_LUT_W_25       (0x4F0)
#define STF_SGMA_LUT_W_26       (0x4F4)
#define STF_SGMA_LUT_W_27       (0x4F8)
#define STF_SGMA_LUT_W_28       (0x4FC)
#define STF_SGMA_LUT_W_29       (0x500)
#define STF_SGMA_LUT_W_30       (0x504)

#define STF_SGMA_POST_W_0       (0x508)
#define STF_SGMA_POST_W_1       (0x50C)
#define STF_SGMA_POST_W_2       (0x510)
#define STF_SGMA_POST_B_0       (0x514)
#define STF_SGMA_POST_B_1       (0x518)
#define STF_SGMA_POST_B_2       (0x51C)

#define STF_IPP_SCCM_W_00       (0x520)
#define STF_IPP_SCCM_W_01       (0x524)
#define STF_IPP_SCCM_W_02       (0x528)
#define STF_IPP_SCCM_W_10       (0x52C)
#define STF_IPP_SCCM_w_11       (0x530)
#define STF_IPP_SCCM_w_12       (0x534)
#define STF_IPP_SCCM_w_20       (0x538)
#define STF_IPP_SCCM_w_21       (0x53C)
#define STF_IPP_SCCM_w_22       (0x540)

#define STF_IPP_SYCC_B_0        (0x544)
#define STF_IPP_SYCC_B_1        (0x548)
#define STF_IPP_SYCC_B_2        (0x54C)
#define STF_IPP_SYCC_W_0        (0x550)
#define STF_IPP_SYCC_W_1        (0x554)
#define STF_IPP_SYCC_W_2        (0x558)

#define STF_IPP_SYCC_B          (0x55C)

#define STF_SFE_CRID_OFST_H     (0x560)
#define STF_SFE_CRID_OFST_V     (0x564)
#define STF_SFE_CRID_SIZE_H     (0x568)
#define STF_SFE_CRID_SIZE_V     (0x56C)
#define STF_SFE_CRID_CONT_H     (0x570)
#define STF_SFE_CRID_CONT_V     (0x574)
#define STF_SFE_CRID_THF_H      (0x578)
#define STF_SFE_CRID_THF_L      (0x57C)

#define STF_SFE_FRID_S          (0x580)
#define STF_SFE_FRID_L          (0x584)
#define STF_SFE_FRID_H          (0x588)

#define STF_SFE_TILE_OFST_H     (0x58C)
#define STF_SFE_TILE_OFST_V     (0x590)
#define STF_SFE_TILE_SIZE_H     (0x594)
#define STF_SFE_TILE_SIZE_V     (0x598)
#define STF_SFE_TILE_CONT_H     (0x59C)
#define STF_SFE_TILE_CONT_V     (0x5A0)
#define STF_SFE_TILE_BINS       (0x5A4)

#define STF_SFE_TILE_S          (0x5A8)
#define STF_SFE_TILE_L          (0x5AC)
#define STF_SFE_TILE_H          (0x5B0)

#define STF_SFE_HIST_OFST_H     (0x5B4)
#define STF_SFE_HIST_OFST_V     (0x5B8)
#define STF_SFE_HIST_SIZE_H     (0x5BC)
#define STF_SFE_HIST_SIZE_V     (0x5C0)
#define STF_SFE_HIST_BINS       (0x5C4)

#define STF_SFE_HIST_S          (0x5C8)
#define STF_SFE_HIST_L          (0x5CC)
#define STF_SFE_HIST_H          (0x5D0)

#define STF_SFE_RSUM_OFST_H     (0x5D4)
#define STF_SFE_RSUM_OFST_V     (0x5D8)
#define STF_SFE_RSUM_SIZE_H     (0x5DC)
#define STF_SFE_RSUM_SIZE_V     (0x5E0)

#define STF_SFE_RSUM_S          (0x5E4)
#define STF_SFE_RSUM_L          (0x5E8)
#define STF_SFE_RSUM_H          (0x5EC)

#define STF_SFE_TILE_ADDR       (0x5F0)
#define STF_SFE_GRID_ADDR       (0x5F4)
#define STF_SFE_RSUM_ADDR       (0x5F8)
#define STF_SFE_HIST_ADDR       (0x5FC)
#define HMP_CDR_ADDR0           (0x600)
#define HMP_CDR_ADDR1           (0x604)

#define SFE_MUX                 (0x608)

#define IPP_CLIP_H              (0x60C)
#define IPP_CLIP_L              (0x610)
#define IPP_CLIP_S              (0x614)

#define STF_IPP_SYCC1_W_0       (0x618)
#define STF_IPP_SYCC1_W_1       (0x61C)
#define STF_IPP_SYCC1_W_2       (0x620)
#define STF_IPP_SYCC1_B         (0x624)

#define HMP_GMA_LUT_CLIP_L      (0x628)
#define HMP_GMA_LUT_CLIP_H      (0x62C)

#define HMP_CB1_CLIP_L          (0x630)
#define HMP_CB1_CLIP_H          (0x634)

#define HMP_CCM_CLIP           (0x638)
#define HMP_CB2_CLIP           (0x63C)
#define HMP_YCC1_CLIP          (0x640)

#define STF_IPP_SGMA_CLIP      (0x644)
#define STF_IPP_SCCM_CLIP      (0x648)
#define STF_IPP_SYCC0_CLIP     (0x64C)
#define STF_IPP_SYCC1_CLIP     (0x650)

#define ISP_CONFIG_DONE        (0x654)

#endif
