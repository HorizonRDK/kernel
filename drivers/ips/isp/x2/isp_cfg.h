#ifndef __ISP_CFG_H__
#define __ISP_CFG_H__

#include <linux/types.h>

struct __ISP_CONFIG {
	unsigned int sip_en:1;
	unsigned int reserved:4;
	unsigned int hmp_gma_lut:1;
	unsigned int stf_sgma_lut:1;
	unsigned int stf_ipp_sycc1:1;
	unsigned int stf_cdr:1;
	unsigned int stf_ipp_sycc:1;
	unsigned int stf_ipp_sccm:1;
	unsigned int stf_ipp_sgma_post:1;
	unsigned int stf_ipp_sgma_pre:1;
	unsigned int stf_ipp_sgma:1;
	unsigned int stf_ipp_scb:1;
	unsigned int dpp_sadp_lpf:1;
	unsigned int dpp_gma_post:1;
	unsigned int dpp_gma_pre:1;
	unsigned int dpp_gma:1;
	unsigned int dpp_ycc2:1;
	unsigned int dpp_ycc1:1;
	unsigned int dpp_cb2:1;
	unsigned int dpp_ccm:1;
	unsigned int dpp_cb1:1;
	unsigned int dpp_wbg:1;
	unsigned int dpp_deg:1;
	unsigned int dpp_amp:1;
	unsigned int dpp:1;
	unsigned int img_format:4;
};

struct __DPP_DS {
	unsigned int reserved:8;
	unsigned int dpp_s_7:4;
	unsigned int dpp_s_6:4;
	unsigned int dpp_s_5:4;
	unsigned int dpp_s_4:4;
	unsigned int dpp_s_3:2;
	unsigned int dpp_s_2:2;
	unsigned int dpp_s_1:2;
	unsigned int dpp_s_0:2;
};

struct __HMP_GMA_LUT_SET {
	unsigned int reserved:24;
	unsigned int gma_r3:2;
	unsigned int gma_r2:2;
	unsigned int gma_r1:2;
	unsigned int gma_r0:2;
};

struct __HMP_CDR_ON {
	unsigned int reserved:30;
	unsigned int cdr_c_on:1;
	unsigned int cdr_y_on:1;
};

struct __SFE_MUX {
	unsigned int reserved:28;
	unsigned int tile_sel:1;
	unsigned int grid_sel:1;
	unsigned int hist_sel:1;
	unsigned int rsum_sel:1;
};

struct __W_B_U_4 {
	unsigned int pare_W[4];
	int pare_B[4];
};

struct __W_B_U_3 {
	unsigned int pare_W[3];
	int pare_B[3];
};

struct __W_B_U_1 {
	unsigned int pare_W;
	int pare_B;
};

struct __VIDEO_SIZE {
	unsigned int width:16;
	unsigned int height:16;
};

#pragma pack(4)
struct isp_init_s {

//      struct __VIDEO_SIZE         Video_Size;
	unsigned int DPP_BL[4];

	    //struct __DPP_DS             DPP_DS_S_1;
	unsigned int DPP_DS_S_1;
	struct __W_B_U_4 DPP_AMP;
	unsigned int DPP_LMT[2][4];	// L0,L1,L2,L3 -- H0,H1,H2,H3
	struct __W_B_U_1 HMP_DEG;
	struct __W_B_U_4 HMP_WBG;
	struct __W_B_U_4 HMP_CB1;
	struct __W_B_U_4 HMP_GMA_PRE;
	unsigned int HMP_GMA_LUT_SET;

//      struct __HMP_GMA_LUT_SET    HMP_GMA_LUT_SET;
	unsigned int HMP_GMA_LUT[3][31];
	struct __W_B_U_4 HMP_GMA_POST;
	int HMP_CCM[12];
	struct __W_B_U_3 HMP_CB2;
	int HMP_YCC1[12];
	unsigned int HMP_CDR[17];

	    //struct  __HMP_CDR_ON        HMP_CDR_Y_C_ON_1;
	unsigned int HMP_CDR_Y_C_ON_1;
	unsigned int HMP_CDR_BYPASS;
	int HMP_YCC2[12];
	int STF_SADP_LPF[5];
	unsigned int STF_SADP[3][2];
	struct __W_B_U_3 STF_IPP_SCB;
	struct __W_B_U_3 STF_SGMA;
	unsigned int STF_SGMA_LUT[31];
	struct __W_B_U_3 STF_SGMA_POST;
	int STF_IPP[16];
	unsigned int STF_SFE[36];
	unsigned int addr[6];
	unsigned int SFE_MUX_S;

	    //struct __SFE_MUX            SFE_MUX_S;
	unsigned int IPP_CLIP[3];
	int STF_IPP_SYCC1[4];
	unsigned int HMP_GMA_LUT_L;
	unsigned int HMP_GMA_LUT_H;
	unsigned int HMP_CB1_L;
	unsigned int HMP_CB1_H;
	unsigned int HMP_CCM_CLIP_S;
	unsigned int HMP_CB2_CLIP_S;
	unsigned int HMP_YCC1_CLIP_S;
	unsigned int HMP_IPP_SGMA_CLIP_S;
	unsigned int HMP_IPP_SCCM_CLIP_S;
	unsigned int HMP_IPP_SYCC0_CLIP_S;
	unsigned int HMP_IPP_SYCC1_CLIP_S;
};

#endif	/*  */
