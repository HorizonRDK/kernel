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

#ifndef __HOBOT_EFUSE_H__
#define __HOBOT_EFUSE_H__

enum EFS_TPE {
	EFS_NS,
	EFS_S
};

/********************************************
 * define secure efuse field
 ********************************************
*/
#define E_BNK_SEC_CFG   0

#define E_BNK_DIS_DEBUG  E_BNK_SEC_CFG
#define E_BIT_DIS_DEBUG  0

#define E_BNK_SECURE_BIT  E_BNK_SEC_CFG
#define E_BIT_SECURE_BIT  1

#define E_BNK_XIP_BOOT  E_BNK_SEC_CFG
#define E_BIT_XIP_BOOT  3

#define E_BNK_NS_DIS_ASSET  E_BNK_SEC_CFG
#define E_BIT_NS_DIS_ASSET  4

#define SYSCTL_REG_BASE	0xA1000900
#define SYSCTL_REG_LEN  0x20

#define HW_EFS_CFG	  0x0
#define HW_EFS_BANK_ADDR	0x4
#define HW_EFS_PROG_DATA	0x8
#define HW_EFS_READ_DATA	0xc
#define HW_EFS_STATUS	  0x10
#define HW_EFS_SW_RESET	  0x14

#define HW_EFUSE_READ_OK	  1
#define HW_EFUSE_READ_UNLOCK	  2
#define HW_EFUSE_READ_FAIL	  3
#define HW_EFUSE_WRITE_LOCKED_BNK   4
#define HW_EFUSE_WRITE_FAIL   5
#define HW_EFUSE_LOCK_BNK	  31
#define HW_EFS_CFG_EFS_EN	  (1 << 31)
#define HW_EFS_CFG_EFS_PROG_RD	  (1 << 1)
#define HW_EFS_CFG_EFS_PROG_WR	  (3)
#define HW_EFS_EFS_RDY			(1)
#define HW_EFS_EFS_RDY_PASS		(7)

#define EFUSE_NS_OFF		0x0
#define NS_EFS_RDONE		0x80
#define EFUSE_S_OFF		0x100
#define S_EFS_RDONE		0x180
#define S_EFS_BAK1_OFF	0x4

#define NS_EFUSE_NUM		32
#define S_EFUSE_NUM			32
#define EFUSE_PROG_NUM		31

#define SYSCTL_DUMMY0		0x700
#define SYSCTL_LOCKDOWN_SECURE	  0x900
#define SYSCTL_MSK_AES_KEY	  0x904
#define SYSCTL_MSK_ROM		0x908
#define SYSCTL_DBG_PORT_EN_LOCK	  0x90c
#define SYSCTL_DBG_PORT_EN	  0x910
#define SYSCTL_S_EFS_LOCK	  0x914

#define TEST_MODE_SHIFT		31
#define EFS_RSA_SEL_SHIFT	  24
#define EFS_RSA_SEL_MASK	  0x1F000000

#define RPU_BIFSPI_SLV_DIS	  0x400
#define RPU_BIFSD_SLV_DIS	  0x404
#define RPU_DEFAULT_ADDR	  0x480

#define SEFUSE_BNK_CFG		0
#define SEFUSE_BNK_LOCK		31
#define SEFUSE_SECURE_CHIP	  (1<<1)
#define SEFUSE_NON_SECURE_CHIP	  (1<<4)

#define SEFUSE_LOCK_AES_MSK	  0x3c0
#define SEFUSE_LOCK_RSA_HASH_MSK	  0x3FC000

#define SEFUSE_DIS_SEC_HW_BASE	(SEC_REG_BASE+EFUSE_S_OFF+S_EFS_BAK1_OFF)

#define RET_API_EFUSE_OK    0
#define RET_API_EFUSE_FAIL  0xffffffff

#define AES_UNIQUE_KEY_31_0_BANK    10
#define AES_UNIQUE_KEY_63_32_BANK   11
#define AES_UNIQUE_KEY_95_64_BANK   12
#define AES_UNIQUE_KEY_127_96_BANK  13
#define PRODUCT_CUSTOMER_ID_BANK    26

/********************************************
 * define non-secure efuse field
 ********************************************
*/
#define NSEFUSE_BNK_SWCFG				23
#define NSEFUSE_BNK_USB2_UTMI_CFG		24

#define RSA_PUB_KEY_SIZE 256
#define RSA_SIGN_SIZE  256
#define AES_KEY_SIZE 16
#define AES_IV_SIZE 16

#define SPL_HDR_WITHOUT_SIGH_H 576
#define SPL_HDR_WITH_SIGH_H (SPL_HDR_WITHOUT_SIGH_H + SPL_HEADER_H_OFF)
#define SPL_OFF_OFF   0
#define SPL_OFF_SIZE  16
#define SPL_HEADER_H_OFF   RSA_SIGN_SIZE
#define SPL_HEADER_I_OFF   RSA_SIGN_SIZE
#define SPL_HEADER_K_OFF   RSA_SIGN_SIZE
#define SPL_IMG_SZ_OFF  (SPL_IMG_LD_OFF + 4)
#define SPL_KB_LD_OFF   (SPL_IMG_SZ_OFF + 4)
#define SPL_KB_SZ_OFF   (SPL_KB_LD_OFF + 4)
#define SHA_256_SIZE   32
#define SHA_256_BLK_SIZE   64
#define RSA_VERIFY_OFF 0x1e0

//MPU register define
#define MPU_BPU0_FETCH_S_RANGE        0x200
#define MPU_BPU0_FETCH_E_RANGE        0x204
#define MPU_BPU1_FETCH_S_RANGE        0x208
#define MPU_BPU1_FETCH_E_RANGE        0x20c
#define MPU_DRAM_BPU_S_RANGE          0x210
#define MPU_DRAM_BPU_E_RANGE          0x214
#define MPU_DRAM_MAX_S_RANGE          0x218
#define MPU_DRAM_MAX_E_RANGE          0x21c
#define MPU_DRAM_BPU_MAX_S_RANGE      0x220
#define MPU_DRAM_BPU_MAX_E_RANGE      0x224
#define MPU_BPU0_FETCH_USER           0x240
#define MPU_BPU1_FETCH_USER           0x244
#define MPU_DRAM_BPU_USER             0x248
#define MPU_DRAM_MAX_USER             0x24c
#define MPU_DRAM_BPU_MAX_USER         0x250
#define MPU_S_RANGE0                  0x300
#define MPU_E_RANGE0                  0x304
#define MPU_S_RANGE1                  0x308
#define MPU_E_RANGE1                  0x30c
#define MPU_S_RANGE2                  0x310
#define MPU_E_RANGE2                  0x314
#define MPU_S_RANGE3                  0x318
#define MPU_E_RANGE3                  0x31c
#define MPU_RANGE0_RUSER              0x320
#define MPU_RANGE1_RUSER              0x324
#define MPU_RANGE2_RUSER              0x328
#define MPU_RANGE3_RUSER              0x32c
#define MPU_RANGE0_WUSER              0x330
#define MPU_RANGE1_WUSER              0x334
#define MPU_RANGE2_WUSER              0x338
#define MPU_RANGE3_WUSER              0x33c
#define MPU_DEFAULT_ADDR              0x380

#define		SW_EFUSE_FAIL	0xffffffff
#define		SW_EFUSE_OK		0x0
#define		SW_EFUSE_AES_KEY_BANK_LOCK		1
#define		SW_EFUSE_AES_KEY_BANK_NOLOCK	0
#define		SW_EFUSE_RSA_HASH_BANK_LOCK		1
#define		SW_EFUSE_RSA_HASH_BANK_NOLOCK	0
#define		SCOMP_VERIFY_OK			0
#define		SCOMP_VERIFY_FAIL		-1

#define MPU_DRAM_BPU_MAX_S_RANGE_1G 0x40000000
#define MPU_DRAM_BPU_MAX_E_RANGE_1G 0x80000000

#define SCOMP_MMU_ENABLE 0
#define CPU_READ	0
#define CPU_WRITE	1

enum FW_USR_ID {
    FW_USR_ID_CPU,              //0
    FW_USR_ID_CR5,
    FW_USR_ID_BIFSPI,
    FW_USR_ID_BIFSD,
    FW_USR_ID_DMAC,
    FW_USR_ID_USB,              //5
    FW_USR_ID_AES,
    FW_USR_ID_RSA,
    FW_USR_ID_CNN0_Fetch = 0xc, //12
    FW_USR_ID_CNN1_Fetch,
    FW_USR_ID_CNN0_Other,
    FW_USR_ID_CNN1_Other,
    FW_USR_ID_UART0,
    FW_USR_ID_UART1,            //17
    FW_USR_ID_UART2,
    FW_USR_ID_UART3,
    FW_USR_ID_SPI0,
    FW_USR_ID_SPI1,
    FW_USR_ID_SPI2,             //22
    FW_USR_ID_I2S0,
    FW_USR_ID_I2S1,
    FW_USR_ID_SDIO0_1_2,
    FW_USR_ID_GMAC,
    FW_USR_ID_RESERVED,         //27
    FW_USR_ID_VIO_M0,
    FW_USR_ID_VPU,
    FW_USR_ID_VIO_M1_JPG,       //30
    FW_USR_ID_FW_USR_ID_MAX
};

typedef struct {
	uint8_t						signature_h[RSA_SIGN_SIZE];
	uint8_t						signature_i[RSA_SIGN_SIZE];
	uint32_t					image_load_addr;
	uint32_t					image_len;
	uint32_t					bpu0_fetch_only_start;
	uint32_t					bpu0_fetch_only_sz;
	uint32_t					bpu1_fetch_only_start;
	uint32_t					bpu1_fetch_only_sz;
	uint32_t					bpu_range_start;
	uint32_t					bpu_range_sz;
} bpu_img_header;

typedef struct {
	uint8_t *					p_sign_h;
	uint8_t *					p_sign_i;
	uint8_t						digest_h[SHA_256_SIZE];
	uint8_t						digest_i[SHA_256_SIZE];
	uint32_t					algo_aes;
	uint32_t					algo_hash;
	uint8_t *					aes_key;
	uint8_t *					aes_iv;
	uint8_t *					pubkey;
	uint32_t					image_load_addr;
	uint32_t					image_len;
	uint32_t					hashed_hdr_len;
} auth_img;

//********************************************
//* efuse data structure
//********************************************
typedef struct {
    unsigned int reserved;
}efs_bank_reserved;

typedef struct {
    unsigned int sec_cfg;
}sefs_bank_0;

typedef struct {
    unsigned int bond_option;
}sefs_bank_1;

typedef struct {
    unsigned int unique_id_31_0;
}sefs_bank_2;

typedef struct {
    unsigned int unique_id_63_32;
}sefs_bank_3;

typedef struct {
    unsigned int unique_id_95_64;
}sefs_bank_4;

typedef struct {
    unsigned int unique_id_127_96;
}sefs_bank_5;

typedef struct {
    unsigned int aes_uniform_key_31_0;
}sefs_bank_6;

typedef struct {
    unsigned int aes_uniform_key_63_32;
}sefs_bank_7;

typedef struct {
    unsigned int aes_uniform_key_95_64;
}sefs_bank_8;

typedef struct {
    unsigned int aes_uniform_key_127_96;
}sefs_bank_9;

typedef struct {
    unsigned int aes_unique_key_31_0;
}sefs_bank_10;

typedef struct {
    unsigned int aes_unique_key_63_32;
}sefs_bank_11;

typedef struct {
    unsigned int aes_unique_key_95_64;
}sefs_bank_12;

typedef struct {
    unsigned int aes_unique_key_127_96;
}sefs_bank_13;

typedef struct {
    unsigned int ras_pub_root_hash_31_0;
}sefs_bank_14;

typedef struct {
    unsigned int ras_pub_root_hash_63_32;
}sefs_bank_15;

typedef struct {
    unsigned int ras_pub_root_hash_95_64;
}sefs_bank_16;

typedef struct {
    unsigned int ras_pub_root_hash_127_96;
}sefs_bank_17;

typedef struct {
    unsigned int ras_pub_root_hash_159_128;
}sefs_bank_18;

typedef struct {
    unsigned int ras_pub_root_hash_191_160;
}sefs_bank_19;

typedef struct {
    unsigned int ras_pub_root_hash_223_192;
}sefs_bank_20;

typedef struct {
    unsigned int ras_pub_root_hash_255_224;
}sefs_bank_21;

typedef struct {
    unsigned int aes_bpu_key_31_0;
}sefs_bank_22;

typedef struct {
    unsigned int aes_bpu_key_63_32;
}sefs_bank_23;

typedef struct {
    unsigned int aes_bpu_key_95_64;
}sefs_bank_24;

typedef struct {
    unsigned int aes_bpu_key_127_96;
}sefs_bank_25;

typedef struct {
    short customer_id;  /// [15: 0]
    char product_id;    /// [23:16]
    char reserved;      /// [31:24]
}sefs_bank_26;

typedef struct {
    unsigned int bank_lock_0:1;
    unsigned int bank_lock_1:1;
    unsigned int bank_lock_2:1;
    unsigned int bank_lock_3:1;
    unsigned int bank_lock_4:1;
    unsigned int bank_lock_5:1;
    unsigned int bank_lock_6:1;
    unsigned int bank_lock_7:1;
    unsigned int bank_lock_8:1;
    unsigned int bank_lock_9:1;
    unsigned int bank_lock_10:1;
    unsigned int bank_lock_11:1;
    unsigned int bank_lock_12:1;
    unsigned int bank_lock_13:1;
    unsigned int bank_lock_14:1;
    unsigned int bank_lock_15:1;
    unsigned int bank_lock_16:1;
    unsigned int bank_lock_17:1;
    unsigned int bank_lock_18:1;
    unsigned int bank_lock_19:1;
    unsigned int bank_lock_20:1;
    unsigned int bank_lock_21:1;
    unsigned int bank_lock_22:1;
    unsigned int bank_lock_23:1;
    unsigned int bank_lock_24:1;
    unsigned int bank_lock_25:1;
    unsigned int bank_lock_26:1;
    unsigned int bank_lock_27:1;
    unsigned int bank_lock_28:1;
    unsigned int bank_lock_29:1;
    unsigned int bank_lock_30:1;
    unsigned int bank_lock_31:1;
}efs_bank_lock;


typedef struct {
    union {
        sefs_bank_0  bank;
        unsigned int data;
    } bnk_0;
    union {
        sefs_bank_1  bank;
        unsigned int data;
    }  bnk_1;
    union {
        sefs_bank_2  bank;
        unsigned int data;
    }  bnk_2;
    union {
        sefs_bank_3  bank;
        unsigned int data;
    }  bnk_3;
    union {
        sefs_bank_4  bank;
        unsigned int data;
    }  bnk_4;
    union {
        sefs_bank_5  bank;
        unsigned int data;
    }  bnk_5;
    union {
        sefs_bank_6  bank;
        unsigned int data;
    }  bnk_6;
    union {
        sefs_bank_7  bank;
        unsigned int data;
    }  bnk_7;
    union {
        sefs_bank_8  bank;
        unsigned int data;
    }  bnk_8;
    union {
        sefs_bank_9  bank;
        unsigned int data;
    }  bnk_9;
    union {
        sefs_bank_10 bank;
        unsigned int data;
    }  bnk_10;
    union {
        sefs_bank_11 bank;
        unsigned int data;
    }  bnk_11;
    union {
        sefs_bank_12 bank;
        unsigned int data;
    }  bnk_12;
    union {
        sefs_bank_13 bank;
        unsigned int data;
    }  bnk_13;
    union {
        sefs_bank_14 bank;
        unsigned int data;
    }  bnk_14;
    union {
        sefs_bank_15 bank;
        unsigned int data;
    }  bnk_15;
    union {
        sefs_bank_16 bank;
        unsigned int data;
    }  bnk_16;
    union {
        sefs_bank_17 bank;
        unsigned int data;
    }  bnk_17;
    union {
        sefs_bank_18 bank;
        unsigned int data;
    }  bnk_18;
    union {
        sefs_bank_19 bank;
        unsigned int data;
    }  bnk_19;
    union {
        sefs_bank_20 bank;
        unsigned int data;
    }  bnk_20;
    union {
        sefs_bank_21 bank;
        unsigned int data;
    }  bnk_21;
    union {
        sefs_bank_22 bank;
        unsigned int data;
    }  bnk_22;
    union {
        sefs_bank_23 bank;
        unsigned int data;
    }  bnk_23;
    union {
        sefs_bank_24 bank;
        unsigned int data;
    }  bnk_24;
    union {
        sefs_bank_25 bank;
        unsigned int data;
    }  bnk_25;
    union {
        sefs_bank_26 bank;
        unsigned int data;
    }  bnk_26;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_27;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_28;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_29;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_30;
    union {
        efs_bank_lock bank;
        unsigned int  data;
    }  bnk_31;
}secure_efuse;

typedef struct {
    short pvt_ts0_b;    /// [15: 0]
    short pvt_ts0_a;    /// [31:16]
}nefs_bank_0;

typedef struct {
    short pvt_ts1_b;    /// [15: 0]
    short pvt_ts1_a;    /// [31:16]
}nefs_bank_1;

typedef struct {
    short pvt_ts2_b;    /// [15: 0]
    short pvt_ts2_a;    /// [31:16]
}nefs_bank_2;

typedef struct {
    short pvt_ts3_b;    /// [15: 0]
    short pvt_ts3_a;    /// [31:16]
}nefs_bank_3;

typedef struct {
    short pvt_vm_n0:12;     /// [11: 0]
    short reserved:4;       /// [15:12]
    short pvt_vm_k3;        /// [31:16]
}nefs_bank_4;

typedef struct {
    unsigned int armpll_trim;
}nefs_bank_5;

typedef struct {
    unsigned int syspll_trim;
}nefs_bank_6;

typedef struct {
    unsigned int cnpll_trim;
}nefs_bank_7;

typedef struct {
    unsigned int ddrpll_trim;
}nefs_bank_8;

typedef struct {
    unsigned int peripll_trim;
}nefs_bank_9;

typedef struct {
    unsigned int viopll_trim;
}nefs_bank_10;

typedef struct {
    unsigned int viopll2_trim;
}nefs_bank_11;

typedef struct {
    unsigned int soft_cfg;
}nefs_bank_23;

typedef struct {
    unsigned int usb_cfg;
}nefs_bank_24;

typedef struct {
    short vdd_cpu_leakage_ua:10;    /// [ 9: 0]
    short vdd_cpu_leakage_ma:6;     /// [15:10]
    short vdd_ddr_leakage_ua:10;    /// [25:16]
    short vdd_ddr_leakage_ma:6;     /// [31:26]
}nefs_bank_25;

typedef struct {
    short vdd_cnn0_leakage_ua:10;   /// [ 9: 0]
    short vdd_cnn0_leakage_ma:6;    /// [15:10]
    short vdd_cnn1_leakage_ua:10;   /// [25:16]
    short vdd_cnn1_leakage_ma:6;    /// [31:26]
}nefs_bank_26;

typedef struct {
    short vdd_core_ao_leakage_ua:10;    /// [ 9: 0]
    short vdd_core_ao_leakage_ma:6;     /// [15:10]
    short vdd_core_pd_leakage_ua:10;    /// [25:16]
    short vdd_core_pd_leakage_ma:6;     /// [31:26]
}nefs_bank_27;

typedef struct {
    char chip_id0:4;    ///[ 3: 0]
    char chip_id1:4;    ///[ 4: 7]
    char chip_id2:4;    ///[11: 8]
    char chip_id3:4;    ///[15:12]
    char chip_id4:4;    ///[19:16]
    char chip_id5:4;    ///[23:20]
    char chip_id6:4;    ///[27:24]
    char chip_id7:4;    ///[31:18]
}nefs_bank_28;

typedef struct {
    unsigned int lotid_2:6;     /// [ 5: 0]
    unsigned int lotid_3:6;     /// [11: 6]
    unsigned int lotid_4:6;     /// [17:12]
    unsigned int lotid_5:6;     /// [23:18]
}nefs_bank_29;

typedef struct {
    unsigned int x:7;           /// [ 6: 0]
    unsigned int y:7;           /// [13: 7]
    unsigned int wafer_id:5;    /// [18:14]
    unsigned int lotid_0:6;     /// [24:19]
    unsigned int lotid_1:6;     /// [30:25]
}nefs_bank_30;

typedef struct {
    union {
        nefs_bank_0  bank;
        unsigned int data;
    } bnk_0;
    union {
        nefs_bank_1  bank;
        unsigned int data;
    }  bnk_1;
    union {
        nefs_bank_2  bank;
        unsigned int data;
    }  bnk_2;
    union {
        nefs_bank_3  bank;
        unsigned int data;
    }  bnk_3;
    union {
        nefs_bank_4  bank;
        unsigned int data;
    }  bnk_4;
    union {
        nefs_bank_5  bank;
        unsigned int data;
    }  bnk_5;
    union {
        nefs_bank_6  bank;
        unsigned int data;
    }  bnk_6;
    union {
        nefs_bank_7  bank;
        unsigned int data;
    }  bnk_7;
    union {
        nefs_bank_8  bank;
        unsigned int data;
    }  bnk_8;
    union {
        nefs_bank_9  bank;
        unsigned int data;
    }  bnk_9;
    union {
        nefs_bank_10 bank;
        unsigned int data;
    }  bnk_10;
    union {
        nefs_bank_11 bank;
        unsigned int data;
    }  bnk_11;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_12;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_13;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_14;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_15;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_16;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_17;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_18;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_19;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_20;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_21;
    union {
        efs_bank_reserved bank;
        unsigned int data;
    }  bnk_22;
    union {
        nefs_bank_23 bank;
        unsigned int data;
    }  bnk_23;
    union {
        nefs_bank_24 bank;
        unsigned int data;
    }  bnk_24;
    union {
        nefs_bank_25 bank;
        unsigned int data;
    }  bnk_25;
    union {
        nefs_bank_26 bank;
        unsigned int data;
    }  bnk_26;
    union {
        nefs_bank_27 bank;
        unsigned int data;
    }  bnk_27;
    union {
        nefs_bank_28 bank;
        unsigned int data;
    }  bnk_28;
    union {
        nefs_bank_29 bank;
        unsigned int data;
    }  bnk_29;
    union {
        nefs_bank_30 bank;
        unsigned int data;
    }  bnk_30;
    union {
        efs_bank_lock bank;
        unsigned int  data;
    }  bnk_31;
}normal_efuse;

typedef struct {
    normal_efuse nefs_data;
    secure_efuse sefs_data;
    unsigned int nefs_mask;
    unsigned int sefs_mask;
} efuse_data;

#endif  //  __HOBOT_EFUSE_H__
