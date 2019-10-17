#ifndef ELPSASPA_HW_H_
#define ELPSASPA_HW_H_

/* Memory regions for individual cores */
#define SASPA_PKA_BASE     0x0000
#define SASPA_PKA_SIZE     0x4000
#define SASPA_TRNG_BASE    0x4000
#define SASPA_TRNG_SIZE    0x80
#define SASPA_AES_BASE     0x4080
#define SASPA_AES_SIZE     0x80
#define SASPA_DES_BASE     0x4100
#define SASPA_DES_SIZE     0x80
#define SASPA_RC4_BASE     0x4200
#define SASPA_RC4_SIZE     0x80
#define SASPA_HMAC_BASE    0x4180
#define SASPA_HMAC_SIZE    0x80
#define SASPA_SNOW_BASE    0x4280
#define SASPA_SNOW_SIZE    0x80
#define SASPA_GLOBAL_BASE  0x4800
#define SASPA_GLOBAL_SIZE  0x80
#define SASPA_RC4_CTX_BASE 0x5000
#define SASPA_RC4_CTX_SIZE 0x1000
#define SASPA_MEMORY_BASE  0x6000

/* Global registers */
enum {
   SASPA_GLBL_IRQ_STAT,
   SASPA_GLBL_IRQ_EN,
   SASPA_GLBL_ENDIAN_SWAP,
   SASPA_GLBL_CTX_OFFSET,
   SASPA_GLBL_MSG_OFFSET,
   SASPA_GLBL_EPN_ID,
   SASPA_GLBL_FEATURE1,
   SASPA_GLBL_RESERVED,
   SASPA_GLBL_MAX_MEM_SZ,
};

/* Bit offsets for individual SASPA interrupts. */
#define SASPA_IRQ_GLBL 31
#define SASPA_IRQ_PKA  30
#define SASPA_IRQ_TRNG 27
#define SASPA_IRQ_SNW  22
#define SASPA_IRQ_RC4  21
#define SASPA_IRQ_SHA  20
#define SASPA_IRQ_DES  19
#define SASPA_IRQ_AES  18

/* Aggregate bit mask for interrupts from all symmetric crypto cores. */
#define SASPA_IRQ_CRYPTO_MASK 0x007c0000

/* Bit fields of EPN_ID register. */
#define SASPA_ID_AES       31
#define SASPA_ID_DES       30
#define SASPA_ID_RC4       29
#define SASPA_ID_MD5       28
#define SASPA_ID_SHA1      27
#define SASPA_ID_SHA224    26
#define SASPA_ID_SHA256    25
#define SASPA_ID_SHA384    24
#define SASPA_ID_SHA512    23
#define SASPA_ID_HMAC      23 /* Covers all 6 hashes. */
#define SASPA_ID_HMAC_BITS  6
#define SASPA_ID_PKA       22
#define SASPA_ID_PARS      21
#define SASPA_ID_TRNG      20
#define SASPA_ID_DPRAM     19
#define SASPA_ID_SNOW      18
#define SASPA_ID_EPN        0
#define SASPA_ID_EPN_BITS  16

/* Bit fields of FEATURE1 register. */
#define SASPA_FEAT_AES_128       31
#define SASPA_FEAT_AES_192       30
#define SASPA_FEAT_AES_256       29
#define SASPA_FEAT_AES_ECB       28
#define SASPA_FEAT_AES_CBC       27
#define SASPA_FEAT_AES_CTR       26
#define SASPA_FEAT_AES_CCM       25
#define SASPA_FEAT_AES_XCBC      24
#define SASPA_FEAT_AES_XCBC_1KEY 23
#define SASPA_FEAT_AES_F8        22
#define SASPA_FEAT_AES_GCM       21
#define SASPA_FEAT_AES_CMAC      20
#define SASPA_FEAT_AES_XTS       19
#define SASPA_FEAT_AES_DPA       17
#define SASPA_FEAT_AES_DPA_BITS   2

/* Bit fields of MAX_MEM_SZ register. */
#define SASPA_MAX_MEM_SZ      0
#define SASPA_MAX_MEM_SZ_BITS 4

/* AES registers */
enum {
   AES_CTRL,
   AES_CTRL1,
   AES_CTRL2,
   AES_CTRL3,
   AES_CTRL4,
   AES_STAT,
};

/* Bit fields for AES_CTRL register. */
#define AES_CTRL_MSG_END     10
#define AES_CTRL_MSG_BEGIN   9
#define AES_CTRL_LOAD_IV     8
#define AES_CTRL_STORE_IV    7
#define AES_CTRL_MODE        3
#define AES_CTRL_MODE_BITS   4
#define AES_CTRL_ENCRYPT     2
#define AES_CTRL_KEY_SZ      0
#define AES_CTRL_KEY_SZ_BITS 2

enum {
   AES_MODE_ECB,
   AES_MODE_CBC,
   AES_MODE_CTR,
   AES_MODE_CCM,
   AES_MODE_CMAC,
   AES_MODE_GCM,
   AES_MODE_XCBC,
   AES_MODE_F8 = 9,
   AES_MODE_XTS,
};

enum {
   AES_KEY_SZ_128,
   AES_KEY_SZ_192,
   AES_KEY_SZ_256,
};

/* Bit fields for AES_CTRL1 register. */
#define AES_CTRL1_MAC_LEN      28
#define AES_CTRL1_MAC_LEN_BITS  4
#define AES_CTRL1_BYTES         0
#define AES_CTRL1_BYTES_BITS   12

/*
 * XXX: CTRL2/3/4 are used only for CCM & GCM combined modes and the HW user
 * guide has a number of errors here.  Skip them for now.
 */

/* Bit fields for AES_STAT register. */
#define AES_STAT_MAC_VALID 0

/* DES registers */
enum {
   DES_CTRL,
};

#define DES_CTRL_N_BLKS    8
#define DES_CTRL_STR_IV    5
#define DES_CTRL_RET_IV    4
#define DES_CTRL_RET_KEYS  3
#define DES_CTRL_MODE_3DES 2
#define DES_CTRL_MODE_CBC  1
#define DES_CTRL_ENCRYPT   0

enum { 
   DES_MODE_ECB,
   DES_MODE_CBC
};


/* RC4 registers */
enum {
   SASPA_RC4_KEY0,
   SASPA_RC4_KEY1,
   SASPA_RC4_KEY2,
   SASPA_RC4_KEY3,
   SASPA_RC4_CTRL,
   SASPA_RC4_LEN,
};

#define RC4_CTRL_KEY128 11
#define RC4_CTRL_RESUME  9
#define RC4_CTRL_KEYGEN  8
#define RC4_CTRL_PAGE    0

#define RC4_GO_GENCTX_NOCIPH 2
#define RC4_GO_CIPH_ONLY     1
#define RC4_GO_GENCTX_CIPH   0

/* SNOW 3G registers */
enum {
   SASPA_SNOW_CTRL,
   SASPA_SNOW_CTRL1,
   SASPA_SNOW_STAT,
};

/* HMAC SHA/MD5 registers */
enum {
   SASPA_HMAC_CTRL,
   SASPA_HMAC_NUM_BYTES,
   SASPA_HMAC_TOT_BYTES,
   SASPA_HMAC_SECRET_OFFSET,
   SASPA_HMAC_SECRET_BYTES,
   SASPA_HMAC_HOLE_1,
   SASPA_HMAC_SSLMAC_SEQNUM,
};

#define HMAC_CTRL_ICV_LEN   24
#define HMAC_CTRL_MODE      7
#define HMAC_CTRL_STR_CTX   5
#define HMAC_CTRL_RET_CTX   4
#define HMAC_CTRL_MSG_END   3
#define HMAC_CTRL_MSG_BEGIN 2
#define HMAC_CTRL_SSLMAC    1
#define HMAC_CTRL_HMAC      0

enum { 
   HMAC_MODE_SHA224=0,
   HMAC_MODE_SHA256,
   HMAC_MODE_SHA384,
   HMAC_MODE_SHA512,
   HMAC_MODE_SHA1,
   HMAC_MODE_MD5,
};


#endif
