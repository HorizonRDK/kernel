#ifndef SPACC_CRYPTOAPI_H_
#define SPACC_CRYPTOAPI_H_

#include <crypto/hash.h>
#include <crypto/aead.h>
#include <crypto/skcipher.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>
#include <crypto/scatterwalk.h>
#include "elppdu.h"
#include "elpspacc.h"

/* Max # of SPAcc devices to look at in our CAPI driver... */
#define ELP_CAPI_MAX_DEV		 8

#define SPACC_MAX_DIGEST_SIZE 64
#define SPACC_MAX_KEY_SIZE    32
#define SPACC_MAX_IV_SIZE     16
#define SPACC_MAX_HASH_BLOCK_SIZE 128

#define SPACC_HASH_STATE_SIZE (SPACC_MAX_DIGEST_SIZE + SPACC_MAX_HASH_BLOCK_SIZE + 64)

#define SPACC_DMA_ALIGN	 4
#define SPACC_DMA_BOUNDARY 0x10000

/* flag means the IV is computed from setkey and crypt */
#define SPACC_MANGLE_IV_FLAG	 0x8000

/* we're doing a CTR mangle (for RFC3686/IPsec) */
#define SPACC_MANGLE_IV_RFC3686 0x0100

/* we're doing GCM */
#define SPACC_MANGLE_IV_RFC4106 0x0200

/* we're doing GMAC */
#define SPACC_MANGLE_IV_RFC4543 0x0300

/* we're doing CCM */
#define SPACC_MANGLE_IV_RFC4309 0x0400

#define dbg(format, x...) printk("%s:%d " format "\n", __func__, __LINE__, ##x)

#define CRYPTO_MODE_AES_CTR_RFC3686 (CRYPTO_MODE_AES_CTR | SPACC_MANGLE_IV_FLAG | SPACC_MANGLE_IV_RFC3686)
#define CRYPTO_MODE_AES_GCM_RFC4106 (CRYPTO_MODE_AES_GCM | SPACC_MANGLE_IV_FLAG | SPACC_MANGLE_IV_RFC4106)
#define CRYPTO_MODE_AES_GCM_RFC4543 (CRYPTO_MODE_AES_GCM | SPACC_MANGLE_IV_FLAG | SPACC_MANGLE_IV_RFC4543)
#define CRYPTO_MODE_AES_CCM_RFC4309 (CRYPTO_MODE_AES_CCM | SPACC_MANGLE_IV_FLAG | SPACC_MANGLE_IV_RFC4309)

struct spacc_crypto_ctx {
	union {
		struct crypto_ahash	 *hash;
		struct crypto_skcipher *cipher;
		struct crypto_aead	  *aead;
	} fb;

	struct device *dev;

	/* save key in setkey and used when set iv, because write_context can't handle NULL key */
	char key[SPACC_MAX_KEY_SIZE];
	int keylen;
	spinlock_t lock;
	struct list_head jobs;
	int handle, mode, auth_size;

	/*
	 * Indicates that the H/W context has been setup and can be used for
	 * crypto; otherwise, the software fallback will be used.
	 */
	bool ctx_valid;

	/* salt used for rfc3686/givencrypt mode */
	unsigned char csalt[16];
};

/* seperate hash reqctx due many members for hash update */
struct spacc_hash_reqctx {
	pdu_ddt src, dst;
	void *digest_buf, *iv_buf;
	dma_addr_t digest_dma, iv_dma, *data_dma;

	int src_nents, dst_nents;

	int digest_mode; /* 1: digest mode, 0: update mode */
	bool last_req;
	bool first_blk;
	bool partial_updating;
	struct completion partial_completion;
	int new_handle;

	struct scatterlist sg[2];
	struct scatterlist sg_hash[2];
	struct scatterlist *reqsrc; /* backup req and recover in complete */
	int nbytes;

	/* below are state info for hash update */
	uint8_t digest[SPACC_MAX_DIGEST_SIZE] __aligned(sizeof(u32));
	int hashlen;
	uint8_t data[SPACC_MAX_HASH_BLOCK_SIZE] __aligned(sizeof(u32));
	uint32_t datalen;
	uint8_t staging_dmabuf[SPACC_MAX_HASH_BLOCK_SIZE] __aligned(sizeof(u32));

	/* The fallback request must be the last member of this struct. */
	union {
		struct ahash_request hash_req;
	} fb;
};

/* spacc_crypto_reqctx is used in cipher and aead */
struct spacc_crypto_reqctx {
	pdu_ddt src, dst;
	void *digest_buf, *iv_buf;
	dma_addr_t digest_dma, iv_dma;
	int fulliv_nents, iv_nents, assoc_nents, src_nents, dst_nents;
	bool partial_processing;
	struct completion partial_completion;
	struct scatterlist *reqsrc;
	struct scatterlist *reqdst;
	int req_cryptlen;


	int new_handle;
};

struct mode_tab {
	char name[48];

	int valid;

	/* mode ID used in hash/cipher mode but not aead */
	int id;

	/* ciph/hash mode used in aead */
	struct {
		int ciph, hash;
	} aead;

	unsigned hashlen, ivlen, blocklen, keylen[3], keylen_mask;

	union {
		unsigned char hash_test[SPACC_MAX_DIGEST_SIZE];
		unsigned char ciph_test[3][2*SPACC_MAX_IV_SIZE];
	};
};

struct spacc_alg {
	struct mode_tab *mode;
	unsigned keylen_mask;

	struct device *dev[ELP_CAPI_MAX_DEV+1];

	struct list_head list;
	struct crypto_alg *calg;

	union {
		struct ahash_alg hash;
		struct skcipher_alg cipher;
		struct aead_alg aead;
	} alg;
};

/*
 * This hack implements SG chaining in a way that works around some
 * limitations of Linux -- the generic sg_chain function fails on ARM, and
 * the scatterwalk_sg_chain function creates chains that cannot be DMA mapped
 * on x86.  So this one is halfway inbetween, and hopefully works in both
 * environments.
 *
 * Unfortunately, if SG debugging is enabled the scatterwalk code will bail
 * on these chains, but it will otherwise work properly.
 */
static inline void spacc_sg_chain(struct scatterlist *sg1, int num,
											 struct scatterlist *sg2)
{
	BUILD_BUG_ON(IS_ENABLED(CONFIG_DEBUG_SG));

	sg_chain(sg1, num, sg2);
	sg1[num-1].page_link |= 1;
}

static inline const struct spacc_alg *spacc_tfm_alg(struct crypto_tfm *tfm)
{
	const struct crypto_alg *calg = tfm->__crt_alg;

	if ((calg->cra_flags & CRYPTO_ALG_TYPE_MASK) == CRYPTO_ALG_TYPE_AHASH) {
		return container_of(calg, struct spacc_alg, alg.hash.halg.base);
	} else if ((calg->cra_flags & CRYPTO_ALG_TYPE_MASK) == CRYPTO_ALG_TYPE_SKCIPHER)
		return container_of(calg, struct spacc_alg, alg.cipher.base);

	return container_of(calg, struct spacc_alg, alg.aead.base);
}

static inline void hexdump(unsigned char *buf, unsigned int len)
{
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
			16, 1, buf, len, false);
}

int spacc_sgs_to_ddt(struct device *dev,
					struct scatterlist *sg1, int len1, int *ents1,
					struct scatterlist *sg2, int len2, int *ents2,
					struct scatterlist *sg3, int len3, int *ents3,
					struct scatterlist *sg4, int len4, int *ents4,
					pdu_ddt *ddt, int dma_direction);

int spacc_sg_to_ddt(struct device *dev, struct scatterlist *sg,
						  int nbytes, pdu_ddt *ddt, int dma_direction);

extern const struct ahash_alg spacc_hash_template;
extern const struct ahash_alg spacc_hmac_template;
extern const struct aead_alg spacc_aead_template;
extern const struct skcipher_alg spacc_cipher_template;

int spacc_hash_module_init(void);
void spacc_hash_module_exit(void);

int spacc_cipher_module_init(void);
void spacc_cipher_module_exit(void);

int spacc_aead_module_init(void);
void spacc_aead_module_exit(void);

#endif
