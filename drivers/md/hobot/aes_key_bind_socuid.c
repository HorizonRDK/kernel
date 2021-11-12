/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright	2021 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/

#include <linux/crypto.h>
#include <linux/of.h>
#include <linux/scatterlist.h>
#include <crypto/skcipher.h>
#include <crypto/hash.h>

struct ahash_result {
    struct completion ahash_comp;
    int error;
};

#if defined(CONFIG_AES_SOFT_HASH_BIND_KEY)
    char *hash_alg_name = SOFT_SHA256;
#elif defined(CONFIG_AES_HW_HASH_BIND_KEY)
    char *hash_alg_name = HW_SHA256;
#endif

static u8 *append_socuid2_key(const char *socuid,
    u8 *in_key, unsigned int key_len, unsigned int *len)
{
    u8 *append_key;

    *len = (strlen(CONFIG_USE_AES_CONFIG_KEY) ?
        strlen(CONFIG_USE_AES_CONFIG_KEY) : IN_KEY_SIZE) +
            strlen(socuid);
    append_key = (u8 *)kmalloc(*len, GFP_ATOMIC);
    if (!append_key) {
		pr_err("Failed to allocate append_key\n");
		return NULL;
	}

    if (strlen(CONFIG_USE_AES_CONFIG_KEY)) {
        memcpy(append_key, CONFIG_USE_AES_CONFIG_KEY,
            strlen(CONFIG_USE_AES_CONFIG_KEY));
        memcpy(append_key + strlen(CONFIG_USE_AES_CONFIG_KEY),
            socuid, strlen(socuid));
    } else {
        memcpy(append_key, in_key, key_len);
        memcpy(append_key + key_len, socuid, strlen(socuid));
    }
    return append_key;
}

static const char * find_socuid(void)
{
	struct device_node *socinfo_node;
	const char *socuid;
	int ret;

	socinfo_node = of_find_node_by_name(NULL, "socinfo");
    if (!socinfo_node) {
      pr_err("can't find %s node\n", "socinfo");
      return ERR_PTR(-ENODEV);
    }

	ret = of_property_read_string(socinfo_node, "socuid",
	    &socuid);
	if (ret != 0) {
		pr_err("of_property_read_string error\n");
		return  ERR_PTR(-ENODEV);
	}
    return socuid;
}

static void ahash_complete(struct crypto_async_request *req, int error)
{
    struct ahash_result *res = req->data;
	if (error == -EINPROGRESS)
		return;

	res->error = error;
	complete(&res->ahash_comp);
}

static int calc_hash(struct crypto_ahash  *alg,
             const u8 *data, unsigned int *datalen,
             u8 *digest, const char *algo_name)
{
    struct ahash_request *req;
    struct scatterlist sg;
    int ret;
    struct ahash_result calc_result;

    init_completion(&calc_result.ahash_comp);
    req = ahash_request_alloc(alg, GFP_KERNEL);
    if (!req) {
		pr_err("alg: hash: Failed to alloc request for %s\n", algo_name);
		return -ENOMEM;
	}
    ahash_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
        ahash_complete, &calc_result);
    /* init  scatterlist */
    sg_init_one(&sg, data, *datalen);
    ahash_request_set_crypt(req, &sg, digest, *datalen);

    ret = crypto_ahash_digest(req);
    switch (ret) {
		case 0:
			break;
		case -EINPROGRESS:
		case -EBUSY:
			wait_for_completion(&calc_result.ahash_comp);
			reinit_completion(&calc_result.ahash_comp);
			ret = calc_result.error;
			if (!ret)
				break;
		/* faild into */
		default:
			pr_err("ret:%d hash %s: digest failed\n", ret, algo_name);
			goto error;
    }
    ahash_request_free(req);
	crypto_free_ahash(alg);
    return 0;
error:
    return ret;
}

void aes_key_bind_socuid(u8 *in_key, unsigned int key_len)
{
    int ret;
    const char *algo_name;
    const u8 *socuid_tmp, *socuid;
    size_t digest_size;
    u8 *digest, *append_key;
    struct crypto_ahash *alg;
    unsigned int len;
    socuid = (const u8 *)kmalloc(SOCUID_SIZE + 1, GFP_ATOMIC);

    if (!socuid) {
		pr_err("Failed to allocate socuid\n");
        return;
	}

    socuid_tmp = find_socuid();
    if (IS_ERR(socuid_tmp))
        goto error;
    memcpy((u8 *)socuid, socuid_tmp, SOCUID_SIZE + 1);

    alg = crypto_alloc_ahash(hash_alg_name, CRYPTO_ALG_TYPE_AHASH, 0);
    if (IS_ERR(alg)) {
        pr_err("can't alloc alg %s\n", hash_alg_name);
        goto error;
    }
    algo_name = crypto_ahash_driver_name(alg);
    digest_size = crypto_ahash_digestsize(alg);

    digest = (unsigned char *)kmalloc(digest_size, GFP_ATOMIC);
    if (!digest) {
        pr_err("Failed to alloc digest\n");
        return;
    }
    memset(digest, 0 , digest_size);
    /* append socuid to in_key  */
    append_key = append_socuid2_key(socuid, in_key, key_len, &len);
    if (!append_key) {
        kfree(digest);
        goto error;
    }

    /* caculate hash */
    ret = calc_hash(alg, append_key,
        &len, digest, algo_name);
    if (!ret)
        memcpy(in_key, digest, digest_size);
    else
        pr_err("Failed to bind aes key use source in_key\n");
    kfree(append_key);
    kfree(digest);
    kfree(socuid);
    return;
error:
    kfree(socuid);
    return;
}
EXPORT_SYMBOL_GPL(aes_key_bind_socuid);

