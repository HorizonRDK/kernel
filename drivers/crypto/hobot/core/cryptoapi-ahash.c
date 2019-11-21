/*
 * This Synopsys software and associated documentation (hereinafter the
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you. The
 * Software IS NOT an item of Licensed Software or a Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Products
 * with Synopsys or any supplement thereto. Synopsys is a registered trademark
 * of Synopsys, Inc. Other names included in the SOFTWARE may be the
 * trademarks of their respective owners.
 *
 * The contents of this file are dual-licensed; you may select either version
 * 2 of the GNU General Public License ("GPL") or the BSD-3-Clause license
 * ("BSD-3-Clause"). The GPL is included in the COPYING file accompanying the
 * SOFTWARE. The BSD License is copied below.
 *
 * BSD-3-Clause License:
 * Copyright (c) 2011-2015 Synopsys, Inc. and/or its affiliates.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions, and the following disclaimer, without
 *	  modification.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 * 3. The names of the above-listed copyright holders may not be used to
 *	  endorse or promote products derived from this software without specific
 *	  prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dmapool.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/md5.h>
#include <crypto/sha.h>
#include <linux/scatterlist.h>

#include "cryptoapi.h"
#include "elpspaccdrv.h"

static int spacc_epn = 0x0;  /* EPN of hardware to look up (TODO: change this to your own core) */
static struct dma_pool *spacc_digest_pool;

static int spacc_count_sg(struct scatterlist *sg, int nbytes);
static int spacc_hash_init_dma(struct device *dev, struct ahash_request *req)
{
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);
	gfp_t mflags = GFP_ATOMIC;
	int rc;

	if (req->base.flags & CRYPTO_TFM_REQ_MAY_SLEEP)
		mflags = GFP_KERNEL;

	ctx->digest_buf = dma_pool_alloc(spacc_digest_pool, mflags, &ctx->digest_dma);

	if (!ctx->digest_buf)
		return -ENOMEM;

	rc = pdu_ddt_init(&ctx->dst, 1|0x80000000);
	if (rc < 0) {
		rc = pdu_error_code(rc);
		goto err_free_digest;
	}
	pdu_ddt_add(&ctx->dst, ctx->digest_dma, SPACC_MAX_DIGEST_SIZE);

	rc = spacc_sg_to_ddt(dev, req->src, req->nbytes, &ctx->src, DMA_TO_DEVICE);
	if (rc < 0)
		goto err_free_dst;

	ctx->src_nents = 0;

	return 0;
err_free_dst:
	pdu_ddt_free(&ctx->dst);
err_free_digest:
	dma_pool_free(spacc_digest_pool, ctx->digest_buf, ctx->digest_dma);
	return rc;
}

static void spacc_hash_cleanup_dma(struct device *dev, struct ahash_request *req)
{
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);

	if (!ctx->partial_updating) {
		dma_unmap_sg(dev, req->src, ctx->src_nents, DMA_TO_DEVICE);
	}

	pdu_ddt_free(&ctx->src);

	dma_pool_free(spacc_digest_pool, ctx->digest_buf, ctx->digest_dma);
	pdu_ddt_free(&ctx->dst);
}

static void spacc_hash_cb(void *spacc, void *data)
{
	struct ahash_request *req = data;
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(tfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int err;

	if (ctx->digest_mode) {
		memcpy(req->result, ctx->digest_buf, ctx->hashlen);
	} else {
		memcpy(ctx->digest, ctx->digest_buf, ctx->hashlen);
		if (ctx->last_req) {
			memcpy(req->result, ctx->digest_buf, ctx->hashlen);
			req->src = ctx->reqsrc;
			req->nbytes = ctx->nbytes;
		}
	}

	spacc_hash_cleanup_dma(tctx->dev, req);

	err = pdu_error_code(priv->spacc.job[ctx->new_handle].job_err);
	spacc_close(&priv->spacc, ctx->new_handle);

	if (err)
		pr_err("%s: err %d\n", __func__, err);

	if (!ctx->digest_mode && ctx->partial_updating)
		complete(&ctx->partial_completion);
	else
		req->base.complete(&req->base, err);
}

static int spacc_hash_close(struct crypto_ahash *tfm)
{
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(tfm);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);

	spacc_close(&priv->spacc, tctx->handle);
	put_device(tctx->dev);
	tctx->handle = -1;
	tctx->dev	 = NULL;

	return 0;
}

static int spacc_hash_open(struct crypto_ahash *tfm)
{
	const struct spacc_alg *salg = spacc_tfm_alg(&tfm->base);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(tfm);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int x;

	if (tctx == NULL) {
		pr_err("%s: tctx == NULL.\n", __func__);
		return -EINVAL;
	}

	/* close handle since key size may have changed */
	if (tctx->handle >= 0)
		spacc_hash_close(tfm);

	for (x = 0; x < ELP_CAPI_MAX_DEV && salg->dev[x]; x++) {
		priv = dev_get_drvdata(salg->dev[x]);
		tctx->dev = get_device(salg->dev[x]);

		tctx->handle = spacc_open(&priv->spacc, CRYPTO_MODE_NULL, salg->mode->id, -1, 0, spacc_hash_cb, tfm);

		if (tctx->handle >= 0) { break; }
		put_device(salg->dev[x]);
	}

	if (tctx->handle < 0) {
		dev_info(salg->dev[0], "failed to open SPAcc context\n");
		return -ENODEV;
	}

	return 0;
}

static int spacc_hash_set_context(struct crypto_ahash *tfm, uint8_t *key, int keylen)
{
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(tfm);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int rc;


	/*
	 * If keylen > hash block len, the key is supposed to be hashed so that it
	 * is less than the block length.  This is kind of a useless property of
	 * HMAC as you can just use that hash as the key directly.  We will just
	 * not use the hardware in this case to avoid the issue.
	 *
	 * This test was meant for hashes but it works for cmac/xcbc since we only intend
	 * to support 128-bit keys...
	 */
	if (keylen > crypto_tfm_alg_blocksize(&tfm->base))
		return 0;

	rc = spacc_write_context(&priv->spacc, tctx->handle, SPACC_HASH_OPERATION, key, keylen, NULL, 0);

	if (rc < 0) {
		dev_warn(tctx->dev, "failed to write SPAcc context %d: %s\n",
								  tctx->handle, spacc_error_msg(rc));
		return -EINVAL;
	}
	rc = spacc_set_operation(&priv->spacc, tctx->handle, OP_ENCRYPT, 0, 0, 0, 0, 0);
	if (rc < 0) {
		pr_err("%s: spacc set operation failed.\n", __func__);
		spacc_hash_close(tfm);
		return rc;
	}

	tctx->ctx_valid = true;

	return 0;
}


static int spacc_hash_queue_req(struct ahash_request *req)
{
	struct crypto_ahash *reqtfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(reqtfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int rc;

	rc = spacc_hash_init_dma(tctx->dev, req);
	if (rc < 0)
		return -EINVAL;

	ctx->new_handle = spacc_clone_handle(&priv->spacc, tctx->handle, req);
	if (ctx->new_handle < 0) {
		spacc_hash_cleanup_dma(tctx->dev, req);
		return -EBUSY;
	}

	rc = spacc_packet_enqueue_ddt(&priv->spacc, ctx->new_handle,
											&ctx->src, &ctx->dst, req->nbytes,
											0, req->nbytes, 0, 0, 0);

	if (rc < 0) {
		spacc_hash_cleanup_dma(tctx->dev, req);
		spacc_close(&priv->spacc, ctx->new_handle);

		if (rc != CRYPTO_FIFO_FULL) {
			dev_err(tctx->dev, "failed to enqueue job: %s\n", spacc_error_msg(rc));
		} else if (!(req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG))
			return -EBUSY;
	}

	return -EINPROGRESS;
}


/*
 * Do don't use update/final to implement digest, since updata/final mode have
 * to add digest due to partial mode not enabled on X3, which cause the digest
 * result is not standard
 */
static int spacc_hash_digest(struct ahash_request *req)
{
	struct crypto_ahash *reqtfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(reqtfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int rc;

	dev_dbg(tctx->dev, "%s (%u)\n", __func__, req->nbytes);

	if (req->nbytes == 0) {
		pr_info("%s: fallback for empty message\n", __func__);
		goto fallback;
	}

	if (tctx->keylen == 0) {
		rc = spacc_hash_open(reqtfm);
		if (rc < 0) {
			pr_err("%s: spacc open failed.\n", __func__);
			return -ENODEV;
		}

		rc = spacc_hash_set_context(reqtfm, NULL, 0);
		if (rc < 0) {
			pr_err("%s: set context failed, rc:%d\n", __func__, rc);
			return rc;
		}
	}

	ctx->digest_mode = 1;

	if (tctx->handle < 0 || !tctx->ctx_valid || req->nbytes > priv->max_msg_len) {
		pr_info("fallback : tctx->handle :%d tctx->ctx_valid:%d\n", tctx->handle, tctx->ctx_valid);
		goto fallback;
	}

	rc = spacc_hash_init_dma(tctx->dev, req);
	if (rc < 0) {
		pr_info("fallback: spacc_hash_init_dma rc :%d\n", rc);
		goto fallback;
	}

	ctx->new_handle = spacc_clone_handle(&priv->spacc, tctx->handle, req);
	if (ctx->new_handle < 0) {
		spacc_hash_cleanup_dma(tctx->dev, req);
		pr_info("fallback:  ctx->new_handle < 0\n");
		goto fallback;
	}

	ctx->hashlen = crypto_ahash_digestsize(reqtfm);

	rc = spacc_packet_enqueue_ddt(&priv->spacc, ctx->new_handle,
								&ctx->src, &ctx->dst, req->nbytes,
								0, req->nbytes, 0, 0, 0);

	if (rc < 0) {
		spacc_hash_cleanup_dma(tctx->dev, req);
		spacc_close(&priv->spacc, ctx->new_handle);

		if (rc != CRYPTO_FIFO_FULL) {
			dev_err(tctx->dev, "failed to enqueue job: %s\n", spacc_error_msg(rc));
		} else if (!(req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG))
			return -EBUSY;

		pr_info("fallback:  spacc_packet_enqueue_ddt failed, rc:%d\n", rc);
		goto fallback;
	}

	return -EINPROGRESS;

fallback:
	dev_info(tctx->dev, "%s: Using SW fallback\n", __func__);

	/* Start from scratch as init is not called before digest. */
	ctx->fb.hash_req.base = req->base;
	ahash_request_set_tfm(&ctx->fb.hash_req, tctx->fb.hash);

	ctx->fb.hash_req.nbytes = req->nbytes;
	ctx->fb.hash_req.src	 = req->src;
	ctx->fb.hash_req.result = req->result;

	return crypto_ahash_digest(&ctx->fb.hash_req);
}

static int spacc_hash_setkey(struct crypto_ahash *tfm, const u8 *key, unsigned keylen)
{
	const struct spacc_alg *salg = spacc_tfm_alg(&tfm->base);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(tfm);
	int rc;

	dev_dbg(salg->dev[0], "%s, keylen: %d\n", __func__, keylen);
	tctx->ctx_valid = false;

	rc = crypto_ahash_setkey(tctx->fb.hash, key, keylen);
	if (rc < 0) {
		pr_err("%s: set key failed, key:%p, keylen:%d\n", __func__, key, keylen);
		return rc;
	}

	rc = spacc_hash_open(tfm);
	if (rc < 0) {
		pr_err("%s: spacc open failed, rc:%d\n", __func__, rc);
		return rc;
	}

	rc = spacc_hash_set_context(tfm, key, keylen);
	if (rc < 0) {
		pr_err("%s: set context failed, rc:%d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static int spacc_hash_cra_init(struct crypto_tfm *tfm)
{
	const struct spacc_alg *salg = spacc_tfm_alg(tfm);
	struct spacc_crypto_ctx *tctx = crypto_tfm_ctx(tfm);
	struct spacc_priv *priv;

	dev_dbg(salg->dev[0], "%s: %s\n", __func__, salg->calg->cra_name);

	tctx->handle    = -1;
	tctx->ctx_valid = false;
	tctx->dev       = get_device(salg->dev[0]);
	tctx->mode   = salg->mode->id;
	tctx->keylen = salg->mode->keylen[0];/* just for check if keylen is 0 */
	priv = dev_get_drvdata(tctx->dev);

	/* allocate an ahash for SW fallback */
	tctx->fb.hash = crypto_alloc_ahash(salg->calg->cra_name, 0, CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(tctx->fb.hash)) {
		if (tctx->handle >= 0)
			spacc_close(&priv->spacc, tctx->handle);
		put_device(tctx->dev);
		return PTR_ERR(tctx->fb.hash);
	}

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
							sizeof (struct spacc_hash_reqctx)
							+ crypto_ahash_reqsize(tctx->fb.hash));

	return 0;
}

static void spacc_hash_cra_exit(struct crypto_tfm *tfm)
{
	struct spacc_crypto_ctx *tctx = crypto_tfm_ctx(tfm);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);

	dev_dbg(tctx->dev, "%s\n", __func__);

	crypto_free_ahash(tctx->fb.hash);

	if (tctx->handle >= 0)
		spacc_close(&priv->spacc, tctx->handle);

	put_device(tctx->dev);
}

/*
 * Due to limitations of the SPAcc, the incremental ahash functions are not
 * supported; only the one-shot mode can work.  For the incremental functions,
 * just pass the state directly to the fallback implementation.
 * This init is only called in update mode
 */
static int spacc_hash_init(struct ahash_request *req)
{
	struct crypto_ahash *reqtfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(reqtfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int rc;

	ctx->hashlen = crypto_ahash_digestsize(crypto_ahash_reqtfm(req));

	dev_dbg(tctx->dev, "%s\n", __func__);

	memset(&ctx->digest, 0, sizeof(ctx->digest));
	memset(&ctx->data, 0, sizeof(ctx->data));
	ctx->datalen = 0;
	ctx->last_req = false;
	ctx->first_blk = true;
	ctx->partial_updating = false;
	init_completion(&ctx->partial_completion);

	/* need to open it for hash mode, hmac mode will open it when setkey*/
	if (tctx->keylen == 0) {
		rc = spacc_hash_open(reqtfm);
		if (rc < 0) {
			pr_err("%s: spacc open failed\n", __func__);
			return -ENODEV;
		}

		rc = spacc_hash_set_context(reqtfm, NULL, 0);
		if (rc < 0) {
			pr_err("%s: set context failed, rc:%d\n", __func__, rc);
			return rc;
		}

		rc = spacc_set_operation(&priv->spacc, tctx->handle, OP_ENCRYPT, 0, 0, 0, 0, 0);
		if (rc < 0) {
			pr_err("%s: spacc set operation failed.\n", __func__);
			spacc_hash_close(reqtfm);
			return rc;
		}
	}

	return 0;
}


static int spacc_count_sg(struct scatterlist *sg, int nbytes)
{
	int i;

	for (i = 0; nbytes > 0 && sg != NULL; i++, sg = sg_next(sg))
		nbytes -= sg->length;

	return i;
}

static size_t spacc_sg_copy_to_buffer(struct scatterlist *sgl,
				unsigned int nents, void *buf, size_t buflen)
{
	int i;
	size_t offset, len;

	for (i = 0, offset = 0; i < nents; ++i) {
		len = sg_copy_to_buffer(sgl, 1, buf, buflen);
		buf += len;
		buflen -= len;
		offset += len;
		sgl = sg_next(sgl);
	}

	return offset;
}

static int spacc_hash_add_digest(struct ahash_request *req)
{
	struct crypto_ahash *reqtfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(reqtfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);

	dev_dbg(tctx->dev, "%s\n", __func__);

	memset(ctx->sg_hash, 0, sizeof(ctx->sg_hash));
	sg_set_buf(&ctx->sg_hash[0], ctx->digest, ctx->hashlen);
	sg_mark_end(&ctx->sg_hash[1]);
	sg_chain(ctx->sg_hash, 2, req->src);
	req->src = ctx->sg_hash;
	req->nbytes += ctx->hashlen;

	return 0;
}


static int __spacc_hash_update(struct ahash_request *req)
{
	struct crypto_ahash *reqtfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(reqtfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int blocksize = crypto_ahash_blocksize(reqtfm);
	uint8_t *pdata;
	uint8_t *staging;
	struct scatterlist *sg_last;
	int datalen;
	int total;
	int nents;
	int ret;
	int padlen;
	int offset;
	int nbytes;
	int num_sg;
	int len;

	dev_dbg(tctx->dev, "%s\n", __func__);

	if (tctx->handle < 0 || !tctx->ctx_valid || req->nbytes > priv->max_msg_len) {
		pr_info("fallback : tctx->handle :%d tctx->ctx_valid:%d\n", tctx->handle, tctx->ctx_valid);
		if (req->result == NULL) {
			pr_info("result pointer is NULL, set to ctx->digest");
			req->result = ctx->digest;
		}
		goto fallback;
	}

	if (req->nbytes == 0) {
		pr_info("%s: fallback when nbytes is 0\n", __func__);
		return 0;
	}

	datalen = ctx->datalen;
	total = req->nbytes + datalen;
	pdata = &ctx->data[0] + datalen;

	if (total <= blocksize) {
		nents = spacc_count_sg(req->src, req->nbytes);
		spacc_sg_copy_to_buffer(req->src, nents, pdata, req->nbytes);
		ctx->datalen = total;

		return 0;
	}

	/* backup and recover in callback */
	ctx->reqsrc = req->src;
	ctx->nbytes = req->nbytes;

	/* change to ALING(ctx->staging_dmabuf, blocksize) if needed */
	staging = ctx->staging_dmabuf;

	memcpy(staging, ctx->data, ctx->datalen);
	pdata = &ctx->data[0];

	padlen = ALIGN(total, blocksize) - total;
	datalen = padlen > 0 ? blocksize - padlen : 0 ;
	offset = req->nbytes - datalen;

	if (offset != req->nbytes)
		scatterwalk_map_and_copy(pdata, req->src, offset, datalen, 0);

	nbytes = total - datalen;

	num_sg = spacc_count_sg(req->src, req->nbytes);

	len = ctx->datalen;
	sg_last = req->src;

	while (len < nbytes && !sg_is_last(sg_last)) {
		if ((len + sg_last->length) > nbytes)
			 break;
		len += sg_last->length;
		sg_last = sg_next(sg_last);
	}

	if (ctx->datalen) {
		sg_mark_end(sg_last);
		memset(ctx->sg, 0, sizeof(ctx->sg));
		sg_set_buf(&ctx->sg[0], staging, ctx->datalen);
		sg_mark_end(&ctx->sg[1]);
		sg_chain(ctx->sg, 2, req->src);
		req->src = ctx->sg;
	} else
		 sg_mark_end(sg_last);

	req->nbytes = nbytes;
	ctx->datalen = datalen;

	if (!ctx->first_blk) {
		spacc_hash_add_digest(req);
	}
	else
		ctx->first_blk = false;

	ctx->src_nents = spacc_count_sg(req->src, req->nbytes);

	ret = spacc_hash_queue_req(req);
	return ret;

fallback:
	/* For some unspported mode such as key longer than blocksize */
	dev_info(tctx->dev, "%s: Using SW fallback\n", __func__);

	/* Start from scratch as init is not called before digest. */
	ctx->fb.hash_req.base = req->base;
	ahash_request_set_tfm(&ctx->fb.hash_req, tctx->fb.hash);

	ctx->fb.hash_req.nbytes = req->nbytes;
	ctx->fb.hash_req.src	 = req->src;
	ctx->fb.hash_req.result = req->result;

	return crypto_ahash_digest(&ctx->fb.hash_req);

}

static int spacc_hash_update(struct ahash_request *req)
{
	struct crypto_ahash *reqtfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(reqtfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	uint32_t max_size = priv->max_msg_len & ~0xfff;
	int ret;
	int total = req->nbytes;
	struct scatterlist *sg_last = NULL;
	struct scatterlist *sg_cur = NULL;
	int len;

	/* backup and recover in callback */
	ctx->reqsrc = req->src;
	ctx->nbytes = req->nbytes;

	ctx->digest_mode = 0;

	if (req->nbytes <= priv->max_msg_len) {
		ret = __spacc_hash_update(req);
		return  ret;
	}

	while(total >= max_size) {
		ctx->partial_updating = true;
		req->nbytes = max_size;
		sg_cur = req->src;
		ret = __spacc_hash_update(req);
		req->src = sg_cur;

		if (ret != -EINPROGRESS) {
			pr_err("%s: partial updating failed %d\n", __func__, ret);
			return ret;
		}

		sg_last = req->src;
		len = 0;
		while (len < max_size) {
			if ((len + sg_last->length) > max_size)
				 break;
			len += sg_last->length;
			sg_last = sg_next(sg_last);
		}
		req->src = sg_last;

		total -= max_size;

		wait_for_completion(&ctx->partial_completion);
		reinit_completion(&ctx->partial_completion);
	}

	ctx->partial_updating = false;
	req->nbytes = total;
	ret = __spacc_hash_update(req);

	return ret;
}

static int spacc_hash_copy_md5_state(struct spacc_hash_reqctx *ctx, void *state, bool import)
{
	struct md5_state *st_md5 = state;

	if (ctx == NULL || state == NULL)
		return -EINVAL;

	if (import) {
		memcpy((uint32_t *)ctx->digest, st_md5->hash, MD5_HASH_WORDS);
		memcpy((uint32_t *)ctx->data, st_md5->block, MD5_BLOCK_WORDS);
		ctx->datalen = st_md5->byte_count <= MD5_BLOCK_WORDS ? st_md5->byte_count : MD5_BLOCK_WORDS<<2;
	} else {
		memcpy(st_md5->hash, (uint32_t *)ctx->digest, MD5_HASH_WORDS);
		memcpy(st_md5->block, (uint32_t *)ctx->data, MD5_BLOCK_WORDS);
		st_md5->byte_count = ctx->datalen;
	}

	return 0;
}

static int spacc_hash_copy_sha1_state(struct spacc_hash_reqctx *ctx, void *state, bool import)
{
	struct sha1_state *st_sha1 = state;

	if (ctx == NULL || state == NULL)
		return -EINVAL;

	if (import) {
		memcpy((uint32_t *)ctx->digest, st_sha1->state, SHA1_DIGEST_SIZE>>2);
		memcpy(ctx->data, st_sha1->buffer, SHA1_BLOCK_SIZE);
		ctx->datalen = st_sha1->count <= SHA1_BLOCK_SIZE ? st_sha1->count : SHA1_BLOCK_SIZE;
	} else {
		memcpy(st_sha1->state, (uint32_t *)ctx->digest, SHA1_DIGEST_SIZE>>2);
		memcpy(st_sha1->buffer, ctx->data, SHA1_BLOCK_SIZE);
		st_sha1->count = ctx->datalen;
	}

	return 0;
}

static int spacc_hash_copy_sha256_state(struct spacc_hash_reqctx *ctx, void *state, bool import)
{
	struct sha256_state *st_sha256 = state;

	if (ctx == NULL || state == NULL)
		return -EINVAL;

	if (import) {
		memcpy((uint32_t *)ctx->digest, st_sha256->state, SHA256_DIGEST_SIZE>>2);
		memcpy(ctx->data, st_sha256->buf, SHA256_BLOCK_SIZE);
		ctx->datalen = st_sha256->count <= SHA256_BLOCK_SIZE ? st_sha256->count : SHA256_BLOCK_SIZE;
	} else {
		memcpy(st_sha256->state, (uint32_t *)ctx->digest, SHA256_DIGEST_SIZE>>2);
		memcpy(st_sha256->buf, ctx->data, SHA256_BLOCK_SIZE);
		st_sha256->count = ctx->datalen;
	}

	return 0;
}

static int spacc_hash_copy_sha512_state(struct spacc_hash_reqctx *ctx, void *state, bool import)
{
	struct sha512_state *st_sha512 = state;

	if (ctx == NULL || state == NULL)
		return -EINVAL;

	if (import) {
		memcpy((uint32_t *)ctx->digest, st_sha512->state, SHA512_DIGEST_SIZE>>2);
		memcpy(ctx->data, st_sha512->buf, SHA512_BLOCK_SIZE);
		ctx->datalen = st_sha512->count[0] <= SHA512_BLOCK_SIZE ? st_sha512->count[0] : SHA512_BLOCK_SIZE;
	} else {
		memcpy(st_sha512->state, (uint32_t *)ctx->digest, SHA512_DIGEST_SIZE>>2);
		memcpy(st_sha512->buf, ctx->data, SHA512_BLOCK_SIZE);
		st_sha512->count[0] = ctx->datalen;
	}

	return 0;
}

static int spacc_hash_copy_state(struct spacc_crypto_ctx *tctx, struct spacc_hash_reqctx *ctx, void *state, bool import)
{
	switch (tctx->mode) {
	case CRYPTO_MODE_HASH_MD5:
	case CRYPTO_MODE_HMAC_MD5:
		spacc_hash_copy_md5_state(ctx, state, import);
		break;

	case CRYPTO_MODE_HASH_SHA1:
	case CRYPTO_MODE_HMAC_SHA1:
		spacc_hash_copy_sha1_state(ctx, state, import);
		break;

	case CRYPTO_MODE_HASH_SHA256:
	case CRYPTO_MODE_HMAC_SHA256:
		spacc_hash_copy_sha256_state(ctx, state, import);
		break;

#if 0
	case CRYPTO_MODE_HASH_SHA384:
	case CRYPTO_MODE_HMAC_SHA384:
		spacc_hash_copy_sha384_state(ctx, state, import);
		break;
#endif

	case CRYPTO_MODE_HASH_SHA512:
	case CRYPTO_MODE_HMAC_SHA512:
		spacc_hash_copy_sha512_state(ctx, state, import);
		break;
	default:
		pr_err("unsupported mode :%d\n", tctx->mode);
	}

	return 0;
}

/*
 * May need to implement import/export later when partial mode is supported
 * on Secure engine, or need to handle the intermediate hash result specially
 *
 */
static int spacc_hash_import(struct ahash_request *req, const void *in)
{
	struct crypto_ahash *reqtfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(reqtfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);
	uint8_t *zero_buf = kzalloc(SPACC_MAX_DIGEST_SIZE, GFP_KERNEL);

	dev_dbg(tctx->dev, "%s\n", __func__);

	if (zero_buf == NULL)
		return -ENOMEM;

	ctx->hashlen = crypto_ahash_digestsize(crypto_ahash_reqtfm(req));

	spacc_hash_copy_state(tctx, ctx, in, true);

	ctx->hashlen = crypto_ahash_digestsize(crypto_ahash_reqtfm(req));

	if (!memcmp(ctx->digest, zero_buf, ctx->hashlen)) {
		ctx->first_blk = true;
	} else
		ctx->first_blk = false;

	kfree(zero_buf);

	return 0;
}

static int spacc_hash_export(struct ahash_request *req, void *out)
{
	struct crypto_ahash *reqtfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(reqtfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);

	dev_dbg(tctx->dev, "%s\n", __func__);

	spacc_hash_copy_state(tctx, ctx, out, false);

	return 0;
}

/* req->nbytes must be 0 in final */
static int spacc_hash_final(struct ahash_request *req)
{
	struct crypto_ahash *reqtfm = crypto_ahash_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_ahash_ctx(reqtfm);
	struct spacc_hash_reqctx *ctx = ahash_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int ret = 0;

	dev_dbg(tctx->dev, "%s\n", __func__);

	if (tctx->handle < 0 || !tctx->ctx_valid || req->nbytes > priv->max_msg_len) {
		pr_info("fallback : tctx->handle :%d tctx->ctx_valid:%d\n", tctx->handle, tctx->ctx_valid);
		goto fallback;
	}

	ctx->last_req = true;

	if (ctx->datalen == 0 && ctx->first_blk) {
		pr_info("%s: fallback for empty plaintext\n", __func__);
		goto fallback;
	}

	if (ctx->datalen == 0) {
		memcpy(req->result, ctx->digest, ctx->hashlen);
		return 0;
	}

	ctx->hashlen = crypto_ahash_digestsize(crypto_ahash_reqtfm(req));

	if (ctx->datalen) {
		memset(ctx->sg, 0, sizeof(ctx->sg));
		memcpy(ctx->staging_dmabuf, ctx->data, ctx->datalen);
		sg_set_buf(&ctx->sg[0], ctx->staging_dmabuf, ctx->datalen);
		sg_mark_end(&ctx->sg[1]);
		req->src = ctx->sg;
		req->nbytes = ctx->datalen;

		memset(ctx->data, 0, ARRAY_SIZE(ctx->data));
		ctx->datalen = 0;

		if (!ctx->first_blk)
		  spacc_hash_add_digest(req);

		ctx->src_nents = spacc_count_sg(req->src, req->nbytes);

		ret = spacc_hash_queue_req(req);
	} else {
		memcpy(req->result, ctx->digest, ctx->hashlen);
		req->base.complete(&req->base, 0);
	}

	return ret;

fallback:
	/* For some unspported mode such as key longer than blocksize */
	dev_info(tctx->dev, "%s: Using SW fallback\n", __func__);

	/* Start from scratch as init is not called before digest. */
	ctx->fb.hash_req.base = req->base;
	ahash_request_set_tfm(&ctx->fb.hash_req, tctx->fb.hash);

	ctx->fb.hash_req.nbytes = req->nbytes;
	ctx->fb.hash_req.src    = req->src;
	ctx->fb.hash_req.result = req->result;

	return crypto_ahash_digest(&ctx->fb.hash_req);

}

const struct ahash_alg spacc_hash_template __devinitconst = {
	.init	= spacc_hash_init,
	.update = spacc_hash_update,
	.final  = spacc_hash_final,
	.import = spacc_hash_import,
	.export = spacc_hash_export,
	.digest = spacc_hash_digest,

	.halg.base = {
		.cra_priority = 300,
		.cra_module	= THIS_MODULE,
		.cra_init	  = spacc_hash_cra_init,
		.cra_exit	  = spacc_hash_cra_exit,
		.cra_ctxsize  = sizeof (struct spacc_crypto_ctx),
		.cra_flags	  = CRYPTO_ALG_TYPE_AHASH
					  | CRYPTO_ALG_ASYNC
					  | CRYPTO_ALG_NEED_FALLBACK,
	},
};

const struct ahash_alg spacc_hmac_template __devinitconst = {
	.init	= spacc_hash_init,
	.update = spacc_hash_update,
	.final  = spacc_hash_final,
	.import = spacc_hash_import,
	.export = spacc_hash_export,
	.digest = spacc_hash_digest,
	.setkey = spacc_hash_setkey,

	.halg.base = {
		.cra_priority = 300,
		.cra_module	  = THIS_MODULE,
		.cra_init	  = spacc_hash_cra_init,
		.cra_exit	  = spacc_hash_cra_exit,
		.cra_ctxsize  = sizeof (struct spacc_crypto_ctx),
		.cra_flags	  = CRYPTO_ALG_TYPE_AHASH
					| CRYPTO_ALG_ASYNC
					| CRYPTO_ALG_NEED_FALLBACK,
	},
};

int __init spacc_hash_module_init(void)
{
	struct platform_device *pdev;

	pdev = get_spacc_platdev_by_epn(spacc_epn, 0);
	if (!pdev) {
		pr_err("could not find SPAcc device (forgot to pass spacc_epn?)\n");
		return -ENODEV;
	}

	spacc_digest_pool = dma_pool_create("spacc-hash-digest", &pdev->dev,
										SPACC_MAX_DIGEST_SIZE,
										SPACC_DMA_ALIGN, SPACC_DMA_BOUNDARY);
	if (!spacc_digest_pool)
		return -ENOMEM;

	return 0;
}

void spacc_hash_module_exit(void)
{
	if (spacc_digest_pool)
		dma_pool_destroy(spacc_digest_pool);
}

module_param (spacc_epn, int, 0);
MODULE_PARM_DESC (spacc_epn, "Set SPAcc EPN number to use");
