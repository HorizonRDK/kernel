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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/crypto.h>

#include "cryptoapi.h"
#include "elpspaccdrv.h"

static int spacc_epn = 0x0;  /* EPN of hardware to look up (TODO: change this to your own core) */

static int spacc_cipher_init_dma(struct device *dev, struct skcipher_request *req)
{
	struct spacc_crypto_reqctx *ctx = skcipher_request_ctx(req);
	int rc;

	rc = spacc_sg_to_ddt(dev, req->src, req->cryptlen, &ctx->src, DMA_TO_DEVICE);
	if (rc < 0) {
		pr_err("%s spacc_sg_to_ddt for src failed, src:%p, cryptlen:%d\n",
			__func__, req->src, req->cryptlen);
		return rc;
	}
	ctx->src_nents = rc;

	rc = spacc_sg_to_ddt(dev, req->dst, req->cryptlen, &ctx->dst, DMA_FROM_DEVICE);
	if (rc < 0) {
		pr_err("%s spacc_sg_to_ddt for src failed, src:%p, cryptlen:%d\n",
			__func__, req->dst, req->cryptlen);
		return rc;
	}
	ctx->dst_nents = rc;

	return 0;
}

static void spacc_cipher_cleanup_dma(struct device *dev, struct skcipher_request *req)
{
	struct spacc_crypto_reqctx *ctx = skcipher_request_ctx(req);

	if (!ctx->partial_processing) {
		dma_unmap_sg(dev, req->src, ctx->src_nents, DMA_TO_DEVICE);
	}
	pdu_ddt_free(&ctx->src);

	dma_unmap_sg(dev, req->dst, ctx->dst_nents, DMA_FROM_DEVICE);
	pdu_ddt_free(&ctx->dst);
}

static void spacc_cipher_cb(void *spacc, void *data)
{
	struct skcipher_request *req = data;
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct spacc_crypto_reqctx *ctx = skcipher_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);

	int err;

	if (!ctx->partial_processing) {
		req->src = ctx->reqsrc;
		req->dst = ctx->reqdst;
		req->cryptlen = ctx->req_cryptlen;
	}

	spacc_cipher_cleanup_dma(tctx->dev, req);

	err = pdu_error_code(priv->spacc.job[ctx->new_handle].job_err);
	spacc_close(&priv->spacc, ctx->new_handle);

	if (ctx->partial_processing) {
		complete(&ctx->partial_completion);
	}
	else {
		req->base.complete(&req->base, err);
	}
}

static int spacc_ciper_prepare(struct skcipher_request *req, int enc)
{
	struct crypto_skcipher *reqtfm = crypto_skcipher_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_skcipher_ctx(reqtfm);
	struct spacc_crypto_reqctx *ctx = skcipher_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int rc;


	/* clone new handle is actually allocate a new job id */
	ctx->new_handle = spacc_clone_handle(&priv->spacc, tctx->handle, req);
	if (ctx->new_handle < 0) {
		spacc_cipher_cleanup_dma(tctx->dev, req);
		pr_err("%s: failed to clone handle.\n", __func__);
		return -EBUSY;
	}

	rc = spacc_set_operation(&priv->spacc, ctx->new_handle, enc ? OP_ENCRYPT : OP_DECRYPT, 0, 0, 0, 0, 0);
	if (rc < 0) {
		spacc_close(&priv->spacc, tctx->handle);
		tctx->handle = -1;
		put_device(tctx->dev);
		return rc;
	}

	/* if we are decrypting set the expand bit (required for RC4/AES) */
	if (!enc)
		spacc_set_key_exp (&priv->spacc, ctx->new_handle);

	rc = spacc_write_context(&priv->spacc, ctx->new_handle, SPACC_CRYPTO_OPERATION,
			tctx->key, tctx->keylen, req->iv, crypto_skcipher_ivsize(reqtfm));
	if (rc < 0) {
		dev_warn(tctx->dev, "failed to write SPAcc context %d: %s\n",
								  tctx->handle, spacc_error_msg(rc));
	  return -EINVAL;
	}

	return 0;
}

static int __spacc_cipher_process(struct skcipher_request *req, int enc)
{
	struct crypto_skcipher *reqtfm = crypto_skcipher_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_skcipher_ctx(reqtfm);
	struct spacc_crypto_reqctx *ctx = skcipher_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	int rc;

	spacc_ciper_prepare(req, enc);

	rc = spacc_cipher_init_dma(tctx->dev, req);
	if (rc < 0) {
		pr_err("%s: failed to init ddt.\n", __func__);
		return -ENOMEM;
	}

	rc = spacc_packet_enqueue_ddt(&priv->spacc, ctx->new_handle,
					&ctx->src, &ctx->dst, req->cryptlen, 0, 0, 0, 0, 0);

	if (rc < 0) {

		spacc_cipher_cleanup_dma(tctx->dev, req);
		spacc_close(&priv->spacc, ctx->new_handle);

		pr_err("%s: failed to enqueue ddt.\n", __func__);
		if (rc != CRYPTO_FIFO_FULL) {
			dev_err(tctx->dev, "failed to enqueue job: %s\n", spacc_error_msg(rc));
		} else if (!(req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG))
			return -EBUSY;

		return -EBUSY;
	}

	return -EINPROGRESS;

}

static int spacc_cipher_process(struct skcipher_request *req, int enc)
{
	struct crypto_skcipher *reqtfm = crypto_skcipher_reqtfm(req);
	struct spacc_crypto_ctx *tctx = crypto_skcipher_ctx(reqtfm);
	struct spacc_crypto_reqctx *ctx = skcipher_request_ctx(req);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	uint32_t chunk_size = (priv->max_msg_len & ~0xfff);
	int total = req->cryptlen;
	struct scatterlist *sg_cur;
	struct scatterlist *sg_src_last;
	struct scatterlist *sg_dst_last;
	int len;
	int rc;

	ctx->partial_processing = false;

	ctx->reqsrc = req->src;
	ctx->reqdst = req->dst;
	ctx->req_cryptlen = req->cryptlen;

	if (tctx->handle < 0 || !tctx->ctx_valid) {
		pr_err("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	if (req->cryptlen <= priv->max_msg_len) {
		rc = __spacc_cipher_process(req, enc);
		return rc;
	}

	/* need to seperate req->cryptlen into equal length chunks */
	chunk_size = req->cryptlen;
	while (chunk_size > priv->max_msg_len) {
		chunk_size = req->cryptlen >> 1;
	}

	init_completion(&ctx->partial_completion);

	while(total > chunk_size) {
		ctx->partial_processing = true;
		req->cryptlen = chunk_size;
		sg_cur = req->src;
		rc = __spacc_cipher_process(req, enc);
		req->src = sg_cur;

		if (rc != -EINPROGRESS) {
			pr_err("%s: partial processing failed %d", __func__, rc);
			return rc;
		}

		sg_src_last = req->src;
		sg_dst_last = req->dst;
		len = 0;
		while (len < chunk_size) {
			if ((len + sg_src_last->length) > chunk_size)
				 break;
			len += sg_src_last->length;
			sg_src_last = sg_next(sg_src_last);
			sg_dst_last = sg_next(sg_dst_last);
		}

		wait_for_completion(&ctx->partial_completion);
		reinit_completion(&ctx->partial_completion);

		req->src = sg_src_last;
		req->dst = sg_dst_last;
		total -= chunk_size;
	}

	ctx->partial_processing = false;

	req->cryptlen = total;
	rc = __spacc_cipher_process(req, enc);

	if (rc != -EINPROGRESS) {
		pr_err("%s: last cipher processing failed %d", __func__, rc);
		return rc;
	}


	return rc;
}

static int spacc_cipher_encrypt(struct skcipher_request *req)
{
	return spacc_cipher_process(req, 1);
}

static int spacc_cipher_decrypt(struct skcipher_request *req)
{
	return spacc_cipher_process(req, 0);
}

static int spacc_cipher_setkey(struct crypto_skcipher *tfm, const u8 *key, unsigned keylen)
{
	const struct spacc_alg *salg = spacc_tfm_alg(&tfm->base);
	struct spacc_crypto_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
	char *week_key	= "\x01\x01\x01\x01\x01\x01\x01\x01";
	int x, rc;

	dev_dbg(salg->dev[0], "%s, keylen:%d\n", __func__, keylen);
	tctx->ctx_valid = false;

	/* close handle since key size may have changed */
	if (tctx->handle >= 0) {
		spacc_close(&priv->spacc, tctx->handle);
		put_device(tctx->dev);
		tctx->handle = -1;
		tctx->dev	 = NULL;
	}

	priv = NULL;
	for (x = 0; x < ELP_CAPI_MAX_DEV && salg->dev[x]; x++) {
		priv = dev_get_drvdata(salg->dev[x]);
		tctx->dev = get_device(salg->dev[x]);
		if (spacc_isenabled(&priv->spacc, salg->mode->id, keylen))
					tctx->handle = spacc_open(&priv->spacc,
					salg->mode->id, CRYPTO_MODE_NULL, -1, 0, spacc_cipher_cb, tfm);
		if (tctx->handle >= 0) { break; }
		put_device(salg->dev[x]);
	}


	if (salg->mode->id == CRYPTO_MODE_DES_ECB) {
		if (!memcmp(key, week_key, keylen)) {
			pr_debug("return failed if got week key for ecb(des)\n");
			return -EINVAL;
		}
	}

	if (tctx->handle < 0) {
		dev_err(salg->dev[0], "failed to open SPAcc context\n");
		return -EEXIST;
	}

	rc = spacc_write_context(&priv->spacc, tctx->handle,
			SPACC_CRYPTO_OPERATION, key, keylen, NULL, 0);
	if (rc < 0) {
		dev_warn(tctx->dev, "failed to write SPAcc context %d: %s\n",
								  tctx->handle, spacc_error_msg(rc));

		/* Non-fatal; we continue with the software fallback. */
		return -EINVAL;
	}

	memcpy(tctx->key, key, keylen);
	tctx->keylen = keylen;

	tctx->ctx_valid = true;

	return 0;
}

static int spacc_cipher_cra_init(struct crypto_tfm *tfm)
{
	const struct spacc_alg *salg = spacc_tfm_alg(tfm);
	struct spacc_crypto_ctx *tctx = crypto_tfm_ctx(tfm);
	unsigned long reqsize;
	unsigned long align;

	tctx->handle    = -1;
	tctx->ctx_valid = false;
	tctx->dev       = get_device(salg->dev[0]);

	tctx->fb.cipher = __crypto_skcipher_cast(tfm);

	align = crypto_skcipher_alignmask(tctx->fb.cipher);
	align &= ~(crypto_tfm_ctx_alignment() - 1);
	reqsize = align + sizeof(struct spacc_crypto_reqctx)
						 + crypto_skcipher_reqsize(tctx->fb.cipher);

	tctx->fb.cipher->reqsize = reqsize;

	return 0;
}

static void spacc_cipher_cra_exit(struct crypto_tfm *tfm)
{
	struct spacc_crypto_ctx *tctx = crypto_tfm_ctx(tfm);
	struct spacc_priv *priv = dev_get_drvdata(tctx->dev);

	dev_dbg(tctx->dev, "%s\n", __func__);

	if (tctx->handle >= 0)
		spacc_close(&priv->spacc, tctx->handle);

	put_device(tctx->dev);
}

const struct skcipher_alg spacc_cipher_template = {
	.min_keysize = 8,
	.max_keysize = 32,
	.setkey  = spacc_cipher_setkey,
	.encrypt = spacc_cipher_encrypt,
	.decrypt = spacc_cipher_decrypt,

	.base = {
		.cra_init = spacc_cipher_cra_init,
		.cra_exit = spacc_cipher_cra_exit,
		.cra_priority = 350,
		.cra_module	= THIS_MODULE,
		.cra_ctxsize  = sizeof (struct spacc_crypto_ctx),
		.cra_alignmask = 15,
		.cra_flags	 = CRYPTO_ALG_TYPE_SKCIPHER
						  | CRYPTO_ALG_ASYNC
						  | CRYPTO_ALG_KERN_DRIVER_ONLY,
	},
};

int __init spacc_cipher_module_init(void)
{
	return 0;
}

void spacc_cipher_module_exit(void)
{
}

module_param (spacc_epn, int, 0);
MODULE_PARM_DESC (spacc_epn, "Set SPAcc EPN number to use");