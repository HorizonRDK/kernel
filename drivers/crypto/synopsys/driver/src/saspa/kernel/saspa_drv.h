#ifndef SASPA_DRV_H_
#define SASPA_DRV_H_

#include <linux/platform_device.h>

extern struct platform_driver saspa_aes_driver;
extern struct platform_driver saspa_des_driver;
extern struct platform_driver saspa_hash_driver;

long saspa_alloc_ctx(struct device *dev, size_t size);
long saspa_alloc_data(struct device *dev, size_t size);
void saspa_free(struct device *dev, long handle, size_t size);

int saspa_mem_write(struct device *dev, long handle, const void *src, size_t size);
int saspa_mem_read(struct device *dev, void *dst, long handle, size_t size);
int saspa_mem_addr(struct device *dev, long handle);
long saspa_mem_size(struct device *dev);

int saspa_prepare_job(struct device *dev, long ctx_handle, long msg_handle);
int saspa_wait(struct device *dev);

void saspa_enable_irq(struct device *dev, u32 mask);
void saspa_disable_irq(struct device *dev, u32 mask);

struct saspa_aes_ctx;
int saspa_aes_run(struct device *dev, int encrypt, int mode, int keysize,
                  struct saspa_aes_ctx *ctx, void *_msg, size_t msglen, size_t aadlen, uint32_t *stat);

struct saspa_des_ctx;
int saspa_des_run(struct device *dev, int encrypt, int mode, int keysize,
                  struct saspa_des_ctx *ctx, void *_msg, size_t msglen);

struct saspa_hash_ctx;
int saspa_hash_run(struct device *dev, int mode, int sslmac, int keysize,
                  const struct saspa_hash_ctx *ctx, 
                  const void *_msg, size_t msglen, void *_out, size_t outlen);


#endif
