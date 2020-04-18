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

#include <linux/init.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#define BUFSIZE  (4*1024)

typedef struct
{
	struct device *dev;
	struct dma_device *dma_dev;
	struct dma_chan *ch;
	struct dma_async_tx_descriptor *tx;
	struct completion completion;
	dma_addr_t src;
	dma_addr_t dst;
	char *src_virt;
	char *dst_virt;
	dma_cookie_t cookie;
}mem_dma_t;

static mem_dma_t *mem_dma;

static void mem_dma_dmacb(void *data)
{
	dev_info(mem_dma->dev, "transfer complete.\n");
	complete(&mem_dma->completion);
}

static int mem_dma_open(struct inode *inode, struct file *file)
{
	pr_info("%s enter.\n", __func__);

	return 0;
}

static ssize_t mem_dma_read(struct file *filp, char __user *buf,
                                 size_t size, loff_t *ppos)
{
    int ret = 0;
    unsigned long p = *ppos;
    unsigned int count = size;
	struct device *dev = mem_dma->dev;

    dev_info(dev, "%s enter.size=%d\n", __func__, (int)size);
    if (p >= BUFSIZE)
        return 0;
    if (count > BUFSIZE - p)
        count = BUFSIZE - p;
    if (copy_to_user(buf, mem_dma->dst_virt+p, count)) {
        ret = -EFAULT;
    } else {
        *ppos += count;
        ret = count;
    }

    return ret;
}

static ssize_t mem_dma_write(struct file *file, const char *data,
				size_t len, loff_t *ppos)
{
	struct dma_async_tx_descriptor *tx;
	struct dma_device *dma_dev = mem_dma->dma_dev;
	struct device *dev = mem_dma->dev;
	dma_cookie_t cookie;
	int ret;

	dev_info(dev, "%s enter.\n", __func__);

	reinit_completion(&mem_dma->completion);

	memset(mem_dma->dst_virt, 0x00, BUFSIZE);
	memset(mem_dma->src_virt, 0x00, BUFSIZE);
	ret = copy_from_user(mem_dma->src_virt, data, len);
	dev_info(dev, "from user: %s\n", mem_dma->src_virt);

	tx = dma_dev->device_prep_dma_memcpy(mem_dma->ch, mem_dma->dst, mem_dma->src, BUFSIZE, 0);
	if (!tx) {
		dev_err(dev, "%s: device_prep_dma_memcpy failed\n", __func__);
		ret = -EIO;
		goto err5;
	}

	tx->callback = mem_dma_dmacb;
	tx->callback_result = NULL;
	tx->callback_param = mem_dma;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(dev, "dma_submit_error %d\n", cookie);
		ret = -EIO;
		goto err5;
	}

	dma_async_issue_pending(mem_dma->ch);

	ret = wait_for_completion_timeout(&mem_dma->completion, msecs_to_jiffies(1000));
	if (!ret) {
		dev_err(dev, "%s: timeout !!\n", __func__);
		ret = -EIO;
		goto err5;
	}

	dev_info(dev, "we get: %s\n", mem_dma->dst_virt);
	return len;

err5:
	return ret;
}

static const struct file_operations mem_dma_fops = {
	.owner = THIS_MODULE,
	.open  = mem_dma_open,
	.read  = mem_dma_read,
	.write = mem_dma_write,
};

static struct miscdevice mem_dma_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "hobot_dmatest",
	.fops	= &mem_dma_fops,
};

static int mem_dma_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct dma_chan *ch;
	int ret = 0;
	dma_addr_t *src, *dst;
	void *src_virt, *dst_virt;
	dma_cap_mask_t mask;

	dev_info(dev, "%s enter.\n", __func__);

	if (!dev->of_node) {
		dev_err(dev, "no platform data.\n");
		return -EINVAL;
	}

	mem_dma = devm_kzalloc(dev, sizeof(mem_dma_t), GFP_KERNEL);
	if (!mem_dma) {
		dev_err(dev, "can not alloc memory for mem_dma\n");
		return -ENOMEM;
	}
	mem_dma->dev = dev;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	ch = dma_request_channel(mask, NULL, NULL);
	if (IS_ERR(ch)) {
		ret = PTR_ERR(ch);
		dev_err(dev, "%s: dma_request_channel failed: %d\n", __func__, ret);
		goto err1;
	}
	mem_dma->ch = ch;
	mem_dma->dma_dev = ch->device;

	src = &mem_dma->src;

	src_virt = dma_zalloc_coherent(dev, BUFSIZE, src, GFP_KERNEL);
	if (!src_virt) {
		dev_err(dev, "%s: alloc src failed.\n", __func__);
		ret = -ENOMEM;
		goto err2;
	}
	mem_dma->src_virt = src_virt;
	dev_info(dev, "vsrc:%p;psrc:0x%x\n", mem_dma->src_virt, (unsigned int)mem_dma->src);

	dst = &mem_dma->dst;
	dst_virt = dma_zalloc_coherent(dev, BUFSIZE, dst, GFP_KERNEL);
	if (!dst_virt) {
		dev_err(dev, "%s: alloc dst failed.\n", __func__);
		ret = -ENOMEM;
		goto err3;
	}
	mem_dma->dst_virt = dst_virt;
	dev_info(dev, "vdst:%p;pdst:0x%x\n", mem_dma->dst_virt, (unsigned int)mem_dma->dst);

	init_completion(&mem_dma->completion);
	ret = misc_register(&mem_dma_miscdev);
	if (ret < 0) {
		pr_err("failed to register mem_dma device\n");
		goto err4;;
	}

	return 0;

err4:
	dma_free_coherent(dev, BUFSIZE, dst_virt, *dst);
err3:
	dma_free_coherent(dev, BUFSIZE, src_virt, *src);
err2:
	dma_release_channel(ch);
err1:
	return ret;

}

static int mem_dma_remove(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;

	dev_info(dev, "%s enter.\n", __func__);

	dmaengine_terminate_async(mem_dma->ch);
	misc_deregister(&mem_dma_miscdev);
	dma_release_channel(mem_dma->ch);
	dma_free_coherent(dev, BUFSIZE, mem_dma->dst_virt, mem_dma->dst);
	dma_free_coherent(dev, BUFSIZE, mem_dma->src_virt, mem_dma->src);

	return 0;
}

static const struct of_device_id x2_dmatest_dt_ids[] = {
	{ .compatible = "hobot,x2-dma-test", },
	{},
};

MODULE_DEVICE_TABLE(of, x2_dmatest_dt_ids);

static struct platform_driver x2_dmatest_driver = {
	.probe  = mem_dma_probe,
	.remove = mem_dma_remove,
	.driver = {
		.name  = "hobot_dmatest",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(x2_dmatest_dt_ids),
	},
};

static int __init mem_dma_init(void)
{
	int ret;

	ret = platform_driver_register(&x2_dmatest_driver);
	if (ret)
		printk(KERN_ERR "x2_dmatest: probe failed: %d\n", ret);

	return ret;
}
module_init(mem_dma_init);

static void __exit mem_dma_exit(void)
{
	platform_driver_unregister(&x2_dmatest_driver);
}
module_exit(mem_dma_exit);

MODULE_LICENSE("GPL");
