#include <linux/avb.h>
#include "avb.h"

MODULE_AUTHOR("Jose Abreu <joabreu@synopsys.com>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("TSN V4L2 shim driver");


static int avb_v4l2_probe(struct platform_device *pdev)
{
	struct avb_v4l2_private *avb_dev;
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	/*if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64))) {
		if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32))) {
			dev_err(dev, "failed to setup DMA mask\n");
			return -ENODEV;
		}
	}*/

	avb_dev = devm_kzalloc(dev, sizeof(*avb_dev), GFP_KERNEL);
	if (!avb_dev)
		return -ENOMEM;

	avb_dev->pdev = pdev;
	v4l2_dev = &avb_dev->v4l2_dev;
	strlcpy(v4l2_dev->name, dev_name(dev), sizeof(v4l2_dev->name));

	ret = v4l2_device_register(dev, v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register v4l2 device\n");
		return ret;
	}

	ret = avb_v4l2_video_init(avb_dev);
	if (ret) {
		dev_err(dev, "failed to initialize video pipeline\n");
		goto err_v4l2;
	}

	platform_set_drvdata(pdev, avb_dev);
	dev_info(dev, "driver probed\n");
	return 0;

err_v4l2:
	v4l2_device_unregister(v4l2_dev);
	return ret;
}

static int avb_v4l2_remove(struct platform_device *pdev)
{
	struct avb_v4l2_private *avb_dev = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	avb_v4l2_video_exit(avb_dev);
	v4l2_device_unregister(&avb_dev->v4l2_dev);
	dev_info(dev, "driver removed\n");
	return 0;
}

static struct platform_driver avb_v4l2_platform_driver = {
	.probe = avb_v4l2_probe,
	.remove = avb_v4l2_remove,
	.driver = {
		.name = "avb-v4l2",
	},
};



static size_t avb_v4l2_shim_buffer_refill(struct tsn_link *link)
{
	struct avb_v4l2_private *avb_dev = link->media_chip;
	struct device *dev = &avb_dev->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);
	return 0;
}



static size_t avb_v4l2_shim_buffer_drain(struct tsn_link *link)
{
	struct avb_v4l2_private *avb_dev = link->media_chip;
	struct device *dev = &avb_dev->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	return avb_v4l2_buffer_done(avb_dev);
}

static size_t avb_v4l2_shim_hdr_size(struct tsn_link *link)
{
	return avb_hdr_size();
}

static size_t avb_v4l2_shim_copy_size(struct tsn_link *link)
{
	struct avb_v4l2_private *avb_dev = link->media_chip;

	return avb_dev->format.sizeimage;
}




static void avb_v4l2_shim_assemble_header(struct tsn_link *link,
		struct avtpdu_header *header, size_t bytes)
{
	avb_assemble_header(header, bytes);
}

static int avb_v4l2_shim_validate_header(struct tsn_link *link,
		struct avtpdu_header *header)
{
	return avb_validate_header(header);
}



static void *avb_v4l2_shim_get_payload_data(struct tsn_link *link,
		struct avtpdu_header *header)
{
	return avb_get_payload_data(header);
}


static int avb_v4l2_shim_open(struct tsn_link *link)
{
	struct platform_device_info avb_pinfo = {
		.parent = NULL,
		.name = "avb-v4l2",
		.id = 0,
		.res = NULL,
		.num_res = 0,
		.data = NULL,
		.size_data = 0,
		.dma_mask = DMA_BIT_MASK(64),
	};
	struct avb_v4l2_private *avb_dev;
	struct platform_device *pdev;
	int ret;

	ret = platform_driver_register(&avb_v4l2_platform_driver);
	if (ret) {
		pr_err("%s: failed to register platform driver\n", __func__);
		return ret;
	}

	pdev = platform_device_register_full(&avb_pinfo);
	if (IS_ERR(pdev)) {
		pr_err("%s: failed to register platform device\n", __func__);
		ret = PTR_ERR(pdev);
		goto fail_pdev;
	}

	avb_dev = platform_get_drvdata(pdev);
	if (!avb_dev) {
		pr_err("%s: failed to get drvdata\n", __func__);
		ret = -ENODEV;
		goto fail_drvdata;
	}

	avb_dev->link = link;
	link->media_chip = avb_dev;

	pr_info("%s: shim opened\n", __func__);
	return 0;

fail_drvdata:
	platform_device_unregister(pdev);
fail_pdev:
	platform_driver_unregister(&avb_v4l2_platform_driver);
	return ret;
}



static int avb_v4l2_shim_close(struct tsn_link *link)
{
	struct avb_v4l2_private *avb_dev = link->media_chip;

	if (!avb_dev)
		return 0;

	platform_device_unregister(avb_dev->pdev);
	platform_driver_unregister(&avb_v4l2_platform_driver);
	link->media_chip = NULL;

	pr_info("%s: shim closed\n", __func__);
	return 0;
}



static struct tsn_shim_ops avb_v4l2_shim_ops = {
	.shim_name = "v4l2",
	.probe = avb_v4l2_shim_open,
	.buffer_refill = avb_v4l2_shim_buffer_refill,
	.buffer_drain = avb_v4l2_shim_buffer_drain,
	.media_close = avb_v4l2_shim_close,
	.hdr_size = avb_v4l2_shim_hdr_size,
	.copy_size = avb_v4l2_shim_copy_size,
	.assemble_header = avb_v4l2_shim_assemble_header,
	.validate_header = avb_v4l2_shim_validate_header,
	.get_payload_data = avb_v4l2_shim_get_payload_data,
};
module_tsn_driver(avb_v4l2_shim_ops);
