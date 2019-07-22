#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include "avb.h"

MODULE_AUTHOR("Jose Abreu <joabreu@synopsys.com>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("TSN DRM shim driver");



static void avb_drm_fb_output_poll_changed(struct drm_device *drm)
{
	struct avb_drm_private *avb_dev = drm->dev_private;

	drm_fbdev_cma_hotplug_event(avb_dev->fbdev_cma);
}




static void avb_drm_lastclose(struct drm_device *drm)
{
	struct avb_drm_private *avb_dev = drm->dev_private;

	drm_fbdev_cma_restore_mode(avb_dev->fbdev_cma);
}




static const struct drm_mode_config_funcs avb_drm_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = avb_drm_fb_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};


static void avb_drm_setup_mode_config(struct drm_device *drm)
{
	drm_mode_config_init(drm);
	drm->mode_config.min_width = 0;
	drm->mode_config.min_height = 0;
	drm->mode_config.max_width = XRES_MAX;
	drm->mode_config.max_height = YRES_MAX;
	drm->mode_config.preferred_depth = DEPTH_DEF;
	drm->mode_config.funcs = &avb_drm_mode_config_funcs;
}


static void avb_drm_cleanup_mode_config(struct drm_device *drm)
{
	drm_mode_config_cleanup(drm);
}



static int avb_drm_register(struct drm_device *drm)
{
	struct avb_drm_private *avb_dev = drm->dev_private;
	int bpp = drm->mode_config.preferred_depth;
	struct drm_fbdev_cma *fbdev;
	int ret;

	DRM_DEBUG_KMS("\n");

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);
	drm_kms_helper_poll_init(drm);

	fbdev = drm_fbdev_cma_init(drm, bpp, drm->mode_config.num_connector);
	if (IS_ERR(fbdev)) {
		DRM_ERROR("failed to init fbdev (%ld)\n", PTR_ERR(fbdev));
		return PTR_ERR(fbdev);
	} else {
		avb_dev->fbdev_cma = fbdev;
	}

	return 0;
}

static void avb_drm_unregister(struct drm_device *drm)
{
	struct avb_drm_private *avb_dev = drm->dev_private;
	struct drm_fbdev_cma *fbdev = avb_dev->fbdev_cma;

	DRM_DEBUG_KMS("\n");

	drm_atomic_helper_shutdown(drm);
	avb_dev->fbdev_cma = NULL;
	drm_dev_unregister(drm);
	if (fbdev)
		drm_fbdev_cma_fini(fbdev);
}



static const struct file_operations avb_drm_ops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.compat_ioctl = drm_compat_ioctl,
	.poll = drm_poll,
	.read = drm_read,
	.llseek = no_llseek,
	.mmap = drm_gem_cma_mmap,
};


static struct drm_driver avb_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.lastclose = avb_drm_lastclose,
	.name = "avb",
	.desc = "TSN DRM Shim Driver",
	.date = "20170706",
	.major = 1,
	.minor = 0,
	.patchlevel = 0,
	.fops = &avb_drm_ops,
	.dumb_create = drm_gem_cma_dumb_create,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,
};


static int avb_drm_probe(struct platform_device *pdev)
{
	struct avb_drm_private *avb_dev;
	struct drm_device *drm;
	int ret;

	DRM_DEBUG_KMS("\n");

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64))) {
		if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))) {
			DRM_ERROR("failed to setup DMA mask\n");
			return -ENODEV;
		}
	}

	drm = drm_dev_alloc(&avb_drm_driver, &pdev->dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	avb_dev = devm_kzalloc(&pdev->dev, sizeof(*avb_dev), GFP_KERNEL);
	if (!avb_dev) {
		ret = -ENOMEM;
		goto err_unref;
	}

	drm->dev_private = avb_dev;
	avb_drm_setup_mode_config(drm);
	platform_set_drvdata(pdev, drm);

	/* Start crtc + plane */
	ret = avb_drm_setup_crtc(drm);
	if (ret) {
		DRM_ERROR("failed to setup crtc/plane\n");
		goto err_cleanup_mode;
	}

	/* Start virtual connector + encoder chain */
	ret = avb_drm_sim_init(drm);
	if (ret) {
		DRM_ERROR("failed to setup simulated connector/encoder\n");
		goto err_cleanup_mode;
	}

	ret = avb_drm_register(drm);
	if (ret) {
		DRM_ERROR("failed to register\n");
		goto err_cleanup_mode;
	}

	DRM_DEBUG_KMS("driver probed\n");
	return 0;

err_cleanup_mode:
	avb_drm_cleanup_mode_config(drm);
err_unref:
	drm_dev_unref(drm);
	return ret;
}



static int avb_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	DRM_DEBUG_KMS("\n");

	avb_drm_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	avb_drm_cleanup_mode_config(drm);
	drm->dev_private = NULL;
	drm_dev_unref(drm);

	DRM_DEBUG_KMS("driver removed\n");
	return 0;
}



static struct platform_driver avb_drm_platform_driver = {
	.probe = avb_drm_probe,
	.remove = avb_drm_remove,
	.driver = {
		.name = "avb-drm",
	},
};

static size_t avb_drm_shim_buffer_refill(struct tsn_link *link)
{
	return 0;
}


static size_t avb_drm_shim_buffer_drain(struct tsn_link *link)
{
	/* Invalid op for this shim */
	return 0;
}

static size_t avb_drm_shim_hdr_size(struct tsn_link *link)
{
	return avb_hdr_size();
}



static size_t avb_drm_shim_copy_size(struct tsn_link *link)
{
	return XRES_DEF * YRES_DEF * DEPTH_DEF / 8;
}

static void avb_drm_shim_copy_done(struct tsn_link *link)
{
	/* */
}

static void avb_drm_shim_assemble_header(struct tsn_link *link,
		struct avtpdu_header *header, size_t bytes)
{
	avb_assemble_header(header, bytes);
}



static int avb_drm_shim_validate_header(struct tsn_link *link,
		struct avtpdu_header *header)
{
	return avb_validate_header(header);
}

static void *avb_drm_shim_get_payload_data(struct tsn_link *link,
		struct avtpdu_header *header)
{
	return avb_get_payload_data(header);
}




static int avb_drm_shim_open(struct tsn_link *link)
{
	struct platform_device_info avb_pinfo = {
		.parent = NULL,
		.name = "avb-drm",
		.id = 0,
		.res = NULL,
		.num_res = 0,
		.data = NULL,
		.size_data = 0,
		.dma_mask = DMA_BIT_MASK(64),
	};
	struct avb_drm_private *avb_dev;
	struct platform_device *pdev;
	struct drm_device *drm;
	int ret;

	DRM_DEBUG_KMS("\n");

	ret = platform_driver_register(&avb_drm_platform_driver);
	if (ret) {
		DRM_ERROR("failed to register platform driver\n");
		return ret;
	}

	pdev = platform_device_register_full(&avb_pinfo);
	if (IS_ERR(pdev)) {
		DRM_ERROR("failed to register platform device\n");
		ret = PTR_ERR(pdev);
		goto fail_pdev;
	}

	drm = platform_get_drvdata(pdev);
	if (!drm || !drm->dev_private) {
		DRM_ERROR("failed to get drvdata\n");
		ret = -ENODEV;
		goto fail_drvdata;
	}

	avb_dev = drm->dev_private;
	avb_dev->pdev = pdev;
	avb_dev->link = link;
	link->media_chip = avb_dev;

	DRM_DEBUG_KMS("shim opened");
	return 0;

fail_drvdata:
	platform_device_unregister(pdev);
fail_pdev:
	platform_driver_unregister(&avb_drm_platform_driver);
	return ret;
}




static int avb_drm_shim_close(struct tsn_link *link)
{
	struct avb_drm_private *avb_dev = link->media_chip;

	if (!avb_dev)
		return 0;

	DRM_DEBUG_KMS("\n");

	platform_device_unregister(avb_dev->pdev);
	platform_driver_unregister(&avb_drm_platform_driver);
	link->media_chip = NULL;

	DRM_DEBUG_KMS("shim closed\n");
	return 0;
}



static struct tsn_shim_ops avb_drm_shim_ops = {
	.shim_name = "drm",
	.probe = avb_drm_shim_open,
	.buffer_refill = avb_drm_shim_buffer_refill,
	.buffer_drain = avb_drm_shim_buffer_drain,
	.media_close = avb_drm_shim_close,
	.hdr_size = avb_drm_shim_hdr_size,
	.copy_size = avb_drm_shim_copy_size,
	.copy_done = avb_drm_shim_copy_done,
	.assemble_header = avb_drm_shim_assemble_header,
	.validate_header = avb_drm_shim_validate_header,
	.get_payload_data = avb_drm_shim_get_payload_data,
};
module_tsn_driver(avb_drm_shim_ops);
