#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include "avb.h"

static const struct drm_display_mode avb_drm_modes[] = {
	{ DRM_MODE("640x480", DRM_MODE_TYPE_DRIVER, 25175, 640, 656,
		   752, 800, 0, 480, 490, 492, 525, 0,
		   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
	  .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
};


static int avb_drm_connector_get_modes(struct drm_connector *connector)
{
	int count;

	for (count = 0; count < ARRAY_SIZE(avb_drm_modes); count++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &avb_drm_modes[count]);
		if (mode)
			drm_mode_probed_add(connector, mode);
	}

	drm_set_preferred_mode(connector, XRES_DEF, YRES_DEF);
	return count;
}



static void avb_drm_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_helper_funcs avb_drm_connector_helper_funcs = {
	.get_modes = avb_drm_connector_get_modes,
};



static const struct drm_connector_funcs avb_drm_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = avb_drm_connector_destroy,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};


static const struct drm_encoder_funcs avb_drm_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};


int avb_drm_sim_init(struct drm_device *drm)
{
	struct drm_encoder_slave *encoder;
	struct drm_connector *connector;
	int ret;

	encoder = devm_kzalloc(drm->dev, sizeof(*encoder), GFP_KERNEL);
	if (!encoder)
		return -ENOMEM;

	encoder->base.possible_crtcs = 1;
	encoder->base.possible_clones = 0;

	ret = drm_encoder_init(drm, &encoder->base, &avb_drm_encoder_funcs,
			DRM_MODE_ENCODER_VIRTUAL, "avb-encoder");
	if (ret)
		return ret;

	connector = devm_kzalloc(drm->dev, sizeof(*connector), GFP_KERNEL);
	if (!connector) {
		ret = -ENOMEM;
		goto err_encoder;
	}

	drm_connector_helper_add(connector, &avb_drm_connector_helper_funcs);

	ret = drm_connector_init(drm, connector, &avb_drm_connector_funcs,
			DRM_MODE_CONNECTOR_VIRTUAL);
	if (ret)
		goto err_encoder;

	ret = drm_mode_connector_attach_encoder(connector, &encoder->base);
	if (ret) {
		DRM_ERROR("failed to attach encoder to connector\n");
		goto err_connector;
	}

	drm_connector_register(connector);
	return 0;

err_connector:
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
err_encoder:
	drm_encoder_cleanup(&encoder->base);
	return ret;
}
