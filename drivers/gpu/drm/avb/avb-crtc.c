/*
 * AVB  DRM driver.
 *
 * Copyright (C) 2020
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include "avb.h"

#define to_avb_drm(x) container_of(x, struct avb_drm_private, crtc)




static unsigned int avb_drm_get_bpp(const struct drm_format_info *fmtinfo)
{
	switch (fmtinfo->format) {
	case DRM_FORMAT_XRGB8888:
		return 4;
	default:
		DRM_ERROR("unsupported format (0x%x)\n", fmtinfo->format);
		return 0;
	}
}


static const struct drm_crtc_funcs avb_drm_crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};


static void avb_drm_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	const struct drm_framebuffer *fb = crtc->primary->state->fb;
	struct avb_drm_private *avb_dev = to_avb_drm(crtc);
	unsigned int bpp, width, height;
	size_t total_len;
	int ret;

	bpp = avb_drm_get_bpp(fb->format);
	width = mode->hdisplay;
	height = mode->vdisplay;
	total_len = width * height * bpp;

	if (!total_len)
		return;

	ret = tsn_set_buffer_size(avb_dev->link, total_len);
	if (ret) {
		DRM_ERROR("failed to set TSN buffer size\n");
		return;
	}

	DRM_DEBUG_KMS("hdisplay=%d, vdisplay=%d\n",
			mode->crtc_hdisplay, mode->crtc_vdisplay);
}


static void avb_drm_crtc_atomic_begin(struct drm_crtc *crtc,
		struct drm_crtc_state *old_state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;
	unsigned long flags;

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	}
}



static void avb_drm_crtc_atomic_enable(struct drm_crtc *crtc,
		struct drm_crtc_state *old_state)
{
	struct avb_drm_private *avb_dev = to_avb_drm(crtc);

	DRM_DEBUG_KMS("\n");
	tsn_lb_enable(avb_dev->link);
}



static void avb_drm_crtc_atomic_disable(struct drm_crtc *crtc)
{
	struct avb_drm_private *avb_dev = to_avb_drm(crtc);

	DRM_DEBUG_KMS("\n");
	tsn_lb_disable(avb_dev->link);
}


static const struct drm_crtc_helper_funcs avb_drm_crtc_helper_funcs = {
	.mode_set = drm_helper_crtc_mode_set,
	.mode_set_base = drm_helper_crtc_mode_set_base,
	.mode_set_nofb = avb_drm_crtc_mode_set_nofb,
	.atomic_begin = avb_drm_crtc_atomic_begin,
	.atomic_enable = avb_drm_crtc_atomic_enable,
	.disable = avb_drm_crtc_atomic_disable,
};


static void avb_drm_plane_atomic_update(struct drm_plane *plane,
		struct drm_plane_state *old_state)
{
	struct drm_plane_state *new_state = plane->state;
	struct drm_gem_cma_object *cma_obj;
	struct avb_drm_private *avb_dev;
	unsigned int bpp, width, height;
	size_t total_len;
	void *data;
	int ret;

	if (!new_state || !new_state->crtc || !new_state->fb)
		return;

	avb_dev = to_avb_drm(new_state->crtc);
	cma_obj = drm_fb_cma_get_gem_obj(new_state->fb, 0);
	data = cma_obj->vaddr;

	bpp = avb_drm_get_bpp(new_state->fb->format);
	width = new_state->crtc_w;
	height = new_state->crtc_h;
	total_len = width * height * bpp;

	if (!total_len)
		return;

	DRM_DEBUG_DRIVER("%dx%d (bpp=%d), vaddr=0x%p, len=%d\n",
    width, height, bpp, data, (unsigned int)total_len);

	ret = tsn_buffer_write(avb_dev->link, data, total_len);
	if (ret != total_len) {
		DRM_ERROR("failed to write TSN buffer (%ld bytes): got %d\n",
				total_len, ret);
		return;
	}
}


static const struct drm_plane_helper_funcs avb_drm_plane_helper_funcs = {
	.atomic_update = avb_drm_plane_atomic_update,
};

static void avb_drm_plane_destroy(struct drm_plane *plane)
{
	drm_plane_helper_disable(plane);
	drm_plane_cleanup(plane);
}




static const struct drm_plane_funcs avb_drm_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = avb_drm_plane_destroy,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};


static struct drm_plane *avb_drm_plane_init(struct drm_device *drm)
{
	struct avb_drm_private *avb_dev = drm->dev_private;
	struct drm_plane *plane = NULL;
	u32 format = FOURCC_DEF;
	int ret;

	plane = devm_kzalloc(drm->dev, sizeof(*plane), GFP_KERNEL);
	if (!plane)
		return ERR_PTR(-ENOMEM);

	ret = drm_universal_plane_init(drm, plane, 0xff, &avb_drm_plane_funcs,
			&format, 1, NULL, DRM_PLANE_TYPE_PRIMARY, "avb-plane");
	if (ret)
		return ERR_PTR(ret);

	drm_plane_helper_add(plane, &avb_drm_plane_helper_funcs);
	avb_dev->plane = plane;

	return plane;
}


int avb_drm_setup_crtc(struct drm_device *drm)
{
	struct avb_drm_private *avb_dev = drm->dev_private;
	struct drm_plane *plane;
	int ret;

	plane = avb_drm_plane_init(drm);
	if (IS_ERR(plane))
		return PTR_ERR(plane);

	ret = drm_crtc_init_with_planes(drm, &avb_dev->crtc, plane, NULL,
			&avb_drm_crtc_funcs, "avb-crtc");
	if (ret) {
		avb_drm_plane_destroy(plane);
		return ret;
	}

	drm_crtc_helper_add(&avb_dev->crtc, &avb_drm_crtc_helper_funcs);
	return 0;
}
