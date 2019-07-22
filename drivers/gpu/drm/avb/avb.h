#ifndef __DRM_AVB_H__
#define __DRM_AVB_H__

#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <linux/avb.h>
#include <linux/platform_device.h>
#include <linux/tsn.h>

#define XRES_MAX		AVB_VIDEO_XRES
#define YRES_MAX		AVB_VIDEO_YRES
#define XRES_DEF		XRES_MAX
#define YRES_DEF		YRES_MAX
#define DEPTH_DEF		32
#define FOURCC_DEF		DRM_FORMAT_XRGB8888


struct avb_drm_private {
	struct drm_pending_vblank_event *event;
	struct drm_crtc *event_crtc;
	struct drm_fbdev_cma *fbdev_cma;
	struct platform_device *pdev;
	struct drm_plane *plane;
	struct tsn_link *link;
	struct drm_crtc crtc;
};


int avb_drm_sim_init(struct drm_device *drm);
void avb_drm_crtc_send_vblank_event(struct drm_crtc *crtc,
		struct drm_pending_vblank_event *event);
int avb_drm_setup_crtc(struct drm_device *drm);

#endif /* __DRM_AVB_H__ */
