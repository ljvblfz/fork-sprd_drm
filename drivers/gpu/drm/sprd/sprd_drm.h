/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2014-2016 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __KIRIN_DRM_DRV_H__
#define __KIRIN_DRM_DRV_H__

#include <drm/drmP.h>
#include <linux/sprd_ion.h>
//#include <linux/hisi/hisi_ion.h>
//#include <linux/hisi/hisi-iommu.h>

#include "drm_crtc.h"
#include "drm_fb_helper.h"

#define MAX_CRTC	2

#define to_sprd_fbdev(x) container_of(x, struct sprd_fbdev, fb_helper)

/* display controller init/cleanup ops */
struct sprd_dc_ops {
	int (*init)(struct drm_device *dev);
	void (*cleanup)(struct drm_device *dev);
};

struct sprd_drm_private {
	struct drm_fb_helper *fb_helper;
	struct drm_fb_helper *fbdev;
	struct drm_crtc *crtc[MAX_CRTC];
};

struct sprd_fbdev {
	struct drm_fb_helper fb_helper;
	struct drm_framebuffer *fb;

	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
//	struct iommu_map_format iommu_format;
	void *screen_base;
	unsigned long smem_start;
	unsigned long screen_size;
	int shared_fd;
};

extern const struct sprd_dc_ops dss_dc_ops;
extern void dsi_set_output_client(struct drm_device *dev);

struct drm_framebuffer *sprd_framebuffer_init(struct drm_device *dev,
		struct drm_mode_fb_cmd2 *mode_cmd);
struct drm_fb_helper *sprd_drm_fbdev_init(struct drm_device *dev);
void sprd_drm_fbdev_fini(struct drm_device *dev);


#endif /* __KIRIN_DRM_DRV_H__ */
