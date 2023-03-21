// SPDX-License-Identifier: GPL-2.0

#include <drm/drm_drv.h>
#include <drm/drm_kunit_helpers.h>
#include <drm/drm_managed.h>

#include <kunit/resource.h>

#include <linux/device.h>
#include <linux/platform_device.h>

static const struct drm_mode_config_funcs drm_mode_config_funcs = {
};

struct drm_device *
__drm_kunit_helper_alloc_drm_device_with_driver(struct kunit *test,
						struct device *dev,
						size_t size, size_t offset,
						const struct drm_driver *driver)
{
	struct drm_device *drm;
	void *container;
	int ret;

	container = __devm_drm_dev_alloc(dev, driver, size, offset);
	if (IS_ERR(container))
		return ERR_CAST(container);

	drm = container + offset;
	drm->mode_config.funcs = &drm_mode_config_funcs;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ERR_PTR(ret);

	return drm;
}
EXPORT_SYMBOL_GPL(__drm_kunit_helper_alloc_drm_device_with_driver);

MODULE_AUTHOR("Maxime Ripard <maxime@cerno.tech>");
MODULE_LICENSE("GPL");
