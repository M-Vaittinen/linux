// SPDX-License-Identifier: GPL-2.0

#include <kunit/device.h>
#include <kunit/test.h>

#include <linux/device.h>

static void kunit_device_drop(struct kunit_resource *res)
{
	root_device_unregister(res->data);
}

struct device *kunit_device_register(struct kunit *test, const char *name)
{
	struct device *dev;

	dev = root_device_register(name);
	if (IS_ERR_OR_NULL(dev))
		return dev;

	return kunit_alloc_resource(test, NULL, kunit_device_drop, GFP_KERNEL,
				    dev);
}
EXPORT_SYMBOL_GPL(kunit_device_register);

static bool kunit_device_match(struct kunit *test, struct kunit_resource *res,
			       void *match_data)
{
	return res->data == match_data;
}

void kunit_device_unregister(struct kunit *test, struct device *dev)
{
	kunit_destroy_resource(test, kunit_device_match, dev);
}
EXPORT_SYMBOL_GPL(kunit_device_unregister);
